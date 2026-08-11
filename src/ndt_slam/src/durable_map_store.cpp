#include "ndt_slam/durable_map_store.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <system_error>
#include <utility>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace ndt_slam {
namespace {

namespace fs = std::filesystem;

bool syncPath(const fs::path& path, bool directory, std::string* reason) {
#ifdef _WIN32
  if (directory) return true;
  const int descriptor = _wopen(path.c_str(), _O_RDONLY | _O_BINARY);
  if (descriptor < 0) {
    if (reason) *reason = "fsync_open_failed:" + path.string();
    return false;
  }
  const int result = _commit(descriptor);
  _close(descriptor);
#else
  const int descriptor =
      ::open(path.c_str(), O_RDONLY | (directory ? O_DIRECTORY : 0));
  if (descriptor < 0) {
    if (reason) *reason = "fsync_open_failed:" + path.string();
    return false;
  }
  const int result = ::fsync(descriptor);
  ::close(descriptor);
#endif
  if (result != 0) {
    if (reason) *reason = "fsync_failed:" + path.string();
    return false;
  }
  return true;
}

bool atomicReplace(const fs::path& source, const fs::path& target,
                   std::string* reason) {
#ifdef _WIN32
  if (!MoveFileExW(source.c_str(), target.c_str(),
                   MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
    if (reason) *reason = "pointer_rename_failed:" + target.string();
    return false;
  }
  return true;
#else
  if (::rename(source.c_str(), target.c_str()) != 0) {
    if (reason) *reason = "pointer_rename_failed:" + target.string();
    return false;
  }
  return true;
#endif
}

std::string trim(std::string value) {
  while (!value.empty() &&
         (value.back() == '\n' || value.back() == '\r' ||
          value.back() == ' ' || value.back() == '\t')) {
    value.pop_back();
  }
  const auto first = value.find_first_not_of(" \t\r\n");
  return first == std::string::npos ? std::string{} : value.substr(first);
}

bool validGenerationName(const std::string& value) {
  if (value.size() < 5U || value.rfind("gen_", 0U) != 0U) return false;
  return std::all_of(value.begin() + 4, value.end(), [](char character) {
    return character >= '0' && character <= '9';
  });
}

std::string readPointer(const fs::path& path) {
  std::ifstream input(path);
  if (!input.is_open()) return {};
  std::ostringstream contents;
  contents << input.rdbuf();
  return trim(contents.str());
}

bool writePointer(const fs::path& maps, const std::string& name,
                  const std::string& value, std::string* reason) {
  const fs::path temporary =
      maps / (name + ".tmp-" + MapSessionSnapshot::generateUuid());
  {
    std::ofstream output(temporary, std::ios::out | std::ios::trunc);
    output << value << '\n';
    output.flush();
    if (!output.good()) {
      if (reason) *reason = "pointer_write_failed:" + name;
      return false;
    }
  }
  if (!syncPath(temporary, false, reason) ||
      !atomicReplace(temporary, maps / name, reason) ||
      !syncPath(maps, true, reason)) {
    std::error_code ignored;
    fs::remove(temporary, ignored);
    return false;
  }
  return true;
}

std::string generationName(std::uint64_t generation) {
  std::ostringstream output;
  output << "gen_" << std::setw(12) << std::setfill('0') << generation;
  return output.str();
}

}  // namespace

DurableMapStore::DurableMapStore(std::string root_directory)
    : root_directory_(std::move(root_directory)) {}

bool DurableMapStore::initialize(std::string* reason) const {
  try {
    if (root_directory_.empty()) {
      if (reason) *reason = "map_store_root_empty";
      return false;
    }
    const fs::path root = fs::absolute(root_directory_);
    fs::create_directories(root / "generations");
    fs::create_directories(root / "staging");
    if (!syncPath(root / "generations", true, reason) ||
        !syncPath(root / "staging", true, reason) ||
        !syncPath(root, true, reason)) {
      return false;
    }
    if (reason) *reason = "initialized";
    return true;
  } catch (const std::exception& error) {
    if (reason) *reason = error.what();
    return false;
  }
}

bool DurableMapStore::save(MapSessionSaveRequest request,
                           std::string* reason) const {
  std::string init_reason;
  if (!initialize(&init_reason)) {
    if (reason) *reason = init_reason;
    return false;
  }
  const fs::path root = fs::absolute(root_directory_);
  const std::string generation = generationName(request.metadata.map_generation);
  const fs::path published = root / "generations" / generation;
  if (fs::exists(published)) {
    if (reason) *reason = "generation_already_exists";
    return false;
  }
  const fs::path staged = root / "staging" /
      (generation + ".stage-" + MapSessionSnapshot::generateUuid());
  request.target_directory = staged.string();
  std::string snapshot_reason;
  if (!MapSessionSnapshot::saveAtomic(request, &snapshot_reason)) {
    if (reason) *reason = "snapshot_save_failed:" + snapshot_reason;
    return false;
  }
  const auto verification = MapSessionSnapshot::loadVerified(staged.string());
  if (!verification.valid ||
      verification.metadata.map_generation != request.metadata.map_generation) {
    if (reason) *reason = "staged_generation_invalid:" + verification.reason;
    return false;
  }
  try {
    fs::rename(staged, published);
  } catch (const std::exception& error) {
    if (reason) *reason = std::string("generation_publish_failed:") + error.what();
    return false;
  }
  if (!syncPath(root / "generations", true, reason)) return false;

  const std::string old_current = readPointer(root / "CURRENT");
  const std::string old_previous = readPointer(root / "PREVIOUS");
  if (!old_previous.empty() &&
      !writePointer(root, "PREVIOUS_2", old_previous, reason)) {
    return false;
  }
  if (!old_current.empty() &&
      !writePointer(root, "PREVIOUS", old_current, reason)) {
    return false;
  }
  if (!writePointer(root, "CURRENT", generation, reason)) return false;
  if (reason) *reason = generation;
  return true;
}

DurableMapLoadResult DurableMapStore::loadBest() const {
  DurableMapLoadResult result;
  const fs::path root = fs::absolute(root_directory_);
  const std::array<const char*, 3> pointers = {
      "CURRENT", "PREVIOUS", "PREVIOUS_2"};
  bool any_pointer = false;
  std::string failures;
  for (const char* pointer : pointers) {
    const fs::path pointer_path = root / pointer;
    const bool pointer_exists = fs::exists(pointer_path);
    const std::string generation = readPointer(pointer_path);
    if (!pointer_exists) continue;
    any_pointer = true;
    if (generation.empty()) {
      failures += std::string(pointer) + ":pointer_empty_or_unreadable;";
      continue;
    }
    if (!validGenerationName(generation)) {
      failures += std::string(pointer) + ":pointer_invalid;";
      continue;
    }
    auto session = MapSessionSnapshot::loadVerified(
        (root / "generations" / generation).string());
    if (!session.valid) {
      failures += std::string(pointer) + ':' + session.reason + ';';
      continue;
    }
    result.status = DurableMapLoadStatus::LOADED;
    result.pointer = pointer;
    result.generation = generation;
    result.reason = "verified:" + result.pointer;
    result.session = std::move(session);
    return result;
  }
  result.status = any_pointer ? DurableMapLoadStatus::REFERENCE_CORRUPTED
                              : DurableMapLoadStatus::FIRST_BOOT;
  result.reason = any_pointer ? "all_generations_invalid:" + failures
                              : "no_map_pointer";
  return result;
}

}  // namespace ndt_slam
