#include "ndt_slam/recovery_checkpoint.hpp"

#include "ndt_slam/map_session_snapshot.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

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

std::string canonical(const RecoveryCheckpointData& value) {
  std::ostringstream output;
  output << value.schema_version << '\n' << value.map_uuid << '\n'
         << value.map_generation << '\n' << value.pose_generation << '\n'
         << value.continuity_generation << '\n' << std::setprecision(17)
         << value.source_stamp_sec << '\n' << value.x << '\n' << value.y << '\n'
         << value.z << '\n' << value.yaw << '\n' << value.source_git_sha << '\n';
  return output.str();
}

std::string checksum(const std::string& value) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char character : value) {
    hash ^= character;
    hash *= 1099511628211ULL;
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << hash;
  return output.str();
}

bool valid(const RecoveryCheckpointData& value) {
  return value.schema_version == RecoveryCheckpoint::kSchemaVersion &&
      !value.map_uuid.empty() && value.pose_generation > 0U &&
      value.continuity_generation > 0U &&
      std::isfinite(value.source_stamp_sec) && value.source_stamp_sec > 0.0 &&
      std::isfinite(value.x) && std::isfinite(value.y) &&
      std::isfinite(value.z) && std::isfinite(value.yaw);
}

bool syncPath(const fs::path& path, bool directory, std::string* reason) {
#ifdef _WIN32
  if (directory) return true;
  const int descriptor = _wopen(path.c_str(), _O_RDONLY | _O_BINARY);
  if (descriptor < 0) return false;
  const int result = _commit(descriptor);
  _close(descriptor);
#else
  const int descriptor =
      ::open(path.c_str(), O_RDONLY | (directory ? O_DIRECTORY : 0));
  if (descriptor < 0) return false;
  const int result = ::fsync(descriptor);
  ::close(descriptor);
#endif
  if (result != 0 && reason) *reason = "checkpoint_fsync_failed";
  return result == 0;
}

bool atomicReplace(const fs::path& source, const fs::path& target) {
#ifdef _WIN32
  return MoveFileExW(source.c_str(), target.c_str(),
                     MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH);
#else
  return ::rename(source.c_str(), target.c_str()) == 0;
#endif
}

}  // namespace

bool RecoveryCheckpoint::saveAtomic(
    const std::string& path, const RecoveryCheckpointData& checkpoint,
    std::string* reason) {
  if (path.empty() || !valid(checkpoint)) {
    if (reason) *reason = "checkpoint_invalid";
    return false;
  }
  try {
    const fs::path target = fs::absolute(path);
    fs::create_directories(target.parent_path());
    const fs::path temporary = target.parent_path() /
        (target.filename().string() + ".tmp-" +
         MapSessionSnapshot::generateUuid());
    RecoveryCheckpointData stored = checkpoint;
    stored.schema_version = kSchemaVersion;
    stored.checksum = checksum(canonical(stored));

    // Serialize double fields with fixed high precision so the round-trip
    // through YAML preserves the exact canonical checksum.  YAML-cpp's
    // default double formatting may truncate fractional digits and silently
    // break the integrity contract.
    auto doubleStr = [](double v) -> std::string {
      std::ostringstream out;
      out << std::setprecision(17) << v;
      return out.str();
    };

    YAML::Node document;
    document["schema"] = "ndt_slam_recovery_checkpoint";
    document["schema_version"] = stored.schema_version;
    document["map_uuid"] = stored.map_uuid;
    document["map_generation"] = stored.map_generation;
    document["pose_generation"] = stored.pose_generation;
    document["continuity_generation"] = stored.continuity_generation;
    document["source_stamp_sec"] = doubleStr(stored.source_stamp_sec);
    document["pose"]["x"] = doubleStr(stored.x);
    document["pose"]["y"] = doubleStr(stored.y);
    document["pose"]["z"] = doubleStr(stored.z);
    document["pose"]["yaw"] = doubleStr(stored.yaw);
    document["source_git_sha"] = stored.source_git_sha;
    document["checksum"] = stored.checksum;
    YAML::Emitter emitter;
    emitter << document;
    std::ofstream output(temporary, std::ios::out | std::ios::trunc);
    output << emitter.c_str();
    output.flush();
    output.close();
    if (!emitter.good() || !syncPath(temporary, false, reason) ||
        !atomicReplace(temporary, target) ||
        !syncPath(target.parent_path(), true, reason)) {
      std::error_code ignored;
      fs::remove(temporary, ignored);
      if (reason && reason->empty()) *reason = "checkpoint_publish_failed";
      return false;
    }
    if (reason) *reason = "saved";
    return true;
  } catch (const std::exception& error) {
    if (reason) *reason = error.what();
    return false;
  }
}

RecoveryCheckpointLoadResult RecoveryCheckpoint::loadVerified(
    const std::string& path, const std::string& expected_map_uuid,
    std::uint64_t expected_map_generation) {
  RecoveryCheckpointLoadResult result;
  try {
    const YAML::Node document = YAML::LoadFile(path);

    // --- strict schema validation ---
    // The canonical checksum covers the payload fields, but YAML-cpp may
    // silently tolerate duplicate keys, extra keys, or missing keys in some
    // versions. Enforce an exact key set so tampered files cannot bypass
    // integrity checks through parser leniency.
    if (!document.IsDefined() || !document.IsMap()) {
      result.reason = "checkpoint_schema_invalid:not_a_map";
      return result;
    }
    const std::array<const char*, 10> kTopLevelKeys = {
        "schema", "schema_version", "map_uuid", "map_generation",
        "pose_generation", "continuity_generation", "source_stamp_sec",
        "pose", "source_git_sha", "checksum"};
    if (document.size() != kTopLevelKeys.size()) {
      result.reason = "checkpoint_schema_invalid:top_level_key_count";
      return result;
    }
    for (const char* key : kTopLevelKeys) {
      if (!document[key]) {
        result.reason = std::string("checkpoint_schema_invalid:missing_key:") +
            key;
        return result;
      }
    }
    const YAML::Node pose_node = document["pose"];
    if (!pose_node.IsDefined() || !pose_node.IsMap()) {
      result.reason = "checkpoint_schema_invalid:pose_not_a_map";
      return result;
    }
    const std::array<const char*, 4> kPoseKeys = {"x", "y", "z", "yaw"};
    if (pose_node.size() != kPoseKeys.size()) {
      result.reason = "checkpoint_schema_invalid:pose_key_count";
      return result;
    }
    for (const char* key : kPoseKeys) {
      if (!pose_node[key]) {
        result.reason = std::string("checkpoint_schema_invalid:missing_pose_key:")
            + key;
        return result;
      }
    }

    if (document["schema"].as<std::string>("") !=
        "ndt_slam_recovery_checkpoint") {
      result.reason = "checkpoint_schema_invalid";
      return result;
    }
    // YAML node → double; tolerates both native numeric nodes and the
    // high-precision string encoding that the save path uses to preserve
    // the canonical checksum contract.
    auto asDouble = [](const YAML::Node& node, double fallback) -> double {
      if (!node.IsDefined()) return fallback;
      if (node.Type() == YAML::NodeType::Scalar) {
        try {
          return std::stod(node.as<std::string>());
        } catch (...) {
          return node.as<double>(fallback);
        }
      }
      return node.as<double>(fallback);
    };

    auto& value = result.checkpoint;
    value.schema_version = document["schema_version"].as<std::uint32_t>(0U);
    value.map_uuid = document["map_uuid"].as<std::string>("");
    value.map_generation = document["map_generation"].as<std::uint64_t>(0U);
    value.pose_generation = document["pose_generation"].as<std::uint64_t>(0U);
    value.continuity_generation =
        document["continuity_generation"].as<std::uint64_t>(0U);
    value.source_stamp_sec = asDouble(document["source_stamp_sec"], 0.0);
    value.x = asDouble(pose_node["x"], NAN);
    value.y = asDouble(pose_node["y"], NAN);
    value.z = asDouble(pose_node["z"], NAN);
    value.yaw = asDouble(pose_node["yaw"], NAN);
    value.source_git_sha = document["source_git_sha"].as<std::string>("");
    value.checksum = document["checksum"].as<std::string>("");
    if (!valid(value) || value.checksum != checksum(canonical(value))) {
      result.reason = "checkpoint_checksum_or_payload_invalid";
      return result;
    }
    if (value.map_uuid != expected_map_uuid ||
        value.map_generation != expected_map_generation) {
      result.reason = "checkpoint_map_identity_mismatch";
      return result;
    }
    result.valid = true;
    result.reason = "verified_seed_only";
    return result;
  } catch (const std::exception& error) {
    result.reason = error.what();
    return result;
  }
}

}  // namespace ndt_slam
