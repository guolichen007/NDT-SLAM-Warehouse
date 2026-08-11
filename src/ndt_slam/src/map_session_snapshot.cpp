#include "ndt_slam/map_session_snapshot.hpp"

#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <vector>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace ndt_slam {
namespace {

namespace fs = std::filesystem;

constexpr std::array<std::uint32_t, 64> kSha256Round = {{
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
    0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
    0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
    0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
    0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
    0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
    0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
    0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
    0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
    0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
    0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
    0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
    0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
    0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
    0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
    0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U}};

std::uint32_t rotateRight(std::uint32_t value, unsigned int amount) {
  return (value >> amount) | (value << (32U - amount));
}

class Sha256 {
 public:
  void update(const unsigned char* data, std::size_t size) {
    for (std::size_t i = 0; i < size; ++i) {
      buffer_[buffer_size_++] = data[i];
      bit_count_ += 8U;
      if (buffer_size_ == buffer_.size()) {
        transform(buffer_.data());
        buffer_size_ = 0U;
      }
    }
  }

  std::string finish() {
    buffer_[buffer_size_++] = 0x80U;
    if (buffer_size_ > 56U) {
      while (buffer_size_ < 64U) buffer_[buffer_size_++] = 0U;
      transform(buffer_.data());
      buffer_size_ = 0U;
    }
    while (buffer_size_ < 56U) buffer_[buffer_size_++] = 0U;
    for (int shift = 56; shift >= 0; shift -= 8) {
      buffer_[buffer_size_++] =
          static_cast<unsigned char>((bit_count_ >> shift) & 0xffU);
    }
    transform(buffer_.data());
    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (const auto value : state_) output << std::setw(8) << value;
    return output.str();
  }

 private:
  void transform(const unsigned char* block) {
    std::array<std::uint32_t, 64> words{};
    for (std::size_t i = 0; i < 16U; ++i) {
      words[i] = (static_cast<std::uint32_t>(block[4U * i]) << 24U) |
          (static_cast<std::uint32_t>(block[4U * i + 1U]) << 16U) |
          (static_cast<std::uint32_t>(block[4U * i + 2U]) << 8U) |
          static_cast<std::uint32_t>(block[4U * i + 3U]);
    }
    for (std::size_t i = 16U; i < words.size(); ++i) {
      const std::uint32_t s0 = rotateRight(words[i - 15U], 7U) ^
          rotateRight(words[i - 15U], 18U) ^ (words[i - 15U] >> 3U);
      const std::uint32_t s1 = rotateRight(words[i - 2U], 17U) ^
          rotateRight(words[i - 2U], 19U) ^ (words[i - 2U] >> 10U);
      words[i] = words[i - 16U] + s0 + words[i - 7U] + s1;
    }
    std::uint32_t a = state_[0];
    std::uint32_t b = state_[1];
    std::uint32_t c = state_[2];
    std::uint32_t d = state_[3];
    std::uint32_t e = state_[4];
    std::uint32_t f = state_[5];
    std::uint32_t g = state_[6];
    std::uint32_t h = state_[7];
    for (std::size_t i = 0; i < words.size(); ++i) {
      const std::uint32_t s1 =
          rotateRight(e, 6U) ^ rotateRight(e, 11U) ^ rotateRight(e, 25U);
      const std::uint32_t choice = (e & f) ^ ((~e) & g);
      const std::uint32_t t1 = h + s1 + choice + kSha256Round[i] + words[i];
      const std::uint32_t s0 =
          rotateRight(a, 2U) ^ rotateRight(a, 13U) ^ rotateRight(a, 22U);
      const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t t2 = s0 + majority;
      h = g;
      g = f;
      f = e;
      e = d + t1;
      d = c;
      c = b;
      b = a;
      a = t1 + t2;
    }
    state_[0] += a;
    state_[1] += b;
    state_[2] += c;
    state_[3] += d;
    state_[4] += e;
    state_[5] += f;
    state_[6] += g;
    state_[7] += h;
  }

  std::array<std::uint32_t, 8> state_{{
      0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
      0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U}};
  std::array<unsigned char, 64> buffer_{};
  std::size_t buffer_size_ = 0U;
  std::uint64_t bit_count_ = 0U;
};

struct LayerSpec {
  const char* key;
  const char* filename;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;
};

std::vector<LayerSpec> layerSpecs(const MapSessionLayers& layers) {
  return {{"registration", "map_registration.pcd", layers.registration},
          {"display", "map_display.pcd", layers.display},
          {"ground", "map_ground.pcd", layers.ground},
          {"objects_raw", "map_objects_raw.pcd", layers.objects_raw},
          {"objects_clean", "map_objects_clean.pcd", layers.objects_clean}};
}

bool safeRelativePath(const std::string& name) {
  const fs::path path(name);
  return !name.empty() && path.is_relative() && !path.has_root_name() &&
      std::none_of(path.begin(), path.end(), [](const fs::path& part) {
        return part == "..";
      });
}

bool writeTextFile(const fs::path& path, const std::string& text) {
  std::ofstream output(path, std::ios::out | std::ios::trunc);
  output << text;
  output.flush();
  return output.good();
}

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

void requireSync(const fs::path& path, bool directory = false) {
  std::string reason;
  if (!syncPath(path, directory, &reason)) throw std::runtime_error(reason);
}

std::uint64_t yamlUInt64(const YAML::Node& node, const char* key) {
  return node[key] ? node[key].as<std::uint64_t>() : 0U;
}

}  // namespace

std::string MapSessionSnapshot::sha256File(const std::string& path,
                                           std::string* reason) {
  std::ifstream input(path, std::ios::binary);
  if (!input.is_open()) {
    if (reason) *reason = "hash_open_failed";
    return {};
  }
  Sha256 hash;
  std::array<unsigned char, 1U << 16U> buffer{};
  while (input.good()) {
    input.read(reinterpret_cast<char*>(buffer.data()), buffer.size());
    const auto count = input.gcount();
    if (count > 0) hash.update(buffer.data(), static_cast<std::size_t>(count));
  }
  if (!input.eof()) {
    if (reason) *reason = "hash_read_failed";
    return {};
  }
  if (reason) *reason = "ok";
  return hash.finish();
}

std::string MapSessionSnapshot::generateUuid() {
  std::random_device device;
  std::mt19937_64 generator(device());
  const auto now = static_cast<std::uint64_t>(
      std::chrono::high_resolution_clock::now().time_since_epoch().count());
  const std::uint64_t first = generator() ^ now;
  const std::uint64_t second = generator();
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(8)
         << static_cast<std::uint32_t>(first >> 32U) << '-' << std::setw(4)
         << static_cast<std::uint16_t>(first >> 16U) << '-' << std::setw(4)
         << static_cast<std::uint16_t>((first & 0x0fffU) | 0x4000U) << '-'
         << std::setw(4)
         << static_cast<std::uint16_t>(
                ((second >> 48U) & 0x3fffU) | 0x8000U)
         << '-' << std::setw(12) << (second & 0x0000ffffffffffffULL);
  return output.str();
}

bool MapSessionSnapshot::saveAtomic(const MapSessionSaveRequest& request,
                                    std::string* reason) {
  fs::path temporary;
  try {
    if (request.target_directory.empty()) {
      if (reason) *reason = "target_directory_empty";
      return false;
    }
    const fs::path target = fs::absolute(request.target_directory);
    if (fs::exists(target)) {
      if (reason) *reason = "target_already_exists";
      return false;
    }
    const auto layers = layerSpecs(request.layers);
    for (const auto& layer : layers) {
      if (!layer.cloud) {
        if (reason) *reason = std::string("formal_layer_missing:") + layer.key;
        return false;
      }
    }
    if (!request.static_evidence) {
      if (reason) *reason = "static_evidence_missing";
      return false;
    }
    fs::create_directories(target.parent_path());
    temporary = target.parent_path() /
        (target.filename().string() + ".tmp-" + generateUuid());
    fs::create_directory(temporary);

    YAML::Node files;
    for (const auto& layer : layers) {
      const fs::path output = temporary / layer.filename;
      if (pcl::io::savePCDFileBinary(output.string(), *layer.cloud) < 0) {
        throw std::runtime_error(std::string("pcd_write_failed:") + layer.key);
      }
      const std::string digest = sha256File(output.string());
      if (digest.empty()) {
        throw std::runtime_error(std::string("pcd_hash_failed:") + layer.key);
      }
      files[layer.key]["path"] = layer.filename;
      files[layer.key]["sha256"] = digest;
      files[layer.key]["points"] =
          static_cast<std::uint64_t>(layer.cloud->size());
    }

    // Full display is a compatibility alias of the coherent display layer,
    // never a mixture of diagnostic clouds from another generation.
    const fs::path display_full = temporary / "map_display_full.pcd";
    fs::copy_file(temporary / "map_display.pcd", display_full,
                  fs::copy_options::overwrite_existing);
    files["display_full"]["path"] = "map_display_full.pcd";
    files["display_full"]["sha256"] = sha256File(display_full.string());
    files["display_full"]["points"] =
        static_cast<std::uint64_t>(request.layers.display->size());

    StaticObstacleEvidenceConfig index_config;
    index_config.cell_size_m = request.static_evidence->cell_size_m;
    StaticObstacleEvidenceIndex serializer(index_config);
    std::string static_reason;
    const fs::path static_path = temporary / "static_evidence.csv";
    if (!serializer.saveSnapshot(request.static_evidence,
                                 static_path.string(), &static_reason)) {
      throw std::runtime_error("static_evidence_write_failed:" + static_reason);
    }
    files["static_evidence"]["path"] = "static_evidence.csv";
    files["static_evidence"]["sha256"] = sha256File(static_path.string());
    files["static_evidence"]["cells"] = static_cast<std::uint64_t>(
        request.static_evidence->cells.size());

    if (request.write_extras) {
      std::string extras_reason;
      if (!request.write_extras(temporary.string(), &extras_reason)) {
        throw std::runtime_error("extras_write_failed:" + extras_reason);
      }
    }

    // Make every payload durable before publishing the manifest which claims
    // the generation is complete. This includes caller-provided extras.
    for (const auto& entry : fs::recursive_directory_iterator(temporary)) {
      if (entry.is_regular_file()) requireSync(entry.path());
    }

    YAML::Node extra_files;
    for (const auto& entry : fs::recursive_directory_iterator(temporary)) {
      if (!entry.is_regular_file()) continue;
      const std::string relative =
          fs::relative(entry.path(), temporary).generic_string();
      if (relative == "map_registration.pcd" ||
          relative == "map_display.pcd" ||
          relative == "map_display_full.pcd" ||
          relative == "map_ground.pcd" ||
          relative == "map_objects_raw.pcd" ||
          relative == "map_objects_clean.pcd" ||
          relative == "static_evidence.csv") {
        continue;
      }
      YAML::Node item;
      item["path"] = relative;
      item["sha256"] = sha256File(entry.path().string());
      item["bytes"] = static_cast<std::uint64_t>(entry.file_size());
      extra_files.push_back(item);
    }

    const std::string map_uuid = request.metadata.map_uuid.empty()
        ? generateUuid() : request.metadata.map_uuid;
    YAML::Node manifest;
    manifest["schema"] = "ndt_slam_map_session";
    manifest["schema_version"] = kSchemaVersion;
    manifest["complete"] = true;
    manifest["map_uuid"] = map_uuid;
    manifest["frame_id"] = request.metadata.frame_id;
    manifest["source_git_sha"] = request.metadata.source_git_sha;
    manifest["source_git_branch"] = request.metadata.source_git_branch;
    manifest["source_stamp_sec"] = request.metadata.source_stamp_sec;
    manifest["map_generation"] = request.metadata.map_generation;
    manifest["objects_content_version"] =
        request.metadata.objects_content_version;
    manifest["clean_build_version"] = request.metadata.clean_build_version;
    manifest["keyframe_count"] = request.metadata.keyframe_count;
    manifest["active_only"] = request.metadata.active_only;
    manifest["objects_voxel_size_m"] =
        request.metadata.objects_voxel_size_m;
    manifest["static_evidence"]["schema_version"] =
        request.static_evidence->schema_version;
    manifest["static_evidence"]["map_generation"] =
        request.static_evidence->map_generation;
    manifest["static_evidence"]["revision"] =
        request.static_evidence->revision;
    manifest["static_evidence"]["authority"] =
        staticEvidenceAuthorityName(request.static_evidence->authority);
    manifest["files"] = files;
    manifest["extra_files"] = extra_files;
    YAML::Emitter emitter;
    emitter << manifest;
    if (!emitter.good() ||
        !writeTextFile(temporary / "manifest.yaml", emitter.c_str())) {
      throw std::runtime_error("manifest_write_failed");
    }
    requireSync(temporary / "manifest.yaml");

    // Read the complete temporary transaction back through the same strict
    // verifier used at restart. A write that cannot be reloaded never becomes
    // visible under the requested session directory.
    const MapSessionLoadResult verification =
        MapSessionSnapshot::loadVerified(temporary.string());
    if (!verification.valid) {
      throw std::runtime_error(
          "temporary_session_verification_failed:" + verification.reason);
    }
    if (verification.metadata.active_only != request.metadata.active_only ||
        verification.metadata.map_generation !=
            request.metadata.map_generation ||
        verification.static_evidence_revision !=
            request.static_evidence->revision) {
      throw std::runtime_error("temporary_session_identity_mismatch");
    }

    requireSync(temporary, true);
    fs::rename(temporary, target);
    requireSync(target.parent_path(), true);
    if (reason) *reason = target.string();
    return true;
  } catch (const std::exception& error) {
    std::error_code cleanup_error;
    if (!temporary.empty()) fs::remove_all(temporary, cleanup_error);
    if (reason) *reason = error.what();
    return false;
  }
}

MapSessionLoadResult MapSessionSnapshot::loadVerified(
    const std::string& directory) {
  MapSessionLoadResult result;
  result.session_directory = fs::absolute(directory).string();
  try {
    const fs::path root(result.session_directory);
    const fs::path manifest_path = root / "manifest.yaml";
    if (!fs::is_regular_file(manifest_path)) {
      result.reason = "manifest_missing";
      return result;
    }
    const YAML::Node manifest = YAML::LoadFile(manifest_path.string());
    if (manifest["schema"].as<std::string>("") != "ndt_slam_map_session" ||
        manifest["schema_version"].as<std::uint32_t>(0U) != kSchemaVersion ||
        !manifest["complete"].as<bool>(false)) {
      result.reason = "manifest_schema_or_completeness_invalid";
      return result;
    }
    result.metadata.map_uuid = manifest["map_uuid"].as<std::string>("");
    result.metadata.frame_id = manifest["frame_id"].as<std::string>("");
    if (result.metadata.map_uuid.empty() || result.metadata.frame_id.empty()) {
      result.reason = "manifest_identity_invalid";
      return result;
    }
    result.metadata.source_git_sha =
        manifest["source_git_sha"].as<std::string>("unknown");
    result.metadata.source_git_branch =
        manifest["source_git_branch"].as<std::string>("unknown");
    result.metadata.source_stamp_sec =
        manifest["source_stamp_sec"].as<double>(0.0);
    result.metadata.map_generation = yamlUInt64(manifest, "map_generation");
    result.metadata.objects_content_version =
        yamlUInt64(manifest, "objects_content_version");
    result.metadata.clean_build_version =
        yamlUInt64(manifest, "clean_build_version");
    result.metadata.keyframe_count = yamlUInt64(manifest, "keyframe_count");
    result.metadata.active_only = manifest["active_only"].as<bool>(false);
    result.metadata.objects_voxel_size_m =
        manifest["objects_voxel_size_m"].as<float>(0.0F);

    const YAML::Node static_node = manifest["static_evidence"];
    result.static_evidence_revision = yamlUInt64(static_node, "revision");
    result.static_evidence_source_generation =
        yamlUInt64(static_node, "map_generation");
    const std::string authority =
        static_node["authority"].as<std::string>("");
    if (authority == "RUNTIME_MATURE") {
      result.static_authority = StaticEvidenceAuthority::RUNTIME_MATURE;
    } else if (authority == "OPERATOR_APPROVED_BASELINE") {
      result.static_authority =
          StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE;
    } else if (authority == "UNVERIFIED_LOADED_CLEAN") {
      result.static_authority =
          StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN;
    } else {
      result.reason = "static_authority_invalid";
      return result;
    }

    const YAML::Node files = manifest["files"];
    const YAML::Node extra_files = manifest["extra_files"];
    if (!extra_files || !extra_files.IsSequence()) {
      result.reason = "extra_file_manifest_missing";
      return result;
    }
    for (const auto& extra : extra_files) {
      const std::string relative = extra["path"].as<std::string>("");
      const std::string expected = extra["sha256"].as<std::string>("");
      if (!safeRelativePath(relative) || expected.size() != 64U ||
          sha256File((root / relative).string()) != expected ||
          fs::file_size(root / relative) !=
              extra["bytes"].as<std::uint64_t>(
                  std::numeric_limits<std::uint64_t>::max())) {
        result.reason = "extra_file_contract_invalid:" + relative;
        return result;
      }
    }
    auto load_layer = [&](const char* key,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr* output) {
      const YAML::Node file = files[key];
      const std::string relative = file["path"].as<std::string>("");
      const std::string expected = file["sha256"].as<std::string>("");
      if (!safeRelativePath(relative) || expected.size() != 64U) {
        throw std::runtime_error(std::string("file_contract_invalid:") + key);
      }
      const fs::path absolute = root / relative;
      if (sha256File(absolute.string()) != expected) {
        throw std::runtime_error(std::string("hash_mismatch:") + key);
      }
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
          new pcl::PointCloud<pcl::PointXYZ>);
      if (pcl::io::loadPCDFile(absolute.string(), *cloud) < 0 ||
          cloud->size() != file["points"].as<std::uint64_t>(
              std::numeric_limits<std::uint64_t>::max())) {
        throw std::runtime_error(std::string("pcd_invalid:") + key);
      }
      *output = std::move(cloud);
    };

    pcl::PointCloud<pcl::PointXYZ>::Ptr registration, display, ground;
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects_raw, objects_clean;
    load_layer("registration", &registration);
    load_layer("display", &display);
    load_layer("ground", &ground);
    load_layer("objects_raw", &objects_raw);
    load_layer("objects_clean", &objects_clean);
    const YAML::Node static_file = files["static_evidence"];
    const std::string static_relative =
        static_file["path"].as<std::string>("");
    if (!safeRelativePath(static_relative) ||
        sha256File((root / static_relative).string()) !=
            static_file["sha256"].as<std::string>("")) {
      result.reason = "hash_mismatch:static_evidence";
      return result;
    }
    // Verify the display compatibility alias too; it must be byte-identical.
    const YAML::Node display_full = files["display_full"];
    const std::string full_relative =
        display_full["path"].as<std::string>("");
    if (!safeRelativePath(full_relative) ||
        sha256File((root / full_relative).string()) !=
            display_full["sha256"].as<std::string>("") ||
        display_full["sha256"].as<std::string>("") !=
            files["display"]["sha256"].as<std::string>("")) {
      result.reason = "display_full_contract_invalid";
      return result;
    }
    result.layers.registration = registration;
    result.layers.display = display;
    result.layers.ground = ground;
    result.layers.objects_raw = objects_raw;
    result.layers.objects_clean = objects_clean;
    result.static_evidence_path = (root / static_relative).string();
    result.valid = true;
    result.reason = "verified";
    return result;
  } catch (const std::exception& error) {
    result.reason = error.what();
    return result;
  }
}

std::vector<Eigen::Vector3f> selectStaticHeightPointsForAuthority(
    const pcl::PointCloud<pcl::PointXYZ>& objects_clean,
    const StaticEvidenceSnapshot& evidence) {
  std::vector<Eigen::Vector3f> selected;
  selected.reserve(objects_clean.size());
  if (!std::isfinite(evidence.cell_size_m) || evidence.cell_size_m <= 0.0F) {
    return selected;
  }
  for (const pcl::PointXYZ& point : objects_clean.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    bool authorized = evidence.authority ==
        StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN;
    if (!authorized) {
      const std::int32_t x = static_cast<std::int32_t>(
          std::floor(point.x / evidence.cell_size_m));
      const std::int32_t y = static_cast<std::int32_t>(
          std::floor(point.y / evidence.cell_size_m));
      const auto found = evidence.cells.find(packStaticEvidenceCell(x, y));
      const bool explicit_cell = found != evidence.cells.end() &&
          found->second.clean_map_confirmed &&
          found->second.map_generation == evidence.map_generation;
      authorized = explicit_cell &&
          (evidence.authority ==
               StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE ||
           (evidence.authority == StaticEvidenceAuthority::RUNTIME_MATURE &&
            found->second.temporally_mature));
    }
    if (authorized) selected.emplace_back(point.x, point.y, point.z);
  }
  return selected;
}

}  // namespace ndt_slam
