#pragma once

#include <cstdint>
#include <string>

namespace ndt_slam {

struct RecoveryCheckpointData {
  std::uint32_t schema_version = 1U;
  std::string map_uuid;
  std::uint64_t map_generation = 0U;
  std::uint64_t pose_generation = 0U;
  std::uint64_t continuity_generation = 0U;
  double source_stamp_sec = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  std::string source_git_sha = "unknown";
  std::string checksum;
};

struct RecoveryCheckpointLoadResult {
  bool valid = false;
  std::string reason;
  RecoveryCheckpointData checkpoint;
};

class RecoveryCheckpoint {
 public:
  static constexpr std::uint32_t kSchemaVersion = 1U;

  static bool saveAtomic(const std::string& path,
                         const RecoveryCheckpointData& checkpoint,
                         std::string* reason = nullptr);
  static RecoveryCheckpointLoadResult loadVerified(
      const std::string& path, const std::string& expected_map_uuid,
      std::uint64_t expected_map_generation);
};

}  // namespace ndt_slam
