#include "ndt_slam/recovery_checkpoint.hpp"

#include "ndt_slam/map_session_snapshot.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace ndt_slam {
namespace {

namespace fs = std::filesystem;

TEST(RecoveryCheckpointTest, IsVerifiedSeedNotAcceptedPoseAuthority) {
  const fs::path root = fs::temp_directory_path() /
      ("ndt-checkpoint-test-" + MapSessionSnapshot::generateUuid());
  fs::create_directories(root);
  const fs::path path = root / "checkpoint.yaml";
  RecoveryCheckpointData value;
  value.map_uuid = "map-a";
  value.map_generation = 7U;
  value.pose_generation = 11U;
  value.continuity_generation = 3U;
  value.source_stamp_sec = 123.5;
  value.x = 18.2;
  value.y = 4.7;
  value.z = 9.0;
  value.yaw = 0.002;
  value.source_git_sha = "334ba3b";
  std::string reason;
  ASSERT_TRUE(RecoveryCheckpoint::saveAtomic(path.string(), value, &reason))
      << reason;
  const auto loaded = RecoveryCheckpoint::loadVerified(
      path.string(), "map-a", 7U);
  ASSERT_TRUE(loaded.valid) << loaded.reason;
  EXPECT_EQ(loaded.reason, "verified_seed_only");
  EXPECT_DOUBLE_EQ(loaded.checkpoint.x, 18.2);
  std::error_code ignored;
  fs::remove_all(root, ignored);
}

TEST(RecoveryCheckpointTest, RejectsMapIdentityAndCorruption) {
  const fs::path root = fs::temp_directory_path() /
      ("ndt-checkpoint-test-" + MapSessionSnapshot::generateUuid());
  fs::create_directories(root);
  const fs::path path = root / "checkpoint.yaml";
  RecoveryCheckpointData value;
  value.map_uuid = "map-a";
  value.map_generation = 7U;
  value.pose_generation = 1U;
  value.continuity_generation = 1U;
  value.source_stamp_sec = 1.0;
  std::string reason;
  ASSERT_TRUE(RecoveryCheckpoint::saveAtomic(path.string(), value, &reason));
  EXPECT_FALSE(RecoveryCheckpoint::loadVerified(
      path.string(), "map-b", 7U).valid);
  std::ofstream corrupt(path, std::ios::app);
  corrupt << "\npose: {x: 999}\n";
  corrupt.close();
  EXPECT_FALSE(RecoveryCheckpoint::loadVerified(
      path.string(), "map-a", 7U).valid);
  std::error_code ignored;
  fs::remove_all(root, ignored);
}

}  // namespace
}  // namespace ndt_slam
