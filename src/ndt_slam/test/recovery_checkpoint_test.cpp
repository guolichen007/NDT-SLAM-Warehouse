#include "ndt_slam/recovery_checkpoint.hpp"

#include "ndt_slam/map_session_snapshot.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace ndt_slam {
namespace {

namespace fs = std::filesystem;

RecoveryCheckpointData sampleCheckpoint() {
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
  return value;
}

TEST(RecoveryCheckpointTest, IsVerifiedSeedNotAcceptedPoseAuthority) {
  const fs::path root = fs::temp_directory_path() /
      ("ndt-checkpoint-test-" + MapSessionSnapshot::generateUuid());
  fs::create_directories(root);
  const fs::path path = root / "checkpoint.yaml";
  auto value = sampleCheckpoint();
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
  auto value = sampleCheckpoint();
  value.map_uuid = "map-a";
  value.map_generation = 7U;
  value.pose_generation = 1U;
  value.continuity_generation = 1U;
  value.source_stamp_sec = 1.0;
  std::string reason;
  ASSERT_TRUE(RecoveryCheckpoint::saveAtomic(path.string(), value, &reason));

  // Map identity mismatch: expected "map-b" but checkpoint stores "map-a".
  EXPECT_FALSE(RecoveryCheckpoint::loadVerified(
      path.string(), "map-b", 7U).valid);

  // Content corruption: overwrite the checksum field so the canonical
  // checksum no longer matches the actual payload.
  {
    std::ifstream input(path);
    ASSERT_TRUE(input.is_open());
    std::string content((std::istreambuf_iterator<char>(input)),
                        std::istreambuf_iterator<char>());
    input.close();
    const auto pos = content.find("checksum:");
    ASSERT_NE(pos, std::string::npos);
    // Replace the checksum with a deliberately wrong value.
    const auto value_start = content.find_first_not_of(" \t", pos + 9);
    const auto value_end = content.find_first_of(" \t\r\n#", value_start);
    const std::string original = content.substr(value_start,
        value_end - value_start);
    std::string tampered = original;
    // Flip the first hex digit.
    tampered[0] = (tampered[0] == 'a') ? 'b' : 'a';
    content.replace(value_start, original.size(), tampered);
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    ASSERT_TRUE(output.is_open());
    output << content;
    output.flush();
    output.close();
  }
  EXPECT_FALSE(RecoveryCheckpoint::loadVerified(
      path.string(), "map-a", 7U).valid);

  std::error_code ignored;
  fs::remove_all(root, ignored);
}

TEST(RecoveryCheckpointTest, RejectsUnknownTopLevelKey) {
  const fs::path root = fs::temp_directory_path() /
      ("ndt-checkpoint-test-" + MapSessionSnapshot::generateUuid());
  fs::create_directories(root);
  const fs::path path = root / "checkpoint.yaml";
  auto value = sampleCheckpoint();
  std::string reason;
  ASSERT_TRUE(RecoveryCheckpoint::saveAtomic(path.string(), value, &reason));

  // Append an unknown top-level key. YAML-cpp will parse it as an additional
  // map entry, increasing document.size() past the expected 10.
  std::ofstream append(path, std::ios::app);
  append << "\nextra_field: injected\n";
  append.close();
  EXPECT_FALSE(RecoveryCheckpoint::loadVerified(
      path.string(), "map-a", 7U).valid);

  std::error_code ignored;
  fs::remove_all(root, ignored);
}

TEST(RecoveryCheckpointTest, RejectsMissingRequiredField) {
  // Create a checkpoint file manually with a missing top-level key
  // (checksum). The strict schema will reject it before checksum comparison.
  const fs::path root = fs::temp_directory_path() /
      ("ndt-checkpoint-test-" + MapSessionSnapshot::generateUuid());
  fs::create_directories(root);
  const fs::path path = root / "checkpoint.yaml";

  // Save a valid checkpoint first, then remove a key.
  auto value = sampleCheckpoint();
  std::string reason;
  ASSERT_TRUE(RecoveryCheckpoint::saveAtomic(path.string(), value, &reason));

  // Read the file, remove the checksum line, write back.
  std::ifstream input(path);
  ASSERT_TRUE(input.is_open());
  std::string content((std::istreambuf_iterator<char>(input)),
                       std::istreambuf_iterator<char>());
  input.close();
  const auto pos = content.find("checksum:");
  ASSERT_NE(pos, std::string::npos);
  const auto line_end = content.find('\n', pos);
  if (line_end != std::string::npos) {
    content.erase(pos, line_end - pos + 1);
  } else {
    // checksum is the last line without trailing newline.
    content.erase(pos);
  }
  std::ofstream output(path, std::ios::out | std::ios::trunc);
  ASSERT_TRUE(output.is_open());
  output << content;
  output.flush();
  output.close();

  EXPECT_FALSE(RecoveryCheckpoint::loadVerified(
      path.string(), "map-a", 7U).valid);

  std::error_code ignored;
  fs::remove_all(root, ignored);
}

TEST(RecoveryCheckpointTest, RejectsExtraPoseField) {
  // Append an extra field inside the pose map. The strict schema will reject
  // it because pose_node.size() will be > 4.
  const fs::path root = fs::temp_directory_path() /
      ("ndt-checkpoint-test-" + MapSessionSnapshot::generateUuid());
  fs::create_directories(root);
  const fs::path path = root / "checkpoint.yaml";
  auto value = sampleCheckpoint();
  std::string reason;
  ASSERT_TRUE(RecoveryCheckpoint::saveAtomic(path.string(), value, &reason));

  // Append an extra key inside the pose block. Since YAML uses indentation,
  // we append at the correct indentation level.
  std::ofstream append(path, std::ios::app);
  append << "  extra_pose_key: 0.5\n";
  append.close();
  EXPECT_FALSE(RecoveryCheckpoint::loadVerified(
      path.string(), "map-a", 7U).valid);

  std::error_code ignored;
  fs::remove_all(root, ignored);
}

}  // namespace
}  // namespace ndt_slam
