#include "ndt_slam/mapping_segment_manager.hpp"
#include "ndt_slam/sha256.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>

namespace ndt_slam {
namespace {

std::filesystem::path segmentRoot(const std::string& name) {
  const auto ticks = std::chrono::steady_clock::now()
      .time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
      ("ndt_segment_" + name + "_" + std::to_string(ticks));
}

MappingSegmentConfig segmentConfig(const std::filesystem::path& root) {
  MappingSegmentConfig config;
  config.enabled = true;
  config.root_dir = root.string();
  config.campaign_uuid = "campaign-contract";
  config.survey_pass_id = "PASS_001";
  return config;
}

MappingSegmentStartPrerequisites readyPrerequisites() {
  MappingSegmentStartPrerequisites prerequisites;
  prerequisites.source_time_continuous = true;
  prerequisites.self_mask_commissioned = true;
  prerequisites.archive_healthy = true;
  prerequisites.archive_idle = true;
  return prerequisites;
}

TEST(MappingSegmentManagerTest, ExplicitOperatorStartIsRequired) {
  const auto root = segmentRoot("operator");
  MappingSegmentManager manager;
  manager.configure(segmentConfig(root));
  std::string reason;
  ASSERT_TRUE(manager.initialize(&reason));
  EXPECT_EQ(manager.snapshot().state, MappingSegmentState::WAIT_OPERATOR);

  auto prerequisites = readyPrerequisites();
  prerequisites.self_mask_commissioned = false;
  EXPECT_FALSE(manager.startNewSegment(prerequisites, &reason));
  EXPECT_EQ(reason, "self_mask_not_commissioned");

  ASSERT_TRUE(manager.startNewSegment(readyPrerequisites(), &reason));
  EXPECT_EQ(manager.snapshot().state, MappingSegmentState::RUNNING);
  EXPECT_TRUE(manager.snapshot().writes_allowed);
  manager.failClosed("confirmed_localization_failure");
  EXPECT_EQ(manager.snapshot().state, MappingSegmentState::FAILED_CLOSED);
  EXPECT_FALSE(manager.snapshot().writes_allowed);
  std::filesystem::remove_all(root);
}

TEST(MappingSegmentManagerTest, OrphanRunningLockLatchesCrashAbort) {
  const auto root = segmentRoot("crash");
  std::filesystem::create_directories(root);
  {
    std::ofstream output(root / "RUNNING.lock");
    output << "orphan-segment\n";
  }
  MappingSegmentManager manager;
  manager.configure(segmentConfig(root));
  std::string reason;
  ASSERT_TRUE(manager.initialize(&reason));
  const auto snapshot = manager.snapshot();
  EXPECT_EQ(snapshot.state, MappingSegmentState::ABORTED_CRASH);
  EXPECT_TRUE(snapshot.previous_crash_detected);
  EXPECT_FALSE(snapshot.writes_allowed);
  std::filesystem::remove_all(root);
}

TEST(MappingSegmentManagerTest, FailedClosedLatchSurvivesRepeatedRestarts) {
  const auto root = segmentRoot("persistent_failed_closed");
  MappingSegmentManager first;
  first.configure(segmentConfig(root));
  std::string reason;
  ASSERT_TRUE(first.initialize(&reason));
  ASSERT_TRUE(first.startNewSegment(readyPrerequisites(), &reason));
  first.failClosed("confirmed_localization_failure");

  const MappingSegmentSnapshot failed = first.snapshot();
  const auto segment_dir = root / "segments" / failed.segment_uuid;
  std::filesystem::create_directories(segment_dir);
  {
    std::ofstream state(segment_dir / "state.json");
    state << first.stateJson();
  }
  {
    std::ofstream checksum(segment_dir / "state.json.sha256");
    checksum << sha256File((segment_dir / "state.json").string())
             << "  state.json\n";
  }

  MappingSegmentManager second;
  second.configure(segmentConfig(root));
  ASSERT_TRUE(second.initialize(&reason));
  EXPECT_EQ(second.snapshot().state, MappingSegmentState::FAILED_CLOSED);
  EXPECT_FALSE(second.snapshot().previous_crash_detected);

  MappingSegmentManager third;
  third.configure(segmentConfig(root));
  ASSERT_TRUE(third.initialize(&reason));
  EXPECT_EQ(third.snapshot().state, MappingSegmentState::FAILED_CLOSED);
  EXPECT_FALSE(third.snapshot().previous_crash_detected);
  std::filesystem::remove_all(root);
}

TEST(MappingSegmentManagerTest, CorruptArchivedTerminalStateIsCrashAborted) {
  const auto root = segmentRoot("corrupt_terminal");
  MappingSegmentManager first;
  first.configure(segmentConfig(root));
  std::string reason;
  ASSERT_TRUE(first.initialize(&reason));
  ASSERT_TRUE(first.startNewSegment(readyPrerequisites(), &reason));
  first.failClosed("confirmed_localization_failure");

  const MappingSegmentSnapshot failed = first.snapshot();
  const auto segment_dir = root / "segments" / failed.segment_uuid;
  std::filesystem::create_directories(segment_dir);
  {
    std::ofstream state(segment_dir / "state.json");
    state << first.stateJson();
  }
  {
    std::ofstream checksum(segment_dir / "state.json.sha256");
    checksum << std::string(64U, '0') << "  state.json\n";
  }

  MappingSegmentManager restarted;
  restarted.configure(segmentConfig(root));
  ASSERT_TRUE(restarted.initialize(&reason));
  EXPECT_EQ(restarted.snapshot().state, MappingSegmentState::ABORTED_CRASH);
  EXPECT_TRUE(restarted.snapshot().previous_crash_detected);
  std::filesystem::remove_all(root);
}

}  // namespace
}  // namespace ndt_slam
