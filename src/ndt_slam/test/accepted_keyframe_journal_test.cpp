#include "ndt_slam/accepted_keyframe_journal.hpp"

#include "ndt_slam/map_session_snapshot.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace ndt_slam {
namespace {

namespace fs = std::filesystem;

AcceptedKeyframeRecord record(std::uint64_t sequence) {
  AcceptedKeyframeRecord value;
  value.sequence = sequence;
  value.map_uuid = "map-a";
  value.map_generation = 4U;
  value.continuity_generation = 2U;
  value.pose_generation = sequence;
  value.source_stamp_sec = static_cast<double>(sequence);
  value.x = static_cast<double>(sequence);
  value.registration_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  value.registration_cloud->push_back({1.0F, 2.0F, 3.0F});
  value.map_write_authorized = true;
  value.accepted = true;
  value.prediction = false;
  value.quarantined = false;
  value.stale = false;
  return value;
}

TEST(AcceptedKeyframeJournalTest, AcceptsOnlyAuthorizedKeyframes) {
  const fs::path root = fs::temp_directory_path() /
      ("ndt-journal-test-" + MapSessionSnapshot::generateUuid());
  const fs::path path = root / "accepted.journal";
  AcceptedKeyframeJournal journal;
  journal.configure({path.string(), 2U, 1024U * 1024U});
  std::string reason;
  ASSERT_TRUE(journal.start(&reason)) << reason;
  auto denied = record(1U);
  denied.map_write_authorized = false;
  EXPECT_FALSE(journal.submit(std::move(denied), &reason));
  EXPECT_TRUE(journal.submit(record(2U), &reason)) << reason;
  journal.stop();
  EXPECT_EQ(journal.writtenRecords(), 1U);
  const auto loaded = AcceptedKeyframeJournal::loadLastVerified(
      path.string(), "map-a", 4U);
  ASSERT_TRUE(loaded.valid) << loaded.reason;
  EXPECT_EQ(loaded.record.sequence, 2U);
  EXPECT_EQ(loaded.record.registration_cloud->size(), 1U);
  std::error_code ignored;
  fs::remove_all(root, ignored);
}

TEST(AcceptedKeyframeJournalTest, TruncatesPartialTailToLastCommit) {
  const fs::path root = fs::temp_directory_path() /
      ("ndt-journal-test-" + MapSessionSnapshot::generateUuid());
  const fs::path path = root / "accepted.journal";
  AcceptedKeyframeJournal journal;
  journal.configure({path.string(), 2U, 1024U * 1024U});
  ASSERT_TRUE(journal.start());
  ASSERT_TRUE(journal.submit(record(1U)));
  journal.stop();
  const auto committed_size = fs::file_size(path);
  std::ofstream partial(path, std::ios::binary | std::ios::app);
  partial << "partial-record";
  partial.close();
  const auto loaded = AcceptedKeyframeJournal::loadLastVerified(
      path.string(), "map-a", 4U, true);
  ASSERT_TRUE(loaded.valid) << loaded.reason;
  EXPECT_TRUE(loaded.truncated_tail);
  EXPECT_EQ(fs::file_size(path), committed_size);
  std::error_code ignored;
  fs::remove_all(root, ignored);
}

}  // namespace
}  // namespace ndt_slam
