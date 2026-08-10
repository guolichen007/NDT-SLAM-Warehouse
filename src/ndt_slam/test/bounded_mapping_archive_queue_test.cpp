#include "ndt_slam/bounded_mapping_archive_queue.hpp"

#include <gtest/gtest.h>

#include <filesystem>

namespace ndt_slam {
namespace {

BoundedMappingArchiveConfig archiveConfig(const std::string& suffix) {
  BoundedMappingArchiveConfig config;
  config.enabled = true;
  config.root_dir = (std::filesystem::temp_directory_path() /
                     ("ndt_archive_contract_" + suffix)).string();
  config.max_jobs = 8U;
  config.max_queue_bytes = 1024U;
  config.minimum_free_disk_gb = 0.0;
  return config;
}

TEST(BoundedMappingArchiveQueueTest, CriticalOversizePausesIo) {
  BoundedMappingArchiveQueue queue;
  queue.configure(archiveConfig("critical"));
  queue.start();
  MappingArchiveJob job;
  job.priority = MappingArchivePriority::CERTIFICATION_CRITICAL;
  job.payload_type = MappingArchivePayload::TEXT;
  job.relative_path = "oversize.txt";
  job.text = "small";
  job.estimated_bytes = 2048U;
  EXPECT_FALSE(queue.enqueue(std::move(job)));
  const auto metrics = queue.metrics();
  EXPECT_TRUE(metrics.paused_io);
  EXPECT_TRUE(metrics.archive_incomplete);
  EXPECT_EQ(metrics.critical_refused_count, 1U);
  queue.stop(false);
}

TEST(BoundedMappingArchiveQueueTest, DiagnosticOversizeOnlyDrops) {
  BoundedMappingArchiveQueue queue;
  queue.configure(archiveConfig("diagnostic"));
  queue.start();
  MappingArchiveJob job;
  job.priority = MappingArchivePriority::BEST_EFFORT_DIAGNOSTIC;
  job.payload_type = MappingArchivePayload::TEXT;
  job.relative_path = "oversize.txt";
  job.estimated_bytes = 2048U;
  EXPECT_FALSE(queue.enqueue(std::move(job)));
  const auto metrics = queue.metrics();
  EXPECT_FALSE(metrics.paused_io);
  EXPECT_FALSE(metrics.archive_incomplete);
  EXPECT_EQ(metrics.diagnostic_drop_count, 1U);
  queue.stop(false);
}

TEST(BoundedMappingArchiveQueueTest, CloudBudgetUsesActualPointStorage) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.resize(100U);
  EXPECT_EQ(BoundedMappingArchiveQueue::estimateCloudBytes(cloud),
            100U * sizeof(pcl::PointXYZ));
}

}  // namespace
}  // namespace ndt_slam
