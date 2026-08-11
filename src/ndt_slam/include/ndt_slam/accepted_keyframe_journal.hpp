#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

namespace ndt_slam {

struct AcceptedKeyframeRecord {
  std::uint64_t sequence = 0U;
  std::string map_uuid;
  std::uint64_t map_generation = 0U;
  std::uint64_t continuity_generation = 0U;
  std::uint64_t pose_generation = 0U;
  double source_stamp_sec = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr registration_cloud;

  // Submission evidence. Only the single existing MapWriteAuthority result is
  // authoritative; the remaining fields prevent accidental misuse at call
  // sites and are independently checked by the journal.
  bool map_write_authorized = false;
  bool accepted = false;
  bool prediction = true;
  bool quarantined = true;
  bool stale = true;
};

struct AcceptedKeyframeJournalConfig {
  std::string path;
  std::size_t max_jobs = 4U;
  std::size_t max_bytes = 16U * 1024U * 1024U;
};

struct AcceptedKeyframeJournalLoadResult {
  bool valid = false;
  std::string reason;
  AcceptedKeyframeRecord record;
  std::uint64_t verified_records = 0U;
  bool truncated_tail = false;
};

class AcceptedKeyframeJournal {
 public:
  AcceptedKeyframeJournal() = default;
  ~AcceptedKeyframeJournal();
  AcceptedKeyframeJournal(const AcceptedKeyframeJournal&) = delete;
  AcceptedKeyframeJournal& operator=(const AcceptedKeyframeJournal&) = delete;

  void configure(const AcceptedKeyframeJournalConfig& config);
  bool start(std::string* reason = nullptr);
  void stop();

  // Never waits for I/O. False means rejected evidence or bounded-queue drop.
  bool submit(AcceptedKeyframeRecord record,
              std::string* reason = nullptr);

  std::uint64_t writtenRecords() const;
  std::uint64_t droppedRecords() const;

  static AcceptedKeyframeJournalLoadResult loadLastVerified(
      const std::string& path, const std::string& expected_map_uuid,
      std::uint64_t expected_map_generation, bool truncate_partial_tail = true);

 private:
  void workerLoop();

  AcceptedKeyframeJournalConfig config_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<AcceptedKeyframeRecord> jobs_;
  std::size_t queued_bytes_ = 0U;
  std::thread worker_;
  bool started_ = false;
  bool stopping_ = false;
  std::uint64_t written_records_ = 0U;
  std::uint64_t dropped_records_ = 0U;
};

}  // namespace ndt_slam
