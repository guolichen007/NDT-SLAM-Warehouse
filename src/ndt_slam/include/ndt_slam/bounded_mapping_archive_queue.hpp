#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ndt_slam {

enum class MappingArchivePriority {
    CERTIFICATION_CRITICAL,
    BEST_EFFORT_DIAGNOSTIC
};

enum class MappingArchivePayload {
    PCD_BINARY,
    TEXT
};

struct BoundedMappingArchiveConfig {
    bool enabled = false;
    std::string root_dir;
    std::size_t max_jobs = 8U;
    std::uint64_t max_queue_bytes = 256ULL * 1024ULL * 1024ULL;
    double max_oldest_job_age_sec = 10.0;
    double max_write_latency_ms = 5000.0;
    double minimum_free_disk_gb = 5.0;
};

struct MappingArchiveJob {
    MappingArchivePriority priority =
        MappingArchivePriority::BEST_EFFORT_DIAGNOSTIC;
    MappingArchivePayload payload_type = MappingArchivePayload::TEXT;
    std::string relative_path;
    std::string text;
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;
    std::uint64_t estimated_bytes = 0U;
    std::chrono::steady_clock::time_point enqueued_at;
};

struct BoundedMappingArchiveMetrics {
    std::size_t queue_jobs = 0U;
    std::uint64_t queue_bytes = 0U;
    double oldest_job_age_sec = 0.0;
    double write_latency_ms = 0.0;
    double free_disk_gb = 0.0;
    std::uint64_t critical_refused_count = 0U;
    std::uint64_t diagnostic_drop_count = 0U;
    std::uint64_t completed_count = 0U;
    bool paused_io = false;
    bool archive_incomplete = false;
    std::string reason = "disabled";
};

// Disk access, PCD serialization, rename and SHA-256 generation happen only
// in this worker. enqueue() performs a bounded in-memory admission check and
// never waits for disk.
class BoundedMappingArchiveQueue {
public:
    BoundedMappingArchiveQueue() = default;
    ~BoundedMappingArchiveQueue();

    BoundedMappingArchiveQueue(const BoundedMappingArchiveQueue&) = delete;
    BoundedMappingArchiveQueue& operator=(
        const BoundedMappingArchiveQueue&) = delete;

    void configure(const BoundedMappingArchiveConfig& config);
    void start();
    void stop(bool drain);
    bool enqueue(MappingArchiveJob job);
    BoundedMappingArchiveMetrics metrics() const;
    bool healthyForCriticalArchive() const;
    bool idle() const;
    void resetForNewSegment();

    static std::uint64_t estimateCloudBytes(
        const pcl::PointCloud<pcl::PointXYZ>& cloud);

private:
    void workerLoop();
    bool writeJob(const MappingArchiveJob& job, double* latency_ms);
    void markFailure(MappingArchivePriority priority,
                     const std::string& reason);

    BoundedMappingArchiveConfig config_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<MappingArchiveJob> critical_queue_;
    std::deque<MappingArchiveJob> diagnostic_queue_;
    std::uint64_t queue_bytes_ = 0U;
    std::thread worker_;
    bool running_ = false;
    bool initialized_ = false;
    bool shutdown_ = false;
    bool drain_ = true;
    bool writing_ = false;
    std::uint64_t writing_bytes_ = 0U;
    std::chrono::steady_clock::time_point writing_enqueued_at_;
    double last_write_latency_ms_ = 0.0;
    double cached_free_disk_gb_ = 0.0;
    std::uint64_t critical_refused_count_ = 0U;
    std::uint64_t diagnostic_drop_count_ = 0U;
    std::uint64_t completed_count_ = 0U;
    bool paused_io_ = false;
    bool archive_incomplete_ = false;
    std::string reason_ = "disabled";
};

}  // namespace ndt_slam
