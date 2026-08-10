#include <ndt_slam/bounded_mapping_archive_queue.hpp>

#include <ndt_slam/sha256.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <system_error>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

#include <pcl/io/pcd_io.h>

namespace ndt_slam {
namespace fs = std::filesystem;

namespace {

bool safeRelativePath(const fs::path& path) {
    if (path.empty() || path.is_absolute()) return false;
    for (const fs::path& component : path) {
        if (component == "..") return false;
    }
    return true;
}

bool syncFile(const fs::path& path) {
#ifdef _WIN32
    const int descriptor = _open(
        path.string().c_str(), _O_RDWR | _O_BINARY);
    if (descriptor < 0) return false;
    const bool ok = _commit(descriptor) == 0;
    _close(descriptor);
    return ok;
#else
    const int descriptor = ::open(
        path.c_str(), O_RDONLY | O_CLOEXEC);
    if (descriptor < 0) return false;
    const bool ok = ::fsync(descriptor) == 0;
    ::close(descriptor);
    return ok;
#endif
}

bool syncDirectory(const fs::path& path) {
#ifdef _WIN32
    (void)path;
    return true;
#else
    const int descriptor = ::open(
        path.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (descriptor < 0) return false;
    const bool ok = ::fsync(descriptor) == 0;
    ::close(descriptor);
    return ok;
#endif
}

}  // namespace

BoundedMappingArchiveQueue::~BoundedMappingArchiveQueue() {
    stop(true);
}

void BoundedMappingArchiveQueue::configure(
    const BoundedMappingArchiveConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
    config_.max_jobs = std::max<std::size_t>(1U, config_.max_jobs);
    config_.max_queue_bytes = std::max<std::uint64_t>(
        1024U, config_.max_queue_bytes);
    config_.max_oldest_job_age_sec = std::max(
        0.1, config_.max_oldest_job_age_sec);
    config_.max_write_latency_ms = std::max(
        1.0, config_.max_write_latency_ms);
    config_.minimum_free_disk_gb = std::max(
        0.0, config_.minimum_free_disk_gb);
    reason_ = config_.enabled ? "ready_to_start" : "disabled";
}

void BoundedMappingArchiveQueue::start() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!config_.enabled || running_) return;
    shutdown_ = false;
    drain_ = true;
    running_ = true;
    initialized_ = false;
    reason_ = "starting";
    worker_ = std::thread(&BoundedMappingArchiveQueue::workerLoop, this);
}

void BoundedMappingArchiveQueue::stop(bool drain) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) return;
        drain_ = drain;
        shutdown_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
}

std::uint64_t BoundedMappingArchiveQueue::estimateCloudBytes(
    const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    return static_cast<std::uint64_t>(cloud.size()) *
        static_cast<std::uint64_t>(sizeof(pcl::PointXYZ));
}

bool BoundedMappingArchiveQueue::enqueue(MappingArchiveJob job) {
    if (job.estimated_bytes == 0U) {
        job.estimated_bytes = job.payload_type == MappingArchivePayload::PCD_BINARY &&
                job.cloud
            ? estimateCloudBytes(*job.cloud)
            : static_cast<std::uint64_t>(job.text.size());
    }
    job.enqueued_at = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    const auto over_jobs_limit = [this]() {
        return critical_queue_.size() + diagnostic_queue_.size() +
            (writing_ ? 1U : 0U) >= config_.max_jobs;
    };
    const auto over_bytes_limit = [this, &job]() {
        return job.estimated_bytes > config_.max_queue_bytes ||
            queue_bytes_ > config_.max_queue_bytes -
                std::min(job.estimated_bytes, config_.max_queue_bytes);
    };
    // Best-effort diagnostics must never consume the last bounded slot or
    // byte budget needed by certification-critical evidence. Eviction is
    // limited to queued diagnostics; an in-flight write is never interrupted.
    if (job.priority == MappingArchivePriority::CERTIFICATION_CRITICAL) {
        while (!diagnostic_queue_.empty() &&
               (over_jobs_limit() || over_bytes_limit())) {
            const std::uint64_t released =
                diagnostic_queue_.back().estimated_bytes;
            diagnostic_queue_.pop_back();
            queue_bytes_ = released > queue_bytes_
                ? 0U : queue_bytes_ - released;
            ++diagnostic_drop_count_;
        }
    }
    const bool over_jobs = over_jobs_limit();
    const bool over_bytes = over_bytes_limit();
    const bool disk_unhealthy = initialized_ &&
        cached_free_disk_gb_ < config_.minimum_free_disk_gb;
    if (!config_.enabled || !running_ || !initialized_ || shutdown_ || over_jobs ||
        over_bytes || disk_unhealthy || paused_io_) {
        if (job.priority == MappingArchivePriority::CERTIFICATION_CRITICAL) {
            ++critical_refused_count_;
            archive_incomplete_ = true;
            paused_io_ = true;
            reason_ = !config_.enabled ? "archive_disabled" :
                (!initialized_ ? "archive_not_initialized" :
                (over_jobs ? "archive_max_jobs" :
                 (over_bytes ? "archive_max_queue_bytes" :
                  (disk_unhealthy ? "archive_disk_low" :
                   "archive_not_accepting"))));
        } else {
            ++diagnostic_drop_count_;
        }
        return false;
    }
    queue_bytes_ += job.estimated_bytes;
    if (job.priority == MappingArchivePriority::CERTIFICATION_CRITICAL) {
        critical_queue_.push_back(std::move(job));
    } else {
        diagnostic_queue_.push_back(std::move(job));
    }
    cv_.notify_one();
    return true;
}

BoundedMappingArchiveMetrics BoundedMappingArchiveQueue::metrics() const {
    std::lock_guard<std::mutex> lock(mutex_);
    BoundedMappingArchiveMetrics result;
    result.queue_jobs = critical_queue_.size() + diagnostic_queue_.size() +
        (writing_ ? 1U : 0U);
    result.queue_bytes = queue_bytes_;
    const auto now = std::chrono::steady_clock::now();
    const auto age = [&now](const std::deque<MappingArchiveJob>& queue) {
        return queue.empty() ? 0.0 :
            std::chrono::duration<double>(now - queue.front().enqueued_at).count();
    };
    const double writing_age = writing_
        ? std::chrono::duration<double>(
              now - writing_enqueued_at_).count()
        : 0.0;
    result.oldest_job_age_sec = std::max(
        writing_age, std::max(age(critical_queue_), age(diagnostic_queue_)));
    result.write_latency_ms = last_write_latency_ms_;
    result.free_disk_gb = cached_free_disk_gb_;
    result.critical_refused_count = critical_refused_count_;
    result.diagnostic_drop_count = diagnostic_drop_count_;
    result.completed_count = completed_count_;
    result.paused_io = paused_io_;
    result.archive_incomplete = archive_incomplete_;
    result.reason = reason_;
    return result;
}

bool BoundedMappingArchiveQueue::healthyForCriticalArchive() const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    const auto queue_age = [&now](
        const std::deque<MappingArchiveJob>& queue) {
        return queue.empty() ? 0.0 : std::chrono::duration<double>(
            now - queue.front().enqueued_at).count();
    };
    const double oldest_age = std::max(
        writing_ ? std::chrono::duration<double>(
                       now - writing_enqueued_at_).count() : 0.0,
        std::max(queue_age(critical_queue_), queue_age(diagnostic_queue_)));
    return config_.enabled && running_ && initialized_ && !shutdown_ && !paused_io_ &&
        !archive_incomplete_ &&
        cached_free_disk_gb_ >= config_.minimum_free_disk_gb &&
        oldest_age <= config_.max_oldest_job_age_sec &&
        last_write_latency_ms_ <= config_.max_write_latency_ms;
}

bool BoundedMappingArchiveQueue::idle() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return critical_queue_.empty() && diagnostic_queue_.empty() && !writing_;
}

void BoundedMappingArchiveQueue::resetForNewSegment() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!critical_queue_.empty() || !diagnostic_queue_.empty() || writing_) {
        return;
    }
    archive_incomplete_ = false;
    paused_io_ = cached_free_disk_gb_ < config_.minimum_free_disk_gb;
    reason_ = paused_io_ ? "archive_disk_low" : "active";
}

void BoundedMappingArchiveQueue::markFailure(
    MappingArchivePriority priority, const std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (priority == MappingArchivePriority::CERTIFICATION_CRITICAL) {
        archive_incomplete_ = true;
        paused_io_ = true;
        ++critical_refused_count_;
        reason_ = reason;
    } else {
        ++diagnostic_drop_count_;
    }
}

bool BoundedMappingArchiveQueue::writeJob(
    const MappingArchiveJob& job, double* latency_ms) {
    const auto start = std::chrono::steady_clock::now();
    const fs::path relative(job.relative_path);
    if (!safeRelativePath(relative)) return false;
    const fs::path target = fs::path(config_.root_dir) / relative;
    const fs::path temporary = target.string() + ".tmp";
    std::error_code error;
    fs::create_directories(target.parent_path(), error);
    if (error) return false;
    bool write_ok = false;
    if (job.payload_type == MappingArchivePayload::PCD_BINARY && job.cloud) {
        write_ok = pcl::io::savePCDFileBinary(
            temporary.string(), *job.cloud) == 0;
    } else if (job.payload_type == MappingArchivePayload::TEXT) {
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        output.write(job.text.data(),
                     static_cast<std::streamsize>(job.text.size()));
        output.flush();
        write_ok = output.good();
    }
    if (!write_ok) {
        fs::remove(temporary, error);
        return false;
    }
    if (!syncFile(temporary)) {
        fs::remove(temporary, error);
        return false;
    }
#ifdef _WIN32
    // Linux rename-overwrite is atomic. Windows requires removal first; the
    // Windows path is used only by local tests, not production collection.
    fs::remove(target, error);
    error.clear();
#endif
    fs::rename(temporary, target, error);
    if (error) return false;
    const std::string checksum = sha256File(target.string());
    if (checksum.empty()) return false;
    const fs::path checksum_target = target.string() + ".sha256";
    const fs::path checksum_temporary = checksum_target.string() + ".tmp";
    {
        std::ofstream checksum_file(
            checksum_temporary, std::ios::binary | std::ios::trunc);
        checksum_file << checksum << "  " << target.filename().string()
                      << '\n';
        checksum_file.flush();
        if (!checksum_file.good()) return false;
    }
    if (!syncFile(checksum_temporary)) {
        fs::remove(checksum_temporary, error);
        return false;
    }
#ifdef _WIN32
    fs::remove(checksum_target, error);
    error.clear();
#endif
    fs::rename(checksum_temporary, checksum_target, error);
    if (error || !syncDirectory(target.parent_path())) return false;
    *latency_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    return true;
}

void BoundedMappingArchiveQueue::workerLoop() {
    std::error_code error;
    fs::create_directories(config_.root_dir, error);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (error) {
            paused_io_ = true;
            archive_incomplete_ = true;
            reason_ = "archive_root_create_failed";
        } else {
            const fs::space_info info = fs::space(config_.root_dir, error);
            if (!error) {
                cached_free_disk_gb_ = static_cast<double>(info.available) /
                    (1024.0 * 1024.0 * 1024.0);
            }
            paused_io_ = error ||
                cached_free_disk_gb_ < config_.minimum_free_disk_gb;
            archive_incomplete_ = error;
            reason_ = error ? "archive_disk_space_query_failed" :
                (paused_io_ ? "archive_disk_low" : "active");
        }
        initialized_ = true;
    }
    while (true) {
        MappingArchiveJob job;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this]() {
                return shutdown_ || !critical_queue_.empty() ||
                    !diagnostic_queue_.empty();
            });
            if (shutdown_ && (!drain_ ||
                (critical_queue_.empty() && diagnostic_queue_.empty()))) {
                break;
            }
            if (!critical_queue_.empty()) {
                job = std::move(critical_queue_.front());
                critical_queue_.pop_front();
            } else if (!diagnostic_queue_.empty()) {
                job = std::move(diagnostic_queue_.front());
                diagnostic_queue_.pop_front();
            } else {
                continue;
            }
            writing_ = true;
            writing_bytes_ = job.estimated_bytes;
            writing_enqueued_at_ = job.enqueued_at;
            const double age = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - job.enqueued_at).count();
            if (age > config_.max_oldest_job_age_sec &&
                job.priority == MappingArchivePriority::CERTIFICATION_CRITICAL) {
                archive_incomplete_ = true;
                paused_io_ = true;
                ++critical_refused_count_;
                reason_ = "archive_job_age_exceeded";
                writing_ = false;
                queue_bytes_ = writing_bytes_ > queue_bytes_
                    ? 0U : queue_bytes_ - writing_bytes_;
                writing_bytes_ = 0U;
                continue;
            }
        }
        double latency_ms = 0.0;
        const bool ok = writeJob(job, &latency_ms);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            writing_ = false;
            queue_bytes_ = writing_bytes_ > queue_bytes_
                ? 0U : queue_bytes_ - writing_bytes_;
            writing_bytes_ = 0U;
            last_write_latency_ms_ = latency_ms;
            std::error_code space_error;
            const fs::space_info info = fs::space(config_.root_dir, space_error);
            if (!space_error) {
                cached_free_disk_gb_ = static_cast<double>(info.available) /
                    (1024.0 * 1024.0 * 1024.0);
            }
            const bool latency_bad = ok &&
                latency_ms > config_.max_write_latency_ms;
            const bool disk_bad = space_error ||
                cached_free_disk_gb_ < config_.minimum_free_disk_gb;
            if (ok && !latency_bad && !disk_bad) {
                ++completed_count_;
                reason_ = "active";
            } else if (job.priority ==
                       MappingArchivePriority::CERTIFICATION_CRITICAL) {
                archive_incomplete_ = true;
                paused_io_ = true;
                ++critical_refused_count_;
                reason_ = !ok ? "archive_write_failed" :
                    (latency_bad ? "archive_write_latency_exceeded" :
                     (space_error ? "archive_disk_space_query_failed" :
                      "archive_disk_low"));
            } else {
                ++diagnostic_drop_count_;
                if (space_error) {
                    paused_io_ = true;
                    reason_ = "archive_disk_space_query_failed";
                }
            }
        }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
    initialized_ = false;
    writing_ = false;
    writing_bytes_ = 0U;
}

}  // namespace ndt_slam
