#include "ndt_slam/accepted_keyframe_journal.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <utility>
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
constexpr std::uint32_t kMagic = 0x4e444a4bU;  // NDJK
constexpr std::uint32_t kCommit = 0x434f4d54U;  // COMT
constexpr std::uint32_t kVersion = 1U;
constexpr std::uint64_t kMaxPayload = 64U * 1024U * 1024U;

template <typename T>
void appendScalar(std::vector<char>* output, const T& value) {
  const char* bytes = reinterpret_cast<const char*>(&value);
  output->insert(output->end(), bytes, bytes + sizeof(T));
}

template <typename T>
bool readScalar(const std::vector<char>& input, std::size_t* offset, T* value) {
  if (*offset > input.size() || input.size() - *offset < sizeof(T)) return false;
  std::memcpy(value, input.data() + *offset, sizeof(T));
  *offset += sizeof(T);
  return true;
}

std::uint64_t checksum(const std::vector<char>& value) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char character : value) {
    hash ^= character;
    hash *= 1099511628211ULL;
  }
  return hash;
}

bool recordValid(const AcceptedKeyframeRecord& record) {
  return record.sequence > 0U && !record.map_uuid.empty() &&
      record.continuity_generation > 0U && record.pose_generation > 0U &&
      std::isfinite(record.source_stamp_sec) && record.source_stamp_sec > 0.0 &&
      std::isfinite(record.x) && std::isfinite(record.y) &&
      std::isfinite(record.z) && std::isfinite(record.yaw) &&
      record.registration_cloud && !record.registration_cloud->empty();
}

std::vector<char> serialize(const AcceptedKeyframeRecord& record) {
  std::vector<char> payload;
  payload.reserve(128U + record.map_uuid.size() +
                  record.registration_cloud->size() * 3U * sizeof(float));
  appendScalar(&payload, record.sequence);
  appendScalar(&payload, record.map_generation);
  appendScalar(&payload, record.continuity_generation);
  appendScalar(&payload, record.pose_generation);
  appendScalar(&payload, record.source_stamp_sec);
  appendScalar(&payload, record.x);
  appendScalar(&payload, record.y);
  appendScalar(&payload, record.z);
  appendScalar(&payload, record.yaw);
  const std::uint32_t uuid_size =
      static_cast<std::uint32_t>(record.map_uuid.size());
  appendScalar(&payload, uuid_size);
  payload.insert(payload.end(), record.map_uuid.begin(), record.map_uuid.end());
  const std::uint64_t points = record.registration_cloud->size();
  appendScalar(&payload, points);
  for (const auto& point : record.registration_cloud->points) {
    appendScalar(&payload, point.x);
    appendScalar(&payload, point.y);
    appendScalar(&payload, point.z);
  }
  return payload;
}

bool deserialize(const std::vector<char>& payload,
                 AcceptedKeyframeRecord* record) {
  std::size_t offset = 0U;
  if (!readScalar(payload, &offset, &record->sequence) ||
      !readScalar(payload, &offset, &record->map_generation) ||
      !readScalar(payload, &offset, &record->continuity_generation) ||
      !readScalar(payload, &offset, &record->pose_generation) ||
      !readScalar(payload, &offset, &record->source_stamp_sec) ||
      !readScalar(payload, &offset, &record->x) ||
      !readScalar(payload, &offset, &record->y) ||
      !readScalar(payload, &offset, &record->z) ||
      !readScalar(payload, &offset, &record->yaw)) {
    return false;
  }
  std::uint32_t uuid_size = 0U;
  if (!readScalar(payload, &offset, &uuid_size) || uuid_size > 4096U ||
      offset > payload.size() || payload.size() - offset < uuid_size) {
    return false;
  }
  record->map_uuid.assign(payload.data() + offset, uuid_size);
  offset += uuid_size;
  std::uint64_t points = 0U;
  if (!readScalar(payload, &offset, &points) || points > 5000000U ||
      points > (payload.size() - offset) / (3U * sizeof(float))) {
    return false;
  }
  record->registration_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  record->registration_cloud->reserve(static_cast<std::size_t>(points));
  for (std::uint64_t index = 0U; index < points; ++index) {
    pcl::PointXYZ point;
    if (!readScalar(payload, &offset, &point.x) ||
        !readScalar(payload, &offset, &point.y) ||
        !readScalar(payload, &offset, &point.z)) {
      return false;
    }
    record->registration_cloud->push_back(point);
  }
  return offset == payload.size() && recordValid(*record);
}

bool fsyncFile(const fs::path& path) {
#ifdef _WIN32
  const int descriptor = _wopen(path.c_str(), _O_RDONLY | _O_BINARY);
  if (descriptor < 0) return false;
  const int result = _commit(descriptor);
  _close(descriptor);
#else
  const int descriptor = ::open(path.c_str(), O_RDONLY);
  if (descriptor < 0) return false;
  const int result = ::fsync(descriptor);
  ::close(descriptor);
#endif
  return result == 0;
}

bool appendRecord(const fs::path& path, const AcceptedKeyframeRecord& record) {
  const auto payload = serialize(record);
  const std::uint64_t payload_size = payload.size();
  const std::uint64_t digest = checksum(payload);
  std::ofstream output(path, std::ios::binary | std::ios::app);
  if (!output.is_open()) return false;
  output.write(reinterpret_cast<const char*>(&kMagic), sizeof(kMagic));
  output.write(reinterpret_cast<const char*>(&kVersion), sizeof(kVersion));
  output.write(reinterpret_cast<const char*>(&payload_size), sizeof(payload_size));
  output.write(reinterpret_cast<const char*>(&digest), sizeof(digest));
  output.write(payload.data(), static_cast<std::streamsize>(payload.size()));
  output.write(reinterpret_cast<const char*>(&kCommit), sizeof(kCommit));
  output.flush();
  const bool good = output.good();
  output.close();
  return good && fsyncFile(path);
}

std::size_t estimatedBytes(const AcceptedKeyframeRecord& record) {
  return 128U + record.map_uuid.size() +
      (record.registration_cloud ? record.registration_cloud->size() * 12U : 0U);
}

}  // namespace

AcceptedKeyframeJournal::~AcceptedKeyframeJournal() { stop(); }

void AcceptedKeyframeJournal::configure(
    const AcceptedKeyframeJournalConfig& config) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (started_) return;
  config_ = config;
  config_.max_jobs = std::max<std::size_t>(1U, config_.max_jobs);
  config_.max_bytes = std::max<std::size_t>(4096U, config_.max_bytes);
}

bool AcceptedKeyframeJournal::start(std::string* reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (started_) return true;
  if (config_.path.empty()) {
    if (reason) *reason = "journal_path_empty";
    return false;
  }
  try {
    const auto parent = fs::absolute(config_.path).parent_path();
    fs::create_directories(parent);
    // Durable journal creation: the parent directory entry must be
    // persisted before any record can claim durability.
    fsyncFile(parent);
  } catch (const std::exception& error) {
    if (reason) *reason = error.what();
    return false;
  }
  stopping_ = false;
  started_ = true;
  worker_ = std::thread(&AcceptedKeyframeJournal::workerLoop, this);
  if (reason) *reason = "started";
  return true;
}

void AcceptedKeyframeJournal::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!started_) return;
    stopping_ = true;
  }
  cv_.notify_all();
  if (worker_.joinable()) worker_.join();
  std::lock_guard<std::mutex> lock(mutex_);
  started_ = false;
}

bool AcceptedKeyframeJournal::submit(AcceptedKeyframeRecord record,
                                     std::string* reason) {
  if (!record.map_write_authorized || !record.accepted || record.prediction ||
      record.quarantined || record.stale || !recordValid(record)) {
    if (reason) *reason = "journal_evidence_rejected";
    return false;
  }
  const std::size_t bytes = estimatedBytes(record);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!started_ || stopping_ || jobs_.size() >= config_.max_jobs ||
      bytes > config_.max_bytes || queued_bytes_ > config_.max_bytes - bytes) {
    ++dropped_records_;
    if (reason) *reason = "journal_queue_full";
    return false;
  }
  queued_bytes_ += bytes;
  jobs_.push_back(std::move(record));
  cv_.notify_one();
  if (reason) *reason = "queued";
  return true;
}

std::uint64_t AcceptedKeyframeJournal::writtenRecords() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return written_records_;
}

std::uint64_t AcceptedKeyframeJournal::droppedRecords() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return dropped_records_;
}

void AcceptedKeyframeJournal::workerLoop() {
  while (true) {
    AcceptedKeyframeRecord record;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this] { return stopping_ || !jobs_.empty(); });
      if (jobs_.empty() && stopping_) break;
      record = std::move(jobs_.front());
      queued_bytes_ -= estimatedBytes(record);
      jobs_.pop_front();
    }
    const bool written = appendRecord(config_.path, record);
    std::lock_guard<std::mutex> lock(mutex_);
    if (written) ++written_records_;
    else ++dropped_records_;
  }
}

AcceptedKeyframeJournalLoadResult
AcceptedKeyframeJournal::loadLastVerified(
    const std::string& path, const std::string& expected_map_uuid,
    std::uint64_t expected_map_generation, bool truncate_partial_tail) {
  AcceptedKeyframeJournalLoadResult result;
  std::ifstream input(path, std::ios::binary);
  if (!input.is_open()) {
    result.reason = "journal_missing";
    return result;
  }
  std::uintmax_t verified_end = 0U;
  while (true) {
    const auto start = input.tellg();
    std::uint32_t magic = 0U;
    input.read(reinterpret_cast<char*>(&magic), sizeof(magic));
    if (input.eof() && input.gcount() == 0) break;
    std::uint32_t version = 0U;
    std::uint64_t payload_size = 0U;
    std::uint64_t digest = 0U;
    input.read(reinterpret_cast<char*>(&version), sizeof(version));
    input.read(reinterpret_cast<char*>(&payload_size), sizeof(payload_size));
    input.read(reinterpret_cast<char*>(&digest), sizeof(digest));
    if (!input.good() || magic != kMagic || version != kVersion ||
        payload_size > kMaxPayload) {
      result.truncated_tail = true;
      break;
    }
    std::vector<char> payload(static_cast<std::size_t>(payload_size));
    input.read(payload.data(), static_cast<std::streamsize>(payload.size()));
    std::uint32_t commit = 0U;
    input.read(reinterpret_cast<char*>(&commit), sizeof(commit));
    AcceptedKeyframeRecord record;
    if (!input.good() || commit != kCommit || checksum(payload) != digest ||
        !deserialize(payload, &record)) {
      result.truncated_tail = true;
      break;
    }
    const auto end = input.tellg();
    verified_end = end < 0 ? verified_end : static_cast<std::uintmax_t>(end);
    ++result.verified_records;
    if (record.map_uuid == expected_map_uuid &&
        record.map_generation == expected_map_generation) {
      result.record = std::move(record);
      result.valid = true;
    }
    (void)start;
  }
  input.close();
  if (result.truncated_tail && truncate_partial_tail) {
    std::error_code ignored;
    fs::resize_file(path, verified_end, ignored);
    // The truncation itself must be durable before the repaired journal
    // can be trusted by crash recovery.
    if (!ignored) fsyncFile(path);
  }
  result.reason = result.valid ? "verified_recovery_reference"
                               : "no_matching_verified_record";
  return result;
}

}  // namespace ndt_slam
