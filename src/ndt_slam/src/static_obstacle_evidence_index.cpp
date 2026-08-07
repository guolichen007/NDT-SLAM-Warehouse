#include "ndt_slam/static_obstacle_evidence_index.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <utility>

namespace ndt_slam {
namespace {

constexpr double kStampEpsilonSec = 1.0e-4;
constexpr std::size_t kMaximumBoundedQueryCells = 65536U;

bool validConfig(const StaticObstacleEvidenceConfig& config) {
  return std::isfinite(config.cell_size_m) && config.cell_size_m > 0.0F &&
      config.minimum_observations > 0U &&
      std::isfinite(config.minimum_stable_duration_sec) &&
      config.minimum_stable_duration_sec >= 0.0 &&
      std::isfinite(config.minimum_cell_overlap) &&
      config.minimum_cell_overlap > 0.0F &&
      config.minimum_cell_overlap <= 1.0F &&
      std::isfinite(config.minimum_iou) && config.minimum_iou > 0.0F &&
      config.minimum_iou <= 1.0F &&
      std::isfinite(config.minimum_height_overlap) &&
      config.minimum_height_overlap > 0.0F &&
      config.minimum_height_overlap <= 1.0F &&
      std::isfinite(config.height_tolerance_m) &&
      config.height_tolerance_m >= 0.0F &&
      config.minimum_matched_cells > 0U &&
      config.maximum_query_area_cells >= config.minimum_matched_cells &&
      config.maximum_query_area_cells <= kMaximumBoundedQueryCells &&
      std::isfinite(config.immature_max_observation_gap_sec) &&
      config.immature_max_observation_gap_sec > 0.0 &&
      std::isfinite(config.immature_gap_retention_ratio) &&
      config.immature_gap_retention_ratio >= 0.0 &&
      config.immature_gap_retention_ratio < 1.0 &&
      config.maximum_observation_sequence_gap > 0U &&
      config.height_history_window >= 3U &&
      config.height_outlier_minimum_samples >= 3U &&
      config.height_outlier_minimum_samples <=
          config.height_history_window &&
      std::isfinite(config.height_outlier_mad_multiplier) &&
      config.height_outlier_mad_multiplier >= 1.0 &&
      std::isfinite(config.height_outlier_minimum_band_m) &&
      config.height_outlier_minimum_band_m > 0.0F;
}

float medianOf(std::vector<float> values) {
  if (values.empty()) return 0.0F;
  const std::size_t middle = values.size() / 2U;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  const float upper = values[middle];
  if ((values.size() & 1U) != 0U) return upper;
  std::nth_element(
      values.begin(), values.begin() + middle - 1U, values.end());
  return 0.5F * (values[middle - 1U] + upper);
}

float medianAbsoluteDeviation(
    const std::deque<float>& samples, float median) {
  std::vector<float> deviations;
  deviations.reserve(samples.size());
  for (const float sample : samples) {
    deviations.push_back(std::abs(sample - median));
  }
  return medianOf(std::move(deviations));
}

float intervalOverlapRatio(float query_min, float query_max,
                           float cell_min, float cell_max,
                           float tolerance_m) {
  if (!std::isfinite(query_min) || !std::isfinite(query_max) ||
      !std::isfinite(cell_min) || !std::isfinite(cell_max) ||
      query_max < query_min || cell_max < cell_min ||
      !std::isfinite(tolerance_m) || tolerance_m < 0.0F) {
    return 0.0F;
  }
  const float overlap = std::max(
      0.0F, std::min(query_max + tolerance_m,
                     cell_max + tolerance_m) -
                std::max(query_min - tolerance_m,
                         cell_min - tolerance_m));
  const float denominator = std::max(
      0.05F, std::min(query_max - query_min + 2.0F * tolerance_m,
                      cell_max - cell_min + 2.0F * tolerance_m));
  return std::clamp(overlap / denominator, 0.0F, 1.0F);
}

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  std::istringstream stream(line);
  while (std::getline(stream, field, ',')) fields.push_back(field);
  return fields;
}

}  // namespace

const char* staticEvidenceAuthorityName(
    StaticEvidenceAuthority authority) noexcept {
  switch (authority) {
    case StaticEvidenceAuthority::RUNTIME_MATURE:
      return "RUNTIME_MATURE";
    case StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE:
      return "OPERATOR_APPROVED_BASELINE";
    case StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN:
      return "UNVERIFIED_LOADED_CLEAN";
  }
  return "UNVERIFIED_LOADED_CLEAN";
}

std::int64_t packStaticEvidenceCell(
    std::int32_t x, std::int32_t y) noexcept {
  const std::uint64_t packed =
      (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U) |
      static_cast<std::uint32_t>(y);
  return static_cast<std::int64_t>(packed);
}

std::pair<std::int32_t, std::int32_t> unpackStaticEvidenceCell(
    std::int64_t key) noexcept {
  const std::uint64_t packed = static_cast<std::uint64_t>(key);
  return {
      static_cast<std::int32_t>(static_cast<std::uint32_t>(packed >> 32U)),
      static_cast<std::int32_t>(static_cast<std::uint32_t>(packed))};
}

StaticEvidenceCellGeometryMap buildStaticEvidenceCellGeometry(
    const std::vector<Eigen::Vector3f>& points, float cell_size_m) {
  StaticEvidenceCellGeometryMap cells;
  if (!std::isfinite(cell_size_m) || cell_size_m <= 0.0F) return cells;
  for (const Eigen::Vector3f& point : points) {
    if (!point.allFinite()) continue;
    const double scaled_x = static_cast<double>(point.x()) / cell_size_m;
    const double scaled_y = static_cast<double>(point.y()) / cell_size_m;
    if (scaled_x < std::numeric_limits<std::int32_t>::min() ||
        scaled_x > std::numeric_limits<std::int32_t>::max() ||
        scaled_y < std::numeric_limits<std::int32_t>::min() ||
        scaled_y > std::numeric_limits<std::int32_t>::max()) {
      continue;
    }
    const auto key = packStaticEvidenceCell(
        static_cast<std::int32_t>(std::floor(scaled_x)),
        static_cast<std::int32_t>(std::floor(scaled_y)));
    auto inserted = cells.emplace(
        key, StaticEvidenceCellGeometry{point.z(), point.z()});
    if (!inserted.second) {
      inserted.first->second.min_z =
          std::min(inserted.first->second.min_z, point.z());
      inserted.first->second.max_z =
          std::max(inserted.first->second.max_z, point.z());
    }
  }
  return cells;
}

StaticObstacleEvidenceIndex::StaticObstacleEvidenceIndex(
    const StaticObstacleEvidenceConfig& config) {
  setConfig(config);
}

void StaticObstacleEvidenceIndex::setConfig(
    const StaticObstacleEvidenceConfig& config) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = validConfig(config) ? config : StaticObstacleEvidenceConfig{};
  working_cells_.clear();
  height_histories_.clear();
  invalidated_versions_.clear();
  observed_free_tombstones_.clear();
  working_generation_ = 0U;
  latest_observation_sequence_ = 0U;
  revision_ = 0U;
  last_observation_stamp_sec_ = 0.0;
  authority_ = StaticEvidenceAuthority::RUNTIME_MATURE;
  diagnostics_totals_ = StaticEvidenceDiagnostics{};
  auto empty = std::make_shared<StaticEvidenceSnapshot>();
  empty->cell_size_m = config_.cell_size_m;
  std::atomic_store_explicit(
      &snapshot_, std::shared_ptr<const StaticEvidenceSnapshot>(empty),
      std::memory_order_release);
}

void StaticObstacleEvidenceIndex::reset(std::uint64_t map_generation) {
  std::lock_guard<std::mutex> lock(mutex_);
  working_cells_.clear();
  height_histories_.clear();
  invalidated_versions_.clear();
  observed_free_tombstones_.clear();
  working_generation_ = map_generation;
  latest_observation_sequence_ = 0U;
  last_observation_stamp_sec_ = 0.0;
  authority_ = StaticEvidenceAuthority::RUNTIME_MATURE;
  diagnostics_totals_ = StaticEvidenceDiagnostics{};
  publishSnapshotLocked(0.0);
}

bool StaticObstacleEvidenceIndex::updateHeightEstimateLocked(
    std::int64_t key,
    const StaticEvidenceCellGeometry& geometry,
    StaticEvidenceCell& cell,
    bool reset_history) {
  if (!std::isfinite(geometry.min_z) ||
      !std::isfinite(geometry.max_z) ||
      geometry.max_z < geometry.min_z) {
    ++diagnostics_totals_.height_invalid;
    return false;
  }

  HeightHistory& history = height_histories_[key];
  if (reset_history) {
    history.min_z.clear();
    history.max_z.clear();
  }

  const auto sample_is_consistent = [this](
      const std::deque<float>& samples, float sample) {
    if (samples.size() < config_.height_outlier_minimum_samples) {
      return true;
    }
    const std::vector<float> values(samples.begin(), samples.end());
    const float median = medianOf(values);
    const float mad = medianAbsoluteDeviation(samples, median);
    const double band = std::max(
        static_cast<double>(config_.height_outlier_minimum_band_m),
        config_.height_outlier_mad_multiplier *
            static_cast<double>(mad));
    return std::abs(static_cast<double>(sample - median)) <= band;
  };

  if (!sample_is_consistent(history.min_z, geometry.min_z) ||
      !sample_is_consistent(history.max_z, geometry.max_z)) {
    ++diagnostics_totals_.height_outliers_rejected;
    return false;
  }

  history.min_z.push_back(geometry.min_z);
  history.max_z.push_back(geometry.max_z);
  while (history.min_z.size() > config_.height_history_window) {
    history.min_z.pop_front();
  }
  while (history.max_z.size() > config_.height_history_window) {
    history.max_z.pop_front();
  }
  cell.min_z = medianOf(
      std::vector<float>(history.min_z.begin(), history.min_z.end()));
  cell.max_z = medianOf(
      std::vector<float>(history.max_z.begin(), history.max_z.end()));
  if (cell.max_z < cell.min_z) {
    const float midpoint = 0.5F * (cell.min_z + cell.max_z);
    cell.min_z = midpoint;
    cell.max_z = midpoint;
  }
  ++diagnostics_totals_.height_samples_accepted;
  return true;
}

void StaticObstacleEvidenceIndex::seedHeightHistoriesLocked() {
  height_histories_.clear();
  const std::size_t seed_count = std::max<std::size_t>(
      1U, config_.height_outlier_minimum_samples);
  for (const auto& item : working_cells_) {
    const StaticEvidenceCell& cell = item.second;
    if (!std::isfinite(cell.min_z) || !std::isfinite(cell.max_z) ||
        cell.max_z < cell.min_z) {
      continue;
    }
    HeightHistory& history = height_histories_[item.first];
    for (std::size_t index = 0U; index < seed_count; ++index) {
      history.min_z.push_back(cell.min_z);
      history.max_z.push_back(cell.max_z);
    }
  }
}

bool StaticObstacleEvidenceIndex::observeFilteredCells(
    const StaticEvidenceCellGeometryMap& cells,
    double stamp_sec,
    std::uint64_t map_generation,
    std::uint64_t objects_version) {
  StaticEvidenceCellKeySet observable;
  for (const auto& item : cells) observable.insert(item.first);
  return observeCleanBuildCells(
      cells, observable, {}, stamp_sec, map_generation, objects_version);
}

bool StaticObstacleEvidenceIndex::observeCleanBuildCells(
    const StaticEvidenceCellGeometryMap& occupied_cells,
    const StaticEvidenceCellKeySet& observable_cells,
    const StaticEvidenceCellKeySet& observed_free_cells,
    double stamp_sec,
    std::uint64_t map_generation,
    std::uint64_t objects_version) {
  if (!std::isfinite(stamp_sec) || stamp_sec <= 0.0) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  if (working_generation_ != map_generation) {
    // Only an explicit lifecycle reset may change the active epoch. A delayed
    // MapCommit from an older pose graph must never roll the index backwards.
    ++diagnostics_totals_.generation_mismatch;
    return false;
  }
  if (last_observation_stamp_sec_ > 0.0 &&
      stamp_sec <= last_observation_stamp_sec_ + kStampEpsilonSec) {
    if (stamp_sec + kStampEpsilonSec < last_observation_stamp_sec_) {
      // Source-time rollback starts a new timing epoch, but persisted map
      // observations remain valid. The independent sequence prevents replayed
      // stamps from being counted twice in one epoch.
      last_observation_stamp_sec_ = stamp_sec;
    }
    return false;
  }
  last_observation_stamp_sec_ = stamp_sec;
  ++latest_observation_sequence_;
  if (latest_observation_sequence_ == 0U) ++latest_observation_sequence_;
  bool snapshot_changed = false;

  // NOT_IN_VIEW deliberately preserves pending streaks. Only an explicitly
  // visible free cell is negative evidence.
  for (const auto& item : working_cells_) {
    if (observable_cells.find(item.first) == observable_cells.end()) {
      ++diagnostics_totals_.not_in_view;
    }
  }
  for (const auto key : observed_free_cells) {
    if (observable_cells.find(key) == observable_cells.end()) continue;
    const auto working = working_cells_.find(key);
    if (working == working_cells_.end()) continue;
    ++diagnostics_totals_.observed_free;
    observed_free_tombstones_.insert(key);
    working_cells_.erase(working);
    height_histories_.erase(key);
    snapshot_changed = true;
    ++diagnostics_totals_.invalidated_by_tombstone;
  }

  for (const auto& item : occupied_cells) {
    if (!observable_cells.empty() &&
        observable_cells.find(item.first) == observable_cells.end()) {
      continue;
    }
    if (observed_free_cells.find(item.first) != observed_free_cells.end()) {
      continue;
    }
    const auto& geometry = item.second;
    if (!std::isfinite(geometry.min_z) || !std::isfinite(geometry.max_z) ||
        geometry.max_z < geometry.min_z) {
      ++diagnostics_totals_.height_invalid;
      continue;
    }
    ++diagnostics_totals_.observed_occupied;
    observed_free_tombstones_.erase(item.first);
    auto inserted = working_cells_.emplace(item.first, StaticEvidenceCell{});
    StaticEvidenceCell& cell = inserted.first->second;
    if (inserted.second) {
      cell.key = item.first;
      cell.first_seen_sec = stamp_sec;
      cell.first_observation_sequence = latest_observation_sequence_;
      updateHeightEstimateLocked(
          item.first, geometry, cell, true);
    } else {
      if (!updateHeightEstimateLocked(
              item.first, geometry, cell, false)) {
        // A height outlier is not an independent stable observation. Keep the
        // mature estimate and do not advance temporal authorization.
        continue;
      }
      const double delta = stamp_sec - cell.last_seen_sec;
      const std::uint64_t sequence_gap =
          latest_observation_sequence_ > cell.last_observation_sequence
              ? latest_observation_sequence_ -
                    cell.last_observation_sequence
              : 0U;
      const bool continuous = std::isfinite(delta) &&
          delta > kStampEpsilonSec &&
          delta <= config_.immature_max_observation_gap_sec &&
          sequence_gap == 1U;
      if (continuous) {
        cell.consecutive_stable_duration_sec += delta;
      } else if (sequence_gap == 1U && !cell.temporally_mature) {
        // A warehouse revisit beyond the immature window weakens the previous
        // evidence instead of deleting it. The unobserved wall-clock gap is
        // never added to stable duration; explicit visible-free evidence above
        // still removes the cell immediately.
        const auto retained_count = static_cast<std::uint32_t>(std::floor(
            static_cast<double>(cell.consecutive_observation_count) *
            config_.immature_gap_retention_ratio));
        cell.consecutive_observation_count = retained_count;
        cell.consecutive_stable_duration_sec *=
            config_.immature_gap_retention_ratio;
        cell.first_seen_sec = stamp_sec;
        cell.first_observation_sequence = latest_observation_sequence_;
        if (retained_count == 0U) {
          ++diagnostics_totals_.reset_by_time_gap;
        } else {
          ++diagnostics_totals_.decayed_by_time_gap;
        }
      } else if (sequence_gap == 0U && !cell.temporally_mature) {
        cell.consecutive_observation_count = 0U;
        cell.consecutive_stable_duration_sec = 0.0;
        ++diagnostics_totals_.reset_by_sequence_gap;
      }
    }
    if (cell.consecutive_observation_count <
        std::numeric_limits<std::uint32_t>::max()) {
      ++cell.consecutive_observation_count;
    }
    if (cell.total_observation_count <
        std::numeric_limits<std::uint64_t>::max()) {
      ++cell.total_observation_count;
    }
    cell.last_seen_sec = stamp_sec;
    cell.last_observation_sequence = latest_observation_sequence_;
    cell.last_observed_objects_version = objects_version;
    cell.map_generation = map_generation;
    if (!cell.temporally_mature && isTemporallyMatureLocked(cell)) {
      cell.temporally_mature = true;
      snapshot_changed = true;
    }
  }
  if (snapshot_changed || !occupied_cells.empty() ||
      !observed_free_cells.empty()) {
    publishSnapshotLocked(stamp_sec);
  }
  return snapshot_changed;
}

StaticEvidenceMutationResult StaticObstacleEvidenceIndex::invalidateCells(
    const StaticEvidenceCellKeySet& invalidated_cells,
    std::uint64_t clean_build_version,
    double stamp_sec,
    std::uint64_t map_generation) {
  StaticEvidenceMutationResult result;
  if (clean_build_version == 0U) return result;
  std::lock_guard<std::mutex> lock(mutex_);
  if (working_generation_ != map_generation) {
    ++diagnostics_totals_.generation_mismatch;
    return result;
  }
  for (const auto key : invalidated_cells) {
    std::uint64_t& tombstone = invalidated_versions_[key];
    tombstone = std::max(tombstone, clean_build_version);
    result.invalidated_cells += working_cells_.erase(key);
    height_histories_.erase(key);
    ++diagnostics_totals_.invalidated_by_tombstone;
  }
  if (!invalidated_cells.empty()) publishSnapshotLocked(stamp_sec);
  const auto current = std::atomic_load_explicit(
      &snapshot_, std::memory_order_acquire);
  result.snapshot_cells = current ? current->cells.size() : 0U;
  result.revision = current ? current->revision : 0U;
  return result;
}

StaticEvidenceMutationResult StaticObstacleEvidenceIndex::confirmCleanCells(
    const StaticEvidenceCellGeometryMap& clean_cells,
    const StaticEvidenceCellKeySet& invalidated_cells,
    double stamp_sec,
    std::uint64_t map_generation,
    std::uint64_t clean_build_version) {
  StaticEvidenceMutationResult result;
  if (clean_build_version == 0U) return result;
  std::lock_guard<std::mutex> lock(mutex_);
  if (working_generation_ != map_generation) {
    ++diagnostics_totals_.generation_mismatch;
    return result;
  }
  // Invalidation is monotonic and always wins over clean evidence from the
  // same or any older asynchronous build.
  for (const auto key : invalidated_cells) {
    std::uint64_t& tombstone = invalidated_versions_[key];
    tombstone = std::max(tombstone, clean_build_version);
    result.invalidated_cells += working_cells_.erase(key);
    height_histories_.erase(key);
    ++diagnostics_totals_.invalidated_by_tombstone;
  }
  for (const auto& item : clean_cells) {
    if (!std::isfinite(item.second.min_z) ||
        !std::isfinite(item.second.max_z) ||
        item.second.max_z < item.second.min_z) {
      ++diagnostics_totals_.height_invalid;
      continue;
    }
    const auto tombstone = invalidated_versions_.find(item.first);
    if (observed_free_tombstones_.find(item.first) !=
            observed_free_tombstones_.end() ||
        invalidated_cells.find(item.first) != invalidated_cells.end() ||
        (tombstone != invalidated_versions_.end() &&
         tombstone->second >= clean_build_version)) {
      continue;
    }
    auto inserted = working_cells_.emplace(item.first, StaticEvidenceCell{});
    StaticEvidenceCell& cell = inserted.first->second;
    if (inserted.second) {
      // A loaded clean layer without observation history is deliberately not
      // authorized until independent map commits establish the minimum count.
      cell.key = item.first;
      cell.first_seen_sec = stamp_sec;
      cell.last_seen_sec = stamp_sec;
      cell.first_observation_sequence = latest_observation_sequence_;
      cell.last_observation_sequence = latest_observation_sequence_;
      updateHeightEstimateLocked(
          item.first, item.second, cell, true);
      cell.map_generation = map_generation;
    } else {
      // Confirmation may originate from the same asynchronous build as the
      // observation. Robust history absorbs duplicates and rejects isolated
      // vertical spikes without revoking a previously valid cell.
      updateHeightEstimateLocked(
          item.first, item.second, cell, false);
    }
    cell.clean_map_confirmed = true;
    cell.last_clean_confirmed_version = std::max(
        cell.last_clean_confirmed_version, clean_build_version);
    cell.last_invalidated_version = tombstone == invalidated_versions_.end()
        ? 0U : tombstone->second;
    if (!cell.temporally_mature && isTemporallyMatureLocked(cell)) {
      cell.temporally_mature = true;
    }
    ++result.confirmed_cells;
  }
  publishSnapshotLocked(stamp_sec);
  const auto current = std::atomic_load_explicit(
      &snapshot_, std::memory_order_acquire);
  result.snapshot_cells = current ? current->cells.size() : 0U;
  result.revision = current ? current->revision : 0U;
  return result;
}

bool StaticObstacleEvidenceIndex::isTemporallyMatureLocked(
    const StaticEvidenceCell& cell) const {
  return cell.clean_map_confirmed &&
      cell.map_generation == working_generation_ &&
      cell.consecutive_observation_count >= config_.minimum_observations &&
      cell.consecutive_stable_duration_sec + kStampEpsilonSec >=
          config_.minimum_stable_duration_sec;
}

void StaticObstacleEvidenceIndex::publishSnapshotLocked(
    double stamp_sec, bool increment_revision) {
  auto next = std::make_shared<StaticEvidenceSnapshot>();
  next->map_generation = working_generation_;
  if (increment_revision) {
    ++revision_;
    if (revision_ == 0U) ++revision_;
  }
  next->revision = revision_;
  next->latest_observation_sequence = latest_observation_sequence_;
  next->source_stamp_sec = stamp_sec;
  next->cell_size_m = config_.cell_size_m;
  next->authority = authority_;
  for (const auto& item : working_cells_) {
    if (item.second.clean_map_confirmed) {
      next->cells.emplace(item.first, item.second);
    }
  }
  std::atomic_store_explicit(
      &snapshot_, std::shared_ptr<const StaticEvidenceSnapshot>(next),
      std::memory_order_release);
}

std::shared_ptr<const StaticEvidenceSnapshot>
StaticObstacleEvidenceIndex::snapshot() const {
  return std::atomic_load_explicit(&snapshot_, std::memory_order_acquire);
}

std::uint64_t StaticObstacleEvidenceIndex::latestObservationSequence()
    const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_observation_sequence_;
}

std::size_t StaticObstacleEvidenceIndex::matureCellCount() const {
  const auto current = snapshot();
  if (!current) return 0U;
  return static_cast<std::size_t>(std::count_if(
      current->cells.begin(), current->cells.end(),
      [generation = current->map_generation](const auto& item) {
        return item.second.clean_map_confirmed &&
            item.second.temporally_mature &&
            item.second.map_generation == generation;
      }));
}

StaticEvidenceDiagnostics StaticObstacleEvidenceIndex::diagnostics() const {
  std::lock_guard<std::mutex> lock(mutex_);
  StaticEvidenceDiagnostics status = diagnostics_totals_;
  status.working_cells = working_cells_.size();
  for (const auto& item : working_cells_) {
    const StaticEvidenceCell& cell = item.second;
    if (cell.clean_map_confirmed) ++status.clean_confirmed_cells;
    if (cell.clean_map_confirmed && cell.temporally_mature) {
      ++status.temporally_mature_cells;
    } else if (cell.clean_map_confirmed) {
      if (cell.consecutive_observation_count < config_.minimum_observations) {
        ++status.pending_observation_count;
      }
      if (cell.consecutive_stable_duration_sec + kStampEpsilonSec <
          config_.minimum_stable_duration_sec) {
        ++status.pending_stable_duration;
      }
    }
    ++status.streak_histogram[cell.consecutive_observation_count];
  }
  return status;
}

void StaticObstacleEvidenceIndex::setSnapshotAuthority(
    StaticEvidenceAuthority authority) {
  std::lock_guard<std::mutex> lock(mutex_);
  authority_ = authority;
  publishSnapshotLocked(last_observation_stamp_sec_);
}

StaticProvenanceDecision StaticObstacleEvidenceIndex::query(
    const StaticProvenanceQuery& input) const {
  StaticProvenanceDecision decision;
  decision.expected_map_generation = input.expected_map_generation;
  decision.cargo_track_start_sequence =
      input.cargo_track_start_sequence;
  const auto current = snapshot();
  if (!current) return decision;
  decision.authority = current->authority;
  decision.map_generation = current->map_generation;
  decision.index_revision = current->revision;
  decision.index_latest_observation_sequence =
      current->latest_observation_sequence;
  decision.index_cell_count = current->cells.size();
  if (current->cells.empty()) return decision;
  if (current->map_generation != input.expected_map_generation) {
    decision.reason = "static_index_generation_mismatch";
    return decision;
  }
  if (current->authority ==
      StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN) {
    decision.reason = "static_index_unverified_loaded_clean";
    return decision;
  }
  if (!std::isfinite(input.min_z_map) || !std::isfinite(input.max_z_map) ||
      input.max_z_map < input.min_z_map || input.occupied_map_cells.empty()) {
    decision.reason = "static_index_query_invalid";
    return decision;
  }

  std::set<std::int64_t> query_cells(
      input.occupied_map_cells.begin(), input.occupied_map_cells.end());
  decision.query_cell_count = query_cells.size();
  std::int32_t min_x = std::numeric_limits<std::int32_t>::max();
  std::int32_t max_x = std::numeric_limits<std::int32_t>::min();
  std::int32_t min_y = std::numeric_limits<std::int32_t>::max();
  std::int32_t max_y = std::numeric_limits<std::int32_t>::min();
  for (const auto key : query_cells) {
    const auto xy = unpackStaticEvidenceCell(key);
    min_x = std::min(min_x, xy.first);
    max_x = std::max(max_x, xy.first);
    min_y = std::min(min_y, xy.second);
    max_y = std::max(max_y, xy.second);
  }
  const std::int64_t width =
      static_cast<std::int64_t>(max_x) - min_x + 1LL;
  const std::int64_t height =
      static_cast<std::int64_t>(max_y) - min_y + 1LL;
  if (width <= 0LL || height <= 0LL ||
      width > static_cast<std::int64_t>(
          config_.maximum_query_area_cells) ||
      height > static_cast<std::int64_t>(
          config_.maximum_query_area_cells) ||
      width * height > static_cast<std::int64_t>(
          config_.maximum_query_area_cells)) {
    decision.reason = "static_index_query_area_exceeded";
    return decision;
  }

  std::size_t matched = 0U;
  std::size_t pre_cargo_matched = 0U;
  std::size_t local_evidence = 0U;
  std::size_t local_pre_cargo = 0U;
  std::size_t spatially_matched = 0U;
  float minimum_height_ratio = 1.0F;
  std::uint32_t minimum_observations =
      std::numeric_limits<std::uint32_t>::max();
  double minimum_stable_age = std::numeric_limits<double>::infinity();
  for (std::int64_t x = min_x; x <= max_x; ++x) {
    for (std::int64_t y = min_y; y <= max_y; ++y) {
      const auto key = packStaticEvidenceCell(
          static_cast<std::int32_t>(x), static_cast<std::int32_t>(y));
      const auto found = current->cells.find(key);
      if (found == current->cells.end()) continue;
      const StaticEvidenceCell& cell = found->second;
      const float height_ratio = intervalOverlapRatio(
          input.min_z_map, input.max_z_map, cell.min_z, cell.max_z,
          config_.height_tolerance_m);
      const bool operator_approved = current->authority ==
          StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE;
      const bool authority_allowed = current->authority !=
          StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN;
      const bool stable_history = cell.clean_map_confirmed &&
          authority_allowed && (cell.temporally_mature || operator_approved) &&
          cell.map_generation == input.expected_map_generation &&
          (operator_approved ||
           (cell.consecutive_observation_count >=
                config_.minimum_observations &&
            cell.consecutive_stable_duration_sec + kStampEpsilonSec >=
                config_.minimum_stable_duration_sec));
      if (stable_history && query_cells.find(key) != query_cells.end()) {
        ++spatially_matched;
      }
      const bool stable = stable_history &&
          height_ratio >= config_.minimum_height_overlap;
      if (!stable) continue;
      ++local_evidence;
      const bool pre_cargo = input.cargo_track_start_sequence > 0U &&
          cell.first_observation_sequence > 0U &&
          cell.first_observation_sequence <=
              input.cargo_track_start_sequence &&
          input.cargo_track_start_sequence -
                  cell.first_observation_sequence >=
              config_.pre_cargo_minimum_sequence_gap;
      if (pre_cargo) ++local_pre_cargo;
      if (query_cells.find(key) == query_cells.end()) continue;
      ++matched;
      if (pre_cargo) ++pre_cargo_matched;
      minimum_height_ratio = std::min(minimum_height_ratio, height_ratio);
      minimum_observations = std::min(
          minimum_observations, cell.consecutive_observation_count);
      minimum_stable_age = std::min(
          minimum_stable_age, cell.consecutive_stable_duration_sec);
    }
  }

  const auto ratio = [denominator = query_cells.size()](std::size_t count) {
    return denominator == 0U ? 0.0F :
        static_cast<float>(count) / static_cast<float>(denominator);
  };
  const auto iou = [query_size = query_cells.size()](
                       std::size_t intersection,
                       std::size_t evidence_size) {
    const std::size_t union_size =
        query_size + evidence_size - intersection;
    return union_size == 0U ? 0.0F :
        static_cast<float>(intersection) / static_cast<float>(union_size);
  };
  decision.matched_cell_ratio = ratio(matched);
  decision.matched_iou = iou(matched, local_evidence);
  decision.matched_cell_count = matched;
  decision.spatially_matched_cell_count = spatially_matched;
  decision.height_overlap = matched > 0U ? minimum_height_ratio : 0.0F;
  decision.stable_observation_count = matched > 0U
      ? minimum_observations : 0U;
  decision.stable_age_sec = matched > 0U ? minimum_stable_age : 0.0;

  const float pre_ratio = ratio(pre_cargo_matched);
  const float pre_iou = iou(pre_cargo_matched, local_pre_cargo);
  const bool enough_pre_cargo =
      pre_cargo_matched >= config_.minimum_matched_cells &&
      pre_ratio >= config_.minimum_cell_overlap;
  if (enough_pre_cargo) {
    decision.provenance = ExternalProvenance::PRE_CARGO_OCCUPANCY;
    decision.authorized = true;
    decision.matched_cell_ratio = pre_ratio;
    decision.matched_iou = pre_iou;
    decision.reason = "pre_cargo_occupancy_confirmed";
    return decision;
  }
  const bool enough_static_map =
      matched >= config_.minimum_matched_cells &&
      decision.matched_cell_ratio >= config_.minimum_cell_overlap;
  if (enough_static_map) {
    decision.provenance = ExternalProvenance::STATIC_MAP_MATCH;
    decision.authorized = true;
    decision.reason = current->authority ==
            StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE
        ? "operator_approved_static_map_match"
        : "static_map_match_confirmed";
    return decision;
  }
  if (spatially_matched > 0U && matched == 0U) {
    decision.reason = "static_map_height_mismatch";
  } else if (matched < config_.minimum_matched_cells) {
    decision.reason = "static_map_insufficient_matched_cells";
  } else {
    decision.reason = "static_map_query_coverage_below_threshold";
  }
  return decision;
}

bool StaticObstacleEvidenceIndex::saveSnapshot(
    const std::shared_ptr<const StaticEvidenceSnapshot>& current,
    const std::string& path, std::string* reason) const {
  if (!current) {
    if (reason) *reason = "snapshot_unavailable";
    return false;
  }
  const std::string temporary = path + ".tmp";
  std::ofstream output(temporary, std::ios::out | std::ios::trunc);
  if (!output.is_open()) {
    if (reason) *reason = "open_failed";
    return false;
  }
  output << "NDT_STATIC_EVIDENCE_INDEX," << current->schema_version << ','
         << std::setprecision(9) << current->cell_size_m << ','
         << current->map_generation << ',' << current->revision << ','
         << current->latest_observation_sequence << ','
         << std::setprecision(17) << current->source_stamp_sec << ','
         << static_cast<unsigned int>(current->authority) << '\n';
  for (const auto& item : current->cells) {
    const StaticEvidenceCell& cell = item.second;
    output << cell.key << ',' << cell.consecutive_observation_count << ','
           << cell.total_observation_count << ','
           << std::setprecision(17) << cell.first_seen_sec << ','
           << cell.last_seen_sec << ','
           << cell.consecutive_stable_duration_sec << ','
           << std::setprecision(9) << cell.min_z << ',' << cell.max_z << ','
           << cell.first_observation_sequence << ','
           << cell.last_observation_sequence << ','
           << cell.last_observed_objects_version << ','
           << cell.last_clean_confirmed_version << ','
           << cell.last_invalidated_version << ','
           << (cell.temporally_mature ? 1 : 0) << '\n';
  }
  output.flush();
  const bool stream_ok = output.good();
  output.close();
  if (!stream_ok || std::rename(temporary.c_str(), path.c_str()) != 0) {
    std::remove(temporary.c_str());
    if (reason) *reason = "atomic_replace_failed";
    return false;
  }
  if (reason) *reason = "saved";
  return true;
}

bool StaticObstacleEvidenceIndex::loadSnapshotCandidate(
    const std::string& path,
    std::uint64_t expected_source_generation,
    std::uint64_t expected_revision,
    StaticEvidenceSnapshot* candidate,
    std::string* reason) const {
  if (!candidate) {
    if (reason) *reason = "candidate_output_missing";
    return false;
  }
  std::ifstream input(path);
  if (!input.is_open()) {
    if (reason) *reason = "index_missing";
    return false;
  }
  std::string line;
  if (!std::getline(input, line)) {
    if (reason) *reason = "index_header_missing";
    return false;
  }
  const auto header = splitCsv(line);
  if ((header.size() != 7U && header.size() != 8U) ||
      header[0] != "NDT_STATIC_EVIDENCE_INDEX") {
    if (reason) *reason = "index_header_invalid";
    return false;
  }
  try {
    const auto schema = static_cast<std::uint32_t>(std::stoul(header[1]));
    const float cell_size = std::stof(header[2]);
    const std::uint64_t source_generation = std::stoull(header[3]);
    const std::uint64_t source_revision = std::stoull(header[4]);
    StaticEvidenceAuthority loaded_authority =
        StaticEvidenceAuthority::RUNTIME_MATURE;
    if (header.size() == 8U) {
      const unsigned long parsed_authority = std::stoul(header[7]);
      if (parsed_authority > static_cast<unsigned long>(
              StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN)) {
        if (reason) *reason = "index_authority_invalid";
        return false;
      }
      loaded_authority = static_cast<StaticEvidenceAuthority>(
          parsed_authority);
    }
    if (schema != StaticEvidenceSnapshot::kSchemaVersion ||
        std::abs(cell_size - config_.cell_size_m) > 1.0e-5F ||
        source_generation == 0U || source_revision == 0U ||
        source_generation != expected_source_generation ||
        source_revision != expected_revision) {
      if (reason) *reason = "index_schema_or_resolution_mismatch";
      return false;
    }
    std::map<std::int64_t, StaticEvidenceCell> loaded;
    std::uint64_t latest_sequence = std::stoull(header[5]);
    const double source_stamp_sec = std::stod(header[6]);
    if (!std::isfinite(source_stamp_sec) || source_stamp_sec < 0.0) {
      if (reason) *reason = "index_source_stamp_invalid";
      return false;
    }
    while (std::getline(input, line)) {
      if (line.empty()) continue;
      const auto fields = splitCsv(line);
      if (fields.size() != 14U) {
        if (reason) *reason = "index_cell_invalid";
        return false;
      }
      StaticEvidenceCell cell;
      cell.key = std::stoll(fields[0]);
      const unsigned long parsed_observation_count = std::stoul(fields[1]);
      if (parsed_observation_count >
          std::numeric_limits<std::uint32_t>::max()) {
        if (reason) *reason = "index_observation_count_invalid";
        return false;
      }
      cell.consecutive_observation_count =
          static_cast<std::uint32_t>(parsed_observation_count);
      cell.total_observation_count = std::stoull(fields[2]);
      cell.first_seen_sec = std::stod(fields[3]);
      cell.last_seen_sec = std::stod(fields[4]);
      cell.consecutive_stable_duration_sec = std::stod(fields[5]);
      cell.min_z = std::stof(fields[6]);
      cell.max_z = std::stof(fields[7]);
      cell.first_observation_sequence = std::stoull(fields[8]);
      cell.last_observation_sequence = std::stoull(fields[9]);
      cell.last_observed_objects_version = std::stoull(fields[10]);
      cell.last_clean_confirmed_version = std::stoull(fields[11]);
      cell.last_invalidated_version = std::stoull(fields[12]);
      const unsigned long mature_value = std::stoul(fields[13]);
      if (mature_value > 1UL) {
        if (reason) *reason = "index_temporal_maturity_invalid";
        return false;
      }
      cell.temporally_mature = mature_value == 1UL;
      cell.clean_map_confirmed = true;
      cell.map_generation = source_generation;
      if (!std::isfinite(cell.first_seen_sec) ||
          !std::isfinite(cell.last_seen_sec) ||
          !std::isfinite(cell.consecutive_stable_duration_sec) ||
          !std::isfinite(cell.min_z) || !std::isfinite(cell.max_z) ||
          cell.consecutive_stable_duration_sec < 0.0 ||
          cell.max_z < cell.min_z ||
          cell.consecutive_observation_count == 0U ||
          cell.total_observation_count <
              cell.consecutive_observation_count ||
          cell.last_observation_sequence <
              cell.first_observation_sequence ||
          (cell.temporally_mature &&
           (cell.consecutive_observation_count <
                config_.minimum_observations ||
            cell.consecutive_stable_duration_sec + kStampEpsilonSec <
                config_.minimum_stable_duration_sec))) {
        if (reason) *reason = "index_cell_nonfinite";
        return false;
      }
      if (!loaded.emplace(cell.key, cell).second) {
        if (reason) *reason = "index_duplicate_cell";
        return false;
      }
      latest_sequence = std::max(
          latest_sequence, cell.last_observation_sequence);
    }
    candidate->schema_version = schema;
    candidate->map_generation = source_generation;
    candidate->revision = source_revision;
    candidate->latest_observation_sequence = latest_sequence;
    candidate->source_stamp_sec = source_stamp_sec;
    candidate->cell_size_m = cell_size;
    candidate->authority = loaded_authority;
    candidate->cells = std::move(loaded);
  } catch (const std::exception&) {
    if (reason) *reason = "index_parse_failed";
    return false;
  }
  if (reason) *reason = "candidate_loaded";
  return true;
}

bool StaticObstacleEvidenceIndex::restoreSnapshotWithoutRevisionIncrement(
    const StaticEvidenceSnapshot& candidate,
    std::uint64_t current_map_generation,
    std::string* reason) {
  PreparedStaticEvidenceInstall prepared;
  if (!prepareSnapshotInstall(
          candidate, current_map_generation, &prepared, reason)) {
    return false;
  }
  installPreparedSnapshot(std::move(prepared));
  if (reason) *reason = "restored_exact_revision";
  return true;
}

bool StaticObstacleEvidenceIndex::prepareSnapshotInstall(
    const StaticEvidenceSnapshot& candidate,
    std::uint64_t current_map_generation,
    PreparedStaticEvidenceInstall* prepared,
    std::string* reason) const {
  if (!prepared) {
    if (reason) *reason = "prepared_output_missing";
    return false;
  }
  *prepared = PreparedStaticEvidenceInstall{};
  if (candidate.schema_version != StaticEvidenceSnapshot::kSchemaVersion ||
      std::abs(candidate.cell_size_m - config_.cell_size_m) > 1.0e-5F ||
      candidate.revision == 0U || current_map_generation == 0U) {
    if (reason) *reason = "candidate_schema_or_generation_invalid";
    return false;
  }
  prepared->working_cells = candidate.cells;
  for (auto& item : prepared->working_cells) {
    item.second.map_generation = current_map_generation;
  }
  auto immutable = std::make_shared<StaticEvidenceSnapshot>(candidate);
  immutable->map_generation = current_map_generation;
  for (auto& item : immutable->cells) {
    item.second.map_generation = current_map_generation;
  }
  prepared->valid = true;
  prepared->map_generation = current_map_generation;
  prepared->latest_observation_sequence =
      candidate.latest_observation_sequence;
  prepared->revision = candidate.revision;
  prepared->authority = candidate.authority;
  prepared->snapshot = std::move(immutable);
  if (reason) *reason = "snapshot_install_prepared";
  return true;
}

void StaticObstacleEvidenceIndex::installPreparedSnapshot(
    PreparedStaticEvidenceInstall&& prepared,
    std::uint64_t current_map_generation) noexcept {
  if (!prepared.valid || !prepared.snapshot) return;
  if (current_map_generation != 0U &&
      current_map_generation != prepared.map_generation) {
    prepared.map_generation = current_map_generation;
    prepared.snapshot->map_generation = current_map_generation;
    for (auto& item : prepared.working_cells) {
      item.second.map_generation = current_map_generation;
    }
    for (auto& item : prepared.snapshot->cells) {
      item.second.map_generation = current_map_generation;
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  working_cells_.swap(prepared.working_cells);
  seedHeightHistoriesLocked();
  invalidated_versions_.clear();
  observed_free_tombstones_.clear();
  working_generation_ = prepared.map_generation;
  authority_ = prepared.authority;
  latest_observation_sequence_ = prepared.latest_observation_sequence;
  revision_ = prepared.revision;
  last_observation_stamp_sec_ = 0.0;
  std::shared_ptr<const StaticEvidenceSnapshot> immutable =
      std::move(prepared.snapshot);
  std::atomic_store(&snapshot_, std::move(immutable));
}

bool StaticObstacleEvidenceIndex::loadSnapshot(
    const std::string& path,
    std::uint64_t current_map_generation,
    std::uint64_t expected_source_generation,
    std::uint64_t expected_revision,
    std::string* reason) {
  StaticEvidenceSnapshot candidate;
  if (!loadSnapshotCandidate(
          path, expected_source_generation, expected_revision,
          &candidate, reason)) {
    return false;
  }
  return restoreSnapshotWithoutRevisionIncrement(
      candidate, current_map_generation, reason);
}

}  // namespace ndt_slam
