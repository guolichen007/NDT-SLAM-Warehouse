#pragma once

#include "ndt_slam/cargo_obstacle_tracker.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace ndt_slam {

struct StaticObstacleEvidenceConfig {
  float cell_size_m = 0.25F;
  std::uint32_t minimum_observations = 4U;
  double minimum_stable_duration_sec = 1.0;
  float minimum_cell_overlap = 0.60F;
  float minimum_iou = 0.45F;
  float minimum_height_overlap = 0.50F;
  float height_tolerance_m = 0.10F;
  std::size_t minimum_matched_cells = 6U;
  std::size_t maximum_query_area_cells = 4096U;
  std::uint64_t pre_cargo_minimum_sequence_gap = 2U;
  double maximum_observation_gap_sec = 5.0;
  std::uint64_t maximum_observation_sequence_gap = 2U;
};

struct StaticEvidenceCellGeometry {
  float min_z = 0.0F;
  float max_z = 0.0F;
};

using StaticEvidenceCellGeometryMap =
    std::map<std::int64_t, StaticEvidenceCellGeometry>;
using StaticEvidenceCellKeySet = std::set<std::int64_t>;

struct StaticEvidenceCell {
  std::int64_t key = 0;
  std::uint32_t consecutive_observation_count = 0U;
  std::uint64_t total_observation_count = 0U;
  double first_seen_sec = 0.0;
  double last_seen_sec = 0.0;
  double consecutive_stable_duration_sec = 0.0;
  float min_z = 0.0F;
  float max_z = 0.0F;
  bool clean_map_confirmed = false;
  std::uint64_t first_observation_sequence = 0U;
  std::uint64_t last_observation_sequence = 0U;
  std::uint64_t last_observed_objects_version = 0U;
  std::uint64_t last_clean_confirmed_version = 0U;
  std::uint64_t last_invalidated_version = 0U;
  std::uint64_t map_generation = 0U;
};

struct StaticEvidenceSnapshot {
  static constexpr std::uint32_t kSchemaVersion = 2U;

  std::uint32_t schema_version = kSchemaVersion;
  std::uint64_t map_generation = 0U;
  std::uint64_t revision = 0U;
  std::uint64_t latest_observation_sequence = 0U;
  double source_stamp_sec = 0.0;
  float cell_size_m = 0.25F;
  std::map<std::int64_t, StaticEvidenceCell> cells;
};

struct StaticProvenanceQuery {
  std::vector<std::int64_t> occupied_map_cells;
  float min_z_map = 0.0F;
  float max_z_map = 0.0F;
  std::uint64_t cargo_track_start_sequence = 0U;
  std::uint64_t expected_map_generation = 0U;
};

struct StaticProvenanceDecision {
  ExternalProvenance provenance = ExternalProvenance::NONE;
  bool authorized = false;
  float matched_cell_ratio = 0.0F;
  float matched_iou = 0.0F;
  float height_overlap = 0.0F;
  std::uint32_t stable_observation_count = 0U;
  double stable_age_sec = 0.0;
  std::uint64_t map_generation = 0U;
  std::uint64_t expected_map_generation = 0U;
  std::uint64_t index_revision = 0U;
  std::uint64_t index_latest_observation_sequence = 0U;
  std::uint64_t cargo_track_start_sequence = 0U;
  std::size_t index_cell_count = 0U;
  std::size_t query_cell_count = 0U;
  std::size_t matched_cell_count = 0U;
  std::size_t spatially_matched_cell_count = 0U;
  std::string reason = "static_index_empty";
};

struct StaticEvidenceMutationResult {
  std::size_t confirmed_cells = 0U;
  std::size_t invalidated_cells = 0U;
  std::size_t snapshot_cells = 0U;
  std::uint64_t revision = 0U;
};

std::int64_t packStaticEvidenceCell(std::int32_t x, std::int32_t y) noexcept;
std::pair<std::int32_t, std::int32_t> unpackStaticEvidenceCell(
    std::int64_t key) noexcept;

StaticEvidenceCellGeometryMap buildStaticEvidenceCellGeometry(
    const std::vector<Eigen::Vector3f>& points, float cell_size_m);

// MapCommit and map maintenance serialize mutations through the internal
// mutex. Safety queries use an immutable atomic snapshot and never scan a PCD
// file or persistent tile.
class StaticObstacleEvidenceIndex {
 public:
  explicit StaticObstacleEvidenceIndex(
      const StaticObstacleEvidenceConfig& config =
          StaticObstacleEvidenceConfig());

  void setConfig(const StaticObstacleEvidenceConfig& config);
  const StaticObstacleEvidenceConfig& config() const noexcept {
    return config_;
  }

  void reset(std::uint64_t map_generation);
  void observeFilteredCells(
      const StaticEvidenceCellGeometryMap& cells,
      double stamp_sec,
      std::uint64_t map_generation,
      std::uint64_t objects_version = 0U);
  StaticEvidenceMutationResult invalidateCells(
      const StaticEvidenceCellKeySet& invalidated_cells,
      std::uint64_t clean_build_version,
      double stamp_sec,
      std::uint64_t map_generation);
  StaticEvidenceMutationResult confirmCleanCells(
      const StaticEvidenceCellGeometryMap& clean_cells,
      const StaticEvidenceCellKeySet& invalidated_cells,
      double stamp_sec,
      std::uint64_t map_generation,
      std::uint64_t clean_build_version = 1U);

  StaticProvenanceDecision query(
      const StaticProvenanceQuery& query) const;
  std::shared_ptr<const StaticEvidenceSnapshot> snapshot() const;
  std::uint64_t latestObservationSequence() const;

  bool saveSnapshot(
      const std::shared_ptr<const StaticEvidenceSnapshot>& snapshot,
      const std::string& path,
      std::string* reason) const;
  bool loadSnapshot(const std::string& path,
                    std::uint64_t current_map_generation,
                    std::uint64_t expected_source_generation,
                    std::uint64_t expected_revision,
                    std::string* reason);

 private:
  void publishSnapshotLocked(double stamp_sec);

  StaticObstacleEvidenceConfig config_;
  mutable std::mutex mutex_;
  std::map<std::int64_t, StaticEvidenceCell> working_cells_;
  // Tombstones survive erasing a contaminated working cell. They prevent an
  // older asynchronous clean result from recreating/confirming that cell.
  std::map<std::int64_t, std::uint64_t> invalidated_versions_;
  std::uint64_t working_generation_ = 0U;
  std::uint64_t latest_observation_sequence_ = 0U;
  std::uint64_t revision_ = 0U;
  double last_observation_stamp_sec_ = 0.0;
  std::shared_ptr<const StaticEvidenceSnapshot> snapshot_;
};

}  // namespace ndt_slam
