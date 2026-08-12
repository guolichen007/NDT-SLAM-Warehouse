#pragma once

#include "ndt_slam/cargo_obstacle_tracker.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace ndt_slam {

// Temporal maturity and operator approval are separate authorities. Loading a
// clean PCD never fabricates runtime history: it is either explicitly approved
// by an offline audit or remains unavailable to formal avoidance.
enum class StaticEvidenceAuthority : std::uint8_t {
  RUNTIME_MATURE = 0,
  OPERATOR_APPROVED_BASELINE = 1,
  UNVERIFIED_LOADED_CLEAN = 2,
};

const char* staticEvidenceAuthorityName(
    StaticEvidenceAuthority authority) noexcept;

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
  // Clean-map observations are produced by asynchronous map maintenance and
  // warehouse revisits may be minutes apart. This window applies only while a
  // cell is immature; mature cells remain authorized until explicit negative
  // evidence or a map lifecycle transition invalidates them.
  double immature_max_observation_gap_sec = 300.0;
  // Evidence older than the immature window is weakened instead of discarded.
  // This preserves useful history without allowing a few ancient samples to
  // authorize a cell indefinitely.
  double immature_gap_retention_ratio = 0.50;
  std::uint64_t maximum_observation_sequence_gap = 2U;
  // Per-cell vertical extents are robust estimates over completed clean-map
  // observations. A single bad registration or isolated LiDAR return must not
  // permanently stretch a mature obstacle to several metres.
  std::size_t height_history_window = 9U;
  std::size_t height_outlier_minimum_samples = 3U;
  double height_outlier_mad_multiplier = 3.5;
  float height_outlier_minimum_band_m = 0.20F;
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
  // Clean-map confirmation and temporal maturity are deliberately separate.
  // Once mature, a cell stays authorized until an explicit deny tombstone or
  // map lifecycle transition. Immature history may decay across a long revisit
  // gap, but can never bypass clean-map confirmation or maturity thresholds.
  bool temporally_mature = false;
  std::uint64_t first_observation_sequence = 0U;
  std::uint64_t last_observation_sequence = 0U;
  std::uint64_t last_observed_objects_version = 0U;
  std::uint64_t last_clean_confirmed_version = 0U;
  std::uint64_t last_invalidated_version = 0U;
  std::uint64_t map_generation = 0U;
};

struct StaticEvidenceSnapshot {
  static constexpr std::uint32_t kSchemaVersion = 3U;

  std::uint32_t schema_version = kSchemaVersion;
  std::uint64_t map_generation = 0U;
  std::uint64_t revision = 0U;
  std::uint64_t latest_observation_sequence = 0U;
  double source_stamp_sec = 0.0;
  float cell_size_m = 0.25F;
  StaticEvidenceAuthority authority =
      StaticEvidenceAuthority::RUNTIME_MATURE;
  std::map<std::int64_t, StaticEvidenceCell> cells;
};

struct StaticEvidenceDiagnostics {
  std::size_t working_cells = 0U;
  std::size_t clean_confirmed_cells = 0U;
  std::size_t temporally_mature_cells = 0U;
  std::size_t pending_observation_count = 0U;
  std::size_t pending_stable_duration = 0U;
  std::uint64_t reset_by_time_gap = 0U;
  std::uint64_t decayed_by_time_gap = 0U;
  std::uint64_t reset_by_sequence_gap = 0U;
  std::uint64_t invalidated_by_tombstone = 0U;
  std::uint64_t generation_mismatch = 0U;
  std::uint64_t height_invalid = 0U;
  std::uint64_t height_samples_accepted = 0U;
  std::uint64_t height_outliers_rejected = 0U;
  std::uint64_t not_in_view = 0U;
  std::uint64_t observed_free = 0U;
  std::uint64_t observed_occupied = 0U;
  std::map<std::uint32_t, std::size_t> streak_histogram;
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
  StaticEvidenceAuthority authority =
      StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN;
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

// Fully allocated, generation-adjusted state for a no-I/O lifecycle install.
// Preparation may fail and must happen before the runtime map is mutated.
struct PreparedStaticEvidenceInstall {
  bool valid = false;
  std::uint64_t map_generation = 0U;
  std::uint64_t latest_observation_sequence = 0U;
  std::uint64_t revision = 0U;
  StaticEvidenceAuthority authority =
      StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN;
  std::map<std::int64_t, StaticEvidenceCell> working_cells;
  std::shared_ptr<StaticEvidenceSnapshot> snapshot;
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
  // Returns true only when a clean-confirmed cell crosses the temporal
  // maturity boundary and the immutable safety snapshot changes.
  bool observeFilteredCells(
      const StaticEvidenceCellGeometryMap& cells,
      double stamp_sec,
      std::uint64_t map_generation,
      std::uint64_t objects_version = 0U);
  // Advances exactly one completed clean-build observation. A cell outside
  // observable_cells is NOT_IN_VIEW: its streak is paused, not cleared. A
  // visible cell in observed_free_cells is explicit negative evidence and is
  // invalidated. This prevents a global build sequence from breaking cells
  // merely because they were outside the LiDAR ROI.
  bool observeCleanBuildCells(
      const StaticEvidenceCellGeometryMap& occupied_cells,
      const StaticEvidenceCellKeySet& observable_cells,
      const StaticEvidenceCellKeySet& observed_free_cells,
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
  std::size_t matureCellCount() const;
  StaticEvidenceDiagnostics diagnostics() const;
  void setSnapshotAuthority(StaticEvidenceAuthority authority);

  bool saveSnapshot(
      const std::shared_ptr<const StaticEvidenceSnapshot>& snapshot,
      const std::string& path,
      std::string* reason) const;

  // Validate a single cell for persistence. Must match the loader contract
  // exactly; save and load use the same rules to prevent drift.
  static bool validateCellForPersistence(
      const StaticEvidenceCell& cell,
      const StaticObstacleEvidenceConfig& config,
      std::string* reason);

  bool loadSnapshot(const std::string& path,
                    std::uint64_t current_map_generation,
                    std::uint64_t expected_source_generation,
                    std::uint64_t expected_revision,
                    std::string* reason);
  bool loadSnapshotCandidate(
      const std::string& path,
      std::uint64_t expected_source_generation,
      std::uint64_t expected_revision,
      StaticEvidenceSnapshot* candidate,
      std::string* reason) const;
  bool restoreSnapshotWithoutRevisionIncrement(
      const StaticEvidenceSnapshot& candidate,
      std::uint64_t current_map_generation,
      std::string* reason);
  bool prepareSnapshotInstall(
      const StaticEvidenceSnapshot& candidate,
      std::uint64_t current_map_generation,
      PreparedStaticEvidenceInstall* prepared,
      std::string* reason) const;
  void installPreparedSnapshot(
      PreparedStaticEvidenceInstall&& prepared,
      std::uint64_t current_map_generation = 0U) noexcept;

 private:
  struct HeightHistory {
    std::deque<float> min_z;
    std::deque<float> max_z;
  };

  void publishSnapshotLocked(double stamp_sec,
                             bool increment_revision = true);
  bool isTemporallyMatureLocked(const StaticEvidenceCell& cell) const;
  bool updateHeightEstimateLocked(
      std::int64_t key,
      const StaticEvidenceCellGeometry& geometry,
      StaticEvidenceCell& cell,
      bool reset_history);
  void seedHeightHistoriesLocked();

  StaticObstacleEvidenceConfig config_;
  mutable std::mutex mutex_;
  std::map<std::int64_t, StaticEvidenceCell> working_cells_;
  std::map<std::int64_t, HeightHistory> height_histories_;
  // Tombstones survive erasing a contaminated working cell. They prevent an
  // older asynchronous clean result from recreating/confirming that cell.
  std::map<std::int64_t, std::uint64_t> invalidated_versions_;
  StaticEvidenceCellKeySet observed_free_tombstones_;
  std::uint64_t working_generation_ = 0U;
  std::uint64_t latest_observation_sequence_ = 0U;
  std::uint64_t revision_ = 0U;
  double last_observation_stamp_sec_ = 0.0;
  StaticEvidenceAuthority authority_ =
      StaticEvidenceAuthority::RUNTIME_MATURE;
  StaticEvidenceDiagnostics diagnostics_totals_;
  std::shared_ptr<const StaticEvidenceSnapshot> snapshot_;
};

}  // namespace ndt_slam
