#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "ndt_slam/cargo_config_validation.hpp"
#include "ndt_slam/pose_authority_identity.hpp"

namespace ndt_slam {

enum class ObstacleSupportKind : std::uint8_t {
  DENSE_CURRENT_FRAME = 0,
  SPARSE_MULTI_FRAME = 1,
};

const char* obstacleSupportKindName(ObstacleSupportKind kind) noexcept;

struct SparseObstacleSupportFrame {
  double stamp_sec = 0.0;
  std::vector<std::int64_t> occupied_map_cells;
};

enum class ExternalProvenance : std::uint8_t {
  NONE = 0,
  OUTSIDE_CARGO_SHELL_ONLY = 1,
  PRE_CARGO_OCCUPANCY = 2,
  STATIC_MAP_MATCH = 3,
  CARGO_MOVED_AWAY_PERSISTENCE = 4,
  DUAL_LIDAR_CONSENSUS = 5,
};

const char* externalProvenanceName(ExternalProvenance provenance) noexcept;
bool authorizesStaticObstacle(ExternalProvenance provenance) noexcept;

struct CargoObstacleTrackerConfig {
  int confirm_frames = 3;
  std::size_t minimum_points = 20U;
  float level1_warning_distance_m = 3.0F;
  float level2_warning_distance_m = 5.0F;
  float acquisition_distance_m = 8.0F;
  // Formal 17/18 warnings require the same physical track to be confirmed in
  // the true 5-8 m acquisition shell. Frame/duration thresholds are tunable
  // evidence strength, not fixed business constants.
  bool require_far_field_history_for_warnings = true;
  int far_history_confirm_frames = 3;
  double far_history_confirm_duration_sec = 0.2;
  // A newly created cluster already embedded in the cargo footprint is
  // ambiguous: it is commonly residual cargo self-points. It may issue a
  // warning only after this same track was independently observed outside
  // the embedded band, or when authoritative static provenance proves it is
  // a warehouse obstacle.
  float embedded_distance_threshold_m = 0.05F;
  double maximum_observation_gap_sec = 0.60;
  double stale_track_sec = 1.00;
  float association_max_centroid_distance_m = 0.75F;
  float association_max_top_step_m = 0.75F;
  int association_neighbor_cell_radius = 1;
  float static_track_cell_overlap_min = 0.20F;
  float static_track_iou_min = 0.10F;
  float static_provenance_min_cargo_motion_m = 0.30F;
  // Library-safe fail-closed default. Production YAML explicitly disables
  // this gate where same-track far-history is the commissioned authority.
  // Phase-one policy: 20-point clusters may retain a diagnostic
  // identity, but only independently proven warehouse cargo stacks can issue
  // a formal 17/18. Small-object warning can be enabled after its dedicated
  // provenance policy is commissioned.
  bool require_static_cargo_for_warning = true;
  // Pending-cargo warning promotion uses this stricter mode so a single
  // intermittent large extent cannot reuse an otherwise confirmed track.
  bool require_large_geometry_for_warning = false;
  std::size_t static_cargo_min_voxel_points = 80U;
  std::size_t static_cargo_min_raw_equivalent_points = 0U;
  float static_cargo_min_xy_area_m2 = 0.50F;
  float static_cargo_min_long_side_m = 0.80F;
  float static_cargo_min_height_span_m = 0.40F;
  std::size_t static_cargo_min_occupied_cells = 12U;
  int known_static_confirm_frames = 3;
  int static_cargo_confirm_frames = 8;
  double static_cargo_confirm_sec = 1.0;
  float static_velocity_threshold_mps = 0.08F;
};

struct CargoObstacleObservation {
  std::size_t source_index = 0U;
  Eigen::Vector3f centroid_map = Eigen::Vector3f::Zero();
  float top_z95_map = std::numeric_limits<float>::quiet_NaN();
  float bottom_z05_map = 0.0F;
  float vertical_continuity_ratio = 1.0F;
  bool entirely_above_cargo = false;
  float footprint_distance_m = std::numeric_limits<float>::infinity();
  float conservative_clearance_m =
      std::numeric_limits<float>::quiet_NaN();
  // False means that physical XY/Z cluster facts are usable for identity and
  // far-field acquisition, but Cargo bottom authority is unavailable. Such
  // an observation can never advance warning, clearance or CLEAR authority.
  bool hazard_geometry_valid = true;
  // Combined cargo, obstacle and localization XY uncertainty. A far sample
  // is authoritative only when footprint_distance - uncertainty remains >5m.
  float horizontal_uncertainty_m = 0.0F;
  std::size_t point_count = 0U;
  std::size_t raw_equivalent_point_count = 0U;
  float xy_area_m2 = 0.0F;
  float long_side_m = 0.0F;
  float height_span_m = 0.0F;
  std::size_t occupied_cells = 0U;
  std::vector<std::int64_t> occupied_map_cells;
  Eigen::Vector2f cargo_center_map = Eigen::Vector2f::Zero();
  bool cargo_center_valid = false;
  std::uint16_t warning_code = 0U;
  // False for a 5-7 m acquisition observation. Such a frame may build
  // identity/provenance history but can never publish 17/18.
  bool warning_eligible = true;
  bool source_validated = true;
  float validation_shell_m = 0.0F;
  ExternalProvenance provenance = ExternalProvenance::NONE;
  TemporalEvidenceAuthority pose_authority;
  // Reserved interface: this branch provides no baseline producer, therefore
  // runtime observations leave this false and 17/18 use live far history.
  bool certified_static_provenance = false;
};

struct CargoObstacleTrack {
  std::uint64_t track_id = 0U;
  Eigen::Vector3f centroid_map = Eigen::Vector3f::Zero();
  Eigen::Vector3f velocity_map = Eigen::Vector3f::Zero();
  float top_z95_map = std::numeric_limits<float>::quiet_NaN();
  float bottom_z05_map = 0.0F;
  float vertical_continuity_ratio = 1.0F;
  bool entirely_above_cargo = false;
  float footprint_distance_m = std::numeric_limits<float>::infinity();
  float conservative_clearance_m =
      std::numeric_limits<float>::quiet_NaN();
  bool current_hazard_geometry_valid = false;
  std::size_t point_count = 0U;
  std::uint16_t warning_code = 0U;
  int total_consecutive_observations = 0;
  int validated_consecutive_observations = 0;
  int geometry_validated_consecutive_observations = 0;
  int static_provenance_consecutive_observations = 0;
  double static_provenance_first_stamp_sec = 0.0;
  // Compatibility diagnostic: mirrors total_consecutive_observations.
  int consecutive_observations = 0;
  double first_stamp_sec = 0.0;
  double last_stamp_sec = 0.0;
  std::uint64_t last_observation_cycle = 0U;
  bool observed_this_cycle = false;
  bool confirmed = false;
  bool static_obstacle = false;
  bool large_cluster_geometry_valid = false;
  ExternalProvenance provenance = ExternalProvenance::NONE;
  bool provenance_valid = false;
  int separated_validated_observations = 0;
  bool separated_obstacle_history_valid = false;
  bool current_embedded = false;
  int far_field_validated_observations = 0;
  double far_field_first_stamp_sec = 0.0;
  double far_field_duration_sec = 0.0;
  bool far_field_history_valid = false;
  bool certified_static_provenance = false;
  bool current_near_field = false;
  std::vector<std::int64_t> occupied_map_cells;
  std::vector<std::int64_t> identity_anchor_map_cells;
  Eigen::Vector2f first_cargo_center_map = Eigen::Vector2f::Zero();
  bool first_cargo_center_valid = false;
  float maximum_cargo_displacement_m = 0.0F;
  float last_cell_overlap = 0.0F;
  float last_cell_iou = 0.0F;
  float last_neighbor_cell_overlap = 0.0F;
  float last_anchor_cell_overlap = 0.0F;
  float last_association_cost = 0.0F;
  bool association_ambiguous = false;
  std::string association_reset_reason;
  TemporalEvidenceAuthority pose_authority;
  bool current_source_validated = false;
  bool current_warning_eligible = false;
  std::size_t current_source_index = 0U;
  ObstacleSupportKind support_kind =
      ObstacleSupportKind::DENSE_CURRENT_FRAME;
  std::size_t real_current_point_count = 0U;
  std::vector<SparseObstacleSupportFrame> sparse_support_ring;
  int sparse_independent_frames = 0;
  bool sparse_to_dense_this_cycle = false;
};

struct CargoObstacleTrackerDecision {
  bool valid = false;
  bool hazard_observed = false;
  bool confirmed_hazard = false;
  // Current dense (or independently mature sparse) hazard evidence that is
  // safe only for downstream Code 29 review. It deliberately does not claim
  // the confirmed true-far authority required by formal Code 17/18.
  bool no_far_review_hazard = false;
  std::uint16_t warning_code = 0U;
  std::uint64_t selected_track_id = 0U;
  std::size_t selected_source_index = 0U;
  int selected_confirm_count = 0;
  int selected_geometry_confirm_count = 0;
  double selected_track_age_sec = 0.0;
  bool selected_track_static = false;
  bool selected_large_geometry_valid = false;
  ExternalProvenance selected_provenance = ExternalProvenance::NONE;
  bool selected_provenance_valid = false;
  bool selected_embedded = false;
  bool selected_embedded_authorized = false;
  int selected_separated_observations = 0;
  bool selected_near_field = false;
  bool selected_near_field_authorized = false;
  bool selected_far_field_history_valid = false;
  int selected_far_field_observations = 0;
  double selected_far_field_duration_sec = 0.0;
  bool selected_certified_static_provenance = false;
  int selected_static_provenance_streak = 0;
  double selected_static_age_sec = 0.0;
  float selected_track_cell_overlap = 0.0F;
  float selected_track_iou = 0.0F;
  float selected_track_neighbor_cell_overlap = 0.0F;
  float selected_association_cost = 0.0F;
  bool selected_association_ambiguous = false;
  std::string selected_association_reset_reason;
  std::uint64_t created_track_count = 0U;
  std::uint64_t association_reset_count = 0U;
  std::uint64_t ambiguous_association_count = 0U;
  Eigen::Vector3f selected_track_velocity = Eigen::Vector3f::Zero();
  TemporalEvidenceAuthority selected_pose_authority;
  ObstacleSupportKind selected_support_kind =
      ObstacleSupportKind::DENSE_CURRENT_FRAME;
  std::size_t selected_real_current_point_count = 0U;
  int selected_sparse_independent_frames = 0;
  bool selected_sparse_to_dense = false;
  std::size_t sparse_track_count = 0U;
  std::size_t sparse_ring_high_water = 0U;
  std::uint64_t sparse_ambiguity_reject_count = 0U;
  std::uint64_t sparse_authority_reset_count = 0U;
  std::string reason = "not_evaluated";
};

// Physical identity/history is stateful and unique. Formal/Pending only
// supply an immutable policy for the current decision projection.
struct CargoObstacleAuthorityPolicy {
  bool require_static_cargo_for_warning = true;
  bool require_large_geometry_for_warning = false;
};

// Associates warning clusters and directional/radial 5-7 m acquisition
// clusters in map coordinates. Confirmation is owned by each physical track,
// so a different per-frame "most dangerous" winner cannot advance or reset
// another obstacle's evidence count. Acquisition-only observations can
// mature identity/provenance but never authorize a warning.
class CargoObstacleTracker {
 public:
  explicit CargoObstacleTracker(
      const CargoObstacleTrackerConfig& config =
          CargoObstacleTrackerConfig());

  CargoConfigValidationResult setConfig(
      const CargoObstacleTrackerConfig& config);
  const CargoConfigValidationResult& configValidation() const noexcept {
    return config_validation_;
  }
  const CargoObstacleTrackerConfig& config() const noexcept { return config_; }
  void reset();
  CargoObstacleTrackerDecision update(
      double stamp_sec,
      const std::vector<CargoObstacleObservation>& observations);
  CargoObstacleTrackerDecision update(
      double stamp_sec,
      const std::vector<CargoObstacleObservation>& observations,
      const CargoObstacleAuthorityPolicy& authority_policy);
  const std::vector<CargoObstacleTrack>& tracks() const noexcept {
    return tracks_;
  }

 private:
  CargoObstacleTrackerConfig config_;
  CargoConfigValidationResult config_validation_;
  std::vector<CargoObstacleTrack> tracks_;
  std::uint64_t next_track_id_ = 1U;
  std::uint64_t cycle_ = 0U;
  bool has_stamp_ = false;
  double last_stamp_sec_ = 0.0;
  std::uint64_t created_track_count_ = 0U;
  std::uint64_t association_reset_count_ = 0U;
  std::uint64_t ambiguous_association_count_ = 0U;
  std::uint64_t sparse_ambiguity_reject_count_ = 0U;
  std::uint64_t sparse_authority_reset_count_ = 0U;
};

using PhysicalObstacleTrackStore = CargoObstacleTracker;

CargoConfigValidationResult validateCargoObstacleTrackerConfig(
    const CargoObstacleTrackerConfig& config);

}  // namespace ndt_slam
