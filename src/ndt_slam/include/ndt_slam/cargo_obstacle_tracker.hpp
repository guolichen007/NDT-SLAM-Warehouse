#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace ndt_slam {

struct CargoObstacleTrackerConfig {
  int confirm_frames = 3;
  std::size_t minimum_points = 20U;
  double maximum_observation_gap_sec = 0.60;
  double stale_track_sec = 1.00;
  float association_max_centroid_distance_m = 0.75F;
  float association_max_top_step_m = 0.75F;
  // Phase-one production policy: 20-point clusters may retain a diagnostic
  // identity, but only independently proven warehouse cargo stacks can issue
  // a formal 17/18. Small-object warning can be enabled after its dedicated
  // provenance policy is commissioned.
  bool require_static_cargo_for_warning = true;
  std::size_t static_cargo_min_voxel_points = 80U;
  std::size_t static_cargo_min_raw_equivalent_points = 0U;
  float static_cargo_min_xy_area_m2 = 0.50F;
  float static_cargo_min_long_side_m = 0.80F;
  float static_cargo_min_height_span_m = 0.40F;
  std::size_t static_cargo_min_occupied_cells = 12U;
  int static_cargo_confirm_frames = 8;
  double static_cargo_confirm_sec = 1.0;
  float static_velocity_threshold_mps = 0.08F;
};

struct CargoObstacleObservation {
  std::size_t source_index = 0U;
  Eigen::Vector3f centroid_map = Eigen::Vector3f::Zero();
  float top_z95_map = std::numeric_limits<float>::quiet_NaN();
  float footprint_distance_m = std::numeric_limits<float>::infinity();
  float conservative_clearance_m =
      std::numeric_limits<float>::quiet_NaN();
  std::size_t point_count = 0U;
  std::size_t raw_equivalent_point_count = 0U;
  float xy_area_m2 = 0.0F;
  float long_side_m = 0.0F;
  float height_span_m = 0.0F;
  std::size_t occupied_cells = 0U;
  std::uint16_t warning_code = 0U;
  bool source_validated = true;
  bool independent_external_provenance = false;
};

struct CargoObstacleTrack {
  std::uint64_t track_id = 0U;
  Eigen::Vector3f centroid_map = Eigen::Vector3f::Zero();
  Eigen::Vector3f velocity_map = Eigen::Vector3f::Zero();
  float top_z95_map = std::numeric_limits<float>::quiet_NaN();
  float footprint_distance_m = std::numeric_limits<float>::infinity();
  float conservative_clearance_m =
      std::numeric_limits<float>::quiet_NaN();
  std::size_t point_count = 0U;
  std::uint16_t warning_code = 0U;
  int total_consecutive_observations = 0;
  int validated_consecutive_observations = 0;
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
  bool independent_external_provenance = false;
  bool current_source_validated = false;
  std::size_t current_source_index = 0U;
};

struct CargoObstacleTrackerDecision {
  bool valid = false;
  bool hazard_observed = false;
  bool confirmed_hazard = false;
  std::uint16_t warning_code = 0U;
  std::uint64_t selected_track_id = 0U;
  std::size_t selected_source_index = 0U;
  int selected_confirm_count = 0;
  double selected_track_age_sec = 0.0;
  bool selected_track_static = false;
  Eigen::Vector3f selected_track_velocity = Eigen::Vector3f::Zero();
  std::string reason = "not_evaluated";
};

// Associates every hazard cluster in map coordinates. Confirmation is owned
// by each physical track, so a different per-frame "most dangerous" winner
// cannot advance or reset another obstacle's evidence count.
class CargoObstacleTracker {
 public:
  explicit CargoObstacleTracker(
      const CargoObstacleTrackerConfig& config =
          CargoObstacleTrackerConfig());

  void setConfig(const CargoObstacleTrackerConfig& config);
  const CargoObstacleTrackerConfig& config() const noexcept { return config_; }
  void reset();
  CargoObstacleTrackerDecision update(
      double stamp_sec,
      const std::vector<CargoObstacleObservation>& observations);
  const std::vector<CargoObstacleTrack>& tracks() const noexcept {
    return tracks_;
  }

 private:
  CargoObstacleTrackerConfig config_;
  std::vector<CargoObstacleTrack> tracks_;
  std::uint64_t next_track_id_ = 1U;
  std::uint64_t cycle_ = 0U;
  bool has_stamp_ = false;
  double last_stamp_sec_ = 0.0;
};

}  // namespace ndt_slam
