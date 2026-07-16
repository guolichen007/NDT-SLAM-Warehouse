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
  float static_velocity_threshold_mps = 0.15F;
};

struct CargoObstacleObservation {
  std::size_t source_index = 0U;
  Eigen::Vector3f centroid_map = Eigen::Vector3f::Zero();
  float top_z95_map = std::numeric_limits<float>::quiet_NaN();
  float footprint_distance_m = std::numeric_limits<float>::infinity();
  float conservative_clearance_m =
      std::numeric_limits<float>::quiet_NaN();
  std::size_t point_count = 0U;
  std::uint16_t warning_code = 0U;
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
  int consecutive_observations = 0;
  double first_stamp_sec = 0.0;
  double last_stamp_sec = 0.0;
  std::uint64_t last_observation_cycle = 0U;
  bool observed_this_cycle = false;
  bool confirmed = false;
  bool static_obstacle = false;
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
