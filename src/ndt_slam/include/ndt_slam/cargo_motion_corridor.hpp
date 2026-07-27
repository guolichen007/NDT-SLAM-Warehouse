#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <string>

namespace ndt_slam {

enum class CargoSafetySpatialMode : std::uint8_t {
  RADIAL_FALLBACK = 0,
  MOTION_CORRIDOR = 1,
  STATIONARY_GUARD = 2,
};

const char* cargoSafetySpatialModeName(CargoSafetySpatialMode mode) noexcept;

struct CargoMotionCorridorConfig {
  bool enabled = true;
  float immediate_near_field_m = 0.30F;
  float minimum_motion_speed_mps = 0.05F;
  float prediction_horizon_sec = 3.0F;
  // Directional obstacle acquisition may need more distance than the
  // time-horizon projection at low crane speeds. This is tracking lookahead,
  // not a warning threshold.
  float minimum_prediction_distance_m = 7.0F;
  float lateral_margin_m = 0.30F;
  float rear_margin_m = 0.30F;
  float velocity_alpha = 0.35F;
  double maximum_velocity_sample_gap_sec = 0.80;
};

struct CargoMotionCorridorInput {
  Eigen::Vector2f cargo_center_map = Eigen::Vector2f::Zero();
  Eigen::Vector2f cargo_velocity_map = Eigen::Vector2f::Zero();
  bool velocity_valid = false;
  float cargo_length_m = 0.0F;
  float cargo_width_m = 0.0F;
  float cargo_yaw_map_rad = 0.0F;
  float horizontal_uncertainty_m = 0.0F;
  Eigen::Vector2f obstacle_nearest_map = Eigen::Vector2f::Zero();
  Eigen::Vector2f obstacle_centroid_map = Eigen::Vector2f::Zero();
  float current_footprint_distance_m = 0.0F;
  // Acquisition-only observations build obstacle identity outside the 5 m
  // warning shell. Without authoritative motion they use a radial fallback;
  // they never publish 17/18 because the caller marks them warning-ineligible.
  bool acquisition_only = false;
};

struct CargoMotionCorridorDecision {
  bool valid = false;
  bool eligible = true;
  CargoSafetySpatialMode mode = CargoSafetySpatialMode::RADIAL_FALLBACK;
  float speed_mps = 0.0F;
  float along_track_m = 0.0F;
  float lateral_distance_m = 0.0F;
  float corridor_half_width_m = 0.0F;
  std::string reason = "not_evaluated";
};

// Near-contact hazards are always retained. Beyond the near field, a valid
// velocity creates a future swept corridor. Acquisition-only calls fall back
// to radial tracking when direction is unavailable or stationary; ordinary
// warning calls preserve the stationary emergency-shell policy.
CargoMotionCorridorDecision evaluateCargoMotionCorridor(
    const CargoMotionCorridorConfig& config,
    const CargoMotionCorridorInput& input);

}  // namespace ndt_slam
