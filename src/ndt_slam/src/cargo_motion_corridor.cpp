#include "ndt_slam/cargo_motion_corridor.hpp"

#include <algorithm>
#include <cmath>

namespace ndt_slam {
namespace {

bool validConfig(const CargoMotionCorridorConfig& config) {
  return std::isfinite(config.immediate_near_field_m) &&
      config.immediate_near_field_m >= 0.0F &&
      std::isfinite(config.minimum_motion_speed_mps) &&
      config.minimum_motion_speed_mps >= 0.0F &&
      std::isfinite(config.prediction_horizon_sec) &&
      config.prediction_horizon_sec > 0.0F &&
      std::isfinite(config.minimum_prediction_distance_m) &&
      config.minimum_prediction_distance_m > 0.0F &&
      std::isfinite(config.lateral_margin_m) &&
      config.lateral_margin_m >= 0.0F &&
      std::isfinite(config.rear_margin_m) && config.rear_margin_m >= 0.0F &&
      std::isfinite(config.velocity_alpha) &&
      config.velocity_alpha >= 0.0F && config.velocity_alpha <= 1.0F &&
      std::isfinite(config.maximum_velocity_sample_gap_sec) &&
      config.maximum_velocity_sample_gap_sec > 0.0;
}

}  // namespace

const char* cargoSafetySpatialModeName(CargoSafetySpatialMode mode) noexcept {
  switch (mode) {
    case CargoSafetySpatialMode::MOTION_CORRIDOR:
      return "MOTION_CORRIDOR";
    case CargoSafetySpatialMode::STATIONARY_GUARD:
      return "STATIONARY_GUARD";
    case CargoSafetySpatialMode::RADIAL_FALLBACK:
    default:
      return "RADIAL_FALLBACK";
  }
}

CargoMotionCorridorDecision evaluateCargoMotionCorridor(
    const CargoMotionCorridorConfig& config,
    const CargoMotionCorridorInput& input) {
  CargoMotionCorridorDecision decision;
  if (!validConfig(config) || !input.cargo_center_map.allFinite() ||
      !input.cargo_velocity_map.allFinite() ||
      !input.obstacle_nearest_map.allFinite() ||
      !input.obstacle_centroid_map.allFinite() ||
      !std::isfinite(input.cargo_length_m) || input.cargo_length_m <= 0.0F ||
      !std::isfinite(input.cargo_width_m) || input.cargo_width_m <= 0.0F ||
      !std::isfinite(input.cargo_yaw_map_rad) ||
      !std::isfinite(input.horizontal_uncertainty_m) ||
      input.horizontal_uncertainty_m < 0.0F ||
      !std::isfinite(input.current_footprint_distance_m) ||
      input.current_footprint_distance_m < 0.0F) {
    decision.eligible = true;
    decision.reason = "invalid_corridor_input_radial_fallback";
    return decision;
  }
  decision.valid = true;
  decision.speed_mps = input.cargo_velocity_map.norm();

  if (!config.enabled || !input.velocity_valid) {
    decision.mode = CargoSafetySpatialMode::RADIAL_FALLBACK;
    decision.eligible = true;
    decision.reason = config.enabled
        ? "motion_unavailable_radial_fallback"
        : "motion_corridor_disabled_radial_fallback";
    return decision;
  }

  if (decision.speed_mps < config.minimum_motion_speed_mps) {
    decision.mode = CargoSafetySpatialMode::STATIONARY_GUARD;
    decision.corridor_half_width_m =
        config.immediate_near_field_m + input.horizontal_uncertainty_m;
    decision.eligible = input.current_footprint_distance_m <=
        config.immediate_near_field_m;
    decision.reason = decision.eligible
        ? "stationary_emergency_shell"
        : "stationary_structure_outside_guard";
    return decision;
  }

  decision.mode = CargoSafetySpatialMode::MOTION_CORRIDOR;
  if (input.current_footprint_distance_m <=
      config.immediate_near_field_m) {
    decision.eligible = true;
    decision.reason = "immediate_near_field";
    return decision;
  }

  const Eigen::Vector2f direction =
      input.cargo_velocity_map / decision.speed_mps;
  const Eigen::Vector2f normal(-direction.y(), direction.x());
  const Eigen::Vector2f cargo_long_axis(
      std::cos(input.cargo_yaw_map_rad),
      std::sin(input.cargo_yaw_map_rad));
  const Eigen::Vector2f cargo_short_axis(
      -cargo_long_axis.y(), cargo_long_axis.x());
  const float projected_half_forward_extent =
      0.5F *
      (std::abs(direction.dot(cargo_long_axis)) * input.cargo_length_m +
       std::abs(direction.dot(cargo_short_axis)) * input.cargo_width_m);
  const float projected_half_width =
      0.5F * (std::abs(normal.dot(cargo_long_axis)) * input.cargo_length_m +
              std::abs(normal.dot(cargo_short_axis)) * input.cargo_width_m);
  decision.corridor_half_width_m = projected_half_width +
      input.horizontal_uncertainty_m + config.lateral_margin_m;
  const float corridor_length = std::max(
      decision.speed_mps * config.prediction_horizon_sec,
      config.minimum_prediction_distance_m);
  const auto inside_corridor = [&](const Eigen::Vector2f& point,
                                   float* along_out,
                                   float* lateral_out) {
    const Eigen::Vector2f relative = point - input.cargo_center_map;
    const float along = relative.dot(direction);
    const float lateral =
        std::abs(relative.x() * direction.y() -
                 relative.y() * direction.x());
    if (along_out != nullptr) *along_out = along;
    if (lateral_out != nullptr) *lateral_out = lateral;
    return along >=
            -projected_half_forward_extent -
                input.horizontal_uncertainty_m - config.rear_margin_m &&
        along <=
            projected_half_forward_extent +
                input.horizontal_uncertainty_m + corridor_length &&
        lateral <= decision.corridor_half_width_m;
  };

  float nearest_along = 0.0F;
  float nearest_lateral = 0.0F;
  const bool nearest_inside = inside_corridor(
      input.obstacle_nearest_map, &nearest_along, &nearest_lateral);
  float centroid_along = 0.0F;
  float centroid_lateral = 0.0F;
  const bool centroid_inside = inside_corridor(
      input.obstacle_centroid_map, &centroid_along, &centroid_lateral);
  decision.eligible = nearest_inside || centroid_inside;
  if (nearest_inside || !centroid_inside) {
    decision.along_track_m = nearest_along;
    decision.lateral_distance_m = nearest_lateral;
  } else {
    decision.along_track_m = centroid_along;
    decision.lateral_distance_m = centroid_lateral;
  }
  decision.reason = decision.eligible
      ? "obstacle_intersects_motion_corridor"
      : "obstacle_outside_motion_corridor";
  return decision;
}

}  // namespace ndt_slam
