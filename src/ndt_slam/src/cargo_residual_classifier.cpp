#include "ndt_slam/cargo_residual_classifier.hpp"

#include <cmath>

namespace ndt_slam {
namespace {

bool validRatio(float value) {
  return std::isfinite(value) && value >= 0.0F && value <= 1.0F;
}

bool validConfig(const CargoResidualClassifierConfig& config) {
  return std::isfinite(config.near_zero_distance_m) &&
      config.near_zero_distance_m >= 0.0F &&
      validRatio(config.minimum_inside_xy_ratio) &&
      validRatio(config.minimum_identity_match_ratio) &&
      validRatio(config.minimum_surface_band_ratio) &&
      validRatio(config.minimum_motion_match_score);
}

}  // namespace

CargoResidualClassifierDecision classifyCargoResidual(
    const CargoResidualClassifierConfig& config,
    const CargoResidualClassifierInput& input) {
  CargoResidualClassifierDecision decision;
  if (!validConfig(config) ||
      !std::isfinite(input.footprint_distance_m) ||
      input.footprint_distance_m < 0.0F ||
      !validRatio(input.inside_xy_ratio) ||
      !validRatio(input.identity_match_ratio) ||
      !validRatio(input.surface_band_ratio) ||
      !validRatio(input.moves_with_cargo_score)) {
    decision.reason = "invalid_residual_classifier_input";
    return decision;
  }
  decision.valid = true;
  if (input.footprint_distance_m > config.near_zero_distance_m) {
    decision.classification = CargoResidualClass::EXTERNAL_OBSTACLE;
    decision.source_validated = true;
    decision.reason = "nonzero_external_obstacle";
    return decision;
  }
  if (input.confirmed_static_track_match) {
    decision.classification = CargoResidualClass::EXTERNAL_OBSTACLE;
    decision.source_validated = true;
    decision.reason = "near_zero_confirmed_static_obstacle";
    return decision;
  }
  if (input.inside_xy_ratio >= config.minimum_inside_xy_ratio &&
      input.identity_match_ratio >= config.minimum_identity_match_ratio &&
      input.surface_band_ratio >= config.minimum_surface_band_ratio &&
      input.moves_with_cargo_score >= config.minimum_motion_match_score) {
    decision.classification = CargoResidualClass::CARGO_SELF;
    decision.source_validated = true;
    decision.reason = "near_zero_identity_moves_with_cargo";
    return decision;
  }
  decision.classification = CargoResidualClass::UNKNOWN;
  decision.source_validated = false;
  decision.reason = "near_zero_source_unresolved";
  return decision;
}

}  // namespace ndt_slam
