#include "ndt_slam/cargo_residual_classifier.hpp"

#include <cmath>

namespace ndt_slam {
namespace {

bool validRatio(float value) {
  return std::isfinite(value) && value >= 0.0F && value <= 1.0F;
}

bool validConfig(const CargoResidualClassifierConfig& config) {
  return std::isfinite(config.validation_shell_m) &&
      config.validation_shell_m >= 0.0F &&
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
  if (input.footprint_distance_m > config.validation_shell_m) {
    decision.classification = CargoResidualClass::EXTERNAL_OBSTACLE;
    decision.source_validated = true;
    decision.reason = "outside_cargo_residual_shell";
    return decision;
  }
  // Cargo identity wins before static provenance. Cargo residuals are also
  // map-static while the crane is stopped, so low velocity is not proof.
  if (input.inside_xy_ratio >= config.minimum_inside_xy_ratio &&
      input.identity_match_ratio >= config.minimum_identity_match_ratio &&
      input.surface_band_ratio >= config.minimum_surface_band_ratio &&
      input.moves_with_cargo_score >= config.minimum_motion_match_score) {
    decision.classification = CargoResidualClass::CARGO_SELF;
    decision.source_validated = true;
    decision.reason = "cargo_shell_identity_moves_with_cargo";
    return decision;
  }
  if (input.independent_external_static_provenance) {
    decision.classification = CargoResidualClass::EXTERNAL_OBSTACLE;
    decision.source_validated = true;
    decision.reason = "independent_external_static_provenance";
    return decision;
  }
  decision.classification = CargoResidualClass::UNKNOWN;
  decision.source_validated = false;
  decision.reason = "cargo_boundary_source_unresolved";
  return decision;
}

}  // namespace ndt_slam
