#pragma once

#include <cstdint>
#include <string>

namespace ndt_slam {

enum class CargoResidualClass : std::uint8_t {
  EXTERNAL_OBSTACLE = 0,
  CARGO_SELF = 1,
  UNKNOWN = 2,
};

struct CargoResidualClassifierConfig {
  float near_zero_distance_m = 0.05F;
  float minimum_inside_xy_ratio = 0.60F;
  float minimum_identity_match_ratio = 0.35F;
  float minimum_surface_band_ratio = 0.50F;
  float minimum_motion_match_score = 0.70F;
};

struct CargoResidualClassifierInput {
  float footprint_distance_m = 0.0F;
  float inside_xy_ratio = 0.0F;
  float identity_match_ratio = 0.0F;
  float surface_band_ratio = 0.0F;
  float moves_with_cargo_score = 0.0F;
  bool confirmed_static_track_match = false;
};

struct CargoResidualClassifierDecision {
  bool valid = false;
  CargoResidualClass classification = CargoResidualClass::UNKNOWN;
  bool source_validated = false;
  std::string reason = "not_evaluated";
};

// A distance-near-zero cluster cannot become 17/18 without provenance.
// Cargo deletion requires both identity and motion agreement; a confirmed
// static map-frame track remains an external obstacle.
CargoResidualClassifierDecision classifyCargoResidual(
    const CargoResidualClassifierConfig& config,
    const CargoResidualClassifierInput& input);

}  // namespace ndt_slam
