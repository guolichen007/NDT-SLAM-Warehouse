#include "ndt_slam/hazard_evaluator.hpp"

#include <cmath>

namespace ndt_slam {

HazardEvaluationResult HazardEvaluator::evaluate(
    const HazardEvaluationInput& input,
    const ObstaclePerceptionCluster& cluster) const noexcept {
  HazardEvaluationResult result;
  result.assessment.source_stamp_sec = input.source_stamp_sec;
  result.assessment.cargo_lifecycle_id = input.cargo_lifecycle_id;
  result.assessment.cargo_track_id = input.cargo_track_id;
  result.assessment.obstacle_track_id = input.obstacle_track_id;
  result.assessment.footprint_distance_m = cluster.footprint_distance_m;
  result.assessment.obstacle_top_z_m = cluster.top_z95_m;
  result.assessment.safe_bottom_z_m = input.safe_bottom_z_m;
  result.assessment.combined_uncertainty_m =
      cluster.obstacle_uncertainty_m;
  if (!input.vertical_geometry_valid ||
      !std::isfinite(input.safe_bottom_z_m) ||
      !std::isfinite(input.cargo_max_z_m) ||
      !std::isfinite(input.overhead_separation_margin_m) ||
      input.overhead_separation_margin_m < 0.0F ||
      !std::isfinite(input.minimum_vertical_continuity_ratio) ||
      input.minimum_vertical_continuity_ratio < 0.0F ||
      input.minimum_vertical_continuity_ratio > 1.0F ||
      !std::isfinite(cluster.footprint_distance_m) ||
      !std::isfinite(cluster.top_z95_m) ||
      !std::isfinite(cluster.bottom_z05_m) ||
      !std::isfinite(cluster.obstacle_uncertainty_m)) {
    result.reason = "hazard_geometry_invalid";
    return result;
  }
  result.assessment.conservative_clearance_m = input.safe_bottom_z_m -
      (cluster.top_z95_m + cluster.obstacle_uncertainty_m);
  result.entirely_above_cargo = cluster.bottom_z05_m >
      input.cargo_max_z_m + input.overhead_separation_margin_m;
  result.vertically_continuous = cluster.vertical_continuity_ratio >=
      input.minimum_vertical_continuity_ratio;
  result.low_clearance = !result.entirely_above_cargo &&
      result.vertically_continuous;
  result.assessment.valid =
      std::isfinite(result.assessment.conservative_clearance_m);
  result.reason = result.assessment.valid
      ? (result.low_clearance ? "low_clearance_geometry" :
         (result.entirely_above_cargo ? "overhead_geometry" :
          "vertical_continuity_insufficient"))
      : "hazard_result_non_finite";
  return result;
}

}  // namespace ndt_slam
