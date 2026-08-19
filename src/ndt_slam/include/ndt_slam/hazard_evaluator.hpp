#pragma once

#include "ndt_slam/cargo_domain_contracts.hpp"
#include "ndt_slam/obstacle_perception.hpp"

#include <cstdint>
#include <string>

namespace ndt_slam {

struct HazardEvaluationInput {
  double source_stamp_sec = 0.0;
  std::uint64_t cargo_lifecycle_id = 0U;
  std::uint64_t cargo_track_id = 0U;
  std::uint64_t obstacle_track_id = 0U;
  float safe_bottom_z_m = 0.0F;
  float cargo_max_z_m = 0.0F;
  float overhead_separation_margin_m = 0.10F;
  float minimum_vertical_continuity_ratio = 0.45F;
  bool vertical_geometry_valid = false;
};

struct HazardEvaluationResult {
  HazardAssessment assessment;
  bool entirely_above_cargo = false;
  bool vertically_continuous = false;
  bool low_clearance = false;
  // True when geometry is finite, the cluster is not entirely above the cargo,
  // and its vertical continuity could not be established. Such a cluster must
  // never be interpreted as "proven safe" (see H1 fail-closed contract).
  bool vertical_geometry_unresolved = false;
  std::string reason = "not_evaluated";
};

class HazardEvaluator {
 public:
  HazardEvaluationResult evaluate(
      const HazardEvaluationInput& input,
      const ObstaclePerceptionCluster& cluster) const noexcept;
};

}  // namespace ndt_slam
