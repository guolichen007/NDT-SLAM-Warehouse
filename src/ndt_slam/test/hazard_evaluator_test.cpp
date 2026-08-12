#include <gtest/gtest.h>

#include "ndt_slam/hazard_evaluator.hpp"

namespace ndt_slam {
namespace {

ObstaclePerceptionCluster cluster(float top, float bottom) {
  ObstaclePerceptionCluster value;
  value.footprint_distance_m = 4.0F;
  value.top_z95_m = top;
  value.bottom_z05_m = bottom;
  value.obstacle_uncertainty_m = 0.1F;
  value.vertical_continuity_ratio = 1.0F;
  return value;
}

TEST(HazardEvaluator, ComputesSafeClearanceExactlyOnce) {
  HazardEvaluationInput input;
  input.safe_bottom_z_m = 1.8F;
  input.cargo_max_z_m = 3.0F;
  input.vertical_geometry_valid = true;
  const auto result = HazardEvaluator().evaluate(input, cluster(1.2F, 0.5F));
  ASSERT_TRUE(result.assessment.valid);
  EXPECT_FLOAT_EQ(result.assessment.conservative_clearance_m, 0.5F);
  EXPECT_TRUE(result.low_clearance);
}

TEST(HazardEvaluator, EntirelyAboveCargoIsNotLowClearance) {
  HazardEvaluationInput input;
  input.safe_bottom_z_m = 1.8F;
  input.cargo_max_z_m = 3.0F;
  input.vertical_geometry_valid = true;
  const auto result = HazardEvaluator().evaluate(input, cluster(4.0F, 3.2F));
  ASSERT_TRUE(result.assessment.valid);
  EXPECT_TRUE(result.entirely_above_cargo);
  EXPECT_FALSE(result.low_clearance);
}

TEST(HazardEvaluator, VerticalInvalidPreservesPhysicalClusterButNoAssessment) {
  HazardEvaluationInput input;
  input.vertical_geometry_valid = false;
  const auto result = HazardEvaluator().evaluate(input, cluster(1.2F, 0.5F));
  EXPECT_FALSE(result.assessment.valid);
  EXPECT_EQ(result.reason, "hazard_geometry_invalid");
}

}  // namespace
}  // namespace ndt_slam
