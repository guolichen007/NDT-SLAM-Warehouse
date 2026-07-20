#include <gtest/gtest.h>

#include "ndt_slam/cargo_component_fusion.hpp"

namespace ndt_slam {
namespace {

CargoComponentFragment fragment(float x, float y, float yaw = 0.0F) {
  CargoComponentFragment value;
  value.center = Eigen::Vector2f(x, y);
  value.length_m = 0.8F;
  value.width_m = 0.35F;
  value.yaw_rad = yaw;
  value.min_z = 1.0F;
  value.max_z = 1.8F;
  value.point_count = 40U;
  return value;
}

bool containsGroup(const std::vector<CargoComponentHypothesis>& hypotheses,
                   std::size_t size) {
  for (const CargoComponentHypothesis& hypothesis : hypotheses) {
    if (hypothesis.component_indices.size() == size) return true;
  }
  return false;
}

TEST(CargoComponentFusion, CollinearSplitComponentsCanMergeIntoLongCargo) {
  const auto hypotheses = buildCargoComponentHypotheses(
      {fragment(-0.75F, 0.0F), fragment(0.0F, 0.02F),
       fragment(0.75F, -0.02F)},
      CargoComponentFusionConfig{});
  EXPECT_TRUE(containsGroup(hypotheses, 2U));
  EXPECT_TRUE(containsGroup(hypotheses, 3U));
}

TEST(CargoComponentFusion, RackAndCargoComponentsCannotMerge) {
  const auto hypotheses = buildCargoComponentHypotheses(
      {fragment(0.0F, 0.0F), fragment(0.0F, 1.2F)},
      CargoComponentFusionConfig{});
  EXPECT_FALSE(containsGroup(hypotheses, 2U));
}

TEST(CargoComponentFusion, IncompatibleHeightCannotMerge) {
  CargoComponentFragment lhs = fragment(0.0F, 0.0F);
  CargoComponentFragment rhs = fragment(0.8F, 0.0F);
  rhs.min_z = 2.0F;
  rhs.max_z = 2.5F;
  const auto hypotheses = buildCargoComponentHypotheses(
      {lhs, rhs}, CargoComponentFusionConfig{});
  EXPECT_FALSE(containsGroup(hypotheses, 2U));
}

TEST(CargoComponentFusion, DiagonalUnrelatedComponentCannotMerge) {
  constexpr float kHalfPi = 1.57079632679489661923F;
  const auto hypotheses = buildCargoComponentHypotheses(
      {fragment(0.0F, 0.0F), fragment(0.8F, 0.0F, kHalfPi)},
      CargoComponentFusionConfig{});
  EXPECT_FALSE(containsGroup(hypotheses, 2U));
}

}  // namespace
}  // namespace ndt_slam
