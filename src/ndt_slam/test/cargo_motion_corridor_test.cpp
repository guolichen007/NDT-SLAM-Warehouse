#include <gtest/gtest.h>

#include "ndt_slam/cargo_motion_corridor.hpp"

namespace ndt_slam {
namespace {

CargoMotionCorridorInput movingInput(float obstacle_x, float obstacle_y) {
  CargoMotionCorridorInput input;
  input.cargo_center_map.setZero();
  input.cargo_velocity_map = Eigen::Vector2f(1.0F, 0.0F);
  input.velocity_valid = true;
  input.cargo_half_diagonal_m = 0.75F;
  input.horizontal_uncertainty_m = 0.10F;
  input.obstacle_nearest_map = Eigen::Vector2f(obstacle_x, obstacle_y);
  input.obstacle_centroid_map = input.obstacle_nearest_map;
  input.current_footprint_distance_m = 1.0F;
  return input;
}

TEST(CargoMotionCorridor, ForwardObstacleIsEligible) {
  const auto decision = evaluateCargoMotionCorridor(
      CargoMotionCorridorConfig{}, movingInput(2.0F, 0.2F));
  EXPECT_TRUE(decision.valid);
  EXPECT_TRUE(decision.eligible);
  EXPECT_EQ(decision.mode, CargoSafetySpatialMode::MOTION_CORRIDOR);
}

TEST(CargoMotionCorridor, SideRearObstacleIsExcluded) {
  const auto decision = evaluateCargoMotionCorridor(
      CargoMotionCorridorConfig{}, movingInput(-2.0F, 2.0F));
  EXPECT_TRUE(decision.valid);
  EXPECT_FALSE(decision.eligible);
  EXPECT_EQ(decision.reason, "obstacle_outside_motion_corridor");
}

TEST(CargoMotionCorridor, ImmediateNearFieldAlwaysWins) {
  CargoMotionCorridorInput input = movingInput(-2.0F, 2.0F);
  input.current_footprint_distance_m = 0.20F;
  EXPECT_TRUE(evaluateCargoMotionCorridor(
      CargoMotionCorridorConfig{}, input).eligible);
}

TEST(CargoMotionCorridor, MissingVelocityIsExplicitRadialFallback) {
  CargoMotionCorridorInput input = movingInput(-2.0F, 2.0F);
  input.velocity_valid = false;
  const auto decision = evaluateCargoMotionCorridor(
      CargoMotionCorridorConfig{}, input);
  EXPECT_TRUE(decision.eligible);
  EXPECT_EQ(decision.mode, CargoSafetySpatialMode::RADIAL_FALLBACK);
}

TEST(CargoMotionCorridor, RotatedDirectionIsCoordinateIndependent) {
  CargoMotionCorridorInput input = movingInput(0.2F, 2.0F);
  input.cargo_velocity_map = Eigen::Vector2f(0.0F, 1.0F);
  EXPECT_TRUE(evaluateCargoMotionCorridor(
      CargoMotionCorridorConfig{}, input).eligible);
}

}  // namespace
}  // namespace ndt_slam
