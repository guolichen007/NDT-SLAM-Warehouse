#include <gtest/gtest.h>

#include "ndt_slam/cargo_motion_corridor.hpp"

namespace ndt_slam {
namespace {

CargoMotionCorridorInput movingInput(float obstacle_x, float obstacle_y) {
  CargoMotionCorridorInput input;
  input.cargo_center_map.setZero();
  input.cargo_velocity_map = Eigen::Vector2f(1.0F, 0.0F);
  input.velocity_valid = true;
  input.cargo_length_m = 1.4F;
  input.cargo_width_m = 0.5F;
  input.cargo_yaw_map_rad = 0.0F;
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

TEST(CargoMotionCorridor, ValidZeroVelocityUsesStationaryGuard) {
  CargoMotionCorridorInput input = movingInput(2.0F, 0.0F);
  input.cargo_velocity_map.setZero();
  input.current_footprint_distance_m = 1.0F;
  const auto decision = evaluateCargoMotionCorridor(
      CargoMotionCorridorConfig{}, input);
  EXPECT_TRUE(decision.valid);
  EXPECT_FALSE(decision.eligible);
  EXPECT_EQ(decision.mode, CargoSafetySpatialMode::STATIONARY_GUARD);
}

TEST(CargoMotionCorridor, StationaryEmergencyShellRemainsEligible) {
  CargoMotionCorridorInput input = movingInput(0.1F, 0.0F);
  input.cargo_velocity_map.setZero();
  input.current_footprint_distance_m = 0.20F;
  const auto decision = evaluateCargoMotionCorridor(
      CargoMotionCorridorConfig{}, input);
  EXPECT_TRUE(decision.eligible);
  EXPECT_EQ(decision.mode, CargoSafetySpatialMode::STATIONARY_GUARD);
}

TEST(CargoMotionCorridor, UsesProjectedObbWidthNotHalfDiagonal) {
  CargoMotionCorridorInput input = movingInput(2.0F, 0.9F);
  input.cargo_length_m = 3.0F;
  input.cargo_width_m = 0.4F;
  input.cargo_yaw_map_rad = 0.0F;
  const auto decision = evaluateCargoMotionCorridor(
      CargoMotionCorridorConfig{}, input);
  EXPECT_LT(decision.corridor_half_width_m, 0.70F);
  EXPECT_FALSE(decision.eligible);
}

TEST(CargoMotionCorridor, RotatedDirectionIsCoordinateIndependent) {
  CargoMotionCorridorInput input = movingInput(0.2F, 2.0F);
  input.cargo_velocity_map = Eigen::Vector2f(0.0F, 1.0F);
  EXPECT_TRUE(evaluateCargoMotionCorridor(
      CargoMotionCorridorConfig{}, input).eligible);
}

}  // namespace
}  // namespace ndt_slam
