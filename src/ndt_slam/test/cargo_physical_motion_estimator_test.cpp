#include "ndt_slam/cargo_physical_motion_estimator.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

CargoPhysicalMotionInput pose(double stamp, double x) {
  CargoPhysicalMotionInput input;
  input.stamp_sec = stamp;
  input.raw_position_valid = true;
  input.raw_position = Eigen::Vector2d(x, 0.0);
  return input;
}

TEST(CargoPhysicalMotionEstimatorTest, StationaryTransitionOccursOnce) {
  CargoPhysicalMotionConfig config;
  config.enter_stationary_confirm_sec = 0.2;
  config.velocity_filter_alpha = 1.0F;
  CargoPhysicalMotionEstimator estimator(config);
  estimator.update(pose(1.0, 0.0));
  EXPECT_EQ(estimator.update(pose(1.1, 0.0)).state,
            CargoPhysicalMotionState::STOPPING_SETTLE);
  EXPECT_EQ(estimator.update(pose(1.3, 0.0)).state,
            CargoPhysicalMotionState::STATIONARY);
  const auto held = estimator.update(pose(1.4, 0.0));
  EXPECT_EQ(held.state, CargoPhysicalMotionState::STATIONARY);
  EXPECT_GT(held.state_duration_sec, 0.0);
}

TEST(CargoPhysicalMotionEstimatorTest, RawMotionExitsStationary) {
  CargoPhysicalMotionConfig config;
  config.enter_stationary_confirm_sec = 0.1;
  config.exit_stationary_confirm_sec = 0.1;
  config.velocity_filter_alpha = 1.0F;
  CargoPhysicalMotionEstimator estimator(config);
  estimator.update(pose(1.0, 0.0));
  estimator.update(pose(1.1, 0.0));
  ASSERT_EQ(estimator.update(pose(1.2, 0.0)).state,
            CargoPhysicalMotionState::STATIONARY);
  estimator.update(pose(1.3, 0.02));
  EXPECT_EQ(estimator.update(pose(1.41, 0.042)).state,
            CargoPhysicalMotionState::MOVING);
}

TEST(CargoPhysicalMotionEstimatorTest, ShortUnknownDoesNotReset) {
  CargoPhysicalMotionEstimator estimator;
  estimator.update(pose(1.0, 0.0));
  ASSERT_TRUE(estimator.update(pose(1.1, 0.02)).valid);
  CargoPhysicalMotionInput missing;
  missing.stamp_sec = 1.2;
  const auto held = estimator.update(missing);
  EXPECT_TRUE(held.valid);
  EXPECT_EQ(held.reason, "short_unknown_motion_input_hold");
}

TEST(CargoPhysicalMotionEstimatorTest, LongGapInvalidatesMotionState) {
  CargoPhysicalMotionEstimator estimator;
  estimator.update(pose(1.0, 0.0));
  estimator.update(pose(1.1, 0.02));
  const auto result = estimator.update(pose(2.0, 0.02));
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.state, CargoPhysicalMotionState::UNKNOWN);
}

TEST(CargoPhysicalMotionEstimatorTest, TimestampRollbackStartsNewEpoch) {
  CargoPhysicalMotionEstimator estimator;
  estimator.update(pose(10.0, 0.0));
  const auto result = estimator.update(pose(1.0, 0.0));
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.source_epoch, 1U);
}

}  // namespace
}  // namespace ndt_slam
