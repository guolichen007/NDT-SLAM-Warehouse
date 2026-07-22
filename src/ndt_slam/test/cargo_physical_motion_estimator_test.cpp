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

TEST(CargoPhysicalMotionEstimatorTest, ConfidenceDecreasesAsSampleGapGrows) {
  CargoPhysicalMotionConfig config;
  config.minimum_valid_confidence = 0.0F;
  CargoPhysicalMotionEstimator fast(config);
  fast.update(pose(1.0, 0.0));
  const float fast_confidence = fast.update(pose(1.05, 0.0)).confidence;
  CargoPhysicalMotionEstimator slow(config);
  slow.update(pose(1.0, 0.0));
  const float slow_confidence = slow.update(pose(1.4, 0.0)).confidence;
  EXPECT_GT(fast_confidence, slow_confidence);
}

TEST(CargoPhysicalMotionEstimatorTest, NearStaleSampleHasLowConfidence) {
  CargoPhysicalMotionConfig config;
  config.minimum_valid_confidence = 0.0F;
  CargoPhysicalMotionEstimator estimator(config);
  estimator.update(pose(1.0, 0.0));
  const auto result = estimator.update(pose(1.49, 0.0));
  EXPECT_LT(result.confidence, 0.20F);
}

TEST(CargoPhysicalMotionEstimatorTest, RawDriftSpikeDoesNotForceMoving) {
  CargoPhysicalMotionConfig config;
  config.enter_stationary_confirm_sec = 0.1;
  config.velocity_filter_alpha = 1.0F;
  CargoPhysicalMotionEstimator estimator(config);
  estimator.update(pose(1.0, 0.0));
  estimator.update(pose(1.1, 0.0));
  ASSERT_EQ(estimator.update(pose(1.2, 0.0)).state,
            CargoPhysicalMotionState::STATIONARY);
  const auto rejected = estimator.update(pose(1.3, 10.0));
  EXPECT_EQ(rejected.state, CargoPhysicalMotionState::STATIONARY);
  EXPECT_EQ(rejected.reason, "raw_pose_drift_spike_rejected_short_hold");
}

TEST(CargoPhysicalMotionEstimatorTest,
     DegeneratePoseDoesNotResetPhysicalHistory) {
  CargoPhysicalMotionConfig config;
  config.enter_stationary_confirm_sec = 0.1;
  CargoPhysicalMotionEstimator estimator(config);
  estimator.update(pose(1.0, 0.0));
  estimator.update(pose(1.1, 0.0));
  ASSERT_EQ(estimator.update(pose(1.2, 0.0)).state,
            CargoPhysicalMotionState::STATIONARY);
  auto degenerate = pose(1.3, 1.0);
  degenerate.localization_degenerate = true;
  degenerate.raw_pose_innovation_m = 1.0F;
  const auto held = estimator.update(degenerate);
  EXPECT_EQ(held.state, CargoPhysicalMotionState::STATIONARY);
  EXPECT_EQ(held.reason, "degenerate_raw_pose_rejected_short_hold");
  EXPECT_EQ(estimator.update(pose(1.4, 0.0)).state,
            CargoPhysicalMotionState::STATIONARY);
}

TEST(CargoPhysicalMotionEstimatorTest, ExternalControllerOverridesRawPose) {
  CargoPhysicalMotionEstimator estimator;
  estimator.update(pose(1.0, 0.0));
  auto input = pose(1.1, 100.0);
  input.external_state_valid = true;
  input.external_state = CargoPhysicalMotionState::MOVING;
  const auto result = estimator.update(input);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.state, CargoPhysicalMotionState::MOVING);
  EXPECT_EQ(result.source, CargoPhysicalMotionSource::EXTERNAL_CONTROLLER);
}

TEST(CargoPhysicalMotionEstimatorTest,
     RepeatedThresholdNoiseDoesNotToggleStationary) {
  CargoPhysicalMotionConfig config;
  config.enter_stationary_confirm_sec = 0.1;
  config.exit_stationary_confirm_sec = 0.2;
  config.velocity_filter_alpha = 1.0F;
  CargoPhysicalMotionEstimator estimator(config);
  estimator.update(pose(1.0, 0.0));
  estimator.update(pose(1.1, 0.0));
  ASSERT_EQ(estimator.update(pose(1.2, 0.0)).state,
            CargoPhysicalMotionState::STATIONARY);
  EXPECT_EQ(estimator.update(pose(1.3, 0.006)).state,
            CargoPhysicalMotionState::STATIONARY);
  EXPECT_EQ(estimator.update(pose(1.4, 0.013)).state,
            CargoPhysicalMotionState::STATIONARY);
}

}  // namespace
}  // namespace ndt_slam
