#include "ndt_slam/cargo_swing_monitor.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

CargoSwingInput validInput(double stamp) {
  CargoSwingInput input;
  input.stamp_sec = stamp;
  input.localization_valid = true;
  input.hook_loaded = true;
  input.hook_anchor_valid = true;
  input.hook_anchor_base = Eigen::Vector3f(0.0F, 0.0F, 2.0F);
  input.track_retained = true;
  input.track_locked = true;
  input.observation_associated_current = true;
  input.cargo_lifecycle_id = 2U;
  input.track_segment_id = 3U;
  input.measured_center_valid = true;
  input.measured_center_base = Eigen::Vector3f(0.0F, 0.0F, 1.0F);
  input.identity_confidence = 0.9F;
  input.shape_confidence = 0.9F;
  input.horizontal_tracking_residual_m = 0.02F;
  input.orientation_confidence = 0.9F;
  input.cargo_length_m = 2.0F;
  input.cargo_width_m = 1.0F;
  return input;
}

TEST(CargoSwingMonitorTest, ExtremeOffsetAlarmsOnFirstMeasurement) {
  CargoSwingMonitor monitor;
  auto input = validInput(1.0);
  input.measured_center_base.x() = 0.8F;
  const auto result = monitor.update(input);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.sway_state, CargoSwayState::SWAY_ALARM);
  EXPECT_TRUE(result.angle_authoritative);
}

TEST(CargoSwingMonitorTest, PoorTrackingQualityCannotUpdateHistory) {
  CargoSwingMonitor monitor;
  auto input = validInput(1.0);
  input.identity_confidence = 0.2F;
  const auto result = monitor.update(input);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.observation_state,
            CargoSwingObservationState::INVALID);
  EXPECT_EQ(result.reason, "tracking_quality_insufficient");
}

TEST(CargoSwingMonitorTest, ConfiguredRopeIsNotAngleAuthoritative) {
  CargoSwingMonitor monitor;
  auto input = validInput(1.0);
  input.measured_center_base.z() = 1.9F;
  const auto result = monitor.update(input);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.rope_length_source,
            CargoRopeLengthSource::CONFIG_FALLBACK);
  EXPECT_FALSE(result.angle_authoritative);
}

TEST(CargoSwingMonitorTest,
     ImmediateAlarmDoesNotDisappearBeforeHistoryMatures) {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 1.0;
  CargoSwingMonitor monitor(config);
  auto extreme = validInput(1.0);
  extreme.measured_center_base.x() = 0.8F;
  ASSERT_EQ(monitor.update(extreme).sway_state,
            CargoSwayState::SWAY_ALARM);
  auto recovered = validInput(1.1);
  recovered.measured_center_base.x() = 0.02F;
  const auto result = monitor.update(recovered);
  EXPECT_EQ(result.sway_state, CargoSwayState::SWAY_ALARM);
}

TEST(CargoSwingMonitorTest, ImmediateAlarmTransitionsThroughSettling) {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.0;
  config.minimum_alarm_hold_sec = 0.1;
  config.sway_end_confirm_sec = 0.2;
  config.measurement_filter_alpha = 1.0F;
  CargoSwingMonitor monitor(config);
  auto extreme = validInput(1.0);
  extreme.measured_center_base.x() = 0.8F;
  ASSERT_EQ(monitor.update(extreme).sway_state,
            CargoSwayState::SWAY_ALARM);
  auto calm = validInput(1.1);
  calm.measured_center_base.x() = 0.0F;
  monitor.update(calm);
  calm.stamp_sec = 1.2;
  monitor.update(calm);
  calm.stamp_sec = 1.41;
  EXPECT_EQ(monitor.update(calm).sway_state,
            CargoSwayState::SETTLING);
  calm.stamp_sec = 1.62;
  EXPECT_EQ(monitor.update(calm).sway_state,
            CargoSwayState::NORMAL);
}

TEST(CargoSwingMonitorTest, ShortGapDoesNotClearLatchedAlarm) {
  CargoSwingMonitor monitor;
  auto extreme = validInput(1.0);
  extreme.measured_center_base.x() = 0.8F;
  ASSERT_EQ(monitor.update(extreme).sway_state,
            CargoSwayState::SWAY_ALARM);
  auto gap = validInput(1.1);
  gap.observation_associated_current = false;
  const auto result = monitor.update(gap);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.observation_state,
            CargoSwingObservationState::SHORT_GAP_HOLD);
  EXPECT_EQ(result.sway_state, CargoSwayState::SWAY_ALARM);
}

TEST(CargoSwingMonitorTest,
     ConfiguredRopeCannotCauseAngleOnlyFormalSwayAlarm) {
  CargoSwingConfig config;
  config.configured_sling_length_m = 0.30F;
  config.minimum_rope_length_m = 0.30F;
  config.minimum_valid_observation_sec = 0.2;
  CargoSwingMonitor monitor(config);
  for (int index = 0; index < 4; ++index) {
    auto input = validInput(1.0 + 0.1 * index);
    input.measured_center_base.x() = 0.04F;
    input.measured_center_base.z() = 1.9F;
    const auto result = monitor.update(input);
    if (index == 3) {
      EXPECT_FALSE(result.angle_authoritative);
      EXPECT_NE(result.sway_state, CargoSwayState::SWAY_ALARM);
    }
  }
}

TEST(CargoSwingMonitorTest, ConfiguredRopeCanAlarmOnAbsoluteOffset) {
  CargoSwingMonitor monitor;
  auto input = validInput(1.0);
  input.measured_center_base.x() = 0.8F;
  input.measured_center_base.z() = 1.9F;
  const auto result = monitor.update(input);
  EXPECT_FALSE(result.angle_authoritative);
  EXPECT_EQ(result.sway_state, CargoSwayState::SWAY_ALARM);
}

TEST(CargoSwingMonitorTest, MeasuredRopeCanAlarmOnAngle) {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.2;
  CargoSwingMonitor monitor(config);
  CargoSwingResult result;
  for (int index = 0; index < 4; ++index) {
    auto input = validInput(1.0 + 0.1 * index);
    input.measured_center_base.x() = 0.10F;
    result = monitor.update(input);
  }
  EXPECT_TRUE(result.angle_authoritative);
  EXPECT_EQ(result.sway_state, CargoSwayState::SWAY_ALARM);
}

TEST(CargoSwingMonitorTest, SkewOffsetAlarmRequiresFreshHoistUp) {
  CargoSwingMonitor monitor;
  auto input = validInput(1.0);
  input.measured_center_base.x() = 0.8F;
  auto result = monitor.update(input);
  EXPECT_EQ(result.skew_pull_state,
            CargoSkewPullState::SKEW_PULL_SUSPECTED);
  EXPECT_TRUE(result.alarm_inhibited);
  input.stamp_sec = 1.1;
  input.hoist_state_fresh = true;
  input.hoist_motion_state = HoistMotionState::UP;
  result = monitor.update(input);
  EXPECT_EQ(result.skew_pull_state,
            CargoSkewPullState::SKEW_PULL_ALARM);
}

TEST(CargoSwingMonitorTest, PersistentDirectionalOffsetBecomesSkew) {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.2;
  config.skew_min_duration_sec = 0.2;
  config.skew_alarm_confirm_sec = 0.0;
  CargoSwingMonitor monitor(config);
  CargoSwingResult result;
  for (int index = 0; index < 5; ++index) {
    auto input = validInput(1.0 + 0.1 * index);
    input.measured_center_base.x() = 0.6F;
    input.hoist_state_fresh = true;
    input.hoist_motion_state = HoistMotionState::UP;
    result = monitor.update(input);
  }
  EXPECT_EQ(result.skew_pull_state,
            CargoSkewPullState::SKEW_PULL_ALARM);
  EXPECT_GT(result.dc_to_ac_ratio, config.skew_min_dc_to_ac_ratio);
}

TEST(CargoSwingMonitorTest, OscillatingOffsetBecomesSwayNotSkew) {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.2;
  config.skew_min_duration_sec = 0.2;
  CargoSwingMonitor monitor(config);
  CargoSwingResult result;
  for (int index = 0; index < 7; ++index) {
    auto input = validInput(1.0 + 0.1 * index);
    input.measured_center_base.x() = index % 2 == 0 ? 0.25F : -0.25F;
    input.measured_center_base.z() = 1.9F;  // diagnostic rope fallback
    result = monitor.update(input);
  }
  EXPECT_TRUE(result.sway_state == CargoSwayState::SWAY_WARNING ||
              result.sway_state == CargoSwayState::SWAY_DETECTED);
  EXPECT_NE(result.skew_pull_state,
            CargoSkewPullState::SKEW_PULL_ALARM);
  EXPECT_GT(result.zero_crossings, config.skew_max_zero_crossings);
}

TEST(CargoSwingMonitorTest, NearSquareCargoCannotAuthorizeTorsion) {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.0;
  CargoSwingMonitor monitor(config);
  auto input = validInput(1.0);
  input.locked_yaw_valid = true;
  input.measured_yaw_valid = true;
  input.measured_yaw_base_rad = 0.5F;
  input.cargo_length_m = 1.0F;
  input.cargo_width_m = 0.95F;
  const auto result = monitor.update(input);
  EXPECT_EQ(result.torsion_state,
            CargoTorsionState::NOT_EVALUATED);
}

}  // namespace
}  // namespace ndt_slam
