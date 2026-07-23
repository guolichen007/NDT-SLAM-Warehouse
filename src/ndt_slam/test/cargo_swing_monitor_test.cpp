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
  input.hook_anchor_source = "topic";
  input.hook_anchor_authority =
      CargoHookAnchorAuthority::TOPIC_MEASURED;
  input.hook_anchor_xy_authoritative = true;
  input.hook_anchor_z_authoritative = true;
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
  input.hoist_state_available = true;
  return input;
}

TEST(CargoSwingMonitorTest, ExtremeOffsetAlarmsOnFirstMeasurement) {
  CargoSwingMonitor monitor;
  auto input = validInput(1.0);
  input.measured_center_base.x() = 0.8F;
  const auto result = monitor.update(input);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.sway_state, CargoSwayState::SWAY_ALARM);
  EXPECT_NE(result.skew_pull_state,
            CargoSkewPullState::SKEW_PULL_ALARM);
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
  EXPECT_EQ(result.skew_pull_readiness,
            CargoSkewPullReadiness::ROPE_LENGTH_MISSING);
}

TEST(CargoSwingMonitorTest, NonAuthoritativeAnchorZUsesConfiguredRope) {
  CargoSwingConfig config;
  config.configured_sling_length_m = 4.0F;
  CargoSwingMonitor monitor(config);
  auto input = validInput(1.0);
  input.hoist_state_fresh = true;
  input.hook_anchor_z_authoritative = false;
  const auto result = monitor.update(input);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.rope_length_source,
            CargoRopeLengthSource::CONFIG_FALLBACK);
  EXPECT_FLOAT_EQ(result.rope_length_m, 4.0F);
  EXPECT_FALSE(result.angle_authoritative);
  EXPECT_EQ(result.skew_pull_readiness,
            CargoSkewPullReadiness::ANGLE_NONAUTHORITATIVE);
}

TEST(CargoSwingMonitorTest, ReadinessReportsMissingHoistSource) {
  CargoSwingMonitor monitor;
  auto input = validInput(1.0);
  input.hoist_state_available = false;
  input.hoist_state_fresh = false;
  const auto result = monitor.update(input);
  EXPECT_EQ(result.skew_pull_readiness,
            CargoSkewPullReadiness::HOIST_MISSING);
  EXPECT_STREQ(
      cargoSkewPullReadinessName(result.skew_pull_readiness),
      "HOIST_MISSING");
}

TEST(CargoSwingMonitorTest, ReadinessRequiresAuthoritativeHookAnchor) {
  CargoSwingMonitor monitor;
  auto input = validInput(1.0);
  input.hoist_state_fresh = true;
  input.hook_anchor_xy_authoritative = false;
  const auto result = monitor.update(input);
  EXPECT_EQ(
      result.skew_pull_readiness,
      CargoSkewPullReadiness::HOOK_ANCHOR_NONAUTHORITATIVE);
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
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.0;
  config.skew_min_duration_sec = 0.1;
  config.skew_alarm_confirm_sec = 0.0;
  CargoSwingMonitor monitor(config);
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

TEST(CargoSwingMonitorTest,
     ConfiguredHookAnchorIsNotAngleAuthoritativeByDefault) {
  CargoSwingMonitor monitor;
  auto input = validInput(1.0);
  input.hook_anchor_source = "config";
  input.hook_anchor_authority =
      CargoHookAnchorAuthority::CONFIG_DIAGNOSTIC;
  input.hook_anchor_xy_authoritative = false;
  input.hook_anchor_z_authoritative = false;
  input.measured_center_base.x() = 0.10F;
  const auto result = monitor.update(input);
  EXPECT_FALSE(result.angle_authoritative);
  EXPECT_FALSE(result.offset_authoritative);
  EXPECT_GT(result.angle_deg, 0.0F);
}

TEST(CargoSwingMonitorTest, ConfiguredHookAnchorCannotCauseFormalSkewAlarm) {
  CargoSwingMonitor monitor;
  auto input = validInput(1.0);
  input.hook_anchor_source = "config";
  input.hook_anchor_authority =
      CargoHookAnchorAuthority::CONFIG_DIAGNOSTIC;
  input.hook_anchor_xy_authoritative = false;
  input.hook_anchor_z_authoritative = false;
  input.hoist_state_fresh = true;
  input.hoist_motion_state = HoistMotionState::UP;
  input.measured_center_base.x() = 0.8F;
  const auto result = monitor.update(input);
  EXPECT_EQ(result.skew_pull_state,
            CargoSkewPullState::SKEW_PULL_SUSPECTED);
  EXPECT_TRUE(result.alarm_inhibited);
  EXPECT_EQ(result.recommended_action,
            CargoSwingRecommendedAction::INHIBIT_HOIST_UP);
}

TEST(CargoSwingMonitorTest, FreshMeasuredHookAnchorCanAuthorizeAngle) {
  CargoSwingMonitor monitor;
  auto input = validInput(1.0);
  input.measured_center_base.x() = 0.10F;
  const auto result = monitor.update(input);
  EXPECT_TRUE(result.angle_authoritative);
  EXPECT_EQ(result.hook_anchor_authority,
            CargoHookAnchorAuthority::TOPIC_MEASURED);
}

TEST(CargoSwingMonitorTest,
     FreshControllerAnchorCanAuthorizeOffsetSkew) {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.0;
  config.skew_min_duration_sec = 0.0;
  config.skew_alarm_confirm_sec = 0.0;
  CargoSwingMonitor monitor(config);
  auto input = validInput(1.0);
  input.hook_anchor_source = "controller";
  input.hook_anchor_authority =
      CargoHookAnchorAuthority::EXTERNAL_CONTROLLER;
  input.hook_anchor_xy_authoritative = true;
  input.hook_anchor_z_authoritative = true;
  input.hoist_state_fresh = true;
  input.hoist_motion_state = HoistMotionState::UP;
  input.measured_center_base.x() = 0.8F;
  EXPECT_EQ(monitor.update(input).skew_pull_state,
            CargoSkewPullState::SKEW_PULL_ALARM);
}

TEST(CargoSwingMonitorTest,
     ConfiguredAnchorStillPublishesDiagnosticAngleAndOffset) {
  CargoSwingMonitor monitor;
  auto input = validInput(1.0);
  input.hook_anchor_source = "config";
  input.hook_anchor_authority =
      CargoHookAnchorAuthority::CONFIG_DIAGNOSTIC;
  input.hook_anchor_xy_authoritative = false;
  input.hook_anchor_z_authoritative = false;
  input.measured_center_base.x() = 0.20F;
  const auto result = monitor.update(input);
  EXPECT_FALSE(result.angle_authoritative);
  EXPECT_FALSE(result.offset_authoritative);
  EXPECT_NEAR(result.offset_m, 0.20F, 1.0e-5F);
  EXPECT_GT(result.angle_deg, 0.0F);
}

TEST(CargoSwingMonitorTest, StaleHookAnchorCannotAuthorizeAlarm) {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.0;
  config.skew_min_duration_sec = 0.0;
  config.skew_alarm_confirm_sec = 0.0;
  CargoSwingMonitor monitor(config);
  auto alarm = validInput(1.0);
  alarm.hoist_state_fresh = true;
  alarm.hoist_motion_state = HoistMotionState::UP;
  alarm.measured_center_base.x() = 0.8F;
  ASSERT_EQ(monitor.update(alarm).skew_pull_state,
            CargoSkewPullState::SKEW_PULL_ALARM);
  auto stale = alarm;
  stale.stamp_sec = 1.1;
  stale.hook_anchor_valid = false;
  const auto result = monitor.update(stale);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.skew_pull_state,
            CargoSkewPullState::SKEW_PULL_ALARM);
}

TEST(CargoSwingMonitorTest, SkewAlarmDoesNotDropOnOneCalmFrame) {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.0;
  config.skew_min_duration_sec = 0.0;
  config.skew_alarm_confirm_sec = 0.0;
  config.measurement_filter_alpha = 1.0F;
  CargoSwingMonitor monitor(config);
  auto alarm = validInput(1.0);
  alarm.hoist_state_fresh = true;
  alarm.hoist_motion_state = HoistMotionState::UP;
  alarm.measured_center_base.x() = 0.8F;
  ASSERT_EQ(monitor.update(alarm).skew_pull_state,
            CargoSkewPullState::SKEW_PULL_ALARM);
  auto calm = validInput(1.1);
  calm.hoist_state_fresh = true;
  calm.hoist_motion_state = HoistMotionState::STOPPED;
  EXPECT_EQ(monitor.update(calm).skew_pull_state,
            CargoSkewPullState::SKEW_PULL_ALARM);
}

TEST(CargoSwingMonitorTest, ShortGapDoesNotClearSkewAlarm) {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.0;
  config.skew_min_duration_sec = 0.0;
  config.skew_alarm_confirm_sec = 0.0;
  CargoSwingMonitor monitor(config);
  auto input = validInput(1.0);
  input.hoist_state_fresh = true;
  input.hoist_motion_state = HoistMotionState::UP;
  input.measured_center_base.x() = 0.8F;
  ASSERT_EQ(monitor.update(input).skew_pull_state,
            CargoSkewPullState::SKEW_PULL_ALARM);
  input.stamp_sec = 1.1;
  input.observation_associated_current = false;
  const auto held = monitor.update(input);
  EXPECT_TRUE(held.valid);
  EXPECT_EQ(held.skew_pull_state,
            CargoSkewPullState::SKEW_PULL_ALARM);
}

TEST(CargoSwingMonitorTest, SkewAlarmClearsAfterConfirmedRecovery) {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.0;
  config.skew_min_duration_sec = 0.0;
  config.skew_alarm_confirm_sec = 0.0;
  config.measurement_filter_alpha = 1.0F;
  config.skew_alarm_hold_sec = 0.1;
  config.skew_clear_confirm_sec = 0.1;
  config.maximum_observation_gap_sec = 0.3;
  config.history_window_sec = 0.3;  // flush old alarm samples quickly
  CargoSwingMonitor monitor(config);
  auto input = validInput(1.0);
  input.hoist_state_fresh = true;
  input.hoist_motion_state = HoistMotionState::UP;
  input.measured_center_base.x() = 0.8F;
  ASSERT_EQ(monitor.update(input).skew_pull_state,
            CargoSkewPullState::SKEW_PULL_ALARM);
  input.measured_center_base.x() = 0.0F;
  input.hoist_motion_state = HoistMotionState::STOPPED;
  // Feed calm frames: first flushes old alarm from history window,
  // second starts clear candidate, third confirms the clear.
  for (double t = 1.1; t <= 1.5; t += 0.1) {
    input.stamp_sec = t;
    monitor.update(input);
  }
  EXPECT_NE(monitor.update(input).skew_pull_state,
            CargoSkewPullState::SKEW_PULL_ALARM);
}

TEST(CargoSwingMonitorTest, TorsionAlarmUsesHysteresis) {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.0;
  config.torsion_clear_confirm_sec = 0.2;
  CargoSwingMonitor monitor(config);
  auto input = validInput(1.0);
  input.locked_yaw_valid = true;
  input.measured_yaw_valid = true;
  input.locked_yaw_base_rad = 0.0F;
  input.measured_yaw_base_rad = 0.25F;
  ASSERT_EQ(monitor.update(input).torsion_state,
            CargoTorsionState::TORSION_ALARM);
  input.measured_yaw_base_rad = 0.0F;
  input.stamp_sec = 1.1;
  EXPECT_EQ(monitor.update(input).torsion_state,
            CargoTorsionState::TORSION_ALARM);
  input.stamp_sec = 1.31;
  EXPECT_NE(monitor.update(input).torsion_state,
            CargoTorsionState::TORSION_ALARM);
}

TEST(CargoSwingMonitorTest,
     LostOrientationEvidenceDoesNotPublishFalseNormal) {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.0;
  config.maximum_alarm_evidence_hold_sec = 0.3;
  CargoSwingMonitor monitor(config);
  auto input = validInput(1.0);
  input.locked_yaw_valid = true;
  input.measured_yaw_valid = true;
  input.locked_yaw_base_rad = 0.0F;
  input.measured_yaw_base_rad = 0.25F;
  ASSERT_EQ(monitor.update(input).torsion_state,
            CargoTorsionState::TORSION_ALARM);
  input.stamp_sec = 1.1;
  input.measured_yaw_valid = false;
  EXPECT_EQ(monitor.update(input).torsion_state,
            CargoTorsionState::TORSION_ALARM);
  input.stamp_sec = 1.5;
  const auto stale = monitor.update(input);
  // P1-2: torsion evidence stale must not invalidate the entire status.
  // Sway and skew channels remain valid; only torsion reports NOT_EVALUATED.
  EXPECT_TRUE(stale.valid);
  EXPECT_EQ(stale.torsion_state,
            CargoTorsionState::NOT_EVALUATED);
  EXPECT_NE(stale.reason.find("torsion_orientation_evidence_stale"),
            std::string::npos);
}

TEST(CargoSwingMonitorTest, SkewAlarmRecommendsStopAndSettle) {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.0;
  config.skew_min_duration_sec = 0.0;
  config.skew_alarm_confirm_sec = 0.0;
  CargoSwingMonitor monitor(config);
  auto input = validInput(1.0);
  input.hoist_state_fresh = true;
  input.hoist_motion_state = HoistMotionState::UP;
  input.measured_center_base.x() = 0.8F;
  const auto result = monitor.update(input);
  EXPECT_EQ(result.skew_pull_state,
            CargoSkewPullState::SKEW_PULL_ALARM);
  EXPECT_EQ(result.recommended_action,
            CargoSwingRecommendedAction::STOP_AND_SETTLE);
  EXPECT_EQ(result.reason, "stop_hoist_and_travel");
}

// ─── P1-1: stale/unknown hoist cannot clear latched skew alarm ───

CargoSwingConfig fastSkewAlarmConfig() {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.0;
  config.skew_min_duration_sec = 0.0;
  config.skew_alarm_confirm_sec = 0.0;
  config.measurement_filter_alpha = 1.0F;
  config.skew_alarm_hold_sec = 0.1;
  config.skew_clear_confirm_sec = 0.2;
  config.maximum_observation_gap_sec = 0.3;
  config.history_window_sec = 0.3;  // flush old alarm samples quickly
  return config;
}

void feedSkewAlarm(CargoSwingMonitor* monitor, double start_stamp) {
  auto input = validInput(start_stamp);
  input.hoist_state_fresh = true;
  input.hoist_motion_state = HoistMotionState::UP;
  input.measured_center_base.x() = 0.8F;
  ASSERT_EQ(monitor->update(input).skew_pull_state,
            CargoSkewPullState::SKEW_PULL_ALARM);
}

TEST(CargoSwingMonitorTest, StaleHoistCannotClearLatchedSkewAlarm) {
  CargoSwingConfig config = fastSkewAlarmConfig();
  CargoSwingMonitor monitor(config);
  feedSkewAlarm(&monitor, 1.0);

  // hoist_state_fresh = false → stale hoist → must not clear alarm
  auto input = validInput(1.2);
  input.hoist_state_fresh = false;  // stale
  input.measured_center_base.x() = 0.0F;
  // Feed several frames past hold + clear confirm
  for (double t = 1.3; t <= 1.7; t += 0.1) {
    input.stamp_sec = t;
    const auto result = monitor.update(input);
    EXPECT_EQ(result.skew_pull_state,
              CargoSkewPullState::SKEW_PULL_ALARM)
        << "stale hoist cleared alarm at t=" << t;
  }
}

TEST(CargoSwingMonitorTest, UnknownHoistCannotClearLatchedSkewAlarm) {
  CargoSwingConfig config = fastSkewAlarmConfig();
  CargoSwingMonitor monitor(config);
  feedSkewAlarm(&monitor, 1.0);

  auto input = validInput(1.2);
  input.hoist_state_fresh = true;
  input.hoist_motion_state = HoistMotionState::UNKNOWN;
  input.measured_center_base.x() = 0.0F;
  for (double t = 1.3; t <= 1.7; t += 0.1) {
    input.stamp_sec = t;
    const auto result = monitor.update(input);
    EXPECT_EQ(result.skew_pull_state,
              CargoSkewPullState::SKEW_PULL_ALARM)
        << "UNKNOWN hoist cleared alarm at t=" << t;
  }
}

TEST(CargoSwingMonitorTest,
     FreshStoppedAndRecoveredGeometryCanClearSkewAlarm) {
  CargoSwingConfig config = fastSkewAlarmConfig();
  config.skew_clear_confirm_sec = 0.1;
  CargoSwingMonitor monitor(config);
  feedSkewAlarm(&monitor, 1.0);

  // Fresh STOPPED + recovered geometry → clear after confirm
  auto input = validInput(1.2);
  input.hoist_state_fresh = true;
  input.hoist_motion_state = HoistMotionState::STOPPED;
  input.measured_center_base.x() = 0.0F;
  for (double t = 1.2; t <= 1.6; t += 0.1) {
    input.stamp_sec = t;
    monitor.update(input);
  }
  const auto result = monitor.update(input);
  EXPECT_NE(result.skew_pull_state,
            CargoSkewPullState::SKEW_PULL_ALARM);
}

TEST(CargoSwingMonitorTest,
     FreshDownAndRecoveredGeometryCanClearSkewAlarm) {
  CargoSwingConfig config = fastSkewAlarmConfig();
  config.skew_clear_confirm_sec = 0.1;
  CargoSwingMonitor monitor(config);
  feedSkewAlarm(&monitor, 1.0);

  // Fresh DOWN + recovered geometry → clear after confirm
  auto input = validInput(1.2);
  input.hoist_state_fresh = true;
  input.hoist_motion_state = HoistMotionState::DOWN;
  input.measured_center_base.x() = 0.0F;
  for (double t = 1.2; t <= 1.6; t += 0.1) {
    input.stamp_sec = t;
    monitor.update(input);
  }
  const auto result = monitor.update(input);
  EXPECT_NE(result.skew_pull_state,
            CargoSkewPullState::SKEW_PULL_ALARM);
}

TEST(CargoSwingMonitorTest, NonAuthoritativeOffsetCannotClearSkewAlarm) {
  CargoSwingConfig config = fastSkewAlarmConfig();
  CargoSwingMonitor monitor(config);
  feedSkewAlarm(&monitor, 1.0);

  // Fresh STOPPED + low offset but non-authoritative → must NOT clear
  auto input = validInput(1.2);
  input.hoist_state_fresh = true;
  input.hoist_motion_state = HoistMotionState::STOPPED;
  input.hook_anchor_xy_authoritative = false;  // offset not authoritative
  input.measured_center_base.x() = 0.0F;
  for (double t = 1.3; t <= 1.7; t += 0.1) {
    input.stamp_sec = t;
    const auto result = monitor.update(input);
    EXPECT_EQ(result.skew_pull_state,
              CargoSkewPullState::SKEW_PULL_ALARM)
        << "non-authoritative offset cleared alarm at t=" << t;
  }
}

// ─── P1-2: torsion stale channel isolation ───

TEST(CargoSwingMonitorTest,
     TorsionStaleDoesNotSuppressCurrentSwayAlarm) {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.0;
  config.maximum_alarm_evidence_hold_sec = 0.3;
  CargoSwingMonitor monitor(config);

  // First establish a torsion alarm
  auto input = validInput(1.0);
  input.locked_yaw_valid = true;
  input.measured_yaw_valid = true;
  input.locked_yaw_base_rad = 0.0F;
  input.measured_yaw_base_rad = 0.25F;
  ASSERT_EQ(monitor.update(input).torsion_state,
            CargoTorsionState::TORSION_ALARM);

  // Then establish a sway alarm via extreme offset
  input.stamp_sec = 1.1;
  input.measured_center_base.x() = 0.8F;
  const auto sway_alarm = monitor.update(input);
  ASSERT_EQ(sway_alarm.sway_state, CargoSwayState::SWAY_ALARM);

  // Now let torsion evidence go stale while sway remains valid
  input.stamp_sec = 1.2;
  input.measured_yaw_valid = false;  // torsion evidence lost
  input.measured_center_base.x() = 0.8F;  // sway still extreme
  monitor.update(input);
  input.stamp_sec = 1.5;  // past maximum_alarm_evidence_hold_sec
  const auto result = monitor.update(input);

  // P1-2: valid must remain true, sway alarm preserved
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.sway_state, CargoSwayState::SWAY_ALARM);
  EXPECT_EQ(result.torsion_state, CargoTorsionState::NOT_EVALUATED);
}

TEST(CargoSwingMonitorTest,
     TorsionStaleDoesNotSuppressCurrentSkewAlarm) {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.0;
  config.maximum_alarm_evidence_hold_sec = 0.3;
  config.skew_min_duration_sec = 0.0;
  config.skew_alarm_confirm_sec = 0.0;
  CargoSwingMonitor monitor(config);

  // Establish both torsion and skew alarms
  auto input = validInput(1.0);
  input.hoist_state_fresh = true;
  input.hoist_motion_state = HoistMotionState::UP;
  input.measured_center_base.x() = 0.8F;
  input.locked_yaw_valid = true;
  input.measured_yaw_valid = true;
  input.locked_yaw_base_rad = 0.0F;
  input.measured_yaw_base_rad = 0.25F;
  const auto initial = monitor.update(input);
  ASSERT_EQ(initial.torsion_state, CargoTorsionState::TORSION_ALARM);
  ASSERT_EQ(initial.skew_pull_state,
            CargoSkewPullState::SKEW_PULL_ALARM);

  // Let torsion go stale while skew alarm persists
  input.stamp_sec = 1.1;
  input.measured_yaw_valid = false;
  monitor.update(input);
  input.stamp_sec = 1.5;
  const auto result = monitor.update(input);

  // P1-2: valid=true, skew alarm preserved, torsion=NOT_EVALUATED
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.skew_pull_state,
            CargoSkewPullState::SKEW_PULL_ALARM);
  EXPECT_EQ(result.torsion_state, CargoTorsionState::NOT_EVALUATED);
}

// ─── P1-3: track segment change clears history, preserves safety latches ───

TEST(CargoSwingMonitorTest,
     TrackSegmentChangeClearsDirectionalHistory) {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.5;
  config.skew_min_duration_sec = 0.5;
  CargoSwingMonitor monitor(config);

  // Feed consistent directional offset on segment 3
  for (int i = 0; i < 8; ++i) {
    auto input = validInput(1.0 + i * 0.1);
    input.hoist_state_fresh = true;
    input.hoist_motion_state = HoistMotionState::UP;
    input.measured_center_base.x() = 0.6F;
    monitor.update(input);
  }

  // Switch to segment 5 — history must reset
  auto seg5 = validInput(2.0);
  seg5.track_segment_id = 5U;
  seg5.hoist_state_fresh = true;
  seg5.hoist_motion_state = HoistMotionState::UP;
  seg5.measured_center_base.x() = -0.6F;  // opposite direction
  const auto result = monitor.update(seg5);

  // P1-3: segment change must be signaled
  EXPECT_EQ(result.observation_state,
            CargoSwingObservationState::TRACK_CHANGED);
  EXPECT_EQ(result.reason,
            "track_segment_changed_history_reset_alarm_preserved");

  // Feed more frames on segment 5 — old DC history must not combine
  for (int i = 1; i < 8; ++i) {
    seg5.stamp_sec = 2.0 + i * 0.1;
    monitor.update(seg5);
  }
  // After fresh history, the -X direction should establish its own skew
  // without the old +X direction creating false zero-crossings
  auto final_input = validInput(2.8);
  final_input.track_segment_id = 5U;
  final_input.hoist_state_fresh = true;
  final_input.hoist_motion_state = HoistMotionState::UP;
  final_input.measured_center_base.x() = -0.6F;
  const auto final_result = monitor.update(final_input);
  // New segment's consistent -X should produce low zero_crossings
  EXPECT_LE(final_result.zero_crossings, 1)
      << "old segment DC combined with new segment produced spurious crossings";
}

TEST(CargoSwingMonitorTest,
     TrackSegmentChangeCannotCombineOldAndNewDcEvidence) {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.5;
  config.skew_min_duration_sec = 0.5;
  CargoSwingMonitor monitor(config);

  // Segment 3: build consistent +X DC
  for (int i = 0; i < 8; ++i) {
    auto input = validInput(1.0 + i * 0.1);
    input.hoist_state_fresh = true;
    input.hoist_motion_state = HoistMotionState::UP;
    input.measured_center_base.x() = 0.55F;
    monitor.update(input);
  }

  // Segment 7: +Y DC (orthogonal), fresh history
  for (int i = 0; i < 8; ++i) {
    auto input = validInput(2.0 + i * 0.1);
    input.track_segment_id = 7U;
    input.hoist_state_fresh = true;
    input.hoist_motion_state = HoistMotionState::UP;
    input.measured_center_base.x() = 0.0F;
    input.measured_center_base.y() = 0.55F;
    monitor.update(input);
  }

  // After segment change and fresh history, the direction consistency
  // must reflect the new segment alone — not a blend of old+X and new+Y
  auto final_input = validInput(2.8);
  final_input.track_segment_id = 7U;
  final_input.hoist_state_fresh = true;
  final_input.hoist_motion_state = HoistMotionState::UP;
  final_input.measured_center_base.y() = 0.55F;
  const auto result = monitor.update(final_input);
  // New segment's pure Y offset should have high direction consistency
  EXPECT_GT(result.direction_consistency, 0.7F)
      << "blended old+X and new+Y DC ruined direction consistency";
}

TEST(CargoSwingMonitorTest,
     TrackSegmentChangePreservesLatchedSwayAlarm) {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.3;
  CargoSwingMonitor monitor(config);

  // Trigger immediate sway alarm
  auto extreme = validInput(1.0);
  extreme.measured_center_base.x() = 0.8F;
  ASSERT_EQ(monitor.update(extreme).sway_state,
            CargoSwayState::SWAY_ALARM);

  // Segment change
  auto seg5 = validInput(1.1);
  seg5.track_segment_id = 5U;
  seg5.measured_center_base.x() = 0.0F;
  const auto result = monitor.update(seg5);

  // P1-3: sway alarm latch must survive segment change
  EXPECT_EQ(result.sway_state, CargoSwayState::SWAY_ALARM);
  EXPECT_EQ(result.observation_state,
            CargoSwingObservationState::TRACK_CHANGED);
}

TEST(CargoSwingMonitorTest,
     TrackSegmentChangePreservesLatchedSkewAlarm) {
  CargoSwingConfig config = fastSkewAlarmConfig();
  CargoSwingMonitor monitor(config);
  feedSkewAlarm(&monitor, 1.0);

  // Segment change
  auto seg5 = validInput(1.1);
  seg5.track_segment_id = 5U;
  seg5.measured_center_base.x() = 0.0F;
  const auto result = monitor.update(seg5);

  // P1-3: skew alarm latch must survive segment change
  EXPECT_EQ(result.skew_pull_state,
            CargoSkewPullState::SKEW_PULL_ALARM);
  EXPECT_EQ(result.observation_state,
            CargoSwingObservationState::TRACK_CHANGED);
}

TEST(CargoSwingMonitorTest,
     TrackSegmentChangePreservesLatchedTorsionAlarm) {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.0;
  CargoSwingMonitor monitor(config);

  // Establish torsion alarm
  auto input = validInput(1.0);
  input.locked_yaw_valid = true;
  input.measured_yaw_valid = true;
  input.locked_yaw_base_rad = 0.0F;
  input.measured_yaw_base_rad = 0.25F;
  ASSERT_EQ(monitor.update(input).torsion_state,
            CargoTorsionState::TORSION_ALARM);

  // Segment change — torsion evidence lost but latch preserved
  auto seg5 = validInput(1.1);
  seg5.track_segment_id = 5U;
  seg5.measured_yaw_valid = false;
  const auto result = monitor.update(seg5);

  // P1-3: torsion latch survives segment change with evidence hold
  EXPECT_EQ(result.torsion_state, CargoTorsionState::TORSION_ALARM);
  EXPECT_EQ(result.observation_state,
            CargoSwingObservationState::TRACK_CHANGED);
}

TEST(CargoSwingMonitorTest,
     TrackSegmentChangeResetsTorsionClearCandidate) {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.0;
  config.torsion_clear_confirm_sec = 1.0;
  CargoSwingMonitor monitor(config);

  auto input = validInput(1.0);
  input.locked_yaw_valid = true;
  input.measured_yaw_valid = true;
  input.locked_yaw_base_rad = 0.0F;
  input.measured_yaw_base_rad = 0.25F;
  ASSERT_EQ(monitor.update(input).torsion_state,
            CargoTorsionState::TORSION_ALARM);

  input.stamp_sec = 1.5;
  input.measured_yaw_base_rad = 0.0F;
  ASSERT_EQ(monitor.update(input).torsion_state,
            CargoTorsionState::TORSION_ALARM);

  input.stamp_sec = 2.0;
  input.track_segment_id = 5U;
  ASSERT_EQ(monitor.update(input).observation_state,
            CargoSwingObservationState::TRACK_CHANGED);

  input.stamp_sec = 2.1;
  ASSERT_EQ(monitor.update(input).torsion_state,
            CargoTorsionState::TORSION_ALARM);
  input.stamp_sec = 2.6;
  EXPECT_EQ(monitor.update(input).torsion_state,
            CargoTorsionState::TORSION_ALARM);
}

TEST(CargoSwingMonitorTest,
     OldSegmentClearEvidenceCannotClearNewSegmentTorsionLatch) {
  CargoSwingConfig config;
  config.minimum_valid_observation_sec = 0.0;
  config.torsion_clear_confirm_sec = 0.5;
  CargoSwingMonitor monitor(config);

  auto input = validInput(1.0);
  input.locked_yaw_valid = true;
  input.measured_yaw_valid = true;
  input.locked_yaw_base_rad = 0.0F;
  input.measured_yaw_base_rad = 0.25F;
  ASSERT_EQ(monitor.update(input).torsion_state,
            CargoTorsionState::TORSION_ALARM);
  input.stamp_sec = 1.2;
  input.measured_yaw_base_rad = 0.0F;
  ASSERT_EQ(monitor.update(input).torsion_state,
            CargoTorsionState::TORSION_ALARM);

  input.stamp_sec = 1.6;
  input.track_segment_id = 9U;
  monitor.update(input);
  input.stamp_sec = 1.7;
  ASSERT_EQ(monitor.update(input).torsion_state,
            CargoTorsionState::TORSION_ALARM);
  input.stamp_sec = 2.0;
  ASSERT_EQ(monitor.update(input).torsion_state,
            CargoTorsionState::TORSION_ALARM);
  input.stamp_sec = 2.21;
  EXPECT_NE(monitor.update(input).torsion_state,
            CargoTorsionState::TORSION_ALARM);
}

}  // namespace
}  // namespace ndt_slam
