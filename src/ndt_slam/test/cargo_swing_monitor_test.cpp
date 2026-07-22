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

}  // namespace
}  // namespace ndt_slam
