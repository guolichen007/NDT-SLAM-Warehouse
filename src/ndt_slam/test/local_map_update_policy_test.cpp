#include <gtest/gtest.h>

#include "ndt_slam/local_map_update_policy.hpp"

namespace ndt_slam {
namespace {

LocalMapUpdateInput readyInput() {
  LocalMapUpdateInput input;
  input.registration_success = true;
  input.registration_cloud_valid = true;
  input.runtime_pose_finite = true;
  input.normal_motion_update_allowed = true;
  input.relocalization_pose_reliable = true;
  input.frames_since_update = 16;
  return input;
}

TEST(LocalMapUpdatePolicyTest,
     RuntimeTargetDoesNotConsumePersistentOrAcceptedPoseAuthority) {
  // The policy has no AcceptedSnapshot, MapWriteAuthority, fitness circuit,
  // NDT-accepted, or registration-quality inputs.  A finite runtime pose and
  // current registration cloud keep the ephemeral target self-healing.
  const auto decision = evaluateLocalMapUpdate(readyInput());
  EXPECT_TRUE(decision.eligible);
  EXPECT_TRUE(decision.due);
  EXPECT_EQ(LocalMapUpdateMode::NORMAL_UPDATE, decision.mode);
  EXPECT_EQ(LocalMapUpdateBlockReason::READY, decision.block_reason);
}

TEST(LocalMapUpdatePolicyTest, TemporaryDegradationCannotLatchPolicyClosed) {
  LocalMapUpdateInput degraded = readyInput();
  degraded.registration_success = false;
  const auto blocked = evaluateLocalMapUpdate(degraded);
  EXPECT_FALSE(blocked.eligible);
  EXPECT_EQ(LocalMapUpdateBlockReason::NO_REGISTRATION,
            blocked.block_reason);

  // A later usable runtime frame is evaluated from current Level-1 evidence;
  // there is no stale AcceptedSnapshot or quality latch to clear first.
  const auto recovered = evaluateLocalMapUpdate(readyInput());
  EXPECT_TRUE(recovered.eligible);
  EXPECT_TRUE(recovered.due);
  EXPECT_EQ(LocalMapUpdateBlockReason::READY, recovered.block_reason);
}

TEST(LocalMapUpdatePolicyTest, MotionLifecycleBoundariesRemainClosed) {
  LocalMapUpdateInput input = readyInput();
  input.normal_motion_update_allowed = false;
  auto decision = evaluateLocalMapUpdate(input);
  EXPECT_FALSE(decision.eligible);
  EXPECT_EQ(LocalMapUpdateBlockReason::MOTION_STATE_BLOCKED,
            decision.block_reason);

  input = readyInput();
  input.relocalization_pose_reliable = false;
  decision = evaluateLocalMapUpdate(input);
  EXPECT_FALSE(decision.eligible);
  EXPECT_EQ(LocalMapUpdateBlockReason::RELOCALIZATION_UNRELIABLE,
            decision.block_reason);
}

TEST(LocalMapUpdatePolicyTest, InvalidCloudOrPoseCannotEnterRuntimeTarget) {
  LocalMapUpdateInput input = readyInput();
  input.registration_cloud_valid = false;
  auto decision = evaluateLocalMapUpdate(input);
  EXPECT_FALSE(decision.eligible);
  EXPECT_EQ(LocalMapUpdateBlockReason::CLOUD_INVALID,
            decision.block_reason);

  input = readyInput();
  input.runtime_pose_finite = false;
  decision = evaluateLocalMapUpdate(input);
  EXPECT_FALSE(decision.eligible);
  EXPECT_EQ(LocalMapUpdateBlockReason::RUNTIME_POSE_NONFINITE,
            decision.block_reason);
}

TEST(LocalMapUpdatePolicyTest, ExistingDistanceRotationAndFrameTriggersRemain) {
  LocalMapUpdateInput input = readyInput();
  input.frames_since_update = 1;
  auto decision = evaluateLocalMapUpdate(input);
  EXPECT_TRUE(decision.eligible);
  EXPECT_FALSE(decision.due);
  EXPECT_EQ(LocalMapUpdateBlockReason::WAITING_UPDATE_TRIGGER,
            decision.block_reason);

  input.translation_since_update_m = 0.51;
  decision = evaluateLocalMapUpdate(input);
  EXPECT_TRUE(decision.due);

  input.translation_since_update_m = 0.0;
  input.rotation_since_update_rad = 0.081;
  decision = evaluateLocalMapUpdate(input);
  EXPECT_TRUE(decision.due);
}

TEST(LocalMapUpdatePolicyTest,
     MotionEscapeRefreshIsExplicitAndStillHonorsQuarantine) {
  LocalMapUpdateInput input = readyInput();
  input.normal_motion_update_allowed = false;
  input.motion_escape_refresh_allowed = true;

  auto decision = evaluateLocalMapUpdate(input);
  EXPECT_TRUE(decision.eligible);
  EXPECT_TRUE(decision.due);
  EXPECT_EQ(LocalMapUpdateMode::MOTION_ESCAPE_REFRESH, decision.mode);

  input.relocalization_pose_reliable = false;
  decision = evaluateLocalMapUpdate(input);
  EXPECT_FALSE(decision.eligible);
  EXPECT_EQ(LocalMapUpdateMode::NONE, decision.mode);
  EXPECT_EQ(LocalMapUpdateBlockReason::RELOCALIZATION_UNRELIABLE,
            decision.block_reason);
}

TEST(LocalMapUpdatePolicyTest,
     LongPredictionOnlyIsMeasuredWithoutInventingAProductionCutoff) {
  LocalMapPoseAuthorityTracker tracker;
  LocalMapPoseSample sample;
  sample.authority = LocalMapPoseAuthority::NDT_MEASURED;
  sample.stamp_sec = 1.0;
  auto metrics = tracker.observe(sample);
  EXPECT_TRUE(metrics.trusted_ndt_reference_valid);

  for (int i = 1; i <= 100; ++i) {
    sample.authority = LocalMapPoseAuthority::EKF_PREDICTED;
    sample.stamp_sec = 1.0 + 0.1 * i;
    sample.x_m = 0.02 * i;
    sample.yaw_rad = 0.001 * i;
    metrics = tracker.observe(sample);
    if (i % 16 == 0) {
      metrics = tracker.recordLocalMapUpdate(sample.authority);
    }
  }
  EXPECT_EQ(100U, metrics.consecutive_prediction_only_frames);
  EXPECT_NEAR(9.9, metrics.prediction_only_duration_sec, 1.0e-9);
  EXPECT_EQ(100U, metrics.frames_since_last_trusted_ndt);
  EXPECT_NEAR(10.0, metrics.time_since_last_trusted_ndt_sec, 1.0e-9);
  EXPECT_NEAR(2.0, metrics.distance_since_last_trusted_ndt_m, 1.0e-9);
  EXPECT_NEAR(0.1, metrics.yaw_since_last_trusted_ndt_rad, 1.0e-9);
  EXPECT_EQ(6U, metrics.local_map_updates_from_predicted_pose);

  sample.authority = LocalMapPoseAuthority::NDT_MEASURED;
  sample.stamp_sec = 11.1;
  metrics = tracker.observe(sample);
  EXPECT_EQ(0U, metrics.consecutive_prediction_only_frames);
  EXPECT_EQ(0U, metrics.frames_since_last_trusted_ndt);
  EXPECT_EQ(LocalMapPoseAuthority::NDT_MEASURED, metrics.authority);
}

TEST(LocalMapUpdatePolicyTest,
     MovingPredictionThenMeasuredRecoveryAdvancesTargetVersion) {
  LocalMapPoseAuthorityTracker tracker;
  LocalMapPoseSample sample;
  sample.authority = LocalMapPoseAuthority::NDT_MEASURED;
  sample.stamp_sec = 1.0;
  tracker.observe(sample);

  std::uint64_t target_version = 7U;
  LocalMapUpdateInput input = readyInput();
  sample.authority = LocalMapPoseAuthority::EKF_PREDICTED;
  sample.stamp_sec = 1.1;
  tracker.observe(sample);
  auto decision = evaluateLocalMapUpdate(input);
  ASSERT_TRUE(decision.eligible);
  ASSERT_TRUE(decision.due);
  ++target_version;
  tracker.recordLocalMapUpdate(sample.authority);

  sample.authority = LocalMapPoseAuthority::NDT_MEASURED;
  sample.stamp_sec = 1.2;
  const auto recovered = tracker.observe(sample);
  decision = evaluateLocalMapUpdate(input);
  ASSERT_TRUE(decision.eligible);
  ASSERT_TRUE(decision.due);
  ++target_version;
  tracker.recordLocalMapUpdate(sample.authority);

  EXPECT_EQ(9U, target_version);
  EXPECT_EQ(LocalMapPoseAuthority::NDT_MEASURED, recovered.authority);
  EXPECT_EQ(0U, recovered.consecutive_prediction_only_frames);
}

TEST(LocalMapUpdatePolicyTest, StationaryIdleIsNeverReportedAsStarvation) {
  LocalMapHealthInput input;
  input.relocalization_pose_reliable = true;
  input.stationary_idle = true;
  input.motion_update_expected = false;
  input.expected_update_age_sec = 3600.0;
  EXPECT_EQ(LocalMapHealthState::IDLE_STATIONARY,
            classifyLocalMapHealth(input));

  input.stationary_idle = false;
  input.motion_update_expected = true;
  input.eligible = true;
  input.due = false;
  input.expected_update_age_sec = 1.0;
  EXPECT_EQ(LocalMapHealthState::WAITING_TRIGGER,
            classifyLocalMapHealth(input));

  input.expected_update_age_sec = 5.1;
  EXPECT_EQ(LocalMapHealthState::STARVED_MOVING,
            classifyLocalMapHealth(input));

  input.relocalization_pose_reliable = false;
  EXPECT_EQ(LocalMapHealthState::QUARANTINED,
            classifyLocalMapHealth(input));
}

}  // namespace
}  // namespace ndt_slam
