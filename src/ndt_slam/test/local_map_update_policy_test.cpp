#include <gtest/gtest.h>

#include "ndt_slam/local_map_update_policy.hpp"

namespace ndt_slam {
namespace {

LocalMapUpdateInput readyInput() {
  LocalMapUpdateInput input;
  input.registration_success = true;
  input.registration_cloud_valid = true;
  input.runtime_pose_finite = true;
  input.motion_state_allows_update = true;
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
  EXPECT_TRUE(decision.attempted);
  EXPECT_TRUE(decision.allowed);
  EXPECT_TRUE(decision.should_update);
  EXPECT_EQ(LocalMapUpdateBlockReason::READY, decision.block_reason);
}

TEST(LocalMapUpdatePolicyTest, TemporaryDegradationCannotLatchPolicyClosed) {
  LocalMapUpdateInput degraded = readyInput();
  degraded.registration_success = false;
  const auto blocked = evaluateLocalMapUpdate(degraded);
  EXPECT_FALSE(blocked.allowed);
  EXPECT_EQ(LocalMapUpdateBlockReason::NO_REGISTRATION,
            blocked.block_reason);

  // A later usable runtime frame is evaluated from current Level-1 evidence;
  // there is no stale AcceptedSnapshot or quality latch to clear first.
  const auto recovered = evaluateLocalMapUpdate(readyInput());
  EXPECT_TRUE(recovered.allowed);
  EXPECT_TRUE(recovered.should_update);
  EXPECT_EQ(LocalMapUpdateBlockReason::READY, recovered.block_reason);
}

TEST(LocalMapUpdatePolicyTest, MotionLifecycleBoundariesRemainClosed) {
  LocalMapUpdateInput input = readyInput();
  input.motion_state_allows_update = false;
  auto decision = evaluateLocalMapUpdate(input);
  EXPECT_FALSE(decision.allowed);
  EXPECT_EQ(LocalMapUpdateBlockReason::MOTION_STATE_BLOCKED,
            decision.block_reason);

  input = readyInput();
  input.relocalization_pose_reliable = false;
  decision = evaluateLocalMapUpdate(input);
  EXPECT_FALSE(decision.allowed);
  EXPECT_EQ(LocalMapUpdateBlockReason::RELOCALIZATION_UNRELIABLE,
            decision.block_reason);
}

TEST(LocalMapUpdatePolicyTest, InvalidCloudOrPoseCannotEnterRuntimeTarget) {
  LocalMapUpdateInput input = readyInput();
  input.registration_cloud_valid = false;
  auto decision = evaluateLocalMapUpdate(input);
  EXPECT_FALSE(decision.allowed);
  EXPECT_EQ(LocalMapUpdateBlockReason::CLOUD_INVALID,
            decision.block_reason);

  input = readyInput();
  input.runtime_pose_finite = false;
  decision = evaluateLocalMapUpdate(input);
  EXPECT_FALSE(decision.allowed);
  EXPECT_EQ(LocalMapUpdateBlockReason::RUNTIME_POSE_NONFINITE,
            decision.block_reason);
}

TEST(LocalMapUpdatePolicyTest, ExistingDistanceRotationAndFrameTriggersRemain) {
  LocalMapUpdateInput input = readyInput();
  input.frames_since_update = 1;
  auto decision = evaluateLocalMapUpdate(input);
  EXPECT_TRUE(decision.allowed);
  EXPECT_FALSE(decision.should_update);
  EXPECT_EQ(LocalMapUpdateBlockReason::WAITING_UPDATE_TRIGGER,
            decision.block_reason);

  input.translation_since_update_m = 0.51;
  decision = evaluateLocalMapUpdate(input);
  EXPECT_TRUE(decision.should_update);

  input.translation_since_update_m = 0.0;
  input.rotation_since_update_rad = 0.081;
  decision = evaluateLocalMapUpdate(input);
  EXPECT_TRUE(decision.should_update);
}

}  // namespace
}  // namespace ndt_slam
