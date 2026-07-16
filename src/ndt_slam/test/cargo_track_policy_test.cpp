#include <gtest/gtest.h>

#include "ndt_slam/cargo_track_policy.hpp"

#include <cmath>

namespace ndt_slam {
namespace {

CargoCandidateDescriptor candidate(
    int id, float x, float y, float z, float length, float width,
    float height, float yaw, bool suspended = true) {
  CargoCandidateDescriptor result;
  result.component_id = id;
  result.center = Eigen::Vector3f(x, y, z);
  result.size = Eigen::Vector3f(length, width, height);
  result.yaw_rad = yaw;
  result.orientation_confidence = 0.95F;
  result.point_count = 160U;
  result.suspension_evidence = suspended;
  return result;
}

CargoCandidateIdentityScore strongScore(int id) {
  CargoCandidateIdentityScore score;
  score.valid = true;
  score.component_id = id;
  score.identity_confidence = 0.90F;
  score.shape_confidence = 0.90F;
  score.motion_confidence = 0.90F;
  score.suspension_confidence = 1.0F;
  score.overall_lock_confidence = 0.90F;
  return score;
}

TEST(CargoTrackPolicy, OrientationConcentrationAloneCannotLock) {
  std::vector<CargoCandidateDescriptor> observations;
  std::vector<CargoCandidateIdentityScore> scores;
  for (int i = 0; i < 4; ++i) {
    observations.push_back(candidate(
        7, 0.1F, -1.0F, 0.4F, 1.5F, 1.0F, 0.55F, 1.21F, false));
    CargoCandidateIdentityScore score = strongScore(7);
    score.identity_confidence = 0.45F;
    score.suspension_confidence = 0.35F;
    scores.push_back(score);
  }
  CargoProvisionalLockConfig config;
  config.minimum_frames = 4U;
  EXPECT_FALSE(summarizeCargoProvisionalLock(
      observations, scores, config).formal_lock_allowed);
}

TEST(CargoTrackPolicy, ProvisionalYawCanConvergeBeforeFormalLock) {
  std::vector<CargoCandidateDescriptor> observations = {
      candidate(1, 0.00F, -1.0F, 2.4F, 2.0F, 0.4F, 0.8F, 0.20F),
      candidate(1, 0.02F, -1.0F, 2.4F, 2.02F, 0.4F, 0.81F, 0.08F),
      candidate(1, 0.04F, -1.0F, 2.4F, 2.01F, 0.4F, 0.80F, 0.03F),
      candidate(1, 0.06F, -1.0F, 2.4F, 2.00F, 0.4F, 0.80F, 0.01F)};
  std::vector<CargoCandidateIdentityScore> scores(
      observations.size(), strongScore(1));
  CargoProvisionalLockConfig config;
  config.minimum_frames = 4U;
  const CargoProvisionalLockSummary summary =
      summarizeCargoProvisionalLock(observations, scores, config);
  EXPECT_TRUE(summary.formal_lock_allowed) << summary.reason;
  EXPECT_NEAR(summary.axial_yaw_rad, 0.08F, 0.12F);
}

TEST(CargoTrackPolicy, WrongStableBackgroundClusterCannotFormalLock) {
  CargoCandidateIdentityContext context;
  context.hook_center = Eigen::Vector2f::Zero();
  context.hook_region_radius_m = 1.5F;
  context.strong_point_count = 80U;
  const CargoCandidateIdentityScore score = scoreCargoCandidateIdentity(
      candidate(3, 1.45F, 1.40F, 0.35F, 1.5F, 1.0F, 0.55F,
                1.21F, false),
      context);
  EXPECT_TRUE(score.valid);
  EXPECT_LT(score.overall_lock_confidence, 0.70F);
}

TEST(CargoTrackPolicy, AssociationUsesPredictedCenterAndDt) {
  CargoAssociationInput input;
  input.candidate = candidate(
      1, 1.0F, 0.0F, 2.5F, 2.0F, 0.4F, 0.8F, 0.0F);
  input.previous_center = Eigen::Vector3f(0.0F, 0.0F, 2.5F);
  input.velocity = Eigen::Vector3f(1.0F, 0.0F, 0.0F);
  input.locked_size = Eigen::Vector3f(2.0F, 0.4F, 0.8F);
  input.sensor_dt_sec = 1.0;
  input.base_center_gate_m = 0.20F;
  input.minimum_overlap_ratio = 0.30F;
  const CargoAssociationDecision decision =
      evaluateCargoPredictedAssociation(input);
  EXPECT_TRUE(decision.accepted) << decision.reason;
  EXPECT_NEAR(decision.predicted_center.x(), 1.0F, 1.0e-5F);
}

TEST(CargoTrackPolicy, NormalHoistDoesNotTriggerHeightMismatch) {
  CargoAssociationInput input;
  input.candidate = candidate(
      1, 0.0F, 0.0F, 3.0F, 2.0F, 0.4F, 0.8F, 0.0F);
  input.previous_center = Eigen::Vector3f(0.0F, 0.0F, 2.0F);
  input.velocity = Eigen::Vector3f(0.0F, 0.0F, 1.0F);
  input.locked_size = Eigen::Vector3f(2.0F, 0.4F, 0.8F);
  input.sensor_dt_sec = 1.0;
  input.base_center_gate_m = 0.20F;
  input.minimum_overlap_ratio = 0.30F;
  const CargoAssociationDecision decision =
      evaluateCargoPredictedAssociation(input);
  EXPECT_TRUE(decision.accepted) << decision.reason;
}

TEST(CargoTrackPolicy, ReacquisitionRejectsIncompatibleShape) {
  CargoAssociationInput input;
  input.candidate = candidate(
      5, 0.0F, 0.0F, 2.5F, 1.0F, 0.9F, 0.4F, 1.3F);
  input.previous_center = input.candidate.center;
  input.locked_size = Eigen::Vector3f(2.0F, 0.4F, 0.8F);
  input.sensor_dt_sec = 0.2;
  input.minimum_overlap_ratio = 0.40F;
  input.strict_reacquisition = true;
  const CargoAssociationDecision decision =
      evaluateCargoPredictedAssociation(input);
  EXPECT_FALSE(decision.accepted);
}

}  // namespace
}  // namespace ndt_slam
