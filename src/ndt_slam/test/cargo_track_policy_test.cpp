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

TEST(CargoTrackPolicy, FormalYawSnapsToNearestWarehouseAxis) {
  constexpr float kDegToRad = 3.14159265358979323846F / 180.0F;
  EXPECT_FLOAT_EQ(quantizeCargoAxialYawToOrthogonal(20.0F * kDegToRad), 0.0F);
  EXPECT_NEAR(
      quantizeCargoAxialYawToOrthogonal(72.7F * kDegToRad),
      90.0F * kDegToRad, 1.0e-6F);
  EXPECT_NEAR(
      quantizeCargoAxialYawToOrthogonal(-70.0F * kDegToRad),
      -90.0F * kDegToRad, 1.0e-6F);
}

TEST(CargoTrackPolicy, RobustTopAndFrozenThicknessRecoverAbsoluteBottom) {
  const CargoTopSurfaceHeightResult result =
      evaluateCargoTopSurfaceHeight({
          0.60F, true, 2.40F, true, 0.20F, 0.25F});
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.used_top_surface);
  EXPECT_FALSE(result.bottom_corroborated);
  EXPECT_NEAR(result.bottom_z_base, 1.80F, 1.0e-6F);
  EXPECT_NEAR(result.center_z_base, 2.10F, 1.0e-6F);
  EXPECT_EQ(result.reason, "top_surface_minus_frozen_thickness");
}

TEST(CargoTrackPolicy, DirectBottomOnlyCorroboratesWhenConsistent) {
  const CargoTopSurfaceHeightResult result =
      evaluateCargoTopSurfaceHeight({
          0.60F, true, 2.40F, true, 1.75F, 0.25F});
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.bottom_corroborated);
  EXPECT_NEAR(result.bottom_z_base, 1.79F, 1.0e-6F);
  EXPECT_NEAR(result.top_z_base - result.bottom_z_base, 0.60F, 1.0e-6F);
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

TEST(CargoTrackPolicy, NearHookStableBackgroundCannotFormalLock) {
  const CargoPhysicalLockAuthorityDecision decision =
      evaluateCargoPhysicalLockAuthority({
          HookLoadSignalRole::AUXILIARY, true, HookLoadState::EMPTY,
          0.05F, 0.30F, 0.0F, 0.25F, 0, 0, 3});
  EXPECT_FALSE(decision.allowed);
  EXPECT_EQ(decision.source, CargoLockAuthoritySource::NONE);
}

TEST(CargoTrackPolicy, AuxiliaryEmptyRequiresIndependentLiftEvidence) {
  const CargoPhysicalLockAuthorityDecision decision =
      evaluateCargoPhysicalLockAuthority({
          HookLoadSignalRole::AUXILIARY, true, HookLoadState::EMPTY,
          0.45F, 0.30F, 0.30F, 0.25F, 3, 3, 3});
  EXPECT_TRUE(decision.allowed);
  EXPECT_EQ(decision.source, CargoLockAuthoritySource::LIFT_FROM_ORIGIN);
  EXPECT_TRUE(decision.gravity_conflict);
}

TEST(CargoTrackPolicy, LiftFromOriginCanAuthorizeLock) {
  const CargoPhysicalLockAuthorityDecision decision =
      evaluateCargoPhysicalLockAuthority({
          HookLoadSignalRole::AUXILIARY, false, HookLoadState::UNKNOWN,
          0.10F, 0.30F, 0.35F, 0.25F, 0, 3, 3});
  EXPECT_TRUE(decision.allowed);
  EXPECT_EQ(decision.source, CargoLockAuthoritySource::LIFT_FROM_ORIGIN);
}

TEST(CargoTrackPolicy, CargoNearGroundCannotFormalLock) {
  const CargoPhysicalLockAuthorityDecision decision =
      evaluateCargoPhysicalLockAuthority({
          HookLoadSignalRole::DISABLED, false, HookLoadState::UNKNOWN,
          0.10F, 0.30F, 0.05F, 0.25F, 8, 8, 3});
  EXPECT_FALSE(decision.allowed);
}

TEST(CargoTrackPolicy, LoadedSignalCanAuthorizeLock) {
  const CargoPhysicalLockAuthorityDecision decision =
      evaluateCargoPhysicalLockAuthority({
          HookLoadSignalRole::AUXILIARY, true, HookLoadState::LOADED,
          0.0F, 0.30F, 0.0F, 0.25F, 0, 0, 3});
  EXPECT_TRUE(decision.allowed);
  EXPECT_EQ(decision.source, CargoLockAuthoritySource::GRAVITY_LOADED);
}

TEST(CargoTrackPolicy, Top1Top2AmbiguityPreventsFormalLock) {
  CargoCandidateIdentityScore first = strongScore(1);
  CargoCandidateIdentityScore second = strongScore(2);
  second.identity_confidence = 0.89F;
  const CargoCandidateRanking ranking =
      rankCargoCandidateIdentityScores({first, second});
  EXPECT_TRUE(ranking.valid);
  EXPECT_LT(ranking.margin, 0.08F);
}

TEST(CargoTrackPolicy, IdentitySelectedComponentEqualsGeometryComponent) {
  CargoCandidateIdentityScore weak = strongScore(4);
  weak.identity_confidence = 0.40F;
  weak.overall_lock_confidence = 0.45F;
  CargoCandidateIdentityScore strong = strongScore(9);
  const CargoCandidateRanking ranking =
      rankCargoCandidateIdentityScores({weak, strong});
  ASSERT_TRUE(ranking.valid);
  EXPECT_EQ(ranking.top1.component_id, 9);
}

TEST(CargoTrackPolicy, DifferentComponentsCannotShareProvisionalWindow) {
  CargoAssociationInput input;
  input.candidate = candidate(
      8, 0.8F, 0.8F, 2.5F, 1.1F, 0.9F, 0.5F, 1.2F);
  input.previous_center = Eigen::Vector3f(0.0F, 0.0F, 2.5F);
  input.locked_size = Eigen::Vector3f(2.0F, 0.4F, 0.8F);
  input.sensor_dt_sec = 0.1;
  input.base_center_gate_m = 0.30F;
  input.maximum_shape_relative_error = 0.30F;
  input.maximum_axial_yaw_error_rad = 0.40F;
  EXPECT_FALSE(evaluateCargoPredictedAssociation(input).accepted);
}

TEST(CargoTrackPolicy, LockedYawNoiseDoesNotRejectSameCargo) {
  CargoAssociationInput input;
  input.candidate = candidate(
      1, 0.02F, 0.0F, 2.5F, 1.3F, 0.6F, 0.8F, 1.20F);
  input.previous_center = Eigen::Vector3f(0.0F, 0.0F, 2.5F);
  input.locked_size = Eigen::Vector3f(2.0F, 0.4F, 0.8F);
  input.sensor_dt_sec = 0.10;
  input.minimum_overlap_ratio = 0.0F;
  input.use_shape_as_hard_gate = false;
  input.use_yaw_as_hard_gate = false;
  const CargoAssociationDecision decision =
      evaluateCargoPredictedAssociation(input);
  EXPECT_TRUE(decision.accepted) << decision.reason;
  EXPECT_FALSE(decision.yaw_used_as_hard_gate);
  EXPECT_GT(decision.axial_yaw_error_rad, 1.0F);
}

TEST(CargoTrackPolicy, PredictionGateDoesNotDoubleCountVelocity) {
  CargoAssociationInput input;
  input.candidate = candidate(
      1, 1.24F, 0.0F, 2.5F, 2.0F, 0.4F, 0.8F, 0.0F);
  input.previous_center = Eigen::Vector3f(0.0F, 0.0F, 2.5F);
  input.velocity = Eigen::Vector3f(1.0F, 0.0F, 0.0F);
  input.locked_size = input.candidate.size;
  input.sensor_dt_sec = 1.0;
  input.base_center_gate_m = 0.20F;
  input.velocity_model_uncertainty_mps = 0.05F;
  const CargoAssociationDecision decision =
      evaluateCargoPredictedAssociation(input);
  EXPECT_TRUE(decision.accepted) << decision.reason;
  EXPECT_NEAR(decision.dynamic_xy_gate_m, 0.25F, 1.0e-5F);
}

TEST(CargoTrackPolicy, ReacquisitionGateHasHardMaximum) {
  CargoAssociationInput input;
  input.candidate = candidate(
      1, 0.50F, 0.0F, 2.5F, 2.0F, 0.4F, 0.8F, 0.0F);
  input.previous_center = Eigen::Vector3f(0.0F, 0.0F, 2.5F);
  input.locked_size = input.candidate.size;
  input.sensor_dt_sec = 4.0;
  input.horizontal_uncertainty_m = 2.0F;
  input.horizontal_tracking_residual_m = 2.0F;
  input.maximum_xy_gate_m = 0.55F;
  input.maximum_z_gate_m = 0.65F;
  input.strict_reacquisition = true;
  const CargoAssociationDecision decision =
      evaluateCargoPredictedAssociation(input);
  EXPECT_LE(decision.dynamic_xy_gate_m, 0.55F);
  EXPECT_LE(decision.dynamic_z_gate_m, 0.65F);
}

TEST(CargoTrackPolicy, SparseLongCargoMaintainsLockedYaw) {
  CargoFrozenObbSupportInput input;
  input.predicted_center = Eigen::Vector3f(0.0F, 0.0F, 2.0F);
  input.locked_size = Eigen::Vector3f(2.0F, 0.5F, 0.8F);
  input.locked_yaw_rad = 0.0F;
  for (int i = 0; i < 20; ++i) {
    input.points.emplace_back(
        -0.8F + 0.08F * static_cast<float>(i),
        0.18F + 0.01F * static_cast<float>(i % 3), 2.1F);
  }
  const CargoFrozenObbSupport support =
      evaluateCargoFrozenObbSupport(input);
  EXPECT_TRUE(support.valid);
  EXPECT_GT(support.inside_ratio, 0.95F);
  EXPECT_GT(support.long_axis_coverage_ratio, 0.70F);
}

TEST(CargoTrackPolicy, ClearRequiresRearmBeforeNewCandidate) {
  CargoRearmInput input;
  input.candidate_valid = true;
  input.independent_suspension_evidence = false;
  input.candidate_score_margin = 0.50F;
  input.retired_identity_confidence = 0.20F;
  EXPECT_FALSE(evaluateCargoRearm(input).allowed);

  input.independent_suspension_evidence = true;
  EXPECT_TRUE(evaluateCargoRearm(input).allowed);
}

TEST(CargoTrackPolicy,
     AuthoritativeEmptyRearmsRecognitionWithoutGrantingSafetyClear) {
  CargoRearmInput input;
  input.gravity_valid = true;
  input.gravity_state = HookLoadState::EMPTY;
  input.gravity_state_at_clear = HookLoadState::LOADED;
  input.rearm_age_sec = 1.0;
  input.minimum_empty_confirm_sec = 1.0;
  input.candidate_valid = true;
  input.independent_suspension_evidence = false;
  const CargoRearmDecision decision = evaluateCargoRearm(input);
  EXPECT_TRUE(decision.allowed);
  EXPECT_EQ(
      decision.reason,
      "gravity_empty_rearm_lidar_conflict_retained");
}

}  // namespace
}  // namespace ndt_slam
