#include "ndt_slam/integrated_cargo_identity_shadow.hpp"
#include "ndt_slam/cargo_safety_evaluator.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

CargoPhysicalIdentityDecision validatedIdentity() {
  CargoPhysicalIdentityDecision identity;
  identity.identity = CargoPhysicalIdentityState::VALIDATED;
  identity.physical_history_id = 7U;
  identity.resolved_candidate_id = 3U;
  identity.resolved_member_component_ids = {9U};
  identity.load_epoch = 2U;
  identity.geometry_resolved = true;
  identity.current_candidate_fresh = true;
  return identity;
}

CargoPhysicalCandidateObservation candidate(double stamp) {
  CargoPhysicalCandidateObservation result;
  result.candidate_id = 3U;
  result.member_component_ids = {9U};
  result.stamp_sec = stamp;
  result.center = Eigen::Vector3d(0.0, 0.0, 1.0);
  result.size = Eigen::Vector3d(1.0, 0.8, 0.4);
  result.yaw_rad = 0.0;
  result.z95 = 1.2;
  result.point_support = 100U;
  return result;
}

TEST(IntegratedCargoIdentityShadowTest,
     PreValidationWrongSamplesCannotEnterShadowFormalSummary) {
  CargoShadowGeometryConfig config;
  config.formal_confirm_frames = 2;
  CargoShadowGeometryAuthority authority(config);
  CargoShadowGeometryInput wrong;
  wrong.stamp_sec = 1.0;
  wrong.candidate = candidate(1.0);
  EXPECT_FALSE(authority.update(wrong).pending_envelope_valid);

  CargoShadowGeometryInput valid;
  valid.stamp_sec = 1.1;
  valid.identity = validatedIdentity();
  valid.candidate = candidate(1.1);
  const auto first = authority.update(valid);
  EXPECT_EQ(first.confirm_count, 1);
  EXPECT_FALSE(first.formal_geometry_valid);
}

TEST(IntegratedCargoIdentityShadowTest,
     ShadowPendingTransitionsToFormalWithoutIdentityChange) {
  CargoShadowGeometryConfig config;
  config.formal_confirm_frames = 2;
  CargoShadowGeometryAuthority authority(config);
  CargoShadowGeometryInput input;
  input.identity = validatedIdentity();
  input.stamp_sec = 1.0;
  input.candidate = candidate(1.0);
  const auto pending = authority.update(input);
  EXPECT_TRUE(pending.pending_envelope_valid);
  EXPECT_FALSE(pending.formal_geometry_valid);
  input.stamp_sec = 1.1;
  input.candidate = candidate(1.1);
  const auto formal = authority.update(input);
  EXPECT_TRUE(formal.formal_geometry_valid);
  EXPECT_TRUE(formal.formal_clear_authorized);
  EXPECT_EQ(formal.physical_history_id, pending.physical_history_id);
}

TEST(IntegratedCargoIdentityShadowTest,
     VerticalLiftDoesNotBreakFormalGeometryContinuity) {
  CargoShadowGeometryConfig config;
  config.formal_confirm_frames = 2;
  config.maximum_xy_step_m = 0.30;
  config.maximum_z_speed_mps = 2.0;
  config.z_step_margin_m = 0.05;
  CargoShadowGeometryAuthority authority(config);
  CargoShadowGeometryInput input;
  input.identity = validatedIdentity();
  input.stamp_sec = 1.0;
  input.candidate = candidate(1.0);
  EXPECT_EQ(authority.update(input).confirm_count, 1);
  input.stamp_sec = 1.2;
  input.candidate = candidate(1.2);
  input.candidate.center.z() += 0.35;
  input.candidate.z95 += 0.35;
  const auto formal = authority.update(input);
  EXPECT_EQ(formal.confirm_count, 2);
  EXPECT_TRUE(formal.formal_geometry_valid);
}

TEST(IntegratedCargoIdentityShadowTest,
     IdentityChangeResetsShadowGeometryWindow) {
  CargoShadowGeometryConfig config;
  config.formal_confirm_frames = 2;
  CargoShadowGeometryAuthority authority(config);
  CargoShadowGeometryInput input;
  input.identity = validatedIdentity();
  input.stamp_sec = 1.0;
  input.candidate = candidate(1.0);
  EXPECT_EQ(authority.update(input).confirm_count, 1);
  input.identity.physical_history_id = 8U;
  input.stamp_sec = 1.1;
  input.candidate = candidate(1.1);
  const auto changed = authority.update(input);
  EXPECT_EQ(changed.confirm_count, 1);
  EXPECT_FALSE(changed.formal_geometry_valid);
}

TEST(IntegratedCargoIdentityShadowTest,
     WrongCandidateThicknessCannotLeakIntoShadow) {
  CargoShadowThicknessProvenance provenance;
  provenance.valid = true;
  provenance.physical_history_id = 99U;
  provenance.load_epoch = 2U;
  provenance.lifecycle_id = 5U;
  provenance.source = "product_wrong_candidate";
  provenance.source_stamp_sec = 1.0;
  EXPECT_FALSE(shadowThicknessAuthorized(
      provenance, validatedIdentity(), 5U));
}

TEST(IntegratedCargoIdentityShadowTest,
     LifecycleMatchAloneDoesNotAuthorizeFrozenThickness) {
  CargoShadowThicknessProvenance provenance;
  provenance.valid = true;
  provenance.lifecycle_id = 5U;
  provenance.source = "lifecycle_only";
  provenance.source_stamp_sec = 1.0;
  EXPECT_FALSE(shadowThicknessAuthorized(
      provenance, validatedIdentity(), 5U));
}

TEST(IntegratedCargoIdentityShadowTest,
     IntegratedShadowUsesCanonicalAvoidanceFusion) {
  CargoAvoidanceFusionInput canonical;
  canonical.localization_valid = true;
  canonical.warning_motion_authorized = true;
  canonical.live_near_field_history_authorized = true;
  canonical.static_near_field_history_authorized = true;
  canonical.static_hazard_track_confirmed = true;
  canonical.live_obstacle_origin_resolved = true;
  canonical.live.obstacle_track_id = 55U;
  canonical.live.far_field_history_valid = true;
  canonical.live.provenance_valid = true;
  canonical.live.validated_streak = 4;
  CargoShadowFusionProjection projection;
  projection.formal = true;
  projection.bottom_valid = true;
  projection.clear_authorized = true;
  projection.cargo_lifecycle_id = 8U;
  projection.cargo_track_id = 7U;
  projection.live = canonical.live;
  projection.live.available = true;
  projection.live.reliable = true;
  projection.live.hazard = true;
  projection.live.warning_code = CargoSafetyEvaluator::kLevel2Code;
  projection.live.distance_m = 4.0F;
  projection.live.clearance_m = 0.2F;
  const auto shadow_input = projectShadowCargoOntoCanonicalFusion(
      canonical, projection);
  const auto result = fuseCargoAvoidanceRisk(shadow_input);
  EXPECT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, CargoSafetyEvaluator::kLevel2Code);
  EXPECT_EQ(shadow_input.live.obstacle_track_id, 55U);
}

TEST(IntegratedCargoIdentityShadowTest,
     ShadowPendingCargoCannotClear) {
  CargoAvoidanceFusionInput canonical;
  canonical.localization_valid = true;
  canonical.warning_motion_authorized = true;
  canonical.live.available = true;
  canonical.live.reliable = true;
  canonical.live.hazard = false;
  CargoShadowFusionProjection projection;
  projection.pending = true;
  projection.live = canonical.live;
  const auto shadow_input = projectShadowCargoOntoCanonicalFusion(
      canonical, projection);
  const auto result = fuseCargoAvoidanceRisk(shadow_input);
  EXPECT_NE(result.official_code, CargoSafetyEvaluator::kSafeCode);
}

TEST(IntegratedCargoIdentityShadowTest,
     ValidatedIdentityCanProducePositiveOnlyPendingHazard) {
  CargoAvoidanceFusionInput canonical;
  canonical.localization_valid = true;
  canonical.warning_motion_authorized = true;
  canonical.live_near_field_history_authorized = true;
  canonical.pending_external_obstacle_authorized = true;
  canonical.pending_external_obstacle_track_id = 41U;
  canonical.pending_external_obstacle_confirmations = 4;
  canonical.pending_external_provenance_valid = true;
  canonical.pending_external_geometry_valid = true;
  canonical.pending_external_separation_valid = true;
  canonical.pending_self_evidence_valid = true;
  canonical.pending_authority_confidence = 1.0F;
  CargoShadowFusionProjection projection;
  projection.pending = true;
  projection.live.available = true;
  projection.live.reliable = true;
  projection.live.hazard = true;
  projection.live.warning_code = CargoSafetyEvaluator::kLevel1Code;
  projection.live.distance_m = 2.0F;
  projection.live.clearance_m = 0.1F;
  projection.live.validated_streak = 4;
  const auto result = fuseCargoAvoidanceRisk(
      projectShadowCargoOntoCanonicalFusion(canonical, projection));
  EXPECT_NE(result.official_code, CargoSafetyEvaluator::kSafeCode);
  EXPECT_NE(result.warning_authority, CargoWarningAuthority::FORMAL);
}

TEST(IntegratedCargoIdentityShadowTest,
     PhysicalObstacleDistanceTimingUsesShadowCargoGeometry) {
  CargoShadowPhysicalDistanceTiming timing;
  updateShadowPhysicalDistanceTiming(
      1.0, 9.0, 5.0, 8.0, true, true, false, &timing);
  updateShadowPhysicalDistanceTiming(
      2.0, 7.5, 5.0, 8.0, true, true, true, &timing);
  updateShadowPhysicalDistanceTiming(
      3.0, 4.5, 5.0, 8.0, true, true, true, &timing);
  EXPECT_DOUBLE_EQ(timing.first_obstacle_8m_stamp_sec, 2.0);
  EXPECT_DOUBLE_EQ(timing.first_obstacle_5m_stamp_sec, 3.0);
  EXPECT_TRUE(timing.identity_validated_before_8m);
  EXPECT_TRUE(timing.pending_or_lock_ready_before_5m);
}

}  // namespace
}  // namespace ndt_slam
