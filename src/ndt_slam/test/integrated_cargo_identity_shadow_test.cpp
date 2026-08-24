#include "ndt_slam/integrated_cargo_identity_shadow.hpp"
#include "ndt_slam/cargo_bottom_fusion.hpp"
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

CargoShadowResolvedGeometryObservation geometry(double stamp) {
  CargoShadowResolvedGeometryObservation result;
  result.valid = true;
  result.source_stamp_sec = stamp;
  result.footprint_center_base = Eigen::Vector2d(0.0, 0.0);
  result.physical_anchor_z = 1.0;
  result.size = Eigen::Vector3d(1.0, 0.8, 0.4);
  result.yaw_rad = 0.0;
  result.point_support = 100U;
  return result;
}

CargoPhysicalGroupObservation physicalGroup(
    double stamp, std::uint64_t candidate_id = 3U) {
  CargoPhysicalGroupObservation group;
  group.frame_group_id = 2U;
  group.member_component_ids = {9U};
  group.geometry_resolved = true;
  group.descriptor.valid = true;
  group.descriptor.stamp_sec = stamp;
  group.descriptor.stable_anchor = Eigen::Vector3d(0.0, 0.0, 1.0);
  group.descriptor.aggregate_extent = Eigen::Vector3d(1.0, 0.8, 0.4);
  group.descriptor.vertical_mode = CargoGroupVerticalMode::SUPPORTED_EVIDENCE;
  group.descriptor.physical_vertical_z = 1.2;
  group.descriptor.vertical_uncertainty_m = 0.05;
  group.representative.candidate_id = candidate_id;
  group.representative.stamp_sec = stamp;
  group.representative.center = Eigen::Vector3d(0.0, 0.0, 1.0);
  group.representative.size = Eigen::Vector3d(1.0, 0.8, 0.4);
  group.representative.yaw_rad = 0.0;
  group.representative.point_support = 100U;
  for (int i = 0; i < 50; ++i) {
    group.union_points_base.emplace_back(
        0.01F * static_cast<float>(i % 10),
        0.01F * static_cast<float>(i / 10),
        0.8F + 0.01F * static_cast<float>(i % 20));
  }
  return group;
}

CargoPhysicalIdentityDecision decisionForGroup() {
  CargoPhysicalIdentityDecision identity = validatedIdentity();
  identity.frame_group_id = 2U;
  return identity;
}

TEST(IntegratedCargoIdentityShadowTest,
     PreValidationWrongSamplesCannotEnterShadowFormalSummary) {
  CargoShadowGeometryConfig config;
  config.formal_confirm_frames = 2;
  CargoShadowGeometryAuthority authority(config);
  CargoShadowGeometryInput wrong;
  wrong.stamp_sec = 1.0;
  wrong.geometry = geometry(1.0);
  EXPECT_FALSE(authority.update(wrong).pending_envelope_valid);

  CargoShadowGeometryInput valid;
  valid.stamp_sec = 1.1;
  valid.identity = validatedIdentity();
  valid.geometry = geometry(1.1);
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
  input.geometry = geometry(1.0);
  const auto pending = authority.update(input);
  EXPECT_TRUE(pending.pending_envelope_valid);
  EXPECT_FALSE(pending.formal_geometry_valid);
  input.stamp_sec = 1.1;
  input.geometry = geometry(1.1);
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
  input.geometry = geometry(1.0);
  EXPECT_EQ(authority.update(input).confirm_count, 1);
  input.stamp_sec = 1.2;
  input.geometry = geometry(1.2);
  input.geometry.physical_anchor_z += 0.35;
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
  input.geometry = geometry(1.0);
  EXPECT_EQ(authority.update(input).confirm_count, 1);
  input.identity.physical_history_id = 8U;
  input.stamp_sec = 1.1;
  input.geometry = geometry(1.1);
  const auto changed = authority.update(input);
  EXPECT_EQ(changed.confirm_count, 1);
  EXPECT_FALSE(changed.formal_geometry_valid);
}

TEST(IntegratedCargoIdentityShadowTest,
     ValidatedGroupFeedsBottomWithoutResolvedCandidatePoints) {
  const CargoPhysicalGroupObservation group = physicalGroup(1.0, 991U);
  const auto snapshot = bindCargoPhysicalGroupEvidence(
      {group}, decisionForGroup(), 2U);
  ASSERT_TRUE(snapshot.valid);
  EXPECT_EQ(snapshot.physical_history_id, 7U);
  EXPECT_EQ(snapshot.union_points_base.size(), group.union_points_base.size());
  EXPECT_TRUE(snapshot.supported_top_valid);
}

TEST(IntegratedCargoIdentityShadowTest,
     ResolvedCandidateMissingTopCannotOverrideGroupSupportedTop) {
  CargoPhysicalGroupObservation group = physicalGroup(1.0);
  group.representative.z95 = -50.0;
  const auto snapshot = bindCargoPhysicalGroupEvidence(
      {group}, decisionForGroup(), 2U);
  ASSERT_TRUE(snapshot.supported_top_valid);
  EXPECT_DOUBLE_EQ(snapshot.supported_top_z, 1.2);
}

TEST(IntegratedCargoIdentityShadowTest,
     BottomPointsComeFromRawGroupUnionNotVerticalTopBand) {
  CargoPhysicalGroupObservation group = physicalGroup(1.0);
  group.union_points_base.emplace_back(0.0F, 0.0F, 0.25F);
  const auto snapshot = bindCargoPhysicalGroupEvidence(
      {group}, decisionForGroup(), 2U);
  ASSERT_EQ(snapshot.union_points_base.size(),
            group.union_points_base.size());
  EXPECT_FLOAT_EQ(snapshot.union_points_base.back().z(), 0.25F);
}

TEST(IntegratedCargoIdentityShadowTest,
     GroupSupportedTopIsNotRecomputedDownstream) {
  CargoPhysicalGroupObservation group = physicalGroup(1.0);
  group.descriptor.physical_vertical_z = 1.031;
  group.representative.z95 = 0.42;
  const auto snapshot = bindCargoPhysicalGroupEvidence(
      {group}, decisionForGroup(), 2U);
  EXPECT_DOUBLE_EQ(snapshot.supported_top_z, 1.031);
}

TEST(IntegratedCargoIdentityShadowTest,
     FrameLocalCandidateIdChurnDoesNotLoseGroupEvidence) {
  const auto first = bindCargoPhysicalGroupEvidence(
      {physicalGroup(1.0, 3U)}, decisionForGroup(), 2U);
  const auto second = bindCargoPhysicalGroupEvidence(
      {physicalGroup(1.1, 88U)}, decisionForGroup(), 2U);
  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(second.valid);
  EXPECT_EQ(first.physical_history_id, second.physical_history_id);
  EXPECT_DOUBLE_EQ(second.supported_top_z, first.supported_top_z);
}

TEST(IntegratedCargoIdentityShadowTest,
     GeometryAuthorityConsumesResolvedGroupGeometry) {
  CargoShadowGeometryConfig config;
  config.formal_confirm_frames = 1;
  CargoShadowGeometryAuthority authority(config);
  const auto snapshot = bindCargoPhysicalGroupEvidence(
      {physicalGroup(1.0)}, decisionForGroup(), 2U);
  CargoShadowGeometryInput input;
  input.identity = decisionForGroup();
  input.stamp_sec = 1.0;
  input.geometry = snapshot.resolved_geometry;
  EXPECT_TRUE(authority.update(input).formal_geometry_valid);
}

TEST(IntegratedCargoIdentityShadowTest,
     RepeatedPipelineFrameDoesNotAdvanceGeometryConfirmation) {
  CargoShadowGeometryConfig config;
  config.formal_confirm_frames = 2;
  CargoShadowGeometryAuthority authority(config);
  CargoShadowGeometryInput input;
  input.identity = decisionForGroup();
  input.stamp_sec = 1.0;
  input.geometry = geometry(1.0);
  EXPECT_EQ(authority.update(input).confirm_count, 1);
  input.stamp_sec = 1.05;
  EXPECT_EQ(authority.update(input).confirm_count, 1);
  EXPECT_FALSE(authority.decision().formal_geometry_valid);
}

TEST(IntegratedCargoIdentityShadowTest,
     GroupEvidenceSourceStampCannotBeArtificiallyRefreshed) {
  CargoPhysicalGroupObservation group = physicalGroup(1.0);
  const auto snapshot = bindCargoPhysicalGroupEvidence(
      {group}, decisionForGroup(), 2U);
  EXPECT_DOUBLE_EQ(snapshot.source_stamp_sec, 1.0);
  EXPECT_NE(snapshot.source_stamp_sec, 1.25);
}

TEST(IntegratedCargoIdentityShadowTest,
     FormalShadowGeometryCanFreezeOwnedThickness) {
  const auto identity = decisionForGroup();
  const auto snapshot = bindCargoPhysicalGroupEvidence(
      {physicalGroup(1.0)}, identity, 2U);
  CargoShadowGeometryDecision formal;
  formal.geometry_resolved = true;
  formal.formal_geometry_valid = true;
  formal.physical_history_id = identity.physical_history_id;
  formal.source_stamp_sec = 1.0;
  formal.median_size = Eigen::Vector3d(1.0, 0.8, 0.4);
  CargoShadowThicknessState thickness;
  EXPECT_TRUE(thickness.freezeFromFormalGeometry(
      snapshot, identity, formal, 0.10F, 5.0F));
  EXPECT_FLOAT_EQ(thickness.frozen_thickness_m, 0.4F);
  EXPECT_EQ(thickness.provenance.source,
            "SHADOW_REFERENCE_INDEPENDENT_FORMAL_GEOMETRY");
}

TEST(IntegratedCargoIdentityShadowTest,
     ShadowThicknessOwnerMustMatchPhysicalHistory) {
  CargoShadowThicknessProvenance provenance;
  provenance.valid = true;
  provenance.physical_history_id = 8U;
  provenance.load_epoch = 2U;
  provenance.lifecycle_id = 2U;
  provenance.source_stamp_sec = 1.0;
  provenance.source = "SHADOW_REFERENCE_INDEPENDENT_FORMAL_GEOMETRY";
  EXPECT_FALSE(shadowThicknessAuthorized(
      provenance, decisionForGroup(), 2U));
}

TEST(IntegratedCargoIdentityShadowTest,
     ShadowThicknessResetsOnLoadEpochChange) {
  CargoShadowThicknessState thickness;
  thickness.provenance.valid = true;
  thickness.provenance.physical_history_id = 7U;
  thickness.provenance.load_epoch = 2U;
  thickness.provenance.lifecycle_id = 2U;
  thickness.provenance.source_stamp_sec = 1.0;
  thickness.provenance.source =
      "SHADOW_REFERENCE_INDEPENDENT_FORMAL_GEOMETRY";
  auto changed = decisionForGroup();
  changed.load_epoch = 3U;
  EXPECT_FALSE(shadowThicknessAuthorized(
      thickness.provenance, changed, 3U));
  thickness.reset();
  EXPECT_FALSE(thickness.provenance.valid);
}

TEST(IntegratedCargoIdentityShadowTest,
     ProductWrongThicknessCannotLeakIntoShadow) {
  CargoShadowThicknessProvenance provenance;
  provenance.valid = true;
  provenance.physical_history_id = 7U;
  provenance.load_epoch = 2U;
  provenance.lifecycle_id = 2U;
  provenance.source_stamp_sec = 1.0;
  provenance.source = "PRODUCT_FROZEN_THICKNESS";
  EXPECT_FALSE(shadowThicknessAuthorized(
      provenance, decisionForGroup(), 2U));
}

TEST(IntegratedCargoIdentityShadowTest,
     DirectTopFrozenUsesGroupSupportedTop) {
  const auto snapshot = bindCargoPhysicalGroupEvidence(
      {physicalGroup(1.0)}, decisionForGroup(), 2U);
  CargoBottomFusion fusion;
  CargoBottomObservation input;
  input.track_valid = true;
  input.track_id = snapshot.physical_history_id;
  input.stamp_sec = snapshot.source_stamp_sec;
  input.transform_stamp_sec = snapshot.source_stamp_sec;
  input.current_top_valid = true;
  input.current_top_support_valid = true;
  input.current_top_z_base = static_cast<float>(snapshot.supported_top_z);
  input.frozen_thickness_valid = true;
  input.frozen_thickness_m = 0.4F;
  input.frozen_thickness_stamp_sec = snapshot.source_stamp_sec;
  input.frozen_thickness_confidence = 1.0F;
  input.footprint_valid = true;
  input.footprint_center_base = Eigen::Vector2f::Zero();
  input.footprint_size_xy = Eigen::Vector2f(1.0F, 0.8F);
  const CargoBottomResult result = fusion.update(input);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.source,
            CargoBottomSource::DIRECT_TOP_FROZEN_THICKNESS);
  EXPECT_NEAR(result.geometry.bottom_z_base, 0.8F, 1.0e-5F);
}

TEST(IntegratedCargoIdentityShadowTest,
     NoThicknessAndNoLowerEdgeFailsClosed) {
  CargoBottomFusion fusion;
  CargoBottomObservation input;
  input.track_valid = true;
  input.track_id = 7U;
  input.stamp_sec = 1.0;
  input.transform_stamp_sec = 1.0;
  input.current_top_valid = true;
  input.current_top_support_valid = true;
  input.current_top_z_base = 1.2F;
  input.footprint_valid = true;
  input.footprint_center_base = Eigen::Vector2f::Zero();
  input.footprint_size_xy = Eigen::Vector2f(1.0F, 0.8F);
  for (int i = 0; i < 50; ++i) {
    input.points_base.emplace_back(
        0.01F * static_cast<float>(i % 10),
        0.01F * static_cast<float>(i / 10), 1.2F);
  }
  EXPECT_FALSE(fusion.update(input).valid);
}

TEST(IntegratedCargoIdentityShadowTest,
     ContinuityOnlyCannotProduceBottomOrClear) {
  CargoPhysicalGroupObservation group = physicalGroup(1.0);
  group.descriptor.vertical_mode = CargoGroupVerticalMode::CONTINUITY_ONLY;
  group.descriptor.physical_vertical_z = 1.0;
  const auto snapshot = bindCargoPhysicalGroupEvidence(
      {group}, decisionForGroup(), 2U);
  EXPECT_TRUE(snapshot.valid);
  EXPECT_FALSE(snapshot.supported_top_valid);
  CargoShadowFusionProjection projection;
  projection.formal = false;
  projection.bottom_valid = false;
  projection.clear_authorized = false;
  const auto input = projectShadowCargoOntoCanonicalFusion(
      CargoAvoidanceFusionInput{}, projection);
  EXPECT_FALSE(input.formal_cargo_bottom_valid);
  EXPECT_FALSE(input.formal_clear_authorized);
}

TEST(IntegratedCargoIdentityShadowTest,
     IdentityChangeResetsGroupEvidenceGeometryThicknessAndBottom) {
  const auto snapshot = bindCargoPhysicalGroupEvidence(
      {physicalGroup(1.0)}, decisionForGroup(), 2U);
  auto changed = decisionForGroup();
  changed.physical_history_id = 99U;
  EXPECT_FALSE(cargoPhysicalGroupEvidenceOwnerMatches(
      snapshot, changed, 2U));
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
