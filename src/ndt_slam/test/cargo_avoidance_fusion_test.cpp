#include "ndt_slam/cargo_avoidance_fusion.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

CargoAvoidanceFusionInput validInput() {
  CargoAvoidanceFusionInput input;
  input.localization_valid = true;
  input.formal_cargo_geometry_valid = true;
  input.formal_cargo_bottom_valid = true;
  input.formal_clear_authorized = true;
  input.static_session_manifest_valid = true;
  input.static_session_hash_valid = true;
  input.static_session_uuid_valid = true;
  input.static_risk_contract_valid = true;
  input.static_clear_contract_valid = true;
  input.static_authority = StaticEvidenceAuthority::RUNTIME_MATURE;
  input.live.available = true;
  input.live.reliable = true;
  input.live.coverage = 0.75F;
  input.static_map.available = true;
  input.static_map.reliable = true;
  return input;
}

CargoAvoidanceFusionInput pendingStaticHazardInput() {
  auto input = validInput();
  input.formal_cargo_geometry_valid = false;
  input.formal_cargo_bottom_valid = false;
  input.pending_envelope_valid = true;
  input.pending_envelope_source =
      PendingCargoEnvelopeSource::CURRENT_CANDIDATE;
  input.pending_pose_source =
      CargoEnvelopePoseSource::CURRENT_ASSOCIATED_LIDAR;
  input.pending_recognition_state_allows_warning = true;
  input.pending_pose_physically_plausible = true;
  input.pending_warning_query_allowed = true;
  input.pending_self_evidence_valid = true;
  input.pending_authority_confidence = 0.8F;
  input.pending_static_obstacle_authorized = true;
  input.pending_static_obstacle_id = 0x80000009U;
  input.pending_static_obstacle_confirmations = 3;
  input.pending_static_provenance_valid = true;
  input.pending_static_authority_confidence = 1.0F;
  input.live.available = false;
  input.live.reliable = false;
  input.static_map.hazard = true;
  input.static_map.warning_code = 17;
  input.static_map.distance_m = 2.0F;
  return input;
}

TEST(CargoAvoidanceFusion, ClearNeedsBothReliableSources) {
  auto input = validInput();
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 14);

  input.static_session_hash_valid = false;
  const auto rejected = fuseCargoAvoidanceRisk(input);
  EXPECT_FALSE(rejected.official_valid);
  EXPECT_EQ(rejected.official_code, 34);
}

TEST(CargoAvoidanceFusion, StaticHazardSurvivesLiveClearConflict) {
  auto input = validInput();
  input.static_map.hazard = true;
  input.static_map.warning_code = 17;
  input.static_map.distance_m = 2.0F;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 17);
  EXPECT_TRUE(result.map_live_conflict);
  EXPECT_EQ(result.reason, "MAP_LIVE_CONFLICT_static_hazard_retained");
}

TEST(CargoAvoidanceFusion,
     FormalClearAuthorityBlocksClearButNotPositiveHazard) {
  auto input = validInput();
  input.formal_clear_authorized = false;
  const auto clear = fuseCargoAvoidanceRisk(input);
  EXPECT_FALSE(clear.official_valid);
  EXPECT_EQ(clear.official_code, 33);
  EXPECT_EQ(clear.reason, "formal_cargo_clear_not_authorized");

  input.live.hazard = true;
  input.live.warning_code = 17;
  const auto hazard = fuseCargoAvoidanceRisk(input);
  EXPECT_TRUE(hazard.official_valid);
  EXPECT_EQ(hazard.official_code, 17);
}

TEST(CargoAvoidanceFusion, StaticHazardSurvivesLiveBlank) {
  auto input = validInput();
  input.live.available = false;
  input.live.reliable = false;
  input.static_map.hazard = true;
  input.static_map.warning_code = 17;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 17);
}

TEST(CargoAvoidanceFusion, PendingEnvelopeCannotGrantClear) {
  auto input = validInput();
  input.formal_cargo_geometry_valid = false;
  input.pending_envelope_valid = true;
  input.pending_recognition_state_allows_warning = true;
  input.pending_pose_physically_plausible = true;
  input.pending_warning_query_allowed = true;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_FALSE(result.official_valid);
  EXPECT_EQ(result.official_code, 33);
  EXPECT_EQ(result.provisional_status, "CLEAR_NOT_AUTHORIZED");
}

TEST(CargoAvoidanceFusion, ConfiguredPendingHazardStaysDiagnosticByDefault) {
  auto input = validInput();
  input.formal_cargo_geometry_valid = false;
  input.formal_cargo_bottom_valid = false;
  input.pending_envelope_valid = true;
  input.pending_envelope_source =
      PendingCargoEnvelopeSource::CONFIGURED_CONSERVATIVE;
  input.pending_pose_source = CargoEnvelopePoseSource::HOOK_DEFAULT_OFFSET;
  input.pending_recognition_state_allows_warning = true;
  input.pending_pose_physically_plausible = true;
  input.pending_warning_query_allowed = false;
  input.live.hazard = true;
  input.live.warning_code = 17;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_FALSE(result.official_valid);
  EXPECT_EQ(result.official_code, 33);
  EXPECT_EQ(result.provisional_status, "QUERY_NOT_AUTHORIZED");
  EXPECT_EQ(
      result.pending_authority_reason,
      "pending_warning_query_not_authorized");
}

TEST(CargoAvoidanceFusion, EvidenceBackedPendingHazardCanWarn) {
  auto input = validInput();
  input.formal_cargo_geometry_valid = false;
  input.formal_cargo_bottom_valid = false;
  input.pending_envelope_valid = true;
  input.pending_envelope_source =
      PendingCargoEnvelopeSource::CURRENT_CANDIDATE;
  input.pending_pose_source =
      CargoEnvelopePoseSource::CURRENT_ASSOCIATED_LIDAR;
  input.pending_recognition_state_allows_warning = true;
  input.pending_pose_physically_plausible = true;
  input.pending_warning_query_allowed = true;
  input.pending_self_evidence_valid = true;
  input.pending_external_separation_valid = true;
  input.pending_external_obstacle_authorized = true;
  input.pending_external_obstacle_track_id = 9U;
  input.pending_external_obstacle_confirmations = 3;
  input.pending_external_provenance_valid = true;
  input.pending_external_geometry_valid = true;
  input.pending_authority_confidence = 0.8F;
  input.live.hazard = true;
  input.live.warning_code = 18;
  CargoAvoidanceFusionConfig config;
  const auto result = fuseCargoAvoidanceRisk(input, config);
  ASSERT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 18);
  EXPECT_NE(result.official_code, 14);
  EXPECT_TRUE(result.pending_warning_authorized);
}

TEST(CargoAvoidanceFusion, PendingNeedsStableExternalObstacleIdentity) {
  auto input = validInput();
  input.formal_cargo_geometry_valid = false;
  input.formal_cargo_bottom_valid = false;
  input.pending_envelope_valid = true;
  input.pending_envelope_source =
      PendingCargoEnvelopeSource::CURRENT_CANDIDATE;
  input.pending_pose_source =
      CargoEnvelopePoseSource::CURRENT_ASSOCIATED_LIDAR;
  input.pending_recognition_state_allows_warning = true;
  input.pending_pose_physically_plausible = true;
  input.pending_warning_query_allowed = true;
  input.pending_self_evidence_valid = true;
  input.pending_external_separation_valid = true;
  input.pending_external_obstacle_authorized = true;
  input.pending_external_obstacle_track_id = 9U;
  input.pending_external_obstacle_confirmations = 2;
  input.pending_external_provenance_valid = true;
  input.pending_external_geometry_valid = true;
  input.pending_authority_confidence = 0.8F;
  input.live.hazard = true;
  input.live.warning_code = 17;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_FALSE(result.official_valid);
  EXPECT_EQ(result.official_code, 33);
  EXPECT_EQ(
      result.pending_authority_reason,
      "external_obstacle_confirmation_pending");
}

TEST(CargoAvoidanceFusion, LegacyPolicyCannotBypassExternalTrackIdentity) {
  auto input = validInput();
  input.formal_cargo_geometry_valid = false;
  input.formal_cargo_bottom_valid = false;
  input.pending_envelope_valid = true;
  input.pending_envelope_source =
      PendingCargoEnvelopeSource::CURRENT_CANDIDATE;
  input.pending_pose_source =
      CargoEnvelopePoseSource::CURRENT_ASSOCIATED_LIDAR;
  input.pending_recognition_state_allows_warning = true;
  input.pending_pose_physically_plausible = true;
  input.pending_warning_query_allowed = true;
  input.pending_self_evidence_valid = true;
  input.pending_external_separation_valid = true;
  input.pending_external_obstacle_authorized = true;
  input.pending_external_obstacle_track_id = 0U;
  input.pending_external_provenance_valid = true;
  input.live.hazard = true;
  input.live.warning_code = 17;
  CargoAvoidanceFusionConfig config;
  config.pending_warning_promotion_policy =
      PendingWarningPromotionPolicy::LEGACY_ANY_PENDING;
  const auto result = fuseCargoAvoidanceRisk(input, config);
  EXPECT_FALSE(result.official_valid);
  EXPECT_EQ(result.official_code, 33);
  EXPECT_EQ(result.pending_authority_reason,
            "external_obstacle_identity_missing");
}

TEST(CargoAvoidanceFusion, UnverifiedStaticCannotAuthorizeClearOrHazard) {
  auto input = validInput();
  input.static_authority =
      StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN;
  input.static_map.hazard = true;
  input.static_map.warning_code = 17;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_FALSE(result.official_valid);
  EXPECT_EQ(result.official_code, 34);
}

TEST(CargoAvoidanceFusion, PendingUnverifiedStaticRemainsAdvisoryOnly) {
  auto input = validInput();
  input.formal_cargo_geometry_valid = false;
  input.formal_cargo_bottom_valid = false;
  input.pending_envelope_valid = true;
  input.pending_recognition_state_allows_warning = true;
  input.pending_pose_physically_plausible = true;
  input.pending_warning_query_allowed = true;
  input.static_authority =
      StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN;
  input.static_map.hazard = true;
  input.static_map.warning_code = 17;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_FALSE(result.official_valid);
  EXPECT_EQ(result.official_code, 33);
  EXPECT_FALSE(result.risk_static);
  EXPECT_EQ(result.provisional_status, "CLEAR_NOT_AUTHORIZED");
}

TEST(CargoAvoidanceFusion,
     ConfirmedAuthoritativeStaticPendingHazardCanWarnWithoutLiveCloud) {
  const auto result =
      fuseCargoAvoidanceRisk(pendingStaticHazardInput());
  ASSERT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 17);
  EXPECT_TRUE(result.pending_warning_authorized);
  EXPECT_FALSE(result.pending_live_warning_authorized);
  EXPECT_TRUE(result.pending_static_warning_authorized);
  EXPECT_EQ(
      result.reason, "pending_static_warning_authorized");
}

TEST(CargoAvoidanceFusion,
     StaticPendingHazardNeedsStableRegionConfirmation) {
  auto input = pendingStaticHazardInput();
  input.static_map.warning_code = 18;
  input.pending_static_obstacle_authorized = false;
  input.pending_static_obstacle_confirmations = 2;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_FALSE(result.official_valid);
  EXPECT_EQ(result.official_code, 33);
  EXPECT_FALSE(result.pending_static_warning_authorized);
  EXPECT_EQ(
      result.pending_authority_reason,
      "static_obstacle_identity_missing");
}

TEST(CargoAvoidanceFusion,
     StaticPendingHazardCannotBypassCargoIdentityConfidence) {
  auto input = pendingStaticHazardInput();
  input.pending_authority_confidence = 0.20F;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_FALSE(result.official_valid);
  EXPECT_EQ(
      result.pending_authority_reason,
      "pending_cargo_identity_confidence_low");
}

TEST(CargoAvoidanceFusion,
     ConfirmedStaticPendingHazardSurvivesAmbiguousLiveSelfPoints) {
  auto input = pendingStaticHazardInput();
  input.live.available = true;
  input.live.reliable = true;
  input.live.hazard = true;
  input.live.warning_code = 17;
  input.live_obstacle_origin_resolved = false;
  const auto result = fuseCargoAvoidanceRisk(input);
  ASSERT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 17);
  EXPECT_TRUE(result.pending_static_warning_authorized);
  EXPECT_FALSE(result.pending_live_warning_authorized);
}

TEST(CargoAvoidanceFusion, MoreSevereSourceWins) {
  auto input = validInput();
  input.live.hazard = true;
  input.live.warning_code = 18;
  input.static_map.hazard = true;
  input.static_map.warning_code = 17;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 17);
}

TEST(CargoAvoidanceFusion,
     StationaryHazardIsCode33WhileTrackingRemainsDiagnostic) {
  CargoAvoidanceFusionInput input = validInput();
  input.warning_motion_authorized = false;
  input.live.available = false;
  input.live.reliable = false;
  input.warning_candidate_present = true;
  input.warning_candidate_code = 18;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_FALSE(result.official_valid);
  EXPECT_EQ(result.official_code, 33);
  EXPECT_EQ(
      result.reason,
      "motion_not_authoritative_warning_suppressed_track_preserved");
}

TEST(CargoAvoidanceFusion,
     Level1NeedsConfirmedHistoryOutsideThreeMeters) {
  CargoAvoidanceFusionInput input = validInput();
  input.near_field_history_authorized = false;
  input.live.available = false;
  input.live.reliable = false;
  input.warning_candidate_present = true;
  input.warning_candidate_code = 17;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_FALSE(result.official_valid);
  EXPECT_EQ(result.official_code, 33);
  EXPECT_EQ(result.reason, "near_field_track_missing_far_history");
}

TEST(CargoAvoidanceFusion,
     UnresolvedEmbeddedLiveClusterCannotWarnOrAuthorizeClear) {
  auto input = validInput();
  input.live.hazard = true;
  input.live.warning_code = 17;
  input.live.distance_m = 0.0F;
  input.live_obstacle_origin_resolved = false;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_FALSE(result.official_valid);
  EXPECT_EQ(result.official_code, 34);
  EXPECT_FALSE(result.risk_live);
  EXPECT_EQ(result.reason, "embedded_obstacle_origin_unresolved");
}

}  // namespace
}  // namespace ndt_slam
