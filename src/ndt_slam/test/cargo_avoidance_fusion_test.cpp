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
  input.live.distance_m = 2.0F;
  input.live.clearance_m = -0.10F;
  input.static_map.available = true;
  input.static_map.reliable = true;
  input.static_map.distance_m = 2.0F;
  input.static_map.clearance_m = -0.10F;
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
  input.static_map.certified_static_provenance = true;
  input.static_near_field_history_authorized = true;
  return input;
}

CargoAvoidanceFusionConfig pendingPromotionConfig() {
  CargoAvoidanceFusionConfig config;
  config.pending_warning_promotion_policy =
      PendingWarningPromotionPolicy::EVIDENCE_BACKED_ONLY;
  return config;
}

TEST(CargoAvoidanceFusion, FormalLiveClearDoesNotRequireStaticBaseline) {
  auto input = validInput();
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 14);

  input.static_session_hash_valid = false;
  const auto without_baseline = fuseCargoAvoidanceRisk(input);
  EXPECT_TRUE(without_baseline.official_valid);
  EXPECT_EQ(without_baseline.official_code, 14);
}

TEST(CargoAvoidanceFusion, StaticHazardSurvivesLiveClearConflict) {
  auto input = validInput();
  input.static_map.hazard = true;
  input.static_map.warning_code = 17;
  input.static_map.distance_m = 2.0F;
  input.static_map.certified_static_provenance = true;
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
  input.static_map.certified_static_provenance = true;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 17);
}

TEST(CargoAvoidanceFusion,
     FormalMatureStaticLevel1UsesIndependentMapHistory) {
  auto input = validInput();
  input.live.available = false;
  input.live.reliable = false;
  input.static_map.hazard = true;
  input.static_map.warning_code = 17;
  input.static_map.distance_m = 2.5F;
  input.static_map.certified_static_provenance = true;
  input.static_near_field_history_authorized = false;
  input.static_hazard_track_confirmed = true;
  const auto result = fuseCargoAvoidanceRisk(input);
  ASSERT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 17);
  EXPECT_FALSE(result.anomaly_review_static);
}

TEST(CargoAvoidanceFusion,
     RuntimeStaticEvidenceCannotReplaceTrueFarHistory) {
  auto input = validInput();
  input.live.available = false;
  input.live.reliable = false;
  input.static_map.hazard = true;
  input.static_map.warning_code = 18;
  input.static_map.distance_m = 4.0F;
  input.static_map.certified_static_provenance = false;
  input.static_near_field_history_authorized = false;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_FALSE(result.official_valid);
  EXPECT_EQ(result.official_code, 34);
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
  const CargoAvoidanceFusionConfig config = pendingPromotionConfig();
  const auto result = fuseCargoAvoidanceRisk(input, config);
  ASSERT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 18);
  EXPECT_NE(result.official_code, 14);
  EXPECT_TRUE(result.pending_warning_authorized);
}

// ========== 修复 ==========
// 默认值已从 DISABLED 改为 EVIDENCE_BACKED_ONLY（与生产 YAML 一致）。
// 此测试改为显式设置 DISABLED 以验证禁用行为。
TEST(CargoAvoidanceFusion, PendingWarningPromotionDisabledExplicitly) {
  CargoAvoidanceFusionConfig config;
  config.pending_warning_promotion_policy =
      PendingWarningPromotionPolicy::DISABLED;
  auto input = pendingStaticHazardInput();
  input.static_map.hazard = false;
  input.live.available = true;
  input.live.reliable = true;
  input.live.hazard = true;
  input.live.warning_code = 18;
  input.pending_external_separation_valid = true;
  input.pending_external_obstacle_authorized = true;
  input.pending_external_obstacle_track_id = 9U;
  input.pending_external_obstacle_confirmations = 3;
  input.pending_external_provenance_valid = true;
  input.pending_external_geometry_valid = true;
  const auto result = fuseCargoAvoidanceRisk(input, config);
  EXPECT_FALSE(result.official_valid);
  EXPECT_EQ(result.official_code, 33);
  EXPECT_FALSE(result.pending_live_warning_authorized);
  EXPECT_EQ(result.pending_authority_reason, "policy_disabled");
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
     ConfirmedAuthoritativeStaticPendingHazardWithApproachHistoryCanWarn) {
  const auto result =
      fuseCargoAvoidanceRisk(
          pendingStaticHazardInput(), pendingPromotionConfig());
  ASSERT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 17);
  EXPECT_TRUE(result.pending_warning_authorized);
  EXPECT_FALSE(result.pending_live_warning_authorized);
  EXPECT_TRUE(result.pending_static_warning_authorized);
  EXPECT_EQ(
      result.reason, "pending_static_warning_authorized");
}

TEST(CargoAvoidanceFusion,
     ConfirmedStaticPendingLevel1UsesIndependentMapHistory) {
  auto input = pendingStaticHazardInput();
  input.static_near_field_history_authorized = false;
  const auto result =
      fuseCargoAvoidanceRisk(input, pendingPromotionConfig());
  ASSERT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 17);
  EXPECT_FALSE(result.anomaly_review_static);
  EXPECT_TRUE(result.pending_static_warning_authorized);
}

TEST(CargoAvoidanceFusion,
     UnconfirmedStaticNearFieldCannotEmitReviewOrLevel1) {
  auto input = pendingStaticHazardInput();
  input.static_hazard_track_confirmed = false;
  input.static_near_field_history_authorized = false;
  const auto result =
      fuseCargoAvoidanceRisk(input, pendingPromotionConfig());
  EXPECT_FALSE(result.official_valid);
  EXPECT_EQ(result.official_code, 34);
  EXPECT_FALSE(result.anomaly_review);
  EXPECT_EQ(result.reason, "static_hazard_track_confirmation_pending");
}

TEST(CargoAvoidanceFusion,
     StaticPendingHazardNeedsStableRegionConfirmation) {
  auto input = pendingStaticHazardInput();
  input.static_map.warning_code = 18;
  input.pending_static_obstacle_authorized = false;
  input.pending_static_obstacle_confirmations = 2;
  const auto result = fuseCargoAvoidanceRisk(
      input, pendingPromotionConfig());
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
  const auto result = fuseCargoAvoidanceRisk(
      input, pendingPromotionConfig());
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
  const auto result = fuseCargoAvoidanceRisk(
      input, pendingPromotionConfig());
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
  input.static_map.certified_static_provenance = true;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 17);
}

// ========== 修复 ==========
// motion_not_authoritative 不再立即返回 33。
// 运动方向只用于更新 approach/far-field history。
// 静止或方向未知时，已确认障碍仍允许输出 17/18。
// 但此处 warning_candidate_present 且 live/static 均不可靠，
// 因此代码仍回落为 33（cargo_invalid）。
TEST(CargoAvoidanceFusion,
     StationaryHazardKeepsDiagnosticButDoesNotBlockConfirmedRisk) {
  CargoAvoidanceFusionInput input = validInput();
  input.warning_motion_authorized = false;
  input.live.available = false;
  input.live.reliable = false;
  input.warning_candidate_present = true;
  input.warning_candidate_code = 18;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_FALSE(result.official_valid);
  // motion_not_authoritative 不阻塞后续标准路径，但此处无 formal/positive-only 证据
  EXPECT_TRUE(result.motion_not_authoritative);
}

TEST(CargoAvoidanceFusion,
     Level1NeedsConfirmedHistoryOutsideThreeMeters) {
  CargoAvoidanceFusionInput input = validInput();
  input.live_near_field_history_authorized = false;
  input.live.hazard = true;
  input.live.warning_code = 17;
  input.live.distance_m = 2.0F;
  input.live.clearance_m = -0.10F;
  input.live.obstacle_track_id = 81U;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 29);
  EXPECT_TRUE(result.anomaly_review);
  EXPECT_EQ(result.reason, "review_warning_without_true_far_history");
}

TEST(CargoAvoidanceFusion, NearHazardWithoutTrueFarHistoryProduces29) {
  CargoAvoidanceFusionInput input = validInput();
  input.live.hazard = true;
  input.live.warning_code = 18;
  input.live.distance_m = 4.0F;
  input.live.clearance_m = 0.10F;
  input.live.obstacle_track_id = 801U;
  input.live_near_field_history_authorized = false;

  const auto result = fuseCargoAvoidanceRisk(input);

  ASSERT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 29);
  EXPECT_TRUE(result.anomaly_review_live);
  ASSERT_TRUE(result.authoritative_hazard.valid);
  EXPECT_EQ(result.authoritative_hazard.obstacle_track_id, 801U);
}

TEST(CargoAvoidanceFusion, Code29DoesNotRequireFarHistory) {
  CargoAvoidanceFusionInput input = validInput();
  input.live.hazard = true;
  input.live.warning_code = 17;
  input.live.distance_m = 2.0F;
  input.live.clearance_m = 0.0F;
  input.live_near_field_history_authorized = false;

  const auto result = fuseCargoAvoidanceRisk(input);

  EXPECT_EQ(result.official_code, 29);
  EXPECT_FALSE(result.authoritative_hazard.far_field_history_valid);
}

TEST(CargoAvoidanceFusion, Code17StillRequiresTrueFarHistory) {
  CargoAvoidanceFusionInput input = validInput();
  input.live.hazard = true;
  input.live.warning_code = 17;
  input.live.distance_m = 2.0F;
  input.live.clearance_m = 0.10F;
  input.live_near_field_history_authorized = false;

  EXPECT_EQ(fuseCargoAvoidanceRisk(input).official_code, 29);
  input.live_near_field_history_authorized = true;
  EXPECT_EQ(fuseCargoAvoidanceRisk(input).official_code, 17);
}

TEST(CargoAvoidanceFusion, Code18StillRequiresTrueFarHistory) {
  CargoAvoidanceFusionInput input = validInput();
  input.live.hazard = true;
  input.live.warning_code = 18;
  input.live.distance_m = 4.0F;
  input.live.clearance_m = 0.10F;
  input.live_near_field_history_authorized = false;

  EXPECT_EQ(fuseCargoAvoidanceRisk(input).official_code, 29);
  input.live_near_field_history_authorized = true;
  EXPECT_EQ(fuseCargoAvoidanceRisk(input).official_code, 18);
}

TEST(CargoAvoidanceFusion, NoFarReviewCannotAuthorizeClear) {
  CargoAvoidanceFusionInput input = validInput();
  input.live.hazard = true;
  input.live.warning_code = 18;
  input.live.distance_m = 4.0F;
  input.live.clearance_m = 0.10F;
  input.live_near_field_history_authorized = false;
  input.formal_clear_authorized = true;

  const auto result = fuseCargoAvoidanceRisk(input);

  EXPECT_EQ(result.official_code, 29);
  EXPECT_NE(result.official_code, 14);
}

TEST(CargoAvoidanceFusion,
     StaticFarHistoryCannotAuthorizeSuddenLiveLevel1) {
  CargoAvoidanceFusionInput input = validInput();
  input.live_near_field_history_authorized = false;
  input.static_near_field_history_authorized = true;
  input.live.hazard = true;
  input.live.warning_code = 17;
  input.live.distance_m = 2.0F;
  input.live.clearance_m = -0.10F;
  input.live.obstacle_track_id = 82U;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 29);
  EXPECT_TRUE(result.anomaly_review_live);
  EXPECT_FALSE(result.anomaly_review_static);
}

// ========== 修复 ==========
// anomaly_review (29) 现在在标准告警之后评估。
// Live 0.30m contact evidence can be cargo-self segmentation, so it remains
// Code 29 even when the live tracker has history. Independently mature static
// evidence is handled by the separate priority test below.
TEST(CargoAvoidanceFusion, ImmediateLiveContactRemainsReviewOnly) {
  CargoAvoidanceFusionInput input = validInput();
  input.live.hazard = true;
  input.live.warning_code = 17;
  input.live.distance_m = 0.20F;
  input.live.clearance_m = -0.10F;
  input.live.obstacle_track_id = 91U;
  input.anomaly_review_candidate = true;
  input.anomaly_review_live = true;
  input.anomaly_review_reason = "review_immediate_contact_guard";
  input.anomaly_review_distance_m = 0.20F;
  input.anomaly_review_clearance_m = -0.10F;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 29);
  EXPECT_TRUE(result.anomaly_review);
  ASSERT_TRUE(result.authoritative_hazard.valid);
  EXPECT_EQ(result.authoritative_hazard.source,
            CargoAvoidanceHazardSource::LIVE);
  EXPECT_EQ(result.authoritative_hazard.obstacle_track_id, 91U);
  EXPECT_FLOAT_EQ(result.distance_m, 0.20F);
}

TEST(CargoAvoidanceFusion, AnomalyReviewWhenNoStandardHazard) {
  CargoAvoidanceFusionInput input = validInput();
  input.live.hazard = true;
  input.live.warning_code = 17;
  input.live.distance_m = 0.20F;
  input.live.clearance_m = -0.10F;
  input.live.obstacle_track_id = 92U;
  input.static_map.hazard = false;
  input.anomaly_review_candidate = true;
  input.anomaly_review_live = true;
  input.anomaly_review_reason = "review_immediate_contact_guard";
  input.anomaly_review_distance_m = 0.20F;
  input.anomaly_review_clearance_m = -0.10F;
  input.formal_clear_authorized = false;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 29);
  EXPECT_TRUE(result.anomaly_review_live);
  ASSERT_TRUE(result.authoritative_hazard.valid);
  EXPECT_EQ(result.authoritative_hazard.obstacle_track_id, 92U);
}

TEST(CargoAvoidanceFusion, ReviewWithoutAuthoritativeHazardIsCode34) {
  CargoAvoidanceFusionInput input = validInput();
  input.live.hazard = false;
  input.anomaly_review_candidate = true;
  input.anomaly_review_live = true;
  input.anomaly_review_distance_m = 0.20F;
  input.anomaly_review_clearance_m = -0.10F;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_FALSE(result.official_valid);
  EXPECT_EQ(result.official_code, 34);
  EXPECT_EQ(result.reason, "review_hazard_identity_or_metrics_invalid");
}

TEST(CargoAvoidanceFusion, ClearanceAtPointEightForbidsAllHazards) {
  CargoAvoidanceFusionInput input = validInput();
  input.live.hazard = true;
  input.live.warning_code = 17;
  input.live.distance_m = 2.0F;
  input.live.clearance_m = 0.80F;
  input.anomaly_review_candidate = true;
  input.anomaly_review_live = true;
  input.anomaly_review_distance_m = 0.20F;
  input.anomaly_review_clearance_m = 0.80F;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 14);
  EXPECT_NE(result.official_code, 17);
  EXPECT_NE(result.official_code, 18);
  EXPECT_NE(result.official_code, 29);
}

TEST(CargoAvoidanceFusion, MissingFarHistoryAtFourMetersBecomesReview) {
  CargoAvoidanceFusionInput input = validInput();
  input.live.hazard = true;
  input.live.warning_code = 18;
  input.live.distance_m = 4.0F;
  input.live.clearance_m = 0.20F;
  input.live.obstacle_track_id = 93U;
  input.live_near_field_history_authorized = false;
  const auto result = fuseCargoAvoidanceRisk(input);
  ASSERT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 29);
  ASSERT_TRUE(result.authoritative_hazard.valid);
  EXPECT_EQ(result.authoritative_hazard.obstacle_track_id, 93U);
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

TEST(CargoAvoidanceFusion,
     AuthoritativeHazardKeepsCodeDistanceAndClearanceFromOneSource) {
  auto input = validInput();
  input.live.hazard = true;
  input.live.warning_code = 18;
  input.live.distance_m = 3.4F;
  input.live.clearance_m = -0.20F;
  input.static_map.available = true;
  input.static_map.reliable = true;
  input.static_map.hazard = true;
  input.static_map.warning_code = 17;
  input.static_map.distance_m = 2.9F;
  input.static_map.clearance_m = -0.05F;
  input.static_map.cargo_lifecycle_id = 41U;
  input.static_map.cargo_track_id = 42U;
  input.static_map.obstacle_track_id = 43U;
  input.static_map.pose_generation = 44U;
  input.static_map.map_generation = 45U;
  input.static_map.obstacle_top_z_map = 3.2F;
  input.static_map.uncertainty_m = 0.08F;
  input.static_map.confidence = 0.95F;
  input.static_map.far_field_history_valid = true;
  input.static_map.provenance_valid = true;
  input.static_map.certified_static_provenance = true;
  input.static_map.validated_streak = 7;
  input.static_hazard_track_confirmed = true;
  const auto result = fuseCargoAvoidanceRisk(input);
  ASSERT_TRUE(result.authoritative_hazard.valid);
  EXPECT_EQ(result.warning_authority, CargoWarningAuthority::FORMAL);
  EXPECT_EQ(result.authoritative_hazard.source,
            CargoAvoidanceHazardSource::STATIC_MAP);
  EXPECT_EQ(result.official_code, 17);
  EXPECT_FLOAT_EQ(result.distance_m, 2.9F);
  EXPECT_FLOAT_EQ(result.clearance_m, -0.05F);
  EXPECT_EQ(result.authoritative_hazard.cargo_lifecycle_id, 41U);
  EXPECT_EQ(result.authoritative_hazard.cargo_track_id, 42U);
  EXPECT_EQ(result.authoritative_hazard.obstacle_track_id, 43U);
  EXPECT_EQ(result.authoritative_hazard.pose_generation, 44U);
  EXPECT_EQ(result.authoritative_hazard.map_generation, 45U);
  EXPECT_FLOAT_EQ(result.authoritative_hazard.obstacle_top_z_map, 3.2F);
  EXPECT_FLOAT_EQ(result.authoritative_hazard.uncertainty_m, 0.08F);
  EXPECT_FLOAT_EQ(result.authoritative_hazard.confidence, 0.95F);
  EXPECT_TRUE(result.authoritative_hazard.far_field_history_valid);
  EXPECT_TRUE(result.authoritative_hazard.provenance_valid);
  EXPECT_EQ(result.authoritative_hazard.validated_streak, 7);
}

TEST(CargoAvoidanceFusion,
     SuddenLiveReviewCannotOverrideMatureStaticLevel1) {
  auto input = validInput();
  input.live.hazard = true;
  input.live.warning_code = 17;
  input.live.distance_m = 0.20F;
  input.live_near_field_history_authorized = false;
  input.static_map.hazard = true;
  input.static_map.warning_code = 17;
  input.static_map.distance_m = 2.80F;
  input.static_map.clearance_m = -0.10F;
  input.static_map.certified_static_provenance = true;
  input.static_near_field_history_authorized = false;
  input.static_hazard_track_confirmed = true;
  const auto result = fuseCargoAvoidanceRisk(input);
  ASSERT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 17);
  ASSERT_TRUE(result.authoritative_hazard.valid);
  EXPECT_EQ(result.authoritative_hazard.source,
            CargoAvoidanceHazardSource::STATIC_MAP);
  EXPECT_FLOAT_EQ(result.distance_m, 2.80F);
  EXPECT_FALSE(result.anomaly_review);
}

TEST(CargoAvoidanceFusion,
     UnboundRawWarningCandidateCannotFallThroughToClear) {
  auto input = validInput();
  input.warning_candidate_present = true;
  input.warning_candidate_code = 18;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_FALSE(result.official_valid);
  EXPECT_EQ(result.official_code, 34);
  EXPECT_EQ(result.reason, "warning_candidate_source_not_authorized");
}

}  // namespace
}  // namespace ndt_slam
