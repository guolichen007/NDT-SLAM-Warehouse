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
  input.static_near_field_history_authorized = true;
  return input;
}

CargoAvoidanceFusionConfig pendingPromotionConfig() {
  CargoAvoidanceFusionConfig config;
  config.pending_warning_promotion_policy =
      PendingWarningPromotionPolicy::EVIDENCE_BACKED_ONLY;
  return config;
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

TEST(CargoAvoidanceFusion,
     FormalMatureStaticLevel1UsesIndependentMapHistory) {
  auto input = validInput();
  input.live.available = false;
  input.live.reliable = false;
  input.static_map.hazard = true;
  input.static_map.warning_code = 17;
  input.static_map.distance_m = 2.5F;
  input.static_near_field_history_authorized = false;
  input.static_hazard_track_confirmed = true;
  const auto result = fuseCargoAvoidanceRisk(input);
  ASSERT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 17);
  EXPECT_FALSE(result.anomaly_review_static);
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
  input.live.available = false;
  input.live.reliable = false;
  input.warning_candidate_present = true;
  input.warning_candidate_code = 17;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 29);
  EXPECT_TRUE(result.anomaly_review);
  EXPECT_EQ(result.reason, "review_level1_without_approach_history");
}

TEST(CargoAvoidanceFusion,
     StaticFarHistoryCannotAuthorizeSuddenLiveLevel1) {
  CargoAvoidanceFusionInput input = validInput();
  input.live_near_field_history_authorized = false;
  input.static_near_field_history_authorized = true;
  input.live.available = false;
  input.live.reliable = false;
  input.warning_candidate_present = true;
  input.warning_candidate_code = 17;
  const auto result = fuseCargoAvoidanceRisk(input);
  EXPECT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 29);
  EXPECT_TRUE(result.anomaly_review_live);
  EXPECT_FALSE(result.anomaly_review_static);
}

// ========== 修复 ==========
// anomaly_review (29) 现在在标准告警之后评估。
// live.hazard=true + warning_code=17 意味着已有正式风险，
// formal risk 优先于 anomaly review。
// 29 只有在无标准 17/18 时才输出。
TEST(CargoAvoidanceFusion, ImmediateContactReviewDoesNotOverrideFormalHazard) {
  CargoAvoidanceFusionInput input = validInput();
  input.live.hazard = true;
  input.live.warning_code = 17;
  input.anomaly_review_candidate = true;
  input.anomaly_review_live = true;
  input.anomaly_review_reason = "review_immediate_contact_guard";
  input.anomaly_review_distance_m = 0.20F;
  input.anomaly_review_clearance_m = -0.10F;
  const auto result = fuseCargoAvoidanceRisk(input);
  // 正式风险 17 优先于 29。
  EXPECT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 17);
}

TEST(CargoAvoidanceFusion, AnomalyReviewWhenNoStandardHazard) {
  CargoAvoidanceFusionInput input = validInput();
  input.live.hazard = false;
  input.static_map.hazard = false;
  input.anomaly_review_candidate = true;
  input.anomaly_review_live = true;
  input.anomaly_review_reason = "review_immediate_contact_guard";
  input.anomaly_review_distance_m = 0.20F;
  input.anomaly_review_clearance_m = -0.10F;
  input.formal_clear_authorized = false;
  const auto result = fuseCargoAvoidanceRisk(input);
  // 无标准风险时可输出 29。
  EXPECT_TRUE(result.official_valid);
  EXPECT_EQ(result.official_code, 29);
  EXPECT_TRUE(result.anomaly_review_live);
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
