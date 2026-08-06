#include "ndt_slam/cargo_avoidance_fusion.hpp"
#include "ndt_slam/static_evidence_authorization.hpp"

#include <algorithm>
#include <cmath>

namespace ndt_slam {
namespace {

constexpr std::int32_t kClear = 14;
constexpr std::int32_t kNear3m = 17;
constexpr std::int32_t kNear5m = 18;
constexpr std::int32_t kAnomalyReview = 29;
constexpr std::int32_t kLocalizationInvalid = 31;
constexpr std::int32_t kCargoInvalid = 33;
constexpr std::int32_t kObstacleInvalid = 34;

bool warningCode(std::int32_t code) {
  return code == kNear3m || code == kNear5m;
}

std::int32_t moreSevere(std::int32_t lhs, std::int32_t rhs) {
  if (lhs == kNear3m || rhs == kNear3m) return kNear3m;
  if (lhs == kNear5m || rhs == kNear5m) return kNear5m;
  return 0;
}

void combineMetric(float value, float* output) {
  if (!std::isfinite(value)) return;
  if (!std::isfinite(*output)) {
    *output = value;
  } else {
    *output = std::min(*output, value);
  }
}

bool pendingSourceCanCarryIdentity(PendingCargoEnvelopeSource source) {
  return source == PendingCargoEnvelopeSource::CURRENT_CANDIDATE ||
      source == PendingCargoEnvelopeSource::ACTIVE_LOCKED_TRACK ||
      source == PendingCargoEnvelopeSource::RETIRED_FORMAL_SHAPE ||
      source == PendingCargoEnvelopeSource::LIFT_ORIGIN_CANDIDATE;
}

bool pendingPoseCanCarryIdentity(CargoEnvelopePoseSource source) {
  return source == CargoEnvelopePoseSource::CURRENT_ASSOCIATED_LIDAR ||
      source == CargoEnvelopePoseSource::SHORT_TERM_TRACK_PREDICTION ||
      source == CargoEnvelopePoseSource::RETIRED_TRACK_PREDICTION ||
      source == CargoEnvelopePoseSource::HOOK_PLUS_LAST_RELIABLE_OFFSET;
}

bool authorizePendingWarning(
    const CargoAvoidanceFusionInput& input,
    const CargoAvoidanceFusionConfig& config,
    std::string* reason) {
  if (!input.pending_recognition_state_allows_warning) {
    *reason = input.pending_warning_state_reason.empty()
        ? "recognition_state_not_warning_authorized"
        : input.pending_warning_state_reason;
    return false;
  }
  if (!input.pending_pose_physically_plausible) {
    *reason = input.pending_warning_state_reason.empty()
        ? "pending_pose_physically_implausible"
        : input.pending_warning_state_reason;
    return false;
  }
  if (!input.pending_warning_query_allowed) {
    *reason = "pending_warning_query_not_authorized";
    return false;
  }
  if (config.pending_warning_promotion_policy ==
      PendingWarningPromotionPolicy::DISABLED) {
    *reason = "policy_disabled";
    return false;
  }
  if (!pendingSourceCanCarryIdentity(input.pending_envelope_source)) {
    *reason = "envelope_source_not_identity_backed";
    return false;
  }
  if (!pendingPoseCanCarryIdentity(input.pending_pose_source)) {
    *reason = "pose_source_not_identity_backed";
    return false;
  }
  if (!input.pending_self_evidence_valid) {
    *reason = "cargo_self_evidence_missing";
    return false;
  }
  if (!input.pending_external_separation_valid) {
    *reason = "cargo_external_separation_unresolved";
    return false;
  }
  if (!input.pending_external_obstacle_authorized ||
      input.pending_external_obstacle_track_id == 0U) {
    *reason = "external_obstacle_identity_missing";
    return false;
  }
  if (input.pending_external_obstacle_confirmations <
      config.pending_minimum_obstacle_confirmations) {
    *reason = "external_obstacle_confirmation_pending";
    return false;
  }
  if (!input.pending_external_provenance_valid) {
    *reason = "external_obstacle_provenance_invalid";
    return false;
  }
  if (config.pending_warning_promotion_policy ==
      PendingWarningPromotionPolicy::LEGACY_ANY_PENDING) {
    // Legacy may relax confirmation/shape thresholds, but it must never
    // bypass the operational identity invariants. An official 17/18 without
    // a concrete external track is unverifiable.
    *reason = "legacy_identity_and_external_track_confirmed";
    return true;
  }
  if (!input.pending_external_geometry_valid) {
    *reason = "external_obstacle_geometry_invalid";
    return false;
  }
  if (!std::isfinite(input.pending_authority_confidence) ||
      input.pending_authority_confidence <
          config.pending_minimum_authority_confidence) {
    *reason = "pending_authority_confidence_low";
    return false;
  }
  *reason = "identity_and_external_obstacle_confirmed";
  return true;
}

bool authorizePendingStaticWarning(
    const CargoAvoidanceFusionInput& input,
    const CargoAvoidanceFusionConfig& config,
    std::string* reason) {
  if (!config.allow_static_only_pending_warning) {
    *reason = "static_pending_warning_policy_disabled";
    return false;
  }
  if (!input.pending_recognition_state_allows_warning) {
    *reason = input.pending_warning_state_reason.empty()
        ? "recognition_state_not_warning_authorized"
        : input.pending_warning_state_reason;
    return false;
  }
  if (!input.pending_pose_physically_plausible) {
    *reason = input.pending_warning_state_reason.empty()
        ? "pending_pose_physically_implausible"
        : input.pending_warning_state_reason;
    return false;
  }
  if (!input.pending_warning_query_allowed) {
    *reason = "pending_warning_query_not_authorized";
    return false;
  }
  if (config.pending_warning_promotion_policy ==
      PendingWarningPromotionPolicy::DISABLED) {
    *reason = "policy_disabled";
    return false;
  }
  if (!pendingSourceCanCarryIdentity(input.pending_envelope_source)) {
    *reason = "envelope_source_not_identity_backed";
    return false;
  }
  if (!pendingPoseCanCarryIdentity(input.pending_pose_source)) {
    *reason = "pose_source_not_identity_backed";
    return false;
  }
  if (!input.pending_self_evidence_valid) {
    *reason = "cargo_self_evidence_missing";
    return false;
  }
  if (!std::isfinite(input.pending_authority_confidence) ||
      input.pending_authority_confidence <
          config.pending_minimum_authority_confidence) {
    *reason = "pending_cargo_identity_confidence_low";
    return false;
  }
  if (!input.pending_static_obstacle_authorized ||
      input.pending_static_obstacle_id == 0U) {
    *reason = "static_obstacle_identity_missing";
    return false;
  }
  if (input.pending_static_obstacle_confirmations <
      config.pending_minimum_obstacle_confirmations) {
    *reason = "static_obstacle_confirmation_pending";
    return false;
  }
  if (!input.pending_static_provenance_valid) {
    *reason = "static_obstacle_provenance_invalid";
    return false;
  }
  if (!std::isfinite(input.pending_static_authority_confidence) ||
      input.pending_static_authority_confidence <
          config.pending_minimum_authority_confidence) {
    *reason = "static_obstacle_authority_confidence_low";
    return false;
  }
  *reason = "identity_and_static_obstacle_confirmed";
  return true;
}

}  // namespace

const char* pendingWarningPromotionPolicyName(
    PendingWarningPromotionPolicy policy) noexcept {
  switch (policy) {
    case PendingWarningPromotionPolicy::DISABLED:
      return "DISABLED";
    case PendingWarningPromotionPolicy::EVIDENCE_BACKED_ONLY:
      return "EVIDENCE_BACKED_ONLY";
    case PendingWarningPromotionPolicy::LEGACY_ANY_PENDING:
      return "LEGACY_ANY_PENDING";
  }
  return "UNKNOWN";
}

CargoAvoidanceFusionResult fuseCargoAvoidanceRisk(
    const CargoAvoidanceFusionInput& input,
    const CargoAvoidanceFusionConfig& config) {
  CargoAvoidanceFusionResult result;
  if (!input.localization_valid) {
    result.official_code = kLocalizationInvalid;
    result.reason = "localization_invalid";
    return result;
  }

  const bool formal_cargo = input.formal_cargo_geometry_valid &&
      input.formal_cargo_bottom_valid;
  const StaticEvidenceAuthorization static_authorization =
      authorizeStaticEvidence(input.static_authority);
  const bool static_identity_valid = input.static_session_manifest_valid &&
      input.static_session_hash_valid && input.static_session_uuid_valid;
  const bool static_risk_contract = static_identity_valid &&
      input.static_risk_contract_valid &&
      static_authorization.official_static_risk_authorized;
  const bool static_clear_contract = static_identity_valid &&
      input.static_clear_contract_valid &&
      static_authorization.official_clear_authorized;
  const bool live_reliable = input.live.available && input.live.reliable;
  const bool static_reliable = static_risk_contract &&
      input.static_map.available && input.static_map.reliable;
  const bool unresolved_embedded_live_hazard =
      live_reliable && input.live.hazard &&
      warningCode(input.live.warning_code) &&
      !input.live_obstacle_origin_resolved;

  result.risk_live = live_reliable && input.live_obstacle_origin_resolved &&
      input.live.hazard &&
      warningCode(input.live.warning_code);
  const bool static_hazard_observed = static_reliable &&
      input.static_map.hazard && warningCode(input.static_map.warning_code);
  result.risk_static = static_hazard_observed &&
      input.static_hazard_track_confirmed;
  if (live_reliable && input.live_obstacle_origin_resolved) {
    combineMetric(input.live.distance_m, &result.distance_m);
    combineMetric(input.live.clearance_m, &result.clearance_m);
  }
  if (static_reliable) {
    combineMetric(input.static_map.distance_m, &result.distance_m);
    combineMetric(input.static_map.clearance_m, &result.clearance_m);
  }

  const bool live_clear_observed =
      live_reliable && input.live_obstacle_origin_resolved &&
      !result.risk_live &&
      input.live.coverage >= config.minimum_live_coverage_for_clear;
  result.map_live_conflict = result.risk_static && live_clear_observed;

  const bool warning_candidate =
      result.risk_live || result.risk_static ||
      (input.warning_candidate_present &&
       warningCode(input.warning_candidate_code));

  // ========== 修复 ==========
  // 旧行为：anomaly_review_candidate 在任何标准告警之前无条件返回 29，
  // 即使同一帧已存在可信货物包络 + 已确认障碍轨迹 + 有远场历史 + 距离进入 3m/5m。
  //
  // 新行为：先计算所有 FORMAL/POSITIVE_ONLY 17/18。
  // 只在没有任何正式 17/18 可以输出时，才允许进入 29。
  // 最终优先级：31/35 等硬故障 → 已授权 17/18 → 29 → 33/34 → Formal CLEAR 14。

  // 不再在此处提前返回 29。anomaly_review 推迟到标准告警评估之后。
  if (static_hazard_observed && !input.static_hazard_track_confirmed &&
      !result.risk_live && !input.warning_candidate_present) {
    result.official_code = kObstacleInvalid;
    result.reason = "static_hazard_track_confirmation_pending";
    result.provisional_status = "TRACK_CONFIRMATION_PENDING";
    result.pending_authority_reason = result.reason;
    return result;
  }
  // ========== 修复 ==========
  // 旧行为：warning_motion_authorized 作为所有标准告警的总开关。
  // 天车静止/运动方向未知/MotionGate 切换时，即使旁边存在已确认固定障碍也被 33 阻断。
  //
  // 新行为：运动方向只用于更新 approach/far-field history。
  // 静止或方向未知时使用径向距离 + 已有远场轨迹/静态历史。
  // 已确认障碍仍允许输出 17/18。
  if (warning_candidate &&
      !input.warning_motion_authorized) {
    // 不立即返回 33。运动方向无效时仍评估后续标准告警路径。
    // 如果后续 formal/positive-only 处理确认了 obstacle，则正常输出 17/18。
    // 只有后续也无法确认时才返回 33。
    result.motion_not_authoritative = true;
  }

  const bool live_level1_risk =
      (result.risk_live && input.live.warning_code == kNear3m) ||
      (input.warning_candidate_present &&
       input.warning_candidate_code == kNear3m);
  const bool static_level1_risk =
      result.risk_static && input.static_map.warning_code == kNear3m;
  const bool live_level1_without_history = live_level1_risk &&
      !input.live_near_field_history_authorized;
  const bool static_level1_without_history = static_level1_risk &&
      !input.static_near_field_history_authorized;
  std::string independent_static_reason;
  const bool static_level1_independently_proven =
      static_level1_without_history && result.risk_static &&
      (formal_cargo
           ? (static_risk_contract && input.static_hazard_track_confirmed)
           : authorizePendingStaticWarning(
                 input, config, &independent_static_reason));
  // A live obstacle that first appears inside 3 m remains abnormal (29).
  // A mature, map-identity-bound static obstacle is different: its physical
  // history predates this approach and is independent evidence, so requiring
  // a newly observed far-field approach would suppress real fixed hazards.
  if (live_level1_without_history ||
      (static_level1_without_history &&
       !static_level1_independently_proven)) {
    result.official_valid = true;
    result.official_code = kAnomalyReview;
    result.anomaly_review = true;
    result.anomaly_review_live = live_level1_without_history;
    result.anomaly_review_static = static_level1_without_history &&
        !static_level1_independently_proven;
    result.reason = "review_level1_without_approach_history";
    result.provisional_status = "REVIEW_REQUIRED";
    result.pending_authority_reason =
        "level1_without_approach_history";
    return result;
  }

  if (!formal_cargo) {
    result.official_code = kCargoInvalid;
    result.reason = "cargo_recognition_or_geometry_invalid";
    if (!input.pending_envelope_valid) {
      result.provisional_status = "UNKNOWN";
      return result;
    }
    if (!input.pending_recognition_state_allows_warning) {
      result.provisional_status = "WARNING_AUTHORITY_REVOKED";
      result.pending_authority_reason =
          input.pending_warning_state_reason.empty()
          ? "recognition_state_not_warning_authorized"
          : input.pending_warning_state_reason;
      result.reason = "pending_warning_state_revoked:" +
          result.pending_authority_reason;
      return result;
    }
    if (!input.pending_pose_physically_plausible) {
      result.provisional_status = "POSE_REJECTED";
      result.pending_authority_reason =
          input.pending_warning_state_reason.empty()
          ? "pending_pose_physically_implausible"
          : input.pending_warning_state_reason;
      result.reason = "pending_hazard_not_authorized:" +
          result.pending_authority_reason;
      return result;
    }
    if (!input.pending_warning_query_allowed) {
      result.provisional_status = "QUERY_NOT_AUTHORIZED";
      result.pending_authority_reason =
          "pending_warning_query_not_authorized";
      result.reason = "pending_hazard_not_authorized:" +
          result.pending_authority_reason;
      return result;
    }
    if (unresolved_embedded_live_hazard && !result.risk_static) {
      result.provisional_status = "SOURCE_UNRESOLVED";
      result.pending_authority_reason =
          "embedded_obstacle_origin_unresolved";
      result.reason = "pending_hazard_not_authorized:" +
          result.pending_authority_reason;
      return result;
    }
    if (result.risk_live || result.risk_static) {
      const std::int32_t provisional = moreSevere(
          result.risk_live ? input.live.warning_code : 0,
          result.risk_static ? input.static_map.warning_code : 0);
      result.provisional_status = provisional == kNear3m
          ? "NEAR_3M" : "NEAR_5M";
      std::string live_reason = "live_hazard_not_present";
      std::string static_reason = "static_hazard_not_present";
      result.pending_live_warning_authorized = result.risk_live &&
          authorizePendingWarning(input, config, &live_reason);
      result.pending_static_warning_authorized = result.risk_static &&
          authorizePendingStaticWarning(input, config, &static_reason);
      result.pending_warning_authorized =
          result.pending_live_warning_authorized ||
          result.pending_static_warning_authorized;
      if (result.pending_warning_authorized) {
        result.official_code = moreSevere(
            result.pending_live_warning_authorized
                ? input.live.warning_code : 0,
            result.pending_static_warning_authorized
                ? input.static_map.warning_code : 0);
        result.official_valid = true;
        if (result.pending_live_warning_authorized &&
            result.pending_static_warning_authorized) {
          result.pending_authority_reason =
              "live_and_static_pending_hazard_confirmed";
          result.reason = "pending_live_and_static_warning_authorized";
        } else if (result.pending_live_warning_authorized) {
          result.pending_authority_reason = live_reason;
          result.reason = "pending_live_warning_authorized";
        } else {
          result.pending_authority_reason = static_reason;
          result.reason = "pending_static_warning_authorized";
        }
      } else {
        result.pending_authority_reason = result.risk_live
            ? live_reason : static_reason;
        if (result.risk_live && result.risk_static) {
          result.pending_authority_reason =
              "live=" + live_reason + ";static=" + static_reason;
        }
        result.reason = "pending_hazard_not_authorized:" +
            result.pending_authority_reason;
      }
    } else {
      // A pending envelope can never grant clear.
      result.provisional_status = "CLEAR_NOT_AUTHORIZED";
      result.pending_authority_reason = "no_positive_hazard";
    }
    return result;
  }

  if (result.risk_live || result.risk_static) {
    result.official_code = moreSevere(
        result.risk_live ? input.live.warning_code : 0,
        result.risk_static ? input.static_map.warning_code : 0);
    result.official_valid = true;
    result.reason = result.map_live_conflict
        ? "MAP_LIVE_CONFLICT_static_hazard_retained"
        : (result.risk_live && result.risk_static
               ? "live_and_static_hazard"
               : (result.risk_live ? "live_hazard" : "static_hazard"));
    return result;
  }

  // ========== 修复 ==========
  // Anomaly review (29) 现在只能在全量标准告警评估之后进入。
  // 29 只能补充"无法成为标准 17/18 的异常候选"，不能覆盖已确认的碰撞风险。
  if (input.anomaly_review_candidate) {
    result.official_valid = true;
    result.official_code = kAnomalyReview;
    result.anomaly_review = true;
    result.anomaly_review_live = input.anomaly_review_live;
    result.anomaly_review_static = input.anomaly_review_static;
    result.reason = input.anomaly_review_reason.empty()
        ? "avoidance_anomaly_review_required"
        : input.anomaly_review_reason;
    result.provisional_status = "REVIEW_REQUIRED";
    result.pending_authority_reason = result.reason;
    result.distance_m = input.anomaly_review_distance_m;
    result.clearance_m = input.anomaly_review_clearance_m;
    return result;
  }

  if (unresolved_embedded_live_hazard) {
    result.official_code = kObstacleInvalid;
    result.reason = "embedded_obstacle_origin_unresolved";
    return result;
  }

  if (!input.formal_clear_authorized) {
    result.official_code = kCargoInvalid;
    result.reason = "formal_cargo_clear_not_authorized";
    return result;
  }

  const bool static_clear_reliable = static_clear_contract &&
      input.static_map.available && input.static_map.reliable &&
      !result.risk_static;
  if (!live_clear_observed || !static_clear_reliable) {
    result.official_code = kObstacleInvalid;
    result.reason = !live_clear_observed
        ? "live_roi_not_reliable_for_clear"
        : "static_session_not_reliable_for_clear";
    return result;
  }

  result.official_valid = true;
  result.official_code = kClear;
  result.reason = "live_and_static_clear";
  return result;
}

}  // namespace ndt_slam
