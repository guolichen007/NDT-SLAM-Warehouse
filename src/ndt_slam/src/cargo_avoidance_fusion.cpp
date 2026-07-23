#include "ndt_slam/cargo_avoidance_fusion.hpp"
#include "ndt_slam/static_evidence_authorization.hpp"

#include <algorithm>
#include <cmath>

namespace ndt_slam {
namespace {

constexpr std::int32_t kClear = 14;
constexpr std::int32_t kNear3m = 17;
constexpr std::int32_t kNear5m = 18;
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
  if (config.pending_warning_promotion_policy ==
      PendingWarningPromotionPolicy::DISABLED) {
    *reason = "policy_disabled";
    return false;
  }
  if (config.pending_warning_promotion_policy ==
      PendingWarningPromotionPolicy::LEGACY_ANY_PENDING) {
    *reason = "explicit_legacy_any_pending";
    return true;
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

  result.risk_live = live_reliable && input.live.hazard &&
      warningCode(input.live.warning_code);
  result.risk_static = static_reliable && input.static_map.hazard &&
      warningCode(input.static_map.warning_code);
  if (live_reliable) {
    combineMetric(input.live.distance_m, &result.distance_m);
    combineMetric(input.live.clearance_m, &result.clearance_m);
  }
  if (static_reliable) {
    combineMetric(input.static_map.distance_m, &result.distance_m);
    combineMetric(input.static_map.clearance_m, &result.clearance_m);
  }

  const bool live_clear_observed = live_reliable && !result.risk_live &&
      input.live.coverage >= config.minimum_live_coverage_for_clear;
  result.map_live_conflict = result.risk_static && live_clear_observed;

  if (!formal_cargo) {
    result.official_code = kCargoInvalid;
    result.reason = "cargo_recognition_or_geometry_invalid";
    if (!input.pending_envelope_valid) {
      result.provisional_status = "UNKNOWN";
      return result;
    }
    if (result.risk_live || result.risk_static) {
      const std::int32_t provisional = moreSevere(
          result.risk_live ? input.live.warning_code : 0,
          result.risk_static ? input.static_map.warning_code : 0);
      result.provisional_status = provisional == kNear3m
          ? "NEAR_3M" : "NEAR_5M";
      result.pending_warning_authorized = authorizePendingWarning(
          input, config, &result.pending_authority_reason);
      if (result.pending_warning_authorized) {
        result.official_code = provisional;
        result.official_valid = true;
        result.reason = "pending_positive_warning_authorized";
      } else {
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
