#include "ndt_slam/rail_localization_authority.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace ndt_slam {
namespace {

double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle <= -M_PI) angle += 2.0 * M_PI;
  return angle;
}

std::uint64_t fnv1a64(const std::string& text) noexcept {
  std::uint64_t hash = 1469598103934665603ULL;
  for (unsigned char byte : text) {
    hash ^= static_cast<std::uint64_t>(byte);
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::string canonicalReference(const RailYawReference& reference) {
  std::ostringstream stream;
  stream << "schema=" << reference.schema_version
         << ";verified=" << (reference.verified ? 1 : 0)
         << ";yaw=" << std::setprecision(17)
         << normalizeAngle(reference.rail_yaw_in_map_rad)
         << ";source=" << yawReferenceSourceName(reference.source)
         << ";map_frame_uuid=" << reference.map_frame_uuid
         << ";map_frame_id=" << reference.map_frame_id
         << ";base_frame_id=" << reference.base_frame_id
         << ";convention=" << reference.map_frame_convention_id
         << ";sensor_rig=" << reference.sensor_rig_calibration_id
         << ";reference_uuid=" << reference.reference_uuid;
  return stream.str();
}

std::string hex64(std::uint64_t value) {
  std::ostringstream stream;
  stream << std::hex << std::setfill('0') << std::setw(16) << value;
  return stream.str();
}

}  // namespace

const char* yawAuthorityModeName(YawAuthorityMode mode) noexcept {
  switch (mode) {
    case YawAuthorityMode::LEGACY: return "LEGACY";
    case YawAuthorityMode::SHADOW: return "SHADOW";
    case YawAuthorityMode::RAIL_AUTHORITY: return "RAIL_AUTHORITY";
  }
  return "INVALID";
}

const char* yawReferenceSourceName(YawReferenceSource source) noexcept {
  switch (source) {
    case YawReferenceSource::NEW_MAP_BOOTSTRAP_REFERENCE:
      return "NEW_MAP_BOOTSTRAP_REFERENCE";
    case YawReferenceSource::CONFIG_SITE_REFERENCE:
      return "CONFIG_SITE_REFERENCE";
    case YawReferenceSource::VERIFIED_MAP_SESSION:
      return "VERIFIED_MAP_SESSION";
  }
  return "INVALID";
}

bool yawReferenceSourceFromName(
    const std::string& name, YawReferenceSource* source) noexcept {
  if (!source) return false;
  if (name == "NEW_MAP_BOOTSTRAP_REFERENCE") {
    *source = YawReferenceSource::NEW_MAP_BOOTSTRAP_REFERENCE;
    return true;
  }
  if (name == "CONFIG_SITE_REFERENCE") {
    *source = YawReferenceSource::CONFIG_SITE_REFERENCE;
    return true;
  }
  if (name == "VERIFIED_MAP_SESSION") {
    *source = YawReferenceSource::VERIFIED_MAP_SESSION;
    return true;
  }
  return false;
}

const char* yawAuthorityTransitionReasonName(
    YawAuthorityTransitionReason reason) noexcept {
  switch (reason) {
    case YawAuthorityTransitionReason::INITIALIZE_FRESH_MAP:
      return "INITIALIZE_FRESH_MAP";
    case YawAuthorityTransitionReason::LOAD_VERIFIED_SESSION:
      return "LOAD_VERIFIED_SESSION";
    case YawAuthorityTransitionReason::EXPLICIT_MAP_FRAME_MIGRATION:
      return "EXPLICIT_MAP_FRAME_MIGRATION";
    case YawAuthorityTransitionReason::RESET: return "RESET";
  }
  return "INVALID";
}

const char* localizationFailureClassName(
    LocalizationFailureClass failure) noexcept {
  switch (failure) {
    case LocalizationFailureClass::NONE: return "NONE";
    case LocalizationFailureClass::RECOVERABLE_TRACKING_DEGRADATION:
      return "RECOVERABLE_TRACKING_DEGRADATION";
    case LocalizationFailureClass::TEMPORARY_OBSERVABILITY_LOSS:
      return "TEMPORARY_OBSERVABILITY_LOSS";
    case LocalizationFailureClass::TARGET_DATA_UNAVAILABLE:
      return "TARGET_DATA_UNAVAILABLE";
    case LocalizationFailureClass::NONRECOVERABLE_REFERENCE_CONFIG:
      return "NONRECOVERABLE_REFERENCE_CONFIG";
    case LocalizationFailureClass::NONRECOVERABLE_MAP_IDENTITY:
      return "NONRECOVERABLE_MAP_IDENTITY";
    case LocalizationFailureClass::INTERNAL_CONTRACT_ERROR:
      return "INTERNAL_CONTRACT_ERROR";
  }
  return "INVALID";
}

const char* registrationTargetSourceName(
    RegistrationTargetSource source) noexcept {
  switch (source) {
    case RegistrationTargetSource::UNKNOWN: return "UNKNOWN";
    case RegistrationTargetSource::GLOBAL_MAP: return "GLOBAL_MAP";
    case RegistrationTargetSource::CROPPED_ACTIVE_MAP:
      return "CROPPED_ACTIVE_MAP";
    case RegistrationTargetSource::LOCALIZATION_MAP:
      return "LOCALIZATION_MAP";
    case RegistrationTargetSource::PERSISTENT_MAP:
      return "PERSISTENT_MAP";
  }
  return "INVALID";
}

std::string semanticYawReferenceHash(const RailYawReference& reference) {
  return hex64(fnv1a64(canonicalReference(reference)));
}

bool validateRailYawReference(const RailYawReference& reference,
                              std::string* reason) {
  if (reference.schema_version != 1U) {
    if (reason) *reason = "unsupported_yaw_reference_schema";
    return false;
  }
  if (!reference.verified) {
    if (reason) *reason = "yaw_reference_not_verified";
    return false;
  }
  if (!std::isfinite(reference.rail_yaw_in_map_rad)) {
    if (reason) *reason = "yaw_reference_nonfinite";
    return false;
  }
  if (reference.map_frame_uuid.empty() || reference.reference_uuid.empty()) {
    if (reason) *reason = "yaw_reference_identity_missing";
    return false;
  }
  if (reference.reference_hash != semanticYawReferenceHash(reference)) {
    if (reason) *reason = "yaw_reference_hash_mismatch";
    return false;
  }
  if (reason) *reason = "ok";
  return true;
}

bool RailYawAuthority::assignReference(
    const RailYawReference& reference,
    YawAuthorityTransitionReason reason) {
  std::string validation_reason;
  if (!validateRailYawReference(reference, &validation_reason)) return false;
  reference_ = reference;
  reference_.rail_yaw_in_map_rad = normalizeAngle(
      reference_.rail_yaw_in_map_rad);
  valid_ = true;
  ++generation_;
  if (generation_ == 0U) ++generation_;
  last_transition_reason_ = reason;
  return true;
}

bool RailYawAuthority::initialize(
    const RailYawReference& reference,
    YawAuthorityTransitionReason reason) {
  if (valid_) return false;
  if (reason != YawAuthorityTransitionReason::INITIALIZE_FRESH_MAP &&
      reason != YawAuthorityTransitionReason::LOAD_VERIFIED_SESSION) {
    return false;
  }
  return assignReference(reference, reason);
}

bool RailYawAuthority::explicitMapFrameMigration(
    const RailYawReference& reference) {
  if (!valid_) return false;
  return assignReference(
      reference, YawAuthorityTransitionReason::EXPLICIT_MAP_FRAME_MIGRATION);
}

void RailYawAuthority::resetForFrameSession() {
  valid_ = false;
  reference_ = RailYawReference{};
  ++generation_;
  if (generation_ == 0U) ++generation_;
  last_proposal_stamp_sec_ = 0.0;
  last_transition_reason_ = YawAuthorityTransitionReason::RESET;
}

void RailYawAuthority::observeProposalYaw(
    double raw_yaw_rad, double stamp_sec) noexcept {
  if (std::isfinite(raw_yaw_rad)) {
    last_proposal_yaw_rad_ = normalizeAngle(raw_yaw_rad);
  }
  if (std::isfinite(stamp_sec) && stamp_sec > last_proposal_stamp_sec_) {
    last_proposal_stamp_sec_ = stamp_sec;
  }
}

void RailYawAuthority::observeRelocalizationProposalYaw(
    double raw_yaw_rad, double stamp_sec) noexcept {
  observeProposalYaw(raw_yaw_rad, stamp_sec);
}

void RailYawAuthority::handleTimestampRollback(double stamp_sec) noexcept {
  last_proposal_stamp_sec_ = std::isfinite(stamp_sec)
      ? stamp_sec : 0.0;
}

bool YawAuthorityModeLatch::initialize(
    YawAuthorityMode mode, std::uint64_t frame_session_id) {
  if (frame_session_id == 0U) return false;
  if (!initialized_) {
    initialized_ = true;
    mode_ = mode;
    frame_session_id_ = frame_session_id;
    return true;
  }
  if (frame_session_id == frame_session_id_) return mode == mode_;
  mode_ = mode;
  frame_session_id_ = frame_session_id;
  return true;
}

std::uint64_t makeRegistrationTargetSnapshotId(
    const RegistrationTargetIdentityInput& input) noexcept {
  std::ostringstream stream;
  stream << registrationTargetSourceName(input.source) << ';'
         << input.content_version << ';'
         << input.map_rebuild_generation << ';'
         << input.map_frame_uuid << ';'
         << input.crop_identity;
  std::uint64_t id = fnv1a64(stream.str());
  return id == 0U ? 1U : id;
}

LocalizationAuthorityHealth evaluateRailLocalizationHealth(
    const RailLocalizationHealthInput& input) {
  LocalizationAuthorityHealth decision;
  if (input.mode != YawAuthorityMode::RAIL_AUTHORITY) {
    decision.authoritative_frame_healthy = input.raw_ndt_proposal_healthy;
    decision.odom_continuity_valid = input.raw_ndt_proposal_healthy ||
        input.prediction_continuity_valid;
    decision.safety_localization_authorized =
        input.raw_ndt_proposal_healthy;
    decision.map_mutation_authorized = input.raw_ndt_proposal_healthy;
    decision.increment_relocalization_bad_frames =
        !input.raw_ndt_proposal_healthy;
    decision.request_relocalization = !input.raw_ndt_proposal_healthy;
    decision.watchdog_restart_authorized =
        !input.raw_ndt_proposal_healthy;
    decision.failure_class = input.raw_ndt_proposal_healthy
        ? LocalizationFailureClass::NONE
        : LocalizationFailureClass::RECOVERABLE_TRACKING_DEGRADATION;
    decision.reason = input.raw_ndt_proposal_healthy
        ? "legacy_registration_healthy"
        : "legacy_registration_degraded";
    return decision;
  }
  if (input.internal_contract_error) {
    decision.failure_class = LocalizationFailureClass::INTERNAL_CONTRACT_ERROR;
    decision.reason = "rail_internal_contract_error";
    return decision;
  }
  if (input.reference_failure_nonrecoverable || !input.yaw_reference_valid) {
    decision.failure_class =
        LocalizationFailureClass::NONRECOVERABLE_REFERENCE_CONFIG;
    decision.reason = "rail_yaw_reference_invalid";
    return decision;
  }
  if (input.map_identity_failure_nonrecoverable) {
    decision.failure_class =
        LocalizationFailureClass::NONRECOVERABLE_MAP_IDENTITY;
    decision.reason = "rail_map_identity_invalid";
    return decision;
  }
  if (!input.target_identity_valid) {
    decision.odom_continuity_valid = input.prediction_continuity_valid;
    decision.failure_class = LocalizationFailureClass::TARGET_DATA_UNAVAILABLE;
    decision.reason = "rail_target_unavailable";
    return decision;
  }
  if (!input.fixed_xy_valid || !input.rail_fitness_allow_measurement) {
    decision.odom_continuity_valid = input.prediction_continuity_valid;
    decision.increment_relocalization_bad_frames = true;
    decision.request_relocalization = true;
    decision.watchdog_restart_authorized = true;
    decision.failure_class =
        LocalizationFailureClass::RECOVERABLE_TRACKING_DEGRADATION;
    decision.reason = "rail_tracking_degraded";
    return decision;
  }
  decision.odom_continuity_valid = true;
  if (!input.ekf_measurement_accepted ||
      !input.rail_fitness_baseline_ready) {
    decision.failure_class =
        LocalizationFailureClass::TEMPORARY_OBSERVABILITY_LOSS;
    decision.reason = !input.rail_fitness_baseline_ready
        ? "rail_fitness_baseline_warming"
        : "rail_ekf_measurement_temporarily_rejected";
    return decision;
  }
  decision.authoritative_frame_healthy = true;
  decision.safety_localization_authorized = true;
  decision.map_mutation_authorized = true;
  decision.failure_class = LocalizationFailureClass::NONE;
  decision.reason = "rail_localization_authority_healthy";
  return decision;
}

}  // namespace ndt_slam
