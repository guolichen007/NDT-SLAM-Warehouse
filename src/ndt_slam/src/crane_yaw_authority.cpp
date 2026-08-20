#include "ndt_slam/crane_yaw_authority.hpp"

#include <algorithm>
#include <cmath>

namespace ndt_slam {

CraneYawAuthority::CraneYawAuthority(
    const CraneYawAuthorityConfig& config) {
  configure(config);
}

void CraneYawAuthority::configure(const CraneYawAuthorityConfig& config) {
  config_ = config;
  decision_ = CraneYawAuthorityDecision{};
  raw_unwrap_initialized_ = false;
  authority_unwrap_initialized_ = false;
  fallback_sin_sum_ = 0.0;
  fallback_cos_sum_ = 0.0;
  fallback_reliable_frames_ = 0U;
  fallback_established_ = false;
  fallback_yaw_rad_ = 0.0;
  conflict_evidence_frames_ = 0U;
}

double CraneYawAuthority::shortestAngle(double angle_rad) {
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

double CraneYawAuthority::unwrap(double wrapped, bool* initialized,
                                 double* previous_wrapped,
                                 double* unwrapped) {
  if (!std::isfinite(wrapped)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (!*initialized) {
    *initialized = true;
    *previous_wrapped = wrapped;
    *unwrapped = wrapped;
    return *unwrapped;
  }
  *unwrapped += shortestAngle(wrapped - *previous_wrapped);
  *previous_wrapped = wrapped;
  return *unwrapped;
}

CraneYawAuthorityDecision CraneYawAuthority::observe(
    const CraneYawEvidence& evidence) {
  CraneYawAuthorityDecision next;
  next.product_application_allowed = false;

  if (config_.apply_to_runtime_pose) {
    next.state = CraneYawAuthorityState::INVALID;
    next.reason = "PRODUCT_MODE_NOT_IMPLEMENTED_IN_SHADOW_BUILD";
    decision_ = next;
    return decision_;
  }
  if (!config_.enabled) {
    next.state = CraneYawAuthorityState::UNCONFIGURED;
    next.reason = "disabled";
    decision_ = next;
    return decision_;
  }
  if (config_.reference_source != "CONFIG") {
    next.state = CraneYawAuthorityState::INVALID;
    next.reason = "unsupported_reference_source";
    decision_ = next;
    return decision_;
  }
  if (config_.map_frame_convention_id.empty() ||
      config_.map_frame_convention_description.empty()) {
    next.state = CraneYawAuthorityState::UNCONFIGURED;
    next.reason = "MAP_FRAME_CONVENTION_REQUIRED";
    decision_ = next;
    return decision_;
  }

  double authority_yaw = config_.configured_base_yaw_in_map_rad;
  if (!std::isfinite(authority_yaw)) {
    if (!config_.allow_first_reliable_fallback) {
      next.state = CraneYawAuthorityState::UNCONFIGURED;
      next.reason = "SITE_YAW_CALIBRATION_REQUIRED";
      decision_ = next;
      return decision_;
    }
    if (config_.fallback_required_reliable_frames < 3U) {
      next.state = CraneYawAuthorityState::INVALID;
      next.reason = "fallback_requires_at_least_three_reliable_frames";
      decision_ = next;
      return decision_;
    }
    if (!fallback_established_ && evidence.yaw_observability_strong &&
        std::isfinite(evidence.raw_ndt_yaw_rad)) {
      fallback_sin_sum_ += std::sin(evidence.raw_ndt_yaw_rad);
      fallback_cos_sum_ += std::cos(evidence.raw_ndt_yaw_rad);
      ++fallback_reliable_frames_;
      if (fallback_reliable_frames_ >=
          config_.fallback_required_reliable_frames) {
        fallback_yaw_rad_ = std::atan2(fallback_sin_sum_, fallback_cos_sum_);
        fallback_established_ = true;
      }
    }
    if (!fallback_established_) {
      next.state = CraneYawAuthorityState::FALLBACK_BOOTSTRAP;
      next.reason = "waiting_for_multiple_reliable_frames";
      decision_ = next;
      return decision_;
    }
    authority_yaw = fallback_yaw_rad_;
  } else {
    next.configured = true;
  }

  authority_yaw = shortestAngle(authority_yaw);
  next.proposal_valid = true;
  next.authoritative_yaw_rad = authority_yaw;
  next.authoritative_yaw_unwrapped_rad = unwrap(
      authority_yaw, &authority_unwrap_initialized_,
      &previous_authority_wrapped_, &authority_unwrapped_);
  if (std::isfinite(evidence.raw_ndt_yaw_rad)) {
    next.raw_innovation_valid = true;
    next.raw_minus_authoritative_yaw_rad = shortestAngle(
        evidence.raw_ndt_yaw_rad - authority_yaw);
    next.raw_yaw_unwrapped_rad = unwrap(
        shortestAngle(evidence.raw_ndt_yaw_rad), &raw_unwrap_initialized_,
        &previous_raw_wrapped_, &raw_unwrapped_);
  }

  const bool raw_threshold_configured =
      std::isfinite(config_.raw_yaw_threshold_rad) &&
      config_.raw_yaw_threshold_rad > 0.0;
  const bool fitness_threshold_configured =
      std::isfinite(config_.rail_fitness_delta_threshold);
  const bool translation_threshold_configured =
      std::isfinite(config_.rail_translation_delta_threshold_m) &&
      config_.rail_translation_delta_threshold_m >= 0.0;
  const bool raw_strong_disagreement = raw_threshold_configured &&
      next.raw_innovation_valid && evidence.yaw_observability_strong &&
      std::abs(next.raw_minus_authoritative_yaw_rad) >
          config_.raw_yaw_threshold_rad;
  const bool rail_fitness_bad = fitness_threshold_configured &&
      std::isfinite(evidence.rail_fitness_delta) &&
      evidence.rail_fitness_delta > config_.rail_fitness_delta_threshold;
  const bool rail_translation_bad = translation_threshold_configured &&
      std::isfinite(evidence.rail_translation_delta_m) &&
      evidence.rail_translation_delta_m >
          config_.rail_translation_delta_threshold_m;
  const bool rail_independent_degradation =
      !evidence.rail_registration_valid || rail_fitness_bad ||
      rail_translation_bad;
  // A failed fixed-yaw registration is independent evidence on its own.
  // Raw/config yaw disagreement contributes only when accompanied by rail
  // degradation; raw yaw innovation alone can never block mapping.
  const bool composite_evidence = !evidence.rail_registration_valid ||
      rail_fitness_bad || rail_translation_bad ||
      (raw_strong_disagreement && rail_independent_degradation);

  if (composite_evidence) {
    ++conflict_evidence_frames_;
  } else {
    conflict_evidence_frames_ = 0U;
  }
  next.conflict_evidence_frames = conflict_evidence_frames_;
  const std::size_t required = config_.required_consecutive_frames;
  next.composite_conflict_evidence = required > 0U &&
      conflict_evidence_frames_ >= required;
  if (next.composite_conflict_evidence) {
    next.state = CraneYawAuthorityState::PHYSICAL_CONFLICT_EVIDENCE;
    next.reason = "composite_rail_and_yaw_evidence";
  } else {
    next.state = CraneYawAuthorityState::CONFIG_HOLD;
    next.reason = evidence.yaw_observability_strong
        ? "config_hold_strong_observability"
        : "config_hold_weak_observability_normal_operation";
  }
  decision_ = next;
  return decision_;
}

const char* CraneYawAuthority::stateName(CraneYawAuthorityState state) {
  switch (state) {
    case CraneYawAuthorityState::UNCONFIGURED:
      return "UNCONFIGURED";
    case CraneYawAuthorityState::CONFIG_HOLD:
      return "CONFIG_HOLD";
    case CraneYawAuthorityState::FALLBACK_BOOTSTRAP:
      return "FALLBACK_BOOTSTRAP";
    case CraneYawAuthorityState::PHYSICAL_CONFLICT_EVIDENCE:
      return "PHYSICAL_CONFLICT_EVIDENCE";
    case CraneYawAuthorityState::INVALID:
      return "INVALID";
  }
  return "INVALID";
}

}  // namespace ndt_slam
