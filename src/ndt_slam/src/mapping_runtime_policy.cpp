#include "ndt_slam/mapping_runtime_policy.hpp"

#include <algorithm>
#include <cmath>

namespace ndt_slam {

const char* mappingAuthorityStateName(MappingAuthorityState state) noexcept {
  switch (state) {
    case MappingAuthorityState::ACTIVE: return "ACTIVE";
    case MappingAuthorityState::PAUSED_QUALITY: return "PAUSED_QUALITY";
    case MappingAuthorityState::PAUSED_IO: return "PAUSED_IO";
    case MappingAuthorityState::FAIL_CLOSED: return "FAIL_CLOSED";
  }
  return "FAIL_CLOSED";
}

MappingRuntimePolicy::MappingRuntimePolicy(
    const MappingRuntimePolicyConfig& config) {
  configure(config);
}

void MappingRuntimePolicy::configure(
    const MappingRuntimePolicyConfig& config) {
  config_ = config;
  config_.confirmed_hard_failure_frames = std::max(
      2, config_.confirmed_hard_failure_frames);
  config_.confirmed_hard_failure_duration_sec = std::max(
      0.0, config_.confirmed_hard_failure_duration_sec);
  resetForNewSegment();
}

void MappingRuntimePolicy::resetForNewSegment() {
  decision_ = MappingRuntimeDecision{};
  hard_failure_started_sec_ = 0.0;
  previous_stamp_sec_ = 0.0;
}

MappingRuntimeDecision MappingRuntimePolicy::latchFailClosed(
    const std::string& reason) {
  const MappingAuthorityState previous = decision_.state;
  decision_.state = MappingAuthorityState::FAIL_CLOSED;
  decision_.trusted_writes_allowed = false;
  decision_.formal_warning_authority_allowed = false;
  decision_.fail_closed_latched = true;
  decision_.transition = previous != MappingAuthorityState::FAIL_CLOSED;
  decision_.reason = reason.empty() ? "explicit_fail_closed" : reason;
  return decision_;
}

MappingRuntimeDecision MappingRuntimePolicy::update(
    const MappingRuntimeEvidence& evidence) {
  if (decision_.fail_closed_latched) {
    decision_.transition = false;
    return decision_;
  }

  const MappingAuthorityState previous = decision_.state;
  const bool detected_severe_rollback =
      previous_stamp_sec_ > 0.0 && std::isfinite(evidence.stamp_sec) &&
      evidence.stamp_sec + 0.5 < previous_stamp_sec_;
  const bool immediate_failure = evidence.nonfinite ||
      evidence.severe_source_time_rollback || detected_severe_rollback ||
      evidence.physical_impossibility || evidence.identity_corruption;
  const bool independent_quality_failure =
      evidence.hard_innovation_reject || evidence.geometry_invalid ||
      evidence.observability_invalid;
  const bool hard_failure_sample = !evidence.ndt_converged ||
      evidence.hard_innovation_reject ||
      (evidence.high_fitness && independent_quality_failure);

  if (hard_failure_sample) {
    if (decision_.consecutive_hard_failure_frames == 0) {
      hard_failure_started_sec_ = evidence.stamp_sec;
    }
    ++decision_.consecutive_hard_failure_frames;
    decision_.hard_failure_duration_sec =
        std::isfinite(evidence.stamp_sec) &&
                evidence.stamp_sec >= hard_failure_started_sec_
            ? evidence.stamp_sec - hard_failure_started_sec_
            : 0.0;
  } else {
    decision_.consecutive_hard_failure_frames = 0;
    decision_.hard_failure_duration_sec = 0.0;
    hard_failure_started_sec_ = 0.0;
  }

  const bool confirmed_failure = hard_failure_sample &&
      decision_.consecutive_hard_failure_frames >=
          config_.confirmed_hard_failure_frames &&
      decision_.hard_failure_duration_sec + 1.0e-4 >=
          config_.confirmed_hard_failure_duration_sec;
  if (immediate_failure || confirmed_failure) {
    decision_.state = MappingAuthorityState::FAIL_CLOSED;
    decision_.fail_closed_latched = true;
    decision_.reason = immediate_failure
        ? (evidence.nonfinite
               ? "nonfinite"
               : (evidence.severe_source_time_rollback ||
                          detected_severe_rollback
                      ? "severe_source_time_rollback"
                      : (evidence.physical_impossibility
                             ? "physical_impossibility"
                             : "identity_corruption")))
        : "confirmed_continuous_localization_failure";
  } else if (evidence.io_paused) {
    decision_.state = MappingAuthorityState::PAUSED_IO;
    decision_.reason = "archive_io_not_ready";
  } else if (hard_failure_sample || evidence.high_fitness ||
             evidence.observability_invalid || evidence.geometry_invalid) {
    decision_.state = MappingAuthorityState::PAUSED_QUALITY;
    decision_.reason = evidence.high_fitness &&
            !independent_quality_failure
        ? "persistent_high_fitness"
        : "transient_localization_degradation";
  } else {
    decision_.state = MappingAuthorityState::ACTIVE;
    decision_.reason = "healthy";
  }
  decision_.trusted_writes_allowed =
      decision_.state == MappingAuthorityState::ACTIVE;
  decision_.formal_warning_authority_allowed =
      decision_.state != MappingAuthorityState::FAIL_CLOSED;
  decision_.transition = decision_.state != previous;
  if (std::isfinite(evidence.stamp_sec) && evidence.stamp_sec > 0.0) {
    previous_stamp_sec_ = evidence.stamp_sec;
  }
  return decision_;
}

}  // namespace ndt_slam
