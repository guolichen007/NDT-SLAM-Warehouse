#include "ndt_slam/pending_cargo_envelope.hpp"

#include <algorithm>
#include <cmath>

namespace ndt_slam {
namespace {

bool validConfig(const PendingCargoEnvelopeConfig& config) {
  return std::isfinite(config.configured_length_m) &&
      config.configured_length_m > 0.0F &&
      std::isfinite(config.configured_width_m) &&
      config.configured_width_m > 0.0F &&
      std::isfinite(config.configured_height_m) &&
      config.configured_height_m > 0.0F &&
      std::isfinite(config.configured_center_offset_z_m) &&
      std::isfinite(config.horizontal_margin_m) &&
      config.horizontal_margin_m >= 0.0F &&
      std::isfinite(config.vertical_margin_m) &&
      config.vertical_margin_m >= 0.0F &&
      std::isfinite(config.maximum_candidate_age_sec) &&
      config.maximum_candidate_age_sec >= 0.0 &&
      std::isfinite(config.maximum_retired_age_sec) &&
      config.maximum_retired_age_sec >= 0.0;
}

bool usable(const PendingCargoEnvelopeCandidate& candidate,
            double stamp_sec, double maximum_age_sec) {
  if (!candidate.valid || !candidate.center_base.allFinite() ||
      !std::isfinite(candidate.length_m) || candidate.length_m <= 0.0F ||
      !std::isfinite(candidate.width_m) || candidate.width_m <= 0.0F ||
      !std::isfinite(candidate.height_m) || candidate.height_m <= 0.0F ||
      !std::isfinite(candidate.yaw_base_rad) ||
      !std::isfinite(candidate.horizontal_uncertainty_m) ||
      !std::isfinite(candidate.vertical_uncertainty_m) ||
      candidate.horizontal_uncertainty_m < 0.0F ||
      candidate.vertical_uncertainty_m < 0.0F ||
      !std::isfinite(candidate.evidence_stamp_sec) ||
      candidate.evidence_stamp_sec <= 0.0) {
    return false;
  }
  const double age_sec = stamp_sec - candidate.evidence_stamp_sec;
  return age_sec >= -1.0e-3 && age_sec <= maximum_age_sec;
}

PendingCargoEnvelope fromCandidate(
    const PendingCargoEnvelopeCandidate& candidate,
    PendingCargoEnvelopeSource source,
    const PendingCargoEnvelopeConfig& config,
    const char* reason) {
  PendingCargoEnvelope result;
  const float expanded_half_length = 0.5F * candidate.length_m +
      candidate.horizontal_uncertainty_m + config.horizontal_margin_m;
  const float expanded_half_width = 0.5F * candidate.width_m +
      candidate.horizontal_uncertainty_m + config.horizontal_margin_m;
  const float expanded_half_height = 0.5F * candidate.height_m +
      candidate.vertical_uncertainty_m + config.vertical_margin_m;
  result.valid = true;
  result.source = source;
  result.center_base = candidate.center_base;
  result.length_m = 2.0F * expanded_half_length;
  result.width_m = 2.0F * expanded_half_width;
  result.height_m = 2.0F * expanded_half_height;
  result.yaw_base_rad = candidate.yaw_base_rad;
  result.horizontal_uncertainty_m =
      candidate.horizontal_uncertainty_m + config.horizontal_margin_m;
  result.vertical_uncertainty_m =
      candidate.vertical_uncertainty_m + config.vertical_margin_m;
  result.bottom_z_base = candidate.center_base.z() - expanded_half_height;
  result.top_z_base = candidate.center_base.z() + expanded_half_height;
  result.reason = reason;
  return result;
}

}  // namespace

const char* pendingCargoEnvelopeSourceName(
    PendingCargoEnvelopeSource source) noexcept {
  switch (source) {
    case PendingCargoEnvelopeSource::NONE: return "NONE";
    case PendingCargoEnvelopeSource::CURRENT_CANDIDATE:
      return "CURRENT_CANDIDATE";
    case PendingCargoEnvelopeSource::RETIRED_FORMAL_SHAPE:
      return "RETIRED_FORMAL_SHAPE";
    case PendingCargoEnvelopeSource::LIFT_ORIGIN_CANDIDATE:
      return "LIFT_ORIGIN_CANDIDATE";
    case PendingCargoEnvelopeSource::CONFIGURED_CONSERVATIVE:
      return "CONFIGURED_CONSERVATIVE";
  }
  return "NONE";
}

PendingCargoEnvelope buildPendingCargoEnvelope(
    const PendingCargoEnvelopeInput& input,
    const PendingCargoEnvelopeConfig& requested_config) {
  PendingCargoEnvelope result;
  const PendingCargoEnvelopeConfig config = validConfig(requested_config)
      ? requested_config : PendingCargoEnvelopeConfig{};
  if (!input.hook_loaded) {
    result.reason = "hook_not_loaded";
    return result;
  }
  if (!std::isfinite(input.stamp_sec) || input.stamp_sec <= 0.0) {
    result.reason = "evaluation_time_invalid";
    return result;
  }
  if (usable(input.current_candidate, input.stamp_sec,
             config.maximum_candidate_age_sec)) {
    return fromCandidate(
        input.current_candidate,
        PendingCargoEnvelopeSource::CURRENT_CANDIDATE,
        config, "current_candidate_envelope");
  }
  if (usable(input.retired_formal_shape, input.stamp_sec,
             config.maximum_retired_age_sec)) {
    return fromCandidate(
        input.retired_formal_shape,
        PendingCargoEnvelopeSource::RETIRED_FORMAL_SHAPE,
        config, "retired_formal_shape_envelope");
  }
  if (usable(input.lift_origin_candidate, input.stamp_sec,
             config.maximum_retired_age_sec)) {
    return fromCandidate(
        input.lift_origin_candidate,
        PendingCargoEnvelopeSource::LIFT_ORIGIN_CANDIDATE,
        config, "bound_lift_origin_envelope");
  }
  if (!input.hook_anchor_valid || !input.hook_anchor_base.allFinite()) {
    result.reason = "hook_anchor_invalid";
    return result;
  }
  PendingCargoEnvelopeCandidate fallback;
  fallback.valid = true;
  fallback.center_base = input.hook_anchor_base;
  fallback.center_base.z() += config.configured_center_offset_z_m;
  fallback.length_m = config.configured_length_m;
  fallback.width_m = config.configured_width_m;
  fallback.height_m = config.configured_height_m;
  fallback.evidence_stamp_sec = input.stamp_sec;
  return fromCandidate(
      fallback, PendingCargoEnvelopeSource::CONFIGURED_CONSERVATIVE,
      config, "configured_conservative_envelope");
}

CargoObbFootprint toCargoObbFootprint(
    const PendingCargoEnvelope& envelope) {
  CargoObbFootprint footprint;
  footprint.valid = envelope.valid;
  footprint.center_base = envelope.center_base.head<2>();
  footprint.length_m = envelope.length_m;
  footprint.width_m = envelope.width_m;
  footprint.yaw_base_rad = envelope.yaw_base_rad;
  footprint.min_z = envelope.bottom_z_base;
  footprint.max_z = envelope.top_z_base;
  return footprint;
}

}  // namespace ndt_slam
