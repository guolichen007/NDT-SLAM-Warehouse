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
      config.maximum_retired_age_sec >= 0.0 &&
      std::isfinite(config.maximum_fallback_sway_offset_m) &&
      config.maximum_fallback_sway_offset_m >= 0.0F &&
      std::isfinite(config.lost_position_uncertainty_per_sec) &&
      config.lost_position_uncertainty_per_sec >= 0.0F &&
      std::isfinite(config.maximum_lost_position_uncertainty_m) &&
      config.maximum_lost_position_uncertainty_m >= 0.0F;
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
    double evaluation_stamp_sec,
    const char* reason) {
  PendingCargoEnvelope result;
  const double age_sec = std::max(
      0.0, evaluation_stamp_sec - candidate.evidence_stamp_sec);
  const float age_uncertainty = std::min(
      config.maximum_lost_position_uncertainty_m,
      config.lost_position_uncertainty_per_sec *
          static_cast<float>(age_sec));
  // Pending evidence may enlarge the conservative envelope immediately, but
  // it can never shrink below the configured commissioning floor.
  const float base_length = std::max(
      candidate.length_m, config.configured_length_m);
  const float base_width = std::max(
      candidate.width_m, config.configured_width_m);
  const float base_height = std::max(
      candidate.height_m, config.configured_height_m);
  const float expanded_half_length = 0.5F * base_length +
      candidate.horizontal_uncertainty_m + age_uncertainty +
      config.horizontal_margin_m;
  const float expanded_half_width = 0.5F * base_width +
      candidate.horizontal_uncertainty_m + age_uncertainty +
      config.horizontal_margin_m;
  const float expanded_half_height = 0.5F * base_height +
      candidate.vertical_uncertainty_m + config.vertical_margin_m;
  result.valid = true;
  result.source = source;
  result.center_base = candidate.center_base;
  result.length_m = 2.0F * expanded_half_length;
  result.width_m = 2.0F * expanded_half_width;
  result.height_m = 2.0F * expanded_half_height;
  result.yaw_base_rad = candidate.yaw_base_rad;
  result.horizontal_uncertainty_m =
      candidate.horizontal_uncertainty_m + age_uncertainty +
      config.horizontal_margin_m;
  result.vertical_uncertainty_m =
      candidate.vertical_uncertainty_m + config.vertical_margin_m;
  result.evidence_stamp_sec = candidate.evidence_stamp_sec;
  result.evaluation_stamp_sec = evaluation_stamp_sec;
  result.age_sec = age_sec;
  result.cargo_lifecycle_id = candidate.cargo_lifecycle_id;
  result.bottom_z_base = candidate.center_base.z() - expanded_half_height;
  result.top_z_base = candidate.center_base.z() + expanded_half_height;
  result.reason = reason;
  return result;
}

EffectiveCargoEnvelopeSource effectiveSource(
    PendingCargoEnvelopeSource source) {
  switch (source) {
    case PendingCargoEnvelopeSource::CURRENT_CANDIDATE:
      return EffectiveCargoEnvelopeSource::CURRENT_ASSOCIATED_TRACK;
    case PendingCargoEnvelopeSource::RETIRED_FORMAL_SHAPE:
      return EffectiveCargoEnvelopeSource::RETIRED_LOCKED_SHAPE;
    case PendingCargoEnvelopeSource::LIFT_ORIGIN_CANDIDATE:
      return EffectiveCargoEnvelopeSource::STATIC_ORIGIN_COMPONENT;
    case PendingCargoEnvelopeSource::CONFIGURED_CONSERVATIVE:
      return EffectiveCargoEnvelopeSource::CONFIGURED_CONSERVATIVE_DEFAULT;
    case PendingCargoEnvelopeSource::NONE:
      return EffectiveCargoEnvelopeSource::NONE;
  }
  return EffectiveCargoEnvelopeSource::NONE;
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
        config, input.stamp_sec, "current_candidate_envelope");
  }
  if (usable(input.retired_formal_shape, input.stamp_sec,
             config.maximum_retired_age_sec)) {
    return fromCandidate(
        input.retired_formal_shape,
        PendingCargoEnvelopeSource::RETIRED_FORMAL_SHAPE,
        config, input.stamp_sec, "retired_formal_shape_envelope");
  }
  if (usable(input.lift_origin_candidate, input.stamp_sec,
             config.maximum_retired_age_sec)) {
    return fromCandidate(
        input.lift_origin_candidate,
        PendingCargoEnvelopeSource::LIFT_ORIGIN_CANDIDATE,
        config, input.stamp_sec, "bound_lift_origin_envelope");
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
  fallback.horizontal_uncertainty_m =
      config.maximum_fallback_sway_offset_m;
  fallback.vertical_uncertainty_m = 0.0F;
  fallback.evidence_stamp_sec = input.stamp_sec;
  fallback.cargo_lifecycle_id = input.cargo_lifecycle_id;
  result = fromCandidate(
      fallback, PendingCargoEnvelopeSource::CONFIGURED_CONSERVATIVE,
      config, input.stamp_sec, "configured_conservative_envelope");
  return result;
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

const char* effectiveCargoEnvelopeSourceName(
    EffectiveCargoEnvelopeSource source) noexcept {
  switch (source) {
    case EffectiveCargoEnvelopeSource::NONE: return "NONE";
    case EffectiveCargoEnvelopeSource::FORMAL_FROZEN_GEOMETRY:
      return "FORMAL_FROZEN_GEOMETRY";
    case EffectiveCargoEnvelopeSource::CURRENT_ASSOCIATED_TRACK:
      return "CURRENT_ASSOCIATED_TRACK";
    case EffectiveCargoEnvelopeSource::RETIRED_LOCKED_SHAPE:
      return "RETIRED_LOCKED_SHAPE";
    case EffectiveCargoEnvelopeSource::STATIC_ORIGIN_COMPONENT:
      return "STATIC_ORIGIN_COMPONENT";
    case EffectiveCargoEnvelopeSource::CONFIGURED_CONSERVATIVE_DEFAULT:
      return "CONFIGURED_CONSERVATIVE_DEFAULT";
  }
  return "NONE";
}

EffectiveCargoEnvelope resolveEffectiveCargoEnvelope(
    const CargoPresenceResult& presence,
    const RigidCargoGeometry& formal_geometry,
    const PendingCargoEnvelope& pending_geometry,
    const CargoEnvelopeResolverConfig& config) {
  EffectiveCargoEnvelope result;
  if (!presence.cargo_present) {
    result.reason = "cargo_not_present";
    return result;
  }
  if (formal_geometry.valid && formal_geometry.shape.valid &&
      formal_geometry.pose.valid) {
    result.valid = true;
    result.formal = true;
    result.can_authorize_clear = presence.gravity_authoritative;
    result.source =
        EffectiveCargoEnvelopeSource::FORMAL_FROZEN_GEOMETRY;
    result.footprint = toCargoObbFootprint(
        formal_geometry, formal_geometry.horizontal_uncertainty_m);
    result.horizontal_uncertainty_m =
        formal_geometry.horizontal_uncertainty_m;
    result.vertical_uncertainty_m = formal_geometry.vertical_uncertainty_m;
    result.evidence_stamp_sec = formal_geometry.pose_evidence_stamp_sec;
    result.age_sec = std::max(
        0.0, formal_geometry.evaluation_stamp_sec -
                 formal_geometry.pose_evidence_stamp_sec);
    result.cargo_lifecycle_id = formal_geometry.track_id;
    result.reason = "formal_frozen_geometry";
    return result;
  }
  if (pending_geometry.valid) {
    result.valid = true;
    result.fallback_active = true;
    result.source = effectiveSource(pending_geometry.source);
    result.footprint = toCargoObbFootprint(pending_geometry);
    result.horizontal_uncertainty_m =
        pending_geometry.horizontal_uncertainty_m;
    result.vertical_uncertainty_m = pending_geometry.vertical_uncertainty_m;
    result.evidence_stamp_sec = pending_geometry.evidence_stamp_sec;
    result.age_sec = pending_geometry.age_sec;
    result.cargo_lifecycle_id = pending_geometry.cargo_lifecycle_id;
    result.reason = pending_geometry.reason;
    return result;
  }

  result.valid = true;
  result.fallback_active = true;
  result.source =
      EffectiveCargoEnvelopeSource::CONFIGURED_CONSERVATIVE_DEFAULT;
  result.footprint.valid = true;
  result.footprint.length_m = config.configured_length_m +
      2.0F * config.horizontal_uncertainty_m;
  result.footprint.width_m = config.configured_width_m +
      2.0F * config.horizontal_uncertainty_m;
  result.footprint.min_z = config.configured_center_z_m -
      0.5F * config.configured_height_m -
      config.vertical_uncertainty_m;
  result.footprint.max_z = config.configured_center_z_m +
      0.5F * config.configured_height_m +
      config.vertical_uncertainty_m;
  result.horizontal_uncertainty_m = config.horizontal_uncertainty_m;
  result.vertical_uncertainty_m = config.vertical_uncertainty_m;
  result.reason = "resolver_emergency_configured_fallback";
  return result;
}

}  // namespace ndt_slam
