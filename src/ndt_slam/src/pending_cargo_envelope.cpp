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
      std::isfinite(config.candidate_shape_hold_sec) &&
      config.candidate_shape_hold_sec >= config.maximum_candidate_age_sec &&
      std::isfinite(config.candidate_shape_growth_rate_mps) &&
      config.candidate_shape_growth_rate_mps >= 0.0F &&
      std::isfinite(config.candidate_shape_shrink_rate_mps) &&
      config.candidate_shape_shrink_rate_mps >= 0.0F &&
      std::isfinite(config.maximum_fallback_sway_offset_m) &&
      config.maximum_fallback_sway_offset_m >= 0.0F &&
      std::isfinite(config.lost_position_uncertainty_per_sec) &&
      config.lost_position_uncertainty_per_sec >= 0.0F &&
      std::isfinite(config.maximum_lost_position_uncertainty_m) &&
      config.maximum_lost_position_uncertainty_m >= 0.0F;
}

bool usablePose(const CargoEnvelopePoseCandidate& candidate,
                double stamp_sec, double maximum_age_sec,
                std::uint64_t lifecycle_id) {
  if (!candidate.valid || candidate.source == CargoEnvelopePoseSource::NONE ||
      !candidate.center_base.allFinite() ||
      !std::isfinite(candidate.yaw_base_rad) ||
      !std::isfinite(candidate.horizontal_uncertainty_m) ||
      !std::isfinite(candidate.vertical_uncertainty_m) ||
      candidate.horizontal_uncertainty_m < 0.0F ||
      candidate.vertical_uncertainty_m < 0.0F ||
      !std::isfinite(candidate.evidence_stamp_sec) ||
      candidate.evidence_stamp_sec <= 0.0 ||
      candidate.cargo_lifecycle_id != lifecycle_id) {
    return false;
  }
  const double age_sec = stamp_sec - candidate.evidence_stamp_sec;
  return age_sec >= -1.0e-3 && age_sec <= maximum_age_sec;
}

bool usableShape(const CargoEnvelopeShapeCandidate& candidate,
                 double stamp_sec, double maximum_age_sec,
                 std::uint64_t lifecycle_id) {
  if (!candidate.valid ||
      candidate.source == CargoEnvelopeShapeSource::NONE ||
      !std::isfinite(candidate.length_m) || candidate.length_m <= 0.0F ||
      !std::isfinite(candidate.width_m) || candidate.width_m <= 0.0F ||
      !std::isfinite(candidate.height_m) || candidate.height_m <= 0.0F ||
      !std::isfinite(candidate.yaw_rad) ||
      !std::isfinite(candidate.uncertainty_m) ||
      candidate.uncertainty_m < 0.0F ||
      candidate.cargo_lifecycle_id != lifecycle_id) {
    return false;
  }
  if (candidate.source ==
      CargoEnvelopeShapeSource::CONFIGURED_CONSERVATIVE_DEFAULT) {
    return true;
  }
  if (!std::isfinite(candidate.evidence_stamp_sec) ||
      candidate.evidence_stamp_sec <= 0.0) {
    return false;
  }
  const double age_sec = stamp_sec - candidate.evidence_stamp_sec;
  return age_sec >= -1.0e-3 && age_sec <= maximum_age_sec;
}

PendingCargoEnvelopeSource combinedSource(
    CargoEnvelopePoseSource pose, CargoEnvelopeShapeSource shape) {
  const bool current_pose =
      pose == CargoEnvelopePoseSource::CURRENT_ASSOCIATED_LIDAR ||
      pose == CargoEnvelopePoseSource::SHORT_TERM_TRACK_PREDICTION;
  const bool current_shape =
      shape == CargoEnvelopeShapeSource::CURRENT_HIGH_QUALITY_LIDAR ||
      shape == CargoEnvelopeShapeSource::CURRENT_TRACKED_BOUNDED_SHAPE;
  if (shape == CargoEnvelopeShapeSource::ACTIVE_LOCKED_TRACK_SHAPE) {
    return PendingCargoEnvelopeSource::ACTIVE_LOCKED_TRACK;
  }
  if (current_pose && current_shape) {
    return PendingCargoEnvelopeSource::CURRENT_CANDIDATE;
  }
  if (shape == CargoEnvelopeShapeSource::RETIRED_LOCKED_SHAPE) {
    return PendingCargoEnvelopeSource::RETIRED_FORMAL_SHAPE;
  }
  if (shape == CargoEnvelopeShapeSource::STATIC_ORIGIN_COMPONENT) {
    return PendingCargoEnvelopeSource::LIFT_ORIGIN_CANDIDATE;
  }
  return PendingCargoEnvelopeSource::CONFIGURED_CONSERVATIVE;
}

EffectiveCargoEnvelopeSource effectiveSource(
    CargoEnvelopeShapeSource shape) {
  switch (shape) {
    case CargoEnvelopeShapeSource::FORMAL_FROZEN_GEOMETRY:
      return EffectiveCargoEnvelopeSource::FORMAL_FROZEN_GEOMETRY;
    case CargoEnvelopeShapeSource::CURRENT_HIGH_QUALITY_LIDAR:
    case CargoEnvelopeShapeSource::CURRENT_TRACKED_BOUNDED_SHAPE:
    case CargoEnvelopeShapeSource::ACTIVE_LOCKED_TRACK_SHAPE:
      return EffectiveCargoEnvelopeSource::CURRENT_ASSOCIATED_TRACK;
    case CargoEnvelopeShapeSource::RETIRED_LOCKED_SHAPE:
      return EffectiveCargoEnvelopeSource::RETIRED_LOCKED_SHAPE;
    case CargoEnvelopeShapeSource::STATIC_ORIGIN_COMPONENT:
      return EffectiveCargoEnvelopeSource::STATIC_ORIGIN_COMPONENT;
    case CargoEnvelopeShapeSource::CONFIGURED_CONSERVATIVE_DEFAULT:
      return EffectiveCargoEnvelopeSource::CONFIGURED_CONSERVATIVE_DEFAULT;
    case CargoEnvelopeShapeSource::NONE:
      return EffectiveCargoEnvelopeSource::NONE;
  }
  return EffectiveCargoEnvelopeSource::NONE;
}

CargoEnvelopePoseSource formalPoseSource(CargoPoseSource source) {
  switch (source) {
    case CargoPoseSource::CURRENT_ASSOCIATED_LIDAR:
      return CargoEnvelopePoseSource::CURRENT_ASSOCIATED_LIDAR;
    case CargoPoseSource::MOTION_PREDICTION:
    case CargoPoseSource::RECENT_STABLE_HOLD:
      return CargoEnvelopePoseSource::SHORT_TERM_TRACK_PREDICTION;
    case CargoPoseSource::HOOK_OR_CONTROLLER:
      return CargoEnvelopePoseSource::HOOK_PLUS_LAST_RELIABLE_OFFSET;
    case CargoPoseSource::STATIC_INSTALLATION_FALLBACK:
      return CargoEnvelopePoseSource::NONE;
  }
  return CargoEnvelopePoseSource::NONE;
}

PendingCargoEnvelope combinePoseAndShape(
    const CargoEnvelopePoseCandidate& pose,
    const CargoEnvelopeShapeCandidate& shape,
    const PendingCargoEnvelopeConfig& config,
    double evaluation_stamp_sec) {
  PendingCargoEnvelope result;
  if (!pose.valid || !shape.valid) {
    result.reason = !pose.valid ? "current_cargo_pose_unavailable"
                                : "cargo_shape_unavailable";
    return result;
  }
  const double age_sec = std::max(
      0.0, evaluation_stamp_sec - pose.evidence_stamp_sec);
  // The configured fallback box is a source of last resort, not a physical floor
  // for measured/current, retired, or static-origin shapes. Position loss is
  // computed by the caller from the last reliable pose timestamp and carried
  // in pose.horizontal_uncertainty_m; pose evidence age is diagnostic only.
  const bool configured_shape = shape.source ==
      CargoEnvelopeShapeSource::CONFIGURED_CONSERVATIVE_DEFAULT;
  const float base_length = configured_shape
      ? config.configured_length_m : shape.length_m;
  const float base_width = configured_shape
      ? config.configured_width_m : shape.width_m;
  const float base_height = configured_shape
      ? config.configured_height_m : shape.height_m;
  const float horizontal_uncertainty = pose.horizontal_uncertainty_m +
      shape.uncertainty_m + config.horizontal_margin_m;
  const float vertical_uncertainty = pose.vertical_uncertainty_m +
      shape.uncertainty_m + config.vertical_margin_m;

  result.valid = true;
  result.source = combinedSource(pose.source, shape.source);
  result.pose_source = pose.source;
  result.shape_source = shape.source;
  result.center_base = pose.center_base;
  // Nominal physical dimensions drive RViz and diagnostics. Safety queries
  // expand the OBB explicitly via toCargoObbFootprint(), exactly once.
  result.length_m = base_length;
  result.width_m = base_width;
  result.height_m = base_height;
  // P1-5: only pose candidates with explicit yaw authority may override
  // the shape yaw. A low-quality or default (0-rad) current-pose yaw must
  // not silently overwrite a reliable static-origin or locked-shape yaw.
  result.yaw_base_rad = pose.yaw_authoritative
      ? pose.yaw_base_rad : shape.yaw_rad;
  result.horizontal_uncertainty_m = horizontal_uncertainty;
  result.vertical_uncertainty_m = vertical_uncertainty;
  result.evidence_stamp_sec = pose.evidence_stamp_sec;
  result.evaluation_stamp_sec = evaluation_stamp_sec;
  result.age_sec = age_sec;
  result.cargo_lifecycle_id = pose.cargo_lifecycle_id;
  result.bottom_z_base = pose.center_base.z() - 0.5F * result.height_m;
  result.top_z_base = pose.center_base.z() + 0.5F * result.height_m;
  result.reason = std::string("pending_pose=") +
      cargoEnvelopePoseSourceName(pose.source) + ";shape=" +
      cargoEnvelopeShapeSourceName(shape.source);
  return result;
}

}  // namespace

const char* cargoEnvelopePoseSourceName(
    CargoEnvelopePoseSource source) noexcept {
  switch (source) {
    case CargoEnvelopePoseSource::NONE: return "NONE";
    case CargoEnvelopePoseSource::CURRENT_ASSOCIATED_LIDAR:
      return "CURRENT_ASSOCIATED_LIDAR";
    case CargoEnvelopePoseSource::SHORT_TERM_TRACK_PREDICTION:
      return "SHORT_TERM_TRACK_PREDICTION";
    case CargoEnvelopePoseSource::RETIRED_TRACK_PREDICTION:
      return "RETIRED_TRACK_PREDICTION";
    case CargoEnvelopePoseSource::HOOK_PLUS_LAST_RELIABLE_OFFSET:
      return "HOOK_PLUS_LAST_RELIABLE_OFFSET";
    case CargoEnvelopePoseSource::HOOK_DEFAULT_OFFSET:
      return "HOOK_DEFAULT_OFFSET";
  }
  return "NONE";
}

const char* cargoEnvelopeShapeSourceName(
    CargoEnvelopeShapeSource source) noexcept {
  switch (source) {
    case CargoEnvelopeShapeSource::NONE: return "NONE";
    case CargoEnvelopeShapeSource::FORMAL_FROZEN_GEOMETRY:
      return "FORMAL_FROZEN_GEOMETRY";
    case CargoEnvelopeShapeSource::CURRENT_HIGH_QUALITY_LIDAR:
      return "CURRENT_HIGH_QUALITY_LIDAR";
    case CargoEnvelopeShapeSource::CURRENT_TRACKED_BOUNDED_SHAPE:
      return "CURRENT_TRACKED_BOUNDED_SHAPE";
    case CargoEnvelopeShapeSource::ACTIVE_LOCKED_TRACK_SHAPE:
      return "ACTIVE_LOCKED_TRACK_SHAPE";
    case CargoEnvelopeShapeSource::RETIRED_LOCKED_SHAPE:
      return "RETIRED_LOCKED_SHAPE";
    case CargoEnvelopeShapeSource::STATIC_ORIGIN_COMPONENT:
      return "STATIC_ORIGIN_COMPONENT";
    case CargoEnvelopeShapeSource::CONFIGURED_CONSERVATIVE_DEFAULT:
      return "CONFIGURED_CONSERVATIVE_DEFAULT";
  }
  return "NONE";
}

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
    case PendingCargoEnvelopeSource::ACTIVE_LOCKED_TRACK:
      return "ACTIVE_LOCKED_TRACK";
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
  if (!std::isfinite(input.stamp_sec) || input.stamp_sec <= 0.0 ||
      input.cargo_lifecycle_id == 0U) {
    result.reason = "evaluation_time_or_lifecycle_invalid";
    return result;
  }

  CargoEnvelopePoseCandidate pose;
  if (usablePose(input.current_associated_pose, input.stamp_sec,
                 config.maximum_candidate_age_sec,
                 input.cargo_lifecycle_id)) {
    pose = input.current_associated_pose;
  } else if (usablePose(input.short_term_track_pose, input.stamp_sec,
                        config.maximum_candidate_age_sec,
                        input.cargo_lifecycle_id)) {
    pose = input.short_term_track_pose;
  } else if (usablePose(input.retired_track_pose, input.stamp_sec,
                        config.maximum_retired_age_sec,
                        input.cargo_lifecycle_id)) {
    pose = input.retired_track_pose;
  } else if (usablePose(input.hook_last_offset_pose, input.stamp_sec,
                        config.maximum_retired_age_sec,
                        input.cargo_lifecycle_id)) {
    pose = input.hook_last_offset_pose;
  } else if (usablePose(input.hook_default_pose, input.stamp_sec,
                        config.maximum_candidate_age_sec,
                        input.cargo_lifecycle_id)) {
    pose = input.hook_default_pose;
  } else {
    result.reason = "current_cargo_pose_unavailable";
    return result;
  }

  CargoEnvelopeShapeCandidate shape;
  if (usableShape(input.formal_frozen_shape, input.stamp_sec,
                  config.maximum_retired_age_sec,
                  input.cargo_lifecycle_id)) {
    shape = input.formal_frozen_shape;
  } else if (usableShape(input.active_locked_shape, input.stamp_sec,
                         config.maximum_candidate_age_sec,
                         input.cargo_lifecycle_id)) {
    shape = input.active_locked_shape;
  } else if (usableShape(input.current_high_quality_shape, input.stamp_sec,
                         config.maximum_candidate_age_sec,
                         input.cargo_lifecycle_id)) {
    shape = input.current_high_quality_shape;
  } else if (usableShape(input.current_tracked_bounded_shape, input.stamp_sec,
                         config.candidate_shape_hold_sec,
                         input.cargo_lifecycle_id)) {
    shape = input.current_tracked_bounded_shape;
  } else if (usableShape(input.retired_locked_shape, input.stamp_sec,
                         config.maximum_retired_age_sec,
                         input.cargo_lifecycle_id)) {
    shape = input.retired_locked_shape;
  } else if (usableShape(input.static_origin_shape, input.stamp_sec,
                         config.maximum_retired_age_sec,
                         input.cargo_lifecycle_id)) {
    shape = input.static_origin_shape;
  } else {
    shape.valid = true;
    shape.length_m = config.configured_length_m;
    shape.width_m = config.configured_width_m;
    shape.height_m = config.configured_height_m;
    shape.evidence_stamp_sec = input.stamp_sec;
    shape.cargo_lifecycle_id = input.cargo_lifecycle_id;
    shape.source =
        CargoEnvelopeShapeSource::CONFIGURED_CONSERVATIVE_DEFAULT;
  }
  return combinePoseAndShape(pose, shape, config, input.stamp_sec);
}

CargoObbFootprint toCargoObbFootprint(
    const PendingCargoEnvelope& envelope,
    float horizontal_expansion_m,
    float vertical_expansion_m) {
  CargoObbFootprint footprint;
  const bool expansion_valid =
      std::isfinite(horizontal_expansion_m) &&
      horizontal_expansion_m >= 0.0F &&
      std::isfinite(vertical_expansion_m) &&
      vertical_expansion_m >= 0.0F;
  footprint.valid = envelope.valid && expansion_valid;
  footprint.center_base = envelope.center_base.head<2>();
  footprint.length_m = envelope.length_m + 2.0F * horizontal_expansion_m;
  footprint.width_m = envelope.width_m + 2.0F * horizontal_expansion_m;
  footprint.yaw_base_rad = envelope.yaw_base_rad;
  footprint.min_z = envelope.bottom_z_base - vertical_expansion_m;
  footprint.max_z = envelope.top_z_base + vertical_expansion_m;
  return footprint;
}

PendingCargoVerticalPlausibilityResult
evaluatePendingCargoVerticalPlausibility(
    const PendingCargoVerticalPlausibilityInput& input) {
  PendingCargoVerticalPlausibilityResult result;
  if (!input.envelope_valid || !std::isfinite(input.center_z_base) ||
      !std::isfinite(input.height_m) || input.height_m <= 0.0F ||
      !std::isfinite(input.vertical_uncertainty_m) ||
      input.vertical_uncertainty_m < 0.0F ||
      !std::isfinite(input.minimum_height_m) ||
      std::isnan(input.maximum_height_m) ||
      input.minimum_height_m < 0.0F ||
      input.maximum_height_m < input.minimum_height_m ||
      !std::isfinite(input.maximum_ground_penetration_m) ||
      input.maximum_ground_penetration_m < 0.0F ||
      !std::isfinite(input.maximum_trusted_center_age_sec) ||
      input.maximum_trusted_center_age_sec < 0.0 ||
      !std::isfinite(input.maximum_trusted_center_z_delta_m) ||
      input.maximum_trusted_center_z_delta_m < 0.0F) {
    result.reason = "retired_vertical_interval_invalid";
    return result;
  }
  if (input.height_m < input.minimum_height_m ||
      input.height_m > input.maximum_height_m) {
    result.reason = "retired_height_out_of_physical_range";
    return result;
  }
  result.bottom_z_base = input.center_z_base - 0.5F * input.height_m;
  result.top_z_base = input.center_z_base + 0.5F * input.height_m;
  if (!std::isfinite(result.bottom_z_base) ||
      !std::isfinite(result.top_z_base) ||
      result.top_z_base <= result.bottom_z_base) {
    result.reason = "retired_vertical_interval_invalid";
    return result;
  }

  bool reference_available = false;
  bool trusted_center_used = false;
  if (input.current_lidar_pose_authoritative) {
    reference_available = true;
  }
  if (input.local_ground_valid &&
      std::isfinite(input.local_ground_z_base)) {
    reference_available = true;
    if (result.top_z_base + input.vertical_uncertainty_m <=
        input.local_ground_z_base) {
      result.reason = "retired_pose_below_local_ground";
      return result;
    }
    if (result.bottom_z_base + input.vertical_uncertainty_m <
        input.local_ground_z_base -
            input.maximum_ground_penetration_m) {
      result.reason = "retired_pose_ground_penetration_excessive";
      return result;
    }
  }
  if (input.hook_anchor_z_authoritative &&
      std::isfinite(input.hook_anchor_z_base)) {
    reference_available = true;
    if (result.top_z_base - input.vertical_uncertainty_m >
        input.hook_anchor_z_base +
            input.maximum_ground_penetration_m) {
      result.reason = "retired_pose_hook_geometry_inconsistent";
      return result;
    }
  }
  if (!reference_available && input.trusted_center_valid &&
      std::isfinite(input.trusted_center_z_base) &&
      std::isfinite(input.trusted_center_age_sec) &&
      input.trusted_center_age_sec >= 0.0 &&
      input.trusted_center_age_sec <=
          input.maximum_trusted_center_age_sec) {
    const float center_delta =
        std::abs(input.center_z_base - input.trusted_center_z_base);
    if (center_delta >
        input.maximum_trusted_center_z_delta_m +
            input.vertical_uncertainty_m) {
      result.reason = "retired_pose_trusted_center_discontinuity";
      return result;
    }
    reference_available = true;
    trusted_center_used = true;
  }
  if (!reference_available) {
    result.reason = "retired_pose_ground_reference_unavailable";
    return result;
  }
  result.valid = true;
  result.reason = input.current_lidar_pose_authoritative
      ? "current_lidar_pose_physically_plausible"
      : (trusted_center_used
             ? "held_pose_matches_trusted_center"
             : "retired_pose_physically_plausible");
  return result;
}

EffectiveCargoEnvelope composeEffectiveCargoEnvelope(
    const CargoPresenceResult& presence,
    const CargoEnvelopePoseCandidate& pose,
    const CargoEnvelopeShapeCandidate& shape,
    const PendingCargoEnvelopeConfig& requested_config) {
  EffectiveCargoEnvelope result;
  if (!presence.cargo_present) {
    result.reason = "cargo_not_present";
    return result;
  }
  const PendingCargoEnvelopeConfig config = validConfig(requested_config)
      ? requested_config : PendingCargoEnvelopeConfig{};
  const double evaluation_stamp_sec = std::max(
      pose.evidence_stamp_sec, shape.evidence_stamp_sec);
  const PendingCargoEnvelope pending = combinePoseAndShape(
      pose, shape, config, evaluation_stamp_sec);
  if (!pending.valid) {
    result.reason = pending.reason;
    return result;
  }
  result.valid = true;
  result.fallback_active = true;
  result.source = effectiveSource(shape.source);
  result.pose_source = pose.source;
  result.shape_source = shape.source;
  result.footprint = toCargoObbFootprint(pending);
  result.horizontal_uncertainty_m = pending.horizontal_uncertainty_m;
  result.vertical_uncertainty_m = pending.vertical_uncertainty_m;
  result.evidence_stamp_sec = pending.evidence_stamp_sec;
  result.age_sec = pending.age_sec;
  result.cargo_lifecycle_id = pending.cargo_lifecycle_id;
  result.reason = pending.reason;
  return result;
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
      formal_geometry.pose.valid &&
      std::isfinite(formal_geometry.bottom_z_base) &&
      std::isfinite(formal_geometry.top_z_base) &&
      formal_geometry.top_z_base > formal_geometry.bottom_z_base) {
    const double height_age_sec = formal_geometry.evaluation_stamp_sec -
        formal_geometry.height_evidence_stamp_sec;
    const double pose_age_sec = formal_geometry.evaluation_stamp_sec -
        formal_geometry.pose_evidence_stamp_sec;
    const bool height_evidence_fresh =
        std::isfinite(formal_geometry.height_evidence_stamp_sec) &&
        formal_geometry.height_evidence_stamp_sec > 0.0 &&
        std::isfinite(height_age_sec) && height_age_sec >= -1.0e-3 &&
        height_age_sec <= config.maximum_formal_height_age_sec;
    const bool pose_evidence_fresh =
        std::isfinite(formal_geometry.pose_evidence_stamp_sec) &&
        formal_geometry.pose_evidence_stamp_sec > 0.0 &&
        std::isfinite(pose_age_sec) && pose_age_sec >= -1.0e-3 &&
        pose_age_sec <= config.maximum_formal_pose_age_sec;
    result.valid = true;
    result.formal = true;
    result.can_authorize_clear = presence.gravity_authoritative &&
        height_evidence_fresh &&
        pose_evidence_fresh &&
        formalPoseSource(formal_geometry.pose.source) !=
            CargoEnvelopePoseSource::NONE;
    result.source =
        EffectiveCargoEnvelopeSource::FORMAL_FROZEN_GEOMETRY;
    result.pose_source = formalPoseSource(formal_geometry.pose.source);
    result.shape_source =
        CargoEnvelopeShapeSource::FORMAL_FROZEN_GEOMETRY;
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
    result.reason = height_evidence_fresh && pose_evidence_fresh
        ? "formal_frozen_geometry"
        : (!pose_evidence_fresh
               ? "formal_geometry_pose_evidence_stale"
               : "formal_geometry_height_evidence_stale");
    return result;
  }
  if (pending_geometry.valid) {
    result.valid = true;
    result.fallback_active = true;
    result.source = effectiveSource(pending_geometry.shape_source);
    result.pose_source = pending_geometry.pose_source;
    result.shape_source = pending_geometry.shape_source;
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

  // A configured size without a current pose is not a geometric observation.
  // In particular, never publish a valid centre=(0,0) emergency box.
  result.reason = "current_cargo_pose_unavailable";
  return result;
}

}  // namespace ndt_slam
