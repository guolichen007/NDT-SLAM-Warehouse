#include "ndt_slam/cargo_track_policy.hpp"

#include "ndt_slam/cargo_oriented_footprint.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ndt_slam {
namespace {

constexpr float kHalfPi = 1.57079632679489661923F;
constexpr float kQuarterPi = 0.78539816339744830962F;

float unitScore(float value, float limit) {
  if (!std::isfinite(value) || !std::isfinite(limit) || limit <= 0.0F) {
    return 0.0F;
  }
  return std::clamp(1.0F - value / limit, 0.0F, 1.0F);
}

float relativeError(float value, float reference) {
  if (!std::isfinite(value) || !std::isfinite(reference) ||
      value <= 0.0F || reference <= 0.0F) {
    return std::numeric_limits<float>::infinity();
  }
  return std::abs(value - reference) / reference;
}

float median(std::vector<float> values) {
  if (values.empty()) return 0.0F;
  std::sort(values.begin(), values.end());
  return values[values.size() / 2U];
}

bool validDescriptor(const CargoCandidateDescriptor& descriptor) {
  return descriptor.component_id >= 0 && descriptor.center.allFinite() &&
      descriptor.size.allFinite() && descriptor.size.minCoeff() > 0.0F &&
      descriptor.size.x() >= descriptor.size.y() &&
      std::isfinite(descriptor.yaw_rad) &&
      std::isfinite(descriptor.orientation_confidence) &&
      descriptor.orientation_confidence >= 0.0F &&
      descriptor.orientation_confidence <= 1.0F &&
      descriptor.point_count > 0U;
}

Eigen::Vector2f projectHalfExtents(
    const CargoCandidateDescriptor& box, float reference_yaw) {
  const float delta = box.yaw_rad - reference_yaw;
  const float cosine = std::abs(std::cos(delta));
  const float sine = std::abs(std::sin(delta));
  return Eigen::Vector2f(
      0.5F * (cosine * box.size.x() + sine * box.size.y()),
      0.5F * (sine * box.size.x() + cosine * box.size.y()));
}

}  // namespace

float quantizeCargoAxialYawToOrthogonal(float yaw_rad) {
  if (!std::isfinite(yaw_rad)) return 0.0F;
  const float axial_yaw = normalizeCargoAxialYaw(yaw_rad);
  if (std::abs(axial_yaw) < kQuarterPi) return 0.0F;
  return std::copysign(kHalfPi, axial_yaw);
}

CargoTopSurfaceHeightResult evaluateCargoTopSurfaceHeight(
    const CargoTopSurfaceHeightInput& input) {
  CargoTopSurfaceHeightResult result;
  if (!std::isfinite(input.frozen_thickness_m) ||
      input.frozen_thickness_m <= 0.0F ||
      !std::isfinite(input.top_bottom_agreement_m) ||
      input.top_bottom_agreement_m < 0.0F) {
    result.reason = "invalid_frozen_thickness";
    return result;
  }
  const float half_height = 0.5F * input.frozen_thickness_m;
  if (input.top_valid && std::isfinite(input.top_z_base)) {
    const float top_derived_bottom =
        input.top_z_base - input.frozen_thickness_m;
    result.bottom_z_base = top_derived_bottom;
    result.top_z_base = input.top_z_base;
    result.used_top_surface = true;
    result.reason = "top_surface_minus_frozen_thickness";
    if (input.direct_bottom_valid &&
        std::isfinite(input.direct_bottom_z_base) &&
        std::abs(input.direct_bottom_z_base - top_derived_bottom) <=
            input.top_bottom_agreement_m) {
      result.bottom_z_base = 0.80F * top_derived_bottom +
          0.20F * input.direct_bottom_z_base;
      result.top_z_base = result.bottom_z_base +
          input.frozen_thickness_m;
      result.bottom_corroborated = true;
      result.reason = "top_surface_with_bottom_corroboration";
    }
    result.center_z_base = result.bottom_z_base + half_height;
    result.valid = std::isfinite(result.center_z_base) &&
        std::isfinite(result.bottom_z_base) &&
        std::isfinite(result.top_z_base);
    return result;
  }
  if (input.direct_bottom_valid &&
      std::isfinite(input.direct_bottom_z_base)) {
    result.bottom_z_base = input.direct_bottom_z_base;
    result.top_z_base = input.direct_bottom_z_base +
        input.frozen_thickness_m;
    result.center_z_base = input.direct_bottom_z_base + half_height;
    result.reason = "direct_bottom_fallback";
    result.valid = true;
  }
  return result;
}

const char* cargoLockAuthoritySourceName(CargoLockAuthoritySource source) {
  switch (source) {
    case CargoLockAuthoritySource::NONE: return "NONE";
    case CargoLockAuthoritySource::GRAVITY_LOADED:
      return "GRAVITY_LOADED";
    case CargoLockAuthoritySource::LIDAR_SUSPENDED:
      return "LIDAR_SUSPENDED";
    case CargoLockAuthoritySource::LIFT_FROM_ORIGIN:
      return "LIFT_FROM_ORIGIN";
  }
  return "NONE";
}

CargoPhysicalLockAuthorityDecision evaluateCargoPhysicalLockAuthority(
    const CargoPhysicalLockAuthorityInput& input) {
  CargoPhysicalLockAuthorityDecision decision;
  const bool gravity_loaded = input.gravity_valid &&
      input.gravity_state == HookLoadState::LOADED;
  const bool gravity_empty = input.gravity_valid &&
      input.gravity_state == HookLoadState::EMPTY;
  if (gravity_loaded && input.signal_role != HookLoadSignalRole::DISABLED) {
    decision.allowed = true;
    decision.source = CargoLockAuthoritySource::GRAVITY_LOADED;
    decision.reason = "gravity_loaded";
    return decision;
  }
  if (input.signal_role == HookLoadSignalRole::REQUIRED) {
    decision.reason = "required_gravity_not_loaded";
    return decision;
  }

  const int required_frames = std::max(1, input.required_lidar_confirm_frames);
  const bool suspended = std::isfinite(input.ground_clearance_m) &&
      input.ground_clearance_m >= input.minimum_ground_clearance_m &&
      input.suspension_confirm_frames >= required_frames;
  const bool lifted_from_origin =
      std::isfinite(input.lift_from_origin_m) &&
      input.lift_from_origin_m >= input.minimum_lift_from_origin_m &&
      input.lift_confirm_frames >= required_frames;
  if (lifted_from_origin) {
    decision.allowed = true;
    decision.source = CargoLockAuthoritySource::LIFT_FROM_ORIGIN;
    decision.gravity_conflict = gravity_empty;
    decision.reason = gravity_empty
        ? "lidar_lift_overrides_auxiliary_empty" : "lidar_lift_from_origin";
    return decision;
  }
  if (suspended) {
    decision.allowed = true;
    decision.source = CargoLockAuthoritySource::LIDAR_SUSPENDED;
    decision.gravity_conflict = gravity_empty;
    decision.reason = gravity_empty
        ? "lidar_suspended_overrides_auxiliary_empty" : "lidar_suspended";
    return decision;
  }
  decision.gravity_conflict = gravity_empty;
  decision.reason = gravity_empty
      ? "auxiliary_empty_requires_independent_lift"
      : "lidar_physical_authority_unconfirmed";
  return decision;
}

CargoRearmDecision evaluateCargoRearm(const CargoRearmInput& input) {
  CargoRearmDecision decision;
  if (!std::isfinite(input.rearm_age_sec) || input.rearm_age_sec < 0.0 ||
      !std::isfinite(input.minimum_empty_confirm_sec) ||
      input.minimum_empty_confirm_sec < 0.0 ||
      !std::isfinite(input.candidate_score_margin) ||
      !std::isfinite(input.minimum_score_margin) ||
      !std::isfinite(input.retired_identity_confidence) ||
      !std::isfinite(input.maximum_retired_identity_confidence)) {
    decision.reason = "invalid_rearm_input";
    return decision;
  }
  if (input.empty_confirmed &&
      input.rearm_age_sec >= input.minimum_empty_confirm_sec) {
    decision.allowed = true;
    decision.reason = "confirmed_empty";
    return decision;
  }
  if (input.gravity_valid &&
      input.gravity_state == HookLoadState::EMPTY &&
      input.rearm_age_sec >= input.minimum_empty_confirm_sec) {
    // Rearming recognition is not a safety CLEAR decision. A persistent
    // LiDAR structure may keep the outward status in gravity/LiDAR conflict,
    // but authoritative EMPTY must still retire the old lock so the next
    // LOADED edge can start a new lifecycle instead of deadlocking forever.
    decision.allowed = true;
    decision.reason = input.candidate_valid
        ? "gravity_empty_rearm_lidar_conflict_retained"
        : "gravity_empty_rearm";
    return decision;
  }
  if (input.gravity_valid && input.gravity_state == HookLoadState::LOADED &&
      input.gravity_state_at_clear != HookLoadState::LOADED) {
    decision.allowed = true;
    decision.reason = "new_gravity_loaded_edge";
    return decision;
  }
  if (input.candidate_valid && input.independent_suspension_evidence &&
      input.candidate_score_margin >= input.minimum_score_margin &&
      input.retired_identity_confidence <
          input.maximum_retired_identity_confidence) {
    decision.allowed = true;
    decision.reason = "independent_suspended_candidate";
  }
  return decision;
}

CargoCandidateRanking rankCargoCandidateIdentityScores(
    const std::vector<CargoCandidateIdentityScore>& scores) {
  CargoCandidateRanking ranking;
  for (const CargoCandidateIdentityScore& score : scores) {
    if (!score.valid) continue;
    const float rank = score.identity_confidence +
        0.25F * score.overall_lock_confidence;
    if (!ranking.valid || rank > ranking.top1_rank) {
      ranking.top2 = ranking.top1;
      ranking.top2_rank = ranking.top1_rank;
      ranking.top1 = score;
      ranking.top1_rank = rank;
      ranking.valid = true;
    } else if (!ranking.top2.valid || rank > ranking.top2_rank) {
      ranking.top2 = score;
      ranking.top2_rank = rank;
    }
  }
  ranking.margin = ranking.valid
      ? ranking.top1_rank - ranking.top2_rank : 0.0F;
  return ranking;
}

float cargoOrientedOverlapRatio(
    const CargoCandidateDescriptor& lhs,
    const CargoCandidateDescriptor& rhs) {
  if (!validDescriptor(lhs) || !validDescriptor(rhs)) return 0.0F;
  const float cosine = std::cos(lhs.yaw_rad);
  const float sine = std::sin(lhs.yaw_rad);
  const Eigen::Vector2f delta = rhs.center.head<2>() - lhs.center.head<2>();
  const Eigen::Vector2f local_delta(
      cosine * delta.x() + sine * delta.y(),
      -sine * delta.x() + cosine * delta.y());
  const Eigen::Vector2f lhs_half = 0.5F * lhs.size.head<2>();
  const Eigen::Vector2f rhs_half = projectHalfExtents(rhs, lhs.yaw_rad);
  const float intersection_x = std::max(
      0.0F, lhs_half.x() + rhs_half.x() - std::abs(local_delta.x()));
  const float intersection_y = std::max(
      0.0F, lhs_half.y() + rhs_half.y() - std::abs(local_delta.y()));
  const float intersection = intersection_x * intersection_y;
  const float smaller_area = std::min(
      lhs.size.x() * lhs.size.y(), rhs.size.x() * rhs.size.y());
  return smaller_area > 1.0e-6F
      ? std::clamp(intersection / smaller_area, 0.0F, 1.0F)
      : 0.0F;
}

bool cargoCandidateContainsHookAnchor(
    const CargoCandidateDescriptor& candidate,
    const Eigen::Vector2f& hook_center,
    float margin_m) {
  if (!validDescriptor(candidate) || !hook_center.allFinite() ||
      !std::isfinite(margin_m) || margin_m < 0.0F) {
    return false;
  }
  const Eigen::Vector2f delta =
      hook_center - candidate.center.head<2>();
  const float cosine = std::cos(candidate.yaw_rad);
  const float sine = std::sin(candidate.yaw_rad);
  const float local_long = cosine * delta.x() + sine * delta.y();
  const float local_short = -sine * delta.x() + cosine * delta.y();
  return std::abs(local_long) <= 0.5F * candidate.size.x() + margin_m &&
      std::abs(local_short) <= 0.5F * candidate.size.y() + margin_m;
}

CargoCandidateIdentityScore scoreCargoCandidateIdentity(
    const CargoCandidateDescriptor& candidate,
    const CargoCandidateIdentityContext& context) {
  CargoCandidateIdentityScore score;
  score.component_id = candidate.component_id;
  if (!validDescriptor(candidate) || !context.hook_center.allFinite() ||
      !std::isfinite(context.hook_region_radius_m) ||
      context.hook_region_radius_m <= 0.0F) {
    score.reason = "invalid_candidate_or_context";
    return score;
  }
  if (context.require_hook_containment &&
      !cargoCandidateContainsHookAnchor(
          candidate, context.hook_center,
          context.hook_containment_margin_m)) {
    score.reason = "hook_anchor_outside_candidate_obb";
    return score;
  }
  const float hook_center_distance =
      (candidate.center.head<2>() - context.hook_center).norm();
  if (std::isfinite(context.maximum_hook_center_distance_m) &&
      (context.maximum_hook_center_distance_m < 0.0F ||
       hook_center_distance > context.maximum_hook_center_distance_m)) {
    score.reason = "candidate_center_too_far_from_hook_anchor";
    return score;
  }
  const Eigen::Vector2f hook_delta =
      context.hook_center - candidate.center.head<2>();
  const float candidate_cosine = std::cos(candidate.yaw_rad);
  const float candidate_sine = std::sin(candidate.yaw_rad);
  const float normalized_long = std::abs(
      candidate_cosine * hook_delta.x() +
      candidate_sine * hook_delta.y()) /
      std::max(0.05F, 0.5F * candidate.size.x());
  const float normalized_short = std::abs(
      -candidate_sine * hook_delta.x() +
      candidate_cosine * hook_delta.y()) /
      std::max(0.05F, 0.5F * candidate.size.y());
  score.hook_normalized_offset =
      std::max(normalized_long, normalized_short);
  if (std::isfinite(context.maximum_hook_normalized_offset) &&
      (context.maximum_hook_normalized_offset <= 0.0F ||
       score.hook_normalized_offset >
           context.maximum_hook_normalized_offset)) {
    score.reason = "hook_anchor_outside_candidate_central_region";
    return score;
  }
  score.hook_distance_score = unitScore(
      hook_center_distance,
      context.hook_region_radius_m);
  score.point_support_confidence = std::clamp(
      static_cast<float>(candidate.point_count) /
          static_cast<float>(std::max<std::size_t>(1U,
                                                   context.strong_point_count)),
      0.0F, 1.0F);
  score.suspension_confidence = candidate.suspension_evidence ? 1.0F : 0.35F;
  score.shape_confidence = std::clamp(
      0.50F * candidate.orientation_confidence +
          0.50F * unitScore(candidate.size.y() / candidate.size.x(), 1.0F),
      0.0F, 1.0F);

  if (context.predicted_track_valid &&
      context.predicted_center.allFinite() &&
      context.predicted_size.allFinite() &&
      context.predicted_size.minCoeff() > 0.0F) {
    score.predicted_center_score = unitScore(
        (candidate.center.head<2>() -
         context.predicted_center.head<2>()).norm(),
        std::max(0.05F, context.association_radius_m));
    CargoCandidateDescriptor predicted = candidate;
    predicted.component_id = 0;
    predicted.center = context.predicted_center;
    predicted.size = context.predicted_size;
    predicted.yaw_rad = context.predicted_yaw_rad;
    predicted.orientation_confidence = 1.0F;
    predicted.point_count = 1U;
    score.overlap_score = cargoOrientedOverlapRatio(predicted, candidate);
    const float length_error = relativeError(
        candidate.size.x(), context.predicted_size.x());
    const float width_error = relativeError(
        candidate.size.y(), context.predicted_size.y());
    const float height_error = relativeError(
        candidate.size.z(), context.predicted_size.z());
    score.shape_confidence = std::min(
        score.shape_confidence,
        unitScore(std::max({length_error, width_error, height_error}), 0.75F));
    score.motion_confidence = score.predicted_center_score;
    score.identity_confidence =
        0.30F * score.predicted_center_score +
        0.25F * score.overlap_score +
        0.25F * score.shape_confidence +
        0.20F * unitScore(
            std::abs(normalizeCargoAxialYaw(
                candidate.yaw_rad - context.predicted_yaw_rad)),
            0.70F);
  } else {
    score.predicted_center_score = score.hook_distance_score;
    score.overlap_score = 0.0F;
    score.motion_confidence = score.hook_distance_score;
    score.identity_confidence =
        0.45F * score.hook_distance_score +
        0.25F * score.point_support_confidence +
        0.20F * score.suspension_confidence +
        0.10F * score.shape_confidence;
  }
  score.overall_lock_confidence = std::clamp(
      0.35F * score.identity_confidence +
          0.20F * score.shape_confidence +
          0.15F * candidate.orientation_confidence +
          0.15F * score.motion_confidence +
          0.15F * score.suspension_confidence,
      0.0F, 1.0F);
  score.valid = true;
  score.reason = "scored";
  return score;
}

CargoAssociationDecision evaluateCargoPredictedAssociation(
    const CargoAssociationInput& input) {
  CargoAssociationDecision decision;
  if (!validDescriptor(input.candidate) ||
      !input.previous_center.allFinite() || !input.velocity.allFinite() ||
      !input.locked_size.allFinite() || input.locked_size.minCoeff() <= 0.0F ||
      !std::isfinite(input.sensor_dt_sec) || input.sensor_dt_sec < 0.0 ||
      !std::isfinite(input.locked_yaw_rad) ||
      !std::isfinite(input.maximum_xy_gate_m) ||
      !std::isfinite(input.maximum_z_gate_m) ||
      input.maximum_xy_gate_m <= 0.0F || input.maximum_z_gate_m <= 0.0F) {
    decision.reason = "invalid_association_input";
    return decision;
  }
  const float dt = static_cast<float>(input.sensor_dt_sec);
  Eigen::Vector3f bounded_velocity = input.velocity;
  const float xy_speed = bounded_velocity.head<2>().norm();
  if (xy_speed > input.max_xy_speed_mps && xy_speed > 1.0e-6F) {
    bounded_velocity.head<2>() *= input.max_xy_speed_mps / xy_speed;
  }
  bounded_velocity.z() = std::clamp(
      bounded_velocity.z(), -input.max_z_speed_mps, input.max_z_speed_mps);
  decision.predicted_center = input.previous_center + bounded_velocity * dt;
  const float model_uncertainty = std::max(
      0.0F, input.velocity_model_uncertainty_mps) * dt;
  decision.dynamic_xy_gate_m = std::min(
      input.maximum_xy_gate_m,
      input.base_center_gate_m + model_uncertainty +
          input.horizontal_uncertainty_m +
          input.horizontal_tracking_residual_m);
  decision.dynamic_z_gate_m = std::min(
      input.maximum_z_gate_m,
      input.base_center_gate_m + model_uncertainty +
          input.vertical_uncertainty_m +
          input.vertical_tracking_residual_m);
  const Eigen::Vector3f center_residual =
      input.candidate.center - decision.predicted_center;
  decision.center_residual_xy_m = center_residual.head<2>().norm();
  decision.center_residual_z_m = std::abs(center_residual.z());
  if (decision.center_residual_xy_m > decision.dynamic_xy_gate_m) {
    decision.reason = "predicted_center_too_far";
    return decision;
  }
  if (decision.center_residual_z_m > decision.dynamic_z_gate_m) {
    decision.reason = "predicted_vertical_too_far";
    return decision;
  }

  CargoCandidateDescriptor predicted = input.candidate;
  predicted.component_id = 0;
  predicted.center = decision.predicted_center;
  predicted.size = input.locked_size;
  predicted.yaw_rad = input.locked_yaw_rad;
  predicted.orientation_confidence = 1.0F;
  predicted.point_count = 1U;
  decision.overlap_ratio = cargoOrientedOverlapRatio(predicted, input.candidate);
  decision.length_relative_error = relativeError(
      input.candidate.size.x(), input.locked_size.x());
  decision.width_relative_error = relativeError(
      input.candidate.size.y(), input.locked_size.y());
  decision.height_relative_error = relativeError(
      input.candidate.size.z(), input.locked_size.z());
  decision.axial_yaw_error_rad = std::abs(normalizeCargoAxialYaw(
      input.candidate.yaw_rad - input.locked_yaw_rad));
  decision.yaw_used_as_hard_gate = input.use_yaw_as_hard_gate;
  const float shape_limit = input.strict_reacquisition
      ? 0.75F * input.maximum_shape_relative_error
      : input.maximum_shape_relative_error;
  if (input.use_shape_as_hard_gate &&
      std::max({decision.length_relative_error,
                decision.width_relative_error,
                decision.height_relative_error}) > shape_limit) {
    decision.reason = "predicted_shape_mismatch";
    return decision;
  }
  const float yaw_limit = input.strict_reacquisition
      ? 0.75F * input.maximum_axial_yaw_error_rad
      : input.maximum_axial_yaw_error_rad;
  if (input.use_yaw_as_hard_gate &&
      decision.axial_yaw_error_rad > yaw_limit) {
    decision.reason = "predicted_yaw_mismatch";
    return decision;
  }
  const float overlap_limit = input.minimum_overlap_ratio;
  if (decision.overlap_ratio < overlap_limit) {
    decision.reason = "predicted_obb_overlap_low";
    return decision;
  }
  decision.accepted = true;
  decision.reason = input.strict_reacquisition
      ? "strict_reacquisition_match" : "predicted_obb_match";
  return decision;
}

CargoFrozenObbSupport evaluateCargoFrozenObbSupport(
    const CargoFrozenObbSupportInput& input) {
  CargoFrozenObbSupport support;
  if (!input.predicted_center.allFinite() ||
      !input.locked_size.allFinite() || input.locked_size.minCoeff() <= 0.0F ||
      !std::isfinite(input.locked_yaw_rad) ||
      !std::isfinite(input.horizontal_margin_m) ||
      !std::isfinite(input.vertical_margin_m) ||
      input.horizontal_margin_m < 0.0F || input.vertical_margin_m < 0.0F) {
    support.reason = "invalid_locked_obb_support_input";
    return support;
  }
  const float cosine = std::cos(input.locked_yaw_rad);
  const float sine = std::sin(input.locked_yaw_rad);
  float min_x = std::numeric_limits<float>::infinity();
  float max_x = -std::numeric_limits<float>::infinity();
  float min_y = std::numeric_limits<float>::infinity();
  float max_y = -std::numeric_limits<float>::infinity();
  float min_z = std::numeric_limits<float>::infinity();
  float max_z = -std::numeric_limits<float>::infinity();
  for (const Eigen::Vector3f& point : input.points) {
    if (!point.allFinite()) continue;
    ++support.finite_points;
    const Eigen::Vector3f delta = point - input.predicted_center;
    const float local_x = cosine * delta.x() + sine * delta.y();
    const float local_y = -sine * delta.x() + cosine * delta.y();
    const bool inside =
        std::abs(local_x) <= 0.5F * input.locked_size.x() +
            input.horizontal_margin_m &&
        std::abs(local_y) <= 0.5F * input.locked_size.y() +
            input.horizontal_margin_m &&
        std::abs(delta.z()) <= 0.5F * input.locked_size.z() +
            input.vertical_margin_m;
    if (!inside) continue;
    ++support.inside_points;
    min_x = std::min(min_x, local_x);
    max_x = std::max(max_x, local_x);
    min_y = std::min(min_y, local_y);
    max_y = std::max(max_y, local_y);
    min_z = std::min(min_z, delta.z());
    max_z = std::max(max_z, delta.z());
  }
  if (support.finite_points == 0U || support.inside_points == 0U) {
    support.reason = "no_locked_obb_point_support";
    return support;
  }
  support.inside_ratio = static_cast<float>(support.inside_points) /
      static_cast<float>(support.finite_points);
  support.long_axis_coverage_ratio = std::clamp(
      (max_x - min_x) / input.locked_size.x(), 0.0F, 1.0F);
  support.short_axis_coverage_ratio = std::clamp(
      (max_y - min_y) / input.locked_size.y(), 0.0F, 1.0F);
  support.vertical_coverage_ratio = std::clamp(
      (max_z - min_z) / input.locked_size.z(), 0.0F, 1.0F);
  support.valid = true;
  support.reason = "locked_obb_point_support";
  return support;
}

CargoProvisionalLockSummary summarizeCargoProvisionalLock(
    const std::vector<CargoCandidateDescriptor>& observations,
    const std::vector<CargoCandidateIdentityScore>& scores,
    const CargoProvisionalLockConfig& config) {
  CargoProvisionalLockSummary summary;
  if (observations.size() < config.minimum_frames ||
      observations.size() != scores.size()) {
    summary.reason = "insufficient_provisional_frames";
    return summary;
  }
  std::vector<float> xs, ys, zs, lengths, widths, heights, yaws;
  float identity_sum = 0.0F;
  float suspension_sum = 0.0F;
  float maximum_step = 0.0F;
  for (std::size_t i = 0U; i < observations.size(); ++i) {
    if (!validDescriptor(observations[i]) || !scores[i].valid) {
      summary.reason = "invalid_provisional_observation";
      return summary;
    }
    const auto& observation = observations[i];
    xs.push_back(observation.center.x());
    ys.push_back(observation.center.y());
    zs.push_back(observation.center.z());
    lengths.push_back(observation.size.x());
    widths.push_back(observation.size.y());
    heights.push_back(observation.size.z());
    yaws.push_back(observation.yaw_rad);
    identity_sum += scores[i].identity_confidence;
    suspension_sum += scores[i].suspension_confidence;
    if (i > 0U) {
      maximum_step = std::max(
          maximum_step,
          (observation.center - observations[i - 1U].center).norm());
    }
  }
  summary.median_center = Eigen::Vector3f(
      median(xs), median(ys), median(zs));
  summary.median_size = Eigen::Vector3f(
      median(lengths), median(widths), median(heights));
  if (!summary.median_center.allFinite() ||
      !summary.median_size.allFinite() ||
      !std::isfinite(config.minimum_length_m) ||
      !std::isfinite(config.minimum_width_m) ||
      !std::isfinite(config.minimum_height_m) ||
      config.minimum_length_m <= 0.0F ||
      config.minimum_width_m <= 0.0F ||
      config.minimum_height_m <= 0.0F ||
      summary.median_size.x() < config.minimum_length_m ||
      summary.median_size.y() < config.minimum_width_m ||
      summary.median_size.z() < config.minimum_height_m ||
      summary.median_size.x() < summary.median_size.y()) {
    summary.reason = "formal_shape_out_of_physical_bounds";
    return summary;
  }
  const CargoAxialYawSummary yaw = summarizeCargoAxialYaw(yaws);
  summary.axial_yaw_rad = yaw.mean_yaw_rad;
  summary.orientation_confidence = yaw.concentration;
  const float length_cv = median(lengths) > 1.0e-6F
      ? median([&]() {
          std::vector<float> deviations;
          for (float value : lengths) deviations.push_back(
              std::abs(value - summary.median_size.x()));
          return deviations;
        }()) / summary.median_size.x() : 1.0F;
  const float width_cv = median(widths) > 1.0e-6F
      ? median([&]() {
          std::vector<float> deviations;
          for (float value : widths) deviations.push_back(
              std::abs(value - summary.median_size.y()));
          return deviations;
        }()) / summary.median_size.y() : 1.0F;
  const float height_cv = median(heights) > 1.0e-6F
      ? median([&]() {
          std::vector<float> deviations;
          for (float value : heights) deviations.push_back(
              std::abs(value - summary.median_size.z()));
          return deviations;
        }()) / summary.median_size.z() : 1.0F;
  summary.shape_confidence = unitScore(
      std::max({length_cv, width_cv, height_cv}),
      std::max(1.0e-3F, config.maximum_shape_cv));
  summary.motion_confidence = unitScore(
      maximum_step, std::max(1.0e-3F, config.maximum_center_step_m));
  summary.identity_confidence = identity_sum /
      static_cast<float>(observations.size());
  summary.suspension_confidence = suspension_sum /
      static_cast<float>(observations.size());
  summary.overall_lock_confidence = std::clamp(
      0.30F * summary.identity_confidence +
          0.25F * summary.shape_confidence +
          0.20F * summary.orientation_confidence +
          0.15F * summary.motion_confidence +
          0.10F * summary.suspension_confidence,
      0.0F, 1.0F);
  if (!yaw.valid ||
      summary.orientation_confidence <
          config.minimum_orientation_concentration) {
    summary.reason = "orientation_not_concentrated";
    return summary;
  }
  if (summary.identity_confidence < config.minimum_identity_confidence) {
    summary.reason = "identity_confidence_low";
    return summary;
  }
  if (std::max({length_cv, width_cv, height_cv}) >
      config.maximum_shape_cv) {
    summary.reason = "shape_spread_high";
    return summary;
  }
  if (maximum_step > config.maximum_center_step_m) {
    summary.reason = "motion_discontinuous";
    return summary;
  }
  if (summary.overall_lock_confidence <
      config.minimum_overall_lock_confidence) {
    summary.reason = "overall_lock_confidence_low";
    return summary;
  }
  summary.formal_lock_allowed = true;
  summary.reason = "formal_lock_confirmed";
  return summary;
}

}  // namespace ndt_slam
