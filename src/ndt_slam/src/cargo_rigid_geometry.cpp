#include "ndt_slam/cargo_rigid_geometry.hpp"

#include "ndt_slam/cargo_oriented_footprint.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ndt_slam {
namespace {

bool validShape(const LockedCargoShape& shape) {
    return shape.valid && std::isfinite(shape.length_m) &&
        std::isfinite(shape.width_m) && std::isfinite(shape.height_m) &&
        std::isfinite(shape.yaw_base_rad) &&
        std::isfinite(shape.orientation_confidence) &&
        shape.length_m >= shape.width_m && shape.width_m > 0.0F &&
        shape.height_m > 0.0F && shape.orientation_confidence >= 0.0F &&
        shape.orientation_confidence <= 1.0F;
}

bool validTransform(const Eigen::Isometry3f& transform) {
    if (!transform.matrix().allFinite()) return false;
    const Eigen::Matrix3f rotation = transform.linear();
    return (rotation.transpose() * rotation - Eigen::Matrix3f::Identity())
                   .cwiseAbs().maxCoeff() <= 1.0e-3F &&
        std::abs(rotation.determinant() - 1.0F) <= 1.0e-3F;
}

}  // namespace

const char* cargoPoseSourceName(CargoPoseSource source) noexcept {
    switch (source) {
        case CargoPoseSource::CURRENT_ASSOCIATED_LIDAR:
            return "CURRENT_ASSOCIATED_LIDAR";
        case CargoPoseSource::HOOK_OR_CONTROLLER:
            return "HOOK_OR_CONTROLLER";
        case CargoPoseSource::MOTION_PREDICTION:
            return "MOTION_PREDICTION";
        case CargoPoseSource::RECENT_STABLE_HOLD:
            return "RECENT_STABLE_HOLD";
        case CargoPoseSource::STATIC_INSTALLATION_FALLBACK:
            return "STATIC_INSTALLATION_FALLBACK";
    }
    return "UNKNOWN";
}

const char* cargoVerticalPoseSourceName(
    CargoVerticalPoseSource source) noexcept {
    switch (source) {
        case CargoVerticalPoseSource::DIRECT_BOTTOM:
            return "DIRECT_BOTTOM";
        case CargoVerticalPoseSource::DIRECT_TOP:
            return "SUPPORTED_TOP_MINUS_FROZEN_HEIGHT";
        case CargoVerticalPoseSource::LOCKED_OBB_POINT_SUPPORT:
            return "LOCKED_OBB_POINT_SUPPORT";
        case CargoVerticalPoseSource::PROVISIONAL_MEDIAN:
            return "FRESH_HELD_FORMAL";
        case CargoVerticalPoseSource::PREDICTION:
            return "PREDICTION";
        case CargoVerticalPoseSource::DISPLAY_FROZEN:
            return "DISPLAY_FROZEN";
    }
    return "DISPLAY_FROZEN";
}

float cargoAxialYawDifference(float lhs_rad, float rhs_rad) {
    return std::abs(normalizeCargoAxialYaw(lhs_rad - rhs_rad));
}

Eigen::Vector3f limitCargoPoseResidualByRate(
    const Eigen::Vector3f& residual,
    double sensor_dt_sec,
    float max_xy_speed_mps,
    float max_z_speed_mps,
    float step_margin_m) {
    if (!residual.allFinite() || !std::isfinite(sensor_dt_sec) ||
        sensor_dt_sec <= 0.0 || !std::isfinite(max_xy_speed_mps) ||
        !std::isfinite(max_z_speed_mps) || !std::isfinite(step_margin_m) ||
        max_xy_speed_mps < 0.0F || max_z_speed_mps < 0.0F ||
        step_margin_m < 0.0F) {
        return Eigen::Vector3f::Zero();
    }
    Eigen::Vector3f limited = residual;
    const float dt = static_cast<float>(sensor_dt_sec);
    const float max_xy_step = max_xy_speed_mps * dt + step_margin_m;
    const float max_z_step = max_z_speed_mps * dt + step_margin_m;
    const float xy_norm = limited.head<2>().norm();
    if (xy_norm > max_xy_step && xy_norm > 1.0e-6F) {
        limited.head<2>() *= max_xy_step / xy_norm;
    }
    limited.z() = std::clamp(limited.z(), -max_z_step, max_z_step);
    return limited;
}

CargoLivePoseStepResult updateCargoLivePoseStep(
    const CargoLivePoseStepInput& input) {
    CargoLivePoseStepResult result;
    if (!input.previous_center.allFinite() ||
        !input.previous_velocity.allFinite() ||
        !input.measured_center.allFinite() ||
        !std::isfinite(input.sensor_dt_sec) ||
        input.sensor_dt_sec <= 0.0 ||
        !std::isfinite(input.center_alpha) ||
        input.center_alpha < 0.0F || input.center_alpha > 1.0F ||
        !std::isfinite(input.velocity_alpha) ||
        input.velocity_alpha < 0.0F || input.velocity_alpha > 1.0F ||
        !std::isfinite(input.max_xy_speed_mps) ||
        input.max_xy_speed_mps < 0.0F ||
        !std::isfinite(input.max_z_speed_mps) ||
        input.max_z_speed_mps < 0.0F ||
        !std::isfinite(input.step_margin_m) ||
        input.step_margin_m < 0.0F) {
        return result;
    }

    const Eigen::Vector3f bounded_previous_velocity =
        limitCargoPoseResidualByRate(
            input.previous_velocity, 1.0,
            input.max_xy_speed_mps, input.max_z_speed_mps, 0.0F);
    const float dt = static_cast<float>(input.sensor_dt_sec);
    result.predicted_center = input.previous_center +
        bounded_previous_velocity * dt;
    result.measurement_residual =
        input.measured_center - result.predicted_center;
    const Eigen::Vector3f limited_residual =
        limitCargoPoseResidualByRate(
            result.measurement_residual, input.sensor_dt_sec,
            input.max_xy_speed_mps, input.max_z_speed_mps,
            input.step_margin_m);
    const Eigen::Vector3f proposed_center = result.predicted_center +
        input.center_alpha * limited_residual;
    const Eigen::Vector3f final_step = limitCargoPoseResidualByRate(
        proposed_center - input.previous_center, input.sensor_dt_sec,
        input.max_xy_speed_mps, input.max_z_speed_mps,
        input.step_margin_m);
    result.filtered_center = input.previous_center + final_step;
    result.tracking_residual =
        input.measured_center - result.filtered_center;

    const Eigen::Vector3f observed_velocity = final_step / dt;
    const Eigen::Vector3f blended_velocity =
        (1.0F - input.velocity_alpha) * bounded_previous_velocity +
        input.velocity_alpha * observed_velocity;
    result.filtered_velocity = limitCargoPoseResidualByRate(
        blended_velocity, 1.0,
        input.max_xy_speed_mps, input.max_z_speed_mps, 0.0F);
    result.valid = result.predicted_center.allFinite() &&
        result.measurement_residual.allFinite() &&
        result.filtered_center.allFinite() &&
        result.tracking_residual.allFinite() &&
        result.filtered_velocity.allFinite();
    return result;
}

LiveCargoPose propagateHeldCargoPose(
    const LiveCargoPose& pose,
    const Eigen::Vector3f& velocity_base,
    double evaluation_stamp_sec,
    double max_prediction_sec,
    float max_xy_speed_mps,
    float max_z_speed_mps,
    double velocity_decay_tau_sec,
    float uncertainty_per_sec,
    float max_uncertainty_m) {
    LiveCargoPose result;
    if (!pose.valid || !pose.center_base.allFinite() ||
        !velocity_base.allFinite() ||
        !std::isfinite(pose.evidence_stamp_sec) ||
        pose.evidence_stamp_sec <= 0.0 ||
        !std::isfinite(evaluation_stamp_sec) ||
        evaluation_stamp_sec + 1.0e-4 < pose.evidence_stamp_sec ||
        !std::isfinite(max_prediction_sec) || max_prediction_sec < 0.0 ||
        !std::isfinite(max_xy_speed_mps) || max_xy_speed_mps < 0.0F ||
        !std::isfinite(max_z_speed_mps) || max_z_speed_mps < 0.0F ||
        !std::isfinite(velocity_decay_tau_sec) ||
        velocity_decay_tau_sec <= 0.0 ||
        !std::isfinite(uncertainty_per_sec) || uncertainty_per_sec < 0.0F ||
        !std::isfinite(max_uncertainty_m) || max_uncertainty_m < 0.0F ||
        !std::isfinite(pose.position_uncertainty_m) ||
        pose.position_uncertainty_m < 0.0F) {
        return result;
    }

    result = pose;
    const double age_sec = std::max(
        0.0, evaluation_stamp_sec - pose.evidence_stamp_sec);
    const double prediction_sec = std::min(age_sec, max_prediction_sec);
    const Eigen::Vector3f bounded_velocity = limitCargoPoseResidualByRate(
        velocity_base, 1.0, max_xy_speed_mps, max_z_speed_mps, 0.0F);
    if (prediction_sec > 0.0 && bounded_velocity.norm() > 1.0e-6F) {
        const double decayed_prediction_sec = velocity_decay_tau_sec *
            (1.0 - std::exp(-prediction_sec / velocity_decay_tau_sec));
        result.center_base += bounded_velocity *
            static_cast<float>(decayed_prediction_sec);
        if (age_sec <= max_prediction_sec + 1.0e-4) {
            result.source = CargoPoseSource::MOTION_PREDICTION;
            result.vertical_source = CargoVerticalPoseSource::PREDICTION;
        } else {
            result.source = CargoPoseSource::RECENT_STABLE_HOLD;
            result.vertical_source = CargoVerticalPoseSource::DISPLAY_FROZEN;
        }
    } else {
        result.source = CargoPoseSource::RECENT_STABLE_HOLD;
        result.vertical_source = CargoVerticalPoseSource::DISPLAY_FROZEN;
    }
    result.evaluation_stamp_sec = evaluation_stamp_sec;
    const float grown_uncertainty = pose.position_uncertainty_m +
        static_cast<float>(age_sec) * uncertainty_per_sec;
    result.position_uncertainty_m = std::max(
        pose.position_uncertainty_m,
        std::min(max_uncertainty_m, grown_uncertainty));
    return result;
}

std::array<Eigen::Vector3f, 8> buildCargoObbCornersBase(
    const LockedCargoShape& shape,
    const Eigen::Vector3f& center_base) {
    std::array<Eigen::Vector3f, 8> corners{};
    if (!validShape(shape) || !center_base.allFinite()) return corners;
    const float cosine = std::cos(shape.yaw_base_rad);
    const float sine = std::sin(shape.yaw_base_rad);
    const float half_length = 0.5F * shape.length_m;
    const float half_width = 0.5F * shape.width_m;
    const float half_height = 0.5F * shape.height_m;
    std::size_t index = 0U;
    for (int z_side = 0; z_side < 2; ++z_side) {
        const float z = center_base.z() +
            (z_side == 0 ? -half_height : half_height);
        for (int y_side = 0; y_side < 2; ++y_side) {
            for (int x_side = 0; x_side < 2; ++x_side) {
                const float local_x = x_side == 0 ? -half_length : half_length;
                const float local_y = y_side == 0 ? -half_width : half_width;
                corners[index++] = Eigen::Vector3f(
                    center_base.x() + cosine * local_x - sine * local_y,
                    center_base.y() + sine * local_x + cosine * local_y,
                    z);
            }
        }
    }
    return corners;
}

RigidCargoGeometry buildCurrentRigidCargoGeometry(
    const LockedCargoShape& shape,
    const LiveCargoPose& live_pose,
    const Eigen::Isometry3f& T_map_base,
    std::uint64_t track_id,
    float horizontal_uncertainty_m,
    float vertical_uncertainty_m) {
    RigidCargoGeometry result;
    result.track_id = track_id;
    result.shape = shape;
    result.pose = live_pose;
    if (!validShape(shape)) {
        result.reason = "invalid_locked_shape";
        return result;
    }
    if (!live_pose.valid || !live_pose.center_base.allFinite() ||
        !std::isfinite(live_pose.evidence_stamp_sec) ||
        live_pose.evidence_stamp_sec <= 0.0 ||
        !std::isfinite(live_pose.evaluation_stamp_sec) ||
        live_pose.evaluation_stamp_sec <= 0.0) {
        result.reason = "invalid_live_pose";
        return result;
    }
    if (!validTransform(T_map_base)) {
        result.reason = "invalid_map_transform";
        return result;
    }
    if (!std::isfinite(horizontal_uncertainty_m) ||
        horizontal_uncertainty_m < 0.0F ||
        !std::isfinite(vertical_uncertainty_m) ||
        vertical_uncertainty_m < 0.0F) {
        result.reason = "invalid_uncertainty";
        return result;
    }

    result.bottom_z_base =
        live_pose.center_base.z() - 0.5F * shape.height_m;
    result.top_z_base =
        live_pose.center_base.z() + 0.5F * shape.height_m;
    result.corners_base = buildCargoObbCornersBase(
        shape, live_pose.center_base);
    result.aabb_min_base = result.corners_base.front();
    result.aabb_max_base = result.corners_base.front();
    for (std::size_t i = 0U; i < result.corners_base.size(); ++i) {
        result.corners_map[i] = T_map_base * result.corners_base[i];
        result.aabb_min_base = result.aabb_min_base.cwiseMin(
            result.corners_base[i]);
        result.aabb_max_base = result.aabb_max_base.cwiseMax(
            result.corners_base[i]);
        if (i == 0U) {
            result.aabb_min_map = result.corners_map[i];
            result.aabb_max_map = result.corners_map[i];
        } else {
            result.aabb_min_map = result.aabb_min_map.cwiseMin(
                result.corners_map[i]);
            result.aabb_max_map = result.aabb_max_map.cwiseMax(
                result.corners_map[i]);
        }
    }
    result.horizontal_uncertainty_m = horizontal_uncertainty_m;
    result.vertical_uncertainty_m = vertical_uncertainty_m;
    result.pose_evidence_stamp_sec = live_pose.evidence_stamp_sec;
    result.height_evidence_stamp_sec = live_pose.evidence_stamp_sec;
    result.evaluation_stamp_sec = live_pose.evaluation_stamp_sec;
    result.valid = true;
    result.reason = "locked_shape_live_pose";
    return result;
}

CargoObbFootprint toCargoObbFootprint(
    const RigidCargoGeometry& geometry,
    float horizontal_expansion_m) {
    CargoObbFootprint footprint;
    if (!geometry.valid || !std::isfinite(horizontal_expansion_m) ||
        horizontal_expansion_m < 0.0F) return footprint;
    footprint.valid = true;
    footprint.center_base = geometry.pose.center_base.head<2>();
    footprint.length_m = geometry.shape.length_m +
        2.0F * horizontal_expansion_m;
    footprint.width_m = geometry.shape.width_m +
        2.0F * horizontal_expansion_m;
    footprint.yaw_base_rad = geometry.shape.yaw_base_rad;
    footprint.min_z = geometry.bottom_z_base;
    footprint.max_z = geometry.top_z_base;
    return footprint;
}

CargoFormalUseDecision evaluateCargoFormalUse(
    bool geometry_valid,
    bool lost_hold,
    double evaluation_stamp_sec,
    double pose_evidence_stamp_sec,
    double height_evidence_stamp_sec,
    double formal_xy_evidence_hold_sec,
    double formal_vertical_evidence_hold_sec,
    float horizontal_uncertainty_m) {
    CargoFormalUseDecision decision;
    if (!geometry_valid || !std::isfinite(evaluation_stamp_sec) ||
        !std::isfinite(pose_evidence_stamp_sec) ||
        !std::isfinite(height_evidence_stamp_sec) ||
        !std::isfinite(formal_xy_evidence_hold_sec) ||
        formal_xy_evidence_hold_sec < 0.0 ||
        !std::isfinite(formal_vertical_evidence_hold_sec) ||
        formal_vertical_evidence_hold_sec < 0.0 ||
        !std::isfinite(horizontal_uncertainty_m) ||
        horizontal_uncertainty_m < 0.0F) {
        decision.reason = "invalid_geometry_or_evidence_time";
        return decision;
    }
    decision.pose_age_sec = evaluation_stamp_sec - pose_evidence_stamp_sec;
    decision.height_age_sec = evaluation_stamp_sec - height_evidence_stamp_sec;
    if (decision.pose_age_sec < -1.0e-4 ||
        decision.height_age_sec < -1.0e-4) {
        decision.reason = "future_cargo_evidence";
        return decision;
    }
    decision.display_valid = true;
    decision.horizontal_uncertainty_m = horizontal_uncertainty_m;
    // Frozen physical thickness does not expire, but both horizontal pose and
    // live vertical position require their own recent physical evidence.
    const bool xy_within_hold = decision.pose_age_sec <=
        formal_xy_evidence_hold_sec + 1.0e-4;
    const bool vertical_within_hold = decision.height_age_sec <=
        formal_vertical_evidence_hold_sec + 1.0e-4;
    const bool within_hold = xy_within_hold && vertical_within_hold;
    decision.formal_safety_valid = within_hold;
    decision.formal_removal_valid = within_hold;
    if (within_hold) {
        decision.reason = lost_hold
            ? "lost_hold_formal_window"
            : "locked_formal_window";
    } else if (!vertical_within_hold) {
        decision.reason = "formal_vertical_evidence_expired";
    } else if (!xy_within_hold) {
        decision.reason = "formal_xy_evidence_expired";
    } else {
        decision.reason = lost_hold
            ? "lost_hold_display_only_evidence_expired"
            : "locked_display_only_evidence_expired";
    }
    return decision;
}

CargoFormalHeightDecision evaluateCargoFormalHeight(
    const CargoBottomResult& fusion,
    const RigidCargoGeometry& rigid,
    const CargoFormalUseDecision& lifetime) {
    CargoFormalHeightDecision decision;
    if (!rigid.valid || !std::isfinite(rigid.bottom_z_base) ||
        !std::isfinite(rigid.top_z_base) ||
        rigid.top_z_base <= rigid.bottom_z_base) {
        decision.reason = "formal_rigid_geometry_invalid";
        return decision;
    }
    if (!lifetime.formal_safety_valid) {
        decision.reason = lifetime.reason;
        return decision;
    }
    if (!std::isfinite(rigid.height_evidence_stamp_sec) ||
        rigid.height_evidence_stamp_sec <= 0.0) {
        decision.reason = "formal_vertical_evidence_stamp_invalid";
        return decision;
    }

    decision.bottom_z_base = rigid.bottom_z_base;
    decision.top_z_base = rigid.top_z_base;
    decision.evidence_stamp_sec = rigid.height_evidence_stamp_sec;
    decision.uncertainty_m = std::max(
        rigid.vertical_uncertainty_m,
        fusion.valid && std::isfinite(fusion.uncertainty)
            ? fusion.uncertainty : 0.0F);
    if (!std::isfinite(decision.uncertainty_m) ||
        decision.uncertainty_m < 0.0F) {
        decision.reason = "formal_vertical_uncertainty_invalid";
        return decision;
    }

    if (fusion.valid) {
        decision.source = fusion.source;
        decision.confidence = std::clamp(fusion.confidence, 0.0F, 1.0F);
        decision.reason = std::string("formal_height:") + fusion.reason;
    } else {
        // The rigid lifetime may outlive the estimator's short display hold.
        // This is still the same physical evidence stamp, never a refreshed
        // sample, and expires at the formal vertical lifetime gate above.
        decision.source = CargoBottomSource::RECENT_STABLE;
        decision.confidence = 0.35F;
        decision.reason = std::string("formal_height_hold:") +
            lifetime.reason;
    }
    decision.valid = true;
    return decision;
}

bool containsPointInCargoObbBase(
    const Eigen::Vector3f& point_base,
    const CargoObbFootprint& footprint,
    float margin_xy_m,
    float margin_z_m) {
    if (!point_base.allFinite() || !footprint.valid ||
        !footprint.center_base.allFinite() ||
        !std::isfinite(footprint.length_m) ||
        !std::isfinite(footprint.width_m) ||
        !std::isfinite(footprint.yaw_base_rad) ||
        !std::isfinite(footprint.min_z) ||
        !std::isfinite(footprint.max_z) ||
        footprint.length_m <= 0.0F || footprint.width_m <= 0.0F ||
        footprint.max_z <= footprint.min_z) {
        return false;
    }
    const float margin_xy = std::max(0.0F, margin_xy_m);
    const float margin_z = std::max(0.0F, margin_z_m);
    const Eigen::Vector2f delta = point_base.head<2>() -
        footprint.center_base;
    const float cosine = std::cos(footprint.yaw_base_rad);
    const float sine = std::sin(footprint.yaw_base_rad);
    const float local_x = cosine * delta.x() + sine * delta.y();
    const float local_y = -sine * delta.x() + cosine * delta.y();
    return std::abs(local_x) <= 0.5F * footprint.length_m + margin_xy &&
        std::abs(local_y) <= 0.5F * footprint.width_m + margin_xy &&
        point_base.z() >= footprint.min_z - margin_z &&
        point_base.z() <= footprint.max_z + margin_z;
}

bool containsPointInSweptCargoObbBase(
    const Eigen::Vector3f& point_base,
    const CargoObbFootprint& start,
    const CargoObbFootprint& finish,
    float margin_xy_m,
    float margin_z_m,
    float maximum_sample_step_m) {
    if (!start.valid || !finish.valid || !point_base.allFinite() ||
        !std::isfinite(maximum_sample_step_m) ||
        maximum_sample_step_m <= 0.0F ||
        cargoAxialYawDifference(
            start.yaw_base_rad, finish.yaw_base_rad) > 1.0e-3F) {
        return false;
    }
    const float center_distance =
        (finish.center_base - start.center_base).norm();
    const float vertical_distance = std::max(
        std::abs(finish.min_z - start.min_z),
        std::abs(finish.max_z - start.max_z));
    const int steps = std::clamp(
        static_cast<int>(std::ceil(
            std::max(center_distance, vertical_distance) /
            maximum_sample_step_m)),
        1, 24);
    for (int index = 0; index <= steps; ++index) {
        const float alpha = static_cast<float>(index) /
            static_cast<float>(steps);
        CargoObbFootprint sample = start;
        sample.center_base =
            (1.0F - alpha) * start.center_base +
            alpha * finish.center_base;
        sample.length_m =
            (1.0F - alpha) * start.length_m + alpha * finish.length_m;
        sample.width_m =
            (1.0F - alpha) * start.width_m + alpha * finish.width_m;
        sample.min_z = (1.0F - alpha) * start.min_z + alpha * finish.min_z;
        sample.max_z = (1.0F - alpha) * start.max_z + alpha * finish.max_z;
        if (containsPointInCargoObbBase(
                point_base, sample, margin_xy_m, margin_z_m)) {
            return true;
        }
    }
    return false;
}

bool isCargoIdentityPointMatch(
    float nearest_distance_squared,
    float match_radius_m,
    bool inside_cargo_neighborhood) {
    return inside_cargo_neighborhood &&
        std::isfinite(nearest_distance_squared) &&
        nearest_distance_squared >= 0.0F &&
        std::isfinite(match_radius_m) && match_radius_m > 0.0F &&
        nearest_distance_squared <= match_radius_m * match_radius_m;
}

float pointToCargoObbDistance2D(
    const Eigen::Vector2f& point_base,
    const CargoObbFootprint& footprint) {
    if (!point_base.allFinite() || !footprint.valid ||
        !footprint.center_base.allFinite() ||
        footprint.length_m <= 0.0F || footprint.width_m <= 0.0F ||
        !std::isfinite(footprint.yaw_base_rad)) {
        return std::numeric_limits<float>::infinity();
    }
    const Eigen::Vector2f delta = point_base - footprint.center_base;
    const float cosine = std::cos(footprint.yaw_base_rad);
    const float sine = std::sin(footprint.yaw_base_rad);
    const float local_x = cosine * delta.x() + sine * delta.y();
    const float local_y = -sine * delta.x() + cosine * delta.y();
    const float dx = std::max(std::abs(local_x) -
                                  0.5F * footprint.length_m,
                              0.0F);
    const float dy = std::max(std::abs(local_y) -
                                  0.5F * footprint.width_m,
                              0.0F);
    return std::hypot(dx, dy);
}

}  // namespace ndt_slam
