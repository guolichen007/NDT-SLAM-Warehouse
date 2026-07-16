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

float cargoAxialYawDifference(float lhs_rad, float rhs_rad) {
    return std::abs(normalizeCargoAxialYaw(lhs_rad - rhs_rad));
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
    float uncertainty_m) {
    RigidCargoGeometry result;
    result.track_id = track_id;
    result.shape = shape;
    result.pose = live_pose;
    if (!validShape(shape)) {
        result.reason = "invalid_locked_shape";
        return result;
    }
    if (!live_pose.valid || !live_pose.center_base.allFinite() ||
        !std::isfinite(live_pose.stamp_sec) || live_pose.stamp_sec <= 0.0) {
        result.reason = "invalid_live_pose";
        return result;
    }
    if (!validTransform(T_map_base)) {
        result.reason = "invalid_map_transform";
        return result;
    }
    if (!std::isfinite(uncertainty_m) || uncertainty_m < 0.0F) {
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
    result.geometry_uncertainty_m = uncertainty_m;
    result.valid = true;
    result.reason = "locked_shape_live_pose";
    return result;
}

CargoObbFootprint toCargoObbFootprint(
    const RigidCargoGeometry& geometry) {
    CargoObbFootprint footprint;
    if (!geometry.valid) return footprint;
    footprint.valid = true;
    footprint.center_base = geometry.pose.center_base.head<2>();
    footprint.length_m = geometry.shape.length_m;
    footprint.width_m = geometry.shape.width_m;
    footprint.yaw_base_rad = geometry.shape.yaw_base_rad;
    footprint.min_z = geometry.bottom_z_base;
    footprint.max_z = geometry.top_z_base;
    return footprint;
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
