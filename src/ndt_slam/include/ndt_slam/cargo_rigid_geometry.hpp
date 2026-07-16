#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstdint>
#include <string>

namespace ndt_slam {

struct LockedCargoShape {
    bool valid = false;
    float length_m = 0.0F;
    float width_m = 0.0F;
    float height_m = 0.0F;
    float yaw_base_rad = 0.0F;
    float orientation_confidence = 0.0F;
};

enum class CargoPoseSource : std::uint8_t {
    CURRENT_ASSOCIATED_LIDAR = 0,
    HOOK_OR_CONTROLLER = 1,
    MOTION_PREDICTION = 2,
    RECENT_STABLE_HOLD = 3,
    STATIC_INSTALLATION_FALLBACK = 4,
};

const char* cargoPoseSourceName(CargoPoseSource source) noexcept;

struct LiveCargoPose {
    bool valid = false;
    Eigen::Vector3f center_base = Eigen::Vector3f::Zero();
    double stamp_sec = 0.0;
    CargoPoseSource source = CargoPoseSource::RECENT_STABLE_HOLD;
    float position_uncertainty_m = 0.0F;
};

struct CargoObbFootprint {
    bool valid = false;
    Eigen::Vector2f center_base = Eigen::Vector2f::Zero();
    float length_m = 0.0F;
    float width_m = 0.0F;
    float yaw_base_rad = 0.0F;
    float min_z = 0.0F;
    float max_z = 0.0F;
};

struct RigidCargoGeometry {
    bool valid = false;
    std::uint64_t track_id = 0U;
    LockedCargoShape shape;
    LiveCargoPose pose;
    float bottom_z_base = 0.0F;
    float top_z_base = 0.0F;
    std::array<Eigen::Vector3f, 8> corners_base{};
    std::array<Eigen::Vector3f, 8> corners_map{};
    Eigen::Vector3f aabb_min_base = Eigen::Vector3f::Zero();
    Eigen::Vector3f aabb_max_base = Eigen::Vector3f::Zero();
    Eigen::Vector3f aabb_min_map = Eigen::Vector3f::Zero();
    Eigen::Vector3f aabb_max_map = Eigen::Vector3f::Zero();
    float geometry_uncertainty_m = 0.0F;
    std::string reason = "not_built";
};

RigidCargoGeometry buildCurrentRigidCargoGeometry(
    const LockedCargoShape& shape,
    const LiveCargoPose& live_pose,
    const Eigen::Isometry3f& T_map_base,
    std::uint64_t track_id,
    float uncertainty_m);

std::array<Eigen::Vector3f, 8> buildCargoObbCornersBase(
    const LockedCargoShape& shape,
    const Eigen::Vector3f& center_base);

CargoObbFootprint toCargoObbFootprint(
    const RigidCargoGeometry& geometry);

bool containsPointInCargoObbBase(
    const Eigen::Vector3f& point_base,
    const CargoObbFootprint& footprint,
    float margin_xy_m = 0.0F,
    float margin_z_m = 0.0F);

float pointToCargoObbDistance2D(
    const Eigen::Vector2f& point_base,
    const CargoObbFootprint& footprint);

float cargoAxialYawDifference(float lhs_rad, float rhs_rad);

}  // namespace ndt_slam
