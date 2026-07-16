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

enum class CargoVerticalPoseSource : std::uint8_t {
    DIRECT_BOTTOM = 0,
    DIRECT_TOP = 1,
    LOCKED_OBB_POINT_SUPPORT = 2,
    PREDICTION = 3,
    DISPLAY_FROZEN = 4
};

const char* cargoVerticalPoseSourceName(
    CargoVerticalPoseSource source) noexcept;

struct LiveCargoPose {
    bool valid = false;
    Eigen::Vector3f center_base = Eigen::Vector3f::Zero();
    // Timestamp of the physical observation/controller sample that last
    // advanced this pose. It is never refreshed by a display/evaluation tick.
    double evidence_stamp_sec = 0.0;
    // Timestamp at which this pose (possibly predicted/held) was evaluated.
    double evaluation_stamp_sec = 0.0;
    CargoPoseSource source = CargoPoseSource::RECENT_STABLE_HOLD;
    CargoVerticalPoseSource vertical_source =
        CargoVerticalPoseSource::DISPLAY_FROZEN;
    float position_uncertainty_m = 0.0F;
};

struct CargoLivePoseStepInput {
    Eigen::Vector3f previous_center = Eigen::Vector3f::Zero();
    Eigen::Vector3f previous_velocity = Eigen::Vector3f::Zero();
    Eigen::Vector3f measured_center = Eigen::Vector3f::Zero();
    double sensor_dt_sec = 0.0;
    float center_alpha = 0.0F;
    float velocity_alpha = 0.0F;
    float max_xy_speed_mps = 0.0F;
    float max_z_speed_mps = 0.0F;
    float step_margin_m = 0.0F;
};

struct CargoLivePoseStepResult {
    bool valid = false;
    Eigen::Vector3f predicted_center = Eigen::Vector3f::Zero();
    Eigen::Vector3f measurement_residual = Eigen::Vector3f::Zero();
    Eigen::Vector3f filtered_center = Eigen::Vector3f::Zero();
    // Residual left after the bounded filter correction.  This is the
    // uncertainty that the formal safety footprint must cover.
    Eigen::Vector3f tracking_residual = Eigen::Vector3f::Zero();
    Eigen::Vector3f filtered_velocity = Eigen::Vector3f::Zero();
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
    double pose_evidence_stamp_sec = 0.0;
    double height_evidence_stamp_sec = 0.0;
    double evaluation_stamp_sec = 0.0;
    std::string reason = "not_built";
};

struct CargoFormalUseDecision {
    bool display_valid = false;
    bool formal_safety_valid = false;
    bool formal_removal_valid = false;
    double pose_age_sec = 0.0;
    double height_age_sec = 0.0;
    float horizontal_uncertainty_m = 0.0F;
    std::string reason = "not_evaluated";
};

// Display retention and formal safety/removal authority are deliberately
// separate. A retained marker may remain visible after physical evidence has
// become too old to produce code 14/17/18 or authorize map removal, even when
// the lifecycle state has not yet transitioned from LOCKED to LOST_HOLD.
CargoFormalUseDecision evaluateCargoFormalUse(
    bool geometry_valid,
    bool lost_hold,
    double evaluation_stamp_sec,
    double pose_evidence_stamp_sec,
    double height_evidence_stamp_sec,
    double formal_hold_sec,
    float horizontal_uncertainty_m);

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
    const RigidCargoGeometry& geometry,
    float horizontal_expansion_m = 0.0F);

bool containsPointInCargoObbBase(
    const Eigen::Vector3f& point_base,
    const CargoObbFootprint& footprint,
    float margin_xy_m = 0.0F,
    float margin_z_m = 0.0F);

float pointToCargoObbDistance2D(
    const Eigen::Vector2f& point_base,
    const CargoObbFootprint& footprint);

float cargoAxialYawDifference(float lhs_rad, float rhs_rad);

Eigen::Vector3f limitCargoPoseResidualByRate(
    const Eigen::Vector3f& residual,
    double sensor_dt_sec,
    float max_xy_speed_mps,
    float max_z_speed_mps,
    float step_margin_m);

// Advances the live center while bounding the historical prediction, the
// measurement correction, the final center step, and the stored velocity.
CargoLivePoseStepResult updateCargoLivePoseStep(
    const CargoLivePoseStepInput& input);

// Propagates retained evidence for display and the short formal-use window.
// The physical evidence timestamp is deliberately preserved.
LiveCargoPose propagateHeldCargoPose(
    const LiveCargoPose& pose,
    const Eigen::Vector3f& velocity_base,
    double evaluation_stamp_sec,
    double max_prediction_sec,
    float max_xy_speed_mps,
    float max_z_speed_mps,
    double velocity_decay_tau_sec,
    float uncertainty_per_sec,
    float max_uncertainty_m);

}  // namespace ndt_slam
