#pragma once

#include <cstdint>
#include <string>

#include <sophus/se3.hpp>

namespace ndt_slam {

struct CranePoseConstraintConfig {
    bool enabled = false;

    bool lock_z = false;
    double fixed_z = 0.0;
    bool constrain_z = false;
    double max_abs_z_drift = 0.0;

    bool lock_roll = false;
    double fixed_roll_rad = 0.0;
    bool constrain_roll = false;
    double max_abs_roll_rad = 0.0;

    bool lock_pitch = false;
    double fixed_pitch_rad = 0.0;
    bool constrain_pitch = false;
    double max_abs_pitch_rad = 0.0;

    bool lock_yaw = false;
    double fixed_yaw_rad = 0.0;
    bool constrain_yaw = false;
    double max_abs_yaw_delta_rad = 0.0;

    double orthogonality_tolerance = 1e-10;
    double determinant_tolerance = 1e-10;
};

struct CranePoseConstraintContext {
    bool low_motion = false;
    double speed_xy_mps = 0.0;
};

struct CranePoseRpy {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    bool valid = false;
};

struct CranePoseConstraintResult {
    Sophus::SE3d pose;
    bool applied = false;
    bool valid = true;
    bool fallback_used = false;
    double orthogonality_error = 0.0;
    double determinant = 1.0;
    std::string reason = "disabled";
};

CranePoseRpy cranePoseRpy(const Sophus::SO3d& rotation);

CranePoseConstraintResult applyCranePoseConstraint(
    const Sophus::SE3d& input,
    const CranePoseConstraintConfig& config,
    const CranePoseConstraintContext& context);

}  // namespace ndt_slam
