#include "ndt_slam/crane_pose_constraint.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

namespace ndt_slam {
namespace {

bool finiteConfig(const CranePoseConstraintConfig& config) {
    return std::isfinite(config.fixed_z) &&
           std::isfinite(config.max_abs_z_drift) &&
           std::isfinite(config.fixed_roll_rad) &&
           std::isfinite(config.max_abs_roll_rad) &&
           std::isfinite(config.fixed_pitch_rad) &&
           std::isfinite(config.max_abs_pitch_rad) &&
           std::isfinite(config.fixed_yaw_rad) &&
           std::isfinite(config.max_abs_yaw_delta_rad) &&
           std::isfinite(config.orthogonality_tolerance) &&
           std::isfinite(config.determinant_tolerance) &&
           config.max_abs_z_drift >= 0.0 &&
           config.max_abs_roll_rad >= 0.0 &&
           config.max_abs_pitch_rad >= 0.0 &&
           config.max_abs_yaw_delta_rad >= 0.0 &&
           config.orthogonality_tolerance > 0.0 &&
           config.determinant_tolerance > 0.0;
}

double clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(value, upper));
}

}  // namespace

CranePoseRpy cranePoseRpy(const Sophus::SO3d& rotation) {
    CranePoseRpy result;
    const Eigen::Matrix3d matrix = rotation.matrix();
    if (!matrix.allFinite()) {
        return result;
    }

    result.pitch = std::asin(clamp(-matrix(2, 0), -1.0, 1.0));
    if (std::abs(std::cos(result.pitch)) > 1e-6) {
        result.roll = std::atan2(matrix(2, 1), matrix(2, 2));
        result.yaw = std::atan2(matrix(1, 0), matrix(0, 0));
    } else {
        result.roll = std::atan2(-matrix(0, 1), matrix(1, 1));
        result.yaw = 0.0;
    }
    result.valid = std::isfinite(result.roll) &&
                   std::isfinite(result.pitch) &&
                   std::isfinite(result.yaw);
    return result;
}

CranePoseConstraintResult applyCranePoseConstraint(
    const Sophus::SE3d& input,
    const CranePoseConstraintConfig& config,
    const CranePoseConstraintContext& context) {
    CranePoseConstraintResult result;
    result.pose = input;

    const Eigen::Matrix3d input_rotation = input.so3().matrix();
    if (!input.translation().allFinite() || !input_rotation.allFinite()) {
        result.pose = Sophus::SE3d();
        result.valid = false;
        result.reason = "input_pose_non_finite";
        return result;
    }

    if (!config.enabled) {
        return result;
    }

    if (!finiteConfig(config) || !std::isfinite(context.speed_xy_mps)) {
        result.fallback_used = true;
        result.reason = "invalid_constraint_configuration";
        return result;
    }

    const CranePoseRpy input_rpy = cranePoseRpy(input.so3());
    if (!input_rpy.valid) {
        result.valid = false;
        result.reason = "input_rotation_not_decomposable";
        result.pose = Sophus::SE3d();
        return result;
    }

    Eigen::Vector3d translation = input.translation();
    double roll = input_rpy.roll;
    double pitch = input_rpy.pitch;
    double yaw = input_rpy.yaw;

    if (config.lock_z) {
        translation.z() = config.fixed_z;
    } else if (config.constrain_z) {
        translation.z() = clamp(translation.z(),
                                config.fixed_z - config.max_abs_z_drift,
                                config.fixed_z + config.max_abs_z_drift);
    }

    if (config.lock_roll) {
        roll = config.fixed_roll_rad;
    } else if (config.constrain_roll) {
        roll = clamp(roll, -config.max_abs_roll_rad, config.max_abs_roll_rad);
    }

    if (config.lock_pitch) {
        pitch = config.fixed_pitch_rad;
    } else if (config.constrain_pitch) {
        pitch = clamp(pitch, -config.max_abs_pitch_rad, config.max_abs_pitch_rad);
    }

    if (config.lock_yaw) {
        yaw = config.fixed_yaw_rad;
    } else if (config.constrain_yaw) {
        yaw = clamp(yaw,
                    config.fixed_yaw_rad - config.max_abs_yaw_delta_rad,
                    config.fixed_yaw_rad + config.max_abs_yaw_delta_rad);
    }

    Eigen::Quaterniond quaternion =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    if (!quaternion.coeffs().allFinite() || quaternion.norm() <= 0.0) {
        result.fallback_used = true;
        result.reason = "rotation_reconstruction_failed";
        return result;
    }
    quaternion.normalize();

    const Sophus::SO3d constrained_rotation(quaternion);
    const Eigen::Matrix3d rotation = constrained_rotation.matrix();
    result.orthogonality_error =
        (rotation.transpose() * rotation - Eigen::Matrix3d::Identity())
            .cwiseAbs().maxCoeff();
    result.determinant = rotation.determinant();

    if (!translation.allFinite() || !rotation.allFinite() ||
        !std::isfinite(result.orthogonality_error) ||
        !std::isfinite(result.determinant) || result.determinant <= 0.0 ||
        result.orthogonality_error > config.orthogonality_tolerance ||
        std::abs(result.determinant - 1.0) > config.determinant_tolerance) {
        result.fallback_used = true;
        result.reason = "rotation_validation_failed";
        return result;
    }

    result.pose = Sophus::SE3d(constrained_rotation, translation);
    result.applied = true;
    result.reason = "constraint_applied";
    return result;
}

}  // namespace ndt_slam
