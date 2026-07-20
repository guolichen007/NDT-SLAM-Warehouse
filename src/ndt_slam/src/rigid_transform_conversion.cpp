#include "ndt_slam/rigid_transform_conversion.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>

namespace ndt_slam {
namespace {

constexpr double kHomogeneousRowTolerance = 1.0e-8;
constexpr double kProjectedRotationTolerance = 1.0e-10;
constexpr double kQuaternionMinimumNorm = 1.0e-12;
constexpr double kQuaternionUnitTolerance = 1.0e-12;
constexpr double kMinimumSingularValue = 1.0e-12;

double maximumOrthogonalityError(const Eigen::Matrix3d& rotation) {
    return (rotation.transpose() * rotation - Eigen::Matrix3d::Identity())
        .cwiseAbs()
        .maxCoeff();
}

}  // namespace

SafeSE3Result makeSafeSE3(const Eigen::Matrix3d& rotation,
                          const Eigen::Vector3d& translation) {
    SafeSE3Result result;
    result.diagnostics.translation_finite = translation.allFinite();
    result.diagnostics.input_finite =
        rotation.allFinite() && result.diagnostics.translation_finite;
    result.diagnostics.homogeneous_row_valid = true;

    if (!result.diagnostics.translation_finite) {
        result.diagnostics.reason = "translation_non_finite";
        return result;
    }
    if (!rotation.allFinite()) {
        result.diagnostics.reason = "rotation_non_finite";
        return result;
    }

    result.diagnostics.input_orthogonality_error =
        maximumOrthogonalityError(rotation);
    result.diagnostics.input_determinant = rotation.determinant();
    if (!std::isfinite(result.diagnostics.input_orthogonality_error) ||
        !std::isfinite(result.diagnostics.input_determinant)) {
        result.diagnostics.reason = "rotation_metrics_non_finite";
        return result;
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
    if (!svd.singularValues().allFinite() ||
        svd.singularValues().minCoeff() <= kMinimumSingularValue) {
        result.diagnostics.reason = "rotation_degenerate";
        return result;
    }

    const Eigen::Matrix3d u = svd.matrixU();
    const Eigen::Matrix3d v = svd.matrixV();
    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
    if ((u * v.transpose()).determinant() < 0.0) {
        correction(2, 2) = -1.0;
    }
    const Eigen::Matrix3d projected = u * correction * v.transpose();

    if (!projected.allFinite()) {
        result.diagnostics.reason = "projected_rotation_non_finite";
        return result;
    }
    result.diagnostics.projected_orthogonality_error =
        maximumOrthogonalityError(projected);
    result.diagnostics.projected_determinant = projected.determinant();
    if (!std::isfinite(result.diagnostics.projected_orthogonality_error) ||
        !std::isfinite(result.diagnostics.projected_determinant) ||
        result.diagnostics.projected_orthogonality_error >
            kProjectedRotationTolerance ||
        result.diagnostics.projected_determinant <= 0.0 ||
        std::abs(result.diagnostics.projected_determinant - 1.0) >
            kProjectedRotationTolerance) {
        result.diagnostics.reason = "projected_rotation_invalid";
        return result;
    }

    Eigen::Quaterniond quaternion(projected);
    result.diagnostics.quaternion_norm_before_normalize = quaternion.norm();
    if (!quaternion.coeffs().allFinite() ||
        !std::isfinite(result.diagnostics.quaternion_norm_before_normalize) ||
        result.diagnostics.quaternion_norm_before_normalize <=
            kQuaternionMinimumNorm) {
        result.diagnostics.reason = "quaternion_invalid";
        return result;
    }

    quaternion.normalize();
    if (!quaternion.coeffs().allFinite() ||
        std::abs(quaternion.squaredNorm() - 1.0) >
            kQuaternionUnitTolerance) {
        result.diagnostics.reason = "quaternion_normalization_failed";
        return result;
    }

    result.pose = Sophus::SE3d(Sophus::SO3d(quaternion), translation);
    result.valid = true;
    result.diagnostics.reason = "ok";
    return result;
}

SafeSE3Result makeSafeSE3FromMatrix(const Eigen::Matrix4d& input) {
    SafeSE3Result result;
    const Eigen::Vector3d translation = input.block<3, 1>(0, 3);
    result.diagnostics.input_finite = input.allFinite();
    result.diagnostics.translation_finite = translation.allFinite();

    if (!result.diagnostics.translation_finite) {
        result.diagnostics.reason = "translation_non_finite";
        return result;
    }
    if (!result.diagnostics.input_finite) {
        result.diagnostics.reason = "matrix_non_finite";
        return result;
    }

    const Eigen::Vector4d expected_bottom(0.0, 0.0, 0.0, 1.0);
    result.diagnostics.homogeneous_row_valid =
        (input.row(3).transpose() - expected_bottom)
            .cwiseAbs()
            .maxCoeff() <= kHomogeneousRowTolerance;
    if (!result.diagnostics.homogeneous_row_valid) {
        result.diagnostics.reason = "invalid_homogeneous_row";
        return result;
    }

    result = makeSafeSE3(input.block<3, 3>(0, 0), translation);
    result.diagnostics.input_finite = true;
    result.diagnostics.translation_finite = true;
    result.diagnostics.homogeneous_row_valid = true;
    return result;
}

}  // namespace ndt_slam
