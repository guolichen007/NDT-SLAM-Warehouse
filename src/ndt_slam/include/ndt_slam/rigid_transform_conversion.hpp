#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <limits>
#include <string>

namespace ndt_slam {

struct RotationProjectionDiagnostics {
    bool input_finite = false;
    bool translation_finite = false;
    bool homogeneous_row_valid = false;

    double input_orthogonality_error =
        std::numeric_limits<double>::infinity();
    double input_determinant =
        std::numeric_limits<double>::quiet_NaN();
    double projected_orthogonality_error =
        std::numeric_limits<double>::infinity();
    double projected_determinant =
        std::numeric_limits<double>::quiet_NaN();
    double quaternion_norm_before_normalize =
        std::numeric_limits<double>::quiet_NaN();

    std::string reason = "not_evaluated";
};

struct SafeSE3Result {
    bool valid = false;
    Sophus::SE3d pose;
    RotationProjectionDiagnostics diagnostics;
};

// Projects an externally produced approximate rotation onto SO(3), validates
// the projection, and only then constructs Sophus from a normalized quaternion.
SafeSE3Result makeSafeSE3(const Eigen::Matrix3d& rotation,
                          const Eigen::Vector3d& translation);

// As above, with an additional finite/homogeneous-row contract for a 4x4
// rigid-transform matrix.
SafeSE3Result makeSafeSE3FromMatrix(const Eigen::Matrix4d& input);

}  // namespace ndt_slam
