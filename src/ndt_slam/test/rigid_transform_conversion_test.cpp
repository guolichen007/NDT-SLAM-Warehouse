#include "ndt_slam/rigid_transform_conversion.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <limits>

namespace ndt_slam {
namespace {

void expectValidRigidTransform(const SafeSE3Result& result) {
    ASSERT_TRUE(result.valid) << result.diagnostics.reason;
    const Eigen::Matrix3d rotation = result.pose.so3().matrix();
    EXPECT_TRUE(rotation.allFinite());
    EXPECT_TRUE(result.pose.translation().allFinite());
    EXPECT_LT((rotation.transpose() * rotation - Eigen::Matrix3d::Identity())
                  .cwiseAbs()
                  .maxCoeff(),
              1.0e-12);
    EXPECT_NEAR(rotation.determinant(), 1.0, 1.0e-12);
    EXPECT_NEAR(result.pose.unit_quaternion().squaredNorm(), 1.0, 1.0e-12);
}

Eigen::Matrix4d transformWithRotation(const Eigen::Matrix3d& rotation) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = rotation;
    transform.block<3, 1>(0, 3) = Eigen::Vector3d(1.0, -2.0, 0.5);
    return transform;
}

TEST(RigidTransformConversion, AcceptsExactAndApproximateRotations) {
    expectValidRigidTransform(
        makeSafeSE3FromMatrix(Eigen::Matrix4d::Identity()));

    const Eigen::Matrix3d yaw =
        Eigen::AngleAxisd(0.73, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    expectValidRigidTransform(makeSafeSE3FromMatrix(transformWithRotation(yaw)));

    const Eigen::Matrix4f float_transform =
        transformWithRotation(yaw).cast<float>();
    expectValidRigidTransform(
        makeSafeSE3FromMatrix(float_transform.cast<double>()));

    for (const double perturbation : {1.0e-15, 1.0e-8, 1.0e-4}) {
        Eigen::Matrix3d perturbed = yaw;
        perturbed(0, 1) += perturbation;
        perturbed(2, 0) -= perturbation * 0.5;
        expectValidRigidTransform(
            makeSafeSE3FromMatrix(transformWithRotation(perturbed)));
    }
}

TEST(RigidTransformConversion, CorrectsReflectionAndNearSingularRotation) {
    Eigen::Matrix3d reflection = Eigen::Matrix3d::Identity();
    reflection(2, 2) = -1.0;
    expectValidRigidTransform(
        makeSafeSE3FromMatrix(transformWithRotation(reflection)));

    Eigen::Matrix3d near_singular = Eigen::Matrix3d::Identity();
    near_singular(2, 2) = 1.0e-8;
    expectValidRigidTransform(
        makeSafeSE3FromMatrix(transformWithRotation(near_singular)));
}

TEST(RigidTransformConversion, RejectsNonFiniteDegenerateAndInvalidHomogeneousInput) {
    Eigen::Matrix4d nan_input = Eigen::Matrix4d::Identity();
    nan_input(0, 0) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(makeSafeSE3FromMatrix(nan_input).valid);

    Eigen::Matrix4d inf_input = Eigen::Matrix4d::Identity();
    inf_input(1, 1) = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(makeSafeSE3FromMatrix(inf_input).valid);

    Eigen::Matrix4d zero_rotation = Eigen::Matrix4d::Identity();
    zero_rotation.block<3, 3>(0, 0).setZero();
    const SafeSE3Result zero_result = makeSafeSE3FromMatrix(zero_rotation);
    EXPECT_FALSE(zero_result.valid);
    EXPECT_EQ(zero_result.diagnostics.reason, "rotation_degenerate");

    Eigen::Matrix4d singular = Eigen::Matrix4d::Identity();
    singular(2, 2) = 1.0e-14;
    EXPECT_FALSE(makeSafeSE3FromMatrix(singular).valid);

    Eigen::Matrix4d invalid_bottom = Eigen::Matrix4d::Identity();
    invalid_bottom(3, 0) = 1.0e-5;
    EXPECT_FALSE(makeSafeSE3FromMatrix(invalid_bottom).valid);

    Eigen::Matrix4d invalid_translation = Eigen::Matrix4d::Identity();
    invalid_translation(0, 3) = std::numeric_limits<double>::infinity();
    const SafeSE3Result translation_result =
        makeSafeSE3FromMatrix(invalid_translation);
    EXPECT_FALSE(translation_result.valid);
    EXPECT_EQ(translation_result.diagnostics.reason,
              "translation_non_finite");
}

TEST(RigidTransformConversion, NdtStyleFloatConversionRemainsStable) {
    Eigen::Matrix4f ndt = Eigen::Matrix4f::Identity();
    ndt.block<3, 3>(0, 0) =
        Eigen::AngleAxisf(0.17F, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    ndt(0, 1) += 2.0e-6F;
    ndt.block<3, 1>(0, 3) = Eigen::Vector3f(4.0F, -3.0F, 0.2F);

    for (int frame = 0; frame < 100000; ++frame) {
        const SafeSE3Result converted =
            makeSafeSE3FromMatrix(ndt.cast<double>());
        ASSERT_TRUE(converted.valid) << "frame=" << frame;
        const Eigen::Matrix3d rotation = converted.pose.so3().matrix();
        ASSERT_LT((rotation.transpose() * rotation -
                   Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(),
                  1.0e-12);
        ASSERT_NEAR(converted.pose.unit_quaternion().squaredNorm(),
                    1.0, 1.0e-12);
    }
}

TEST(RigidTransformConversion, IcpStyleCorrectionCanBeComposedSafely) {
    Eigen::Matrix4f icp = Eigen::Matrix4f::Identity();
    icp.block<3, 3>(0, 0) =
        Eigen::AngleAxisf(0.01F, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    icp(1, 0) += 1.0e-6F;
    icp.block<3, 1>(0, 3) = Eigen::Vector3f(0.03F, -0.02F, 0.0F);

    const SafeSE3Result converted =
        makeSafeSE3FromMatrix(icp.cast<double>());
    expectValidRigidTransform(converted);
    const Sophus::SE3d correction = Sophus::SE3d().inverse() * converted.pose;
    EXPECT_LT(correction.translation().norm(), 0.05);
    EXPECT_LT(correction.so3().log().norm(), 0.02);
}

}  // namespace
}  // namespace ndt_slam
