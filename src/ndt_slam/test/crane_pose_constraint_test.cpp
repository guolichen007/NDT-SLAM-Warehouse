#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>

#include <Eigen/Geometry>

#include "ndt_slam/crane_pose_constraint.hpp"

namespace ndt_slam {
namespace {

constexpr double kPi = 3.14159265358979323846;

Sophus::SE3d makePose(double roll,
                      double pitch,
                      double yaw,
                      const Eigen::Vector3d& translation) {
    Eigen::Quaterniond quaternion =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    quaternion.normalize();
    return Sophus::SE3d(Sophus::SO3d(quaternion), translation);
}

void expectValidRotation(const Sophus::SE3d& pose, double tolerance = 1e-12) {
    const Eigen::Matrix3d rotation = pose.so3().matrix();
    ASSERT_TRUE(rotation.allFinite());
    EXPECT_LT((rotation.transpose() * rotation - Eigen::Matrix3d::Identity())
                  .cwiseAbs().maxCoeff(),
              tolerance);
    EXPECT_NEAR(rotation.determinant(), 1.0, tolerance);
    EXPECT_GT(rotation.determinant(), 0.0);
    EXPECT_NO_THROW({
        const Sophus::SO3d reconstructed(pose.so3().unit_quaternion());
        (void)reconstructed;
    });
}

TEST(CranePoseConstraint, DisabledPreservesPoseExactly) {
    const Sophus::SE3d input = makePose(0.2, -0.3, 1.1, {1.0, 2.0, 3.0});
    CranePoseConstraintConfig config;
    const CranePoseConstraintResult result =
        applyCranePoseConstraint(input, config, {});

    EXPECT_TRUE(result.valid);
    EXPECT_FALSE(result.applied);
    EXPECT_FALSE(result.fallback_used);
    EXPECT_EQ((result.pose.matrix() - input.matrix()).cwiseAbs().maxCoeff(), 0.0);
}

TEST(CranePoseConstraint, SupportsEveryProductionLockCombination) {
    const Sophus::SE3d input = makePose(0.2, -0.3, 1.1, {1.0, 2.0, 3.0});
    for (int mask = 1; mask < 8; ++mask) {
        CranePoseConstraintConfig config;
        config.enabled = true;
        config.lock_roll = (mask & 1) != 0;
        config.fixed_roll_rad = -0.12;
        config.lock_pitch = (mask & 2) != 0;
        config.fixed_pitch_rad = 0.08;
        config.lock_z = (mask & 4) != 0;
        config.fixed_z = 7.5;

        const CranePoseConstraintResult result =
            applyCranePoseConstraint(input, config, {true, 0.0});
        ASSERT_TRUE(result.valid) << result.reason;
        ASSERT_TRUE(result.applied) << result.reason;
        expectValidRotation(result.pose);
        EXPECT_DOUBLE_EQ(result.pose.translation().x(), 1.0);
        EXPECT_DOUBLE_EQ(result.pose.translation().y(), 2.0);
        EXPECT_DOUBLE_EQ(result.pose.translation().z(),
                         config.lock_z ? config.fixed_z : 3.0);

        const CranePoseRpy rpy = cranePoseRpy(result.pose.so3());
        ASSERT_TRUE(rpy.valid);
        EXPECT_NEAR(rpy.roll, config.lock_roll ? config.fixed_roll_rad : 0.2,
                    1e-12);
        EXPECT_NEAR(rpy.pitch, config.lock_pitch ? config.fixed_pitch_rad : -0.3,
                    1e-12);
        EXPECT_NEAR(rpy.yaw, 1.1, 1e-12);
    }
}

TEST(CranePoseConstraint, ReconstructsValidRotationAtYawBoundaries) {
    const double yaws[] = {0.0, 1e-15, kPi - 1e-12, -kPi + 1e-12,
                           kPi + 1e-9, -kPi - 1e-9};
    for (double yaw : yaws) {
        CranePoseConstraintConfig config;
        config.enabled = true;
        config.lock_roll = true;
        config.fixed_roll_rad = 0.17;
        config.lock_pitch = true;
        config.fixed_pitch_rad = -0.11;
        config.lock_yaw = true;
        config.fixed_yaw_rad = yaw;

        const CranePoseConstraintResult result = applyCranePoseConstraint(
            makePose(-0.4, 0.3, 0.5, {-4.0, 8.0, 2.0}), config, {});
        ASSERT_TRUE(result.valid) << result.reason;
        expectValidRotation(result.pose);
        const CranePoseRpy rpy = cranePoseRpy(result.pose.so3());
        ASSERT_TRUE(rpy.valid);
        EXPECT_NEAR(rpy.roll, 0.17, 1e-12);
        EXPECT_NEAR(rpy.pitch, -0.11, 1e-12);
        EXPECT_NEAR(std::sin(rpy.yaw), std::sin(yaw), 1e-12);
        EXPECT_NEAR(std::cos(rpy.yaw), std::cos(yaw), 1e-12);
    }
}

TEST(CranePoseConstraint, RepeatedApplicationDoesNotAccumulateError) {
    CranePoseConstraintConfig config;
    config.enabled = true;
    config.lock_z = true;
    config.fixed_z = 4.25;
    config.lock_roll = true;
    config.fixed_roll_rad = 0.03;
    config.lock_pitch = true;
    config.fixed_pitch_rad = -0.02;

    Sophus::SE3d pose = makePose(0.6, -0.5, kPi - 1e-8, {2.0, -3.0, 9.0});
    const CranePoseConstraintResult first =
        applyCranePoseConstraint(pose, config, {true, 0.0});
    ASSERT_TRUE(first.valid);
    pose = first.pose;
    for (int i = 0; i < 10000; ++i) {
        const CranePoseConstraintResult result =
            applyCranePoseConstraint(pose, config, {true, 0.0});
        ASSERT_TRUE(result.valid) << i << ": " << result.reason;
        pose = result.pose;
        expectValidRotation(pose);
    }
    EXPECT_LT((pose.matrix() - first.pose.matrix()).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(CranePoseConstraint, RandomStressAlwaysProducesProperSO3) {
    std::mt19937 generator(588u);
    std::uniform_real_distribution<double> roll_pitch(-1.4, 1.4);
    std::uniform_real_distribution<double> yaw(-4.0 * kPi, 4.0 * kPi);
    std::uniform_real_distribution<double> translation(-100.0, 100.0);
    std::uniform_real_distribution<double> speed(0.0, 3.0);
    std::bernoulli_distribution boolean;

    for (int i = 0; i < 10000; ++i) {
        CranePoseConstraintConfig config;
        config.enabled = true;
        config.lock_z = boolean(generator);
        config.fixed_z = translation(generator);
        config.constrain_z = !config.lock_z && boolean(generator);
        config.max_abs_z_drift = 0.25;
        config.lock_roll = boolean(generator);
        config.fixed_roll_rad = roll_pitch(generator);
        config.constrain_roll = !config.lock_roll;
        config.max_abs_roll_rad = 0.4;
        config.lock_pitch = boolean(generator);
        config.fixed_pitch_rad = roll_pitch(generator);
        config.constrain_pitch = !config.lock_pitch;
        config.max_abs_pitch_rad = 0.4;
        config.lock_yaw = boolean(generator);
        config.fixed_yaw_rad = yaw(generator);
        config.constrain_yaw = !config.lock_yaw && boolean(generator);
        config.max_abs_yaw_delta_rad = 0.3;

        const Sophus::SE3d input = makePose(
            roll_pitch(generator), roll_pitch(generator), yaw(generator),
            {translation(generator), translation(generator), translation(generator)});
        const CranePoseConstraintResult result = applyCranePoseConstraint(
            input, config, {boolean(generator), speed(generator)});
        ASSERT_TRUE(result.valid) << i << ": " << result.reason;
        ASSERT_FALSE(result.fallback_used) << i << ": " << result.reason;
        expectValidRotation(result.pose);
        EXPECT_DOUBLE_EQ(result.pose.translation().x(), input.translation().x());
        EXPECT_DOUBLE_EQ(result.pose.translation().y(), input.translation().y());
    }
}

TEST(CranePoseConstraint, InvalidContextFallsBackToValidInput) {
    const Sophus::SE3d input = makePose(0.1, 0.2, 0.3, {1.0, 2.0, 3.0});
    CranePoseConstraintConfig config;
    config.enabled = true;
    const CranePoseConstraintResult result = applyCranePoseConstraint(
        input, config, {false, std::numeric_limits<double>::quiet_NaN()});

    EXPECT_TRUE(result.valid);
    EXPECT_TRUE(result.fallback_used);
    EXPECT_EQ(result.reason, "invalid_constraint_configuration");
    EXPECT_EQ((result.pose.matrix() - input.matrix()).cwiseAbs().maxCoeff(), 0.0);
    expectValidRotation(result.pose);
}

TEST(CranePoseConstraint, NonFiniteInputIsRejectedWithoutReturningNaN) {
    Sophus::SE3d input = makePose(0.1, 0.2, 0.3, {1.0, 2.0, 3.0});
    input.translation().x() = std::numeric_limits<double>::quiet_NaN();
    CranePoseConstraintConfig config;
    config.enabled = true;
    const CranePoseConstraintResult result =
        applyCranePoseConstraint(input, config, {});

    EXPECT_FALSE(result.valid);
    EXPECT_FALSE(result.fallback_used);
    EXPECT_EQ(result.reason, "input_pose_non_finite");
    EXPECT_TRUE(result.pose.matrix().allFinite());
    expectValidRotation(result.pose);
}

}  // namespace
}  // namespace ndt_slam
