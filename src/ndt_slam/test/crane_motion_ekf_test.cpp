#include <gtest/gtest.h>

#include <cmath>

#include "ndt_slam/crane_motion_ekf.hpp"

namespace ndt_slam {
namespace {

Sophus::SE3d poseAt(double x, double y) {
    return Sophus::SE3d(
        Sophus::SO3d(), Eigen::Vector3d(x, y, 0.0));
}

CraneMotionEKF configuredFilter() {
    CraneMotionEKFConfig config;
    config.reject_high_fitness = false;
    config.max_speed_mps = 2.0;
    config.max_speed_x = 2.0;
    config.max_speed_y = 2.0;
    config.max_accel_x = 1.5;
    config.max_accel_y = 1.5;
    config.max_step_safety_factor = 1.10;
    config.output_soft_limit_ratio = 1.50;
    config.absolute_output_step_limit_m = 2.50;
    config.correction_nominal_limit_m = 0.35;
    config.correction_soft_limit_m = 1.00;
    CraneMotionEKF filter;
    filter.setConfig(config);
    return filter;
}

TEST(CraneMotionEkfTest, CorrectionAtThirtyThreeCentimetersIsAccepted) {
    CraneMotionEKF filter = configuredFilter();
    filter.initialize(poseAt(0.0, 0.0), ros::Time(1, 0));
    filter.updateWithNDT(
        poseAt(0.33, 0.0), 0.02, poseAt(0.33, 0.0),
        ros::Time(1, 100000000));
    EXPECT_TRUE(filter.status().ndt_accepted);
    EXPECT_FALSE(filter.status().correction_soft);
}

TEST(CraneMotionEkfTest, ModerateCorrectionIsDownweightedNotRejected) {
    CraneMotionEKF filter = configuredFilter();
    filter.initialize(poseAt(0.0, 0.0), ros::Time(1, 0));
    filter.updateWithNDT(
        poseAt(0.60, 0.0), 0.02, poseAt(0.60, 0.0),
        ros::Time(1, 300000000));
    EXPECT_TRUE(filter.status().ndt_accepted);
    EXPECT_TRUE(filter.status().correction_soft);
    EXPECT_FALSE(filter.status().map_commit_safe);
}

TEST(CraneMotionEkfTest, DynamicOutputLimitSoftAcceptsTwentyEightCentimeters) {
    CraneMotionEKF filter = configuredFilter();
    filter.initialize(poseAt(0.0, 0.0), ros::Time(1, 0));
    filter.updateWithNDT(
        poseAt(0.29, 0.0), 0.02, poseAt(0.29, 0.0),
        ros::Time(1, 100000000));
    EXPECT_TRUE(filter.status().ndt_accepted);
    EXPECT_TRUE(filter.status().output_step_soft);
    EXPECT_FALSE(filter.status().map_commit_safe);
}

TEST(CraneMotionEkfTest, CorrectionBeyondOneMeterIsRejected) {
    CraneMotionEKF filter = configuredFilter();
    filter.initialize(poseAt(0.0, 0.0), ros::Time(1, 0));
    filter.updateWithNDT(
        poseAt(1.01, 0.0), 0.02, poseAt(1.01, 0.0),
        ros::Time(1, 100000000));
    EXPECT_FALSE(filter.status().ndt_accepted);
    EXPECT_EQ(filter.status().reject_reason,
              "NDT_CORRECTION_HARD_LIMIT");
}

TEST(CraneMotionEkfTest, VehicleYawNoiseNeverRejectsXyState) {
    CraneMotionEKF filter = configuredFilter();
    filter.initialize(poseAt(0.0, 0.0), ros::Time(1, 0));
    constexpr double kDeg = 3.14159265358979323846 / 180.0;
    bool transitioned = false;
    for (int frame = 0; frame < 6; ++frame) {
        transitioned = filter.observeRuntimeYaw(92.0 * kDeg);
    }
    EXPECT_TRUE(transitioned);
    EXPECT_TRUE(filter.status().yaw_latched);
    filter.observeRuntimeYaw(92.31 * kDeg);
    EXPECT_TRUE(filter.status().yaw_latched);
    filter.updateWithNDT(
        poseAt(0.20, 0.0), 0.02, poseAt(0.20, 0.0),
        ros::Time(1, 100000000));
    EXPECT_TRUE(filter.status().ndt_accepted);
}

TEST(CraneMotionEkfTest, LatchedRailVehicleYawDoesNotFollowCargoSwingNoise) {
    CraneMotionEKF filter = configuredFilter();
    filter.initialize(poseAt(0.0, 0.0), ros::Time(1, 0));
    constexpr double kDeg = 3.14159265358979323846 / 180.0;
    for (int frame = 0; frame < 6; ++frame) {
        filter.observeRuntimeYaw(92.0 * kDeg);
    }
    const double fixed_yaw = filter.status().latched_yaw_rad;
    for (int frame = 0; frame < 100; ++frame) {
        filter.observeRuntimeYaw((frame % 2 == 0 ? 92.8 : 91.2) * kDeg);
    }
    EXPECT_TRUE(filter.status().yaw_latched);
    EXPECT_NEAR(filter.status().latched_yaw_rad, fixed_yaw, 1.0e-12);
    for (int frame = 0; frame < 10; ++frame) {
        filter.observeRuntimeYaw(98.0 * kDeg);
    }
    EXPECT_TRUE(filter.status().yaw_latched);
    EXPECT_GT(filter.status().yaw_anomaly_frames, 0);
    EXPECT_NEAR(filter.status().latched_yaw_rad, fixed_yaw, 1.0e-12);
}

TEST(CraneMotionEkfTest, RuntimeReseedPreservesBoundedVelocity) {
    CraneMotionEKF filter = configuredFilter();
    filter.initialize(poseAt(0.0, 0.0), ros::Time(1, 0));
    filter.updateWithNDT(
        poseAt(0.10, 0.0), 0.02, poseAt(0.10, 0.0),
        ros::Time(1, 100000000));
    const double speed_before = filter.state().tail<2>().norm();
    filter.reseedFromRelocalization(
        poseAt(3.0, -2.0), ros::Time(2, 0));
    EXPECT_NEAR(filter.state()(0), 3.0, 1.0e-9);
    EXPECT_NEAR(filter.state()(1), -2.0, 1.0e-9);
    EXPECT_NEAR(filter.state().tail<2>().norm(), speed_before, 1.0e-6);
    EXPECT_LE(filter.status().p_trace, 25.0 + 1.0e-9);
}

}  // namespace
}  // namespace ndt_slam
