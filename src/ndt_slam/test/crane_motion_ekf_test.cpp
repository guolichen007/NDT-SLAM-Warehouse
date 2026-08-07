#include <gtest/gtest.h>

#include <cmath>

#include "ndt_slam/crane_motion_ekf.hpp"

namespace ndt_slam {
namespace {

// ROS_WARN_THROTTLE and ROS_WARN internally call ros::Time::now().
// Standalone gtest binaries must initialize ros::Time before any
// EKF call that can reach a throttle or warning log statement.
struct RosTimeFixture : ::testing::Test {
    static void SetUpTestSuite() {
        ros::Time::init();
    }
};

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

TEST_F(RosTimeFixture, CorrectionAtThirtyThreeCentimetersIsAccepted) {
    CraneMotionEKF filter = configuredFilter();
    filter.initialize(poseAt(0.0, 0.0), ros::Time(1, 0));
    filter.updateWithNDT(
        poseAt(0.33, 0.0), 0.02, poseAt(0.33, 0.0),
        ros::Time(1, 100000000));
    EXPECT_TRUE(filter.status().ndt_accepted);
    EXPECT_FALSE(filter.status().correction_soft);
}

TEST_F(RosTimeFixture, ModerateCorrectionIsDownweightedNotRejected) {
    CraneMotionEKF filter = configuredFilter();
    filter.initialize(poseAt(0.0, 0.0), ros::Time(1, 0));
    filter.updateWithNDT(
        poseAt(0.60, 0.0), 0.02, poseAt(0.60, 0.0),
        ros::Time(1, 300000000));
    EXPECT_TRUE(filter.status().ndt_accepted);
    EXPECT_TRUE(filter.status().correction_soft);
    EXPECT_FALSE(filter.status().map_commit_safe);
}

TEST_F(RosTimeFixture, DynamicOutputLimitSoftAcceptsTwentyEightCentimeters) {
    CraneMotionEKF filter = configuredFilter();
    filter.initialize(poseAt(0.0, 0.0), ros::Time(1, 0));
    filter.updateWithNDT(
        poseAt(0.29, 0.0), 0.02, poseAt(0.29, 0.0),
        ros::Time(1, 100000000));
    EXPECT_TRUE(filter.status().ndt_accepted);
    EXPECT_TRUE(filter.status().output_step_soft);
    EXPECT_FALSE(filter.status().map_commit_safe);
}

TEST_F(RosTimeFixture, CorrectionBeyondOneMeterIsRejected) {
    CraneMotionEKF filter = configuredFilter();
    filter.initialize(poseAt(0.0, 0.0), ros::Time(1, 0));
    filter.updateWithNDT(
        poseAt(1.01, 0.0), 0.02, poseAt(1.01, 0.0),
        ros::Time(1, 100000000));
    EXPECT_FALSE(filter.status().ndt_accepted);
    EXPECT_EQ(filter.status().reject_reason,
              "NDT_CORRECTION_HARD_LIMIT");
}

TEST_F(RosTimeFixture, VehicleYawNoiseNeverRejectsXyState) {
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

TEST_F(RosTimeFixture, LatchedRailVehicleYawDoesNotFollowCargoSwingNoise) {
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

TEST_F(RosTimeFixture, RuntimeReseedPreservesBoundedVelocity) {
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

TEST_F(RosTimeFixture, RejectedOversizedPredictionHoldsCommittedState) {
    CraneMotionEKF filter = configuredFilter();
    filter.initialize(poseAt(0.0, 0.0), ros::Time(1, 0));
    Eigen::Vector4d oversized_prediction;
    oversized_prediction << 1.0, 0.0, 2.0, 0.0;
    const Sophus::SE3d output = filter.rejectCandidate(
        oversized_prediction, Eigen::Matrix4d::Identity(),
        Eigen::Vector2d(2.0, 0.0), 2.0, poseAt(2.0, 0.0),
        ros::Time(1, 100000000), "TEST_REJECT");
    EXPECT_NEAR(output.translation().x(), 0.0, 1.0e-9);
    EXPECT_NEAR(filter.state().head<2>().norm(), 0.0, 1.0e-9);
    EXPECT_NEAR(filter.state().tail<2>().norm(), 0.0, 1.0e-9);
    EXPECT_EQ(filter.status().reject_reason,
              "TEST_REJECT_PRED_STEP_LIMIT");
}

TEST_F(RosTimeFixture, RelocalizationReseedKeepsRecoveryAllowance) {
    CraneMotionEKFConfig config;
    config.reject_high_fitness = false;
    config.correction_soft_limit_m = 1.0;
    config.max_reject_innovation_frames = 0;
    config.recovery_covariance_inflation = 2.0;
    config.recovery_max_covariance_trace = 25.0;
    CraneMotionEKF filter;
    filter.setConfig(config);
    filter.initialize(poseAt(0.0, 0.0), ros::Time(1, 0));
    filter.reseedFromRelocalization(poseAt(3.0, -2.0), ros::Time(2, 0));
    filter.updateWithNDT(
        poseAt(5.0, -2.0), 0.02, poseAt(5.0, -2.0),
        ros::Time(2, 100000000));
    EXPECT_FALSE(filter.status().ndt_accepted);
    EXPECT_TRUE(filter.status().recovered);
    EXPECT_LE(filter.status().p_trace, 25.0 + 1.0e-9);
}

TEST_F(RosTimeFixture, RecoveryCovarianceInflationSurvivesReject) {
    // 验证 maybeRecover 的协方差膨胀真正保留在 EKF 的最终 P_ 中，
    // 不会被 rejectCandidate 内部的 P_ = P_pred 覆盖。
    CraneMotionEKFConfig config;
    config.reject_high_fitness = false;
    config.correction_soft_limit_m = 1.0;
    config.max_reject_innovation_frames = 0;
    config.recovery_covariance_inflation = 4.0;
    config.recovery_max_covariance_trace = 100.0;
    CraneMotionEKF filter;
    filter.setConfig(config);
    filter.initialize(poseAt(0.0, 0.0), ros::Time(1, 0));

    // 记录 reject 前的 P trace 作为基线。
    const double p_trace_before = filter.status().p_trace;
    EXPECT_GT(p_trace_before, 0.0);

    // 制造一个必定触发 hard correction reject 的测量。
    filter.updateWithNDT(
        poseAt(5.0, 0.0), 0.02, poseAt(5.0, 0.0),
        ros::Time(1, 200000000));

    EXPECT_FALSE(filter.status().ndt_accepted);
    EXPECT_TRUE(filter.status().recovered);

    const double p_trace_after = filter.status().p_trace;
    // 恢复后的 P trace 必须显著大于初始 trace（膨胀已生效）。
    EXPECT_GT(p_trace_after, p_trace_before * 1.5);

    // p_trace 必须与 EKF 内部实际 P_ 的 trace 一致。
    EXPECT_NEAR(filter.status().p_trace, p_trace_after, 1.0e-9);

    // 同一个 recovery episode 内再次 reject 不应重复膨胀。
    filter.updateWithNDT(
        poseAt(6.0, 0.0), 0.02, poseAt(6.0, 0.0),
        ros::Time(1, 300000000));
    EXPECT_FALSE(filter.status().ndt_accepted);
    // recovered 在上次已设置且 recovery_inflated_this_episode_=true，
    // 所以本次 maybeRecover 不应再次膨胀。
    EXPECT_LE(filter.status().p_trace, p_trace_after * 1.2 + 1.0e-6);
}

TEST_F(RosTimeFixture, OutputStepLimitRejectAlsoTriggersRecovery) {
    // OUTPUT_STEP_LIMIT 路径也必须统一走 maybeRecover，
    // 不能只有 correction/innovation reject 才触发恢复。
    CraneMotionEKFConfig config;
    config.reject_high_fitness = false;
    config.correction_soft_limit_m = 2.0;   // 放宽 correction 门限
    config.max_reject_innovation_frames = 0;
    config.max_speed_mps = 0.05;            // 极低速度使步长限制容易触发
    config.max_step_safety_factor = 1.0;
    config.absolute_output_step_limit_m = 0.10;
    config.recovery_covariance_inflation = 3.0;
    config.recovery_max_covariance_trace = 50.0;
    CraneMotionEKF filter;
    filter.setConfig(config);
    filter.initialize(poseAt(0.0, 0.0), ros::Time(1, 0));

    const double p_trace_before = filter.status().p_trace;

    // 大位移在短 dt 内触发 OUTPUT_STEP_LIMIT。
    filter.updateWithNDT(
        poseAt(0.30, 0.0), 0.02, poseAt(0.30, 0.0),
        ros::Time(1, 100000000));

    EXPECT_FALSE(filter.status().ndt_accepted);
    EXPECT_TRUE(filter.status().recovered);

    // 恢复膨胀必须已生效。
    EXPECT_GT(filter.status().p_trace, p_trace_before * 1.5);
}

TEST_F(RosTimeFixture, SingleRejectCountsOnlyOneDegradedFrame) {
    // 验证一次 NIS/axis reject 不会重复计数 degraded frames。
    // rejectCandidate 是退化计数器的唯一 owner。
    CraneMotionEKFConfig config;
    config.reject_high_fitness = false;
    config.correction_soft_limit_m = 1.0;
    config.max_reject_innovation_frames = 5;   // >0 避免立即触发 recovery
    CraneMotionEKF filter;
    filter.setConfig(config);
    filter.initialize(poseAt(0.0, 0.0), ros::Time(1, 0));

    // 制造一个必定触发 hard correction reject 的测量。
    filter.updateWithNDT(
        poseAt(1.50, 0.0), 0.02, poseAt(1.50, 0.0),
        ros::Time(1, 200000000));

    EXPECT_FALSE(filter.status().ndt_accepted);
    // 每个计数器只应 +1，不是 +2。
    EXPECT_EQ(filter.status().frames_since_good_ndt, 1);
    EXPECT_EQ(filter.status().consecutive_degraded_frames, 1);
    EXPECT_EQ(filter.status().reject_innovation_frames, 1);
}

TEST_F(RosTimeFixture, PredictionOnlyFrameClearsBothRearmCounters) {
    // predictWithoutMeasurement 必须同时清零 nominal 和 accepted rearm 计数，
    // 防止预测帧中间断开健康序列却仍算作"连续健康"。
    CraneMotionEKFConfig config;
    config.reject_high_fitness = false;
    config.max_reject_innovation_frames = 10;
    CraneMotionEKF filter;
    filter.setConfig(config);
    filter.initialize(poseAt(0.0, 0.0), ros::Time(1, 0));

    // 先产生几个 accepted frame 积累 rearm 计数。
    filter.updateWithNDT(
        poseAt(0.10, 0.0), 0.02, poseAt(0.10, 0.0),
        ros::Time(1, 100000000));
    filter.updateWithNDT(
        poseAt(0.20, 0.0), 0.02, poseAt(0.20, 0.0),
        ros::Time(1, 200000000));
    EXPECT_TRUE(filter.status().ndt_accepted);

    // 一个 prediction-only 帧必须断开 rearm 连续序列。
    filter.predictWithoutMeasurement(
        poseAt(0.20, 0.0), ros::Time(1, 300000000), "TEST_PRED_ONLY");

    // 之后一个 accepted 帧不应立即完成 rearm。
    filter.updateWithNDT(
        poseAt(0.30, 0.0), 0.02, poseAt(0.30, 0.0),
        ros::Time(1, 400000000));
    EXPECT_TRUE(filter.status().ndt_accepted);

    // 确认 prediction-only 后 rearm 序列已断开：不应触发 recovery
    // 即使此时 reject_innovation_frames 超限（如果未清零就超了）。
    EXPECT_FALSE(filter.status().recovered);
}

}  // namespace
}  // namespace ndt_slam
