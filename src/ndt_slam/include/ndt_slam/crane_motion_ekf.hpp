#pragma once

#include <Eigen/Dense>
#include <ros/ros.h>
#include <sophus/se3.hpp>
#include <algorithm>
#include <cmath>
#include <string>

#include "ndt_slam/ndt_observability.hpp"

namespace ndt_slam {

struct CraneMotionEKFConfig {
    bool enabled = true;

    double q_pos = 0.05;
    double q_vel = 0.30;

    double r_ndt_base = 0.02;
    double r_ndt_max = 2.0;
    double fitness_to_r_scale = 5.0;

    double innovation_gate_m = 0.35;   // V3: 收紧到 0.35
    double innovation_reject_m = 1.00; // V3: 收紧到 1.00

    double max_speed_x = 2.0;
    double max_speed_y = 2.0;
    double max_accel_x = 1.0;
    double max_accel_y = 1.0;

    // V3: 慢帧降权
    bool slow_frame_guard_enabled = true;
    double slow_frame_warn_ms = 100.0;
    double slow_frame_emergency_ms = 120.0;
    double slow_frame_extra_r = 0.30;

    // V3: 物理步长保护
    double max_speed_mps = 0.50;
    double max_step_safety_factor = 1.5;
    double max_step_min_m = 0.08;
    double max_step_max_m = 0.25;

    double stationary_position_hold_variance = 0.0025;
    double stationary_velocity_hold_variance = 0.001;
    NdtObservabilityConfig observability;

    // Legacy velocity-direction damping can suppress a real second-axis
    // start.  Independent X/Y innovation gates are the production default.
    bool axis_independent_gate = true;
    bool diagonal_enabled = false;
    double diagonal_min_vx = 0.05;
    double diagonal_min_vy = 0.05;
    double diagonal_min_speed = 0.10;
    double lateral_gate_m = 0.20;
    double tangential_gate_m = 0.40;
    double lateral_damping = 0.70;
    double tangential_damping = 0.40;
    double nis_reject_threshold = 13.82;  // chi-square, 2 DoF, 99.9%

    int max_frames_since_good_ndt = 30;
    int max_high_fitness_frames = 10;
    int max_reject_innovation_frames = 5;
    double high_fitness_threshold = 0.15;

    // 高 fitness 拒绝
    bool reject_high_fitness = true;
    double ndt_fitness_reject_threshold = 0.30;
    double ndt_fitness_recover_threshold = 0.12;

    // ========== 运行时 Yaw 锁存 ==========
    // 固定轨道天车运行时车体 yaw 不应自由变化。
    // HEALTHY 正常运行时 NDT 只更新 XY；Z/roll/pitch/yaw 使用 accepted pose。
    bool runtime_yaw_latched = true;
    double maximum_runtime_yaw_step_deg = 0.30;
    double maximum_runtime_yaw_deviation_deg = 1.00;
    int relocalization_yaw_required_frames = 6;

    // ========== 连续退化门限 ==========
    // 连续坏帧数超过此值后进入 RELOCALIZING 而非 TRANSIENT_DEGRADED。
    int max_consecutive_degraded_frames = 3;
    // maybeRecover 的协方差膨胀因子（替代 P 重置）。
    double recovery_covariance_inflation = 4.0;
};

struct CraneMotionEKFStatus {
    bool initialized = false;
    bool ndt_accepted = false;
    bool diagonal_mode = false;
    bool recovered = false;
    bool prediction_only = false;
    bool step_limited = false;

    Eigen::Vector2d predicted_pos = Eigen::Vector2d::Zero();
    Eigen::Vector2d ndt_pos = Eigen::Vector2d::Zero();
    Eigen::Vector2d output_pos = Eigen::Vector2d::Zero();
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
    Eigen::Vector2d innovation = Eigen::Vector2d::Zero();

    double innovation_norm = 0.0;
    double lateral_error = 0.0;
    double tangential_error = 0.0;
    double measurement_r = 0.0;
    Eigen::Matrix2d measurement_covariance = Eigen::Matrix2d::Zero();
    double observability_ratio = 1.0;
    double weak_variance_inflation = 1.0;
    double p_trace = 0.0;
    double fitness = 0.0;
    double sensor_dt = 0.0;
    double ndt_time_ms = 0.0;
    double nis = 0.0;
    double output_step = 0.0;
    double max_allowed_step = 0.0;

    int frames_since_good_ndt = 0;
    int high_fitness_frames = 0;
    int reject_innovation_frames = 0;

    std::string reject_reason = "NONE";

    // ========== Yaw 锁存状态 ==========
    bool yaw_latched = false;
    double latched_yaw_rad = 0.0;
    int yaw_confirm_frames = 0;

    // ========== 连续退化跟踪 ==========
    int consecutive_degraded_frames = 0;
};

class CraneMotionEKF {
public:
    void setConfig(const CraneMotionEKFConfig& cfg) { cfg_ = cfg; }

    bool initialized() const { return initialized_; }

    void reset();

    void initialize(const Sophus::SE3d& first_pose, const ros::Time& stamp);

    Sophus::SE3d predictPose(const Sophus::SE3d& pose_template,
                             const ros::Time& stamp);

    // V3: 只读预测，不修改EKF状态，用于NDT initial guess
    Sophus::SE3d predictPoseReadOnly(const Sophus::SE3d& current_pose,
                                     double dt) const;

    Sophus::SE3d updateWithNDT(const Sophus::SE3d& ndt_pose,
                               double ndt_fitness,
                               const Sophus::SE3d& pose_template,
                               const ros::Time& stamp,
                               double ndt_time_ms = 0.0,
                               const NdtObservability* observability = nullptr);

    // Advance x/P/stamp when a measurement is unavailable or rejected.  A
    // read-only prediction must never be used as the published runtime state.
    Sophus::SE3d predictWithoutMeasurement(
        const Sophus::SE3d& pose_template,
        const ros::Time& stamp,
        const std::string& reason);

    // V3: 慢帧保护 - 传入 NDT 时间，返回额外的 R 增量
    double computeSlowFrameExtraR(double ndt_time_ms) const;

    // V3: 物理步长保护 - 检查 NDT 结果是否非物理
    bool isNonPhysicalStep(double raw_step, double ndt_time_ms, double dt) const;

    const CraneMotionEKFStatus& status() const { return status_; }

    const Eigen::Vector4d& state() const { return x_; }

    // v8-stable-r3-hotfix-minimal: 静止零速约束
    Sophus::SE3d applyStationaryConstraint(
        const Sophus::SE3d& pose_template,
        const Eigen::Vector2d& anchor_position,
        const ros::Time& stamp);

    // fix/588-runtime-localization-stable: 只清速度，不改位置
    void applyZeroVelocityConstraint();

    // ========== Yaw 锁存 ==========
    // 运行时检查 NDT yaw 是否在锁存范围内。返回 true 表示 yaw 可接受。
    bool checkRuntimeYaw(double ndt_yaw_rad, double dt);

    // 尝试锁存 yaw。需要连续多帧一致后才锁存。
    bool tryLatchYaw(double ndt_yaw_rad);

    // 解锁 yaw（进入 RELOCALIZING 时调用）。
    void unlatchYaw();

    // ========== 候选拒绝（替代裁剪提交） ==========
    // 当 candidate 超过物理上限时，返回 bounded prediction 而非裁剪后的 candidate。
    Sophus::SE3d rejectCandidate(
        const Eigen::Vector4d& x_pred,
        const Eigen::Matrix4d& P_pred,
        const Eigen::Vector2d& innovation,
        double raw_innov_norm,
        const Sophus::SE3d& pose_template,
        const ros::Time& stamp,
        const std::string& reason);

private:
    void predict(double dt, Eigen::Vector4d& x_pred, Eigen::Matrix4d& P_pred);
    void maybeRecover(const std::string& reason);
    double sanitizeDt(const ros::Time& stamp) const;
    double computeMaxOutputStep(double dt) const;
    bool enforceOutputStep(const Eigen::Vector2d& previous_pos,
                           double dt,
                           Eigen::Vector4d& state);
    void enforceVelocityAndAcceleration(const Eigen::Vector2d& previous_velocity,
                                        double dt,
                                        Eigen::Vector4d& state) const;

    Sophus::SE3d buildPoseFromState(const Eigen::Vector4d& state,
                                    const Sophus::SE3d& pose_template) const;

private:
    CraneMotionEKFConfig cfg_;
    CraneMotionEKFStatus status_;

    Eigen::Vector4d x_ = Eigen::Vector4d::Zero();   // [x, y, vx, vy]
    Eigen::Matrix4d P_ = Eigen::Matrix4d::Identity();

    // 最后一个被完全接受的测量状态（output_limited=false, innovation_accepted）。
    Eigen::Vector4d last_accepted_x_ = Eigen::Vector4d::Zero();
    Eigen::Matrix4d last_accepted_P_ = Eigen::Matrix4d::Identity();
    bool has_last_accepted_ = false;

    bool initialized_ = false;
    ros::Time last_stamp_;

    double last_good_fitness_ = 0.05;

    // Yaw 锁存内部状态
    double candidate_yaw_rad_ = 0.0;
    int yaw_consistent_count_ = 0;
    double last_accepted_yaw_rad_ = 0.0;
    bool yaw_ever_latched_ = false;
};

}  // namespace ndt_slam
