#pragma once

#include <Eigen/Dense>
#include <ros/ros.h>
#include <sophus/se3.hpp>
#include <algorithm>
#include <cmath>
#include <string>

namespace ndt_slam {

struct CraneMotionEKFConfig {
    bool enabled = true;

    double q_pos = 0.05;
    double q_vel = 0.30;

    double r_ndt_base = 0.02;
    double r_ndt_max = 2.0;
    double fitness_to_r_scale = 5.0;

    double innovation_gate_m = 0.50;
    double innovation_reject_m = 1.50;

    double max_speed_x = 2.0;
    double max_speed_y = 2.0;

    bool diagonal_enabled = true;
    double diagonal_min_vx = 0.05;
    double diagonal_min_vy = 0.05;
    double diagonal_min_speed = 0.10;
    double lateral_gate_m = 0.20;
    double tangential_gate_m = 0.40;
    double lateral_damping = 0.70;
    double tangential_damping = 0.40;

    int max_frames_since_good_ndt = 30;
    int max_high_fitness_frames = 10;
    int max_reject_innovation_frames = 5;
    double high_fitness_threshold = 0.15;
};

struct CraneMotionEKFStatus {
    bool initialized = false;
    bool ndt_accepted = false;
    bool diagonal_mode = false;
    bool recovered = false;

    Eigen::Vector2d predicted_pos = Eigen::Vector2d::Zero();
    Eigen::Vector2d ndt_pos = Eigen::Vector2d::Zero();
    Eigen::Vector2d output_pos = Eigen::Vector2d::Zero();
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
    Eigen::Vector2d innovation = Eigen::Vector2d::Zero();

    double innovation_norm = 0.0;
    double lateral_error = 0.0;
    double tangential_error = 0.0;
    double measurement_r = 0.0;
    double p_trace = 0.0;
    double fitness = 0.0;

    int frames_since_good_ndt = 0;
    int high_fitness_frames = 0;
    int reject_innovation_frames = 0;

    std::string reject_reason = "NONE";
};

class CraneMotionEKF {
public:
    void setConfig(const CraneMotionEKFConfig& cfg) { cfg_ = cfg; }

    bool initialized() const { return initialized_; }

    void initialize(const Sophus::SE3d& first_pose, const ros::Time& stamp);

    Sophus::SE3d predictPose(const Sophus::SE3d& pose_template,
                             const ros::Time& stamp);

    Sophus::SE3d updateWithNDT(const Sophus::SE3d& ndt_pose,
                               double ndt_fitness,
                               const Sophus::SE3d& pose_template,
                               const ros::Time& stamp);

    const CraneMotionEKFStatus& status() const { return status_; }

    const Eigen::Vector4d& state() const { return x_; }

    // v8-stable-r3-hotfix-minimal: 静止零速约束
    void applyStationaryConstraint(const Eigen::Vector2d& anchor_xy);

private:
    void predict(double dt, Eigen::Vector4d& x_pred, Eigen::Matrix4d& P_pred);
    void maybeRecover(const std::string& reason);

    Sophus::SE3d buildPoseFromState(const Eigen::Vector4d& state,
                                    const Sophus::SE3d& pose_template) const;

private:
    CraneMotionEKFConfig cfg_;
    CraneMotionEKFStatus status_;

    Eigen::Vector4d x_ = Eigen::Vector4d::Zero();   // [x, y, vx, vy]
    Eigen::Matrix4d P_ = Eigen::Matrix4d::Identity();

    bool initialized_ = false;
    ros::Time last_stamp_;

    double last_good_fitness_ = 0.05;
};

}  // namespace ndt_slam
