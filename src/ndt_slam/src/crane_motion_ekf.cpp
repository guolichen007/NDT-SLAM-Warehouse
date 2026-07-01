#include "ndt_slam/crane_motion_ekf.hpp"

namespace ndt_slam {

void CraneMotionEKF::initialize(const Sophus::SE3d& first_pose,
                                const ros::Time& stamp) {
    x_ << first_pose.translation().x(),
          first_pose.translation().y(),
          0.0,
          0.0;

    P_.setIdentity();
    P_(0, 0) = 1.0;
    P_(1, 1) = 1.0;
    P_(2, 2) = 2.0;
    P_(3, 3) = 2.0;

    initialized_ = true;
    last_stamp_ = stamp;

    status_ = CraneMotionEKFStatus();
    status_.initialized = true;
    status_.output_pos = x_.head<2>();
    status_.velocity = x_.tail<2>();

    ROS_INFO("[CraneMotionEKF] initialized at xy=(%.3f, %.3f)",
             x_(0), x_(1));
}

void CraneMotionEKF::predict(double dt,
                             Eigen::Vector4d& x_pred,
                             Eigen::Matrix4d& P_pred) {
    dt = std::max(1e-3, std::min(dt, 0.5));

    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;

    x_pred = F * x_;

    x_pred(2) = std::clamp(x_pred(2), -cfg_.max_speed_x, cfg_.max_speed_x);
    x_pred(3) = std::clamp(x_pred(3), -cfg_.max_speed_y, cfg_.max_speed_y);

    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();

    // 简化恒速模型：位置噪声 + 速度噪声
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;

    Q(0, 0) = cfg_.q_pos * dt + 0.25 * cfg_.q_vel * dt4;
    Q(1, 1) = Q(0, 0);
    Q(2, 2) = cfg_.q_vel * dt2;
    Q(3, 3) = Q(2, 2);

    Q(0, 2) = 0.5 * cfg_.q_vel * dt3;
    Q(2, 0) = Q(0, 2);
    Q(1, 3) = Q(0, 2);
    Q(3, 1) = Q(1, 3);

    P_pred = F * P_ * F.transpose() + Q;
}

Sophus::SE3d CraneMotionEKF::predictPose(const Sophus::SE3d& pose_template,
                                         const ros::Time& stamp) {
    if (!initialized_) {
        return pose_template;
    }

    const double dt = (stamp - last_stamp_).toSec();

    Eigen::Vector4d x_pred;
    Eigen::Matrix4d P_pred;
    predict(dt, x_pred, P_pred);

    status_.predicted_pos = x_pred.head<2>();
    status_.velocity = x_pred.tail<2>();

    return buildPoseFromState(x_pred, pose_template);
}

Sophus::SE3d CraneMotionEKF::updateWithNDT(const Sophus::SE3d& ndt_pose,
                                           double ndt_fitness,
                                           const Sophus::SE3d& pose_template,
                                           const ros::Time& stamp) {
    if (!initialized_) {
        initialize(ndt_pose, stamp);
        return ndt_pose;
    }

    const double dt = (stamp - last_stamp_).toSec();

    Eigen::Vector4d x_pred;
    Eigen::Matrix4d P_pred;
    predict(dt, x_pred, P_pred);

    Eigen::Vector2d z(ndt_pose.translation().x(),
                      ndt_pose.translation().y());

    Eigen::Vector2d z_pred = x_pred.head<2>();
    Eigen::Vector2d innovation = z - z_pred;

    status_.ndt_pos = z;
    status_.predicted_pos = z_pred;
    status_.fitness = ndt_fitness;
    status_.reject_reason = "NONE";

    const double raw_innov_norm = innovation.norm();

    // 完全拒绝：NDT 明显跳飞，且 fitness 比历史好帧差很多
    if (raw_innov_norm > cfg_.innovation_reject_m &&
        ndt_fitness > std::max(0.03, last_good_fitness_ * 1.5)) {
        x_ = x_pred;
        P_ = P_pred;
        status_.ndt_accepted = false;
        status_.frames_since_good_ndt++;
        status_.reject_innovation_frames++;
        status_.reject_reason = "INNOVATION_REJECT";

        maybeRecover("innovation_reject");

        last_stamp_ = stamp;
        status_.output_pos = x_.head<2>();
        status_.velocity = x_.tail<2>();
        status_.innovation = innovation;
        status_.innovation_norm = raw_innov_norm;
        status_.p_trace = P_.trace();

        return buildPoseFromState(x_, pose_template);
    }

    // 斜向运动门控
    status_.diagonal_mode = false;
    status_.lateral_error = 0.0;
    status_.tangential_error = 0.0;

    const double vx = x_pred(2);
    const double vy = x_pred(3);
    const double speed = std::hypot(vx, vy);

    if (cfg_.diagonal_enabled &&
        speed > cfg_.diagonal_min_speed &&
        std::abs(vx) > cfg_.diagonal_min_vx &&
        std::abs(vy) > cfg_.diagonal_min_vy) {
        status_.diagonal_mode = true;

        Eigen::Vector2d v_dir(vx, vy);
        v_dir.normalize();
        Eigen::Vector2d n_dir(-v_dir.y(), v_dir.x());

        status_.lateral_error = innovation.dot(n_dir);
        status_.tangential_error = innovation.dot(v_dir);

        if (std::abs(status_.lateral_error) > cfg_.lateral_gate_m) {
            const double sign = status_.lateral_error > 0.0 ? 1.0 : -1.0;
            const double excess =
                status_.lateral_error - sign * cfg_.lateral_gate_m;
            innovation -= n_dir * excess * cfg_.lateral_damping;
        }

        if (std::abs(status_.tangential_error) > cfg_.tangential_gate_m) {
            const double sign = status_.tangential_error > 0.0 ? 1.0 : -1.0;
            const double excess =
                status_.tangential_error - sign * cfg_.tangential_gate_m;
            innovation -= v_dir * excess * cfg_.tangential_damping;
        }
    }

    const double gated_norm = innovation.norm();
    if (gated_norm > cfg_.innovation_gate_m) {
        innovation *= (cfg_.innovation_gate_m / gated_norm);
    }

    // fitness 越差，R 越大，NDT 权重越小
    double r = cfg_.r_ndt_base +
               cfg_.fitness_to_r_scale * ndt_fitness * ndt_fitness;
    r = std::min(r, cfg_.r_ndt_max);

    Eigen::Matrix2d R = r * Eigen::Matrix2d::Identity();

    Eigen::Matrix<double, 2, 4> H;
    H.setZero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    Eigen::Matrix2d S = H * P_pred * H.transpose() + R;
    Eigen::Matrix<double, 4, 2> K =
        P_pred * H.transpose() * S.inverse();

    x_ = x_pred + K * innovation;
    P_ = (Eigen::Matrix4d::Identity() - K * H) * P_pred;

    status_.ndt_accepted = true;
    status_.frames_since_good_ndt = 0;
    status_.reject_innovation_frames = 0;

    if (ndt_fitness > cfg_.high_fitness_threshold) {
        status_.high_fitness_frames++;
    } else {
        status_.high_fitness_frames = 0;
        last_good_fitness_ = ndt_fitness;
    }

    if (status_.high_fitness_frames > cfg_.max_high_fitness_frames) {
        maybeRecover("high_fitness");
    }

    last_stamp_ = stamp;

    status_.innovation = innovation;
    status_.innovation_norm = innovation.norm();
    status_.measurement_r = r;
    status_.p_trace = P_.trace();
    status_.output_pos = x_.head<2>();
    status_.velocity = x_.tail<2>();

    return buildPoseFromState(x_, pose_template);
}

void CraneMotionEKF::maybeRecover(const std::string& reason) {
    if (status_.frames_since_good_ndt <= cfg_.max_frames_since_good_ndt &&
        status_.reject_innovation_frames <= cfg_.max_reject_innovation_frames &&
        status_.high_fitness_frames <= cfg_.max_high_fitness_frames) {
        return;
    }

    // 恢复时保留状态，但重置不确定性，让后续好帧重新拉回
    P_.setIdentity();
    P_(0, 0) = 0.5;
    P_(1, 1) = 0.5;
    P_(2, 2) = 1.0;
    P_(3, 3) = 1.0;

    status_.frames_since_good_ndt = 0;
    status_.reject_innovation_frames = 0;
    status_.high_fitness_frames = 0;
    status_.recovered = true;

    ROS_WARN("[CraneMotionEKF] recovery: reason=%s, P reset",
             reason.c_str());
}

Sophus::SE3d CraneMotionEKF::buildPoseFromState(
    const Eigen::Vector4d& state,
    const Sophus::SE3d& pose_template) const {
    Sophus::SE3d out = pose_template;
    out.translation().x() = state(0);
    out.translation().y() = state(1);
    return out;
}

// v8-stable-r3-hotfix-minimal: 静止零速约束
void CraneMotionEKF::applyStationaryConstraint(const Eigen::Vector2d& anchor_xy) {
    // 强制位置到锚点
    x_(0) = anchor_xy.x();
    x_(1) = anchor_xy.y();
    // 强制速度为零
    x_(2) = 0.0;
    x_(3) = 0.0;

    // 收紧不确定性：位置 ±5cm，速度 ±0.02m/s
    P_(0, 0) = 0.0025;
    P_(1, 1) = 0.0025;
    P_(2, 2) = 0.0004;
    P_(3, 3) = 0.0004;

    // 更新状态输出
    status_.output_pos = x_.head<2>();
    status_.velocity = x_.tail<2>();
}

}  // namespace ndt_slam
