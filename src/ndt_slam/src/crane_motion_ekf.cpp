#include "ndt_slam/crane_motion_ekf.hpp"

#include <limits>

namespace ndt_slam {

void CraneMotionEKF::reset() {
    x_.setZero();
    P_.setIdentity();
    initialized_ = false;
    last_stamp_ = ros::Time();
    last_good_fitness_ = 0.05;
    status_ = CraneMotionEKFStatus{};
}

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
    status_.ndt_accepted = true;
    status_.ndt_pos = x_.head<2>();
    status_.predicted_pos = x_.head<2>();
    status_.output_pos = x_.head<2>();
    status_.velocity = x_.tail<2>();

    ROS_INFO("[CraneMotionEKF] initialized at xy=(%.3f, %.3f)",
             x_(0), x_(1));
}

double CraneMotionEKF::sanitizeDt(const ros::Time& stamp) const {
    const double raw_dt = (stamp - last_stamp_).toSec();
    if (!std::isfinite(raw_dt) || raw_dt <= 1e-4 || raw_dt > 1.0) {
        return 0.10;
    }
    return raw_dt;
}

double CraneMotionEKF::computeMaxOutputStep(double dt) const {
    dt = std::max(1e-3, std::min(dt, 1.0));
    const double accel_norm = std::hypot(cfg_.max_accel_x, cfg_.max_accel_y);
    const double physical_step =
        cfg_.max_speed_mps * dt * cfg_.max_step_safety_factor +
        0.5 * accel_norm * dt * dt;
    return std::clamp(physical_step,
                      cfg_.max_step_min_m,
                      cfg_.max_step_max_m);
}

void CraneMotionEKF::enforceVelocityAndAcceleration(
    const Eigen::Vector2d& previous_velocity,
    double dt,
    Eigen::Vector4d& state) const {
    dt = std::max(1e-3, std::min(dt, 1.0));

    const double dv_x = std::max(0.0, cfg_.max_accel_x) * dt;
    const double dv_y = std::max(0.0, cfg_.max_accel_y) * dt;
    state(2) = std::clamp(state(2),
                          previous_velocity.x() - dv_x,
                          previous_velocity.x() + dv_x);
    state(3) = std::clamp(state(3),
                          previous_velocity.y() - dv_y,
                          previous_velocity.y() + dv_y);
    state(2) = std::clamp(state(2), -cfg_.max_speed_x, cfg_.max_speed_x);
    state(3) = std::clamp(state(3), -cfg_.max_speed_y, cfg_.max_speed_y);

    const double speed = std::hypot(state(2), state(3));
    if (cfg_.max_speed_mps > 0.0 && speed > cfg_.max_speed_mps) {
        const double scale = cfg_.max_speed_mps / speed;
        state(2) *= scale;
        state(3) *= scale;
    }
}

bool CraneMotionEKF::enforceOutputStep(
    const Eigen::Vector2d& previous_pos,
    double dt,
    Eigen::Vector4d& state) {
    const Eigen::Vector2d delta = state.head<2>() - previous_pos;
    const double step = delta.norm();
    const double max_step = computeMaxOutputStep(dt);
    status_.output_step = step;
    status_.max_allowed_step = max_step;
    status_.step_limited = false;

    if (!std::isfinite(step)) {
        state.head<2>() = previous_pos;
        state.tail<2>().setZero();
        status_.output_step = 0.0;
        status_.step_limited = true;
        return true;
    }

    if (step <= max_step || step < 1e-9) {
        return false;
    }

    const Eigen::Vector2d limited_delta = delta * (max_step / step);
    state.head<2>() = previous_pos + limited_delta;
    state.tail<2>() = limited_delta / std::max(dt, 1e-3);
    state(2) = std::clamp(state(2), -cfg_.max_speed_x, cfg_.max_speed_x);
    state(3) = std::clamp(state(3), -cfg_.max_speed_y, cfg_.max_speed_y);
    status_.output_step = max_step;
    status_.step_limited = true;
    return true;
}

void CraneMotionEKF::predict(double dt,
                             Eigen::Vector4d& x_pred,
                             Eigen::Matrix4d& P_pred) {
    dt = std::max(1e-3, std::min(dt, 1.0));

    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;

    Eigen::Vector4d bounded_state = x_;
    bounded_state(2) = std::clamp(
        bounded_state(2), -cfg_.max_speed_x, cfg_.max_speed_x);
    bounded_state(3) = std::clamp(
        bounded_state(3), -cfg_.max_speed_y, cfg_.max_speed_y);
    const double speed = bounded_state.tail<2>().norm();
    if (cfg_.max_speed_mps > 0.0 && speed > cfg_.max_speed_mps) {
        bounded_state.tail<2>() *= cfg_.max_speed_mps / speed;
    }
    x_pred = F * bounded_state;

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

    const double dt = sanitizeDt(stamp);

    Eigen::Vector4d x_pred;
    Eigen::Matrix4d P_pred;
    predict(dt, x_pred, P_pred);

    status_.predicted_pos = x_pred.head<2>();
    status_.velocity = x_pred.tail<2>();

    return buildPoseFromState(x_pred, pose_template);
}

Sophus::SE3d CraneMotionEKF::predictPoseReadOnly(const Sophus::SE3d& current_pose,
                                                  double dt) const {
    if (!initialized_) {
        return current_pose;
    }

    // 限制 dt 范围，避免异常值
    dt = std::max(1e-3, std::min(dt, 1.0));

    Eigen::Vector4d x_pred = x_;
    x_pred(2) = std::clamp(x_pred(2), -cfg_.max_speed_x, cfg_.max_speed_x);
    x_pred(3) = std::clamp(x_pred(3), -cfg_.max_speed_y, cfg_.max_speed_y);
    const double speed = x_pred.tail<2>().norm();
    if (cfg_.max_speed_mps > 0.0 && speed > cfg_.max_speed_mps) {
        x_pred.tail<2>() *= cfg_.max_speed_mps / speed;
    }
    x_pred(0) += x_pred(2) * dt;
    x_pred(1) += x_pred(3) * dt;

    // 构建预测 pose：只修改 x/y，保持 z/roll/pitch/yaw 不变
    Sophus::SE3d predicted = current_pose;
    predicted.translation().x() = x_pred(0);
    predicted.translation().y() = x_pred(1);

    return predicted;
}

Sophus::SE3d CraneMotionEKF::predictWithoutMeasurement(
    const Sophus::SE3d& pose_template,
    const ros::Time& stamp,
    const std::string& reason) {
    if (!initialized_) {
        status_.ndt_accepted = false;
        status_.prediction_only = true;
        status_.reject_reason = reason;
        return pose_template;
    }

    const double dt = sanitizeDt(stamp);
    const Eigen::Vector2d previous_pos = x_.head<2>();

    Eigen::Vector4d x_pred;
    Eigen::Matrix4d P_pred;
    predict(dt, x_pred, P_pred);
    const bool limited = enforceOutputStep(previous_pos, dt, x_pred);

    x_ = x_pred;
    P_ = P_pred;
    last_stamp_ = stamp;

    status_.initialized = true;
    status_.ndt_accepted = false;
    status_.prediction_only = true;
    status_.recovered = false;
    status_.frames_since_good_ndt++;
    status_.predicted_pos = x_.head<2>();
    status_.output_pos = x_.head<2>();
    status_.velocity = x_.tail<2>();
    status_.innovation.setZero();
    status_.innovation_norm = 0.0;
    status_.measurement_r = 0.0;
    status_.measurement_covariance.setZero();
    status_.observability_ratio = 0.0;
    status_.weak_variance_inflation = 1.0;
    status_.nis = 0.0;
    status_.sensor_dt = dt;
    status_.ndt_time_ms = 0.0;
    status_.p_trace = P_.trace();
    status_.reject_reason = limited ? reason + "_STEP_LIMIT" : reason;

    maybeRecover(reason);
    return buildPoseFromState(x_, pose_template);
}

Sophus::SE3d CraneMotionEKF::updateWithNDT(const Sophus::SE3d& ndt_pose,
                                           double ndt_fitness,
                                           const Sophus::SE3d& pose_template,
                                           const ros::Time& stamp,
                                           double ndt_time_ms,
                                           const NdtObservability* observability) {
    if (!initialized_) {
        initialize(ndt_pose, stamp);
        status_.fitness = ndt_fitness;
        status_.ndt_time_ms = ndt_time_ms;
        return ndt_pose;
    }

    status_.recovered = false;
    status_.prediction_only = false;
    status_.step_limited = false;

    // High fitness is target-density dependent.  Production defaults to
    // covariance inflation; this hard reject remains an explicit opt-in.
    if (cfg_.reject_high_fitness && ndt_fitness > cfg_.ndt_fitness_reject_threshold) {
        status_.high_fitness_frames++;
        status_.fitness = ndt_fitness;
        status_.ndt_pos = ndt_pose.translation().head<2>();
        status_.ndt_time_ms = ndt_time_ms;

        ROS_WARN_THROTTLE(1.0, "[EKFReject] reason=high_fitness fitness=%.3f threshold=%.3f use=prediction",
                         ndt_fitness, cfg_.ndt_fitness_reject_threshold);
        Sophus::SE3d predicted = predictWithoutMeasurement(
            pose_template, stamp, "HIGH_FITNESS");
        status_.fitness = ndt_fitness;
        status_.ndt_time_ms = ndt_time_ms;
        return predicted;
    }

    const double dt = sanitizeDt(stamp);
    const Eigen::Vector2d previous_pos = x_.head<2>();
    const Eigen::Vector2d previous_velocity = x_.tail<2>();

    Eigen::Vector4d x_pred;
    Eigen::Matrix4d P_pred;
    predict(dt, x_pred, P_pred);

    Eigen::Vector2d z(ndt_pose.translation().x(),
                      ndt_pose.translation().y());

    Eigen::Vector2d z_pred = x_pred.head<2>();
    Eigen::Vector2d innovation = z - z_pred;
    const Eigen::Vector2d raw_innovation = innovation;

    status_.ndt_pos = z;
    status_.predicted_pos = z_pred;
    status_.fitness = ndt_fitness;
    status_.reject_reason = "NONE";
    status_.sensor_dt = dt;
    status_.ndt_time_ms = ndt_time_ms;

    const double raw_innov_norm = innovation.norm();

    // Fitness scales measurement covariance and slow frames receive an
    // additional penalty.  This avoids a binary fitness cliff.
    double r = cfg_.r_ndt_base +
               cfg_.fitness_to_r_scale * ndt_fitness * ndt_fitness +
               computeSlowFrameExtraR(ndt_time_ms);
    r = std::clamp(r, cfg_.r_ndt_base, cfg_.r_ndt_max);

    const NdtObservability isotropic_observability;
    const NdtObservability& frame_observability = observability
        ? *observability
        : isotropic_observability;
    Eigen::Matrix2d R = observability
        ? buildObservabilityAwareMeasurementCovariance(
              r, frame_observability, cfg_.observability)
        : r * Eigen::Matrix2d::Identity();
    Eigen::Matrix<double, 2, 4> H;
    H.setZero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    Eigen::Matrix2d S = H * P_pred * H.transpose() + R;
    const double nis = raw_innovation.dot(S.ldlt().solve(raw_innovation));
    status_.nis = std::isfinite(nis) ? nis : std::numeric_limits<double>::infinity();
    status_.measurement_r = r;
    status_.measurement_covariance = R;
    status_.observability_ratio = observability && observability->valid
        ? observability->eigenvalue_ratio
        : 1.0;
    status_.weak_variance_inflation =
        observability && observability->valid &&
                observability->severely_degenerate
            ? std::max(1.0, cfg_.observability.severe_weak_inflation)
            : observability && observability->valid &&
                    observability->degenerate
                ? std::max(
                      1.0,
                      cfg_.observability.moderate_weak_inflation)
                : 1.0;

    // Reject only when motion inconsistency and degraded matching quality
    // agree.  The per-axis test preserves legitimate diagonal motion.
    const bool observability_directional_gate =
        observability && observability->valid && observability->degenerate;
    const bool gross_axis_innovation = observability_directional_gate
        ? std::abs(raw_innovation.dot(
              observability->strong_direction)) > cfg_.innovation_reject_m
        : std::abs(raw_innovation.x()) > cfg_.innovation_reject_m ||
            std::abs(raw_innovation.y()) > cfg_.innovation_reject_m;
    const bool nis_outlier = status_.nis > cfg_.nis_reject_threshold;
    if ((gross_axis_innovation || nis_outlier) &&
        ndt_fitness > std::max(0.03, last_good_fitness_ * 1.5)) {
        enforceOutputStep(previous_pos, dt, x_pred);
        x_ = x_pred;
        P_ = P_pred;
        status_.ndt_accepted = false;
        status_.prediction_only = true;
        status_.frames_since_good_ndt++;
        status_.reject_innovation_frames++;
        status_.reject_reason = nis_outlier
            ? "NIS_INNOVATION_REJECT"
            : "AXIS_INNOVATION_REJECT";

        maybeRecover("innovation_reject");

        last_stamp_ = stamp;
        status_.output_pos = x_.head<2>();
        status_.velocity = x_.tail<2>();
        status_.innovation = innovation;
        status_.innovation_norm = raw_innov_norm;
        status_.p_trace = P_.trace();

        return buildPoseFromState(x_, pose_template);
    }

    status_.diagonal_mode = false;
    status_.lateral_error = 0.0;
    status_.tangential_error = 0.0;

    const double vx = x_pred(2);
    const double vy = x_pred(3);
    const double speed = std::hypot(vx, vy);

    if (observability_directional_gate) {
        double strong_innovation = innovation.dot(
            observability->strong_direction);
        double weak_innovation = innovation.dot(
            observability->weak_direction);
        strong_innovation = std::clamp(
            strong_innovation, -cfg_.innovation_gate_m,
            cfg_.innovation_gate_m);
        weak_innovation = std::clamp(
            weak_innovation, -cfg_.innovation_gate_m,
            cfg_.innovation_gate_m);
        innovation = strong_innovation *
                observability->strong_direction +
            weak_innovation * observability->weak_direction;
    } else if (cfg_.axis_independent_gate) {
        innovation.x() = std::clamp(
            innovation.x(), -cfg_.innovation_gate_m, cfg_.innovation_gate_m);
        innovation.y() = std::clamp(
            innovation.y(), -cfg_.innovation_gate_m, cfg_.innovation_gate_m);
    } else if (cfg_.diagonal_enabled &&
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

    if (!cfg_.axis_independent_gate) {
        const double gated_norm = innovation.norm();
        if (gated_norm > cfg_.innovation_gate_m) {
            innovation *= (cfg_.innovation_gate_m / gated_norm);
        }
    }

    Eigen::Matrix<double, 4, 2> K =
        P_pred * H.transpose() * S.inverse();

    Eigen::Vector4d candidate = x_pred + K * innovation;
    enforceVelocityAndAcceleration(previous_velocity, dt, candidate);
    const double unconstrained_output_step =
        (candidate.head<2>() - previous_pos).norm();
    const bool output_limited = enforceOutputStep(previous_pos, dt, candidate);
    x_ = candidate;

    // Joseph form keeps P symmetric positive semi-definite under finite
    // precision and repeated covariance inflation.
    const Eigen::Matrix4d I_KH = Eigen::Matrix4d::Identity() - K * H;
    P_ = I_KH * P_pred * I_KH.transpose() + K * R * K.transpose();

    status_.ndt_accepted = !output_limited;
    status_.prediction_only = false;
    if (output_limited) {
        status_.frames_since_good_ndt++;
        status_.reject_innovation_frames++;
        status_.reject_reason = "OUTPUT_STEP_LIMIT";
        ROS_WARN_THROTTLE(
            1.0,
            "[EKFGuard] output step limited raw=%.3f max=%.3f dt=%.3f",
            unconstrained_output_step,
            status_.max_allowed_step, dt);
        maybeRecover("output_step_limit");
    } else {
        status_.frames_since_good_ndt = 0;
        status_.reject_innovation_frames = 0;
    }

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
    status_.innovation_norm = raw_innov_norm;
    status_.measurement_r = r;
    status_.measurement_covariance = R;
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

Sophus::SE3d CraneMotionEKF::applyStationaryConstraint(
    const Sophus::SE3d& pose_template,
    const Eigen::Vector2d& anchor_position,
    const ros::Time& stamp) {
    if (!initialized_ || !anchor_position.allFinite()) {
        return pose_template;
    }

    Eigen::Matrix<double, 2, 4> H;
    H.setZero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    const double position_variance =
        std::max(1.0e-9, cfg_.stationary_position_hold_variance);
    const Eigen::Matrix2d R =
        position_variance * Eigen::Matrix2d::Identity();
    const Eigen::Matrix2d S = H * P_ * H.transpose() + R;
    const Eigen::Matrix<double, 4, 2> K =
        P_ * H.transpose() * S.ldlt().solve(Eigen::Matrix2d::Identity());
    x_ += K * (anchor_position - x_.head<2>());
    const Eigen::Matrix4d I_KH = Eigen::Matrix4d::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();

    // The zero-drift hold model is an equality constraint after the
    // pseudo-measurement covariance update.
    x_.head<2>() = anchor_position;
    x_.tail<2>().setZero();
    P_.block<2, 2>(0, 2).setZero();
    P_.block<2, 2>(2, 0).setZero();
    P_(0, 0) = std::min(P_(0, 0), position_variance);
    P_(1, 1) = std::min(P_(1, 1), position_variance);
    const double velocity_variance =
        std::max(1.0e-9, cfg_.stationary_velocity_hold_variance);
    P_(2, 2) = std::min(P_(2, 2), velocity_variance);
    P_(3, 3) = std::min(P_(3, 3), velocity_variance);

    last_stamp_ = stamp;
    status_.initialized = true;
    status_.output_pos = x_.head<2>();
    status_.velocity = x_.tail<2>();
    status_.p_trace = P_.trace();
    return buildPoseFromState(x_, pose_template);
}

// fix/588-runtime-localization-stable: 只清速度，不改位置
void CraneMotionEKF::applyZeroVelocityConstraint() {
    if (!initialized_) {
        return;
    }

    // 只零速，不改位置
    x_(2) = 0.0;  // vx
    x_(3) = 0.0;  // vy

    // 只收紧速度协方差，不收紧位置协方差
    P_(2, 2) = std::min(P_(2, 2), 0.0004);
    P_(3, 3) = std::min(P_(3, 3), 0.0004);

    // P_(0,0)、P_(1,1) 不允许在这里修改

    status_.output_pos = x_.head<2>();
    status_.velocity = x_.tail<2>();
}

// V3: 慢帧保护 - 计算额外的 R 增量
double CraneMotionEKF::computeSlowFrameExtraR(double ndt_time_ms) const {
    if (!cfg_.slow_frame_guard_enabled) {
        return 0.0;
    }

    if (ndt_time_ms > cfg_.slow_frame_emergency_ms) {
        return 2.0 * cfg_.slow_frame_extra_r;
    }

    if (ndt_time_ms > cfg_.slow_frame_warn_ms) {
        return cfg_.slow_frame_extra_r;
    }

    return 0.0;
}

// V3: 物理步长保护 - 检查 NDT 结果是否非物理
bool CraneMotionEKF::isNonPhysicalStep(double raw_step, double ndt_time_ms, double dt) const {
    (void)ndt_time_ms;
    return raw_step > computeMaxOutputStep(dt);
}

}  // namespace ndt_slam
