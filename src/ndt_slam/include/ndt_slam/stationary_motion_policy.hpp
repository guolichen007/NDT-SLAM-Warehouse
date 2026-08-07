#pragma once

#include <Eigen/Core>

#include <deque>
#include <string>

namespace ndt_slam {

enum class RuntimeMotionState {
    MOVING,
    STATIONARY_HOLD,
    MOVING_CONFIRM,
    CATCH_UP,
};

const char* runtimeMotionStateName(RuntimeMotionState state);

struct StationaryMotionPolicyConfig {
    int enter_confirm_frames = 20;
    double enter_max_raw_increment_m = 0.015;
    double enter_max_speed_mps = 0.03;

    int exit_confirm_frames = 3;
    double exit_min_increment_m = 0.02;
    double exit_cumulative_motion_m = 0.15;
    double exit_direction_cosine_min = 0.80;
    double exit_evidence_window_sec = 1.50;
    double exit_min_speed_mps = 0.01;
    double exit_force_anchor_drift_m = 0.30;
    double moving_confirm_timeout_sec = 1.50;

    double catch_up_max_step_m = 0.08;
    double catch_up_complete_error_m = 0.03;
    int catch_up_confirm_frames = 2;

    double timestamp_epsilon_sec = 1.0e-6;
};

struct StationaryMotionInput {
    double stamp_sec = 0.0;

    bool ndt_converged = false;
    bool ndt_accepted = false;
    bool prediction_only = false;
    bool registration_quality_valid = false;
    // Persistent maps require the stricter EKF/map-commit gate.  Ephemeral
    // local-map tracking may still use an accepted, finite soft correction.
    bool persistent_map_quality_valid = false;
    bool severe_degeneracy = false;

    Eigen::Vector2d raw_position = Eigen::Vector2d::Zero();
    Eigen::Vector2d filtered_position = Eigen::Vector2d::Zero();
    Eigen::Vector2d filtered_velocity = Eigen::Vector2d::Zero();

    double raw_increment_m = 0.0;
    double allowed_physical_step_m = 0.0;
};

struct StationaryMotionDecision {
    RuntimeMotionState state = RuntimeMotionState::MOVING;

    bool apply_stationary_hold = false;
    bool apply_position_constraint = false;
    bool allow_local_map_update = false;
    bool allow_persistent_map_commit = false;

    bool movement_confirmed = false;
    bool start_catch_up = false;

    Eigen::Vector2d constrained_position = Eigen::Vector2d::Zero();
    double catch_up_step_m = 0.0;
    std::string reason = "RESET";
};

// Pure timestamp-driven policy. It owns only motion evidence; it never writes
// an EKF, runtime pose, local map, or persistent map.
class StationaryMotionPolicy {
public:
    void setConfig(const StationaryMotionPolicyConfig& config);
    void reset();

    StationaryMotionDecision update(const StationaryMotionInput& input);

    RuntimeMotionState state() const { return state_; }
    const Eigen::Vector2d& anchorPosition() const { return anchor_position_; }

private:
    bool isFreshTimestamp(double stamp_sec) const;
    bool isReliableMeasurement(const StationaryMotionInput& input) const;
    void enterStationary(const StationaryMotionInput& input);
    void beginMovementConfirmation(double stamp_sec);
    void rejectMovementEvidence(const StationaryMotionInput& input,
                                const std::string& reason);
    void appendMotionSample(double stamp_sec,
                            const Eigen::Vector2d& raw_position);
    void pruneMotionSamples(double stamp_sec);
    StationaryMotionDecision baseDecision(
        const StationaryMotionInput& input) const;

    struct MotionSample {
        double stamp_sec = 0.0;
        Eigen::Vector2d raw_position = Eigen::Vector2d::Zero();
    };

    StationaryMotionPolicyConfig config_;
    RuntimeMotionState state_ = RuntimeMotionState::MOVING;

    bool has_last_stamp_ = false;
    double last_stamp_sec_ = 0.0;
    bool has_last_raw_position_ = false;
    Eigen::Vector2d last_raw_position_ = Eigen::Vector2d::Zero();

    Eigen::Vector2d anchor_position_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d raw_anchor_position_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d accumulated_motion_ = Eigen::Vector2d::Zero();
    double confirmed_path_length_m_ = 0.0;
    std::deque<MotionSample> motion_samples_;
    double movement_confirm_start_stamp_sec_ = 0.0;

    int stationary_enter_count_ = 0;
    int movement_confirm_count_ = 0;
    int catch_up_complete_count_ = 0;
};

}  // namespace ndt_slam
