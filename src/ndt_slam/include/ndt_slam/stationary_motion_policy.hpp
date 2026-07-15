#pragma once

#include <Eigen/Core>

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

    double catch_up_max_step_m = 0.08;
    double catch_up_complete_error_m = 0.10;
    int catch_up_confirm_frames = 2;

    double timestamp_epsilon_sec = 1.0e-6;
};

struct StationaryMotionInput {
    double stamp_sec = 0.0;

    bool ndt_converged = false;
    bool ndt_accepted = false;
    bool prediction_only = false;
    bool registration_quality_valid = false;
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
    void beginMovementConfirmation(const Eigen::Vector2d& delta);
    void rejectMovementEvidence(const StationaryMotionInput& input,
                                const std::string& reason);
    StationaryMotionDecision baseDecision(
        const StationaryMotionInput& input) const;

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

    int stationary_enter_count_ = 0;
    int movement_confirm_count_ = 0;
    int catch_up_complete_count_ = 0;
};

}  // namespace ndt_slam
