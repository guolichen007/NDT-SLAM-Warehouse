#include "ndt_slam/stationary_motion_policy.hpp"

#include <algorithm>
#include <cmath>

namespace ndt_slam {

const char* runtimeMotionStateName(RuntimeMotionState state) {
    switch (state) {
        case RuntimeMotionState::MOVING:
            return "MOVING";
        case RuntimeMotionState::STATIONARY_HOLD:
            return "STATIONARY_HOLD";
        case RuntimeMotionState::MOVING_CONFIRM:
            return "MOVING_CONFIRM";
        case RuntimeMotionState::CATCH_UP:
            return "CATCH_UP";
    }
    return "UNKNOWN";
}

void StationaryMotionPolicy::setConfig(
    const StationaryMotionPolicyConfig& config) {
    config_ = config;
    config_.enter_confirm_frames = std::max(1, config_.enter_confirm_frames);
    config_.exit_confirm_frames = std::max(1, config_.exit_confirm_frames);
    config_.catch_up_confirm_frames =
        std::max(1, config_.catch_up_confirm_frames);
    config_.enter_max_raw_increment_m =
        std::max(0.0, config_.enter_max_raw_increment_m);
    config_.enter_max_speed_mps = std::max(0.0, config_.enter_max_speed_mps);
    config_.exit_min_increment_m =
        std::max(0.0, config_.exit_min_increment_m);
    config_.exit_cumulative_motion_m =
        std::max(config_.exit_min_increment_m,
                 config_.exit_cumulative_motion_m);
    config_.exit_direction_cosine_min =
        std::clamp(config_.exit_direction_cosine_min, -1.0, 1.0);
    config_.catch_up_max_step_m =
        std::max(1.0e-6, config_.catch_up_max_step_m);
    config_.catch_up_complete_error_m =
        std::max(0.0, config_.catch_up_complete_error_m);
    config_.timestamp_epsilon_sec =
        std::max(0.0, config_.timestamp_epsilon_sec);
}

void StationaryMotionPolicy::reset() {
    state_ = RuntimeMotionState::MOVING;
    has_last_stamp_ = false;
    last_stamp_sec_ = 0.0;
    has_last_raw_position_ = false;
    last_raw_position_.setZero();
    anchor_position_.setZero();
    raw_anchor_position_.setZero();
    accumulated_motion_.setZero();
    confirmed_path_length_m_ = 0.0;
    stationary_enter_count_ = 0;
    movement_confirm_count_ = 0;
    catch_up_complete_count_ = 0;
}

bool StationaryMotionPolicy::isFreshTimestamp(double stamp_sec) const {
    return std::isfinite(stamp_sec) &&
        (!has_last_stamp_ ||
         stamp_sec > last_stamp_sec_ + config_.timestamp_epsilon_sec);
}

bool StationaryMotionPolicy::isReliableMeasurement(
    const StationaryMotionInput& input) const {
    return input.ndt_converged && input.ndt_accepted &&
        !input.prediction_only && input.registration_quality_valid &&
        !input.severe_degeneracy && input.raw_position.allFinite() &&
        input.filtered_position.allFinite() &&
        input.filtered_velocity.allFinite() &&
        std::isfinite(input.raw_increment_m) &&
        std::isfinite(input.allowed_physical_step_m) &&
        input.allowed_physical_step_m > 0.0 &&
        input.raw_increment_m <=
            input.allowed_physical_step_m + 1.0e-9;
}

void StationaryMotionPolicy::enterStationary(
    const StationaryMotionInput& input) {
    state_ = RuntimeMotionState::STATIONARY_HOLD;
    anchor_position_ = input.filtered_position;
    raw_anchor_position_ = input.raw_position;
    accumulated_motion_.setZero();
    confirmed_path_length_m_ = 0.0;
    movement_confirm_count_ = 0;
    catch_up_complete_count_ = 0;
}

void StationaryMotionPolicy::beginMovementConfirmation(
    const Eigen::Vector2d& delta) {
    state_ = RuntimeMotionState::MOVING_CONFIRM;
    accumulated_motion_ = delta;
    confirmed_path_length_m_ = delta.norm();
    movement_confirm_count_ = 1;
    catch_up_complete_count_ = 0;
}

void StationaryMotionPolicy::rejectMovementEvidence(
    const StationaryMotionInput& input,
    const std::string& reason) {
    (void)input;
    (void)reason;
    state_ = RuntimeMotionState::STATIONARY_HOLD;
    accumulated_motion_.setZero();
    confirmed_path_length_m_ = 0.0;
    movement_confirm_count_ = 0;
    catch_up_complete_count_ = 0;
}

StationaryMotionDecision StationaryMotionPolicy::baseDecision(
    const StationaryMotionInput& input) const {
    StationaryMotionDecision decision;
    decision.state = state_;
    decision.constrained_position = input.filtered_position;

    const bool reliable = isReliableMeasurement(input);
    decision.allow_local_map_update =
        state_ == RuntimeMotionState::MOVING && reliable;
    decision.allow_persistent_map_commit =
        state_ == RuntimeMotionState::MOVING && reliable;

    if (state_ == RuntimeMotionState::STATIONARY_HOLD ||
        state_ == RuntimeMotionState::MOVING_CONFIRM) {
        decision.apply_stationary_hold = true;
        decision.apply_position_constraint = true;
        decision.constrained_position = anchor_position_;
        decision.allow_local_map_update = false;
        decision.allow_persistent_map_commit = false;
    } else if (state_ == RuntimeMotionState::CATCH_UP) {
        decision.apply_position_constraint = true;
        decision.allow_local_map_update = false;
        decision.allow_persistent_map_commit = false;
    }
    return decision;
}

StationaryMotionDecision StationaryMotionPolicy::update(
    const StationaryMotionInput& input) {
    if (!isFreshTimestamp(input.stamp_sec)) {
        StationaryMotionDecision decision = baseDecision(input);
        decision.reason = "DUPLICATE_OR_NONFINITE_STAMP";
        return decision;
    }

    const bool reliable = isReliableMeasurement(input);
    const bool had_previous_raw = has_last_raw_position_;
    const Eigen::Vector2d previous_raw = last_raw_position_;
    has_last_stamp_ = true;
    last_stamp_sec_ = input.stamp_sec;
    if (reliable) {
        has_last_raw_position_ = true;
        last_raw_position_ = input.raw_position;
    }

    const Eigen::Vector2d raw_delta =
        reliable && had_previous_raw
            ? Eigen::Vector2d(input.raw_position - previous_raw)
            : Eigen::Vector2d::Zero();
    const double raw_delta_norm = raw_delta.norm();

    if (state_ == RuntimeMotionState::MOVING) {
        const bool stationary_evidence = reliable &&
            input.raw_increment_m <=
                config_.enter_max_raw_increment_m + 1.0e-9 &&
            input.filtered_velocity.norm() <=
                config_.enter_max_speed_mps + 1.0e-9;
        stationary_enter_count_ = stationary_evidence
            ? stationary_enter_count_ + 1
            : 0;

        if (stationary_enter_count_ >= config_.enter_confirm_frames) {
            enterStationary(input);
            StationaryMotionDecision decision = baseDecision(input);
            decision.reason = "STATIONARY_CONFIRMED";
            return decision;
        }

        StationaryMotionDecision decision = baseDecision(input);
        decision.reason = stationary_evidence
            ? "STATIONARY_ENTRY_PENDING"
            : (reliable ? "MOVING" : "UNRELIABLE_MEASUREMENT");
        return decision;
    }

    if (state_ == RuntimeMotionState::CATCH_UP) {
        StationaryMotionDecision decision = baseDecision(input);
        if (!reliable) {
            decision.constrained_position = input.filtered_position;
            decision.reason = "CATCH_UP_WAIT_RELIABLE";
            catch_up_complete_count_ = 0;
            return decision;
        }

        const Eigen::Vector2d error =
            input.raw_position - input.filtered_position;
        const double error_norm = error.norm();
        const double step = std::min(error_norm, config_.catch_up_max_step_m);
        if (error_norm > 1.0e-12) {
            decision.constrained_position =
                input.filtered_position + error * (step / error_norm);
        }
        decision.catch_up_step_m = step;
        decision.reason = "CATCH_UP_BOUNDED";

        if (error_norm <= config_.catch_up_complete_error_m + 1.0e-9) {
            ++catch_up_complete_count_;
        } else {
            catch_up_complete_count_ = 0;
        }

        if (catch_up_complete_count_ >= config_.catch_up_confirm_frames) {
            state_ = RuntimeMotionState::MOVING;
            stationary_enter_count_ = 0;
            movement_confirm_count_ = 0;
            accumulated_motion_.setZero();
            confirmed_path_length_m_ = 0.0;
            decision = baseDecision(input);
            decision.state = state_;
            decision.reason = "CATCH_UP_COMPLETE";
        }
        return decision;
    }

    const double anchor_drift = input.raw_position.allFinite()
        ? (input.raw_position - raw_anchor_position_).norm()
        : 0.0;
    const bool physically_plausible = reliable && had_previous_raw &&
        raw_delta_norm >= config_.exit_min_increment_m - 1.0e-9 &&
        raw_delta_norm <= input.allowed_physical_step_m + 1.0e-9;

    if (!physically_plausible) {
        const bool drift_only =
            anchor_drift >= config_.exit_cumulative_motion_m;
        rejectMovementEvidence(
            input, drift_only ? "DRIFT_ONLY_REJECTED" : "HOLD");
        StationaryMotionDecision decision = baseDecision(input);
        decision.reason = drift_only
            ? "DRIFT_ONLY_REJECTED"
            : (reliable ? "STATIONARY_HOLD" : "UNRELIABLE_MOVE_EVIDENCE");
        return decision;
    }

    if (state_ == RuntimeMotionState::STATIONARY_HOLD) {
        beginMovementConfirmation(raw_delta);
        StationaryMotionDecision decision = baseDecision(input);
        decision.reason = "MOVEMENT_CONFIRM_PENDING";
        return decision;
    }

    const double accumulated_norm = accumulated_motion_.norm();
    const double direction_cosine = accumulated_norm > 1.0e-12
        ? raw_delta.dot(accumulated_motion_) /
            (raw_delta_norm * accumulated_norm)
        : 1.0;
    if (!std::isfinite(direction_cosine) ||
        direction_cosine < config_.exit_direction_cosine_min) {
        rejectMovementEvidence(input, "DRIFT_ONLY_REJECTED");
        StationaryMotionDecision decision = baseDecision(input);
        decision.reason = "DRIFT_ONLY_REJECTED";
        return decision;
    }

    accumulated_motion_ += raw_delta;
    confirmed_path_length_m_ += raw_delta_norm;
    ++movement_confirm_count_;

    if (movement_confirm_count_ >= config_.exit_confirm_frames &&
        accumulated_motion_.norm() + 1.0e-9 >=
            config_.exit_cumulative_motion_m) {
        state_ = RuntimeMotionState::CATCH_UP;
        catch_up_complete_count_ = 0;
        StationaryMotionDecision decision = baseDecision(input);
        decision.movement_confirmed = true;
        decision.start_catch_up = true;
        decision.constrained_position = anchor_position_;
        decision.reason = "MOVEMENT_CONFIRMED_START_CATCH_UP";
        return decision;
    }

    StationaryMotionDecision decision = baseDecision(input);
    decision.reason = "MOVEMENT_CONFIRM_PENDING";
    return decision;
}

}  // namespace ndt_slam
