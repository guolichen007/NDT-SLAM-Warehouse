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
    config_.exit_evidence_window_sec =
        std::max(0.1, config_.exit_evidence_window_sec);
    config_.exit_min_speed_mps =
        std::max(0.0, config_.exit_min_speed_mps);
    config_.exit_force_anchor_drift_m =
        std::max(config_.exit_cumulative_motion_m,
                 config_.exit_force_anchor_drift_m);
    config_.moving_confirm_timeout_sec =
        std::max(config_.exit_evidence_window_sec,
                 config_.moving_confirm_timeout_sec);
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
    motion_samples_.clear();
    movement_confirm_start_stamp_sec_ = 0.0;
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
    motion_samples_.clear();
    motion_samples_.push_back({input.stamp_sec, input.raw_position});
    movement_confirm_start_stamp_sec_ = 0.0;
    movement_confirm_count_ = 0;
    catch_up_complete_count_ = 0;
}

void StationaryMotionPolicy::beginMovementConfirmation(double stamp_sec) {
    state_ = RuntimeMotionState::MOVING_CONFIRM;
    movement_confirm_start_stamp_sec_ = stamp_sec;
    catch_up_complete_count_ = 0;
}

void StationaryMotionPolicy::rejectMovementEvidence(
    const StationaryMotionInput& input,
    const std::string& reason) {
    (void)input;
    (void)reason;
    state_ = RuntimeMotionState::STATIONARY_HOLD;
    anchor_position_ = input.filtered_position;
    raw_anchor_position_ = input.raw_position;
    accumulated_motion_.setZero();
    confirmed_path_length_m_ = 0.0;
    motion_samples_.clear();
    if (input.raw_position.allFinite() && std::isfinite(input.stamp_sec)) {
        motion_samples_.push_back({input.stamp_sec, input.raw_position});
    }
    movement_confirm_start_stamp_sec_ = 0.0;
    movement_confirm_count_ = 0;
    catch_up_complete_count_ = 0;
}

void StationaryMotionPolicy::appendMotionSample(
    double stamp_sec, const Eigen::Vector2d& raw_position) {
    if (!std::isfinite(stamp_sec) || !raw_position.allFinite()) return;
    if (!motion_samples_.empty() &&
        stamp_sec <= motion_samples_.back().stamp_sec +
                         config_.timestamp_epsilon_sec) {
        return;
    }
    motion_samples_.push_back({stamp_sec, raw_position});
    pruneMotionSamples(stamp_sec);
}

void StationaryMotionPolicy::pruneMotionSamples(double stamp_sec) {
    const double minimum_stamp =
        stamp_sec - config_.exit_evidence_window_sec;
    while (motion_samples_.size() > 2U &&
           motion_samples_[1].stamp_sec < minimum_stamp) {
        motion_samples_.pop_front();
    }
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
        state_ == RuntimeMotionState::MOVING && reliable &&
        input.persistent_map_quality_valid;

    if (state_ == RuntimeMotionState::STATIONARY_HOLD) {
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

        const double post_step_residual_m = std::max(0.0, error_norm - step);
        if (post_step_residual_m <=
            config_.catch_up_complete_error_m + 1.0e-9) {
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
            // The transition frame is a release guard: runtime pose may leave
            // CATCH_UP, but neither map path can consume that same frame.
            decision.allow_local_map_update = false;
            decision.allow_persistent_map_commit = false;
            decision.reason = "CATCH_UP_COMPLETE_RELEASE_GUARD";
        }
        return decision;
    }

    const double anchor_drift = input.raw_position.allFinite()
        ? (input.raw_position - raw_anchor_position_).norm()
        : 0.0;
    const bool physically_plausible = reliable && had_previous_raw &&
        raw_delta_norm <= input.allowed_physical_step_m + 1.0e-9;
    if (physically_plausible) {
        appendMotionSample(input.stamp_sec, input.raw_position);
    } else {
        pruneMotionSamples(input.stamp_sec);
    }

    accumulated_motion_.setZero();
    confirmed_path_length_m_ = 0.0;
    double evidence_duration_sec = 0.0;
    if (motion_samples_.size() >= 2U) {
        accumulated_motion_ =
            motion_samples_.back().raw_position -
            motion_samples_.front().raw_position;
        evidence_duration_sec =
            motion_samples_.back().stamp_sec -
            motion_samples_.front().stamp_sec;
        for (std::size_t i = 1U; i < motion_samples_.size(); ++i) {
            confirmed_path_length_m_ +=
                (motion_samples_[i].raw_position -
                 motion_samples_[i - 1U].raw_position).norm();
        }
    }
    const double net_motion_m = accumulated_motion_.norm();
    const double direction_coherence = confirmed_path_length_m_ > 1.0e-12
        ? net_motion_m / confirmed_path_length_m_
        : 0.0;
    const double window_speed_mps = evidence_duration_sec > 1.0e-6
        ? net_motion_m / evidence_duration_sec
        : 0.0;
    movement_confirm_count_ = motion_samples_.empty()
        ? 0
        : static_cast<int>(motion_samples_.size()) - 1;

    const bool movement_window_started = physically_plausible &&
        confirmed_path_length_m_ + 1.0e-9 >=
            config_.exit_min_increment_m;
    if (state_ == RuntimeMotionState::STATIONARY_HOLD &&
        movement_window_started) {
        beginMovementConfirmation(input.stamp_sec);
    }

    const bool confirmed_by_window =
        movement_confirm_count_ >= config_.exit_confirm_frames &&
        net_motion_m + 1.0e-9 >= config_.exit_cumulative_motion_m &&
        direction_coherence + 1.0e-9 >=
            config_.exit_direction_cosine_min &&
        window_speed_mps + 1.0e-9 >= config_.exit_min_speed_mps;
    const bool forced_anchor_release =
        movement_confirm_count_ >= config_.exit_confirm_frames &&
        anchor_drift + 1.0e-9 >= config_.exit_force_anchor_drift_m;

    if (confirmed_by_window || forced_anchor_release) {
        state_ = RuntimeMotionState::CATCH_UP;
        catch_up_complete_count_ = 0;
        StationaryMotionDecision decision = baseDecision(input);
        decision.movement_confirmed = true;
        decision.start_catch_up = true;
        decision.constrained_position = input.filtered_position;
        decision.reason = confirmed_by_window
            ? "MOVEMENT_CONFIRMED_START_CATCH_UP"
            : "ANCHOR_DRIFT_FAILSAFE_START_CATCH_UP";
        return decision;
    }

    if (state_ == RuntimeMotionState::MOVING_CONFIRM) {
        const double confirm_age_sec =
            input.stamp_sec - movement_confirm_start_stamp_sec_;
        if (confirm_age_sec > config_.moving_confirm_timeout_sec &&
            anchor_drift < config_.exit_cumulative_motion_m &&
            direction_coherence < config_.exit_direction_cosine_min) {
            rejectMovementEvidence(input, "DRIFT_ONLY_REJECTED");
            StationaryMotionDecision decision = baseDecision(input);
            decision.reason = "DRIFT_ONLY_REJECTED";
            return decision;
        }
        StationaryMotionDecision decision = baseDecision(input);
        decision.reason = physically_plausible
            ? "MOVEMENT_CONFIRM_PENDING"
            : "MOVEMENT_CONFIRM_GAP";
        return decision;
    }

    StationaryMotionDecision decision = baseDecision(input);
    decision.reason = reliable
        ? "STATIONARY_HOLD"
        : "UNRELIABLE_MOVE_EVIDENCE";
    return decision;
}

}  // namespace ndt_slam
