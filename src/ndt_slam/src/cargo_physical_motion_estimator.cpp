#include "ndt_slam/cargo_physical_motion_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace ndt_slam {
namespace {

bool validConfig(const CargoPhysicalMotionConfig& config) {
  return std::isfinite(config.enter_stationary_speed_mps) &&
      config.enter_stationary_speed_mps >= 0.0F &&
      std::isfinite(config.exit_stationary_speed_mps) &&
      config.exit_stationary_speed_mps >
          config.enter_stationary_speed_mps &&
      std::isfinite(config.enter_stationary_confirm_sec) &&
      config.enter_stationary_confirm_sec >= 0.0 &&
      std::isfinite(config.exit_stationary_confirm_sec) &&
      config.exit_stationary_confirm_sec >= 0.0 &&
      std::isfinite(config.maximum_sample_gap_sec) &&
      config.maximum_sample_gap_sec > 0.0 &&
      std::isfinite(config.velocity_filter_alpha) &&
      config.velocity_filter_alpha > 0.0F &&
      config.velocity_filter_alpha <= 1.0F;
}

}  // namespace

const char* cargoPhysicalMotionStateName(
    CargoPhysicalMotionState state) noexcept {
  switch (state) {
    case CargoPhysicalMotionState::UNKNOWN: return "UNKNOWN";
    case CargoPhysicalMotionState::MOVING: return "MOVING";
    case CargoPhysicalMotionState::STOPPING_SETTLE:
      return "STOPPING_SETTLE";
    case CargoPhysicalMotionState::STATIONARY: return "STATIONARY";
  }
  return "UNKNOWN";
}

const char* cargoPhysicalMotionSourceName(
    CargoPhysicalMotionSource source) noexcept {
  switch (source) {
    case CargoPhysicalMotionSource::NONE: return "NONE";
    case CargoPhysicalMotionSource::EXTERNAL_CONTROLLER:
      return "EXTERNAL_CONTROLLER";
    case CargoPhysicalMotionSource::RAW_POSE_DELTA:
      return "RAW_POSE_DELTA";
  }
  return "NONE";
}

CargoPhysicalMotionEstimator::CargoPhysicalMotionEstimator(
    const CargoPhysicalMotionConfig& config) {
  setConfig(config);
}

void CargoPhysicalMotionEstimator::setConfig(
    const CargoPhysicalMotionConfig& config) {
  config_ = validConfig(config) ? config : CargoPhysicalMotionConfig{};
  reset();
}

void CargoPhysicalMotionEstimator::resetKinematics(bool preserve_epoch) {
  const std::uint64_t epoch = result_.source_epoch;
  result_ = CargoPhysicalMotionResult{};
  if (preserve_epoch) result_.source_epoch = epoch;
  last_stamp_sec_ = 0.0;
  last_valid_sample_stamp_sec_ = 0.0;
  state_since_sec_ = 0.0;
  transition_candidate_since_sec_ = 0.0;
  previous_position_valid_ = false;
  previous_position_.setZero();
  filtered_speed_valid_ = false;
}

void CargoPhysicalMotionEstimator::reset() {
  resetKinematics(false);
}

void CargoPhysicalMotionEstimator::setState(
    CargoPhysicalMotionState state, double stamp_sec) {
  if (result_.state != state || state_since_sec_ <= 0.0) {
    state_since_sec_ = stamp_sec;
  }
  result_.state = state;
}

CargoPhysicalMotionResult CargoPhysicalMotionEstimator::update(
    const CargoPhysicalMotionInput& input) {
  if (!config_.enabled) {
    reset();
    result_.reason = "disabled";
    return result_;
  }
  if (!std::isfinite(input.stamp_sec) || input.stamp_sec <= 0.0) {
    result_.valid = false;
    result_.reason = "invalid_timestamp";
    return result_;
  }
  if (last_stamp_sec_ > 0.0 && input.stamp_sec <= last_stamp_sec_) {
    const std::uint64_t next_epoch = result_.source_epoch + 1U;
    resetKinematics(false);
    result_.source_epoch = next_epoch;
    last_stamp_sec_ = input.stamp_sec;
    result_.reason = "timestamp_rollback_reset";
    return result_;
  }
  const bool long_gap = last_stamp_sec_ > 0.0 &&
      input.stamp_sec - last_stamp_sec_ > config_.maximum_sample_gap_sec;
  last_stamp_sec_ = input.stamp_sec;
  if (long_gap) {
    const std::uint64_t epoch = result_.source_epoch;
    resetKinematics(false);
    result_.source_epoch = epoch;
    last_stamp_sec_ = input.stamp_sec;
    result_.reason = "motion_sample_gap_reset";
  }

  if (input.external_state_valid &&
      input.external_state != CargoPhysicalMotionState::UNKNOWN) {
    setState(input.external_state, input.stamp_sec);
    result_.valid = true;
    result_.source = CargoPhysicalMotionSource::EXTERNAL_CONTROLLER;
    result_.confidence = 1.0F;
    result_.state_duration_sec = input.stamp_sec - state_since_sec_;
    result_.reason = "external_motion_state";
    last_valid_sample_stamp_sec_ = input.stamp_sec;
    transition_candidate_since_sec_ = 0.0;
    return result_;
  }

  if (!input.raw_position_valid || !input.raw_position.allFinite()) {
    const bool short_unknown = last_valid_sample_stamp_sec_ > 0.0 &&
        input.stamp_sec - last_valid_sample_stamp_sec_ <=
            config_.maximum_sample_gap_sec;
    result_.valid = short_unknown &&
        result_.state != CargoPhysicalMotionState::UNKNOWN;
    result_.reason = result_.valid
        ? "short_unknown_motion_input_hold"
        : "motion_input_unavailable";
    return result_;
  }

  if (!previous_position_valid_) {
    previous_position_ = input.raw_position;
    previous_position_valid_ = true;
    last_valid_sample_stamp_sec_ = input.stamp_sec;
    result_.valid = false;
    result_.source = CargoPhysicalMotionSource::RAW_POSE_DELTA;
    result_.reason = "raw_pose_baseline_initialized";
    return result_;
  }

  const double dt = input.stamp_sec - last_valid_sample_stamp_sec_;
  if (!std::isfinite(dt) || dt <= 1.0e-6) {
    result_.valid = false;
    result_.reason = "raw_pose_delta_time_invalid";
    return result_;
  }
  const float measured_speed = static_cast<float>(
      (input.raw_position - previous_position_).norm() / dt);
  previous_position_ = input.raw_position;
  last_valid_sample_stamp_sec_ = input.stamp_sec;
  if (!filtered_speed_valid_) {
    result_.filtered_speed_mps = measured_speed;
    filtered_speed_valid_ = true;
  } else {
    result_.filtered_speed_mps =
        (1.0F - config_.velocity_filter_alpha) *
            result_.filtered_speed_mps +
        config_.velocity_filter_alpha * measured_speed;
  }
  result_.source = CargoPhysicalMotionSource::RAW_POSE_DELTA;
  result_.confidence = std::clamp(
      static_cast<float>(dt / config_.maximum_sample_gap_sec),
      0.2F, 1.0F);

  const float speed = result_.filtered_speed_mps;
  if (result_.state == CargoPhysicalMotionState::UNKNOWN) {
    if (speed >= config_.exit_stationary_speed_mps) {
      setState(CargoPhysicalMotionState::MOVING, input.stamp_sec);
    } else {
      setState(CargoPhysicalMotionState::STOPPING_SETTLE,
               input.stamp_sec);
      transition_candidate_since_sec_ = input.stamp_sec;
    }
  } else if (result_.state == CargoPhysicalMotionState::MOVING) {
    if (speed <= config_.enter_stationary_speed_mps) {
      setState(CargoPhysicalMotionState::STOPPING_SETTLE,
               input.stamp_sec);
      transition_candidate_since_sec_ = input.stamp_sec;
    }
  } else if (result_.state == CargoPhysicalMotionState::STOPPING_SETTLE) {
    if (speed >= config_.exit_stationary_speed_mps) {
      setState(CargoPhysicalMotionState::MOVING, input.stamp_sec);
      transition_candidate_since_sec_ = 0.0;
    } else if (speed <= config_.enter_stationary_speed_mps) {
      if (transition_candidate_since_sec_ <= 0.0) {
        transition_candidate_since_sec_ = input.stamp_sec;
      }
      if (input.stamp_sec - transition_candidate_since_sec_ >=
          config_.enter_stationary_confirm_sec) {
        setState(CargoPhysicalMotionState::STATIONARY, input.stamp_sec);
        transition_candidate_since_sec_ = 0.0;
      }
    } else {
      transition_candidate_since_sec_ = input.stamp_sec;
    }
  } else if (result_.state == CargoPhysicalMotionState::STATIONARY) {
    if (speed >= config_.exit_stationary_speed_mps) {
      if (transition_candidate_since_sec_ <= 0.0) {
        transition_candidate_since_sec_ = input.stamp_sec;
      }
      if (input.stamp_sec - transition_candidate_since_sec_ >=
          config_.exit_stationary_confirm_sec) {
        setState(CargoPhysicalMotionState::MOVING, input.stamp_sec);
        transition_candidate_since_sec_ = 0.0;
      }
    } else {
      transition_candidate_since_sec_ = 0.0;
    }
  }

  result_.valid = result_.state != CargoPhysicalMotionState::UNKNOWN;
  result_.state_duration_sec = state_since_sec_ > 0.0
      ? input.stamp_sec - state_since_sec_ : 0.0;
  result_.reason = "raw_pose_motion_estimate";
  return result_;
}

}  // namespace ndt_slam
