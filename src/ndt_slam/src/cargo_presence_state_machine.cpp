#include "ndt_slam/cargo_presence_state_machine.hpp"

#include <cmath>

namespace ndt_slam {

namespace {

bool validConfig(const CargoPresenceConfig& config) {
  return std::isfinite(config.gravity_stale_hold_sec) &&
      config.gravity_stale_hold_sec >= 0.0;
}

}  // namespace

const char* cargoPresenceStateName(CargoPresenceState state) noexcept {
  switch (state) {
    case CargoPresenceState::EMPTY: return "EMPTY";
    case CargoPresenceState::LOADED_AUTHORITATIVE:
      return "LOADED_AUTHORITATIVE";
    case CargoPresenceState::LOADED_GRAVITY_STALE_HOLD:
      return "LOADED_GRAVITY_STALE_HOLD";
    case CargoPresenceState::UNKNOWN_HARD_FAULT:
      return "UNKNOWN_HARD_FAULT";
  }
  return "UNKNOWN_HARD_FAULT";
}

CargoPresenceStateMachine::CargoPresenceStateMachine(
    const CargoPresenceConfig& config) {
  setConfig(config);
}

void CargoPresenceStateMachine::setConfig(
    const CargoPresenceConfig& config) {
  config_ = validConfig(config) ? config : CargoPresenceConfig{};
  reset();
}

void CargoPresenceStateMachine::reset() {
  result_ = CargoPresenceResult{};
  last_stamp_sec_ = 0.0;
  loaded_since_sec_ = 0.0;
  state_since_sec_ = 0.0;
  cargo_latched_ = false;
}

void CargoPresenceStateMachine::transition(
    CargoPresenceState state, double stamp_sec) {
  if (result_.state != state || state_since_sec_ <= 0.0) {
    state_since_sec_ = stamp_sec;
  }
  result_.state = state;
}

CargoPresenceResult CargoPresenceStateMachine::update(
    const CargoPresenceInput& input) {
  if (!std::isfinite(input.stamp_sec) || input.stamp_sec <= 0.0) {
    result_.cargo_present = cargo_latched_;
    result_.clear_allowed = false;
    result_.fallback_envelope_required = cargo_latched_;
    result_.gravity_authoritative = false;
    result_.state = CargoPresenceState::UNKNOWN_HARD_FAULT;
    result_.reason = "invalid_timestamp";
    return result_;
  }
  if (last_stamp_sec_ > 0.0 && input.stamp_sec <= last_stamp_sec_) {
    ++result_.source_epoch;
    if (cargo_latched_) loaded_since_sec_ = input.stamp_sec;
    result_.cargo_present = cargo_latched_;
    result_.clear_allowed = false;
    result_.fallback_envelope_required = cargo_latched_;
    result_.gravity_authoritative = false;
    transition(CargoPresenceState::UNKNOWN_HARD_FAULT, input.stamp_sec);
    result_.loaded_duration_sec = 0.0;
    result_.state_duration_sec = 0.0;
    result_.reason = "timestamp_rollback_conservative_hold";
    last_stamp_sec_ = input.stamp_sec;
    return result_;
  }
  last_stamp_sec_ = input.stamp_sec;

  if (!input.gravity_enabled) {
    cargo_latched_ = input.formal_track_retained ||
        input.lidar_candidate_visible;
    if (cargo_latched_ && loaded_since_sec_ <= 0.0) {
      loaded_since_sec_ = input.stamp_sec;
    } else if (!cargo_latched_) {
      loaded_since_sec_ = 0.0;
    }
    transition(cargo_latched_
                   ? CargoPresenceState::LOADED_AUTHORITATIVE
                   : CargoPresenceState::EMPTY,
               input.stamp_sec);
    result_.cargo_present = cargo_latched_;
    result_.clear_allowed = !cargo_latched_;
    result_.fallback_envelope_required =
        cargo_latched_ && !input.formal_track_retained;
    result_.gravity_authoritative = false;
    result_.reason = cargo_latched_
        ? "gravity_disabled_lidar_presence"
        : "gravity_disabled_no_cargo";
  } else if (input.gravity_valid &&
             input.gravity_state == HookLoadState::LOADED) {
    if (!cargo_latched_ || loaded_since_sec_ <= 0.0) {
      loaded_since_sec_ = input.stamp_sec;
    }
    cargo_latched_ = true;
    transition(CargoPresenceState::LOADED_AUTHORITATIVE, input.stamp_sec);
    result_.cargo_present = true;
    result_.clear_allowed = false;
    result_.fallback_envelope_required = !input.formal_track_retained;
    result_.gravity_authoritative = true;
    result_.reason = "gravity_loaded_authoritative";
  } else if (input.gravity_valid &&
             input.gravity_state == HookLoadState::EMPTY) {
    cargo_latched_ = false;
    loaded_since_sec_ = 0.0;
    transition(CargoPresenceState::EMPTY, input.stamp_sec);
    result_.cargo_present = false;
    result_.clear_allowed = true;
    result_.fallback_envelope_required = false;
    result_.gravity_authoritative = true;
    result_.reason = "gravity_empty_confirmed";
  } else {
    const bool short_stale = cargo_latched_ &&
        std::isfinite(input.gravity_age_sec) &&
        input.gravity_age_sec <= config_.gravity_stale_hold_sec;
    transition(short_stale
                   ? CargoPresenceState::LOADED_GRAVITY_STALE_HOLD
                   : CargoPresenceState::UNKNOWN_HARD_FAULT,
               input.stamp_sec);
    result_.cargo_present = cargo_latched_;
    result_.clear_allowed = false;
    result_.fallback_envelope_required = cargo_latched_;
    result_.gravity_authoritative = false;
    result_.reason = short_stale
        ? "gravity_stale_loaded_hold"
        : "gravity_unavailable_hard_fault";
  }

  result_.loaded_duration_sec = cargo_latched_ && loaded_since_sec_ > 0.0
      ? input.stamp_sec - loaded_since_sec_ : 0.0;
  result_.state_duration_sec = state_since_sec_ > 0.0
      ? input.stamp_sec - state_since_sec_ : 0.0;
  return result_;
}

}  // namespace ndt_slam
