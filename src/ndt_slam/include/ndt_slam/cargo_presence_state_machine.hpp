#pragma once

#include "ndt_slam/hook_load_state_filter.hpp"

#include <cstdint>
#include <string>

namespace ndt_slam {

enum class CargoPresenceState : std::uint8_t {
  EMPTY = 0,
  LOADED_AUTHORITATIVE,
  LOADED_GRAVITY_STALE_HOLD,
  UNKNOWN_HARD_FAULT,
};

struct CargoPresenceConfig {
  double gravity_stale_hold_sec = 3.0;
};

struct CargoPresenceInput {
  double stamp_sec = 0.0;
  bool gravity_enabled = true;
  bool gravity_valid = false;
  HookLoadState gravity_state = HookLoadState::UNKNOWN;
  double gravity_age_sec = 0.0;
  bool lidar_candidate_visible = false;
  bool formal_track_retained = false;
};

struct CargoPresenceResult {
  bool cargo_present = false;
  // Permission to clear the presence latch and lifecycle. This is true only
  // for an authoritative EMPTY (or a gravity-disabled no-cargo decision); it
  // is unrelated to obstacle-clear status code 14.
  bool clear_allowed = false;
  bool fallback_envelope_required = false;
  bool gravity_authoritative = false;
  CargoPresenceState state = CargoPresenceState::EMPTY;
  double loaded_duration_sec = 0.0;
  double state_duration_sec = 0.0;
  std::uint64_t source_epoch = 0U;
  std::string reason = "not_evaluated";
};

class CargoPresenceStateMachine {
 public:
  explicit CargoPresenceStateMachine(
      const CargoPresenceConfig& config = CargoPresenceConfig{});

  void setConfig(const CargoPresenceConfig& config);
  void reset();
  CargoPresenceResult update(const CargoPresenceInput& input);
  const CargoPresenceResult& result() const noexcept { return result_; }

 private:
  void transition(CargoPresenceState state, double stamp_sec);

  CargoPresenceConfig config_;
  CargoPresenceResult result_;
  double last_stamp_sec_ = 0.0;
  double loaded_since_sec_ = 0.0;
  double state_since_sec_ = 0.0;
  bool cargo_latched_ = false;
};

const char* cargoPresenceStateName(CargoPresenceState state) noexcept;

}  // namespace ndt_slam
