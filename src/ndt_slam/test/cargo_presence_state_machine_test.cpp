#include "ndt_slam/cargo_presence_state_machine.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

CargoPresenceInput loaded(double stamp) {
  CargoPresenceInput input;
  input.stamp_sec = stamp;
  input.gravity_enabled = true;
  input.gravity_valid = true;
  input.gravity_state = HookLoadState::LOADED;
  input.gravity_age_sec = 0.0;
  return input;
}

TEST(CargoPresenceStateMachineTest, GravityLoadedAlwaysMeansCargoPresent) {
  CargoPresenceStateMachine machine;
  const auto result = machine.update(loaded(1.0));
  EXPECT_TRUE(result.cargo_present);
  EXPECT_TRUE(result.gravity_authoritative);
  EXPECT_FALSE(result.clear_allowed);
  EXPECT_TRUE(result.fallback_envelope_required);
  EXPECT_EQ(result.state, CargoPresenceState::LOADED_AUTHORITATIVE);
}

TEST(CargoPresenceStateMachineTest,
     LidarCannotCreateCargoWhileRequiredGravityIsUnknown) {
  CargoPresenceStateMachine machine;
  CargoPresenceInput input;
  input.stamp_sec = 1.0;
  input.gravity_enabled = true;
  input.gravity_valid = false;
  input.gravity_state = HookLoadState::UNKNOWN;
  input.gravity_age_sec = 1.0;
  input.lidar_candidate_visible = true;
  input.formal_track_retained = true;
  const auto result = machine.update(input);
  EXPECT_FALSE(result.cargo_present);
  EXPECT_FALSE(result.clear_allowed);
  EXPECT_EQ(result.state, CargoPresenceState::UNKNOWN_HARD_FAULT);
}

TEST(CargoPresenceStateMachineTest, GravityStaleDoesNotClearCargoPresence) {
  CargoPresenceConfig config;
  config.gravity_stale_hold_sec = 2.0;
  CargoPresenceStateMachine machine(config);
  ASSERT_TRUE(machine.update(loaded(1.0)).cargo_present);
  auto stale = loaded(1.5);
  stale.gravity_valid = false;
  stale.gravity_state = HookLoadState::UNKNOWN;
  stale.gravity_age_sec = 0.5;
  const auto held = machine.update(stale);
  EXPECT_TRUE(held.cargo_present);
  EXPECT_FALSE(held.clear_allowed);
  EXPECT_EQ(held.state,
            CargoPresenceState::LOADED_GRAVITY_STALE_HOLD);

  stale.stamp_sec = 4.0;
  stale.gravity_age_sec = 3.0;
  const auto fault = machine.update(stale);
  EXPECT_TRUE(fault.cargo_present);
  EXPECT_FALSE(fault.clear_allowed);
  EXPECT_EQ(fault.state, CargoPresenceState::UNKNOWN_HARD_FAULT);
}

TEST(CargoPresenceStateMachineTest, ConfirmedEmptyClearsLatchedPresence) {
  CargoPresenceStateMachine machine;
  ASSERT_TRUE(machine.update(loaded(1.0)).cargo_present);
  auto inhibited = loaded(1.1);
  inhibited.gravity_valid = false;
  inhibited.gravity_state = HookLoadState::INHIBIT;
  inhibited.gravity_age_sec = 0.1;
  EXPECT_TRUE(machine.update(inhibited).cargo_present);
  auto empty = loaded(1.2);
  empty.gravity_state = HookLoadState::EMPTY;
  const auto result = machine.update(empty);
  EXPECT_FALSE(result.cargo_present);
  EXPECT_TRUE(result.clear_allowed);
  EXPECT_EQ(result.state, CargoPresenceState::EMPTY);
}

TEST(CargoPresenceStateMachineTest, RollbackNeverPublishesFalseEmpty) {
  CargoPresenceStateMachine machine;
  ASSERT_TRUE(machine.update(loaded(10.0)).cargo_present);
  const auto result = machine.update(loaded(1.0));
  EXPECT_TRUE(result.cargo_present);
  EXPECT_FALSE(result.clear_allowed);
  EXPECT_EQ(result.state, CargoPresenceState::UNKNOWN_HARD_FAULT);
  EXPECT_EQ(result.source_epoch, 1U);
}

}  // namespace
}  // namespace ndt_slam
