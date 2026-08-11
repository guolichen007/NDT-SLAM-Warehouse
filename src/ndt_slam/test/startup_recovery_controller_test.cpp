#include "ndt_slam/startup_recovery_controller.hpp"

#include <gtest/gtest.h>

#include <set>

namespace ndt_slam {
namespace {

TEST(StartupRecoveryControllerTest, RestartIsAutomaticAndReadOnlyUntilRearmed) {
  StartupRecoveryController controller;
  auto decision = controller.boot();
  EXPECT_TRUE(decision.reset_runtime_evidence);
  EXPECT_FALSE(decision.map_write_allowed);
  const auto process_epoch = decision.process_epoch;

  EXPECT_EQ(controller.update({}).state, StartupRecoveryState::LOAD_REFERENCE);
  StartupRecoveryInput input;
  input.reference_load_finished = true;
  input.reference_present = true;
  input.reference_valid = true;
  EXPECT_EQ(controller.update(input).state,
            StartupRecoveryState::SENSOR_WARMUP);
  input = {};
  input.sensors_warm = true;
  input.checkpoint_available = true;
  EXPECT_EQ(controller.update(input).state,
            StartupRecoveryState::LOCAL_RECOVERY);
  input = {};
  input.local_recovery_finished = true;
  input.local_recovery_succeeded = true;
  EXPECT_EQ(controller.update(input).state, StartupRecoveryState::VERIFYING);
  input = {};
  input.verification_finished = true;
  input.verification_succeeded = true;
  decision = controller.update(input);
  EXPECT_EQ(decision.state, StartupRecoveryState::READONLY_STABILIZING);
  EXPECT_TRUE(decision.startup_recovery_verified);
  EXPECT_FALSE(decision.map_write_allowed);
  input = {};
  input.map_write_rearmed = true;
  decision = controller.update(input);
  EXPECT_EQ(decision.state, StartupRecoveryState::ACTIVE);
  EXPECT_TRUE(decision.map_write_allowed);
  EXPECT_EQ(decision.process_epoch, process_epoch);
}

TEST(StartupRecoveryControllerTest, CorruptionCreatesIsolatedFreshEpoch) {
  StartupRecoveryController controller;
  controller.update({});
  StartupRecoveryInput input;
  input.reference_load_finished = true;
  input.reference_present = true;
  input.reference_valid = false;
  auto decision = controller.update(input);
  EXPECT_EQ(decision.state, StartupRecoveryState::REFERENCE_INVALID);
  EXPECT_TRUE(decision.create_isolated_map);
  EXPECT_FALSE(decision.map_write_allowed);
  input = {};
  input.isolated_map_created = true;
  decision = controller.update(input);
  EXPECT_EQ(decision.state, StartupRecoveryState::ACTIVE);
  EXPECT_TRUE(decision.map_write_allowed);
  EXPECT_GT(decision.continuity_generation, 1U);
}

}  // namespace
}  // namespace ndt_slam
