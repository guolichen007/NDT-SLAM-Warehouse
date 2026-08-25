#include "ndt_slam/rail_localization_authority.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace ndt_slam {
namespace {

RailYawReference reference(double yaw = 0.25) {
  RailYawReference result;
  result.verified = true;
  result.rail_yaw_in_map_rad = yaw;
  result.source = YawReferenceSource::CONFIG_SITE_REFERENCE;
  result.map_frame_uuid = "map-frame-1";
  result.reference_uuid = "yaw-reference-1";
  result.reference_hash = semanticYawReferenceHash(result);
  return result;
}

TEST(RailYawAuthorityTest, RepeatedRawNdtYawBiasCannotMutateRailAuthority) {
  RailYawAuthority authority;
  ASSERT_TRUE(authority.initialize(
      reference(), YawAuthorityTransitionReason::LOAD_VERIFIED_SESSION));
  const auto generation = authority.generation();
  for (int frame = 0; frame < 10000; ++frame) {
    authority.observeProposalYaw(0.25 + frame * 0.00001, frame * 0.1);
  }
  EXPECT_NEAR(authority.yawRad(), 0.25, 1.0e-12);
  EXPECT_EQ(authority.generation(), generation);
}

TEST(RailYawAuthorityTest, TimeRollbackPreservesRailYawAuthority) {
  RailYawAuthority authority;
  ASSERT_TRUE(authority.initialize(
      reference(), YawAuthorityTransitionReason::LOAD_VERIFIED_SESSION));
  const auto generation = authority.generation();
  authority.observeProposalYaw(0.5, 10.0);
  authority.handleTimestampRollback(2.0);
  EXPECT_TRUE(authority.valid());
  EXPECT_NEAR(authority.yawRad(), 0.25, 1.0e-12);
  EXPECT_EQ(authority.generation(), generation);
}

TEST(RailYawAuthorityTest, NormalRelocalizationCannotMutateYawAuthority) {
  RailYawAuthority authority;
  ASSERT_TRUE(authority.initialize(
      reference(), YawAuthorityTransitionReason::LOAD_VERIFIED_SESSION));
  authority.observeRelocalizationProposalYaw(-1.2, 20.0);
  EXPECT_NEAR(authority.yawRad(), 0.25, 1.0e-12);
}

TEST(RailYawAuthorityTest, ExplicitMigrationIsAtomicYawWriter) {
  RailYawAuthority authority;
  ASSERT_TRUE(authority.initialize(
      reference(), YawAuthorityTransitionReason::LOAD_VERIFIED_SESSION));
  const auto generation = authority.generation();
  auto migrated = reference(-0.4);
  migrated.reference_uuid = "yaw-reference-2";
  migrated.reference_hash = semanticYawReferenceHash(migrated);
  ASSERT_TRUE(authority.explicitMapFrameMigration(migrated));
  EXPECT_NEAR(authority.yawRad(), -0.4, 1.0e-12);
  EXPECT_GT(authority.generation(), generation);
}

TEST(RailLocalizationHealthTest,
     RailFitnessWarmupCannotAuthorizeSafetyOrPersistentMap) {
  RailLocalizationHealthInput input;
  input.mode = YawAuthorityMode::RAIL_AUTHORITY;
  input.yaw_reference_valid = true;
  input.target_identity_valid = true;
  input.fixed_xy_valid = true;
  input.ekf_measurement_accepted = true;
  input.rail_fitness_allow_measurement = true;
  input.rail_fitness_baseline_ready = false;
  const auto decision = evaluateRailLocalizationHealth(input);
  EXPECT_TRUE(decision.odom_continuity_valid);
  EXPECT_FALSE(decision.safety_localization_authorized);
  EXPECT_FALSE(decision.map_mutation_authorized);
  EXPECT_EQ(decision.failure_class,
            LocalizationFailureClass::TEMPORARY_OBSERVABILITY_LOSS);
}

TEST(RailLocalizationHealthTest,
     RawProposalFailureCannotIncrementAuthoritativeBadFrames) {
  RailLocalizationHealthInput input;
  input.mode = YawAuthorityMode::RAIL_AUTHORITY;
  input.raw_ndt_proposal_healthy = false;
  input.yaw_reference_valid = true;
  input.target_identity_valid = true;
  input.fixed_xy_valid = true;
  input.ekf_measurement_accepted = true;
  input.rail_fitness_allow_measurement = true;
  input.rail_fitness_baseline_ready = true;
  const auto decision = evaluateRailLocalizationHealth(input);
  EXPECT_TRUE(decision.authoritative_frame_healthy);
  EXPECT_FALSE(decision.increment_relocalization_bad_frames);
  EXPECT_TRUE(decision.safety_localization_authorized);
}

TEST(RailLocalizationHealthTest,
     NonrecoverableReferenceFaultCannotEnterRestartLoop) {
  RailLocalizationHealthInput input;
  input.mode = YawAuthorityMode::RAIL_AUTHORITY;
  input.yaw_reference_valid = false;
  input.reference_failure_nonrecoverable = true;
  const auto decision = evaluateRailLocalizationHealth(input);
  EXPECT_EQ(decision.failure_class,
            LocalizationFailureClass::NONRECOVERABLE_REFERENCE_CONFIG);
  EXPECT_FALSE(decision.increment_relocalization_bad_frames);
  EXPECT_FALSE(decision.request_relocalization);
  EXPECT_FALSE(decision.watchdog_restart_authorized);
  EXPECT_FALSE(decision.safety_localization_authorized);
  EXPECT_FALSE(decision.map_mutation_authorized);
}

TEST(RegistrationTargetSnapshotTest,
     SameMapVersionDifferentCropHasDifferentSnapshotIdentity) {
  RegistrationTargetIdentityInput first;
  first.source = RegistrationTargetSource::CROPPED_ACTIVE_MAP;
  first.content_version = 9U;
  first.map_rebuild_generation = 4U;
  first.crop_identity = "x0=0;x1=10;y0=0;y1=5";
  RegistrationTargetIdentityInput second = first;
  second.crop_identity = "x0=10;x1=20;y0=0;y1=5";
  EXPECT_NE(makeRegistrationTargetSnapshotId(first),
            makeRegistrationTargetSnapshotId(second));
}

TEST(RailYawAuthorityTest, ModeCannotHotSwitchWithinSession) {
  YawAuthorityModeLatch latch;
  EXPECT_TRUE(latch.initialize(YawAuthorityMode::LEGACY, 1U));
  EXPECT_FALSE(latch.initialize(YawAuthorityMode::RAIL_AUTHORITY, 1U));
  EXPECT_TRUE(latch.initialize(YawAuthorityMode::RAIL_AUTHORITY, 2U));
}

}  // namespace
}  // namespace ndt_slam
