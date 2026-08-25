#include "ndt_slam/rail_localization_authority.hpp"
#include "ndt_slam/registration_target_snapshot.hpp"
#include "ndt_slam/fixed_yaw_translation_solver.hpp"

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

TEST(RegistrationTargetSnapshotTest,
     SnapshotOwnsExactImmutableCloudAndCropIdentity) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  cloud->push_back(pcl::PointXYZ(1.0F, 2.0F, 0.0F));
  const auto first = makeRegistrationTargetSnapshot(
      cloud, RegistrationTargetSource::CROPPED_ACTIVE_MAP,
      7U, 3U, "map-frame-1", "crop-a");
  const auto second = makeRegistrationTargetSnapshot(
      cloud, RegistrationTargetSource::CROPPED_ACTIVE_MAP,
      7U, 3U, "map-frame-1", "crop-b");
  ASSERT_TRUE(first.valid());
  ASSERT_TRUE(second.valid());
  EXPECT_EQ(first.cloud.get(), cloud.get());
  EXPECT_NE(first.target_snapshot_id, second.target_snapshot_id);
}

TEST(RailYawAuthorityTest, ModeCannotHotSwitchWithinSession) {
  YawAuthorityModeLatch latch;
  EXPECT_TRUE(latch.initialize(YawAuthorityMode::LEGACY, 1U));
  EXPECT_FALSE(latch.initialize(YawAuthorityMode::RAIL_AUTHORITY, 1U));
  EXPECT_TRUE(latch.initialize(YawAuthorityMode::RAIL_AUTHORITY, 2U));
}

TEST(FixedYawTranslationSolverTest, SolvesTranslationWithoutChangingYaw) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr source(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr target(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (int index = -20; index <= 20; ++index) {
    const float coordinate = 0.1F * static_cast<float>(index);
    source->push_back(pcl::PointXYZ(coordinate, -1.0F, 0.0F));
    source->push_back(pcl::PointXYZ(coordinate, 1.0F, 0.0F));
    source->push_back(pcl::PointXYZ(-2.0F, 0.05F * index, 0.0F));
    source->push_back(pcl::PointXYZ(2.0F, 0.05F * index, 0.0F));
  }
  for (const auto& point : source->points) {
    target->push_back(pcl::PointXYZ(
        point.x + 2.0F, point.y + 3.0F, point.z));
  }
  const auto snapshot = makeRegistrationTargetSnapshot(
      target, RegistrationTargetSource::GLOBAL_MAP,
      1U, 1U, "map-frame-1", "full");
  FixedYawTranslationSolverConfig config;
  config.minimum_inliers = 20U;
  config.maximum_correspondence_distance_m = 1.0;
  FixedYawTranslationSolver solver(config);
  FixedYawTranslationInput input;
  input.source_cloud_base = source;
  input.target = snapshot;
  input.authoritative_yaw_rad = 0.0;
  input.seed_pose_map_base = Sophus::SE3d(
      Sophus::SO3d(), Eigen::Vector3d(1.7, 2.7, 0.0));
  const auto result = solver.solve(input);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_NEAR(result.xy.x(), 2.0, 0.05);
  EXPECT_NEAR(result.xy.y(), 3.0, 0.05);
  EXPECT_NEAR(result.pose_map_base.so3().log().z(), 0.0, 1.0e-12);
}

TEST(FixedYawTranslationSolverTest,
     DualSeedBasinDisagreementCannotAutoSelectLowerResidual) {
  FixedYawTranslationResult predicted;
  predicted.valid = true;
  predicted.xy = Eigen::Vector2d(0.0, 0.0);
  predicted.fitness = 0.2;
  FixedYawTranslationResult free;
  free.valid = true;
  free.xy = Eigen::Vector2d(4.0, 0.0);
  free.fitness = 0.01;
  const auto decision = selectFixedYawDualSeed(predicted, free, 0.5);
  EXPECT_FALSE(decision.authoritative_measurement_valid);
  EXPECT_EQ(decision.outcome,
            FixedYawDualSeedOutcome::SEED_BASIN_AMBIGUOUS);
}

}  // namespace
}  // namespace ndt_slam
