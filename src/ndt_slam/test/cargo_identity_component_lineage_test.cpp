#include "ndt_slam/cargo_identity_component_lineage.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace ndt_slam {
namespace {

PoseAuthorityIdentity poseIdentity(std::uint64_t generation = 1U) {
  PoseAuthorityIdentity identity;
  identity.map_rebuild_generation = 2U;
  identity.keyframe_pose_version = 3U;
  identity.yaw_authority_generation = generation;
  identity.map_frame_uuid = "map-frame";
  identity.yaw_reference_hash = "yaw-reference";
  identity.target_snapshot_id = 4U;
  return identity;
}

SourceFrameIdentity sourceIdentity(std::uint64_t index, double stamp,
                                   std::uint64_t epoch = 1U) {
  SourceFrameIdentity identity;
  identity.processing_frame_index = index;
  identity.sensor_source_stamp_sec = stamp;
  identity.time_epoch_id = epoch;
  identity.source_cloud_size = 10U;
  identity.source_cloud_signature_hash = index + 100U;
  return identity;
}

CargoIdentityComponentDescriptor component(
    std::uint64_t id, std::uint64_t group, double stamp,
    double x, double y = 0.0, double sx = 1.0, double sy = 0.8) {
  CargoIdentityComponentDescriptor descriptor;
  descriptor.component_id = id;
  descriptor.exact_seed_frame_group_id = group;
  descriptor.source_stamp_sec = stamp;
  descriptor.center_base = Eigen::Vector2d(x, y);
  descriptor.robust_x05 = x - 0.5 * sx;
  descriptor.robust_x95 = x + 0.5 * sx;
  descriptor.robust_y05 = y - 0.5 * sy;
  descriptor.robust_y95 = y + 0.5 * sy;
  descriptor.robust_xy_extent = Eigen::Vector2d(sx, sy);
  return descriptor;
}

CargoIdentityComponentLineageFrame frame(
    std::uint64_t index, double stamp, double pose_x,
    std::vector<CargoIdentityComponentDescriptor> components,
    std::uint64_t lifecycle = 7U,
    std::uint64_t pose_generation = 1U,
    std::uint64_t time_epoch = 1U,
    bool gravity_valid = false,
    HookLoadState gravity_state = HookLoadState::UNKNOWN) {
  CargoIdentityComponentLineageFrame result;
  result.source_stamp_sec = stamp;
  result.lifecycle_id = lifecycle;
  result.source_frame_identity =
      sourceIdentity(index, stamp, time_epoch);
  result.pose_identity = poseIdentity(pose_generation);
  result.pose_map_base(0, 3) = pose_x;
  result.gravity_valid = gravity_valid;
  result.gravity_state = gravity_state;
  result.components = std::move(components);
  return result;
}

TEST(CargoIdentityComponentLineageTest,
     ReciprocalMatchSurvivesFrameLocalIdReorder) {
  CargoIdentityComponentLineage lineage;
  EXPECT_TRUE(lineage.update(frame(
      1U, 1.0, 0.0,
      {component(0U, 1U, 1.0, 0.0),
       component(1U, 2U, 1.0, 2.0)})).observations.empty());

  const auto result = lineage.update(frame(
      2U, 1.1, 1.0,
      {component(0U, 1U, 1.1, 2.03),
       component(1U, 2U, 1.1, 0.04)}));
  ASSERT_EQ(result.observations.size(), 2U);
  for (const auto& observation : result.observations) {
    if (observation.current_component_id == 0U) {
      EXPECT_EQ(observation.previous_component_id, 1U);
    } else if (observation.current_component_id == 1U) {
      EXPECT_EQ(observation.previous_component_id, 0U);
    } else {
      FAIL() << "unexpected current component id";
    }
  }
}

TEST(CargoIdentityComponentLineageTest,
     RejectsNonReciprocalAndAmbiguousCompetition) {
  CargoIdentityComponentLineage lineage;
  lineage.update(frame(
      1U, 1.0, 0.0,
      {component(10U, 1U, 1.0, 0.00),
       component(11U, 2U, 1.0, 0.08)}));
  const auto result = lineage.update(frame(
      2U, 1.1, 1.0,
      {component(20U, 1U, 1.1, 0.04)}));
  EXPECT_TRUE(result.observations.empty());
  EXPECT_GT(result.ambiguous_count, 0U);
}

TEST(CargoIdentityComponentLineageTest,
     MovingCraneStaticRackIsNotBaseAttachedFamily) {
  CargoIdentityComponentLineage lineage;
  lineage.update(frame(
      1U, 1.0, 0.0, {component(1U, 1U, 1.0, 2.0)}));
  // A world-static rack at map x=2 appears at base x=1 after +1m ego motion.
  const auto result = lineage.update(frame(
      2U, 1.1, 1.0, {component(2U, 1U, 1.1, 1.0)}));
  EXPECT_TRUE(result.observations.empty());
}

TEST(CargoIdentityComponentLineageTest,
     LowEgoUnknownLoadFailsClosedWithoutClaimingWorldStatic) {
  CargoIdentityComponentLineage lineage;
  lineage.update(frame(
      1U, 1.0, 0.0, {component(1U, 1U, 1.0, 0.0)}));
  const auto result = lineage.update(frame(
      2U, 1.1, 0.05, {component(2U, 1U, 1.1, 0.02)}));
  EXPECT_TRUE(result.observations.empty());
  EXPECT_EQ(result.world_static_veto_count, 0U);
  EXPECT_EQ(result.motion_observability_state,
            CargoIdentityMotionObservabilityState::UNKNOWN_FAIL_CLOSED);
}

TEST(CargoIdentityComponentLineageTest,
     LoadedLowEgoIsNotWorldStaticProven) {
  CargoIdentityComponentLineage lineage;
  lineage.update(frame(
      1U, 1.0, 0.0, {component(1U, 1U, 1.0, 0.0)},
      7U, 1U, 1U, true, HookLoadState::LOADED));
  const auto result = lineage.update(frame(
      2U, 1.1, 0.05, {component(2U, 1U, 1.1, 0.02)},
      7U, 1U, 1U, true, HookLoadState::LOADED));
  ASSERT_EQ(result.observations.size(), 1U);
  EXPECT_EQ(result.motion_observability_state,
            CargoIdentityMotionObservabilityState::
                LOAD_PRESENT_UNOBSERVABLE);
  EXPECT_TRUE(result.load_present_unobservable);
  EXPECT_EQ(result.world_static_veto_count, 0U);
  EXPECT_EQ(result.observations.front().motion_observability_state,
            CargoIdentityMotionObservabilityState::
                LOAD_PRESENT_UNOBSERVABLE);
}

TEST(CargoIdentityComponentLineageTest,
     ZeroLoadLowEgoCannotInventLineageCargo) {
  CargoIdentityComponentLineage lineage;
  lineage.update(frame(
      1U, 1.0, 0.0, {component(1U, 1U, 1.0, 0.0)},
      7U, 1U, 1U, true, HookLoadState::EMPTY));
  const auto result = lineage.update(frame(
      2U, 1.1, 0.05, {component(2U, 1U, 1.1, 0.02)},
      7U, 1U, 1U, true, HookLoadState::EMPTY));
  EXPECT_TRUE(result.observations.empty());
  EXPECT_EQ(result.motion_observability_state,
            CargoIdentityMotionObservabilityState::IDLE_ZERO_LOAD);
}

TEST(CargoIdentityComponentLineageTest,
     UnknownLoadLowEgoFailsClosed) {
  CargoIdentityComponentLineage lineage;
  lineage.update(frame(
      1U, 1.0, 0.0, {component(1U, 1U, 1.0, 0.0)}));
  const auto result = lineage.update(frame(
      2U, 1.1, 0.05, {component(2U, 1U, 1.1, 0.02)}));
  EXPECT_TRUE(result.observations.empty());
  EXPECT_EQ(result.motion_observability_state,
            CargoIdentityMotionObservabilityState::UNKNOWN_FAIL_CLOSED);
}

TEST(CargoIdentityComponentLineageTest,
     ObservableWorldStaticStillVetoed) {
  CargoIdentityComponentLineage lineage;
  lineage.update(frame(
      1U, 1.0, 0.0, {component(1U, 1U, 1.0, 0.0)}));
  const auto result = lineage.update(frame(
      2U, 1.1, 0.40, {component(2U, 1U, 1.1, -0.20)}));
  EXPECT_TRUE(result.observations.empty());
  EXPECT_EQ(result.motion_observability_state,
            CargoIdentityMotionObservabilityState::EGO_MOTION_OBSERVABLE);
  EXPECT_GT(result.world_static_veto_count, 0U);
}

TEST(CargoIdentityComponentLineageTest,
     LoadSignalCannotSelectComponent) {
  CargoIdentityComponentLineage lineage;
  lineage.update(frame(
      1U, 1.0, 0.0,
      {component(10U, 1U, 1.0, 0.00),
       component(11U, 2U, 1.0, 0.08)},
      7U, 1U, 1U, true, HookLoadState::LOADED));
  const auto result = lineage.update(frame(
      2U, 1.1, 0.05, {component(20U, 1U, 1.1, 0.04)},
      7U, 1U, 1U, true, HookLoadState::LOADED));
  EXPECT_TRUE(result.observations.empty());
  EXPECT_GT(result.ambiguous_count, 0U);
}

TEST(CargoIdentityComponentLineageTest,
     PersonLikeProximityAloneCannotBecomeCargoLineage) {
  CargoIdentityComponentLineage lineage;
  lineage.update(frame(
      1U, 1.0, 0.0, {component(1U, 1U, 1.0, 0.0, 0.0, 0.4, 0.4)}));
  const auto result = lineage.update(frame(
      2U, 1.1, 0.0, {component(2U, 1U, 1.1, 0.10, 0.0, 0.4, 0.4)}));
  EXPECT_TRUE(result.observations.empty());
}

TEST(CargoIdentityComponentLineageTest, ResetsAcrossGapRollbackAndAuthority) {
  CargoIdentityComponentLineage lineage;
  lineage.update(frame(
      1U, 1.0, 0.0, {component(1U, 1U, 1.0, 0.0)}));
  auto gap = lineage.update(frame(
      2U, 2.0, 1.0, {component(2U, 1U, 2.0, 0.0)}));
  EXPECT_TRUE(gap.observations.empty());
  EXPECT_EQ(gap.reset_reason, "source_gap");

  auto rollback = lineage.update(frame(
      3U, 1.5, 2.0, {component(3U, 1U, 1.5, 0.0)}));
  EXPECT_TRUE(rollback.observations.empty());
  EXPECT_EQ(rollback.reset_reason, "source_stamp_rollback");

  auto authority = lineage.update(frame(
      4U, 1.6, 3.0, {component(4U, 1U, 1.6, 0.0)},
      7U, 2U));
  EXPECT_TRUE(authority.observations.empty());
  EXPECT_EQ(authority.reset_reason, "pose_authority_changed");
}

TEST(CargoIdentityComponentLineageTest, ResetsAcrossLifecycleAndSourceEpoch) {
  CargoIdentityComponentLineage lifecycle_lineage;
  lifecycle_lineage.update(frame(
      1U, 1.0, 0.0, {component(1U, 1U, 1.0, 0.0)}, 7U));
  const auto lifecycle = lifecycle_lineage.update(frame(
      2U, 1.1, 1.0, {component(2U, 1U, 1.1, 0.0)}, 8U));
  EXPECT_TRUE(lifecycle.observations.empty());
  EXPECT_EQ(lifecycle.reset_reason, "lifecycle_changed");

  CargoIdentityComponentLineage epoch_lineage;
  epoch_lineage.update(frame(
      1U, 1.0, 0.0, {component(1U, 1U, 1.0, 0.0)},
      7U, 1U, 1U));
  const auto epoch = epoch_lineage.update(frame(
      2U, 1.1, 1.0, {component(2U, 1U, 1.1, 0.0)},
      7U, 1U, 2U));
  EXPECT_TRUE(epoch.observations.empty());
  EXPECT_EQ(epoch.reset_reason, "source_frame_epoch_changed");
}

TEST(CargoIdentityComponentLineageTest, DoesNotMutateCurrentDescriptors) {
  CargoIdentityComponentLineage lineage;
  const auto previous_component = component(11U, 1U, 1.0, 0.0);
  const auto current_component = component(22U, 2U, 1.1, 0.05);
  lineage.update(frame(1U, 1.0, 0.0, {previous_component}));
  lineage.update(frame(2U, 1.1, 1.0, {current_component}));
  EXPECT_EQ(current_component.component_id, 22U);
  EXPECT_EQ(current_component.exact_seed_frame_group_id, 2U);
  EXPECT_DOUBLE_EQ(current_component.center_base.x(), 0.05);
  EXPECT_DOUBLE_EQ(current_component.robust_xy_extent.x(), 1.0);
}

TEST(CargoIdentityComponentLineageTest,
     LineageCanBridgeOneMissingSourceFrameWithinExistingGap) {
  CargoIdentityComponentLineage lineage;
  lineage.update(frame(
      1U, 1.0, 0.0, {component(1U, 1U, 1.0, 0.0)}));

  const auto empty = lineage.update(frame(2U, 1.1, 1.0, {}));
  EXPECT_TRUE(empty.observations.empty());

  const auto after_empty = lineage.update(frame(
      3U, 1.2, 2.0, {component(3U, 1U, 1.2, 0.02)}));
  ASSERT_EQ(after_empty.observations.size(), 1U);
  EXPECT_EQ(after_empty.observations.front().previous_component_id, 1U);
  EXPECT_EQ(after_empty.observations.front().current_component_id, 3U);
  EXPECT_EQ(after_empty.observations.front().source_frame_offset, 2U);
  EXPECT_NEAR(after_empty.observations.front().source_age_sec, 0.2, 1.0e-9);
}

TEST(CargoIdentityComponentLineageTest,
     LineageOlderFallbackStillRequiresReciprocalUniqueness) {
  CargoIdentityComponentLineage lineage;
  lineage.update(frame(
      1U, 1.0, 0.0,
      {component(10U, 1U, 1.0, 0.00),
       component(11U, 2U, 1.0, 0.08)}));
  lineage.update(frame(2U, 1.1, 1.0, {}));
  const auto result = lineage.update(frame(
      3U, 1.2, 2.0, {component(20U, 1U, 1.2, 0.04)}));
  EXPECT_TRUE(result.observations.empty());
  EXPECT_GT(result.ambiguous_count, 0U);
}

TEST(CargoIdentityComponentLineageTest,
     OlderAmbiguityDoesNotInvalidateNewerUniqueMatch) {
  CargoIdentityComponentLineage lineage;
  lineage.update(frame(
      1U, 1.0, 0.0,
      {component(10U, 1U, 1.0, 0.00),
       component(11U, 2U, 1.0, 0.08)}));
  lineage.update(frame(
      2U, 1.1, 1.0, {component(12U, 3U, 1.1, 0.04)}));

  const auto result = lineage.update(frame(
      3U, 1.2, 2.0, {component(20U, 4U, 1.2, 0.05)}));
  ASSERT_EQ(result.observations.size(), 1U);
  EXPECT_EQ(result.observations.front().previous_component_id, 12U);
  EXPECT_EQ(result.observations.front().current_component_id, 20U);
  EXPECT_EQ(result.observations.front().source_frame_offset, 1U);
  EXPECT_NEAR(result.observations.front().source_age_sec, 0.1, 1.0e-9);
}

TEST(CargoIdentityComponentLineageTest,
     LineageOlderFallbackCannotCrossObservationGap) {
  CargoIdentityComponentLineage lineage;
  lineage.update(frame(
      1U, 1.0, 0.0, {component(1U, 1U, 1.0, 0.0)}));
  lineage.update(frame(2U, 1.4, 1.0, {}));
  const auto result = lineage.update(frame(
      3U, 1.6, 2.0, {component(3U, 1U, 1.6, 0.02)}));
  EXPECT_TRUE(result.observations.empty());
}

TEST(CargoIdentityComponentLineageTest,
     LineageOlderFallbackCannotCrossLifecycleOrPoseAuthority) {
  CargoIdentityComponentLineage lifecycle;
  lifecycle.update(frame(
      1U, 1.0, 0.0, {component(1U, 1U, 1.0, 0.0)}, 7U));
  lifecycle.update(frame(2U, 1.1, 1.0, {}, 8U));
  const auto after_lifecycle = lifecycle.update(frame(
      3U, 1.2, 2.0, {component(3U, 1U, 1.2, 0.02)}, 8U));
  EXPECT_TRUE(after_lifecycle.observations.empty());

  CargoIdentityComponentLineage authority;
  authority.update(frame(
      1U, 1.0, 0.0, {component(1U, 1U, 1.0, 0.0)}, 7U, 1U));
  authority.update(frame(2U, 1.1, 1.0, {}, 7U, 2U));
  const auto after_authority = authority.update(frame(
      3U, 1.2, 2.0, {component(3U, 1U, 1.2, 0.02)}, 7U, 2U));
  EXPECT_TRUE(after_authority.observations.empty());
}

TEST(CargoIdentityComponentLineageTest, CompactTypeOwnsNoPointCloud) {
  EXPECT_LT(sizeof(CargoIdentityComponentDescriptor), 256U);
  EXPECT_LT(sizeof(CargoIdentitySupportLineageObservation), 256U);
}

}  // namespace
}  // namespace ndt_slam
