#include <gtest/gtest.h>

#include "ndt_slam/avoidance_map_mutation.hpp"

namespace ndt_slam {
namespace {

PoseAuthorityIdentity poseIdentity() {
  PoseAuthorityIdentity identity;
  identity.map_rebuild_generation = 1U;
  identity.keyframe_pose_version = 2U;
  identity.yaw_authority_generation = 3U;
  identity.map_frame_uuid = "map-frame";
  identity.yaw_reference_hash = "yaw-reference";
  identity.target_snapshot_id = 4U;
  return identity;
}

TEST(AvoidanceMapMutation, KeyFrameRemovalRequiresExactOwnership) {
  pcl::PointCloud<pcl::PointXYZ> source;
  const pcl::PointXYZ owned{0.10F, 0.10F, 1.00F};
  const pcl::PointXYZ same_voxel_background{0.11F, 0.10F, 1.00F};
  source.push_back(owned);
  source.push_back(same_voxel_background);
  const SourceFrameIdentity frame =
      makeSourceFrameIdentity(10U, 1.0, 1U, source);

  CurrentFramePointOwnership ownership;
  ownership.valid = true;
  ownership.source_frame_identity = frame;
  ownership.voxel_size_m = 0.05F;
  SourcePointKey key;
  ASSERT_TRUE(makeSourcePointKey(owned, &key));
  ownership.exact_points.insert(key);
  PointOwnershipVoxel voxel;
  ASSERT_TRUE(makePointOwnershipVoxel(owned, 0.05F, &voxel));
  ownership.voxels.insert(voxel);

  EXPECT_TRUE(ownership.owns(frame, owned));
  EXPECT_FALSE(ownership.owns(frame, same_voxel_background));
}

TEST(AvoidanceMapMutation, StaleSourceFrameDropsWholeSnapshot) {
  pcl::PointCloud<pcl::PointXYZ> source;
  source.push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
  const SourceFrameIdentity first =
      makeSourceFrameIdentity(10U, 1.0, 1U, source);
  const SourceFrameIdentity later =
      makeSourceFrameIdentity(11U, 2.0, 1U, source);
  AvoidanceMapMutationSnapshot snapshot;
  snapshot.source_stamp_sec = 1.0;
  snapshot.source_cloud_instance_id = 10U;
  snapshot.source_frame_identity = first;
  snapshot.pose_identity = poseIdentity();
  snapshot.localization_map_write_authorized = true;
  snapshot.human_points.valid = true;
  snapshot.human_points.source_frame_identity = first;
  snapshot.reason = MapMutationReason::AUTHORIZED;

  EXPECT_TRUE(snapshot.validFor(
      poseIdentity(), first, 1.0, 10U));
  EXPECT_FALSE(snapshot.validFor(
      poseIdentity(), later, 2.0, 11U));
}

TEST(AvoidanceMapMutation, PoseGenerationMismatchDropsWholeSnapshot) {
  pcl::PointCloud<pcl::PointXYZ> source;
  source.push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
  const SourceFrameIdentity frame =
      makeSourceFrameIdentity(10U, 1.0, 1U, source);
  AvoidanceMapMutationSnapshot snapshot;
  snapshot.source_stamp_sec = 1.0;
  snapshot.source_cloud_instance_id = 10U;
  snapshot.source_frame_identity = frame;
  snapshot.pose_identity = poseIdentity();
  snapshot.localization_map_write_authorized = true;
  snapshot.human_points.valid = true;
  snapshot.human_points.source_frame_identity = frame;
  PoseAuthorityIdentity next = poseIdentity();
  ++next.yaw_authority_generation;

  EXPECT_FALSE(snapshot.validFor(next, frame, 1.0, 10U));
}

TEST(AvoidanceMapMutation, NestedHumanOwnerMustBelongToSourceFrame) {
  pcl::PointCloud<pcl::PointXYZ> source;
  source.push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
  const SourceFrameIdentity frame =
      makeSourceFrameIdentity(10U, 1.0, 1U, source);
  SourceFrameIdentity stale = frame;
  stale.processing_frame_index = 9U;

  AvoidanceMapMutationSnapshot snapshot;
  snapshot.source_stamp_sec = 1.0;
  snapshot.source_cloud_instance_id = 10U;
  snapshot.source_frame_identity = frame;
  snapshot.pose_identity = poseIdentity();
  snapshot.localization_map_write_authorized = true;
  snapshot.human_points.valid = true;
  snapshot.human_points.source_frame_identity = stale;

  EXPECT_FALSE(snapshot.validFor(poseIdentity(), frame, 1.0, 10U));
}

TEST(AvoidanceMapMutation, AuthorizedCargoOwnerMustBelongToSourceFrame) {
  pcl::PointCloud<pcl::PointXYZ> source;
  source.push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
  const SourceFrameIdentity frame =
      makeSourceFrameIdentity(10U, 1.0, 1U, source);
  SourceFrameIdentity stale = frame;
  stale.processing_frame_index = 9U;

  AvoidanceMapMutationSnapshot snapshot;
  snapshot.source_stamp_sec = 1.0;
  snapshot.source_cloud_instance_id = 10U;
  snapshot.source_frame_identity = frame;
  snapshot.pose_identity = poseIdentity();
  snapshot.localization_map_write_authorized = true;
  snapshot.human_points.valid = true;
  snapshot.human_points.source_frame_identity = frame;
  snapshot.cargo_points.authorized = true;
  snapshot.cargo_points.tight_geometry_valid = true;
  snapshot.cargo_points.owner_points.valid = true;
  snapshot.cargo_points.owner_points.source_frame_identity = stale;

  EXPECT_FALSE(snapshot.validFor(poseIdentity(), frame, 1.0, 10U));
}

TEST(AvoidanceMapMutation, UnknownCargoCannotOwnAnyPoint) {
  CargoMapMutationSnapshot cargo;
  cargo.authorized = false;
  cargo.tight_geometry_valid = true;
  const pcl::PointXYZ point{0.0F, 0.0F, 1.0F};
  pcl::PointCloud<pcl::PointXYZ> source;
  source.push_back(point);
  cargo.owner_points.valid = true;
  cargo.owner_points.source_frame_identity =
      makeSourceFrameIdentity(1U, 1.0, 1U, source);
  SourcePointKey key;
  ASSERT_TRUE(makeSourcePointKey(point, &key));
  cargo.owner_points.exact_points.insert(key);
  cargo.center_x = 0.0F;
  cargo.center_y = 0.0F;
  cargo.half_length = 1.0F;
  cargo.half_width = 1.0F;
  cargo.min_z = 0.0F;
  cargo.max_z = 2.0F;

  EXPECT_TRUE(cargo.ownsCurrentPoint(point));
  EXPECT_FALSE(cargo.owns(point));
}

TEST(AvoidanceMapMutation, BroadQuarantineCannotMutateV6LocalizationMap) {
  CargoMapMutationSnapshot cargo;
  cargo.authorized = false;
  CurrentFramePointOwnership no_candidates;
  EXPECT_TRUE(shouldDropV6MapCommitWithoutExactCargoOwnership(
      true, true, false, cargo, no_candidates));
  EXPECT_FALSE(shouldDropV6MapCommitWithoutExactCargoOwnership(
      false, true, false, cargo, no_candidates));
}

TEST(AvoidanceMapMutation,
     UnknownCargoWithNoExactCandidateOwnershipDropsMapCommit) {
  CargoMapMutationSnapshot cargo;
  CurrentFramePointOwnership no_candidates;
  cargo.authorized = false;
  cargo.tight_geometry_valid = false;
  cargo.owner_points.valid = false;
  EXPECT_TRUE(shouldDropV6MapCommitWithoutExactCargoOwnership(
      true, true, false, cargo, no_candidates));

  cargo.authorized = true;
  cargo.tight_geometry_valid = true;
  cargo.owner_points.valid = true;
  SourcePointKey exact;
  ASSERT_TRUE(makeSourcePointKey(
      pcl::PointXYZ(0.0F, 0.0F, 1.0F), &exact));
  cargo.owner_points.exact_points.insert(exact);
  EXPECT_FALSE(shouldDropV6MapCommitWithoutExactCargoOwnership(
      true, true, false, cargo, no_candidates));
}

TEST(AvoidanceMapMutation,
     ExactCurrentCandidateOwnershipAvoidsWholeCommitDrop) {
  const pcl::PointXYZ candidate_point{0.1F, 0.2F, 0.3F};
  pcl::PointCloud<pcl::PointXYZ> source;
  source.push_back(candidate_point);
  CurrentFramePointOwnership candidates;
  candidates.valid = true;
  candidates.source_frame_identity =
      makeSourceFrameIdentity(9U, 9.0, 1U, source);
  SourcePointKey key;
  ASSERT_TRUE(makeSourcePointKey(candidate_point, &key));
  candidates.exact_points.insert(key);

  EXPECT_FALSE(shouldDropV6MapCommitWithoutExactCargoOwnership(
      true, true, false, CargoMapMutationSnapshot{}, candidates));
  EXPECT_TRUE(candidates.owns(
      candidates.source_frame_identity, candidate_point));
}

TEST(AvoidanceMapMutation,
     StaticConflictCannotBypassCanonicalMapVetoViaCandidateQuarantine) {
  const pcl::PointXYZ candidate_point{0.1F, 0.2F, 0.3F};
  pcl::PointCloud<pcl::PointXYZ> source;
  source.push_back(candidate_point);
  CurrentFramePointOwnership candidates;
  candidates.valid = true;
  candidates.source_frame_identity =
      makeSourceFrameIdentity(9U, 9.0, 1U, source);
  SourcePointKey key;
  ASSERT_TRUE(makeSourcePointKey(candidate_point, &key));
  candidates.exact_points.insert(key);

  EXPECT_TRUE(shouldDropV6MapCommitWithoutExactCargoOwnership(
      true, true, true, CargoMapMutationSnapshot{}, candidates));
}

TEST(AvoidanceMapMutation,
     CandidateOwnershipCannotCrossSourceFrameIdentity) {
  pcl::PointCloud<pcl::PointXYZ> source;
  source.push_back(pcl::PointXYZ(0.0F, 0.0F, 1.0F));
  const SourceFrameIdentity frame =
      makeSourceFrameIdentity(10U, 1.0, 1U, source);
  SourceFrameIdentity stale = frame;
  stale.processing_frame_index = 9U;

  AvoidanceMapMutationSnapshot snapshot;
  snapshot.source_stamp_sec = 1.0;
  snapshot.source_cloud_instance_id = 10U;
  snapshot.source_frame_identity = frame;
  snapshot.pose_identity = poseIdentity();
  snapshot.localization_map_write_authorized = true;
  snapshot.human_points.valid = true;
  snapshot.human_points.source_frame_identity = frame;
  snapshot.cargo_candidate_points.valid = true;
  snapshot.cargo_candidate_points.source_frame_identity = stale;

  EXPECT_FALSE(snapshot.validFor(poseIdentity(), frame, 1.0, 10U));
}

}  // namespace
}  // namespace ndt_slam
