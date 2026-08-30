#include <gtest/gtest.h>

#include "ndt_slam/cargo_v6_authority_adapter.hpp"

namespace ndt_slam {
namespace {

CanonicalCargoAuthorityInput validInput() {
  CanonicalCargoAuthorityInput input;
  input.mode = CargoAuthorityMode::V6_AUTHORITY;
  input.source_stamp_sec = 10.0;
  input.pose_identity.map_rebuild_generation = 5U;
  input.pose_identity.keyframe_pose_version = 7U;
  input.pose_identity.yaw_authority_generation = 3U;
  input.pose_identity.map_frame_uuid = "map-frame";
  input.pose_identity.yaw_reference_hash = "yaw-ref";
  input.pose_identity.target_snapshot_id = 11U;
  input.identity.valid_input = true;
  input.identity.identity = CargoPhysicalIdentityState::VALIDATED;
  input.identity.lift_confirmed = true;
  input.identity.current_candidate_fresh = true;
  input.identity.geometry_resolved = true;
  input.identity.physical_history_id = 42U;
  input.identity.physical_cargo_epoch_id = 9U;
  input.identity.load_epoch = 4U;
  input.group.valid = true;
  input.group.physical_history_id = 42U;
  input.group.load_epoch = 4U;
  input.group.lifecycle_id = 9U;
  input.group.source_stamp_sec = 10.0;
  input.group.supported_top_valid = true;
  input.group.supported_top_z = 1.5;
  input.group.geometry_resolved = true;
  input.group.resolved_geometry.valid = true;
  input.group.resolved_geometry.footprint_center_base =
      Eigen::Vector2d(0.5, 0.25);
  input.group.resolved_geometry.size = Eigen::Vector3d(1.0, 0.8, 1.0);
  input.group.resolved_geometry.yaw_rad = 0.0;
  input.group.union_points_base = {
      Eigen::Vector3f(0.5F, 0.25F, 0.6F),
      Eigen::Vector3f(0.6F, 0.25F, 1.5F)};
  pcl::PointCloud<pcl::PointXYZ> source_cloud;
  for (const Eigen::Vector3f& point : input.group.union_points_base) {
    source_cloud.push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
  }
  input.source_frame_identity =
      makeSourceFrameIdentity(1U, 10.0, 1U, source_cloud);
  input.geometry.formal_geometry_valid = true;
  input.geometry.physical_history_id = 42U;
  input.bottom.valid = true;
  input.bottom.geometry_valid = true;
  input.bottom.track_id = 42U;
  input.bottom.stamp_sec = 10.0;
  input.bottom.geometry.bottom_z_base = 0.5F;
  input.independent_static_provenance_conflict = false;
  return input;
}

TEST(CargoV6AuthorityAdapter,
     CargoSafetyAuthorityDoesNotImplyMapMutationAuthority) {
  auto input = validInput();
  input.independent_static_provenance_conflict = true;

  const auto output = buildCanonicalCargoAuthoritySnapshot(input);

  EXPECT_TRUE(output.cargo_safety_authorized);
  EXPECT_FALSE(output.cargo_map_mutation_authorized);
  EXPECT_EQ(output.reason, "independent_static_provenance_conflict");
}

TEST(CargoV6AuthorityAdapter, CargoUnknownCannotExpandMapRemovalMask) {
  auto input = validInput();
  input.identity.identity = CargoPhysicalIdentityState::UNKNOWN;

  const auto output = buildCanonicalCargoAuthoritySnapshot(input);

  EXPECT_FALSE(output.cargo_safety_authorized);
  EXPECT_FALSE(output.cargo_map_mutation_authorized);
  EXPECT_TRUE(output.map_mutation.owner_points.voxels.empty());
}

TEST(CargoV6AuthorityAdapter, CargoMapMutationRequiresExactCurrentOwnerPoint) {
  const auto output = buildCanonicalCargoAuthoritySnapshot(validInput());
  ASSERT_TRUE(output.cargo_map_mutation_authorized);

  EXPECT_TRUE(output.map_mutation.owns(
      pcl::PointXYZ(0.5F, 0.25F, 0.6F)));
  EXPECT_FALSE(output.map_mutation.owns(
      pcl::PointXYZ(0.51F, 0.40F, 0.6F)));
}

TEST(CargoV6AuthorityAdapter, ShadowComputesEligibilityWithoutProductAuthority) {
  auto input = validInput();
  input.mode = CargoAuthorityMode::V6_SHADOW;

  const auto output = buildCanonicalCargoAuthoritySnapshot(input);

  EXPECT_TRUE(output.would_authorize_safety);
  EXPECT_TRUE(output.would_authorize_map_mutation);
  EXPECT_FALSE(output.cargo_safety_authorized);
  EXPECT_FALSE(output.cargo_map_mutation_authorized);
}

TEST(CargoV6AuthorityAdapter, StaleGroupCannotOwnSafetyOrMapMutation) {
  auto input = validInput();
  input.group.source_stamp_sec = 9.0;

  const auto output = buildCanonicalCargoAuthoritySnapshot(input);

  EXPECT_FALSE(output.cargo_safety_authorized);
  EXPECT_FALSE(output.cargo_map_mutation_authorized);
}

TEST(CargoV6AuthorityAdapter, MatureStaticConflictIsMapOnlyVeto) {
  const auto input = validInput();
  auto snapshot = std::make_shared<StaticEvidenceSnapshot>();
  snapshot->map_generation = input.pose_identity.map_rebuild_generation;
  snapshot->cell_size_m = 0.25F;
  StaticEvidenceCell cell;
  cell.key = packStaticEvidenceCell(2, 1);
  cell.clean_map_confirmed = true;
  cell.temporally_mature = true;
  cell.min_z = 0.0F;
  cell.max_z = 2.0F;
  snapshot->cells[cell.key] = cell;

  EXPECT_TRUE(cargoGroupOverlapsMatureStaticEvidence(
      input.group, Sophus::SE3d(), input.pose_identity, snapshot));
}

TEST(CargoV6AuthorityAdapter,
     RegistrationHygieneShadowNeverGrantsProductAuthority) {
  const CanonicalCargoAuthoritySnapshot canonical =
      buildCanonicalCargoAuthoritySnapshot(validInput());
  pcl::PointCloud<pcl::PointXYZ> registration;
  registration.push_back(pcl::PointXYZ(0.5F, 0.25F, 0.6F));
  registration.push_back(pcl::PointXYZ(0.5F, 0.40F, 0.6F));
  CargoObbFootprint legacy = canonical.safety_geometry.footprint_base;

  const CargoRegistrationHygieneShadow shadow =
      evaluateCargoRegistrationHygieneShadow(
          registration, true, legacy, canonical);

  EXPECT_TRUE(shadow.valid_input);
  EXPECT_TRUE(shadow.legacy_authorized);
  EXPECT_TRUE(shadow.v6_proposed_authorized);
  EXPECT_EQ(shadow.source_points, 2U);
  EXPECT_EQ(shadow.legacy_removed_points, 2U);
  EXPECT_EQ(shadow.v6_proposed_removed_points, 1U);
  EXPECT_EQ(shadow.intersection_points, 1U);
  EXPECT_EQ(shadow.legacy_only_points, 1U);
  EXPECT_EQ(shadow.v6_only_points, 0U);
  // This value object exposes a counterfactual only; no product write bit is
  // part of the contract.
}

TEST(CargoV6AuthorityAdapter,
     ExactCargoOwnershipPreservesSameVoxelBackgroundPoint) {
  const CanonicalCargoAuthoritySnapshot canonical =
      buildCanonicalCargoAuthoritySnapshot(validInput());
  ASSERT_TRUE(canonical.cargo_map_mutation_authorized);
  EXPECT_TRUE(canonical.map_mutation.owns(
      pcl::PointXYZ(0.5F, 0.25F, 0.6F)));
  EXPECT_FALSE(canonical.map_mutation.owns(
      pcl::PointXYZ(0.51F, 0.25F, 0.6F)));
}

}  // namespace
}  // namespace ndt_slam
