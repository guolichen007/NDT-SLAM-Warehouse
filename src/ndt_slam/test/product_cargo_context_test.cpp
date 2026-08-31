#include "ndt_slam/product_cargo_context.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

ProductCargoContext authorized(CargoAuthorityMode mode,
                               std::uint64_t id) {
  ProductCargoContext context;
  context.mode = mode;
  context.valid_input = true;
  context.identity_authorized = true;
  context.geometry_authorized = true;
  context.bottom_authorized = true;
  context.safety_authorized = true;
  context.clear_authorized = true;
  context.self_removal_authorized = true;
  context.map_mutation_authorized = true;
  context.cargo_id = id;
  return context;
}

struct V6LiveSelfRemovalFixture {
  double source_stamp_sec = 10.0;
  PoseAuthorityIdentity pose_identity;
  SourceFrameIdentity source_frame_identity;
  CanonicalCargoAuthoritySnapshot canonical;
  ProductCargoContext product;
};

V6LiveSelfRemovalFixture makeV6LiveSelfRemovalFixture(
    bool static_conflict = false) {
  V6LiveSelfRemovalFixture fixture;
  fixture.pose_identity.map_rebuild_generation = 5U;
  fixture.pose_identity.keyframe_pose_version = 7U;
  fixture.pose_identity.yaw_authority_generation = 3U;
  fixture.pose_identity.map_frame_uuid = "map-frame";
  fixture.pose_identity.yaw_reference_hash = "yaw-ref";
  fixture.pose_identity.target_snapshot_id = 11U;

  pcl::PointCloud<pcl::PointXYZ> source_cloud;
  source_cloud.push_back(pcl::PointXYZ(0.5F, 0.25F, 0.6F));
  source_cloud.push_back(pcl::PointXYZ(0.6F, 0.25F, 1.5F));
  fixture.source_frame_identity = makeSourceFrameIdentity(
      1U, fixture.source_stamp_sec, 1U, source_cloud);
  fixture.canonical.valid_input = true;
  fixture.canonical.cargo_safety_authorized = true;
  fixture.canonical.cargo_map_mutation_authorized = !static_conflict;
  fixture.canonical.source_stamp_sec = fixture.source_stamp_sec;
  fixture.canonical.pose_identity = fixture.pose_identity;
  fixture.canonical.physical_history_id = 42U;
  fixture.canonical.physical_cargo_epoch_id = 9U;
  fixture.canonical.safety_geometry.valid = true;
  fixture.canonical.map_mutation.authorized = !static_conflict;
  fixture.canonical.map_mutation.tight_geometry_valid = true;
  fixture.canonical.map_mutation.center_x = 0.5F;
  fixture.canonical.map_mutation.center_y = 0.25F;
  fixture.canonical.map_mutation.min_z = 0.5F;
  fixture.canonical.map_mutation.max_z = 1.5F;
  fixture.canonical.map_mutation.half_length = 0.5F;
  fixture.canonical.map_mutation.half_width = 0.4F;
  fixture.canonical.map_mutation.owner_points.valid = true;
  fixture.canonical.map_mutation.owner_points.source_frame_identity =
      fixture.source_frame_identity;
  for (const pcl::PointXYZ& point : source_cloud.points) {
    SourcePointKey key;
    if (makeSourcePointKey(point, &key)) {
      fixture.canonical.map_mutation.owner_points.exact_points.insert(key);
    }
  }

  fixture.product = authorized(CargoAuthorityMode::V6_AUTHORITY, 42U);
  fixture.product.source_stamp_sec = fixture.source_stamp_sec;
  fixture.product.cargo_lifecycle_id = 9U;
  fixture.product.self_removal_authorized =
      fixture.canonical.cargo_map_mutation_authorized;
  fixture.product.map_mutation_authorized =
      fixture.canonical.cargo_map_mutation_authorized;
  return fixture;
}

TEST(ProductCargoContext, V6BypassesLegacyFormalGateInAuthorityMode) {
  ProductCargoContext legacy;
  legacy.mode = CargoAuthorityMode::LEGACY;
  legacy.reason = "legacy_formal_gate_closed";
  const ProductCargoContext v6 =
      authorized(CargoAuthorityMode::V6_AUTHORITY, 42U);

  const auto selected = selectProductCargoContext(
      CargoAuthorityMode::V6_AUTHORITY, legacy, v6, false, false);

  EXPECT_TRUE(selected.product.safety_authorized);
  EXPECT_EQ(selected.product.cargo_id, 42U);
  EXPECT_EQ(selected.reason, "v6_product");
}

TEST(ProductCargoContext, V6InvalidCannotFallbackToLegacyClear) {
  const ProductCargoContext legacy =
      authorized(CargoAuthorityMode::LEGACY, 7U);
  ProductCargoContext v6;
  v6.mode = CargoAuthorityMode::V6_AUTHORITY;
  v6.reason = "identity_not_current_validated";

  const auto selected = selectProductCargoContext(
      CargoAuthorityMode::V6_AUTHORITY, legacy, v6, false, true);

  EXPECT_FALSE(selected.product.clear_authorized);
  EXPECT_FALSE(selected.product.map_mutation_authorized);
  EXPECT_TRUE(selected.legacy_clear_rejected);
  EXPECT_EQ(selected.reason, "v6_invalid_fail_closed");
}

TEST(ProductCargoContext, LegacyModeKeepsExactExistingGate) {
  const ProductCargoContext legacy =
      authorized(CargoAuthorityMode::LEGACY, 11U);
  const ProductCargoContext v6 =
      authorized(CargoAuthorityMode::V6_AUTHORITY, 22U);

  const auto selected = selectProductCargoContext(
      CargoAuthorityMode::LEGACY, legacy, v6, false, false);

  EXPECT_EQ(selected.product.cargo_id, 11U);
  EXPECT_TRUE(selected.product.clear_authorized);
  EXPECT_EQ(selected.reason, "legacy_product");
}

TEST(ProductCargoContext, ShadowModeCannotChangeProductDecision) {
  ProductCargoContext legacy;
  legacy.mode = CargoAuthorityMode::LEGACY;
  legacy.reason = "legacy_formal_gate_closed";
  const ProductCargoContext v6 =
      authorized(CargoAuthorityMode::V6_AUTHORITY, 22U);

  const auto selected = selectProductCargoContext(
      CargoAuthorityMode::V6_SHADOW, legacy, v6, false, false);

  EXPECT_FALSE(selected.product.safety_authorized);
  EXPECT_EQ(selected.product.cargo_id, 0U);
  EXPECT_EQ(selected.reason, "shadow_legacy_product");
}

TEST(ProductCargoContext, V6InvalidMayRetainSameAuthorityPositiveOnly) {
  ProductCargoContext legacy;
  legacy.clear_authorized = true;
  ProductCargoContext v6;
  v6.mode = CargoAuthorityMode::V6_AUTHORITY;

  const auto selected = selectProductCargoContext(
      CargoAuthorityMode::V6_AUTHORITY, legacy, v6, true, true);

  EXPECT_TRUE(selected.legacy_positive_hazard_retained);
  EXPECT_TRUE(selected.legacy_clear_rejected);
  EXPECT_FALSE(selected.product.clear_authorized);
}

TEST(ProductCargoContext, V6SelfRemovalRequiresProductSelfRemovalAuthority) {
  auto fixture = makeV6LiveSelfRemovalFixture();
  fixture.product.self_removal_authorized = false;

  EXPECT_FALSE(shouldRemoveV6LiveCargoSelfPoint(
      fixture.product, fixture.canonical, fixture.source_frame_identity,
      fixture.pose_identity, fixture.source_stamp_sec,
      pcl::PointXYZ(0.5F, 0.25F, 0.6F)));
}

TEST(ProductCargoContext, V6SelfRemovalCannotConsumeLegacyIdentityCore) {
  const auto fixture = makeV6LiveSelfRemovalFixture();
  // This point represents a Legacy hook core point. It lies inside the broad
  // Cargo geometry but was not owned by the current V6 source frame.
  EXPECT_FALSE(shouldRemoveV6LiveCargoSelfPoint(
      fixture.product, fixture.canonical, fixture.source_frame_identity,
      fixture.pose_identity, fixture.source_stamp_sec,
      pcl::PointXYZ(0.55F, 0.25F, 0.6F)));
}

TEST(ProductCargoContext,
     V6ExternalObstacleInsideExpandedObbIsPreservedWithoutOwnership) {
  const auto fixture = makeV6LiveSelfRemovalFixture();

  EXPECT_FALSE(shouldRemoveV6LiveCargoSelfPoint(
      fixture.product, fixture.canonical, fixture.source_frame_identity,
      fixture.pose_identity, fixture.source_stamp_sec,
      pcl::PointXYZ(0.5F, 0.35F, 0.6F)));
}

TEST(ProductCargoContext, V6CurrentCanonicalOwnedCargoPointIsRemoved) {
  const auto fixture = makeV6LiveSelfRemovalFixture();

  EXPECT_TRUE(shouldRemoveV6LiveCargoSelfPoint(
      fixture.product, fixture.canonical, fixture.source_frame_identity,
      fixture.pose_identity, fixture.source_stamp_sec,
      pcl::PointXYZ(0.5F, 0.25F, 0.6F)));
}

TEST(ProductCargoContext, V6AdjacentPersonCannotBeRemovedAsCargoSelf) {
  const auto fixture = makeV6LiveSelfRemovalFixture();

  EXPECT_FALSE(shouldRemoveV6LiveCargoSelfPoint(
      fixture.product, fixture.canonical, fixture.source_frame_identity,
      fixture.pose_identity, fixture.source_stamp_sec,
      pcl::PointXYZ(0.75F, 0.25F, 1.2F)));
}

TEST(ProductCargoContext, V6PreviousOrSweptMaskRequiresCurrentV6Evidence) {
  auto fixture = makeV6LiveSelfRemovalFixture();
  SourceFrameIdentity previous_frame = fixture.source_frame_identity;
  previous_frame.sensor_source_stamp_sec = 9.0;

  EXPECT_FALSE(shouldRemoveV6LiveCargoSelfPoint(
      fixture.product, fixture.canonical, previous_frame,
      fixture.pose_identity, 9.0,
      pcl::PointXYZ(0.5F, 0.25F, 0.6F)));
}

TEST(ProductCargoContext,
     V6LegacyTrackingResidualCannotExpandProductSelfMask) {
  EXPECT_FLOAT_EQ(
      cargoLiveSelfRemovalMargin(
          CargoAuthorityMode::V6_AUTHORITY, 0.1F, 0.5F, 0.3F, 0.05F),
      0.15F);
  EXPECT_FLOAT_EQ(
      cargoLiveSelfRemovalMargin(
          CargoAuthorityMode::LEGACY, 0.1F, 0.5F, 0.3F, 0.05F),
      0.45F);
}

TEST(ProductCargoContext,
     V6StaticConflictDisablesSelfRemovalButKeepsPositiveSafety) {
  const auto fixture = makeV6LiveSelfRemovalFixture(true);
  ASSERT_TRUE(fixture.canonical.cargo_safety_authorized);
  ASSERT_FALSE(fixture.canonical.cargo_map_mutation_authorized);
  EXPECT_TRUE(fixture.product.safety_authorized);
  EXPECT_FALSE(fixture.product.self_removal_authorized);
  EXPECT_FALSE(shouldRemoveV6LiveCargoSelfPoint(
      fixture.product, fixture.canonical, fixture.source_frame_identity,
      fixture.pose_identity, fixture.source_stamp_sec,
      pcl::PointXYZ(0.5F, 0.25F, 0.6F)));
}

}  // namespace
}  // namespace ndt_slam
