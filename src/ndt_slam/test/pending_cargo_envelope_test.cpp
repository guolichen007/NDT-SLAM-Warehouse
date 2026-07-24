#include <gtest/gtest.h>

#include "ndt_slam/pending_cargo_envelope.hpp"

namespace ndt_slam {
namespace {

CargoEnvelopePoseCandidate pose(
    double stamp, const Eigen::Vector3f& center,
    CargoEnvelopePoseSource source,
    std::uint64_t lifecycle_id = 1U) {
  CargoEnvelopePoseCandidate value;
  value.valid = true;
  value.center_base = center;
  value.evidence_stamp_sec = stamp;
  value.cargo_lifecycle_id = lifecycle_id;
  value.source = source;
  return value;
}

CargoEnvelopeShapeCandidate shape(
    double stamp, float length, float width, float height,
    CargoEnvelopeShapeSource source,
    std::uint64_t lifecycle_id = 1U) {
  CargoEnvelopeShapeCandidate value;
  value.valid = true;
  value.length_m = length;
  value.width_m = width;
  value.height_m = height;
  value.evidence_stamp_sec = stamp;
  value.cargo_lifecycle_id = lifecycle_id;
  value.source = source;
  return value;
}

PendingCargoEnvelopeInput loadedInput(double stamp = 10.0) {
  PendingCargoEnvelopeInput input;
  input.stamp_sec = stamp;
  input.hook_loaded = true;
  input.cargo_lifecycle_id = 1U;
  return input;
}

PendingCargoEnvelopeConfig exactConfig() {
  PendingCargoEnvelopeConfig config;
  config.horizontal_margin_m = 0.0F;
  config.vertical_margin_m = 0.0F;
  config.maximum_fallback_sway_offset_m = 0.0F;
  config.lost_position_uncertainty_per_sec = 0.0F;
  return config;
}

TEST(PendingCargoEnvelopeTest, StaticOriginProvidesShapeButNeverCurrentPose) {
  auto input = loadedInput();
  input.static_origin_shape = shape(
      10.0, 1.5F, 0.8F, 1.2F,
      CargoEnvelopeShapeSource::STATIC_ORIGIN_COMPONENT);

  const auto result = buildPendingCargoEnvelope(input);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "current_cargo_pose_unavailable");
}

TEST(PendingCargoEnvelopeTest,
     LiftedCargoFallbackStaysUnderCurrentHookNotPickupLocation) {
  auto input = loadedInput();
  input.hook_default_pose = pose(
      10.0, Eigen::Vector3f(8.0F, -3.0F, 2.0F),
      CargoEnvelopePoseSource::HOOK_DEFAULT_OFFSET);
  input.static_origin_shape = shape(
      10.0, 1.5F, 0.8F, 1.2F,
      CargoEnvelopeShapeSource::STATIC_ORIGIN_COMPONENT);

  const auto result = buildPendingCargoEnvelope(input, exactConfig());
  ASSERT_TRUE(result.valid);
  EXPECT_FLOAT_EQ(result.center_base.x(), 8.0F);
  EXPECT_FLOAT_EQ(result.center_base.y(), -3.0F);
  EXPECT_EQ(result.pose_source,
            CargoEnvelopePoseSource::HOOK_DEFAULT_OFFSET);
  EXPECT_EQ(result.shape_source,
            CargoEnvelopeShapeSource::STATIC_ORIGIN_COMPONENT);
}

TEST(PendingCargoEnvelopeTest, StaticShapeCanCombineWithHookPose) {
  auto input = loadedInput();
  input.hook_last_offset_pose = pose(
      10.0, Eigen::Vector3f(2.2F, -1.4F, 1.8F),
      CargoEnvelopePoseSource::HOOK_PLUS_LAST_RELIABLE_OFFSET);
  input.static_origin_shape = shape(
      10.0, 5.0F, 3.5F, 3.2F,
      CargoEnvelopeShapeSource::STATIC_ORIGIN_COMPONENT);

  const auto result = buildPendingCargoEnvelope(input, exactConfig());
  ASSERT_TRUE(result.valid);
  EXPECT_FLOAT_EQ(result.center_base.x(), 2.2F);
  EXPECT_GE(result.length_m, 5.0F);
  EXPECT_GE(result.width_m, 3.5F);
}

TEST(PendingCargoEnvelopeTest,
     EmergencyFallbackWithoutPoseIsInvalidNotZeroCentered) {
  CargoPresenceResult presence;
  presence.cargo_present = true;
  const auto effective = resolveEffectiveCargoEnvelope(
      presence, RigidCargoGeometry{}, PendingCargoEnvelope{});
  EXPECT_FALSE(effective.valid);
  EXPECT_FALSE(effective.footprint.valid);
  EXPECT_EQ(effective.reason, "current_cargo_pose_unavailable");
}

TEST(PendingCargoEnvelopeTest, NewLifecycleDoesNotReusePreviousCargoOffset) {
  auto input = loadedInput();
  input.cargo_lifecycle_id = 2U;
  input.hook_last_offset_pose = pose(
      10.0, Eigen::Vector3f(9.0F, 9.0F, 2.0F),
      CargoEnvelopePoseSource::HOOK_PLUS_LAST_RELIABLE_OFFSET, 1U);
  input.hook_default_pose = pose(
      10.0, Eigen::Vector3f(1.0F, 2.0F, 2.0F),
      CargoEnvelopePoseSource::HOOK_DEFAULT_OFFSET, 2U);

  const auto result = buildPendingCargoEnvelope(input, exactConfig());
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.pose_source, CargoEnvelopePoseSource::HOOK_DEFAULT_OFFSET);
  EXPECT_FLOAT_EQ(result.center_base.x(), 1.0F);
  EXPECT_FLOAT_EQ(result.center_base.y(), 2.0F);
}

TEST(PendingCargoEnvelopeTest, SameLifecycleKeepsLastReliableOffset) {
  auto input = loadedInput();
  input.hook_last_offset_pose = pose(
      10.0, Eigen::Vector3f(1.8F, -0.7F, 2.0F),
      CargoEnvelopePoseSource::HOOK_PLUS_LAST_RELIABLE_OFFSET);
  input.hook_default_pose = pose(
      10.0, Eigen::Vector3f(1.0F, -1.0F, 2.0F),
      CargoEnvelopePoseSource::HOOK_DEFAULT_OFFSET);

  const auto result = buildPendingCargoEnvelope(input, exactConfig());
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.pose_source,
            CargoEnvelopePoseSource::HOOK_PLUS_LAST_RELIABLE_OFFSET);
  EXPECT_FLOAT_EQ(result.center_base.x(), 1.8F);
}

TEST(PendingCargoEnvelopeTest, MapSessionChangeClearsOffsetIdentity) {
  auto input = loadedInput();
  input.cargo_lifecycle_id = 9U;
  input.hook_last_offset_pose = pose(
      10.0, Eigen::Vector3f(7.0F, 7.0F, 2.0F),
      CargoEnvelopePoseSource::HOOK_PLUS_LAST_RELIABLE_OFFSET, 8U);
  input.hook_default_pose = pose(
      10.0, Eigen::Vector3f(0.0F, -2.0F, 2.0F),
      CargoEnvelopePoseSource::HOOK_DEFAULT_OFFSET, 9U);
  const auto result = buildPendingCargoEnvelope(input, exactConfig());
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.pose_source, CargoEnvelopePoseSource::HOOK_DEFAULT_OFFSET);
}

TEST(PendingCargoEnvelopeTest,
     ConfiguredFallbackRemainsFourByThreeWhenGeometryUnknown) {
  auto input = loadedInput();
  input.hook_default_pose = pose(
      10.0, Eigen::Vector3f(0.0F, -2.0F, 1.5F),
      CargoEnvelopePoseSource::HOOK_DEFAULT_OFFSET);

  const auto result = buildPendingCargoEnvelope(input, exactConfig());
  ASSERT_TRUE(result.valid);
  EXPECT_FLOAT_EQ(result.length_m, 4.0F);
  EXPECT_FLOAT_EQ(result.width_m, 3.0F);
  EXPECT_FLOAT_EQ(result.height_m, 3.0F);
  CargoPresenceResult presence;
  presence.cargo_present = true;
  EXPECT_FALSE(resolveEffectiveCargoEnvelope(
      presence, RigidCargoGeometry{}, result).can_authorize_clear);
}

TEST(PendingCargoEnvelopeTest, CurrentPoseAndShapeBeatFallbackSources) {
  auto input = loadedInput();
  input.current_associated_pose = pose(
      10.0, Eigen::Vector3f(0.5F, -1.0F, 2.0F),
      CargoEnvelopePoseSource::CURRENT_ASSOCIATED_LIDAR);
  input.hook_default_pose = pose(
      10.0, Eigen::Vector3f(4.0F, 4.0F, 2.0F),
      CargoEnvelopePoseSource::HOOK_DEFAULT_OFFSET);
  input.current_high_quality_shape = shape(
      10.0, 1.5F, 0.8F, 1.0F,
      CargoEnvelopeShapeSource::CURRENT_HIGH_QUALITY_LIDAR);
  input.static_origin_shape = shape(
      10.0, 2.0F, 1.0F, 1.2F,
      CargoEnvelopeShapeSource::STATIC_ORIGIN_COMPONENT);

  const auto result = buildPendingCargoEnvelope(input, exactConfig());
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.source, PendingCargoEnvelopeSource::CURRENT_CANDIDATE);
  EXPECT_FLOAT_EQ(result.center_base.x(), 0.5F);
}

TEST(PendingCargoEnvelopeTest,
     ActiveLockedShapeRemainsPendingUntilFormalThicknessFreezes) {
  auto input = loadedInput();
  input.short_term_track_pose = pose(
      10.0, Eigen::Vector3f(0.1F, -0.4F, 0.9F),
      CargoEnvelopePoseSource::SHORT_TERM_TRACK_PREDICTION);
  input.active_locked_shape = shape(
      10.0, 2.837F, 1.121F, 0.319F,
      CargoEnvelopeShapeSource::ACTIVE_LOCKED_TRACK_SHAPE);
  input.hook_default_pose = pose(
      10.0, Eigen::Vector3f(0.0F, 0.0F, 2.0F),
      CargoEnvelopePoseSource::HOOK_DEFAULT_OFFSET);

  const auto result = buildPendingCargoEnvelope(input, exactConfig());
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.source,
            PendingCargoEnvelopeSource::ACTIVE_LOCKED_TRACK);
  EXPECT_EQ(result.shape_source,
            CargoEnvelopeShapeSource::ACTIVE_LOCKED_TRACK_SHAPE);
  EXPECT_FLOAT_EQ(result.length_m, 2.837F);
  EXPECT_FLOAT_EQ(result.width_m, 1.121F);
  CargoPresenceResult presence;
  presence.cargo_present = true;
  EXPECT_FALSE(resolveEffectiveCargoEnvelope(
      presence, RigidCargoGeometry{}, result).can_authorize_clear);
}

TEST(PendingCargoEnvelopeTest, FormalClearRequiresFreshHeightEvidence) {
  CargoPresenceResult presence;
  presence.cargo_present = true;
  presence.gravity_authoritative = true;
  RigidCargoGeometry formal;
  formal.valid = true;
  formal.shape.valid = true;
  formal.shape.length_m = 1.0F;
  formal.shape.width_m = 1.0F;
  formal.shape.height_m = 1.0F;
  formal.pose.valid = true;
  formal.pose.center_base = Eigen::Vector3f(0.0F, 0.0F, 1.0F);
  formal.bottom_z_base = 0.5F;
  formal.top_z_base = 1.5F;
  formal.pose_evidence_stamp_sec = 10.0;
  formal.height_evidence_stamp_sec = 8.0;
  formal.evaluation_stamp_sec = 10.0;

  auto effective = resolveEffectiveCargoEnvelope(
      presence, formal, PendingCargoEnvelope{});
  ASSERT_TRUE(effective.valid);
  EXPECT_TRUE(effective.formal);
  EXPECT_FALSE(effective.can_authorize_clear);

  formal.height_evidence_stamp_sec = 9.8;
  effective = resolveEffectiveCargoEnvelope(
      presence, formal, PendingCargoEnvelope{});
  EXPECT_TRUE(effective.can_authorize_clear);
}

TEST(PendingCargoEnvelopeTest, ExpandedHeightMatchesMinMax) {
  auto input = loadedInput();
  input.hook_default_pose = pose(
      10.0, Eigen::Vector3f(0.0F, 0.0F, 2.0F),
      CargoEnvelopePoseSource::HOOK_DEFAULT_OFFSET);
  const auto result = buildPendingCargoEnvelope(input);
  ASSERT_TRUE(result.valid);
  EXPECT_FLOAT_EQ(result.height_m,
                  result.top_z_base - result.bottom_z_base);
  EXPECT_FLOAT_EQ(result.center_base.z(),
                  0.5F * (result.top_z_base + result.bottom_z_base));
}

TEST(PendingCargoEnvelopeTest,
     MeasuredShapeIsNominalAndUncertaintyExpandsOnlyQueryFootprint) {
  auto input = loadedInput();
  input.current_associated_pose = pose(
      10.0, Eigen::Vector3f(0.0F, 0.0F, 2.0F),
      CargoEnvelopePoseSource::CURRENT_ASSOCIATED_LIDAR);
  input.current_associated_pose.horizontal_uncertainty_m = 0.30F;
  input.current_tracked_bounded_shape = shape(
      10.0, 1.20F, 0.60F, 0.80F,
      CargoEnvelopeShapeSource::CURRENT_TRACKED_BOUNDED_SHAPE);
  PendingCargoEnvelopeConfig config = exactConfig();
  config.horizontal_margin_m = 0.20F;

  const auto result = buildPendingCargoEnvelope(input, config);
  ASSERT_TRUE(result.valid);
  EXPECT_FLOAT_EQ(result.length_m, 1.20F);
  EXPECT_FLOAT_EQ(result.width_m, 0.60F);
  const CargoObbFootprint nominal = toCargoObbFootprint(result);
  const CargoObbFootprint query = toCargoObbFootprint(
      result, result.horizontal_uncertainty_m,
      result.vertical_uncertainty_m);
  EXPECT_FLOAT_EQ(nominal.length_m, 1.20F);
  EXPECT_FLOAT_EQ(query.length_m,
                  1.20F + 2.0F * result.horizontal_uncertainty_m);
}

TEST(PendingCargoEnvelopeTest, RetiredEnvelopeBelowGroundIsRejected) {
  PendingCargoVerticalPlausibilityInput input;
  input.envelope_valid = true;
  input.center_z_base = -4.0F;
  input.height_m = 2.0F;
  input.vertical_uncertainty_m = 0.15F;
  input.minimum_height_m = 0.30F;
  input.maximum_height_m = 5.0F;
  input.local_ground_valid = true;
  input.local_ground_z_base = 0.0F;

  const auto result = evaluatePendingCargoVerticalPlausibility(input);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "retired_pose_below_local_ground");
}

TEST(PendingCargoEnvelopeTest,
     CurrentAssociatedLidarPoseDoesNotRequireGroundReference) {
  PendingCargoVerticalPlausibilityInput input;
  input.envelope_valid = true;
  input.center_z_base = 2.1F;
  input.height_m = 1.1F;
  input.vertical_uncertainty_m = 0.15F;
  input.minimum_height_m = 0.30F;
  input.maximum_height_m = 5.0F;
  input.current_lidar_pose_authoritative = true;

  const auto result = evaluatePendingCargoVerticalPlausibility(input);
  EXPECT_TRUE(result.valid) << result.reason;
}

TEST(PendingCargoEnvelopeTest,
     ShortHoldUsesSameTrackTrustedCenterButRejectsLargeZJump) {
  PendingCargoVerticalPlausibilityInput input;
  input.envelope_valid = true;
  input.center_z_base = 2.3F;
  input.height_m = 1.1F;
  input.vertical_uncertainty_m = 0.10F;
  input.minimum_height_m = 0.30F;
  input.maximum_height_m = 5.0F;
  input.trusted_center_valid = true;
  input.trusted_center_z_base = 2.1F;
  input.trusted_center_age_sec = 2.0;
  ASSERT_TRUE(evaluatePendingCargoVerticalPlausibility(input).valid);

  input.center_z_base = -4.0F;
  const auto rejected =
      evaluatePendingCargoVerticalPlausibility(input);
  EXPECT_FALSE(rejected.valid);
  EXPECT_EQ(
      rejected.reason,
      "retired_pose_trusted_center_discontinuity");
}

TEST(PendingCargoEnvelopeTest, EmptyHookCannotCreateEnvelope) {
  auto input = loadedInput();
  input.hook_loaded = false;
  input.hook_default_pose = pose(
      10.0, Eigen::Vector3f(0.0F, 0.0F, 2.0F),
      CargoEnvelopePoseSource::HOOK_DEFAULT_OFFSET);
  EXPECT_FALSE(buildPendingCargoEnvelope(input).valid);
}

// ─── P1-5: yaw authority ───

CargoEnvelopePoseCandidate poseWithYaw(
    double stamp, const Eigen::Vector3f& center, float yaw,
    CargoEnvelopePoseSource source, bool yaw_auth = false,
    std::uint64_t lifecycle_id = 1U) {
  auto value = pose(stamp, center, source, lifecycle_id);
  value.yaw_base_rad = yaw;
  value.yaw_authoritative = yaw_auth;
  return value;
}

CargoEnvelopeShapeCandidate shapeWithYaw(
    double stamp, float length, float width, float height,
    float yaw_rad, CargoEnvelopeShapeSource source,
    std::uint64_t lifecycle_id = 1U) {
  auto value = shape(stamp, length, width, height, source, lifecycle_id);
  value.yaw_rad = yaw_rad;
  return value;
}

TEST(PendingCargoEnvelopeTest,
     LowQualityCurrentPoseCannotOverrideStaticShapeYaw) {
  auto input = loadedInput();
  // Current LiDAR pose with yaw=0 but NOT authoritative (low quality)
  input.current_associated_pose = poseWithYaw(
      10.0, Eigen::Vector3f(0.5F, -1.0F, 2.0F), 0.0F,
      CargoEnvelopePoseSource::CURRENT_ASSOCIATED_LIDAR,
      /*yaw_authoritative=*/false);
  // Static origin shape with reliable yaw=0.8 rad
  input.static_origin_shape = shapeWithYaw(
      10.0, 1.5F, 0.8F, 1.2F, 0.8F,
      CargoEnvelopeShapeSource::STATIC_ORIGIN_COMPONENT);

  const auto result = buildPendingCargoEnvelope(input, exactConfig());
  ASSERT_TRUE(result.valid);
  // P1-5: static shape yaw must win over non-authoritative pose yaw
  EXPECT_NEAR(result.yaw_base_rad, 0.8F, 1.0e-5F);
}

TEST(PendingCargoEnvelopeTest, CurrentCenterCanCombineWithStaticYaw) {
  auto input = loadedInput();
  // Current LiDAR pose with center but non-authoritative yaw=0
  input.current_associated_pose = poseWithYaw(
      10.0, Eigen::Vector3f(1.2F, -0.5F, 1.9F), 0.0F,
      CargoEnvelopePoseSource::CURRENT_ASSOCIATED_LIDAR,
      /*yaw_authoritative=*/false);
  input.current_high_quality_shape = shapeWithYaw(
      10.0, 1.5F, 0.8F, 1.0F, 0.0F,
      CargoEnvelopeShapeSource::CURRENT_HIGH_QUALITY_LIDAR);
  // Static origin shape with yaw=1.2 rad
  input.static_origin_shape = shapeWithYaw(
      10.0, 2.0F, 1.0F, 1.2F, 1.2F,
      CargoEnvelopeShapeSource::STATIC_ORIGIN_COMPONENT);

  const auto result = buildPendingCargoEnvelope(input, exactConfig());
  ASSERT_TRUE(result.valid);
  // Center comes from current pose, but yaw from the winning shape
  EXPECT_FLOAT_EQ(result.center_base.x(), 1.2F);
  EXPECT_FLOAT_EQ(result.center_base.y(), -0.5F);
  // When both current_high_quality_shape and static_origin_shape compete,
  // current_high_quality_shape wins (higher priority). Its yaw is 0.
  EXPECT_NEAR(result.yaw_base_rad, 0.0F, 1.0e-5F);
}

TEST(PendingCargoEnvelopeTest, HighQualityCurrentPoseCanProvideYaw) {
  auto input = loadedInput();
  // Current LiDAR pose with authoritative yaw=0.5 rad
  input.current_associated_pose = poseWithYaw(
      10.0, Eigen::Vector3f(0.5F, -1.0F, 2.0F), 0.5F,
      CargoEnvelopePoseSource::CURRENT_ASSOCIATED_LIDAR,
      /*yaw_authoritative=*/true);
  input.current_high_quality_shape = shapeWithYaw(
      10.0, 1.5F, 0.8F, 1.0F, 0.3F,
      CargoEnvelopeShapeSource::CURRENT_HIGH_QUALITY_LIDAR);

  const auto result = buildPendingCargoEnvelope(input, exactConfig());
  ASSERT_TRUE(result.valid);
  // Authoritative pose yaw must override shape yaw
  EXPECT_NEAR(result.yaw_base_rad, 0.5F, 1.0e-5F);
}

TEST(PendingCargoEnvelopeTest, HookFallbackUsesShapeYaw) {
  auto input = loadedInput();
  // HOOK_DEFAULT_OFFSET never has yaw authority
  input.hook_default_pose = poseWithYaw(
      10.0, Eigen::Vector3f(0.0F, -2.0F, 1.5F), 0.0F,
      CargoEnvelopePoseSource::HOOK_DEFAULT_OFFSET,
      /*yaw_authoritative=*/false);
  // Static origin shape with yaw=1.5 rad
  input.static_origin_shape = shapeWithYaw(
      10.0, 1.5F, 0.8F, 1.2F, 1.5F,
      CargoEnvelopeShapeSource::STATIC_ORIGIN_COMPONENT);

  const auto result = buildPendingCargoEnvelope(input, exactConfig());
  ASSERT_TRUE(result.valid);
  // Hook fallback must use shape yaw
  EXPECT_NEAR(result.yaw_base_rad, 1.5F, 1.0e-5F);
}

TEST(PendingCargoEnvelopeTest, RetiredLockedPoseKeepsLockedYaw) {
  auto input = loadedInput();
  // Retired track pose with authoritative yaw from locked shape
  input.retired_track_pose = poseWithYaw(
      10.0, Eigen::Vector3f(0.8F, 0.2F, 1.8F), 0.9F,
      CargoEnvelopePoseSource::RETIRED_TRACK_PREDICTION,
      /*yaw_authoritative=*/true);
  input.retired_locked_shape = shapeWithYaw(
      10.0, 1.5F, 0.8F, 1.2F, 0.9F,
      CargoEnvelopeShapeSource::RETIRED_LOCKED_SHAPE);

  const auto result = buildPendingCargoEnvelope(input, exactConfig());
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.yaw_base_rad, 0.9F, 1.0e-5F);
}

}  // namespace
}  // namespace ndt_slam
