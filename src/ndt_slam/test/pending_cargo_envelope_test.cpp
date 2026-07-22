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

TEST(PendingCargoEnvelopeTest, EmptyHookCannotCreateEnvelope) {
  auto input = loadedInput();
  input.hook_loaded = false;
  input.hook_default_pose = pose(
      10.0, Eigen::Vector3f(0.0F, 0.0F, 2.0F),
      CargoEnvelopePoseSource::HOOK_DEFAULT_OFFSET);
  EXPECT_FALSE(buildPendingCargoEnvelope(input).valid);
}

}  // namespace
}  // namespace ndt_slam
