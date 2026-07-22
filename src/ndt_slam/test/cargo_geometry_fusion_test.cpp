#include "ndt_slam/cargo_geometry_fusion.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

CargoGeometryFrame frame(double stamp, float static_height = 1.50F,
                         float diff_height = 1.55F) {
  CargoGeometryFrame value;
  value.cargo_lifecycle_id = 11U;
  value.track_segment_id = 1U;
  value.stamp_sec = stamp;
  value.center_valid = true;
  value.center = Eigen::Vector3f(0.0F, 0.0F, 2.0F);
  value.footprint_valid = true;
  value.length_m = 4.0F;
  value.width_m = 1.6F;
  value.yaw_rad = 0.2F;
  value.observed_top_valid = true;
  value.observed_top_m = 2.75F;
  value.top_uncertainty_m = 0.05F;
  value.tracking_uncertainty_m = 0.04F;
  value.thickness = {
      {CargoThicknessSource::STATIC_ORIGIN_TOP_SUPPORT,
       static_height, 0.12F, 0.9F, true},
      {CargoThicknessSource::MAP_DIFF_REVEALED_SUPPORT,
       diff_height, 0.15F, 0.8F, true}};
  return value;
}

TEST(CargoGeometryFusionTest, FreezesAfterIndependentConsistentSources) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 2;
  CargoGeometryFusion fusion(config);
  auto result = fusion.update(frame(1.0));
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.frozen);
  result = fusion.update(frame(1.1));
  ASSERT_TRUE(result.frozen);
  EXPECT_NEAR(result.height_m, 1.52F, 0.08F);
  EXPECT_LT(result.conservative_bottom_m, result.bottom_m);
}

TEST(CargoGeometryFusionTest, ConfiguredFallbackIsNotIndependentEvidence) {
  CargoGeometryFusion fusion;
  auto value = frame(1.0);
  value.thickness.resize(1U);
  value.thickness.push_back({
      CargoThicknessSource::CONFIGURED_FALLBACK,
      1.5F, 0.5F, 0.4F, true});
  const auto result = fusion.update(value);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "independent_thickness_sources_insufficient");
}

TEST(CargoGeometryFusionTest, ConflictingSourcesDoNotFreeze) {
  CargoGeometryFusion fusion;
  const auto result = fusion.update(frame(1.0, 1.0F, 1.8F));
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "thickness_source_disagreement");
}

TEST(CargoGeometryFusionTest, FrozenShapeOnlyUpdatesCenterAndBottom) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 1;
  CargoGeometryFusion fusion(config);
  auto result = fusion.update(frame(1.0));
  ASSERT_TRUE(result.frozen);
  const float frozen_length = result.length_m;
  auto next = frame(1.1);
  next.track_segment_id = 2U;
  next.center = Eigen::Vector3f(1.0F, 2.0F, 3.0F);
  next.length_m = 9.0F;
  next.observed_top_m = 3.75F;
  result = fusion.update(next);
  EXPECT_FLOAT_EQ(result.length_m, frozen_length);
  EXPECT_FLOAT_EQ(result.center.x(), 1.0F);
  EXPECT_EQ(result.track_segment_id, 2U);
  EXPECT_NEAR(result.bottom_m, 3.75F - result.height_m, 1.0e-5F);
}

TEST(CargoGeometryFusionTest, LargerMeasurementExpandsImmediately) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 1;
  config.minimum_conservative_length_m = 0.0F;
  config.minimum_conservative_width_m = 0.0F;
  CargoGeometryFusion fusion(config);
  auto result = fusion.update(frame(1.0));
  ASSERT_TRUE(result.frozen);
  auto larger = frame(1.1);
  larger.length_m = 4.5F;
  larger.width_m = 2.0F;
  larger.dimension_observation_complete = true;
  larger.dimension_support_points = 100U;
  larger.dimension_shape_confidence = 0.9F;
  result = fusion.update(larger);
  EXPECT_FLOAT_EQ(result.length_m, 4.5F);
  EXPECT_FLOAT_EQ(result.width_m, 2.0F);
}

TEST(CargoGeometryFusionTest, TinyClusterCannotShrinkFrozenEnvelope) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 1;
  CargoGeometryFusion fusion(config);
  auto result = fusion.update(frame(1.0));
  ASSERT_TRUE(result.frozen);
  const float length = result.length_m;
  auto tiny = frame(1.1);
  tiny.length_m = 1.0F;
  tiny.width_m = 0.5F;
  tiny.dimension_observation_complete = false;
  tiny.dimension_support_points = 5U;
  tiny.dimension_shape_confidence = 0.2F;
  result = fusion.update(tiny);
  EXPECT_FLOAT_EQ(result.length_m, length);
}

TEST(CargoGeometryFusionTest, ShrinkRequiresConsecutiveQualityEvidence) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 1;
  config.conservative_shrink_confirm_frames = 3;
  config.maximum_shrink_per_frame_m = 0.05F;
  CargoGeometryFusion fusion(config);
  auto result = fusion.update(frame(1.0));
  ASSERT_TRUE(result.frozen);
  auto smaller = frame(1.1);
  smaller.length_m = 3.0F;
  smaller.width_m = 1.2F;
  smaller.dimension_observation_complete = true;
  smaller.dimension_support_points = 100U;
  smaller.dimension_shape_confidence = 0.9F;
  const float original = result.length_m;
  EXPECT_FLOAT_EQ(fusion.update(smaller).length_m, original);
  smaller.stamp_sec = 1.2;
  EXPECT_FLOAT_EQ(fusion.update(smaller).length_m, original);
  smaller.stamp_sec = 1.3;
  result = fusion.update(smaller);
  EXPECT_NEAR(result.length_m, original - 0.05F, 1.0e-6F);
}

TEST(CargoGeometryFusionTest, ObservationGapRestartsConfirmation) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 2;
  config.maximum_observation_gap_sec = 0.5;
  CargoGeometryFusion fusion(config);
  EXPECT_EQ(fusion.update(frame(1.0)).confirm_frames, 1);
  const auto result = fusion.update(frame(2.0));
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.frozen);
  EXPECT_EQ(result.confirm_frames, 1);
}

TEST(CargoGeometryFusionTest, DuplicateStampCannotFreezeGeometry) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 2;
  CargoGeometryFusion fusion(config);
  EXPECT_EQ(fusion.update(frame(1.0)).confirm_frames, 1);
  const auto result = fusion.update(frame(1.0));
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.frozen);
  EXPECT_EQ(result.reason, "source_time_invalid_or_rollback");
}

}  // namespace
}  // namespace ndt_slam
