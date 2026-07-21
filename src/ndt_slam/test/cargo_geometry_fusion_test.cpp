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
