#include "ndt_slam/cargo_geometry_fusion.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

CargoGeometryFrame frame(double stamp, float live_height = 1.50F,
                         float diff_height = 1.55F) {
  CargoGeometryFrame value;
  value.cargo_lifecycle_id = 11U;
  value.track_segment_id = 1U;
  value.stamp_sec = stamp;
  value.warning_track_stable = true;
  value.formal_track_locked = true;
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
  value.dimension_observation_complete = true;
  value.dimension_support_points = 100U;
  value.dimension_shape_confidence = 0.90F;
  value.thickness = {
      {CargoThicknessSource::MAP_DIFF_REVEALED_SUPPORT,
       diff_height, 0.15F, 0.8F, true},
      {CargoThicknessSource::LIVE_VISIBLE_EXTENT,
       live_height, 0.12F, 0.9F, true}};
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

TEST(CargoGeometryFusionTest,
     InvalidConfigIsRetainedAndCannotSilentlyRestoreFormalDefaults) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 0;
  config.minimum_height_m = 2.0F;
  config.maximum_height_m = 1.0F;
  config.minimum_live_dimension_support = 0U;
  CargoGeometryFusion fusion;

  const CargoConfigValidationResult validation = fusion.setConfig(config);
  EXPECT_FALSE(validation.valid);
  EXPECT_NE(validation.summary().find("minimum_confirm_frames"),
            std::string::npos);
  EXPECT_NE(validation.summary().find("maximum_height_m"),
            std::string::npos);
  EXPECT_NE(validation.summary().find("minimum_live_dimension_support"),
            std::string::npos);
  EXPECT_EQ(fusion.config().minimum_confirm_frames, 0);
  EXPECT_FLOAT_EQ(fusion.config().minimum_height_m, 2.0F);
  EXPECT_FLOAT_EQ(fusion.config().maximum_height_m, 1.0F);

  const CargoFrozenGeometry result = fusion.update(frame(1.0));
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.frozen);
  EXPECT_FALSE(result.formal_authorized);
  EXPECT_EQ(result.authorization, CargoGeometryAuthorization::PENDING);
  EXPECT_NE(result.reason.find("invalid_geometry_fusion_config"),
            std::string::npos);
}

TEST(CargoGeometryFusionTest,
     AuthoritativeOriginAndLiveExtentCanFreezeFormalThickness) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 1;
  CargoGeometryFusion fusion(config);
  auto value = frame(1.0);
  value.thickness.front().source =
      CargoThicknessSource::STATIC_ORIGIN_TOP_SUPPORT;
  const auto result = fusion.update(value);
  EXPECT_TRUE(result.valid) << result.reason;
  EXPECT_TRUE(result.frozen);
  EXPECT_TRUE(result.formal_authorized);
  EXPECT_FALSE(result.degraded_live_only);
  EXPECT_EQ(result.independent_sources, 2U);
}

TEST(CargoGeometryFusionTest,
     StableLiveOnlyHeightFormsPositiveOnlyWithoutFormalAuthority) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 2;
  config.shape_confirmation_window_frames = 4;
  config.positive_only_confirm_frames = 4;
  config.allow_positive_only_without_static_baseline = true;
  CargoGeometryFusion fusion(config);
  auto live_only = frame(1.0);
  live_only.thickness = {
      {CargoThicknessSource::LIVE_VISIBLE_EXTENT,
       1.50F, 0.12F, 0.9F, true}};

  EXPECT_FALSE(fusion.update(live_only).frozen);
  live_only.stamp_sec = 1.1;
  EXPECT_FALSE(fusion.update(live_only).frozen);
  live_only.stamp_sec = 1.2;
  EXPECT_FALSE(fusion.update(live_only).frozen);
  live_only.stamp_sec = 1.3;
  const auto result = fusion.update(live_only);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_TRUE(result.frozen);
  EXPECT_FALSE(result.formal_authorized);
  EXPECT_TRUE(result.degraded_live_only);
  EXPECT_EQ(result.independent_sources, 1U);
  EXPECT_GE(
      result.height_uncertainty_m,
      config.positive_only_uncertainty_floor_m);
  EXPECT_EQ(result.authorization,
            CargoGeometryAuthorization::POSITIVE_ONLY);
  EXPECT_EQ(result.reason, "geometry_frozen_positive_only_live_bound");
}

TEST(CargoGeometryFusionTest,
     LiveOnlyHeightRemainsBlockedWhenDegradedModeIsDisabled) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 1;
  config.allow_positive_only_without_static_baseline = false;
  config.allow_positive_only_on_source_conflict = false;
  CargoGeometryFusion fusion(config);
  auto live_only = frame(1.0);
  live_only.thickness = {
      {CargoThicknessSource::LIVE_VISIBLE_EXTENT,
       1.50F, 0.12F, 0.9F, true}};

  const auto result = fusion.update(live_only);
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.frozen);
  EXPECT_FALSE(result.formal_authorized);
  EXPECT_EQ(result.reason, "independent_thickness_sources_insufficient");
}

TEST(CargoGeometryFusionTest,
     StableWarningTrackCanFreezeBeforeFormalTrackLock) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 1;
  config.shape_confirmation_window_frames = 2;
  config.positive_only_confirm_frames = 2;
  CargoGeometryFusion fusion(config);
  auto live_only = frame(1.0);
  live_only.formal_track_locked = false;
  live_only.thickness = {
      {CargoThicknessSource::LIVE_VISIBLE_EXTENT,
       1.50F, 0.12F, 0.9F, true}};

  EXPECT_FALSE(fusion.update(live_only).frozen);
  live_only.stamp_sec = 1.1;
  const auto result = fusion.update(live_only);
  EXPECT_TRUE(result.valid) << result.reason;
  EXPECT_TRUE(result.frozen);
  EXPECT_FALSE(result.formal_authorized);
  EXPECT_EQ(result.authorization,
            CargoGeometryAuthorization::POSITIVE_ONLY);
}

TEST(CargoGeometryFusionTest,
     FullThicknessPairCannotGrantFormalBeforeFormalTrackLock) {
  CargoGeometryFusion fusion;
  auto value = frame(1.0, 1.04F, 1.00F);
  value.formal_track_locked = false;
  CargoFrozenGeometry result;
  for (int index = 0; index < 5; ++index) {
    value.stamp_sec = 1.0 + 0.1 * index;
    result = fusion.update(value);
  }
  ASSERT_TRUE(result.frozen) << result.reason;
  EXPECT_FALSE(result.formal_authorized);
  EXPECT_EQ(result.authorization,
            CargoGeometryAuthorization::POSITIVE_ONLY);
}

TEST(CargoGeometryFusionTest,
     FiveStableThicknessFramesInEightTolerateOneOutlier) {
  CargoGeometryFusionConfig config;
  config.positive_only_confirm_frames = 5;
  CargoGeometryFusion fusion(config);
  auto value = frame(1.0);
  value.thickness = {{
      CargoThicknessSource::LIVE_VISIBLE_EXTENT,
      1.50F, 0.12F, 0.9F, true,
      CargoThicknessConstraint::LOWER_BOUND}};
  CargoFrozenGeometry result;
  for (int index = 0; index < 6; ++index) {
    value.stamp_sec = 1.0 + 0.1 * index;
    value.thickness.front().height_m = index == 2 ? 1.90F : 1.50F;
    result = fusion.update(value);
  }
  ASSERT_TRUE(result.frozen) << result.reason;
  EXPECT_EQ(result.confirm_frames, 5);
  EXPECT_NEAR(result.height_m, 1.50F, 1.0e-6F);
  EXPECT_EQ(result.authorization,
            CargoGeometryAuthorization::POSITIVE_ONLY);
}

TEST(CargoGeometryFusionTest,
     InitialDimensionsUseWindowMedianInsteadOfLastQualifiedFrame) {
  CargoGeometryFusion fusion;
  auto value = frame(1.0);
  value.formal_track_locked = false;
  value.thickness = {{
      CargoThicknessSource::LIVE_VISIBLE_EXTENT,
      1.50F, 0.12F, 0.9F, true,
      CargoThicknessConstraint::LOWER_BOUND}};
  CargoFrozenGeometry result;
  for (int index = 0; index < 5; ++index) {
    value.stamp_sec = 1.0 + 0.1 * index;
    value.length_m = index == 4 ? 5.0F : 2.40F;
    value.width_m = index == 4 ? 2.4F : 1.20F;
    result = fusion.update(value);
  }
  ASSERT_TRUE(result.frozen) << result.reason;
  EXPECT_NEAR(result.length_m, 2.40F, 1.0e-6F);
  EXPECT_NEAR(result.width_m, 1.20F, 1.0e-6F);
}

TEST(CargoGeometryFusionTest,
     LiveOnlyCandidateCannotFreezeWithoutStableWarningIdentity) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 1;
  config.shape_confirmation_window_frames = 2;
  config.positive_only_confirm_frames = 2;
  CargoGeometryFusion fusion(config);
  auto live_only = frame(1.0);
  live_only.formal_track_locked = false;
  live_only.warning_track_stable = false;
  live_only.thickness = {
      {CargoThicknessSource::LIVE_VISIBLE_EXTENT,
       1.50F, 0.12F, 0.9F, true}};

  EXPECT_FALSE(fusion.update(live_only).frozen);
  live_only.stamp_sec = 1.1;
  const auto result = fusion.update(live_only);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.authorization, CargoGeometryAuthorization::PENDING);
  EXPECT_EQ(result.reason, "independent_thickness_sources_insufficient");
}

TEST(CargoGeometryFusionTest,
     DegradedGeometryPromotesAfterAuthoritativePairConfirms) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 2;
  config.shape_confirmation_window_frames = 4;
  config.positive_only_confirm_frames = 4;
  CargoGeometryFusion fusion(config);
  auto live_only = frame(1.0);
  live_only.thickness = {
      {CargoThicknessSource::LIVE_VISIBLE_EXTENT,
       1.50F, 0.12F, 0.9F, true}};
  for (int index = 0; index < 4; ++index) {
    live_only.stamp_sec = 1.0 + 0.1 * index;
    fusion.update(live_only);
  }
  ASSERT_TRUE(fusion.result().degraded_live_only);

  auto authoritative = frame(1.4);
  auto result = fusion.update(authoritative);
  EXPECT_FALSE(result.formal_authorized);
  authoritative.stamp_sec = 1.5;
  result = fusion.update(authoritative);
  EXPECT_TRUE(result.formal_authorized);
  EXPECT_FALSE(result.degraded_live_only);
  EXPECT_EQ(result.independent_sources, 2U);
}

TEST(CargoGeometryFusionTest,
     FormalThicknessIsInvariantToPositiveOnlyPromotionPath) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 2;
  config.shape_confirmation_window_frames = 4;
  config.positive_only_confirm_frames = 2;

  CargoGeometryFusion direct(config);
  auto authoritative = frame(1.0, 1.10F, 1.00F);
  authoritative.thickness[0].uncertainty_m = 0.15F;
  authoritative.thickness[0].confidence = 0.80F;
  authoritative.thickness[1].uncertainty_m = 0.12F;
  authoritative.thickness[1].confidence = 0.90F;
  EXPECT_FALSE(direct.update(authoritative).frozen);
  authoritative.stamp_sec = 1.1;
  const CargoFrozenGeometry direct_result = direct.update(authoritative);
  ASSERT_TRUE(direct_result.formal_authorized) << direct_result.reason;

  CargoGeometryFusion promoted(config);
  auto live_only = frame(1.0, 1.10F, 1.00F);
  live_only.thickness = {authoritative.thickness[1]};
  EXPECT_FALSE(promoted.update(live_only).frozen);
  live_only.stamp_sec = 1.1;
  ASSERT_TRUE(promoted.update(live_only).degraded_live_only);
  authoritative.stamp_sec = 1.2;
  EXPECT_FALSE(promoted.update(authoritative).formal_authorized);
  authoritative.stamp_sec = 1.3;
  const CargoFrozenGeometry promoted_result = promoted.update(authoritative);
  ASSERT_TRUE(promoted_result.formal_authorized) << promoted_result.reason;

  EXPECT_NEAR(promoted_result.height_m, direct_result.height_m, 1.0e-6F);
  EXPECT_NEAR(promoted_result.height_uncertainty_m,
              direct_result.height_uncertainty_m, 1.0e-6F);
  EXPECT_NEAR(promoted_result.bottom_m, direct_result.bottom_m, 1.0e-6F);
}

TEST(CargoGeometryFusionTest,
     PositiveOnlyFramesCannotCountTowardInitialFormalFreeze) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 3;
  config.positive_only_confirm_frames = 4;
  config.shape_confirmation_window_frames = 8;
  CargoGeometryFusion fusion(config);
  auto value = frame(1.0);
  value.thickness = {{
      CargoThicknessSource::LIVE_VISIBLE_EXTENT,
      1.50F, 0.12F, 0.9F, true,
      CargoThicknessConstraint::LOWER_BOUND}};
  for (int index = 0; index < 2; ++index) {
    value.stamp_sec = 1.0 + 0.1 * index;
    EXPECT_FALSE(fusion.update(value).frozen);
  }
  value = frame(1.2);
  auto result = fusion.update(value);
  EXPECT_FALSE(result.frozen);
  EXPECT_EQ(result.confirm_frames, 1);
  value.stamp_sec = 1.3;
  EXPECT_FALSE(fusion.update(value).frozen);
  value.stamp_sec = 1.4;
  result = fusion.update(value);
  EXPECT_TRUE(result.formal_authorized) << result.reason;
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

TEST(CargoGeometryFusionTest,
     StaticOneMeterAndLiveLowerBoundOnePointSixBecomePositiveOnly) {
  CargoGeometryFusion fusion;
  CargoGeometryFrame value = frame(1.0, 1.60F, 1.00F);
  value.thickness[1].constraint = CargoThicknessConstraint::LOWER_BOUND;
  CargoFrozenGeometry result;
  for (int index = 0; index < 5; ++index) {
    value.stamp_sec = 1.0 + 0.1 * index;
    result = fusion.update(value);
  }
  ASSERT_TRUE(result.frozen) << result.reason;
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.formal_authorized);
  EXPECT_EQ(result.authorization,
            CargoGeometryAuthorization::POSITIVE_ONLY);
  EXPECT_TRUE(result.source_conflict);
  EXPECT_GE(result.height_m, 1.60F);
  EXPECT_GE(result.thickness_upper_bound_m, result.height_m);
}

TEST(CargoGeometryFusionTest,
     ConflictingStaticAndLiveBoundsRespectPositiveOnlyConflictSwitch) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 1;
  config.allow_positive_only_without_static_baseline = true;
  config.allow_positive_only_on_source_conflict = false;
  CargoGeometryFusion fusion(config);
  CargoGeometryFrame value = frame(1.0, 1.60F, 1.00F);
  value.thickness[1].constraint = CargoThicknessConstraint::LOWER_BOUND;
  const CargoFrozenGeometry result = fusion.update(value);
  EXPECT_FALSE(result.frozen);
  EXPECT_EQ(result.authorization, CargoGeometryAuthorization::PENDING);
  EXPECT_EQ(result.reason,
            "thickness_full_measurement_confirmation_pending");
}

TEST(CargoGeometryFusionTest, CompatibleFullMeasurementsBecomeFormal) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 2;
  CargoGeometryFusion fusion(config);
  CargoGeometryFrame value = frame(1.0, 1.04F, 1.00F);
  value.thickness[0].constraint =
      CargoThicknessConstraint::FULL_MEASUREMENT;
  value.thickness[1].constraint =
      CargoThicknessConstraint::FULL_MEASUREMENT;
  EXPECT_FALSE(fusion.update(value).frozen);
  value.stamp_sec = 1.1;
  const CargoFrozenGeometry result = fusion.update(value);
  ASSERT_TRUE(result.frozen) << result.reason;
  EXPECT_TRUE(result.formal_authorized);
  EXPECT_EQ(result.authorization, CargoGeometryAuthorization::FORMAL);
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
  next.dimension_observation_complete = false;
  next.observed_top_m = 3.75F;
  result = fusion.update(next);
  EXPECT_FLOAT_EQ(result.length_m, frozen_length);
  EXPECT_FLOAT_EQ(result.center.x(), 1.0F);
  EXPECT_EQ(result.track_segment_id, 2U);
  EXPECT_NEAR(result.bottom_m, 3.75F - result.height_m, 1.0e-5F);
}

TEST(CargoGeometryFusionTest,
     LargerMeasurementNeedsContinuousHighConfidenceConfirmation) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 1;
  config.conservative_expand_confirm_frames = 3;
  config.immediate_expand_enabled = false;
  config.minimum_physical_length_m = 0.30F;
  config.minimum_physical_width_m = 0.20F;
  config.formal_transition_start_length_m = 0.30F;
  config.formal_transition_start_width_m = 0.20F;
  CargoGeometryFusion fusion(config);
  auto result = fusion.update(frame(1.0));
  ASSERT_TRUE(result.frozen);
  auto larger = frame(1.1);
  larger.length_m = 4.5F;
  larger.width_m = 2.0F;
  larger.dimension_observation_complete = true;
  larger.dimension_support_points = 100U;
  larger.dimension_shape_confidence = 0.9F;
  const float original_length = result.length_m;
  const float original_width = result.width_m;
  result = fusion.update(larger);
  EXPECT_FLOAT_EQ(result.length_m, original_length);
  EXPECT_FLOAT_EQ(result.width_m, original_width);
  larger.stamp_sec = 1.2;
  result = fusion.update(larger);
  EXPECT_FLOAT_EQ(result.length_m, original_length);
  EXPECT_FLOAT_EQ(result.width_m, original_width);
  larger.stamp_sec = 1.3;
  result = fusion.update(larger);
  EXPECT_FLOAT_EQ(result.length_m, 4.5F);
  EXPECT_FLOAT_EQ(result.width_m, 2.0F);
}

TEST(CargoGeometryFusionTest,
     TrustedCompleteMeasurementExpandsDefaultEnvelopeImmediately) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 1;
  config.immediate_expand_enabled = true;
  CargoGeometryFusion fusion(config);
  auto result = fusion.update(frame(1.0));
  ASSERT_TRUE(result.frozen);
  auto larger = frame(1.1);
  larger.length_m = result.length_m + 0.40F;
  larger.width_m = result.width_m + 0.20F;
  larger.dimension_observation_complete = true;
  larger.dimension_support_points = 100U;
  larger.dimension_shape_confidence = 0.9F;
  result = fusion.update(larger);
  EXPECT_FLOAT_EQ(result.length_m, larger.length_m);
  EXPECT_FLOAT_EQ(result.width_m, larger.width_m);
}

TEST(CargoGeometryFusionTest,
     ImmediateExpansionStillRequiresExpansionConfidence) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 1;
  config.immediate_expand_enabled = true;
  CargoGeometryFusion fusion(config);
  auto result = fusion.update(frame(1.0));
  ASSERT_TRUE(result.frozen);
  const float frozen_length = result.length_m;
  const float frozen_width = result.width_m;

  auto weak_larger = frame(1.1);
  weak_larger.length_m += 0.40F;
  weak_larger.width_m += 0.20F;
  weak_larger.dimension_shape_confidence = 0.76F;
  result = fusion.update(weak_larger);

  EXPECT_FLOAT_EQ(result.length_m, frozen_length);
  EXPECT_FLOAT_EQ(result.width_m, frozen_width);
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

TEST(CargoGeometryFusionTest, ShrinkRequiresQualityEvidenceInRecentWindow) {
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
  auto partial = smaller;
  partial.stamp_sec = 1.3;
  partial.dimension_observation_complete = false;
  EXPECT_FLOAT_EQ(fusion.update(partial).length_m, original);
  smaller.stamp_sec = 1.4;
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

TEST(CargoGeometryFusionTest, TimeRollbackDropsPriorAuthorization) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 1;
  CargoGeometryFusion fusion(config);
  ASSERT_TRUE(fusion.update(frame(10.0)).formal_authorized);
  const CargoFrozenGeometry rolled_back = fusion.update(frame(9.0));
  EXPECT_FALSE(rolled_back.valid);
  EXPECT_FALSE(rolled_back.frozen);
  EXPECT_EQ(rolled_back.authorization,
            CargoGeometryAuthorization::PENDING);
}

TEST(CargoGeometryFusionTest,
     HighQualitySmallCargoCanLeaveFallbackFloorAfterConfirmation) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 2;
  config.conservative_shrink_confirm_frames = 1;
  config.formal_transition_start_length_m = 1.56F;
  config.formal_transition_start_width_m = 0.86F;
  CargoGeometryFusion fusion(config);
  auto small = frame(1.0);
  small.length_m = 1.5F;
  small.width_m = 0.8F;
  EXPECT_FALSE(fusion.update(small).frozen);
  small.stamp_sec = 1.1;
  auto result = fusion.update(small);
  ASSERT_TRUE(result.frozen);
  small.stamp_sec = 1.2;
  result = fusion.update(small);
  small.stamp_sec = 1.3;
  result = fusion.update(small);
  EXPECT_NEAR(result.length_m, 1.5F, 1.0e-6F);
  EXPECT_NEAR(result.width_m, 0.8F, 1.0e-6F);
  EXPECT_LT(result.length_m, 4.0F);
  EXPECT_LT(result.width_m, 3.0F);
}

TEST(CargoGeometryFusionTest, FormalGeometryUsesPhysicalFloorNotFallbackFloor) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 1;
  config.minimum_physical_length_m = 0.30F;
  config.minimum_physical_width_m = 0.20F;
  config.formal_transition_start_length_m = 0.30F;
  config.formal_transition_start_width_m = 0.20F;
  CargoGeometryFusion fusion(config);
  auto tiny = frame(1.0);
  tiny.length_m = 0.10F;
  tiny.width_m = 0.10F;
  const auto result = fusion.update(tiny);
  ASSERT_TRUE(result.frozen);
  EXPECT_FLOAT_EQ(result.length_m, 0.30F);
  EXPECT_FLOAT_EQ(result.width_m, 0.20F);
}

TEST(CargoGeometryFusionTest,
     DefaultFormalTransitionDoesNotReusePendingFourByThreeEnvelope) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 1;
  CargoGeometryFusion fusion(config);
  auto measured = frame(1.0);
  measured.length_m = 1.0F;
  measured.width_m = 0.5F;
  const auto result = fusion.update(measured);
  ASSERT_TRUE(result.frozen);
  EXPECT_FLOAT_EQ(result.length_m, 1.0F);
  EXPECT_FLOAT_EQ(result.width_m, 0.5F);
}

TEST(CargoGeometryFusionTest, AuthorizedStaticComponentCanRaiseFormalLowerBound) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 1;
  config.formal_transition_start_length_m = 0.30F;
  config.formal_transition_start_width_m = 0.20F;
  CargoGeometryFusion fusion(config);
  auto value = frame(1.0);
  value.length_m = 1.0F;
  value.width_m = 0.5F;
  value.static_length_lower_bound_m = 1.8F;
  value.static_width_lower_bound_m = 0.9F;
  const auto result = fusion.update(value);
  ASSERT_TRUE(result.frozen);
  EXPECT_FLOAT_EQ(result.length_m, 1.8F);
  EXPECT_FLOAT_EQ(result.width_m, 0.9F);
}

TEST(CargoGeometryFusionTest, UnverifiedStaticCannotRaiseFormalLowerBound) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 1;
  config.formal_transition_start_length_m = 0.30F;
  config.formal_transition_start_width_m = 0.20F;
  CargoGeometryFusion fusion(config);
  auto value = frame(1.0);
  value.length_m = 1.0F;
  value.width_m = 0.5F;
  // Runtime leaves lower bounds at zero for unverified static evidence.
  const auto result = fusion.update(value);
  ASSERT_TRUE(result.frozen);
  EXPECT_FLOAT_EQ(result.length_m, 1.0F);
  EXPECT_FLOAT_EQ(result.width_m, 0.5F);
}

TEST(CargoGeometryFusionTest, PartialSideCannotFreezeFormalEnvelope) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 1;
  CargoGeometryFusion fusion(config);
  auto partial = frame(1.0);
  partial.dimension_observation_complete = false;
  const auto result = fusion.update(partial);
  EXPECT_FALSE(result.frozen);
  EXPECT_EQ(result.reason, "formal_shape_confirmation_pending");
}

TEST(CargoGeometryFusionTest,
     OnePartialScanDoesNotEraseSameSegmentShapeEvidence) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 3;
  config.shape_confirmation_window_frames = 4;
  CargoGeometryFusion fusion(config);

  EXPECT_FALSE(fusion.update(frame(1.0)).frozen);
  auto partial = frame(1.1);
  partial.dimension_observation_complete = false;
  EXPECT_FALSE(fusion.update(partial).frozen);
  auto good = frame(1.2);
  EXPECT_FALSE(fusion.update(good).frozen);
  good.stamp_sec = 1.3;
  const auto result = fusion.update(good);
  EXPECT_TRUE(result.frozen) << result.reason;
  EXPECT_EQ(result.shape_confirm_frames, 3);
}

TEST(CargoGeometryFusionTest,
     ProvisionalTrackChangeCannotInheritGeometryWindow) {
  CargoGeometryFusionConfig config;
  config.minimum_confirm_frames = 3;
  config.shape_confirmation_window_frames = 4;
  CargoGeometryFusion fusion(config);
  EXPECT_FALSE(fusion.update(frame(1.0)).frozen);
  auto value = frame(1.1);
  EXPECT_FALSE(fusion.update(value).frozen);
  value.stamp_sec = 1.2;
  value.track_segment_id = 2U;
  const auto changed = fusion.update(value);
  EXPECT_FALSE(changed.frozen);
  EXPECT_EQ(changed.confirm_frames, 1);
  EXPECT_EQ(changed.shape_confirm_frames, 1);
}

}  // namespace
}  // namespace ndt_slam
