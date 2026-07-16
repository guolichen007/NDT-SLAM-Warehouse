#include "ndt_slam/cargo_bottom_fusion.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace ndt_slam {
namespace {

std::vector<Eigen::Vector3f> boxPoints(float bottom,
                                       float top,
                                       int cells_x = 5,
                                       int cells_y = 4) {
    std::vector<Eigen::Vector3f> points;
    for (int ix = 0; ix < cells_x; ++ix) {
        for (int iy = 0; iy < cells_y; ++iy) {
            const float x = -0.4F + 0.2F * static_cast<float>(ix);
            const float y = -0.3F + 0.2F * static_cast<float>(iy);
            for (int iz = 0; iz <= 10; ++iz) {
                const float ratio = static_cast<float>(iz) / 10.0F;
                points.emplace_back(x, y, bottom + ratio * (top - bottom));
            }
        }
    }
    return points;
}

CargoBottomObservation observation(std::uint64_t track,
                                   double stamp,
                                   const std::vector<Eigen::Vector3f>& points) {
    CargoBottomObservation obs;
    obs.track_valid = true;
    obs.track_id = track;
    obs.stamp_sec = stamp;
    obs.transform_stamp_sec = stamp;
    obs.points_base = points;
    obs.footprint_valid = true;
    obs.footprint_center_base = Eigen::Vector2f::Zero();
    obs.footprint_size_xy = Eigen::Vector2f(1.2F, 1.0F);
    return obs;
}

TEST(CargoBottomFusion, RobustPercentileRejectsSingleLowOutlier) {
    CargoBottomFusion fusion;
    auto points = boxPoints(1.0F, 2.0F);
    points.emplace_back(0.0F, 0.0F, -5.0F);
    const CargoBottomResult result = fusion.update(observation(1, 1.0, points));
    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_EQ(result.source, CargoBottomSource::POINTS);
    EXPECT_GT(result.selected_stats.z02, 0.8F);
    EXPECT_NEAR(result.geometry.bottom_z_base, 1.0F, 0.08F);
    EXPECT_GE(result.selected_stats.bottom_band_xy_cells, 3U);
}

TEST(CargoBottomFusion, RotatedFootprintUsesCargoLocalSupportCoordinates) {
    constexpr float kYaw = 0.78539816339744830962F;
    const float cosine = std::cos(kYaw);
    const float sine = std::sin(kYaw);
    auto points = boxPoints(1.0F, 2.0F, 7, 4);
    for (auto& point : points) {
        const float x = point.x();
        const float y = point.y();
        point.x() = cosine * x - sine * y + 1.5F;
        point.y() = sine * x + cosine * y - 0.7F;
    }
    CargoBottomObservation rotated = observation(2, 1.0, points);
    rotated.footprint_center_base = Eigen::Vector2f(1.5F, -0.7F);
    rotated.footprint_size_xy = Eigen::Vector2f(1.6F, 1.0F);
    rotated.footprint_yaw_base_rad = kYaw;

    CargoBottomFusion fusion;
    const CargoBottomResult result = fusion.update(rotated);
    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_EQ(result.source, CargoBottomSource::POINTS);
    EXPECT_NEAR(result.geometry.center_base.x(), 1.5F, 0.08F);
    EXPECT_NEAR(result.geometry.center_base.y(), -0.7F, 0.08F);
    EXPECT_NEAR(result.geometry.bottom_z_base, 1.0F, 0.08F);
    EXPECT_GE(result.selected_stats.bottom_band_xy_cells, 3U);
}

TEST(CargoBottomFusion, RejectsBottomSupportInSingleCell) {
    CargoBottomFusionConfig config;
    config.points_min_bottom_band_xy_cells = 3;
    CargoBottomFusion fusion(config);
    std::vector<Eigen::Vector3f> points;
    for (int i = 0; i < 60; ++i) {
        points.emplace_back(0.01F, 0.01F, i < 30 ? 1.0F : 2.0F);
    }
    const CargoBottomResult result = fusion.update(observation(1, 1.0, points));
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.source, CargoBottomSource::INVALID);
    EXPECT_EQ(result.points_stats.reject_reason, "bottom_band_cells_too_few");
}

TEST(CargoBottomFusion, UsesMapDiffBeforeStaticWhenPointsUnsupported) {
    CargoBottomFusion fusion;
    CargoBottomObservation obs = observation(3, 2.0, {});
    obs.current_top_valid = true;
    obs.current_top_z_base = 1.8F;
    obs.map_diff_height_valid = true;
    obs.map_diff_height_m = 1.0F;
    obs.map_static_height_valid = true;
    obs.map_static_height_m = 1.3F;
    const CargoBottomResult result = fusion.update(obs);
    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_EQ(result.source, CargoBottomSource::MAP_DIFF);
    EXPECT_NEAR(result.geometry.bottom_z_base, 0.8F, 0.08F);
}

TEST(CargoBottomFusion, OriginHeightIsNotReportedAsMapEvidence) {
    CargoBottomFusion fusion;
    CargoBottomObservation obs = observation(31, 2.0, {});
    obs.current_top_valid = true;
    obs.current_top_z_base = 2.4F;
    obs.origin_height_valid = true;
    obs.origin_height_m = 1.2F;

    const CargoBottomResult result = fusion.update(obs);
    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_EQ(result.source, CargoBottomSource::ORIGIN_HEIGHT);
    EXPECT_STREQ(cargoBottomSourceName(result.source), "ORIGIN_HEIGHT");
    EXPECT_NEAR(result.geometry.bottom_z_base, 1.2F, 0.08F);
    EXPECT_FALSE(result.map_static_stats.valid);
    EXPECT_TRUE(result.origin_height_stats.valid);
}

TEST(CargoBottomFusion, PreLiftThicknessOverridesOccludedLowerEdge) {
    CargoBottomFusion fusion;
    CargoBottomObservation obs = observation(
        32, 2.0, boxPoints(0.20F, 1.20F));
    obs.current_top_valid = true;
    obs.current_top_z_base = 2.40F;
    obs.origin_height_valid = true;
    obs.origin_height_m = 0.60F;

    const CargoBottomResult result = fusion.update(obs);
    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_EQ(result.source, CargoBottomSource::ORIGIN_HEIGHT);
    EXPECT_EQ(result.reason, "pre_lift_frozen_height_prior");
    EXPECT_NEAR(result.geometry.top_z_base, 2.40F, 0.08F);
    EXPECT_NEAR(result.geometry.bottom_z_base, 1.80F, 0.08F);
    EXPECT_NEAR(result.height, 0.60F, 0.08F);
}

TEST(CargoBottomFusion, RejectsVisibleUpperPatchAsPhysicalBottom) {
    CargoBottomFusion fusion;
    CargoBottomObservation obs = observation(4, 2.0, {});
    for (int ix = 0; ix < 8; ++ix) {
        for (int iz = 0; iz < 8; ++iz) {
            // A narrow vertical face near the top has enough points and height,
            // but it does not laterally support the tracked footprint.
            obs.points_base.emplace_back(
                -0.35F + 0.10F * static_cast<float>(ix), 0.28F,
                1.50F + 0.05F * static_cast<float>(iz));
        }
    }
    const CargoBottomResult result = fusion.update(obs);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.points_stats.reject_reason,
              "bottom_band_lateral_span_too_small");
}

TEST(CargoBottomFusion, LargeCrossSourceJumpRequiresConfirmation) {
    CargoBottomFusionConfig config;
    config.large_jump_confirm_frames = 3;
    config.accumulation_window_sec = 0.05;
    CargoBottomFusion fusion(config);
    ASSERT_TRUE(fusion.update(observation(5, 1.0, boxPoints(1.0F, 2.0F))).valid);

    CargoBottomObservation prior = observation(5, 1.1, {});
    prior.current_top_valid = true;
    prior.current_top_z_base = 3.0F;
    prior.map_static_height_valid = true;
    prior.map_static_height_m = 1.0F;
    CargoBottomResult first = fusion.update(prior);
    ASSERT_TRUE(first.valid);
    EXPECT_NEAR(first.geometry.bottom_z_base, 1.0F, 0.10F);
    EXPECT_EQ(first.reason, "large_jump_confirmation_pending");
    prior.stamp_sec = prior.transform_stamp_sec = 1.2;
    CargoBottomResult second = fusion.update(prior);
    EXPECT_NEAR(second.geometry.bottom_z_base, 1.0F, 0.10F);
    prior.stamp_sec = prior.transform_stamp_sec = 1.3;
    CargoBottomResult third = fusion.update(prior);
    EXPECT_NEAR(third.geometry.bottom_z_base, 2.0F, 0.10F);
}

TEST(CargoBottomFusion, DuplicateStampDoesNotAdvanceLargeJumpConfirmation) {
    CargoBottomFusionConfig config;
    config.large_jump_confirm_frames = 3;
    config.accumulation_window_sec = 0.05;
    CargoBottomFusion fusion(config);
    ASSERT_TRUE(fusion.update(
        observation(51, 1.0, boxPoints(1.0F, 2.0F))).valid);

    CargoBottomObservation jump = observation(51, 1.1, {});
    jump.current_top_valid = true;
    jump.current_top_z_base = 3.0F;
    jump.map_static_height_valid = true;
    jump.map_static_height_m = 1.0F;

    const CargoBottomResult first = fusion.update(jump);
    ASSERT_TRUE(first.valid);
    EXPECT_EQ(first.reason, "large_jump_confirmation_pending");
    EXPECT_NEAR(first.geometry.bottom_z_base, 1.0F, 0.10F);

    for (int replay = 0; replay < 5; ++replay) {
        const CargoBottomResult duplicate = fusion.update(jump);
        ASSERT_TRUE(duplicate.valid);
        EXPECT_EQ(duplicate.reason, "large_jump_confirmation_pending");
        EXPECT_NEAR(duplicate.geometry.bottom_z_base, 1.0F, 0.10F);
        EXPECT_FLOAT_EQ(duplicate.confidence, first.confidence);
    }

    jump.stamp_sec = jump.transform_stamp_sec = 1.2;
    const CargoBottomResult second_fresh = fusion.update(jump);
    ASSERT_TRUE(second_fresh.valid);
    EXPECT_EQ(second_fresh.reason, "large_jump_confirmation_pending");
    EXPECT_NEAR(second_fresh.geometry.bottom_z_base, 1.0F, 0.10F);

    for (int replay = 0; replay < 3; ++replay) {
        const CargoBottomResult duplicate = fusion.update(jump);
        ASSERT_TRUE(duplicate.valid);
        EXPECT_EQ(duplicate.reason, "large_jump_confirmation_pending");
        EXPECT_NEAR(duplicate.geometry.bottom_z_base, 1.0F, 0.10F);
    }

    jump.stamp_sec = jump.transform_stamp_sec = 1.3;
    const CargoBottomResult third_fresh = fusion.update(jump);
    ASSERT_TRUE(third_fresh.valid);
    EXPECT_NEAR(third_fresh.geometry.bottom_z_base, 2.0F, 0.10F);
}

TEST(CargoBottomFusion, TopOrHeightJumpAlsoRequiresConfirmation) {
    CargoBottomFusionConfig config;
    config.accumulation_window_sec = 0.05;
    config.large_jump_confirm_frames = 2;
    CargoBottomFusion fusion(config);
    ASSERT_TRUE(fusion.update(observation(6, 1.0, boxPoints(1.0F, 2.0F))).valid);

    CargoBottomObservation taller = observation(6, 1.1, {});
    taller.current_top_valid = true;
    taller.current_top_z_base = 3.0F;
    taller.map_static_height_valid = true;
    taller.map_static_height_m = 2.0F;  // same bottom, top/height jumps 1 m
    const CargoBottomResult held = fusion.update(taller);
    ASSERT_TRUE(held.valid);
    EXPECT_NEAR(held.geometry.top_z_base, 2.0F, 0.10F);
    EXPECT_EQ(held.reason, "large_jump_confirmation_pending");

    taller.stamp_sec = taller.transform_stamp_sec = 1.2;
    const CargoBottomResult accepted = fusion.update(taller);
    EXPECT_NEAR(accepted.geometry.top_z_base, 3.0F, 0.10F);
}

TEST(CargoBottomFusion, RecentStableExpiresAndNeverLeaksAcrossTrack) {
    CargoBottomFusionConfig config;
    config.accumulation_window_sec = 0.10;
    config.stable_hold_sec = 0.50;
    CargoBottomFusion fusion(config);
    const CargoBottomResult first = fusion.update(
        observation(7, 1.0, boxPoints(1.0F, 2.0F)));
    ASSERT_TRUE(first.valid);

    CargoBottomResult held = fusion.update(observation(7, 1.2, {}));
    ASSERT_TRUE(held.valid) << held.reason;
    EXPECT_EQ(held.source, CargoBottomSource::RECENT_STABLE);
    EXPECT_DOUBLE_EQ(held.evidence_stamp_sec, first.evidence_stamp_sec);
    EXPECT_GT(held.uncertainty, config.recent_stable_uncertainty_min);

    CargoBottomResult expired = fusion.update(observation(7, 1.7, {}));
    EXPECT_FALSE(expired.valid);

    ASSERT_TRUE(fusion.update(observation(7, 2.0, boxPoints(1.0F, 2.0F))).valid);
    CargoBottomResult new_track = fusion.update(observation(8, 2.1, {}));
    EXPECT_FALSE(new_track.valid);
    EXPECT_EQ(new_track.source, CargoBottomSource::INVALID);
}

TEST(CargoBottomFusion, RollbackFrameIsInvalid) {
    CargoBottomFusion fusion;
    ASSERT_TRUE(fusion.update(observation(9, 5.0, boxPoints(1.0F, 2.0F))).valid);
    CargoBottomResult rollback =
        fusion.update(observation(9, 4.0, boxPoints(1.0F, 2.0F)));
    EXPECT_FALSE(rollback.valid);
    EXPECT_EQ(rollback.reason, "time_rollback_reset");
    EXPECT_EQ(fusion.accumulatedPointCount(), 0U);
}

TEST(CargoBottomFusion, NextEpochCanRecoverAfterRollback) {
    CargoBottomFusion fusion;
    ASSERT_TRUE(fusion.update(observation(10, 5.0, boxPoints(1.0F, 2.0F))).valid);
    ASSERT_FALSE(
        fusion.update(observation(10, 1.0, boxPoints(1.0F, 2.0F))).valid);

    const CargoBottomResult recovered =
        fusion.update(observation(10, 1.1, boxPoints(1.0F, 2.0F)));
    ASSERT_TRUE(recovered.valid) << recovered.reason;
    EXPECT_EQ(recovered.source, CargoBottomSource::POINTS);
    EXPECT_GT(fusion.accumulatedPointCount(), 0U);
}

TEST(CargoBottomFusion, StaleGapFrameIsInvalid) {
    CargoBottomFusion fusion;
    ASSERT_TRUE(fusion.update(observation(12, 1.0, boxPoints(1.0F, 2.0F))).valid);

    const CargoBottomResult stale =
        fusion.update(observation(12, 2.0, boxPoints(1.0F, 2.0F)));
    EXPECT_FALSE(stale.valid);
    EXPECT_EQ(stale.reason, "stale_gap_reset");
    EXPECT_EQ(fusion.accumulatedPointCount(), 0U);
}

TEST(CargoBottomFusion, NextFrameCanRecoverAfterStaleGap) {
    CargoBottomFusion fusion;
    ASSERT_TRUE(fusion.update(observation(13, 1.0, boxPoints(1.0F, 2.0F))).valid);
    ASSERT_FALSE(
        fusion.update(observation(13, 2.0, boxPoints(1.0F, 2.0F))).valid);

    const CargoBottomResult recovered =
        fusion.update(observation(13, 2.1, boxPoints(1.0F, 2.0F)));
    ASSERT_TRUE(recovered.valid) << recovered.reason;
    EXPECT_EQ(recovered.source, CargoBottomSource::POINTS);
}

TEST(CargoBottomFusion, TrackCenterTranslationCompensatesAccumulatedPoints) {
    CargoBottomFusionConfig config;
    config.stable_hold_sec = 0.0;
    CargoBottomFusion fusion(config);
    CargoBottomObservation first = observation(14, 1.0, boxPoints(1.0F, 2.0F));
    first.track_center_valid = true;
    first.track_center_base = Eigen::Vector3f(0.0F, 0.0F, 1.5F);
    ASSERT_TRUE(fusion.update(first).valid);

    CargoBottomObservation moved = observation(14, 1.1, {});
    moved.track_center_valid = true;
    moved.track_center_base = Eigen::Vector3f(2.0F, 0.0F, 1.5F);
    moved.footprint_center_base = Eigen::Vector2f(2.0F, 0.0F);
    const CargoBottomResult result = fusion.update(moved);

    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_EQ(result.source, CargoBottomSource::POINTS);
    EXPECT_EQ(result.reason, "accumulated_points_supported");
    EXPECT_GE(result.selected_stats.finite_points, config.points_min_points);
    EXPECT_NEAR(result.geometry.center_base.x(), 2.0F, 0.08F);
}

TEST(CargoBottomFusion, TrackCenterHoistCompensatesAccumulatedPoints) {
    CargoBottomFusionConfig config;
    config.stable_hold_sec = 0.0;
    CargoBottomFusion fusion(config);
    CargoBottomObservation first = observation(15, 1.0, boxPoints(1.0F, 2.0F));
    first.track_center_valid = true;
    first.track_center_base = Eigen::Vector3f(0.0F, 0.0F, 1.5F);
    ASSERT_TRUE(fusion.update(first).valid);

    CargoBottomObservation hoisted = observation(15, 1.1, {});
    hoisted.track_center_valid = true;
    hoisted.track_center_base = Eigen::Vector3f(0.0F, 0.0F, 2.5F);
    const CargoBottomResult result = fusion.update(hoisted);

    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_EQ(result.source, CargoBottomSource::POINTS);
    EXPECT_EQ(result.reason, "accumulated_points_supported");
    EXPECT_NEAR(result.selected_stats.z05, 2.0F, 0.08F);
    EXPECT_NEAR(result.geometry.bottom_z_base, 2.0F, 0.08F);
    EXPECT_NEAR(result.geometry.top_z_base, 3.0F, 0.08F);
}

TEST(CargoBottomFusion, AccumulatedPointCountNeverExceedsMaximum) {
    CargoBottomFusionConfig config;
    config.accumulation_window_sec = 10.0;
    config.max_accumulated_points = 300U;
    CargoBottomFusion fusion(config);

    for (int frame = 0; frame < 5; ++frame) {
        const CargoBottomResult result = fusion.update(observation(
            16, 1.0 + 0.1 * static_cast<double>(frame),
            boxPoints(1.0F, 2.0F)));
        ASSERT_TRUE(result.valid) << result.reason;
        EXPECT_LE(fusion.accumulatedPointCount(),
                  config.max_accumulated_points);
    }
}

TEST(CargoBottomFusion,
     OversizedSingleFrameBoundsHistoryButUsesCompleteCurrentFrame) {
    CargoBottomFusionConfig config;
    config.max_accumulated_points = 25U;
    CargoBottomFusion fusion(config);

    const std::vector<Eigen::Vector3f> points =
        boxPoints(1.0F, 2.0F);

    const CargoBottomResult result =
        fusion.update(observation(17, 1.0, points));

    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_EQ(result.source, CargoBottomSource::POINTS);
    EXPECT_EQ(result.reason, "current_points_supported");

    EXPECT_EQ(fusion.accumulatedPointCount(), 25U);
    EXPECT_EQ(result.accumulated_points, 25U);

    EXPECT_EQ(result.points_stats.finite_points, points.size());
    EXPECT_GT(result.points_stats.finite_points,
              config.max_accumulated_points);

    EXPECT_NEAR(result.geometry.bottom_z_base, 1.0F, 0.08F);
    EXPECT_NEAR(result.geometry.top_z_base, 2.0F, 0.08F);
}

TEST(CargoBottomFusion,
     RobustVerticalContinuityRejectsExtremeTailOutliers) {
    CargoBottomFusion fusion;

    auto points = boxPoints(1.0F, 2.0F);
    points.emplace_back(0.0F, 0.0F, -5.0F);
    points.emplace_back(0.0F, 0.0F, 8.0F);

    const CargoBottomResult result =
        fusion.update(observation(101, 1.0, points));

    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_EQ(result.source, CargoBottomSource::POINTS);
    EXPECT_NEAR(result.geometry.bottom_z_base, 1.0F, 0.08F);
    EXPECT_NEAR(result.geometry.top_z_base, 2.0F, 0.08F);
    EXPECT_LE(result.selected_stats.max_vertical_gap,
              fusion.config().points_max_vertical_gap);
}

TEST(CargoBottomFusion,
     RobustVerticalContinuityStillRejectsInteriorGap) {
    CargoBottomFusion fusion;
    std::vector<Eigen::Vector3f> points;

    for (int ix = 0; ix < 5; ++ix) {
        for (int iy = 0; iy < 4; ++iy) {
            const float x =
                -0.4F + 0.2F * static_cast<float>(ix);
            const float y =
                -0.3F + 0.2F * static_cast<float>(iy);

            points.emplace_back(x, y, 1.0F);
            points.emplace_back(x, y, 1.1F);
            points.emplace_back(x, y, 1.8F);
            points.emplace_back(x, y, 1.9F);
        }
    }

    const CargoBottomResult result =
        fusion.update(observation(102, 1.0, points));

    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.points_stats.reject_reason,
              "vertical_continuity_gap");
    EXPECT_GT(result.points_stats.max_vertical_gap,
              fusion.config().points_max_vertical_gap);
}

TEST(CargoBottomFusion, DuplicateStampDoesNotIncreaseAccumulation) {
    CargoBottomFusion fusion;
    const CargoBottomObservation frame =
        observation(18, 1.0, boxPoints(1.0F, 2.0F));
    const CargoBottomResult first = fusion.update(frame);
    ASSERT_TRUE(first.valid) << first.reason;
    const std::size_t first_count = fusion.accumulatedPointCount();

    const CargoBottomResult duplicate = fusion.update(frame);
    ASSERT_TRUE(duplicate.valid) << duplicate.reason;
    EXPECT_EQ(fusion.accumulatedPointCount(), first_count);
    EXPECT_EQ(duplicate.accumulated_points, first.accumulated_points);
    EXPECT_EQ(duplicate.selected_stats.finite_points,
              first.selected_stats.finite_points);
}

TEST(CargoBottomFusion, ResultCarriesObservationTrackId) {
    CargoBottomFusion fusion;
    const CargoBottomResult result = fusion.update(observation(987654U, 1.0, {}));
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.track_id, 987654U);
}

TEST(CargoBottomFusion, SetConfigClearsTemporalEvidence) {
    CargoBottomFusionConfig config;
    CargoBottomFusion fusion(config);
    ASSERT_TRUE(fusion.update(
        observation(19, 1.0, boxPoints(1.0F, 2.0F))).valid);
    ASSERT_TRUE(fusion.hasTrack());
    ASSERT_GT(fusion.accumulatedPointCount(), 0U);

    config.points_confidence_base = 0.75F;
    fusion.setConfig(config);
    EXPECT_FALSE(fusion.hasTrack());
    EXPECT_EQ(fusion.trackId(), 0U);
    EXPECT_EQ(fusion.accumulatedPointCount(), 0U);

    const CargoBottomResult empty = fusion.update(observation(19, 1.1, {}));
    EXPECT_FALSE(empty.valid);
    EXPECT_EQ(empty.source, CargoBottomSource::INVALID);
}

TEST(CargoBottomFusion, InvalidMapSupportConfigIsRejected) {
    const auto expect_invalid = [](const CargoBottomFusionConfig& config,
                                   const char* expected_reason) {
        CargoBottomFusion fusion(config);
        const CargoBottomResult result =
            fusion.update(observation(20, 1.0, boxPoints(1.0F, 2.0F)));
        EXPECT_FALSE(result.valid);
        EXPECT_EQ(result.reason, expected_reason);
        EXPECT_FALSE(fusion.hasTrack());
        EXPECT_EQ(fusion.accumulatedPointCount(), 0U);
    };

    CargoBottomFusionConfig config;
    config.map_diff_min_points = 0U;
    expect_invalid(config, "invalid_config:map_minimum_support");
    config = CargoBottomFusionConfig{};
    config.map_static_min_points = 0U;
    expect_invalid(config, "invalid_config:map_minimum_support");
    config = CargoBottomFusionConfig{};
    config.map_min_visible_height = -0.1F;
    expect_invalid(config, "invalid_config:map_minimum_support");
    config = CargoBottomFusionConfig{};
    config.map_min_bottom_band_points = 0U;
    expect_invalid(config, "invalid_config:map_minimum_support");
    config = CargoBottomFusionConfig{};
    config.map_min_bottom_band_xy_cells = 0U;
    expect_invalid(config, "invalid_config:map_minimum_support");
    config = CargoBottomFusionConfig{};
    config.map_min_bottom_band_point_ratio = 1.1F;
    expect_invalid(config, "invalid_config:map_support_ratio");
    config = CargoBottomFusionConfig{};
    config.map_min_bottom_band_xy_cell_ratio = -0.1F;
    expect_invalid(config, "invalid_config:map_support_ratio");
}

TEST(CargoBottomFusion, TrackCenterLossFailsClosed) {
    CargoBottomFusion fusion;
    CargoBottomObservation first = observation(21, 1.0, boxPoints(1.0F, 2.0F));
    first.track_center_valid = true;
    first.track_center_base = Eigen::Vector3f(0.0F, 0.0F, 1.5F);
    ASSERT_TRUE(fusion.update(first).valid);

    const CargoBottomResult lost =
        fusion.update(observation(21, 1.1, boxPoints(1.0F, 2.0F)));
    EXPECT_FALSE(lost.valid);
    EXPECT_EQ(lost.reason, "track_center_lost");
    EXPECT_EQ(fusion.accumulatedPointCount(), 0U);
}

TEST(CargoBottomFusion, MapCornersUseSameStampTransform) {
    CargoBottomObservation obs = observation(11, 3.0, boxPoints(1.0F, 2.0F));
    obs.T_map_base = Eigen::Translation3f(10.0F, 20.0F, 3.0F) *
        Eigen::AngleAxisf(1.57079632679F, Eigen::Vector3f::UnitZ());
    CargoBottomFusion fusion;
    const CargoBottomResult result = fusion.update(obs);
    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_NEAR(result.geometry.center_map.x(), 10.0F, 0.05F);
    EXPECT_NEAR(result.geometry.center_map.y(), 20.0F, 0.05F);
    EXPECT_NEAR(result.geometry.bottom_z_map, 4.0F, 0.08F);
    EXPECT_NEAR(result.geometry.top_z_map, 5.0F, 0.08F);
    for (const auto& corner : result.geometry.corners_map) {
        EXPECT_TRUE(corner.allFinite());
    }
}

}  // namespace
}  // namespace ndt_slam
