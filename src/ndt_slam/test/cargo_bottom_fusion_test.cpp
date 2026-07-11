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
    ASSERT_TRUE(fusion.update(observation(7, 1.0, boxPoints(1.0F, 2.0F))).valid);

    CargoBottomResult held = fusion.update(observation(7, 1.2, {}));
    ASSERT_TRUE(held.valid) << held.reason;
    EXPECT_EQ(held.source, CargoBottomSource::RECENT_STABLE);
    EXPECT_GT(held.uncertainty, config.recent_stable_uncertainty_min);

    CargoBottomResult expired = fusion.update(observation(7, 1.7, {}));
    EXPECT_FALSE(expired.valid);

    ASSERT_TRUE(fusion.update(observation(7, 2.0, boxPoints(1.0F, 2.0F))).valid);
    CargoBottomResult new_track = fusion.update(observation(8, 2.1, {}));
    EXPECT_FALSE(new_track.valid);
    EXPECT_EQ(new_track.source, CargoBottomSource::INVALID);
}

TEST(CargoBottomFusion, BackwardTimeClearsTemporalEvidence) {
    CargoBottomFusion fusion;
    ASSERT_TRUE(fusion.update(observation(9, 5.0, boxPoints(1.0F, 2.0F))).valid);
    CargoBottomResult rollback = fusion.update(observation(9, 4.0, {}));
    EXPECT_FALSE(rollback.valid);
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
