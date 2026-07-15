#include <gtest/gtest.h>

#include "ndt_slam/cargo_marker_lifecycle.hpp"

namespace ndt_slam {
namespace {

CargoBoxGeometry geometry(float x = 1.0F) {
    CargoBoxGeometry result;
    result.valid = true;
    result.center_base = Eigen::Vector3f(x, 0.0F, 1.0F);
    result.center_map = result.center_base;
    for (std::size_t i = 0; i < result.corners_base.size(); ++i) {
        result.corners_base[i] =
            Eigen::Vector3f(x, static_cast<float>(i), 1.0F);
        result.corners_map[i] = result.corners_base[i];
    }
    return result;
}

TEST(CargoMarkerLifecycleTest, SingleHeightFailureKeepsLastGoodBox) {
    CargoMarkerLifecycle lifecycle;
    CargoMarkerLifecycleInput input;
    input.stamp_sec = 1.0;
    input.geometry_valid = true;
    input.safety_height_valid = true;
    input.geometry = geometry();
    EXPECT_EQ(lifecycle.update(input).style, CargoMarkerStyle::VALID);

    input.stamp_sec = 1.1;
    input.geometry_valid = false;
    input.safety_height_valid = false;
    const auto held = lifecycle.update(input);
    EXPECT_TRUE(held.show);
    EXPECT_TRUE(held.using_last_good_geometry);
    EXPECT_EQ(held.style, CargoMarkerStyle::HEIGHT_DEGRADED);
}

TEST(CargoMarkerLifecycleTest, SustainedInvalidityExpires) {
    CargoMarkerLifecycle lifecycle;
    CargoMarkerLifecycleInput input;
    input.stamp_sec = 1.0;
    input.geometry_valid = true;
    input.safety_height_valid = true;
    input.geometry = geometry();
    lifecycle.update(input);

    input.stamp_sec = 3.1;
    input.geometry_valid = false;
    EXPECT_FALSE(lifecycle.update(input).show);
}

TEST(CargoMarkerLifecycleTest, ExplicitEmptyDeletesImmediately) {
    CargoMarkerLifecycle lifecycle;
    CargoMarkerLifecycleInput input;
    input.stamp_sec = 1.0;
    input.geometry_valid = true;
    input.safety_height_valid = true;
    input.geometry = geometry();
    lifecycle.update(input);

    input.stamp_sec = 1.1;
    input.explicit_empty = true;
    input.geometry_valid = false;
    EXPECT_FALSE(lifecycle.update(input).show);
}

TEST(CargoMarkerLifecycleTest, LocalizationFailureFreezesGrayBox) {
    CargoMarkerLifecycle lifecycle;
    CargoMarkerLifecycleInput input;
    input.stamp_sec = 1.0;
    input.geometry_valid = true;
    input.safety_height_valid = true;
    input.geometry = geometry();
    lifecycle.update(input);

    input.stamp_sec = 1.2;
    input.geometry_valid = false;
    input.localization_valid = false;
    const auto held = lifecycle.update(input);
    EXPECT_TRUE(held.show);
    EXPECT_EQ(held.style, CargoMarkerStyle::LOCALIZATION_DEGRADED);
}

TEST(CargoMarkerLifecycleTest, SourceEpochRollbackDropsOldGeometry) {
    CargoMarkerLifecycle lifecycle;
    CargoMarkerLifecycleInput input;
    input.stamp_sec = 100.0;
    input.geometry_valid = true;
    input.safety_height_valid = true;
    input.geometry = geometry();
    lifecycle.update(input);

    input.stamp_sec = 1.0;
    input.geometry_valid = false;
    EXPECT_FALSE(lifecycle.update(input).show);
}

}  // namespace
}  // namespace ndt_slam
