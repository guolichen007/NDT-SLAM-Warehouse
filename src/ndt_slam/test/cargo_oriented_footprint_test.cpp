#include <gtest/gtest.h>

#include "ndt_slam/cargo_oriented_footprint.hpp"

#include <cmath>

namespace ndt_slam {
namespace {

std::vector<Eigen::Vector2f> rectangle(float length, float width, float yaw) {
    std::vector<Eigen::Vector2f> points;
    const float cosine = std::cos(yaw);
    const float sine = std::sin(yaw);
    for (int ix = -10; ix <= 10; ++ix) {
        for (int iy = -5; iy <= 5; ++iy) {
            const Eigen::Vector2f local(
                0.5F * length * static_cast<float>(ix) / 10.0F,
                0.5F * width * static_cast<float>(iy) / 5.0F);
            points.emplace_back(
                cosine * local.x() - sine * local.y(),
                sine * local.x() + cosine * local.y());
        }
    }
    return points;
}

float axialError(float lhs, float rhs) {
    return std::abs(normalizeCargoAxialYaw(lhs - rhs));
}

TEST(CargoOrientedFootprintTest, RecoversHorizontalAndVerticalAxes) {
    for (const float yaw : {0.0F, 0.5F * 3.14159265358979323846F}) {
        const auto result = estimateCargoOrientedFootprint(
            rectangle(2.0F, 0.8F, yaw), Eigen::Vector2f::Zero());
        ASSERT_TRUE(result.valid) << result.reason;
        EXPECT_NEAR(result.size_long_short.x(), 2.0F, 0.20F);
        EXPECT_NEAR(result.size_long_short.y(), 0.8F, 0.15F);
        EXPECT_LT(axialError(result.yaw_base_rad, yaw), 0.03F);
    }
}

TEST(CargoOrientedFootprintTest, RecoversRotatedFootprint) {
    constexpr float kYaw = 35.0F * 3.14159265358979323846F / 180.0F;
    const auto result = estimateCargoOrientedFootprint(
        rectangle(2.2F, 0.9F, kYaw), Eigen::Vector2f::Zero());
    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_LT(axialError(result.yaw_base_rad, kYaw), 0.03F);
    EXPECT_GT(result.axis_ratio, 2.0F);
}

TEST(CargoOrientedFootprintTest, PercentilesRejectSingleExtentOutlier) {
    auto points = rectangle(2.0F, 0.8F, 0.0F);
    points.emplace_back(20.0F, 20.0F);
    const auto result = estimateCargoOrientedFootprint(
        points, Eigen::Vector2f::Zero());
    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_LT(result.size_long_short.x(), 2.5F);
    EXPECT_LT(result.size_long_short.y(), 1.2F);
}

TEST(CargoOrientedFootprintTest, SquareDoesNotInventOrientation) {
    const auto result = estimateCargoOrientedFootprint(
        rectangle(1.0F, 1.0F, 0.4F), Eigen::Vector2f::Zero());
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.reason, "orientation_ambiguous");
}

TEST(CargoOrientedFootprintTest, AxialMeanHandlesNinetyDegreeWrap) {
    constexpr float kDeg = 3.14159265358979323846F / 180.0F;
    float mean = 0.0F;
    ASSERT_TRUE(meanCargoAxialYaw({89.0F * kDeg, -89.0F * kDeg}, &mean));
    EXPECT_LT(axialError(mean, 0.5F * 3.14159265358979323846F),
              2.0F * kDeg);
}

}  // namespace
}  // namespace ndt_slam
