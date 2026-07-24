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
            rectangle(2.0F, 0.8F, yaw));
        ASSERT_TRUE(result.valid) << result.reason;
        EXPECT_NEAR(result.size_long_short.x(), 2.0F, 0.20F);
        EXPECT_NEAR(result.size_long_short.y(), 0.8F, 0.15F);
        EXPECT_LT(axialError(result.yaw_base_rad, yaw), 0.03F);
    }
}

TEST(CargoOrientedFootprintTest, RecoversRotatedFootprint) {
    constexpr float kYaw = 35.0F * 3.14159265358979323846F / 180.0F;
    const auto result = estimateCargoOrientedFootprint(
        rectangle(2.2F, 0.9F, kYaw));
    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_LT(axialError(result.yaw_base_rad, kYaw), 0.03F);
    EXPECT_GT(result.eigenvalue_ratio, 2.0F);
}

TEST(CargoOrientedFootprintTest, PercentilesRejectSingleExtentOutlier) {
    auto points = rectangle(2.0F, 0.8F, 0.0F);
    points.emplace_back(20.0F, 20.0F);
    const auto result = estimateCargoOrientedFootprint(
        points);
    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_LT(result.size_long_short.x(), 2.5F);
    EXPECT_LT(result.size_long_short.y(), 1.2F);
}

TEST(CargoOrientedFootprintTest, ReportsDimensionClamping) {
    CargoOrientedFootprintConfig config;
    config.maximum_long_side_m = 3.0F;
    const auto result = estimateCargoOrientedFootprint(
        rectangle(4.0F, 0.8F, 0.0F), config);
    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_TRUE(result.long_side_clamped);
    EXPECT_GT(result.raw_size_long_short.x(), 3.0F);
    EXPECT_FLOAT_EQ(result.size_long_short.x(), 3.0F);
}

TEST(CargoOrientedFootprintTest, SquareDoesNotInventOrientation) {
    const auto result = estimateCargoOrientedFootprint(
        rectangle(1.0F, 1.0F, 0.4F));
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.reason, "orientation_ambiguous");
}

TEST(CargoOrientedFootprintTest, RecoversTranslatedCenter) {
    auto points = rectangle(2.0F, 0.8F, 0.3F);
    for (auto& point : points) point += Eigen::Vector2f(0.45F, -0.25F);
    const auto result = estimateCargoOrientedFootprint(points);
    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_NEAR(result.center_base.x(), 0.45F, 0.03F);
    EXPECT_NEAR(result.center_base.y(), -0.25F, 0.03F);
}

TEST(CargoOrientedFootprintTest, RejectsFivePercentAspectRatio) {
    CargoOrientedFootprintConfig config;
    config.minimum_eigenvalue_ratio = 1.0F;
    const auto result = estimateCargoOrientedFootprint(
        rectangle(1.05F, 1.0F, 0.7F), config);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.reason, "geometric_aspect_ambiguous");
}

TEST(CargoOrientedFootprintTest, AxialSummaryRejectsConflictingFrames) {
    constexpr float kDeg = 3.14159265358979323846F / 180.0F;
    const CargoAxialYawSummary coherent = summarizeCargoAxialYaw(
        {29.0F * kDeg, 30.0F * kDeg, 31.0F * kDeg});
    ASSERT_TRUE(coherent.valid);
    EXPECT_GT(coherent.concentration, 0.99F);
    EXPECT_LT(coherent.maximum_deviation_rad, 2.0F * kDeg);

    const CargoAxialYawSummary conflict = summarizeCargoAxialYaw(
        {0.0F, 45.0F * kDeg, 89.0F * kDeg});
    ASSERT_TRUE(conflict.valid);
    EXPECT_LT(conflict.concentration, 0.40F);
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
