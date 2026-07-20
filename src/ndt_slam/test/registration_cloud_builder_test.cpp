#include <gtest/gtest.h>

#include "ndt_slam/registration_cloud_builder.hpp"

#include <cmath>

namespace ndt_slam {
namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

Cloud::Ptr makeWall(int horizontal_count,
                    int vertical_count,
                    float spacing_xy,
                    float spacing_z,
                    float x_offset = 0.0F,
                    float y_offset = 0.0F) {
    Cloud::Ptr cloud(new Cloud);
    for (int x = 0; x < horizontal_count; ++x) {
        for (int z = 0; z < vertical_count; ++z) {
            cloud->push_back(pcl::PointXYZ(
                x_offset + spacing_xy * static_cast<float>(x),
                y_offset, 0.20F + spacing_z * static_cast<float>(z)));
        }
    }
    return cloud;
}

Cloud::Ptr makeGrid(int x_count,
                    int y_count,
                    float spacing,
                    float z = 0.0F) {
    Cloud::Ptr cloud(new Cloud);
    for (int x = 0; x < x_count; ++x) {
        for (int y = 0; y < y_count; ++y) {
            cloud->push_back(pcl::PointXYZ(
                spacing * static_cast<float>(x),
                spacing * static_cast<float>(y), z));
        }
    }
    return cloud;
}

Cloud::Ptr makeUncertain(int count) {
    Cloud::Ptr cloud(new Cloud);
    for (int index = 0; index < count; ++index) {
        cloud->push_back(pcl::PointXYZ(
            20.0F + 0.31F * static_cast<float>(index % 50),
            5.0F + 0.31F * static_cast<float>(index / 50),
            0.8F + 0.31F * static_cast<float>(index % 3)));
    }
    return cloud;
}

RegistrationCloudBuildConfig testConfig() {
    RegistrationCloudBuildConfig config;
    config.min_structure_xy_cells = 30U;
    return config;
}

TEST(RegistrationCloudBuilderTest, StaticStructureRichNeedsNoGroundFallback) {
    Cloud::Ptr structure(new Cloud);
    *structure += *makeWall(120, 18, 0.12F, 0.15F, 0.0F, 0.0F);
    *structure += *makeWall(120, 18, 0.12F, 0.15F, 0.0F, 4.0F);
    const auto result = buildStructurePreservingRegistrationCloud(
        structure, Cloud::Ptr(new Cloud), makeGrid(40, 40, 0.5F),
        testConfig());

    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_EQ(result.mode, "STRUCTURE_RICH");
    EXPECT_TRUE(result.structure_quality_valid);
    EXPECT_GE(result.static_object_points, 600U);
    EXPECT_LE(result.total_points, 6000U);
    EXPECT_LE(result.ground_fraction, 0.35 + 1.0e-9);
}

TEST(RegistrationCloudBuilderTest, SmallerStaticVoxelRecoversSparseWall) {
    const Cloud::Ptr structure = makeWall(120, 8, 0.15F, 0.15F);
    const auto result = buildStructurePreservingRegistrationCloud(
        structure, makeUncertain(600), makeGrid(50, 50, 0.5F),
        testConfig());

    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_EQ(result.mode, "STRUCTURE_RECOVERY");
    EXPECT_GE(result.static_object_points, 600U);
    EXPECT_LE(result.ground_fraction, 0.35 + 1.0e-9);
}

TEST(RegistrationCloudBuilderTest, FullGroundWithoutStructureIsInvalid) {
    const auto result = buildStructurePreservingRegistrationCloud(
        Cloud::Ptr(new Cloud), Cloud::Ptr(new Cloud),
        makeGrid(100, 100, 0.25F), testConfig());

    EXPECT_FALSE(result.valid);
    EXPECT_FALSE(result.structure_quality_valid);
    EXPECT_EQ(result.mode, "INSUFFICIENT_STRUCTURE");
    EXPECT_EQ(result.total_points, 0U);
}

TEST(RegistrationCloudBuilderTest, GroundAugmentationStaysBelowFractionCap) {
    const Cloud::Ptr structure = makeWall(130, 10, 0.15F, 0.15F);
    const auto result = buildStructurePreservingRegistrationCloud(
        structure, makeUncertain(500), makeGrid(60, 60, 0.5F),
        testConfig());

    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_EQ(result.mode, "GROUND_AUGMENTED");
    EXPECT_GT(result.ground_points, 0U);
    EXPECT_LE(result.ground_fraction, 0.35 + 1.0e-9);
}

TEST(RegistrationCloudBuilderTest, RemovedHumanAndAuthorizedCargoNeverReturn) {
    Cloud::Ptr safe_structure(new Cloud);
    *safe_structure += *makeWall(120, 18, 0.12F, 0.15F, 0.0F, 0.0F);
    *safe_structure += *makeWall(120, 18, 0.12F, 0.15F, 0.0F, 4.0F);
    Cloud removed_human;
    removed_human.push_back(pcl::PointXYZ(100.0F, 100.0F, 1.0F));
    Cloud authorized_cargo;
    authorized_cargo.push_back(pcl::PointXYZ(200.0F, 200.0F, 1.0F));

    const auto result = buildStructurePreservingRegistrationCloud(
        safe_structure, Cloud::Ptr(new Cloud), makeGrid(40, 40, 0.5F),
        testConfig());
    ASSERT_TRUE(result.valid);
    for (const auto& point : result.cloud->points) {
        EXPECT_LT(point.x, 50.0F);
        EXPECT_LT(point.y, 50.0F);
    }
}

}  // namespace
}  // namespace ndt_slam
