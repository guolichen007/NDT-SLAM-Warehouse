#include <gtest/gtest.h>

#include "ndt_slam/cargo_observation_policy.hpp"

namespace ndt_slam {
namespace {

TEST(CargoObservationPolicy, ExternalRingDoesNotUsePayloadBottomAsGround) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    // Cargo bottom inside the payload ROI.
    cloud.push_back(pcl::PointXYZ(0.0F, 0.0F, 1.00F));
    cloud.push_back(pcl::PointXYZ(0.2F, 0.2F, 1.05F));
    // Distributed floor cells outside the payload ROI.
    cloud.push_back(pcl::PointXYZ(1.5F, 0.0F, 0.01F));
    cloud.push_back(pcl::PointXYZ(-1.5F, 0.0F, 0.00F));
    cloud.push_back(pcl::PointXYZ(0.0F, 1.5F, 0.02F));
    cloud.push_back(pcl::PointXYZ(0.0F, -1.5F, 0.01F));

    ExternalGroundConfig config;
    const auto estimate = estimateExternalGround(
        cloud, 0.0F, 0.0F, 1.0F, 1.0F, config);
    ASSERT_TRUE(estimate.valid);
    EXPECT_NEAR(estimate.z_m, 0.01F, 0.02F);
}

TEST(CargoObservationPolicy, UnstableExternalRingFailsClosed) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(1.5F, 0.0F, 0.0F));
    cloud.push_back(pcl::PointXYZ(-1.5F, 0.0F, 0.4F));
    cloud.push_back(pcl::PointXYZ(0.0F, 1.5F, 0.8F));
    cloud.push_back(pcl::PointXYZ(0.0F, -1.5F, 1.2F));
    ExternalGroundConfig config;
    const auto estimate = estimateExternalGround(
        cloud, 0.0F, 0.0F, 1.0F, 1.0F, config);
    EXPECT_FALSE(estimate.valid);
}

TEST(CargoObservationPolicy, CompactCargoRequiresLoadedHook) {
    const auto unloaded = classifyCargoLockProfile(
        45U, 0.22F, 0.15F, false,
        80, 0.50F, 0.40F, true, 40, 0.18F, 0.12F);
    EXPECT_EQ(unloaded, CargoLockProfile::NONE);

    const auto loaded = classifyCargoLockProfile(
        45U, 0.22F, 0.15F, true,
        80, 0.50F, 0.40F, true, 40, 0.18F, 0.12F);
    EXPECT_EQ(loaded, CargoLockProfile::COMPACT_BODY);
}

TEST(CargoObservationPolicy, LargeCargoDoesNotDependOnCompactMode) {
    const auto profile = classifyCargoLockProfile(
        90U, 0.60F, 0.50F, false,
        80, 0.50F, 0.40F, false, 40, 0.18F, 0.12F);
    EXPECT_EQ(profile, CargoLockProfile::LARGE_BODY);
}

}  // namespace
}  // namespace ndt_slam
