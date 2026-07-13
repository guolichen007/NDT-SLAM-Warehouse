#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <utility>

#include "ndt_slam/cargo_observation_policy.hpp"
#include "ndt_slam/registration_input_policy.hpp"

namespace ndt_slam {
namespace {

void addGroundCell(pcl::PointCloud<pcl::PointXYZ>& cloud,
                   float x, float y, float z) {
    cloud.push_back(pcl::PointXYZ(x, y, z));
    cloud.push_back(pcl::PointXYZ(x + 0.02F, y, z + 0.005F));
    cloud.push_back(pcl::PointXYZ(x, y + 0.02F, z - 0.005F));
}

TEST(CargoObservationPolicy, ExternalRingDoesNotUsePayloadBottomAsGround) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    // Cargo bottom inside the payload ROI.
    cloud.push_back(pcl::PointXYZ(0.0F, 0.0F, 1.00F));
    cloud.push_back(pcl::PointXYZ(0.2F, 0.2F, 1.05F));
    // Distributed floor cells outside the payload ROI.
    addGroundCell(cloud, 1.5F, 0.0F, 0.01F);
    addGroundCell(cloud, -1.5F, 0.0F, 0.00F);
    addGroundCell(cloud, 0.0F, 1.5F, 0.02F);
    addGroundCell(cloud, 0.0F, -1.5F, 0.01F);

    ExternalGroundConfig config;
    const auto estimate = estimateExternalGround(
        cloud, 0.0F, 0.0F, 1.0F, 1.0F, config);
    ASSERT_TRUE(estimate.valid);
    EXPECT_NEAR(estimate.z_m, 0.01F, 0.02F);
}

TEST(CargoObservationPolicy, UnstableExternalRingFailsClosed) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    addGroundCell(cloud, 1.5F, 0.0F, 0.0F);
    addGroundCell(cloud, -1.5F, 0.0F, 0.4F);
    addGroundCell(cloud, 0.0F, 1.5F, 0.8F);
    addGroundCell(cloud, 0.0F, -1.5F, 1.2F);
    ExternalGroundConfig config;
    const auto estimate = estimateExternalGround(
        cloud, 0.0F, 0.0F, 1.0F, 1.0F, config);
    EXPECT_FALSE(estimate.valid);
}

TEST(CargoObservationPolicy, OneSidedExternalRingFailsClosed) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    addGroundCell(cloud, 0.8F, 0.1F, 0.00F);
    addGroundCell(cloud, 1.3F, 0.3F, 0.01F);
    addGroundCell(cloud, 1.8F, -0.1F, 0.00F);
    addGroundCell(cloud, 2.3F, -0.3F, 0.01F);
    ExternalGroundConfig config;
    const auto estimate = estimateExternalGround(
        cloud, 0.0F, 0.0F, 1.0F, 1.0F, config);
    EXPECT_FALSE(estimate.valid);
}

TEST(CargoObservationPolicy, OppositeAisleSidesAreSufficient) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    addGroundCell(cloud, 1.4F, 0.1F, 0.00F);
    addGroundCell(cloud, 1.8F, -0.1F, 0.01F);
    addGroundCell(cloud, -1.4F, 0.1F, 0.00F);
    addGroundCell(cloud, -1.8F, -0.1F, 0.01F);
    ExternalGroundConfig config;
    const auto estimate = estimateExternalGround(
        cloud, 0.0F, 0.0F, 1.0F, 1.0F, config);
    EXPECT_TRUE(estimate.valid);
    EXPECT_TRUE(estimate.has_opposite_sides);
}

TEST(CargoObservationPolicy, IsolatedLowOutlierDoesNotBiasGround) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (const auto& xy : {std::pair<float, float>{1.5F, 0.0F},
                           {-1.5F, 0.0F}, {0.0F, 1.5F},
                           {0.0F, -1.5F}}) {
        cloud.push_back(pcl::PointXYZ(xy.first, xy.second, -2.0F));
        cloud.push_back(pcl::PointXYZ(xy.first + 0.02F, xy.second, 0.00F));
        cloud.push_back(pcl::PointXYZ(xy.first, xy.second + 0.02F, 0.01F));
    }
    ExternalGroundConfig config;
    const auto estimate = estimateExternalGround(
        cloud, 0.0F, 0.0F, 1.0F, 1.0F, config);
    ASSERT_TRUE(estimate.valid);
    EXPECT_NEAR(estimate.z_m, 0.0F, 0.02F);
}

TEST(CargoObservationPolicy, RaisedPlatformViolatesExpectedGroundHeight) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    addGroundCell(cloud, 1.5F, 0.0F, 0.80F);
    addGroundCell(cloud, -1.5F, 0.0F, 0.80F);
    addGroundCell(cloud, 0.0F, 1.5F, 0.80F);
    addGroundCell(cloud, 0.0F, -1.5F, 0.80F);
    ExternalGroundConfig config;
    config.expected_height_enabled = true;
    config.expected_height_m = 0.0F;
    config.maximum_expected_height_delta_m = 0.30F;
    const auto estimate = estimateExternalGround(
        cloud, 0.0F, 0.0F, 1.0F, 1.0F, config);
    EXPECT_FALSE(estimate.valid);
}

TEST(CargoObservationPolicy, MinimumFiniteZRejectsNonFiniteInput) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(0.0F, 0.0F,
                                  std::numeric_limits<float>::quiet_NaN()));
    EXPECT_FALSE(std::isfinite(minimumFiniteZ(cloud)));
    cloud.push_back(pcl::PointXYZ(0.0F, 0.0F, 0.75F));
    EXPECT_FLOAT_EQ(minimumFiniteZ(cloud), 0.75F);
}

TEST(RegistrationInputPolicy, HumanFilterRemovalIsNotResurrected) {
    pcl::PointCloud<pcl::PointXYZ> candidates;
    candidates.push_back(pcl::PointXYZ(1.0F, 2.0F, 3.0F));
    candidates.push_back(pcl::PointXYZ(4.0F, 5.0F, 6.0F));
    pcl::PointCloud<pcl::PointXYZ> human_safe;
    human_safe.push_back(pcl::PointXYZ(4.0F, 5.0F, 6.0F));
    human_safe.push_back(pcl::PointXYZ(8.0F, 9.0F, 10.0F));

    const auto partition = partitionRegistrationObjects(
        human_safe, candidates);
    ASSERT_EQ(partition.uncertain_candidates->size(), 1U);
    EXPECT_EQ(partition.static_objects->size(), 1U);
    EXPECT_EQ(partition.candidate_input_points, 2U);
    EXPECT_EQ(partition.candidate_survivor_points, 1U);
    EXPECT_EQ(partition.candidate_human_filtered_points, 1U);
}

TEST(RegistrationInputPolicy, DuplicateCandidateMultiplicityIsPreserved) {
    pcl::PointCloud<pcl::PointXYZ> candidates;
    candidates.push_back(pcl::PointXYZ(1.0F, 2.0F, 3.0F));
    candidates.push_back(pcl::PointXYZ(1.0F, 2.0F, 3.0F));
    pcl::PointCloud<pcl::PointXYZ> human_safe = candidates;

    const auto partition = partitionRegistrationObjects(
        human_safe, candidates);
    EXPECT_EQ(partition.uncertain_candidates->size(), 2U);
    EXPECT_TRUE(partition.static_objects->empty());
    EXPECT_EQ(partition.candidate_human_filtered_points, 0U);
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
