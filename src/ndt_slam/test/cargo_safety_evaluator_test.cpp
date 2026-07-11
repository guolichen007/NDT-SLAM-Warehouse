#include "ndt_slam/cargo_safety_evaluator.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

CargoSafetyInput baseInput() {
    CargoSafetyInput input;
    input.height.valid = true;
    input.height.stale = false;
    input.height.bottom_z = 2.0F;
    input.height.bottom_uncertainty_m = 0.10F;
    input.height.stamp_sec = 10.0;
    input.evaluation_time_sec = 10.0;
    input.footprint_base.min_x = -0.5F;
    input.footprint_base.max_x = 0.5F;
    input.footprint_base.min_y = -0.5F;
    input.footprint_base.max_y = 0.5F;
    input.footprint_base.min_z = 1.9F;
    input.footprint_base.max_z = 3.0F;
    input.obstacle_cloud_base.reset(new pcl::PointCloud<pcl::PointXYZ>);
    input.obstacle_observation_valid = true;
    input.obstacle_cloud_age_sec = 0.0;
    input.obstacle_roi_finite_points = 100;
    input.obstacle_roi_coverage_ratio = 0.80F;
    return input;
}

void addCluster(CargoSafetyInput* input,
                float footprint_distance,
                float top_z,
                float y = 0.0F) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (input->obstacle_cloud_base) {
        *cloud = *input->obstacle_cloud_base;
    }
    const float x = 0.5F + footprint_distance;
    for (int i = 0; i < 8; ++i) {
        cloud->push_back(pcl::PointXYZ(
            x + 0.005F * static_cast<float>(i),
            y + 0.004F * static_cast<float>(i % 3),
            top_z - 0.01F * static_cast<float>(i % 2)));
    }
    input->obstacle_cloud_base = cloud;
}

TEST(CargoSafetyEvaluator, DistanceBoundariesUseSingleClusterEvidence) {
    CargoSafetyEvaluator evaluator;
    struct Case { float distance; std::uint16_t expected; };
    const Case cases[] = {
        {2.9F, CargoSafetyEvaluator::kLevel1Code},
        {3.1F, CargoSafetyEvaluator::kLevel2OrFailSafeCode},
        {4.9F, CargoSafetyEvaluator::kLevel2OrFailSafeCode},
        {5.1F, CargoSafetyEvaluator::kSafeCode},
    };
    for (const auto& test_case : cases) {
        CargoSafetyInput input = baseInput();
        addCluster(&input, test_case.distance, 1.10F);
        const CargoSafetyResult result = evaluator.evaluate(input);
        ASSERT_TRUE(result.input_valid) << result.reason;
        EXPECT_EQ(result.raw_code, test_case.expected) << test_case.distance;
    }
}

TEST(CargoSafetyEvaluator, ClearanceBoundaryIsConservative) {
    CargoSafetyEvaluator evaluator;
    CargoSafetyInput exactly_safe = baseInput();
    // conservative bottom=1.85, obstacle safe top=1.00+0.05 => clearance=0.80
    addCluster(&exactly_safe, 2.0F, 1.00F);
    CargoSafetyResult safe = evaluator.evaluate(exactly_safe);
    ASSERT_TRUE(safe.has_cluster_evidence);
    EXPECT_NEAR(safe.most_dangerous_cluster.conservative_clearance_m, 0.80F, 0.02F);
    EXPECT_EQ(safe.raw_code, CargoSafetyEvaluator::kSafeCode);

    CargoSafetyInput unsafe = baseInput();
    addCluster(&unsafe, 2.0F, 1.02F);
    CargoSafetyResult hazard = evaluator.evaluate(unsafe);
    ASSERT_TRUE(hazard.has_cluster_evidence);
    EXPECT_LT(hazard.most_dangerous_cluster.conservative_clearance_m, 0.80F);
    EXPECT_EQ(hazard.raw_code, CargoSafetyEvaluator::kLevel1Code);
}

TEST(CargoSafetyEvaluator, NeverCombinesDistanceAndHeightAcrossClusters) {
    CargoSafetyInput input = baseInput();
    // A close but vertically safe cluster.
    addCluster(&input, 1.0F, 0.20F, -1.0F);
    // A farther, high cluster that independently produces level 2.
    addCluster(&input, 4.0F, 1.30F, 1.0F);

    CargoSafetyEvaluator evaluator;
    const CargoSafetyResult result = evaluator.evaluate(input);
    ASSERT_EQ(result.evaluated_cluster_count, 2U);
    ASSERT_TRUE(result.has_cluster_evidence);
    EXPECT_EQ(result.raw_code, CargoSafetyEvaluator::kLevel2OrFailSafeCode);
    EXPECT_GT(result.most_dangerous_cluster.footprint_distance_m, 3.0F);
    EXPECT_GT(result.most_dangerous_cluster.obstacle_top_z95_m, 1.0F);
}

TEST(CargoSafetyEvaluator, InvalidAndStaleHeightFailSafe) {
    CargoSafetyEvaluator evaluator;
    CargoSafetyInput invalid = baseInput();
    invalid.height.valid = false;
    EXPECT_EQ(evaluator.evaluate(invalid).raw_code,
              CargoSafetyEvaluator::kLevel2OrFailSafeCode);

    CargoSafetyInput stale = baseInput();
    stale.evaluation_time_sec = 11.0;
    const CargoSafetyResult result = evaluator.evaluate(stale);
    EXPECT_TRUE(result.height_stale);
    EXPECT_EQ(result.raw_code, CargoSafetyEvaluator::kLevel2OrFailSafeCode);
}

TEST(CargoSafetyEvaluator, CompleteValidObservationWithoutObstacleIsClear) {
    CargoSafetyEvaluator evaluator;
    CargoSafetyInput input = baseInput();
    const CargoSafetyResult result = evaluator.evaluate(input);
    EXPECT_TRUE(result.input_valid);
    EXPECT_EQ(result.raw_code, CargoSafetyEvaluator::kSafeCode);
}

TEST(CargoSafetyEvaluator, InsufficientObstacleObservationFailsSafe) {
    CargoSafetyEvaluator evaluator;
    CargoSafetyInput input = baseInput();
    input.obstacle_roi_finite_points = 3;
    const CargoSafetyResult sparse = evaluator.evaluate(input);
    EXPECT_FALSE(sparse.input_valid);
    EXPECT_EQ(sparse.raw_code,
              CargoSafetyEvaluator::kLevel2OrFailSafeCode);
    EXPECT_EQ(sparse.reason, "obstacle_observation_insufficient");

    input = baseInput();
    input.obstacle_cloud_age_sec = 1.0;
    const CargoSafetyResult stale = evaluator.evaluate(input);
    EXPECT_EQ(stale.raw_code,
              CargoSafetyEvaluator::kLevel2OrFailSafeCode);
    EXPECT_EQ(stale.reason, "obstacle_observation_insufficient");
}

TEST(CargoSafetyEvaluator, SparseOrRejectedObstacleReturnsFailSafe) {
    CargoSafetyEvaluator evaluator;
    CargoSafetyInput sparse = baseInput();
    for (int i = 0; i < 3; ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        if (sparse.obstacle_cloud_base) *cloud = *sparse.obstacle_cloud_base;
        cloud->push_back(pcl::PointXYZ(2.0F + 0.01F * i, 0.0F, 1.0F));
        sparse.obstacle_cloud_base = cloud;
    }
    CargoSafetyResult sparse_result = evaluator.evaluate(sparse);
    EXPECT_FALSE(sparse_result.input_valid);
    EXPECT_EQ(sparse_result.raw_code,
              CargoSafetyEvaluator::kLevel2OrFailSafeCode);
    EXPECT_EQ(sparse_result.reason, "sparse_obstacle_returns");

    CargoSafetyConfig config;
    config.obstacle_max_cluster_points = 10;
    CargoSafetyEvaluator bounded(config);
    CargoSafetyInput oversized = baseInput();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < 20; ++i) {
        cloud->push_back(pcl::PointXYZ(
            2.0F + 0.001F * static_cast<float>(i), 0.0F, 1.0F));
    }
    oversized.obstacle_cloud_base = cloud;
    CargoSafetyResult rejected = bounded.evaluate(oversized);
    EXPECT_FALSE(rejected.input_valid);
    EXPECT_EQ(rejected.raw_code,
              CargoSafetyEvaluator::kLevel2OrFailSafeCode);
    EXPECT_EQ(rejected.reason,
              "obstacle_clustering_rejected_all_candidates");
}

}  // namespace
}  // namespace ndt_slam
