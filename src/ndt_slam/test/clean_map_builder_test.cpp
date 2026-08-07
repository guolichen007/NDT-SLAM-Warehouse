#include <gtest/gtest.h>

#include "ndt_slam/clean_map_builder.hpp"

namespace ndt_slam {
namespace {

CleanMapBuildInput observableCell() {
    CleanMapBuildInput input;
    input.object_points = {
        {0.01F, 0.01F, 0.0F}, {0.02F, 0.02F, 0.4F},
        {0.03F, 0.03F, 0.8F}};
    input.observation_counts[{0, 0}] = 2;
    return input;
}

TEST(CleanMapBuilder, KeepsObservableVerticalStructure) {
    const auto result = buildCleanMapFromSnapshot(observableCell());
    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_EQ(result.clean_points.size(), 3U);
    EXPECT_EQ(result.passed_cells, 1);
}

TEST(CleanMapBuilder, RemovesSingleVerticalSpikeFromCleanSnapshot) {
    CleanMapBuildInput input;
    input.object_points = {
        {0.01F, 0.01F, 0.0F}, {0.02F, 0.02F, 0.2F},
        {0.03F, 0.03F, 0.4F}, {0.04F, 0.04F, 0.6F},
        {0.05F, 0.05F, 0.8F}, {0.06F, 0.06F, 1.0F},
        {0.07F, 0.07F, 6.0F}};
    input.observation_counts[{0, 0}] = 2;

    const auto result = buildCleanMapFromSnapshot(input);

    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_EQ(result.clean_points.size(), 6U);
    EXPECT_EQ(result.vertical_outlier_points, 1);
    for (const auto& point : result.clean_points) {
        EXPECT_LE(point.z(), 1.0F);
    }
}

TEST(CleanMapBuilder, PreservesContinuousTallWallStructure) {
    CleanMapBuildInput input;
    for (int index = 0; index < 7; ++index) {
        input.object_points.emplace_back(
            0.01F + 0.01F * index,
            0.01F + 0.01F * index,
            static_cast<float>(index));
    }
    input.observation_counts[{0, 0}] = 2;

    const auto result = buildCleanMapFromSnapshot(input);

    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_EQ(result.clean_points.size(), 7U);
    EXPECT_EQ(result.vertical_outlier_points, 0);
}

TEST(CleanMapBuilder, PreservesMultipleRealVerticalLayersAndDropsSingleSpike) {
    // 低层固定结构 (4 点) + 上层真实梁 (3 点) + 孤立尖峰 (1 点)
    // 只有孤立尖峰应被删除，两层真实结构全部保留。
    CleanMapBuildInput input;
    // 低层：0.10 - 0.40
    input.object_points.emplace_back(0.01F, 0.01F, 0.10F);
    input.object_points.emplace_back(0.02F, 0.02F, 0.20F);
    input.object_points.emplace_back(0.03F, 0.03F, 0.30F);
    input.object_points.emplace_back(0.04F, 0.04F, 0.40F);
    // 上层：1.50 - 1.70
    input.object_points.emplace_back(0.05F, 0.05F, 1.50F);
    input.object_points.emplace_back(0.06F, 0.06F, 1.60F);
    input.object_points.emplace_back(0.07F, 0.07F, 1.70F);
    // 孤立尖峰：6.0m
    input.object_points.emplace_back(0.08F, 0.08F, 6.00F);
    input.observation_counts[{0, 0}] = 2;

    const auto result = buildCleanMapFromSnapshot(input);

    ASSERT_TRUE(result.valid) << result.reason;
    // 8 点输入，1 点删除 → 7 点保留。
    EXPECT_EQ(result.clean_points.size(), 7U);
    EXPECT_EQ(result.vertical_outlier_points, 1);
    // 低层所有点必须保留。
    int lower_kept = 0;
    int upper_kept = 0;
    for (const auto& point : result.clean_points) {
        if (point.z() >= 0.05F && point.z() <= 0.45F) ++lower_kept;
        if (point.z() >= 1.45F && point.z() <= 1.75F) ++upper_kept;
    }
    EXPECT_EQ(lower_kept, 4);
    EXPECT_EQ(upper_kept, 3);
}

TEST(CleanMapBuilder, RetainsExactPreviousCellUntilObservationThreshold) {
    auto input = observableCell();
    input.observation_counts[{0, 0}] = 1;
    input.previous_clean_points = {
        {0.04F, 0.04F, 0.1F}, {0.05F, 0.05F, 0.7F}};

    const auto result = buildCleanMapFromSnapshot(input);

    ASSERT_TRUE(result.valid) << result.reason;
    ASSERT_EQ(result.clean_points.size(), 2U);
    EXPECT_FLOAT_EQ(result.clean_points[0].z(), 0.1F);
    EXPECT_FLOAT_EQ(result.clean_points[1].z(), 0.7F);
    EXPECT_EQ(result.retained_cells, 1);
    EXPECT_EQ(result.retained_points, 2);
    EXPECT_EQ(result.passed_cells, 1);
}

TEST(CleanMapBuilder, PartialObservationDoesNotEraseUnobservedRegion) {
    CleanMapBuildInput input;
    input.object_points = {
        {0.01F, 0.01F, 0.0F}, {0.02F, 0.02F, 0.4F},
        {0.03F, 0.03F, 0.8F}, {1.51F, 0.01F, 0.0F},
        {1.52F, 0.02F, 0.4F}, {1.53F, 0.03F, 0.8F}};
    input.previous_clean_points = {
        {1.54F, 0.04F, 0.1F}, {1.55F, 0.05F, 0.7F}};
    input.observation_counts[{0, 0}] = 2;

    const auto result = buildCleanMapFromSnapshot(input);

    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_EQ(result.clean_points.size(), 5U);
    EXPECT_EQ(result.passed_cells, 2);
    EXPECT_EQ(result.retained_cells, 1);
    EXPECT_EQ(result.retained_points, 2);
}

TEST(CleanMapBuilder, RebuildsFromRawAfterObservationThreshold) {
    auto input = observableCell();
    input.previous_clean_points = {{0.04F, 0.04F, 0.2F}};

    const auto result = buildCleanMapFromSnapshot(input);

    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_EQ(result.clean_points.size(), 3U);
    EXPECT_EQ(result.retained_cells, 0);
    EXPECT_EQ(result.retained_points, 0);
}

TEST(CleanMapBuilder, ExplicitDenyStillRemovesRetainedCell) {
    auto input = observableCell();
    input.observation_counts[{0, 0}] = 1;
    input.previous_clean_points = {
        {0.04F, 0.04F, 0.1F}, {0.05F, 0.05F, 0.7F}};
    input.deny_cells.insert({0, 0});

    const auto result = buildCleanMapFromSnapshot(input);

    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_TRUE(result.clean_points.empty());
    EXPECT_EQ(result.retained_cells, 0);
    EXPECT_EQ(result.denied_cells, 1);
}

TEST(CleanMapBuilder, ThreeDimensionalDenyAppliesToRetainedCell) {
    auto input = observableCell();
    input.observation_counts[{0, 0}] = 1;
    input.previous_clean_points = {
        {0.04F, 0.04F, 0.1F}, {0.05F, 0.05F, 0.7F}};
    input.use_3d_deny = true;
    input.deny_ranges[{0, 0}].push_back({0.5F, 0.9F});

    const auto result = buildCleanMapFromSnapshot(input);

    ASSERT_TRUE(result.valid) << result.reason;
    ASSERT_EQ(result.clean_points.size(), 1U);
    EXPECT_FLOAT_EQ(result.clean_points.front().z(), 0.1F);
    EXPECT_EQ(result.retained_cells, 1);
    EXPECT_EQ(result.retained_points, 1);
    EXPECT_EQ(result.denied_points, 1);
}

TEST(CleanMapBuilder, ProtectWinsOverDenyAndRestoresCandidate) {
    auto input = observableCell();
    input.deny_cells.insert({0, 0});
    input.protect_cells.insert({0, 0});
    input.payload_candidate_points.push_back({0.04F, 0.04F, 0.5F});
    const auto result = buildCleanMapFromSnapshot(input);
    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_EQ(result.clean_points.size(), 4U);
    EXPECT_EQ(result.protected_cells, 1);
}

TEST(CleanMapBuilder, HumanAndThreeDimensionalDenyAreApplied) {
    auto human = observableCell();
    human.use_human_deny = true;
    human.human_deny_cells.insert({0, 0});
    const auto human_result = buildCleanMapFromSnapshot(human);
    ASSERT_TRUE(human_result.valid);
    EXPECT_TRUE(human_result.clean_points.empty());
    EXPECT_EQ(human_result.human_denied_cells, 1);

    auto volume = observableCell();
    volume.use_3d_deny = true;
    volume.deny_ranges[{0, 0}].push_back({0.3F, 0.9F});
    const auto volume_result = buildCleanMapFromSnapshot(volume);
    ASSERT_TRUE(volume_result.valid);
    EXPECT_EQ(volume_result.clean_points.size(), 1U);
    EXPECT_EQ(volume_result.denied_points, 2);
}

TEST(CleanMapBuilder, LargeMapWithoutObservationHistoryFailsClosed) {
    CleanMapBuildInput input;
    input.object_points.resize(1001U, Eigen::Vector3f(0.0F, 0.0F, 1.0F));
    const auto result = buildCleanMapFromSnapshot(input);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.reason, "observation_history_empty");
}

TEST(CleanMapBuilder, NewerRequestDoesNotStarveCurrentObjectsSnapshot) {
    EXPECT_EQ(evaluateCleanMapBuildAction(true, false, 7U, 7U),
              CleanMapBuildAction::APPLY);
    EXPECT_EQ(evaluateCleanMapBuildAction(false, false, 7U, 7U),
              CleanMapBuildAction::DISCARD_INVALID);
    EXPECT_EQ(evaluateCleanMapBuildAction(true, true, 7U, 7U),
              CleanMapBuildAction::APPLY);
    EXPECT_EQ(evaluateCleanMapBuildAction(true, false, 7U, 8U),
              CleanMapBuildAction::PUBLISH_SNAPSHOT_ONLY);
}

}  // namespace
}  // namespace ndt_slam
