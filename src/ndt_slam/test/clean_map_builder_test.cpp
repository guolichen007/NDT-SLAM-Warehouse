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

TEST(CleanMapBuilder, WorkerResultRequiresCurrentUnsupersededSnapshot) {
    EXPECT_EQ(evaluateCleanMapBuildAction(true, false, 7U, 7U),
              CleanMapBuildAction::APPLY);
    EXPECT_EQ(evaluateCleanMapBuildAction(false, false, 7U, 7U),
              CleanMapBuildAction::DISCARD_INVALID);
    EXPECT_EQ(evaluateCleanMapBuildAction(true, true, 7U, 7U),
              CleanMapBuildAction::DISCARD_SUPERSEDED);
    EXPECT_EQ(evaluateCleanMapBuildAction(true, false, 7U, 8U),
              CleanMapBuildAction::DISCARD_STALE_OBJECTS);
}

}  // namespace
}  // namespace ndt_slam
