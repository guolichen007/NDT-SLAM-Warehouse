#include <gtest/gtest.h>

#include <limits>

#include "ndt_slam/hook_load_state_filter.hpp"

namespace ndt_slam {
namespace {

TEST(HookLoadStateFilter, RequiresConsecutiveLoadedSamples) {
    HookLoadStateFilter filter;
    EXPECT_EQ(filter.ingest(2.20, 0.0).state, HookLoadState::UNKNOWN);
    const auto loaded = filter.ingest(2.20, 0.1);
    EXPECT_TRUE(loaded.valid);
    EXPECT_EQ(loaded.state, HookLoadState::LOADED);
}

TEST(HookLoadStateFilter, HysteresisRejectsThresholdChatter) {
    HookLoadStateConfig config;
    config.confirm_samples = 3;
    HookLoadStateFilter filter(config);
    filter.ingest(2.00, 0.0);
    EXPECT_EQ(filter.ingest(2.00, 0.1).state, HookLoadState::UNKNOWN);
    ASSERT_EQ(filter.ingest(2.00, 0.2).state, HookLoadState::EMPTY);
    EXPECT_EQ(filter.ingest(2.11, 0.3).state, HookLoadState::EMPTY);
    EXPECT_EQ(filter.ingest(2.12, 0.4).state, HookLoadState::EMPTY);
    EXPECT_EQ(filter.ingest(2.14, 0.5).state, HookLoadState::EMPTY);
    EXPECT_EQ(filter.ingest(2.14, 0.6).state, HookLoadState::EMPTY);
    EXPECT_EQ(filter.ingest(2.14, 0.7).state, HookLoadState::LOADED);
}

TEST(HookLoadStateFilter, DefaultFreshTwoSamplesConfirmLoaded) {
    HookLoadStateFilter filter;
    EXPECT_EQ(filter.ingest(2.20, 10.0, 1.0).state,
              HookLoadState::UNKNOWN);
    const auto loaded = filter.ingest(2.20, 11.0, 2.0);
    EXPECT_TRUE(loaded.valid);
    EXPECT_TRUE(loaded.fresh);
    EXPECT_EQ(loaded.state, HookLoadState::LOADED);
}

TEST(HookLoadStateFilter, TransitionRequiresContinuousMinimumDuration) {
    HookLoadStateConfig config;
    config.confirm_samples = 2;
    config.minimum_transition_duration_sec = 0.50;
    HookLoadStateFilter filter(config);

    EXPECT_EQ(filter.ingest(2.20, 10.0).state, HookLoadState::UNKNOWN);
    EXPECT_EQ(filter.ingest(2.20, 10.2).state, HookLoadState::UNKNOWN);
    EXPECT_EQ(filter.ingest(2.20, 10.49).state, HookLoadState::UNKNOWN);
    EXPECT_EQ(filter.ingest(2.20, 10.50).state, HookLoadState::LOADED);

    EXPECT_EQ(filter.ingest(2.00, 10.60).state, HookLoadState::LOADED);
    EXPECT_EQ(filter.ingest(2.20, 10.70).state, HookLoadState::LOADED);
    EXPECT_EQ(filter.ingest(2.00, 10.80).state, HookLoadState::LOADED);
}

TEST(HookLoadStateFilter, InhibitAndInvalidInputsAreFailSafe) {
    HookLoadStateFilter filter;
    filter.ingest(1.80, 0.0);
    EXPECT_EQ(filter.ingest(1.80, 0.1).state, HookLoadState::INHIBIT);
    const auto invalid = filter.ingest(
        std::numeric_limits<double>::quiet_NaN(), 0.3);
    EXPECT_FALSE(invalid.valid);
    EXPECT_EQ(invalid.state, HookLoadState::UNKNOWN);
}

TEST(HookLoadStateFilter, ShortStaleBecomesHeldStale) {
    HookLoadStateFilter filter;
    filter.ingest(2.00, 0.0);
    ASSERT_EQ(filter.ingest(2.00, 0.1).state, HookLoadState::EMPTY);
    // 2.61s > stale_timeout(2.50s) 但 < held_stale_timeout(5.00s)
    const auto held = filter.tick(2.61);
    EXPECT_TRUE(held.valid);
    EXPECT_FALSE(held.fresh);
    EXPECT_TRUE(held.held_stale);
    EXPECT_EQ(held.state, HookLoadState::EMPTY);
    EXPECT_EQ(held.reason, "empty_held_stale");
}

TEST(HookLoadStateFilter, LongStaleBecomesUnknown) {
    HookLoadStateFilter filter;
    filter.ingest(2.00, 0.0);
    ASSERT_EQ(filter.ingest(2.00, 0.1).state, HookLoadState::EMPTY);
    // 10.0s > stale_timeout + held_stale_timeout(7.50s)
    const auto unknown = filter.tick(10.0);
    EXPECT_FALSE(unknown.valid);
    EXPECT_EQ(unknown.state, HookLoadState::UNKNOWN);
}

TEST(HookLoadStateFilter, ThresholdEndpointsBelongToEmpty) {
    HookLoadStateFilter at_low;
    at_low.ingest(1.90, 0.0);
    EXPECT_EQ(at_low.ingest(1.90, 0.1).state, HookLoadState::EMPTY);

    HookLoadStateFilter at_high;
    at_high.ingest(2.10, 0.0);
    EXPECT_EQ(at_high.ingest(2.10, 0.1).state, HookLoadState::EMPTY);
}

TEST(HookLoadStateFilter, OutOfRangeAndClockRollbackAreFailSafe) {
    HookLoadStateFilter filter;
    filter.ingest(2.20, 1.0);
    ASSERT_EQ(filter.ingest(2.20, 1.1).state, HookLoadState::LOADED);

    const auto out_of_range = filter.ingest(6.1, 1.3);
    EXPECT_FALSE(out_of_range.valid);
    EXPECT_EQ(out_of_range.state, HookLoadState::UNKNOWN);
    EXPECT_EQ(out_of_range.reason, "voltage_out_of_range");

    filter.ingest(2.0, 2.0);
    const auto rollback = filter.tick(1.9);
    EXPECT_FALSE(rollback.valid);
    EXPECT_EQ(rollback.state, HookLoadState::UNKNOWN);
    EXPECT_EQ(rollback.reason, "wall_time_rollback");
}

TEST(HookLoadStateFilter, OneHertzAndDuplicateSamplesUseIndependentEvidence) {
    HookLoadStateConfig config;
    config.confirm_samples = 2;
    config.stale_timeout_sec = 2.5;
    config.valid_voltage_max_v = 6.0;
    HookLoadStateFilter filter(config);

    EXPECT_EQ(filter.ingest(5.417, 100.0, 10.0).state,
              HookLoadState::UNKNOWN);
    const auto duplicate = filter.ingest(5.417, 100.0, 10.8);
    EXPECT_EQ(duplicate.state, HookLoadState::UNKNOWN);
    EXPECT_EQ(duplicate.reason, "duplicate_sample_ignored");
    EXPECT_EQ(filter.tick(10.9).state, HookLoadState::UNKNOWN);
    EXPECT_EQ(filter.ingest(5.417, 101.0, 11.0).state,
              HookLoadState::LOADED);

    const auto stale_duplicate = filter.ingest(5.417, 101.0, 13.5);
    EXPECT_TRUE(stale_duplicate.valid);
    EXPECT_FALSE(stale_duplicate.fresh);
    EXPECT_TRUE(stale_duplicate.held_stale);
    EXPECT_EQ(stale_duplicate.state, HookLoadState::LOADED);
    EXPECT_EQ(stale_duplicate.reason, "loaded_held_stale");
    const auto recovered = filter.ingest(5.417, 102.0, 14.0);
    EXPECT_TRUE(recovered.valid);
    EXPECT_TRUE(recovered.fresh);
    EXPECT_FALSE(recovered.held_stale);
    EXPECT_EQ(recovered.state, HookLoadState::LOADED);
    EXPECT_EQ(recovered.reason, "recovered_from_held_stale");
}

TEST(HookLoadStateFilter, SourceRollbackStartsARecoverableEpoch) {
    HookLoadStateFilter filter;
    filter.ingest(5.0, 100.0, 1.0);
    ASSERT_EQ(filter.ingest(5.0, 101.0, 2.0).state,
              HookLoadState::LOADED);

    const auto rollback = filter.ingest(5.0, 1.0, 3.0);
    EXPECT_FALSE(rollback.valid);
    EXPECT_EQ(rollback.reason, "source_time_rollback");
    EXPECT_EQ(filter.ingest(5.0, 1.1, 4.0).state,
              HookLoadState::UNKNOWN);
    EXPECT_EQ(filter.ingest(5.0, 1.2, 5.0).state,
              HookLoadState::LOADED);
}

TEST(HookLoadStateFilter, StaleReplayCannotSeedASecondConfirmation) {
    HookLoadStateFilter filter;
    filter.ingest(5.0, 10.0, 1.0);
    ASSERT_EQ(filter.ingest(5.0, 11.0, 2.0).state,
              HookLoadState::LOADED);
    // 4.6s: stale_timeout(2.5s) exceeded, held_stale kicks in
    const auto held = filter.tick(4.6);
    ASSERT_EQ(held.reason, "loaded_held_stale");
    ASSERT_TRUE(held.held_stale);

    const auto replay = filter.ingest(5.0, 11.0, 4.7);
    EXPECT_TRUE(replay.valid);
    EXPECT_FALSE(replay.fresh);
    EXPECT_TRUE(replay.held_stale);
    EXPECT_EQ(replay.state, HookLoadState::LOADED);
    EXPECT_EQ(replay.reason, "loaded_held_stale");
    const auto first_fresh = filter.ingest(5.0, 12.0, 5.0);
    EXPECT_TRUE(first_fresh.valid);
    EXPECT_TRUE(first_fresh.fresh);
    EXPECT_FALSE(first_fresh.held_stale);
    EXPECT_EQ(first_fresh.reason, "recovered_from_held_stale");
}

TEST(HookLoadStateFilter, InvalidHeldTimeoutFailsConfiguration) {
    HookLoadStateConfig config;
    config.held_stale_timeout_sec = 0.0;
    HookLoadStateFilter filter(config);
    const auto result = filter.ingest(5.0, 1.0, 1.0);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.reason, "invalid_config");
}

}  // namespace
}  // namespace ndt_slam
