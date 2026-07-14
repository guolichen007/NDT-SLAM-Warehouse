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
    HookLoadStateFilter filter;
    filter.ingest(2.00, 0.0);
    ASSERT_EQ(filter.ingest(2.00, 0.1).state, HookLoadState::EMPTY);
    EXPECT_EQ(filter.ingest(2.11, 0.3).state, HookLoadState::EMPTY);
    EXPECT_EQ(filter.ingest(2.12, 0.4).state, HookLoadState::EMPTY);
    EXPECT_EQ(filter.ingest(2.14, 0.5).state, HookLoadState::EMPTY);
    EXPECT_EQ(filter.ingest(2.14, 0.6).state, HookLoadState::EMPTY);
    EXPECT_EQ(filter.ingest(2.14, 0.7).state, HookLoadState::LOADED);
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

TEST(HookLoadStateFilter, StaleSignalBecomesUnknown) {
    HookLoadStateFilter filter;
    filter.ingest(2.00, 0.0);
    ASSERT_EQ(filter.ingest(2.00, 0.1).state, HookLoadState::EMPTY);
    const auto stale = filter.tick(2.61);
    EXPECT_FALSE(stale.valid);
    EXPECT_FALSE(stale.fresh);
    EXPECT_EQ(stale.state, HookLoadState::UNKNOWN);
    EXPECT_EQ(stale.reason, "signal_stale");
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

    const auto stale = filter.ingest(5.417, 101.0, 13.5);
    EXPECT_FALSE(stale.valid);
    EXPECT_EQ(stale.state, HookLoadState::UNKNOWN);
    EXPECT_EQ(stale.reason, "signal_stale");
    EXPECT_EQ(filter.ingest(5.417, 102.0, 14.0).state,
              HookLoadState::UNKNOWN);
    EXPECT_EQ(filter.ingest(5.417, 103.0, 15.0).state,
              HookLoadState::LOADED);
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

}  // namespace
}  // namespace ndt_slam
