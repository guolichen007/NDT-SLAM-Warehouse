#include <gtest/gtest.h>

#include <limits>

#include "ndt_slam/hook_load_state_filter.hpp"

namespace ndt_slam {
namespace {

TEST(HookLoadStateFilter, RequiresConsecutiveLoadedSamples) {
    HookLoadStateFilter filter;
    EXPECT_EQ(filter.ingest(2.20, 0.0).state, HookLoadState::UNKNOWN);
    EXPECT_EQ(filter.ingest(2.20, 0.1).state, HookLoadState::UNKNOWN);
    const auto loaded = filter.ingest(2.20, 0.2);
    EXPECT_TRUE(loaded.valid);
    EXPECT_EQ(loaded.state, HookLoadState::LOADED);
}

TEST(HookLoadStateFilter, HysteresisRejectsThresholdChatter) {
    HookLoadStateFilter filter;
    filter.ingest(2.00, 0.0);
    filter.ingest(2.00, 0.1);
    ASSERT_EQ(filter.ingest(2.00, 0.2).state, HookLoadState::EMPTY);
    EXPECT_EQ(filter.ingest(2.11, 0.3).state, HookLoadState::EMPTY);
    EXPECT_EQ(filter.ingest(2.12, 0.4).state, HookLoadState::EMPTY);
    EXPECT_EQ(filter.ingest(2.14, 0.5).state, HookLoadState::EMPTY);
    EXPECT_EQ(filter.ingest(2.14, 0.6).state, HookLoadState::EMPTY);
    EXPECT_EQ(filter.ingest(2.14, 0.7).state, HookLoadState::LOADED);
}

TEST(HookLoadStateFilter, InhibitAndInvalidInputsAreFailSafe) {
    HookLoadStateFilter filter;
    filter.ingest(1.80, 0.0);
    filter.ingest(1.80, 0.1);
    EXPECT_EQ(filter.ingest(1.80, 0.2).state, HookLoadState::INHIBIT);
    const auto invalid = filter.ingest(
        std::numeric_limits<double>::quiet_NaN(), 0.3);
    EXPECT_FALSE(invalid.valid);
    EXPECT_EQ(invalid.state, HookLoadState::UNKNOWN);
}

TEST(HookLoadStateFilter, StaleSignalBecomesUnknown) {
    HookLoadStateFilter filter;
    filter.ingest(2.00, 0.0);
    filter.ingest(2.00, 0.1);
    ASSERT_EQ(filter.ingest(2.00, 0.2).state, HookLoadState::EMPTY);
    const auto stale = filter.tick(0.71);
    EXPECT_FALSE(stale.valid);
    EXPECT_FALSE(stale.fresh);
    EXPECT_EQ(stale.state, HookLoadState::UNKNOWN);
    EXPECT_EQ(stale.reason, "signal_stale");
}

TEST(HookLoadStateFilter, ThresholdEndpointsBelongToEmpty) {
    HookLoadStateFilter at_low;
    at_low.ingest(1.90, 0.0);
    at_low.ingest(1.90, 0.1);
    EXPECT_EQ(at_low.ingest(1.90, 0.2).state, HookLoadState::EMPTY);

    HookLoadStateFilter at_high;
    at_high.ingest(2.10, 0.0);
    at_high.ingest(2.10, 0.1);
    EXPECT_EQ(at_high.ingest(2.10, 0.2).state, HookLoadState::EMPTY);
}

TEST(HookLoadStateFilter, OutOfRangeAndClockRollbackAreFailSafe) {
    HookLoadStateFilter filter;
    filter.ingest(2.20, 1.0);
    filter.ingest(2.20, 1.1);
    ASSERT_EQ(filter.ingest(2.20, 1.2).state, HookLoadState::LOADED);

    const auto out_of_range = filter.ingest(5.1, 1.3);
    EXPECT_FALSE(out_of_range.valid);
    EXPECT_EQ(out_of_range.state, HookLoadState::UNKNOWN);
    EXPECT_EQ(out_of_range.reason, "voltage_out_of_range");

    filter.ingest(2.0, 2.0);
    const auto rollback = filter.tick(1.9);
    EXPECT_FALSE(rollback.valid);
    EXPECT_EQ(rollback.state, HookLoadState::UNKNOWN);
    EXPECT_EQ(rollback.reason, "wall_time_rollback");
}

}  // namespace
}  // namespace ndt_slam
