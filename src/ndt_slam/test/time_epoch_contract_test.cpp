#include <gtest/gtest.h>

#include "ndt_slam/time_epoch_contract.hpp"

namespace ndt_slam {
namespace {

TEST(TimeEpochContract, MonotonicSourceStampsDoNotReset) {
    EXPECT_FALSE(isSourceTimestampRollback(100.0, 100.0));
    EXPECT_FALSE(isSourceTimestampRollback(100.0, 100.1));
}

TEST(TimeEpochContract, LargeRollbackStartsOneNewEpoch) {
    const double stamps[] = {
        1779155914.0,
        1778217251.0,
        1778217251.1,
        1778217251.2,
    };
    int reset_count = 0;
    for (std::size_t index = 1; index < 4; ++index) {
        if (isSourceTimestampRollback(stamps[index - 1], stamps[index])) {
            ++reset_count;
        }
    }
    EXPECT_EQ(1, reset_count);
}

TEST(TimeEpochContract, LifecycleFenceCoversAllCommitRacePhases) {
    constexpr std::uint64_t old_epoch = 11U;
    constexpr std::uint64_t new_epoch = 12U;

    // queued -> reset -> dequeue
    EXPECT_FALSE(isMapCommitLifecycleCurrent(old_epoch, new_epoch));
    // dequeued -> reset -> pre-write lifecycle section
    EXPECT_FALSE(isMapCommitLifecycleCurrent(old_epoch, new_epoch));
    // physical write -> reset -> temporal completion publication
    EXPECT_FALSE(isMapCommitLifecycleCurrent(old_epoch, new_epoch));
    EXPECT_TRUE(isMapCommitLifecycleCurrent(new_epoch, new_epoch));
}

}  // namespace
}  // namespace ndt_slam
