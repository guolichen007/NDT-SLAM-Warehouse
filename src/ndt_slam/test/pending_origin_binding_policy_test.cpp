#include "ndt_slam/pending_origin_binding_policy.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace ndt_slam {
namespace {

PendingOriginBindingInput validInput() {
    PendingOriginBindingInput input;
    input.pending_valid = true;
    input.lidar_stamp_sec = 100.1;
    input.origin_stamp_sec = 100.0;
    input.max_age_sec = 2.0;
    input.future_stamp_tolerance_sec = 0.05;
    input.track_center_finite = true;
    input.origin_center_finite = true;
    input.center_distance_m = 0.10;
    input.maximum_center_distance_m = 0.50;
    return input;
}

TEST(PendingOriginBindingPolicy, KeepsFutureOriginUntilLidarCatchesUp) {
    PendingOriginBindingInput input = validInput();
    input.lidar_stamp_sec = 99.9;
    input.origin_stamp_sec = 100.0;
    bool pending_retained = true;
    PendingOriginAction action = evaluatePendingOriginBinding(input);
    EXPECT_EQ(action, PendingOriginAction::KEEP_WAITING_FOR_LIDAR_TIME);
    if (action != PendingOriginAction::KEEP_WAITING_FOR_LIDAR_TIME) {
        pending_retained = false;
    }
    EXPECT_TRUE(pending_retained);

    // The caller retains pending state after KEEP; the next LiDAR frame can
    // attach the same origin when its sensor time catches up.
    input.lidar_stamp_sec = 100.1;
    action = evaluatePendingOriginBinding(input);
    EXPECT_EQ(action, PendingOriginAction::ATTACH);
    if (action == PendingOriginAction::ATTACH) pending_retained = false;
    EXPECT_FALSE(pending_retained);
}

TEST(PendingOriginBindingPolicy, DiscardsExpiredOrigin) {
    PendingOriginBindingInput input = validInput();
    input.lidar_stamp_sec = 103.0;
    input.origin_stamp_sec = 100.0;
    input.max_age_sec = 2.0;
    EXPECT_EQ(evaluatePendingOriginBinding(input),
              PendingOriginAction::DISCARD_EXPIRED);
}

TEST(PendingOriginBindingPolicy, DiscardsSpatialMismatch) {
    PendingOriginBindingInput input = validInput();
    input.center_distance_m = 0.75;
    input.maximum_center_distance_m = 0.50;
    EXPECT_EQ(evaluatePendingOriginBinding(input),
              PendingOriginAction::DISCARD_SPATIAL_MISMATCH);
}

TEST(PendingOriginBindingPolicy, AcceptsSmallFutureSkewAsZeroAge) {
    PendingOriginBindingInput input = validInput();
    input.lidar_stamp_sec = 99.98;
    input.origin_stamp_sec = 100.0;
    input.future_stamp_tolerance_sec = 0.05;
    EXPECT_EQ(evaluatePendingOriginBinding(input),
              PendingOriginAction::ATTACH);
}

TEST(PendingOriginBindingPolicy, RejectsNonFiniteOrInvalidEpochInput) {
    PendingOriginBindingInput input = validInput();
    input.lidar_stamp_sec = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(evaluatePendingOriginBinding(input),
              PendingOriginAction::DISCARD_INVALID);

    input = validInput();
    input.origin_stamp_sec = std::numeric_limits<double>::infinity();
    EXPECT_EQ(evaluatePendingOriginBinding(input),
              PendingOriginAction::DISCARD_INVALID);

    input = validInput();
    input.track_center_finite = false;
    EXPECT_EQ(evaluatePendingOriginBinding(input),
              PendingOriginAction::DISCARD_INVALID);
}

TEST(PendingOriginBindingPolicy, ReturnsNoneWithoutPendingOrigin) {
    PendingOriginBindingInput input = validInput();
    input.pending_valid = false;
    EXPECT_EQ(evaluatePendingOriginBinding(input),
              PendingOriginAction::NONE);
}

TEST(PendingOriginBindingPolicy, EpochResetRemovesPendingBeforeEvaluation) {
    PendingOriginBindingInput input = validInput();
    // The runtime rollback handler clears pending_valid before any new-epoch
    // LiDAR frame can evaluate the old origin.
    input.pending_valid = false;
    input.lidar_stamp_sec = 1.0;
    input.origin_stamp_sec = 100.0;
    EXPECT_EQ(evaluatePendingOriginBinding(input),
              PendingOriginAction::NONE);
}

}  // namespace
}  // namespace ndt_slam
