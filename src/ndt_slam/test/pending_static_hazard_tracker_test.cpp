#include "ndt_slam/pending_static_hazard_tracker.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

PendingStaticHazardObservation hazard(double stamp) {
  PendingStaticHazardObservation observation;
  observation.stamp_sec = stamp;
  observation.cargo_lifecycle_id = 7U;
  observation.map_generation = 3U;
  observation.authority_valid = true;
  observation.query_valid = true;
  observation.query_bounded = true;
  observation.hazard = true;
  observation.warning_code = 18;
  observation.matched_cell_keys = {10, 11, 12, 13};
  return observation;
}

TEST(PendingStaticHazardTracker, RequiresContinuousStableRegion) {
  PendingStaticHazardTracker tracker;
  EXPECT_FALSE(tracker.update(hazard(1.0)).authorized);
  EXPECT_FALSE(tracker.update(hazard(1.1)).authorized);
  const auto confirmed = tracker.update(hazard(1.2));
  EXPECT_TRUE(confirmed.authorized);
  EXPECT_NE(confirmed.obstacle_id, 0U);
  EXPECT_NE(confirmed.obstacle_id & 0x80000000U, 0U);
}

TEST(PendingStaticHazardTracker, UnverifiedMapNeverAuthorizes) {
  PendingStaticHazardTracker tracker;
  auto observation = hazard(1.0);
  observation.authority_valid = false;
  const auto decision = tracker.update(observation);
  EXPECT_FALSE(decision.authorized);
  EXPECT_EQ(decision.reason, "static_authority_not_valid");
}

TEST(PendingStaticHazardTracker, LifecycleAndGenerationResetEvidence) {
  PendingStaticHazardTracker tracker;
  tracker.update(hazard(1.0));
  tracker.update(hazard(1.1));

  auto next_lifecycle = hazard(1.2);
  next_lifecycle.cargo_lifecycle_id = 8U;
  EXPECT_EQ(tracker.update(next_lifecycle).confirmations, 1);

  auto next_generation = next_lifecycle;
  next_generation.stamp_sec = 1.3;
  next_generation.map_generation = 4U;
  EXPECT_EQ(tracker.update(next_generation).confirmations, 1);
}

TEST(PendingStaticHazardTracker, RegionSwitchAndGapResetEvidence) {
  PendingStaticHazardTracker tracker;
  tracker.update(hazard(1.0));
  tracker.update(hazard(1.1));

  auto switched = hazard(1.2);
  switched.matched_cell_keys = {100, 101, 102};
  EXPECT_EQ(tracker.update(switched).confirmations, 1);

  auto delayed = switched;
  delayed.stamp_sec = 2.0;
  EXPECT_EQ(tracker.update(delayed).confirmations, 1);
}

TEST(PendingStaticHazardTracker, DuplicateStampCannotForgeConfirmation) {
  PendingStaticHazardTracker tracker;
  EXPECT_EQ(tracker.update(hazard(1.0)).confirmations, 1);
  EXPECT_EQ(tracker.update(hazard(1.0)).confirmations, 1);
  EXPECT_FALSE(tracker.update(hazard(1.0)).authorized);
  EXPECT_EQ(tracker.update(hazard(1.1)).confirmations, 2);
}

TEST(PendingStaticHazardTracker, ClearObservationDropsAuthority) {
  PendingStaticHazardTracker tracker;
  tracker.update(hazard(1.0));
  tracker.update(hazard(1.1));
  ASSERT_TRUE(tracker.update(hazard(1.2)).authorized);

  auto clear = hazard(1.3);
  clear.hazard = false;
  clear.warning_code = 14;
  const auto decision = tracker.update(clear);
  EXPECT_FALSE(decision.authorized);
  EXPECT_EQ(decision.reason, "no_static_positive_hazard");
  EXPECT_EQ(tracker.update(hazard(1.4)).confirmations, 1);
}

}  // namespace
}  // namespace ndt_slam
