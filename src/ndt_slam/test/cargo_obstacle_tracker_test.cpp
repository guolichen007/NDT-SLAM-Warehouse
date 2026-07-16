#include <gtest/gtest.h>

#include "ndt_slam/cargo_obstacle_tracker.hpp"

namespace ndt_slam {
namespace {

CargoObstacleObservation hazard(
    std::size_t source, float x, float y, std::uint16_t code = 17U) {
  CargoObstacleObservation observation;
  observation.source_index = source;
  observation.centroid_map = Eigen::Vector3f(x, y, 1.0F);
  observation.top_z95_map = 1.5F;
  observation.footprint_distance_m = code == 17U ? 2.0F : 4.0F;
  observation.conservative_clearance_m = 0.30F;
  observation.point_count = 30U;
  observation.warning_code = code;
  return observation;
}

TEST(CargoObstacleTracker, DifferentWinnerOrderKeepsIndependentIdentity) {
  CargoObstacleTracker tracker;
  EXPECT_FALSE(tracker.update(
      1.0, {hazard(0U, 0.0F, 0.0F), hazard(1U, 3.0F, 0.0F, 18U)})
                   .confirmed_hazard);
  EXPECT_FALSE(tracker.update(
      1.2, {hazard(1U, 3.02F, 0.0F, 18U), hazard(0U, 0.02F, 0.0F)})
                   .confirmed_hazard);
  const CargoObstacleTrackerDecision decision = tracker.update(
      1.4, {hazard(0U, 0.04F, 0.0F), hazard(1U, 3.04F, 0.0F, 18U)});
  EXPECT_TRUE(decision.confirmed_hazard);
  EXPECT_EQ(decision.warning_code, 17U);
  EXPECT_EQ(decision.selected_confirm_count, 3);
}

TEST(CargoObstacleTracker, JumpingClustersCannotShareConfirmationCount) {
  CargoObstacleTracker tracker;
  tracker.update(1.0, {hazard(0U, 0.0F, 0.0F)});
  tracker.update(1.2, {hazard(0U, 2.0F, 0.0F)});
  const CargoObstacleTrackerDecision decision =
      tracker.update(1.4, {hazard(0U, -2.0F, 0.0F)});
  EXPECT_FALSE(decision.confirmed_hazard);
  EXPECT_EQ(decision.selected_confirm_count, 1);
}

TEST(CargoObstacleTracker, RepeatedStampDoesNotAdvanceTrack) {
  CargoObstacleTracker tracker;
  tracker.update(1.0, {hazard(0U, 0.0F, 0.0F)});
  tracker.update(1.0, {hazard(0U, 0.0F, 0.0F)});
  EXPECT_FALSE(
      tracker.update(1.2, {hazard(0U, 0.02F, 0.0F)}).confirmed_hazard);
  EXPECT_TRUE(
      tracker.update(1.4, {hazard(0U, 0.04F, 0.0F)}).confirmed_hazard);
}

TEST(CargoObstacleTracker, MissingCycleBreaksConsecutiveEvidence) {
  CargoObstacleTracker tracker;
  tracker.update(1.0, {hazard(0U, 0.0F, 0.0F)});
  tracker.update(1.2, {});
  tracker.update(1.4, {hazard(0U, 0.02F, 0.0F)});
  const CargoObstacleTrackerDecision decision =
      tracker.update(1.6, {hazard(0U, 0.04F, 0.0F)});
  EXPECT_FALSE(decision.confirmed_hazard);
  EXPECT_EQ(decision.selected_confirm_count, 2);
}

}  // namespace
}  // namespace ndt_slam
