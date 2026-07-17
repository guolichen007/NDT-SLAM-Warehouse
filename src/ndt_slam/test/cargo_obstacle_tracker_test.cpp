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

CargoObstacleObservation staticCargo(
    std::size_t source, float x, float y, std::uint16_t code = 17U) {
  CargoObstacleObservation observation = hazard(source, x, y, code);
  observation.point_count = 120U;
  observation.raw_equivalent_point_count = 700U;
  observation.xy_area_m2 = 1.20F;
  observation.long_side_m = 1.60F;
  observation.height_span_m = 0.80F;
  observation.occupied_cells = 24U;
  observation.independent_external_provenance = true;
  return observation;
}

CargoObstacleTrackerConfig ordinaryHazardConfig() {
  CargoObstacleTrackerConfig config;
  config.require_static_cargo_for_warning = false;
  return config;
}

TEST(CargoObstacleTracker, DifferentWinnerOrderKeepsIndependentIdentity) {
  CargoObstacleTracker tracker(ordinaryHazardConfig());
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
  CargoObstacleTracker tracker(ordinaryHazardConfig());
  tracker.update(1.0, {hazard(0U, 0.0F, 0.0F)});
  tracker.update(1.2, {hazard(0U, 2.0F, 0.0F)});
  const CargoObstacleTrackerDecision decision =
      tracker.update(1.4, {hazard(0U, -2.0F, 0.0F)});
  EXPECT_FALSE(decision.confirmed_hazard);
  EXPECT_EQ(decision.selected_confirm_count, 1);
}

TEST(CargoObstacleTracker, RepeatedStampDoesNotAdvanceTrack) {
  CargoObstacleTracker tracker(ordinaryHazardConfig());
  tracker.update(1.0, {hazard(0U, 0.0F, 0.0F)});
  tracker.update(1.0, {hazard(0U, 0.0F, 0.0F)});
  EXPECT_FALSE(
      tracker.update(1.2, {hazard(0U, 0.02F, 0.0F)}).confirmed_hazard);
  EXPECT_TRUE(
      tracker.update(1.4, {hazard(0U, 0.04F, 0.0F)}).confirmed_hazard);
}

TEST(CargoObstacleTracker, MissingCycleBreaksConsecutiveEvidence) {
  CargoObstacleTracker tracker(ordinaryHazardConfig());
  tracker.update(1.0, {hazard(0U, 0.0F, 0.0F)});
  tracker.update(1.2, {});
  tracker.update(1.4, {hazard(0U, 0.02F, 0.0F)});
  const CargoObstacleTrackerDecision decision =
      tracker.update(1.6, {hazard(0U, 0.04F, 0.0F)});
  EXPECT_FALSE(decision.confirmed_hazard);
  EXPECT_EQ(decision.selected_confirm_count, 2);
}

TEST(CargoObstacleTracker, UnresolvedNearZeroTrackCannotPublishHazard) {
  CargoObstacleTracker tracker(ordinaryHazardConfig());
  CargoObstacleObservation unresolved = hazard(0U, 0.0F, 0.0F);
  unresolved.source_validated = false;
  tracker.update(1.0, {unresolved});
  tracker.update(1.2, {unresolved});
  const CargoObstacleTrackerDecision pending =
      tracker.update(1.4, {unresolved});
  EXPECT_FALSE(pending.confirmed_hazard);
  EXPECT_EQ(pending.selected_confirm_count, 0);

  unresolved.source_validated = true;
  const CargoObstacleTrackerDecision validated =
      tracker.update(1.6, {unresolved});
  EXPECT_FALSE(validated.confirmed_hazard);
  EXPECT_EQ(validated.selected_confirm_count, 1);
  EXPECT_FALSE(
      tracker.update(1.8, {unresolved}).confirmed_hazard);
  EXPECT_TRUE(
      tracker.update(2.0, {unresolved}).confirmed_hazard);
}

TEST(CargoObstacleTracker, UnvalidatedFrameResetsValidatedStreak) {
  CargoObstacleTracker tracker(ordinaryHazardConfig());
  CargoObstacleObservation observation = hazard(0U, 0.0F, 0.0F);
  tracker.update(1.0, {observation});
  tracker.update(1.2, {observation});
  observation.source_validated = false;
  tracker.update(1.4, {observation});
  observation.source_validated = true;
  EXPECT_FALSE(tracker.update(1.6, {observation}).confirmed_hazard);
  EXPECT_EQ(tracker.tracks().front().validated_consecutive_observations, 1);
}

TEST(CargoObstacleTracker, TwentyPointTrackCannotBecomeStaticCargo) {
  CargoObstacleTracker tracker;
  CargoObstacleObservation observation = hazard(0U, 0.0F, 0.0F);
  observation.independent_external_provenance = true;
  for (int i = 0; i < 12; ++i) {
    const CargoObstacleTrackerDecision decision = tracker.update(
        1.0 + 0.2 * i, {observation});
    EXPECT_FALSE(decision.confirmed_hazard);
  }
  ASSERT_FALSE(tracker.tracks().empty());
  EXPECT_FALSE(tracker.tracks().front().static_obstacle);
}

TEST(CargoObstacleTracker, LargePersistentCargoStackCanWarn) {
  CargoObstacleTrackerConfig config;
  config.static_cargo_min_raw_equivalent_points = 600U;
  CargoObstacleTracker tracker(config);
  CargoObstacleObservation observation = staticCargo(0U, 0.0F, 0.0F);
  CargoObstacleTrackerDecision decision;
  for (int i = 0; i < 8; ++i) {
    observation.centroid_map.x() = 0.002F * static_cast<float>(i);
    decision = tracker.update(1.0 + 0.2 * i, {observation});
  }
  EXPECT_TRUE(decision.confirmed_hazard) << decision.reason;
  EXPECT_TRUE(decision.selected_track_static);
}

TEST(CargoObstacleTracker, StaticCargoRequiresIndependentProvenance) {
  CargoObstacleTracker tracker;
  CargoObstacleObservation observation = staticCargo(0U, 0.0F, 0.0F);
  observation.independent_external_provenance = false;
  CargoObstacleTrackerDecision decision;
  for (int i = 0; i < 12; ++i) {
    decision = tracker.update(1.0 + 0.2 * i, {observation});
  }
  EXPECT_FALSE(decision.confirmed_hazard);
  ASSERT_FALSE(tracker.tracks().empty());
  EXPECT_FALSE(tracker.tracks().front().static_obstacle);
}

}  // namespace
}  // namespace ndt_slam
