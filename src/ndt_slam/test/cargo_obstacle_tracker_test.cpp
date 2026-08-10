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
  observation.occupied_map_cells = {1, 2, 3, 4, 5, 6};
  observation.cargo_center_valid = true;
  observation.provenance = ExternalProvenance::STATIC_MAP_MATCH;
  return observation;
}

CargoObstacleObservation directionalPretrack(
    std::size_t source, float x, float y) {
  CargoObstacleObservation observation = hazard(source, x, y, 18U);
  observation.footprint_distance_m = 6.0F;
  observation.warning_code = 14U;
  observation.warning_eligible = false;
  return observation;
}

CargoObstacleTrackerConfig ordinaryHazardConfig() {
  CargoObstacleTrackerConfig config;
  config.require_static_cargo_for_warning = false;
  config.require_far_field_history_for_warnings = false;
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

TEST(CargoObstacleTracker,
     AdjacentCellFragmentRetainsObstacleIdentityAcrossCentroidShift) {
  CargoObstacleTracker tracker(ordinaryHazardConfig());
  CargoObstacleObservation first = hazard(0U, 0.0F, 0.0F);
  first.occupied_map_cells = {100};
  const CargoObstacleTrackerDecision initial =
      tracker.update(1.0, {first});
  ASSERT_NE(initial.selected_track_id, 0U);

  CargoObstacleObservation adjacent = hazard(0U, 1.0F, 0.0F);
  adjacent.occupied_map_cells = {101};
  const CargoObstacleTrackerDecision associated =
      tracker.update(1.2, {adjacent});
  EXPECT_EQ(associated.selected_track_id, initial.selected_track_id);
  EXPECT_GT(associated.selected_track_neighbor_cell_overlap, 0.0F);

  CargoObstacleObservation implausibly_far = hazard(0U, 3.0F, 0.0F);
  implausibly_far.occupied_map_cells = {102};
  const CargoObstacleTrackerDecision reset =
      tracker.update(1.4, {implausibly_far});
  EXPECT_NE(reset.selected_track_id, initial.selected_track_id);
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

TEST(CargoObstacleTracker,
     DirectionalPretrackMaturesIdentityWithoutPublishingWarning) {
  CargoObstacleTracker tracker(ordinaryHazardConfig());
  EXPECT_FALSE(tracker.update(
      1.0, {directionalPretrack(10U, 0.0F, 0.0F)})
                   .hazard_observed);
  EXPECT_FALSE(tracker.update(
      1.2, {directionalPretrack(10U, 0.02F, 0.0F)})
                   .confirmed_hazard);
  const CargoObstacleTrackerDecision acquired = tracker.update(
      1.4, {directionalPretrack(10U, 0.04F, 0.0F)});
  EXPECT_FALSE(acquired.hazard_observed);
  EXPECT_FALSE(acquired.confirmed_hazard);
  EXPECT_EQ(acquired.reason, "directional_collision_track_acquiring");

  CargoObstacleObservation level2 = hazard(3U, 0.06F, 0.0F, 18U);
  const CargoObstacleTrackerDecision warned =
      tracker.update(1.6, {level2});
  EXPECT_TRUE(warned.confirmed_hazard) << warned.reason;
  EXPECT_EQ(warned.warning_code, 18U);
  EXPECT_GE(warned.selected_confirm_count, 3);
}

TEST(CargoObstacleTracker,
     MaturePretrackDoesNotDelayAccurateLevel1AtThreeMeters) {
  CargoObstacleTrackerConfig config = ordinaryHazardConfig();
  config.require_far_field_history_for_warnings = true;
  CargoObstacleTracker tracker(config);
  tracker.update(1.0, {directionalPretrack(10U, 0.0F, 0.0F)});
  tracker.update(1.2, {directionalPretrack(10U, 0.02F, 0.0F)});
  tracker.update(1.4, {directionalPretrack(10U, 0.04F, 0.0F)});

  CargoObstacleObservation level1 = hazard(3U, 0.06F, 0.0F, 17U);
  level1.footprint_distance_m = 2.9F;
  const CargoObstacleTrackerDecision warned =
      tracker.update(1.6, {level1});
  EXPECT_TRUE(warned.confirmed_hazard) << warned.reason;
  EXPECT_EQ(warned.warning_code, 17U);
}

TEST(CargoObstacleTracker,
     Level1FirstSeenInsideThreeMetersWaitsForFarFieldHistory) {
  CargoObstacleTrackerConfig config = ordinaryHazardConfig();
  config.require_far_field_history_for_warnings = true;
  CargoObstacleTracker tracker(config);
  CargoObstacleObservation near = hazard(0U, 0.0F, 0.0F, 17U);
  near.footprint_distance_m = 2.0F;
  tracker.update(1.0, {near});
  tracker.update(1.2, {near});
  const CargoObstacleTrackerDecision suppressed =
      tracker.update(1.4, {near});
  EXPECT_FALSE(suppressed.confirmed_hazard);
  EXPECT_TRUE(suppressed.selected_near_field);
  EXPECT_FALSE(suppressed.selected_near_field_authorized);
  EXPECT_FALSE(suppressed.selected_far_field_history_valid);
  EXPECT_EQ(suppressed.reason, "warning_track_missing_true_far_history");

  CargoObstacleObservation far = near;
  far.footprint_distance_m = 6.0F;
  far.warning_code = 14U;
  far.warning_eligible = false;
  tracker.update(1.6, {far});
  tracker.update(1.8, {far});
  ASSERT_TRUE(tracker.update(2.0, {far}).confirmed_hazard);
  const CargoObstacleTrackerDecision authorized =
      tracker.update(2.2, {near});
  EXPECT_TRUE(authorized.confirmed_hazard) << authorized.reason;
  EXPECT_TRUE(authorized.selected_near_field_authorized);
  EXPECT_TRUE(authorized.selected_far_field_history_valid);
  EXPECT_EQ(authorized.warning_code, 17U);
}

TEST(CargoObstacleTracker,
     StrictSixFrameHalfSecondFarHistoryRemainsAConfigurableTestCase) {
  CargoObstacleTrackerConfig config = ordinaryHazardConfig();
  config.require_far_field_history_for_warnings = true;
  config.far_history_confirm_frames = 6;
  config.far_history_confirm_duration_sec = 0.5;
  CargoObstacleTracker tracker(config);
  CargoObstacleTrackerDecision decision;
  for (int index = 0; index < 5; ++index) {
    decision = tracker.update(
        1.0 + 0.1 * index,
        {directionalPretrack(10U, 0.01F * index, 0.0F)});
    EXPECT_FALSE(decision.selected_far_field_history_valid);
  }
  decision = tracker.update(
      1.5, {directionalPretrack(10U, 0.05F, 0.0F)});
  EXPECT_TRUE(decision.selected_far_field_history_valid);
}

TEST(CargoObstacleTracker, FarHistoryUsesDistanceMinusUncertainty) {
  CargoObstacleTrackerConfig config = ordinaryHazardConfig();
  config.require_far_field_history_for_warnings = true;
  CargoObstacleTracker tracker(config);
  CargoObstacleObservation uncertain = directionalPretrack(10U, 0.0F, 0.0F);
  uncertain.footprint_distance_m = 5.2F;
  uncertain.horizontal_uncertainty_m = 0.25F;
  tracker.update(1.0, {uncertain});
  tracker.update(1.2, {uncertain});
  const auto rejected = tracker.update(1.4, {uncertain});
  EXPECT_FALSE(rejected.selected_far_field_history_valid);

  CargoObstacleObservation safe = uncertain;
  safe.footprint_distance_m = 5.4F;
  safe.horizontal_uncertainty_m = 0.20F;
  tracker.update(1.6, {safe});
  tracker.update(1.8, {safe});
  const auto acquired = tracker.update(2.0, {safe});
  EXPECT_TRUE(acquired.selected_far_field_history_valid);
}

TEST(CargoObstacleTracker,
     StaticDirectionalPretrackAuthorizesRealLevel2AtFiveMeters) {
  CargoObstacleTracker tracker;
  CargoObstacleObservation pretrack =
      staticCargo(10U, 0.0F, 0.0F, 18U);
  pretrack.footprint_distance_m = 6.0F;
  pretrack.warning_code = 14U;
  pretrack.warning_eligible = false;
  CargoObstacleTrackerDecision acquired;
  for (int frame = 0; frame < 8; ++frame) {
    pretrack.centroid_map.x() =
        0.002F * static_cast<float>(frame);
    acquired = tracker.update(
        1.0 + 0.2 * frame, {pretrack});
    EXPECT_FALSE(acquired.hazard_observed);
    EXPECT_FALSE(acquired.confirmed_hazard);
  }
  ASSERT_FALSE(tracker.tracks().empty());
  EXPECT_TRUE(tracker.tracks().front().static_obstacle);

  CargoObstacleObservation level2 =
      staticCargo(2U, 0.016F, 0.0F, 18U);
  level2.footprint_distance_m = 4.9F;
  const CargoObstacleTrackerDecision warned =
      tracker.update(2.6, {level2});
  EXPECT_TRUE(warned.confirmed_hazard) << warned.reason;
  EXPECT_EQ(warned.warning_code, 18U);
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

TEST(CargoObstacleTracker,
     TrackBornEmbeddedNeedsSeparatedHistoryOrIndependentProvenance) {
  CargoObstacleTracker tracker(ordinaryHazardConfig());
  CargoObstacleObservation embedded = hazard(0U, 0.0F, 0.0F);
  embedded.footprint_distance_m = 0.0F;
  tracker.update(1.0, {embedded});
  tracker.update(1.2, {embedded});
  const CargoObstacleTrackerDecision decision =
      tracker.update(1.4, {embedded});
  EXPECT_FALSE(decision.confirmed_hazard);
  EXPECT_TRUE(decision.selected_embedded);
  EXPECT_FALSE(decision.selected_embedded_authorized);
  EXPECT_EQ(decision.reason, "embedded_obstacle_origin_unresolved");
}

TEST(CargoObstacleTracker,
     PreviouslySeparatedTrackRetainsRealCollisionWarningAtContact) {
  CargoObstacleTracker tracker(ordinaryHazardConfig());
  CargoObstacleObservation approaching = hazard(0U, 0.0F, 0.0F);
  approaching.footprint_distance_m = 2.0F;
  tracker.update(1.0, {approaching});
  tracker.update(1.2, {approaching});
  ASSERT_TRUE(tracker.update(1.4, {approaching}).confirmed_hazard);

  CargoObstacleObservation contact = approaching;
  contact.footprint_distance_m = 0.0F;
  const CargoObstacleTrackerDecision decision =
      tracker.update(1.6, {contact});
  EXPECT_TRUE(decision.confirmed_hazard) << decision.reason;
  EXPECT_TRUE(decision.selected_embedded);
  EXPECT_TRUE(decision.selected_embedded_authorized);
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

TEST(CargoObstacleTracker,
     PendingLargeGeometryMustBeConsecutiveBeforeWarning) {
  CargoObstacleTrackerConfig config = ordinaryHazardConfig();
  config.confirm_frames = 3;
  config.require_large_geometry_for_warning = true;
  CargoObstacleTracker tracker(config);
  CargoObstacleObservation large = staticCargo(0U, 0.0F, 0.0F);
  large.provenance =
      ExternalProvenance::OUTSIDE_CARGO_SHELL_ONLY;
  EXPECT_FALSE(tracker.update(1.0, {large}).confirmed_hazard);
  EXPECT_FALSE(tracker.update(1.2, {large}).confirmed_hazard);

  CargoObstacleObservation incomplete = hazard(0U, 0.0F, 0.0F);
  const auto rejected = tracker.update(1.4, {incomplete});
  EXPECT_FALSE(rejected.confirmed_hazard);
  EXPECT_EQ(rejected.selected_geometry_confirm_count, 0);

  EXPECT_FALSE(tracker.update(1.6, {large}).confirmed_hazard);
  EXPECT_FALSE(tracker.update(1.8, {large}).confirmed_hazard);
  const auto confirmed = tracker.update(2.0, {large});
  EXPECT_TRUE(confirmed.confirmed_hazard);
  EXPECT_EQ(confirmed.selected_geometry_confirm_count, 3);
}

TEST(CargoObstacleTracker, TwentyPointTrackCannotBecomeStaticCargo) {
  CargoObstacleTracker tracker;
  CargoObstacleObservation observation = hazard(0U, 0.0F, 0.0F);
  observation.provenance = ExternalProvenance::STATIC_MAP_MATCH;
  for (int i = 0; i < 12; ++i) {
    const CargoObstacleTrackerDecision decision = tracker.update(
        1.0 + 0.2 * i, {observation});
    EXPECT_FALSE(decision.confirmed_hazard);
  }
  ASSERT_FALSE(tracker.tracks().empty());
  EXPECT_FALSE(tracker.tracks().front().static_obstacle);
}

TEST(CargoObstacleTracker,
     RuntimeStaticMaturityCannotReplaceLiveFarHistory) {
  CargoObstacleTrackerConfig config;
  config.static_cargo_min_raw_equivalent_points = 600U;
  CargoObstacleTracker tracker(config);
  CargoObstacleObservation observation = staticCargo(0U, 0.0F, 0.0F);
  CargoObstacleTrackerDecision decision;
  for (int i = 0; i < 8; ++i) {
    observation.centroid_map.x() = 0.002F * static_cast<float>(i);
    decision = tracker.update(1.0 + 0.2 * i, {observation});
  }
  EXPECT_FALSE(decision.confirmed_hazard);
  EXPECT_TRUE(decision.selected_track_static);
  EXPECT_FALSE(decision.selected_certified_static_provenance);
}

TEST(CargoObstacleTracker, StaticCargoRequiresIndependentProvenance) {
  CargoObstacleTracker tracker;
  CargoObstacleObservation observation = staticCargo(0U, 0.0F, 0.0F);
  observation.provenance =
      ExternalProvenance::OUTSIDE_CARGO_SHELL_ONLY;
  CargoObstacleTrackerDecision decision;
  for (int i = 0; i < 12; ++i) {
    decision = tracker.update(1.0 + 0.2 * i, {observation});
  }
  EXPECT_FALSE(decision.confirmed_hazard);
  ASSERT_FALSE(tracker.tracks().empty());
  EXPECT_FALSE(tracker.tracks().front().static_obstacle);
}

TEST(CargoObstacleTracker, KnownStaticNeedsOnlyThreeFreshConfirmations) {
  CargoObstacleTrackerConfig config;
  config.static_cargo_confirm_frames = 3;
  config.static_cargo_confirm_sec = 1.0;
  CargoObstacleTracker tracker(config);
  CargoObstacleObservation observation = staticCargo(0U, 0.0F, 0.0F);
  observation.provenance =
      ExternalProvenance::OUTSIDE_CARGO_SHELL_ONLY;
  tracker.update(1.0, {observation});
  tracker.update(1.4, {observation});

  observation.provenance = ExternalProvenance::STATIC_MAP_MATCH;
  observation.certified_static_provenance = true;
  EXPECT_FALSE(tracker.update(2.0, {observation}).confirmed_hazard);
  EXPECT_FALSE(tracker.update(2.2, {observation}).confirmed_hazard);
  EXPECT_TRUE(tracker.update(2.4, {observation}).confirmed_hazard);
}

TEST(CargoObstacleTracker,
     KnownStaticLevel1ConfirmsTrackButReportsMissingFarFieldHistory) {
  CargoObstacleTracker tracker;
  CargoObstacleObservation observation =
      staticCargo(0U, 0.0F, 0.0F, 17U);
  observation.certified_static_provenance = true;
  observation.footprint_distance_m = 2.99F;
  EXPECT_FALSE(tracker.update(1.0, {observation}).confirmed_hazard);
  EXPECT_FALSE(tracker.update(1.2, {observation}).confirmed_hazard);
  const CargoObstacleTrackerDecision decision =
      tracker.update(1.4, {observation});
  EXPECT_TRUE(decision.confirmed_hazard) << decision.reason;
  EXPECT_EQ(decision.warning_code, 17U);
  EXPECT_TRUE(decision.selected_near_field_authorized);
  EXPECT_FALSE(decision.selected_far_field_history_valid);
}

TEST(CargoObstacleTracker, OutsideCargoShellAloneIsNotIndependentProvenance) {
  CargoObstacleTracker tracker;
  CargoObstacleObservation observation = staticCargo(0U, 0.0F, 0.0F);
  observation.provenance =
      ExternalProvenance::OUTSIDE_CARGO_SHELL_ONLY;
  observation.cargo_center_map.setZero();
  CargoObstacleTrackerDecision decision;
  for (int i = 0; i < 12; ++i) {
    decision = tracker.update(1.0 + 0.2 * i, {observation});
  }
  EXPECT_FALSE(decision.confirmed_hazard);
  EXPECT_EQ(decision.reason, "static_provenance_unavailable");
  EXPECT_EQ(decision.selected_provenance,
            ExternalProvenance::OUTSIDE_CARGO_SHELL_ONLY);
}

TEST(CargoObstacleTracker,
     CargoMovesAwayPersistenceDoesNotReplaceCertifiedAuthority) {
  CargoObstacleTrackerConfig config;
  config.static_cargo_confirm_frames = 3;
  config.static_cargo_confirm_sec = 0.4;
  config.static_provenance_min_cargo_motion_m = 0.30F;
  CargoObstacleTracker tracker(config);
  CargoObstacleObservation observation = staticCargo(0U, 0.0F, 0.0F);
  observation.provenance =
      ExternalProvenance::OUTSIDE_CARGO_SHELL_ONLY;
  CargoObstacleTrackerDecision decision;
  for (int i = 0; i < 8; ++i) {
    observation.cargo_center_map =
        Eigen::Vector2f(0.12F * static_cast<float>(i), 0.0F);
    decision = tracker.update(1.0 + 0.2 * i, {observation});
  }
  EXPECT_FALSE(decision.confirmed_hazard);
  EXPECT_EQ(decision.selected_provenance,
            ExternalProvenance::CARGO_MOVED_AWAY_PERSISTENCE);
}

TEST(CargoObstacleTracker, CellOverlapPreservesStaticTrackIdentity) {
  CargoObstacleTrackerConfig config;
  config.association_max_centroid_distance_m = 0.20F;
  CargoObstacleTracker tracker(config);
  CargoObstacleObservation first = staticCargo(0U, 0.0F, 0.0F);
  const CargoObstacleTrackerDecision initial = tracker.update(1.0, {first});
  CargoObstacleObservation shifted = first;
  shifted.centroid_map.x() = 0.55F;
  shifted.occupied_map_cells = {2, 3, 4, 5, 6, 7};
  const CargoObstacleTrackerDecision associated =
      tracker.update(1.2, {shifted});
  EXPECT_EQ(associated.selected_track_id, initial.selected_track_id);
  ASSERT_EQ(tracker.tracks().size(), 1U);
  EXPECT_GT(associated.selected_track_cell_overlap, 0.70F);
}

TEST(CargoObstacleTracker, TrackCreationAndAssociationResetAreCounted) {
  CargoObstacleTracker tracker(ordinaryHazardConfig());
  const auto first = tracker.update(1.0, {hazard(0U, 0.0F, 0.0F)});
  EXPECT_EQ(first.created_track_count, 1U);
  EXPECT_EQ(first.association_reset_count, 0U);

  const auto jumped = tracker.update(1.2, {hazard(0U, 5.0F, 0.0F)});
  EXPECT_EQ(jumped.created_track_count, 2U);
  EXPECT_EQ(jumped.association_reset_count, 1U);
}

}  // namespace
}  // namespace ndt_slam
