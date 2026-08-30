#include <gtest/gtest.h>

#include "ndt_slam/cargo_obstacle_tracker.hpp"
#include "ndt_slam/frame_authority_context.hpp"

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
  PoseAuthorityIdentity identity;
  identity.map_rebuild_generation = 1U;
  identity.keyframe_pose_version = 2U;
  identity.yaw_authority_generation = 3U;
  identity.map_frame_uuid = "map-frame";
  identity.yaw_reference_hash = "yaw-reference";
  identity.target_snapshot_id = 4U;
  observation.pose_authority = bindTemporalEvidenceAuthority(identity, 1.0);
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

CargoObstacleObservation withKeyframePoseVersion(
    CargoObstacleObservation observation, std::uint64_t version,
    double stamp_sec) {
  observation.pose_authority.pose_identity.keyframe_pose_version = version;
  observation.pose_authority.source_stamp_sec = stamp_sec;
  return observation;
}

void expectCrossAuthorityUniqueObservationStartsFreshEvidence(
    const PoseAuthorityIdentity& next_identity) {
  CargoObstacleTracker tracker(ordinaryHazardConfig());
  CargoObstacleObservation observation = hazard(0U, 0.0F, 0.0F);
  tracker.update(1.0, {observation});
  observation.source_index = 1U;
  observation.pose_authority.source_stamp_sec = 1.2;
  tracker.update(1.2, {observation});
  observation.source_index = 2U;
  observation.pose_authority.source_stamp_sec = 1.4;
  const CargoObstacleTrackerDecision mature =
      tracker.update(1.4, {observation});
  ASSERT_TRUE(mature.confirmed_hazard);
  ASSERT_EQ(mature.selected_confirm_count, 3);
  const std::uint64_t physical_track_id = mature.selected_track_id;

  CargoObstacleObservation next = hazard(3U, 0.02F, 0.0F);
  next.pose_authority = bindTemporalEvidenceAuthority(next_identity, 1.6);
  const CargoObstacleTrackerDecision fresh = tracker.update(1.6, {next});

  EXPECT_FALSE(fresh.confirmed_hazard);
  EXPECT_EQ(fresh.selected_track_id, physical_track_id);
  EXPECT_EQ(fresh.selected_confirm_count, 1);
  ASSERT_TRUE(fresh.selected_pose_authority.valid);
  EXPECT_TRUE(samePoseAuthorityIdentity(
      fresh.selected_pose_authority.pose_identity, next_identity));
  ASSERT_EQ(tracker.tracks().size(), 1U);
}

TEST(CargoObstacleTracker,
     AmbiguousTemporalHoldPreservesProducingPoseAuthority) {
  CargoObstacleTracker tracker(ordinaryHazardConfig());
  CargoObstacleObservation first = hazard(0U, -0.4F, 0.0F);
  CargoObstacleObservation second = hazard(1U, 0.4F, 0.0F);
  first.pose_authority.source_stamp_sec = 1.0;
  second.pose_authority.source_stamp_sec = 1.0;
  tracker.update(1.0, {first, second});
  CargoObstacleObservation ambiguous_a = hazard(2U, 0.0F, 0.0F);
  CargoObstacleObservation ambiguous_b = hazard(3U, 0.0F, 0.0F);
  ambiguous_a.pose_authority.pose_identity.keyframe_pose_version = 9U;
  ambiguous_b.pose_authority.pose_identity.keyframe_pose_version = 9U;
  ambiguous_a.pose_authority.source_stamp_sec = 1.2;
  ambiguous_b.pose_authority.source_stamp_sec = 1.2;
  const CargoObstacleTrackerDecision decision = tracker.update(
      1.2, {ambiguous_a, ambiguous_b});
  ASSERT_TRUE(decision.selected_pose_authority.valid);
  EXPECT_EQ(
      decision.selected_pose_authority.pose_identity.keyframe_pose_version,
      2U);
}

TEST(CargoObstacleTracker, SameAuthorityUniqueObservationContinuesTrack) {
  CargoObstacleTracker tracker(ordinaryHazardConfig());
  const CargoObstacleTrackerDecision initial =
      tracker.update(1.0, {hazard(0U, 0.0F, 0.0F)});
  CargoObstacleObservation continued = hazard(1U, 0.02F, 0.0F);
  continued.pose_authority.source_stamp_sec = 1.2;
  const CargoObstacleTrackerDecision decision =
      tracker.update(1.2, {continued});

  EXPECT_EQ(decision.selected_track_id, initial.selected_track_id);
  EXPECT_EQ(decision.selected_confirm_count, 2);
  ASSERT_EQ(tracker.tracks().size(), 1U);
}

TEST(CargoObstacleTracker, SameAuthorityAmbiguityStillHolds) {
  CargoObstacleTracker tracker(ordinaryHazardConfig());
  tracker.update(1.0, {
      hazard(0U, -0.4F, 0.0F), hazard(1U, 0.4F, 0.0F)});
  CargoObstacleObservation ambiguous_a = hazard(2U, 0.0F, 0.0F);
  CargoObstacleObservation ambiguous_b = hazard(3U, 0.0F, 0.0F);
  ambiguous_a.pose_authority.source_stamp_sec = 1.2;
  ambiguous_b.pose_authority.source_stamp_sec = 1.2;
  const CargoObstacleTrackerDecision decision = tracker.update(
      1.2, {ambiguous_a, ambiguous_b});

  EXPECT_FALSE(decision.confirmed_hazard);
  EXPECT_EQ(decision.reason,
            "ambiguous_obstacle_association_authority_frozen");
  EXPECT_EQ(tracker.tracks().size(), 2U);
  for (const CargoObstacleTrack& track : tracker.tracks()) {
    EXPECT_TRUE(track.association_ambiguous);
    EXPECT_EQ(track.pose_authority.pose_identity.keyframe_pose_version, 2U);
  }
}

TEST(CargoObstacleTracker,
     CrossAuthorityUniqueObservationStartsFreshEvidence) {
  PoseAuthorityIdentity next =
      hazard(9U, 0.0F, 0.0F).pose_authority.pose_identity;
  next.keyframe_pose_version = 9U;
  expectCrossAuthorityUniqueObservationStartsFreshEvidence(next);
}

TEST(CargoObstacleTracker,
     CrossAuthorityAmbiguousObservationCannotCreateFreshAuthoritativeTrack) {
  CargoObstacleTracker tracker(ordinaryHazardConfig());
  tracker.update(1.0, {
      hazard(0U, -0.4F, 0.0F), hazard(1U, 0.4F, 0.0F)});
  CargoObstacleObservation ambiguous_a = withKeyframePoseVersion(
      hazard(2U, 0.0F, 0.0F), 9U, 1.2);
  CargoObstacleObservation ambiguous_b = withKeyframePoseVersion(
      hazard(3U, 0.0F, 0.0F), 9U, 1.2);
  const CargoObstacleTrackerDecision decision = tracker.update(
      1.2, {ambiguous_a, ambiguous_b});

  EXPECT_FALSE(decision.confirmed_hazard);
  EXPECT_EQ(decision.reason,
            "ambiguous_obstacle_association_authority_frozen");
  ASSERT_EQ(tracker.tracks().size(), 2U);
  for (const CargoObstacleTrack& track : tracker.tracks()) {
    EXPECT_TRUE(track.association_ambiguous);
    EXPECT_EQ(track.pose_authority.pose_identity.keyframe_pose_version, 2U);
  }
}

TEST(CargoObstacleTracker,
     HeldOldAuthorityCannotAuthorizeCurrentFrameClear) {
  CargoObstacleTracker tracker(ordinaryHazardConfig());
  tracker.update(1.0, {
      hazard(0U, -0.4F, 0.0F), hazard(1U, 0.4F, 0.0F)});
  CargoObstacleObservation ambiguous_a = withKeyframePoseVersion(
      hazard(2U, 0.0F, 0.0F), 9U, 1.2);
  CargoObstacleObservation ambiguous_b = withKeyframePoseVersion(
      hazard(3U, 0.0F, 0.0F), 9U, 1.2);
  const CargoObstacleTrackerDecision held = tracker.update(
      1.2, {ambiguous_a, ambiguous_b});
  ASSERT_TRUE(held.selected_pose_authority.valid);
  ASSERT_EQ(
      held.selected_pose_authority.pose_identity.keyframe_pose_version, 2U);

  FrameAuthorityContext frame;
  frame.rail_authority_mode = true;
  frame.pose_identity = ambiguous_a.pose_authority.pose_identity;
  frame.localization_health.safety_localization_authorized = true;
  const TemporalEvidenceAuthority current_cargo =
      bindTemporalEvidenceAuthority(frame.pose_identity, 1.2);

  EXPECT_FALSE(safetyFrameAuthorityMatches(
      frame, current_cargo, held.selected_pose_authority));
}

TEST(CargoObstacleTracker,
     MapGenerationMismatchCannotTransferTemporalEvidence) {
  PoseAuthorityIdentity next =
      hazard(9U, 0.0F, 0.0F).pose_authority.pose_identity;
  ++next.map_rebuild_generation;
  expectCrossAuthorityUniqueObservationStartsFreshEvidence(next);
}

TEST(CargoObstacleTracker,
     YawAuthorityGenerationMismatchCannotTransferTemporalEvidence) {
  PoseAuthorityIdentity next =
      hazard(9U, 0.0F, 0.0F).pose_authority.pose_identity;
  ++next.yaw_authority_generation;
  expectCrossAuthorityUniqueObservationStartsFreshEvidence(next);
}

TEST(CargoObstacleTracker,
     YawReferenceMismatchCannotTransferTemporalEvidence) {
  PoseAuthorityIdentity next =
      hazard(9U, 0.0F, 0.0F).pose_authority.pose_identity;
  next.yaw_reference_hash = "different-yaw-reference";
  expectCrossAuthorityUniqueObservationStartsFreshEvidence(next);
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

TEST(CargoObstacleTracker,
     GlobalAssociationIsInvariantToObservationInputOrder) {
  CargoObstacleTracker forward(ordinaryHazardConfig());
  CargoObstacleTracker reversed(ordinaryHazardConfig());
  const std::vector<CargoObstacleObservation> initial = {
      hazard(10U, 0.0F, 0.0F), hazard(20U, 1.0F, 0.0F)};
  forward.update(1.0, initial);
  reversed.update(1.0, initial);

  // "flexible" can match either track; "left_only" can match only track 1.
  // Per-observation greedy creates an extra identity when flexible is first.
  const CargoObstacleObservation flexible = hazard(21U, 0.40F, 0.0F);
  const CargoObstacleObservation left_only = hazard(11U, -0.30F, 0.0F);
  forward.update(1.2, {flexible, left_only});
  reversed.update(1.2, {left_only, flexible});

  ASSERT_EQ(forward.tracks().size(), reversed.tracks().size());
  ASSERT_EQ(forward.tracks().size(), 2U);
  for (std::size_t index = 0U; index < forward.tracks().size(); ++index) {
    const CargoObstacleTrack& first = forward.tracks()[index];
    const CargoObstacleTrack& second = reversed.tracks()[index];
    EXPECT_EQ(first.track_id, second.track_id);
    EXPECT_EQ(first.current_source_index, second.current_source_index);
    EXPECT_FLOAT_EQ(first.centroid_map.x(), second.centroid_map.x());
    EXPECT_FLOAT_EQ(first.centroid_map.y(), second.centroid_map.y());
  }
}

TEST(CargoObstacleTracker,
     CentroidZShiftDoesNotDuplicateIndependentTopGate) {
  CargoObstacleTracker tracker(ordinaryHazardConfig());
  CargoObstacleObservation lower_visible = hazard(10U, 0.0F, 0.0F);
  lower_visible.centroid_map.z() = 0.20F;
  const auto initial = tracker.update(1.0, {lower_visible});
  ASSERT_NE(initial.selected_track_id, 0U);

  CargoObstacleObservation upper_visible = hazard(11U, 0.0F, 0.0F);
  upper_visible.centroid_map.z() = 1.80F;
  const auto associated = tracker.update(1.2, {upper_visible});
  EXPECT_EQ(associated.selected_track_id, initial.selected_track_id);
  EXPECT_EQ(associated.selected_confirm_count, 2);
}

TEST(CargoObstacleTracker,
     ConfirmedFarHistoryTrackRetainsSideCollisionAuthority) {
  CargoObstacleTrackerConfig config = ordinaryHazardConfig();
  config.require_far_field_history_for_warnings = true;
  CargoObstacleTracker tracker(config);
  CargoObstacleObservation far = directionalPretrack(10U, 0.0F, 2.0F);
  tracker.update(1.0, {far});
  far.centroid_map.x() = 0.02F;
  tracker.update(1.1, {far});
  far.centroid_map.x() = 0.04F;
  ASSERT_TRUE(tracker.update(1.2, {far})
                  .selected_far_field_history_valid);

  CargoObstacleObservation side_hazard = hazard(11U, 0.06F, 2.0F, 18U);
  side_hazard.footprint_distance_m = 4.0F;
  const CargoObstacleTrackerDecision warning =
      tracker.update(1.3, {side_hazard});
  EXPECT_TRUE(warning.confirmed_hazard) << warning.reason;
  EXPECT_TRUE(warning.selected_far_field_history_valid);
  EXPECT_EQ(warning.warning_code, 18U);
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
  implausibly_far.occupied_map_cells = {500};
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
  ASSERT_TRUE(tracker.update(2.0, {far}).selected_far_field_history_valid);
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
  CargoObstacleTrackerConfig config;
  config.require_far_field_history_for_warnings = false;
  CargoObstacleTracker tracker(config);
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

TEST(CargoObstacleTracker,
     PartialBottomVisibilityDoesNotSplitSameXyAndTopIdentity) {
  CargoObstacleTracker tracker(ordinaryHazardConfig());
  CargoObstacleObservation full = hazard(0U, 0.0F, 0.0F);
  full.top_z95_map = 2.0F;
  full.bottom_z05_map = 0.2F;
  const CargoObstacleTrackerDecision initial = tracker.update(1.0, {full});

  CargoObstacleObservation partial = full;
  partial.source_index = 1U;
  partial.centroid_map.x() = 0.02F;
  partial.bottom_z05_map = 1.4F;
  const CargoObstacleTrackerDecision continued =
      tracker.update(1.2, {partial});

  // Bottom-Z is visibility-dependent for the overhead LiDAR and is not an
  // identity gate. XY/cell and top-Z identify the same conservative hazard;
  // a partial lower edge must not churn the physical track or lose its true
  // far-history. Clearance authority still uses the current evaluated top.
  EXPECT_EQ(continued.selected_track_id, initial.selected_track_id);
  ASSERT_EQ(tracker.tracks().size(), 1U);
  EXPECT_FLOAT_EQ(tracker.tracks().front().top_z95_map, 2.0F);
  EXPECT_FLOAT_EQ(tracker.tracks().front().bottom_z05_map, 1.4F);
  EXPECT_EQ(tracker.tracks().front().validated_consecutive_observations, 2);
}

TEST(CargoObstacleTracker,
     DistinctTopAtSameXyCannotInheritMatureFarHistory) {
  CargoObstacleTrackerConfig config = ordinaryHazardConfig();
  config.require_far_field_history_for_warnings = true;
  config.far_history_confirm_frames = 3;
  config.far_history_confirm_duration_sec = 0.2;
  CargoObstacleTracker tracker(config);

  CargoObstacleObservation far = directionalPretrack(0U, 0.0F, 0.0F);
  tracker.update(1.0, {far});
  far.source_index = 1U;
  tracker.update(1.1, {far});
  far.source_index = 2U;
  const CargoObstacleTrackerDecision mature = tracker.update(1.2, {far});
  ASSERT_TRUE(mature.selected_far_field_history_valid);
  const std::uint64_t mature_track_id = mature.selected_track_id;

  CargoObstacleObservation vertically_distinct =
      hazard(3U, 0.01F, 0.0F, 18U);
  vertically_distinct.top_z95_map =
      far.top_z95_map + config.association_max_top_step_m + 0.10F;
  vertically_distinct.bottom_z05_map = 1.4F;
  const CargoObstacleTrackerDecision decision =
      tracker.update(1.3, {vertically_distinct});

  EXPECT_FALSE(decision.confirmed_hazard);
  EXPECT_FALSE(decision.selected_far_field_history_valid);
  EXPECT_NE(decision.selected_track_id, mature_track_id);
  EXPECT_EQ(tracker.tracks().size(), 2U);
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

TEST(CargoObstacleTracker,
     HeightInvalidObservationBuildsIdentityButNeverWarningAuthority) {
  CargoObstacleTrackerConfig config = ordinaryHazardConfig();
  config.require_far_field_history_for_warnings = true;
  config.far_history_confirm_frames = 3;
  config.far_history_confirm_duration_sec = 0.2;
  CargoObstacleTracker tracker(config);
  CargoObstacleObservation physical = directionalPretrack(7U, 0.0F, 0.0F);
  physical.hazard_geometry_valid = false;
  physical.conservative_clearance_m =
      std::numeric_limits<float>::quiet_NaN();

  EXPECT_FALSE(tracker.update(1.0, {physical}).hazard_observed);
  EXPECT_FALSE(tracker.update(1.1, {physical}).confirmed_hazard);
  const CargoObstacleTrackerDecision acquired =
      tracker.update(1.2, {physical});
  EXPECT_FALSE(acquired.hazard_observed);
  EXPECT_FALSE(acquired.confirmed_hazard);
  EXPECT_TRUE(acquired.selected_far_field_history_valid);
  ASSERT_EQ(tracker.tracks().size(), 1U);
  EXPECT_FALSE(tracker.tracks().front().current_hazard_geometry_valid);

  CargoObstacleObservation near = hazard(8U, 0.02F, 0.0F, 18U);
  const CargoObstacleTrackerDecision promoted =
      tracker.update(1.3, {near});
  EXPECT_TRUE(promoted.confirmed_hazard) << promoted.reason;
  EXPECT_EQ(promoted.selected_track_id, acquired.selected_track_id);
}

TEST(CargoObstacleTracker,
     InvalidConfigIsRetainedAndCannotSilentlyUseDefaults) {
  CargoObstacleTrackerConfig config;
  config.confirm_frames = 1;
  config.association_neighbor_cell_radius = 4;
  config.far_history_confirm_duration_sec = -0.1;
  config.acquisition_distance_m = 4.0F;
  CargoObstacleTracker tracker;
  const CargoConfigValidationResult validation = tracker.setConfig(config);
  EXPECT_FALSE(validation.valid);
  EXPECT_NE(validation.summary().find("confirm_frames"), std::string::npos);
  EXPECT_NE(validation.summary().find("association_neighbor_cell_radius"),
            std::string::npos);
  EXPECT_NE(validation.summary().find("far_history_confirm_duration_sec"),
            std::string::npos);
  EXPECT_NE(validation.summary().find("acquisition_distance_m"),
            std::string::npos);
  EXPECT_EQ(tracker.config().confirm_frames, 1);
  EXPECT_EQ(tracker.config().association_neighbor_cell_radius, 4);
  const CargoObstacleTrackerDecision decision =
      tracker.update(1.0, {hazard(0U, 0.0F, 0.0F)});
  EXPECT_FALSE(decision.valid);
  EXPECT_NE(decision.reason.find("invalid_obstacle_tracker_config"),
            std::string::npos);
}

TEST(CargoObstacleTracker,
     TwoByTwoEqualCostAssociationFreezesAllAuthorityMaturity) {
  CargoObstacleTracker tracker(ordinaryHazardConfig());
  tracker.update(1.0, {
      hazard(10U, -0.50F, 0.0F), hazard(20U, 0.50F, 0.0F)});
  tracker.update(1.2, {
      hazard(11U, -0.48F, 0.0F), hazard(21U, 0.48F, 0.0F)});
  ASSERT_EQ(tracker.tracks().size(), 2U);
  EXPECT_EQ(tracker.tracks()[0].validated_consecutive_observations, 2);
  EXPECT_EQ(tracker.tracks()[1].validated_consecutive_observations, 2);

  // Both observations are equidistant from both existing tracks. Canonical
  // global pairing may retain a deterministic physical projection, but must
  // not transfer confirmation/provenance/far-history authority.
  CargoObstacleObservation ambiguous_a = hazard(30U, 0.0F, 0.0F);
  CargoObstacleObservation ambiguous_b = hazard(31U, 0.0F, 0.0F);
  const CargoObstacleTrackerDecision ambiguous =
      tracker.update(1.4, {ambiguous_a, ambiguous_b});
  EXPECT_FALSE(ambiguous.confirmed_hazard);
  EXPECT_EQ(ambiguous.reason,
            "ambiguous_obstacle_association_authority_frozen");
  EXPECT_EQ(ambiguous.ambiguous_association_count, 2U);
  ASSERT_EQ(tracker.tracks().size(), 2U);
  for (const CargoObstacleTrack& track : tracker.tracks()) {
    EXPECT_TRUE(track.association_ambiguous);
    EXPECT_FALSE(track.current_source_validated);
    EXPECT_FALSE(track.current_warning_eligible);
    EXPECT_EQ(track.validated_consecutive_observations, 2);
    EXPECT_EQ(track.geometry_validated_consecutive_observations, 0);
  }
}

TEST(CargoObstacleTracker,
     AmbiguousFarSamplesCannotTransferOrAdvanceFarHistory) {
  CargoObstacleTrackerConfig config = ordinaryHazardConfig();
  config.require_far_field_history_for_warnings = true;
  config.far_history_confirm_frames = 3;
  config.far_history_confirm_duration_sec = 0.2;
  CargoObstacleTracker tracker(config);
  CargoObstacleObservation left = directionalPretrack(1U, -0.5F, 0.0F);
  CargoObstacleObservation right = directionalPretrack(2U, 0.5F, 0.0F);
  tracker.update(1.0, {left, right});
  left.source_index = 3U;
  right.source_index = 4U;
  tracker.update(1.1, {left, right});
  ASSERT_EQ(tracker.tracks().size(), 2U);
  for (const CargoObstacleTrack& track : tracker.tracks()) {
    EXPECT_EQ(track.far_field_validated_observations, 2);
    EXPECT_FALSE(track.far_field_history_valid);
  }

  CargoObstacleObservation a = directionalPretrack(5U, 0.0F, 0.0F);
  CargoObstacleObservation b = directionalPretrack(6U, 0.0F, 0.0F);
  tracker.update(1.2, {a, b});
  for (const CargoObstacleTrack& track : tracker.tracks()) {
    EXPECT_EQ(track.far_field_validated_observations, 2);
    EXPECT_FALSE(track.far_field_history_valid);
  }
}

TEST(CargoObstacleTracker,
     AmbiguousAssociationDoesNotMoveAuthoritativePhysicalPredictor) {
  CargoObstacleTrackerConfig config = ordinaryHazardConfig();
  config.require_far_field_history_for_warnings = true;
  config.far_history_confirm_frames = 3;
  config.far_history_confirm_duration_sec = 0.2;
  CargoObstacleTracker tracker(config);
  CargoObstacleObservation left = directionalPretrack(1U, -0.5F, 0.0F);
  CargoObstacleObservation right = directionalPretrack(2U, 0.5F, 0.0F);
  tracker.update(1.0, {left, right});
  left.source_index = 3U;
  right.source_index = 4U;
  tracker.update(1.1, {left, right});
  const auto before = tracker.tracks();
  ASSERT_EQ(before.size(), 2U);

  CargoObstacleObservation ambiguous_a =
      directionalPretrack(5U, 0.0F, 0.0F);
  CargoObstacleObservation ambiguous_b =
      directionalPretrack(6U, 0.0F, 0.0F);
  tracker.update(1.2, {ambiguous_a, ambiguous_b});
  ASSERT_EQ(tracker.tracks().size(), before.size());
  for (std::size_t index = 0U; index < before.size(); ++index) {
    const CargoObstacleTrack& frozen = tracker.tracks()[index];
    EXPECT_TRUE(frozen.centroid_map.isApprox(before[index].centroid_map));
    EXPECT_TRUE(frozen.velocity_map.isApprox(before[index].velocity_map));
    EXPECT_FLOAT_EQ(frozen.top_z95_map, before[index].top_z95_map);
    EXPECT_FLOAT_EQ(frozen.bottom_z05_map, before[index].bottom_z05_map);
    EXPECT_EQ(frozen.occupied_map_cells, before[index].occupied_map_cells);
    EXPECT_DOUBLE_EQ(frozen.last_stamp_sec, before[index].last_stamp_sec);
    EXPECT_EQ(
        frozen.last_observation_cycle,
        before[index].last_observation_cycle);
    EXPECT_EQ(
        frozen.far_field_validated_observations,
        before[index].far_field_validated_observations);
  }

  // A later right-side observation cannot use the ambiguous midpoint as a
  // bridge into the left track's physical identity or maturity.
  CargoObstacleObservation recovered =
      directionalPretrack(7U, 0.52F, 0.0F);
  const CargoObstacleTrackerDecision decision =
      tracker.update(1.3, {recovered});
  EXPECT_EQ(decision.selected_track_id, before[1].track_id);
  EXPECT_FALSE(decision.selected_far_field_history_valid);
}

TEST(PhysicalObstacleTrackStore,
     PendingToFormalChangesAuthorityWithoutDuplicatingHistory) {
  CargoObstacleTrackerConfig config = ordinaryHazardConfig();
  config.require_far_field_history_for_warnings = true;
  config.far_history_confirm_frames = 3;
  config.far_history_confirm_duration_sec = 0.2;
  PhysicalObstacleTrackStore store(config);
  CargoObstacleAuthorityPolicy pending_policy;
  pending_policy.require_static_cargo_for_warning = false;
  pending_policy.require_large_geometry_for_warning = true;
  CargoObstacleAuthorityPolicy formal_policy;
  formal_policy.require_static_cargo_for_warning = false;
  formal_policy.require_large_geometry_for_warning = false;

  CargoObstacleObservation far = directionalPretrack(10U, 0.0F, 0.0F);
  store.update(1.0, {far}, pending_policy);
  far.source_index = 11U;
  store.update(1.1, {far}, pending_policy);
  far.source_index = 12U;
  const auto acquired = store.update(1.2, {far}, pending_policy);
  ASSERT_TRUE(acquired.selected_far_field_history_valid);
  ASSERT_EQ(store.tracks().size(), 1U);
  const std::uint64_t physical_id = acquired.selected_track_id;
  const int far_count = acquired.selected_far_field_observations;

  CargoObstacleObservation near = hazard(13U, 0.02F, 0.0F, 18U);
  const auto formal = store.update(1.3, {near}, formal_policy);
  EXPECT_TRUE(formal.confirmed_hazard) << formal.reason;
  EXPECT_EQ(formal.selected_track_id, physical_id);
  EXPECT_EQ(formal.selected_far_field_observations, far_count);
  EXPECT_EQ(store.tracks().size(), 1U);
}

TEST(PhysicalObstacleTrackStore,
     SparseFarHistoryMaturesInFiveToEightMeterBand) {
  CargoObstacleTrackerConfig config = ordinaryHazardConfig();
  config.require_far_field_history_for_warnings = true;
  config.far_history_confirm_frames = 3;
  config.far_history_confirm_duration_sec = 0.2;
  PhysicalObstacleTrackStore store(config);

  CargoObstacleObservation far = directionalPretrack(1U, 0.0F, 0.0F);
  far.point_count = 8U;
  far.occupied_map_cells = {10, 11, 12};
  far.pose_authority.source_stamp_sec = 1.0;
  store.update(1.0, {far});
  far.source_index = 2U;
  far.pose_authority.source_stamp_sec = 1.1;
  store.update(1.1, {far});
  far.source_index = 3U;
  far.pose_authority.source_stamp_sec = 1.2;
  const CargoObstacleTrackerDecision acquired = store.update(1.2, {far});

  EXPECT_TRUE(acquired.selected_far_field_history_valid);
  EXPECT_EQ(acquired.selected_support_kind,
            ObstacleSupportKind::SPARSE_MULTI_FRAME);
  EXPECT_EQ(acquired.selected_real_current_point_count, 8U);
  EXPECT_EQ(acquired.selected_sparse_independent_frames, 3);
  EXPECT_LE(acquired.sparse_ring_high_water, 3U);
}

TEST(PhysicalObstacleTrackStore,
     SparseWarningRequiresNearFieldSourceResolution) {
  CargoObstacleTrackerConfig config = ordinaryHazardConfig();
  config.require_far_field_history_for_warnings = true;
  config.far_history_confirm_frames = 3;
  config.far_history_confirm_duration_sec = 0.2;
  PhysicalObstacleTrackStore store(config);

  CargoObstacleObservation far = directionalPretrack(1U, 0.0F, 0.0F);
  far.point_count = 8U;
  far.occupied_map_cells = {20, 21, 22};
  for (int index = 0; index < 3; ++index) {
    far.source_index = static_cast<std::size_t>(index + 1);
    far.pose_authority.source_stamp_sec = 1.0 + 0.1 * index;
    store.update(1.0 + 0.1 * index, {far});
  }

  CargoObstacleObservation unresolved = hazard(10U, 0.02F, 0.0F, 18U);
  unresolved.point_count = 8U;
  unresolved.occupied_map_cells = far.occupied_map_cells;
  unresolved.source_validated = false;
  unresolved.pose_authority.source_stamp_sec = 1.3;
  const CargoObstacleTrackerDecision rejected =
      store.update(1.3, {unresolved});
  EXPECT_FALSE(rejected.confirmed_hazard);
  EXPECT_EQ(rejected.reason, "current_source_unvalidated");

  CargoObstacleObservation resolved = unresolved;
  resolved.source_validated = true;
  CargoObstacleTrackerDecision accepted;
  for (int index = 0; index < config.confirm_frames; ++index) {
    resolved.source_index = static_cast<std::size_t>(11 + index);
    const double resolved_stamp = 1.4 + 0.1 * index;
    resolved.pose_authority.source_stamp_sec = resolved_stamp;
    accepted = store.update(resolved_stamp, {resolved});
  }
  EXPECT_TRUE(accepted.confirmed_hazard) << accepted.reason;
}

TEST(PhysicalObstacleTrackStore,
     SparseToDenseKeepsPhysicalTrackIdentity) {
  CargoObstacleTrackerConfig config = ordinaryHazardConfig();
  PhysicalObstacleTrackStore store(config);
  CargoObstacleObservation sparse = hazard(1U, 0.0F, 0.0F, 18U);
  sparse.point_count = 8U;
  sparse.occupied_map_cells = {30, 31, 32};
  sparse.pose_authority.source_stamp_sec = 1.0;
  const CargoObstacleTrackerDecision initial = store.update(1.0, {sparse});
  sparse.source_index = 2U;
  sparse.pose_authority.source_stamp_sec = 1.1;
  store.update(1.1, {sparse});

  CargoObstacleObservation dense = sparse;
  dense.source_index = 3U;
  dense.point_count = 30U;
  dense.pose_authority.source_stamp_sec = 1.2;
  const CargoObstacleTrackerDecision promoted = store.update(1.2, {dense});

  EXPECT_EQ(promoted.selected_track_id, initial.selected_track_id);
  EXPECT_EQ(promoted.selected_support_kind,
            ObstacleSupportKind::DENSE_CURRENT_FRAME);
  EXPECT_TRUE(promoted.selected_sparse_to_dense);
  EXPECT_EQ(promoted.selected_real_current_point_count, 30U);
  EXPECT_EQ(store.tracks().size(), 1U);
}

TEST(PhysicalObstacleTrackStore,
     CrossAuthoritySparseCannotReuseSupport) {
  CargoObstacleTrackerConfig config = ordinaryHazardConfig();
  PhysicalObstacleTrackStore store(config);
  CargoObstacleObservation sparse = hazard(1U, 0.0F, 0.0F, 18U);
  sparse.point_count = 8U;
  sparse.occupied_map_cells = {40, 41, 42};
  sparse.pose_authority.source_stamp_sec = 1.0;
  store.update(1.0, {sparse});
  sparse.source_index = 2U;
  sparse.pose_authority.source_stamp_sec = 1.1;
  store.update(1.1, {sparse});

  sparse.source_index = 3U;
  ++sparse.pose_authority.pose_identity.yaw_authority_generation;
  sparse.pose_authority.source_stamp_sec = 1.2;
  const CargoObstacleTrackerDecision reset = store.update(1.2, {sparse});

  EXPECT_FALSE(reset.confirmed_hazard);
  EXPECT_EQ(reset.selected_sparse_independent_frames, 1);
  EXPECT_EQ(reset.selected_support_kind,
            ObstacleSupportKind::SPARSE_MULTI_FRAME);
}

TEST(PhysicalObstacleTrackStore, DuplicateStampCannotAdvanceSparseSupport) {
  CargoObstacleTracker tracker(ordinaryHazardConfig());
  CargoObstacleObservation sparse = hazard(1U, 0.0F, 0.0F, 18U);
  sparse.point_count = 8U;
  sparse.pose_authority.source_stamp_sec = 1.0;
  tracker.update(1.0, {sparse});
  const CargoObstacleTrackerDecision duplicate = tracker.update(1.0, {sparse});
  EXPECT_EQ(duplicate.reason, "repeated_obstacle_track_stamp");
  ASSERT_EQ(tracker.tracks().size(), 1U);
  EXPECT_EQ(tracker.tracks().front().sparse_independent_frames, 1);
}

}  // namespace
}  // namespace ndt_slam
