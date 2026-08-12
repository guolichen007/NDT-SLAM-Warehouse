from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/ndt_slam"
NODE = (PACKAGE / "src/ndt_slam.cpp").read_text(encoding="utf-8")
FUSION_HEADER = (
    PACKAGE / "include/ndt_slam/cargo_geometry_fusion.hpp"
).read_text(encoding="utf-8")
FUSION_SOURCE = (
    PACKAGE / "src/cargo_geometry_fusion.cpp"
).read_text(encoding="utf-8")
PRELOAD_HEADER = (
    PACKAGE / "include/ndt_slam/cargo_preload_baseline_tracker.hpp"
).read_text(encoding="utf-8")
PRELOAD_SOURCE = (
    PACKAGE / "src/cargo_preload_baseline_tracker.cpp"
).read_text(encoding="utf-8")
SAFETY_SOURCE = (
    PACKAGE / "src/cargo_safety_evaluator.cpp"
).read_text(encoding="utf-8")
SAFETY_HEADER = (
    PACKAGE / "include/ndt_slam/cargo_safety_evaluator.hpp"
).read_text(encoding="utf-8")
TRACKER_SOURCE = (
    PACKAGE / "src/cargo_obstacle_tracker.cpp"
).read_text(encoding="utf-8")
GEOMETRY_TEST = (
    PACKAGE / "test/cargo_geometry_fusion_test.cpp"
).read_text(encoding="utf-8")
TRACKER_TEST = (
    PACKAGE / "test/cargo_obstacle_tracker_test.cpp"
).read_text(encoding="utf-8")
MARKER_SOURCE = (
    PACKAGE / "src/cargo_marker_lifecycle.cpp"
).read_text(encoding="utf-8")
MOTION_HEADER = (
    PACKAGE / "include/ndt_slam/cargo_motion_corridor.hpp"
).read_text(encoding="utf-8")
MOTION_SOURCE = (
    PACKAGE / "src/cargo_motion_corridor.cpp"
).read_text(encoding="utf-8")
MOTION_TEST = (
    PACKAGE / "test/cargo_motion_corridor_test.cpp"
).read_text(encoding="utf-8")
STATIC_HEIGHT_HEADER = (
    PACKAGE / "include/ndt_slam/static_height_field.hpp"
).read_text(encoding="utf-8")
STATIC_HEIGHT_SOURCE = (
    PACKAGE / "src/static_height_field.cpp"
).read_text(encoding="utf-8")
STATIC_HEIGHT_TEST = (
    PACKAGE / "test/static_height_field_test.cpp"
).read_text(encoding="utf-8")
STATIC_EVIDENCE_HEADER = (
    PACKAGE / "include/ndt_slam/static_obstacle_evidence_index.hpp"
).read_text(encoding="utf-8")
STATIC_EVIDENCE_SOURCE = (
    PACKAGE / "src/static_obstacle_evidence_index.cpp"
).read_text(encoding="utf-8")
STATIC_EVIDENCE_TEST = (
    PACKAGE / "test/static_obstacle_evidence_index_test.cpp"
).read_text(encoding="utf-8")
CONFIG = (PACKAGE / "config/live_longterm_mapping.yaml").read_text(
    encoding="utf-8"
)
CMAKE = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")


class CargoGeometryRedesignContractTest(unittest.TestCase):
    def test_authorization_and_constraint_vocabulary_is_explicit(self):
        for token in (
            "FULL_MEASUREMENT",
            "LOWER_BOUND",
            "PRIOR_ONLY",
            "PENDING",
            "POSITIVE_ONLY",
            "FORMAL",
        ):
            self.assertIn(token, FUSION_HEADER)
        self.assertIn(
            "allow_positive_only_without_static_baseline: true", CONFIG
        )
        self.assertIn("positive_only_confirm_frames: 3", CONFIG)
        self.assertNotIn("allow_degraded_live_only_freeze:", CONFIG)
        self.assertNotIn("minimum_independent_sources:", CONFIG)

    def test_source_conflict_builds_conservative_positive_only_geometry(self):
        self.assertIn("const bool source_conflict = best_static && live", FUSION_SOURCE)
        self.assertIn("CargoGeometryAuthorization::POSITIVE_ONLY", FUSION_SOURCE)
        self.assertIn("std::max(0.15F, 3.0F *", FUSION_SOURCE)
        self.assertIn("result_.thickness_upper_bound_m", FUSION_SOURCE)
        self.assertIn("result_.conservative_tracking_allowance_m", FUSION_SOURCE)
        self.assertIn("result_.conservative_safety_margin_m", FUSION_SOURCE)
        self.assertNotIn('result_.reason = "thickness_source_disagreement"', FUSION_SOURCE)

    def test_positive_only_is_reachable_without_formal_lock_and_stays_stable(self):
        self.assertIn("bool warning_track_stable", FUSION_HEADER)
        self.assertIn(
            "frame.warning_track_stable && live", FUSION_SOURCE
        )
        self.assertIn("minimum_initial_shape_confidence: 0.55", CONFIG)
        self.assertIn("maximum_initial_dimension_mad_m: 0.30", CONFIG)
        self.assertIn("thickness_candidate_window_", FUSION_SOURCE)
        self.assertIn(
            "positive_only_confirmed_nominal_geometry", NODE
        )
        self.assertIn(
            "hook_lock_.provisional_track_id", NODE
        )

    def test_frozen_box_and_formal_height_paths_are_stable(self):
        self.assertIn("immediate_expand_enabled: false", CONFIG)
        self.assertIn(
            "expands &&\n          expansion_quality_valid", FUSION_SOURCE
        )
        self.assertIn("fuseFormalThicknessPair", FUSION_SOURCE)
        self.assertIn(
            "FormalThicknessIsInvariantToPositiveOnlyPromotionPath",
            GEOMETRY_TEST,
        )

    def test_obstacle_association_is_global_deterministic_and_2d(self):
        self.assertIn("std::vector<AssociationPair> legal_pairs", TRACKER_SOURCE)
        self.assertIn("std::sort(legal_pairs.begin()", TRACKER_SOURCE)
        self.assertIn("centroid_map.head<2>()", TRACKER_SOURCE)
        self.assertIn("association_max_top_step_m", TRACKER_SOURCE)
        self.assertIn(
            "GlobalAssociationIsInvariantToObservationInputOrder",
            TRACKER_TEST,
        )
        self.assertIn(
            "CentroidZShiftDoesNotDuplicateIndependentTopGate",
            TRACKER_TEST,
        )

    def test_pending_static_warning_has_safe_geometry_self_exclusion(self):
        self.assertIn(
            "pending_static_geometry_exclusion_authorized", NODE
        )
        self.assertIn(
            "pending_static_self_exclusion_authorized", NODE
        )
        self.assertIn("query.cargo_self_exclusion_authorized", NODE)
        self.assertIn("excluded_cargo_self_cells", NODE)

    def test_preload_baseline_is_empty_static_and_map_frame_only(self):
        self.assertIn("bool hook_empty", PRELOAD_HEADER)
        self.assertIn("bool localization_valid", PRELOAD_HEADER)
        self.assertIn("bool stationary", PRELOAD_HEADER)
        self.assertIn("allow_moving_mature_static", PRELOAD_HEADER)
        self.assertIn("independently_mature_static", PRELOAD_HEADER)
        self.assertIn("maximum_anchor_component_distance_m = 0.50F", PRELOAD_HEADER)
        self.assertIn("minimum_occupied_cells = 6U", PRELOAD_HEADER)
        self.assertIn("maximum_component_uncertainty_m = 0.20F", PRELOAD_HEADER)
        self.assertIn("previous.map_generation", PRELOAD_SOURCE)
        self.assertIn("input.component.component_id == previous.component_id", PRELOAD_SOURCE)
        self.assertIn("anchor_component_spatially_uncertain", PRELOAD_SOURCE)
        self.assertIn("baseline_spatial_coverage_insufficient", PRELOAD_SOURCE)
        self.assertNotIn("odom_z", PRELOAD_HEADER + PRELOAD_SOURCE)
        self.assertIn("cargo_preload_baseline_tracker_.update", NODE)
        self.assertNotIn("recordEmptyHookOriginHeight", NODE)
        self.assertIn("if (!preserve_origin_height)", NODE)
        self.assertIn("resetCargoForHookState(true, true)", NODE)

    def test_positive_only_cannot_clear_remove_or_commit(self):
        self.assertIn('"positive_only_frozen_geometry"', NODE)
        self.assertIn("effective_cargo_envelope_.can_authorize_clear = false", NODE)
        self.assertIn("const bool positive_only_warning =", NODE)
        self.assertIn(
            "lidar_cargo_accepted && formal_cargo_authorized", NODE
        )
        self.assertIn(
            "cargo_frozen_geometry_.formal_authorized &&\n"
            "            effective_cargo_envelope_.can_authorize_clear",
            NODE,
        )
        self.assertIn(
            "cargo_frozen_geometry_.formal_authorized &&\n"
            "        effective_cargo_envelope_.can_authorize_clear",
            NODE,
        )

    def test_obstacle_vertical_interval_rejects_overhead_clusters(self):
        hazard_source = (
            PACKAGE / "src/hazard_evaluator.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("bottom_z05_m", hazard_source)
        self.assertIn("vertical_continuity_ratio", SAFETY_SOURCE)
        self.assertIn("evidence.entirely_above_cargo", SAFETY_SOURCE)
        self.assertIn("!result.entirely_above_cargo", hazard_source)
        self.assertIn("!observation.entirely_above_cargo", TRACKER_SOURCE)

    def test_only_certified_static_can_replace_true_far_history(self):
        self.assertIn("known_static_confirm_frames: 3", CONFIG)
        self.assertIn("static_cargo_confirm_frames: 5", CONFIG)
        self.assertIn("static_cargo_confirm_sec: 0.8", CONFIG)
        self.assertIn("qualifiesForFarHistory", TRACKER_SOURCE)
        self.assertIn("track.far_field_history_valid", TRACKER_SOURCE)
        self.assertIn("selected_far_field_history_valid", NODE)
        self.assertIn("pending_static_decision.far_field_history_valid", NODE)
        fusion = (
            PACKAGE / "src/cargo_avoidance_fusion.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("static_warning_independently_proven", fusion)
        self.assertIn("certified_static_provenance", fusion)
        self.assertIn("certified_static_provenance = false", NODE)

    def test_envelope_source_changes_do_not_reset_obstacle_identity(self):
        start = NODE.index("const bool pending_obstacle_context_changed =")
        end = NODE.index("if (pending_obstacle_context_changed)", start)
        reset_expression = NODE[start:end]
        self.assertIn("pending_obstacle_context_lifecycle_id_", reset_expression)
        self.assertIn("track_segment_pose_discontinuity", reset_expression)
        self.assertIn("actual_geometry_discontinuity", reset_expression)
        self.assertNotIn("pending_obstacle_context_envelope_source_ !=", reset_expression)
        self.assertNotIn("pending_obstacle_context_pose_source_ !=", reset_expression)
        self.assertNotIn("pending_obstacle_context_shape_source_ !=", reset_expression)

    def test_forward_sector_gates_acquisition_not_final_live_collision(self):
        self.assertIn(
            "motion_corridor_forward_half_angle_deg: 45.0", CONFIG
        )
        self.assertIn("forward_half_angle_deg = 45.0F", MOTION_HEADER)
        self.assertIn("cargoPointInForwardSector", MOTION_HEADER)
        self.assertIn("cargoPointInForwardSector", MOTION_SOURCE)
        self.assertIn("obstacle_outside_forward_sector", MOTION_SOURCE)
        self.assertIn("FortyFiveDegreeBoundaryIsIncluded", MOTION_TEST)
        self.assertIn(
            "SideObstacleInsideWideRectangleIsAngleRejected", MOTION_TEST
        )

        pending_start = NODE.index(
            "void NdtSlamNode::runPendingCargoAvoidance("
        )
        pending_end = NODE.index(
            "void NdtSlamNode::cargoSwingHookAnchorCallback(",
            pending_start,
        )
        pending = NODE[pending_start:pending_end]
        self.assertIn("const bool raw_warning_candidate =", pending)
        self.assertIn(
            "raw_warning_candidate || acquisition_candidate", pending
        )
        self.assertIn(
            "warning_eligible = raw_warning_candidate;", pending
        )
        self.assertIn(
            "acquisition_candidate &&\n"
            "                    corridor_decision.eligible",
            pending,
        )
        self.assertIn("query.directional_filter_enabled = false;", pending)
        self.assertIn("pending_angle_rejected_clusters", pending)
        self.assertIn("static_hazard_observation.authority_valid", pending)
        self.assertIn("pending_warning_motion_authorized;", pending)

        formal_start = NODE.index(
            "void NdtSlamNode::updateAndPublishCargoSafetyPipeline("
        )
        formal = NODE[formal_start:]
        self.assertIn(
            "Retain real <=5 m low-clearance geometry", formal
        )
        self.assertIn(
            "raw_cargo_safety_result.cluster_evidence.push_back(evidence);",
            formal,
        )

        self.assertIn("directional_filter_enabled", STATIC_HEIGHT_HEADER)
        self.assertIn("forward_half_angle_deg", STATIC_HEIGHT_HEADER)
        self.assertIn("cargoPointInForwardSector", STATIC_HEIGHT_SOURCE)
        self.assertIn(
            "DirectionalQueryRejectsSideStaticStructure", STATIC_HEIGHT_TEST
        )
        formal_start = NODE.index(
            "void NdtSlamNode::updateAndPublishCargoSafetyPipeline("
        )
        formal = NODE[formal_start:]
        self.assertIn(
            "static_query.directional_filter_enabled = false;", formal
        )
        self.assertIn(
            "cargo_static_directional_rejected_cells_", formal
        )

    def test_code29_owns_immediate_and_sudden_level1_review(self):
        self.assertIn("CODE_ANOMALY_REVIEW=29", (
            ROOT / "src/lidar_slam2_msgs/msg/CargoSafetyStatus.msg"
        ).read_text(encoding="utf-8"))
        self.assertIn("kAnomalyReview = 29", SAFETY_HEADER)
        self.assertIn("review_immediate_contact_guard", NODE)
        self.assertIn("review_level1_without_approach_history", NODE)
        self.assertIn("selected_far_field_history_valid", NODE)
        self.assertIn("pending_static_far_field_history_valid", NODE)
        self.assertIn("CargoSafetyEvaluator::kReviewCode", NODE)

    def test_marker_deletes_on_localization_loss(self):
        self.assertIn('"localization_invalid_delete"', MARKER_SOURCE)
        localization_branch = MARKER_SOURCE.split(
            "if (!input.localization_valid)", 1
        )[1].split("if (input.geometry_valid", 1)[0]
        self.assertIn("has_last_geometry_ = false", localization_branch)
        self.assertIn("return decision", localization_branch)

    def test_static_evidence_survives_sparse_warehouse_revisits_safely(self):
        self.assertIn(
            "immature_max_observation_gap_sec = 300.0",
            STATIC_EVIDENCE_HEADER,
        )
        self.assertIn(
            "immature_gap_retention_ratio = 0.50",
            STATIC_EVIDENCE_HEADER,
        )
        self.assertIn(
            "cell.consecutive_stable_duration_sec *=",
            STATIC_EVIDENCE_SOURCE,
        )
        self.assertIn("observed_free_tombstones_.insert", STATIC_EVIDENCE_SOURCE)
        self.assertIn("cell.clean_map_confirmed &&", STATIC_EVIDENCE_SOURCE)
        self.assertIn(
            "WarehouseRevisitsWithinImmatureWindowCanMature",
            STATIC_EVIDENCE_TEST,
        )
        self.assertIn(
            "ExpiredImmatureHistoryDecaysInsteadOfBeingDiscarded",
            STATIC_EVIDENCE_TEST,
        )
        self.assertIn(
            "static_map_immature_max_observation_gap_sec: 300.0", CONFIG
        )
        self.assertNotIn("static_map_max_observation_gap_sec: 30.0", CONFIG)

    def test_static_vertical_history_rejects_isolated_spikes(self):
        self.assertIn("height_history_window = 9U", STATIC_EVIDENCE_HEADER)
        self.assertIn("height_outlier_mad_multiplier = 3.5", STATIC_EVIDENCE_HEADER)
        self.assertIn("updateHeightEstimateLocked", STATIC_EVIDENCE_SOURCE)
        self.assertIn("height_outliers_rejected", STATIC_EVIDENCE_SOURCE)
        self.assertNotIn(
            "cell.max_z = std::max(cell.max_z, geometry.max_z)",
            STATIC_EVIDENCE_SOURCE,
        )
        self.assertIn(
            "IsolatedVerticalSpikeDoesNotStretchMatureCell",
            STATIC_EVIDENCE_TEST,
        )
        self.assertIn("static_map_height_history_window: 9", CONFIG)

    def test_new_unit_is_linked_to_the_real_library(self):
        self.assertIn("src/cargo_preload_baseline_tracker.cpp", CMAKE)
        self.assertIn("test/cargo_preload_baseline_tracker_test.cpp", CMAKE)
        self.assertNotIn("ndt_slam_core", CMAKE)


if __name__ == "__main__":
    unittest.main()
