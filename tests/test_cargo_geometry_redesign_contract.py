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
TRACKER_SOURCE = (
    PACKAGE / "src/cargo_obstacle_tracker.cpp"
).read_text(encoding="utf-8")
MARKER_SOURCE = (
    PACKAGE / "src/cargo_marker_lifecycle.cpp"
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
        self.assertIn("positive_only_confirm_frames: 5", CONFIG)
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

    def test_preload_baseline_is_empty_static_and_map_frame_only(self):
        self.assertIn("bool hook_empty", PRELOAD_HEADER)
        self.assertIn("bool localization_valid", PRELOAD_HEADER)
        self.assertIn("bool stationary", PRELOAD_HEADER)
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
        self.assertIn("obstacle_bottom_z05_m", SAFETY_SOURCE)
        self.assertIn("vertical_continuity_ratio", SAFETY_SOURCE)
        self.assertIn("evidence.entirely_above_cargo", SAFETY_SOURCE)
        self.assertIn("!evidence.entirely_above_cargo", SAFETY_SOURCE)
        self.assertIn("!observation.entirely_above_cargo", TRACKER_SOURCE)

    def test_known_static_obstacles_do_not_require_far_history(self):
        self.assertIn("known_static_confirm_frames: 3", CONFIG)
        self.assertIn("static_cargo_confirm_frames: 5", CONFIG)
        self.assertIn("static_cargo_confirm_sec: 0.8", CONFIG)
        self.assertIn("track.far_field_history_valid || track.provenance_valid", TRACKER_SOURCE)

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

    def test_new_unit_is_linked_to_the_real_library(self):
        self.assertIn("src/cargo_preload_baseline_tracker.cpp", CMAKE)
        self.assertIn("test/cargo_preload_baseline_tracker_test.cpp", CMAKE)
        self.assertNotIn("ndt_slam_core", CMAKE)


if __name__ == "__main__":
    unittest.main()
