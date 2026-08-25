#!/usr/bin/env python3
"""Static mutation-authority gates for the investigation-only branch."""

from pathlib import Path
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
NODE = (PACKAGE / "src" / "ndt_slam.cpp").read_text(encoding="utf-8")
HEADER = (PACKAGE / "include" / "ndt_slam" / "ndt_slam.hpp").read_text(
    encoding="utf-8"
)
CONFIG = (
    PACKAGE / "config" / "integrated_cargo_identity_shadow_v1.yaml"
).read_text(encoding="utf-8")
AUTHORITY = (
    PACKAGE / "src" / "cargo_physical_identity_authority.cpp"
).read_text(encoding="utf-8")
AUTHORITY_HEADER = (
    PACKAGE / "include" / "ndt_slam" /
    "cargo_physical_identity_authority.hpp"
).read_text(encoding="utf-8")
VERTICAL = (
    PACKAGE / "src" / "cargo_vertical_evidence.cpp"
).read_text(encoding="utf-8")
RUNNER = (
    PACKAGE / "scripts" / "analysis" /
    "run_integrated_cargo_identity_shadow_four_bags.sh"
).read_text(encoding="utf-8")
ANALYZER = (
    PACKAGE / "scripts" / "analysis" /
    "analyze_integrated_cargo_identity_shadow.py"
).read_text(encoding="utf-8")


def function_body(signature: str, next_signature: str) -> str:
    start = NODE.index(signature)
    end = NODE.index(next_signature, start)
    return NODE[start:end]


class IntegratedCargoIdentityShadowStaticTest(unittest.TestCase):
    def test_investigation_config_is_shadow_only(self) -> None:
        self.assertIn("enabled: true", CONFIG)
        self.assertIn("shadow_only: true", CONFIG)
        self.assertIn(
            "PRODUCT_MODE_NOT_IMPLEMENTED_IN_INVESTIGATION_BUILD", NODE
        )

    def test_shadow_uses_canonical_fusion(self) -> None:
        body = function_body(
            "void NdtSlamNode::evaluateIntegratedCargoIdentityShadow(",
            "void NdtSlamNode::updateAndPublishCargoSafetyPipeline(",
        )
        self.assertIn("fuseCargoAvoidanceRisk(", body)
        self.assertIn("projectShadowCargoOntoCanonicalFusion", body)
        self.assertNotIn("publishCargoWarning", body)
        self.assertNotIn("commitCargoFrameDecision", body)
        self.assertNotIn("physical_obstacle_track_store_.update", body)
        self.assertNotIn("hook_lock_ =", body)

    def test_shadow_obstacle_input_is_immutable_and_authority_is_exact_frame(self) -> None:
        body = function_body(
            "void NdtSlamNode::evaluateIntegratedCargoIdentityShadow(",
            "void NdtSlamNode::updateAndPublishCargoSafetyPipeline(",
        )
        self.assertIn(
            "safety_input.obstacle_cloud_base = obstacle_cloud_base", body
        )
        self.assertNotIn("if (!inside)", body)
        self.assertIn("integrated_canonical_fusion_snapshot_stamp_ == stamp", body)
        self.assertNotIn("canonical_snapshot_age_sec", body)
        self.assertNotIn("track.footprint_distance_m <", body)

    def test_shadow_fusion_runs_after_current_canonical_snapshot(self) -> None:
        pipeline = function_body(
            "void NdtSlamNode::updateAndPublishCargoSafetyPipeline(",
            "void NdtSlamNode::publishPayloadTrackInfoFromFusion(",
        )
        snapshot = pipeline.index(
            "integrated_canonical_fusion_snapshot_ = avoidance_input"
        )
        shadow = pipeline.index("evaluateIntegratedCargoIdentityShadow(")
        product_fusion = pipeline.index("fuseCargoAvoidanceRisk(")
        self.assertLess(snapshot, shadow)
        self.assertLess(shadow, product_fusion)
        self.assertIn("hook, external_obstacle_cloud, pose_map_base", pipeline)

    def test_shadow_bottom_uses_group_owned_points_top_and_track_center(self) -> None:
        body = function_body(
            "void NdtSlamNode::evaluateIntegratedCargoIdentityShadow(",
            "void NdtSlamNode::updateAndPublishCargoSafetyPipeline(",
        )
        self.assertIn("bottom_input.track_center_base", body)
        self.assertIn("integrated_group_evidence_.stable_anchor", body)
        self.assertIn(
            "integrated_group_evidence_.union_points_base", body
        )
        self.assertIn(
            "integrated_group_evidence_.supported_top_z", body
        )
        self.assertNotIn("integrated_candidate_.points_base", body)
        self.assertNotIn("clean_vertical_points_base", body)
        self.assertNotIn("extractCargoVerticalEvidence(", body)
        self.assertNotIn("hook_lock_.live_pose", body)
        self.assertNotIn("cargo_frozen_geometry_", body)

    def test_group_geometry_does_not_depend_on_candidate_lookup(self) -> None:
        body = function_body(
            "void NdtSlamNode::updateIntegratedCargoIdentityShadow(",
            "NdtSlamNode::HookCargoDetection NdtSlamNode::detectCargoAroundOdomAnchor(",
        )
        self.assertIn("bindCargoPhysicalGroupEvidence(", body)
        self.assertIn("resolved_geometry", body)
        self.assertNotIn("matchesResolvedPhysicalHypothesis(", body)
        self.assertNotIn("detection.shadow_candidates", body)

    def test_geometry_ambiguity_breaks_current_bottom_authority(self) -> None:
        body = function_body(
            "void NdtSlamNode::updateIntegratedCargoIdentityShadow(",
            "NdtSlamNode::HookCargoDetection NdtSlamNode::detectCargoAroundOdomAnchor(",
        )
        self.assertIn(
            "geometry_input.geometry.source_stamp_sec", body
        )
        self.assertIn(
            "!integrated_group_evidence_.geometry_resolved", body
        )
        self.assertIn(
            "integrated_shadow_bottom_result_ = CargoBottomResult{}", body
        )
        self.assertNotIn("integrated_shadow_thickness_.reset()", body[body.index(
            "if (group_observed_this_update)"
        ):])

    def test_shadow_thickness_has_only_reference_independent_owner(self) -> None:
        body = function_body(
            "void NdtSlamNode::evaluateIntegratedCargoIdentityShadow(",
            "void NdtSlamNode::updateAndPublishCargoSafetyPipeline(",
        )
        self.assertIn("freezeFromFormalGeometry(", body)
        self.assertIn("shadowThicknessAuthorized(", body)
        self.assertNotIn("hook_lock_.locked_shape", body)
        self.assertNotIn("product frozen thickness", body.lower())

    def test_preload_pass_does_not_write_product_detection(self) -> None:
        marker = "SHADOW preload enumeration"
        start = NODE.index(marker)
        end = NODE.index("// OdomAnchorBox 检测", start)
        preload = NODE[start:end]
        self.assertIn("preload_shadow_detection", preload)
        self.assertNotIn("hook_fixed_cargo_ =", preload)

    def test_trace_never_holds_old_code(self) -> None:
        self.assertIn("shadow_code_held_from_previous", NODE)
        self.assertIn("<< 0 << ','", NODE)

    def test_authority_is_not_a_product_pose_interface(self) -> None:
        self.assertIn("CargoPhysicalIdentityAuthority", HEADER)
        self.assertNotIn("apply_to_product", HEADER)
        self.assertNotIn("product_pose", HEADER)

    def test_association_uses_descriptor_not_frame_local_member_ids(self) -> None:
        pair_start = AUTHORITY.index("struct Pair {")
        pair_end = AUTHORITY.index(
            "std::vector<int> group_match", pair_start
        )
        association_costs = AUTHORITY[pair_start:pair_end]
        self.assertIn("robust_xy_center", association_costs)
        self.assertIn("support_xy_separation", association_costs)
        self.assertIn("physical_vertical_z", association_costs)
        self.assertIn("robust_xy_extent", association_costs)
        self.assertNotIn("member_component_ids", association_costs)
        self.assertNotIn("candidate_id", association_costs)

    def test_support_continuity_requires_positive_area_overlap(self) -> None:
        self.assertIn("hasPositiveAreaSupportOverlap", AUTHORITY)
        self.assertIn("intersection_x > kEpsilon", AUTHORITY)
        self.assertIn("intersection_y > kEpsilon", AUTHORITY)
        self.assertNotIn(
            "pair.support_xy_separation <=\n"
            "                 config_.maximum_xy_step_m", AUTHORITY
        )
        self.assertIn("SUPPORT_OVERLAP_CONTINUITY", AUTHORITY)
        self.assertNotIn("support_overlap_threshold", AUTHORITY)

    def test_raw_roi_is_frame_owned_and_moved_out_of_product_snapshot(self) -> None:
        self.assertIn("CargoShadowFrameEvidence shadow_frame_evidence", HEADER)
        self.assertIn("std::move(hook_fixed_cargo_.shadow_frame_evidence)", NODE)
        self.assertIn("raw_roi_current_frame = crop_cloud", NODE)
        history_start = AUTHORITY_HEADER.index("struct History {")
        history_end = AUTHORITY_HEADER.index("};", history_start)
        history = AUTHORITY_HEADER[history_start:history_end]
        self.assertNotIn("PointCloud", history)
        self.assertNotIn("points_base", history)
        self.assertNotIn("last_group", history)

    def test_raw_and_component_vertical_share_one_context(self) -> None:
        self.assertIn("makeVerticalInput", AUTHORITY)
        self.assertIn("ground_context_matches", AUTHORITY)
        self.assertIn("frame_stamp_matches", AUTHORITY)
        self.assertEqual(AUTHORITY.count("extractCargoVerticalEvidence("), 3)
        self.assertNotIn("estimateExternalGround", AUTHORITY)

    def test_owner_proof_uses_extractor_grid_helpers(self) -> None:
        self.assertIn("makeCargoFootprintGridIndex", AUTHORITY)
        self.assertIn("cargoPointInsideFootprint", AUTHORITY)
        self.assertIn("makeCargoFootprintGridIndex", VERTICAL)
        self.assertIn("cargoPointInsideFootprint", VERTICAL)

    def test_owner_proof_is_hypothesis_member_local(self) -> None:
        self.assertIn("hypothesis_owner_points", AUTHORITY)
        self.assertIn(
            "for (std::uint64_t member : hypothesis.member_component_ids)",
            AUTHORITY,
        )
        self.assertIn(
            "raw_evidence, hypothesis_owner_points", AUTHORITY
        )

    def test_reacquired_vertical_type_has_no_authority_fields(self) -> None:
        start = AUTHORITY_HEADER.index(
            "struct AssociationOnlyReacquiredVerticalEvidence"
        )
        end = AUTHORITY_HEADER.index("};", start)
        value_type = AUTHORITY_HEADER[start:end]
        for forbidden in (
            "identity", "lift", "baseline", "formal", "bottom", "clear"
        ):
            self.assertNotIn(forbidden, value_type.lower())
        self.assertIn("Reciprocal", AUTHORITY)
        self.assertIn("already-unique pair", AUTHORITY)

    def test_diagnostic_percentiles_are_not_association_inputs(self) -> None:
        pair_start = AUTHORITY.index("struct Pair {")
        pair_end = AUTHORITY.index("std::vector<int> group_match", pair_start)
        pair_logic = AUTHORITY[pair_start:pair_end]
        self.assertNotIn("diagnostic_z05", pair_logic)
        self.assertNotIn("diagnostic_z95", pair_logic)
        self.assertNotIn("diagnostic_z_extent", pair_logic)

    def test_ambiguity_revocation_is_history_local(self) -> None:
        self.assertIn("validated_history_conflict", AUTHORITY)
        self.assertIn("frame_has_unrelated_ambiguity", AUTHORITY)
        self.assertNotIn(
            "if (frame_association == CargoCandidateAssociationState::AMBIGUOUS) {\n"
            "    validated_history_id_ = 0U;",
            AUTHORITY,
        )

    def test_group_vertical_is_robust_across_all_hypotheses(self) -> None:
        self.assertIn(
            "for (const auto& hypothesis : group.hypotheses)", AUTHORITY
        )
        self.assertIn("physical_vertical_z = median(supported_tops)", AUTHORITY)
        self.assertIn("CONFLICTING_HYPOTHESIS_SUPPORTED_TOPS", AUTHORITY)

    def test_exactly_two_integrated_shadow_trace_files(self) -> None:
        self.assertIn("integrated_avoidance_shadow.csv", NODE)
        self.assertIn("integrated_identity_groups.csv", NODE)
        self.assertEqual(NODE.count("integrated_avoidance_shadow.csv"), 1)
        self.assertEqual(NODE.count("integrated_identity_groups.csv"), 1)

    def test_v3_trace_exposes_physical_evidence_lineage(self) -> None:
        for field in (
            "group_evidence_owner_valid",
            "group_evidence_source_stamp",
            "group_union_point_count",
            "downstream_points_source",
            "downstream_top_source",
            "shadow_thickness_valid",
            "shadow_thickness_m",
            "shadow_thickness_owner_history_id",
            "shadow_thickness_source",
            "bottom_source",
            "bottom_reason",
            "bottom_points_finite",
            "bottom_points_visible_height",
            "bottom_points_support_strong",
            "resolved_candidate_points_used_for_bottom",
        ):
            self.assertIn(field, NODE)

    def test_continuity_v2_does_not_change_parameters(self) -> None:
        self.assertIn("maximum_xy_step_m: 0.30", CONFIG)
        self.assertIn("maximum_observation_gap_sec: 0.50", CONFIG)
        self.assertIn("minimum_significant_change_m: 0.15", CONFIG)
        self.assertIn("significance_sigma: 3.0", CONFIG)
        self.assertIn("confirm_frames: 4", CONFIG)

    def test_v4_group_trace_extends_existing_file_only(self) -> None:
        for field in (
            "robust_x05",
            "robust_x95",
            "robust_y05",
            "robust_y95",
            "robust_xy_center_x",
            "robust_xy_extent_x",
            "association_mode",
            "support_xy_separation",
            "validated_history_conflict",
            "conflicting_history_id",
            "frame_has_unrelated_ambiguity",
        ):
            self.assertIn(field, NODE)

    def test_v5_trace_exposes_ownership_and_runtime_cost(self) -> None:
        for field in (
            "vertical_source",
            "owner_overlap_cell_count",
            "owner_overlap_coverage",
            "component_vertical_valid",
            "component_vertical_z",
            "raw_roi_vertical_valid",
            "raw_roi_vertical_z",
            "diagnostic_z05",
            "diagnostic_z95",
            "reacquired_vertical_valid",
            "v5_raw_roi_vertical_total_ms",
            "v5_raw_roi_vertical_hypothesis_count",
            "v5_raw_roi_vertical_points_examined",
            "integrated_shadow_update_ms",
            "callback_or_pipeline_total_ms",
        ):
            self.assertIn(field, NODE)

    def test_one_shot_runner_has_exact_source_and_clean_tree_gates(self) -> None:
        self.assertIn("EXPECTED_SHA", RUNNER)
        self.assertIn('actual_sha="$(git -C "$workspace" rev-parse HEAD)"', RUNNER)
        self.assertIn("status --porcelain --untracked-files=all", RUNNER)
        self.assertIn("SHA_GATE=FAIL", RUNNER)
        self.assertIn("WORKTREE_GATE=FAIL", RUNNER)

    def test_each_bag_gets_a_new_trace_generation(self) -> None:
        self.assertIn("rm -f \\", RUNNER)
        self.assertIn("trace_generation_marker", RUNNER)
        self.assertIn("-nt", RUNNER)

    def test_full_test_results_are_machine_gated(self) -> None:
        self.assertIn("catkin_test_results --all --verbose", RUNNER)
        self.assertIn("CATKIN_TEST_RESULTS_RC", RUNNER)

    def test_root_classifier_uses_existing_continuity_contract(self) -> None:
        self.assertIn("longest_oracle_correct_supported_sequence", ANALYZER)
        self.assertIn("maximum_observation_gap_sec", ANALYZER)
        root_start = ANALYZER.index("def determine_earliest_root")
        root_end = ANALYZER.index("def analyze_trace", root_start)
        root = ANALYZER[root_start:root_end]
        self.assertNotIn("raw_current_footprint_still_invalid", root)
        self.assertIn("yes_bag_exit_classification", root)

    def test_runtime_gate_does_not_fail_on_rate_delta_alone(self) -> None:
        self.assertIn("callback_hz_delta", ANALYZER)
        self.assertIn("no_counter_regression", ANALYZER)
        self.assertNotIn("no_rate_regression", ANALYZER)


if __name__ == "__main__":
    unittest.main()
