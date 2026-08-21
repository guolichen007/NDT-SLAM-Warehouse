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

    def test_shadow_bottom_has_no_product_pose_or_thickness(self) -> None:
        body = function_body(
            "void NdtSlamNode::evaluateIntegratedCargoIdentityShadow(",
            "void NdtSlamNode::updateAndPublishCargoSafetyPipeline(",
        )
        self.assertIn("bottom_input.track_center_base", body)
        self.assertIn("integrated_candidate_.identity.center", body)
        self.assertIn("bottom_input.frozen_thickness_valid = false", body)
        self.assertNotIn("hook_lock_.live_pose", body)
        self.assertNotIn("cargo_frozen_geometry_", body)

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
        self.assertIn("stable_anchor", association_costs)
        self.assertIn("physical_vertical_z", association_costs)
        self.assertIn("aggregate_extent", association_costs)
        self.assertNotIn("member_component_ids", association_costs)
        self.assertNotIn("candidate_id", association_costs)

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

    def test_continuity_v2_does_not_change_parameters(self) -> None:
        self.assertIn("maximum_xy_step_m: 0.30", CONFIG)
        self.assertIn("maximum_observation_gap_sec: 0.50", CONFIG)
        self.assertIn("minimum_significant_change_m: 0.15", CONFIG)
        self.assertIn("significance_sigma: 3.0", CONFIG)
        self.assertIn("confirm_frames: 4", CONFIG)


if __name__ == "__main__":
    unittest.main()
