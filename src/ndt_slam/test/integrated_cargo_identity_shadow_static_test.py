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


if __name__ == "__main__":
    unittest.main()
