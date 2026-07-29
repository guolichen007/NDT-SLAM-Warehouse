from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
NODE = (ROOT / "src/ndt_slam/src/ndt_slam.cpp").read_text(
    encoding="utf-8"
)
CMAKE = (ROOT / "src/ndt_slam/CMakeLists.txt").read_text(
    encoding="utf-8"
)


def section(start: str, end: str) -> str:
    first = NODE.index(start)
    last = NODE.index(end, first)
    return NODE[first:last]


class SafetyRecoveryContractTest(unittest.TestCase):
    def test_StaticPendingHazardRequiresIndependentAuthority(self):
        body = section(
            "void NdtSlamNode::runPendingCargoAvoidance(",
            "void NdtSlamNode::cargoSwingHookAnchorCallback(",
        )
        tracker = body.index("pending_static_hazard_tracker_.update(")
        fusion = body.index("fuseCargoAvoidanceRisk(")
        self.assertLess(tracker, fusion)
        self.assertIn(
            "pending_static_origin_exclusion_authorized", body
        )
        self.assertIn(
            "query_static_authorization."
            "official_static_risk_authorized",
            body,
        )
        self.assertIn(
            "ExternalProvenance::STATIC_MAP_MATCH", body
        )
        self.assertIn(
            "provisional_static_geometry_authorized", body
        )

    def test_FitnessCircuitGatesEkfMeasurementAndMapCommit(self):
        start = NODE.index(
            "frame_fitness_decision =\n"
            "                        "
            "ndt_fitness_circuit_breaker_.update("
        )
        ekf_update = NODE.index(
            "crane_motion_ekf_.updateWithNDT(", start
        )
        self.assertLess(start, ekf_update)
        self.assertIn(
            "!frame_fitness_decision.allow_measurement",
            NODE[start:ekf_update],
        )
        runtime = section(
            "const bool runtime_ndt_accepted =",
            "// v8-stable-r3-hotfix-minimal",
        )
        self.assertIn(
            "frame_fitness_decision.allow_measurement", runtime
        )

    def test_RelocalizationUsesValidatedConfirmationPolicy(self):
        body = section(
            "void NdtSlamNode::consumeRelocalizationResult(",
            "void NdtSlamNode::updateRelocalization(",
        )
        evaluate = body.index(
            "evaluateRelocalizationConfirmation("
        )
        apply_pose = body.index("applyRelocalizedPose(")
        self.assertLess(evaluate, apply_pose)
        self.assertIn(
            "RelocalizationConfirmationOutcome::CONFIRMED", body
        )
        self.assertIn(
            "RelocalizationConfirmationOutcome::DISCARD_IDENTITY",
            body,
        )

    def test_NewPoliciesAreBuiltAndHaveUnitTests(self):
        required = (
            "src/pending_static_hazard_tracker.cpp",
            "src/ndt_fitness_circuit_breaker.cpp",
            "src/relocalization_confirmation_policy.cpp",
            "pending_static_hazard_tracker_test",
            "ndt_fitness_circuit_breaker_test",
            "relocalization_confirmation_policy_test",
        )
        for token in required:
            with self.subTest(token=token):
                self.assertIn(token, CMAKE)


if __name__ == "__main__":
    unittest.main()
