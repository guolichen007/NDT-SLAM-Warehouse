from __future__ import annotations

import pathlib
import re
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
NODE = (ROOT / "src/ndt_slam/src/ndt_slam.cpp").read_text(
    encoding="utf-8"
)
HEADER = (
    ROOT / "src/ndt_slam/include/ndt_slam/local_map_update_policy.hpp"
).read_text(encoding="utf-8")
CMAKE = (ROOT / "src/ndt_slam/CMakeLists.txt").read_text(encoding="utf-8")


def runtime_local_map_block() -> str:
    start_marker = "// local_map_ is the short-lived NDT working target"
    end_marker = "Sophus::SE3d constrained_pose = new_pose;"
    start = NODE.index(start_marker)
    end = NODE.index(end_marker, start)
    return NODE[start:end]


class LocalMapSelfHealingContractTest(unittest.TestCase):
    def test_RuntimeTargetUsesCurrentPoseAndIndependentPolicy(self) -> None:
        block = runtime_local_map_block()
        self.assertIn("evaluateLocalMapUpdate(local_map_input)", block)
        self.assertIn("new_pose.matrix().cast<float>()", block)
        self.assertIn("last_local_map_pose = new_pose", block)
        self.assertIn(
            "stationary_motion_decision_.state == RuntimeMotionState::MOVING",
            block,
        )

    def test_DownstreamAuthoritiesCannotGateRuntimeTarget(self) -> None:
        block = runtime_local_map_block()
        authority_prefix = block[: block.index("const LocalMapUpdateDecision")]
        for forbidden in (
            "accepted_snapshot_is_current",
            "frame_ndt_accepted",
            "frame_registration_quality_valid",
            "frame_fitness_decision.allow_measurement",
            "allow_persistent_map_commit_",
            "evaluateMapWriteAuthority",
            "evaluateCleanWorkerLineage",
        ):
            self.assertNotIn(forbidden, authority_prefix)

        policy_input = re.search(
            r"struct LocalMapUpdateInput \{(?P<body>.*?)\n\};",
            HEADER,
            re.DOTALL,
        )
        self.assertIsNotNone(policy_input)
        input_body = policy_input.group("body").lower()
        for forbidden_field in (
            "accepted_snapshot",
            "ndt_accepted",
            "registration_quality",
            "fitness",
            "persistent",
        ):
            self.assertNotIn(forbidden_field, input_body)

    def test_PersistentWriteAuthorityRemainsStrictAndSeparate(self) -> None:
        self.assertGreaterEqual(NODE.count("evaluateMapWriteAuthority("), 3)
        self.assertGreaterEqual(NODE.count("evaluateCleanWorkerLineage("), 2)
        self.assertIn("allow_persistent_map_commit_", NODE)
        self.assertIn("accepted_localization_snapshot_", NODE)

    def test_StarvationDiagnosticsExposeRequiredCausalSignals(self) -> None:
        required = (
            "[LocalMapHealth] state=STARVED",
            "local_map_update_attempted_count",
            "local_map_update_allowed_count",
            "local_map_block_reason",
            "ndt_target_source",
            "ndt_target_point_count",
            "ndt_target_version",
            "accepted_snapshot_valid",
            "accepted_snapshot_sequence",
            "accepted_snapshot_age_sec",
            "frame_ndt_accepted",
            "frame_registration_quality_valid",
            "fitness_allow_measurement",
            "recovery_scan_buffer_size",
        )
        for token in required:
            self.assertIn(token, NODE)
        self.assertRegex(NODE, r"ROS_WARN_THROTTLE\(\s*5\.0,")

    def test_PolicyAndGtestAreBuilt(self) -> None:
        self.assertIn("src/local_map_update_policy.cpp", CMAKE)
        self.assertIn("local_map_update_policy_test", CMAKE)
        self.assertTrue(
            (ROOT / "src/ndt_slam/test/local_map_update_policy_test.cpp").is_file()
        )


if __name__ == "__main__":
    unittest.main()
