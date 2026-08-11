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
RUNTIME_DIAGNOSTICS_HEADER = (
    ROOT / "src/ndt_slam/include/ndt_slam/runtime_diagnostics.hpp"
).read_text(encoding="utf-8")
RUNTIME_DIAGNOSTICS_SOURCE = (
    ROOT / "src/ndt_slam/src/runtime_diagnostics.cpp"
).read_text(encoding="utf-8")


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
        self.assertIn("last_local_map_pose_ = new_pose", block)
        self.assertIn(
            "local_map_input.normal_motion_update_allowed",
            block,
        )
        self.assertRegex(
            block,
            r"local_map_input\.normal_motion_update_allowed\s*=\s*"
            r"stationary_motion_decision_\.state\s*==\s*"
            r"RuntimeMotionState::MOVING",
        )

    def test_DownstreamAuthoritiesCannotGateRuntimeTarget(self) -> None:
        block = runtime_local_map_block()
        policy_start = block.index("LocalMapUpdateInput local_map_input")
        authority_prefix = block[
            policy_start : block.index(
                "local_map_decision = evaluateLocalMapUpdate"
            )
        ]
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
            "[LocalMapHealth] state=STARVED_MOVING",
            "local_map_update_eligible_count",
            "local_map_update_due_count",
            "local_map_update_committed_count",
            "local_map_motion_escape_refresh_count",
            "local_map_health_state",
            "local_map_update_mode",
            "local_map_block_reason",
            "local_map_pose_authority",
            "local_map_consecutive_prediction_only_frames",
            "local_map_prediction_only_duration_sec",
            "local_map_frames_since_last_trusted_ndt",
            "local_map_time_since_last_trusted_ndt_sec",
            "local_map_distance_since_last_trusted_ndt_m",
            "local_map_yaw_since_last_trusted_ndt_rad",
            "local_map_updates_from_measured_pose",
            "local_map_updates_from_predicted_pose",
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

    def test_PerFrameCsvCapturesLocalMapPoseAndUpdateAuthority(self) -> None:
        required = (
            "local_map_pose_authority",
            "local_map_consecutive_prediction_only_frames",
            "local_map_prediction_only_duration_sec",
            "local_map_frames_since_last_trusted_ndt",
            "local_map_time_since_last_trusted_ndt_sec",
            "local_map_distance_since_last_trusted_ndt_m",
            "local_map_yaw_since_last_trusted_ndt_rad",
            "local_map_update_eligible",
            "local_map_update_due",
            "local_map_update_committed",
            "local_map_update_mode",
            "local_map_health_state",
            "local_map_version",
            "local_map_point_count",
            "local_map_updates_from_measured_pose",
            "local_map_updates_from_predicted_pose",
            "local_map_motion_escape_refresh_count",
            "stationary_entry_confirm_count",
            "stationary_entry_unreliable_count",
        )
        for token in required:
            self.assertIn(token, RUNTIME_DIAGNOSTICS_HEADER)
            self.assertIn(token, RUNTIME_DIAGNOSTICS_SOURCE)
            self.assertIn(f"ndt_rec.{token}", NODE)

    def test_MotionEscapeIsTargetOnlyAndUsesExistingEvidenceWindow(self) -> None:
        stationary_header = (
            ROOT / "src/ndt_slam/include/ndt_slam/stationary_motion_policy.hpp"
        ).read_text(encoding="utf-8")
        stationary_source = (
            ROOT / "src/ndt_slam/src/stationary_motion_policy.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("allow_local_map_motion_escape_refresh", stationary_header)
        self.assertIn("raw_motion_observation_valid", stationary_header)
        for existing_threshold in (
            "exit_confirm_frames",
            "exit_cumulative_motion_m",
            "exit_direction_cosine_min",
            "exit_min_speed_mps",
            "allowed_physical_step_m",
        ):
            self.assertIn(existing_threshold, stationary_source)
        persistent_assignment = re.search(
            r"decision\.allow_persistent_map_commit\s*=\s*(?P<body>.*?);",
            stationary_source,
            re.DOTALL,
        )
        self.assertIsNotNone(persistent_assignment)
        self.assertNotIn(
            "motion_escape", persistent_assignment.group("body")
        )

    def test_StationaryEntryUsesBoundedEvidenceGaps(self) -> None:
        stationary_header = (
            ROOT / "src/ndt_slam/include/ndt_slam/stationary_motion_policy.hpp"
        ).read_text(encoding="utf-8")
        stationary_source = (
            ROOT / "src/ndt_slam/src/stationary_motion_policy.cpp"
        ).read_text(encoding="utf-8")
        live_config = (
            ROOT / "src/ndt_slam/config/live_longterm_mapping.yaml"
        ).read_text(encoding="utf-8")
        for text in (stationary_header, stationary_source, live_config, NODE):
            self.assertIn(
                "enter_max_consecutive_unreliable_frames", text
            )
        self.assertIn("STATIONARY_ENTRY_EVIDENCE_GAP", stationary_source)
        self.assertIn("raw_motion_evidence", stationary_source)

    def test_UpdaterLifecycleHasNoFunctionLocalStatic(self) -> None:
        block = runtime_local_map_block()
        self.assertNotIn("static Sophus::SE3d last_local_map_pose", NODE)
        self.assertNotIn("static int frames_since_last_update", NODE)
        self.assertIn("resetLocalMapUpdateState(", NODE)
        self.assertIn("frames_since_last_local_map_update_", block)
        node_header = (
            ROOT / "src/ndt_slam/include/ndt_slam/ndt_slam.hpp"
        ).read_text(encoding="utf-8")
        self.assertIn("local_map_update_state_mutex_", node_header)
        self.assertIn(
            "std::unique_lock<std::recursive_mutex> local_map_state_lock",
            block,
        )
        reset_start = NODE.index("void NdtSlamNode::resetLocalMapUpdateState")
        reset_body = NODE[reset_start : NODE.index(
            "void NdtSlamNode::handleLidarTimeRollback", reset_start
        )]
        self.assertIn(
            "std::lock_guard<std::recursive_mutex> local_map_state_lock",
            reset_body,
        )

    def test_StationaryIdleCannotBeReportedAsStarved(self) -> None:
        policy = (
            ROOT / "src/ndt_slam/src/local_map_update_policy.cpp"
        ).read_text(encoding="utf-8")
        classify = policy[
            policy.index("LocalMapHealthState classifyLocalMapHealth") :
        ]
        idle_position = classify.index("LocalMapHealthState::IDLE_STATIONARY")
        starved_position = classify.index("LocalMapHealthState::STARVED_MOVING")
        self.assertLess(idle_position, starved_position)
        self.assertIn("input.stationary_idle", classify)
        self.assertIn("input.motion_update_expected", classify)

    def test_PolicyAndGtestAreBuilt(self) -> None:
        self.assertIn("src/local_map_update_policy.cpp", CMAKE)
        self.assertIn("local_map_update_policy_test", CMAKE)
        self.assertTrue(
            (ROOT / "src/ndt_slam/test/local_map_update_policy_test.cpp").is_file()
        )


if __name__ == "__main__":
    unittest.main()
