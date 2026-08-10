from __future__ import annotations

from pathlib import Path
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[1]
BASE_SHA = "ec64a9fddb1c9c4d828f448c27c4f7399457eac4"
FROZEN_LOCALIZATION_FILES = (
    "src/ndt_slam/src/crane_motion_ekf.cpp",
    "src/ndt_slam/include/ndt_slam/crane_motion_ekf.hpp",
    "src/ndt_slam/src/ndt_fitness_circuit_breaker.cpp",
    "src/ndt_slam/include/ndt_slam/ndt_fitness_circuit_breaker.hpp",
    "src/ndt_slam/src/registration_cloud_builder.cpp",
    "src/ndt_slam/include/ndt_slam/registration_cloud_builder.hpp",
    "src/ndt_slam/src/crane_pose_constraint.cpp",
    "src/ndt_slam/include/ndt_slam/crane_pose_constraint.hpp",
)


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def git(*arguments: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", *arguments],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )


def yaml_block(document: str, key: str) -> str:
    marker = f"{key}:\n"
    start = document.index(marker) + len(marker)
    lines = []
    for line in document[start:].splitlines():
        if line and not line[0].isspace() and not line.startswith("#"):
            break
        lines.append(line)
    return "\n".join(lines)


class AvoidanceFirstStableContractTest(unittest.TestCase):
    def test_FrozenLocalizationFilesAreByteIdenticalToEc64(self):
        result = git("diff", "--exit-code", BASE_SHA, "--",
                     *FROZEN_LOCALIZATION_FILES)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_ProfileDisablesRecoveryWithoutDeletingBaselineCode(self):
        config = read("src/ndt_slam/config/live_longterm_mapping.yaml")
        relocalization = yaml_block(config, "relocalization")
        global_consistency = yaml_block(
            relocalization, "  global_consistency")
        loop_closure = yaml_block(config, "loop_closure")
        self.assertIn("  enabled: false", relocalization)
        self.assertIn("    enabled: false", global_consistency)
        self.assertIn("  enabled: false", loop_closure)

        launch = read(
            "src/ndt_slam/launch/warehouse_live_longterm_mapping.launch")
        service = read(
            "src/ndt_slam/scripts/ops/ndt-slam.service.in")
        installer = read(
            "src/ndt_slam/scripts/ops/install_server_services.sh")
        self.assertIn(
            'name="use_ndt_recovery_watchdog" default="false"', launch)
        self.assertNotIn("use_ndt_recovery_watchdog:=true", service)
        self.assertIn("use_ndt_recovery_watchdog:=false", service)
        self.assertNotIn("use_ndt_recovery_watchdog:=true", installer)
        self.assertIn("use_ndt_recovery_watchdog:=false", installer)

        # The frozen baseline implementation remains available for a later
        # branch; this profile changes activation only.
        node = read("src/ndt_slam/src/ndt_slam.cpp")
        self.assertIn("void NdtSlamNode::updateRelocalization(", node)

    def test_BranchAddsNoOutOfScopeRuntimeOrControlInterface(self):
        paths = (
            "src/ndt_slam/include",
            "src/ndt_slam/src",
            "src/ndt_slam/config",
            "src/ndt_slam/launch",
            "src/ndt_slam/scripts",
        )
        result = git("diff", "--unified=0", BASE_SHA, "--", *paths)
        self.assertEqual(result.returncode, 0, result.stderr)
        additions = "\n".join(
            line[1:] for line in result.stdout.splitlines()
            if line.startswith("+") and not line.startswith("+++")
        )
        forbidden = (
            "FAIL_CLOSED",
            "WAIT_OPERATOR",
            "WAIT_STATIONARY",
            "/start_new_mapping_segment",
            "MappingRuntimePolicy",
            "MappingSegmentManager",
            "campaign_uuid",
            "survey_pass_id",
            "BoundedMappingArchiveQueue",
            "static_map_rebuilder",
            "baseline_installer",
            "TrackingEphemeralMap",
            "/ndt_slam/stop_request",
            "/crane/stop",
            "/plc/stop",
            "/motion/stop",
            "emergency_stop",
            "Modbus",
        )
        for token in forbidden:
            self.assertNotIn(token, additions)
        self.assertNotIn("ROS_INFO", additions)
        self.assertNotIn("std::cout", additions)

    def test_CargoSafetyAndMapRegressionTestsAreWired(self):
        cmake = read("src/ndt_slam/CMakeLists.txt")
        cargo_track = read("src/ndt_slam/test/cargo_track_policy_test.cpp")
        geometry = read(
            "src/ndt_slam/test/cargo_geometry_fusion_test.cpp")
        obstacle = read(
            "src/ndt_slam/test/cargo_obstacle_tracker_test.cpp")
        avoidance = read(
            "src/ndt_slam/test/cargo_avoidance_fusion_test.cpp")
        frame = read("src/ndt_slam/test/cargo_frame_decision_test.cpp")
        map_write = read("src/ndt_slam/test/map_write_authority_test.cpp")
        lineage = read("src/ndt_slam/test/clean_worker_lineage_test.cpp")
        static_z = read(
            "src/ndt_slam/test/static_obstacle_evidence_index_test.cpp")

        for target in (
            "cargo_frame_decision_test",
            "map_write_authority_test",
            "clean_worker_lineage_test",
        ):
            self.assertIn(target, cmake)
        for token in (
            "LongEccentricCargoUsesBoundedSizeAwareHookGate",
            "LearnedOffsetDoesNotMoveSafetyGeometryToHook",
            "AdjacentCandidateCannotBeRecenteredOntoHook",
        ):
            self.assertIn(token, cargo_track)
        for token in (
            "TinyClusterCannotShrinkFrozenEnvelope",
            "ShrinkRequiresQualityEvidenceInRecentWindow",
            "CompatibleFullMeasurementsBecomeFormal",
        ):
            self.assertIn(token, geometry)
        for token in (
            "MaturePretrackDoesNotDelayAccurateLevel1AtThreeMeters",
            "Level1FirstSeenInsideThreeMetersWaitsForFarFieldHistory",
            "FarHistoryUsesDistanceMinusUncertainty",
        ):
            self.assertIn(token, obstacle)
        for token in (
            "ClearanceAtPointEightForbidsAllHazards",
            "MissingFarHistoryAtFourMetersBecomesReview",
            "FormalLiveClearDoesNotRequireStaticBaseline",
        ):
            self.assertIn(token, avoidance)
        self.assertIn("IdentityAndDangerCommitAtomically", frame)
        self.assertIn("MismatchedIdentityBecomesCode35", frame)
        self.assertIn("PredictionAndRejectNeverWrite", map_write)
        self.assertIn("AsyncOlderPoseRemainsValidWithinContinuity", map_write)
        self.assertIn("NewAcceptedPoseDoesNotStarveHistory", lineage)
        self.assertIn("IdentityMismatchHasZeroMutationAuthority", lineage)
        self.assertIn("IsolatedVerticalSpikeDoesNotStretchMatureCell", static_z)

    def test_ProductionConfirmationAndLiveOnlyAuthorityAreFixed(self):
        config = read("src/ndt_slam/config/live_longterm_mapping.yaml")
        node = read("src/ndt_slam/src/ndt_slam.cpp")
        lineage = read("src/ndt_slam/src/clean_worker_lineage.cpp")
        self.assertIn("far_history_confirm_frames: 3", config)
        self.assertIn("far_history_confirm_duration_sec: 0.2", config)
        self.assertIn("positive_only_confirm_frames: 3", config)
        self.assertIn("minimum_confirm_frames: 5", config)
        self.assertIn(
            "fusion_input.static_map.certified_static_provenance = false;",
            node,
        )
        self.assertNotIn(
            "source.source_accepted_pose_generation !=\n"
            "          current.source_accepted_pose_generation",
            lineage,
        )


if __name__ == "__main__":
    unittest.main()
