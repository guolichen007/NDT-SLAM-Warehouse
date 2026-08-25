from __future__ import annotations

from pathlib import Path
import re
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[1]
STAGE1 = "b5565fe648cfff6a9a1486bcdb96c291c63d99cf"


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def section(text: str, start: str, end: str) -> str:
    begin = text.index(start)
    finish = text.index(end, begin)
    return text[begin:finish]


def git_show(path: str) -> bytes:
    return subprocess.check_output(
        ["git", "show", f"{STAGE1}:{path}"], cwd=ROOT
    )


class Stage2StabilityAvoidanceContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.node = read("src/ndt_slam/src/ndt_slam.cpp")
        cls.header = read("src/ndt_slam/include/ndt_slam/ndt_slam.hpp")
        cls.tracker = read("src/ndt_slam/src/cargo_obstacle_tracker.cpp")
        cls.fusion = read("src/ndt_slam/src/cargo_avoidance_fusion.cpp")
        cls.message = read(
            "src/lidar_slam2_msgs/msg/CargoSafetyStatus.msg")

    def test_stage1_owned_files_are_byte_identical(self) -> None:
        protected = (
            "src/ndt_slam/include/ndt_slam/time_epoch_contract.hpp",
            "src/ndt_slam/src/PointCloudMerger.cpp",
            "src/ndt_slam/scripts/ops/ndt_recovery_watchdog.py",
        )
        for path in protected:
            with self.subTest(path=path):
                self.assertEqual((ROOT / path).read_bytes(), git_show(path))

    def test_no_forbidden_threshold_policy_was_imported(self) -> None:
        diff = subprocess.check_output(
            ["git", "diff", STAGE1, "--",
             "src/ndt_slam/config/live_longterm_mapping.yaml"],
            cwd=ROOT, text=True, encoding="utf-8")
        changed = "\n".join(
            line for line in diff.splitlines()
            if line.startswith(("+", "-")) and
            not line.startswith(("+++", "---")))
        self.assertNotRegex(
            changed,
            re.compile(r"ndt|ekf|relocalization|watchdog|systemd|service",
                       re.IGNORECASE),
        )

    def test_schema7_and_all_in_tree_consumers_agree(self) -> None:
        self.assertIn("SCHEMA_VERSION=7", self.message)
        self.assertIn("CODE_ANOMALY_REVIEW=29", self.message)
        self.assertIn("EVIDENCE_REVIEW_REQUIRED=8", self.message)
        heartbeat = read("src/ndt_slam/src/cargo_alarm_heartbeat_node.cpp")
        self.assertIn("SCHEMA_VERSION == 7", heartbeat)
        self.assertIn("CODE_ANOMALY_REVIEW", heartbeat)
        self.assertIn("EVIDENCE_REVIEW_REQUIRED", heartbeat)
        stale = []
        for root in (ROOT / "src", ROOT / "scripts", ROOT / "tests"):
            for path in root.rglob("*"):
                if path.is_file() and path.suffix in {
                    ".cpp", ".hpp", ".py", ".md", ".msg"
                }:
                    text = path.read_text(encoding="utf-8", errors="ignore")
                    if ("CargoSafetyStatus" in text and
                            ("schema " + "v6") in text):
                        stale.append(str(path.relative_to(ROOT)))
        self.assertEqual(stale, [])

    def test_time_epoch_reset_invalidates_avoidance_temporal_authority(self) -> None:
        reset = section(
            self.node,
            "void NdtSlamNode::resetCargoForHookState(",
            "void NdtSlamNode::pointCloudCallback(",
        )
        for token in (
            "physical_obstacle_track_store_.reset()",
            "pending_static_hazard_tracker_.reset()",
            "cargo_subsystem_.reset()",
            "anomaly_review_episode_tracker_.reset()",
            "cargo_safety_temporal_filter_.reset()",
        ):
            self.assertIn(token, reset)
        rollback = section(
            self.node,
            "void NdtSlamNode::handleLidarTimeRollback(",
            "bool NdtSlamNode::cargoTrackRetained(",
        )
        self.assertIn("resetCargoForHookState(false)", rollback)
        self.assertIn("MAP_STATE_PRESERVED", rollback)

    def test_one_physical_track_store_owns_pending_and_formal_history(self) -> None:
        self.assertIn(
            "PhysicalObstacleTrackStore physical_obstacle_track_store_",
            self.header,
        )
        self.assertNotIn("pending_cargo_obstacle_tracker_", self.header)
        self.assertNotIn("cargo_obstacle_tracker_", self.header)
        self.assertIn("far_field_history_valid = false", read(
            "src/ndt_slam/include/ndt_slam/cargo_obstacle_tracker.hpp"))
        self.assertIn("qualifiesForFarHistory", self.tracker)
        self.assertIn("observation.hazard_geometry_valid", self.tracker)
        self.assertIn("pending_physical_identity_changed", self.node)

    def test_missing_vertical_authority_keeps_identity_but_not_warning(self) -> None:
        pending = section(
            self.node,
            "void NdtSlamNode::runPendingCargoAvoidance(",
            "void NdtSlamNode::cargoSwingHookAnchorCallback(",
        )
        self.assertIn("pending_obstacle_perception", pending)
        self.assertIn("observation.hazard_geometry_valid = false", pending)
        self.assertIn("observation.warning_eligible = false", pending)
        self.assertIn("cargo_snapshot.capability.tracking", pending)
        self.assertIn("cargo_snapshot.capability.positive_warning", pending)

    def test_authoritative_hazard_and_frame_identity_commit_atomically(self) -> None:
        for token in (
            "AuthoritativeCargoHazard",
            "applySelectedHazard",
            "far_field_history_valid",
            "obstacle_track_id",
            "pose_generation",
            "map_generation",
        ):
            self.assertIn(token, self.fusion)
        self.assertGreaterEqual(
            self.node.count("commitCargoFrameDecision("), 2)
        self.assertIn("pending_frame_commit", self.node)
        self.assertIn("cargo_frame_commit", self.node)

    def test_observer_reports_execution_layers_without_owning_authority(self) -> None:
        diagnostics = read(
            "src/ndt_slam/include/ndt_slam/avoidance_diagnostics.hpp")
        self.assertIn("Diagnostics is a post-decision observer", diagnostics)
        for token in (
            "perception_executed",
            "external_extraction_executed",
            "clustering_completed",
            "tracking_attempted",
            "warning_authority_valid",
            "block_reason",
        ):
            self.assertIn(token, diagnostics)
            self.assertIn(token, self.node)
        self.assertIn('"NOT_EXECUTED"', self.node)
        self.assertIn('"TRACK_PENDING"', self.node)
        self.assertIn('"AUTHORITY_BLOCKED"', self.node)

    def test_avoidance_generation_lineage_is_authoritative(self) -> None:
        # cargo_frame_decision.hpp must be included by the header itself, not
        # via a local .cpp include that masks the header dependency.
        self.assertIn(
            "#include <ndt_slam/cargo_frame_decision.hpp>", self.header)
        # Avoidance pose generation starts at 1, never 0.
        self.assertIn(
            "std::atomic<std::uint64_t> avoidance_pose_generation_{1U}",
            self.header)
        self.assertIn("advanceAvoidancePoseGeneration", self.header)
        # The increment helper exists and wraps a 0 result back to 1.
        self.assertIn("avoidance_pose_generation_.fetch_add", self.node)
        self.assertIn("avoidance_pose_generation_.store(1U", self.node)
        # resetCargoAfterPoseDiscontinuity bumps exactly once at entry.
        reset_after = section(
            self.node,
            "void NdtSlamNode::resetCargoAfterPoseDiscontinuity()",
            "void NdtSlamNode::publishRelocalizationStatus(",
        )
        self.assertEqual(
            reset_after.count("advanceAvoidancePoseGeneration("), 1)
        # handleLidarTimeRollback bumps exactly once and does NOT route
        # through resetCargoAfterPoseDiscontinuity (no double increment).
        rollback = section(
            self.node,
            "void NdtSlamNode::handleLidarTimeRollback(",
            "bool NdtSlamNode::cargoTrackRetained(",
        )
        self.assertEqual(
            rollback.count(
                'advanceAvoidancePoseGeneration("lidar_source_time_rollback")'),
            1)
        self.assertNotIn("resetCargoAfterPoseDiscontinuity()", rollback)
        # Map generation is a static_evidence_epoch_ snapshot; the dead
        # localization_map_generation_ member must not exist anywhere.
        for token in (
            "review_input.key.map_generation",
            "fusion_input.live.map_generation",
            "fusion_input.static_map.map_generation",
            "avoidance_input.live.map_generation",
            "avoidance_input.static_map.map_generation",
        ):
            self.assertIn(token, self.node)
        self.assertIn("static_evidence_epoch_.load", self.node)
        for source in (self.node, self.header):
            self.assertNotIn("localization_map_generation_", source)
            self.assertNotIn("localization_continuity_generation_", source)


if __name__ == "__main__":
    unittest.main()
