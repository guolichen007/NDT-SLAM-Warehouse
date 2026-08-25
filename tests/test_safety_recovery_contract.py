from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
NODE = (ROOT / "src/ndt_slam/src/ndt_slam.cpp").read_text(
    encoding="utf-8"
)
CMAKE = (ROOT / "src/ndt_slam/CMakeLists.txt").read_text(
    encoding="utf-8"
)
CONFIG = (
    ROOT / "src/ndt_slam/config/live_longterm_mapping.yaml"
).read_text(encoding="utf-8")
LAUNCH = (
    ROOT / "src/ndt_slam/launch/warehouse_live_longterm_mapping.launch"
).read_text(encoding="utf-8")
RVIZ = (
    ROOT / "src/ndt_slam/launch/rviz.rviz"
).read_text(encoding="utf-8")
WATCHDOG = (
    ROOT / "src/ndt_slam/scripts/ops/ndt_recovery_watchdog.py"
).read_text(encoding="utf-8")
SERVICE = (
    ROOT / "src/ndt_slam/scripts/ops/ndt-slam.service.in"
).read_text(encoding="utf-8")
MONITOR_SERVICE = (
    ROOT / "src/ndt_slam/scripts/ops/ndt-slam-monitor.service.in"
).read_text(encoding="utf-8")
INSTALLER = (
    ROOT / "src/ndt_slam/scripts/ops/install_server_services.sh"
).read_text(encoding="utf-8")


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

    def test_GlobalRecoveryUsesCleanStaticMapAndDedicatedAge(self):
        consume = section(
            "void NdtSlamNode::consumeRelocalizationResult(",
            "void NdtSlamNode::updateRelocalization(",
        )
        self.assertIn("RelocalizationMode::GLOBAL", consume)
        self.assertIn(
            "relocalization_global_result_max_age_frames_", consume
        )
        self.assertIn(
            "relocalization_global_result_max_age_sec_", consume
        )

        update = section(
            "void NdtSlamNode::updateRelocalization(",
            "void NdtSlamNode::applyRelocalizedPose(",
        )
        clean = update.index("objects_clean_map_")
        registration = update.index(
            'job.map_source = "global_registration_fallback"'
        )
        self.assertLess(clean, registration)
        self.assertIn(
            'job.map_source = "objects_clean_static"', update
        )
        self.assertIn("coarse_map_farthest_grid", update)
        self.assertIn("job.candidate_limit", update)
        self.assertIn("kMaxCoarseGridAxisSegments = 64", update)
        self.assertIn("static_cast<double>(max_x)", update)

        apply_pose = section(
            "void NdtSlamNode::applyRelocalizedPose(",
            "void NdtSlamNode::resetCargoAfterPoseDiscontinuity(",
        )
        self.assertIn("objects_clean_map_", apply_pose)
        self.assertIn("result.map_source", apply_pose)

    def test_WatchdogAndRvizDefaultsAreWired(self):
        self.assertIn("ndt_recovery_watchdog.py", CMAKE)
        self.assertIn('required="true"', LAUNCH)
        self.assertIn("hard_restart_bad_frames", LAUNCH)
        self.assertIn("global_result_max_age_sec: 12.0", CONFIG)
        self.assertIn("global_max_candidates: 48", CONFIG)

        name = RVIZ.index("Name: display_map")
        block_start = RVIZ.rfind("    - Alpha:", 0, name)
        block_end = RVIZ.find("    - Alpha:", name)
        display_block = RVIZ[block_start:block_end]
        self.assertIn("Enabled: false", display_block)
        self.assertIn("Value: false", display_block)

        self.assertNotIn("os._exit", WATCHDOG)
        self.assertIn("restart_requested.wait", WATCHDOG)
        self.assertIn("return 75", WATCHDOG)
        self.assertIn("use_ndt_recovery_watchdog:=true", SERVICE)
        unit_section, service_section = SERVICE.split("[Service]", 1)
        self.assertIn("StartLimitIntervalSec=300", unit_section)
        self.assertIn("StartLimitBurst=5", unit_section)
        self.assertNotIn("StartLimitIntervalSec", service_section)
        self.assertIn("Restart=on-failure", service_section)
        self.assertIn("RestartPreventExitStatus=78", service_section)
        self.assertNotIn("Restart=always", service_section)
        self.assertEqual(service_section.count("Restart="), 1)
        self.assertIn('"@DATA_ROOT@/.ndt-slam.lock"', SERVICE)

        self.assertIn("After=ndt-slam.service", MONITOR_SERVICE)
        self.assertIn("Wants=ndt-slam.service", MONITOR_SERVICE)
        self.assertIn(
            "server_runtime_monitor.py", MONITOR_SERVICE
        )
        monitor_service_section = MONITOR_SERVICE.split(
            "[Service]", 1
        )[1]
        self.assertIn("Restart=always", monitor_service_section)
        self.assertIn("RestartSec=10", monitor_service_section)
        self.assertIn("TimeoutStopSec=30", monitor_service_section)
        self.assertNotIn(
            "Restart=on-failure", monitor_service_section
        )
        self.assertEqual(
            monitor_service_section.count("Restart="), 1
        )

        self.assertIn("monitor_unit=", INSTALLER)
        self.assertIn("RestartSec=5", INSTALLER)
        self.assertIn("RestartSec=10", INSTALLER)
        self.assertIn(
            "server_runtime_monitor.py", INSTALLER
        )
        self.assertIn(
            '--workspace "$NDT_SLAM_WORKSPACE"', INSTALLER
        )
        self.assertIn(
            "server_monitor.yaml", INSTALLER
        )
        self.assertIn(
            ".service-monitor.lock", INSTALLER
        )
        self.assertIn(
            "warehouse_live_longterm_mapping.launch", INSTALLER
        )
        self.assertIn(
            "use_ndt_recovery_watchdog:=true", INSTALLER
        )
        self.assertIn("RestartPreventExitStatus=78", INSTALLER)
        self.assertIn("ndt_slam_service_supervisor.py", INSTALLER)
        self.assertIn("systemctl show", INSTALLER)
        self.assertIn("EnvironmentFiles", INSTALLER)
        self.assertIn("StartLimitIntervalUSec", INSTALLER)
        self.assertIn("StartLimitBurst", INSTALLER)
        self.assertIn("TimeoutStopUSec", INSTALLER)
        self.assertIn("@[A-Z_][A-Z0-9_]*@", INSTALLER)
        self.assertIn("(check drop-ins)", INSTALLER)

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
