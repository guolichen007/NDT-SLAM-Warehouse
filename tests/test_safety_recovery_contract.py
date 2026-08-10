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
SUPERVISOR = (
    ROOT / "src/ndt_slam/scripts/ops/run_ndt_slam_supervised.sh"
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
MONITOR = (
    ROOT / "src/ndt_slam/scripts/ops/server_runtime_monitor.py"
).read_text(encoding="utf-8")
MONITOR_CTL = (
    ROOT / "src/ndt_slam/scripts/ops/server_monitorctl.sh"
).read_text(encoding="utf-8")
MONITOR_CONFIG = (
    ROOT / "src/ndt_slam/config/server_monitor.yaml"
).read_text(encoding="utf-8")
FUSION_HEADER = (
    ROOT / "src/ndt_slam/include/ndt_slam/cargo_avoidance_fusion.hpp"
).read_text(encoding="utf-8")


def section(start: str, end: str) -> str:
    first = NODE.index(start)
    last = NODE.index(end, first)
    return NODE[first:last]


class SafetyRecoveryContractTest(unittest.TestCase):
    def test_NdtTimingStationaryAndInputSanityDiagnosticsAdvance(self):
        self.assertIn("pointcloud_sanity:", CONFIG)
        self.assertIn("minimum_z_m: -4.0", CONFIG)
        self.assertIn("maximum_z_m: 10.0", CONFIG)
        self.assertIn("pointcloud_z_outlier_rejected_", NODE)
        self.assertIn("pointcloud_nonfinite_rejected_", NODE)
        self.assertIn("kNdtTimingEmaAlpha", NODE)
        self.assertIn(
            "average_ndt_time_ms_ += kNdtTimingEmaAlpha", NODE
        )

        stationary_update = section(
            "StationaryMotionDecision NdtSlamNode::updateStationaryMotionState(",
            "void NdtSlamNode::enterStationaryState(",
        )
        self.assertIn("++stationary_frame_count_", stationary_update)
        stationary_entry = section(
            "void NdtSlamNode::enterStationaryState(",
            "void NdtSlamNode::exitStationaryState(",
        )
        self.assertIn("stationary_frame_count_ = 1", stationary_entry)

    def test_StaticEvidenceRuntimeDiagnosticsUseV3Fields(self):
        runtime = section(
            'f << "  \\"static_evidence_epoch\\": "',
            'f << "  \\"static_height_field_cells\\": "',
        )
        self.assertNotIn(".maximum_observation_gap_sec", runtime)
        self.assertIn(
            "static_evidence_config.immature_max_observation_gap_sec",
            runtime,
        )
        self.assertIn(
            "static_evidence_config.immature_gap_retention_ratio", runtime
        )
        self.assertIn(
            "static_diagnostics.decayed_by_time_gap", runtime
        )

    def test_PendingWarningsDefaultToEvidenceBackedOnly(self):
        self.assertIn(
            "fusion_pending_warning_promotion_policy: evidence_backed_only",
            CONFIG,
        )
        self.assertIn(
            "fusion_provisional_warning_to_official_code: true", CONFIG
        )
        parser = section(
            "const std::string pending_promotion_policy =",
            "cargo_collision_tracking_acquisition_distance_m_ =",
        )
        self.assertIn(
            '.as<std::string>("evidence_backed_only")', parser
        )
        self.assertIn("unknown pending warning promotion", parser)
        self.assertIn("PendingWarningPromotionPolicy::DISABLED", parser)
        self.assertIn(
            "PendingWarningPromotionPolicy::EVIDENCE_BACKED_ONLY", parser
        )
        self.assertIn(
            '"fusion_provisional_warning_to_official_code"', parser
        )
        self.assertIn(
            "pending_warning_promotion_policy =\n"
            "      PendingWarningPromotionPolicy::EVIDENCE_BACKED_ONLY",
            FUSION_HEADER,
        )

    def test_MonitorRetentionAndForegroundFollowAreWired(self):
        self.assertIn("prune_run_directories(", MONITOR)
        self.assertIn("output_retention_runs", MONITOR)
        self.assertIn("path.relative_to(self.run_dir)", MONITOR)
        self.assertIn("max_tracked_identities", MONITOR)
        self.assertIn("max_throttle_keys", MONITOR)
        self.assertIn('parser.add_argument("--quiet-stdout"', MONITOR)
        self.assertIn("output_retention_runs: 20", MONITOR_CONFIG)
        self.assertIn("max_tracked_identities: 2048", MONITOR_CONFIG)
        self.assertIn("--follow) follow_output=true", MONITOR_CTL)
        self.assertIn("--quiet-stdout", MONITOR_CTL)
        self.assertIn("tail -n 50 -F", MONITOR_CTL)

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
        self.assertIn("/ndt_slam/localization_health", LAUNCH)
        self.assertIn("localization_control_streams_stale", WATCHDOG)
        self.assertIn("RECOVERY_IN_PROGRESS_STATES", WATCHDOG)
        self.assertIn('value="/relocalize"', LAUNCH)
        self.assertIn("WAITING_STATIONARY", WATCHDOG)
        self.assertIn("event_log_max_bytes", WATCHDOG)
        self.assertIn("run_ndt_slam_supervised.sh", CMAKE)
        self.assertIn("NDT_SLAM_SUPERVISOR_RUN_ID", SUPERVISOR)
        self.assertIn(
            'REQUEST_RUN_ID" == "$RUN_ID', SUPERVISOR
        )
        self.assertIn(
            "user stop received; not restarting", SUPERVISOR
        )
        self.assertIn("restart budget exhausted (3/900s)", SUPERVISOR)
        self.assertIn("use_ndt_recovery_watchdog:=false", SERVICE)
        unit_section, service_section = SERVICE.split("[Service]", 1)
        self.assertIn("StartLimitIntervalSec=300", unit_section)
        self.assertIn("StartLimitBurst=5", unit_section)
        self.assertNotIn("StartLimitIntervalSec", service_section)
        self.assertIn("Restart=always", service_section)
        self.assertNotIn("Restart=on-failure", service_section)
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
            "use_ndt_recovery_watchdog:=false", INSTALLER
        )
        self.assertIn(
            "contains obsolete Restart=on-failure", INSTALLER
        )
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
            "src/localization_health_policy.cpp",
            "pending_static_hazard_tracker_test",
            "ndt_fitness_circuit_breaker_test",
            "relocalization_confirmation_policy_test",
            "localization_health_policy_test",
        )
        for token in required:
            with self.subTest(token=token):
                self.assertIn(token, CMAKE)

    def test_PendingVelocityScopeAndStartupQuarantineContracts(self):
        pending = section(
            "void NdtSlamNode::runPendingCargoAvoidance(",
            "void NdtSlamNode::cargoSwingHookAnchorCallback(",
        )
        declaration = pending.index(
            "Eigen::Vector2f pending_velocity_map"
        )
        live_branch = pending.index(
            "if (external_live_result.input_valid"
        )
        static_use = pending.index(
            "query.forward_direction_map = pending_velocity_map"
        )
        self.assertLess(declaration, live_branch)
        self.assertLess(live_branch, static_use)
        self.assertIn("bool pending_velocity_valid = false", pending)

        self.assertIn(
            '"/ndt_slam/localization_health"', NODE
        )
        self.assertIn(
            "StartupLocalizationState::STARTUP_QUARANTINE", NODE
        )
        self.assertIn(
            "persistent_manifest_tile_hash_invalid", NODE
        )
        self.assertIn(
            "strict_health_window_verification_complete", NODE
        )
        self.assertIn(
            "runtime_transient_monitored_by_relocalization_gate", NODE
        )
        self.assertIn(
            "pending_static_self_exclusion_authorized", NODE
        )
        self.assertIn(
            "publish_restored_layer(restored.objects_clean", NODE
        )
        self.assertIn(
            "!frame_ndt_accepted", NODE
        )
        self.assertIn(
            "!frame_registration_quality_valid", NODE
        )
        self.assertIn(
            "quarantine_alignment_ready", NODE
        )
        self.assertIn(
            "localization_quarantine_publish_pose_", NODE
        )
        self.assertIn(
            "relocalization_pose_reliable_);", NODE
        )
        self.assertIn(
            "if (!external_output_authorized)", NODE
        )


if __name__ == "__main__":
    unittest.main()
