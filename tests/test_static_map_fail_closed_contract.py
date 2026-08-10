import subprocess
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NDT = ROOT / "src" / "ndt_slam"
NODE = (NDT / "src" / "ndt_slam.cpp").read_text(encoding="utf-8")
NODE_HEADER = (NDT / "include" / "ndt_slam" / "ndt_slam.hpp").read_text(
    encoding="utf-8"
)
YAML = (NDT / "config" / "live_longterm_mapping.yaml").read_text(
    encoding="utf-8"
)
ARCHIVE = (NDT / "src" / "bounded_mapping_archive_queue.cpp").read_text(
    encoding="utf-8"
)
ARCHIVE_HEADER = (
    NDT / "include" / "ndt_slam" / "bounded_mapping_archive_queue.hpp"
).read_text(encoding="utf-8")
LINEAGE = (NDT / "src" / "clean_worker_lineage.cpp").read_text(
    encoding="utf-8"
)
TOMBSTONE = (NDT / "src" / "static_obstacle_evidence_index.cpp").read_text(
    encoding="utf-8"
)
POLICY = (NDT / "src" / "mapping_runtime_policy.cpp").read_text(
    encoding="utf-8"
)
SEGMENT = (NDT / "src" / "mapping_segment_manager.cpp").read_text(
    encoding="utf-8"
)
REBUILDER = (NDT / "src" / "static_map_rebuilder.cpp").read_text(
    encoding="utf-8"
)


class StaticMapFailClosedContractTest(unittest.TestCase):
    def test_far_history_business_rule_is_configurable(self):
        self.assertIn("far_history_confirm_frames", YAML)
        self.assertIn("far_history_confirm_duration_sec", YAML)
        self.assertIn("far_history_confirm_frames: 3", YAML)
        self.assertIn("far_history_confirm_duration_sec: 0.2", YAML)
        tracker = (NDT / "src" / "cargo_obstacle_tracker.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("const float safe_distance", tracker)
        self.assertIn("observation.footprint_distance_m -", tracker)
        self.assertIn("config_.far_history_confirm_frames", tracker)
        self.assertIn("config_.far_history_confirm_duration_sec", tracker)
        strict_test = (NDT / "test" / "cargo_obstacle_tracker_test.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("StrictSixFrameHalfSecondFarHistory", strict_test)

    def test_async_clean_uses_authority_not_latest_pose(self):
        self.assertIn("mapping_authority_epoch_mismatch", LINEAGE)
        self.assertIn("lineage_valid_historical_objects_snapshot", LINEAGE)
        self.assertNotIn(
            "source.source_accepted_pose_generation !=\n"
            "      current.source_accepted_pose_generation",
            LINEAGE,
        )
        self.assertIn("PUBLISH_SNAPSHOT_ONLY", NODE)

    def test_tombstones_compare_only_named_version_domains(self):
        self.assertIn("StaticMutationVersion", TOMBSTONE)
        self.assertIn("mapping_authority_epoch", TOMBSTONE)
        self.assertIn("source_objects_version", TOMBSTONE)
        self.assertIn("invalidation_sequence", TOMBSTONE)
        self.assertIn("stale_against_tombstone", TOMBSTONE)
        self.assertNotIn("clean_build_version < tombstone", TOMBSTONE)
        self.assertNotIn("pose_generation < tombstone", TOMBSTONE)

    def test_negative_evidence_revokes_before_clean_worker_completion(self):
        self.assertIn("invalidateStaticAuthorityImmediately", NODE_HEADER)
        self.assertIn("invalidateCleanDenyCellsImmediately", NODE_HEADER)
        self.assertIn(
            "invalidateCleanDenyCellsImmediately(\n"
            "        touched_cells",
            NODE,
        )
        self.assertRegex(
            NODE,
            r"invalidateStaticAuthorityImmediately\(\s*"
            r"observed_free_static_cells",
        )
        self.assertIn(
            "human_filter_.getDenyCellsSnapshot",
            NODE,
        )
        producer = NODE.index("void NdtSlamNode::invalidateStaticAuthorityImmediately")
        worker = NODE.index("void NdtSlamNode::startCleanMapRebuildJob")
        self.assertLess(producer, worker)
        immediate = NODE[producer:worker]
        self.assertIn("mapping_authority_mutex_", immediate)
        self.assertIn("static_invalidation_sequence_", immediate)
        self.assertIn("source_objects_version", immediate)

    def test_quality_pause_and_confirmed_fail_closed_are_separate(self):
        self.assertIn("persistent_high_fitness", POLICY)
        self.assertIn("confirmed_continuous_localization_failure", POLICY)
        self.assertIn("consecutive_hard_failure_frames", POLICY)
        self.assertIn("decision_.fail_closed_latched", POLICY)
        self.assertIn("MappingAuthorityState::PAUSED_QUALITY", POLICY)
        self.assertIn("MappingAuthorityState::PAUSED_IO", POLICY)

    def test_confirmed_failure_reuses_existing_degraded_frame_window(self):
        self.assertIn("inherit_degraded_frame_window: true", YAML)
        self.assertIn("config[\"relocalization\"][\"trigger_frames\"]", NODE)
        self.assertIn("inherit_degraded_frame_window", NODE)
        rollback = NODE.index("void NdtSlamNode::handleLidarTimeRollback")
        rollback_end = NODE.index("// CRITICAL RUNTIME CHAIN", rollback)
        self.assertIn(
            "invalidateAcceptedLocalizationContinuity()",
            NODE[rollback:rollback_end],
        )
        self.assertIn("accepted_measurement_nonfinite", NODE)
        self.assertIn(
            "const bool accepted_measurement_nonfinite = ndt_accepted",
            NODE,
        )

    def test_fail_closed_freezes_accepted_pose_but_keeps_preview_alive(self):
        self.assertIn(
            "[MappingFailClosed] raw preview alive; NDT, AcceptedPose,",
            NODE,
        )
        freeze_guard = NODE.index(
            "[MappingFailClosed] raw preview alive; NDT, AcceptedPose,"
        )
        registration = NODE.index("阶段 1.5", freeze_guard)
        self.assertLess(freeze_guard, registration)
        freeze_window = NODE[freeze_guard:registration]
        self.assertIn("publishRelocalizationSafetyInvalid", freeze_window)
        self.assertIn("mapping_fail_closed:", freeze_window)
        accepted_advance = NODE.index("Accepted Pose Generation 更新")
        accepted_window = NODE[accepted_advance:accepted_advance + 2200]
        self.assertIn("MappingAuthorityState::FAIL_CLOSED", accepted_window)
        self.assertIn("publishLocalizationHealth(msg->header.stamp)", NODE)
        self.assertIn("The frame that confirms failure is already outside", NODE)
        self.assertIn("registration_success = false", NODE)

    def test_legacy_commit_gate_includes_profile_write_authority(self):
        can_commit = NODE.split("bool NdtSlamNode::canCommit()", 1)[1]
        can_commit = can_commit.split("\n}", 1)[0]
        self.assertIn("trustedMappingWritesAllowed()", can_commit)

    def test_new_segment_policy_handoff_is_runtime_mutex_serialized(self):
        service = NODE.index("bool NdtSlamNode::startNewMappingSegmentService")
        service_end = NODE.index("NdtSlamNode::~NdtSlamNode", service)
        body = NODE[service:service_end]
        reset = body.index("resetService(reset_request, reset_response)")
        runtime_lock = body.index("runtime_state_mutex_", reset)
        policy_reset = body.index("mapping_runtime_policy_.resetForNewSegment", reset)
        self.assertLess(runtime_lock, policy_reset)

    def test_failed_closed_segment_remains_code31_after_restart(self):
        initializer = NODE.index(
            "void NdtSlamNode::initializeStaticMapCollectionRuntime"
        )
        initializer_end = NODE.index(
            "bool NdtSlamNode::trustedMappingWritesAllowed", initializer
        )
        body = NODE[initializer:initializer_end]
        self.assertIn("persistent_localization_latch", body)
        self.assertIn("MappingSegmentState::FAILED_CLOSED", body)
        self.assertIn("previous_segment_failed_closed", body)
        self.assertIn("latchFailClosed", body)
        self.assertIn("checksumSidecarValid", SEGMENT)
        self.assertIn("archived_segment_uuid != segment_uuid", SEGMENT)
        self.assertEqual(NODE.count("persistent_mapping_fail_closed:"), 2)
        self.assertGreaterEqual(
            NODE.count("StartupLocalizationState::WAITING_STATIONARY"), 2
        )

    def test_collection_profile_has_no_relocalize_reseed_or_verifying_path(self):
        self.assertIn("relocalization:\n", YAML)
        relocalization = YAML.split("relocalization:\n", 1)[1].split("\n\n", 1)[0]
        self.assertIn("enabled: false", relocalization)
        self.assertIn("forced disabled by", NODE)
        self.assertIn("static_map_build_fail_closed_profile_ ||\n        !relocalization_enabled_", NODE)
        self.assertIn("candidate retained for offline rebuild only", NODE)
        self.assertIn("manual request ignored", NODE)
        self.assertIn("online pose-graph mutation forced", NODE)
        self.assertIn(
            "legacy_map_load_quarantined_in_static_collection_profile", NODE
        )
        self.assertIn(
            "map_session_load_quarantined_in_static_collection_profile", NODE
        )
        self.assertIn("offline static_map_rebuilder on closed segments", NODE)
        loop_section = YAML.split("loop_closure:\n", 1)[1].split("\n\n", 1)[0]
        self.assertIn("enabled: false", loop_section)

    def test_fail_closed_blocks_manual_and_persistent_write_bypasses(self):
        self.assertIn("[SaveMap] rejected: mapping authority is not ACTIVE", NODE)
        self.assertIn("[RebuildMap] rejected in collection profile", NODE)
        self.assertIn("offline static_map_rebuilder on closed segments", NODE)
        write_static = NODE.index("bool NdtSlamNode::writePersistentStaticEvidence")
        static_lock = NODE.index("static_evidence_persistence_mutex_", write_static)
        self.assertIn("trustedMappingWritesAllowed", NODE[write_static:static_lock])

    def test_offline_rebuild_rejects_incomplete_segments_and_groups_episodes(self):
        self.assertIn("segmentAllowsCertification", REBUILDER)
        self.assertIn('terminal == "CLOSED" || terminal == "FAILED_CLOSED"', REBUILDER)
        self.assertIn("episode_gap_sec", REBUILDER)
        self.assertIn("episode_ordinal_by_segment", REBUILDER)
        self.assertIn("robust_z_bin_size_m", REBUILDER)
        self.assertIn("pose graph is disconnected", REBUILDER)
        self.assertIn("pose_graph_adjacency", REBUILDER)
        self.assertIn("histogramQuantile", REBUILDER)
        self.assertIn("robust_minimum_z", REBUILDER)
        self.assertIn("robust_z_bin_size_m: 0.05", YAML)

    def test_archive_is_bounded_by_jobs_and_bytes_off_callback(self):
        self.assertIn("config_.max_jobs", ARCHIVE)
        self.assertIn("config_.max_queue_bytes", ARCHIVE)
        self.assertIn("writing_bytes_", ARCHIVE)
        self.assertIn("CERTIFICATION_CRITICAL", ARCHIVE)
        self.assertIn("BEST_EFFORT_DIAGNOSTIC", ARCHIVE_HEADER)
        self.assertIn("archive_incomplete_ = true", ARCHIVE)
        self.assertIn("diagnostic_queue_.pop_back()", ARCHIVE)
        self.assertIn("archive_disk_space_query_failed", ARCHIVE)
        callback_start = NODE.index("void NdtSlamNode::pointCloudCallback")
        callback_end = NODE.index("bool NdtSlamNode::localizationInputPending", callback_start)
        callback = NODE[callback_start:callback_end]
        for blocking_io in ("savePCDFile", "sha256File", "std::ofstream", "fsync"):
            self.assertNotIn(blocking_io, callback)

    def test_self_mask_preview_cannot_bypass_commissioning(self):
        mask = (NDT / "src" / "sensor_body_self_mask.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("self_mask_geometry_invalid", mask)
        self.assertIn("result.mapping_ready = false", mask)
        self.assertNotIn("result.mapping_ready = !config_.enabled", mask)

    def test_profile_has_no_crane_stop_control_interface(self):
        all_new_runtime = "\n".join(
            path.read_text(encoding="utf-8", errors="replace")
            for path in (NDT / "src").glob("*.cpp")
        ).lower()
        forbidden = (
            "/ndt_slam/stop_request",
            "/crane/stop",
            "/plc/stop",
            "/motion/stop",
            "emergency_stop service",
            "modbus stop",
        )
        for token in forbidden:
            self.assertNotIn(token, all_new_runtime)
        self.assertIn("STATIC_MAP_BUILD_FAIL_CLOSED", YAML)
        self.assertIn("/start_new_mapping_segment", NODE)

    def test_ephemeral_shadow_cannot_bind_ndt_target(self):
        shadow = (NDT / "src" / "tracking_ephemeral_shadow.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("bind_to_ndt_target", shadow)
        self.assertIn("ephemeral_ndt_binding_forbidden", shadow)
        self.assertIn("enabled: false", YAML)
        self.assertIn("bind_to_ndt_target: false", YAML)

    def test_frozen_localization_modules_are_unchanged_from_baseline(self):
        frozen = [
            "src/ndt_slam/src/crane_motion_ekf.cpp",
            "src/ndt_slam/include/ndt_slam/crane_motion_ekf.hpp",
            "src/ndt_slam/src/ndt_fitness_circuit_breaker.cpp",
            "src/ndt_slam/include/ndt_slam/ndt_fitness_circuit_breaker.hpp",
            "src/ndt_slam/src/registration_cloud_builder.cpp",
            "src/ndt_slam/include/ndt_slam/registration_cloud_builder.hpp",
            "src/ndt_slam/src/crane_pose_constraint.cpp",
            "src/ndt_slam/include/ndt_slam/crane_pose_constraint.hpp",
        ]
        result = subprocess.run(
            ["git", "diff", "--name-only", "ec64a9fddb1c9c4d828f448c27c4f7399457eac4", "--", *frozen],
            cwd=ROOT,
            text=True,
            capture_output=True,
            check=True,
        )
        self.assertEqual(result.stdout.strip(), "")

    def test_atomic_cargo_frame_decision_maps_mismatch_to_35(self):
        decision = (NDT / "src" / "cargo_frame_decision.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("identity_and_positive_warning_committed_same_frame", decision)
        self.assertIn("authoritative_hazard_cargo_identity_mismatch", decision)
        self.assertIn("CargoSafetyProtocol::kInternalError", NODE)
        self.assertIn("commitCargoFrameDecision", NODE)

    def test_build_profile_has_no_clear_without_certified_reference(self):
        evaluator = (NDT / "src" / "cargo_safety_evaluator.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("clear_authority_incomplete", evaluator)
        self.assertIn("certified_static_clear_authority_unavailable", evaluator)
        self.assertIn("StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE", NODE)


if __name__ == "__main__":
    unittest.main()
