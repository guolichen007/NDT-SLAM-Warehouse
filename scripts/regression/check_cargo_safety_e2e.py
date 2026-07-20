#!/usr/bin/env python3
"""Static contract check for the production Cargo typed-message chain."""

from __future__ import annotations

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def require(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def main() -> int:
    failures: list[str] = []
    cmake = read("src/ndt_slam/CMakeLists.txt")
    node = read("src/ndt_slam/src/ndt_slam.cpp")
    header = read("src/ndt_slam/include/ndt_slam/ndt_slam.hpp")
    heartbeat = read("src/ndt_slam/src/cargo_alarm_heartbeat_node.cpp")
    hook_node = read("src/ndt_slam/src/hook_load_state_node.cpp")
    hook_policy = read(
        "src/ndt_slam/src/hook_load_evidence_policy.cpp")
    crane_constraint = read("src/ndt_slam/src/crane_pose_constraint.cpp")
    safety_header = read(
        "src/ndt_slam/include/ndt_slam/cargo_safety_evaluator.hpp")
    runtime_header = read(
        "src/ndt_slam/include/ndt_slam/runtime_diagnostics.hpp")
    safety_evaluator = read("src/ndt_slam/src/cargo_safety_evaluator.cpp")
    launch = read("src/ndt_slam/launch/warehouse_live_longterm_mapping.launch")
    messages = read("src/lidar_slam2_msgs/CMakeLists.txt")
    bottom_message = read("src/lidar_slam2_msgs/msg/CargoBottomEstimate.msg")
    safety_message = read("src/lidar_slam2_msgs/msg/CargoSafetyStatus.msg")
    merger = read("src/ndt_slam/config/merger_params.yaml")
    live_config = read("src/ndt_slam/config/live_longterm_mapping.yaml")
    rviz = read("src/ndt_slam/launch/rviz.rviz")

    for source in ("src/cargo_bottom_fusion.cpp", "src/cargo_safety_evaluator.cpp"):
        require(source in cmake, f"CMake does not compile {source}", failures)
    require("cargo_alarm_heartbeat_node" in cmake,
            "heartbeat executable is missing", failures)
    require("CargoBottomEstimate.msg" in messages and
            "CargoSafetyStatus.msg" in messages,
            "typed Cargo messages are not generated", failures)
    require("SCHEMA_VERSION=6" in safety_message and
            all(f"CODE_{name}" in safety_message for name in (
                "CLEAR", "LEVEL1_WARNING", "LEVEL2_WARNING",
                "SYSTEM_NOT_READY", "LOCALIZATION_INVALID",
                "GRAVITY_INVALID", "CARGO_INVALID", "OBSTACLE_INVALID",
                "INTERNAL_ERROR")) and
            all(name in safety_message for name in (
                "HOOK_ROLE_DISABLED", "HOOK_ROLE_REQUIRED",
                "HOOK_ROLE_AUXILIARY", "hook_signal_role",
                "hook_signal_conflict", "EVIDENCE_HAZARD_CONFIRMED",
                "EVIDENCE_TRACK_CONFIRMATION_PENDING",
                "EVIDENCE_SPARSE_PENDING", "EVIDENCE_SOURCE_UNRESOLVED",
                "evidence_state", "obstacle_track_id")),
            "CargoSafetyStatus v6 role/code/evidence contract is incomplete",
            failures)

    for include in ("cargo_bottom_fusion.hpp", "cargo_safety_evaluator.hpp",
                    "CargoBottomEstimate.h", "CargoSafetyStatus.h"):
        require(include in header, f"main header is missing {include}", failures)

    require('"/cargo_avoidance/bottom_estimate"' in node,
            "bottom estimate publisher is not advertised", failures)
    require('"/cargo_avoidance/safety_status"' in node,
            "safety status publisher is not advertised", failures)
    require('"/cargo_avoidance/raw_safety_status"' in node and
            '"/cargo_avoidance/raw_status_code"' in node,
            "raw cargo safety diagnostics are not advertised", failures)
    require("cargo_bottom_fusion_.update(observation)" in node,
            "CargoBottomFusion is not invoked by the runtime", failures)
    require("cargo_safety_evaluator_.evaluate(safety_input)" in node,
            "CargoSafetyEvaluator is not invoked by the runtime", failures)
    require("composeCargoSafetyStatus(" in node and
            len(re.findall(
                r"status\.requested_alarm_code\s*=(?!=)", node)) == 1,
            "final status code is not composed at one authoritative site",
            failures)
    require("CargoSafetyFault" in safety_header and
            "kLevel2OrFailSafeCode" not in safety_header and
            "raw_code" not in safety_header + safety_evaluator and
            "forceFailSafe" not in heartbeat,
            "physical warnings remain coupled to fail-safe semantics",
            failures)
    require('result.reason = "clear_no_external_obstacle"' in
            safety_evaluator and
            "result.warning_code = kSafeCode" in safety_evaluator,
            "valid empty obstacle ROI is not classified CLEAR", failures)
    require("message.obstacle_valid = result.input_valid &&" in node and
            "result.fault == CargoSafetyFault::NONE" in node and
            "status.obstacle_count > 0U" in node,
            "obstacle validity is still coupled to cluster presence", failures)
    require("input.obstacle_count == 0U" in heartbeat and
            "input.obstacle_count > 0U" in heartbeat,
            "heartbeat does not distinguish clear empty ROI from hazards",
            failures)
    for token in (
            "source_stamp_advanced", "duplicate_source_stamp",
            "warning_geometry_mismatch", "clear_geometry_mismatch"):
        require(token in heartbeat,
                f"heartbeat safety contract is missing {token}", failures)
    require("current_code_ = requested_code" in heartbeat and
            "if (!source_stamp_advanced)" in heartbeat,
            "fresh formal status is not applied immediately or duplicate stamp "
            "can still transition", failures)
    for forbidden in (
            "candidateConfirmed", "clear_confirm_pending",
            "clear_delay_started", "level1_exit_distance_m_",
            "level2_exit_distance_m_", "clearance_exit_m_",
            "clear_delay_sec_"):
        require(forbidden not in heartbeat,
                f"formal heartbeat still contains legacy state delay {forbidden}",
                failures)
    rollback_epoch = re.search(
        r"source_stamp_sec\s*\+\s*kTimeEpsilonSec\s*<\s*"
        r"last_source_stamp_sec_\s*\)\s*\{(?P<body>.*?)"
        r"return\s+forceCode\s*\(\s*kSystemNotReady\s*,\s*"
        r"\"source_time_rollback\"\s*\)", heartbeat, re.DOTALL)
    require(rollback_epoch is not None and
            "last_source_stamp_sec_ = source_stamp_sec" in
            rollback_epoch.group("body") and
            "last_source_progress_wall_sec_ = wall_now_sec" in
            rollback_epoch.group("body"),
            "source rollback does not establish a recoverable new epoch",
            failures)
    require("return {current_code_, false, \"heartbeat\"};" in heartbeat,
            "heartbeat tick can still synthesize a formal transition",
            failures)
    require(re.search(
                r"!result\.has_cluster_evidence.*?"
                r"CargoSafetyFault::OBSTACLE_EVIDENCE_INVALID.*?"
                r"obstacle_clusters_insufficient",
                safety_evaluator, re.DOTALL) is not None,
            "rejected obstacle clusters are not invalid evidence", failures)
    for token in (
            "obstacle_clusters_insufficient",
            "CargoSafetyFault::OBSTACLE_EVIDENCE_INVALID"):
        require(token in safety_evaluator,
                f"evaluator safety contract is missing {token}", failures)
    for value in (
            'level1_distance_m" value="3.0',
            'level2_distance_m" value="5.0',
            'minimum_vertical_clearance_m" value="0.80'):
        require(value in launch,
                f"launch status contract missing {value}", failures)
    require("updateAndPublishCargoSafetyPipeline(" in node,
            "formal Cargo pipeline has no runtime call site", failures)
    require("optional_signal_legacy_fallback" not in node and
            "hook_signal_disabled_legacy_mode" not in node,
            "auxiliary/disabled mode still fabricates LOADED evidence", failures)
    require("HookLoadSignalRole::REQUIRED" in node and
            "HookLoadSignalRole::AUXILIARY" in hook_policy and
            "gravity_required_fault" in hook_policy,
            "hook signal role-aware evidence policy is incomplete", failures)
    require("auxiliary_gravity_fault_forbidden" in heartbeat and
            "hook_supports_loaded" in heartbeat and
            "hook_supports_empty" in heartbeat,
            "heartbeat is not enforcing the schema-v4 role contract", failures)
    require("exclude_candidate_region" in hook_policy and
            "objects_channel_safe" in node and
            "hook_map_policy.use_formal_remove_box" in node,
            "auxiliary MapCommit can admit an unauthorized cargo candidate",
            failures)
    require('role: "auxiliary"' in live_config and
            "confirm_samples: 2" in live_config and
            "stale_timeout_sec: 2.50" in live_config and
            "consumer_timeout_sec: 3.00" in live_config and
            "valid_voltage_max_v: 6.0" in live_config and
            "diagnostic_disconnect_sec: 10.0" in live_config,
            "production Gravity auxiliary parameters are incomplete", failures)
    require("duplicate_sample_ignored" in
            read("src/ndt_slam/src/hook_load_state_filter.cpp"),
            "duplicate Gravity samples can advance confirmation", failures)
    rotation_failure = crane_constraint.find(
        'result.reason = "rotation_validation_failed";')
    rotation_return = crane_constraint.find("return result;", rotation_failure)
    rotation_failure_block = crane_constraint[rotation_failure:rotation_return]
    require(rotation_failure >= 0 and rotation_return >= 0 and
            "result.orthogonality_error =" not in rotation_failure_block and
            "result.determinant =" not in rotation_failure_block,
            "SO3 validation failure overwrites measured diagnostics", failures)
    gravity_filter = read("src/ndt_slam/src/hook_load_state_filter.cpp")
    require("has_seen_source_time_" in gravity_filter and
            "last_seen_source_time_sec_" in gravity_filter,
            "stale Gravity state forgets the last source timestamp", failures)
    require("lidar_no_cargo_evidence_.result().confirmed" in node and
            "cargo_state_.state == CargoState::EMPTY && !visual_conflict" not in node and
            "lidar_empty_confirm_frames: 3" in live_config,
            "AUXILIARY no-cargo output still trusts default CargoState EMPTY",
            failures)
    require("CargoObservationOutcome::UNKNOWN" in node and
            "CargoObservationOutcome::EMPTY_CONFIRMED" in node and
            "classifyCargoObservationOutcome" in hook_policy and
            "hook_fixed_cargo_.observation_valid" not in node and
            "empty_max_hag_candidate_points: 2" in live_config,
            "cargo detector still derives EMPTY from a generic detection failure",
            failures)
    empty_gate = node.find(
        "if (result.outcome == CargoObservationOutcome::EMPTY_CONFIRMED)")
    voxel_stage = node.find("vf.filter(*voxel_cloud)", empty_gate)
    cargo_outcome = node.find("classify_outcome(true)", voxel_stage)
    require(empty_gate >= 0 and voxel_stage > empty_gate and
            cargo_outcome > voxel_stage and
            'result.reject_reason = "too_few_points"' in node and
            'result.reject_reason = "no_clusters"' in node,
            "post-HAG detector failures can still advance EMPTY evidence",
            failures)
    require("evaluateSuspendedCargoLock" in node and
            "auxiliary_empty_delayed_confirmation" in hook_policy and
            "auxiliary_gravity_unavailable_strict_lidar" in hook_policy and
            "decision.allow_lock = input.lidar_lift_evidence" in hook_policy and
            "suspended_min_ground_clearance_m: 0.30" in live_config,
            "compact cargo lock is not role-aware", failures)
    candidate_branch = node.find("case HookCargoLockState::CANDIDATE:")
    geometry_branch = node.find(
        "case HookCargoLockState::GEOMETRY_CONFIRMING:", candidate_branch)
    locked_branch = node.find("case HookCargoLockState::LOCKED:", geometry_branch)
    candidate_code = node[candidate_branch:geometry_branch]
    geometry_code = node[geometry_branch:locked_branch]
    require(candidate_branch >= 0 and
            geometry_branch > candidate_branch and
            locked_branch > geometry_branch and
            "if (!candidate_policy.allow_candidate)" in candidate_code and
            "clearHookLock();" in candidate_code and
            "!candidate_policy.allow_lock" in geometry_code,
            "REQUIRED policy is not enforced inside the candidate state",
            failures)
    lock_function = node.find("void NdtSlamNode::updateHookCargoLock(")
    require(lock_function >= 0 and
            "std::unique_lock<std::mutex> hook_policy_guard" in
                node[lock_function:candidate_branch] and
            "hook_policy_guard.lock();" in node[lock_function:candidate_branch],
            "REQUIRED Gravity snapshot is not held across lock transition",
            failures)
    origin_capture = node.find("if (hook_is_empty && localization_evidence_valid")
    origin_record = node.find("recordEmptyHookOriginHeight(", origin_capture)
    tracking_update = node.find("if (hook_allows_tracking)", origin_capture)
    require(origin_capture >= 0 and origin_record >= 0 and tracking_update >= 0 and
            origin_record < tracking_update and
            "!hook_fixed_cargo_.lidar_lift_evidence" in
                node[origin_capture:origin_record],
            "AUXILIARY origin-height capture remains hidden behind tracking",
            failures)
    require("if (active_track && !cargo_origin_height_valid_)" in node and
            "cargo_origin_height_track_id_ = cargo_fusion_track_id_" in node,
            "late Gravity LOADED origin cannot attach to an active track",
            failures)
    require("publishPayloadTrackInfoFromFusion(last_cargo_bottom_result_" in node,
            "legacy payload compatibility output does not consume fusion", failures)
    require("SOURCE_ORIGIN_HEIGHT=5" in bottom_message and
            "SOURCE_DIRECT_TOP_FROZEN_THICKNESS=6" in bottom_message and
            "SCHEMA_VERSION=3" in bottom_message,
            "formal height sources are missing from message schema v3", failures)
    require("observation.origin_height_valid" in node and
            "observation.map_static_height_valid = origin_height" not in node,
            "track origin height is mislabeled as static-map evidence", failures)
    require("hook_observation_associated_current_" in node and
            "associated_detection.valid = false" in node and
            "LOST_HOLD recovery rejected" in node,
            "rejected or recovered cargo observations can bypass association",
            failures)
    require("processing_queue:" in live_config and "capacity: 1" in live_config,
            "latest-frame queue policy is not explicit in production config", failures)
    require("runtime_diag_.recordProcessed(msg->header.stamp.toSec())" in node and
            node.index("runtime_diag_.recordProcessed(msg->header.stamp.toSec())") <
            node.index("publishOdometry(publish_time"),
            "processed rate is still tied to successful odom publication", failures)
    require("diag_pending_ndt_record_ = std::move(ndt_rec)" in node,
            "diagnostic write overhead is still hidden from steady-state total_ms",
            failures)
    require(node.rindex("diag_pending_ndt_record_.total_ms") >
            node.index("rebuildActiveMapFromRecentKeyframes();"),
            "total_ms is finalized before periodic maintenance", failures)
    require(node.index("runtime_diag_.recordProcessed(msg->header.stamp.toSec())") >
            node.index("filtered_cloud->size() < 100"),
            "too-small frames are counted as processed without a diagnostic row",
            failures)
    require('"/cargo_avoidance/safety_status", 1' in node and
            "int status_queue_size = 1" in heartbeat and
            'status_queue_size" value="1' in launch,
            "formal safety/heartbeat chain is not latest-only", failures)
    require('use_cargo_visualizer" default="false' in launch,
            "legacy compatibility visualizer is still enabled by default",
            failures)
    require("Class: rviz/Marker" in rviz and
            "Marker Topic: /cargo_avoidance/fused_box_marker" in rviz,
            "RViz does not display the authoritative fused cargo marker",
            failures)

    cargo_track_policy = read(
        "src/ndt_slam/src/cargo_track_policy.cpp")
    require("GEOMETRY_CONFIRMING" in node and
            "summarizeCargoProvisionalLock" in node and
            "overall_lock_confidence" in node and
            "legacy_candidate_confirmation_disabled" not in node,
            "cargo can still reach formal lock without provisional identity confirmation",
            failures)
    require("scoreCargoCandidateIdentity" in node and
            "candidate_components_base" in node and
            "buildCargoComponentHypotheses" in node and
            "hypothesis_point_indices" in node and
            "cluster_indices.assign(1U, selected_component)" not in node,
            "cargo detector does not score single/merged component hypotheses",
            failures)
    require("reference_center = hook_lock_.live_pose.center_base" in node and
            '"center_too_far"' in node and
            "velocity extrapolation must not drag the gate" in node,
            "retained cargo association is not anchored to the last filtered pose",
            failures)
    temporal_filter = read(
        "src/ndt_slam/src/cargo_safety_temporal_filter.cpp")
    obstacle_tracker = read(
        "src/ndt_slam/src/cargo_obstacle_tracker.cpp")
    motion_corridor = read(
        "src/ndt_slam/src/cargo_motion_corridor.cpp")
    residual_classifier = read(
        "src/ndt_slam/src/cargo_residual_classifier.cpp")
    require("minimum_hazard_cluster_points" in temporal_filter and
            "maximum_centroid_step_m" in temporal_filter and
            "repeated_source_stamp_ignored" in temporal_filter and
            "pendingDecision(\"hazard_cluster_too_sparse\")" in
                temporal_filter and
            "hazard_transition_pending_hold_previous" not in
                temporal_filter and
            "clear_pending_hold_previous_hazard" not in temporal_filter and
            "hazard_confirm_frames: 3" in live_config and
            "clear_confirm_frames: 2" in live_config and
            "cargo_safety_temporal_filter_.update" in node,
            "17/18 do not require fresh spatially continuous cluster evidence",
            failures)
    require("src/cargo_obstacle_tracker.cpp" in cmake and
            "cargo_obstacle_tracker_.update" in node and
            "cluster_evidence.push_back" in safety_evaluator and
            "current_source_index" in obstacle_tracker and
            "consecutive_observations" in obstacle_tracker and
            "validated_consecutive_observations" in obstacle_tracker and
            "static_provenance_first_stamp_sec" in obstacle_tracker and
            "require_static_cargo_for_warning: true" in live_config and
            "static_cargo_min_voxel_points: 80" in live_config and
            "static_cargo_min_occupied_cells: 12" in live_config and
            "ExternalProvenance" in obstacle_tracker and
            "CARGO_MOVED_AWAY_PERSISTENCE" in obstacle_tracker and
            "cellOverlap" in obstacle_tracker and
            "centroid_map" in obstacle_tracker,
            "hazards are not confirmed by persistent map-frame identity",
            failures)
    require("src/cargo_motion_corridor.cpp" in cmake and
            "evaluateCargoMotionCorridor" in node and
            "immediate_near_field_m" in motion_corridor and
            "RADIAL_FALLBACK" in motion_corridor and
            "STATIONARY_GUARD" in motion_corridor and
            "MOTION_CORRIDOR" in motion_corridor and
            "clear_no_hazard_in_motion_corridor" in node,
            "cargo safety does not gate radial structures by a swept corridor",
            failures)
    require("src/cargo_residual_classifier.cpp" in cmake and
            "classifyCargoResidual" in node and
            "cargo_boundary_source_unresolved" in residual_classifier and
            "independent_external_static_provenance" in residual_classifier and
            "validation_shell_m" in residual_classifier and
            "minimum_motion_match_score" in residual_classifier and
            "isInsideExpandedCargo" not in safety_evaluator and
            "external_obstacle_cloud" in node,
            "near-zero cargo residual provenance is not fail-safe",
            failures)
    require("axis_aligned_yaw_after_lock: true" in live_config and
            "freeze_vertical_position_after_lock: false" in live_config and
            "track_vertical_from_top_surface: true" in live_config and
            "quantizeCargoAxialYawToOrthogonal" in node and
            "evaluateCargoTopSurfaceHeight" in node and
            "CargoVerticalPoseSource::DIRECT_TOP" in node,
            "formal cargo OBB yaw/vertical stability contract is missing",
            failures)
    require("marker.color.r = bottom.source" not in node and
            "marker.color.g = bottom.source" not in node and
            "Green means formal geometry + vertical authority" in node and
            'name="use_cargo_visualizer" default="false"' in launch,
            "RViz cargo authority color or single-marker contract regressed",
            failures)
    require("top_surface_minus_frozen_thickness" in cargo_track_policy and
            "CargoBottomSource::DIRECT_TOP_FROZEN_THICKNESS" in
                read("src/ndt_slam/src/cargo_bottom_fusion.cpp") and
            "evaluateCargoFormalHeight" in node and
            "consistent_partial_height_observation" in node and
            "rigid_shape_height_mismatch" not in node,
            "pre-lift thickness/top-surface height authority is incomplete",
            failures)
    require("live_vertical_pose_evidence_stamp" in node and
            "tracking_residual" in read(
                "src/ndt_slam/src/cargo_rigid_geometry.cpp") and
            "horizontal_tracking_residual_m" in node,
            "live vertical evidence or tracking residual uncertainty is missing",
            failures)
    require("formal_xy_evidence_hold_sec" in node + live_config and
            "formal_vertical_evidence_hold_sec" in node + live_config and
            "formal_pose_hold_sec" not in node + live_config and
            "maximum_height_age_sec" not in
                safety_header + safety_evaluator + live_config and
            "formal_vertical_measurement" in node and
            "CargoVerticalPoseSource::PREDICTION" not in
                node[node.find("const bool formal_vertical_measurement"):
                     node.find("if (formal_vertical_measurement)")],
            "formal vertical evidence lifetime is duplicated or prediction-fed",
            failures)
    for topic in ("/cargo_avoidance/candidate_components",
                  "/cargo_avoidance/selected_candidate_cloud",
                  "/cargo_avoidance/predicted_obb",
                  "/cargo_avoidance/self_removed_cloud",
                  "/cargo_avoidance/external_obstacle_cloud",
                  "/cargo_avoidance/most_dangerous_cluster"):
        require(topic in node, f"cargo debug topic missing: {topic}", failures)
    require("cargo_track_not_initialized" in node and
            "evidence_initialized" in node and
            "CargoSafetyProtocol::kSystemNotReady" in node,
            "pre-authority cargo startup is not mapped to code 30", failures)
    require("self_cargo_point_match_radius_m" in node + live_config and
            "self_cargo_point_match_radius_m: 0.15" in live_config and
            "self_rigging_radius_m" in node + live_config and
            "identity_self_tree.nearestKSearch" in node and
            "inside_identity_neighborhood" in node and
            "isCargoIdentityPointMatch" in node and
            "inside_rigging" in node and
            "cargo_identity_self_removed_points_" in node,
            "identity-selected cargo returns can still leak into obstacles",
            failures)
    require("dangerous_cluster_points" in runtime_header and
            "nearest_obstacle_x" in runtime_header and
            "conservative_clearance_m" in runtime_header and
            "raw_warning_code" in runtime_header and
            "confirmed_warning_code" in runtime_header and
            "temporal_candidate_code" in runtime_header and
            "used_previous_confirmation" in runtime_header and
            "obstacle_track_id" in runtime_header and
            "obstacle_track_velocity" in runtime_header and
            "obstacle_static_provenance_streak" in runtime_header and
            "obstacle_provenance_type" in runtime_header and
            "obstacle_track_cell_overlap" in runtime_header and
            "safety_spatial_mode" in runtime_header and
            "corridor_rejected_clusters" in runtime_header and
            "residual_unknown_clusters" in runtime_header and
            "requested_alarm_code" in runtime_header and
            "candidate_present" in runtime_header and
            "candidate_authoritative" in runtime_header and
            "candidate_merged_components" in runtime_header and
            "candidate_gate_failure" in runtime_header and
            "vertical_evidence_age_sec" in runtime_header and
            "top_support_points" in runtime_header and
            "top_surface_coverage" in runtime_header and
            "locked_obb_support_ratio" in runtime_header and
            "vertical_reject_reason" in runtime_header and
            "association_reject_reason" in runtime_header and
            "evaluateCargoFrozenObbSupport" in node,
            "cargo safety evidence is incomplete in frame diagnostics",
            failures)
    require("NDT_SKIPPED_BOOTSTRAP" in node and
            "fitnessStats().count() >= 30U" in node and
            "last_ndt_fitness_ >= map_commit_max_fitness_" in node,
            "NDT skipped-state or mature absolute fitness spike gate is missing",
            failures)

    require("cargo_alarm_heartbeat_node" in launch,
            "production launch does not start heartbeat", failures)
    require("SAFETY_PENDING" in heartbeat and
            "SAFETY_PENDING_SUMMARY" in heartbeat and
            "SAFETY_FAULT_PERSISTENT" not in heartbeat and
            "reason.find" not in heartbeat and
            "pending_error_sec" in heartbeat + launch and
            "pending_repeat_sec" in heartbeat + launch and
            "SAFETY_WARN" in heartbeat and
            "warning_repeat_sec" in heartbeat + launch and
            "CARGO_SAFETY_INTERNAL" in node and
            "CARGO_SAFETY_PENDING_SUMMARY" not in node and
            "CARGO_SAFETY_PERSISTENT" not in node,
            "heartbeat operator ownership or urgent warning logs are missing",
            failures)
    for value in ("/cargo_avoidance/safety_status",
                  "/cargo_avoidance/status_code",
                  'publish_legacy_alarm_topic" value="false',
                  'stale_timeout_sec" value="0.8'):
        require(value in launch, f"launch contract missing {value}", failures)

    status_code_advertisers = []
    for path in (ROOT / "src/ndt_slam/src").glob("*.cpp"):
        text = path.read_text(encoding="utf-8", errors="replace")
        if '"/cargo_avoidance/status_code"' in text:
            status_code_advertisers.append(path.name)
    require(status_code_advertisers == ["cargo_alarm_heartbeat_node.cpp"],
            "simple status topic must have exactly one source; found " +
            repr(status_code_advertisers), failures)
    require('topic: "/gravity"' in live_config,
            "gravity input topic is not the production /gravity spelling",
            failures)
    require('as<std::string>("/gravity")' in hook_node and
            "std_msgs::Float32" in hook_node,
            "gravity node default topic/type contract is incorrect", failures)

    if "motion_compensation:" in merger:
        motion_block = merger.split("motion_compensation:", 1)[1].split(
            "lidars:", 1)[0]
        require("enabled: false" in motion_block,
                "unvalidated odom-feedback motion compensation must default OFF",
                failures)

    for obsolete in ("map_diff_points_map", "map_static_points_map",
                     "diag_allow_map_commit"):
        require(obsolete not in node and obsolete not in header,
                f"obsolete/inconsistent symbol remains: {obsolete}", failures)

    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1
    print("PASS: Cargo bottom -> safety status -> heartbeat status-code chain is wired")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
