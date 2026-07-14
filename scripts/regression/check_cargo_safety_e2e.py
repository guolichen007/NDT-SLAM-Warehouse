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
    require("SCHEMA_VERSION=4" in safety_message and
            all(f"CODE_{name}" in safety_message for name in (
                "CLEAR", "LEVEL1_WARNING", "LEVEL2_WARNING",
                "SYSTEM_NOT_READY", "LOCALIZATION_INVALID",
                "GRAVITY_INVALID", "CARGO_INVALID", "OBSTACLE_INVALID",
                "INTERNAL_ERROR")) and
            all(name in safety_message for name in (
                "HOOK_ROLE_DISABLED", "HOOK_ROLE_REQUIRED",
                "HOOK_ROLE_AUXILIARY", "hook_signal_role",
                "hook_signal_conflict")),
            "CargoSafetyStatus v4 role/code contract is incomplete", failures)

    for include in ("cargo_bottom_fusion.hpp", "cargo_safety_evaluator.hpp",
                    "CargoBottomEstimate.h", "CargoSafetyStatus.h"):
        require(include in header, f"main header is missing {include}", failures)

    require('"/cargo_avoidance/bottom_estimate"' in node,
            "bottom estimate publisher is not advertised", failures)
    require('"/cargo_avoidance/safety_status"' in node,
            "safety status publisher is not advertised", failures)
    require("cargo_bottom_fusion_.update(observation)" in node,
            "CargoBottomFusion is not invoked by the runtime", failures)
    require("cargo_safety_evaluator_.evaluate(safety_input)" in node,
            "CargoSafetyEvaluator is not invoked by the runtime", failures)
    require("composeCargoSafetyStatus(" in node and
            node.count("status.requested_alarm_code =") == 1,
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
    require("last_cargo_safety_result_.input_valid &&" in node and
            "last_cargo_safety_result_.fault == CargoSafetyFault::NONE" in node and
            "status.obstacle_count > 0U" in node,
            "obstacle validity is still coupled to cluster presence", failures)
    require("input.obstacle_count == 0U" in heartbeat and
            "input.obstacle_count > 0U" in heartbeat,
            "heartbeat does not distinguish clear empty ROI from hazards",
            failures)
    for token in (
            "source_stamp_advanced", "fresh_source_evidence",
            "clear_confirm_pending", "clear_delay_started",
            "warning_geometry_mismatch", "clear_geometry_mismatch"):
        require(token in heartbeat,
                f"heartbeat safety contract is missing {token}", failures)
    require(not re.search(
                r"applyCandidate\s*\([^;]*?\btrue\s*,\s*\"fresh_status\"",
                heartbeat, re.DOTALL),
            "heartbeat treats every received status as fresh evidence",
            failures)
    clear_confirmation_calls = re.findall(
        r"candidateConfirmed\s*\(\s*kClear\s*,\s*"
        r"fresh_source_evidence\s*\)", heartbeat)
    require(len(clear_confirmation_calls) == 1,
            "CLEAR confirmation must have exactly one common counting entry",
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
    require("requires_clear_confirmation" in heartbeat and
            "current_code_ != kClear" in heartbeat and
            "leaving_warning" not in heartbeat,
            "CLEAR confirmation is not required for every non-clear state",
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
    require("evaluateSuspendedCargoLock" in node and
            "auxiliary_empty_delayed_confirmation" in hook_policy and
            "auxiliary_gravity_unavailable_strict_lidar" in hook_policy,
            "compact cargo lock is not role-aware", failures)
    origin_capture = node.find("if (hook_is_empty && localization_evidence_valid")
    origin_record = node.find("recordEmptyHookOriginHeight(", origin_capture)
    tracking_update = node.find("if (hook_allows_tracking)", origin_capture)
    require(origin_capture >= 0 and origin_record >= 0 and tracking_update >= 0 and
            origin_record < tracking_update,
            "AUXILIARY origin-height capture remains hidden behind tracking",
            failures)
    require("publishPayloadTrackInfoFromFusion(last_cargo_bottom_result_" in node,
            "legacy payload compatibility output does not consume fusion", failures)
    require("SOURCE_ORIGIN_HEIGHT=5" in bottom_message and
            "SCHEMA_VERSION=2" in bottom_message,
            "origin height source is missing from message schema v2", failures)
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

    require("cargo_alarm_heartbeat_node" in launch,
            "production launch does not start heartbeat", failures)
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
