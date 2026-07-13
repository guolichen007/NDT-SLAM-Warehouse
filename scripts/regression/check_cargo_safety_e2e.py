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
    require("SCHEMA_VERSION=3" in safety_message and
            all(f"CODE_{name}" in safety_message for name in (
                "CLEAR", "LEVEL1_WARNING", "LEVEL2_WARNING",
                "SYSTEM_NOT_READY", "LOCALIZATION_INVALID",
                "GRAVITY_INVALID", "CARGO_INVALID", "OBSTACLE_INVALID",
                "INTERNAL_ERROR")),
            "CargoSafetyStatus v3 code contract is incomplete", failures)

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
