#!/usr/bin/env python3
"""Static contract check for the production Cargo typed-message chain."""

from __future__ import annotations

from pathlib import Path
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
    launch = read("src/ndt_slam/launch/warehouse_live_longterm_mapping.launch")
    messages = read("src/lidar_slam2_msgs/CMakeLists.txt")
    bottom_message = read("src/lidar_slam2_msgs/msg/CargoBottomEstimate.msg")
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
                  "/cargo_avoidance/alarm_code",
                  'stale_timeout_sec" value="0.8'):
        require(value in launch, f"launch contract missing {value}", failures)

    alarm_advertisers = []
    for path in (ROOT / "src/ndt_slam/src").glob("*.cpp"):
        text = path.read_text(encoding="utf-8", errors="replace")
        if '"/cargo_avoidance/alarm_code"' in text:
            alarm_advertisers.append(path.name)
    require(alarm_advertisers == ["cargo_alarm_heartbeat_node.cpp"],
            "formal alarm topic must have exactly one source; found " +
            repr(alarm_advertisers), failures)

    motion_block = merger.split("motion_compensation:", 1)[-1].split("lidars:", 1)[0]
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
    print("PASS: Cargo bottom -> safety status -> heartbeat alarm chain is wired")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
