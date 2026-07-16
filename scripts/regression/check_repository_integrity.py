#!/usr/bin/env python3
"""Reject tracked source/config files polluted by tool output or bad encoding."""

from __future__ import annotations

from pathlib import Path
import re
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
SCANNED_SUFFIXES = {
    ".c",
    ".cc",
    ".cmake",
    ".cpp",
    ".h",
    ".hpp",
    ".launch",
    ".msg",
    ".py",
    ".yaml",
    ".yml",
}
SKIPPED_PARTS = {
    ".git",
    "build",
    "devel",
    "install",
    "log",
    "test_artifacts",
}


def tracked_paths() -> list[Path]:
    completed = subprocess.run(
        ["git", "ls-files", "-z"],
        cwd=ROOT,
        check=True,
        stdout=subprocess.PIPE,
    )
    paths: list[Path] = []
    for raw_name in completed.stdout.split(b"\0"):
        if not raw_name:
            continue
        name = raw_name.decode("utf-8")
        relative = Path(name)
        if any(part in SKIPPED_PARTS for part in relative.parts):
            continue
        if (relative.name == "CMakeLists.txt" or
                relative.suffix.lower() in SCANNED_SUFFIXES):
            paths.append(relative)
    return paths


def pollution_patterns() -> list[tuple[str, re.Pattern[str]]]:
    literals = {
        "token truncation marker": "tokens " + "truncated",
        "response truncation header": "Response output was " + "truncated",
        "citation transport marker": "Citation " + "Marker:",
        "resource transport marker": "Resource " + "uri:",
        "tool output header": "Tool " + "output:",
        "tool line-count header": "Total output " + "lines:",
        "tool truncation warning": "Warning: truncated " + "output",
    }
    patterns = [
        (name, re.compile(re.escape(value), re.IGNORECASE))
        for name, value in literals.items()
    ]
    patterns.extend([
        (
            "partial line-range header",
            re.compile(r"Showing\s+\d+\s+of\s+\d+\s+lines", re.IGNORECASE),
        ),
        (
            "unicode token truncation marker",
            re.compile(r"\u2026\s*\d+\s+tokens\s+truncated\s*\u2026",
                       re.IGNORECASE),
        ),
        (
            "merge conflict marker",
            re.compile(r"^(?:<{7}|={7}|>{7})(?: .*)?$", re.MULTILINE),
        ),
    ])
    return patterns


def runtime_so3_contract_failures() -> list[str]:
    """Guard known external registration/optimizer matrix boundaries."""
    failures: list[str] = []
    required_tokens = {
        Path("src/ndt_slam/src/ndt_slam.cpp"): (
            "makeSafeSE3FromMatrix(ndt_matrix)",
            "makeSafeSE3FromMatrix(refined_matrix)",
            '"ndt_safe_se3"',
            '"icp_matrix"',
        ),
        Path("src/ndt_slam/src/ndt_relocalizer.cpp"): (
            "makeSafeSE3FromMatrix(transform.cast<double>())",
        ),
        Path("src/ndt_slam/src/loop_closure.cpp"): (
            "makeSafeSE3FromMatrix(pose.matrix())",
            "makeSafeSE3FromMatrix(transformation.cast<double>())",
        ),
        Path("src/ndt_slam/src/rigid_transform_conversion.cpp"): (
            "Sophus::SE3d(Sophus::SO3d(quaternion), translation)",
        ),
    }
    forbidden_tokens = {
        Path("src/ndt_slam/src/ndt_slam.cpp"): (
            "Sophus::SE3d(result_ortho)",
            "Sophus::SE3d refined(refined_matrix)",
            "Sophus::SE3d(refined_matrix)",
            "Sophus::SE3d(initial_guess.cast<double>())",
        ),
        Path("src/ndt_slam/src/ndt_relocalizer.cpp"): (
            "Sophus::SE3d(result)",
        ),
        Path("src/ndt_slam/src/loop_closure.cpp"): (
            "Sophus::SE3d(pose.matrix())",
            "Sophus::SE3d refined(transformation_double)",
        ),
    }

    for relative, tokens in required_tokens.items():
        try:
            text = (ROOT / relative).read_text(encoding="utf-8")
        except OSError as error:
            failures.append(f"{relative.as_posix()}: cannot read: {error}")
            continue
        for token in tokens:
            if token not in text:
                failures.append(
                    f"{relative.as_posix()}: missing SO3 boundary token {token!r}")

    for relative, tokens in forbidden_tokens.items():
        try:
            text = (ROOT / relative).read_text(encoding="utf-8")
        except OSError:
            continue
        for token in tokens:
            if token in text:
                failures.append(
                    f"{relative.as_posix()}: unsafe external matrix "
                    f"construction remains: {token!r}")
    return failures


def pending_origin_contract_failures() -> list[str]:
    failures: list[str] = []
    node_path = Path("src/ndt_slam/src/ndt_slam.cpp")
    policy_path = Path(
        "src/ndt_slam/src/pending_origin_binding_policy.cpp")
    config_path = Path("src/ndt_slam/config/live_longterm_mapping.yaml")
    try:
        node = (ROOT / node_path).read_text(encoding="utf-8")
        policy = (ROOT / policy_path).read_text(encoding="utf-8")
        config = (ROOT / config_path).read_text(encoding="utf-8")
    except OSError as error:
        return [f"pending origin contract cannot be read: {error}"]

    for token in (
            "evaluatePendingOriginBinding",
            "PendingOriginAction::KEEP_WAITING_FOR_LIDAR_TIME",
            "PendingOriginAction::ATTACH",
            "PendingOriginAction::DISCARD_EXPIRED",
            "PendingOriginAction::DISCARD_SPATIAL_MISMATCH",
            "PendingOriginAction::DISCARD_INVALID"):
        if token not in node + policy:
            failures.append(f"pending origin contract missing {token!r}")

    keep_case = re.search(
        r"case\s+PendingOriginAction::KEEP_WAITING_FOR_LIDAR_TIME\s*:"
        r"(?P<body>.*?)case\s+PendingOriginAction::ATTACH\s*:",
        node,
        re.DOTALL,
    )
    if keep_case is None:
        failures.append("pending origin KEEP branch is missing")
    elif "pending_origin_height_valid_ = false" in keep_case.group("body"):
        failures.append("pending origin KEEP branch clears pending evidence")

    for token in (
            "origin_future_stamp_tolerance_sec: 0.05",
            "source_time_rollback",
            "lidar_time_rollback"):
        if token not in node + config:
            failures.append(f"pending origin epoch/config guard missing {token!r}")
    return failures


def stationary_motion_contract_failures() -> list[str]:
    failures: list[str] = []
    node_path = Path("src/ndt_slam/src/ndt_slam.cpp")
    policy_path = Path(
        "src/ndt_slam/src/stationary_motion_policy.cpp")
    ekf_path = Path("src/ndt_slam/src/crane_motion_ekf.cpp")
    config_path = Path("src/ndt_slam/config/live_longterm_mapping.yaml")
    try:
        node = (ROOT / node_path).read_text(encoding="utf-8")
        policy = (ROOT / policy_path).read_text(encoding="utf-8")
        ekf = (ROOT / ekf_path).read_text(encoding="utf-8")
        config = (ROOT / config_path).read_text(encoding="utf-8")
    except OSError as error:
        return [f"stationary motion contract cannot be read: {error}"]

    required = (
        "RuntimeMotionState::STATIONARY_HOLD",
        "RuntimeMotionState::MOVING_CONFIRM",
        "RuntimeMotionState::CATCH_UP",
        "DRIFT_ONLY_REJECTED",
        "updateStationaryMotionState",
        "enterStationaryState",
        "exitStationaryState",
        "resetStationaryState",
        "allow_runtime_local_map_update_",
        "allow_persistent_map_commit_",
        "catch_up_blocks_local_map",
        "catch_up_blocks_maps",
        "applyStationaryConstraint",
        "stationary_policy:",
        "exit_evidence_window_sec",
        "exit_force_anchor_drift_m",
        "ANCHOR_DRIFT_FAILSAFE_START_CATCH_UP",
    )
    combined = node + policy + ekf + config
    for token in required:
        if token not in combined:
            failures.append(f"stationary motion contract missing {token!r}")

    for token in (
            "raw_drift",
            "motion_gate_stationary_drift_ignore_radius_",
            "unfreeze_initial_map_commit"):
        if token in node:
            failures.append(
                f"drift-only stationary exit token remains: {token!r}")

    policy_call = node.find("updateStationaryMotionState(")
    local_map_write = node.find("*local_map_ += *transformed")
    if policy_call < 0 or local_map_write < 0 or policy_call > local_map_write:
        failures.append(
            "stationary policy must run before runtime local-map writes")

    catch_up_gate = re.search(
        r"shouldCommitKeyframe\(.*?RuntimeMotionState::CATCH_UP.*?return false;",
        node,
        re.DOTALL,
    )
    if catch_up_gate is None:
        failures.append("CATCH_UP persistent MapCommit gate is missing")

    if "if (state_ == RuntimeMotionState::STATIONARY_HOLD)" not in policy:
        failures.append(
            "MOVING_CONFIRM must not share the stationary position constraint")

    return failures


def runtime_visualization_contract_failures() -> list[str]:
    failures: list[str] = []
    node_path = Path("src/ndt_slam/src/ndt_slam.cpp")
    header_path = Path("src/ndt_slam/include/ndt_slam/ndt_slam.hpp")
    marker_path = Path(
        "src/ndt_slam/src/cargo_marker_lifecycle.cpp")
    config_path = Path("src/ndt_slam/config/live_longterm_mapping.yaml")
    try:
        node = (ROOT / node_path).read_text(encoding="utf-8")
        header = (ROOT / header_path).read_text(encoding="utf-8")
        marker = (ROOT / marker_path).read_text(encoding="utf-8")
        config = (ROOT / config_path).read_text(encoding="utf-8")
    except OSError as error:
        return [f"runtime visualization contract cannot be read: {error}"]

    combined = node + header + marker + config
    required = (
        "mapPublicationThread",
        "requestMapPublication(",
        "map_maintenance_max_deferral_frames_",
        "force_map_maintenance_timeslice",
        "max_deferral_frames: 5",
        "CargoMarkerLifecycle",
        "freeze_geometry_after_lock: true",
        "axis_aligned_yaw_after_lock: true",
        "freeze_vertical_position_after_lock: true",
        "current_rigid_cargo_geometry_",
        "rigid_suspended_track",
        "last_good_height_hold",
        "last_good_localization_hold",
        "marker.action = lifecycle.show",
        "cargo_marker:",
        "estimateCargoOrientedFootprint",
        "LockedCargoShape",
        "LiveCargoPose",
        "buildCurrentRigidCargoGeometryForPose",
        "orientation_enabled: true",
        "orientation_min_geometric_aspect_ratio: 1.20",
        "orientation_min_eigenvalue_ratio: 1.44",
        "LockedCargoHeightAction::INITIALIZE_ONCE",
        "updateLockedHeightAfterAssociation",
        "MapPublicationSnapshot",
        "MapLayerBundle",
        "latest_completed_map_bundle_",
        "map_layer_generation_",
        "advanceMapLayerGenerationLocked",
        "captureMapPublicationSnapshot",
        "publishMapPublicationSnapshot",
        "snapshot.generation",
        "message.header.seq",
        'tight_box_marker.header.frame_id = map_frame_',
        "tight_box_marker.pose.orientation.z = marker_orientation_z",
        "buildCleanMapFromSnapshot",
        "clean_map_rebuild_thread_",
        "clean_map_rebuild_result_ready_",
        "source_objects_version",
        "objects_map_content_version_",
        "getDenyCellsSnapshot",
        "CleanMapBuildAction::APPLY",
        "CleanMapBuildAction::PUBLISH_SNAPSHOT_ONLY",
        "result.bundle.objects_clean",
        "evaluateCargoFormalUse",
        "evidence_stamp_sec",
        "formal_hold_sec",
        "propagateHeldCargoPose",
        "updateCargoLivePoseStep",
    )
    for token in required:
        if token not in combined:
            failures.append(
                f"runtime visualization contract missing {token!r}")

    marker_function = re.search(
        r"void NdtSlamNode::publishCargoFusionMarker\(.*?\n}\n",
        node,
        re.DOTALL,
    )
    if marker_function is None:
        failures.append("authoritative Cargo marker function is missing")
    elif "marker.action = bottom.valid" in marker_function.group(0):
        failures.append("single invalid CargoBottom frame still deletes marker")

    for forbidden in (
            "if (display_map_->empty()) return",
            "if (ground_map_->empty()) return",
            "if (objects_map_->empty()) return",
            "if (objects_clean_map_->empty()) return",
            "rebuildCleanMap(!force_timeslice)",
            "void NdtSlamNode::publishMap()",
            "void NdtSlamNode::publishDisplayMap()",
            "void NdtSlamNode::publishGroundMap()",
            "void NdtSlamNode::publishObjectsMap()",
            "void NdtSlamNode::publishObjectsCleanMap()"):
        if forbidden in node:
            failures.append(
                f"latched map publication still skips empty layer: {forbidden!r}")

    for legacy_declaration in (
            "void publishMap();",
            "void publishDisplayMap();",
            "void publishGroundMap();",
            "void publishObjectsMap();",
            "void publishObjectsCleanMap();"):
        if legacy_declaration in header:
            failures.append(
                f"legacy direct map publisher remains declared: "
                f"{legacy_declaration!r}")

    for publisher in (
            "map_pub_.publish(",
            "display_map_pub_.publish(",
            "ground_map_pub_.publish(",
            "objects_map_pub_.publish(",
            "objects_clean_map_pub_.publish("):
        call_count = len(re.findall(
            rf"(?<![A-Za-z0-9_]){re.escape(publisher)}", node))
        if call_count != 1:
            failures.append(
                f"map publisher must have exactly one bundled call site: "
                f"{publisher!r}")

    formal_function = re.search(
        r"CargoFormalUseDecision evaluateCargoFormalUse\(.*?\n\}",
        (ROOT / Path(
            "src/ndt_slam/src/cargo_rigid_geometry.cpp")).read_text(
                encoding="utf-8"),
        re.DOTALL,
    )
    if formal_function is None:
        failures.append("formal Cargo evidence-age policy is missing")
    elif "if (!lost_hold)" in formal_function.group(0):
        failures.append(
            "LOCKED lifecycle state still bypasses formal evidence age")
    return failures


def registration_source_contract_failures() -> list[str]:
    failures: list[str] = []
    paths = (
        Path("src/ndt_slam/include/ndt_slam/registration_cloud_builder.hpp"),
        Path("src/ndt_slam/src/registration_cloud_builder.cpp"),
        Path("src/ndt_slam/src/ndt_slam.cpp"),
        Path("src/ndt_slam/include/ndt_slam/runtime_diagnostics.hpp"),
        Path("src/ndt_slam/src/runtime_diagnostics.cpp"),
        Path("src/ndt_slam/config/live_longterm_mapping.yaml"),
    )
    try:
        text = "\n".join(
            (ROOT / path).read_text(encoding="utf-8") for path in paths)
    except OSError as error:
        return [f"registration source contract cannot be read: {error}"]

    required = (
        "RegistrationCloudBuildResult",
        "STRUCTURE_RICH",
        "STRUCTURE_RECOVERY",
        "GROUND_AUGMENTED",
        "INSUFFICIENT_STRUCTURE",
        "ground_max_fraction",
        "structure_quality_valid",
        "registration_mode",
        "static_object_points",
        "uncertain_candidate_points",
        "ground_points",
        "ground_fraction",
        '"INSUFFICIENT_STRUCTURE"',
        "predictWithoutMeasurement",
        "allow_full_ground_fallback: false",
    )
    for token in required:
        if token not in text:
            failures.append(
                f"registration source contract missing {token!r}")

    for token in ('"FULL_GROUND"', "fallback to full ground"):
        if token in text:
            failures.append(
                f"unsafe full-ground fallback remains: {token!r}")

    node = (ROOT / paths[2]).read_text(encoding="utf-8")
    partition = node.find("partitionRegistrationObjects(")
    build = node.find("buildRegistrationCloud(", partition)
    if partition < 0 or build < 0 or partition > build:
        failures.append(
            "registration cloud must be built from the post-HumanFilter partition")

    return failures


def ndt_observability_contract_failures() -> list[str]:
    failures: list[str] = []
    paths = (
        Path("src/ndt_slam/include/ndt_slam/ndt_observability.hpp"),
        Path("src/ndt_slam/src/ndt_observability.cpp"),
        Path("src/ndt_slam/src/crane_motion_ekf.cpp"),
        Path("src/ndt_slam/src/ndt_slam.cpp"),
        Path("src/ndt_slam/test/ndt_observability_test.cpp"),
        Path("src/ndt_slam/config/live_longterm_mapping.yaml"),
    )
    try:
        texts = {
            path: (ROOT / path).read_text(encoding="utf-8")
            for path in paths
        }
    except OSError as error:
        return [f"NDT observability contract cannot be read: {error}"]
    combined = "\n".join(texts.values())

    required = (
        "static_local_xy_normals",
        "local-normal information proxy, not the internal NDT Hessian",
        "information += anisotropy * normal * normal.transpose()",
        "directions * directional_variance * directions.transpose()",
        "moderate_weak_inflation: 5.0",
        "severe_weak_inflation: 20.0",
        "frame_severe_degeneracy",
        "observability_weak_direction_x",
        "HorizontalWallsHaveWeakXDirection",
        "RotatedNinetyDegreesHasWeakYDirection",
        "FortyFiveDegreeWeakDirectionIsCoordinateFree",
        "UniformPerpendicularStructureIsObservable",
    )
    for token in required:
        if token not in combined:
            failures.append(
                f"NDT observability contract missing {token!r}")

    covariance_function = re.search(
        r"buildObservabilityAwareMeasurementCovariance\(.*?\n\}",
        texts[paths[1]],
        re.DOTALL,
    )
    if covariance_function is None:
        failures.append("observability-aware covariance function is missing")
    elif "directional_variance(1, 1) = base * weak_inflation" not in (
            covariance_function.group(0)):
        failures.append("weak-direction covariance is not anisotropic")
    return failures


def runtime_console_contract_failures() -> list[str]:
    failures: list[str] = []
    paths = (
        Path("src/ndt_slam/config/live_longterm_mapping.yaml"),
        Path("src/ndt_slam/include/ndt_slam/runtime_diagnostics.hpp"),
        Path("src/ndt_slam/src/runtime_diagnostics.cpp"),
        Path("src/ndt_slam/src/ndt_slam.cpp"),
        Path("src/ndt_slam/src/PointCloudMerger.cpp"),
        Path("src/ndt_slam/config/merger_params.yaml"),
        Path("src/ndt_slam/test/runtime_diagnostics_test.cpp"),
        Path("src/ndt_slam/CMakeLists.txt"),
    )
    try:
        texts = {
            path: (ROOT / path).read_text(encoding="utf-8")
            for path in paths
        }
    except OSError as error:
        return [f"runtime console contract cannot be read: {error}"]

    config = texts[paths[0]]
    combined = "\n".join(texts.values())
    required_config = (
        "health_period_sec: 10.0",
        "risk_repeat_period_sec: 10.0",
        "summary_interval_sec: 10.0",
        "debug_perf: false",
        "csv_enabled: true",
        "console_health_enabled: false",
        "console_risk_enabled: false",
        "cargo_console_enabled: true",
    )
    for token in required_config:
        if token not in config:
            failures.append(
                f"production console config missing {token!r}")

    required_runtime = (
        "runtime_frames.csv",
        "cargo_frames.csv",
        "writeNdtFrame",
        "writeCargoFrame",
        "[CARGO_MONITOR]",
        "[CARGO_SAFETY]",
        "console_health_enabled: false",
        "[PIPELINE_HEALTH]",
        "updatePipelineRisk",
        "clearPipelineRisk",
        "shouldEmitConsoleRisk",
        "clearConsoleRisk",
        '"ENTER"',
        '"CHANGE"',
        '"REPEAT"',
        '"CLEAR"',
        "[MERGER_RISK_",
        "[MERGER_RISK_CLEAR]",
        "diagnostic_log_period_sec_",
        "RuntimeDiagnosticsTest",
        "runtime_diagnostics_test",
    )
    for token in required_runtime:
        if token not in combined:
            failures.append(
                f"runtime console contract missing {token!r}")

    forbidden = (
        "logPipelineRiskFrameOverrun",
        "logPipelineRiskSustainedOverrun",
        "[PIPELINE_RISK] reason=FRAME_OVERRUN",
        "[PIPELINE_RISK] reason=SUSTAINED_OVERRUN",
        "fallback to full ground",
    )
    for token in forbidden:
        if token in combined:
            failures.append(
                f"per-frame or unsafe runtime contract remains: {token!r}")

    return failures


def asynchronous_map_commit_contract_failures() -> list[str]:
    failures: list[str] = []
    node_path = Path("src/ndt_slam/src/ndt_slam.cpp")
    header_path = Path("src/ndt_slam/include/ndt_slam/ndt_slam.hpp")
    try:
        node = (ROOT / node_path).read_text(encoding="utf-8")
        header = (ROOT / header_path).read_text(encoding="utf-8")
    except OSError as error:
        return [f"asynchronous MapCommit contract cannot be read: {error}"]

    combined = node + header
    required = (
        "struct MapCommitJob",
        "map_commit_queue_capacity_ = 2U",
        "enqueueMapCommitJob(filtered_cloud, final_pose, publish_time)",
        "std::thread(&NdtSlamNode::mapCommitThread, this)",
        "map_commit_lifecycle_mutex_",
        "job.lifecycle_epoch",
        "map_commit_queue_.back() = std::move(job)",
        "consumeMapCommitCompletion()",
        "job.formal_footprint",
    )
    for token in required:
        if token not in combined:
            failures.append(
                f"asynchronous MapCommit contract missing {token!r}")

    owner_gate = node.find("if (allow_map_commit)")
    owner_call_window = (
        node[owner_gate:owner_gate + 1200] if owner_gate >= 0 else "")
    if "commitKeyFrameWithDynamicFiltering(" in owner_call_window:
        failures.append(
            "LiDAR owner thread still invokes the synchronous MapCommit pipeline")
    return failures


def main() -> int:
    failures: list[str] = runtime_so3_contract_failures()
    failures.extend(pending_origin_contract_failures())
    failures.extend(stationary_motion_contract_failures())
    failures.extend(registration_source_contract_failures())
    failures.extend(ndt_observability_contract_failures())
    failures.extend(runtime_console_contract_failures())
    failures.extend(runtime_visualization_contract_failures())
    failures.extend(asynchronous_map_commit_contract_failures())
    try:
        paths = tracked_paths()
    except (OSError, subprocess.CalledProcessError, UnicodeDecodeError) as error:
        print(f"FAIL: cannot enumerate tracked files: {error}", file=sys.stderr)
        return 2

    patterns = pollution_patterns()
    scanned = 0
    for relative in paths:
        absolute = ROOT / relative
        if not absolute.is_file():
            failures.append(f"{relative.as_posix()}: tracked file is missing")
            continue
        scanned += 1
        try:
            payload = absolute.read_bytes()
        except OSError as error:
            failures.append(f"{relative.as_posix()}: cannot read: {error}")
            continue
        if b"\0" in payload:
            failures.append(f"{relative.as_posix()}: contains NUL byte")
            continue
        try:
            text = payload.decode("utf-8", errors="strict")
        except UnicodeDecodeError as error:
            failures.append(
                f"{relative.as_posix()}: invalid UTF-8 at byte {error.start}")
            continue
        for name, pattern in patterns:
            match = pattern.search(text)
            if match is None:
                continue
            line = text.count("\n", 0, match.start()) + 1
            failures.append(f"{relative.as_posix()}:{line}: {name}")

    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        print(
            f"FAIL: repository integrity scan found {len(failures)} issue(s) "
            f"in {scanned} tracked source/config file(s)",
            file=sys.stderr,
        )
        return 1

    print(
        f"PASS: repository integrity scan checked {scanned} tracked "
        "source/config file(s)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
