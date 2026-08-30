#!/usr/bin/env python3
"""Static authority audit for the Avoidance V4 integration branch."""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BASE = "13c0af4aa44f366b1d63c38ab37366890677c979"


def fail(message: str) -> None:
    print(f"FAIL: {message}", file=sys.stderr)
    raise SystemExit(1)


def git(*args: str) -> str:
    result = subprocess.run(
        ["git", *args], cwd=ROOT, check=False, text=True,
        encoding="utf-8", errors="replace",
        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode != 0:
        fail(f"git {' '.join(args)}: {result.stderr.strip()}")
    return result.stdout


def strip_if0(source: str) -> str:
    output: list[str] = []
    disabled_depth = 0
    for line in source.splitlines():
        if re.match(r"\s*#\s*if\s+0\b", line):
            disabled_depth += 1
            continue
        if disabled_depth and re.match(r"\s*#\s*if\b", line):
            disabled_depth += 1
            continue
        if disabled_depth and re.match(r"\s*#\s*endif\b", line):
            disabled_depth -= 1
            continue
        if not disabled_depth:
            output.append(line)
    return "\n".join(output)


def main() -> int:
    changed = set(git("diff", "--name-only", BASE, "--").splitlines())
    frozen = {
        "src/ndt_slam/src/cargo_physical_identity_authority.cpp",
        "src/ndt_slam/src/cargo_bottom_fusion.cpp",
        "src/ndt_slam/src/cargo_safety_temporal_filter.cpp",
        "src/ndt_slam/src/cargo_avoidance_fusion.cpp",
        "src/ndt_slam/src/fixed_yaw_translation_solver.cpp",
        "src/ndt_slam/src/rail_translation_pose_graph.cpp",
        "src/ndt_slam/src/crane_motion_ekf.cpp",
    }
    touched = sorted(changed & frozen)
    if touched:
        fail("frozen implementation changed: " + ", ".join(touched))

    diff = git("diff", "--unified=0", BASE, "--", "src/ndt_slam")
    forbidden_added_writers = (
        r"^\+[^+].*\bcurrent_pose_\s*=",
        r"^\+[^+].*\bpublished_pose_\s*=",
        r"^\+[^+].*\brail_yaw_authority_\.(?:initialize|reset|transition)",
        r"^\+[^+].*\bkeyframe_pose_version_\.fetch_add",
        r"^\+[^+].*\bmap_rebuild_generation_\.(?:fetch_add|store)",
        r"^\+[^+].*\bcurrent_registration_target_snapshot_id_\.store",
        r"^\+[^+].*\blocalization_authority_health_\s*=",
    )
    for pattern in forbidden_added_writers:
        match = re.search(pattern, diff, re.MULTILINE)
        if match:
            fail(f"Avoidance diff added forbidden SLAM writer: {match.group(0)}")

    node = strip_if0((ROOT / "src/ndt_slam/src/ndt_slam.cpp").read_text(
        encoding="utf-8"))
    for forbidden_call in (
        "human_filter_.processFrame(",
        "dynamic_event_manager_.createHumanEvent(",
    ):
        if forbidden_call in node:
            fail(f"active duplicate Human authority call: {forbidden_call}")
    if "CARGO_V6_PRODUCT_MODE_BLOCKED_BY_LEGACY_FORMAL_PIPELINE" in node:
        fail("V6 product startup block remains")

    sparse = (ROOT / "src/ndt_slam/src/cargo_obstacle_tracker.cpp").read_text(
        encoding="utf-8")
    for forbidden in (
        "global_map_", "objects_map_", "objects_clean_map_",
        "StaticObstacleEvidence", "LocalizationTarget", "current_pose_",
        "rail_yaw_authority_", "relocalization",
    ):
        if forbidden in sparse:
            fail(f"Sparse/physical tracker gained forbidden side effect: {forbidden}")

    required = {
        "SourceFrameIdentity": "src/ndt_slam/include/ndt_slam/avoidance_map_mutation.hpp",
        "SourcePointKey": "src/ndt_slam/include/ndt_slam/avoidance_map_mutation.hpp",
        "ProductCargoContext": "src/ndt_slam/include/ndt_slam/product_cargo_context.hpp",
        "cargo_candidate_points":
            "src/ndt_slam/include/ndt_slam/avoidance_map_mutation.hpp",
        "SPARSE_MULTI_FRAME": "src/ndt_slam/include/ndt_slam/cargo_obstacle_tracker.hpp",
        "evaluateCargoRegistrationHygieneShadow":
            "src/ndt_slam/src/cargo_v6_authority_adapter.cpp",
        "last_keyframe_commit_cloud_hash_":
            "src/ndt_slam/src/ndt_slam.cpp",
    }
    for token, relative in required.items():
        if token not in (ROOT / relative).read_text(encoding="utf-8"):
            fail(f"required V4 contract missing: {token}")

    if ("cargo_authority_mode_ != CargoAuthorityMode::V6_AUTHORITY" not in
            node or "v6_candidate_quarantine_active" not in node or
            "cargo_candidate_points.owns(" not in node):
        fail("V6 exact-candidate quarantine did not replace broad product removal")
    if ("retained_legacy_hazard_same_authority" not in node or
            "v6_authority_positive_only" not in node or
            re.search(
                r"selectProductCargoContext\([^;]+false\s*,\s*false\s*\)",
                node, re.DOTALL)):
        fail("V6 same-authority positive-only retention is not product-wired")

    print("PASS: Avoidance V4 static authority firewall")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
