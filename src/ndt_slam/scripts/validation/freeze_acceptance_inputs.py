#!/usr/bin/env python3
"""Freeze the immutable acceptance inputs for the P0 combined Ubuntu gate.

This is an acceptance-only preflight. It never touches product algorithm code
and never plays a bag. It verifies that every input the four-bag matrix will
consume exists, is the exact file that will be used, and records a content
hash so a later report can prove which data was actually tested.

Exit codes:
  0  every gate passed, ``frozen_acceptance_inputs.json`` written
  1  a preflight gate failed (the failing gate is printed as ``*_GATE=FAIL``)
  2  usage error
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys

BAG_NAMES = ("无", "有", "长件", "大件")

YAW_REQUIRED_FIELDS = (
    "schema_version",
    "verified",
    "rail_yaw_in_map_rad",
    "source",
    "map_frame_uuid",
    "map_frame_id",
    "base_frame_id",
    "map_frame_convention_id",
    "sensor_rig_calibration_id",
    "reference_uuid",
    "reference_hash",
)


def _sha256_file(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _run(*args: str) -> str:
    try:
        result = subprocess.run(
            args, capture_output=True, text=True, timeout=20
        )
    except (OSError, subprocess.TimeoutExpired):
        return ""
    return (result.stdout or "").strip().splitlines()[0] if (
        result.stdout or ""
    ).strip() else ""


def _pcl_version() -> str:
    version = _run("pkg-config", "--modversion", "pcl_common-1.10")
    if version:
        return version
    # dpkg query for libpcl-dev as a fallback on Ubuntu installations.
    try:
        result = subprocess.run(
            ["dpkg-query", "-W", "-f=${Version}", "libpcl-dev"],
            capture_output=True, text=True, timeout=20,
        )
        if result.returncode == 0 and result.stdout.strip():
            return result.stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        pass
    return ""


def _ros_distro() -> str:
    env = os.environ.get("ROS_DISTRO", "")
    if env:
        return env
    if os.path.isdir("/opt/ros"):
        entries = sorted(
            name for name in os.listdir("/opt/ros")
            if os.path.isfile(os.path.join("/opt/ros", name, "setup.bash"))
        )
        if entries:
            return entries[0]
    return "unknown"


def _load_yaml(path: str):
    try:
        import yaml  # noqa: PLC0415
    except ImportError:
        return None
    try:
        with open(path, "r", encoding="utf-8") as stream:
            return yaml.safe_load(stream)
    except Exception:
        return None


def fail(gate: str, reason: str) -> None:
    print(f"{gate}=FAIL reason={reason}", file=sys.stderr)
    raise SystemExit(1)


def require_file(gate: str, path: str) -> None:
    if not os.path.isfile(path):
        fail(gate, f"missing={path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", required=True)
    parser.add_argument("--candidate-sha", required=True)
    parser.add_argument("--map-source", required=True)
    parser.add_argument("--yaw-reference", required=True)
    parser.add_argument("--oracle-dir", required=True)
    parser.add_argument("--baseline-trace-dir", required=True)
    parser.add_argument("--output", required=True)
    for name in BAG_NAMES:
        parser.add_argument(f"--bag-{name}", required=True)
    args = parser.parse_args()

    workspace = os.path.realpath(args.workspace)
    map_source = os.path.realpath(args.map_source)
    yaw_reference = os.path.realpath(args.yaw_reference)
    oracle_dir = os.path.realpath(args.oracle_dir)
    baseline_trace_dir = os.path.realpath(args.baseline_trace_dir)

    # Workspace HEAD must match the candidate before anything else.
    actual_sha = subprocess.run(
        ["git", "-C", workspace, "rev-parse", "HEAD"],
        capture_output=True, text=True,
    ).stdout.strip()
    if actual_sha != args.candidate_sha:
        fail("SHA_GATE", f"expected={args.candidate_sha} actual={actual_sha}")

    # Bag existence + content hash.
    bags = {}
    for name in BAG_NAMES:
        bag = os.path.realpath(getattr(args, f"bag_{name}"))
        require_file("BAG_GATE", bag)
        bags[name] = {"path": bag, "sha256": _sha256_file(bag)}

    # Oracle existence + content hash (one per bag, frozen before replay).
    oracles = {}
    for name in BAG_NAMES:
        oracle = os.path.join(oracle_dir, f"{name}.json")
        require_file("ORACLE_GATE", oracle)
        oracles[name] = {"path": oracle, "sha256": _sha256_file(oracle)}

    # Baseline traces (V5) existence + content hash, one pair per bag.
    baselines = {}
    for name in BAG_NAMES:
        group = os.path.join(
            baseline_trace_dir, f"{name}_integrated_identity_groups.csv"
        )
        shadow = os.path.join(
            baseline_trace_dir, f"{name}_integrated_avoidance_shadow.csv"
        )
        require_file("BASELINE_TRACE_GATE", group)
        require_file("BASELINE_TRACE_GATE", shadow)
        baselines[name] = {
            "identity_groups": {"path": group, "sha256": _sha256_file(group)},
            "avoidance_shadow": {"path": shadow, "sha256": _sha256_file(shadow)},
        }

    # Map source tree hash (existence + immutable content digest).
    if not os.path.isdir(map_source):
        fail("MAP_SOURCE_GATE", f"missing_dir={map_source}")
    tree_files = []
    for root, _, files in os.walk(map_source):
        for filename in files:
            tree_files.append(os.path.join(root, filename))
    tree_files.sort()
    tree_hasher = hashlib.sha256()
    for path in tree_files:
        tree_hasher.update(
            os.path.relpath(path, map_source).encode("utf-8", "replace")
        )
        tree_hasher.update(b"\0")
        tree_hasher.update(_sha256_file(path).encode("ascii"))
        tree_hasher.update(b"\0")
    map_entry = {
        "path": map_source,
        "file_count": len(tree_files),
        "tree_hash": tree_hasher.hexdigest(),
    }

    # Yaw reference schema + content hash. A missing or unverified reference
    # is a hard fail; the reference is the source of Rail-authority identity.
    require_file("YAW_REFERENCE_GATE", yaw_reference)
    yaw_sha256 = _sha256_file(yaw_reference)
    document = _load_yaml(yaw_reference)
    if document is None:
        fail("YAW_REFERENCE_GATE", "unparseable_yaml")
    reference = document.get("reference", document)
    if not isinstance(reference, dict):
        fail("YAW_REFERENCE_GATE", "reference_not_mapping")
    missing = sorted(set(YAW_REQUIRED_FIELDS) - set(reference))
    if missing or reference.get("verified") is not True:
        fail(
            "YAW_REFERENCE_GATE",
            "invalid_frozen_yaw_reference missing=" + ",".join(missing),
        )
    yaw_entry = {
        "path": yaw_reference,
        "sha256": yaw_sha256,
        "schema_version": reference.get("schema_version"),
        "reference_hash": reference.get("reference_hash"),
        "map_frame_uuid": reference.get("map_frame_uuid"),
        "rail_yaw_in_map_rad": reference.get("rail_yaw_in_map_rad"),
        "verified": reference.get("verified"),
    }

    environment = {
        "ros_distro": _ros_distro(),
        "pcl_version": _pcl_version(),
        "cmake_version": _run("cmake", "--version"),
        "gcc_version": _run("gcc", "--version"),
        "python_version": _run("python3", "--version"),
    }

    manifest = {
        "schema_version": 1,
        "candidate_sha": args.candidate_sha,
        "bags": bags,
        "oracles": oracles,
        "baseline_traces": baselines,
        "map_source": map_entry,
        "yaw_reference": yaw_entry,
        "environment": environment,
    }

    output_dir = os.path.dirname(args.output)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2)
        stream.write("\n")

    manifest_sha256 = _sha256_file(args.output)
    print(f"INPUT_PREFLIGHT=PASS")
    print(f"FROZEN_INPUT_MANIFEST={args.output}")
    print(f"INPUT_MANIFEST_SHA256={manifest_sha256}")
    for name in BAG_NAMES:
        print(f"BAG_GATE=PASS bag={name} sha256={bags[name]['sha256'][:16]}")
    print(f"ORACLE_GATE=PASS sha256s={','.join(o['sha256'][:8] for o in oracles.values())}")
    print(f"BASELINE_TRACE_GATE=PASS")
    print(f"MAP_SOURCE_GATE=PASS file_count={map_entry['file_count']} tree_hash={map_entry['tree_hash'][:16]}")
    print(f"YAW_REFERENCE_GATE=PASS reference_hash={yaw_entry['reference_hash']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
