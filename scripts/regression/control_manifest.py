#!/usr/bin/env python3
"""Create, validate, and compare deterministic runtime control manifests."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import subprocess
import tempfile
import os
from pathlib import Path
from typing import Any


FIXED_BAG_PATH = "/home/ydkj/AutoCraneSlam-ROS1/bag/调运大件.bag"
FIXED_BAG_SHA256 = "a6805f48ca0cccf231370045808c60ca1c623ac2c6bf2c7b9ec05b804d7df33c"
SHA_PATTERN = re.compile(r"^[0-9a-f]{64}$")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_json_hash(path: Path) -> str:
    value = json.loads(path.read_text(encoding="utf-8"))
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def path_hash(path: Path) -> str:
    if path.is_file():
        return sha256_file(path)
    if not path.is_dir():
        raise FileNotFoundError(path)
    digest = hashlib.sha256()
    for item in sorted(p for p in path.rglob("*") if p.is_file()):
        relative = item.relative_to(path).as_posix().encode("utf-8")
        digest.update(len(relative).to_bytes(8, "big"))
        digest.update(relative)
        digest.update(bytes.fromhex(sha256_file(item)))
    return digest.hexdigest()


def yaml_section_hash(path: Path, section: str) -> str:
    lines = path.read_text(encoding="utf-8").splitlines()
    start = None
    body: list[str] = []
    for index, raw in enumerate(lines):
        if raw and not raw[0].isspace() and raw.split("#", 1)[0].rstrip() == f"{section}:":
            start = index
            continue
        if start is not None:
            if raw and not raw[0].isspace() and not raw.lstrip().startswith("#"):
                break
            stripped = raw.split("#", 1)[0].rstrip()
            if stripped:
                body.append(stripped)
    if start is None:
        raise ValueError(f"YAML section not found: {section}")
    return hashlib.sha256(("\n".join(body) + "\n").encode("utf-8")).hexdigest()


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(handle, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def validate(manifest: dict[str, Any], verify_files: bool = False) -> list[str]:
    errors: list[str] = []
    required = {
        "schema_version", "experiment_name", "commit_sha", "bag_path",
        "bag_sha256", "playback_rate", "config_sha256",
        "canonical_ros_parameters_sha256", "sensor_topics",
        "tf_extrinsic_sha256", "self_mask_config_sha256",
        "registration_target_snapshot_sha256",
        "persistent_map_initial_state_sha256", "runtime_profile", "feature_flags",
        "input_paths",
    }
    missing = sorted(required - set(manifest))
    if missing:
        errors.append(f"missing fields: {missing}")
    if manifest.get("schema_version") != 1:
        errors.append("schema_version must be 1")
    if not str(manifest.get("experiment_name", "")).strip():
        errors.append("experiment_name is required")
    if not re.fullmatch(r"[0-9a-f]{40}", str(manifest.get("commit_sha", ""))):
        errors.append("commit_sha must be a full lowercase SHA-1")
    for key in (
        "bag_sha256", "config_sha256", "canonical_ros_parameters_sha256",
        "tf_extrinsic_sha256", "self_mask_config_sha256",
        "registration_target_snapshot_sha256",
    ):
        if not SHA_PATTERN.fullmatch(str(manifest.get(key, ""))):
            errors.append(f"{key} must be SHA-256")
    persistent = str(manifest.get("persistent_map_initial_state_sha256", ""))
    if persistent != "EMPTY" and not SHA_PATTERN.fullmatch(persistent):
        errors.append("persistent_map_initial_state_sha256 must be SHA-256 or EMPTY")
    if manifest.get("bag_path") == FIXED_BAG_PATH and manifest.get("bag_sha256") != FIXED_BAG_SHA256:
        errors.append("fixed bag hash differs from the approved value")
    playback_rate = manifest.get("playback_rate")
    if not isinstance(playback_rate, (int, float)) or \
            not math.isfinite(float(playback_rate)) or playback_rate <= 0:
        errors.append("playback_rate must be finite and positive")
    if not isinstance(manifest.get("sensor_topics"), dict) or \
            not manifest.get("sensor_topics"):
        errors.append("sensor_topics must be a non-empty object")
    if not isinstance(manifest.get("feature_flags"), dict):
        errors.append("feature_flags must be an object")
    if not str(manifest.get("runtime_profile", "")).strip():
        errors.append("runtime_profile is required")
    input_paths = manifest.get("input_paths")
    if not isinstance(input_paths, dict):
        errors.append("input_paths must be an object")
        input_paths = {}
    required_paths = {
        "config", "ros_parameters_json", "sensor_topics_json",
        "feature_flags_json", "tf_extrinsic", "registration_target",
        "persistent_map",
    }
    missing_paths = sorted(required_paths - set(input_paths))
    if missing_paths:
        errors.append(f"input_paths missing fields: {missing_paths}")
    if verify_files:
        def verify_path(name: str, expected: str, hasher=path_hash) -> None:
            value = input_paths.get(name)
            if not isinstance(value, str) or not value:
                errors.append(f"input path unavailable: {name}")
                return
            source = Path(value)
            if not source.exists():
                errors.append(f"input file unavailable: {name}={source}")
                return
            if hasher(source) != expected:
                errors.append(f"input content no longer matches manifest: {name}")

        bag = Path(str(manifest.get("bag_path", "")))
        if not bag.is_file():
            errors.append(f"bag unavailable: {bag}")
        elif sha256_file(bag) != manifest.get("bag_sha256"):
            errors.append("bag content no longer matches manifest")
        verify_path("config", str(manifest.get("config_sha256", "")), sha256_file)
        verify_path(
            "ros_parameters_json",
            str(manifest.get("canonical_ros_parameters_sha256", "")),
            canonical_json_hash,
        )
        verify_path(
            "tf_extrinsic", str(manifest.get("tf_extrinsic_sha256", "")))
        verify_path(
            "registration_target",
            str(manifest.get("registration_target_snapshot_sha256", "")),
        )
        config_source = input_paths.get("config")
        if isinstance(config_source, str) and Path(config_source).is_file():
            if yaml_section_hash(Path(config_source), "sensor_body_self_mask") != \
                    manifest.get("self_mask_config_sha256"):
                errors.append("SelfMask config no longer matches manifest")
        for path_key, value_key in (
            ("sensor_topics_json", "sensor_topics"),
            ("feature_flags_json", "feature_flags"),
        ):
            value = input_paths.get(path_key)
            if not isinstance(value, str) or not Path(value).is_file():
                errors.append(f"input file unavailable: {path_key}={value}")
            else:
                actual_json = json.loads(Path(value).read_text(encoding="utf-8"))
                if actual_json != manifest.get(value_key):
                    errors.append(f"{path_key} content no longer matches manifest")
        persistent_path = input_paths.get("persistent_map")
        persistent_hash = manifest.get("persistent_map_initial_state_sha256")
        if persistent_hash == "EMPTY":
            if persistent_path != "EMPTY":
                errors.append("EMPTY persistent state must use the EMPTY path sentinel")
        elif not isinstance(persistent_path, str) or not Path(persistent_path).exists():
            errors.append(f"persistent map unavailable: {persistent_path}")
        elif path_hash(Path(persistent_path)) != persistent_hash:
            errors.append("persistent map content no longer matches manifest")
    return errors


def create(args: argparse.Namespace) -> int:
    commit = args.commit or subprocess.check_output(
        ["git", "rev-parse", "HEAD"], text=True
    ).strip()
    bag_hash = sha256_file(args.bag) if args.bag.exists() else args.bag_sha256
    if not bag_hash:
        raise FileNotFoundError(f"bag unavailable and --bag-sha256 omitted: {args.bag}")
    topics = json.loads(args.sensor_topics_json.read_text(encoding="utf-8"))
    flags = json.loads(args.feature_flags_json.read_text(encoding="utf-8"))
    manifest = {
        "schema_version": 1,
        "experiment_name": args.experiment_name,
        "commit_sha": commit,
        "bag_path": args.bag.as_posix(),
        "bag_sha256": bag_hash,
        "playback_rate": args.playback_rate,
        "config_sha256": sha256_file(args.config),
        "canonical_ros_parameters_sha256": canonical_json_hash(args.ros_parameters_json),
        "sensor_topics": topics,
        "tf_extrinsic_sha256": path_hash(args.tf_extrinsic),
        "self_mask_config_sha256": yaml_section_hash(args.config, "sensor_body_self_mask"),
        "registration_target_snapshot_sha256": path_hash(args.registration_target),
        "persistent_map_initial_state_sha256": (
            "EMPTY" if args.persistent_map is None else path_hash(args.persistent_map)
        ),
        "runtime_profile": args.runtime_profile,
        "feature_flags": flags,
        "input_paths": {
            "config": str(args.config.resolve()),
            "ros_parameters_json": str(args.ros_parameters_json.resolve()),
            "sensor_topics_json": str(args.sensor_topics_json.resolve()),
            "feature_flags_json": str(args.feature_flags_json.resolve()),
            "tf_extrinsic": str(args.tf_extrinsic.resolve()),
            "registration_target": str(args.registration_target.resolve()),
            "persistent_map": "EMPTY" if args.persistent_map is None else
                str(args.persistent_map.resolve()),
        },
    }
    errors = validate(manifest)
    if errors:
        raise ValueError("; ".join(errors))
    atomic_json(args.output, manifest)
    print(args.output)
    return 0


def compare(
    left: dict[str, Any], right: dict[str, Any], ephemeral_ab: bool,
    candidate_code: bool = False,
) -> list[str]:
    differences: list[str] = []
    keys = sorted(set(left) | set(right))
    for key in keys:
        if key == "experiment_name":
            continue
        # Paths can differ between the immutable control checkout and the
        # candidate checkout. verify-files checks both sets; comparison is
        # intentionally governed by their canonical content hashes.
        if key == "input_paths":
            continue
        # A mainline gate intentionally compares different code revisions.
        # Both full SHAs remain recorded and validated; every environmental
        # input is still required to match byte-for-byte.
        if key == "commit_sha" and candidate_code:
            continue
        if key == "feature_flags" and ephemeral_ab:
            left_flags = dict(left.get(key, {}))
            right_flags = dict(right.get(key, {}))
            left_ephemeral = left_flags.pop("tracking_ephemeral_map_enabled", None)
            right_ephemeral = right_flags.pop("tracking_ephemeral_map_enabled", None)
            if left_flags != right_flags or left_ephemeral == right_ephemeral:
                differences.append("feature_flags are not a pure ephemeral OFF/ON pair")
            continue
        if left.get(key) != right.get(key):
            differences.append(key)
    return differences


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser()
    commands = root.add_subparsers(dest="command", required=True)
    make = commands.add_parser("create")
    make.add_argument("--experiment-name", required=True)
    make.add_argument("--commit")
    make.add_argument("--bag", type=Path, default=Path(FIXED_BAG_PATH))
    make.add_argument("--bag-sha256", default=FIXED_BAG_SHA256)
    make.add_argument("--playback-rate", type=float, required=True)
    make.add_argument("--config", type=Path, required=True)
    make.add_argument("--ros-parameters-json", type=Path, required=True)
    make.add_argument("--sensor-topics-json", type=Path, required=True)
    make.add_argument("--tf-extrinsic", type=Path, required=True)
    make.add_argument("--registration-target", type=Path, required=True)
    make.add_argument("--persistent-map", type=Path)
    make.add_argument("--runtime-profile", required=True)
    make.add_argument("--feature-flags-json", type=Path, required=True)
    make.add_argument("--output", type=Path, required=True)
    check = commands.add_parser("validate")
    check.add_argument("manifest", type=Path)
    check.add_argument("--verify-files", action="store_true")
    diff = commands.add_parser("compare")
    diff.add_argument("control", type=Path)
    diff.add_argument("experiment", type=Path)
    diff.add_argument("--ephemeral-ab", action="store_true")
    diff.add_argument("--candidate-code", action="store_true")
    return root


def main() -> int:
    args = parser().parse_args()
    if args.command == "create":
        return create(args)
    if args.command == "validate":
        manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
        errors = validate(manifest, args.verify_files)
    else:
        left = json.loads(args.control.read_text(encoding="utf-8"))
        right = json.loads(args.experiment.read_text(encoding="utf-8"))
        errors = validate(left) + validate(right)
        errors += compare(
            left, right, args.ephemeral_ab, args.candidate_code)
    if errors:
        for error in errors:
            print(f"ERROR {error}")
        return 1
    print("PASS control_manifest")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
