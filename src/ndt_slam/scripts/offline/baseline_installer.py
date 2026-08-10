#!/usr/bin/env python3
"""Atomically install an approved immutable split baseline."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import tempfile
from pathlib import Path

from offline_common import atomic_json, require_hash, require_sidecar, sha256_file


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("certified_dir", type=Path)
    parser.add_argument("baseline_root", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    manifest_path = args.certified_dir / "certified_baseline_manifest.json"
    require_sidecar(manifest_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("authority") != "CERTIFIED":
        raise ValueError("only a human-approved CERTIFIED baseline can be installed")
    baseline_uuid = str(manifest.get("baseline_uuid", ""))
    if len(baseline_uuid) != 64:
        raise ValueError("invalid baseline UUID")
    for section in ("localization_reference", "avoidance_static_baseline"):
        item = manifest[section]
        require_hash(args.certified_dir / item["file"], item["sha256"])
    identity_material = "\n".join(
        [
            manifest["localization_reference"]["sha256"],
            manifest["avoidance_static_baseline"]["sha256"],
            manifest["config_sha256"],
            manifest["extrinsic_sha256"],
            manifest["source_manifest_sha256"],
        ]
    ).encode("utf-8")
    if hashlib.sha256(identity_material).hexdigest() != baseline_uuid:
        raise ValueError("certified baseline identity does not match its contents")
    if not str(manifest.get("approved_by", "")).strip():
        raise ValueError("certified baseline has no human approver")

    immutable_root = args.baseline_root / "baselines"
    immutable_root.mkdir(parents=True, exist_ok=True)
    target = immutable_root / baseline_uuid
    if target.exists():
        installed_manifest = target / manifest_path.name
        if not installed_manifest.exists() or sha256_file(installed_manifest) != sha256_file(manifest_path):
            raise FileExistsError("immutable baseline UUID already exists with different content")
    else:
        staging = Path(tempfile.mkdtemp(prefix=f".{baseline_uuid}.", dir=immutable_root))
        try:
            for name in (
                "certified_baseline_manifest.json",
                manifest["localization_reference"]["file"],
                manifest["avoidance_static_baseline"]["file"],
            ):
                shutil.copy2(args.certified_dir / name, staging / name)
            os.replace(staging, target)
        finally:
            if staging.exists():
                shutil.rmtree(staging)
    atomic_json(
        args.baseline_root / "CURRENT.json",
        {
            "schema_version": 1,
            "baseline_uuid": baseline_uuid,
            "manifest_sha256": sha256_file(target / manifest_path.name),
            "immutable_relative_path": f"baselines/{baseline_uuid}",
        },
    )
    print(f"INSTALLED baseline_uuid={baseline_uuid}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
