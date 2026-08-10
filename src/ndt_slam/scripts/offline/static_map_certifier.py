#!/usr/bin/env python3
"""Certify split localization/avoidance candidates after independent review."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

from offline_common import (
    atomic_copy,
    atomic_json,
    atomic_text,
    require_hash,
    require_sidecar,
    sha256_file,
    simple_yaml,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("candidate_dir", type=Path)
    parser.add_argument("validation_report", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--extrinsic", type=Path, required=True)
    parser.add_argument("--source-manifest", type=Path, required=True)
    parser.add_argument("--approve-by", required=True)
    parser.add_argument("--minimum-mature-cells", type=int, default=100)
    parser.add_argument("--minimum-mature-coverage", type=float, default=0.50)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    candidate_manifest_path = args.candidate_dir / "candidate_manifest.yaml"
    require_sidecar(candidate_manifest_path)
    candidate = simple_yaml(candidate_manifest_path)
    if candidate.get("authority") != "CANDIDATE_NOT_CERTIFIED":
        raise ValueError("runtime evidence or an already-installed map cannot be certified")
    localization = args.candidate_dir / "candidate_localization_reference.pcd"
    avoidance = args.candidate_dir / "candidate_avoidance_static_baseline.pcd"
    require_hash(localization, candidate["localization_reference_sha256"])
    require_hash(avoidance, candidate["avoidance_static_baseline_sha256"])

    validation = json.loads(args.validation_report.read_text(encoding="utf-8"))
    required_validation = {
        "passed": True,
        "route_independent": True,
        "baseline_not_used_for_collection": True,
    }
    for key, expected in required_validation.items():
        if validation.get(key) is not expected:
            raise ValueError(f"independent validation requirement failed: {key}")
    if not str(validation.get("survey_pass_id", "")).strip():
        raise ValueError("validation survey_pass_id is required")

    total_cells = 0
    mature_cells = 0
    evidence_path = args.candidate_dir / "static_evidence_candidate.csv"
    require_hash(evidence_path, candidate["static_evidence_candidate_sha256"])
    with evidence_path.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            total_cells += 1
            if int(row["episode_count"]) >= 3 and int(row["pass_count"]) >= 2:
                mature_cells += 1
    coverage = mature_cells / total_cells if total_cells else 0.0
    if mature_cells < args.minimum_mature_cells or coverage < args.minimum_mature_coverage:
        raise ValueError(
            f"static support insufficient: mature={mature_cells} total={total_cells} "
            f"coverage={coverage:.6f}"
        )

    localization_hash = sha256_file(localization)
    avoidance_hash = sha256_file(avoidance)
    config_hash = candidate.get("config_sha256", "")
    extrinsic_hash = sha256_file(args.extrinsic)
    source_manifest_hash = sha256_file(args.source_manifest)
    if not args.approve_by.strip():
        raise ValueError("approve-by must identify the human approver")
    identity_material = "\n".join(
        [localization_hash, avoidance_hash, config_hash, extrinsic_hash, source_manifest_hash]
    ).encode("utf-8")
    baseline_uuid = hashlib.sha256(identity_material).hexdigest()

    output_localization = args.output_dir / "localization_reference.pcd"
    output_avoidance = args.output_dir / "avoidance_static_baseline.pcd"
    atomic_copy(localization, output_localization)
    atomic_copy(avoidance, output_avoidance)
    manifest = {
        "schema_version": 1,
        "authority": "CERTIFIED",
        "baseline_uuid": baseline_uuid,
        "frame_id": candidate.get("frame_id", "map"),
        "approved_by": args.approve_by,
        "validation_report_sha256": sha256_file(args.validation_report),
        "validation_survey_pass_id": validation["survey_pass_id"],
        "source_manifest_sha256": source_manifest_hash,
        "config_sha256": config_hash,
        "extrinsic_sha256": extrinsic_hash,
        "localization_reference": {
            "file": output_localization.name,
            "sha256": sha256_file(output_localization),
        },
        "avoidance_static_baseline": {
            "file": output_avoidance.name,
            "sha256": sha256_file(output_avoidance),
        },
        "static_support": {
            "mature_cells": mature_cells,
            "total_cells": total_cells,
            "mature_coverage": coverage,
            "minimum_episodes": 3,
            "minimum_survey_passes": 2,
        },
    }
    atomic_json(args.output_dir / "certified_baseline_manifest.json", manifest)
    certified_manifest = args.output_dir / "certified_baseline_manifest.json"
    atomic_text(
        Path(str(certified_manifest) + ".sha256"),
        f"{sha256_file(certified_manifest)}  {certified_manifest.name}\n",
    )
    print(f"CERTIFIED baseline_uuid={baseline_uuid}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
