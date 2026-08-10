#!/usr/bin/env python3
"""Summarize cargo/static diagnostic CSVs without changing acceptance state."""

from __future__ import annotations

import argparse
import collections
import csv
import json
import math
from pathlib import Path


AVOIDANCE_FIELDS = {
    "stamp",
    "requested_alarm_code",
    "nearest_cluster_distance",
    "conservative_clearance_m",
    "obstacle_track_id",
}


def load_cargo_rows(
    path: Path, *, require_avoidance_fields: bool = False
) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or ())
        if require_avoidance_fields:
            missing = sorted(AVOIDANCE_FIELDS - fields)
            if missing:
                raise ValueError(
                    "cargo CSV missing columns: " + ", ".join(missing))
        return list(reader)


def _finite_float(row: dict[str, object], key: str) -> float:
    try:
        value = float(row.get(key, "nan"))
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def _integer(row: dict[str, object], key: str) -> int:
    try:
        return int(row.get(key, 0))
    except (TypeError, ValueError):
        return 0


def validate_avoidance_rows(
    rows: list[dict[str, object]],
) -> list[str]:
    """Validate invariants visible in cargo_frames.csv after bag playback.

    Far-history establishment remains a tracker/GTest contract because the
    legacy CSV schema does not expose its full timestamp window. This checker
    verifies the observable warning bands, the 0.8 m total gate, one-track
    identity and the two-distinct-frame minimum for Code 29.
    """
    violations: list[str] = []
    observed_stamps: dict[int, set[float]] = collections.defaultdict(set)
    for index, row in enumerate(rows, start=2):
        code = _integer(row, "requested_alarm_code")
        distance = _finite_float(row, "nearest_cluster_distance")
        clearance = _finite_float(row, "conservative_clearance_m")
        track_id = _integer(row, "obstacle_track_id")
        stamp = _finite_float(row, "stamp")

        if (track_id > 0 and math.isfinite(stamp) and
                math.isfinite(distance) and distance <= 5.0 and
                math.isfinite(clearance) and clearance < 0.8):
            observed_stamps[track_id].add(stamp)

        if code not in (17, 18, 29):
            continue
        prefix = f"row={index} code={code}"
        if track_id <= 0:
            violations.append(f"{prefix} obstacle_track_identity")
        if not math.isfinite(clearance) or clearance >= 0.8:
            violations.append(f"{prefix} clearance_gate")
        if not math.isfinite(distance):
            violations.append(f"{prefix} distance_nonfinite")
            continue
        if code == 17 and distance > 3.0:
            violations.append(f"{prefix} code17_distance")
        elif code == 18 and not (3.0 < distance <= 5.0):
            violations.append(f"{prefix} code18_distance")
        elif code == 29:
            if distance > 5.0:
                violations.append(f"{prefix} code29_distance")
            if track_id <= 0 or len(observed_stamps[track_id]) < 2:
                violations.append(f"{prefix} code29_distinct_frames")
    return violations


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("cargo_csv", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--validate-avoidance-first",
        action="store_true",
        help="fail when observable 17/18/29 contracts are violated",
    )
    parser.add_argument(
        "--require-code",
        action="append",
        type=int,
        default=[],
        help="require a warning code to occur (repeatable)",
    )
    args = parser.parse_args()

    codes = collections.Counter()
    reasons = collections.Counter()
    query_reasons = collections.Counter()
    track_ids = set()
    rows = 0
    last = {}
    try:
        cargo_rows = load_cargo_rows(
            args.cargo_csv,
            require_avoidance_fields=args.validate_avoidance_first,
        )
    except (OSError, ValueError) as error:
        parser.error(str(error))
    for row in cargo_rows:
        rows += 1
        last = row
        codes[row.get("requested_alarm_code", "unknown")] += 1
        reasons[row.get("safety_reason", "unknown")] += 1
        query_reasons[row.get("static_query_reason", "unknown")] += 1
        track_id = row.get("obstacle_track_id", "0")
        if track_id not in ("", "0"):
            track_ids.add(track_id)

    violations = (validate_avoidance_rows(cargo_rows)
                  if args.validate_avoidance_first else [])
    for required_code in args.require_code:
        if codes[str(required_code)] == 0:
            violations.append(f"required_code_missing={required_code}")

    summary = {
        "rows": rows,
        "codes": dict(codes),
        "safety_reasons": dict(reasons),
        "static_query_reasons": dict(query_reasons),
        "unique_obstacle_tracks": len(track_ids),
        "last_static_index_revision": last.get("static_index_revision", "0"),
        "last_static_index_cells": last.get("static_index_cell_count", "0"),
        "last_track_created_count": last.get("obstacle_track_created_count", "0"),
        "last_track_reset_count": last.get("obstacle_track_reset_count", "0"),
        "avoidance_contract_valid": not violations,
        "avoidance_contract_violations": violations,
    }
    rendered = json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    else:
        print(rendered)
    return 0 if not violations else 2


if __name__ == "__main__":
    raise SystemExit(main())
