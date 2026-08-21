#!/usr/bin/env python3
"""Aggregate all four Integrated Cargo Identity SHADOW replays.

The script deliberately never stops on a failed bag. Each --bag argument is
NAME=integrated_avoidance_shadow.csv and the combined verdict is emitted only
after every supplied trace has been analyzed.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import median
from typing import Any, Iterable


def number(row: dict[str, str], key: str, default: float = math.nan) -> float:
    try:
        return float(row.get(key, ""))
    except (TypeError, ValueError):
        return default


def flag(row: dict[str, str], key: str) -> bool:
    return number(row, key, 0.0) == 1.0


def percentile(values: Iterable[float], quantile: float) -> float:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return math.nan
    index = int(math.ceil(quantile * len(finite))) - 1
    return finite[max(0, min(index, len(finite) - 1))]


def analyze_trace(name: str, path: Path) -> dict[str, Any]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    valid_geometry = [
        row for row in rows
        if flag(row, "shadow_geometry_valid_this_frame")
    ]
    evaluated = [
        row for row in valid_geometry
        if flag(row, "shadow_official_valid")
    ]
    validated = [row for row in rows if row.get("shadow_identity") == "VALIDATED"]
    wrong_locks = [
        row for row in rows
        if flag(row, "formal_lock")
        and row.get("baseline_selected_candidate_id")
        != row.get("shadow_candidate_id")
    ]
    false_warning_rows = [
        row for row in evaluated
        if int(number(row, "shadow_code", 0.0)) in (17, 18, 29)
    ]
    contamination = any(
        flag(row, "obstacle_self_contamination_blocking") for row in rows
    )
    compute = [number(row, "shadow_total_compute_ms") for row in rows]
    callback_hz = [number(row, "pointcloud_callback_hz") for row in rows]
    processed_hz = [number(row, "ndt_processing_hz") for row in rows]
    dropped = [number(row, "dropped_frame_count") for row in rows]
    large_gaps = [number(row, "large_gap_count") for row in rows]
    result: dict[str, Any] = {
        "bag": name,
        "trace": str(path),
        "frames": len(rows),
        "identity_validated": bool(validated),
        "geometry_valid_frames": len(valid_geometry),
        "evaluated_frames": len(evaluated),
        "wrong_formal_lock_frames": len(wrong_locks),
        "valid_geometry_warning_17_18_29_frames": len(false_warning_rows),
        "identity_validated_before_8m": any(
            flag(row, "identity_before_8m") for row in rows
        ),
        "pending_or_lock_ready_before_5m": any(
            flag(row, "ready_before_5m") for row in rows
        ),
        "canonical_far_history_valid": any(
            flag(row, "canonical_far_history_valid") for row in rows
        ),
        "obstacle_self_contamination_blocking": contamination,
        "shadow_total_compute_p50_ms": median(
            value for value in compute if math.isfinite(value)
        ) if any(math.isfinite(value) for value in compute) else math.nan,
        "shadow_total_compute_p95_ms": percentile(compute, 0.95),
        "shadow_total_compute_max_ms": max(
            (value for value in compute if math.isfinite(value)),
            default=math.nan,
        ),
        "pointcloud_callback_hz": median(
            value for value in callback_hz
            if math.isfinite(value) and value > 0.0
        ) if any(math.isfinite(value) and value > 0.0
                 for value in callback_hz) else math.nan,
        "ndt_processing_hz": median(
            value for value in processed_hz
            if math.isfinite(value) and value > 0.0
        ) if any(math.isfinite(value) and value > 0.0
                 for value in processed_hz) else math.nan,
        "dropped_frame_count": max(dropped, default=0.0),
        "large_gap_count": max(large_gaps, default=0.0),
    }
    result["identity_verdict"] = (
        "PASS" if result["identity_validated"]
        and result["wrong_formal_lock_frames"] == 0 else "FAIL"
    )
    result["vertical_geometry_verdict"] = (
        "PASS" if result["geometry_valid_frames"] > 0 else "FAIL"
    )
    result["timing_verdict"] = (
        "PASS" if result["identity_validated_before_8m"]
        and result["pending_or_lock_ready_before_5m"] else "FAIL"
    )
    if contamination:
        result["avoidance_cargo_side_verdict"] = (
            "BLOCKED_BY_OBSTACLE_SELF_CONTAMINATION"
        )
    else:
        result["avoidance_cargo_side_verdict"] = (
            "PASS" if result["geometry_valid_frames"] > 0 else "FAIL"
        )
    return result


def analyze_runtime(path: Path) -> dict[str, float]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    cpu = [number(row, "cpu_percent") for row in rows]
    rss = [number(row, "rss_mb") for row in rows]
    return {
        "cpu_percent_p50": percentile(cpu, 0.50),
        "cpu_percent_p95": percentile(cpu, 0.95),
        "cpu_percent_max": max(
            (value for value in cpu if math.isfinite(value)),
            default=math.nan,
        ),
        "rss_mb_p50": percentile(rss, 0.50),
        "rss_mb_p95": percentile(rss, 0.95),
        "rss_mb_max": max(
            (value for value in rss if math.isfinite(value)),
            default=math.nan,
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--bag", action="append", required=True,
        help="NAME=/path/to/integrated_avoidance_shadow.csv",
    )
    parser.add_argument(
        "--runtime", action="append", default=[],
        help="NAME=/path/to/runtime_samples.csv",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    reports: list[dict[str, Any]] = []
    errors: list[dict[str, str]] = []
    runtime_paths: dict[str, Path] = {}
    for item in args.runtime:
        try:
            runtime_name, runtime_path = item.split("=", 1)
            runtime_paths[runtime_name] = Path(runtime_path)
        except ValueError:
            errors.append({"bag": item, "error": "invalid runtime NAME=PATH"})
    for item in args.bag:
        try:
            name, raw_path = item.split("=", 1)
            report = analyze_trace(name, Path(raw_path))
            if name in runtime_paths:
                report["runtime"] = analyze_runtime(runtime_paths[name])
            reports.append(report)
        except Exception as error:  # continue through all four bags
            errors.append({"bag": item, "error": str(error)})

    identity_values = [report["identity_verdict"] for report in reports]
    geometry_values = [report["vertical_geometry_verdict"] for report in reports]
    timing_values = [report["timing_verdict"] for report in reports]
    avoidance_values = [report["avoidance_cargo_side_verdict"] for report in reports]
    long_partial = any(
        report["bag"].lower() in ("long", "长件")
        and not report["identity_validated"] for report in reports
    )
    summary = {
        "bags": reports,
        "errors": errors,
        "CARGO_IDENTITY_CORRECTNESS": (
            "PASS" if reports and all(value == "PASS" for value in identity_values)
            else "PARTIAL" if reports and any(value == "PASS" for value in identity_values)
            else "FAIL"
        ),
        "CARGO_VERTICAL_GEOMETRY_CORRECTNESS": (
            "PASS" if reports and all(value == "PASS" for value in geometry_values)
            else "PARTIAL" if reports and any(value == "PASS" for value in geometry_values)
            else "FAIL"
        ),
        "AVOIDANCE_TIMING_READINESS": (
            "PASS" if reports and all(value == "PASS" for value in timing_values)
            else "PARTIAL" if reports and any(value == "PASS" for value in timing_values)
            else "FAIL"
        ),
        "AVOIDANCE_CARGO_SIDE_CORRECTNESS": (
            "BLOCKED_BY_OBSTACLE_SELF_CONTAMINATION"
            if any(value == "BLOCKED_BY_OBSTACLE_SELF_CONTAMINATION"
                   for value in avoidance_values)
            else "PASS" if reports and all(value == "PASS" for value in avoidance_values)
            else "PARTIAL" if reports and any(value == "PASS" for value in avoidance_values)
            else "FAIL"
        ),
        "GLOBAL_CARGO_AVAILABILITY": "PARTIAL" if long_partial else (
            "PASS" if reports and all(value == "PASS" for value in identity_values)
            else "FAIL"
        ),
        "PRODUCT_BEHAVIOR_CHANGED": "NO",
        "FIELD_READY": "NO",
    }
    pass_values = (
        summary["CARGO_IDENTITY_CORRECTNESS"],
        summary["CARGO_VERTICAL_GEOMETRY_CORRECTNESS"],
        summary["AVOIDANCE_TIMING_READINESS"],
        summary["AVOIDANCE_CARGO_SIDE_CORRECTNESS"],
        summary["GLOBAL_CARGO_AVAILABILITY"],
    )
    summary["INTEGRATED_SHADOW_VERDICT"] = (
        "PASS" if len(reports) == 4 and not errors
        and all(value == "PASS" for value in pass_values)
        else "PARTIAL_PASS" if reports and not all(value == "FAIL" for value in pass_values)
        else "FAIL"
    )
    rendered = json.dumps(summary, ensure_ascii=False, indent=2)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if not errors else 2


if __name__ == "__main__":
    raise SystemExit(main())
