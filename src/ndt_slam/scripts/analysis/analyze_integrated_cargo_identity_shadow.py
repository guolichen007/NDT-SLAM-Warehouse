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


def normalized_member_set(value: Any) -> tuple[int, ...]:
    if isinstance(value, str):
        tokens = value.replace(",", "|").split("|")
    elif isinstance(value, (list, tuple)):
        tokens = value
    else:
        return ()
    try:
        return tuple(sorted({int(token) for token in tokens if str(token).strip()}))
    except (TypeError, ValueError):
        return ()


def oracle_identity_sets(
    oracle: dict[str, Any], prefix: str
) -> tuple[set[str], set[tuple[int, ...]]]:
    candidate_ids = {str(value) for value in oracle.get(
        f"{prefix}_candidate_ids", [])}
    member_sets = {
        normalized_member_set(value)
        for value in oracle.get(f"{prefix}_member_sets", [])
    }
    member_sets.discard(())
    return candidate_ids, member_sets


def row_matches_identity(
    row: dict[str, str], candidate_ids: set[str],
    member_sets: set[tuple[int, ...]],
) -> bool:
    return (
        row.get("shadow_candidate_id", "") in candidate_ids
        or normalized_member_set(row.get("shadow_member_component_ids", ""))
        in member_sets
    )


def oracle_window(
    rows: list[dict[str, str]], oracle: dict[str, Any]
) -> list[dict[str, str]] | None:
    start = oracle.get("window_start_stamp_sec")
    end = oracle.get("window_end_stamp_sec")
    if not isinstance(start, (int, float)) or not isinstance(end, (int, float)):
        return None
    if not math.isfinite(float(start)) or not math.isfinite(float(end)) or end < start:
        return None
    return [
        row for row in rows
        if start <= number(row, "pipeline_stamp") <= end
    ]


def analyze_trace(
    name: str, path: Path, oracle: dict[str, Any] | None = None
) -> dict[str, Any]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    role = oracle.get("role") if oracle else None
    window_rows = oracle_window(rows, oracle) if oracle else None
    oracle_conclusive = role in {
        "negative_safe_over", "positive_collision", "long_geometry",
        "positive_control",
    } and window_rows is not None and bool(window_rows)
    judged_rows = window_rows if oracle_conclusive else []
    true_ids, true_members = oracle_identity_sets(oracle or {}, "true")
    wrong_ids, wrong_members = oracle_identity_sets(oracle or {}, "wrong")
    identity_oracle_available = bool(
        true_ids or true_members or wrong_ids or wrong_members
    )
    valid_geometry = [
        row for row in judged_rows
        if flag(row, "shadow_geometry_valid_this_frame")
    ]
    evaluated = [
        row for row in valid_geometry
        if flag(row, "shadow_official_valid")
    ]
    validated = [
        row for row in judged_rows
        if row.get("shadow_identity") == "VALIDATED"
    ]
    true_validated = [
        row for row in validated
        if row_matches_identity(row, true_ids, true_members)
    ]
    wrong_locks = [
        row for row in judged_rows
        if flag(row, "formal_lock")
        and row_matches_identity(row, wrong_ids, wrong_members)
    ]
    warning_rows = [
        row for row in evaluated
        if int(number(row, "shadow_code", 0.0)) in (17, 18, 29)
    ]
    contamination = any(
        flag(row, "obstacle_self_contamination_blocking")
        for row in judged_rows
    )
    compute = [number(row, "shadow_total_compute_ms") for row in rows]
    callback_hz = [number(row, "pointcloud_callback_hz") for row in rows]
    processed_hz = [number(row, "ndt_processing_hz") for row in rows]
    dropped = [number(row, "dropped_frame_count") for row in rows]
    large_gaps = [number(row, "large_gap_count") for row in rows]
    result: dict[str, Any] = {
        "bag": name,
        "trace": str(path),
        "bag_role": role or "UNSPECIFIED",
        "oracle_status": (
            "CONCLUSIVE" if oracle_conclusive and identity_oracle_available
            else "ORACLE_INCONCLUSIVE"
        ),
        "oracle_window_frames": len(judged_rows),
        "frames": len(rows),
        "identity_validated": bool(true_validated),
        "geometry_valid_frames": len(valid_geometry),
        "evaluated_frames": len(evaluated),
        "wrong_formal_lock_frames": len(wrong_locks),
        "valid_geometry_warning_17_18_29_frames": len(warning_rows),
        "identity_validated_before_8m": any(
            flag(row, "identity_before_8m") for row in judged_rows
        ),
        "pending_or_lock_ready_before_5m": any(
            flag(row, "ready_before_5m") for row in judged_rows
        ),
        "canonical_far_history_valid": any(
            flag(row, "canonical_far_history_valid") for row in judged_rows
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
    if not oracle_conclusive or not identity_oracle_available:
        result["identity_verdict"] = "ORACLE_INCONCLUSIVE"
        result["vertical_geometry_verdict"] = "ORACLE_INCONCLUSIVE"
        result["timing_verdict"] = "ORACLE_INCONCLUSIVE"
        result["avoidance_cargo_side_verdict"] = "ORACLE_INCONCLUSIVE"
        return result

    if true_ids or true_members:
        result["identity_verdict"] = (
            "PASS" if result["identity_validated"]
            and result["wrong_formal_lock_frames"] == 0 else "FAIL"
        )
    else:
        # The long-piece oracle may prove only that the known wrong-X
        # hypothesis never obtained formal authority.
        result["identity_verdict"] = (
            "PASS" if result["wrong_formal_lock_frames"] == 0 else "FAIL"
        )

    bottom_valid_rows = [row for row in valid_geometry if flag(row, "bottom_valid")]
    collision_rows = [
        row for row in bottom_valid_rows
        if number(row, "clearance") < 0.80
    ]
    unsafe_clear_rows = [
        row for row in collision_rows
        if flag(row, "shadow_official_valid")
        and int(number(row, "shadow_code", 0.0)) == 14
    ]
    if role == "long_geometry" and not valid_geometry:
        result["vertical_geometry_verdict"] = "PARTIAL"
    else:
        result["vertical_geometry_verdict"] = (
            "PASS" if bottom_valid_rows else "FAIL"
        )
    result["timing_verdict"] = (
        "PASS" if role in ("long_geometry", "positive_control")
        else "PASS" if result["identity_validated_before_8m"]
        and result["pending_or_lock_ready_before_5m"] else "FAIL"
    )
    if contamination:
        result["avoidance_cargo_side_verdict"] = (
            "BLOCKED_BY_OBSTACLE_SELF_CONTAMINATION"
        )
    elif role == "negative_safe_over":
        result["avoidance_cargo_side_verdict"] = (
            "PASS" if evaluated and not warning_rows else "FAIL"
        )
    elif role == "positive_collision":
        # 17/18 are expected positive outputs here, never false warnings.
        result["avoidance_cargo_side_verdict"] = (
            "PASS" if collision_rows and not unsafe_clear_rows else "FAIL"
        )
    elif role == "long_geometry" and not valid_geometry:
        result["avoidance_cargo_side_verdict"] = "PARTIAL"
    else:
        result["avoidance_cargo_side_verdict"] = (
            "PASS" if bottom_valid_rows and not unsafe_clear_rows else "FAIL"
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
    parser.add_argument(
        "--oracle", action="append", default=[],
        help="NAME=/path/to/bag_local_oracle.json",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    reports: list[dict[str, Any]] = []
    errors: list[dict[str, str]] = []
    runtime_paths: dict[str, Path] = {}
    oracle_paths: dict[str, Path] = {}
    for item in args.runtime:
        try:
            runtime_name, runtime_path = item.split("=", 1)
            runtime_paths[runtime_name] = Path(runtime_path)
        except ValueError:
            errors.append({"bag": item, "error": "invalid runtime NAME=PATH"})
    for item in args.oracle:
        try:
            oracle_name, oracle_path = item.split("=", 1)
            oracle_paths[oracle_name] = Path(oracle_path)
        except ValueError:
            errors.append({"bag": item, "error": "invalid oracle NAME=PATH"})
    for item in args.bag:
        try:
            name, raw_path = item.split("=", 1)
            oracle = None
            if name in oracle_paths:
                oracle = json.loads(oracle_paths[name].read_text(encoding="utf-8"))
            report = analyze_trace(name, Path(raw_path), oracle)
            if name in runtime_paths:
                report["runtime"] = analyze_runtime(runtime_paths[name])
            reports.append(report)
        except Exception as error:  # continue through all four bags
            errors.append({"bag": item, "error": str(error)})

    identity_values = [report["identity_verdict"] for report in reports]
    geometry_values = [report["vertical_geometry_verdict"] for report in reports]
    timing_values = [report["timing_verdict"] for report in reports]
    avoidance_values = [report["avoidance_cargo_side_verdict"] for report in reports]
    oracle_inconclusive = any(
        report["oracle_status"] != "CONCLUSIVE" for report in reports
    )
    long_partial = any(
        report["bag"].lower() in ("long", "长件")
        and not report["identity_validated"] for report in reports
    )
    summary = {
        "bags": reports,
        "errors": errors,
        "ORACLE_STATUS": (
            "ORACLE_INCONCLUSIVE" if oracle_inconclusive else "CONCLUSIVE"
        ),
        "CARGO_IDENTITY_CORRECTNESS": (
            "ORACLE_INCONCLUSIVE" if oracle_inconclusive else
            "PASS" if reports and all(value == "PASS" for value in identity_values)
            else "PARTIAL" if reports and any(value == "PASS" for value in identity_values)
            else "FAIL"
        ),
        "CARGO_VERTICAL_GEOMETRY_CORRECTNESS": (
            "ORACLE_INCONCLUSIVE" if oracle_inconclusive else
            "PASS" if reports and all(value == "PASS" for value in geometry_values)
            else "PARTIAL" if reports and any(value == "PASS" for value in geometry_values)
            else "FAIL"
        ),
        "AVOIDANCE_TIMING_READINESS": (
            "ORACLE_INCONCLUSIVE" if oracle_inconclusive else
            "PASS" if reports and all(value == "PASS" for value in timing_values)
            else "PARTIAL" if reports and any(value == "PASS" for value in timing_values)
            else "FAIL"
        ),
        "AVOIDANCE_CARGO_SIDE_CORRECTNESS": (
            "ORACLE_INCONCLUSIVE" if oracle_inconclusive else
            "BLOCKED_BY_OBSTACLE_SELF_CONTAMINATION"
            if any(value == "BLOCKED_BY_OBSTACLE_SELF_CONTAMINATION"
                   for value in avoidance_values)
            else "PASS" if reports and all(value == "PASS" for value in avoidance_values)
            else "PARTIAL" if reports and any(value == "PASS" for value in avoidance_values)
            else "FAIL"
        ),
        "GLOBAL_CARGO_AVAILABILITY": "ORACLE_INCONCLUSIVE"
        if oracle_inconclusive else "PARTIAL" if long_partial else (
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
        "ORACLE_INCONCLUSIVE" if oracle_inconclusive else
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
