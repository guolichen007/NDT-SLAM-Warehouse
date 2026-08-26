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
from collections import Counter
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
    bounds = oracle_phase_bounds(oracle)
    if bounds is None:
        return None
    start, end = bounds
    return [
        row for row in rows
        if start <= number(row, "pipeline_stamp") < end
    ]


def group_oracle_window(
    rows: list[dict[str, str]], oracle: dict[str, Any]
) -> list[dict[str, str]] | None:
    bounds = oracle_phase_bounds(oracle)
    if bounds is None:
        return None
    start, end = bounds
    return [row for row in rows if start <= number(row, "stamp") < end]


def oracle_phase_bounds(
    oracle: dict[str, Any]
) -> tuple[float, float] | None:
    """Return the explicitly selected v2 [start,end) truth window."""
    if oracle.get("oracle_version") != 2:
        return None
    if oracle.get("time_base") != "bag_source_stamp":
        return None
    phase_name = oracle.get("analysis_phase")
    phases = oracle.get("phases")
    if not isinstance(phase_name, str) or not isinstance(phases, dict):
        return None
    phase = phases.get(phase_name)
    if not isinstance(phase, dict) or phase.get("applicable") is not True:
        return None
    start = phase.get("start_stamp_sec")
    end = phase.get("end_stamp_sec")
    if not isinstance(start, (int, float)) or not isinstance(end, (int, float)):
        return None
    start = float(start)
    end = float(end)
    if not math.isfinite(start) or not math.isfinite(end) or end <= start:
        return None
    return start, end


def oracle_top_range(oracle: dict[str, Any]) -> tuple[float, float] | None:
    phase_name = oracle.get("analysis_phase")
    phases = oracle.get("phases")
    if not isinstance(phase_name, str) or not isinstance(phases, dict):
        return None
    phase = phases.get(phase_name)
    values = phase.get("cargo_top_range_m") if isinstance(phase, dict) else None
    if not isinstance(values, (list, tuple)) or len(values) != 2:
        return None
    low, high = values
    if not isinstance(low, (int, float)) or not isinstance(high, (int, float)):
        return None
    low = float(low)
    high = float(high)
    if not math.isfinite(low) or not math.isfinite(high) or high < low:
        return None
    return low, high


def in_oracle_top_range(value: float, top_range: tuple[float, float] | None) -> bool:
    return top_range is not None and math.isfinite(value) and (
        top_range[0] <= value <= top_range[1]
    )


def validate_oracle_v2(oracle: dict[str, Any]) -> tuple[bool, str]:
    if oracle.get("oracle_version") != 2:
        return False, "oracle_version_must_be_2"
    if oracle.get("time_base") != "bag_source_stamp":
        return False, "time_base_must_be_bag_source_stamp"
    phases = oracle.get("phases")
    if not isinstance(phases, dict):
        return False, "phases_missing"
    for name in ("pre_lift", "lifted", "safe_over", "terminal_lowering"):
        phase = phases.get(name)
        if not isinstance(phase, dict) or not isinstance(
            phase.get("applicable"), bool
        ):
            return False, f"phase_{name}_applicability_missing"
        if phase["applicable"]:
            start = phase.get("start_stamp_sec")
            end = phase.get("end_stamp_sec")
            if not isinstance(start, (int, float)) or not isinstance(
                end, (int, float)
            ) or not math.isfinite(float(start)) or not math.isfinite(
                float(end)
            ) or float(end) <= float(start):
                return False, f"phase_{name}_invalid_half_open_window"
            top_range = phase.get("cargo_top_range_m")
            if not isinstance(top_range, (list, tuple)) or len(top_range) != 2:
                return False, f"phase_{name}_cargo_top_range_missing"
            low, high = top_range
            if not isinstance(low, (int, float)) or not isinstance(
                high, (int, float)
            ) or not math.isfinite(float(low)) or not math.isfinite(
                float(high)
            ) or float(high) < float(low):
                return False, f"phase_{name}_cargo_top_range_invalid"
        elif not str(phase.get("reason", "")).strip():
            return False, f"phase_{name}_not_applicable_reason_missing"
    if oracle_phase_bounds(oracle) is None:
        return False, "analysis_phase_invalid_or_not_applicable"
    return True, "valid"


def group_matches_members(
    row: dict[str, str], member_sets: set[tuple[int, ...]]
) -> bool:
    return normalized_member_set(row.get("canonical_member_ids", "")) in member_sets


def longest_oracle_correct_supported_sequence(
    rows: list[dict[str, str]], top_range: tuple[float, float] | None
) -> int:
    """Apply the existing history gap contract; do not invent a ratio gate."""
    longest = 0
    current = 0
    previous_stamp = math.nan
    previous_history = ""
    for row in sorted(rows, key=lambda value: number(value, "stamp")):
        stamp = number(row, "stamp")
        history = row.get("matched_history_id", "")
        gap_limit = number(row, "maximum_observation_gap_sec")
        source = row.get("vertical_source", "COMPONENT_UNION")
        owned_supported = (
            row.get("vertical_mode") == "SUPPORTED_EVIDENCE" and
            in_oracle_top_range(number(row, "physical_vertical_z"), top_range)
            and (source != "RAW_ROI_CURRENT_FOOTPRINT" or
                 flag(row, "raw_roi_vertical_valid"))
        )
        continuous = (
            current > 0 and history not in ("", "0") and
            history == previous_history and math.isfinite(stamp) and
            math.isfinite(previous_stamp) and math.isfinite(gap_limit) and
            gap_limit > 0.0 and 0.0 < stamp - previous_stamp <= gap_limit
        )
        if owned_supported:
            current = current + 1 if continuous else 1
            longest = max(longest, current)
            previous_stamp = stamp
            previous_history = history
        else:
            current = 0
            previous_stamp = math.nan
            previous_history = ""
    return longest


def analyze_prelift_reference(rows: list[dict[str, str]]) -> dict[str, Any]:
    """Derive per-history prelift reference timelines and the three
    reference-integrity error gates from the V6 group trace (not by string
    matching).

    REFERENCE_SLID_AFTER_FREEZE: once a (history, epoch) first reaches FROZEN,
        its reference first/last stamp, baseline z and baseline source must
        never change on any later FROZEN frame.
    WRONG_HISTORY_REFERENCE_BORROW: a PRE_LOAD_FROZEN_BASELINE must belong to
        a history that actually reached FROZEN.
    REACQUISITION_REFERENCE_AUTHORITY_LEAK: an association-only reacquire must
        not be the frame that first freezes the prelift reference.
    """
    timelines: dict[tuple[str, str], list[dict[str, str]]] = {}
    for row in sorted(rows, key=lambda value: number(value, "stamp")):
        history = row.get("matched_history_id", "")
        if history in ("", "0"):
            continue
        epoch = row.get("physical_cargo_epoch_id", "")
        timelines.setdefault((history, epoch), []).append(row)

    slid_count = 0
    borrow_count = 0
    reacquire_leak_count = 0
    reference_timelines: list[dict[str, Any]] = []

    for (history, epoch), timeline in timelines.items():
        frozen_index = -1
        first: dict[str, str] | None = None
        for index, row in enumerate(timeline):
            if row.get("prelift_state") == "FROZEN":
                frozen_index = index
                first = row
                break
        if first is None:
            if any(
                row.get("lift_baseline_source") == "PRE_LOAD_FROZEN_BASELINE"
                for row in timeline
            ):
                borrow_count += 1
            continue

        first_stamp = first.get("prelift_reference_first_stamp", "")
        last_stamp = first.get("prelift_reference_last_stamp", "")
        baseline_z = first.get("baseline_z", "")
        baseline_source = first.get("lift_baseline_source", "")

        # Every FROZEN frame of this history must keep the identical reference.
        frozen_frames = [
            row for row in timeline if row.get("prelift_state") == "FROZEN"
        ]
        for row in frozen_frames:
            if (
                row.get("prelift_reference_first_stamp", "") != first_stamp
                or row.get("prelift_reference_last_stamp", "") != last_stamp
                or row.get("baseline_z", "") != baseline_z
                or row.get("lift_baseline_source", "") != baseline_source
            ):
                slid_count += 1
                break

        if flag(first, "reacquired_vertical_valid"):
            reacquire_leak_count += 1

        reference_timelines.append({
            "history_id": history,
            "physical_cargo_epoch_id": epoch,
            "prelift_state": first.get("prelift_state", ""),
            "prelift_reference_first_stamp": first_stamp,
            "prelift_reference_last_stamp": last_stamp,
            "baseline_z": baseline_z,
            "baseline_source": baseline_source,
        })

    return {
        "reference_timelines": reference_timelines,
        "reference_slid_after_freeze": slid_count,
        "wrong_history_reference_borrow": borrow_count,
        "reacquisition_reference_authority_leak": reacquire_leak_count,
    }


def analyze_group_trace(
    path: Path, oracle: dict[str, Any] | None,
    baseline_path: Path | None = None,
) -> dict[str, Any]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    window_rows = group_oracle_window(rows, oracle) if oracle else None
    judged = window_rows if window_rows is not None else rows
    _, true_members = oracle_identity_sets(oracle or {}, "true")
    _, wrong_members = oracle_identity_sets(oracle or {}, "wrong")
    true_rows = [row for row in judged if group_matches_members(row, true_members)]
    wrong_rows = [row for row in judged if group_matches_members(row, wrong_members)]
    new_rows = [
        row for row in judged if row.get("association_state") == "NEW_HISTORY"
    ]
    history_lengths = Counter(
        row.get("matched_history_id", "") for row in judged
        if row.get("matched_history_id", "") not in ("", "0")
    )
    vertical_modes = Counter(
        row.get("vertical_mode", "INVALID") or "INVALID" for row in judged
    )
    reject_reasons = Counter(
        row.get("association_reject_reason", "UNKNOWN") or "UNKNOWN"
        for row in judged
    )
    true_validated = any(
        row.get("identity_state") == "VALIDATED" for row in true_rows
    )
    wrong_validated = any(
        row.get("identity_state") == "VALIDATED" for row in wrong_rows
    )
    max_confirm = max(
        (int(number(row, "lift_confirm_count", 0.0)) for row in true_rows),
        default=0,
    )
    top_range = oracle_top_range(oracle or {})
    longest_supported_sequence = longest_oracle_correct_supported_sequence(
        true_rows, top_range
    )
    component_invalid_recovered = sum(
        flag(row, "raw_roi_vertical_valid") and
        not flag(row, "component_vertical_valid") and
        in_oracle_top_range(number(row, "raw_roi_vertical_z"), top_range)
        for row in true_rows
    )
    valid_but_low_corrected = sum(
        flag(row, "raw_roi_vertical_valid") and
        flag(row, "component_vertical_valid") and
        not in_oracle_top_range(
            number(row, "component_vertical_z"), top_range
        ) and in_oracle_top_range(
            number(row, "raw_roi_vertical_z"), top_range
        )
        for row in true_rows
    )
    owner_proof_rejected = sum(
        number(row, "owner_proof_rejected_hypothesis_count", 0.0) > 0.0
        for row in true_rows
    )
    raw_current_still_invalid = sum(
        not flag(row, "raw_roi_vertical_valid") or
        not in_oracle_top_range(
            number(row, "raw_roi_vertical_z"), top_range
        ) for row in true_rows
    )
    raw_recovered_to_oracle_range = sum(
        flag(row, "raw_roi_vertical_valid") and
        in_oracle_top_range(number(row, "raw_roi_vertical_z"), top_range)
        for row in true_rows
    )
    required = max(
        (int(number(row, "lift_confirm_required", 0.0)) for row in true_rows),
        default=0,
    )
    result: dict[str, Any] = {
        "group_trace": str(path),
        "group_rows": len(rows),
        "oracle_group_rows": len(true_rows),
        "true_validated": true_validated,
        "wrong_low_validated": wrong_validated,
        "raw_representative_xy_step_p95": percentile(
            (number(row, "raw_representative_xy_step") for row in judged),
            0.95,
        ),
        "stable_anchor_xy_step_p95": percentile(
            (number(row, "stable_anchor_xy_step") for row in judged), 0.95
        ),
        "new_history_rate_after": (
            len(new_rows) / len(judged) if judged else math.nan
        ),
        "association_reject_reason_distribution": dict(reject_reasons),
        "vertical_mode_distribution": dict(vertical_modes),
        "longest_physical_history": max(history_lengths.values(), default=0),
        "max_lift_confirm_count": max_confirm,
        "lift_confirm_required": required,
        "component_invalid_recovered": component_invalid_recovered,
        "valid_but_low_corrected": valid_but_low_corrected,
        "owner_proof_rejected": owner_proof_rejected,
        "raw_current_footprint_still_invalid": raw_current_still_invalid,
        "raw_recovered_to_oracle_range": raw_recovered_to_oracle_range,
        "longest_oracle_correct_supported_sequence":
            longest_supported_sequence,
    }
    if baseline_path is None:
        result["baseline_comparison"] = "BASELINE_COMPARISON_UNAVAILABLE"
        result["new_history_rate_before"] = math.nan
    else:
        with baseline_path.open(newline="", encoding="utf-8") as stream:
            baseline_rows = list(csv.DictReader(stream))
        baseline_window = (
            group_oracle_window(baseline_rows, oracle) if oracle else None
        )
        baseline_judged = (
            baseline_window if baseline_window is not None else baseline_rows
        )
        baseline_new = sum(
            row.get("association_state") == "NEW_HISTORY"
            for row in baseline_judged
        )
        result["baseline_comparison"] = "AVAILABLE"
        result["new_history_rate_before"] = (
            baseline_new / len(baseline_judged)
            if baseline_judged else math.nan
        )

    if not true_members or window_rows is None:
        result["yes_bag_exit_classification"] = "ORACLE_INCONCLUSIVE"
    elif true_validated:
        result["yes_bag_exit_classification"] = "TRUE_VALIDATED"
    elif required > 0 and longest_supported_sequence >= required:
        result["yes_bag_exit_classification"] = (
            "IDENTITY_LIFT_IMPLEMENTATION_FAIL"
        )
    elif required > 0 and len(true_rows) >= required:
        result["yes_bag_exit_classification"] = (
            "VERTICAL_EVIDENCE_AVAILABILITY_BLOCKING"
        )
    else:
        result["yes_bag_exit_classification"] = (
            "DETECTOR_AVAILABILITY_BLOCKING"
        )
    result.update({
        "RAW_REPRESENTATIVE_XY_STEP_P95":
            result["raw_representative_xy_step_p95"],
        "STABLE_ANCHOR_XY_STEP_P95":
            result["stable_anchor_xy_step_p95"],
        "ASSOCIATION_REJECT_REASON_DISTRIBUTION":
            result["association_reject_reason_distribution"],
        "VERTICAL_MODE_DISTRIBUTION": result["vertical_mode_distribution"],
        "LONGEST_PHYSICAL_HISTORY": result["longest_physical_history"],
        "MAX_LIFT_CONFIRM_COUNT": result["max_lift_confirm_count"],
        "TRUE_VALIDATED": result["true_validated"],
        "WRONG_LOW_VALIDATED": result["wrong_low_validated"],
        "COMPONENT_INVALID_RECOVERED": component_invalid_recovered,
        "VALID_BUT_LOW_CORRECTED": valid_but_low_corrected,
        "OWNER_PROOF_REJECTED": owner_proof_rejected,
        "RAW_CURRENT_FOOTPRINT_STILL_INVALID": raw_current_still_invalid,
        "RAW_RECOVERED_TO_ORACLE_RANGE": raw_recovered_to_oracle_range,
        "LONGEST_ORACLE_CORRECT_SUPPORTED_SEQUENCE":
            longest_supported_sequence,
    })
    # Pre-lift reference timeline spans the full trace (the FROZEN event
    # happens before the Oracle lifted window), so derive it from all rows.
    prelift = analyze_prelift_reference(rows)
    timelines = prelift["reference_timelines"]
    result.update({
        "PRELIFT_REFERENCE_STATE":
            ",".join(sorted({t["prelift_state"] for t in timelines}))
            or "NONE",
        "PRELIFT_REFERENCE_FIRST_STAMP":
            ",".join(sorted({t["prelift_reference_first_stamp"]
                             for t in timelines})) or "NONE",
        "PRELIFT_REFERENCE_LAST_STAMP":
            ",".join(sorted({t["prelift_reference_last_stamp"]
                             for t in timelines})) or "NONE",
        "PRELIFT_BASELINE_Z":
            ",".join(sorted({t["baseline_z"] for t in timelines}))
            or "NONE",
        "PRELIFT_BASELINE_SOURCE":
            ",".join(sorted({t["baseline_source"] for t in timelines}))
            or "NONE",
        "PHYSICAL_CARGO_EPOCH":
            ",".join(sorted({t["physical_cargo_epoch_id"]
                             for t in timelines})) or "NONE",
        "REFERENCE_SLID_AFTER_FREEZE": prelift["reference_slid_after_freeze"],
        "WRONG_HISTORY_REFERENCE_BORROW": prelift[
            "wrong_history_reference_borrow"],
        "REACQUISITION_REFERENCE_AUTHORITY_LEAK": prelift[
            "reacquisition_reference_authority_leak"],
        "LIFT_DELTA_MAX": max(
            (number(row, "lift_delta") for row in true_rows),
            default=math.nan),
        "LIFT_THRESHOLD": max(
            (number(row, "lift_threshold") for row in true_rows),
            default=math.nan),
        "LIFT_CONFIRM_COUNT": result["max_lift_confirm_count"],
    })
    return result


def determine_earliest_root(result: dict[str, Any]) -> str:
    """Single root classifier consumed unchanged by JSON and Markdown."""
    if result.get("oracle_status") != "CONCLUSIVE":
        return "ORACLE_INCONCLUSIVE"
    if result.get("wrong_formal_lock_frames", 0) > 0 or result.get(
        "continuity_v2", {}
    ).get("wrong_low_validated", False):
        return "WRONG_OWNER_AUTHORITY_IMPLEMENTATION_FAIL"
    if not result.get("identity_validated", False):
        continuity = result.get("continuity_v2", {})
        classification = continuity.get(
            "yes_bag_exit_classification", "ORACLE_INCONCLUSIVE"
        )
        if classification in {
            "IDENTITY_LIFT_IMPLEMENTATION_FAIL",
            "VERTICAL_EVIDENCE_AVAILABILITY_BLOCKING",
            "DETECTOR_AVAILABILITY_BLOCKING",
            "ORACLE_INCONCLUSIVE",
        }:
            return classification
        return "UPSTREAM_IDENTITY_BLOCKING"
    if result.get("obstacle_self_contamination_blocking", False) or (
        result.get("bag_role") == "positive_collision" and
        not result.get("canonical_far_history_valid", False)
    ):
        return "OBSTACLE_AUTHORITY_BLOCKING"
    return "NONE"


def analyze_trace(
    name: str, path: Path, oracle: dict[str, Any] | None = None,
    group_path: Path | None = None,
    baseline_group_path: Path | None = None,
    baseline_trace_path: Path | None = None,
) -> dict[str, Any]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    role = oracle.get("role") if oracle else None
    oracle_valid, oracle_reason = validate_oracle_v2(oracle or {})
    window_rows = oracle_window(rows, oracle) if oracle else None
    oracle_conclusive = oracle_valid and role in {
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
    slow_warn_count = sum(
        flag(row, "existing_slow_frame_warn_active") for row in rows
    )
    slow_emergency_count = sum(
        flag(row, "existing_slow_frame_emergency_active") for row in rows
    )
    consecutive_overruns = [
        number(row, "existing_consecutive_overruns", 0.0) for row in rows
    ]
    result: dict[str, Any] = {
        "bag": name,
        "trace": str(path),
        "bag_role": role or "UNSPECIFIED",
        "oracle_status": (
            "CONCLUSIVE" if oracle_conclusive and identity_oracle_available
            else "ORACLE_INCONCLUSIVE"
        ),
        "oracle_reason": oracle_reason,
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
        "existing_slow_frame_warn_count": slow_warn_count,
        "existing_slow_frame_emergency_count": slow_emergency_count,
        "existing_consecutive_overruns_max": max(
            consecutive_overruns, default=0.0
        ),
        "v5_raw_roi_vertical_total_p50_ms": percentile(
            (number(row, "v5_raw_roi_vertical_total_ms") for row in rows),
            0.50,
        ),
        "v5_raw_roi_vertical_total_p95_ms": percentile(
            (number(row, "v5_raw_roi_vertical_total_ms") for row in rows),
            0.95,
        ),
        "v5_raw_roi_vertical_total_max_ms": max(
            (number(row, "v5_raw_roi_vertical_total_ms") for row in rows
             if math.isfinite(number(row, "v5_raw_roi_vertical_total_ms"))),
            default=math.nan,
        ),
        "v5_raw_roi_vertical_hypothesis_count_max": max(
            (number(row, "v5_raw_roi_vertical_hypothesis_count")
             for row in rows), default=0.0,
        ),
        "v5_raw_roi_vertical_points_examined_max": max(
            (number(row, "v5_raw_roi_vertical_points_examined")
             for row in rows), default=0.0,
        ),
        "new_history_rate_after": (
            sum(row.get("shadow_association") == "NEW_HISTORY"
                for row in judged_rows) / len(judged_rows)
            if judged_rows else math.nan
        ),
    }
    owned_top_rows = [
        row for row in judged_rows
        if row.get("downstream_top_source") in {
            "COMPONENT_UNION", "RAW_ROI_CURRENT_FOOTPRINT"
        }
    ]
    owned_bottom_rows = [row for row in owned_top_rows if flag(row, "bottom_valid")]
    if role == "positive_collision" and owned_top_rows and not owned_bottom_rows:
        if any(flag(row, "shadow_thickness_valid") for row in owned_top_rows):
            result["downstream_bottom_blocker"] = (
                "POINTS_BOTTOM_OBSERVABILITY_BLOCKING"
            )
        elif any(flag(row, "formal_lock") for row in owned_top_rows):
            result["downstream_bottom_blocker"] = (
                "FORMAL_THICKNESS_NOT_AVAILABLE"
            )
        else:
            result["downstream_bottom_blocker"] = (
                "FORMAL_THICKNESS_NOT_AVAILABLE"
            )
    else:
        result["downstream_bottom_blocker"] = "NONE"
    result["no_bag_upstream_identity_blocking"] = (
        role == "negative_safe_over" and bool(judged_rows) and
        not bool(true_validated)
    )
    if baseline_trace_path is None:
        result["new_history_rate_before"] = math.nan
        result["baseline_trace_comparison"] = (
            "BASELINE_COMPARISON_UNAVAILABLE"
        )
        result["runtime_regression"] = "BASELINE_COMPARISON_UNAVAILABLE"
    else:
        with baseline_trace_path.open(newline="", encoding="utf-8") as stream:
            baseline_rows = list(csv.DictReader(stream))
        baseline_window = (
            oracle_window(baseline_rows, oracle) if oracle else None
        )
        baseline_judged = (
            baseline_window if baseline_window is not None else baseline_rows
        )
        result["new_history_rate_before"] = (
            sum(row.get("shadow_association") == "NEW_HISTORY"
                for row in baseline_judged) / len(baseline_judged)
            if baseline_judged else math.nan
        )
        result["baseline_trace_comparison"] = "AVAILABLE"
        baseline_callback = [
            number(row, "pointcloud_callback_hz") for row in baseline_rows
        ]
        baseline_processed = [
            number(row, "ndt_processing_hz") for row in baseline_rows
        ]
        baseline_dropped = [
            number(row, "dropped_frame_count") for row in baseline_rows
        ]
        baseline_gaps = [
            number(row, "large_gap_count") for row in baseline_rows
        ]
        baseline_slow_warn = sum(
            flag(row, "existing_slow_frame_warn_active")
            for row in baseline_rows
        )
        baseline_slow_emergency = sum(
            flag(row, "existing_slow_frame_emergency_active")
            for row in baseline_rows
        )
        baseline_overruns = [
            number(row, "existing_consecutive_overruns", 0.0)
            for row in baseline_rows
        ]
        baseline_callback_median = median(
            value for value in baseline_callback
            if math.isfinite(value) and value > 0.0
        ) if any(
            math.isfinite(value) and value > 0.0
            for value in baseline_callback
        ) else math.nan
        baseline_processed_median = median(
            value for value in baseline_processed
            if math.isfinite(value) and value > 0.0
        ) if any(
            math.isfinite(value) and value > 0.0
            for value in baseline_processed
        ) else math.nan
        result["baseline_pointcloud_callback_hz"] = baseline_callback_median
        result["baseline_ndt_processing_hz"] = baseline_processed_median
        result["baseline_slow_frame_warn_count"] = baseline_slow_warn
        result["baseline_slow_frame_emergency_count"] = (
            baseline_slow_emergency
        )
        result["baseline_consecutive_overruns_max"] = max(
            baseline_overruns, default=0.0
        )
        result["callback_hz_delta"] = (
            result["pointcloud_callback_hz"] - baseline_callback_median
            if math.isfinite(result["pointcloud_callback_hz"]) and
            math.isfinite(baseline_callback_median) else math.nan
        )
        result["ndt_processing_hz_delta"] = (
            result["ndt_processing_hz"] - baseline_processed_median
            if math.isfinite(result["ndt_processing_hz"]) and
            math.isfinite(baseline_processed_median) else math.nan
        )
        no_counter_regression = (
            result["dropped_frame_count"] <= max(baseline_dropped, default=0.0)
            and result["large_gap_count"] <= max(baseline_gaps, default=0.0)
            and (slow_emergency_count == 0 or baseline_slow_emergency > 0)
            and (slow_warn_count == 0 or baseline_slow_warn > 0)
            and result["existing_consecutive_overruns_max"] <=
                max(baseline_overruns, default=0.0)
        )
        result["runtime_regression"] = (
            "PASS" if no_counter_regression else "FAIL"
        )
    result["NEW_HISTORY_RATE_BEFORE"] = result["new_history_rate_before"]
    result["NEW_HISTORY_RATE_AFTER"] = result["new_history_rate_after"]
    if group_path is not None:
        result["continuity_v2"] = analyze_group_trace(
            group_path, oracle, baseline_group_path
        )
    else:
        result["continuity_v2"] = {
            "group_trace": "MISSING",
            "baseline_comparison": "BASELINE_COMPARISON_UNAVAILABLE",
            "yes_bag_exit_classification": "ORACLE_INCONCLUSIVE",
        }
    if not oracle_conclusive or not identity_oracle_available:
        result["identity_verdict"] = "ORACLE_INCONCLUSIVE"
        result["vertical_geometry_verdict"] = "ORACLE_INCONCLUSIVE"
        result["timing_verdict"] = "ORACLE_INCONCLUSIVE"
        result["avoidance_cargo_side_verdict"] = "ORACLE_INCONCLUSIVE"
        result["EARLIEST_ROOT"] = determine_earliest_root(result)
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
    result["EARLIEST_ROOT"] = determine_earliest_root(result)
    return result


def analyze_runtime(path: Path) -> dict[str, float]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    cpu = [number(row, "cpu_percent") for row in rows]
    rss = [number(row, "rss_mb") for row in rows]
    result = {
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
    timing_fields = {
        "fixed_yaw_solver_ms": "runtime_fixed_yaw_solver_ms",
        "rail_pose_fitness_ms": "runtime_rail_pose_fitness_ms",
        "target_normal_cache_build_ms": (
            "runtime_target_normal_cache_build_ms"
        ),
        "rail_graph_worker_ms": "runtime_rail_graph_worker_ms",
        "whole_frame_ms": "runtime_average_process_time_ms",
    }
    for output_name, field_name in timing_fields.items():
        values = [number(row, field_name) for row in rows]
        finite = [value for value in values if math.isfinite(value)]
        result[f"{output_name}_p50"] = percentile(finite, 0.50)
        result[f"{output_name}_p95"] = percentile(finite, 0.95)
        result[f"{output_name}_max"] = max(finite, default=math.nan)
    return result


def render_markdown(summary: dict[str, Any]) -> str:
    """Render only the already-decided AnalysisResult; never reclassify."""
    lines = [
        "# Cargo V6 SHADOW analysis",
        "",
        f"- `CARGO_V6_CANDIDATE_SHA={summary.get('CARGO_V6_CANDIDATE_SHA', '')}`",
        f"- `CARGO_V6_SHADOW_IMPLEMENTATION_GATE="
        f"{summary['CARGO_V6_SHADOW_IMPLEMENTATION_GATE']}`",
        f"- `CARGO_V6_SHADOW_FUNCTION_GATE="
        f"{summary['CARGO_V6_SHADOW_FUNCTION_GATE']}`",
        f"- `CARGO_V6_SHADOW_RESULT={summary['CARGO_V6_SHADOW_RESULT']}`",
        f"- `CARGO_V6_RUNTIME_REGRESSION="
        f"{summary['CARGO_V6_RUNTIME_REGRESSION']}`",
        f"- `REFERENCE_SLID_AFTER_FREEZE="
        f"{summary['REFERENCE_SLID_AFTER_FREEZE']}`",
        f"- `WRONG_HISTORY_REFERENCE_BORROW="
        f"{summary['WRONG_HISTORY_REFERENCE_BORROW']}`",
        f"- `REACQUISITION_REFERENCE_AUTHORITY_LEAK="
        f"{summary['REACQUISITION_REFERENCE_AUTHORITY_LEAK']}`",
        f"- `EARLIEST_REMAINING_BLOCKER={summary['EARLIEST_REMAINING_BLOCKER']}`",
        "",
        "| Bag | Oracle | Identity | Vertical | Avoidance | EARLIEST_ROOT |",
        "|---|---|---|---|---|---|",
    ]
    for report in summary["bags"]:
        lines.append(
            "| {bag} | {oracle_status} | {identity_verdict} | "
            "{vertical_geometry_verdict} | {avoidance_cargo_side_verdict} | "
            "{EARLIEST_ROOT} |".format(**report)
        )
    return "\n".join(lines) + "\n"


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
        "--groups", action="append", default=[],
        help="NAME=/path/to/integrated_identity_groups.csv",
    )
    parser.add_argument(
        "--baseline-groups", action="append", default=[],
        help="NAME=/path/to/baseline_integrated_identity_groups.csv",
    )
    parser.add_argument(
        "--baseline", action="append", default=[],
        help="NAME=/path/to/baseline_integrated_avoidance_shadow.csv",
    )
    parser.add_argument(
        "--oracle", action="append", default=[],
        help="NAME=/path/to/bag_local_oracle.json",
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--markdown-output", type=Path)
    parser.add_argument("--source-sha", default="")
    args = parser.parse_args()
    reports: list[dict[str, Any]] = []
    errors: list[dict[str, str]] = []
    runtime_paths: dict[str, Path] = {}
    group_paths: dict[str, Path] = {}
    baseline_group_paths: dict[str, Path] = {}
    baseline_trace_paths: dict[str, Path] = {}
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
    for item in args.groups:
        try:
            group_name, group_path = item.split("=", 1)
            group_paths[group_name] = Path(group_path)
        except ValueError:
            errors.append({"bag": item, "error": "invalid groups NAME=PATH"})
    for item in args.baseline_groups:
        try:
            baseline_name, baseline_path = item.split("=", 1)
            baseline_group_paths[baseline_name] = Path(baseline_path)
        except ValueError:
            errors.append({
                "bag": item, "error": "invalid baseline-groups NAME=PATH"
            })
    for item in args.baseline:
        try:
            baseline_name, baseline_path = item.split("=", 1)
            baseline_trace_paths[baseline_name] = Path(baseline_path)
        except ValueError:
            errors.append({"bag": item, "error": "invalid baseline NAME=PATH"})
    for item in args.bag:
        try:
            name, raw_path = item.split("=", 1)
            oracle = None
            if name in oracle_paths:
                oracle = json.loads(oracle_paths[name].read_text(encoding="utf-8"))
            report = analyze_trace(
                name, Path(raw_path), oracle, group_paths.get(name),
                baseline_group_paths.get(name), baseline_trace_paths.get(name),
            )
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
        "PRODUCT_LOGIC_CHANGED": "NO",
        "PRODUCT_OUTPUT_AUTHORITY_CHANGED": "NO",
        "CARGO_V6_PRODUCT_TAKEOVER": "NOT_PERFORMED",
        "FIELD_READY": "NO",
    }
    summary["CONTINUITY_V2_BAG_CLASSIFICATIONS"] = {
        report["bag"]: report["continuity_v2"].get(
            "yes_bag_exit_classification", "ORACLE_INCONCLUSIVE"
        )
        for report in reports
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
    runtime_values = [
        report.get("runtime_regression", "BASELINE_COMPARISON_UNAVAILABLE")
        for report in reports
    ]
    summary["CARGO_V6_CANDIDATE_SHA"] = args.source_sha
    summary["CARGO_V6_RUNTIME_REGRESSION"] = (
        "PASS" if len(runtime_values) == 4 and
        all(value == "PASS" for value in runtime_values) else "FAIL"
    )
    unsafe_implementation = any(
        report.get("wrong_formal_lock_frames", 0) > 0 or
        report.get("continuity_v2", {}).get("wrong_low_validated", False)
        for report in reports
    )
    summary["CARGO_V6_SHADOW_IMPLEMENTATION_GATE"] = (
        "PASS" if len(reports) == 4 and not errors and not oracle_inconclusive
        and not unsafe_implementation
        and summary["CARGO_V6_RUNTIME_REGRESSION"] == "PASS" else "FAIL"
    )
    summary["CARGO_V6_STABLE_SHA"] = (
        args.source_sha
        if summary["CARGO_V6_SHADOW_IMPLEMENTATION_GATE"] == "PASS" else ""
    )
    functional_values = (
        summary["CARGO_IDENTITY_CORRECTNESS"],
        summary["CARGO_VERTICAL_GEOMETRY_CORRECTNESS"],
        summary["AVOIDANCE_CARGO_SIDE_CORRECTNESS"],
    )
    summary["CARGO_V6_SHADOW_FUNCTION_GATE"] = (
        "PASS" if all(value == "PASS" for value in functional_values)
        else "PARTIAL" if any(value == "PASS" for value in functional_values)
        else "FAIL"
    )
    summary["CARGO_V6_SHADOW_RESULT"] = summary["INTEGRATED_SHADOW_VERDICT"]

    def reference_gate(name: str) -> int:
        return sum(
            report.get("continuity_v2", {}).get(name, 0)
            for report in reports
        )

    summary["REFERENCE_SLID_AFTER_FREEZE"] = reference_gate(
        "REFERENCE_SLID_AFTER_FREEZE"
    )
    summary["WRONG_HISTORY_REFERENCE_BORROW"] = reference_gate(
        "WRONG_HISTORY_REFERENCE_BORROW"
    )
    summary["REACQUISITION_REFERENCE_AUTHORITY_LEAK"] = reference_gate(
        "REACQUISITION_REFERENCE_AUTHORITY_LEAK"
    )
    summary["PRELIFT_REFERENCE_BY_BAG"] = {
        report["bag"]: {
            key: report.get("continuity_v2", {}).get(key, "N/A")
            for key in (
                "PRELIFT_REFERENCE_STATE",
                "PRELIFT_REFERENCE_FIRST_STAMP",
                "PRELIFT_REFERENCE_LAST_STAMP",
                "PRELIFT_BASELINE_Z",
                "PRELIFT_BASELINE_SOURCE",
                "PHYSICAL_CARGO_EPOCH",
                "LIFT_DELTA_MAX",
                "LIFT_THRESHOLD",
                "LIFT_CONFIRM_COUNT",
            )
        }
        for report in reports
    }
    root_priority = (
        "WRONG_OWNER_AUTHORITY_IMPLEMENTATION_FAIL",
        "IDENTITY_LIFT_IMPLEMENTATION_FAIL",
        "VERTICAL_EVIDENCE_AVAILABILITY_BLOCKING",
        "DETECTOR_AVAILABILITY_BLOCKING",
        "UPSTREAM_IDENTITY_BLOCKING",
        "OBSTACLE_AUTHORITY_BLOCKING",
        "ORACLE_INCONCLUSIVE",
    )
    bag_roots = {report.get("EARLIEST_ROOT", "ORACLE_INCONCLUSIVE")
                 for report in reports}
    summary["EARLIEST_REMAINING_BLOCKER"] = next(
        (root for root in root_priority if root in bag_roots), "NONE"
    )
    summary["CARGO_V6_HARD_STOP"] = "YES"
    rendered = json.dumps(summary, ensure_ascii=False, indent=2)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    if args.markdown_output:
        args.markdown_output.write_text(
            render_markdown(summary), encoding="utf-8"
        )
    return 0 if not errors else 2


if __name__ == "__main__":
    raise SystemExit(main())
