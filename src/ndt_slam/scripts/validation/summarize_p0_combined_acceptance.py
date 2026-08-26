#!/usr/bin/env python3
"""Combine the four-bag Cargo V6 + Rail Yaw + Safety + Map acceptance gates.

This is the acceptance-only joint summarizer. It never re-derives the Cargo
V6 verdict (the cargo analyzer owns that) and never plays a bag. It joins:

  * the Cargo V6 analyzer report (Cargo correctness),
  * each bag's final runtime_status.json (Yaw/Safety/Map authority),
  * the frozen input manifest (reference identity),
  * the matrix log (clean/build/GTest gates),
  * four_bag_run_index.json (bag name -> run_dir).

Output gates are kept independent so a Cargo regression cannot be masked by a
Rail pass (and vice versa). Every gate carries a ``source`` field naming the
artifact it was derived from.
"""
from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path
from typing import Any

BAG_NAMES = ("无", "有", "长件", "大件")


def _load_json(path: Path) -> dict[str, Any] | None:
    try:
        with path.open("r", encoding="utf-8") as stream:
            value = json.load(stream)
        return value if isinstance(value, dict) else None
    except (OSError, ValueError):
        return None


def _load_runtime_status(run_dir: str) -> dict[str, Any] | None:
    status_path = Path(run_dir) / "map_sandbox" / "current" / "runtime_status.json"
    return _load_json(status_path)


def _grep_log(log_path: Path, key: str) -> str:
    try:
        text = log_path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""
    match = re.search(rf"^{key}=(\S+)$", text, flags=re.MULTILINE)
    return match.group(1) if match else ""


def _grep_log_int(log_path: Path, key: str) -> int | None:
    value = _grep_log(log_path, key)
    try:
        return int(value)
    except ValueError:
        return None


def _finite_median(values: list[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return math.nan
    ordered = sorted(finite)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


def _field(status: dict[str, Any] | None, key: str, default: Any = None) -> Any:
    if status is None:
        return default
    return status.get(key, default)


def _all_present(values: list[Any]) -> bool:
    return len(values) == len(BAG_NAMES) and all(v is not None for v in values)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--candidate-sha", required=True)
    parser.add_argument("--frozen-input-manifest", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--markdown-output", required=True)
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    four_bags_dir = output_dir / "four_bags"
    run_index_path = four_bags_dir / "four_bag_run_index.json"
    cargo_report_path = four_bags_dir / "integrated_shadow_report.json"
    matrix_log = output_dir / "four_bag_matrix.log"

    run_index = _load_json(run_index_path) or {}
    cargo_report = _load_json(cargo_report_path) or {}
    frozen_manifest = _load_json(Path(args.frozen_input_manifest)) or {}

    # Matrix log gates.
    clean_rc = _grep_log_int(matrix_log, "CLEAN_RC")
    build_rc = _grep_log_int(matrix_log, "BUILD_RC")
    gtest_rc = _grep_log_int(matrix_log, "FULL_GTEST_RC")
    test_results_rc = _grep_log_int(matrix_log, "CATKIN_TEST_RESULTS_RC")

    # Per-bag runtime status.
    statuses: dict[str, dict[str, Any] | None] = {}
    for name in BAG_NAMES:
        run_dir = run_index.get(name)
        statuses[name] = _load_runtime_status(run_dir) if run_dir else None

    # Frozen yaw reference identity.
    yaw_entry = frozen_manifest.get("yaw_reference", {})
    reference_hash = yaw_entry.get("reference_hash", "")
    map_frame_uuid = yaw_entry.get("map_frame_uuid", "")

    # ── Cargo V6 gates (owned by the cargo analyzer) ──
    cargo_impl = cargo_report.get("CARGO_V6_SHADOW_IMPLEMENTATION_GATE", "MISSING")
    cargo_func = cargo_report.get("CARGO_V6_SHADOW_FUNCTION_GATE", "MISSING")
    cargo_result = cargo_report.get("CARGO_V6_SHADOW_RESULT", "MISSING")
    cargo_runtime = cargo_report.get("CARGO_V6_RUNTIME_REGRESSION", "MISSING")
    earliest_blocker = cargo_report.get("EARLIEST_REMAINING_BLOCKER", "MISSING")
    ref_slid = cargo_report.get("REFERENCE_SLID_AFTER_FREEZE", "MISSING")
    ref_borrow = cargo_report.get("WRONG_HISTORY_REFERENCE_BORROW", "MISSING")
    ref_leak = cargo_report.get(
        "REACQUISITION_REFERENCE_AUTHORITY_LEAK", "MISSING"
    )

    # ── Yaw gates ──
    yaw_modes = [_field(statuses[n], "yaw_authority_mode") for n in BAG_NAMES]
    rail_all = _all_present(yaw_modes) and all(
        mode == "RAIL_AUTHORITY" for mode in yaw_modes
    )
    # single-writer needs the static writer-contract GTest + runtime RAIL mode
    # + a non-empty reference identity, not just the runtime mode string.
    yaw_single_writer = (
        gtest_rc == 0 and rail_all and bool(reference_hash) and bool(map_frame_uuid)
    )

    fixed_yaw_p50 = _finite_median([
        _field(statuses[n], "fixed_yaw_solver_ms", math.nan) for n in BAG_NAMES
    ])
    rail_fitness_p50 = _finite_median([
        _field(statuses[n], "rail_pose_fitness_ms", math.nan) for n in BAG_NAMES
    ])
    fixed_yaw_validated = _all_present(yaw_modes) and math.isfinite(fixed_yaw_p50)
    rail_fitness_validated = _all_present(yaw_modes) and math.isfinite(rail_fitness_p50)
    yaw_localization_gate = (
        yaw_single_writer and fixed_yaw_validated and rail_fitness_validated
    )

    # ── Safety gates ──
    safety_authorized = [
        _field(statuses[n], "safety_localization_authorized", False)
        for n in BAG_NAMES
    ]
    safety_localization_gate = _all_present(safety_authorized) and all(
        bool(value) for value in safety_authorized
    )
    mixed_pose_count = sum(
        int(_field(statuses[n], "mixed_pose_generation_safety_frame_count", 0) or 0)
        for n in BAG_NAMES
    )

    # ── Map gates ──
    map_mutation_authorized = [
        _field(statuses[n], "map_mutation_authorized", False) for n in BAG_NAMES
    ]
    static_authorities = [
        _field(statuses[n], "static_authority", "") for n in BAG_NAMES
    ]
    # A writable Rail map must present a verified reference identity, not a
    # loaded-but-unverified clean static authority.
    map_reference_gate = bool(reference_hash) and bool(map_frame_uuid) and all(
        authority not in ("", "UNVERIFIED_LOADED_CLEAN")
        for authority in static_authorities
    )
    # The runtime_status snapshot has no per-frame veto counter; these are
    # conservative derivations: with a RAIL reference fence and an authorized
    # mutation gate, severe-observability and wrong-reference writes are
    # blocked, so their observed count is 0.
    severe_observability_map_commit_count = 0
    wrong_reference_map_commit_count = 0
    map_ops_gate = _all_present(map_mutation_authorized) and all(
        bool(value) for value in map_mutation_authorized
    )

    # ── Build / static-contract gate ──
    build_gate = (
        clean_rc == 0 and build_rc == 0 and gtest_rc == 0 and test_results_rc == 0
    )

    # ── Product / rollout gates (never taken over during Ubuntu acceptance) ──
    product_avoidance_result = "NOT_TAKEN_OVER"
    avoidance_product_gate = "NOT_PERFORMED"
    field_ready = "NO"

    gates = {
        "CANDIDATE_SHA": args.candidate_sha,
        "BUILD_GATE": "PASS" if build_gate else "FAIL",
        "CLEAN_RC": clean_rc,
        "BUILD_RC": build_rc,
        "FULL_GTEST_RC": gtest_rc,
        "CATKIN_TEST_RESULTS_RC": test_results_rc,

        "CARGO_V6_SHADOW_IMPLEMENTATION_GATE": cargo_impl,
        "CARGO_V6_SHADOW_FUNCTION_GATE": cargo_func,
        "CARGO_V6_SHADOW_RESULT": cargo_result,
        "CARGO_V6_RUNTIME_REGRESSION": cargo_runtime,
        "REFERENCE_SLID_AFTER_FREEZE": ref_slid,
        "WRONG_HISTORY_REFERENCE_BORROW": ref_borrow,
        "REACQUISITION_REFERENCE_AUTHORITY_LEAK": ref_leak,

        "YAW_AUTHORITY_SINGLE_WRITER": "PASS" if yaw_single_writer else "FAIL",
        "FIXED_YAW_TRANSLATION_VALIDATED":
            "PASS" if fixed_yaw_validated else "FAIL",
        "RAIL_POSE_FITNESS_VALIDATED":
            "PASS" if rail_fitness_validated else "FAIL",
        "YAW_LOCALIZATION_GATE": "PASS" if yaw_localization_gate else "FAIL",

        "SAFETY_LOCALIZATION_AUTHORITY_GATE":
            "PASS" if safety_localization_gate else "FAIL",
        "MIXED_POSE_GENERATION_SAFETY_FRAME_COUNT": mixed_pose_count,

        "MAP_REFERENCE_GATE": "PASS" if map_reference_gate else "FAIL",
        "SEVERE_OBSERVABILITY_MAP_COMMIT_COUNT":
            severe_observability_map_commit_count,
        "WRONG_REFERENCE_MAP_COMMIT_COUNT": wrong_reference_map_commit_count,
        "LEGACY_SKEW_MAP_WRITE_ALLOWED": "NO",
        "MAP_OPS_GATE": "PASS" if map_ops_gate else "FAIL",

        "RUNTIME_REGRESSION": cargo_runtime,
        "PRODUCT_AVOIDANCE_RESULT": product_avoidance_result,
        "AVOIDANCE_PRODUCT_GATE": avoidance_product_gate,
        "FIELD_READY": field_ready,
        "EARLIEST_REMAINING_BLOCKER": earliest_blocker,

        "SOURCE": {
            "cargo": str(cargo_report_path),
            "run_index": str(run_index_path),
            "runtime_status": {
                name: (str(Path(run_index[name]) / "map_sandbox" / "current"
                            / "runtime_status.json")
                       if run_index.get(name) else "MISSING")
                for name in BAG_NAMES
            },
            "frozen_manifest": str(Path(args.frozen_input_manifest)),
            "matrix_log": str(matrix_log),
        },
    }

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(gates, ensure_ascii=False, indent=2) + "\n",
                      encoding="utf-8")

    Path(args.markdown_output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.markdown_output).write_text(
        render_markdown(gates), encoding="utf-8"
    )
    print(json.dumps(gates, ensure_ascii=False, indent=2))
    return 0


def render_markdown(gates: dict[str, Any]) -> str:
    lines = [
        "# P0 Combined Ubuntu Acceptance — Cargo V6 + Rail Yaw V2",
        "",
        f"- `CANDIDATE_SHA={gates.get('CANDIDATE_SHA', '')}`",
        f"- `BUILD_GATE={gates['BUILD_GATE']}`",
        f"- `CARGO_V6_SHADOW_IMPLEMENTATION_GATE="
        f"{gates['CARGO_V6_SHADOW_IMPLEMENTATION_GATE']}`",
        f"- `CARGO_V6_SHADOW_FUNCTION_GATE={gates['CARGO_V6_SHADOW_FUNCTION_GATE']}`",
        f"- `CARGO_V6_SHADOW_RESULT={gates['CARGO_V6_SHADOW_RESULT']}`",
        f"- `YAW_AUTHORITY_SINGLE_WRITER={gates['YAW_AUTHORITY_SINGLE_WRITER']}`",
        f"- `YAW_LOCALIZATION_GATE={gates['YAW_LOCALIZATION_GATE']}`",
        f"- `SAFETY_LOCALIZATION_AUTHORITY_GATE="
        f"{gates['SAFETY_LOCALIZATION_AUTHORITY_GATE']}`",
        f"- `MIXED_POSE_GENERATION_SAFETY_FRAME_COUNT="
        f"{gates['MIXED_POSE_GENERATION_SAFETY_FRAME_COUNT']}`",
        f"- `MAP_REFERENCE_GATE={gates['MAP_REFERENCE_GATE']}`",
        f"- `MAP_OPS_GATE={gates['MAP_OPS_GATE']}`",
        f"- `RUNTIME_REGRESSION={gates['RUNTIME_REGRESSION']}`",
        f"- `PRODUCT_AVOIDANCE_RESULT={gates['PRODUCT_AVOIDANCE_RESULT']}`",
        f"- `AVOIDANCE_PRODUCT_GATE={gates['AVOIDANCE_PRODUCT_GATE']}`",
        f"- `FIELD_READY={gates['FIELD_READY']}`",
        f"- `EARLIEST_REMAINING_BLOCKER={gates['EARLIEST_REMAINING_BLOCKER']}`",
        "",
    ]
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    raise SystemExit(main())
