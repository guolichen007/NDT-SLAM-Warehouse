#!/usr/bin/env python3
"""Summarize paired Cargo Vertical Evidence V2 SHADOW traces."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable


def finite(row: dict[str, str], field: str) -> float | None:
    try:
        value = float(row.get(field, "nan"))
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def quantile(values: Iterable[float], probability: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    position = probability * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def load_trace(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {
            "baseline_det_z95", "shadow_top_valid", "shadow_top_z",
            "shadow_bottom_selected", "shadow_clearance",
            "shadow_hazard_code", "shadow_reject_reason",
        }
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(
                f"{path}: missing SHADOW columns: {sorted(missing)}")
        return list(reader)


def report(label: str, rows: list[dict[str, str]]) -> None:
    valid_rows = [row for row in rows if row["shadow_top_valid"] == "1"]

    def values(field: str, source=rows) -> list[float]:
        return [value for row in source
                if (value := finite(row, field)) is not None]

    baseline_top = values("baseline_det_z95")
    shadow_top = values("shadow_top_z", valid_rows)
    shadow_bottom = values("shadow_bottom_selected", valid_rows)
    shadow_clearance = values("shadow_clearance", valid_rows)
    codes = [int(value) for row in rows
             if (value := finite(row, "shadow_hazard_code")) is not None]
    invalid_reasons: dict[str, int] = {}
    for row in rows:
        if row["shadow_top_valid"] == "1":
            continue
        reason = row.get("shadow_reject_reason", "unknown") or "unknown"
        invalid_reasons[reason] = invalid_reasons.get(reason, 0) + 1

    print(f"{label}_FRAMES={len(rows)}")
    print(f"{label}_SHADOW_VALID_FRAMES={len(valid_rows)}")
    print(f"{label}_SHADOW_VALID_RATIO={len(valid_rows) / max(1, len(rows)):.6f}")
    print(f"{label}_BASELINE_TOP_P50={quantile(baseline_top, 0.50):.6f}")
    print(f"{label}_BASELINE_TOP_P95={quantile(baseline_top, 0.95):.6f}")
    print(f"{label}_SHADOW_TOP_P50={quantile(shadow_top, 0.50):.6f}")
    print(f"{label}_SHADOW_TOP_P95={quantile(shadow_top, 0.95):.6f}")
    print(f"{label}_SHADOW_BOTTOM_P50={quantile(shadow_bottom, 0.50):.6f}")
    print(f"{label}_SHADOW_BOTTOM_P95={quantile(shadow_bottom, 0.95):.6f}")
    print(f"{label}_SHADOW_CLEARANCE_P05={quantile(shadow_clearance, 0.05):.6f}")
    print(f"{label}_SHADOW_CLEARANCE_P50={quantile(shadow_clearance, 0.50):.6f}")
    for code in (17, 18, 29):
        print(f"{label}_SHADOW_CODE{code}={codes.count(code)}")
    print(f"{label}_SHADOW_INVALID_REASONS={invalid_reasons}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--wu", type=Path, required=True,
                        help="无.bag frame_causal_trace.csv")
    parser.add_argument("--you", type=Path, required=True,
                        help="有.bag frame_causal_trace.csv")
    args = parser.parse_args()
    report("WU", load_trace(args.wu))
    report("YOU", load_trace(args.you))
    print("VERTICAL_EVIDENCE_V2_VALID=UNVERIFIED_ON_UBUNTU")
    print("PRODUCT_BEHAVIOR_CHANGED=NO")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
