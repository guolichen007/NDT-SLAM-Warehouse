#!/usr/bin/env python3
"""Summarize free-NDT versus fixed-yaw rail SHADOW CSV evidence."""

import argparse
import csv
import json
import math
from pathlib import Path


def finite(row, key):
    try:
        value = float(row.get(key, "nan"))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def percentile(values, fraction):
    values = sorted(value for value in values if value is not None)
    if not values:
        return None
    return values[min(len(values) - 1, int(round((len(values) - 1) * fraction)))]


def summarize(rows):
    output = {}
    for bucket in ("ALL_TIME", "CRANE_STATIONARY", "CRANE_MOVING"):
        selected = rows if bucket == "ALL_TIME" else [
            row for row in rows if row.get("motion_bucket") == bucket]
        report = {"samples": len(selected)}
        valid = [row.get("rail_registration_valid") == "1" for row in selected]
        report["rail_refinement_valid_ratio"] = (
            sum(valid) / len(valid) if valid else None)
        for key in ("raw_config_innovation_deg", "translation_delta_m",
                    "fitness_delta", "rail_refine_time_ms"):
            values = [finite(row, key) for row in selected]
            values = [value for value in values if value is not None]
            report[key] = {
                "count": len(values),
                "p50": percentile(values, 0.50),
                "p95": percentile(values, 0.95),
                "maximum_abs": max((abs(value) for value in values), default=None),
            }
        output[bucket] = report
    return {
        "thresholds_applied": False,
        "RAIL_CONSTRAINED_XY_VALID": "UNVERIFIED",
        "buckets": output,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    with args.csv.open("r", encoding="utf-8", newline="") as stream:
        report = summarize(list(csv.DictReader(stream)))
    text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        with args.output.open("w", encoding="utf-8", newline="\n") as stream:
            stream.write(text)
    else:
        print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
