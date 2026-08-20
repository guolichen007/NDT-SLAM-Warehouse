#!/usr/bin/env python3
"""Analyze dual-LiDAR timestamp/residual evidence without declaring health.

Input is a CSV exported from incident replay. Required columns are
rs201_stamp, rs203_stamp and merged_stamp. Optional columns include
cross_lidar_residual_m, crane_stationary and ghosting_score. Until approved
limits are supplied by a separate Ubuntu gate, health remains UNVERIFIED.
"""

import argparse
import csv
import json
import math
from pathlib import Path


def values(rows, name):
    output = []
    for row in rows:
        try:
            value = float(row.get(name, "nan"))
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            output.append(value)
    return output


def stats(data):
    ordered = sorted(data)
    return {
        "count": len(ordered),
        "mean": sum(ordered) / len(ordered) if ordered else None,
        "p95": ordered[int(0.95 * (len(ordered) - 1))] if ordered else None,
        "maximum": max(ordered) if ordered else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    with args.csv.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    pair_dt_ms = []
    for row in rows:
        stamps = values([row], "rs201_stamp") + values([row], "rs203_stamp")
        if len(stamps) == 2:
            pair_dt_ms.append(abs(stamps[0] - stamps[1]) * 1000.0)
    stationary = [row for row in rows
                  if str(row.get("crane_stationary", "")).lower() in
                  ("1", "true", "yes")]
    moving = [row for row in rows if row not in stationary]
    report = {
        "DUAL_LIDAR_INPUT_HEALTH": "UNVERIFIED",
        "approved_thresholds_present": False,
        "samples": len(rows),
        "pair_dt_ms": stats(pair_dt_ms),
        "cross_lidar_residual_m": stats(values(rows, "cross_lidar_residual_m")),
        "stationary_residual_m": stats(values(stationary, "cross_lidar_residual_m")),
        "moving_residual_m": stats(values(moving, "cross_lidar_residual_m")),
        "ghosting_score": stats(values(rows, "ghosting_score")),
        "correlation_requires_ubuntu_incident_replay": True,
    }
    text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        with args.output.open("w", encoding="utf-8", newline="\n") as stream:
            stream.write(text)
    else:
        print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
