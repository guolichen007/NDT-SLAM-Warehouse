#!/usr/bin/env python3
"""Machine-readable mainline gate with explicit stability thresholds."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


LOWER_IS_BETTER = {
    "fitness_median": 1.05,
    "fitness_p95": 1.05,
    "fitness_max": 1.10,
    "prediction_only_count": 1.00,
    "odom_step_p99_m": 1.05,
    "cpu_mean_percent": 1.10,
    "rss_peak_mib": 1.10,
    "callback_latency_p95_ms": 1.10,
    "ndt_latency_p95_ms": 1.05,
    "clean_build_latency_p95_ms": 1.10,
    "worker_discard_ratio": 1.05,
    "cargo_lock_valid_frames_p95": 1.00,
    "warning_latency_valid_frames_p95": 1.00,
}
HIGHER_IS_BETTER = {
    "convergence_ratio": 0.99,
    "odom_publish_ratio": 0.99,
    "static_evidence_growth_cells": 0.95,
}
HARD_MAXIMUMS = {
    "archive_queue_peak_jobs": 8.0,
    "archive_queue_peak_mib": 256.0,
    "archive_oldest_job_age_max_sec": 10.0,
    "archive_write_latency_max_ms": 5000.0,
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("control", type=Path)
    parser.add_argument("candidate", type=Path)
    args = parser.parse_args()
    control = json.loads(args.control.read_text(encoding="utf-8"))
    candidate = json.loads(args.candidate.read_text(encoding="utf-8"))
    failures: list[str] = []
    for key, factor in LOWER_IS_BETTER.items():
        if key not in control or key not in candidate:
            failures.append(f"missing metric: {key}")
        else:
            try:
                control_value = float(control[key])
                candidate_value = float(candidate[key])
            except (TypeError, ValueError):
                failures.append(f"invalid metric: {key}")
                continue
            if not math.isfinite(control_value) or not math.isfinite(candidate_value):
                failures.append(f"non-finite metric: {key}")
            elif control_value < 0.0 or candidate_value < 0.0:
                failures.append(f"negative metric: {key}")
            elif candidate_value > control_value * factor:
                failures.append(
                    f"{key} regressed: {candidate[key]} > {control[key]}*{factor}")
    for key, factor in HIGHER_IS_BETTER.items():
        if key not in control or key not in candidate:
            failures.append(f"missing metric: {key}")
        else:
            try:
                control_value = float(control[key])
                candidate_value = float(candidate[key])
            except (TypeError, ValueError):
                failures.append(f"invalid metric: {key}")
                continue
            if not math.isfinite(control_value) or not math.isfinite(candidate_value):
                failures.append(f"non-finite metric: {key}")
            elif control_value < 0.0 or candidate_value < 0.0:
                failures.append(f"negative metric: {key}")
            elif candidate_value < control_value * factor:
                failures.append(
                    f"{key} regressed: {candidate[key]} < {control[key]}*{factor}")
    for key, maximum in HARD_MAXIMUMS.items():
        try:
            value = float(candidate[key])
        except (KeyError, TypeError, ValueError):
            failures.append(f"missing or invalid hard-limit metric: {key}")
            continue
        if not math.isfinite(value) or value < 0.0:
            failures.append(f"invalid hard-limit metric: {key}")
        elif value > maximum:
            failures.append(f"{key} exceeds hard limit: {value} > {maximum}")
    for required_zero in (
        "crash_count",
        "objects_clean_height_spike_count",
        "nearby_object_identity_steal_count",
        "worker_starvation_count",
        "archive_critical_refused_count",
        "archive_incomplete_count",
    ):
        try:
            value = int(candidate[required_zero])
        except (KeyError, TypeError, ValueError):
            failures.append(f"missing or invalid zero metric: {required_zero}")
            continue
        if value != 0:
            failures.append(f"{required_zero} must be zero")
    if failures:
        for failure in failures:
            print(f"FAIL {failure}")
        return 1
    print("PASS MAINLINE_NON_REGRESSION_GATE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
