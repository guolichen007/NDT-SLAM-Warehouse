#!/usr/bin/env python3
"""
analyze_588_runtime.py — Analyze runtime_frames.csv for 588 V4 acceptance.

Usage:
  python3 analyze_588_runtime.py runtime_frames.csv \
    --baseline baseline_metrics.json \
    --output analysis_result.json

Exit code:
  0 = PASS
  1 = FAIL
"""

import argparse
import csv
import json
import sys
from collections import Counter


def load_csv(path):
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        return list(reader)


def percentile(sorted_vals, p):
    if not sorted_vals:
        return 0.0
    idx = int(len(sorted_vals) * p)
    idx = min(idx, len(sorted_vals) - 1)
    return sorted_vals[idx]


def analyze(rows):
    if not rows:
        return {"status": "FAIL", "reason": "no data"}

    n = len(rows)

    # Parse numeric fields
    def col(name):
        vals = []
        for r in rows:
            try:
                vals.append(float(r[name]))
            except (ValueError, KeyError):
                pass
        return vals

    total_ms = sorted(col("total_ms"))
    ndt_ms = sorted(col("ndt_align_ms"))
    fitness_vals = sorted(col("fitness"))
    raw_step = col("raw_step_m")
    output_step = col("output_step_m")
    allowed_step = col("allowed_step_m")
    innovation = col("innovation_m")

    # Convergence
    converged = sum(1 for r in rows if r.get("ndt_converged", "0") == "1")
    convergence_ratio = converged / n if n > 0 else 0

    # Prediction-only
    pred_only = [r for r in rows if r.get("prediction_only", "0") == "1"]
    pred_count = len(pred_only)
    pred_ratio = pred_count / n if n > 0 else 0

    # Max prediction streak
    max_streak = 0
    current_streak = 0
    for r in rows:
        if r.get("prediction_only", "0") == "1":
            current_streak += 1
            max_streak = max(max_streak, current_streak)
        else:
            current_streak = 0

    # Target source counts
    target_counts = Counter(r.get("target_source", "unknown") for r in rows)

    # Fitness > 1.0 count
    fitness_over_1 = sum(1 for v in fitness_vals if v > 1.0)

    # Raw step exceed count
    raw_step_exceed = 0
    output_step_violation = 0
    for r in rows:
        try:
            rs = float(r.get("raw_step_m", 0))
            al = float(r.get("allowed_step_m", 0))
            os_val = float(r.get("output_step_m", 0))
            if rs > al + 0.001 and al > 0:
                raw_step_exceed += 1
            if os_val > al + 0.0001 and al > 0:
                output_step_violation += 1
        except ValueError:
            pass

    # Cropped target disaster count
    cropped_disaster = 0
    for r in rows:
        if r.get("target_source") == "cropped_localization_target":
            try:
                f = float(r.get("fitness", 0))
                rs = float(r.get("raw_step_m", 0))
                al = float(r.get("allowed_step_m", 0))
                po = r.get("prediction_only", "0") == "1"
                if f > 1.0 or (al > 0 and rs > al) or po:
                    cropped_disaster += 1
            except ValueError:
                pass

    # EKF reject reasons
    reject_reasons = Counter(r.get("prediction_reason", "NONE") for r in rows)

    result = {
        "total_frames": n,
        "converged": converged,
        "convergence_ratio": round(convergence_ratio, 4),
        "total_ms_p50": round(percentile(total_ms, 0.50), 2),
        "total_ms_p95": round(percentile(total_ms, 0.95), 2),
        "total_ms_p99": round(percentile(total_ms, 0.99), 2),
        "total_ms_max": round(max(total_ms), 2) if total_ms else 0,
        "ndt_ms_p50": round(percentile(ndt_ms, 0.50), 2),
        "ndt_ms_p95": round(percentile(ndt_ms, 0.95), 2),
        "ndt_ms_p99": round(percentile(ndt_ms, 0.99), 2),
        "fitness_p50": round(percentile(fitness_vals, 0.50), 4),
        "fitness_p95": round(percentile(fitness_vals, 0.95), 4),
        "fitness_max": round(max(fitness_vals), 4) if fitness_vals else 0,
        "fitness_over_1_count": fitness_over_1,
        "prediction_only_count": pred_count,
        "prediction_only_ratio": round(pred_ratio, 4),
        "max_prediction_streak": max_streak,
        "raw_step_exceed_count": raw_step_exceed,
        "output_step_violation_count": output_step_violation,
        "cropped_target_disaster_count": cropped_disaster,
        "target_source_counts": dict(target_counts),
        "reject_reason_counts": dict(reject_reasons),
    }

    return result


def check_verdict(metrics, baseline=None):
    """Return (verdict, reasons). verdict is 'PASS' or 'FAIL'."""
    reasons = []

    # Hard failures
    if metrics.get("output_step_violation_count", 0) > 0:
        reasons.append("output_step_violation > 0")

    if metrics.get("convergence_ratio", 0) < 0.98:
        reasons.append(f"convergence_ratio={metrics['convergence_ratio']} < 0.98")

    if metrics.get("fitness_p95", 0) >= 0.50:
        reasons.append(f"fitness P95={metrics['fitness_p95']} >= 0.50")

    if metrics.get("fitness_max", 0) >= 1.0:
        reasons.append(f"fitness max={metrics['fitness_max']} >= 1.0")

    if metrics.get("prediction_only_ratio", 0) > 0.01:
        reasons.append(f"prediction_only_ratio={metrics['prediction_only_ratio']} > 0.01")

    if metrics.get("max_prediction_streak", 0) > 2:
        reasons.append(f"max_prediction_streak={metrics['max_prediction_streak']} > 2")

    if metrics.get("cropped_target_disaster_count", 0) > 0:
        reasons.append(f"cropped_target_disaster={metrics['cropped_target_disaster_count']} > 0")

    # Check if cropped_localization_target was used formally
    target_counts = metrics.get("target_source_counts", {})
    cropped_count = target_counts.get("cropped_localization_target", 0)
    if cropped_count > 0:
        reasons.append(f"cropped_localization_target used {cropped_count} times")

    verdict = "PASS" if not reasons else "FAIL"
    return verdict, reasons


def main():
    parser = argparse.ArgumentParser(description="Analyze 588 V4 runtime CSV")
    parser.add_argument("csv", help="Path to runtime_frames.csv")
    parser.add_argument("--baseline", help="Path to baseline_metrics.json for comparison")
    parser.add_argument("--output", help="Output JSON path")
    args = parser.parse_args()

    rows = load_csv(args.csv)
    metrics = analyze(rows)

    # Check baseline if provided
    baseline_fail = False
    if args.baseline:
        try:
            with open(args.baseline) as f:
                baseline = json.load(f)
            metrics["baseline_status"] = baseline.get("status", "UNKNOWN")
            # If baseline says FAIL, our analysis of the same data should also say FAIL
            if baseline.get("status") == "FAIL":
                baseline_fail = True
        except Exception as e:
            print(f"Warning: could not load baseline: {e}", file=sys.stderr)

    verdict, reasons = check_verdict(metrics)

    # If baseline is FAIL and we're analyzing the same pre-fix data, verdict must be FAIL
    if baseline_fail and verdict == "PASS":
        verdict = "FAIL"
        reasons.append("baseline is FAIL but analysis returned PASS — regression detected")

    metrics["verdict"] = verdict
    metrics["verdict_reasons"] = reasons

    # Print summary
    print(f"Frames: {metrics['total_frames']}")
    print(f"Convergence: {metrics['convergence_ratio']}")
    print(f"Total P50/P95/P99: {metrics['total_ms_p50']}/{metrics['total_ms_p95']}/{metrics['total_ms_p99']} ms")
    print(f"Fitness P50/P95/max: {metrics['fitness_p50']}/{metrics['fitness_p95']}/{metrics['fitness_max']}")
    print(f"Prediction-only: {metrics['prediction_only_count']} ({metrics['prediction_only_ratio']})")
    print(f"Cropped target disasters: {metrics['cropped_target_disaster_count']}")
    print(f"Target sources: {metrics['target_source_counts']}")
    print(f"Verdict: {verdict}")
    if reasons:
        for r in reasons:
            print(f"  - {r}")

    if args.output:
        with open(args.output, "w") as f:
            json.dump(metrics, f, indent=2)
        print(f"\nResults saved to: {args.output}")

    return 0 if verdict == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
