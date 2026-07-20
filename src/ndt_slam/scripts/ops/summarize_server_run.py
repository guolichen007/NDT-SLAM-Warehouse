#!/usr/bin/env python3
"""Generate append-safe NDT-SLAM server-run JSON and Markdown reports."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parent))
from server_runtime_monitor import SafetyAggregator, atomic_write_json, _percentile  # noqa: E402


def _read_json(path: Path) -> Dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def _read_csv(path: Path) -> List[Dict[str, str]]:
    try:
        with path.open("r", encoding="utf-8", newline="") as stream:
            return list(csv.DictReader(stream))
    except OSError:
        return []


def _number(row: Mapping[str, Any], key: str) -> Optional[float]:
    try:
        value = float(row.get(key, ""))
        return value if math.isfinite(value) else None
    except (TypeError, ValueError):
        return None


def _first_last(rows: Sequence[Mapping[str, Any]], key: str) -> Dict[str, Any]:
    values = [_number(row, key) for row in rows]
    clean = [value for value in values if value is not None]
    return {"start": clean[0] if clean else None,
            "end": clean[-1] if clean else None,
            "delta": clean[-1] - clean[0] if len(clean) >= 2 else None}


def build_summary(run_dir: Path) -> Dict[str, Any]:
    manifest = _read_json(run_dir / "run_manifest.json")
    safety_rows = _read_csv(run_dir / "samples" / "safety_samples.csv")
    runtime_rows = _read_csv(run_dir / "samples" / "runtime_samples.csv")
    aggregator = SafetyAggregator((60.0, 600.0))
    for row in safety_rows:
        stamp = _number(row, "source_stamp")
        wall = _number(row, "wall_time")
        if stamp is None or wall is None:
            continue
        values: Dict[str, Any] = dict(row)
        for key in ("requested_alarm_code", "evidence_state", "cargo_track_id",
                    "obstacle_track_id", "obstacle_provenance_type"):
            try:
                values[key] = int(float(row.get(key, "0")))
            except ValueError:
                values[key] = 0
        for key in ("warning_valid", "obstacle_provenance_valid",
                    "obstacle_large_geometry_valid", "static_authorized"):
            values[key] = str(row.get(key, "")).lower() in ("1", "true", "yes")
        aggregator.ingest(values, source_stamp=stamp, wall_time=wall)
    end_time = _number(runtime_rows[-1], "wall_time") if runtime_rows else None
    if end_time is None and aggregator.records:
        end_time = aggregator.records[-1].wall_time
    end_time = end_time or time.time()
    safety = aggregator.full_summary(now=end_time)

    rss_values = [value for row in runtime_rows
                  if (value := _number(row, "rss_mb")) is not None]
    process_values = [value for row in runtime_rows
                      if (value := _number(row, "runtime_average_process_time_ms")) is not None]
    ndt_values = [value for row in runtime_rows
                  if (value := _number(row, "runtime_average_ndt_time_ms")) is not None]
    stale_count = sum(str(row.get("runtime_pointcloud_stale", "")).lower() in
                      ("1", "true", "yes") for row in runtime_rows)
    restart_count = max([int(float(row.get("restart_count", "0") or 0))
                         for row in runtime_rows] or [0])
    duration_sec = 0.0
    if runtime_rows:
        first = _number(runtime_rows[0], "wall_time")
        last = _number(runtime_rows[-1], "wall_time")
        if first is not None and last is not None:
            duration_sec = max(0.0, last - first)
    rss_growth_mb_hour = None
    if len(rss_values) >= 2 and duration_sec > 0.0:
        rss_growth_mb_hour = (rss_values[-1] - rss_values[0]) * 3600.0 / duration_sec

    checks = {
        "monitor_samples": "PASS" if runtime_rows else "FAIL",
        "typed_safety_samples": "PASS" if safety_rows else "FAIL",
        "ubuntu_clean_build": manifest.get("ubuntu_clean_build", "NOT_RUN"),
        "ubuntu_gtests": manifest.get("ubuntu_gtests", "NOT_RUN"),
        "bag_validation": manifest.get("bag_validation", "NOT_RUN"),
        "server_soak": manifest.get("server_soak", "NOT_RUN"),
    }
    summary: Dict[str, Any] = {
        "generated_at": time.time(),
        "run_id": manifest.get("run_id", run_dir.name),
        "exact_sha": manifest.get("actual_sha", manifest.get("expected_sha", "unknown")),
        "start_time": manifest.get("created_at"),
        "end_time": end_time,
        "duration_sec": duration_sec,
        "node_restart_count": restart_count,
        "safety": safety,
        "runtime": {
            "sample_count": len(runtime_rows),
            "rss_mb": {"p50": _percentile(rss_values, 0.50),
                       "p95": _percentile(rss_values, 0.95),
                       "max": max(rss_values) if rss_values else None,
                       "growth_mb_per_hour": rss_growth_mb_hour},
            "disk_free_gb": _first_last(runtime_rows, "disk_free_gb"),
            "persistent_size_mb": _first_last(runtime_rows, "persistent_size_mb"),
            "static_revision": _first_last(runtime_rows, "runtime_static_evidence_revision"),
            "static_cells": _first_last(runtime_rows, "runtime_static_evidence_cells"),
            "static_mature_cells": _first_last(
                runtime_rows, "runtime_static_evidence_mature_cells"),
            "manifest_state_counts": dict(Counter(
                row.get("manifest_state", "MISSING") for row in runtime_rows)),
            "manifest_revision": _first_last(runtime_rows, "manifest_revision"),
            "clean_build_started": _first_last(
                runtime_rows, "runtime_static_clean_build_started"),
            "clean_build_applied": _first_last(
                runtime_rows, "runtime_static_clean_build_applied"),
            "clean_build_snapshot_only": _first_last(
                runtime_rows, "runtime_static_clean_build_snapshot_only"),
            "clean_build_discarded": _first_last(
                runtime_rows, "runtime_static_clean_build_discarded"),
            "dirty_tiles": _first_last(runtime_rows, "runtime_dirty_tile_count"),
            "flushed_tiles": _first_last(runtime_rows, "runtime_flushed_tile_count"),
            "pointcloud_stale_samples": stale_count,
            "process_time_ms": {"p50": _percentile(process_values, 0.50),
                                "p95": _percentile(process_values, 0.95)},
            "ndt_time_ms": {"p50": _percentile(ndt_values, 0.50),
                            "p95": _percentile(ndt_values, 0.95)},
        },
        "checks": checks,
        "overall": "FAIL" if "FAIL" in checks.values() else (
            "NOT_RUN" if "NOT_RUN" in checks.values() else "PASS"),
    }
    return summary


def _ratio_table(summary: Mapping[str, Any]) -> str:
    run = summary.get("safety", {}).get("run", {})
    ratios = run.get("code_duration_ratio", {})
    counts = run.get("code_counts", {})
    lines = ["| Code | Samples | Time ratio |", "|---:|---:|---:|"]
    for code in (14, 17, 18, 30, 31, 32, 33, 34, 35):
        lines.append("| {} | {} | {:.2%} |".format(
            code, counts.get(str(code), 0), float(ratios.get(str(code), 0.0))))
    return "\n".join(lines)


def render_markdown(summary: Mapping[str, Any]) -> str:
    run = summary.get("safety", {}).get("run", {})
    runtime = summary.get("runtime", {})
    checks = summary.get("checks", {})
    lines = [
        "# NDT-SLAM Server Validation Report",
        "",
        "- Run: `{}`".format(summary.get("run_id", "unknown")),
        "- SHA: `{}`".format(summary.get("exact_sha", "unknown")),
        "- Duration: `{:.1f}s`".format(float(summary.get("duration_sec", 0.0))),
        "- Overall: **{}**".format(summary.get("overall", "NOT_RUN")),
        "- Node restarts: `{}`".format(summary.get("node_restart_count", 0)),
        "",
        "## Safety",
        "",
        _ratio_table(summary),
        "",
        "- Longest 33: `{:.3f}s`".format(float(run.get("longest_33_sec", 0.0))),
        "- Longest 34: `{:.3f}s`".format(float(run.get("longest_34_sec", 0.0))),
        "- Warning events: `{}`".format(run.get("warning_events", 0)),
        "- Track churn: `{:.3f}/min`".format(float(run.get("track_churn_per_min", 0.0))),
        "- Static authorization ratio: `{:.2%}`".format(float(run.get("static_authorized_ratio", 0.0))),
        "- Reasons: `{}`".format(json.dumps(run.get("reason_counts", {}), ensure_ascii=False)),
        "",
        "## Runtime",
        "",
        "- Samples: `{}`".format(runtime.get("sample_count", 0)),
        "- RSS: `{}`".format(json.dumps(runtime.get("rss_mb", {}), ensure_ascii=False)),
        "- Disk: `{}`".format(json.dumps(runtime.get("disk_free_gb", {}), ensure_ascii=False)),
        "- Persistent data: `{}`".format(json.dumps(runtime.get("persistent_size_mb", {}), ensure_ascii=False)),
        "- Static revision: `{}`".format(json.dumps(runtime.get("static_revision", {}), ensure_ascii=False)),
        "- Static cells: `{}`".format(json.dumps(runtime.get("static_cells", {}), ensure_ascii=False)),
        "- Static mature cells: `{}`".format(json.dumps(runtime.get("static_mature_cells", {}), ensure_ascii=False)),
        "- Manifest states: `{}`".format(json.dumps(runtime.get("manifest_state_counts", {}), ensure_ascii=False)),
        "- Clean worker applied/snapshot/discarded: `{}` / `{}` / `{}`".format(
            json.dumps(runtime.get("clean_build_applied", {}), ensure_ascii=False),
            json.dumps(runtime.get("clean_build_snapshot_only", {}), ensure_ascii=False),
            json.dumps(runtime.get("clean_build_discarded", {}), ensure_ascii=False)),
        "- Pointcloud stale samples: `{}`".format(runtime.get("pointcloud_stale_samples", 0)),
        "",
        "## Acceptance status",
        "",
    ]
    lines.extend("- {}: **{}**".format(key, value) for key, value in checks.items())
    lines.extend(["", "> NOT_RUN is never interpreted as PASS.", ""])
    return "\n".join(lines)


def summarize(run_dir: Path) -> Dict[str, Any]:
    summary = build_summary(run_dir)
    reports = run_dir / "reports"
    reports.mkdir(parents=True, exist_ok=True)
    atomic_write_json(reports / "final_summary.json", summary)
    temporary = reports / "final_report.md.tmp"
    with temporary.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(render_markdown(summary))
    temporary.replace(reports / "final_report.md")
    return summary


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir")
    args = parser.parse_args(argv)
    summary = summarize(Path(args.run_dir).expanduser().resolve())
    print(json.dumps({"run_id": summary["run_id"], "overall": summary["overall"]},
                     indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
