#!/usr/bin/env python3
"""Summarize authoritative cargo dimension samples from monitor output.

Reads ``samples/cargo_geometry_authoritative.jsonl`` from a run directory
and produces a dimension report with per-track median statistics,
rejection reasons, and default-size recommendations.

Usage:
    python3 summarize_cargo_dimensions.py <run_dir> [--output-dir <dir>]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


def load_samples(run_dir: Path) -> List[Dict[str, Any]]:
    path = run_dir / "samples" / "cargo_geometry_authoritative.jsonl"
    if not path.exists():
        print(f"No authoritative samples found at {path}", file=sys.stderr)
        return []
    samples = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            line = line.strip()
            if not line:
                continue
            try:
                samples.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return samples


def percentile(values: List[float], pct: float) -> Optional[float]:
    clean = sorted(v for v in values if math.isfinite(v))
    if not clean:
        return None
    if len(clean) == 1:
        return clean[0]
    pos = (len(clean) - 1) * pct
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return clean[lo]
    frac = pos - lo
    return clean[lo] * (1.0 - frac) + clean[hi] * frac


def summarize_dimensions(samples: List[Dict[str, Any]]) -> Dict[str, Any]:
    by_track: Dict[int, List[Dict[str, Any]]] = defaultdict(list)
    all_lengths: List[float] = []
    all_widths: List[float] = []
    all_heights: List[float] = []
    all_centers_x: List[float] = []
    all_centers_y: List[float] = []
    all_centers_z: List[float] = []
    all_bottom_z: List[float] = []
    all_top_z: List[float] = []
    source_counts: Counter[str] = Counter()
    track_stamps: Dict[int, Tuple[float, float]] = {}

    for sample in samples:
        tid = int(sample.get("track_id", 0))
        if tid <= 0:
            continue
        by_track[tid].append(sample)
        length = float(sample.get("length_m", 0))
        width = float(sample.get("width_m", 0))
        height = float(sample.get("height_m", 0))
        cx = float(sample.get("center_x", 0))
        cy = float(sample.get("center_y", 0))
        cz = float(sample.get("center_z", 0))
        bz = float(sample.get("bottom_z", 0))
        tz = float(sample.get("top_z", 0))
        stamp = float(sample.get("stamp", 0))

        if all(v > 0 and math.isfinite(v) for v in (length, width, height)):
            all_lengths.append(length)
            all_widths.append(width)
            all_heights.append(height)
            all_centers_x.append(cx)
            all_centers_y.append(cy)
            all_centers_z.append(cz)
            all_bottom_z.append(bz)
            all_top_z.append(tz)
            source_counts[str(sample.get("geometry_source", "UNKNOWN"))] += 1

            if tid not in track_stamps:
                track_stamps[tid] = (stamp, stamp)
            else:
                lo, hi = track_stamps[tid]
                track_stamps[tid] = (min(lo, stamp), max(hi, stamp))

    # Per-track median dimensions
    track_summaries = []
    for tid, items in sorted(by_track.items()):
        lo, hi = track_stamps.get(tid, (0, 0))
        track_summaries.append({
            "track_id": tid,
            "samples": len(items),
            "duration_sec": max(0.0, hi - lo),
            "length_m_p50": percentile([float(s["length_m"]) for s in items], 0.50),
            "width_m_p50": percentile([float(s["width_m"]) for s in items], 0.50),
            "height_m_p50": percentile([float(s["height_m"]) for s in items], 0.50),
            "center_x_p50": percentile([float(s["center_x"]) for s in items], 0.50),
            "center_y_p50": percentile([float(s["center_y"]) for s in items], 0.50),
            "center_z_p50": percentile([float(s["center_z"]) for s in items], 0.50),
            "bottom_z_p50": percentile([float(s.get("bottom_z", 0)) for s in items], 0.50),
        })

    unique_tracks = len(by_track)
    total_samples = len(all_lengths)

    result: Dict[str, Any] = {
        "total_authoritative_samples": len(samples),
        "filtered_valid_samples": total_samples,
        "unique_tracks": unique_tracks,
        "tracks": track_summaries,
        "length_m": {
            "p50": percentile(all_lengths, 0.50),
            "p75": percentile(all_lengths, 0.75),
            "p90": percentile(all_lengths, 0.90),
            "p95": percentile(all_lengths, 0.95),
            "p99": percentile(all_lengths, 0.99),
            "max": max(all_lengths) if all_lengths else None,
        },
        "width_m": {
            "p50": percentile(all_widths, 0.50),
            "p75": percentile(all_widths, 0.75),
            "p90": percentile(all_widths, 0.90),
            "p95": percentile(all_widths, 0.95),
            "p99": percentile(all_widths, 0.99),
            "max": max(all_widths) if all_widths else None,
        },
        "height_m": {
            "p50": percentile(all_heights, 0.50),
            "p75": percentile(all_heights, 0.75),
            "p90": percentile(all_heights, 0.90),
            "p95": percentile(all_heights, 0.95),
            "p99": percentile(all_heights, 0.99),
            "max": max(all_heights) if all_heights else None,
        },
        "center_offset_from_odom": {
            "x_p50": percentile(all_centers_x, 0.50),
            "y_p50": percentile(all_centers_y, 0.50),
            "z_p50": percentile(all_centers_z, 0.50),
        },
        "source_distribution": dict(source_counts.most_common()),
    }

    # Default size recommendation (only if enough tracks)
    if unique_tracks >= 30 and total_samples >= 100:
        result["default_dimension_recommendation"] = {
            "ready": True,
            "length_m": round(percentile(all_lengths, 0.95) or 4.0, 2),
            "width_m": round(percentile(all_widths, 0.95) or 1.6, 2),
            "height_m": round(percentile(all_heights, 0.95) or 2.0, 2),
            "note": "P95 across all authoritative tracks. Review before enabling fallback."
        }
    else:
        result["default_dimension_recommendation"] = {
            "ready": False,
            "reason": (f"Need ≥30 unique tracks (have {unique_tracks}) "
                       f"and ≥100 samples (have {total_samples})"),
            "note": "Continue collecting data before recommending default dimensions."
        }

    return result


def generate_markdown_report(summary: Dict[str, Any], output_path: Path) -> None:
    lines = [
        "# Cargo Dimension Summary",
        "",
        f"**Total authoritative samples:** {summary['total_authoritative_samples']}",
        f"**Filtered valid samples:** {summary['filtered_valid_samples']}",
        f"**Unique tracks:** {summary['unique_tracks']}",
        "",
        "## Dimension Distribution (all valid tracks)",
        "",
        "| Dimension | P50 | P75 | P90 | P95 | P99 | Max |",
        "|-----------|-----|-----|-----|-----|-----|-----|",
    ]
    for dim in ("length_m", "width_m", "height_m"):
        d = summary[dim]
        lines.append(
            f"| {dim.replace('_m','')} | {d['p50']} | {d['p75']} | "
            f"{d['p90']} | {d['p95']} | {d['p99']} | {d['max']} |"
        )

    lines.extend([
        "",
        "## Per-Track Medians",
        "",
        "| Track | Samples | Duration(s) | L(m) | W(m) | H(m) |",
        "|-------|---------|-------------|------|------|------|",
    ])
    for t in summary.get("tracks", []):
        lines.append(
            f"| {t['track_id']} | {t['samples']} | {t['duration_sec']:.0f} | "
            f"{t['length_m_p50']} | {t['width_m_p50']} | {t['height_m_p50']} |"
        )

    rec = summary.get("default_dimension_recommendation", {})
    lines.extend([
        "",
        "## Default Dimension Recommendation",
        "",
        f"**Ready:** {rec.get('ready', False)}",
    ])
    if rec.get("ready"):
        lines.append(f"**Recommended (P95):** L={rec['length_m']}m W={rec['width_m']}m H={rec['height_m']}m")
        lines.append(f"**Note:** {rec.get('note', '')}")
    else:
        lines.append(f"**Reason:** {rec.get('reason', 'insufficient data')}")
        lines.append(f"**Note:** {rec.get('note', '')}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path,
                        help="Monitor run directory (contains samples/)")
    parser.add_argument("--output-dir", type=Path, default=None,
                        help="Output directory (default: run_dir/reports)")
    args = parser.parse_args()
    run_dir = args.run_dir.expanduser().resolve()
    output_dir = (args.output_dir or run_dir / "reports").expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    samples = load_samples(run_dir)
    if not samples:
        print("No samples to analyze.", file=sys.stderr)
        return 1

    summary = summarize_dimensions(samples)
    json_path = output_dir / "cargo_dimension_summary.json"
    json_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
                         encoding="utf-8")
    print(f"Wrote {json_path}")

    md_path = output_dir / "cargo_dimension_summary.md"
    generate_markdown_report(summary, md_path)
    print(f"Wrote {md_path}")

    rec = summary.get("default_dimension_recommendation", {})
    print(f"\nTracks: {summary['unique_tracks']}, Samples: {summary['filtered_valid_samples']}")
    print(f"Recommendation ready: {rec.get('ready', False)}")
    if rec.get("ready"):
        print(f"  L={rec['length_m']}m W={rec['width_m']}m H={rec['height_m']}m")
    else:
        print(f"  {rec.get('reason', '')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
