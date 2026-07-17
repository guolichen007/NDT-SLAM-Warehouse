#!/usr/bin/env python3
"""Summarize cargo/static diagnostic CSVs without changing acceptance state."""

import argparse
import collections
import csv
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("cargo_csv", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    codes = collections.Counter()
    reasons = collections.Counter()
    query_reasons = collections.Counter()
    track_ids = set()
    rows = 0
    last = {}
    with args.cargo_csv.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            rows += 1
            last = row
            codes[row.get("requested_alarm_code", "unknown")] += 1
            reasons[row.get("safety_reason", "unknown")] += 1
            query_reasons[row.get("static_query_reason", "unknown")] += 1
            track_id = row.get("obstacle_track_id", "0")
            if track_id not in ("", "0"):
                track_ids.add(track_id)

    summary = {
        "rows": rows,
        "codes": dict(codes),
        "safety_reasons": dict(reasons),
        "static_query_reasons": dict(query_reasons),
        "unique_obstacle_tracks": len(track_ids),
        "last_static_index_revision": last.get("static_index_revision", "0"),
        "last_static_index_cells": last.get("static_index_cell_count", "0"),
        "last_track_created_count": last.get("obstacle_track_created_count", "0"),
        "last_track_reset_count": last.get("obstacle_track_reset_count", "0"),
    }
    rendered = json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    else:
        print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
