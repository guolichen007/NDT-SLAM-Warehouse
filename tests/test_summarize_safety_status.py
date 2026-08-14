from __future__ import annotations

import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/acceptance/summarize_safety_status.py"
SPEC = importlib.util.spec_from_file_location("summarize_safety_status", SCRIPT)
SUMMARY = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(SUMMARY)


FIELDS = (
    "stamp",
    "requested_alarm_code",
    "nearest_cluster_distance",
    "conservative_clearance_m",
    "obstacle_track_id",
)


def write_rows(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


class SummarizeSafetyStatusTest(unittest.TestCase):
    def test_valid_warning_bands_and_two_frame_review_pass(self):
        rows = [
            {"stamp": 1.0, "requested_alarm_code": 34,
             "nearest_cluster_distance": 4.0,
             "conservative_clearance_m": 0.5,
             "obstacle_track_id": 7},
            {"stamp": 1.1, "requested_alarm_code": 29,
             "nearest_cluster_distance": 4.0,
             "conservative_clearance_m": 0.5,
             "obstacle_track_id": 7},
            {"stamp": 2.0, "requested_alarm_code": 18,
             "nearest_cluster_distance": 4.0,
             "conservative_clearance_m": 0.5,
             "obstacle_track_id": 8},
            {"stamp": 2.1, "requested_alarm_code": 17,
             "nearest_cluster_distance": 2.5,
             "conservative_clearance_m": 0.5,
             "obstacle_track_id": 8},
        ]
        self.assertEqual(SUMMARY.validate_avoidance_rows(rows), [])

    def test_clearance_and_distance_bands_are_checked(self):
        rows = [{"stamp": 1.0, "requested_alarm_code": 18,
                 "nearest_cluster_distance": 2.0,
                 "conservative_clearance_m": 0.8,
                 "obstacle_track_id": 9}]
        violations = SUMMARY.validate_avoidance_rows(rows)
        self.assertTrue(any("clearance_gate" in item for item in violations))
        self.assertTrue(any("code18_distance" in item for item in violations))

    def test_code29_requires_two_distinct_source_stamps(self):
        rows = [{"stamp": 3.0, "requested_alarm_code": 29,
                 "nearest_cluster_distance": 4.0,
                 "conservative_clearance_m": 0.5,
                 "obstacle_track_id": 10}]
        violations = SUMMARY.validate_avoidance_rows(rows)
        self.assertTrue(any("code29_distinct_frames" in item
                            for item in violations))

    def test_csv_loader_rejects_missing_contract_columns(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "cargo.csv"
            path.write_text("stamp,requested_alarm_code\n1,14\n",
                            encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "missing columns"):
                SUMMARY.load_cargo_rows(path, require_avoidance_fields=True)


if __name__ == "__main__":
    unittest.main()
