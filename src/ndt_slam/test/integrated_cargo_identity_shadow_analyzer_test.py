#!/usr/bin/env python3
"""Regression tests for bag-role/oracle-aware SHADOW verdicts."""

import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
ANALYZER_PATH = (
    PACKAGE / "scripts" / "analysis" /
    "analyze_integrated_cargo_identity_shadow.py"
)
SPEC = importlib.util.spec_from_file_location("integrated_shadow_analyzer", ANALYZER_PATH)
ANALYZER = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(ANALYZER)


def trace_row(code: int, clearance: float) -> dict[str, object]:
    return {
        "pipeline_stamp": 10.0,
        "baseline_selected_candidate_id": 1,
        "shadow_candidate_id": 2,
        "shadow_member_component_ids": "7|9",
        "shadow_identity": "VALIDATED",
        "formal_lock": 1,
        "shadow_geometry_valid_this_frame": 1,
        "bottom_valid": 1,
        "shadow_official_valid": 1,
        "shadow_code": code,
        "clearance": clearance,
        "identity_before_8m": 1,
        "ready_before_5m": 1,
        "canonical_far_history_valid": 0,
        "obstacle_self_contamination_blocking": 0,
        "shadow_total_compute_ms": 1.0,
        "pointcloud_callback_hz": 10.0,
        "ndt_processing_hz": 10.0,
        "dropped_frame_count": 0,
        "large_gap_count": 0,
    }


class IntegratedCargoIdentityShadowAnalyzerTest(unittest.TestCase):
    def analyze(self, row: dict[str, object], oracle=None):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.csv"
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(row))
                writer.writeheader()
                writer.writerow(row)
            return ANALYZER.analyze_trace("test", path, oracle)

    def test_missing_oracle_never_guesses_pass(self):
        result = self.analyze(trace_row(14, 1.2))
        self.assertEqual(result["oracle_status"], "ORACLE_INCONCLUSIVE")
        self.assertEqual(result["identity_verdict"], "ORACLE_INCONCLUSIVE")

    def test_baseline_shadow_id_difference_is_not_wrong_lock(self):
        oracle = {
            "role": "negative_safe_over",
            "window_start_stamp_sec": 9.0,
            "window_end_stamp_sec": 11.0,
            "true_member_sets": [[7, 9]],
            "wrong_member_sets": [[1]],
        }
        result = self.analyze(trace_row(14, 1.2), oracle)
        self.assertEqual(result["wrong_formal_lock_frames"], 0)
        self.assertEqual(result["identity_verdict"], "PASS")
        self.assertEqual(result["avoidance_cargo_side_verdict"], "PASS")

    def test_positive_collision_code17_is_not_false_warning(self):
        oracle = {
            "role": "positive_collision",
            "window_start_stamp_sec": 9.0,
            "window_end_stamp_sec": 11.0,
            "true_member_sets": [[7, 9]],
        }
        result = self.analyze(trace_row(17, 0.2), oracle)
        self.assertEqual(result["identity_verdict"], "PASS")
        self.assertEqual(result["avoidance_cargo_side_verdict"], "PASS")


if __name__ == "__main__":
    unittest.main()
