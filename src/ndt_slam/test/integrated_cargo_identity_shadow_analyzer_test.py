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
        "shadow_association": "MATCHED",
        "shadow_identity": "VALIDATED",
        "formal_lock": 1,
        "shadow_geometry_valid_this_frame": 1,
        "bottom_valid": 1,
        "downstream_top_source": "COMPONENT_UNION",
        "shadow_thickness_valid": 1,
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


def group_row(
    stamp: float, mode: str, confirm: int, required: int = 4,
    identity: str = "UNKNOWN", association: str = "MATCHED",
) -> dict[str, object]:
    return {
        "stamp": stamp,
        "canonical_member_ids": "7|9",
        "matched_history_id": 3,
        "association_state": association,
        "raw_representative_xy_step": 0.70,
        "stable_anchor_xy_step": 0.10,
        "vertical_mode": mode,
        "association_reject_reason": "NONE",
        "lift_confirm_count": confirm,
        "lift_confirm_required": required,
        "identity_state": identity,
    }


def oracle_v2(role="positive_collision", **identity):
    phases = {}
    for name in ("pre_lift", "lifted", "safe_over", "terminal_lowering"):
        phases[name] = {"applicable": False, "reason": "not used by test"}
    phases["lifted"] = {
        "applicable": True,
        "start_stamp_sec": 9.0,
        "end_stamp_sec": 12.0,
    }
    return {
        "oracle_version": 2,
        "time_base": "bag_source_stamp",
        "analysis_phase": "lifted",
        "phases": phases,
        "role": role,
        **identity,
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

    def test_legacy_oracle_is_inconclusive_in_v5(self):
        result = self.analyze(trace_row(14, 1.2), {
            "role": "negative_safe_over",
            "window_start_stamp_sec": 9.0,
            "window_end_stamp_sec": 11.0,
            "true_member_sets": [[7, 9]],
        })
        self.assertEqual(result["oracle_status"], "ORACLE_INCONCLUSIVE")
        self.assertEqual(result["EARLIEST_ROOT"], "ORACLE_INCONCLUSIVE")

    def test_oracle_windows_are_half_open(self):
        oracle = oracle_v2(true_member_sets=[[7, 9]])
        rows = [{"pipeline_stamp": "9.0"}, {"pipeline_stamp": "12.0"}]
        self.assertEqual(len(ANALYZER.oracle_window(rows, oracle)), 1)

    def test_baseline_shadow_id_difference_is_not_wrong_lock(self):
        oracle = oracle_v2(
            "negative_safe_over", true_member_sets=[[7, 9]],
            wrong_member_sets=[[1]],
        )
        result = self.analyze(trace_row(14, 1.2), oracle)
        self.assertEqual(result["wrong_formal_lock_frames"], 0)
        self.assertEqual(result["identity_verdict"], "PASS")
        self.assertEqual(result["avoidance_cargo_side_verdict"], "PASS")

    def test_positive_collision_code17_is_not_false_warning(self):
        oracle = oracle_v2(true_member_sets=[[7, 9]])
        result = self.analyze(trace_row(17, 0.2), oracle)
        self.assertEqual(result["identity_verdict"], "PASS")
        self.assertEqual(result["avoidance_cargo_side_verdict"], "PASS")

    def test_group_top_without_bottom_reports_terminal_bottom_blocker(self):
        oracle = oracle_v2(true_member_sets=[[7, 9]])
        row = trace_row(33, 1.0)
        row["bottom_valid"] = 0
        row["shadow_geometry_valid_this_frame"] = 0
        result = self.analyze(row, oracle)
        self.assertEqual(
            result["downstream_bottom_blocker"],
            "POINTS_BOTTOM_OBSERVABILITY_BLOCKING",
        )

    def test_negative_bag_missing_identity_is_upstream_blocking(self):
        oracle = oracle_v2(
            "negative_safe_over", true_member_sets=[[7, 9]]
        )
        row = trace_row(33, 1.0)
        row["shadow_identity"] = "UNKNOWN"
        result = self.analyze(row, oracle)
        self.assertTrue(result["no_bag_upstream_identity_blocking"])

    def analyze_groups(self, rows, oracle, baseline_rows=None):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "groups.csv"
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            baseline_path = None
            if baseline_rows is not None:
                baseline_path = Path(directory) / "baseline.csv"
                with baseline_path.open(
                    "w", newline="", encoding="utf-8"
                ) as stream:
                    writer = csv.DictWriter(
                        stream, fieldnames=list(baseline_rows[0])
                    )
                    writer.writeheader()
                    writer.writerows(baseline_rows)
            return ANALYZER.analyze_group_trace(path, oracle, baseline_path)

    def test_group_trace_reports_rate_and_stable_anchor_improvement(self):
        oracle = oracle_v2(true_member_sets=[[7, 9]])
        rows = [
            group_row(10.0, "SUPPORTED_EVIDENCE", 1, association="NEW_HISTORY"),
            group_row(10.1, "SUPPORTED_EVIDENCE", 2),
        ]
        baseline = [
            group_row(10.0, "SUPPORTED_EVIDENCE", 0,
                      association="NEW_HISTORY"),
            group_row(10.1, "SUPPORTED_EVIDENCE", 0,
                      association="NEW_HISTORY"),
        ]
        result = self.analyze_groups(rows, oracle, baseline)
        self.assertEqual(result["new_history_rate_before"], 1.0)
        self.assertEqual(result["new_history_rate_after"], 0.5)
        self.assertGreater(
            result["raw_representative_xy_step_p95"],
            result["stable_anchor_xy_step_p95"],
        )

    def test_yes_bag_vertical_availability_blocking_is_terminal(self):
        oracle = oracle_v2(true_member_sets=[[7, 9]])
        rows = [
            group_row(10.0 + index * 0.1, "CONTINUITY_ONLY", 0)
            for index in range(4)
        ]
        result = self.analyze_groups(rows, oracle)
        self.assertEqual(
            result["yes_bag_exit_classification"],
            "VERTICAL_EVIDENCE_AVAILABILITY_BLOCKING",
        )

    def test_yes_bag_supported_sequence_without_validation_is_impl_fail(self):
        oracle = oracle_v2(true_member_sets=[[7, 9]])
        rows = [
            group_row(10.0 + index * 0.1, "SUPPORTED_EVIDENCE", index + 1)
            for index in range(4)
        ]
        result = self.analyze_groups(rows, oracle)
        self.assertEqual(
            result["yes_bag_exit_classification"],
            "IDENTITY_LIFT_IMPLEMENTATION_FAIL",
        )

    def test_yes_bag_missing_true_group_is_detector_blocking(self):
        oracle = oracle_v2(true_member_sets=[[7, 9]])
        rows = [group_row(10.0, "SUPPORTED_EVIDENCE", 0)]
        rows[0]["canonical_member_ids"] = "1"
        result = self.analyze_groups(rows, oracle)
        self.assertEqual(
            result["yes_bag_exit_classification"],
            "DETECTOR_AVAILABILITY_BLOCKING",
        )

    def test_report_root_matches_analysis_json(self):
        report = self.analyze(
            trace_row(14, 1.2),
            oracle_v2("negative_safe_over", true_member_sets=[[7, 9]]),
        )
        summary = {
            "V5_STABLE_SHA": "abc",
            "V5_IMPLEMENTATION_GATE": "PASS",
            "V5_FUNCTION_RESULT": "PASS",
            "V5_RUNTIME_REGRESSION": "PASS",
            "EARLIEST_REMAINING_BLOCKER": report["EARLIEST_ROOT"],
            "bags": [report],
        }
        markdown = ANALYZER.render_markdown(summary)
        self.assertIn(report["EARLIEST_ROOT"], markdown)
        self.assertIn(summary["EARLIEST_REMAINING_BLOCKER"], markdown)


if __name__ == "__main__":
    unittest.main()
