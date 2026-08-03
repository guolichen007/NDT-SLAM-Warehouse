import csv
import json
import os
import sys
import tempfile
import threading
import unittest
from pathlib import Path

OPS = Path(__file__).resolve().parents[1] / "src" / "ndt_slam" / "scripts" / "ops"
sys.path.insert(0, str(OPS))

from server_runtime_monitor import (  # noqa: E402
    AsyncRunWriter,
    AvoidancePipelineObserver,
    CargoMonitorGate,
    RosRuntimeMonitor,
    SafetyAggregator,
    append_csv,
    atomic_write_json,
    bounded_setdefault,
    is_runtime_status_stale,
    normalize_timeout_status,
    prune_run_directories,
    classify_cargo_geometry,
    cargo_monitor_ready,
    select_static_status,
)
from summarize_server_run import summarize  # noqa: E402


def sample(code, reason="ok", track=0, **extra):
    value = {
        "requested_alarm_code": code,
        "reason": reason,
        "obstacle_track_id": track,
        "obstacle_provenance_valid": True,
        "obstacle_large_geometry_valid": True,
    }
    value.update(extra)
    return value


class SafetyAggregatorTest(unittest.TestCase):
    def test_sliding_window_code_duration(self):
        aggregate = SafetyAggregator((4,))
        aggregate.ingest(sample(34), source_stamp=1, wall_time=0)
        aggregate.ingest(sample(14), source_stamp=2, wall_time=2)
        summary = aggregate.summarize(now=4, window=4)
        self.assertAlmostEqual(summary["code_duration_sec"]["34"], 2.0)
        self.assertAlmostEqual(summary["code_duration_sec"]["14"], 2.0)

    def test_code_and_reason_transitions(self):
        aggregate = SafetyAggregator()
        first = aggregate.ingest(sample(14, "clear"), source_stamp=1, wall_time=1)
        reason = aggregate.ingest(sample(14, "new_reason"), source_stamp=2, wall_time=2)
        warning = aggregate.ingest(sample(17, "hazard"), source_stamp=3, wall_time=3)
        self.assertEqual(first[0]["event"], "SAFETY_ENTER")
        self.assertEqual(reason[0]["event"], "SAFETY_REASON_CHANGE")
        self.assertEqual(warning[0]["event"], "SAFETY_WARNING_ENTER")

    def test_fault_to_hazard_emits_clear_and_warning_edges(self):
        aggregate = SafetyAggregator()
        aggregate.ingest(sample(30, "hook_not_loaded"),
                         source_stamp=1, wall_time=1)
        events = aggregate.ingest(sample(17, "hazard"),
                                  source_stamp=2, wall_time=2)
        self.assertEqual(
            [event["event"] for event in events],
            ["SAFETY_FAULT_CLEAR", "SAFETY_WARNING_ENTER"])

    def test_current_summary_is_detached_and_uses_record_field_names(self):
        aggregate = SafetyAggregator()
        aggregate.ingest(
            sample(17, nearest_obstacle_distance_m=0.4,
                   conservative_vertical_clearance_m=-0.2),
            source_stamp=1, wall_time=1)
        current = aggregate.current_summary()
        self.assertEqual(current["nearest_distance_m"], 0.4)
        self.assertEqual(current["vertical_clearance_m"], -0.2)
        current["code"] = 99
        self.assertEqual(aggregate.records[-1].code, 17)

    def test_longest_continuous_33_and_34(self):
        aggregate = SafetyAggregator()
        aggregate.ingest(sample(33), source_stamp=1, wall_time=0)
        aggregate.ingest(sample(14), source_stamp=2, wall_time=3)
        aggregate.ingest(sample(34), source_stamp=3, wall_time=4)
        aggregate.ingest(sample(14), source_stamp=4, wall_time=9)
        summary = aggregate.summarize(now=10)
        self.assertEqual(summary["longest_33_sec"], 3)
        self.assertEqual(summary["longest_34_sec"], 5)

    def test_34_recovery_and_warning_confirmation(self):
        aggregate = SafetyAggregator()
        aggregate.ingest(sample(34), source_stamp=1, wall_time=0)
        aggregate.ingest(sample(14), source_stamp=2, wall_time=2)
        aggregate.ingest(sample(34), source_stamp=3, wall_time=4)
        aggregate.ingest(sample(18), source_stamp=4, wall_time=7)
        summary = aggregate.summarize(now=8)
        self.assertEqual(summary["recovery_34_to_14_sec"], [2])
        self.assertEqual(summary["confirmation_34_to_warning_sec"], [3])

    def test_track_churn_per_minute(self):
        aggregate = SafetyAggregator()
        aggregate.ingest(sample(14, track=1), source_stamp=1, wall_time=0)
        aggregate.ingest(sample(14, track=2), source_stamp=2, wall_time=30)
        aggregate.ingest(sample(14, track=0), source_stamp=3, wall_time=60)
        summary = aggregate.summarize(now=60)
        self.assertGreaterEqual(summary["track_churn_per_min"], 2.0)
        self.assertEqual(summary["unique_obstacle_tracks"], 2)

    def test_repeated_timestamp_is_not_new_evidence(self):
        aggregate = SafetyAggregator()
        aggregate.ingest(sample(34), source_stamp=10, wall_time=1)
        aggregate.ingest(sample(14), source_stamp=10, wall_time=2)
        self.assertEqual(len(aggregate.records), 1)
        self.assertEqual(aggregate.duplicate_stamps, 1)
        self.assertEqual(aggregate.records[-1].code, 34)

    def test_time_rollback_starts_new_monitor_epoch(self):
        aggregate = SafetyAggregator()
        aggregate.ingest(sample(34), source_stamp=100, wall_time=1)
        events = aggregate.ingest(sample(14), source_stamp=1, wall_time=2)
        self.assertEqual(len(aggregate.records), 2)
        self.assertEqual(aggregate.time_rollbacks, 1)
        self.assertEqual(events[0]["event"], "SOURCE_TIME_ROLLBACK")

    def test_duplicate_status_code_mismatch_is_suppressed(self):
        aggregate = SafetyAggregator()
        aggregate.ingest(sample(34), source_stamp=1, wall_time=1)
        self.assertIsNotNone(aggregate.check_status_code(17, wall_time=1.1))
        self.assertIsNone(aggregate.check_status_code(17, wall_time=1.2))
        self.assertEqual(aggregate.status_code_mismatches, 1)


class TypedCargoCallbackTimeTest(unittest.TestCase):
    @staticmethod
    def recognition_monitor():
        monitor = RosRuntimeMonitor.__new__(RosRuntimeMonitor)
        monitor._last_recognition_source_stamp = 10.0
        monitor._recognition_duplicate_count = 0
        monitor._recognition_rollback_count = 0
        monitor._recognition_epoch = 2
        monitor._last_recognition_state = 4
        monitor._recognition_failed_since = 9.0
        monitor._recognition_stale_active = True
        monitor.events = []
        monitor._emit_typed_event = lambda path, event: monitor.events.append(
            (path, event))
        return monitor

    @staticmethod
    def swing_monitor():
        monitor = RosRuntimeMonitor.__new__(RosRuntimeMonitor)
        monitor._last_swing_source_stamp = 10.0
        monitor._swing_duplicate_count = 0
        monitor._swing_rollback_count = 0
        monitor._swing_epoch = 3
        monitor._last_sway_state = 2
        monitor._last_skew_state = 2
        monitor._last_torsion_state = 1
        monitor._last_swing_alarm_inhibited = True
        monitor._swing_stale_active = True
        monitor.events = []
        monitor._emit_typed_event = lambda path, event: monitor.events.append(
            (path, event))
        return monitor

    def test_recognition_duplicate_is_not_new_evidence(self):
        monitor = self.recognition_monitor()
        self.assertFalse(monitor._accept_recognition_source_stamp(10.0, 20.0))
        self.assertEqual(monitor._recognition_duplicate_count, 1)
        self.assertEqual(monitor._recognition_epoch, 2)
        self.assertEqual(monitor.events, [])

    def test_recognition_rollback_starts_new_epoch(self):
        monitor = self.recognition_monitor()
        self.assertTrue(monitor._accept_recognition_source_stamp(1.0, 20.0))
        self.assertEqual(monitor._recognition_rollback_count, 1)
        self.assertEqual(monitor._recognition_epoch, 3)
        self.assertIsNone(monitor._last_recognition_state)
        self.assertIsNone(monitor._recognition_failed_since)
        self.assertFalse(monitor._recognition_stale_active)
        self.assertEqual(monitor.events[0][1]["event"],
                         "CARGO_RECOGNITION_TIME_ROLLBACK")

    def test_swing_duplicate_is_not_new_evidence(self):
        monitor = self.swing_monitor()
        self.assertFalse(monitor._accept_swing_source_stamp(10.0, 20.0))
        self.assertEqual(monitor._swing_duplicate_count, 1)
        self.assertEqual(monitor._swing_epoch, 3)
        self.assertEqual(monitor.events, [])

    def test_swing_rollback_resets_state_baselines(self):
        monitor = self.swing_monitor()
        self.assertTrue(monitor._accept_swing_source_stamp(1.0, 20.0))
        self.assertEqual(monitor._swing_rollback_count, 1)
        self.assertEqual(monitor._swing_epoch, 4)
        self.assertIsNone(monitor._last_sway_state)
        self.assertIsNone(monitor._last_skew_state)
        self.assertIsNone(monitor._last_torsion_state)
        self.assertFalse(monitor._last_swing_alarm_inhibited)
        self.assertFalse(monitor._swing_stale_active)
        self.assertEqual(monitor.events[0][1]["event"],
                         "CARGO_SWING_TIME_ROLLBACK")


class CargoMonitorGateTest(unittest.TestCase):
    def test_loaded_starts_episode_and_stale_closes_it_once(self):
        gate = CargoMonitorGate()
        empty = gate.update(valid=True, fresh=True, state=2,
                            wall_time=1.0)
        self.assertFalse(gate.active)
        self.assertEqual(empty[0]["state"], "EMPTY")
        loaded = gate.update(valid=True, fresh=True, state=3,
                             wall_time=2.0)
        self.assertTrue(gate.active)
        self.assertEqual(loaded[0]["event"], "GRAVITY_LOADED")
        self.assertEqual(gate.episode_id, 1)
        lost = gate.update(valid=False, fresh=False, state=0,
                           wall_time=4.0)
        self.assertFalse(gate.active)
        self.assertEqual(lost[0]["event"], "GRAVITY_LOST_DURING_CARGO")
        self.assertEqual(lost[0]["duration_sec"], 2.0)
        self.assertEqual(gate.update(valid=False, fresh=False, state=0,
                                     wall_time=5.0), [])

    def test_no_gravity_never_creates_episode(self):
        gate = CargoMonitorGate()
        gate.update(valid=False, fresh=False, state=0, wall_time=1.0)
        self.assertFalse(gate.active)
        self.assertEqual(gate.episode_id, 0)


class AvoidancePipelineObserverTest(unittest.TestCase):
    def test_mismatch_uses_grace_window(self):
        observer = AvoidancePipelineObserver(0.4)
        observer.set_code("raw_typed", 34, 1.0)
        observer.set_code("raw_simple", 17, 1.0)
        self.assertFalse(observer.snapshot(1.3)[
            "raw_typed_simple_mismatch"])
        self.assertTrue(observer.snapshot(1.5)[
            "raw_typed_simple_mismatch"])

    def test_pending_clear_and_normal_35_are_p0_flags(self):
        observer = AvoidancePipelineObserver()
        observer.set_pending({"pending_provisional_status": 14}, 1.0)
        observer.set_code("final_typed", 35, 1.0)
        snapshot = observer.snapshot(2.0)
        self.assertTrue(snapshot["pending_illegal_clear"])
        self.assertTrue(snapshot["normal_code35"])

    def test_reset_drops_previous_episode_mirrors(self):
        observer = AvoidancePipelineObserver()
        observer.set_code("final_typed", 14, 1.0)
        observer.set_operational({"operational_code": 14}, 1.0)
        observer.set_pending({"pending_provisional_status": "NEAR_3M"}, 1.0)
        observer.reset()
        self.assertEqual(observer.values, {})
        self.assertEqual(observer.operational, {})
        self.assertEqual(observer.pending, {})


class CargoGeometryClassificationTest(unittest.TestCase):
    def test_frozen_formal_geometry_is_not_direct_measured(self):
        direct, formal = classify_cargo_geometry({
            "track_state": "LOCKED", "lock_state": "LOST_HOLD",
            "geometry_source": "DIRECT_TOP_FROZEN_THICKNESS",
            "vertical_source": "DIRECT_TOP_FROZEN_THICKNESS",
            "frozen": True, "height_valid": True,
            "length_m": 2.0, "width_m": 1.0, "height_m": 0.5,
            "bottom_z": 0.4, "top_z": 0.9,
            "support": 0, "points": 0, "confidence": 0.8,
        })
        self.assertFalse(direct)
        self.assertTrue(formal)

    def test_direct_geometry_requires_current_point_support(self):
        value = {
            "track_state": "LOCKED", "lock_state": "LOCKED",
            "geometry_source": "MEASURED", "authoritative": True,
            "observation_valid": True, "height_valid": True,
            "length_m": 2.0, "width_m": 1.0, "height_m": 0.5,
            "bottom_z": 0.4, "top_z": 0.9,
            "support": 4, "points": 20, "confidence": 0.8,
        }
        direct, formal = classify_cargo_geometry(value)
        self.assertTrue(direct)
        self.assertTrue(formal)
        value["support"] = 0
        direct, formal = classify_cargo_geometry(value)
        self.assertFalse(direct)
        self.assertFalse(formal)


class CargoMonitorReadinessTest(unittest.TestCase):
    def test_inactive_monitor_is_not_cargo_ready(self):
        self.assertFalse(cargo_monitor_ready(False, True, True, True, True))

    def test_active_monitor_requires_all_current_episode_sources(self):
        self.assertTrue(cargo_monitor_ready(True, True, True, True, True))
        for missing in range(4):
            values = [True, True, True, True]
            values[missing] = False
            self.assertFalse(cargo_monitor_ready(True, *values))


class CargoTerminalFormattingTest(unittest.TestCase):
    def test_terminal_uses_snapshot_without_aggregator_method_lookup(self):
        monitor = RosRuntimeMonitor.__new__(RosRuntimeMonitor)
        monitor._state_lock = threading.RLock()
        monitor.latest_gravity = {"wall_time": 9.0}
        monitor.latest_geometry = {}
        monitor.latest_recognition = {}
        monitor.latest_swing = {}
        monitor.cargo_gate = CargoMonitorGate()
        monitor.cargo_gate.update(
            valid=True, fresh=True, state=3, wall_time=9.0)
        line = monitor._cargo_terminal_line(
            10.0,
            {"raw_typed_code": 33, "final_typed_code": 33,
             "heartbeat_code": 33},
            {"nearest_distance_m": 0.4,
             "vertical_clearance_m": -0.2,
             "obstacle_track_id": 7,
             "provenance": 3,
             "reason": "pending_not_authorized"})
        self.assertIn("distance=0.4", line)
        self.assertIn("obstacle_track=7", line)
        self.assertIn("reason=pending_not_authorized", line)


class StaticStatusSelectionTest(unittest.TestCase):
    def test_operational_precedes_static_and_pending(self):
        result = select_static_status(
            10.0,
            {"_wall_time": 9.5, "map_session_verified": True},
            {"_wall_time": 9.7, "map_session_verified": False,
             "authorized": True},
            {"_wall_time": 9.9, "map_session_verified": False,
             "static_authority": "PENDING"},
            2.0)
        self.assertTrue(result["map_session_verified"])
        self.assertEqual(result["static_authority"], "AUTHORIZED")

    def test_stale_pending_cannot_keep_session_ready(self):
        result = select_static_status(
            10.0, {}, {},
            {"_wall_time": 7.0, "map_session_verified": True,
             "static_authority": "RUNTIME_MATURE"},
            2.0)
        self.assertFalse(result["map_session_verified"])
        self.assertEqual(result["static_authority"], "UNKNOWN")


class AppendSafeOutputTest(unittest.TestCase):
    def test_all_append_only_outputs_rotate(self):
        with tempfile.TemporaryDirectory() as temporary:
            run_dir = Path(temporary)
            writer = AsyncRunWriter(
                run_dir, max_queue=16,
                rotation_size_mb=0.00001, rotation_count=2)
            writer.submit("jsonl", (
                "samples/events.jsonl", {"payload": "first-record"}))
            writer.submit("jsonl", (
                "samples/events.jsonl", {"payload": "second-record"}))
            writer.close()
            current = run_dir / "samples/events.jsonl"
            self.assertTrue(current.is_file())
            self.assertTrue(current.with_name("events.jsonl.1").is_file())

    def test_run_retention_removes_only_old_manifest_runs(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "server_runs"
            root.mkdir()
            runs = []
            for index in range(4):
                run = root / "run-{}".format(index)
                atomic_write_json(run / "run_manifest.json", {"run": index})
                os.utime(run, (100 + index, 100 + index))
                runs.append(run)
            unrelated = root / "operator-notes"
            unrelated.mkdir()
            removed = prune_run_directories(
                root, keep_runs=2, protected_run=runs[-1])
            self.assertEqual({path.name for path in removed},
                             {"run-0", "run-1"})
            self.assertTrue(runs[-1].is_dir())
            self.assertTrue(unrelated.is_dir())

    def test_keyed_statistics_are_bounded(self):
        values = {}
        for index in range(5):
            _, evicted = bounded_setdefault(
                values, str(index), {"index": index}, max_entries=3)
        self.assertTrue(evicted)
        self.assertEqual(list(values), ["2", "3", "4"])

    def test_atomic_live_summary(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "live_summary.json"
            atomic_write_json(path, {"generation": 1})
            atomic_write_json(path, {"generation": 2})
            self.assertEqual(json.loads(path.read_text())["generation"], 2)
            self.assertFalse(path.with_name(path.name + ".tmp").exists())

    def test_csv_header_is_not_overwritten_on_restart(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "samples.csv"
            append_csv(path, ["stamp", "code"], {"stamp": 1, "code": 34})
            append_csv(path, ["stamp", "code"], {"stamp": 2, "code": 14})
            with path.open(newline="") as stream:
                rows = list(csv.reader(stream))
            self.assertEqual(rows[0], ["stamp", "code"])
            self.assertEqual(len(rows), 3)

    def test_timeout_124_is_normal_completion(self):
        self.assertEqual(normalize_timeout_status(124), 0)
        self.assertEqual(normalize_timeout_status(7), 7)

    def test_stale_runtime_status(self):
        self.assertTrue(is_runtime_status_stale(None, 10, 5))
        self.assertTrue(is_runtime_status_stale(1, 10, 5))
        self.assertFalse(is_runtime_status_stale(8, 10, 5))

    def test_final_report_preserves_not_run(self):
        with tempfile.TemporaryDirectory() as temporary:
            run_dir = Path(temporary)
            atomic_write_json(run_dir / "run_manifest.json", {
                "run_id": "offline-test", "expected_sha": "abc",
                "ubuntu_clean_build": "NOT_RUN", "ubuntu_gtests": "NOT_RUN",
                "bag_validation": "NOT_RUN", "server_soak": "NOT_RUN"})
            append_csv(run_dir / "samples/safety_samples.csv",
                       ["wall_time", "source_stamp", "requested_alarm_code", "reason"],
                       {"wall_time": 1, "source_stamp": 1,
                        "requested_alarm_code": 14, "reason": "clear"})
            append_csv(run_dir / "samples/runtime_samples.csv",
                       ["wall_time", "rss_mb", "disk_free_gb"],
                       {"wall_time": 1, "rss_mb": 100, "disk_free_gb": 50})
            summary = summarize(run_dir)
            self.assertEqual(summary["overall"], "NOT_RUN")
            self.assertTrue((run_dir / "reports/final_report.md").is_file())


if __name__ == "__main__":
    unittest.main()
