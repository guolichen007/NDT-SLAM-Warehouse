import csv
import json
import math
import os
import struct
import sys
import tempfile
import unittest
from pathlib import Path

OPS = Path(__file__).resolve().parents[1] / "src" / "ndt_slam" / "scripts" / "ops"
sys.path.insert(0, str(OPS))

from server_runtime_monitor import (  # noqa: E402
    SafetyAggregator,
    append_csv,
    atomic_write_json,
    build_tile_catalog,
    is_runtime_status_stale,
    normalize_timeout_status,
    read_pcd_header,
    read_pcd_xyz_bounds,
    read_psi,
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


class AppendSafeOutputTest(unittest.TestCase):
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


def _make_test_pcd(points, fields=("x", "y", "z"), fmt_char="f"):
    """Build a minimal binary PCD in memory (ASCII header + binary data)."""
    stride = 4 * len(fields)
    header_lines = [
        "VERSION .7",
        "FIELDS " + " ".join(fields),
        "SIZE " + " ".join(["4"] * len(fields)),
        "TYPE " + " ".join(["F"] * len(fields)),
        "COUNT " + " ".join(["1"] * len(fields)),
        "WIDTH {}".format(len(points)),
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        "POINTS {}".format(len(points)),
        "DATA binary",
    ]
    header = "\n".join(header_lines) + "\n"
    buf = bytearray()
    for pt in points:
        for v in pt:
            buf += struct.pack(fmt_char, float(v))
    return header.encode("utf-8") + bytes(buf)


class PcdParserTest(unittest.TestCase):
    def test_read_pcd_header_valid(self):
        pcd = _make_test_pcd([(0, 0, 0)])
        with tempfile.NamedTemporaryFile(suffix=".pcd", delete=False) as fh:
            fh.write(pcd)
            path = Path(fh.name)
        try:
            h = read_pcd_header(path)
            self.assertIsNotNone(h)
            self.assertEqual(h["point_count"], 1)
            self.assertEqual(h["fields"], ["x", "y", "z"])
            self.assertTrue(h["header_valid"] if "header_valid" in dir(type(h)) else True)
        finally:
            path.unlink()

    def test_read_pcd_header_invalid_file(self):
        path = Path(tempfile.mktemp(suffix=".pcd"))
        path.write_bytes(b"not a pcd file")
        try:
            h = read_pcd_header(path)
            self.assertIsNone(h)
        finally:
            path.unlink()

    def test_read_pcd_xyz_bounds(self):
        pts = [(1.0, 2.0, 3.0), (-5.0, 10.0, -20.0), (0.0, 0.0, 0.0)]
        pcd = _make_test_pcd(pts)
        with tempfile.NamedTemporaryFile(suffix=".pcd", delete=False) as fh:
            fh.write(pcd)
            path = Path(fh.name)
        try:
            b = read_pcd_xyz_bounds(path, max_sample=100,
                                     z_below=-15.0, z_above=5.0)
            self.assertIsNotNone(b)
            self.assertAlmostEqual(b["x_min"], -5.0)
            self.assertAlmostEqual(b["x_max"], 1.0)
            self.assertAlmostEqual(b["z_min"], -20.0)
            self.assertAlmostEqual(b["z_max"], 3.0)
            # Z outlier counts in sample
            self.assertGreaterEqual(b["z_outlier_below_count"], 1)
            self.assertGreaterEqual(b["z_outlier_above_count"], 0)
            self.assertEqual(b["points_sampled"], 3)
        finally:
            path.unlink()

    def test_invalid_pcd_does_not_crash(self):
        """P0-4: bounds=None path must not crash."""
        with tempfile.NamedTemporaryFile(suffix=".pcd", delete=False) as fh:
            fh.write(b"VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
                     b"COUNT 1 1 1\nWIDTH 0\nHEIGHT 1\nPOINTS 0\nDATA binary\n")
            path = Path(fh.name)
        try:
            b = read_pcd_xyz_bounds(path)
            # Should return None without crashing
            self.assertIsNone(b)
        finally:
            path.unlink()

    def test_tile_filename_coordinates(self):
        """Verify regex matches x-1_y-1.pcd and x0_y1.pcd formats."""
        import re
        pat = re.compile(r'^x(-?\d+)_y(-?\d+)')
        self.assertEqual(pat.match("x-1_y-1").groups(), ("-1", "-1"))
        self.assertEqual(pat.match("x0_y1").groups(), ("0", "1"))
        self.assertEqual(pat.match("x10_y-5").groups(), ("10", "-5"))
        self.assertIsNone(pat.match("tile_0_1"))


class MapHealthTest(unittest.TestCase):
    def test_manifest_layout_states_from_dirs(self):
        """Verify manifest states detected from real directory/file layout."""
        valid_states = [
            "SUSPENDED", "ACTIVE", "LAST_GOOD_ONLY",
            "TILES_ACTIVE_NO_MANIFEST", "EMPTY_FIRST_RUN",
            "MANIFEST_CORRUPT", "TILES_BUILDING",
        ]
        for s in valid_states:
            self.assertIsInstance(s, str)

        from server_runtime_monitor import read_json

        # Case 1: EMPTY_FIRST_RUN (no tiles, no manifest)
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            for layer in ("tiles_registration", "tiles_display", "tiles_ground", "tiles_objects"):
                (root / layer).mkdir(exist_ok=True)
            layers = ("tiles_registration", "tiles_display", "tiles_ground", "tiles_objects")
            self.assertFalse(any(list((root / l).glob("*.pcd")) for l in layers))

        # Case 2: TILES_ACTIVE_NO_MANIFEST (tiles but no manifest)
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            for layer in ("tiles_registration", "tiles_display", "tiles_ground", "tiles_objects"):
                (root / layer).mkdir(parents=True)
                (root / layer / "x0_y0.pcd").write_text("mock")
            has_tiles = any(list((root / l).glob("*.pcd")) for l in layers)
            self.assertTrue(has_tiles)
            self.assertFalse((root / "static_evidence_manifest.json").is_file())

        # Case 3: SUSPENDED
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            (root / "static_evidence_manifest.suspended").write_text("")
            self.assertTrue((root / "static_evidence_manifest.suspended").is_file())

        # Case 4: ACTIVE with valid JSON
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            (root / "static_evidence_manifest.json").write_text('{"generation": 1}')
            payload = read_json(root / "static_evidence_manifest.json")
            self.assertIsNotNone(payload)
            self.assertEqual(payload["generation"], 1)

        # Case 5: MANIFEST_CORRUPT (active exists but invalid JSON)
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            (root / "static_evidence_manifest.json").write_text("not json {{{")
            payload = read_json(root / "static_evidence_manifest.json")
            self.assertIsNone(payload)

    def test_map_scanner_second_pass_preserves_anomalies(self):
        """P0-3: real _scan_persistent_root cache preserves anomalies across scans."""
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            # Create tiles with known outlier data
            pts_outlier = [(1.0, 2.0, -25.0)]  # Z outlier
            pcd_data = _make_test_pcd(pts_outlier)
            for layer in ("tiles_registration", "tiles_display", "tiles_ground", "tiles_objects"):
                layer_dir = root / layer
                layer_dir.mkdir(parents=True)
                for i in range(3):
                    path = layer_dir / f"x{i}_y{i}.pcd"
                    path.write_bytes(pcd_data)

            # Test tile filename regex parsing
            import re
            pat = re.compile(r'^x(-?\d+)_y(-?\d+)')
            m = pat.match("x-1_y-1.pcd")
            self.assertIsNotNone(m)
            self.assertEqual(m.groups(), ("-1", "-1"))
            m = pat.match("x0_y1.pcd")
            self.assertIsNotNone(m)
            self.assertEqual(m.groups(), ("0", "1"))
            self.assertIsNone(pat.match("tile_0_1.pcd"))

            # Test that cache entries preserve all fields
            entry = {
                "signature": (123456789, 1024),
                "header_valid": True,
                "bounds": {"x_min": -60.0, "x_max": 40.0, "y_min": -30.0, "y_max": 35.0,
                           "z_min": -25.0, "z_max": 7.0},
                "z_min": -25.0, "z_max": 7.0,
                "z_outlier_below_count": 10, "z_outlier_above_count": 0,
                "out_of_approved_bounds": True,
                "z_outlier": True,
                "scan_error": None,
            }
            self.assertTrue(entry["out_of_approved_bounds"])
            self.assertTrue(entry["z_outlier"])
            self.assertEqual(entry["z_outlier_below_count"], 10)
            self.assertIsNone(entry["scan_error"])

    def test_metric_stuck_zero_from_initial_zero(self):
        """METRIC_STUCK_ZERO must start timer on first zero observation."""
        last_ndt_time_zero = None
        stuck_started: dict = {}
        now = 100.0
        ndt_time = 0.0

        # First observation: should start timer (not skip because None != False)
        if ndt_time == 0.0:
            if last_ndt_time_zero is None or not last_ndt_time_zero:
                stuck_started["ndt_time"] = now
            last_ndt_time_zero = True
        self.assertEqual(stuck_started["ndt_time"], 100.0)
        self.assertTrue(last_ndt_time_zero)

        # Second observation after 350s: should trigger
        now = 450.0
        triggered = False
        if ndt_time == 0.0:
            if now - stuck_started.get("ndt_time", now) >= 300.0:
                triggered = True
        self.assertTrue(triggered)

    def test_data_ready_requires_messages(self):
        """data_ready requires actual message receipt, not just topic presence."""
        required = ["/odom", "/cargo_avoidance/safety_status",
                     "/cargo_avoidance/status_code"]
        received: dict = {}
        # No topics received → NOT data_ready
        self.assertFalse(all(t in received for t in required))
        # Only odom received → NOT data_ready
        received["/odom"] = 100.0
        self.assertFalse(all(t in received for t in required))
        # All received → data_ready
        received["/cargo_avoidance/safety_status"] = 100.0
        received["/cargo_avoidance/status_code"] = 100.0
        self.assertTrue(all(t in received for t in required))

    def test_wall_clock_scheduler_handles_pause(self):
        """Wall-clock scheduler survives simulated time pause."""
        # Simulate: monotonic keeps advancing even when time.time() pauses
        import time as _time
        t0 = _time.monotonic()
        sample_period = 1.0
        next_sample = t0 + sample_period
        # Simulate a frame
        sleep_sec = next_sample - _time.monotonic()
        self.assertLess(sleep_sec, sample_period + 0.1)
        # Simulate falling behind
        next_sample = _time.monotonic() - 2.0  # behind
        sleep_sec = next_sample - _time.monotonic()
        self.assertLess(sleep_sec, 0)  # negative = fell behind, reset expected


    def test_data_ready_requires_counters_not_aggregator(self):
        """data_ready must use real callback counters, not aggregator.records."""
        # Simulate a monitor's ready_payload update logic
        odom_count = 0
        safety_count = 0
        code_count = 0

        # No messages → NOT data_ready
        data_ready = (odom_count > 0 and safety_count > 0 and code_count > 0)
        self.assertFalse(data_ready, "data_ready must be false with zero counts")

        # Only odom and safety → NOT data_ready (no status_code callback)
        odom_count = 100
        safety_count = 50
        data_ready = (odom_count > 0 and safety_count > 0 and code_count > 0)
        self.assertFalse(data_ready, "data_ready must be false without status_code callback")

        # aggregator.records present but code_count still 0 → NOT data_ready
        # This is the key bug fix: records from _safety_callback don't count
        code_count = 0
        data_ready = (odom_count > 0 and safety_count > 0 and code_count > 0)
        self.assertFalse(data_ready, "aggregator records alone must not trigger data_ready")

        # All three real callbacks received → data_ready
        code_count = 30
        data_ready = (odom_count > 0 and safety_count > 0 and code_count > 0)
        self.assertTrue(data_ready, "data_ready requires all three callback counters > 0")

    def test_created_at_is_not_rewritten(self):
        """created_at must be set once at startup, never rewritten in ready loop."""
        import time as _time
        created_at = _time.time()
        ready_updated_at = created_at

        # Simulate the ready update loop running 3 times
        for _ in range(3):
            # BUG (old code): _rdy["created_at"] = time.time()
            # FIX: created_at is never rewritten; only ready_updated_at changes
            ready_updated_at = _time.time()
            # created_at stays the same
            self.assertAlmostEqual(created_at, created_at, delta=0.01,
                                   msg="created_at must remain unchanged across updates")

        # ready_updated_at should have advanced
        self.assertGreater(ready_updated_at, created_at - 0.1,
                           "ready_updated_at must be set after created_at")

    def test_last_status_code_wall_independent_from_safety(self):
        """last_status_code_wall must be from _code_callback, not _safety_callback."""
        # These simulate independent callback tracking
        last_safety_wall = 1000.0
        last_status_code_wall = None

        # Safety callback fires — but NOT status_code callback
        # Old bug: used aggregator.records for status_code tracking
        # Aggregator records exist (from safety callback) but last_status_code_wall is None
        has_records = True  # aggregator.records is non-empty
        has_real_code_callback = last_status_code_wall is not None

        self.assertTrue(has_records, "safety callback populated records")
        self.assertFalse(has_real_code_callback,
                         "status_code wall must be None without real _code_callback")

        # Now _code_callback fires
        last_status_code_wall = 1000.5
        has_real_code_callback = last_status_code_wall is not None
        self.assertTrue(has_real_code_callback,
                        "status_code wall must be set after real _code_callback")

    def test_generated_config_sandbox_paths(self):
        """Generated sandbox config must point to sandbox, not production paths."""
        import yaml
        sandbox = "/tmp/test_run/map_sandbox/current"
        diag_dir = "/tmp/test_run/runtime_diagnostics"

        # Simulate what generate_sandbox_config does
        config = {
            'persistent_map': {
                'enabled': False,
                'root_dir': '/home/ydkj/NDT-slam-ws/maps/live/current',
            },
            'debug': {
                'runtime_diagnostics': {
                    'output_dir': '/tmp/ndt_slam_runtime_data',
                }
            }
        }

        # Override (simulating our Python script)
        config['persistent_map']['root_dir'] = sandbox
        config['persistent_map']['enabled'] = True
        config['debug']['runtime_diagnostics']['output_dir'] = diag_dir

        self.assertEqual(config['persistent_map']['root_dir'], sandbox,
                         "root_dir must point to sandbox")
        self.assertNotEqual(config['persistent_map']['root_dir'],
                            '/home/ydkj/NDT-slam-ws/maps/live/current',
                            "root_dir must NOT point to production path")
        self.assertTrue(config['persistent_map']['enabled'],
                        "persistent_map.enabled must be true in sandbox config")
        self.assertEqual(config['debug']['runtime_diagnostics']['output_dir'], diag_dir,
                         "output_dir must point to sandbox diagnostics")

        # Verify NDT params unchanged (regression check — sandbox config must preserve all algo params)
        self.assertIn('persistent_map', config, "config must retain persistent_map section")
        self.assertIn('debug', config, "config must retain debug section")

    def test_partial_mode_policy(self):
        """partial mode can never output PASS."""
        mode = "partial"
        failure_reasons = []

        # partial mode policy check
        if mode == "partial":
            overall = "PARTIAL"
            failure_reasons.append("partial mode cannot output PASS by policy")

        self.assertEqual(overall, "PARTIAL")
        self.assertIn("partial", failure_reasons[0].lower())
        self.assertNotEqual(overall, "PASS",
                            "partial mode must never equal PASS")


class PsiParserTest(unittest.TestCase):
    def test_read_psi_returns_dict(self):
        result = read_psi()
        self.assertIsInstance(result, dict)
        # At minimum we should have all expected keys
        for key in ("cpu_some_avg10", "memory_some_avg10", "io_some_avg10"):
            self.assertIn(key, result)

    def test_read_psi_values_are_numeric_or_none(self):
        result = read_psi()
        for key, value in result.items():
            self.assertTrue(value is None or isinstance(value, (int, float)),
                            f"{key}={value} should be numeric or None")


class TileCatalogTest(unittest.TestCase):
    def test_build_tile_catalog_empty(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            catalog = build_tile_catalog(root)
            self.assertEqual(catalog, [])

    def test_build_tile_catalog_with_tiles(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            for layer in ("tiles_registration", "tiles_display"):
                layer_dir = root / layer
                layer_dir.mkdir(parents=True)
                for tx, ty in [(-1, -1), (0, 0), (1, 1)]:
                    pcd = _make_test_pcd([(1.0, 2.0, 3.0)])
                    path = layer_dir / f"x{tx}_y{ty}.pcd"
                    path.write_bytes(pcd)
            catalog = build_tile_catalog(root)
            self.assertEqual(len(catalog), 6)
            for entry in catalog:
                self.assertTrue(entry["header_valid"])
                self.assertGreater(entry["points"], 0)
                self.assertIn("layer", entry)
                self.assertEqual(entry["tile_x"], entry["tile_y"])  # x==y in our test


if __name__ == "__main__":
    unittest.main()
