#!/usr/bin/env python3
"""Read-only NDT-SLAM server monitor with an offline-testable core.

The module intentionally imports ROS only inside ``RosRuntimeMonitor``.  The
aggregation, append-safe writers and report data model therefore run on a
developer machine without a ROS master.  This process never creates a ROS
publisher and never writes SLAM parameters or persistent-map state.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import math
import os
import queue
import shutil
import signal
import subprocess
import sys
import threading
import time
from collections import Counter, deque
from pathlib import Path
from typing import Any, Deque, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

try:  # unavailable on Windows, where only offline aggregation is exercised
    import fcntl  # type: ignore
except ImportError:  # pragma: no cover - platform dependent
    fcntl = None  # type: ignore


CLEAR_CODE = 14
HAZARD_CODES = {17, 18}
FAULT_CODES = set(range(30, 36))
DEFAULT_WINDOWS = (60.0, 600.0)
RUNTIME_SAMPLE_FIELDS = (
    "wall_time", "pid", "cpu_percent", "rss_mb", "threads", "fd_count",
    "restart_count", "disk_free_gb", "runtime_status_age_sec",
    "runtime_status_stale", "odom_age_sec", "odom_hz", "x", "y", "z",
    "yaw_deg", "speed_mps", "pose_step_p50_m", "pose_step_p95_m",
    "pose_step_max_m", "manifest_state", "active_manifest_present",
    "last_good_manifest_present", "suspension_marker_present",
    "persistent_tmp_files", "persistent_tile_files",
    "persistent_newest_tile_mtime", "persistent_size_mb",
    "manifest_generation", "manifest_revision", "manifest_total_cells",
    "manifest_mature_cells",
    "runtime_total_frames", "runtime_total_keyframes",
    "runtime_active_keyframes", "runtime_is_stationary",
    "runtime_stationary_frame_count", "runtime_global_map_points",
    "runtime_display_map_points", "runtime_ground_map_points",
    "runtime_objects_map_points", "runtime_objects_clean_map_points",
    "runtime_local_map_points", "runtime_local_map_update_allowed",
    "runtime_persistent_map_commit_allowed",
    "runtime_prediction_only_consecutive_frames", "runtime_dirty_tile_count",
    "runtime_flushed_tile_count", "runtime_memory_guard_triggered",
    "runtime_disk_guard_triggered", "runtime_pointcloud_timeout_sec",
    "runtime_pointcloud_stale", "runtime_last_ndt_fitness",
    "runtime_average_process_time_ms", "runtime_average_ndt_time_ms",
    "runtime_static_evidence_epoch", "runtime_static_evidence_revision",
    "runtime_static_evidence_cells", "runtime_static_evidence_mature_cells",
    "runtime_static_evidence_latest_sequence", "runtime_static_query_reason",
    "runtime_static_query_authorized", "runtime_static_clean_build_started",
    "runtime_static_clean_build_applied",
    "runtime_static_clean_build_snapshot_only",
    "runtime_static_clean_build_discarded", "runtime_static_cells_confirmed",
    "runtime_static_cells_invalidated", "runtime_obstacle_track_created_count",
    "runtime_obstacle_track_reset_count",
)


def _finite(value: Any, default: float = float("nan")) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def _as_bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _json_safe(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, deque)):
        return [_json_safe(item) for item in value]
    return value


def _percentile(values: Sequence[float], percentile: float) -> Optional[float]:
    clean = sorted(value for value in values if math.isfinite(value))
    if not clean:
        return None
    if len(clean) == 1:
        return clean[0]
    position = (len(clean) - 1) * percentile
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return clean[lower]
    fraction = position - lower
    return clean[lower] * (1.0 - fraction) + clean[upper] * fraction


def is_runtime_status_stale(last_mtime: Optional[float], now: float,
                            threshold_sec: float) -> bool:
    return (last_mtime is None or not math.isfinite(last_mtime) or
            now - last_mtime > max(0.0, threshold_sec))


def normalize_timeout_status(status: int) -> int:
    """GNU timeout 124 means a scheduled soak completed normally."""
    return 0 if int(status) == 124 else int(status)


def atomic_write_json(path: Path, payload: Mapping[str, Any]) -> None:
    """Atomically replace a JSON file in the same directory."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(_json_safe(payload), stream, ensure_ascii=False, indent=2,
                  sort_keys=True, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def append_csv(path: Path, fieldnames: Sequence[str], row: Mapping[str, Any]) -> None:
    """Append one CSV row and write its header only for a new/empty file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    write_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(fieldnames), extrasaction="ignore")
        if write_header:
            writer.writeheader()
        writer.writerow(dict(row))


@dataclasses.dataclass(frozen=True)
class SafetyRecord:
    wall_time: float
    source_stamp: float
    code: int
    reason: str
    warning_valid: bool = False
    evidence_state: int = 0
    cargo_track_id: int = 0
    obstacle_track_id: int = 0
    provenance: int = 0
    source_validated: bool = False
    large_geometry_valid: bool = False
    static_authorized: bool = False
    nearest_distance_m: float = float("nan")
    vertical_clearance_m: float = float("nan")
    association_reset_reason: str = ""


class SafetyAggregator:
    """Bounded, source-stamp-aware safety history and transition statistics."""

    def __init__(self, windows: Sequence[float] = DEFAULT_WINDOWS,
                 max_records: int = 200_000) -> None:
        self.windows = tuple(sorted({float(v) for v in windows if float(v) > 0.0}))
        self.records: Deque[SafetyRecord] = deque(maxlen=max(100, int(max_records)))
        self.events: Deque[Dict[str, Any]] = deque(maxlen=max(100, int(max_records // 4)))
        self.last_source_stamp: Optional[float] = None
        self.last_status_code: Optional[int] = None
        self.time_rollbacks = 0
        self.duplicate_stamps = 0
        self.status_code_mismatches = 0
        self.start_wall_time: Optional[float] = None
        self.track_created = 0
        self.track_reset = 0
        self.unique_track_ids: set[int] = set()
        self._last_track_id = 0
        self._fault34_started: Optional[float] = None
        self._hazard_started: Optional[Tuple[int, float]] = None
        self.hazard_event_durations: Deque[Dict[str, float]] = deque(maxlen=10_000)
        self.recovery_34_to_14_sec: Deque[float] = deque(maxlen=10_000)
        self.confirmation_34_to_warning_sec: Deque[float] = deque(maxlen=10_000)
        self._last_mismatch_pair: Optional[Tuple[int, int]] = None

    def ingest(self, values: Mapping[str, Any], *, source_stamp: float,
               wall_time: Optional[float] = None) -> List[Dict[str, Any]]:
        wall = time.time() if wall_time is None else float(wall_time)
        stamp = float(source_stamp)
        emitted: List[Dict[str, Any]] = []
        if not math.isfinite(wall) or not math.isfinite(stamp):
            return emitted
        if self.last_source_stamp is not None:
            if abs(stamp - self.last_source_stamp) <= 1.0e-6:
                self.duplicate_stamps += 1
                return emitted
            if stamp < self.last_source_stamp - 1.0e-6:
                self.time_rollbacks += 1
                emitted.append({"event": "SOURCE_TIME_ROLLBACK",
                                "wall_time": wall,
                                "previous_stamp": self.last_source_stamp,
                                "source_stamp": stamp})
        self.last_source_stamp = stamp
        code = int(values.get("requested_alarm_code", values.get("code", 30)))
        record = SafetyRecord(
            wall_time=wall,
            source_stamp=stamp,
            code=code,
            reason=str(values.get("reason", "unknown")),
            warning_valid=bool(values.get("warning_valid", False)),
            evidence_state=int(values.get("evidence_state", 0)),
            cargo_track_id=int(values.get("cargo_track_id", 0)),
            obstacle_track_id=int(values.get("obstacle_track_id", 0)),
            provenance=int(values.get("obstacle_provenance_type",
                                      values.get("provenance", 0))),
            source_validated=bool(values.get("obstacle_provenance_valid",
                                             values.get("source_validated", False))),
            large_geometry_valid=bool(values.get("obstacle_large_geometry_valid",
                                                 values.get("large_geometry_valid", False))),
            static_authorized=bool(values.get("static_authorized", False)),
            nearest_distance_m=_finite(values.get("nearest_obstacle_distance_m")),
            vertical_clearance_m=_finite(values.get("conservative_vertical_clearance_m")),
            association_reset_reason=str(values.get("obstacle_association_reset_reason", "")),
        )
        previous = self.records[-1] if self.records else None
        if self.start_wall_time is None:
            self.start_wall_time = wall
        self.records.append(record)

        if record.code == 34 and (previous is None or previous.code != 34):
            self._fault34_started = wall
        elif previous is not None and previous.code == 34 and record.code != 34:
            started = self._fault34_started
            if started is not None:
                elapsed = max(0.0, wall - started)
                if record.code == CLEAR_CODE:
                    self.recovery_34_to_14_sec.append(elapsed)
                elif record.code in HAZARD_CODES:
                    self.confirmation_34_to_warning_sec.append(elapsed)
            self._fault34_started = None
        if record.code in HAZARD_CODES and (
                previous is None or previous.code != record.code):
            if self._hazard_started is not None:
                prior_code, started = self._hazard_started
                self.hazard_event_durations.append(
                    {"code": float(prior_code), "duration_sec": max(0.0, wall - started)})
            self._hazard_started = (record.code, wall)
        elif previous is not None and previous.code in HAZARD_CODES and record.code not in HAZARD_CODES:
            if self._hazard_started is not None:
                prior_code, started = self._hazard_started
                self.hazard_event_durations.append(
                    {"code": float(prior_code), "duration_sec": max(0.0, wall - started)})
            self._hazard_started = None

        if record.obstacle_track_id > 0:
            self.unique_track_ids.add(record.obstacle_track_id)
        if record.obstacle_track_id != self._last_track_id:
            if record.obstacle_track_id > 0:
                self.track_created += 1
            if self._last_track_id > 0:
                self.track_reset += 1
            self._last_track_id = record.obstacle_track_id
        if record.association_reset_reason and (
                previous is None or
                record.association_reset_reason != previous.association_reset_reason):
            self.track_reset += 1

        transition = self._transition(previous, record)
        if transition:
            emitted.append(transition)
        for event in emitted:
            self.events.append(event)
        return emitted

    def check_status_code(self, code: int, *, wall_time: Optional[float] = None) -> Optional[Dict[str, Any]]:
        self.last_status_code = int(code)
        if not self.records or self.records[-1].code == int(code):
            self._last_mismatch_pair = None
            return None
        mismatch_pair = (self.records[-1].code, int(code))
        if self._last_mismatch_pair == mismatch_pair:
            return None
        self._last_mismatch_pair = mismatch_pair
        self.status_code_mismatches += 1
        event = {"event": "STATUS_CODE_MISMATCH",
                 "wall_time": time.time() if wall_time is None else wall_time,
                 "typed_code": self.records[-1].code,
                 "simple_code": int(code)}
        self.events.append(event)
        return event

    @staticmethod
    def _transition(previous: Optional[SafetyRecord], current: SafetyRecord) -> Optional[Dict[str, Any]]:
        if previous is None:
            kind = "SAFETY_ENTER"
        elif previous.code != current.code:
            if previous.code in HAZARD_CODES and current.code == CLEAR_CODE:
                kind = "SAFETY_CLEAR"
            elif previous.code in FAULT_CODES and current.code not in FAULT_CODES:
                kind = "SAFETY_FAULT_CLEAR"
            elif current.code in FAULT_CODES:
                kind = "SAFETY_FAULT_ENTER"
            elif current.code in HAZARD_CODES:
                kind = "SAFETY_WARNING_ENTER"
            else:
                kind = "SAFETY_CHANGE"
        elif previous.reason != current.reason:
            kind = "SAFETY_REASON_CHANGE"
        elif previous.obstacle_track_id != current.obstacle_track_id:
            kind = "OBSTACLE_TRACK_CHANGE"
        elif previous.static_authorized != current.static_authorized:
            kind = "STATIC_AUTHORIZATION_CHANGE"
        else:
            return None
        return {
            "event": kind,
            "wall_time": current.wall_time,
            "source_stamp": current.source_stamp,
            "previous_code": None if previous is None else previous.code,
            "code": current.code,
            "reason": current.reason,
            "cargo_track_id": current.cargo_track_id,
            "obstacle_track_id": current.obstacle_track_id,
            "distance_m": current.nearest_distance_m,
            "clearance_m": current.vertical_clearance_m,
            "provenance": current.provenance,
        }

    def _selected(self, now: float, window: Optional[float]) -> List[SafetyRecord]:
        if window is None:
            return list(self.records)
        cutoff = now - window
        records = list(self.records)
        index = 0
        while index < len(records) and records[index].wall_time < cutoff:
            index += 1
        if index > 0:
            index -= 1  # retain the state spanning the window boundary
        return records[index:]

    def summarize(self, *, now: Optional[float] = None,
                  window: Optional[float] = None) -> Dict[str, Any]:
        current_time = time.time() if now is None else float(now)
        records = self._selected(current_time, window)
        if not records:
            return {"window_sec": window, "samples": 0, "code_counts": {},
                    "code_duration_sec": {}, "code_duration_ratio": {}}
        cutoff = records[0].wall_time if window is None else current_time - window
        durations: Counter[int] = Counter()
        reasons: Counter[str] = Counter()
        counts: Counter[int] = Counter()
        longest = {33: 0.0, 34: 0.0}
        current_streak = {33: 0.0, 34: 0.0}
        run_code: Optional[int] = None
        run_start = max(cutoff, records[0].wall_time)
        authorized = source_unvalidated = geometry_rejected = evidence_samples = 0
        local_track_created = local_track_reset = 0
        prior_track = 0
        prior_reset_reason = ""
        for index, record in enumerate(records):
            start = max(cutoff, record.wall_time)
            end = current_time if index + 1 == len(records) else min(
                current_time, records[index + 1].wall_time)
            if end < start:
                continue
            durations[record.code] += end - start
            counts[record.code] += 1
            reasons[record.reason] += 1
            if record.obstacle_track_id > 0 or record.code in HAZARD_CODES or record.code == 34:
                evidence_samples += 1
                authorized += int(record.static_authorized)
                source_unvalidated += int(not record.source_validated)
                geometry_rejected += int(not record.large_geometry_valid)
            if record.obstacle_track_id != prior_track:
                if record.obstacle_track_id > 0:
                    local_track_created += 1
                if prior_track > 0:
                    local_track_reset += 1
                prior_track = record.obstacle_track_id
            if (record.association_reset_reason and
                    record.association_reset_reason != prior_reset_reason):
                local_track_reset += 1
            prior_reset_reason = record.association_reset_reason
            if run_code != record.code:
                if run_code in longest:
                    longest[run_code] = max(longest[run_code], start - run_start)
                run_code = record.code
                run_start = start
        if run_code in longest:
            longest[run_code] = max(longest[run_code], current_time - run_start)
            current_streak[run_code] = current_time - run_start
        total_duration = sum(durations.values())
        run_minutes = max(1.0 / 60.0, total_duration / 60.0)
        denominator = max(1, evidence_samples)
        warning_events = sum(1 for event in self.events
                             if event.get("event") == "SAFETY_WARNING_ENTER" and
                             event.get("wall_time", 0.0) >= cutoff)
        result = {
            "window_sec": window,
            "samples": len(records),
            "code_counts": {str(k): v for k, v in sorted(counts.items())},
            "code_duration_sec": {str(k): round(v, 6) for k, v in sorted(durations.items())},
            "code_duration_ratio": {
                str(k): (v / total_duration if total_duration > 0.0 else 0.0)
                for k, v in sorted(durations.items())},
            "reason_counts": dict(reasons.most_common()),
            "longest_33_sec": longest[33],
            "longest_34_sec": longest[34],
            "current_33_sec": current_streak[33],
            "current_34_sec": current_streak[34],
            "warning_events": warning_events,
            "completed_warning_durations": list(self.hazard_event_durations),
            "unique_obstacle_tracks": len({r.obstacle_track_id for r in records if r.obstacle_track_id > 0}),
            "track_created": local_track_created,
            "track_reset": local_track_reset,
            "track_churn_per_min": local_track_reset / run_minutes,
            "static_authorized_ratio": authorized / denominator,
            "source_unvalidated_ratio": source_unvalidated / denominator,
            "geometry_rejected_ratio": geometry_rejected / denominator,
            "recovery_34_to_14_sec": list(self.recovery_34_to_14_sec),
            "confirmation_34_to_warning_sec": list(
                self.confirmation_34_to_warning_sec),
            "recovery_34_to_14_p95_sec": _percentile(
                list(self.recovery_34_to_14_sec), 0.95),
            "confirmation_34_to_warning_p95_sec": _percentile(
                list(self.confirmation_34_to_warning_sec), 0.95),
        }
        return result

    def full_summary(self, *, now: Optional[float] = None) -> Dict[str, Any]:
        current_time = time.time() if now is None else float(now)
        return {
            "generated_at": current_time,
            "current": dataclasses.asdict(self.records[-1]) if self.records else None,
            "windows": {str(int(window)): self.summarize(now=current_time, window=window)
                        for window in self.windows},
            "run": self.summarize(now=current_time, window=None),
            "duplicate_source_stamps": self.duplicate_stamps,
            "source_time_rollbacks": self.time_rollbacks,
            "status_code_mismatches": self.status_code_mismatches,
        }


class AsyncRunWriter:
    """Single bounded writer thread; callbacks never perform disk I/O."""

    def __init__(self, run_dir: Path, max_queue: int = 4096,
                 rotation_size_mb: float = 50.0,
                 rotation_count: int = 5) -> None:
        self.run_dir = run_dir
        self.queue: "queue.Queue[Tuple[str, Any]]" = queue.Queue(maxsize=max_queue)
        self.stop_event = threading.Event()
        self.dropped = 0
        self.rotation_bytes = max(0, int(float(rotation_size_mb) * 1024 * 1024))
        self.rotation_count = max(1, int(rotation_count))
        self.thread = threading.Thread(target=self._run, name="server-monitor-writer", daemon=True)
        self.thread.start()

    def submit(self, kind: str, payload: Any) -> None:
        try:
            self.queue.put_nowait((kind, payload))
        except queue.Full:
            self.dropped += 1

    def _run(self) -> None:
        while not self.stop_event.is_set() or not self.queue.empty():
            try:
                kind, payload = self.queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                if kind == "jsonl":
                    relative, value = payload
                    path = self.run_dir / relative
                    path.parent.mkdir(parents=True, exist_ok=True)
                    self._rotate(path)
                    with path.open("a", encoding="utf-8", newline="\n") as stream:
                        stream.write(json.dumps(_json_safe(value), ensure_ascii=False,
                                                sort_keys=True, allow_nan=False) + "\n")
                elif kind == "csv":
                    relative, fields, value = payload
                    path = self.run_dir / relative
                    self._rotate(path)
                    append_csv(path, fields, value)
                elif kind == "atomic_json":
                    relative, value = payload
                    atomic_write_json(self.run_dir / relative, value)
                elif kind == "text":
                    relative, line = payload
                    path = self.run_dir / relative
                    path.parent.mkdir(parents=True, exist_ok=True)
                    self._rotate(path)
                    with path.open("a", encoding="utf-8", newline="\n") as stream:
                        stream.write(str(line).rstrip("\n") + "\n")
            finally:
                self.queue.task_done()

    def _rotate(self, path: Path) -> None:
        try:
            path.relative_to(self.run_dir / "logs")
        except ValueError:
            return
        if self.rotation_bytes <= 0 or not path.exists() or path.stat().st_size < self.rotation_bytes:
            return
        oldest = path.with_name(path.name + ".{}".format(self.rotation_count))
        try:
            oldest.unlink()
        except FileNotFoundError:
            pass
        for index in range(self.rotation_count - 1, 0, -1):
            source = path.with_name(path.name + ".{}".format(index))
            if source.exists():
                source.replace(path.with_name(path.name + ".{}".format(index + 1)))
        path.replace(path.with_name(path.name + ".1"))

    def close(self, timeout: float = 10.0) -> None:
        self.stop_event.set()
        self.thread.join(timeout=timeout)


def load_config(path: Optional[Path]) -> Dict[str, Any]:
    defaults: Dict[str, Any] = {
        "sample_period_sec": 1.0,
        "summary_period_sec": 10.0,
        "windows_sec": [60, 600],
        "odom_stale_sec": 1.0,
        "runtime_status_stale_sec": 5.0,
        "repeat_period_sec": 30.0,
        "writer_queue_size": 4096,
        # Root-level keys from server_monitor.yaml
        "motion_capture": {},
        "fallback_cargo_envelope": {},
        "operational_output": {},
    }
    if not path or not path.exists():
        return defaults
    try:
        import yaml  # type: ignore
        loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        # Merge monitor section (backward compat)
        monitor = loaded.get("monitor", {})
        if isinstance(monitor, dict):
            defaults.update(monitor)
        # Merge root-level observability keys
        for key in ("motion_capture", "fallback_cargo_envelope", "operational_output"):
            if key in loaded and isinstance(loaded[key], dict):
                defaults[key] = loaded[key]
    except (ImportError, OSError, ValueError):
        pass
    return defaults


def read_json(path: Path) -> Optional[Dict[str, Any]]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else None
    except (OSError, ValueError):
        return None


def process_metrics(pid: Optional[int]) -> Dict[str, Any]:
    result: Dict[str, Any] = {"pid": pid, "rss_mb": None, "threads": None,
                              "fd_count": None, "process_start_ticks": None,
                              "cpu_ticks": None}
    if not pid or pid <= 0:
        return result
    root = Path("/proc") / str(pid)
    try:
        for line in (root / "status").read_text(encoding="utf-8").splitlines():
            if line.startswith("VmRSS:"):
                result["rss_mb"] = int(line.split()[1]) / 1024.0
            elif line.startswith("Threads:"):
                result["threads"] = int(line.split()[1])
        stat = (root / "stat").read_text(encoding="utf-8").split()
        result["process_start_ticks"] = int(stat[21])
        result["cpu_ticks"] = int(stat[13]) + int(stat[14])
        result["fd_count"] = len(list((root / "fd").iterdir()))
    except (OSError, ValueError, IndexError):
        result["pid"] = None
    return result


def find_process(name: str = "ndt_slam_node") -> Optional[int]:
    proc = Path("/proc")
    if not proc.exists():
        return None
    for item in proc.iterdir():
        if not item.name.isdigit():
            continue
        try:
            command = (item / "cmdline").read_bytes().replace(b"\0", b" ").decode("utf-8", "replace")
        except OSError:
            continue
        if name in command:
            return int(item.name)
    return None


class RosRuntimeMonitor:
    """ROS adapter. All subscriptions are read-only."""

    def __init__(self, args: argparse.Namespace, config: Mapping[str, Any]) -> None:
        import rospy
        from lidar_slam2_msgs.msg import CargoSafetyStatus
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Log
        from std_msgs.msg import Int32, String

        self.rospy = rospy
        self.args = args
        self.config = dict(config)
        self.run_dir = Path(args.run_dir).expanduser().resolve()
        for relative in ("logs", "samples", "snapshots", "reports", "bags"):
            (self.run_dir / relative).mkdir(parents=True, exist_ok=True)
        self.writer = AsyncRunWriter(
            self.run_dir, int(config.get("writer_queue_size", 4096)),
            float(config.get("log_rotation_size_mb", 50.0)),
            int(config.get("log_rotation_count", 5)))
        self.aggregator = SafetyAggregator(config.get("windows_sec", DEFAULT_WINDOWS))
        self.aggregator_lock = threading.Lock()
        self.runtime_path = Path(args.persistent_root).expanduser().resolve() / "runtime_status.json"
        self.persistent_root = self.runtime_path.parent
        self.last_odom_wall: Optional[float] = None
        self.odom_times: Deque[float] = deque(maxlen=6000)
        self.pose_steps: Deque[float] = deque(maxlen=6000)
        self.last_pose: Optional[Tuple[float, float, float]] = None
        self.current_pose: Optional[Dict[str, float]] = None
        self.latest_static_debug: Dict[str, Any] = {}
        self.latest_runtime: Dict[str, Any] = {}
        self.last_runtime_mtime: Optional[float] = None
        self.last_summary_wall = 0.0
        self.last_safety_repeat_wall = 0.0
        self.last_process_start: Optional[int] = None
        self.restart_count = 0
        self.pid: Optional[int] = None
        self.shutdown_requested = False
        self.snapshot_requested = False
        self.last_guard_state: Dict[str, bool] = {}
        self.last_static_epoch: Optional[int] = None
        self.last_manifest_state: Optional[str] = None
        self.last_filesystem_scan_wall = 0.0
        self.filesystem_cache: Dict[str, Any] = {}
        self.last_cpu_sample: Optional[Tuple[float, int]] = None

        # Motion episode tracking
        motion_cfg = config.get("motion_capture", {})
        self._motion_enter_speed = float(motion_cfg.get("enter_speed_mps", 0.08))
        self._motion_enter_confirm = float(motion_cfg.get("enter_confirm_sec", 0.5))
        self._motion_exit_speed = float(motion_cfg.get("exit_speed_mps", 0.03))
        self._motion_exit_confirm = float(motion_cfg.get("exit_confirm_sec", 2.0))
        self._motion_state: str = "UNKNOWN"
        self._motion_enter_candidate: Optional[float] = None
        self._motion_exit_candidate: Optional[float] = None
        self._motion_episode_start: Optional[float] = None
        self._motion_episode_start_pose: Optional[Dict[str, float]] = None
        self._motion_speeds: Deque[float] = deque(maxlen=600)
        self._motion_episode_count = 0

        # Terminal event classifier state
        self._last_health_line = 0.0
        self._terminal_event_counts: Counter[str] = Counter()

        rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=100)
        rospy.Subscriber("/cargo_avoidance/safety_status", CargoSafetyStatus,
                         self._safety_callback, queue_size=100)
        rospy.Subscriber("/cargo_avoidance/status_code", Int32,
                         self._code_callback, queue_size=100)
        rospy.Subscriber("/cargo_avoidance/static_evidence_debug", String,
                         self._static_callback, queue_size=20)
        rospy.Subscriber("/cargo_avoidance/cargo_geometry_debug", String,
                         self._cargo_geometry_callback, queue_size=50)
        rospy.Subscriber("/rosout_agg", Log, self._rosout_callback, queue_size=200)

        # Track-level geometry sample filtering state
        self._geo_track_samples: Dict[int, int] = {}
        self._geo_track_episodes: Dict[int, str] = {}
        self._geo_last_authoritative: Dict[int, float] = {}

    @staticmethod
    def _read_boot_id() -> str:
        try:
            return Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        except OSError:
            return "unknown"

    def _cargo_geometry_callback(self, message: Any) -> None:
        """Filter and save authoritative measured geometry samples only."""
        wall = time.time()
        text = str(message.data)
        self.writer.submit("jsonl", ("samples/cargo_geometry_samples.jsonl",
                                     {"wall_time": wall, "raw": text}))
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            return
        if not isinstance(parsed, dict):
            return

        # CSV record all samples (for debugging)
        csv_fields = [
            "wall_time", "stamp", "track_id", "track_state", "lock_state",
            "geometry_source", "authoritative", "observation_valid", "height_valid",
            "points", "support", "confidence",
            "center_x", "center_y", "center_z",
            "length_m", "width_m", "height_m", "yaw_deg",
            "bottom_z", "top_z", "vertical_source", "failure_reason",
            "hook_load_state", "gravity_voltage", "gravity_age_sec",
            "fallback_active"
        ]
        row = {"wall_time": wall}
        for field in csv_fields[1:]:  # skip wall_time which we already set
            row[field] = parsed.get(field, "")
        self.writer.submit("csv", ("samples/cargo_geometry_samples.csv", csv_fields, row))

        # Filter: only authoritative measured geometry
        geo_source = str(parsed.get("geometry_source", ""))
        authoritative = parsed.get("authoritative", False)
        track_state = str(parsed.get("track_state", ""))
        height_valid = parsed.get("height_valid", False)
        observation_valid = parsed.get("observation_valid", False)
        support = int(parsed.get("support", 0))
        points = int(parsed.get("points", 0))
        confidence = float(parsed.get("confidence", 0.0))

        # Rejection rules from plan
        if geo_source != "MEASURED":
            return  # DISPLAY_FROZEN, DEFAULT_FALLBACK, NONE rejected
        if not authoritative:
            return
        if track_state != "LOCKED":
            return  # EMPTY, LOCKING, LOST_HOLD rejected
        if not observation_valid:
            return
        if not height_valid:
            return
        if support <= 0:
            return
        if points <= 0:
            return
        if confidence <= 0.0:
            return
        dims = [float(parsed.get(k, 0.0)) for k in ("length_m", "width_m", "height_m")]
        if any(v <= 0.0 or not math.isfinite(v) for v in dims):
            return

        # Save filtered authoritative sample
        filtered = dict(parsed)
        filtered["wall_time"] = wall
        self.writer.submit("jsonl", ("samples/cargo_geometry_authoritative.jsonl", filtered))
        filtered_csv = {"wall_time": wall}
        for field in csv_fields[1:]:
            filtered_csv[field] = parsed.get(field, "")
        self.writer.submit("csv", ("samples/cargo_geometry_authoritative.csv",
                                   csv_fields, filtered_csv))

        # Track per-track_id sample count
        track_id = int(parsed.get("track_id", 0))
        if track_id > 0:
            self._geo_track_samples[track_id] = self._geo_track_samples.get(track_id, 0) + 1

    def _odom_callback(self, message: Any) -> None:
        wall = time.time()
        pose = message.pose.pose
        position = (float(pose.position.x), float(pose.position.y), float(pose.position.z))
        if not all(math.isfinite(v) for v in position):
            self.writer.submit("jsonl", ("logs/ros_events.jsonl",
                                         {"event": "NONFINITE_ODOM", "wall_time": wall}))
            return
        if self.last_pose is not None:
            self.pose_steps.append(math.sqrt(sum((a - b) ** 2 for a, b in zip(position, self.last_pose))))
        self.last_pose = position
        self.last_odom_wall = wall
        self.odom_times.append(wall)
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        velocity = message.twist.twist.linear
        self.current_pose = {"x": position[0], "y": position[1], "z": position[2],
                             "yaw_deg": math.degrees(yaw),
                             "speed_mps": math.hypot(velocity.x, velocity.y)}

    def _safety_callback(self, message: Any) -> None:
        wall = time.time()
        stamp = float(message.header.stamp.to_sec())
        values = {field: getattr(message, field) for field in (
            "requested_alarm_code", "reason", "warning_valid", "evidence_state",
            "cargo_track_id", "obstacle_track_id", "obstacle_provenance_type",
            "obstacle_provenance_valid", "obstacle_large_geometry_valid",
            "nearest_obstacle_distance_m", "conservative_vertical_clearance_m",
            "obstacle_association_reset_reason")}
        values["static_authorized"] = _as_bool(
            self.latest_static_debug.get("authorized", False))
        with self.aggregator_lock:
            events = self.aggregator.ingest(
                values, source_stamp=stamp, wall_time=wall)
        for event in events:
            self._emit_event(event)
        row = dict(values)
        row.update({"wall_time": wall, "source_stamp": stamp,
                    "warning_code": int(message.warning_code),
                    "fault_code": int(message.fault_code),
                    "fault_mask": int(message.fault_mask),
                    "valid": bool(message.valid),
                    "message_age_sec": max(0.0, self.rospy.Time.now().to_sec() - stamp)})
        row.update({
            "static_query_reason": self.latest_static_debug.get("query_reason", ""),
            "static_matched_ratio": self.latest_static_debug.get("matched_ratio", ""),
            "static_matched_iou": self.latest_static_debug.get("matched_iou", ""),
            "static_height_overlap": self.latest_static_debug.get("height_overlap", ""),
            "static_index_epoch": self.latest_static_debug.get("index_epoch", ""),
            "static_index_revision": self.latest_static_debug.get("index_revision", ""),
        })
        fields = list(row.keys())
        self.writer.submit("csv", ("samples/safety_samples.csv", fields, row))

    def _code_callback(self, message: Any) -> None:
        with self.aggregator_lock:
            event = self.aggregator.check_status_code(
                int(message.data), wall_time=time.time())
        if event:
            self._emit_event(event)

    def _static_callback(self, message: Any) -> None:
        text = str(message.data)
        parsed: Dict[str, Any] = {}
        try:
            value = json.loads(text)
            if isinstance(value, dict):
                parsed = value
        except ValueError:
            for token in text.replace(",", " ").split():
                if "=" in token:
                    key, value = token.split("=", 1)
                    parsed[key.strip()] = value.strip()
        self.latest_static_debug = parsed
        row = {"wall_time": time.time(), "raw": text}
        row.update(parsed)
        self.writer.submit("jsonl", ("samples/static_evidence_samples.jsonl", row))
        self.writer.submit("csv", ("samples/static_evidence_samples.csv", [
            "wall_time", "authorized", "query_reason", "index_epoch",
            "index_revision", "index_cells", "latest_sequence",
            "matched_ratio", "matched_iou",
            "height_overlap", "raw"], row))

    def _rosout_callback(self, message: Any) -> None:
        if int(message.level) < int(message.WARN):
            return
        text = str(message.msg)
        tags = ("SO3", "MemoryGuard", "DiskGuard", "SAFETY", "Relocalization",
                "non-finite", "StaticMapEvidence", "CARGO_MONITOR",
                "CargoAlarmHeartbeat", "HookLoadState", "SAFETY_PENDING",
                "MotionGate", "NDT", "time rollback", "source stale")
        level = int(message.level)
        # ERROR/FATAL always saved
        if level >= int(message.ERROR):
            pass
        # WARN: only if tag matches
        elif not any(tag in text for tag in tags):
            # Count suppressed WARNs
            self._terminal_event_counts["WARN_suppressed"] += 1
            return
        event = {
            "wall_time": time.time(), "level": level,
            "name": message.name, "message": text,
            "tags": [t for t in tags if t in text],
        }
        self.writer.submit("jsonl", ("logs/ros_events.jsonl", event))
        self._terminal_event_counts["ERROR" if level >= int(message.ERROR) else "WARN_matched"] += 1

    def _update_motion_state(self, now: float) -> None:
        """Track motion episodes based on odom speed."""
        speed = self.current_pose.get("speed_mps", 0.0) if self.current_pose else 0.0
        if not math.isfinite(speed):
            return
        self._motion_speeds.append(speed)

        if self._motion_state in ("UNKNOWN", "STATIONARY"):
            if speed >= self._motion_enter_speed:
                if self._motion_enter_candidate is None:
                    self._motion_enter_candidate = now
                elif now - self._motion_enter_candidate >= self._motion_enter_confirm:
                    self._motion_state = "MOVING"
                    self._motion_episode_start = now
                    self._motion_episode_start_pose = dict(self.current_pose) if self.current_pose else None
                    self._motion_episode_count += 1
                    event = {
                        "event": "MOTION_ENTER",
                        "wall_time": now,
                        "episode_id": self._motion_episode_count,
                        "pose": self._motion_episode_start_pose,
                    }
                    self.writer.submit("jsonl", ("samples/motion_events.jsonl", event))
                    self._emit_event({"event": "MOTION_ENTER", "wall_time": now,
                                      "code": "-", "reason": f"speed={speed:.3f}",
                                      "obstacle_track_id": 0})
            else:
                self._motion_enter_candidate = None
                self._motion_state = "STATIONARY"
        elif self._motion_state == "MOVING":
            if speed < self._motion_exit_speed:
                if self._motion_exit_candidate is None:
                    self._motion_exit_candidate = now
                elif now - self._motion_exit_candidate >= self._motion_exit_confirm:
                    self._motion_state = "STATIONARY"
                    self._motion_exit_candidate = None
                    start_pose = self._motion_episode_start_pose
                    current_pose = dict(self.current_pose) if self.current_pose else None
                    displacement = None
                    if start_pose and current_pose:
                        displacement = math.sqrt(
                            (current_pose.get("x", 0) - start_pose.get("x", 0))**2 +
                            (current_pose.get("y", 0) - start_pose.get("y", 0))**2)
                    speeds = list(self._motion_speeds)
                    event = {
                        "event": "MOTION_EXIT",
                        "wall_time": now,
                        "episode_id": self._motion_episode_count,
                        "duration_sec": now - (self._motion_episode_start or now),
                        "displacement_m": displacement,
                        "max_speed_mps": max(speeds) if speeds else 0.0,
                        "p95_speed_mps": _percentile(speeds, 0.95) if speeds else 0.0,
                    }
                    self.writer.submit("jsonl", ("samples/motion_events.jsonl", event))
                    self._emit_event({"event": "MOTION_EXIT", "wall_time": now,
                                      "code": "-", "reason": f"displacement={displacement or 0:.2f}m",
                                      "obstacle_track_id": 0})
            else:
                self._motion_exit_candidate = None
        # Write motion sample CSV
        self.writer.submit("csv", ("samples/motion_samples.csv", [
            "wall_time", "speed_mps", "motion_state", "episode_id",
            "x", "y", "z", "yaw_deg"
        ], {
            "wall_time": now,
            "speed_mps": speed,
            "motion_state": self._motion_state,
            "episode_id": self._motion_episode_count,
            "x": self.current_pose.get("x") if self.current_pose else None,
            "y": self.current_pose.get("y") if self.current_pose else None,
            "z": self.current_pose.get("z") if self.current_pose else None,
            "yaw_deg": self.current_pose.get("yaw_deg") if self.current_pose else None,
        }))

    def _emit_event(self, event: Mapping[str, Any]) -> None:
        line = "[{event}] code={code} reason={reason} track={obstacle_track_id}".format(
            event=event.get("event", "EVENT"), code=event.get("code", "-"),
            reason=event.get("reason", "-"),
            obstacle_track_id=event.get("obstacle_track_id", 0))
        print(line, flush=True)
        self.writer.submit("text", ("logs/monitor.log", line))
        self.writer.submit("jsonl", ("samples/safety_events.jsonl", dict(event)))
        if event.get("event") != "SAFETY_REPEAT":
            self.last_safety_repeat_wall = float(event.get("wall_time", time.time()))

    def _sample_runtime(self, now: float) -> Dict[str, Any]:
        runtime = read_json(self.runtime_path)
        if runtime is not None:
            for key, label in (("memory_guard_triggered", "MEMORY_GUARD"),
                               ("disk_guard_triggered", "DISK_GUARD")):
                state = _as_bool(runtime.get(key, False))
                if key in self.last_guard_state and state != self.last_guard_state[key]:
                    self._emit_event({"event": label + ("_ENTER" if state else "_CLEAR"),
                                      "wall_time": now, "code": "-", "reason": key,
                                      "obstacle_track_id": 0})
                self.last_guard_state[key] = state
            epoch = int(runtime.get("static_evidence_epoch", 0) or 0)
            if self.last_static_epoch is not None and epoch != self.last_static_epoch:
                self._emit_event({"event": "STATIC_EPOCH_CHANGE", "wall_time": now,
                                  "code": "-", "reason": "{}->{}".format(
                                      self.last_static_epoch, epoch),
                                  "obstacle_track_id": 0})
            self.last_static_epoch = epoch
            self.latest_runtime = runtime
            try:
                self.last_runtime_mtime = self.runtime_path.stat().st_mtime
            except OSError:
                pass
            self.writer.submit("jsonl", ("snapshots/runtime_status.jsonl",
                                         {"wall_time": now, "status": runtime}))
        runtime_age = None if self.last_runtime_mtime is None else max(0.0, now - self.last_runtime_mtime)
        runtime_stale = is_runtime_status_stale(
            self.last_runtime_mtime, now,
            float(self.config.get("runtime_status_stale_sec", 5.0)))
        if now - self.last_filesystem_scan_wall >= max(
                5.0, float(self.config.get("summary_period_sec", 10.0))):
            self.filesystem_cache = self._scan_persistent_root()
            self.last_filesystem_scan_wall = now
            manifest_state = str(self.filesystem_cache.get("manifest_state", "MISSING"))
            if self.last_manifest_state is not None and manifest_state != self.last_manifest_state:
                self._emit_event({"event": "STATIC_MANIFEST_CHANGE", "wall_time": now,
                                  "code": "-", "reason": "{}->{}".format(
                                      self.last_manifest_state, manifest_state),
                                  "obstacle_track_id": 0})
            self.last_manifest_state = manifest_state
        if self.pid is None or not (Path("/proc") / str(self.pid)).exists():
            self.pid = find_process()
        process = process_metrics(self.pid)
        cpu_percent = None
        cpu_ticks = process.get("cpu_ticks")
        if cpu_ticks is not None and self.last_cpu_sample is not None:
            prior_wall, prior_ticks = self.last_cpu_sample
            elapsed = now - prior_wall
            if elapsed > 0.0:
                cpu_percent = (int(cpu_ticks) - prior_ticks) / float(
                    os.sysconf("SC_CLK_TCK")) / elapsed * 100.0
        if cpu_ticks is not None:
            self.last_cpu_sample = (now, int(cpu_ticks))
        start_ticks = process.get("process_start_ticks")
        if start_ticks is not None and self.last_process_start is not None and start_ticks != self.last_process_start:
            self.restart_count += 1
            self._emit_event({"event": "SLAM_NODE_RESTART", "wall_time": now,
                              "code": "-", "reason": "process_start_changed",
                              "obstacle_track_id": 0})
        if start_ticks is not None:
            self.last_process_start = int(start_ticks)
        try:
            disk = shutil.disk_usage(str(self.persistent_root))
            disk_free_gb = disk.free / (1024.0 ** 3)
        except OSError:
            disk_free_gb = None
        odom_age = None if self.last_odom_wall is None else now - self.last_odom_wall
        recent_odom = [value for value in self.odom_times if now - value <= 10.0]
        odom_hz = (len(recent_odom) - 1) / max(1.0e-6, recent_odom[-1] - recent_odom[0]) if len(recent_odom) > 1 else 0.0
        row: Dict[str, Any] = {
            "wall_time": now, "pid": process.get("pid"), "rss_mb": process.get("rss_mb"),
            "cpu_percent": cpu_percent,
            "threads": process.get("threads"), "fd_count": process.get("fd_count"),
            "restart_count": self.restart_count, "disk_free_gb": disk_free_gb,
            "runtime_status_age_sec": runtime_age, "odom_age_sec": odom_age,
            "runtime_status_stale": runtime_stale,
            "odom_hz": odom_hz, "pose_step_p50_m": _percentile(list(self.pose_steps), 0.50),
            "pose_step_p95_m": _percentile(list(self.pose_steps), 0.95),
            "pose_step_max_m": max(self.pose_steps) if self.pose_steps else None,
        }
        if self.current_pose:
            row.update(self.current_pose)
        row.update({"runtime_" + key: value for key, value in self.latest_runtime.items()
                    if isinstance(value, (str, int, float, bool))})
        row.update(self.filesystem_cache)
        self.writer.submit("csv", ("samples/runtime_samples.csv",
                                   RUNTIME_SAMPLE_FIELDS, row))
        self.writer.submit("csv", ("samples/localization_samples.csv", [
            "wall_time", "odom_hz", "odom_age_sec", "x", "y", "z", "yaw_deg",
            "speed_mps", "pose_step_p50_m", "pose_step_p95_m", "pose_step_max_m"], row))
        mapping = {key: value for key, value in row.items()
                   if key.startswith("runtime_")}
        mapping["wall_time"] = now
        mapping_fields = ["wall_time"] + [field for field in RUNTIME_SAMPLE_FIELDS
                                          if field.startswith("runtime_")]
        self.writer.submit("csv", ("samples/mapping_samples.csv",
                                   mapping_fields, mapping))
        return row

    def _scan_persistent_root(self) -> Dict[str, Any]:
        active = self.persistent_root / "static_evidence_manifest.json"
        last_good = self.persistent_root / "static_evidence_manifest.last_good.json"
        suspended = self.persistent_root / "static_evidence_manifest.suspended"
        if suspended.is_file():
            manifest_state = "SUSPENDED"
        elif active.is_file():
            manifest_state = "ACTIVE"
        elif last_good.is_file():
            manifest_state = "LAST_GOOD_ONLY"
        else:
            manifest_state = "FIRST_RUN"
        manifest_payload = read_json(active if active.is_file() else last_good)
        total_bytes = tile_files = tmp_files = 0
        newest_tile_mtime: Optional[float] = None
        try:
            for root, _directories, files in os.walk(str(self.persistent_root)):
                for name in files:
                    path = Path(root) / name
                    try:
                        stat = path.stat()
                    except OSError:
                        continue
                    total_bytes += stat.st_size
                    if name.endswith(".tmp"):
                        tmp_files += 1
                    if "tiles_" in root and name.endswith((".pcd", ".bin", ".csv")):
                        tile_files += 1
                        newest_tile_mtime = max(newest_tile_mtime or stat.st_mtime,
                                               stat.st_mtime)
        except OSError:
            pass
        return {"manifest_state": manifest_state,
                "active_manifest_present": active.is_file(),
                "last_good_manifest_present": last_good.is_file(),
                "suspension_marker_present": suspended.is_file(),
                "persistent_tmp_files": tmp_files,
                "persistent_tile_files": tile_files,
                "persistent_newest_tile_mtime": newest_tile_mtime,
                "persistent_size_mb": total_bytes / (1024.0 * 1024.0),
                "manifest_generation": manifest_payload.get("generation"),
                "manifest_revision": manifest_payload.get("revision"),
                "manifest_total_cells": manifest_payload.get("total_cells"),
                "manifest_mature_cells": manifest_payload.get("mature_cells")}

    def snapshot(self, now: Optional[float] = None) -> Dict[str, Any]:
        wall = time.time() if now is None else now
        self._update_motion_state(wall)
        runtime = self._sample_runtime(wall)
        with self.aggregator_lock:
            summary = self.aggregator.full_summary(now=wall)
        summary.update({"runtime": runtime, "writer_dropped": self.writer.dropped,
                        "restart_count": self.restart_count})
        self.writer.submit("atomic_json", ("reports/live_summary.json", summary))
        current = summary.get("current") or {}
        repeat_period = float(self.config.get("repeat_period_sec", 30.0))
        if (int(current.get("code", 0)) in HAZARD_CODES.union(FAULT_CODES) and
                wall - self.last_safety_repeat_wall >= repeat_period):
            self._emit_event({"event": "SAFETY_REPEAT", "wall_time": wall,
                              "code": current.get("code"),
                              "reason": current.get("reason"),
                              "obstacle_track_id": current.get("obstacle_track_id", 0)})
            self.last_safety_repeat_wall = wall
        return summary

    def run(self) -> None:
        sample_period = float(self.config.get("sample_period_sec", 1.0))
        summary_period = float(self.config.get("summary_period_sec", 10.0))
        rate = self.rospy.Rate(max(0.2, 1.0 / max(0.1, sample_period)))

        # ── readiness handshake ──
        ready_payload: Dict[str, Any] = {
            "ready": True,
            "pid": os.getpid(),
            "run_id": self.args.run_id,
            "boot_id": self._read_boot_id(),
            "ros_node": "/ndt_slam_server_monitor",
            "workspace_sha": self.args.expected_sha or "unknown",
            "created_at": time.time(),
            "topics": {},
        }
        try:
            import rosgraph
            master = rosgraph.Master(self.rospy.get_name())
            topic_types = master.getTopicTypes()
            for topic_name, topic_type in topic_types:
                if topic_name in ("/odom", "/cargo_avoidance/safety_status",
                                  "/cargo_avoidance/status_code",
                                  "/cargo_avoidance/static_evidence_debug"):
                    ready_payload["topics"][topic_name] = topic_type
        except Exception:
            pass
        atomic_write_json(self.run_dir / "reports" / "monitor_ready.json", ready_payload)

        while not self.rospy.is_shutdown() and not self.shutdown_requested:
            now = time.time()
            summary = self.snapshot(now)
            if now - self.last_summary_wall >= summary_period or self.snapshot_requested:
                current = summary.get("current") or {}
                window = summary.get("windows", {}).get("60", {})
                health_line = ("[SERVER_HEALTH] code={code} reason={reason} "
                               "odom={hz:.2f}Hz rss={rss}MB 60s={ratio}").format(
                          code=current.get("code", "-"), reason=current.get("reason", "-"),
                          hz=float(summary["runtime"].get("odom_hz") or 0.0),
                          rss=summary["runtime"].get("rss_mb"),
                          ratio=window.get("code_duration_ratio", {}))
                print(health_line, flush=True)
                self.writer.submit("text", ("logs/monitor.log", health_line))
                self.last_summary_wall = now
                self.snapshot_requested = False
            rate.sleep()
        final = self.snapshot(time.time())
        self.writer.submit("atomic_json", ("reports/final_summary.json", final))
        self.writer.close()


def create_run_manifest(run_dir: Path, args: argparse.Namespace) -> None:
    for relative in ("logs/preflight.log", "logs/build.log", "logs/tests.log",
                     "logs/monitor.log", "logs/slam_journal.log"):
        path = run_dir / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch(exist_ok=True)
    if (run_dir / "run_manifest.json").exists():
        return
    try:
        actual_sha = subprocess.run(
            ["git", "-C", str(Path(args.workspace).expanduser()),
             "rev-parse", "HEAD"], check=True, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        actual_sha = "unknown"
    payload = {
        "run_id": args.run_id,
        "created_at": time.time(),
        "expected_sha": args.expected_sha,
        "actual_sha": actual_sha,
        "workspace": str(Path(args.workspace).expanduser().resolve()),
        "persistent_root": str(Path(args.persistent_root).expanduser().resolve()),
        "monitor_read_only": True,
        "ubuntu_runtime_validation": "NOT_RUN",
    }
    atomic_write_json(run_dir / "run_manifest.json", payload)


def replay(path: Path, windows: Sequence[float]) -> Dict[str, Any]:
    aggregator = SafetyAggregator(windows)
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            if not line.strip():
                continue
            value = json.loads(line)
            aggregator.ingest(value, source_stamp=float(value["source_stamp"]),
                              wall_time=float(value.get("wall_time", value["source_stamp"])))
    return aggregator.full_summary(now=(aggregator.records[-1].wall_time if aggregator.records else time.time()))


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", default=os.environ.get("NDT_SLAM_WORKSPACE", str(Path.home() / "NDT-slam-ws")))
    parser.add_argument("--persistent-root", default=os.environ.get("NDT_SLAM_DATA_ROOT", ""))
    parser.add_argument("--run-dir", default="")
    parser.add_argument("--run-id", default=time.strftime("%Y%m%d_%H%M%S"))
    parser.add_argument("--config", default="")
    parser.add_argument("--expected-sha", default="")
    parser.add_argument("--lock-file", default="")
    parser.add_argument("--replay", default="")
    parser.add_argument("--windows", default="")
    parser.add_argument("--summary-sec", type=float, default=None)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    workspace = Path(args.workspace).expanduser().resolve()
    if not args.persistent_root:
        args.persistent_root = str(workspace / "maps" / "live" / "current")
    if not args.run_dir:
        args.run_dir = str(workspace / "server_runs" / args.run_id)
    config = load_config(Path(args.config).expanduser() if args.config else None)
    if args.windows:
        config["windows_sec"] = [float(value) for value in args.windows.split(",")]
    if args.summary_sec is not None:
        config["summary_period_sec"] = max(1.0, args.summary_sec)
    if args.replay:
        print(json.dumps(replay(Path(args.replay), config["windows_sec"]), indent=2, sort_keys=True))
        return 0

    run_dir = Path(args.run_dir).expanduser().resolve()
    create_run_manifest(run_dir, args)
    lock_path = Path(args.lock_file).expanduser() if args.lock_file else run_dir / "monitor.lock"
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    lock_stream = lock_path.open("a+")
    if fcntl is not None:
        try:
            fcntl.flock(lock_stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print("monitor already running: {}".format(lock_path), file=sys.stderr)
            return 3

    try:
        import rospy
    except ImportError:
        print("ROS Python modules unavailable; use --replay for offline mode", file=sys.stderr)
        return 4
    rospy.init_node("ndt_slam_server_monitor", anonymous=False, disable_signals=True)
    monitor = RosRuntimeMonitor(args, config)

    def stop_handler(_signum: int, _frame: Any) -> None:
        monitor.shutdown_requested = True

    def snapshot_handler(_signum: int, _frame: Any) -> None:
        monitor.snapshot_requested = True

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    if hasattr(signal, "SIGUSR1"):
        signal.signal(signal.SIGUSR1, snapshot_handler)
    monitor.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
