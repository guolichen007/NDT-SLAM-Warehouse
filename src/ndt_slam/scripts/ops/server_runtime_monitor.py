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
    "cpu_some_avg10", "memory_some_avg10", "memory_full_avg10",
    "io_some_avg10", "io_full_avg10",
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


class CargoMonitorGate:
    """Gravity-authoritative cargo episode gate, independent of ROS."""

    NO_MESSAGE = "NO_MESSAGE"
    INVALID_OR_STALE = "INVALID_OR_STALE"
    INHIBIT = "INHIBIT"
    EMPTY = "EMPTY"
    LOADED_ACTIVE = "LOADED_ACTIVE"

    def __init__(self) -> None:
        self.state = self.NO_MESSAGE
        self.episode_id = 0
        self.episode_started_wall: Optional[float] = None
        self.last_closed_episode: Optional[Dict[str, Any]] = None

    @property
    def active(self) -> bool:
        return self.state == self.LOADED_ACTIVE

    @staticmethod
    def classify(valid: bool, fresh: bool, state: int) -> str:
        if not valid or not fresh or state not in (1, 2, 3):
            return CargoMonitorGate.INVALID_OR_STALE
        if state == 1:
            return CargoMonitorGate.INHIBIT
        if state == 2:
            return CargoMonitorGate.EMPTY
        return CargoMonitorGate.LOADED_ACTIVE

    def update(self, *, valid: bool, fresh: bool, state: int,
               wall_time: float, source_stamp: float = 0.0,
               reason: str = "") -> List[Dict[str, Any]]:
        next_state = self.classify(valid, fresh, state)
        if next_state == self.state:
            return []
        previous = self.state
        events: List[Dict[str, Any]] = []
        if previous == self.LOADED_ACTIVE:
            duration = max(0.0, wall_time - (self.episode_started_wall or wall_time))
            event_name = ("GRAVITY_LOST_DURING_CARGO" if
                          next_state == self.INVALID_OR_STALE else
                          "CARGO_EPISODE_CLOSED")
            closed = {
                "event": event_name, "wall_time": wall_time,
                "source_stamp": source_stamp, "episode_id": self.episode_id,
                "duration_sec": duration, "next_state": next_state,
                "reason": reason or next_state.lower(),
            }
            self.last_closed_episode = dict(closed)
            events.append(closed)
            self.episode_started_wall = None
        self.state = next_state
        if next_state == self.LOADED_ACTIVE:
            self.episode_id += 1
            self.episode_started_wall = wall_time
            events.append({
                "event": "GRAVITY_LOADED", "wall_time": wall_time,
                "source_stamp": source_stamp, "episode_id": self.episode_id,
                "previous_state": previous, "reason": reason or "loaded",
            })
        elif previous != self.LOADED_ACTIVE:
            events.append({
                "event": ("CARGO_MONITOR_INACTIVE_NO_GRAVITY" if
                          next_state == self.INVALID_OR_STALE else
                          "CARGO_MONITOR_STATE_CHANGED"),
                "wall_time": wall_time, "source_stamp": source_stamp,
                "previous_state": previous, "state": next_state,
                "reason": reason or next_state.lower(),
            })
        return events


class AvoidancePipelineObserver:
    """Correlate the read-only raw/final/heartbeat safety pipeline."""

    def __init__(self, grace_sec: float = 0.4) -> None:
        self.grace_sec = max(0.0, float(grace_sec))
        self.values: Dict[str, Tuple[int, float]] = {}
        self.operational: Dict[str, Any] = {}
        self.pending: Dict[str, Any] = {}
        self._divergence_since: Dict[str, float] = {}

    def set_code(self, name: str, code: int, wall_time: float) -> None:
        self.values[name] = (int(code), float(wall_time))

    def set_operational(self, value: Mapping[str, Any], wall_time: float) -> None:
        self.operational = dict(value)
        self.operational["_wall_time"] = float(wall_time)

    def set_pending(self, value: Mapping[str, Any], wall_time: float) -> None:
        self.pending = dict(value)
        self.pending["_wall_time"] = float(wall_time)

    def _mismatch(self, name: str, left: str, right: str,
                  now: float) -> Tuple[bool, bool]:
        if left not in self.values or right not in self.values:
            self._divergence_since.pop(name, None)
            return False, False
        left_code, left_wall = self.values[left]
        right_code, right_wall = self.values[right]
        if left_code == right_code:
            self._divergence_since.pop(name, None)
            return False, False
        started = self._divergence_since.setdefault(
            name, max(left_wall, right_wall))
        return True, now - started > self.grace_sec

    def snapshot(self, now: float) -> Dict[str, Any]:
        code = lambda name: self.values.get(name, (None, 0.0))[0]
        raw_typed = code("raw_typed")
        raw_simple = code("raw_simple")
        final_typed = code("final_typed")
        heartbeat = code("heartbeat")
        operational_final = self.operational.get("operational_code")
        pending_code = self.pending.get("pending_provisional_status",
                                        self.pending.get("provisional_status"))
        pending_numeric: Optional[int]
        try:
            pending_numeric = int(pending_code)
        except (TypeError, ValueError):
            pending_numeric = {
                "NEAR_3M": 17, "NEAR_5M": 18, "CLEAR": 14,
            }.get(str(pending_code).upper())
        raw_mismatch, raw_mature = self._mismatch(
            "raw", "raw_typed", "raw_simple", now)
        final_mismatch, final_mature = self._mismatch(
            "final", "final_typed", "heartbeat", now)
        raw_final_mismatch, raw_final_mature = self._mismatch(
            "raw_final", "raw_typed", "final_typed", now)
        operational_mismatch = (
            operational_final is not None and final_typed is not None and
            int(operational_final) != final_typed)
        if operational_mismatch:
            operational_started = self._divergence_since.setdefault(
                "operational", max(
                    float(self.operational.get("_wall_time", now)),
                    self.values.get("final_typed", (0, now))[1]))
        else:
            self._divergence_since.pop("operational", None)
            operational_started = now
        result = {
            "wall_time": now, "raw_typed_code": raw_typed,
            "raw_simple_code": raw_simple, "final_typed_code": final_typed,
            "heartbeat_code": heartbeat,
            "operational_raw_code": self.operational.get("raw_code"),
            "operational_code": operational_final,
            "pending_provisional_status": pending_code,
            "pending_numeric_code": pending_numeric,
            "raw_typed_simple_mismatch": raw_mismatch and raw_mature,
            "final_typed_heartbeat_mismatch": final_mismatch and final_mature,
            "raw_final_transition_pending": (raw_final_mismatch and
                                              not raw_final_mature),
            "operational_final_mismatch": (operational_mismatch and
                now - operational_started > self.grace_sec),
            "pending_illegal_clear": pending_numeric == CLEAR_CODE,
            "normal_code35": final_typed == 35,
        }
        return result


def classify_cargo_geometry(value: Mapping[str, Any]) -> Tuple[bool, bool]:
    """Return (direct_measured, formal_operational) without conflating them."""
    def positive(name: str) -> bool:
        number = _finite(value.get(name), 0.0)
        return number > 0.0

    dimensions_valid = all(positive(name) for name in
                           ("length_m", "width_m", "height_m"))
    geometry_source = str(value.get("geometry_source", ""))
    track_state = str(value.get("track_state", ""))
    lock_state = str(value.get("lock_state", ""))
    direct = (
        geometry_source == "MEASURED" and
        _as_bool(value.get("authoritative")) and track_state == "LOCKED" and
        _as_bool(value.get("observation_valid")) and
        _as_bool(value.get("height_valid")) and
        int(value.get("support", 0) or 0) > 0 and
        int(value.get("points", 0) or 0) > 0 and
        positive("confidence") and dimensions_valid)
    formal_source = (geometry_source in
                     ("MEASURED", "FORMAL_FROZEN", "FROZEN", "LOST_HOLD") or
                     _as_bool(value.get("frozen")) or
                     _as_bool(value.get("effective_envelope_clear_authority")))
    vertical_source = str(value.get("vertical_source", ""))
    formal_vertical = vertical_source in (
        "DIRECT_TOP_FROZEN_THICKNESS", "LOCKED_OBB_POINT_SUPPORT",
        "MAP_DIFF_REVEALED_SUPPORT", "STATIC_ORIGIN_TOP_SUPPORT")
    bottom_z = _finite(value.get("bottom_z"))
    top_z = _finite(value.get("top_z"))
    formal = ((track_state == "LOCKED" or lock_state in
               ("LOCKED", "LOST_HOLD")) and dimensions_valid and
              _as_bool(value.get("height_valid")) and
              math.isfinite(bottom_z) and math.isfinite(top_z) and
              top_z > bottom_z and
              (direct or formal_source or formal_vertical))
    return direct, formal


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
        "gravity_monitor_stale_sec": 1.0,
        "avoidance_pipeline_grace_sec": 0.4,
        "writer_queue_size": 4096,
        # Root-level keys from server_monitor.yaml
        "motion_capture": {},
        "fallback_cargo_envelope": {},
        "operational_output": {},
        "map_health": {},
        "cargo_recognition_monitor": {},
        "cargo_swing_monitor": {},
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
        for key in ("motion_capture", "fallback_cargo_envelope",
                    "operational_output", "map_health",
                    "cargo_recognition_monitor", "cargo_swing_monitor"):
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


def read_pcd_header(path: Path) -> Optional[Dict[str, Any]]:
    """Read PCD header fields without loading point data.

    Returns dict with keys: points, width, height, fields, size, type, count,
    viewpoint, data_format, header_bytes, file_size.
    Returns None if not a valid PCD file.
    """
    try:
        file_size = path.stat().st_size
        with path.open("rb") as fh:
            header_bytes = 0
            info: Dict[str, Any] = {"file_size": file_size, "fields": []}
            for _ in range(120):
                line = fh.readline()
                header_bytes += len(line)
                text = line.decode("utf-8", "replace").strip()
                if text.startswith("FIELDS"):
                    info["fields"] = text.split()[1:]
                elif text.startswith("SIZE"):
                    info["size"] = [int(v) for v in text.split()[1:]]
                elif text.startswith("TYPE"):
                    info["type"] = text.split()[1:]
                elif text.startswith("COUNT"):
                    info["count"] = [int(v) for v in text.split()[1:]]
                elif text.startswith("WIDTH"):
                    info["width"] = int(text.split()[1])
                elif text.startswith("HEIGHT"):
                    info["height"] = int(text.split()[1])
                elif text.startswith("POINTS"):
                    info["points"] = int(text.split()[1])
                elif text.startswith("VIEWPOINT"):
                    parts = text.split()[1:]
                    if len(parts) >= 7:
                        info["viewpoint"] = [float(v) for v in parts[:7]]
                elif text.startswith("DATA"):
                    info["data_format"] = text.split()[1]
                    break
            info["header_bytes"] = header_bytes
            info["point_count"] = info.get("points", info.get("width", 0))
            # Must have at minimum a DATA field to be valid
            if "data_format" not in info:
                return None
            return info
    except (OSError, ValueError, IndexError):
        return None


def read_pcd_xyz_bounds(path: Path, max_sample: int = 5000,
                        z_below: Optional[float] = None,
                        z_above: Optional[float] = None) -> Optional[Dict[str, Any]]:
    """Quick sample of PCD XYZ data to get spatial bounds and Z outlier counts.

    Only reads first max_sample points for performance.
    Returns {x_min, x_max, y_min, y_max, z_min, z_max, z_outlier_below_count,
             z_outlier_above_count, points_sampled} or None.
    """
    header = read_pcd_header(path)
    if header is None:
        return None
    field_names = header.get("fields", [])
    try:
        xi = field_names.index("x")
        yi = field_names.index("y")
        zi = field_names.index("z")
    except ValueError:
        return None
    field_sizes = header.get("size", [4] * len(field_names))
    stride = sum(field_sizes)
    x_off = sum(field_sizes[:xi])
    y_off = sum(field_sizes[:yi])
    z_off = sum(field_sizes[:zi])
    fmt = {4: "f", 8: "d"}

    bounds: Dict[str, float] = {}
    z_outlier_below_count = 0
    z_outlier_above_count = 0
    points_sampled = 0
    try:
        with path.open("rb") as fh:
            fh.seek(header["header_bytes"])
            sample_count = min(max_sample, header.get("point_count", 0))
            for _ in range(sample_count):
                row = fh.read(stride)
                if len(row) < stride:
                    break
                import struct
                x = struct.unpack(fmt.get(field_sizes[xi], "f"), row[x_off:x_off + field_sizes[xi]])[0]
                y = struct.unpack(fmt.get(field_sizes[yi], "f"), row[y_off:y_off + field_sizes[yi]])[0]
                z = struct.unpack(fmt.get(field_sizes[zi], "f"), row[z_off:z_off + field_sizes[zi]])[0]
                if not all(math.isfinite(v) for v in (x, y, z)):
                    continue
                points_sampled += 1
                bounds["x_min"] = min(bounds.get("x_min", float("inf")), x)
                bounds["x_max"] = max(bounds.get("x_max", float("-inf")), x)
                bounds["y_min"] = min(bounds.get("y_min", float("inf")), y)
                bounds["y_max"] = max(bounds.get("y_max", float("-inf")), y)
                bounds["z_min"] = min(bounds.get("z_min", float("inf")), z)
                bounds["z_max"] = max(bounds.get("z_max", float("-inf")), z)
                # Count Z outliers during the same pass
                if z_below is not None and z < z_below:
                    z_outlier_below_count += 1
                if z_above is not None and z > z_above:
                    z_outlier_above_count += 1
    except (OSError, struct.error):
        return None
    if not bounds:
        return None
    bounds["z_outlier_below_count"] = z_outlier_below_count
    bounds["z_outlier_above_count"] = z_outlier_above_count
    bounds["points_sampled"] = points_sampled
    return bounds


def read_psi() -> Dict[str, Any]:
    """Read Linux Pressure Stall Information from /proc/pressure/*.

    Returns dict with cpu, memory, io pressure averages (avg10, avg60, avg300).
    Each metric has 'some' (share of time some tasks stalled) and
    'full' (share of time all tasks stalled, for memory/io only).
    Values are None if /proc/pressure is not available.
    """
    result: Dict[str, Any] = {
        "cpu_some_avg10": None, "cpu_some_avg60": None, "cpu_some_avg300": None,
        "memory_some_avg10": None, "memory_some_avg60": None, "memory_some_avg300": None,
        "memory_full_avg10": None, "memory_full_avg60": None, "memory_full_avg300": None,
        "io_some_avg10": None, "io_some_avg60": None, "io_some_avg300": None,
        "io_full_avg10": None, "io_full_avg60": None, "io_full_avg300": None,
    }
    for resource in ("cpu", "memory", "io"):
        pressure_path = Path("/proc/pressure") / resource
        if not pressure_path.is_file():
            continue
        try:
            text = pressure_path.read_text()
        except OSError:
            continue
        for line in text.strip().split("\n"):
            parts = line.split()
            if len(parts) < 2:
                continue
            stall_type = parts[0]  # "some" or "full"
            for token in parts[1:]:
                if "=" not in token:
                    continue
                key, val = token.split("=", 1)
                field = f"{resource}_{stall_type}_{key}"
                if field in result:
                    try:
                        result[field] = float(val)
                    except ValueError:
                        pass
    return result


def build_tile_catalog(persistent_root: Path) -> List[Dict[str, Any]]:
    """Build a catalog of all PCD tiles with header metadata.

    Reads header of every PCD file, collecting points/bytes/mtime per tile.
    Returns list of tile catalog entries.
    """
    import re
    _tile_coord_re = re.compile(r'^x(-?\d+)_y(-?\d+)')
    catalog: List[Dict[str, Any]] = []
    for layer in ("tiles_registration", "tiles_display", "tiles_ground", "tiles_objects"):
        layer_dir = persistent_root / layer
        if not layer_dir.is_dir():
            continue
        for pcd_path in sorted(layer_dir.glob("*.pcd")):
            entry: Dict[str, Any] = {
                "layer": layer,
                "path": str(pcd_path.relative_to(persistent_root)),
                "file_size": None,
                "mtime_ns": None,
                "points": None,
                "header_valid": False,
                "data_format": None,
            }
            try:
                stat = pcd_path.stat()
                entry["file_size"] = stat.st_size
                entry["mtime_ns"] = int(stat.st_mtime * 1e9)
            except OSError:
                pass
            header = read_pcd_header(pcd_path)
            if header:
                entry["points"] = header.get("point_count", 0)
                entry["header_valid"] = True
                entry["data_format"] = header.get("data_format")
            # Extract tile coordinates from filename (x-1_y-1.pcd or x0_y1.pcd)
            m = _tile_coord_re.match(pcd_path.stem)
            if m:
                entry["tile_x"] = int(m.group(1))
                entry["tile_y"] = int(m.group(2))
            catalog.append(entry)
    return catalog


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
        from lidar_slam2_msgs.msg import (
            CargoBottomEstimate,
            CargoRecognitionStatus,
            CargoSafetyStatus,
            CargoSwingStatus,
            HookLoadState,
        )
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
        self.odom_message_count: int = 0
        self.last_safety_wall: Optional[float] = None
        self.safety_status_message_count: int = 0
        self.last_status_code_wall: Optional[float] = None
        self.status_code_message_count: int = 0
        self.last_gravity_wall: Optional[float] = None
        self.gravity_message_count: int = 0
        self.last_recognition_wall: Optional[float] = None
        self.recognition_status_message_count: int = 0
        self.last_swing_wall: Optional[float] = None
        self.swing_status_message_count: int = 0
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
        self.shutdown_event = threading.Event()
        self.snapshot_requested = False
        self.last_guard_state: Dict[str, bool] = {}
        self.last_static_epoch: Optional[int] = None
        self.last_manifest_state: Optional[str] = None
        self.last_filesystem_scan_wall = 0.0
        self.filesystem_cache: Dict[str, Any] = {}
        self.last_cpu_sample: Optional[Tuple[float, int]] = None
        self.cargo_gate = CargoMonitorGate()
        self.pipeline = AvoidancePipelineObserver(float(config.get(
            "avoidance_pipeline_grace_sec", 0.4)))
        self._pipeline_last_flags: Dict[str, bool] = {}
        self.latest_gravity: Dict[str, Any] = {}
        self.latest_recognition: Dict[str, Any] = {}
        self.latest_swing: Dict[str, Any] = {}
        self.latest_geometry: Dict[str, Any] = {}
        self.latest_bottom: Dict[str, Any] = {}
        self.latest_safety: Dict[str, Any] = {}
        self._episode_geometry: List[Dict[str, Any]] = []
        self._episode_first_lock_wall: Optional[float] = None
        self._episode_recognition_failures_start = 0
        self._episode_recognition_messages_start = 0
        self._episode_swing_messages_start = 0
        self._geo_track_samples: Dict[int, int] = {}
        self._geo_track_episodes: Dict[int, str] = {}
        self._geo_last_authoritative: Dict[int, float] = {}

        # Typed recognition and suspended-motion observability. These values
        # are read-only mirrors; they never feed ROS parameters or control.
        self._recognition_state_counts: Counter[int] = Counter()
        self._sway_state_counts: Counter[int] = Counter()
        self._skew_state_counts: Counter[int] = Counter()
        self._torsion_state_counts: Counter[int] = Counter()
        self._last_recognition_state: Optional[int] = None
        self._last_sway_state: Optional[int] = None
        self._last_skew_state: Optional[int] = None
        self._last_torsion_state: Optional[int] = None
        self._last_recognition_source_stamp: Optional[float] = None
        self._last_swing_source_stamp: Optional[float] = None
        self._recognition_epoch = 0
        self._swing_epoch = 0
        self._recognition_duplicate_count = 0
        self._swing_duplicate_count = 0
        self._recognition_rollback_count = 0
        self._swing_rollback_count = 0
        self._last_swing_alarm_inhibited = False
        self._recognition_failure_count = 0
        self._recognition_failed_since: Optional[float] = None
        self._recognition_recovery_sec: Deque[float] = deque(maxlen=10_000)
        self._longest_loaded_without_lock_sec = 0.0
        self._swing_offsets: Deque[float] = deque(maxlen=200_000)
        self._swing_angles: Deque[float] = deque(maxlen=200_000)
        self._swing_speeds: Deque[float] = deque(maxlen=200_000)
        self._swing_yaw_errors: Deque[float] = deque(maxlen=200_000)
        self._longest_sway_warning_sec = 0.0
        self._longest_skew_suspected_sec = 0.0
        self._sway_alarm_count = 0
        self._skew_pull_alarm_count = 0
        self._skew_alarm_inhibited_count = 0
        self._torsion_alarm_count = 0
        self._swing_stale_count = 0
        self._recognition_stale_active = False
        self._swing_stale_active = False
        self._recognition_track_stats: Dict[str, Dict[str, Any]] = {}
        self._swing_track_stats: Dict[str, Dict[str, Any]] = {}
        self._last_loaded_without_lock_event_wall = 0.0
        self._last_sway_warning_event_wall = 0.0
        self._last_skew_suspected_event_wall = 0.0

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
        rospy.Subscriber("/hook/load_state", HookLoadState,
                         self._gravity_callback, queue_size=100)
        rospy.Subscriber("/cargo_avoidance/raw_safety_status",
                         CargoSafetyStatus, self._raw_safety_callback,
                         queue_size=100)
        rospy.Subscriber("/cargo_avoidance/raw_status_code", Int32,
                         self._raw_code_callback, queue_size=100)
        rospy.Subscriber("/cargo_avoidance/operational_status", String,
                         self._operational_callback, queue_size=50)
        rospy.Subscriber("/cargo_avoidance/pending_status", String,
                         self._pending_callback, queue_size=50)
        rospy.Subscriber("/cargo_avoidance/bottom_estimate",
                         CargoBottomEstimate, self._bottom_callback,
                         queue_size=50)
        rospy.Subscriber("/cargo_avoidance/static_evidence_debug", String,
                         self._static_callback, queue_size=20)
        rospy.Subscriber("/cargo_avoidance/cargo_geometry_debug", String,
                         self._cargo_geometry_callback, queue_size=50)
        rospy.Subscriber("/cargo_avoidance/recognition_status",
                         CargoRecognitionStatus,
                         self._recognition_callback, queue_size=50)
        rospy.Subscriber("/cargo_avoidance/swing_status", CargoSwingStatus,
                         self._swing_callback, queue_size=100)
        rospy.Subscriber("/rosout_agg", Log, self._rosout_callback, queue_size=200)

        # Map health tracking
        self._last_tile_files: Optional[int] = None
        self._last_tile_points: Optional[int] = None
        self._last_map_scan_result: Dict[str, Any] = {}
        self._map_health_events_suppressed: Dict[str, float] = {}
        self._maturity_starvation_started: Optional[float] = None
        self._metric_stuck_zero_started: Dict[str, Optional[float]] = {}
        self._last_ndt_time_zero: Optional[bool] = None  # None = not yet observed
        # Tile-level caching: key=tile_key, value=complete health analysis
        self._tile_health_cache: Dict[str, Dict[str, Any]] = {}
        # Readiness tracking
        self._start_requested_at: float = time.time()

    @staticmethod
    def _read_boot_id() -> str:
        try:
            return Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        except OSError:
            return "unknown"

    def _emit_typed_event(self, relative: str,
                          event: Mapping[str, Any]) -> None:
        payload = dict(event)
        payload.setdefault("wall_time", time.time())
        self.writer.submit("jsonl", (relative, payload))
        line = "[CARGO_EVENT] {event} reason={reason}".format(
            event=payload.get("event", "CARGO_EVENT"),
            reason=payload.get("reason", "-"))
        self.writer.submit("text", ("logs/monitor.log", line))
        self.writer.submit("text", ("logs/cargo_terminal.log", line))
        print(line, flush=True)

    def _gravity_callback(self, message: Any) -> None:
        wall = time.time()
        source_stamp = float(message.header.stamp.to_sec())
        row = {
            "wall_time": wall, "source_stamp": source_stamp,
            "valid": bool(message.valid), "fresh": bool(message.fresh),
            "state": int(message.state), "voltage": float(message.voltage),
            "stable_samples": int(message.stable_samples),
            "reason": str(message.reason),
            "message_age_sec": max(0.0, self.rospy.Time.now().to_sec() -
                                   source_stamp),
        }
        self.last_gravity_wall = wall
        self.gravity_message_count += 1
        events = self.cargo_gate.update(
            valid=row["valid"], fresh=row["fresh"], state=row["state"],
            wall_time=wall, source_stamp=source_stamp, reason=row["reason"])
        row["cargo_monitor_state"] = self.cargo_gate.state
        row["cargo_monitor_active"] = self.cargo_gate.active
        row["cargo_episode_id"] = self.cargo_gate.episode_id
        self.latest_gravity = dict(row)
        self.writer.submit("csv", ("samples/gravity_samples.csv",
                                   list(row.keys()), row))
        for event in events:
            self._emit_typed_event("samples/gravity_events.jsonl", event)
            self._emit_typed_event("samples/cargo_episode_events.jsonl", event)
            if event["event"] == "GRAVITY_LOADED":
                self._episode_geometry = []
                self._episode_first_lock_wall = None
                self._episode_recognition_failures_start = (
                    self._recognition_failure_count)
                self._episode_recognition_messages_start = (
                    self.recognition_status_message_count)
                self._episode_swing_messages_start = (
                    self.swing_status_message_count)
                self.latest_recognition = {}
                self.latest_swing = {}
                self.last_recognition_wall = None
                self.last_swing_wall = None
                self._last_recognition_source_stamp = None
                self._last_swing_source_stamp = None
                self._last_recognition_state = None
                self._last_sway_state = None
                self._last_skew_state = None
                self._last_torsion_state = None
                self._recognition_failed_since = None
                self._recognition_stale_active = False
                self._swing_stale_active = False
            elif event["event"] in (
                    "GRAVITY_LOST_DURING_CARGO", "CARGO_EPISODE_CLOSED"):
                self._write_cargo_episode_report(event)

    def _expire_gravity_if_needed(self, wall: float) -> None:
        stale_sec = float(self.config.get("gravity_monitor_stale_sec", 1.0))
        if (self.last_gravity_wall is None or
                wall - self.last_gravity_wall <= stale_sec or
                self.cargo_gate.state == CargoMonitorGate.INVALID_OR_STALE):
            return
        events = self.cargo_gate.update(
            valid=False, fresh=False, state=0, wall_time=wall,
            source_stamp=_finite(self.latest_gravity.get("source_stamp"), 0.0),
            reason="gravity_message_stale")
        self.latest_gravity["cargo_monitor_state"] = self.cargo_gate.state
        self.latest_gravity["cargo_monitor_active"] = False
        for event in events:
            self._emit_typed_event("samples/gravity_events.jsonl", event)
            self._emit_typed_event("samples/cargo_episode_events.jsonl", event)
            if event["event"] in (
                    "GRAVITY_LOST_DURING_CARGO", "CARGO_EPISODE_CLOSED"):
                self._write_cargo_episode_report(event)

    def _raw_safety_callback(self, message: Any) -> None:
        self.pipeline.set_code("raw_typed", int(message.requested_alarm_code),
                               time.time())

    def _raw_code_callback(self, message: Any) -> None:
        self.pipeline.set_code("raw_simple", int(message.data), time.time())

    @staticmethod
    def _parse_json_message(message: Any) -> Dict[str, Any]:
        try:
            value = json.loads(str(message.data))
        except (TypeError, ValueError):
            return {}
        return value if isinstance(value, dict) else {}

    def _operational_callback(self, message: Any) -> None:
        value = self._parse_json_message(message)
        if value:
            self.pipeline.set_operational(value, time.time())

    def _pending_callback(self, message: Any) -> None:
        value = self._parse_json_message(message)
        if value:
            self.pipeline.set_pending(value, time.time())

    def _bottom_callback(self, message: Any) -> None:
        if not self.cargo_gate.active:
            return
        self.latest_bottom = {
            "wall_time": time.time(),
            "source_stamp": float(message.header.stamp.to_sec()),
            "valid": bool(message.valid), "track_id": int(message.track_id),
            "track_state": int(message.track_state),
            "source": int(message.source),
            "source_detail": str(message.source_detail),
            "bottom_z_base": float(message.bottom_z_base),
            "top_z_base": float(message.top_z_base),
            "height_m": float(message.height_m),
            "uncertainty_m": float(message.uncertainty_m),
            "confidence": float(message.confidence),
        }

    def _accept_recognition_source_stamp(self, source_stamp: float,
                                         wall: float) -> bool:
        if not math.isfinite(source_stamp):
            return False
        previous = self._last_recognition_source_stamp
        if previous is not None:
            if abs(source_stamp - previous) <= 1.0e-9:
                self._recognition_duplicate_count += 1
                return False
            if source_stamp < previous:
                self._recognition_rollback_count += 1
                self._recognition_epoch += 1
                self._last_recognition_state = None
                self._recognition_failed_since = None
                self._recognition_stale_active = False
                self._emit_typed_event(
                    "samples/cargo_recognition_events.jsonl", {
                        "event": "CARGO_RECOGNITION_TIME_ROLLBACK",
                        "wall_time": wall,
                        "previous_source_stamp": previous,
                        "source_stamp": source_stamp,
                        "epoch": self._recognition_epoch,
                        "reason": "source_stamp_rollback",
                    })
        self._last_recognition_source_stamp = source_stamp
        return True

    def _accept_swing_source_stamp(self, source_stamp: float,
                                   wall: float) -> bool:
        if not math.isfinite(source_stamp):
            return False
        previous = self._last_swing_source_stamp
        if previous is not None:
            if abs(source_stamp - previous) <= 1.0e-9:
                self._swing_duplicate_count += 1
                return False
            if source_stamp < previous:
                self._swing_rollback_count += 1
                self._swing_epoch += 1
                self._last_sway_state = None
                self._last_skew_state = None
                self._last_torsion_state = None
                self._last_swing_alarm_inhibited = False
                self._swing_stale_active = False
                self._emit_typed_event(
                    "samples/cargo_swing_events.jsonl", {
                        "event": "CARGO_SWING_TIME_ROLLBACK",
                        "wall_time": wall,
                        "previous_source_stamp": previous,
                        "source_stamp": source_stamp,
                        "epoch": self._swing_epoch,
                        "reason": "source_stamp_rollback",
                    })
        self._last_swing_source_stamp = source_stamp
        return True

    def _recognition_callback(self, message: Any) -> None:
        if not self.cargo_gate.active:
            return
        wall = time.time()
        source_stamp = float(message.header.stamp.to_sec())
        if not self._accept_recognition_source_stamp(source_stamp, wall):
            return
        self.last_recognition_wall = wall
        self.recognition_status_message_count += 1
        state = int(message.state)
        row = {
            "wall_time": wall,
            "source_stamp": source_stamp,
            "source_epoch": self._recognition_epoch,
            "state": state,
            "valid": bool(message.valid),
            "hook_loaded": bool(message.hook_loaded),
            "lock_state": int(message.lock_state),
            "cargo_lifecycle_id": int(message.cargo_lifecycle_id),
            "track_segment_id": int(message.track_segment_id),
            "provisional_track_id": int(message.provisional_track_id),
            "formal_track_id": int(message.formal_track_id),
            "candidate_visible": bool(message.candidate_visible),
            "candidate_count": int(message.candidate_count),
            "candidate_progress": int(message.candidate_progress),
            "candidate_required": int(message.candidate_required),
            "recognition_age_sec": float(message.recognition_age_sec),
            "identity_confidence": float(message.identity_confidence),
            "shape_confidence": float(message.shape_confidence),
            "overall_confidence": float(message.overall_confidence),
            "detector_reason": str(message.detector_reason),
            "association_reason": str(message.association_reason),
            "vertical_reason": str(message.vertical_reason),
            "reason": str(message.reason),
        }
        fields = list(row.keys())
        self.latest_recognition = dict(row)
        self.writer.submit("csv", (
            "samples/cargo_recognition_samples.csv", fields, row))
        self._recognition_state_counts[state] += 1
        track_key = "{}:{}:{}".format(
            self._recognition_epoch, int(message.cargo_lifecycle_id),
            int(message.track_segment_id))
        track_stats = self._recognition_track_stats.setdefault(track_key, {
            "source_epoch": self._recognition_epoch,
            "cargo_lifecycle_id": int(message.cargo_lifecycle_id),
            "track_segment_id": int(message.track_segment_id),
            "samples": 0,
            "state_counts": {},
            "failure_count": 0,
            "maximum_recognition_age_sec": 0.0,
        })
        track_stats["samples"] += 1
        state_key = str(state)
        track_stats["state_counts"][state_key] = (
            int(track_stats["state_counts"].get(state_key, 0)) + 1)
        track_stats["maximum_recognition_age_sec"] = max(
            float(track_stats["maximum_recognition_age_sec"]),
            max(0.0, float(message.recognition_age_sec)))
        if bool(message.hook_loaded) and state != int(message.STATE_CARGO_LOCKED):
            self._longest_loaded_without_lock_sec = max(
                self._longest_loaded_without_lock_sec,
                max(0.0, float(message.recognition_age_sec)))
            monitor_cfg = self.config.get("cargo_recognition_monitor", {})
            event_after = float(monitor_cfg.get(
                "loaded_without_lock_event_sec", 5.0))
            repeat_after = float(monitor_cfg.get("repeat_period_sec", 30.0))
            if (float(message.recognition_age_sec) >= event_after and
                    wall - self._last_loaded_without_lock_event_wall >=
                    repeat_after):
                self._emit_typed_event(
                    "samples/cargo_recognition_events.jsonl", {
                        "event": "CARGO_LOADED_WITHOUT_LOCK",
                        "wall_time": wall,
                        "source_stamp": source_stamp,
                        "cargo_lifecycle_id": int(
                            message.cargo_lifecycle_id),
                        "track_segment_id": int(message.track_segment_id),
                        "recognition_age_sec": float(
                            message.recognition_age_sec),
                        "reason": str(message.reason),
                    })
                self._last_loaded_without_lock_event_wall = wall
        if self._last_recognition_state != state:
            self._emit_typed_event(
                "samples/cargo_recognition_events.jsonl", {
                    "event": "CARGO_RECOGNITION_STATE_CHANGE",
                    "wall_time": wall,
                    "source_stamp": source_stamp,
                    "previous_state": self._last_recognition_state,
                    "state": state,
                    "cargo_lifecycle_id": int(message.cargo_lifecycle_id),
                    "track_segment_id": int(message.track_segment_id),
                    "reason": str(message.reason),
                })
        if state == int(message.STATE_RECOGNITION_FAILED) and \
                self._last_recognition_state != state:
            self._recognition_failure_count += 1
            track_stats["failure_count"] += 1
            self._recognition_failed_since = wall
            self._emit_typed_event(
                "samples/cargo_recognition_events.jsonl", {
                    "event": "CARGO_RECOGNITION_FAILED_ENTER",
                    "wall_time": wall, "source_stamp": source_stamp,
                    "reason": str(message.reason)})
        elif self._last_recognition_state == int(
                message.STATE_RECOGNITION_FAILED) and state != int(
                    message.STATE_RECOGNITION_FAILED):
            if self._recognition_failed_since is not None:
                recovery = max(0.0, wall - self._recognition_failed_since)
                self._recognition_recovery_sec.append(recovery)
            self._recognition_failed_since = None
            self._emit_typed_event(
                "samples/cargo_recognition_events.jsonl", {
                    "event": "CARGO_RECOGNITION_RECOVERED",
                    "wall_time": wall, "source_stamp": source_stamp,
                    "reason": str(message.reason)})
        special_events = {
            int(message.STATE_GRAVITY_LIDAR_CONFLICT):
                "CARGO_GRAVITY_LIDAR_CONFLICT",
            int(message.STATE_CARGO_LOCKED): "CARGO_TRACK_LOCKED",
            int(message.STATE_CARGO_LOST_HOLD): "CARGO_TRACK_LOST_HOLD",
        }
        if state in special_events and self._last_recognition_state != state:
            if (state == int(message.STATE_CARGO_LOCKED) and
                    self._episode_first_lock_wall is None):
                self._episode_first_lock_wall = wall
            self._emit_typed_event(
                "samples/cargo_recognition_events.jsonl", {
                    "event": special_events[state], "wall_time": wall,
                    "source_stamp": source_stamp,
                    "reason": str(message.reason)})
        self._last_recognition_state = state
        self._recognition_stale_active = False

    def _swing_callback(self, message: Any) -> None:
        if not self.cargo_gate.active:
            return
        wall = time.time()
        source_stamp = float(message.header.stamp.to_sec())
        if not self._accept_swing_source_stamp(source_stamp, wall):
            return
        self.last_swing_wall = wall
        self.swing_status_message_count += 1
        sway = int(message.sway_state)
        skew = int(message.skew_pull_state)
        torsion = int(message.torsion_state)
        row = {
            "wall_time": wall,
            "source_stamp": source_stamp,
            "source_epoch": self._swing_epoch,
            "valid": bool(message.valid),
            "observation_state": int(message.observation_state),
            "sway_state": sway,
            "skew_pull_state": skew,
            "torsion_state": torsion,
            "cargo_lifecycle_id": int(message.cargo_lifecycle_id),
            "track_segment_id": int(message.track_segment_id),
            "track_id": int(message.track_id),
            "hook_anchor_source": str(message.hook_anchor_source),
            "hoist_state": int(message.hoist_state),
            "hoist_state_fresh": bool(message.hoist_state_fresh),
            "hoist_up_confirmed": bool(message.hoist_up_confirmed),
            "alarm_inhibited": bool(message.alarm_inhibited),
            "offset_x_m": float(message.offset_x_m),
            "offset_y_m": float(message.offset_y_m),
            "offset_m": float(message.offset_m),
            "rope_length_m": float(message.rope_length_m),
            "rope_length_source": int(getattr(
                message, "rope_length_source", 0)),
            "angle_authoritative": bool(getattr(
                message, "angle_authoritative", False)),
            "angle_deg": float(message.angle_deg),
            "horizontal_speed_mps": float(message.horizontal_speed_mps),
            "radial_speed_mps": float(message.radial_speed_mps),
            "oscillation_amplitude_m": float(
                message.oscillation_amplitude_m),
            "direction_consistency": float(message.direction_consistency),
            "zero_crossings": int(message.zero_crossings),
            "yaw_error_deg": float(message.yaw_error_deg),
            "observation_age_sec": float(message.observation_age_sec),
            "state_duration_sec": float(message.state_duration_sec),
            "sway_state_duration_sec": float(getattr(
                message, "sway_state_duration_sec",
                message.state_duration_sec)),
            "skew_state_duration_sec": float(getattr(
                message, "skew_state_duration_sec",
                message.state_duration_sec)),
            "torsion_state_duration_sec": float(getattr(
                message, "torsion_state_duration_sec",
                message.state_duration_sec)),
            "recommended_action": int(message.recommended_action),
            "reason": str(message.reason),
        }
        fields = list(row.keys())
        self.latest_swing = dict(row)
        self.writer.submit("csv", (
            "samples/cargo_swing_samples.csv", fields, row))
        self._sway_state_counts[sway] += 1
        self._skew_state_counts[skew] += 1
        self._torsion_state_counts[torsion] += 1
        track_key = "{}:{}:{}".format(
            self._swing_epoch, int(message.cargo_lifecycle_id),
            int(message.track_segment_id))
        track_stats = self._swing_track_stats.setdefault(track_key, {
            "source_epoch": self._swing_epoch,
            "cargo_lifecycle_id": int(message.cargo_lifecycle_id),
            "track_segment_id": int(message.track_segment_id),
            "track_id": int(message.track_id),
            "samples": 0,
            "sway_state_counts": {},
            "skew_pull_state_counts": {},
            "torsion_state_counts": {},
            "maximum_offset_m": 0.0,
            "maximum_angle_deg": 0.0,
            "maximum_horizontal_speed_mps": 0.0,
            "maximum_yaw_error_deg": 0.0,
            "longest_sway_warning_sec": 0.0,
            "longest_skew_suspected_sec": 0.0,
        })
        track_stats["samples"] += 1
        track_stats["track_id"] = int(message.track_id)
        for field, value in (("sway_state_counts", sway),
                             ("skew_pull_state_counts", skew),
                             ("torsion_state_counts", torsion)):
            value_key = str(value)
            track_stats[field][value_key] = (
                int(track_stats[field].get(value_key, 0)) + 1)
        if bool(message.valid):
            offset = abs(float(message.offset_m))
            angle = abs(float(message.angle_deg))
            speed = abs(float(message.horizontal_speed_mps))
            yaw_error = abs(float(message.yaw_error_deg))
            self._swing_offsets.append(offset)
            self._swing_angles.append(angle)
            self._swing_speeds.append(speed)
            self._swing_yaw_errors.append(yaw_error)
            track_stats["maximum_offset_m"] = max(
                float(track_stats["maximum_offset_m"]), offset)
            track_stats["maximum_angle_deg"] = max(
                float(track_stats["maximum_angle_deg"]), angle)
            track_stats["maximum_horizontal_speed_mps"] = max(
                float(track_stats["maximum_horizontal_speed_mps"]), speed)
            track_stats["maximum_yaw_error_deg"] = max(
                float(track_stats["maximum_yaw_error_deg"]), yaw_error)
        if sway in (int(message.SWAY_WARNING), int(message.SWAY_ALARM),
                    int(message.SWAY_SETTLING)):
            self._longest_sway_warning_sec = max(
                self._longest_sway_warning_sec,
                max(0.0, row["sway_state_duration_sec"]))
            track_stats["longest_sway_warning_sec"] = max(
                float(track_stats["longest_sway_warning_sec"]),
                max(0.0, row["sway_state_duration_sec"]))
        if skew in (int(message.SKEW_PULL_SUSPECTED),
                    int(message.SKEW_PULL_ALARM)):
            self._longest_skew_suspected_sec = max(
                self._longest_skew_suspected_sec,
                max(0.0, row["skew_state_duration_sec"]))
            track_stats["longest_skew_suspected_sec"] = max(
                float(track_stats["longest_skew_suspected_sec"]),
                max(0.0, row["skew_state_duration_sec"]))
        sway_events = {
            int(message.SWAY_DETECTED): "CARGO_SWAY_DETECTED_ENTER",
            int(message.SWAY_WARNING): "CARGO_SWAY_WARNING_ENTER",
            int(message.SWAY_ALARM): "CARGO_SWAY_ALARM_ENTER",
            int(message.SWAY_SETTLING): "CARGO_SWAY_SETTLING",
            int(message.SWAY_NORMAL): "CARGO_SWAY_CLEAR",
        }
        if sway != self._last_sway_state and sway in sway_events:
            self._emit_typed_event("samples/cargo_swing_events.jsonl", {
                "event": sway_events[sway], "wall_time": wall,
                "source_stamp": source_stamp, "reason": str(message.reason)})
            if sway == int(message.SWAY_ALARM):
                self._sway_alarm_count += 1
        skew_events = {
            int(message.SKEW_PULL_SUSPECTED): "CARGO_SKEW_PULL_SUSPECTED",
            int(message.SKEW_PULL_ALARM): "CARGO_SKEW_PULL_ALARM",
        }
        if skew != self._last_skew_state and skew in skew_events:
            self._emit_typed_event("samples/cargo_swing_events.jsonl", {
                "event": skew_events[skew], "wall_time": wall,
                "source_stamp": source_stamp, "reason": str(message.reason)})
            if skew == int(message.SKEW_PULL_ALARM):
                self._skew_pull_alarm_count += 1
        if bool(message.alarm_inhibited) and not self._last_swing_alarm_inhibited:
            self._skew_alarm_inhibited_count += 1
            self._emit_typed_event("samples/cargo_swing_events.jsonl", {
                "event": "CARGO_SKEW_PULL_ALARM_INHIBITED",
                "wall_time": wall, "source_stamp": source_stamp,
                "reason": str(message.reason)})
        torsion_events = {
            int(message.TORSION_WARNING): "CARGO_TORSION_WARNING",
            int(message.TORSION_ALARM): "CARGO_TORSION_ALARM",
        }
        if torsion != self._last_torsion_state and torsion in torsion_events:
            self._emit_typed_event("samples/cargo_swing_events.jsonl", {
                "event": torsion_events[torsion], "wall_time": wall,
                "source_stamp": source_stamp, "reason": str(message.reason)})
            if torsion == int(message.TORSION_ALARM):
                self._torsion_alarm_count += 1
        if int(message.observation_state) == int(message.OBSERVATION_TRACK_CHANGED):
            self._emit_typed_event("samples/cargo_swing_events.jsonl", {
                "event": "CARGO_SWING_TRACK_RESET", "wall_time": wall,
                "source_stamp": source_stamp, "reason": str(message.reason)})
        monitor_cfg = self.config.get("cargo_swing_monitor", {})
        repeat_after = float(monitor_cfg.get("repeat_period_sec", 10.0))
        if (sway in (int(message.SWAY_WARNING), int(message.SWAY_ALARM)) and
                row["sway_state_duration_sec"] >= float(monitor_cfg.get(
                    "sustained_warning_event_sec", 1.0)) and
                wall - self._last_sway_warning_event_wall >= repeat_after):
            self._emit_typed_event("samples/cargo_swing_events.jsonl", {
                "event": "CARGO_SWAY_WARNING_SUSTAINED",
                "wall_time": wall, "source_stamp": source_stamp,
                "state_duration_sec": row["sway_state_duration_sec"],
                "reason": str(message.reason)})
            self._last_sway_warning_event_wall = wall
        if (skew in (int(message.SKEW_PULL_SUSPECTED),
                     int(message.SKEW_PULL_ALARM)) and
                row["skew_state_duration_sec"] >= float(monitor_cfg.get(
                    "sustained_suspected_event_sec", 1.0)) and
                wall - self._last_skew_suspected_event_wall >= repeat_after):
            self._emit_typed_event("samples/cargo_swing_events.jsonl", {
                "event": "CARGO_SKEW_PULL_SUSPECTED_SUSTAINED",
                "wall_time": wall, "source_stamp": source_stamp,
                "state_duration_sec": row["skew_state_duration_sec"],
                "reason": str(message.reason)})
            self._last_skew_suspected_event_wall = wall
        self._last_sway_state = sway
        self._last_skew_state = skew
        self._last_torsion_state = torsion
        self._last_swing_alarm_inhibited = bool(message.alarm_inhibited)
        self._swing_stale_active = False

    def _cargo_geometry_callback(self, message: Any) -> None:
        """Archive direct and formal geometry as separate evidence classes."""
        if not self.cargo_gate.active:
            return
        wall = time.time()
        text = str(message.data)
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            return
        if not isinstance(parsed, dict):
            return
        parsed["wall_time"] = wall
        parsed["cargo_episode_id"] = self.cargo_gate.episode_id
        direct_measured, formal_operational = classify_cargo_geometry(parsed)
        parsed["direct_measured"] = direct_measured
        parsed["formal_operational"] = formal_operational
        parsed["frozen"] = (not direct_measured and formal_operational)
        self.latest_geometry = dict(parsed)

        csv_fields = [
            "wall_time", "cargo_episode_id", "stamp", "track_id",
            "track_state", "lock_state",
            "geometry_source", "authoritative", "observation_valid", "height_valid",
            "points", "support", "confidence",
            "center_x", "center_y", "center_z",
            "length_m", "width_m", "height_m", "yaw_deg",
            "bottom_z", "top_z", "vertical_source", "failure_reason",
            "hook_load_state", "gravity_voltage", "gravity_age_sec",
            "fallback_active", "direct_measured", "formal_operational",
            "frozen", "pose_evidence_age_sec", "height_evidence_age_sec",
            "horizontal_uncertainty", "vertical_uncertainty"
        ]
        row = {field: parsed.get(field, "") for field in csv_fields}
        self.writer.submit("csv", ("samples/cargo_geometry_all.csv",
                                   csv_fields, row))
        if direct_measured:
            self.writer.submit("csv", (
                "samples/cargo_geometry_direct_measured.csv",
                csv_fields, row))
        if formal_operational:
            self.writer.submit("csv", (
                "samples/cargo_geometry_formal_operational.csv",
                csv_fields, row))
        self._episode_geometry.append(dict(parsed))
        track_id = int(parsed.get("track_id", 0))
        if formal_operational and track_id > 0:
            self._geo_track_samples[track_id] = self._geo_track_samples.get(track_id, 0) + 1

    def _cargo_episode_summary(self, closed: Mapping[str, Any]) -> Dict[str, Any]:
        samples = list(self._episode_geometry)
        formal = [value for value in samples
                  if _as_bool(value.get("formal_operational"))]
        direct = [value for value in samples
                  if _as_bool(value.get("direct_measured"))]

        def range_summary(name: str) -> Dict[str, Any]:
            values = [_finite(item.get(name)) for item in formal]
            values = [value for value in values if math.isfinite(value)]
            return {"median": _percentile(values, 0.5),
                    "min": min(values) if values else None,
                    "max": max(values) if values else None}

        maximum_jump = 0.0
        previous: Optional[Tuple[float, float, float]] = None
        for item in formal:
            current = tuple(_finite(item.get(name)) for name in
                            ("length_m", "width_m", "height_m"))
            if previous is not None and all(math.isfinite(v) for v in current):
                maximum_jump = max(maximum_jump, max(
                    abs(a - b) for a, b in zip(current, previous)))
            if all(math.isfinite(v) for v in current):
                previous = current
        sources = Counter(str(item.get("geometry_source", "UNKNOWN"))
                          for item in formal)
        ages = [_finite(item.get(name)) for item in formal for name in
                ("pose_evidence_age_sec", "height_evidence_age_sec")]
        uncertainty = [_finite(item.get(name)) for item in formal for name in
                       ("horizontal_uncertainty", "vertical_uncertainty")]
        episode_start_wall = (_finite(closed.get("wall_time"), 0.0) -
                              _finite(closed.get("duration_sec"), 0.0))
        return {
            "episode_id": int(closed.get("episode_id", self.cargo_gate.episode_id)),
            "closed_event": closed.get("event"),
            "closed_wall_time": closed.get("wall_time"),
            "duration_sec": closed.get("duration_sec"),
            "geometry_samples": len(samples),
            "direct_measured_samples": len(direct),
            "formal_geometry_samples": len(formal),
            "formal_geometry_coverage_ratio": len(formal) / max(1, len(samples)),
            "first_lock_sec": (max(0.0, self._episode_first_lock_wall -
                                   episode_start_wall)
                               if self._episode_first_lock_wall is not None
                               else None),
            "recognition_failure_count": max(
                0, self._recognition_failure_count -
                self._episode_recognition_failures_start),
            "length_m": range_summary("length_m"),
            "width_m": range_summary("width_m"),
            "height_m": range_summary("height_m"),
            "bottom_z": range_summary("bottom_z"),
            "top_z": range_summary("top_z"),
            "maximum_dimension_jump_m": maximum_jump,
            "source_distribution": dict(sources),
            "maximum_evidence_age_sec": max(
                [value for value in ages if math.isfinite(value)], default=None),
            "maximum_uncertainty_m": max(
                [value for value in uncertainty if math.isfinite(value)],
                default=None),
        }

    def _write_cargo_episode_report(self, closed: Mapping[str, Any]) -> None:
        report = self._cargo_episode_summary(closed)
        self.writer.submit("atomic_json", (
            "reports/cargo_episode_latest.json", report))
        self.writer.submit("jsonl", (
            "reports/cargo_episode_summary.jsonl", report))
        self.writer.submit("atomic_json", (
            "reports/cargo_episode_summary.json", report))

    def _record_pipeline_snapshot(self, now: float) -> Dict[str, Any]:
        row = self.pipeline.snapshot(now)
        row["normal_code35"] = (self.cargo_gate.active and
                                bool(row.get("normal_code35")))
        formal_clear = (
            _as_bool(self.latest_geometry.get(
                "effective_envelope_clear_authority", False)) and
            _as_bool(self.latest_static_debug.get("authorized", False)) and
            _as_bool(self.latest_safety.get("cargo_valid", False)) and
            _as_bool(self.latest_safety.get("obstacle_valid", False)))
        row["final_illegal_clear"] = (
            self.cargo_gate.active and row.get("final_typed_code") == CLEAR_CODE
            and not formal_clear)
        row["cargo_monitor_active"] = self.cargo_gate.active
        row["cargo_episode_id"] = self.cargo_gate.episode_id
        fields = list(row.keys())
        self.writer.submit("csv", (
            "samples/avoidance_pipeline_samples.csv", fields, row))
        for flag in ("raw_typed_simple_mismatch",
                     "final_typed_heartbeat_mismatch",
                     "operational_final_mismatch", "pending_illegal_clear",
                     "final_illegal_clear", "normal_code35"):
            active = bool(row.get(flag))
            if active and not self._pipeline_last_flags.get(flag, False):
                event = dict(row)
                event.update({"event": "AVOIDANCE_PIPELINE_" + flag.upper(),
                              "reason": flag})
                self._emit_typed_event(
                    "samples/avoidance_pipeline_events.jsonl", event)
            self._pipeline_last_flags[flag] = active
        return row

    def _cargo_terminal_line(self, now: float,
                             pipeline: Mapping[str, Any]) -> str:
        gravity = self.latest_gravity
        geometry = self.latest_geometry
        recognition = self.latest_recognition
        swing = self.latest_swing
        current = self.aggregator.current_summary()
        gravity_age = (now - float(gravity.get("wall_time", now))
                       if gravity else float("nan"))
        return ("[CARGO_RT] episode={episode} gravity={gravity} "
                "gravity_age={gravity_age:.2f} recognition={recognition} "
                "lifecycle={lifecycle} segment={segment} track={track} "
                "box={length}x{width}x{height}m bottom={bottom} top={top} "
                "geometry_source={geometry_source} vertical_source={vertical} "
                "raw={raw} final={final} heartbeat={heartbeat} "
                "distance={distance} clearance={clearance} "
                "obstacle_track={obstacle_track} provenance={provenance} "
                "sway={sway} skew={skew} torsion={torsion} "
                "offset={offset} angle={angle} speed={speed} "
                "hoist={hoist} hoist_fresh={hoist_fresh} "
                "static={static} session_verified={session_verified} "
                "reason={reason}").format(
                    episode=self.cargo_gate.episode_id,
                    gravity=self.cargo_gate.state,
                    gravity_age=gravity_age,
                    recognition=recognition.get("state", "-"),
                    lifecycle=recognition.get("cargo_lifecycle_id", "-"),
                    segment=recognition.get("track_segment_id", "-"),
                    track=geometry.get("track_id", "-"),
                    length=geometry.get("length_m", "-"),
                    width=geometry.get("width_m", "-"),
                    height=geometry.get("height_m", "-"),
                    bottom=geometry.get("bottom_z", "-"),
                    top=geometry.get("top_z", "-"),
                    geometry_source=geometry.get("geometry_source", "-"),
                    vertical=geometry.get("vertical_source", "-"),
                    raw=pipeline.get("raw_typed_code"),
                    final=pipeline.get("final_typed_code"),
                    heartbeat=pipeline.get("heartbeat_code"),
                    distance=current.get("distance_m"),
                    clearance=current.get("clearance_m"),
                    obstacle_track=current.get("obstacle_track_id"),
                    provenance=current.get("provenance"),
                    sway=swing.get("sway_state", "-"),
                    skew=swing.get("skew_pull_state", "-"),
                    torsion=swing.get("torsion_state", "-"),
                    offset=swing.get("offset_m", "-"),
                    angle=swing.get("angle_deg", "-"),
                    speed=swing.get("horizontal_speed_mps", "-"),
                    hoist=swing.get("hoist_state", "-"),
                    hoist_fresh=swing.get("hoist_state_fresh", "-"),
                    static=self.pipeline.pending.get(
                        "static_authority",
                        self.latest_static_debug.get("authorized", "-")),
                    session_verified=self.pipeline.pending.get(
                        "map_session_verified", "-"),
                    reason=current.get("reason", "-"))

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
        self.odom_message_count += 1
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
        self.pipeline.set_code("final_typed",
                               int(message.requested_alarm_code), wall)
        self.last_safety_wall = wall
        self.safety_status_message_count += 1
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
                    "cargo_valid": bool(message.cargo_valid),
                    "obstacle_valid": bool(message.obstacle_valid),
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
        self.latest_safety = dict(row)
        self.writer.submit("csv", ("samples/safety_samples.csv", fields, row))

    def _code_callback(self, message: Any) -> None:
        wall = time.time()
        self.pipeline.set_code("heartbeat", int(message.data), wall)
        self.last_status_code_wall = wall
        self.status_code_message_count += 1
        with self.aggregator_lock:
            event = self.aggregator.check_status_code(
                int(message.data), wall_time=wall)
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

    def _check_runtime_invariants(self, runtime: Dict[str, Any], now: float) -> None:
        """Emit events for runtime invariant violations."""
        map_health_cfg = self.config.get("map_health", {})

        # 1. stationary=true but stationary_frame_count==0
        is_stationary = runtime.get("is_stationary", False)
        stationary_frames = int(runtime.get("stationary_frame_count", 0) or 0)
        if is_stationary and stationary_frames == 0:
            self._throttled_emit("RUNTIME_INVARIANT_VIOLATION", now, 60.0, {
                "detail": "stationary_true_but_frame_count_zero",
                "is_stationary": True, "stationary_frame_count": 0,
            })

        # 2. static maturity starvation
        mature = int(runtime.get("static_evidence_mature_cells", 0) or 0)
        cells = int(runtime.get("static_evidence_cells", 0) or 0)
        confirmed = int(runtime.get("static_cells_confirmed", 0) or 0)
        revision = int(runtime.get("static_evidence_revision", 0) or 0)
        min_cells = map_health_cfg.get("min_mature_required_cells", 100)
        starvation_sec = map_health_cfg.get("maturity_starvation_sec", 60.0)
        if mature == 0 and cells >= min_cells and confirmed > 0:
            if self._maturity_starvation_started is None:
                self._maturity_starvation_started = now
            elif now - self._maturity_starvation_started >= starvation_sec:
                self._throttled_emit("STATIC_MATURITY_STARVATION", now, 120.0, {
                    "detail": "mature=0 despite confirmed cells",
                    "cells": cells, "mature_cells": mature,
                    "confirmed": confirmed, "revision": revision,
                    "duration_sec": now - self._maturity_starvation_started,
                })
        else:
            self._maturity_starvation_started = None

        # 3. metric stuck-zero: ndt_time
        ndt_time = float(runtime.get("average_ndt_time_ms", 0) or 0)
        stuck_dur = map_health_cfg.get("metric_stuck_zero_duration_sec", 300.0)
        if ndt_time == 0.0:
            if not self._last_ndt_time_zero:
                self._metric_stuck_zero_started["ndt_time"] = now
            elif now - self._metric_stuck_zero_started.get("ndt_time", now) >= stuck_dur:
                self._throttled_emit("METRIC_STUCK_ZERO", now, 300.0, {
                    "detail": "average_ndt_time_ms=0 for extended period",
                    "duration_sec": now - self._metric_stuck_zero_started.get("ndt_time", now),
                })
            self._last_ndt_time_zero = True
        else:
            self._last_ndt_time_zero = False
            self._metric_stuck_zero_started["ndt_time"] = None

        # 4. track churn
        created = int(runtime.get("obstacle_track_created_count", 0) or 0)
        reset = int(runtime.get("obstacle_track_reset_count", 0) or 0)
        max_ratio = map_health_cfg.get("max_track_reset_ratio", 0.5)
        if created >= 10 and reset / max(1, created) > max_ratio:
            self._throttled_emit("TRACK_CHURN_HIGH", now, 120.0, {
                "detail": f"reset/created={reset}/{created}={reset / max(1, created):.3f}",
                "track_created": created, "track_reset": reset,
                "churn_ratio": reset / max(1, created),
            })

    def _emit_map_health_events(self, now: float) -> None:
        """Emit throttled events based on map scan results."""
        result = self._last_map_scan_result
        if not result:
            return

        # TILES_ACTIVE_NO_MANIFEST
        if result.get("manifest_state") == "TILES_ACTIVE_NO_MANIFEST":
            self._throttled_emit("TILES_ACTIVE_NO_MANIFEST", now, 300.0, {
                "detail": f"{result.get('persistent_tile_files', 0)} tiles, no manifest",
                "tile_count": result.get("persistent_tile_files", 0),
            })

        # Map layer ratios
        for alert in result.get("ratio_alerts", []):
            self._throttled_emit(f"MAP_LAYER_RATIO_ANOMALY", now, 300.0, {
                "detail": alert,
            })

        # Out of approved bounds
        for tile_name in result.get("tiles_out_of_bounds", []):
            self._throttled_emit("OUT_OF_APPROVED_MAP_BOUNDS", now, 300.0, {
                "detail": tile_name,
            })

        # Z outliers
        for zt in result.get("z_outlier_tiles", []):
            self._throttled_emit("MAP_Z_OUTLIER", now, 300.0, {
                "detail": f"{zt['tile']} z=[{zt.get('z_min')}, {zt.get('z_max')}]",
            })

        # Tile growth rate
        max_growth = self.config.get("map_health", {}).get("max_tile_growth_points_per_hour", 500000)
        growth = result.get("tile_growth_points_per_hour", 0.0)
        if growth > max_growth:
            self._throttled_emit("MAP_TILE_GROWTH_HIGH", now, 600.0, {
                "detail": f"{growth:.0f} pts/hour > {max_growth}",
            })

    def _throttled_emit(self, event_type: str, now: float, interval: float,
                         payload: Dict[str, Any]) -> None:
        """Emit event if not suppressed (throttled per event type + detail key)."""
        key = f"{event_type}:{payload.get('detail', '')}"
        last = self._map_health_events_suppressed.get(key, 0.0)
        if now - last < interval:
            return
        self._map_health_events_suppressed[key] = now
        self._emit_event({
            "event": event_type,
            "wall_time": now,
            "code": "-",
            "reason": payload.get("detail", ""),
            "obstacle_track_id": 0,
        })
        self.writer.submit("jsonl", ("logs/map_health_events.jsonl", {
            "event": event_type, "wall_time": now, **payload,
        }))

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

            # ── runtime invariant checks ──
            self._check_runtime_invariants(runtime, now)
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

            # Emit map health events (throttled per event type)
            self._emit_map_health_events(now)
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
        # ── PSI (Pressure Stall Information) ──
        psi = read_psi()
        for psi_key in ("cpu_some_avg10", "memory_some_avg10", "memory_full_avg10",
                         "io_some_avg10", "io_full_avg10"):
            if psi_key in psi:
                row[psi_key] = psi[psi_key]
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

        # ── manifest state detection ──
        has_tiles = False
        tile_layers_present: List[str] = []
        for layer in ("tiles_registration", "tiles_display", "tiles_ground", "tiles_objects"):
            layer_dir = self.persistent_root / layer
            if layer_dir.is_dir() and any(layer_dir.glob("*.pcd")):
                has_tiles = True
                tile_layers_present.append(layer)

        manifest_payload = None
        if suspended.is_file():
            manifest_state = "SUSPENDED"
        elif active.is_file():
            manifest_payload = read_json(active)
            if manifest_payload is None:
                manifest_state = "MANIFEST_CORRUPT"
            else:
                manifest_state = "ACTIVE"
        elif last_good.is_file():
            manifest_payload = read_json(last_good)
            if manifest_payload is None:
                manifest_state = "MANIFEST_CORRUPT"
            else:
                manifest_state = "LAST_GOOD_ONLY"
        elif has_tiles:
            # Check if tiles are still being written (recent mtime within last 30s)
            now_ts = time.time()
            building = False
            for layer in tile_layers_present:
                layer_dir = self.persistent_root / layer
                for pcd_path in layer_dir.glob("*.pcd"):
                    try:
                        if now_ts - pcd_path.stat().st_mtime < 30.0:
                            building = True
                            break
                    except OSError:
                        pass
                if building:
                    break
            if building:
                manifest_state = "TILES_BUILDING"
            else:
                manifest_state = "TILES_ACTIVE_NO_MANIFEST"
        else:
            manifest_state = "EMPTY_FIRST_RUN"

        # ── per-layer tile audit ──
        map_health_cfg = self.config.get("map_health", {})
        approved = map_health_cfg.get("approved_bounds", {})
        layer_stats: Dict[str, Dict[str, Any]] = {}
        total_tile_files = 0
        total_tile_bytes = 0
        total_tile_points = 0
        newest_tile_mtime: Optional[float] = None
        tmp_files = 0
        tiles_out_of_bounds: List[str] = []
        z_outlier_tiles: List[Dict[str, Any]] = []
        total_z_below = 0
        total_z_above = 0

        z_below = map_health_cfg.get("z_outlier_below_m", -4.0)
        z_above = map_health_cfg.get("z_outlier_above_m", 10.0)

        # Build tile catalog with bounds sampling
        tile_catalog: List[Dict[str, Any]] = []
        changed_tiles: List[Dict[str, Any]] = []
        # Regex for tile filenames: x-1_y-1.pcd or x0_y1.pcd
        import re
        _tile_coord_re = re.compile(r'^x(-?\d+)_y(-?\d+)')

        for layer in ("tiles_registration", "tiles_display", "tiles_ground", "tiles_objects"):
            layer_dir = self.persistent_root / layer
            layer_points = 0
            layer_bytes = 0
            layer_count = 0
            if layer_dir.is_dir():
                for pcd_path in sorted(layer_dir.glob("*.pcd")):
                    layer_count += 1
                    stat_mtime = None
                    stat_size = 0
                    try:
                        stat = pcd_path.stat()
                        layer_bytes += stat.st_size
                        stat_mtime = stat.st_mtime
                        stat_size = stat.st_size
                        newest_tile_mtime = max(newest_tile_mtime or stat.st_mtime, stat.st_mtime)
                    except OSError:
                        pass
                    # Quick header scan for point count
                    header = read_pcd_header(pcd_path)
                    tile_key = f"{layer}/{pcd_path.name}"

                    # ── tile signature for cache invalidation ──
                    signature = (int(stat_mtime * 1e9) if stat_mtime else 0, stat_size)
                    cached = self._tile_health_cache.get(tile_key)

                    catalog_entry: Dict[str, Any] = {
                        "layer": layer,
                        "tile": pcd_path.name,
                        "points": header.get("point_count", 0) if header else 0,
                        "bytes": stat_size if stat_mtime else 0,
                        "mtime_ns": int(stat_mtime * 1e9) if stat_mtime else None,
                        "header_valid": header is not None,
                        "data_format": header.get("data_format") if header else None,
                    }
                    # Parse tile coordinates from filename
                    m = _tile_coord_re.match(pcd_path.stem)
                    if m:
                        catalog_entry["tile_x"] = int(m.group(1))
                        catalog_entry["tile_y"] = int(m.group(2))

                    if header:
                        layer_points += header.get("point_count", 0)

                    # ── bounds / outlier analysis ──
                    if approved:
                        if cached is not None and cached.get("signature") == signature:
                            # Reuse complete cached analysis → P0-3 fix
                            if cached.get("out_of_approved_bounds"):
                                tiles_out_of_bounds.append(tile_key)
                            if cached.get("z_outlier"):
                                z_outlier_tiles.append({
                                    "tile": tile_key,
                                    "z_min": cached.get("z_min"),
                                    "z_max": cached.get("z_max"),
                                })
                            total_z_below += cached.get("z_outlier_below_count", 0)
                            total_z_above += cached.get("z_outlier_above_count", 0)
                            catalog_entry["bounds"] = cached.get("bounds")
                            catalog_entry["z_outlier_below_count"] = cached.get("z_outlier_below_count", 0)
                            catalog_entry["z_outlier_above_count"] = cached.get("z_outlier_above_count", 0)
                        else:
                            # Sample XYZ bounds — only when tile is new or changed
                            bounds = read_pcd_xyz_bounds(pcd_path, max_sample=2000,
                                                         z_below=z_below, z_above=z_above)
                            # Build complete cache entry
                            cache_entry: Dict[str, Any] = {
                                "signature": signature,
                                "header_valid": header is not None,
                                "scan_error": None,
                            }
                            if bounds is not None:
                                bnd = {k: bounds[k] for k in
                                       ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max")
                                       if k in bounds}
                                catalog_entry["bounds"] = bnd
                                catalog_entry["z_outlier_below_count"] = bounds.get("z_outlier_below_count", 0)
                                catalog_entry["z_outlier_above_count"] = bounds.get("z_outlier_above_count", 0)
                                cache_entry["bounds"] = bnd
                                cache_entry["z_min"] = bounds.get("z_min")
                                cache_entry["z_max"] = bounds.get("z_max")
                                cache_entry["z_outlier_below_count"] = bounds.get("z_outlier_below_count", 0)
                                cache_entry["z_outlier_above_count"] = bounds.get("z_outlier_above_count", 0)
                                total_z_below += bounds.get("z_outlier_below_count", 0)
                                total_z_above += bounds.get("z_outlier_above_count", 0)

                                # Approved bounds check
                                ax_min = approved.get("x_min", -100)
                                ax_max = approved.get("x_max", 100)
                                ay_min = approved.get("y_min", -100)
                                ay_max = approved.get("y_max", 100)
                                is_oob = (bounds.get("x_min", ax_min) < ax_min or
                                          bounds.get("x_max", ax_max) > ax_max or
                                          bounds.get("y_min", ay_min) < ay_min or
                                          bounds.get("y_max", ay_max) > ay_max)
                                cache_entry["out_of_approved_bounds"] = is_oob
                                if is_oob:
                                    tiles_out_of_bounds.append(tile_key)

                                # Z outlier check
                                is_z_outlier = (bounds.get("z_min", 0) < z_below or
                                                bounds.get("z_max", 0) > z_above)
                                cache_entry["z_outlier"] = is_z_outlier
                                if is_z_outlier:
                                    z_outlier_tiles.append({
                                        "tile": tile_key,
                                        "z_min": bounds.get("z_min"),
                                        "z_max": bounds.get("z_max"),
                                    })
                            else:
                                # P0-4 fix: bounds=None → record scan error, don't crash
                                cache_entry["scan_error"] = "read_pcd_xyz_bounds returned None"
                                cache_entry["out_of_approved_bounds"] = False
                                cache_entry["z_outlier"] = False
                                cache_entry["z_outlier_below_count"] = 0
                                cache_entry["z_outlier_above_count"] = 0
                            self._tile_health_cache[tile_key] = cache_entry
                            changed_tiles.append(catalog_entry)
                    tile_catalog.append(catalog_entry)
            total_tile_files += layer_count
            total_tile_bytes += layer_bytes
            total_tile_points += layer_points
            layer_stats[layer] = {"files": layer_count, "points": layer_points, "bytes": layer_bytes}

        # ── layer ratio anomalies ──
        display_points = layer_stats.get("tiles_display", {}).get("points", 0)
        ratio_alerts: List[str] = []
        if display_points > 0:
            for layer, key in [("tiles_objects", "max_objects_to_display_ratio"),
                               ("tiles_ground", "max_ground_to_display_ratio"),
                               ("tiles_registration", "max_registration_to_display_ratio")]:
                threshold = map_health_cfg.get(key, 0.85)
                lp = layer_stats.get(layer, {}).get("points", 0)
                if lp / display_points > threshold:
                    ratio_alerts.append(f"{layer}/display={lp / display_points:.2f}")

        # ── tile growth tracking ──
        prev_points = self._last_tile_points
        prev_scan_time = self.last_filesystem_scan_wall
        self._last_tile_files = total_tile_files
        self._last_tile_points = total_tile_points
        growth_points_per_hour = 0.0
        if prev_scan_time and prev_scan_time > 0 and prev_points is not None:
            dt_hours = (time.time() - prev_scan_time) / 3600.0
            if dt_hours > 0:
                growth_points_per_hour = (total_tile_points - prev_points) / dt_hours

        # ── write tile catalog: atomic latest snapshot + changes-only JSONL ──
        self.writer.submit("atomic_json", ("reports/map_tile_catalog_latest.json",
                                           tile_catalog))
        for entry in changed_tiles:
            self.writer.submit("jsonl", ("samples/map_tile_changes.jsonl", entry))

        # ── write map health CSV sample ──
        psi = read_psi()
        health_csv_fields = [
            "wall_time", "manifest_state", "tile_files", "tile_points",
            "tile_bytes", "tile_size_mb", "tiles_out_of_bounds_count",
            "z_outlier_tiles_count", "z_outlier_below_total", "z_outlier_above_total",
            "ratio_alerts_count", "growth_points_per_hour",
            "cpu_some_avg10", "memory_some_avg10", "io_some_avg10",
            "io_full_avg10",
        ]
        health_csv_row = {
            "wall_time": time.time(),
            "manifest_state": manifest_state,
            "tile_files": total_tile_files,
            "tile_points": total_tile_points,
            "tile_bytes": total_tile_bytes,
            "tile_size_mb": total_tile_bytes / (1024.0 * 1024.0),
            "tiles_out_of_bounds_count": len(tiles_out_of_bounds),
            "z_outlier_tiles_count": len(z_outlier_tiles),
            "z_outlier_below_total": total_z_below,
            "z_outlier_above_total": total_z_above,
            "ratio_alerts_count": len(ratio_alerts),
            "growth_points_per_hour": growth_points_per_hour,
            "cpu_some_avg10": psi.get("cpu_some_avg10"),
            "memory_some_avg10": psi.get("memory_some_avg10"),
            "io_some_avg10": psi.get("io_some_avg10"),
            "io_full_avg10": psi.get("io_full_avg10"),
        }
        self.writer.submit("csv", ("samples/map_health_samples.csv",
                                   health_csv_fields, health_csv_row))

        # ── write map health latest JSON report ──
        report: Dict[str, Any] = {
            "wall_time": time.time(),
            "manifest_state": manifest_state,
            "active_manifest_present": active.is_file(),
            "last_good_manifest_present": last_good.is_file(),
            "suspension_marker_present": suspended.is_file(),
            "persistent_tmp_files": tmp_files,
            "persistent_tile_files": total_tile_files,
            "persistent_total_tile_points": total_tile_points,
            "persistent_total_tile_bytes": total_tile_bytes,
            "persistent_newest_tile_mtime": newest_tile_mtime,
            "persistent_size_mb": total_tile_bytes / (1024.0 * 1024.0),
            "layer_stats": layer_stats,
            "tiles_out_of_bounds": tiles_out_of_bounds,
            "z_outlier_tiles": z_outlier_tiles,
            "ratio_alerts": ratio_alerts,
            "tile_growth_points_per_hour": growth_points_per_hour,
            "z_outlier_below_total": total_z_below,
            "z_outlier_above_total": total_z_above,
            "psi": psi,
            "manifest_generation": manifest_payload.get("generation") if manifest_payload else None,
            "manifest_revision": manifest_payload.get("revision") if manifest_payload else None,
            "manifest_total_cells": manifest_payload.get("total_cells") if manifest_payload else None,
            "manifest_mature_cells": manifest_payload.get("mature_cells") if manifest_payload else None,
        }
        self.writer.submit("atomic_json", ("reports/map_health_latest.json", report))

        result = {
            "manifest_state": manifest_state,
            "active_manifest_present": active.is_file(),
            "last_good_manifest_present": last_good.is_file(),
            "suspension_marker_present": suspended.is_file(),
            "persistent_tmp_files": tmp_files,
            "persistent_tile_files": total_tile_files,
            "persistent_total_tile_points": total_tile_points,
            "persistent_total_tile_bytes": total_tile_bytes,
            "persistent_newest_tile_mtime": newest_tile_mtime,
            "persistent_size_mb": total_tile_bytes / (1024.0 * 1024.0),
            "layer_stats": layer_stats,
            "tiles_out_of_bounds": tiles_out_of_bounds,
            "z_outlier_tiles": z_outlier_tiles,
            "ratio_alerts": ratio_alerts,
            "tile_growth_points_per_hour": growth_points_per_hour,
            "z_outlier_below_total": total_z_below,
            "z_outlier_above_total": total_z_above,
            "psi": psi,
            "manifest_generation": manifest_payload.get("generation") if manifest_payload else None,
            "manifest_revision": manifest_payload.get("revision") if manifest_payload else None,
            "manifest_total_cells": manifest_payload.get("total_cells") if manifest_payload else None,
            "manifest_mature_cells": manifest_payload.get("mature_cells") if manifest_payload else None,
        }
        self._last_map_scan_result = result
        return result

    def snapshot(self, now: Optional[float] = None) -> Dict[str, Any]:
        wall = time.time() if now is None else now
        self._update_motion_state(wall)
        recognition_cfg = self.config.get("cargo_recognition_monitor", {})
        recognition_stale_sec = float(recognition_cfg.get("stale_sec", 1.0))
        recognition_reference = (self.last_recognition_wall or
                                 self.cargo_gate.episode_started_wall)
        if (self.cargo_gate.active and
                recognition_reference is not None and
                wall - recognition_reference > recognition_stale_sec and
                not self._recognition_stale_active):
            self._emit_typed_event(
                "samples/cargo_recognition_events.jsonl", {
                    "event": "CARGO_RECOGNITION_STALE",
                    "wall_time": wall,
                    "age_sec": wall - recognition_reference,
                    "reason": "recognition_status_stale",
                })
            self._recognition_stale_active = True
        swing_cfg = self.config.get("cargo_swing_monitor", {})
        swing_stale_sec = float(swing_cfg.get("stale_sec", 0.75))
        swing_reference = (self.last_swing_wall or
                           self.cargo_gate.episode_started_wall)
        if (self.cargo_gate.active and swing_reference is not None and
                wall - swing_reference > swing_stale_sec and
                not self._swing_stale_active):
            self._emit_typed_event(
                "samples/cargo_swing_events.jsonl", {
                    "event": "CARGO_SWING_EVIDENCE_STALE",
                    "wall_time": wall,
                    "age_sec": wall - swing_reference,
                    "reason": "swing_status_stale",
                })
            self._swing_stale_count += 1
            self._swing_stale_active = True
        runtime = self._sample_runtime(wall)
        # Real safety_status age from callback wall time
        runtime["safety_status_age_sec"] = (
            wall - self.last_safety_wall if self.last_safety_wall else None)
        with self.aggregator_lock:
            summary = self.aggregator.full_summary(now=wall)
        recovery_values = list(self._recognition_recovery_sec)
        offset_values = list(self._swing_offsets)
        angle_values = list(self._swing_angles)
        speed_values = list(self._swing_speeds)
        yaw_error_values = list(self._swing_yaw_errors)
        summary.update({
            "runtime": runtime,
            "writer_dropped": self.writer.dropped,
            "restart_count": self.restart_count,
            "recognition_state_counts": {
                str(key): value
                for key, value in self._recognition_state_counts.items()
            },
            "recognition_failure_count": self._recognition_failure_count,
            "recognition_source_epoch": self._recognition_epoch,
            "recognition_duplicate_source_stamps":
                self._recognition_duplicate_count,
            "recognition_source_stamp_rollbacks":
                self._recognition_rollback_count,
            "longest_loaded_without_lock_sec":
                self._longest_loaded_without_lock_sec,
            "recognition_recovery_sec": {
                "count": len(recovery_values),
                "last": recovery_values[-1] if recovery_values else None,
                "maximum": max(recovery_values) if recovery_values else None,
                "p95": _percentile(recovery_values, 0.95),
            },
            "sway_state_counts": {
                str(key): value for key, value in self._sway_state_counts.items()
            },
            "skew_pull_state_counts": {
                str(key): value for key, value in self._skew_state_counts.items()
            },
            "torsion_state_counts": {
                str(key): value
                for key, value in self._torsion_state_counts.items()
            },
            "maximum_offset_m": max(offset_values) if offset_values else None,
            "p95_offset_m": _percentile(offset_values, 0.95),
            "maximum_angle_deg": max(angle_values) if angle_values else None,
            "p95_angle_deg": _percentile(angle_values, 0.95),
            "maximum_horizontal_speed_mps": (
                max(speed_values) if speed_values else None),
            "maximum_yaw_error_deg": (
                max(yaw_error_values) if yaw_error_values else None),
            "longest_sway_warning_sec": self._longest_sway_warning_sec,
            "longest_skew_suspected_sec":
                self._longest_skew_suspected_sec,
            "sway_alarm_count": self._sway_alarm_count,
            "skew_pull_alarm_count": self._skew_pull_alarm_count,
            "skew_alarm_inhibited_count": self._skew_alarm_inhibited_count,
            "torsion_alarm_count": self._torsion_alarm_count,
            "swing_stale_count": self._swing_stale_count,
            "swing_source_epoch": self._swing_epoch,
            "swing_duplicate_source_stamps": self._swing_duplicate_count,
            "swing_source_stamp_rollbacks": self._swing_rollback_count,
            "cargo_recognition_track_stats": self._recognition_track_stats,
            "cargo_swing_track_stats": self._swing_track_stats,
            "gravity_data_ready": self.gravity_message_count > 0,
            "cargo_monitor_state": self.cargo_gate.state,
            "cargo_monitor_active": self.cargo_gate.active,
            "cargo_episode_id": self.cargo_gate.episode_id,
        })
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

        # ── readiness handshake ──
        # Read process start ticks for identity verification
        try:
            _proc_start_ticks = int(open('/proc/self/stat').read().split()[21])
        except Exception:
            _proc_start_ticks = 0

        # Track received topics for data_ready
        _first_message_wall: Dict[str, float] = {}
        _last_message_wall: Dict[str, float] = {}
        _required_topics = ["/odom", "/cargo_avoidance/safety_status",
                            "/cargo_avoidance/status_code"]

        ready_payload: Dict[str, Any] = {
            "ready": True,
            "process_ready": True,
            "data_ready": False,
            "pid": os.getpid(),
            "run_id": self.args.run_id,
            "boot_id": self._read_boot_id(),
            "process_start_ticks": _proc_start_ticks,
            "ros_node": "/ndt_slam_server_monitor",
            "workspace_sha": self.args.expected_sha or "unknown",
            "created_at": time.time(),
            "ready_updated_at": time.time(),
            "odom_message_count": 0,
            "safety_status_message_count": 0,
            "status_code_message_count": 0,
            "recognition_status_message_count": 0,
            "swing_status_message_count": 0,
            "gravity_message_count": 0,
            "recognition_source_epoch": 0,
            "swing_source_epoch": 0,
            "recognition_duplicate_source_stamps": 0,
            "swing_duplicate_source_stamps": 0,
            "recognition_source_stamp_rollbacks": 0,
            "swing_source_stamp_rollbacks": 0,
            "last_recognition_wall": None,
            "last_swing_wall": None,
            "recognition_data_ready": False,
            "swing_data_ready": False,
            "gravity_data_ready": False,
            "cargo_monitor_active": False,
            "cargo_monitor_ready": False,
            "avoidance_pipeline_ready": False,
            "static_session_ready": False,
            "extended_data_ready": False,
            "received_topics": {},
            "first_message_wall": {},
            "last_message_wall": {},
            "topics": {},
        }
        try:
            import rosgraph
            master = rosgraph.Master(self.rospy.get_name())
            topic_types = master.getTopicTypes()
            for topic_name, topic_type in topic_types:
                if topic_name in ("/odom", "/cargo_avoidance/safety_status",
                                  "/cargo_avoidance/status_code",
                                  "/cargo_avoidance/static_evidence_debug",
                                  "/cargo_avoidance/cargo_geometry_debug",
                                  "/cargo_avoidance/operational_status",
                                  "/cargo_avoidance/raw_safety_status",
                                  "/cargo_avoidance/raw_status_code",
                                  "/cargo_avoidance/pending_status",
                                  "/cargo_avoidance/bottom_estimate",
                                  "/hook/load_state",
                                  "/cargo_avoidance/recognition_status",
                                  "/cargo_avoidance/swing_status"):
                    ready_payload["topics"][topic_name] = topic_type
        except Exception:
            pass
        atomic_write_json(self.run_dir / "reports" / "monitor_ready.json", ready_payload)

        # ── wall-clock scheduling (not rospy.Rate) ──
        # Uses time.monotonic() so the loop runs even when /use_sim_time is set
        # and /clock is paused.  ROS time is only used for message source stamps.
        next_sample = time.monotonic()
        next_ready_update = time.monotonic() + 5.0  # update ready after 5s

        while not self.rospy.is_shutdown() and not self.shutdown_requested:
            now = time.time()
            mono_now = time.monotonic()
            self._expire_gravity_if_needed(now)

            # ── update data_ready tracking ──
            if mono_now >= next_ready_update:
                _rdy = ready_payload
                _rdy["received_topics"] = {
                    t: t in _first_message_wall for t in _required_topics
                }
                _rdy["first_message_wall"] = dict(_first_message_wall)
                _rdy["last_message_wall"] = dict(_last_message_wall)
                _rdy["odom_message_count"] = self.odom_message_count
                _rdy["safety_status_message_count"] = self.safety_status_message_count
                _rdy["status_code_message_count"] = self.status_code_message_count
                _rdy["recognition_status_message_count"] = (
                    self.recognition_status_message_count)
                _rdy["swing_status_message_count"] = (
                    self.swing_status_message_count)
                _rdy["gravity_message_count"] = self.gravity_message_count
                _rdy["recognition_source_epoch"] = self._recognition_epoch
                _rdy["swing_source_epoch"] = self._swing_epoch
                _rdy["recognition_duplicate_source_stamps"] = (
                    self._recognition_duplicate_count)
                _rdy["swing_duplicate_source_stamps"] = (
                    self._swing_duplicate_count)
                _rdy["recognition_source_stamp_rollbacks"] = (
                    self._recognition_rollback_count)
                _rdy["swing_source_stamp_rollbacks"] = (
                    self._swing_rollback_count)
                _rdy["last_recognition_wall"] = self.last_recognition_wall
                _rdy["last_swing_wall"] = self.last_swing_wall
                _rdy["data_ready"] = (
                    self.odom_message_count > 0
                    and self.safety_status_message_count > 0
                    and self.status_code_message_count > 0
                )
                _rdy["recognition_data_ready"] = (
                    self.recognition_status_message_count > 0)
                _rdy["swing_data_ready"] = (
                    self.swing_status_message_count > 0)
                _rdy["gravity_data_ready"] = self.gravity_message_count > 0
                _rdy["cargo_monitor_active"] = self.cargo_gate.active
                _rdy["cargo_monitor_ready"] = (
                    not self.cargo_gate.active or
                    (self.recognition_status_message_count >
                         self._episode_recognition_messages_start and
                     self.swing_status_message_count >
                         self._episode_swing_messages_start))
                _rdy["avoidance_pipeline_ready"] = all(
                    name in self.pipeline.values for name in
                    ("raw_typed", "raw_simple", "final_typed", "heartbeat"))
                _rdy["static_session_ready"] = _as_bool(
                    self.pipeline.pending.get("map_session_verified", False))
                _rdy["extended_data_ready"] = (
                    _rdy["data_ready"]
                    and _rdy["recognition_data_ready"]
                    and _rdy["swing_data_ready"])
                _rdy["ready_updated_at"] = time.time()
                # created_at is set once at startup; never rewritten
                atomic_write_json(
                    self.run_dir / "reports" / "monitor_ready.json", _rdy)
                next_ready_update = mono_now + 5.0

            # ── snapshot ──
            summary = self.snapshot(now)
            pipeline = self._record_pipeline_snapshot(now)
            if self.cargo_gate.active:
                cargo_line = self._cargo_terminal_line(now, pipeline)
                print(cargo_line, flush=True)
                self.writer.submit("text", ("logs/cargo_terminal.log",
                                             cargo_line))

            # ── record message receipt timestamps ──
            if self.last_odom_wall:
                _first_message_wall.setdefault("/odom", self.last_odom_wall)
                _last_message_wall["/odom"] = self.last_odom_wall
            if self.last_safety_wall:
                _first_message_wall.setdefault(
                    "/cargo_avoidance/safety_status", self.last_safety_wall)
                _last_message_wall["/cargo_avoidance/safety_status"] = self.last_safety_wall
            if self.last_status_code_wall:
                _first_message_wall.setdefault(
                    "/cargo_avoidance/status_code",
                    self.last_status_code_wall)
                _last_message_wall["/cargo_avoidance/status_code"] = (
                    self.last_status_code_wall)
            if self.last_recognition_wall:
                _first_message_wall.setdefault(
                    "/cargo_avoidance/recognition_status",
                    self.last_recognition_wall)
                _last_message_wall[
                    "/cargo_avoidance/recognition_status"] = (
                        self.last_recognition_wall)
            if self.last_swing_wall:
                _first_message_wall.setdefault(
                    "/cargo_avoidance/swing_status", self.last_swing_wall)
                _last_message_wall["/cargo_avoidance/swing_status"] = (
                    self.last_swing_wall)
            if self.last_gravity_wall:
                _first_message_wall.setdefault(
                    "/hook/load_state", self.last_gravity_wall)
                _last_message_wall["/hook/load_state"] = (
                    self.last_gravity_wall)

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

            # Wall-clock sleep (not rospy.Rate — survives /clock pause)
            next_sample += sample_period
            sleep_sec = next_sample - time.monotonic()
            if sleep_sec > 0:
                self.shutdown_event.wait(min(sleep_sec, 1.0))
            else:
                # We fell behind; reset to avoid tight loop
                next_sample = time.monotonic() + sample_period

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
        monitor.shutdown_event.set()

    def snapshot_handler(_signum: int, _frame: Any) -> None:
        monitor.snapshot_requested = True
        monitor.shutdown_event.set()

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    if hasattr(signal, "SIGUSR1"):
        signal.signal(signal.SIGUSR1, snapshot_handler)
    monitor.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
