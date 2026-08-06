#!/usr/bin/env python3
"""NDT recovery watchdog for strict health and a foreground supervisor.

Responsive localization failures stay quarantined and recover in-process.
Only simultaneous loss of the health/status/odom control streams requests a
bounded full-stack restart from the foreground supervisor. A failed recovery
RPC remains diagnostic while the SLAM processing streams are alive.
"""

from __future__ import annotations

import dataclasses
import json
import os
import re
import socket
import threading
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple


STATE_PATTERN = re.compile(r"(?:^|\s)state=([^\s]+)")
BAD_FRAMES_PATTERN = re.compile(r"(?:^|\s)bad_frames=(\d+)")
HEALTHY_STATES = frozenset(("IDLE", "COOLDOWN"))
DEGRADED_STATES = frozenset(
    ("DEGRADED", "SEARCHING_LOCAL", "SEARCHING_GLOBAL", "CONFIRMING")
)
RECOVERY_IN_PROGRESS_STATES = frozenset(
    ("SEARCHING_LOCAL", "SEARCHING_GLOBAL", "CONFIRMING")
)
RELOCALIZE_SERVICE_CANDIDATES = (
    "/relocalize",
    "/ndt_slam/relocalize",
    "/ndt_slam_node/relocalize",
)


@dataclasses.dataclass(frozen=True)
class RecoveryStatus:
    state: str
    bad_frames: int
    raw: str


@dataclasses.dataclass(frozen=True)
class LocalizationHealth:
    startup_state: str
    relocalization_state: str
    pose_reliable: bool
    strict_verified: bool
    reason: str
    raw: Mapping[str, Any]


@dataclasses.dataclass
class WatchdogConfig:
    startup_grace_sec: float = 30.0
    soft_relocalize_after_sec: float = 8.0
    hard_restart_after_sec: float = 45.0
    hard_restart_bad_frames: int = 300
    hard_restart_bad_frames_after_sec: float = 15.0
    healthy_reset_sec: float = 10.0
    restart_window_sec: float = 900.0
    max_restarts_in_window: int = 3
    health_stale_sec: float = 3.0
    service_retry_sec: float = 10.0
    service_failure_threshold: int = 3
    event_log_max_bytes: int = 5 * 1024 * 1024
    event_log_backups: int = 5


@dataclasses.dataclass(frozen=True)
class WatchdogDecision:
    action: str
    degraded_duration_sec: float
    reason: str


def parse_relocalization_status(message: str) -> Optional[RecoveryStatus]:
    state_match = STATE_PATTERN.search(message or "")
    bad_frames_match = BAD_FRAMES_PATTERN.search(message or "")
    if state_match is None or bad_frames_match is None:
        return None
    return RecoveryStatus(
        state=state_match.group(1).upper(),
        bad_frames=int(bad_frames_match.group(1)),
        raw=message.strip(),
    )


def parse_localization_health(message: str) -> Optional[LocalizationHealth]:
    try:
        payload = json.loads(message or "")
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict) or payload.get("schema_version") != 1:
        return None
    startup_state = str(payload.get("startup_state", "")).upper()
    relocalization_state = str(
        payload.get("relocalization_state", "")
    ).upper()
    if not startup_state or not relocalization_state:
        return None
    return LocalizationHealth(
        startup_state=startup_state,
        relocalization_state=relocalization_state,
        pose_reliable=payload.get("pose_reliable") is True,
        strict_verified=payload.get("strict_verified") is True,
        reason=str(payload.get("reason", "unknown")),
        raw=payload,
    )


def normalize_ros_name(name: str) -> str:
    normalized = str(name or "").strip()
    if not normalized:
        return ""
    return normalized if normalized.startswith("/") else "/" + normalized


def resolve_relocalize_service(
    configured: str, published_services: Iterable[Any]
) -> str:
    """Resolve the global service advertised by the SLAM node.

    rospy service listings can contain either names or ``(name, providers)``
    pairs.  Prefer an explicitly configured live service, then the known
    global/private variants.  Returning the configured name when discovery is
    unavailable preserves compatibility while keeping the common
    ``advertiseService(\"relocalize\")`` case correct.
    """
    available = set()
    for item in published_services:
        value = item[0] if isinstance(item, (tuple, list)) and item else item
        normalized = normalize_ros_name(value)
        if normalized:
            available.add(normalized)
    configured_name = normalize_ros_name(configured)
    ordered = (configured_name,) + RELOCALIZE_SERVICE_CANDIDATES
    for candidate in ordered:
        if candidate and candidate in available:
            return candidate
    return configured_name or RELOCALIZE_SERVICE_CANDIDATES[0]


def _finite_history(values: Iterable[Any], now: float,
                    window_sec: float) -> List[float]:
    lower_bound = now - max(1.0, float(window_sec))
    history: List[float] = []
    for value in values:
        try:
            timestamp = float(value)
        except (TypeError, ValueError):
            continue
        if lower_bound <= timestamp <= now + 1.0:
            history.append(timestamp)
    return sorted(history)


class RecoveryWatchdogPolicy:
    def __init__(self, config: WatchdogConfig, started_at: float,
                 restart_history: Iterable[Any] = ()) -> None:
        self.config = config
        self.started_at = float(started_at)
        self.restart_history = _finite_history(
            restart_history, self.started_at, config.restart_window_sec
        )
        self.degraded_since: Optional[float] = None
        self.healthy_since: Optional[float] = None
        self.soft_requested = False
        self.suppression_reported = False

    def observe(self, status: RecoveryStatus,
                now: float) -> WatchdogDecision:
        now = float(now)
        self.restart_history = _finite_history(
            self.restart_history, now, self.config.restart_window_sec
        )

        if status.state in HEALTHY_STATES:
            self.degraded_since = None
            if self.healthy_since is None:
                self.healthy_since = now
            if now - self.healthy_since >= self.config.healthy_reset_sec:
                self.soft_requested = False
                self.suppression_reported = False
            return WatchdogDecision("none", 0.0, "localization_healthy")

        self.healthy_since = None
        if status.state not in DEGRADED_STATES:
            return WatchdogDecision("none", 0.0, "state_not_actionable")
        if self.degraded_since is None:
            self.degraded_since = now
        degraded_duration = max(0.0, now - self.degraded_since)

        if now - self.started_at < self.config.startup_grace_sec:
            return WatchdogDecision(
                "none", degraded_duration, "startup_grace"
            )

        # SEARCHING/CONFIRMING are owned by the in-process recovery episode.
        # Re-submitting the service here resets confirmation state and can
        # prevent an otherwise good candidate from ever reaching HEALTHY.
        if status.state in RECOVERY_IN_PROGRESS_STATES:
            return WatchdogDecision(
                "none", degraded_duration, "recovery_episode_in_progress"
            )

        if (not self.soft_requested and degraded_duration >=
                self.config.soft_relocalize_after_sec):
            self.soft_requested = True
            return WatchdogDecision(
                "soft_relocalize", degraded_duration,
                "degraded_state_soft_recovery"
            )
        return WatchdogDecision(
            "none", degraded_duration,
            "responsive_localization_waiting_for_recovery"
        )

    def process_fault(self, now: float, reason: str) -> WatchdogDecision:
        now = float(now)
        self.restart_history = _finite_history(
            self.restart_history, now, self.config.restart_window_sec
        )
        if now - self.started_at < self.config.startup_grace_sec:
            return WatchdogDecision("none", 0.0, "startup_grace")
        if len(self.restart_history) >= self.config.max_restarts_in_window:
            if self.suppression_reported:
                return WatchdogDecision(
                    "none", 0.0, "restart_budget_already_reported"
                )
            self.suppression_reported = True
            return WatchdogDecision(
                "restart_suppressed", 0.0, "restart_budget_exhausted"
            )
        self.restart_history.append(now)
        return WatchdogDecision("hard_restart", 0.0, reason)


def atomic_write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2,
                  sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def rotate_jsonl(path: Path, max_bytes: int, backups: int) -> None:
    if max_bytes <= 0 or backups <= 0:
        return
    try:
        if path.stat().st_size < max_bytes:
            return
    except FileNotFoundError:
        return
    for index in range(backups, 0, -1):
        source = path if index == 1 else path.with_name(
            f"{path.name}.{index - 1}"
        )
        target = path.with_name(f"{path.name}.{index}")
        if not source.exists():
            continue
        if index == backups and target.exists():
            target.unlink()
        os.replace(str(source), str(target))


def append_jsonl(path: Path, payload: Mapping[str, Any],
                 max_bytes: int = 5 * 1024 * 1024,
                 backups: int = 5) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rotate_jsonl(path, max_bytes, backups)
    with path.open("a", encoding="utf-8", newline="\n") as stream:
        json.dump(payload, stream, ensure_ascii=False, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def read_restart_history(path: Path) -> List[float]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return []
    values = payload.get("restart_history", [])
    return list(values) if isinstance(values, list) else []


class RosRecoveryWatchdog:
    def __init__(self) -> None:
        import rospy
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        from std_srvs.srv import Empty

        self.rospy = rospy
        self.status: Optional[RecoveryStatus] = None
        self.status_received_at: Optional[float] = None
        self.odom_received_at: Optional[float] = None
        self.health: Optional[LocalizationHealth] = None
        self.health_received_at: Optional[float] = None
        self.restart_requested = threading.Event()
        self.started_at = time.time()
        config = WatchdogConfig(
            startup_grace_sec=float(
                rospy.get_param("~startup_grace_sec", 30.0)
            ),
            soft_relocalize_after_sec=float(
                rospy.get_param("~soft_relocalize_after_sec", 8.0)
            ),
            hard_restart_after_sec=float(
                rospy.get_param("~hard_restart_after_sec", 45.0)
            ),
            hard_restart_bad_frames=int(
                rospy.get_param("~hard_restart_bad_frames", 300)
            ),
            hard_restart_bad_frames_after_sec=float(
                rospy.get_param(
                    "~hard_restart_bad_frames_after_sec", 15.0
                )
            ),
            healthy_reset_sec=float(
                rospy.get_param("~healthy_reset_sec", 10.0)
            ),
            restart_window_sec=float(
                rospy.get_param("~restart_window_sec", 900.0)
            ),
            max_restarts_in_window=int(
                rospy.get_param("~max_restarts_in_window", 3)
            ),
            health_stale_sec=float(
                rospy.get_param("~health_stale_sec", 3.0)
            ),
            service_retry_sec=float(
                rospy.get_param("~service_retry_sec", 10.0)
            ),
            service_failure_threshold=max(
                1, int(rospy.get_param("~service_failure_threshold", 3))
            ),
            event_log_max_bytes=int(
                rospy.get_param(
                    "~event_log_max_bytes", 5 * 1024 * 1024
                )
            ),
            event_log_backups=int(
                rospy.get_param("~event_log_backups", 5)
            ),
        )
        data_root = os.environ.get("NDT_SLAM_DATA_ROOT", "").strip()
        default_root = (
            Path(data_root) / "recovery_watchdog" if data_root
            else Path("/tmp/ndt_slam/recovery_watchdog")
        )
        configured_root = str(
            rospy.get_param("~evidence_dir", "")
        ).strip()
        self.evidence_root = (
            Path(configured_root) if configured_root else default_root
        )
        self.state_path = self.evidence_root / "state.json"
        self.events_path = self.evidence_root / "events.jsonl"
        self.restart_request_path = (
            self.evidence_root / "restart_request.json"
        )
        self.supervisor_run_id = os.environ.get(
            "NDT_SLAM_SUPERVISOR_RUN_ID", ""
        ).strip()
        self.policy = RecoveryWatchdogPolicy(
            config, self.started_at,
            read_restart_history(self.state_path)
        )
        self.relocalize_service_name = str(
            rospy.get_param(
                "~relocalize_service", "/relocalize"
            )
        )
        self.empty_service_type = Empty
        self.relocalize = None
        self.consecutive_service_failures = 0
        self.soft_retry_not_before = 0.0
        self.last_service_error = ""
        self.last_resolved_service = ""
        status_topic = str(
            rospy.get_param(
                "~status_topic", "/ndt_slam/relocalization_status"
            )
        )
        self.subscriber = rospy.Subscriber(
            status_topic, String, self._status_callback, queue_size=10
        )
        health_topic = str(
            rospy.get_param(
                "~health_topic", "/ndt_slam/localization_health"
            )
        )
        self.health_subscriber = rospy.Subscriber(
            health_topic, String, self._health_callback, queue_size=10
        )
        odom_topic = str(rospy.get_param("~odom_topic", "/odom"))
        self.odom_subscriber = rospy.Subscriber(
            odom_topic, Odometry, self._odom_callback, queue_size=10
        )
        self.timer = rospy.Timer(
            rospy.Duration(float(rospy.get_param("~poll_sec", 1.0))),
            self._timer_callback,
        )

    def _status_callback(self, message: Any) -> None:
        parsed = parse_relocalization_status(message.data)
        if parsed is not None:
            self.status = parsed
            self.status_received_at = time.time()

    def _health_callback(self, message: Any) -> None:
        parsed = parse_localization_health(message.data)
        if parsed is not None:
            self.health = parsed
            self.health_received_at = time.time()

    def _odom_callback(self, _message: Any) -> None:
        self.odom_received_at = time.time()

    def _stream_is_fresh(self, received_at: Optional[float],
                         now: float) -> bool:
        return received_at is not None and (
            now - received_at <=
            max(3.0, self.policy.config.health_stale_sec)
        )

    def _all_control_streams_stale(self, now: float) -> bool:
        return not any((
            self._stream_is_fresh(
                getattr(self, "health_received_at", None), now
            ),
            self._stream_is_fresh(
                getattr(self, "status_received_at", None), now
            ),
            self._stream_is_fresh(
                getattr(self, "odom_received_at", None), now
            ),
        ))

    def _resolve_relocalize_service(self) -> str:
        try:
            published = self.rospy.get_published_services("/")
        except Exception as error:
            self.rospy.logwarn_throttle(
                30.0,
                "[NdtRecoveryWatchdog] service discovery unavailable: %s",
                error,
            )
            published = ()
        resolved = resolve_relocalize_service(
            self.relocalize_service_name, published
        )
        self.last_resolved_service = resolved
        return resolved

    def _call_relocalize(self) -> Tuple[str, float]:
        service_name = self._resolve_relocalize_service()
        started_at = time.monotonic()
        self.rospy.wait_for_service(service_name, timeout=2.0)
        proxy = self.rospy.ServiceProxy(
            service_name, self.empty_service_type
        )
        proxy()
        return service_name, max(0.0, time.monotonic() - started_at)

    def _event(self, decision: WatchdogDecision,
               status: RecoveryStatus, now: float,
               details: Optional[Mapping[str, Any]] = None
               ) -> Dict[str, Any]:
        event = {
            "action": decision.action,
            "bad_frames": status.bad_frames,
            "degraded_duration_sec": round(
                decision.degraded_duration_sec, 3
            ),
            "hostname": socket.gethostname(),
            "pid": os.getpid(),
            "reason": decision.reason,
            "restart_history": list(self.policy.restart_history),
            "status": status.raw,
            "state": status.state,
            "health": (
                dict(self.health.raw) if self.health is not None else None
            ),
            "health_age_sec": (
                round(max(0.0, now - self.health_received_at), 3)
                if self.health_received_at is not None else None
            ),
            "supervisor_run_id": self.supervisor_run_id,
            "wall_time": now,
            "wall_time_iso": time.strftime(
                "%Y-%m-%dT%H:%M:%SZ", time.gmtime(now)
            ),
        }
        if details:
            event["details"] = dict(details)
        return event

    def _persist(self, event: Mapping[str, Any]) -> None:
        append_jsonl(
            self.events_path,
            event,
            self.policy.config.event_log_max_bytes,
            self.policy.config.event_log_backups,
        )
        atomic_write_json(
            self.state_path,
            {
                "last_event": dict(event),
                "restart_history": list(self.policy.restart_history),
            },
        )

    def _request_hard_restart(
        self, decision: WatchdogDecision,
        status: RecoveryStatus, now: float
    ) -> None:
        evidence = self._event(decision, status, now)
        try:
            self._persist(evidence)
            atomic_write_json(
                self.restart_request_path,
                {
                    "schema_version": 1,
                    "action": "hard_restart",
                    "supervisor_run_id": self.supervisor_run_id,
                    "wall_time": now,
                    "reason": decision.reason,
                    "event": evidence,
                },
            )
        except Exception as error:
            self.rospy.logerr(
                "[NdtRecoveryWatchdog] restart evidence write failed: %s",
                error,
            )
            return
        self.rospy.logfatal(
            "[NdtRecoveryWatchdog] process-level fault; requesting "
            "orderly exit 75 for foreground-supervised full-stack restart; "
            "reason=%s evidence=%s",
            decision.reason,
            self.events_path,
        )
        self.restart_requested.set()

    def _timer_callback(self, _event: Any) -> None:
        if self.restart_requested.is_set():
            return
        now = time.time()
        fallback_status = self.status or RecoveryStatus(
            "UNKNOWN", 0, "state=UNKNOWN bad_frames=0"
        )
        if self.health_received_at is None:
            # A source update without a rebuilt C++ binary has no health
            # topic, but the legacy relocalization status remains live. That
            # is a deployment mismatch, not a frozen process: restarting the
            # same old binary can never repair it and creates a restart loop.
            status_received_at = getattr(
                self, "status_received_at", None
            )
            status_fresh = status_received_at is not None and (
                now - status_received_at <=
                max(3.0, self.policy.config.health_stale_sec)
            )
            odom_received_at = getattr(self, "odom_received_at", None)
            odom_fresh = odom_received_at is not None and (
                now - odom_received_at <=
                max(3.0, self.policy.config.health_stale_sec)
            )
            if status_fresh or odom_fresh:
                self.rospy.logwarn_throttle(
                    30.0,
                    "[NdtRecoveryWatchdog] localization_health has never "
                    "been observed while a legacy processing stream is "
                    "live; "
                    "rebuild the workspace, suppressing hard restart",
                )
                return
            decision = self.policy.process_fault(
                now, "localization_control_streams_missing"
            )
        elif now - self.health_received_at > self.policy.config.health_stale_sec:
            if not self._all_control_streams_stale(now):
                self.rospy.logwarn_throttle(
                    30.0,
                    "[NdtRecoveryWatchdog] localization_health is stale "
                    "while status/odom processing remains live; suppressing "
                    "full-stack restart",
                )
                return
            decision = self.policy.process_fault(
                now, "localization_control_streams_stale"
            )
        elif self.health is not None and self.health.startup_state in {
            "MAP_INVALID", "WAITING_STATIONARY"
        }:
            # A responsive node has deliberately isolated an untrusted pose.
            # More restarts cannot repair map coverage or create a stationary
            # window, so leave recovery to the node/operator.
            return
        elif (
            self.health is not None
            and self.health.pose_reliable
            and self.health.strict_verified
        ):
            healthy = RecoveryStatus(
                "IDLE", 0, "state=IDLE bad_frames=0"
            )
            self.policy.observe(healthy, now)
            return
        elif self.status is None:
            return
        else:
            if now < self.soft_retry_not_before:
                return
            decision = self.policy.observe(self.status, now)
        if decision.action == "none":
            return
        if decision.action == "hard_restart":
            self._request_hard_restart(
                decision, fallback_status, now
            )
            return
        evidence = self._event(decision, fallback_status, now)
        try:
            self._persist(evidence)
        except Exception as error:
            # Evidence failure must be visible, but a full disk or damaged
            # evidence directory must not disable localization recovery.
            self.rospy.logerr(
                "[NdtRecoveryWatchdog] evidence write failed: %s", error
            )
        if decision.action == "soft_relocalize":
            try:
                service_name, service_duration = self._call_relocalize()
                self.consecutive_service_failures = 0
                self.last_service_error = ""
                self.rospy.logwarn(
                    "[NdtRecoveryWatchdog] requested global relocalization "
                    "after %.1fs degraded service=%s duration=%.3fs",
                    decision.degraded_duration_sec,
                    service_name,
                    service_duration,
                )
            except Exception as error:
                self.consecutive_service_failures += 1
                self.last_service_error = str(error)
                self.soft_retry_not_before = (
                    now + self.policy.config.service_retry_sec
                )
                # A failed RPC is not proof that the SLAM process is frozen.
                # Re-arm the soft request after a bounded delay and keep the
                # stack alive while any control/data stream is still fresh.
                self.policy.soft_requested = False
                self.rospy.logerr(
                    "[NdtRecoveryWatchdog] relocalize request failed "
                    "service=%s failures=%d/%d retry_in=%.1fs error=%s",
                    self.last_resolved_service or
                    normalize_ros_name(self.relocalize_service_name),
                    self.consecutive_service_failures,
                    self.policy.config.service_failure_threshold,
                    self.policy.config.service_retry_sec,
                    error,
                )
                failure_event = self._event(
                    WatchdogDecision(
                        "soft_relocalize_failed",
                        decision.degraded_duration_sec,
                        "relocalization_service_call_failed",
                    ),
                    fallback_status,
                    now,
                    {
                        "service": self.last_resolved_service or
                            normalize_ros_name(
                                self.relocalize_service_name
                            ),
                        "exception": str(error),
                        "consecutive_failures":
                            self.consecutive_service_failures,
                    },
                )
                try:
                    self._persist(failure_event)
                except Exception as persist_error:
                    self.rospy.logerr(
                        "[NdtRecoveryWatchdog] service failure evidence "
                        "write failed: %s", persist_error
                    )
                if (
                    self.consecutive_service_failures >=
                        self.policy.config.service_failure_threshold
                    and self._all_control_streams_stale(now)
                ):
                    fault = self.policy.process_fault(
                        now, "relocalization_control_plane_unresponsive"
                    )
                    if fault.action == "hard_restart":
                        self._request_hard_restart(
                            fault, fallback_status, now
                        )
            return
        if decision.action == "restart_suppressed":
            self.rospy.logerr(
                "[NdtRecoveryWatchdog] restart suppressed: %d attempts "
                "inside %.0fs; evidence=%s",
                len(self.policy.restart_history),
                self.policy.config.restart_window_sec,
                self.events_path,
            )
            return

    def run(self) -> int:
        """Wait in the main thread so exit 75 runs normal Python/ROS cleanup."""
        while not self.rospy.is_shutdown():
            if self.restart_requested.wait(timeout=0.2):
                self.timer.shutdown()
                self.rospy.signal_shutdown(
                    "persistent localization failure"
                )
                return 75
        return 0


def main() -> int:
    import rospy

    rospy.init_node("ndt_recovery_watchdog")
    return RosRecoveryWatchdog().run()


if __name__ == "__main__":
    raise SystemExit(main())
