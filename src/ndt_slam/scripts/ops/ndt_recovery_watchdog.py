#!/usr/bin/env python3
"""Bounded NDT relocalization watchdog with an offline-testable policy.

The ROS wrapper first requests the in-process global relocalizer. If the
latched recovery state remains degraded for a configured interval, this
required roslaunch node records durable evidence and exits non-zero. The
production systemd unit then restarts the complete launch. A persisted restart
budget prevents a damaged sensor or map from creating an unbounded restart
loop.
"""

from __future__ import annotations

import dataclasses
import json
import os
import re
import socket
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional


STATE_PATTERN = re.compile(r"(?:^|\s)state=([^\s]+)")
BAD_FRAMES_PATTERN = re.compile(r"(?:^|\s)bad_frames=(\d+)")
HEALTHY_STATES = frozenset(("IDLE", "COOLDOWN"))
DEGRADED_STATES = frozenset(
    ("DEGRADED", "SEARCHING_LOCAL", "SEARCHING_GLOBAL", "CONFIRMING")
)


@dataclasses.dataclass(frozen=True)
class RecoveryStatus:
    state: str
    bad_frames: int
    raw: str


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

        hard_by_time = (
            degraded_duration >= self.config.hard_restart_after_sec
        )
        hard_by_frames = (
            status.bad_frames >= self.config.hard_restart_bad_frames
            and degraded_duration >=
            self.config.hard_restart_bad_frames_after_sec
        )
        if hard_by_time or hard_by_frames:
            if (len(self.restart_history) >=
                    self.config.max_restarts_in_window):
                if self.suppression_reported:
                    return WatchdogDecision(
                        "none", degraded_duration,
                        "restart_budget_already_reported"
                    )
                self.suppression_reported = True
                return WatchdogDecision(
                    "restart_suppressed", degraded_duration,
                    "restart_budget_exhausted"
                )
            self.restart_history.append(now)
            reason = (
                "persistent_bad_frames" if hard_by_frames
                else "persistent_degraded_state"
            )
            return WatchdogDecision(
                "hard_restart", degraded_duration, reason
            )

        if (not self.soft_requested and degraded_duration >=
                self.config.soft_relocalize_after_sec):
            self.soft_requested = True
            return WatchdogDecision(
                "soft_relocalize", degraded_duration,
                "degraded_state_soft_recovery"
            )
        return WatchdogDecision(
            "none", degraded_duration, "waiting_for_recovery"
        )


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


def append_jsonl(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
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
        from std_msgs.msg import String
        from std_srvs.srv import Empty

        self.rospy = rospy
        self.empty_service = Empty
        self.status: Optional[RecoveryStatus] = None
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
        self.policy = RecoveryWatchdogPolicy(
            config, self.started_at,
            read_restart_history(self.state_path)
        )
        self.relocalize_service_name = str(
            rospy.get_param(
                "~relocalize_service", "/ndt_slam/relocalize"
            )
        )
        status_topic = str(
            rospy.get_param(
                "~status_topic", "/ndt_slam/relocalization_status"
            )
        )
        self.relocalize = rospy.ServiceProxy(
            self.relocalize_service_name, Empty
        )
        self.subscriber = rospy.Subscriber(
            status_topic, String, self._status_callback, queue_size=10
        )
        self.timer = rospy.Timer(
            rospy.Duration(float(rospy.get_param("~poll_sec", 1.0))),
            self._timer_callback,
        )

    def _status_callback(self, message: Any) -> None:
        parsed = parse_relocalization_status(message.data)
        if parsed is not None:
            self.status = parsed

    def _event(self, decision: WatchdogDecision,
               status: RecoveryStatus, now: float) -> Dict[str, Any]:
        return {
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
            "wall_time": now,
            "wall_time_iso": time.strftime(
                "%Y-%m-%dT%H:%M:%SZ", time.gmtime(now)
            ),
        }

    def _persist(self, event: Mapping[str, Any]) -> None:
        append_jsonl(self.events_path, event)
        atomic_write_json(
            self.state_path,
            {
                "last_event": dict(event),
                "restart_history": list(self.policy.restart_history),
            },
        )

    def _timer_callback(self, _event: Any) -> None:
        if self.status is None:
            return
        now = time.time()
        decision = self.policy.observe(self.status, now)
        if decision.action == "none":
            return
        evidence = self._event(decision, self.status, now)
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
                self.rospy.wait_for_service(
                    self.relocalize_service_name, timeout=2.0
                )
                self.relocalize()
                self.rospy.logwarn(
                    "[NdtRecoveryWatchdog] requested global relocalization "
                    "after %.1fs degraded",
                    decision.degraded_duration_sec,
                )
            except Exception as error:
                self.rospy.logerr(
                    "[NdtRecoveryWatchdog] relocalize request failed: %s",
                    error,
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

        self.rospy.logfatal(
            "[NdtRecoveryWatchdog] persistent localization failure; "
            "exiting 75 for supervised full-stack restart; evidence=%s",
            self.events_path,
        )
        os._exit(75)


def main() -> int:
    import rospy

    rospy.init_node("ndt_recovery_watchdog")
    RosRecoveryWatchdog()
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
