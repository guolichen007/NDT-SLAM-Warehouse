#!/usr/bin/env python3
"""Classify NDT-SLAM service exits without creating a restart loop.

The in-process recovery watchdog remains the owner of bounded soft recovery.
This supervisor owns only the service exit contract: a fresh runtime status
from the process classifies recoverable tracking failure as 75 and semantic
reference/map/contract faults as 78.  A child that fails before publishing a
fresh status is treated as a nonrecoverable startup contract failure.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence


RECOVERABLE_FAILURES = frozenset(("RECOVERABLE_TRACKING_DEGRADATION",))
NONRECOVERABLE_FAILURES = frozenset(
    (
        "NONRECOVERABLE_REFERENCE_CONFIG",
        "NONRECOVERABLE_MAP_IDENTITY",
        "INTERNAL_CONTRACT_ERROR",
    )
)


@dataclass(frozen=True)
class StatusSnapshot:
    sequence: int
    modified_ns: int
    failure_class: str
    payload: Mapping[str, Any]


def read_status_snapshot(path: Path) -> Optional[StatusSnapshot]:
    try:
        stat = path.stat()
        payload = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            return None
        failure_class = str(
            payload.get("localization_failure_class", "")
        ).strip().upper()
        return StatusSnapshot(
            sequence=int(payload.get("runtime_status_seq", 0)),
            modified_ns=int(stat.st_mtime_ns),
            failure_class=failure_class,
            payload=payload,
        )
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return None


def status_is_fresh(
    current: Optional[StatusSnapshot],
    baseline: Optional[StatusSnapshot],
) -> bool:
    if current is None:
        return False
    if baseline is None:
        return True
    return (
        current.modified_ns != baseline.modified_ns
        or current.sequence != baseline.sequence
    )


def classified_exit_code(
    child_return_code: int,
    failure_class: str,
    fresh_status_seen: bool,
) -> int:
    failure = (failure_class or "").strip().upper()
    if failure in NONRECOVERABLE_FAILURES:
        return 78
    if child_return_code != 0 and failure in RECOVERABLE_FAILURES:
        return 75
    if child_return_code != 0 and not fresh_status_seen:
        return 78
    return int(child_return_code)


def atomic_write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(payload, sort_keys=True, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    os.replace(temporary, path)


def terminate_child(process: subprocess.Popen[Any], timeout_sec: float) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except (AttributeError, OSError):
        process.terminate()
    try:
        process.wait(timeout=max(0.1, timeout_sec))
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except (AttributeError, OSError):
        process.kill()
    process.wait()


def supervise(
    command: Sequence[str],
    runtime_status: Path,
    evidence_file: Path,
    poll_sec: float,
    terminate_timeout_sec: float,
) -> int:
    baseline = read_status_snapshot(runtime_status)
    process = subprocess.Popen(list(command), start_new_session=True)
    forwarded_signal: list[int] = []

    def forward(signum: int, _frame: Any) -> None:
        forwarded_signal.append(signum)
        if process.poll() is None:
            try:
                os.killpg(process.pid, signum)
            except (AttributeError, OSError):
                process.send_signal(signum)

    previous_handlers = {}
    for signum in (signal.SIGTERM, signal.SIGINT):
        previous_handlers[signum] = signal.signal(signum, forward)

    fresh_snapshot: Optional[StatusSnapshot] = None
    try:
        while True:
            current = read_status_snapshot(runtime_status)
            if status_is_fresh(current, baseline):
                fresh_snapshot = current
                if current.failure_class in NONRECOVERABLE_FAILURES:
                    terminate_child(process, terminate_timeout_sec)
                    atomic_write_json(
                        evidence_file,
                        {
                            "child_return_code": process.returncode,
                            "failure_class": current.failure_class,
                            "service_exit_code": 78,
                            "runtime_status_seq": current.sequence,
                            "wall_time": time.time(),
                        },
                    )
                    return 78
            child_return_code = process.poll()
            if child_return_code is not None:
                failure_class = (
                    fresh_snapshot.failure_class if fresh_snapshot else ""
                )
                exit_code = classified_exit_code(
                    child_return_code,
                    failure_class,
                    fresh_snapshot is not None,
                )
                if child_return_code != 0:
                    atomic_write_json(
                        evidence_file,
                        {
                            "child_return_code": child_return_code,
                            "failure_class": failure_class
                            or "STARTUP_CONTRACT_UNCLASSIFIED",
                            "forwarded_signal": forwarded_signal[-1]
                            if forwarded_signal else None,
                            "service_exit_code": exit_code,
                            "runtime_status_seq": fresh_snapshot.sequence
                            if fresh_snapshot else 0,
                            "wall_time": time.time(),
                        },
                    )
                return exit_code
            time.sleep(max(0.02, poll_sec))
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runtime-status", required=True, type=Path)
    parser.add_argument("--evidence-file", required=True, type=Path)
    parser.add_argument("--poll-sec", type=float, default=0.25)
    parser.add_argument("--terminate-timeout-sec", type=float, default=20.0)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    if args.command and args.command[0] == "--":
        args.command = args.command[1:]
    if not args.command:
        parser.error("a supervised command is required after --")
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    return supervise(
        args.command,
        args.runtime_status,
        args.evidence_file,
        args.poll_sec,
        args.terminate_timeout_sec,
    )


if __name__ == "__main__":
    raise SystemExit(main())
