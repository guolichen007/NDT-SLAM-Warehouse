#!/usr/bin/env python3
"""Run the repository's platform-neutral static contracts in CI order."""

from __future__ import annotations

import shlex
import subprocess
import sys
import time
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


def display_command(command: list[str]) -> str:
    return " ".join(shlex.quote(part) for part in command)


def run_step(name: str, command: list[str]) -> int:
    printable = display_command(command)
    started = time.monotonic()
    print(f"START name={name}", flush=True)
    print(f"command={printable}", flush=True)
    completed = subprocess.run(command, cwd=REPOSITORY_ROOT, check=False)
    elapsed = time.monotonic() - started
    outcome = "PASS" if completed.returncode == 0 else "FAIL"
    print(f"{outcome} name={name}", flush=True)
    print(f"exit_code={completed.returncode}", flush=True)
    print(f"elapsed_sec={elapsed:.3f}", flush=True)
    return completed.returncode


def main() -> int:
    python = sys.executable
    steps = [
        (
            "yaml_duplicate_keys",
            [python, "scripts/regression/check_yaml_duplicate_keys.py"],
        ),
        (
            "repository_integrity",
            [python, "scripts/regression/check_repository_integrity.py"],
        ),
        (
            "cargo_safety_e2e",
            [python, "scripts/regression/check_cargo_safety_e2e.py"],
        ),
        (
            "compileall",
            [python, "-m", "compileall", "-q", "scripts",
             "src/ndt_slam/scripts", "tests"],
        ),
        (
            "unittest_discover",
            [python, "-m", "unittest", "discover", "-v"],
        ),
        ("git_diff_check", ["git", "diff", "--check"]),
    ]

    for name, command in steps:
        exit_code = run_step(name, command)
        if exit_code != 0:
            return exit_code
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
