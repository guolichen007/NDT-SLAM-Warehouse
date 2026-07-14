#!/usr/bin/env python3
"""Reject tracked source/config files polluted by tool output or bad encoding."""

from __future__ import annotations

from pathlib import Path
import re
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
SCANNED_SUFFIXES = {
    ".c",
    ".cc",
    ".cmake",
    ".cpp",
    ".h",
    ".hpp",
    ".launch",
    ".msg",
    ".py",
    ".yaml",
    ".yml",
}
SKIPPED_PARTS = {
    ".git",
    "build",
    "devel",
    "install",
    "log",
    "test_artifacts",
}


def tracked_paths() -> list[Path]:
    completed = subprocess.run(
        ["git", "ls-files", "-z"],
        cwd=ROOT,
        check=True,
        stdout=subprocess.PIPE,
    )
    paths: list[Path] = []
    for raw_name in completed.stdout.split(b"\0"):
        if not raw_name:
            continue
        name = raw_name.decode("utf-8")
        relative = Path(name)
        if any(part in SKIPPED_PARTS for part in relative.parts):
            continue
        if (relative.name == "CMakeLists.txt" or
                relative.suffix.lower() in SCANNED_SUFFIXES):
            paths.append(relative)
    return paths


def pollution_patterns() -> list[tuple[str, re.Pattern[str]]]:
    literals = {
        "token truncation marker": "tokens " + "truncated",
        "response truncation header": "Response output was " + "truncated",
        "citation transport marker": "Citation " + "Marker:",
        "resource transport marker": "Resource " + "uri:",
        "tool output header": "Tool " + "output:",
        "tool line-count header": "Total output " + "lines:",
        "tool truncation warning": "Warning: truncated " + "output",
    }
    patterns = [
        (name, re.compile(re.escape(value), re.IGNORECASE))
        for name, value in literals.items()
    ]
    patterns.extend([
        (
            "partial line-range header",
            re.compile(r"Showing\s+\d+\s+of\s+\d+\s+lines", re.IGNORECASE),
        ),
        (
            "unicode token truncation marker",
            re.compile(r"\u2026\s*\d+\s+tokens\s+truncated\s*\u2026",
                       re.IGNORECASE),
        ),
        (
            "merge conflict marker",
            re.compile(r"^(?:<{7}|={7}|>{7})(?: .*)?$", re.MULTILINE),
        ),
    ])
    return patterns


def main() -> int:
    failures: list[str] = []
    try:
        paths = tracked_paths()
    except (OSError, subprocess.CalledProcessError, UnicodeDecodeError) as error:
        print(f"FAIL: cannot enumerate tracked files: {error}", file=sys.stderr)
        return 2

    patterns = pollution_patterns()
    scanned = 0
    for relative in paths:
        absolute = ROOT / relative
        if not absolute.is_file():
            failures.append(f"{relative.as_posix()}: tracked file is missing")
            continue
        scanned += 1
        try:
            payload = absolute.read_bytes()
        except OSError as error:
            failures.append(f"{relative.as_posix()}: cannot read: {error}")
            continue
        if b"\0" in payload:
            failures.append(f"{relative.as_posix()}: contains NUL byte")
            continue
        try:
            text = payload.decode("utf-8", errors="strict")
        except UnicodeDecodeError as error:
            failures.append(
                f"{relative.as_posix()}: invalid UTF-8 at byte {error.start}")
            continue
        for name, pattern in patterns:
            match = pattern.search(text)
            if match is None:
                continue
            line = text.count("\n", 0, match.start()) + 1
            failures.append(f"{relative.as_posix()}:{line}: {name}")

    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        print(
            f"FAIL: repository integrity scan found {len(failures)} issue(s) "
            f"in {scanned} tracked source/config file(s)",
            file=sys.stderr,
        )
        return 1

    print(
        f"PASS: repository integrity scan checked {scanned} tracked "
        "source/config file(s)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
