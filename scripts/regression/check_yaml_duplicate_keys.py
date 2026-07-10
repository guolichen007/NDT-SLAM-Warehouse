#!/usr/bin/env python3
"""Fail when a YAML file contains duplicate top-level mapping keys.

This intentionally small checker uses only the Python standard library.  It is
not a full YAML parser; it scans column-zero block-mapping entries, which is the
form used by the project's ROS configuration files.
"""

import argparse
import json
from collections import defaultdict
from pathlib import Path
import sys
from typing import DefaultDict, Dict, List, Optional


def _normalise_key(raw_key: str) -> str:
    """Return a comparable value for plain and simply quoted YAML keys."""
    if len(raw_key) >= 2 and raw_key[0] == raw_key[-1] == "'":
        return raw_key[1:-1].replace("''", "'")
    if len(raw_key) >= 2 and raw_key[0] == raw_key[-1] == '"':
        try:
            return json.loads(raw_key)
        except json.JSONDecodeError:
            # Keep uncommon YAML-only escape sequences comparable verbatim.
            return raw_key
    return raw_key


def _top_level_key(line: str) -> Optional[str]:
    """Extract a column-zero block-mapping key, if this line defines one."""
    if not line or line[0].isspace():
        return None

    stripped = line.rstrip("\r\n")
    if not stripped or stripped.startswith(("#", "%", "---", "...", "- ", "? ")):
        return None

    quote = None  # type: Optional[str]
    escaped = False
    index = 0
    while index < len(stripped):
        character = stripped[index]

        if quote == '"':
            if escaped:
                escaped = False
            elif character == "\\":
                escaped = True
            elif character == quote:
                quote = None
        elif quote == "'":
            if character == quote:
                if index + 1 < len(stripped) and stripped[index + 1] == quote:
                    index += 1
                else:
                    quote = None
        elif character in ("'", '"'):
            quote = character
        elif character == ":":
            next_character = stripped[index + 1 : index + 2]
            if not next_character or next_character.isspace():
                raw_key = stripped[:index].strip()
                return _normalise_key(raw_key) if raw_key else None

        index += 1

    return None


def find_duplicate_keys(path: Path) -> Dict[str, List[int]]:
    occurrences = defaultdict(list)  # type: DefaultDict[str, List[int]]
    with path.open("r", encoding="utf-8-sig") as stream:
        for line_number, line in enumerate(stream, start=1):
            key = _top_level_key(line)
            if key is not None:
                occurrences[key].append(line_number)
    return {key: lines for key, lines in occurrences.items() if len(lines) > 1}


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check YAML files for duplicate top-level mapping keys."
    )
    parser.add_argument("files", metavar="FILE", nargs="+", type=Path)
    args = parser.parse_args()

    failed = False
    for path in args.files:
        try:
            duplicates = find_duplicate_keys(path)
        except (OSError, UnicodeError) as error:
            print(f"{path}: unable to read file: {error}", file=sys.stderr)
            failed = True
            continue

        if not duplicates:
            print(f"{path}: OK (no duplicate top-level keys)")
            continue

        failed = True
        for key, lines in sorted(duplicates.items()):
            locations = ", ".join(str(line) for line in lines)
            print(
                f"{path}: duplicate top-level key {key!r} at lines {locations}",
                file=sys.stderr,
            )

    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
