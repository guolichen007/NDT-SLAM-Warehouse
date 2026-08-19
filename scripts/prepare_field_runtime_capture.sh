#!/usr/bin/env bash
# Diagnostics-only pre-run helper: archive stale /tmp runtime data, create a
# fresh capture directory, and record build/source metadata for the field run.
#
# This script NEVER:
#   - controls the crane, sends ROS commands, or starts physical motion,
#   - launches roslaunch,
#   - deletes old data (it only moves it under the archive root).
#
# Usage:
#   scripts/prepare_field_runtime_capture.sh [repo_dir] [yaml_path ...]
set -euo pipefail

OLD="/tmp/ndt_slam_runtime_data"
ARCHIVE="/tmp/ndt_slam_runtime_archive"
REPO="${1:-$(pwd)}"
shift 2>/dev/null || true
YAML_PATHS=("$@")

mkdir -p "$ARCHIVE"

# Archive previous session data (never delete).
if [ -d "$OLD" ] && [ -n "$(ls -A "$OLD" 2>/dev/null)" ]; then
    TS="$(date --iso-8601=seconds)"
    mv "$OLD" "$ARCHIVE/${TS}_prelaunch"
    echo "Archived previous runtime data -> $ARCHIVE/${TS}_prelaunch"
fi

mkdir -p "$OLD"

METADATA="$OLD/prepare_metadata.txt"
{
    echo "prepare_stamp=$(date --iso-8601=seconds)"
    echo "git_head=$(git -C "$REPO" rev-parse HEAD)"
    echo "git_branch=$(git -C "$REPO" rev-parse --abbrev-ref HEAD)"
    echo "--- git status ---"
    git -C "$REPO" status --short
    echo "--- uname ---"
    uname -a
    echo "--- free -m ---"
    free -m
    echo "--- df -h ---"
    df -h
} > "$METADATA"

# Record each production YAML path and sha256 (read-only; never modifies YAML).
if [ "${#YAML_PATHS[@]}" -gt 0 ]; then
    {
        echo "--- production yaml ---"
        for yaml in "${YAML_PATHS[@]}"; do
            if [ -f "$yaml" ]; then
                echo "yaml_path=$yaml"
                echo "yaml_sha256=$(sha256sum "$yaml" | awk '{print $1}')"
            else
                echo "yaml_path=$yaml (MISSING)"
            fi
        done
    } >> "$METADATA"
fi

echo "Prepared field capture dir: $OLD"
echo "Metadata: $METADATA"
