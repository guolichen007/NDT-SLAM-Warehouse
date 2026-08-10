#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 4 ]]; then
  echo "usage: $0 PHASE CONTROL_MANIFEST CANDIDATE_MANIFEST RESULT_DIR" >&2
  exit 2
fi

phase="$1"
control_manifest="$2"
candidate_manifest="$3"
result_dir="$4"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
mkdir -p "$result_dir"

python3 "$repo_root/scripts/regression/control_manifest.py" validate \
  "$control_manifest" --verify-files
python3 "$repo_root/scripts/regression/control_manifest.py" validate \
  "$candidate_manifest" --verify-files
python3 "$repo_root/scripts/regression/control_manifest.py" compare \
  "$control_manifest" "$candidate_manifest" --candidate-code

# The manifest is validated before rosbag starts, so a different bag, map,
# config, TF/extrinsic, SelfMask, initial persistent map, topic mapping or
# playback rate cannot be silently compared with the control.
case "$phase" in
  A|B|C|D) ;;
  *) echo "phase must be A, B, C, or D" >&2; exit 2 ;;
esac

echo "MANIFEST_LOCKED phase=$phase result_dir=$result_dir"
echo "Run the site-specific roslaunch + rosbag command, write runtime_metrics.json, then:"
echo "python3 $repo_root/scripts/regression/compare_runtime_results.py CONTROL_METRICS runtime_metrics.json"
