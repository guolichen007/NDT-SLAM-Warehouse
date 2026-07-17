#!/usr/bin/env bash
set -euo pipefail

workspace="${1:-$HOME/NDT-slam-ws}"
map_root="${2:-$workspace/maps/live/current}"

cd "$workspace"
echo "HEAD=$(git rev-parse HEAD)"
git status --short
git submodule status --recursive

use_sim_time="$(rosparam get /use_sim_time 2>/dev/null || echo unavailable)"
echo "use_sim_time=$use_sim_time"
if [[ "$use_sim_time" == "true" || "$use_sim_time" == "True" ]]; then
  echo "ERROR: live server validation requires use_sim_time=false" >&2
  exit 2
fi

mkdir -p "$map_root"
test -w "$map_root"
df -h "$map_root"
free -m

manifest="$map_root/static_evidence_manifest.json"
if [[ -e "$manifest" ]]; then
  echo "static_manifest=$manifest"
else
  echo "static_manifest=missing_first_run_or_rebuilding"
fi
