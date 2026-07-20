#!/usr/bin/env bash
set -euo pipefail

duration_sec="${1:-3600}"
output_dir="${2:-server_soak_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$output_dir"
bag="$output_dir/safety_runtime.bag"
{
  echo "started_at=$(date --iso-8601=seconds)"
  echo "duration_sec=$duration_sec"
  echo "head=$(git rev-parse HEAD)"
  echo "use_sim_time=$(rosparam get /use_sim_time)"
} > "$output_dir/run.txt"

status=0
timeout --signal=INT --kill-after=15s "$duration_sec" rosbag record \
  -O "$bag" \
  /odom \
  /cargo_avoidance/safety_status \
  /cargo_avoidance/status_code \
  /cargo_avoidance/static_evidence_debug || status=$?

# GNU timeout uses 124 when the requested soak duration expires. That is the
# expected completion path; recorder failures retain their non-zero code.
if [[ "$status" -eq 124 ]]; then status=0; fi
echo "finished_at=$(date --iso-8601=seconds)" >> "$output_dir/run.txt"
echo "recorder_status=$status" >> "$output_dir/run.txt"
if [[ "$status" -ne 0 ]]; then
  echo "rosbag recorder failed status=$status" >&2
  exit "$status"
fi
if [[ ! -s "$bag" && ! -s "$bag.active" ]]; then
  echo "soak bag missing or empty" >&2
  exit 6
fi
echo "SOAK_RECORDING_COMPLETE bag=$bag"
