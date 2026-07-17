#!/usr/bin/env bash
set -euo pipefail

duration_sec="${1:-3600}"
output_dir="${2:-server_soak_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$output_dir"

echo "started_at=$(date --iso-8601=seconds)" > "$output_dir/run.txt"
echo "duration_sec=$duration_sec" >> "$output_dir/run.txt"
echo "head=$(git rev-parse HEAD)" >> "$output_dir/run.txt"
rosparam get /use_sim_time >> "$output_dir/run.txt"

timeout --signal=INT "$duration_sec" rosbag record \
  -O "$output_dir/safety_runtime.bag" \
  /odom \
  /cargo_avoidance/safety_status \
  /cargo_avoidance/status_code \
  /cargo_avoidance/static_evidence_debug || status=$?

status="${status:-0}"
echo "finished_at=$(date --iso-8601=seconds)" >> "$output_dir/run.txt"
echo "recorder_status=$status" >> "$output_dir/run.txt"
exit "$status"
