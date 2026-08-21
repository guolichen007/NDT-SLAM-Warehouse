#!/usr/bin/env bash
# Clean-build once, replay all four bags without fail-fast, then aggregate.
set -u

if [[ $# -lt 7 ]]; then
  echo "Usage: $0 WORKSPACE MAP_SOURCE OUTPUT_DIR 无.bag 有.bag 长件.bag 大件.bag [DURATION]" >&2
  exit 2
fi

workspace="$1"
map_source="$2"
output_dir="$3"
shift 3
bags=("$1" "$2" "$3" "$4")
names=("无" "有" "长件" "大件")
duration="${5:-1200}"
expected_sha="$(git -C "$workspace" rev-parse HEAD)"
runner="$workspace/src/ndt_slam/scripts/ops/server_monitor_bag_validate.sh"
analyzer="$workspace/src/ndt_slam/scripts/analysis/analyze_integrated_cargo_identity_shadow.py"

mkdir -p "$output_dir"
source /opt/ros/noetic/setup.bash
cd "$workspace"
catkin_make clean
catkin_make --pkg ndt_slam
build_rc=$?
catkin_make run_tests_ndt_slam
test_rc=$?
if [[ $build_rc -ne 0 || $test_rc -ne 0 ]]; then
  echo "BUILD_RC=$build_rc"
  echo "FULL_GTEST_RC=$test_rc"
  exit 1
fi

trace_args=()
runtime_args=()
run_rc=0
for index in 0 1 2 3; do
  name="${names[$index]}"
  bag="${bags[$index]}"
  port=$((11331 + index))
  run_log="$output_dir/${name}_replay.log"
  set +e
  "$runner" \
    --bag "$bag" \
    --mode full-chain \
    --workspace "$workspace" \
    --map-source "$map_source" \
    --duration "$duration" \
    --expected-sha "$expected_sha" \
    --ros-master-port "$port" >"$run_log" 2>&1
  bag_rc=$?
  set -e
  if [[ $bag_rc -ne 0 ]]; then
    run_rc=1
  fi
  trace="$output_dir/${name}_integrated_avoidance_shadow.csv"
  if [[ -f /tmp/cargo_forensic/integrated_avoidance_shadow.csv ]]; then
    cp /tmp/cargo_forensic/integrated_avoidance_shadow.csv "$trace"
    trace_args+=(--bag "$name=$trace")
  else
    echo "TRACE_MISSING bag=$name" >>"$run_log"
    run_rc=1
  fi
  run_dir="$(sed -n 's/^run_dir:[[:space:]]*//p' "$run_log" | tail -n 1)"
  runtime_csv="$run_dir/samples/runtime_samples.csv"
  if [[ -n "$run_dir" && -f "$runtime_csv" ]]; then
    runtime_copy="$output_dir/${name}_runtime_samples.csv"
    cp "$runtime_csv" "$runtime_copy"
    runtime_args+=(--runtime "$name=$runtime_copy")
  fi
done

set +e
python3 "$analyzer" "${trace_args[@]}" "${runtime_args[@]}" \
  --output "$output_dir/integrated_shadow_report.json"
analysis_rc=$?
set -e

echo "SOURCE_SHA=$expected_sha"
echo "BUILD_RC=$build_rc"
echo "FULL_GTEST_RC=$test_rc"
echo "REPLAY_RC=$run_rc"
echo "ANALYSIS_RC=$analysis_rc"
echo "PRODUCT_BEHAVIOR_CHANGED=NO"
echo "FIELD_READY=NO"

if [[ $build_rc -ne 0 || $run_rc -ne 0 || $analysis_rc -ne 0 ]]; then
  exit 1
fi
