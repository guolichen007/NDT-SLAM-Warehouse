#!/usr/bin/env bash
# Clean-build once, replay all four bags without fail-fast, then aggregate.
set -u

if [[ $# -lt 8 ]]; then
  echo "Usage: $0 WORKSPACE EXPECTED_SHA MAP_SOURCE OUTPUT_DIR 无.bag 有.bag 长件.bag 大件.bag [DURATION] [ORACLE_DIR] [BASELINE_TRACE_DIR] [YAW_REFERENCE]" >&2
  exit 2
fi

workspace="$1"
expected_sha="$2"
map_source="$3"
output_dir="$4"
shift 4
bags=("$1" "$2" "$3" "$4")
names=("无" "有" "长件" "大件")
duration="${5:-1200}"
oracle_dir="${6:-}"
baseline_trace_dir="${7:-}"
yaw_reference="${8:-}"
runner="$workspace/src/ndt_slam/scripts/ops/server_monitor_bag_validate.sh"
analyzer="$workspace/src/ndt_slam/scripts/analysis/analyze_integrated_cargo_identity_shadow.py"

actual_sha="$(git -C "$workspace" rev-parse HEAD)"
if [[ "$actual_sha" != "$expected_sha" ]]; then
  echo "SHA_GATE=FAIL expected=$expected_sha actual=$actual_sha" >&2
  exit 3
fi
if [[ -n "$(git -C "$workspace" status --porcelain --untracked-files=all)" ]]; then
  echo "WORKTREE_GATE=FAIL reason=not_clean" >&2
  git -C "$workspace" status --short >&2
  exit 4
fi
echo "SHA_GATE=PASS expected=$expected_sha"
echo "WORKTREE_GATE=PASS"

mkdir -p "$output_dir"
source /opt/ros/noetic/setup.bash
cd "$workspace"
catkin_make clean
catkin_make --pkg ndt_slam
build_rc=$?
catkin_make run_tests_ndt_slam
test_rc=$?
catkin_test_results --all --verbose "$workspace/build/test_results"
test_results_rc=$?
if [[ $build_rc -ne 0 || $test_rc -ne 0 || $test_results_rc -ne 0 ]]; then
  echo "BUILD_RC=$build_rc"
  echo "FULL_GTEST_RC=$test_rc"
  echo "CATKIN_TEST_RESULTS_RC=$test_results_rc"
  exit 1
fi

trace_args=()
group_args=()
baseline_group_args=()
baseline_args=()
runtime_args=()
oracle_args=()
run_rc=0
for index in 0 1 2 3; do
  name="${names[$index]}"
  bag="${bags[$index]}"
  port=$((11331 + index))
  run_log="$output_dir/${name}_replay.log"
  rm -f \
    /tmp/cargo_forensic/integrated_avoidance_shadow.csv \
    /tmp/cargo_forensic/integrated_identity_groups.csv
  trace_generation_marker="$output_dir/${name}_trace_generation.marker"
  touch "$trace_generation_marker"
  set +e
  runner_args=(
    --bag "$bag" \
    --mode full-chain \
    --workspace "$workspace" \
    --map-source "$map_source" \
    --duration "$duration" \
    --expected-sha "$expected_sha" \
    --ros-master-port "$port")
  if [[ -n "$yaw_reference" ]]; then
    runner_args+=(--yaw-reference "$yaw_reference")
  fi
  "$runner" "${runner_args[@]}" >"$run_log" 2>&1
  bag_rc=$?
  set -e
  if [[ $bag_rc -ne 0 ]]; then
    run_rc=1
  fi
  trace="$output_dir/${name}_integrated_avoidance_shadow.csv"
  source_trace=/tmp/cargo_forensic/integrated_avoidance_shadow.csv
  if [[ -f "$source_trace" && "$source_trace" -nt "$trace_generation_marker" ]]; then
    cp "$source_trace" "$trace"
    trace_args+=(--bag "$name=$trace")
  else
    echo "TRACE_MISSING bag=$name" >>"$run_log"
    run_rc=1
  fi
  group_trace="$output_dir/${name}_integrated_identity_groups.csv"
  source_group_trace=/tmp/cargo_forensic/integrated_identity_groups.csv
  if [[ -f "$source_group_trace" &&
        "$source_group_trace" -nt "$trace_generation_marker" ]]; then
    cp "$source_group_trace" "$group_trace"
    group_args+=(--groups "$name=$group_trace")
  else
    echo "GROUP_TRACE_MISSING bag=$name" >>"$run_log"
    run_rc=1
  fi
  baseline_group="$baseline_trace_dir/${name}_integrated_identity_groups.csv"
  if [[ -n "$baseline_trace_dir" && -f "$baseline_group" ]]; then
    baseline_group_args+=(--baseline-groups "$name=$baseline_group")
  fi
  baseline_trace="$baseline_trace_dir/${name}_integrated_avoidance_shadow.csv"
  if [[ -n "$baseline_trace_dir" && -f "$baseline_trace" ]]; then
    baseline_args+=(--baseline "$name=$baseline_trace")
  fi
  run_dir="$(sed -n 's/^run_dir:[[:space:]]*//p' "$run_log" | tail -n 1)"
  runtime_csv="$run_dir/samples/runtime_samples.csv"
  if [[ -n "$run_dir" && -f "$runtime_csv" ]]; then
    runtime_copy="$output_dir/${name}_runtime_samples.csv"
    cp "$runtime_csv" "$runtime_copy"
    runtime_args+=(--runtime "$name=$runtime_copy")
  fi
  oracle_file="$oracle_dir/${name}.json"
  if [[ -n "$oracle_dir" && -f "$oracle_file" ]]; then
    oracle_args+=(--oracle "$name=$oracle_file")
  else
    echo "ORACLE_INCONCLUSIVE bag=$name reason=bag_local_oracle_missing" \
      >>"$run_log"
  fi
done

set +e
python3 "$analyzer" "${trace_args[@]}" "${runtime_args[@]}" \
  "${group_args[@]}" "${baseline_group_args[@]}" "${baseline_args[@]}" \
  "${oracle_args[@]}" \
  --source-sha "$expected_sha" \
  --output "$output_dir/integrated_shadow_report.json" \
  --markdown-output "$output_dir/integrated_shadow_report.md"
analysis_rc=$?
set -e

echo "SOURCE_SHA=$expected_sha"
echo "V5_CANDIDATE_SHA=$expected_sha"
echo "BUILD_RC=$build_rc"
echo "FULL_GTEST_RC=$test_rc"
echo "CATKIN_TEST_RESULTS_RC=$test_results_rc"
echo "REPLAY_RC=$run_rc"
echo "ANALYSIS_RC=$analysis_rc"
echo "CARGO_V5_PRODUCT_TAKEOVER=NOT_YET"
echo "PRODUCT_LOGIC_CHANGED=NO"
echo "PRODUCT_OUTPUT_AUTHORITY_CHANGED=NO"
echo "FIELD_READY=NO"

if [[ $build_rc -ne 0 || $run_rc -ne 0 || $analysis_rc -ne 0 ]]; then
  exit 1
fi
