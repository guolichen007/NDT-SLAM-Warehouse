#!/usr/bin/env bash
set -euo pipefail

[[ $# -ge 1 ]] || {
  echo "Usage: run_server_validation.sh <prepare|start|status|snapshot|stop|report|pack> [options]" >&2
  exit 2
}
action="$1"
shift
workspace="${NDT_SLAM_WORKSPACE:-$HOME/NDT-slam-ws}"
run_id=""
expected_sha=""
config=""
service="ndt-slam.service"
build_and_test=false
record_bag=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace) workspace="$2"; shift 2 ;;
    --run-id) run_id="$2"; shift 2 ;;
    --expected-sha) expected_sha="$2"; shift 2 ;;
    --config) config="$2"; shift 2 ;;
    --service) service="$2"; shift 2 ;;
    --build-and-test) build_and_test=true; shift ;;
    --record-bag) record_bag=true; shift ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
workspace="$(readlink -f "$workspace")"
runs_root="$workspace/server_runs"
current_file="$runs_root/.current_run"
mkdir -p "$runs_root"
if [[ -z "$run_id" && -f "$current_file" ]]; then run_id="$(<"$current_file")"; fi
if [[ -z "$run_id" && "$action" == "prepare" ]]; then
  run_id="$(date +%Y%m%d_%H%M%S)_$(git -C "$workspace" rev-parse --short=8 HEAD)"
fi
[[ -n "$run_id" ]] || { echo "run-id is required" >&2; exit 2; }
run_dir="$runs_root/$run_id"
mkdir -p "$run_dir"/{logs,samples,snapshots,reports,bags}
printf '%s\n' "$run_id" > "$current_file"
ops_dir="$(rospack find ndt_slam)/scripts/ops"
monitor_args=(--workspace "$workspace" --run-id "$run_id")
[[ -z "$config" ]] || monitor_args+=(--config "$config")
[[ -z "$expected_sha" ]] || monitor_args+=(--expected-sha "$expected_sha")

write_manifest() {
  local build_status="$1" test_status="$2"
  ACTUAL_SHA="$(git -C "$workspace" rev-parse HEAD)" \
  EXPECTED_SHA="$expected_sha" RUN_ID="$run_id" WORKSPACE="$workspace" \
  BUILD_STATUS="$build_status" TEST_STATUS="$test_status" RUN_DIR="$run_dir" \
  python3 -c 'import json,os,time,pathlib
p=pathlib.Path(os.environ["RUN_DIR"])/"run_manifest.json"
v=json.loads(p.read_text()) if p.exists() else {}
v.update({"run_id":os.environ["RUN_ID"],"created_at":v.get("created_at",time.time()),"actual_sha":os.environ["ACTUAL_SHA"],"expected_sha":os.environ["EXPECTED_SHA"],"workspace":os.environ["WORKSPACE"],"ubuntu_clean_build":os.environ["BUILD_STATUS"],"ubuntu_gtests":os.environ["TEST_STATUS"],"bag_validation":"NOT_RUN","server_soak":"NOT_RUN"})
t=p.with_suffix(".json.tmp"); t.write_text(json.dumps(v,indent=2,sort_keys=True)+"\n"); t.replace(p)'
}

case "$action" in
  prepare)
    [[ -n "$expected_sha" ]] || { echo "prepare requires --expected-sha" >&2; exit 2; }
    "$ops_dir/server_preflight.sh" --workspace "$workspace" \
      --expected-sha "$expected_sha" --phase prepare | tee "$run_dir/logs/preflight.log"
    build_status=NOT_RUN
    test_status=NOT_RUN
    if $build_and_test; then
      (
        cd "$workspace"
        source /opt/ros/noetic/setup.bash
        catkin clean -y
        catkin build --no-status
      ) 2>&1 | tee "$run_dir/logs/build.log"
      build_status=PASS
      (
        cd "$workspace"
        source devel/setup.bash
        catkin run_tests --no-status
        catkin_test_results --verbose
      ) 2>&1 | tee "$run_dir/logs/tests.log"
      test_status=PASS
    fi
    write_manifest "$build_status" "$test_status"
    echo "PREPARED run_id=$run_id dir=$run_dir"
    ;;
  start)
    [[ -n "$expected_sha" ]] || { echo "start requires --expected-sha" >&2; exit 2; }
    actual_sha="$(git -C "$workspace" rev-parse HEAD)"
    [[ "$actual_sha" == "$expected_sha" ]] || { echo "HEAD mismatch" >&2; exit 3; }
    if ! systemctl is-active --quiet "$service"; then
      systemctl start "$service"
    fi
    "$ops_dir/server_preflight.sh" --workspace "$workspace" \
      --expected-sha "$expected_sha" --phase live | tee -a "$run_dir/logs/preflight.log"
    rosparam dump "$run_dir/snapshots/rosparams.yaml"
    [[ ! -f "$workspace/maps/live/current/static_evidence_manifest.json" ]] ||
      cp -f "$workspace/maps/live/current/static_evidence_manifest.json" \
        "$run_dir/snapshots/manifest_start.json"
    [[ ! -f "$workspace/maps/live/current/static_evidence_manifest.last_good.json" ]] ||
      cp -f "$workspace/maps/live/current/static_evidence_manifest.last_good.json" \
        "$run_dir/snapshots/manifest_last_good_start.json"
    "$ops_dir/server_monitorctl.sh" start "${monitor_args[@]}"
    if $record_bag; then
      nohup rosbag record -O "$run_dir/bags/safety_runtime.bag" \
        /odom /cargo_avoidance/safety_status /cargo_avoidance/status_code \
        /cargo_avoidance/static_evidence_debug \
        >>"$run_dir/logs/rosbag.log" 2>&1 &
      printf '%s\n' "$!" > "$run_dir/bags/rosbag.pid"
    fi
    echo "STARTED run_id=$run_id"
    ;;
  status)
    "$ops_dir/server_monitorctl.sh" status "${monitor_args[@]}"
    systemctl --no-pager --full status "$service" || true
    ;;
  snapshot)
    "$ops_dir/server_monitorctl.sh" snapshot "${monitor_args[@]}"
    ;;
  stop)
    if [[ -f "$run_dir/bags/rosbag.pid" ]]; then
      bag_pid="$(<"$run_dir/bags/rosbag.pid")"
      kill -INT "$bag_pid" 2>/dev/null || true
      wait "$bag_pid" 2>/dev/null || true
      rm -f "$run_dir/bags/rosbag.pid"
    fi
    "$ops_dir/server_monitorctl.sh" stop "${monitor_args[@]}" || true
    journalctl -u "$service" --since "$(date -d @$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("created_at",0))' "$run_dir/run_manifest.json") --iso-8601=seconds)" \
      --no-pager > "$run_dir/logs/slam_journal.log" || true
    [[ ! -f "$workspace/maps/live/current/runtime_status.json" ]] ||
      cp -f "$workspace/maps/live/current/runtime_status.json" "$run_dir/snapshots/runtime_status_end.json"
    [[ ! -f "$workspace/maps/live/current/static_evidence_manifest.json" ]] ||
      cp -f "$workspace/maps/live/current/static_evidence_manifest.json" "$run_dir/snapshots/manifest_end.json"
    [[ ! -f "$workspace/maps/live/current/static_evidence_manifest.last_good.json" ]] ||
      cp -f "$workspace/maps/live/current/static_evidence_manifest.last_good.json" \
        "$run_dir/snapshots/manifest_last_good_end.json"
    find "$workspace/maps/live/current" -maxdepth 1 -type f \
      -name 'static_evidence_index_*' -printf '%f,%s,%T@\n' \
      > "$run_dir/snapshots/static_index_inventory.csv" || true
    rosrun ndt_slam summarize_server_run.py "$run_dir"
    echo "STOPPED_AND_REPORTED run_id=$run_id"
    ;;
  report)
    rosrun ndt_slam summarize_server_run.py "$run_dir"
    ;;
  pack)
    "$ops_dir/collect_server_artifacts.sh" "$run_dir"
    ;;
  *) echo "Unknown action: $action" >&2; exit 2 ;;
esac
