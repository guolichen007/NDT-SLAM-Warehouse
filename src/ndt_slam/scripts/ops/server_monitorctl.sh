#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: server_monitorctl.sh <start|stop|restart|status|follow|snapshot|report>
  [--workspace PATH] [--run-id ID] [--config PATH]
  [--summary-sec N] [--windows 60,600] [--expected-sha SHA]
EOF
}

[[ $# -ge 1 ]] || { usage; exit 2; }
action="$1"
shift
workspace="${NDT_SLAM_WORKSPACE:-$HOME/NDT-slam-ws}"
run_id=""
config=""
summary_sec=""
windows=""
expected_sha=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace) workspace="$2"; shift 2 ;;
    --run-id) run_id="$2"; shift 2 ;;
    --config) config="$2"; shift 2 ;;
    --summary-sec) summary_sec="$2"; shift 2 ;;
    --windows) windows="$2"; shift 2 ;;
    --expected-sha) expected_sha="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

workspace="$(readlink -f "$workspace")"
runs_root="$workspace/server_runs"
current_file="$runs_root/.current_run"
mkdir -p "$runs_root"
if [[ -z "$run_id" && -f "$current_file" ]]; then
  run_id="$(<"$current_file")"
fi
if [[ -z "$run_id" && "$action" == "start" ]]; then
  run_id="$(date +%Y%m%d_%H%M%S)_$(git -C "$workspace" rev-parse --short=8 HEAD)"
fi
[[ -n "$run_id" ]] || { echo "No run-id and no current run" >&2; exit 2; }
run_dir="$runs_root/$run_id"
pid_file="$run_dir/monitor.pid"
lock_file="$run_dir/monitor.lock"

monitor_pid() {
  [[ -f "$pid_file" ]] || return 1
  local pid
  pid="$(<"$pid_file")"
  [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null || return 1
  printf '%s\n' "$pid"
}

start_monitor() {
  if monitor_pid >/dev/null; then
    echo "monitor already running pid=$(monitor_pid) run_id=$run_id"
    return 0
  fi
  if [[ -n "$expected_sha" ]]; then
    actual_sha="$(git -C "$workspace" rev-parse HEAD)"
    [[ "$actual_sha" == "$expected_sha" ]] || {
      echo "HEAD mismatch expected=$expected_sha actual=$actual_sha" >&2
      return 3
    }
  fi
  mkdir -p "$run_dir/logs" "$run_dir/samples" "$run_dir/snapshots" "$run_dir/reports" "$run_dir/bags"
  if [[ -z "$config" ]]; then
    config="$(rospack find ndt_slam)/config/server_monitor.yaml"
  fi
  args=(--workspace "$workspace" --run-id "$run_id" --run-dir "$run_dir"
        --persistent-root "${NDT_SLAM_DATA_ROOT:-$workspace/maps/live/current}"
        --config "$config" --lock-file "$lock_file")
  [[ -z "$expected_sha" ]] || args+=(--expected-sha "$expected_sha")
  [[ -z "$windows" ]] || args+=(--windows "$windows")
  [[ -z "$summary_sec" ]] || args+=(--summary-sec "$summary_sec")
  nohup rosrun ndt_slam server_runtime_monitor.py "${args[@]}" \
    >>"$run_dir/logs/monitor.stdout.log" 2>&1 &
  pid=$!
  printf '%s\n' "$pid" > "$pid_file"
  printf '%s\n' "$run_id" > "$current_file"
  sleep 1
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "monitor failed to start; inspect $run_dir/logs/monitor.log" >&2
    return 4
  fi
  echo "monitor started pid=$pid run_id=$run_id dir=$run_dir"
}

stop_monitor() {
  if ! pid="$(monitor_pid)"; then
    echo "monitor not running run_id=$run_id"
  else
    kill -TERM "$pid"
    for _ in $(seq 1 50); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.2
    done
    kill -0 "$pid" 2>/dev/null && {
      echo "monitor did not stop cleanly pid=$pid" >&2
      return 5
    }
    echo "monitor stopped pid=$pid"
  fi
  rm -f "$pid_file"
}

case "$action" in
  start) start_monitor ;;
  stop) stop_monitor ;;
  restart) stop_monitor || true; start_monitor ;;
  status)
    if pid="$(monitor_pid)"; then
      echo "RUNNING pid=$pid run_id=$run_id dir=$run_dir"
      [[ -f "$run_dir/reports/live_summary.json" ]] &&
        python3 -m json.tool "$run_dir/reports/live_summary.json"
    else
      echo "STOPPED run_id=$run_id dir=$run_dir"
      exit 1
    fi
    ;;
  follow)
    touch "$run_dir/logs/monitor.log"
    exec tail -F "$run_dir/logs/monitor.log"
    ;;
  snapshot)
    pid="$(monitor_pid)" || { echo "monitor not running" >&2; exit 1; }
    kill -USR1 "$pid"
    echo "snapshot requested pid=$pid"
    ;;
  report)
    rosrun ndt_slam summarize_server_run.py "$run_dir"
    ;;
  *) usage; exit 2 ;;
esac
