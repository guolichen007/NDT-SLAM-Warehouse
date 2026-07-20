#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: server_monitorctl.sh <start|stop|restart|status|follow|snapshot|report|doctor>
  [--workspace PATH] [--run-id ID] [--config PATH]
  [--summary-sec N] [--windows 60,600] [--expected-sha SHA]
  [--json]
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
json_output=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace) workspace="$2"; shift 2 ;;
    --run-id) run_id="$2"; shift 2 ;;
    --config) config="$2"; shift 2 ;;
    --summary-sec) summary_sec="$2"; shift 2 ;;
    --windows) windows="$2"; shift 2 ;;
    --expected-sha) expected_sha="$2"; shift 2 ;;
    --json) json_output=true; shift ;;
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
  run_id="$(date +%Y%m%d_%H%M%S)_$(git -C "$workspace" rev-parse --short=8 HEAD 2>/dev/null || echo "nogit")"
fi
[[ -n "$run_id" ]] || { echo "No run-id and no current run" >&2; exit 2; }
run_dir="$runs_root/$run_id"
pid_file="$run_dir/monitor.pid"
lock_file="$run_dir/monitor.lock"
ready_file="$run_dir/reports/monitor_ready.json"
summary_file="$run_dir/reports/live_summary.json"

monitor_pid() {
  [[ -f "$pid_file" ]] || return 1
  local pid
  pid="$(<"$pid_file")"
  [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null || return 1
  printf '%s\n' "$pid"
}

# ─── doctor ────────────────────────────────────────────────────────────────────

doctor_check() {
  local label="$1" pass="$2" detail="$3"
  if $json_output; then
    printf '{"check":"%s","status":"%s","detail":"%s"}\n' "$label" "$pass" "$detail"
  else
    printf '%-40s %s  %s\n' "$label" "$([ "$pass" = "PASS" ] && echo "PASS" || echo "FAIL")" "$detail"
  fi
}

run_doctor() {
  local failures=0
  if $json_output; then echo "["; fi

  # WORKSPACE_EXISTS
  if [[ -d "$workspace" && -f "$workspace/src/ndt_slam/package.xml" ]]; then
    doctor_check "WORKSPACE_EXISTS" "PASS" "$workspace"
  else
    doctor_check "WORKSPACE_EXISTS" "FAIL" "$workspace"
    ((failures++)) || true
  fi

  # GIT_HEAD
  local git_head
  if git_head="$(git -C "$workspace" rev-parse HEAD 2>/dev/null)"; then
    doctor_check "GIT_HEAD" "PASS" "${git_head:0:12}"
  else
    doctor_check "GIT_HEAD" "FAIL" "no git repo at workspace"
    ((failures++)) || true
  fi

  # EXPECTED_SHA_MATCH
  if [[ -n "$expected_sha" ]]; then
    local actual
    actual="$(git -C "$workspace" rev-parse HEAD 2>/dev/null || echo "")"
    if [[ "$actual" == "$expected_sha" ]]; then
      doctor_check "EXPECTED_SHA_MATCH" "PASS" "$expected_sha"
    else
      doctor_check "EXPECTED_SHA_MATCH" "FAIL" "expected=$expected_sha actual=${actual:0:12}"
      ((failures++)) || true
    fi
  else
    doctor_check "EXPECTED_SHA_MATCH" "PASS" "(not configured)"
  fi

  # ROS_MASTER_REACHABLE
  if rostopic list &>/dev/null; then
    doctor_check "ROS_MASTER_REACHABLE" "PASS" "$ROS_MASTER_URI"
  else
    doctor_check "ROS_MASTER_REACHABLE" "FAIL" "ROS_MASTER_URI=$ROS_MASTER_URI"
    ((failures++)) || true
  fi

  # ROS_PACKAGE_FOUND
  if rospack find ndt_slam &>/dev/null; then
    doctor_check "ROS_PACKAGE_FOUND" "PASS" "$(rospack find ndt_slam)"
  else
    doctor_check "ROS_PACKAGE_FOUND" "FAIL" "ndt_slam not in ROS_PACKAGE_PATH"
    ((failures++)) || true
  fi

  # DEVEL_SETUP_EXISTS
  if [[ -f "$workspace/devel/setup.bash" ]]; then
    doctor_check "DEVEL_SETUP_EXISTS" "PASS" "$workspace/devel/setup.bash"
  else
    doctor_check "DEVEL_SETUP_EXISTS" "FAIL" "missing devel/setup.bash"
    ((failures++)) || true
  fi

  # PYTHON_ROSPY_IMPORT
  if python3 -c "import rospy; print(rospy.__file__)" &>/dev/null; then
    doctor_check "PYTHON_ROSPY_IMPORT" "PASS" "rospy available"
  else
    doctor_check "PYTHON_ROSPY_IMPORT" "FAIL" "cannot import rospy"
    ((failures++)) || true
  fi

  # CARGO_STATUS_MESSAGE_IMPORT
  if python3 -c "from lidar_slam2_msgs.msg import CargoSafetyStatus" &>/dev/null; then
    doctor_check "CARGO_STATUS_MESSAGE_IMPORT" "PASS" "lidar_slam2_msgs available"
  else
    doctor_check "CARGO_STATUS_MESSAGE_IMPORT" "FAIL" "cannot import CargoSafetyStatus"
    ((failures++)) || true
  fi

  # RUN_DIR_WRITABLE
  if mkdir -p "$run_dir/logs" "$run_dir/samples" "$run_dir/reports" 2>/dev/null && [[ -w "$run_dir" ]]; then
    doctor_check "RUN_DIR_WRITABLE" "PASS" "$run_dir"
  else
    doctor_check "RUN_DIR_WRITABLE" "FAIL" "$run_dir"
    ((failures++)) || true
  fi

  # PERSISTENT_ROOT_EXISTS
  local persistent_root="${NDT_SLAM_DATA_ROOT:-$workspace/maps/live/current}"
  if [[ -d "$persistent_root" ]]; then
    doctor_check "PERSISTENT_ROOT_EXISTS" "PASS" "$persistent_root"
  else
    doctor_check "PERSISTENT_ROOT_EXISTS" "FAIL" "$persistent_root"
    ((failures++)) || true
  fi

  # RUNTIME_STATUS_LOCATION
  local runtime_status="$persistent_root/runtime_status.json"
  if [[ -r "$runtime_status" ]]; then
    doctor_check "RUNTIME_STATUS_LOCATION" "PASS" "readable, $(stat --format=%s "$runtime_status") bytes"
  else
    doctor_check "RUNTIME_STATUS_LOCATION" "FAIL" "$runtime_status not readable"
    ((failures++)) || true
  fi

  # CONFIG_PARSE_OK
  if [[ -z "$config" ]]; then
    config="$(rospack find ndt_slam 2>/dev/null || echo "")/config/server_monitor.yaml"
  fi
  if [[ -f "$config" ]] && python3 -c "import yaml; yaml.safe_load(open('$config')); print('ok')" &>/dev/null; then
    doctor_check "CONFIG_PARSE_OK" "PASS" "$config"
  else
    doctor_check "CONFIG_PARSE_OK" "FAIL" "$config"
    ((failures++)) || true
  fi

  # REQUIRED_TOPIC checks (only if ROS master is reachable)
  if rostopic list &>/dev/null; then
    local topics
    topics="$(rostopic list 2>/dev/null || echo "")"
    for topic in /odom /cargo_avoidance/safety_status /cargo_avoidance/status_code; do
      if echo "$topics" | grep -qxF "$topic"; then
        local ttype
        ttype="$(rostopic type "$topic" 2>/dev/null || echo "unknown")"
        doctor_check "REQUIRED_TOPIC: $topic" "PASS" "$ttype"
      else
        doctor_check "REQUIRED_TOPIC: $topic" "FAIL" "not publishing"
        ((failures++)) || true
      fi
    done
    if echo "$topics" | grep -qxF "/cargo_avoidance/static_evidence_debug"; then
      doctor_check "OPTIONAL_TOPIC: static_evidence_debug" "PASS" "available"
    else
      doctor_check "OPTIONAL_TOPIC: static_evidence_debug" "PASS" "(not present, optional)"
    fi
  else
    for topic in /odom /cargo_avoidance/safety_status /cargo_avoidance/status_code; do
      doctor_check "REQUIRED_TOPIC: $topic" "SKIP" "ROS master not reachable"
    done
  fi

  # TOPIC_RATE_AVAILABLE (check odom rate if available)
  if rostopic list &>/dev/null && rostopic list 2>/dev/null | grep -qxF "/odom"; then
    local hz
    hz="$(rostopic hz /odom -w 2 2>/dev/null | grep -oP 'average: \K[0-9.]+' || echo "0")"
    if [[ "$hz" != "0" ]]; then
      doctor_check "TOPIC_RATE_AVAILABLE" "PASS" "/odom ~${hz} Hz"
    else
      doctor_check "TOPIC_RATE_AVAILABLE" "FAIL" "/odom rate unavailable"
      ((failures++)) || true
    fi
  else
    doctor_check "TOPIC_RATE_AVAILABLE" "SKIP" "no ROS master"
  fi

  # NO_DUPLICATE_MONITOR_PROCESS
  local dupes
  dupes="$(pgrep -f "server_runtime_monitor.py" 2>/dev/null | wc -l)"
  if [[ "$dupes" -eq 0 ]]; then
    doctor_check "NO_DUPLICATE_MONITOR_PROCESS" "PASS" "no existing monitor"
  elif [[ "$dupes" -eq 1 ]]; then
    doctor_check "NO_DUPLICATE_MONITOR_PROCESS" "PASS" "1 existing monitor (this is ok)"
  else
    doctor_check "NO_DUPLICATE_MONITOR_PROCESS" "FAIL" "$dupes monitor processes running"
    ((failures++)) || true
  fi

  # LOCK_AVAILABLE
  if [[ -f "$lock_file" ]]; then
    if python3 -c "
import fcntl, os, sys
try:
  fd = os.open('$lock_file', os.O_RDWR)
  fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
  fcntl.flock(fd, fcntl.LOCK_UN)
  os.close(fd)
  sys.exit(0)
except BlockingIOError:
  sys.exit(1)
except Exception:
  sys.exit(2)
" 2>/dev/null; then
      doctor_check "LOCK_AVAILABLE" "PASS" "lock file not held"
    else
      doctor_check "LOCK_AVAILABLE" "FAIL" "lock held by another process"
      ((failures++)) || true
    fi
  else
    doctor_check "LOCK_AVAILABLE" "PASS" "no lock file yet"
  fi

  # DISK_SPACE_OK
  local disk_gb
  disk_gb="$(df -BG "$workspace" 2>/dev/null | awk 'NR==2 {print $4}' | sed 's/G//')"
  if [[ -n "$disk_gb" && "$disk_gb" -ge 20 ]]; then
    doctor_check "DISK_SPACE_OK" "PASS" "${disk_gb} GB available"
  elif [[ -n "$disk_gb" ]]; then
    doctor_check "DISK_SPACE_OK" "FAIL" "only ${disk_gb} GB available (< 20 GB)"
    ((failures++)) || true
  else
    doctor_check "DISK_SPACE_OK" "SKIP" "cannot determine"
  fi

  if $json_output; then echo "]"; fi

  if [[ "$failures" -gt 0 ]]; then
    echo ""
    echo "Doctor found $failures failure(s). Fix before starting." >&2
    return 1
  fi
  echo ""
  echo "All doctor checks passed."
  return 0
}

# ─── start ──────────────────────────────────────────────────────────────────────

start_monitor() {
  # Run doctor first unless --skip-doctor
  run_doctor || {
    echo "Doctor checks failed. Use 'server_monitorctl.sh doctor' to diagnose." >&2
    return 3
  }

  if monitor_pid >/dev/null; then
    local existing_pid
    existing_pid="$(monitor_pid)"
    # Verify PID is not stale
    if [[ -r "/proc/$existing_pid/cmdline" ]]; then
      local cmdline
      cmdline="$(tr '\0' ' ' < "/proc/$existing_pid/cmdline" 2>/dev/null || echo "")"
      if echo "$cmdline" | grep -q "server_runtime_monitor"; then
        echo "monitor already running pid=$existing_pid run_id=$run_id"
        return 0
      fi
    fi
    # Stale PID — clean up
    echo "Removing stale PID file (pid=$existing_pid no longer monitor)" >&2
    rm -f "$pid_file"
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
  local pid=$!
  printf '%s\n' "$pid" > "$pid_file"
  printf '%s\n' "$run_id" > "$current_file"

  # Readiness wait (up to 15 seconds)
  local waited=0
  local ready=false
  while [[ $waited -lt 15 ]]; do
    sleep 1
    waited=$((waited + 1))
    if ! kill -0 "$pid" 2>/dev/null; then
      echo "" >&2
      echo "=== MONITOR FAILED TO START ===" >&2
      echo "pid=$pid" >&2
      echo "exit code: $(wait "$pid" 2>/dev/null || echo 'killed')" >&2
      echo "ROS_MASTER_URI=$ROS_MASTER_URI" >&2
      echo "" >&2
      echo "--- monitor.stdout.log (last 100 lines) ---" >&2
      tail -100 "$run_dir/logs/monitor.stdout.log" 2>/dev/null || true
      echo "" >&2
      echo "--- monitor.log (last 100 lines) ---" >&2
      tail -100 "$run_dir/logs/monitor.log" 2>/dev/null || true
      echo "" >&2
      echo "--- Python import check ---" >&2
      python3 -c "import rospy; print('rospy:', rospy.__file__)" 2>&1 || true
      python3 -c "from lidar_slam2_msgs.msg import CargoSafetyStatus; print('CargoSafetyStatus: ok')" 2>&1 || true
      return 4
    fi
    if [[ -f "$ready_file" ]]; then
      local ready_content
      ready_content="$(python3 -c "import json; d=json.load(open('$ready_file')); print(d.get('ready',False))" 2>/dev/null || echo "false")"
      if [[ "$ready_content" == "True" ]]; then
        ready=true
        break
      fi
    fi
  done

  if ! $ready; then
    echo "monitor did not become ready within 15s; check $run_dir/logs/" >&2
    echo "--- monitor.stdout.log (last 100 lines) ---" >&2
    tail -100 "$run_dir/logs/monitor.stdout.log" 2>/dev/null || true
    return 4
  fi

  echo "monitor started pid=$pid run_id=$run_id dir=$run_dir ready=yes"
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
  rm -f "$pid_file" "$ready_file"
}

# ─── status ─────────────────────────────────────────────────────────────────────

print_status() {
  if $json_output; then
    status_json
  else
    status_text
  fi
}

status_json() {
  local pid="" pid_valid=false run_state="STOPPED"
  if pid="$(monitor_pid 2>/dev/null)"; then
    pid_valid=true
    run_state="RUNNING"
    # Check stale
    if [[ -r "/proc/$pid/cmdline" ]]; then
      if ! tr '\0' ' ' < "/proc/$pid/cmdline" | grep -q "server_runtime_monitor"; then
        run_state="STALE"
        pid_valid=false
      fi
    else
      run_state="STALE"
      pid_valid=false
    fi
  fi
  local uptime="null" last_summary_age="null" ready="false" dropped="null"
  local code="null" reason="null" odom_hz="null" runtime_stale="null" safety_age="null"
  if [[ -f "$ready_file" ]]; then
    ready="$(python3 -c "import json; d=json.load(open('$ready_file')); print(d.get('ready',False))" 2>/dev/null || echo "false")"
  fi
  if [[ -f "$summary_file" ]]; then
    last_summary_age="$(python3 -c "import time,os; print(int(time.time()-os.path.getmtime('$summary_file')))" 2>/dev/null || echo "null")"
    code="$(python3 -c "import json; d=json.load(open('$summary_file')); c=d.get('current',{}); print(c.get('code','null'))" 2>/dev/null || echo "null")"
    reason="$(python3 -c "import json; d=json.load(open('$summary_file')); c=d.get('current',{}); print(c.get('reason','null'))" 2>/dev/null || echo "null")"
    dropped="$(python3 -c "import json; d=json.load(open('$summary_file')); print(d.get('writer_dropped','null'))" 2>/dev/null || echo "null")"
    odom_hz="$(python3 -c "import json; d=json.load(open('$summary_file')); r=d.get('runtime',{}); print(r.get('odom_hz','null'))" 2>/dev/null || echo "null")"
    runtime_stale="$(python3 -c "import json; d=json.load(open('$summary_file')); r=d.get('runtime',{}); print(r.get('runtime_status_stale','null'))" 2>/dev/null || echo "null")"
    safety_age="$(python3 -c "import json; d=json.load(open('$summary_file')); r=d.get('runtime',{}); print(r.get('runtime_status_age_sec','null'))" 2>/dev/null || echo "null")"
  fi
  local proc_start_ticks=""
  if [[ -n "$pid" && "$pid_valid" == "true" ]]; then
    proc_start_ticks="$(awk '{print $22}' "/proc/$pid/stat" 2>/dev/null || echo "")"
    # rough uptime in seconds
    local clk_tck uptime_sec
    clk_tck="$(getconf CLK_TCK 2>/dev/null || echo 100)"
    if [[ -n "$proc_start_ticks" ]]; then
      uptime="$(awk -v ticks="$proc_start_ticks" -v hz="$clk_tck" 'BEGIN{printf "%.0f", (systime() - ticks/hz)}' 2>/dev/null || echo "null")"
    fi
  fi
  cat <<EOJ
{
  "state": "$run_state",
  "pid": $([[ -n "$pid" ]] && echo "$pid" || echo "null"),
  "run_id": "$run_id",
  "run_dir": "$run_dir",
  "uptime_sec": $uptime,
  "monitor_ready": $ready,
  "last_summary_age_sec": $last_summary_age,
  "writer_dropped": $dropped,
  "current_code": $code,
  "current_reason": "$reason",
  "odom_hz": $odom_hz,
  "runtime_status_stale": $runtime_stale,
  "safety_status_age_sec": $safety_age
}
EOJ
}

status_text() {
  local pid="" pid_valid=false run_state="STOPPED"
  if pid="$(monitor_pid 2>/dev/null)"; then
    pid_valid=true
    run_state="RUNNING"
    if [[ -r "/proc/$pid/cmdline" ]]; then
      if ! tr '\0' ' ' < "/proc/$pid/cmdline" | grep -q "server_runtime_monitor"; then
        run_state="STALE"
        pid_valid=false
      fi
    else
      run_state="STALE"
    fi
  fi
  echo "STATE=$run_state pid=${pid:-none} run_id=$run_id dir=$run_dir"
  if [[ -f "$ready_file" ]]; then
    echo "monitor_ready=$(python3 -c "import json; print(json.load(open('$ready_file')).get('ready',False))" 2>/dev/null || echo false)"
  fi
  if [[ -f "$summary_file" ]]; then
    echo "last_summary_age_sec=$(python3 -c "import time,os; print(int(time.time()-os.path.getmtime('$summary_file')))" 2>/dev/null || echo ?)"
    echo "writer_dropped=$(python3 -c "import json; d=json.load(open('$summary_file')); print(d.get('writer_dropped','?'))" 2>/dev/null || echo ?)"
    echo "current_code=$(python3 -c "import json; d=json.load(open('$summary_file')); c=d.get('current',{}); print(c.get('code','?'))" 2>/dev/null || echo ?)"
    echo "current_reason=$(python3 -c "import json; d=json.load(open('$summary_file')); c=d.get('current',{}); print(c.get('reason','?'))" 2>/dev/null || echo ?)"
    echo "odom_hz=$(python3 -c "import json; d=json.load(open('$summary_file')); r=d.get('runtime',{}); print(r.get('odom_hz','?'))" 2>/dev/null || echo ?)"
    echo "runtime_status_stale=$(python3 -c "import json; d=json.load(open('$summary_file')); r=d.get('runtime',{}); print(r.get('runtime_status_stale','?'))" 2>/dev/null || echo ?)"
  fi
  if [[ "$run_state" == "RUNNING" ]]; then
    exit 0
  else
    exit 1
  fi
}

# ─── dispatch ────────────────────────────────────────────────────────────────────

case "$action" in
  start) start_monitor ;;
  stop) stop_monitor ;;
  restart) stop_monitor || true; start_monitor ;;
  status) print_status ;;
  doctor) run_doctor ;;
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
