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

# doctor and status work without a prior run
if [[ "$action" == "doctor" ]]; then
  # Doctor ALWAYS uses an independent temp directory — never touches
  # .current_run or any real run directory.  This is a read-only diagnostic.
  doctor_dir="$(mktemp -d "$runs_root/.doctor_XXXXXX")"
  trap 'rm -rf "$doctor_dir"' EXIT
  run_id="doctor"
  run_dir="$doctor_dir"
  mkdir -p "$run_dir/logs" "$run_dir/samples" "$run_dir/reports"
elif [[ "$action" == "status" && -z "$run_id" ]]; then
  # status without a run: still produce valid output
  run_id=""
  run_dir=""
elif [[ -z "$run_id" ]]; then
  echo "No run-id and no current run. Use 'start' or '--run-id <id>'." >&2
  exit 2
else
  run_dir="$runs_root/$run_id"
fi

pid_file="$run_dir/monitor.pid"
lock_file="$run_dir/monitor.lock"
ready_file="$run_dir/reports/monitor_ready.json"
summary_file="$run_dir/reports/live_summary.json"

# ─── PID identity verification ───────────────────────────────────────────────
# Uses /proc/<pid>/stat starttime + /proc/sys/kernel/random/boot_id to avoid
# killing a recycled PID.
verify_pid_identity() {
  local pid="$1" expected_name="${2:-server_runtime_monitor}"
  [[ "$pid" =~ ^[0-9]+$ ]] || return 1
  local proc_dir="/proc/$pid"
  [[ -d "$proc_dir" ]] || return 1
  # check cmdline contains expected process name
  local cmdline
  cmdline="$(tr '\0' ' ' < "$proc_dir/cmdline" 2>/dev/null || echo "")"
  if ! echo "$cmdline" | grep -q "$expected_name"; then
    return 1
  fi
  return 0
}

get_boot_id() {
  cat /proc/sys/kernel/random/boot_id 2>/dev/null || echo "unknown"
}

get_process_start_ticks() {
  local pid="$1"
  awk '{print $22}' "/proc/$pid/stat" 2>/dev/null || echo ""
}

get_uptime_sec() {
  local pid="$1"
  local start_ticks clk_tck uptime_sec
  start_ticks="$(get_process_start_ticks "$pid")"
  [[ -n "$start_ticks" ]] || { echo "null"; return; }
  clk_tck="$(getconf CLK_TCK 2>/dev/null || echo 100)"
  # uptime = seconds since boot - process start time
  # /proc/uptime gives seconds since boot
  local boot_sec
  boot_sec="$(awk '{print int($1)}' /proc/uptime 2>/dev/null || echo 0)"
  local proc_start_sec=$(( start_ticks / clk_tck ))
  uptime_sec=$(( boot_sec - proc_start_sec ))
  if [[ "$uptime_sec" -ge 0 ]]; then
    echo "$uptime_sec"
  else
    echo "null"
  fi
}

monitor_pid() {
  [[ -f "$pid_file" ]] || return 1
  local pid
  pid="$(<"$pid_file")"
  [[ "$pid" =~ ^[0-9]+$ ]] || return 1
  # verify PID identity before trusting it
  kill -0 "$pid" 2>/dev/null || return 1
  verify_pid_identity "$pid" "server_runtime_monitor" || return 1
  printf '%s\n' "$pid"
}

# ─── doctor ────────────────────────────────────────────────────────────────────
# All doctor checks are collected as TSV lines and serialized to JSON by Python
# at the end. This avoids hand-rolled JSON with missing commas, unescaped
# strings, and trailing text.

DOCTOR_TEMP_DIR="${run_dir}/.doctor_checks"
DOCTOR_TSV="${DOCTOR_TEMP_DIR}/checks.tsv"

_run_doctor_checks() {
  mkdir -p "$DOCTOR_TEMP_DIR"
  :> "$DOCTOR_TSV"

  local persistent_root="${NDT_SLAM_DATA_ROOT:-$workspace/maps/live/current}"

  # helper: writes name<TAB>status<TAB>severity<TAB>detail<TAB>duration_ms
  _dc() {
    local name="$1" status="$2" severity="$3" detail="$4" duration_ms="${5:-0}"
    printf '%s\t%s\t%s\t%s\t%s\n' "$name" "$status" "$severity" "$detail" "$duration_ms" >> "$DOCTOR_TSV"
  }

  local t0 t1
  t0="$(date +%s%3N 2>/dev/null || echo 0)"

  # ── WORKSPACE_EXISTS ──
  if [[ -d "$workspace" && -f "$workspace/src/ndt_slam/package.xml" ]]; then
    _dc "WORKSPACE_EXISTS" "PASS" "BLOCKING" "$workspace"
  else
    _dc "WORKSPACE_EXISTS" "FAIL" "BLOCKING" "$workspace"
  fi

  # ── GIT_HEAD ──
  local git_head
  if git_head="$(git -C "$workspace" rev-parse HEAD 2>/dev/null)"; then
    _dc "GIT_HEAD" "PASS" "INFO" "${git_head:0:12}"
  else
    _dc "GIT_HEAD" "FAIL" "WARNING" "no git repo at workspace"
  fi

  # ── EXPECTED_SHA_MATCH ──
  if [[ -n "$expected_sha" ]]; then
    local actual
    actual="$(git -C "$workspace" rev-parse HEAD 2>/dev/null || echo "")"
    if [[ "$actual" == "$expected_sha" ]]; then
      _dc "EXPECTED_SHA_MATCH" "PASS" "BLOCKING" "$expected_sha"
    else
      _dc "EXPECTED_SHA_MATCH" "FAIL" "BLOCKING" "expected=$expected_sha actual=${actual:0:12}"
    fi
  else
    _dc "EXPECTED_SHA_MATCH" "PASS" "INFO" "(not configured)"
  fi

  # ── ROS_MASTER_REACHABLE ──
  t0="$(date +%s%3N 2>/dev/null || echo 0)"
  if timeout 3s rostopic list &>/dev/null; then
    t1="$(date +%s%3N 2>/dev/null || echo 0)"
    _dc "ROS_MASTER_REACHABLE" "PASS" "BLOCKING" "${ROS_MASTER_URI:-default}" "$(( t1 - t0 ))"
  else
    t1="$(date +%s%3N 2>/dev/null || echo 0)"
    _dc "ROS_MASTER_REACHABLE" "FAIL" "BLOCKING" "ROS_MASTER_URI=${ROS_MASTER_URI:-not set}" "$(( t1 - t0 ))"
  fi

  # ── ROS_PACKAGE_FOUND ──
  local pkg_path
  if pkg_path="$(rospack find ndt_slam 2>/dev/null)"; then
    _dc "ROS_PACKAGE_FOUND" "PASS" "BLOCKING" "$pkg_path"
  else
    _dc "ROS_PACKAGE_FOUND" "FAIL" "BLOCKING" "ndt_slam not in ROS_PACKAGE_PATH"
  fi

  # ── DEVEL_SETUP_EXISTS ──
  if [[ -f "$workspace/devel/setup.bash" ]]; then
    _dc "DEVEL_SETUP_EXISTS" "PASS" "BLOCKING" "$workspace/devel/setup.bash"
  else
    _dc "DEVEL_SETUP_EXISTS" "FAIL" "BLOCKING" "missing devel/setup.bash"
  fi

  # ── PYTHON_ROSPY_IMPORT ──
  if python3 -c "import rospy" &>/dev/null; then
    _dc "PYTHON_ROSPY_IMPORT" "PASS" "BLOCKING" "rospy available"
  else
    _dc "PYTHON_ROSPY_IMPORT" "FAIL" "BLOCKING" "cannot import rospy"
  fi

  # ── CARGO_STATUS_MESSAGE_IMPORT ──
  if python3 -c "from lidar_slam2_msgs.msg import CargoSafetyStatus" &>/dev/null; then
    _dc "CARGO_STATUS_MESSAGE_IMPORT" "PASS" "BLOCKING" "lidar_slam2_msgs available"
  else
    _dc "CARGO_STATUS_MESSAGE_IMPORT" "FAIL" "BLOCKING" "cannot import CargoSafetyStatus"
  fi

  # ── RUN_DIR_WRITABLE ──
  if mkdir -p "$run_dir/logs" "$run_dir/samples" "$run_dir/reports" 2>/dev/null && [[ -w "$run_dir" ]]; then
    _dc "RUN_DIR_WRITABLE" "PASS" "BLOCKING" "$run_dir"
  else
    _dc "RUN_DIR_WRITABLE" "FAIL" "BLOCKING" "$run_dir"
  fi

  # ── PERSISTENT_ROOT_EXISTS ──
  if [[ -d "$persistent_root" ]]; then
    _dc "PERSISTENT_ROOT_EXISTS" "PASS" "WARNING" "$persistent_root"
  else
    _dc "PERSISTENT_ROOT_EXISTS" "FAIL" "WARNING" "$persistent_root not found"
  fi

  # ── RUNTIME_STATUS_LOCATION ──
  local runtime_status="$persistent_root/runtime_status.json"
  if [[ -r "$runtime_status" ]]; then
    _dc "RUNTIME_STATUS_LOCATION" "PASS" "INFO" "readable, $(stat --format=%s "$runtime_status") bytes"
  else
    _dc "RUNTIME_STATUS_LOCATION" "FAIL" "WARNING" "$runtime_status not readable"
  fi

  # ── CONFIG_PARSE_OK ──
  if [[ -z "$config" ]]; then
    config="$(rospack find ndt_slam 2>/dev/null || echo "")/config/server_monitor.yaml"
  fi
  if [[ -f "$config" ]] && python3 -c "import yaml; yaml.safe_load(open('$config'))" &>/dev/null; then
    _dc "CONFIG_PARSE_OK" "PASS" "BLOCKING" "$config"
  else
    _dc "CONFIG_PARSE_OK" "FAIL" "BLOCKING" "${config:-not found}"
  fi

  # ── TOPIC CHECKS (only if ROS master reachable) ──
  if timeout 3s rostopic list &>/dev/null; then
    local topics
    topics="$(timeout 3s rostopic list 2>/dev/null || echo "")"
    for topic in /odom /cargo_avoidance/safety_status /cargo_avoidance/status_code; do
      if echo "$topics" | grep -qxF "$topic"; then
        local ttype
        ttype="$(timeout 3s rostopic type "$topic" 2>/dev/null || echo "unknown")"
        _dc "REQUIRED_TOPIC: $topic" "PASS" "BLOCKING" "$ttype"
      else
        _dc "REQUIRED_TOPIC: $topic" "FAIL" "BLOCKING" "not publishing"
      fi
    done
    if echo "$topics" | grep -qxF "/cargo_avoidance/static_evidence_debug"; then
      _dc "OPTIONAL_TOPIC: static_evidence_debug" "PASS" "INFO" "available"
    else
      _dc "OPTIONAL_TOPIC: static_evidence_debug" "PASS" "INFO" "(not present, optional)"
    fi
  else
    for topic in /odom /cargo_avoidance/safety_status /cargo_avoidance/status_code; do
      _dc "REQUIRED_TOPIC: $topic" "SKIP" "BLOCKING" "ROS master not reachable"
    done
    _dc "OPTIONAL_TOPIC: static_evidence_debug" "SKIP" "INFO" "ROS master not reachable"
  fi

  # ── TOPIC_RATE_AVAILABLE (timeout-guarded, never hangs) ──
  if timeout 3s rostopic list 2>/dev/null | grep -qxF "/odom"; then
    local hz_raw hz
    hz_raw="$(timeout 5s rostopic hz /odom -w 5 2>/dev/null || echo "")"
    hz="$(echo "$hz_raw" | grep -oP 'average:\s*\K[0-9.]+' | tail -1 || echo "")"
    if [[ -n "$hz" && "$hz" != "0" ]]; then
      _dc "TOPIC_RATE_AVAILABLE" "PASS" "INFO" "/odom ~${hz} Hz"
    else
      _dc "TOPIC_RATE_AVAILABLE" "FAIL" "WARNING" "/odom rate unavailable (timeout or no messages)"
    fi
  else
    _dc "TOPIC_RATE_AVAILABLE" "SKIP" "INFO" "no ROS master or /odom not present"
  fi

  # ── NO_DUPLICATE_MONITOR_PROCESS ──
  local dupes
  dupes="$(pgrep -f "server_runtime_monitor.py" 2>/dev/null | wc -l || true)"
  if [[ "$dupes" -eq 0 ]]; then
    _dc "NO_DUPLICATE_MONITOR_PROCESS" "PASS" "WARNING" "no existing monitor"
  elif [[ "$dupes" -eq 1 ]]; then
    _dc "NO_DUPLICATE_MONITOR_PROCESS" "PASS" "INFO" "1 existing monitor (this is ok)"
  else
    _dc "NO_DUPLICATE_MONITOR_PROCESS" "FAIL" "WARNING" "$dupes monitor processes running"
  fi

  # ── LOCK_AVAILABLE ──
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
      _dc "LOCK_AVAILABLE" "PASS" "WARNING" "lock file not held"
    else
      _dc "LOCK_AVAILABLE" "FAIL" "WARNING" "lock held by another process"
    fi
  else
    _dc "LOCK_AVAILABLE" "PASS" "INFO" "no lock file yet"
  fi

  # ── DISK_SPACE_OK ──
  local disk_gb
  disk_gb="$(df -BG "$workspace" 2>/dev/null | awk 'NR==2 {print $4}' | sed 's/G//')"
  if [[ -n "$disk_gb" && "$disk_gb" -ge 20 ]]; then
    _dc "DISK_SPACE_OK" "PASS" "WARNING" "${disk_gb} GB available"
  elif [[ -n "$disk_gb" ]]; then
    _dc "DISK_SPACE_OK" "FAIL" "WARNING" "only ${disk_gb} GB available (< 20 GB)"
  else
    _dc "DISK_SPACE_OK" "SKIP" "WARNING" "cannot determine"
  fi

  # ── MAP_TILES_PRESENT ──
  local tile_count=0
  for layer in tiles_registration tiles_display tiles_ground tiles_objects; do
    local layer_dir="$persistent_root/$layer"
    if [[ -d "$layer_dir" ]]; then
      tile_count=$(( tile_count + $(find "$layer_dir" -maxdepth 1 -name '*.pcd' 2>/dev/null | wc -l) ))
    fi
  done
  if [[ "$tile_count" -gt 0 ]]; then
    _dc "MAP_TILES_PRESENT" "PASS" "INFO" "$tile_count PCD tiles across 4 layers"
  else
    _dc "MAP_TILES_PRESENT" "SKIP" "INFO" "no map tiles found (first run or no persistent map)"
  fi

  # ── STATIC_MANIFEST_STATE ──
  if [[ -f "$persistent_root/static_evidence_manifest.json" ]]; then
    _dc "STATIC_MANIFEST_STATE" "PASS" "INFO" "MANIFEST_ACTIVE"
  elif [[ -f "$persistent_root/static_evidence_manifest.last_good.json" ]]; then
    _dc "STATIC_MANIFEST_STATE" "PASS" "INFO" "MANIFEST_LAST_GOOD"
  elif [[ -f "$persistent_root/static_evidence_manifest.suspended" ]]; then
    _dc "STATIC_MANIFEST_STATE" "FAIL" "WARNING" "MANIFEST_SUSPENDED"
  elif [[ "$tile_count" -gt 0 ]]; then
    _dc "STATIC_MANIFEST_STATE" "FAIL" "WARNING" "TILES_ACTIVE_NO_MANIFEST: $tile_count tiles but no manifest"
  else
    _dc "STATIC_MANIFEST_STATE" "PASS" "INFO" "EMPTY_FIRST_RUN"
  fi
}

run_doctor() {
  _run_doctor_checks

  if $json_output; then
    # Use Python to build valid JSON from the TSV file
    # Returns exit code: 0=PASS, 10=DEGRADED, 1=FAIL
    local doctor_rc=0
    python3 - "$DOCTOR_TSV" "$(date +%s.%3N 2>/dev/null || date +%s)" <<'PY'
import json, sys, time

tsv_path = sys.argv[1]
generated_at = float(sys.argv[2])
checks = []

with open(tsv_path, 'r', encoding='utf-8') as fh:
    for line in fh:
        line = line.rstrip('\n')
        if not line:
            continue
        parts = line.split('\t')
        if len(parts) < 5:
            continue
        name, status, severity, detail, duration_ms = parts[0], parts[1], parts[2], parts[3], parts[4]
        check = {
            "name": name,
            "status": status,
            "severity": severity,
            "detail": detail,
        }
        try:
            check["duration_ms"] = int(duration_ms)
        except ValueError:
            check["duration_ms"] = 0
        checks.append(check)

failures = sum(1 for c in checks if c["status"] == "FAIL")
warnings = sum(1 for c in checks if c["status"] == "FAIL" and c["severity"] != "BLOCKING" or c["status"] == "SKIP")
blocking_failures = sum(1 for c in checks if c["status"] == "FAIL" and c["severity"] == "BLOCKING")

if blocking_failures > 0:
    overall = "FAIL"
    exit_code = 1
elif failures > 0:
    overall = "DEGRADED"
    exit_code = 10
else:
    overall = "PASS"
    exit_code = 0

payload = {
    "schema_version": 2,
    "overall": overall,
    "generated_at": generated_at,
    "failures": failures,
    "blocking_failures": blocking_failures,
    "warnings": warnings,
    "checks": checks,
}
print(json.dumps(payload, ensure_ascii=False, indent=2))
sys.exit(exit_code)
PY
    doctor_rc=$?
    # Clean up temp dir
    rm -rf "$DOCTOR_TEMP_DIR"
    return $doctor_rc
  else
    # Text mode
    local pass_count=0 fail_count=0 skip_count=0 blocking_fail_count=0
    while IFS=$'\t' read -r name status severity detail duration_ms; do
      printf '%-45s %-6s [%-8s] %s\n' "$name" "$status" "$severity" "$detail"
      case "$status" in
        PASS) pass_count=$((pass_count + 1)) ;;
        FAIL)
          fail_count=$((fail_count + 1))
          if [[ "$severity" == "BLOCKING" ]]; then
            blocking_fail_count=$((blocking_fail_count + 1))
          fi
          ;;
        SKIP) skip_count=$((skip_count + 1)) ;;
      esac
    done < "$DOCTOR_TSV"
    echo ""
    echo "Doctor: $pass_count PASS, $fail_count FAIL ($blocking_fail_count BLOCKING), $skip_count SKIP"
    rm -rf "$DOCTOR_TEMP_DIR"
    if [[ "$blocking_fail_count" -gt 0 ]]; then
      echo "Doctor found $blocking_fail_count BLOCKING failure(s). Fix before starting." >&2
      return 1
    elif [[ "$fail_count" -gt 0 ]]; then
      echo "Doctor found $fail_count non-blocking failure(s). Monitor can start in DEGRADED mode." >&2
      return 0
    fi
    echo "All doctor checks passed."
    return 0
  fi
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
    echo "monitor already running pid=$existing_pid run_id=$run_id"
    return 0
  fi

  # Clean up any stale PID file whose process is gone or not a monitor
  if [[ -f "$pid_file" ]]; then
    local stale_pid
    stale_pid="$(<"$pid_file")"
    if [[ "$stale_pid" =~ ^[0-9]+$ ]]; then
      if ! verify_pid_identity "$stale_pid" "server_runtime_monitor"; then
        echo "Removing stale PID file (pid=$stale_pid not a monitor process)" >&2
        rm -f "$pid_file"
      fi
    else
      rm -f "$pid_file"
    fi
  fi

  # Clean stale readiness files so they don't fool readiness check
  rm -f "$ready_file" "$run_dir/reports/live_summary.json"

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
  local start_requested_at
  start_requested_at="$(date +%s.%3N 2>/dev/null || date +%s)"
  nohup rosrun ndt_slam server_runtime_monitor.py "${args[@]}" \
    >>"$run_dir/logs/monitor.stdout.log" 2>&1 &
  local pid=$!
  printf '%s\n' "$pid" > "$pid_file"
  printf '%s\n' "$run_id" > "$current_file"
  local boot_id
  boot_id="$(get_boot_id)"

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
      # Verify ready file belongs to THIS process and was created after start
      local ready_ok
      ready_ok="$(python3 - "$ready_file" "$pid" "$run_id" "$boot_id" "$start_requested_at" <<'PY'
import json, sys, os, time
ready_path = sys.argv[1]
expected_pid = int(sys.argv[2])
expected_run_id = sys.argv[3]
expected_boot_id = sys.argv[4]
start_requested_at = float(sys.argv[5])

try:
    d = json.load(open(ready_path, 'r', encoding='utf-8'))
except Exception:
    print("false")
    sys.exit(0)

# Must be marked ready
if not d.get("ready", False):
    print("false")
    sys.exit(0)

# PID must match
ready_pid = d.get("pid", 0)
if int(ready_pid) != expected_pid:
    print("false")
    sys.exit(0)

# run_id must match (if present)
ready_run = d.get("run_id", "")
if ready_run and ready_run != expected_run_id:
    print("false")
    sys.exit(0)

# boot_id must match (if present)
ready_boot = d.get("boot_id", "")
if ready_boot and ready_boot != expected_boot_id:
    print("false")
    sys.exit(0)

# ready file created_at must be >= start_requested_at
created = d.get("created_at", 0)
if created > 0 and created < start_requested_at:
    print("false")
    sys.exit(0)

# Verify process_start_ticks if present in ready file
ready_ticks = d.get("process_start_ticks")
if ready_ticks is not None:
    try:
        actual_ticks = int(open('/proc/%d/stat' % expected_pid).read().split()[21])
        if ready_ticks != actual_ticks:
            print("false")
            sys.exit(0)
    except Exception:
        pass

print("true")
PY
)"
      if [[ "$ready_ok" == "true" ]]; then
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
  local pid
  if ! pid="$(monitor_pid)"; then
    # Check if there's a PID file but process is not a monitor
    if [[ -f "$pid_file" ]]; then
      local stale_pid
      stale_pid="$(<"$pid_file")"
      if [[ "$stale_pid" =~ ^[0-9]+$ ]] && ! verify_pid_identity "$stale_pid" "server_runtime_monitor"; then
        echo "PID file exists ($stale_pid) but process is not a monitor — cleaning up"
        rm -f "$pid_file" "$ready_file"
        return 0
      fi
    fi
    echo "monitor not running run_id=$run_id"
  else
    # Verify identity before killing
    if ! verify_pid_identity "$pid" "server_runtime_monitor"; then
      echo "PID $pid does not belong to monitor — cleaning up stale files" >&2
      rm -f "$pid_file" "$ready_file"
      return 0
    fi
    kill -TERM "$pid"
    for _ in $(seq 1 50); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.2
    done
    if kill -0 "$pid" 2>/dev/null; then
      echo "monitor did not stop cleanly pid=$pid" >&2
      return 5
    fi
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
  # When no run exists, output minimal valid status
  if [[ -z "$run_dir" || -z "$run_id" ]]; then
    local boot_id
    boot_id="$(get_boot_id)"
    python3 - "$boot_id" <<'PY'
import json, sys
boot_id = sys.argv[1]
payload = {
    "state": "NOT_CONFIGURED",
    "pid": None, "pid_valid": False, "boot_id": boot_id,
    "run_id": None, "run_dir": None, "uptime_sec": None,
    "monitor_ready": False, "last_summary_age_sec": None,
    "writer_dropped": None, "current_code": None, "current_reason": None,
    "odom_hz": None, "runtime_status_stale": None, "safety_status_age_sec": None,
}
print(json.dumps(payload, ensure_ascii=False, indent=2))
PY
    return
  fi

  # Use Python to build valid JSON with proper escaping and correct values
  python3 - "$pid_file" "$run_id" "$run_dir" "$ready_file" "$summary_file" <<'PY'
import json, os, sys, time

pid_file = sys.argv[1]
run_id = sys.argv[2]
run_dir = sys.argv[3]
ready_file = sys.argv[4]
summary_file = sys.argv[5]

def get_boot_id():
    try:
        return open('/proc/sys/kernel/random/boot_id').read().strip()
    except Exception:
        return None

def get_process_info(pid):
    try:
        stat = open(f'/proc/{pid}/stat').read().split()
        cmdline = open(f'/proc/{pid}/cmdline').read().replace('\0', ' ').strip()
        return True, int(stat[21]), cmdline
    except Exception:
        return False, None, None

def get_uptime_sec(pid):
    try:
        valid, start_ticks, _ = get_process_info(pid)
        if not valid:
            return None
        clk_tck = os.sysconf('SC_CLK_TCK')
        boot_sec = float(open('/proc/uptime').read().split()[0])
        proc_start_sec = start_ticks / clk_tck
        uptime = boot_sec - proc_start_sec
        return round(uptime, 1) if uptime >= 0 else None
    except Exception:
        return None

def read_pid_file():
    try:
        return open(pid_file).read().strip()
    except Exception:
        return None

pid_str = read_pid_file()
pid = None
run_state = "STOPPED"
pid_valid = False
boot_id = get_boot_id()

if pid_str and pid_str.isdigit():
    pid = int(pid_str)
    valid, start_ticks, cmdline = get_process_info(pid)
    if valid and 'server_runtime_monitor' in cmdline:
        pid_valid = True
        run_state = "RUNNING"
    else:
        run_state = "STALE"

ready = False
if os.path.exists(ready_file):
    try:
        ready_data = json.load(open(ready_file, 'r', encoding='utf-8'))
        ready = ready_data.get('ready', False)
    except Exception:
        pass

summary_data = {}
last_summary_age = None
if os.path.exists(summary_file):
    try:
        last_summary_age = int(time.time() - os.path.getmtime(summary_file))
        summary_data = json.load(open(summary_file, 'r', encoding='utf-8'))
    except Exception:
        pass

current = summary_data.get('current', {}) if summary_data else {}
runtime_info = summary_data.get('runtime', {}) if summary_data else {}
uptime_sec = get_uptime_sec(pid) if pid and pid_valid else None
safety_age = runtime_info.get('safety_status_age_sec')
odom_hz = runtime_info.get('odom_hz')

payload = {
    "state": run_state, "pid": pid if pid else None, "pid_valid": pid_valid,
    "boot_id": boot_id, "run_id": run_id, "run_dir": run_dir,
    "uptime_sec": uptime_sec, "monitor_ready": ready,
    "last_summary_age_sec": last_summary_age,
    "writer_dropped": summary_data.get('writer_dropped'),
    "current_code": current.get('code'),
    "current_reason": current.get('reason'),
    "odom_hz": odom_hz,
    "runtime_status_stale": runtime_info.get('runtime_status_stale'),
    "safety_status_age_sec": safety_age,
}
print(json.dumps(payload, ensure_ascii=False, indent=2))
PY
}

status_text() {
  if [[ -z "$run_dir" || -z "$run_id" ]]; then
    echo "STATE=NOT_CONFIGURED pid=none run_id=none"
    echo "No monitor has been started. Use 'server_monitorctl.sh start' to initialize."
    exit 1
  fi
  local pid="" pid_valid=false run_state="STOPPED"
  if pid="$(monitor_pid 2>/dev/null)"; then
    pid_valid=true
    run_state="RUNNING"
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
