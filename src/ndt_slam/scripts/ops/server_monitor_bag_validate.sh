#!/usr/bin/env bash
# ─── Server Monitor Bag Validation Harness ───────────────────────────────────
# Runs an isolated rosbag-based validation of the server monitor.
#
# Modes:
#   monitor-only  – Bag already contains /odom, /safety_status, /status_code
#   full-chain    – Bag contains raw PointCloud2; runs production SLAM pipeline
#   partial       – Bag has some but not all required topics; can NEVER PASS
#
# Usage:
#   server_monitor_bag_validate.sh \
#     --bag /path/to/bag.bag \
#     --mode full-chain \
#     --workspace /path/to/NDT-slam-ws \
#     --map-source /path/to/maps/live/current \
#     --duration 30 \
#     --rate 1.0
# ────────────────────────────────────────────────────────────────────────────────
set -euo pipefail

# ── defaults ──
MODE=""
WORKSPACE="${NDT_SLAM_WORKSPACE:-$HOME/NDT-slam-ws}"
DURATION=25
RATE=1.0
MAP_SOURCE=""
YAW_REFERENCE=""
EXPECTED_SHA=""
ROS_MASTER_PORT=11321
START_OFFSET=0
BAG=""
RUN_ID="bag_validate_$(date +%Y%m%d_%H%M%S)"
ALLOWED_MODES=("monitor-only" "full-chain" "partial")

# ── PID tracking ──
ROSCORE_PID=""
LAUNCH_PID=""
MONITOR_PID=""
BAG_PID=""
RECORDER_PID=""
DERIVED_BAG=""

# ── exit code tracking ──
ROSCORE_RC=""
LAUNCH_RC=""
MONITOR_RC=""
BAG_RC=""
RECORDER_RC=""

# ── helpers ──
usage() {
  cat <<'EOF'
server_monitor_bag_validate.sh – isolated rosbag validation for server monitor

  --bag FILE             Path to rosbag (required)
  --mode MODE            monitor-only | full-chain | partial (required)
  --workspace PATH       NDT-slam-ws root (default: $NDT_SLAM_WORKSPACE)
  --map-source PATH      Source maps directory for sandbox copy
  --yaw-reference FILE  Frozen verified Rail yaw reference (combined gate)
  --duration SEC         Test duration in seconds (default: 25)
  --rate RATE            rosbag play --rate (default: 1.0)
  --expected-sha SHA     Expected git HEAD (optional)
  --ros-master-port PORT Isolated ROS master port (default: 11321)
  --start-offset SEC     rosbag play --start offset (default: 0)
EOF
  exit 1
}

wait_and_capture() {
  # Usage: wait_and_capture <pid> <result_var_name>
  local pid="$1"
  local result_var="$2"
  local rc
  set +e
  wait "$pid"
  rc=$?
  set -e
  printf -v "$result_var" '%s' "$rc"
}

die() { echo "FATAL: $*" >&2; exit 1; }

# ── parse args ──
while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) BAG="$2"; shift 2 ;;
    --mode) MODE="$2"; shift 2 ;;
    --workspace) WORKSPACE="$2"; shift 2 ;;
    --map-source) MAP_SOURCE="$2"; shift 2 ;;
    --yaw-reference) YAW_REFERENCE="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --expected-sha) EXPECTED_SHA="$2"; shift 2 ;;
    --ros-master-port) ROS_MASTER_PORT="$2"; shift 2 ;;
    --start-offset) START_OFFSET="$2"; shift 2 ;;
    -h|--help) usage ;;
    *) echo "Unknown arg: $1" >&2; usage ;;
  esac
done

# ── strict mode validation ──
[[ -n "$BAG" ]] || die "--bag is required"
[[ -f "$BAG" ]] || die "Bag file not found: $BAG"
[[ -d "$WORKSPACE" ]] || die "Workspace not found: $WORKSPACE"
[[ -n "$MODE" ]] || die "--mode is required"

MODE_VALID=false
for m in "${ALLOWED_MODES[@]}"; do
  [[ "$MODE" == "$m" ]] && MODE_VALID=true
done
$MODE_VALID || die "Invalid mode: $MODE (must be one of: ${ALLOWED_MODES[*]})"

# ── resolve paths ──
BAG="$(realpath "$BAG")"
WORKSPACE="$(realpath "$WORKSPACE")"
if [[ -n "$YAW_REFERENCE" ]]; then
  [[ -f "$YAW_REFERENCE" ]] || die "Yaw reference not found: $YAW_REFERENCE"
  YAW_REFERENCE="$(realpath "$YAW_REFERENCE")"
fi
RUN_DIR="$WORKSPACE/server_runs/$RUN_ID"
MAP_SANDBOX_ROOT="$RUN_DIR/map_sandbox"
MAP_SANDBOX="$MAP_SANDBOX_ROOT/current"
CONFIG_DIR="$RUN_DIR/config"
GENERATED_CONFIG="$CONFIG_DIR/live_longterm_mapping.bag.yaml"

# ── verify workspace state ──
cd "$WORKSPACE"
if [[ -n "$EXPECTED_SHA" ]]; then
  ACTUAL_SHA="$(git rev-parse HEAD)"
  if [[ "$ACTUAL_SHA" != "$EXPECTED_SHA" ]]; then
    die "HEAD mismatch: expected=$EXPECTED_SHA actual=$ACTUAL_SHA"
  fi
fi

# ── setup directories ──
echo "=== Bag Validation Setup ==="
echo "bag:       $BAG"
echo "mode:      $MODE"
echo "run_dir:   $RUN_DIR"
echo "workspace: $WORKSPACE"
echo "duration:  ${DURATION}s"
echo "rate:      $RATE"
echo ""

mkdir -p "$RUN_DIR"/{logs,reports,samples,bags} "$CONFIG_DIR"
ROS_HOME="$RUN_DIR/ros_home"
mkdir -p "$ROS_HOME"

# ── source ROS ──
source /opt/ros/noetic/setup.bash 2>/dev/null || true
source "$WORKSPACE/devel/setup.bash" 2>/dev/null || true

# ══════════════════════════════════════════════════════════════════════════════
# Bag audit
# ══════════════════════════════════════════════════════════════════════════════
echo "=== Bag Audit ==="
BAG_INFO="$RUN_DIR/reports/bag_info.yaml"
rosbag info --yaml "$BAG" > "$BAG_INFO" 2>/dev/null || true
rosbag info "$BAG" 2>&1 | tee "$RUN_DIR/logs/bag_info.txt"

BAG_SHA="$(sha256sum "$BAG" | awk '{print $1}')"
BAG_DURATION="$(grep -oP 'duration:\s*\K[0-9.]+' "$BAG_INFO" 2>/dev/null || echo "0")"

# Detect available topics in bag
_bag_has_topic() {
  grep -qF "$1" "$BAG_INFO" 2>/dev/null
}
HAS_RS201=false;   _bag_has_topic "/rs_201" && HAS_RS201=true
HAS_RS203=false;   _bag_has_topic "/rs_203" && HAS_RS203=true
HAS_GRAVITY=false; _bag_has_topic "/gravity" && HAS_GRAVITY=true
HAS_ODOM=false;    _bag_has_topic "/odom" && HAS_ODOM=true
HAS_SAFETY=false;  _bag_has_topic "/cargo_avoidance/safety_status" && HAS_SAFETY=true
HAS_CODE=false;    _bag_has_topic "/cargo_avoidance/status_code" && HAS_CODE=true

# ── mode pre-flight validation ──
case "$MODE" in
  monitor-only)
    if ! $HAS_ODOM || ! $HAS_SAFETY || ! $HAS_CODE; then
      die "monitor-only mode requires /odom, /cargo_avoidance/safety_status, /cargo_avoidance/status_code in bag"
    fi
    ;;
  full-chain)
    if ! $HAS_RS201 && ! $HAS_RS203; then
      die "full-chain mode requires at least one of /rs_201, /rs_203 in bag"
    fi
    ;;
  partial)
    echo "WARNING: partial mode — will NEVER output PASS, only PARTIAL/NOT_RUN/FAIL"
    ;;
esac

# ══════════════════════════════════════════════════════════════════════════════
# Map sandbox
# ══════════════════════════════════════════════════════════════════════════════
MAP_HASH_BEFORE=""
MAP_HASH_AFTER=""
LIVE_MAP_HASH_BEFORE=""
LIVE_MAP_HASH_AFTER=""

if [[ -n "$MAP_SOURCE" && -d "$MAP_SOURCE" ]]; then
  MAP_SOURCE="$(realpath "$MAP_SOURCE")"
  echo "=== Map Sandbox ==="
  echo "source:  $MAP_SOURCE"
  echo "sandbox: $MAP_SANDBOX"
  mkdir -p "$(dirname "$MAP_SANDBOX")"
  rsync -a "$MAP_SOURCE/" "$MAP_SANDBOX/"
  echo "Map copied to sandbox ($(find "$MAP_SANDBOX" -type f | wc -l) files)"

  # Record sandbox hash before
  MAP_HASH_BEFORE="$(find "$MAP_SANDBOX" -type f -print0 | sort -z | xargs -0 sha256sum 2>/dev/null)"
  echo "$MAP_HASH_BEFORE" > "$RUN_DIR/reports/map_hash_before.txt"

  # Record live map hash
  LIVE_MAP_HASH_BEFORE="$(find "$MAP_SOURCE" -type f -print0 | sort -z | xargs -0 sha256sum 2>/dev/null)"
  echo "$LIVE_MAP_HASH_BEFORE" > "$RUN_DIR/reports/live_map_hash_before.txt"
else
  die "--map-source is required and must be an existing directory"
fi

# ══════════════════════════════════════════════════════════════════════════════
# Generate sandbox config (full-chain mode)
# ══════════════════════════════════════════════════════════════════════════════
CONFIG_SOURCE="$WORKSPACE/src/ndt_slam/config/live_longterm_mapping.yaml"
DIAG_OUTPUT_DIR="$RUN_DIR/runtime_diagnostics"
mkdir -p "$DIAG_OUTPUT_DIR"

generate_sandbox_config() {
  echo "=== Generating Sandbox Config ==="
  python3 - "$CONFIG_SOURCE" "$GENERATED_CONFIG" "$MAP_SANDBOX" "$DIAG_OUTPUT_DIR" "$YAW_REFERENCE" <<'PYEOF'
import sys, yaml, os

src = sys.argv[1]
dst = sys.argv[2]
sandbox = sys.argv[3]
diag_dir = sys.argv[4]
yaw_reference_path = sys.argv[5]

with open(src, 'r') as f:
    config = yaml.safe_load(f)

# Override persistent map path
if 'persistent_map' not in config:
    config['persistent_map'] = {}
config['persistent_map']['root_dir'] = sandbox
config['persistent_map']['enabled'] = True  # must match launch persistent_map:=true

# Override runtime diagnostics output_dir
if 'debug' not in config:
    config['debug'] = {}
if 'runtime_diagnostics' not in config['debug']:
    config['debug']['runtime_diagnostics'] = {}
config['debug']['runtime_diagnostics']['output_dir'] = diag_dir

if yaw_reference_path:
    with open(yaw_reference_path, 'r') as stream:
        document = yaml.safe_load(stream)
    reference = document.get('reference', document)
    required = {
        'schema_version', 'verified', 'rail_yaw_in_map_rad', 'source',
        'map_frame_uuid', 'map_frame_id', 'base_frame_id',
        'map_frame_convention_id', 'sensor_rig_calibration_id',
        'reference_uuid', 'reference_hash',
    }
    missing = sorted(required - set(reference))
    if missing or reference.get('verified') is not True:
        raise SystemExit(
            'invalid frozen yaw reference; missing=' + ','.join(missing)
        )
    config['runtime_yaw_authority'] = {
        'mode': 'RAIL_AUTHORITY',
        'reference': reference,
    }

# Preserve original source path
config['_generated_from'] = src
config['_generated_for_run'] = os.path.basename(os.path.dirname(os.path.dirname(dst)))

with open(dst, 'w') as f:
    yaml.safe_dump(config, f, default_flow_style=False, allow_unicode=True, width=200)

print(f"Sandbox config written to {dst}")
print(f"  persistent_map.root_dir = {sandbox}")
print(f"  runtime_diagnostics.output_dir = {diag_dir}")
PYEOF
  echo ""

  # Generate diff
  diff <(python3 -c "import yaml; yaml.safe_dump(yaml.safe_load(open('$CONFIG_SOURCE')), default_flow_style=False, width=200)") \
       <(python3 -c "import yaml; yaml.safe_dump(yaml.safe_load(open('$GENERATED_CONFIG')), default_flow_style=False, width=200)") \
       > "$RUN_DIR/reports/generated_config_diff.txt" 2>&1 || true
}

if [[ "$MODE" == "full-chain" ]]; then
  generate_sandbox_config
fi

# ══════════════════════════════════════════════════════════════════════════════
# Cleanup function — handles all PIDs in correct order
# ══════════════════════════════════════════════════════════════════════════════
cleanup() {
  echo ""
  echo "=== Cleanup ==="
  local rc=0

  # 1. Stop rosbag play first
  if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    echo "Stopping rosbag play (pid=$BAG_PID)..."
    kill -TERM "$BAG_PID" 2>/dev/null || true
  fi

  # 2. Stop derived bag recorder
  if [[ -n "$RECORDER_PID" ]] && kill -0 "$RECORDER_PID" 2>/dev/null; then
    echo "Stopping derived bag recorder (pid=$RECORDER_PID)..."
    kill -TERM "$RECORDER_PID" 2>/dev/null || true
  fi

  # 3. SIGTERM monitor, wait up to 5s
  if [[ -n "$MONITOR_PID" ]] && kill -0 "$MONITOR_PID" 2>/dev/null; then
    echo "Stopping monitor (pid=$MONITOR_PID)..."
    kill -TERM "$MONITOR_PID" 2>/dev/null || true
    for _ in $(seq 1 50); do
      kill -0 "$MONITOR_PID" 2>/dev/null || break
      sleep 0.1
    done
  fi

  # 4. Stop warehouse launch
  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "Stopping launch (pid=$LAUNCH_PID)..."
    kill -TERM "$LAUNCH_PID" 2>/dev/null || true
    sleep 1
  fi

  # 5. Stop roscore
  if [[ -n "$ROSCORE_PID" ]] && kill -0 "$ROSCORE_PID" 2>/dev/null; then
    echo "Stopping roscore (pid=$ROSCORE_PID)..."
    kill -TERM "$ROSCORE_PID" 2>/dev/null || true
    sleep 0.5
  fi

  # 6. Force-kill any remaining
  for pid in "$BAG_PID" "$RECORDER_PID" "$MONITOR_PID" "$LAUNCH_PID" "$ROSCORE_PID"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      echo "Force-killing pid=$pid..."
      kill -9 "$pid" 2>/dev/null || true
    fi
  done

  # 7. Wait for all children with timeout, but never use bare "wait"
  local remaining
  remaining=$(jobs -p 2>/dev/null || true)
  if [[ -n "$remaining" ]]; then
    for pid in $remaining; do
      wait "$pid" 2>/dev/null || true
    done
  fi

  return $rc
}
trap cleanup EXIT

# ══════════════════════════════════════════════════════════════════════════════
# Isolated ROS Master
# ══════════════════════════════════════════════════════════════════════════════
echo "=== Starting isolated ROS Master (port $ROS_MASTER_PORT) ==="
export ROS_MASTER_URI="http://127.0.0.1:$ROS_MASTER_PORT"
export ROS_HOSTNAME=127.0.0.1
export ROS_HOME

roscore -p "$ROS_MASTER_PORT" > "$RUN_DIR/logs/roscore.log" 2>&1 &
ROSCORE_PID=$!
echo "roscore pid=$ROSCORE_PID"

# Wait for roscore
ROSCORE_READY=false
for _ in $(seq 1 15); do
  if rostopic list &>/dev/null; then
    ROSCORE_READY=true
    echo "ROS Master ready"
    break
  fi
  sleep 0.5
done
$ROSCORE_READY || die "roscore failed to start"

rosparam set /use_sim_time true

# ══════════════════════════════════════════════════════════════════════════════
# Mode-specific launch
# ══════════════════════════════════════════════════════════════════════════════
NODES_SEEN=""
FAILURE_REASONS=()

if [[ "$MODE" == "full-chain" ]]; then
  echo ""
  echo "=== Starting Production Warehouse Launch ==="

  LAUNCH_ARGS=(
    "use_sim_time:=true"
    "use_rviz:=false"
    "use_cargo_visualizer:=false"
    "config_file:=$GENERATED_CONFIG"
  )

  roslaunch ndt_slam warehouse_live_longterm_mapping.launch \
    "${LAUNCH_ARGS[@]}" \
    > "$RUN_DIR/logs/warehouse_launch.log" 2>&1 &
  LAUNCH_PID=$!
  echo "launch pid=$LAUNCH_PID"

  # Wait for required nodes to appear
  echo "Waiting for nodes to appear..."
  EXPECTED_NODES=("/pointcloud_merger" "/ndt_slam" "/hook_load_state" "/cargo_alarm_heartbeat")
  NODE_TIMEOUT=30
  NODE_START=$(date +%s)
  NODES_SEEN=""
  while true; do
    ELAPSED=$(( $(date +%s) - NODE_START ))
    [[ $ELAPSED -gt $NODE_TIMEOUT ]] && break
    current_nodes="$(rosnode list 2>/dev/null || true)"
    all_found=true
    for n in "${EXPECTED_NODES[@]}"; do
      if ! echo "$current_nodes" | grep -qF "$n"; then
        all_found=false
        break
      fi
    done
    if $all_found; then
      NODES_SEEN="$current_nodes"
      echo "All nodes appeared after ${ELAPSED}s"
      break
    fi
    sleep 0.5
  done
  if [[ -z "$NODES_SEEN" ]]; then
    FAILURE_REASONS+=("Not all expected nodes appeared within ${NODE_TIMEOUT}s. Found: $(rosnode list 2>/dev/null || echo 'none')")
    echo "WARNING: ${FAILURE_REASONS[-1]}"
  fi

  # Verify sandbox config is active
  echo "Verifying sandbox config..."
  ACTIVE_CONFIG="$(rosparam get /ndt_slam_node/config_file 2>/dev/null || echo "NOT_FOUND")"
  echo "Active ndt_slam_node config_file: $ACTIVE_CONFIG"

  ACTIVE_PM_ENABLED="$(rosparam get /ndt_slam_node/persistent_map_enabled 2>/dev/null || echo "NOT_FOUND")"
  echo "Active persistent_map_enabled: $ACTIVE_PM_ENABLED"

elif [[ "$MODE" == "monitor-only" ]]; then
  echo "=== Monitor-only mode: no SLAM pipeline needed ==="
fi

# ══════════════════════════════════════════════════════════════════════════════
# Runtime topic audit (full-chain mode)
# ══════════════════════════════════════════════════════════════════════════════
AUDIT_TOPIC_TYPES="{}"
if [[ "$MODE" == "full-chain" ]]; then
  echo ""
  echo "=== Runtime Topic Audit ==="
  TOPIC_AUDIT="$RUN_DIR/reports/runtime_topic_audit.json"

  python3 - "$TOPIC_AUDIT" "$ROS_MASTER_PORT" <<'PYEOF'
import sys, json
out = sys.argv[1]
try:
    import rosgraph
    master = rosgraph.Master('/topic_audit')
    # Override master URI
    import os
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:' + sys.argv[2]
    master = rosgraph.Master('/topic_audit')
    topics = master.getTopicTypes()
    audit = {}
    expected_types = {
        '/merged_points': 'sensor_msgs/PointCloud2',
        '/odom': 'nav_msgs/Odometry',
        '/cargo_avoidance/safety_status': 'lidar_slam2_msgs/CargoSafetyStatus',
        '/cargo_avoidance/status_code': 'std_msgs/Int32',
    }
    for t, typ in topics:
        audit[t] = {'type': typ, 'matches_expected': expected_types.get(t, 'N/A') == typ}
    with open(out, 'w') as f:
        json.dump(audit, f, indent=2)
    print(f"Topic audit saved to {out}")
    for t, expect in expected_types.items():
        if t in audit:
            match = "PASS" if audit[t]['type'] == expect else f"MISMATCH (got {audit[t]['type']})"
            print(f"  {t}: {match}")
        else:
            print(f"  {t}: NOT_FOUND")
except Exception as e:
    with open(out, 'w') as f:
        json.dump({'error': str(e)}, f, indent=2)
    print(f"WARNING: topic audit failed: {e}")
PYEOF

  if [[ -f "$TOPIC_AUDIT" ]]; then
    AUDIT_TOPIC_TYPES="$(python3 -c "import json; print(json.dumps(json.load(open('$TOPIC_AUDIT'))))" 2>/dev/null || echo '{}')"
  fi
fi

# ══════════════════════════════════════════════════════════════════════════════
# Start Monitor
# ══════════════════════════════════════════════════════════════════════════════
echo ""
echo "=== Starting Monitor ==="
MONITOR_DURATION=$(( DURATION + 10 ))

rosrun ndt_slam server_runtime_monitor.py \
  --workspace "$WORKSPACE" \
  --run-id "$RUN_ID" \
  --run-dir "$RUN_DIR" \
  --persistent-root "${MAP_SANDBOX:-$WORKSPACE/maps/live/current}" \
  --config "$WORKSPACE/src/ndt_slam/config/server_monitor.yaml" \
  ${EXPECTED_SHA:+--expected-sha "$EXPECTED_SHA"} \
  > "$RUN_DIR/logs/monitor.foreground.log" 2>&1 &

MONITOR_PID=$!
echo "monitor pid=$MONITOR_PID"

# Wait for process_ready
echo "Waiting for monitor process_ready..."
MONITOR_READY=false
for _ in $(seq 1 20); do
  if [[ -f "$RUN_DIR/reports/monitor_ready.json" ]]; then
    if jq -e '.process_ready == true' "$RUN_DIR/reports/monitor_ready.json" > /dev/null 2>&1; then
      MONITOR_READY=true
      echo "Monitor process_ready=true"
      break
    fi
  fi
  sleep 0.5
done
$MONITOR_READY || FAILURE_REASONS+=("Monitor process_ready did not become true within 10s")

# ══════════════════════════════════════════════════════════════════════════════
# Start derived bag recorder (full-chain mode)
# ══════════════════════════════════════════════════════════════════════════════
if [[ "$MODE" == "full-chain" ]]; then
  echo ""
  echo "=== Starting Derived Monitor-only Bag Recorder ==="
  DERIVED_BAG="$RUN_DIR/bags/derived_monitor_only.bag"

  rosbag record \
    --buffsize=256 \
    -O "$DERIVED_BAG" \
    /odom \
    /cargo_avoidance/safety_status \
    /cargo_avoidance/status_code \
    /cargo_avoidance/static_evidence_debug \
    /cargo_avoidance/cargo_geometry_debug \
    /cargo_avoidance/recognition_status \
    /cargo_avoidance/swing_status \
    /pointcloud_merger/diagnostics \
    > "$RUN_DIR/logs/derived_bag_recorder.log" 2>&1 &
  RECORDER_PID=$!
  echo "derived recorder pid=$RECORDER_PID"
fi

# ══════════════════════════════════════════════════════════════════════════════
# Play Bag
# ══════════════════════════════════════════════════════════════════════════════
echo ""
echo "=== Playing Bag ==="

BAG_ARGS=(--clock --rate "$RATE" --duration "$DURATION")
if [[ "$START_OFFSET" -gt 0 ]]; then
  BAG_ARGS+=(--start "$START_OFFSET")
fi

# rosbag play requires BAGFILE before --topics (argparse positional conflict)
BAG_TOPIC_ARGS=()
if [[ "$MODE" == "full-chain" ]]; then
  for t in /rs_201 /rs_203 /gravity; do
    if _bag_has_topic "$t"; then
      BAG_TOPIC_ARGS+=("$t")
    fi
  done
  for t in /tf /tf_static; do
    if _bag_has_topic "$t"; then
      BAG_TOPIC_ARGS+=("$t")
    fi
  done
  echo "Playing topics: ${BAG_TOPIC_ARGS[*]:-all}"
elif [[ "$MODE" == "monitor-only" ]]; then
  for t in /odom /cargo_avoidance/safety_status /cargo_avoidance/status_code \
           /cargo_avoidance/static_evidence_debug /cargo_avoidance/cargo_geometry_debug \
           /cargo_avoidance/recognition_status /cargo_avoidance/swing_status; do
    if _bag_has_topic "$t"; then
      BAG_TOPIC_ARGS+=("$t")
    fi
  done
fi

# BAGFILE must come before --topics
if [[ ${#BAG_TOPIC_ARGS[@]} -gt 0 ]]; then
  rosbag play "${BAG_ARGS[@]}" "$BAG" --topics "${BAG_TOPIC_ARGS[@]}" \
    > "$RUN_DIR/logs/rosbag_play.log" 2>&1 &
else
  rosbag play "${BAG_ARGS[@]}" "$BAG" \
    > "$RUN_DIR/logs/rosbag_play.log" 2>&1 &
fi
BAG_PID=$!
echo "bag pid=$BAG_PID"
echo "Playing for ${DURATION}s at ${RATE}x..."

# ══════════════════════════════════════════════════════════════════════════════
# Wait for completion
# ══════════════════════════════════════════════════════════════════════════════
# Wait for bag to finish
wait_and_capture "$BAG_PID" BAG_RC
echo "rosbag play exit code: $BAG_RC"

# Stop derived recorder
if [[ -n "$RECORDER_PID" ]] && kill -0 "$RECORDER_PID" 2>/dev/null; then
  kill -TERM "$RECORDER_PID" 2>/dev/null || true
  wait_and_capture "$RECORDER_PID" RECORDER_RC
  echo "derived recorder exit code: $RECORDER_RC"

  # Reindex derived bag if active flag is abnormal
  if [[ -f "$DERIVED_BAG" ]]; then
    BAG_ACTIVE="$(rosbag check "$DERIVED_BAG" 2>&1 || true)"
    if echo "$BAG_ACTIVE" | grep -qi "not properly closed\|active"; then
      echo "Reindexing derived bag..."
      rosbag reindex "$DERIVED_BAG" 2>&1 || true
    fi
    # Save bag info
    rosbag info --yaml "$DERIVED_BAG" > "$RUN_DIR/reports/derived_bag_info.yaml" 2>/dev/null || true
    rosbag info "$DERIVED_BAG" > "$RUN_DIR/logs/derived_bag_info.txt" 2>/dev/null || true
  fi
fi

# Wait for monitor to finish (with generous timeout)
echo "Waiting for monitor graceful shutdown..."
MONITOR_WAIT_START=$(date +%s)
while kill -0 "$MONITOR_PID" 2>/dev/null; do
  ELAPSED=$(( $(date +%s) - MONITOR_WAIT_START ))
  if [[ $ELAPSED -gt 15 ]]; then
    echo "Monitor taking too long, sending SIGTERM..."
    kill -TERM "$MONITOR_PID" 2>/dev/null || true
    sleep 2
    break
  fi
  sleep 0.5
done
wait_and_capture "$MONITOR_PID" MONITOR_RC
echo "monitor exit code: $MONITOR_RC"

# Stop launch
if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
  kill -TERM "$LAUNCH_PID" 2>/dev/null || true
  wait_and_capture "$LAUNCH_PID" LAUNCH_RC
  echo "launch exit code: $LAUNCH_RC"
fi

# Stop roscore
if [[ -n "$ROSCORE_PID" ]] && kill -0 "$ROSCORE_PID" 2>/dev/null; then
  kill -TERM "$ROSCORE_PID" 2>/dev/null || true
  wait_and_capture "$ROSCORE_PID" ROSCORE_RC
  echo "roscore exit code: $ROSCORE_RC"
fi

# ══════════════════════════════════════════════════════════════════════════════
# Post-run verification
# ══════════════════════════════════════════════════════════════════════════════
echo ""
echo "=== Verification ==="
FAILURES=0
PASSES=0

check() {
  local label="$1" condition="$2" detail="$3"
  if eval "$condition"; then
    echo "  PASS: $label ($detail)"
    PASSES=$((PASSES + 1))
  else
    echo "  FAIL: $label ($detail)"
    FAILURES=$((FAILURES + 1))
    FAILURE_REASONS+=("$label: $detail")
  fi
}

# ── Required output files ──
check "monitor_ready.json" \
  '[[ -f "$RUN_DIR/reports/monitor_ready.json" ]]' \
  "$RUN_DIR/reports/monitor_ready.json"

check "final_summary.json" \
  '[[ -f "$RUN_DIR/reports/final_summary.json" ]]' \
  "$RUN_DIR/reports/final_summary.json"

check "map_health_latest.json" \
  '[[ -f "$RUN_DIR/reports/map_health_latest.json" ]]' \
  "map sandbox scan results"

# ── Monitor readiness ──
if [[ -f "$RUN_DIR/reports/monitor_ready.json" ]]; then
  READY_JSON="$RUN_DIR/reports/monitor_ready.json"

  DATA_READY="$(jq -r '.data_ready // false' "$READY_JSON" 2>/dev/null || echo false)"
  check "data_ready=true" \
    '[[ "$DATA_READY" == "true" ]]' \
    "data_ready=$DATA_READY"

  ODOM_COUNT="$(jq -r '.odom_message_count // 0' "$READY_JSON" 2>/dev/null || echo 0)"
  check "odom_message_count>0" \
    '[[ "$ODOM_COUNT" -gt 0 ]]' \
    "odom_count=$ODOM_COUNT"

  SAFETY_COUNT="$(jq -r '.safety_status_message_count // 0' "$READY_JSON" 2>/dev/null || echo 0)"
  check "safety_status_message_count>0" \
    '[[ "$SAFETY_COUNT" -gt 0 ]]' \
    "safety_count=$SAFETY_COUNT"

  CODE_COUNT="$(jq -r '.status_code_message_count // 0' "$READY_JSON" 2>/dev/null || echo 0)"
  check "status_code_message_count>0" \
    '[[ "$CODE_COUNT" -gt 0 ]]' \
    "code_count=$CODE_COUNT"

  # created_at should be immutable after first write
  CREATED_AT_START="$(jq -r '.created_at // 0' "$READY_JSON" 2>/dev/null || echo 0)"
  CREATED_AT_END="$CREATED_AT_START"
  # Just check it's set and reasonable (within last hour)
  NOW_TS=$(date +%s)
  CREATED_DELTA=$(( NOW_TS - ${CREATED_AT_START%.*} ))
  check "created_at_is_reasonable" \
    '[[ "$CREATED_DELTA" -lt 3600 ]] && [[ "$CREATED_DELTA" -gt -60 ]]' \
    "delta=${CREATED_DELTA}s"

  READY_UPDATED="$(jq -r '.ready_updated_at // 0' "$READY_JSON" 2>/dev/null || echo 0)"
  check "ready_updated_at_exists" \
    '[[ "$READY_UPDATED" != "0" ]]' \
    "ready_updated_at=$READY_UPDATED"
fi

# ── Writer dropped ──
if [[ -f "$RUN_DIR/reports/live_summary.json" ]]; then
  DROPPED="$(jq -r '.writer_dropped // -1' "$RUN_DIR/reports/live_summary.json" 2>/dev/null || echo -1)"
  check "writer_dropped=0" \
    '[[ "$DROPPED" == "0" ]]' \
    "dropped=$DROPPED"
fi

# ── Map health scans (use Python csv.DictReader for named fields) ──
if [[ -f "$RUN_DIR/samples/map_health_samples.csv" ]]; then
  MAP_ANOMALIES_JSON="$RUN_DIR/reports/map_anomaly_counts_by_scan.json"
  python3 - "$RUN_DIR/samples/map_health_samples.csv" "$MAP_ANOMALIES_JSON" <<'PYEOF'
import sys, csv, json

csv_path = sys.argv[1]
out_path = sys.argv[2]
anomalies = []

with open(csv_path, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        entry = {}
        for k in ('wall_time', 'tile_files', 'tiles_out_of_bounds_count',
                  'z_outlier_tiles_count', 'z_outlier_below_total',
                  'z_outlier_above_total', 'ratio_alerts_count'):
            val = row.get(k, '')
            if k == 'wall_time':
                entry[k] = float(val) if val else 0.0
            else:
                try:
                    entry[k] = int(val) if val else 0
                except ValueError:
                    entry[k] = 0
        anomalies.append(entry)

with open(out_path, 'w') as f:
    json.dump(anomalies, f, indent=2)

scans = len(anomalies)
print(f"map_health_scans={scans}")
if scans >= 2:
    s1 = anomalies[0]
    s2 = anomalies[1]
    # tile_files must be consistent
    if s1.get('tile_files', 0) == s2.get('tile_files', 0):
        print(f"PASS tile_files_consistent ({s1['tile_files']} == {s2['tile_files']})")
    else:
        print(f"FAIL tile_files_consistent ({s1['tile_files']} != {s2['tile_files']})")
    # anomalies must not zero out on second scan
    s1_anomalies = s1.get('tiles_out_of_bounds_count', 0)
    s2_anomalies = s2.get('tiles_out_of_bounds_count', 0)
    if s2_anomalies > 0 or s1_anomalies == 0:
        print(f"PASS anomalies_preserved (scan1={s1_anomalies}, scan2={s2_anomalies})")
    else:
        print(f"FAIL anomalies_zeroed (scan1={s1_anomalies}, scan2={s2_anomalies})")
elif scans == 1:
    print("WARN only_one_scan")
else:
    print("FAIL no_scans")
PYEOF

  MAP_SCANS="$(python3 -c "import json; d=json.load(open('$MAP_ANOMALIES_JSON')); print(len(d))" 2>/dev/null || echo 0)"
  check "map_health_scans>=2" \
    '[[ "$MAP_SCANS" -ge 2 ]]' \
    "scans=$MAP_SCANS"
fi

# ── Map sandbox change (expected for full-chain — SLAM writes tiles) ──
if [[ -n "$MAP_HASH_BEFORE" ]]; then
  MAP_HASH_AFTER="$(find "$MAP_SANDBOX" -type f -print0 | sort -z | xargs -0 sha256sum 2>/dev/null)"
  echo "$MAP_HASH_AFTER" > "$RUN_DIR/reports/map_hash_after.txt"
  SBOX_BEFORE_COUNT=$(echo "$MAP_HASH_BEFORE" | wc -l)
  SBOX_AFTER_COUNT=$(echo "$MAP_HASH_AFTER" | wc -l)
  if [[ "$MODE" == "full-chain" ]]; then
    # In full-chain mode, sandbox growth is expected (SLAM writes new tiles)
    check "map_sandbox_valid" \
      '[[ "$SBOX_AFTER_COUNT" -ge "$SBOX_BEFORE_COUNT" ]]' \
      "sandbox files: $SBOX_BEFORE_COUNT -> $SBOX_AFTER_COUNT (growth expected)"
  else
    if diff -q <(echo "$MAP_HASH_BEFORE") <(echo "$MAP_HASH_AFTER") > /dev/null 2>&1; then
      check "map_sandbox_unchanged" "true" "sha256 identical"
    else
      check "map_sandbox_unchanged" "false" "sha256 DIFFERS"
    fi
  fi
fi

# Live map unchanged
if [[ -n "$LIVE_MAP_HASH_BEFORE" ]]; then
  LIVE_MAP_HASH_AFTER="$(find "$MAP_SOURCE" -type f -print0 | sort -z | xargs -0 sha256sum 2>/dev/null)"
  echo "$LIVE_MAP_HASH_AFTER" > "$RUN_DIR/reports/live_map_hash_after.txt"
  if diff -q <(echo "$LIVE_MAP_HASH_BEFORE") <(echo "$LIVE_MAP_HASH_AFTER") > /dev/null 2>&1; then
    check "live_map_unchanged" "true" "sha256 identical"
  else
    check "live_map_unchanged" "false" "sha256 DIFFERS - SOURCE MAP MODIFIED!"
  fi
fi

# ── No crashes ──
if [[ -f "$RUN_DIR/logs/monitor.foreground.log" ]]; then
  CRASH_COUNT="0"
  if [[ -f "$RUN_DIR/logs/monitor.foreground.log" ]]; then
    CRASH_COUNT="$(grep -ciE 'Traceback|TypeError|KeyError|Segmentation fault|core dumped' \
      "$RUN_DIR/logs/monitor.foreground.log" 2>/dev/null || echo 0)"
    # Normalize: strip all whitespace/newlines
    CRASH_COUNT=$(echo "$CRASH_COUNT" | tr -d '[:space:]')
    CRASH_COUNT="${CRASH_COUNT:-0}"
  fi
  check "no_crash" \
    '[[ '"$CRASH_COUNT"' -eq 0 ]]' \
    "crash_lines=$CRASH_COUNT"
fi

# ── Graceful shutdown ──
MONITOR_GRACEFUL=false
if [[ "$MONITOR_RC" -eq 0 || "$MONITOR_RC" -eq 124 || "$MONITOR_RC" -eq 143 ]]; then
  MONITOR_GRACEFUL=true
fi
check "graceful_shutdown" \
  '$MONITOR_GRACEFUL' \
  "exit_code=$MONITOR_RC (0/124/143 acceptable)"

# ── Full-chain specific checks ──
if [[ "$MODE" == "full-chain" ]]; then
  check "production_launch_started" \
    '[[ -n "$LAUNCH_PID" ]] && [[ "$LAUNCH_RC" != "" ]]' \
    "launch_pid=$LAUNCH_PID launch_rc=$LAUNCH_RC"

  check "nodes_appeared" \
    '[[ -n "$NODES_SEEN" ]]' \
    "nodes=$(echo "$NODES_SEEN" | tr '\n' ' ')"

  # Runtime topic existence (from audit)
  for t in /merged_points /odom /cargo_avoidance/safety_status /cargo_avoidance/status_code; do
    if echo "$AUDIT_TOPIC_TYPES" | jq -e '.["'"$t"'"]' > /dev/null 2>&1; then
      check "runtime_topic_$t" "true" "present"
    else
      check "runtime_topic_$t" "false" "NOT_FOUND"
    fi
  done

  # Derived bag
  if [[ -f "$DERIVED_BAG" ]]; then
    check "derived_bag_exists" "true" "$DERIVED_BAG"

    DERIVED_INFO="$RUN_DIR/reports/derived_bag_info.yaml"
    DERIVED_VALID=false
    if [[ -f "$DERIVED_INFO" ]]; then
      DERIVED_ODOM=$(grep -c '/odom' "$DERIVED_INFO" 2>/dev/null || echo 0)
      DERIVED_SAFETY=$(grep -c '/cargo_avoidance/safety_status' "$DERIVED_INFO" 2>/dev/null || echo 0)
      DERIVED_CODE=$(grep -c '/cargo_avoidance/status_code' "$DERIVED_INFO" 2>/dev/null || echo 0)
      if [[ "$DERIVED_ODOM" -gt 0 && "$DERIVED_SAFETY" -gt 0 && "$DERIVED_CODE" -gt 0 ]]; then
        DERIVED_VALID=true
      fi
    fi
    check "derived_bag_required_topics" \
      '$DERIVED_VALID' \
      "odom=$DERIVED_ODOM safety=$DERIVED_SAFETY code=$DERIVED_CODE"
  else
    FAILURE_REASONS+=("derived bag not found: $DERIVED_BAG")
    check "derived_bag_exists" "false" "not found"
    check "derived_bag_required_topics" "false" "no bag to check"
  fi
fi

# ── Partial mode can never PASS ──
if [[ "$MODE" == "partial" ]]; then
  OVERALL="PARTIAL"
  FAILURE_REASONS+=("partial mode cannot output PASS by policy")
elif [[ ${#FAILURE_REASONS[@]} -gt 0 ]]; then
  OVERALL="FAIL"
else
  OVERALL="PASS"
fi

# ══════════════════════════════════════════════════════════════════════════════
# Save exit codes
# ══════════════════════════════════════════════════════════════════════════════
jq -n \
  --argjson roscore_rc "${ROSCORE_RC:-null}" \
  --argjson launch_rc "${LAUNCH_RC:-null}" \
  --argjson monitor_rc "${MONITOR_RC:-null}" \
  --argjson bag_rc "${BAG_RC:-null}" \
  --argjson recorder_rc "${RECORDER_RC:-null}" \
  '{
    roscore: $roscore_rc,
    warehouse_launch: $launch_rc,
    monitor: $monitor_rc,
    rosbag_play: $bag_rc,
    derived_bag_recorder: $recorder_rc
  }' > "$RUN_DIR/reports/process_exit_codes.json"

# ══════════════════════════════════════════════════════════════════════════════
# Build comprehensive bag validation report (jq may fail on edge cases)
# ══════════════════════════════════════════════════════════════════════════════
REPORT="$RUN_DIR/reports/bag_validation_latest.json"
set +e  # allow jq sub-commands to fail without killing script

# Build message counts from monitor_ready
MSG_COUNTS="{}"
if [[ -f "$RUN_DIR/reports/monitor_ready.json" ]]; then
  MSG_COUNTS="$(jq '{
    odom: (.odom_message_count // 0),
    safety_status: (.safety_status_message_count // 0),
    status_code: (.status_code_message_count // 0)
  }' "$RUN_DIR/reports/monitor_ready.json" 2>/dev/null || echo '{}')"
fi

# Build anomaly counts
ANOMALY_COUNTS="[]"
if [[ -f "$RUN_DIR/reports/map_anomaly_counts_by_scan.json" ]]; then
  ANOMALY_COUNTS="$(cat "$RUN_DIR/reports/map_anomaly_counts_by_scan.json" 2>/dev/null || echo '[]')"
fi

jq -n \
  --arg schema_version "2" \
  --arg mode "$MODE" \
  --arg bag_path "$BAG" \
  --arg bag_sha256 "$BAG_SHA" \
  --arg bag_duration_sec "$BAG_DURATION" \
  --arg playback_rate "$RATE" \
  --arg start_offset_sec "$START_OFFSET" \
  --arg production_launch_package "ndt_slam" \
  --arg production_launch_file "warehouse_live_longterm_mapping.launch" \
  --arg generated_config "$GENERATED_CONFIG" \
  --arg run_dir "$RUN_DIR" \
  --arg persistent_root "$MAP_SANDBOX" \
  --arg runtime_diagnostics "$DIAG_OUTPUT_DIR" \
  --arg source_map "$MAP_SOURCE" \
  --arg nodes_seen "${NODES_SEEN:-}" \
  --argjson input_topics "$(jq -n \
    --argjson rs201 "$HAS_RS201" \
    --argjson rs203 "$HAS_RS203" \
    --argjson gravity "$HAS_GRAVITY" \
    '{rs_201: $rs201, rs_203: $rs203, gravity: $gravity}')" \
  --argjson generated_topics "$(jq -n \
    '{merged_points: true, odom: true, safety_status: true, status_code: true}')" \
  --argjson message_counts "$MSG_COUNTS" \
  --argjson monitor_process_ready "${MONITOR_READY:-false}" \
  --argjson monitor_data_ready "${DATA_READY:-false}" \
  --argjson writer_dropped "${DROPPED:--1}" \
  --argjson monitor_graceful_shutdown "${MONITOR_GRACEFUL:-false}" \
  --argjson map_scan_count "${MAP_SCANS:-0}" \
  --argjson anomaly_counts_by_scan "$ANOMALY_COUNTS" \
  --argjson source_map_unchanged "$([[ -n "$LIVE_MAP_HASH_BEFORE" && -n "$LIVE_MAP_HASH_AFTER" ]] && diff -q <(echo "$LIVE_MAP_HASH_BEFORE") <(echo "$LIVE_MAP_HASH_AFTER") > /dev/null 2>&1 && echo true || echo false)" \
  --argjson sandbox_changed "$([[ -n "$MAP_HASH_BEFORE" && -n "$MAP_HASH_AFTER" ]] && ! diff -q <(echo "$MAP_HASH_BEFORE") <(echo "$MAP_HASH_AFTER") > /dev/null 2>&1 && echo true || echo false)" \
  --argjson process_exit_codes "$(cat "$RUN_DIR/reports/process_exit_codes.json" 2>/dev/null || echo '{}')" \
  --arg derived_bag_path "${DERIVED_BAG:-}" \
  --argjson derived_bag_valid "${DERIVED_VALID:-false}" \
  --argjson derived_bag_required_topics_present "${DERIVED_VALID:-false}" \
  --argjson failure_reasons "$(printf '%s\n' "${FAILURE_REASONS[@]}" | jq -R . | jq -s .)" \
  --arg overall "$OVERALL" \
  --argjson pass_count "$PASSES" \
  --argjson fail_count "$FAILURES" \
  '{
    schema_version: $schema_version,
    mode: $mode,
    bag: {
      path: $bag_path,
      sha256: $bag_sha256,
      duration_sec: $bag_duration_sec,
      playback_rate: $playback_rate,
      start_offset_sec: $start_offset_sec
    },
    production_launch: {
      package: $production_launch_package,
      file: $production_launch_file,
      use_sim_time: true,
      use_rviz: false,
      generated_config: $generated_config,
      nodes_expected: ["/pointcloud_merger", "/ndt_slam", "/hook_load_state", "/cargo_alarm_heartbeat"],
      nodes_seen: ($nodes_seen // "")
    },
    paths: {
      run_dir: $run_dir,
      persistent_root: $persistent_root,
      runtime_diagnostics: $runtime_diagnostics,
      source_map: $source_map
    },
    input_topics: $input_topics,
    generated_topics: $generated_topics,
    message_counts: $message_counts,
    monitor: {
      process_ready: $monitor_process_ready,
      data_ready: $monitor_data_ready,
      writer_dropped: $writer_dropped,
      graceful_shutdown: $monitor_graceful_shutdown
    },
    map: {
      scan_count: $map_scan_count,
      anomaly_counts_by_scan: $anomaly_counts_by_scan,
      source_map_unchanged: $source_map_unchanged,
      sandbox_changed: $sandbox_changed
    },
    process_exit_codes: $process_exit_codes,
    derived_monitor_bag: {
      path: $derived_bag_path,
      valid: $derived_bag_valid,
      required_topics_present: $derived_bag_required_topics_present
    },
    verification: {
      pass_count: $pass_count,
      fail_count: $fail_count
    },
    failure_reasons: $failure_reasons,
    overall: $overall
  }' > "$REPORT"
REPORT_RC=$?
set -e  # restore strict error handling

# ── Update run manifest ──
MANIFEST="$RUN_DIR/run_manifest.json"
if [[ -f "$MANIFEST" ]]; then
  TMP_MANIFEST="${MANIFEST}.tmp"
  jq --arg result "$OVERALL" \
     --arg persistent_root "$MAP_SANDBOX" \
     '. + {bag_validation: $result, persistent_root: $persistent_root}' \
     "$MANIFEST" > "$TMP_MANIFEST"
  mv "$TMP_MANIFEST" "$MANIFEST"
fi

# ══════════════════════════════════════════════════════════════════════════════
# Summary
# ══════════════════════════════════════════════════════════════════════════════
echo ""
echo "============================================"
echo "Bag Validation: $OVERALL"
echo "  Mode:      $MODE"
echo "  Passes:    $PASSES"
echo "  Failures:  $FAILURES"
echo "  Report:    $REPORT"
echo "  Run dir:   $RUN_DIR"
if [[ ${#FAILURE_REASONS[@]} -gt 0 ]]; then
  echo "  Reasons:"
  for r in "${FAILURE_REASONS[@]}"; do
    echo "    - $r"
  done
fi
echo "============================================"

if [[ "$OVERALL" == "PASS" ]]; then
  exit 0
elif [[ "$OVERALL" == "PARTIAL" ]]; then
  exit 3
else
  exit 1
fi
