#!/usr/bin/env bash
# ─── Server Monitor Bag Validation Harness ───────────────────────────────────
# Runs an isolated rosbag-based validation of the server monitor.
#
# Modes:
#   monitor-only  – Bag already contains /odom, /safety_status, /status_code
#   full-chain    – Bag contains raw PointCloud2; needs full SLAM pipeline
#   partial       – Bag has some but not all required topics
#
# Usage:
#   server_monitor_bag_validate.sh \
#     --bag /path/to/bag.bag \
#     --mode monitor-only|full-chain|partial \
#     --workspace /path/to/NDT-slam-ws \
#     --map-source /path/to/maps/live/current \
#     --duration 25 \
#     --rate 1.0
# ────────────────────────────────────────────────────────────────────────────────
set -euo pipefail

# ── defaults ──
MODE="monitor-only"
WORKSPACE="${NDT_SLAM_WORKSPACE:-$HOME/NDT-slam-ws}"
DURATION=25
RATE=1.0
MAP_SOURCE=""
EXPECTED_SHA=""
ROS_MASTER_PORT=11321
START_OFFSET=0
BAG=""
RUN_ID="bag_validate_$(date +%Y%m%d_%H%M%S)"

usage() {
  cat <<'EOF'
server_monitor_bag_validate.sh – isolated rosbag validation for server monitor

  --bag FILE             Path to rosbag (required)
  --mode MODE            monitor-only | full-chain | partial (default: monitor-only)
  --workspace PATH       NDT-slam-ws root (default: $NDT_SLAM_WORKSPACE)
  --map-source PATH      Source maps directory for sandbox copy
  --duration SEC         Monitor run duration before SIGTERM (default: 25)
  --rate RATE            rosbag play --rate (default: 1.0)
  --expected-sha SHA     Expected git HEAD (optional)
  --ros-master-port PORT Isolated ROS master port (default: 11321)
  --start-offset SEC     rosbag play --start offset (default: 0)
EOF
  exit 1
}

# ── parse args ──
while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) BAG="$2"; shift 2 ;;
    --mode) MODE="$2"; shift 2 ;;
    --workspace) WORKSPACE="$2"; shift 2 ;;
    --map-source) MAP_SOURCE="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --expected-sha) EXPECTED_SHA="$2"; shift 2 ;;
    --ros-master-port) ROS_MASTER_PORT="$2"; shift 2 ;;
    --start-offset) START_OFFSET="$2"; shift 2 ;;
    -h|--help) usage ;;
    *) echo "Unknown arg: $1" >&2; usage ;;
  esac
done

[[ -n "$BAG" ]] || { echo "--bag is required" >&2; usage; }
[[ -f "$BAG" ]] || { echo "Bag file not found: $BAG" >&2; exit 1; }
[[ -d "$WORKSPACE" ]] || { echo "Workspace not found: $WORKSPACE" >&2; exit 1; }

BAG="$(realpath "$BAG")"
WORKSPACE="$(realpath "$WORKSPACE")"
RUN_DIR="$WORKSPACE/server_runs/$RUN_ID"
MAP_SANDBOX="$RUN_DIR/map_sandbox/current"

# ── verify workspace state ──
cd "$WORKSPACE"
if [[ -n "$EXPECTED_SHA" ]]; then
  ACTUAL_SHA="$(git rev-parse HEAD)"
  if [[ "$ACTUAL_SHA" != "$EXPECTED_SHA" ]]; then
    echo "HEAD mismatch: expected=$EXPECTED_SHA actual=$ACTUAL_SHA" >&2
    exit 1
  fi
fi

# ── setup ──
echo "=== Bag Validation Setup ==="
echo "bag:       $BAG"
echo "mode:      $MODE"
echo "run_dir:   $RUN_DIR"
echo "workspace: $WORKSPACE"
echo "duration:  ${DURATION}s"
echo "rate:      $RATE"
echo ""

mkdir -p "$RUN_DIR/logs" "$RUN_DIR/reports" "$RUN_DIR/samples"
ROS_HOME="$RUN_DIR/ros_home"
mkdir -p "$ROS_HOME"

# ── bag audit ──
echo "=== Bag Audit ==="
BAG_INFO="$RUN_DIR/reports/bag_info.yaml"
rosbag info --yaml "$BAG" > "$BAG_INFO" 2>/dev/null || true
rosbag info "$BAG" 2>&1 | tee "$RUN_DIR/logs/bag_info.txt"

BAG_SHA="$(sha256sum "$BAG" | awk '{print $1}')"
BAG_DURATION="$(grep 'duration:' "$BAG_INFO" | awk '{print $2}')"

# Determine available topics
HAS_ODOM=false
HAS_SAFETY=false
HAS_STATUS_CODE=false
if grep -q '/odom' "$BAG_INFO" 2>/dev/null; then HAS_ODOM=true; fi
if grep -q '/cargo_avoidance/safety_status' "$BAG_INFO" 2>/dev/null; then HAS_SAFETY=true; fi
if grep -q '/cargo_avoidance/status_code' "$BAG_INFO" 2>/dev/null; then HAS_STATUS_CODE=true; fi

# ── map sandbox ──
MAP_HASH_BEFORE=""
MAP_HASH_AFTER=""
LIVE_MAP_HASH_BEFORE=""
LIVE_MAP_HASH_AFTER=""

if [[ -n "$MAP_SOURCE" && -d "$MAP_SOURCE" ]]; then
  MAP_SOURCE="$(realpath "$MAP_SOURCE")"
  echo "=== Map Sandbox ==="
  echo "source: $MAP_SOURCE"
  echo "sandbox: $MAP_SANDBOX"
  mkdir -p "$MAP_SANDBOX"
  rsync -a "$MAP_SOURCE/" "$MAP_SANDBOX/"
  echo "Map copied to sandbox"

  if [[ "$MODE" == "monitor-only" ]]; then
    chmod -R a-w "$MAP_SANDBOX"
    echo "Sandbox set to read-only"
  fi

  # Record sandbox hash before
  MAP_HASH_BEFORE="$(find "$MAP_SANDBOX" -type f -print0 | sort -z | xargs -0 sha256sum)"
  echo "$MAP_HASH_BEFORE" > "$RUN_DIR/reports/map_hash_before.txt"

  # Record live map hash (if different)
  if [[ "$MAP_SOURCE" != "$MAP_SANDBOX" ]]; then
    LIVE_MAP_HASH_BEFORE="$(find "$MAP_SOURCE" -type f -print0 | sort -z | xargs -0 sha256sum)"
    echo "$LIVE_MAP_HASH_BEFORE" > "$RUN_DIR/reports/live_map_hash_before.txt"
  fi
fi

# ── isolated ROS master ──
echo "=== Starting isolated ROS Master (port $ROS_MASTER_PORT) ==="
export ROS_MASTER_URI="http://127.0.0.1:$ROS_MASTER_PORT"
export ROS_HOSTNAME=127.0.0.1
export ROS_HOME

roscore -p "$ROS_MASTER_PORT" > "$RUN_DIR/logs/roscore.log" 2>&1 &
ROSCORE_PID=$!
echo "roscore pid=$ROSCORE_PID"

# Wait for roscore
for _ in $(seq 1 10); do
  if rostopic list &>/dev/null; then
    echo "ROS Master ready"
    break
  fi
  sleep 0.5
done

rosparam set /use_sim_time true

# ── trap cleanup ──
cleanup() {
  echo ""
  echo "=== Cleanup ==="
  for sig in TERM TERM TERM; do
    for pid in "$MONITOR_PID" "$BAG_PID" "$ROSCORE_PID"; do
      if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
        kill -$sig "$pid" 2>/dev/null || true
      fi
    done
    sleep 0.5
  done
  # force kill any stragglers
  for pid in "$MONITOR_PID" "$BAG_PID" "$ROSCORE_PID"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -9 "$pid" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT

# ── start monitor ──
echo ""
echo "=== Starting Monitor ==="
MONITOR_DURATION=$(( DURATION + 5 ))

timeout --signal=TERM --kill-after=5s "${MONITOR_DURATION}s" \
  rosrun ndt_slam server_runtime_monitor.py \
  --workspace "$WORKSPACE" \
  --run-id "$RUN_ID" \
  --run-dir "$RUN_DIR" \
  --persistent-root "${MAP_SANDBOX:-$WORKSPACE/maps/live/current}" \
  --config "$WORKSPACE/src/ndt_slam/config/server_monitor.yaml" \
  ${EXPECTED_SHA:+--expected-sha "$EXPECTED_SHA"} \
  > "$RUN_DIR/logs/monitor.foreground.log" 2>&1 &

MONITOR_PID=$!
echo "monitor pid=$MONITOR_PID (wrapper)"
sleep 2

# ── play bag ──
echo ""
echo "=== Playing Bag ==="
BAG_TOPICS=()
if [[ "$MODE" == "monitor-only" ]]; then
  for t in /odom /cargo_avoidance/safety_status /cargo_avoidance/status_code \
           /cargo_avoidance/static_evidence_debug /cargo_avoidance/cargo_geometry_debug; do
    if grep -q "$t" "$BAG_INFO" 2>/dev/null; then
      BAG_TOPICS+=("$t")
    fi
  done
fi

BAG_ARGS=(--clock --rate "$RATE")
if [[ "$START_OFFSET" -gt 0 ]]; then
  BAG_ARGS+=(--start "$START_OFFSET")
fi
if [[ ${#BAG_TOPICS[@]} -gt 0 ]]; then
  BAG_ARGS+=(--topics "${BAG_TOPICS[@]}")
fi

rosbag play "${BAG_ARGS[@]}" "$BAG" \
  > "$RUN_DIR/logs/rosbag_play.log" 2>&1 &

BAG_PID=$!
echo "bag pid=$BAG_PID"

# ── wait for monitor to finish ──
echo ""
echo "=== Waiting for monitor (${MONITOR_DURATION}s timeout) ==="
wait "$MONITOR_PID" 2>/dev/null || true
MONITOR_RC=$?
echo "monitor exit code: $MONITOR_RC"

# ── graceful stop ──
kill -TERM "$BAG_PID" 2>/dev/null || true
wait "$BAG_PID" 2>/dev/null || true

kill -TERM "$ROSCORE_PID" 2>/dev/null || true
wait "$ROSCORE_PID" 2>/dev/null || true

# ── post-run verification ──
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
  fi
}

# Required output files
check "monitor_ready.json" \
  '[[ -f "$RUN_DIR/reports/monitor_ready.json" ]]' \
  "$RUN_DIR/reports/monitor_ready.json"

check "live_summary.json" \
  '[[ -f "$RUN_DIR/reports/live_summary.json" ]]' \
  "$RUN_DIR/reports/live_summary.json"

check "final_summary.json" \
  '[[ -f "$RUN_DIR/reports/final_summary.json" ]]' \
  "$RUN_DIR/reports/final_summary.json"

check "map_health_latest.json" \
  '[[ -f "$RUN_DIR/reports/map_health_latest.json" ]]' \
  "present if map sandbox available"

# Writer dropped
if [[ -f "$RUN_DIR/reports/live_summary.json" ]]; then
  DROPPED="$(jq -r '.writer_dropped // -1' "$RUN_DIR/reports/live_summary.json" 2>/dev/null || echo -1)"
  check "writer_dropped=0" \
    '[[ "$DROPPED" == "0" ]]' \
    "dropped=$DROPPED"
fi

# Odom samples (if odom topic was in bag)
if $HAS_ODOM && [[ -f "$RUN_DIR/samples/localization_samples.csv" ]]; then
  ODOM_LINES="$(tail -n +2 "$RUN_DIR/samples/localization_samples.csv" 2>/dev/null | wc -l)"
  check "odom_samples>0" \
    '[[ "$ODOM_LINES" -gt 0 ]]' \
    "odom_lines=$ODOM_LINES"
fi

# Safety samples
if $HAS_SAFETY && [[ -f "$RUN_DIR/samples/safety_samples.csv" ]]; then
  SAFETY_LINES="$(tail -n +2 "$RUN_DIR/samples/safety_samples.csv" 2>/dev/null | wc -l)"
  check "safety_samples>0" \
    '[[ "$SAFETY_LINES" -gt 0 ]]' \
    "safety_lines=$SAFETY_LINES"
fi

# Map health scans
if [[ -f "$RUN_DIR/samples/map_health_samples.csv" ]]; then
  MAP_SCANS="$(tail -n +2 "$RUN_DIR/samples/map_health_samples.csv" 2>/dev/null | wc -l)"
  check "map_health_scans>=2" \
    '[[ "$MAP_SCANS" -ge 2 ]]' \
    "scans=$MAP_SCANS"

  # Check second scan doesn't lose anomalies
  if [[ "$MAP_SCANS" -ge 2 ]]; then
    SCAN1_BOUNDS="$(sed -n '2p' "$RUN_DIR/samples/map_health_samples.csv" 2>/dev/null | cut -d, -f6)"
    SCAN2_BOUNDS="$(sed -n '3p' "$RUN_DIR/samples/map_health_samples.csv" 2>/dev/null | cut -d, -f6)"
    if [[ "$SCAN1_BOUNDS" == "$SCAN2_BOUNDS" ]]; then
      echo "  PASS: map_scan_anomalies_consistent (bounds=$SCAN1_BOUNDS vs $SCAN2_BOUNDS)"
      PASSES=$((PASSES + 1))
    else
      echo "  FAIL: map_scan_anomalies_consistent (bounds=$SCAN1_BOUNDS vs $SCAN2_BOUNDS)"
      FAILURES=$((FAILURES + 1))
    fi
  fi
fi

# Map hash unchanged
if [[ -n "$MAP_HASH_BEFORE" ]]; then
  MAP_HASH_AFTER="$(find "$MAP_SANDBOX" -type f -print0 | sort -z | xargs -0 sha256sum)"
  echo "$MAP_HASH_AFTER" > "$RUN_DIR/reports/map_hash_after.txt"
  if diff -q <(echo "$MAP_HASH_BEFORE") <(echo "$MAP_HASH_AFTER") > /dev/null 2>&1; then
    check "map_sandbox_unchanged" "true" "sha256 identical"
  else
    check "map_sandbox_unchanged" "false" "sha256 DIFFERS"
  fi
fi

# Live map unchanged
if [[ -n "$LIVE_MAP_HASH_BEFORE" ]]; then
  LIVE_MAP_HASH_AFTER="$(find "$MAP_SOURCE" -type f -print0 | sort -z | xargs -0 sha256sum)"
  echo "$LIVE_MAP_HASH_AFTER" > "$RUN_DIR/reports/live_map_hash_after.txt"
  if diff -q <(echo "$LIVE_MAP_HASH_BEFORE") <(echo "$LIVE_MAP_HASH_AFTER") > /dev/null 2>&1; then
    check "live_map_unchanged" "true" "sha256 identical"
  else
    check "live_map_unchanged" "false" "sha256 DIFFERS"
  fi
fi

# No crashes
if [[ -f "$RUN_DIR/logs/monitor.foreground.log" ]]; then
  CRASHES="$(grep -ciE 'Traceback|TypeError|KeyError|Segmentation fault|core dumped' \
    "$RUN_DIR/logs/monitor.foreground.log" 2>/dev/null || echo 0)"
  check "no_crash" \
    '[[ "$CRASHES" -eq 0 ]]' \
    "crash_lines=$CRASHES"
fi

# graceful shutdown
MONITOR_GRACEFUL=false
if [[ "$MONITOR_RC" -eq 0 || "$MONITOR_RC" -eq 124 || "$MONITOR_RC" -eq 143 ]]; then
  MONITOR_GRACEFUL=true
fi
check "graceful_shutdown" \
  '$MONITOR_GRACEFUL' \
  "exit_code=$MONITOR_RC (0/124/143 acceptable)"

# ── bag validation report ──
OVERALL="PASS"
[[ "$FAILURES" -gt 0 ]] && OVERALL="FAIL"

REPORT="$RUN_DIR/reports/bag_validation_latest.json"
jq -n \
  --arg bag_path "$BAG" \
  --arg bag_sha256 "$BAG_SHA" \
  --arg bag_duration_sec "$BAG_DURATION" \
  --arg playback_rate "$RATE" \
  --arg mode "$MODE" \
  --argjson has_odom "$HAS_ODOM" \
  --argjson has_safety "$HAS_SAFETY" \
  --argjson has_status_code "$HAS_STATUS_CODE" \
  --argjson monitor_exit_code "$MONITOR_RC" \
  --argjson monitor_graceful_shutdown "$MONITOR_GRACEFUL" \
  --argjson pass_count "$PASSES" \
  --argjson fail_count "$FAILURES" \
  --arg overall "$OVERALL" \
  '{
    bag_path: $bag_path,
    bag_sha256: $bag_sha256,
    bag_duration_sec: $bag_duration_sec,
    playback_rate: $playback_rate,
    mode: $mode,
    required_topics: {
      odom: $has_odom,
      safety_status: $has_safety,
      status_code: $has_status_code
    },
    monitor_exit_code: $monitor_exit_code,
    monitor_graceful_shutdown: $monitor_graceful_shutdown,
    verification: {
      pass_count: $pass_count,
      fail_count: $fail_count,
    },
    overall: $overall
  }' > "$REPORT"

# ── update run manifest ──
MANIFEST="$RUN_DIR/run_manifest.json"
if [[ -f "$MANIFEST" ]]; then
  TMP_MANIFEST="${MANIFEST}.tmp"
  jq --arg result "$OVERALL" '. + {bag_validation: $result}' "$MANIFEST" > "$TMP_MANIFEST"
  mv "$TMP_MANIFEST" "$MANIFEST"
fi

# ── save exit codes ──
jq -n \
  --argjson monitor_rc "$MONITOR_RC" \
  '{monitor_exit_code: $monitor_rc}' \
  > "$RUN_DIR/reports/process_exit_codes.json"

# ── summary ──
echo ""
echo "============================================"
echo "Bag Validation: $OVERALL"
echo "  Passes: $PASSES"
echo "  Failures: $FAILURES"
echo "  Report: $REPORT"
echo "  Run dir: $RUN_DIR"
echo "============================================"

if [[ "$OVERALL" == "PASS" ]]; then
  exit 0
else
  exit 1
fi
