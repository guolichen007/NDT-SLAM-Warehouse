#!/bin/bash
# run_588_normal_speed.sh — Reproducible 1.0× acceptance test for issue 588
# Usage:
#   ./scripts/acceptance/run_588_normal_speed.sh --mode failure-window
#   ./scripts/acceptance/run_588_normal_speed.sh --mode full

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BAG_FILE="/home/ydkj/AutoCraneSlam-ROS1/bag/调运大件.bag"
BAG_SHA256_EXPECTED="a6805f48ca0cccf231370045808c60ca1c623ac2c6bf2c7b9ec05b804d7df33c"
PLAYBACK_RATE=1.0
MODE="${1:---mode}"
MODE_VAL="${2:-full}"

if [ "$MODE" != "--mode" ]; then
  echo "Usage: $0 --mode {failure-window|full}"
  exit 1
fi

if [ "$MODE_VAL" != "failure-window" ] && [ "$MODE_VAL" != "full" ]; then
  echo "Error: mode must be 'failure-window' or 'full'"
  exit 1
fi

# Verify bag exists and SHA256
if [ ! -f "$BAG_FILE" ]; then
  echo "Error: bag file not found: $BAG_FILE"
  exit 1
fi

BAG_SHA256_ACTUAL=$(sha256sum "$BAG_FILE" | cut -d' ' -f1)
if [ "$BAG_SHA256_ACTUAL" != "$BAG_SHA256_EXPECTED" ]; then
  echo "Error: bag SHA256 mismatch"
  echo "  expected: $BAG_SHA256_EXPECTED"
  echo "  actual:   $BAG_SHA256_ACTUAL"
  exit 1
fi

# Create timestamped result directory
STAMP=$(date +%Y%m%d_%H%M%S)
RESULT_DIR="$WS_ROOT/test_results/runtime_acceptance/${STAMP}"
mkdir -p "$RESULT_DIR"

echo "=========================================="
echo "588 V4 Acceptance Test"
echo "  Mode: $MODE_VAL"
echo "  Rate: $PLAYBACK_RATE"
echo "  Result: $RESULT_DIR"
echo "=========================================="

# Record source identity
cd "$WS_ROOT"
{
  echo "git_sha=$(git rev-parse HEAD)"
  echo "git_branch=$(git branch --show-current)"
  echo "git_status=$(git status --short)"
  echo "bag_sha256=$BAG_SHA256_ACTUAL"
  echo "playback_rate=$PLAYBACK_RATE"
  echo "mode=$MODE_VAL"
  echo "test_time=$(date -Iseconds)"
} > "$RESULT_DIR/run_identity.txt"

# Clean old state
rm -f /tmp/588_ndt_profile.csv
pkill -9 -f "ndt_slam_node" 2>/dev/null || true
pkill -9 -f "pointcloud_merger" 2>/dev/null || true
pkill -9 -f "roscore" 2>/dev/null || true
pkill -9 -f "roslaunch" 2>/dev/null || true
pkill -9 -f "rosbag" 2>/dev/null || true
sleep 3

# Source environment
source /opt/ros/noetic/setup.bash
source "$WS_ROOT/devel/setup.bash"

# Start roscore
roscore &
sleep 3

# Set sim time
rosparam set /use_sim_time true

# Dump params before play
rosparam dump "$RESULT_DIR/params_before_play.yaml" 2>/dev/null || true

# Launch
roslaunch ndt_slam warehouse_live_longterm_mapping.launch \
  use_rviz:=false use_cargo_visualizer:=false \
  > "$RESULT_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!
sleep 5

# Record nodes and topics
rosnode list > "$RESULT_DIR/nodes.txt" 2>/dev/null || true
rostopic list > "$RESULT_DIR/topics.txt" 2>/dev/null || true

# Start recording
rosbag record -O "$RESULT_DIR/output_record.bag" \
  /merged_points /odom /tf /tf_static /path \
  /payload_track_info /payload_precise_box_info \
  --duration=0 &
RECORD_PID=$!
sleep 1

# Play bag
echo "Playing bag at rate=$PLAYBACK_RATE mode=$MODE_VAL..."
if [ "$MODE_VAL" = "failure-window" ]; then
  METADATA="$WS_ROOT/tests/regression/588_v4/failure_window_metadata.json"
  if [ ! -f "$METADATA" ]; then
    echo "Error: failure_window_metadata.json not found"
    exit 1
  fi
  OFFSET=$(python3 -c "import json; print(json.load(open('$METADATA'))['rosbag_start_offset_sec'])")
  DURATION=$(python3 -c "import json; print(json.load(open('$METADATA'))['duration_sec'])")
  echo "  offset=${OFFSET}s duration=${DURATION}s"
  rosbag play "$BAG_FILE" --clock -r "$PLAYBACK_RATE" \
    --start "$OFFSET" --duration "$DURATION" 2>&1
else
  rosbag play "$BAG_FILE" --clock -r "$PLAYBACK_RATE" 2>&1
fi
echo "Bag playback completed"

# Wait for processing
sleep 15

# Copy CSV files
cp /home/ydkj/ndt_slam_runtime_data/runtime_frames.csv "$RESULT_DIR/" 2>/dev/null || true
cp /home/ydkj/ndt_slam_runtime_data/cargo_frames.csv "$RESULT_DIR/" 2>/dev/null || true
cp /tmp/588_ndt_profile.csv "$RESULT_DIR/" 2>/dev/null || true

# Stop recording
kill $RECORD_PID 2>/dev/null || true
sleep 2

# Stop nodes
kill $LAUNCH_PID 2>/dev/null || true
pkill -9 -f "ndt_slam_node" 2>/dev/null || true
pkill -9 -f "pointcloud_merger" 2>/dev/null || true
pkill -9 -f "roscore" 2>/dev/null || true
pkill -9 -f "roslaunch" 2>/dev/null || true
sleep 2

echo ""
echo "Results saved to: $RESULT_DIR"
echo ""

# Run analysis
ANALYSIS_SCRIPT="$SCRIPT_DIR/analyze_588_runtime.py"
if [ -f "$ANALYSIS_SCRIPT" ]; then
  echo "Running analysis..."
  python3 "$ANALYSIS_SCRIPT" "$RESULT_DIR/runtime_frames.csv" \
    --baseline "$WS_ROOT/tests/regression/588_v4/baseline_metrics.json" \
    --output "$RESULT_DIR/analysis_result.json" 2>&1
  ANALYSIS_EXIT=$?
  echo ""
  if [ $ANALYSIS_EXIT -eq 0 ]; then
    echo "VERDICT: PASS"
  else
    echo "VERDICT: FAIL"
  fi
  exit $ANALYSIS_EXIT
else
  echo "Warning: analysis script not found, skipping automated verdict"
  exit 0
fi
