#!/usr/bin/env bash
set -euo pipefail

BAG="${1:-/home/ydkj/AutoCraneSlam-ROS1/bag/调运大件.bag}"
LOG="/tmp/cargo_v11_real.log"

cd /home/ydkj/NDT-slam-ws
source devel/setup.bash

export ROSCONSOLE_CONFIG_FILE=/home/ydkj/NDT-slam-ws/src/ndt_slam/config/rosconsole_cargo_minimal.config

echo "========== v11 Real Run Test =========="
echo "Bag: ${BAG}"
echo "Log: ${LOG}"

rosnode kill -a 2>/dev/null || true
sleep 2

rm -f "$LOG"

rosparam set use_sim_time true

echo "========== Starting launch (single launch with cargo_core_box_visualizer) =========="
roslaunch ndt_slam warehouse_live_longterm_mapping.launch \
  use_sim_time:=true \
  enable_rviz:=false \
  > "$LOG" 2>&1 &
LIVE_PID=$!

sleep 8

# 检查重复节点
rosnode list | sort | uniq -d > /tmp/duplicate_nodes.txt || true
if [ -s /tmp/duplicate_nodes.txt ]; then
  echo "[VALIDATION] FAIL: duplicate ros node names" | tee -a "$LOG"
  cat /tmp/duplicate_nodes.txt | tee -a "$LOG"
  exit 1
fi

echo "========== Playing bag =========="
rosbag play --clock -r 1.0 "$BAG" >> "$LOG" 2>&1 || true

sleep 2

echo "========== Running validation =========="
/home/ydkj/NDT-slam-ws/src/ndt_slam/scripts/validate_cargo_pipeline_v11.sh "$LOG"
/home/ydkj/NDT-slam-ws/src/ndt_slam/scripts/validate_rviz_core_box_only.sh

echo "========== Cleanup =========="
kill $LIVE_PID 2>/dev/null || true

echo "Test log saved to: ${LOG}"
