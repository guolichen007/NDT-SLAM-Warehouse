#!/usr/bin/env bash
# Paired avoidance forensic replay harness.
#
# Collects, for one bag, the full causal-chain evidence:
#   raw_safety_status / safety_status / operational_status
#   cargo_geometry_debug (JSON) + frame_causal_trace (CSV, forensic-only)
#
# Usage: run_paired_avoidance_forensic.sh <bag> <tag>
set -u
BAG="${1:?bag}"; TAG="${2:?tag}"
WS="/home/ydkj/NDT-slam-ws"
CONFIG="${WS}/maps/test_config_diskguard10.yaml"
source /opt/ros/noetic/setup.bash
source "${WS}/devel/setup.bash"

BASE="/home/ydkj/avoidance_forensics/20260820_paired_v1/instrumented"
OUT="${BASE}/${TAG}"
mkdir -p "${OUT}" /tmp/cargo_forensic
LOG="${OUT}/launch.log"
RAW="${OUT}/raw_safety_status.csv"
SAFETY="${OUT}/safety_status.csv"
OPS="${OUT}/operational_status.csv"
GEO="${OUT}/cargo_geometry_debug.txt"
TRACE="${OUT}/frame_causal_trace.csv"
MANIFEST="${OUT}/manifest.txt"

rm -f "${LOG}" "${RAW}" "${SAFETY}" "${OPS}" "${GEO}" "${TRACE}"
rm -f /tmp/cargo_forensic/frame_causal_trace.csv
rm -rf /tmp/ndt_slam_runtime_data

{
  echo "run_id=${TAG}"
  echo "bag=${BAG}"
  echo "source_sha=$(git -C ${WS} rev-parse HEAD)"
  echo "binary_sha=$(sha256sum ${WS}/devel/lib/ndt_slam/ndt_slam_node | cut -d' ' -f1)"
  echo "config=${CONFIG}"
  echo "config_sha=$(sha256sum ${CONFIG} | cut -d' ' -f1)"
  echo "bag_sha256=$(sha256sum ${BAG} | cut -d' ' -f1)"
  echo "start_time=$(date '+%Y-%m-%d %H:%M:%S')"
  echo "persistent_map=false"
  echo "use_sim_time=true"
  echo "replay_rate=1.0"
  echo "forensic_trace=/tmp/cargo_forensic/frame_causal_trace.csv"
} > "${MANIFEST}"

echo "[forensic] tag=${TAG}"
echo "[forensic] killing old nodes"
rosnode kill -a 2>/dev/null || true
pkill -f "ndt_slam_node|pointcloud_merger|hook_load_state|rosbag play|server_runtime_monitor" 2>/dev/null || true
sleep 3

echo "[forensic] launching stack (sim time, cold start)"
roslaunch ndt_slam warehouse_live_longterm_mapping.launch \
  config_file:=${CONFIG} \
  persistent_map:=false \
  use_sim_time:=true use_rviz:=false use_ndt_recovery_watchdog:=false \
  use_cargo_alarm_heartbeat:=false use_cargo_visualizer:=false \
  > "${LOG}" 2>&1 &
LAUNCH_PID=$!

echo "[forensic] waiting for topics"
for i in $(seq 1 45); do
  if rostopic list 2>/dev/null | grep -q "/cargo_avoidance/safety_status"; then
    echo "[forensic] topics ready ${i}s"; break
  fi
  sleep 1
done

echo "[forensic] starting recorders"
(rostopic echo -p /cargo_avoidance/raw_safety_status > "${RAW}" 2>/dev/null) &
P_RAW=$!
(rostopic echo -p /cargo_avoidance/safety_status > "${SAFETY}" 2>/dev/null) &
P_SAFETY=$!
(rostopic echo -p /cargo_avoidance/operational_status > "${OPS}" 2>/dev/null) &
P_OPS=$!
(rostopic echo /cargo_avoidance/cargo_geometry_debug > "${GEO}" 2>/dev/null) &
P_GEO=$!

echo "[forensic] playing bag (1.0x)"
rosbag play --clock "${BAG}" >> "${LOG}" 2>&1 &
BAG_PID=$!
START=$(date +%s)
while kill -0 "${BAG_PID}" 2>/dev/null; do
  if [ $(( $(date +%s) - START )) -gt 500 ]; then kill "${BAG_PID}" 2>/dev/null; break; fi
  sleep 5
done
sleep 6

echo "[forensic] stopping"
kill "${P_RAW}" "${P_SAFETY}" "${P_OPS}" "${P_GEO}" 2>/dev/null
kill "${LAUNCH_PID}" 2>/dev/null
pkill -f "ndt_slam_node|pointcloud_merger|rosbag play|hook_load_state" 2>/dev/null
sleep 3

if [ -f /tmp/cargo_forensic/frame_causal_trace.csv ]; then
  cp /tmp/cargo_forensic/frame_causal_trace.csv "${TRACE}"
fi
echo "end_time=$(date '+%Y-%m-%d %H:%M:%S')" >> "${MANIFEST}"

echo "[forensic] DONE ${TAG}"
echo "  raw_lines=$(wc -l < "${RAW}" 2>/dev/null || echo 0)"
echo "  safety_lines=$(wc -l < "${SAFETY}" 2>/dev/null || echo 0)"
echo "  ops_lines=$(wc -l < "${OPS}" 2>/dev/null || echo 0)"
echo "  geo_lines=$(wc -l < "${GEO}" 2>/dev/null || echo 0)"
echo "  trace_lines=$(wc -l < "${TRACE}" 2>/dev/null || echo 0)"
echo "  output_dir=${OUT}"
