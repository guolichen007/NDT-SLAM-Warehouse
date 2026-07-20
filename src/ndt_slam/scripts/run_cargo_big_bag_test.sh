#!/usr/bin/env bash
set -euo pipefail

BAG="${1:-}"
if [ -z "${BAG}" ]; then
  echo "Usage: $0 /path/to/big_cargo.bag"
  exit 1
fi

if [ ! -f "${BAG}" ]; then
  echo "Bag not found: ${BAG}"
  exit 1
fi

WS="${WS:-/home/ydkj/NDT-slam-ws}"
LOG="/tmp/cargo_pipeline_test.log"

cd "${WS}"
source devel/setup.bash

echo "========== Build =========="
catkin_make --pkg ndt_slam

echo "========== Kill old nodes =========="
rosnode kill -a 2>/dev/null || true
sleep 2

echo "========== Start launch =========="
rm -f "${LOG}"

# 主建图
roslaunch ndt_slam warehouse_live_longterm_mapping.launch >"${LOG}" 2>&1 &
LAUNCH_PID=$!

sleep 5

# 禁行区节点
roslaunch ndt_slam cargo_forbidden_zone.launch >>"${LOG}" 2>&1 &
CARGO_PID=$!

sleep 3

echo "========== Play bag =========="
rosbag play --clock "${BAG}" >>"${LOG}" 2>&1 &
BAG_PID=$!

echo "========== Monitoring =========="
echo "Log: ${LOG}"
echo "Press Ctrl+C to stop early"

# 跑 120 秒，或者 bag 自己结束
START=$(date +%s)
while kill -0 "${BAG_PID}" 2>/dev/null; do
  NOW=$(date +%s)
  ELAPSED=$((NOW - START))
  if [ "${ELAPSED}" -gt 120 ]; then
    echo "Reached 120s test window, stopping bag"
    kill "${BAG_PID}" || true
    break
  fi

  echo "----- ${ELAPSED}s summary -----"
  grep -E "\[CargoCommit\]|\[HumanFilter\]|\[MapCommitInput\]|\[MapCommit\]|\[CargoBoxV2\]|\[CleanMapDynamicGate3D\]" "${LOG}" | tail -n 20 || true
  sleep 10
done

sleep 3

echo "========== Validation =========="
"${WS}/src/ndt_slam/scripts/validate_cargo_pipeline.sh" "${LOG}"

echo "========== Cleanup =========="
kill "${LAUNCH_PID}" "${CARGO_PID}" 2>/dev/null || true

echo "Test log saved to: ${LOG}"
