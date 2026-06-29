#!/usr/bin/env bash
set -euo pipefail

LOG_FILE="${1:-/tmp/cargo_v13_real.log}"

echo "========== Cargo Pipeline Validation v13 =========="
echo "Log file: ${LOG_FILE}"

if [ ! -f "${LOG_FILE}" ]; then
  echo "[VALIDATION] FAIL: log file not found"
  exit 1
fi

fail() {
  echo "[VALIDATION] FAIL: $1"
  exit 1
}

pass_msg() {
  echo "[VALIDATION] PASS: $1"
}

warn_msg() {
  echo "[VALIDATION] WARN: $1"
}

# ========== 1. Launch 检查 ==========

if grep -q "new node registered with same name" "${LOG_FILE}"; then
  fail "duplicate node name detected"
fi

pass_msg "launch clean"

# ========== 2. use_sim_time 检查 ==========

if rosparam get /use_sim_time 2>/dev/null | grep -q "true"; then
  pass_msg "/use_sim_time=true"
else
  fail "/use_sim_time is not true"
fi

# ========== 3. lock_yaw 检查 ==========

if grep -q "lock_yaw: true" "${LOG_FILE}"; then
  pass_msg "lock_yaw=true"
else
  fail "lock_yaw is not true"
fi

# ========== 4. MapCommit 次数检查 ==========

MAP_COMMIT_COUNT=$(grep -c "\[MapCommit\]" "${LOG_FILE}" || echo "0")
if [ "${MAP_COMMIT_COUNT}" -ge 3 ]; then
  pass_msg "MapCommit count=${MAP_COMMIT_COUNT} >= 3"
else
  fail "MapCommit count=${MAP_COMMIT_COUNT} < 3, map may be stalled"
fi

# ========== 5. keyframe_count 检查 ==========

LAST_KF=$(grep "\[PipelineSummary\]" "${LOG_FILE}" | tail -1 | grep -oP 'kf=\K\d+' || echo "0")
if [ "${LAST_KF}" -ge 3 ]; then
  pass_msg "keyframe_count=${LAST_KF} >= 3"
else
  fail "keyframe_count=${LAST_KF} < 3"
fi

# ========== 6. PoseFreezeCheck raw_vel 检查 ==========

ZERO_VEL_COUNT=$(grep -c "\[PoseFreezeCheck\].*raw_vel=0.000" "${LOG_FILE}" || true)
TOTAL_CHECK=$(grep -c "\[PoseFreezeCheck\]" "${LOG_FILE}" || true)

if [ "${TOTAL_CHECK:-0}" -gt 0 ]; then
  ZERO_RATIO=$(( (${ZERO_VEL_COUNT:-0} * 100) / ${TOTAL_CHECK:-1} ))
  if [ "${ZERO_RATIO}" -gt 90 ]; then
    fail "PoseFreezeCheck raw_vel=0.000 in ${ZERO_RATIO}% of checks (${ZERO_VEL_COUNT}/${TOTAL_CHECK})"
  fi
  pass_msg "PoseFreezeCheck raw_vel not always zero (${ZERO_RATIO}%)"
fi

# ========== 7. PoseFreeze 状态转换检查 ==========

if grep -q "\[PoseFreeze\].*mode=START_RELEASE_BLEND" "${LOG_FILE}"; then
  pass_msg "PoseFreeze START_RELEASE_BLEND observed"
else
  warn_msg "no START_RELEASE_BLEND (crane may not have moved enough)"
fi

if grep -q "\[PoseFreeze\].*mode=END_RELEASE_BLEND" "${LOG_FILE}"; then
  pass_msg "PoseFreeze END_RELEASE_BLEND observed"
else
  warn_msg "no END_RELEASE_BLEND"
fi

# ========== 8. MapStall 检查 ==========

if grep -q "\[MapStall\]" "${LOG_FILE}"; then
  STALL_COUNT=$(grep -c "\[MapStall\]" "${LOG_FILE}" || echo "0")
  warn_msg "MapStall observed ${STALL_COUNT} times (map may have stalled)"
else
  pass_msg "no MapStall"
fi

# ========== 9. CargoCommit 有效性检查 ==========

python3 - "$LOG_FILE" <<'PY'
import re, sys
log = sys.argv[1]
consecutive_fail = 0
max_consecutive = 0
total_active = 0
total_removed_gt0 = 0

pat = re.compile(r'\[CargoCommit\].*?active_boxes=(\d+).*?removed=(\d+)')

with open(log, 'r', errors='ignore') as f:
    for line in f:
        m = pat.search(line)
        if not m:
            continue
        active = int(m.group(1))
        removed = int(m.group(2))
        if active > 0:
            total_active += 1
            if removed > 0:
                total_removed_gt0 += 1
                consecutive_fail = 0
            else:
                consecutive_fail += 1
                max_consecutive = max(max_consecutive, consecutive_fail)

if total_active == 0:
    print('[VALIDATION] WARN: no active_boxes observed')
else:
    print('[VALIDATION] INFO: active_boxes>0: %d, removed>0: %d, max_consecutive_fail: %d' % (
        total_active, total_removed_gt0, max_consecutive))
    if max_consecutive > 3:
        print('[VALIDATION] FAIL: active_boxes>0 but removed=0 for %d consecutive frames' % max_consecutive)
        sys.exit(1)
    print('[VALIDATION] PASS: CargoCommit removal working')
PY

# ========== 10. CleanMap 检查 ==========

CLEAN_POINTS=$(grep "\[CleanMap\].*rebuilt" "${LOG_FILE}" | tail -1 | grep -oP 'points=\K\d+' || echo "0")
if [ "${CLEAN_POINTS}" -gt 0 ]; then
  pass_msg "CleanMap rebuilt points=${CLEAN_POINTS} > 0"
else
  warn_msg "CleanMap rebuilt points=0 (may need more keyframes)"
fi

# ========== 11. display_points 检查 ==========

DISPLAY_POINTS=$(grep "\[MapAlive\]" "${LOG_FILE}" | tail -1 | grep -oP 'display_points=\K\d+' || echo "0")
if [ "${DISPLAY_POINTS}" -gt 0 ]; then
  pass_msg "display_points=${DISPLAY_POINTS} > 0"
else
  warn_msg "display_points=0"
fi

# ========== 12. RViz 配置检查 ==========

RVIZ="/home/ydkj/NDT-slam-ws/src/ndt_slam/launch/rviz.rviz"
if [ -f "$RVIZ" ]; then
  # 检查 display_map 是否存在且启用
  if grep -q "Topic: /display_map" "$RVIZ"; then
    pass_msg "RViz display_map topic exists"
  else
    fail "RViz display_map topic missing"
  fi

  # 检查 objects_clean 是否存在且启用
  if grep -q "Topic: /objects_clean\|Topic: /display_map_objects_clean" "$RVIZ"; then
    pass_msg "RViz objects_clean topic exists"
  else
    fail "RViz objects_clean topic missing"
  fi

  # 检查 cargo_core_bbox_marker 是否存在
  if grep -q "Marker Topic: /cargo_core_bbox_marker" "$RVIZ"; then
    pass_msg "RViz cargo_core_bbox_marker exists"
  else
    fail "RViz cargo_core_bbox_marker missing"
  fi
fi

# ========== 13. 日志密度检查 ==========

LOG_LINES=$(wc -l < "${LOG_FILE}" || echo "0")
LOG_DURATION=$(grep "\[PipelineSummary\]" "${LOG_FILE}" | tail -1 | grep -oP 'stamp=\K[0-9.]+' || echo "0")
if [ "${LOG_LINES}" -gt 0 ] && [ "$(echo "$LOG_DURATION > 0" | bc)" -eq 1 ]; then
  # 简单估算：每秒 INFO 行数
  INFO_LINES=$(grep -c "\[INFO\]" "${LOG_FILE}" || echo "0")
  warn_msg "total INFO lines: ${INFO_LINES}"
fi

echo ""
echo "========== [VALIDATION] PASS: cargo pipeline v13 checks passed =========="
