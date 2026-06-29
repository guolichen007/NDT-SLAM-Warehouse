#!/usr/bin/env bash
set -euo pipefail

LOG=${1:-/tmp/cargo_v8_stable_r3.log}

fail() {
  echo "[VALIDATION] FAIL: $1"
  exit 1
}

warn() {
  echo "[VALIDATION] WARN: $1"
}

# 检查必要日志
grep -q "\[EKF\]" "$LOG" || fail "missing EKF summary"
grep -q "\[Perf\]" "$LOG" || fail "missing Perf summary"
grep -q "\[Cargo\]" "$LOG" || fail "missing Cargo summary"

# 检查不应该存在的配置
if grep -q "lock_yaw: true" "$LOG"; then
  fail "hard lock yaw is enabled"
fi

# 检查不应该存在的状态机
if grep -q "STATIONARY_ANCHOR\|RELEASE_BLEND" "$LOG"; then
  fail "PoseFreeze state machine should not be active"
fi

# 检查 INFO 行数（60 秒内）
INFO_LINES=$(grep -c "\[INFO\]" "$LOG" || true)
if [ "$INFO_LINES" -gt 600 ]; then
  fail "too many INFO lines in 60s: $INFO_LINES"
fi

# 检查斜向运动模式
grep -q "mode=DIAG" "$LOG" || warn "no diagonal mode observed in this run"

# 检查 CargoCommit
grep -q "CargoCommit.*removed=[1-9]" "$LOG" || \
  fail "CargoCommit removal not observed"

# 检查 DynamicHistoryEraser
grep -q "DynamicHistoryEraser.*erased_objects=[1-9]" "$LOG" || \
  fail "DynamicHistoryEraser not observed"

# 检查 cargo 话题
if rostopic list 2>/dev/null | grep -E "cargo_forbidden_overlay|cargo_forbidden_height|cargo_raw_bbox|cargo_stable_bbox|cargo_remove_bbox|cargo_forbidden_grid|cargo_collision_warning|cargo_predicted_path"; then
  fail "forbidden/debug cargo topics still exist"
fi

rostopic list 2>/dev/null | grep -q "/cargo_core_bbox_marker" || \
  fail "/cargo_core_bbox_marker missing"

# 检查 EKF 接受率
EKF_ACCEPT=$(grep -c "accept=1" "$LOG" || true)
EKF_REJECT=$(grep -c "accept=0" "$LOG" || true)
if [ "$EKF_ACCEPT" -eq 0 ] && [ "$EKF_REJECT" -gt 0 ]; then
  fail "EKF never accepted NDT"
fi

# 检查 fitness
HIGH_FITNESS=$(grep -c "fitness=0\.[1-9]" "$LOG" || true)
if [ "$HIGH_FITNESS" -gt 100 ]; then
  warn "high fitness observed $HIGH_FITNESS times"
fi

echo "[VALIDATION] PASS: v8-stable-r3 basic checks passed"
echo "[VALIDATION] EKF accepted=$EKF_ACCEPT rejected=$EKF_REJECT"
echo "[VALIDATION] INFO lines=$INFO_LINES"
