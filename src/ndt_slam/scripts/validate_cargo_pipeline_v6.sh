#!/usr/bin/env bash
set -euo pipefail

LOG_FILE="${1:-/tmp/cargo_pipeline_test.log}"

echo "========== Cargo Pipeline Validation v6 =========="
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

# 1. PipelineSummary 存在
grep -q "\[PipelineSummary\]" "${LOG_FILE}" || fail "missing [PipelineSummary]"
pass_msg "PipelineSummary exists"

# 2. CargoCommit 在 MapCommit 前
python3 - "$LOG_FILE" <<'PY'
import re, sys
log = sys.argv[1]
order = {}
with open(log, 'r', errors='ignore') as f:
    for idx, line in enumerate(f):
        m = re.search(r'\[(CargoCommit|MapCommit)\].*?(?:kf|seq)=(\d+)', line)
        if not m:
            continue
        tag, seq = m.group(1), int(m.group(2))
        order.setdefault(seq, {})[tag] = idx

bad = []
checked = 0
for seq, d in order.items():
    if 'MapCommit' not in d or 'CargoCommit' not in d:
        continue
    checked += 1
    if d['CargoCommit'] > d['MapCommit']:
        bad.append((seq, 'CargoCommit after MapCommit'))

if checked == 0:
    print('[VALIDATION] WARN: no complete sequence found')
else:
    if bad:
        print('[VALIDATION] FAIL: wrong pipeline order')
        for b in bad[:10]:
            print('  seq=%s %s' % b)
        sys.exit(1)
    print('[VALIDATION] PASS: CargoCommit before MapCommit (%d checked)' % checked)
PY

# 3. v6: BoxFollow SWING_FOLLOW 存在
if grep -q "\[BoxFollow\].*mode=SWING_FOLLOW" "${LOG_FILE}"; then
  pass_msg "BoxFollow SWING_FOLLOW observed"
else
  warn_msg "no SWING_FOLLOW (crane may not have been stationary long enough)"
fi

# 4. v6: band_height 边界不再被拒
if grep -q "band_height=0.10 not in" "${LOG_FILE}"; then
  fail "band_height=0.10 boundary bug still present"
fi
pass_msg "no band_height=0.10 boundary bug"

# 5. v6: CargoFallbackActive 存在（如果 size_too_large 发生）
if grep -q "\[CargoFallbackActive\]" "${LOG_FILE}"; then
  FALLBACK_COUNT=$(grep -c "\[CargoFallbackActive\]" "${LOG_FILE}" || true)
  pass_msg "CargoFallbackActive observed: ${FALLBACK_COUNT} times"
else
  warn_msg "no CargoFallbackActive (size_too_large may not have occurred)"
fi

# 6. v6: CargoHistoryAdd 存在
if grep -q "\[CargoHistoryAdd\]" "${LOG_FILE}"; then
  HISTORY_COUNT=$(grep -c "\[CargoHistoryAdd\]" "${LOG_FILE}" || true)
  pass_msg "CargoHistoryAdd observed: ${HISTORY_COUNT} volumes"
else
  fail "missing [CargoHistoryAdd]"
fi

# 7. v6: DynamicHistoryEraser 存在
if grep -q "\[DynamicHistoryEraser\]" "${LOG_FILE}"; then
  ERASER_ERASED=$(grep "\[DynamicHistoryEraser\]" "${LOG_FILE}" | tail -1 | grep -oP 'erased_objects=\K\d+' || echo "0")
  if [ "${ERASER_ERASED}" -gt 0 ]; then
    pass_msg "DynamicHistoryEraser erased_objects=${ERASER_ERASED} > 0"
  else
    warn_msg "DynamicHistoryEraser exists but erased_objects=0"
  fi
else
  fail "missing [DynamicHistoryEraser]"
fi

# 8. v6: MotionGate stationary_freeze 存在
if grep -q "\[MotionGate\] stationary_freeze" "${LOG_FILE}"; then
  FREEZE_COUNT=$(grep -c "\[MotionGate\] stationary_freeze" "${LOG_FILE}" || true)
  pass_msg "MotionGate stationary_freeze observed: ${FREEZE_COUNT} times"
else
  warn_msg "no stationary_freeze"
fi

# 9. v6: 静止期间不能有 MapCommit
python3 - "$LOG_FILE" <<'PY'
import re, sys
log = sys.argv[1]

with open(log, 'r', errors='ignore') as f:
    lines = f.readlines()

last_freeze_line = -1
commit_in_freeze = []
for idx, line in enumerate(lines):
    if '[MotionGate] stationary_freeze' in line:
        last_freeze_line = idx
    elif '[MapCommit]' in line and last_freeze_line >= 0:
        if idx - last_freeze_line < 5:
            commit_in_freeze.append((last_freeze_line, idx))

if commit_in_freeze:
    print('[VALIDATION] FAIL: MapCommit during stationary_freeze')
    for f, c in commit_in_freeze[:3]:
        print('  freeze at line %d, commit at line %d' % (f, c))
    sys.exit(1)
else:
    print('[VALIDATION] PASS: no MapCommit during stationary_freeze')
PY

# 10. active_boxes>0 时 removed>0
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
    if total_removed_gt0 == 0:
        print('[VALIDATION] FAIL: active_boxes>0 but never removed any points')
        sys.exit(1)
    print('[VALIDATION] PASS: CargoCommit removal working')
PY

# 11. CleanMap points > 0
CLEAN_POINTS=$(grep "\[CleanMap\].*rebuilt" "${LOG_FILE}" | tail -1 | grep -oP 'points=\K\d+' || echo "0")
if [ "${CLEAN_POINTS}" -gt 0 ]; then
  pass_msg "CleanMap rebuilt points=${CLEAN_POINTS} > 0"
else
  fail "CleanMap rebuilt points=0"
fi

# 12. PayloadTrackInfoCore source=CORE_BOX
if grep -q "\[PayloadTrackInfoCore\].*source=CORE_BOX" "${LOG_FILE}"; then
  pass_msg "PayloadTrackInfoCore source=CORE_BOX exists"
else
  fail "missing [PayloadTrackInfoCore] source=CORE_BOX"
fi

# 13. 不允许出现旧 reinit 逻辑
if grep -q "reinit size after 3 frames" "${LOG_FILE}"; then
  fail "old reinit logic still present"
fi
pass_msg "no old reinit logic"

# 14. source=objects_base
if grep -q "\[CargoCommit\].*source=objects_base" "${LOG_FILE}"; then
  pass_msg "CargoCommit source=objects_base"
else
  fail "CargoCommit missing source=objects_base"
fi

# 15. constrain_yaw=true 在配置中
if grep -q "constrain_yaw: true" src/ndt_slam/config/live_longterm_mapping.yaml 2>/dev/null; then
  pass_msg "constrain_yaw=true in config"
else
  fail "constrain_yaw is not true in config"
fi

# 16. TrackCleanup 存在（如果僵尸 track 发生）
if grep -q "\[TrackCleanup\]" "${LOG_FILE}"; then
  CLEANUP_COUNT=$(grep -c "\[TrackCleanup\]" "${LOG_FILE}" || true)
  pass_msg "TrackCleanup observed: ${CLEANUP_COUNT} tracks cleaned"
else
  warn_msg "no TrackCleanup (no stale tracks in this run)"
fi

echo ""
echo "========== [VALIDATION] PASS: cargo pipeline v6 checks passed =========="
