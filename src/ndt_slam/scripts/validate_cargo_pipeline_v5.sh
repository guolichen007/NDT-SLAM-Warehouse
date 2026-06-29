#!/usr/bin/env bash
set -euo pipefail

LOG_FILE="${1:-/tmp/cargo_pipeline_test.log}"

echo "========== Cargo Pipeline Validation v5 =========="
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

# 3. DuplicateFrameGuard 存在
if grep -q "\[DuplicateFrameGuard\]" "${LOG_FILE}"; then
  SKIP_COUNT=$(grep -c "\[DuplicateFrameGuard\]" "${LOG_FILE}" || true)
  pass_msg "DuplicateFrameGuard observed: skipped=${SKIP_COUNT}"
else
  warn_msg "no DuplicateFrameGuard observed (may be no duplicates in this run)"
fi

# 4. MotionGate stationary_freeze 存在
if grep -q "\[MotionGate\] stationary_freeze" "${LOG_FILE}"; then
  FREEZE_COUNT=$(grep -c "\[MotionGate\] stationary_freeze" "${LOG_FILE}" || true)
  pass_msg "MotionGate stationary_freeze observed: ${FREEZE_COUNT} times"
else
  warn_msg "no stationary_freeze (crane may not have been stationary long enough)"
fi

# 5. 静止期间不能有 MapCommit
python3 - "$LOG_FILE" <<'PY'
import re, sys
log = sys.argv[1]

# 找到 stationary_freeze 段
freeze_ranges = []
commit_in_freeze = []

with open(log, 'r', errors='ignore') as f:
    lines = f.readlines()

# 简化检查：如果 stationary_freeze 后紧跟 MapCommit，说明有误触发
last_freeze_line = -1
for idx, line in enumerate(lines):
    if '[MotionGate] stationary_freeze' in line:
        last_freeze_line = idx
    elif '[MapCommit]' in line and last_freeze_line >= 0:
        # 检查是否在 freeze 段内
        if idx - last_freeze_line < 5:  # 5 行以内
            commit_in_freeze.append((last_freeze_line, idx))

if commit_in_freeze:
    print('[VALIDATION] FAIL: MapCommit during stationary_freeze')
    for f, c in commit_in_freeze[:3]:
        print('  freeze at line %d, commit at line %d' % (f, c))
    sys.exit(1)
else:
    print('[VALIDATION] PASS: no MapCommit during stationary_freeze')
PY

# 6. TrackCleanup 存在
if grep -q "\[TrackCleanup\]" "${LOG_FILE}"; then
  CLEANUP_COUNT=$(grep -c "\[TrackCleanup\]" "${LOG_FILE}" || true)
  pass_msg "TrackCleanup observed: ${CLEANUP_COUNT} tracks cleaned"
else
  warn_msg "no TrackCleanup (no stale tracks in this run)"
fi

# 7. PipelineSummary 中的指标
python3 - "$LOG_FILE" <<'PY'
import re, sys
log = sys.argv[1]

pat = re.compile(
    r'\[PipelineSummary\].*?raw=(\d+).*?cargo_removed=(\d+).*?clean_points=(\d+)'
)

last = None
with open(log, 'r', errors='ignore') as f:
    for line in f:
        m = pat.search(line)
        if m:
            last = m

if not last:
    print('[VALIDATION] FAIL: no PipelineSummary found')
    sys.exit(1)

raw = int(last.group(1))
cargo_removed = int(last.group(2))
clean_points = int(last.group(3))

print('[VALIDATION] INFO: last PipelineSummary: raw=%d cargo_removed=%d clean_points=%d' % (
    raw, cargo_removed, clean_points))

if clean_points == 0:
    print('[VALIDATION] FAIL: clean_points=0')
    sys.exit(1)

print('[VALIDATION] PASS: PipelineSummary metrics valid')
PY

# 8. CleanMap points > 0
CLEAN_POINTS=$(grep "\[CleanMap\].*rebuilt" "${LOG_FILE}" | tail -1 | grep -oP 'points=\K\d+' || echo "0")
if [ "${CLEAN_POINTS}" -gt 0 ]; then
  pass_msg "CleanMap rebuilt points=${CLEAN_POINTS} > 0"
else
  fail "CleanMap rebuilt points=0"
fi

# 9. PayloadTrackInfoCore source=CORE_BOX
if grep -q "\[PayloadTrackInfoCore\].*source=CORE_BOX" "${LOG_FILE}"; then
  pass_msg "PayloadTrackInfoCore source=CORE_BOX exists"
else
  fail "missing [PayloadTrackInfoCore] source=CORE_BOX"
fi

# 10. 不允许出现 oversized HumanEvent
if grep -E "HumanEvent created.*points=([3-9][0-9]{2,}|[1-9][0-9]{3,})" "${LOG_FILE}" >/dev/null; then
  fail "HumanEvent points too large"
fi
pass_msg "no oversized HumanEvent"

# 11. 不允许出现旧 reinit 逻辑
if grep -q "reinit size after 3 frames" "${LOG_FILE}"; then
  fail "old reinit logic still present"
fi
pass_msg "no old reinit logic"

# 12. CargoBoxReinitCheck 存在
if grep -q "\[CargoBoxReinitCheck\]" "${LOG_FILE}"; then
  REINIT_ACCEPTED=$(grep -c "\[CargoBoxReinitCheck\].*accepted=1" "${LOG_FILE}" || true)
  REINIT_REJECTED=$(grep -c "\[CargoBoxReinitCheck\].*accepted=0" "${LOG_FILE}" || true)
  pass_msg "CargoBoxReinitCheck: accepted=${REINIT_ACCEPTED} rejected=${REINIT_REJECTED}"
else
  warn_msg "no CargoBoxReinitCheck"
fi

# 13. active_boxes>0 时 removed>0
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
    if max_consecutive > 5:
        print('[VALIDATION] FAIL: active_boxes>0 but removed=0 for %d consecutive frames' % max_consecutive)
        sys.exit(1)
    if total_removed_gt0 == 0:
        print('[VALIDATION] FAIL: active_boxes>0 but never removed any points')
        sys.exit(1)
    print('[VALIDATION] PASS: CargoCommit removal working')
PY

# 14. source=objects_base
if grep -q "\[CargoCommit\].*source=objects_base" "${LOG_FILE}"; then
  pass_msg "CargoCommit source=objects_base"
else
  fail "CargoCommit missing source=objects_base"
fi

# 15. ValidationHint 存在
if grep -q "\[ValidationHint\]" "${LOG_FILE}"; then
  pass_msg "ValidationHint exists"
else
  warn_msg "missing [ValidationHint]"
fi

echo ""
echo "========== [VALIDATION] PASS: cargo pipeline v5 checks passed =========="
