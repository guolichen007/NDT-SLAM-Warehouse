#!/usr/bin/env bash
set -euo pipefail

LOG_FILE="${1:-/tmp/cargo_pipeline_test.log}"

echo "========== Cargo Pipeline Validation v3 =========="
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

# 3. active_boxes>0 时 removed>0 不能连续失败超过 5 次
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
    print('[VALIDATION] INFO: active_boxes>0 frames: %d, removed>0 frames: %d, max_consecutive_fail: %d' % (
        total_active, total_removed_gt0, max_consecutive))
    if max_consecutive > 5:
        print('[VALIDATION] FAIL: active_boxes>0 but removed=0 for %d consecutive frames' % max_consecutive)
        sys.exit(1)
    if total_removed_gt0 == 0:
        print('[VALIDATION] FAIL: active_boxes>0 but never removed any points')
        sys.exit(1)
    print('[VALIDATION] PASS: CargoCommit removal working')
PY

# 4. 不允许出现旧字符串 "reinit size after 3 frames"
if grep -q "reinit size after 3 frames" "${LOG_FILE}"; then
  fail "old reinit logic still present: 'reinit size after 3 frames'"
fi
pass_msg "no old reinit logic"

# 5. CargoBoxReinitCheck 存在
if grep -q "\[CargoBoxReinitCheck\]" "${LOG_FILE}"; then
  REINIT_ACCEPTED=$(grep -c "\[CargoBoxReinitCheck\].*accepted=1" "${LOG_FILE}" || true)
  REINIT_REJECTED=$(grep -c "\[CargoBoxReinitCheck\].*accepted=0" "${LOG_FILE}" || true)
  pass_msg "CargoBoxReinitCheck exists: accepted=${REINIT_ACCEPTED} rejected=${REINIT_REJECTED}"
else
  warn_msg "no CargoBoxReinitCheck (size jump may not have occurred)"
fi

# 6. PipelineSummary 中的指标
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

# 7. CleanMap points > 0
CLEAN_POINTS=$(grep "\[CleanMap\].*rebuilt" "${LOG_FILE}" | tail -1 | grep -oP 'points=\K\d+' || echo "0")
if [ "${CLEAN_POINTS}" -gt 0 ]; then
  pass_msg "CleanMap rebuilt points=${CLEAN_POINTS} > 0"
else
  fail "CleanMap rebuilt points=0"
fi

# 8. PayloadTrackInfoCore source=CORE_BOX
if grep -q "\[PayloadTrackInfoCore\].*source=CORE_BOX" "${LOG_FILE}"; then
  pass_msg "PayloadTrackInfoCore source=CORE_BOX exists"
else
  fail "missing [PayloadTrackInfoCore] source=CORE_BOX"
fi

# 9. 不允许出现 oversized HumanEvent
if grep -E "HumanEvent created.*points=([3-9][0-9]{2,}|[1-9][0-9]{3,})" "${LOG_FILE}" >/dev/null; then
  fail "HumanEvent points too large"
fi
pass_msg "no oversized HumanEvent"

# 10. 同一个 keyframe seq 重复 CommitStart 超过 10 次
python3 - "$LOG_FILE" <<'PY'
import re, sys
from collections import Counter
log = sys.argv[1]
seq_counts = Counter()

pat = re.compile(r'\[CommitStart\].*?seq=(\d+)')

with open(log, 'r', errors='ignore') as f:
    for line in f:
        m = pat.search(line)
        if m:
            seq_counts[int(m.group(1))] += 1

bad = [(seq, count) for seq, count in seq_counts.items() if count > 10]
if bad:
    print('[VALIDATION] FAIL: duplicate CommitStart')
    for seq, count in sorted(bad)[:5]:
        print('  seq=%d repeated %d times' % (seq, count))
    sys.exit(1)
print('[VALIDATION] PASS: no excessive duplicate CommitStart')
PY

# 11. DuplicateFrameGuard 存在（如果日志中有）
if grep -q "\[DuplicateFrameGuard\]" "${LOG_FILE}"; then
  GUARD_COUNT=$(grep -c "\[DuplicateFrameGuard\]" "${LOG_FILE}" || true)
  warn_msg "DuplicateFrameGuard triggered ${GUARD_COUNT} times"
fi

# 12. MotionGate 存在（如果日志中有）
if grep -q "\[MotionGate\]" "${LOG_FILE}"; then
  pass_msg "MotionGate exists"
fi

# 13. CargoActiveBox 存在
if grep -q "\[CargoActiveBox\]" "${LOG_FILE}"; then
  pass_msg "CargoActiveBox exists"
else
  warn_msg "missing [CargoActiveBox]"
fi

# 14. 检查 source=objects_base
if grep -q "\[CargoCommit\].*source=objects_base" "${LOG_FILE}"; then
  pass_msg "CargoCommit source=objects_base"
else
  fail "CargoCommit missing source=objects_base"
fi

echo ""
echo "========== [VALIDATION] PASS: cargo pipeline v3 checks passed =========="
