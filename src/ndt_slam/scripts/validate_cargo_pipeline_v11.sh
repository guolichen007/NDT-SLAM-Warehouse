#!/usr/bin/env bash
set -euo pipefail

LOG_FILE="${1:-/tmp/cargo_v11_real.log}"

echo "========== Cargo Pipeline Validation v11 =========="
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

if grep "XmlRpcClient::writeRequest" "${LOG_FILE}" | grep -q "ndt_slam\|cargo"; then
  fail "XmlRpcClient write error from our nodes"
fi

pass_msg "launch clean"

# ========== 2. core_box_only 模式检查 ==========

if grep -q "\[CargoCoreBox\] mode=core_box_only" "${LOG_FILE}"; then
  pass_msg "CargoCoreBox mode=core_box_only"
else
  fail "missing CargoCoreBox mode=core_box_only"
fi

if grep -q "\[CargoMarkerConfig\] core=1 raw=0 stable=0 remove=0 forbidden=0" "${LOG_FILE}"; then
  pass_msg "CargoMarkerConfig core=1 raw=0 stable=0 remove=0 forbidden=0"
else
  fail "CargoMarkerConfig not correct"
fi

# ========== 3. PoseFreeze 检查 ==========

if grep -q "\[PoseFreeze\]" "${LOG_FILE}"; then
  pass_msg "PoseFreeze exists"
else
  warn_msg "no PoseFreeze observed"
fi

# ========== 4. PipelineSummary 检查 ==========

grep -q "\[PipelineSummary\]" "${LOG_FILE}" || fail "missing [PipelineSummary]"
pass_msg "PipelineSummary exists"

# ========== 5. CargoCommit 顺序检查 ==========

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

# ========== 6. CargoActiveSummary 检查 ==========

if grep -q "\[CargoActiveSummary\]" "${LOG_FILE}"; then
  pass_msg "CargoActiveSummary exists"
else
  fail "missing [CargoActiveSummary]"
fi

# ========== 7. active_boxes 有效性检查 ==========

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

# ========== 8. CargoCoreDisplayGate 检查 ==========

if grep -q "\[CargoCoreDisplayGate\]" "${LOG_FILE}"; then
  pass_msg "CargoCoreDisplayGate exists"
else
  warn_msg "missing [CargoCoreDisplayGate]"
fi

# ========== 9. CargoHistoryAdd 检查 ==========

if grep -q "\[CargoHistoryAdd\]" "${LOG_FILE}"; then
  HISTORY_COUNT=$(grep -c "\[CargoHistoryAdd\]" "${LOG_FILE}" || true)
  if [ "${HISTORY_COUNT}" -lt 10 ]; then
    fail "CargoHistoryAdd volumes < 10 (${HISTORY_COUNT})"
  fi
  pass_msg "CargoHistoryAdd observed: ${HISTORY_COUNT} volumes"
else
  fail "missing [CargoHistoryAdd]"
fi

# ========== 10. DynamicHistoryEraser 检查 ==========

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

# ========== 11. CleanMap 检查 ==========

CLEAN_POINTS=$(grep "\[CleanMap\].*rebuilt" "${LOG_FILE}" | tail -1 | grep -oP 'points=\K\d+' || echo "0")
if [ "${CLEAN_POINTS}" -gt 0 ]; then
  pass_msg "CleanMap rebuilt points=${CLEAN_POINTS} > 0"
else
  fail "CleanMap rebuilt points=0"
fi

# ========== 12. band_height 边界检查 ==========

if grep -q "band_height=0.10 not in" "${LOG_FILE}"; then
  fail "band_height=0.10 boundary bug still present"
fi
pass_msg "no band_height=0.10 boundary bug"

# ========== 13. constrain_yaw 检查 ==========

if grep -q "lock_yaw: true" src/ndt_slam/config/live_longterm_mapping.yaml 2>/dev/null; then
  pass_msg "lock_yaw=true in config"
else
  warn_msg "lock_yaw is not true in config"
fi

# ========== 14. source=objects_base 检查 ==========

if grep -q "\[CargoCommit\].*source=objects_base" "${LOG_FILE}"; then
  pass_msg "CargoCommit source=objects_base"
else
  fail "CargoCommit missing source=objects_base"
fi

# ========== 15. PayloadTrackInfoCore 检查 ==========

if grep -q "\[PayloadTrackInfoCore\].*source=CORE_BOX" "${LOG_FILE}"; then
  pass_msg "PayloadTrackInfoCore source=CORE_BOX exists"
else
  fail "missing [PayloadTrackInfoCore] source=CORE_BOX"
fi

# ========== 16. TrackCleanup 检查 ==========

if grep -q "\[TrackCleanup\]" "${LOG_FILE}"; then
  CLEANUP_COUNT=$(grep -c "\[TrackCleanup\]" "${LOG_FILE}" || true)
  pass_msg "TrackCleanup observed: ${CLEANUP_COUNT} tracks cleaned"
else
  warn_msg "no TrackCleanup (no stale tracks in this run)"
fi

echo ""
echo "========== [VALIDATION] PASS: cargo pipeline v11 checks passed =========="
