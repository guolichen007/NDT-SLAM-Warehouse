#!/usr/bin/env bash
set -euo pipefail

LOG_FILE="${1:-/tmp/cargo_pipeline_test.log}"

echo "========== Cargo Pipeline Validation =========="
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

# 1. 必须有核心日志
grep -q "\[CommitStart\]" "${LOG_FILE}" || fail "missing [CommitStart]"
grep -q "\[GroundSplit\]" "${LOG_FILE}" || fail "missing [GroundSplit]"
grep -q "\[ChannelFilter\]" "${LOG_FILE}" || fail "missing [ChannelFilter]"
grep -q "\[CargoCommit\]" "${LOG_FILE}" || fail "missing [CargoCommit]"
grep -q "\[HumanFilter\]" "${LOG_FILE}" || fail "missing [HumanFilter]"
grep -q "\[MapCommitInput\]" "${LOG_FILE}" || fail "missing [MapCommitInput]"
grep -q "\[MapCommit\]" "${LOG_FILE}" || fail "missing [MapCommit]"

pass_msg "required log tags exist"

# 2. 检查是否出现错误顺序：MapCommit 在 CargoCommit 前
python3 - "$LOG_FILE" <<'PY'
import re, sys
log = sys.argv[1]
order = {}
with open(log, 'r', errors='ignore') as f:
    for idx, line in enumerate(f):
        m = re.search(r'\[(CargoCommit|HumanFilter|MapCommitInput|MapCommit|CargoBoxV2|PayloadTrack)\].*?seq=(\d+)', line)
        if not m:
            continue
        tag, seq = m.group(1), int(m.group(2))
        order.setdefault(seq, {})[tag] = idx

bad = []
checked = 0
for seq, d in order.items():
    if 'MapCommit' not in d:
        continue
    checked += 1
    for tag in ['CargoCommit', 'HumanFilter', 'MapCommitInput']:
        if tag not in d:
            bad.append((seq, f'missing {tag} before MapCommit'))
        elif d[tag] > d['MapCommit']:
            bad.append((seq, f'{tag} appears after MapCommit'))

if checked == 0:
    print('[VALIDATION] FAIL: no complete MapCommit sequence found')
    sys.exit(1)

if bad:
    print('[VALIDATION] FAIL: wrong pipeline order')
    for b in bad[:20]:
        print('  seq=%s %s' % b)
    sys.exit(1)

print('[VALIDATION] PASS: CargoCommit/HumanFilter/MapCommitInput all before MapCommit')
PY

# 3. 检查 commit_objects 是否小于等于 raw_objects
python3 - "$LOG_FILE" <<'PY'
import re, sys
log = sys.argv[1]
bad = []
seen = 0
removed_seen = 0
with open(log, 'r', errors='ignore') as f:
    for line in f:
        if '[MapCommitInput]' not in line:
            continue
        seen += 1
        vals = dict(re.findall(r'(raw|ground|raw_objects|commit_objects|commit_total|cargo_removed|human_removed)=(\d+)', line))
        if not vals:
            continue
        raw_objects = int(vals.get('raw_objects', 0))
        commit_objects = int(vals.get('commit_objects', 0))
        cargo_removed = int(vals.get('cargo_removed', 0))
        human_removed = int(vals.get('human_removed', 0))

        if commit_objects > raw_objects:
            bad.append('commit_objects > raw_objects: ' + line.strip())

        if cargo_removed > 0 or human_removed > 0:
            removed_seen += 1
            if commit_objects >= raw_objects:
                bad.append('removed>0 but commit_objects not reduced: ' + line.strip())

if seen == 0:
    print('[VALIDATION] FAIL: no MapCommitInput lines')
    sys.exit(1)

if bad:
    print('[VALIDATION] FAIL: invalid MapCommitInput')
    for x in bad[:10]:
        print('  ' + x)
    sys.exit(1)

print('[VALIDATION] PASS: MapCommitInput object counts valid')
if removed_seen == 0:
    print('[VALIDATION] WARN: no cargo/human removed observed in this log')
else:
    print('[VALIDATION] PASS: removal observed in MapCommitInput')
PY

# 4. 检查 CargoBoxV2 是否至少有有效框
if grep -q "\[CargoBoxV2\].*valid=1" "${LOG_FILE}"; then
  pass_msg "CargoBoxV2 valid box observed"
else
  echo "[VALIDATION] WARN: no CargoBoxV2 valid=1 observed; cargo may be absent or detection failed"
fi

# 5. 检查 size jump 是否连续刷屏
SIZE_JUMP_COUNT=$(grep -c "\[CargoBoxV2SizeGate\]" "${LOG_FILE}" || true)
VALID_BOX_COUNT=$(grep -c "\[CargoBoxV2\].*valid=1" "${LOG_FILE}" || true)

echo "[VALIDATION] INFO: size_jump_count=${SIZE_JUMP_COUNT}, valid_box_count=${VALID_BOX_COUNT}"

if [ "${SIZE_JUMP_COUNT}" -gt 50 ] && [ "${VALID_BOX_COUNT}" -eq 0 ]; then
  fail "too many size jumps and no valid boxes"
fi

# 6. HumanEvent 大点数误检检查
if grep -E "HumanEvent created.*points=([3-9][0-9]{2,}|[1-9][0-9]{3,})" "${LOG_FILE}" >/dev/null; then
  fail "HumanEvent points too large; likely cargo misclassified as human"
else
  pass_msg "no oversized HumanEvent"
fi

# 7. CleanMap 3D gate 检查
if grep -q "\[CleanMapDynamicGate3D\]" "${LOG_FILE}"; then
  pass_msg "CleanMapDynamicGate3D exists"
else
  fail "missing CleanMapDynamicGate3D; old 2D deny may still be used"
fi

# 8. CargoBoxFix 日志必须存在
grep -q "\[CargoBoxFix\]" "${LOG_FILE}" || fail "missing [CargoBoxFix]; box correction not verified"

# 9. CargoMarkerCheck 日志必须存在
grep -q "\[CargoRemoveBoxCheck\]" "${LOG_FILE}" || fail "missing [CargoRemoveBoxCheck]; remove box not verified"

# 10. Cargo core box 至少有一次 valid=1
if grep -q "\[CargoBoxFix\].*valid=1" "${LOG_FILE}"; then
  pass_msg "CargoBoxFix valid core box observed"
else
  fail "no valid cargo core box observed"
fi

# 11. 检查 remove_box z_down_expand
python3 - "$LOG_FILE" <<'PY'
import re, sys
log = sys.argv[1]
bad = []
seen = 0

pat = re.compile(r'\[CargoRemoveBoxCheck\].*?z_down_expand=([0-9.]+).*?z_up_expand=([0-9.]+)')

with open(log, 'r', errors='ignore') as f:
    for line in f:
        m = pat.search(line)
        if not m:
            continue
        seen += 1
        down = float(m.group(1))
        up = float(m.group(2))
        if down > 0.05:
            bad.append(f'z_down_expand too large: {down:.2f}; line={line.strip()}')
        if up > 0.25:
            bad.append(f'z_up_expand too large: {up:.2f}; line={line.strip()}')

if seen == 0:
    print('[VALIDATION] FAIL: no CargoRemoveBoxCheck lines parsed')
    sys.exit(1)

if bad:
    print('[VALIDATION] FAIL: remove box z expansion invalid')
    for x in bad[:10]:
        print('  ' + x)
    sys.exit(1)

print('[VALIDATION] PASS: remove box z expansion valid')
PY

# 12. 检查是否还在使用旧大框
if grep -q "\[CargoBoxReject\].*old_box_used_for_marker=1" "${LOG_FILE}"; then
  fail "old rejected box is still used for marker"
fi

if grep -q "\[CargoBoxReject\].*old_box_used_for_remove=1" "${LOG_FILE}"; then
  fail "old rejected box is still used for remove"
fi

pass_msg "rejected old box is not reused"

# 13. 检查 MapWrite 日志
if grep -q "\[MapWrite\]" "${LOG_FILE}"; then
  pass_msg "MapWrite log exists"
else
  echo "[VALIDATION] WARN: missing [MapWrite] log"
fi

# 14. 检查是否还有直接调用 addKeyFrame(pose, cloud) 的情况
if grep -q "addKeyFrame(pose, cloud" "${LOG_FILE}"; then
  fail "direct addKeyFrame(pose, cloud) call detected - should use commitKeyFrameWithDynamicFiltering"
fi

# 15. 检查 BevObsUpdate（CleanMap 依赖此数据）
if grep -q "\[BevObsUpdate\]" "${LOG_FILE}"; then
  BEV_CELLS=$(grep "\[BevObsUpdate\]" "${LOG_FILE}" | tail -1 | grep -oP 'unique_cells=\K\d+' || echo "0")
  if [ "${BEV_CELLS}" -gt 0 ]; then
    pass_msg "BevObsUpdate unique_cells=${BEV_CELLS} > 0"
  else
    fail "BevObsUpdate unique_cells=0; CleanMap will be empty"
  fi
else
  fail "missing [BevObsUpdate]; CleanMap cannot work"
fi

# 16. 检查 CleanMap 是否有实际点数
CLEAN_POINTS=$(grep "\[CleanMap\].*rebuilt" "${LOG_FILE}" | tail -1 | grep -oP 'points=\K\d+' || echo "0")
if [ "${CLEAN_POINTS}" -gt 0 ]; then
  pass_msg "CleanMap rebuilt points=${CLEAN_POINTS} > 0"
else
  fail "CleanMap rebuilt points=0; clean map is empty"
fi

# 17. 检查 PayloadTrackInfoCore source=CORE_BOX
if grep -q "\[PayloadTrackInfoCore\].*source=CORE_BOX" "${LOG_FILE}"; then
  pass_msg "PayloadTrackInfoCore source=CORE_BOX exists"
else
  fail "missing [PayloadTrackInfoCore] source=CORE_BOX; old bbox may still be used"
fi

# 18. 检查 CargoCommit 删除是否生效
REMOVED_GT0=$(grep "\[CargoCommit\].*source=objects_base" "${LOG_FILE}" | grep -c "removed=[1-9]" || true)
if [ "${REMOVED_GT0}" -gt 0 ]; then
  pass_msg "CargoCommit removal working: ${REMOVED_GT0} frames with removed>0"
else
  fail "CargoCommit never removed any points; active_boxes may not work"
fi

# 19. 检查 source=objects_base
if grep -q "\[CargoCommit\].*source=objects_base" "${LOG_FILE}"; then
  pass_msg "CargoCommit source=objects_base (correct coordinate system)"
else
  fail "CargoCommit missing source=objects_base"
fi

echo "========== [VALIDATION] PASS: cargo pipeline basic checks passed =========="
