#!/usr/bin/env bash
set -euo pipefail

RVIZ="/home/ydkj/NDT-slam-ws/src/ndt_slam/launch/rviz.rviz"

fail() {
  echo "[VALIDATION] FAIL: $1"
  exit 1
}

pass_msg() {
  echo "[VALIDATION] PASS: $1"
}

if [ ! -f "$RVIZ" ]; then
  fail "RViz config not found: $RVIZ"
fi

# 必须包含 cargo_core_bbox_marker
grep -q "/cargo_core_bbox_marker" "$RVIZ" || fail "missing /cargo_core_bbox_marker"
pass_msg "/cargo_core_bbox_marker exists"

# 必须包含 objects_clean
grep -q "/objects_clean" "$RVIZ" || fail "missing /objects_clean"
pass_msg "/objects_clean exists"

# 必须包含 path
grep -q "/path" "$RVIZ" || fail "missing /path"
pass_msg "/path exists"

# 禁止包含的 topic
for topic in \
  "/cargo_raw_bbox_marker" \
  "/cargo_stable_bbox_marker" \
  "/cargo_remove_bbox_marker" \
  "/cargo_unknown_bbox_marker" \
  "/cargo_forbidden_zone_marker" \
  "/cargo_forbidden_overlay" \
  "/suspended_payload_candidate_cloud" \
  "/suspended_payload_cloud" \
  "/cargo_dynamic_removed_cloud" \
  "/cargo_removed_cloud" \
  "/cargo_track_status_marker"
do
  if grep -q "$topic" "$RVIZ"; then
    fail "RViz still contains disabled topic: $topic"
  fi
done

pass_msg "no disabled topics in RViz"

echo ""
echo "[VALIDATION] PASS: RViz core-box-only display"
