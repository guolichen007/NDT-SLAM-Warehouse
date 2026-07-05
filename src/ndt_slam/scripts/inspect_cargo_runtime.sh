#!/usr/bin/env bash
set -euo pipefail

echo "========== Topic List =========="
rostopic list | grep -E "display_map|objects_map|objects_clean|ground_map|cargo_.*bbox|cargo_.*cloud|map$" || true

echo ""
echo "========== Map Topic Hz =========="
for t in /display_map /objects_map /objects_clean /ground_map /map; do
  echo "--- $t ---"
  timeout 6 rostopic hz "$t" 2>&1 | head -3 || true
done

echo ""
echo "========== Cargo Marker Type =========="
for t in /cargo_core_bbox_marker /cargo_remove_bbox_marker /cargo_stable_bbox_marker /cargo_forbidden_zone_marker; do
  echo "--- $t ---"
  timeout 3 rostopic echo -n 1 "$t" 2>&1 | grep -E "type:|ns:|scale\.x:|color\.a:" | head -5 || true
done

echo ""
echo "========== Cargo Clouds Hz =========="
for t in /cargo_core_points_cloud /cargo_removed_cloud /suspended_payload_cloud; do
  echo "--- $t ---"
  timeout 6 rostopic hz "$t" 2>&1 | head -3 || true
done

echo ""
echo "========== Latest PipelineSummary =========="
timeout 3 rostopic echo -n 1 /rosout 2>&1 | grep -A2 "PipelineSummary" | head -5 || true
