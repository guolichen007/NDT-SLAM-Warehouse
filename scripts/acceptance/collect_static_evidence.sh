#!/usr/bin/env bash
set -euo pipefail

map_root="${1:?usage: collect_static_evidence.sh MAP_ROOT OUTPUT_DIR}"
output_dir="${2:?usage: collect_static_evidence.sh MAP_ROOT OUTPUT_DIR}"
mkdir -p "$output_dir"

find "$map_root" -maxdepth 2 -type f \
  \( -name 'static_evidence_manifest*.json' \
     -o -name 'static_evidence_index_v2_*.csv' \
     -o -name 'runtime_status.json' \) \
  -print -exec cp --parents '{}' "$output_dir" \;

find "$map_root" -maxdepth 2 -type f -name '*.tmp' -print \
  > "$output_dir/pending_tmp_files.txt"
find "$map_root/tiles_objects" -maxdepth 1 -type f -name '*.pcd' -printf '%T@ %p\n' \
  2>/dev/null | sort -n > "$output_dir/object_tile_timeline.txt" || true
