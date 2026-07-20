#!/usr/bin/env bash
set -euo pipefail

[[ $# -ge 1 ]] || { echo "Usage: collect_server_artifacts.sh RUN_DIR [--include-bag]" >&2; exit 2; }
run_dir="$(readlink -f "$1")"
shift
include_bag=false
[[ "${1:-}" != "--include-bag" ]] || include_bag=true
[[ -d "$run_dir" ]] || { echo "run directory missing: $run_dir" >&2; exit 2; }
parent="$(dirname "$run_dir")"
name="$(basename "$run_dir")"
archive="$parent/$name.tar.zst"
checksum="$archive.sha256"
exclude_args=(--exclude="$name/monitor.lock" --exclude="$name/monitor.pid")
$include_bag || exclude_args+=(--exclude="$name/bags/*.bag")

if tar --help 2>/dev/null | grep -q -- '--zstd'; then
  tar --zstd -C "$parent" "${exclude_args[@]}" -cf "$archive" "$name"
else
  command -v zstd >/dev/null 2>&1 || { echo "zstd is required" >&2; exit 3; }
  tar -C "$parent" "${exclude_args[@]}" -cf - "$name" | zstd -T0 -q -o "$archive"
fi
sha256sum "$archive" > "$checksum"
echo "artifact=$archive"
echo "checksum=$checksum"
