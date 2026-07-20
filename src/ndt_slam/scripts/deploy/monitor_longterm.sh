#!/usr/bin/env bash
set -euo pipefail
echo "DEPRECATED: monitor_longterm.sh now follows the unified append-safe monitor." >&2
[[ "${1:-}" =~ ^[0-9]+$ ]] && shift || true
exec rosrun ndt_slam server_monitorctl.sh follow "$@"
