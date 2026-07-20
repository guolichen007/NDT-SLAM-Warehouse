#!/usr/bin/env bash
set -euo pipefail
echo "DEPRECATED: slam_monitor.sh now follows the unified read-only monitor." >&2
[[ "${1:-}" =~ ^[0-9]+$ ]] && shift || true
exec rosrun ndt_slam server_monitorctl.sh follow "$@"
