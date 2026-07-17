#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec "$repo_root/src/ndt_slam/scripts/ops/run_server_validation.sh" "$@"
