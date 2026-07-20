#!/usr/bin/env bash
set -euo pipefail
echo "DEPRECATED: deploy_slam.sh delegates to the explicit service installer." >&2
echo "Required: --workspace PATH --user USER --data-root PATH" >&2
exec rosrun ndt_slam install_server_services.sh "$@"
