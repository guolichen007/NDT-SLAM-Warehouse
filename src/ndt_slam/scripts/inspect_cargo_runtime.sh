#!/usr/bin/env bash
set -euo pipefail
echo "DEPRECATED: use 'rosrun ndt_slam server_monitorctl.sh status'" >&2
exec rosrun ndt_slam server_monitorctl.sh status "$@"
