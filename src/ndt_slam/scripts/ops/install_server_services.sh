#!/usr/bin/env bash
set -euo pipefail

workspace=""
service_user=""
data_root=""
yes=false
enable=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace) workspace="$2"; shift 2 ;;
    --user) service_user="$2"; shift 2 ;;
    --data-root) data_root="$2"; shift 2 ;;
    --yes) yes=true; shift ;;
    --enable) enable=true; shift ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ -n "$workspace" && -n "$service_user" && -n "$data_root" ]] || {
  echo "Usage: install_server_services.sh --workspace PATH --user USER --data-root PATH [--yes] [--enable]" >&2
  exit 2
}
workspace="$(readlink -f "$workspace")"
mkdir -p "$data_root"
data_root="$(readlink -f "$data_root")"
[[ -d "$workspace" ]] || { echo "workspace missing" >&2; exit 2; }
id "$service_user" >/dev/null 2>&1 || { echo "user missing: $service_user" >&2; exit 2; }
[[ "$EUID" -eq 0 ]] || { echo "run this installer as root" >&2; exit 2; }
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
template_dir="$script_dir"
if [[ ! -f "$template_dir/ndt-slam.service.in" ]]; then
  template_dir="$(rospack find ndt_slam)/systemd"
fi
[[ -f "$template_dir/ndt-slam.service.in" &&
   -f "$template_dir/ndt-slam-monitor.service.in" ]] || {
  echo "service templates not found" >&2
  exit 3
}
env_dir=/etc/ndt-slam
env_file="$env_dir/ndt-slam.env"
cat <<EOF
NDT-SLAM service configuration
  user:       $service_user
  workspace:  $workspace
  data root:  $data_root
  env file:   $env_file
EOF
if ! $yes; then
  read -r -p "Install these service files? [y/N] " answer
  [[ "$answer" =~ ^[Yy]$ ]] || exit 1
fi

install -d -m 0755 "$env_dir" "$data_root" "$workspace/logs" "$workspace/server_runs"
cat > "$env_file" <<EOF
NDT_SLAM_WORKSPACE=$workspace
NDT_SLAM_DATA_ROOT=$data_root
ROS_LOG_DIR=$workspace/logs
EOF
chmod 0644 "$env_file"
chown -R "$service_user":"$service_user" "$data_root" "$workspace/logs" "$workspace/server_runs"

escape_sed() { printf '%s' "$1" | sed 's/[&|]/\\&/g'; }
for unit in ndt-slam.service ndt-slam-monitor.service; do
  sed -e "s|@USER@|$(escape_sed "$service_user")|g" \
      -e "s|@WORKSPACE@|$(escape_sed "$workspace")|g" \
      -e "s|@DATA_ROOT@|$(escape_sed "$data_root")|g" \
      -e "s|@ENV_FILE@|$(escape_sed "$env_file")|g" \
      "$template_dir/$unit.in" > "/etc/systemd/system/$unit"
  chmod 0644 "/etc/systemd/system/$unit"
done

fail_rendered_unit() {
  echo "rendered unit validation failed: $1" >&2
  exit 4
}
require_unit_line() {
  local unit_file="$1"
  local expected="$2"
  grep -Fqx "$expected" "$unit_file" ||
    fail_rendered_unit "$(basename "$unit_file") missing line: $expected"
}
require_unit_text() {
  local unit_file="$1"
  local expected="$2"
  grep -Fq "$expected" "$unit_file" ||
    fail_rendered_unit "$(basename "$unit_file") missing text: $expected"
}

slam_unit=/etc/systemd/system/ndt-slam.service
monitor_unit=/etc/systemd/system/ndt-slam-monitor.service
for unit_file in "$slam_unit" "$monitor_unit"; do
  if grep -Eq '@[A-Z_][A-Z0-9_]*@' "$unit_file"; then
    fail_rendered_unit "$(basename "$unit_file") contains an unresolved placeholder"
  fi
  require_unit_line "$unit_file" "User=$service_user"
  require_unit_line "$unit_file" "WorkingDirectory=$workspace"
  require_unit_line "$unit_file" "EnvironmentFile=$env_file"
done

require_unit_line "$slam_unit" 'StartLimitIntervalSec=300'
require_unit_line "$slam_unit" 'StartLimitBurst=5'
require_unit_line "$slam_unit" 'Restart=always'
require_unit_line "$slam_unit" 'RestartSec=5'
require_unit_text "$slam_unit" '/usr/bin/flock --no-fork --exclusive --nonblock'
require_unit_text "$slam_unit" "$data_root/.ndt-slam.lock"
require_unit_text "$slam_unit" \
  'exec roslaunch ndt_slam warehouse_live_longterm_mapping.launch'
require_unit_text "$slam_unit" 'use_ndt_recovery_watchdog:=true'
if grep -Fq 'Restart=on-failure' "$slam_unit"; then
  fail_rendered_unit "ndt-slam.service contains obsolete Restart=on-failure"
fi

require_unit_line "$monitor_unit" 'After=ndt-slam.service'
require_unit_line "$monitor_unit" 'Wants=ndt-slam.service'
require_unit_line "$monitor_unit" 'Restart=always'
require_unit_line "$monitor_unit" 'RestartSec=10'
require_unit_line "$monitor_unit" 'TimeoutStopSec=30'
require_unit_text "$monitor_unit" \
  'exec rosrun ndt_slam server_runtime_monitor.py'
require_unit_text "$monitor_unit" \
  '--workspace "$NDT_SLAM_WORKSPACE"'
require_unit_text "$monitor_unit" '--persistent-root "$NDT_SLAM_DATA_ROOT"'
require_unit_text "$monitor_unit" \
  '--config "$(rospack find ndt_slam)/config/server_monitor.yaml"'
require_unit_text "$monitor_unit" \
  '--lock-file "$NDT_SLAM_WORKSPACE/server_runs/.service-monitor.lock"'

systemctl daemon-reload

fail_effective_unit() {
  local unit="$1"
  local reason="$2"
  echo "effective unit validation failed: $unit: $reason" >&2
  systemctl cat --no-pager "$unit" >&2 || true
  exit 5
}
effective_property() {
  local unit="$1"
  local property="$2"
  local value
  if ! value="$(systemctl show "$unit" --property="$property" --value)"; then
    fail_effective_unit "$unit" "cannot read property $property"
  fi
  printf '%s' "$value"
}
require_effective_exact() {
  local unit="$1"
  local property="$2"
  local expected="$3"
  local actual
  actual="$(effective_property "$unit" "$property")"
  [[ "$actual" == "$expected" ]] ||
    fail_effective_unit "$unit" \
      "$property expected '$expected' but is '$actual' (check drop-ins)"
}
require_effective_text() {
  local unit="$1"
  local property="$2"
  local expected="$3"
  local actual
  actual="$(effective_property "$unit" "$property")"
  [[ "$actual" == *"$expected"* ]] ||
    fail_effective_unit "$unit" \
      "$property does not contain '$expected' (check drop-ins)"
}

require_effective_exact ndt-slam.service Restart always
require_effective_exact ndt-slam.service RestartUSec 5s
require_effective_exact ndt-slam.service StartLimitIntervalUSec 5min
require_effective_exact ndt-slam.service StartLimitBurst 5
require_effective_exact ndt-slam.service User "$service_user"
require_effective_exact ndt-slam.service WorkingDirectory "$workspace"
require_effective_text ndt-slam.service EnvironmentFiles "$env_file"
require_effective_text ndt-slam.service ExecStart '/usr/bin/flock'
require_effective_text ndt-slam.service ExecStart \
  "$data_root/.ndt-slam.lock"
require_effective_text ndt-slam.service ExecStart \
  'warehouse_live_longterm_mapping.launch'
require_effective_text ndt-slam.service ExecStart \
  'use_ndt_recovery_watchdog:=true'

require_effective_exact ndt-slam-monitor.service Restart always
require_effective_exact ndt-slam-monitor.service RestartUSec 10s
require_effective_exact ndt-slam-monitor.service TimeoutStopUSec 30s
require_effective_exact ndt-slam-monitor.service User "$service_user"
require_effective_exact ndt-slam-monitor.service WorkingDirectory "$workspace"
require_effective_text ndt-slam-monitor.service EnvironmentFiles "$env_file"
require_effective_text ndt-slam-monitor.service ExecStart \
  'server_runtime_monitor.py'
require_effective_text ndt-slam-monitor.service ExecStart \
  '--workspace'
require_effective_text ndt-slam-monitor.service ExecStart \
  '--persistent-root'
require_effective_text ndt-slam-monitor.service ExecStart \
  'server_monitor.yaml'
require_effective_text ndt-slam-monitor.service ExecStart \
  '.service-monitor.lock'

if $enable; then
  systemctl enable ndt-slam.service ndt-slam-monitor.service
fi
echo "SERVICES_INSTALLED start with: systemctl start ndt-slam.service ndt-slam-monitor.service"
