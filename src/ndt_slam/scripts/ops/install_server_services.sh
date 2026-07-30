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

grep -Fqx 'RestartSec=5' /etc/systemd/system/ndt-slam.service || {
  echo "installed ndt-slam.service has unexpected RestartSec" >&2
  exit 4
}
grep -Fqx 'Restart=always' /etc/systemd/system/ndt-slam.service || {
  echo "installed ndt-slam.service cannot recover a clean roslaunch exit" >&2
  exit 4
}
grep -Fq 'use_ndt_recovery_watchdog:=true' \
  /etc/systemd/system/ndt-slam.service || {
  echo "installed ndt-slam.service does not enforce the recovery watchdog" >&2
  exit 4
}
systemctl daemon-reload
if $enable; then
  systemctl enable ndt-slam.service ndt-slam-monitor.service
fi
echo "SERVICES_INSTALLED start with: systemctl start ndt-slam.service ndt-slam-monitor.service"
