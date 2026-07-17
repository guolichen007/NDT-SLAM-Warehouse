#!/usr/bin/env bash
set -euo pipefail

workspace="${NDT_SLAM_WORKSPACE:-$HOME/NDT-slam-ws}"
map_root=""
expected_sha=""
phase="live"
minimum_disk_gb=20
while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace) workspace="$2"; shift 2 ;;
    --map-root|--persistent-root) map_root="$2"; shift 2 ;;
    --expected-sha) expected_sha="$2"; shift 2 ;;
    --phase) phase="$2"; shift 2 ;;
    --minimum-disk-gb) minimum_disk_gb="$2"; shift 2 ;;
    -h|--help)
      echo "Usage: server_preflight.sh --expected-sha SHA [--workspace PATH] [--phase prepare|live]"
      exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ "$phase" == "prepare" || "$phase" == "live" ]] || { echo "invalid phase" >&2; exit 2; }
workspace="$(readlink -f "$workspace")"
map_root="${map_root:-$workspace/maps/live/current}"
mkdir -p "$map_root"

failures=0
pass() { echo "PASS $*"; }
fail() { echo "FAIL $*" >&2; failures=$((failures + 1)); }
info() { echo "INFO $*"; }

actual_sha="$(git -C "$workspace" rev-parse HEAD 2>/dev/null || true)"
[[ -n "$expected_sha" ]] || fail "--expected-sha is mandatory"
[[ -z "$expected_sha" || "$actual_sha" == "$expected_sha" ]] &&
  pass "exact_sha=$actual_sha" || fail "exact_sha expected=$expected_sha actual=$actual_sha"

branch="$(git -C "$workspace" symbolic-ref --short -q HEAD || echo detached)"
info "branch=$branch"
if [[ -z "$(git -C "$workspace" status --porcelain --untracked-files=normal)" ]]; then
  pass "worktree_clean"
else
  fail "worktree_dirty"
  git -C "$workspace" status --short >&2
fi

submodule_status="$(git -C "$workspace" submodule status --recursive || true)"
if grep -Eq '^[+-U]' <<<"$submodule_status"; then
  fail "submodule_not_exact"
else
  pass "submodule_exact"
fi
printf '%s\n' "$submodule_status"

[[ -w "$map_root" ]] && pass "persistent_root_writable=$map_root" || fail "persistent_root_not_writable=$map_root"
available_kb="$(df -Pk "$map_root" | awk 'NR==2 {print $4}')"
required_kb=$((minimum_disk_gb * 1024 * 1024))
[[ "$available_kb" -ge "$required_kb" ]] &&
  pass "disk_free_kb=$available_kb" || fail "disk_low free_kb=$available_kb required_kb=$required_kb"
info "run_user=$(id -un) uid=$(id -u)"

if command -v timedatectl >/dev/null 2>&1; then
  synchronized="$(timedatectl show -p NTPSynchronized --value 2>/dev/null || echo unknown)"
  [[ "$synchronized" == "yes" ]] && pass "time_synchronized" || fail "time_not_synchronized=$synchronized"
else
  info "time_sync_check=unavailable"
fi

manifest="$map_root/static_evidence_manifest.json"
last_good="$map_root/static_evidence_manifest.last_good.json"
suspended="$map_root/static_evidence_manifest.suspended"
tmp_count="$(find "$map_root" -maxdepth 1 -type f -name '*.tmp' | wc -l)"
[[ "$tmp_count" -eq 0 ]] && pass "no_tmp_files" || fail "tmp_files=$tmp_count"
if [[ -f "$suspended" ]]; then
  info "static_manifest=SUSPENDED_FAIL_SAFE"
elif [[ -f "$manifest" ]]; then
  pass "static_manifest=ACTIVE"
elif [[ -f "$last_good" ]]; then
  info "static_manifest=LAST_GOOD_INACTIVE"
else
  info "static_manifest=FIRST_RUN"
fi

if pgrep -x ndt_slam_node >/dev/null 2>&1; then
  process_count="$(pgrep -x ndt_slam_node | wc -l)"
  [[ "$process_count" -eq 1 ]] && pass "single_slam_instance" || fail "slam_instances=$process_count"
elif [[ "$phase" == "live" ]]; then
  fail "slam_process_missing"
else
  pass "slam_process_absent_before_start"
fi

if ! rosnode list >/dev/null 2>&1; then
  [[ "$phase" == "prepare" ]] && info "ros_master=NOT_RUNNING_PREPARE" || fail "ros_master_unreachable"
else
  pass "ros_master_reachable"
  use_sim_time="$(rosparam get /use_sim_time 2>/dev/null || echo missing)"
  [[ "$use_sim_time" == "false" || "$use_sim_time" == "False" ]] &&
    pass "use_sim_time=false" || fail "use_sim_time=$use_sim_time"
  if [[ "$phase" == "live" ]]; then
    persistent="$(rosparam get /ndt_slam_node/persistent_map_enabled 2>/dev/null || echo missing)"
    [[ "$persistent" == "true" || "$persistent" == "True" ]] &&
      pass "persistent_map_enabled=true" || fail "persistent_map_enabled=$persistent"
    nodes="$(rosnode list)"
    for node in /ndt_slam_node /cargo_alarm_heartbeat; do
      grep -Fxq "$node" <<<"$nodes" && pass "node=$node" || fail "node_missing=$node"
    done
    topics="$(rostopic list)"
    for topic in /odom /cargo_avoidance/safety_status /cargo_avoidance/status_code; do
      grep -Fxq "$topic" <<<"$topics" && pass "topic=$topic" || fail "topic_missing=$topic"
    done
  fi
fi

if [[ "$failures" -ne 0 ]]; then
  echo "PREFLIGHT=FAIL failures=$failures" >&2
  exit 1
fi
echo "PREFLIGHT=PASS phase=$phase sha=$actual_sha"
