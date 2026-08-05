#!/usr/bin/env bash
set -uo pipefail

usage() {
  echo "Usage: $0 --workspace PATH [--use-rviz true|false] [--data-root PATH] [--config PATH]"
}

WORKSPACE=""
USE_RVIZ="true"
DATA_ROOT=""
CONFIG_FILE=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace)
      WORKSPACE="${2:-}"
      shift 2
      ;;
    --use-rviz)
      USE_RVIZ="${2:-}"
      shift 2
      ;;
    --data-root)
      DATA_ROOT="${2:-}"
      shift 2
      ;;
    --config)
      CONFIG_FILE="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "$WORKSPACE" ]]; then
  echo "--workspace is required" >&2
  exit 2
fi
WORKSPACE="$(cd "$WORKSPACE" 2>/dev/null && pwd -P)" || {
  echo "Workspace does not exist: $WORKSPACE" >&2
  exit 2
}
if [[ "$USE_RVIZ" != "true" && "$USE_RVIZ" != "false" ]]; then
  echo "--use-rviz must be true or false" >&2
  exit 2
fi
if [[ -z "$DATA_ROOT" ]]; then
  DATA_ROOT="$WORKSPACE/maps/live/current"
fi
if [[ ! -f "$WORKSPACE/devel/setup.bash" ]]; then
  echo "Missing workspace environment: $WORKSPACE/devel/setup.bash" >&2
  exit 3
fi

mkdir -p "$DATA_ROOT/recovery_watchdog"
LOCK_FILE="$DATA_ROOT/.ndt-slam-supervisor.lock"
REQUEST_FILE="$DATA_ROOT/recovery_watchdog/restart_request.json"
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  echo "Another NDT SLAM supervisor already holds $LOCK_FILE" >&2
  exit 4
fi

# shellcheck disable=SC1090
source "$WORKSPACE/devel/setup.bash"
export NDT_SLAM_DATA_ROOT="$DATA_ROOT"

USER_STOP=0
CHILD_PID=""
on_signal() {
  USER_STOP=1
  if [[ -n "$CHILD_PID" ]] && kill -0 "$CHILD_PID" 2>/dev/null; then
    kill -INT -- "-$CHILD_PID" 2>/dev/null || kill -INT "$CHILD_PID" 2>/dev/null
  fi
}
trap on_signal INT TERM

declare -a RESTART_TIMES=()
GENERATION=0
while true; do
  GENERATION=$((GENERATION + 1))
  LAUNCH_STARTED_AT="$(date +%s)"
  RUN_ID="${LAUNCH_STARTED_AT}-$$-${GENERATION}"
  export NDT_SLAM_SUPERVISOR_RUN_ID="$RUN_ID"

  echo "[NDT supervisor] generation=$GENERATION run_id=$RUN_ID"
  echo "[NDT supervisor] data_root=$DATA_ROOT use_rviz=$USE_RVIZ"
  LAUNCH_ARGS=(
    ndt_slam warehouse_live_longterm_mapping.launch
    "use_rviz:=$USE_RVIZ"
  )
  if [[ -n "$CONFIG_FILE" ]]; then
    LAUNCH_ARGS+=("config_file:=$CONFIG_FILE")
  fi

  setsid roslaunch "${LAUNCH_ARGS[@]}" &
  CHILD_PID=$!
  wait "$CHILD_PID"
  LAUNCH_EXIT=$?
  CHILD_PID=""

  if [[ "$USER_STOP" -eq 1 ]]; then
    echo "[NDT supervisor] user stop received; not restarting"
    exit 0
  fi

  RESTART_REASON=""
  if [[ -f "$REQUEST_FILE" ]]; then
    REQUEST_VALUES="$(python3 -c 'import json,sys; p=json.load(open(sys.argv[1], encoding="utf-8")); print(str(p.get("supervisor_run_id",""))); print(str(p.get("action",""))); print(str(p.get("wall_time",0))); print(str(p.get("reason","unknown")))' "$REQUEST_FILE" 2>/dev/null)"
    REQUEST_RUN_ID="$(printf '%s\n' "$REQUEST_VALUES" | sed -n '1p')"
    REQUEST_ACTION="$(printf '%s\n' "$REQUEST_VALUES" | sed -n '2p')"
    REQUEST_TIME="$(printf '%s\n' "$REQUEST_VALUES" | sed -n '3p')"
    REQUEST_REASON="$(printf '%s\n' "$REQUEST_VALUES" | sed -n '4p')"
    REQUEST_CURRENT="$(python3 -c 'import sys; print(1 if float(sys.argv[1]) >= float(sys.argv[2]) else 0)' "${REQUEST_TIME:-0}" "$LAUNCH_STARTED_AT" 2>/dev/null || printf '0')"
    if [[ "$REQUEST_RUN_ID" == "$RUN_ID" &&
          "$REQUEST_ACTION" == "hard_restart" &&
          "$REQUEST_CURRENT" == "1" ]]; then
      RESTART_REASON="watchdog:${REQUEST_REASON:-unknown}"
      # Consume only the current generation's request. A later crash must not
      # inherit a marker from an already completed restart.
      rm -f -- "$REQUEST_FILE"
    else
      echo "[NDT supervisor] ignored stale/mismatched restart request"
    fi
  fi

  if [[ -z "$RESTART_REASON" ]]; then
    if [[ "$LAUNCH_EXIT" -eq 0 ]]; then
      echo "[NDT supervisor] roslaunch exited normally; not restarting"
      exit 0
    fi
    RESTART_REASON="roslaunch_crash_exit_${LAUNCH_EXIT}"
  fi

  NOW="$(date +%s)"
  RETAINED=()
  for RESTART_TIME in "${RESTART_TIMES[@]}"; do
    if (( NOW - RESTART_TIME <= 900 )); then
      RETAINED+=("$RESTART_TIME")
    fi
  done
  RESTART_TIMES=("${RETAINED[@]}")
  if (( ${#RESTART_TIMES[@]} >= 3 )); then
    echo "[NDT supervisor] restart budget exhausted (3/900s); leaving stack stopped"
    echo "[NDT supervisor] last_reason=$RESTART_REASON request=$REQUEST_FILE"
    exit 75
  fi
  RESTART_TIMES+=("$NOW")
  echo "[NDT supervisor] full-stack restart $(( ${#RESTART_TIMES[@]} ))/3 reason=$RESTART_REASON"
  echo "[NDT supervisor] waiting 5 seconds; next generation remains quarantined until strict verification"
  sleep 5
done
