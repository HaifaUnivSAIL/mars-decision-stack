#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

visual_mode="${VISUAL_MODE:-experiment}"
fleet_args=()
drone_id=""
profile_enabled=0

while [ "$#" -gt 0 ]; do
  case "$1" in
    --calib)
      visual_mode="calib"
      ;;
    --experiment)
      visual_mode="experiment"
      ;;
    --fleet)
      shift
      if [ "$#" -eq 0 ]; then
        printf 'Missing value for --fleet\n' >&2
        exit 1
      fi
      fleet_args+=(--fleet "$1")
      ;;
    --drone)
      shift
      if [ "$#" -eq 0 ]; then
        printf 'Missing value for --drone\n' >&2
        exit 1
      fi
      drone_id="$1"
      ;;
    --profile)
      profile_enabled=1
      ;;
    *)
      printf 'Unsupported argument: %s\n' "$1" >&2
      printf 'Usage: %s [--experiment|--calib] [--fleet PATH] [--drone ID] [--profile]\n' "$0" >&2
      exit 1
      ;;
  esac
  shift
done

visual_mode_file="${LOG_DIR}/runtime/visual.mode"
visual_current_run_file="${LOG_DIR}/runtime/visual.current_run_dir"

current_run_dir() {
  if [ -f "${visual_current_run_file}" ]; then
    cat "${visual_current_run_file}"
  fi
}

resolve_drone_id() {
  local manifest_path="$1"
  python3 - "$manifest_path" "$drone_id" <<'PY'
import json
import sys
from pathlib import Path
manifest = json.loads(Path(sys.argv[1]).read_text())
requested = sys.argv[2].strip()
if manifest.get('fleet'):
    drone_ids = [drone['id'] for drone in manifest['drones']]
    if requested:
        if requested not in drone_ids:
            raise SystemExit(f'Unknown drone id {requested!r}. Available: {drone_ids}')
        print(requested)
    else:
        print(manifest.get('active_drone_id') or drone_ids[0])
else:
    print('')
PY
}

wait_for_runtime_ready() {
  local manifest_path="$1"
  local selected_drone_id="$2"
  local timeout_sec="${3:-120}"
  local start_sec
  start_sec="$(date +%s)"

  if python3 - "$manifest_path" <<'PY' >/dev/null
import json, sys
from pathlib import Path
manifest = json.loads(Path(sys.argv[1]).read_text())
raise SystemExit(0 if manifest.get('fleet') else 1)
PY
  then
    while true; do
      local hlc_logs mc_logs
      hlc_logs="$(docker logs "${STACK_NAME}-${selected_drone_id}-hlc" 2>&1 || true)"
      mc_logs="$(docker logs "${STACK_NAME}-${selected_drone_id}-mc" 2>&1 || true)"
      if grep -q 'HLC entering state: Ready' <<<"${hlc_logs}" && grep -q 'MissionControl entering state: Standby' <<<"${mc_logs}"; then
        return 0
      fi
      if [ $(( $(date +%s) - start_sec )) -ge "${timeout_sec}" ]; then
        printf 'Timed out waiting for HLC/MC readiness of %s.\n' "${selected_drone_id}" >&2
        return 1
      fi
      sleep 1
    done
  fi

  while true; do
    local hlc_logs mc_logs
    hlc_logs="$(docker logs "${STACK_NAME}-hlc" 2>&1 || true)"
    mc_logs="$(docker logs "${STACK_NAME}-mc" 2>&1 || true)"
    if grep -q 'HLC entering state: Ready' <<<"${hlc_logs}" && grep -q 'MissionControl entering state: Standby' <<<"${mc_logs}"; then
      return 0
    fi
    if [ $(( $(date +%s) - start_sec )) -ge "${timeout_sec}" ]; then
      printf 'Timed out waiting for HLC/MC readiness.\n' >&2
      return 1
    fi
    sleep 1
  done
}

ensure_visual_stack() {
  local requested_args=()
  if [ "${visual_mode}" = "calib" ]; then
    requested_args+=(--calib)
  fi
  if [ "${profile_enabled}" = '1' ]; then
    requested_args+=(--profile)
  fi
  requested_args+=("${fleet_args[@]}")
  printf 'Refreshing the visual stack in %s mode so the latest config values are applied.\n' "${visual_mode}"
  RUN_VISUAL_STACK_QUIET_HINTS=1 "${ROOT_DIR}/scripts/run_visual_stack.sh" "${requested_args[@]}"
}

ensure_visual_stack

run_dir="$(current_run_dir)"
if [ -z "${run_dir}" ] || [ ! -d "${run_dir}" ]; then
  printf 'Missing current visual run directory.\n' >&2
  exit 1
fi

manifest_path="${run_dir}/visual_assets/manifest.json"
if [ ! -f "${manifest_path}" ]; then
  printf 'Missing runtime manifest: %s\n' "${manifest_path}" >&2
  exit 1
fi

selected_drone_id="$(resolve_drone_id "${manifest_path}")"
wait_for_runtime_ready "${manifest_path}" "${selected_drone_id}" 120

teleop_params_host="${run_dir}/ros_params/manual_runtime_test.keyboard.yaml"
teleop_params_mount="/workspace/run/ros_params/manual_runtime_test.keyboard.yaml"
mkdir -p "$(dirname "${teleop_params_host}")"
if [ -n "${selected_drone_id}" ]; then
  python3 "${ROOT_DIR}/scripts/render_namespaced_ros_params.py" \
    --source "${CONFIG_DIR}/manual_runtime_test/keyboard.yaml" \
    --node-name manual_runtime_test \
    --namespace "/${selected_drone_id}" \
    --output "${teleop_params_host}"
else
  cp "${CONFIG_DIR}/manual_runtime_test/keyboard.yaml" "${teleop_params_host}"
fi

printf 'Vehicle control is routed through Gazebo-backed SITL.\n'
printf 'Controls: t=takeoff l=land g=gohome w=forward a=left s=back d=right r=up f=down space=stop h=help q=quit\n'
if [ -n "${selected_drone_id}" ]; then
  printf 'Gazebo should display the swarm world and the active chase/deployed camera views.\n'
else
  printf 'Gazebo should display the chase and deployed camera views for the single-drone runtime.\n'
fi
printf 'Visual mode: %s\n' "${visual_mode}"
printf 'Run directory: %s\n' "${run_dir}"
if [ -n "${selected_drone_id}" ]; then
  printf 'Selected drone: %s\n' "${selected_drone_id}"
fi

ns_args=()
if [ -n "${selected_drone_id}" ]; then
  ns_args+=( -r "__ns:=/${selected_drone_id}" )
fi

teleop_cmd=(
  ros2 run manual_runtime_test keyboard_teleop
  --ros-args
  --params-file "${teleop_params_mount}"
)
if [ "${#ns_args[@]}" -gt 0 ]; then
  teleop_cmd+=( "${ns_args[@]}" )
fi

docker exec -it "${STACK_NAME}-decision-dev" bash -lc "
  set -eo pipefail
  set +u
  source /opt/ros/humble/setup.bash
  if [ -f /workspace/ws/install/setup.bash ]; then
    source /workspace/ws/install/setup.bash
  else
    source /opt/ros2_ws/install/setup.bash
  fi
  set -u
  mkdir -p /workspace/run/logs
  exec > >(tee -a /workspace/run/logs/manual-runtime-test.log) 2>&1
  exec ${teleop_cmd[*]}
"
