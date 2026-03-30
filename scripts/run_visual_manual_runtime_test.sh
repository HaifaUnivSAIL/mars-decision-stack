#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

if [ "$#" -gt 1 ]; then
  printf 'Usage: %s [--calib|--experiment]\n' "$0" >&2
  exit 1
fi

visual_mode="${VISUAL_MODE:-experiment}"
case "${1:-}" in
  "")
    ;;
  --calib)
    visual_mode="calib"
    ;;
  --experiment)
    visual_mode="experiment"
    ;;
  *)
    printf 'Unsupported visual mode flag: %s\n' "${1}" >&2
    printf 'Usage: %s [--calib|--experiment]\n' "$0" >&2
    exit 1
    ;;
esac

visual_mode_file="${LOG_DIR}/runtime/visual.mode"
visual_current_run_file="${LOG_DIR}/runtime/visual.current_run_dir"

current_visual_mode() {
  if [ -f "${visual_mode_file}" ]; then
    cat "${visual_mode_file}"
  fi
}

current_run_dir() {
  if [ -f "${visual_current_run_file}" ]; then
    cat "${visual_current_run_file}"
  fi
}

wait_for_runtime_ready() {
  local timeout_sec="${1:-90}"
  local start_sec
  local hlc_logs
  local mc_logs
  start_sec="$(date +%s)"

  while true; do
    hlc_logs="$(docker logs "${STACK_NAME}-hlc" 2>&1 || true)"
    mc_logs="$(docker logs "${STACK_NAME}-mc" 2>&1 || true)"

    if grep -q 'HLC entering state: Ready' <<<"${hlc_logs}" && \
      grep -q 'MissionControl entering state: Standby' <<<"${mc_logs}"; then
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
  local requested_arg=""

  if [ "${visual_mode}" = "calib" ]; then
    requested_arg="--calib"
  fi

  printf 'Refreshing the visual stack in %s mode so the latest camera/config values are applied.\n' "${visual_mode}"
  RUN_VISUAL_STACK_QUIET_HINTS=1 "${ROOT_DIR}/scripts/run_visual_stack.sh" ${requested_arg:+"${requested_arg}"}
}

ensure_visual_stack

wait_for_runtime_ready 90

run_dir="$(current_run_dir)"
if [ -z "${run_dir}" ]; then
  printf 'Missing current visual run directory.\n' >&2
  exit 1
fi

printf 'Vehicle is already spawned by Gazebo-backed SITL.\n'
printf 'Controls: t=takeoff l=land g=gohome w=forward a=left s=back d=right r=up f=down space=stop h=help q=quit\n'
printf 'Gazebo should display the chase and deployed camera views while the keyboard test is running.\n'
printf 'Visual mode: %s\n' "${visual_mode}"
printf 'Run directory: %s\n' "${run_dir}"

docker exec -it "${STACK_NAME}-decision-dev" bash -lc '
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
  exec ros2 run manual_runtime_test keyboard_teleop --ros-args --params-file /workspace/config/manual_runtime_test/keyboard.yaml
'
