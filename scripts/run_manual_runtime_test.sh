#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

wait_for_runtime_ready() {
  local timeout_sec="${1:-60}"
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

if ! docker ps --format '{{.Names}}' | grep -qx "${STACK_NAME}-hlc"; then
  "${ROOT_DIR}/scripts/run_stack.sh"
fi

wait_for_runtime_ready 60

printf 'Vehicle is already spawned by SITL. Use keyboard control to issue commands.\n'
printf 'Controls: t=takeoff l=land g=gohome w=forward a=left s=back d=right r=up f=down space=stop h=help q=quit\n'
printf 'This runtime test is headless. No GUI or simulator window is launched by the current stack.\n'

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
  exec ros2 run manual_runtime_test keyboard_teleop --ros-args --params-file /workspace/config/manual_runtime_test/keyboard.yaml
'
