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

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  "${ROOT_DIR}/scripts/build_ws.sh"
fi

if ! docker ps --format '{{.Names}}' | grep -qx "${STACK_NAME}-hlc"; then
  "${ROOT_DIR}/scripts/run_stack.sh"
fi

wait_for_runtime_ready 60

docker exec "${STACK_NAME}-decision-dev" bash -lc '
  set -eo pipefail
  set +u
  source /opt/ros/humble/setup.bash
  if [ -f /workspace/ws/install/setup.bash ]; then
    source /workspace/ws/install/setup.bash
  else
    source /opt/ros2_ws/install/setup.bash
  fi
  set -u
  timeout 20 ros2 launch manual_runtime_test manual_runtime_test.launch.py config:=/workspace/config/manual_runtime_test/scripted_smoke_test.yaml
' >/tmp/manual-runtime-test-smoke.log 2>&1 &
TELEOP_PID=$!

docker exec "${STACK_NAME}-ros-alate" bash -lc '
  source /opt/ros/humble/setup.bash
  source /opt/ros2_ws/install/setup.bash
  export ROS2CLI_DISABLE_DAEMON=1
  timeout 10 ros2 topic echo --once /alate_input_velocity
' >/dev/null

wait "${TELEOP_PID}"

printf 'Manual runtime test validation complete.\n'
