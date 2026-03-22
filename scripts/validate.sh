#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

hlc_log="$(mktemp)"
mc_log="$(mktemp)"
topic_snapshot="$(mktemp)"
decision_log="$(mktemp)"
DECISION_EXEC_PID=""

cleanup() {
  if [ -n "${DECISION_EXEC_PID}" ]; then
    kill "${DECISION_EXEC_PID}" >/dev/null 2>&1 || true
    wait "${DECISION_EXEC_PID}" >/dev/null 2>&1 || true
  fi
  rm -f "${hlc_log}" "${mc_log}" "${topic_snapshot}" "${decision_log}"
}
trap cleanup EXIT

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  "${ROOT_DIR}/scripts/build_ws.sh"
fi

if ! docker ps --format '{{.Names}}' | grep -qx "${STACK_NAME}-hlc"; then
  "${ROOT_DIR}/scripts/run_stack.sh"
fi

sleep 5

docker ps --format '{{.Names}} {{.Status}}' | grep "^${STACK_NAME}" | sort

docker logs "${STACK_NAME}-hlc" >"${hlc_log}" 2>&1
grep -q 'HLC entering state: Ready' "${hlc_log}"

docker logs "${STACK_NAME}-mc" >"${mc_log}" 2>&1
grep -q 'MissionControl entering state: Standby' "${mc_log}"

docker exec "${STACK_NAME}-ros-alate" bash -lc 'source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && timeout 15 ros2 topic list -t' >"${topic_snapshot}"
grep -Fq '/alate_output_high_level_control_telemetry [ros_alate_interfaces/msg/HlcTelemetry]' "${topic_snapshot}"
grep -Fq '/alate_output_high_level_control_state [ros_alate_interfaces/msg/HlcState]' "${topic_snapshot}"
grep -Fq '/alate_output_high_level_control_platform_errors [ros_alate_interfaces/msg/HlcPlatformError]' "${topic_snapshot}"
grep -Fq '/alate_output_mission_control_state [ros_alate_interfaces/msg/McState]' "${topic_snapshot}"

docker exec "${STACK_NAME}-ros-alate" bash -lc 'source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && timeout 10 ros2 topic echo --once /alate_output_high_level_control_telemetry' >/dev/null

docker exec "${STACK_NAME}-decision-dev" bash -lc 'source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && python3 -m pytest /workspace/ws/src/decision_agent/test' >/dev/null

docker exec "${STACK_NAME}-decision-dev" bash -lc 'source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && timeout 8 ros2 run decision_agent policy_node --ros-args --params-file /workspace/config/decision_agent/policy.yaml --params-file /workspace/config/decision_agent/mission.yaml' >"${decision_log}" 2>&1 &
DECISION_EXEC_PID=$!
sleep 3
docker exec "${STACK_NAME}-ros-alate" bash -lc 'source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && timeout 10 ros2 topic echo --once /alate_input_velocity' >/dev/null
kill "${DECISION_EXEC_PID}" >/dev/null 2>&1 || true
wait "${DECISION_EXEC_PID}" >/dev/null 2>&1 || true
DECISION_EXEC_PID=""

printf 'Validation complete.\n'
