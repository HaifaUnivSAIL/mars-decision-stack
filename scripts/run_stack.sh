#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

containers=(
  "${STACK_NAME}-decision-dev"
  "${STACK_NAME}-ros-nemala"
  "${STACK_NAME}-ros-alate"
  "${STACK_NAME}-hlc"
  "${STACK_NAME}-mc"
  "${STACK_NAME}-logger"
  "${STACK_NAME}-proxy"
  "${STACK_NAME}-sitl"
)

mkdir -p "${LOG_DIR}"

running_stack_containers() {
  docker ps --format '{{.Names}}' | grep -E "^${STACK_NAME}-(decision-dev|ros-nemala|ros-alate|hlc|mc|logger|proxy|sitl|visual-sim)$" || true
}

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  "${ROOT_DIR}/scripts/build_ws.sh"
fi

existing_stack="$(running_stack_containers)"
if [ -n "${existing_stack}" ]; then
  printf 'Existing %s stack detected. Stopping it before fresh bring-up.\n' "${STACK_NAME}"
  printf '%s\n' "${existing_stack}"
  "${ROOT_DIR}/scripts/stop_stack.sh"
fi

docker volume inspect "${VOLUME}" >/dev/null 2>&1 || docker volume create "${VOLUME}" >/dev/null
docker network inspect "${NETWORK}" >/dev/null 2>&1 || docker network create "${NETWORK}" >/dev/null

docker run -d --rm \
  --name "${STACK_NAME}-sitl" \
  --network "${NETWORK}" \
  --network-alias sitl \
  --entrypoint /ardupilot/build/sitl/bin/arducopter \
  "${SITL_IMAGE}" \
  -S \
  -I0 \
  --model + \
  --defaults /ardupilot/Tools/autotest/default_params/copter.parm

docker run -d --rm \
  --name "${STACK_NAME}-proxy" \
  --network "${NETWORK}" \
  -v "${VOLUME}:/tmp/nemala" \
  -v "${CONFIG_DIR}:/stack/config:ro" \
  "${NEMALA_TOOLS_IMAGE}" \
  proxy uav /stack/config/alate/uav.sitl.json

docker run -d --rm \
  --name "${STACK_NAME}-logger" \
  --network "${NETWORK}" \
  -v "${VOLUME}:/tmp/nemala" \
  -v "${LOG_DIR}:/stack/logs" \
  "${NEMALA_TOOLS_IMAGE}" \
  log ipc:///tmp/nemala/alate_log

sleep 2

docker run -d --rm \
  --name "${STACK_NAME}-mc" \
  --network "${NETWORK}" \
  -w /opt/alate \
  -v "${VOLUME}:/tmp/nemala" \
  -v "${CONFIG_DIR}:/stack/config:ro" \
  "${IMAGE}" \
  ./mc uav /stack/config/alate/uav.sitl.json

docker run -d --rm \
  --name "${STACK_NAME}-hlc" \
  --network "${NETWORK}" \
  -w /opt/alate \
  -v "${VOLUME}:/tmp/nemala" \
  -v "${CONFIG_DIR}:/stack/config:ro" \
  "${IMAGE}" \
  ./hlc uav /stack/config/alate/uav.sitl.json

sleep 2

docker run -d --rm \
  --name "${STACK_NAME}-ros-alate" \
  --network "${NETWORK}" \
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
  -v "${VOLUME}:/tmp/nemala" \
  -v "${CONFIG_DIR}:/stack/config:ro" \
  "${IMAGE}" \
  bash -lc "source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && ros2 run ros_alate adapter --ros-args --params-file /stack/config/ros_alate/adapter.yaml"

docker run -d --rm \
  --name "${STACK_NAME}-ros-nemala" \
  --network "${NETWORK}" \
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
  -v "${VOLUME}:/tmp/nemala" \
  -v "${CONFIG_DIR}:/stack/config:ro" \
  "${IMAGE}" \
  bash -lc "source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && ros2 run ros_nemala node_manager --ros-args --params-file /stack/config/ros_nemala/node_manager.yaml"

docker run -d --rm \
  --name "${STACK_NAME}-decision-dev" \
  --network "${NETWORK}" \
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
  -v "${VOLUME}:/tmp/nemala" \
  -v "${CONFIG_DIR}:/workspace/config:ro" \
  -v "${WS_DIR}:/workspace/ws:rw" \
  -v "${EXTERNAL_DIR}:/workspace/external:ro" \
  "${IMAGE}" \
  bash -lc "source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && sleep infinity"

printf 'Started %s on docker network %s\n' "${STACK_NAME}" "${NETWORK}"
printf 'Open a dev shell with %s\n' "${ROOT_DIR}/scripts/dev_shell.sh"
