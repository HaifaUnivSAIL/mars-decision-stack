#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

visual_runtime_config_src="${CONFIG_DIR}/alate/uav.visual.sitl.json"
visual_runtime_dir="${LOG_DIR}/runtime"
visual_runtime_config_host="${visual_runtime_dir}/uav.visual.sitl.json"
visual_runtime_config="/stack/runtime/uav.visual.sitl.json"

containers=(
  "${STACK_NAME}-decision-dev"
  "${STACK_NAME}-ros-nemala"
  "${STACK_NAME}-ros-alate"
  "${STACK_NAME}-hlc"
  "${STACK_NAME}-mc"
  "${STACK_NAME}-logger"
  "${STACK_NAME}-proxy"
  "${STACK_NAME}-visual-sim"
  "${STACK_NAME}-sitl"
)

mkdir -p "${LOG_DIR}" "${visual_runtime_dir}"

start_visual_gui() {
  local runtime_dir="/tmp/runtime-$(id -u)"
  local gui_home="/tmp/visual-home"
  local deadline=$((SECONDS + 10))

  docker exec -d \
    -u "$(id -u):$(id -g)" \
    -e "DISPLAY=${DISPLAY}" \
    -e "QT_X11_NO_MITSHM=1" \
    -e "HOME=${gui_home}" \
    -e "XDG_RUNTIME_DIR=${runtime_dir}" \
    -e "GZ_PARTITION=${GZ_PARTITION}" \
    "${STACK_NAME}-visual-sim" \
    bash -lc "mkdir -p '${gui_home}' '${runtime_dir}' && chmod 700 '${runtime_dir}' && exec gz sim -g -v4 >/tmp/visual-gui.log 2>&1"

  while true; do
    if docker exec "${STACK_NAME}-visual-sim" bash -lc "pgrep -af 'gz sim -g -v4' >/dev/null"; then
      return 0
    fi

    if [ "${SECONDS}" -ge "${deadline}" ]; then
      printf 'Timed out waiting for the Gazebo GUI client to stay up. Check /tmp/visual-gui.log in %s.\n' "${STACK_NAME}-visual-sim" >&2
      return 1
    fi

    sleep 1
  done
}

detect_visual_mavlink_port() {
  local detect_probe

  detect_probe="$(
    cat <<'PY'
import socket
import time

ports = (5762, 5763)
deadline = time.time() + 45.0
last_error = None

while time.time() < deadline:
    for port in ports:
        sock = socket.socket()
        sock.settimeout(3.0)
        try:
            sock.connect(("sitl", port))
            if sock.recv(1):
                print(port, flush=True)
                raise SystemExit(0)
        except Exception as exc:
            last_error = f"{port}: {exc!r}"
        finally:
            try:
                sock.close()
            except Exception:
                pass
    time.sleep(1.0)

raise SystemExit(f"Timed out waiting for MAVLink traffic on tcp:sitl:5762 or tcp:sitl:5763. Last error: {last_error}")
PY
  )"

  docker run --rm \
    --network "${NETWORK}" \
    --entrypoint python3 \
    "${IMAGE}" \
    -c "${detect_probe}"
}

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  "${ROOT_DIR}/scripts/build_ws.sh"
fi

if ! docker image inspect "${VISUAL_SIM_IMAGE}" >/dev/null 2>&1; then
  "${ROOT_DIR}/scripts/build_visual_stack.sh"
fi

for container in "${containers[@]}"; do
  docker rm -f "${container}" >/dev/null 2>&1 || true
done

docker volume inspect "${VOLUME}" >/dev/null 2>&1 || docker volume create "${VOLUME}" >/dev/null
docker network inspect "${NETWORK}" >/dev/null 2>&1 || docker network create "${NETWORK}" >/dev/null

visual_gui="${VISUAL_GUI:-1}"
visual_args=(
  -d
  --rm
  --name "${STACK_NAME}-visual-sim"
  --network "${NETWORK}"
  -e "GZ_PARTITION=${GZ_PARTITION}"
)

if [ "${visual_gui}" = "1" ]; then
  if [ -z "${DISPLAY:-}" ]; then
    printf 'DISPLAY is not set. Export DISPLAY or run VISUAL_GUI=0 ./scripts/run_visual_stack.sh for headless validation.\n' >&2
    exit 1
  fi

  visual_args+=(
    -e "DISPLAY=${DISPLAY}"
    -e "QT_X11_NO_MITSHM=1"
    -u "$(id -u):$(id -g)"
    -e HOME=/tmp/visual-home
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  )

  if [ -n "${XAUTHORITY:-}" ] && [ -f "${XAUTHORITY}" ]; then
    visual_args+=( -e "XAUTHORITY=${XAUTHORITY}" -v "${XAUTHORITY}:${XAUTHORITY}:ro" )
  elif [ -f "${HOME}/.Xauthority" ]; then
    visual_args+=( -e XAUTHORITY=/tmp/.Xauthority -v "${HOME}/.Xauthority:/tmp/.Xauthority:ro" )
  fi

  if [ -e /dev/dri ]; then
    visual_args+=( --device /dev/dri:/dev/dri )
  fi
fi

docker run "${visual_args[@]}" "${VISUAL_SIM_IMAGE}"

visual_sim_ip="$(
  docker inspect \
    --format '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' \
    "${STACK_NAME}-visual-sim"
)"

if [ -z "${visual_sim_ip}" ]; then
  printf 'Failed to determine the IP address of %s.\n' "${STACK_NAME}-visual-sim" >&2
  exit 1
fi

docker run -d --rm \
  --name "${STACK_NAME}-sitl" \
  --network "${NETWORK}" \
  --network-alias sitl \
  --entrypoint /opt/ardupilot/build/sitl/bin/arducopter \
  "${VISUAL_SIM_IMAGE}" \
  -I0 \
  --model JSON \
  --sim-address "${visual_sim_ip}" \
  --sim-port-in 9003 \
  --sim-port-out 9002 \
  --serial0 tcp:0.0.0.0:5760 \
  --defaults /opt/ardupilot/Tools/autotest/default_params/copter.parm,/opt/ardupilot/Tools/autotest/default_params/gazebo-iris.parm,/opt/ardupilot_gazebo/config/gazebo-iris-gimbal.parm

visual_mavlink_port="$(detect_visual_mavlink_port)"
sed "s#\"master\": \"tcp:sitl:[0-9]\\+\"#\"master\": \"tcp:sitl:${visual_mavlink_port}\"#" \
  "${visual_runtime_config_src}" >"${visual_runtime_config_host}"

docker run -d --rm \
  --name "${STACK_NAME}-proxy" \
  --network "${NETWORK}" \
  -v "${VOLUME}:/tmp/nemala" \
  -v "${visual_runtime_dir}:/stack/runtime:ro" \
  "${NEMALA_TOOLS_IMAGE}" \
  proxy uav "${visual_runtime_config}"

docker run -d --rm \
  --name "${STACK_NAME}-logger" \
  --network "${NETWORK}" \
  -v "${VOLUME}:/tmp/nemala" \
  -v "${LOG_DIR}:/stack/logs" \
  "${NEMALA_TOOLS_IMAGE}" \
  log ipc:///tmp/nemala/alate_log

printf 'Observed MAVLink traffic on tcp:sitl:%s\n' "${visual_mavlink_port}"

docker run -d --rm \
  --name "${STACK_NAME}-mc" \
  --network "${NETWORK}" \
  -w /opt/alate \
  -v "${VOLUME}:/tmp/nemala" \
  -v "${visual_runtime_dir}:/stack/runtime:ro" \
  "${IMAGE}" \
  ./mc uav "${visual_runtime_config}"

docker run -d --rm \
  --name "${STACK_NAME}-hlc" \
  --network "${NETWORK}" \
  -w /opt/alate \
  -v "${VOLUME}:/tmp/nemala" \
  -v "${visual_runtime_dir}:/stack/runtime:ro" \
  "${IMAGE}" \
  ./hlc uav "${visual_runtime_config}"

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

if [ "${visual_gui}" = "1" ]; then
  start_visual_gui
fi

printf 'Started %s visual stack on docker network %s\n' "${STACK_NAME}" "${NETWORK}"
if [ "${visual_gui}" = "1" ]; then
  printf 'Gazebo GUI was launched against the headless simulator server in %s.\n' "${STACK_NAME}-visual-sim"
else
  printf 'Gazebo is running headlessly in %s.\n' "${STACK_NAME}-visual-sim"
fi
printf 'Open a dev shell with %s\n' "${ROOT_DIR}/scripts/dev_shell.sh"
