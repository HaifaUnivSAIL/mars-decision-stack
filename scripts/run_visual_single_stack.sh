#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

visual_runtime_config_src="${CONFIG_DIR}/alate/uav.visual.sitl.json"
camera_deployment_config_src="${CONFIG_DIR}/visual/camera.deployment.json"
visual_runtime_state_dir="${LOG_DIR}/runtime"
visual_mode_file="${visual_runtime_state_dir}/visual.mode"
visual_current_run_file="${visual_runtime_state_dir}/visual.current_run_dir"
visual_manifest_file="${visual_runtime_state_dir}/visual.manifest.json"
visual_manifest_env_file="${visual_runtime_state_dir}/visual.manifest.env"

visual_mode="${VISUAL_MODE:-experiment}"
profile_enabled=0

while [ "$#" -gt 0 ]; do
  case "$1" in
    --calib)
      visual_mode="calib"
      ;;
    --experiment)
      visual_mode="experiment"
      ;;
    --profile)
      profile_enabled=1
      ;;
    *)
      printf 'Unsupported argument: %s\n' "$1" >&2
      printf 'Usage: %s [--calib|--experiment] [--profile]\n' "$0" >&2
      exit 1
      ;;
  esac
  shift
done

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

run_id="$(date +%Y%m%d-%H%M%S)"
current_run_dir="${LOG_DIR}/runs/${run_id}-${visual_mode}"
current_run_mount="/stack/run"
current_workspace_run_mount="/workspace/run"
visual_nvidia_egl_vendor_host="${current_run_dir}/nvidia-egl-vendor.json"
visual_nvidia_egl_vendor_mount="${current_run_mount}/nvidia-egl-vendor.json"
visual_assets_dir="${current_run_dir}/visual_assets"
visual_tools_dir="${current_run_dir}/tools"
run_logs_dir="${current_run_dir}/logs"
run_diagnostics_dir="${current_run_dir}/diagnostics"
profile_output_dir="${run_diagnostics_dir}/profile"
profile_manifest_path="${profile_output_dir}/profile.manifest.json"
profile_state_path="${profile_output_dir}/profile.state.json"
profile_supervisor_pid_file="${profile_output_dir}/profile.supervisor.pid"
visual_runtime_config_host="${current_run_dir}/uav.visual.sitl.json"
visual_runtime_config="${current_run_mount}/uav.visual.sitl.json"
log_capture_pid_file="${current_run_dir}/log_capture.pids"

mkdir -p "${visual_runtime_state_dir}" "${current_run_dir}" "${visual_assets_dir}" "${visual_tools_dir}" "${run_logs_dir}" "${run_diagnostics_dir}" "${profile_output_dir}"
: >"${log_capture_pid_file}"

on_error() {
  local exit_code=$?
  printf 'Visual stack startup failed. Run directory: %s\n' "${current_run_dir}" >&2
  if [ -f "${run_diagnostics_dir}/deployment.verify.json" ]; then
    printf 'Latest deployment verification report: %s\n' "${run_diagnostics_dir}/deployment.verify.json" >&2
  fi
  exit "${exit_code}"
}
trap on_error ERR

log() {
  printf '[run_visual_stack] %s\n' "$*"
}

collect_dri_group_ids() {
  local path
  for path in /dev/dri/card* /dev/dri/renderD*; do
    [ -e "${path}" ] || continue
    stat -c '%g' "${path}"
  done | sort -u
}

has_nvidia_runtime() {
  docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"'
}

append_visual_nvidia_args() {
  local -n target_ref="$1"

  if [ ! -e /dev/nvidia0 ]; then
    return 1
  fi
  if ! has_nvidia_runtime; then
    return 1
  fi

  target_ref+=(
    --gpus all
    -e 'NVIDIA_VISIBLE_DEVICES=all'
    -e 'NVIDIA_DRIVER_CAPABILITIES=all'
    -e '__NV_PRIME_RENDER_OFFLOAD=1'
    -e '__GLX_VENDOR_LIBRARY_NAME=nvidia'
    -e "__EGL_VENDOR_LIBRARY_FILENAMES=${visual_nvidia_egl_vendor_mount}"
    -e '__VK_LAYER_NV_optimus=NVIDIA_only'
    -e 'GBM_BACKEND=nvidia-drm'
    -e 'LIBGL_ALWAYS_SOFTWARE=0'
  )
  return 0
}

append_visual_gpu_args() {
  local -n target_ref="$1"
  local gid

  if append_visual_nvidia_args "$1"; then
    log "Enabled NVIDIA GPU acceleration for the visual container."
    return 0
  fi

  if [ ! -e /dev/dri ]; then
    return 0
  fi

  target_ref+=( --device /dev/dri:/dev/dri )
  while IFS= read -r gid; do
    [ -n "${gid}" ] || continue
    target_ref+=( --group-add "${gid}" )
  done < <(collect_dri_group_ids)
}

running_stack_containers() {
  docker ps --format '{{.Names}}' | grep -E "^${STACK_NAME}-(decision-dev|ros-nemala|ros-alate|hlc|mc|logger|proxy|visual-sim|sitl)$" || true
}

start_log_capture() {
  local container_name="$1"
  local log_file="$2"
  nohup bash -lc "docker logs -f '${container_name}' >> '${log_file}' 2>&1" >/dev/null 2>&1 &
  echo "$!" >>"${log_capture_pid_file}"
}

profile_env_args=()
if [ "${profile_enabled}" = '1' ]; then
  profile_env_args=(
    -e STACK_PROFILE=1
    -e "STACK_PROFILE_RUN_DIR=${current_run_mount}"
  )
fi

start_profile_supervisor() {
  local sitl_host="$1"
  local mavlink_port="$2"
  local profile_supervisor_log="${run_logs_dir}/profile-supervisor.log"

  if [ "${profile_enabled}" != '1' ]; then
    return 0
  fi

  python3 "${ROOT_DIR}/scripts/build_stack_profile_manifest.py" \
    --visual-manifest "${visual_assets_dir}/manifest.json" \
    --output "${profile_manifest_path}" \
    --run-dir "${current_run_dir}" \
    --root-dir "${ROOT_DIR}" \
    --stack-name "${STACK_NAME}" \
    --network "${NETWORK}" \
    --image "${IMAGE}" \
    --mode single \
    --visual-gui "${visual_gui}" \
    --container "${STACK_NAME}-visual-sim" \
    --container "${STACK_NAME}-sitl" \
    --container "${STACK_NAME}-proxy" \
    --container "${STACK_NAME}-logger" \
    --container "${STACK_NAME}-mc" \
    --container "${STACK_NAME}-hlc" \
    --container "${STACK_NAME}-ros-alate" \
    --container "${STACK_NAME}-ros-nemala" \
    --container "${STACK_NAME}-decision-dev" \
    --container "${STACK_NAME}-visual-gui" \
    --container "${STACK_NAME}-profile-visual" \
    --container "${STACK_NAME}-profile-ros" \
    --drone-endpoint "drone_1=tcp:${sitl_host}:${mavlink_port}"

  install -m 755 "${ROOT_DIR}/sim/tools/stack_visual_profiler.py" "${visual_tools_dir}/stack_visual_profiler.py"
  nohup python3 "${ROOT_DIR}/scripts/stack_profile_supervisor.py" \
    --profile-manifest "${profile_manifest_path}" >"${profile_supervisor_log}" 2>&1 &
  printf '%s\n' "$!" >"${profile_supervisor_pid_file}"
}

start_profile_sidecars() {
  if [ "${profile_enabled}" != '1' ]; then
    return 0
  fi

  docker run -d --rm \
    --name "${STACK_NAME}-profile-visual" \
    --network "${NETWORK}" \
    -e "GZ_PARTITION=${GZ_PARTITION}" \
    -v "${current_run_dir}:${current_run_mount}:rw" \
    --entrypoint python3 \
    "${VISUAL_SIM_IMAGE}" \
    "${current_run_mount}/tools/stack_visual_profiler.py" "${current_run_mount}/diagnostics/profile/profile.manifest.json"
  start_log_capture "${STACK_NAME}-profile-visual" "${run_logs_dir}/stack-visual-profiler.log"

  docker run -d --rm \
    --name "${STACK_NAME}-profile-ros" \
    --network "${NETWORK}" \
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
    -e PYTHONUNBUFFERED=1 \
    -v "${WS_DIR}:/workspace/ws:rw" \
    -v "${current_run_dir}:${current_workspace_run_mount}:rw" \
    "${IMAGE}" \
    bash -lc "source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && python3 /workspace/ws/src/decision_agent/decision_agent/stack_ros_profiler.py --profile-manifest ${current_workspace_run_mount}/diagnostics/profile/profile.manifest.json"
  start_log_capture "${STACK_NAME}-profile-ros" "${run_logs_dir}/stack-ros-profiler.log"
}

write_run_manifest() {
  python3 - \
    "${visual_assets_dir}/manifest.json" \
    "${camera_deployment_config_src}" \
    "${current_run_dir}/run_manifest.json" \
    "${current_run_dir}" \
    "${IMAGE}" \
    "${VISUAL_SIM_IMAGE}" \
    "${NEMALA_TOOLS_IMAGE}" \
    "${NETWORK}" \
    "${VOLUME}" \
    "${ROS_DOMAIN_ID}" \
    "${GZ_PARTITION}" <<'PY'
import json
import subprocess
import sys
from pathlib import Path

manifest_path = Path(sys.argv[1])
config_path = Path(sys.argv[2])
out_path = Path(sys.argv[3])
run_dir = sys.argv[4]
image = sys.argv[5]
visual_image = sys.argv[6]
nemala_tools = sys.argv[7]
network = sys.argv[8]
volume = sys.argv[9]
ros_domain_id = sys.argv[10]
gz_partition = sys.argv[11]

with manifest_path.open() as handle:
    manifest = json.load(handle)
with config_path.open() as handle:
    requested_config = json.load(handle)

def image_id(name: str) -> str:
    try:
        return subprocess.check_output([
            'docker', 'image', 'inspect', '--format', '{{.Id}}', name
        ], text=True).strip()
    except Exception:
        return ''

manifest.update({
    'run_dir': run_dir,
    'requested_camera_config': requested_config,
    'image_ids': {
        'stack': image_id(image),
        'visual_sim': image_id(visual_image),
        'nemala_tools': image_id(nemala_tools),
    },
    'environment': {
        'NETWORK': network,
        'VOLUME': volume,
        'ROS_DOMAIN_ID': ros_domain_id,
        'GZ_PARTITION': gz_partition,
    },
})
out_path.write_text(json.dumps(manifest, indent=2) + '\n')
PY
}

capture_basic_diagnostics() {
  docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}' >"${run_diagnostics_dir}/docker.ps.txt" || true
  docker network inspect "${NETWORK}" >"${run_diagnostics_dir}/docker.network.json" 2>/dev/null || true
}

capture_ros_diagnostics() {
  docker exec "${STACK_NAME}-decision-dev" bash -lc \
    'source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && timeout 20 ros2 topic list -t' \
    >"${run_diagnostics_dir}/ros2.topics.txt" 2>&1 || true
}

wait_for_visual_topics() {
  local deadline=$((SECONDS + 30))
  while true; do
    if docker exec -u "$(id -u):$(id -g)" -e "GZ_PARTITION=${GZ_PARTITION}" "${STACK_NAME}-visual-sim" \
      bash -lc "timeout 10 gz topic -l" >"${run_diagnostics_dir}/gz.topics.txt" 2>/dev/null; then
      if grep -Fxq '/world/mars_iris_dual_view/pose/info' "${run_diagnostics_dir}/gz.topics.txt"; then
        return 0
      fi
    fi
    if [ "${SECONDS}" -ge "${deadline}" ]; then
      printf 'Timed out waiting for Gazebo topics to become available.\n' >&2
      return 1
    fi
    sleep 1
  done
}

capture_pose_snapshot() {
  local target_file="$1"
  docker exec -u "$(id -u):$(id -g)" -e "GZ_PARTITION=${GZ_PARTITION}" "${STACK_NAME}-visual-sim" \
    bash -lc "timeout 15 gz topic -e -n 1 -t /world/${VISUAL_RUNTIME_WORLD_NAME}/dynamic_pose/info" >"${target_file}"
}

capture_joint_state_snapshot() {
  local target_file="$1"
  docker exec -u "$(id -u):$(id -g)" -e "GZ_PARTITION=${GZ_PARTITION}" "${STACK_NAME}-visual-sim" \
    bash -lc "timeout 15 gz topic -e -n 1 -t /world/${VISUAL_RUNTIME_WORLD_NAME}/model/${VISUAL_RUNTIME_MODEL_NAME}/joint_state" >"${target_file}"
}

publish_calibration_pose() {
  if [ "${visual_mode}" != 'calib' ]; then
    return 0
  fi

  docker exec -u "$(id -u):$(id -g)" -e "GZ_PARTITION=${GZ_PARTITION}" "${STACK_NAME}-visual-sim" bash -lc "
    set -euo pipefail
    gz topic -t '/model/${VISUAL_RUNTIME_MODEL_NAME}/joint/deployed_camera_x_joint/0/cmd_pos' -m gz.msgs.Double -p 'data: ${VISUAL_DEPLOYMENT_X}' >/dev/null
    gz topic -t '/model/${VISUAL_RUNTIME_MODEL_NAME}/joint/deployed_camera_y_joint/0/cmd_pos' -m gz.msgs.Double -p 'data: ${VISUAL_DEPLOYMENT_Y}' >/dev/null
    gz topic -t '/model/${VISUAL_RUNTIME_MODEL_NAME}/joint/deployed_camera_z_joint/0/cmd_pos' -m gz.msgs.Double -p 'data: ${VISUAL_DEPLOYMENT_Z}' >/dev/null
    gz topic -t '/model/${VISUAL_RUNTIME_MODEL_NAME}/joint/deployed_camera_yaw_joint/0/cmd_pos' -m gz.msgs.Double -p 'data: ${VISUAL_DEPLOYMENT_YAW}' >/dev/null
    gz topic -t '/model/${VISUAL_RUNTIME_MODEL_NAME}/joint/deployed_camera_pitch_joint/0/cmd_pos' -m gz.msgs.Double -p 'data: ${VISUAL_DEPLOYMENT_PITCH}' >/dev/null
    gz topic -t '/model/${VISUAL_RUNTIME_MODEL_NAME}/joint/deployed_camera_roll_joint/0/cmd_pos' -m gz.msgs.Double -p 'data: ${VISUAL_DEPLOYMENT_ROLL}' >/dev/null
  "
}

initialize_calibration_pose() {
  if [ "${visual_mode}" != 'calib' ]; then
    return 0
  fi
  log "Initializing calibration joints from ${camera_deployment_config_src}"
  publish_calibration_pose
  sleep 1
}

verify_visual_deployment() {
  local pose_file="${run_diagnostics_dir}/pose.info.txt"
  local joint_state_file="${run_diagnostics_dir}/joint_state.txt"
  local deadline attempts
  if [ "${visual_mode}" = 'calib' ]; then
    deadline=$((SECONDS + 20))
  else
    deadline=$((SECONDS + 5))
  fi
  attempts=0

  while true; do
    attempts=$((attempts + 1))
    if [ "${visual_mode}" = 'calib' ]; then
      publish_calibration_pose
      sleep 1
    fi
    capture_pose_snapshot "${pose_file}"
    capture_joint_state_snapshot "${joint_state_file}"
    if python3 "${ROOT_DIR}/scripts/verify_visual_runtime.py" \
      --manifest "${visual_assets_dir}/manifest.json" \
      --pose-info "${pose_file}" \
      --joint-state "${joint_state_file}" \
      --report "${run_diagnostics_dir}/deployment.verify.json"; then
      return 0
    fi
    if [ "${SECONDS}" -ge "${deadline}" ]; then
      printf 'Timed out verifying visual deployment after %d attempts.\n' "${attempts}" >&2
      return 1
    fi
    sleep 1
  done
}

detect_visual_mavlink_port() {
  local detect_probe

  detect_probe="$({
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
  })"

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

existing_stack="$(running_stack_containers)"
if [ -n "${existing_stack}" ]; then
  log "Existing ${STACK_NAME} stack detected. Restarting it in ${visual_mode} mode with fresh visual assets."
  printf '%s\n' "${existing_stack}"
  "${ROOT_DIR}/scripts/stop_stack.sh"
fi

docker volume inspect "${VOLUME}" >/dev/null 2>&1 || docker volume create "${VOLUME}" >/dev/null
docker network inspect "${NETWORK}" >/dev/null 2>&1 || docker network create "${NETWORK}" >/dev/null

cp "${camera_deployment_config_src}" "${current_run_dir}/camera.deployment.requested.json"
cat >"${visual_nvidia_egl_vendor_host}" <<'EOF'
{
  "file_format_version": "1.0.0",
  "ICD": {
    "library_path": "libEGL_nvidia.so.0"
  }
}
EOF
python3 "${ROOT_DIR}/scripts/generate_visual_assets.py" \
  --mode "${visual_mode}" \
  --run-id "${run_id}" \
  --config "${camera_deployment_config_src}" \
  --output-dir "${visual_assets_dir}" \
  --source-model "${ROOT_DIR}/external/ardupilot_gazebo/models/iris_with_camera_calibration/model.sdf" \
  --experiment-world-template "${ROOT_DIR}/sim/worlds/mars_iris_dual_view.sdf" \
  --calibration-world-template "${ROOT_DIR}/sim/worlds/mars_iris_dual_view_calibration.sdf"

# shellcheck disable=SC1090
source "${visual_assets_dir}/manifest.env"
write_run_manifest
cp "${visual_assets_dir}/manifest.json" "${visual_manifest_file}"
cp "${visual_assets_dir}/manifest.env" "${visual_manifest_env_file}"
printf '%s\n' "${visual_mode}" >"${visual_mode_file}"
printf '%s\n' "${current_run_dir}" >"${visual_current_run_file}"

sed "s#\"master\": \"tcp:sitl:[0-9]\\+\"#\"master\": \"tcp:sitl:5760\"#" \
  "${visual_runtime_config_src}" >"${visual_runtime_config_host}"
cp "${visual_runtime_config_host}" "${current_run_dir}/uav.visual.sitl.generated.json"

capture_basic_diagnostics

visual_gui="${VISUAL_GUI:-1}"
visual_args=(
  -d
  --rm
  --name "${STACK_NAME}-visual-sim"
  --network "${NETWORK}"
  -e "GZ_PARTITION=${GZ_PARTITION}"
  -e "VISUAL_WORLD=${current_run_mount}/visual_assets/${VISUAL_RUNTIME_WORLD_RELATIVE}"
  -e "VISUAL_RESOURCE_PATH_PREFIX=${current_run_mount}/visual_assets/models"
  -v "${current_run_dir}:${current_run_mount}:rw"
)

if [ "${visual_gui}" = '1' ]; then
  if [ -z "${DISPLAY:-}" ]; then
    printf 'DISPLAY is not set. Export DISPLAY or run VISUAL_GUI=0 ./scripts/run_visual_stack.sh for headless validation.\n' >&2
    exit 1
  fi

  visual_args+=(
    -e "DISPLAY=${DISPLAY}"
    -e 'QT_X11_NO_MITSHM=1'
    -u "$(id -u):$(id -g)"
    -e 'HOME=/tmp/visual-home'
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  )

  if [ -n "${XAUTHORITY:-}" ] && [ -f "${XAUTHORITY}" ]; then
    visual_args+=( -e "XAUTHORITY=${XAUTHORITY}" -v "${XAUTHORITY}:${XAUTHORITY}:ro" )
  elif [ -f "${HOME}/.Xauthority" ]; then
    visual_args+=( -e XAUTHORITY=/tmp/.Xauthority -v "${HOME}/.Xauthority:/tmp/.Xauthority:ro" )
  fi

fi

append_visual_gpu_args visual_args

docker run "${visual_args[@]}" "${VISUAL_SIM_IMAGE}"
start_log_capture "${STACK_NAME}-visual-sim" "${run_logs_dir}/visual-sim-server.log"
wait_for_visual_topics
initialize_calibration_pose
verify_visual_deployment

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
  -v "${CONFIG_DIR}:/stack/config:ro" \
  --entrypoint /opt/ardupilot/build/sitl/bin/arducopter \
  "${VISUAL_SIM_IMAGE}" \
  -I0 \
  --model JSON \
  --sim-address "${visual_sim_ip}" \
  --sim-port-in 9003 \
  --sim-port-out 9002 \
  --serial0 tcp:5760 \
  --defaults /opt/ardupilot/Tools/autotest/default_params/copter.parm,/opt/ardupilot/Tools/autotest/default_params/gazebo-iris.parm,/stack/config/ardupilot/visual-sitl.parm
start_log_capture "${STACK_NAME}-sitl" "${run_logs_dir}/sitl.log"

visual_mavlink_port="$(detect_visual_mavlink_port)"
sitl_runtime_host="$(
  docker inspect --format '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "${STACK_NAME}-sitl"
)"
if [ -z "${sitl_runtime_host}" ]; then
  sitl_runtime_host='sitl'
fi
sed "s#\"master\": \"tcp:sitl:[0-9]\\+\"#\"master\": \"tcp:sitl:${visual_mavlink_port}\"#" \
  "${visual_runtime_config_src}" >"${visual_runtime_config_host}"
cp "${visual_runtime_config_host}" "${current_run_dir}/uav.visual.sitl.generated.json"
start_profile_supervisor "${sitl_runtime_host}" "${visual_mavlink_port}"

docker run -d --rm \
  --name "${STACK_NAME}-proxy" \
  --network "${NETWORK}" \
  -v "${VOLUME}:/tmp/nemala" \
  -v "${current_run_dir}:${current_run_mount}:ro" \
  "${NEMALA_TOOLS_IMAGE}" \
  proxy uav "${visual_runtime_config}"
start_log_capture "${STACK_NAME}-proxy" "${run_logs_dir}/proxy.log"

docker run -d --rm \
  --name "${STACK_NAME}-logger" \
  --network "${NETWORK}" \
  -v "${VOLUME}:/tmp/nemala" \
  -v "${LOG_DIR}:/stack/logs" \
  "${NEMALA_TOOLS_IMAGE}" \
  log ipc:///tmp/nemala/alate_log
start_log_capture "${STACK_NAME}-logger" "${run_logs_dir}/nemala-logger.log"

log "Observed MAVLink traffic on tcp:sitl:${visual_mavlink_port}"

docker run -d --rm \
  --name "${STACK_NAME}-mc" \
  --network "${NETWORK}" \
  -w /opt/alate \
  -e PYTHONUNBUFFERED=1 \
  "${profile_env_args[@]}" \
  -v "${VOLUME}:/tmp/nemala" \
  -v "${current_run_dir}:${current_run_mount}:ro" \
  "${IMAGE}" \
  ./mc uav "${visual_runtime_config}"
start_log_capture "${STACK_NAME}-mc" "${run_logs_dir}/mc.log"

docker run -d --rm \
  --name "${STACK_NAME}-hlc" \
  --network "${NETWORK}" \
  -w /opt/alate \
  -e PYTHONUNBUFFERED=1 \
  "${profile_env_args[@]}" \
  -v "${VOLUME}:/tmp/nemala" \
  -v "${current_run_dir}:${current_run_mount}:ro" \
  "${IMAGE}" \
  ./hlc uav "${visual_runtime_config}"
start_log_capture "${STACK_NAME}-hlc" "${run_logs_dir}/hlc.log"

sleep 2

docker run -d --rm \
  --name "${STACK_NAME}-ros-alate" \
  --network "${NETWORK}" \
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
  -e RCUTILS_LOGGING_BUFFERED_STREAM=1 \
  -e PYTHONUNBUFFERED=1 \
  "${profile_env_args[@]}" \
  -v "${VOLUME}:/tmp/nemala" \
  -v "${CONFIG_DIR}:/stack/config:ro" \
  "${IMAGE}" \
  bash -lc "source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && ros2 run ros_alate adapter --ros-args --log-level debug --params-file /stack/config/ros_alate/adapter.yaml"
start_log_capture "${STACK_NAME}-ros-alate" "${run_logs_dir}/ros-alate.log"

docker run -d --rm \
  --name "${STACK_NAME}-ros-nemala" \
  --network "${NETWORK}" \
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
  -e RCUTILS_LOGGING_BUFFERED_STREAM=1 \
  -e PYTHONUNBUFFERED=1 \
  "${profile_env_args[@]}" \
  -v "${VOLUME}:/tmp/nemala" \
  -v "${CONFIG_DIR}:/stack/config:ro" \
  "${IMAGE}" \
  bash -lc "source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && ros2 run ros_nemala node_manager --ros-args --log-level debug --params-file /stack/config/ros_nemala/node_manager.yaml"
start_log_capture "${STACK_NAME}-ros-nemala" "${run_logs_dir}/ros-nemala.log"

docker run -d --rm \
  --name "${STACK_NAME}-decision-dev" \
  --network "${NETWORK}" \
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
  -e PYTHONUNBUFFERED=1 \
  "${profile_env_args[@]}" \
  -v "${VOLUME}:/tmp/nemala" \
  -v "${CONFIG_DIR}:/workspace/config:ro" \
  -v "${WS_DIR}:/workspace/ws:rw" \
  -v "${EXTERNAL_DIR}:/workspace/external:ro" \
  -v "${current_run_dir}:${current_workspace_run_mount}:rw" \
  "${IMAGE}" \
  bash -lc "source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && sleep infinity"
start_log_capture "${STACK_NAME}-decision-dev" "${run_logs_dir}/decision-dev.log"

start_profile_sidecars

capture_basic_diagnostics
capture_ros_diagnostics

if [ "${visual_gui}" = '1' ]; then
  "${ROOT_DIR}/scripts/ensure_visual_gui.sh"
fi

trap - ERR
printf 'Started %s visual stack on docker network %s\n' "${STACK_NAME}" "${NETWORK}"
printf 'Visual mode: %s\n' "${visual_mode}"
printf 'Run directory: %s\n' "${current_run_dir}"
if [ "${profile_enabled}" = '1' ]; then
  printf 'Profiling diagnostics: %s\n' "${profile_output_dir}"
fi
if [ "${visual_gui}" = '1' ]; then
  printf 'Gazebo GUI was launched in %s and connected to the simulator server in %s.\n' "${STACK_NAME}-visual-gui" "${STACK_NAME}-visual-sim"
else
  printf 'Gazebo is running headlessly in %s.\n' "${STACK_NAME}-visual-sim"
fi
if [ "${RUN_VISUAL_STACK_QUIET_HINTS:-0}" != '1' ]; then
  printf 'This command only starts the stack and GUI. It does not open keyboard control.\n'
  printf 'For keyboard flight control, run %s\n' "${ROOT_DIR}/scripts/run_visual_teleop.sh"
fi
printf 'Open a dev shell with %s\n' "${ROOT_DIR}/scripts/dev_shell.sh"
