#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

fleet_config_src="${CONFIG_DIR}/swarm/visual.swarm.json"
profile_enabled=0
while [ "$#" -gt 0 ]; do
  case "$1" in
    --fleet)
      shift
      if [ "$#" -eq 0 ]; then
        printf 'Missing value for --fleet\n' >&2
        exit 1
      fi
      fleet_config_src="$1"
      ;;
    --profile)
      profile_enabled=1
      ;;
    *)
      printf 'Unsupported argument: %s\n' "$1" >&2
      printf 'Usage: %s [--fleet PATH] [--profile]\n' "$0" >&2
      exit 1
      ;;
  esac
  shift
done

visual_runtime_state_dir="${LOG_DIR}/runtime"
visual_mode_file="${visual_runtime_state_dir}/visual.mode"
visual_current_run_file="${visual_runtime_state_dir}/visual.current_run_dir"
visual_manifest_file="${visual_runtime_state_dir}/visual.manifest.json"
visual_manifest_env_file="${visual_runtime_state_dir}/visual.manifest.env"
run_id="$(date +%Y%m%d-%H%M%S)"
current_run_dir="${LOG_DIR}/runs/${run_id}-experiment"
current_run_mount="/stack/run"
current_workspace_run_mount="/workspace/run"
visual_nvidia_egl_vendor_host="${current_run_dir}/nvidia-egl-vendor.json"
visual_nvidia_egl_vendor_mount="${current_run_mount}/nvidia-egl-vendor.json"
visual_assets_dir="${current_run_dir}/visual_assets"
visual_tools_dir="${current_run_dir}/tools"
run_logs_dir="${current_run_dir}/logs"
run_diagnostics_dir="${current_run_dir}/diagnostics"
run_latency_dir="${run_diagnostics_dir}/latency"
run_readiness_dir="${run_diagnostics_dir}/readiness"
profile_output_dir="${run_diagnostics_dir}/profile"
profile_manifest_path="${profile_output_dir}/profile.manifest.json"
profile_state_path="${profile_output_dir}/profile.state.json"
profile_supervisor_pid_file="${profile_output_dir}/profile.supervisor.pid"
log_capture_pid_file="${current_run_dir}/log_capture.pids"
requested_fleet_config_copy="${current_run_dir}/visual.swarm.requested.json"

mkdir -p "${visual_runtime_state_dir}" "${current_run_dir}" "${visual_assets_dir}" "${visual_tools_dir}" "${run_logs_dir}" "${run_diagnostics_dir}" "${run_latency_dir}" "${run_readiness_dir}" "${profile_output_dir}"
: >"${log_capture_pid_file}"

on_error() {
  local exit_code=$?
  printf 'Visual fleet stack startup failed. Run directory: %s\n' "${current_run_dir}" >&2
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
  docker ps --format '{{.Names}}' | grep -E "^${STACK_NAME}-" || true
}

start_log_capture() {
  local container_name="$1"
  local log_file="$2"
  mkdir -p "$(dirname "${log_file}")"
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
  local container_args=(
    --container "${STACK_NAME}-visual-sim"
    --container "${STACK_NAME}-visual-gui"
    --container "${STACK_NAME}-decision-dev"
    --container "${STACK_NAME}-visual-focus-router"
    --container "${STACK_NAME}-profile-visual"
    --container "${STACK_NAME}-profile-ros"
  )
  local endpoint_args=()
  local drone_id sitl_host mavlink_port mavlink_aux_port
  local profile_supervisor_log="${run_logs_dir}/profile-supervisor.log"

  if [ "${profile_enabled}" != '1' ]; then
    return 0
  fi

  while IFS=$'\t' read -r drone_id _namespace sitl_host _runtime_model_name _serial0_port mavlink_port mavlink_aux_port _fdm_port_in _fdm_port_out _proxy_name _proxy_sub _proxy_pub _proxy_log _chase_topic _camera_topic _alate_rel _ros_alate_rel _ros_nemala_rel; do
    [ -n "${drone_id}" ] || continue
    container_args+=( --container "${STACK_NAME}-${drone_id}-sitl" )
    container_args+=( --container "${STACK_NAME}-${drone_id}-proxy" )
    container_args+=( --container "${STACK_NAME}-${drone_id}-logger" )
    container_args+=( --container "${STACK_NAME}-${drone_id}-mc" )
    container_args+=( --container "${STACK_NAME}-${drone_id}-hlc" )
    container_args+=( --container "${STACK_NAME}-${drone_id}-ros-alate" )
    container_args+=( --container "${STACK_NAME}-${drone_id}-ros-nemala" )
    endpoint_args+=( --drone-endpoint "${drone_id}=tcp:${sitl_host}:${mavlink_aux_port}" )
  done <"${visual_assets_dir}/drones.tsv"

  python3 "${ROOT_DIR}/scripts/build_stack_profile_manifest.py" \
    --visual-manifest "${visual_assets_dir}/manifest.json" \
    --output "${profile_manifest_path}" \
    --run-dir "${current_run_dir}" \
    --root-dir "${ROOT_DIR}" \
    --stack-name "${STACK_NAME}" \
    --network "${NETWORK}" \
    --image "${IMAGE}" \
    --mode fleet \
    --visual-gui "${visual_gui}" \
    "${container_args[@]}" \
    "${endpoint_args[@]}"

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
      if grep -Fxq "/world/${VISUAL_RUNTIME_WORLD_NAME}/dynamic_pose/info" "${run_diagnostics_dir}/gz.topics.txt"; then
        return 0
      fi
    fi
    if [ "${SECONDS}" -ge "${deadline}" ]; then
      printf 'Timed out waiting for Gazebo fleet topics to become available.\n' >&2
      return 1
    fi
    sleep 1
  done
}

wait_for_focus_topics() {
  local deadline=$((SECONDS + 30))
  while true; do
    if docker exec -u "$(id -u):$(id -g)" -e "GZ_PARTITION=${GZ_PARTITION}" "${STACK_NAME}-visual-sim" \
      bash -lc "timeout 10 gz topic -l" >"${run_diagnostics_dir}/gz.topics.txt" 2>/dev/null; then
      if grep -Fxq "${VISUAL_ACTIVE_CHASE_TOPIC}" "${run_diagnostics_dir}/gz.topics.txt" && \
         grep -Fxq "${VISUAL_ACTIVE_CAMERA_TOPIC}" "${run_diagnostics_dir}/gz.topics.txt" && \
         grep -Fxq "${VISUAL_FOCUS_STATE_TOPIC}" "${run_diagnostics_dir}/gz.topics.txt"; then
        return 0
      fi
    fi
    if [ "${SECONDS}" -ge "${deadline}" ]; then
      printf 'Timed out waiting for fleet focus-router topics to become available.\n' >&2
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

verify_visual_deployment() {
  local pose_file="${run_diagnostics_dir}/pose.info.txt"
  capture_pose_snapshot "${pose_file}"
  python3 "${ROOT_DIR}/scripts/verify_visual_fleet_runtime.py" \
    --manifest "${visual_assets_dir}/manifest.json" \
    --pose-info "${pose_file}" \
    --report "${run_diagnostics_dir}/deployment.verify.json"
}

write_run_manifest() {
  python3 - \
    "${visual_assets_dir}/manifest.json" \
    "${requested_fleet_config_copy}" \
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
requested_path = Path(sys.argv[2])
out_path = Path(sys.argv[3])
run_dir = sys.argv[4]
image = sys.argv[5]
visual_image = sys.argv[6]
nemala_tools = sys.argv[7]
network = sys.argv[8]
volume = sys.argv[9]
ros_domain_id = sys.argv[10]
gz_partition = sys.argv[11]

manifest = json.loads(manifest_path.read_text())
requested = json.loads(requested_path.read_text())

def image_id(name: str) -> str:
    try:
        return subprocess.check_output([
            'docker', 'image', 'inspect', '--format', '{{.Id}}', name
        ], text=True).strip()
    except Exception:
        return ''

manifest.update({
    'run_dir': run_dir,
    'requested_fleet_config': requested,
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

create_ipc_directories() {
  local cmd="mkdir -p /tmp/nemala"
  while IFS=$'\t' read -r drone_id _rest; do
    [ -n "${drone_id}" ] || continue
    cmd+=" /tmp/nemala/${drone_id}"
  done <"${visual_assets_dir}/drones.tsv"
  docker run --rm -v "${VOLUME}:/tmp/nemala" "${IMAGE}" bash -lc "${cmd}"
}

wait_for_drone_tcp_port() {
  local sitl_host="$1"
  local port="$2"
  local timeout_sec="${3:-45}"
  local tcp_probe

  tcp_probe="$({
    cat <<'PY'
import socket
import sys
import time

host = sys.argv[1]
port = int(sys.argv[2])
deadline = time.time() + float(sys.argv[3])
last_error = None

while time.time() < deadline:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2.0)
    try:
        sock.connect((host, port))
        sock.close()
        print(port, flush=True)
        raise SystemExit(0)
    except Exception as exc:
        last_error = repr(exc)
    finally:
        try:
            sock.close()
        except Exception:
            pass
    time.sleep(1.0)

raise SystemExit(
    f"Timed out waiting for TCP endpoint {host}:{port}. Last error: {last_error}"
)
PY
  })"

  docker run --rm \
    --network "${NETWORK}" \
    --entrypoint python3 \
    "${IMAGE}" \
    -c "${tcp_probe}" \
    "${sitl_host}" \
    "${port}" \
    "${timeout_sec}" >/dev/null
}

run_pre_hlc_mavlink_probe() {
  local drone_id="$1"
  local sitl_host="$2"
  local mavlink_aux_port="$3"
  local timeout_sec="${4:-60}"
  local report_mount="${current_run_mount}/diagnostics/readiness/${drone_id}.pre_hlc_endpoint.json"

  docker run --rm \
    --network "${NETWORK}" \
    -v "${ROOT_DIR}:/workspace/root:ro" \
    -v "${current_run_dir}:${current_run_mount}:rw" \
    --entrypoint python3 \
    "${IMAGE}" \
    /workspace/root/scripts/sitl_endpoint_readiness.py \
    --endpoint "tcp:${sitl_host}:${mavlink_aux_port}" \
    --timeout-sec "${timeout_sec}" \
    --min-heartbeats 3 \
    --max-gap-sec 2.0 \
    --settle-window-sec 3.0 \
    --report "${report_mount}"
}

pre_hlc_probe_failure_reason() {
  local report_path="$1"
  python3 - "$report_path" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
if not path.exists():
    print("")
    raise SystemExit(0)
try:
    payload = json.loads(path.read_text())
except Exception:
    print("unparseable_report")
    raise SystemExit(0)
print(str(payload.get("failure_reason") or ""))
PY
}

should_tolerate_pre_hlc_probe_failure() {
  local report_path="$1"
  local reason
  reason="$(pre_hlc_probe_failure_reason "${report_path}")"
  case "${reason}" in
    no_heartbeat|insufficient_consecutive_heartbeats|heartbeat_never_stabilized)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

write_drone_startup_failure_report() {
  local drone_id="$1"
  local stage="$2"
  local endpoint_report_path="${3:-}"
  local readiness_report_path="${4:-}"
  local report_path="${run_readiness_dir}/${drone_id}.startup_failure.json"
  local cmd=(
    python3
    "${ROOT_DIR}/scripts/fleet_startup_report.py"
    --run-dir "${current_run_dir}"
    --drone-id "${drone_id}"
    --stage "${stage}"
    --report "${report_path}"
  )
  if [ -n "${endpoint_report_path}" ]; then
    cmd+=( --endpoint-report "${endpoint_report_path}" )
  fi
  if [ -n "${readiness_report_path}" ]; then
    cmd+=( --readiness-report "${readiness_report_path}" )
  fi
  "${cmd[@]}" >/dev/null
  printf 'Startup diagnostics for %s written to %s\n' "${drone_id}" "${report_path}" >&2
}

set_runtime_master_port() {
  local config_path="$1"
  local sitl_host="$2"
  local resolved_port="$3"
  python3 - "$config_path" "$sitl_host" "$resolved_port" <<'PY'
import json
import sys
from pathlib import Path

config_path = Path(sys.argv[1])
sitl_host = sys.argv[2]
resolved_port = int(sys.argv[3])

data = json.loads(config_path.read_text())
data.setdefault("autopilot", {})["master"] = f"tcp:{sitl_host}:{resolved_port}"
config_path.write_text(json.dumps(data, indent=2) + "\n")
PY
}

set_manifest_mavlink_endpoint() {
  local manifest_path="$1"
  local drone_id="$2"
  local sitl_host="$3"
  local resolved_port="$4"
  python3 - "$manifest_path" "$drone_id" "$sitl_host" "$resolved_port" <<'PY'
import json
import sys
from pathlib import Path

manifest_path = Path(sys.argv[1])
drone_id = sys.argv[2]
sitl_host = sys.argv[3]
resolved_port = int(sys.argv[4])

manifest = json.loads(manifest_path.read_text())
for drone in manifest.get("drones", []):
    if drone.get("id") == drone_id:
        drone["sitl_host"] = sitl_host
        drone["mavlink_port"] = resolved_port
        break
manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
PY
}

wait_for_drone_readiness() {
  local drone_id="$1"
  local sitl_host="$2"
  local mavlink_aux_port="$3"
  local timeout_sec="${4:-120}"
  local start_sec
  local hlc_log_file mc_log_file
  local readiness_report
  start_sec="$(date +%s)"
  hlc_log_file="${run_logs_dir}/${drone_id}/hlc.log"
  mc_log_file="${run_logs_dir}/${drone_id}/mc.log"
  readiness_report="${run_readiness_dir}/${drone_id}.json"

  while true; do
    if [ -f "${hlc_log_file}" ] && [ -f "${mc_log_file}" ] && \
      grep -q 'HLC entering state: Ready' "${hlc_log_file}" && \
      grep -q 'MissionControl entering state: Standby' "${mc_log_file}"; then
      break
    fi
    if [ $(( $(date +%s) - start_sec )) -ge "${timeout_sec}" ]; then
      printf 'Timed out waiting for readiness of %s.\n' "${drone_id}" >&2
      write_drone_startup_failure_report \
        "${drone_id}" \
        "hlc_mc_readiness" \
        "${run_readiness_dir}/${drone_id}.pre_hlc_endpoint.json"
      return 1
    fi
    sleep 1
  done

  local readiness_cmd=(
    docker run --rm
    --network "${NETWORK}"
    -v "${ROOT_DIR}:/workspace/root:ro"
    -v "${current_run_dir}:${current_run_mount}:rw"
    --entrypoint python3
    "${IMAGE}"
    /workspace/root/scripts/drone_readiness.py
    --timeout-sec 30
    --clear-prearm-window-sec 3
    --report "${current_run_mount}/diagnostics/readiness/${drone_id}.json"
  )
  if [ "${profile_enabled}" = '1' ]; then
    readiness_cmd+=(--input-dir "${current_run_mount}/diagnostics/profile/mavlink/${drone_id}")
  else
    readiness_cmd+=(--endpoint "tcp:${sitl_host}:${mavlink_aux_port}")
  fi
  if ! "${readiness_cmd[@]}"; then
    printf 'MAVLink readiness check failed for %s. See %s\n' "${drone_id}" "${readiness_report}" >&2
    write_drone_startup_failure_report \
      "${drone_id}" \
      "post_ready_mavlink" \
      "${run_readiness_dir}/${drone_id}.pre_hlc_endpoint.json" \
      "${readiness_report}"
    return 1
  fi
  return 0
}

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  "${ROOT_DIR}/scripts/build_ws.sh"
fi

if ! docker image inspect "${VISUAL_SIM_IMAGE}" >/dev/null 2>&1; then
  "${ROOT_DIR}/scripts/build_visual_stack.sh"
fi

existing_stack="$(running_stack_containers)"
if [ -n "${existing_stack}" ]; then
  log "Existing ${STACK_NAME} stack detected. Restarting it in experiment mode with fresh fleet assets."
  printf '%s\n' "${existing_stack}"
  "${ROOT_DIR}/scripts/stop_stack.sh"
fi

docker volume inspect "${VOLUME}" >/dev/null 2>&1 || docker volume create "${VOLUME}" >/dev/null
docker network inspect "${NETWORK}" >/dev/null 2>&1 || docker network create "${NETWORK}" >/dev/null

cp "${fleet_config_src}" "${requested_fleet_config_copy}"
cat >"${visual_nvidia_egl_vendor_host}" <<'EOF'
{
  "file_format_version": "1.0.0",
  "ICD": {
    "library_path": "libEGL_nvidia.so.0"
  }
}
EOF
install -m 755 "${ROOT_DIR}/sim/tools/visual_focus_router.py" "${visual_tools_dir}/visual_focus_router.py"
install -m 755 "${ROOT_DIR}/sim/tools/swarm_visual_latency_recorder.py" "${visual_tools_dir}/swarm_visual_latency_recorder.py"
python3 "${ROOT_DIR}/scripts/generate_visual_fleet_assets.py" \
  --fleet-config "${fleet_config_src}" \
  --run-id "${run_id}" \
  --output-dir "${visual_assets_dir}" \
  --source-model "${ROOT_DIR}/external/ardupilot_gazebo/models/iris_with_camera_calibration/model.sdf" \
  --world-template "${ROOT_DIR}/sim/worlds/mars_iris_fleet_experiment.sdf" \
  --root-dir "${ROOT_DIR}"

# shellcheck disable=SC1090
source "${visual_assets_dir}/manifest.env"
write_run_manifest
cp "${visual_assets_dir}/manifest.json" "${visual_manifest_file}"
cp "${visual_assets_dir}/manifest.env" "${visual_manifest_env_file}"
printf '%s\n' 'experiment' >"${visual_mode_file}"
printf '%s\n' "${current_run_dir}" >"${visual_current_run_file}"

capture_basic_diagnostics

visual_gui="${VISUAL_GUI:-1}"
headless_visual_routing="${VISUAL_HEADLESS_ACTIVE_STREAMS:-0}"
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

if [ "${profile_enabled}" = '1' ]; then
  visual_args+=(
    -e STACK_PROFILE=1
    -e "STACK_PROFILE_RUN_DIR=${current_run_mount}"
  )
fi

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
verify_visual_deployment

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

start_visual_focus_router=0
if [ "${visual_gui}" = '1' ] || [ "${headless_visual_routing}" = '1' ]; then
  start_visual_focus_router=1
  docker run -d --rm \
    --name "${STACK_NAME}-visual-focus-router" \
    --network "${NETWORK}" \
    -e "GZ_PARTITION=${GZ_PARTITION}" \
    -v "${current_run_dir}:${current_run_mount}:rw" \
    --entrypoint python3 \
    "${VISUAL_SIM_IMAGE}" \
    "${current_run_mount}/tools/visual_focus_router.py" "${current_run_mount}/visual_assets/focus_router.json"
  start_log_capture "${STACK_NAME}-visual-focus-router" "${run_logs_dir}/visual-focus-router.log"
fi

if [ "${profile_enabled}" != '1' ] && [ "${start_visual_focus_router}" = '1' ]; then
  docker run -d --rm \
    --name "${STACK_NAME}-visual-latency-recorder" \
    --network "${NETWORK}" \
    -e "GZ_PARTITION=${GZ_PARTITION}" \
    -v "${current_run_dir}:${current_run_mount}:rw" \
    --entrypoint python3 \
    "${VISUAL_SIM_IMAGE}" \
    "${current_run_mount}/tools/swarm_visual_latency_recorder.py" "${current_run_mount}/visual_assets/visual_latency_recorder.json"
  start_log_capture "${STACK_NAME}-visual-latency-recorder" "${run_logs_dir}/visual-latency-recorder.log"

  docker run -d --rm \
    --name "${STACK_NAME}-swarm-latency" \
    --network "${NETWORK}" \
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
    -e PYTHONUNBUFFERED=1 \
    -v "${WS_DIR}:/workspace/ws:rw" \
    -v "${current_run_dir}:${current_workspace_run_mount}:rw" \
    "${IMAGE}" \
    bash -lc "source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && python3 /workspace/ws/src/decision_agent/decision_agent/swarm_latency_recorder.py --manifest ${current_workspace_run_mount}/visual_assets/manifest.json --output-dir ${current_workspace_run_mount}/diagnostics/latency"
  start_log_capture "${STACK_NAME}-swarm-latency" "${run_logs_dir}/swarm-latency-recorder.log"
fi

visual_sim_ip="$({ docker inspect --format '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "${STACK_NAME}-visual-sim"; } )"
if [ -z "${visual_sim_ip}" ]; then
  printf 'Failed to determine the IP address of %s.\n' "${STACK_NAME}-visual-sim" >&2
  exit 1
fi

create_ipc_directories
start_profile_supervisor
start_profile_sidecars

resolved_sitl_endpoints_tsv="${run_readiness_dir}/resolved_sitl_endpoints.tsv"
: >"${resolved_sitl_endpoints_tsv}"

while IFS=$'\t' read -r drone_id namespace sitl_host runtime_model_name serial0_port mavlink_port mavlink_aux_port fdm_port_in fdm_port_out proxy_name proxy_sub proxy_pub proxy_log chase_topic camera_topic alate_rel ros_alate_rel ros_nemala_rel; do
  [ -n "${drone_id}" ] || continue
  drone_log_dir="${run_logs_dir}/${drone_id}"
  mkdir -p "${drone_log_dir}"

  docker run -d --rm \
    --name "${STACK_NAME}-${drone_id}-sitl" \
    --network "${NETWORK}" \
    --network-alias "${sitl_host}" \
    -v "${CONFIG_DIR}:/stack/config:ro" \
    --entrypoint /opt/ardupilot/build/sitl/bin/arducopter \
    "${VISUAL_SIM_IMAGE}" \
    -I"$(python3 -c 'import sys; print(int(sys.argv[1]) - 1)' "$(awk -v id="${drone_id}" '$1==id{print NR}' "${visual_assets_dir}/drones.tsv")")" \
    --model JSON \
    --sim-address "${visual_sim_ip}" \
    --sim-port-in "${fdm_port_out}" \
    --sim-port-out "${fdm_port_in}" \
    --serial0 "tcp:${serial0_port}" \
    --serial1 "tcp:${mavlink_port}" \
    --serial2 "tcp:${mavlink_aux_port}" \
    --defaults /opt/ardupilot/Tools/autotest/default_params/copter.parm,/opt/ardupilot/Tools/autotest/default_params/gazebo-iris.parm,/stack/config/ardupilot/visual-sitl.parm
  start_log_capture "${STACK_NAME}-${drone_id}-sitl" "${drone_log_dir}/sitl.log"

  sitl_runtime_host="$(
    docker inspect --format '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "${STACK_NAME}-${drone_id}-sitl"
  )"
  if [ -z "${sitl_runtime_host}" ]; then
    sitl_runtime_host="${sitl_host}"
  fi

  # Fleet mode uses SERIAL1 as the deterministic DroneKit endpoint.
  resolved_mavlink_port="${mavlink_port}"
  resolved_mavlink_aux_port="${mavlink_aux_port}"
  wait_for_drone_tcp_port "${sitl_runtime_host}" "${resolved_mavlink_port}" 60
  wait_for_drone_tcp_port "${sitl_runtime_host}" "${resolved_mavlink_aux_port}" 60
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "${drone_id}" \
    "${namespace}" \
    "${sitl_host}" \
    "${runtime_model_name}" \
    "${serial0_port}" \
    "${resolved_mavlink_port}" \
    "${resolved_mavlink_aux_port}" \
    "${fdm_port_in}" \
    "${fdm_port_out}" \
    "${proxy_name}" \
    "${proxy_sub}" \
    "${proxy_pub}" \
    "${proxy_log}" \
    "${chase_topic}" \
    "${camera_topic}" \
    "${alate_rel}" \
    "${ros_alate_rel}" \
    "${ros_nemala_rel}" \
    "${sitl_runtime_host}" \
    "${drone_log_dir}" >>"${resolved_sitl_endpoints_tsv}"
done <"${visual_assets_dir}/drones.tsv"

while IFS=$'\t' read -r drone_id namespace sitl_host runtime_model_name serial0_port resolved_mavlink_port resolved_mavlink_aux_port fdm_port_in fdm_port_out proxy_name proxy_sub proxy_pub proxy_log chase_topic camera_topic alate_rel ros_alate_rel ros_nemala_rel sitl_runtime_host drone_log_dir; do
  [ -n "${drone_id}" ] || continue

  if ! run_pre_hlc_mavlink_probe "${drone_id}" "${sitl_runtime_host}" "${resolved_mavlink_aux_port}" 15; then
    write_drone_startup_failure_report \
      "${drone_id}" \
      "pre_hlc_endpoint" \
      "${run_readiness_dir}/${drone_id}.pre_hlc_endpoint.json"
    if should_tolerate_pre_hlc_probe_failure "${run_readiness_dir}/${drone_id}.pre_hlc_endpoint.json"; then
      printf 'Pre-HLC MAVLink readiness probe did not stabilize for %s; continuing with HLC attach and relying on post-ready MAVLink validation.\n' "${drone_id}" >&2
    else
      printf 'Pre-HLC MAVLink readiness probe failed for %s.\n' "${drone_id}" >&2
      exit 1
    fi
  fi
  set_runtime_master_port "${current_run_dir}/visual_assets/${alate_rel}" "${sitl_runtime_host}" "${resolved_mavlink_port}"
  set_manifest_mavlink_endpoint "${visual_assets_dir}/manifest.json" "${drone_id}" "${sitl_runtime_host}" "${resolved_mavlink_port}"
  set_manifest_mavlink_endpoint "${current_run_dir}/run_manifest.json" "${drone_id}" "${sitl_runtime_host}" "${resolved_mavlink_port}"
  log "Using deterministic fleet MAVLink endpoint for ${drone_id}: tcp:${sitl_runtime_host}:${resolved_mavlink_port}"

  docker run -d --rm \
    --name "${STACK_NAME}-${drone_id}-proxy" \
    --network "${NETWORK}" \
    -v "${VOLUME}:/tmp/nemala" \
    -v "${current_run_dir}:${current_run_mount}:ro" \
    "${NEMALA_TOOLS_IMAGE}" \
    proxy "${proxy_name}" "${current_run_mount}/visual_assets/${alate_rel}"
  start_log_capture "${STACK_NAME}-${drone_id}-proxy" "${drone_log_dir}/proxy.log"

  docker run -d --rm \
    --name "${STACK_NAME}-${drone_id}-logger" \
    --network "${NETWORK}" \
    -v "${VOLUME}:/tmp/nemala" \
    -v "${LOG_DIR}:/stack/logs" \
    "${NEMALA_TOOLS_IMAGE}" \
    log "${proxy_log}"
  start_log_capture "${STACK_NAME}-${drone_id}-logger" "${drone_log_dir}/nemala-logger.log"

  docker run -d --rm \
    --name "${STACK_NAME}-${drone_id}-mc" \
    --network "${NETWORK}" \
    -w /opt/alate \
    -e PYTHONUNBUFFERED=1 \
    "${profile_env_args[@]}" \
    -v "${VOLUME}:/tmp/nemala" \
    -v "${current_run_dir}:${current_run_mount}:ro" \
    "${IMAGE}" \
    ./mc "${proxy_name}" "${current_run_mount}/visual_assets/${alate_rel}"
  start_log_capture "${STACK_NAME}-${drone_id}-mc" "${drone_log_dir}/mc.log"

  docker run -d --rm \
    --name "${STACK_NAME}-${drone_id}-hlc" \
    --network "${NETWORK}" \
    -w /opt/alate \
    -e PYTHONUNBUFFERED=1 \
    "${profile_env_args[@]}" \
    -v "${VOLUME}:/tmp/nemala" \
    -v "${current_run_dir}:${current_run_mount}:ro" \
    "${IMAGE}" \
    ./hlc "${proxy_name}" "${current_run_mount}/visual_assets/${alate_rel}"
  start_log_capture "${STACK_NAME}-${drone_id}-hlc" "${drone_log_dir}/hlc.log"

  docker run -d --rm \
    --name "${STACK_NAME}-${drone_id}-ros-alate" \
    --network "${NETWORK}" \
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
    -e RCUTILS_LOGGING_BUFFERED_STREAM=1 \
    -e PYTHONUNBUFFERED=1 \
    "${profile_env_args[@]}" \
    -v "${VOLUME}:/tmp/nemala" \
    -v "${current_run_dir}:${current_run_mount}:ro" \
    "${IMAGE}" \
    bash -lc "source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && ros2 run ros_alate adapter --ros-args -r __ns:=${namespace} --log-level debug --params-file ${current_run_mount}/visual_assets/${ros_alate_rel}"
  start_log_capture "${STACK_NAME}-${drone_id}-ros-alate" "${drone_log_dir}/ros-alate.log"

  docker run -d --rm \
    --name "${STACK_NAME}-${drone_id}-ros-nemala" \
    --network "${NETWORK}" \
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
    -e RCUTILS_LOGGING_BUFFERED_STREAM=1 \
    -e PYTHONUNBUFFERED=1 \
    "${profile_env_args[@]}" \
    -v "${VOLUME}:/tmp/nemala" \
    -v "${current_run_dir}:${current_run_mount}:ro" \
  "${IMAGE}" \
    bash -lc "source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && ros2 run ros_nemala node_manager --ros-args -r __ns:=${namespace} --log-level debug --params-file ${current_run_mount}/visual_assets/${ros_nemala_rel}"
  start_log_capture "${STACK_NAME}-${drone_id}-ros-nemala" "${drone_log_dir}/ros-nemala.log"

  # Stage the control slices one vehicle at a time. This keeps fleet bring-up
  # deterministic and avoids overloading the shared simulator before each drone
  # has fully reached its takeoff-capable ready state.
  wait_for_drone_readiness "${drone_id}" "${sitl_runtime_host}" "${resolved_mavlink_aux_port}" 240

done <"${resolved_sitl_endpoints_tsv}"
wait_for_focus_topics

capture_basic_diagnostics
capture_ros_diagnostics

if [ "${visual_gui}" = '1' ]; then
  "${ROOT_DIR}/scripts/ensure_visual_gui.sh"
fi

trap - ERR
printf 'Started %s visual stack on docker network %s\n' "${STACK_NAME}" "${NETWORK}"
printf 'Visual mode: experiment\n'
printf 'Run directory: %s\n' "${current_run_dir}"
printf 'Latency diagnostics: %s\n' "${run_latency_dir}"
if [ "${profile_enabled}" = '1' ]; then
  printf 'Profiling diagnostics: %s\n' "${profile_output_dir}"
fi
printf 'Fleet manifest: %s\n' "${fleet_config_src}"
printf 'Active drone: %s\n' "${VISUAL_ACTIVE_DRONE_ID}"
printf 'Drones: %s\n' "${VISUAL_FLEET_DRONE_IDS}"
if [ "${visual_gui}" = '1' ]; then
  printf 'Gazebo GUI was launched in %s and connected to the simulator server in %s.\n' "${STACK_NAME}-visual-gui" "${STACK_NAME}-visual-sim"
else
  printf 'Gazebo is running headlessly in %s.\n' "${STACK_NAME}-visual-sim"
  if [ "${start_visual_focus_router}" != '1' ]; then
    printf 'Headless swarm routing is disabled, so active camera streams stay unsubscribed during control validation.\n'
  fi
fi
if [ "${RUN_VISUAL_STACK_QUIET_HINTS:-0}" != '1' ]; then
  printf 'For keyboard flight control, run %s --drone %s\n' "${ROOT_DIR}/scripts/run_visual_teleop.sh" "${VISUAL_ACTIVE_DRONE_ID}"
  printf 'To switch the active camera views, run %s --drone <id>\n' "${ROOT_DIR}/scripts/set_visual_focus.sh"
fi
printf 'Open a dev shell with %s\n' "${ROOT_DIR}/scripts/dev_shell.sh"
