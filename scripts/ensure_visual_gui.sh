#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

gui_home="/tmp/visual-home"
runtime_dir="/tmp/runtime-$(id -u)"
server_container_name="${STACK_NAME}-visual-sim"
gui_container_name="${STACK_NAME}-visual-gui"
run_mount="/stack/run"
run_dir_file="${LOG_DIR}/runtime/visual.current_run_dir"

if [ ! -f "${run_dir_file}" ]; then
  printf 'Visual runtime state file %s is missing.\n' "${run_dir_file}" >&2
  exit 1
fi

current_run_dir="$(cat "${run_dir_file}")"
gui_log_host="${current_run_dir}/logs/visual-sim-gui.log"
gui_log_mount="${run_mount}/logs/visual-sim-gui.log"

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

append_gui_gpu_args() {
  local -n target_ref="$1"
  local gid

  if [ -e /dev/nvidia0 ] && has_nvidia_runtime; then
    target_ref+=(
      --gpus all
      -e 'NVIDIA_VISIBLE_DEVICES=all'
      -e 'NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility'
      -e '__NV_PRIME_RENDER_OFFLOAD=1'
      -e '__GLX_VENDOR_LIBRARY_NAME=nvidia'
      -e 'LIBGL_ALWAYS_SOFTWARE=0'
      -e 'QT_XCB_GL_INTEGRATION=xcb_glx'
      -e 'QT_OPENGL=desktop'
    )
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

gui_container_is_running() {
  docker ps --format '{{.Names}}' | grep -qx "${gui_container_name}"
}

if ! docker ps --format '{{.Names}}' | grep -qx "${server_container_name}"; then
  printf 'Visual simulator container %s is not running.\n' "${server_container_name}" >&2
  exit 1
fi

if [ -z "${DISPLAY:-}" ]; then
  printf 'DISPLAY is not set. Export DISPLAY to launch the Gazebo GUI.\n' >&2
  exit 1
fi

if ! [ -S /tmp/.X11-unix/X"${DISPLAY#:}" ] && ! [ -d /tmp/.X11-unix ]; then
  printf 'X11 socket directory /tmp/.X11-unix is not available on the host.\n' >&2
  exit 1
fi

if gui_container_is_running; then
  printf 'Gazebo GUI is already running in %s.\n' "${gui_container_name}"
  exit 0
fi

docker rm -f "${gui_container_name}" >/dev/null 2>&1 || true
mkdir -p "$(dirname "${gui_log_host}")"
: >"${gui_log_host}"

gui_args=(
  -d
  --rm
  --name "${gui_container_name}"
  --network "${NETWORK}"
  -u "$(id -u):$(id -g)"
  -e "DISPLAY=${DISPLAY}"
  -e "QT_X11_NO_MITSHM=1"
  -e "HOME=${gui_home}"
  -e "XDG_RUNTIME_DIR=${runtime_dir}"
  -e "GZ_PARTITION=${GZ_PARTITION}"
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  -v "${current_run_dir}:${run_mount}:rw"
)

if [ -n "${XAUTHORITY:-}" ] && [ -f "${XAUTHORITY}" ]; then
  gui_args+=( -e "XAUTHORITY=${XAUTHORITY}" -v "${XAUTHORITY}:${XAUTHORITY}:ro" )
elif [ -f "${HOME}/.Xauthority" ]; then
  gui_args+=( -e XAUTHORITY=/tmp/.Xauthority -v "${HOME}/.Xauthority:/tmp/.Xauthority:ro" )
fi

append_gui_gpu_args gui_args

docker run "${gui_args[@]}" \
  --entrypoint bash \
  "${VISUAL_SIM_IMAGE}" \
  -lc "mkdir -p '${gui_home}' '${runtime_dir}' '${run_mount}/logs' && chmod 700 '${runtime_dir}' && export GZ_SIM_RESOURCE_PATH='${run_mount}/visual_assets/models:\${GZ_SIM_RESOURCE_PATH:-}' && exec gz sim -g -v4 >'${gui_log_mount}' 2>&1" >/dev/null

deadline=$((SECONDS + 20))
stable_deadline=$((SECONDS + 3))
while true; do
  if gui_container_is_running; then
    if [ "${SECONDS}" -ge "${stable_deadline}" ]; then
      printf 'Gazebo GUI is running in %s.\n' "${gui_container_name}"
      exit 0
    fi
  else
    printf 'Gazebo GUI client exited shortly after startup.\n' >&2
    tail -n 120 "${gui_log_host}" >&2 || true
    exit 1
  fi

  if [ "${SECONDS}" -ge "${deadline}" ]; then
    printf 'Timed out waiting for the Gazebo GUI client to stabilize in %s.\n' "${gui_container_name}" >&2
    tail -n 120 "${gui_log_host}" >&2 || true
    exit 1
  fi

  sleep 1
done
