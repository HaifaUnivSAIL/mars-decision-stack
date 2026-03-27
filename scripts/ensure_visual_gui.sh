#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

gui_home="/tmp/visual-home"
runtime_dir="/tmp/runtime-$(id -u)"
container_name="${STACK_NAME}-visual-sim"
run_mount="/stack/run"
gui_log="${run_mount}/logs/visual-sim-gui.log"

visual_gui_is_running() {
  docker exec "${container_name}" bash -lc "ps -eo args | grep -F 'gz sim -g -v4' | grep -v grep >/dev/null"
}

if ! docker ps --format '{{.Names}}' | grep -qx "${container_name}"; then
  printf 'Visual simulator container %s is not running.\n' "${container_name}" >&2
  exit 1
fi

if [ -z "${DISPLAY:-}" ]; then
  printf 'DISPLAY is not set. Export DISPLAY to launch the Gazebo GUI.\n' >&2
  exit 1
fi

gui_mount_present="$(
  docker inspect \
    --format '{{range .Mounts}}{{if eq .Destination "/tmp/.X11-unix"}}yes{{end}}{{end}}' \
    "${container_name}"
)"

if [ "${gui_mount_present}" != "yes" ]; then
  printf 'Container %s was started without X11 socket access. Recreate the visual stack with GUI access.\n' "${container_name}" >&2
  exit 2
fi

if visual_gui_is_running; then
  printf 'Gazebo GUI is already running in %s.\n' "${container_name}"
  exit 0
fi

docker exec -d \
  -u "$(id -u):$(id -g)" \
  -e "DISPLAY=${DISPLAY}" \
  -e "QT_X11_NO_MITSHM=1" \
  -e "HOME=${gui_home}" \
  -e "XDG_RUNTIME_DIR=${runtime_dir}" \
  -e "GZ_PARTITION=${GZ_PARTITION}" \
  "${container_name}" \
  bash -lc "mkdir -p '${gui_home}' '${runtime_dir}' '${run_mount}/logs' && chmod 700 '${runtime_dir}' && exec gz sim -g -v4 >'${gui_log}' 2>&1"

deadline=$((SECONDS + 15))
while true; do
  if visual_gui_is_running; then
    printf 'Gazebo GUI is running in %s.\n' "${container_name}"
    exit 0
  fi

  if [ "${SECONDS}" -ge "${deadline}" ]; then
    printf 'Timed out waiting for the Gazebo GUI client to stay up in %s.\n' "${container_name}" >&2
    docker exec "${container_name}" bash -lc "tail -n 80 '${gui_log}' 2>/dev/null || true" >&2 || true
    exit 1
  fi

  sleep 1
done
