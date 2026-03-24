#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

hlc_log="$(mktemp)"
mc_log="$(mktemp)"
visual_topics="$(mktemp)"
manual_log="$(mktemp)"

cleanup() {
  rm -f "${hlc_log}" "${mc_log}" "${visual_topics}" "${manual_log}"
}
trap cleanup EXIT

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  "${ROOT_DIR}/scripts/build_ws.sh"
fi

if ! docker image inspect "${VISUAL_SIM_IMAGE}" >/dev/null 2>&1; then
  "${ROOT_DIR}/scripts/build_visual_stack.sh"
fi

if ! docker ps --format '{{.Names}}' | grep -qx "${STACK_NAME}-visual-sim"; then
  visual_gui="${VISUAL_GUI:-1}"
  if [ "${visual_gui}" = "1" ] && [ -z "${DISPLAY:-}" ]; then
    printf 'DISPLAY is not set. Visual stack validation requires a desktop X display in the current visual workflow.\n' >&2
    exit 1
  fi

  VISUAL_GUI="${visual_gui}" "${ROOT_DIR}/scripts/run_visual_stack.sh"
fi

deadline=$((SECONDS + 60))
while true; do
  docker ps --format '{{.Names}} {{.Status}}' | grep "^${STACK_NAME}" | sort || true
  docker logs "${STACK_NAME}-hlc" >"${hlc_log}" 2>&1 || true
  docker logs "${STACK_NAME}-mc" >"${mc_log}" 2>&1 || true

  if grep -q 'HLC entering state: Ready' "${hlc_log}" && \
     grep -q 'MissionControl entering state: Standby' "${mc_log}"; then
    break
  fi

  if [ "${SECONDS}" -ge "${deadline}" ]; then
    printf 'Timed out waiting for visual stack readiness.\n' >&2
    exit 1
  fi

  sleep 2
done

docker exec \
  -u "$(id -u):$(id -g)" \
  -e "GZ_PARTITION=${GZ_PARTITION}" \
  "${STACK_NAME}-visual-sim" \
  bash -lc 'timeout 10 gz topic -l' >"${visual_topics}"
grep -Fxq '/mars/visual/chase_camera' "${visual_topics}"
grep -Fxq '/mars/visual/top_camera' "${visual_topics}"

docker exec "${STACK_NAME}-decision-dev" bash -lc 'source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && timeout 12 ros2 run manual_runtime_test keyboard_teleop --ros-args --params-file /workspace/config/manual_runtime_test/scripted_smoke_test.yaml' >"${manual_log}" 2>&1 || true
grep -q 'Published motion command forward' "${manual_log}"

printf 'Visual stack validation complete.\n'
