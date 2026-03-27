#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

if [ "$#" -gt 1 ]; then
  printf 'Usage: %s [--calib|--experiment]\n' "$0" >&2
  exit 1
fi

visual_mode="${VISUAL_MODE:-experiment}"
case "${1:-}" in
  "")
    ;;
  --calib)
    visual_mode="calib"
    ;;
  --experiment)
    visual_mode="experiment"
    ;;
  *)
    printf 'Unsupported visual mode flag: %s\n' "${1}" >&2
    printf 'Usage: %s [--calib|--experiment]\n' "$0" >&2
    exit 1
    ;;
esac

visual_runtime_state_dir="${LOG_DIR}/runtime"
visual_mode_file="${visual_runtime_state_dir}/visual.mode"
visual_current_run_file="${visual_runtime_state_dir}/visual.current_run_dir"
visual_manifest_env_file="${visual_runtime_state_dir}/visual.manifest.env"

current_visual_mode() {
  if [ -f "${visual_mode_file}" ]; then
    cat "${visual_mode_file}"
  fi
}

current_run_dir() {
  if [ -f "${visual_current_run_file}" ]; then
    cat "${visual_current_run_file}"
  fi
}

load_visual_manifest() {
  if [ ! -f "${visual_manifest_env_file}" ]; then
    printf 'Missing %s\n' "${visual_manifest_env_file}" >&2
    exit 1
  fi
  # shellcheck disable=SC1090
  source "${visual_manifest_env_file}"
}

hlc_log="$(mktemp)"
mc_log="$(mktemp)"
visual_topics="$(mktemp)"
manual_log="$(mktemp)"
pose_log="$(mktemp)"
joint_state_log="$(mktemp)"
verify_log="$(mktemp)"

cleanup() {
  rm -f "${hlc_log}" "${mc_log}" "${visual_topics}" "${manual_log}" "${pose_log}" "${joint_state_log}" "${verify_log}"
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
  if [ "${visual_gui}" = '1' ] && [ -z "${DISPLAY:-}" ]; then
    printf 'DISPLAY is not set. Visual stack validation requires a desktop X display in the current visual workflow.\n' >&2
    exit 1
  fi

  if [ "${visual_mode}" = 'calib' ]; then
    VISUAL_GUI="${visual_gui}" "${ROOT_DIR}/scripts/run_visual_stack.sh" --calib
  else
    VISUAL_GUI="${visual_gui}" "${ROOT_DIR}/scripts/run_visual_stack.sh"
  fi
else
  if [ "$(current_visual_mode)" != "${visual_mode}" ]; then
    visual_gui="${VISUAL_GUI:-1}"
    "${ROOT_DIR}/scripts/stop_stack.sh"
    if [ "${visual_mode}" = 'calib' ]; then
      VISUAL_GUI="${visual_gui}" "${ROOT_DIR}/scripts/run_visual_stack.sh" --calib
    else
      VISUAL_GUI="${visual_gui}" "${ROOT_DIR}/scripts/run_visual_stack.sh"
    fi
  elif [ "${VISUAL_GUI:-1}" = '1' ]; then
    if ! "${ROOT_DIR}/scripts/ensure_visual_gui.sh"; then
      ensure_rc=$?
      if [ "${ensure_rc}" -ne 2 ]; then
        exit "${ensure_rc}"
      fi
      "${ROOT_DIR}/scripts/stop_stack.sh"
      if [ "${visual_mode}" = 'calib' ]; then
        VISUAL_GUI=1 "${ROOT_DIR}/scripts/run_visual_stack.sh" --calib
      else
        VISUAL_GUI=1 "${ROOT_DIR}/scripts/run_visual_stack.sh"
      fi
    fi
  fi
fi

load_visual_manifest
run_dir="$(current_run_dir)"
if [ -z "${run_dir}" ] || [ ! -d "${run_dir}" ]; then
  printf 'Missing current visual run directory.\n' >&2
  exit 1
fi
run_diagnostics_dir="${run_dir}/diagnostics"
mkdir -p "${run_diagnostics_dir}"

deadline=$((SECONDS + 90))
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

cp "${hlc_log}" "${run_diagnostics_dir}/hlc.readiness.log"
cp "${mc_log}" "${run_diagnostics_dir}/mc.readiness.log"

docker exec "${STACK_NAME}-decision-dev" bash -lc \
  'source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && timeout 20 ros2 topic list -t' \
  >"${run_diagnostics_dir}/ros2.topics.txt" 2>&1 || true

docker exec -u "$(id -u):$(id -g)" -e "GZ_PARTITION=${GZ_PARTITION}" "${STACK_NAME}-visual-sim" \
  bash -lc 'timeout 10 gz topic -l' >"${visual_topics}"
grep -Fxq "${VISUAL_CHASE_TOPIC}" "${visual_topics}"
grep -Fxq "${VISUAL_CAMERA_TOPIC}" "${visual_topics}"

if [ "${visual_mode}" = 'calib' ]; then
  grep -Fxq "/model/${VISUAL_RUNTIME_MODEL_NAME}/joint/deployed_camera_x_joint/0/cmd_pos" "${visual_topics}"
  grep -Fxq "/model/${VISUAL_RUNTIME_MODEL_NAME}/joint/deployed_camera_y_joint/0/cmd_pos" "${visual_topics}"
  grep -Fxq "/model/${VISUAL_RUNTIME_MODEL_NAME}/joint/deployed_camera_z_joint/0/cmd_pos" "${visual_topics}"
  grep -Fxq "/model/${VISUAL_RUNTIME_MODEL_NAME}/joint/deployed_camera_roll_joint/0/cmd_pos" "${visual_topics}"
  grep -Fxq "/model/${VISUAL_RUNTIME_MODEL_NAME}/joint/deployed_camera_pitch_joint/0/cmd_pos" "${visual_topics}"
  grep -Fxq "/model/${VISUAL_RUNTIME_MODEL_NAME}/joint/deployed_camera_yaw_joint/0/cmd_pos" "${visual_topics}"
fi

docker exec -u "$(id -u):$(id -g)" -e "GZ_PARTITION=${GZ_PARTITION}" "${STACK_NAME}-visual-sim" \
  bash -lc "timeout 15 gz topic -e -n 1 -t /world/${VISUAL_RUNTIME_WORLD_NAME}/dynamic_pose/info" >"${pose_log}"

verify_args=(
  --manifest "${run_dir}/visual_assets/manifest.json"
  --pose-info "${pose_log}"
  --joint-state "${joint_state_log}"
  --report "${verify_log}"
)

docker exec -u "$(id -u):$(id -g)" -e "GZ_PARTITION=${GZ_PARTITION}" "${STACK_NAME}-visual-sim" \
  bash -lc "timeout 15 gz topic -e -n 1 -t /world/${VISUAL_RUNTIME_WORLD_NAME}/model/${VISUAL_RUNTIME_MODEL_NAME}/joint_state" >"${joint_state_log}"

python3 "${ROOT_DIR}/scripts/verify_visual_runtime.py" "${verify_args[@]}"

test -f "${run_dir}/run_manifest.json"
test -f "${run_dir}/visual_assets/deployment.applied.json"
test -f "${run_dir}/logs/visual-sim-server.log"
test -f "${run_dir}/logs/hlc.log"
test -f "${run_dir}/logs/mc.log"
if [ "${VISUAL_GUI:-1}" = '1' ]; then
  test -f "${run_dir}/logs/visual-sim-gui.log"
fi

docker exec "${STACK_NAME}-decision-dev" bash -lc 'source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && timeout 12 ros2 run manual_runtime_test keyboard_teleop --ros-args --params-file /workspace/config/manual_runtime_test/scripted_smoke_test.yaml' >"${manual_log}" 2>&1 || true
grep -q 'Published motion command forward' "${manual_log}"
grep -q 'Published motion command left' "${manual_log}"
grep -q 'Published motion command right' "${manual_log}"
grep -q 'Published motion command back' "${manual_log}"

printf 'Visual stack validation complete.\n'
printf 'Run directory: %s\n' "${run_dir}"
