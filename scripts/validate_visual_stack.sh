#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

usage() {
  printf 'Usage: %s [--experiment|--calib] [--fleet PATH]\n' "$0" >&2
}

visual_mode="${VISUAL_MODE:-experiment}"
fleet_config_src="${CONFIG_DIR}/swarm/visual.swarm.json"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --calib)
      visual_mode="calib"
      ;;
    --experiment)
      visual_mode="experiment"
      ;;
    --fleet)
      shift
      if [ "$#" -eq 0 ]; then
        usage
        exit 1
      fi
      fleet_config_src="$1"
      ;;
    *)
      printf 'Unsupported argument: %s\n' "$1" >&2
      usage
      exit 1
      ;;
  esac
  shift
done

if [ "${visual_mode}" = 'calib' ] && [ "${fleet_config_src}" != "${CONFIG_DIR}/swarm/visual.swarm.json" ]; then
  printf 'Calibration validation is single-drone only in Phase 1.\n' >&2
  exit 1
fi

visual_runtime_state_dir="${LOG_DIR}/runtime"
visual_current_run_file="${visual_runtime_state_dir}/visual.current_run_dir"
visual_manifest_env_file="${visual_runtime_state_dir}/visual.manifest.env"

current_run_dir() {
  if [ -f "${visual_current_run_file}" ]; then
    cat "${visual_current_run_file}"
  fi
}

load_visual_manifest_env() {
  if [ ! -f "${visual_manifest_env_file}" ]; then
    printf 'Missing %s\n' "${visual_manifest_env_file}" >&2
    exit 1
  fi
  # shellcheck disable=SC1090
  source "${visual_manifest_env_file}"
}

count_pattern() {
  local file="$1"
  local pattern="$2"
  if [ ! -f "${file}" ]; then
    printf '0\n'
    return 0
  fi
  grep -c "${pattern}" "${file}" 2>/dev/null || true
}

wait_for_count_increment() {
  local file="$1"
  local pattern="$2"
  local before_count="$3"
  local timeout_sec="$4"
  local deadline=$((SECONDS + timeout_sec))
  while true; do
    local current_count
    current_count="$(count_pattern "${file}" "${pattern}")"
    if [ "${current_count}" -gt "${before_count}" ]; then
      return 0
    fi
    if [ "${SECONDS}" -ge "${deadline}" ]; then
      printf 'Timed out waiting for pattern %s in %s\n' "${pattern}" "${file}" >&2
      return 1
    fi
    sleep 1
  done
}

fleet_takeoff_progress_count() {
  local hlc_log_file="$1"
  local min_altitude_m="${2:-0.5}"
  python3 - "${hlc_log_file}" "${min_altitude_m}" <<'PY'
import re
import sys
from pathlib import Path

log_path = Path(sys.argv[1])
min_altitude = float(sys.argv[2])
if not log_path.is_file():
    print(0)
    raise SystemExit(0)

text = log_path.read_text()
airborne_count = text.count('HLC entering state: Airborne')
progress_count = airborne_count

status_re = re.compile(
    r'AutopilotDronekit status .*armed=(True|False).*altitude=([-+0-9.]+).*state=([A-Z_]+)'
)
for line in text.splitlines():
    match = status_re.search(line)
    if not match:
        continue
    armed = match.group(1) == 'True'
    altitude = float(match.group(2))
    state = match.group(3)
    if armed and state == 'ACTIVE' and altitude >= min_altitude:
        progress_count += 1

print(progress_count)
PY
}

wait_for_fleet_takeoff_progress() {
  local hlc_log_file="$1"
  local before_progress="$2"
  local timeout_sec="${3:-180}"
  local min_altitude_m="${4:-0.5}"
  local deadline=$((SECONDS + timeout_sec))
  while true; do
    local current_progress
    current_progress="$(fleet_takeoff_progress_count "${hlc_log_file}" "${min_altitude_m}")"
    if [ "${current_progress}" -gt "${before_progress}" ]; then
      return 0
    fi
    if [ "${SECONDS}" -ge "${deadline}" ]; then
      printf 'Timed out waiting for fleet takeoff progress in %s\n' "${hlc_log_file}" >&2
      return 1
    fi
    sleep 1
  done
}

publish_operator_command() {
  local drone_id="${1:-}"
  local op_command="$2"
  local ns_args=()
  if [ -n "${drone_id}" ]; then
    ns_args+=(--ros-args -r "__ns:=/${drone_id}")
  fi
  docker exec "${STACK_NAME}-decision-dev" bash -lc \
    "source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && timeout 15 ros2 run decision_agent command_publisher ${ns_args[*]} -- --op-command ${op_command} --duration-sec 2.5 --rate-hz 5.0 --wait-for-subscribers-sec 10.0 --discovery-settle-sec 0.6" >/dev/null
}

fleet_drone_rows() {
  local manifest_path="$1"
  python3 - "${manifest_path}" <<'PY'
import json
import sys
from pathlib import Path
manifest = json.loads(Path(sys.argv[1]).read_text())
for drone in manifest.get('drones', []):
    print('\t'.join([drone['id'], drone.get('namespace', ''), drone['camera_topics']['chase'], drone['camera_topics']['deployed']]))
PY
}

wait_for_single_readiness() {
  local timeout_sec="${1:-120}"
  local deadline=$((SECONDS + timeout_sec))
  while true; do
    local hlc_logs mc_logs
    hlc_logs="$(docker logs "${STACK_NAME}-hlc" 2>&1 || true)"
    mc_logs="$(docker logs "${STACK_NAME}-mc" 2>&1 || true)"
    if grep -q 'HLC entering state: Ready' <<<"${hlc_logs}" && grep -q 'MissionControl entering state: Standby' <<<"${mc_logs}"; then
      return 0
    fi
    if [ "${SECONDS}" -ge "${deadline}" ]; then
      printf 'Timed out waiting for single-drone readiness.\n' >&2
      return 1
    fi
    sleep 1
  done
}

wait_for_fleet_readiness() {
  local manifest_path="$1"
  local timeout_sec="${2:-120}"
  while IFS=$'\t' read -r drone_id _namespace _chase_topic _camera_topic; do
    [ -n "${drone_id}" ] || continue
    local hlc_log_file="${run_logs_dir}/${drone_id}/hlc.log"
    local mc_log_file="${run_logs_dir}/${drone_id}/mc.log"
    local deadline=$((SECONDS + timeout_sec))
    while true; do
      if [ -f "${hlc_log_file}" ] && [ -f "${mc_log_file}" ] && \
        grep -q 'HLC entering state: Ready' "${hlc_log_file}" && \
        grep -q 'MissionControl entering state: Standby' "${mc_log_file}"; then
        break
      fi
      if [ "${SECONDS}" -ge "${deadline}" ]; then
        printf 'Timed out waiting for readiness of %s.\n' "${drone_id}" >&2
        return 1
      fi
      sleep 1
    done
  done < <(fleet_drone_rows "${manifest_path}")
}

run_fleet_takeoff_land_smoke() {
  local manifest_path="$1"
  local fleet_takeoff_timeout_sec="${FLEET_TAKEOFF_TIMEOUT_SEC:-210}"
  # Shared-world fleet descents are materially slower than the single-drone
  # runtime, especially after the second vehicle has taken off. Keep enough
  # budget here to observe a full return to Ready/Standby instead of failing
  # a healthy but slow landing.
  local fleet_land_timeout_sec="${FLEET_LAND_TIMEOUT_SEC:-360}"
  local fleet_takeoff_altitude_m="${FLEET_TAKEOFF_ALTITUDE_M:-0.5}"
  mapfile -t drone_ids < <(python3 - "${manifest_path}" <<'PY'
import json
import sys
from pathlib import Path
manifest = json.loads(Path(sys.argv[1]).read_text())
for drone in manifest.get('drones', []):
    print(drone['id'])
PY
)

  for drone_id in "${drone_ids[@]}"; do
    [ -n "${drone_id}" ] || continue
    local hlc_log_file="${run_logs_dir}/${drone_id}/hlc.log"
    local mc_log_file="${run_logs_dir}/${drone_id}/mc.log"
    local before_progress before_ready before_standby
    before_progress="$(fleet_takeoff_progress_count "${hlc_log_file}" "${fleet_takeoff_altitude_m}")"
    before_ready="$(count_pattern "${hlc_log_file}" 'HLC entering state: Ready')"
    before_standby="$(count_pattern "${mc_log_file}" 'MissionControl entering state: Standby')"
    declare -A other_before_progress=()
    for other_id in "${drone_ids[@]}"; do
      [ "${other_id}" != "${drone_id}" ] || continue
      local other_hlc_log_file="${run_logs_dir}/${other_id}/hlc.log"
      other_before_progress["${other_id}"]="$(fleet_takeoff_progress_count "${other_hlc_log_file}" "${fleet_takeoff_altitude_m}")"
    done

    publish_operator_command "${drone_id}" takeoff
    wait_for_fleet_takeoff_progress "${hlc_log_file}" "${before_progress}" "${fleet_takeoff_timeout_sec}" "${fleet_takeoff_altitude_m}"

    for other_id in "${drone_ids[@]}"; do
      [ "${other_id}" != "${drone_id}" ] || continue
      local other_hlc_log_file="${run_logs_dir}/${other_id}/hlc.log"
      local other_after_progress
      other_after_progress="$(fleet_takeoff_progress_count "${other_hlc_log_file}" "${fleet_takeoff_altitude_m}")"
      if [ "${other_after_progress}" -gt "${other_before_progress["${other_id}"]}" ]; then
        printf 'Unexpected takeoff progress observed on %s while commanding %s.\n' "${other_id}" "${drone_id}" >&2
        return 1
      fi
    done

    publish_operator_command "${drone_id}" land
    wait_for_count_increment "${hlc_log_file}" 'HLC entering state: Ready' "${before_ready}" "${fleet_land_timeout_sec}"
    wait_for_count_increment "${mc_log_file}" 'MissionControl entering state: Standby' "${before_standby}" "${fleet_land_timeout_sec}"
  done
}

single_hlc_log="$(mktemp)"
single_mc_log="$(mktemp)"
visual_topics="$(mktemp)"
pose_log="$(mktemp)"
joint_state_log="$(mktemp)"
verify_log="$(mktemp)"

cleanup() {
  rm -f "${single_hlc_log}" "${single_mc_log}" "${visual_topics}" "${pose_log}" "${joint_state_log}" "${verify_log}"
}
trap cleanup EXIT

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  "${ROOT_DIR}/scripts/build_ws.sh"
fi

if ! docker image inspect "${VISUAL_SIM_IMAGE}" >/dev/null 2>&1; then
  "${ROOT_DIR}/scripts/build_visual_stack.sh"
fi

visual_gui="${VISUAL_GUI:-1}"
requested_args=()
if [ "${visual_mode}" = 'calib' ]; then
  requested_args+=(--calib)
else
  requested_args+=(--experiment --fleet "${fleet_config_src}")
fi

RUN_VISUAL_STACK_QUIET_HINTS=1 VISUAL_GUI="${visual_gui}" "${ROOT_DIR}/scripts/run_visual_stack.sh" "${requested_args[@]}"
load_visual_manifest_env

run_dir="$(current_run_dir)"
if [ -z "${run_dir}" ] || [ ! -d "${run_dir}" ]; then
  printf 'Missing current visual run directory.\n' >&2
  exit 1
fi
manifest_path="${run_dir}/visual_assets/manifest.json"
if [ ! -f "${manifest_path}" ]; then
  printf 'Missing runtime manifest: %s\n' "${manifest_path}" >&2
  exit 1
fi
run_logs_dir="${run_dir}/logs"
run_diagnostics_dir="${run_dir}/diagnostics"
mkdir -p "${run_diagnostics_dir}"

is_fleet="$(python3 - "${manifest_path}" <<'PY'
import json
import sys
from pathlib import Path
manifest = json.loads(Path(sys.argv[1]).read_text())
print('1' if manifest.get('fleet') else '0')
PY
)"

if [ "${is_fleet}" = '1' ]; then
  wait_for_fleet_readiness "${manifest_path}" 150
else
  wait_for_single_readiness 120
fi

docker exec "${STACK_NAME}-decision-dev" bash -lc \
  'source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && timeout 20 ros2 topic list -t' \
  >"${run_diagnostics_dir}/ros2.topics.txt" 2>&1 || true

docker exec -u "$(id -u):$(id -g)" -e "GZ_PARTITION=${GZ_PARTITION}" "${STACK_NAME}-visual-sim" \
  bash -lc 'timeout 10 gz topic -l' >"${visual_topics}"

grep -Fxq "${VISUAL_ACTIVE_CHASE_TOPIC:-${VISUAL_CHASE_TOPIC:-}}" "${visual_topics}"
grep -Fxq "${VISUAL_ACTIVE_CAMERA_TOPIC:-${VISUAL_CAMERA_TOPIC:-}}" "${visual_topics}"

if [ "${is_fleet}" = '1' ]; then
  grep -Fxq "${VISUAL_FOCUS_SELECT_TOPIC}" "${visual_topics}"
  grep -Fxq "${VISUAL_FOCUS_STATE_TOPIC}" "${visual_topics}"
  while IFS=$'\t' read -r drone_id _namespace chase_topic camera_topic; do
    [ -n "${drone_id}" ] || continue
    grep -Fxq "${chase_topic}" "${visual_topics}"
    grep -Fxq "${camera_topic}" "${visual_topics}"
    test -f "${run_logs_dir}/${drone_id}/sitl.log"
    test -f "${run_logs_dir}/${drone_id}/mc.log"
    test -f "${run_logs_dir}/${drone_id}/hlc.log"
    test -f "${run_logs_dir}/${drone_id}/ros-alate.log"
    test -f "${run_logs_dir}/${drone_id}/ros-nemala.log"
  done < <(fleet_drone_rows "${manifest_path}")
else
  if [ "${visual_mode}" = 'calib' ]; then
    grep -Fxq "/model/${VISUAL_RUNTIME_MODEL_NAME}/joint/deployed_camera_x_joint/0/cmd_pos" "${visual_topics}"
    grep -Fxq "/model/${VISUAL_RUNTIME_MODEL_NAME}/joint/deployed_camera_y_joint/0/cmd_pos" "${visual_topics}"
    grep -Fxq "/model/${VISUAL_RUNTIME_MODEL_NAME}/joint/deployed_camera_z_joint/0/cmd_pos" "${visual_topics}"
    grep -Fxq "/model/${VISUAL_RUNTIME_MODEL_NAME}/joint/deployed_camera_roll_joint/0/cmd_pos" "${visual_topics}"
    grep -Fxq "/model/${VISUAL_RUNTIME_MODEL_NAME}/joint/deployed_camera_pitch_joint/0/cmd_pos" "${visual_topics}"
    grep -Fxq "/model/${VISUAL_RUNTIME_MODEL_NAME}/joint/deployed_camera_yaw_joint/0/cmd_pos" "${visual_topics}"
  fi
  test -f "${run_logs_dir}/hlc.log"
  test -f "${run_logs_dir}/mc.log"
fi

docker exec -u "$(id -u):$(id -g)" -e "GZ_PARTITION=${GZ_PARTITION}" "${STACK_NAME}-visual-sim" \
  bash -lc "timeout 15 gz topic -e -n 1 -t /world/${VISUAL_RUNTIME_WORLD_NAME}/dynamic_pose/info" >"${pose_log}"

if [ "${is_fleet}" = '1' ]; then
  python3 "${ROOT_DIR}/scripts/verify_visual_fleet_runtime.py" \
    --manifest "${manifest_path}" \
    --pose-info "${pose_log}" \
    --report "${run_diagnostics_dir}/deployment.verify.json"
else
  docker exec -u "$(id -u):$(id -g)" -e "GZ_PARTITION=${GZ_PARTITION}" "${STACK_NAME}-visual-sim" \
    bash -lc "timeout 15 gz topic -e -n 1 -t /world/${VISUAL_RUNTIME_WORLD_NAME}/model/${VISUAL_RUNTIME_MODEL_NAME}/joint_state" >"${joint_state_log}"
  python3 "${ROOT_DIR}/scripts/verify_visual_runtime.py" \
    --manifest "${manifest_path}" \
    --pose-info "${pose_log}" \
    --joint-state "${joint_state_log}" \
    --report "${run_diagnostics_dir}/deployment.verify.json"
fi

test -f "${run_dir}/run_manifest.json"
test -f "${run_dir}/visual_assets/deployment.applied.json"
test -f "${run_dir}/logs/visual-sim-server.log"
if [ "${VISUAL_GUI:-1}" = '1' ]; then
  test -f "${run_dir}/logs/visual-sim-gui.log"
fi

if [ "${is_fleet}" = '1' ]; then
  run_fleet_takeoff_land_smoke "${manifest_path}"
  second_drone_id="$(python3 - "${manifest_path}" <<'PY'
import json
import sys
from pathlib import Path
manifest = json.loads(Path(sys.argv[1]).read_text())
drones = manifest.get('drones', [])
print(drones[1]['id'] if len(drones) > 1 else '')
PY
)"
  if [ -n "${second_drone_id}" ]; then
    "${ROOT_DIR}/scripts/set_visual_focus.sh" --fleet "${fleet_config_src}" --drone "${second_drone_id}"
  fi
else
  docker logs "${STACK_NAME}-hlc" >"${single_hlc_log}" 2>&1 || true
  docker logs "${STACK_NAME}-mc" >"${single_mc_log}" 2>&1 || true
  grep -q 'HLC entering state: Ready' "${single_hlc_log}"
  grep -q 'MissionControl entering state: Standby' "${single_mc_log}"
fi

printf 'Visual stack validation complete.\n'
printf 'Run directory: %s\n' "${run_dir}"
if [ "${is_fleet}" = '1' ]; then
  printf 'Fleet manifest: %s\n' "${fleet_config_src}"
fi
