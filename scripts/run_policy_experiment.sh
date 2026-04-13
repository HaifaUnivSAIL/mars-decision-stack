#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

usage() {
  printf 'Usage: %s [--scenario NAME|--params-file PATH] [--calib|--experiment] [--fleet PATH] [--drone ID] [--profile]\n' "$0" >&2
}

visual_mode="${VISUAL_MODE:-experiment}"
scenario_name="axis_sweep"
scenario_params_file=""
fleet_args=()
drone_id=""
profile_enabled=0

while [ "$#" -gt 0 ]; do
  case "$1" in
    --scenario)
      shift
      if [ "$#" -eq 0 ]; then
        usage
        exit 1
      fi
      scenario_name="$1"
      ;;
    --params-file)
      shift
      if [ "$#" -eq 0 ]; then
        usage
        exit 1
      fi
      scenario_params_file="$1"
      ;;
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
      fleet_args+=(--fleet "$1")
      ;;
    --drone)
      shift
      if [ "$#" -eq 0 ]; then
        usage
        exit 1
      fi
      drone_id="$1"
      ;;
    --profile)
      profile_enabled=1
      ;;
    *)
      printf 'Unsupported argument: %s\n' "$1" >&2
      usage
      exit 1
      ;;
  esac
  shift
done

if [ "${visual_mode}" = "calib" ] && [ "${#fleet_args[@]}" -gt 0 ]; then
  printf 'Fleet experiments currently support experiment mode only.\n' >&2
  exit 1
fi

if [ -z "${scenario_params_file}" ]; then
  scenario_params_file="${CONFIG_DIR}/decision_agent/scenarios/${scenario_name}.yaml"
fi

if [ ! -f "${scenario_params_file}" ]; then
  printf 'Scenario params file not found: %s\n' "${scenario_params_file}" >&2
  exit 1
fi

estimate_json="$(
python3 - "${scenario_params_file}" <<'PY'
import json
import sys
from pathlib import Path

import yaml

path = Path(sys.argv[1])
data = yaml.safe_load(path.read_text())
node_params = (
    data.get('policy_scenario', {}).get('ros__parameters')
    or data.get('/policy_scenario', {}).get('ros__parameters')
    or {}
)
actions = node_params.get('scripted_actions', [])

def action_duration(action_text: str) -> float:
    raw = str(action_text).strip()
    if ':' not in raw:
        return 0.0
    kind, payload = raw.split(':', 1)
    kind = kind.strip().lower()
    payload = payload.strip()
    if kind in ('wait', 'stop'):
        return float(payload)
    if kind == 'velocity':
        parts = [part.strip() for part in payload.split(',')]
        if len(parts) != 4:
            raise SystemExit(f'Invalid velocity action in {path}: {raw}')
        return float(parts[3])
    raise SystemExit(f'Unsupported action in {path}: {raw}')

action_total = sum(action_duration(action) for action in actions)
result = {
    'scenario_name': node_params.get('scenario_name', path.stem),
    'action_total_sec': action_total,
    'auto_takeoff': bool(node_params.get('auto_takeoff', True)),
    'takeoff_timeout_sec': float(node_params.get('takeoff_timeout_sec', 30.0)),
    'land_after_actions': bool(node_params.get('land_after_actions', True)),
    'landing_timeout_sec': float(node_params.get('landing_timeout_sec', 30.0)),
    'stop_after_actions_sec': float(node_params.get('stop_after_actions_sec', 1.0)),
}
result['capture_duration_sec'] = (
    2.0
    + (result['takeoff_timeout_sec'] if result['auto_takeoff'] else 0.0)
    + result['action_total_sec']
    + result['stop_after_actions_sec']
    + (result['landing_timeout_sec'] if result['land_after_actions'] else 0.0)
    + 3.0
)
print(json.dumps(result))
PY
)"

scenario_name="$(python3 -c 'import json,sys; print(json.loads(sys.argv[1])["scenario_name"])' "${estimate_json}")"
capture_duration_sec="$(python3 -c 'import json,sys; print(json.loads(sys.argv[1])["capture_duration_sec"])' "${estimate_json}")"

requested_visual_arg="--experiment"
if [ "${visual_mode}" = "calib" ]; then
  requested_visual_arg="--calib"
fi

profile_args=()
if [ "${profile_enabled}" = '1' ]; then
  profile_args+=(--profile)
fi

RUN_VISUAL_STACK_QUIET_HINTS=1 VISUAL_GUI="${VISUAL_GUI:-0}" "${ROOT_DIR}/scripts/run_visual_stack.sh" "${requested_visual_arg}" "${fleet_args[@]}" "${profile_args[@]}"

run_dir_file="${LOG_DIR}/runtime/visual.current_run_dir"
if [ ! -f "${run_dir_file}" ]; then
  printf 'Missing %s after visual stack start.\n' "${run_dir_file}" >&2
  exit 1
fi

current_run_dir="$(<"${run_dir_file}")"
if [ ! -d "${current_run_dir}" ]; then
  printf 'Current run directory does not exist: %s\n' "${current_run_dir}" >&2
  exit 1
fi

manifest_path="${current_run_dir}/visual_assets/manifest.json"
if [ ! -f "${manifest_path}" ]; then
  printf 'Missing runtime manifest: %s\n' "${manifest_path}" >&2
  exit 1
fi

selected_drone_id="$(
python3 - "${manifest_path}" "${drone_id}" <<'PY'
import json
import sys
from pathlib import Path

manifest = json.loads(Path(sys.argv[1]).read_text())
requested = sys.argv[2].strip()
if manifest.get('fleet'):
    drones = manifest.get('drones', [])
    drone_ids = [drone['id'] for drone in drones]
    if requested:
        if requested not in drone_ids:
            raise SystemExit(f'Unknown drone id {requested!r}. Available: {drone_ids}')
        print(requested)
    else:
        print(manifest.get('active_drone_id') or drone_ids[0])
else:
    print(requested)
PY
)"

experiment_name="policy_experiment_${scenario_name}_$(date +%Y%m%d-%H%M%S)"
analysis_dir="${current_run_dir}/analysis/${experiment_name}"
mkdir -p "${analysis_dir}"
scenario_params_copy="${analysis_dir}/scenario.params.yaml"
cp "${scenario_params_file}" "${scenario_params_copy}"
printf '%s\n' "${estimate_json}" >"${analysis_dir}/scenario.estimate.json"

scenario_params_runtime_host="${analysis_dir}/scenario.params.runtime.yaml"
if [ -n "${selected_drone_id}" ]; then
  python3 "${ROOT_DIR}/scripts/render_namespaced_ros_params.py" \
    --source "${scenario_params_copy}" \
    --node-name policy_scenario \
    --namespace "/${selected_drone_id}" \
    --output "${scenario_params_runtime_host}"
else
  cp "${scenario_params_copy}" "${scenario_params_runtime_host}"
fi
scenario_params_mount="/workspace/run/analysis/${experiment_name}/scenario.params.runtime.yaml"

scenario_log="${analysis_dir}/scenario_node.log"
recorder_log="${analysis_dir}/recorder.log"
recorder_dir="${analysis_dir}/recorder"

printf '[run_policy_experiment] Visual mode: %s\n' "${visual_mode}"
printf '[run_policy_experiment] Scenario: %s\n' "${scenario_name}"
printf '[run_policy_experiment] Capture duration: %.1fs\n' "${capture_duration_sec}"
printf '[run_policy_experiment] Analysis directory: %s\n' "${analysis_dir}"
if [ -n "${selected_drone_id}" ]; then
  printf '[run_policy_experiment] Selected drone: %s\n' "${selected_drone_id}"
fi

recorder_cmd=(
  python3 "${ROOT_DIR}/scripts/compare_policy_input_to_imu.py"
  --duration "${capture_duration_sec}"
  --output-dir "${recorder_dir}"
)
if [ -n "${selected_drone_id}" ]; then
  recorder_cmd+=( --drone "${selected_drone_id}" )
fi

"${recorder_cmd[@]}" >"${recorder_log}" 2>&1 &
recorder_pid=$!

cleanup() {
  local rc=$?
  if kill -0 "${recorder_pid}" >/dev/null 2>&1; then
    kill "${recorder_pid}" >/dev/null 2>&1 || true
    wait "${recorder_pid}" >/dev/null 2>&1 || true
  fi
  if [ "${rc}" -ne 0 ]; then
    printf 'Policy experiment failed. Analysis directory: %s\n' "${analysis_dir}" >&2
  fi
  return "${rc}"
}
trap cleanup EXIT

docker exec "${STACK_NAME}-decision-dev" bash -lc "
  set -euo pipefail
  set +u
  source /opt/ros/humble/setup.bash
  if [ -f /workspace/ws/install/setup.bash ]; then
    source /workspace/ws/install/setup.bash
  else
    source /opt/ros2_ws/install/setup.bash
  fi
  set -u
  cmd=(ros2 run decision_agent scenario_node --ros-args --params-file '${scenario_params_mount}')
  if [ -n '${selected_drone_id}' ]; then
    cmd+=( -r '__ns:=/${selected_drone_id}' )
  fi
  timeout $(python3 -c 'import json,sys; print(int(float(json.loads(sys.argv[1])["capture_duration_sec"]) + 5))' "${estimate_json}") \
    \"\${cmd[@]}\"
" >"${scenario_log}" 2>&1

wait "${recorder_pid}"
trap - EXIT

python3 - "${analysis_dir}" <<'PY'
import csv
import json
import sys
from pathlib import Path

analysis_dir = Path(sys.argv[1])
recorder_dir = analysis_dir / 'recorder'
summary_path = recorder_dir / 'summary.json'
comparison_path = recorder_dir / 'comparison.csv'
result = {
    'analysis_dir': str(analysis_dir),
    'recorder_dir': str(recorder_dir),
    'summary': json.loads(summary_path.read_text()) if summary_path.exists() else {},
    'windows': [],
}
if comparison_path.exists():
    with comparison_path.open() as handle:
        reader = csv.DictReader(handle)
        result['windows'] = list(reader)
(analysis_dir / 'experiment.report.json').write_text(json.dumps(result, indent=2) + '\n')
if not result['windows']:
    raise SystemExit('No active command windows were recorded during the policy experiment.')
PY

printf '[run_policy_experiment] Scenario completed successfully.\n'
printf '[run_policy_experiment] Recorder directory: %s\n' "${recorder_dir}"
printf '[run_policy_experiment] Scenario log: %s\n' "${scenario_log}"
