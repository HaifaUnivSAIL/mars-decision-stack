#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

usage() {
  cat <<EOF >&2
Usage: $0 [--scenario NAME|--scenario-file PATH] [--count N|--fleet PATH] [--runtime-profile NAME] [--headless] [--profile]

Examples:
  $0 --count 2 --profile
  $0 --count 3 --scenario formation_smoke --headless --profile
  $0 --fleet config/swarm/visual.swarm.json --scenario-file config/decision_agent/swarm_scenarios/formation_smoke.yaml
EOF
}

scenario_name="formation_smoke"
scenario_file=""
count=""
fleet_config=""
runtime_profile=""
visual_gui="${VISUAL_GUI:-1}"
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
    --scenario-file)
      shift
      if [ "$#" -eq 0 ]; then
        usage
        exit 1
      fi
      scenario_file="$1"
      ;;
    --count)
      shift
      if [ "$#" -eq 0 ]; then
        usage
        exit 1
      fi
      count="$1"
      ;;
    --fleet)
      shift
      if [ "$#" -eq 0 ]; then
        usage
        exit 1
      fi
      fleet_config="$1"
      ;;
    --runtime-profile)
      shift
      if [ "$#" -eq 0 ]; then
        usage
        exit 1
      fi
      runtime_profile="$1"
      ;;
    --headless)
      visual_gui=0
      ;;
    --gui)
      visual_gui=1
      ;;
    --profile)
      profile_enabled=1
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      printf 'Unsupported argument: %s\n' "$1" >&2
      usage
      exit 1
      ;;
  esac
  shift
done

if [ -n "${count}" ] && [ -n "${fleet_config}" ]; then
  printf 'Use either --count or --fleet, not both.\n' >&2
  exit 1
fi

if [ -n "${runtime_profile}" ] && [ -n "${fleet_config}" ]; then
  printf 'Use --runtime-profile only with --count. Fleet manifests should carry their own runtime profile.\n' >&2
  exit 1
fi

if [ -n "${count}" ]; then
  if ! [[ "${count}" =~ ^[0-9]+$ ]] || [ "${count}" -lt 1 ]; then
    printf 'Drone count must be a positive integer. Got: %s\n' "${count}" >&2
    exit 1
  fi
fi

if [ -z "${runtime_profile}" ] && [ -n "${count}" ]; then
  runtime_profile="single_equivalent"
fi

if [ -z "${scenario_file}" ]; then
  scenario_file="${CONFIG_DIR}/decision_agent/swarm_scenarios/${scenario_name}.yaml"
fi

if [ ! -f "${scenario_file}" ]; then
  printf 'Swarm scenario file not found: %s\n' "${scenario_file}" >&2
  exit 1
fi

generate_manifest_from_count() {
  local output_path="$1"
  local requested_count="$2"
  local requested_runtime_profile="$3"
  python3 - "$output_path" "$requested_count" "$requested_runtime_profile" <<'PY'
import json
import sys
from pathlib import Path

output_path = Path(sys.argv[1])
count = int(sys.argv[2])
runtime_profile = sys.argv[3]
alate_profile = (
    "config/alate/uav.visual.fleet.sitl.json"
    if count > 1
    else "config/alate/uav.visual.sitl.json"
)

manifest = {
    "active_drone_id": "drone_1",
    "defaults": {
        "camera_deployment_config": "config/visual/camera.deployment.json",
        "alate_profile": alate_profile,
        "ros_alate_profile": "config/ros_alate/adapter.yaml",
        "ros_nemala_profile": "config/ros_nemala/node_manager.yaml",
        "runtime_profile": runtime_profile,
    },
    "drones": [],
}

for index in range(count):
    manifest["drones"].append(
        {
            "id": f"drone_{index + 1}",
            "spawn": {
                "x": 0.0,
                "y": float(index) * 4.0,
                "z": 0.195,
                "yaw_deg": 90.0,
            },
        }
    )

output_path.parent.mkdir(parents=True, exist_ok=True)
output_path.write_text(json.dumps(manifest, indent=2) + "\n")
PY
}

estimate_json="$(
python3 - "${scenario_file}" <<'PY'
import json
import sys
from pathlib import Path

import yaml

path = Path(sys.argv[1])
data = yaml.safe_load(path.read_text()) or {}
actions = list(data.get("actions") or [])

def action_duration(entry: dict) -> float:
    if len(entry) != 1:
        raise SystemExit(f"Invalid action entry in {path}: {entry}")
    kind, payload = next(iter(entry.items()))
    if kind == "hold":
        return float(payload)
    if kind in {"translate_world", "translate_body", "yaw"}:
        payload = payload or {}
        return float(payload.get("duration_sec", 0.0))
    if kind == "land":
        return 0.0
    raise SystemExit(f"Unsupported swarm action in {path}: {entry}")

action_total = sum(action_duration(action) for action in actions)
result = {
    "scenario_name": str(data.get("scenario_name", path.stem)),
    "action_total_sec": action_total,
    "auto_takeoff": bool(data.get("auto_takeoff", True)),
    "takeoff_timeout_sec": float(data.get("takeoff_timeout_sec", 45.0)),
    "auto_land_after_actions": bool(data.get("auto_land_after_actions", True)),
    "landing_timeout_sec": float(data.get("landing_timeout_sec", 45.0)),
    "ready_hold_sec": float(data.get("ready_hold_sec", 1.0)),
}
result["capture_duration_sec"] = (
    5.0
    + result["ready_hold_sec"]
    + (result["takeoff_timeout_sec"] if result["auto_takeoff"] else 0.0)
    + result["action_total_sec"]
    + (result["landing_timeout_sec"] if result["auto_land_after_actions"] else 5.0)
    + 5.0
)
print(json.dumps(result))
PY
)"

scenario_name="$(python3 -c 'import json,sys; print(json.loads(sys.argv[1])["scenario_name"])' "${estimate_json}")"
capture_duration_sec="$(python3 -c 'import json,sys; print(json.loads(sys.argv[1])["capture_duration_sec"])' "${estimate_json}")"

if [ -n "${count}" ]; then
  fleet_config="${LOG_DIR}/runtime/swarm_experiment.requested.json"
  generate_manifest_from_count "${fleet_config}" "${count}" "${runtime_profile}"
fi

profile_args=()
if [ "${profile_enabled}" = '1' ]; then
  profile_args+=(--profile)
fi

if [ -n "${fleet_config}" ]; then
  VISUAL_GUI="${visual_gui}" "${ROOT_DIR}/scripts/run_visual_stack.sh" --fleet "${fleet_config}" "${profile_args[@]}"
fi

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

experiment_name="swarm_experiment_${scenario_name}_$(date +%Y%m%d-%H%M%S)"
analysis_dir="${current_run_dir}/analysis/${experiment_name}"
mkdir -p "${analysis_dir}"
scenario_copy="${analysis_dir}/swarm.scenario.yaml"
cp "${scenario_file}" "${scenario_copy}"
printf '%s\n' "${estimate_json}" >"${analysis_dir}/scenario.estimate.json"
if [ -n "${fleet_config}" ]; then
  cp "${fleet_config}" "${analysis_dir}/requested.fleet.json"
fi

scenario_mount="/workspace/run/analysis/${experiment_name}/swarm.scenario.yaml"
manifest_mount="/workspace/run/visual_assets/manifest.json"
scenario_log="${analysis_dir}/swarm_scenario_node.log"

printf '[run_swarm_experiment] Scenario: %s\n' "${scenario_name}"
printf '[run_swarm_experiment] Capture duration: %.1fs\n' "${capture_duration_sec}"
printf '[run_swarm_experiment] Analysis directory: %s\n' "${analysis_dir}"
printf '[run_swarm_experiment] Manifest: %s\n' "${manifest_path}"

if ! docker ps --format '{{.Names}}' | grep -qx "${STACK_NAME}-decision-dev"; then
  printf 'Missing %s container. Start the visual fleet stack first.\n' "${STACK_NAME}-decision-dev" >&2
  exit 1
fi

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
  timeout $(python3 -c 'import json,sys; print(int(float(json.loads(sys.argv[1])["capture_duration_sec"]) + 10))' "${estimate_json}") \
    ros2 run decision_agent swarm_scenario_node -- \
      --manifest '${manifest_mount}' \
      --scenario-file '${scenario_mount}'
" >"${scenario_log}" 2>&1

python3 - "${analysis_dir}" "${current_run_dir}" <<'PY'
import json
import sys
from pathlib import Path

analysis_dir = Path(sys.argv[1])
run_dir = Path(sys.argv[2])
result = {
    "analysis_dir": str(analysis_dir),
    "run_dir": str(run_dir),
    "scenario_log": str(analysis_dir / "swarm_scenario_node.log"),
    "profile_summary": None,
}

summary_path = run_dir / "diagnostics" / "profile" / "summary.json"
if summary_path.exists():
    result["profile_summary"] = str(summary_path)

(analysis_dir / "experiment.report.json").write_text(json.dumps(result, indent=2) + "\n")
PY

profile_manifest_path="${current_run_dir}/diagnostics/profile/profile.manifest.json"
if [ -f "${profile_manifest_path}" ]; then
  python3 "${ROOT_DIR}/scripts/analyze_stack_profile.py" --run-dir "${current_run_dir}" >/dev/null
  printf '[run_swarm_experiment] Profile summary: %s\n' "${current_run_dir}/diagnostics/profile/summary.json"
fi

printf '[run_swarm_experiment] Swarm scenario completed successfully.\n'
printf '[run_swarm_experiment] Scenario log: %s\n' "${scenario_log}"
