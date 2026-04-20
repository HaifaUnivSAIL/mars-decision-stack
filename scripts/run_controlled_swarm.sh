#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

usage() {
  cat <<EOF
Usage:
  $0 --count N [--drone ID] [--headless] [--profile] [--runtime-profile NAME]
  $0 --drone ID
  $0 --fleet PATH [--drone ID] [--headless] [--profile]
  $0 --help

Examples:
  $0 --count 2
  $0 --count 2 --drone drone_1
  $0 --drone drone_2

Behavior:
  - With --count N, generate a fleet manifest for N drones, start the visual swarm,
    and prepare per-drone keyboard teleop params automatically.
  - Count-based bringup defaults to runtime profile single_equivalent for every N
    unless --runtime-profile is provided explicitly.
  - With --drone ID, attach a keyboard teleop session to the selected drone in the
    currently running swarm stack.
  - With both --count and --drone, start the swarm first and then attach to that drone.
EOF
}

count=""
drone_id=""
fleet_config=""
visual_gui="${VISUAL_GUI:-1}"
dry_run=0
profile_enabled=0
runtime_profile=""

while [ "$#" -gt 0 ]; do
  case "$1" in
    --count)
      shift
      if [ "$#" -eq 0 ]; then
        printf 'Missing value for --count\n' >&2
        exit 1
      fi
      count="$1"
      ;;
    --drone)
      shift
      if [ "$#" -eq 0 ]; then
        printf 'Missing value for --drone\n' >&2
        exit 1
      fi
      drone_id="$1"
      ;;
    --fleet)
      shift
      if [ "$#" -eq 0 ]; then
        printf 'Missing value for --fleet\n' >&2
        exit 1
      fi
      fleet_config="$1"
      ;;
    --headless)
      visual_gui=0
      ;;
    --gui)
      visual_gui=1
      ;;
    --dry-run)
      dry_run=1
      ;;
    --profile)
      profile_enabled=1
      ;;
    --runtime-profile)
      shift
      if [ "$#" -eq 0 ]; then
        printf 'Missing value for --runtime-profile\n' >&2
        exit 1
      fi
      runtime_profile="$1"
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      printf 'Unsupported argument: %s\n' "$1" >&2
      usage >&2
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

current_run_dir() {
  local runtime_file="${LOG_DIR}/runtime/visual.current_run_dir"
  if [ -f "${runtime_file}" ]; then
    cat "${runtime_file}"
  fi
}

visual_manifest_path() {
  local run_dir
  run_dir="$(current_run_dir)"
  if [ -n "${run_dir}" ]; then
    printf '%s/visual_assets/manifest.json\n' "${run_dir}"
  fi
}

stack_running() {
  docker ps --format '{{.Names}}' | grep -qx "${STACK_NAME}-visual-sim"
}

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

resolve_drone_id() {
  local manifest_path="$1"
  python3 - "$manifest_path" "$drone_id" <<'PY'
import json
import sys
from pathlib import Path

manifest = json.loads(Path(sys.argv[1]).read_text())
requested = sys.argv[2].strip()
drone_ids = [drone["id"] for drone in manifest.get("drones", [])]
if not drone_ids:
    raise SystemExit("Manifest does not define any drones")
if requested:
    if requested not in drone_ids:
        raise SystemExit(f"Unknown drone id {requested!r}. Available: {drone_ids}")
    print(requested)
else:
    print(manifest.get("active_drone_id") or drone_ids[0])
PY
}

wait_for_runtime_ready() {
  local manifest_path="$1"
  local selected_drone_id="$2"
  local timeout_sec="${3:-180}"
  local start_sec
  start_sec="$(date +%s)"

  while true; do
    local hlc_logs mc_logs
    hlc_logs="$(docker logs "${STACK_NAME}-${selected_drone_id}-hlc" 2>&1 || true)"
    mc_logs="$(docker logs "${STACK_NAME}-${selected_drone_id}-mc" 2>&1 || true)"
    if grep -q 'HLC entering state: Ready' <<<"${hlc_logs}" && grep -q 'MissionControl entering state: Standby' <<<"${mc_logs}"; then
      return 0
    fi
    if [ $(( $(date +%s) - start_sec )) -ge "${timeout_sec}" ]; then
      printf 'Timed out waiting for HLC/MC readiness of %s.\n' "${selected_drone_id}" >&2
      return 1
    fi
    sleep 1
  done
}

render_teleop_params() {
  local manifest_path="$1"
  local run_dir="$2"
  python3 - "$manifest_path" "$run_dir" "$ROOT_DIR" <<'PY'
import json
import subprocess
import sys
from pathlib import Path

manifest_path = Path(sys.argv[1])
run_dir = Path(sys.argv[2])
root_dir = Path(sys.argv[3])
manifest = json.loads(manifest_path.read_text())

out_dir = run_dir / "ros_params"
out_dir.mkdir(parents=True, exist_ok=True)

for drone in manifest.get("drones", []):
    output_path = out_dir / f"manual_runtime_test.{drone['id']}.keyboard.yaml"
    subprocess.run(
        [
            "python3",
            str(root_dir / "scripts" / "render_namespaced_ros_params.py"),
            "--source",
            str(root_dir / "config" / "manual_runtime_test" / "keyboard.yaml"),
            "--node-name",
            "manual_runtime_test",
            "--namespace",
            drone["namespace"],
            "--output",
            str(output_path),
        ],
        check=True,
    )
PY
}

print_ready_banner() {
  local manifest_path="$1"
  python3 - "$manifest_path" "$0" <<'PY'
import json
import sys
from pathlib import Path

manifest = json.loads(Path(sys.argv[1]).read_text())
script_path = sys.argv[2]
drone_ids = [drone["id"] for drone in manifest.get("drones", [])]
print("Swarm stack is ready.")
print("Control terminals:")
for drone_id in drone_ids:
    print(f"  {script_path} --drone {drone_id}")
print("Camera focus examples:")
for drone_id in drone_ids:
    print(f"  ./scripts/set_visual_focus.sh --drone {drone_id}")
PY
}

attach_teleop() {
  local manifest_path="$1"
  local run_dir="$2"
  local selected_drone_id="$3"
  local teleop_params_host="${run_dir}/ros_params/manual_runtime_test.${selected_drone_id}.keyboard.yaml"
  local teleop_params_mount="/workspace/run/ros_params/manual_runtime_test.${selected_drone_id}.keyboard.yaml"

  if [ ! -f "${teleop_params_host}" ]; then
    render_teleop_params "${manifest_path}" "${run_dir}"
  fi

  wait_for_runtime_ready "${manifest_path}" "${selected_drone_id}" 180

  printf 'Vehicle control is routed through Gazebo-backed SITL.\n'
  printf 'Selected drone: %s\n' "${selected_drone_id}"
  printf 'Controls: t=takeoff l=land g=gohome w=forward a=left s=back d=right r=up f=down space=stop h=help q=quit\n'
  printf 'Use ./scripts/set_visual_focus.sh --drone %s to switch the active camera.\n' "${selected_drone_id}"

  docker exec -it "${STACK_NAME}-decision-dev" bash -lc "
    set -eo pipefail
    set +u
    source /opt/ros/humble/setup.bash
    if [ -f /workspace/ws/install/setup.bash ]; then
      source /workspace/ws/install/setup.bash
    else
      source /opt/ros2_ws/install/setup.bash
    fi
    set -u
    mkdir -p /workspace/run/logs
    exec > >(tee -a /workspace/run/logs/manual-runtime-test.${selected_drone_id}.log) 2>&1
    exec ros2 run manual_runtime_test keyboard_teleop --ros-args \
      --params-file '${teleop_params_mount}' \
      -r __ns:=/${selected_drone_id}
  "
}

if [ -n "${count}" ] || [ -n "${fleet_config}" ]; then
  if [ -z "${fleet_config}" ]; then
    fleet_config="${LOG_DIR}/runtime/controlled_swarm.requested.json"
    generate_manifest_from_count "${fleet_config}" "${count}" "${runtime_profile}"
  fi

  if [ "${dry_run}" = "1" ]; then
    printf 'Generated fleet manifest at %s\n' "${fleet_config}"
    cat "${fleet_config}"
    exit 0
  fi

  profile_args=()
  if [ "${profile_enabled}" = '1' ]; then
    profile_args+=(--profile)
  fi
  VISUAL_GUI="${visual_gui}" "${ROOT_DIR}/scripts/run_visual_stack.sh" --fleet "${fleet_config}" "${profile_args[@]}"

  run_dir="$(current_run_dir)"
  if [ -z "${run_dir}" ] || [ ! -d "${run_dir}" ]; then
    printf 'Missing current visual run directory after startup.\n' >&2
    exit 1
  fi

  manifest_path="${run_dir}/visual_assets/manifest.json"
  if [ ! -f "${manifest_path}" ]; then
    printf 'Missing runtime manifest: %s\n' "${manifest_path}" >&2
    exit 1
  fi

  render_teleop_params "${manifest_path}" "${run_dir}"
  print_ready_banner "${manifest_path}"

  if [ -n "${drone_id}" ]; then
    selected_drone_id="$(resolve_drone_id "${manifest_path}")"
    attach_teleop "${manifest_path}" "${run_dir}" "${selected_drone_id}"
  fi

  exit 0
fi

if [ -z "${drone_id}" ]; then
  usage >&2
  exit 1
fi

if ! stack_running; then
  printf 'Swarm stack is not running. Start it first with %s --count 2\n' "$0" >&2
  exit 1
fi

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

selected_drone_id="$(resolve_drone_id "${manifest_path}")"
attach_teleop "${manifest_path}" "${run_dir}" "${selected_drone_id}"
