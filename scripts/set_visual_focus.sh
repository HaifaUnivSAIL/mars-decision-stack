#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

drone_id=""
fleet_config=""
while [ "$#" -gt 0 ]; do
  case "$1" in
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
    *)
      printf 'Unsupported argument: %s\n' "$1" >&2
      printf 'Usage: %s --drone ID [--fleet PATH]\n' "$0" >&2
      exit 1
      ;;
  esac
  shift
done

if [ -z "${drone_id}" ]; then
  printf 'Usage: %s --drone ID [--fleet PATH]\n' "$0" >&2
  exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -qx "${STACK_NAME}-visual-sim"; then
  printf 'Visual stack is not running. Start it first with %s\n' "${ROOT_DIR}/scripts/run_visual_stack.sh" >&2
  exit 1
fi

manifest_path="${LOG_DIR}/runtime/visual.manifest.json"
if [ -f "${manifest_path}" ]; then
  python3 - "$manifest_path" "$drone_id" <<'PY' >/dev/null
import json
import sys
from pathlib import Path
manifest = json.loads(Path(sys.argv[1]).read_text())
drone_id = sys.argv[2]
known = [drone['id'] for drone in manifest.get('drones', [])]
if known and drone_id not in known:
    raise SystemExit(f'Unknown drone id {drone_id!r}. Available: {known}')
PY
elif [ -n "${fleet_config}" ]; then
  python3 - "$fleet_config" "$drone_id" <<'PY' >/dev/null
import json
import sys
from pathlib import Path
manifest = json.loads(Path(sys.argv[1]).read_text())
drone_id = sys.argv[2]
known = [drone['id'] for drone in manifest.get('drones', [])]
if drone_id not in known:
    raise SystemExit(f'Unknown drone id {drone_id!r}. Available: {known}')
PY
fi

docker exec -u "$(id -u):$(id -g)" -e "GZ_PARTITION=${GZ_PARTITION}" "${STACK_NAME}-visual-sim" \
  bash -lc "gz topic -t /mars/visual/active_drone/select -m gz.msgs.StringMsg -p 'data: \"${drone_id}\"' >/dev/null"

echo_file="$(mktemp)"
trap 'rm -f "${echo_file}"' EXIT
if docker exec -u "$(id -u):$(id -g)" -e "GZ_PARTITION=${GZ_PARTITION}" "${STACK_NAME}-visual-sim" \
  bash -lc 'timeout 5 gz topic -e -n 1 -t /mars/visual/active_drone/state' >"${echo_file}" 2>/dev/null; then
  if grep -Fq "${drone_id}" "${echo_file}"; then
    printf 'Active visual focus switched to %s\n' "${drone_id}"
    exit 0
  fi
fi

printf 'Published visual focus request for %s\n' "${drone_id}"
