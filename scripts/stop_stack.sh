#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

current_run_dir_file="${LOG_DIR}/runtime/visual.current_run_dir"
current_run_dir=""
if [ -f "${current_run_dir_file}" ]; then
  current_run_dir="$(cat "${current_run_dir_file}")"
fi

profile_state_file=""
if [ -n "${current_run_dir}" ]; then
  profile_state_file="${current_run_dir}/diagnostics/profile/profile.state.json"
fi

if [ -n "${profile_state_file}" ] && [ -f "${profile_state_file}" ]; then
  supervisor_pid="$(python3 - "${profile_state_file}" <<'PY'
import json
import sys
from pathlib import Path
data = json.loads(Path(sys.argv[1]).read_text())
print(data.get('pid', ''))
PY
)"
  if [ -n "${supervisor_pid}" ]; then
    kill "${supervisor_pid}" >/dev/null 2>&1 || true
    sleep 2
  fi
fi

while IFS= read -r container; do
  [ -n "${container}" ] || continue
  docker rm -f "${container}" >/dev/null 2>&1 || true
done < <(docker ps -a --format '{{.Names}}' | grep -E "^${STACK_NAME}-" || true)

docker network rm "${NETWORK}" >/dev/null 2>&1 || true
docker volume rm "${VOLUME}" >/dev/null 2>&1 || true

if [ -n "${current_run_dir}" ] && [ -f "${current_run_dir}/diagnostics/profile/profile.manifest.json" ]; then
  python3 "${ROOT_DIR}/scripts/analyze_stack_profile.py" --run-dir "${current_run_dir}" >/dev/null 2>&1 || true
fi

printf 'Stopped %s\n' "${STACK_NAME}"
