#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

mkdir -p "${EXTERNAL_DIR}" "${WS_SRC_DIR}" "${LOG_DIR}"

if ! command -v vcs >/dev/null 2>&1; then
  python3 -m pip install --user vcstool
  export PATH="${HOME}/.local/bin:${PATH}"
fi

vcs import --recursive --skip-existing "${ROOT_DIR}" < "${MANIFEST_FILE}"

apply_if_needed() {
  local repo_dir="$1"
  local patch_file="$2"

  if [ ! -f "${patch_file}" ]; then
    return 0
  fi

  if git -C "${repo_dir}" apply --reverse --check "${patch_file}" >/dev/null 2>&1; then
    printf 'Patch already applied: %s\n' "${patch_file}"
    return 0
  fi

  if git -C "${repo_dir}" apply --check "${patch_file}" >/dev/null 2>&1; then
    git -C "${repo_dir}" apply "${patch_file}"
    printf 'Applied patch: %s\n' "${patch_file}"
    return 0
  fi

  printf 'Patch failed to apply cleanly: %s\n' "${patch_file}" >&2
  return 1
}

apply_if_needed "${ALATE_DIR}" "${PATCH_DIR}/alate-local.patch"
apply_if_needed "${EXTERNAL_DIR}/ardupilot_gazebo" "${PATCH_DIR}/ardupilot_gazebo-visual.patch"
apply_if_needed "${WS_SRC_DIR}/ros_alate_interfaces" "${PATCH_DIR}/ros_alate_interfaces.patch"
apply_if_needed "${WS_SRC_DIR}/ros_alate" "${PATCH_DIR}/ros_alate.patch"

printf 'Bootstrap complete.\n'
