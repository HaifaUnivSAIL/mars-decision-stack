#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

if [ ! -d "${EXTERNAL_DIR}/ardupilot_gazebo" ]; then
  printf 'Missing external/ardupilot_gazebo. Run ./scripts/bootstrap.sh first.\n' >&2
  exit 1
fi

docker build \
  -t "${VISUAL_SIM_IMAGE}" \
  --build-arg "ARDUPILOT_REF=${VISUAL_ARDUPILOT_REF}" \
  -f "${DOCKER_DIR}/visual-sim.Dockerfile" \
  "${ROOT_DIR}"
printf 'Built %s\n' "${VISUAL_SIM_IMAGE}"
