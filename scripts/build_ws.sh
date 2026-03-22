#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

docker build -t "${IMAGE}" -f "${DOCKER_DIR}/alate-ros-stack.Dockerfile" "${ROOT_DIR}"
printf 'Built %s\n' "${IMAGE}"
