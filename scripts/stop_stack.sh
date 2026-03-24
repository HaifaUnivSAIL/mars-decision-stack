#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

containers=(
  "${STACK_NAME}-decision-dev"
  "${STACK_NAME}-ros-nemala"
  "${STACK_NAME}-ros-alate"
  "${STACK_NAME}-hlc"
  "${STACK_NAME}-mc"
  "${STACK_NAME}-logger"
  "${STACK_NAME}-proxy"
  "${STACK_NAME}-visual-sim"
  "${STACK_NAME}-sitl"
)

for container in "${containers[@]}"; do
  docker rm -f "${container}" >/dev/null 2>&1 || true
done

docker network rm "${NETWORK}" >/dev/null 2>&1 || true
docker volume rm "${VOLUME}" >/dev/null 2>&1 || true

printf 'Stopped %s\n' "${STACK_NAME}"
