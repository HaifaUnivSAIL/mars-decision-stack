#!/usr/bin/env bash

set -euo pipefail

VISUAL_WORLD="${VISUAL_WORLD:-/opt/mars-sim/worlds/mars_iris_dual_view.sdf}"

if [ -n "${VISUAL_RESOURCE_PATH_PREFIX:-}" ]; then
  export GZ_SIM_RESOURCE_PATH="${VISUAL_RESOURCE_PATH_PREFIX}:${GZ_SIM_RESOURCE_PATH:-}"
fi

exec gz sim -s -v4 -r "${VISUAL_WORLD}"
