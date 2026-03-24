#!/usr/bin/env bash

set -euo pipefail

VISUAL_WORLD="${VISUAL_WORLD:-/opt/mars-sim/worlds/mars_iris_dual_view.sdf}"
exec gz sim -s -v4 -r "${VISUAL_WORLD}"
