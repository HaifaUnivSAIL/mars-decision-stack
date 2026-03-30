#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

if ! docker ps --format '{{.Names}}' | grep -qx "${STACK_NAME}-decision-dev"; then
  printf 'decision-dev is not running. Start the stack first with %s\n' "${ROOT_DIR}/scripts/run_stack.sh" >&2
  exit 1
fi

printf 'Opening a development shell only.\n'
printf 'For keyboard flight control, run %s instead.\n' "${ROOT_DIR}/scripts/run_visual_teleop.sh"

docker exec -it "${STACK_NAME}-decision-dev" bash
