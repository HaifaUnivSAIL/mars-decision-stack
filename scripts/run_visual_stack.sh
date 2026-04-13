#!/usr/bin/env bash

set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

visual_mode="${VISUAL_MODE:-experiment}"
fleet_args=()
force_single=0
profile_enabled=0

while [ "$#" -gt 0 ]; do
  case "$1" in
    --calib)
      visual_mode="calib"
      ;;
    --experiment)
      visual_mode="experiment"
      ;;
    --fleet)
      shift
      if [ "$#" -eq 0 ]; then
        printf 'Missing value for --fleet\n' >&2
        exit 1
      fi
      fleet_args+=(--fleet "$1")
      ;;
    --single)
      force_single=1
      ;;
    --profile)
      profile_enabled=1
      ;;
    *)
      printf 'Unsupported argument: %s\n' "$1" >&2
      printf 'Usage: %s [--experiment|--calib] [--single] [--fleet PATH] [--profile]\n' "$0" >&2
      exit 1
      ;;
  esac
  shift
done

if [ "${force_single}" = '1' ] && [ "${#fleet_args[@]}" -gt 0 ]; then
  printf 'Use either --single or --fleet, not both.\n' >&2
  exit 1
fi

if [ "${visual_mode}" = 'calib' ]; then
  if [ "${#fleet_args[@]}" -gt 0 ]; then
    printf 'Calibration mode is single-drone only in Phase 1; omit --fleet.\n' >&2
    exit 1
  fi
  if [ "${profile_enabled}" = '1' ]; then
    exec "${ROOT_DIR}/scripts/run_visual_single_stack.sh" --calib --profile
  fi
  exec "${ROOT_DIR}/scripts/run_visual_single_stack.sh" --calib
fi

if [ "${#fleet_args[@]}" -gt 0 ]; then
  if [ "${profile_enabled}" = '1' ]; then
    exec "${ROOT_DIR}/scripts/run_visual_fleet_stack.sh" "${fleet_args[@]}" --profile
  fi
  exec "${ROOT_DIR}/scripts/run_visual_fleet_stack.sh" "${fleet_args[@]}"
fi

if [ "${profile_enabled}" = '1' ]; then
  exec "${ROOT_DIR}/scripts/run_visual_single_stack.sh" --profile
fi
exec "${ROOT_DIR}/scripts/run_visual_single_stack.sh"
