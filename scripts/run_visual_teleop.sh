#!/usr/bin/env bash

set -euo pipefail
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/run_visual_manual_runtime_test.sh" "$@"
