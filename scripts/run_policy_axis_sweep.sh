#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR_FILE="$ROOT_DIR/logs/runtime/visual.current_run_dir"
MAGNITUDE="${MAGNITUDE:-0.40}"
PROBE_DURATION="${PROBE_DURATION:-6}"
COMMAND_DURATION="${COMMAND_DURATION:-2.5}"
PRE_CMD_DELAY="${PRE_CMD_DELAY:-2.5}"
SETTLE_DELAY="${SETTLE_DELAY:-2}"
TAKEOFF="${TAKEOFF:-0}"
TAKEOFF_WAIT_SEC="${TAKEOFF_WAIT_SEC:-10}"

if [[ ! -f "$RUN_DIR_FILE" ]]; then
  echo "Missing $RUN_DIR_FILE. Start the visual stack first." >&2
  exit 1
fi

CURRENT_RUN_DIR="$(<"$RUN_DIR_FILE")"
ANALYSIS_DIR="$CURRENT_RUN_DIR/analysis/axis_sweep_$(date +%Y%m%d-%H%M%S)"
mkdir -p "$ANALYSIS_DIR"

ros_exec() {
  docker exec mars-decision-stack-decision-dev bash -lc \
    "source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && $*"
}

run_probe() {
  local name="$1"
  local x="$2"
  local y="$3"
  local output_dir="$ANALYSIS_DIR/$name"
  local log_file="/tmp/${name}_axis_sweep.log"

  rm -rf "$output_dir"
  python3 "$ROOT_DIR/scripts/compare_policy_input_to_imu.py" \
    --duration "$PROBE_DURATION" \
    --output-dir "$output_dir" >"$log_file" 2>&1 &
  local probe_pid=$!

  sleep "$PRE_CMD_DELAY"
  set +e
  ros_exec "timeout $COMMAND_DURATION ros2 topic pub -r 10 /alate_input_velocity geometry_msgs/msg/Twist \"{linear: {x: $x, y: $y, z: 0.00}, angular: {x: 0.00, y: 0.00, z: 0.00}}\"" >/dev/null
  status=$?
  set -e
  if [[ "$status" -ne 0 && "$status" -ne 124 ]]; then
    return "$status"
  fi
  wait "$probe_pid"

  echo "=== $name ==="
  cat "$log_file"
}

if [[ "$TAKEOFF" == "1" ]]; then
  echo "[axis_sweep] Sending takeoff command and waiting ${TAKEOFF_WAIT_SEC}s"
  ros_exec "ros2 topic pub --once /alate_input_operator_command ros_alate_interfaces/msg/OpCom \"{op_com_enum: 1}\"" >/dev/null
  sleep "$TAKEOFF_WAIT_SEC"
fi

run_probe x_pos "$MAGNITUDE" "0.00"
sleep "$SETTLE_DELAY"
run_probe x_neg "-$MAGNITUDE" "0.00"
sleep "$SETTLE_DELAY"
run_probe y_pos "0.00" "$MAGNITUDE"
sleep "$SETTLE_DELAY"
run_probe y_neg "0.00" "-$MAGNITUDE"

python3 - "$ANALYSIS_DIR" <<'PY'
import csv
import sys
from pathlib import Path

analysis_dir = Path(sys.argv[1])
rows = []
for probe_dir in sorted(path for path in analysis_dir.iterdir() if path.is_dir()):
    comparison_path = probe_dir / "comparison.csv"
    if not comparison_path.exists():
        continue
    with comparison_path.open() as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            row = dict(row)
            row["probe_name"] = probe_dir.name
            rows.append(row)

if rows:
    output_path = analysis_dir / "axis_sweep.comparison.csv"
    with output_path.open("w", newline="") as handle:
        fieldnames = ["probe_name", *rows[0].keys()]
        deduped = []
        seen = set()
        for field in fieldnames:
            if field in seen:
                continue
            deduped.append(field)
            seen.add(field)
        writer = csv.DictWriter(handle, fieldnames=deduped)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    print(output_path)
PY

echo "[axis_sweep] Analysis directory: $ANALYSIS_DIR"
