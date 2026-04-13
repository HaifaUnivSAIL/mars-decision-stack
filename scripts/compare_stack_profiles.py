#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Compare two analyzed stack profile runs.')
    parser.add_argument('--baseline', required=True, type=Path)
    parser.add_argument('--candidate', required=True, type=Path)
    return parser.parse_args()


def load_summary(run_dir: Path) -> dict[str, Any]:
    summary_path = run_dir / 'diagnostics' / 'profile' / 'summary.json'
    if not summary_path.exists():
        raise SystemExit(f'Missing summary.json in {run_dir}')
    return json.loads(summary_path.read_text())


def delta(candidate_value, baseline_value):
    if candidate_value is None or baseline_value is None:
        return None
    return candidate_value - baseline_value


def main() -> int:
    args = parse_args()
    baseline_dir = args.baseline.resolve()
    candidate_dir = args.candidate.resolve()
    baseline = load_summary(baseline_dir)
    candidate = load_summary(candidate_dir)

    comparison = {
        'baseline_run_id': baseline.get('run_id'),
        'candidate_run_id': candidate.get('run_id'),
        'startup_deltas': {},
        'world_deltas': {
            'rtf_median_delta': delta(
                candidate.get('world', {}).get('real_time_factor', {}).get('median'),
                baseline.get('world', {}).get('real_time_factor', {}).get('median'),
            ),
            'rtf_min_delta': delta(
                candidate.get('world', {}).get('real_time_factor', {}).get('min'),
                baseline.get('world', {}).get('real_time_factor', {}).get('min'),
            ),
        },
        'camera_deltas': {},
        'container_cpu_peak_deltas': {},
        'drone_deltas': {},
    }

    all_streams = set(baseline.get('camera', {}).keys()) | set(candidate.get('camera', {}).keys())
    for stream_name in sorted(all_streams):
        comparison['camera_deltas'][stream_name] = {
            'fps_median_delta': delta(
                candidate.get('camera', {}).get(stream_name, {}).get('fps', {}).get('median'),
                baseline.get('camera', {}).get(stream_name, {}).get('fps', {}).get('median'),
            ),
            'backlog_p95_delta': delta(
                candidate.get('camera', {}).get(stream_name, {}).get('backlog_sec', {}).get('p95'),
                baseline.get('camera', {}).get(stream_name, {}).get('backlog_sec', {}).get('p95'),
            ),
        }

    all_containers = set(baseline.get('containers', {}).keys()) | set(candidate.get('containers', {}).keys())
    for container_name in sorted(all_containers):
        comparison['container_cpu_peak_deltas'][container_name] = delta(
            candidate.get('containers', {}).get(container_name, {}).get('cpu_percent', {}).get('max'),
            baseline.get('containers', {}).get(container_name, {}).get('cpu_percent', {}).get('max'),
        )

    all_drones = set(baseline.get('drones', {}).keys()) | set(candidate.get('drones', {}).keys())
    for drone_id in sorted(all_drones):
        baseline_drone = baseline.get('drones', {}).get(drone_id, {})
        candidate_drone = candidate.get('drones', {}).get(drone_id, {})
        comparison['drone_deltas'][drone_id] = {
            'startup_to_mc_delta': delta(
                candidate_drone.get('startup_time_to_mc_standby_sec'),
                baseline_drone.get('startup_time_to_mc_standby_sec'),
            ),
            'startup_to_hlc_delta': delta(
                candidate_drone.get('startup_time_to_hlc_ready_sec'),
                baseline_drone.get('startup_time_to_hlc_ready_sec'),
            ),
            'command_to_motion_delta': delta(
                candidate_drone.get('command_to_first_motion_latency', {}).get('wall_sec'),
                baseline_drone.get('command_to_first_motion_latency', {}).get('wall_sec'),
            ),
            'command_to_actuator_delta': delta(
                candidate_drone.get('command_to_first_actuator_change_latency', {}).get('wall_sec'),
                baseline_drone.get('command_to_first_actuator_change_latency', {}).get('wall_sec'),
            ),
            'takeoff_duration_delta': delta(
                candidate_drone.get('takeoff_duration_to_altitude_threshold', {}).get('wall_sec'),
                baseline_drone.get('takeoff_duration_to_altitude_threshold', {}).get('wall_sec'),
            ),
            'landing_duration_delta': delta(
                candidate_drone.get('landing_duration_to_standby', {}).get('wall_sec'),
                baseline_drone.get('landing_duration_to_standby', {}).get('wall_sec'),
            ),
        }

    output_dir = candidate_dir / 'diagnostics' / 'profile'
    comparison_json = output_dir / 'comparison.json'
    comparison_md = output_dir / 'comparison.md'
    comparison_json.write_text(json.dumps(comparison, indent=2) + '\n')

    lines = [
        '# Stack Profile Comparison',
        '',
        f"Baseline: `{baseline.get('run_id', '')}`",
        f"Candidate: `{candidate.get('run_id', '')}`",
        '',
        '## World',
        f"- RTF median delta: `{comparison['world_deltas']['rtf_median_delta']}`",
        f"- RTF min delta: `{comparison['world_deltas']['rtf_min_delta']}`",
        '',
        '## Camera',
    ]
    for stream_name, stream_delta in comparison['camera_deltas'].items():
        lines.append(
            f"- `{stream_name}` fps median delta={stream_delta['fps_median_delta']} backlog p95 delta={stream_delta['backlog_p95_delta']}"
        )
    lines.append('')
    lines.append('## Per Drone')
    for drone_id, drone_delta in comparison['drone_deltas'].items():
        lines.append(
            f"- `{drone_id}` startup(mc/hlc) delta={drone_delta['startup_to_mc_delta']} / {drone_delta['startup_to_hlc_delta']} command->motion delta={drone_delta['command_to_motion_delta']}"
        )
    lines.append('')
    comparison_md.write_text('\n'.join(lines) + '\n')
    print(comparison_json)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
