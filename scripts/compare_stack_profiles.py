#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


YAW_SPIN_THRESHOLD_DEG = 45.0
YAW_RATE_SPIN_THRESHOLD_RAD_S = 0.8


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Compare two analyzed stack profile runs.')
    parser.add_argument('--baseline', required=True, type=Path)
    parser.add_argument('--candidate', required=True, type=Path)
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise SystemExit(f'Missing required file: {path}')
    return json.loads(path.read_text())


def load_summary(run_dir: Path) -> dict[str, Any]:
    return load_json(run_dir / 'diagnostics' / 'profile' / 'summary.json')


def load_control_summary(run_dir: Path) -> dict[str, Any]:
    return load_json(run_dir / 'diagnostics' / 'profile' / 'control_summary.json')


def delta(candidate_value, baseline_value):
    if candidate_value is None or baseline_value is None:
        return None
    return candidate_value - baseline_value


def drone_reference(drone_map: dict[str, Any], drone_id: str) -> tuple[str | None, dict[str, Any]]:
    if drone_id in drone_map:
        return drone_id, drone_map[drone_id]
    if 'drone_1' in drone_map:
        return 'drone_1', drone_map['drone_1']
    if drone_map:
        reference_id = sorted(drone_map)[0]
        return reference_id, drone_map[reference_id]
    return None, {}


class ProfileComparison:
    def __init__(self, baseline_dir: Path, candidate_dir: Path):
        self.baseline_dir = baseline_dir.resolve()
        self.candidate_dir = candidate_dir.resolve()
        self.baseline = load_summary(self.baseline_dir)
        self.candidate = load_summary(self.candidate_dir)
        self.baseline_control = load_control_summary(self.baseline_dir)
        self.candidate_control = load_control_summary(self.candidate_dir)

    def _world_deltas(self) -> dict[str, Any]:
        return {
            'rtf_median_delta': delta(
                self.candidate.get('world', {}).get('real_time_factor', {}).get('median'),
                self.baseline.get('world', {}).get('real_time_factor', {}).get('median'),
            ),
            'rtf_min_delta': delta(
                self.candidate.get('world', {}).get('real_time_factor', {}).get('min'),
                self.baseline.get('world', {}).get('real_time_factor', {}).get('min'),
            ),
            'rtf_p95_delta': delta(
                self.candidate.get('world', {}).get('real_time_factor', {}).get('p95'),
                self.baseline.get('world', {}).get('real_time_factor', {}).get('p95'),
            ),
        }

    def _camera_deltas(self) -> dict[str, Any]:
        all_streams = set(self.baseline.get('camera', {}).keys()) | set(self.candidate.get('camera', {}).keys())
        result = {}
        for stream_name in sorted(all_streams):
            baseline_stream = self.baseline.get('camera', {}).get(stream_name, {})
            candidate_stream = self.candidate.get('camera', {}).get(stream_name, {})
            result[stream_name] = {
                'fps_median_delta': delta(
                    candidate_stream.get('fps', {}).get('median'),
                    baseline_stream.get('fps', {}).get('median'),
                ),
                'fps_p95_delta': delta(
                    candidate_stream.get('fps', {}).get('p95'),
                    baseline_stream.get('fps', {}).get('p95'),
                ),
                'backlog_p95_delta': delta(
                    candidate_stream.get('backlog_sec', {}).get('p95'),
                    baseline_stream.get('backlog_sec', {}).get('p95'),
                ),
                'backlog_max_delta': delta(
                    candidate_stream.get('backlog_sec', {}).get('max'),
                    baseline_stream.get('backlog_sec', {}).get('max'),
                ),
            }
        return result

    def _container_deltas(self) -> dict[str, Any]:
        all_containers = set(self.baseline.get('containers', {}).keys()) | set(self.candidate.get('containers', {}).keys())
        result = {}
        for container_name in sorted(all_containers):
            baseline_container = self.baseline.get('containers', {}).get(container_name, {})
            candidate_container = self.candidate.get('containers', {}).get(container_name, {})
            result[container_name] = {
                'cpu_peak_delta': delta(
                    candidate_container.get('cpu_percent', {}).get('max'),
                    baseline_container.get('cpu_percent', {}).get('max'),
                ),
                'cpu_mean_delta': delta(
                    candidate_container.get('cpu_percent', {}).get('mean'),
                    baseline_container.get('cpu_percent', {}).get('mean'),
                ),
                'mem_peak_delta_bytes': delta(
                    candidate_container.get('mem_usage_bytes', {}).get('max'),
                    baseline_container.get('mem_usage_bytes', {}).get('max'),
                ),
                'mem_mean_delta_bytes': delta(
                    candidate_container.get('mem_usage_bytes', {}).get('mean'),
                    baseline_container.get('mem_usage_bytes', {}).get('mean'),
                ),
            }
        return result

    def _performance_drone_deltas(self) -> dict[str, Any]:
        candidate_drones = self.candidate.get('drones', {})
        baseline_drones = self.baseline.get('drones', {})
        all_drones = set(candidate_drones.keys()) | set(baseline_drones.keys())
        result = {}
        for drone_id in sorted(all_drones):
            baseline_ref_id, baseline_drone = drone_reference(baseline_drones, drone_id)
            candidate_drone = candidate_drones.get(drone_id, {})
            result[drone_id] = {
                'baseline_reference_drone': baseline_ref_id,
                'startup_to_mc_delta': delta(
                    candidate_drone.get('startup_time_to_mc_standby_sec'),
                    baseline_drone.get('startup_time_to_mc_standby_sec'),
                ),
                'startup_to_hlc_delta': delta(
                    candidate_drone.get('startup_time_to_hlc_ready_sec'),
                    baseline_drone.get('startup_time_to_hlc_ready_sec'),
                ),
                'command_to_state_delta': delta(
                    candidate_drone.get('command_to_state_transition_latency', {}).get('wall_sec'),
                    baseline_drone.get('command_to_state_transition_latency', {}).get('wall_sec'),
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
        return result

    def _control_deltas(self) -> tuple[dict[str, Any], list[dict[str, Any]]]:
        result = {}
        findings: list[dict[str, Any]] = []
        baseline_drones = self.baseline_control.get('drones', {})
        candidate_drones = self.candidate_control.get('drones', {})
        all_drones = set(candidate_drones.keys()) | set(baseline_drones.keys())
        baseline_rtf = self.baseline.get('world', {}).get('real_time_factor', {}).get('median')
        candidate_rtf = self.candidate.get('world', {}).get('real_time_factor', {}).get('median')

        for drone_id in sorted(all_drones):
            baseline_ref_id, baseline_drone = drone_reference(baseline_drones, drone_id)
            candidate_drone = candidate_drones.get(drone_id, {})
            baseline_takeoff = baseline_drone.get('latest_takeoff') or {}
            candidate_takeoff = candidate_drone.get('latest_takeoff') or {}
            baseline_warning = baseline_takeoff.get('last_warning_text') or ''
            candidate_warning = candidate_takeoff.get('last_warning_text') or ''
            result[drone_id] = {
                'baseline_reference_drone': baseline_ref_id,
                'baseline_outcome': baseline_takeoff.get('outcome'),
                'candidate_outcome': candidate_takeoff.get('outcome'),
                'stage_latency_deltas': {
                    'mc_transition_delta': delta(candidate_takeoff.get('command_to_mc_transition_sec'), baseline_takeoff.get('command_to_mc_transition_sec')),
                    'hlc_takeoff_delta': delta(candidate_takeoff.get('command_to_hlc_takeoff_sec'), baseline_takeoff.get('command_to_hlc_takeoff_sec')),
                    'armed_delta': delta(candidate_takeoff.get('command_to_first_armed_sec'), baseline_takeoff.get('command_to_first_armed_sec')),
                    'actuator_delta': delta(candidate_takeoff.get('command_to_first_actuator_change_sec'), baseline_takeoff.get('command_to_first_actuator_change_sec')),
                    'motion_delta': delta(candidate_takeoff.get('command_to_first_motion_sec'), baseline_takeoff.get('command_to_first_motion_sec')),
                    'altitude_0_2_delta': delta(candidate_takeoff.get('command_to_altitude_0_2_sec'), baseline_takeoff.get('command_to_altitude_0_2_sec')),
                    'altitude_1_0_delta': delta(candidate_takeoff.get('command_to_altitude_1_0_sec'), baseline_takeoff.get('command_to_altitude_1_0_sec')),
                },
                'yaw_deltas': {
                    'max_yaw_delta_before_liftoff_deg_delta': delta(
                        candidate_takeoff.get('max_yaw_delta_before_0_2_deg'),
                        baseline_takeoff.get('max_yaw_delta_before_0_2_deg'),
                    ),
                    'max_yaw_rate_before_liftoff_rad_s_delta': delta(
                        candidate_takeoff.get('max_yaw_rate_before_0_2_rad_s'),
                        baseline_takeoff.get('max_yaw_rate_before_0_2_rad_s'),
                    ),
                },
                'state_sequences': {
                    'baseline_hlc': baseline_takeoff.get('hlc_state_sequence'),
                    'candidate_hlc': candidate_takeoff.get('hlc_state_sequence'),
                    'baseline_mc': baseline_takeoff.get('mc_state_sequence'),
                    'candidate_mc': candidate_takeoff.get('mc_state_sequence'),
                },
                'warnings': {
                    'baseline_warning_count': baseline_takeoff.get('warning_count'),
                    'candidate_warning_count': candidate_takeoff.get('warning_count'),
                    'baseline_last_warning': baseline_warning,
                    'candidate_last_warning': candidate_warning,
                },
            }

            outcome = candidate_takeoff.get('outcome')
            if outcome and outcome != baseline_takeoff.get('outcome'):
                findings.append(
                    {
                        'priority': 1,
                        'drone_id': drone_id,
                        'title': 'Takeoff outcome divergence',
                        'detail': f"baseline={baseline_takeoff.get('outcome')} candidate={outcome}",
                    }
                )
            if outcome == 'prearm_blocked' or 'prearm' in candidate_warning.lower():
                findings.append(
                    {
                        'priority': 2,
                        'drone_id': drone_id,
                        'title': 'Pre-arm failure detected',
                        'detail': candidate_warning or 'Candidate takeoff classified as prearm_blocked',
                    }
                )
            if outcome == 'yaw_spin_no_climb' or (
                (candidate_takeoff.get('max_yaw_delta_before_0_2_deg') or 0.0) >= YAW_SPIN_THRESHOLD_DEG
                and candidate_takeoff.get('command_to_altitude_0_2_sec') is None
            ):
                findings.append(
                    {
                        'priority': 3,
                        'drone_id': drone_id,
                        'title': 'Yaw instability before liftoff',
                        'detail': (
                            f"yaw_delta={candidate_takeoff.get('max_yaw_delta_before_0_2_deg')}deg "
                            f"yaw_rate={candidate_takeoff.get('max_yaw_rate_before_0_2_rad_s')}rad/s"
                        ),
                    }
                )
            if candidate_takeoff.get('command_to_first_armed_sec') is not None and candidate_takeoff.get('command_to_altitude_0_2_sec') is None:
                findings.append(
                    {
                        'priority': 4,
                        'drone_id': drone_id,
                        'title': 'Armed without climb',
                        'detail': (
                            f"armed at {candidate_takeoff.get('command_to_first_armed_sec')}s but never reached 0.2m"
                        ),
                    }
                )
            if candidate_takeoff.get('command_to_first_actuator_change_sec') is None and candidate_takeoff.get('command_to_first_armed_sec') is not None:
                findings.append(
                    {
                        'priority': 5,
                        'drone_id': drone_id,
                        'title': 'No actuator response after arming',
                        'detail': 'Takeoff armed but control trace never saw a first actuator delta',
                    }
                )
            if candidate_rtf is not None and baseline_rtf is not None and candidate_rtf < baseline_rtf * 0.8:
                findings.append(
                    {
                        'priority': 6,
                        'drone_id': drone_id,
                        'title': 'Simulator slowdown may be amplifying control differences',
                        'detail': f'baseline RTF={baseline_rtf:.3f} candidate RTF={candidate_rtf:.3f}',
                    }
                )
            if (candidate_takeoff.get('warning_count') or 0) > (baseline_takeoff.get('warning_count') or 0):
                findings.append(
                    {
                        'priority': 7,
                        'drone_id': drone_id,
                        'title': 'Extra warnings during takeoff',
                        'detail': f"baseline={baseline_takeoff.get('warning_count')} candidate={candidate_takeoff.get('warning_count')}",
                    }
                )

        findings.sort(key=lambda row: (row['priority'], row['drone_id'], row['title']))
        return result, findings

    def compare(self) -> dict[str, Any]:
        control_deltas, control_findings = self._control_deltas()
        comparison = {
            'baseline_run_id': self.baseline.get('run_id'),
            'candidate_run_id': self.candidate.get('run_id'),
            'startup_deltas': {},
            'world_deltas': self._world_deltas(),
            'camera_deltas': self._camera_deltas(),
            'container_deltas': self._container_deltas(),
            'drone_deltas': self._performance_drone_deltas(),
            'control': {
                'drones': control_deltas,
                'root_cause_candidates': control_findings,
            },
        }
        return comparison

    def render_markdown(self, comparison: dict[str, Any]) -> str:
        lines = [
            '# Stack Profile Comparison',
            '',
            f"Baseline: `{comparison.get('baseline_run_id', '')}`",
            f"Candidate: `{comparison.get('candidate_run_id', '')}`",
            '',
            '## World',
            f"- RTF median delta: `{comparison['world_deltas'].get('rtf_median_delta')}`",
            f"- RTF p95 delta: `{comparison['world_deltas'].get('rtf_p95_delta')}`",
            f"- RTF min delta: `{comparison['world_deltas'].get('rtf_min_delta')}`",
            '',
            '## Camera',
        ]
        for stream_name, stream_delta in comparison['camera_deltas'].items():
            lines.append(
                f"- `{stream_name}` fps median delta={stream_delta.get('fps_median_delta')} backlog p95 delta={stream_delta.get('backlog_p95_delta')} backlog max delta={stream_delta.get('backlog_max_delta')}"
            )
        lines.extend(['', '## Containers'])
        for container_name, container_delta in comparison['container_deltas'].items():
            lines.append(
                f"- `{container_name}` cpu peak delta={container_delta.get('cpu_peak_delta')} cpu mean delta={container_delta.get('cpu_mean_delta')} mem peak delta bytes={container_delta.get('mem_peak_delta_bytes')}"
            )
        lines.extend(['', '## Per Drone'])
        for drone_id, drone_delta in comparison['drone_deltas'].items():
            lines.append(
                f"- `{drone_id}` ref=`{drone_delta.get('baseline_reference_drone')}` startup(mc/hlc) delta={drone_delta.get('startup_to_mc_delta')} / {drone_delta.get('startup_to_hlc_delta')} command->state delta={drone_delta.get('command_to_state_delta')} command->motion delta={drone_delta.get('command_to_motion_delta')}"
            )
        lines.extend(['', '## Control'])
        for drone_id, control_delta in comparison['control']['drones'].items():
            stage = control_delta.get('stage_latency_deltas', {})
            yaw = control_delta.get('yaw_deltas', {})
            warnings = control_delta.get('warnings', {})
            lines.append(
                f"- `{drone_id}` ref=`{control_delta.get('baseline_reference_drone')}` outcome `{control_delta.get('baseline_outcome')}` -> `{control_delta.get('candidate_outcome')}`"
            )
            lines.append(
                f"- `{drone_id}` stage deltas mc/hlc/armed/actuator/motion/alt0.2/alt1.0 = {stage.get('mc_transition_delta')} / {stage.get('hlc_takeoff_delta')} / {stage.get('armed_delta')} / {stage.get('actuator_delta')} / {stage.get('motion_delta')} / {stage.get('altitude_0_2_delta')} / {stage.get('altitude_1_0_delta')}"
            )
            lines.append(
                f"- `{drone_id}` yaw delta={yaw.get('max_yaw_delta_before_liftoff_deg_delta')} deg yaw-rate delta={yaw.get('max_yaw_rate_before_liftoff_rad_s_delta')} rad/s"
            )
            if warnings.get('candidate_last_warning') or warnings.get('baseline_last_warning'):
                lines.append(
                    f"- `{drone_id}` warnings baseline=`{warnings.get('baseline_last_warning')}` candidate=`{warnings.get('candidate_last_warning')}`"
                )
        lines.extend(['', '## Control Root Causes'])
        if comparison['control']['root_cause_candidates']:
            for item in comparison['control']['root_cause_candidates']:
                lines.append(f"- `{item.get('drone_id')}` {item.get('title')}: {item.get('detail')}")
        else:
            lines.append('- No strong control-specific deltas detected.')
        lines.append('')
        return '\n'.join(lines)


def main() -> int:
    args = parse_args()
    comparison_tool = ProfileComparison(args.baseline, args.candidate)
    comparison = comparison_tool.compare()

    output_dir = args.candidate.resolve() / 'diagnostics' / 'profile'
    comparison_json = output_dir / 'comparison.json'
    comparison_md = output_dir / 'comparison.md'
    comparison_json.write_text(json.dumps(comparison, indent=2) + '\n')
    comparison_md.write_text(comparison_tool.render_markdown(comparison))
    print(comparison_json)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
