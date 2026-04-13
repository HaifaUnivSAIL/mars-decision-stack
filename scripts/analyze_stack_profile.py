#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import statistics
from bisect import bisect_left
from pathlib import Path
from typing import Any


PROFILE_PATTERN = re.compile(r'PROFILE:(\{.*\})')
ALTITUDE_THRESHOLD_M = 1.0
MOTION_DELTA_THRESHOLD_M = 0.15
SERVO_DELTA_THRESHOLD = 5.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Analyze one profiled visual stack run.')
    parser.add_argument('--run-dir', required=True, type=Path)
    return parser.parse_args()


def load_json(path: Path, default=None):
    if not path.exists():
        return {} if default is None else default
    return json.loads(path.read_text())


def safe_float(value: Any) -> float | None:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        return float(text)
    except Exception:
        return None


def safe_int(value: Any) -> int | None:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        return int(text)
    except Exception:
        return None


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline='') as handle:
        return list(csv.DictReader(handle))


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    if len(values) == 1:
        return values[0]
    ordered = sorted(values)
    pos = (len(ordered) - 1) * q
    lower = math.floor(pos)
    upper = math.ceil(pos)
    if lower == upper:
        return ordered[lower]
    frac = pos - lower
    return ordered[lower] * (1.0 - frac) + ordered[upper] * frac


def series_stats(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {'count': 0, 'mean': None, 'median': None, 'p95': None, 'min': None, 'max': None}
    return {
        'count': len(values),
        'mean': statistics.fmean(values),
        'median': statistics.median(values),
        'p95': percentile(values, 0.95),
        'min': min(values),
        'max': max(values),
    }


class SimTimeInterpolator:
    def __init__(self, world_rows: list[dict[str, str]]):
        points: list[tuple[float, float]] = []
        for row in world_rows:
            wall = safe_float(row.get('wall_time_epoch_sec'))
            sim = safe_float(row.get('sim_time_sec'))
            if wall is None or sim is None:
                continue
            points.append((wall, sim))
        points.sort()
        self._walls = [point[0] for point in points]
        self._sims = [point[1] for point in points]

    def at(self, wall_time: float) -> float | None:
        if not self._walls:
            return None
        idx = bisect_left(self._walls, wall_time)
        if idx <= 0:
            return self._sims[0]
        if idx >= len(self._walls):
            return self._sims[-1]
        left_wall = self._walls[idx - 1]
        right_wall = self._walls[idx]
        left_sim = self._sims[idx - 1]
        right_sim = self._sims[idx]
        if right_wall <= left_wall:
            return left_sim
        frac = (wall_time - left_wall) / (right_wall - left_wall)
        return left_sim + frac * (right_sim - left_sim)


class ProfileAnalyzer:
    def __init__(self, run_dir: Path):
        self.run_dir = run_dir.resolve()
        self.profile_dir = self.run_dir / 'diagnostics' / 'profile'
        self.manifest = load_json(self.profile_dir / 'profile.manifest.json')
        self.summary_path = self.profile_dir / 'summary.json'
        self.summary_md_path = self.profile_dir / 'summary.md'
        self.component_events_path = self.profile_dir / 'component_events.csv'
        self.ros_events = [self._normalize_ros_event(row) for row in read_csv_rows(self.profile_dir / 'ros_events.csv')]
        self.pose_rows = read_csv_rows(self.profile_dir / 'pose_trace.csv')
        self.world_rows = read_csv_rows(self.profile_dir / 'world_stats.csv')
        self.camera_rows = read_csv_rows(self.profile_dir / 'camera_frames.csv')
        self.container_rows = read_csv_rows(self.profile_dir / 'container_stats.csv')
        self.gpu_rows = read_csv_rows(self.profile_dir / 'gpu_stats.csv')
        self.focus_rows = read_csv_rows(self.profile_dir / 'focus_events.csv')
        self.sim_time = SimTimeInterpolator(self.world_rows)
        self.component_events = self._extract_component_events()

    def _normalize_ros_event(self, row: dict[str, str]) -> dict[str, Any]:
        payload = {}
        raw_payload = row.get('payload_json', '')
        if raw_payload:
            try:
                payload = json.loads(raw_payload)
            except Exception:
                payload = {'raw_payload': raw_payload}
        return {
            **row,
            'wall_time_epoch_sec': safe_float(row.get('wall_time_epoch_sec')),
            'ros_time_sec': safe_float(row.get('ros_time_sec')),
            'payload': payload,
        }

    def _extract_component_events(self) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        log_roots = [self.run_dir / 'logs', self.run_dir / 'analysis']
        for root in log_roots:
            if not root.exists():
                continue
            for path in sorted(root.rglob('*.log')):
                drone_id = ''
                rel_parts = path.relative_to(root).parts
                if rel_parts and rel_parts[0].startswith('drone_'):
                    drone_id = rel_parts[0]
                with path.open(errors='replace') as handle:
                    for line in handle:
                        match = PROFILE_PATTERN.search(line)
                        if not match:
                            continue
                        try:
                            payload = json.loads(match.group(1))
                        except Exception:
                            continue
                        rows.append(
                            {
                                'wall_time_epoch_sec': safe_float(payload.get('wall_time_epoch_sec')),
                                'component': payload.get('component', ''),
                                'event_type': payload.get('event_type', ''),
                                'drone_id': payload.get('drone_id', drone_id),
                                'payload_json': json.dumps(payload, separators=(',', ':')),
                                'source_file': str(path.relative_to(self.run_dir)),
                                'payload': payload,
                            }
                        )
        rows.sort(key=lambda row: (row['wall_time_epoch_sec'] or 0.0, row['component'], row['event_type']))
        with self.component_events_path.open('w', newline='') as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=['wall_time_epoch_sec', 'component', 'event_type', 'drone_id', 'payload_json', 'source_file'],
            )
            writer.writeheader()
            for row in rows:
                writer.writerow({k: row[k] for k in writer.fieldnames})
        return rows

    def _rows_for_drone(self, rows: list[dict[str, Any]], drone_id: str, key: str = 'drone_id') -> list[dict[str, Any]]:
        return [row for row in rows if str(row.get(key, '')) == drone_id]

    def _ros_event_time(self, row: dict[str, Any]) -> float | None:
        return row.get('wall_time_epoch_sec')

    def _first_event_time(self, rows: list[dict[str, Any]], predicate) -> float | None:
        for row in rows:
            if predicate(row):
                return self._ros_event_time(row)
        return None

    def _startup_metrics(self, drone_id: str) -> dict[str, float | None]:
        rows = self._rows_for_drone(self.ros_events, drone_id)
        first_row_time = self._first_event_time(rows, lambda _row: True)
        mc_ready = self._first_event_time(
            rows,
            lambda row: row.get('event_type') == 'mission_control_state' and row['payload'].get('mc_state_name') == 'Standby',
        )
        hlc_ready = self._first_event_time(
            rows,
            lambda row: row.get('event_type') == 'high_level_control_state' and row['payload'].get('hlc_state_name') == 'Ready',
        )
        return {
            'first_event_wall_time_sec': first_row_time,
            'mc_standby_wall_time_sec': mc_ready,
            'hlc_ready_wall_time_sec': hlc_ready,
            'startup_time_to_mc_standby_sec': None if mc_ready is None or first_row_time is None else mc_ready - first_row_time,
            'startup_time_to_hlc_ready_sec': None if hlc_ready is None or first_row_time is None else hlc_ready - first_row_time,
        }

    def _operator_commands(self, drone_id: str) -> list[dict[str, Any]]:
        rows = self._rows_for_drone(self.ros_events, drone_id)
        return [
            row for row in rows
            if row.get('event_type') == 'input_operator_command' and row['payload'].get('op_com_name') in {'takeoff', 'land', 'gohome'}
        ]

    def _velocity_commands(self, drone_id: str) -> list[dict[str, Any]]:
        rows = self._rows_for_drone(self.ros_events, drone_id)
        return [
            row for row in rows
            if row.get('event_type') == 'input_velocity' and any(abs(float(row['payload'].get(axis, 0.0))) > 1e-6 for axis in ('linear_x', 'linear_y', 'linear_z'))
        ]

    def _servo_rows(self, drone_id: str) -> list[dict[str, Any]]:
        return read_csv_rows(self.profile_dir / 'mavlink' / drone_id / 'servo_output_raw.csv')

    def _position_rows(self, drone_id: str) -> list[dict[str, Any]]:
        return [row for row in self.pose_rows if row.get('drone_id') == drone_id]

    def _first_servo_change_latency(self, drone_id: str, command_time: float) -> dict[str, float | None]:
        rows = self._servo_rows(drone_id)
        if not rows:
            return {'wall_sec': None, 'sim_sec': None}
        baseline = None
        for row in rows:
            host = safe_float(row.get('host_time_sec'))
            if host is not None and host <= command_time:
                baseline = row
            if host is not None and host > command_time:
                break
        if baseline is None:
            baseline = rows[0]
        baseline_values = [safe_float(baseline.get(f'servo{i}_raw')) or 0.0 for i in range(1, 5)]
        for row in rows:
            host = safe_float(row.get('host_time_sec'))
            if host is None or host < command_time:
                continue
            values = [safe_float(row.get(f'servo{i}_raw')) or 0.0 for i in range(1, 5)]
            if max(abs(values[i] - baseline_values[i]) for i in range(4)) >= SERVO_DELTA_THRESHOLD:
                sim_start = self.sim_time.at(command_time)
                sim_end = self.sim_time.at(host)
                return {
                    'wall_sec': host - command_time,
                    'sim_sec': None if sim_start is None or sim_end is None else sim_end - sim_start,
                }
        return {'wall_sec': None, 'sim_sec': None}

    def _first_motion_latency(self, drone_id: str, command_time: float) -> dict[str, float | None]:
        rows = self._position_rows(drone_id)
        if not rows:
            return {'wall_sec': None, 'sim_sec': None}
        baseline = None
        for row in rows:
            wall = safe_float(row.get('wall_time_epoch_sec'))
            if wall is not None and wall <= command_time:
                baseline = row
            if wall is not None and wall > command_time:
                break
        if baseline is None:
            baseline = rows[0]
        base_x = safe_float(baseline.get('x')) or 0.0
        base_y = safe_float(baseline.get('y')) or 0.0
        base_z = safe_float(baseline.get('z')) or 0.0
        for row in rows:
            wall = safe_float(row.get('wall_time_epoch_sec'))
            if wall is None or wall < command_time:
                continue
            x = safe_float(row.get('x')) or 0.0
            y = safe_float(row.get('y')) or 0.0
            z = safe_float(row.get('z')) or 0.0
            delta = math.sqrt((x - base_x) ** 2 + (y - base_y) ** 2 + (z - base_z) ** 2)
            if delta >= MOTION_DELTA_THRESHOLD_M:
                sim_start = self.sim_time.at(command_time)
                sim_end = safe_float(row.get('sim_time_sec')) or self.sim_time.at(wall)
                return {
                    'wall_sec': wall - command_time,
                    'sim_sec': None if sim_start is None or sim_end is None else sim_end - sim_start,
                }
        return {'wall_sec': None, 'sim_sec': None}

    def _state_transition_latency(self, drone_id: str, command_time: float) -> dict[str, float | None]:
        rows = self._rows_for_drone(self.ros_events, drone_id)
        for row in rows:
            wall = row.get('wall_time_epoch_sec')
            if wall is None or wall < command_time:
                continue
            if row.get('event_type') in {'mission_control_state', 'high_level_control_state'}:
                sim_start = self.sim_time.at(command_time)
                sim_end = self.sim_time.at(wall)
                return {
                    'wall_sec': wall - command_time,
                    'sim_sec': None if sim_start is None or sim_end is None else sim_end - sim_start,
                    'event_type': row.get('event_type'),
                }
        return {'wall_sec': None, 'sim_sec': None, 'event_type': None}

    def _takeoff_duration(self, drone_id: str) -> dict[str, float | None]:
        takeoff_events = [event for event in self._operator_commands(drone_id) if event['payload'].get('op_com_name') == 'takeoff']
        if not takeoff_events:
            return {'wall_sec': None, 'sim_sec': None}
        start = takeoff_events[-1]['wall_time_epoch_sec']
        rows = self._rows_for_drone(self.ros_events, drone_id)
        for row in rows:
            wall = row.get('wall_time_epoch_sec')
            if wall is None or wall < start:
                continue
            if row.get('event_type') == 'hlc_telemetry' and float(row['payload'].get('altitude', 0.0)) >= ALTITUDE_THRESHOLD_M:
                sim_start = self.sim_time.at(start)
                sim_end = self.sim_time.at(wall)
                return {
                    'wall_sec': wall - start,
                    'sim_sec': None if sim_start is None or sim_end is None else sim_end - sim_start,
                }
        return {'wall_sec': None, 'sim_sec': None}

    def _landing_duration(self, drone_id: str) -> dict[str, float | None]:
        landing_events = [event for event in self._operator_commands(drone_id) if event['payload'].get('op_com_name') == 'land']
        if not landing_events:
            return {'wall_sec': None, 'sim_sec': None}
        start = landing_events[-1]['wall_time_epoch_sec']
        rows = self._rows_for_drone(self.ros_events, drone_id)
        for row in rows:
            wall = row.get('wall_time_epoch_sec')
            if wall is None or wall < start:
                continue
            if row.get('event_type') == 'mission_control_state' and row['payload'].get('mc_state_name') == 'Standby':
                sim_start = self.sim_time.at(start)
                sim_end = self.sim_time.at(wall)
                return {
                    'wall_sec': wall - start,
                    'sim_sec': None if sim_start is None or sim_end is None else sim_end - sim_start,
                }
        return {'wall_sec': None, 'sim_sec': None}

    def _camera_summary(self) -> dict[str, Any]:
        streams: dict[str, list[dict[str, str]]] = {}
        for row in self.camera_rows:
            streams.setdefault(row.get('stream_name', 'unknown'), []).append(row)
        result = {}
        for stream_name, rows in streams.items():
            gaps = [safe_float(row.get('interframe_gap_sec')) for row in rows]
            gaps = [gap for gap in gaps if gap is not None and gap > 0.0]
            fps_values = [1.0 / gap for gap in gaps if gap > 0.0]
            backlog_values = [safe_float(row.get('stream_backlog_sec')) for row in rows]
            backlog_values = [value for value in backlog_values if value is not None]
            result[stream_name] = {
                'fps': series_stats(fps_values),
                'backlog_sec': series_stats(backlog_values),
                'frame_count': len(rows),
            }
        return result

    def _world_summary(self) -> dict[str, Any]:
        rtfs = [safe_float(row.get('real_time_factor')) for row in self.world_rows]
        rtfs = [value for value in rtfs if value is not None]
        step_sizes = [safe_float(row.get('step_size_sec')) for row in self.world_rows]
        step_sizes = [value for value in step_sizes if value is not None]
        return {
            'real_time_factor': series_stats(rtfs),
            'step_size_sec': series_stats(step_sizes),
            'sample_count': len(self.world_rows),
        }

    def _pose_summary(self, drone_id: str) -> dict[str, Any]:
        rows = self._position_rows(drone_id)
        backlog_values = [safe_float(row.get('pose_backlog_sec')) for row in rows]
        backlog_values = [value for value in backlog_values if value is not None]
        altitudes = [safe_float(row.get('z')) for row in rows]
        altitudes = [value for value in altitudes if value is not None]
        return {
            'pose_backlog_sec': series_stats(backlog_values),
            'altitude_m': series_stats(altitudes),
            'sample_count': len(rows),
        }

    def _container_summary(self) -> dict[str, Any]:
        by_name: dict[str, list[dict[str, str]]] = {}
        for row in self.container_rows:
            by_name.setdefault(row.get('container_name', ''), []).append(row)
        result = {}
        for name, rows in by_name.items():
            cpu = [safe_float(row.get('cpu_percent')) for row in rows]
            cpu = [value for value in cpu if value is not None]
            mem = [safe_float(row.get('mem_usage_bytes')) for row in rows]
            mem = [value for value in mem if value is not None]
            result[name] = {
                'cpu_percent': series_stats(cpu),
                'mem_usage_bytes': series_stats(mem),
                'sample_count': len(rows),
            }
        return result

    def _gpu_summary(self) -> dict[str, Any]:
        util = [safe_float(row.get('utilization_gpu_percent')) for row in self.gpu_rows]
        util = [value for value in util if value is not None]
        mem = [safe_float(row.get('memory_used_mib')) for row in self.gpu_rows]
        mem = [value for value in mem if value is not None]
        return {
            'utilization_gpu_percent': series_stats(util),
            'memory_used_mib': series_stats(mem),
            'sample_count': len(self.gpu_rows),
        }

    def _drone_summary(self, drone_id: str) -> dict[str, Any]:
        startup = self._startup_metrics(drone_id)
        operator_commands = self._operator_commands(drone_id)
        velocity_commands = self._velocity_commands(drone_id)
        latest_velocity = velocity_commands[-1]['wall_time_epoch_sec'] if velocity_commands else None
        latest_operator = operator_commands[-1]['wall_time_epoch_sec'] if operator_commands else None
        reference_time = latest_velocity if latest_velocity is not None else latest_operator
        state_latency = self._state_transition_latency(drone_id, reference_time) if reference_time is not None else {'wall_sec': None, 'sim_sec': None, 'event_type': None}
        actuator_latency = self._first_servo_change_latency(drone_id, latest_velocity) if latest_velocity is not None else {'wall_sec': None, 'sim_sec': None}
        motion_latency = self._first_motion_latency(drone_id, latest_velocity) if latest_velocity is not None else {'wall_sec': None, 'sim_sec': None}
        return {
            **startup,
            'operator_command_count': len(operator_commands),
            'velocity_command_count': len(velocity_commands),
            'command_to_state_transition_latency': state_latency,
            'command_to_first_actuator_change_latency': actuator_latency,
            'command_to_first_motion_latency': motion_latency,
            'takeoff_duration_to_altitude_threshold': self._takeoff_duration(drone_id),
            'landing_duration_to_standby': self._landing_duration(drone_id),
            'pose': self._pose_summary(drone_id),
        }

    def _bottlenecks(self, summary: dict[str, Any]) -> list[dict[str, Any]]:
        findings: list[dict[str, Any]] = []
        world_rtf = summary.get('world', {}).get('real_time_factor', {}).get('median')
        if world_rtf is not None and world_rtf < 0.7:
            findings.append({'priority': 1, 'title': 'Simulator RTF collapse', 'detail': f'median real_time_factor={world_rtf:.3f}'})
        for stream_name, stream_summary in summary.get('camera', {}).items():
            backlog = stream_summary.get('backlog_sec', {}).get('p95')
            fps = stream_summary.get('fps', {}).get('median')
            if backlog is not None and backlog > 1.0:
                findings.append({'priority': 2, 'title': f'Active camera backlog growth ({stream_name})', 'detail': f'p95 backlog={backlog:.3f}s'})
            if fps is not None and fps < 5.0:
                findings.append({'priority': 3, 'title': f'Low active camera FPS ({stream_name})', 'detail': f'median fps={fps:.2f}'})
        for container_name, container_summary in summary.get('containers', {}).items():
            cpu_peak = container_summary.get('cpu_percent', {}).get('max')
            if cpu_peak is not None and cpu_peak > 150.0:
                findings.append({'priority': 4, 'title': f'High container CPU pressure ({container_name})', 'detail': f'peak cpu={cpu_peak:.1f}%'})
        for drone_id, drone_summary in summary.get('drones', {}).items():
            motion_latency = drone_summary.get('command_to_first_motion_latency', {}).get('wall_sec')
            if motion_latency is not None and motion_latency > 2.0:
                findings.append({'priority': 5, 'title': f'Slow command-to-motion response ({drone_id})', 'detail': f'wall latency={motion_latency:.3f}s'})
        findings.sort(key=lambda row: (row['priority'], row['title']))
        return findings

    def analyze(self) -> dict[str, Any]:
        drone_ids = [drone['id'] for drone in self.manifest.get('drones', [])]
        summary = {
            'run_id': self.manifest.get('run_id'),
            'mode': self.manifest.get('mode'),
            'fleet': bool(self.manifest.get('fleet', False)),
            'visual_gui': bool(self.manifest.get('visual_gui', False)),
            'drones': {drone_id: self._drone_summary(drone_id) for drone_id in drone_ids},
            'world': self._world_summary(),
            'camera': self._camera_summary(),
            'containers': self._container_summary(),
            'gpu': self._gpu_summary(),
            'focus_event_count': len(self.focus_rows),
        }
        summary['bottlenecks'] = self._bottlenecks(summary)
        self.summary_path.write_text(json.dumps(summary, indent=2) + '\n')
        self.summary_md_path.write_text(self._render_markdown(summary))
        return summary

    def _render_markdown(self, summary: dict[str, Any]) -> str:
        lines = ['# Stack Profile Summary', '', f"Run: `{summary.get('run_id', '')}`", f"Mode: `{summary.get('mode', '')}`", '']
        lines.append('## Bottlenecks')
        if summary['bottlenecks']:
            for item in summary['bottlenecks']:
                lines.append(f"- {item['title']}: {item['detail']}")
        else:
            lines.append('- No obvious bottlenecks detected from the collected metrics.')
        lines.append('')
        lines.append('## World')
        world_rtf = summary.get('world', {}).get('real_time_factor', {})
        lines.append(f"- Real-time factor median/p95/min: `{world_rtf.get('median')}` / `{world_rtf.get('p95')}` / `{world_rtf.get('min')}`")
        lines.append('')
        lines.append('## Camera Streams')
        for stream_name, stream_summary in summary.get('camera', {}).items():
            lines.append(
                f"- `{stream_name}` fps median={stream_summary['fps'].get('median')} backlog p95={stream_summary['backlog_sec'].get('p95')} frame_count={stream_summary.get('frame_count')}"
            )
        lines.append('')
        lines.append('## Per Drone')
        for drone_id, drone_summary in summary.get('drones', {}).items():
            lines.append(f"- `{drone_id}` startup(mc/hlc)={drone_summary.get('startup_time_to_mc_standby_sec')} / {drone_summary.get('startup_time_to_hlc_ready_sec')} s")
            lines.append(f"- `{drone_id}` command->state={drone_summary['command_to_state_transition_latency'].get('wall_sec')} s")
            lines.append(f"- `{drone_id}` command->actuator={drone_summary['command_to_first_actuator_change_latency'].get('wall_sec')} s")
            lines.append(f"- `{drone_id}` command->motion={drone_summary['command_to_first_motion_latency'].get('wall_sec')} s")
            lines.append(f"- `{drone_id}` takeoff/landing={drone_summary['takeoff_duration_to_altitude_threshold'].get('wall_sec')} / {drone_summary['landing_duration_to_standby'].get('wall_sec')} s")
            lines.append(f"- `{drone_id}` pose backlog p95={drone_summary['pose']['pose_backlog_sec'].get('p95')}")
        lines.append('')
        lines.append('## GPU')
        lines.append(
            f"- GPU util peak={summary.get('gpu', {}).get('utilization_gpu_percent', {}).get('max')} memory peak MiB={summary.get('gpu', {}).get('memory_used_mib', {}).get('max')}"
        )
        lines.append('')
        return '\n'.join(lines) + '\n'


def main() -> int:
    args = parse_args()
    analyzer = ProfileAnalyzer(args.run_dir)
    analyzer.analyze()
    print(analyzer.summary_path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
