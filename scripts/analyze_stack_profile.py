#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
from bisect import bisect_left
from pathlib import Path
from typing import Any


PROFILE_PATTERN = re.compile(r'PROFILE:(\{.*\})')
ROS_LOG_TS_PATTERN = re.compile(r'\[(\d+\.\d+)\]')
DRONE_ID_PATTERN = re.compile(r'(drone_\d+)')
ALTITUDE_THRESHOLD_M = 1.0
MOTION_DELTA_THRESHOLD_M = 0.15
SERVO_DELTA_THRESHOLD = 5.0
TAKEOFF_ALTITUDE_EPSILON_M = 0.05
YAW_SPIN_THRESHOLD_DEG = 45.0
YAW_RATE_SPIN_THRESHOLD_RAD_S = 0.8
IDLE_HORIZONTAL_DRIFT_THRESHOLD_M = 0.05
IDLE_VERTICAL_DRIFT_THRESHOLD_M = 0.02
IDLE_YAW_DRIFT_THRESHOLD_DEG = 2.0
CONTROL_COMPONENTS = {
    'manual_runtime_test',
    'command_publisher',
    'scenario_node',
    'ros_alate',
    'mc',
    'hlc',
    'autopilot',
    'autopilot_bridge',
    'autopilot_dronekit',
}


MC_STATE_NAMES = {
    0: 'None',
    1: 'Init',
    2: 'Standby',
    3: 'TakingOff',
    4: 'PerformingMission',
    5: 'Landing',
    6: 'Manual',
    7: 'ReturnToLaunch',
    8: 'NoError',
    9: 'Error',
}

HLC_STATE_NAMES = {
    0: 'None',
    1: 'Init',
    2: 'WaitingForMc',
    3: 'WaitingForLlc',
    4: 'Ready',
    5: 'Takeoff',
    6: 'GainingAltitude',
    7: 'Airborne',
    8: 'Landing',
    9: 'Manual',
    10: 'ReturnToLaunch',
    11: 'NoError',
    12: 'LowBattery',
    13: 'LlcDown',
}


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


def safe_bool(value: Any) -> bool | None:
    if value is None:
        return None
    text = str(value).strip().lower()
    if not text:
        return None
    if text in {'1', 'true', 'yes'}:
        return True
    if text in {'0', 'false', 'no'}:
        return False
    return None


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline='') as handle:
        return list(csv.DictReader(handle))


def write_csv_rows(path: Path, fieldnames: list[str], rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({name: row.get(name, '') for name in fieldnames})


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


def wrap_angle_delta(current: float, baseline: float) -> float:
    delta = current - baseline
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return delta


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
        self.control_summary_path = self.profile_dir / 'control_summary.json'
        self.control_summary_md_path = self.profile_dir / 'control_summary.md'
        self.component_events_path = self.profile_dir / 'component_events.csv'
        self.control_events_path = self.profile_dir / 'control_events.csv'
        self.control_windows_path = self.profile_dir / 'control_windows.csv'

        self.ros_events = [self._normalize_ros_event(row) for row in read_csv_rows(self.profile_dir / 'ros_events.csv')]
        self.pose_rows = read_csv_rows(self.profile_dir / 'pose_trace.csv')
        self.world_rows = read_csv_rows(self.profile_dir / 'world_stats.csv')
        self.camera_rows = read_csv_rows(self.profile_dir / 'camera_frames.csv')
        self.container_rows = read_csv_rows(self.profile_dir / 'container_stats.csv')
        self.gpu_rows = read_csv_rows(self.profile_dir / 'gpu_stats.csv')
        self.focus_rows = read_csv_rows(self.profile_dir / 'focus_events.csv')
        self.sim_time = SimTimeInterpolator(self.world_rows)
        self._mavlink_cache: dict[tuple[str, str], list[dict[str, Any]]] = {}
        self.component_events = self._extract_component_events()

    def _default_drone_id(self) -> str:
        drones = self.manifest.get('drones', [])
        if drones:
            return drones[0]['id']
        return 'drone_1'

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

    def _infer_drone_id(self, root: Path, path: Path) -> str:
        rel_parts = path.relative_to(root).parts
        if rel_parts and rel_parts[0].startswith('drone_'):
            return rel_parts[0]
        match = DRONE_ID_PATTERN.search(path.name)
        if match:
            return match.group(1)
        if len(self.manifest.get('drones', [])) == 1:
            return self._default_drone_id()
        return ''

    def _extract_component_events(self) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        log_roots = [self.run_dir / 'logs', self.run_dir / 'analysis']
        for root in log_roots:
            if not root.exists():
                continue
            for path in sorted(root.rglob('*.log')):
                drone_id = self._infer_drone_id(root, path)
                with path.open(errors='replace') as handle:
                    for line in handle:
                        match = PROFILE_PATTERN.search(line)
                        if not match:
                            continue
                        try:
                            payload = json.loads(match.group(1))
                        except Exception:
                            continue
                        wall_time = safe_float(payload.get('wall_time_epoch_sec'))
                        if wall_time is None:
                            ts_match = ROS_LOG_TS_PATTERN.search(line)
                            if ts_match:
                                wall_time = safe_float(ts_match.group(1))
                        if wall_time is None:
                            continue
                        payload['wall_time_epoch_sec'] = wall_time
                        rows.append(
                            {
                                'wall_time_epoch_sec': wall_time,
                                'component': payload.get('component', ''),
                                'event_type': payload.get('event_type', ''),
                                'drone_id': payload.get('drone_id', drone_id),
                                'payload_json': json.dumps(payload, separators=(',', ':')),
                                'source_file': str(path.relative_to(self.run_dir)),
                                'payload': payload,
                            }
                        )
        rows.sort(key=lambda row: (row['wall_time_epoch_sec'] or 0.0, row['component'], row['event_type']))
        write_csv_rows(
            self.component_events_path,
            ['wall_time_epoch_sec', 'component', 'event_type', 'drone_id', 'payload_json', 'source_file'],
            rows,
        )
        return rows

    def _rows_for_drone(self, rows: list[dict[str, Any]], drone_id: str, key: str = 'drone_id') -> list[dict[str, Any]]:
        return [row for row in rows if str(row.get(key, '')) == drone_id]

    def _mavlink_rows(self, drone_id: str, filename: str) -> list[dict[str, Any]]:
        key = (drone_id, filename)
        if key not in self._mavlink_cache:
            path = self.profile_dir / 'mavlink' / drone_id / filename
            normalized = []
            for row in read_csv_rows(path):
                normalized_row = dict(row)
                normalized_row['host_time_sec'] = safe_float(row.get('host_time_sec'))
                normalized.append(normalized_row)
            self._mavlink_cache[key] = normalized
        return self._mavlink_cache[key]

    def _servo_rows(self, drone_id: str) -> list[dict[str, Any]]:
        return self._mavlink_rows(drone_id, 'servo_output_raw.csv')

    def _attitude_rows(self, drone_id: str) -> list[dict[str, Any]]:
        return self._mavlink_rows(drone_id, 'attitude.csv')

    def _local_position_rows(self, drone_id: str) -> list[dict[str, Any]]:
        return self._mavlink_rows(drone_id, 'local_position_ned.csv')

    def _heartbeat_rows(self, drone_id: str) -> list[dict[str, Any]]:
        rows = []
        for row in self._mavlink_rows(drone_id, 'heartbeat.csv'):
            rows.append(
                {
                    **row,
                    'armed': safe_bool(row.get('armed')),
                    'mode_string': row.get('mode_string', ''),
                    'system_status': safe_int(row.get('system_status')),
                }
            )
        return rows

    def _statustext_rows(self, drone_id: str) -> list[dict[str, Any]]:
        rows = []
        for row in self._mavlink_rows(drone_id, 'statustext.csv'):
            rows.append(
                {
                    **row,
                    'severity': safe_int(row.get('severity')),
                    'text': row.get('text', ''),
                }
            )
        return rows

    def _position_rows(self, drone_id: str) -> list[dict[str, Any]]:
        result = []
        for row in self.pose_rows:
            if row.get('drone_id') != drone_id:
                continue
            result.append(
                {
                    **row,
                    'wall_time_epoch_sec': safe_float(row.get('wall_time_epoch_sec')),
                    'sim_time_sec': safe_float(row.get('sim_time_sec')),
                    'x': safe_float(row.get('x')),
                    'y': safe_float(row.get('y')),
                    'z': safe_float(row.get('z')),
                    'yaw': safe_float(row.get('yaw')),
                    'pose_backlog_sec': safe_float(row.get('pose_backlog_sec')),
                }
            )
        return result

    def _telemetry_rows(self, drone_id: str) -> list[dict[str, Any]]:
        result = []
        for row in self._rows_for_drone(self.ros_events, drone_id):
            if row.get('event_type') != 'hlc_telemetry':
                continue
            result.append(
                {
                    **row,
                    'altitude': safe_float(row['payload'].get('altitude')),
                    'yaw': safe_float(row['payload'].get('yaw')),
                    'armed': bool(row['payload'].get('armed', False)),
                    'mode': row['payload'].get('mode', ''),
                    'state': row['payload'].get('state', ''),
                }
            )
        result.sort(key=lambda row: row.get('wall_time_epoch_sec') or 0.0)
        return result

    def _operator_commands(self, drone_id: str) -> list[dict[str, Any]]:
        rows = self._rows_for_drone(self.ros_events, drone_id)
        commands = [
            row
            for row in rows
            if row.get('event_type') == 'input_operator_command' and row['payload'].get('op_com_name') in {'takeoff', 'land', 'gohome'}
        ]
        commands.sort(key=lambda row: row.get('wall_time_epoch_sec') or 0.0)
        return commands

    def _all_operator_commands(self) -> list[dict[str, Any]]:
        commands = [
            row
            for row in self.ros_events
            if row.get('event_type') == 'input_operator_command' and row['payload'].get('op_com_name') in {'takeoff', 'land', 'gohome'}
        ]
        commands.sort(key=lambda row: row.get('wall_time_epoch_sec') or 0.0)
        return commands

    def _velocity_commands(self, drone_id: str) -> list[dict[str, Any]]:
        rows = self._rows_for_drone(self.ros_events, drone_id)
        commands = [
            row
            for row in rows
            if row.get('event_type') == 'input_velocity'
            and any(abs(float(row['payload'].get(axis, 0.0))) > 1e-6 for axis in ('linear_x', 'linear_y', 'linear_z'))
        ]
        commands.sort(key=lambda row: row.get('wall_time_epoch_sec') or 0.0)
        return commands

    def _first_event_time(self, rows: list[dict[str, Any]], predicate) -> float | None:
        for row in rows:
            if predicate(row):
                return row.get('wall_time_epoch_sec')
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

    def _first_servo_change_latency(self, drone_id: str, command_time: float) -> dict[str, float | None]:
        rows = self._servo_rows(drone_id)
        if not rows:
            return {'wall_sec': None, 'sim_sec': None}
        baseline = None
        for row in rows:
            host = row.get('host_time_sec')
            if host is not None and host <= command_time:
                baseline = row
            if host is not None and host > command_time:
                break
        if baseline is None:
            baseline = rows[0]
        baseline_values = [safe_float(baseline.get(f'servo{i}_raw')) or 0.0 for i in range(1, 5)]
        for row in rows:
            host = row.get('host_time_sec')
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
            wall = row.get('wall_time_epoch_sec')
            if wall is not None and wall <= command_time:
                baseline = row
            if wall is not None and wall > command_time:
                break
        if baseline is None:
            baseline = rows[0]
        base_x = baseline.get('x') or 0.0
        base_y = baseline.get('y') or 0.0
        base_z = baseline.get('z') or 0.0
        for row in rows:
            wall = row.get('wall_time_epoch_sec')
            if wall is None or wall < command_time:
                continue
            x = row.get('x') or 0.0
            y = row.get('y') or 0.0
            z = row.get('z') or 0.0
            delta = math.sqrt((x - base_x) ** 2 + (y - base_y) ** 2 + (z - base_z) ** 2)
            if delta >= MOTION_DELTA_THRESHOLD_M:
                sim_start = self.sim_time.at(command_time)
                sim_end = row.get('sim_time_sec') or self.sim_time.at(wall)
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
        backlog_values = [row.get('pose_backlog_sec') for row in rows]
        backlog_values = [value for value in backlog_values if value is not None]
        altitudes = [row.get('z') for row in rows]
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
        idle_drift = self._idle_drift_summary(drone_id, startup['hlc_ready_wall_time_sec'])
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
            **idle_drift,
        }

    def _drone_tail_time(self, drone_id: str) -> float | None:
        tail_candidates: list[float] = []
        for rows in (
            self._rows_for_drone(self.ros_events, drone_id),
            self._rows_for_drone(self.component_events, drone_id),
            self._position_rows(drone_id),
            self._telemetry_rows(drone_id),
            self._heartbeat_rows(drone_id),
            self._statustext_rows(drone_id),
            self._attitude_rows(drone_id),
            self._local_position_rows(drone_id),
        ):
            if not rows:
                continue
            tail = rows[-1].get('wall_time_epoch_sec')
            if tail is None:
                tail = rows[-1].get('host_time_sec')
            if tail is not None:
                tail_candidates.append(tail)
        return max(tail_candidates) if tail_candidates else None

    def _window_end_time(self, drone_id: str, start_time: float, operator_commands: list[dict[str, Any]]) -> float:
        next_command_time = None
        for command in operator_commands:
            wall = command.get('wall_time_epoch_sec')
            if wall is not None and wall > start_time:
                next_command_time = wall
                break
        if next_command_time is not None:
            return next_command_time
        tail_time = self._drone_tail_time(drone_id)
        if tail_time is not None:
            return tail_time
        return start_time + 60.0

    def _idle_drift_thresholds(self) -> dict[str, float]:
        return {
            'horizontal_drift_m': IDLE_HORIZONTAL_DRIFT_THRESHOLD_M,
            'vertical_drift_m': IDLE_VERTICAL_DRIFT_THRESHOLD_M,
            'yaw_drift_deg': IDLE_YAW_DRIFT_THRESHOLD_DEG,
        }

    def _pose_drift_metrics(self, drone_id: str, start_time: float | None, end_time: float | None) -> dict[str, Any]:
        empty = {
            'horizontal_drift_m': None,
            'vertical_drift_m': None,
            'yaw_drift_deg': None,
            'pose_sample_count': 0,
        }
        if start_time is None or end_time is None or end_time < start_time:
            return empty

        all_rows = self._position_rows(drone_id)
        baseline = None
        window_rows: list[dict[str, Any]] = []
        for row in all_rows:
            wall = row.get('wall_time_epoch_sec')
            if wall is None:
                continue
            if wall <= start_time:
                baseline = row
            if start_time <= wall <= end_time:
                window_rows.append(row)
        if baseline is None and window_rows:
            baseline = window_rows[0]
        if baseline is None or not window_rows:
            return empty

        baseline_x = baseline.get('x') or 0.0
        baseline_y = baseline.get('y') or 0.0
        baseline_z = baseline.get('z') or 0.0
        baseline_yaw = baseline.get('yaw')
        max_horizontal = 0.0
        max_vertical = 0.0
        max_yaw_deg = 0.0 if baseline_yaw is not None else None
        for row in window_rows:
            x = row.get('x') or 0.0
            y = row.get('y') or 0.0
            z = row.get('z') or 0.0
            yaw = row.get('yaw')
            max_horizontal = max(max_horizontal, math.hypot(x - baseline_x, y - baseline_y))
            max_vertical = max(max_vertical, abs(z - baseline_z))
            if baseline_yaw is not None and yaw is not None:
                max_yaw_deg = max(max_yaw_deg or 0.0, abs(math.degrees(wrap_angle_delta(yaw, baseline_yaw))))
        return {
            'horizontal_drift_m': max_horizontal,
            'vertical_drift_m': max_vertical,
            'yaw_drift_deg': max_yaw_deg,
            'pose_sample_count': len(window_rows),
        }

    def _idle_servo_distinct_values(self, drone_id: str, start_time: float | None, end_time: float | None) -> dict[str, list[int]]:
        if start_time is None or end_time is None or end_time < start_time:
            return {}
        values = {f'servo{i}_raw': set() for i in range(1, 5)}
        for row in self._servo_rows(drone_id):
            host = row.get('host_time_sec')
            if host is None or host < start_time or host > end_time:
                continue
            for index in range(1, 5):
                value = safe_int(row.get(f'servo{index}_raw'))
                if value is not None:
                    values[f'servo{index}_raw'].add(value)
        return {key: sorted(items) for key, items in values.items() if items}

    def _idle_drift_gate(self, drift_metrics: dict[str, Any]) -> bool | None:
        horizontal = drift_metrics.get('horizontal_drift_m')
        vertical = drift_metrics.get('vertical_drift_m')
        yaw = drift_metrics.get('yaw_drift_deg')
        if horizontal is None or vertical is None or yaw is None:
            return None
        return (
            horizontal <= IDLE_HORIZONTAL_DRIFT_THRESHOLD_M
            and vertical <= IDLE_VERTICAL_DRIFT_THRESHOLD_M
            and yaw <= IDLE_YAW_DRIFT_THRESHOLD_DEG
        )

    def _idle_drift_summary(self, drone_id: str, ready_time: float | None) -> dict[str, Any]:
        all_operator_commands = self._all_operator_commands()
        first_operator_command = all_operator_commands[0]['wall_time_epoch_sec'] if all_operator_commands else self._drone_tail_time(drone_id)
        precommand_metrics = self._pose_drift_metrics(drone_id, ready_time, first_operator_command)

        foreign_command_time = None
        if not self._operator_commands(drone_id):
            for command in all_operator_commands:
                wall = command.get('wall_time_epoch_sec')
                if wall is None or command.get('drone_id') == drone_id:
                    continue
                foreign_command_time = wall
                break
        if ready_time is not None and foreign_command_time is not None:
            foreign_command_time = max(foreign_command_time, ready_time)
        uncommanded_metrics = self._pose_drift_metrics(drone_id, foreign_command_time, self._drone_tail_time(drone_id))

        return {
            'idle_drift_thresholds': self._idle_drift_thresholds(),
            'precommand_horizontal_drift_m': precommand_metrics['horizontal_drift_m'],
            'precommand_vertical_drift_m': precommand_metrics['vertical_drift_m'],
            'precommand_yaw_drift_deg': precommand_metrics['yaw_drift_deg'],
            'precommand_pose_sample_count': precommand_metrics['pose_sample_count'],
            'precommand_idle_servo_distinct_values': self._idle_servo_distinct_values(drone_id, ready_time, first_operator_command),
            'precommand_drift_gate_passed': self._idle_drift_gate(precommand_metrics),
            'uncommanded_post_foreign_command_horizontal_drift_m': uncommanded_metrics['horizontal_drift_m'],
            'uncommanded_post_foreign_command_vertical_drift_m': uncommanded_metrics['vertical_drift_m'],
            'uncommanded_post_foreign_command_yaw_drift_deg': uncommanded_metrics['yaw_drift_deg'],
            'uncommanded_post_foreign_command_drift_gate_passed': self._idle_drift_gate(uncommanded_metrics),
        }

    def _last_value_before(self, rows: list[dict[str, Any]], time_key: str, value_key: str, cutoff: float) -> Any:
        last_value = None
        for row in rows:
            wall = safe_float(row.get(time_key))
            if wall is None:
                continue
            if wall <= cutoff:
                value = row.get(value_key)
                if value is not None:
                    last_value = value
            else:
                break
        return last_value

    def _combined_altitude_samples(self, drone_id: str) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        for row in self._telemetry_rows(drone_id):
            wall = row.get('wall_time_epoch_sec')
            altitude = row.get('altitude')
            if wall is None or altitude is None:
                continue
            rows.append({'wall_time_epoch_sec': wall, 'altitude': altitude, 'source': 'ros_telemetry'})
        for row in self._local_position_rows(drone_id):
            wall = row.get('host_time_sec')
            z = safe_float(row.get('z'))
            if wall is None or z is None:
                continue
            rows.append({'wall_time_epoch_sec': wall, 'altitude': -z, 'source': 'mavlink_local_position'})
        rows.sort(key=lambda row: row['wall_time_epoch_sec'])
        return rows

    def _combined_yaw_samples(self, drone_id: str) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        for row in self._telemetry_rows(drone_id):
            wall = row.get('wall_time_epoch_sec')
            yaw = row.get('yaw')
            if wall is None or yaw is None:
                continue
            rows.append({'wall_time_epoch_sec': wall, 'yaw': yaw, 'source': 'ros_telemetry'})
        for row in self._attitude_rows(drone_id):
            wall = row.get('host_time_sec')
            yaw = safe_float(row.get('yaw'))
            if wall is None or yaw is None:
                continue
            rows.append({'wall_time_epoch_sec': wall, 'yaw': yaw, 'source': 'mavlink_attitude'})
        rows.sort(key=lambda row: row['wall_time_epoch_sec'])
        return rows

    def _yaw_rate_stats(self, drone_id: str, start: float, end: float) -> dict[str, float | None]:
        values = []
        for row in self._attitude_rows(drone_id):
            wall = row.get('host_time_sec')
            if wall is None or wall < start or wall > end:
                continue
            yaw_rate = safe_float(row.get('yawspeed'))
            if yaw_rate is None:
                continue
            values.append(abs(yaw_rate))
        return {
            'max_abs_rad_s': max(values) if values else None,
            'median_abs_rad_s': statistics.median(values) if values else None,
        }

    def _collect_warning_events(self, drone_id: str, start: float, end: float) -> list[dict[str, Any]]:
        warnings = []
        for row in self._rows_for_drone(self.ros_events, drone_id):
            wall = row.get('wall_time_epoch_sec')
            if wall is None or wall < start or wall > end:
                continue
            if row.get('event_type') == 'platform_error':
                warnings.append({'wall_time_epoch_sec': wall, 'source': 'platform_error', 'text': row['payload'].get('error_text', '')})
        for row in self._statustext_rows(drone_id):
            wall = row.get('host_time_sec')
            if wall is None or wall < start or wall > end:
                continue
            text = row.get('text', '')
            if text:
                warnings.append({'wall_time_epoch_sec': wall, 'source': 'statustext', 'text': text})
        warnings.sort(key=lambda row: row['wall_time_epoch_sec'])
        return warnings

    def _state_sequence(self, drone_id: str, start: float, end: float, event_type: str, name_key: str) -> tuple[list[str], dict[str, float]]:
        rows = [row for row in self._rows_for_drone(self.ros_events, drone_id) if row.get('event_type') == event_type]
        rows.sort(key=lambda row: row.get('wall_time_epoch_sec') or 0.0)
        current_name = None
        current_time = start
        sequence: list[str] = []
        durations: dict[str, float] = {}
        for row in rows:
            wall = row.get('wall_time_epoch_sec')
            if wall is None:
                continue
            if wall <= start:
                current_name = str(row['payload'].get(name_key, current_name or ''))
                current_time = start
                continue
            if wall > end:
                break
            if current_name:
                durations[current_name] = durations.get(current_name, 0.0) + (wall - current_time)
            current_name = str(row['payload'].get(name_key, ''))
            current_time = wall
            if current_name and (not sequence or sequence[-1] != current_name):
                sequence.append(current_name)
        if current_name:
            durations[current_name] = durations.get(current_name, 0.0) + max(end - current_time, 0.0)
            if current_name and (not sequence or sequence[-1] != current_name):
                sequence.append(current_name)
        return sequence, durations

    def _takeoff_windows(self, drone_id: str) -> list[dict[str, Any]]:
        commands = [row for row in self._operator_commands(drone_id) if row['payload'].get('op_com_name') == 'takeoff']
        operator_commands = self._operator_commands(drone_id)
        telemetry_rows = self._telemetry_rows(drone_id)
        heartbeats = self._heartbeat_rows(drone_id)
        altitude_samples = self._combined_altitude_samples(drone_id)
        yaw_samples = self._combined_yaw_samples(drone_id)
        component_rows = self._rows_for_drone(self.component_events, drone_id)
        windows: list[dict[str, Any]] = []
        for index, command in enumerate(commands, start=1):
            start = command['wall_time_epoch_sec']
            end = self._window_end_time(drone_id, start, operator_commands)
            baseline_altitude = 0.0
            for sample in altitude_samples:
                wall = sample['wall_time_epoch_sec']
                if wall is not None and wall <= start:
                    baseline_altitude = sample['altitude']
                elif wall is not None and wall > start:
                    break
            first_mc_transition = self._first_event_time(
                [row for row in self._rows_for_drone(self.ros_events, drone_id) if row.get('wall_time_epoch_sec') and start <= row['wall_time_epoch_sec'] <= end],
                lambda row: row.get('event_type') == 'mission_control_state' and row['payload'].get('mc_state_name') != 'Standby',
            )
            first_hlc_takeoff = self._first_event_time(
                [row for row in self._rows_for_drone(self.ros_events, drone_id) if row.get('wall_time_epoch_sec') and start <= row['wall_time_epoch_sec'] <= end],
                lambda row: row.get('event_type') == 'high_level_control_state' and row['payload'].get('hlc_state_name') == 'Takeoff',
            )
            armed_candidates = []
            for row in telemetry_rows:
                wall = row.get('wall_time_epoch_sec')
                if wall is not None and start <= wall <= end and row.get('armed'):
                    armed_candidates.append(wall)
            for row in heartbeats:
                wall = row.get('host_time_sec')
                if wall is not None and start <= wall <= end and row.get('armed'):
                    armed_candidates.append(wall)
            first_armed = min(armed_candidates) if armed_candidates else None

            takeoff_sent_times = [
                row['wall_time_epoch_sec']
                for row in component_rows
                if row.get('component') == 'autopilot_dronekit'
                and row.get('event_type') == 'takeoff_sent'
                and row.get('wall_time_epoch_sec') is not None
                and start <= row['wall_time_epoch_sec'] <= end
            ]
            takeoff_sent_times.sort()
            first_takeoff_resend = takeoff_sent_times[1] if len(takeoff_sent_times) > 1 else None

            altitude_increase = None
            altitude_0_2 = None
            altitude_1_0 = None
            for sample in altitude_samples:
                wall = sample['wall_time_epoch_sec']
                if wall is None or wall < start or wall > end:
                    continue
                delta = sample['altitude'] - baseline_altitude
                if altitude_increase is None and delta >= TAKEOFF_ALTITUDE_EPSILON_M:
                    altitude_increase = wall
                if altitude_0_2 is None and delta >= 0.2:
                    altitude_0_2 = wall
                if altitude_1_0 is None and delta >= 1.0:
                    altitude_1_0 = wall
            cutoff = altitude_0_2 if altitude_0_2 is not None else end

            baseline_yaw = None
            for sample in yaw_samples:
                wall = sample['wall_time_epoch_sec']
                if wall is not None and wall <= start:
                    baseline_yaw = sample['yaw']
                elif wall is not None and wall > start:
                    break
            if baseline_yaw is None and yaw_samples:
                baseline_yaw = yaw_samples[0]['yaw']
            max_yaw_delta = 0.0
            if baseline_yaw is not None:
                for sample in yaw_samples:
                    wall = sample['wall_time_epoch_sec']
                    if wall is None or wall < start or wall > cutoff:
                        continue
                    max_yaw_delta = max(max_yaw_delta, abs(wrap_angle_delta(sample['yaw'], baseline_yaw)))
            yaw_rate_stats = self._yaw_rate_stats(drone_id, start, cutoff)

            hlc_sequence, hlc_durations = self._state_sequence(drone_id, start, end, 'high_level_control_state', 'hlc_state_name')
            mc_sequence, _mc_durations = self._state_sequence(drone_id, start, end, 'mission_control_state', 'mc_state_name')
            reached_airborne = 'Airborne' in hlc_sequence
            reached_performing_mission = 'PerformingMission' in mc_sequence
            saw_landing = 'Landing' in hlc_sequence or 'Landing' in mc_sequence
            reached_success_terminal = reached_airborne or reached_performing_mission
            warnings = self._collect_warning_events(drone_id, start, end)
            last_warning = warnings[-1]['text'] if warnings else ''
            has_prearm_warning = any('prearm' in warning['text'].lower() for warning in warnings)
            actuator_latency = self._first_servo_change_latency(drone_id, start)
            motion_latency = self._first_motion_latency(drone_id, start)

            if has_prearm_warning and first_armed is None:
                outcome = 'prearm_blocked'
            elif saw_landing and not reached_success_terminal:
                outcome = 'forced_land_after_takeoff_timeout'
            elif reached_success_terminal or (altitude_1_0 is not None and not saw_landing):
                outcome = 'success_climb'
            elif first_armed is not None and (math.degrees(max_yaw_delta) >= YAW_SPIN_THRESHOLD_DEG or (yaw_rate_stats['max_abs_rad_s'] or 0.0) >= YAW_RATE_SPIN_THRESHOLD_RAD_S):
                outcome = 'yaw_spin_no_climb'
            elif first_armed is not None:
                outcome = 'armed_no_climb'
            else:
                outcome = 'insufficient_data'

            target_altitude = None
            for row in component_rows:
                wall = row.get('wall_time_epoch_sec')
                if wall is None or wall < start or wall > end:
                    continue
                payload = row.get('payload', {})
                if row.get('event_type') in {'takeoff_requested', 'takeoff_sent'}:
                    target_altitude = safe_float(payload.get('altitude'))
                    if target_altitude is not None:
                        break

            window = {
                'drone_id': drone_id,
                'window_index': index,
                'window_type': 'takeoff_attempt',
                'command_name': 'takeoff',
                'start_wall_time_sec': start,
                'end_wall_time_sec': end,
                'outcome': outcome,
                'command_to_mc_transition_sec': None if first_mc_transition is None else first_mc_transition - start,
                'command_to_hlc_takeoff_sec': None if first_hlc_takeoff is None else first_hlc_takeoff - start,
                'command_to_first_armed_sec': None if first_armed is None else first_armed - start,
                'command_to_first_takeoff_resend_sec': None if first_takeoff_resend is None else first_takeoff_resend - start,
                'command_to_first_actuator_change_sec': actuator_latency.get('wall_sec'),
                'command_to_first_motion_sec': motion_latency.get('wall_sec'),
                'command_to_first_altitude_increase_sec': None if altitude_increase is None else altitude_increase - start,
                'command_to_altitude_0_2_sec': None if altitude_0_2 is None else altitude_0_2 - start,
                'command_to_altitude_1_0_sec': None if altitude_1_0 is None else altitude_1_0 - start,
                'target_altitude_m': target_altitude,
                'max_yaw_delta_before_0_2_deg': math.degrees(max_yaw_delta),
                'max_yaw_rate_before_0_2_rad_s': yaw_rate_stats['max_abs_rad_s'],
                'time_in_takeoff_sec': hlc_durations.get('Takeoff', 0.0),
                'time_in_gaining_altitude_sec': hlc_durations.get('GainingAltitude', 0.0),
                'reached_performing_mission': reached_performing_mission,
                'reached_airborne': reached_airborne,
                'hlc_state_sequence': '|'.join(hlc_sequence),
                'mc_state_sequence': '|'.join(mc_sequence),
                'warning_count': len(warnings),
                'last_warning_text': last_warning,
            }
            windows.append(window)
        return windows

    def _land_windows(self, drone_id: str) -> list[dict[str, Any]]:
        commands = [row for row in self._operator_commands(drone_id) if row['payload'].get('op_com_name') == 'land']
        operator_commands = self._operator_commands(drone_id)
        windows = []
        for index, command in enumerate(commands, start=1):
            start = command['wall_time_epoch_sec']
            end = self._window_end_time(drone_id, start, operator_commands)
            standby = self._first_event_time(
                [row for row in self._rows_for_drone(self.ros_events, drone_id) if row.get('wall_time_epoch_sec') and start <= row['wall_time_epoch_sec'] <= end],
                lambda row: row.get('event_type') == 'mission_control_state' and row['payload'].get('mc_state_name') == 'Standby',
            )
            windows.append(
                {
                    'drone_id': drone_id,
                    'window_index': index,
                    'window_type': 'land_attempt',
                    'command_name': 'land',
                    'start_wall_time_sec': start,
                    'end_wall_time_sec': end,
                    'outcome': 'success' if standby is not None else 'incomplete',
                    'command_to_standby_sec': None if standby is None else standby - start,
                }
            )
        return windows

    def _rtl_windows(self, drone_id: str) -> list[dict[str, Any]]:
        commands = [row for row in self._operator_commands(drone_id) if row['payload'].get('op_com_name') == 'gohome']
        operator_commands = self._operator_commands(drone_id)
        windows = []
        for index, command in enumerate(commands, start=1):
            start = command['wall_time_epoch_sec']
            end = self._window_end_time(drone_id, start, operator_commands)
            rtl = self._first_event_time(
                [row for row in self._rows_for_drone(self.ros_events, drone_id) if row.get('wall_time_epoch_sec') and start <= row['wall_time_epoch_sec'] <= end],
                lambda row: (
                    row.get('event_type') == 'mission_control_state' and row['payload'].get('mc_state_name') == 'ReturnToLaunch'
                ) or (
                    row.get('event_type') == 'high_level_control_state' and row['payload'].get('hlc_state_name') == 'ReturnToLaunch'
                ),
            )
            windows.append(
                {
                    'drone_id': drone_id,
                    'window_index': index,
                    'window_type': 'rtl_attempt',
                    'command_name': 'gohome',
                    'start_wall_time_sec': start,
                    'end_wall_time_sec': end,
                    'outcome': 'success' if rtl is not None else 'incomplete',
                    'command_to_rtl_state_sec': None if rtl is None else rtl - start,
                }
            )
        return windows

    def _velocity_windows(self, drone_id: str, operator_commands: list[dict[str, Any]]) -> list[dict[str, Any]]:
        rows = [
            row for row in self._rows_for_drone(self.component_events, drone_id)
            if row.get('event_type') == 'velocity_command_published'
        ]
        windows = []
        for index, row in enumerate(rows, start=1):
            start = row.get('wall_time_epoch_sec')
            if start is None:
                continue
            end = self._window_end_time(drone_id, start, operator_commands)
            payload = row.get('payload', {})
            label = payload.get('label', 'velocity')
            windows.append(
                {
                    'drone_id': drone_id,
                    'window_index': index,
                    'window_type': 'velocity_window',
                    'command_name': label,
                    'start_wall_time_sec': start,
                    'end_wall_time_sec': end,
                    'outcome': 'captured',
                    'command_to_first_actuator_change_sec': self._first_servo_change_latency(drone_id, start).get('wall_sec'),
                    'command_to_first_motion_sec': self._first_motion_latency(drone_id, start).get('wall_sec'),
                }
            )
        return windows

    def _build_control_events(self) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        for row in self.ros_events:
            if row.get('event_type') not in {
                'input_operator_command',
                'input_velocity',
                'mission_control_state',
                'high_level_control_state',
                'hlc_telemetry',
                'platform_error',
            }:
                continue
            wall = row.get('wall_time_epoch_sec')
            if wall is None:
                continue
            rows.append(
                {
                    'wall_time_epoch_sec': wall,
                    'sim_time_sec': self.sim_time.at(wall),
                    'drone_id': row.get('drone_id', ''),
                    'source': 'ros',
                    'component': 'ros',
                    'event_type': row.get('event_type', ''),
                    'summary': row.get('event_type', ''),
                    'payload_json': json.dumps(row.get('payload', {}), separators=(',', ':')),
                }
            )
        for row in self.component_events:
            if row.get('component') not in CONTROL_COMPONENTS:
                continue
            wall = row.get('wall_time_epoch_sec')
            if wall is None:
                continue
            rows.append(
                {
                    'wall_time_epoch_sec': wall,
                    'sim_time_sec': self.sim_time.at(wall),
                    'drone_id': row.get('drone_id', ''),
                    'source': 'component',
                    'component': row.get('component', ''),
                    'event_type': row.get('event_type', ''),
                    'summary': f"{row.get('component','')}:{row.get('event_type','')}",
                    'payload_json': row.get('payload_json', '{}'),
                }
            )
        for drone in self.manifest.get('drones', []):
            drone_id = drone['id']
            for row in self._heartbeat_rows(drone_id):
                wall = row.get('host_time_sec')
                if wall is None:
                    continue
                payload = {
                    'armed': row.get('armed'),
                    'mode_string': row.get('mode_string', ''),
                    'system_status': row.get('system_status'),
                }
                rows.append(
                    {
                        'wall_time_epoch_sec': wall,
                        'sim_time_sec': self.sim_time.at(wall),
                        'drone_id': drone_id,
                        'source': 'mavlink',
                        'component': 'heartbeat',
                        'event_type': 'heartbeat',
                        'summary': row.get('mode_string', ''),
                        'payload_json': json.dumps(payload, separators=(',', ':')),
                    }
                )
            for row in self._statustext_rows(drone_id):
                wall = row.get('host_time_sec')
                if wall is None:
                    continue
                payload = {
                    'severity': row.get('severity'),
                    'text': row.get('text', ''),
                }
                rows.append(
                    {
                        'wall_time_epoch_sec': wall,
                        'sim_time_sec': self.sim_time.at(wall),
                        'drone_id': drone_id,
                        'source': 'mavlink',
                        'component': 'statustext',
                        'event_type': 'statustext',
                        'summary': row.get('text', ''),
                        'payload_json': json.dumps(payload, separators=(',', ':')),
                    }
                )
        rows.sort(key=lambda row: (row['wall_time_epoch_sec'] or 0.0, row['drone_id'], row['event_type']))
        write_csv_rows(
            self.control_events_path,
            ['wall_time_epoch_sec', 'sim_time_sec', 'drone_id', 'source', 'component', 'event_type', 'summary', 'payload_json'],
            rows,
        )
        return rows

    def _build_control_windows(self) -> tuple[list[dict[str, Any]], dict[str, Any]]:
        control_windows: list[dict[str, Any]] = []
        summary: dict[str, Any] = {
            'run_id': self.manifest.get('run_id'),
            'mode': self.manifest.get('mode'),
            'control_focus': 'takeoff',
            'idle_drift_thresholds': self._idle_drift_thresholds(),
            'drones': {},
        }
        for drone in self.manifest.get('drones', []):
            drone_id = drone['id']
            operator_commands = self._operator_commands(drone_id)
            takeoff_windows = self._takeoff_windows(drone_id)
            land_windows = self._land_windows(drone_id)
            rtl_windows = self._rtl_windows(drone_id)
            velocity_windows = self._velocity_windows(drone_id, operator_commands)
            control_windows.extend(takeoff_windows)
            control_windows.extend(land_windows)
            control_windows.extend(rtl_windows)
            control_windows.extend(velocity_windows)
            idle_drift = self._idle_drift_summary(drone_id, self._startup_metrics(drone_id)['hlc_ready_wall_time_sec'])
            summary['drones'][drone_id] = {
                'takeoff_windows': takeoff_windows,
                'land_windows': land_windows,
                'rtl_windows': rtl_windows,
                'velocity_windows': velocity_windows,
                'latest_takeoff': takeoff_windows[-1] if takeoff_windows else None,
                'latest_land': land_windows[-1] if land_windows else None,
                'latest_rtl': rtl_windows[-1] if rtl_windows else None,
                'latest_velocity': velocity_windows[-1] if velocity_windows else None,
                **idle_drift,
            }
        write_csv_rows(
            self.control_windows_path,
            [
                'drone_id',
                'window_index',
                'window_type',
                'command_name',
                'start_wall_time_sec',
                'end_wall_time_sec',
                'outcome',
                'command_to_mc_transition_sec',
                'command_to_hlc_takeoff_sec',
                'command_to_first_armed_sec',
                'command_to_first_takeoff_resend_sec',
                'command_to_first_actuator_change_sec',
                'command_to_first_motion_sec',
                'command_to_first_altitude_increase_sec',
                'command_to_altitude_0_2_sec',
                'command_to_altitude_1_0_sec',
                'target_altitude_m',
                'max_yaw_delta_before_0_2_deg',
                'max_yaw_rate_before_0_2_rad_s',
                'time_in_takeoff_sec',
                'time_in_gaining_altitude_sec',
                'reached_performing_mission',
                'reached_airborne',
                'hlc_state_sequence',
                'mc_state_sequence',
                'warning_count',
                'last_warning_text',
                'command_to_standby_sec',
                'command_to_rtl_state_sec',
            ],
            control_windows,
        )
        self.control_summary_path.write_text(json.dumps(summary, indent=2) + '\n')
        self.control_summary_md_path.write_text(self._render_control_markdown(summary))
        return control_windows, summary

    def _bottlenecks(self, summary: dict[str, Any], control_summary: dict[str, Any]) -> list[dict[str, Any]]:
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
        for drone_id, drone_summary in control_summary.get('drones', {}).items():
            latest_takeoff = drone_summary.get('latest_takeoff')
            if drone_summary.get('precommand_drift_gate_passed') is False:
                findings.append(
                    {
                        'priority': 5,
                        'title': f'Ground drift before command ({drone_id})',
                        'detail': (
                            f"h={drone_summary.get('precommand_horizontal_drift_m'):.3f}m "
                            f"v={drone_summary.get('precommand_vertical_drift_m'):.3f}m "
                            f"yaw={drone_summary.get('precommand_yaw_drift_deg'):.2f}deg"
                        ),
                    }
                )
            if drone_summary.get('uncommanded_post_foreign_command_drift_gate_passed') is False:
                findings.append(
                    {
                        'priority': 6,
                        'title': f'Uncommanded drone motion after foreign command ({drone_id})',
                        'detail': (
                            f"h={drone_summary.get('uncommanded_post_foreign_command_horizontal_drift_m'):.3f}m "
                            f"v={drone_summary.get('uncommanded_post_foreign_command_vertical_drift_m'):.3f}m "
                            f"yaw={drone_summary.get('uncommanded_post_foreign_command_yaw_drift_deg'):.2f}deg"
                        ),
                    }
                )
            if not latest_takeoff:
                continue
            outcome = latest_takeoff.get('outcome')
            if outcome != 'success_climb':
                findings.append({'priority': 7, 'title': f'Takeoff control anomaly ({drone_id})', 'detail': f'outcome={outcome}'})
        findings.sort(key=lambda row: (row['priority'], row['title']))
        return findings

    def analyze(self) -> dict[str, Any]:
        drone_ids = [drone['id'] for drone in self.manifest.get('drones', [])]
        self._build_control_events()
        _control_windows, control_summary = self._build_control_windows()
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
            'idle_drift_thresholds': self._idle_drift_thresholds(),
            'control': {
                'summary_file': str(self.control_summary_path.relative_to(self.run_dir)),
                'latest_takeoff_outcomes': {
                    drone_id: (control_summary['drones'][drone_id]['latest_takeoff'] or {}).get('outcome')
                    for drone_id in drone_ids
                },
            },
        }
        summary['bottlenecks'] = self._bottlenecks(summary, control_summary)
        self.summary_path.write_text(json.dumps(summary, indent=2) + '\n')
        self.summary_md_path.write_text(self._render_markdown(summary, control_summary))
        return summary

    def _render_markdown(self, summary: dict[str, Any], control_summary: dict[str, Any]) -> str:
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
            lines.append(
                f"- `{drone_id}` precommand drift h/v/yaw={drone_summary.get('precommand_horizontal_drift_m')} / {drone_summary.get('precommand_vertical_drift_m')} / {drone_summary.get('precommand_yaw_drift_deg')} samples={drone_summary.get('precommand_pose_sample_count')} gate={drone_summary.get('precommand_drift_gate_passed')}"
            )
            lines.append(
                f"- `{drone_id}` uncommanded-after-foreign drift h/v/yaw={drone_summary.get('uncommanded_post_foreign_command_horizontal_drift_m')} / {drone_summary.get('uncommanded_post_foreign_command_vertical_drift_m')} / {drone_summary.get('uncommanded_post_foreign_command_yaw_drift_deg')} gate={drone_summary.get('uncommanded_post_foreign_command_drift_gate_passed')}"
            )
            lines.append(f"- `{drone_id}` command->state={drone_summary['command_to_state_transition_latency'].get('wall_sec')} s")
            lines.append(f"- `{drone_id}` command->actuator={drone_summary['command_to_first_actuator_change_latency'].get('wall_sec')} s")
            lines.append(f"- `{drone_id}` command->motion={drone_summary['command_to_first_motion_latency'].get('wall_sec')} s")
            lines.append(f"- `{drone_id}` takeoff/landing={drone_summary['takeoff_duration_to_altitude_threshold'].get('wall_sec')} / {drone_summary['landing_duration_to_standby'].get('wall_sec')} s")
            lines.append(f"- `{drone_id}` pose backlog p95={drone_summary['pose']['pose_backlog_sec'].get('p95')}")
        lines.append('')
        lines.append('## Control Snapshot')
        for drone_id, drone_summary in control_summary.get('drones', {}).items():
            latest_takeoff = drone_summary.get('latest_takeoff')
            if not latest_takeoff:
                lines.append(f"- `{drone_id}` no takeoff window captured")
                continue
            lines.append(
                f"- `{drone_id}` takeoff outcome={latest_takeoff.get('outcome')} arm={latest_takeoff.get('command_to_first_armed_sec')}s alt0.2={latest_takeoff.get('command_to_altitude_0_2_sec')}s alt1.0={latest_takeoff.get('command_to_altitude_1_0_sec')}s yaw_delta={latest_takeoff.get('max_yaw_delta_before_0_2_deg')}deg"
            )
            if latest_takeoff.get('last_warning_text'):
                lines.append(f"- `{drone_id}` last warning: {latest_takeoff.get('last_warning_text')}")
        lines.append('')
        lines.append('## GPU')
        lines.append(
            f"- GPU util peak={summary.get('gpu', {}).get('utilization_gpu_percent', {}).get('max')} memory peak MiB={summary.get('gpu', {}).get('memory_used_mib', {}).get('max')}"
        )
        lines.append('')
        return '\n'.join(lines) + '\n'

    def _render_control_markdown(self, summary: dict[str, Any]) -> str:
        lines = ['# Control Trace Summary', '', f"Run: `{summary.get('run_id', '')}`", f"Mode: `{summary.get('mode', '')}`", '']
        for drone_id, drone_summary in summary.get('drones', {}).items():
            lines.append(f'## {drone_id}')
            lines.append(
                f"- Precommand drift h/v/yaw: `{drone_summary.get('precommand_horizontal_drift_m')}` / `{drone_summary.get('precommand_vertical_drift_m')}` / `{drone_summary.get('precommand_yaw_drift_deg')}`"
            )
            lines.append(f"- Precommand pose samples / gate: `{drone_summary.get('precommand_pose_sample_count')}` / `{drone_summary.get('precommand_drift_gate_passed')}`")
            lines.append(
                f"- Uncommanded-after-foreign drift h/v/yaw: `{drone_summary.get('uncommanded_post_foreign_command_horizontal_drift_m')}` / `{drone_summary.get('uncommanded_post_foreign_command_vertical_drift_m')}` / `{drone_summary.get('uncommanded_post_foreign_command_yaw_drift_deg')}`"
            )
            if drone_summary.get('precommand_idle_servo_distinct_values'):
                lines.append(f"- Idle servo distinct values: `{json.dumps(drone_summary.get('precommand_idle_servo_distinct_values'), sort_keys=True)}`")
            latest_takeoff = drone_summary.get('latest_takeoff')
            if latest_takeoff:
                lines.append(f"- Takeoff outcome: `{latest_takeoff.get('outcome')}`")
                lines.append(f"- Command -> MC/HLC takeoff: `{latest_takeoff.get('command_to_mc_transition_sec')}` / `{latest_takeoff.get('command_to_hlc_takeoff_sec')}` s")
                lines.append(f"- Command -> armed / actuator / altitude0.2 / altitude1.0: `{latest_takeoff.get('command_to_first_armed_sec')}` / `{latest_takeoff.get('command_to_first_actuator_change_sec')}` / `{latest_takeoff.get('command_to_altitude_0_2_sec')}` / `{latest_takeoff.get('command_to_altitude_1_0_sec')}` s")
                lines.append(f"- Max yaw delta before liftoff: `{latest_takeoff.get('max_yaw_delta_before_0_2_deg')}` deg")
                lines.append(f"- Max yaw rate before liftoff: `{latest_takeoff.get('max_yaw_rate_before_0_2_rad_s')}` rad/s")
                lines.append(f"- HLC sequence: `{latest_takeoff.get('hlc_state_sequence')}`")
                lines.append(f"- MC sequence: `{latest_takeoff.get('mc_state_sequence')}`")
                if latest_takeoff.get('last_warning_text'):
                    lines.append(f"- Last warning: {latest_takeoff.get('last_warning_text')}")
            else:
                lines.append('- No takeoff window captured.')
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
