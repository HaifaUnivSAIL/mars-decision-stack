from __future__ import annotations

import csv
import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ANALYZE = ROOT / 'scripts' / 'analyze_stack_profile.py'
COMPARE = ROOT / 'scripts' / 'compare_stack_profiles.py'


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def make_run(tmp_path: Path, name: str, rtf: float, motion_latency: float) -> Path:
    run_dir = tmp_path / name
    profile_dir = run_dir / 'diagnostics' / 'profile'
    logs_dir = run_dir / 'logs'
    profile_dir.mkdir(parents=True)
    logs_dir.mkdir(parents=True)

    (profile_dir / 'profile.manifest.json').write_text(
        json.dumps(
            {
                'run_id': name,
                'mode': 'single',
                'fleet': False,
                'visual_gui': False,
                'drones': [{'id': 'drone_1'}],
            }
        )
        + '\n'
    )

    ros_events = [
        {
            'wall_time_epoch_sec': '1.0',
            'monotonic_sec': '1.0',
            'ros_time_sec': '1.0',
            'drone_id': 'drone_1',
            'topic_name': 'alate_output_mission_control_state',
            'event_type': 'mission_control_state',
            'payload_json': json.dumps({'mc_state_name': 'Standby'}),
        },
        {
            'wall_time_epoch_sec': '1.1',
            'monotonic_sec': '1.1',
            'ros_time_sec': '1.1',
            'drone_id': 'drone_1',
            'topic_name': 'alate_output_high_level_control_state',
            'event_type': 'high_level_control_state',
            'payload_json': json.dumps({'hlc_state_name': 'Ready'}),
        },
        {
            'wall_time_epoch_sec': '2.0',
            'monotonic_sec': '2.0',
            'ros_time_sec': '2.0',
            'drone_id': 'drone_1',
            'topic_name': 'alate_input_operator_command',
            'event_type': 'input_operator_command',
            'payload_json': json.dumps({'op_com_name': 'takeoff'}),
        },
        {
            'wall_time_epoch_sec': '2.1',
            'monotonic_sec': '2.1',
            'ros_time_sec': '2.1',
            'drone_id': 'drone_1',
            'topic_name': 'alate_input_velocity',
            'event_type': 'input_velocity',
            'payload_json': json.dumps({'linear_x': 0.4, 'linear_y': 0.0, 'linear_z': 0.0}),
        },
        {
            'wall_time_epoch_sec': '3.5',
            'monotonic_sec': '3.5',
            'ros_time_sec': '3.5',
            'drone_id': 'drone_1',
            'topic_name': 'alate_output_high_level_control_telemetry',
            'event_type': 'hlc_telemetry',
            'payload_json': json.dumps({'altitude': 1.2, 'armed': True, 'mode': 'GUIDED', 'state': 'ACTIVE'}),
        },
        {
            'wall_time_epoch_sec': '5.0',
            'monotonic_sec': '5.0',
            'ros_time_sec': '5.0',
            'drone_id': 'drone_1',
            'topic_name': 'alate_input_operator_command',
            'event_type': 'input_operator_command',
            'payload_json': json.dumps({'op_com_name': 'land'}),
        },
        {
            'wall_time_epoch_sec': '6.5',
            'monotonic_sec': '6.5',
            'ros_time_sec': '6.5',
            'drone_id': 'drone_1',
            'topic_name': 'alate_output_mission_control_state',
            'event_type': 'mission_control_state',
            'payload_json': json.dumps({'mc_state_name': 'Standby'}),
        },
    ]
    write_csv(profile_dir / 'ros_events.csv', list(ros_events[0].keys()), ros_events)

    write_csv(
        profile_dir / 'world_stats.csv',
        ['wall_time_epoch_sec', 'monotonic_sec', 'sim_time_sec', 'real_time_sec', 'pause_time_sec', 'real_time_factor', 'iterations', 'model_count', 'paused', 'step_size_sec', 'stepping'],
        [
            {'wall_time_epoch_sec': '1.0', 'monotonic_sec': '1.0', 'sim_time_sec': '0.0', 'real_time_sec': '0.0', 'pause_time_sec': '0.0', 'real_time_factor': str(rtf), 'iterations': '1', 'model_count': '1', 'paused': '0', 'step_size_sec': '0.01', 'stepping': '0'},
            {'wall_time_epoch_sec': '6.5', 'monotonic_sec': '6.5', 'sim_time_sec': '5.5', 'real_time_sec': '5.5', 'pause_time_sec': '0.0', 'real_time_factor': str(rtf), 'iterations': '2', 'model_count': '1', 'paused': '0', 'step_size_sec': '0.01', 'stepping': '0'},
        ],
    )

    write_csv(
        profile_dir / 'pose_trace.csv',
        ['wall_time_epoch_sec', 'monotonic_sec', 'sim_time_sec', 'drone_id', 'entity_name', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'pose_backlog_sec'],
        [
            {'wall_time_epoch_sec': '2.1', 'monotonic_sec': '2.1', 'sim_time_sec': '1.1', 'drone_id': 'drone_1', 'entity_name': 'iris', 'x': '0.0', 'y': '0.0', 'z': '0.2', 'roll': '0', 'pitch': '0', 'yaw': '0', 'pose_backlog_sec': '0.1'},
            {'wall_time_epoch_sec': str(2.1 + motion_latency), 'monotonic_sec': str(2.1 + motion_latency), 'sim_time_sec': str(1.1 + motion_latency), 'drone_id': 'drone_1', 'entity_name': 'iris', 'x': '0.2', 'y': '0.0', 'z': '0.3', 'roll': '0', 'pitch': '0', 'yaw': '0', 'pose_backlog_sec': '0.2'},
        ],
    )

    write_csv(
        profile_dir / 'camera_frames.csv',
        ['wall_time_epoch_sec', 'monotonic_sec', 'drone_id', 'stream_name', 'width', 'height', 'source_sim_time_sec', 'stream_backlog_sec', 'interframe_gap_sec'],
        [
            {'wall_time_epoch_sec': '2.0', 'monotonic_sec': '2.0', 'drone_id': 'drone_1', 'stream_name': 'chase', 'width': '640', 'height': '480', 'source_sim_time_sec': '1.0', 'stream_backlog_sec': '0.2', 'interframe_gap_sec': '0.2'},
            {'wall_time_epoch_sec': '2.2', 'monotonic_sec': '2.2', 'drone_id': 'drone_1', 'stream_name': 'chase', 'width': '640', 'height': '480', 'source_sim_time_sec': '1.2', 'stream_backlog_sec': '0.3', 'interframe_gap_sec': '0.2'},
        ],
    )

    write_csv(
        profile_dir / 'container_stats.csv',
        ['wall_time_epoch_sec', 'container_name', 'cpu_percent', 'mem_usage_bytes', 'mem_limit_bytes', 'mem_percent', 'net_in_bytes', 'net_out_bytes', 'block_in_bytes', 'block_out_bytes', 'pids'],
        [
            {'wall_time_epoch_sec': '2.0', 'container_name': 'mars-decision-stack-visual-sim', 'cpu_percent': '80', 'mem_usage_bytes': '1000000', 'mem_limit_bytes': '8000000', 'mem_percent': '12.5', 'net_in_bytes': '0', 'net_out_bytes': '0', 'block_in_bytes': '0', 'block_out_bytes': '0', 'pids': '10'},
            {'wall_time_epoch_sec': '3.0', 'container_name': 'mars-decision-stack-visual-sim', 'cpu_percent': '100', 'mem_usage_bytes': '2000000', 'mem_limit_bytes': '8000000', 'mem_percent': '25', 'net_in_bytes': '0', 'net_out_bytes': '0', 'block_in_bytes': '0', 'block_out_bytes': '0', 'pids': '11'},
        ],
    )

    write_csv(
        profile_dir / 'gpu_stats.csv',
        ['wall_time_epoch_sec', 'gpu_index', 'gpu_name', 'utilization_gpu_percent', 'utilization_memory_percent', 'memory_used_mib', 'memory_total_mib'],
        [
            {'wall_time_epoch_sec': '2.0', 'gpu_index': '0', 'gpu_name': 'GPU', 'utilization_gpu_percent': '10', 'utilization_memory_percent': '20', 'memory_used_mib': '300', 'memory_total_mib': '1000'},
            {'wall_time_epoch_sec': '3.0', 'gpu_index': '0', 'gpu_name': 'GPU', 'utilization_gpu_percent': '20', 'utilization_memory_percent': '30', 'memory_used_mib': '500', 'memory_total_mib': '1000'},
        ],
    )

    write_csv(
        profile_dir / 'mavlink' / 'drone_1' / 'servo_output_raw.csv',
        ['host_time_sec', 'time_usec', 'servo1_raw', 'servo2_raw', 'servo3_raw', 'servo4_raw'],
        [
            {'host_time_sec': '2.0', 'time_usec': '1', 'servo1_raw': '1000', 'servo2_raw': '1000', 'servo3_raw': '1000', 'servo4_raw': '1000'},
            {'host_time_sec': '2.3', 'time_usec': '2', 'servo1_raw': '1010', 'servo2_raw': '1000', 'servo3_raw': '1000', 'servo4_raw': '1000'},
        ],
    )

    (logs_dir / 'decision-dev.log').write_text('INFO PROFILE:{"component":"manual_runtime_test","event_type":"operator_command_published","wall_time_epoch_sec":2.0}\n')
    return run_dir


def test_analyze_stack_profile(tmp_path: Path) -> None:
    run_dir = make_run(tmp_path, 'baseline-run', 0.95, 0.5)
    subprocess.run([sys.executable, str(ANALYZE), '--run-dir', str(run_dir)], check=True)
    summary = json.loads((run_dir / 'diagnostics' / 'profile' / 'summary.json').read_text())
    assert summary['run_id'] == 'baseline-run'
    assert 'drone_1' in summary['drones']
    assert summary['drones']['drone_1']['command_to_first_motion_latency']['wall_sec'] is not None
    component_events = (run_dir / 'diagnostics' / 'profile' / 'component_events.csv').read_text()
    assert 'manual_runtime_test' in component_events


def test_compare_stack_profiles(tmp_path: Path) -> None:
    baseline = make_run(tmp_path, 'baseline-run', 1.0, 0.5)
    candidate = make_run(tmp_path, 'candidate-run', 0.6, 1.5)
    subprocess.run([sys.executable, str(ANALYZE), '--run-dir', str(baseline)], check=True)
    subprocess.run([sys.executable, str(ANALYZE), '--run-dir', str(candidate)], check=True)
    subprocess.run([sys.executable, str(COMPARE), '--baseline', str(baseline), '--candidate', str(candidate)], check=True)
    comparison = json.loads((candidate / 'diagnostics' / 'profile' / 'comparison.json').read_text())
    assert comparison['world_deltas']['rtf_median_delta'] is not None
    assert 'drone_1' in comparison['drone_deltas']
