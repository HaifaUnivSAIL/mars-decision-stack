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


def append_profile_log(path: Path, payloads: list[dict[str, object]], include_malformed: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    if include_malformed:
        lines.append('INFO PROFILE:{"component":"bad"}\n')
    for payload in payloads:
        lines.append(f"INFO PROFILE:{json.dumps(payload, separators=(',', ':'))}\n")
    path.write_text(''.join(lines))


def base_world_rows(rtf: float) -> list[dict[str, object]]:
    return [
        {
            'wall_time_epoch_sec': '1.0',
            'monotonic_sec': '1.0',
            'sim_time_sec': '0.0',
            'real_time_sec': '0.0',
            'pause_time_sec': '0.0',
            'real_time_factor': str(rtf),
            'iterations': '1',
            'model_count': '1',
            'paused': '0',
            'step_size_sec': '0.01',
            'stepping': '0',
        },
        {
            'wall_time_epoch_sec': '7.0',
            'monotonic_sec': '7.0',
            'sim_time_sec': f'{6.0 * rtf:.3f}',
            'real_time_sec': '6.0',
            'pause_time_sec': '0.0',
            'real_time_factor': str(rtf),
            'iterations': '2',
            'model_count': '1',
            'paused': '0',
            'step_size_sec': '0.01',
            'stepping': '0',
        },
    ]


def base_camera_rows(drone_ids: list[str], backlog: float) -> list[dict[str, object]]:
    rows = []
    for drone_id in drone_ids:
        rows.extend(
            [
                {
                    'wall_time_epoch_sec': '2.0',
                    'monotonic_sec': '2.0',
                    'drone_id': drone_id,
                    'stream_name': 'chase',
                    'width': '640',
                    'height': '480',
                    'source_sim_time_sec': '1.0',
                    'stream_backlog_sec': str(backlog),
                    'interframe_gap_sec': '0.2',
                },
                {
                    'wall_time_epoch_sec': '2.2',
                    'monotonic_sec': '2.2',
                    'drone_id': drone_id,
                    'stream_name': 'chase',
                    'width': '640',
                    'height': '480',
                    'source_sim_time_sec': '1.2',
                    'stream_backlog_sec': str(backlog + 0.1),
                    'interframe_gap_sec': '0.25',
                },
            ]
        )
    return rows


def base_container_rows(cpu_peak: float) -> list[dict[str, object]]:
    return [
        {
            'wall_time_epoch_sec': '2.0',
            'container_name': 'mars-decision-stack-visual-sim',
            'cpu_percent': str(cpu_peak - 20.0),
            'mem_usage_bytes': '1000000',
            'mem_limit_bytes': '8000000',
            'mem_percent': '12.5',
            'net_in_bytes': '0',
            'net_out_bytes': '0',
            'block_in_bytes': '0',
            'block_out_bytes': '0',
            'pids': '10',
        },
        {
            'wall_time_epoch_sec': '3.0',
            'container_name': 'mars-decision-stack-visual-sim',
            'cpu_percent': str(cpu_peak),
            'mem_usage_bytes': '2000000',
            'mem_limit_bytes': '8000000',
            'mem_percent': '25',
            'net_in_bytes': '0',
            'net_out_bytes': '0',
            'block_in_bytes': '0',
            'block_out_bytes': '0',
            'pids': '11',
        },
    ]


def base_gpu_rows(util: float) -> list[dict[str, object]]:
    return [
        {
            'wall_time_epoch_sec': '2.0',
            'gpu_index': '0',
            'gpu_name': 'GPU',
            'utilization_gpu_percent': str(util - 5.0),
            'utilization_memory_percent': '20',
            'memory_used_mib': '300',
            'memory_total_mib': '1000',
        },
        {
            'wall_time_epoch_sec': '3.0',
            'gpu_index': '0',
            'gpu_name': 'GPU',
            'utilization_gpu_percent': str(util),
            'utilization_memory_percent': '30',
            'memory_used_mib': '500',
            'memory_total_mib': '1000',
        },
    ]


def success_drone_fixture(drone_id: str) -> dict[str, list[dict[str, object]]]:
    ros_events = [
        {'wall_time_epoch_sec': '1.0', 'monotonic_sec': '1.0', 'ros_time_sec': '1.0', 'drone_id': drone_id, 'topic_name': 'alate_output_mission_control_state', 'event_type': 'mission_control_state', 'payload_json': json.dumps({'mc_state_name': 'Standby'})},
        {'wall_time_epoch_sec': '1.1', 'monotonic_sec': '1.1', 'ros_time_sec': '1.1', 'drone_id': drone_id, 'topic_name': 'alate_output_high_level_control_state', 'event_type': 'high_level_control_state', 'payload_json': json.dumps({'hlc_state_name': 'Ready'})},
        {'wall_time_epoch_sec': '2.0', 'monotonic_sec': '2.0', 'ros_time_sec': '2.0', 'drone_id': drone_id, 'topic_name': 'alate_input_operator_command', 'event_type': 'input_operator_command', 'payload_json': json.dumps({'op_com_name': 'takeoff'})},
        {'wall_time_epoch_sec': '2.05', 'monotonic_sec': '2.05', 'ros_time_sec': '2.05', 'drone_id': drone_id, 'topic_name': 'alate_output_mission_control_state', 'event_type': 'mission_control_state', 'payload_json': json.dumps({'mc_state_name': 'TakingOff'})},
        {'wall_time_epoch_sec': '2.06', 'monotonic_sec': '2.06', 'ros_time_sec': '2.06', 'drone_id': drone_id, 'topic_name': 'alate_output_high_level_control_state', 'event_type': 'high_level_control_state', 'payload_json': json.dumps({'hlc_state_name': 'Takeoff'})},
        {'wall_time_epoch_sec': '2.2', 'monotonic_sec': '2.2', 'ros_time_sec': '2.2', 'drone_id': drone_id, 'topic_name': 'alate_output_high_level_control_state', 'event_type': 'high_level_control_state', 'payload_json': json.dumps({'hlc_state_name': 'GainingAltitude'})},
        {'wall_time_epoch_sec': '2.3', 'monotonic_sec': '2.3', 'ros_time_sec': '2.3', 'drone_id': drone_id, 'topic_name': 'alate_output_high_level_control_telemetry', 'event_type': 'hlc_telemetry', 'payload_json': json.dumps({'altitude': 0.05, 'yaw': 0.01, 'armed': True, 'mode': 'GUIDED', 'state': 'ACTIVE'})},
        {'wall_time_epoch_sec': '2.8', 'monotonic_sec': '2.8', 'ros_time_sec': '2.8', 'drone_id': drone_id, 'topic_name': 'alate_output_high_level_control_telemetry', 'event_type': 'hlc_telemetry', 'payload_json': json.dumps({'altitude': 0.25, 'yaw': 0.03, 'armed': True, 'mode': 'GUIDED', 'state': 'ACTIVE'})},
        {'wall_time_epoch_sec': '3.3', 'monotonic_sec': '3.3', 'ros_time_sec': '3.3', 'drone_id': drone_id, 'topic_name': 'alate_output_high_level_control_state', 'event_type': 'high_level_control_state', 'payload_json': json.dumps({'hlc_state_name': 'Airborne'})},
        {'wall_time_epoch_sec': '3.35', 'monotonic_sec': '3.35', 'ros_time_sec': '3.35', 'drone_id': drone_id, 'topic_name': 'alate_output_mission_control_state', 'event_type': 'mission_control_state', 'payload_json': json.dumps({'mc_state_name': 'PerformingMission'})},
        {'wall_time_epoch_sec': '3.5', 'monotonic_sec': '3.5', 'ros_time_sec': '3.5', 'drone_id': drone_id, 'topic_name': 'alate_output_high_level_control_telemetry', 'event_type': 'hlc_telemetry', 'payload_json': json.dumps({'altitude': 1.2, 'yaw': 0.05, 'armed': True, 'mode': 'GUIDED', 'state': 'ACTIVE'})},
        {'wall_time_epoch_sec': '4.0', 'monotonic_sec': '4.0', 'ros_time_sec': '4.0', 'drone_id': drone_id, 'topic_name': 'alate_input_velocity', 'event_type': 'input_velocity', 'payload_json': json.dumps({'linear_x': 0.4, 'linear_y': 0.0, 'linear_z': 0.0})},
        {'wall_time_epoch_sec': '5.0', 'monotonic_sec': '5.0', 'ros_time_sec': '5.0', 'drone_id': drone_id, 'topic_name': 'alate_input_operator_command', 'event_type': 'input_operator_command', 'payload_json': json.dumps({'op_com_name': 'land'})},
        {'wall_time_epoch_sec': '6.5', 'monotonic_sec': '6.5', 'ros_time_sec': '6.5', 'drone_id': drone_id, 'topic_name': 'alate_output_mission_control_state', 'event_type': 'mission_control_state', 'payload_json': json.dumps({'mc_state_name': 'Standby'})},
    ]
    pose_rows = [
        {'wall_time_epoch_sec': '2.0', 'monotonic_sec': '2.0', 'sim_time_sec': '1.0', 'drone_id': drone_id, 'entity_name': 'iris', 'x': '0.0', 'y': '0.0', 'z': '0.0', 'roll': '0', 'pitch': '0', 'yaw': '0.0', 'pose_backlog_sec': '0.1'},
        {'wall_time_epoch_sec': '2.8', 'monotonic_sec': '2.8', 'sim_time_sec': '1.8', 'drone_id': drone_id, 'entity_name': 'iris', 'x': '0.0', 'y': '0.0', 'z': '0.25', 'roll': '0', 'pitch': '0', 'yaw': '0.03', 'pose_backlog_sec': '0.1'},
        {'wall_time_epoch_sec': '3.5', 'monotonic_sec': '3.5', 'sim_time_sec': '2.5', 'drone_id': drone_id, 'entity_name': 'iris', 'x': '0.0', 'y': '0.0', 'z': '1.2', 'roll': '0', 'pitch': '0', 'yaw': '0.05', 'pose_backlog_sec': '0.15'},
        {'wall_time_epoch_sec': '4.5', 'monotonic_sec': '4.5', 'sim_time_sec': '3.5', 'drone_id': drone_id, 'entity_name': 'iris', 'x': '0.25', 'y': '0.0', 'z': '1.2', 'roll': '0', 'pitch': '0', 'yaw': '0.05', 'pose_backlog_sec': '0.2'},
    ]
    heartbeats = [
        {'host_time_sec': '2.15', 'base_mode': '128', 'custom_mode': '4', 'system_status': '4', 'mav_type': '2', 'autopilot': '3', 'mode_string': 'GUIDED', 'armed': 'true'},
        {'host_time_sec': '3.5', 'base_mode': '128', 'custom_mode': '4', 'system_status': '4', 'mav_type': '2', 'autopilot': '3', 'mode_string': 'GUIDED', 'armed': 'true'},
    ]
    servo = [
        {'host_time_sec': '2.0', 'time_usec': '1', 'servo1_raw': '1000', 'servo2_raw': '1000', 'servo3_raw': '1000', 'servo4_raw': '1000'},
        {'host_time_sec': '2.3', 'time_usec': '2', 'servo1_raw': '1015', 'servo2_raw': '1000', 'servo3_raw': '1000', 'servo4_raw': '1000'},
        {'host_time_sec': '4.2', 'time_usec': '3', 'servo1_raw': '1025', 'servo2_raw': '1000', 'servo3_raw': '1000', 'servo4_raw': '1000'},
    ]
    attitude = [
        {'host_time_sec': '2.0', 'time_boot_ms': '100', 'roll': '0', 'pitch': '0', 'yaw': '0.0', 'rollspeed': '0', 'pitchspeed': '0', 'yawspeed': '0.0'},
        {'host_time_sec': '2.4', 'time_boot_ms': '140', 'roll': '0', 'pitch': '0', 'yaw': '0.03', 'rollspeed': '0', 'pitchspeed': '0', 'yawspeed': '0.1'},
        {'host_time_sec': '3.0', 'time_boot_ms': '200', 'roll': '0', 'pitch': '0', 'yaw': '0.04', 'rollspeed': '0', 'pitchspeed': '0', 'yawspeed': '0.12'},
    ]
    local_position = [
        {'host_time_sec': '2.0', 'time_boot_ms': '100', 'x': '0.0', 'y': '0.0', 'z': '0.0', 'vx': '0.0', 'vy': '0.0', 'vz': '0.0'},
        {'host_time_sec': '2.8', 'time_boot_ms': '180', 'x': '0.0', 'y': '0.0', 'z': '-0.25', 'vx': '0.0', 'vy': '0.0', 'vz': '-0.3'},
        {'host_time_sec': '3.5', 'time_boot_ms': '250', 'x': '0.0', 'y': '0.0', 'z': '-1.2', 'vx': '0.0', 'vy': '0.0', 'vz': '-0.6'},
    ]
    statustext = []
    component_logs = {
        'autopilot.log': [
            {'component': 'autopilot_dronekit', 'event_type': 'takeoff_sent', 'wall_time_epoch_sec': 2.08, 'drone_id': drone_id, 'altitude': 5.0},
        ],
        'manual_runtime_test.log': [
            {'component': 'manual_runtime_test', 'event_type': 'operator_command_published', 'wall_time_epoch_sec': 2.0, 'drone_id': drone_id, 'command': 'takeoff'},
            {'component': 'manual_runtime_test', 'event_type': 'velocity_command_published', 'wall_time_epoch_sec': 4.0, 'drone_id': drone_id, 'label': 'forward', 'linear_x': 0.4},
        ],
    }
    return {
        'ros_events': ros_events,
        'pose_rows': pose_rows,
        'heartbeat_rows': heartbeats,
        'servo_rows': servo,
        'attitude_rows': attitude,
        'local_position_rows': local_position,
        'statustext_rows': statustext,
        'component_logs': component_logs,
    }


def yaw_spin_drone_fixture(drone_id: str) -> dict[str, list[dict[str, object]]]:
    ros_events = [
        {'wall_time_epoch_sec': '1.0', 'monotonic_sec': '1.0', 'ros_time_sec': '1.0', 'drone_id': drone_id, 'topic_name': 'alate_output_mission_control_state', 'event_type': 'mission_control_state', 'payload_json': json.dumps({'mc_state_name': 'Standby'})},
        {'wall_time_epoch_sec': '1.1', 'monotonic_sec': '1.1', 'ros_time_sec': '1.1', 'drone_id': drone_id, 'topic_name': 'alate_output_high_level_control_state', 'event_type': 'high_level_control_state', 'payload_json': json.dumps({'hlc_state_name': 'Ready'})},
        {'wall_time_epoch_sec': '2.0', 'monotonic_sec': '2.0', 'ros_time_sec': '2.0', 'drone_id': drone_id, 'topic_name': 'alate_input_operator_command', 'event_type': 'input_operator_command', 'payload_json': json.dumps({'op_com_name': 'takeoff'})},
        {'wall_time_epoch_sec': '2.05', 'monotonic_sec': '2.05', 'ros_time_sec': '2.05', 'drone_id': drone_id, 'topic_name': 'alate_output_mission_control_state', 'event_type': 'mission_control_state', 'payload_json': json.dumps({'mc_state_name': 'TakingOff'})},
        {'wall_time_epoch_sec': '2.06', 'monotonic_sec': '2.06', 'ros_time_sec': '2.06', 'drone_id': drone_id, 'topic_name': 'alate_output_high_level_control_state', 'event_type': 'high_level_control_state', 'payload_json': json.dumps({'hlc_state_name': 'Takeoff'})},
        {'wall_time_epoch_sec': '2.15', 'monotonic_sec': '2.15', 'ros_time_sec': '2.15', 'drone_id': drone_id, 'topic_name': 'alate_output_high_level_control_state', 'event_type': 'high_level_control_state', 'payload_json': json.dumps({'hlc_state_name': 'GainingAltitude'})},
        {'wall_time_epoch_sec': '2.3', 'monotonic_sec': '2.3', 'ros_time_sec': '2.3', 'drone_id': drone_id, 'topic_name': 'alate_output_high_level_control_telemetry', 'event_type': 'hlc_telemetry', 'payload_json': json.dumps({'altitude': 0.0, 'yaw': 0.5, 'armed': True, 'mode': 'GUIDED', 'state': 'ACTIVE'})},
        {'wall_time_epoch_sec': '2.8', 'monotonic_sec': '2.8', 'ros_time_sec': '2.8', 'drone_id': drone_id, 'topic_name': 'alate_output_high_level_control_telemetry', 'event_type': 'hlc_telemetry', 'payload_json': json.dumps({'altitude': 0.01, 'yaw': 1.0, 'armed': True, 'mode': 'GUIDED', 'state': 'ACTIVE'})},
        {'wall_time_epoch_sec': '3.3', 'monotonic_sec': '3.3', 'ros_time_sec': '3.3', 'drone_id': drone_id, 'topic_name': 'alate_output_high_level_control_telemetry', 'event_type': 'hlc_telemetry', 'payload_json': json.dumps({'altitude': 0.02, 'yaw': 1.5, 'armed': True, 'mode': 'GUIDED', 'state': 'ACTIVE'})},
    ]
    pose_rows = [
        {'wall_time_epoch_sec': '2.0', 'monotonic_sec': '2.0', 'sim_time_sec': '0.6', 'drone_id': drone_id, 'entity_name': 'iris', 'x': '0.0', 'y': '0.0', 'z': '0.0', 'roll': '0', 'pitch': '0', 'yaw': '0.0', 'pose_backlog_sec': '0.6'},
        {'wall_time_epoch_sec': '2.8', 'monotonic_sec': '2.8', 'sim_time_sec': '1.0', 'drone_id': drone_id, 'entity_name': 'iris', 'x': '0.01', 'y': '0.0', 'z': '0.01', 'roll': '0', 'pitch': '0', 'yaw': '1.0', 'pose_backlog_sec': '0.8'},
        {'wall_time_epoch_sec': '3.4', 'monotonic_sec': '3.4', 'sim_time_sec': '1.3', 'drone_id': drone_id, 'entity_name': 'iris', 'x': '0.02', 'y': '0.0', 'z': '0.01', 'roll': '0', 'pitch': '0', 'yaw': '1.6', 'pose_backlog_sec': '0.9'},
    ]
    heartbeats = [
        {'host_time_sec': '2.15', 'base_mode': '128', 'custom_mode': '4', 'system_status': '4', 'mav_type': '2', 'autopilot': '3', 'mode_string': 'GUIDED', 'armed': 'true'},
        {'host_time_sec': '3.2', 'base_mode': '128', 'custom_mode': '4', 'system_status': '4', 'mav_type': '2', 'autopilot': '3', 'mode_string': 'GUIDED', 'armed': 'true'},
    ]
    servo = [
        {'host_time_sec': '2.0', 'time_usec': '1', 'servo1_raw': '1000', 'servo2_raw': '1000', 'servo3_raw': '1000', 'servo4_raw': '1000'},
        {'host_time_sec': '2.3', 'time_usec': '2', 'servo1_raw': '1018', 'servo2_raw': '1004', 'servo3_raw': '1000', 'servo4_raw': '998'},
    ]
    attitude = [
        {'host_time_sec': '2.0', 'time_boot_ms': '100', 'roll': '0', 'pitch': '0', 'yaw': '0.0', 'rollspeed': '0', 'pitchspeed': '0', 'yawspeed': '0.0'},
        {'host_time_sec': '2.4', 'time_boot_ms': '140', 'roll': '0', 'pitch': '0', 'yaw': '0.9', 'rollspeed': '0', 'pitchspeed': '0', 'yawspeed': '1.1'},
        {'host_time_sec': '3.0', 'time_boot_ms': '200', 'roll': '0', 'pitch': '0', 'yaw': '1.5', 'rollspeed': '0', 'pitchspeed': '0', 'yawspeed': '1.2'},
    ]
    local_position = [
        {'host_time_sec': '2.0', 'time_boot_ms': '100', 'x': '0.0', 'y': '0.0', 'z': '0.0', 'vx': '0.0', 'vy': '0.0', 'vz': '0.0'},
        {'host_time_sec': '2.8', 'time_boot_ms': '180', 'x': '0.01', 'y': '0.0', 'z': '-0.01', 'vx': '0.0', 'vy': '0.0', 'vz': '-0.01'},
    ]
    statustext = []
    component_logs = {
        'autopilot.log': [
            {'component': 'autopilot_dronekit', 'event_type': 'takeoff_sent', 'wall_time_epoch_sec': 2.08, 'drone_id': drone_id, 'altitude': 5.0},
            {'component': 'autopilot_dronekit', 'event_type': 'takeoff_sent', 'wall_time_epoch_sec': 2.55, 'drone_id': drone_id, 'altitude': 5.0},
        ],
        'manual_runtime_test.log': [
            {'component': 'manual_runtime_test', 'event_type': 'operator_command_published', 'wall_time_epoch_sec': 2.0, 'drone_id': drone_id, 'command': 'takeoff'},
        ],
    }
    return {
        'ros_events': ros_events,
        'pose_rows': pose_rows,
        'heartbeat_rows': heartbeats,
        'servo_rows': servo,
        'attitude_rows': attitude,
        'local_position_rows': local_position,
        'statustext_rows': statustext,
        'component_logs': component_logs,
    }


def forced_land_after_partial_climb_fixture(drone_id: str) -> dict[str, list[dict[str, object]]]:
    ros_events = [
        {'wall_time_epoch_sec': '1.0', 'monotonic_sec': '1.0', 'ros_time_sec': '1.0', 'drone_id': drone_id, 'topic_name': 'alate_output_mission_control_state', 'event_type': 'mission_control_state', 'payload_json': json.dumps({'mc_state_name': 'Standby'})},
        {'wall_time_epoch_sec': '1.1', 'monotonic_sec': '1.1', 'ros_time_sec': '1.1', 'drone_id': drone_id, 'topic_name': 'alate_output_high_level_control_state', 'event_type': 'high_level_control_state', 'payload_json': json.dumps({'hlc_state_name': 'Ready'})},
        {'wall_time_epoch_sec': '2.0', 'monotonic_sec': '2.0', 'ros_time_sec': '2.0', 'drone_id': drone_id, 'topic_name': 'alate_input_operator_command', 'event_type': 'input_operator_command', 'payload_json': json.dumps({'op_com_name': 'takeoff'})},
        {'wall_time_epoch_sec': '2.05', 'monotonic_sec': '2.05', 'ros_time_sec': '2.05', 'drone_id': drone_id, 'topic_name': 'alate_output_mission_control_state', 'event_type': 'mission_control_state', 'payload_json': json.dumps({'mc_state_name': 'TakingOff'})},
        {'wall_time_epoch_sec': '2.06', 'monotonic_sec': '2.06', 'ros_time_sec': '2.06', 'drone_id': drone_id, 'topic_name': 'alate_output_high_level_control_state', 'event_type': 'high_level_control_state', 'payload_json': json.dumps({'hlc_state_name': 'Takeoff'})},
        {'wall_time_epoch_sec': '2.2', 'monotonic_sec': '2.2', 'ros_time_sec': '2.2', 'drone_id': drone_id, 'topic_name': 'alate_output_high_level_control_state', 'event_type': 'high_level_control_state', 'payload_json': json.dumps({'hlc_state_name': 'GainingAltitude'})},
        {'wall_time_epoch_sec': '3.4', 'monotonic_sec': '3.4', 'ros_time_sec': '3.4', 'drone_id': drone_id, 'topic_name': 'alate_output_high_level_control_telemetry', 'event_type': 'hlc_telemetry', 'payload_json': json.dumps({'altitude': 1.1, 'yaw': 0.4, 'armed': True, 'mode': 'GUIDED', 'state': 'ACTIVE'})},
        {'wall_time_epoch_sec': '3.8', 'monotonic_sec': '3.8', 'ros_time_sec': '3.8', 'drone_id': drone_id, 'topic_name': 'alate_output_high_level_control_state', 'event_type': 'high_level_control_state', 'payload_json': json.dumps({'hlc_state_name': 'Landing'})},
        {'wall_time_epoch_sec': '3.85', 'monotonic_sec': '3.85', 'ros_time_sec': '3.85', 'drone_id': drone_id, 'topic_name': 'alate_output_mission_control_state', 'event_type': 'mission_control_state', 'payload_json': json.dumps({'mc_state_name': 'Landing'})},
    ]
    pose_rows = [
        {'wall_time_epoch_sec': '2.0', 'monotonic_sec': '2.0', 'sim_time_sec': '1.0', 'drone_id': drone_id, 'entity_name': 'iris', 'x': '0.0', 'y': '0.0', 'z': '0.0', 'roll': '0', 'pitch': '0', 'yaw': '0.0', 'pose_backlog_sec': '0.2'},
        {'wall_time_epoch_sec': '3.4', 'monotonic_sec': '3.4', 'sim_time_sec': '2.4', 'drone_id': drone_id, 'entity_name': 'iris', 'x': '0.0', 'y': '0.0', 'z': '1.1', 'roll': '0', 'pitch': '0', 'yaw': '0.4', 'pose_backlog_sec': '0.3'},
        {'wall_time_epoch_sec': '4.2', 'monotonic_sec': '4.2', 'sim_time_sec': '3.2', 'drone_id': drone_id, 'entity_name': 'iris', 'x': '0.0', 'y': '0.0', 'z': '0.1', 'roll': '0', 'pitch': '0', 'yaw': '0.45', 'pose_backlog_sec': '0.3'},
    ]
    heartbeats = [
        {'host_time_sec': '2.15', 'base_mode': '128', 'custom_mode': '4', 'system_status': '4', 'mav_type': '2', 'autopilot': '3', 'mode_string': 'GUIDED', 'armed': 'true'},
        {'host_time_sec': '3.6', 'base_mode': '128', 'custom_mode': '9', 'system_status': '4', 'mav_type': '2', 'autopilot': '3', 'mode_string': 'LAND', 'armed': 'true'},
    ]
    servo = [
        {'host_time_sec': '2.0', 'time_usec': '1', 'servo1_raw': '1000', 'servo2_raw': '1000', 'servo3_raw': '1000', 'servo4_raw': '1000'},
        {'host_time_sec': '2.3', 'time_usec': '2', 'servo1_raw': '1020', 'servo2_raw': '1005', 'servo3_raw': '1000', 'servo4_raw': '995'},
    ]
    attitude = [
        {'host_time_sec': '2.0', 'time_boot_ms': '100', 'roll': '0', 'pitch': '0', 'yaw': '0.0', 'rollspeed': '0', 'pitchspeed': '0', 'yawspeed': '0.0'},
        {'host_time_sec': '3.0', 'time_boot_ms': '200', 'roll': '0', 'pitch': '0', 'yaw': '0.35', 'rollspeed': '0', 'pitchspeed': '0', 'yawspeed': '0.4'},
    ]
    local_position = [
        {'host_time_sec': '2.0', 'time_boot_ms': '100', 'x': '0.0', 'y': '0.0', 'z': '0.0', 'vx': '0.0', 'vy': '0.0', 'vz': '0.0'},
        {'host_time_sec': '3.4', 'time_boot_ms': '240', 'x': '0.0', 'y': '0.0', 'z': '-1.1', 'vx': '0.0', 'vy': '0.0', 'vz': '-0.5'},
    ]
    statustext = []
    component_logs = {
        'autopilot.log': [
            {'component': 'autopilot_dronekit', 'event_type': 'takeoff_sent', 'wall_time_epoch_sec': 2.08, 'drone_id': drone_id, 'altitude': 5.0},
        ],
        'manual_runtime_test.log': [
            {'component': 'manual_runtime_test', 'event_type': 'operator_command_published', 'wall_time_epoch_sec': 2.0, 'drone_id': drone_id, 'command': 'takeoff'},
        ],
    }
    return {
        'ros_events': ros_events,
        'pose_rows': pose_rows,
        'heartbeat_rows': heartbeats,
        'servo_rows': servo,
        'attitude_rows': attitude,
        'local_position_rows': local_position,
        'statustext_rows': statustext,
        'component_logs': component_logs,
    }


def prearm_drone_fixture(drone_id: str) -> dict[str, list[dict[str, object]]]:
    ros_events = [
        {'wall_time_epoch_sec': '1.0', 'monotonic_sec': '1.0', 'ros_time_sec': '1.0', 'drone_id': drone_id, 'topic_name': 'alate_output_mission_control_state', 'event_type': 'mission_control_state', 'payload_json': json.dumps({'mc_state_name': 'Standby'})},
        {'wall_time_epoch_sec': '1.1', 'monotonic_sec': '1.1', 'ros_time_sec': '1.1', 'drone_id': drone_id, 'topic_name': 'alate_output_high_level_control_state', 'event_type': 'high_level_control_state', 'payload_json': json.dumps({'hlc_state_name': 'Ready'})},
        {'wall_time_epoch_sec': '2.0', 'monotonic_sec': '2.0', 'ros_time_sec': '2.0', 'drone_id': drone_id, 'topic_name': 'alate_input_operator_command', 'event_type': 'input_operator_command', 'payload_json': json.dumps({'op_com_name': 'takeoff'})},
    ]
    pose_rows = [
        {'wall_time_epoch_sec': '2.0', 'monotonic_sec': '2.0', 'sim_time_sec': '0.6', 'drone_id': drone_id, 'entity_name': 'iris', 'x': '0.0', 'y': '0.0', 'z': '0.0', 'roll': '0', 'pitch': '0', 'yaw': '0.0', 'pose_backlog_sec': '0.7'},
    ]
    heartbeats = []
    servo = [
        {'host_time_sec': '2.0', 'time_usec': '1', 'servo1_raw': '1000', 'servo2_raw': '1000', 'servo3_raw': '1000', 'servo4_raw': '1000'},
    ]
    attitude = [
        {'host_time_sec': '2.0', 'time_boot_ms': '100', 'roll': '0', 'pitch': '0', 'yaw': '0.0', 'rollspeed': '0', 'pitchspeed': '0', 'yawspeed': '0.0'},
    ]
    local_position = [
        {'host_time_sec': '2.0', 'time_boot_ms': '100', 'x': '0.0', 'y': '0.0', 'z': '0.0', 'vx': '0.0', 'vy': '0.0', 'vz': '0.0'},
    ]
    statustext = [
        {'host_time_sec': '2.2', 'severity': '3', 'text': 'PreArm: Gyros inconsistent'},
    ]
    component_logs = {
        'manual_runtime_test.log': [
            {'component': 'manual_runtime_test', 'event_type': 'operator_command_published', 'wall_time_epoch_sec': 2.0, 'drone_id': drone_id, 'command': 'takeoff'},
        ],
    }
    return {
        'ros_events': ros_events,
        'pose_rows': pose_rows,
        'heartbeat_rows': heartbeats,
        'servo_rows': servo,
        'attitude_rows': attitude,
        'local_position_rows': local_position,
        'statustext_rows': statustext,
        'component_logs': component_logs,
    }


def make_run(tmp_path: Path, name: str, mode: str, drone_fixtures: dict[str, dict[str, list[dict[str, object]]]], rtf: float, backlog: float, cpu_peak: float) -> Path:
    run_dir = tmp_path / name
    profile_dir = run_dir / 'diagnostics' / 'profile'
    logs_dir = run_dir / 'logs'
    profile_dir.mkdir(parents=True)
    logs_dir.mkdir(parents=True)

    manifest = {
        'run_id': name,
        'mode': mode,
        'fleet': mode == 'fleet',
        'visual_gui': False,
        'drones': [{'id': drone_id} for drone_id in drone_fixtures],
    }
    (profile_dir / 'profile.manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')

    all_ros_events: list[dict[str, object]] = []
    all_pose_rows: list[dict[str, object]] = []
    for drone_id, fixture in drone_fixtures.items():
        all_ros_events.extend(fixture['ros_events'])
        all_pose_rows.extend(fixture['pose_rows'])

        mavlink_dir = profile_dir / 'mavlink' / drone_id
        write_csv(
            mavlink_dir / 'servo_output_raw.csv',
            ['host_time_sec', 'time_usec', 'servo1_raw', 'servo2_raw', 'servo3_raw', 'servo4_raw'],
            fixture['servo_rows'],
        )
        write_csv(
            mavlink_dir / 'attitude.csv',
            ['host_time_sec', 'time_boot_ms', 'roll', 'pitch', 'yaw', 'rollspeed', 'pitchspeed', 'yawspeed'],
            fixture['attitude_rows'],
        )
        write_csv(
            mavlink_dir / 'local_position_ned.csv',
            ['host_time_sec', 'time_boot_ms', 'x', 'y', 'z', 'vx', 'vy', 'vz'],
            fixture['local_position_rows'],
        )
        write_csv(
            mavlink_dir / 'heartbeat.csv',
            ['host_time_sec', 'base_mode', 'custom_mode', 'system_status', 'mav_type', 'autopilot', 'mode_string', 'armed'],
            fixture['heartbeat_rows'],
        )
        write_csv(
            mavlink_dir / 'statustext.csv',
            ['host_time_sec', 'severity', 'text'],
            fixture['statustext_rows'],
        )

        for log_name, payloads in fixture['component_logs'].items():
            append_profile_log(logs_dir / drone_id / log_name, payloads, include_malformed=(log_name == 'manual_runtime_test.log'))

    all_ros_events.sort(key=lambda row: float(str(row['wall_time_epoch_sec'])))
    all_pose_rows.sort(key=lambda row: float(str(row['wall_time_epoch_sec'])))

    write_csv(profile_dir / 'ros_events.csv', list(all_ros_events[0].keys()), all_ros_events)
    write_csv(
        profile_dir / 'world_stats.csv',
        ['wall_time_epoch_sec', 'monotonic_sec', 'sim_time_sec', 'real_time_sec', 'pause_time_sec', 'real_time_factor', 'iterations', 'model_count', 'paused', 'step_size_sec', 'stepping'],
        base_world_rows(rtf),
    )
    write_csv(
        profile_dir / 'pose_trace.csv',
        ['wall_time_epoch_sec', 'monotonic_sec', 'sim_time_sec', 'drone_id', 'entity_name', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'pose_backlog_sec'],
        all_pose_rows,
    )
    write_csv(
        profile_dir / 'camera_frames.csv',
        ['wall_time_epoch_sec', 'monotonic_sec', 'drone_id', 'stream_name', 'width', 'height', 'source_sim_time_sec', 'stream_backlog_sec', 'interframe_gap_sec'],
        base_camera_rows(list(drone_fixtures.keys()), backlog),
    )
    write_csv(
        profile_dir / 'container_stats.csv',
        ['wall_time_epoch_sec', 'container_name', 'cpu_percent', 'mem_usage_bytes', 'mem_limit_bytes', 'mem_percent', 'net_in_bytes', 'net_out_bytes', 'block_in_bytes', 'block_out_bytes', 'pids'],
        base_container_rows(cpu_peak),
    )
    write_csv(
        profile_dir / 'gpu_stats.csv',
        ['wall_time_epoch_sec', 'gpu_index', 'gpu_name', 'utilization_gpu_percent', 'utilization_memory_percent', 'memory_used_mib', 'memory_total_mib'],
        base_gpu_rows(20.0 if mode == 'single' else 35.0),
    )
    return run_dir


def test_analyze_stack_profile_single_success(tmp_path: Path) -> None:
    run_dir = make_run(
        tmp_path,
        'single-success-run',
        'single',
        {'drone_1': success_drone_fixture('drone_1')},
        rtf=0.98,
        backlog=0.2,
        cpu_peak=90.0,
    )
    subprocess.run([sys.executable, str(ANALYZE), '--run-dir', str(run_dir)], check=True)

    profile_dir = run_dir / 'diagnostics' / 'profile'
    summary = json.loads((profile_dir / 'summary.json').read_text())
    control_summary = json.loads((profile_dir / 'control_summary.json').read_text())
    control_windows = (profile_dir / 'control_windows.csv').read_text()
    control_events = (profile_dir / 'control_events.csv').read_text()

    assert summary['run_id'] == 'single-success-run'
    assert summary['drones']['drone_1']['command_to_first_actuator_change_latency']['wall_sec'] is not None
    assert control_summary['drones']['drone_1']['latest_takeoff']['outcome'] == 'success_climb'
    assert 'takeoff_attempt' in control_windows
    assert 'statustext' in control_events or 'heartbeat' in control_events
    assert (profile_dir / 'control_summary.md').exists()


def test_analyze_stack_profile_swarm_failures(tmp_path: Path) -> None:
    run_dir = make_run(
        tmp_path,
        'fleet-failure-run',
        'fleet',
        {
            'drone_1': yaw_spin_drone_fixture('drone_1'),
            'drone_2': prearm_drone_fixture('drone_2'),
        },
        rtf=0.42,
        backlog=1.4,
        cpu_peak=180.0,
    )
    subprocess.run([sys.executable, str(ANALYZE), '--run-dir', str(run_dir)], check=True)

    profile_dir = run_dir / 'diagnostics' / 'profile'
    control_summary = json.loads((profile_dir / 'control_summary.json').read_text())

    assert control_summary['drones']['drone_1']['latest_takeoff']['outcome'] == 'yaw_spin_no_climb'
    assert control_summary['drones']['drone_2']['latest_takeoff']['outcome'] == 'prearm_blocked'
    assert 'Gyros inconsistent' in control_summary['drones']['drone_2']['latest_takeoff']['last_warning_text']


def test_analyze_stack_profile_forced_landing_beats_partial_climb(tmp_path: Path) -> None:
    run_dir = make_run(
        tmp_path,
        'forced-land-run',
        'fleet',
        {'drone_1': forced_land_after_partial_climb_fixture('drone_1')},
        rtf=0.92,
        backlog=0.3,
        cpu_peak=120.0,
    )
    subprocess.run([sys.executable, str(ANALYZE), '--run-dir', str(run_dir)], check=True)

    profile_dir = run_dir / 'diagnostics' / 'profile'
    control_summary = json.loads((profile_dir / 'control_summary.json').read_text())

    assert control_summary['drones']['drone_1']['latest_takeoff']['outcome'] == 'forced_land_after_takeoff_timeout'


def test_compare_stack_profiles_includes_control_section(tmp_path: Path) -> None:
    baseline = make_run(
        tmp_path,
        'baseline-run',
        'single',
        {'drone_1': success_drone_fixture('drone_1')},
        rtf=1.0,
        backlog=0.2,
        cpu_peak=90.0,
    )
    candidate = make_run(
        tmp_path,
        'candidate-run',
        'fleet',
        {
            'drone_1': yaw_spin_drone_fixture('drone_1'),
            'drone_2': prearm_drone_fixture('drone_2'),
        },
        rtf=0.38,
        backlog=1.8,
        cpu_peak=200.0,
    )
    subprocess.run([sys.executable, str(ANALYZE), '--run-dir', str(baseline)], check=True)
    subprocess.run([sys.executable, str(ANALYZE), '--run-dir', str(candidate)], check=True)
    subprocess.run([sys.executable, str(COMPARE), '--baseline', str(baseline), '--candidate', str(candidate)], check=True)

    comparison = json.loads((candidate / 'diagnostics' / 'profile' / 'comparison.json').read_text())
    comparison_md = (candidate / 'diagnostics' / 'profile' / 'comparison.md').read_text()

    assert comparison['world_deltas']['rtf_median_delta'] is not None
    assert comparison['control']['drones']['drone_1']['candidate_outcome'] == 'yaw_spin_no_climb'
    assert comparison['control']['drones']['drone_2']['candidate_outcome'] == 'prearm_blocked'
    assert any(item['title'] == 'Yaw instability before liftoff' for item in comparison['control']['root_cause_candidates'])
    assert '## Control' in comparison_md
    assert 'Takeoff outcome divergence' in comparison_md
