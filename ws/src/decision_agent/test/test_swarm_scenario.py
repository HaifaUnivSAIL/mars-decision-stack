import math
from pathlib import Path

from decision_agent.swarm_geometry import CenterTarget, LocalPose
from decision_agent.swarm_scenario import (
    FormationOffset,
    SwarmControllerConfig,
    SwarmScenarioAction,
    compute_body_frame_command,
    compute_initial_center_target,
    desired_drone_pose,
    interpolate_center_target,
    load_swarm_scenario,
    resolve_formation_offsets,
)


def test_load_swarm_scenario(tmp_path: Path):
    scenario_path = tmp_path / 'scenario.yaml'
    scenario_path.write_text(
        '\n'.join(
            [
                'scenario_name: unit_swarm',
                'control_frequency_hz: 5.0',
                'actions:',
                '  - hold: 1.0',
                '  - translate_world:',
                '      x: 2.0',
                '      y: 0.0',
                '      z: 0.5',
                '      duration_sec: 4.0',
            ]
        )
        + '\n'
    )

    scenario = load_swarm_scenario(scenario_path)
    assert scenario.scenario_name == 'unit_swarm'
    assert scenario.control_frequency_hz == 5.0
    assert [action.kind for action in scenario.actions] == ['hold', 'translate_world']


def test_resolve_formation_offsets_defaults_to_spawn_layout():
    manifest = {
        'active_drone_id': 'drone_1',
        'drones': [
            {'id': 'drone_1', 'spawn': {'x': 0.0, 'y': 0.0, 'z': 0.2}},
            {'id': 'drone_2', 'spawn': {'x': 2.0, 'y': 4.0, 'z': 0.2}},
        ],
    }

    offsets = resolve_formation_offsets(manifest, {})
    assert offsets['drone_1'] == FormationOffset(x=0.0, y=0.0, z=0.0, yaw_rad=0.0)
    assert offsets['drone_2'] == FormationOffset(x=2.0, y=4.0, z=0.0, yaw_rad=0.0)


def test_center_target_and_desired_pose_round_trip():
    offsets = {
        'drone_1': FormationOffset(x=0.0, y=0.0),
        'drone_2': FormationOffset(x=2.0, y=0.0),
    }
    local_poses = {
        'drone_1': LocalPose(x=10.0, y=5.0, z=3.0, yaw=0.0),
        'drone_2': LocalPose(x=12.0, y=5.0, z=3.0, yaw=0.0),
    }

    center = compute_initial_center_target(local_poses, offsets)
    assert math.isclose(center.x, 10.0, abs_tol=1e-9)
    assert math.isclose(center.y, 5.0, abs_tol=1e-9)

    desired = desired_drone_pose(center, offsets['drone_2'])
    assert math.isclose(desired.x, 12.0, abs_tol=1e-9)
    assert math.isclose(desired.y, 5.0, abs_tol=1e-9)


def test_compute_body_frame_command_respects_bounds_and_deadbands():
    actual = LocalPose(x=0.0, y=0.0, z=0.0, yaw=0.0)
    desired = LocalPose(x=10.0, y=0.0, z=1.0, yaw=math.radians(45.0))
    controller = SwarmControllerConfig(max_linear_xy_mps=0.5, max_linear_z_mps=0.3, max_angular_z_rad_s=0.4)

    command, error = compute_body_frame_command(actual, desired, controller)
    assert command['linear_x'] == 0.5
    assert command['linear_z'] == 0.3
    assert command['angular_z'] == 0.4
    assert error['horizontal_error_m'] == 10.0


def test_interpolate_center_target_translate_body_uses_heading():
    start = CenterTarget(x=0.0, y=0.0, z=1.0, yaw=math.pi / 2.0)
    action = SwarmScenarioAction(kind='translate_body', duration_sec=4.0, delta_x=2.0, delta_y=0.0, delta_z=1.0)

    target = interpolate_center_target(start, action, 0.5)
    assert math.isclose(target.x, 0.0, abs_tol=1e-9)
    assert math.isclose(target.y, 1.0, abs_tol=1e-9)
    assert math.isclose(target.z, 1.5, abs_tol=1e-9)
