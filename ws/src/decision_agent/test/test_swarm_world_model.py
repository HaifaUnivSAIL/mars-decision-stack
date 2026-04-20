import math

from decision_agent.swarm_world_model import MC_STATE_ERROR, SwarmWorldModel
from decision_agent.world_model import TelemetryState


def test_swarm_world_model_derives_local_pose_and_ready_state():
    model = SwarmWorldModel(['drone_1', 'drone_2'])

    model.update_mc_state('drone_1', 2)
    model.update_hlc_state('drone_1', 4)
    model.update_telemetry(
        'drone_1',
        TelemetryState(latitude=32.0, longitude=34.0, altitude=100.0, yaw=0.1, armed=False, state='STANDBY'),
        10.0,
    )

    assert model.set_local_origin_from_drone('drone_1')

    model.update_mc_state('drone_2', 2)
    model.update_hlc_state('drone_2', 4)
    model.update_telemetry(
        'drone_2',
        TelemetryState(latitude=32.0001, longitude=34.0, altitude=101.5, yaw=0.2, armed=False, state='STANDBY'),
        10.0,
    )

    pose = model.local_pose('drone_2')
    assert pose is not None
    assert math.isclose(pose.x, 0.0, abs_tol=0.5)
    assert pose.y > 10.0
    assert math.isclose(pose.z, 1.5, abs_tol=1e-9)
    assert model.all_runtime_ready(['drone_1', 'drone_2'], 11.0, 2.0)


def test_swarm_world_model_reports_blocking_issue():
    model = SwarmWorldModel(['drone_1', 'drone_2'])
    for drone_id in ['drone_1', 'drone_2']:
        model.update_mc_state(drone_id, 2)
        model.update_hlc_state(drone_id, 4)
        model.update_telemetry(
            drone_id,
            TelemetryState(latitude=32.0, longitude=34.0, altitude=100.0, yaw=0.0, armed=False, state='STANDBY'),
            10.0,
        )

    model.update_mc_state('drone_2', MC_STATE_ERROR)
    assert model.first_blocking_issue(['drone_1', 'drone_2'], 11.0, 2.0) == ('drone_2', 'mission_control_error')
