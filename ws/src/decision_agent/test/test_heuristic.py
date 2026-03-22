from decision_agent.policies.heuristic import compute_velocity_command
from decision_agent.world_model import WorldModel


def test_heuristic_outputs_zero_when_not_ready():
    world_model = WorldModel(mc_state=1, hlc_state=1)
    assert compute_velocity_command(world_model, telemetry_fresh=True, require_ready_state=True, linear_x=0.5, angular_z=0.1) == (0.0, 0.0)


def test_heuristic_outputs_requested_command_when_ready():
    world_model = WorldModel(mc_state=2, hlc_state=4)
    assert compute_velocity_command(world_model, telemetry_fresh=True, require_ready_state=True, linear_x=0.5, angular_z=0.1) == (0.5, 0.1)
