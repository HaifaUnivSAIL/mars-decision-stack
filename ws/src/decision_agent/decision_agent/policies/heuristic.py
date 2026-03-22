from .fsm import system_ready


def compute_velocity_command(world_model, telemetry_fresh: bool, require_ready_state: bool, linear_x: float, angular_z: float):
    if not telemetry_fresh:
        return 0.0, 0.0
    if require_ready_state and not system_ready(world_model):
        return 0.0, 0.0
    return linear_x, angular_z
