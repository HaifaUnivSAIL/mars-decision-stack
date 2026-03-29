from dataclasses import dataclass


@dataclass(frozen=True)
class ScenarioAction:
    kind: str
    duration_sec: float
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    label: str = ''


def parse_scenario_action(action_text: str) -> ScenarioAction:
    normalized = action_text.strip().lower()
    if normalized.startswith('wait:'):
        duration_sec = float(normalized.split(':', 1)[1])
        return ScenarioAction(kind='wait', duration_sec=duration_sec, label=f'wait {duration_sec:.2f}s')

    if normalized.startswith('stop:'):
        duration_sec = float(normalized.split(':', 1)[1])
        return ScenarioAction(kind='stop', duration_sec=duration_sec, label=f'stop {duration_sec:.2f}s')

    if normalized.startswith('velocity:'):
        payload = normalized.split(':', 1)[1]
        parts = [part.strip() for part in payload.split(',')]
        if len(parts) != 4:
            raise ValueError(
                "Velocity action must be 'velocity:<linear_x>,<linear_y>,<linear_z>,<duration_sec>'"
            )
        linear_x, linear_y, linear_z, duration_sec = [float(part) for part in parts]
        return ScenarioAction(
            kind='velocity',
            linear_x=linear_x,
            linear_y=linear_y,
            linear_z=linear_z,
            duration_sec=duration_sec,
            label=f'velocity ({linear_x:.2f}, {linear_y:.2f}, {linear_z:.2f}) for {duration_sec:.2f}s',
        )

    raise ValueError(f'Unsupported scenario action: {action_text}')


def estimate_action_schedule_duration(actions: list[ScenarioAction]) -> float:
    return sum(action.duration_sec for action in actions)
