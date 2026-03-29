from decision_agent.scenario_actions import (
    ScenarioAction,
    estimate_action_schedule_duration,
    parse_scenario_action,
)


def test_parse_velocity_action():
    action = parse_scenario_action('velocity:0.4,-0.2,0.0,2.5')
    assert action == ScenarioAction(
        kind='velocity',
        linear_x=0.4,
        linear_y=-0.2,
        linear_z=0.0,
        duration_sec=2.5,
        label='velocity (0.40, -0.20, 0.00) for 2.50s',
    )


def test_parse_wait_action():
    action = parse_scenario_action('wait:1.5')
    assert action.kind == 'wait'
    assert action.duration_sec == 1.5


def test_estimate_schedule_duration():
    actions = [
        parse_scenario_action('wait:1.0'),
        parse_scenario_action('velocity:0.4,0.0,0.0,2.5'),
        parse_scenario_action('stop:0.5'),
    ]
    assert estimate_action_schedule_duration(actions) == 4.0
