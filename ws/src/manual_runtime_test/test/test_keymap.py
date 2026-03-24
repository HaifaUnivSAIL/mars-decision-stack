from manual_runtime_test.keymap import build_key_bindings, parse_script_action


def test_key_bindings_cover_expected_controls():
    bindings = build_key_bindings(0.4, 0.2)

    assert bindings['w'].kind == 'velocity'
    assert bindings['w'].linear_x == 0.4
    assert bindings['s'].linear_x == -0.4
    assert bindings['r'].linear_z == 0.2
    assert bindings['f'].linear_z == -0.2
    assert bindings['t'].operator_command == 'takeoff'
    assert bindings['l'].operator_command == 'land'
    assert bindings[' '].kind == 'stop'
    assert bindings['q'].kind == 'quit'


def test_parse_script_action_supports_wait_and_named_actions():
    action = parse_script_action('forward', 0.5, 0.3)
    wait_action = parse_script_action('wait:1.5', 0.5, 0.3)

    assert action.kind == 'velocity'
    assert action.linear_x == 0.5
    assert wait_action.kind == 'wait'
    assert wait_action.wait_sec == 1.5
