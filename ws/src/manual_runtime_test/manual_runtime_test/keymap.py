from dataclasses import dataclass
from typing import Dict


@dataclass(frozen=True)
class Action:
    kind: str
    label: str
    linear_x: float = 0.0
    linear_z: float = 0.0
    operator_command: str = ''
    wait_sec: float = 0.0


def build_named_actions(linear_step_x: float, linear_step_z: float) -> Dict[str, Action]:
    return {
        'forward': Action(kind='velocity', label='forward', linear_x=linear_step_x),
        'back': Action(kind='velocity', label='back', linear_x=-linear_step_x),
        'up': Action(kind='velocity', label='up', linear_z=linear_step_z),
        'down': Action(kind='velocity', label='down', linear_z=-linear_step_z),
        'stop': Action(kind='stop', label='stop'),
        'takeoff': Action(kind='operator_command', label='takeoff', operator_command='takeoff'),
        'land': Action(kind='operator_command', label='land', operator_command='land'),
        'gohome': Action(kind='operator_command', label='gohome', operator_command='gohome'),
        'help': Action(kind='help', label='help'),
        'quit': Action(kind='quit', label='quit'),
    }


def build_key_bindings(linear_step_x: float, linear_step_z: float) -> Dict[str, Action]:
    named_actions = build_named_actions(linear_step_x, linear_step_z)
    return {
        'w': named_actions['forward'],
        's': named_actions['back'],
        'r': named_actions['up'],
        'f': named_actions['down'],
        ' ': named_actions['stop'],
        't': named_actions['takeoff'],
        'l': named_actions['land'],
        'g': named_actions['gohome'],
        'h': named_actions['help'],
        'q': named_actions['quit'],
    }


def parse_script_action(action_text: str, linear_step_x: float, linear_step_z: float) -> Action:
    normalized = action_text.strip().lower()
    if normalized.startswith('wait:'):
        wait_sec = float(normalized.split(':', 1)[1])
        return Action(kind='wait', label=f'wait {wait_sec:.2f}s', wait_sec=wait_sec)

    named_actions = build_named_actions(linear_step_x, linear_step_z)
    if normalized not in named_actions:
        raise ValueError(f'Unsupported scripted action: {action_text}')
    return named_actions[normalized]


def render_help() -> str:
    return (
        'Controls: '
        '[t] takeoff  '
        '[l] land  '
        '[g] gohome  '
        '[w] forward  '
        '[s] back  '
        '[r] up  '
        '[f] down  '
        '[space] stop  '
        '[h] help  '
        '[q] quit'
    )
