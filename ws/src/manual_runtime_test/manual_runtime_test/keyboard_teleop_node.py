import select
import sys
import termios
import time
import tty
from typing import Optional

import rclpy
from rclpy.node import Node
from ros_alate_interfaces.msg import OpCom

from .keymap import build_key_bindings, parse_script_action, render_help
from .ros_io import ManualRuntimeRosIo
from .runtime_state import RuntimeState


class TerminalReader:
    def __init__(self, stream):
        self._stream = stream
        self._fd = stream.fileno()
        self._settings = None

    def enable(self) -> None:
        if not self._stream.isatty():
            raise RuntimeError('Interactive keyboard mode requires a TTY.')
        self._settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)

    def restore(self) -> None:
        if self._settings is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._settings)
            self._settings = None

    def read_key(self, timeout_sec: float) -> Optional[str]:
        ready, _, _ = select.select([self._stream], [], [], timeout_sec)
        if not ready:
            return None

        char = self._stream.read(1)
        if char == '\x03':
            return 'q'
        if char == '\x1b':
            select.select([self._stream], [], [], 0.0)
            return None
        return char


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('manual_runtime_test')

        self.declare_parameter('linear_step_x', 0.5)
        self.declare_parameter('linear_step_y', 0.5)
        self.declare_parameter('linear_step_z', 0.3)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('motion_hold_sec', 0.0)
        self.declare_parameter('telemetry_timeout_sec', 2.0)
        self.declare_parameter('require_ready_state_for_takeoff', True)
        self.declare_parameter('status_period_sec', 1.0)
        self.declare_parameter('startup_snapshot_timeout_sec', 5.0)
        self.declare_parameter('motion_debug_window_sec', 3.0)
        self.declare_parameter('scripted_action_interval_sec', 0.25)
        self.declare_parameter('scripted_actions', [''])

        self._linear_step_x = float(self.get_parameter('linear_step_x').value)
        self._linear_step_y = float(self.get_parameter('linear_step_y').value)
        self._linear_step_z = float(self.get_parameter('linear_step_z').value)
        self._publish_rate_hz = max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self._motion_hold_sec = max(float(self.get_parameter('motion_hold_sec').value), 0.0)
        self._telemetry_timeout_sec = float(self.get_parameter('telemetry_timeout_sec').value)
        self._require_ready_state_for_takeoff = bool(self.get_parameter('require_ready_state_for_takeoff').value)
        self._status_period_sec = max(float(self.get_parameter('status_period_sec').value), 0.1)
        self._startup_snapshot_timeout_sec = max(float(self.get_parameter('startup_snapshot_timeout_sec').value), 0.5)
        self._motion_debug_window_sec = max(float(self.get_parameter('motion_debug_window_sec').value), 0.0)
        self._scripted_action_interval_sec = max(float(self.get_parameter('scripted_action_interval_sec').value), 0.0)
        self._scripted_actions = [
            action for action in list(self.get_parameter('scripted_actions').value) if action
        ]

        self._runtime_state = RuntimeState()
        self._io = ManualRuntimeRosIo(self, self._runtime_state)
        self._key_bindings = build_key_bindings(self._linear_step_x, self._linear_step_y, self._linear_step_z)
        self._terminal = TerminalReader(sys.stdin)

        self._running = True
        self._current_linear_x = 0.0
        self._current_linear_y = 0.0
        self._current_linear_z = 0.0
        self._last_motion_command_sec: Optional[float] = None
        self._last_publish_sec = 0.0
        self._last_status_sec = 0.0
        self._next_script_action_sec = self._now_sec()
        self._script_index = 0
        self._last_reported_error: Optional[str] = None
        self._startup_started_sec = self._next_script_action_sec
        self._initial_status_reported = False
        self._waiting_for_snapshot_reported = False
        self._last_logged_motion_signature: Optional[tuple[str, float, float, float]] = None

        self.get_logger().info('manual_runtime_test started')
        self.get_logger().info(render_help())
        if self._scripted_actions:
            self.get_logger().info(f'Running in scripted mode with actions: {self._scripted_actions}')

    def enable_keyboard(self) -> None:
        if not self._scripted_actions:
            self._terminal.enable()

    def shutdown(self) -> None:
        try:
            if rclpy.ok():
                self._io.publish_zero()
        except Exception as exc:
            self.get_logger().debug(f'Skipped final zero publish during shutdown: {exc}')
        finally:
            self._terminal.restore()

    def is_running(self) -> bool:
        return self._running and rclpy.ok()

    def is_scripted_mode(self) -> bool:
        return bool(self._scripted_actions)

    def read_key(self, timeout_sec: float) -> Optional[str]:
        if self._scripted_actions:
            return None
        return self._terminal.read_key(timeout_sec)

    def step(self, key: Optional[str]) -> None:
        if key is not None:
            self._handle_key(key)

        now_sec = self._now_sec()
        self._run_scripted_actions(now_sec)
        self._publish_motion(now_sec)
        self._print_status(now_sec)

    def _handle_key(self, key: str) -> None:
        action = self._key_bindings.get(key.lower())
        if action is None:
            self.get_logger().info(f'Ignored key: {repr(key)}')
            return
        self._apply_action(action)

    def _run_scripted_actions(self, now_sec: float) -> None:
        if not self._scripted_actions or self._script_index >= len(self._scripted_actions):
            return
        if now_sec < self._next_script_action_sec:
            return

        action_text = self._scripted_actions[self._script_index]
        action = parse_script_action(action_text, self._linear_step_x, self._linear_step_y, self._linear_step_z)

        if action.kind == 'wait':
            self.get_logger().info(f'Scripted action: {action.label}')
            self._script_index += 1
            self._next_script_action_sec = now_sec + action.wait_sec
            return

        if self._apply_action(action):
            self._script_index += 1
        self._next_script_action_sec = now_sec + self._scripted_action_interval_sec

    def _apply_action(self, action) -> bool:
        now_sec = self._now_sec()

        if action.kind == 'help':
            self.get_logger().info(render_help())
            return True

        if action.kind == 'quit':
            self.get_logger().info('Quit requested')
            self._running = False
            return True

        if action.kind == 'stop':
            self._current_linear_x = 0.0
            self._current_linear_y = 0.0
            self._current_linear_z = 0.0
            self._last_motion_command_sec = None
            self._last_logged_motion_signature = None
            self._runtime_state.mark_command_reference(action.label, now_sec)
            self._io.publish_zero()
            self._last_publish_sec = now_sec
            self.get_logger().info('Published stop command')
            return True

        if action.kind == 'operator_command':
            return self._publish_operator_command(action.operator_command)

        if action.kind == 'velocity':
            if not self._runtime_state.telemetry_is_fresh(now_sec, self._telemetry_timeout_sec):
                self.get_logger().warning('Ignored motion command because telemetry is stale or unavailable')
                return False

            self._current_linear_x = action.linear_x
            self._current_linear_y = action.linear_y
            self._current_linear_z = action.linear_z
            self._last_motion_command_sec = now_sec
            self._runtime_state.mark_command_reference(action.label, now_sec)
            self._io.publish_velocity(self._current_linear_x, self._current_linear_y, self._current_linear_z)
            self._last_publish_sec = now_sec
            motion_signature = (
                action.label,
                self._current_linear_x,
                self._current_linear_y,
                self._current_linear_z,
            )
            if motion_signature != self._last_logged_motion_signature:
                self.get_logger().info(
                    f'Published motion command {action.label}: '
                    f'linear_x={self._current_linear_x:.2f} '
                    f'linear_y={self._current_linear_y:.2f} '
                    f'linear_z={self._current_linear_z:.2f}'
                )
                self._last_logged_motion_signature = motion_signature
            return True

        return True

    def _publish_operator_command(self, operator_command: str) -> bool:
        now_sec = self._now_sec()
        if operator_command == 'takeoff' and self._require_ready_state_for_takeoff and not self._runtime_state.is_takeoff_ready(
            now_sec, self._telemetry_timeout_sec
        ):
            self.get_logger().warning(
                'Ignored takeoff command because runtime is not ready '
                f'(MC={self._runtime_state.mc_state_name()} HLC={self._runtime_state.hlc_state_name()})'
            )
            return False

        mapping = {
            'takeoff': OpCom.OP_COMMAND_TAKEOFF,
            'land': OpCom.OP_COMMAND_LAND,
            'gohome': OpCom.OP_COMMAND_GOHOME,
        }
        self._io.publish_operator_command(mapping[operator_command])
        self.get_logger().info(f'Published operator command: {operator_command}')
        self._last_publish_sec = now_sec
        return True

    def _publish_motion(self, now_sec: float) -> None:
        publish_period_sec = 1.0 / self._publish_rate_hz

        if self._last_motion_command_sec is None:
            return

        if self._motion_hold_sec > 0.0 and (now_sec - self._last_motion_command_sec) > self._motion_hold_sec:
            if self._current_linear_x != 0.0 or self._current_linear_y != 0.0 or self._current_linear_z != 0.0:
                self._current_linear_x = 0.0
                self._current_linear_y = 0.0
                self._current_linear_z = 0.0
                self._io.publish_zero()
                self._last_publish_sec = now_sec
                self.get_logger().info('Motion hold timeout elapsed, published stop command')
            self._last_motion_command_sec = None
            return

        if (now_sec - self._last_publish_sec) >= publish_period_sec:
            self._io.publish_velocity(self._current_linear_x, self._current_linear_y, self._current_linear_z)
            self._last_publish_sec = now_sec

    def _print_status(self, now_sec: float) -> None:
        if not self._initial_status_reported:
            if self._runtime_state.has_complete_status_snapshot(now_sec, self._telemetry_timeout_sec):
                self._initial_status_reported = True
                self._waiting_for_snapshot_reported = False
            else:
                if (
                    not self._waiting_for_snapshot_reported and
                    (now_sec - self._startup_started_sec) >= self._status_period_sec
                ):
                    self.get_logger().info('Waiting for initial runtime snapshot (MC/HLC + telemetry)...')
                    self._waiting_for_snapshot_reported = True
                if (now_sec - self._startup_started_sec) < self._startup_snapshot_timeout_sec:
                    return
                if not self._waiting_for_snapshot_reported:
                    self.get_logger().info('Waiting for initial runtime snapshot (MC/HLC + telemetry)...')
                    self._waiting_for_snapshot_reported = True
                self.get_logger().warning(
                    'Startup snapshot timeout elapsed; showing partial runtime status while subscriptions continue to settle'
                )
                self._initial_status_reported = True

        if (now_sec - self._last_status_sec) < self._status_period_sec:
            return
        self._last_status_sec = now_sec
        self.get_logger().info(
            self._runtime_state.status_line(
                now_sec,
                self._telemetry_timeout_sec,
                self._motion_debug_window_sec,
            )
        )
        if self._runtime_state.last_error:
            if self._runtime_state.last_error != self._last_reported_error:
                self.get_logger().warning(f'Latest platform error: {self._runtime_state.last_error}')
                self._last_reported_error = self._runtime_state.last_error
        else:
            self._last_reported_error = None

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    timeout_sec = 0.05

    try:
        node.enable_keyboard()
        while node.is_running():
            rclpy.spin_once(node, timeout_sec=0.0 if not node.is_scripted_mode() else timeout_sec)
            key = node.read_key(timeout_sec)
            node.step(key)
            if node.is_scripted_mode():
                time.sleep(timeout_sec)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
