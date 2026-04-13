from __future__ import annotations

from dataclasses import dataclass
import json
import os
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from ros_alate_interfaces.msg import OpCom

from .ros_io import AlateRosIo
from .scenario_actions import ScenarioAction, parse_scenario_action
from .world_model import WorldModel


@dataclass
class ScenarioPhase:
    name: str
    started_sec: float


class ScenarioNode(Node):
    def __init__(self):
        super().__init__('policy_scenario')
        self.declare_parameter('control_frequency_hz', 10.0)
        self.declare_parameter('telemetry_timeout_sec', 2.0)
        self.declare_parameter('require_ready_state', True)
        self.declare_parameter('ready_hold_sec', 1.0)
        self.declare_parameter('auto_takeoff', True)
        self.declare_parameter('takeoff_altitude_m', 3.0)
        self.declare_parameter('takeoff_timeout_sec', 30.0)
        self.declare_parameter('land_after_actions', True)
        self.declare_parameter('landing_timeout_sec', 30.0)
        self.declare_parameter('landed_altitude_m', 0.3)
        self.declare_parameter('landing_touchdown_grace_sec', 8.0)
        self.declare_parameter('stop_after_actions_sec', 1.0)
        self.declare_parameter('scenario_name', 'axis_sweep')
        self.declare_parameter('scripted_actions', ['wait:1.0'])

        self._world_model = WorldModel()
        self._io = AlateRosIo(self, self._world_model)

        self._telemetry_timeout_sec = float(self.get_parameter('telemetry_timeout_sec').value)
        self._require_ready_state = bool(self.get_parameter('require_ready_state').value)
        self._ready_hold_sec = float(self.get_parameter('ready_hold_sec').value)
        self._auto_takeoff = bool(self.get_parameter('auto_takeoff').value)
        self._takeoff_altitude_m = float(self.get_parameter('takeoff_altitude_m').value)
        self._takeoff_timeout_sec = float(self.get_parameter('takeoff_timeout_sec').value)
        self._land_after_actions = bool(self.get_parameter('land_after_actions').value)
        self._landing_timeout_sec = float(self.get_parameter('landing_timeout_sec').value)
        self._landed_altitude_m = float(self.get_parameter('landed_altitude_m').value)
        self._landing_touchdown_grace_sec = float(self.get_parameter('landing_touchdown_grace_sec').value)
        self._stop_after_actions_sec = float(self.get_parameter('stop_after_actions_sec').value)
        self._scenario_name = str(self.get_parameter('scenario_name').value)
        self._actions = [
            parse_scenario_action(action_text)
            for action_text in list(self.get_parameter('scripted_actions').value)
            if action_text
        ]
        if not self._actions:
            raise ValueError('scripted_actions must contain at least one action')

        self._phase: Optional[ScenarioPhase] = None
        self._action_index = 0
        self._action_started_sec: Optional[float] = None
        self._scenario_started_sec: Optional[float] = None
        self._ready_observed_sec: Optional[float] = None
        self._takeoff_sent = False
        self._landing_sent = False
        self._landing_touchdown_sec: Optional[float] = None
        self._finished = False
        self._failed = False
        self._profile_enabled = os.getenv('STACK_PROFILE', '0') == '1'

        control_frequency_hz = max(float(self.get_parameter('control_frequency_hz').value), 0.1)
        self.create_timer(1.0 / control_frequency_hz, self._tick)
        self.get_logger().info(
            f'policy_scenario started scenario={self._scenario_name} actions={len(self._actions)} '
            f'auto_takeoff={self._auto_takeoff} land_after_actions={self._land_after_actions}'
        )

    def is_finished(self) -> bool:
        return self._finished

    def failed(self) -> bool:
        return self._failed

    def _tick(self) -> None:
        if self._finished:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self._phase is None:
            self._set_phase('waiting_ready', now_sec)

        if self._phase.name == 'waiting_ready':
            self._handle_waiting_ready(now_sec)
        elif self._phase.name == 'waiting_airborne':
            self._handle_waiting_airborne(now_sec)
        elif self._phase.name == 'running_actions':
            self._handle_running_actions(now_sec)
        elif self._phase.name == 'stopping_after_actions':
            self._handle_stopping_after_actions(now_sec)
        elif self._phase.name == 'landing':
            self._handle_landing(now_sec)

    def _handle_waiting_ready(self, now_sec: float) -> None:
        if not self._world_model.is_runtime_ready(now_sec, self._telemetry_timeout_sec, self._require_ready_state):
            self._ready_observed_sec = None
            return

        if self._ready_observed_sec is None:
            self._ready_observed_sec = now_sec
            self.get_logger().info('Runtime ready detected; holding briefly before scenario start')
            return

        if (now_sec - self._ready_observed_sec) < self._ready_hold_sec:
            return

        if not self._auto_takeoff:
            self._scenario_started_sec = now_sec
            self._set_phase('running_actions', now_sec)
            return

        if not self._takeoff_sent:
            self._io.publish_operator_command(OpCom.OP_COMMAND_TAKEOFF)
            self._takeoff_sent = True
            self.get_logger().info('Published takeoff command')
            self._profile('operator_command_published', operator_command='takeoff')
        self._set_phase('waiting_airborne', now_sec)

    def _handle_waiting_airborne(self, now_sec: float) -> None:
        if self._world_model.is_airborne(now_sec, self._telemetry_timeout_sec, self._takeoff_altitude_m):
            self._scenario_started_sec = now_sec
            self._set_phase('running_actions', now_sec)
            return

        if (now_sec - self._phase.started_sec) > self._takeoff_timeout_sec:
            self._fail(now_sec, 'Timed out waiting for airborne state')

    def _handle_running_actions(self, now_sec: float) -> None:
        if self._action_index >= len(self._actions):
            self._io.publish_zero()
            self._set_phase('stopping_after_actions', now_sec)
            return

        action = self._actions[self._action_index]
        if self._action_started_sec is None:
            self._action_started_sec = now_sec
            self.get_logger().info(f'Starting action {self._action_index + 1}/{len(self._actions)}: {action.label}')

        elapsed_sec = now_sec - self._action_started_sec
        if action.kind == 'velocity':
            self._io.publish_twist(
                linear_x=action.linear_x,
                linear_y=action.linear_y,
                linear_z=action.linear_z,
            )
            self._profile(
                'velocity_command_published',
                label=action.label,
                linear_x=action.linear_x,
                linear_y=action.linear_y,
                linear_z=action.linear_z,
            )
        elif action.kind == 'stop':
            self._io.publish_zero()
            self._profile('zero_command_published', reason='scenario_stop_action')

        if elapsed_sec >= action.duration_sec:
            self._action_index += 1
            self._action_started_sec = None

    def _handle_stopping_after_actions(self, now_sec: float) -> None:
        self._io.publish_zero()
        self._profile('zero_command_published', reason='scenario_stop_after_actions')
        if (now_sec - self._phase.started_sec) < self._stop_after_actions_sec:
            return
        if self._land_after_actions:
            if not self._landing_sent:
                self._landing_touchdown_sec = None
                self._io.publish_operator_command(OpCom.OP_COMMAND_LAND)
                self._landing_sent = True
                self.get_logger().info('Published landing command')
                self._profile('operator_command_published', operator_command='land')
            self._set_phase('landing', now_sec)
        else:
            self._finish(now_sec)

    def _handle_landing(self, now_sec: float) -> None:
        self._io.publish_zero()
        if self._world_model.is_landed(now_sec, self._telemetry_timeout_sec, self._landed_altitude_m):
            self._finish(now_sec)
            return
        telemetry = self._world_model.telemetry
        if self._world_model.telemetry_is_fresh(now_sec, self._telemetry_timeout_sec):
            if telemetry.altitude <= self._landed_altitude_m and self._landing_touchdown_sec is None:
                self._landing_touchdown_sec = now_sec
                self.get_logger().info(
                    f'Touchdown detected at altitude={telemetry.altitude:.2f}m while waiting for disarm'
                )
            if self._landing_touchdown_sec is not None:
                touchdown_elapsed_sec = now_sec - self._landing_touchdown_sec
                if touchdown_elapsed_sec <= self._landing_touchdown_grace_sec:
                    return
                self._fail(
                    now_sec,
                    (
                        f'Landing reached ground but stayed armed for {touchdown_elapsed_sec:.1f}s '
                        f'(grace {self._landing_touchdown_grace_sec:.1f}s)'
                    ),
                )
                return
        if (now_sec - self._phase.started_sec) > self._landing_timeout_sec:
            self._fail(
                now_sec,
                (
                    f'Landing timeout elapsed after {self._landing_timeout_sec:.1f}s '
                    f'at altitude={telemetry.altitude:.2f}m armed={telemetry.armed}'
                ),
            )

    def _set_phase(self, name: str, now_sec: float) -> None:
        self._phase = ScenarioPhase(name=name, started_sec=now_sec)
        self.get_logger().info(f'Scenario phase -> {name}')
        self._profile('scenario_phase_entered', phase=name)

    def _finish(self, now_sec: float) -> None:
        self._io.publish_zero()
        total_sec = 0.0 if self._scenario_started_sec is None else now_sec - self._scenario_started_sec
        self.get_logger().info(f'Scenario finished after {total_sec:.2f}s')
        self._finished = True

    def _fail(self, now_sec: float, reason: str) -> None:
        self.get_logger().error(reason)
        self._failed = True
        self._finish(now_sec)

    def _profile(self, event_type: str, **payload) -> None:
        if not self._profile_enabled:
            return
        payload.update(
            {
                'component': 'policy_scenario',
                'event_type': event_type,
                'wall_time_epoch_sec': time.time(),
                'ros_time_sec': self.get_clock().now().nanoseconds / 1e9,
                'scenario_name': self._scenario_name,
            }
        )
        self.get_logger().info(f"PROFILE:{json.dumps(payload, separators=(',', ':'))}")


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioNode()
    exit_code = 0
    try:
        while rclpy.ok() and not node.is_finished():
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.failed():
            exit_code = 1
    finally:
        if rclpy.ok():
            node._io.publish_zero()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code
