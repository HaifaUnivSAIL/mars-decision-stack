from __future__ import annotations

import argparse
import json
import os
import time
from typing import Sequence

import rclpy
from rclpy.node import Node
from ros_alate_interfaces.msg import OpCom


COMMAND_MAP = {
    'takeoff': OpCom.OP_COMMAND_TAKEOFF,
    'land': OpCom.OP_COMMAND_LAND,
    'gohome': OpCom.OP_COMMAND_GOHOME,
    'aux': OpCom.OP_COMMAND_AUX,
}


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Publish a namespaced operator command after subscriber discovery.')
    parser.add_argument('--op-command', required=True, choices=sorted(COMMAND_MAP.keys()))
    parser.add_argument('--duration-sec', type=float, default=2.0)
    parser.add_argument('--rate-hz', type=float, default=4.0)
    parser.add_argument('--wait-for-subscribers-sec', type=float, default=10.0)
    parser.add_argument('--discovery-settle-sec', type=float, default=0.5)
    return parser.parse_args(list(argv))


class CommandPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__('command_publisher')
        self._publisher = self.create_publisher(OpCom, 'alate_input_operator_command', 10)
        self._profile_enabled = os.getenv('STACK_PROFILE', '0') == '1'

    def wait_for_subscribers(self, timeout_sec: float, settle_sec: float) -> bool:
        deadline_ns = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline_ns:
            if self._publisher.get_subscription_count() > 0:
                if settle_sec > 0.0:
                    rclpy.spin_once(self, timeout_sec=min(settle_sec, 0.1))
                    self._publisher.get_subscription_count()
                    self._sleep(settle_sec)
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._publisher.get_subscription_count() > 0

    def publish_repeated(self, op_command: int, duration_sec: float, rate_hz: float) -> None:
        msg = OpCom()
        msg.op_com_enum = int(op_command)
        period_sec = 1.0 / max(rate_hz, 0.1)
        deadline_ns = self.get_clock().now().nanoseconds + int(duration_sec * 1e9)
        publish_count = 0
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline_ns:
            self._publisher.publish(msg)
            publish_count += 1
            if publish_count == 1:
                self._profile('operator_command_published', op_command=op_command)
            self._sleep(period_sec)
        self.get_logger().info(
            f'Published op_command={op_command} count={publish_count} subscribers={self._publisher.get_subscription_count()}'
        )

    def _sleep(self, seconds: float) -> None:
        remaining = max(seconds, 0.0)
        while rclpy.ok() and remaining > 0.0:
            step = min(remaining, 0.1)
            rclpy.spin_once(self, timeout_sec=step)
            remaining -= step

    def _profile(self, event_type: str, **payload) -> None:
        if not self._profile_enabled:
            return
        payload.update(
            {
                'component': 'command_publisher',
                'event_type': event_type,
                'wall_time_epoch_sec': time.time(),
                'ros_time_sec': self.get_clock().now().nanoseconds / 1e9,
            }
        )
        self.get_logger().info(f"PROFILE:{json.dumps(payload, separators=(',', ':'))}")


def main(args=None) -> int:
    rclpy.init(args=args)
    cli_args = parse_args(rclpy.utilities.remove_ros_args(args=args)[1:])
    node = CommandPublisherNode()
    exit_code = 0
    try:
        if not node.wait_for_subscribers(cli_args.wait_for_subscribers_sec, cli_args.discovery_settle_sec):
            node.get_logger().error('No subscribers discovered for alate_input_operator_command')
            exit_code = 1
        else:
            node.get_logger().info(
                f'Discovered {node._publisher.get_subscription_count()} subscribers; publishing {cli_args.op_command}'
            )
            node.publish_repeated(
                COMMAND_MAP[cli_args.op_command],
                duration_sec=cli_args.duration_sec,
                rate_hz=cli_args.rate_hz,
            )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code
