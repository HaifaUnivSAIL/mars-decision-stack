from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_alate_interfaces.msg import HlcPlatformError, HlcState, HlcTelemetry, McState, OpCom


MC_STATE_NAMES = {
    0: 'None',
    1: 'Init',
    2: 'Standby',
    3: 'TakingOff',
    4: 'PerformingMission',
    5: 'Landing',
    6: 'Manual',
    7: 'ReturnToLaunch',
    8: 'NoError',
    9: 'Error',
}

HLC_STATE_NAMES = {
    0: 'None',
    1: 'Init',
    2: 'WaitingForMc',
    3: 'WaitingForLlc',
    4: 'Ready',
    5: 'Takeoff',
    6: 'GainingAltitude',
    7: 'Airborne',
    8: 'Landing',
    9: 'Manual',
    10: 'ReturnToLaunch',
    11: 'NoError',
    12: 'LowBattery',
    13: 'LlcDown',
}

OPCOM_NAMES = {
    int(OpCom.OP_COMMAND_TAKEOFF): 'takeoff',
    int(OpCom.OP_COMMAND_LAND): 'land',
    int(OpCom.OP_COMMAND_GOHOME): 'gohome',
    int(OpCom.OP_COMMAND_SETDIRECTION): 'setdirection',
    int(OpCom.OP_COMMAND_AUX): 'aux',
}


class StackRosProfiler(Node):
    def __init__(self, profile_manifest_path: Path):
        super().__init__('stack_ros_profiler')
        profile_manifest = json.loads(profile_manifest_path.read_text())
        output_dir = Path(profile_manifest['output_dir'])
        if not output_dir.exists():
            output_dir = profile_manifest_path.resolve().parent
        output_dir.mkdir(parents=True, exist_ok=True)
        self._events_handle = (output_dir / 'ros_events.csv').open('a', newline='')
        self._events_writer = csv.DictWriter(
            self._events_handle,
            fieldnames=[
                'wall_time_epoch_sec',
                'monotonic_sec',
                'ros_time_sec',
                'drone_id',
                'topic_name',
                'event_type',
                'payload_json',
            ],
        )
        if self._events_handle.tell() == 0:
            self._events_writer.writeheader()
            self._events_handle.flush()
        self._flush_period_sec = float(profile_manifest.get('flush_period_sec', 1.0))
        self._last_flush_sec = time.monotonic()
        self._event_count = 0
        self._last_summary_sec = time.monotonic()
        self.create_timer(5.0, self._emit_summary)

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        for drone in profile_manifest.get('drones', []):
            drone_id = str(drone['id'])
            namespace = str(drone.get('namespace') or '').rstrip('/')
            self._subscribe(Twist, self._topic(namespace, 'alate_input_velocity'), drone_id, 'input_velocity', self._on_velocity)
            self._subscribe(OpCom, self._topic(namespace, 'alate_input_operator_command'), drone_id, 'input_operator_command', self._on_opcom)
            self._subscribe(McState, self._topic(namespace, 'alate_output_mission_control_state'), drone_id, 'mission_control_state', self._on_mc_state, state_qos)
            self._subscribe(HlcState, self._topic(namespace, 'alate_output_high_level_control_state'), drone_id, 'high_level_control_state', self._on_hlc_state, state_qos)
            self._subscribe(HlcTelemetry, self._topic(namespace, 'alate_output_high_level_control_telemetry'), drone_id, 'hlc_telemetry', self._on_telemetry)
            self._subscribe(
                HlcPlatformError,
                self._topic(namespace, 'alate_output_high_level_control_platform_errors'),
                drone_id,
                'platform_error',
                self._on_platform_error,
            )

        self.get_logger().info(f'stack_ros_profiler started for {len(profile_manifest.get("drones", []))} drones')

    @staticmethod
    def _topic(namespace: str, topic_name: str) -> str:
        if not namespace:
            return topic_name
        return f'{namespace}/{topic_name}'

    def destroy_node(self):
        try:
            self._events_handle.flush()
            self._events_handle.close()
        finally:
            super().destroy_node()

    def _subscribe(self, msg_type, topic_name: str, drone_id: str, event_type: str, handler, qos=10) -> None:
        def _callback(msg):
            handler(drone_id, topic_name, event_type, msg)

        self.create_subscription(msg_type, topic_name, _callback, qos)

    def _record_event(self, drone_id: str, topic_name: str, event_type: str, payload: dict) -> None:
        ros_time_sec = self.get_clock().now().nanoseconds / 1e9
        self._events_writer.writerow(
            {
                'wall_time_epoch_sec': f'{time.time():.6f}',
                'monotonic_sec': f'{time.monotonic():.6f}',
                'ros_time_sec': f'{ros_time_sec:.6f}',
                'drone_id': drone_id,
                'topic_name': topic_name,
                'event_type': event_type,
                'payload_json': json.dumps(payload, separators=(',', ':')),
            }
        )
        now = time.monotonic()
        if (now - self._last_flush_sec) >= self._flush_period_sec:
            self._events_handle.flush()
            self._last_flush_sec = now
        self._event_count += 1

    def _emit_summary(self) -> None:
        now = time.monotonic()
        elapsed = max(now - self._last_summary_sec, 1e-6)
        rate = self._event_count / elapsed
        self.get_logger().info(f'stack_ros_profiler captured {self._event_count} events ({rate:.2f} events/sec)')
        self._event_count = 0
        self._last_summary_sec = now

    def _on_velocity(self, drone_id: str, topic_name: str, event_type: str, msg: Twist) -> None:
        self._record_event(
            drone_id,
            topic_name,
            event_type,
            {
                'linear_x': msg.linear.x,
                'linear_y': msg.linear.y,
                'linear_z': msg.linear.z,
                'angular_x': msg.angular.x,
                'angular_y': msg.angular.y,
                'angular_z': msg.angular.z,
            },
        )

    def _on_opcom(self, drone_id: str, topic_name: str, event_type: str, msg: OpCom) -> None:
        enum_value = int(msg.op_com_enum)
        self._record_event(
            drone_id,
            topic_name,
            event_type,
            {
                'op_com_enum': enum_value,
                'op_com_name': OPCOM_NAMES.get(enum_value, 'unknown'),
            },
        )

    def _on_mc_state(self, drone_id: str, topic_name: str, event_type: str, msg: McState) -> None:
        enum_value = int(msg.mc_state_enum)
        self._record_event(
            drone_id,
            topic_name,
            event_type,
            {
                'mc_state_enum': enum_value,
                'mc_state_name': MC_STATE_NAMES.get(enum_value, str(enum_value)),
            },
        )

    def _on_hlc_state(self, drone_id: str, topic_name: str, event_type: str, msg: HlcState) -> None:
        enum_value = int(msg.hlc_state_enum)
        self._record_event(
            drone_id,
            topic_name,
            event_type,
            {
                'hlc_state_enum': enum_value,
                'hlc_state_name': HLC_STATE_NAMES.get(enum_value, str(enum_value)),
            },
        )

    def _on_telemetry(self, drone_id: str, topic_name: str, event_type: str, msg: HlcTelemetry) -> None:
        self._record_event(
            drone_id,
            topic_name,
            event_type,
            {
                'latitude': msg.latitude,
                'longitude': msg.longitude,
                'altitude': msg.altitude,
                'yaw': msg.yaw,
                'armed': bool(msg.armed),
                'battery_voltage': msg.battery_voltage,
                'gps_fix': int(msg.gps_fix),
                'gps_hdop': msg.gps_hdop,
                'mode': msg.mode,
                'state': msg.state,
            },
        )

    def _on_platform_error(self, drone_id: str, topic_name: str, event_type: str, msg: HlcPlatformError) -> None:
        self._record_event(
            drone_id,
            topic_name,
            event_type,
            {
                'error_text': msg.error_text,
            },
        )


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Record namespaced ROS profile events for the visual stack.')
    parser.add_argument('--profile-manifest', required=True, type=Path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init()
    node = StackRosProfiler(args.profile_manifest)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
