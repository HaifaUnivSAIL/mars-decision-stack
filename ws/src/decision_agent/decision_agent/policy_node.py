import rclpy
from rclpy.node import Node

from .policies.heuristic import compute_velocity_command
from .ros_io import AlateRosIo
from .world_model import WorldModel


class PolicyNode(Node):
    def __init__(self):
        super().__init__('decision_agent')
        self.declare_parameter('policy_type', 'heuristic')
        self.declare_parameter('control_frequency_hz', 5.0)
        self.declare_parameter('telemetry_timeout_sec', 2.0)
        self.declare_parameter('require_ready_state', True)
        self.declare_parameter('constant_linear_x', 0.0)
        self.declare_parameter('constant_angular_z', 0.0)
        self.declare_parameter('mission_name', 'sitl_baseline')
        self.declare_parameter('frame_id', 'map')

        self._world_model = WorldModel()
        self._io = AlateRosIo(self, self._world_model)

        control_frequency_hz = self.get_parameter('control_frequency_hz').value
        self._telemetry_timeout_sec = float(self.get_parameter('telemetry_timeout_sec').value)
        self._require_ready_state = bool(self.get_parameter('require_ready_state').value)
        self._constant_linear_x = float(self.get_parameter('constant_linear_x').value)
        self._constant_angular_z = float(self.get_parameter('constant_angular_z').value)
        self._policy_type = str(self.get_parameter('policy_type').value)

        period_sec = 1.0 / max(control_frequency_hz, 0.1)
        self.create_timer(period_sec, self._tick)
        self.get_logger().info('decision_agent started')

    def _tick(self) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        telemetry_fresh = self._world_model.telemetry_is_fresh(now_sec, self._telemetry_timeout_sec)

        if self._policy_type != 'heuristic':
            linear_x, angular_z = 0.0, 0.0
        else:
            linear_x, angular_z = compute_velocity_command(
                self._world_model,
                telemetry_fresh,
                self._require_ready_state,
                self._constant_linear_x,
                self._constant_angular_z,
            )

        self._io.publish_velocity(linear_x, angular_z)


def main(args=None):
    rclpy.init(args=args)
    node = PolicyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
