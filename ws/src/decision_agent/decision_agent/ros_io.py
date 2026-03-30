from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_alate_interfaces.msg import HlcPlatformError, HlcState, HlcTelemetry, McState, OpCom

from .world_model import TelemetryState


class AlateRosIo:
    def __init__(self, node, world_model):
        self._node = node
        self._world_model = world_model
        self._velocity_pub = node.create_publisher(Twist, '/alate_input_velocity', 10)
        self._opcom_pub = node.create_publisher(OpCom, '/alate_input_operator_command', 10)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        node.create_subscription(McState, '/alate_output_mission_control_state', self._on_mc_state, state_qos)
        node.create_subscription(HlcState, '/alate_output_high_level_control_state', self._on_hlc_state, state_qos)
        node.create_subscription(HlcTelemetry, '/alate_output_high_level_control_telemetry', self._on_telemetry, 10)
        node.create_subscription(HlcPlatformError, '/alate_output_high_level_control_platform_errors', self._on_error, 10)

    def publish_twist(
        self,
        linear_x: float = 0.0,
        linear_y: float = 0.0,
        linear_z: float = 0.0,
        angular_z: float = 0.0,
    ) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = linear_z
        msg.angular.z = angular_z
        self._velocity_pub.publish(msg)

    def publish_velocity(self, linear_x: float, angular_z: float = 0.0) -> None:
        self.publish_twist(linear_x=linear_x, angular_z=angular_z)

    def publish_zero(self) -> None:
        self.publish_twist()

    def publish_operator_command(self, op_com_enum: int) -> None:
        msg = OpCom()
        msg.op_com_enum = op_com_enum
        self._opcom_pub.publish(msg)

    def _on_mc_state(self, msg: McState) -> None:
        self._world_model.update_mc_state(int(msg.mc_state_enum))

    def _on_hlc_state(self, msg: HlcState) -> None:
        self._world_model.update_hlc_state(int(msg.hlc_state_enum))

    def _on_telemetry(self, msg: HlcTelemetry) -> None:
        telemetry = TelemetryState(
            latitude=msg.latitude,
            longitude=msg.longitude,
            altitude=msg.altitude,
            yaw=msg.yaw,
            armed=msg.armed,
            battery_voltage=msg.battery_voltage,
            gps_fix=msg.gps_fix,
            gps_hdop=msg.gps_hdop,
            mode=msg.mode,
            state=msg.state,
        )
        now_sec = self._node.get_clock().now().nanoseconds / 1e9
        self._world_model.update_telemetry(telemetry, now_sec)

    def _on_error(self, msg: HlcPlatformError) -> None:
        self._world_model.update_error(msg.error_text)
