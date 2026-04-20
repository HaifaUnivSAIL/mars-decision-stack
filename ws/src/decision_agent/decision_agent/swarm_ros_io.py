from __future__ import annotations

from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_alate_interfaces.msg import HlcPlatformError, HlcState, HlcTelemetry, McState, OpCom

from .world_model import TelemetryState


class SwarmRosIo:
    def __init__(self, node, world_model, manifest: dict):
        self._node = node
        self._world_model = world_model
        self._velocity_publishers = {}
        self._operator_publishers = {}

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        for drone in manifest.get("drones", []):
            drone_id = str(drone["id"])
            namespace = str(drone.get("namespace") or f"/{drone_id}").rstrip("/")
            self._world_model.ensure_drone(drone_id)
            self._velocity_publishers[drone_id] = node.create_publisher(Twist, f"{namespace}/alate_input_velocity", 10)
            self._operator_publishers[drone_id] = node.create_publisher(OpCom, f"{namespace}/alate_input_operator_command", 10)

            node.create_subscription(
                McState,
                f"{namespace}/alate_output_mission_control_state",
                lambda msg, drone_id=drone_id: self._world_model.update_mc_state(drone_id, int(msg.mc_state_enum)),
                state_qos,
            )
            node.create_subscription(
                HlcState,
                f"{namespace}/alate_output_high_level_control_state",
                lambda msg, drone_id=drone_id: self._world_model.update_hlc_state(drone_id, int(msg.hlc_state_enum)),
                state_qos,
            )
            node.create_subscription(
                HlcTelemetry,
                f"{namespace}/alate_output_high_level_control_telemetry",
                lambda msg, drone_id=drone_id: self._on_telemetry(drone_id, msg),
                10,
            )
            node.create_subscription(
                HlcPlatformError,
                f"{namespace}/alate_output_high_level_control_platform_errors",
                lambda msg, drone_id=drone_id: self._world_model.update_error(drone_id, msg.error_text),
                10,
            )

    def publish_twist(
        self,
        drone_id: str,
        *,
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
        self._velocity_publishers[drone_id].publish(msg)

    def publish_zero(self, drone_id: str) -> None:
        self.publish_twist(drone_id)

    def publish_zero_all(self, drone_ids: list[str]) -> None:
        for drone_id in drone_ids:
            self.publish_zero(drone_id)

    def publish_operator_command(self, drone_id: str, op_com_enum: int) -> None:
        msg = OpCom()
        msg.op_com_enum = int(op_com_enum)
        self._operator_publishers[drone_id].publish(msg)

    def publish_operator_command_all(self, drone_ids: list[str], op_com_enum: int) -> None:
        for drone_id in drone_ids:
            self.publish_operator_command(drone_id, op_com_enum)

    def _on_telemetry(self, drone_id: str, msg: HlcTelemetry) -> None:
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
        self._world_model.update_telemetry(drone_id, telemetry, now_sec)
