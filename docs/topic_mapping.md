# Topic Mapping

## NeMALA / Alate topic IDs

| ID | Alate topic | ROS topic | ROS type |
| --- | --- | --- | --- |
| 101 | `operator_command` | `/alate_input_operator_command` | `ros_alate_interfaces/msg/OpCom` |
| 102 | `mcm_state` | `/alate_output_mission_control_state` | `ros_alate_interfaces/msg/McState` |
| 103 | `hlc_state` | `/alate_output_high_level_control_state` | `ros_alate_interfaces/msg/HlcState` |
| 104 | `velocity_command` | `/alate_input_velocity` | `geometry_msgs/msg/Twist` |
| 105 | `hlc_telemetry` | `/alate_output_high_level_control_telemetry` | `ros_alate_interfaces/msg/HlcTelemetry` |
| 106 | `hlc_error` | `/alate_output_high_level_control_platform_errors` | `ros_alate_interfaces/msg/HlcPlatformError` |

## Decision-agent contract

`decision_agent` subscribes to:

- `/alate_output_mission_control_state`
- `/alate_output_high_level_control_state`
- `/alate_output_high_level_control_telemetry`
- `/alate_output_high_level_control_platform_errors`

`decision_agent` publishes:

- `/alate_input_velocity`
- optional future `/alate_input_operator_command`
