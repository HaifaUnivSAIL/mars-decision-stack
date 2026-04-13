# Topic Mapping

## NeMALA / Alate topic IDs

| ID | Alate topic | ROS topic | ROS type |
| --- | --- | --- | --- |
| 101 | `operator_command` | `/<drone_id>/alate_input_operator_command` | `ros_alate_interfaces/msg/OpCom` |
| 102 | `mcm_state` | `/<drone_id>/alate_output_mission_control_state` | `ros_alate_interfaces/msg/McState` |
| 103 | `hlc_state` | `/<drone_id>/alate_output_high_level_control_state` | `ros_alate_interfaces/msg/HlcState` |
| 104 | `velocity_command` | `/<drone_id>/alate_input_velocity` | `geometry_msgs/msg/Twist` |
| 105 | `hlc_telemetry` | `/<drone_id>/alate_output_high_level_control_telemetry` | `ros_alate_interfaces/msg/HlcTelemetry` |
| 106 | `hlc_error` | `/<drone_id>/alate_output_high_level_control_platform_errors` | `ros_alate_interfaces/msg/HlcPlatformError` |

## Decision-agent contract

`decision_agent` subscribes to:

- `/<drone_id>/alate_output_mission_control_state`
- `/<drone_id>/alate_output_high_level_control_state`
- `/<drone_id>/alate_output_high_level_control_telemetry`
- `/<drone_id>/alate_output_high_level_control_platform_errors`

`decision_agent` publishes:

- `/<drone_id>/alate_input_velocity`
- optional future `/<drone_id>/alate_input_operator_command`

## Visual topics

Per drone:

- `/mars/<drone_id>/visual/chase_camera`
- `/mars/<drone_id>/visual/deployed_camera`

Active GUI focus:

- `/mars/visual/active/chase_camera`
- `/mars/visual/active/deployed_camera`
- selector: `/mars/visual/active_drone/select`
