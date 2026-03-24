# Local Patch Overlay

`manifests/stack.repos` pins the upstream source bases for `alate`,
`ardupilot_gazebo`, `nemala_core`, and the ROS bridge packages.

Some required integration work is not available upstream yet, so
`./scripts/bootstrap.sh` applies the patch files in this directory after
`vcs import` completes:

- `alate-local.patch`: local runtime fixes kept on top of upstream `alate`
- `ardupilot_gazebo-visual.patch`: adds the chase camera, the front-bottom
  deployed camera, and the GUI-controllable gimbal mount used by the visual
  simulation path
- `ros_alate_interfaces.patch`: added bridge output message types
- `ros_alate.patch`: completed the ROS output bridge for Alate state, telemetry,
  and platform errors

This keeps the top-level repo reproducible without requiring git submodules or
organization-hosted forks during the startup phase.
