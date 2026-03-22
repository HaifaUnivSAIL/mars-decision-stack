# Integration Architecture

## Layering

```text
Decision Agent
    |
    v
ROS 2 bridge layer (ros_alate, ros_nemala)
    |
    v
NeMALA proxy / dispatcher
    |
    v
Alate runtime (mc, hlc, behaviors)
```

## Repository roles

- `external/alate`: runtime source, kept as an external checkout with local compatibility patches
- `external/nemala_core`: NeMALA core source used for the container build
- `ws/src/ros_alate*`: ROS 2 bridge packages
- `ws/src/ros_nemala*`: ROS 2 runtime-management packages
- `ws/src/decision_agent`: local algorithm package; no direct dependency on Alate internals

## Grounded deviations from the supervisor note

- `vcstool` plus `patches/` is the reproducibility mechanism. This replaces submodules and also captures the local bridge/runtime delta that is not yet available as organization-hosted forks.
- The stack keeps shell scripts and Docker orchestration outside ROS launch files. ROS launch is used only for ROS nodes.
- The bridge contract is completed locally. Upstream `ros_alate` only implements command ingress; this repo adds the missing ROS egress for mission-control state, HLC state, telemetry, and platform-error strings.
- The current stable SITL mode uses direct `ArduCopter` execution instead of detached `sim_vehicle.py` plus `MAVProxy`.
