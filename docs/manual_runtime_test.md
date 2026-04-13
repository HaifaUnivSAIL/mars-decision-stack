# Manual Runtime Test

The manual runtime test provides an operator-driven validation path for the full
stack without deploying a closed-loop decision policy.

## Purpose

This workflow is intended for:

- checking that the full stack starts correctly
- verifying that the simulated vehicle is available in SITL
- sending simple manual commands through the same ROS interface used by future
  policies
- validating that command direction matches the expected simulator response

## Available Modes

### Visual runtime test

The recommended workflow uses the Gazebo-backed visual stack:

- a chase-camera rendered view that keeps the vehicle in frame
- a front-bottom deployed camera rendered from the vehicle itself
- a fleet experiment workflow where each drone keeps its own ROS namespace and control slice
- a JSON-configured deployed-camera mount loaded from `config/visual/camera.deployment.json`
- the same ROS and NeMALA runtime used by decision modules
- keyboard control through the `manual_runtime_test` package

By default, the visual stack starts in `experiment` mode, where the two rendered
camera views dominate the GUI and the calibration controls are hidden.

Use `--calib` when you want the full 6-DOF camera-mount control panel back for
interactive tuning.

This mode requires a local X11 desktop session with `DISPLAY` available.

### Headless runtime test

The original headless path is still available for lightweight command-path
checks when a visual frontend is not required.

## What It Uses

- full stack bring-up from `scripts/run_stack.sh` or `scripts/run_visual_stack.sh`
- existing ROS bridge topics
- a dedicated package: `ws/src/manual_runtime_test`

The manual test does not depend directly on Alate internals. It publishes
through the same ROS interface boundary used by other decision modules.

## Controls

- `t`: takeoff
- `l`: land
- `g`: go home
- `w`: forward
- `a`: left
- `s`: back
- `d`: right
- `r`: up
- `f`: down
- `space`: stop
- `h`: print help
- `q`: quit

The vehicle is already created by the SITL runtime when the stack starts, so no
additional spawn implementation is required for the current simulation path.

## Run the Interactive Visual Test

```bash
./scripts/run_visual_teleop.sh
```

Target one drone explicitly:

```bash
./scripts/run_visual_teleop.sh --drone drone_1
./scripts/run_visual_teleop.sh --drone drone_2
```

Calibration mode:

```bash
./scripts/run_visual_teleop.sh --calib
```

`run_visual_teleop.sh` is the short operator-facing alias for
`run_visual_manual_runtime_test.sh`. It is the recommended command when you
want the GUI and keyboard control together.

The script:

1. refreshes the Gazebo-backed visual stack so the latest deployment/config values are active
2. launches or relaunches the Gazebo GUI client as part of that fresh visual bring-up
3. waits for HLC and MC readiness
4. launches the keyboard teleoperation node in the development container
5. keeps the chase and deployed-camera views available in Gazebo while commands are issued
6. uses the deployment stored in `config/visual/camera.deployment.json`
7. writes the full run record under `logs/runs/<timestamp>-<mode>/`

In fleet experiment mode, keyboard commands are sent into the selected drone
namespace:

- `/<drone_id>/alate_input_velocity`
- `/<drone_id>/alate_input_operator_command`

The default fleet manifest is `config/swarm/visual.swarm.json`.

The run record includes:

- generated world/model assets
- requested and applied camera deployment manifests
- per-service stdout/stderr logs
- `deployment.verify.json`
- `ros2.topics.txt`
- `hlc.readiness.log`
- `mc.readiness.log`

When `--calib` is used, Gazebo also exposes `Camera Mount 6-DOF Controls` for
interactive camera position and orientation adjustment.

The deployed camera is mounted at the front-bottom of the vehicle.

The controls are intentionally split into two stages:

- `x`, `y`, `z`: absolute translation of the rigid deployed-camera mount in the drone body frame, measured in meters
- `yaw`, `pitch`, `roll`: absolute orientation of the rigid deployed-camera mount, measured in radians

The same 6-DOF values drive both visual modes:

- in `experiment`, the deployment is baked into a generated rigid-camera model
- in `--calib`, Gazebo initializes the six helper joints to those same values so the GUI reads the real configured deployment instead of zero

This keeps calibration mode and experiment mode consistent, and avoids hidden
offsets between what you tune and what the default stack actually loads.

By default, motion commands latch until a new direction is requested or
`space` is pressed. This makes the visual test behave like a simple operator
controller instead of a short pulse generator.

If you stop the GUI window but leave the visual stack running, rerunning
`./scripts/run_visual_teleop.sh` will now relaunch the GUI client
before starting keyboard control.

## Validate the Visual Stack

```bash
./scripts/validate_visual_stack.sh
```

Calibration-mode validation:

```bash
./scripts/validate_visual_stack.sh --calib
```

This validator checks:

- visual stack bring-up
- HLC and MC readiness for every drone in the fleet manifest
- availability of `/mars/visual/active/chase_camera` and `/mars/visual/active/deployed_camera`
- per-drone takeoff / land smoke in the selected fleet manifest
- live focus switching between drones
- live deployed-camera pose verification against `config/visual/camera.deployment.json`
- scripted teleoperation command publication against the visual runtime
- presence of the run manifest, generated visual assets, and per-service runtime logs

In `--calib` mode, it also checks:

- availability of the native Gazebo camera-mount position topics for `x`, `y`, `z`, `yaw`, `pitch`, and `roll`
- convergence of the camera-mount joint state to the configured `x`, `y`, `z`, `yaw`, `pitch`, and `roll`

## Run the Headless Interactive Test

```bash
./scripts/run_manual_runtime_test.sh
```

Use this path when you only need command-path validation without Gazebo GUI
visualization.

## Run the Scripted Smoke Test

```bash
./scripts/validate_manual_runtime_test.sh
```

This launches the same teleop node in scripted mode and verifies that commands
reach the ROS bridge input topic on the headless runtime path.
