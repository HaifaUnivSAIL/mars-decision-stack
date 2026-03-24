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
- a GUI gimbal-control panel for roll, pitch, and yaw
- the same ROS and NeMALA runtime used by decision modules
- keyboard control through the `manual_runtime_test` package

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
- `s`: back
- `r`: up
- `f`: down
- `space`: stop
- `h`: print help
- `q`: quit

The vehicle is already created by the SITL runtime when the stack starts, so no
additional spawn implementation is required for the current simulation path.

## Run the Interactive Visual Test

```bash
./scripts/run_visual_manual_runtime_test.sh
```

The script:

1. starts the Gazebo-backed visual stack if it is not already running
2. waits for HLC and MC readiness
3. launches the keyboard teleoperation node in the development container
4. keeps the chase and deployed-camera views available in Gazebo while commands are issued
5. exposes `Camera Gimbal Controls` in Gazebo for interactive roll, pitch, and yaw adjustment

The deployed camera is mounted at the front-bottom of the vehicle. The gimbal
starts at a neutral roll, pitch, and yaw of `0, 0, 0`, and the GUI controls
command those three rotation axes directly.

## Validate the Visual Stack

```bash
./scripts/validate_visual_stack.sh
```

This validator checks:

- visual stack bring-up
- HLC and MC readiness
- availability of `/mars/visual/chase_camera` and `/mars/visual/deployed_camera`
- availability of the native Gazebo gimbal position topics for roll, pitch, and yaw
- scripted teleoperation command publication against the visual runtime

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
