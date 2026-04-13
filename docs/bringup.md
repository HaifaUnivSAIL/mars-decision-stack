# Bringup

## 1. Populate source checkouts

```bash
./scripts/bootstrap.sh
```

This imports pinned repos from `manifests/stack.repos` and applies local patch files from `patches/` when needed.

## 2. Build the integrated image

```bash
./scripts/build_ws.sh
```

The Docker build compiles:

1. `external/nemala_core`
2. `external/alate`
3. ROS workspace packages under `ws/`

## 3. Run the stack

```bash
./scripts/run_stack.sh
```

This starts:

- ArduCopter SITL
- NeMALA proxy
- NeMALA logger
- Alate `mc`
- Alate `hlc`
- `ros_alate`
- `ros_nemala`
- `decision-dev`

## 4. Validate the runtime

```bash
./scripts/validate.sh
```

Validation checks container health, HLC/MC state transitions, ROS graph visibility, and a `decision_agent` publish smoke test.

This command validates the default runtime stack. It does not launch the Gazebo
GUI.

## 5. Visual bring-up

To start the Gazebo-backed visual workflow with the chase-camera and deployed
camera windows in the default experiment layout:

```bash
./scripts/run_visual_stack.sh
```

Each visual run now creates a run directory under `logs/runs/` containing:

- the generated world and runtime model actually passed to Gazebo
- the requested camera deployment config and applied manifest
- per-service stdout/stderr logs
- deployment verification reports and runtime diagnostics
- `ros2 topic list -t` snapshots
- HLC/MC readiness evidence copied from the live logs

If a visual stack is already running, `run_visual_stack.sh` reports that,
stops the old stack, and starts a fresh one so updated camera deployment values
and runtime assets are always picked up.

In Phase 1, the default visual experiment workflow is fleet-based and reads
`config/swarm/visual.swarm.json`. That manifest controls:

- drone ids such as `drone_1` and `drone_2`
- spawn poses
- per-drone camera topics
- deterministic ports and runtime namespaces

To target a different fleet manifest:

```bash
./scripts/run_visual_stack.sh --fleet config/swarm/visual.swarm.json
```

To start the calibration layout with the 6-DOF camera tuning controls:

```bash
./scripts/run_visual_stack.sh --calib
```

To validate the visual workflow:

```bash
./scripts/validate_visual_stack.sh
```

To validate the calibration layout:

```bash
./scripts/validate_visual_stack.sh --calib
```

The visual validator now checks the live deployed-camera pose against
`config/visual/camera.deployment.json` and verifies the run artifacts under the
current run directory.

To run the keyboard-controlled visual runtime test:

```bash
./scripts/run_visual_teleop.sh
```

This command also refreshes the visual stack first so the latest
`config/visual/camera.deployment.json` values are guaranteed to be active when
teleoperation starts.

To run the keyboard-controlled visual runtime test in calibration mode:

```bash
./scripts/run_visual_teleop.sh --calib
```

To control one fleet member explicitly:

```bash
./scripts/run_visual_teleop.sh --drone drone_1
./scripts/run_visual_teleop.sh --drone drone_2
```

To switch the active camera views in the running GUI without restarting:

```bash
./scripts/set_visual_focus.sh --drone drone_2
```
