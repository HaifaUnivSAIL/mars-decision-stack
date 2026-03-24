# Mars Decision Stack

Mars Decision Stack is a ROS 2 and NeMALA integration repository for research
and lab development of decision-making algorithms on top of the Alate runtime.
It provides a reproducible workspace that combines the vehicle runtime, ROS
bridge packages, a local decision-agent package, and a Docker-based startup
stack for simulation-first experimentation and controlled transition to
platform-specific deployment.

## What This Repository Provides

- pinned source dependencies for `alate`, `nemala_core`, and ROS bridge packages
- a reproducible Docker build for the integrated runtime and ROS workspace
- a startup stack for single-UAV SITL bring-up with Alate mission control and
  high-level control
- a Gazebo-based visual simulation path with chase and deployed-camera views
  for operator-in-the-loop runtime testing
- ROS 2 bridge topics for command ingress and runtime state, telemetry, and
  platform-error egress
- a baseline `decision_agent` package for iterative autonomy and policy research

## Architecture

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

The repository separates external runtime code from local autonomy research
code:

- `external/`: pinned non-ROS source dependencies
- `ws/src/`: ROS workspace packages, including the local decision package
- `config/`: runtime and algorithm configuration
- `docker/`: reproducible build and stack definitions
- `scripts/`: bootstrap, build, bring-up, validation, and development helpers
- `patches/`: local overlays applied on top of pinned upstream dependencies

## Repository Layout

- `manifests/stack.repos`: authoritative dependency versions
- `config/alate/`: Alate and autopilot runtime configuration
- `config/ros_alate/`: ROS-to-NeMALA bridge configuration
- `config/ros_nemala/`: NeMALA ROS management configuration
- `config/decision_agent/`: runtime policy and mission parameters
- `ws/src/decision_agent/`: local decision-making package
- `docs/`: architecture, topic mapping, and bring-up notes

## Supported Workflow

The current repository version is optimized for simulation-first research and
integration validation:

- single-UAV bring-up
- ArduCopter SITL
- Gazebo visual simulation with keyboard-driven operator testing
- Alate mission control and high-level control
- ROS 2 Humble
- decision-agent development inside a Docker-based workspace
- reproducible experimental iteration across configuration profiles

## Quick Start

### 1. Bootstrap dependencies

```bash
./scripts/bootstrap.sh
```

### 2. Build the integrated image

```bash
./scripts/build_ws.sh
```

### 3. Start the stack

```bash
./scripts/run_stack.sh
```

### 4. Validate the runtime

```bash
./scripts/validate.sh
```

### 5. Open a development shell

```bash
./scripts/dev_shell.sh
```

### 6. Stop the stack

```bash
./scripts/stop_stack.sh
```

## Visual Simulation Workflow

The repository also provides a GUI-backed visual runtime for operator-in-the-loop
testing. This path launches the Gazebo-backed SITL stack, opens a desktop GUI,
and exposes two rendered views:

- `Chase Camera`: a trailing view that keeps the vehicle in frame
- `Deployed Camera`: a rendered image from a front-bottom camera mounted on the vehicle

The visual GUI also exposes `Camera Gimbal Controls`, which drive the deployed
camera roll, pitch, and yaw from a neutral `0, 0, 0` orientation.

### Start the visual stack

```bash
./scripts/run_visual_stack.sh
```

### Validate the visual stack

```bash
./scripts/validate_visual_stack.sh
```

### Run keyboard control against the visual stack

```bash
./scripts/run_visual_manual_runtime_test.sh
```

This workflow uses the same ROS command interface as local decision modules, so
manual testing and algorithm development stay aligned at the interface boundary.

## ROS Interface Contract

The current bridge exposes the operational interface used by local research code
and runtime integration:

### Inputs to Alate

- `/alate_input_velocity` (`geometry_msgs/msg/Twist`)
- `/alate_input_operator_command` (`ros_alate_interfaces/msg/OpCom`)

### Outputs from Alate

- `/alate_output_mission_control_state` (`ros_alate_interfaces/msg/McState`)
- `/alate_output_high_level_control_state` (`ros_alate_interfaces/msg/HlcState`)
- `/alate_output_high_level_control_telemetry` (`ros_alate_interfaces/msg/HlcTelemetry`)
- `/alate_output_high_level_control_platform_errors` (`ros_alate_interfaces/msg/HlcPlatformError`)

`decision_agent` is expected to consume the output topics as its experimental
observation interface and publish commands through the input topics.

## Developing a Decision-Making Algorithm

The local algorithm package lives in `ws/src/decision_agent` and is
intentionally kept independent from `external/alate`. New decision logic should
depend on ROS messages and configuration only. This keeps policy development
modular and makes it possible to compare alternative controllers without
coupling them to Alate internals.

### Recommended development flow

1. Start from the existing package structure:
   - `policy_node.py`: node entry point and control loop
   - `ros_io.py`: ROS subscriptions and command publishers
   - `world_model.py`: internal state representation
   - `policies/`: policy implementations

2. Add or modify a policy implementation under `ws/src/decision_agent/decision_agent/policies/`.

3. Wire the policy into `policy_node.py` if it needs new runtime behavior.

4. Expose tunable parameters through:
   - `config/decision_agent/policy.yaml`
   - `config/decision_agent/mission.yaml`

5. Run the stack and test the policy against SITL before transferring the
   experiment to a hardware-specific profile.

The baseline policy is deliberately safe: it falls back to zero command output
when telemetry is stale or the runtime is not in a ready state. It should be
treated as a reference controller and extension point rather than a final
algorithm.

## Developing for a Specific Hardware Platform

To target a specific vehicle or hardware profile, keep the algorithm package
stable and adapt the repository through configuration first. In practice, this
means separating platform assumptions from the policy logic so that experiments
can be repeated in simulation, compared across profiles, and then moved to the
intended hardware with minimal code churn.

### 1. Create a hardware-specific Alate runtime profile

Add a new file under `config/alate/`, for example:

```text
config/alate/<platform>.json
```

Use it to define:

- node IDs
- topic IDs
- proxy endpoints
- autopilot connection details
- deployment working directory and runtime behavior

The most important hardware-specific field is the autopilot connection section,
including the transport endpoint in `autopilot.master`.

### 2. Keep the ROS bridge aligned with the runtime profile

If topic IDs or proxy endpoints differ from the default profile, create a
matching ROS bridge configuration under `config/ros_alate/` and update:

- topic IDs
- dispatcher node ID
- publisher/subscriber proxy endpoints

The ROS bridge configuration must remain consistent with the Alate runtime
configuration.

### 3. Create hardware-specific algorithm parameters

Add dedicated policy and mission configuration files under
`config/decision_agent/`, for example:

```text
config/decision_agent/<platform>-policy.yaml
config/decision_agent/<platform>-mission.yaml
```

Use them for:

- control frequency
- safety limits
- mission identifiers
- frame conventions
- hardware-specific thresholds or behaviors

### 4. Develop and validate in simulation first

Use the SITL startup stack to validate the algorithm against the same ROS topic
contract used by the runtime.

This keeps algorithm work independent from flight hardware while allowing the
same decision package to move forward into a lab or field validation cycle.

### 5. Transition to the real platform

For hardware experiments, reuse the same decision package and ROS interfaces,
then swap the runtime and bridge configuration to the hardware-specific profile.
If the platform requires additional telemetry, commands, or error semantics,
extend `ros_alate_interfaces` and `ros_alate` rather than coupling the decision
algorithm directly to Alate internals. This preserves experimental comparability
across simulators, testbeds, and deployed platforms.

## Running the Decision Agent

Inside the development shell, the package can be launched with:

```bash
source /opt/ros/humble/setup.bash
source /opt/ros2_ws/install/setup.bash
ros2 launch decision_agent decision_agent.launch.py \
  policy_config:=/workspace/config/decision_agent/policy.yaml \
  mission_config:=/workspace/config/decision_agent/mission.yaml
```

## Validation

The repository includes an integrated validation script for routine lab bring-up
checks:

```bash
./scripts/validate.sh
```

It checks:

- container bring-up
- Alate state-machine readiness
- ROS topic visibility and typing
- telemetry availability
- `decision_agent` tests and command publication smoke checks

## Additional Documentation

- [`docs/manual_runtime_test.md`](docs/manual_runtime_test.md)
- [`docs/stack_modularity.md`](docs/stack_modularity.md)
- [`docs/integration_architecture.md`](docs/integration_architecture.md)
- [`docs/topic_mapping.md`](docs/topic_mapping.md)
- [`docs/bringup.md`](docs/bringup.md)
