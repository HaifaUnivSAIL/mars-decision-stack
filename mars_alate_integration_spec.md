# MARS-Technion + NeMALA/Alate Integration Specification

## 1. Purpose

This document proposes a practical integration architecture that connects:

- **MARS-Technion ROS 2 repositories**
- **NeMALA / Alate runtime repositories**

into a **single working development stack** for future **decision-making algorithms**.

The main goal is **not** to merge all projects into one codebase. Instead, the goal is to create a **stable super-repo / ROS 2 workspace** in which:

- NeMALA and Alate provide the autonomy runtime and messaging substrate
- MARS-Technion ROS 2 packages provide the bridge into ROS 2
- your own decision-making code lives in a clean, separate package on top

This structure is intended to support future work such as:

- heuristic decision logic
- finite state machines
- behavior trees
- optimization / MPC-based controllers
- reinforcement learning policies
- multi-agent coordination modules

---

## 2. High-Level Integration Idea

The cleanest integration model is:

- **NeMALA core/tools** = communication / dispatcher / proxy / node tooling
- **Alate** = autonomy runtime and application framework
- **ros_alate** = ROS 2 bridge to Alate topics and control flow
- **ros_nemala** = ROS 2 bridge / tooling around NeMALA services
- **decision_agent** = your future algorithms package

### Target layering

```text
Decision Algorithm Layer (your code)
    |
    v
ROS 2 interface layer (ros_alate, ros_nemala)
    |
    v
NeMALA proxy / dispatcher
    |
    v
Alate runtime / autonomy modules
```

This keeps the stack modular and prevents your future policy code from depending directly on Alate internal implementation details.

---

## 3. Design Principles

### 3.1 Separation of concerns

Your own algorithms should depend on **ROS topics and messages**, not on internal NeMALA/Alate implementation details.

### 3.2 Minimal invasive integration

Avoid rewriting Alate into native ROS 2.
Use the existing bridge packages instead.

### 3.3 Research-friendly structure

The stack should let you replace the decision layer easily:

- simple scripted controller today
- FSM tomorrow
- RL or MPC later

### 3.4 Reproducibility

The integrated repo should support:

- deterministic setup
- containerization
- workspace build scripts
- version-controlled config files

---

## 4. Proposed Super-Repo

Recommended repo name:

```text
mars_decision_stack
```

### Proposed directory structure

```text
mars_decision_stack/
├── README.md
├── .gitmodules
├── docker/
│   ├── Dockerfile
│   ├── docker-compose.yml
│   └── entrypoint.sh
├── docs/
│   ├── integration_architecture.md
│   ├── topic_mapping.md
│   └── bringup.md
├── scripts/
│   ├── bootstrap.sh
│   ├── build_ws.sh
│   ├── run_proxy.sh
│   ├── run_stack.sh
│   └── env.sh
├── config/
│   ├── alate/
│   │   └── uav.json
│   ├── ros_alate/
│   │   └── adapter.yaml
│   ├── ros_nemala/
│   │   └── node_manager.yaml
│   └── decision_agent/
│       ├── policy.yaml
│       └── mission.yaml
├── external/
│   ├── nemala_core/
│   ├── nemala_tools/
│   ├── alate/
│   ├── ros_alate/
│   ├── ros_alate_interfaces/
│   ├── ros_nemala/
│   └── ros_nemala_interfaces/
└── ws/
    ├── src/
    │   ├── ros_alate -> ../../external/ros_alate
    │   ├── ros_alate_interfaces -> ../../external/ros_alate_interfaces
    │   ├── ros_nemala -> ../../external/ros_nemala
    │   ├── ros_nemala_interfaces -> ../../external/ros_nemala_interfaces
    │   └── decision_agent/
    │       ├── package.xml
    │       ├── setup.py
    │       ├── resource/
    │       ├── launch/
    │       │   └── stack.launch.py
    │       ├── config/
    │       │   ├── policy.yaml
    │       │   └── mission.yaml
    │       ├── decision_agent/
    │       │   ├── __init__.py
    │       │   ├── policy_node.py
    │       │   ├── world_model.py
    │       │   ├── alate_ros_io.py
    │       │   └── policies/
    │       │       ├── __init__.py
    │       │       ├── heuristic.py
    │       │       ├── fsm.py
    │       │       ├── mpc.py
    │       │       └── rl_policy.py
    │       └── test/
    │           ├── test_policy_import.py
    │           └── test_launch.py
    ├── build/
    ├── install/
    └── log/
```

---

## 5. Repository Roles

### 5.1 `external/nemala_core`

Holds the NeMALA communication core.
This should remain external and unmodified unless absolutely necessary.

### 5.2 `external/nemala_tools`

Contains runtime utilities such as:

- proxy execution
- logging / replay utilities
- node tooling

This is required for bringing up the dispatcher/proxy side of the stack.

### 5.3 `external/alate`

Contains the Alate autonomy runtime.
Treat this as the application runtime side of the system.

### 5.4 `external/ros_alate`

ROS 2 adapter between Alate and ROS topics.
This is the key bridge that your decision package will use.

### 5.5 `external/ros_alate_interfaces`

ROS message and service definitions required by `ros_alate`.

### 5.6 `external/ros_nemala`

ROS 2-facing tooling around NeMALA runtime management.
Useful for node management and system orchestration.

### 5.7 `external/ros_nemala_interfaces`

ROS interface package used by `ros_nemala`.

### 5.8 `ws/src/decision_agent`

This is your package.
It should contain:

- policy logic
- world model construction
- topic IO abstraction
- mission-level decisions
- future RL/MPC/FSM modules

This package is the correct place for experimentation and future algorithms.

---

## 6. Why This Architecture Is Preferred

### 6.1 Keeps Alate stable

You avoid making your experimental code part of Alate internals.
That makes upgrades and debugging easier.

### 6.2 Keeps algorithm work modular

The decision layer can be versioned and benchmarked independently.

### 6.3 Supports future replacement of components

If later you want:

- a simulator-only stack
- a hardware-specific bridge
- a custom ROS-native autonomy backend

you can replace only selected layers.

### 6.4 Encourages clean interfaces

A ROS-based contract between runtime and policy is much healthier than importing internal runtime modules directly.

---

## 7. Integration Contract

The most important idea is the **contract** between your algorithm and the bridge layer.

### 7.1 Decision package should consume

- telemetry
- mission state
- high-level control state
- error signals
- optional operator commands / overrides

### 7.2 Decision package should produce

- velocity commands
- operator-level commands
- future mission requests
- optional debug/diagnostic topics

### 7.3 Decision package should not depend on

- NeMALA dispatcher implementation details
- Alate internal class layout
- hardcoded proxy internals
- custom build assumptions inside external repos

---

## 8. Topic-Level Conceptual Flow

A conceptual runtime flow:

```text
[Alate Runtime / Platform Modules]
           ^
           |
           v
[NeMALA Proxy / Dispatcher]
           ^
           |
           v
[ros_alate Adapter]
      ^             ^
      |             |
      |             +-- publishes state/telemetry/error topics
      |
      +-- receives control commands from decision_agent

[decision_agent]
    - subscribes to state and telemetry
    - updates world model
    - runs policy
    - publishes commands
```

### Conceptual ROS-facing inputs to `decision_agent`

```text
/alate_output_mission_control_state
/alate_output_high_level_control_state
/alate_output_high_level_control_telemetry
/alate_output_high_level_control_platform_errors
```

### Conceptual ROS-facing outputs from `decision_agent`

```text
/alate_input_velocity
/alate_input_operator_command
```

> Note: exact topic names and message types must be verified against the currently checked-out versions of the repositories during implementation.

---

## 9. Decision Agent Internal Structure

The decision package should be structured around a stable internal API.

### Recommended internal decomposition

```text
Perception/Telemetry -> WorldState -> DecisionPolicy -> CommandPublisher
```

### File responsibilities

#### `world_model.py`
Responsible for transforming incoming ROS messages into a normalized internal state.

Example outputs:

- current mode
- platform health flags
- mission phase
- pose / velocity summaries
- communication status
- error state

#### `alate_ros_io.py`
Responsible for all ROS publishers/subscribers and message conversion.

This should isolate the rest of your code from raw ROS message details.

#### `policy_node.py`
Main executable ROS node.
This should:

- initialize IO
- collect state
- call the selected policy
- publish commands on a timer or event basis

#### `policies/heuristic.py`
Contains a basic deterministic controller.
This should be the first working policy.

#### `policies/fsm.py`
Contains explicit mode-switching / state-machine logic.

#### `policies/mpc.py`
Placeholder for future optimization-based control.

#### `policies/rl_policy.py`
Placeholder for future learned decision policies.

---

## 10. First Milestone Definition

The first milestone should be a **minimal end-to-end working stack**.

### Milestone 1 goals

1. All external repos clone successfully
2. ROS 2 workspace builds
3. NeMALA proxy launches
4. `ros_alate` launches successfully
5. relevant ROS topics are visible
6. `decision_agent` launches successfully
7. `decision_agent` publishes a safe constant command
8. feedback topics are received and logged

### What not to do in Milestone 1

Do not start with:

- multi-agent logic
- RL training integration
- advanced planners
- deeply modifying Alate internals
- optimizing latency before basic functionality exists

---

## 11. Suggested Bringup Sequence

Recommended order of execution:

### Step 1: environment setup

- install ROS 2 distribution used by the MARS packages
- install build tools (`colcon`, `vcstool`, Python deps, compiler toolchain)
- ensure container or host dependencies are stable

### Step 2: fetch repositories

Use submodules or pinned clones.
Submodules are recommended.

### Step 3: prepare workspace symlinks or vendor copies

Only the ROS packages should appear under `ws/src`.
The NeMALA / Alate repos can remain under `external/` and be used through scripts/config.

### Step 4: build the workspace

- source ROS
- run `colcon build`
- source workspace install

### Step 5: start proxy

Launch the NeMALA proxy with the matching configuration.

### Step 6: start bridge nodes

Launch:

- `ros_alate` adapter
- `ros_nemala` tooling if needed

### Step 7: start decision node

Run the `decision_agent` package.

### Step 8: inspect system

Verify:

- topic list
- topic echo
- command publication
- log flow
- health / error topics

---

## 12. Submodule Bootstrap Example

Example commands:

```bash
mkdir mars_decision_stack && cd mars_decision_stack
git init

mkdir -p external ws/src docs scripts config docker

git submodule add https://gitlab.com/nemala/core.git external/nemala_core
git submodule add https://gitlab.com/nemala/tools.git external/nemala_tools
git submodule add https://gitlab.com/nemala/alate.git external/alate

git submodule add https://github.com/MARS-Technion/ros_alate.git external/ros_alate
git submodule add https://github.com/MARS-Technion/ros_alate_interfaces.git external/ros_alate_interfaces
git submodule add https://github.com/MARS-Technion/ros_nemala.git external/ros_nemala
git submodule add https://github.com/MARS-Technion/ros_nemala_interfaces.git external/ros_nemala_interfaces
```

### Symlink ROS packages into workspace

```bash
cd ws/src
ln -s ../../external/ros_alate .
ln -s ../../external/ros_alate_interfaces .
ln -s ../../external/ros_nemala .
ln -s ../../external/ros_nemala_interfaces .
```

---

## 13. Decision Package Skeleton

### Package creation

```bash
cd ws/src
ros2 pkg create --build-type ament_python decision_agent
```

### Minimal first node sketch

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class PolicyNode(Node):
    def __init__(self):
        super().__init__("policy_node")
        self.cmd_pub = self.create_publisher(Twist, "alate_input_velocity", 10)
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PolicyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

> The message type above is only a placeholder example. Actual types should match the currently exposed `ros_alate_interfaces` definitions.

---

## 14. Recommended Scripts

### `scripts/bootstrap.sh`
Responsibilities:

- initialize submodules
- install Python requirements
- install system dependencies
- set up environment variables

### `scripts/build_ws.sh`
Responsibilities:

- source ROS
- build `ws/` with `colcon`
- source the local install space

### `scripts/run_proxy.sh`
Responsibilities:

- launch NeMALA proxy with correct config
- mount runtime config files if using Docker

### `scripts/run_stack.sh`
Responsibilities:

- optionally start proxy
- start bridge nodes
- start decision node
- optionally start RViz / debug tools

### `scripts/env.sh`
Responsibilities:

- export shared environment vars
- centralize config paths
- define ROS domain / namespace settings if relevant

---

## 15. Recommended Config Layout

### `config/alate/uav.json`
Holds NeMALA/Alate-side configuration required by the proxy/runtime.

### `config/ros_alate/adapter.yaml`
Holds adapter configuration such as:

- topic IDs
- node IDs
- proxy address / connection fields
- runtime topic mappings

### `config/ros_nemala/node_manager.yaml`
Holds node manager / orchestration configuration.

### `config/decision_agent/policy.yaml`
Holds algorithm-level settings, for example:

- policy type
- control frequency
- safety bounds
- timeout values
- debug flags

### `config/decision_agent/mission.yaml`
Holds mission-specific behavior parameters.

---

## 16. Docker Recommendation

A container is strongly recommended for reproducibility.

### Container responsibilities

- install correct ROS 2 distribution
- install compiler and Python dependencies
- expose workspace path
- provide entrypoint for sourcing ROS and workspace
- optionally include NeMALA tooling dependencies

### Suggested Docker workflow

- build one base image for the integrated stack
- mount the repo into the container during development
- keep config files in the host repo
- use one script to bring up the proxy and one to build/run ROS nodes

---

## 17. Integration Risks

### 17.1 Version mismatch risk

The biggest likely issue is mismatch between:

- ROS 2 distro expectations
- interface package versions
- current Alate / NeMALA config assumptions

### 17.2 Hidden runtime assumptions

Some bridge packages may assume specific:

- topic identifiers
- config file paths
- environment variables
- network / IPC paths

### 17.3 Interface drift

Public README-level information may not match the current code exactly.
Therefore, implementation must verify actual message types, launch commands, and parameter names from the checked-out repositories.

### 17.4 Over-coupling risk

If your decision code imports internal runtime classes directly, future maintenance will become much harder.
Avoid that.

---

## 18. Required Validation Tasks

Before calling the integrated repo “working”, validate the following.

### 18.1 Build validation

- all submodules checkout successfully
- workspace builds without manual hacks
- interface packages compile correctly

### 18.2 Runtime validation

- proxy starts successfully
- adapter starts successfully
- node manager starts successfully if required
- no immediate topic/config mismatch errors

### 18.3 ROS graph validation

- required topics appear in `ros2 topic list`
- messages can be echoed
- topic types match expectations

### 18.4 Control loop validation

- `decision_agent` can publish commands
- bridge receives them
- corresponding state feedback is visible

### 18.5 Fault validation

- observe system behavior when proxy is unavailable
- observe system behavior when topic/config IDs mismatch
- verify safe handling of stale telemetry

---

## 19. Recommended Development Roadmap

### Phase 1 — Bringup

Goal: make the stack launch and exchange messages.

Deliverables:

- repo skeleton
- submodule setup
- build scripts
- minimal decision node
- documented launch sequence

### Phase 2 — Clean interfaces

Goal: create a stable algorithm interface.

Deliverables:

- `world_model.py`
- IO abstraction layer
- config-driven policy loading
- better logging and diagnostics

### Phase 3 — Baseline policies

Goal: add useful control logic.

Deliverables:

- heuristic policy
- FSM policy
- safety wrappers
- replay/debug tools

### Phase 4 — Advanced decision algorithms

Goal: enable research experimentation.

Deliverables:

- MPC prototype
- RL integration hooks
- benchmark harness
- scenario evaluation framework

### Phase 5 — Multi-agent / mission-level extensions

Goal: support larger autonomy research workflows.

Deliverables:

- multi-agent state sharing
- mission planner integration
- coordination policies

---

## 20. What Should Be Implemented First

The first concrete implementation target should be:

1. create the super-repo
2. pin external repositories
3. build ROS workspace
4. launch proxy
5. launch `ros_alate`
6. create a minimal `decision_agent`
7. publish a constant safe command
8. confirm feedback loop

That will give you a **real working base repo** for future decision-making algorithms.

---

## 21. Strong Recommendation

Use this exact strategic rule throughout development:

> **All future decision logic should be developed in `decision_agent` and communicate through ROS interfaces only.**

This will preserve:

- maintainability
- modularity
- research flexibility
- future portability

---

## 22. Open Items To Verify During Implementation

The following details must be verified directly from the checked-out repositories before coding against them:

- exact supported ROS 2 distro
- exact package dependencies
- exact launch commands
- exact parameter file names and fields
- actual ROS topic names
- actual message/service definitions
- proxy startup commands
- runtime config format expectations

These are implementation-time checks, not blockers for the proposed architecture.

---

## 23. Final Summary

The best way to connect MARS-Technion and NeMALA/Alate into a usable research stack is to build a **new integration repo** with three clean layers:

1. **NeMALA + Alate runtime** in `external/`
2. **ROS 2 bridge packages** in `ws/src`
3. **Your own decision algorithms** in `ws/src/decision_agent`

This gives you a scalable foundation for future decision-making work without entangling experimental algorithms with autonomy runtime internals.

