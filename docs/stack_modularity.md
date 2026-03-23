# Stack Architecture and Modularity

This document describes the current stack architecture, the role of each
module, and the correct integration point for future robot-specific development
modules.

## Architecture Diagram

```mermaid
flowchart TD
    subgraph DEV["Decision / Research Layer"]
        ref_agent["Reference Decision Agent\nws/src/decision_agent\n\nFunctionality:\n- world model\n- policy loop\n- safety gating\n\nPurpose:\n- baseline controller\n- reference implementation"]
        robot_agent["Robot-Specific Development Module\nexample: ~/projects/Lite3\nor ws/src/robot_decision_agent\n\nFunctionality:\n- robot-specific policy\n- mission logic\n- experiment code\n\nPurpose:\n- platform-specific research\n- replaceable development layer"]
    end

    subgraph ROS["ROS 2 Interface Layer"]
        ros_alate["ros_alate\n\nFunctionality:\n- publish Alate state/telemetry into ROS\n- forward ROS commands into NeMALA\n\nPurpose:\n- integration boundary between research code and runtime"]
        ros_alate_if["ros_alate_interfaces\n\nFunctionality:\n- ROS message definitions\n\nPurpose:\n- stable contract for decision modules"]
        ros_nemala["ros_nemala\n\nFunctionality:\n- node manager / dispatcher control\n\nPurpose:\n- runtime management through ROS"]
        ros_nemala_if["ros_nemala_interfaces\n\nFunctionality:\n- ROS management messages\n\nPurpose:\n- stable contract for runtime control"]
    end

    subgraph NEMALA["NeMALA Communication Layer"]
        proxy["NeMALA Proxy / Dispatcher\n\nFunctionality:\n- topic routing\n- pub/sub transport\n\nPurpose:\n- decouple producers and consumers"]
        logger["NeMALA Logger\n\nFunctionality:\n- runtime log capture\n\nPurpose:\n- observability and experiment tracing"]
        core["nemala_core\n\nFunctionality:\n- core transport library\n\nPurpose:\n- communication substrate used by runtime and bridge"]
    end

    subgraph RUNTIME["Alate Runtime Layer"]
        mc["Mission Control (mc)\n\nFunctionality:\n- mission-level state machine\n\nPurpose:\n- supervisory runtime control"]
        hlc["High-Level Control (hlc)\n\nFunctionality:\n- command handling\n- telemetry publishing\n- autopilot integration\n\nPurpose:\n- vehicle control interface"]
        behaviors["Alate Behaviors / Modules\n\nFunctionality:\n- autonomy submodules\n- payload / behavior logic\n\nPurpose:\n- extend runtime capabilities"]
    end

    subgraph PLATFORM["Platform Layer"]
        autopilot["Autopilot / Robot Hardware / SITL\n\nFunctionality:\n- actuation\n- sensing\n- low-level state\n\nPurpose:\n- physical or simulated execution target"]
    end

    subgraph OPS["Configuration and Bring-Up"]
        cfg["config/\n\nFunctionality:\n- runtime profiles\n- bridge params\n- policy params\n\nPurpose:\n- switch between SITL and hardware profiles"]
        scripts["scripts/ + docker/\n\nFunctionality:\n- bootstrap\n- build\n- run\n- validate\n\nPurpose:\n- reproducible lab workflow"]
    end

    ref_agent -->|"ROS topics"| ros_alate
    robot_agent -->|"ROS topics\nrecommended integration point"| ros_alate
    robot_agent -->|"optional runtime-management calls"| ros_nemala

    ros_alate_if -. "message contract" .- ros_alate
    ros_nemala_if -. "message contract" .- ros_nemala

    ros_alate -->|"NeMALA topics"| proxy
    ros_nemala -->|"NeMALA topics"| proxy
    proxy --> mc
    proxy --> hlc
    proxy --> behaviors
    logger --> proxy
    core -. "transport library" .- proxy
    hlc --> autopilot

    cfg --> scripts
    cfg --> ros_alate
    cfg --> ros_nemala
    cfg --> mc
    cfg --> hlc
    scripts --> proxy
    scripts --> logger
    scripts --> mc
    scripts --> hlc
    scripts --> ros_alate
    scripts --> ros_nemala

    robot_agent -. "avoid direct dependency" .- mc
    robot_agent -. "avoid direct dependency" .- hlc
```

## Module Breakdown

| Module | Location | Functionality | Purpose in the system |
| --- | --- | --- | --- |
| Reference Decision Agent | `ws/src/decision_agent` | Maintains a world model, subscribes to ROS outputs, and publishes commands | Serves as the baseline decision layer and a reference for new controllers |
| Robot-Specific Development Module | Example: `~/projects/Lite3` or `ws/src/robot_decision_agent` | Implements robot-specific policy logic, state estimation assumptions, mission logic, and experiments | Main place for future development without coupling research code to Alate internals |
| `ros_alate` | `ws/src/ros_alate` | Bridges ROS command topics to NeMALA and publishes Alate state, telemetry, and errors into ROS | Defines the operational integration boundary between decision code and runtime |
| `ros_alate_interfaces` | `ws/src/ros_alate_interfaces` | Defines ROS messages for commands, state, telemetry, and errors | Creates a stable interface contract for all decision modules |
| `ros_nemala` | `ws/src/ros_nemala` | Exposes dispatcher and node-management functionality through ROS | Supports runtime lifecycle and management flows |
| `ros_nemala_interfaces` | `ws/src/ros_nemala_interfaces` | Defines ROS messages for runtime-management operations | Keeps management interactions explicit and versioned |
| NeMALA Proxy / Dispatcher | Runtime service | Routes messages between runtime modules and bridge nodes | Decouples message producers and consumers and preserves modularity |
| NeMALA Logger | Runtime service | Captures logs from the NeMALA communication layer | Supports observability, debugging, and experiment traceability |
| `nemala_core` | `external/nemala_core` | Provides the communication substrate used by NeMALA-based components | Shared low-level transport foundation |
| Mission Control (`mc`) | `external/alate` build output | Runs mission-level state transitions and supervisory logic | Coordinates vehicle-level runtime behavior |
| High-Level Control (`hlc`) | `external/alate` build output | Handles command execution, autopilot integration, and telemetry publication | Connects runtime decisions to the platform |
| Behaviors / Alate Modules | `external/alate` | Optional autonomy, payload, or behavior extensions | Extends runtime capabilities below the decision layer |
| Autopilot / Hardware / SITL | External platform | Executes low-level motion and exposes platform state | Final execution target, either simulated or physical |
| Configuration Profiles | `config/` | Stores runtime, bridge, and algorithm parameters | Allows the same architecture to move between SITL and hardware-specific setups |
| Bring-Up and Validation Tooling | `scripts/`, `docker/` | Builds, starts, validates, and stops the stack | Makes experiments reproducible across developers and platforms |

## Where the New Development Module Belongs

The module you will implement for a specific platform, such as a robot-specific
package in `~/projects/Lite3`, belongs in the **Decision / Research Layer**.

Its dependency boundary should be:

- consume ROS state, telemetry, and error topics
- publish ROS command topics
- remain independent from `external/alate` internal classes
- avoid direct use of NeMALA transport internals unless you are explicitly
  extending the bridge layer

In other words, the correct dependency path is:

```text
Robot-Specific Development Module
    -> ROS 2 topics and messages
    -> ros_alate / ros_nemala
    -> NeMALA proxy
    -> Alate runtime
    -> hardware or SITL
```

The module should **not** depend directly on:

- `mc` internal implementation details
- `hlc` internal implementation details
- raw NeMALA transport code for normal policy logic

## Practical Integration Guidance for a Robot-Specific Module

For a development package such as `~/projects/Lite3`, the recommended approach
is:

1. Keep the module in its own ROS package or repository if it represents a
   distinct robot program.
2. Subscribe to the ROS interface contract exposed by this repository.
3. Publish velocity or operator-command topics back through the same ROS
   interface layer.
4. Add hardware-specific parameters through configuration, not through direct
   edits to Alate runtime code.
5. Extend the bridge packages only when the platform truly requires new command
   types, telemetry fields, or error semantics.

This preserves the main modularity goal of the stack: decision logic remains
replaceable, while the integration and runtime layers remain stable.
