# Swarm Stack Analysis Report

## Scope

This report documents the current swarm-stack workflow and verifies the current behavior against the latest successful and failing runs.

Target state under analysis:

- Non-swarm single-drone stack works.
- Swarm stack with `--count 1` works.
- Swarm stack with `--count > 1` fails during readiness with a timeout.

This report is based on the current code in `main` and on these run artifacts:

- Successful fleet-single run: `logs/runs/20260420-105519-experiment`
- Failing fleet-multi runs: `logs/runs/20260415-173340-experiment`, `logs/runs/20260415-173847-experiment`
- Successful non-fleet single run: `logs/runs/20260415-165023-experiment`

## Executive Summary

The current code now correctly defaults fleet mode to the same runtime profile as single-drone mode: `single_equivalent`. This means the earlier fleet-profile mismatch is no longer the active explanation for the multi-drone failure. That part has been fixed.

The remaining failure for `--count > 1` is real and happens later in the stack:

- Fleet orchestration starts correctly.
- The simulator world and drone models are generated and launched correctly.
- The per-drone ROS bridge and node-manager containers start correctly.
- The failure occurs when `drone_1` is waiting to become takeoff-capable.

The strongest verified symptom is not simply "the timeout is too short." The stronger finding is that in the failing multi-drone runs, `drone_1`'s autopilot readiness path is unstable:

- DroneKit repeatedly reports `No heartbeat in 10.0 seconds, aborting.`
- SITL shows repeated MAVLink connection churn on `SERIAL1` and `SERIAL2`.
- After connection eventually establishes, `drone_1` remains stuck at `gps_fix=0`, `armable=False`, `state=BOOT`.
- Because of that, HLC never reaches `Ready`, MC never reaches `Standby`, and the fleet launcher times out exactly as designed.

So the current root problem is best described as:

> Multi-drone fleet bring-up is failing in the per-drone autopilot readiness chain for `drone_1`, not in the ROS layer and not in the old fleet runtime-profile divergence.

## Current Verified Status

### 1. Non-fleet single-drone works

The single-drone stack reaches a stable pre-takeoff idle state and later takes off successfully.

Evidence:

- `manual_runtime_test` reports repeated stable idle status with `MC=Standby`, `HLC=Ready`, `armed=False`, `gps_fix=6`: `logs/runs/20260415-165023-experiment/logs/manual-runtime-test.log:3`
- The same single run uses `runtime_profile = single_equivalent`, `max_step_size = 0.001`, `online_recv_timeout_ms = 10`: `logs/runs/20260415-165023-experiment/run_manifest.json`

### 2. Fleet with one drone works

The fleet stack also works when launched as `--count 1`.

Evidence:

- The run manifest shows this was a fleet run with one drone using `runtime_profile = single_equivalent`, `max_step_size = 0.001`, `online_recv_timeout_ms = 10`: `logs/runs/20260420-105519-experiment/run_manifest.json:4`
- `drone_1` HLC reaches `Ready`: `logs/runs/20260420-105519-experiment/logs/drone_1/hlc.log:39`
- `drone_1` MC reaches `Standby`: `logs/runs/20260420-105519-experiment/logs/drone_1/mc.log:14`

### 3. Fleet with more than one drone fails

The failing multi-drone runs also use `single_equivalent`, so they are already on the parity profile.

Evidence:

- `logs/runs/20260415-173340-experiment/run_manifest.json` shows `fleet = true`, `runtime_profile = single_equivalent`, `drone_count = 3`
- `logs/runs/20260415-173847-experiment/run_manifest.json` shows `fleet = true`, `runtime_profile = single_equivalent`, `drone_count = 2`

This verifies that the current failure is not caused by the old `fleet_control_stable` profile mismatch.

## Stack Workflow Breakdown

## 1. User entrypoint

The primary swarm entrypoint is [`scripts/run_controlled_swarm.sh`](../scripts/run_controlled_swarm.sh).

Its behavior is:

- `--count N` generates a fleet manifest and starts the visual fleet stack: `scripts/run_controlled_swarm.sh:19`
- Count-based fleet bring-up now defaults to `single_equivalent` for any `N`: `scripts/run_controlled_swarm.sh:22`, `scripts/run_controlled_swarm.sh:114`
- It synthesizes a manifest with `drone_1`, `drone_2`, ... and spawn offsets in `y`: `scripts/run_controlled_swarm.sh:137`
- After the stack comes up, it can attach teleop to a selected drone: `scripts/run_controlled_swarm.sh:223`
- It has a lightweight readiness gate that checks the selected drone's HLC and MC logs: `scripts/run_controlled_swarm.sh:201`

This script is the convenience wrapper. It is not the deep source of the failure.

## 2. Single-vs-fleet dispatch

[`scripts/run_visual_stack.sh`](../scripts/run_visual_stack.sh) decides whether to launch the single stack or the fleet stack.

- `--fleet PATH` dispatches to [`scripts/run_visual_fleet_stack.sh`](../scripts/run_visual_fleet_stack.sh): `scripts/run_visual_stack.sh:58`
- Without `--fleet`, it dispatches to [`scripts/run_visual_single_stack.sh`](../scripts/run_visual_single_stack.sh): `scripts/run_visual_stack.sh:65`

So the actual divergence between single and fleet starts here.

## 3. Runtime profile resolution

[`scripts/runtime_profiles.py`](../scripts/runtime_profiles.py) now defines `single_equivalent` as the base/default behavior.

- Base defaults include:
  - `online_recv_timeout_ms = 10`
  - `max_step_size = 0.001`
  - full camera behavior and rendering defaults
  - `scripts/runtime_profiles.py:9`
- `default_runtime_profile()` now always returns `single_equivalent`: `scripts/runtime_profiles.py:115`
- The old fleet optimization profiles still exist, but only as explicit opt-ins:
  - `fleet_control_stable`: `scripts/runtime_profiles.py:50`
  - `fleet_visual_optimized`: `scripts/runtime_profiles.py:81`

This confirms the intended parity contract is now implemented in code.

## 4. Single-stack asset generation

[`scripts/generate_visual_assets.py`](../scripts/generate_visual_assets.py) always builds single-drone assets using `single_equivalent`.

- It resolves single runtime defaults with `resolve_runtime_defaults("single_equivalent")`: `scripts/generate_visual_assets.py:488`
- It generates:
  - one runtime model
  - one runtime world
  - one Alate config
  - one `ros_alate` config
  - one `ros_nemala` config
  - `scripts/generate_visual_assets.py:500`

This is the reference implementation for per-drone behavior.

## 5. Fleet manifest resolution and asset generation

Fleet asset generation is split across two files:

- [`scripts/fleet_manifest.py`](../scripts/fleet_manifest.py)
- [`scripts/generate_visual_fleet_assets.py`](../scripts/generate_visual_fleet_assets.py)

### 5.1 Fleet manifest resolution

[`scripts/fleet_manifest.py`](../scripts/fleet_manifest.py):

- Loads the requested fleet manifest: `scripts/fleet_manifest.py:44`
- Resolves `runtime_profile` from the manifest, defaulting via `default_runtime_profile(...)`: `scripts/fleet_manifest.py:52`
- Applies runtime defaults from `runtime_profiles.py`: `scripts/fleet_manifest.py:66`
- Generates a per-drone vehicle slice with unique:
  - ID
  - namespace
  - SITL host alias
  - ports
  - proxy endpoints
  - spawn data
  - `scripts/fleet_manifest.py:71`

### 5.2 Fleet visual assets

[`scripts/generate_visual_fleet_assets.py`](../scripts/generate_visual_fleet_assets.py):

- Builds a fleet runtime model for each drone: `scripts/generate_visual_fleet_assets.py:289`
- Namespaces each drone camera topic: `scripts/generate_visual_fleet_assets.py:309`
- Applies runtime-profile-controlled sensor behavior: `scripts/generate_visual_fleet_assets.py:312`
- Rewrites each ArduPilot plugin's FDM ports for that specific drone: `scripts/generate_visual_fleet_assets.py:318`

This is where "single instance multiplied N times" is materialized.

## 6. Single visual stack workflow

[`scripts/run_visual_single_stack.sh`](../scripts/run_visual_single_stack.sh) launches the single-drone runtime slice.

It creates a run directory and then starts:

- `visual-sim`
- one SITL
- one proxy
- one logger
- one Mission Control (`mc`)
- one High-Level Control (`hlc`)
- one `ros_alate`
- one `ros_nemala`
- one `decision-dev`

Container set is declared here: `scripts/run_visual_single_stack.sh:36`

This is the known-good reference workflow.

## 7. Fleet visual stack workflow

[`scripts/run_visual_fleet_stack.sh`](../scripts/run_visual_fleet_stack.sh) is the main multi-drone orchestrator.

Its workflow is:

### 7.1 Generate fleet assets

- Copies the requested fleet config into the run directory: `scripts/run_visual_fleet_stack.sh:512`
- Generates visual/runtime assets via `generate_visual_fleet_assets.py`: `scripts/run_visual_fleet_stack.sh:523`
- Writes a run manifest and exports `manifest.env`: `scripts/run_visual_fleet_stack.sh:531`

### 7.2 Start shared simulator and sidecars

- Starts `visual-sim`: `scripts/run_visual_fleet_stack.sh:585`
- Verifies Gazebo topics: `scripts/run_visual_fleet_stack.sh:587`
- Verifies model deployment in the world: `scripts/run_visual_fleet_stack.sh:588`
- Starts shared `decision-dev`: `scripts/run_visual_fleet_stack.sh:590`
- Optionally starts focus-router and profiling sidecars: `scripts/run_visual_fleet_stack.sh:605`, `scripts/run_visual_fleet_stack.sh:620`

### 7.3 Start all SITL instances first

For every drone in `visual_assets/drones.tsv`, it starts a dedicated SITL:

- per-drone SITL container: `scripts/run_visual_fleet_stack.sh:655`
- unique `-I` instance index: `scripts/run_visual_fleet_stack.sh:662`
- unique `--serial0`, `--serial1`, `--serial2`: `scripts/run_visual_fleet_stack.sh:667`
- unique `--sim-port-in` and `--sim-port-out`: `scripts/run_visual_fleet_stack.sh:665`

Then it:

- waits for the chosen MAVLink TCP endpoint to come up: `scripts/run_visual_fleet_stack.sh:682`
- rewrites the generated Alate runtime config to point at that resolved SITL IP and port: `scripts/run_visual_fleet_stack.sh:683`

Important detail:

- Fleet mode intentionally uses `SERIAL1` as the deterministic DroneKit endpoint: `scripts/run_visual_fleet_stack.sh:680`

### 7.4 Start each drone's control slice one-by-one

After all SITL instances are up, the script sleeps five seconds and then, for each drone, starts:

- `proxy`: `scripts/run_visual_fleet_stack.sh:700`
- `logger`: `scripts/run_visual_fleet_stack.sh:709`
- `mc`: `scripts/run_visual_fleet_stack.sh:718`
- `hlc`: `scripts/run_visual_fleet_stack.sh:730`
- `ros_alate`: `scripts/run_visual_fleet_stack.sh:742`
- `ros_nemala`: `scripts/run_visual_fleet_stack.sh:755`

Then it waits for readiness before moving to the next drone:

- `wait_for_drone_readiness "${drone_id}" ... 240`: `scripts/run_visual_fleet_stack.sh:771`

This staged startup is intentional:

> "Stage the control slices one vehicle at a time. This keeps fleet bring-up deterministic and avoids overloading the shared simulator before each drone has fully reached its takeoff-capable ready state."

Source: `scripts/run_visual_fleet_stack.sh:768`

## 8. Readiness criteria

Fleet readiness is stricter than "container exists."

[`scripts/run_visual_fleet_stack.sh`](../scripts/run_visual_fleet_stack.sh) first waits for:

- `HLC entering state: Ready`
- `MissionControl entering state: Standby`

Source: `scripts/run_visual_fleet_stack.sh:444`

After that, it runs [`scripts/drone_readiness.py`](../scripts/drone_readiness.py), which performs a MAVLink-level readiness check:

- waits for stable heartbeats: `scripts/drone_readiness.py:145`
- records recent `STATUSTEXT`
- ensures recent `PreArm:` messages have cleared for a minimum window: `scripts/drone_readiness.py:186`

So the fleet timeout means the drone did not actually become takeoff-capable, not merely that logs were late.

## Verified Comparison: Single Works vs Multi Fails

## 1. The current profile parity fix is active

This part is verified.

Successful fleet-single:

- `fleet = true`
- `runtime_profile = single_equivalent`
- `max_step_size = 0.001`
- `online_recv_timeout_ms = 10`

Source: `logs/runs/20260420-105519-experiment/run_manifest.json:4`

Failing fleet-multi:

- same `runtime_profile = single_equivalent`
- same physics and FDM settings

Source:

- `logs/runs/20260415-173340-experiment/run_manifest.json`
- `logs/runs/20260415-173847-experiment/run_manifest.json`

Conclusion:

- The old fleet-profile mismatch is no longer the live blocker.

## 2. Simulator bring-up succeeds in the failing multi-drone path

The multi-drone failure happens after fleet visual/simulator startup, not before it.

Evidence:

- `run_visual_fleet_stack.sh` only begins per-drone readiness after:
  - Gazebo topics appear: `scripts/run_visual_fleet_stack.sh:587`
  - deployment verification succeeds: `scripts/run_visual_fleet_stack.sh:588`
  - all SITL instances are started and their TCP endpoints are reachable: `scripts/run_visual_fleet_stack.sh:650`

In the failing run, `visual-sim-server.log` exists and contains simulator activity and spawned-model behavior rather than an immediate startup crash.

## 3. ROS side is not the first failing layer

In the failing run `20260415-173847-experiment`, `ros_alate` and `ros_nemala` for `drone_1` are up and running.

This matches the launcher order:

- `ros_alate` starts before readiness wait: `scripts/run_visual_fleet_stack.sh:742`
- `ros_nemala` starts before readiness wait: `scripts/run_visual_fleet_stack.sh:755`

The strongest failure evidence does not point to ROS topics or node startup as the primary blocker.

## 4. The actual failing layer is the autopilot readiness chain

This is the strongest verified finding.

In `logs/runs/20260415-173847-experiment/logs/drone_1/hlc.log`:

- HLC loads the expected DroneKit master endpoint: `hlc.log:9`
- HLC launches the autopilot bridge: `hlc.log:17`
- DroneKit repeatedly fails with heartbeat timeouts:
  - `hlc.log:41`
  - `hlc.log:53`
  - `hlc.log:70`
  - `hlc.log:87`
  - `hlc.log:99`
- Only later does the connection establish: `hlc.log:102`
- After that, status stays stuck at:
  - `gps_fix=0`
  - `armable=False`
  - `state=BOOT`
  - examples:
    - `hlc.log:103`
    - `hlc.log:135`
    - `hlc.log:160`
    - `hlc.log:194`

At the same time, SITL shows repeated MAVLink endpoint churn:

- `logs/runs/20260415-173847-experiment/logs/drone_1/sitl.log:30`
- `logs/runs/20260415-173847-experiment/logs/drone_1/sitl.log:31`
- `logs/runs/20260415-173847-experiment/logs/drone_1/sitl.log:53`
- `logs/runs/20260415-173847-experiment/logs/drone_1/sitl.log:56`

This is consistent with an unstable MAVLink/DroneKit startup path for `drone_1`.

## 5. Why the readiness timeout is a consequence, not the cause

The fleet launcher times out only because the drone never satisfies the readiness gate.

The relevant gate is:

- wait for `HLC entering state: Ready`
- wait for `MissionControl entering state: Standby`
- then run MAVLink readiness verification

Source: `scripts/run_visual_fleet_stack.sh:444`

In the failing run, `drone_1` never gets there because the autopilot remains effectively unready. So "timeout" is the symptom seen by the launcher, not the root failure mode.

## Verification of Previous Analysis

## What was verified as correct

The previous analysis was correct on these main points:

- Multi-drone fleet failure is real.
- The failure is not explained by user input or higher-level swarm control logic.
- The fleet startup path gets far enough to launch the simulator and per-drone containers.
- The stronger failure signal is in the autopilot readiness chain, not in the ROS-side nodes.

## What was refined

One earlier suspicion no longer holds as the main explanation:

- The old difference between single and fleet runtime profiles was a valid historical bug.
- But it is no longer the active root blocker for the current `>1` failure.

Why:

- The current failing multi-drone runs already use `single_equivalent`.
- The current successful fleet-single run also uses `single_equivalent`.

So the parity-profile fix was necessary, but not sufficient.

Another refinement:

- `ArduPilot controller has reset` warnings are present even in successful runs.
- Example successful runs:
  - `logs/runs/20260420-105519-experiment/logs/visual-sim-server.log:117`
  - `logs/runs/20260415-165023-experiment/logs/visual-sim-server.log:92`

Therefore:

- controller reset warnings are useful supporting evidence
- but they are not by themselves enough to explain failure

The stronger discriminator is:

- repeated DroneKit heartbeat loss
- repeated SITL connection churn
- persistent `gps_fix=0`, `armable=False`, `state=BOOT`

## Most Likely Failure Location

The current best description of the failure location is:

1. Fleet launcher starts all SITLs.
2. Fleet launcher starts `drone_1` control slice.
3. HLC launches the DroneKit-based autopilot bridge against `tcp:<sitl_ip>:5762`.
4. DroneKit repeatedly times out waiting for heartbeat.
5. When a connection finally forms, the vehicle remains in `BOOT` with no GPS fix and not armable.
6. HLC never reaches `Ready`.
7. MC never reaches `Standby`.
8. Fleet readiness times out on `drone_1`.

This is the narrowest verified problem statement supported by current code and logs.

## Hypotheses That Still Need Debugging

The report does not claim root cause is fully solved. The following are the leading unresolved hypotheses:

### 1. MAVLink endpoint instability during multi-SITL bring-up

This is strongly supported by the repeated `SERIAL1`/`SERIAL2` connect-disconnect churn in `drone_1` SITL.

### 2. DroneKit startup racing with SITL stabilization

Although the fleet script waits for the TCP port to open, that only proves the socket is connectable. It does not prove the autopilot behind it is already stable enough for heartbeat and GPS readiness.

Relevant code:

- TCP endpoint probe only checks TCP connect success: `scripts/run_visual_fleet_stack.sh:350`
- HLC/DroneKit then immediately targets that endpoint: `scripts/run_visual_fleet_stack.sh:683`

### 3. Multi-instance SITL or simulator interaction preventing normal GPS/boot convergence

The failing drone eventually connects, but remains at:

- `gps_fix=0`
- `armable=False`
- `state=BOOT`

That suggests the problem may be later than transport alone. The transport stabilizes enough for some status traffic, but the autopilot never reaches a takeoff-capable state.

## Current Bottom Line

The stack status is now:

- Single non-fleet: good
- Single fleet: good
- Multi-fleet: broken

And the most defensible code-and-log-backed explanation is:

> The current multi-drone fleet failure occurs in `drone_1`'s autopilot readiness path during staged fleet bring-up. The launcher, simulator, and ROS side all progress far enough to start normally, but the DroneKit/SITL path becomes unstable, repeatedly loses heartbeat, and never converges to a GPS-fix/armable state. That prevents HLC/MC readiness and triggers the observed timeout.

## Recommended Next Debugging Branch

The next debugging session should focus narrowly on `drone_1` startup in multi-drone fleet mode:

1. Compare `drone_1` SITL startup sequence between:
   - successful `--count 1`
   - failing `--count 2` or `--count 3`
2. Trace why `SERIAL1` and `SERIAL2` are churning in the failing run.
3. Check whether the "TCP port is open" condition is too weak and allows HLC/DroneKit to attach before SITL is truly stable.
4. Compare autopilot boot/GPS readiness progression between single-fleet and multi-fleet for `drone_1`.
5. Only after that, decide whether the fix belongs in:
   - fleet launch sequencing
   - SITL endpoint selection
   - DroneKit connection strategy
   - simulator/SITL multi-instance configuration
