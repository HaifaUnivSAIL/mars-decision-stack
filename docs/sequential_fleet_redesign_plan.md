# Sequential Fleet Redesign Plan

## Status

This document captures a fallback redesign for fleet bring-up while we continue debugging the current shared-world fleet design.

Current working state:

- Non-fleet single-drone stack works.
- Fleet stack with `--count 1` works.
- Fleet stack with `--count > 1` fails during readiness.

Current strategy:

- Keep debugging the current fleet design first.
- Do not implement this redesign yet.
- Trigger this redesign only if the current design stops yielding progress or proves fundamentally brittle.

## Goal

Make fleet bring-up behave as close as possible to:

> single-drone bring-up, repeated one drone at a time

while preserving the final outcome of a shared swarm world.

The intent is:

1. Start Gazebo once with a base world.
2. Insert one drone into the running world.
3. Bring that drone fully to health using the same effective procedure as the working single-drone path.
4. Only after that drone is healthy, insert the next drone.
5. Repeat until the requested fleet size is online.

This design does not eliminate shared-world simulation coupling, but it makes fleet deployment incremental, observable, and much closer to the single-drone boot contract.

## Why Consider This Redesign

The current fleet codestack preloads all drone models into Gazebo before per-drone readiness is proven.

That means:

- Gazebo starts in full multi-drone mode immediately.
- All ArduPilot plugin instances are live from the beginning.
- The first drone is not truly brought up in a single-drone-equivalent simulator state.

So even if control slices are started sequentially, the simulator layer is still already in multi-drone mode.

The proposed redesign changes that:

- simulator insertion becomes sequential, not just control-slice startup
- the first drone gets the closest possible environment to the single-drone case
- adding each additional drone becomes an explicit transition point that can be validated

## Architectural Direction

### Keep

- One SITL per drone.
- One Alate `mc` per drone.
- One Alate `hlc` per drone.
- One `ros_alate` per drone.
- One `ros_nemala` per drone.
- Shared `decision-dev` environment.
- Existing per-drone namespaces, ports, manifests, and runtime configs.
- Existing swarm-controller work in the ROS decision layer.

### Change

- Stop launching Gazebo with all drone models pre-inserted.
- Launch Gazebo with a base world only.
- Spawn each drone model into the already running world one at a time.
- Gate deployment on per-drone readiness before proceeding to the next drone.

## Proposed Bring-Up Workflow

### Phase 1: Base World Startup

1. Generate the fleet manifest as today.
2. Generate per-drone model/runtime assets as today.
3. Generate a base world SDF with no drone models embedded.
4. Start `visual-sim` using that base world.
5. Start shared sidecars that do not depend on a specific drone already being healthy.

Expected shared services:

- Gazebo server
- `decision-dev`
- optional profiling sidecars
- optional focus router only after at least one drone exists

### Phase 2: Sequential Drone Deployment

For each drone in manifest order:

1. Spawn that drone’s model into the running Gazebo world.
2. Wait until Gazebo confirms the model exists and its required topics/plugins are live.
3. Start that drone’s SITL.
4. Wait for its deterministic MAVLink endpoints to open.
5. Run the low-level readiness probe.
6. Rewrite the generated Alate `autopilot.master` using the resolved runtime endpoint.
7. Start that drone’s proxy/logger/`mc`/`hlc`/`ros_alate`/`ros_nemala`.
8. Wait until the drone reaches full readiness:
   - stable MAVLink
   - HLC ready
   - MC standby
   - healthy telemetry
9. Only then continue to the next drone.

### Phase 3: Swarm Ready

After all drones are healthy:

1. Mark the fleet ready.
2. Start higher-level swarm actions or experiments.
3. Keep all existing profiling/validation hooks active.

## Codestack Impact

### Files Likely To Change

- `scripts/run_visual_fleet_stack.sh`
  - convert from preload-world fleet launch to incremental spawn workflow
- `scripts/generate_visual_fleet_assets.py`
  - generate a base world plus per-drone spawnable model assets
- possibly `scripts/generate_visual_assets.py`
  - only if we want stronger reuse of the single-drone generator path
- `scripts/verify_visual_fleet_runtime.py`
  - validate runtime state under sequential deployment
- `scripts/analyze_stack_profile.py`
  - add deployment-stage metrics per drone

### Likely New Helpers

- a script to spawn one drone model into a live Gazebo world
- a script to verify that a specific model’s plugin/topic footprint is active
- optional despawn/cleanup helper for failed partial bring-up

### Likely Unchanged

- `fleet_manifest.py`
- per-drone Alate configs
- per-drone ROS bridge contract
- swarm controller design in `decision_agent`
- `run_controlled_swarm.sh` public interface, except possibly improved status output

## Benefits

- Brings fleet startup closer to the working single-drone mental model.
- Makes the moment of failure easier to localize.
- Lets us test a stronger invariant:
  - drone 1 healthy alone in the world
  - then drone 2 inserted
  - then observe whether drone 1 regresses
- Reduces ambiguity between:
  - bad per-drone startup
  - bad multi-drone shared-world interaction

## Risks

- This is a real bring-up redesign, not a small patch.
- Gazebo dynamic spawning may expose different issues than world-preloaded models.
- Some plugins or assumptions may currently rely on model-at-world-start semantics.
- The shared-world multi-drone bug may still exist after sequential insertion; this redesign improves isolation and observability, but does not guarantee removal of the underlying simulator coupling.

## Debugging Strategy Before Triggering This Redesign

We should continue debugging the current design while all of these remain true:

- the failure is clearly narrowing
- new instrumentation is producing new evidence
- fixes are still removing meaningful uncertainty
- we do not yet have proof that preloaded multi-drone world startup is inherently incompatible with the current Gazebo/ArduPilot path

## Trigger Conditions For Switching To This Redesign

Implement this redesign if one or more of these become true:

1. The current debugging path stops producing new evidence.
2. We confirm the shared-world preloaded multi-drone startup is the root coupling problem.
3. Fixes in orchestration, readiness gating, and plugin/bootstrap behavior still leave `--count > 1` unstable.
4. We need a stronger architectural guarantee that drone bring-up is single-equivalent before swarm activation.

## Success Criteria

### Startup

- `--count 1`, `--count 2`, and `--count 3` all complete readiness successfully.
- Each drone is brought up one at a time.
- No drone already marked healthy regresses when the next drone is inserted.

### Behavioral Parity

- Drone 1 in sequential fleet deployment behaves like the working single-drone stack before drone 2 is inserted.
- After drone 2 is inserted, drone 1 remains healthy.
- The same holds as additional drones are added.

### Observability

- The run artifacts clearly record:
  - spawn time per drone
  - SITL ready time per drone
  - HLC ready time per drone
  - MC standby time per drone
  - any regression after a subsequent drone insertion

## Recommendation

Do not implement this redesign yet.

Keep debugging the current fleet design first, because we may still be close to the actual low-level failure. But preserve this document in the repo so that if progress stalls, we can switch from debugging to redesign with a clear, pre-agreed implementation direction.
