# Mars Decision Stack

`Mars Decision Stack` is the integration repo that binds together:

- `external/alate` for the NeMALA/Alate runtime
- `external/nemala_core` for NeMALA communication primitives
- `ws/src/ros_*` for the ROS 2 bridge layer
- `ws/src/decision_agent` for new decision-making algorithms

The original supervisor architecture document remains at [`mars_alate_integration_spec.md`](./mars_alate_integration_spec.md).
This repo implements that direction with a few grounded adjustments:

- `vcstool` manifests are used instead of git submodules
- ROS packages live directly in `ws/src`
- `ws/build`, `ws/install`, and `ws/log` are not tracked
- bridge output topics are completed locally because upstream `ros_alate` does not currently publish them

## Layout

- `manifests/stack.repos`: pinned source dependency versions
- `patches/`: local patch set applied on top of pinned upstream bases
- `docker/`: reproducible container build and optional Compose topology
- `scripts/`: bootstrap, build, runtime, dev-shell, and validation helpers
- `config/`: runtime configuration for Alate, bridge nodes, and `decision_agent`
- `docs/`: architecture, topic mapping, and bringup notes
- `ws/src/decision_agent`: local decision package

## Quick Start

```bash
cd /home/guy-sassy/projects/Nemala
./scripts/bootstrap.sh
./scripts/build_ws.sh
./scripts/run_stack.sh
./scripts/validate.sh
./scripts/dev_shell.sh
```

Stop the stack with:

```bash
./scripts/stop_stack.sh
```
