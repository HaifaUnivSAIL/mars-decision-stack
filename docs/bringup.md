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
