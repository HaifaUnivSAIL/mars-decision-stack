#!/usr/bin/env python3

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from runtime_profiles import default_runtime_profile, resolve_runtime_defaults
from vehicle_slice import build_fleet_vehicle_slice, safe_slug


DEFAULTS = {
    "camera_deployment_config": "config/visual/camera.deployment.json",
    "alate_profile": "config/alate/uav.visual.sitl.json",
    "ros_alate_profile": "config/ros_alate/adapter.yaml",
    "ros_nemala_profile": "config/ros_nemala/node_manager.yaml",
    "ports": {
        "serial0_base": 5760,
        "serial1_base": 5762,
        "serial2_base": 5763,
        "mavlink_step": 10,
        "fdm_in_base": 9002,
        "fdm_out_base": 9003,
        "fdm_step": 10,
        "gst_udp_base": 5600,
        "gst_udp_step": 10,
    },
}

ACTIVE_CHASE_TOPIC = "/mars/visual/active/chase_camera"
ACTIVE_CAMERA_TOPIC = "/mars/visual/active/deployed_camera"
FOCUS_SELECT_TOPIC = "/mars/visual/active_drone/select"
FOCUS_STATE_TOPIC = "/mars/visual/active_drone/state"
WORLD_NAME = "mars_iris_dual_view"
BASE_RUNTIME_MODEL_NAME = "iris_with_camera"


def load_fleet_definition(manifest_path: Path, root_dir: Path) -> dict[str, Any]:
    manifest_path = manifest_path.resolve()
    raw = json.loads(manifest_path.read_text())
    raw_defaults = json.loads(json.dumps(raw.get("defaults", {})))
    drones_raw = raw.get("drones", [])
    if not drones_raw:
        raise RuntimeError(f"Fleet manifest {manifest_path} does not define any drones")

    runtime_profile = str(
        raw_defaults.pop("runtime_profile", raw.get("runtime_profile", default_runtime_profile(len(drones_raw))))
    )

    runtime_overrides: dict[str, Any] = {}
    for key in ("fdm_exchange", "camera_streams", "rendering", "physics", "sensor_behavior"):
        if key in raw_defaults:
            runtime_overrides[key] = raw_defaults.pop(key)

    defaults = dict(DEFAULTS)
    defaults.update(raw_defaults)
    defaults_ports = dict(DEFAULTS["ports"])
    defaults_ports.update(defaults.get("ports", {}))
    defaults["ports"] = defaults_ports
    defaults.update(resolve_runtime_defaults(runtime_profile, runtime_overrides))
    defaults["runtime_profile"] = runtime_profile

    seen_ids: set[str] = set()
    drones: list[dict[str, Any]] = []
    for index, entry in enumerate(drones_raw):
        drone = build_fleet_vehicle_slice(
            index=index,
            drone_id=str(entry.get("id", "")),
            manifest_path=manifest_path,
            root_dir=root_dir,
            camera_deployment_config=str(entry.get("camera_deployment_config", defaults["camera_deployment_config"])),
            alate_profile=str(entry.get("alate_profile", defaults["alate_profile"])),
            ros_alate_profile=str(entry.get("ros_alate_profile", defaults["ros_alate_profile"])),
            ros_nemala_profile=str(entry.get("ros_nemala_profile", defaults["ros_nemala_profile"])),
            spawn=entry.get("spawn", {}),
            ports=defaults["ports"],
        )
        if drone["id"] in seen_ids:
            raise RuntimeError(f"Fleet manifest {manifest_path} defines duplicate drone id {drone['id']}")
        seen_ids.add(drone["id"])

        drones.append(drone)

    active_drone_id = safe_slug(str(raw.get("active_drone_id") or drones[0]["id"]))
    if active_drone_id not in seen_ids:
        raise RuntimeError(
            f"Fleet manifest {manifest_path} sets active_drone_id={active_drone_id!r}, which is not defined"
        )

    return {
        "manifest_path": str(manifest_path),
        "active_drone_id": active_drone_id,
        "defaults": defaults,
        "runtime_profile": defaults["runtime_profile"],
        "camera_streams": defaults["camera_streams"],
        "rendering": defaults["rendering"],
        "physics": defaults["physics"],
        "fdm_exchange": defaults["fdm_exchange"],
        "sensor_behavior": defaults["sensor_behavior"],
        "world_name": WORLD_NAME,
        "active_topics": {
            "chase": ACTIVE_CHASE_TOPIC,
            "deployed": ACTIVE_CAMERA_TOPIC,
        },
        "focus_topics": {
            "select": FOCUS_SELECT_TOPIC,
            "state": FOCUS_STATE_TOPIC,
        },
        "drones": drones,
    }


def build_runtime_fleet(manifest_path: Path, root_dir: Path, run_id: str) -> dict[str, Any]:
    definition = load_fleet_definition(manifest_path, root_dir)
    runtime_drones: list[dict[str, Any]] = []
    for drone in definition["drones"]:
        runtime_model_name = f"{BASE_RUNTIME_MODEL_NAME}_experiment_{safe_slug(run_id)}_{drone['id']}"
        runtime_drones.append(
            {
                **drone,
                "runtime_model_name": runtime_model_name,
            }
        )

    return {
        **definition,
        "run_id": run_id,
        "mode": "experiment",
        "runtime_world_name": WORLD_NAME,
        "runtime_profile": definition["runtime_profile"],
        "fdm_exchange": definition["fdm_exchange"],
        "sensor_behavior": definition["sensor_behavior"],
        "drones": runtime_drones,
    }
