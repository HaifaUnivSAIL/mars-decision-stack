#!/usr/bin/env python3

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any


DEFAULTS = {
    "camera_deployment_config": "config/visual/camera.deployment.json",
    "alate_profile": "config/alate/uav.visual.sitl.json",
    "ros_alate_profile": "config/ros_alate/adapter.yaml",
    "ros_nemala_profile": "config/ros_nemala/node_manager.yaml",
    "camera_streams": {
        "deployed": {
            "width": 640,
            "height": 480,
            "update_rate_hz": 10.0,
        },
        "chase": {
            "width": 960,
            "height": 540,
            "update_rate_hz": 15.0,
        },
    },
    "rendering": {
        "sun_cast_shadows": False,
    },
    "physics": {
        "max_step_size": 0.005,
        "real_time_factor": 1.0,
    },
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


def _safe_slug(value: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9_]+", "_", value).strip("_")
    if not slug:
        raise RuntimeError(f"Invalid empty identifier derived from {value!r}")
    return slug


def _dns_safe_slug(value: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9-]+", "-", value.replace("_", "-")).strip("-").lower()
    if not slug:
        raise RuntimeError(f"Invalid empty DNS identifier derived from {value!r}")
    return slug


def _resolve_path(value: str, manifest_path: Path, root_dir: Path) -> Path:
    candidate = Path(value)
    if candidate.is_absolute():
        return candidate
    manifest_relative = (manifest_path.parent / candidate).resolve()
    if manifest_relative.exists():
        return manifest_relative
    root_relative = (root_dir / candidate).resolve()
    return root_relative


def load_camera_pose(config_path: Path) -> dict[str, float]:
    data = json.loads(config_path.read_text())
    deployment = data["deployment"]
    position = deployment["position_m"]
    orientation = deployment["orientation_rad"]
    return {
        "x": float(position["x"]),
        "y": float(position["y"]),
        "z": float(position["z"]),
        "yaw": float(orientation["yaw"]),
        "pitch": float(orientation["pitch"]),
        "roll": float(orientation["roll"]),
    }


def load_fleet_definition(manifest_path: Path, root_dir: Path) -> dict[str, Any]:
    manifest_path = manifest_path.resolve()
    raw = json.loads(manifest_path.read_text())
    defaults = dict(DEFAULTS)
    defaults.update(raw.get("defaults", {}))
    defaults_camera_streams = json.loads(json.dumps(DEFAULTS["camera_streams"]))
    defaults_camera_streams.update(defaults.get("camera_streams", {}))
    for stream_name, stream_defaults in DEFAULTS["camera_streams"].items():
        merged_stream_defaults = dict(stream_defaults)
        merged_stream_defaults.update(defaults_camera_streams.get(stream_name, {}))
        defaults_camera_streams[stream_name] = merged_stream_defaults
    defaults["camera_streams"] = defaults_camera_streams
    defaults_rendering = dict(DEFAULTS["rendering"])
    defaults_rendering.update(defaults.get("rendering", {}))
    defaults["rendering"] = defaults_rendering
    defaults_physics = dict(DEFAULTS["physics"])
    defaults_physics.update(defaults.get("physics", {}))
    defaults["physics"] = defaults_physics
    defaults_ports = dict(DEFAULTS["ports"])
    defaults_ports.update(defaults.get("ports", {}))
    defaults["ports"] = defaults_ports

    drones_raw = raw.get("drones", [])
    if not drones_raw:
        raise RuntimeError(f"Fleet manifest {manifest_path} does not define any drones")

    seen_ids: set[str] = set()
    drones: list[dict[str, Any]] = []
    for index, entry in enumerate(drones_raw):
        drone_id = _safe_slug(str(entry.get("id", "")))
        if drone_id in seen_ids:
            raise RuntimeError(f"Fleet manifest {manifest_path} defines duplicate drone id {drone_id}")
        seen_ids.add(drone_id)

        spawn = entry.get("spawn", {})
        camera_config = _resolve_path(
            str(entry.get("camera_deployment_config", defaults["camera_deployment_config"])),
            manifest_path,
            root_dir,
        )
        alate_profile = _resolve_path(
            str(entry.get("alate_profile", defaults["alate_profile"])),
            manifest_path,
            root_dir,
        )
        ros_alate_profile = _resolve_path(str(defaults["ros_alate_profile"]), manifest_path, root_dir)
        ros_nemala_profile = _resolve_path(str(defaults["ros_nemala_profile"]), manifest_path, root_dir)
        pose = load_camera_pose(camera_config)

        ports = defaults["ports"]
        serial0_port = int(ports["serial0_base"]) + index * int(ports["mavlink_step"])
        mavlink_port = int(ports["serial1_base"]) + index * int(ports["mavlink_step"])
        mavlink_aux_port = int(ports["serial2_base"]) + index * int(ports["mavlink_step"])
        fdm_port_in = int(ports["fdm_in_base"]) + index * int(ports["fdm_step"])
        fdm_port_out = int(ports["fdm_out_base"]) + index * int(ports["fdm_step"])
        gst_udp_port = int(ports["gst_udp_base"]) + index * int(ports["gst_udp_step"])

        drones.append(
            {
                "id": drone_id,
                "index": index,
                "namespace": f"/{drone_id}",
                "proxy_name": drone_id,
                "sitl_host": f"sitl-{_dns_safe_slug(drone_id)}",
                "spawn": {
                    "x": float(spawn.get("x", 0.0)),
                    "y": float(spawn.get("y", 0.0)),
                    "z": float(spawn.get("z", 0.195)),
                    "yaw_deg": float(spawn.get("yaw_deg", 90.0)),
                },
                "camera_deployment_config": str(camera_config),
                "camera_deployment": pose,
                "alate_profile": str(alate_profile),
                "ros_alate_profile": str(ros_alate_profile),
                "ros_nemala_profile": str(ros_nemala_profile),
                "serial0_port": serial0_port,
                "mavlink_port": mavlink_port,
                "mavlink_aux_port": mavlink_aux_port,
                "fdm_port_in": fdm_port_in,
                "fdm_port_out": fdm_port_out,
                "gst_udp_port": gst_udp_port,
                "ipc_dir": f"/tmp/nemala/{drone_id}",
                "proxy_endpoints": {
                    "subscribers": f"ipc:///tmp/nemala/{drone_id}/alate_subscribers",
                    "publishers": f"ipc:///tmp/nemala/{drone_id}/alate_publishers",
                    "logger": f"ipc:///tmp/nemala/{drone_id}/alate_log",
                },
                "camera_topics": {
                    "chase": f"/mars/{drone_id}/visual/chase_camera",
                    "deployed": f"/mars/{drone_id}/visual/deployed_camera",
                },
            }
        )

    active_drone_id = str(raw.get("active_drone_id") or drones[0]["id"])
    if active_drone_id not in seen_ids:
        raise RuntimeError(
            f"Fleet manifest {manifest_path} sets active_drone_id={active_drone_id!r}, which is not defined"
        )

    return {
        "manifest_path": str(manifest_path),
        "active_drone_id": active_drone_id,
        "defaults": defaults,
        "camera_streams": defaults["camera_streams"],
        "rendering": defaults["rendering"],
        "physics": defaults["physics"],
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
        runtime_model_name = f"{BASE_RUNTIME_MODEL_NAME}_experiment_{_safe_slug(run_id)}_{drone['id']}"
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
        "drones": runtime_drones,
    }
