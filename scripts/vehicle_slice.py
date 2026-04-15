#!/usr/bin/env python3

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any

import yaml


DEFAULT_PORTS = {
    "serial0_base": 5760,
    "serial1_base": 5762,
    "serial2_base": 5763,
    "mavlink_step": 10,
    "fdm_in_base": 9002,
    "fdm_out_base": 9003,
    "fdm_step": 10,
    "gst_udp_base": 5600,
    "gst_udp_step": 10,
}

SINGLE_DRONE_ID = "drone_1"


def safe_slug(value: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9_]+", "_", value).strip("_")
    if not slug:
        raise RuntimeError(f"Invalid empty identifier derived from {value!r}")
    return slug


def dns_safe_slug(value: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9-]+", "-", value.replace("_", "-")).strip("-").lower()
    if not slug:
        raise RuntimeError(f"Invalid empty DNS identifier derived from {value!r}")
    return slug


def resolve_path(value: str, reference_path: Path, root_dir: Path) -> Path:
    candidate = Path(value)
    if candidate.is_absolute():
        return candidate
    reference_relative = (reference_path.parent / candidate).resolve()
    if reference_relative.exists():
        return reference_relative
    return (root_dir / candidate).resolve()


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


def runtime_config_relative_paths(drone_id: str) -> dict[str, str]:
    return {
        "alate": f"runtime/{drone_id}/uav.visual.sitl.json",
        "ros_alate": f"runtime/{drone_id}/ros_alate.adapter.yaml",
        "ros_nemala": f"runtime/{drone_id}/ros_nemala.node_manager.yaml",
    }


def _derive_ports(index: int, ports: dict[str, Any] | None = None) -> dict[str, int]:
    merged_ports = dict(DEFAULT_PORTS)
    if ports:
        merged_ports.update(ports)
    return {
        "serial0_port": int(merged_ports["serial0_base"]) + index * int(merged_ports["mavlink_step"]),
        "mavlink_port": int(merged_ports["serial1_base"]) + index * int(merged_ports["mavlink_step"]),
        "mavlink_aux_port": int(merged_ports["serial2_base"]) + index * int(merged_ports["mavlink_step"]),
        "fdm_port_in": int(merged_ports["fdm_in_base"]) + index * int(merged_ports["fdm_step"]),
        "fdm_port_out": int(merged_ports["fdm_out_base"]) + index * int(merged_ports["fdm_step"]),
        "gst_udp_port": int(merged_ports["gst_udp_base"]) + index * int(merged_ports["gst_udp_step"]),
    }


def _build_slice(
    *,
    index: int,
    drone_id: str,
    reference_path: Path,
    root_dir: Path,
    camera_deployment_config: str,
    alate_profile: str,
    ros_alate_profile: str,
    ros_nemala_profile: str,
    namespace: str,
    proxy_name: str,
    sitl_host: str,
    ipc_dir: str,
    spawn: dict[str, float],
    ports: dict[str, Any] | None = None,
) -> dict[str, Any]:
    resolved_camera_config = resolve_path(str(camera_deployment_config), reference_path, root_dir)
    resolved_alate_profile = resolve_path(str(alate_profile), reference_path, root_dir)
    resolved_ros_alate_profile = resolve_path(str(ros_alate_profile), reference_path, root_dir)
    resolved_ros_nemala_profile = resolve_path(str(ros_nemala_profile), reference_path, root_dir)
    derived_ports = _derive_ports(index, ports)
    relative_paths = runtime_config_relative_paths(drone_id)

    return {
        "id": safe_slug(drone_id),
        "index": index,
        "namespace": namespace,
        "proxy_name": proxy_name,
        "sitl_host": sitl_host,
        "spawn": {
            "x": float(spawn.get("x", 0.0)),
            "y": float(spawn.get("y", 0.0)),
            "z": float(spawn.get("z", 0.195)),
            "yaw_deg": float(spawn.get("yaw_deg", 90.0)),
        },
        "camera_deployment_config": str(resolved_camera_config),
        "camera_deployment": load_camera_pose(resolved_camera_config),
        "alate_profile": str(resolved_alate_profile),
        "ros_alate_profile": str(resolved_ros_alate_profile),
        "ros_nemala_profile": str(resolved_ros_nemala_profile),
        **derived_ports,
        "ipc_dir": ipc_dir,
        "proxy_endpoints": {
            "subscribers": f"ipc://{ipc_dir}/alate_subscribers",
            "publishers": f"ipc://{ipc_dir}/alate_publishers",
            "logger": f"ipc://{ipc_dir}/alate_log",
        },
        "camera_topics": {
            "chase": f"/mars/{drone_id}/visual/chase_camera" if namespace else "/mars/visual/chase_camera",
            "deployed": f"/mars/{drone_id}/visual/deployed_camera" if namespace else "/mars/visual/deployed_camera",
        },
        "runtime_paths": relative_paths,
    }


def build_fleet_vehicle_slice(
    *,
    index: int,
    drone_id: str,
    manifest_path: Path,
    root_dir: Path,
    camera_deployment_config: str,
    alate_profile: str,
    ros_alate_profile: str,
    ros_nemala_profile: str,
    spawn: dict[str, Any] | None = None,
    ports: dict[str, Any] | None = None,
) -> dict[str, Any]:
    normalized_drone_id = safe_slug(drone_id)
    return _build_slice(
        index=index,
        drone_id=normalized_drone_id,
        reference_path=manifest_path.resolve(),
        root_dir=root_dir,
        camera_deployment_config=camera_deployment_config,
        alate_profile=alate_profile,
        ros_alate_profile=ros_alate_profile,
        ros_nemala_profile=ros_nemala_profile,
        namespace=f"/{normalized_drone_id}",
        proxy_name=normalized_drone_id,
        sitl_host=f"sitl-{dns_safe_slug(normalized_drone_id)}",
        ipc_dir=f"/tmp/nemala/{normalized_drone_id}",
        spawn=spawn or {},
        ports=ports,
    )


def build_single_vehicle_slice(
    *,
    reference_path: Path,
    root_dir: Path,
    camera_deployment_config: str,
    alate_profile: str,
    ros_alate_profile: str,
    ros_nemala_profile: str,
    ports: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return _build_slice(
        index=0,
        drone_id=SINGLE_DRONE_ID,
        reference_path=reference_path.resolve(),
        root_dir=root_dir,
        camera_deployment_config=camera_deployment_config,
        alate_profile=alate_profile,
        ros_alate_profile=ros_alate_profile,
        ros_nemala_profile=ros_nemala_profile,
        namespace="",
        proxy_name=SINGLE_DRONE_ID,
        sitl_host="sitl",
        ipc_dir=f"/tmp/nemala/{SINGLE_DRONE_ID}",
        spawn={"x": 0.0, "y": 0.0, "z": 0.195, "yaw_deg": 90.0},
        ports=ports,
    )


def _resolve_node_key(data: dict[str, Any], node_name: str) -> str:
    for key in data.keys():
        if key == node_name or key == f"/{node_name}" or key.rsplit("/", 1)[-1] == node_name:
            return key
    return node_name


def _extract_params(data: dict[str, Any], node_name: str) -> dict[str, Any]:
    resolved_node_name = _resolve_node_key(data, node_name)
    return dict(data.get(resolved_node_name, {}).get("ros__parameters", {}))


def render_alate_config(drone: dict[str, Any], output_path: Path) -> None:
    data = json.loads(Path(drone["alate_profile"]).read_text())
    data["proxies"] = {
        drone["proxy_name"]: {
            "subscribers": drone["proxy_endpoints"]["subscribers"],
            "publishers": drone["proxy_endpoints"]["publishers"],
            "logger": drone["proxy_endpoints"]["logger"],
        }
    }
    autopilot = dict(data.get("autopilot", {}))
    autopilot["master"] = f"tcp:{drone['sitl_host']}:{drone['mavlink_port']}"
    data["autopilot"] = autopilot
    output_path.write_text(json.dumps(data, indent=2) + "\n")


def render_ros_alate_config(drone: dict[str, Any], output_path: Path) -> None:
    data = yaml.safe_load(Path(drone["ros_alate_profile"]).read_text())
    params = _extract_params(data, "ros_alate_adapter")
    params["proxy_endpoint_for_publishing"] = drone["proxy_endpoints"]["publishers"]
    params["proxy_endpoint_for_subscribing"] = drone["proxy_endpoints"]["subscribers"]
    node_name = "ros_alate_adapter" if not drone["namespace"] else f"{drone['namespace']}/ros_alate_adapter"
    output_path.write_text(yaml.safe_dump({node_name: {"ros__parameters": params}}, sort_keys=False))


def render_ros_nemala_config(drone: dict[str, Any], output_path: Path) -> None:
    data = yaml.safe_load(Path(drone["ros_nemala_profile"]).read_text())
    params = _extract_params(data, "nemala_node_manager")
    params["proxy_endpoint_publishers"] = drone["proxy_endpoints"]["publishers"]
    node_name = "nemala_node_manager" if not drone["namespace"] else f"{drone['namespace']}/nemala_node_manager"
    output_path.write_text(yaml.safe_dump({node_name: {"ros__parameters": params}}, sort_keys=False))
