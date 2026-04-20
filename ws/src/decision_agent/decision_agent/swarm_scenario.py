from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path

import yaml

from .swarm_geometry import CenterTarget, LocalPose, angle_delta, circular_mean, clamp, rotate_xy, world_to_body


@dataclass(frozen=True)
class FormationOffset:
    x: float
    y: float
    z: float = 0.0
    yaw_rad: float = 0.0


@dataclass(frozen=True)
class SwarmControllerConfig:
    horizontal_gain: float = 0.6
    vertical_gain: float = 0.7
    yaw_gain: float = 0.8
    max_linear_xy_mps: float = 0.45
    max_linear_z_mps: float = 0.35
    max_angular_z_rad_s: float = 0.6
    position_deadband_m: float = 0.05
    vertical_deadband_m: float = 0.05
    yaw_deadband_rad: float = math.radians(5.0)


@dataclass(frozen=True)
class SwarmScenarioAction:
    kind: str
    duration_sec: float
    delta_x: float = 0.0
    delta_y: float = 0.0
    delta_z: float = 0.0
    delta_yaw_rad: float = 0.0
    label: str = ""


@dataclass(frozen=True)
class SwarmScenarioConfig:
    scenario_name: str
    control_frequency_hz: float
    telemetry_timeout_sec: float
    require_ready_state: bool
    ready_hold_sec: float
    auto_takeoff: bool
    takeoff_altitude_m: float
    takeoff_timeout_sec: float
    auto_land_after_actions: bool
    landing_timeout_sec: float
    landed_altitude_m: float
    actions: list[SwarmScenarioAction]
    controller: SwarmControllerConfig
    formation_offsets: dict[str, FormationOffset]


def _as_float(payload: dict, key: str, default: float = 0.0) -> float:
    value = payload.get(key, default)
    return float(value)


def _parse_offset(payload: dict) -> FormationOffset:
    yaw_deg = float(payload.get("yaw_deg", 0.0))
    return FormationOffset(
        x=_as_float(payload, "x"),
        y=_as_float(payload, "y"),
        z=_as_float(payload, "z"),
        yaw_rad=math.radians(yaw_deg),
    )


def _parse_action(entry: dict) -> SwarmScenarioAction:
    if len(entry) != 1:
        raise ValueError(f"Action must contain exactly one key: {entry}")
    kind, raw_payload = next(iter(entry.items()))
    if kind == "hold":
        duration_sec = float(raw_payload)
        return SwarmScenarioAction(kind="hold", duration_sec=duration_sec, label=f"hold {duration_sec:.2f}s")
    if kind in {"translate_world", "translate_body"}:
        payload = raw_payload or {}
        duration_sec = _as_float(payload, "duration_sec")
        delta_x = _as_float(payload, "x")
        delta_y = _as_float(payload, "y")
        delta_z = _as_float(payload, "z")
        return SwarmScenarioAction(
            kind=kind,
            duration_sec=duration_sec,
            delta_x=delta_x,
            delta_y=delta_y,
            delta_z=delta_z,
            label=f"{kind} ({delta_x:.2f}, {delta_y:.2f}, {delta_z:.2f}) for {duration_sec:.2f}s",
        )
    if kind == "yaw":
        payload = raw_payload or {}
        duration_sec = _as_float(payload, "duration_sec")
        delta_yaw_rad = math.radians(_as_float(payload, "yaw_deg"))
        return SwarmScenarioAction(
            kind="yaw",
            duration_sec=duration_sec,
            delta_yaw_rad=delta_yaw_rad,
            label=f"yaw {math.degrees(delta_yaw_rad):.2f}deg for {duration_sec:.2f}s",
        )
    if kind == "land":
        return SwarmScenarioAction(kind="land", duration_sec=0.0, label="land")
    raise ValueError(f"Unsupported swarm action: {json.dumps(entry, sort_keys=True)}")


def load_swarm_scenario(path: Path) -> SwarmScenarioConfig:
    raw = yaml.safe_load(path.read_text()) or {}
    controller_raw = raw.get("controller") or {}
    controller = SwarmControllerConfig(
        horizontal_gain=float(controller_raw.get("horizontal_gain", 0.6)),
        vertical_gain=float(controller_raw.get("vertical_gain", 0.7)),
        yaw_gain=float(controller_raw.get("yaw_gain", 0.8)),
        max_linear_xy_mps=float(controller_raw.get("max_linear_xy_mps", 0.45)),
        max_linear_z_mps=float(controller_raw.get("max_linear_z_mps", 0.35)),
        max_angular_z_rad_s=float(controller_raw.get("max_angular_z_rad_s", 0.6)),
        position_deadband_m=float(controller_raw.get("position_deadband_m", 0.05)),
        vertical_deadband_m=float(controller_raw.get("vertical_deadband_m", 0.05)),
        yaw_deadband_rad=math.radians(float(controller_raw.get("yaw_deadband_deg", 5.0))),
    )
    formation_offsets = {
        str(drone_id): _parse_offset(payload or {})
        for drone_id, payload in (raw.get("formation_offsets") or {}).items()
    }
    actions = [_parse_action(entry) for entry in list(raw.get("actions") or [])]
    if not actions:
        raise ValueError("Swarm scenario must define at least one action")
    return SwarmScenarioConfig(
        scenario_name=str(raw.get("scenario_name", path.stem)),
        control_frequency_hz=float(raw.get("control_frequency_hz", 10.0)),
        telemetry_timeout_sec=float(raw.get("telemetry_timeout_sec", 2.0)),
        require_ready_state=bool(raw.get("require_ready_state", True)),
        ready_hold_sec=float(raw.get("ready_hold_sec", 1.0)),
        auto_takeoff=bool(raw.get("auto_takeoff", True)),
        takeoff_altitude_m=float(raw.get("takeoff_altitude_m", 3.0)),
        takeoff_timeout_sec=float(raw.get("takeoff_timeout_sec", 40.0)),
        auto_land_after_actions=bool(raw.get("auto_land_after_actions", True)),
        landing_timeout_sec=float(raw.get("landing_timeout_sec", 45.0)),
        landed_altitude_m=float(raw.get("landed_altitude_m", 0.3)),
        actions=actions,
        controller=controller,
        formation_offsets=formation_offsets,
    )


def resolve_formation_offsets(manifest: dict, configured_offsets: dict[str, FormationOffset]) -> dict[str, FormationOffset]:
    if configured_offsets:
        missing = [drone["id"] for drone in manifest.get("drones", []) if drone["id"] not in configured_offsets]
        if missing:
            raise ValueError(f"Formation offsets missing drones: {missing}")
        return configured_offsets
    drones = manifest.get("drones", [])
    if not drones:
        raise ValueError("Manifest does not define any drones")
    active_drone_id = str(manifest.get("active_drone_id") or drones[0]["id"])
    origin_drone = next((drone for drone in drones if str(drone["id"]) == active_drone_id), drones[0])
    origin_spawn = origin_drone.get("spawn", {})
    origin_x = float(origin_spawn.get("x", 0.0))
    origin_y = float(origin_spawn.get("y", 0.0))
    origin_z = float(origin_spawn.get("z", 0.0))
    resolved = {}
    for drone in drones:
        spawn = drone.get("spawn", {})
        resolved[str(drone["id"])] = FormationOffset(
            x=float(spawn.get("x", 0.0)) - origin_x,
            y=float(spawn.get("y", 0.0)) - origin_y,
            z=float(spawn.get("z", 0.0)) - origin_z,
            yaw_rad=0.0,
        )
    return resolved


def compute_initial_center_target(local_poses: dict[str, LocalPose], formation_offsets: dict[str, FormationOffset]) -> CenterTarget:
    yaw_samples = [local_poses[drone_id].yaw - formation_offsets[drone_id].yaw_rad for drone_id in local_poses]
    center_yaw = circular_mean(yaw_samples)
    center_x_samples = []
    center_y_samples = []
    center_z_samples = []
    for drone_id, pose in local_poses.items():
        offset = formation_offsets[drone_id]
        offset_x, offset_y = rotate_xy(offset.x, offset.y, center_yaw)
        center_x_samples.append(pose.x - offset_x)
        center_y_samples.append(pose.y - offset_y)
        center_z_samples.append(pose.z - offset.z)
    return CenterTarget(
        x=sum(center_x_samples) / len(center_x_samples),
        y=sum(center_y_samples) / len(center_y_samples),
        z=sum(center_z_samples) / len(center_z_samples),
        yaw=center_yaw,
    )


def desired_drone_pose(center: CenterTarget, offset: FormationOffset) -> LocalPose:
    offset_x, offset_y = rotate_xy(offset.x, offset.y, center.yaw)
    return LocalPose(
        x=center.x + offset_x,
        y=center.y + offset_y,
        z=center.z + offset.z,
        yaw=center.yaw + offset.yaw_rad,
    )


def interpolate_center_target(start: CenterTarget, action: SwarmScenarioAction, progress: float) -> CenterTarget:
    clamped_progress = max(min(progress, 1.0), 0.0)
    if action.kind == "hold":
        return start
    if action.kind == "translate_world":
        return CenterTarget(
            x=start.x + clamped_progress * action.delta_x,
            y=start.y + clamped_progress * action.delta_y,
            z=start.z + clamped_progress * action.delta_z,
            yaw=start.yaw,
        )
    if action.kind == "translate_body":
        delta_x, delta_y = rotate_xy(action.delta_x, action.delta_y, start.yaw)
        return CenterTarget(
            x=start.x + clamped_progress * delta_x,
            y=start.y + clamped_progress * delta_y,
            z=start.z + clamped_progress * action.delta_z,
            yaw=start.yaw,
        )
    if action.kind == "yaw":
        return CenterTarget(
            x=start.x,
            y=start.y,
            z=start.z,
            yaw=start.yaw + clamped_progress * action.delta_yaw_rad,
        )
    return start


def compute_body_frame_command(
    actual_pose: LocalPose,
    desired_pose: LocalPose,
    controller: SwarmControllerConfig,
) -> tuple[dict[str, float], dict[str, float]]:
    error_x_world = desired_pose.x - actual_pose.x
    error_y_world = desired_pose.y - actual_pose.y
    error_z = desired_pose.z - actual_pose.z
    horizontal_error_m = math.hypot(error_x_world, error_y_world)
    yaw_error_rad = angle_delta(desired_pose.yaw, actual_pose.yaw)
    body_x, body_y = world_to_body(error_x_world, error_y_world, actual_pose.yaw)

    command_x = 0.0 if abs(body_x) <= controller.position_deadband_m else clamp(controller.horizontal_gain * body_x, controller.max_linear_xy_mps)
    command_y = 0.0 if abs(body_y) <= controller.position_deadband_m else clamp(controller.horizontal_gain * body_y, controller.max_linear_xy_mps)
    command_z = 0.0 if abs(error_z) <= controller.vertical_deadband_m else clamp(controller.vertical_gain * error_z, controller.max_linear_z_mps)
    command_yaw = 0.0 if abs(yaw_error_rad) <= controller.yaw_deadband_rad else clamp(controller.yaw_gain * yaw_error_rad, controller.max_angular_z_rad_s)

    return (
        {
            "linear_x": command_x,
            "linear_y": command_y,
            "linear_z": command_z,
            "angular_z": command_yaw,
        },
        {
            "horizontal_error_m": horizontal_error_m,
            "vertical_error_m": abs(error_z),
            "yaw_error_deg": abs(math.degrees(yaw_error_rad)),
        },
    )
