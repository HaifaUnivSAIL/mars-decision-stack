#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Verify the live fleet visual runtime against the generated manifest.")
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--pose-info", required=True, type=Path)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--position-tol", type=float, default=0.02)
    parser.add_argument("--orientation-tol", type=float, default=0.04)
    return parser.parse_args()


def parse_blocks(text: str, block_name: str) -> list[str]:
    blocks: list[str] = []
    current: list[str] = []
    depth = 0
    target = f"{block_name} {{"
    for line in text.splitlines():
        stripped = line.strip()
        if depth == 0 and stripped == target:
            current = [line]
            depth = 1
            continue
        if depth > 0:
            current.append(line)
            depth += line.count("{")
            depth -= line.count("}")
            if depth == 0:
                blocks.append("\n".join(current))
                current = []
    return blocks


def parse_xyz(block: str, section_name: str) -> tuple[float, float, float]:
    match = re.search(rf"{section_name}\s*\{{(?P<body>.*?)\}}", block, re.S)
    if not match:
        return (0.0, 0.0, 0.0)
    body = match.group("body")

    def component(name: str) -> float:
        value_match = re.search(rf"{name}:\s*([-+0-9.eE]+)", body)
        return float(value_match.group(1)) if value_match else 0.0

    return (component("x"), component("y"), component("z"))


def parse_xyzw(block: str, section_name: str) -> tuple[float, float, float, float]:
    match = re.search(rf"{section_name}\s*\{{(?P<body>.*?)\}}", block, re.S)
    if not match:
        return (0.0, 0.0, 0.0, 1.0)
    body = match.group("body")

    def component(name: str, default: float) -> float:
        value_match = re.search(rf"{name}:\s*([-+0-9.eE]+)", body)
        return float(value_match.group(1)) if value_match else default

    return (
        component("x", 0.0),
        component("y", 0.0),
        component("z", 0.0),
        component("w", 1.0),
    )


def parse_pose_entries(text: str) -> dict[str, dict[str, tuple[float, ...]]]:
    poses: dict[str, dict[str, tuple[float, ...]]] = {}
    for block in parse_blocks(text, "pose"):
        name_match = re.search(r'name:\s*"([^"]+)"', block)
        if not name_match:
            continue
        poses[name_match.group(1)] = {
            "position": parse_xyz(block, "position"),
            "orientation_xyzw": parse_xyzw(block, "orientation"),
        }
    return poses


def resolve_pose_entry(poses: dict[str, dict[str, tuple[float, ...]]], model_name: str, target_name: str):
    short_name = target_name.split("::")[-1]
    candidate_names = (
        target_name,
        short_name,
        f"{model_name}::{target_name}",
        f"{model_name}::{short_name}",
    )
    for candidate in candidate_names:
        if candidate in poses:
            return poses[candidate]
    for pose_name, pose in poses.items():
        if pose_name.endswith(f"::{target_name}") or pose_name.endswith(f"::{short_name}"):
            return pose
    return None


def quat_conjugate(quat: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = quat
    return (-x, -y, -z, w)


def quat_multiply(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate(quat: tuple[float, float, float, float], vector: tuple[float, float, float]) -> tuple[float, float, float]:
    vec_quat = (vector[0], vector[1], vector[2], 0.0)
    rotated = quat_multiply(quat_multiply(quat, vec_quat), quat_conjugate(quat))
    return (rotated[0], rotated[1], rotated[2])


def rpy_from_quat(quat: tuple[float, float, float, float]) -> tuple[float, float, float]:
    x, y, z, w = quat
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def angle_error(actual: float, expected: float) -> float:
    delta = actual - expected
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return abs(delta)


def compute_relative_pose(base_pose, target_pose):
    base_pos = base_pose["position"]
    base_quat = base_pose["orientation_xyzw"]
    target_pos = target_pose["position"]
    target_quat = target_pose["orientation_xyzw"]
    delta_world = (
        target_pos[0] - base_pos[0],
        target_pos[1] - base_pos[1],
        target_pos[2] - base_pos[2],
    )
    inv_base = quat_conjugate(base_quat)
    rel_pos = quat_rotate(inv_base, delta_world)
    rel_quat = quat_multiply(inv_base, target_quat)
    roll, pitch, yaw = rpy_from_quat(rel_quat)
    return {
        "position_m": {"x": rel_pos[0], "y": rel_pos[1], "z": rel_pos[2]},
        "orientation_rad": {"yaw": yaw, "pitch": pitch, "roll": roll},
    }


def main() -> int:
    args = parse_args()
    manifest = json.loads(args.manifest.read_text())
    poses = parse_pose_entries(args.pose_info.read_text())

    report = {
        "runtime_world_name": manifest["runtime_world_name"],
        "active_drone_id": manifest["active_drone_id"],
        "drones": [],
        "ok": True,
    }

    for drone in manifest["drones"]:
        model_name = drone["runtime_model_name"]
        base_pose = resolve_pose_entry(poses, model_name, drone["base_link_name"])
        target_pose = resolve_pose_entry(poses, model_name, drone["deployment_target_link_name"])
        if base_pose is None or target_pose is None:
            report["ok"] = False
            report["drones"].append(
                {
                    "id": drone["id"],
                    "runtime_model_name": model_name,
                    "error": "Missing base or deployment target pose in dynamic_pose/info",
                }
            )
            continue

        actual = compute_relative_pose(base_pose, target_pose)
        expected = {
            "position_m": {
                "x": float(drone["camera_deployment"]["x"]),
                "y": float(drone["camera_deployment"]["y"]),
                "z": float(drone["camera_deployment"]["z"]),
            },
            "orientation_rad": {
                "yaw": float(drone["camera_deployment"]["yaw"]),
                "pitch": float(drone["camera_deployment"]["pitch"]),
                "roll": float(drone["camera_deployment"]["roll"]),
            },
        }
        position_errors = {
            axis: abs(actual["position_m"][axis] - expected["position_m"][axis])
            for axis in ("x", "y", "z")
        }
        orientation_errors = {
            axis: angle_error(actual["orientation_rad"][axis], expected["orientation_rad"][axis])
            for axis in ("yaw", "pitch", "roll")
        }
        ok = max(position_errors.values()) <= args.position_tol and max(orientation_errors.values()) <= args.orientation_tol
        report["ok"] = report["ok"] and ok
        report["drones"].append(
            {
                "id": drone["id"],
                "runtime_model_name": model_name,
                "expected": expected,
                "actual": actual,
                "position_errors": position_errors,
                "orientation_errors": orientation_errors,
                "ok": ok,
            }
        )

    if args.report is not None:
        args.report.write_text(json.dumps(report, indent=2) + "\n")
    if not report["ok"]:
        raise SystemExit(1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
