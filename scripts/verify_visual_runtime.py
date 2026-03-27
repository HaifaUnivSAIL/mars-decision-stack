#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Verify the live visual runtime against the generated manifest.")
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--pose-info", required=True, type=Path)
    parser.add_argument("--joint-state", required=True, type=Path)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--position-tol", type=float, default=0.01)
    parser.add_argument("--orientation-tol", type=float, default=0.02)
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
    match = re.search(
        rf"{section_name}\s*\{{(?P<body>.*?)\}}",
        block,
        re.S,
    )
    if not match:
        return (0.0, 0.0, 0.0)
    body = match.group("body")

    def component(name: str) -> float:
        value_match = re.search(rf"{name}:\s*([-+0-9.eE]+)", body)
        return float(value_match.group(1)) if value_match else 0.0

    return (component("x"), component("y"), component("z"))


def parse_xyzw(block: str, section_name: str) -> tuple[float, float, float, float]:
    match = re.search(
        rf"{section_name}\s*\{{(?P<body>.*?)\}}",
        block,
        re.S,
    )
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


def parse_pose_names(text: str) -> set[str]:
    names: set[str] = set()
    for block in parse_blocks(text, "pose"):
        name_match = re.search(r'name:\s*"([^"]+)"', block)
        if name_match:
            names.add(name_match.group(1))
    return names


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


def parse_axis_position(block: str) -> float | None:
    position_match = re.search(r"axis1\s*\{[\s\S]*?position:\s*([-+0-9.eE]+)", block)
    if not position_match:
        return None
    return float(position_match.group(1))


def parse_joint_state(text: str) -> tuple[str | None, dict[str, dict[str, float | tuple[float, ...] | None]]]:
    model_name_match = re.search(r'name:\s*"([^"]+)"', text)
    model_name = model_name_match.group(1) if model_name_match else None
    joints: dict[str, dict[str, float | tuple[float, ...] | None]] = {}
    for block in parse_blocks(text, "joint"):
        name_match = re.search(r'name:\s*"([^"]+)"', block)
        if not name_match:
            continue
        joints[name_match.group(1)] = {
            "position": parse_xyz(block, "position"),
            "orientation_xyzw": parse_xyzw(block, "orientation"),
            "axis_position": parse_axis_position(block),
        }
    return model_name, joints


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


def pose_name_present(names: set[str], model_name: str, target_name: str) -> bool:
    short_name = target_name.split("::")[-1]
    if target_name in names or short_name in names:
        return True
    prefix = f"{model_name}::{target_name}"
    return any(name == prefix or name.endswith(f"::{target_name}") or name.endswith(f"::{short_name}") for name in names)


def resolve_pose_entry(
    poses: dict[str, dict[str, tuple[float, ...]]],
    model_name: str,
    target_name: str,
) -> dict[str, tuple[float, ...]] | None:
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


def required_joint(
    joints: dict[str, dict[str, float | tuple[float, ...] | None]],
    joint_name: str,
) -> dict[str, float | tuple[float, ...] | None]:
    if joint_name not in joints:
        raise KeyError(f"Failed to find joint state for {joint_name}")
    return joints[joint_name]


def pose_to_mount_pose(pose_entry: dict[str, tuple[float, ...]] | None) -> dict[str, dict[str, float]] | None:
    if pose_entry is None:
        return None
    roll, pitch, yaw = rpy_from_quat(pose_entry["orientation_xyzw"])
    return {
        "position_m": {
            "x": float(pose_entry["position"][0]),
            "y": float(pose_entry["position"][1]),
            "z": float(pose_entry["position"][2]),
        },
        "orientation_rad": {
            "yaw": float(yaw),
            "pitch": float(pitch),
            "roll": float(roll),
        },
    }


def pose_errors(
    actual_pose: dict[str, dict[str, float]] | None,
    expected_pose: dict[str, dict[str, float]],
) -> tuple[dict[str, float], dict[str, float]]:
    if actual_pose is None:
        inf = float("inf")
        return (
            {"x": inf, "y": inf, "z": inf},
            {"yaw": inf, "pitch": inf, "roll": inf},
        )
    return (
        {
            "x": abs(actual_pose["position_m"]["x"] - expected_pose["position_m"]["x"]),
            "y": abs(actual_pose["position_m"]["y"] - expected_pose["position_m"]["y"]),
            "z": abs(actual_pose["position_m"]["z"] - expected_pose["position_m"]["z"]),
        },
        {
            "yaw": angle_error(actual_pose["orientation_rad"]["yaw"], expected_pose["orientation_rad"]["yaw"]),
            "pitch": angle_error(actual_pose["orientation_rad"]["pitch"], expected_pose["orientation_rad"]["pitch"]),
            "roll": angle_error(actual_pose["orientation_rad"]["roll"], expected_pose["orientation_rad"]["roll"]),
        },
    )


def build_report(args: argparse.Namespace) -> dict:
    manifest = json.loads(args.manifest.read_text())
    pose_info_text = args.pose_info.read_text()
    pose_names = parse_pose_names(pose_info_text)
    pose_entries = parse_pose_entries(pose_info_text)
    live_model_name, joints = parse_joint_state(args.joint_state.read_text())
    expected = manifest["deployment"]
    if live_model_name is None and manifest["runtime_model_name"] in pose_names:
        live_model_name = manifest["runtime_model_name"]

    actual_mount_pose = pose_to_mount_pose(resolve_pose_entry(pose_entries, manifest["runtime_model_name"], manifest["mount_link_name"]))
    actual_optical_pose = pose_to_mount_pose(resolve_pose_entry(pose_entries, manifest["runtime_model_name"], manifest["optical_link_name"]))
    link_position_errors, link_orientation_errors = pose_errors(actual_mount_pose, expected)
    optical_position_errors, optical_orientation_errors = pose_errors(actual_optical_pose, expected)

    report = {
        "mode": manifest["mode"],
        "runtime_model_name": manifest["runtime_model_name"],
        "live_model_name": live_model_name,
        "expected": expected,
        "actual_mount_pose": actual_mount_pose,
        "actual_optical_pose": actual_optical_pose,
        "position_errors": link_position_errors,
        "orientation_errors": link_orientation_errors,
        "optical_position_errors": optical_position_errors,
        "optical_orientation_errors": optical_orientation_errors,
        "pose_entities_present": {
            "runtime_model": manifest["runtime_model_name"] in pose_names,
            "base_link": pose_name_present(pose_names, manifest["runtime_model_name"], manifest["base_link_name"]),
            "mount_link": pose_name_present(pose_names, manifest["runtime_model_name"], manifest["mount_link_name"]),
            "optical_link": pose_name_present(pose_names, manifest["runtime_model_name"], manifest["optical_link_name"]),
        },
        "joint_state_pose": None,
        "joint_state_errors": None,
        "joint_axis_sanity": None,
    }

    if manifest["mode"] == "calib":
        x_joint = required_joint(joints, "deployed_camera_x_joint")
        y_joint = required_joint(joints, "deployed_camera_y_joint")
        z_joint = required_joint(joints, "deployed_camera_z_joint")
        yaw_joint = required_joint(joints, "deployed_camera_yaw_joint")
        pitch_joint = required_joint(joints, "deployed_camera_pitch_joint")
        roll_joint = required_joint(joints, "deployed_camera_roll_joint")

        yaw_roll, yaw_pitch, yaw_angle = rpy_from_quat(yaw_joint["orientation_xyzw"])
        pitch_roll, pitch_angle, pitch_yaw = rpy_from_quat(pitch_joint["orientation_xyzw"])
        roll_angle, roll_pitch, roll_yaw = rpy_from_quat(roll_joint["orientation_xyzw"])

        yaw_joint_axis = yaw_joint["axis_position"]
        pitch_joint_axis = pitch_joint["axis_position"]
        roll_joint_axis = roll_joint["axis_position"]

        joint_state_pose = {
            "position_m": {
                "x": float(x_joint["axis_position"] if x_joint["axis_position"] is not None else x_joint["position"][0]),
                "y": float(y_joint["axis_position"] if y_joint["axis_position"] is not None else y_joint["position"][1]),
                "z": float(z_joint["axis_position"] if z_joint["axis_position"] is not None else z_joint["position"][2]),
            },
            "orientation_rad": {
                "yaw": float(yaw_joint_axis if yaw_joint_axis is not None else yaw_angle),
                "pitch": float(pitch_joint_axis if pitch_joint_axis is not None else pitch_angle),
                "roll": float(roll_joint_axis if roll_joint_axis is not None else roll_angle),
            },
        }
        joint_position_errors, joint_orientation_errors = pose_errors(joint_state_pose, expected)
        report["joint_state_pose"] = joint_state_pose
        report["joint_state_errors"] = {
            "position": joint_position_errors,
            "orientation": joint_orientation_errors,
        }
        report["joint_axis_sanity"] = {
            "yaw_joint_cross_axis_rad": max(abs(yaw_roll), abs(yaw_pitch)),
            "pitch_joint_cross_axis_rad": max(abs(pitch_roll), abs(pitch_yaw)),
            "roll_joint_cross_axis_rad": max(abs(roll_pitch), abs(roll_yaw)),
        }

    return report


def main() -> int:
    args = parse_args()
    report = build_report(args)
    if args.report is not None:
        args.report.write_text(json.dumps(report, indent=2) + "\n")
    else:
        print(json.dumps(report, indent=2))

    ok = report["live_model_name"] == report["runtime_model_name"]
    ok = ok and all(report["pose_entities_present"].values())

    for error in report["position_errors"].values():
        ok = ok and error <= args.position_tol
    for error in report["orientation_errors"].values():
        ok = ok and error <= args.orientation_tol
    for error in report["optical_position_errors"].values():
        ok = ok and error <= args.position_tol
    for error in report["optical_orientation_errors"].values():
        ok = ok and error <= args.orientation_tol

    if report["mode"] == "calib":
        assert report["joint_state_errors"] is not None
        assert report["joint_axis_sanity"] is not None
        for error in report["joint_state_errors"]["position"].values():
            ok = ok and error <= args.position_tol
        for error in report["joint_state_errors"]["orientation"].values():
            ok = ok and error <= args.orientation_tol
        for error in report["joint_axis_sanity"].values():
            ok = ok and error <= args.orientation_tol

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
