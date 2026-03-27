#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path
import xml.etree.ElementTree as ET

BASE_RUNTIME_MODEL_NAME = "iris_with_camera"
CALIBRATION_CONTROL_JOINTS = (
    "deployed_camera_x_joint",
    "deployed_camera_y_joint",
    "deployed_camera_z_joint",
    "deployed_camera_yaw_joint",
    "deployed_camera_pitch_joint",
    "deployed_camera_roll_joint",
)
EXPERIMENT_REMOVED_JOINTS = (
    "deployed_camera_x_joint",
    "deployed_camera_y_joint",
    "deployed_camera_z_joint",
    "deployed_camera_mount_frame_joint",
    "deployed_camera_default_orientation_joint",
    "deployed_camera_yaw_joint",
    "deployed_camera_pitch_joint",
    "deployed_camera_roll_joint",
)
EXPERIMENT_REMOVED_LINKS = (
    "deployed_camera_default_translation_link",
    "deployed_camera_x_link",
    "deployed_camera_y_link",
    "deployed_camera_z_link",
    "deployed_camera_mount_frame_link",
    "deployed_camera_default_orientation_link",
    "deployed_camera_yaw_link",
    "deployed_camera_pitch_link",
)
CALIBRATION_LIMITS = {
    "x": (-0.10, 0.35),
    "y": (-0.20, 0.20),
    "z": (-0.35, 0.10),
    "yaw": (-math.pi, math.pi),
    "pitch": (-math.pi, math.pi),
    "roll": (-math.pi, math.pi),
}
MODEL_CONFIG_TEMPLATE = """<?xml version=\"1.0\"?>

<model>
  <name>{display_name}</name>
  <version>1.0</version>
  <sdf version=\"1.9\">model.sdf</sdf>

  <author>
    <name>MARS Decision Stack</name>
  </author>

  <description>
    {description}
  </description>
  <depend>
    <model>
      <uri>model://iris_with_standoffs</uri>
      <version>2.0</version>
    </model>
  </depend>
</model>
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate runtime visual assets from camera deployment config.")
    parser.add_argument("--mode", required=True, choices=("experiment", "calib"))
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--source-model", required=True, type=Path)
    parser.add_argument("--experiment-world-template", required=True, type=Path)
    parser.add_argument("--calibration-world-template", required=True, type=Path)
    return parser.parse_args()


def load_pose(config_path: Path) -> dict[str, float]:
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


def format_pose(values: tuple[float, float, float, float, float, float]) -> str:
    return " ".join(f"{value:.6f}" for value in values)


def safe_slug(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]+", "_", value)


def validate_calibration_pose(pose: dict[str, float]) -> None:
    out_of_range: list[str] = []
    for axis, (lower, upper) in CALIBRATION_LIMITS.items():
        value = pose[axis]
        if lower <= value <= upper:
            continue
        out_of_range.append(
            f"{axis}={value:.6f} is outside the calibration limit [{lower:.6f}, {upper:.6f}]"
        )
    if out_of_range:
        details = "; ".join(out_of_range)
        raise RuntimeError(
            "Calibration mode requires deployment values to stay within the live tuning joint limits: "
            f"{details}"
        )


def load_xml(path: Path) -> ET.ElementTree:
    return ET.ElementTree(ET.fromstring(path.read_text()))


def find_model(root: ET.Element) -> ET.Element:
    model = root.find("model")
    if model is None:
        raise RuntimeError("SDF is missing a top-level model element")
    return model


def set_model_name(root: ET.Element, model_name: str) -> None:
    model = find_model(root)
    model.set("name", model_name)


def find_joint(root: ET.Element, joint_name: str) -> ET.Element:
    joint = root.find(f'.//joint[@name="{joint_name}"]')
    if joint is None:
        raise RuntimeError(f"Failed to find joint {joint_name}")
    return joint


def find_link(root: ET.Element, link_name: str) -> ET.Element:
    link = root.find(f'.//link[@name="{link_name}"]')
    if link is None:
        raise RuntimeError(f"Failed to find link {link_name}")
    return link


def ensure_pose_element(element: ET.Element) -> ET.Element:
    pose_element = element.find("pose")
    if pose_element is None:
        pose_element = ET.Element("pose")
        element.insert(0, pose_element)
    return pose_element


def set_link_pose(root: ET.Element, link_name: str, pose_values: tuple[float, float, float, float, float, float]) -> None:
    link = find_link(root, link_name)
    ensure_pose_element(link).text = format_pose(pose_values)


def set_joint_parent_child(root: ET.Element, joint_name: str, parent_name: str, child_name: str) -> None:
    joint = find_joint(root, joint_name)
    parent_element = joint.find("parent")
    if parent_element is None:
        parent_element = ET.SubElement(joint, "parent")
    parent_element.text = parent_name

    child_element = joint.find("child")
    if child_element is None:
        child_element = ET.SubElement(joint, "child")
    child_element.text = child_name


def set_joint_type_and_pose(root: ET.Element, joint_name: str, joint_type: str, pose_values: tuple[float, float, float, float, float, float]) -> None:
    joint = find_joint(root, joint_name)
    joint.set("type", joint_type)
    ensure_pose_element(joint).text = format_pose(pose_values)


def remove_joint_position_controller_plugins(root: ET.Element, joint_names: set[str]) -> None:
    model = find_model(root)

    for plugin in list(model.findall("plugin")):
        if plugin.get("filename") != "gz-sim-joint-position-controller-system":
            continue
        joint_name_element = plugin.find("joint_name")
        if joint_name_element is None:
            continue
        if (joint_name_element.text or "").strip() in joint_names:
            model.remove(plugin)


def remove_named_model_elements(root: ET.Element, tag_name: str, element_names: tuple[str, ...]) -> None:
    model = find_model(root)
    names = set(element_names)
    for element in list(model.findall(tag_name)):
        if element.get("name") in names:
            model.remove(element)


def write_xml(tree: ET.ElementTree, path: Path) -> None:
    xml_text = ET.tostring(tree.getroot(), encoding="unicode")
    path.write_text("<?xml version='1.0'?>\n" + xml_text + "\n")


def build_model_config(model_name: str, mode: str) -> str:
    if mode == "experiment":
        description = (
            "Iris quadrotor with a rigid front-bottom deployed camera. "
            "The configured deployment is baked into this runtime model and no calibration controls are exposed."
        )
        display_name = f"{model_name} (Experiment)"
    else:
        description = (
            "Iris quadrotor with a rigid front-bottom deployed camera calibration rig. "
            "The visible payload is a fixed camera pod while helper joints expose 6-DOF tuning controls."
        )
        display_name = f"{model_name} (Calibration)"
    return MODEL_CONFIG_TEMPLATE.format(display_name=display_name, description=description)


def generate_runtime_model(source_model: Path, mode: str, pose: dict[str, float], runtime_model_name: str) -> ET.ElementTree:
    tree = load_xml(source_model)
    root = tree.getroot()
    set_model_name(root, runtime_model_name)

    if mode == "experiment":
        remove_joint_position_controller_plugins(root, set(CALIBRATION_CONTROL_JOINTS))
        remove_named_model_elements(root, "joint", EXPERIMENT_REMOVED_JOINTS)
        remove_named_model_elements(root, "link", EXPERIMENT_REMOVED_LINKS)

        set_joint_type_and_pose(root, "deployed_camera_mount_joint", "fixed", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        set_joint_parent_child(root, "deployed_camera_mount_joint", "iris_with_standoffs::base_link", "deployed_camera_rigid_mount_link")
        set_link_pose(
            root,
            "deployed_camera_rigid_mount_link",
            (pose["x"], pose["y"], pose["z"], pose["roll"], pose["pitch"], pose["yaw"]),
        )
        set_link_pose(
            root,
            "deployed_camera_pod_link",
            (pose["x"], pose["y"], pose["z"], pose["roll"], pose["pitch"], pose["yaw"]),
        )
        set_link_pose(
            root,
            "deployed_camera_optical_link",
            (pose["x"], pose["y"], pose["z"], pose["roll"], pose["pitch"], pose["yaw"]),
        )
        set_joint_type_and_pose(root, "deployed_camera_pod_joint", "fixed", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        set_joint_type_and_pose(root, "deployed_camera_optical_joint", "fixed", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    else:
        # Calibration uses live joint commands, so the source model stays zeroed.
        for joint_name in CALIBRATION_CONTROL_JOINTS:
            joint = find_joint(root, joint_name)
            if joint.find("pose") is not None:
                joint.find("pose").text = format_pose((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    return tree


def generate_runtime_world(template_path: Path, runtime_model_name: str, mode: str) -> ET.ElementTree:
    tree = load_xml(template_path)
    root = tree.getroot()

    include = None
    for candidate in root.findall(".//include"):
        uri_element = candidate.find("uri")
        if uri_element is not None and (uri_element.text or "").strip() == "model://iris_with_camera_calibration":
            include = candidate
            break
    if include is None:
        raise RuntimeError(f"Failed to find the runtime drone include in {template_path}")

    include.find("uri").text = f"model://{runtime_model_name}"

    if mode == "calib":
        plugin = root.find('.//plugin[@filename="JointPositionController"]')
        if plugin is None:
            raise RuntimeError(f"Calibration world template {template_path} is missing the JointPositionController GUI plugin")
        model_name = plugin.find("model_name")
        if model_name is None:
            model_name = ET.SubElement(plugin, "model_name")
        model_name.text = runtime_model_name

    return tree


def emit_manifest(output_dir: Path, mode: str, run_id: str, runtime_model_name: str, world_filename: str, pose: dict[str, float]) -> None:
    manifest = {
        "run_id": run_id,
        "mode": mode,
        "runtime_model_name": runtime_model_name,
        "runtime_model_uri": f"model://{runtime_model_name}",
        "runtime_world_name": "mars_iris_dual_view",
        "runtime_world_relative_path": f"worlds/{world_filename}",
        "runtime_model_relative_path": f"models/{runtime_model_name}/model.sdf",
        "camera_topic": "/mars/visual/deployed_camera",
        "chase_topic": "/mars/visual/chase_camera",
        "base_link_name": "iris_with_standoffs::base_link",
        "mount_link_name": "deployed_camera_rigid_mount_link",
        "optical_link_name": "deployed_camera_optical_link",
        "deployment_reference_link_name": "iris_with_standoffs::base_link",
        "deployment_target_link_name": "deployed_camera_rigid_mount_link",
        "deployment": {
            "position_m": {
                "x": pose["x"],
                "y": pose["y"],
                "z": pose["z"],
            },
            "orientation_rad": {
                "yaw": pose["yaw"],
                "pitch": pose["pitch"],
                "roll": pose["roll"],
            },
        },
        "calibration_joint_names": list(CALIBRATION_CONTROL_JOINTS) if mode == "calib" else [],
    }
    (output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")

    env_lines = [
        f"export VISUAL_RUN_ID='{run_id}'",
        f"export VISUAL_MODE='{mode}'",
        f"export VISUAL_RUNTIME_MODEL_NAME='{runtime_model_name}'",
        f"export VISUAL_RUNTIME_MODEL_URI='model://{runtime_model_name}'",
        f"export VISUAL_RUNTIME_WORLD_NAME='mars_iris_dual_view'",
        f"export VISUAL_RUNTIME_WORLD_RELATIVE='worlds/{world_filename}'",
        f"export VISUAL_RUNTIME_MODEL_RELATIVE='models/{runtime_model_name}/model.sdf'",
        "export VISUAL_BASE_LINK_NAME='iris_with_standoffs::base_link'",
        "export VISUAL_MOUNT_LINK_NAME='deployed_camera_rigid_mount_link'",
        "export VISUAL_OPTICAL_LINK_NAME='deployed_camera_optical_link'",
        "export VISUAL_CAMERA_TOPIC='/mars/visual/deployed_camera'",
        "export VISUAL_CHASE_TOPIC='/mars/visual/chase_camera'",
        f"export VISUAL_DEPLOYMENT_X='{pose['x']:.6f}'",
        f"export VISUAL_DEPLOYMENT_Y='{pose['y']:.6f}'",
        f"export VISUAL_DEPLOYMENT_Z='{pose['z']:.6f}'",
        f"export VISUAL_DEPLOYMENT_YAW='{pose['yaw']:.6f}'",
        f"export VISUAL_DEPLOYMENT_PITCH='{pose['pitch']:.6f}'",
        f"export VISUAL_DEPLOYMENT_ROLL='{pose['roll']:.6f}'",
    ]
    (output_dir / "manifest.env").write_text("\n".join(env_lines) + "\n")
    (output_dir / "deployment.applied.json").write_text(json.dumps(manifest["deployment"], indent=2) + "\n")


def main() -> int:
    args = parse_args()
    pose = load_pose(args.config)
    if args.mode == "calib":
        validate_calibration_pose(pose)
    output_dir = args.output_dir
    model_dir_root = output_dir / "models"
    world_dir_root = output_dir / "worlds"
    model_dir_root.mkdir(parents=True, exist_ok=True)
    world_dir_root.mkdir(parents=True, exist_ok=True)

    runtime_model_name = f"{BASE_RUNTIME_MODEL_NAME}_{args.mode}_{safe_slug(args.run_id)}"
    model_dir = model_dir_root / runtime_model_name
    model_dir.mkdir(parents=True, exist_ok=True)

    model_tree = generate_runtime_model(args.source_model, args.mode, pose, runtime_model_name)
    write_xml(model_tree, model_dir / "model.sdf")
    (model_dir / "model.config").write_text(build_model_config(runtime_model_name, args.mode))

    world_template = args.experiment_world_template if args.mode == "experiment" else args.calibration_world_template
    world_tree = generate_runtime_world(world_template, runtime_model_name, args.mode)
    world_filename = f"mars_iris_dual_view.{args.mode}.{safe_slug(args.run_id)}.sdf"
    write_xml(world_tree, world_dir_root / world_filename)

    emit_manifest(output_dir, args.mode, args.run_id, runtime_model_name, world_filename, pose)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
