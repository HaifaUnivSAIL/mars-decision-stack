#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from fleet_manifest import build_runtime_fleet
from vehicle_slice import (
    render_alate_config,
    render_ros_alate_config,
    render_ros_nemala_config,
)

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
    parser = argparse.ArgumentParser(description="Generate multi-drone runtime visual assets from a fleet manifest.")
    parser.add_argument("--fleet-config", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--source-model", required=True, type=Path)
    parser.add_argument("--world-template", required=True, type=Path)
    parser.add_argument("--root-dir", required=True, type=Path)
    return parser.parse_args()


def format_pose(values: tuple[float, float, float, float, float, float]) -> str:
    return " ".join(f"{value:.6f}" for value in values)


def load_xml(path: Path) -> ET.ElementTree:
    return ET.ElementTree(ET.fromstring(path.read_text()))


def write_xml(tree: ET.ElementTree, path: Path) -> None:
    xml_text = ET.tostring(tree.getroot(), encoding="unicode")
    path.write_text("<?xml version='1.0'?>\n" + xml_text + "\n")


def find_model(root: ET.Element) -> ET.Element:
    model = root.find("model")
    if model is None:
        raise RuntimeError("SDF is missing a top-level model element")
    return model


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


def find_sensor(root: ET.Element, sensor_name: str) -> ET.Element:
    sensor = root.find(f'.//sensor[@name="{sensor_name}"]')
    if sensor is None:
        raise RuntimeError(f"Failed to find sensor {sensor_name}")
    return sensor


def find_plugin(root: ET.Element, plugin_name: str) -> ET.Element:
    plugin = root.find(f'.//plugin[@name="{plugin_name}"]')
    if plugin is None:
        raise RuntimeError(f"Failed to find plugin {plugin_name}")
    return plugin


def remove_plugin(root: ET.Element, plugin_name: str) -> None:
    model = find_model(root)
    for plugin in list(model.findall("plugin")):
        if plugin.get("name") == plugin_name:
            model.remove(plugin)


def remove_sensor_plugin(root: ET.Element, sensor_name: str, plugin_name: str) -> None:
    sensor = find_sensor(root, sensor_name)
    for plugin in list(sensor.findall("plugin")):
        if plugin.get("name") == plugin_name:
            sensor.remove(plugin)


def apply_sensor_behavior(
    root: ET.Element,
    sensor_name: str,
    stream_settings: dict,
    behavior: dict,
) -> None:
    set_sensor_image_size(root, sensor_name, int(stream_settings["width"]), int(stream_settings["height"]))
    set_sensor_update_rate(root, sensor_name, float(stream_settings["update_rate_hz"]))
    set_sensor_visualize(root, sensor_name, bool(behavior["visualize"]))
    set_sensor_always_on(root, sensor_name, bool(behavior["always_on"]))
    for plugin_name in behavior.get("remove_plugins", []):
        remove_sensor_plugin(root, sensor_name, str(plugin_name))


def ensure_pose_element(element: ET.Element) -> ET.Element:
    pose_element = element.find("pose")
    if pose_element is None:
        pose_element = ET.Element("pose")
        element.insert(0, pose_element)
    return pose_element


def set_model_name(root: ET.Element, model_name: str) -> None:
    find_model(root).set("name", model_name)


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


def set_sensor_topic(root: ET.Element, sensor_name: str, topic_name: str) -> None:
    sensor = find_sensor(root, sensor_name)
    topic = sensor.find("topic")
    if topic is None:
        topic = ET.SubElement(sensor, "topic")
    topic.text = topic_name


def set_sensor_update_rate(root: ET.Element, sensor_name: str, rate_hz: float) -> None:
    sensor = find_sensor(root, sensor_name)
    update_rate = sensor.find("update_rate")
    if update_rate is None:
        update_rate = ET.SubElement(sensor, "update_rate")
    update_rate.text = f"{rate_hz:.3f}"


def set_sensor_visualize(root: ET.Element, sensor_name: str, visualize: bool) -> None:
    sensor = find_sensor(root, sensor_name)
    visualize_element = sensor.find("visualize")
    if visualize_element is None:
        visualize_element = ET.SubElement(sensor, "visualize")
    visualize_element.text = "true" if visualize else "false"


def set_sensor_always_on(root: ET.Element, sensor_name: str, always_on: bool) -> None:
    sensor = find_sensor(root, sensor_name)
    always_on_element = sensor.find("always_on")
    if always_on_element is None:
        always_on_element = ET.SubElement(sensor, "always_on")
    always_on_element.text = "1" if always_on else "0"


def set_sensor_image_size(root: ET.Element, sensor_name: str, width: int, height: int) -> None:
    sensor = find_sensor(root, sensor_name)
    image = sensor.find("./camera/image")
    if image is None:
        raise RuntimeError(f"Sensor {sensor_name} is missing <camera><image>")
    width_element = image.find("width")
    height_element = image.find("height")
    if width_element is None or height_element is None:
        raise RuntimeError(f"Sensor {sensor_name} is missing image width/height")
    width_element.text = str(width)
    height_element.text = str(height)


def set_world_light_cast_shadows(root: ET.Element, light_name: str, cast_shadows: bool) -> None:
    light = root.find(f'.//light[@name="{light_name}"]')
    if light is None:
        raise RuntimeError(f"Failed to find light {light_name}")
    cast_shadows_element = light.find("cast_shadows")
    if cast_shadows_element is None:
        cast_shadows_element = ET.SubElement(light, "cast_shadows")
    cast_shadows_element.text = "true" if cast_shadows else "false"


def set_plugin_text(root: ET.Element, plugin_name: str, element_name: str, value: str) -> None:
    plugin = find_plugin(root, plugin_name)
    element = plugin.find(element_name)
    if element is None:
        element = ET.SubElement(plugin, element_name)
    element.text = value


def remove_named_model_elements(root: ET.Element, tag_name: str, element_names: tuple[str, ...]) -> None:
    model = find_model(root)
    names = set(element_names)
    for element in list(model.findall(tag_name)):
        if element.get("name") in names:
            model.remove(element)


def build_model_config(model_name: str) -> str:
    return MODEL_CONFIG_TEMPLATE.format(
        display_name=f"{model_name} (Fleet Experiment)",
        description=(
            "Iris quadrotor with a rigid front-bottom deployed camera. "
            "This runtime model is generated for a multi-drone experiment world."
        ),
    )


def generate_runtime_model(source_model: Path, drone: dict, runtime_fleet: dict) -> ET.ElementTree:
    tree = load_xml(source_model)
    root = tree.getroot()
    set_model_name(root, drone["runtime_model_name"])

    remove_joint_position_controller_plugins(root, set(CALIBRATION_CONTROL_JOINTS))
    remove_named_model_elements(root, "joint", EXPERIMENT_REMOVED_JOINTS)
    remove_named_model_elements(root, "link", EXPERIMENT_REMOVED_LINKS)

    pose = drone["camera_deployment"]
    baked_pose = (pose["x"], pose["y"], pose["z"], pose["roll"], pose["pitch"], pose["yaw"])

    set_joint_type_and_pose(root, "deployed_camera_mount_joint", "fixed", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    set_joint_parent_child(root, "deployed_camera_mount_joint", "iris_with_standoffs::base_link", "deployed_camera_rigid_mount_link")
    set_link_pose(root, "deployed_camera_rigid_mount_link", baked_pose)
    set_link_pose(root, "deployed_camera_pod_link", baked_pose)
    set_link_pose(root, "deployed_camera_optical_link", baked_pose)
    set_joint_type_and_pose(root, "deployed_camera_pod_joint", "fixed", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    set_joint_type_and_pose(root, "deployed_camera_optical_joint", "fixed", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    set_sensor_topic(root, "camera", drone["camera_topics"]["deployed"])
    set_sensor_topic(root, "chase_camera", drone["camera_topics"]["chase"])
    set_plugin_text(root, "GstCameraPlugin", "udp_port", str(drone["gst_udp_port"]))

    deployed_stream = runtime_fleet["camera_streams"]["deployed"]
    chase_stream = runtime_fleet["camera_streams"]["chase"]
    sensor_behavior = runtime_fleet["sensor_behavior"]
    apply_sensor_behavior(root, "camera", deployed_stream, sensor_behavior["deployed"])
    apply_sensor_behavior(root, "chase_camera", chase_stream, sensor_behavior["chase"])

    ardupilot_plugin = find_plugin(root, "ArduPilotPlugin")
    fdm_port_in = ardupilot_plugin.find("fdm_port_in")
    if fdm_port_in is None:
        fdm_port_in = ET.SubElement(ardupilot_plugin, "fdm_port_in")
    fdm_port_in.text = str(drone["fdm_port_in"])

    fdm_exchange = runtime_fleet.get("fdm_exchange", {})
    for element_name in ("offline_recv_timeout_ms", "online_recv_timeout_ms"):
        value = fdm_exchange.get(element_name)
        if value is None:
            continue
        element = ardupilot_plugin.find(element_name)
        if element is None:
            element = ET.SubElement(ardupilot_plugin, element_name)
        element.text = str(int(value))
    return tree


def generate_runtime_world(template_path: Path, runtime_fleet: dict) -> ET.ElementTree:
    tree = load_xml(template_path)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError(f"World template {template_path} is missing a world element")

    physics = world.find("physics")
    if physics is not None:
        max_step_size = physics.find("max_step_size")
        if max_step_size is not None:
            max_step_size.text = f"{float(runtime_fleet['physics']['max_step_size']):.6f}"
        real_time_factor = physics.find("real_time_factor")
        if real_time_factor is not None:
            real_time_factor.text = f"{float(runtime_fleet['physics']['real_time_factor']):.6f}"
    set_world_light_cast_shadows(root, "sun", bool(runtime_fleet["rendering"]["sun_cast_shadows"]))

    for drone in runtime_fleet["drones"]:
        include = ET.SubElement(world, "include")
        uri = ET.SubElement(include, "uri")
        uri.text = f"model://{drone['runtime_model_name']}"
        name = ET.SubElement(include, "name")
        name.text = drone["runtime_model_name"]
        pose = ET.SubElement(include, "pose")
        pose.set("degrees", "true")
        pose.text = (
            f"{drone['spawn']['x']:.6f} {drone['spawn']['y']:.6f} {drone['spawn']['z']:.6f} "
            f"0 0 {drone['spawn']['yaw_deg']:.6f}"
        )

    return tree


def emit_manifest(output_dir: Path, runtime_fleet: dict, world_filename: str) -> None:
    drones_out: list[dict] = []
    for drone in runtime_fleet["drones"]:
        drones_out.append(
            {
                "id": drone["id"],
                "index": drone["index"],
                "namespace": drone["namespace"],
                "sitl_host": drone["sitl_host"],
                "runtime_model_name": drone["runtime_model_name"],
                "spawn": drone["spawn"],
                "serial0_port": drone["serial0_port"],
                "mavlink_port": drone["mavlink_port"],
                "mavlink_aux_port": drone["mavlink_aux_port"],
                "fdm_port_in": drone["fdm_port_in"],
                "fdm_port_out": drone["fdm_port_out"],
                "gst_udp_port": drone["gst_udp_port"],
                "proxy_name": drone["proxy_name"],
                "proxy_endpoints": drone["proxy_endpoints"],
                "camera_topics": drone["camera_topics"],
                "camera_deployment": drone["camera_deployment"],
                "alate_config_relative_path": drone["runtime_paths"]["alate"],
                "ros_alate_config_relative_path": drone["runtime_paths"]["ros_alate"],
                "ros_nemala_config_relative_path": drone["runtime_paths"]["ros_nemala"],
                "deployment_target_link_name": "deployed_camera_rigid_mount_link",
                "base_link_name": "iris_with_standoffs::base_link",
                "optical_link_name": "deployed_camera_optical_link",
            }
        )

    manifest = {
        "run_id": runtime_fleet["run_id"],
        "mode": runtime_fleet["mode"],
        "fleet": True,
        "manifest_path": runtime_fleet["manifest_path"],
        "runtime_profile": runtime_fleet["runtime_profile"],
        "physics": runtime_fleet["physics"],
        "fdm_exchange": runtime_fleet.get("fdm_exchange", {}),
        "camera_streams": runtime_fleet["camera_streams"],
        "rendering": runtime_fleet["rendering"],
        "sensor_behavior": runtime_fleet["sensor_behavior"],
        "runtime_world_name": runtime_fleet["runtime_world_name"],
        "runtime_world_relative_path": f"worlds/{world_filename}",
        "active_topics": runtime_fleet["active_topics"],
        "focus_topics": runtime_fleet["focus_topics"],
        "active_drone_id": runtime_fleet["active_drone_id"],
        "drones": drones_out,
    }
    (output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (output_dir / "deployment.applied.json").write_text(
        json.dumps({drone["id"]: drone["camera_deployment"] for drone in drones_out}, indent=2) + "\n"
    )
    env_lines = [
        f"export VISUAL_RUN_ID='{runtime_fleet['run_id']}'",
        "export VISUAL_MODE='experiment'",
        f"export VISUAL_RUNTIME_WORLD_NAME='{runtime_fleet['runtime_world_name']}'",
        f"export VISUAL_RUNTIME_WORLD_RELATIVE='worlds/{world_filename}'",
        f"export VISUAL_ACTIVE_CHASE_TOPIC='{runtime_fleet['active_topics']['chase']}'",
        f"export VISUAL_ACTIVE_CAMERA_TOPIC='{runtime_fleet['active_topics']['deployed']}'",
        f"export VISUAL_FOCUS_SELECT_TOPIC='{runtime_fleet['focus_topics']['select']}'",
        f"export VISUAL_FOCUS_STATE_TOPIC='{runtime_fleet['focus_topics']['state']}'",
        f"export VISUAL_ACTIVE_DRONE_ID='{runtime_fleet['active_drone_id']}'",
        "export VISUAL_FLEET=1",
        "export VISUAL_FLEET_DRONE_IDS='{}'".format(" ".join(drone["id"] for drone in drones_out)),
    ]
    (output_dir / "manifest.env").write_text("\n".join(env_lines) + "\n")

    focus_router_config = {
        "active_drone_id": runtime_fleet["active_drone_id"],
        "active_topics": runtime_fleet["active_topics"],
        "focus_topics": runtime_fleet["focus_topics"],
        "drones": [{"id": drone["id"], "camera_topics": drone["camera_topics"]} for drone in drones_out],
        "diagnostics": {
            "frame_events_csv": "../diagnostics/latency/active_camera_frames.csv",
            "focus_events_csv": "../diagnostics/latency/focus_events.csv",
            "summary_interval_sec": 5.0,
        },
    }
    (output_dir / "focus_router.json").write_text(json.dumps(focus_router_config, indent=2) + "\n")

    visual_latency_config = {
        "world_name": runtime_fleet["runtime_world_name"],
        "output_dir": "../diagnostics/latency",
        "focus_state_topic": runtime_fleet["focus_topics"]["state"],
        "pose_sample_period_sec": 0.05,
        "drones": [
            {
                "id": drone["id"],
                "runtime_model_name": drone["runtime_model_name"],
            }
            for drone in drones_out
        ],
    }
    (output_dir / "visual_latency_recorder.json").write_text(json.dumps(visual_latency_config, indent=2) + "\n")

    fleet_tsv = output_dir / "drones.tsv"
    with fleet_tsv.open("w") as handle:
        for drone in drones_out:
            handle.write(
                "\t".join(
                    [
                        drone["id"],
                        drone["namespace"],
                        drone["sitl_host"],
                        drone["runtime_model_name"],
                        str(drone["serial0_port"]),
                        str(drone["mavlink_port"]),
                        str(drone["mavlink_aux_port"]),
                        str(drone["fdm_port_in"]),
                        str(drone["fdm_port_out"]),
                        drone["proxy_name"],
                        drone["proxy_endpoints"]["subscribers"],
                        drone["proxy_endpoints"]["publishers"],
                        drone["proxy_endpoints"]["logger"],
                        drone["camera_topics"]["chase"],
                        drone["camera_topics"]["deployed"],
                        drone["alate_config_relative_path"],
                        drone["ros_alate_config_relative_path"],
                        drone["ros_nemala_config_relative_path"],
                    ]
                )
                + "\n"
            )


def main() -> int:
    args = parse_args()
    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    model_dir_root = output_dir / "models"
    world_dir_root = output_dir / "worlds"
    runtime_dir_root = output_dir / "runtime"
    model_dir_root.mkdir(parents=True, exist_ok=True)
    world_dir_root.mkdir(parents=True, exist_ok=True)
    runtime_dir_root.mkdir(parents=True, exist_ok=True)

    runtime_fleet = build_runtime_fleet(args.fleet_config, args.root_dir, args.run_id)

    for drone in runtime_fleet["drones"]:
        model_dir = model_dir_root / drone["runtime_model_name"]
        model_dir.mkdir(parents=True, exist_ok=True)
        model_tree = generate_runtime_model(args.source_model, drone, runtime_fleet)
        write_xml(model_tree, model_dir / "model.sdf")
        (model_dir / "model.config").write_text(build_model_config(drone["runtime_model_name"]))

        runtime_dir = runtime_dir_root / drone["id"]
        runtime_dir.mkdir(parents=True, exist_ok=True)
        render_alate_config(drone, runtime_dir / "uav.visual.sitl.json")
        render_ros_alate_config(drone, runtime_dir / "ros_alate.adapter.yaml")
        render_ros_nemala_config(drone, runtime_dir / "ros_nemala.node_manager.yaml")

    world_tree = generate_runtime_world(args.world_template, runtime_fleet)
    world_filename = f"mars_iris_dual_view.fleet.{args.run_id}.sdf"
    write_xml(world_tree, world_dir_root / world_filename)
    emit_manifest(output_dir, runtime_fleet, world_filename)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
