#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import queue
import re
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import TypeVar


ROOT_DIR = Path(__file__).resolve().parents[1]
LOG_DIR = ROOT_DIR / "logs"
RUNTIME_DIR = LOG_DIR / "runtime"
STACK_NAME = "mars-decision-stack"
VISUAL_CONTAINER = f"{STACK_NAME}-visual-sim"
DECISION_DEV_CONTAINER = f"{STACK_NAME}-decision-dev"
GZ_PARTITION = "mars-decision-stack-gazebo"
SampleT = TypeVar("SampleT")


@dataclass
class CommandSample:
    host_time_sec: float
    linear_x: float
    linear_y: float
    linear_z: float
    angular_x: float
    angular_y: float
    angular_z: float


@dataclass
class ImuSample:
    host_time_sec: float
    sim_time_sec: float
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    roll: float
    pitch: float
    yaw: float


@dataclass
class PoseSample:
    host_time_sec: float
    sim_time_sec: float
    pos_x: float
    pos_y: float
    pos_z: float
    roll: float
    pitch: float
    yaw: float


@dataclass
class ActuatorSample:
    host_time_sec: float
    time_usec: float
    servo1_raw: float
    servo2_raw: float
    servo3_raw: float
    servo4_raw: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture the future-policy-equivalent velocity input topic and align it "
            "with Gazebo IMU acceleration from the current visual stack."
        )
    )
    parser.add_argument("--duration", type=float, default=20.0, help="Capture duration in seconds.")
    parser.add_argument("--drone", help="Optional drone id when the current visual stack is running in fleet mode.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Optional output directory. Defaults under the current visual run directory.",
    )
    return parser.parse_args()


ACTUATOR_CAPTURE_CODE_TEMPLATE = """
import sys
import time
from pymavlink import mavutil

duration = float(sys.argv[1])
m = mavutil.mavlink_connection(sys.argv[2], timeout=5)
hb = m.wait_heartbeat(timeout=10)
if hb is None:
    raise SystemExit(f"No heartbeat on {sys.argv[2]}")

for _ in range(3):
    m.mav.request_data_stream_send(
        m.target_system,
        m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
        10,
        1,
    )
    m.mav.command_long_send(
        m.target_system,
        m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
        100000,
        0, 0, 0, 0, 0,
    )
    time.sleep(0.2)

deadline = time.time() + duration + 2.0
while time.time() < deadline:
    msg = m.recv_match(type=["SERVO_OUTPUT_RAW"], blocking=True, timeout=1)
    if msg is None:
        m.mav.command_long_send(
            m.target_system,
            m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
            100000,
            0, 0, 0, 0, 0,
        )
        continue
    data = msg.to_dict()
    print(
        f"{time.time():.6f},"
        f"{int(data.get('time_usec', 0))},"
        f"{int(data.get('servo1_raw', 0))},"
        f"{int(data.get('servo2_raw', 0))},"
        f"{int(data.get('servo3_raw', 0))},"
        f"{int(data.get('servo4_raw', 0))}",
        flush=True,
    )
"""


def load_runtime_manifest() -> dict:
    current_run_path = RUNTIME_DIR / "visual.current_run_dir"
    if not current_run_path.exists():
        raise RuntimeError("Missing logs/runtime/visual.current_run_dir. Start the visual stack first.")
    current_run_dir = Path(current_run_path.read_text().strip())
    manifest_path = current_run_dir / "visual_assets" / "manifest.json"
    if not manifest_path.exists():
        raise RuntimeError(f"Missing runtime manifest: {manifest_path}")
    manifest = json.loads(manifest_path.read_text())
    manifest["current_run_dir"] = str(current_run_dir)
    return manifest


def resolve_runtime_target(manifest: dict, requested_drone: str) -> dict:
    requested_drone = (requested_drone or "").strip()
    if manifest.get("fleet"):
        drones = manifest.get("drones", [])
        if not drones:
            raise RuntimeError("Fleet runtime manifest does not define any drones")
        by_id = {drone["id"]: drone for drone in drones}
        selected_id = requested_drone or manifest.get("active_drone_id") or drones[0]["id"]
        if selected_id not in by_id:
            raise RuntimeError(f"Unknown drone id {selected_id!r}. Available: {sorted(by_id)}")
        drone = by_id[selected_id]
        namespace = drone.get("namespace") or f"/{selected_id}"
        return {
            "drone_id": selected_id,
            "namespace": namespace,
            "runtime_model_name": drone["runtime_model_name"],
            "mavlink_port": int(drone["mavlink_port"]),
            "mavlink_endpoint": f"tcp:{drone['sitl_host']}:{int(drone['mavlink_port'])}",
            "command_topic": f"{namespace}/alate_input_velocity",
        }

    return {
        "drone_id": requested_drone,
        "namespace": "",
        "runtime_model_name": manifest["runtime_model_name"],
        "mavlink_port": 5763,
        "mavlink_endpoint": "tcp:sitl:5763",
        "command_topic": "/alate_input_velocity",
    }


def container_is_running(container_name: str) -> bool:
    result = subprocess.run(
        ["/bin/bash", "-lc", "docker ps --format '{{.Names}}'"],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f"Failed to query docker containers: {result.stderr.strip()}")
    return container_name in result.stdout.splitlines()


def rpy_from_quat(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
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


def parse_numeric_field(block: str, field_name: str, default: float = 0.0) -> float:
    match = re.search(rf"{field_name}:\s*([-+0-9.eE]+)", block)
    if match is None:
        return default
    return float(match.group(1))


def extract_section(block: str, section_name: str) -> str:
    match = re.search(rf"{section_name}\s*\{{(?P<body>.*?)\}}", block, re.S)
    if match is None:
        return ""
    return match.group("body")


def extract_repeated_blocks(message_text: str, block_name: str) -> list[str]:
    blocks: list[str] = []
    current_lines: list[str] = []
    collecting = False
    depth = 0

    for raw_line in message_text.splitlines(keepends=True):
        stripped = raw_line.strip()
        if not collecting:
            if stripped != f"{block_name} {{":
                continue
            collecting = True
            current_lines = [raw_line]
            depth = raw_line.count("{") - raw_line.count("}")
            continue

        current_lines.append(raw_line)
        depth += raw_line.count("{") - raw_line.count("}")
        if depth == 0:
            blocks.append("".join(current_lines))
            current_lines = []
            collecting = False

    return blocks


def parse_imu_message(message_text: str, host_time_sec: float) -> ImuSample:
    stamp_body = extract_section(message_text, "stamp")
    sim_sec = parse_numeric_field(stamp_body, "sec", 0.0)
    sim_nsec = parse_numeric_field(stamp_body, "nsec", 0.0)

    linear_body = extract_section(message_text, "linear_acceleration")
    angular_body = extract_section(message_text, "angular_velocity")
    orientation_body = extract_section(message_text, "orientation")

    qx = parse_numeric_field(orientation_body, "x")
    qy = parse_numeric_field(orientation_body, "y")
    qz = parse_numeric_field(orientation_body, "z")
    qw = parse_numeric_field(orientation_body, "w", 1.0)
    roll, pitch, yaw = rpy_from_quat(qx, qy, qz, qw)

    return ImuSample(
        host_time_sec=host_time_sec,
        sim_time_sec=sim_sec + sim_nsec / 1e9,
        accel_x=parse_numeric_field(linear_body, "x"),
        accel_y=parse_numeric_field(linear_body, "y"),
        accel_z=parse_numeric_field(linear_body, "z"),
        gyro_x=parse_numeric_field(angular_body, "x"),
        gyro_y=parse_numeric_field(angular_body, "y"),
        gyro_z=parse_numeric_field(angular_body, "z"),
        roll=roll,
        pitch=pitch,
        yaw=yaw,
    )


def parse_pose_message(message_text: str, host_time_sec: float, runtime_model_name: str) -> PoseSample | None:
    stamp_body = extract_section(message_text, "stamp")
    sim_sec = parse_numeric_field(stamp_body, "sec", 0.0)
    sim_nsec = parse_numeric_field(stamp_body, "nsec", 0.0)

    for pose_block in extract_repeated_blocks(message_text, "pose"):
        name_match = re.search(r'name:\s*"([^"]+)"', pose_block)
        if name_match is None or name_match.group(1) != runtime_model_name:
            continue

        position_body = extract_section(pose_block, "position")
        orientation_body = extract_section(pose_block, "orientation")
        qx = parse_numeric_field(orientation_body, "x")
        qy = parse_numeric_field(orientation_body, "y")
        qz = parse_numeric_field(orientation_body, "z")
        qw = parse_numeric_field(orientation_body, "w", 1.0)
        roll, pitch, yaw = rpy_from_quat(qx, qy, qz, qw)

        return PoseSample(
            host_time_sec=host_time_sec,
            sim_time_sec=sim_sec + sim_nsec / 1e9,
            pos_x=parse_numeric_field(position_body, "x"),
            pos_y=parse_numeric_field(position_body, "y"),
            pos_z=parse_numeric_field(position_body, "z"),
            roll=roll,
            pitch=pitch,
            yaw=yaw,
        )

    return None


def start_command_capture(duration_sec: float, command_topic: str) -> subprocess.Popen[str]:
    cmd = (
        "docker exec "
        f"{DECISION_DEV_CONTAINER} "
        "bash -lc "
        "'source /opt/ros/humble/setup.bash && "
        "source /opt/ros2_ws/install/setup.bash && "
        f"timeout {max(duration_sec + 2.0, 5.0):.1f} ros2 topic echo --csv {command_topic}'"
    )
    return subprocess.Popen(
        ["/bin/bash", "-lc", cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )


def start_imu_capture(duration_sec: float, imu_topic: str) -> subprocess.Popen[str]:
    cmd = (
        "docker exec "
        f"-u {os.getuid()}:{os.getgid()} "
        f"-e GZ_PARTITION={GZ_PARTITION} "
        f"{VISUAL_CONTAINER} "
        "bash -lc "
        f"\"timeout {max(duration_sec + 2.0, 5.0):.1f} gz topic -e -t {imu_topic}\""
    )
    return subprocess.Popen(
        ["/bin/bash", "-lc", cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )


def start_pose_capture(duration_sec: float, pose_topic: str) -> subprocess.Popen[str]:
    cmd = (
        "docker exec "
        f"-u {os.getuid()}:{os.getgid()} "
        f"-e GZ_PARTITION={GZ_PARTITION} "
        f"{VISUAL_CONTAINER} "
        "bash -lc "
        f"\"timeout {max(duration_sec + 2.0, 5.0):.1f} gz topic -e -t {pose_topic}\""
    )
    return subprocess.Popen(
        ["/bin/bash", "-lc", cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )


def start_actuator_capture(duration_sec: float, mavlink_endpoint: str) -> subprocess.Popen[str]:
    return subprocess.Popen(
        [
            "docker",
            "exec",
            DECISION_DEV_CONTAINER,
            "python3",
            "-u",
            "-c",
            ACTUATOR_CAPTURE_CODE_TEMPLATE,
            f"{duration_sec:.1f}",
            mavlink_endpoint,
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )


def command_reader(proc: subprocess.Popen[str], out_queue: queue.Queue[CommandSample]) -> None:
    assert proc.stdout is not None
    for raw_line in proc.stdout:
        line = raw_line.strip()
        if not line:
            continue
        fields = [field.strip() for field in line.split(",")]
        if len(fields) != 6:
            continue
        try:
            values = [float(field) for field in fields]
        except ValueError:
            continue
        out_queue.put(CommandSample(time.time(), *values))


def actuator_reader(proc: subprocess.Popen[str], out_queue: queue.Queue[ActuatorSample]) -> None:
    assert proc.stdout is not None
    for raw_line in proc.stdout:
        line = raw_line.strip()
        if not line:
            continue
        fields = [field.strip() for field in line.split(",")]
        if len(fields) != 6:
            continue
        try:
            values = [float(field) for field in fields]
        except ValueError:
            continue
        out_queue.put(ActuatorSample(*values))


def imu_reader(proc: subprocess.Popen[str], out_queue: queue.Queue[ImuSample]) -> None:
    assert proc.stdout is not None
    current_lines: list[str] = []
    collecting = False

    for raw_line in proc.stdout:
        stripped = raw_line.strip()
        if not collecting:
            if stripped != "header {":
                continue
            collecting = True
            current_lines = [raw_line]
            continue

        if not stripped:
            host_time_sec = time.time()
            try:
                out_queue.put(parse_imu_message("".join(current_lines), host_time_sec))
            except Exception:
                pass
            current_lines = []
            collecting = False
            continue

        current_lines.append(raw_line)

    if collecting and current_lines:
        host_time_sec = time.time()
        try:
            out_queue.put(parse_imu_message("".join(current_lines), host_time_sec))
        except Exception:
            pass


def pose_reader(
    proc: subprocess.Popen[str], out_queue: queue.Queue[PoseSample], runtime_model_name: str
) -> None:
    assert proc.stdout is not None
    current_lines: list[str] = []
    collecting = False

    for raw_line in proc.stdout:
        stripped = raw_line.strip()
        if not collecting:
            if stripped != "header {":
                continue
            collecting = True
            current_lines = [raw_line]
            continue

        if not stripped:
            host_time_sec = time.time()
            try:
                sample = parse_pose_message("".join(current_lines), host_time_sec, runtime_model_name)
                if sample is not None:
                    out_queue.put(sample)
            except Exception:
                pass
            current_lines = []
            collecting = False
            continue

        current_lines.append(raw_line)

    if collecting and current_lines:
        host_time_sec = time.time()
        try:
            sample = parse_pose_message("".join(current_lines), host_time_sec, runtime_model_name)
            if sample is not None:
                out_queue.put(sample)
        except Exception:
            pass


def drain_queue(sample_queue: queue.Queue[SampleT]) -> list[SampleT]:
    samples: list[SampleT] = []
    while True:
        try:
            samples.append(sample_queue.get_nowait())
        except queue.Empty:
            break
    return samples


def write_command_csv(path: Path, samples: list[CommandSample]) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ["host_time_sec", "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"]
        )
        for sample in samples:
            writer.writerow(
                [
                    f"{sample.host_time_sec:.6f}",
                    f"{sample.linear_x:.6f}",
                    f"{sample.linear_y:.6f}",
                    f"{sample.linear_z:.6f}",
                    f"{sample.angular_x:.6f}",
                    f"{sample.angular_y:.6f}",
                    f"{sample.angular_z:.6f}",
                ]
            )


def write_imu_csv(path: Path, samples: list[ImuSample]) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "host_time_sec",
                "sim_time_sec",
                "accel_x",
                "accel_y",
                "accel_z",
                "gyro_x",
                "gyro_y",
                "gyro_z",
                "roll_rad",
                "pitch_rad",
                "yaw_rad",
            ]
        )
        for sample in samples:
            writer.writerow(
                [
                    f"{sample.host_time_sec:.6f}",
                    f"{sample.sim_time_sec:.6f}",
                    f"{sample.accel_x:.6f}",
                    f"{sample.accel_y:.6f}",
                    f"{sample.accel_z:.6f}",
                    f"{sample.gyro_x:.6f}",
                    f"{sample.gyro_y:.6f}",
                    f"{sample.gyro_z:.6f}",
                    f"{sample.roll:.6f}",
                    f"{sample.pitch:.6f}",
                    f"{sample.yaw:.6f}",
                ]
            )


def write_pose_csv(path: Path, samples: list[PoseSample]) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "host_time_sec",
                "sim_time_sec",
                "pos_x",
                "pos_y",
                "pos_z",
                "roll_rad",
                "pitch_rad",
                "yaw_rad",
            ]
        )
        for sample in samples:
            writer.writerow(
                [
                    f"{sample.host_time_sec:.6f}",
                    f"{sample.sim_time_sec:.6f}",
                    f"{sample.pos_x:.6f}",
                    f"{sample.pos_y:.6f}",
                    f"{sample.pos_z:.6f}",
                    f"{sample.roll:.6f}",
                    f"{sample.pitch:.6f}",
                    f"{sample.yaw:.6f}",
                ]
            )


def write_actuator_csv(path: Path, samples: list[ActuatorSample]) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "host_time_sec",
                "time_usec",
                "servo1_raw",
                "servo2_raw",
                "servo3_raw",
                "servo4_raw",
            ]
        )
        for sample in samples:
            writer.writerow(
                [
                    f"{sample.host_time_sec:.6f}",
                    f"{sample.time_usec:.0f}",
                    f"{sample.servo1_raw:.0f}",
                    f"{sample.servo2_raw:.0f}",
                    f"{sample.servo3_raw:.0f}",
                    f"{sample.servo4_raw:.0f}",
                ]
            )


def merge_command_and_imu(command_samples: list[CommandSample], imu_samples: list[ImuSample]) -> list[dict[str, float]]:
    merged: list[dict[str, float]] = []
    command_samples = sorted(command_samples, key=lambda sample: sample.host_time_sec)
    imu_samples = sorted(imu_samples, key=lambda sample: sample.host_time_sec)
    command_index = 0
    last_command = CommandSample(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    for imu_sample in imu_samples:
        while command_index < len(command_samples) and command_samples[command_index].host_time_sec <= imu_sample.host_time_sec:
            last_command = command_samples[command_index]
            command_index += 1
        merged.append(
            {
                "host_time_sec": imu_sample.host_time_sec,
                "sim_time_sec": imu_sample.sim_time_sec,
                "cmd_linear_x": last_command.linear_x,
                "cmd_linear_y": last_command.linear_y,
                "cmd_linear_z": last_command.linear_z,
                "imu_accel_x": imu_sample.accel_x,
                "imu_accel_y": imu_sample.accel_y,
                "imu_accel_z": imu_sample.accel_z,
                "imu_roll_rad": imu_sample.roll,
                "imu_pitch_rad": imu_sample.pitch,
                "imu_yaw_rad": imu_sample.yaw,
                "imu_gyro_x": imu_sample.gyro_x,
                "imu_gyro_y": imu_sample.gyro_y,
                "imu_gyro_z": imu_sample.gyro_z,
            }
        )
    return merged


def merge_command_and_actuator(
    command_samples: list[CommandSample], actuator_samples: list[ActuatorSample]
) -> list[dict[str, float]]:
    merged: list[dict[str, float]] = []
    command_samples = sorted(command_samples, key=lambda sample: sample.host_time_sec)
    actuator_samples = sorted(actuator_samples, key=lambda sample: sample.host_time_sec)
    command_index = 0
    last_command = CommandSample(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    for actuator_sample in actuator_samples:
        while command_index < len(command_samples) and command_samples[command_index].host_time_sec <= actuator_sample.host_time_sec:
            last_command = command_samples[command_index]
            command_index += 1
        merged.append(
            {
                "host_time_sec": actuator_sample.host_time_sec,
                "time_usec": actuator_sample.time_usec,
                "cmd_linear_x": last_command.linear_x,
                "cmd_linear_y": last_command.linear_y,
                "cmd_linear_z": last_command.linear_z,
                "servo1_raw": actuator_sample.servo1_raw,
                "servo2_raw": actuator_sample.servo2_raw,
                "servo3_raw": actuator_sample.servo3_raw,
                "servo4_raw": actuator_sample.servo4_raw,
            }
        )
    return merged


def rotate_world_to_body(
    world_x: float, world_y: float, world_z: float, roll: float, pitch: float, yaw: float
) -> tuple[float, float, float]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    r00 = cy * cp
    r01 = cy * sp * sr - sy * cr
    r02 = cy * sp * cr + sy * sr
    r10 = sy * cp
    r11 = sy * sp * sr + cy * cr
    r12 = sy * sp * cr - cy * sr
    r20 = -sp
    r21 = cp * sr
    r22 = cp * cr

    body_x = r00 * world_x + r10 * world_y + r20 * world_z
    body_y = r01 * world_x + r11 * world_y + r21 * world_z
    body_z = r02 * world_x + r12 * world_y + r22 * world_z
    return body_x, body_y, body_z


def derive_pose_kinematics(pose_samples: list[PoseSample]) -> list[dict[str, float]]:
    samples = sorted(pose_samples, key=lambda sample: sample.host_time_sec)
    if len(samples) < 3:
        return []

    velocities: list[tuple[float, float, float]] = [(0.0, 0.0, 0.0)] * len(samples)
    accelerations: list[tuple[float, float, float]] = [(0.0, 0.0, 0.0)] * len(samples)

    for index in range(1, len(samples)):
        dt = samples[index].host_time_sec - samples[index - 1].host_time_sec
        if dt <= 1e-6:
            velocities[index] = velocities[index - 1]
            continue
        velocities[index] = (
            (samples[index].pos_x - samples[index - 1].pos_x) / dt,
            (samples[index].pos_y - samples[index - 1].pos_y) / dt,
            (samples[index].pos_z - samples[index - 1].pos_z) / dt,
        )

    for index in range(1, len(samples)):
        dt = samples[index].host_time_sec - samples[index - 1].host_time_sec
        if dt <= 1e-6:
            accelerations[index] = accelerations[index - 1]
            continue
        accel_world = (
            (velocities[index][0] - velocities[index - 1][0]) / dt,
            (velocities[index][1] - velocities[index - 1][1]) / dt,
            (velocities[index][2] - velocities[index - 1][2]) / dt,
        )
        accelerations[index] = rotate_world_to_body(
            accel_world[0],
            accel_world[1],
            accel_world[2],
            samples[index].roll,
            samples[index].pitch,
            samples[index].yaw,
        )

    rows: list[dict[str, float]] = []
    for index in range(len(samples)):
        rows.append(
            {
                "host_time_sec": samples[index].host_time_sec,
                "sim_time_sec": samples[index].sim_time_sec,
                "pose_x": samples[index].pos_x,
                "pose_y": samples[index].pos_y,
                "pose_z": samples[index].pos_z,
                "pose_roll_rad": samples[index].roll,
                "pose_pitch_rad": samples[index].pitch,
                "pose_yaw_rad": samples[index].yaw,
                "vel_world_x": velocities[index][0],
                "vel_world_y": velocities[index][1],
                "vel_world_z": velocities[index][2],
                "accel_body_x": accelerations[index][0],
                "accel_body_y": accelerations[index][1],
                "accel_body_z": accelerations[index][2],
            }
        )
    return rows


def merge_command_and_pose(command_samples: list[CommandSample], pose_rows: list[dict[str, float]]) -> list[dict[str, float]]:
    merged: list[dict[str, float]] = []
    command_samples = sorted(command_samples, key=lambda sample: sample.host_time_sec)
    pose_rows = sorted(pose_rows, key=lambda row: row["host_time_sec"])
    command_index = 0
    last_command = CommandSample(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    for pose_row in pose_rows:
        while command_index < len(command_samples) and command_samples[command_index].host_time_sec <= pose_row["host_time_sec"]:
            last_command = command_samples[command_index]
            command_index += 1
        merged.append(
            {
                "host_time_sec": pose_row["host_time_sec"],
                "sim_time_sec": pose_row["sim_time_sec"],
                "cmd_linear_x": last_command.linear_x,
                "cmd_linear_y": last_command.linear_y,
                "cmd_linear_z": last_command.linear_z,
                **pose_row,
            }
        )
    return merged


def write_merged_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        path.write_text("")
        return
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        for row in rows:
            formatted: dict[str, str] = {}
            for key, value in row.items():
                if isinstance(value, (int, float)):
                    formatted[key] = f"{value:.6f}"
                else:
                    formatted[key] = str(value)
            writer.writerow(formatted)


def summarize(rows: list[dict[str, float]]) -> dict[str, float]:
    summary: dict[str, float] = {
        "imu_samples": float(len(rows)),
        "mean_accel_x_when_cmd_x_pos": 0.0,
        "mean_accel_x_when_cmd_x_neg": 0.0,
        "mean_accel_y_when_cmd_y_pos": 0.0,
        "mean_accel_y_when_cmd_y_neg": 0.0,
    }

    def mean_for(condition_key: str, accel_key: str, predicate) -> float:
        values = [row[accel_key] for row in rows if predicate(row[condition_key])]
        if not values:
            return 0.0
        return sum(values) / len(values)

    summary["mean_accel_x_when_cmd_x_pos"] = mean_for("cmd_linear_x", "imu_accel_x", lambda value: value > 1e-6)
    summary["mean_accel_x_when_cmd_x_neg"] = mean_for("cmd_linear_x", "imu_accel_x", lambda value: value < -1e-6)
    summary["mean_accel_y_when_cmd_y_pos"] = mean_for("cmd_linear_y", "imu_accel_y", lambda value: value > 1e-6)
    summary["mean_accel_y_when_cmd_y_neg"] = mean_for("cmd_linear_y", "imu_accel_y", lambda value: value < -1e-6)
    return summary


def summarize_pose(rows: list[dict[str, float]]) -> dict[str, float]:
    summary: dict[str, float] = {
        "pose_samples": float(len(rows)),
        "mean_body_accel_x_when_cmd_x_pos": 0.0,
        "mean_body_accel_x_when_cmd_x_neg": 0.0,
        "mean_body_accel_y_when_cmd_y_pos": 0.0,
        "mean_body_accel_y_when_cmd_y_neg": 0.0,
    }

    def mean_for(condition_key: str, accel_key: str, predicate) -> float:
        values = [row[accel_key] for row in rows if predicate(row[condition_key])]
        if not values:
            return 0.0
        return sum(values) / len(values)

    summary["mean_body_accel_x_when_cmd_x_pos"] = mean_for("cmd_linear_x", "accel_body_x", lambda value: value > 1e-6)
    summary["mean_body_accel_x_when_cmd_x_neg"] = mean_for("cmd_linear_x", "accel_body_x", lambda value: value < -1e-6)
    summary["mean_body_accel_y_when_cmd_y_pos"] = mean_for("cmd_linear_y", "accel_body_y", lambda value: value > 1e-6)
    summary["mean_body_accel_y_when_cmd_y_neg"] = mean_for("cmd_linear_y", "accel_body_y", lambda value: value < -1e-6)
    return summary


def find_active_windows(rows: list[dict[str, float]]) -> list[list[dict[str, float]]]:
    windows: list[list[dict[str, float]]] = []
    current: list[dict[str, float]] = []

    for row in rows:
        is_active = any(abs(row[key]) > 1e-6 for key in ("cmd_linear_x", "cmd_linear_y", "cmd_linear_z"))
        if is_active:
            current.append(row)
            continue
        if current:
            windows.append(current)
            current = []

    if current:
        windows.append(current)
    return windows


def build_comparison_rows(
    merged_imu_rows: list[dict[str, float]],
    merged_pose_rows: list[dict[str, float]],
    actuator_rows: list[dict[str, float]],
) -> list[dict[str, float | str]]:
    comparison_rows: list[dict[str, float | str]] = []
    for window_index, pose_window in enumerate(find_active_windows(merged_pose_rows), start=1):
        start_time = pose_window[0]["host_time_sec"]
        end_time = pose_window[-1]["host_time_sec"]
        imu_window = [row for row in merged_imu_rows if start_time <= row["host_time_sec"] <= end_time]
        baseline_actuator_window = [
            row for row in actuator_rows if max(0.0, start_time - 0.75) <= row["host_time_sec"] < start_time
        ]
        active_actuator_window = [row for row in actuator_rows if start_time <= row["host_time_sec"] <= end_time]
        if not baseline_actuator_window and actuator_rows:
            baseline_actuator_window = actuator_rows[: min(5, len(actuator_rows))]

        mean_cmd_x = sum(row["cmd_linear_x"] for row in pose_window) / len(pose_window)
        mean_cmd_y = sum(row["cmd_linear_y"] for row in pose_window) / len(pose_window)
        mean_cmd_z = sum(row["cmd_linear_z"] for row in pose_window) / len(pose_window)
        dominant_axis = max(
            (("x", mean_cmd_x), ("y", mean_cmd_y), ("z", mean_cmd_z)),
            key=lambda item: abs(item[1]),
        )[0]
        dominant_cmd = {"x": mean_cmd_x, "y": mean_cmd_y, "z": mean_cmd_z}[dominant_axis]
        dominant_accel_key = f"accel_body_{dominant_axis}"
        dominant_values = [row[dominant_accel_key] for row in pose_window]
        peak_abs_dominant = max(abs(value) for value in dominant_values) if dominant_values else 0.0
        accel_threshold = max(0.25, 0.10 * peak_abs_dominant)
        onset_latency_sec = 0.0
        for row in pose_window:
            if dominant_cmd == 0.0:
                break
            if math.copysign(1.0, dominant_cmd) * row[dominant_accel_key] >= accel_threshold:
                onset_latency_sec = row["host_time_sec"] - start_time
                break

        row: dict[str, float | str] = {
            "window_index": float(window_index),
            "start_host_time_sec": start_time,
            "end_host_time_sec": end_time,
            "duration_sec": end_time - start_time,
            "dominant_axis": dominant_axis,
            "mean_cmd_x": mean_cmd_x,
            "mean_cmd_y": mean_cmd_y,
            "mean_cmd_z": mean_cmd_z,
            "mean_body_accel_x": sum(r["accel_body_x"] for r in pose_window) / len(pose_window),
            "mean_body_accel_y": sum(r["accel_body_y"] for r in pose_window) / len(pose_window),
            "mean_body_accel_z": sum(r["accel_body_z"] for r in pose_window) / len(pose_window),
            "peak_abs_body_accel_x": max(abs(r["accel_body_x"]) for r in pose_window),
            "peak_abs_body_accel_y": max(abs(r["accel_body_y"]) for r in pose_window),
            "peak_abs_body_accel_z": max(abs(r["accel_body_z"]) for r in pose_window),
            "mean_pose_roll_rad": sum(r["pose_roll_rad"] for r in pose_window) / len(pose_window),
            "mean_pose_pitch_rad": sum(r["pose_pitch_rad"] for r in pose_window) / len(pose_window),
            "mean_pose_yaw_rad": sum(r["pose_yaw_rad"] for r in pose_window) / len(pose_window),
            "mean_vel_world_x": sum(r["vel_world_x"] for r in pose_window) / len(pose_window),
            "mean_vel_world_y": sum(r["vel_world_y"] for r in pose_window) / len(pose_window),
            "mean_vel_world_z": sum(r["vel_world_z"] for r in pose_window) / len(pose_window),
            "dominant_response_ratio": (
                sum(dominant_values) / len(dominant_values) / dominant_cmd if abs(dominant_cmd) > 1e-6 else 0.0
            ),
            "onset_latency_sec": onset_latency_sec,
        }

        if imu_window:
            row["mean_imu_accel_x"] = sum(r["imu_accel_x"] for r in imu_window) / len(imu_window)
            row["mean_imu_accel_y"] = sum(r["imu_accel_y"] for r in imu_window) / len(imu_window)
            row["mean_imu_accel_z"] = sum(r["imu_accel_z"] for r in imu_window) / len(imu_window)
        else:
            row["mean_imu_accel_x"] = 0.0
            row["mean_imu_accel_y"] = 0.0
            row["mean_imu_accel_z"] = 0.0

        for servo_index in range(1, 5):
            servo_key = f"servo{servo_index}_raw"
            baseline_mean = (
                sum(r[servo_key] for r in baseline_actuator_window) / len(baseline_actuator_window)
                if baseline_actuator_window
                else 0.0
            )
            active_mean = (
                sum(r[servo_key] for r in active_actuator_window) / len(active_actuator_window)
                if active_actuator_window
                else 0.0
            )
            row[f"baseline_{servo_key}"] = baseline_mean
            row[f"active_{servo_key}"] = active_mean
            row[f"delta_{servo_key}"] = active_mean - baseline_mean

        row["mean_total_servo_delta_raw"] = sum(row[f"delta_servo{servo_index}_raw"] for servo_index in range(1, 5))

        comparison_rows.append(row)

    return comparison_rows


def main() -> int:
    args = parse_args()
    manifest = load_runtime_manifest()
    target = resolve_runtime_target(manifest, args.drone or "")
    if not container_is_running(VISUAL_CONTAINER):
        raise RuntimeError(f"{VISUAL_CONTAINER} is not running. Start the visual stack first.")
    if not container_is_running(DECISION_DEV_CONTAINER):
        raise RuntimeError(f"{DECISION_DEV_CONTAINER} is not running. Start the visual stack first.")

    run_dir = Path(manifest["current_run_dir"])
    output_dir = args.output_dir or (run_dir / "analysis" / f"policy_input_vs_imu_{time.strftime('%Y%m%d-%H%M%S')}")
    output_dir.mkdir(parents=True, exist_ok=True)

    imu_topic = (
        f"/world/{manifest['runtime_world_name']}/model/{target['runtime_model_name']}"
        "/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu"
    )
    pose_topic = f"/world/{manifest['runtime_world_name']}/dynamic_pose/info"

    print(f"Capturing policy-equivalent input from {target['command_topic']} for {args.duration:.1f}s")
    print(f"Capturing Gazebo IMU from {imu_topic}")
    print(f"Capturing Gazebo pose stream from {pose_topic}")
    print(f"Capturing MAVLink SERVO_OUTPUT_RAW from {target['mavlink_endpoint']}")
    print(f"Output directory: {output_dir}")

    command_queue: queue.Queue[CommandSample] = queue.Queue()
    imu_queue: queue.Queue[ImuSample] = queue.Queue()
    pose_queue: queue.Queue[PoseSample] = queue.Queue()
    actuator_queue: queue.Queue[ActuatorSample] = queue.Queue()

    command_proc = start_command_capture(args.duration, target["command_topic"])
    imu_proc = start_imu_capture(args.duration, imu_topic)
    pose_proc = start_pose_capture(args.duration, pose_topic)
    actuator_proc = start_actuator_capture(args.duration, target["mavlink_endpoint"])

    command_thread = threading.Thread(target=command_reader, args=(command_proc, command_queue), daemon=True)
    imu_thread = threading.Thread(target=imu_reader, args=(imu_proc, imu_queue), daemon=True)
    pose_thread = threading.Thread(
        target=pose_reader, args=(pose_proc, pose_queue, target["runtime_model_name"]), daemon=True
    )
    actuator_thread = threading.Thread(target=actuator_reader, args=(actuator_proc, actuator_queue), daemon=True)
    command_thread.start()
    imu_thread.start()
    pose_thread.start()
    actuator_thread.start()

    deadline = time.time() + args.duration
    try:
        while time.time() < deadline:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Capture interrupted by user.")
    finally:
        for proc in (command_proc, imu_proc, pose_proc, actuator_proc):
            if proc.poll() is None:
                proc.terminate()
        command_thread.join(timeout=2.0)
        imu_thread.join(timeout=2.0)
        pose_thread.join(timeout=2.0)
        actuator_thread.join(timeout=2.0)
        for proc_name, proc in (
            ("command", command_proc),
            ("imu", imu_proc),
            ("pose", pose_proc),
            ("actuator", actuator_proc),
        ):
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()
            stderr_path = output_dir / f"{proc_name}.stderr.log"
            if proc.stderr is not None:
                stderr_path.write_text(proc.stderr.read())

    command_samples = drain_queue(command_queue)
    imu_samples = drain_queue(imu_queue)
    pose_samples = drain_queue(pose_queue)
    actuator_samples = drain_queue(actuator_queue)

    write_command_csv(output_dir / "command_velocity.csv", command_samples)
    write_imu_csv(output_dir / "imu.csv", imu_samples)
    write_pose_csv(output_dir / "pose.csv", pose_samples)
    write_actuator_csv(output_dir / "actuator.csv", actuator_samples)
    merged_rows = merge_command_and_imu(command_samples, imu_samples)
    write_merged_csv(output_dir / "command_vs_imu.csv", merged_rows)
    pose_rows = derive_pose_kinematics(pose_samples)
    merged_pose_rows = merge_command_and_pose(command_samples, pose_rows)
    write_merged_csv(output_dir / "command_vs_pose.csv", merged_pose_rows)
    merged_actuator_rows = merge_command_and_actuator(command_samples, actuator_samples)
    write_merged_csv(output_dir / "command_vs_actuator.csv", merged_actuator_rows)
    comparison_rows = build_comparison_rows(merged_rows, merged_pose_rows, merged_actuator_rows)
    write_merged_csv(output_dir / "comparison.csv", comparison_rows)

    summary = {
        "imu": summarize(merged_rows),
        "pose": summarize_pose(merged_pose_rows),
        "windows": len(comparison_rows),
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    print(
        f"Captured {len(command_samples)} command samples, {len(imu_samples)} IMU samples, "
        f"{len(pose_samples)} pose samples, and {len(actuator_samples)} actuator samples."
    )
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
