#!/usr/bin/env python3

from __future__ import annotations

import csv
import json
import math
import signal
import sys
import threading
import time
from pathlib import Path

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
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


class SwarmVisualLatencyRecorder:
    def __init__(self, config_path: Path):
        config = json.loads(config_path.read_text())
        self._running = True
        self._lock = threading.Lock()
        self._node = Node()
        self._world_name = str(config["world_name"])
        self._pose_topic = f"/world/{self._world_name}/dynamic_pose/info"
        self._focus_state_topic = str(config["focus_state_topic"])
        self._drones = {drone["runtime_model_name"]: drone["id"] for drone in config["drones"]}
        self._pose_sample_period_sec = float(config.get("pose_sample_period_sec", 0.05))
        self._flush_period_sec = float(config.get("flush_period_sec", 1.0))
        self._last_pose_sample_sec = {drone_id: 0.0 for drone_id in self._drones.values()}
        self._first_pose_wall_time: dict[str, float] = {}
        self._first_pose_sim_time: dict[str, float] = {}
        self._last_pose_flush_sec = time.monotonic()
        self._last_focus_flush_sec = time.monotonic()

        output_dir = Path(config["output_dir"])
        if not output_dir.is_absolute():
            output_dir = (config_path.parent / output_dir).resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        self._pose_handle = (output_dir / "pose_trace.csv").open("a", newline="")
        self._pose_writer = csv.DictWriter(
            self._pose_handle,
            fieldnames=[
                "wall_time_epoch_sec",
                "monotonic_sec",
                "sim_time_sec",
                "drone_id",
                "entity_name",
                "x",
                "y",
                "z",
                "roll",
                "pitch",
                "yaw",
                "pose_backlog_sec",
            ],
        )
        if self._pose_handle.tell() == 0:
            self._pose_writer.writeheader()
            self._pose_handle.flush()
            self._last_pose_flush_sec = time.monotonic()

        self._focus_handle = (output_dir / "focus_state_trace.csv").open("a", newline="")
        self._focus_writer = csv.DictWriter(
            self._focus_handle,
            fieldnames=[
                "wall_time_epoch_sec",
                "monotonic_sec",
                "active_drone_id",
            ],
        )
        if self._focus_handle.tell() == 0:
            self._focus_writer.writeheader()
            self._focus_handle.flush()
            self._last_focus_flush_sec = time.monotonic()

        self._node.subscribe(Pose_V, self._pose_topic, self._on_pose)
        self._node.subscribe(StringMsg, self._focus_state_topic, self._on_focus_state)

    def _extract_sim_time_sec(self, msg: Pose_V):
        header = getattr(msg, "header", None)
        if header is None:
            return None
        stamp = getattr(header, "stamp", None)
        if stamp is None:
            return None
        return float(getattr(stamp, "sec", 0.0)) + (float(getattr(stamp, "nsec", 0.0)) / 1e9)

    def _on_focus_state(self, msg: StringMsg) -> None:
        with self._lock:
            self._focus_writer.writerow(
                {
                    "wall_time_epoch_sec": f"{time.time():.6f}",
                    "monotonic_sec": f"{time.monotonic():.6f}",
                    "active_drone_id": str(msg.data).strip(),
                }
            )
            self._maybe_flush_focus()

    def _maybe_flush_pose(self) -> None:
        now = time.monotonic()
        if (now - self._last_pose_flush_sec) >= self._flush_period_sec:
            self._pose_handle.flush()
            self._last_pose_flush_sec = now

    def _maybe_flush_focus(self) -> None:
        now = time.monotonic()
        if (now - self._last_focus_flush_sec) >= self._flush_period_sec:
            self._focus_handle.flush()
            self._last_focus_flush_sec = now

    def _on_pose(self, msg: Pose_V) -> None:
        wall_time = time.time()
        monotonic_time = time.monotonic()
        sim_time_sec = self._extract_sim_time_sec(msg)
        poses = getattr(msg, "pose", [])
        with self._lock:
            for pose in poses:
                entity_name = str(getattr(pose, "name", "")).strip()
                drone_id = self._drones.get(entity_name)
                if not drone_id:
                    continue
                last_sample = self._last_pose_sample_sec.get(drone_id, 0.0)
                if (monotonic_time - last_sample) < self._pose_sample_period_sec:
                    continue
                self._last_pose_sample_sec[drone_id] = monotonic_time

                position = getattr(pose, "position", None)
                orientation = getattr(pose, "orientation", None)
                if position is None or orientation is None:
                    continue
                if drone_id not in self._first_pose_wall_time:
                    self._first_pose_wall_time[drone_id] = wall_time
                if sim_time_sec is not None and drone_id not in self._first_pose_sim_time:
                    self._first_pose_sim_time[drone_id] = sim_time_sec
                pose_backlog_sec = None
                first_sim_time = self._first_pose_sim_time.get(drone_id)
                if sim_time_sec is not None and first_sim_time is not None:
                    pose_backlog_sec = (
                        (wall_time - self._first_pose_wall_time[drone_id]) -
                        (sim_time_sec - first_sim_time)
                    )
                roll, pitch, yaw = quaternion_to_euler(
                    float(getattr(orientation, "x", 0.0)),
                    float(getattr(orientation, "y", 0.0)),
                    float(getattr(orientation, "z", 0.0)),
                    float(getattr(orientation, "w", 1.0)),
                )
                self._pose_writer.writerow(
                    {
                        "wall_time_epoch_sec": f"{wall_time:.6f}",
                        "monotonic_sec": f"{monotonic_time:.6f}",
                        "sim_time_sec": "" if sim_time_sec is None else f"{sim_time_sec:.6f}",
                        "drone_id": drone_id,
                        "entity_name": entity_name,
                        "x": f"{float(getattr(position, 'x', 0.0)):.6f}",
                        "y": f"{float(getattr(position, 'y', 0.0)):.6f}",
                        "z": f"{float(getattr(position, 'z', 0.0)):.6f}",
                        "roll": f"{roll:.6f}",
                        "pitch": f"{pitch:.6f}",
                        "yaw": f"{yaw:.6f}",
                        "pose_backlog_sec": "" if pose_backlog_sec is None else f"{pose_backlog_sec:.6f}",
                    }
                )
            self._maybe_flush_pose()

    def run(self) -> int:
        print(
            (
                f"[swarm_visual_latency_recorder] Recording pose topic={self._pose_topic} "
                f"focus_state_topic={self._focus_state_topic}"
            ),
            flush=True,
        )
        while self._running:
            time.sleep(0.25)
        return 0

    def stop(self) -> None:
        self._running = False
        with self._lock:
            self._pose_handle.flush()
            self._pose_handle.close()
            self._focus_handle.flush()
            self._focus_handle.close()


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(f"Usage: {argv[0]} CONFIG", file=sys.stderr)
        return 2

    recorder = SwarmVisualLatencyRecorder(Path(argv[1]))

    def _handle_signal(_signum, _frame):
        recorder.stop()

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)
    return recorder.run()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
