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

from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.world_stats_pb2 import WorldStatistics
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


class StackVisualProfiler:
    def __init__(self, profile_manifest_path: Path):
        self._profile_manifest_path = profile_manifest_path.resolve()
        self._profile_manifest = json.loads(self._profile_manifest_path.read_text())
        self._running = True
        self._lock = threading.Lock()
        self._node = Node()
        self._flush_period_sec = float(self._profile_manifest.get("flush_period_sec", 1.0))
        self._pose_sample_period_sec = float(self._profile_manifest.get("pose_sample_period_sec", 0.05))
        self._focus_state_topic = str(self._profile_manifest.get("focus_topics", {}).get("state", "")).strip()
        self._world_name = str(self._profile_manifest["world_name"])
        self._pose_topic = f"/world/{self._world_name}/dynamic_pose/info"
        self._world_stats_topic = f"/world/{self._world_name}/stats"
        self._active_topics = dict(self._profile_manifest.get("active_topics", {}))
        self._drones = {
            drone["runtime_model_name"]: drone["id"]
            for drone in self._profile_manifest.get("drones", [])
        }
        self._last_pose_sample_sec = {drone_id: 0.0 for drone_id in self._drones.values()}
        self._first_pose_wall_time: dict[str, float] = {}
        self._first_pose_sim_time: dict[str, float] = {}
        self._first_frame_wall_time: dict[str, float] = {}
        self._first_frame_sim_time: dict[str, float] = {}
        self._frame_stats: dict[str, dict[str, float]] = {}
        self._last_flush_sec = time.monotonic()
        self._last_summary_sec = time.monotonic()
        self._summary_interval_sec = float(self._profile_manifest.get("summary_interval_sec", 5.0))
        self._active_drone_id = str(self._profile_manifest.get("active_drone_id") or "")

        output_dir = Path(self._profile_manifest["output_dir"])
        if not output_dir.exists():
            output_dir = self._profile_manifest_path.parent
        output_dir = output_dir.resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        self._output_dir = output_dir

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

        self._world_handle = (output_dir / "world_stats.csv").open("a", newline="")
        self._world_writer = csv.DictWriter(
            self._world_handle,
            fieldnames=[
                "wall_time_epoch_sec",
                "monotonic_sec",
                "sim_time_sec",
                "real_time_sec",
                "pause_time_sec",
                "real_time_factor",
                "iterations",
                "model_count",
                "paused",
                "step_size_sec",
                "stepping",
            ],
        )
        if self._world_handle.tell() == 0:
            self._world_writer.writeheader()

        self._camera_handle = (output_dir / "camera_frames.csv").open("a", newline="")
        self._camera_writer = csv.DictWriter(
            self._camera_handle,
            fieldnames=[
                "wall_time_epoch_sec",
                "monotonic_sec",
                "drone_id",
                "stream_name",
                "width",
                "height",
                "source_sim_time_sec",
                "stream_backlog_sec",
                "interframe_gap_sec",
            ],
        )
        if self._camera_handle.tell() == 0:
            self._camera_writer.writeheader()

        self._focus_handle = (output_dir / "focus_events.csv").open("a", newline="")
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

        self._node.subscribe(Pose_V, self._pose_topic, self._on_pose)
        self._node.subscribe(WorldStatistics, self._world_stats_topic, self._on_world_stats)
        chase_topic = str(self._active_topics.get("chase") or "").strip()
        if chase_topic:
            self._node.subscribe(Image, chase_topic, self._build_image_callback("chase"))
        deployed_topic = str(self._active_topics.get("deployed") or "").strip()
        if deployed_topic:
            self._node.subscribe(Image, deployed_topic, self._build_image_callback("deployed"))
        if self._focus_state_topic:
            self._node.subscribe(StringMsg, self._focus_state_topic, self._on_focus_state)

    @staticmethod
    def _sim_time_from_stamp(stamp) -> float | None:
        if stamp is None:
            return None
        return float(getattr(stamp, "sec", 0.0)) + (float(getattr(stamp, "nsec", 0.0)) / 1e9)

    def _extract_sim_time_sec(self, msg) -> float | None:
        header = getattr(msg, "header", None)
        if header is None:
            return None
        return self._sim_time_from_stamp(getattr(header, "stamp", None))

    def _maybe_flush(self) -> None:
        now = time.monotonic()
        if (now - self._last_flush_sec) < self._flush_period_sec:
            return
        self._pose_handle.flush()
        self._world_handle.flush()
        self._camera_handle.flush()
        self._focus_handle.flush()
        self._last_flush_sec = now

    def _record_focus_event(self, drone_id: str) -> None:
        self._focus_writer.writerow(
            {
                "wall_time_epoch_sec": f"{time.time():.6f}",
                "monotonic_sec": f"{time.monotonic():.6f}",
                "active_drone_id": drone_id,
            }
        )
        self._maybe_flush()

    def _on_focus_state(self, msg: StringMsg) -> None:
        with self._lock:
            self._active_drone_id = str(msg.data).strip()
            self._record_focus_event(self._active_drone_id)

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
                backlog = None
                first_sim = self._first_pose_sim_time.get(drone_id)
                if sim_time_sec is not None and first_sim is not None:
                    backlog = (wall_time - self._first_pose_wall_time[drone_id]) - (sim_time_sec - first_sim)
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
                        "pose_backlog_sec": "" if backlog is None else f"{backlog:.6f}",
                    }
                )
            self._maybe_flush()

    def _on_world_stats(self, msg: WorldStatistics) -> None:
        with self._lock:
            self._world_writer.writerow(
                {
                    "wall_time_epoch_sec": f"{time.time():.6f}",
                    "monotonic_sec": f"{time.monotonic():.6f}",
                    "sim_time_sec": "" if msg.sim_time is None else f"{self._sim_time_from_stamp(msg.sim_time):.6f}",
                    "real_time_sec": "" if msg.real_time is None else f"{self._sim_time_from_stamp(msg.real_time):.6f}",
                    "pause_time_sec": "" if msg.pause_time is None else f"{self._sim_time_from_stamp(msg.pause_time):.6f}",
                    "real_time_factor": f"{float(getattr(msg, 'real_time_factor', 0.0)):.6f}",
                    "iterations": int(getattr(msg, 'iterations', 0)),
                    "model_count": int(getattr(msg, 'model_count', 0)),
                    "paused": int(bool(getattr(msg, 'paused', False))),
                    "step_size_sec": "" if msg.step_size is None else f"{self._sim_time_from_stamp(msg.step_size):.6f}",
                    "stepping": int(bool(getattr(msg, 'stepping', False))),
                }
            )
            self._maybe_flush()

    def _build_image_callback(self, stream_name: str):
        def _callback(msg: Image) -> None:
            wall_time = time.time()
            monotonic_time = time.monotonic()
            sim_time_sec = self._extract_sim_time_sec(msg)
            with self._lock:
                drone_id = self._active_drone_id or self._profile_manifest.get("active_drone_id", "")
                stats = self._frame_stats.setdefault(stream_name, {})
                last_wall = stats.get("last_wall_time")
                interframe_gap = None if last_wall is None else (wall_time - last_wall)
                first_wall = stats.get("first_wall_time", wall_time)
                first_sim = stats.get("first_sim_time", sim_time_sec)
                backlog = None
                if sim_time_sec is not None and first_sim is not None:
                    backlog = (wall_time - first_wall) - (sim_time_sec - first_sim)
                stats.update(
                    {
                        "count": float(stats.get("count", 0.0) + 1.0),
                        "interval_count": float(stats.get("interval_count", 0.0) + 1.0),
                        "last_wall_time": wall_time,
                        "first_wall_time": first_wall,
                        "first_sim_time": first_sim,
                    }
                )
                self._camera_writer.writerow(
                    {
                        "wall_time_epoch_sec": f"{wall_time:.6f}",
                        "monotonic_sec": f"{monotonic_time:.6f}",
                        "drone_id": drone_id,
                        "stream_name": stream_name,
                        "width": int(getattr(msg, "width", 0)),
                        "height": int(getattr(msg, "height", 0)),
                        "source_sim_time_sec": "" if sim_time_sec is None else f"{sim_time_sec:.6f}",
                        "stream_backlog_sec": "" if backlog is None else f"{backlog:.6f}",
                        "interframe_gap_sec": "" if interframe_gap is None else f"{interframe_gap:.6f}",
                    }
                )
                self._maybe_flush()

        return _callback

    def _emit_summary(self) -> None:
        now = time.monotonic()
        elapsed = now - self._last_summary_sec
        if elapsed < self._summary_interval_sec:
            return
        self._last_summary_sec = now
        for stream_name, stats in sorted(self._frame_stats.items()):
            interval_count = float(stats.get("interval_count", 0.0))
            if interval_count <= 0.0:
                continue
            approx_fps = interval_count / max(elapsed, 1e-6)
            print(
                f"[stack_visual_profiler] stream={stream_name} approx_fps={approx_fps:.2f} total_frames={int(stats.get('count', 0.0))}",
                flush=True,
            )
            stats["interval_count"] = 0.0

    def run(self) -> int:
        print(
            (
                f"[stack_visual_profiler] Recording pose={self._pose_topic} world_stats={self._world_stats_topic} "
                f"active_topics={self._active_topics}"
            ),
            flush=True,
        )
        while self._running:
            time.sleep(0.25)
            with self._lock:
                self._emit_summary()
        return 0

    def stop(self) -> None:
        self._running = False
        with self._lock:
            self._pose_handle.flush()
            self._pose_handle.close()
            self._world_handle.flush()
            self._world_handle.close()
            self._camera_handle.flush()
            self._camera_handle.close()
            self._focus_handle.flush()
            self._focus_handle.close()


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(f"Usage: {argv[0]} PROFILE_MANIFEST", file=sys.stderr)
        return 2

    profiler = StackVisualProfiler(Path(argv[1]))

    def _handle_signal(_signum, _frame):
        profiler.stop()

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)
    return profiler.run()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
