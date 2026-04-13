#!/usr/bin/env python3

from __future__ import annotations

import json
import csv
import signal
import sys
import threading
import time
from pathlib import Path

from gz.msgs10.image_pb2 import Image
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node


class VisualFocusRouter:
    def __init__(self, config_path: Path):
        config = json.loads(config_path.read_text())
        self._active_drone_id = str(config["active_drone_id"])
        self._drones = {drone["id"]: drone for drone in config["drones"]}
        self._focus_topics = config["focus_topics"]
        self._active_topics = config["active_topics"]
        if self._active_drone_id not in self._drones:
            raise RuntimeError(f"Unknown active drone id {self._active_drone_id!r}")

        self._node = Node()
        self._lock = threading.Lock()
        self._running = True
        self._active_subscriptions: dict[str, str] = {}
        self._frame_stats: dict[str, dict[str, float]] = {}
        self._summary_interval_sec = float(config.get("diagnostics", {}).get("summary_interval_sec", 5.0))
        self._last_summary_sec = time.monotonic()
        self._frame_events_writer = None
        self._focus_events_writer = None
        self._frame_events_handle = None
        self._focus_events_handle = None
        self._flush_period_sec = float(config.get("diagnostics", {}).get("flush_period_sec", 1.0))
        self._last_frame_flush_sec = time.monotonic()
        self._last_focus_flush_sec = time.monotonic()
        self._active_chase_publisher = self._node.advertise(self._active_topics["chase"], Image)
        self._active_camera_publisher = self._node.advertise(self._active_topics["deployed"], Image)
        self._focus_state_publisher = self._node.advertise(self._focus_topics["state"], StringMsg)

        diagnostics = config.get("diagnostics", {})
        self._init_diagnostics(config_path, diagnostics)

        self._node.subscribe(StringMsg, self._focus_topics["select"], self._on_select)
        self._activate_drone_subscriptions(self._active_drone_id)

    def _init_diagnostics(self, config_path: Path, diagnostics: dict) -> None:
        frame_events_csv = diagnostics.get("frame_events_csv")
        focus_events_csv = diagnostics.get("focus_events_csv")
        if frame_events_csv:
            frame_path = Path(frame_events_csv)
            if not frame_path.is_absolute():
                frame_path = (config_path.parent / frame_path).resolve()
            frame_path.parent.mkdir(parents=True, exist_ok=True)
            self._frame_events_handle = frame_path.open("a", newline="")
            self._frame_events_writer = csv.DictWriter(
                self._frame_events_handle,
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
            if self._frame_events_handle.tell() == 0:
                self._frame_events_writer.writeheader()
                self._frame_events_handle.flush()
                self._last_frame_flush_sec = time.monotonic()
        if focus_events_csv:
            focus_path = Path(focus_events_csv)
            if not focus_path.is_absolute():
                focus_path = (config_path.parent / focus_path).resolve()
            focus_path.parent.mkdir(parents=True, exist_ok=True)
            self._focus_events_handle = focus_path.open("a", newline="")
            self._focus_events_writer = csv.DictWriter(
                self._focus_events_handle,
                fieldnames=[
                    "wall_time_epoch_sec",
                    "monotonic_sec",
                    "event_type",
                    "requested_drone_id",
                    "active_drone_id",
                ],
            )
            if self._focus_events_handle.tell() == 0:
                self._focus_events_writer.writeheader()
                self._focus_events_handle.flush()
                self._last_focus_flush_sec = time.monotonic()

    def _maybe_flush_frame_events(self) -> None:
        if self._frame_events_handle is None:
            return
        now = time.monotonic()
        if (now - self._last_frame_flush_sec) >= self._flush_period_sec:
            self._frame_events_handle.flush()
            self._last_frame_flush_sec = now

    def _maybe_flush_focus_events(self) -> None:
        if self._focus_events_handle is None:
            return
        now = time.monotonic()
        if (now - self._last_focus_flush_sec) >= self._flush_period_sec:
            self._focus_events_handle.flush()
            self._last_focus_flush_sec = now

    def _extract_sim_time_sec(self, msg: Image):
        try:
            header = getattr(msg, "header", None)
            if header is None:
                return None
            stamp = getattr(header, "stamp", None)
            if stamp is not None:
                sec = float(getattr(stamp, "sec", 0.0))
                nsec = float(getattr(stamp, "nsec", 0.0))
                return sec + (nsec / 1e9)
        except Exception:
            return None
        return None

    def _record_focus_event(self, event_type: str, requested_drone_id: str, active_drone_id: str) -> None:
        if self._focus_events_writer is None:
            return
        self._focus_events_writer.writerow(
            {
                "wall_time_epoch_sec": f"{time.time():.6f}",
                "monotonic_sec": f"{time.monotonic():.6f}",
                "event_type": event_type,
                "requested_drone_id": requested_drone_id,
                "active_drone_id": active_drone_id,
            }
        )
        self._maybe_flush_focus_events()

    def _record_frame_event(self, drone_id: str, stream_name: str, msg: Image) -> None:
        if self._frame_events_writer is None:
            return
        wall_time = time.time()
        monotonic_time = time.monotonic()
        sim_time = self._extract_sim_time_sec(msg)
        last_stat = self._frame_stats.get(stream_name, {})
        last_wall_time = last_stat.get("last_wall_time")
        interframe_gap_sec = None if last_wall_time is None else (wall_time - last_wall_time)
        first_wall_time = last_stat.get("first_wall_time", wall_time)
        first_sim_time = last_stat.get("first_sim_time", sim_time if sim_time is not None else None)
        backlog_sec = None
        if sim_time is not None and first_sim_time is not None:
            backlog_sec = (wall_time - first_wall_time) - (sim_time - first_sim_time)
        self._frame_stats[stream_name] = {
            "count": float(last_stat.get("count", 0.0) + 1.0),
            "interval_count": float(last_stat.get("interval_count", 0.0) + 1.0),
            "last_wall_time": wall_time,
            "first_wall_time": first_wall_time,
            "first_sim_time": first_sim_time,
            "last_sim_time": sim_time if sim_time is not None else last_stat.get("last_sim_time"),
        }
        self._frame_events_writer.writerow(
            {
                "wall_time_epoch_sec": f"{wall_time:.6f}",
                "monotonic_sec": f"{monotonic_time:.6f}",
                "drone_id": drone_id,
                "stream_name": stream_name,
                "width": int(getattr(msg, "width", 0)),
                "height": int(getattr(msg, "height", 0)),
                "source_sim_time_sec": "" if sim_time is None else f"{sim_time:.6f}",
                "stream_backlog_sec": "" if backlog_sec is None else f"{backlog_sec:.6f}",
                "interframe_gap_sec": "" if interframe_gap_sec is None else f"{interframe_gap_sec:.6f}",
            }
        )
        self._maybe_flush_frame_events()

    def _emit_periodic_summary(self) -> None:
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
                f"[visual_focus_router] stream={stream_name} approx_fps={approx_fps:.2f} total_frames={int(stats.get('count', 0.0))}",
                flush=True,
            )
            stats["interval_count"] = 0.0

    def _activate_drone_subscriptions(self, drone_id: str) -> None:
        for topic in list(self._active_subscriptions.values()):
            try:
                self._node.unsubscribe(topic)
            except Exception as exc:
                print(f"[visual_focus_router] Failed to unsubscribe from {topic}: {exc}", flush=True)
        self._active_subscriptions.clear()

        drone = self._drones[drone_id]
        chase_topic = drone["camera_topics"]["chase"]
        deployed_topic = drone["camera_topics"]["deployed"]
        self._node.subscribe(Image, chase_topic, self._build_image_callback(drone_id, "chase"))
        self._node.subscribe(Image, deployed_topic, self._build_image_callback(drone_id, "deployed"))
        self._active_subscriptions["chase"] = chase_topic
        self._active_subscriptions["deployed"] = deployed_topic
        self._record_focus_event("activate_subscriptions", drone_id, drone_id)
        print(
            (
                f"[visual_focus_router] Active subscriptions set for {drone_id}: "
                f"chase={chase_topic} deployed={deployed_topic}"
            ),
            flush=True,
        )

    def _build_image_callback(self, drone_id: str, stream_name: str):
        def _callback(msg: Image) -> None:
            with self._lock:
                if drone_id != self._active_drone_id:
                    return
            self._record_frame_event(drone_id, stream_name, msg)
            if stream_name == "chase":
                self._active_chase_publisher.publish(msg)
            else:
                self._active_camera_publisher.publish(msg)

        return _callback

    def _publish_focus_state(self) -> None:
        state = StringMsg()
        state.data = self._active_drone_id
        self._focus_state_publisher.publish(state)

    def _on_select(self, msg: StringMsg) -> None:
        requested = str(msg.data).strip()
        if requested not in self._drones:
            print(f"[visual_focus_router] Ignored unknown focus request: {requested}", flush=True)
            return
        self._record_focus_event("select_request", requested, self._active_drone_id)
        with self._lock:
            if requested == self._active_drone_id:
                self._publish_focus_state()
                return
            self._active_drone_id = requested
            self._activate_drone_subscriptions(requested)
        print(f"[visual_focus_router] Active drone -> {requested}", flush=True)
        self._record_focus_event("select_applied", requested, requested)
        self._publish_focus_state()

    def run(self) -> int:
        print(f"[visual_focus_router] Starting with active drone {self._active_drone_id}", flush=True)
        self._record_focus_event("router_started", self._active_drone_id, self._active_drone_id)
        self._publish_focus_state()
        while self._running:
            self._emit_periodic_summary()
            time.sleep(0.25)
        return 0

    def stop(self) -> None:
        self._running = False
        if self._frame_events_handle is not None:
            self._frame_events_handle.flush()
            self._frame_events_handle.close()
            self._frame_events_handle = None
        if self._focus_events_handle is not None:
            self._focus_events_handle.flush()
            self._focus_events_handle.close()
            self._focus_events_handle = None



def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(f"Usage: {argv[0]} CONFIG", file=sys.stderr)
        return 2

    router = VisualFocusRouter(Path(argv[1]))

    def _handle_signal(_signum, _frame):
        router.stop()

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)
    return router.run()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
