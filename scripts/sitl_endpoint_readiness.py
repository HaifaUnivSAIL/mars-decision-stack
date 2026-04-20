#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass
class HeartbeatStabilityTracker:
    min_heartbeats: int
    max_gap_sec: float
    settle_window_sec: float
    heartbeat_count: int = 0
    consecutive_heartbeats: int = 0
    stable_window_reset_count: int = 0
    max_observed_gap_sec: float = 0.0
    stable_since_monotonic: float | None = None
    last_heartbeat_monotonic: float | None = None

    def observe_gap(self, now_monotonic: float) -> None:
        if self.last_heartbeat_monotonic is None:
            return
        gap_sec = now_monotonic - self.last_heartbeat_monotonic
        self.max_observed_gap_sec = max(self.max_observed_gap_sec, gap_sec)
        if gap_sec <= self.max_gap_sec:
            return
        if self.consecutive_heartbeats > 0 or self.stable_since_monotonic is not None:
            self.stable_window_reset_count += 1
        self.consecutive_heartbeats = 0
        self.stable_since_monotonic = None

    def record_heartbeat(self, now_monotonic: float) -> None:
        gap_sec = None
        if self.last_heartbeat_monotonic is not None:
            gap_sec = now_monotonic - self.last_heartbeat_monotonic
            self.max_observed_gap_sec = max(self.max_observed_gap_sec, gap_sec)
        if gap_sec is None or gap_sec <= self.max_gap_sec:
            self.consecutive_heartbeats += 1
        else:
            self.consecutive_heartbeats = 1
            self.stable_since_monotonic = None
            self.stable_window_reset_count += 1
        self.heartbeat_count += 1
        self.last_heartbeat_monotonic = now_monotonic
        if self.consecutive_heartbeats >= self.min_heartbeats and self.stable_since_monotonic is None:
            self.stable_since_monotonic = now_monotonic

    def stable_window_sec(self, now_monotonic: float) -> float:
        if self.stable_since_monotonic is None:
            return 0.0
        return max(now_monotonic - self.stable_since_monotonic, 0.0)

    def is_ready(self, now_monotonic: float) -> bool:
        if self.consecutive_heartbeats < self.min_heartbeats:
            return False
        return self.stable_window_sec(now_monotonic) >= self.settle_window_sec


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Wait for a stable SITL MAVLink endpoint before HLC attaches.")
    parser.add_argument("--endpoint", required=True)
    parser.add_argument("--timeout-sec", type=float, default=60.0)
    parser.add_argument("--min-heartbeats", type=int, default=3)
    parser.add_argument("--max-gap-sec", type=float, default=2.0)
    parser.add_argument("--settle-window-sec", type=float, default=3.0)
    parser.add_argument("--report", type=Path)
    return parser.parse_args()


def write_report(path: Path | None, payload: dict) -> None:
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n")


def default_report(args: argparse.Namespace) -> dict:
    return {
        "endpoint": args.endpoint,
        "timeout_sec": float(args.timeout_sec),
        "min_heartbeats": int(args.min_heartbeats),
        "max_gap_sec": float(args.max_gap_sec),
        "settle_window_sec": float(args.settle_window_sec),
        "ready": False,
        "failure_reason": None,
        "connection_attempt_count": 0,
        "heartbeat_count": 0,
        "consecutive_heartbeats": 0,
        "stable_window_reset_count": 0,
        "stable_window_sec": 0.0,
        "max_observed_gap_sec": 0.0,
        "last_mode": None,
        "last_system_status": None,
        "recent_statustext": [],
        "last_exception": None,
    }


def main() -> int:
    from pymavlink import mavutil

    args = parse_args()
    report = default_report(args)
    tracker = HeartbeatStabilityTracker(
        min_heartbeats=max(int(args.min_heartbeats), 1),
        max_gap_sec=max(float(args.max_gap_sec), 0.1),
        settle_window_sec=max(float(args.settle_window_sec), 0.0),
    )
    deadline = time.monotonic() + max(float(args.timeout_sec), 1.0)
    recent_statustext: list[dict[str, object]] = []

    conn = None
    silence_deadline = None
    try:
        while time.monotonic() < deadline:
            if conn is None:
                try:
                    conn = mavutil.mavlink_connection(args.endpoint, timeout=max(tracker.max_gap_sec, 1.0))
                    report["connection_attempt_count"] = int(report["connection_attempt_count"]) + 1
                    silence_deadline = time.monotonic() + max(tracker.max_gap_sec * 2.0, 3.0)
                except Exception as exc:
                    report["last_exception"] = repr(exc)
                    time.sleep(1.0)
                    continue

            now_monotonic = time.monotonic()
            tracker.observe_gap(now_monotonic)
            try:
                msg = conn.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=1)
            except Exception as exc:
                report["last_exception"] = repr(exc)
                try:
                    conn.close()
                except Exception:
                    pass
                conn = None
                time.sleep(1.0)
                continue

            now_monotonic = time.monotonic()
            if msg is None:
                if tracker.heartbeat_count == 0 and silence_deadline is not None and now_monotonic >= silence_deadline:
                    try:
                        conn.close()
                    except Exception:
                        pass
                    conn = None
                    silence_deadline = None
                    time.sleep(1.0)
                    continue
                if tracker.is_ready(now_monotonic):
                    report["ready"] = True
                    report["failure_reason"] = None
                    break
                continue

            payload = msg.to_dict()
            if msg.get_type() == "HEARTBEAT":
                tracker.record_heartbeat(now_monotonic)
                silence_deadline = now_monotonic + max(tracker.max_gap_sec * 2.0, 3.0)
                report["last_mode"] = mavutil.mode_string_v10(msg)
                report["last_system_status"] = int(payload.get("system_status", 0))
                if tracker.is_ready(now_monotonic):
                    report["ready"] = True
                    report["failure_reason"] = None
                    break
            elif msg.get_type() == "STATUSTEXT":
                text = payload.get("text", "")
                if isinstance(text, bytes):
                    text = text.decode(errors="replace")
                recent_statustext.append(
                    {
                        "wall_time_epoch_sec": round(time.time(), 6),
                        "severity": int(payload.get("severity", 0)),
                        "text": str(text).strip(),
                    }
                )
                recent_statustext = recent_statustext[-20:]

        if not report["ready"]:
            if tracker.heartbeat_count == 0:
                report["failure_reason"] = "no_heartbeat"
            elif tracker.consecutive_heartbeats < tracker.min_heartbeats:
                report["failure_reason"] = "insufficient_consecutive_heartbeats"
            else:
                report["failure_reason"] = "heartbeat_never_stabilized"
    finally:
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass

    now_monotonic = time.monotonic()
    report["heartbeat_count"] = tracker.heartbeat_count
    report["consecutive_heartbeats"] = tracker.consecutive_heartbeats
    report["stable_window_reset_count"] = tracker.stable_window_reset_count
    report["stable_window_sec"] = round(tracker.stable_window_sec(now_monotonic), 6)
    report["max_observed_gap_sec"] = round(tracker.max_observed_gap_sec, 6)
    report["recent_statustext"] = recent_statustext

    write_report(args.report, report)
    return 0 if report["ready"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
