#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import time
from collections import deque
from pathlib import Path

from pymavlink import mavutil


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check whether a drone is actually takeoff-capable over MAVLink.")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--endpoint")
    source.add_argument("--input-dir", type=Path)
    parser.add_argument("--timeout-sec", type=float, default=30.0)
    parser.add_argument("--clear-prearm-window-sec", type=float, default=3.0)
    parser.add_argument("--report", type=Path)
    return parser.parse_args()


def write_report(path: Path | None, payload: dict) -> None:
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n")


def _evaluate_from_rows(
    *,
    report: dict,
    heartbeat_rows: list[dict[str, str]],
    statustext_rows: list[dict[str, str]],
    clear_prearm_window_sec: float,
    now_wall: float,
) -> bool:
    report["heartbeat_count"] = len(heartbeat_rows)
    if heartbeat_rows:
        last_heartbeat = heartbeat_rows[-1]
        report["last_mode"] = last_heartbeat.get("mode_string")
        armed_text = str(last_heartbeat.get("armed", "")).strip().lower()
        report["last_armed"] = armed_text == "true"

    recent_statustext: list[dict[str, object]] = []
    last_prearm_time: float | None = None
    last_prearm_text: str | None = None
    for row in statustext_rows[-20:]:
        wall_time = float(row.get("host_time_sec", "0") or 0.0)
        text = str(row.get("text", "")).strip()
        severity = int(str(row.get("severity", "0") or 0))
        recent_statustext.append(
            {
                "wall_time_epoch_sec": round(wall_time, 6),
                "severity": severity,
                "text": text,
            }
        )
        if text.startswith("PreArm:"):
            last_prearm_time = wall_time
            last_prearm_text = text

    report["recent_statustext"] = recent_statustext
    report["last_prearm_text"] = last_prearm_text
    if len(heartbeat_rows) < 2:
        return False
    if last_prearm_time is None:
        return True
    return (now_wall - last_prearm_time) >= clear_prearm_window_sec


def _wait_for_profile_trace(input_dir: Path, args: argparse.Namespace, report: dict) -> int:
    heartbeat_path = input_dir / "heartbeat.csv"
    statustext_path = input_dir / "statustext.csv"
    deadline = time.monotonic() + max(args.timeout_sec, 1.0)

    while time.monotonic() < deadline:
        heartbeat_rows: list[dict[str, str]] = []
        statustext_rows: list[dict[str, str]] = []
        if heartbeat_path.exists():
            try:
                import csv

                with heartbeat_path.open(newline="") as handle:
                    heartbeat_rows = list(csv.DictReader(handle))
            except Exception:
                heartbeat_rows = []
        if statustext_path.exists():
            try:
                import csv

                with statustext_path.open(newline="") as handle:
                    statustext_rows = list(csv.DictReader(handle))
            except Exception:
                statustext_rows = []

        now_wall = time.time()
        if _evaluate_from_rows(
            report=report,
            heartbeat_rows=heartbeat_rows,
            statustext_rows=statustext_rows,
            clear_prearm_window_sec=args.clear_prearm_window_sec,
            now_wall=now_wall,
        ):
            report["ready"] = True
            write_report(args.report, report)
            return 0
        time.sleep(1)

    if int(report["heartbeat_count"]) == 0:
        report["failure_reason"] = "no_heartbeat"
    else:
        report["failure_reason"] = "prearm_not_clear" if report["last_prearm_text"] else "heartbeat_not_stable"
    write_report(args.report, report)
    return 1


def main() -> int:
    args = parse_args()
    report = {
        "endpoint": args.endpoint,
        "input_dir": str(args.input_dir) if args.input_dir else None,
        "timeout_sec": float(args.timeout_sec),
        "clear_prearm_window_sec": float(args.clear_prearm_window_sec),
        "heartbeat_count": 0,
        "last_mode": None,
        "last_armed": None,
        "last_prearm_text": None,
        "recent_statustext": [],
        "ready": False,
        "failure_reason": None,
    }
    if args.input_dir is not None:
        return _wait_for_profile_trace(args.input_dir, args, report)

    started = time.monotonic()
    deadline = started + max(args.timeout_sec, 1.0)
    conn = mavutil.mavlink_connection(args.endpoint, timeout=5)
    recent_statustext: deque[dict[str, object]] = deque(maxlen=20)
    last_prearm_monotonic: float | None = None

    try:
        first_heartbeat = conn.wait_heartbeat(timeout=max(min(args.timeout_sec, 20.0), 1.0))
        if first_heartbeat is None:
            report["failure_reason"] = "no_heartbeat"
            write_report(args.report, report)
            return 1

        report["heartbeat_count"] = 1
        report["last_mode"] = mavutil.mode_string_v10(first_heartbeat)
        report["last_armed"] = bool(first_heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        while time.monotonic() < deadline:
            msg = conn.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=1)
            now_wall = time.time()
            now_mono = time.monotonic()
            if msg is None:
                if report["heartbeat_count"] >= 2 and (
                    last_prearm_monotonic is None or (now_mono - last_prearm_monotonic) >= args.clear_prearm_window_sec
                ):
                    report["ready"] = True
                    write_report(args.report, report)
                    return 0
                continue

            payload = msg.to_dict()
            if msg.get_type() == "HEARTBEAT":
                report["heartbeat_count"] = int(report["heartbeat_count"]) + 1
                report["last_mode"] = mavutil.mode_string_v10(msg)
                base_mode = int(payload.get("base_mode", 0))
                report["last_armed"] = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif msg.get_type() == "STATUSTEXT":
                text = payload.get("text", "")
                if isinstance(text, bytes):
                    text = text.decode(errors="replace")
                text = str(text).strip()
                recent_statustext.append(
                    {
                        "wall_time_epoch_sec": round(now_wall, 6),
                        "severity": int(payload.get("severity", 0)),
                        "text": text,
                    }
                )
                if text.startswith("PreArm:"):
                    last_prearm_monotonic = now_mono
                    report["last_prearm_text"] = text

            report["recent_statustext"] = list(recent_statustext)
            if report["heartbeat_count"] >= 2 and (
                last_prearm_monotonic is None or (now_mono - last_prearm_monotonic) >= args.clear_prearm_window_sec
            ):
                report["ready"] = True
                write_report(args.report, report)
                return 0

        report["failure_reason"] = "prearm_not_clear" if report["last_prearm_text"] else "heartbeat_not_stable"
        write_report(args.report, report)
        return 1
    finally:
        try:
            conn.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
