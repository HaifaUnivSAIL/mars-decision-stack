#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


STATUS_PATTERN = re.compile(
    r"status mode=(?P<mode>\S+) armed=(?P<armed>\S+) altitude=(?P<altitude>[-+0-9.eE]+) "
    r"gps_fix=(?P<gps_fix>\d+) armable=(?P<armable>\S+) state=(?P<state>\S+)"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Summarize a fleet startup failure for one drone.")
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--drone-id", required=True)
    parser.add_argument("--stage", required=True)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--endpoint-report", type=Path)
    parser.add_argument("--readiness-report", type=Path)
    return parser.parse_args()


def load_json(path: Path | None) -> dict | None:
    if path is None or not path.exists():
        return None
    try:
        return json.loads(path.read_text())
    except Exception:
        return {"error": f"failed_to_parse:{path}"}


def count_serial_events(lines: list[str]) -> dict[str, int]:
    counts = {
        "serial1_open_count": 0,
        "serial1_close_count": 0,
        "serial2_open_count": 0,
        "serial2_close_count": 0,
    }
    for line in lines:
        if "New connection on SERIAL1" in line:
            counts["serial1_open_count"] += 1
        elif "Closed connection on SERIAL1" in line:
            counts["serial1_close_count"] += 1
        elif "New connection on SERIAL2" in line:
            counts["serial2_open_count"] += 1
        elif "Closed connection on SERIAL2" in line:
            counts["serial2_close_count"] += 1
    return counts


def parse_status_samples(lines: list[str]) -> list[dict]:
    samples = []
    for index, line in enumerate(lines, start=1):
        match = STATUS_PATTERN.search(line)
        if not match:
            continue
        groups = match.groupdict()
        samples.append(
            {
                "line_number": index,
                "mode": groups["mode"],
                "armed": groups["armed"].lower() == "true",
                "altitude": float(groups["altitude"]),
                "gps_fix": int(groups["gps_fix"]),
                "armable": groups["armable"].lower() == "true",
                "state": groups["state"],
            }
        )
    return samples


def first_unready_status(samples: list[dict]) -> dict | None:
    for sample in samples:
        if sample["gps_fix"] < 3 or not sample["armable"] or sample["state"].upper() == "BOOT":
            return sample
    return None


def first_ready_like_status(samples: list[dict]) -> dict | None:
    for sample in samples:
        if sample["gps_fix"] >= 3 and sample["armable"] and sample["state"].upper() != "BOOT":
            return sample
    return None


def main() -> int:
    args = parse_args()
    drone_log_dir = args.run_dir / "logs" / args.drone_id
    sitl_log = drone_log_dir / "sitl.log"
    hlc_log = drone_log_dir / "hlc.log"
    mc_log = drone_log_dir / "mc.log"

    sitl_lines = sitl_log.read_text(errors="replace").splitlines() if sitl_log.exists() else []
    hlc_lines = hlc_log.read_text(errors="replace").splitlines() if hlc_log.exists() else []
    mc_lines = mc_log.read_text(errors="replace").splitlines() if mc_log.exists() else []
    status_samples = parse_status_samples(hlc_lines)

    report = {
        "drone_id": args.drone_id,
        "stage": args.stage,
        "endpoint_report": load_json(args.endpoint_report),
        "readiness_report": load_json(args.readiness_report),
        "sitl_serial_events": count_serial_events(sitl_lines),
        "hlc_markers": {
            "ready_seen": any("HLC entering state: Ready" in line for line in hlc_lines),
            "waiting_for_llc_seen": any("HLC entering state: WaitingForLlc" in line for line in hlc_lines),
            "connection_established_seen": any("connection established" in line.lower() for line in hlc_lines),
            "heartbeat_timeout_count": sum("No heartbeat in 10.0 seconds, aborting." in line for line in hlc_lines),
        },
        "mc_markers": {
            "standby_seen": any("MissionControl entering state: Standby" in line for line in mc_lines),
        },
        "first_status_sample": status_samples[0] if status_samples else None,
        "first_unready_status": first_unready_status(status_samples),
        "first_ready_like_status": first_ready_like_status(status_samples),
        "status_sample_count": len(status_samples),
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
