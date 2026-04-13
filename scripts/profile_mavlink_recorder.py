#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import signal
import sys
import time
from pathlib import Path

from pymavlink import mavutil


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Record MAVLink actuator and motion signals for one drone.')
    parser.add_argument('--endpoint', required=True)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--request-rate-hz', type=float, default=10.0)
    return parser.parse_args()


class MavlinkRecorder:
    def __init__(self, endpoint: str, output_dir: Path, request_rate_hz: float):
        self._endpoint = endpoint
        self._output_dir = output_dir
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._running = True
        self._period_usec = max(int(1_000_000 / max(request_rate_hz, 0.1)), 100_000)
        self._conn = mavutil.mavlink_connection(endpoint, timeout=5)
        self._attitude_handle = (self._output_dir / 'attitude.csv').open('a', newline='')
        self._attitude_writer = csv.DictWriter(
            self._attitude_handle,
            fieldnames=['host_time_sec', 'time_boot_ms', 'roll', 'pitch', 'yaw', 'rollspeed', 'pitchspeed', 'yawspeed'],
        )
        if self._attitude_handle.tell() == 0:
            self._attitude_writer.writeheader()
        self._position_handle = (self._output_dir / 'local_position_ned.csv').open('a', newline='')
        self._position_writer = csv.DictWriter(
            self._position_handle,
            fieldnames=['host_time_sec', 'time_boot_ms', 'x', 'y', 'z', 'vx', 'vy', 'vz'],
        )
        if self._position_handle.tell() == 0:
            self._position_writer.writeheader()
        self._servo_handle = (self._output_dir / 'servo_output_raw.csv').open('a', newline='')
        self._servo_writer = csv.DictWriter(
            self._servo_handle,
            fieldnames=['host_time_sec', 'time_usec', 'servo1_raw', 'servo2_raw', 'servo3_raw', 'servo4_raw'],
        )
        if self._servo_handle.tell() == 0:
            self._servo_writer.writeheader()

    def _request_streams(self) -> None:
        self._conn.mav.request_data_stream_send(
            self._conn.target_system,
            self._conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            max(int(1_000_000 / self._period_usec), 1),
            1,
        )
        for message_id in (
            mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        ):
            self._conn.mav.command_long_send(
                self._conn.target_system,
                self._conn.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                self._period_usec,
                0, 0, 0, 0, 0,
            )

    def run(self) -> int:
        hb = self._conn.wait_heartbeat(timeout=20)
        if hb is None:
            raise RuntimeError(f'No MAVLink heartbeat on {self._endpoint}')
        self._request_streams()
        last_request_sec = time.monotonic()
        while self._running:
            msg = self._conn.recv_match(type=['SERVO_OUTPUT_RAW', 'ATTITUDE', 'LOCAL_POSITION_NED'], blocking=True, timeout=1)
            now = time.monotonic()
            if (now - last_request_sec) >= 5.0:
                self._request_streams()
                last_request_sec = now
            if msg is None:
                continue
            host_time_sec = time.time()
            data = msg.to_dict()
            msg_type = msg.get_type()
            if msg_type == 'SERVO_OUTPUT_RAW':
                self._servo_writer.writerow(
                    {
                        'host_time_sec': f'{host_time_sec:.6f}',
                        'time_usec': int(data.get('time_usec', 0)),
                        'servo1_raw': int(data.get('servo1_raw', 0)),
                        'servo2_raw': int(data.get('servo2_raw', 0)),
                        'servo3_raw': int(data.get('servo3_raw', 0)),
                        'servo4_raw': int(data.get('servo4_raw', 0)),
                    }
                )
                self._servo_handle.flush()
            elif msg_type == 'ATTITUDE':
                self._attitude_writer.writerow(
                    {
                        'host_time_sec': f'{host_time_sec:.6f}',
                        'time_boot_ms': int(data.get('time_boot_ms', 0)),
                        'roll': float(data.get('roll', 0.0)),
                        'pitch': float(data.get('pitch', 0.0)),
                        'yaw': float(data.get('yaw', 0.0)),
                        'rollspeed': float(data.get('rollspeed', 0.0)),
                        'pitchspeed': float(data.get('pitchspeed', 0.0)),
                        'yawspeed': float(data.get('yawspeed', 0.0)),
                    }
                )
                self._attitude_handle.flush()
            elif msg_type == 'LOCAL_POSITION_NED':
                self._position_writer.writerow(
                    {
                        'host_time_sec': f'{host_time_sec:.6f}',
                        'time_boot_ms': int(data.get('time_boot_ms', 0)),
                        'x': float(data.get('x', 0.0)),
                        'y': float(data.get('y', 0.0)),
                        'z': float(data.get('z', 0.0)),
                        'vx': float(data.get('vx', 0.0)),
                        'vy': float(data.get('vy', 0.0)),
                        'vz': float(data.get('vz', 0.0)),
                    }
                )
                self._position_handle.flush()
        return 0

    def stop(self) -> None:
        self._running = False
        self._servo_handle.flush()
        self._servo_handle.close()
        self._attitude_handle.flush()
        self._attitude_handle.close()
        self._position_handle.flush()
        self._position_handle.close()
        try:
            self._conn.close()
        except Exception:
            pass


def main() -> int:
    args = parse_args()
    recorder = MavlinkRecorder(args.endpoint, args.output_dir, args.request_rate_hz)

    def _handle_signal(_signum, _frame):
        recorder.stop()

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)
    try:
        return recorder.run()
    finally:
        recorder.stop()


if __name__ == '__main__':
    raise SystemExit(main())
