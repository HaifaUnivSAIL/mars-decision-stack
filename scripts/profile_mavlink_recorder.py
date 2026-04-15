#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import signal
import time
from pathlib import Path

from pymavlink import mavutil


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Record long-lived MAVLink control traces for one drone.')
    parser.add_argument('--endpoint', required=True)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--request-rate-hz', type=float, default=10.0)
    return parser.parse_args()


class CsvStream:
    def __init__(self, path: Path, fieldnames: list[str]):
        self._handle = path.open('a', newline='')
        self._writer = csv.DictWriter(self._handle, fieldnames=fieldnames)
        if self._handle.tell() == 0:
            self._writer.writeheader()
            self._handle.flush()

    def write(self, row: dict[str, object]) -> None:
        self._writer.writerow(row)
        self._handle.flush()

    def close(self) -> None:
        self._handle.flush()
        self._handle.close()


class MavlinkRecorder:
    def __init__(self, endpoint: str, output_dir: Path, request_rate_hz: float):
        self._endpoint = endpoint
        self._output_dir = output_dir
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._running = True
        self._period_usec = max(int(1_000_000 / max(request_rate_hz, 0.1)), 100_000)
        self._conn = mavutil.mavlink_connection(endpoint, timeout=5)

        self._servo = CsvStream(
            self._output_dir / 'servo_output_raw.csv',
            ['host_time_sec', 'time_usec', 'servo1_raw', 'servo2_raw', 'servo3_raw', 'servo4_raw'],
        )
        self._attitude = CsvStream(
            self._output_dir / 'attitude.csv',
            ['host_time_sec', 'time_boot_ms', 'roll', 'pitch', 'yaw', 'rollspeed', 'pitchspeed', 'yawspeed'],
        )
        self._position = CsvStream(
            self._output_dir / 'local_position_ned.csv',
            ['host_time_sec', 'time_boot_ms', 'x', 'y', 'z', 'vx', 'vy', 'vz'],
        )
        self._heartbeat = CsvStream(
            self._output_dir / 'heartbeat.csv',
            [
                'host_time_sec',
                'base_mode',
                'custom_mode',
                'system_status',
                'mav_type',
                'autopilot',
                'mode_string',
                'armed',
            ],
        )
        self._statustext = CsvStream(
            self._output_dir / 'statustext.csv',
            ['host_time_sec', 'severity', 'text'],
        )

    def _request_streams(self) -> None:
        rate_hz = max(int(1_000_000 / self._period_usec), 1)
        self._conn.mav.request_data_stream_send(
            self._conn.target_system,
            self._conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            rate_hz,
            1,
        )
        for message_id in (
            mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,
        ):
            self._conn.mav.command_long_send(
                self._conn.target_system,
                self._conn.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                self._period_usec,
                0,
                0,
                0,
                0,
                0,
                0,
            )

    def run(self) -> int:
        heartbeat = self._conn.wait_heartbeat(timeout=20)
        if heartbeat is None:
            raise RuntimeError(f'No MAVLink heartbeat on {self._endpoint}')
        self._request_streams()
        last_request_sec = time.monotonic()
        while self._running:
            msg = self._conn.recv_match(
                type=['SERVO_OUTPUT_RAW', 'ATTITUDE', 'LOCAL_POSITION_NED', 'HEARTBEAT', 'STATUSTEXT'],
                blocking=True,
                timeout=1,
            )
            now = time.monotonic()
            if (now - last_request_sec) >= 5.0:
                self._request_streams()
                last_request_sec = now
            if msg is None:
                continue
            host_time_sec = time.time()
            payload = msg.to_dict()
            msg_type = msg.get_type()
            if msg_type == 'SERVO_OUTPUT_RAW':
                self._servo.write(
                    {
                        'host_time_sec': f'{host_time_sec:.6f}',
                        'time_usec': int(payload.get('time_usec', 0)),
                        'servo1_raw': int(payload.get('servo1_raw', 0)),
                        'servo2_raw': int(payload.get('servo2_raw', 0)),
                        'servo3_raw': int(payload.get('servo3_raw', 0)),
                        'servo4_raw': int(payload.get('servo4_raw', 0)),
                    }
                )
            elif msg_type == 'ATTITUDE':
                self._attitude.write(
                    {
                        'host_time_sec': f'{host_time_sec:.6f}',
                        'time_boot_ms': int(payload.get('time_boot_ms', 0)),
                        'roll': float(payload.get('roll', 0.0)),
                        'pitch': float(payload.get('pitch', 0.0)),
                        'yaw': float(payload.get('yaw', 0.0)),
                        'rollspeed': float(payload.get('rollspeed', 0.0)),
                        'pitchspeed': float(payload.get('pitchspeed', 0.0)),
                        'yawspeed': float(payload.get('yawspeed', 0.0)),
                    }
                )
            elif msg_type == 'LOCAL_POSITION_NED':
                self._position.write(
                    {
                        'host_time_sec': f'{host_time_sec:.6f}',
                        'time_boot_ms': int(payload.get('time_boot_ms', 0)),
                        'x': float(payload.get('x', 0.0)),
                        'y': float(payload.get('y', 0.0)),
                        'z': float(payload.get('z', 0.0)),
                        'vx': float(payload.get('vx', 0.0)),
                        'vy': float(payload.get('vy', 0.0)),
                        'vz': float(payload.get('vz', 0.0)),
                    }
                )
            elif msg_type == 'HEARTBEAT':
                base_mode = int(payload.get('base_mode', 0))
                armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self._heartbeat.write(
                    {
                        'host_time_sec': f'{host_time_sec:.6f}',
                        'base_mode': base_mode,
                        'custom_mode': int(payload.get('custom_mode', 0)),
                        'system_status': int(payload.get('system_status', 0)),
                        'mav_type': int(payload.get('type', 0)),
                        'autopilot': int(payload.get('autopilot', 0)),
                        'mode_string': mavutil.mode_string_v10(msg),
                        'armed': 'true' if armed else 'false',
                    }
                )
            elif msg_type == 'STATUSTEXT':
                text = payload.get('text', '')
                if isinstance(text, bytes):
                    text = text.decode(errors='replace')
                self._statustext.write(
                    {
                        'host_time_sec': f'{host_time_sec:.6f}',
                        'severity': int(payload.get('severity', 0)),
                        'text': str(text).strip(),
                    }
                )
        return 0

    def stop(self) -> None:
        if not self._running:
            return
        self._running = False
        try:
            self._conn.close()
        except Exception:
            pass
        for stream in (self._servo, self._attitude, self._position, self._heartbeat, self._statustext):
            stream.close()


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
