#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Host-side profiling supervisor for visual stack runs.')
    parser.add_argument('--profile-manifest', required=True, type=Path)
    return parser.parse_args()


def parse_percentage(value: str) -> float:
    text = str(value).strip().rstrip('%')
    return float(text or 0.0)


def parse_human_bytes(value: str) -> float:
    text = str(value).strip()
    if not text or text.lower() == 'n/a':
        return 0.0
    units = {
        'b': 1.0,
        'kb': 1_000.0,
        'kib': 1024.0,
        'mb': 1_000_000.0,
        'mib': 1024.0 ** 2,
        'gb': 1_000_000_000.0,
        'gib': 1024.0 ** 3,
        'tb': 1_000_000_000_000.0,
        'tib': 1024.0 ** 4,
    }
    number = ''
    unit = ''
    for char in text:
        if char.isdigit() or char in '.-':
            number += char
        elif not char.isspace():
            unit += char
    if not number:
        return 0.0
    return float(number) * units.get(unit.lower(), 1.0)


class ProfileSupervisor:
    def __init__(self, profile_manifest_path: Path):
        self._profile_manifest_path = profile_manifest_path.resolve()
        self._manifest = json.loads(self._profile_manifest_path.read_text())
        self._output_dir = Path(self._manifest['output_dir']).resolve()
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._snapshots_dir = self._output_dir / 'snapshots'
        self._snapshots_dir.mkdir(parents=True, exist_ok=True)
        self._running = True
        self._sample_period_sec = float(self._manifest.get('sample_period_sec', 1.0))
        self._root_dir = Path(self._manifest.get('root_dir', '.')).resolve()
        self._state_path = self._output_dir / 'profile.state.json'
        self._core_containers = list(self._manifest.get('core_containers', []))
        self._mavlink_processes: list[subprocess.Popen[str]] = []
        self._mavlink_state: list[dict[str, Any]] = []
        self._snapshotted_containers: set[str] = set()
        self._started_wall_time_epoch_sec = time.time()

        self._container_stats_handle = (self._output_dir / 'container_stats.csv').open('a', newline='')
        self._container_stats_writer = csv.DictWriter(
            self._container_stats_handle,
            fieldnames=[
                'wall_time_epoch_sec',
                'container_name',
                'cpu_percent',
                'mem_usage_bytes',
                'mem_limit_bytes',
                'mem_percent',
                'net_in_bytes',
                'net_out_bytes',
                'block_in_bytes',
                'block_out_bytes',
                'pids',
            ],
        )
        if self._container_stats_handle.tell() == 0:
            self._container_stats_writer.writeheader()

        self._gpu_stats_handle = (self._output_dir / 'gpu_stats.csv').open('a', newline='')
        self._gpu_stats_writer = csv.DictWriter(
            self._gpu_stats_handle,
            fieldnames=[
                'wall_time_epoch_sec',
                'gpu_index',
                'gpu_name',
                'utilization_gpu_percent',
                'utilization_memory_percent',
                'memory_used_mib',
                'memory_total_mib',
            ],
        )
        if self._gpu_stats_handle.tell() == 0:
            self._gpu_stats_writer.writeheader()

        self._gpu_process_handle = (self._output_dir / 'gpu_process_stats.csv').open('a', newline='')
        self._gpu_process_writer = csv.DictWriter(
            self._gpu_process_handle,
            fieldnames=[
                'wall_time_epoch_sec',
                'gpu_index',
                'pid',
                'type',
                'sm_util',
                'mem_util',
                'enc_util',
                'dec_util',
                'command',
            ],
        )
        if self._gpu_process_handle.tell() == 0:
            self._gpu_process_writer.writeheader()

    def _run(self, cmd: list[str]) -> subprocess.CompletedProcess[str]:
        return subprocess.run(cmd, capture_output=True, text=True)

    def _write_state(self) -> None:
        self._state_path.write_text(
            json.dumps(
                {
                    'pid': os.getpid(),
                    'profile_manifest': str(self._profile_manifest_path),
                    'started_wall_time_epoch_sec': self._started_wall_time_epoch_sec,
                    'mavlink_recorders': self._mavlink_state,
                },
                indent=2,
            )
            + '\n'
        )

    def _existing_containers(self) -> set[str]:
        result = self._run(['docker', 'ps', '--format', '{{.Names}}'])
        if result.returncode != 0:
            return set()
        return {line.strip() for line in result.stdout.splitlines() if line.strip()}

    def _snapshot_container(self, container_name: str) -> None:
        if container_name in self._snapshotted_containers:
            return
        inspect = self._run(['docker', 'inspect', container_name])
        if inspect.returncode == 0:
            (self._snapshots_dir / f'{container_name}.inspect.json').write_text(inspect.stdout)
        top = self._run(['docker', 'top', container_name])
        if top.returncode == 0:
            (self._snapshots_dir / f'{container_name}.top.txt').write_text(top.stdout)
        self._snapshotted_containers.add(container_name)

    def _sample_container_stats(self) -> None:
        existing = self._existing_containers()
        active = [name for name in self._core_containers if name in existing]
        for container_name in active:
            self._snapshot_container(container_name)
        if not active:
            return

        result = self._run(['docker', 'stats', '--no-stream', '--format', '{{json .}}', *active])
        if result.returncode != 0:
            return

        wall_time = time.time()
        for line in result.stdout.splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
                mem_usage, mem_limit = str(row.get('MemUsage', '0 / 0')).split(' / ', 1)
                net_in, net_out = str(row.get('NetIO', '0 / 0')).split(' / ', 1)
                block_in, block_out = str(row.get('BlockIO', '0 / 0')).split(' / ', 1)
                self._container_stats_writer.writerow(
                    {
                        'wall_time_epoch_sec': f'{wall_time:.6f}',
                        'container_name': row.get('Name', ''),
                        'cpu_percent': f'{parse_percentage(row.get("CPUPerc", "0")):.6f}',
                        'mem_usage_bytes': f'{parse_human_bytes(mem_usage):.6f}',
                        'mem_limit_bytes': f'{parse_human_bytes(mem_limit):.6f}',
                        'mem_percent': f'{parse_percentage(row.get("MemPerc", "0")):.6f}',
                        'net_in_bytes': f'{parse_human_bytes(net_in):.6f}',
                        'net_out_bytes': f'{parse_human_bytes(net_out):.6f}',
                        'block_in_bytes': f'{parse_human_bytes(block_in):.6f}',
                        'block_out_bytes': f'{parse_human_bytes(block_out):.6f}',
                        'pids': int(str(row.get('PIDs', '0')).strip() or 0),
                    }
                )
            except Exception:
                continue
        self._container_stats_handle.flush()

    def _sample_gpu_stats(self) -> None:
        if not shutil.which('nvidia-smi'):
            return
        wall_time = time.time()
        gpu = self._run(
            [
                'nvidia-smi',
                '--query-gpu=index,name,utilization.gpu,utilization.memory,memory.used,memory.total',
                '--format=csv,noheader,nounits',
            ]
        )
        if gpu.returncode == 0:
            for line in gpu.stdout.splitlines():
                parts = [part.strip() for part in line.split(',')]
                if len(parts) < 6:
                    continue
                self._gpu_stats_writer.writerow(
                    {
                        'wall_time_epoch_sec': f'{wall_time:.6f}',
                        'gpu_index': parts[0],
                        'gpu_name': parts[1],
                        'utilization_gpu_percent': parts[2],
                        'utilization_memory_percent': parts[3],
                        'memory_used_mib': parts[4],
                        'memory_total_mib': parts[5],
                    }
                )
            self._gpu_stats_handle.flush()

        pmon = self._run(['nvidia-smi', 'pmon', '-c', '1', '-s', 'um'])
        if pmon.returncode != 0:
            return
        for line in pmon.stdout.splitlines():
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                continue
            parts = stripped.split()
            if len(parts) < 8:
                continue
            self._gpu_process_writer.writerow(
                {
                    'wall_time_epoch_sec': f'{wall_time:.6f}',
                    'gpu_index': parts[0],
                    'pid': parts[1],
                    'type': parts[2],
                    'sm_util': parts[3],
                    'mem_util': parts[4],
                    'enc_util': parts[5],
                    'dec_util': parts[6],
                    'command': parts[7],
                }
            )
        self._gpu_process_handle.flush()

    def _start_mavlink_recorders(self) -> None:
        for drone in self._manifest.get('drones', []):
            endpoint = str(drone.get('mavlink_profile_endpoint') or drone.get('mavlink_endpoint') or '').strip()
            if not endpoint:
                continue
            drone_id = str(drone['id'])
            output_dir = self._output_dir / 'mavlink' / drone_id
            output_dir.mkdir(parents=True, exist_ok=True)
            cmd = [
                sys.executable,
                str(self._root_dir / 'scripts' / 'profile_mavlink_recorder.py'),
                '--endpoint', endpoint,
                '--output-dir', str(output_dir),
            ]
            proc = subprocess.Popen(cmd)
            self._mavlink_processes.append(proc)
            self._mavlink_state.append(
                {
                    'drone_id': drone_id,
                    'endpoint': endpoint,
                    'output_dir': str(output_dir),
                    'pid': proc.pid,
                }
            )
        self._write_state()

    def _stop_mavlink_recorders(self) -> None:
        for proc in self._mavlink_processes:
            if proc.poll() is None:
                proc.terminate()
        deadline = time.time() + 10.0
        for proc in self._mavlink_processes:
            if proc.poll() is not None:
                continue
            remaining = max(deadline - time.time(), 0.1)
            try:
                proc.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                proc.kill()
        self._mavlink_processes.clear()

    def stop(self) -> None:
        self._running = False

    def run(self) -> int:
        self._start_mavlink_recorders()
        self._write_state()
        while self._running:
            self._sample_container_stats()
            self._sample_gpu_stats()
            time.sleep(self._sample_period_sec)
        self._sample_container_stats()
        self._sample_gpu_stats()
        self._stop_mavlink_recorders()
        self._write_state()
        self._container_stats_handle.flush()
        self._gpu_stats_handle.flush()
        self._gpu_process_handle.flush()
        self._container_stats_handle.close()
        self._gpu_stats_handle.close()
        self._gpu_process_handle.close()
        return 0


def main() -> int:
    args = parse_args()
    supervisor = ProfileSupervisor(args.profile_manifest)

    def _handle_signal(_signum, _frame):
        supervisor.stop()

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)
    return supervisor.run()


if __name__ == '__main__':
    raise SystemExit(main())
