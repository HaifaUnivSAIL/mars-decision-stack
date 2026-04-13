#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Build a generic stack profiling manifest from a visual runtime manifest.')
    parser.add_argument('--visual-manifest', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--run-dir', required=True, type=Path)
    parser.add_argument('--root-dir', required=True, type=Path)
    parser.add_argument('--stack-name', required=True)
    parser.add_argument('--network', required=True)
    parser.add_argument('--image', required=True)
    parser.add_argument('--mode', required=True, choices=('single', 'fleet'))
    parser.add_argument('--visual-gui', required=True, choices=('0', '1'))
    parser.add_argument('--sample-period-sec', type=float, default=1.0)
    parser.add_argument('--pose-sample-period-sec', type=float, default=0.05)
    parser.add_argument('--flush-period-sec', type=float, default=1.0)
    parser.add_argument('--summary-interval-sec', type=float, default=5.0)
    parser.add_argument('--container', action='append', default=[])
    parser.add_argument('--drone-endpoint', action='append', default=[])
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    visual_manifest = json.loads(args.visual_manifest.read_text())
    output_dir = args.run_dir / 'diagnostics' / 'profile'
    output_dir.mkdir(parents=True, exist_ok=True)

    endpoint_map: dict[str, str] = {}
    for item in args.drone_endpoint:
        if '=' not in item:
            raise SystemExit(f'Invalid --drone-endpoint value: {item!r}')
        drone_id, endpoint = item.split('=', 1)
        endpoint_map[drone_id.strip()] = endpoint.strip()

    drones: list[dict] = []
    for drone in visual_manifest.get('drones', []):
        drone_copy = dict(drone)
        drone_copy['mavlink_profile_endpoint'] = endpoint_map.get(drone_copy['id'], drone_copy.get('mavlink_profile_endpoint', ''))
        drones.append(drone_copy)

    manifest = {
        'run_id': visual_manifest.get('run_id'),
        'mode': args.mode,
        'fleet': bool(visual_manifest.get('fleet', False)),
        'run_dir': str(args.run_dir.resolve()),
        'root_dir': str(args.root_dir.resolve()),
        'output_dir': str(output_dir.resolve()),
        'stack_name': args.stack_name,
        'network': args.network,
        'image': args.image,
        'world_name': visual_manifest['runtime_world_name'],
        'active_drone_id': visual_manifest.get('active_drone_id') or (drones[0]['id'] if drones else ''),
        'active_topics': visual_manifest.get('active_topics', {}),
        'focus_topics': visual_manifest.get('focus_topics', {}),
        'visual_gui': args.visual_gui == '1',
        'sample_period_sec': float(args.sample_period_sec),
        'pose_sample_period_sec': float(args.pose_sample_period_sec),
        'flush_period_sec': float(args.flush_period_sec),
        'summary_interval_sec': float(args.summary_interval_sec),
        'core_containers': args.container,
        'drones': drones,
    }
    args.output.write_text(json.dumps(manifest, indent=2) + '\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
