from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts' / 'verify_visual_fleet_runtime.py'


def test_verify_visual_fleet_runtime_accepts_two_matching_drones(tmp_path: Path) -> None:
    manifest_path = tmp_path / 'manifest.json'
    pose_path = tmp_path / 'pose.info.txt'
    report_path = tmp_path / 'report.json'

    manifest_path.write_text(
        json.dumps(
            {
                'runtime_world_name': 'mars_iris_dual_view',
                'active_drone_id': 'drone_1',
                'drones': [
                    {
                        'id': 'drone_1',
                        'runtime_model_name': 'iris_with_camera_experiment_test_drone_1',
                        'base_link_name': 'iris_with_standoffs::base_link',
                        'deployment_target_link_name': 'deployed_camera_rigid_mount_link',
                        'camera_deployment': {
                            'x': 0.06,
                            'y': 0.0,
                            'z': -0.03,
                            'yaw': 0.0,
                            'pitch': 0.16,
                            'roll': 0.0,
                        },
                    },
                    {
                        'id': 'drone_2',
                        'runtime_model_name': 'iris_with_camera_experiment_test_drone_2',
                        'base_link_name': 'iris_with_standoffs::base_link',
                        'deployment_target_link_name': 'deployed_camera_rigid_mount_link',
                        'camera_deployment': {
                            'x': 0.10,
                            'y': 0.02,
                            'z': -0.05,
                            'yaw': 0.0,
                            'pitch': 0.0,
                            'roll': 0.0,
                        },
                    },
                ],
            },
            indent=2,
        )
        + '\n'
    )

    pose_path.write_text(
        '\n'.join(
            [
                'pose {',
                '  name: "iris_with_camera_experiment_test_drone_1::base_link"',
                '  position { x: 0 y: 0 z: 0 }',
                '  orientation { x: 0 y: 0 z: 0 w: 1 }',
                '}',
                'pose {',
                '  name: "iris_with_camera_experiment_test_drone_1::deployed_camera_rigid_mount_link"',
                '  position { x: 0.06 y: 0.0 z: -0.03 }',
                '  orientation { x: 0 y: 0.0799146939691727 z: 0 w: 0.9968017063026194 }',
                '}',
                'pose {',
                '  name: "iris_with_camera_experiment_test_drone_2::base_link"',
                '  position { x: 0 y: 0 z: 0 }',
                '  orientation { x: 0 y: 0 z: 0 w: 1 }',
                '}',
                'pose {',
                '  name: "iris_with_camera_experiment_test_drone_2::deployed_camera_rigid_mount_link"',
                '  position { x: 0.10 y: 0.02 z: -0.05 }',
                '  orientation { x: 0 y: 0 z: 0 w: 1 }',
                '}',
            ]
        )
        + '\n'
    )

    subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            '--manifest',
            str(manifest_path),
            '--pose-info',
            str(pose_path),
            '--report',
            str(report_path),
        ],
        check=True,
    )

    report = json.loads(report_path.read_text())
    assert report['ok'] is True
    assert len(report['drones']) == 2
    assert all(drone['ok'] for drone in report['drones'])
