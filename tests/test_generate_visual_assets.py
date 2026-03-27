from __future__ import annotations

import json
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts' / 'generate_visual_assets.py'
SOURCE_MODEL = ROOT / 'external' / 'ardupilot_gazebo' / 'models' / 'iris_with_camera_calibration' / 'model.sdf'
EXPERIMENT_WORLD = ROOT / 'sim' / 'worlds' / 'mars_iris_dual_view.sdf'
CALIBRATION_WORLD = ROOT / 'sim' / 'worlds' / 'mars_iris_dual_view_calibration.sdf'


def run_generator(tmp_path: Path, mode: str) -> Path:
    config = tmp_path / 'camera.json'
    config.write_text(
        json.dumps(
            {
                'deployment': {
                    'position_m': {'x': 0.06, 'y': 0.0, 'z': -0.03},
                    'orientation_rad': {'yaw': 0.1, 'pitch': 0.16, 'roll': -0.05},
                }
            }
        )
    )
    output_dir = tmp_path / 'out'
    subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            '--mode',
            mode,
            '--run-id',
            'pytest-run',
            '--config',
            str(config),
            '--output-dir',
            str(output_dir),
            '--source-model',
            str(SOURCE_MODEL),
            '--experiment-world-template',
            str(EXPERIMENT_WORLD),
            '--calibration-world-template',
            str(CALIBRATION_WORLD),
        ],
        check=True,
    )
    return output_dir


def run_generator_with_config(tmp_path: Path, mode: str, config_payload: dict) -> subprocess.CompletedProcess[str]:
    config = tmp_path / 'camera.json'
    config.write_text(json.dumps(config_payload))
    output_dir = tmp_path / 'out'
    return subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            '--mode',
            mode,
            '--run-id',
            'pytest-run',
            '--config',
            str(config),
            '--output-dir',
            str(output_dir),
            '--source-model',
            str(SOURCE_MODEL),
            '--experiment-world-template',
            str(EXPERIMENT_WORLD),
            '--calibration-world-template',
            str(CALIBRATION_WORLD),
        ],
        check=False,
        text=True,
        capture_output=True,
    )


def load_manifest(output_dir: Path) -> dict:
    return json.loads((output_dir / 'manifest.json').read_text())


def find_joint(root: ET.Element, joint_name: str) -> ET.Element:
    joint = root.find(f'.//joint[@name="{joint_name}"]')
    assert joint is not None
    return joint


def test_experiment_generation_bakes_absolute_pose_and_world_runtime_model(tmp_path: Path) -> None:
    output_dir = run_generator(tmp_path, 'experiment')
    manifest = load_manifest(output_dir)
    model_path = output_dir / manifest['runtime_model_relative_path']
    world_path = output_dir / manifest['runtime_world_relative_path']

    model_root = ET.fromstring(model_path.read_text())
    world_root = ET.fromstring(world_path.read_text())

    runtime_model_name = manifest['runtime_model_name']
    assert model_root.find('model').attrib['name'] == runtime_model_name
    assert model_root.find('.//joint[@name="deployed_camera_x_joint"]') is None
    assert model_root.find('.//joint[@name="deployed_camera_pitch_joint"]') is None
    assert find_joint(model_root, 'deployed_camera_mount_joint').find('child').text == 'deployed_camera_rigid_mount_link'
    rigid_mount_link = model_root.find('.//link[@name="deployed_camera_rigid_mount_link"]')
    pod_link = model_root.find('.//link[@name="deployed_camera_pod_link"]')
    optical_link = model_root.find('.//link[@name="deployed_camera_optical_link"]')
    assert rigid_mount_link is not None
    assert pod_link is not None
    assert optical_link is not None
    assert rigid_mount_link.find('pose').text == '0.060000 0.000000 -0.030000 -0.050000 0.160000 0.100000'
    assert pod_link.find('pose').text == '0.060000 0.000000 -0.030000 -0.050000 0.160000 0.100000'
    assert optical_link.find('pose').text == '0.060000 0.000000 -0.030000 -0.050000 0.160000 0.100000'
    include_uris = [uri.text for uri in world_root.findall('.//include/uri')]
    assert f'model://{runtime_model_name}' in include_uris
    assert manifest['deployment_target_link_name'] == 'deployed_camera_rigid_mount_link'
    assert manifest['calibration_joint_names'] == []


def test_calibration_generation_keeps_runtime_model_name_and_gui_binding(tmp_path: Path) -> None:
    output_dir = run_generator(tmp_path, 'calib')
    manifest = load_manifest(output_dir)
    model_path = output_dir / manifest['runtime_model_relative_path']
    world_path = output_dir / manifest['runtime_world_relative_path']

    model_root = ET.fromstring(model_path.read_text())
    world_root = ET.fromstring(world_path.read_text())

    runtime_model_name = manifest['runtime_model_name']
    assert model_root.find('model').attrib['name'] == runtime_model_name
    assert find_joint(model_root, 'deployed_camera_x_joint').attrib['type'] == 'prismatic'
    assert find_joint(model_root, 'deployed_camera_pitch_joint').attrib['type'] == 'revolute'
    assert world_root.find('.//plugin[@filename="JointPositionController"]/model_name').text == runtime_model_name
    assert manifest['deployment']['position_m']['x'] == 0.06
    assert manifest['deployment']['orientation_rad']['pitch'] == 0.16


def test_calibration_generation_rejects_out_of_range_pose(tmp_path: Path) -> None:
    result = run_generator_with_config(
        tmp_path,
        'calib',
        {
            'deployment': {
                'position_m': {'x': 16.0, 'y': 0.0, 'z': -0.03},
                'orientation_rad': {'yaw': 0.0, 'pitch': 0.16, 'roll': 0.0},
            }
        },
    )
    assert result.returncode != 0
    assert 'outside the calibration limit' in result.stderr
