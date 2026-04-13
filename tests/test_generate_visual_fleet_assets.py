from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
FLEET_MANIFEST_MODULE = ROOT / 'scripts' / 'fleet_manifest.py'
GENERATOR = ROOT / 'scripts' / 'generate_visual_fleet_assets.py'
SOURCE_MODEL = ROOT / 'external' / 'ardupilot_gazebo' / 'models' / 'iris_with_camera_calibration' / 'model.sdf'
WORLD_TEMPLATE = ROOT / 'sim' / 'worlds' / 'mars_iris_fleet_experiment.sdf'


def load_module(module_path: Path, module_name: str):
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def write_fleet_manifest(tmp_path: Path) -> Path:
    manifest_path = tmp_path / 'visual.swarm.json'
    manifest_path.write_text(
        json.dumps(
            {
                'active_drone_id': 'drone_1',
                'defaults': {
                    'camera_deployment_config': 'config/visual/camera.deployment.json',
                    'alate_profile': 'config/alate/uav.visual.sitl.json',
                    'ros_alate_profile': 'config/ros_alate/adapter.yaml',
                    'ros_nemala_profile': 'config/ros_nemala/node_manager.yaml',
                    'camera_streams': {
                        'deployed': {
                            'width': 640,
                            'height': 480,
                            'update_rate_hz': 10.0,
                        },
                        'chase': {
                            'width': 960,
                            'height': 540,
                            'update_rate_hz': 15.0,
                        },
                    },
                    'rendering': {
                        'sun_cast_shadows': False,
                    },
                    'physics': {
                        'max_step_size': 0.005,
                        'real_time_factor': 1.0,
                    },
                },
                'drones': [
                    {'id': 'drone_1', 'spawn': {'x': 0.0, 'y': 0.0, 'z': 0.195, 'yaw_deg': 90.0}},
                    {'id': 'drone_2', 'spawn': {'x': 2.0, 'y': 4.0, 'z': 0.195, 'yaw_deg': 180.0}},
                ],
            },
            indent=2,
        )
        + '\n'
    )
    return manifest_path


def test_fleet_manifest_derives_unique_namespaces_ports_and_topics(tmp_path: Path) -> None:
    module = load_module(FLEET_MANIFEST_MODULE, 'fleet_manifest')
    manifest_path = write_fleet_manifest(tmp_path)

    fleet = module.load_fleet_definition(manifest_path, ROOT)

    assert fleet['active_drone_id'] == 'drone_1'
    assert fleet['physics']['max_step_size'] == 0.005
    assert fleet['camera_streams']['deployed']['width'] == 640
    assert fleet['camera_streams']['chase']['update_rate_hz'] == 15.0
    assert fleet['rendering']['sun_cast_shadows'] is False
    assert [drone['id'] for drone in fleet['drones']] == ['drone_1', 'drone_2']
    assert [drone['namespace'] for drone in fleet['drones']] == ['/drone_1', '/drone_2']
    assert [drone['sitl_host'] for drone in fleet['drones']] == ['sitl-drone-1', 'sitl-drone-2']
    assert [drone['serial0_port'] for drone in fleet['drones']] == [5760, 5770]
    assert [drone['mavlink_port'] for drone in fleet['drones']] == [5762, 5772]
    assert [drone['mavlink_aux_port'] for drone in fleet['drones']] == [5763, 5773]
    assert [drone['fdm_port_in'] for drone in fleet['drones']] == [9002, 9012]
    assert [drone['fdm_port_out'] for drone in fleet['drones']] == [9003, 9013]
    assert fleet['drones'][0]['proxy_endpoints']['publishers'].endswith('/drone_1/alate_publishers')
    assert fleet['drones'][1]['proxy_endpoints']['subscribers'].endswith('/drone_2/alate_subscribers')
    assert fleet['drones'][0]['camera_topics']['deployed'] == '/mars/drone_1/visual/deployed_camera'
    assert fleet['drones'][1]['camera_topics']['chase'] == '/mars/drone_2/visual/chase_camera'


def test_fleet_generator_emits_unique_world_models_and_configs(tmp_path: Path) -> None:
    manifest_path = write_fleet_manifest(tmp_path)
    output_dir = tmp_path / 'out'
    subprocess.run(
        [
            sys.executable,
            str(GENERATOR),
            '--fleet-config',
            str(manifest_path),
            '--run-id',
            'pytest-fleet',
            '--output-dir',
            str(output_dir),
            '--source-model',
            str(SOURCE_MODEL),
            '--world-template',
            str(WORLD_TEMPLATE),
            '--root-dir',
            str(ROOT),
        ],
        check=True,
    )

    manifest = json.loads((output_dir / 'manifest.json').read_text())
    assert manifest['fleet'] is True
    assert manifest['active_drone_id'] == 'drone_1'
    assert manifest['physics']['max_step_size'] == 0.005
    assert manifest['camera_streams']['deployed']['width'] == 640
    assert manifest['rendering']['sun_cast_shadows'] is False
    assert len(manifest['drones']) == 2

    world_path = output_dir / manifest['runtime_world_relative_path']
    world_root = ET.fromstring(world_path.read_text())
    assert world_root.find('.//physics/max_step_size').text == '0.005000'
    assert world_root.find('.//light[@name="sun"]/cast_shadows').text == 'false'
    include_names = [element.text for element in world_root.findall('.//include/name')]
    assert include_names == [
        'iris_with_camera_experiment_pytest_fleet_drone_1',
        'iris_with_camera_experiment_pytest_fleet_drone_2',
    ]

    pose_values = [element.text for element in world_root.findall('.//include/pose')]
    assert '0.000000 0.000000 0.195000 0 0 90.000000' in pose_values
    assert '2.000000 4.000000 0.195000 0 0 180.000000' in pose_values

    model_1_root = ET.fromstring(
        (output_dir / 'models' / 'iris_with_camera_experiment_pytest_fleet_drone_1' / 'model.sdf').read_text()
    )
    mount_joint = model_1_root.find('.//joint[@name="deployed_camera_mount_joint"]')
    assert mount_joint is not None
    assert mount_joint.find('parent').text == 'iris_with_standoffs::base_link'
    assert mount_joint.find('child').text == 'deployed_camera_rigid_mount_link'
    deployed_camera_sensor = model_1_root.find('.//sensor[@name="camera"]')
    assert deployed_camera_sensor is not None
    assert deployed_camera_sensor.find('./plugin[@name="CameraZoomPlugin"]') is None
    assert deployed_camera_sensor.find('./plugin[@name="GstCameraPlugin"]') is None
    assert deployed_camera_sensor.find('./camera/image/width').text == '640'
    assert deployed_camera_sensor.find('./camera/image/height').text == '480'
    assert deployed_camera_sensor.find('update_rate').text == '10.000'
    assert deployed_camera_sensor.find('always_on').text == '0'
    chase_camera_sensor = model_1_root.find('.//sensor[@name="chase_camera"]')
    assert chase_camera_sensor is not None
    assert chase_camera_sensor.find('./camera/image/width').text == '960'
    assert chase_camera_sensor.find('./camera/image/height').text == '540'
    assert chase_camera_sensor.find('update_rate').text == '15.000'
    assert chase_camera_sensor.find('always_on').text == '0'

    drone_1_alate = json.loads((output_dir / 'runtime' / 'drone_1' / 'uav.visual.sitl.json').read_text())
    drone_2_alate = json.loads((output_dir / 'runtime' / 'drone_2' / 'uav.visual.sitl.json').read_text())
    assert drone_1_alate['autopilot']['master'] == 'tcp:sitl-drone-1:5762'
    assert drone_2_alate['autopilot']['master'] == 'tcp:sitl-drone-2:5772'

    focus_router_config = json.loads((output_dir / 'focus_router.json').read_text())
    assert focus_router_config['diagnostics']['frame_events_csv'] == '../diagnostics/latency/active_camera_frames.csv'
    visual_latency_config = json.loads((output_dir / 'visual_latency_recorder.json').read_text())
    assert visual_latency_config['world_name'] == 'mars_iris_dual_view'
    assert visual_latency_config['output_dir'] == '../diagnostics/latency'
    assert [drone['id'] for drone in visual_latency_config['drones']] == ['drone_1', 'drone_2']

    ros_alate_cfg = yaml.safe_load((output_dir / 'runtime' / 'drone_1' / 'ros_alate.adapter.yaml').read_text())
    assert '/drone_1/ros_alate_adapter' in ros_alate_cfg
    ros_nemala_cfg = yaml.safe_load((output_dir / 'runtime' / 'drone_2' / 'ros_nemala.node_manager.yaml').read_text())
    assert '/drone_2/nemala_node_manager' in ros_nemala_cfg
    assert ros_nemala_cfg['/drone_2/nemala_node_manager']['ros__parameters']['proxy_endpoint_publishers'].endswith('/drone_2/alate_publishers')
