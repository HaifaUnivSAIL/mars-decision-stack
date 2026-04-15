from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts" / "vehicle_slice.py"


def load_module(module_path: Path, module_name: str):
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_single_and_fleet_slice_share_canonical_base_profile() -> None:
    module = load_module(MODULE_PATH, "vehicle_slice")
    reference_path = ROOT / "config" / "visual" / "camera.deployment.json"

    single = module.build_single_vehicle_slice(
        reference_path=reference_path,
        root_dir=ROOT,
        camera_deployment_config="config/visual/camera.deployment.json",
        alate_profile="config/alate/uav.visual.sitl.json",
        ros_alate_profile="config/ros_alate/adapter.yaml",
        ros_nemala_profile="config/ros_nemala/node_manager.yaml",
    )
    fleet = module.build_fleet_vehicle_slice(
        index=0,
        drone_id="drone_1",
        manifest_path=ROOT / "config" / "swarm" / "visual.swarm.json",
        root_dir=ROOT,
        camera_deployment_config="config/visual/camera.deployment.json",
        alate_profile="config/alate/uav.visual.sitl.json",
        ros_alate_profile="config/ros_alate/adapter.yaml",
        ros_nemala_profile="config/ros_nemala/node_manager.yaml",
        spawn={},
    )

    assert Path(single["alate_profile"]).resolve() == Path(fleet["alate_profile"]).resolve()
    assert single["mavlink_port"] == fleet["mavlink_port"] == 5762
    assert single["mavlink_aux_port"] == fleet["mavlink_aux_port"] == 5763
    assert single["runtime_paths"] == fleet["runtime_paths"]
    assert single["camera_deployment"] == fleet["camera_deployment"]
    assert single["proxy_name"] == "drone_1"
    assert fleet["proxy_name"] == "drone_1"
