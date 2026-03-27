from __future__ import annotations

import importlib.util
import json
from pathlib import Path


def load_module():
    module_path = Path(__file__).resolve().parents[1] / "scripts" / "verify_visual_runtime.py"
    spec = importlib.util.spec_from_file_location("verify_visual_runtime", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_build_report_uses_live_mount_and_optical_pose_for_experiment(tmp_path):
    module = load_module()
    manifest_path = tmp_path / "manifest.json"
    pose_path = tmp_path / "dynamic_pose.txt"
    joint_state_path = tmp_path / "joint_state.txt"

    manifest_path.write_text(
        json.dumps(
            {
                "mode": "experiment",
                "runtime_model_name": "iris_with_camera_experiment_test",
                "base_link_name": "iris_with_standoffs::base_link",
                "mount_link_name": "deployed_camera_rigid_mount_link",
                "optical_link_name": "deployed_camera_optical_link",
                "deployment": {
                    "position_m": {"x": 0.06, "y": 0.0, "z": -0.03},
                    "orientation_rad": {"yaw": 0.0, "pitch": 0.16, "roll": 0.0},
                },
            }
        )
        + "\n"
    )
    pose_path.write_text(
        """
pose {
  name: "iris_with_camera_experiment_test"
}
pose {
  name: "base_link"
  position { }
  orientation { w: 1 }
}
pose {
  name: "deployed_camera_rigid_mount_link"
  position { x: 0.06 y: 0.0 z: -0.03 }
  orientation { y: 0.0799146939691727 w: 0.9968017063026194 }
}
pose {
  name: "deployed_camera_optical_link"
  position { x: 0.06 y: 0.0 z: -0.03 }
  orientation { y: 0.0799146939691727 w: 0.9968017063026194 }
}
""".strip()
        + "\n"
    )
    joint_state_path.write_text('name: "iris_with_camera_experiment_test"\n')

    namespace = type(
        "Args",
        (),
        {
            "manifest": manifest_path,
            "pose_info": pose_path,
            "joint_state": joint_state_path,
            "report": None,
            "position_tol": 0.01,
            "orientation_tol": 0.02,
        },
    )()

    report = module.build_report(namespace)
    assert report["live_model_name"] == "iris_with_camera_experiment_test"
    assert report["position_errors"]["x"] < 1e-9
    assert report["position_errors"]["z"] < 1e-9
    assert report["orientation_errors"]["pitch"] < 1e-9
    assert report["optical_position_errors"]["x"] < 1e-9
    assert report["optical_orientation_errors"]["pitch"] < 1e-9
    assert all(report["pose_entities_present"].values())


def test_build_report_prefers_axis_position_for_calibration_joints(tmp_path):
    module = load_module()
    manifest_path = tmp_path / "manifest.json"
    pose_path = tmp_path / "dynamic_pose.txt"
    joint_state_path = tmp_path / "joint_state.txt"

    manifest_path.write_text(
        json.dumps(
            {
                "mode": "calib",
                "runtime_model_name": "iris_with_camera_calib_test",
                "base_link_name": "iris_with_standoffs::base_link",
                "mount_link_name": "deployed_camera_rigid_mount_link",
                "optical_link_name": "deployed_camera_optical_link",
                "deployment": {
                    "position_m": {"x": 0.06, "y": 0.0, "z": -0.03},
                    "orientation_rad": {"yaw": 0.0, "pitch": 0.16, "roll": 0.0},
                },
            }
        )
        + "\n"
    )
    pose_path.write_text(
        """
pose {
  name: "iris_with_camera_calib_test"
}
pose {
  name: "base_link"
  position { }
  orientation { w: 1 }
}
pose {
  name: "deployed_camera_rigid_mount_link"
  position { x: 0.06 y: 0.0 z: -0.03 }
  orientation { y: 0.0799146939691727 w: 0.9968017063026194 }
}
pose {
  name: "deployed_camera_optical_link"
  position { x: 0.06 y: 0.0 z: -0.03 }
  orientation { y: 0.0799146939691727 w: 0.9968017063026194 }
}
""".strip()
        + "\n"
    )
    joint_state_path.write_text(
        """
name: "iris_with_camera_calib_test"
joint {
  name: "deployed_camera_x_joint"
  pose { position { } orientation { w: 1 } }
  axis1 { position: 0.06 }
}
joint {
  name: "deployed_camera_y_joint"
  pose { position { } orientation { w: 1 } }
  axis1 { position: 0.0 }
}
joint {
  name: "deployed_camera_z_joint"
  pose { position { } orientation { w: 1 } }
  axis1 { position: -0.03 }
}
joint {
  name: "deployed_camera_yaw_joint"
  pose { position { } orientation { w: 1 } }
  axis1 { position: 0.0 }
}
joint {
  name: "deployed_camera_pitch_joint"
  pose { position { } orientation { w: 1 } }
  axis1 { position: 0.16 }
}
joint {
  name: "deployed_camera_roll_joint"
  pose { position { } orientation { w: 1 } }
  axis1 { position: 0.0 }
}
""".strip()
        + "\n"
    )

    namespace = type(
        "Args",
        (),
        {
            "manifest": manifest_path,
            "pose_info": pose_path,
            "joint_state": joint_state_path,
            "report": None,
            "position_tol": 0.01,
            "orientation_tol": 0.02,
        },
    )()

    report = module.build_report(namespace)
    assert report["position_errors"]["z"] < 1e-9
    assert report["orientation_errors"]["pitch"] < 1e-9
    assert report["joint_state_errors"]["position"]["z"] < 1e-9
    assert report["joint_state_errors"]["orientation"]["pitch"] < 1e-9
