#!/usr/bin/env python3

from __future__ import annotations

import copy
from typing import Any


BASE_RUNTIME_DEFAULTS: dict[str, Any] = {
    "fdm_exchange": {
        "offline_recv_timeout_ms": 1,
        "online_recv_timeout_ms": 10,
    },
    "camera_streams": {
        "deployed": {
            "width": 640,
            "height": 480,
            "update_rate_hz": 10.0,
        },
        "chase": {
            "width": 1280,
            "height": 720,
            "update_rate_hz": 30.0,
        },
    },
    "rendering": {
        "sun_cast_shadows": True,
    },
    "physics": {
        "max_step_size": 0.001,
        "real_time_factor": 1.0,
    },
    "sensor_behavior": {
        "deployed": {
            "visualize": True,
            "always_on": True,
            "remove_plugins": [],
        },
        "chase": {
            "visualize": False,
            "always_on": True,
            "remove_plugins": [],
        },
    },
}


RUNTIME_PROFILE_OVERRIDES: dict[str, dict[str, Any]] = {
    "single_equivalent": {},
    "fleet_control_stable": {
        "fdm_exchange": {
            "online_recv_timeout_ms": 1,
        },
        "camera_streams": {
            "chase": {
                "width": 960,
                "height": 540,
                "update_rate_hz": 15.0,
            },
        },
        "rendering": {
            "sun_cast_shadows": False,
        },
        "physics": {
            "max_step_size": 0.005,
            "real_time_factor": 1.0,
        },
        "sensor_behavior": {
            "deployed": {
                "visualize": False,
                "always_on": False,
                "remove_plugins": ["CameraZoomPlugin", "GstCameraPlugin"],
            },
            "chase": {
                "visualize": False,
                "always_on": False,
                "remove_plugins": [],
            },
        },
    },
    "fleet_visual_optimized": {
        "fdm_exchange": {
            "online_recv_timeout_ms": 1,
        },
        "camera_streams": {
            "chase": {
                "width": 960,
                "height": 540,
                "update_rate_hz": 15.0,
            },
        },
        "rendering": {
            "sun_cast_shadows": False,
        },
        "physics": {
            "max_step_size": 0.02,
            "real_time_factor": 1.0,
        },
        "sensor_behavior": {
            "deployed": {
                "visualize": False,
                "always_on": False,
                "remove_plugins": ["CameraZoomPlugin", "GstCameraPlugin"],
            },
            "chase": {
                "visualize": False,
                "always_on": False,
                "remove_plugins": [],
            },
        },
    },
}


def default_runtime_profile(drone_count: int) -> str:
    return "single_equivalent"


def deep_merge(base: dict[str, Any], overrides: dict[str, Any]) -> dict[str, Any]:
    merged = copy.deepcopy(base)
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def resolve_runtime_defaults(profile_name: str, overrides: dict[str, Any] | None = None) -> dict[str, Any]:
    normalized = str(profile_name or "").strip()
    if normalized not in RUNTIME_PROFILE_OVERRIDES:
        available = ", ".join(sorted(RUNTIME_PROFILE_OVERRIDES))
        raise RuntimeError(f"Unknown runtime profile {normalized!r}. Available: {available}")

    resolved = deep_merge(BASE_RUNTIME_DEFAULTS, RUNTIME_PROFILE_OVERRIDES[normalized])
    if overrides:
        resolved = deep_merge(resolved, overrides)
    return resolved
