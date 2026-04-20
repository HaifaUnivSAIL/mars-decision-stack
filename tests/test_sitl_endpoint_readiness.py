from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / 'scripts' / 'sitl_endpoint_readiness.py'


def load_module():
    spec = importlib.util.spec_from_file_location('sitl_endpoint_readiness', MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_heartbeat_tracker_reaches_ready_after_stable_window() -> None:
    module = load_module()
    tracker = module.HeartbeatStabilityTracker(min_heartbeats=3, max_gap_sec=1.0, settle_window_sec=2.0)

    tracker.record_heartbeat(0.0)
    tracker.record_heartbeat(0.5)
    tracker.record_heartbeat(1.0)

    assert not tracker.is_ready(2.5)
    assert tracker.is_ready(3.0)


def test_heartbeat_tracker_resets_after_large_gap() -> None:
    module = load_module()
    tracker = module.HeartbeatStabilityTracker(min_heartbeats=2, max_gap_sec=1.0, settle_window_sec=1.0)

    tracker.record_heartbeat(0.0)
    tracker.record_heartbeat(0.4)
    assert tracker.consecutive_heartbeats == 2

    tracker.observe_gap(2.0)
    assert tracker.consecutive_heartbeats == 0
    assert tracker.stable_window_reset_count == 1

    tracker.record_heartbeat(2.1)
    tracker.record_heartbeat(2.6)
    assert tracker.is_ready(3.7)
