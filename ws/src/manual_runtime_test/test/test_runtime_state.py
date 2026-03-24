from manual_runtime_test.runtime_state import HLC_STATE_READY, MC_STATE_STANDBY, RuntimeState, TelemetryState


def test_takeoff_ready_requires_standby_and_ready():
    runtime_state = RuntimeState()
    runtime_state.update_mc_state(MC_STATE_STANDBY)
    runtime_state.update_hlc_state(HLC_STATE_READY)

    assert runtime_state.is_takeoff_ready() is True


def test_takeoff_ready_can_fallback_to_fresh_standby_telemetry():
    runtime_state = RuntimeState()
    runtime_state.update_telemetry(TelemetryState(state='STANDBY'), stamp_sec=5.0)

    assert runtime_state.is_takeoff_ready(now_sec=6.0, telemetry_timeout_sec=2.0) is True


def test_telemetry_freshness_uses_timestamp():
    runtime_state = RuntimeState()
    runtime_state.update_telemetry(TelemetryState(altitude=1.0), stamp_sec=10.0)

    assert runtime_state.telemetry_is_fresh(now_sec=11.0, timeout_sec=2.0) is True
    assert runtime_state.telemetry_is_fresh(now_sec=13.5, timeout_sec=2.0) is False
