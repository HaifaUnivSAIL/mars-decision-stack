from manual_runtime_test.runtime_state import (
    HLC_STATE_GAINING_ALTITUDE,
    HLC_STATE_AIRBORNE,
    HLC_STATE_MANUAL,
    HLC_STATE_READY,
    MC_STATE_PERFORMING_MISSION,
    MC_STATE_STANDBY,
    RuntimeState,
    TelemetryState,
)


def test_takeoff_ready_requires_standby_and_ready():
    runtime_state = RuntimeState()
    runtime_state.update_mc_state(MC_STATE_STANDBY)
    runtime_state.update_hlc_state(HLC_STATE_READY)

    assert runtime_state.is_takeoff_ready() is True


def test_takeoff_ready_no_longer_falls_back_to_telemetry_only():
    runtime_state = RuntimeState()
    runtime_state.update_telemetry(TelemetryState(state='STANDBY'), stamp_sec=5.0)

    assert runtime_state.is_takeoff_ready(now_sec=6.0, telemetry_timeout_sec=2.0) is False


def test_telemetry_freshness_uses_timestamp():
    runtime_state = RuntimeState()
    runtime_state.update_telemetry(TelemetryState(altitude=1.0), stamp_sec=10.0)

    assert runtime_state.telemetry_is_fresh(now_sec=11.0, timeout_sec=2.0) is True
    assert runtime_state.telemetry_is_fresh(now_sec=13.5, timeout_sec=2.0) is False


def test_motion_commands_allowed_in_airborne_or_manual():
    runtime_state = RuntimeState()

    assert runtime_state.motion_commands_allowed() is False

    runtime_state.update_hlc_state(HLC_STATE_AIRBORNE)
    assert runtime_state.motion_commands_allowed() is True

    runtime_state.update_hlc_state(HLC_STATE_MANUAL)
    assert runtime_state.motion_commands_allowed() is True

    runtime_state.update_hlc_state(HLC_STATE_READY)
    assert runtime_state.motion_commands_allowed() is False


def test_motion_commands_allowed_in_flight_capable_gaining_altitude():
    runtime_state = RuntimeState()
    runtime_state.update_hlc_state(HLC_STATE_GAINING_ALTITUDE)
    runtime_state.update_telemetry(
        TelemetryState(altitude=0.75, armed=True, state='ACTIVE'),
        stamp_sec=5.0,
    )

    assert runtime_state.motion_commands_allowed(min_altitude_m=0.5) is True


def test_motion_commands_blocked_when_gaining_altitude_but_not_yet_flying():
    runtime_state = RuntimeState()
    runtime_state.update_hlc_state(HLC_STATE_GAINING_ALTITUDE)
    runtime_state.update_telemetry(
        TelemetryState(altitude=0.10, armed=True, state='ACTIVE'),
        stamp_sec=5.0,
    )

    assert runtime_state.motion_commands_allowed(min_altitude_m=0.5) is False


def test_motion_commands_allowed_in_performing_mission_with_active_flight():
    runtime_state = RuntimeState()
    runtime_state.update_mc_state(MC_STATE_PERFORMING_MISSION)
    runtime_state.update_hlc_state(HLC_STATE_READY)
    runtime_state.update_telemetry(
        TelemetryState(altitude=0.80, armed=True, state='ACTIVE'),
        stamp_sec=5.0,
    )

    assert runtime_state.motion_commands_allowed(min_altitude_m=0.5) is True


def test_clear_command_reference_resets_motion_debug_state():
    runtime_state = RuntimeState()
    runtime_state.update_telemetry(TelemetryState(latitude=1.0, longitude=2.0, altitude=3.0), stamp_sec=5.0)
    runtime_state.mark_command_reference('forward', now_sec=6.0)

    runtime_state.clear_command_reference()

    assert runtime_state.last_command_label == ''
    assert runtime_state.last_command_time is None
    assert runtime_state.command_reference_telemetry is None
