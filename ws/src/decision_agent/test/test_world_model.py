from decision_agent.world_model import TelemetryState, WorldModel


def test_telemetry_freshness():
    world_model = WorldModel()
    world_model.update_telemetry(TelemetryState(), 10.0)
    assert world_model.telemetry_is_fresh(11.0, 2.0)
    assert not world_model.telemetry_is_fresh(13.5, 2.0)


def test_runtime_ready_uses_state_and_fresh_telemetry():
    world_model = WorldModel(mc_state=2, hlc_state=4)
    world_model.update_telemetry(TelemetryState(state='STANDBY'), 10.0)
    assert world_model.is_runtime_ready(11.0, 2.0)


def test_airborne_requires_fresh_armed_altitude():
    world_model = WorldModel(mc_state=4, hlc_state=7)
    world_model.update_telemetry(TelemetryState(altitude=3.2, armed=True, state='ACTIVE'), 10.0)
    assert world_model.is_airborne(11.0, 2.0, 3.0)


def test_landed_requires_disarmed_low_altitude():
    world_model = WorldModel()
    world_model.update_telemetry(TelemetryState(altitude=0.2, armed=False), 10.0)
    assert world_model.is_landed(11.0, 2.0)
