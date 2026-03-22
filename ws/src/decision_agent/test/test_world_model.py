from decision_agent.world_model import TelemetryState, WorldModel


def test_telemetry_freshness():
    world_model = WorldModel()
    world_model.update_telemetry(TelemetryState(), 10.0)
    assert world_model.telemetry_is_fresh(11.0, 2.0)
    assert not world_model.telemetry_is_fresh(13.5, 2.0)
