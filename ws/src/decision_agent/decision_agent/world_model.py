from dataclasses import dataclass, field
from typing import Optional


@dataclass
class TelemetryState:
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    yaw: float = 0.0
    armed: bool = False
    battery_voltage: float = 0.0
    gps_fix: int = 0
    gps_hdop: float = 0.0
    mode: str = ""
    state: str = ""


@dataclass
class WorldModel:
    mc_state: Optional[int] = None
    hlc_state: Optional[int] = None
    telemetry: TelemetryState = field(default_factory=TelemetryState)
    last_error: str = ""
    last_telemetry_time: Optional[float] = None

    def update_mc_state(self, state: int) -> None:
        self.mc_state = state

    def update_hlc_state(self, state: int) -> None:
        self.hlc_state = state

    def update_telemetry(self, telemetry: TelemetryState, stamp_sec: float) -> None:
        self.telemetry = telemetry
        self.last_telemetry_time = stamp_sec

    def update_error(self, error_text: str) -> None:
        self.last_error = error_text

    def telemetry_is_fresh(self, now_sec: float, timeout_sec: float) -> bool:
        if self.last_telemetry_time is None:
            return False
        return (now_sec - self.last_telemetry_time) <= timeout_sec
