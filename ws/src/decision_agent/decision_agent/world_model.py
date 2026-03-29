from dataclasses import dataclass, field
from typing import Optional

MC_STATE_STANDBY = 2
MC_STATE_PERFORMINGMISSION = 4
HLC_STATE_READY = 4
HLC_STATE_TAKEOFF = 5
HLC_STATE_GAINING_ALTITUDE = 6
HLC_STATE_AIRBORNE = 7


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

    def is_runtime_ready(self, now_sec: float, timeout_sec: float, require_ready_state: bool = True) -> bool:
        if not self.telemetry_is_fresh(now_sec, timeout_sec):
            return False
        if require_ready_state:
            return self.hlc_state == HLC_STATE_READY and self.mc_state == MC_STATE_STANDBY
        if self.hlc_state == HLC_STATE_READY and self.mc_state == MC_STATE_STANDBY:
            return True
        return self.telemetry.state.upper() == 'STANDBY'

    def is_airborne(self, now_sec: float, timeout_sec: float, min_altitude_m: float) -> bool:
        if not self.telemetry_is_fresh(now_sec, timeout_sec):
            return False
        if not self.telemetry.armed:
            return False
        if self.telemetry.altitude < min_altitude_m:
            return False
        return self.hlc_state in (HLC_STATE_GAINING_ALTITUDE, HLC_STATE_AIRBORNE) or self.telemetry.state.upper() == 'ACTIVE'

    def is_landed(self, now_sec: float, timeout_sec: float, max_altitude_m: float = 0.3) -> bool:
        if not self.telemetry_is_fresh(now_sec, timeout_sec):
            return False
        if self.telemetry.armed:
            return False
        return self.telemetry.altitude <= max_altitude_m
