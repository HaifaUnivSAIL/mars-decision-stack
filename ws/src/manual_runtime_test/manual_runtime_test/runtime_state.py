from dataclasses import dataclass, field
from typing import Optional


MC_STATE_STANDBY = 2
HLC_STATE_READY = 4

MC_STATE_NAMES = {
    0: 'None',
    1: 'Init',
    2: 'Standby',
    3: 'TakingOff',
    4: 'PerformingMission',
    5: 'Landing',
    6: 'Manual',
    7: 'ReturnToLaunch',
    8: 'NoError',
    9: 'Error',
}

HLC_STATE_NAMES = {
    0: 'None',
    1: 'Init',
    2: 'WaitingForMc',
    3: 'WaitingForLlc',
    4: 'Ready',
    5: 'Takeoff',
    6: 'GainingAltitude',
    7: 'Airborne',
    8: 'Landing',
    9: 'Manual',
    10: 'ReturnToLaunch',
    11: 'NoError',
    12: 'LowBattery',
    13: 'LlcDown',
}


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
    mode: str = ''
    state: str = ''


@dataclass
class RuntimeState:
    mc_state: Optional[int] = None
    hlc_state: Optional[int] = None
    telemetry: TelemetryState = field(default_factory=TelemetryState)
    last_error: str = ''
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

    def is_takeoff_ready(self, now_sec: Optional[float] = None, telemetry_timeout_sec: Optional[float] = None) -> bool:
        if self.hlc_state == HLC_STATE_READY and self.mc_state == MC_STATE_STANDBY:
            return True

        if now_sec is None or telemetry_timeout_sec is None:
            return False

        if not self.telemetry_is_fresh(now_sec, telemetry_timeout_sec):
            return False

        return self.telemetry.state.upper() == 'STANDBY'

    def mc_state_name(self) -> str:
        return MC_STATE_NAMES.get(self.mc_state, str(self.mc_state))

    def hlc_state_name(self) -> str:
        return HLC_STATE_NAMES.get(self.hlc_state, str(self.hlc_state))

    def status_line(self, now_sec: float, timeout_sec: float) -> str:
        telemetry_fresh = self.telemetry_is_fresh(now_sec, timeout_sec)
        return (
            f"MC={self.mc_state_name()} "
            f"HLC={self.hlc_state_name()} "
            f"alt={self.telemetry.altitude:.2f} "
            f"armed={self.telemetry.armed} "
            f"mode={self.telemetry.mode or '-'} "
            f"state={self.telemetry.state or '-'} "
            f"gps_fix={self.telemetry.gps_fix} "
            f"telemetry_fresh={telemetry_fresh}"
        )
