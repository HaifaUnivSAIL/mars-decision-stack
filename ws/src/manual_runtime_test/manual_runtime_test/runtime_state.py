import math
from dataclasses import dataclass, field
from typing import Optional


MC_STATE_STANDBY = 2
MC_STATE_PERFORMING_MISSION = 4
HLC_STATE_READY = 4
HLC_STATE_GAINING_ALTITUDE = 6
HLC_STATE_AIRBORNE = 7
HLC_STATE_MANUAL = 9

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
    command_reference_telemetry: Optional[TelemetryState] = None
    last_command_label: str = ''
    last_command_time: Optional[float] = None

    def update_mc_state(self, state: int) -> None:
        self.mc_state = state

    def update_hlc_state(self, state: int) -> None:
        self.hlc_state = state

    def update_telemetry(self, telemetry: TelemetryState, stamp_sec: float) -> None:
        self.telemetry = telemetry
        self.last_telemetry_time = stamp_sec

    def update_error(self, error_text: str) -> None:
        self.last_error = error_text

    def mark_command_reference(self, label: str, now_sec: float) -> None:
        self.last_command_label = label
        self.last_command_time = now_sec
        self.command_reference_telemetry = TelemetryState(
            latitude=self.telemetry.latitude,
            longitude=self.telemetry.longitude,
            altitude=self.telemetry.altitude,
            yaw=self.telemetry.yaw,
            armed=self.telemetry.armed,
            battery_voltage=self.telemetry.battery_voltage,
            gps_fix=self.telemetry.gps_fix,
            gps_hdop=self.telemetry.gps_hdop,
            mode=self.telemetry.mode,
            state=self.telemetry.state,
        )

    def clear_command_reference(self) -> None:
        self.command_reference_telemetry = None
        self.last_command_label = ''
        self.last_command_time = None

    def telemetry_is_fresh(self, now_sec: float, timeout_sec: float) -> bool:
        if self.last_telemetry_time is None:
            return False
        return (now_sec - self.last_telemetry_time) <= timeout_sec

    def has_complete_status_snapshot(self, now_sec: float, timeout_sec: float) -> bool:
        return (
            self.mc_state is not None and
            self.hlc_state is not None and
            self.telemetry_is_fresh(now_sec, timeout_sec)
        )

    def is_takeoff_ready(self, now_sec: Optional[float] = None, telemetry_timeout_sec: Optional[float] = None) -> bool:
        return self.hlc_state == HLC_STATE_READY and self.mc_state == MC_STATE_STANDBY

    def motion_commands_allowed(self, min_altitude_m: float = 0.5) -> bool:
        if self.hlc_state in {HLC_STATE_AIRBORNE, HLC_STATE_MANUAL}:
            return True

        flight_capable = (
            self.telemetry.armed and
            self.telemetry.state.upper() == 'ACTIVE' and
            self.telemetry.altitude >= min_altitude_m
        )
        if not flight_capable:
            return False

        return self.hlc_state == HLC_STATE_GAINING_ALTITUDE or self.mc_state == MC_STATE_PERFORMING_MISSION

    def mc_state_name(self) -> str:
        return MC_STATE_NAMES.get(self.mc_state, str(self.mc_state))

    def hlc_state_name(self) -> str:
        return HLC_STATE_NAMES.get(self.hlc_state, str(self.hlc_state))

    def status_line(self, now_sec: float, timeout_sec: float, motion_debug_window_sec: float = 0.0) -> str:
        telemetry_fresh = self.telemetry_is_fresh(now_sec, timeout_sec)
        status = (
            f"MC={self.mc_state_name()} "
            f"HLC={self.hlc_state_name()} "
            f"alt={self.telemetry.altitude:.2f} "
            f"yaw={math.degrees(self.telemetry.yaw):.1f}deg "
            f"armed={self.telemetry.armed} "
            f"mode={self.telemetry.mode or '-'} "
            f"state={self.telemetry.state or '-'} "
            f"gps_fix={self.telemetry.gps_fix} "
            f"telemetry_fresh={telemetry_fresh}"
        )

        if (
            motion_debug_window_sec > 0.0 and
            self.command_reference_telemetry is not None and
            self.last_command_time is not None and
            (now_sec - self.last_command_time) <= motion_debug_window_sec and
            telemetry_fresh
        ):
            lat_scale = 111320.0
            lon_scale = 111320.0 * math.cos(math.radians(self.command_reference_telemetry.latitude))
            delta_north = (self.telemetry.latitude - self.command_reference_telemetry.latitude) * lat_scale
            delta_east = (self.telemetry.longitude - self.command_reference_telemetry.longitude) * lon_scale
            delta_alt = self.telemetry.altitude - self.command_reference_telemetry.altitude
            delta_yaw = math.degrees(self.telemetry.yaw - self.command_reference_telemetry.yaw)

            status += (
                f" cmd={self.last_command_label or '-'} "
                f"dN={delta_north:+.2f}m "
                f"dE={delta_east:+.2f}m "
                f"dAlt={delta_alt:+.2f}m "
                f"dYaw={delta_yaw:+.1f}deg"
            )

        return status
