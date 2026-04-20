from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from .swarm_geometry import LocalPose, geodetic_to_local_xy
from .world_model import (
    HLC_STATE_AIRBORNE,
    HLC_STATE_GAINING_ALTITUDE,
    HLC_STATE_READY,
    MC_STATE_STANDBY,
    TelemetryState,
    WorldModel,
)


HLC_STATE_LOW_BATTERY = 12
HLC_STATE_LLC_DOWN = 13
MC_STATE_ERROR = 9


@dataclass
class LocalOrigin:
    latitude: float
    longitude: float
    altitude: float


@dataclass
class SwarmDroneState:
    runtime: WorldModel = field(default_factory=WorldModel)
    local_pose: Optional[LocalPose] = None
    last_local_pose_time: Optional[float] = None


class SwarmWorldModel:
    def __init__(self, drone_ids: list[str] | None = None):
        self._drones: dict[str, SwarmDroneState] = {}
        self._origin: Optional[LocalOrigin] = None
        for drone_id in drone_ids or []:
            self.ensure_drone(drone_id)

    def ensure_drone(self, drone_id: str) -> SwarmDroneState:
        if drone_id not in self._drones:
            self._drones[drone_id] = SwarmDroneState()
        return self._drones[drone_id]

    def drone_ids(self) -> list[str]:
        return sorted(self._drones.keys())

    def drone_state(self, drone_id: str) -> SwarmDroneState:
        return self.ensure_drone(drone_id)

    def set_local_origin(self, latitude: float, longitude: float, altitude: float) -> None:
        self._origin = LocalOrigin(latitude=latitude, longitude=longitude, altitude=altitude)
        for drone_id in self.drone_ids():
            self._refresh_local_pose(drone_id)

    def set_local_origin_from_drone(self, drone_id: str) -> bool:
        telemetry = self.ensure_drone(drone_id).runtime.telemetry
        if telemetry.latitude == 0.0 and telemetry.longitude == 0.0:
            return False
        self.set_local_origin(telemetry.latitude, telemetry.longitude, telemetry.altitude)
        return True

    def has_local_origin(self) -> bool:
        return self._origin is not None

    def local_origin(self) -> Optional[LocalOrigin]:
        return self._origin

    def update_mc_state(self, drone_id: str, state: int) -> None:
        self.ensure_drone(drone_id).runtime.update_mc_state(state)

    def update_hlc_state(self, drone_id: str, state: int) -> None:
        self.ensure_drone(drone_id).runtime.update_hlc_state(state)

    def update_error(self, drone_id: str, error_text: str) -> None:
        self.ensure_drone(drone_id).runtime.update_error(error_text)

    def update_telemetry(self, drone_id: str, telemetry: TelemetryState, stamp_sec: float) -> None:
        state = self.ensure_drone(drone_id)
        state.runtime.update_telemetry(telemetry, stamp_sec)
        self._refresh_local_pose(drone_id)

    def local_pose(self, drone_id: str) -> Optional[LocalPose]:
        return self.ensure_drone(drone_id).local_pose

    def telemetry_is_fresh(self, drone_id: str, now_sec: float, timeout_sec: float) -> bool:
        return self.ensure_drone(drone_id).runtime.telemetry_is_fresh(now_sec, timeout_sec)

    def all_runtime_ready(self, drone_ids: list[str], now_sec: float, timeout_sec: float, require_ready_state: bool = True) -> bool:
        return all(
            self.ensure_drone(drone_id).runtime.is_runtime_ready(now_sec, timeout_sec, require_ready_state)
            for drone_id in drone_ids
        )

    def all_airborne(self, drone_ids: list[str], now_sec: float, timeout_sec: float, min_altitude_m: float) -> bool:
        return all(
            self.ensure_drone(drone_id).runtime.is_airborne(now_sec, timeout_sec, min_altitude_m)
            for drone_id in drone_ids
        )

    def all_landed(self, drone_ids: list[str], now_sec: float, timeout_sec: float, max_altitude_m: float = 0.3) -> bool:
        return all(
            self.ensure_drone(drone_id).runtime.is_landed(now_sec, timeout_sec, max_altitude_m)
            for drone_id in drone_ids
        )

    def first_blocking_issue(self, drone_ids: list[str], now_sec: float, timeout_sec: float) -> tuple[str, str] | None:
        for drone_id in drone_ids:
            state = self.ensure_drone(drone_id)
            if not state.runtime.telemetry_is_fresh(now_sec, timeout_sec):
                return drone_id, "telemetry_stale"
            if state.runtime.mc_state == MC_STATE_ERROR:
                return drone_id, "mission_control_error"
            if state.runtime.hlc_state in {HLC_STATE_LOW_BATTERY, HLC_STATE_LLC_DOWN}:
                return drone_id, "high_level_control_error"
            if state.runtime.last_error:
                return drone_id, "platform_error"
        return None

    def _refresh_local_pose(self, drone_id: str) -> None:
        state = self.ensure_drone(drone_id)
        if self._origin is None:
            state.local_pose = None
            state.last_local_pose_time = None
            return
        telemetry = state.runtime.telemetry
        if telemetry.latitude == 0.0 and telemetry.longitude == 0.0:
            state.local_pose = None
            state.last_local_pose_time = None
            return
        x_east, y_north = geodetic_to_local_xy(
            telemetry.latitude,
            telemetry.longitude,
            self._origin.latitude,
            self._origin.longitude,
        )
        state.local_pose = LocalPose(
            x=x_east,
            y=y_north,
            z=telemetry.altitude - self._origin.altitude,
            yaw=telemetry.yaw,
        )
        state.last_local_pose_time = state.runtime.last_telemetry_time
