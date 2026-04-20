from __future__ import annotations

import math
from dataclasses import dataclass


EARTH_RADIUS_M = 6378137.0


@dataclass(frozen=True)
class LocalPose:
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class CenterTarget:
    x: float
    y: float
    z: float
    yaw: float


def clamp(value: float, limit: float) -> float:
    if limit <= 0.0:
        return 0.0
    return max(min(value, limit), -limit)


def wrap_angle(angle_rad: float) -> float:
    wrapped = angle_rad
    while wrapped > math.pi:
        wrapped -= 2.0 * math.pi
    while wrapped < -math.pi:
        wrapped += 2.0 * math.pi
    return wrapped


def angle_delta(target_rad: float, current_rad: float) -> float:
    return wrap_angle(target_rad - current_rad)


def rotate_xy(x: float, y: float, yaw_rad: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    return (
        cos_yaw * x - sin_yaw * y,
        sin_yaw * x + cos_yaw * y,
    )


def world_to_body(x_world: float, y_world: float, yaw_rad: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    return (
        cos_yaw * x_world + sin_yaw * y_world,
        -sin_yaw * x_world + cos_yaw * y_world,
    )


def geodetic_to_local_xy(
    latitude_deg: float,
    longitude_deg: float,
    origin_latitude_deg: float,
    origin_longitude_deg: float,
) -> tuple[float, float]:
    lat_rad = math.radians(latitude_deg)
    lon_rad = math.radians(longitude_deg)
    origin_lat_rad = math.radians(origin_latitude_deg)
    origin_lon_rad = math.radians(origin_longitude_deg)
    delta_lat = lat_rad - origin_lat_rad
    delta_lon = lon_rad - origin_lon_rad
    mean_lat = 0.5 * (lat_rad + origin_lat_rad)
    x_east = delta_lon * math.cos(mean_lat) * EARTH_RADIUS_M
    y_north = delta_lat * EARTH_RADIUS_M
    return x_east, y_north


def circular_mean(angles_rad: list[float]) -> float:
    if not angles_rad:
        return 0.0
    sin_sum = sum(math.sin(angle) for angle in angles_rad)
    cos_sum = sum(math.cos(angle) for angle in angles_rad)
    if abs(sin_sum) < 1e-9 and abs(cos_sum) < 1e-9:
        return 0.0
    return math.atan2(sin_sum, cos_sum)
