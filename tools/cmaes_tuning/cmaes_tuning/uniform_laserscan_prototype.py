"""Explicit legacy/prototype LaserScan contracts for simulator-only auditing.

This module is diagnostic support.  It is deliberately not imported by the
production obstacle detector, planner, controller, or state machine.  Callers
must name the contract mode so recorded legacy scans cannot be silently
reinterpreted as prototype scans.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import struct

import numpy as np

from .lidar_beam_contract import BackendBeamContract


class LaserScanContractMode(str, Enum):
    LEGACY_QUANTIZED_2000 = "LEGACY_QUANTIZED_2000"
    PROTOTYPE_UNIFORM_INCLUSIVE = "PROTOTYPE_UNIFORM_INCLUSIVE"


# Public audit-mode names used by migration plans and bag readers.  They are
# aliases, not implicit defaults: every caller still has to pass one mode.
LEGACY_SCAN_MODE = LaserScanContractMode.LEGACY_QUANTIZED_2000
PROTOTYPE_UNIFORM_SCAN_MODE = LaserScanContractMode.PROTOTYPE_UNIFORM_INCLUSIVE


def float32(value: float) -> float:
    """Round exactly as a ROS 2 ``LaserScan`` float32 metadata field does."""

    return struct.unpack("<f", struct.pack("<f", float(value)))[0]


@dataclass(frozen=True)
class UniformLaserScanContract:
    """Inclusive-endpoint uniform physical and published angular contract."""

    beam_count: int
    angle_min_rad: float
    angle_max_rad: float
    angle_increment_rad: float

    @classmethod
    def from_fov(cls, field_of_view_rad: float, beam_count: int) -> "UniformLaserScanContract":
        field_of_view_rad = float(field_of_view_rad)
        beam_count = int(beam_count)
        if not math.isfinite(field_of_view_rad) or field_of_view_rad <= 0.0:
            raise ValueError("field_of_view_rad must be finite and positive")
        if beam_count < 2:
            raise ValueError("beam_count must be at least two")
        angle_min = float32(-0.5 * field_of_view_rad)
        increment = float32(field_of_view_rad / float(beam_count - 1))
        # Publish the endpoint generated from the same float32 minimum and
        # increment consumed by the C++ backend.  The final float32 rounding is
        # part of the message representation, not an independent definition.
        angle_max = float32(angle_min + (beam_count - 1) * increment)
        return cls(beam_count, angle_min, angle_max, increment)

    @property
    def physical_fov_rad(self) -> float:
        return (self.beam_count - 1) * self.angle_increment_rad

    def physical_relative_angles_rad(self) -> np.ndarray:
        return self.angle_min_rad + np.arange(self.beam_count, dtype=np.float64) * self.angle_increment_rad

    def published_relative_angles_rad(self) -> np.ndarray:
        # ROS consumers use this exact formula.
        return self.angle_min_rad + np.arange(self.beam_count, dtype=np.float64) * self.angle_increment_rad

    def physical_world_angles_rad(self, yaw_rad: float) -> np.ndarray:
        if not math.isfinite(yaw_rad):
            raise ValueError("yaw_rad must be finite")
        return yaw_rad + self.physical_relative_angles_rad()

    def directions(self, yaw_rad: float) -> np.ndarray:
        angles = self.physical_world_angles_rad(yaw_rad)
        return np.column_stack((np.cos(angles), np.sin(angles)))


def physical_angles_for_mode(
    mode: LaserScanContractMode,
    field_of_view_rad: float,
    beam_count: int,
    yaw_rad: float,
) -> np.ndarray:
    """Return physical rays only after an explicit legacy/prototype choice."""

    mode = LaserScanContractMode(mode)
    if mode is LaserScanContractMode.LEGACY_QUANTIZED_2000:
        return BackendBeamContract(field_of_view_rad, beam_count).physical_angles_rad(yaw_rad)
    return UniformLaserScanContract.from_fov(
        field_of_view_rad, beam_count
    ).physical_world_angles_rad(yaw_rad)


def uniform_noise_free_scan(model, x_m: float, y_m: float, yaw_rad: float) -> np.ndarray:
    """Ray march the audit raster with the prototype's continuous angles."""

    contract = UniformLaserScanContract.from_fov(model.scan_fov, model.scan_beams)
    directions = contract.directions(yaw_rad)
    lidar_x = x_m + model.lidar_offset * math.cos(yaw_rad)
    lidar_y = y_m + model.lidar_offset * math.sin(yaw_rad)
    ray_x = np.full(model.scan_beams, lidar_x, dtype=np.float64)
    ray_y = np.full(model.scan_beams, lidar_y, dtype=np.float64)
    distance = model._distance_lookup(ray_x, ray_y)
    total = distance.copy()
    for _ in range(4096):
        active = (distance > model.ray_epsilon) & (total <= model.maximum_range)
        if not np.any(active):
            break
        ray_x[active] += distance[active] * directions[active, 0]
        ray_y[active] += distance[active] * directions[active, 1]
        next_distance = model._distance_lookup(ray_x, ray_y)
        total[active] += next_distance[active]
        distance = next_distance
    else:
        raise RuntimeError("prototype uniform ray march failed to converge")
    return np.minimum(total, model.maximum_range)


def oriented_rectangle_ranges(
    origin_xy: np.ndarray,
    directions: np.ndarray,
    centre_xy: np.ndarray,
    rectangle_yaw_rad: float,
    width_m: float,
    height_m: float,
    maximum_range_m: float = 30.0,
) -> np.ndarray:
    """Deterministic analytic scan against one oriented rectangular target."""

    origin = np.asarray(origin_xy, dtype=np.float64)
    directions = np.asarray(directions, dtype=np.float64)
    centre = np.asarray(centre_xy, dtype=np.float64)
    cosine, sine = math.cos(rectangle_yaw_rad), math.sin(rectangle_yaw_rad)
    delta = origin - centre
    local_origin = np.asarray(
        [delta[0] * cosine + delta[1] * sine, -delta[0] * sine + delta[1] * cosine]
    )
    local_x = directions[:, 0] * cosine + directions[:, 1] * sine
    local_y = -directions[:, 0] * sine + directions[:, 1] * cosine
    enter = np.zeros(directions.shape[0], dtype=np.float64)
    leave = np.full(directions.shape[0], math.inf, dtype=np.float64)
    valid = np.ones(directions.shape[0], dtype=bool)
    for local_o, local_d, lower, upper in (
        (local_origin[0], local_x, -0.5 * width_m, 0.5 * width_m),
        (local_origin[1], local_y, -0.5 * height_m, 0.5 * height_m),
    ):
        parallel = np.abs(local_d) <= 1.0e-12
        valid &= ~parallel | ((local_o >= lower) & (local_o <= upper))
        safe = np.where(parallel, 1.0, local_d)
        first = np.where(parallel, -math.inf, (lower - local_o) / safe)
        second = np.where(parallel, math.inf, (upper - local_o) / safe)
        enter = np.maximum(enter, np.minimum(first, second))
        leave = np.minimum(leave, np.maximum(first, second))
    valid &= (leave >= enter) & (leave >= 0.0) & (enter <= maximum_range_m)
    return np.where(valid, enter, maximum_range_m)
