"""Diagnostic contract for the simulator's physical LiDAR beam lookup.

This module deliberately keeps the range-generating backend angle separate from
the angle advertised by ``sensor_msgs/LaserScan``.  It is audit/test support;
production publishers and consumers do not import it.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import re
from typing import Iterable

import numpy as np


TWO_PI = 2.0 * math.pi


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _matching_line(path: Path, pattern: str) -> tuple[int, str]:
    expression = re.compile(pattern)
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if expression.search(line):
            return number, line.strip()
    raise RuntimeError(f"required beam-contract source expression absent: {path}: {pattern}")


def locate_simulator_sources() -> tuple[Path, Path]:
    """Locate the installed simulator backend and its ROS LaserScan publisher."""

    import f110_gym  # Imported lazily so pure formula users do not need the simulator.

    package = Path(f110_gym.__file__).resolve().parent
    backend = package / "cpp_backend.cpp"
    project = package.parents[1]
    publisher = project / "f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py"
    if not backend.is_file() or not publisher.is_file():
        raise RuntimeError(f"simulator source tree incomplete: {backend}, {publisher}")
    return backend, publisher


def inspect_authoritative_sources(
    backend_path: Path | None = None,
    publisher_path: Path | None = None,
) -> dict[str, object]:
    """Read the actual source and extract the executable beam-contract evidence."""

    if backend_path is None or publisher_path is None:
        located_backend, located_publisher = locate_simulator_sources()
        backend_path = backend_path or located_backend
        publisher_path = publisher_path or located_publisher

    theta_line, theta_text = _matching_line(backend_path, r"int\s+theta_dis_\s*=\s*\d+")
    theta_match = re.search(r"=\s*(\d+)", theta_text)
    if theta_match is None:
        raise RuntimeError("failed to parse theta_dis_ from authoritative backend")
    theta_discretization = int(theta_match.group(1))
    increment_line, increment_text = _matching_line(
        backend_path, r"angle_increment_\(fov\s*/.*num_beams\s*-\s*1"
    )
    table_line, table_text = _matching_line(
        backend_path, r"2\.0\s*\*\s*kPi\s*/.*theta_dis_\s*-\s*1"
    )
    initial_line, initial_text = _matching_line(
        backend_path, r"theta_dis_\s*\*\s*\(pose\[2\]\s*-\s*fov_\s*/\s*2\.0\)"
    )
    conversion_line, conversion_text = _matching_line(
        backend_path, r"static_cast<int>\(theta_index\)"
    )
    wrap_line, wrap_text = _matching_line(backend_path, r"std::fmod\(theta_index, theta_dis_\)")
    publisher_line, publisher_text = _matching_line(
        publisher_path, r"self\.angle_inc\s*=\s*scan_fov\s*/\s*scan_beams"
    )
    message_line, message_text = _matching_line(
        publisher_path, r"scan\.angle_increment\s*=\s*self\.angle_inc"
    )
    return {
        "backend_path": str(backend_path),
        "backend_sha256": _sha256(backend_path),
        "publisher_path": str(publisher_path),
        "publisher_sha256": _sha256(publisher_path),
        "theta_discretization": theta_discretization,
        "backend_increment_line": increment_line,
        "backend_increment_source": increment_text,
        "backend_table_line": table_line,
        "backend_table_source": table_text,
        "backend_initial_line": initial_line,
        "backend_initial_source": initial_text,
        "backend_conversion_line": conversion_line,
        "backend_conversion_source": conversion_text,
        "backend_wrap_line": wrap_line,
        "backend_wrap_source": wrap_text,
        "publisher_increment_line": publisher_line,
        "publisher_increment_source": publisher_text,
        "publisher_message_line": message_line,
        "publisher_message_source": message_text,
    }


@dataclass(frozen=True)
class BackendBeamContract:
    """Exact index-to-table-bin contract mirrored from ``cpp_backend.cpp``."""

    scan_fov_rad: float
    scan_beams: int
    theta_discretization: int = 2000

    def __post_init__(self) -> None:
        if not math.isfinite(self.scan_fov_rad) or self.scan_fov_rad <= 0.0:
            raise ValueError("scan_fov_rad must be finite and positive")
        if self.scan_beams < 2:
            raise ValueError("scan_beams must be at least two")
        if self.theta_discretization < 2:
            raise ValueError("theta_discretization must be at least two")

    @property
    def continuous_angle_increment_rad(self) -> float:
        return self.scan_fov_rad / float(self.scan_beams - 1)

    @property
    def theta_index_increment(self) -> float:
        return (
            self.theta_discretization
            * self.continuous_angle_increment_rad
            / TWO_PI
        )

    def table_angle_rad(self, table_index: int) -> float:
        if not 0 <= table_index < self.theta_discretization:
            raise IndexError(table_index)
        return table_index * TWO_PI / float(self.theta_discretization - 1)

    def table_indices(self, yaw_rad: float) -> np.ndarray:
        """Mirror the C++ sequential accumulation, wrapping, and int truncation."""

        if not math.isfinite(yaw_rad):
            raise ValueError("yaw_rad must be finite")
        theta_index = (
            self.theta_discretization
            * (yaw_rad - 0.5 * self.scan_fov_rad)
            / TWO_PI
        )
        theta_index = math.fmod(theta_index, self.theta_discretization)
        while theta_index < 0.0:
            theta_index += self.theta_discretization
        result = np.empty(self.scan_beams, dtype=np.int64)
        for index in range(self.scan_beams):
            # C++ static_cast<int> truncates toward zero.  The value is normalized
            # non-negative before this cast, so this is also floor.
            result[index] = int(theta_index)
            theta_index += self.theta_index_increment
            while theta_index >= self.theta_discretization:
                theta_index -= self.theta_discretization
        return result

    def physical_angles_rad(self, yaw_rad: float) -> np.ndarray:
        indices = self.table_indices(yaw_rad)
        return indices.astype(np.float64) * TWO_PI / float(self.theta_discretization - 1)

    def directions(self, yaw_rad: float) -> np.ndarray:
        angles = self.physical_angles_rad(yaw_rad)
        return np.column_stack((np.cos(angles), np.sin(angles)))

    def scan_indices_for_backend_bin(self, yaw_rad: float, table_index: int) -> tuple[int, ...]:
        return tuple(int(value) for value in np.flatnonzero(self.table_indices(yaw_rad) == table_index))


def metadata_angles_rad(
    angle_min_rad: float,
    angle_increment_rad: float,
    ranges_size: int,
    *,
    yaw_rad: float = 0.0,
) -> np.ndarray:
    return (
        yaw_rad
        + angle_min_rad
        + np.arange(ranges_size, dtype=np.float64) * angle_increment_rad
    )


def old_analytic_angles_rad(
    scan_fov_rad: float,
    scan_beams: int,
    *,
    yaw_rad: float = 0.0,
) -> np.ndarray:
    return yaw_rad + np.linspace(-0.5 * scan_fov_rad, 0.5 * scan_fov_rad, scan_beams)


def wrapped_angle_delta_rad(first: np.ndarray | float, second: np.ndarray | float) -> np.ndarray:
    first_values = np.asarray(first, dtype=np.float64)
    second_values = np.asarray(second, dtype=np.float64)
    return np.arctan2(np.sin(first_values - second_values), np.cos(first_values - second_values))


def endpoint_xy(
    angles_rad: np.ndarray | Iterable[float],
    ranges_m: np.ndarray | Iterable[float],
    *,
    origin_x_m: float = 0.0,
    origin_y_m: float = 0.0,
) -> np.ndarray:
    angles = np.asarray(angles_rad, dtype=np.float64)
    ranges = np.asarray(ranges_m, dtype=np.float64)
    return np.column_stack(
        (origin_x_m + ranges * np.cos(angles), origin_y_m + ranges * np.sin(angles))
    )
