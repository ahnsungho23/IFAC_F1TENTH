"""Independent reproduction of the f110_gym map-collision convention.

The simulator checks map collision with noisy LiDAR TTC, not with a continuous
manifest polygon. Recorded scans reproduce that check exactly. Legacy bags that
do not contain scans use a deterministic raster footprint swept over the TTC
horizon and expanded by the configured scan-noise envelope.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml

from .geometry import convex_polygons_intersect, oriented_rectangle, polygon_distance


Point = tuple[float, float]


def repair_collision_yaw_reset(
    samples: list[dict[str, float]], configuration: dict[str, Any]
) -> tuple[list[dict[str, float]], dict[str, Any]]:
    """Repair f110_gym's collision-frame yaw reset without consulting the collision topic.

    cpp_backend::check_ttc currently zeros state indices 3..6, which includes
    yaw at index 4. An impossible one-step speed drop combined with yaw=0 and a
    short forward pose step uniquely identifies that corrupted collision frame.
    """

    repaired = [dict(sample, collision_speed=float(sample["speed"])) for sample in samples]
    tolerance = float(configuration["yaw_reset_tolerance_rad"])
    minimum_speed = float(configuration["minimum_pre_reset_speed_mps"])
    maximum_step = float(configuration["maximum_reset_step_m"])
    previous_moving: dict[str, float] | None = None
    moving_before_previous: dict[str, float] | None = None
    previous_moving_state: tuple[float, float, float, float] | None = None
    frozen_pose: tuple[float, float] | None = None
    recovered_yaw: float | None = None
    diagnostic: dict[str, Any] = {
        "detected": False,
        "event_timestamp_ns": None,
        "recorded_yaw_rad": None,
        "recovered_yaw_rad": None,
        "yaw_increment_rad": None,
        "yaw_recovery_method": None,
        "pre_reset_speed_mps": None,
        "recovered_collision_speed_mps": None,
        "speed_increment_mps": None,
        "step_distance_m": None,
        "repaired_sample_count": 0,
        "cause": "f110_gym_check_ttc_zeroes_state_indices_3_through_6",
    }
    for sample in repaired:
        if frozen_pose is not None:
            stationary = float(sample["speed"]) <= 1.0e-9
            same_pose = math.hypot(
                float(sample["x"]) - frozen_pose[0], float(sample["y"]) - frozen_pose[1]
            ) <= 1.0e-9
            if stationary and same_pose and abs(float(sample["yaw"])) <= tolerance:
                sample["yaw"] = float(recovered_yaw)
                diagnostic["repaired_sample_count"] += 1
                continue
            frozen_pose = None
            recovered_yaw = None

        if previous_moving is not None:
            step_distance = math.hypot(
                float(sample["x"]) - float(previous_moving["x"]),
                float(sample["y"]) - float(previous_moving["y"]),
            )
            impossible_stop = (
                float(sample["speed"]) <= 1.0e-9
                and abs(float(sample["yaw"])) <= tolerance
                and 1.0e-9 < step_distance <= maximum_step
            )
            if impossible_stop:
                raw_yaw = float(sample["yaw"])
                if moving_before_previous is not None:
                    yaw_increment = math.atan2(
                        math.sin(
                            float(previous_moving["yaw"])
                            - float(moving_before_previous["yaw"])
                        ),
                        math.cos(
                            float(previous_moving["yaw"])
                            - float(moving_before_previous["yaw"])
                        ),
                    )
                    recovery_method = "last_unique_physics_yaw_increment"
                    speed_increment = (
                        float(previous_moving["speed"])
                        - float(moving_before_previous["speed"])
                    )
                else:
                    yaw_increment = float(previous_moving.get("yaw_rate", 0.0)) * float(
                        configuration.get("physics_timestep_sec", 0.0)
                    )
                    recovery_method = "previous_yaw_rate_times_physics_step"
                    speed_increment = 0.0
                recovered_yaw = math.atan2(
                    math.sin(float(previous_moving["yaw"]) + yaw_increment),
                    math.cos(float(previous_moving["yaw"]) + yaw_increment),
                )
                sample["yaw"] = recovered_yaw
                collision_speed = float(previous_moving["speed"]) + speed_increment
                sample["collision_speed"] = collision_speed
                frozen_pose = (float(sample["x"]), float(sample["y"]))
                diagnostic.update(
                    {
                        "detected": True,
                        "event_timestamp_ns": int(sample["timestamp_ns"]),
                        "recorded_yaw_rad": raw_yaw,
                        "recovered_yaw_rad": recovered_yaw,
                        "yaw_increment_rad": yaw_increment,
                        "yaw_recovery_method": recovery_method,
                        "pre_reset_speed_mps": float(previous_moving["speed"]),
                        "recovered_collision_speed_mps": collision_speed,
                        "speed_increment_mps": speed_increment,
                        "step_distance_m": step_distance,
                        "repaired_sample_count": 1,
                    }
                )
                continue

        if float(sample["speed"]) >= minimum_speed:
            moving_state = (
                float(sample["x"]),
                float(sample["y"]),
                float(sample["yaw"]),
                float(sample["speed"]),
            )
            if moving_state != previous_moving_state:
                moving_before_previous = previous_moving
                previous_moving = sample
                previous_moving_state = moving_state
    return repaired, diagnostic


class SimulatorRasterCollisionModel:
    """Base-link swept footprint against the exact simulator occupancy raster."""

    def __init__(self, map_yaml: str | Path, model: dict[str, Any]):
        self.yaml_path = Path(map_yaml).resolve()
        with self.yaml_path.open("r", encoding="utf-8") as stream:
            metadata = yaml.safe_load(stream)
        image_path = Path(metadata["image"])
        if not image_path.is_absolute():
            image_path = self.yaml_path.parent / image_path
        image = np.asarray(Image.open(image_path).convert("L"), dtype=np.uint8)
        # CppSimulator.set_map performs PIL.FLIP_TOP_BOTTOM before the C++ core.
        self.occupied = np.flipud(image) <= int(model["occupied_gray_threshold"])
        self.distance_field = distance_transform_edt(~self.occupied) * float(
            metadata["resolution"]
        )
        self.height, self.width = self.occupied.shape
        self.resolution = float(metadata["resolution"])
        self.origin_x = float(metadata["origin"][0])
        self.origin_y = float(metadata["origin"][1])
        self.origin_yaw = float(metadata["origin"][2])
        self.origin_cos = math.cos(self.origin_yaw)
        self.origin_sin = math.sin(self.origin_yaw)
        self.vehicle_length = float(model["vehicle_length_m"])
        self.vehicle_width = float(model["vehicle_width_m"])
        self.lidar_offset = float(model["lidar_offset_x_m"])
        self.scan_beams = int(model["scan_beams"])
        self.scan_fov = float(model["scan_fov_rad"])
        self.ttc_threshold = float(model["ttc_threshold_sec"])
        self.scan_noise_std = float(model["scan_noise_std_m"])
        self.noise_guard_sigma = float(model["scan_noise_guard_sigma"])
        self.theta_discretization = 2000
        self.ray_epsilon = 0.0001
        self.maximum_range = 30.0
        self._precompute_scan_geometry()

    def _precompute_scan_geometry(self) -> None:
        relative_angles = np.linspace(
            -0.5 * self.scan_fov, 0.5 * self.scan_fov, self.scan_beams
        )
        self.beam_cosines = np.cos(relative_angles)
        half_length = 0.5 * self.vehicle_length
        half_width = 0.5 * self.vehicle_width
        x_min = -half_length - self.lidar_offset
        x_max = half_length - self.lidar_offset
        if x_min > 0.0 or x_max < 0.0:
            raise ValueError("simulator LiDAR origin lies outside the vehicle footprint")

        direction_x = np.cos(relative_angles)
        direction_y = np.sin(relative_angles)
        distance_x = np.full(self.scan_beams, math.inf, dtype=np.float64)
        distance_y = np.full(self.scan_beams, math.inf, dtype=np.float64)
        positive_x = direction_x > 1.0e-12
        negative_x = direction_x < -1.0e-12
        positive_y = direction_y > 1.0e-12
        negative_y = direction_y < -1.0e-12
        distance_x[positive_x] = x_max / direction_x[positive_x]
        distance_x[negative_x] = x_min / direction_x[negative_x]
        distance_y[positive_y] = half_width / direction_y[positive_y]
        distance_y[negative_y] = -half_width / direction_y[negative_y]
        self.side_distances = np.minimum(distance_x, distance_y)

        table_angles = np.arange(self.theta_discretization, dtype=np.float64)
        table_angles *= 2.0 * math.pi / (self.theta_discretization - 1)
        self.theta_sines = np.sin(table_angles)
        self.theta_cosines = np.cos(table_angles)
        self.theta_index_increment = (
            self.theta_discretization
            * (self.scan_fov / (self.scan_beams - 1))
            / (2.0 * math.pi)
        )

    def _world_to_map(self, point: Point) -> Point:
        translated_x = point[0] - self.origin_x
        translated_y = point[1] - self.origin_y
        return (
            translated_x * self.origin_cos + translated_y * self.origin_sin,
            -translated_x * self.origin_sin + translated_y * self.origin_cos,
        )

    def _map_polygon(self, polygon: list[Point]) -> list[Point]:
        return [self._world_to_map(point) for point in polygon]

    def _distance_lookup(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        translated_x = x - self.origin_x
        translated_y = y - self.origin_y
        local_x = translated_x * self.origin_cos + translated_y * self.origin_sin
        local_y = -translated_x * self.origin_sin + translated_y * self.origin_cos
        columns = np.full(x.shape, self.width - 1, dtype=np.int64)
        rows = np.full(x.shape, self.height - 1, dtype=np.int64)
        valid = (
            (local_x >= 0.0)
            & (local_x < self.width * self.resolution)
            & (local_y >= 0.0)
            & (local_y < self.height * self.resolution)
        )
        columns[valid] = (local_x[valid] / self.resolution).astype(np.int64)
        rows[valid] = (local_y[valid] / self.resolution).astype(np.int64)
        return self.distance_field[rows, columns]

    def _noise_free_scan(self, x: float, y: float, yaw: float) -> np.ndarray:
        lidar_x = x + self.lidar_offset * math.cos(yaw)
        lidar_y = y + self.lidar_offset * math.sin(yaw)
        initial_index = (
            self.theta_discretization * (yaw - 0.5 * self.scan_fov) / (2.0 * math.pi)
        )
        theta_indices = np.fmod(
            initial_index
            + np.arange(self.scan_beams, dtype=np.float64) * self.theta_index_increment,
            self.theta_discretization,
        )
        theta_indices[theta_indices < 0.0] += self.theta_discretization
        table_indices = theta_indices.astype(np.int64)
        direction_x = self.theta_cosines[table_indices]
        direction_y = self.theta_sines[table_indices]
        ray_x = np.full(self.scan_beams, lidar_x, dtype=np.float64)
        ray_y = np.full(self.scan_beams, lidar_y, dtype=np.float64)
        distance = self._distance_lookup(ray_x, ray_y)
        total = distance.copy()
        for _ in range(4096):
            active = (distance > self.ray_epsilon) & (total <= self.maximum_range)
            if not np.any(active):
                break
            ray_x[active] += distance[active] * direction_x[active]
            ray_y[active] += distance[active] * direction_y[active]
            next_distance = self._distance_lookup(ray_x, ray_y)
            total[active] += next_distance[active]
            distance = next_distance
        else:
            raise RuntimeError("simulator-convention ray march failed to converge")
        return np.minimum(total, self.maximum_range)

    def _lidar_ttc_collision(
        self, x: float, y: float, yaw: float, speed: float, noise_guard: float
    ) -> tuple[bool, dict[str, float | int | None]]:
        scan = self._noise_free_scan(x, y, yaw)
        projected_velocity = speed * self.beam_cosines
        gap = scan - self.side_distances
        eligible = projected_velocity != 0.0
        noise_bound_a = -gap
        noise_bound_b = projected_velocity * self.ttc_threshold - gap
        noise_lower = np.minimum(noise_bound_a, noise_bound_b)
        noise_upper = np.maximum(noise_bound_a, noise_bound_b)
        zero_noise_ttc = np.full(gap.shape, math.inf, dtype=np.float64)
        zero_noise_ttc[eligible] = gap[eligible] / projected_velocity[eligible]
        zero_noise_hit = (zero_noise_ttc >= 0.0) & (
            zero_noise_ttc < self.ttc_threshold
        )
        guard_reaches_interval = (
            (noise_lower <= noise_guard) & (noise_upper >= -noise_guard)
            if noise_guard > 0.0
            else np.zeros(gap.shape, dtype=bool)
        )
        hits = np.flatnonzero(eligible & (zero_noise_hit | guard_reaches_interval))
        if hits.size:
            index = int(hits[0])
            return True, {
                "beam_index": index,
                "noise_free_scan_m": float(scan[index]),
                "vehicle_side_distance_m": float(self.side_distances[index]),
                "noise_free_gap_m": float(gap[index]),
                "projected_speed_mps": float(projected_velocity[index]),
                "noise_guard_m": noise_guard,
            }
        eligible_gaps = gap[eligible]
        return False, {
            "beam_index": None,
            "noise_free_scan_m": None,
            "vehicle_side_distance_m": None,
            "noise_free_gap_m": float(np.min(eligible_gaps)) if eligible_gaps.size else math.inf,
            "projected_speed_mps": None,
            "noise_guard_m": noise_guard,
        }

    def _outside_map(self, polygon: list[Point]) -> bool:
        maximum_x = self.width * self.resolution
        maximum_y = self.height * self.resolution
        return any(
            x < 0.0 or y < 0.0 or x >= maximum_x or y >= maximum_y
            for x, y in polygon
        )

    def _occupied_cells_near(self, polygon: list[Point]) -> list[list[Point]]:
        min_x = min(point[0] for point in polygon)
        max_x = max(point[0] for point in polygon)
        min_y = min(point[1] for point in polygon)
        max_y = max(point[1] for point in polygon)
        first_column = max(0, int(math.floor(min_x / self.resolution)) - 1)
        last_column = min(self.width - 1, int(math.floor(max_x / self.resolution)) + 1)
        first_row = max(0, int(math.floor(min_y / self.resolution)) - 1)
        last_row = min(self.height - 1, int(math.floor(max_y / self.resolution)) + 1)
        if first_column > last_column or first_row > last_row:
            return []
        rows, columns = np.nonzero(
            self.occupied[first_row : last_row + 1, first_column : last_column + 1]
        )
        cells = []
        for row_offset, column_offset in zip(rows, columns):
            row = first_row + int(row_offset)
            column = first_column + int(column_offset)
            x0 = column * self.resolution
            y0 = row * self.resolution
            x1 = x0 + self.resolution
            y1 = y0 + self.resolution
            cells.append([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
        return cells

    def _polygon_raster_overlap(self, polygon: list[Point]) -> tuple[bool, str]:
        if self._outside_map(polygon):
            return True, "map_boundary"
        if any(
            convex_polygons_intersect(polygon, cell)
            for cell in self._occupied_cells_near(polygon)
        ):
            return True, "occupied_raster_cell"
        return False, "none"

    def _clearance_within(self, polygon: list[Point], radius: float) -> float:
        xs = [point[0] for point in polygon]
        ys = [point[1] for point in polygon]
        expanded = [
            (min(xs) - radius, min(ys) - radius),
            (max(xs) + radius, min(ys) - radius),
            (max(xs) + radius, max(ys) + radius),
            (min(xs) - radius, max(ys) + radius),
        ]
        cells = self._occupied_cells_near(expanded)
        return min((polygon_distance(polygon, cell) for cell in cells), default=math.inf)

    def evaluate_pose(
        self, x: float, y: float, yaw: float, speed: float
    ) -> dict[str, Any]:
        base_world = oriented_rectangle(
            x, y, yaw, self.vehicle_length, self.vehicle_width
        )
        base = self._map_polygon(base_world)
        base_overlap, base_cause = self._polygon_raster_overlap(base)
        sweep = float(speed) * self.ttc_threshold
        noise_guard = (
            self.scan_noise_std * self.noise_guard_sigma
            if abs(speed) > 1.0e-9
            else 0.0
        )
        envelope_center_x = x + 0.5 * sweep * math.cos(yaw)
        envelope_center_y = y + 0.5 * sweep * math.sin(yaw)
        envelope_world = oriented_rectangle(
            envelope_center_x,
            envelope_center_y,
            yaw,
            self.vehicle_length + abs(sweep) + 2.0 * noise_guard,
            self.vehicle_width + 2.0 * noise_guard,
        )
        envelope = self._map_polygon(envelope_world)
        envelope_overlap, envelope_cause = self._polygon_raster_overlap(envelope)
        search_radius = abs(sweep) + noise_guard + math.sqrt(2.0) * self.resolution
        clearance = 0.0 if base_overlap else self._clearance_within(base, search_radius)
        ttc_collision = False
        ttc_diagnostics: dict[str, float | int | None] | None = None
        if not envelope_overlap and abs(speed) > 1.0e-9 and math.isfinite(clearance):
            ttc_collision, ttc_diagnostics = self._lidar_ttc_collision(
                x, y, yaw, speed, noise_guard
            )
        collision = envelope_overlap or ttc_collision
        if base_overlap:
            cause = base_cause
        elif envelope_overlap:
            cause = "raster_ttc_swept_footprint"
        elif ttc_collision:
            cause = "lidar_ttc_noise_guard"
        else:
            cause = "none"
        return {
            "collision": collision,
            "cause": cause,
            "base_raster_overlap": base_overlap,
            "ttc_swept_raster_overlap": envelope_overlap,
            "ttc_swept_raster_cause": envelope_cause,
            "ttc_noise_envelope_overlap": ttc_collision,
            "lidar_ttc_diagnostics": ttc_diagnostics,
            "raster_clearance_m": clearance,
            "ttc_sweep_m": abs(sweep),
            "scan_noise_guard_m": noise_guard,
        }

    def evaluate_recorded_scans(
        self, scan_records: list[Any], samples: list[dict[str, float]]
    ) -> dict[str, Any] | None:
        """Reapply cpp_backend::check_ttc to recorded noisy LaserScan messages."""
        if not scan_records:
            return None
        header_samples = {
            int(sample["header_timestamp_ns"]): sample
            for sample in samples
            if "header_timestamp_ns" in sample
        }
        sample_index = 0
        current_speed = 0.0
        last_pose: dict[str, float] | None = None
        evaluated_count = 0
        for record in scan_records:
            while (
                sample_index < len(samples)
                and int(samples[sample_index]["timestamp_ns"]) <= record.timestamp_ns
            ):
                current = samples[sample_index]
                current_speed = float(current["speed"])
                last_pose = current
                sample_index += 1
            scan_header_timestamp = None
            if hasattr(record.message, "header"):
                stamp = record.message.header.stamp
                scan_header_timestamp = int(stamp.sec) * 1_000_000_000 + int(
                    stamp.nanosec
                )
            matching_sample = header_samples.get(scan_header_timestamp)
            if matching_sample is not None:
                current_speed = float(
                    matching_sample.get("collision_speed", matching_sample["speed"])
                )
                last_pose = matching_sample
            ranges = np.asarray(record.message.ranges, dtype=np.float64)
            if ranges.size != self.scan_beams:
                raise ValueError(
                    f"recorded scan has {ranges.size} beams, expected {self.scan_beams}"
                )
            projected_velocity = current_speed * self.beam_cosines
            valid_velocity = projected_velocity != 0.0
            ttc = np.full(ranges.shape, math.inf, dtype=np.float64)
            with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
                ttc[valid_velocity] = (
                    ranges[valid_velocity] - self.side_distances[valid_velocity]
                ) / projected_velocity[valid_velocity]
            hits = np.flatnonzero(
                valid_velocity & (ttc >= 0.0) & (ttc < self.ttc_threshold)
            )
            evaluated_count += 1
            if hits.size:
                index = int(hits[0])
                pose = None
                if last_pose is not None:
                    pose = {
                        "x": float(last_pose["x"]),
                        "y": float(last_pose["y"]),
                        "yaw": float(last_pose["yaw"]),
                        "speed": current_speed,
                    }
                return {
                    "collision": True,
                    "first_collision_timestamp_ns": int(record.timestamp_ns),
                    "first_collision_pose": pose,
                    "cause": "recorded_lidar_ttc",
                    "minimum_raster_clearance_m": None,
                    "evaluated_scan_count": evaluated_count,
                    "pose_diagnostics": {
                        "beam_index": index,
                        "recorded_scan_m": float(ranges[index]),
                        "vehicle_side_distance_m": float(self.side_distances[index]),
                        "projected_speed_mps": float(projected_velocity[index]),
                        "ttc_sec": float(ttc[index]),
                    },
                }
        return {
            "collision": False,
            "first_collision_timestamp_ns": None,
            "first_collision_pose": None,
            "cause": "none",
            "minimum_raster_clearance_m": None,
            "evaluated_scan_count": evaluated_count,
            "pose_diagnostics": None,
        }

    def evaluate_trajectory(self, samples: list[dict[str, float]]) -> dict[str, Any]:
        minimum_clearance = math.inf
        last_state: tuple[float, float, float, float] | None = None
        evaluated_count = 0
        for sample in samples:
            speed = float(sample.get("collision_speed", sample["speed"]))
            state = (
                float(sample["x"]),
                float(sample["y"]),
                float(sample["yaw"]),
                speed,
            )
            if state == last_state:
                continue
            last_state = state
            evaluated_count += 1
            pose_result = self.evaluate_pose(*state)
            minimum_clearance = min(
                minimum_clearance, float(pose_result["raster_clearance_m"])
            )
            if pose_result["collision"]:
                return {
                    "collision": True,
                    "first_collision_timestamp_ns": int(sample["timestamp_ns"]),
                    "first_collision_pose": {
                        "x": state[0],
                        "y": state[1],
                        "yaw": state[2],
                        "speed": state[3],
                    },
                    "cause": pose_result["cause"],
                    "minimum_raster_clearance_m": minimum_clearance
                    if math.isfinite(minimum_clearance)
                    else None,
                    "evaluated_unique_pose_count": evaluated_count,
                    "pose_diagnostics": pose_result,
                }
        return {
            "collision": False,
            "first_collision_timestamp_ns": None,
            "first_collision_pose": None,
            "cause": "none",
            "minimum_raster_clearance_m": minimum_clearance
            if math.isfinite(minimum_clearance)
            else None,
            "evaluated_unique_pose_count": evaluated_count,
            "pose_diagnostics": None,
        }
