"""Pure geometry used by the external evaluator."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml


Point = tuple[float, float]


def oriented_rectangle(
    x: float, y: float, yaw: float, length: float, width: float
) -> list[Point]:
    cosine, sine = math.cos(yaw), math.sin(yaw)
    result = []
    for local_x, local_y in (
        (-0.5 * length, -0.5 * width),
        (0.5 * length, -0.5 * width),
        (0.5 * length, 0.5 * width),
        (-0.5 * length, 0.5 * width),
    ):
        result.append(
            (x + cosine * local_x - sine * local_y, y + sine * local_x + cosine * local_y)
        )
    return result


def _axes(polygon: list[Point]) -> Iterable[Point]:
    for first, second in zip(polygon, polygon[1:] + polygon[:1]):
        edge_x, edge_y = second[0] - first[0], second[1] - first[1]
        norm = math.hypot(edge_x, edge_y)
        if norm > 1.0e-12:
            yield -edge_y / norm, edge_x / norm


def convex_polygons_intersect(first: list[Point], second: list[Point]) -> bool:
    for axis_x, axis_y in list(_axes(first)) + list(_axes(second)):
        projection_first = [x * axis_x + y * axis_y for x, y in first]
        projection_second = [x * axis_x + y * axis_y for x, y in second]
        if max(projection_first) < min(projection_second) or max(projection_second) < min(
            projection_first
        ):
            return False
    return True


def _point_segment_distance(point: Point, first: Point, second: Point) -> float:
    dx, dy = second[0] - first[0], second[1] - first[1]
    denominator = dx * dx + dy * dy
    if denominator <= 1.0e-18:
        return math.hypot(point[0] - first[0], point[1] - first[1])
    ratio = max(
        0.0,
        min(1.0, ((point[0] - first[0]) * dx + (point[1] - first[1]) * dy) / denominator),
    )
    return math.hypot(point[0] - (first[0] + ratio * dx), point[1] - (first[1] + ratio * dy))


def polygon_distance(first: list[Point], second: list[Point]) -> float:
    if convex_polygons_intersect(first, second):
        return 0.0
    minimum = math.inf
    for point in first:
        for edge_a, edge_b in zip(second, second[1:] + second[:1]):
            minimum = min(minimum, _point_segment_distance(point, edge_a, edge_b))
    for point in second:
        for edge_a, edge_b in zip(first, first[1:] + first[:1]):
            minimum = min(minimum, _point_segment_distance(point, edge_a, edge_b))
    return minimum


def obstacle_polygon(obstacle: dict) -> list[Point]:
    if obstacle["shape"] != "rect":
        raise ValueError(f"unsupported obstacle shape: {obstacle['shape']}")
    return oriented_rectangle(
        float(obstacle["x"]),
        float(obstacle["y"]),
        float(obstacle.get("yaw", 0.0)),
        float(obstacle["width"]),
        float(obstacle["height"]),
    )


def footprint_obstacle_clearance(
    x: float,
    y: float,
    yaw: float,
    vehicle_length: float,
    vehicle_width: float,
    obstacle: dict,
) -> tuple[float, bool]:
    vehicle = oriented_rectangle(x, y, yaw, vehicle_length, vehicle_width)
    target = obstacle_polygon(obstacle)
    overlap = convex_polygons_intersect(vehicle, target)
    return polygon_distance(vehicle, target), overlap


class OccupancyDistanceField:
    """Conservative clean-map wall clearance for an oriented vehicle footprint."""

    def __init__(self, map_yaml: str | Path):
        self.yaml_path = Path(map_yaml).resolve()
        with self.yaml_path.open("r", encoding="utf-8") as stream:
            metadata = yaml.safe_load(stream)
        image_path = Path(metadata["image"])
        if not image_path.is_absolute():
            image_path = self.yaml_path.parent / image_path
        image = np.asarray(Image.open(image_path).convert("L"), dtype=np.float64) / 255.0
        self.height, self.width = image.shape
        self.resolution = float(metadata["resolution"])
        self.origin_x = float(metadata["origin"][0])
        self.origin_y = float(metadata["origin"][1])
        origin_yaw = float(metadata["origin"][2])
        if abs(origin_yaw) > 1.0e-9:
            raise ValueError("rotated occupancy-map origins are not supported")
        negate = int(metadata.get("negate", 0))
        free_threshold = float(metadata.get("free_thresh", 0.25))
        occupancy = image if negate else 1.0 - image
        self.free = occupancy <= free_threshold
        self.distance = distance_transform_edt(self.free) * self.resolution

    def _sample_indices(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        u = np.floor((x - self.origin_x) / self.resolution).astype(np.int64)
        v = np.floor(self.height - (y - self.origin_y) / self.resolution).astype(np.int64)
        return u, v

    @staticmethod
    def _local_footprint_samples(length: float, width: float, step: float) -> np.ndarray:
        nx = max(2, int(math.ceil(length / step)) + 1)
        ny = max(2, int(math.ceil(width / step)) + 1)
        local_x = np.linspace(-0.5 * length, 0.5 * length, nx)
        local_y = np.linspace(-0.5 * width, 0.5 * width, ny)
        grid_x, grid_y = np.meshgrid(local_x, local_y, indexing="xy")
        return np.column_stack((grid_x.ravel(), grid_y.ravel()))

    def footprint_clearance(
        self,
        x: float,
        y: float,
        yaw: float,
        length: float,
        width: float,
        sample_step: float,
    ) -> tuple[float, bool]:
        local = self._local_footprint_samples(length, width, sample_step)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        world_x = x + cosine * local[:, 0] - sine * local[:, 1]
        world_y = y + sine * local[:, 0] + cosine * local[:, 1]
        u, v = self._sample_indices(world_x, world_y)
        valid = (u >= 0) & (u < self.width) & (v >= 0) & (v < self.height)
        if not np.all(valid):
            return 0.0, True
        occupied = ~self.free[v, u]
        overlap = bool(np.any(occupied))
        # A map pixel represents an area, not a point. Subtract half its diagonal
        # so a reported positive value remains conservative at map resolution.
        clearance = max(
            0.0,
            float(np.min(self.distance[v, u])) - 0.5 * math.sqrt(2.0) * self.resolution,
        )
        return clearance, overlap
