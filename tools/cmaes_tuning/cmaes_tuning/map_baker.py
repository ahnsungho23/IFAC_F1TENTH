"""Headless deterministic obstacle-map baking independent of planner/perception."""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
from typing import Iterable

import numpy as np
from PIL import Image, ImageDraw
import yaml

from .schemas import ObstacleSpec, sha256_file


class MapModel:
    def __init__(self, yaml_path: str | Path):
        self.yaml_path = Path(yaml_path).resolve()
        with self.yaml_path.open("r", encoding="utf-8") as stream:
            self.metadata = yaml.safe_load(stream)
        required = {"image", "resolution", "origin"}
        if not required.issubset(self.metadata):
            raise ValueError(f"invalid occupancy map YAML: {self.yaml_path}")
        image_path = Path(self.metadata["image"])
        if not image_path.is_absolute():
            image_path = self.yaml_path.parent / image_path
        self.image_path = image_path.resolve()
        self.image = Image.open(self.image_path).convert("L")
        self.width, self.height = self.image.size
        self.resolution = float(self.metadata["resolution"])
        self.origin_x = float(self.metadata["origin"][0])
        self.origin_y = float(self.metadata["origin"][1])
        self.origin_yaw = float(self.metadata["origin"][2])
        if abs(self.origin_yaw) > 1.0e-9:
            raise ValueError("rotated occupancy-map origins are not supported")
        self.negate = int(self.metadata.get("negate", 0))
        self.occupied_threshold = float(self.metadata.get("occupied_thresh", 0.65))
        self.free_threshold = float(self.metadata.get("free_thresh", 0.25))

    def world_to_pixel(self, x: float, y: float) -> tuple[float, float]:
        u = (x - self.origin_x) / self.resolution
        v = self.height - (y - self.origin_y) / self.resolution
        return u, v

    def pixel_to_world(self, u: float, v: float) -> tuple[float, float]:
        return (
            self.origin_x + u * self.resolution,
            self.origin_y + (self.height - v) * self.resolution,
        )

    def is_free(self, x: float, y: float) -> bool:
        u, v = self.world_to_pixel(x, y)
        ui, vi = int(math.floor(u)), int(math.floor(v))
        if not (0 <= ui < self.width and 0 <= vi < self.height):
            return False
        gray = float(self.image.getpixel((ui, vi))) / 255.0
        occupancy = gray if self.negate else 1.0 - gray
        return occupancy <= self.free_threshold

    @staticmethod
    def obstacle_corners(obstacle: ObstacleSpec) -> list[tuple[float, float]]:
        cosine = math.cos(obstacle.yaw)
        sine = math.sin(obstacle.yaw)
        corners = []
        for local_x, local_y in (
            (-0.5 * obstacle.width, -0.5 * obstacle.height),
            (0.5 * obstacle.width, -0.5 * obstacle.height),
            (0.5 * obstacle.width, 0.5 * obstacle.height),
            (-0.5 * obstacle.width, 0.5 * obstacle.height),
        ):
            corners.append(
                (
                    obstacle.x + cosine * local_x - sine * local_y,
                    obstacle.y + sine * local_x + cosine * local_y,
                )
            )
        return corners

    def obstacle_region_is_free(self, obstacle: ObstacleSpec, step_m: float) -> bool:
        if obstacle.shape != "rect":
            raise ValueError(f"unsupported obstacle shape: {obstacle.shape}")
        nx = max(2, int(math.ceil(obstacle.width / step_m)) + 1)
        ny = max(2, int(math.ceil(obstacle.height / step_m)) + 1)
        cosine = math.cos(obstacle.yaw)
        sine = math.sin(obstacle.yaw)
        for ix in range(nx):
            local_x = -0.5 * obstacle.width + obstacle.width * ix / (nx - 1)
            for iy in range(ny):
                local_y = -0.5 * obstacle.height + obstacle.height * iy / (ny - 1)
                x = obstacle.x + cosine * local_x - sine * local_y
                y = obstacle.y + sine * local_x + cosine * local_y
                if not self.is_free(x, y):
                    return False
        return True

    def bake(self, obstacles: Iterable[ObstacleSpec]) -> Image.Image:
        baked = self.image.copy()
        draw = ImageDraw.Draw(baked)
        fill = 255 if self.negate else 0
        for obstacle in obstacles:
            if obstacle.shape != "rect":
                raise ValueError(f"unsupported obstacle shape: {obstacle.shape}")
            polygon = [self.world_to_pixel(x, y) for x, y in self.obstacle_corners(obstacle)]
            draw.polygon(polygon, fill=fill)
        return baked

    def write_baked(
        self,
        output_directory: str | Path,
        stem: str,
        obstacles: Iterable[ObstacleSpec],
    ) -> dict[str, object]:
        destination = Path(output_directory)
        destination.mkdir(parents=True, exist_ok=True)
        image_path = destination / f"{stem}.png"
        yaml_path = destination / f"{stem}.yaml"
        obstacle_list = list(obstacles)
        baked_image = self.bake(obstacle_list)
        baked_image.save(image_path)
        metadata = dict(self.metadata)
        metadata["image"] = image_path.name
        with yaml_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(metadata, stream, sort_keys=False)
        combined = hashlib.sha256()
        combined.update(bytes.fromhex(sha256_file(yaml_path)))
        combined.update(bytes.fromhex(sha256_file(image_path)))
        changed = np.asarray(baked_image) != np.asarray(self.image)
        simulator_rows, simulator_columns = np.nonzero(np.flipud(changed))
        if simulator_rows.size:
            raster_geometry: dict[str, object] = {
                "changed_cell_count": int(simulator_rows.size),
                "column_index_min": int(np.min(simulator_columns)),
                "column_index_max": int(np.max(simulator_columns)),
                "row_index_min_after_vertical_flip": int(np.min(simulator_rows)),
                "row_index_max_after_vertical_flip": int(np.max(simulator_rows)),
                "world_half_open_bounds_m": {
                    "x_min": self.origin_x
                    + int(np.min(simulator_columns)) * self.resolution,
                    "x_max": self.origin_x
                    + (int(np.max(simulator_columns)) + 1) * self.resolution,
                    "y_min": self.origin_y
                    + int(np.min(simulator_rows)) * self.resolution,
                    "y_max": self.origin_y
                    + (int(np.max(simulator_rows)) + 1) * self.resolution,
                },
                "pixel_convention": (
                    "PIL polygon raster; simulator vertical flip; floor lookup into "
                    "half-open resolution-sized cells"
                ),
            }
        else:
            raster_geometry = {
                "changed_cell_count": 0,
                "world_half_open_bounds_m": None,
                "pixel_convention": (
                    "PIL polygon raster; simulator vertical flip; floor lookup into "
                    "half-open resolution-sized cells"
                ),
            }
        return {
            "yaml": str(yaml_path.resolve()),
            "image": str(image_path.resolve()),
            "yaml_sha256": sha256_file(yaml_path),
            "image_sha256": sha256_file(image_path),
            "combined_sha256": combined.hexdigest(),
            "raster_geometry": raster_geometry,
        }

    def clean_hash(self) -> str:
        combined = hashlib.sha256()
        combined.update(bytes.fromhex(sha256_file(self.yaml_path)))
        combined.update(bytes.fromhex(sha256_file(self.image_path)))
        return combined.hexdigest()
