"""Shared schemas and reproducibility helpers."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import os
from pathlib import Path
import tempfile
from typing import Any


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def atomic_write_json(path: str | Path, payload: Any) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary, destination)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


@dataclass(frozen=True)
class ObstacleSpec:
    shape: str
    x: float
    y: float
    s: float
    d: float
    yaw: float
    width: float
    height: float


@dataclass(frozen=True)
class SpawnPose:
    x: float
    y: float
    yaw: float
    s: float


@dataclass(frozen=True)
class ScenarioManifest:
    schema: str
    scenario_id: str
    dataset_split: str
    seed: int
    category: str
    lateral_band: str
    map_name: str
    generator_version: str
    obstacle: ObstacleSpec
    spawn_pose: SpawnPose
    clean_map_yaml: str
    clean_map_image: str
    baked_map_yaml: str
    baked_map_image: str
    clean_map_yaml_sha256: str
    clean_map_image_sha256: str
    clean_map_hash: str
    baked_map_yaml_sha256: str
    baked_map_image_sha256: str
    baked_map_hash: str
    waypoint_file: str
    waypoint_sha256: str
    simulator_config: str
    simulator_config_sha256: str
    simulator_collision_source: str
    simulator_collision_source_sha256: str
    simulator_collision_model: dict[str, Any]
    baked_obstacle_raster: dict[str, Any]
    vehicle_length_m: float
    vehicle_width_m: float
    map_resolution_m: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)
