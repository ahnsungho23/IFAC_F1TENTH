"""Deterministic, stratified, single-static-obstacle scenario generator."""

from __future__ import annotations

import json
import math
from pathlib import Path
import random
from typing import Any

from .map_baker import MapModel
from .schemas import (
    ObstacleSpec,
    ScenarioManifest,
    SpawnPose,
    atomic_write_json,
    sha256_file,
)


def _waypoint_array(document: dict[str, Any]) -> list[dict[str, Any]]:
    for key in ("global_traj_wpnts_iqp", "global_traj_wpnts_sp", "centerline_waypoints"):
        waypoints = document.get(key, {}).get("wpnts", [])
        if waypoints:
            return waypoints
    raise ValueError("waypoint JSON has no supported waypoint array")


def load_waypoints(path: str | Path) -> list[dict[str, Any]]:
    with Path(path).open("r", encoding="utf-8") as stream:
        return _waypoint_array(json.load(stream))


def track_length(waypoints: list[dict[str, Any]]) -> float:
    last = waypoints[-1]
    first = waypoints[0]
    return float(last["s_m"]) + math.hypot(
        float(last["x_m"]) - float(first["x_m"]),
        float(last["y_m"]) - float(first["y_m"]),
    )


def classify_indices(waypoints: list[dict[str, Any]]) -> dict[str, list[int]]:
    count = len(waypoints)
    curvature = [abs(float(waypoint["kappa_radpm"])) for waypoint in waypoints]
    classes: dict[str, list[int]] = {
        "straight": [],
        "corner_entry": [],
        "corner_mid": [],
        "corner_exit": [],
        "narrow_or_difficult": [],
    }
    for index, waypoint in enumerate(waypoints):
        previous = curvature[(index - 2) % count]
        current = curvature[index]
        following = curvature[(index + 2) % count]
        width = float(waypoint["d_left"]) + float(waypoint["d_right"])
        if width < 1.20:
            classes["narrow_or_difficult"].append(index)
        if current < 0.08 and previous < 0.12 and following < 0.12:
            classes["straight"].append(index)
        if current >= 0.15 and following > current + 0.05:
            classes["corner_entry"].append(index)
        if current >= 0.30 and current >= previous - 0.05 and current >= following - 0.05:
            classes["corner_mid"].append(index)
        if current >= 0.15 and previous > current + 0.05:
            classes["corner_exit"].append(index)
    return classes


def _spawn_waypoint(
    waypoints: list[dict[str, Any]], obstacle_s: float, distance_before: float
) -> dict[str, Any]:
    length = track_length(waypoints)
    target_s = (obstacle_s - distance_before) % length
    return min(
        waypoints,
        key=lambda waypoint: min(
            abs(float(waypoint["s_m"]) - target_s),
            length - abs(float(waypoint["s_m"]) - target_s),
        ),
    )


def _lateral_offset(waypoint: dict[str, Any], band: str, rng: random.Random) -> float:
    if band == "left":
        return 0.20 + rng.uniform(-0.025, 0.025)
    if band == "right":
        return -0.20 + rng.uniform(-0.025, 0.025)
    if band == "center":
        return rng.uniform(-0.025, 0.025)
    raise ValueError(f"unsupported lateral band: {band}")


def _passage_exists(
    waypoint: dict[str, Any], obstacle: ObstacleSpec, maximum_clearance: float
) -> bool:
    tangent_yaw = float(waypoint["psi_rad"])
    normal_x, normal_y = -math.sin(tangent_yaw), math.cos(tangent_yaw)
    obstacle_x_axis = (math.cos(obstacle.yaw), math.sin(obstacle.yaw))
    obstacle_y_axis = (-math.sin(obstacle.yaw), math.cos(obstacle.yaw))
    half_d = (
        0.5 * obstacle.width * abs(normal_x * obstacle_x_axis[0] + normal_y * obstacle_x_axis[1])
        + 0.5 * obstacle.height * abs(normal_x * obstacle_y_axis[0] + normal_y * obstacle_y_axis[1])
    )
    left_gap = float(waypoint["d_left"]) - (obstacle.d + half_d)
    right_gap = (obstacle.d - half_d) + float(waypoint["d_right"])
    return max(left_gap, right_gap) >= maximum_clearance


def generate_fixed_scenarios(config: dict[str, Any], workspace_root: str | Path) -> list[Path]:
    root = Path(workspace_root).resolve()
    paths = config["paths"]
    experiment = config["experiment"]
    scenario_config = config["scenario"]
    clean_map_yaml = (root / paths["clean_map_yaml"]).resolve()
    waypoint_json = (root / paths["waypoint_json"]).resolve()
    simulator_config = Path(paths["simulator_config"]).resolve()
    output_root = Path(paths["output_root"]).resolve()
    scenario_root = output_root / "scenarios" / "train"
    map_model = MapModel(clean_map_yaml)
    waypoints = load_waypoints(waypoint_json)
    classes = classify_indices(waypoints)

    scenario_ids = experiment["fixed_training_scenario_ids"]
    seeds = experiment["fixed_training_seeds"]
    categories = scenario_config["categories"]
    bands = scenario_config["lateral_bands"]
    if not (len(scenario_ids) == len(seeds) == len(categories) == len(bands)):
        raise ValueError("fixed scenario IDs, seeds, categories, and bands must have equal length")

    manifests: list[Path] = []
    for scenario_id, seed, category, band in zip(scenario_ids, seeds, categories, bands):
        rng = random.Random(int(seed))
        indices = list(classes.get(category, []))
        rng.shuffle(indices)
        selected: tuple[dict[str, Any], ObstacleSpec] | None = None
        for index in indices:
            waypoint = waypoints[index]
            d = _lateral_offset(waypoint, band, rng)
            yaw = float(scenario_config["obstacle_yaw_rad"])
            x = float(waypoint["x_m"]) - d * math.sin(float(waypoint["psi_rad"]))
            y = float(waypoint["y_m"]) + d * math.cos(float(waypoint["psi_rad"]))
            obstacle = ObstacleSpec(
                shape=str(scenario_config["obstacle_shape"]),
                x=x,
                y=y,
                s=float(waypoint["s_m"]),
                d=d,
                yaw=yaw,
                width=float(scenario_config["obstacle_width_m"]),
                height=float(scenario_config["obstacle_height_m"]),
            )
            if not _passage_exists(
                waypoint, obstacle, float(scenario_config["maximum_candidate_clearance_m"])
            ):
                continue
            if not map_model.obstacle_region_is_free(
                obstacle, float(scenario_config["map_validation_sample_step_m"])
            ):
                continue
            selected = waypoint, obstacle
            break
        if selected is None:
            raise RuntimeError(f"could not generate a valid {category}/{band} scenario")

        waypoint, obstacle = selected
        spawn_waypoint = _spawn_waypoint(
            waypoints,
            obstacle.s,
            float(scenario_config["spawn_distance_before_obstacle_m"]),
        )
        spawn = SpawnPose(
            x=float(spawn_waypoint["x_m"]),
            y=float(spawn_waypoint["y_m"]),
            yaw=float(spawn_waypoint["psi_rad"]),
            s=float(spawn_waypoint["s_m"]),
        )
        forward_distance = (obstacle.s - spawn.s) % track_length(waypoints)
        if forward_distance < float(scenario_config["minimum_spawn_obstacle_distance_m"]):
            raise RuntimeError(f"{scenario_id} obstacle is too close to spawn")

        directory = scenario_root / scenario_id
        baked = map_model.write_baked(directory, f"{scenario_id}_map", [obstacle])
        manifest = ScenarioManifest(
            schema="cmaes_scenario/1",
            scenario_id=str(scenario_id),
            seed=int(seed),
            category=str(category),
            lateral_band=str(band),
            map_name=str(experiment["map_name"]),
            generator_version=str(experiment["generator_version"]),
            obstacle=obstacle,
            spawn_pose=spawn,
            clean_map_yaml=str(map_model.yaml_path),
            clean_map_image=str(map_model.image_path),
            baked_map_yaml=baked["yaml"],
            baked_map_image=baked["image"],
            clean_map_yaml_sha256=sha256_file(map_model.yaml_path),
            clean_map_image_sha256=sha256_file(map_model.image_path),
            clean_map_hash=map_model.clean_hash(),
            baked_map_yaml_sha256=baked["yaml_sha256"],
            baked_map_image_sha256=baked["image_sha256"],
            baked_map_hash=baked["combined_sha256"],
            waypoint_file=str(waypoint_json),
            waypoint_sha256=sha256_file(waypoint_json),
            simulator_config=str(simulator_config),
            simulator_config_sha256=sha256_file(simulator_config),
            vehicle_length_m=float(config["evaluation"]["vehicle_length_m"]),
            vehicle_width_m=float(config["evaluation"]["vehicle_width_m"]),
            map_resolution_m=map_model.resolution,
        )
        manifest_path = directory / "manifest.json"
        atomic_write_json(manifest_path, manifest.to_dict())
        manifests.append(manifest_path)
    return manifests
