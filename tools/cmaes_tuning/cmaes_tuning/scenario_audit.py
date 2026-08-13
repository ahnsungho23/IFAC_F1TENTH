"""Offline validity and train/validation separation audit for CMA scenarios."""

from __future__ import annotations

from collections import Counter, defaultdict
import json
import math
from pathlib import Path
from typing import Any

from .map_baker import MapModel
from .scenario_generator import load_waypoints, passage_gaps, track_length
from .schemas import ObstacleSpec, atomic_write_json, sha256_file


def _load(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _cyclic_distance(first: float, second: float, length: float) -> float:
    delta = abs(first - second)
    return min(delta, length - delta)


def _identity(manifest: dict[str, Any]) -> tuple[float | str, ...]:
    obstacle = manifest["obstacle"]
    spawn = manifest["spawn_pose"]
    return (
        str(obstacle["shape"]),
        round(float(obstacle["x"]), 10),
        round(float(obstacle["y"]), 10),
        round(float(obstacle["yaw"]), 10),
        round(float(obstacle["width"]), 10),
        round(float(obstacle["height"]), 10),
        round(float(spawn["x"]), 10),
        round(float(spawn["y"]), 10),
        round(float(spawn["yaw"]), 10),
    )


def _distribution(documents: list[dict[str, Any]]) -> dict[str, Any]:
    category = Counter(str(item["category"]) for item in documents)
    lateral = Counter(str(item["lateral_band"]) for item in documents)
    matrix: dict[str, Counter[str]] = defaultdict(Counter)
    for item in documents:
        matrix[str(item["category"])][str(item["lateral_band"])] += 1
    return {
        "count": len(documents),
        "category": dict(sorted(category.items())),
        "lateral": dict(sorted(lateral.items())),
        "category_by_lateral": {
            name: dict(sorted(counts.items())) for name, counts in sorted(matrix.items())
        },
    }


def audit_scenario_dataset(
    datasets: dict[str, list[str | Path]],
    config: dict[str, Any],
    output_path: str | Path | None = None,
) -> dict[str, Any]:
    scenario_config = config["scenario"]
    dataset_config = config["dataset"]
    all_paths = [Path(path).resolve() for paths in datasets.values() for path in paths]
    documents = {path: _load(path) for path in all_paths}
    map_models: dict[str, MapModel] = {}
    waypoint_sets: dict[str, list[dict[str, Any]]] = {}
    scenario_rows = []

    for path, manifest in documents.items():
        failures: list[str] = []
        expected_split = next(
            split for split, paths in datasets.items() if path in {Path(item).resolve() for item in paths}
        )
        if manifest.get("dataset_split") != expected_split:
            failures.append("dataset_split_mismatch")
        if not manifest.get("category") or not manifest.get("lateral_band"):
            failures.append("missing_stratum")

        clean_map = str(Path(manifest["clean_map_yaml"]).resolve())
        map_model = map_models.setdefault(clean_map, MapModel(clean_map))
        waypoint_file = str(Path(manifest["waypoint_file"]).resolve())
        waypoints = waypoint_sets.setdefault(waypoint_file, load_waypoints(waypoint_file))
        length = track_length(waypoints)
        obstacle = ObstacleSpec(**manifest["obstacle"])
        waypoint = min(
            waypoints,
            key=lambda item: _cyclic_distance(
                float(item["s_m"]), float(obstacle.s), length
            ),
        )

        obstacle_region_free = map_model.obstacle_region_is_free(
            obstacle, float(scenario_config["map_validation_sample_step_m"])
        )
        if not obstacle_region_free:
            failures.append("obstacle_overlaps_clean_map_wall")
        spawn = manifest["spawn_pose"]
        spawn_free = map_model.is_free(float(spawn["x"]), float(spawn["y"]))
        if not spawn_free:
            failures.append("spawn_not_drivable")
        forward_distance = (float(obstacle.s) - float(spawn["s"])) % length
        if forward_distance < float(
            scenario_config["minimum_spawn_obstacle_distance_m"]
        ):
            failures.append("spawn_obstacle_distance_too_short")

        left_gap, right_gap = passage_gaps(waypoint, obstacle)
        required_gap = float(scenario_config["maximum_candidate_clearance_m"])
        passage_exists = max(left_gap, right_gap) >= required_gap
        if not passage_exists:
            failures.append("no_theoretical_vehicle_passage")

        raster = manifest.get("baked_obstacle_raster", {})
        raster_bounds = raster.get("world_half_open_bounds_m")
        corners = map_model.obstacle_corners(obstacle)
        continuous_bounds = {
            "x_min": min(point[0] for point in corners),
            "x_max": max(point[0] for point in corners),
            "y_min": min(point[1] for point in corners),
            "y_max": max(point[1] for point in corners),
        }
        raster_matches_manifest = bool(raster.get("changed_cell_count", 0)) and bool(
            raster_bounds
        )
        if raster_matches_manifest:
            resolution = float(manifest["map_resolution_m"])
            for axis in ("x", "y"):
                raster_min = float(raster_bounds[f"{axis}_min"])
                raster_max = float(raster_bounds[f"{axis}_max"])
                continuous_min = continuous_bounds[f"{axis}_min"]
                continuous_max = continuous_bounds[f"{axis}_max"]
                raster_matches_manifest = raster_matches_manifest and (
                    raster_min <= continuous_min + 1.0e-12
                    and raster_max >= continuous_max - 1.0e-12
                    and continuous_min - raster_min <= resolution + 1.0e-12
                    and raster_max - continuous_max <= resolution + 1.0e-12
                )
        if not raster_matches_manifest:
            failures.append("baked_raster_manifest_geometry_mismatch")

        hash_checks = {
            "clean_map_yaml": "clean_map_yaml_sha256",
            "clean_map_image": "clean_map_image_sha256",
            "baked_map_yaml": "baked_map_yaml_sha256",
            "baked_map_image": "baked_map_image_sha256",
            "waypoint_file": "waypoint_sha256",
            "simulator_config": "simulator_config_sha256",
            "simulator_collision_source": "simulator_collision_source_sha256",
        }
        hash_ok = True
        for path_key, hash_key in hash_checks.items():
            artifact = Path(manifest[path_key])
            if not artifact.is_file() or sha256_file(artifact) != manifest[hash_key]:
                hash_ok = False
                failures.append(f"hash_mismatch:{path_key}")

        scenario_rows.append(
            {
                "scenario_id": manifest["scenario_id"],
                "split": expected_split,
                "category": manifest["category"],
                "lateral_band": manifest["lateral_band"],
                "valid": not failures,
                "failures": failures,
                "obstacle_region_free": obstacle_region_free,
                "spawn_free": spawn_free,
                "spawn_obstacle_forward_distance_m": forward_distance,
                "passage_left_gap_m": left_gap,
                "passage_right_gap_m": right_gap,
                "required_passage_gap_m": required_gap,
                "passage_exists": passage_exists,
                "raster_matches_manifest": raster_matches_manifest,
                "hashes_valid": hash_ok,
                "manifest_path": str(path),
                "manifest_sha256": sha256_file(path),
            }
        )

    split_documents = {
        split: [_load(path) for path in paths] for split, paths in datasets.items()
    }
    training_identities = {_identity(item) for item in split_documents["training"]}
    validation_identities = {_identity(item) for item in split_documents["validation"]}
    duplicate_identities = training_identities & validation_identities
    baked_hash_duplicates = (
        {item["baked_map_hash"] for item in split_documents["training"]}
        & {item["baked_map_hash"] for item in split_documents["validation"]}
    )
    expected_counts = {
        "training": int(dataset_config["training_count"]),
        "validation": int(dataset_config["validation_count"]),
    }
    count_match = all(
        len(split_documents[split]) == expected for split, expected in expected_counts.items()
    )
    all_valid = (
        count_match
        and not duplicate_identities
        and not baked_hash_duplicates
        and all(row["valid"] for row in scenario_rows)
    )
    report = {
        "schema": "cmaes_scenario_dataset_audit/1",
        "all_valid": all_valid,
        "expected_counts": expected_counts,
        "count_match": count_match,
        "distribution": {
            split: _distribution(items) for split, items in split_documents.items()
        },
        "training_validation_duplicate_count": len(duplicate_identities),
        "training_validation_baked_hash_duplicate_count": len(baked_hash_duplicates),
        "scenario_count": len(scenario_rows),
        "valid_scenario_count": sum(row["valid"] for row in scenario_rows),
        "invalid_scenarios": [row for row in scenario_rows if not row["valid"]],
        "scenarios": scenario_rows,
    }
    if output_path is not None:
        atomic_write_json(output_path, report)
    return report
