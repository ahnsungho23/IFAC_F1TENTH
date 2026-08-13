"""Experiment provenance, tabular result, and checkpoint helpers."""

from __future__ import annotations

import csv
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import pickle
import shutil
import subprocess
import tempfile
from typing import Any

from .schemas import atomic_write_json, sha256_file


def _git(workspace_root: Path, *arguments: str) -> str:
    result = subprocess.run(
        ["git", *arguments], cwd=workspace_root, capture_output=True, text=True, check=False
    )
    return result.stdout.strip()


def dependency_versions() -> dict[str, str]:
    versions = {}
    for distribution in ("cma", "numpy", "Pillow", "PyYAML", "scipy"):
        try:
            versions[distribution] = importlib.metadata.version(distribution)
        except importlib.metadata.PackageNotFoundError:
            versions[distribution] = "missing"
    return versions


def _module_binary_provenance(
    module_name: str, artifact_directory: Path
) -> dict[str, str]:
    module = importlib.import_module(module_name)
    path = Path(module.__file__).resolve()
    artifact_directory.mkdir(parents=True, exist_ok=True)
    snapshot = artifact_directory / path.name
    shutil.copy2(path, snapshot)
    return {
        "module": module_name,
        "loaded_path": str(path),
        "loaded_sha256": sha256_file(path),
        "snapshot_path": str(snapshot.resolve()),
        "snapshot_sha256": sha256_file(snapshot),
    }


def create_experiment_manifest(
    path: str | Path,
    workspace_root: str | Path,
    config_path: str | Path,
    config: dict[str, Any],
    scenario_manifests: list[str | Path],
) -> dict[str, Any]:
    root = Path(workspace_root).resolve()
    dependencies = dependency_versions()
    destination = Path(path)
    if destination.is_file():
        try:
            previous = json.loads(destination.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            previous = {}
        for distribution, version in previous.get("dependencies", {}).items():
            if dependencies.get(distribution) == "missing" and version != "missing":
                # Test A-C do not import pycma. Preserve a version already
                # observed during Test D when the same experiment is reopened
                # from a Python environment without that optional dependency.
                dependencies[distribution] = version
    scenario_documents = [
        json.loads(Path(item).read_text(encoding="utf-8"))
        for item in scenario_manifests
    ]
    map_resolutions = sorted(
        {float(document["map_resolution_m"]) for document in scenario_documents}
    )
    simulator_configs = {
        document["simulator_config"]: document["simulator_config_sha256"]
        for document in scenario_documents
    }
    manifest = {
        "schema": "cmaes_experiment/1",
        "git_commit": _git(root, "rev-parse", "HEAD"),
        "git_status": _git(root, "status", "--short"),
        "config_path": str(Path(config_path).resolve()),
        "config_sha256": sha256_file(config_path),
        "localization_mode": config["experiment"].get("localization_mode", "mcl"),
        "localization_contract": {
            "ground_truth_allowed_for": "ego_pose_only",
            "ground_truth_obstacle_leakage_allowed": False,
            "planner_obstacle_input": "/scan -> obstacle_detector -> /static_obs",
            "mcl_output_interface": "/pf/pose/odom",
            "simulator_tf_owner": "f1tenth_gym_ros/gym_bridge",
        },
        "controller_configuration": config["controller"],
        "simulator_runtime_configuration": config["simulator_runtime"],
        "timing_audit_configuration": config.get("timing_audit", {}),
        "timing_runtime_provenance": {
            name: {
                "path": str(resolve.resolve()),
                "sha256": sha256_file(resolve),
            }
            for name, resolve in (
                (
                    "state_machine",
                    root / config["paths"]["state_machine_source"],
                ),
                (
                    "control_map",
                    root / config["paths"]["control_map_source"],
                ),
                (
                    "drive_selector",
                    root / config["paths"]["drive_selector_source"],
                ),
            )
        },
        "simulator_runtime_provenance": {
            "bridge_source": {
                "path": str(Path(config["paths"]["simulator_bridge_source"]).resolve()),
                "sha256": sha256_file(config["paths"]["simulator_bridge_source"]),
            },
            "loaded_cpp_backend": _module_binary_provenance(
                "f110_gym._cpp_backend", destination.parent / "artifacts"
            ),
        },
        "cma_configuration": config["cma"],
        "objective_configuration": config["objective"],
        "evaluation_configuration": config["evaluation"],
        "environment_ground_truth": {
            "vehicle_length_m": float(config["evaluation"]["vehicle_length_m"]),
            "vehicle_width_m": float(config["evaluation"]["vehicle_width_m"]),
            "map_resolutions_m": map_resolutions,
            "simulator_configuration_hashes": simulator_configs,
            "simulator_collision_source_hashes": {
                document["simulator_collision_source"]: document[
                    "simulator_collision_source_sha256"
                ]
                for document in scenario_documents
            },
            "simulator_collision_models": {
                document["scenario_id"]: document["simulator_collision_model"]
                for document in scenario_documents
            },
            "clean_map_hashes": sorted(
                {document["clean_map_hash"] for document in scenario_documents}
            ),
            "baked_map_hashes": {
                document["scenario_id"]: document["baked_map_hash"]
                for document in scenario_documents
            },
        },
        "dependencies": dependencies,
        "scenario_manifests": [
            {"path": str(Path(item).resolve()), "sha256": sha256_file(item)}
            for item in scenario_manifests
        ],
    }
    atomic_write_json(path, manifest)
    return manifest


CSV_FIELDS = (
    "candidate_id",
    "generation",
    "scenario_id",
    "fitness",
    "classification",
    "collision",
    "off_track",
    "planner_failure",
    "completed",
    "completion_time_s",
    "progress_fraction",
    "minimum_obstacle_clearance_m",
    "minimum_wall_clearance_m",
    "steering_total_variation_per_s",
    "planned_curvature_rate_rms_radpm2",
)


def append_result_csv(path: str | Path, row: dict[str, Any]) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    existing: list[dict[str, str]] = []
    if destination.is_file():
        with destination.open("r", newline="", encoding="utf-8") as stream:
            existing = list(csv.DictReader(stream))
    key = (str(row.get("candidate_id", "")), str(row.get("scenario_id", "")))
    existing = [
        item
        for item in existing
        if (item.get("candidate_id", ""), item.get("scenario_id", "")) != key
    ]
    fd, temporary = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    with os.fdopen(fd, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(existing)
        writer.writerow({key: row.get(key, "") for key in CSV_FIELDS})
    os.replace(temporary, destination)


def atomic_pickle(path: str | Path, payload: Any) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    try:
        with os.fdopen(fd, "wb") as stream:
            pickle.dump(payload, stream, protocol=pickle.HIGHEST_PROTOCOL)
        os.replace(temporary, destination)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def load_json_if_valid(path: str | Path) -> dict[str, Any] | None:
    candidate = Path(path)
    if not candidate.is_file():
        return None
    try:
        payload = json.loads(candidate.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None
    return payload if payload.get("valid", False) else None
