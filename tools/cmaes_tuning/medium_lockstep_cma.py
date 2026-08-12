#!/usr/bin/env python3
"""Run the resumable Stage-1 medium CMA-ES experiment in lockstep mode."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import pickle
import queue
import shutil
import subprocess
import sys
import threading
import time
from typing import Any, Iterable

import numpy as np

TOOL_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOL_ROOT))

from cmaes_tuning.configuration import load_config  # noqa: E402
from cmaes_tuning.experiment_logger import atomic_pickle  # noqa: E402
from cmaes_tuning.objective import candidate_fitness  # noqa: E402
from cmaes_tuning.parameter_space import ParameterSpace  # noqa: E402
from cmaes_tuning.schemas import atomic_write_json, sha256_file  # noqa: E402


CATEGORIES = (
    "straight",
    "corner_entry",
    "corner_mid",
    "corner_exit",
    "narrow_or_difficult",
)
EXACT_REPEAT_FIELDS = (
    "physics_state_sequence_hash",
    "scan_hash_sequence_hash",
    "static_geometry_sequence_hash",
    "avoid_waypoints_hash_sequence_hash",
    "controller_command_sequence_hash",
    "trajectory_hash",
    "confirmation_scan_index",
    "commitment_scan_index",
    "committed_obstacle_id",
    "selected_side",
    "collision",
    "off_track",
    "planner_failure",
    "scenario_success",
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _sha256_tree(path: Path) -> str:
    digest = hashlib.sha256()
    for item in sorted(
        candidate for candidate in path.rglob("*")
        if candidate.is_file()
        and "__pycache__" not in candidate.parts
        and candidate.suffix != ".pyc"
    ):
        digest.update(str(item.relative_to(path)).encode("utf-8"))
        digest.update(b"\0")
        digest.update(item.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def _load_pycma(output: Path):
    """Load pycma, including the already provisioned offline package if needed."""
    destinations = [
        output / "artifacts" / "python",
        Path("/tmp/cmaes_pycma_nodeps"),
        Path("/tmp/cmaes_pycma"),
    ]
    try:
        import cma  # type: ignore
        return cma
    except ImportError:
        pass
    for destination in destinations:
        if (destination / "cma" / "__init__.py").is_file():
            sys.path.insert(0, str(destination))
            import cma  # type: ignore
            return cma
    raise RuntimeError(
        "pycma is unavailable; install tools/cmaes_tuning/requirements.txt"
    )


def _mean(values: Iterable[float]) -> float:
    samples = [float(value) for value in values]
    return sum(samples) / len(samples) if samples else math.nan


def _copy_once(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        if sha256_file(source) != sha256_file(destination):
            raise RuntimeError(f"frozen artifact changed: {destination}")
        return
    shutil.copy2(source, destination)


def _scenario_documents(root: Path, split: str) -> list[tuple[Path, dict[str, Any]]]:
    results = []
    for path in sorted((root / split).glob("*/manifest.json")):
        document = json.loads(path.read_text(encoding="utf-8"))
        if document.get("dataset_split") != split:
            raise RuntimeError(f"split mismatch in {path}")
        results.append((path.resolve(), document))
    return results


def select_training_scenarios(
    documents: list[tuple[Path, dict[str, Any]]],
) -> list[tuple[Path, dict[str, Any]]]:
    """Select the first two ID-ordered examples/category and verify stratification."""
    selected: list[tuple[Path, dict[str, Any]]] = []
    for category in CATEGORIES:
        matching = sorted(
            (item for item in documents if item[1]["category"] == category),
            key=lambda item: item[1]["scenario_id"],
        )
        if len(matching) < 2:
            raise RuntimeError(f"category {category} has fewer than two training scenarios")
        selected.extend(matching[:2])
    # Preserve dataset ID ordering as the permanent CRN order. The generated v2
    # dataset cycles categories/lateral bands, so this is training_000..009.
    selected.sort(key=lambda item: item[1]["scenario_id"])
    counts = {category: 0 for category in CATEGORIES}
    lateral = {band: 0 for band in ("left", "center", "right")}
    for _, document in selected:
        counts[document["category"]] += 1
        lateral[document["lateral_band"]] += 1
    if any(value != 2 for value in counts.values()):
        raise RuntimeError(f"training category selection is not 2/category: {counts}")
    if max(lateral.values()) - min(lateral.values()) > 1:
        raise RuntimeError(f"training lateral selection is unbalanced: {lateral}")
    return selected


def _freeze_scenarios(
    selected: list[tuple[Path, dict[str, Any]]], output: Path, split: str,
) -> list[Path]:
    frozen = []
    for source, document in selected:
        target = output / "artifacts" / "scenarios" / split / document["scenario_id"] / "manifest.json"
        _copy_once(source, target)
        frozen.append(target.resolve())
    return frozen


def _distribution(paths: list[Path]) -> dict[str, Any]:
    categories = {category: 0 for category in CATEGORIES}
    lateral = {band: 0 for band in ("left", "center", "right")}
    cross: dict[str, int] = {}
    for path in paths:
        document = json.loads(path.read_text(encoding="utf-8"))
        categories[document["category"]] += 1
        lateral[document["lateral_band"]] += 1
        key = f"{document['category']}:{document['lateral_band']}"
        cross[key] = cross.get(key, 0) + 1
    return {"count": len(paths), "category": categories, "lateral": lateral, "cross": cross}


def _keys(paths: list[Path], seeds: list[int]) -> list[dict[str, Any]]:
    result = []
    for path in paths:
        scenario_id = json.loads(path.read_text(encoding="utf-8"))["scenario_id"]
        for seed in seeds:
            result.append({
                "scenario_id": scenario_id,
                "scenario_manifest": str(path),
                "simulator_seed": int(seed),
            })
    return result


def _valid_cached_attempt(
    attempt: Path, candidate_hash: str, key: dict[str, Any], sigma: float,
) -> dict[str, Any] | None:
    required = (
        attempt / "lockstep_result.json",
        attempt / "episode_result.json",
        attempt / "runner_status.json",
    )
    if not all(path.is_file() for path in required):
        return None
    try:
        lockstep = json.loads(required[0].read_text(encoding="utf-8"))
        episode = json.loads(required[1].read_text(encoding="utf-8"))
        status = json.loads(required[2].read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return None
    runtime = status.get("simulator_runtime_configuration", {})
    if not lockstep.get("valid") or not episode.get("valid"):
        return None
    if status.get("candidate_sha256") != candidate_hash:
        return None
    if lockstep.get("scenario_id") != key["scenario_id"]:
        return None
    if int(lockstep.get("simulator_seed", -1)) != int(key["simulator_seed"]):
        return None
    if abs(float(runtime.get("scan_noise_std_m", math.nan)) - sigma) > 1.0e-15:
        return None
    return {"attempt": str(attempt), "lockstep": lockstep, "episode": episode}


class RolloutExecutor:
    def __init__(
        self, workspace: Path, config: Path, output: Path, parallel: int,
        domain_start: int, sigma: float, post_obstacle_distance: float,
        maximum_duration: float, dds_transport: str, retries: int,
    ) -> None:
        self.workspace = workspace
        self.config = config
        self.output = output
        self.sigma = sigma
        self.post_obstacle_distance = post_obstacle_distance
        self.maximum_duration = maximum_duration
        self.dds_transport = dds_transport
        self.retries = retries
        self.domains: queue.Queue[int] = queue.Queue()
        for index in range(parallel):
            domain = domain_start + index
            if domain > 230:
                raise ValueError("ROS domain pool exceeds 230")
            self.domains.put(domain)
        self.parallel = parallel
        self.print_lock = threading.Lock()
        self.finished = 0
        self.total = 0
        self.started = time.monotonic()

    def _run_one(
        self, candidate: Path, key: dict[str, Any], rollout_root: Path,
    ) -> dict[str, Any]:
        candidate_hash = sha256_file(candidate)
        for attempt in sorted(rollout_root.glob("attempt_*")):
            cached = _valid_cached_attempt(attempt, candidate_hash, key, self.sigma)
            if cached is not None:
                return cached
        for attempt_index in range(self.retries + 1):
            attempt = rollout_root / f"attempt_{attempt_index:02d}"
            if attempt.exists():
                continue
            domain = self.domains.get()
            try:
                environment = dict(os.environ)
                environment["ROS_DOMAIN_ID"] = str(domain)
                environment.setdefault("MPLCONFIGDIR", "/tmp/cmaes_matplotlib")
                command = [
                    sys.executable,
                    str(TOOL_ROOT / "lockstep_episode.py"),
                    "--workspace", str(self.workspace),
                    "--config", str(self.config),
                    "--candidate", str(candidate),
                    "--scenario", str(key["scenario_manifest"]),
                    "--output", str(attempt),
                    "--simulator-seed", str(key["simulator_seed"]),
                    "--scan-noise-std", str(self.sigma),
                    "--post-obstacle-distance", str(self.post_obstacle_distance),
                    "--maximum-duration", str(self.maximum_duration),
                    "--dds-transport", self.dds_transport,
                ]
                completed = subprocess.run(
                    command, cwd=self.workspace, env=environment, text=True,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False,
                )
                attempt.mkdir(parents=True, exist_ok=True)
                (attempt / "medium_cma_stdout.log").write_text(
                    completed.stdout, encoding="utf-8")
                cached = _valid_cached_attempt(attempt, candidate_hash, key, self.sigma)
                if completed.returncode == 0 and cached is not None:
                    return cached
                atomic_write_json(attempt / "medium_cma_failure.json", {
                    "returncode": completed.returncode,
                    "domain_id": domain,
                    "scenario_id": key["scenario_id"],
                    "simulator_seed": key["simulator_seed"],
                    "candidate_sha256": candidate_hash,
                })
            finally:
                self.domains.put(domain)
        raise RuntimeError(
            f"rollout failed after {self.retries + 1} attempts: "
            f"{candidate.name}/{key['scenario_id']}/{key['simulator_seed']}"
        )

    def run(
        self, candidate: Path, keys: list[dict[str, Any]], episode_root: Path,
    ) -> list[dict[str, Any]]:
        jobs = []
        for key in keys:
            rollout_root = (
                episode_root / key["scenario_id"] / f"seed_{key['simulator_seed']}"
            )
            jobs.append((key, rollout_root))
        self.total = len(jobs)
        self.finished = 0
        self.started = time.monotonic()
        indexed_results: dict[int, dict[str, Any]] = {}
        with ThreadPoolExecutor(max_workers=self.parallel) as pool:
            futures = {
                pool.submit(self._run_one, candidate, key, root): index
                for index, (key, root) in enumerate(jobs)
            }
            for future in as_completed(futures):
                index = futures[future]
                indexed_results[index] = future.result()
                self.finished += 1
                elapsed = time.monotonic() - self.started
                rate = self.finished / elapsed if elapsed > 0 else 0.0
                remaining = (self.total - self.finished) / rate if rate > 0 else math.inf
                with self.print_lock:
                    print(
                        f"  rollout {self.finished:03d}/{self.total:03d} "
                        f"elapsed={elapsed:.1f}s eta={remaining:.1f}s",
                        flush=True,
                    )
        return [indexed_results[index] for index in range(len(jobs))]


def summarize_candidate(
    candidate_id: str, generation: int | str, candidate_yaml: Path,
    normalized: list[float], physical: dict[str, float],
    keys: list[dict[str, Any]], results: list[dict[str, Any]], config: dict[str, Any],
) -> dict[str, Any]:
    episodes = [item["episode"] for item in results]
    aggregate = candidate_fitness(episodes, config)
    collisions = sum(bool(item["lockstep"]["collision"]) for item in results)
    off_tracks = sum(bool(item["lockstep"]["off_track"]) for item in results)
    planner_failures = sum(bool(item["lockstep"]["planner_failure"]) for item in results)
    successes = sum(bool(item["lockstep"]["scenario_success"]) for item in results)
    metrics = [episode["metrics"] for episode in episodes]
    rollout_rows = []
    for key, result in zip(keys, results):
        episode_metrics = result["episode"]["metrics"]
        lockstep = result["lockstep"]
        rollout_rows.append({
            "scenario_id": key["scenario_id"],
            "simulator_seed": key["simulator_seed"],
            "attempt": result["attempt"],
            "success": bool(lockstep["scenario_success"]),
            "collision": bool(lockstep["collision"]),
            "off_track": bool(lockstep["off_track"]),
            "planner_failure": bool(lockstep["planner_failure"]),
            "fitness": float(lockstep["fitness"]),
            "minimum_obstacle_clearance_m": float(
                episode_metrics["minimum_obstacle_clearance_m"]),
            "completion_time_s": float(episode_metrics["completion_time_s"]),
            "steering_tv_per_s": float(
                episode_metrics["steering_total_variation_per_s"]),
            "curvature_rate_rms": float(
                episode_metrics["planned_curvature_rate_rms_radpm2"]),
            "confirmation_scan_index": lockstep.get("confirmation_scan_index"),
            "commitment_scan_index": lockstep.get("commitment_scan_index"),
            "committed_target_d": lockstep.get("committed_target_d"),
            "selected_side": lockstep.get("selected_side"),
            "path_hash": lockstep["avoid_waypoints_hash_sequence_hash"],
            "trajectory_hash": lockstep["trajectory_hash"],
        })
    return {
        "schema": "cma_medium_candidate/1",
        "candidate_id": candidate_id,
        "generation": generation,
        "candidate_yaml": str(candidate_yaml),
        "candidate_sha256": sha256_file(candidate_yaml),
        "normalized_vector": [float(value) for value in normalized],
        "physical_parameters": {name: float(value) for name, value in physical.items()},
        "rollout_count": len(results),
        "hard_failure_count": int(aggregate["failure_count"]),
        "collision_count": collisions,
        "off_track_count": off_tracks,
        "planner_failure_count": planner_failures,
        "success_count": successes,
        "success_rate": successes / len(results),
        "mean_performance_cost": float(aggregate["mean_performance_cost"]),
        "cvar90": float(aggregate["cvar_performance_cost"]),
        "quality_cost": float(aggregate["quality_cost"]),
        "fitness": float(aggregate["fitness"]),
        "minimum_clearance_m": min(
            float(metric["minimum_obstacle_clearance_m"]) for metric in metrics),
        "mean_completion_time_s": _mean(
            metric["completion_time_s"] for metric in metrics),
        "mean_steering_tv_per_s": _mean(
            metric["steering_total_variation_per_s"] for metric in metrics),
        "mean_curvature_rate_rms": _mean(
            metric["planned_curvature_rate_rms_radpm2"] for metric in metrics),
        "failure_multiplier": float(aggregate["failure_multiplier"]),
        "j_max": float(aggregate["j_max"]),
        "rollouts": rollout_rows,
    }


def _write_candidate_yaml(
    space: ParameterSpace, baseline: Path, path: Path, normalized: list[float],
) -> dict[str, Any]:
    metadata_path = path.with_suffix(".json")
    if path.is_file() and metadata_path.is_file():
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        previous = [float(value) for value in metadata["normalized"]]
        if previous != [float(value) for value in normalized]:
            raise RuntimeError(f"candidate vector changed on resume: {path}")
        return metadata
    metadata = space.write_candidate_yaml(baseline, path, normalized)
    atomic_write_json(metadata_path, metadata)
    return metadata


def evaluate_candidate(
    executor: RolloutExecutor, space: ParameterSpace, baseline: Path,
    output: Path, candidate_id: str, generation: int | str,
    normalized: list[float], keys: list[dict[str, Any]], config: dict[str, Any],
    split: str,
) -> dict[str, Any]:
    candidate_yaml = output / "candidates" / f"{candidate_id}.yaml"
    metadata = _write_candidate_yaml(space, baseline, candidate_yaml, normalized)
    result_path = output / "candidate_results" / split / f"{candidate_id}.json"
    if result_path.is_file():
        result = json.loads(result_path.read_text(encoding="utf-8"))
        if result.get("candidate_sha256") == metadata["candidate_sha256"]:
            print(f"[{candidate_id}] cached aggregate", flush=True)
            return result
    print(f"[{candidate_id}] {len(keys)} rollouts", flush=True)
    results = executor.run(
        candidate_yaml, keys, output / "episodes" / split / candidate_id)
    summary = summarize_candidate(
        candidate_id, generation, candidate_yaml, normalized,
        metadata["physical"], keys, results, config)
    atomic_write_json(result_path, summary)
    return summary


def _covariance_payload(strategy: Any) -> dict[str, Any]:
    covariance = np.asarray(strategy.C, dtype=float)
    diagonal = np.diag(covariance)
    standard = np.sqrt(np.maximum(diagonal, 0.0))
    denominator = np.outer(standard, standard)
    correlation = np.divide(
        covariance, denominator, out=np.zeros_like(covariance), where=denominator > 0.0)
    return {
        "matrix": covariance.tolist(),
        "diagonal": diagonal.tolist(),
        "correlation": correlation.tolist(),
        "eigenvalues": np.linalg.eigvalsh(covariance).tolist(),
        "condition_number": float(np.linalg.cond(covariance)),
    }


def generation_zero_sanity(
    executor: RolloutExecutor, space: ParameterSpace, baseline: Path, output: Path,
    baseline_result: dict[str, Any], candidates: list[dict[str, Any]],
    training_keys: list[dict[str, Any]], config: dict[str, Any],
) -> dict[str, Any]:
    finite = all(
        all(math.isfinite(float(value)) for value in candidate["normalized_vector"])
        and all(math.isfinite(float(value)) for value in candidate["physical_parameters"].values())
        and math.isfinite(float(candidate["fitness"]))
        for candidate in candidates
    )
    fitness_diverse = len({round(float(item["fitness"]), 12) for item in candidates}) > 1
    yaml_changed = all(
        item["candidate_sha256"] != baseline_result["candidate_sha256"] for item in candidates)
    baseline_by_key = {
        (item["scenario_id"], item["simulator_seed"]): item
        for item in baseline_result["rollouts"]
    }
    behavior_changed_candidates = []
    for candidate in candidates:
        changed = any(
            rollout["path_hash"] != baseline_by_key[
                (rollout["scenario_id"], rollout["simulator_seed"])
            ]["path_hash"]
            or rollout["trajectory_hash"] != baseline_by_key[
                (rollout["scenario_id"], rollout["simulator_seed"])
            ]["trajectory_hash"]
            for rollout in candidate["rollouts"]
        )
        if changed:
            behavior_changed_candidates.append(candidate["candidate_id"])
    groups: dict[int, list[float]] = {}
    for candidate in candidates:
        groups.setdefault(int(candidate["hard_failure_count"]), []).append(
            float(candidate["fitness"]))
    ordered_failures = sorted(groups)
    dominance_checks = []
    safety_dominance = True
    for lower, upper in zip(ordered_failures, ordered_failures[1:]):
        passed = max(groups[lower]) < min(groups[upper])
        dominance_checks.append({
            "lower_failure_count": lower,
            "upper_failure_count": upper,
            "max_lower_fitness": max(groups[lower]),
            "min_upper_fitness": min(groups[upper]),
            "passed": passed,
        })
        safety_dominance = safety_dominance and passed

    reference = candidates[0]
    reference_yaml = Path(reference["candidate_yaml"])
    repeat_key = training_keys[0]
    repeat_result = executor.run(
        reference_yaml, [repeat_key], output / "sanity" / "repeat_gen_000_cand_000")
    repeated = repeat_result[0]["lockstep"]
    original = reference["rollouts"][0]
    exact_agreement = {
        field: repeated.get(field) == original.get(
            "path_hash" if field == "avoid_waypoints_hash_sequence_hash" else
            "trajectory_hash" if field == "trajectory_hash" else field)
        for field in EXACT_REPEAT_FIELDS
    }
    # rollout rows only retain the two most important sequence hashes. Load the
    # original full lockstep result for all exact acceptance fields.
    original_attempt = Path(reference["rollouts"][0]["attempt"])
    original_full = json.loads(
        (original_attempt / "lockstep_result.json").read_text(encoding="utf-8"))
    exact_agreement = {
        field: repeated.get(field) == original_full.get(field) for field in EXACT_REPEAT_FIELDS
    }
    target_delta = abs(
        float(repeated.get("committed_target_d") or 0.0)
        - float(original_full.get("committed_target_d") or 0.0))
    fitness_delta = abs(float(repeated["fitness"]) - float(original_full["fitness"]))
    repeatability = (
        all(exact_agreement.values()) and target_delta <= 1.0e-12 and fitness_delta <= 1.0e-12)
    checks = {
        "finite_parameters_and_fitness": finite,
        "fitness_not_all_identical": fitness_diverse,
        "candidate_yaml_changed_from_baseline": yaml_changed,
        "planner_behavior_changed": bool(behavior_changed_candidates),
        "behavior_changed_candidate_ids": behavior_changed_candidates,
        "safety_dominance": safety_dominance,
        "safety_dominance_comparisons": dominance_checks,
        "lockstep_repeatability": repeatability,
        "repeat_exact_field_agreement": exact_agreement,
        "repeat_target_d_abs_delta_m": target_delta,
        "repeat_fitness_abs_delta": fitness_delta,
    }
    checks["passed"] = all((
        finite, fitness_diverse, yaml_changed, bool(behavior_changed_candidates),
        safety_dominance, repeatability,
    ))
    atomic_write_json(output / "generation_000_sanity.json", checks)
    return checks


def _pickle_load(path: Path) -> Any:
    with path.open("rb") as stream:
        return pickle.load(stream)


def _source_manifest(
    workspace: Path, output: Path, config_path: Path, parameter_path: Path,
    baseline: Path, training: list[Path], validation: list[Path],
    training_keys: list[dict[str, Any]], validation_keys: list[dict[str, Any]],
    args: argparse.Namespace, cma_module: Any,
) -> dict[str, Any]:
    source_paths = {
        "medium_runner": Path(__file__).resolve(),
        "lockstep_episode": TOOL_ROOT / "lockstep_episode.py",
        "objective": TOOL_ROOT / "cmaes_tuning" / "objective.py",
        "parameter_space": TOOL_ROOT / "cmaes_tuning" / "parameter_space.py",
        "detector": workspace / "src/obstacle_detector/src/obstacle_detector_node.cpp",
        "planner": workspace / "src/local_planning/src/local_planner_node.cpp",
        "state_machine": workspace / "src/state_machine/src/state_machine_node.cpp",
        "controller": workspace / "src/f1tenth_control/control_code/control_map_node.cpp",
    }
    revision = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=workspace, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False).stdout.strip()
    dirty = subprocess.run(
        ["git", "status", "--porcelain"], cwd=workspace, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False).stdout
    cma_path = Path(cma_module.__file__).resolve().parent
    frozen_cma = output / "artifacts" / "python" / "cma"
    if not frozen_cma.exists():
        frozen_cma.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(
            cma_path, frozen_cma,
            ignore=shutil.ignore_patterns("__pycache__", "*.pyc"),
        )
    return {
        "schema": "cma_medium_lockstep_experiment/1",
        "created_utc": _utc_now(),
        "experiment_id": output.name,
        "source_revision": revision,
        "source_tree_dirty": bool(dirty.strip()),
        "source_sha256": {
            name: sha256_file(path) for name, path in source_paths.items() if path.is_file()
        },
        "config": {"path": str(config_path), "sha256": sha256_file(config_path)},
        "parameter_space": {
            "path": str(parameter_path), "sha256": sha256_file(parameter_path),
            "dimension": 10,
        },
        "baseline": {"path": str(baseline), "sha256": sha256_file(baseline)},
        "conditions": {
            "localization_mode": "ground_truth",
            "execution_mode": "deterministic_lockstep",
            "scan_publication": "fresh_only",
            "scan_noise_std_m": args.scan_noise_std,
            "fixed_operating_speed_mps": 4.0,
            "physics_timestep_s": 0.01,
            "episode_scope": "spawn_to_obstacle_plus_3m_or_safety_failure_or_timeout",
            "maximum_simulated_duration_s": args.maximum_duration,
            "obstacle_input": "simulator_lidar_only",
            "obstacle_ground_truth_available_to_planner": False,
            "production_async_used_for_fitness": False,
        },
        "cma": {
            "implementation": "pycma ask/tell",
            "pycma_version": getattr(cma_module, "__version__", "unknown"),
            "pycma_loaded_source": str(cma_path),
            "pycma_frozen_source": str(frozen_cma),
            "pycma_source_tree_sha256": _sha256_tree(frozen_cma),
            "population_size": 10,
            "generations": 5,
            "sigma0": args.sigma0,
            "cma_seed": args.cma_seed,
            "bounds_normalized": [0.0, 1.0],
        },
        "objective": {
            "aggregation": "mean_plus_CVaR90",
            "hard_safety_dominance": True,
            "failure_types": ["collision", "off_track", "planner_failure"],
        },
        "training": {
            "distribution": _distribution(training),
            "ordered_scenario_ids": [
                json.loads(path.read_text(encoding="utf-8"))["scenario_id"] for path in training
            ],
            "noise_seeds": args.training_seeds,
            "ordered_crn_rollout_keys": training_keys,
            "expected_cma_episodes": 10 * 5 * len(training_keys),
            "baseline_episodes": len(training_keys),
        },
        "validation": {
            "used_by_ask_tell": False,
            "distribution": _distribution(validation),
            "ordered_scenario_ids": [
                json.loads(path.read_text(encoding="utf-8"))["scenario_id"] for path in validation
            ],
            "unseen_noise_seed_by_scenario": {
                key["scenario_id"]: key["simulator_seed"] for key in validation_keys
            },
            "ordered_rollout_keys": validation_keys,
        },
        "parallel_execution": {
            "workers": args.parallel,
            "domain_start": args.domain_start,
            "maximum_infrastructure_retries": args.infrastructure_retries,
            "semantic_effect": "none; each rollout is isolated and internally lockstep",
        },
        "resume_contract": {
            "after_ask_pickle_per_generation": True,
            "after_tell_pickle_per_generation": True,
            "rollout_cache_key": [
                "candidate_sha256", "scenario_id", "simulator_seed", "scan_noise_std_m"
            ],
        },
    }


def _top_couplings(strategy: Any, names: list[str], limit: int = 10) -> list[dict[str, Any]]:
    correlation = np.asarray(_covariance_payload(strategy)["correlation"], dtype=float)
    pairs = []
    for first in range(len(names)):
        for second in range(first + 1, len(names)):
            pairs.append({
                "parameter_a": names[first], "parameter_b": names[second],
                "correlation": float(correlation[first, second]),
                "absolute_correlation": abs(float(correlation[first, second])),
            })
    return sorted(pairs, key=lambda item: item["absolute_correlation"], reverse=True)[:limit]


def _parameter_comparison(
    space: ParameterSpace, baseline_physical: dict[str, float], best: dict[str, Any],
) -> list[dict[str, Any]]:
    rows = []
    for definition in space.definitions:
        before = float(baseline_physical[definition.name])
        after = float(best["physical_parameters"][definition.name])
        span = definition.upper - definition.lower
        tolerance = max(1.0e-9 * span, 1.0e-12)
        bound = (
            "lower" if abs(after - definition.lower) <= tolerance else
            "upper" if abs(after - definition.upper) <= tolerance else None
        )
        rows.append({
            "name": definition.name,
            "baseline": before,
            "optimized": after,
            "absolute_change": after - before,
            "relative_change_percent": (
                100.0 * (after - before) / abs(before) if before != 0.0 else None),
            "search_range_fraction_change": (after - before) / span,
            "lower": definition.lower,
            "upper": definition.upper,
            "at_bound": bound,
        })
    return rows


def build_final_report(
    output: Path, space: ParameterSpace, strategy: Any,
    baseline_train: dict[str, Any], generation_reports: list[dict[str, Any]],
    validation_results: list[dict[str, Any]], all_candidates: list[dict[str, Any]],
) -> dict[str, Any]:
    ranked = sorted(all_candidates, key=lambda item: float(item["fitness"]))
    best = ranked[0]
    baseline_validation = next(
        item for item in validation_results if item["candidate_id"] == "validation_baseline")
    best_validation = next(
        item for item in validation_results
        if item.get("source_candidate_id") == best["candidate_id"])
    training_improved = float(best["fitness"]) < float(baseline_train["fitness"])
    validation_safety_not_worse = (
        int(best_validation["hard_failure_count"])
        <= int(baseline_validation["hard_failure_count"])
    )
    validation_improved = float(best_validation["fitness"]) < float(
        baseline_validation["fitness"])
    overfitting = training_improved and (
        not validation_safety_not_worse or not validation_improved)
    parameter_changes = _parameter_comparison(
        space, space.baseline_physical(), best)
    bound_items = [item for item in parameter_changes if item["at_bound"] is not None]
    report = {
        "schema": "cma_medium_final_report/1",
        "completed_utc": _utc_now(),
        "baseline_training": baseline_train,
        "generation_summary": [{
            "generation": item["generation"],
            "best_fitness": item["best_fitness"],
            "mean_fitness": item["mean_fitness"],
            "best_candidate_id": item["best_candidate_id"],
            "best_so_far_candidate_id": item["best_so_far_candidate_id"],
            "best_so_far_fitness": item["best_so_far_fitness"],
        } for item in generation_reports],
        "final_best": best,
        "parameter_changes": parameter_changes,
        "bound_attached_parameters": bound_items,
        "strong_covariance_couplings": _top_couplings(strategy, space.names),
        "validation_results": validation_results,
        "comparison": {
            "training_fitness_delta": float(best["fitness"]) - float(baseline_train["fitness"]),
            "training_hard_failure_delta": (
                int(best["hard_failure_count"]) - int(baseline_train["hard_failure_count"])),
            "training_success_rate_delta": (
                float(best["success_rate"]) - float(baseline_train["success_rate"])),
            "validation_fitness_delta": (
                float(best_validation["fitness"]) - float(baseline_validation["fitness"])),
            "validation_hard_failure_delta": (
                int(best_validation["hard_failure_count"])
                - int(baseline_validation["hard_failure_count"])),
            "validation_success_rate_delta": (
                float(best_validation["success_rate"])
                - float(baseline_validation["success_rate"])),
        },
        "overfitting": {
            "detected": overfitting,
            "training_improved": training_improved,
            "validation_fitness_improved": validation_improved,
            "validation_safety_not_worse": validation_safety_not_worse,
        },
        "medium_cma_conclusion": (
            "scale_up" if training_improved and validation_safety_not_worse and not overfitting
            else "hold_and_review"
        ),
        "full_scale_recommendation": {
            "population_size": 16,
            "generations": 15,
            "training_scenarios": 25,
            "noise_seeds_per_scenario": 2,
            "fixed_speed_mps": 4.0,
            "execution_mode": "deterministic_lockstep",
            "validation": "40 scenarios x one or more unseen seeds; never in ask/tell",
        },
    }
    atomic_write_json(output / "final_report.json", report)
    shutil.copy2(Path(best["candidate_yaml"]), output / "final_best.yaml")
    return report


def run(args: argparse.Namespace) -> int:
    workspace = Path(args.workspace).resolve()
    output = (workspace / args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    cma = _load_pycma(output)
    config_source = Path(args.config).resolve()
    config = load_config(config_source)
    if abs(float(config["controller"]["max_speed_mps"]) - 4.0) > 1.0e-12:
        raise RuntimeError("controller max_speed_mps must be exactly 4.0")
    if args.training_seeds[0] == args.training_seeds[1]:
        raise ValueError("two distinct training seeds are required")
    if any(seed in args.training_seeds for seed in args.validation_seeds):
        raise ValueError("validation and training seeds overlap")

    parameter_source = (workspace / config["paths"]["parameter_space_yaml"]).resolve()
    baseline_source = (workspace / config["paths"]["baseline_planner_yaml"]).resolve()
    frozen_config = output / "artifacts" / "tuning_config.yaml"
    frozen_parameter = output / "artifacts" / "parameter_space.yaml"
    frozen_baseline = output / "artifacts" / "baseline.yaml"
    for source, target in (
        (config_source, frozen_config),
        (parameter_source, frozen_parameter),
        (baseline_source, frozen_baseline),
    ):
        _copy_once(source, target)
    space = ParameterSpace.load(frozen_parameter)
    if space.dimension != 10:
        raise RuntimeError(f"expected 10 parameters, found {space.dimension}")
    space.validate_baseline(frozen_baseline)

    dataset_root = (workspace / args.scenario_root).resolve()
    training_documents = _scenario_documents(dataset_root, "training")
    validation_documents = _scenario_documents(dataset_root, "validation")
    if len(training_documents) != 25 or len(validation_documents) != 40:
        raise RuntimeError(
            f"expected 25/40 dataset, found {len(training_documents)}/{len(validation_documents)}")
    training_selected = select_training_scenarios(training_documents)
    validation_selected = sorted(
        validation_documents, key=lambda item: item[1]["scenario_id"])
    training_paths = _freeze_scenarios(training_selected, output, "training")
    validation_paths = _freeze_scenarios(validation_selected, output, "validation")
    training_keys = _keys(training_paths, args.training_seeds)
    validation_keys = [
        {
            "scenario_id": json.loads(path.read_text(encoding="utf-8"))["scenario_id"],
            "scenario_manifest": str(path),
            "simulator_seed": args.validation_seeds[index],
        }
        for index, path in enumerate(validation_paths)
    ]
    manifest_path = output / "experiment_manifest.json"
    manifest = _source_manifest(
        workspace, output, frozen_config, frozen_parameter, frozen_baseline,
        training_paths, validation_paths, training_keys, validation_keys, args, cma)
    if manifest_path.is_file():
        previous = json.loads(manifest_path.read_text(encoding="utf-8"))
        for key in ("conditions", "cma", "training", "validation"):
            if previous.get(key) != manifest.get(key):
                raise RuntimeError(f"resume contract differs in experiment manifest field: {key}")
        manifest = previous
    else:
        atomic_write_json(manifest_path, manifest)

    executor = RolloutExecutor(
        workspace, frozen_config, output, args.parallel, args.domain_start,
        args.scan_noise_std, args.post_obstacle_distance, args.maximum_duration,
        args.dds_transport, args.infrastructure_retries,
    )
    baseline_z = space.baseline_z()
    baseline_result = evaluate_candidate(
        executor, space, frozen_baseline, output, "training_baseline", "baseline",
        baseline_z, training_keys, config, "training")
    atomic_write_json(output / "baseline_training.json", baseline_result)
    if args.stop_after == "baseline":
        return 0

    options = {
        "bounds": [0.0, 1.0],
        "popsize": 10,
        "seed": args.cma_seed,
        "verbose": -9,
        "verb_disp": 0,
    }
    checkpoint = output / "checkpoints" / "cma_after_tell.pkl"
    checkpoint_json = output / "checkpoints" / "cma_after_tell.json"
    if checkpoint.is_file() and checkpoint_json.is_file():
        strategy = _pickle_load(checkpoint)
        state = json.loads(checkpoint_json.read_text(encoding="utf-8"))
        next_generation = int(state["next_generation"])
    else:
        strategy = cma.CMAEvolutionStrategy(baseline_z, args.sigma0, options)
        next_generation = 0
    all_candidates: list[dict[str, Any]] = []
    generation_reports: list[dict[str, Any]] = []
    for existing in sorted((output / "generations").glob("generation_*/generation_result.json")):
        payload = json.loads(existing.read_text(encoding="utf-8"))
        generation_reports.append(payload)
        all_candidates.extend(payload["candidates"])

    for generation in range(next_generation, 5):
        generation_root = output / "generations" / f"generation_{generation:03d}"
        plan_path = generation_root / "ask_plan.json"
        after_ask_path = generation_root / "cma_after_ask.pkl"
        if plan_path.is_file() and after_ask_path.is_file():
            plan = json.loads(plan_path.read_text(encoding="utf-8"))
            strategy = _pickle_load(after_ask_path)
            vectors = [[float(value) for value in row] for row in plan["candidate_vectors"]]
        else:
            mean_before = [float(value) for value in strategy.mean]
            sigma_before = float(strategy.sigma)
            vectors = [[float(value) for value in row] for row in strategy.ask()]
            if len(vectors) != 10 or any(
                len(row) != space.dimension or any(value < 0.0 or value > 1.0 for value in row)
                for row in vectors
            ):
                raise RuntimeError("pycma ask returned an invalid bounded population")
            plan = {
                "schema": "cma_medium_ask_plan/1",
                "generation": generation,
                "mean_before_ask": mean_before,
                "sigma_before_ask": sigma_before,
                "candidate_vectors": vectors,
                "ordered_training_rollout_keys": training_keys,
            }
            atomic_write_json(plan_path, plan)
            atomic_pickle(after_ask_path, strategy)

        candidate_results = []
        for index, vector in enumerate(vectors):
            candidate_id = f"gen_{generation:03d}_cand_{index:03d}"
            result = evaluate_candidate(
                executor, space, frozen_baseline, output, candidate_id, generation,
                vector, training_keys, config, "training")
            candidate_results.append(result)
        fitnesses = [float(item["fitness"]) for item in candidate_results]
        if generation == 0:
            sanity = generation_zero_sanity(
                executor, space, frozen_baseline, output, baseline_result,
                candidate_results, training_keys, config)
            if not sanity["passed"]:
                atomic_write_json(output / "experiment_halted.json", {
                    "reason": "generation_0_sanity_failed", "checks": sanity,
                    "medium_cma_completed": False,
                })
                print(json.dumps(sanity, indent=2, sort_keys=True))
                return 4
        strategy.tell(vectors, fitnesses)
        ranked = sorted(candidate_results, key=lambda item: float(item["fitness"]))
        combined = all_candidates + candidate_results
        best_so_far = min(combined, key=lambda item: float(item["fitness"]))
        generation_payload = {
            "schema": "cma_medium_generation/1",
            "generation": generation,
            "mean_before": plan["mean_before_ask"],
            "sigma_before": plan["sigma_before_ask"],
            "mean_after": [float(value) for value in strategy.mean],
            "sigma_after": float(strategy.sigma),
            "covariance_after": _covariance_payload(strategy),
            "best_fitness": float(ranked[0]["fitness"]),
            "mean_fitness": _mean(fitnesses),
            "best_candidate_id": ranked[0]["candidate_id"],
            "best_so_far_candidate_id": best_so_far["candidate_id"],
            "best_so_far_fitness": float(best_so_far["fitness"]),
            "ranking": [item["candidate_id"] for item in ranked],
            "candidates": candidate_results,
        }
        atomic_write_json(generation_root / "generation_result.json", generation_payload)
        atomic_pickle(generation_root / "cma_after_tell.pkl", strategy)
        atomic_pickle(checkpoint, strategy)
        atomic_write_json(checkpoint_json, {
            "schema": "cma_medium_checkpoint/1",
            "completed_generation": generation,
            "next_generation": generation + 1,
            "mean": generation_payload["mean_after"],
            "sigma": generation_payload["sigma_after"],
            "best_candidate_id": best_so_far["candidate_id"],
            "best_fitness": best_so_far["fitness"],
            "exact_resume_pickle": str(checkpoint),
        })
        all_candidates.extend(candidate_results)
        generation_reports.append(generation_payload)
        print(
            f"generation {generation}: best={generation_payload['best_fitness']:.9f} "
            f"mean={generation_payload['mean_fitness']:.9f} "
            f"best_so_far={generation_payload['best_so_far_fitness']:.9f}",
            flush=True,
        )
        if args.stop_after == "generation0" and generation == 0:
            return 0

    ranked_candidates = sorted(all_candidates, key=lambda item: float(item["fitness"]))
    top_three = []
    seen_vectors = set()
    for candidate in ranked_candidates:
        key = tuple(round(float(value), 15) for value in candidate["normalized_vector"])
        if key in seen_vectors:
            continue
        seen_vectors.add(key)
        top_three.append(candidate)
        if len(top_three) == 3:
            break
    atomic_write_json(output / "top_three_training.json", {
        "note": "final best is rank 1; validation evaluates the deduplicated top 3 including it",
        "candidates": top_three,
    })
    if args.stop_after == "optimization":
        return 0

    validation_results = []
    baseline_validation = evaluate_candidate(
        executor, space, frozen_baseline, output, "validation_baseline", "validation",
        baseline_z, validation_keys, config, "validation")
    validation_results.append(baseline_validation)
    for rank, candidate in enumerate(top_three, start=1):
        candidate_id = f"validation_rank_{rank:02d}_{candidate['candidate_id']}"
        result = evaluate_candidate(
            executor, space, frozen_baseline, output, candidate_id, "validation",
            candidate["normalized_vector"], validation_keys, config, "validation")
        result["source_candidate_id"] = candidate["candidate_id"]
        result["training_rank"] = rank
        atomic_write_json(
            output / "candidate_results" / "validation" / f"{candidate_id}.json", result)
        validation_results.append(result)
    atomic_write_json(output / "validation_comparison.json", {
        "schema": "cma_medium_validation/1",
        "ask_tell_usage": False,
        "unseen_seed_count_per_scenario": 1,
        "results": validation_results,
    })
    final_report = build_final_report(
        output, space, strategy, baseline_result, generation_reports,
        validation_results, all_candidates)
    print(json.dumps({
        "final_best": final_report["final_best"]["candidate_id"],
        "training_fitness": final_report["final_best"]["fitness"],
        "validation_fitness_delta": final_report["comparison"]["validation_fitness_delta"],
        "conclusion": final_report["medium_cma_conclusion"],
    }, indent=2, sort_keys=True))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", default=str(TOOL_ROOT.parents[1]))
    parser.add_argument(
        "--config", default=str(TOOL_ROOT / "config" / "tuning_config.yaml"))
    parser.add_argument(
        "--scenario-root",
        default="runs/cmaes_tuning/e2e_timing_10hz_v1/scenarios")
    parser.add_argument("--output", default="runs/cmaes_tuning/medium_lockstep_stage1_v2")
    parser.add_argument("--training-seeds", nargs=2, type=int, default=[310101, 310202])
    parser.add_argument(
        "--validation-seeds", nargs=40, type=int,
        default=list(range(410001, 410041)))
    parser.add_argument("--scan-noise-std", type=float, default=0.01)
    parser.add_argument("--sigma0", type=float, default=0.18)
    parser.add_argument("--cma-seed", type=int, default=260809)
    parser.add_argument("--parallel", type=int, default=6)
    parser.add_argument("--domain-start", type=int, default=201)
    parser.add_argument("--post-obstacle-distance", type=float, default=3.0)
    parser.add_argument("--maximum-duration", type=float, default=8.0)
    parser.add_argument("--dds-transport", default="UDPv4")
    parser.add_argument("--infrastructure-retries", type=int, default=5)
    parser.add_argument(
        "--stop-after", choices=("baseline", "generation0", "optimization", "all"),
        default="all")
    args = parser.parse_args()
    if args.parallel < 1:
        parser.error("--parallel must be positive")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
