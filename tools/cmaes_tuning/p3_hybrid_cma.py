#!/usr/bin/env python3
"""Run a bounded hybrid P3-primary/P0-backup CMA on frozen Q2 and Finals maps."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import shutil
import subprocess
import sys
from typing import Any


TOOL_ROOT = Path(__file__).resolve().parent
ROOT = TOOL_ROOT.parents[1]
sys.path.insert(0, str(TOOL_ROOT))

from cmaes_tuning.configuration import load_config  # noqa: E402
from cmaes_tuning.objective import episode_cost, upper_tail_cvar  # noqa: E402
from cmaes_tuning.parameter_space import ParameterSpace  # noqa: E402
from cmaes_tuning.scenario_generator import load_waypoints, track_length  # noqa: E402
from cmaes_tuning.schemas import atomic_write_json  # noqa: E402
from medium_lockstep_cma import _load_pycma  # noqa: E402
from p3_production_cma_smoke import (  # noqa: E402
    EXPECTED_RULE_GATE_SHA256,
    EXPECTED_WHITELIST_SHA256,
    PARAMETER_SPACE,
    RULE_GATE,
    WHITELIST,
)


TASK_ROOT = ROOT / "runs/cmaes_tuning/p3_hybrid_p0_cma_v1"
SCENARIO_CLASSES = ("Q2_COMPETITION_STYLE", "FINALS_COMPETITION_STYLE")
EXPECTED_COUNTS = {"Q2_COMPETITION_STYLE": 2, "FINALS_COMPETITION_STYLE": 3}
HARD_FAILURE_KEYS = (
    "collision", "off_track", "nonfinite", "unsafe_path_published",
    "planner_failure", "incomplete_scenario",
)
HYBRID_WEIGHTS = {
    "speed_loss": 1.0,
    "p0_fallback_duration": 2.0,
    "p0_fallback_interval": 0.25,
    "safe_stop_duration": 6.0,
    "safe_stop_interval": 1.0,
    "p3_ownership_reward": 0.5,
    "p3_completion_reward": 0.5,
    "hard_failure_multiplier": 1000.0,
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def preflight() -> dict[str, Any]:
    if sha256(WHITELIST) != EXPECTED_WHITELIST_SHA256:
        raise RuntimeError("frozen six-dimensional P3 whitelist changed")
    if sha256(RULE_GATE) != EXPECTED_RULE_GATE_SHA256:
        raise RuntimeError("competition obstacle rule gate changed")
    whitelist = json.loads(WHITELIST.read_text(encoding="utf-8"))
    if whitelist["status"] != "FROZEN_PRE_SMOKE" or whitelist["parameter_count"] != 6:
        raise RuntimeError("invalid six-dimensional whitelist contract")
    if sha256(PARAMETER_SPACE) != whitelist["production_parameter_space_sha256"]:
        raise RuntimeError("production P3 parameter space differs from frozen whitelist")

    gate = json.loads(RULE_GATE.read_text(encoding="utf-8"))
    configurations = {item["configuration_class"]: item for item in gate["configurations"]}
    injections = {item["configuration_class"]: item for item in gate["injection_freeze"]}
    selected: dict[str, dict[str, Any]] = {}
    for name in SCENARIO_CLASSES:
        configuration = configurations[name]
        injection = injections[name]
        expected_count = EXPECTED_COUNTS[name]
        if not configuration["all_rules_pass"] or not configuration["eligible_for_injection"]:
            raise RuntimeError(f"{name} competition-rule gate is not PASS")
        if (configuration["actual_obstacle_count"] != expected_count or
                configuration["expected_obstacle_count"] != expected_count):
            raise RuntimeError(f"{name} obstacle count mismatch")
        if not injection["rule_check_passed_before_bake"]:
            raise RuntimeError(f"{name} map was not frozen after rule-check PASS")
        if configuration["ordered_obstacle_digest"] != injection["ordered_obstacle_digest"]:
            raise RuntimeError(f"{name} obstacle digest differs between gate and baked map")
        for field in ("yaml", "image"):
            path = Path(injection["map"][field])
            if sha256(path) != injection["map"][f"{field}_sha256"]:
                raise RuntimeError(f"{name} baked map {field} hash mismatch")
        selected[name] = {"configuration": configuration, "injection": injection}

    authority = gate["authority"]
    clean_yaml = ROOT / authority["clean_map"]
    clean_image = clean_yaml.with_suffix(".png")
    waypoints = ROOT / authority["waypoints"]
    for path, expected in (
        (clean_yaml, authority["clean_map_yaml_sha256"]),
        (clean_image, authority["clean_map_image_sha256"]),
        (waypoints, authority["waypoints_sha256"]),
    ):
        if sha256(path) != expected:
            raise RuntimeError(f"authoritative asset changed: {path}")
    space = ParameterSpace.load(PARAMETER_SPACE)
    space.validate_baseline(ROOT / "src/local_planning/config/local_planning.yaml")
    if space.names != [item["name"] for item in whitelist["whitelist"]]:
        raise RuntimeError("parameter order differs from frozen whitelist")
    return {
        "gate": gate,
        "selected": selected,
        "clean_yaml": clean_yaml,
        "clean_image": clean_image,
        "waypoints": waypoints,
        "space": space,
        "whitelist": whitelist,
    }


def write_scenarios(inputs: dict[str, Any], output: Path, config: dict[str, Any]) -> list[Path]:
    by_id = {item["case_id"]: item for item in inputs["gate"]["obstacles"]}
    waypoints = load_waypoints(inputs["waypoints"])
    first = waypoints[0]
    simulator_config = Path(config["paths"]["simulator_config"])
    simulator_source = Path(config["paths"]["simulator_collision_source"])
    results = []
    for name in SCENARIO_CLASSES:
        configuration = inputs["selected"][name]["configuration"]
        injection = inputs["selected"][name]["injection"]
        obstacles = [dict(by_id[identifier]["obstacle"])
                     for identifier in configuration["ordered_obstacle_ids"]]
        scenario = {
            "schema": "cmaes_scenario/1",
            "scenario_id": f"p3_hybrid_{name.lower()}",
            "dataset_split": "hybrid_cma",
            "generator_version": "frozen_competition_rule_gate_reuse_v1",
            "category": name.lower(),
            "lateral_band": "mixed",
            "map_name": "ifac_track",
            "map_resolution_m": 0.025,
            "obstacle": obstacles[0],
            "obstacles": obstacles,
            "competition_configuration_class": name,
            "competition_rule_gate": str(RULE_GATE),
            "competition_rule_gate_sha256": EXPECTED_RULE_GATE_SHA256,
            "ordered_obstacle_digest": configuration["ordered_obstacle_digest"],
            "baked_map_hash": injection["map"]["combined_sha256"],
            "baked_map_image": str(Path(injection["map"]["image"]).resolve()),
            "baked_map_image_sha256": injection["map"]["image_sha256"],
            "baked_map_yaml": str(Path(injection["map"]["yaml"]).resolve()),
            "baked_map_yaml_sha256": injection["map"]["yaml_sha256"],
            "clean_map_hash": "caee95ffd6f75896def6113a3393affe21031fb966507b5a907766ab699b586c",
            "clean_map_image": str(inputs["clean_image"].resolve()),
            "clean_map_image_sha256": sha256(inputs["clean_image"]),
            "clean_map_yaml": str(inputs["clean_yaml"].resolve()),
            "clean_map_yaml_sha256": sha256(inputs["clean_yaml"]),
            "spawn_pose": {
                "s": float(first["s_m"]), "x": float(first["x_m"]),
                "y": float(first["y_m"]), "yaw": float(first["psi_rad"]),
            },
            "vehicle_length_m": 0.56,
            "vehicle_width_m": 0.287,
            "waypoint_file": str(inputs["waypoints"].resolve()),
            "waypoint_sha256": sha256(inputs["waypoints"]),
            "simulator_config": str(simulator_config),
            "simulator_config_sha256": sha256(simulator_config),
            "simulator_collision_source": str(simulator_source),
            "simulator_collision_source_sha256": sha256(simulator_source),
            "simulator_collision_model": {
                "image_vertical_flip": True,
                "lidar_offset_x_m": 0.275,
                "occupied_gray_threshold": 128,
                "physics_timestep_sec": 0.01,
                "pixel_lookup": "floor_half_open_cell",
                "reference_point": "base_link_center",
                "scan_beams": 1080,
                "scan_fov_rad": 4.7,
                "scan_noise_guard_sigma": 0.75,
                "scan_noise_std_m": 0.01,
                "ttc_threshold_sec": 0.005,
                "vehicle_length_m": 0.56,
                "vehicle_width_m": 0.287,
            },
        }
        path = output / "frozen_inputs" / f"{name.lower()}_manifest.json"
        atomic_write_json(path, scenario)
        results.append(path)
    return results


def hybrid_fitness(
    episode_pairs: list[tuple[dict[str, Any], dict[str, Any]]], config: dict[str, Any]
) -> dict[str, Any]:
    """Safety-dominant hybrid score; safely rejected P3 suffixes are diagnostics, not failures."""
    if not episode_pairs or any(not episode.get("valid") for episode, _ in episode_pairs):
        raise ValueError("hybrid fitness requires valid episode records")
    base_costs = [episode_cost(episode["metrics"], config) for episode, _ in episode_pairs]
    values = [float(item["j_performance"]) for item in base_costs]
    beta = float(config["objective"]["beta_cvar"])
    base_quality = sum(values) / len(values) + beta * upper_tail_cvar(
        values, float(config["objective"]["cvar_alpha"]))
    j_max = sum(float(value) for value in config["objective"]["weights"].values())

    elapsed = [max(float(result["step_count"]) * 0.01, 0.01) for _, result in episode_pairs]
    fallback_fractions = [
        min(1.0, float(result["p0_fallback_duration_s"]) / duration)
        for (_, result), duration in zip(episode_pairs, elapsed)
    ]
    safe_stop_fractions = [
        min(1.0, float(result["safe_stop_duration_s"]) / duration)
        for (_, result), duration in zip(episode_pairs, elapsed)
    ]
    fallback_interval_norm = [
        min(1.0, float(result["p0_fallback_interval_count"]) /
            max(1, int(result["obstacle_count"])))
        for _, result in episode_pairs
    ]
    safe_stop_interval_norm = [
        min(1.0, float(result["safe_stop_interval_count"]) /
            max(1, int(result["obstacle_count"])))
        for _, result in episode_pairs
    ]
    ownership = [float(result["p3_ownership_fraction"]) for _, result in episode_pairs]
    completions = [
        min(1.0, float(result["p3_completion_count"]) /
            max(1, int(result["obstacle_count"])))
        for _, result in episode_pairs
    ]
    speed_loss = [
        max(0.0, min(1.0, float(episode["metrics"]["speed_loss_during_avoidance_fraction"])))
        for episode, _ in episode_pairs
    ]

    hard_rows = []
    for _, result in episode_pairs:
        row = {
            "collision": bool(result["collision"]),
            "off_track": bool(result["off_track"]),
            "nonfinite": bool(result["nonfinite"]),
            "unsafe_path_published": bool(result["unsafe_path_published"]),
            "planner_failure": bool(result["planner_failure"]),
            "incomplete_scenario": not bool(result["completed"]),
        }
        hard_rows.append(row)
    hard_failure_count = sum(any(row.values()) for row in hard_rows)
    mean = lambda samples: sum(samples) / len(samples)
    components = {
        "base_performance": base_quality,
        "speed_loss": HYBRID_WEIGHTS["speed_loss"] * j_max * mean(speed_loss),
        "p0_fallback_duration": (
            HYBRID_WEIGHTS["p0_fallback_duration"] * j_max * mean(fallback_fractions)),
        "p0_fallback_interval": (
            HYBRID_WEIGHTS["p0_fallback_interval"] * j_max * mean(fallback_interval_norm)),
        "safe_stop_duration": (
            HYBRID_WEIGHTS["safe_stop_duration"] * j_max * mean(safe_stop_fractions)),
        "safe_stop_interval": (
            HYBRID_WEIGHTS["safe_stop_interval"] * j_max * mean(safe_stop_interval_norm)),
        "p3_ownership_reward": -(
            HYBRID_WEIGHTS["p3_ownership_reward"] * j_max * mean(ownership)),
        "p3_completion_reward": -(
            HYBRID_WEIGHTS["p3_completion_reward"] * j_max * mean(completions)),
        "hard_failure": HYBRID_WEIGHTS["hard_failure_multiplier"] * hard_failure_count,
    }
    return {
        "fitness": sum(components.values()),
        "feasible": hard_failure_count == 0,
        "hard_failure_count": hard_failure_count,
        "hard_failure_keys": list(HARD_FAILURE_KEYS),
        "hard_failure_rows": hard_rows,
        "components": components,
        "scenario_base_costs": base_costs,
        "mean_p0_fallback_fraction": mean(fallback_fractions),
        "mean_safe_stop_fraction": mean(safe_stop_fractions),
        "mean_p3_ownership_fraction": mean(ownership),
        "mean_p3_completion_fraction": mean(completions),
        "mean_speed_loss_fraction": mean(speed_loss),
    }


def cache_key(candidate_sha: str, scenario: Path, config: Path, post_distance: float,
              maximum_duration: float) -> str:
    payload = {
        "candidate_sha256": candidate_sha,
        "scenario_sha256": sha256(scenario),
        "config_sha256": sha256(config),
        "lockstep_sha256": sha256(TOOL_ROOT / "lockstep_episode.py"),
        "simulator_seed": 12345,
        "scan_noise_std": 0.01,
        "post_obstacle_distance_m": post_distance,
        "maximum_duration_s": maximum_duration,
        "p3_mode": "TEST_ACTIVE",
    }
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def run_episode_cached(
    *, candidate: Path, candidate_sha: str, scenario: Path, config_path: Path,
    cache_root: Path, domain: int, maximum_duration: float,
) -> tuple[dict[str, Any], dict[str, Any], bool, str]:
    manifest = json.loads(scenario.read_text(encoding="utf-8"))
    reference = load_waypoints(Path(manifest["waypoint_file"]))
    length = track_length(reference)
    spawn_s = float(manifest["spawn_pose"]["s"])
    furthest = max((float(item["s"]) - spawn_s) % length for item in manifest["obstacles"])
    post_distance = max(3.0, length * 0.97 - furthest)
    key = cache_key(candidate_sha, scenario, config_path, post_distance, maximum_duration)
    episode_root = cache_root / key
    lockstep_path = episode_root / "lockstep_result.json"
    episode_path = episode_root / "episode_result.json"
    if lockstep_path.is_file() and episode_path.is_file():
        lockstep = json.loads(lockstep_path.read_text(encoding="utf-8"))
        episode = json.loads(episode_path.read_text(encoding="utf-8"))
        if lockstep.get("valid") and episode.get("valid"):
            return lockstep, episode, True, key

    completed = None
    # 4 attempts: the first-tick discovery race (a late inter-node subscription match drops the
    # volatile tick-1 messages and the chain stalls) killed two 16x16 runs at 2 attempts —
    # ~7% single-attempt flake makes back-to-back failures likely across 500+ episodes.
    for attempt in range(4):
        if attempt > 0 and episode_root.exists():
            shutil.rmtree(episode_root)
        environment = dict(os.environ)
        environment.update({
            "ROS_DOMAIN_ID": str(domain),
            "FASTDDS_BUILTIN_TRANSPORTS": "UDPv4",
            "MPLCONFIGDIR": "/tmp/cmaes_matplotlib",
        })
        command = [
            sys.executable, str(TOOL_ROOT / "lockstep_episode.py"),
            "--workspace", str(ROOT), "--config", str(config_path),
            "--candidate", str(candidate), "--scenario", str(scenario),
            "--output", str(episode_root), "--simulator-seed", "12345",
            "--scan-noise-std", "0.01", "--post-obstacle-distance", str(post_distance),
            "--maximum-duration", str(maximum_duration), "--dds-transport", "UDPv4",
            "--p3-mode", "TEST_ACTIVE", "--controlled-stop-early-termination",
        ]
        completed = subprocess.run(
            command, cwd=ROOT, env=environment, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False,
        )
        episode_root.mkdir(parents=True, exist_ok=True)
        (episode_root / f"hybrid_cma_stdout_attempt_{attempt + 1}.log").write_text(
            completed.stdout, encoding="utf-8")
        if completed.returncode == 0 and lockstep_path.is_file() and episode_path.is_file():
            break
    if (completed is None or completed.returncode != 0 or
            not lockstep_path.is_file() or not episode_path.is_file()):
        return_code = completed.returncode if completed is not None else None
        raise RuntimeError(
            f"hybrid episode infrastructure failure after retry "
            f"class={manifest['competition_configuration_class']} returncode={return_code}")
    lockstep = json.loads(lockstep_path.read_text(encoding="utf-8"))
    episode = json.loads(episode_path.read_text(encoding="utf-8"))
    if not lockstep.get("valid") or not episode.get("valid"):
        raise RuntimeError("hybrid episode produced an invalid evaluator result")
    return lockstep, episode, False, key


def run_candidate(
    *, phase: str, generation: int, index: int, normalized: list[float], domain: int,
    output: Path, scenarios: list[Path], config_path: Path, config: dict[str, Any],
    space: ParameterSpace, maximum_duration: float,
) -> dict[str, Any]:
    candidate_id = f"generation_{generation:03d}_candidate_{index:03d}"
    root = output / phase / "generations" / f"generation_{generation:03d}" / candidate_id
    candidate_yaml = root / "candidate.yaml"
    metadata = space.write_candidate_yaml(
        ROOT / "src/local_planning/config/local_planning.yaml", candidate_yaml, normalized)
    pairs = []
    scenario_results = []
    cache_hits = 0
    for scenario in scenarios:
        lockstep, episode, cache_hit, key = run_episode_cached(
            candidate=candidate_yaml, candidate_sha=metadata["candidate_sha256"],
            scenario=scenario, config_path=config_path, cache_root=output / "cache",
            domain=domain, maximum_duration=maximum_duration)
        cache_hits += int(cache_hit)
        lockstep["obstacle_count"] = len(
            json.loads(scenario.read_text(encoding="utf-8"))["obstacles"])
        pairs.append((episode, lockstep))
        scenario_results.append({
            "scenario_class": json.loads(scenario.read_text(encoding="utf-8"))[
                "competition_configuration_class"],
            "scenario_manifest_sha256": sha256(scenario),
            "cache_key": key,
            "cache_hit": cache_hit,
            "lockstep": lockstep,
            "metrics": episode["metrics"],
        })
    score = hybrid_fitness(pairs, config)
    result = {
        "schema": "p3_hybrid_cma_candidate/1",
        "candidate_id": candidate_id,
        "phase": phase,
        "generation": generation,
        "index": index,
        "domain_id": domain,
        "normalized": normalized,
        "physical": metadata["physical"],
        "candidate_sha256": metadata["candidate_sha256"],
        "fitness": float(score["fitness"]),
        "feasible": bool(score["feasible"]),
        "score": score,
        "scenario_results": scenario_results,
        "cache_hits": cache_hits,
        "pipeline": [
            "CMA_PARAMETER_DECODE", "PRODUCTION_TEST_ACTIVE_P3_M1",
            "P0_BACKUP_ONLY", "SAFE_STOP", "HYBRID_SCORE",
        ],
    }
    atomic_write_json(root / "candidate_result.json", result)
    return result


def run_phase(
    *, name: str, population_size: int, generations: int, seed: int, sigma: float,
    initial_mean: list[float], inject: list[float] | None, output: Path,
    scenarios: list[Path], config_path: Path, config: dict[str, Any],
    space: ParameterSpace, cma: Any, max_workers: int, domain_start: int,
    maximum_duration: float,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    strategy = cma.CMAEvolutionStrategy(
        initial_mean, sigma,
        {"bounds": [0.0, 1.0], "popsize": population_size, "seed": seed, "verbose": -9},
    )
    domains: queue.Queue[int] = queue.Queue()
    for offset in range(max_workers):
        domains.put(domain_start + offset)
    generation_rows = []
    candidates = []
    for generation in range(generations):
        population = [list(map(float, item)) for item in strategy.ask(population_size)]
        if generation == 0 and inject is not None:
            population[0] = list(inject)
        results: list[dict[str, Any] | None] = [None] * population_size

        def run_with_domain(index: int, normalized: list[float]) -> dict[str, Any]:
            domain = domains.get()
            try:
                return run_candidate(
                    phase=name, generation=generation, index=index,
                    normalized=normalized, domain=domain, output=output, scenarios=scenarios,
                    config_path=config_path, config=config, space=space,
                    maximum_duration=maximum_duration)
            finally:
                domains.put(domain)

        with ThreadPoolExecutor(max_workers=max_workers) as pool:
            futures = {}
            for index, normalized in enumerate(population):
                future = pool.submit(run_with_domain, index, normalized)
                futures[future] = index
            for future in as_completed(futures):
                index = futures[future]
                results[index] = future.result()
        resolved = [item for item in results if item is not None]
        if len(resolved) != population_size:
            raise RuntimeError(f"{name} generation {generation} is incomplete")
        scores = [float(item["fitness"]) for item in resolved]
        strategy.tell(population, scores)
        candidates.extend(resolved)
        row = {
            "generation": generation,
            "population": population_size,
            "scores": scores,
            "candidate_results": resolved,
            "mean_after": [float(value) for value in strategy.mean],
            "sigma_after": float(strategy.sigma),
            "cma_update_completed": True,
        }
        generation_rows.append(row)
        atomic_write_json(
            output / name / "generations" / f"generation_{generation:03d}" /
            "generation_result.json", row)
    best = min(candidates, key=lambda item: float(item["fitness"]))
    phase_summary = {
        "schema": "p3_hybrid_cma_phase/1",
        "phase": name,
        "population": population_size,
        "generations": generations,
        "candidate_count": len(candidates),
        "episode_count": len(candidates) * len(scenarios),
        "cma_updates": len(generation_rows),
        "feasible_candidate_count": sum(bool(item["feasible"]) for item in candidates),
        "cache_hits": sum(int(item["cache_hits"]) for item in candidates),
        "best_candidate": best,
        "status": "PASS_PIPELINE",
    }
    atomic_write_json(output / name / "phase_summary.json", phase_summary)
    return candidates, phase_summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=TASK_ROOT / ".evaluation")
    parser.add_argument("--config", type=Path, default=TOOL_ROOT / "config/tuning_config.yaml")
    parser.add_argument("--max-workers", type=int, default=3)
    parser.add_argument("--domain-start", type=int, default=221)
    parser.add_argument("--maximum-duration", type=float, default=20.0)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if not 1 <= args.max_workers <= 3:
        parser.error("--max-workers must be within [1, 3]")
    if args.domain_start + args.max_workers - 1 > 230:
        parser.error("ROS domain pool exceeds 230")
    inputs = preflight()
    preflight_record = {
        "schema": "p3_hybrid_cma_preflight/1",
        "status": "PASS",
        "architecture": ["P3_M1_PRIMARY", "P0_BACKUP_ONLY", "SAFE_STOP"],
        "parameter_count": 6,
        "parameters": inputs["space"].names,
        "scenario_classes": list(SCENARIO_CLASSES),
        "scenario_obstacle_counts": EXPECTED_COUNTS,
        "competition_rule_gate_sha256": EXPECTED_RULE_GATE_SHA256,
        "whitelist_sha256": EXPECTED_WHITELIST_SHA256,
        "maximum_workers": args.max_workers,
        "cache_reuse": True,
        "hard_failure_keys": list(HARD_FAILURE_KEYS),
        "hybrid_weights": HYBRID_WEIGHTS,
        "trajectory_executed": False,
    }
    if not args.execute:
        print(json.dumps(preflight_record, sort_keys=True))
        return 0

    output = args.output.resolve()
    if output.exists() and not args.resume:
        raise FileExistsError(f"output already exists; pass --resume to reuse cache: {output}")
    output.mkdir(parents=True, exist_ok=args.resume)
    atomic_write_json(output / "preflight.json", preflight_record)
    config_path = args.config.resolve()
    config = load_config(config_path)
    scenarios = write_scenarios(inputs, output, config)
    space: ParameterSpace = inputs["space"]
    cma = _load_pycma(output)

    smoke_candidates, smoke = run_phase(
        name="smoke", population_size=4, generations=2, seed=260812, sigma=0.12,
        initial_mean=space.baseline_z(), inject=None, output=output, scenarios=scenarios,
        config_path=config_path, config=config, space=space, cma=cma,
        max_workers=args.max_workers, domain_start=args.domain_start,
        maximum_duration=args.maximum_duration)
    smoke_best = min(smoke_candidates, key=lambda item: float(item["fitness"]))

    short_candidates, short = run_phase(
        name="short", population_size=6, generations=3, seed=260813, sigma=0.10,
        initial_mean=list(smoke_best["normalized"]), inject=list(smoke_best["normalized"]),
        output=output, scenarios=scenarios, config_path=config_path, config=config,
        space=space, cma=cma, max_workers=args.max_workers,
        domain_start=args.domain_start, maximum_duration=args.maximum_duration)
    all_candidates = smoke_candidates + short_candidates
    best = min(all_candidates, key=lambda item: float(item["fitness"]))
    feasible = [item for item in all_candidates if item["feasible"]]
    best_feasible = min(feasible, key=lambda item: float(item["fitness"])) if feasible else None
    summary = {
        "schema": "p3_hybrid_p0_cma/1",
        "task": "RUN_HYBRID_P3_P0_CMA",
        "created_at_utc": utc_now(),
        "status": "PASS_PIPELINE" if short["cma_updates"] == 3 else "FAIL_PIPELINE",
        "architecture": ["P3_M1_PRIMARY", "P0_BACKUP_ONLY", "SAFE_STOP"],
        "feasibility": (
            "ALL_Q2_AND_FINALS_EPISODES_COMPLETE_WITHOUT_COLLISION_OFFTRACK_"
            "NONFINITE_UNSAFE_PATH_OR_PLANNER_FAILURE"),
        "p3_only_completion_required": False,
        "parameter_count": 6,
        "parameters": space.names,
        "maximum_workers": args.max_workers,
        "scenario_classes": list(SCENARIO_CLASSES),
        "smoke": smoke,
        "short": short,
        "total_candidate_count": len(all_candidates),
        "total_episode_requests": len(all_candidates) * len(scenarios),
        "total_cache_hits": sum(int(item["cache_hits"]) for item in all_candidates),
        "feasible_candidate_count": len(feasible),
        "best_candidate": best,
        "best_feasible_candidate": best_feasible,
        "pipeline_completed": short["cma_updates"] == 3,
        "main_cma_ready": short["cma_updates"] == 3 and best_feasible is not None,
        "production_planner_modified_by_task": False,
    }
    atomic_write_json(output / "hybrid_cma_summary.json", summary)
    best_source = (
        output / best["phase"] / "generations" /
        f"generation_{best['generation']:03d}" / best["candidate_id"] / "candidate.yaml")
    shutil.copyfile(best_source, output / "best_candidate.yaml")
    if best_feasible is not None:
        feasible_source = (
            output / best_feasible["phase"] / "generations" /
            f"generation_{best_feasible['generation']:03d}" /
            best_feasible["candidate_id"] / "candidate.yaml")
        shutil.copyfile(feasible_source, output / "best_feasible_candidate.yaml")
    print(json.dumps({
        "status": summary["status"],
        "feasible_candidate_count": len(feasible),
        "best_fitness": best["fitness"],
        "best_feasible_fitness": (
            best_feasible["fitness"] if best_feasible is not None else None),
        "output": str(output),
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
