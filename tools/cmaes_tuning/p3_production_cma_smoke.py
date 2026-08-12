#!/usr/bin/env python3
"""Bounded pop=4, generations=2 CMA smoke against production TEST_ACTIVE P3.

The adapter reuses the existing lockstep episode, objective, parameter-space,
and pycma machinery.  It adds no planner algorithm and accepts only the frozen
competition-rule-checked Q2 map and physical obstacles.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
from typing import Any


TOOL_ROOT = Path(__file__).resolve().parent
ROOT = TOOL_ROOT.parents[1]
sys.path.insert(0, str(TOOL_ROOT))

from cmaes_tuning.objective import candidate_fitness  # noqa: E402
from cmaes_tuning.parameter_space import ParameterSpace  # noqa: E402
from cmaes_tuning.scenario_generator import load_waypoints  # noqa: E402
from cmaes_tuning.schemas import atomic_write_json, sha256_file  # noqa: E402
from medium_lockstep_cma import _load_pycma  # noqa: E402
from cmaes_tuning.configuration import load_config  # noqa: E402


TASK_ROOT = ROOT / "runs/cmaes_tuning/p3_cma_minimal_readiness_and_smoke_v1"
RULE_GATE = (
    ROOT / "runs/cmaes_tuning/ifac_track_p3_m1_lifecycle_shadow_v1/"
    "pre_result/competition_obstacle_rule_check.json"
)
PARAMETER_SPACE = TOOL_ROOT / "config/p3_production_parameter_space.yaml"
WHITELIST = TASK_ROOT / "parameter_whitelist.json"
EXPECTED_WHITELIST_SHA256 = "74049f7ce5dcf95638172d7da5f67f9c49f67ae0debd94fb69e6ba4a60aeb141"
EXPECTED_RULE_GATE_SHA256 = "7c16582724dcfa6b96bb4ce07c051ed8d7eb320f1a602792e184965e235637d7"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def q2_configuration(gate: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    configurations = {
        item["configuration_class"]: item for item in gate["configurations"]
    }
    injections = {
        item["configuration_class"]: item for item in gate["injection_freeze"]
    }
    configuration = configurations["Q2_COMPETITION_STYLE"]
    injection = injections["Q2_COMPETITION_STYLE"]
    if not configuration["all_rules_pass"] or not configuration["eligible_for_injection"]:
        raise RuntimeError("Q2 competition rule gate is not PASS")
    if configuration["actual_obstacle_count"] != 2 or configuration["expected_obstacle_count"] != 2:
        raise RuntimeError("Q2 obstacle count is not exactly two")
    if not injection["rule_check_passed_before_bake"]:
        raise RuntimeError("Q2 map was not baked after a PASS rule check")
    if configuration["ordered_obstacle_digest"] != injection["ordered_obstacle_digest"]:
        raise RuntimeError("Q2 rule/injection obstacle digest mismatch")
    return configuration, injection


def preflight() -> dict[str, Any]:
    if sha256(WHITELIST) != EXPECTED_WHITELIST_SHA256:
        raise RuntimeError("frozen P3 CMA whitelist changed")
    if sha256(RULE_GATE) != EXPECTED_RULE_GATE_SHA256:
        raise RuntimeError("competition obstacle rule gate changed")
    whitelist = json.loads(WHITELIST.read_text(encoding="utf-8"))
    if whitelist["status"] != "FROZEN_PRE_SMOKE" or whitelist["parameter_count"] != 6:
        raise RuntimeError("invalid whitelist contract")
    if sha256(PARAMETER_SPACE) != whitelist["production_parameter_space_sha256"]:
        raise RuntimeError("production P3 parameter-space hash differs from whitelist")
    gate = json.loads(RULE_GATE.read_text(encoding="utf-8"))
    configuration, injection = q2_configuration(gate)
    map_yaml = Path(injection["map"]["yaml"])
    map_image = Path(injection["map"]["image"])
    if sha256(map_yaml) != injection["map"]["yaml_sha256"]:
        raise RuntimeError("Q2 map YAML hash mismatch")
    if sha256(map_image) != injection["map"]["image_sha256"]:
        raise RuntimeError("Q2 map image hash mismatch")
    authority = gate["authority"]
    clean_yaml = ROOT / authority["clean_map"]
    clean_image = clean_yaml.with_suffix(".png")
    waypoints = ROOT / authority["waypoints"]
    if sha256(clean_yaml) != authority["clean_map_yaml_sha256"]:
        raise RuntimeError("clean map YAML hash mismatch")
    if sha256(clean_image) != authority["clean_map_image_sha256"]:
        raise RuntimeError("clean map image hash mismatch")
    if sha256(waypoints) != authority["waypoints_sha256"]:
        raise RuntimeError("ifac_track waypoint hash mismatch")
    space = ParameterSpace.load(PARAMETER_SPACE)
    space.validate_baseline(ROOT / "src/local_planning/config/local_planning.yaml")
    if space.names != [item["name"] for item in whitelist["whitelist"]]:
        raise RuntimeError("parameter-space order differs from frozen whitelist")
    return {
        "rule_gate": gate,
        "configuration": configuration,
        "injection": injection,
        "clean_yaml": clean_yaml,
        "clean_image": clean_image,
        "waypoints": waypoints,
        "space": space,
    }


def write_scenario(inputs: dict[str, Any], output: Path, config: dict[str, Any]) -> Path:
    gate = inputs["rule_gate"]
    selected_ids = inputs["configuration"]["ordered_obstacle_ids"]
    by_id = {item["case_id"]: item for item in gate["obstacles"]}
    obstacles = [dict(by_id[identifier]["obstacle"]) for identifier in selected_ids]
    waypoints = load_waypoints(inputs["waypoints"])
    first = waypoints[0]
    sim_config = Path(config["paths"]["simulator_config"])
    sim_source = Path(config["paths"]["simulator_collision_source"])
    injection = inputs["injection"]
    scenario = {
        "schema": "cmaes_scenario/1",
        "scenario_id": "p3_cma_smoke_q2_competition_style",
        "dataset_split": "smoke",
        "generator_version": "frozen_competition_rule_gate_reuse_v1",
        "category": "competition_q2",
        "lateral_band": "mixed",
        "map_name": "ifac_track",
        "map_resolution_m": 0.025,
        "obstacle": obstacles[0],
        "obstacles": obstacles,
        "competition_configuration_class": "Q2_COMPETITION_STYLE",
        "competition_rule_gate": str(RULE_GATE),
        "competition_rule_gate_sha256": EXPECTED_RULE_GATE_SHA256,
        "ordered_obstacle_digest": inputs["configuration"]["ordered_obstacle_digest"],
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
        "simulator_config": str(sim_config),
        "simulator_config_sha256": sha256(sim_config),
        "simulator_collision_source": str(sim_source),
        "simulator_collision_source_sha256": sha256(sim_source),
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
    path = output / "frozen_inputs/q2_competition_style_manifest.json"
    atomic_write_json(path, scenario)
    return path


def run_candidate(
    *, index: int, generation: int, normalized: list[float], domain: int,
    output: Path, scenario: Path, config_path: Path, space: ParameterSpace,
) -> dict[str, Any]:
    candidate_id = f"generation_{generation:03d}_candidate_{index:03d}"
    root = output / "generations" / f"generation_{generation:03d}" / candidate_id
    candidate_yaml = root / "candidate.yaml"
    metadata = space.write_candidate_yaml(
        ROOT / "src/local_planning/config/local_planning.yaml", candidate_yaml, normalized)
    episode_root = root / "episode"
    environment = dict(os.environ)
    environment.update({
        "ROS_DOMAIN_ID": str(domain),
        "FASTDDS_BUILTIN_TRANSPORTS": "UDPv4",
        "MPLCONFIGDIR": "/tmp/cmaes_matplotlib",
    })
    command = [
        sys.executable, str(TOOL_ROOT / "lockstep_episode.py"),
        "--workspace", str(ROOT), "--config", str(config_path),
        "--candidate", str(candidate_yaml), "--scenario", str(scenario),
        "--output", str(episode_root), "--simulator-seed", "12345",
        "--scan-noise-std", "0.01", "--post-obstacle-distance", "3.0",
        "--maximum-duration", "8.0", "--dds-transport", "UDPv4",
        "--p3-mode", "TEST_ACTIVE",
    ]
    completed = subprocess.run(
        command, cwd=ROOT, env=environment, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False,
    )
    root.mkdir(parents=True, exist_ok=True)
    (root / "stdout.log").write_text(completed.stdout, encoding="utf-8")
    lockstep_path = episode_root / "lockstep_result.json"
    episode_path = episode_root / "episode_result.json"
    if completed.returncode != 0 or not lockstep_path.is_file() or not episode_path.is_file():
        raise RuntimeError(
            f"{candidate_id} infrastructure failure returncode={completed.returncode}")
    lockstep = json.loads(lockstep_path.read_text(encoding="utf-8"))
    episode = json.loads(episode_path.read_text(encoding="utf-8"))
    if not lockstep.get("valid") or not episode.get("valid"):
        raise RuntimeError(f"{candidate_id} produced an invalid episode")
    fitness = candidate_fitness([episode], load_config(config_path))
    result = {
        "candidate_id": candidate_id,
        "generation": generation,
        "index": index,
        "domain_id": domain,
        "normalized": normalized,
        "physical": metadata["physical"],
        "candidate_sha256": metadata["candidate_sha256"],
        "fitness": float(fitness["fitness"]),
        "hard_failure_count": int(fitness["failure_count"]),
        "hard_failure_keys": fitness["hard_failure_keys"],
        "failure": episode["failure"],
        "lockstep": {
            name: lockstep.get(name) for name in (
                "collision", "off_track", "planner_failure", "invalid_suffix",
                "safe_stop", "p0_fallback", "p3_event_count", "p3_selected_count",
                "p3_continuation_count", "p3_completion_count", "scenario_success",
            )
        },
        "pipeline": ["parameter_decode", "candidate_yaml", "production_simulation", "score"],
    }
    atomic_write_json(root / "candidate_result.json", result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=TASK_ROOT / ".evaluation/smoke")
    parser.add_argument("--config", type=Path, default=TOOL_ROOT / "config/tuning_config.yaml")
    parser.add_argument("--max-workers", type=int, default=4)
    parser.add_argument("--domain-start", type=int, default=221)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if not 1 <= args.max_workers <= 4:
        parser.error("--max-workers must be within [1, 4]")
    if args.domain_start + args.max_workers - 1 > 230:
        parser.error("ROS domain pool exceeds 230")
    inputs = preflight()
    output = args.output.resolve()
    preflight_result = {
        "schema": "production_p3_cma_smoke_preflight/1",
        "status": "PASS",
        "whitelist_sha256": EXPECTED_WHITELIST_SHA256,
        "rule_gate_sha256": EXPECTED_RULE_GATE_SHA256,
        "configuration": "Q2_COMPETITION_STYLE",
        "obstacle_count": 2,
        "population": 4,
        "generations": 2,
        "max_workers": args.max_workers,
        "p3_mode": "TEST_ACTIVE",
        "trajectory_executed": False,
    }
    if not args.execute:
        print(json.dumps(preflight_result, sort_keys=True))
        return 0

    output.mkdir(parents=True, exist_ok=False)
    atomic_write_json(output / "preflight.json", preflight_result)
    config_path = args.config.resolve()
    config = load_config(config_path)
    scenario = write_scenario(inputs, output, config)
    space: ParameterSpace = inputs["space"]
    cma = _load_pycma(output)
    strategy = cma.CMAEvolutionStrategy(
        space.baseline_z(), 0.12,
        {"bounds": [0.0, 1.0], "popsize": 4, "seed": 260812, "verbose": -9},
    )
    domain_pool: queue.Queue[int] = queue.Queue()
    for offset in range(args.max_workers):
        domain_pool.put(args.domain_start + offset)
    generations = []
    total_simulations = 0
    for generation in range(2):
        population = [list(map(float, item)) for item in strategy.ask(4)]
        results: list[dict[str, Any] | None] = [None] * 4
        with ThreadPoolExecutor(max_workers=args.max_workers) as pool:
            futures = {}
            for index, normalized in enumerate(population):
                domain = domain_pool.get()
                future = pool.submit(
                    run_candidate, index=index, generation=generation,
                    normalized=normalized, domain=domain, output=output,
                    scenario=scenario, config_path=config_path, space=space)
                futures[future] = (index, domain)
            for future in as_completed(futures):
                index, domain = futures[future]
                try:
                    results[index] = future.result()
                finally:
                    domain_pool.put(domain)
        resolved = [item for item in results if item is not None]
        if len(resolved) != 4:
            raise RuntimeError("generation did not produce four candidate scores")
        scores = [float(item["fitness"]) for item in resolved]
        strategy.tell(population, scores)
        total_simulations += len(resolved)
        generation_result = {
            "generation": generation,
            "population": 4,
            "candidate_results": resolved,
            "scores": scores,
            "mean_after": [float(value) for value in strategy.mean],
            "sigma_after": float(strategy.sigma),
            "cma_update_completed": True,
        }
        generations.append(generation_result)
        atomic_write_json(
            output / "generations" / f"generation_{generation:03d}" / "generation_result.json",
            generation_result,
        )
    summary = {
        "schema": "production_p3_cma_smoke/1",
        "task": "P3_CMA_MINIMAL_READINESS_AND_SMOKE",
        "status": "PASS",
        "created_at_utc": _utc_now(),
        "population": 4,
        "generations": 2,
        "maximum_workers": args.max_workers,
        "completed_simulations": total_simulations,
        "expected_simulations": 8,
        "scenario_count": 1,
        "scenario_class": "Q2_COMPETITION_STYLE",
        "competition_rule_gate_sha256": EXPECTED_RULE_GATE_SHA256,
        "parameter_count": space.dimension,
        "parameters": space.names,
        "full_pipeline": [
            "CMA_ASK", "PARAMETER_DECODE", "CANDIDATE_YAML",
            "PRODUCTION_P3_TEST_ACTIVE_SIMULATION", "SAFETY_DOMINANT_SCORE", "CMA_TELL",
        ],
        "cma_updates": sum(item["cma_update_completed"] for item in generations),
        "failure_penalties": [
            "collision", "off_track", "invalid_suffix", "safe_stop",
            "p0_fallback", "planner_failure",
        ],
        "generation_best_fitness": [min(item["scores"]) for item in generations],
        "short_cma_ready": total_simulations == 8 and len(generations) == 2,
    }
    atomic_write_json(output / "smoke_summary.json", summary)
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
