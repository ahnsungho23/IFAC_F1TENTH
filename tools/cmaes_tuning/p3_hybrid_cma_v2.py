#!/usr/bin/env python3
"""Hybrid P3-primary/P0-backup CMA, v2.

Differences from p3_hybrid_cma.py (v1):
- v2 parameter-space contract: outside_line_transition_scale lower bound 0.35 (v1 allowed
  0.05, a curvature-infeasible dead zone that produced zero feasible candidates in 64
  evaluations) and baseline 0.5, matching the updated production YAML.
- The production baseline now carries the measured tracking-error LUT, the gap-driven
  avoidance speed cap with floor 2.0 m/s, and the footprint-aware target gate; candidates
  inherit those through write_candidate_yaml.
- Adds an explicit baseline evaluation stage before CMA so the run records whether the
  production configuration itself completes both competition scenario classes.
- Larger main phase (population 8 x 8 generations) after the pipeline smoke.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
from typing import Any

TOOL_ROOT = Path(__file__).resolve().parent
ROOT = TOOL_ROOT.parents[1]
sys.path.insert(0, str(TOOL_ROOT))

from cmaes_tuning.configuration import load_config  # noqa: E402
from cmaes_tuning.parameter_space import ParameterSpace  # noqa: E402
from cmaes_tuning.schemas import atomic_write_json  # noqa: E402
from medium_lockstep_cma import _load_pycma  # noqa: E402
import p3_hybrid_cma as v1  # noqa: E402


TASK_ROOT = ROOT / "runs/cmaes_tuning/p3_hybrid_p0_cma_v2"
PARAMETER_SPACE_V2 = TOOL_ROOT / "config/p3_production_parameter_space_v2.yaml"
WHITELIST_V2 = TASK_ROOT / "parameter_whitelist.json"
# v2 scenarios: the frozen v1 gate's obstacle #1 (s=9.114) sits in the vehicle's full-lock
# curvature shadow and is impassable for any candidate, which structurally forced the v1
# CMA's 0/26 completions. generate_cma_scenarios_v2.py regenerates rule-compliant AND
# vehicle-feasible placements and bakes the maps this runner consumes.
SCENARIO_ROOT = TASK_ROOT / "scenarios"
RULE_GATE_V2 = SCENARIO_ROOT / "rule_gate_v2.json"
SCENARIO_CLASSES = ("Q2_COMPETITION_STYLE", "FINALS_COMPETITION_STYLE")
EXPECTED_COUNTS = {"Q2_COMPETITION_STYLE": 2, "FINALS_COMPETITION_STYLE": 3}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def preflight() -> dict[str, Any]:
    whitelist = json.loads(WHITELIST_V2.read_text(encoding="utf-8"))
    if whitelist["status"] != "FROZEN_PRE_SMOKE" or whitelist["parameter_count"] != 6:
        raise RuntimeError("invalid v2 whitelist contract")
    if sha256(PARAMETER_SPACE_V2) != whitelist["production_parameter_space_sha256"]:
        raise RuntimeError("v2 parameter space differs from the frozen v2 whitelist")

    gate = json.loads(RULE_GATE_V2.read_text(encoding="utf-8"))
    gate_sha = sha256(RULE_GATE_V2)
    configurations = {item["configuration_class"]: item for item in gate["configurations"]}
    injections = {item["configuration_class"]: item for item in gate["injection_freeze"]}
    scenario_paths: list[Path] = []
    for name in SCENARIO_CLASSES:
        configuration = configurations[name]
        injection = injections[name]
        expected_count = EXPECTED_COUNTS[name]
        if not configuration["all_rules_pass"] or not configuration["eligible_for_injection"]:
            raise RuntimeError(f"{name} competition-rule gate is not PASS")
        if (configuration["actual_obstacle_count"] != expected_count or
                configuration["expected_obstacle_count"] != expected_count):
            raise RuntimeError(f"{name} obstacle count mismatch")
        if configuration["ordered_obstacle_digest"] != injection["ordered_obstacle_digest"]:
            raise RuntimeError(f"{name} obstacle digest differs between gate and baked map")
        for field in ("yaml", "image"):
            path = Path(injection["map"][field])
            if sha256(path) != injection["map"][f"{field}_sha256"]:
                raise RuntimeError(f"{name} baked map {field} hash mismatch")
        manifest_path = SCENARIO_ROOT / f"{name.lower()}_manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest["competition_rule_gate_sha256"] != gate_sha:
            raise RuntimeError(f"{name} manifest references a different rule gate")
        if manifest["ordered_obstacle_digest"] != configuration["ordered_obstacle_digest"]:
            raise RuntimeError(f"{name} manifest obstacle digest mismatch")
        if len(manifest["obstacles"]) != expected_count:
            raise RuntimeError(f"{name} manifest obstacle count mismatch")
        scenario_paths.append(manifest_path)
    for item in gate["obstacles"]:
        rules = item["rules"]
        if not (rules["dimension_pass"] and rules["start_distance_pass"] and
                rules["free_side_gap_pass"] and rules["clean_map_region_free"]):
            raise RuntimeError(f"rule violation recorded for {item['case_id']}")

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

    space = ParameterSpace.load(PARAMETER_SPACE_V2)
    space.validate_baseline(ROOT / "src/local_planning/config/local_planning.yaml")
    if space.names != [item["name"] for item in whitelist["whitelist"]]:
        raise RuntimeError("v2 parameter order differs from the frozen v2 whitelist")
    for definition, frozen in zip(space.definitions, whitelist["whitelist"]):
        if (abs(definition.baseline - float(frozen["baseline"])) > 1.0e-12 or
                abs(definition.lower - float(frozen["lower"])) > 1.0e-12 or
                abs(definition.upper - float(frozen["upper"])) > 1.0e-12):
            raise RuntimeError(f"v2 bounds differ from the frozen v2 whitelist: {definition.name}")
    return {
        "gate": gate,
        "gate_sha256": gate_sha,
        "scenario_paths": scenario_paths,
        "clean_yaml": clean_yaml,
        "clean_image": clean_image,
        "waypoints": waypoints,
        "space": space,
        "whitelist": whitelist,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=TASK_ROOT / ".evaluation")
    parser.add_argument("--config", type=Path, default=TOOL_ROOT / "config/tuning_config.yaml")
    parser.add_argument("--max-workers", type=int, default=3)
    parser.add_argument("--domain-start", type=int, default=221)
    parser.add_argument("--maximum-duration", type=float, default=20.0)
    parser.add_argument("--baseline-only", action="store_true",
                        help="evaluate only the production baseline on both scenarios")
    parser.add_argument("--main-population", type=int, default=8)
    parser.add_argument("--main-generations", type=int, default=8)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if not 1 <= args.max_workers <= 3:
        parser.error("--max-workers must be within [1, 3]")
    if args.domain_start + args.max_workers - 1 > 230:
        parser.error("ROS domain pool exceeds 230")

    inputs = preflight()
    space: ParameterSpace = inputs["space"]
    preflight_record = {
        "schema": "p3_hybrid_cma_v2_preflight/1",
        "status": "PASS",
        "architecture": ["P3_M1_PRIMARY", "P0_BACKUP_ONLY", "SAFE_STOP"],
        "parameter_count": 6,
        "parameters": space.names,
        "parameter_space_sha256": sha256(PARAMETER_SPACE_V2),
        "whitelist_sha256": sha256(WHITELIST_V2),
        "scenario_classes": list(SCENARIO_CLASSES),
        "scenario_obstacle_counts": EXPECTED_COUNTS,
        "competition_rule_gate": str(RULE_GATE_V2),
        "competition_rule_gate_sha256": inputs["gate_sha256"],
        "baseline_yaml_sha256": sha256(ROOT / "src/local_planning/config/local_planning.yaml"),
        "maximum_workers": args.max_workers,
        "hybrid_weights": v1.HYBRID_WEIGHTS,
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
    frozen_inputs = output / "frozen_inputs"
    frozen_inputs.mkdir(parents=True, exist_ok=True)
    scenarios = []
    for source in inputs["scenario_paths"]:
        destination = frozen_inputs / source.name
        shutil.copyfile(source, destination)
        scenarios.append(destination)
    cma = _load_pycma(output)

    baseline = v1.run_candidate(
        phase="baseline", generation=0, index=0, normalized=space.baseline_z(),
        domain=args.domain_start, output=output, scenarios=scenarios,
        config_path=config_path, config=config, space=space,
        maximum_duration=args.maximum_duration)
    baseline_row = {
        "feasible": baseline["feasible"],
        "fitness": baseline["fitness"],
        "hard_failure_count": baseline["score"]["hard_failure_count"],
        "hard_failure_rows": baseline["score"]["hard_failure_rows"],
        "per_scenario_completed": [
            bool(item["lockstep"]["completed"]) for item in baseline["scenario_results"]],
    }
    atomic_write_json(output / "baseline_result.json", baseline)
    print(json.dumps({"baseline": baseline_row}, sort_keys=True), flush=True)
    if args.baseline_only:
        return 0

    smoke_candidates, smoke = v1.run_phase(
        name="smoke", population_size=4, generations=2, seed=260814, sigma=0.12,
        initial_mean=space.baseline_z(), inject=space.baseline_z(), output=output,
        scenarios=scenarios, config_path=config_path, config=config, space=space, cma=cma,
        max_workers=args.max_workers, domain_start=args.domain_start,
        maximum_duration=args.maximum_duration)
    pool = smoke_candidates + [baseline]
    smoke_best = min(pool, key=lambda item: float(item["fitness"]))
    print(json.dumps({"smoke_best_fitness": smoke_best["fitness"],
                      "smoke_best_feasible": smoke_best["feasible"]}, sort_keys=True), flush=True)

    main_candidates, main_phase = v1.run_phase(
        name="main", population_size=args.main_population,
        generations=args.main_generations, seed=260815, sigma=0.15,
        initial_mean=list(smoke_best["normalized"]), inject=list(smoke_best["normalized"]),
        output=output, scenarios=scenarios, config_path=config_path, config=config,
        space=space, cma=cma, max_workers=args.max_workers,
        domain_start=args.domain_start, maximum_duration=args.maximum_duration)

    all_candidates = [baseline] + smoke_candidates + main_candidates
    best = min(all_candidates, key=lambda item: float(item["fitness"]))
    feasible = [item for item in all_candidates if item["feasible"]]
    best_feasible = min(feasible, key=lambda item: float(item["fitness"])) if feasible else None
    summary = {
        "schema": "p3_hybrid_p0_cma/2",
        "task": "RUN_HYBRID_P3_P0_CMA_V2",
        "created_at_utc": v1.utc_now(),
        "status": "PASS_PIPELINE" if main_phase["cma_updates"] == args.main_generations
                  else "FAIL_PIPELINE",
        "architecture": ["P3_M1_PRIMARY", "P0_BACKUP_ONLY", "SAFE_STOP"],
        "parameter_count": 6,
        "parameters": space.names,
        "scenario_classes": list(SCENARIO_CLASSES),
        "baseline": baseline_row,
        "smoke": smoke,
        "main": main_phase,
        "phase_plan": {
            "baseline": {"population": 1, "generations": 1},
            "smoke": {"population": 4, "generations": 2},
            "main": {"population": args.main_population,
                     "generations": args.main_generations},
        },
        "total_candidate_count": len(all_candidates),
        "total_episode_requests": len(all_candidates) * len(scenarios),
        "total_cache_hits": sum(int(item["cache_hits"]) for item in all_candidates),
        "feasible_candidate_count": len(feasible),
        "best_candidate": best,
        "best_feasible_candidate": best_feasible,
        "production_planner_modified_by_task": False,
    }
    atomic_write_json(output / "hybrid_cma_v2_summary.json", summary)

    def candidate_yaml_path(item: dict[str, Any]) -> Path:
        return (output / item["phase"] / "generations" /
                f"generation_{item['generation']:03d}" / item["candidate_id"] /
                "candidate.yaml")

    shutil.copyfile(candidate_yaml_path(best), output / "best_candidate.yaml")
    if best_feasible is not None:
        shutil.copyfile(
            candidate_yaml_path(best_feasible), output / "best_feasible_candidate.yaml")
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
