#!/usr/bin/env python3
"""Feasibility-first bounded CMA search for the frozen production P3 Q2 case.

This runner deliberately reuses the frozen six-dimensional parameter space,
competition-rule gate, production TEST_ACTIVE episode, and existing objective.
It stops at the first hard-safe completed P3 maneuver or after 64 evaluations.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
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
from cmaes_tuning.schemas import atomic_write_json  # noqa: E402
from medium_lockstep_cma import _load_pycma  # noqa: E402
from p3_production_cma_smoke import (  # noqa: E402
    EXPECTED_RULE_GATE_SHA256,
    EXPECTED_WHITELIST_SHA256,
    PARAMETER_SPACE,
    TASK_ROOT,
    preflight,
    run_candidate,
    write_scenario,
)


SMOKE_ROOT = TASK_ROOT / ".evaluation/smoke_final"
EXPECTED_SMOKE_SUMMARY_SHA256 = (
    "f1d7232c412328bc0b8d85ce90d0e0f1c9de59973c3605beb53f792ea8e7325a"
)
HARD_FAILURE_KEYS = (
    "collision",
    "off_track",
    "invalid_suffix",
    "safe_stop",
    "p0_fallback",
    "planner_failure",
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _smoke_results() -> list[dict[str, Any]]:
    summary = SMOKE_ROOT / "smoke_summary.json"
    if _sha256(summary) != EXPECTED_SMOKE_SUMMARY_SHA256:
        raise RuntimeError("authoritative eight-evaluation smoke summary changed")
    results: list[dict[str, Any]] = []
    for generation_path in sorted(
        (SMOKE_ROOT / "generations").glob("generation_*/generation_result.json")
    ):
        document = json.loads(generation_path.read_text(encoding="utf-8"))
        results.extend(document["candidate_results"])
    if len(results) != 8:
        raise RuntimeError(f"expected eight smoke candidates, found {len(results)}")
    return results


def _invalidation_observation(candidate_id: str) -> dict[str, Any]:
    generation = candidate_id.split("_candidate_")[0]
    log_path = (
        SMOKE_ROOT / "generations" / generation / candidate_id
        / "episode/logs/local_planning.log"
    )
    marker = "P3_LIFECYCLE_INVALIDATION "
    for line in log_path.read_text(encoding="utf-8", errors="replace").splitlines():
        if marker in line:
            return json.loads(line.split(marker, 1)[1])
    raise RuntimeError(f"missing invalidation observation for {candidate_id}")


def analyze_smoke() -> dict[str, Any]:
    results = _smoke_results()
    fitness = [float(item["fitness"]) for item in results]
    observations = [_invalidation_observation(item["candidate_id"]) for item in results]
    reasons = sorted({item["lifecycle_reason"] for item in observations})
    raw_rejections = sorted(
        {item["raw_validator"]["rejection"] for item in observations}
    )
    guarded_rejections = sorted(
        {item["guarded_validator"]["rejection"] for item in observations}
    )
    # The frozen observation schema records exact validator verdict/reason and
    # envelopes, but does not expose a numeric penetration/slack at invalidation.
    numeric_severity_available = all(
        any(
            key in item["raw_validator"]
            for key in ("minimum_safety_slack", "violation_magnitude_m")
        )
        for item in observations
    )
    table = []
    for result, observation in zip(results, observations):
        episode_path = (
            SMOKE_ROOT / "generations"
            / result["candidate_id"].split("_candidate_")[0]
            / result["candidate_id"] / "episode/episode_result.json"
        )
        episode = json.loads(episode_path.read_text(encoding="utf-8"))
        table.append({
            "candidate_id": result["candidate_id"],
            "fitness": float(result["fitness"]),
            "quality_cost": float(result["fitness"]) - 13.0,
            "invalidation_reason": observation["lifecycle_reason"],
            "guarded_validator_rejection": observation["guarded_validator"]["rejection"],
            "raw_validator_rejection": observation["raw_validator"]["rejection"],
            "minimum_obstacle_clearance_m": episode["metrics"][
                "minimum_obstacle_clearance_m"
            ],
            "minimum_wall_clearance_m": episode["metrics"][
                "minimum_wall_clearance_m"
            ],
            "planned_curvature_rate_rms_radpm2": episode["metrics"][
                "planned_curvature_rate_rms_radpm2"
            ],
        })
    unique_fitness = len(set(fitness))
    return {
        "schema": "p3_cma_existing_smoke_ranking_analysis/1",
        "authoritative_smoke_summary_sha256": EXPECTED_SMOKE_SUMMARY_SHA256,
        "candidate_count": 8,
        "fitness_unique_count": unique_fitness,
        "fitness_min": min(fitness),
        "fitness_max": max(fitness),
        "fitness_range": max(fitness) - min(fitness),
        "fitness_flat": unique_fitness == 1,
        "invalid_suffix_count": sum(
            bool(item["lockstep"]["invalid_suffix"]) for item in results
        ),
        "invalidation_reason_unique_count": len(reasons),
        "invalidation_reasons": reasons,
        "guarded_rejections": guarded_rejections,
        "raw_rejections": raw_rejections,
        "numeric_invalidation_severity_available": numeric_severity_available,
        "ranking_decision": (
            "USE_EXISTING_OBJECTIVE_UNCHANGED"
            if unique_fitness > 1
            else "REQUIRE_EXACT_VALIDATOR_CONTINUOUS_VIOLATION_PENALTY"
        ),
        "continuous_penalty_added": False,
        "candidates": table,
    }


def _is_feasible(result: dict[str, Any]) -> bool:
    lockstep = result["lockstep"]
    failure = result["failure"]
    return (
        not any(bool(failure.get(key, False)) for key in HARD_FAILURE_KEYS)
        and bool(lockstep.get("scenario_success"))
        and int(lockstep.get("p3_selected_count") or 0) >= 1
        and int(lockstep.get("p3_completion_count") or 0) >= 1
    )


def _copy_feasible_seed(result: dict[str, Any], output: Path) -> dict[str, Any]:
    source_root = Path(result["artifact_root"])
    seed_root = output / "feasible_seed"
    seed_root.mkdir(parents=True, exist_ok=False)
    shutil.copy2(source_root / "candidate.yaml", seed_root / "candidate.yaml")
    shutil.copy2(source_root / "candidate_result.json", seed_root / "candidate_result.json")
    trajectory = source_root / "episode/trajectory.csv"
    if trajectory.is_file():
        shutil.copy2(trajectory, seed_root / "trajectory.csv")
    episode_result = source_root / "episode/episode_result.json"
    lockstep_result = source_root / "episode/lockstep_result.json"
    shutil.copy2(episode_result, seed_root / "episode_result.json")
    shutil.copy2(lockstep_result, seed_root / "lockstep_result.json")
    return {
        "candidate_id": result["candidate_id"],
        "physical": result["physical"],
        "normalized": result["normalized"],
        "source_artifact_root": str(source_root),
        "seed_root": str(seed_root),
        "candidate_yaml_sha256": _sha256(seed_root / "candidate.yaml"),
        "trajectory_sha256": (
            _sha256(seed_root / "trajectory.csv")
            if (seed_root / "trajectory.csv").is_file() else None
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path,
        default=TASK_ROOT / ".evaluation/feasible_seed_search",
    )
    parser.add_argument(
        "--config", type=Path,
        default=TOOL_ROOT / "config/tuning_config.yaml",
    )
    parser.add_argument("--max-workers", type=int, default=4)
    parser.add_argument("--domain-start", type=int, default=221)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if args.max_workers != 4:
        parser.error("initial --max-workers must be exactly 4")
    if args.domain_start + 3 > 230:
        parser.error("four-worker ROS domain pool exceeds 230")

    inputs = preflight()
    smoke_analysis = analyze_smoke()
    if smoke_analysis["fitness_flat"]:
        raise RuntimeError(
            "existing objective is flat; numeric exact-validator violation severity "
            "is not frozen, so fail closed instead of inventing a penalty"
        )
    preflight_result = {
        "schema": "p3_cma_feasible_seed_search_preflight/1",
        "status": "PASS",
        "population": 8,
        "maximum_generations": 8,
        "maximum_evaluations": 64,
        "initial_workers": 4,
        "timeout_recovery_workers": 3,
        "parameter_count": inputs["space"].dimension,
        "parameter_space_sha256": _sha256(PARAMETER_SPACE),
        "whitelist_sha256": EXPECTED_WHITELIST_SHA256,
        "competition_rule_gate_sha256": EXPECTED_RULE_GATE_SHA256,
        "configuration": "Q2_COMPETITION_STYLE",
        "obstacle_count": 2,
        "ranking_decision": smoke_analysis["ranking_decision"],
        "continuous_penalty_added": False,
        "trajectory_executed": False,
    }
    if not args.execute:
        print(json.dumps(preflight_result, sort_keys=True))
        return 0

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    atomic_write_json(output / "preflight.json", preflight_result)
    atomic_write_json(output / "existing_smoke_ranking_analysis.json", smoke_analysis)
    config_path = args.config.resolve()
    load_config(config_path)
    scenario = write_scenario(inputs, output, load_config(config_path))
    space = inputs["space"]
    best_smoke = min(_smoke_results(), key=lambda item: float(item["fitness"]))
    initial_mean = [float(value) for value in best_smoke["normalized"]]
    cma = _load_pycma(output)
    strategy = cma.CMAEvolutionStrategy(
        initial_mean,
        0.12,
        {"bounds": [0.0, 1.0], "popsize": 8, "seed": 260813, "verbose": -9},
    )

    evaluations = 0
    generation_documents: list[dict[str, Any]] = []
    feasible_result: dict[str, Any] | None = None
    active_workers = 4
    timeout_or_infrastructure_failures = 0
    retry_count = 0
    for generation in range(8):
        population = [list(map(float, item)) for item in strategy.ask(8)]
        results: list[dict[str, Any] | None] = [None] * 8
        pending_indices = list(range(8))
        attempt = 0
        while pending_indices and feasible_result is None:
            chunk = pending_indices[:active_workers]
            pending_indices = pending_indices[active_workers:]
            attempt_root = output / "attempts" / f"attempt_{attempt:03d}"
            failures: list[int] = []
            with ThreadPoolExecutor(max_workers=active_workers) as pool:
                futures = {}
                for slot, index in enumerate(chunk):
                    future = pool.submit(
                        run_candidate,
                        index=index,
                        generation=generation,
                        normalized=population[index],
                        domain=args.domain_start + slot,
                        output=attempt_root,
                        scenario=scenario,
                        config_path=config_path,
                        space=space,
                    )
                    futures[future] = index
                for future in as_completed(futures):
                    index = futures[future]
                    try:
                        result = future.result()
                        candidate_root = (
                            attempt_root / "generations"
                            / f"generation_{generation:03d}"
                            / result["candidate_id"]
                        )
                        result["attempt"] = attempt
                        result["artifact_root"] = str(candidate_root)
                        result["feasible"] = _is_feasible(result)
                        results[index] = result
                        evaluations += 1
                        atomic_write_json(
                            output / "latest_progress.json",
                            {
                                "schema": "p3_cma_feasible_seed_search_progress/1",
                                "completed_evaluations": evaluations,
                                "current_generation": generation,
                                "current_candidate_index": index,
                                "feasible_found": result["feasible"],
                                "active_workers": active_workers,
                            },
                        )
                        if result["feasible"] and feasible_result is None:
                            feasible_result = result
                    except RuntimeError as error:
                        failures.append(index)
                        timeout_or_infrastructure_failures += 1
                        atomic_write_json(
                            output / "attempts" / f"failure_g{generation:03d}_c{index:03d}_a{attempt:03d}.json",
                            {
                                "generation": generation,
                                "candidate_index": index,
                                "attempt": attempt,
                                "error": str(error),
                                "action": "RETRY_WITH_THREE_WORKERS",
                            },
                        )
            if feasible_result is not None:
                break
            if failures:
                # A prior smoke rerun already exposed one timeout.  The first
                # recurrence in this bounded search permanently lowers the pool.
                active_workers = 3
                retry_count += len(failures)
                if retry_count > 8:
                    raise RuntimeError("infrastructure retry budget exceeded")
                pending_indices = failures + pending_indices
            attempt += 1

        resolved = [item for item in results if item is not None]
        generation_document = {
            "generation": generation,
            "population": 8,
            "completed_candidates": len(resolved),
            "candidate_results": resolved,
            "feasible_found": feasible_result is not None,
            "cma_update_completed": False,
        }
        if feasible_result is None:
            if len(resolved) != 8:
                raise RuntimeError("generation did not produce eight candidate scores")
            scores = [float(item["fitness"]) for item in resolved]
            strategy.tell(population, scores)
            generation_document.update({
                "scores": scores,
                "mean_after": [float(value) for value in strategy.mean],
                "sigma_after": float(strategy.sigma),
                "cma_update_completed": True,
            })
        generation_documents.append(generation_document)
        atomic_write_json(
            output / "generations" / f"generation_{generation:03d}.json",
            generation_document,
        )
        if feasible_result is not None:
            break

    feasible_seed = (
        _copy_feasible_seed(feasible_result, output)
        if feasible_result is not None else None
    )
    status = "FEASIBLE_SEED_FOUND" if feasible_seed else "PARAMETER_SPACE_BLOCKER"
    summary = {
        "schema": "p3_cma_feasible_seed_search/1",
        "task": "P3_CMA_FEASIBLE_SEED_SEARCH",
        "created_at_utc": _utc_now(),
        "status": status,
        "population": 8,
        "maximum_generations": 8,
        "completed_generations": len(generation_documents),
        "maximum_evaluations": 64,
        "completed_evaluations": evaluations,
        "initial_workers": 4,
        "final_workers": active_workers,
        "infrastructure_failure_count": timeout_or_infrastructure_failures,
        "retry_count": retry_count,
        "parameter_count": 6,
        "ranking_source": "EXISTING_SAFETY_DOMINANT_OBJECTIVE_UNCHANGED",
        "continuous_penalty_added": False,
        "hard_safety_relaxed": False,
        "scenario_class": "Q2_COMPETITION_STYLE",
        "competition_rule_gate_sha256": EXPECTED_RULE_GATE_SHA256,
        "feasible_seed": feasible_seed,
        "main_cma_ready": feasible_seed is not None,
        "parameter_space_blocker": feasible_seed is None and evaluations == 64,
    }
    atomic_write_json(output / "search_summary.json", summary)
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
