"""Pure helpers for deterministic lockstep traces and common-random-number schedules."""

from __future__ import annotations

import hashlib
import json
import math
import statistics
from typing import Any, Iterable


def float_token(value: float) -> str:
    number = float(value)
    return number.hex() if math.isfinite(number) else str(number)


def canonical_hash(value: Any) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def numeric_stats(values: Iterable[float | int | None]) -> dict[str, float | int | None]:
    samples = [float(value) for value in values if value is not None]
    if not samples:
        return {"count": 0, "mean": None, "std": None, "min": None, "max": None}
    return {
        "count": len(samples),
        "mean": statistics.fmean(samples),
        "std": statistics.stdev(samples) if len(samples) > 1 else 0.0,
        "min": min(samples),
        "max": max(samples),
    }


def common_random_number_schedule(
    candidate_ids: Iterable[str], scenario_ids: Iterable[str], simulator_seeds: Iterable[int]
) -> list[dict[str, Any]]:
    """Return candidate-major jobs while preserving identical scenario/seed pairs."""
    pairs = [
        {"scenario_id": str(scenario), "simulator_seed": int(seed)}
        for scenario in scenario_ids
        for seed in simulator_seeds
    ]
    return [
        {"candidate_id": str(candidate), **pair}
        for candidate in candidate_ids
        for pair in pairs
    ]


def summarize_lockstep_runs(results: list[dict[str, Any]]) -> dict[str, Any]:
    if not results:
        raise ValueError("at least one lockstep result is required")
    exact_fields = (
        "physics_state_sequence_hash",
        "scan_hash_sequence_hash",
        "static_geometry_sequence_hash",
        "avoid_waypoints_hash_sequence_hash",
        "controller_command_sequence_hash",
        "trajectory_hash",
        "collision",
        "off_track",
        "planner_failure",
        "confirmation_scan_index",
        "commitment_scan_index",
        "committed_obstacle_id",
        "selected_side",
        "entry_transition_scale",
        "exit_transition_scale",
        "effective_entry_transition_scale",
        "effective_exit_transition_scale",
    )
    boolean_fields = {"collision", "off_track", "planner_failure"}
    exact_agreement = {
        field: len({
            json.dumps(
                bool(result.get(field)) if field in boolean_fields else result.get(field),
                sort_keys=True,
            )
            for result in results
        }) == 1
        for field in exact_fields
    }
    targets = [result.get("committed_target_d") for result in results]
    fitness = [result.get("fitness") for result in results]
    target_stats = numeric_stats(targets)
    fitness_stats = numeric_stats(fitness)
    safety_outcomes = {
        (
            bool(result.get("collision")),
            bool(result.get("off_track")),
            bool(result.get("planner_failure")),
        )
        for result in results
    }
    accepted = (
        all(exact_agreement.values())
        and len(safety_outcomes) == 1
        and float(target_stats["std"] or 0.0) <= 1.0e-12
        and float(fitness_stats["std"] or 0.0) <= 1.0e-12
        and all(bool(result.get("valid")) for result in results)
    )
    return {
        "run_count": len(results),
        "accepted": accepted,
        "safety_outcome_flip_count": max(0, len(safety_outcomes) - 1),
        "unique_safety_outcomes": [
            {"collision": collision, "off_track": off_track,
             "planner_failure": planner_failure}
            for collision, off_track, planner_failure in sorted(safety_outcomes)
        ],
        "exact_agreement": exact_agreement,
        "target_d_statistics": target_stats,
        "fitness_statistics": fitness_stats,
        "unique_path_sequence_hashes": sorted(
            {str(result.get("avoid_waypoints_hash_sequence_hash")) for result in results}
        ),
        "unique_trajectory_hashes": sorted(
            {str(result.get("trajectory_hash")) for result in results}
        ),
    }
