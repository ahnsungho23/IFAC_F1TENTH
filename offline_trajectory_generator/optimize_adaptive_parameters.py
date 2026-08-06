#!/usr/bin/env python3
"""Find safe local-planner parameters with feasibility-first CMA-ES."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import tempfile
from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Sequence
from zoneinfo import ZoneInfo

import numpy as np
import yaml

from generate_adaptive_overlays import (
    evaluator_path,
    load_evaluations,
    load_yaml,
    planner_values,
    run_evaluator,
)


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
DEFAULT_REFERENCE = (
    REPO_ROOT / "ruleset_adaptive_globalpath/map_smooth_4p1/global_waypoints.csv"
)
DEFAULT_LOCAL_PARAMS = REPO_ROOT / "src/local_planning/config/local_planning.yaml"
DEFAULT_SEARCH_CONFIG = SCRIPT_DIR / "config/adaptive_cmaes.yaml"
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "learning_adaptive_globalpath/cmaes"


@dataclass(frozen=True)
class ParameterRange:
    name: str
    minimum: float
    maximum: float
    initial: float

    def __post_init__(self) -> None:
        values = (self.minimum, self.maximum, self.initial)
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"non-finite CMA-ES range for {self.name}")
        if self.maximum <= self.minimum:
            raise ValueError(f"invalid CMA-ES bounds for {self.name}")
        if not self.minimum <= self.initial <= self.maximum:
            raise ValueError(f"initial value is outside bounds for {self.name}")

    def decode(self, normalized: float) -> float:
        clipped = min(1.0, max(0.0, float(normalized)))
        return self.minimum + clipped * (self.maximum - self.minimum)

    def encode(self, value: float) -> float:
        return (float(value) - self.minimum) / (self.maximum - self.minimum)


class SearchSpace:
    """Normalized CMA genome with ordered transition distances."""

    SCALAR_KEYS = (
        "outside_line_transition_scale",
        "commitment_clearance_reserve_m",
        "minimum_avoidance_clearance_m",
        "boundary_margin_m",
        "minimum_target_offset_m",
        "maximum_lateral_slope",
    )

    def __init__(self, config: dict[str, Any], base_values: dict[str, Any]) -> None:
        self.base_values = dict(base_values)
        transition = config.get("transition_distance_scales", {})
        initial_scales = [float(value) for value in transition.get("initial", [])]
        if len(initial_scales) != 3 or not (
            0.0 < initial_scales[0] < initial_scales[1] < initial_scales[2]
        ):
            raise ValueError("transition_distance_scales.initial must have three ordered values")

        def pair(name: str) -> tuple[float, float]:
            values = transition.get(name, [])
            if not isinstance(values, list) or len(values) != 2:
                raise ValueError(f"{name} must be a two-value list")
            return float(values[0]), float(values[1])

        self.ranges = [
            ParameterRange("transition_first", *pair("first_bounds"), initial_scales[0]),
            ParameterRange(
                "transition_gap_1",
                *pair("first_gap_bounds"),
                initial_scales[1] - initial_scales[0],
            ),
            ParameterRange(
                "transition_gap_2",
                *pair("second_gap_bounds"),
                initial_scales[2] - initial_scales[1],
            ),
        ]

        range_config = config.get("parameter_ranges", {})
        physical_clearance = (
            float(base_values["vehicle_half_width_m"])
            + float(base_values["hard_collision_margin_m"])
        )
        obstacle_clearance = float(base_values["obstacle_clearance_m"])
        for key in self.SCALAR_KEYS:
            spec = range_config.get(key, {})
            bounds = spec.get("bounds", [])
            if not isinstance(bounds, list) or len(bounds) != 2:
                raise ValueError(f"parameter_ranges.{key}.bounds must have two values")
            minimum, maximum = float(bounds[0]), float(bounds[1])
            if key == "minimum_avoidance_clearance_m":
                minimum = max(minimum, physical_clearance)
                maximum = min(maximum, obstacle_clearance)
            self.ranges.append(
                ParameterRange(key, minimum, maximum, float(spec.get("initial")))
            )

    @property
    def dimension(self) -> int:
        return len(self.ranges)

    @property
    def initial_vector(self) -> np.ndarray:
        return np.asarray([item.encode(item.initial) for item in self.ranges], dtype=float)

    @property
    def clearance_range(self) -> ParameterRange:
        return next(item for item in self.ranges if item.name == "minimum_avoidance_clearance_m")

    def decode(self, normalized: Sequence[float]) -> dict[str, Any]:
        if len(normalized) != self.dimension:
            raise ValueError(f"expected {self.dimension} CMA values, got {len(normalized)}")
        decoded = {
            item.name: item.decode(value) for item, value in zip(self.ranges, normalized)
        }
        first = decoded.pop("transition_first")
        second = first + decoded.pop("transition_gap_1")
        third = second + decoded.pop("transition_gap_2")
        result = dict(self.base_values)
        result["transition_distance_scales"] = [first, second, third]
        result.update(decoded)
        return result

    def encode(self, parameters: dict[str, Any]) -> np.ndarray:
        scales = [float(value) for value in parameters["transition_distance_scales"]]
        physical = {
            "transition_first": scales[0],
            "transition_gap_1": scales[1] - scales[0],
            "transition_gap_2": scales[2] - scales[1],
            **{key: float(parameters[key]) for key in self.SCALAR_KEYS},
        }
        return np.asarray([item.encode(physical[item.name]) for item in self.ranges], dtype=float)


@dataclass
class CandidateResult:
    generation: int
    candidate: int
    parameters: dict[str, Any]
    decision_counts: dict[str, int]
    selected_reduced_clearance: int
    min_selected_headroom: float
    row_count: int
    error: str = ""

    @property
    def safe_stop_count(self) -> int:
        return self.decision_counts.get("safe_stop", 0)

    @property
    def clearance(self) -> float:
        return float(self.parameters["minimum_avoidance_clearance_m"])

    @property
    def rank_key(self) -> tuple[float, ...]:
        if self.error:
            return (1.0, math.inf, math.inf, math.inf, math.inf)
        # Feasibility first. Only candidates with the same safe-stop count compete on clearance.
        return (
            0.0,
            float(self.safe_stop_count),
            -self.clearance,
            float(self.selected_reduced_clearance),
            -self.min_selected_headroom,
        )


def summarize_evaluations(
    rows: list[dict[str, str]],
    generation: int,
    candidate: int,
    parameters: dict[str, Any],
) -> CandidateResult:
    decisions = Counter(row["decision"] for row in rows)
    headrooms: list[float] = []
    for row in rows:
        side = row["decision"]
        if side not in {"left", "right"}:
            continue
        text = row.get(f"{side}_headroom", "")
        if text:
            headrooms.append(float(text))
    return CandidateResult(
        generation=generation,
        candidate=candidate,
        parameters=parameters,
        decision_counts=dict(decisions),
        selected_reduced_clearance=sum(
            row.get("selected_reduced_clearance") == "1" for row in rows
        ),
        min_selected_headroom=min(headrooms) if headrooms else -math.inf,
        row_count=len(rows),
    )


def rank_population(results: list[CandidateResult]) -> list[float]:
    order = sorted(range(len(results)), key=lambda index: results[index].rank_key)
    ranks = [0.0] * len(results)
    for rank, index in enumerate(order):
        ranks[index] = float(rank)
    return ranks


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, default=DEFAULT_REFERENCE)
    parser.add_argument("--local-params", type=Path, default=DEFAULT_LOCAL_PARAMS)
    parser.add_argument("--search-config", type=Path, default=DEFAULT_SEARCH_CONFIG)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--evaluator", type=Path)
    parser.add_argument("--workers", type=int, default=min(8, os.cpu_count() or 1))
    parser.add_argument("--population-size", type=int, default=12)
    parser.add_argument("--max-generations", type=int, default=80)
    parser.add_argument("--stall-generations", type=int, default=15)
    parser.add_argument("--sigma", type=float, default=0.20)
    parser.add_argument("--seed", type=int, default=20260806)
    parser.add_argument("--strict-count", type=int, default=1562)
    parser.add_argument("--clearance-tolerance", type=float, default=1.0e-4)
    parser.add_argument("--ego-lookback", type=float, default=7.0)
    parser.add_argument("--obstacle-size", type=float, default=0.20)
    parser.add_argument("--d-min", type=float, default=-0.5)
    parser.add_argument("--d-max", type=float, default=0.5)
    parser.add_argument("--d-step", type=float, default=0.1)
    parser.add_argument("--timezone", default="Asia/Seoul")
    parser.add_argument("--run-id")
    return parser.parse_args()


def make_run_id(timezone_name: str) -> tuple[str, str]:
    now = datetime.now(ZoneInfo(timezone_name))
    milliseconds = now.microsecond // 1000
    return f"cmaes_{now:%Y%m%d_%H%M%S}_{milliseconds:03d}_KST", now.isoformat(
        timespec="milliseconds"
    )


def evaluate_candidate(
    generation: int,
    candidate: int,
    parameters: dict[str, Any],
    executable: Path,
    reference: Path,
    evaluator_args: SimpleNamespace,
    strict_count: int,
) -> CandidateResult:
    try:
        with tempfile.TemporaryDirectory(prefix="adaptive_cmaes_") as directory:
            output = Path(directory) / "evaluations.csv"
            run_evaluator(executable, reference, output, parameters, evaluator_args)
            rows = load_evaluations(output)
        if len(rows) != strict_count:
            raise RuntimeError(f"expected {strict_count} rows, got {len(rows)}")
        return summarize_evaluations(rows, generation, candidate, parameters)
    except Exception as exc:  # noqa: BLE001 - invalid candidates must be ranked, not abort the run.
        return CandidateResult(
            generation=generation,
            candidate=candidate,
            parameters=parameters,
            decision_counts={},
            selected_reduced_clearance=0,
            min_selected_headroom=-math.inf,
            row_count=0,
            error=str(exc),
        )


def candidate_record(result: CandidateResult, rank: float) -> dict[str, Any]:
    scales = result.parameters["transition_distance_scales"]
    return {
        "generation": result.generation,
        "candidate": result.candidate,
        "rank": rank,
        "safe_stop": result.safe_stop_count,
        "left": result.decision_counts.get("left", 0),
        "right": result.decision_counts.get("right", 0),
        "minimum_avoidance_clearance_m": result.clearance,
        "min_selected_headroom": result.min_selected_headroom,
        "selected_reduced_clearance": result.selected_reduced_clearance,
        "transition_scale_1": scales[0],
        "transition_scale_2": scales[1],
        "transition_scale_3": scales[2],
        "outside_line_transition_scale": result.parameters["outside_line_transition_scale"],
        "commitment_clearance_reserve_m": result.parameters[
            "commitment_clearance_reserve_m"
        ],
        "boundary_margin_m": result.parameters["boundary_margin_m"],
        "minimum_target_offset_m": result.parameters["minimum_target_offset_m"],
        "maximum_lateral_slope": result.parameters["maximum_lateral_slope"],
        "row_count": result.row_count,
        "error": result.error,
    }


def write_best_yaml(path: Path, result: CandidateResult) -> None:
    payload = {"local_planner_node": {"ros__parameters": result.parameters}}
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


def main() -> int:
    args = parse_args()
    if min(
        args.workers,
        args.population_size,
        args.max_generations,
        args.stall_generations,
        args.strict_count,
    ) <= 0:
        raise RuntimeError("workers, population/generation counts, and strict-count must be positive")
    if not 0.0 < args.sigma <= 1.0:
        raise RuntimeError("--sigma must be in (0, 1]")

    try:
        import cma
    except ImportError as exc:
        raise RuntimeError(
            "CMA-ES dependency is missing; run `python3 -m pip install -r "
            "offline_trajectory_generator/requirements.txt`"
        ) from exc

    generated_id, started_at = make_run_id(args.timezone)
    run_id = args.run_id or generated_id
    if not run_id or Path(run_id).name != run_id:
        raise RuntimeError("--run-id must be one directory name")
    output_root = args.output_root.expanduser().resolve()
    working_dir = output_root / "_incomplete" / run_id
    final_dir = output_root / run_id
    if working_dir.exists() or final_dir.exists():
        raise RuntimeError(f"run directory already exists: {run_id}")
    report_dir = working_dir / "reports"
    parameter_dir = working_dir / "parameters"
    report_dir.mkdir(parents=True)
    parameter_dir.mkdir()

    reference = args.reference.expanduser().resolve()
    base_values = planner_values(args.local_params.expanduser().resolve(), [])
    search_config = load_yaml(args.search_config.expanduser().resolve())
    space = SearchSpace(search_config, base_values)
    executable = evaluator_path(args.evaluator)
    evaluator_args = SimpleNamespace(
        ego_lookback=args.ego_lookback,
        obstacle_size=args.obstacle_size,
        d_min=args.d_min,
        d_max=args.d_max,
        d_step=args.d_step,
    )

    run_config = {
        "run_id": run_id,
        "started_at": started_at,
        "timezone": args.timezone,
        "reference": str(reference),
        "local_params": str(args.local_params.expanduser().resolve()),
        "search_config": str(args.search_config.expanduser().resolve()),
        "evaluator": str(executable),
        "population_size": args.population_size,
        "max_generations": args.max_generations,
        "stall_generations": args.stall_generations,
        "sigma": args.sigma,
        "seed": args.seed,
        "strict_count": args.strict_count,
        "objective_order": [
            "valid_candidate",
            "minimum_safe_stop_count",
            "maximum_minimum_avoidance_clearance_m",
            "minimum_reduced_clearance_selections",
            "maximum_selected_headroom",
        ],
    }
    (parameter_dir / "run_config.yaml").write_text(
        yaml.safe_dump(run_config, sort_keys=False), encoding="utf-8"
    )
    (parameter_dir / "effective_search_space.yaml").write_text(
        yaml.safe_dump(
            {
                item.name: {
                    "minimum": item.minimum,
                    "maximum": item.maximum,
                    "initial": item.initial,
                }
                for item in space.ranges
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    initial_vector = space.initial_vector
    initial_parameters = space.decode(initial_vector)
    incumbent = evaluate_candidate(
        -1,
        0,
        initial_parameters,
        executable,
        reference,
        evaluator_args,
        args.strict_count,
    )
    if incumbent.error:
        raise RuntimeError(f"initial candidate failed: {incumbent.error}")
    write_best_yaml(parameter_dir / "initial_parameters.yaml", incumbent)
    print(
        f"[baseline] safe_stop={incumbent.safe_stop_count} "
        f"clearance={incumbent.clearance:.6f}",
        flush=True,
    )

    options = {
        "bounds": [0.0, 1.0],
        "popsize": args.population_size,
        "seed": args.seed,
        "verbose": -9,
    }
    mean = np.clip(initial_vector, 1.0e-6, 1.0 - 1.0e-6)
    strategy = cma.CMAEvolutionStrategy(mean.tolist(), args.sigma, options)
    history_path = report_dir / "history.csv"
    history_fields: list[str] | None = None
    stall_count = 0
    stop_reason = "max_generations"
    completed_generations = 0

    for generation in range(args.max_generations):
        solutions = strategy.ask()
        # Always retain the best feasible point found so far, including the exact boundary start.
        solutions[0] = space.encode(incumbent.parameters).tolist()
        results: list[CandidateResult | None] = [None] * len(solutions)
        with ThreadPoolExecutor(max_workers=min(args.workers, len(solutions))) as executor:
            futures = {
                executor.submit(
                    evaluate_candidate,
                    generation,
                    candidate,
                    space.decode(solution),
                    executable,
                    reference,
                    evaluator_args,
                    args.strict_count,
                ): candidate
                for candidate, solution in enumerate(solutions)
            }
            for future in as_completed(futures):
                results[futures[future]] = future.result()
        evaluated = [result for result in results if result is not None]
        if len(evaluated) != len(solutions):
            raise RuntimeError("internal error: CMA-ES candidate result is missing")
        ranks = rank_population(evaluated)
        strategy.tell(solutions, ranks)

        records = [candidate_record(result, ranks[index]) for index, result in enumerate(evaluated)]
        if history_fields is None:
            history_fields = list(records[0])
        with history_path.open("a", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=history_fields)
            if generation == 0:
                writer.writeheader()
            writer.writerows(records)

        generation_best = min(evaluated, key=lambda result: result.rank_key)
        previous_safe = incumbent.safe_stop_count
        previous_clearance = incumbent.clearance
        if generation_best.rank_key < incumbent.rank_key:
            incumbent = generation_best
        meaningful_improvement = (
            incumbent.safe_stop_count < previous_safe
            or (
                incumbent.safe_stop_count == previous_safe
                and incumbent.clearance
                >= previous_clearance + args.clearance_tolerance
            )
        )
        stall_count = 0 if meaningful_improvement else stall_count + 1
        completed_generations = generation + 1
        print(
            f"[generation {generation:03d}] safe_stop={incumbent.safe_stop_count} "
            f"clearance={incumbent.clearance:.6f} stall={stall_count}",
            flush=True,
        )

        clearance_upper = space.clearance_range.maximum
        if (
            incumbent.safe_stop_count == 0
            and incumbent.clearance >= clearance_upper - args.clearance_tolerance
        ):
            stop_reason = "safe_at_clearance_upper_bound"
            break
        if incumbent.safe_stop_count == 0 and stall_count >= args.stall_generations:
            stop_reason = "safe_solution_stalled"
            break
        if strategy.stop():
            stop_reason = "cmaes_stop_condition"
            break

    final_csv = report_dir / "best_side_evaluations.csv"
    run_evaluator(executable, reference, final_csv, incumbent.parameters, evaluator_args)
    final_rows = load_evaluations(final_csv)
    final_result = summarize_evaluations(
        final_rows, completed_generations, 0, incumbent.parameters
    )
    if final_result.rank_key != incumbent.rank_key:
        raise RuntimeError("final best-candidate evaluation did not reproduce the incumbent")
    write_best_yaml(parameter_dir / "best_parameters.yaml", final_result)

    summary = {
        **run_config,
        "completed_generations": completed_generations,
        "stop_reason": stop_reason,
        "decision_counts": final_result.decision_counts,
        "safe_stop_count": final_result.safe_stop_count,
        "minimum_avoidance_clearance_m": final_result.clearance,
        "physical_clearance_floor_m": space.clearance_range.minimum,
        "clearance_upper_bound_m": space.clearance_range.maximum,
        "selected_reduced_clearance": final_result.selected_reduced_clearance,
        "min_selected_headroom": final_result.min_selected_headroom,
    }
    (report_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    (working_dir / "_SUCCESS").touch()
    final_dir.parent.mkdir(parents=True, exist_ok=True)
    os.replace(working_dir, final_dir)
    (output_root / "latest_run.txt").write_text(
        str(final_dir.relative_to(output_root)) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
