"""Compare two state-machine-rate end-to-end timing audits.

This module only reads completed repeatability artifacts.  It does not launch
ROS, mutate planner/controller parameters, or invoke CMA ask/tell.
"""

from __future__ import annotations

from collections import defaultdict
import argparse
import json
import math
from pathlib import Path
from statistics import mean, median, stdev
from typing import Any, Callable

from .schemas import atomic_write_json, sha256_file


LATENCY_KEYS = (
    "t3_minus_t2",
    "t4_minus_t3",
    "t5_minus_t4",
    "t8_minus_t6",
    "t9_minus_t8",
    "t9_minus_t1",
)


def _finite(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = fraction * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def statistics(values: list[float]) -> dict[str, float | int | None]:
    clean = [value for item in values if (value := _finite(item)) is not None]
    if not clean:
        return {
            "count": 0,
            "mean": None,
            "std": None,
            "min": None,
            "median": None,
            "p95": None,
            "max": None,
        }
    return {
        "count": len(clean),
        "mean": mean(clean),
        "std": stdev(clean) if len(clean) > 1 else 0.0,
        "min": min(clean),
        "median": median(clean),
        "p95": _percentile(clean, 0.95),
        "max": max(clean),
    }


def _timing(row: dict[str, Any]) -> dict[str, Any]:
    return row.get("noise", {}).get("end_to_end_timing", {})


def _latency(row: dict[str, Any], key: str) -> float | None:
    return _finite(_timing(row).get("latencies_ms", {}).get(key))


def _initial_target(row: dict[str, Any]) -> float | None:
    value = _finite(_timing(row).get("initial_committed_target_d_m"))
    if value is not None:
        return value
    return _finite(
        row.get("noise", {}).get("planner_log", {}).get("initial_commit_target_d_m")
    )


def _avoidance_start_s(row: dict[str, Any]) -> float | None:
    return _finite(_timing(row).get("avoidance_start_ego_s"))


def _grouped_dispersion(
    rows: list[dict[str, Any]], extractor: Callable[[dict[str, Any]], float | None]
) -> dict[str, Any]:
    grouped: dict[str, list[float]] = defaultdict(list)
    for row in rows:
        value = extractor(row)
        if value is not None:
            grouped[str(row["scenario_id"])].append(value)
    per_scenario = {
        scenario_id: statistics(values)
        for scenario_id, values in sorted(grouped.items())
    }
    variances = []
    degrees_of_freedom = 0
    scenario_stds = []
    for item in per_scenario.values():
        count = int(item["count"])
        std = _finite(item["std"])
        if std is None:
            continue
        scenario_stds.append(std)
        if count > 1:
            variances.append((count - 1) * std * std)
            degrees_of_freedom += count - 1
    return {
        "per_scenario": per_scenario,
        "mean_within_scenario_std": mean(scenario_stds) if scenario_stds else None,
        "maximum_within_scenario_std": max(scenario_stds) if scenario_stds else None,
        "pooled_within_scenario_std": (
            math.sqrt(sum(variances) / degrees_of_freedom)
            if degrees_of_freedom else None
        ),
    }


def _fitness_repeatability(rows: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "aggregate_across_scenarios": statistics(
            [float(row["fitness"]) for row in rows]
        ),
        "within_scenario": _grouped_dispersion(
            rows, lambda row: _finite(row.get("fitness"))
        ),
    }


def summarize(rows: list[dict[str, Any]], rate_hz: float) -> dict[str, Any]:
    by_scenario: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        by_scenario[str(row["scenario_id"])].append(row)

    flipped = []
    binary_flipped = []
    collision_flipped = []
    off_track_flipped = []
    planner_failure_flipped = []
    scenario_outcomes = {}
    for scenario_id, group in sorted(by_scenario.items()):
        signatures = sorted(
            {
                (
                    bool(row["collision"]),
                    bool(row["off_track"]),
                    bool(row["planner_failure"]),
                )
                for row in group
            }
        )
        scenario_outcomes[scenario_id] = [
            {
                "collision": item[0],
                "off_track": item[1],
                "planner_failure": item[2],
            }
            for item in signatures
        ]
        if len(signatures) > 1:
            flipped.append(scenario_id)
        binary_outcomes = {
            bool(row["collision"] or row["off_track"] or row["planner_failure"])
            for row in group
        }
        if len(binary_outcomes) > 1:
            binary_flipped.append(scenario_id)
        if len({bool(row["collision"]) for row in group}) > 1:
            collision_flipped.append(scenario_id)
        if len({bool(row["off_track"]) for row in group}) > 1:
            off_track_flipped.append(scenario_id)
        if len({bool(row["planner_failure"]) for row in group}) > 1:
            planner_failure_flipped.append(scenario_id)

    def fraction(predicate: Callable[[dict[str, Any]], bool]) -> float:
        return sum(predicate(row) for row in rows) / len(rows) if rows else 0.0

    complete = [row for row in rows if bool(_timing(row).get("complete_chain"))]
    success = [row for row in complete if row["classification"] == "success"]
    failure = [row for row in complete if row["classification"] != "success"]

    latency_all = {
        key: statistics(
            [value for row in complete if (value := _latency(row, key)) is not None]
        )
        for key in LATENCY_KEYS
    }
    latency_by_outcome = {
        name: {
            key: statistics(
                [value for row in group if (value := _latency(row, key)) is not None]
            )
            for key in LATENCY_KEYS
        }
        for name, group in (("success", success), ("failure", failure))
    }
    return {
        "state_machine_publish_rate_hz": rate_hz,
        "run_count": len(rows),
        "scenario_count": len(by_scenario),
        "success_rate": fraction(lambda row: row["classification"] == "success"),
        "collision_rate": fraction(lambda row: bool(row["collision"])),
        "off_track_rate": fraction(lambda row: bool(row["off_track"])),
        "planner_failure_rate": fraction(lambda row: bool(row["planner_failure"])),
        "collision_agreement_rate": fraction(
            lambda row: bool(row["collision_agreement"])
        ),
        "safety_flip_count": len(flipped),
        "safety_flip_scenarios": flipped,
        "binary_safety_outcome_flip_count": len(binary_flipped),
        "binary_safety_outcome_flip_scenarios": binary_flipped,
        "collision_flip_count": len(collision_flipped),
        "collision_flip_scenarios": collision_flipped,
        "off_track_flip_count": len(off_track_flipped),
        "off_track_flip_scenarios": off_track_flipped,
        "planner_failure_flip_count": len(planner_failure_flipped),
        "planner_failure_flip_scenarios": planner_failure_flipped,
        "scenario_safety_outcomes": scenario_outcomes,
        "fitness": _fitness_repeatability(rows),
        "complete_timing_chain_count": len(complete),
        "incomplete_timing_chain_count": len(rows) - len(complete),
        "latency_ms": latency_all,
        "latency_ms_by_outcome": latency_by_outcome,
        "avoidance_start_ego_s": _grouped_dispersion(rows, _avoidance_start_s),
        "initial_committed_target_d_m": _grouped_dispersion(rows, _initial_target),
        "infrastructure_retry_episode_count": sum(
            int(row.get("infrastructure_attempt_count", 1)) > 1 for row in rows
        ),
    }


def _ratio(new: float | None, old: float | None) -> float | None:
    if new is None or old in (None, 0.0):
        return None
    return new / old


def compare(
    ten_hz_rows: list[dict[str, Any]], hundred_hz_rows: list[dict[str, Any]]
) -> dict[str, Any]:
    ten = summarize(ten_hz_rows, 10.0)
    hundred = summarize(hundred_hz_rows, 100.0)
    changes = {
        "safety_flip_count": hundred["safety_flip_count"] - ten["safety_flip_count"],
        "binary_safety_outcome_flip_count": (
            hundred["binary_safety_outcome_flip_count"] -
            ten["binary_safety_outcome_flip_count"]
        ),
        "collision_flip_count": (
            hundred["collision_flip_count"] - ten["collision_flip_count"]
        ),
        "off_track_flip_count": (
            hundred["off_track_flip_count"] - ten["off_track_flip_count"]
        ),
        "collision_rate": hundred["collision_rate"] - ten["collision_rate"],
        "planner_failure_rate": (
            hundred["planner_failure_rate"] - ten["planner_failure_rate"]
        ),
    }
    for key, selector in (
        (
            "fitness_pooled_within_scenario_std_ratio_100_to_10",
            lambda item: item["fitness"]["within_scenario"][
                "pooled_within_scenario_std"
            ],
        ),
        (
            "t3_minus_t2_mean_ratio_100_to_10",
            lambda item: item["latency_ms"]["t3_minus_t2"]["mean"],
        ),
        (
            "t9_minus_t1_mean_ratio_100_to_10",
            lambda item: item["latency_ms"]["t9_minus_t1"]["mean"],
        ),
        (
            "avoidance_start_ego_s_pooled_std_ratio_100_to_10",
            lambda item: item["avoidance_start_ego_s"][
                "pooled_within_scenario_std"
            ],
        ),
        (
            "target_d_pooled_std_ratio_100_to_10",
            lambda item: item["initial_committed_target_d_m"][
                "pooled_within_scenario_std"
            ],
        ),
    ):
        changes[key] = _ratio(_finite(selector(hundred)), _finite(selector(ten)))
    return {
        "schema": "cmaes_timing_rate_comparison/1",
        "medium_or_full_cma_executed": False,
        "conditions_held_constant": {
            "localization_mode": "ground_truth",
            "scan_publication_mode": "fresh_only",
            "scan_noise_std_m": 0.01,
            "simulator_seed": 12345,
            "controller_speed_mps": 4.0,
            "candidate": "baseline",
        },
        "ten_hz": ten,
        "hundred_hz": hundred,
        "changes": changes,
    }


def _load_rows(root: Path) -> list[dict[str, Any]]:
    document = json.loads(
        (root / "repeatability_runs.json").read_text(encoding="utf-8")
    )
    rows = document.get("runs", [])
    if len(rows) != 25:
        raise ValueError(f"expected 25 valid episodes in {root}, got {len(rows)}")
    if {str(row.get("candidate")) for row in rows} != {"baseline"}:
        raise ValueError(f"comparison accepts baseline-only audits: {root}")
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ten-hz-experiment", type=Path, required=True)
    parser.add_argument("--hundred-hz-experiment", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    ten_hz_root = arguments.ten_hz_experiment.resolve()
    hundred_hz_root = arguments.hundred_hz_experiment.resolve()
    report = compare(_load_rows(ten_hz_root), _load_rows(hundred_hz_root))
    report["sources"] = {
        "ten_hz": {
            "experiment": str(ten_hz_root),
            "manifest_sha256": sha256_file(ten_hz_root / "experiment_manifest.json"),
            "repeatability_runs_sha256": sha256_file(
                ten_hz_root / "repeatability_runs.json"
            ),
            "repeatability_audit_sha256": sha256_file(
                ten_hz_root / "repeatability_audit.json"
            ),
        },
        "hundred_hz": {
            "experiment": str(hundred_hz_root),
            "manifest_sha256": sha256_file(
                hundred_hz_root / "experiment_manifest.json"
            ),
            "repeatability_runs_sha256": sha256_file(
                hundred_hz_root / "repeatability_runs.json"
            ),
            "repeatability_audit_sha256": sha256_file(
                hundred_hz_root / "repeatability_audit.json"
            ),
        },
    }
    atomic_write_json(arguments.output, report)
    print(json.dumps(report["changes"], indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
