#!/usr/bin/env python3
# flake8: noqa
"""Generate compact exact-Top-K complexity tables from the raw profile passes."""

from __future__ import annotations

import argparse
import csv
import pathlib


def read_off(path: pathlib.Path) -> list[dict[str, str]]:
    """Read the instrumentation-OFF rows from one raw profiler CSV."""
    with path.open(newline="", encoding="utf-8") as stream:
        return [
            row for row in csv.DictReader(stream)
            if row["mode"] == "PRODUCTION_EQUIVALENT"
        ]


def write(path: pathlib.Path, rows: list[dict[str, object]]) -> None:
    """Write a non-empty compact CSV."""
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def milliseconds(row: dict[str, str], field: str) -> str:
    """Format one microsecond timer as milliseconds."""
    return f"{float(row[field]) / 1000.0:.3f}"


def main() -> int:
    """Generate per-event factor and large-pool timing tables."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--before", required=True, type=pathlib.Path)
    parser.add_argument("--after", required=True, type=pathlib.Path)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    arguments = parser.parse_args()

    before = {row["event_id"]: row for row in read_off(arguments.before)}
    after = read_off(arguments.after)
    arguments.output.mkdir(parents=True, exist_ok=True)

    complexity = []
    for row in sorted(after, key=lambda item: item["event_id"]):
        pool = int(row["factor_pool_count"])
        evaluated = int(row["proxy_metric_evaluation_count"])
        reused = int(row["proxy_metric_cache_hit_count"])
        complexity.append({
            "event_id": row["event_id"],
            "lateral_factors": row["lateral_factor_count"],
            "transition_factors": row["transition_count"],
            "raw_combinations": row["raw_combination_count"],
            "production_configurations_excluded": row["production_excluded_count"],
            "unique_configurations": pool,
            "unique_exact_profiles": row["unique_profile_count"],
            "full_proxy_evaluations_after": evaluated,
            "exact_profile_reuses": reused,
            "evaluation_reduction_fraction": f"{(pool - evaluated) / pool:.6f}",
            "exact_skip_reason": (
                "EXACT_PROFILE_EQUIVALENCE_REUSE" if reused else
                "NONE_ALL_PROFILES_UNIQUE"
            ),
            "station_basis_builds": row["profile_basis_build_count"],
            "station_basis_reuses": row["profile_basis_cache_hit_count"],
            "r3_total_ms_after": milliseconds(row, "r3_total_us"),
        })
    write(arguments.output / "factor_pool_complexity.csv", complexity)

    largest = sorted(
        after, key=lambda item: int(item["factor_pool_count"]), reverse=True,
    )[:12]
    breakdown = []
    for row in largest:
        old = before[row["event_id"]]
        old_total = float(old["r3_total_us"])
        new_total = float(row["r3_total_us"])
        breakdown.append({
            "event_id": row["event_id"],
            "lateral_factors": row["lateral_factor_count"],
            "transition_factors": row["transition_count"],
            "raw_combinations": row["raw_combination_count"],
            "unique_configurations": row["factor_pool_count"],
            "full_proxy_before": row["factor_pool_count"],
            "full_proxy_after": row["proxy_metric_evaluation_count"],
            "exact_profile_reuses": row["proxy_metric_cache_hit_count"],
            "station_basis_builds_after": row["profile_basis_build_count"],
            "lateral_ms_after": milliseconds(row, "lateral_factor_generation_us"),
            "proxy_ms_before": milliseconds(old, "pair_priority_computation_us"),
            "proxy_ms_after": milliseconds(row, "pair_priority_computation_us"),
            "lex_ms_before": milliseconds(old, "lexicographic_ordering_us"),
            "lex_ms_after": milliseconds(row, "lexicographic_ordering_us"),
            "coverage_ms_before": milliseconds(old, "coverage_ordering_us"),
            "coverage_ms_after": milliseconds(row, "coverage_ordering_us"),
            "validation_ms_after": milliseconds(row, "exact_validation_us"),
            "r3_total_ms_before": f"{old_total / 1000.0:.3f}",
            "r3_total_ms_after": f"{new_total / 1000.0:.3f}",
            "r3_runtime_reduction_fraction": f"{1.0 - new_total / old_total:.6f}",
        })
    write(arguments.output / "worst_case_breakdown.csv", breakdown)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
