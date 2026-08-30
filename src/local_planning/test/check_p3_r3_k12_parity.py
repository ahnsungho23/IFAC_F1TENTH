#!/usr/bin/env python3
# Copyright 2026 2026_IFAC contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# flake8: noqa
"""Compare integrated R3-K12 against the immutable seen-only frozen artifacts."""

from __future__ import annotations

import argparse
import csv
import pathlib
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor


REPO = pathlib.Path(__file__).resolve().parents[3]
STUDY = REPO / "planning_study/p3_geometry_factor_ranking_v2"
EVENT_ROOTS = (
    REPO / "planning_study/p3_oracle_pilot_v2/inputs",
    REPO / "planning_study/p3_mapping_research_corpus_v1/raw_oracle/inputs",
    REPO / "planning_study/p3_geometry_conditioned_validation_v1/raw_oracle/inputs",
)


def exact(value: str) -> str:
    return float(value).hex()


def truth(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def read_csv(path: pathlib.Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def factor_key(fields: list[str]) -> tuple[str, ...]:
    return (
        fields[4], exact(fields[5]), exact(fields[6]), exact(fields[7]), exact(fields[8]),
        fields[9], fields[10], fields[11], fields[14], fields[15],
    )


def expected_factor_key(row: dict[str, str]) -> tuple[str, ...]:
    return (
        row["side"], exact(row["d_target"]), exact(row["d_mid"]),
        exact(row["entry_scale"]), exact(row["exit_scale"]), row["target_source"],
        row["mid_source"], row["path_digest"], "1" if truth(row["hard_valid"]) else "0",
        "1" if truth(row["usable_valid"]) else "0",
    )


def selected_expected(summary: dict[str, str], factors: list[dict[str, str]]) -> str:
    if not truth(summary["hard_recovered"]):
        return "NONE"
    key = (
        summary["best_hard_side"], exact(summary["best_hard_d_target"]),
        exact(summary["best_hard_d_mid"]), exact(summary["best_hard_entry_scale"]),
        exact(summary["best_hard_exit_scale"]),
    )
    matches = [
        row for row in factors if (
            row["side"], exact(row["d_target"]), exact(row["d_mid"]),
            exact(row["entry_scale"]), exact(row["exit_scale"]),
        ) == key
    ]
    digests = {row["path_digest"] for row in matches}
    if len(digests) != 1:
        raise RuntimeError(f"expected selected tuple is not unique: {summary['event_id']}")
    return next(iter(digests))


def check_event(
    harness: pathlib.Path,
    event_path: pathlib.Path,
    expected_factors: list[dict[str, str]],
    expected_summary: dict[str, str],
) -> dict[str, object]:
    completed = subprocess.run(
        [str(harness), str(event_path)], text=True, capture_output=True, check=False)
    if completed.returncode != 0:
        raise RuntimeError(
            f"{event_path.stem}: harness exit {completed.returncode}: {completed.stderr.strip()}")
    records = [line.split("\t") for line in completed.stdout.splitlines()]
    summary = next(record for record in records if record[0] == "SUMMARY")
    downstream = next(record for record in records if record[0] == "DOWNSTREAM_SELECTED")
    factors = [record for record in records if record[0] == "FACTOR"]
    expected_factors = sorted(expected_factors, key=lambda row: int(row["selection_rank"]))

    factor_order_exact = len(factors) == len(expected_factors) and all(
        factor_key(actual) == expected_factor_key(expected)
        for actual, expected in zip(factors, expected_factors))
    expected_counts = (
        int(expected_summary["lateral_factor_evaluations"]),
        int(expected_summary["pair_priority_evaluations"]),
        int(expected_summary["fully_reconstructed_candidate_count"]),
        int(expected_summary["duplicate_constructed_paths"]),
        int(expected_summary["logical_exact_validator_calls"]),
        int(expected_summary["hard_valid_count"]),
        int(expected_summary["usable_valid_count"]),
    )
    actual_counts = tuple(int(summary[index]) for index in range(4, 11))
    expected_digest = selected_expected(expected_summary, expected_factors)
    selected_digest_exact = summary[14] == expected_digest
    recovery_exact = truth(summary[3]) == truth(expected_summary["hard_recovered"])
    downstream_selected_exact = (
        not truth(expected_summary["hard_recovered"]) or downstream[3] == expected_digest)
    overall = (
        factor_order_exact and actual_counts == expected_counts and selected_digest_exact and
        downstream_selected_exact and recovery_exact)
    return {
        "dataset_role": expected_summary["dataset_role"],
        "event_id": event_path.stem,
        "ordered_factor_candidates_exact": factor_order_exact,
        "expected_reconstructed_count": expected_counts[2],
        "integrated_reconstructed_count": actual_counts[2],
        "reconstructed_count_exact": actual_counts[2] == expected_counts[2],
        "expected_duplicate_path_count": expected_counts[3],
        "integrated_duplicate_path_count": actual_counts[3],
        "path_digest_sequence_exact": factor_order_exact,
        "expected_validator_calls": expected_counts[4],
        "integrated_validator_calls": actual_counts[4],
        "validator_count_exact": actual_counts[4] == expected_counts[4],
        "validator_verdict_sequence_exact": factor_order_exact,
        "expected_hard_valid_count": expected_counts[5],
        "integrated_hard_valid_count": actual_counts[5],
        "hard_valid_sequence_exact": factor_order_exact,
        "expected_usable_valid_count": expected_counts[6],
        "integrated_usable_valid_count": actual_counts[6],
        "usable_valid_sequence_exact": factor_order_exact,
        "expected_hard_recovered": truth(expected_summary["hard_recovered"]),
        "integrated_hard_recovered": truth(summary[3]),
        "expected_selected_path_digest": expected_digest,
        "integrated_selected_path_digest": summary[14],
        "selected_candidate_exact": selected_digest_exact,
        "downstream_selected_path_digest": downstream[3],
        "downstream_selected_candidate_exact": downstream_selected_exact,
        "recovery_outcome_exact": recovery_exact,
        "factor_pool_count_exact": actual_counts[:2] == expected_counts[:2],
        "overall_parity": overall,
        "integrated_r3_runtime_us": summary[16],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--harness", required=True, type=pathlib.Path)
    parser.add_argument("--output", type=pathlib.Path)
    parser.add_argument("--recovery-output", type=pathlib.Path)
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--event", action="append", default=[])
    arguments = parser.parse_args()

    event_paths = {
        path.stem: path for root in EVENT_ROOTS for path in root.glob("*.event")
    }
    candidate_rows = read_csv(STUDY / "selected_candidates_audit.csv")
    expected_factors: dict[str, list[dict[str, str]]] = {}
    for row in candidate_rows:
        if (row["hypothesis"] == "R3_LEXICOGRAPHIC_COVERAGE_RESERVE" and
                row["budget_k"] == "12"):
            expected_factors.setdefault(row["event_id"], []).append(row)
    summary_rows = read_csv(STUDY / "factor_ranking_results.csv")
    expected_summaries = {
        row["event_id"]: row for row in summary_rows
        if row["hypothesis"] == "R3_LEXICOGRAPHIC_COVERAGE_RESERVE" and
        row["budget_k"] == "12"
    }
    event_ids = sorted(arguments.event or expected_factors)
    missing = [event_id for event_id in event_ids if event_id not in event_paths]
    if missing:
        raise RuntimeError("missing seen event inputs: " + ",".join(missing))

    def run(event_id: str) -> dict[str, object]:
        return check_event(
            arguments.harness, event_paths[event_id], expected_factors[event_id],
            expected_summaries[event_id])

    with ThreadPoolExecutor(max_workers=max(1, arguments.jobs)) as executor:
        rows = list(executor.map(run, event_ids))
    if arguments.output:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        with arguments.output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    if arguments.recovery_output:
        recovery_rows = [row for row in rows if row["expected_hard_recovered"]]
        arguments.recovery_output.parent.mkdir(parents=True, exist_ok=True)
        with arguments.recovery_output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(recovery_rows[0]))
            writer.writeheader()
            writer.writerows(recovery_rows)
    passed = sum(bool(row["overall_parity"]) for row in rows)
    print(f"R3_K12_SEEN_PARITY {passed}/{len(rows)}")
    for row in rows:
        if not row["overall_parity"]:
            print(f"PARITY_MISMATCH {row['event_id']} {row}")
    return 0 if passed == len(rows) else 1


if __name__ == "__main__":
    sys.exit(main())
