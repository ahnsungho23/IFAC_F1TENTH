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
"""Measure R3-K12 core, instrumentation, and standalone harness costs."""

from __future__ import annotations

import argparse
import csv
import os
import pathlib
import subprocess
import time


REPO = pathlib.Path(__file__).resolve().parents[3]
STUDY = REPO / "planning_study/p3_geometry_factor_ranking_v2"
EVENT_ROOTS = (
    REPO / "planning_study/p3_oracle_pilot_v2/inputs",
    REPO / "planning_study/p3_mapping_research_corpus_v1/raw_oracle/inputs",
    REPO / "planning_study/p3_geometry_conditioned_validation_v1/raw_oracle/inputs",
)


def read_csv(path: pathlib.Path) -> list[dict[str, str]]:
    """Read one compact research CSV."""
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def selected_event_ids() -> list[str]:
    """Return the immutable combined-seen R3-K12 evaluation event set."""
    rows = read_csv(STUDY / "factor_ranking_results.csv")
    return sorted({
        row["event_id"] for row in rows
        if row["hypothesis"] == "R3_LEXICOGRAPHIC_COVERAGE_RESERVE"
        and row["budget_k"] == "12"
    })


def parse_profile(stdout: str) -> dict[str, str]:
    """Parse the machine-readable PROFILE line emitted by the harness."""
    lines = [line for line in stdout.splitlines() if line.startswith("PROFILE\t")]
    if len(lines) != 1:
        raise RuntimeError(f"expected one PROFILE line, received {len(lines)}")
    fields = lines[0].split("\t")
    row = {"event_id": fields[1]}
    for field in fields[2:]:
        key, value = field.split("=", 1)
        row[key] = value
    return row


def run_event(
    harness: pathlib.Path, event_path: pathlib.Path, mode: str,
) -> dict[str, object]:
    """Run one event in a fresh process so harness overhead is observable."""
    environment = os.environ.copy()
    environment["R3_PROFILE_ONLY"] = "1"
    if mode == "INSTRUMENTATION_ON":
        environment["LOCAL_PLANNING_RESEARCH_PARITY"] = "1"
        environment["R3_PROFILE_SERIALIZE"] = "1"
    else:
        environment.pop("LOCAL_PLANNING_RESEARCH_PARITY", None)
        environment.pop("R3_PROFILE_SERIALIZE", None)
    start_ns = time.perf_counter_ns()
    completed = subprocess.run(
        [str(harness), str(event_path)], text=True, capture_output=True,
        check=False, env=environment,
    )
    process_wall_us = (time.perf_counter_ns() - start_ns) / 1000.0
    if completed.returncode:
        raise RuntimeError(
            f"{event_path.stem}: harness exit {completed.returncode}: "
            f"{completed.stderr.strip()}"
        )
    profile = parse_profile(completed.stdout)
    accounted_us = sum(float(profile[key]) for key in (
        "event_read_us", "reference_setup_us", "evaluation_wall_us",
        "research_serialization_us",
    ))
    return {
        "mode": mode,
        "event_id": event_path.stem,
        "process_wall_us": process_wall_us,
        "standalone_harness_overhead_us": process_wall_us - accounted_us,
        **{key: value for key, value in profile.items() if key != "event_id"},
    }


def main() -> int:
    """Profile all immutable seen events and write per-event raw measurements."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--harness", required=True, type=pathlib.Path)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    parser.add_argument(
        "--mode", action="append",
        choices=("PRODUCTION_EQUIVALENT", "INSTRUMENTATION_ON"),
    )
    parser.add_argument("--event", action="append", default=[])
    arguments = parser.parse_args()

    event_paths = {
        path.stem: path for root in EVENT_ROOTS for path in root.glob("*.event")
    }
    event_ids = sorted(arguments.event or selected_event_ids())
    missing = [event_id for event_id in event_ids if event_id not in event_paths]
    if missing:
        raise RuntimeError("missing seen event inputs: " + ",".join(missing))
    modes = arguments.mode or ["PRODUCTION_EQUIVALENT", "INSTRUMENTATION_ON"]
    rows = [
        run_event(arguments.harness, event_paths[event_id], mode)
        for mode in modes for event_id in event_ids
    ]
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    with arguments.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"R3_K12_PROFILE {len(event_ids)} events x {len(modes)} modes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
