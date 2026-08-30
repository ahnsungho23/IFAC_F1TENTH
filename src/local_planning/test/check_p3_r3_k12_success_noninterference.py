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
"""Compare integrated production-success results with a detached pre-integration HEAD build."""

from __future__ import annotations

import argparse
import csv
import hashlib
import pathlib
import subprocess
import sys


REPO = pathlib.Path(__file__).resolve().parents[3]
INPUTS = REPO / "planning_study/p3_geometry_conditioned_method_v1/success_control_inputs"
MANIFEST = REPO / "planning_study/p3_geometry_conditioned_method_v1/success_control_manifest.csv"


def run(binary: pathlib.Path, event: pathlib.Path) -> list[list[str]]:
    completed = subprocess.run(
        [str(binary), str(event)], text=True, capture_output=True, check=False)
    if completed.returncode:
        raise RuntimeError(f"{binary.name} {event.stem}: {completed.stderr.strip()}")
    return [line.split("\t") for line in completed.stdout.splitlines()]


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-harness", required=True, type=pathlib.Path)
    parser.add_argument("--integrated-harness", required=True, type=pathlib.Path)
    parser.add_argument("--output", type=pathlib.Path)
    arguments = parser.parse_args()

    with MANIFEST.open(newline="", encoding="utf-8") as stream:
        manifest = list(csv.DictReader(stream))
    rows = []
    for item in manifest:
        event = INPUTS / f"{item['control_event_id']}.event"
        if sha256(event) != item["event_input_sha256"]:
            raise RuntimeError(f"input SHA mismatch: {event}")
        baseline_records = run(arguments.baseline_harness, event)
        integrated_records = run(arguments.integrated_harness, event)
        baseline = next(row for row in baseline_records if row[0] == "BASELINE_SELECTED")
        integrated = next(row for row in integrated_records if row[0] == "INTEGRATED_SELECTED")
        summary = next(row for row in integrated_records if row[0] == "SUMMARY")
        baseline_sequence = [row[1:] for row in baseline_records if row[0] == "BASELINE_SEQUENCE"]
        integrated_sequence = [row[1:] for row in integrated_records if row[0] == "INTEGRATED_SEQUENCE"]
        selected_exact = baseline[2:] == integrated[2:]
        sequence_exact = baseline_sequence == integrated_sequence
        r3_invoked = summary[2] == "1"
        r3_validators = int(summary[8])
        rows.append({
            "event_id": item["control_event_id"],
            "bag": item["bag"],
            "input_sha256": item["event_input_sha256"],
            "baseline_selected_path_digest": baseline[6],
            "integrated_selected_path_digest": integrated[6],
            "selected_result_exact": selected_exact,
            "production_candidate_sequence_exact": sequence_exact,
            "r3_invoked": r3_invoked,
            "extra_r3_validator_calls": r3_validators,
            "production_success_noninterference":
                selected_exact and sequence_exact and not r3_invoked and r3_validators == 0,
        })
    if arguments.output:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        with arguments.output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    passed = sum(row["production_success_noninterference"] for row in rows)
    print(f"R3_K12_SUCCESS_NONINTERFERENCE {passed}/{len(rows)}")
    return 0 if passed == len(rows) else 1


if __name__ == "__main__":
    sys.exit(main())
