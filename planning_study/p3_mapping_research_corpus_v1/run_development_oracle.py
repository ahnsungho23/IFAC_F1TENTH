#!/usr/bin/env python3
"""Run the unchanged pilot-v2 P3 oracle on the pre-frozen DEVELOPMENT subset only."""

from __future__ import annotations

import csv
import hashlib
import json
import shutil
from collections import defaultdict
from pathlib import Path
from types import ModuleType


HERE = Path(__file__).resolve().parent
MANIFEST = HERE / "development_oracle_manifest.csv"
FREEZE = HERE / "split_freeze.json"
PILOT_RUNNER = Path("/tmp/run_p3_oracle_v2.py")
LOG_ROOT = Path("/tmp/p3_oracle_v2_logs")
WORK = Path("/tmp/p3_mapping_research_corpus_v1_work")
RAW = HERE / "raw_oracle"


def sha256(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def read_jsonl(path: Path):
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def load_runner() -> ModuleType:
    source = PILOT_RUNNER.read_text(encoding="utf-8")
    source = source.replace(
        'selected = list(csv.DictReader(stream))[:9]',
        'selected = list(csv.DictReader(stream))',
    ).replace('f"V2E{index:02d}"', 'f"DVE{index:03d}"')
    module = ModuleType("frozen_pilot_v2_runner")
    module.__file__ = str(PILOT_RUNNER)
    exec(compile(source, str(PILOT_RUNNER), "exec"), module.__dict__)
    return module


def selected_inputs_cached() -> list[dict]:
    with MANIFEST.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    wanted: dict[str, set[tuple[int, int]]] = defaultdict(set)
    for row in rows:
        wanted[row["bag"]].add((int(row["callback_sequence"]), int(row["evaluation_sequence"])))
    indexed = {}
    for bag, keys in wanted.items():
        run = LOG_ROOT / f"replay_v2_{bag.replace('-', '_')}"
        evaluations = {}
        candidates: dict[tuple[int, int], list[dict]] = defaultdict(list)
        for item in read_jsonl(run / "evaluation_events.jsonl"):
            key = (int(item["callback_sequence"]), int(item["evaluation_sequence"]))
            if key in keys and item["clearance_pass"] == "STRICT":
                if key in evaluations:
                    raise RuntimeError(f"duplicate strict evaluation: {bag} {key}")
                evaluations[key] = item
        for item in read_jsonl(run / "candidate_events.jsonl"):
            key = (int(item["callback_sequence"]), int(item["evaluation_sequence"]))
            if key in keys and item["clearance_pass"] == "STRICT":
                candidates[key].append(item)
        for key in keys:
            if key not in evaluations:
                raise RuntimeError(f"missing strict evaluation: {bag} {key}")
            indexed[(bag, *key)] = (evaluations[key], candidates[key])
    output = []
    for row in rows:
        key = (row["bag"], int(row["callback_sequence"]), int(row["evaluation_sequence"]))
        evaluation, candidates = indexed[key]
        expected_ids = (row["input_snapshot_id"], row["ego_snapshot_id"],
                        row["obstacle_snapshot_id"], row["reference_snapshot_id"])
        actual_ids = tuple(evaluation[name] for name in
                           ("input_snapshot_id", "ego_snapshot_id", "obstacle_snapshot_id",
                            "reference_snapshot_id"))
        if actual_ids != expected_ids:
            raise RuntimeError(f"snapshot lineage mismatch: {row['development_event_id']}")
        output.append({"row": row, "evaluation": evaluation, "candidates": candidates})
    return output


def main() -> None:
    freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
    if sha256(MANIFEST) != freeze["development_oracle_manifest_sha256"]:
        raise RuntimeError("development manifest changed after split freeze")
    module = load_runner()
    module.PREP = MANIFEST
    module.WORK = WORK
    module.OUT = RAW
    module.selected_inputs = selected_inputs_cached
    module.main()
    inputs = RAW / "inputs"
    inputs.mkdir(parents=True, exist_ok=True)
    for event in sorted(WORK.glob("DVE*.event")):
        shutil.copyfile(event, inputs / event.name)
    (RAW / "execution_provenance.json").write_text(json.dumps({
        "dataset_split_manifest_sha256": freeze["dataset_split_manifest_sha256"],
        "development_oracle_manifest_sha256": freeze["development_oracle_manifest_sha256"],
        "pilot_v2_runner_sha256": sha256(PILOT_RUNNER),
        "oracle_harness_sha256": sha256(module.HARNESS),
        "oracle_event_count": len(selected_inputs_cached()),
        "input_policy": "DEVELOPMENT oracle_selected only; VALIDATION and FINAL_HOLDOUT not loaded",
        "search_policy": "exact deterministic pilot-v2 coarse 0.05 m plus 30-seed local refine 0.01 m",
    }, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
