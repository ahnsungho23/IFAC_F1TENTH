#!/usr/bin/env python3
"""One-shot immutable FINAL_HOLDOUT evaluation for frozen R3-K12.

This research-only runner is deliberately fail-closed.  It verifies the checkpoint,
opens the frozen holdout manifest once, reconstructs exact evaluator inputs, checks
production digest parity, and evaluates the frozen R3-K12 and Reference Oracle v2.
It does not modify production planner source, parameters, validator, or ranking.
"""

from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import math
import os
import statistics
import subprocess
import sys
import time
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/p3_r3_k12_final_holdout_v1_mpl")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
CORPUS = REPO / "planning_study/p3_mapping_research_corpus_v1"
METHOD_DIR = REPO / "planning_study/p3_geometry_factor_ranking_v2"
REFERENCE_DIR = REPO / "planning_study/p3_reference_oracle_v2"
METHOD_V1_DIR = REPO / "planning_study/p3_geometry_conditioned_method_v1"
CHECKPOINT_DIR = REPO / "planning_study/p3_r3_k12_pre_holdout_checkpoint"
MANIFEST = CORPUS / "dataset_split_manifest.csv"
METHOD_SPEC = METHOD_DIR / "selected_v2_method_spec.json"
METHOD_IMPL = METHOD_DIR / "run_factor_ranking_v2.py"
REFERENCE_IMPL = REFERENCE_DIR / "run_reference_oracle_v2.py"
METHOD_V1_IMPL = METHOD_V1_DIR / "analyze_method.py"
ORACLE_SPEC = REFERENCE_DIR / "oracle_v2_spec.json"
CONTRACT = REFERENCE_DIR / "evaluation_contract.json"
HARNESS = Path("/tmp/p3_oracle_v2_build/p3_family_oracle_harness")
REFERENCE_STREAM = Path("/tmp/p3_oracle_snapshots_v1.tsv")
LOG_ROOT = Path("/tmp/p3_oracle_v2_logs")
WORK = Path("/tmp/p3_r3_k12_final_holdout_v1")
INPUTS = HERE / "inputs"
PLOTS = HERE / "plots"
PREFLIGHT = HERE / "preflight_verification.json"
OPEN_RECORD = HERE / "holdout_open_record.json"

CHECKPOINT_COMMIT = "1fe52ed5ea6c64d1deb2dcb9059e3ca64fa5f83d"
CHECKPOINT_TAG = "p3_r3_k12_pre_holdout"
EXPECTED = {
    "selected_method_spec": "7b861e8c7e23ae168413885ea0dc2d09769046ff48a562700b658e084d116fcc",
    "reference_oracle_v2_spec": "62b63b8ab05d5a73565398141bd05a9db4a102541855fb2e3634d85b18a6bffe",
    "evaluation_contract": "226b1b44a9bea6adf26715658f36ae8e7b2c322e030f363270728f9e044b1dad",
    "dataset_split_manifest": "c57cfe8e57dfca4bb318d30047e8f7215f6994e88a39babfbdbb10ce7637b2f4",
    "method_impl": "2cd87b4c9faf9e9f84c96295f137b4edeed21fafc9abeccf48298b603f075c03",
    "reference_impl": "afe8fe9298be34f1b88ee44f35fd1c833fc11eab55fe05b3177e882d7e8e839a",
    "method_v1_impl": "900a262e49436fa75a1567d49b7823ef4e4cb7f08d313389cd2de26c647c3722",
    "harness": "8b23f2b6df54c2e93794ee087f3ebd7a14f9921544af4f97eef755535e045e7e",
    "reference_stream": "29c724e3244b4f62466b41c8c814ccd3fbe8dd81ac42cfd5d62747cc40b96c14",
}
AUTHORITY_PATHS = {
    "selected_method_spec": METHOD_SPEC,
    "reference_oracle_v2_spec": ORACLE_SPEC,
    "evaluation_contract": CONTRACT,
    "dataset_split_manifest": MANIFEST,
    "method_impl": METHOD_IMPL,
    "reference_impl": REFERENCE_IMPL,
    "method_v1_impl": METHOD_V1_IMPL,
    "harness": HARNESS,
    "reference_stream": REFERENCE_STREAM,
}
SELECTORS = {
    "H3_FIXED_BOUNDED_K24": "H3_FIXED_BOUNDED",
    "H4A_GEOMETRY_TRANSITION_K24": "H4A_GEOMETRY_TRANSITION",
    "H4B_GEOMETRY_LATERAL_TRANSITION_K12": "H4B_GEOMETRY_LATERAL_TRANSITION",
}
EPS = 1.0e-9


def now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=REPO, text=True).strip()


def truth(value) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def number(row: dict | None, key: str, default: float = math.nan) -> float:
    if row is None:
        return default
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def read_jsonl(path: Path):
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_csv(path: Path, rows: list[dict], fields: list[str] | None = None) -> None:
    if fields is None:
        fields = []
        for row in rows:
            for key in row:
                if key not in fields:
                    fields.append(key)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def percentile(values: list[float], q: float) -> float:
    if not values:
        return math.nan
    values = sorted(values)
    position = (len(values) - 1) * q / 100.0
    lo, hi = math.floor(position), math.ceil(position)
    return values[lo] if lo == hi else values[lo] + (values[hi] - values[lo]) * (position - lo)


def wilson(successes: int, total: int, z: float = 1.959963984540054) -> tuple[float, float]:
    if total == 0:
        return math.nan, math.nan
    p = successes / total
    den = 1.0 + z * z / total
    center = (p + z * z / (2.0 * total)) / den
    half = z * math.sqrt(p * (1.0 - p) / total + z * z / (4.0 * total * total)) / den
    return center - half, center + half


def compact_best(best: dict | None) -> dict:
    return best or {}


def production_usable(candidate: dict) -> bool:
    rank = candidate.get("rank_tuple", {})
    return (truth(candidate.get("hard_valid"))
            and not truth(rank.get("exit_reaches_next_obstacle"))
            and float(rank.get("braking_deficit_m", math.inf)) <= EPS)


def fail_closed_preflight() -> dict:
    if OPEN_RECORD.exists():
        raise RuntimeError("ONE_SHOT_VIOLATION: holdout was already opened; rerun prohibited")
    if not PREFLIGHT.is_file():
        raise RuntimeError("missing predeclared preflight_verification.json")
    declaration = json.loads(PREFLIGHT.read_text(encoding="utf-8"))
    if declaration.get("holdout_rows_loaded") != 0:
        raise RuntimeError("preflight declaration does not certify zero holdout access")
    if declaration.get("runner_sha256") != sha256(Path(__file__)):
        raise RuntimeError("runner changed after predeclaration")
    observed = {name: sha256(path) for name, path in AUTHORITY_PATHS.items()}
    mismatch = {name: {"expected": EXPECTED[name], "observed": value}
                for name, value in observed.items() if value != EXPECTED[name]}
    head = git("rev-parse", "HEAD")
    tag_target = git("rev-parse", f"{CHECKPOINT_TAG}^{{commit}}")
    production_diff = git("diff", "--name-only", CHECKPOINT_TAG, "--", "src/local_planning")
    dirty = git("status", "--porcelain")
    outside_output = [line for line in dirty.splitlines()
                      if "planning_study/p3_r3_k12_final_holdout_v1" not in line]
    if mismatch or head != CHECKPOINT_COMMIT or tag_target != CHECKPOINT_COMMIT:
        raise RuntimeError("FROZEN_ARTIFACT_MISMATCH " + json.dumps({
            "hashes": mismatch, "head": head, "tag_target": tag_target}, sort_keys=True))
    if production_diff or outside_output:
        raise RuntimeError("WORKTREE_OR_PRODUCTION_MISMATCH " + json.dumps({
            "production_diff": production_diff, "outside_output": outside_output}))
    method = json.loads(METHOD_SPEC.read_text(encoding="utf-8"))
    if method["method_name"] != "R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12":
        raise RuntimeError("frozen method name mismatch")
    if method["budget"]["k_fully_reconstructed_candidates"] != 12:
        raise RuntimeError("frozen method budget mismatch")
    return {"observed": observed, "head": head, "tag_target": tag_target,
            "production_diff_empty": not production_diff,
            "outside_output_dirty_empty": not outside_output,
            "runner_sha256": declaration["runner_sha256"]}


def open_holdout_once() -> tuple[list[dict], str]:
    opened = now()
    with MANIFEST.open(newline="", encoding="utf-8") as stream:
        rows = [row for row in csv.DictReader(stream)
                if row["dataset_role"] == "FINAL_HOLDOUT_UNSEEN"]
    if len(rows) != 37:
        raise RuntimeError(f"frozen holdout count mismatch: {len(rows)}")
    for index, row in enumerate(rows, 1):
        row["final_event_id"] = f"FHE{index:03d}"
        row["holdout_status"] = "FINAL_HOLDOUT_OPENED_NO_LONGER_UNSEEN"
        row["opened_at_utc"] = opened
    write_json(OPEN_RECORD, {
        "schema_version": "P3_FINAL_HOLDOUT_OPEN_RECORD_1",
        "status": "FINAL_HOLDOUT_OPENED_NO_LONGER_UNSEEN",
        "opened_at_utc": opened,
        "row_count": len(rows),
        "dataset_split_manifest_sha256": sha256(MANIFEST),
        "one_shot_execution": True,
        "rerun_allowed": False,
    })
    write_csv(HERE / "final_holdout_manifest.csv", rows)
    return rows, opened


def selected_inputs(rows: list[dict]) -> list[dict]:
    wanted: dict[str, set[tuple[int, int]]] = defaultdict(set)
    for row in rows:
        wanted[row["bag"]].add((int(row["callback_sequence"]),
                                int(row["evaluation_sequence"])))
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
            indexed[(bag, *key)] = evaluations[key], candidates[key]

    output = []
    id_fields = ("input_snapshot_id", "ego_snapshot_id", "obstacle_snapshot_id",
                 "reference_snapshot_id")
    for row in rows:
        key = (row["bag"], int(row["callback_sequence"]), int(row["evaluation_sequence"]))
        evaluation, candidates = indexed[key]
        expected_ids = tuple(row[name] for name in id_fields)
        if tuple(evaluation[name] for name in id_fields) != expected_ids:
            raise RuntimeError(f"{row['final_event_id']}: evaluation lineage mismatch")
        if any(tuple(candidate[name] for name in id_fields) != expected_ids
               for candidate in candidates):
            raise RuntimeError(f"{row['final_event_id']}: candidate lineage mismatch")
        checks = {
            "candidate_count": (len(candidates), int(row["production_constructed_candidate_count"])),
            "hard_valid_count": (int(evaluation["hard_valid_total_actual"]),
                                 int(row["production_hard_valid_count"])),
            "returned_count": (int(evaluation["returned_candidate_count"]),
                               int(row["production_returned_candidate_count"])),
        }
        mismatch = {name: value for name, value in checks.items() if value[0] != value[1]}
        if mismatch:
            raise RuntimeError(f"{row['final_event_id']}: frozen row mismatch {mismatch}")
        output.append({"row": row, "evaluation": evaluation, "candidates": candidates})
    return output


def reconstruct_inputs(items: list[dict], method_v1) -> tuple[list[dict], list[dict]]:
    refs = method_v1.reference_snapshots()
    INPUTS.mkdir(parents=True, exist_ok=True)
    method_v1.WORK = WORK / "production_parity"
    method_v1.WORK.mkdir(parents=True, exist_ok=True)
    production_rows, prepared = [], []
    for item in items:
        row, evaluation, production = item["row"], item["evaluation"], item["candidates"]
        event_id = row["final_event_id"]
        event_path = INPUTS / f"{event_id}.event"
        method_v1.write_event(event_path, event_id, row["bag"], evaluation,
                              refs[evaluation["reference_snapshot_id"]])
        _, runtime = method_v1.run_harness(
            event_id, event_path, "FINAL_PRODUCTION_PARITY", 0, [])
        output = method_v1.WORK / f"{event_id}.FINAL_PRODUCTION_PARITY.K0.out"
        reconstructed_all = method_v1.records(output, "BASELINE_CONSTRUCTED_CANDIDATE")
        reconstructed_returned = method_v1.records(output, "BASELINE_CANDIDATE")
        constructed_parity = Counter(x["path_digest"] for x in reconstructed_all) == Counter(
            str(x["path_digest"]) for x in production)
        returned_parity = Counter(x["path_digest"] for x in reconstructed_returned) == Counter(
            str(x["path_digest"]) for x in production if truth(x.get("returned_by_policy")))
        if not constructed_parity or not returned_parity:
            raise RuntimeError(f"{event_id}: exact production candidate-digest parity failed")
        hard = [x for x in production if truth(x.get("hard_valid"))]
        usable = [x for x in production if production_usable(x)]
        validators = sum(truth(x.get("validator_executed", True)) for x in production)
        production_rows.append({
            "final_event_id": event_id, "bag": row["bag"],
            "callback_sequence": row["callback_sequence"],
            "evaluation_sequence": row["evaluation_sequence"],
            "input_snapshot_id": row["input_snapshot_id"],
            "ego_snapshot_id": row["ego_snapshot_id"],
            "obstacle_snapshot_id": row["obstacle_snapshot_id"],
            "reference_snapshot_id": row["reference_snapshot_id"],
            "exact_input_lineage": True, "constructed_digest_multiset_parity": True,
            "returned_digest_multiset_parity": True,
            "constructed_candidate_count": len(production),
            "returned_candidate_count": sum(truth(x.get("returned_by_policy")) for x in production),
            "validator_execution_count": validators,
            "hard_valid_count": len(hard), "usable_valid_count": len(usable),
            "hard_success": bool(hard), "usable_success": bool(usable),
            "lifecycle_state": row["lifecycle_state"], "lifecycle_owner": row["lifecycle_owner"],
            "safe_stop": row["safe_stop"], "fallback_reason": row["fallback_reason"],
            "runtime_wall_s": runtime, "event_input_sha256": sha256(event_path),
        })
        prepared.append({
            "dataset_role": "FINAL_HOLDOUT_OPENED_V1", "event_id": event_id,
            "row": {**row, "production_candidate_parameters": json.dumps(production)},
            "lineage": {name: row[name] for name in (
                "bag", "callback_sequence", "evaluation_sequence", "input_snapshot_id",
                "ego_snapshot_id", "obstacle_snapshot_id", "reference_snapshot_id",
                "source_stamp_ns", "source_epoch")},
            "event_path": event_path, "production": production,
        })
        print("PRODUCTION_PARITY", event_id, "PASS", flush=True)
    return production_rows, prepared


def run_r3(items: list[dict], production_by_id: dict[str, dict], v2) -> tuple[list[dict], dict]:
    v2.WORK = WORK / "r3"
    v2.R.WORK = WORK / "r3_context"
    v2.WORK.mkdir(parents=True, exist_ok=True)
    rows, audit = [], {}
    for item in items:
        event_id = item["event_id"]
        production = production_by_id[event_id]
        if production["hard_success"]:
            rows.append({
                "final_event_id": event_id, "bag": item["row"]["bag"],
                "activation": "SKIPPED_PRODUCTION_HARD_VALID",
                "production_first_prevented_change": True,
                "fully_reconstructed_candidate_count": 0,
                "constructed_unique_count": 0, "logical_exact_validator_calls": 0,
                "raw_harness_validator_executions": 0, "hard_valid_count": 0,
                "usable_valid_count": 0, "hard_recovered": False,
                "usable_recovered": False, "runtime_wall_s": 0.0,
            })
            audit[event_id] = {"pool_keys": set(), "selected_keys": set()}
            continue
        contexts = v2.strict_contexts(item["event_path"])
        pool, pool_stats = v2.build_pair_pool(item, contexts)
        ordered = v2.ordered_candidates(pool, "R3_LEXICOGRAPHIC_COVERAGE_RESERVE")
        configs, candidate_rows, runtime = v2.run_ranked_prefix(
            item, "R3_LEXICOGRAPHIC_COVERAGE_RESERVE", ordered, pool)
        result = v2.prefixes(configs, candidate_rows)[12]
        if result["fully_reconstructed_candidate_count"] != 12:
            raise RuntimeError(f"{event_id}: frozen R3 failed to fill K=12")
        if result["logical_exact_validator_calls"] > 12:
            raise RuntimeError(f"{event_id}: R3 logical validator cap exceeded")
        best_hard, best_usable = compact_best(result["best_hard"]), compact_best(result["best_usable"])
        rows.append({
            "final_event_id": event_id, "bag": item["row"]["bag"],
            "callback_sequence": item["row"]["callback_sequence"],
            "evaluation_sequence": item["row"]["evaluation_sequence"],
            "activation": "ACTIVATED_PRODUCTION_FAILURE",
            "production_first_prevented_change": False,
            "raw_factor_pool_count": len(pool),
            "fully_reconstructed_candidate_count": result["fully_reconstructed_candidate_count"],
            "constructed_unique_count": result["constructed_unique_count"],
            "construction_guard_rejects": result["construction_guard_rejects"],
            "duplicate_constructed_paths": result["duplicate_constructed_paths"],
            "logical_exact_validator_calls": result["logical_exact_validator_calls"],
            "raw_harness_validator_executions": result["raw_harness_validator_executions"],
            "hard_valid_count": result["hard_valid_count"],
            "usable_valid_count": result["usable_valid_count"],
            "hard_recovered": result["hard_recovered"],
            "usable_recovered": result["usable_recovered"],
            "best_hard_side": best_hard.get("side", ""),
            "best_hard_d_target": best_hard.get("d_target", ""),
            "best_hard_d_mid": best_hard.get("d_mid", ""),
            "best_hard_entry_scale": best_hard.get("entry_scale", ""),
            "best_hard_exit_scale": best_hard.get("exit_scale", ""),
            "best_hard_path_digest": best_hard.get("path_digest", ""),
            "best_usable_side": best_usable.get("side", ""),
            "best_usable_d_target": best_usable.get("d_target", ""),
            "best_usable_d_mid": best_usable.get("d_mid", ""),
            "best_usable_entry_scale": best_usable.get("entry_scale", ""),
            "best_usable_exit_scale": best_usable.get("exit_scale", ""),
            "best_usable_path_digest": best_usable.get("path_digest", ""),
            "best_hard_json": json.dumps(best_hard, sort_keys=True),
            "best_usable_json": json.dumps(best_usable, sort_keys=True),
            "pool_stats_json": json.dumps(pool_stats, sort_keys=True),
            "runtime_wall_s": runtime,
        })
        audit[event_id] = {
            "pool_keys": {v2.config_key(x) for x in pool},
            "selected_keys": {v2.config_key(x) for x in result["configs"]},
        }
        print("R3_K12", event_id, "usable", int(result["usable_recovered"]), flush=True)
    return rows, audit


def run_oracle(items: list[dict], reference, method_v1) -> tuple[list[dict], dict]:
    reference.WORK = WORK / "oracle"
    reference.CACHE = WORK / "oracle/event_results"
    reference.WORK.mkdir(parents=True, exist_ok=True)
    rows, full = [], {}
    for item in items:
        event_id = item["event_id"]
        result = reference.evaluate_event(item, method_v1, {}, force=True)
        if not result["coverage_complete"] or not result["production_digest_and_validator_parity"]:
            raise RuntimeError(f"{event_id}: oracle coverage or production parity failure")
        full[event_id] = result
        best_hard, best_usable = compact_best(result["best_hard"]), compact_best(result["best_usable"])
        rows.append({
            "final_event_id": event_id, "bag": item["row"]["bag"],
            "callback_sequence": item["row"]["callback_sequence"],
            "evaluation_sequence": item["row"]["evaluation_sequence"],
            "exact_input_lineage": True, "production_digest_and_validator_parity": True,
            "coverage_complete": result["coverage_complete"],
            "declared_request_count": result["declared_request_count"],
            "emitted_candidate_rows": result["emitted_candidate_rows"],
            "constructed_path_count": result["constructed_path_count"],
            "validator_execution_count": result["validator_execution_count"],
            "unique_constructed_path_digest_count": result["unique_constructed_path_digest_count"],
            "hard_valid_count": result["hard_valid_count"],
            "usable_valid_count": result["usable_valid_count"],
            "hard_classification": result["hard_classification"],
            "usable_classification": result["usable_classification"],
            "best_hard_side": best_hard.get("side", ""),
            "best_hard_d_target": best_hard.get("d_target", ""),
            "best_hard_d_mid": best_hard.get("d_mid", ""),
            "best_hard_entry_scale": best_hard.get("entry_scale", ""),
            "best_hard_exit_scale": best_hard.get("exit_scale", ""),
            "best_usable_side": best_usable.get("side", ""),
            "best_usable_d_target": best_usable.get("d_target", ""),
            "best_usable_d_mid": best_usable.get("d_mid", ""),
            "best_usable_entry_scale": best_usable.get("entry_scale", ""),
            "best_usable_exit_scale": best_usable.get("exit_scale", ""),
            "best_hard_json": json.dumps(best_hard, sort_keys=True),
            "best_usable_json": json.dumps(best_usable, sort_keys=True),
            "domain_json": json.dumps(result["domain"], sort_keys=True),
            "first_failure_distribution_json": json.dumps(
                result["first_failure_distribution"], sort_keys=True),
            "runtime_wall_s": result["runtime_wall_s"],
        })
        print("ORACLE_V2", event_id, result["usable_classification"], flush=True)
    return rows, full


def method_result_row(event_id: str, name: str, source: dict, oracle: dict) -> dict:
    if name == "R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12":
        return {
            "final_event_id": event_id, "method": name,
            "hard_success": truth(source["hard_recovered"]),
            "usable_success": truth(source["usable_recovered"]),
            "candidate_count": int(source["fully_reconstructed_candidate_count"]),
            "validator_count": int(source["logical_exact_validator_calls"]),
            "runtime_wall_s": float(source["runtime_wall_s"]),
        }
    selector = SELECTORS[name]
    result = oracle["selector_results"][selector]
    return {
        "final_event_id": event_id, "method": name,
        "hard_success": result["hard_recovered"], "usable_success": result["usable_recovered"],
        "candidate_count": result["selected_count"],
        "validator_count": result["validator_execution_count"],
        "runtime_wall_s": math.nan,
    }


def aggregate(production_rows: list[dict], r3_rows: list[dict], oracle_rows: list[dict],
              full_oracle: dict, audit: dict, opened: str, started: str,
              preflight: dict) -> None:
    prod = {row["final_event_id"]: row for row in production_rows}
    r3 = {row["final_event_id"]: row for row in r3_rows}
    oracle = {row["final_event_id"]: row for row in oracle_rows}
    event_ids = [row["final_event_id"] for row in production_rows]
    method_event_rows = []
    for event_id in event_ids:
        method_event_rows.append({
            "final_event_id": event_id, "method": "PRODUCTION",
            "hard_success": truth(prod[event_id]["hard_success"]),
            "usable_success": truth(prod[event_id]["usable_success"]),
            "candidate_count": int(prod[event_id]["constructed_candidate_count"]),
            "validator_count": int(prod[event_id]["validator_execution_count"]),
            "runtime_wall_s": float(prod[event_id]["runtime_wall_s"]),
        })
        for method_name in SELECTORS:
            method_event_rows.append(method_result_row(
                event_id, method_name, {}, full_oracle[event_id]))
        method_event_rows.append(method_result_row(
            event_id, "R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12", r3[event_id],
            full_oracle[event_id]))
        method_event_rows.append({
            "final_event_id": event_id, "method": "REFERENCE_ORACLE_V2",
            "hard_success": int(oracle[event_id]["hard_valid_count"]) > 0,
            "usable_success": int(oracle[event_id]["usable_valid_count"]) > 0,
            "candidate_count": int(oracle[event_id]["constructed_path_count"]),
            "validator_count": int(oracle[event_id]["validator_execution_count"]),
            "runtime_wall_s": float(oracle[event_id]["runtime_wall_s"]),
        })
    write_csv(HERE / "hard_vs_usable_final.csv", method_event_rows)

    comparisons = []
    r3_usable_set = {event_id for event_id in event_ids if truth(r3[event_id]["usable_recovered"])}
    for method_name in ("PRODUCTION", *SELECTORS.keys(),
                        "R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12", "REFERENCE_ORACLE_V2"):
        rows = [row for row in method_event_rows if row["method"] == method_name]
        hard_set = {row["final_event_id"] for row in rows if truth(row["hard_success"])}
        usable_set = {row["final_event_id"] for row in rows if truth(row["usable_success"])}
        runtimes = [float(row["runtime_wall_s"]) for row in rows
                    if math.isfinite(float(row["runtime_wall_s"]))]
        comparisons.append({
            "method": method_name, "event_count": len(rows),
            "hard_success_count": len(hard_set), "usable_success_count": len(usable_set),
            "hard_success_event_ids": "|".join(sorted(hard_set)),
            "usable_success_event_ids": "|".join(sorted(usable_set)),
            "usable_overlap_with_r3_count": len(usable_set & r3_usable_set),
            "candidate_count_total": sum(int(row["candidate_count"]) for row in rows),
            "validator_count_total": sum(int(row["validator_count"]) for row in rows),
            "runtime_total_s": sum(runtimes) if runtimes else math.nan,
        })
    write_csv(HERE / "final_method_comparison.csv", comparisons)

    production_failures = [event_id for event_id in event_ids if not truth(prod[event_id]["hard_success"])]
    usable_feasible = [event_id for event_id in production_failures
                       if int(oracle[event_id]["usable_valid_count"]) > 0]
    hard_feasible = [event_id for event_id in production_failures
                     if int(oracle[event_id]["hard_valid_count"]) > 0]
    usable_recovered = [event_id for event_id in usable_feasible
                        if truth(r3[event_id]["usable_recovered"])]
    hard_recovered = [event_id for event_id in hard_feasible
                      if truth(r3[event_id]["hard_recovered"])]
    ulo, uhi = wilson(len(usable_recovered), len(usable_feasible))
    hlo, hhi = wilson(len(hard_recovered), len(hard_feasible))
    bags = {prod[event_id]["bag"] for event_id in usable_recovered}
    refs = {next(item["lineage"]["reference_snapshot_id"]
                 for item in full_oracle.values() if item["event_id"] == event_id)
            for event_id in usable_recovered}
    summary = [{
        "holdout_event_count": len(event_ids), "infrastructure_failure_count": 0,
        "conclusive_event_count": len(event_ids),
        "production_hard_failure_count": len(production_failures),
        "oracle_v2_usable_feasible_production_failure_count": len(usable_feasible),
        "r3_k12_usable_recovery_count": len(usable_recovered),
        "primary_usable_recovery_rate": len(usable_recovered) / len(usable_feasible)
        if usable_feasible else math.nan,
        "primary_wilson_95_low": ulo, "primary_wilson_95_high": uhi,
        "oracle_v2_hard_feasible_production_failure_count": len(hard_feasible),
        "r3_k12_hard_recovery_count": len(hard_recovered),
        "secondary_hard_recovery_rate": len(hard_recovered) / len(hard_feasible)
        if hard_feasible else math.nan,
        "secondary_hard_wilson_95_low": hlo, "secondary_hard_wilson_95_high": hhi,
        "oracle_v2_usable_infeasible_count": len(production_failures) - len(usable_feasible),
        "seen_usable_recovery": "32/36", "seen_usable_rate": 32 / 36,
        "holdout_minus_seen_usable_rate_pp":
            100.0 * ((len(usable_recovered) / len(usable_feasible)) - 32 / 36)
            if usable_feasible else math.nan,
        "seen_hard_recovery": "36/48", "seen_hard_rate": 36 / 48,
        "holdout_minus_seen_hard_rate_pp":
            100.0 * ((len(hard_recovered) / len(hard_feasible)) - 36 / 48)
            if hard_feasible else math.nan,
        "r3_usable_success_bag_count": len(bags),
        "r3_usable_success_reference_domain_count": len(refs),
    }]
    write_csv(HERE / "final_recovery_summary.csv", summary)

    by_domain = []
    manifest = {row["final_event_id"]: row for row in csv.DictReader(
        (HERE / "final_holdout_manifest.csv").open(newline="", encoding="utf-8"))}
    for field in ("bag", "reference_snapshot_id"):
        groups: dict[str, list[str]] = defaultdict(list)
        for event_id in event_ids:
            groups[manifest[event_id][field]].append(event_id)
        for domain, ids in sorted(groups.items()):
            denom = [event_id for event_id in ids if event_id in usable_feasible]
            numer = [event_id for event_id in denom if event_id in usable_recovered]
            lo, hi = wilson(len(numer), len(denom))
            by_domain.append({
                "domain_type": field, "domain": domain, "event_count": len(ids),
                "oracle_usable_feasible_count": len(denom),
                "r3_usable_recovery_count": len(numer),
                "usable_recovery_rate": len(numer) / len(denom) if denom else math.nan,
                "wilson_95_low": lo, "wilson_95_high": hi,
                "recovered_event_ids": "|".join(numer),
            })
    write_csv(HERE / "recovery_by_domain.csv", by_domain)

    failure_rows = []
    for event_id in event_ids:
        best = full_oracle[event_id].get("best_usable")
        if truth(prod[event_id]["hard_success"]):
            label = "PRODUCTION_ALREADY_HARD_VALID"
        elif best is None:
            label = "NO_USABLE_P3_IN_REFERENCE_ORACLE_V2_DOMAIN"
        elif truth(r3[event_id]["usable_recovered"]):
            label = "R3_K12_USABLE_RECOVERY"
        else:
            key = (best["side"], float(best["d_target"]).hex(), float(best["d_mid"]).hex(),
                   float(best["entry_scale"]).hex(), float(best["exit_scale"]).hex())
            if key in audit[event_id]["pool_keys"]:
                label = "RANKING_OR_K12_BUDGET_MISS"
            else:
                production_candidates = next(
                    item["production"] for item in full_oracle.values()
                    if item["event_id"] == event_id) if False else []
                label = "FACTOR_SPACE_COVERAGE_MISS"
        failure_rows.append({
            "final_event_id": event_id, "bag": prod[event_id]["bag"],
            "production_hard_failure": not truth(prod[event_id]["hard_success"]),
            "oracle_usable_feasible": best is not None,
            "r3_usable_recovered": truth(r3[event_id]["usable_recovered"]),
            "descriptive_taxonomy": label,
            "oracle_best_usable_json": json.dumps(best or {}, sort_keys=True),
            "r3_best_usable_json": r3[event_id].get("best_usable_json", "{}"),
        })
    write_csv(HERE / "failure_analysis.csv", failure_rows)

    budget_rows = []
    for comparison in comparisons:
        name = comparison["method"]
        rows = [row for row in method_event_rows if row["method"] == name]
        candidate_values = [int(row["candidate_count"]) for row in rows]
        validator_values = [int(row["validator_count"]) for row in rows]
        runtime_values = [float(row["runtime_wall_s"]) for row in rows
                          if math.isfinite(float(row["runtime_wall_s"]))]
        budget_rows.append({
            "method": name, "event_count": len(rows),
            "candidate_mean": statistics.mean(candidate_values),
            "candidate_max": max(candidate_values),
            "validator_mean": statistics.mean(validator_values),
            "validator_max": max(validator_values),
            "runtime_p50_s": percentile(runtime_values, 50),
            "runtime_p95_s": percentile(runtime_values, 95),
            "runtime_p99_s": percentile(runtime_values, 99),
            "runtime_max_s": max(runtime_values) if runtime_values else math.nan,
            "runtime_scope": ("standalone exact oracle wall time" if name == "REFERENCE_ORACLE_V2"
                              else "standalone research harness wall time" if name in {
                                  "PRODUCTION", "R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12"}
                              else "not separately timed; embedded in oracle audit"),
        })
    r3_active = [row for row in r3_rows if row["activation"] == "ACTIVATED_PRODUCTION_FAILURE"]
    budget_rows.append({
        "method": "R3_K12_OFFLINE_AUDIT_OVERHEAD_NOT_CONTRACT_BUDGET",
        "event_count": len(r3_active), "candidate_mean": 12, "candidate_max": 12,
        "validator_mean": statistics.mean(int(row["raw_harness_validator_executions"])
                                             for row in r3_active) if r3_active else 0,
        "validator_max": max((int(row["raw_harness_validator_executions"])
                              for row in r3_active), default=0),
        "runtime_p50_s": math.nan, "runtime_p95_s": math.nan,
        "runtime_p99_s": math.nan, "runtime_max_s": math.nan,
        "runtime_scope": "audit implementation evaluates ranking streams; not deployable K12 budget",
    })
    write_csv(HERE / "computation_budget.csv", budget_rows)

    make_plots(comparisons, summary[0])
    ended = now()
    write_readme(summary[0], comparisons, failure_rows, opened, ended)
    write_json(HERE / "execution_manifest.json", {
        "schema_version": "P3_R3_K12_FINAL_HOLDOUT_EXECUTION_1",
        "status": "FINAL_HOLDOUT_OPENED_AND_EVALUATED_NO_LONGER_UNSEEN",
        "checkpoint_commit": CHECKPOINT_COMMIT, "checkpoint_tag": CHECKPOINT_TAG,
        "selected_method": "R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12",
        "started_at_utc": started, "holdout_opened_at_utc": opened,
        "ended_at_utc": ended, "holdout_row_count": len(event_ids),
        "one_shot_execution": True, "rerun_performed": False,
        "method_redesign_or_tuning_performed": False,
        "production_planner_modified": False, "commit_or_push_performed": False,
        "preflight": preflight, "authority_sha256_before": EXPECTED,
        "authority_sha256_after": {name: sha256(path) for name, path in AUTHORITY_PATHS.items()},
    })
    lines = [f"{sha256(path)}  {path.relative_to(REPO) if path.is_relative_to(REPO) else path}"
             for path in AUTHORITY_PATHS.values()]
    lines += [f"{sha256(Path(__file__))}  {Path(__file__).relative_to(REPO)}",
              f"{sha256(PREFLIGHT)}  {PREFLIGHT.relative_to(REPO)}"]
    (HERE / "frozen_artifacts.sha256").write_text("\n".join(lines) + "\n", encoding="utf-8")


def make_plots(comparisons: list[dict], summary: dict) -> None:
    PLOTS.mkdir(parents=True, exist_ok=True)
    methods = [row for row in comparisons if row["method"] not in {"PRODUCTION", "REFERENCE_ORACLE_V2"}]
    labels = [row["method"].replace("_GEOMETRY", "\nGEOMETRY").replace("_K", " K") for row in methods]
    x = range(len(methods))
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.bar([i - 0.18 for i in x], [row["hard_success_count"] for row in methods],
           width=0.36, label="hard")
    ax.bar([i + 0.18 for i in x], [row["usable_success_count"] for row in methods],
           width=0.36, label="usable")
    ax.set_xticks(list(x), labels, fontsize=8)
    ax.set_ylabel("Recovered holdout episodes")
    ax.legend(); fig.tight_layout()
    fig.savefig(PLOTS / "method_hard_vs_usable_recovery.png", dpi=160); plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 5))
    seen, holdout = 32 / 36, float(summary["primary_usable_recovery_rate"])
    lo, hi = float(summary["primary_wilson_95_low"]), float(summary["primary_wilson_95_high"])
    ax.bar([0, 1], [seen, holdout], color=["#777777", "#2878B5"])
    ax.errorbar([1], [holdout], yerr=[[holdout - lo], [hi - holdout]], fmt="none", color="black")
    ax.set_xticks([0, 1], ["Combined seen\n32/36", "Final holdout"])
    ax.set_ylim(0, 1.05); ax.set_ylabel("Usable recovery rate")
    fig.tight_layout(); fig.savefig(PLOTS / "seen_vs_final_usable_recovery.png", dpi=160); plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.bar([row["method"].replace("_", "\n", 1) for row in comparisons],
           [row["validator_count_total"] for row in comparisons])
    ax.set_ylabel("Total exact validator executions"); ax.tick_params(axis="x", labelsize=7)
    fig.tight_layout(); fig.savefig(PLOTS / "validator_compute_comparison.png", dpi=160); plt.close(fig)


def write_readme(summary: dict, comparisons: list[dict], failures: list[dict],
                 opened: str, ended: str) -> None:
    comparison = {row["method"]: row for row in comparisons}
    taxonomy = Counter(row["descriptive_taxonomy"] for row in failures)
    usable_n = int(summary["r3_k12_usable_recovery_count"])
    usable_d = int(summary["oracle_v2_usable_feasible_production_failure_count"])
    hard_n = int(summary["r3_k12_hard_recovery_count"])
    hard_d = int(summary["oracle_v2_hard_feasible_production_failure_count"])
    text = f"""# FINAL HOLDOUT — immutable R3-K12 evaluation v1

상태: `FINAL_HOLDOUT_OPENED_AND_EVALUATED_NO_LONGER_UNSEEN`. 동결 checkpoint
`{CHECKPOINT_COMMIT}` (`{CHECKPOINT_TAG}`)에서 사전 선언된 one-shot runner만 사용했다.
holdout open: `{opened}`, evaluation end: `{ended}`. Production planner, validator,
R3 ranking, parameter, lifecycle는 수정하지 않았고 commit/push도 하지 않았다.

## Primary result

Reference Oracle v2에서 usable-feasible인 production failure {usable_d}개 중 R3-K12가
{usable_n}개를 usable recovery했다. 비율은 `{float(summary['primary_usable_recovery_rate']):.6f}`,
Wilson 95% CI는 `[{float(summary['primary_wilson_95_low']):.6f},
{float(summary['primary_wilson_95_high']):.6f}]`이다. Hard recovery는 {hard_n}/{hard_d}
(`{float(summary['secondary_hard_recovery_rate']):.6f}`)이다. Seen 결과는 usable 32/36,
hard 36/48이었으며 holdout 차이는 각각
`{float(summary['holdout_minus_seen_usable_rate_pp']):+.3f} pp`,
`{float(summary['holdout_minus_seen_hard_rate_pp']):+.3f} pp`이다.

## Frozen comparison

| Method | Hard | Usable | Candidates | Validators |
|---|---:|---:|---:|---:|
"""
    for name in ("H3_FIXED_BOUNDED_K24", "H4A_GEOMETRY_TRANSITION_K24",
                 "H4B_GEOMETRY_LATERAL_TRANSITION_K12",
                 "R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12"):
        row = comparison[name]
        text += (f"| {name} | {row['hard_success_count']} | {row['usable_success_count']} | "
                 f"{row['candidate_count_total']} | {row['validator_count_total']} |\n")
    text += f"""

R3의 contract budget은 episode당 fully reconstructed candidate 최대 12, path-digest
dedup 뒤 logical exact-validator 최대 12이다. `raw_harness_validator_executions`는 두 rank
stream을 검증하기 위한 offline 연구 audit overhead이며 deployable K=12 예산과 구분했다.

## Required conclusions

1. **R3-K12 final usable recovery:** {usable_n}/{usable_d}, Wilson CI는 위와 같다.
2. **Seen 대비 유지:** 절대 차이는 `{float(summary['holdout_minus_seen_usable_rate_pp']):+.3f} pp`이다. 이는 기술통계이며 별도 허용폭을 사후 정의하지 않았다.
3. **여러 bag/input domain:** usable 성공은 {summary['r3_usable_success_bag_count']}개 bag, {summary['r3_usable_success_reference_domain_count']}개 reference domain에 걸친다.
4. **H3/H4-A/H4-B 비교:** `final_method_comparison.csv`의 exact 동일-event 결과를 따른다.
5. **주요 실패 메커니즘:** `{json.dumps(dict(taxonomy), sort_keys=True)}`. 이는 사전 동결 feature space와 K=12 rank 결과에 대한 기술적 분류다.
6. **Probe/root 관련 잔여 실패:** 이 평가만으로 설계 의도를 추측하지 않는다. `FACTOR_SPACE_COVERAGE_MISS`에는 probe/root 좌표 부재 가능성이 포함되지만 원인 확정 라벨은 아니다.
7. **계산 예산:** K=12 및 logical validator cap은 지켜졌다. Oracle은 상한 평가용 offline finite-domain 계산이므로 production budget과 직접 비교하지 않는다.
8. **새 방법을 정당화하는가:** 이 문서는 동결 결과만 보고하며 새 방법/재설계를 제안하거나 튜닝하지 않는다.
9. **Production integration:** 이 holdout만으로 즉시 통합 결정을 자동 승인하지 않는다. 안전·runtime·독립 재현성 검토가 별도 필요하다.
10. **Holdout 상태:** 37개 행은 이제 unseen이 아니며 같은 결과에 대한 재튜닝·재실행은 금지한다.

Exact lineage, production constructed/returned digest multiset parity, Oracle coverage와
production validator parity는 전 event에서 fail-closed로 검증했다. 개별 수치와 domain,
failure taxonomy, 계산량은 동명 CSV에 기록했다.
"""
    (HERE / "README.md").write_text(text, encoding="utf-8")


def main() -> None:
    started = now()
    preflight = fail_closed_preflight()
    rows, opened = open_holdout_once()
    print("FINAL_HOLDOUT_OPENED", len(rows), opened, flush=True)
    items = selected_inputs(rows)
    method_v1 = load_module("final_holdout_method_v1", METHOD_V1_IMPL)
    v2 = load_module("final_holdout_r3", METHOD_IMPL)
    reference = load_module("final_holdout_reference", REFERENCE_IMPL)
    production_rows, prepared = reconstruct_inputs(items, method_v1)
    write_csv(HERE / "production_results.csv", production_rows)
    production_by_id = {row["final_event_id"]: row for row in production_rows}
    r3_rows, audit = run_r3(prepared, production_by_id, v2)
    write_csv(HERE / "r3_k12_results.csv", r3_rows)
    oracle_rows, full_oracle = run_oracle(prepared, reference, method_v1)
    write_csv(HERE / "oracle_v2_results.csv", oracle_rows)
    aggregate(production_rows, r3_rows, oracle_rows, full_oracle, audit,
              opened, started, preflight)
    print("FINAL_HOLDOUT_EVALUATION_COMPLETE", now(), flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        write_json(HERE / "execution_failure.json", {
            "schema_version": "P3_R3_K12_FINAL_HOLDOUT_FAILURE_1",
            "failed_at_utc": now(), "error_type": type(exc).__name__,
            "error": str(exc), "holdout_open_record_exists": OPEN_RECORD.exists(),
            "holdout_status": ("NO_LONGER_UNSEEN" if OPEN_RECORD.exists() else "NOT_OPENED"),
            "rerun_prohibited_if_opened": OPEN_RECORD.exists(),
        })
        raise
