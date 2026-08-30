#!/usr/bin/env python3
"""First frozen validation of H4A_GEOMETRY_TRANSITION_K24.

The freeze gate runs before the VALIDATION_UNSEEN rows are loaded.  The loader stops
after the 37th contiguous validation row, so FINAL_HOLDOUT_UNSEEN rows are never parsed.
The frozen DEVELOPMENT method module and pilot-v2 oracle runner are reused without
changing selector thresholds, formulas, P3 reconstruction, validator, or ranking.
"""

from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import math
import shutil
import subprocess
import sys
import time
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from types import ModuleType

import numpy as np


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
CORPUS = REPO / "planning_study/p3_mapping_research_corpus_v1"
METHOD_DIR = REPO / "planning_study/p3_geometry_conditioned_method_v1"
MANIFEST = CORPUS / "dataset_split_manifest.csv"
SPLIT_FREEZE = CORPUS / "split_freeze.json"
METHOD_SPEC = METHOD_DIR / "method_spec.json"
SELECTED_SPEC = METHOD_DIR / "selected_prototype_spec.json"
METHOD_IMPL = METHOD_DIR / "analyze_method.py"
ORACLE_RUNNER = Path("/tmp/run_p3_oracle_v2.py")
HARNESS = Path("/tmp/p3_oracle_v2_build/p3_family_oracle_harness")
REFERENCE_STREAM = Path("/tmp/p3_oracle_snapshots_v1.tsv")
LOG_ROOT = Path("/tmp/p3_oracle_v2_logs")
WORK = Path("/tmp/p3_geometry_conditioned_validation_v1")
ORACLE_WORK = WORK / "oracle"
METHOD_WORK = WORK / "method"
RAW_ORACLE = HERE / "raw_oracle"
PLOTS = HERE / "plots"

EXPECTED = {
    "method_spec_sha256": "6f3f5a66fe901ae1121c6e9997332e09a1e0421df09a7da7c1673110e9a69f91",
    "selected_prototype_spec_sha256": "7e6aad0541b87b1568a82b27f03d2087f31b2c059af4c5380effd25d0a8bd018",
    "dataset_split_manifest_sha256": "c57cfe8e57dfca4bb318d30047e8f7215f6994e88a39babfbdbb10ce7637b2f4",
    "split_freeze_sha256": "eade6b9dc1cb6603e873303fa04599f98672eed74e2cff123f3968a21c420e02",
    "method_impl_sha256": "900a262e49436fa75a1567d49b7823ef4e4cb7f08d313389cd2de26c647c3722",
    "oracle_runner_sha256": "ed0d8a54cd8d3e3452c78f336c378f384e60a74f1d641ffd6f05208acdc51198",
    "harness_sha256": "8b23f2b6df54c2e93794ee087f3ebd7a14f9921544af4f97eef755535e045e7e",
    "repository_head": "80ae205fd470f537bd1e5449fb906c5548f6653a",
    "validation_count": 37,
}

FROZEN_SELECTORS = {
    "H3_FIXED_BOUNDED": 24,
    "H4A_GEOMETRY_TRANSITION": 24,
    "H4B_GEOMETRY_LATERAL_TRANSITION": 12,
}


def sha256(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=REPO, text=True).strip()


def write_csv(path: Path, rows: list[dict]) -> None:
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def truth(value) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def number(row: dict | None, key: str, default: float = math.nan) -> float:
    if row is None:
        return default
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def percentile(values: list[float], q: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=float), q)) if values else math.nan


def wilson_interval(successes: int, total: int, z: float = 1.959963984540054) -> tuple[float, float]:
    if total == 0:
        return math.nan, math.nan
    estimate = successes / total
    denominator = 1.0 + z * z / total
    center = (estimate + z * z / (2.0 * total)) / denominator
    half = z * math.sqrt(estimate * (1.0 - estimate) / total + z * z / (4.0 * total * total))
    half /= denominator
    return center - half, center + half


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def freeze_gate() -> dict:
    required = [METHOD_SPEC, SELECTED_SPEC, METHOD_IMPL, MANIFEST, SPLIT_FREEZE,
                ORACLE_RUNNER, HARNESS, REFERENCE_STREAM]
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise RuntimeError(f"freeze gate missing files: {missing}")

    observed = {
        "method_spec_sha256": sha256(METHOD_SPEC),
        "selected_prototype_spec_sha256": sha256(SELECTED_SPEC),
        "dataset_split_manifest_sha256": sha256(MANIFEST),
        "split_freeze_sha256": sha256(SPLIT_FREEZE),
        "method_impl_sha256": sha256(METHOD_IMPL),
        "oracle_runner_sha256": sha256(ORACLE_RUNNER),
        "harness_sha256": sha256(HARNESS),
        "repository_head": git("rev-parse", "HEAD"),
    }
    expected_hashes = {key: value for key, value in EXPECTED.items()
                       if key != "validation_count"}
    mismatch = {key: {"expected": expected_hashes[key], "observed": observed[key]}
                for key in expected_hashes if observed[key] != expected_hashes[key]}
    if mismatch:
        raise RuntimeError(f"FROZEN_ARTIFACT_MISMATCH: {json.dumps(mismatch, sort_keys=True)}")

    split = json.loads(SPLIT_FREEZE.read_text(encoding="utf-8"))
    selected = json.loads(SELECTED_SPEC.read_text(encoding="utf-8"))
    if split["dataset_split_manifest_sha256"] != observed["dataset_split_manifest_sha256"]:
        raise RuntimeError("split-freeze manifest SHA mismatch")
    if split["counts"]["VALIDATION_UNSEEN"] != EXPECTED["validation_count"]:
        raise RuntimeError("frozen validation count mismatch")
    if selected["primary"]["name"] != "H4A_GEOMETRY_TRANSITION_K24":
        raise RuntimeError("frozen primary name mismatch")
    if selected["primary"]["candidate_budget_k"] != 24:
        raise RuntimeError("frozen primary K mismatch")
    if selected["method_spec_sha256"] != observed["method_spec_sha256"]:
        raise RuntimeError("selected prototype method-spec SHA mismatch")
    if selected["exact_validator_harness_sha256"] != observed["harness_sha256"]:
        raise RuntimeError("selected prototype harness SHA mismatch")

    source_paths = [
        REPO / "src/local_planning/src/p3_shadow.cpp",
        REPO / "src/local_planning/include/local_planning/p3_analytic_solver.hpp",
        REPO / "src/local_planning/include/local_planning/candidate_rank.hpp",
        REPO / "src/local_planning/config/local_planning.yaml",
        REPO / "planning_study/p3_mapping_miss_diagnosis_v1/analyze_mapping_miss.py",
        REFERENCE_STREAM,
    ]
    source_hashes = {str(path.relative_to(REPO)) if path.is_relative_to(REPO) else str(path):
                     sha256(path) for path in source_paths}
    production_diff = git("diff", "--name-only", "--", "src/local_planning")
    verification = {
        "schema": "p3_geometry_conditioned_validation_freeze_verification_v1",
        "verified_at_utc": datetime.now(timezone.utc).isoformat(),
        "freeze_gate_passed": True,
        "expected": EXPECTED,
        "observed": observed,
        "split_manifest_authority": {
            "split_seed": split["split_seed"],
            "split_algorithm_version": split["split_algorithm_version"],
            "counts": split["counts"],
            "outcome_fields_consulted_for_split": split["outcome_fields_consulted_for_non_pilot_split"],
        },
        "frozen_primary": selected["primary"],
        "source_file_sha256": source_hashes,
        "production_source_worktree_diff_files": production_diff.splitlines() if production_diff else [],
        "production_source_worktree_clean": not bool(production_diff),
        "validation_rows_loaded_after_gate": False,
        "final_holdout_rows_loaded": 0,
    }
    HERE.mkdir(parents=True, exist_ok=True)
    (HERE / "freeze_verification.json").write_text(
        json.dumps(verification, indent=2) + "\n", encoding="utf-8")
    return verification


def validation_rows_only() -> list[dict]:
    """Load exactly the contiguous 37 validation rows and stop before holdout."""
    rows = []
    started = False
    with MANIFEST.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if not started:
                if row["dataset_role"] != "VALIDATION_UNSEEN":
                    continue
                started = True
            if row["dataset_role"] != "VALIDATION_UNSEEN":
                raise RuntimeError("validation rows were not contiguous in frozen manifest")
            rows.append(row)
            if len(rows) == EXPECTED["validation_count"]:
                break
    if len(rows) != EXPECTED["validation_count"]:
        raise RuntimeError(f"expected 37 validation rows, got {len(rows)}")
    for index, row in enumerate(rows, 1):
        row["validation_event_id"] = f"VUE{index:03d}"
    return rows


def read_jsonl(path: Path):
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def selected_inputs(rows: list[dict]) -> list[dict]:
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
        evaluation, candidate_rows = indexed[key]
        id_fields = ("input_snapshot_id", "ego_snapshot_id", "obstacle_snapshot_id",
                     "reference_snapshot_id")
        expected_ids = tuple(row[name] for name in id_fields)
        actual_ids = tuple(evaluation[name] for name in id_fields)
        if actual_ids != expected_ids:
            raise RuntimeError(f"snapshot lineage mismatch: {row['validation_event_id']}")
        if any(tuple(candidate[name] for name in id_fields) != expected_ids
               for candidate in candidate_rows):
            raise RuntimeError(f"candidate lineage mismatch: {row['validation_event_id']}")
        if len(candidate_rows) != int(row["production_constructed_candidate_count"]):
            raise RuntimeError(f"candidate count mismatch: {row['validation_event_id']}")
        if int(evaluation["hard_valid_total_actual"]) != int(row["production_hard_valid_count"]):
            raise RuntimeError(f"hard-valid count mismatch: {row['validation_event_id']}")
        output.append({"row": row, "evaluation": evaluation, "candidates": candidate_rows})
    return output


def load_oracle_runner() -> ModuleType:
    source = ORACLE_RUNNER.read_text(encoding="utf-8")
    source = source.replace('f"V2E{index:02d}"', 'f"VUE{index:03d}"')
    source = source.replace(
        '"LOCALIZATION_STRESS_CANDIDATE" if index == 8 else "PRIMARY_REAL_BAG"',
        '"VALIDATION_UNSEEN"',
    )
    module = ModuleType("frozen_validation_oracle_runner")
    module.__file__ = str(ORACLE_RUNNER)
    exec(compile(source, str(ORACLE_RUNNER), "exec"), module.__dict__)
    return module


def run_oracle(items: list[dict]) -> ModuleType:
    module = load_oracle_runner()
    module.WORK = ORACLE_WORK
    module.OUT = RAW_ORACLE
    module.selected_inputs = lambda: items
    module.main()
    input_dir = RAW_ORACLE / "inputs"
    input_dir.mkdir(parents=True, exist_ok=True)
    for event in sorted(ORACLE_WORK.glob("VUE*.event")):
        shutil.copyfile(event, input_dir / event.name)
    return module


def run_method(items: list[dict], oracle_summary: list[dict], method) -> tuple[list[dict], list[dict], list[dict]]:
    oracle_by_event = {row["event_id"]: row for row in oracle_summary}
    results = []
    selected_audit = []
    geometry_rows = []
    METHOD_WORK.mkdir(parents=True, exist_ok=True)
    for item in items:
        event_id = item["row"]["validation_event_id"]
        event_path = RAW_ORACLE / "inputs" / f"{event_id}.event"
        event = method.G.parse_event(event_path)
        production = item["candidates"]
        pool, features = method.build_pool(event_id, event, production)
        for feature in features:
            feature["dataset_role"] = "VALIDATION_UNSEEN"
        geometry_rows.extend(features)
        production_hard_valid = int(item["evaluation"]["hard_valid_total_actual"])
        oracle = oracle_by_event[event_id]
        for selector, budget in FROZEN_SELECTORS.items():
            if production_hard_valid > 0:
                results.append({
                    "validation_event_id": event_id, "selector": selector, "budget_k": budget,
                    "activation": "SKIPPED_PRODUCTION_HARD_VALID",
                    "production_first_prevented_change": True,
                    "selected_exact_tuple_count": 0, "constructed_path_count": 0,
                    "construction_guard_reject_count": 0,
                    "raw_harness_validator_execution_count": 0,
                    "duplicate_path_count": 0, "deduplicated_validator_call_count": 0,
                    "hard_valid_candidate_count": 0, "recovered": False,
                    "unsafe_selected_count": 0, "runtime_wall_s": 0.0,
                })
                continue

            ordered = sorted(pool, key=lambda candidate: method.selector_score(candidate, selector))
            selected = ordered[:budget]
            request = METHOD_WORK / f"{event_id}.{selector}.K{budget}.requests"
            output = METHOD_WORK / f"{event_id}.{selector}.K{budget}.out"
            method.write_requests(request, selector, selected)
            started = time.perf_counter()
            with output.open("w", encoding="utf-8", newline="\n") as stream:
                subprocess.run([str(HARNESS), str(event_path), str(request)], check=True, stdout=stream)
            runtime = time.perf_counter() - started
            rows = method.records(output, "CANDIDATE")
            if len(rows) != len(selected):
                raise RuntimeError(f"{event_id}/{selector}: result count mismatch")
            digest_first = {}
            effective = []
            for index, row in enumerate(rows):
                digest = row.get("path_digest", "")
                if digest and digest not in digest_first:
                    digest_first[digest] = index
                    effective.append(row)
            constructed = sum(bool(row.get("path_digest", "")) for row in rows)
            duplicate_count = constructed - len(effective)
            hard_valid = [row for row in effective if row["hard_valid"] == "1"]
            best = min(hard_valid, key=method.oracle_rank) if hard_valid else None
            available_sides = sorted({row["side"] for row in hard_valid})
            production_sides = sorted({row["side"] for row in production})
            result = {
                "validation_event_id": event_id,
                "episode_id": item["row"]["episode_id"],
                "bag": item["row"]["bag"],
                "callback_sequence": item["row"]["callback_sequence"],
                "evaluation_sequence": item["row"]["evaluation_sequence"],
                "selector": selector, "budget_k": budget,
                "activation": "ACTIVATED_PRODUCTION_FAILURE",
                "production_first_prevented_change": False,
                "production_constructed_count": item["evaluation"]["constructed_total_actual"],
                "production_returned_count": item["evaluation"]["returned_candidate_count"],
                "production_hard_valid_count": production_hard_valid,
                "oracle_classification": oracle["classification"],
                "oracle_hard_valid_count": oracle["oracle_hard_valid_count"],
                "raw_prevalidation_pool_count": len(pool),
                "selected_exact_tuple_count": len(selected),
                "constructed_path_count": constructed,
                "construction_guard_reject_count": len(rows) - constructed,
                "raw_harness_validator_execution_count": sum(
                    row["validator_executed"] == "1" for row in rows),
                "duplicate_path_count": duplicate_count,
                "deduplicated_validator_call_count": len(effective),
                "hard_valid_candidate_count": len(hard_valid),
                "hard_valid_side_availability": "|".join(available_sides),
                "recovered": bool(best),
                "unsafe_selected_count": 0,
                "selected_side": best["side"] if best else "",
                "selected_side_in_production_generated_sides":
                    best["side"] in production_sides if best else "",
                "selected_side_matches_oracle_best":
                    best["side"] == oracle.get("best_side", "") if best else "",
                "selected_d_target": best["d_target"] if best else "",
                "selected_d_mid": best["d_mid"] if best else "",
                "selected_entry_scale": best["entry_scale"] if best else "",
                "selected_exit_scale": best["exit_scale"] if best else "",
                "selected_path_digest": best["path_digest"] if best else "",
                "selected_exit_reaches_next_obstacle":
                    best["exit_reaches_next_obstacle"] if best else "",
                "selected_braking_deficit_m": best["braking_deficit_m"] if best else "",
                "selected_velocity_loss": best["velocity_loss"] if best else "",
                "selected_global_path_deviation_m":
                    best["global_path_deviation_m"] if best else "",
                "selected_minimum_normalized_safety_slack":
                    best["minimum_normalized_safety_slack"] if best else "",
                "selected_footprint_track_margin_m":
                    best["footprint_track_margin_m"] if best else "",
                "selected_obstacle_margin_m": best["obstacle_margin_m"] if best else "",
                "selected_lateral_slope_margin": best["lateral_slope_margin"] if best else "",
                "selected_signed_curvature_margin_radpm":
                    best["signed_curvature_margin_radpm"] if best else "",
                "selected_curvature_rate_margin_radpm2":
                    best["curvature_rate_margin_radpm2"] if best else "",
                "runtime_wall_s": runtime,
                "runtime_contract": "raw exact-harness subprocess wall; not production callback latency",
            }
            if len(selected) > budget or len(effective) > budget:
                raise RuntimeError(f"candidate budget exceeded: {event_id}/{selector}")
            results.append(result)
            for index, (candidate, row) in enumerate(zip(selected, rows)):
                selected_audit.append({
                    "validation_event_id": event_id, "selector": selector, "budget_k": budget,
                    "selection_rank": index + 1, "side": candidate["side"],
                    "d_target": candidate["d_target"], "d_mid": candidate["d_mid"],
                    "target_source": candidate["target_source"],
                    "mid_source": candidate["mid_source"],
                    "entry_scale": candidate["entry_scale"],
                    "exit_scale": candidate["exit_scale"],
                    "geometry_lateral_error": candidate["geometry_lateral_error"],
                    "geometry_transition_error": candidate["geometry_transition_error"],
                    "path_digest": row["path_digest"],
                    "duplicate_of_earlier_rank": bool(
                        row["path_digest"] and digest_first[row["path_digest"]] < index),
                    "validator_executed_in_audit_harness": row["validator_executed"],
                    "hard_valid": row["hard_valid"],
                    "first_failure_reason": row["first_failure_reason"],
                    "minimum_normalized_safety_slack": row["minimum_normalized_safety_slack"],
                })
        print("METHOD", event_id, flush=True)
    return results, selected_audit, geometry_rows


def mechanism_from_oracle(row: dict) -> str:
    if row["classification"] == "ORACLE_INCONCLUSIVE":
        return "OTHER_INCONCLUSIVE"
    if row["classification"] == "NO_VALID_P3_FOUND_IN_ORACLE_DOMAIN":
        return "ORACLE_INFEASIBLE_IN_SEARCH_DOMAIN"
    attribution = row.get("mapping_attribution", "")
    if attribution == "SAME_D_TARGET_AND_TRANSITION_SCALES_VALID_ONLY_AT_MISSED_D_MID":
        return "ROOT_PROBE_ISSUE"
    if attribution == "VALID_AT_PRODUCTION_D_TARGET_AND_D_MID_ONLY_WITH_OTHER_TRANSITION_SCALES":
        return "TRANSITION_SELECTOR_MISS"
    if attribution == "VALID_AT_PRODUCTION_D_TARGET_WITH_D_MID_OR_SCALE_DIFFERENCE":
        return "ROOT_PROBE_ISSUE"
    if attribution == "VALID_REQUIRES_DIFFERENT_D_TARGET":
        return "MISSING_LATERAL_FACTOR"
    if attribution == "VALID_REQUIRES_DIFFERENT_D_TARGET_AND_D_MID":
        return "MULTI_PARAMETER_INTERACTION"
    return "OTHER"


def aggregate_outputs(items: list[dict], oracle_summary: list[dict], method_results: list[dict]) -> None:
    result_by_key = {(row["validation_event_id"], row["selector"]): row
                     for row in method_results}
    oracle_by_event = {row["event_id"]: row for row in oracle_summary}
    failure_analysis = []
    for item in items:
        event_id = item["row"]["validation_event_id"]
        oracle = oracle_by_event[event_id]
        h4a = result_by_key[(event_id, "H4A_GEOMETRY_TRANSITION")]
        failure_analysis.append({
            "validation_event_id": event_id,
            "episode_id": item["row"]["episode_id"],
            "bag": item["row"]["bag"],
            "callback_sequence": item["row"]["callback_sequence"],
            "evaluation_sequence": item["row"]["evaluation_sequence"],
            "oracle_classification": oracle["classification"],
            "oracle_hard_valid_count": oracle["oracle_hard_valid_count"],
            "descriptive_failure_mechanism": mechanism_from_oracle(oracle),
            "oracle_mapping_attribution": oracle.get("mapping_attribution", ""),
            "oracle_best_gate": oracle.get("best_gate", ""),
            "oracle_best_side": oracle.get("best_side", ""),
            "h4a_recovered": h4a["recovered"],
            "h4a_selected_side": h4a["selected_side"],
            "h4a_unrecovered_oracle_feasible":
                oracle["classification"] == "ORACLE_VALID_P3_EXISTS" and not truth(h4a["recovered"]),
            "oracle_search_miss_with_frozen_h4a_valid_witness":
                oracle["classification"] == "NO_VALID_P3_FOUND_IN_ORACLE_DOMAIN"
                and truth(h4a["recovered"]),
            "production_first_failure_distribution": item["row"]["first_failure_distribution"],
            "classification_contract":
                "descriptive post-validation analysis; no selector or threshold change",
        })
    write_csv(HERE / "validation_failure_analysis.csv", failure_analysis)

    feasible_ids = {row["event_id"] for row in oracle_summary
                    if row["classification"] == "ORACLE_VALID_P3_EXISTS"}
    frozen_witness_ids = {row["validation_event_id"] for row in method_results
                          if truth(row["recovered"])}
    known_exact_feasible_ids = feasible_ids | frozen_witness_ids
    conclusive_ids = {row["event_id"] for row in oracle_summary
                      if row["classification"] != "ORACLE_INCONCLUSIVE"}
    ablation = [{
        "method": "PRODUCTION", "budget_k": "PRODUCTION",
        "online_selectable": True, "production_first": True,
        "validation_episode_count": len(items),
        "production_failure_episode_count": sum(
            int(item["evaluation"]["hard_valid_total_actual"]) == 0 for item in items),
        "oracle_conclusive_failure_count": len(conclusive_ids),
        "oracle_feasible_failure_count": len(feasible_ids),
        "known_exact_feasible_failure_count": len(known_exact_feasible_ids),
        "recovered_oracle_feasible_count": 0,
        "recovery_rate_oracle_feasible": 0.0 if feasible_ids else math.nan,
        "selected_tuple_count_feasible_total": 0,
        "deduplicated_validator_calls_feasible_total": 0,
        "hard_valid_candidate_count_feasible_total": 0,
    }]
    budget_rows = []
    for selector, budget in FROZEN_SELECTORS.items():
        rows = [row for row in method_results if row["selector"] == selector]
        feasible = [row for row in rows if row["validation_event_id"] in feasible_ids]
        recovered = [row for row in feasible if truth(row["recovered"])]
        known_feasible = [row for row in rows
                          if row["validation_event_id"] in known_exact_feasible_ids]
        known_recovered = [row for row in known_feasible if truth(row["recovered"])]
        low, high = wilson_interval(len(recovered), len(feasible))
        known_low, known_high = wilson_interval(len(known_recovered), len(known_feasible))
        ablation.append({
            "method": selector, "budget_k": budget, "online_selectable": True,
            "production_first": True, "validation_episode_count": len(items),
            "production_failure_episode_count": sum(
                int(row["production_hard_valid_count"]) == 0 for row in rows),
            "oracle_conclusive_failure_count": len(conclusive_ids),
            "oracle_feasible_failure_count": len(feasible),
            "known_exact_feasible_failure_count": len(known_feasible),
            "recovered_oracle_feasible_count": len(recovered),
            "recovery_rate_oracle_feasible": len(recovered) / len(feasible) if feasible else math.nan,
            "recovery_wilson95_lower": low, "recovery_wilson95_upper": high,
            "recovered_known_exact_feasible_count": len(known_recovered),
            "recovery_rate_known_exact_feasible":
                len(known_recovered) / len(known_feasible) if known_feasible else math.nan,
            "known_exact_feasible_wilson95_lower": known_low,
            "known_exact_feasible_wilson95_upper": known_high,
            "valid_witness_outside_predeclared_oracle_samples_count": sum(
                row["validation_event_id"] not in feasible_ids and truth(row["recovered"])
                for row in rows),
            "selected_tuple_count_feasible_total": sum(
                int(row["selected_exact_tuple_count"]) for row in feasible),
            "deduplicated_validator_calls_feasible_total": sum(
                int(row["deduplicated_validator_call_count"]) for row in feasible),
            "hard_valid_candidate_count_feasible_total": sum(
                int(row["hard_valid_candidate_count"]) for row in feasible),
        })
        for scope, scoped in (("ALL_PRODUCTION_FAILURES", rows),
                              ("ORACLE_FEASIBLE_FAILURES", feasible)):
            runtimes = [float(row["runtime_wall_s"]) for row in scoped]
            selected_counts = [int(row["selected_exact_tuple_count"]) for row in scoped]
            validator_counts = [int(row["deduplicated_validator_call_count"]) for row in scoped]
            budget_rows.append({
                "selector": selector, "budget_k": budget, "scope": scope,
                "episode_count": len(scoped),
                "selected_tuple_count_total": sum(selected_counts),
                "selected_tuple_per_episode_p50": percentile(selected_counts, 50),
                "selected_tuple_per_episode_p95": percentile(selected_counts, 95),
                "selected_tuple_per_episode_max": max(selected_counts, default=0),
                "constructed_path_count_total": sum(
                    int(row["constructed_path_count"]) for row in scoped),
                "construction_guard_reject_count_total": sum(
                    int(row["construction_guard_reject_count"]) for row in scoped),
                "raw_harness_validator_executions_total": sum(
                    int(row["raw_harness_validator_execution_count"]) for row in scoped),
                "deduplicated_validator_calls_total": sum(validator_counts),
                "validator_calls_per_episode_p50": percentile(validator_counts, 50),
                "validator_calls_per_episode_p95": percentile(validator_counts, 95),
                "validator_calls_per_episode_max": max(validator_counts, default=0),
                "duplicate_path_count_total": sum(
                    int(row["duplicate_path_count"]) for row in scoped),
                "hard_valid_candidate_count_total": sum(
                    int(row["hard_valid_candidate_count"]) for row in scoped),
                "runtime_wall_p50_s": percentile(runtimes, 50),
                "runtime_wall_p95_s": percentile(runtimes, 95),
                "runtime_wall_p99_s": percentile(runtimes, 99),
                "runtime_wall_max_s": max(runtimes, default=0.0),
                "runtime_contract": "raw exact-harness subprocess wall; not production callback latency",
                "budget_respected": max(selected_counts, default=0) <= budget and
                    max(validator_counts, default=0) <= budget,
            })
    write_csv(HERE / "validation_ablation.csv", ablation)
    write_csv(HERE / "candidate_budget_runtime.csv", budget_rows)

    h4a_recovered = [row for row in method_results
                     if row["selector"] == "H4A_GEOMETRY_TRANSITION"
                     and truth(row["recovered"])]
    margin_fields = [
        "selected_minimum_normalized_safety_slack",
        "selected_footprint_track_margin_m", "selected_obstacle_margin_m",
        "selected_lateral_slope_margin", "selected_signed_curvature_margin_radpm",
        "selected_curvature_rate_margin_radpm2",
    ]
    margin_summary = []
    for field in margin_fields:
        values = [float(row[field]) for row in h4a_recovered]
        margin_summary.append({
            "method": "H4A_GEOMETRY_TRANSITION", "budget_k": 24,
            "margin": field, "selected_hard_valid_count": len(values),
            "minimum": min(values) if values else math.nan,
            "p05": percentile(values, 5), "p50": percentile(values, 50),
            "p95": percentile(values, 95), "maximum": max(values) if values else math.nan,
        })
    write_csv(HERE / "validation_margin_summary.csv", margin_summary)

    create_plots(ablation, budget_rows, failure_analysis)


def create_plots(ablation: list[dict], budget_rows: list[dict], failures: list[dict]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    PLOTS.mkdir(parents=True, exist_ok=True)
    methods = ["H3_FIXED_BOUNDED", "H4A_GEOMETRY_TRANSITION",
               "H4B_GEOMETRY_LATERAL_TRANSITION"]
    rows = [next(row for row in ablation if row["method"] == method) for method in methods]
    figure, axis = plt.subplots(figsize=(8.4, 5.1))
    values = [int(row["recovered_oracle_feasible_count"]) for row in rows]
    denominator = int(rows[0]["oracle_feasible_failure_count"])
    axis.bar(["H3 K24", "H4-A K24", "H4-B K12"], values,
             color=["#4c78a8", "#f58518", "#54a24b"])
    axis.axhline(denominator, color="#555", linestyle="--", linewidth=1,
                 label=f"oracle-feasible failures {denominator}")
    axis.set(ylabel="recovered validation episodes",
             title="Frozen validation recovery among oracle-feasible failures")
    axis.grid(axis="y", alpha=.2); axis.legend(); figure.tight_layout()
    figure.savefig(PLOTS / "validation_recovery_ablation.png", dpi=170); plt.close(figure)

    primary = next(row for row in rows if row["method"] == "H4A_GEOMETRY_TRANSITION")
    validation_rate = float(primary["recovery_rate_oracle_feasible"])
    lower, upper = float(primary["recovery_wilson95_lower"]), float(primary["recovery_wilson95_upper"])
    development_rate = 16 / 23
    figure, axis = plt.subplots(figsize=(7.2, 5.1))
    axis.bar(["DEVELOPMENT\n16/23", "VALIDATION"], [development_rate, validation_rate],
             color=["#999999", "#f58518"])
    axis.errorbar([1], [validation_rate],
                  yerr=[[validation_rate-lower], [upper-validation_rate]],
                  fmt="none", color="black", capsize=5, label="validation Wilson 95%")
    axis.set(ylim=(0, 1), ylabel="recovery fraction",
             title="Frozen H4-A: DEVELOPMENT vs VALIDATION")
    axis.grid(axis="y", alpha=.2); axis.legend(); figure.tight_layout()
    figure.savefig(PLOTS / "development_validation_recovery.png", dpi=170); plt.close(figure)

    all_budget = [row for row in budget_rows if row["scope"] == "ALL_PRODUCTION_FAILURES"]
    figure, axis = plt.subplots(figsize=(8.4, 5.1))
    axis.bar(["H3 K24", "H4-A K24", "H4-B K12"],
             [float(row["validator_calls_per_episode_p95"]) for row in all_budget],
             color=["#4c78a8", "#f58518", "#54a24b"])
    axis.set(ylabel="deduplicated validator calls p95",
             title="Frozen candidate-budget use on validation failures")
    axis.grid(axis="y", alpha=.2); figure.tight_layout()
    figure.savefig(PLOTS / "validation_candidate_budget.png", dpi=170); plt.close(figure)

    counter = Counter(row["descriptive_failure_mechanism"] for row in failures
                      if row["oracle_classification"] == "ORACLE_VALID_P3_EXISTS")
    figure, axis = plt.subplots(figsize=(9.0, 5.2))
    labels = sorted(counter)
    axis.bar(labels, [counter[label] for label in labels], color="#4c78a8")
    axis.tick_params(axis="x", labelrotation=20)
    axis.set(ylabel="oracle-feasible production failures",
             title="Descriptive validation failure mechanisms")
    axis.grid(axis="y", alpha=.2); figure.tight_layout()
    figure.savefig(PLOTS / "validation_failure_mechanisms.png", dpi=170); plt.close(figure)


def main() -> None:
    verification = freeze_gate()
    validation = validation_rows_only()
    verification["validation_rows_loaded_after_gate"] = True
    verification["validation_rows_loaded"] = len(validation)
    verification["final_holdout_rows_loaded"] = 0
    (HERE / "freeze_verification.json").write_text(
        json.dumps(verification, indent=2) + "\n", encoding="utf-8")

    items = selected_inputs(validation)
    manifest_rows = []
    for item in items:
        row = item["row"]
        manifest_rows.append({
            "validation_event_id": row["validation_event_id"],
            "dataset_role": row["dataset_role"], "episode_id": row["episode_id"],
            "bag": row["bag"], "elapsed_s": row["elapsed_s"],
            "callback_sequence": row["callback_sequence"],
            "evaluation_sequence": row["evaluation_sequence"],
            "input_snapshot_id": row["input_snapshot_id"],
            "ego_snapshot_id": row["ego_snapshot_id"],
            "obstacle_snapshot_id": row["obstacle_snapshot_id"],
            "reference_snapshot_id": row["reference_snapshot_id"],
            "source_stamp_ns": row["source_stamp_ns"], "source_epoch": row["source_epoch"],
            "candidate_lineage_exact": row["candidate_lineage_exact"],
            "exact_input_lineage_verification": row["exact_input_lineage_verification"],
            "production_constructed_candidate_count": row["production_constructed_candidate_count"],
            "production_returned_candidate_count": row["production_returned_candidate_count"],
            "production_hard_valid_count": row["production_hard_valid_count"],
            "localization_label": row["localization_label"],
            "localization_temporal_coverage": row["localization_temporal_coverage"],
            "final_holdout_accessed": False,
        })
    write_csv(HERE / "validation_manifest.csv", manifest_rows)

    if "--reuse-existing-oracle" not in sys.argv:
        run_oracle(items)
    oracle_summary = json.loads((ORACLE_WORK / "manifest.json").read_text(encoding="utf-8"))
    if len(oracle_summary) != EXPECTED["validation_count"]:
        raise RuntimeError("oracle summary count mismatch")

    method = load_module("frozen_h4a_method", METHOD_IMPL)
    method_results, selected_audit, geometry_rows = run_method(items, oracle_summary, method)
    write_csv(HERE / "validation_method_results.csv", method_results)
    write_csv(HERE / "validation_selected_candidates.csv", selected_audit)
    write_csv(HERE / "validation_geometry_features.csv", geometry_rows)
    result_by_event = defaultdict(list)
    for result in method_results:
        result_by_event[result["validation_event_id"]].append(result)
    for oracle in oracle_summary:
        witnesses = [row["selector"] for row in result_by_event[oracle["event_id"]]
                     if truth(row["recovered"])]
        oracle["frozen_method_exact_valid_witnesses"] = "|".join(sorted(witnesses))
        if oracle["classification"] == "ORACLE_VALID_P3_EXISTS":
            oracle["validation_feasibility_interpretation"] = "ORACLE_DETECTED_FEASIBLE"
        elif witnesses:
            oracle["validation_feasibility_interpretation"] = (
                "EXACT_FROZEN_METHOD_WITNESS_OUTSIDE_PREDECLARED_ORACLE_SAMPLES")
        elif oracle["classification"] == "ORACLE_INCONCLUSIVE":
            oracle["validation_feasibility_interpretation"] = "INCONCLUSIVE"
        else:
            oracle["validation_feasibility_interpretation"] = (
                "NO_VALID_P3_FOUND_IN_PREDECLARED_ORACLE_DOMAIN")
    write_csv(HERE / "validation_oracle_summary.csv", oracle_summary)
    aggregate_outputs(items, oracle_summary, method_results)

    for event in RAW_ORACLE.joinpath("inputs").glob("VUE*.event"):
        event_id = event.stem
        next(row for row in manifest_rows if row["validation_event_id"] == event_id)[
            "event_input_sha256"] = sha256(event)
    parity = {row["event_id"]: row for row in read_csv(RAW_ORACLE / "lineage_parity.csv")}
    for row in manifest_rows:
        source = parity[row["validation_event_id"]]
        row["constructed_digest_multiset_parity"] = source["constructed_digest_multiset_parity"]
        row["returned_digest_multiset_parity"] = source["returned_digest_multiset_parity"]
    write_csv(HERE / "validation_manifest.csv", manifest_rows)

    execution = {
        "schema": "p3_geometry_conditioned_validation_execution_v1",
        "dataset_role": "VALIDATION_UNSEEN",
        "episode_count": len(items),
        "production_failure_count": sum(
            int(item["evaluation"]["hard_valid_total_actual"]) == 0 for item in items),
        "selectors": FROZEN_SELECTORS,
        "oracle_policy": "unchanged deterministic pilot-v2 broad oracle",
        "oracle_runner_sha256": sha256(ORACLE_RUNNER),
        "harness_sha256": sha256(HARNESS),
        "method_spec_sha256": sha256(METHOD_SPEC),
        "selected_prototype_spec_sha256": sha256(SELECTED_SPEC),
        "dataset_split_manifest_sha256": sha256(MANIFEST),
        "lineage_and_digest_parity_pass_count": sum(
            truth(row["constructed_digest_multiset_parity"]) and
            truth(row["returned_digest_multiset_parity"]) for row in manifest_rows),
        "final_holdout_rows_loaded": 0,
        "frozen_method_modified": False,
        "production_source_modified": False,
        "tuning_performed": False,
    }
    (HERE / "execution_manifest.json").write_text(
        json.dumps(execution, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(execution, indent=2), flush=True)


if __name__ == "__main__":
    main()
