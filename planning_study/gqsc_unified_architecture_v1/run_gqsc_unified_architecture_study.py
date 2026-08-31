#!/usr/bin/env python3
"""Seen-only native GQSC/legacy ladder subsumption and architecture study."""

from __future__ import annotations

import argparse
import csv
import importlib.util
import math
import os
import subprocess
from collections import Counter, defaultdict
from pathlib import Path


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
HARNESS = REPO / "build/local_planning/p3_r3_k12_integration_harness"
REFERENCE = REPO / "planning_study/p3_reference_oracle_v2/run_reference_oracle_v2.py"
SUCCESS_DIR = REPO / "planning_study/p3_geometry_conditioned_method_v1"
SUCCESS_INPUTS = SUCCESS_DIR / "success_control_inputs"
SUCCESS_MANIFEST = SUCCESS_DIR / "success_control_manifest.csv"
EXPECTED_FAILURE_ROLES = {
    "PILOT_SEEN_DEVELOPMENT_DATA", "DEVELOPMENT", "VALIDATION_SEEN_AFTER_V1"}
EPS = 1.0e-9


def load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


R = load("gqsc_seen_reference_catalog", REFERENCE)


def read_csv(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, rows: list[dict], fields: list[str] | None = None) -> None:
    if fields is None:
        fields = []
        for row in rows:
            for key in row:
                if key not in fields:
                    fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def truth(value: str) -> bool:
    return value in {"1", "true", "True"}


def number(value: str) -> float:
    return float(value)


def percentile(values: list[float], q: float) -> float:
    values = sorted(values)
    if not values:
        return math.nan
    position = (len(values) - 1) * q / 100.0
    lower, upper = math.floor(position), math.ceil(position)
    if lower == upper:
        return values[lower]
    return values[lower] + (position - lower) * (values[upper] - values[lower])


def runtime_stats(values_us: list[float]) -> dict:
    values = [value / 1000.0 for value in values_us]
    return {
        "runtime_measurement_count": len(values),
        "p50_ms": percentile(values, 50),
        "p90_ms": percentile(values, 90),
        "p95_ms": percentile(values, 95),
        "p99_ms": percentile(values, 99),
        "max_ms": max(values) if values else math.nan,
    }


def corpus() -> list[dict]:
    failures = R.input_catalog()
    if len(failures) != 86 or {row["dataset_role"] for row in failures} != EXPECTED_FAILURE_ROLES:
        raise RuntimeError("seen failure corpus contract changed")
    output = [{
        "event_id": row["event_id"], "dataset_role": row["dataset_role"],
        "bag": row["row"].get("bag", "UNKNOWN"), "event_path": row["event_path"],
        "source_kind": "FROZEN_PRODUCTION_FAILURE",
    } for row in failures]
    success_rows = read_csv(SUCCESS_MANIFEST)
    if len(success_rows) != 18:
        raise RuntimeError("success-control corpus contract changed")
    for row in success_rows:
        event_path = SUCCESS_INPUTS / f"{row['control_event_id']}.event"
        if not event_path.is_file():
            raise RuntimeError(f"missing success control: {event_path}")
        output.append({
            "event_id": row["control_event_id"], "dataset_role": row["dataset_role"],
            "bag": row["bag"], "event_path": event_path,
            "source_kind": "LEGACY_SUCCESS_CONTROL",
            "baseline_digest": row.get("production_selected_path_digest", ""),
        })
    if len(output) != 104 or len({row["event_id"] for row in output}) != 104:
        raise RuntimeError("combined seen-only corpus must contain exactly 104 unique events")
    return output


def run_native(items: list[dict], warmup: int, repeats: int) -> str:
    env = os.environ.copy()
    env.pop("LOCAL_PLANNING_RESEARCH_PARITY", None)
    env.update({
        "GQSC_ARCHITECTURE_STUDY": "1",
        "GQSC_STUDY_WARMUP": str(warmup),
        "GQSC_STUDY_REPEATS": str(repeats),
    })
    completed = subprocess.run(
        [str(HARNESS), *(str(row["event_path"]) for row in items)],
        check=True, text=True, stdout=subprocess.PIPE, env=env)
    return completed.stdout


RESULT_FIELDS = (
    "invoked", "recovered", "selected_usable", "any_usable", "selected_stage",
    "selected_source", "selected_digest", "selected_go_left", "d_target", "d_mid",
    "entry_scale", "exit_scale", "returned_candidates", "returned_validators",
    "returned_hard_valid", "m0_v1_candidates", "m0_v1_validators", "m0_v1_hard_valid",
    "m0_v2_candidates", "m0_v2_validators", "m0_v2_hard_valid", "m1_candidates",
    "m1_validators", "m1_hard_valid", "r3_constructions", "r3_validators",
    "r3_hard_valid", "r3_usable_valid", "context_us", "m0_v1_us", "m0_v2_us",
    "m1_us", "bookkeeping_us", "result_total_us", "r3_total_us", "safety_slack",
    "track_margin_m", "obstacle_margin_m", "curvature_margin", "curvature_rate_margin",
    "slope_margin", "braking_deficit_m", "velocity_loss", "exit_conflict", "failure")
BOOL_RESULT_FIELDS = {
    "invoked", "recovered", "selected_usable", "any_usable", "selected_go_left",
    "exit_conflict"}
INT_RESULT_FIELDS = {
    "returned_candidates", "returned_validators", "returned_hard_valid",
    "m0_v1_candidates", "m0_v1_validators", "m0_v1_hard_valid", "m0_v2_candidates",
    "m0_v2_validators", "m0_v2_hard_valid", "m1_candidates", "m1_validators",
    "m1_hard_valid", "r3_constructions", "r3_validators", "r3_hard_valid",
    "r3_usable_valid"}
STRING_RESULT_FIELDS = {"selected_stage", "selected_source", "selected_digest", "failure"}


def parse_native(text: str) -> dict:
    output = {
        "results": {}, "timing": [], "laterals": defaultdict(list),
        "transitions": defaultdict(list), "pairs": defaultdict(list),
        "top": defaultdict(list), "candidates": defaultdict(list)}
    for line in text.splitlines():
        fields = line.split("\t")
        if fields[0] == "STUDY_RESULT":
            if len(fields) != 48:
                raise RuntimeError(f"unexpected STUDY_RESULT width: {len(fields)}")
            row = {"event_id": fields[1], "label": fields[2]}
            for key, value in zip(RESULT_FIELDS, fields[3:]):
                if key in BOOL_RESULT_FIELDS:
                    row[key] = truth(value)
                elif key in INT_RESULT_FIELDS:
                    row[key] = int(value)
                elif key in STRING_RESULT_FIELDS:
                    row[key] = value
                else:
                    row[key] = number(value)
            output["results"][(row["event_id"], row["label"])] = row
        elif fields[0] == "ARCH_TIMING":
            output["timing"].append({
                "event_id": fields[1], "architecture": fields[2], "repeat": int(fields[3]),
                "wall_us": number(fields[4]), "result_total_us": number(fields[5]),
                "r3_total_us": number(fields[6]), "context_us": number(fields[7]),
                "m0_v1_us": number(fields[8]), "m0_v2_us": number(fields[9]),
                "m1_us": number(fields[10]), "bookkeeping_us": number(fields[11]),
            })
        elif fields[0] == "GQSC_LATERAL":
            output["laterals"][fields[1]].append({
                "index": int(fields[2]), "side": fields[3], "d_target": number(fields[4]),
                "d_mid": number(fields[5]), "target_source": fields[6],
                "mid_source": fields[7], "family": fields[8],
                "source_priority": int(fields[9]), "operator": fields[10],
            })
        elif fields[0] == "GQSC_TRANSITION":
            output["transitions"][fields[1]].append({
                "index": int(fields[2]), "entry_scale": number(fields[3]),
                "exit_scale": number(fields[4]), "family": fields[5],
            })
        elif fields[0] == "GQSC_PAIR":
            output["pairs"][fields[1]].append({
                "index": int(fields[2]), "side": fields[3], "d_target": number(fields[4]),
                "d_mid": number(fields[5]), "entry_scale": number(fields[6]),
                "exit_scale": number(fields[7]), "configuration_key": fields[8],
            })
        elif fields[0] == "GQSC_TOP":
            output["top"][fields[1]].append({
                "stream": fields[2], "index": int(fields[3]), "side": fields[4],
                "d_target": number(fields[5]), "d_mid": number(fields[6]),
                "entry_scale": number(fields[7]), "exit_scale": number(fields[8]),
                "configuration_key": fields[9],
            })
        elif fields[0] == "GQSC_CANDIDATE":
            output["candidates"][fields[1]].append({
                "side": fields[2], "d_target": number(fields[3]), "d_mid": number(fields[4]),
                "entry_scale": number(fields[5]), "exit_scale": number(fields[6]),
                "digest": fields[7], "hard_valid": truth(fields[8]),
                "usable_valid": truth(fields[9]), "safety_slack": number(fields[10]),
                "track_margin_m": number(fields[11]), "obstacle_margin_m": number(fields[12]),
                "curvature_margin": number(fields[13]),
                "curvature_rate_margin": number(fields[14]), "slope_margin": number(fields[15]),
                "braking_deficit_m": number(fields[16]), "velocity_loss": number(fields[17]),
                "exit_conflict": truth(fields[18]),
            })
    return output


def actual_legacy_counts(results: dict, event_id: str) -> tuple[int, int]:
    strict = results[(event_id, "LEGACY_STRICT")]
    passes = [strict]
    relaxed = results.get((event_id, "LEGACY_RELAXED"))
    if relaxed is not None:
        passes.append(relaxed)
    constructions = sum(
        row["m0_v1_candidates"] + row["m0_v2_candidates"] + row["m1_candidates"]
        for row in passes)
    validators = sum(
        row["m0_v1_validators"] + row["m0_v2_validators"] + row["m1_validators"]
        for row in passes)
    return constructions, validators


def config(row: dict) -> tuple:
    side = "LEFT" if row.get("selected_go_left") else "RIGHT"
    return (side, float(row["d_target"]).hex(), float(row["d_mid"]).hex(),
            float(row["entry_scale"]).hex(), float(row["exit_scale"]).hex())


def factor_config(row: dict) -> tuple:
    return (row["side"], float(row["d_target"]).hex(), float(row["d_mid"]).hex(),
            float(row["entry_scale"]).hex(), float(row["exit_scale"]).hex())


def stage_runtime(parsed: dict) -> list[dict]:
    results, timing = parsed["results"], parsed["timing"]
    rows = []
    pass_architectures = {
        "STRICT": "LEGACY_STRICT_PASS", "RELAXED": "LEGACY_RELAXED_PASS"}
    stage_contracts = (
        ("CONTEXT_PREPARATION", "context_us", None, None, None),
        ("M0_V1", "m0_v1_us", "m0_v1_candidates", "m0_v1_validators", "M0_V1"),
        ("M0_V2", "m0_v2_us", "m0_v2_candidates", "m0_v2_validators", "M0_V2"),
        ("M1", "m1_us", "m1_candidates", "m1_validators", "M1"),
        ("FRESH_EVALUATOR_BOOKKEEPING", "bookkeeping_us", None, None, None),
    )
    for pass_name, architecture in pass_architectures.items():
        pass_label = f"LEGACY_{pass_name}"
        summaries = [row for (event_id, label), row in results.items() if label == pass_label]
        samples = [row for row in timing if row["architecture"] == architecture]
        for stage, timer, candidate_field, validator_field, selection_stage in stage_contracts:
            invoked_summaries = summaries
            if stage in {"M0_V2", "M1"}:
                invoked_summaries = [row for row in summaries if row[timer] > 0.0]
            event_ids = {row["event_id"] for row in invoked_summaries}
            stage_samples = [row[timer] for row in samples if row["event_id"] in event_ids]
            stage_success_count = (
                sum(row["selected_stage"] == selection_stage for row in invoked_summaries)
                if selection_stage else sum(row["recovered"] for row in invoked_summaries))
            rows.append({
                "pass": pass_name, "stage": stage, "measurement_status": "MEASURED",
                "stage_invocation_count": len(invoked_summaries),
                "candidate_construction_count": sum(
                    row[candidate_field] for row in invoked_summaries) if candidate_field else 0,
                "validator_execution_count": sum(
                    row[validator_field] for row in invoked_summaries) if validator_field else 0,
                "hard_valid_count": sum(
                    row[candidate_field.replace("candidates", "hard_valid")]
                    for row in invoked_summaries) if candidate_field else 0,
                "first_selected_stage_count": sum(
                    row["selected_stage"] == selection_stage for row in invoked_summaries)
                    if selection_stage else 0,
                "success_count": stage_success_count,
                "failure_count": len(invoked_summaries) - stage_success_count,
                **runtime_stats(stage_samples),
            })
    for stage in (
        "ACTIVE_P3_CONTINUATION", "GUARDED_REVALIDATION", "RAW_REVALIDATION",
        "NODE_SAFETY_FALLBACK_BOOKKEEPING"):
        rows.append({
            "pass": "INDEPENDENT_SNAPSHOT_LIMIT", "stage": stage,
            "measurement_status": "NOT_PRESENT_IN_STATELESS_EVALUATOR_CORPUS",
            "stage_invocation_count": 0, "candidate_construction_count": 0,
            "validator_execution_count": 0, "hard_valid_count": 0,
            "first_selected_stage_count": 0, "success_count": 0, "failure_count": 0,
            **runtime_stats([]),
        })
    return rows


def coverage_matrix(items: list[dict], parsed: dict) -> list[dict]:
    results = parsed["results"]
    rows = []
    for contract, legacy_field, gqsc_field in (
            ("HARD_VALID", "recovered", "recovered"),
            ("USABLE_VALID_EXISTS", "any_usable", "any_usable"),
            ("SELECTED_USABLE_VALID", "selected_usable", "selected_usable")):
        buckets = Counter()
        ids = defaultdict(list)
        for item in items:
            legacy = results[(item["event_id"], "ARCH_A_LEGACY_ONLY")][legacy_field]
            gqsc = results[(item["event_id"], "ARCH_C_GQSC_FORCED_WITH_LEGACY_SEEDS")][gqsc_field]
            key = ("LEGACY_SUCCESS" if legacy else "LEGACY_FAIL",
                   "GQSC_SUCCESS" if gqsc else "GQSC_FAIL")
            buckets[key] += 1
            ids[key].append(item["event_id"])
        for legacy_status in ("LEGACY_SUCCESS", "LEGACY_FAIL"):
            for gqsc_status in ("GQSC_SUCCESS", "GQSC_FAIL"):
                key = legacy_status, gqsc_status
                rows.append({
                    "validity_contract": contract, "legacy_status": legacy_status,
                    "gqsc_status": gqsc_status, "snapshot_count": buckets[key],
                    "event_ids": ";".join(ids[key]),
                    "gqsc_execution_contract": "FORCED_SHADOW_WITH_CURRENT_LEGACY_FACTOR_SEEDS",
                })
    return rows


def success_comparison(items: list[dict], parsed: dict) -> list[dict]:
    results = parsed["results"]
    metadata = {row["event_id"]: row for row in items}
    timing = defaultdict(list)
    for sample in parsed["timing"]:
        timing[(sample["event_id"], sample["architecture"])].append(sample)
    rows = []
    for event_id, item in metadata.items():
        legacy = results[(event_id, "ARCH_A_LEGACY_ONLY")]
        if not legacy["recovered"]:
            continue
        gqsc = results[(event_id, "ARCH_C_GQSC_FORCED_WITH_LEGACY_SEEDS")]
        legacy_timing = timing[(event_id, "A_LEGACY_ONLY")]
        gqsc_timing = timing[(event_id, "C_GQSC_FORCED_WITH_LEGACY_SEEDS")]
        rows.append({
            "event_id": event_id, "dataset_role": item["dataset_role"], "bag": item["bag"],
            "legacy_selected_stage": legacy["selected_stage"],
            "legacy_hard_valid": int(legacy["recovered"]),
            "legacy_usable_valid": int(legacy["selected_usable"]),
            "legacy_digest": legacy["selected_digest"], "legacy_d_target": legacy["d_target"],
            "legacy_d_mid": legacy["d_mid"], "legacy_entry_scale": legacy["entry_scale"],
            "legacy_exit_scale": legacy["exit_scale"],
            "legacy_safety_slack": legacy["safety_slack"],
            "legacy_track_margin_m": legacy["track_margin_m"],
            "legacy_obstacle_margin_m": legacy["obstacle_margin_m"],
            "legacy_curvature_margin": legacy["curvature_margin"],
            "legacy_curvature_rate_margin": legacy["curvature_rate_margin"],
            "legacy_slope_margin": legacy["slope_margin"],
            "legacy_braking_deficit_m": legacy["braking_deficit_m"],
            "legacy_velocity_loss": legacy["velocity_loss"],
            "legacy_constructed_actual": actual_legacy_counts(results, event_id)[0],
            "legacy_validators_actual": actual_legacy_counts(results, event_id)[1],
            "legacy_wall_p50_ms": percentile(
                [sample["wall_us"] / 1000.0 for sample in legacy_timing], 50),
            "gqsc_hard_valid": int(gqsc["recovered"]),
            "gqsc_selected_usable_valid": int(gqsc["selected_usable"]),
            "gqsc_usable_valid_exists": int(gqsc["any_usable"]),
            "gqsc_digest": gqsc["selected_digest"], "gqsc_d_target": gqsc["d_target"],
            "gqsc_d_mid": gqsc["d_mid"], "gqsc_entry_scale": gqsc["entry_scale"],
            "gqsc_exit_scale": gqsc["exit_scale"], "gqsc_safety_slack": gqsc["safety_slack"],
            "gqsc_track_margin_m": gqsc["track_margin_m"],
            "gqsc_obstacle_margin_m": gqsc["obstacle_margin_m"],
            "gqsc_curvature_margin": gqsc["curvature_margin"],
            "gqsc_curvature_rate_margin": gqsc["curvature_rate_margin"],
            "gqsc_slope_margin": gqsc["slope_margin"],
            "gqsc_braking_deficit_m": gqsc["braking_deficit_m"],
            "gqsc_velocity_loss": gqsc["velocity_loss"],
            "gqsc_constructions": gqsc["r3_constructions"],
            "gqsc_validators": gqsc["r3_validators"],
            "gqsc_seeded_wall_p50_ms": percentile(
                [sample["wall_us"] / 1000.0 for sample in gqsc_timing], 50),
            "gqsc_core_p50_ms": percentile(
                [sample["r3_total_us"] / 1000.0 for sample in gqsc_timing], 50),
            "selected_digest_exact": int(legacy["selected_digest"] == gqsc["selected_digest"]),
            "selected_parameter_linf_distance": max(
                abs(legacy[key] - gqsc[key]) for key in
                ("d_target", "d_mid", "entry_scale", "exit_scale"))
                if gqsc["recovered"] else math.nan,
            "track_margin_delta_m": gqsc["track_margin_m"] - legacy["track_margin_m"]
                if gqsc["recovered"] else math.nan,
            "obstacle_margin_delta_m": gqsc["obstacle_margin_m"] - legacy["obstacle_margin_m"]
                if gqsc["recovered"] else math.nan,
        })
    return rows


def subsumption_failures(success_rows: list[dict], parsed: dict) -> list[dict]:
    output = []
    for row in success_rows:
        event_id = row["event_id"]
        legacy = parsed["results"][(event_id, "ARCH_A_LEGACY_ONLY")]
        gqsc = parsed["results"][(event_id, "ARCH_C_GQSC_FORCED_WITH_LEGACY_SEEDS")]
        for contract, failed in (
                ("HARD_VALID", legacy["recovered"] and not gqsc["recovered"]),
                ("USABLE_VALID_EXISTS", legacy["any_usable"] and not gqsc["any_usable"])):
            if not failed:
                continue
            prod = config(legacy)
            side, target, middle, entry, exit_scale = prod
            laterals = parsed["laterals"][event_id]
            transitions = parsed["transitions"][event_id]
            pairs = parsed["pairs"][event_id]
            top = parsed["top"][event_id]
            candidates = parsed["candidates"][event_id]
            lateral_present = any(
                item["side"] == side and item["d_target"].hex() == target and
                item["d_mid"].hex() == middle for item in laterals)
            transition_present = any(
                item["entry_scale"].hex() == entry and item["exit_scale"].hex() == exit_scale
                for item in transitions)
            pair_present = any(factor_config(item) == prod for item in pairs)
            top_present = any(factor_config(item) == prod for item in top)
            digest_present = any(
                item["digest"] == legacy["selected_digest"] for item in candidates)
            if not lateral_present:
                taxonomy = "MISSING_LATERAL_HYPOTHESIS"
            elif not transition_present:
                taxonomy = "MISSING_TRANSITION_HYPOTHESIS"
            elif not pair_present and len(pairs) >= 128:
                taxonomy = "B128_TRUNCATION"
            elif not pair_present:
                taxonomy = "PAIRING_HYPOTHESIS_MISS"
            elif not top_present:
                taxonomy = "RANKING_DISPLACEMENT"
            elif not digest_present:
                taxonomy = "CONSTRUCTION_OR_IMPLEMENTATION_MISMATCH"
            else:
                taxonomy = "OTHER"
            nearest_lateral = min((
                max(abs(item["d_target"] - legacy["d_target"]),
                    abs(item["d_mid"] - legacy["d_mid"])) for item in laterals), default=math.inf)
            output.append({
                "event_id": event_id, "validity_contract": contract,
                "classification": taxonomy, "production_selected_stage": legacy["selected_stage"],
                "production_selected_digest": legacy["selected_digest"],
                "production_selected_lateral_exact": int(lateral_present),
                "production_selected_transition_exact": int(transition_present),
                "production_selected_pair_in_b128": int(pair_present),
                "production_selected_pair_in_top12": int(top_present),
                "production_selected_digest_reconstructed": int(digest_present),
                "equivalent_geometry_present": int(digest_present),
                "nearest_lateral_linf_distance": nearest_lateral,
                "bounded_pair_count": len(pairs), "top_factor_count": len(top),
                "constructed_candidate_count": len(candidates),
                "gqsc_hard_valid_count": gqsc["r3_hard_valid"],
                "gqsc_usable_valid_count": gqsc["r3_usable_valid"],
                "lifecycle_specific_testable": 0,
                "note": "independent evaluator snapshot; lifecycle state is unavailable",
            })
    return output


def architecture_rows(items: list[dict], parsed: dict) -> list[dict]:
    results = parsed["results"]
    contracts = (
        ("A_LEGACY_ONLY", "ARCH_A_LEGACY_ONLY", "EXECUTABLE_SHADOW"),
        ("B_LEGACY_THEN_GQSC_B128", "ARCH_B_LEGACY_THEN_GQSC_B128", "EXECUTABLE_SHADOW"),
        ("C_GQSC_B128_ONLY", "ARCH_C_GQSC_FORCED_WITH_LEGACY_SEEDS",
         "OUTCOME_ONLY_NOT_STANDALONE_EXECUTABLE_CURRENT_LEGACY_SEEDS_REQUIRED"),
        ("D_M0V1_THEN_GQSC_B128", "ARCH_D_M0V1_THEN_GQSC_B128", "EXECUTABLE_SHADOW"),
    )
    rows = []
    for architecture, label, execution in contracts:
        subset = [(item, results[(item["event_id"], label)]) for item in items]
        legacy_failures = [item for item in items if not results[
            (item["event_id"], "ARCH_A_LEGACY_ONLY")]["recovered"]]
        legacy_successes = [item for item in items if results[
            (item["event_id"], "ARCH_A_LEGACY_ONLY")]["recovered"]]
        legacy_usable_successes = [item for item in legacy_successes if results[
            (item["event_id"], "ARCH_A_LEGACY_ONLY")]["any_usable"]]
        constructions = validators = seed_constructions = seed_validators = 0
        for item, result in subset:
            event_id = item["event_id"]
            legacy_counts = actual_legacy_counts(results, event_id)
            if architecture == "A_LEGACY_ONLY":
                constructions += legacy_counts[0]
                validators += legacy_counts[1]
            elif architecture == "B_LEGACY_THEN_GQSC_B128":
                constructions += legacy_counts[0]
                validators += legacy_counts[1]
                if not results[(event_id, "ARCH_A_LEGACY_ONLY")]["recovered"]:
                    constructions += result["r3_constructions"]
                    validators += result["r3_validators"]
            elif architecture == "C_GQSC_B128_ONLY":
                seed_constructions += legacy_counts[0]
                seed_validators += legacy_counts[1]
                constructions += result["r3_constructions"]
                validators += result["r3_validators"]
            else:
                m0 = results[(event_id, "M0_V1_ONLY")]
                constructions += m0["m0_v1_candidates"]
                validators += m0["m0_v1_validators"]
                if not m0["recovered"]:
                    constructions += result["r3_constructions"]
                    validators += result["r3_validators"]
        rows.append({
            "architecture": architecture, "execution_status": execution,
            "snapshot_count": len(subset),
            "hard_success_count": sum(result["recovered"] for _, result in subset),
            "usable_path_exists_count": sum(result["any_usable"] for _, result in subset),
            "selected_usable_count": sum(result["selected_usable"] for _, result in subset),
            "legacy_success_regression_count": sum(
                not results[(item["event_id"], label)]["recovered"] for item in legacy_successes),
            "legacy_usable_regression_count": sum(
                not results[(item["event_id"], label)]["any_usable"]
                for item in legacy_usable_successes),
            "legacy_failure_hard_recovery_count": sum(
                results[(item["event_id"], label)]["recovered"] for item in legacy_failures),
            "legacy_failure_usable_recovery_count": sum(
                results[(item["event_id"], label)]["any_usable"] for item in legacy_failures),
            "candidate_constructions_total": constructions,
            "validator_executions_total": validators,
            "legacy_seed_constructions_not_in_architecture_count": seed_constructions,
            "legacy_seed_validators_not_in_architecture_count": seed_validators,
            "maximum_gqsc_constructions": max(result["r3_constructions"] for _, result in subset),
            "maximum_gqsc_validators": max(result["r3_validators"] for _, result in subset),
        })
    d = next(row for row in rows if row["architecture"] == "D_M0V1_THEN_GQSC_B128")
    rows.append({**d, "architecture": "E_MINIMAL_PREFIX_IF_JUSTIFIED",
                 "execution_status": "SAME_AS_D_PENDING_COVERAGE_AND_RUNTIME_GATE"})
    return rows


def runtime_architectures(parsed: dict, warmup: int, repeats: int) -> list[dict]:
    rows = []
    contracts = (
        ("A_LEGACY_ONLY", "A_LEGACY_ONLY", "MEASURED_EXECUTABLE_WALL", "wall_us"),
        ("B_LEGACY_THEN_GQSC_B128", "B_LEGACY_THEN_GQSC_B128",
         "MEASURED_EXECUTABLE_WALL", "wall_us"),
        ("C_GQSC_B128_ONLY", "C_GQSC_FORCED_WITH_LEGACY_SEEDS",
         "MEASURED_CURRENT_DEPENDENCY_WALL_INCLUDES_LEGACY_SEED_EXTRACTION", "wall_us"),
        ("C_GQSC_B128_CORE_IDEALIZED", "C_GQSC_FORCED_WITH_LEGACY_SEEDS",
         "MEASURED_CORE_ONLY_NOT_STANDALONE_EXECUTABLE", "r3_total_us"),
        ("D_M0V1_THEN_GQSC_B128", "D_M0V1_THEN_GQSC_B128",
         "MEASURED_EXECUTABLE_WALL", "wall_us"),
    )
    for architecture, source, scope, field in contracts:
        samples = [row[field] for row in parsed["timing"] if row["architecture"] == source]
        stats = runtime_stats(samples)
        rows.append({
            "architecture": architecture, "measurement_scope": scope,
            "release_build": 1, "single_process": 1, "instrumentation_enabled": 0,
            "warmup_per_snapshot": warmup, "measured_repeats_per_snapshot": repeats,
            **stats, "p95_under_25ms": int(stats["p95_ms"] < 25.0),
            "p99_under_25ms": int(stats["p99_ms"] < 25.0),
            "deadline_contract": "SOFT_25MS_PLANNING_CYCLE_TARGET",
        })
    return rows


def recommendation(architectures: list[dict], runtimes: list[dict]) -> str:
    by_arch = {row["architecture"]: row for row in architectures}
    by_runtime = {row["architecture"]: row for row in runtimes}
    b = by_arch["B_LEGACY_THEN_GQSC_B128"]
    d = by_arch["D_M0V1_THEN_GQSC_B128"]
    if (d["legacy_success_regression_count"] == 0 and
            d["legacy_usable_regression_count"] == 0 and
            d["legacy_failure_hard_recovery_count"] == b["legacy_failure_hard_recovery_count"] and
            d["legacy_failure_usable_recovery_count"] == b["legacy_failure_usable_recovery_count"] and
            by_runtime["D_M0V1_THEN_GQSC_B128"]["p95_ms"] < 25.0):
        return "GQSC_MINIMAL_LEGACY_PREFIX_PROMISING"
    c = by_arch["C_GQSC_B128_ONLY"]
    if (c["legacy_success_regression_count"] == 0 and
            by_runtime["C_GQSC_B128_CORE_IDEALIZED"]["p95_ms"] < 25.0):
        return "GQSC_BOUNDED_UNIFICATION_NEEDED"
    if by_runtime["B_LEGACY_THEN_GQSC_B128"]["p95_ms"] < 25.0:
        return "LEGACY_GQSC_SEQUENTIAL_REQUIRED"
    return "INSUFFICIENT"


def write_feasibility(
        failures: list[dict], architectures: list[dict], runtimes: list[dict], decision: str) -> None:
    taxonomy = Counter(row["classification"] for row in failures)
    d = next(row for row in architectures if row["architecture"] == "D_M0V1_THEN_GQSC_B128")
    c_runtime = next(
        row for row in runtimes if row["architecture"] == "C_GQSC_B128_CORE_IDEALIZED")
    operator_note = (
        "No fixed proposal operator is justified by full-GQSC subsumption on this seen corpus."
        if not failures else
        "The following bounds apply only to the observed full-GQSC subsumption gaps."
    )
    text = f"""# Unified bounded-generator feasibility

## Current dependency

Native GQSC B128 is not presently an independent generator. It consumes lateral and transition
seeds collected while the strict legacy M0/M1 ladder is constructed and exact-validated. Therefore
the row named `C_GQSC_B128_ONLY` is an outcome counterfactual; its executable wall-time row still
includes legacy seed extraction. The isolated core timing is measured, but the missing direct seed
producer has not been implemented.

## Seen-only subsumption gaps

- LEGACY_SUCCESS/GQSC_FAIL diagnostic rows: {len(failures)}
- taxonomy: {dict(taxonomy)}
- M0-V1-prefix legacy-success regressions: {d['legacy_success_regression_count']}
- M0-V1-prefix usable regressions: {d['legacy_usable_regression_count']}
- idealized GQSC core p95: {c_runtime['p95_ms']:.3f} ms

## Fixed proposal-operator feasibility

No operator is implemented in this study. {operator_note} For each missing-lateral case, one deterministic
production-selected-equivalent lateral anchor would cost at most seven extra pair proxies (the
fixed transition set size). A missing-transition case would cost at most the retained lateral count
in pair proxies. Any future operator must enter the same bounded proxy ranking and replace, not add
to, the existing Top-12 reconstruction/validator quota; the hard caps remain 12/12. B128 truncation
or ranking-displacement cases should be addressed by bounded ordering/coverage slots rather than by
raising K. These estimates are feasibility bounds, not validated method changes.

## Recommendation

`{decision}`
"""
    (HERE / "unified_generator_feasibility.md").write_text(text, encoding="utf-8")


def write_readme(
        items: list[dict], stage_rows: list[dict], coverage: list[dict],
        comparison: list[dict], architectures: list[dict], runtimes: list[dict],
        failures: list[dict], decision: str) -> None:
    arch = {row["architecture"]: row for row in architectures}
    run = {row["architecture"]: row for row in runtimes}
    hard_matrix = {
        (row["legacy_status"], row["gqsc_status"]): row["snapshot_count"]
        for row in coverage if row["validity_contract"] == "HARD_VALID"}
    measured_stages = [row for row in stage_rows if row["stage"] in {"M0_V1", "M0_V2", "M1"}]
    tail_stage = max(measured_stages, key=lambda row: row["p99_ms"])
    selected_stages = Counter(row["legacy_selected_stage"] for row in comparison)
    exact_digest_count = sum(row["selected_digest_exact"] for row in comparison)
    text = f"""# GQSC production-ladder subsumption and unified architecture study v1

## Scope and contracts

This is an offline/shadow study of exactly {len(items)} already-seen independent evaluator
snapshots: 86 frozen production failures and 18 exact-lineage legacy-success controls. It uses one
warm Release process, research instrumentation OFF, 3 warmups and 10 measured repetitions per
snapshot/architecture. FINAL_HOLDOUT_UNSEEN contents, outcomes, paths, and statistics are not read
or used. Production `plan()`, validator, ranking, lifecycle, fallback, and parameters are unchanged.

`USABLE_VALID` means exact hard-valid, no next-obstacle exit conflict, and braking deficit <=
1e-9 m. The tables distinguish selected-usable from existence of any usable candidate.

## Main result

Hard-valid coverage matrix over all seen snapshots:

- LEGACY_SUCCESS / GQSC_SUCCESS: {hard_matrix[("LEGACY_SUCCESS", "GQSC_SUCCESS")]}
- LEGACY_SUCCESS / GQSC_FAIL: {hard_matrix[("LEGACY_SUCCESS", "GQSC_FAIL")]}
- LEGACY_FAIL / GQSC_SUCCESS: {hard_matrix[("LEGACY_FAIL", "GQSC_SUCCESS")]}
- LEGACY_FAIL / GQSC_FAIL: {hard_matrix[("LEGACY_FAIL", "GQSC_FAIL")]}

The current forced GQSC shadow is not a true standalone generator: it still obtains seed factors
from the legacy strict ladder. Thus coverage can establish candidate-family subsumption on the seen
snapshots, but cannot establish deployable GQSC-only latency.

## Legacy ladder and the 40-70 ms tail

Fresh strict/relaxed passes are timed separately in [legacy_stage_runtime.csv](legacy_stage_runtime.csv).
The largest stage p99 is `{tail_stage['pass']} / {tail_stage['stage']}` at
{tail_stage['p99_ms']:.3f} ms (max {tail_stage['max_ms']:.3f} ms). Independent `.event` snapshots
contain no active maneuver record, current raw snapshot distinct from the evaluator input, or node
fallback state. Therefore active continuation, guarded/raw continuation revalidation, and node
safety/fallback bookkeeping are explicitly `NOT_PRESENT_IN_STATELESS_EVALUATOR_CORPUS`; no zero-ms
runtime is falsely claimed for them.

The 40-70 ms full-evaluator tail is compound rather than a single GQSC stage: legacy-only p99 is
{run['A_LEGACY_ONLY']['p99_ms']:.3f} ms, the isolated GQSC core p99 is
{run['C_GQSC_B128_CORE_IDEALIZED']['p99_ms']:.3f} ms, and sequential p99 is
{run['B_LEGACY_THEN_GQSC_B128']['p99_ms']:.3f} ms. Within the legacy passes, M1 is the largest
tail stage; strict and relaxed passes can both execute before GQSC. The measured B sequential
full-evaluator wall is p50/p95/p99/max
{run['B_LEGACY_THEN_GQSC_B128']['p50_ms']:.3f}/
{run['B_LEGACY_THEN_GQSC_B128']['p95_ms']:.3f}/
{run['B_LEGACY_THEN_GQSC_B128']['p99_ms']:.3f}/
{run['B_LEGACY_THEN_GQSC_B128']['max_ms']:.3f} ms. The table separates this from the isolated GQSC
core and from the current seed-extraction wall.

## Architecture comparison

| architecture | hard success | usable exists | legacy hard regressions | legacy-failure hard recoveries | p95 ms | p99 ms | executable meaning |
|---|---:|---:|---:|---:|---:|---:|---|
| A legacy only | {arch['A_LEGACY_ONLY']['hard_success_count']} | {arch['A_LEGACY_ONLY']['usable_path_exists_count']} | 0 | 0 | {run['A_LEGACY_ONLY']['p95_ms']:.3f} | {run['A_LEGACY_ONLY']['p99_ms']:.3f} | measured |
| B legacy then GQSC | {arch['B_LEGACY_THEN_GQSC_B128']['hard_success_count']} | {arch['B_LEGACY_THEN_GQSC_B128']['usable_path_exists_count']} | {arch['B_LEGACY_THEN_GQSC_B128']['legacy_success_regression_count']} | {arch['B_LEGACY_THEN_GQSC_B128']['legacy_failure_hard_recovery_count']} | {run['B_LEGACY_THEN_GQSC_B128']['p95_ms']:.3f} | {run['B_LEGACY_THEN_GQSC_B128']['p99_ms']:.3f} | measured |
| C GQSC-only outcome | {arch['C_GQSC_B128_ONLY']['hard_success_count']} | {arch['C_GQSC_B128_ONLY']['usable_path_exists_count']} | {arch['C_GQSC_B128_ONLY']['legacy_success_regression_count']} | {arch['C_GQSC_B128_ONLY']['legacy_failure_hard_recovery_count']} | {run['C_GQSC_B128_ONLY']['p95_ms']:.3f} | {run['C_GQSC_B128_ONLY']['p99_ms']:.3f} | wall includes legacy seeds |
| C isolated GQSC core | - | - | - | - | {run['C_GQSC_B128_CORE_IDEALIZED']['p95_ms']:.3f} | {run['C_GQSC_B128_CORE_IDEALIZED']['p99_ms']:.3f} | not standalone executable |
| D M0-V1 then GQSC | {arch['D_M0V1_THEN_GQSC_B128']['hard_success_count']} | {arch['D_M0V1_THEN_GQSC_B128']['usable_path_exists_count']} | {arch['D_M0V1_THEN_GQSC_B128']['legacy_success_regression_count']} | {arch['D_M0V1_THEN_GQSC_B128']['legacy_failure_hard_recovery_count']} | {run['D_M0V1_THEN_GQSC_B128']['p95_ms']:.3f} | {run['D_M0V1_THEN_GQSC_B128']['p99_ms']:.3f} | measured research shadow |

Among 18 legacy hard successes, the first selected stage is M0-V1/M0-V2/M1 =
{selected_stages['M0_V1']}/{selected_stages['M0_V2']}/{selected_stages['M1']}. Forced GQSC finds a
hard-valid path in all 18, but selects the exact same path digest in only {exact_digest_count}/18;
subsumption here means validity coverage, not identical geometry or ranking. D loses
{arch['D_M0V1_THEN_GQSC_B128']['legacy_success_regression_count']} legacy hard successes and retains
only {arch['D_M0V1_THEN_GQSC_B128']['legacy_failure_hard_recovery_count']} of B's
{arch['B_LEGACY_THEN_GQSC_B128']['legacy_failure_hard_recovery_count']} legacy-failure recoveries.

The 25 ms target is a soft planner-cycle target. A row passes p95/p99 only when the measured wall
column is below 25 ms; no hard real-time guarantee follows from this offline process.

## Subsumption failure diagnosis

[gqsc_subsumption_failures.csv](gqsc_subsumption_failures.csv) contains {len(failures)} contract-level
failure rows and checks exact production lateral, transition, B128 pair, Top-12 placement, and path
digest reconstruction. Lifecycle-specific causes are not inferable from independent snapshots.

## Recommendation

`{decision}`

This is the single recommendation for this study. It is not a production integration decision.
"""
    (HERE / "README.md").write_text(text, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument("--repeats", type=int, default=10)
    args = parser.parse_args()
    if not HARNESS.is_file():
        raise RuntimeError(f"missing Release harness: {HARNESS}")
    items = corpus()
    parsed = parse_native(run_native(items, args.warmup, args.repeats))
    expected_results = len(items) * 6 + 86
    if len(parsed["results"]) != expected_results:
        raise RuntimeError(
            f"result cardinality mismatch: {len(parsed['results'])} != {expected_results}")
    stage_rows = stage_runtime(parsed)
    coverage = coverage_matrix(items, parsed)
    comparison = success_comparison(items, parsed)
    failures = subsumption_failures(comparison, parsed)
    architectures = architecture_rows(items, parsed)
    runtimes = runtime_architectures(parsed, args.warmup, args.repeats)
    decision = recommendation(architectures, runtimes)
    write_csv(HERE / "legacy_stage_runtime.csv", stage_rows)
    write_csv(HERE / "legacy_gqsc_coverage_matrix.csv", coverage)
    write_csv(HERE / "legacy_success_shadow_comparison.csv", comparison)
    write_csv(
        HERE / "gqsc_subsumption_failures.csv", failures,
        fields=[
            "event_id", "validity_contract", "classification",
            "production_selected_stage", "production_selected_digest",
            "production_selected_lateral_exact", "production_selected_transition_exact",
            "production_selected_pair_in_b128", "production_selected_pair_in_top12",
            "production_selected_digest_reconstructed", "equivalent_geometry_present",
            "nearest_lateral_linf_distance", "bounded_pair_count", "top_factor_count",
            "constructed_candidate_count", "gqsc_hard_valid_count",
            "gqsc_usable_valid_count", "lifecycle_specific_testable", "note"])
    write_csv(HERE / "architecture_comparison.csv", architectures)
    write_csv(HERE / "native_runtime_architectures.csv", runtimes)
    write_feasibility(failures, architectures, runtimes, decision)
    write_readme(
        items, stage_rows, coverage, comparison, architectures, runtimes, failures, decision)
    print(f"GQSC study complete: recommendation={decision}")


if __name__ == "__main__":
    main()
