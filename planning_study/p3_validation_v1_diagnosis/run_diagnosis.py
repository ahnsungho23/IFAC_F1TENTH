#!/usr/bin/env python3
"""Read-only diagnosis of the frozen validation-v1 outputs.

This script deliberately reads only the already-opened DEVELOPMENT and validation-v1
artifacts.  It never opens the FINAL_HOLDOUT_UNSEEN portion of the split manifest and
does not alter the frozen H4-A implementation.
"""

from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import math
import shutil
import subprocess
from collections import Counter
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
VALIDATION = REPO / "planning_study/p3_geometry_conditioned_validation_v1"
METHOD = REPO / "planning_study/p3_geometry_conditioned_method_v1"
CORPUS = REPO / "planning_study/p3_mapping_research_corpus_v1"
WORK = Path("/tmp/p3_validation_v1_diagnosis")
HARNESS = Path("/tmp/p3_oracle_v2_build/p3_family_oracle_harness")
EXPECTED_HARNESS_SHA256 = (
    "8b23f2b6df54c2e93794ee087f3ebd7a14f9921544af4f97eef755535e045e7e"
)
EPS = 1.0e-9


def read_csv(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, rows: list[dict]) -> None:
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def truth(value) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def num(row: dict, key: str, default: float = math.nan) -> float:
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_method():
    path = METHOD / "analyze_method.py"
    spec = importlib.util.spec_from_file_location("frozen_method_diagnosis", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def close(a, b, tol=1.0e-9) -> bool:
    return abs(float(a) - float(b)) <= tol


def usable(row: dict, prefix: str = "selected_") -> tuple[bool, list[str]]:
    reasons = []
    if prefix:
        hard_valid = True
        field = lambda name: prefix + name
    else:
        hard_valid = row.get("hard_valid") == "1"
        field = lambda name: name
    if not hard_valid:
        reasons.append("NOT_EXACT_HARD_VALID")
    hard_margins = (
        "footprint_track_margin_m", "lateral_slope_margin",
        "signed_curvature_margin_radpm", "curvature_rate_margin_radpm2",
    )
    for name in hard_margins:
        if num(row, field(name)) < -EPS:
            reasons.append("NEGATIVE_HARD_MARGIN:" + name)
    if num(row, field("minimum_normalized_safety_slack")) < -EPS:
        reasons.append("NEGATIVE_NORMALIZED_SAFETY_SLACK")
    if num(row, field("obstacle_margin_m")) < -EPS:
        reasons.append("NEGATIVE_OBSTACLE_DIAGNOSTIC_MARGIN")
    if str(row.get(field("exit_reaches_next_obstacle"), "0")) == "1":
        reasons.append("NEXT_OBSTACLE_EXIT_CONFLICT")
    if num(row, field("braking_deficit_m"), 0.0) > EPS:
        reasons.append("POSITIVE_BRAKING_DEFICIT")
    return not reasons, reasons


def full_pool_audit(method, oracle_rows: list[dict]) -> dict[str, dict]:
    WORK.mkdir(parents=True, exist_ok=True)
    results = {}
    for oracle in oracle_rows:
        event_id = oracle["event_id"]
        event_path = VALIDATION / "raw_oracle/inputs" / f"{event_id}.event"
        event = method.G.parse_event(event_path)
        production = json.loads(oracle["production_candidate_parameters"])
        pool, _ = method.build_pool(event_id, event, production)
        ordered = sorted(pool, key=lambda row: method.selector_score(
            row, "H4A_GEOMETRY_TRANSITION"))
        request = WORK / f"{event_id}.H4A_FULL_POOL.requests"
        output = WORK / f"{event_id}.H4A_FULL_POOL.out"
        method.write_requests(request, "H4A_FULL_POOL_DIAGNOSIS", ordered)
        with output.open("w", encoding="utf-8", newline="\n") as stream:
            subprocess.run([str(HARNESS), str(event_path), str(request)],
                           check=True, stdout=stream)
        rows = method.records(output, "CANDIDATE")
        if len(rows) != len(ordered):
            raise RuntimeError(f"full-pool result count mismatch for {event_id}")
        first_by_digest = {}
        effective = []
        for rank, (candidate, row) in enumerate(zip(ordered, rows), 1):
            row = dict(row)
            row["selection_rank"] = rank
            row["target_source"] = candidate["target_source"]
            row["mid_source"] = candidate["mid_source"]
            row["geometry_transition_error"] = candidate["geometry_transition_error"]
            digest = row.get("path_digest", "")
            if digest and digest not in first_by_digest:
                first_by_digest[digest] = rank
                effective.append(row)
        hard = [row for row in effective if row["hard_valid"] == "1"]
        usable_rows = [row for row in hard if usable(row, prefix="")[0]]
        results[event_id] = {
            "pool": ordered, "rows": rows, "effective": effective,
            "hard": hard, "usable": usable_rows,
            "first_hard_rank": min((int(row["selection_rank"]) for row in hard), default=None),
            "first_usable_rank": min((int(row["selection_rank"]) for row in usable_rows), default=None),
        }
        print("FULL_POOL", event_id, len(ordered), len(effective), len(hard), flush=True)
    return results


def vue036_audit(method, method_rows: list[dict], selected_rows: list[dict],
                 oracle_by_id: dict[str, dict], full_pool: dict[str, dict]) -> None:
    event_id = "VUE036"
    valid_digest = "95a7394f79189d58"
    target = -0.67258957694719368
    middle = -0.72465798078932808
    entry = 0.51458109301505117
    exit_scale = 0.49716841625749453
    coarse_request = Path("/tmp/p3_geometry_conditioned_validation_v1/oracle/VUE036.coarse.requests")
    refine_request = Path("/tmp/p3_geometry_conditioned_validation_v1/oracle/VUE036.refine.requests")

    def parse_requests(path: Path) -> list[dict]:
        rows = []
        for line in path.read_text(encoding="utf-8").splitlines()[1:]:
            parts = line.split("\t")
            if len(parts) != 8:
                continue
            rows.append({
                "phase": parts[1], "relaxed": int(parts[2]),
                "side": "LEFT" if int(parts[3]) else "RIGHT",
                "d_target": float(parts[4]), "d_mid": float(parts[5]),
                "entry_scale": float(parts[6]), "exit_scale": float(parts[7]),
            })
        return rows

    coarse = parse_requests(coarse_request)
    refine = parse_requests(refine_request)
    same_factor_coarse = [row for row in coarse if row["side"] == "RIGHT"
                          and close(row["entry_scale"], entry)
                          and close(row["exit_scale"], exit_scale)]
    same_factor_refine = [row for row in refine if row["side"] == "RIGHT"
                          and close(row["entry_scale"], entry)
                          and close(row["exit_scale"], exit_scale)]
    nearest_coarse = min(same_factor_coarse, key=lambda row: math.hypot(
        row["d_target"] - target, row["d_mid"] - middle))
    nearest_refine = min(same_factor_refine, key=lambda row: math.hypot(
        row["d_target"] - target, row["d_mid"] - middle))
    refinement_covers = any(abs(row["d_target"] - target) <= 0.04 + EPS and
                            abs(row["d_mid"] - middle) <= 0.04 + EPS
                            for row in same_factor_refine)

    def violation_score(row: dict) -> tuple[int, float]:
        if row.get("validator_executed") != "1":
            return 99, 99.0
        checks = [
            -num(row, "footprint_track_margin_m") / .01,
            -num(row, "obstacle_margin_m") / .01,
            -num(row, "lateral_slope_margin") / .02,
            -num(row, "signed_curvature_margin_radpm") / .02,
            -num(row, "curvature_rate_margin_radpm2") / .2,
            num(row, "braking_deficit_m") / .05,
        ]
        violations = [max(0.0, value) for value in checks
                      if math.isfinite(value) and value > 0.0]
        count = len(violations) or (0 if row.get("hard_valid") == "1" else 1)
        return count, max(violations, default=(0.0 if row.get("hard_valid") == "1" else 1.0))

    coarse_output = method.records(
        Path("/tmp/p3_geometry_conditioned_validation_v1/oracle/VUE036.coarse.out"),
        "CANDIDATE")
    ordered_coarse = sorted((row for row in coarse_output if row["phase"] == "COARSE"),
                            key=violation_score)
    nearby_ranks = []
    for rank, row in enumerate(ordered_coarse, 1):
        if (row["side"] == "RIGHT" and close(row["entry_scale"], entry)
                and close(row["exit_scale"], exit_scale)
                and abs(num(row, "d_target") - target) <= .04 + EPS
                and abs(num(row, "d_mid") - middle) <= .04 + EPS):
            nearby_ranks.append(rank)

    # Test the predeclared 0.01-m lattice locally without changing its resolution.
    # This distinguishes seed selection from resolution as the proximate miss.
    local_grid = []
    for target_i in np.arange(-.71, -.639999, .01):
        for middle_i in np.arange(-.76, -.689999, .01):
            local_grid.append({
                "side": "RIGHT", "d_target": float(target_i), "d_mid": float(middle_i),
                "entry_scale": entry, "exit_scale": exit_scale,
            })
    local_request = WORK / "VUE036.LOCAL_0P01_GRID.requests"
    local_output = WORK / "VUE036.LOCAL_0P01_GRID.out"
    method.write_requests(local_request, "LOCAL_0P01_GRID_DIAGNOSIS", local_grid)
    with local_output.open("w", encoding="utf-8", newline="\n") as stream:
        subprocess.run([
            str(HARNESS),
            str(VALIDATION / "raw_oracle/inputs/VUE036.event"), str(local_request),
        ], check=True, stdout=stream)
    local_rows = method.records(local_output, "CANDIDATE")
    local_valid = [row for row in local_rows if row["hard_valid"] == "1"]
    local_best = min(local_valid, key=method.oracle_rank) if local_valid else None

    witness = next(row for row in selected_rows
                   if row["validation_event_id"] == event_id
                   and row["selector"] == "H4A_GEOMETRY_TRANSITION"
                   and row["path_digest"] == valid_digest)
    output_rows = []
    for selector in ("H3_FIXED_BOUNDED", "H4A_GEOMETRY_TRANSITION",
                     "H4B_GEOMETRY_LATERAL_TRANSITION"):
        row = next(item for item in method_rows
                   if item["validation_event_id"] == event_id
                   and item["selector"] == selector)
        audit = next(item for item in selected_rows
                     if item["validation_event_id"] == event_id
                     and item["selector"] == selector
                     and item["path_digest"] == valid_digest)
        output_rows.append({
            "validation_event_id": event_id, "record_kind": "SELECTOR_VALID_WITNESS",
            "selector_or_search": selector, "selection_rank": audit["selection_rank"],
            "side": row["selected_side"], "d_target": row["selected_d_target"],
            "d_mid": row["selected_d_mid"], "entry_scale": row["selected_entry_scale"],
            "exit_scale": row["selected_exit_scale"],
            "target_source": audit["target_source"], "mid_source": audit["mid_source"],
            "transition_factor_source": "PRODUCTION_M1_ZERO_BOUNDARY_SHORT_TUPLE",
            "template": "DIRECT_FACTORIZED_P3_RECONSTRUCTION",
            "probe_root_source": "NOT_AN_ANALYTIC_ROOT_REQUEST",
            "construction_guard": "PASSED_29_POINTS",
            "dedup_status": "UNIQUE_PATH_DIGEST", "path_digest": valid_digest,
            "hard_valid": True,
            "oracle_miss_classification": "ORACLE_REFINEMENT_MISS",
        })
    oracle = oracle_by_id[event_id]
    output_rows.append({
        "validation_event_id": event_id, "record_kind": "ORACLE_SEARCH_CONTRACT",
        "selector_or_search": "FROZEN_BROAD_ORACLE_V1",
        "side": "RIGHT", "d_target": target, "d_mid": middle,
        "entry_scale": entry, "exit_scale": exit_scale,
        "target_source": "VALID_WITNESS_NOT_ORACLE_GRID_ANCHOR",
        "mid_source": "VALID_WITNESS_NOT_ORACLE_GRID_ANCHOR",
        "transition_factor_source": "ENUMERATED_PRODUCTION_TRANSITION_PAIR",
        "template": "DIRECT_P3_PARAMETER_SEARCH",
        "probe_root_source": "SEARCHES_D_TARGET_D_MID_NOT_ANALYTIC_ROOT_LINEAGE",
        "strict_right_domain": "[-0.9850000000000001,-0.5684527692629249]",
        "d_mid_domain": "[-1.5,1.5]", "coarse_grid_step_m": 0.05,
        "refinement_seed_rule": "TOP_30_BY_NORMALIZED_VIOLATION_SCORE",
        "refinement_half_width_m": 0.04, "refinement_grid_step_m": 0.01,
        "coarse_request_count": oracle["coarse_request_count"],
        "refine_request_count": oracle["refine_request_count"],
        "exact_tuple_in_coarse": False, "exact_tuple_in_refine": False,
        "nearest_same_factor_coarse_d_target": nearest_coarse["d_target"],
        "nearest_same_factor_coarse_d_mid": nearest_coarse["d_mid"],
        "nearest_same_factor_coarse_distance_2d": math.hypot(
            nearest_coarse["d_target"] - target, nearest_coarse["d_mid"] - middle),
        "nearest_same_factor_refine_d_target": nearest_refine["d_target"],
        "nearest_same_factor_refine_d_mid": nearest_refine["d_mid"],
        "nearest_same_factor_refine_distance_2d": math.hypot(
            nearest_refine["d_target"] - target, nearest_refine["d_mid"] - middle),
        "refinement_requests_cover_valid_witness_window": refinement_covers,
        "nearby_coarse_violation_rank_min": min(nearby_ranks),
        "nearby_coarse_violation_ranks": "|".join(map(str, nearby_ranks)),
        "top30_boundary_violation_score": str(violation_score(ordered_coarse[29])),
        "nearby_coarse_violation_score": str(violation_score(ordered_coarse[nearby_ranks[0]-1])),
        "local_0p01_grid_request_count": len(local_rows),
        "local_0p01_grid_hard_valid_count": len(local_valid),
        "local_0p01_grid_best_d_target": local_best["d_target"] if local_best else "",
        "local_0p01_grid_best_d_mid": local_best["d_mid"] if local_best else "",
        "local_0p01_grid_best_path_digest": local_best["path_digest"] if local_best else "",
        "construction_guard": "WITNESS_PASSES; NO_GUARD_REJECTION",
        "dedup_status": "WITNESS_DIGEST_NOT_PRESENT_IN_ORACLE_OUTPUT",
        "path_digest": valid_digest, "hard_valid": True,
        "oracle_miss_classification": "ORACLE_REFINEMENT_MISS",
    })
    write_csv(HERE / "vue036_oracle_miss_diagnosis.csv", output_rows)


def hard_vs_usable(method_rows: list[dict], full_pool: dict[str, dict]) -> None:
    rows = []
    recovered = [row for row in method_rows
                 if row["selector"] == "H4A_GEOMETRY_TRANSITION"
                 and truth(row["recovered"])]
    for row in recovered:
        is_usable, reasons = usable(row)
        event_id = row["validation_event_id"]
        full = full_pool[event_id]
        rows.append({
            "validation_event_id": event_id,
            "witness_scope": "FROZEN_H4A_K24_SELECTED_BEST_HARD_VALID",
            "exact_hard_valid": True,
            "side": row["selected_side"], "d_target": row["selected_d_target"],
            "d_mid": row["selected_d_mid"],
            "footprint_track_margin_m": row["selected_footprint_track_margin_m"],
            "lateral_slope_margin": row["selected_lateral_slope_margin"],
            "signed_curvature_margin_radpm": row["selected_signed_curvature_margin_radpm"],
            "curvature_rate_margin_radpm2": row["selected_curvature_rate_margin_radpm2"],
            "obstacle_diagnostic_margin_m": row["selected_obstacle_margin_m"],
            "minimum_normalized_safety_slack": row["selected_minimum_normalized_safety_slack"],
            "exit_reaches_next_obstacle": row["selected_exit_reaches_next_obstacle"],
            "braking_deficit_m": row["selected_braking_deficit_m"],
            "velocity_loss": row["selected_velocity_loss"],
            "global_path_deviation_m": row["selected_global_path_deviation_m"],
            "velocity_loss_interpretation": "RECORDED_QUALITY_DIAGNOSTIC_NO_FROZEN_THRESHOLD",
            "negative_obstacle_diagnostic": num(row, "selected_obstacle_margin_m") < -EPS,
            "negative_normalized_slack":
                num(row, "selected_minimum_normalized_safety_slack") < -EPS,
            "next_obstacle_exit_conflict": row["selected_exit_reaches_next_obstacle"] == "1",
            "braking_concern": num(row, "selected_braking_deficit_m", 0.0) > EPS,
            "usable_valid_p3": is_usable,
            "usable_exclusion_reasons": "|".join(reasons),
            "any_usable_witness_in_complete_factor_pool": bool(full["usable"]),
            "first_usable_h4a_order_rank": full["first_usable_rank"] or "",
            "criterion_version": "USABLE_VALID_P3_DIAGNOSIS_V1",
        })
    write_csv(HERE / "hard_vs_usable_valid.csv", rows)


def unrecovered_taxonomy(method_rows: list[dict], oracle_rows: list[dict],
                         full_pool: dict[str, dict]) -> None:
    oracle_by_id = {row["event_id"]: row for row in oracle_rows}
    h4a = {row["validation_event_id"]: row for row in method_rows
           if row["selector"] == "H4A_GEOMETRY_TRANSITION"}
    event_ids = [event_id for event_id, row in h4a.items()
                 if row["oracle_classification"] == "ORACLE_VALID_P3_EXISTS"
                 and not truth(row["recovered"])]
    rows = []
    for event_id in sorted(event_ids):
        oracle = oracle_by_id[event_id]
        full = full_pool[event_id]
        pool = full["pool"]
        exact_target = any(close(candidate["d_target"], oracle["best_d_target"])
                           for candidate in pool)
        exact_mid = any(close(candidate["d_mid"], oracle["best_d_mid"])
                        for candidate in pool)
        exact_lateral = any(close(candidate["d_target"], oracle["best_d_target"])
                            and close(candidate["d_mid"], oracle["best_d_mid"])
                            for candidate in pool)
        exact_tuple = any(candidate["side"] == oracle["best_side"]
                          and close(candidate["d_target"], oracle["best_d_target"])
                          and close(candidate["d_mid"], oracle["best_d_mid"])
                          and close(candidate["entry_scale"], oracle["best_entry_scale"])
                          and close(candidate["exit_scale"], oracle["best_exit_scale"])
                          for candidate in pool)
        first_rank = full["first_hard_rank"]
        if first_rank is not None and first_rank > 24:
            primary = "CANDIDATE_BUDGET_TRUNCATION"
            secondary = "GEOMETRY_RULE_GENERALIZATION_FAILURE"
        elif oracle["mapping_attribution"].startswith("SAME_D_TARGET"):
            primary = "PROBE_ROOT_ISSUE"
            secondary = "LATERAL_FACTOR_MISS" if not exact_lateral else "OTHER_LOCAL_BASIS_MISS"
        elif oracle["mapping_attribution"] == "VALID_REQUIRES_DIFFERENT_D_TARGET_AND_D_MID":
            primary = "MULTI_PARAMETER_INTERACTION"
            secondary = "LATERAL_FACTOR_MISS" if not exact_lateral else "OTHER_LOCAL_BASIS_MISS"
        else:
            primary = "OTHER"
            secondary = ""
        rows.append({
            "validation_event_id": event_id,
            "oracle_mapping_attribution": oracle["mapping_attribution"],
            "oracle_best_side": oracle["best_side"],
            "oracle_best_d_target": oracle["best_d_target"],
            "oracle_best_d_mid": oracle["best_d_mid"],
            "oracle_best_entry_scale": oracle["best_entry_scale"],
            "oracle_best_exit_scale": oracle["best_exit_scale"],
            "h4a_k24_recovered": False,
            "complete_factor_pool_size": len(pool),
            "complete_factor_pool_unique_constructed": len(full["effective"]),
            "complete_factor_pool_hard_valid_count": len(full["hard"]),
            "first_hard_valid_h4a_order_rank": first_rank or "",
            "hard_valid_exists_beyond_k24": first_rank is not None and first_rank > 24,
            "oracle_best_target_present_in_factor_pool": exact_target,
            "oracle_best_mid_present_in_factor_pool": exact_mid,
            "oracle_best_lateral_pair_present_in_factor_pool": exact_lateral,
            "oracle_best_exact_tuple_present_in_factor_pool": exact_tuple,
            "primary_mechanism": primary, "secondary_mechanism": secondary,
            "candidate_budget_k": 24,
            "classification_contract": "DESCRIPTIVE_NO_H4A_REDESIGN",
        })
    write_csv(HERE / "validation_unrecovered_taxonomy.csv", rows)


def mechanism_comparison(method_rows: list[dict]) -> None:
    development = read_csv(METHOD / "development_results.csv")
    dev = [row for row in development
           if row["selector"] == "H4A_GEOMETRY_TRANSITION"
           and row["budget_k"] == "24"
           and truth(row["oracle_valid_in_full_development_oracle"])]
    val = [row for row in method_rows
           if row["selector"] == "H4A_GEOMETRY_TRANSITION"
           and row["oracle_classification"] == "ORACLE_VALID_P3_EXISTS"]
    mapping = {
        "D_PROBE_DOMINANT": "PROBE_ROOT_ISSUE",
        "S_PROBE_DOMINANT": "PROBE_ROOT_ISSUE",
        "TEMPLATE_OR_TRANSITION_SELECTION": "TRANSITION_SELECTOR_MISS",
        "OTHER": "OTHER",
    }
    dev_unrecovered = Counter(mapping[row["failure_mechanism"]]
                              for row in dev if not truth(row["recovered"]))
    val_tax = read_csv(HERE / "validation_unrecovered_taxonomy.csv")
    val_unrecovered = Counter(row["primary_mechanism"] for row in val_tax)
    categories = sorted(set(dev_unrecovered) | set(val_unrecovered))
    rows = []
    for category in categories:
        rows.append({
            "mechanism": category,
            "development_oracle_feasible_count": len(dev),
            "development_unrecovered_count": dev_unrecovered[category],
            "development_unrecovered_fraction": dev_unrecovered[category] / len(dev),
            "validation_oracle_feasible_count": len(val),
            "validation_unrecovered_count": val_unrecovered[category],
            "validation_unrecovered_fraction": val_unrecovered[category] / len(val),
            "comparison_note": "descriptive taxonomy; labels differ in source granularity",
        })
    write_csv(HERE / "development_vs_validation_mechanisms.csv", rows)


def geometry_comparison(oracle_rows: list[dict]) -> None:
    dev_summary = read_csv(CORPUS / "raw_oracle/oracle_summary.csv")
    dev_valid = {row["event_id"] for row in dev_summary
                 if row["classification"] == "ORACLE_VALID_P3_EXISTS"}
    val_valid = {row["event_id"] for row in oracle_rows
                 if row["classification"] == "ORACLE_VALID_P3_EXISTS"}
    dev_features = [row for row in read_csv(METHOD / "geometry_features.csv")
                    if row["development_event_id"] in dev_valid]
    val_features = [row for row in read_csv(VALIDATION / "validation_geometry_features.csv")
                    if row["development_event_id"] in val_valid]
    dev_best = {row["event_id"]: row["best_side"] for row in dev_summary
                if row["event_id"] in dev_valid}
    val_best = {row["event_id"]: row["best_side"] for row in oracle_rows
                if row["event_id"] in val_valid}

    features = [
        "ego_d_m", "ego_speed_mps", "obstacle_start_distance_m",
        "available_entry_distance_m", "obstacle_longitudinal_span_m",
        "chosen_side_free_width_m", "corridor_bottleneck_width_m",
        "corridor_bottleneck_station_m", "available_merge_exit_distance_m",
        "maximum_abs_reference_curvature_radpm", "track_width_variation_m",
        "corridor_component_count",
    ]
    rows = []
    for scope in ("ALL_GENERATED_SIDES", "ORACLE_BEST_SIDE"):
        if scope == "ORACLE_BEST_SIDE":
            drows = [row for row in dev_features
                     if row["side"] == dev_best[row["development_event_id"]]]
            vrows = [row for row in val_features
                     if row["side"] == val_best[row["development_event_id"]]]
        else:
            drows, vrows = dev_features, val_features
        for feature in features:
            dv = np.asarray([num(row, feature) for row in drows], dtype=float)
            vv = np.asarray([num(row, feature) for row in vrows], dtype=float)
            pooled = math.sqrt((float(np.var(dv, ddof=1)) + float(np.var(vv, ddof=1))) / 2.0)
            smd = (float(np.mean(vv)) - float(np.mean(dv))) / pooled if pooled > EPS else 0.0
            rows.append({
                "scope": scope, "feature": feature,
                "development_count": len(dv), "validation_count": len(vv),
                "development_mean": float(np.mean(dv)),
                "development_p10": float(np.percentile(dv, 10)),
                "development_p50": float(np.percentile(dv, 50)),
                "development_p90": float(np.percentile(dv, 90)),
                "validation_mean": float(np.mean(vv)),
                "validation_p10": float(np.percentile(vv, 10)),
                "validation_p50": float(np.percentile(vv, 50)),
                "validation_p90": float(np.percentile(vv, 90)),
                "standardized_mean_difference_validation_minus_development": smd,
            })
        for rule, predicate in (
            ("H4A_DESIRED_ENTRY_LONG", lambda row: num(row, "available_entry_distance_m") < .25
             or num(row, "maximum_abs_reference_curvature_radpm") > .5
             or num(row, "obstacle_start_distance_m") > 2.0),
            ("H4A_DESIRED_EXIT_SHORT", lambda row: num(row, "available_merge_exit_distance_m") < 2.0),
            ("H4A_DESIRED_EXIT_LONG", lambda row: num(row, "available_merge_exit_distance_m") >= 2.0
             and (truth(row["curvature_sign_change"])
                  or num(row, "track_width_variation_m") > .6)),
        ):
            rows.append({
                "scope": scope, "feature": rule,
                "development_count": len(drows), "validation_count": len(vrows),
                "development_mean": sum(predicate(row) for row in drows) / len(drows),
                "validation_mean": sum(predicate(row) for row in vrows) / len(vrows),
                "standardized_mean_difference_validation_minus_development": "",
                "interpretation": "fraction activating frozen binary geometry rule",
            })
    write_csv(HERE / "geometry_distribution_comparison.csv", rows)


def protocol_overlay() -> None:
    rows = read_csv(VALIDATION / "validation_manifest.csv")
    if len(rows) != 37 or any(row["final_holdout_accessed"] != "False" for row in rows):
        raise RuntimeError("validation-only manifest guard failed")
    output = []
    for row in rows:
        output.append({
            "validation_event_id": row["validation_event_id"],
            "episode_id": row["episode_id"], "bag": row["bag"],
            "original_dataset_role": row["dataset_role"],
            "post_v1_protocol_role": "VALIDATION_SEEN_AFTER_V1",
            "dataset_split_manifest_modified": False,
            "final_holdout_accessed": False,
            "protocol_note": "post-analysis overlay only; frozen split and holdout untouched",
        })
    write_csv(HERE / "validation_seen_after_v1_manifest.csv", output)


def main() -> None:
    if not HARNESS.is_file() or sha256(HARNESS) != EXPECTED_HARNESS_SHA256:
        raise RuntimeError("frozen exact-validator harness missing or changed")
    method = load_method()
    method_rows = read_csv(VALIDATION / "validation_method_results.csv")
    selected_rows = read_csv(VALIDATION / "validation_selected_candidates.csv")
    oracle_rows = read_csv(VALIDATION / "validation_oracle_summary.csv")
    if len(oracle_rows) != 37:
        raise RuntimeError("expected exactly 37 validation rows")
    oracle_by_id = {row["event_id"]: row for row in oracle_rows}
    full_pool = full_pool_audit(method, oracle_rows)
    vue036_audit(method, method_rows, selected_rows, oracle_by_id, full_pool)
    hard_vs_usable(method_rows, full_pool)
    unrecovered_taxonomy(method_rows, oracle_rows, full_pool)
    mechanism_comparison(method_rows)
    geometry_comparison(oracle_rows)
    protocol_overlay()

    completeness = []
    for row in oracle_rows:
        if row["classification"] != "NO_VALID_P3_FOUND_IN_ORACLE_DOMAIN":
            continue
        full = full_pool[row["event_id"]]
        completeness.append({
            "validation_event_id": row["event_id"],
            "old_oracle_classification": row["classification"],
            "old_oracle_hard_valid_count": row["oracle_hard_valid_count"],
            "complete_existing_factor_pool_hard_valid_count": len(full["hard"]),
            "complete_existing_factor_pool_first_hard_valid_h4a_order_rank":
                full["first_hard_rank"] or "",
            "same_issue_directly_demonstrated": row["event_id"] == "VUE036",
            "old_result_relabelled": False,
            "interpretation": (
                "EXACT_VALID_WITNESS_OUTSIDE_SAMPLED_ORACLE_REQUESTS" if full["hard"] else
                "NO_ADDITIONAL_WITNESS_IN_EXISTING_FACTORIZED_POOL_NOT_PROOF_OF_INFEASIBILITY"
            ),
        })
    write_csv(HERE / "oracle_negative_case_audit.csv", completeness)


if __name__ == "__main__":
    main()
