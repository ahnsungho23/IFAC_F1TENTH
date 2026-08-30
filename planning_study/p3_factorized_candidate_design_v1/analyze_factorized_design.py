#!/usr/bin/env python3
"""DEVELOPMENT-only offline factorization study for the frozen P3 family."""

from __future__ import annotations

import csv
import hashlib
import importlib.util
import itertools
import json
import math
import os
import subprocess
import time
from collections import Counter, defaultdict
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/p3_factorized_design_mpl")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
CORPUS = REPO / "planning_study/p3_mapping_research_corpus_v1"
RAW = CORPUS / "raw_oracle"
TEMPLATE_DIAG = REPO / "planning_study/p3_template_transition_diagnosis_v1"
GEOMETRY_SOURCE = REPO / "planning_study/p3_mapping_miss_diagnosis_v1/analyze_mapping_miss.py"
HARNESS = Path("/tmp/p3_oracle_v2_build/p3_family_oracle_harness")
HARNESS_SOURCE = Path("/tmp/p3_oracle_v2_src/src/local_planning/test/p3_family_oracle_harness.cpp")
WORK = Path("/tmp/p3_factorized_candidate_design_v1")
PLOTS = HERE / "plots"
EXPECTED_HARNESS_SHA256 = "8b23f2b6df54c2e93794ee087f3ebd7a14f9921544af4f97eef755535e045e7e"
EPS = 1.0e-9

VARIANTS = [
    "BASELINE_PRODUCTION",
    "A_EXISTING_D_MID_POOL",
    "A_UNCOUPLE_D_MID_DEV_COUNTERFACTUAL",
    "B_CROSS_PAIR_TRANSITIONS",
    "AB_EXISTING_FACTOR_BASIS",
    "AB_DEV_COUNTERFACTUAL",
    "FULL_LxT_DEV_UPPER_BOUND",
]


def load_geometry():
    spec = importlib.util.spec_from_file_location("development_geometry", GEOMETRY_SOURCE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


G = load_geometry()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_csv(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


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


def truth(value) -> bool:
    return str(value).lower() in {"1", "true", "yes"}


def number(row: dict, key: str, default: float = math.nan) -> float:
    value = row.get(key, "")
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def rounded(value: float) -> float:
    return round(float(value), 12)


def lateral_key(row: dict) -> tuple[str, float, float]:
    # Preserve exact binary-double values. Analytic roots that differ below 1e-12 can still
    # produce different sampled path digests, so canonical rounding is only for aggregate plots.
    return row["side"], number(row, "d_target"), number(row, "d_mid")


def transition_key(row: dict) -> tuple[float, float]:
    return number(row, "entry_scale"), number(row, "exit_scale")


def config_key(row: dict) -> tuple[str, float, float, float, float]:
    return lateral_key(row) + transition_key(row)


def key_text(values: tuple) -> str:
    return "|".join(str(value) for value in values)


def stable_id(prefix: str, event_id: str, key: tuple) -> str:
    digest = hashlib.sha256((event_id + "|" + key_text(key)).encode()).hexdigest()[:12]
    return f"{prefix}_{event_id}_{digest}"


def aggregate_sources(rows: list[dict]) -> str:
    values = sorted({
        ":".join(filter(None, [row.get("generator_stage", ""), row.get("template", ""),
                               row.get("root_type", "")]))
        for row in rows
    })
    return "|".join(value for value in values if value)


def make_factors(event_id: str, production: list[dict], valid: list[dict]):
    prod_lateral_groups: dict[tuple, list[dict]] = defaultdict(list)
    transition_groups: dict[tuple, list[dict]] = defaultdict(list)
    valid_lateral_groups: dict[tuple, list[dict]] = defaultdict(list)
    for row in production:
        prod_lateral_groups[lateral_key(row)].append(row)
        transition_groups[transition_key(row)].append(row)
    for row in valid:
        valid_lateral_groups[lateral_key(row)].append(row)

    prod_lateral = {}
    for key, rows in prod_lateral_groups.items():
        exemplar = rows[0]
        prod_lateral[key] = {
            "id": stable_id("LP", event_id, key),
            "side": key[0], "d_target": number(exemplar, "d_target"),
            "d_mid": number(exemplar, "d_mid"),
            "origin": "PRODUCTION_LATERAL",
            "source": aggregate_sources(rows),
            "probe_sources": "|".join(sorted({row.get("mapping_source", "") for row in rows})),
        }
    transitions = {}
    for key, rows in transition_groups.items():
        exemplar = rows[0]
        transitions[key] = {
            "id": stable_id("TP", event_id, key),
            "entry_scale": number(exemplar, "entry_scale"),
            "exit_scale": number(exemplar, "exit_scale"),
            "origin": "PRODUCTION_TRANSITION",
            "source": aggregate_sources(rows),
        }

    dev_lateral = {}
    for key, rows in valid_lateral_groups.items():
        exemplar = rows[0]
        dev_lateral[key] = {
            "id": stable_id("LD", event_id, key),
            "side": key[0], "d_target": number(exemplar, "d_target"),
            "d_mid": number(exemplar, "d_mid"),
            "origin": "DEVELOPMENT_ORACLE_VALID_LATERAL",
            "source": "DIRECT_P3_STRICT_HARD_VALID",
            "probe_sources": "NONE_DIRECT_RECONSTRUCTION",
        }
    return prod_lateral, transitions, dev_lateral


def add_config(destination: dict, lateral: dict, transition: dict, origin: str) -> None:
    key = (
        lateral["side"], lateral["d_target"], lateral["d_mid"],
        transition["entry_scale"], transition["exit_scale"],
    )
    if key not in destination:
        destination[key] = {
            "key": key,
            "side": lateral["side"], "d_target": lateral["d_target"],
            "d_mid": lateral["d_mid"], "entry_scale": transition["entry_scale"],
            "exit_scale": transition["exit_scale"],
            "lateral_factor_id": lateral["id"],
            "lateral_factor_origin": lateral["origin"],
            "lateral_factor_source": lateral["source"],
            "transition_factor_id": transition["id"],
            "transition_factor_origin": transition["origin"],
            "transition_factor_source": transition["source"],
            "combination_origins": set(),
        }
    destination[key]["combination_origins"].add(origin)


def best_dev_mid_counterfactuals(production: list[dict], strict_valid: list[dict]) -> dict[tuple, dict]:
    """One nearest already-open hard-valid d_mid for each M1 zero-interface current pairing."""
    output = {}
    for zero in production:
        if zero.get("generator_stage") != "M1" or zero.get("root_type") != "ZERO_INTERFACE":
            continue
        matches = [
            row for row in strict_valid
            if row["side"] == zero["side"]
            and abs(number(row, "d_target") - number(zero, "d_target")) <= EPS
            and abs(number(row, "entry_scale") - number(zero, "entry_scale")) <= EPS
            and abs(number(row, "exit_scale") - number(zero, "exit_scale")) <= EPS
            and abs(number(row, "d_mid") - number(zero, "d_mid")) > 1.0e-6
        ]
        if not matches:
            continue
        chosen = min(matches, key=lambda row: (
            abs(number(row, "d_mid") - number(zero, "d_mid")),
            -number(row, "minimum_normalized_safety_slack"),
            number(row, "request_index"),
        ))
        output[config_key(zero)] = chosen
    return output


def build_variants(event_id: str, production: list[dict], strict_valid: list[dict]):
    prod_lateral, transitions, dev_lateral = make_factors(event_id, production, strict_valid)
    baseline = {}
    production_keys = set()
    for row in production:
        l = prod_lateral[lateral_key(row)]
        t = transitions[transition_key(row)]
        add_config(baseline, l, t, "EXACT_PRODUCTION_PAIR")
        production_keys.add(config_key(row))

    # Online-current-value diagnostic: reuse d_mid values already generated on the same side.
    existing_mid_lateral = dict(prod_lateral)
    existing_a = dict(baseline)
    mids_by_side: dict[str, set[float]] = defaultdict(set)
    for lateral in prod_lateral.values():
        mids_by_side[lateral["side"]].add(lateral["d_mid"])
    for zero in production:
        if zero.get("generator_stage") != "M1" or zero.get("root_type") != "ZERO_INTERFACE":
            continue
        transition = transitions[transition_key(zero)]
        for middle in sorted(mids_by_side[zero["side"]]):
            key = (zero["side"], number(zero, "d_target"), middle)
            lateral = existing_mid_lateral.get(key)
            if lateral is None:
                lateral = {
                    "id": stable_id("LE", event_id, key), "side": zero["side"],
                    "d_target": number(zero, "d_target"), "d_mid": middle,
                    "origin": "PRODUCTION_EXISTING_D_MID_RECOMBINATION",
                    "source": "M1_ZERO_TARGET_X_SAME_SIDE_PRODUCTION_D_MID",
                }
                existing_mid_lateral[key] = lateral
            add_config(existing_a, lateral, transition, "A_EXISTING_D_MID_CURRENT_PAIRING")

    # Mechanism-isolation upper bound: one nearest hard-valid DEVELOPMENT d_mid per zero pairing.
    dev_cf_lateral = dict(prod_lateral)
    dev_a = dict(baseline)
    dev_counterfactuals = best_dev_mid_counterfactuals(production, strict_valid)
    for zero_key, valid_row in dev_counterfactuals.items():
        zero = next(row for row in production if config_key(row) == zero_key)
        key = lateral_key(valid_row)
        lateral = dev_cf_lateral.get(key)
        if lateral is None:
            lateral = {
                "id": stable_id("LC", event_id, key), "side": key[0],
                "d_target": number(valid_row, "d_target"), "d_mid": number(valid_row, "d_mid"),
                "origin": "DEV_ORACLE_MINIMUM_D_MID_COUNTERFACTUAL",
                "source": "SAME_SIDE_TARGET_TRANSITION_STRICT_HARD_VALID",
            }
            dev_cf_lateral[key] = lateral
        add_config(dev_a, lateral, transitions[transition_key(zero)],
                   "A_DEV_COUNTERFACTUAL_CURRENT_PAIRING")

    b_cross = {}
    for lateral in prod_lateral.values():
        for transition in transitions.values():
            add_config(b_cross, lateral, transition, "B_PRODUCTION_L_X_PRODUCTION_T")

    ab_existing = {}
    for lateral in existing_mid_lateral.values():
        for transition in transitions.values():
            add_config(ab_existing, lateral, transition, "AB_EXISTING_L_X_PRODUCTION_T")

    ab_dev = {}
    for lateral in dev_cf_lateral.values():
        for transition in transitions.values():
            add_config(ab_dev, lateral, transition, "AB_DEV_CF_L_X_PRODUCTION_T")

    full = {}
    full_lateral = dict(prod_lateral)
    for key, lateral in dev_lateral.items():
        if key not in full_lateral:
            full_lateral[key] = lateral
    for lateral in full_lateral.values():
        for transition in transitions.values():
            add_config(full, lateral, transition, "FULL_DEV_L_X_PRODUCTION_T")

    variants = {
        "BASELINE_PRODUCTION": baseline,
        "A_EXISTING_D_MID_POOL": existing_a,
        "A_UNCOUPLE_D_MID_DEV_COUNTERFACTUAL": dev_a,
        "B_CROSS_PAIR_TRANSITIONS": b_cross,
        "AB_EXISTING_FACTOR_BASIS": ab_existing,
        "AB_DEV_COUNTERFACTUAL": ab_dev,
        "FULL_LxT_DEV_UPPER_BOUND": full,
    }
    return variants, prod_lateral, transitions, dev_counterfactuals, production_keys


def write_requests(path: Path, variant: str, configurations: list[dict]) -> None:
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write("P3_ORACLE_REQUESTS_V1\n")
        for config in configurations:
            stream.write(
                "Q\t{}\t0\t{}\t{:.17g}\t{:.17g}\t{:.17g}\t{:.17g}\n".format(
                    variant, int(config["side"] == "LEFT"), config["d_target"], config["d_mid"],
                    config["entry_scale"], config["exit_scale"],
                )
            )


def records(path: Path, kind: str) -> list[dict]:
    rows = [line.split("\t") for line in path.read_text(encoding="utf-8").splitlines()
            if line.startswith(kind + "\t")]
    if not rows:
        return []
    header = rows[0][1:]
    return [dict(zip(header, row[1:])) for row in rows[1:]]


def run_harness(event_id: str, event_path: Path, variant: str,
                configurations: list[dict]) -> tuple[list[dict], float, Path]:
    request_path = WORK / f"{event_id}.{variant}.requests"
    output_path = WORK / f"{event_id}.{variant}.out"
    write_requests(request_path, variant, configurations)
    started = time.perf_counter()
    with output_path.open("w", encoding="utf-8", newline="\n") as stream:
        subprocess.run([str(HARNESS), str(event_path), str(request_path)],
                       check=True, stdout=stream)
    runtime = time.perf_counter() - started
    return records(output_path, "CANDIDATE"), runtime, output_path


def empty_runtime_and_parity(event_id: str, event_path: Path, production: list[dict]) -> float:
    request_path = WORK / f"{event_id}.EMPTY.requests"
    output_path = WORK / f"{event_id}.EMPTY.out"
    write_requests(request_path, "EMPTY", [])
    started = time.perf_counter()
    with output_path.open("w", encoding="utf-8", newline="\n") as stream:
        subprocess.run([str(HARNESS), str(event_path), str(request_path)],
                       check=True, stdout=stream)
    runtime = time.perf_counter() - started
    got = Counter(row["path_digest"] for row in records(output_path, "BASELINE_CONSTRUCTED_CANDIDATE"))
    expected = Counter(row["path_digest"] for row in production)
    if got != expected:
        raise RuntimeError(f"{event_id}: frozen baseline digest parity failed")
    return runtime


def percentile(values: list[float], q: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=float), q)) if values else math.nan


def oracle_rank(row: dict):
    return (
        int(row.get("exit_reaches_next_obstacle", "0")),
        number(row, "braking_deficit_m") > EPS,
        number(row, "braking_deficit_m"),
        number(row, "velocity_loss"),
        -number(row, "minimum_normalized_safety_slack"),
        number(row, "global_path_deviation_m"),
        int(row.get("request_index", "0")),
    )


def coupling_semantics(row: dict) -> tuple[str, str, str]:
    stage, template, root = row["generator_stage"], row["template"], row["root_type"]
    if stage == "M0_V1":
        lateral = "existingTargetAnchor + CURVATURE_CONTINUITY probe analytic d_mid root"
        transition = "M0 adaptive entry endpoint/bisection x three exit ratios"
        skipped = "only canonical retained side proceeds; V2/M1 run only when M0 has no hard-valid"
    elif stage == "M0_V2" and root == "ZERO_INTERFACE":
        lateral = "ranked branch/domain target with d_mid=d_target"
        transition = f"fixed {template} transition pairing"
        skipped = "other M0-V2 lateral/transition cross-pairs are not formed; cap 12/remaining total"
    elif stage == "M0_V2":
        lateral = "BOTTLENECK_CENTER analytic d_mid root at ranked near target"
        transition = f"fixed {template} transition pairing"
        skipped = "root is not crossed with other existing transition templates"
    elif root == "ZERO_INTERFACE":
        lateral = "boundary_inset target with hard coupling d_mid=d_target; no probe/root"
        transition = "SHORT=(entry_min,exit_min) or SPAN=(entry_quarter,exit_span)"
        skipped = "alternate d_mid and cross-template transition combinations impossible"
    else:
        lateral = "branch near/far target + BOTTLENECK_CENTER active/inactive analytic d_mid root"
        transition = f"fixed {template} transition pairing"
        skipped = "analytic lateral result is not crossed with other event transition pairs"
    return lateral, transition, skipped


def current_coupling_rows(production_by_event: dict[str, list[dict]]) -> list[dict]:
    output = []
    for event_id in sorted(production_by_event):
        production = production_by_event[event_id]
        prod_lateral, transitions, _ = make_factors(event_id, production, [])
        for row in production:
            lateral_semantics, transition_semantics, skipped = coupling_semantics(row)
            l = prod_lateral[lateral_key(row)]
            t = transitions[transition_key(row)]
            output.append({
                "development_event_id": event_id,
                "generation_order": row["generation_order"],
                "generator_stage": row["generator_stage"],
                "template": row["template"],
                "side": row["side"],
                "lateral_factor_id": l["id"],
                "d_target": row["d_target"], "d_mid": row["d_mid"],
                "probe_source": row["mapping_source"], "s_probe": row["s_probe"],
                "d_probe": row["d_probe"], "root_type": row["root_type"],
                "root_index": row["root_index"],
                "probe_location_rule": row["probe_location_rule"],
                "probe_anchor_rule": row["probe_anchor_rule"],
                "analytic_branch_regime": row["analytic_branch_regime"],
                "lateral_constraint_semantics": lateral_semantics,
                "transition_factor_id": t["id"],
                "entry_scale": row["entry_scale"],
                "entry_length_m": number(row, "z1") - number(row, "z0"),
                "exit_scale": row["exit_scale"],
                "exit_length_m": number(row, "z4") - number(row, "z3"),
                "transition_semantics": transition_semantics,
                "current_coupling_rule": "ONLY_THIS_OBSERVED_FULL_TUPLE_IS_LEGAL",
                "currently_impossible_or_skipped": skipped,
                "returned_by_policy": row["returned_by_policy"],
                "discarded_side": row["discarded_side"],
                "validator_executed": row["validator_executed"],
                "hard_valid": row["hard_valid"],
                "path_digest": row["path_digest"],
            })
    return output


def geometry_row(event_id: str, event: dict, best: dict, recovery: dict) -> dict:
    left = best["side"] == "LEFT"
    corridor = G.make_corridor(event, left)
    cluster = corridor["cluster"]
    start = min(row["start"] for row in cluster)
    end = max(row["end"] for row in cluster)
    midpoint = 0.5 * (start + end)
    mid_sample = G.sample_at(event, corridor["visible"], midpoint, left)
    span = G.span_samples(corridor)
    bottleneck = min(span, key=lambda row: (row["width"], row["station"]))
    maneuver_start = max(0.0, number(best, "z0", 0.0))
    maneuver_end = number(best, "z4", 15.0)
    reference = G.local_reference_points(event, maneuver_end)
    local = [(station, row) for station, row in reference if station + EPS >= maneuver_start]
    kappas = [row["kappa"] for _, row in local] or [G.nearest_ref(event, event["ego_s"])["kappa"]]
    widths = [row["d_left"] + row["d_right"] for _, row in local]
    curvature_pattern = (
        "SIGN_CHANGE" if min(kappas) < -0.05 and max(kappas) > 0.05 else
        "POSITIVE" if max(kappas) > 0.05 else
        "NEGATIVE" if min(kappas) < -0.05 else "NEAR_ZERO"
    )
    cluster_ids = {row["id"] for row in cluster}
    later = [row for row in corridor["visible"]
             if row["id"] not in cluster_ids and row["start"] > end + EPS]
    if later:
        merge_distance = min(row["start"] for row in later) - end
        merge_status = "TO_NEXT_VISIBLE_OBSTACLE"
    else:
        merge_distance = max(0.0, 15.0 - end)
        merge_status = "CENSORED_AT_15M_LOOKAHEAD"
    entry_ref = G.nearest_ref(event, G.wrap(event, event["ego_s"] + maneuver_start))
    obstacle_ref = G.nearest_ref(event, G.wrap(event, event["ego_s"] + midpoint))
    exit_ref = G.nearest_ref(event, G.wrap(event, event["ego_s"] + maneuver_end))
    return {
        "development_event_id": event_id,
        **recovery,
        "required_side": best["side"],
        "ego_to_obstacle_start_m": start,
        "obstacle_longitudinal_span_m": end - start,
        "obstacle_raw_d_right_m": min(row["raw_d_right"] for row in cluster),
        "obstacle_raw_d_left_m": max(row["raw_d_left"] for row in cluster),
        "obstacle_inflated_d_right_m": min(row["d_right"] for row in cluster),
        "obstacle_inflated_d_left_m": max(row["d_left"] for row in cluster),
        "chosen_side_free_space_width_m": mid_sample["width"],
        "corridor_bottleneck_width_m": bottleneck["width"],
        "corridor_bottleneck_station_m": bottleneck["station"],
        "corridor_bottleneck_center_m": bottleneck["center"],
        "corridor_bottleneck_active_obstacles": json.dumps(bottleneck["active_obstacles"]),
        "available_merge_distance_after_obstacle_m": merge_distance,
        "available_merge_distance_status": merge_status,
        "ego_d_m": event["ego_d"], "ego_speed_mps": event["ego_speed"],
        "required_d_target_m": best["d_target"], "required_d_mid_m": best["d_mid"],
        "required_entry_scale": best["entry_scale"], "required_exit_scale": best["exit_scale"],
        "reference_curvature_entry_radpm": entry_ref["kappa"],
        "reference_curvature_obstacle_radpm": obstacle_ref["kappa"],
        "reference_curvature_exit_radpm": exit_ref["kappa"],
        "maximum_abs_reference_curvature_maneuver_radpm": max(abs(value) for value in kappas),
        "reference_curvature_pattern": curvature_pattern,
        "track_total_width_entry_m": entry_ref["d_left"] + entry_ref["d_right"],
        "track_total_width_obstacle_m": obstacle_ref["d_left"] + obstacle_ref["d_right"],
        "track_total_width_exit_m": exit_ref["d_left"] + exit_ref["d_right"],
        "track_width_range_over_maneuver_m": max(widths) - min(widths) if widths else 0.0,
    }


def maximum_set_coverage(covers: dict[tuple, set[str]], universe: set[str], max_budget: int,
                         kind: str) -> list[dict]:
    keys = sorted(covers)
    output = []
    for budget in range(1, min(max_budget, len(keys)) + 1):
        best = None
        for combo in itertools.combinations(keys, budget):
            covered = set().union(*(covers[key] for key in combo))
            score = len(covered)
            if best is None or score > best[0] or (score == best[0] and combo < best[1]):
                best = (score, combo, covered)
        assert best is not None
        score, combo, covered = best
        output.append({
            "record_type": kind, "budget": budget,
            "selected_options": json.dumps(combo),
            "covered_oracle_valid_episodes": score,
            "oracle_valid_episode_count": len(universe),
            "coverage_fraction": score / len(universe),
            "covered_event_ids": "|".join(sorted(covered)),
            "uncovered_event_ids": "|".join(sorted(universe - covered)),
            "selection_algorithm": "EXACT_MAXIMUM_SET_COVERAGE_TIE_LEXICOGRAPHIC",
            "coverage_interpretation": "DEVELOPMENT_ORACLE_UPPER_BOUND_NOT_ONLINE_SELECTOR",
        })
    return output


def main() -> None:
    HERE.mkdir(parents=True, exist_ok=True)
    PLOTS.mkdir(parents=True, exist_ok=True)
    WORK.mkdir(parents=True, exist_ok=True)
    if not HARNESS.is_file() or sha256(HARNESS) != EXPECTED_HARNESS_SHA256:
        raise RuntimeError("frozen pilot-v2 oracle harness missing or hash mismatch")

    production = read_csv(RAW / "production_candidates.csv")
    valid_all = read_csv(RAW / "oracle_valid_candidates.csv")
    strict_valid = [row for row in valid_all if row["gate"] == "STRICT"]
    summary = read_csv(CORPUS / "development_oracle_summary.csv")
    taxonomy = read_csv(CORPUS / "mapping_failure_taxonomy.csv")
    template_taxonomy = read_csv(TEMPLATE_DIAG / "transition_failure_taxonomy.csv")
    production_by_event: dict[str, list[dict]] = defaultdict(list)
    valid_by_event: dict[str, list[dict]] = defaultdict(list)
    for row in production:
        production_by_event[row["event_id"]].append(row)
    for row in strict_valid:
        valid_by_event[row["event_id"]].append(row)
    event_ids = sorted(production_by_event)
    if event_ids != [f"DVE{index:03d}" for index in range(1, 41)]:
        raise RuntimeError("expected exact DEVELOPMENT DVE001..DVE040 corpus")
    oracle_valid_events = {
        row["development_event_id"] for row in summary
        if row["oracle_classification"] == "ORACLE_VALID_P3_EXISTS"
    }
    mechanism = {row["development_event_id"]: row["mechanism_class"] for row in taxonomy}
    template_primary = {
        row["development_event_id"]: row["primary_option_failure_class"]
        for row in template_taxonomy
    }

    coupling_rows = current_coupling_rows(production_by_event)
    write_csv(HERE / "current_factor_coupling.csv", coupling_rows)

    candidate_space = []
    ablations = []
    duplicates = []
    results_by_event_variant: dict[tuple[str, str], list[dict]] = {}
    configs_by_event_variant: dict[tuple[str, str], list[dict]] = {}
    counterfactual_deltas: dict[float, set[str]] = defaultdict(set)

    for event_id in event_ids:
        event_path = RAW / "inputs" / f"{event_id}.event"
        event_production = production_by_event[event_id]
        event_valid = valid_by_event[event_id]
        variants, _, _, counterfactuals, production_keys = build_variants(
            event_id, event_production, event_valid)
        for zero_key, valid_row in counterfactuals.items():
            delta = rounded(number(valid_row, "d_mid") - zero_key[2])
            counterfactual_deltas[delta].add(event_id)

        empty_runtime = empty_runtime_and_parity(event_id, event_path, event_production)
        for variant in VARIANTS:
            configurations = [variants[variant][key] for key in sorted(variants[variant])]
            rows, wall_runtime, _ = run_harness(event_id, event_path, variant, configurations)
            if len(rows) != len(configurations):
                raise RuntimeError(f"{event_id}/{variant}: request/result count mismatch")
            if variant == "BASELINE_PRODUCTION":
                got = Counter(row["path_digest"] for row in rows)
                expected = Counter(row["path_digest"] for row in event_production)
                if got != expected:
                    raise RuntimeError(f"{event_id}: direct candidate reconstruction parity failed")
            digest_counts = Counter(row["path_digest"] for row in rows if row["path_digest"])
            hard_valid_digests = {
                row["path_digest"] for row in rows if row["hard_valid"] == "1" and row["path_digest"]
            }
            for config, row in zip(configurations, rows):
                candidate_space.append({
                    "development_event_id": event_id, "variant": variant,
                    "request_index": row["request_index"],
                    "configuration_key": key_text(config["key"]),
                    "lateral_factor_id": config["lateral_factor_id"],
                    "lateral_factor_origin": config["lateral_factor_origin"],
                    "lateral_factor_source": config["lateral_factor_source"],
                    "transition_factor_id": config["transition_factor_id"],
                    "transition_factor_origin": config["transition_factor_origin"],
                    "transition_factor_source": config["transition_factor_source"],
                    "combination_origins": "|".join(sorted(config["combination_origins"])),
                    "is_exact_production_configuration": config["key"] in production_keys,
                    "side": row["side"], "d_target": row["d_target"], "d_mid": row["d_mid"],
                    "entry_scale": row["entry_scale"], "exit_scale": row["exit_scale"],
                    "z0": row["z0"], "z1": row["z1"], "z2": row["z2"],
                    "z3": row["z3"], "z4": row["z4"],
                    "constructed": int(row["point_count"]) > 0,
                    "construction_guard_reject": row["validator_executed"] != "1",
                    "validator_executed": row["validator_executed"],
                    "hard_valid": row["hard_valid"],
                    "first_failure_reason": row["first_failure_reason"],
                    "path_digest": row["path_digest"],
                    "path_digest_multiplicity_within_variant": digest_counts[row["path_digest"]]
                    if row["path_digest"] else 0,
                    "duplicate_path_within_variant": bool(
                        row["path_digest"] and digest_counts[row["path_digest"]] > 1),
                    "minimum_normalized_safety_slack": row["minimum_normalized_safety_slack"],
                    "footprint_track_margin_m": row["footprint_track_margin_m"],
                    "obstacle_margin_m": row["obstacle_margin_m"],
                    "lateral_slope_margin": row["lateral_slope_margin"],
                    "signed_curvature_margin_radpm": row["signed_curvature_margin_radpm"],
                    "curvature_rate_margin_radpm2": row["curvature_rate_margin_radpm2"],
                })
            constructed = [row for row in rows if int(row["point_count"]) > 0]
            validators = [row for row in rows if row["validator_executed"] == "1"]
            hard_valid = [row for row in rows if row["hard_valid"] == "1"]
            duplicate_count = len(constructed) - len({row["path_digest"] for row in constructed})
            ablations.append({
                "development_event_id": event_id, "variant": variant,
                "oracle_valid_in_full_development_oracle": event_id in oracle_valid_events,
                "mechanism_class": mechanism.get(event_id, "NO_VALID_P3_IN_ORACLE_DOMAIN"),
                "template_primary_class": template_primary.get(event_id, ""),
                "unique_combination_count": len(configurations),
                "constructed_path_count": len(constructed),
                "construction_guard_reject_count": len(configurations) - len(validators),
                "validator_execution_count": len(validators),
                "hard_valid_candidate_count": len(hard_valid),
                "unique_hard_valid_path_count": len(hard_valid_digests),
                "recovered": bool(hard_valid),
                "additional_recovery_beyond_production": bool(hard_valid),
                "duplicate_path_count": duplicate_count,
                "duplicate_path_rate": duplicate_count / len(constructed) if constructed else 0.0,
                "hard_valid_rate_per_validator": len(hard_valid) / len(validators) if validators else 0.0,
                "harness_wall_runtime_s": wall_runtime,
                "empty_baseline_harness_runtime_s": empty_runtime,
                "approx_incremental_wall_runtime_s": max(0.0, wall_runtime - empty_runtime),
                "runtime_interpretation": "subprocess wall diagnostic; includes output and residual noise",
            })
            duplicates.append({
                "development_event_id": event_id, "variant": variant,
                "request_count": len(configurations), "constructed_path_count": len(constructed),
                "unique_path_digest_count": len({row["path_digest"] for row in constructed}),
                "duplicate_path_count": duplicate_count,
                "duplicate_path_rate": duplicate_count / len(constructed) if constructed else 0.0,
                "maximum_path_digest_multiplicity": max(digest_counts.values(), default=0),
                "duplicate_hard_valid_count": len(hard_valid) - len(hard_valid_digests),
            })
            results_by_event_variant[(event_id, variant)] = rows
            configs_by_event_variant[(event_id, variant)] = configurations
        print("FACTORIZED", event_id, {variant: sum(
            row["hard_valid"] == "1" for row in results_by_event_variant[(event_id, variant)])
            for variant in VARIANTS}, flush=True)

    write_csv(HERE / "factorized_candidate_space.csv", candidate_space)
    write_csv(HERE / "factorization_ablation.csv", ablations)
    write_csv(HERE / "duplicate_analysis.csv", duplicates)

    ablation_index = {(row["development_event_id"], row["variant"]): row for row in ablations}
    recovery_rows = []
    for event_id in event_ids:
        row = {
            "development_event_id": event_id,
            "oracle_valid_in_full_development_oracle": event_id in oracle_valid_events,
            "mechanism_class": mechanism.get(event_id, "NO_VALID_P3_IN_ORACLE_DOMAIN"),
            "template_primary_class": template_primary.get(event_id, ""),
        }
        for variant in VARIANTS:
            result = ablation_index[(event_id, variant)]
            row[f"{variant}__recovered"] = result["recovered"]
            row[f"{variant}__candidates"] = result["unique_combination_count"]
            row[f"{variant}__validators"] = result["validator_execution_count"]
            row[f"{variant}__hard_valid"] = result["hard_valid_candidate_count"]
        recovery_rows.append(row)
    write_csv(HERE / "recovery_by_variant.csv", recovery_rows)

    budget_rows = []
    for variant in VARIANTS:
        rows = [row for row in ablations if row["variant"] == variant]
        candidates = [int(row["unique_combination_count"]) for row in rows]
        validators = [int(row["validator_execution_count"]) for row in rows]
        runtimes = [float(row["approx_incremental_wall_runtime_s"]) for row in rows]
        construction_rejects = [int(row["construction_guard_reject_count"]) for row in rows]
        constructed = [int(row["constructed_path_count"]) for row in rows]
        hard_valid = [int(row["hard_valid_candidate_count"]) for row in rows]
        duplicate_paths = [int(row["duplicate_path_count"]) for row in rows]
        recovered = {row["development_event_id"] for row in rows if truth(row["recovered"])}
        recovered_oracle = recovered & oracle_valid_events
        budget_rows.append({
            "record_type": "VARIANT_AGGREGATE", "variant": variant,
            "candidate_count_total": sum(candidates),
            "candidates_per_evaluation_p50": percentile(candidates, 50),
            "candidates_per_evaluation_p95": percentile(candidates, 95),
            "candidates_per_evaluation_p99": percentile(candidates, 99),
            "candidates_per_evaluation_max": max(candidates),
            "validator_calls_total": sum(validators),
            "validator_calls_per_evaluation_p50": percentile(validators, 50),
            "validator_calls_per_evaluation_p95": percentile(validators, 95),
            "validator_calls_per_evaluation_p99": percentile(validators, 99),
            "validator_calls_per_evaluation_max": max(validators),
            "constructed_path_total": sum(constructed),
            "construction_guard_reject_total": sum(construction_rejects),
            "hard_valid_candidate_total": sum(hard_valid),
            "hard_valid_rate_per_validator_global": sum(hard_valid) / max(1, sum(validators)),
            "recovered_oracle_valid_episodes": len(recovered_oracle),
            "oracle_valid_episode_count": len(oracle_valid_events),
            "recovery_fraction_of_oracle_valid": len(recovered_oracle) / len(oracle_valid_events),
            "additional_recovery_beyond_production": len(recovered_oracle),
            "duplicate_path_count_total": sum(duplicate_paths),
            "duplicate_path_rate_global": sum(duplicate_paths) / max(1, sum(constructed)),
            "approx_incremental_runtime_total_s": sum(runtimes),
            "approx_incremental_runtime_p50_s": percentile(runtimes, 50),
            "approx_incremental_runtime_p95_s": percentile(runtimes, 95),
            "approx_incremental_runtime_p99_s": percentile(runtimes, 99),
            "approx_incremental_runtime_max_s": max(runtimes),
            "runtime_interpretation": "subprocess wall minus empty baseline; diagnostic only",
        })

    # Exact transition-pair oracle upper-bound coverage using hard-valid full-factorial rows.
    transition_covers: dict[tuple, set[str]] = defaultdict(set)
    for event_id in oracle_valid_events:
        for row in results_by_event_variant[(event_id, "FULL_LxT_DEV_UPPER_BOUND")]:
            if row["hard_valid"] == "1":
                transition_covers[(rounded(number(row, "entry_scale")),
                                   rounded(number(row, "exit_scale")))].add(event_id)
    budget_rows.extend(maximum_set_coverage(
        transition_covers, oracle_valid_events, 4, "TRANSITION_PAIR_ORACLE_UPPER_BOUND"))
    a_recoverable = set().union(*counterfactual_deltas.values()) if counterfactual_deltas else set()
    budget_rows.extend(maximum_set_coverage(
        counterfactual_deltas, a_recoverable, 4, "D_MID_DELTA_ORACLE_COUNTERFACTUAL_COVERAGE"))
    write_csv(HERE / "candidate_budget_analysis.csv", budget_rows)

    geometry_rows = []
    for event_id in sorted(oracle_valid_events):
        full_rows = [row for row in results_by_event_variant[
            (event_id, "FULL_LxT_DEV_UPPER_BOUND")] if row["hard_valid"] == "1"]
        best = min(full_rows, key=oracle_rank)
        recovery = {
            "mechanism_class": mechanism[event_id],
            "template_primary_class": template_primary.get(event_id, ""),
            "a_existing_mid_pool_recovered": ablation_index[
                (event_id, "A_EXISTING_D_MID_POOL")]["recovered"],
            "a_dev_counterfactual_recovered": ablation_index[
                (event_id, "A_UNCOUPLE_D_MID_DEV_COUNTERFACTUAL")]["recovered"],
            "b_cross_transition_recovered": ablation_index[
                (event_id, "B_CROSS_PAIR_TRANSITIONS")]["recovered"],
            "ab_dev_counterfactual_recovered": ablation_index[
                (event_id, "AB_DEV_COUNTERFACTUAL")]["recovered"],
            "full_hard_valid_configuration_count": len(full_rows),
            "full_unique_hard_valid_lateral_count": len({lateral_key(row) for row in full_rows}),
            "full_unique_hard_valid_transition_count": len({transition_key(row) for row in full_rows}),
        }
        event = G.parse_event(RAW / "inputs" / f"{event_id}.event")
        geometry_rows.append(geometry_row(event_id, event, best, recovery))
    write_csv(HERE / "geometry_conditioning_features.csv", geometry_rows)

    # Plots.
    aggregate = {row["variant"]: row for row in budget_rows
                 if row["record_type"] == "VARIANT_AGGREGATE"}
    short = ["BASE", "A-current", "A-dev", "B", "AB-current", "AB-dev", "FULL"]
    recovered_values = [int(aggregate[v]["recovered_oracle_valid_episodes"]) for v in VARIANTS]
    figure, axis = plt.subplots(figsize=(9, 5))
    axis.bar(short, recovered_values, color="#3f7da8")
    axis.axhline(23, color="#555", linestyle="--", linewidth=1, label="oracle-valid 23")
    axis.set(ylabel="recovered DEVELOPMENT episodes", title="Factorization ablation recovery")
    axis.grid(axis="y", alpha=.2); axis.legend(); figure.tight_layout()
    figure.savefig(PLOTS / "recovery_by_variant.png", dpi=170); plt.close(figure)

    by_variant_counts = [[int(row["unique_combination_count"]) for row in ablations
                          if row["variant"] == variant] for variant in VARIANTS]
    figure, axis = plt.subplots(figsize=(10, 5.3))
    axis.boxplot(by_variant_counts, labels=short, showfliers=True)
    axis.set_yscale("symlog", linthresh=1)
    axis.set(ylabel="unique requested configurations per evaluation",
             title="Candidate-space growth across 40 DEVELOPMENT evaluations")
    axis.grid(axis="y", alpha=.2); figure.tight_layout()
    figure.savefig(PLOTS / "candidate_explosion.png", dpi=170); plt.close(figure)

    duplicate_rates = [float(aggregate[v]["duplicate_path_rate_global"]) for v in VARIANTS]
    figure, axis = plt.subplots(figsize=(9, 5))
    axis.bar(short, duplicate_rates, color="#ae6746")
    axis.set(ylabel="duplicate constructed-path rate", title="Path duplication after factorization")
    axis.grid(axis="y", alpha=.2); figure.tight_layout()
    figure.savefig(PLOTS / "duplicate_rate_by_variant.png", dpi=170); plt.close(figure)

    matrix = np.asarray([[int(truth(ablation_index[(event_id, variant)]["recovered"]))
                          for variant in VARIANTS] for event_id in event_ids])
    figure, axis = plt.subplots(figsize=(10, 9))
    axis.imshow(matrix, aspect="auto", cmap="Greens", vmin=0, vmax=1)
    axis.set_xticks(range(len(short)), short, rotation=25, ha="right")
    axis.set_yticks(range(len(event_ids)), event_ids, fontsize=7)
    axis.set_title("Episode recovery matrix (green = at least one hard-valid P3)")
    figure.tight_layout(); figure.savefig(PLOTS / "recovery_matrix.png", dpi=170); plt.close(figure)

    figure, axis = plt.subplots(figsize=(8, 5.3))
    colors = {
        "A_ONLY": "#b65c3a", "B_ONLY": "#3978a8",
        "A_AND_B": "#7b4aa3", "FULL_ONLY": "#777777",
    }
    labeled_routes = set()
    for row in geometry_rows:
        a_recovered = truth(row["a_dev_counterfactual_recovered"])
        b_recovered = truth(row["b_cross_transition_recovered"])
        route = "A_AND_B" if a_recovered and b_recovered else (
            "A_ONLY" if a_recovered else "B_ONLY" if b_recovered else "FULL_ONLY")
        axis.scatter(float(row["ego_to_obstacle_start_m"]),
                     float(row["maximum_abs_reference_curvature_maneuver_radpm"]),
                     color=colors[route], marker="*" if route == "A_AND_B" else "o",
                     s=100 if route == "A_AND_B" else 48,
                     zorder=4 if route == "A_AND_B" else 2,
                     label=route if route not in labeled_routes else None)
        labeled_routes.add(route)
        if row["development_event_id"] in {"DVE029", "DVE039"}:
            axis.annotate(row["development_event_id"],
                          (float(row["ego_to_obstacle_start_m"]),
                           float(row["maximum_abs_reference_curvature_maneuver_radpm"])),
                          xytext=(4, 4), textcoords="offset points", fontsize=7)
    axis.set(xlabel="ego to obstacle start [m]", ylabel="max |reference curvature| [rad/m]",
             title="Pre-planning geometry and factorization recovery route")
    axis.grid(alpha=.2); axis.legend(title="recovery route"); figure.tight_layout()
    figure.savefig(PLOTS / "geometry_recovery_route.png", dpi=170); plt.close(figure)

    transition_budget = [row for row in budget_rows
                         if row["record_type"] == "TRANSITION_PAIR_ORACLE_UPPER_BOUND"]
    figure, axis = plt.subplots(figsize=(7.5, 4.8))
    axis.plot([int(row["budget"]) for row in transition_budget],
              [float(row["coverage_fraction"]) for row in transition_budget], marker="o")
    axis.set(xticks=range(1, 5), ylim=(0, 1.05), xlabel="transition-pair budget",
             ylabel="oracle-valid episode coverage",
             title="DEVELOPMENT oracle upper-bound transition coverage")
    axis.grid(alpha=.2); figure.tight_layout()
    figure.savefig(PLOTS / "bounded_transition_coverage.png", dpi=170); plt.close(figure)

    manifest = {
        "study": "P3_FACTORIZED_CANDIDATE_GENERATION_DESIGN_V1",
        "repository_head": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=REPO, text=True).strip(),
        "dataset_scope": "DVE001..DVE040 DEVELOPMENT only",
        "validation_or_holdout_rows_read": 0,
        "baseline_direct_reconstruction_parity_events": len(event_ids),
        "baseline_direct_reconstruction_parity_status": "PASS",
        "production_planner_modified": False,
        "new_spline_family": False,
        "clearance_gate": "STRICT_ONLY (all 23 oracle-valid episodes have strict hard-valid rows)",
        "harness": str(HARNESS), "harness_sha256": sha256(HARNESS),
        "harness_source_sha256": sha256(HARNESS_SOURCE),
        "inputs": {
            "production_candidates.csv": sha256(RAW / "production_candidates.csv"),
            "oracle_valid_candidates.csv": sha256(RAW / "oracle_valid_candidates.csv"),
            "development_oracle_summary.csv": sha256(CORPUS / "development_oracle_summary.csv"),
            "mapping_failure_taxonomy.csv": sha256(CORPUS / "mapping_failure_taxonomy.csv"),
            "transition_failure_taxonomy.csv": sha256(
                TEMPLATE_DIAG / "transition_failure_taxonomy.csv"),
        },
        "variant_definitions": {
            "BASELINE_PRODUCTION": "exact production full tuples",
            "A_EXISTING_D_MID_POOL": "zero target x same-side production d_mid; current transition",
            "A_UNCOUPLE_D_MID_DEV_COUNTERFACTUAL":
                "nearest already-open strict hard-valid d_mid at same zero side/target/transition",
            "B_CROSS_PAIR_TRANSITIONS": "unique production lateral tuples x event production transitions",
            "AB_EXISTING_FACTOR_BASIS": "A-existing lateral basis x event production transitions",
            "AB_DEV_COUNTERFACTUAL": "A-development lateral basis x event production transitions",
            "FULL_LxT_DEV_UPPER_BOUND":
                "production plus strict oracle-valid lateral tuples x event production transitions",
        },
    }
    (HERE / "execution_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    summary_payload = {
        variant: {
            "candidates": int(aggregate[variant]["candidate_count_total"]),
            "validators": int(aggregate[variant]["validator_calls_total"]),
            "hard_valid_candidates": int(aggregate[variant]["hard_valid_candidate_total"]),
            "recovered_oracle_valid_episodes": int(
                aggregate[variant]["recovered_oracle_valid_episodes"]),
            "duplicate_path_rate": float(aggregate[variant]["duplicate_path_rate_global"]),
        } for variant in VARIANTS
    }
    (HERE / "analysis_summary.json").write_text(
        json.dumps(summary_payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary_payload, indent=2))


if __name__ == "__main__":
    main()
