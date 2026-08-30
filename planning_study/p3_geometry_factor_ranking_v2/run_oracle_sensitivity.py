#!/usr/bin/env python3
"""Seen-only exact-witness sensitivity audit for Reference Oracle v2 subsets.

Every variant is a declared subset of the frozen Oracle-v2 request domain.  Baseline
negatives therefore remain negative.  A baseline positive is retained only when an
already exact-validated hard/usable witness belongs to the variant domain; otherwise the
variant label is explicitly inconclusive rather than silently changed to negative.
"""

from __future__ import annotations

import csv
import importlib.util
import json
import math
from collections import Counter, defaultdict
from pathlib import Path


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
REFERENCE = REPO / "planning_study/p3_reference_oracle_v2"
RUNNER = REFERENCE / "run_reference_oracle_v2.py"
METHOD_IMPL = REPO / "planning_study/p3_geometry_conditioned_method_v1/analyze_method.py"
RESULTS = HERE / "factor_ranking_results.csv"
SELECTED = HERE / "selected_candidates_audit.csv"
EPS = 1.0e-9

VARIANTS = {
    "BASELINE_ORACLE_V2": {
        "grid": 0.01, "mid_bound": 1.5, "transition": "ALL",
        "factor_anchors": True, "factor_union": True,
    },
    "COARSE_GRID_0P02": {
        "grid": 0.02, "mid_bound": 1.5, "transition": "ALL",
        "factor_anchors": True, "factor_union": True,
    },
    "NARROW_D_MID_1P25": {
        "grid": 0.01, "mid_bound": 1.25, "transition": "ALL",
        "factor_anchors": True, "factor_union": True,
    },
    "MINIMUM_TRANSITION_PAIR_ONLY": {
        "grid": 0.01, "mid_bound": 1.5, "transition": "MIN_PAIR",
        "factor_anchors": True, "factor_union": True,
    },
    "NO_FROZEN_FACTOR_ANCHORS": {
        "grid": 0.01, "mid_bound": 1.5, "transition": "ALL",
        "factor_anchors": False, "factor_union": False,
    },
}


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


R = load_module("sensitivity_reference", RUNNER)
M = load_module("sensitivity_method", METHOD_IMPL)


def read_csv(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, rows: list[dict]) -> None:
    fields = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def canonical_config(row: dict, default_gate: str = "STRICT") -> tuple:
    return R.config_key(
        row.get("gate", default_gate), row["side"], row["d_target"], row["d_mid"],
        row["entry_scale"], row["exit_scale"])


def witness_catalog() -> dict[tuple[str, str, str], list[dict]]:
    witnesses = defaultdict(list)
    for path in sorted((Path("/tmp/p3_reference_oracle_v2/event_results")).glob("*.json")):
        result = json.loads(path.read_text(encoding="utf-8"))
        role, event = result["dataset_role"], result["event_id"]
        for metric, key in [("HARD", "best_hard"), ("USABLE", "best_usable")]:
            if result.get(key):
                witnesses[(role, event, metric)].append({**result[key], "source": "ORACLE_V2_BEST"})
        for selector, selector_result in result.get("selector_results", {}).items():
            for metric, key in [("HARD", "best_hard"), ("USABLE", "best_usable")]:
                if selector_result.get(key):
                    witnesses[(role, event, metric)].append({
                        **selector_result[key], "source": selector + "_BEST"})

    event_role = {}
    for row in read_csv(RESULTS):
        event_role[row["event_id"]] = row["dataset_role"]
    for row in read_csv(SELECTED):
        role, event = row["dataset_role"], row["event_id"]
        base = {**row, "gate": "STRICT", "source": row["hypothesis"] + "_SELECTED"}
        if row["hard_valid"] == "1":
            witnesses[(role, event, "HARD")].append(base)
        if str(row["usable_valid"]).lower() == "true":
            witnesses[(role, event, "USABLE")].append(base)
    return witnesses


def variant_domain(item: dict, variant: dict) -> dict:
    contexts = R.query_contexts(item["event_path"])
    pool, pool_by_side, *_ = R.special_maps(item, M, {})
    production = item["production"]
    pairs = sorted({(float(row["entry_scale"]), float(row["exit_scale"]))
                    for row in production})
    if variant["transition"] == "MIN_PAIR":
        pairs = [min(pairs, key=lambda pair: (pair[0] + pair[1], pair[0], pair[1]))]
    context_sets = {}
    mid_bound = float(variant["mid_bound"])
    for context in contexts:
        gate, side = context["gate"], context["side"]
        lower, upper = sorted((float(context["minimum_target"]),
                               float(context["maximum_target"])))
        side_production = [row for row in production if row["side"] == side]
        targets = R.base_grid(lower, upper, variant["grid"])
        targets = R.add_anchors(
            targets, [float(row["d_target"]) for row in side_production], lower, upper, True)
        middles = R.base_grid(-mid_bound, mid_bound, variant["grid"])
        middles = R.add_anchors(
            middles, [float(row["d_mid"]) for row in side_production],
            -mid_bound, mid_bound, True)
        if variant["factor_anchors"]:
            targets = R.add_anchors(
                targets, [float(row["d_target"]) for row in pool_by_side[side]],
                lower, upper, False)
            middles = R.add_anchors(
                middles, [float(row["d_mid"]) for row in pool_by_side[side]],
                -mid_bound, mid_bound, False)
        context_sets[(gate, side)] = {
            "targets": {float(value).hex() for value in targets},
            "middles": {float(value).hex() for value in middles},
            "pairs": {(float(entry).hex(), float(exit_scale).hex())
                      for entry, exit_scale in pairs},
        }

    allowed_pairs = set(pairs)
    exact_union = set()
    for row in production:
        pair = (float(row["entry_scale"]), float(row["exit_scale"]))
        if pair in allowed_pairs and abs(float(row["d_mid"])) <= mid_bound + EPS:
            exact_union.add(R.config_key(
                "STRICT", row["side"], row["d_target"], row["d_mid"],
                row["entry_scale"], row["exit_scale"]))
    if variant["factor_union"]:
        for row in pool:
            pair = (float(row["entry_scale"]), float(row["exit_scale"]))
            if pair in allowed_pairs and abs(float(row["d_mid"])) <= mid_bound + EPS:
                exact_union.add(R.config_key(
                    "STRICT", row["side"], row["d_target"], row["d_mid"],
                    row["entry_scale"], row["exit_scale"]))
    return {"contexts": context_sets, "exact_union": exact_union}


def domain_contains(domain: dict, key: tuple) -> bool:
    if key in domain["exact_union"]:
        return True
    gate, side, target, middle, entry, exit_scale = key
    context = domain["contexts"].get((gate, side))
    return bool(context and target in context["targets"] and middle in context["middles"]
                and (entry, exit_scale) in context["pairs"])


def main() -> None:
    witnesses = witness_catalog()
    items = R.input_catalog()
    event_rows = []
    for item in items:
        cache = json.loads((Path("/tmp/p3_reference_oracle_v2/event_results") /
                            f"{item['event_id']}.json").read_text(encoding="utf-8"))
        baseline = {"HARD": bool(cache.get("best_hard")),
                    "USABLE": bool(cache.get("best_usable"))}
        for variant_name, variant in VARIANTS.items():
            domain = None if variant_name == "BASELINE_ORACLE_V2" else variant_domain(item, variant)
            for metric in ["HARD", "USABLE"]:
                known = witnesses.get((item["dataset_role"], item["event_id"], metric), [])
                retained = baseline[metric] if domain is None else any(
                    domain_contains(domain, canonical_config(row)) for row in known)
                source = "|".join(sorted({row["source"] for row in known
                                          if domain is None or domain_contains(
                                              domain, canonical_config(row))}))
                if not baseline[metric]:
                    label = "NEGATIVE_RETAINED_BY_SUBSET_MONOTONICITY"
                elif retained:
                    label = "POSITIVE_RETAINED_BY_EXACT_VALID_WITNESS"
                else:
                    label = "INCONCLUSIVE_WITNESS_NOT_RETAINED"
                event_rows.append({
                    "dataset_role": item["dataset_role"], "event_id": item["event_id"],
                    "variant": variant_name, "metric": metric,
                    "baseline_positive": baseline[metric], "retained_positive": retained,
                    "sensitivity_label": label, "retained_witness_sources": source,
                    "declared_subset_of_oracle_v2": variant_name != "BASELINE_ORACLE_V2",
                    "continuous_completeness_claim": False,
                })
        print("SENSITIVITY", item["event_id"], flush=True)

    selected = {(row["dataset_role"], row["event_id"]): row
                for row in read_csv(RESULTS)
                if row["hypothesis"] == "R3_LEXICOGRAPHIC_COVERAGE_RESERVE"
                and row["budget_k"] == "12"}
    summary = []
    roles = ["PILOT_SEEN_DEVELOPMENT_DATA", "DEVELOPMENT",
             "VALIDATION_SEEN_AFTER_V1", "COMBINED_SEEN"]
    for role in roles:
        allowed = ({"DEVELOPMENT", "VALIDATION_SEEN_AFTER_V1"}
                   if role == "COMBINED_SEEN" else {role})
        for variant_name in VARIANTS:
            for metric in ["HARD", "USABLE"]:
                rows = [row for row in event_rows if row["dataset_role"] in allowed
                        and row["variant"] == variant_name and row["metric"] == metric]
                baseline_positive = [row for row in rows if row["baseline_positive"]]
                retained = [row for row in rows if row["retained_positive"]]
                method_recovered = []
                for row in retained:
                    method = selected.get((row["dataset_role"], row["event_id"]))
                    if method and str(method[f"{metric.lower()}_recovered"]).lower() == "true":
                        method_recovered.append(row)
                summary.append({
                    "dataset_role": role, "variant": variant_name, "metric": metric,
                    "baseline_oracle_v2_positive_count": len(baseline_positive),
                    "exact_witness_retained_positive_count": len(retained),
                    "inconclusive_positive_count": len(baseline_positive) - len(retained),
                    "selected_r3_k12_recovered_among_retained": len(method_recovered),
                    "selected_r3_k12_recovery_rate_among_retained":
                        len(method_recovered) / len(retained) if retained else math.nan,
                    "interpretation": (
                        "EXACT_WITNESS_LOWER_BOUND_FOR_DECLARED_SUBSET; NOT_FULL_VARIANT_SEARCH"),
                    "material_conclusion_change": (
                        "INCONCLUSIVE_IF_WITNESS_DROPS" if len(retained) < len(baseline_positive)
                        else "NO_ON_RETAINED_EXACT_WITNESSES"),
                })
    write_csv(HERE / "oracle_v2_sensitivity_events.csv", event_rows)
    write_csv(HERE / "oracle_v2_sensitivity.csv", summary)


if __name__ == "__main__":
    main()
