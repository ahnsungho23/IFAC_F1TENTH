#!/usr/bin/env python3
"""Diagnose template/transition mechanisms in already-open DEVELOPMENT oracle results."""

from __future__ import annotations

import csv
import importlib.util
import itertools
import json
import math
import os
from collections import Counter, defaultdict
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/p3_template_transition_mpl")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
CORPUS = REPO / "planning_study/p3_mapping_research_corpus_v1"
RAW = CORPUS / "raw_oracle"
GEOMETRY_SOURCE = REPO / "planning_study/p3_mapping_miss_diagnosis_v1/analyze_mapping_miss.py"
PLOTS = HERE / "plots"
EPS = 1.0e-9


def load_geometry():
    spec = importlib.util.spec_from_file_location("frozen_geometry", GEOMETRY_SOURCE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


G = load_geometry()


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
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def f(row: dict, key: str, default: float = math.nan) -> float:
    value = row.get(key, "")
    return float(value) if value not in ("", None) else default


def close(a: dict, b: dict, key: str, tolerance: float = 1.0e-9) -> bool:
    return abs(f(a, key) - f(b, key)) <= tolerance


def scale_key(row: dict) -> tuple[float, float]:
    return round(f(row, "entry_scale"), 12), round(f(row, "exit_scale"), 12)


def transition_lengths(row: dict) -> tuple[float, float]:
    return f(row, "z1") - f(row, "z0"), f(row, "z4") - f(row, "z3")


def hard_margin_payload(row: dict) -> dict:
    return {
        "oracle_center_track_margin_m": row["center_track_margin_m"],
        "oracle_footprint_track_margin_m": row["footprint_track_margin_m"],
        "oracle_obstacle_margin_m": row["obstacle_margin_m"],
        "oracle_lateral_slope_margin": row["lateral_slope_margin"],
        "oracle_signed_curvature_margin_radpm": row["signed_curvature_margin_radpm"],
        "oracle_curvature_rate_margin_radpm2": row["curvature_rate_margin_radpm2"],
        "oracle_braking_deficit_m": row["braking_deficit_m"],
        "oracle_minimum_normalized_safety_slack": row["minimum_normalized_safety_slack"],
    }


def route_candidates(event_prod: list[dict], event_valid: list[dict]) -> tuple[list[tuple], list[tuple]]:
    exact_scale_change = []
    zero_mid_change = []
    strict = [row for row in event_valid if row["gate"] == "STRICT"] or event_valid
    for oracle in strict:
        for prod in event_prod:
            if oracle["side"] != prod["side"]:
                continue
            same_target = close(oracle, prod, "d_target")
            same_mid = close(oracle, prod, "d_mid")
            same_entry = close(oracle, prod, "entry_scale")
            same_exit = close(oracle, prod, "exit_scale")
            if same_target and same_mid and not (same_entry and same_exit):
                oracle_entry, oracle_exit = transition_lengths(oracle)
                prod_entry, prod_exit = transition_lengths(prod)
                exact_scale_change.append((
                    int(not same_entry) + int(not same_exit),
                    abs(oracle_entry - prod_entry) + abs(oracle_exit - prod_exit),
                    -f(oracle, "minimum_normalized_safety_slack"), oracle, prod,
                ))
            if (not prod.get("s_probe", "") and same_target and same_entry and same_exit
                    and abs(f(oracle, "d_mid") - f(prod, "d_mid")) > 1.0e-6):
                zero_mid_change.append((
                    abs(f(oracle, "d_mid") - f(prod, "d_mid")),
                    -f(oracle, "minimum_normalized_safety_slack"), oracle, prod,
                ))
    return exact_scale_change, zero_mid_change


def route_row(event_id: str, route: str, oracle: dict, prod: dict,
              event_prod: list[dict], primary: bool) -> dict:
    prod_entry_length, prod_exit_length = transition_lengths(prod)
    oracle_entry_length, oracle_exit_length = transition_lengths(oracle)
    providers = sorted({f"{row['generator_stage']}:{row['template']}"
                        for row in event_prod if scale_key(row) == scale_key(oracle)})
    entry_delta = oracle_entry_length - prod_entry_length
    exit_delta = oracle_exit_length - prod_exit_length
    categories = []
    recovery_class = ""
    if route == "EXISTING_SCALE_CROSS_COMBINATION":
        if abs(entry_delta) > 1.0e-9:
            categories.append("ENTRY_TRANSITION_TOO_SHORT" if entry_delta > 0 else "ENTRY_TRANSITION_TOO_LONG")
        if abs(exit_delta) > 1.0e-9:
            categories.append("EXIT_TRANSITION_TOO_SHORT" if exit_delta > 0 else "EXIT_TRANSITION_TOO_LONG")
        if abs(entry_delta) > 1.0e-9 and abs(exit_delta) > 1.0e-9:
            categories.append("ENTRY_EXIT_COMBINATION_MISS")
        if providers and f"{prod['generator_stage']}:{prod['template']}" not in providers:
            categories.append("TEMPLATE_FAMILY_SELECTION_MISS")
        if providers and all(row.get("s_probe", "") for row in event_prod
                             if scale_key(row) == scale_key(oracle)) and not prod.get("s_probe", ""):
            categories.append("INTERACTION_WITH_PROBE")
        recovery_class = "SELECTION_FAILURE_WITH_EXISTING_OPTION"
    else:
        categories.extend(["TEMPLATE_FAMILY_SELECTION_MISS", "INTERACTION_WITH_D_MID",
                           "OTHER_TEMPLATE_MECHANISM"])
        recovery_class = "COVERAGE_FAILURE_MISSING_OPTION"
    return {
        "development_event_id": event_id,
        "route": route,
        "is_minimum_change_route": primary,
        "precise_categories": "|".join(categories),
        "option_failure_class": recovery_class,
        "production_stage": prod["generator_stage"],
        "production_template": prod["template"],
        "production_side": prod["side"],
        "production_entry_scale": prod["entry_scale"],
        "production_entry_length_m": prod_entry_length,
        "production_exit_scale": prod["exit_scale"],
        "production_exit_length_m": prod_exit_length,
        "production_d_target": prod["d_target"],
        "production_d_mid": prod["d_mid"],
        "production_probe_source": prod["mapping_source"],
        "production_probe_location_rule": prod["probe_location_rule"],
        "production_probe_anchor_rule": prod["probe_anchor_rule"],
        "production_first_failure": prod["first_failure_reason"],
        "oracle_family": "DIRECT_P3_WITH_EXISTING_SCALE_PAIR",
        "existing_production_scale_pair_providers": "|".join(providers),
        "oracle_entry_scale": oracle["entry_scale"],
        "oracle_entry_length_m": oracle_entry_length,
        "oracle_exit_scale": oracle["exit_scale"],
        "oracle_exit_length_m": oracle_exit_length,
        "oracle_d_target": oracle["d_target"],
        "oracle_d_mid": oracle["d_mid"],
        "delta_entry_scale": f(oracle, "entry_scale") - f(prod, "entry_scale"),
        "delta_entry_length_m": entry_delta,
        "delta_exit_scale": f(oracle, "exit_scale") - f(prod, "exit_scale"),
        "delta_exit_length_m": exit_delta,
        "delta_d_target_m": f(oracle, "d_target") - f(prod, "d_target"),
        "delta_d_mid_m": f(oracle, "d_mid") - f(prod, "d_mid"),
        "oracle_scale_pair_present_in_event_production_options": bool(providers),
        "oracle_exact_full_tuple_present_in_production": False,
        **hard_margin_payload(oracle),
    }


def geometry_features(event_id: str, event: dict, oracle: dict, route: dict) -> dict:
    left = oracle["side"] == "LEFT"
    corridor = G.make_corridor(event, left)
    cluster = corridor["cluster"]
    cluster_start = min(row["start"] for row in cluster)
    cluster_end = max(row["end"] for row in cluster)
    midpoint = 0.5 * (cluster_start + cluster_end)
    mid_sample = G.sample_at(event, corridor["visible"], midpoint, left)
    span = G.span_samples(corridor)
    bottleneck = min(span, key=lambda row: (row["width"], row["station"]))
    maneuver_start = max(0.0, f(oracle, "z0"))
    maneuver_end = f(oracle, "z4")
    reference = G.local_reference_points(event, maneuver_end)
    kappas = [row["kappa"] for station, row in reference if station + EPS >= maneuver_start]
    if not kappas:
        kappas = [G.nearest_ref(event, event["ego_s"])["kappa"]]
    curvature_sign = ("SIGN_CHANGE" if min(kappas) < -0.05 and max(kappas) > 0.05 else
                      "POSITIVE" if max(kappas) > 0.05 else
                      "NEGATIVE" if min(kappas) < -0.05 else "NEAR_ZERO")
    widths = [row["d_left"] + row["d_right"] for station, row in reference
              if station + EPS >= maneuver_start]
    cluster_ids = {row["id"] for row in cluster}
    later = [row for row in corridor["visible"]
             if row["id"] not in cluster_ids and row["start"] > cluster_end + EPS]
    if later:
        available_merge = min(row["start"] for row in later) - cluster_end
        merge_status = "TO_NEXT_VISIBLE_OBSTACLE"
    else:
        available_merge = max(0.0, 15.0 - cluster_end)
        merge_status = "CENSORED_AT_15M_LOOKAHEAD"
    raw_right = min(row["raw_d_right"] for row in cluster)
    raw_left = max(row["raw_d_left"] for row in cluster)
    inflated_right = min(row["d_right"] for row in cluster)
    inflated_left = max(row["d_left"] for row in cluster)
    return {
        "development_event_id": event_id,
        "minimum_change_route": route["route"],
        "option_failure_class": route["option_failure_class"],
        "precise_categories": route["precise_categories"],
        "chosen_side": oracle["side"],
        "ego_to_obstacle_start_m": cluster_start,
        "obstacle_cluster_span_m": cluster_end - cluster_start,
        "obstacle_raw_d_right_m": raw_right,
        "obstacle_raw_d_left_m": raw_left,
        "obstacle_inflated_d_right_m": inflated_right,
        "obstacle_inflated_d_left_m": inflated_left,
        "chosen_side_free_width_at_obstacle_mid_m": mid_sample["width"],
        "corridor_bottleneck_width_m": bottleneck["width"],
        "corridor_bottleneck_station_m": bottleneck["station"],
        "corridor_bottleneck_center_m": bottleneck["center"],
        "corridor_bottleneck_active_obstacles": json.dumps(bottleneck["active_obstacles"]),
        "ego_d_m": event["ego_d"],
        "ego_speed_mps": event["ego_speed"],
        "d_target_m": oracle["d_target"],
        "reference_curvature_entry_radpm": G.nearest_ref(event, G.wrap(event, event["ego_s"] + maneuver_start))["kappa"],
        "reference_curvature_obstacle_radpm": G.nearest_ref(event, G.wrap(event, event["ego_s"] + midpoint))["kappa"],
        "reference_curvature_exit_radpm": G.nearest_ref(event, G.wrap(event, event["ego_s"] + maneuver_end))["kappa"],
        "maximum_abs_reference_curvature_maneuver_radpm": max(abs(value) for value in kappas),
        "reference_curvature_pattern": curvature_sign,
        "available_merge_distance_after_obstacle_m": available_merge,
        "available_merge_distance_status": merge_status,
        "track_total_width_entry_m": G.nearest_ref(event, G.wrap(event, event["ego_s"] + maneuver_start))["d_left"] + G.nearest_ref(event, G.wrap(event, event["ego_s"] + maneuver_start))["d_right"],
        "track_total_width_obstacle_m": G.nearest_ref(event, G.wrap(event, event["ego_s"] + midpoint))["d_left"] + G.nearest_ref(event, G.wrap(event, event["ego_s"] + midpoint))["d_right"],
        "track_total_width_exit_m": G.nearest_ref(event, G.wrap(event, event["ego_s"] + maneuver_end))["d_left"] + G.nearest_ref(event, G.wrap(event, event["ego_s"] + maneuver_end))["d_right"],
        "track_width_range_over_maneuver_m": max(widths) - min(widths) if widths else 0.0,
        "required_entry_scale": oracle["entry_scale"],
        "required_entry_length_m": transition_lengths(oracle)[0],
        "required_exit_scale": oracle["exit_scale"],
        "required_exit_length_m": transition_lengths(oracle)[1],
    }


def refine_other(event_id: str, event_prod: list[dict], event_valid: list[dict]) -> dict:
    strict = [row for row in event_valid if row["gate"] == "STRICT"] or event_valid
    pairs = [(math.hypot(f(oracle, "d_target") - f(prod, "d_target"),
                         f(oracle, "d_mid") - f(prod, "d_mid")),
              -f(oracle, "minimum_normalized_safety_slack"), oracle, prod)
             for oracle in strict for prod in event_prod if oracle["side"] == prod["side"]]
    _, _, nearest, prod = min(pairs, key=lambda row: row[:2])
    one_mid = [(oracle, candidate) for oracle in strict for candidate in event_prod
               if oracle["side"] == candidate["side"] and close(oracle, candidate, "d_target")
               and close(oracle, candidate, "entry_scale") and close(oracle, candidate, "exit_scale")]
    one_target = [(oracle, candidate) for oracle in strict for candidate in event_prod
                  if oracle["side"] == candidate["side"] and close(oracle, candidate, "d_mid")
                  and close(oracle, candidate, "entry_scale") and close(oracle, candidate, "exit_scale")]
    one_scale = [(oracle, candidate) for oracle in strict for candidate in event_prod
                 if oracle["side"] == candidate["side"] and close(oracle, candidate, "d_target")
                 and close(oracle, candidate, "d_mid")]
    if one_mid:
        refined = "PROBE_SELECTION_OR_D_MID_COVERAGE"
        evidence = "same target and scale pair valid after d_mid-only change"
        nearest, prod = min(one_mid, key=lambda pair: (abs(f(pair[0], "d_mid") - f(pair[1], "d_mid")),
                                                       -f(pair[0], "minimum_normalized_safety_slack")))
    elif one_target:
        refined = "TRUE_UNCLASSIFIED_MECHANISM__D_TARGET_ONLY_COVERAGE_GAP"
        evidence = "same d_mid and scale pair valid after d_target-only change; no direct probe/root/transition isolation"
        nearest, prod = min(one_target, key=lambda pair: (abs(f(pair[0], "d_target") - f(pair[1], "d_target")),
                                                          -f(pair[0], "minimum_normalized_safety_slack")))
    elif one_scale:
        refined = "TRANSITION_TEMPLATE_SELECTION"
        evidence = "same target and mid valid after scale-only change"
        nearest, prod = max(one_scale, key=lambda pair: f(pair[0], "minimum_normalized_safety_slack"))
    else:
        refined = "INTERACTION_OF_MULTIPLE_EXISTING_PARAMETERS"
        evidence = "no valid one-parameter target, mid, or scale counterfactual; nearest valid path changes target and mid"
    return {
        "development_event_id": event_id,
        "original_class": "OTHER",
        "refined_class": refined,
        "evidence": evidence,
        "ranking_or_lifecycle_issue_supported": False,
        "ranking_lifecycle_exclusion_reason": "exact evaluator production hard-valid count is zero, so no valid generated candidate existed to rank or select",
        "production_stage": prod["generator_stage"],
        "production_template": prod["template"],
        "side": prod["side"],
        "production_d_target": prod["d_target"],
        "production_d_mid": prod["d_mid"],
        "production_entry_scale": prod["entry_scale"],
        "production_exit_scale": prod["exit_scale"],
        "production_first_failure": prod["first_failure_reason"],
        "oracle_d_target": nearest["d_target"],
        "oracle_d_mid": nearest["d_mid"],
        "oracle_entry_scale": nearest["entry_scale"],
        "oracle_exit_scale": nearest["exit_scale"],
        "delta_d_target_m": f(nearest, "d_target") - f(prod, "d_target"),
        "delta_d_mid_m": f(nearest, "d_mid") - f(prod, "d_mid"),
        "one_parameter_d_mid_recovery_exists": bool(one_mid),
        "one_parameter_d_target_recovery_exists": bool(one_target),
        "one_parameter_scale_recovery_exists": bool(one_scale),
        **hard_margin_payload(nearest),
    }


def exact_set_coverage(valid: list[dict], valid_events: set[str], template_events: set[str]) -> list[dict]:
    covers: dict[tuple[float, float], set[str]] = defaultdict(set)
    for row in valid:
        if row["event_id"] in valid_events:
            covers[scale_key(row)].add(row["event_id"])
    keys = sorted(covers)
    output = []
    for budget in range(1, 6):
        best = None
        for combo in itertools.combinations(keys, budget):
            covered = set().union(*(covers[key] for key in combo))
            template_covered = covered & template_events
            score = (len(covered), len(template_covered))
            candidate = (score, combo, covered, template_covered)
            if best is None or score > best[0] or (score == best[0] and combo < best[1]):
                best = candidate
        assert best is not None
        _, combo, covered, template_covered = best
        output.append({
            "configuration_budget": budget,
            "selected_entry_exit_scale_pairs": json.dumps(combo),
            "covered_oracle_valid_episode_count": len(covered),
            "oracle_valid_episode_count": len(valid_events),
            "coverage_fraction": len(covered) / len(valid_events),
            "covered_template_case_count": len(template_covered),
            "template_case_count": len(template_events),
            "template_case_coverage_fraction": len(template_covered) / len(template_events),
            "covered_event_ids": "|".join(sorted(covered)),
            "uncovered_event_ids": "|".join(sorted(valid_events - covered)),
            "selection_algorithm": "EXACT_MAXIMUM_SET_COVERAGE_ENUMERATION_TIE_LEXICOGRAPHIC",
            "coverage_condition": "transition pair only; assumes oracle-valid lateral target/mid is available",
        })
    return output


def main() -> None:
    HERE.mkdir(parents=True, exist_ok=True)
    PLOTS.mkdir(exist_ok=True)
    taxonomy = read_csv(CORPUS / "mapping_failure_taxonomy.csv")
    summary = read_csv(CORPUS / "development_oracle_summary.csv")
    production = read_csv(RAW / "production_candidates.csv")
    valid = read_csv(RAW / "oracle_valid_candidates.csv")
    production_by_event: dict[str, list[dict]] = defaultdict(list)
    valid_by_event: dict[str, list[dict]] = defaultdict(list)
    for row in production:
        production_by_event[row["event_id"]].append(row)
    for row in valid:
        valid_by_event[row["event_id"]].append(row)
    template_ids = [row["development_event_id"] for row in taxonomy
                    if row["mechanism_class"] == "TEMPLATE_OR_TRANSITION_SELECTION"]
    other_ids = [row["development_event_id"] for row in taxonomy if row["mechanism_class"] == "OTHER"]
    assert len(template_ids) == 13 and len(other_ids) == 6

    comparisons = []
    case_summary = []
    transition_taxonomy = []
    geometries = []
    for event_id in template_ids:
        event_prod = production_by_event[event_id]
        event_valid = valid_by_event[event_id]
        scale_routes, zero_routes = route_candidates(event_prod, event_valid)
        alternatives = []
        if scale_routes:
            _, _, _, oracle, prod = min(scale_routes, key=lambda row: row[:3])
            alternatives.append(("EXISTING_SCALE_CROSS_COMBINATION", oracle, prod,
                                 int(not close(oracle, prod, "entry_scale")) + int(not close(oracle, prod, "exit_scale")),
                                 abs(transition_lengths(oracle)[0] - transition_lengths(prod)[0])
                                 + abs(transition_lengths(oracle)[1] - transition_lengths(prod)[1])))
        if zero_routes:
            delta, _, oracle, prod = min(zero_routes, key=lambda row: row[:2])
            alternatives.append(("ZERO_INTERFACE_D_MID_FREEDOM", oracle, prod, 1, delta))
        assert alternatives
        primary_alt = min(alternatives, key=lambda row: (row[3], row[4], row[0]))
        route_rows = []
        for route, oracle, prod, _, _ in alternatives:
            row = route_row(event_id, route, oracle, prod, event_prod, (route, oracle, prod) == primary_alt[:3])
            comparisons.append(row)
            route_rows.append(row)
        primary = next(row for row in route_rows if row["is_minimum_change_route"])
        primary_oracle = primary_alt[1]
        event = G.parse_event(RAW / "inputs" / f"{event_id}.event")
        geometries.append(geometry_features(event_id, event, primary_oracle, primary))
        case_summary.append({
            "development_event_id": event_id,
            "minimum_change_route": primary["route"],
            "minimum_change_precise_categories": primary["precise_categories"],
            "minimum_change_option_failure_class": primary["option_failure_class"],
            "alternative_route_count": len(route_rows),
            "all_supported_routes": "|".join(row["route"] for row in route_rows),
            "existing_scale_route_exists": bool(scale_routes),
            "missing_d_mid_route_exists": bool(zero_routes),
            "minimum_changed_coordinate_count": primary_alt[3],
            "minimum_change_magnitude": primary_alt[4],
            "representative_production_template": primary["production_template"],
            "representative_oracle_scale_pair_providers": primary["existing_production_scale_pair_providers"],
            "representative_production_first_failure": primary["production_first_failure"],
        })
        transition_taxonomy.append({
            "development_event_id": event_id,
            "primary_precise_categories": primary["precise_categories"],
            "primary_option_failure_class": primary["option_failure_class"],
            "secondary_supported_categories": "|".join(
                row["precise_categories"] for row in route_rows if row is not primary),
            "scale_pair_already_existed": primary["oracle_scale_pair_present_in_event_production_options"],
            "exact_full_tuple_existed": False,
            "diagnostic_note": (
                "existing scale option was not crossed with the recoverable lateral tuple"
                if primary["option_failure_class"] == "SELECTION_FAILURE_WITH_EXISTING_OPTION" else
                "scale pair existed, but zero-interface did not represent the required d_mid"
            ),
        })

    other_rows = [refine_other(event_id, production_by_event[event_id], valid_by_event[event_id])
                  for event_id in other_ids]
    coverage = exact_set_coverage(valid, {row["development_event_id"] for row in summary
                                          if row["oracle_classification"] == "ORACLE_VALID_P3_EXISTS"},
                                  set(template_ids))
    selected_sets = [set(tuple(pair) for pair in json.loads(row["selected_entry_exit_scale_pairs"]))
                     for row in coverage]
    budget_rows = []
    for coverage_row, selected in zip(coverage, selected_sets):
        matching_production = [row for row in production if scale_key(row) in selected]
        budget = int(coverage_row["configuration_budget"])
        budget_rows.append({
            "configuration_budget": budget,
            "covered_oracle_valid_episode_count": coverage_row["covered_oracle_valid_episode_count"],
            "coverage_fraction": coverage_row["coverage_fraction"],
            "all_40_events_one_lateral_tuple_request_upper_bound": 40 * budget,
            "empirical_existing_production_candidates_with_selected_pairs": len(matching_production),
            "empirical_existing_validator_executions_with_selected_pairs": sum(
                str(row["validator_executed"]).lower() in {"1", "true"} for row in matching_production),
            "events_where_selected_pair_was_actually_generated": len({row["event_id"] for row in matching_production}),
            "current_all_production_candidates_40_events": len(production),
            "estimate_limit": "does not include unknown target/root multiplicity of a future selector; empirical count is a replay proxy",
        })

    write_csv(HERE / "template_case_summary.csv", case_summary)
    write_csv(HERE / "production_vs_oracle_template.csv", comparisons)
    write_csv(HERE / "transition_failure_taxonomy.csv", transition_taxonomy)
    write_csv(HERE / "geometry_features.csv", geometries)
    write_csv(HERE / "other_case_refinement.csv", other_rows)
    write_csv(HERE / "configuration_coverage.csv", coverage)
    write_csv(HERE / "candidate_budget_estimate.csv", budget_rows)

    # Descriptive plots only.
    option_counts = Counter(row["minimum_change_option_failure_class"] for row in case_summary)
    figure, axis = plt.subplots(figsize=(7.5, 4.8))
    labels = ["SELECTION_FAILURE_WITH_EXISTING_OPTION", "COVERAGE_FAILURE_MISSING_OPTION"]
    display_labels = ["existing-option\nselection", "missing full\nconfiguration"]
    axis.bar(display_labels, [option_counts[label] for label in labels], color=["#3978a8", "#b65c3a"])
    axis.set_ylabel("template/transition episodes")
    axis.set_title("Existing-option selection vs missing full configuration")
    axis.grid(axis="y", alpha=.2)
    figure.tight_layout(); figure.savefig(PLOTS / "selection_vs_coverage.png", dpi=170); plt.close(figure)

    figure, axis = plt.subplots(figsize=(7.5, 5.5))
    transition_label_counts: dict[tuple[float, float], int] = defaultdict(int)
    for row in comparisons:
        if not row["is_minimum_change_route"]:
            continue
        point = (round(float(row["oracle_entry_length_m"]), 3),
                 round(float(row["oracle_exit_length_m"]), 3))
        label_index = transition_label_counts[point]
        transition_label_counts[point] += 1
        axis.plot([float(row["production_entry_length_m"]), float(row["oracle_entry_length_m"])],
                  [float(row["production_exit_length_m"]), float(row["oracle_exit_length_m"])],
                  color="0.65", linewidth=1)
        axis.scatter(float(row["oracle_entry_length_m"]), float(row["oracle_exit_length_m"]), s=55)
        axis.annotate(row["development_event_id"],
                      (float(row["oracle_entry_length_m"]), float(row["oracle_exit_length_m"])),
                      xytext=(4, 3 + 10 * label_index), textcoords="offset points", fontsize=7)
    axis.set_xlabel("entry transition length [m]"); axis.set_ylabel("exit transition length [m]")
    axis.set_title("Minimum-change oracle-valid transition lengths")
    axis.grid(alpha=.2); figure.tight_layout()
    figure.savefig(PLOTS / "required_transition_lengths.png", dpi=170); plt.close(figure)

    figure, axis = plt.subplots(figsize=(7.5, 5.2))
    colors = {"SELECTION_FAILURE_WITH_EXISTING_OPTION": "#3978a8",
              "COVERAGE_FAILURE_MISSING_OPTION": "#b65c3a"}
    geometry_label_counts: dict[tuple[float, float], int] = defaultdict(int)
    for row in geometries:
        point = (round(float(row["corridor_bottleneck_width_m"]), 3),
                 round(float(row["maximum_abs_reference_curvature_maneuver_radpm"]), 3))
        label_index = geometry_label_counts[point]
        geometry_label_counts[point] += 1
        axis.scatter(float(row["corridor_bottleneck_width_m"]),
                     float(row["maximum_abs_reference_curvature_maneuver_radpm"]),
                     color=colors[row["option_failure_class"]], s=55)
        axis.annotate(row["development_event_id"],
                      (float(row["corridor_bottleneck_width_m"]),
                       float(row["maximum_abs_reference_curvature_maneuver_radpm"])),
                      xytext=(4, 3 + 10 * label_index), textcoords="offset points", fontsize=7)
    axis.set_xlabel("corridor bottleneck width [m]")
    axis.set_ylabel("max |reference curvature| over maneuver [rad/m]")
    axis.set_title("Pre-generation geometry by recovery mechanism")
    axis.grid(alpha=.2); figure.tight_layout()
    figure.savefig(PLOTS / "geometry_mechanism_scatter.png", dpi=170); plt.close(figure)

    figure, axis = plt.subplots(figsize=(7.5, 4.8))
    x = [int(row["configuration_budget"]) for row in coverage]
    y = [float(row["coverage_fraction"]) for row in coverage]
    yt = [float(row["template_case_coverage_fraction"]) for row in coverage]
    axis.plot(x, y, marker="o", label="all 23 oracle-valid")
    axis.plot(x, yt, marker="s", label="13 template/transition")
    axis.set_xticks(x); axis.set_ylim(0, 1.05)
    axis.set_xlabel("entry/exit scale-pair budget")
    axis.set_ylabel("episode coverage")
    axis.set_title("Exact deterministic scale-pair set coverage")
    axis.legend(); axis.grid(alpha=.2); figure.tight_layout()
    figure.savefig(PLOTS / "configuration_set_coverage.png", dpi=170); plt.close(figure)

    figure, axis = plt.subplots(figsize=(7.5, 5))
    for row in other_rows:
        axis.scatter(float(row["delta_d_target_m"]), float(row["delta_d_mid_m"]), s=65)
        axis.annotate(row["development_event_id"],
                      (float(row["delta_d_target_m"]), float(row["delta_d_mid_m"])),
                      xytext=(4, 3), textcoords="offset points", fontsize=8)
    axis.axhline(0, color="black", linewidth=.7); axis.axvline(0, color="black", linewidth=.7)
    axis.set_xlabel("oracle - production d_target [m]")
    axis.set_ylabel("oracle - production d_mid [m]")
    axis.set_title("Refined OTHER: nearest supported lateral changes")
    axis.grid(alpha=.2); figure.tight_layout()
    figure.savefig(PLOTS / "other_lateral_parameter_changes.png", dpi=170); plt.close(figure)

    result = {
        "template_case_count": len(case_summary),
        "minimum_route_option_failure_counts": dict(option_counts),
        "other_refined_counts": dict(Counter(row["refined_class"] for row in other_rows)),
        "best_scale_pair_coverage": [{"budget": row["configuration_budget"],
                                      "covered": row["covered_oracle_valid_episode_count"],
                                      "fraction": row["coverage_fraction"]} for row in coverage],
        "new_oracle_searches_run": 0,
        "validation_or_holdout_rows_read": 0,
    }
    (HERE / "analysis_summary.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
