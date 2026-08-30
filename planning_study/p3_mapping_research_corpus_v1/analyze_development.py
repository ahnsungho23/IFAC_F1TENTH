#!/usr/bin/env python3
"""Diagnose frozen DEVELOPMENT oracle results without fitting or changing a mapping."""

from __future__ import annotations

import csv
import importlib.util
import json
import math
import os
from collections import Counter, defaultdict
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/p3_mapping_research_mpl")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
RAW = HERE / "raw_oracle"
PRIOR_DIAGNOSIS = REPO / "planning_study/p3_mapping_miss_diagnosis_v1/analyze_mapping_miss.py"
PLOTS = HERE / "plots"
EPS = 1.0e-9


def load_geometry_module():
    spec = importlib.util.spec_from_file_location("frozen_mapping_diagnosis", PRIOR_DIAGNOSIS)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


G = load_geometry_module()


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


def number(row: dict, key: str, default: float = math.nan) -> float:
    value = row.get(key, "")
    return float(value) if value not in ("", None) else default


def truth(value) -> bool:
    return str(value).lower() in {"1", "true", "yes"}


def close(a: dict, b: dict, key: str, tolerance: float = 1.0e-9) -> bool:
    return abs(number(a, key) - number(b, key)) <= tolerance


def same_tuple(oracle: dict, production: dict, include_mid: bool) -> bool:
    keys = ["d_target", "entry_scale", "exit_scale"]
    if include_mid:
        keys.append("d_mid")
    return oracle["side"] == production["side"] and all(close(oracle, production, key) for key in keys)


def best_safety(row: dict) -> float:
    return number(row, "minimum_normalized_safety_slack", -math.inf)


def main() -> None:
    PLOTS.mkdir(exist_ok=True)
    summary = read_csv(RAW / "oracle_summary.csv")
    production = read_csv(RAW / "production_candidates.csv")
    valid = read_csv(RAW / "oracle_valid_candidates.csv")
    development_manifest = read_csv(HERE / "development_oracle_manifest.csv")
    search_domain = json.loads((RAW / "search_domain.json").read_text(encoding="utf-8"))
    manifest_by_event = {row["development_event_id"]: row for row in development_manifest}
    production_by_event: dict[str, list[dict]] = defaultdict(list)
    valid_by_event: dict[str, list[dict]] = defaultdict(list)
    for row in production:
        production_by_event[row["event_id"]].append(row)
    for row in valid:
        valid_by_event[row["event_id"]].append(row)

    taxonomy = []
    geometry_rows = []
    correction_rows = []
    station_rows = []

    for event_summary in summary:
        event_id = event_summary["event_id"]
        if event_summary["classification"] != "ORACLE_VALID_P3_EXISTS":
            continue
        event = G.parse_event(RAW / "inputs" / f"{event_id}.event")
        event_prod = production_by_event[event_id]
        event_valid = valid_by_event[event_id]
        lineage = (truth(event_summary["exact_input_lineage"])
                   and truth(event_summary["constructed_digest_multiset_parity"])
                   and truth(event_summary["returned_digest_multiset_parity"]))
        base = {
            "development_event_id": event_id,
            "dataset_role": "DEVELOPMENT",
            "episode_id": manifest_by_event[event_id]["episode_id"],
            "bag": event_summary["bag"],
            "elapsed_s": event_summary["elapsed_s"],
            "callback_sequence": event_summary["callback_sequence"],
            "evaluation_sequence": event_summary["evaluation_sequence"],
            "exact_lineage_and_reconstruction_parity": lineage,
            "production_hard_valid_count": event_summary["production_hard_valid_count"],
            "oracle_hard_valid_count": event_summary["oracle_hard_valid_count"],
        }
        if not lineage:
            taxonomy.append(base | {"mechanism_class": "NOT_IDENTIFIABLE",
                                    "classification_evidence": "lineage or reconstruction parity failed"})
            continue

        exact_mid_pairs = []
        analytic_pairs = []
        zero_interface_pairs = []
        for oracle in event_valid:
            for prod in event_prod:
                same_target_and_mid = (oracle["side"] == prod["side"]
                                       and close(oracle, prod, "d_target")
                                       and close(oracle, prod, "d_mid"))
                if same_target_and_mid:
                    scale_changed = not (close(oracle, prod, "entry_scale") and close(oracle, prod, "exit_scale"))
                    if scale_changed:
                        exact_mid_pairs.append((oracle, prod))
                if (same_tuple(oracle, prod, include_mid=False)
                        and abs(number(oracle, "d_mid") - number(prod, "d_mid")) > 1.0e-6):
                    if prod.get("s_probe", ""):
                        analytic_pairs.append((oracle, prod))
                    else:
                        zero_interface_pairs.append((oracle, prod))

        # Existing d_target/d_mid becomes valid under another existing transition pairing.
        if exact_mid_pairs:
            oracle, prod = max(exact_mid_pairs, key=lambda pair: best_safety(pair[0]))
            taxonomy.append(base | {
                "mechanism_class": "TEMPLATE_OR_TRANSITION_SELECTION",
                "classification_evidence": "exact production d_target and d_mid valid with another existing entry/exit pair",
                "representative_side": oracle["side"],
                "representative_d_target": oracle["d_target"],
                "representative_d_mid": oracle["d_mid"],
                "production_entry_scale": prod["entry_scale"],
                "production_exit_scale": prod["exit_scale"],
                "oracle_entry_scale": oracle["entry_scale"],
                "oracle_exit_scale": oracle["exit_scale"],
            })
            continue

        # Prefer STRICT exact-station analytic evidence, then nearest d_mid to production.
        strict_analytic = [pair for pair in analytic_pairs if pair[0]["gate"] == "STRICT"
                           and all(close(pair[0], pair[1], f"z{i}", 1.0e-8) for i in range(5))]
        if strict_analytic:
            oracle, prod = min(
                strict_analytic,
                key=lambda pair: (abs(number(pair[0], "d_mid") - number(pair[1], "d_mid")),
                                  -best_safety(pair[0])),
            )
            left = prod["side"] == "LEFT"
            corridor = G.make_corridor(event, left)
            lower, upper, bound_rule = G.root_bounds(event, prod, corridor, search_domain)
            stations = [number(prod, f"z{i}") for i in range(5)]
            s_probe = number(prod, "s_probe")
            d_probe = number(prod, "d_probe")
            target = number(oracle, "d_target")
            oracle_mid = number(oracle, "d_mid")
            required = G.eval_profile(stations, event["ego_d"], target, oracle_mid, s_probe)
            correction = required - d_probe
            equation = G.equation_details(stations, event["ego_d"], number(prod, "d_target"),
                                          s_probe, d_probe, lower, upper)
            oracle_branch = "__".join(G.knot_states(
                stations, [event["ego_d"], target, oracle_mid, target, 0.0])[3])
            branch_ok = oracle_branch == equation["assumed_branch_regime"]
            bounds_ok = lower - EPS <= oracle_mid <= upper + EPS
            policy = "CURVATURE_CONTINUITY" if prod["generator_stage"] == "M0_V1" else "BOTTLENECK_CENTER"
            start = max(0.0, corridor["start"]) + 1.0e-8
            end = corridor["end"] - 1.0e-8
            local_scan = []
            if end > start:
                for station in np.linspace(start, end, 501):
                    sample, current_anchor = G.forced_anchor(event, left, float(station), target, policy)
                    oracle_d = G.eval_profile(stations, event["ego_d"], target, oracle_mid, float(station))
                    local_scan.append({
                        "development_event_id": event_id,
                        "production_stage": prod["generator_stage"],
                        "production_template": prod["template"],
                        "probe_policy": policy,
                        "station_forward_m": float(station),
                        "current_policy_d_probe": current_anchor,
                        "oracle_d_at_station": oracle_d,
                        "residual_m": oracle_d - current_anchor,
                        "branch_lower": sample["lower"],
                        "branch_upper": sample["upper"],
                    })
            minimum = min(local_scan, key=lambda row: abs(row["residual_m"]))
            zero_crossing = any(a["residual_m"] * b["residual_m"] < 0.0
                                for a, b in zip(local_scan, local_scan[1:]))
            for row in local_scan:
                row["is_minimum_abs_residual"] = row is minimum
                row["minimum_abs_residual_m"] = abs(minimum["residual_m"])
                row["zero_crossing_exists"] = zero_crossing
            station_rows.extend(local_scan)

            if not branch_ok or not bounds_ok:
                mechanism = "ROOT_FILTERING_DOMINANT"
                evidence = "oracle d_mid fails current analytic branch or lateral-root bound"
            elif abs(minimum["residual_m"]) <= 1.0e-4:
                mechanism = "S_PROBE_DOMINANT"
                evidence = "current anchor policy intersects oracle path at another station within 1e-4 m"
            else:
                mechanism = "D_PROBE_DOMINANT"
                evidence = "fixed-s anchor correction recovers oracle tuple; current-anchor station sweep has no intersection"
            taxonomy.append(base | {
                "mechanism_class": mechanism,
                "classification_evidence": evidence,
                "production_stage": prod["generator_stage"],
                "production_template": prod["template"],
                "side": prod["side"],
                "production_s_probe": s_probe,
                "production_d_probe": d_probe,
                "production_d_target": prod["d_target"],
                "production_d_mid": prod["d_mid"],
                "nearest_oracle_valid_d_mid": oracle_mid,
                "d_oracle_at_production_s_probe": required,
                "required_delta_d_probe_m": correction,
                "branch_filter_pass_if_anchor_changed": branch_ok,
                "root_bound_pass_if_anchor_changed": bounds_ok,
                "root_lower_bound": lower,
                "root_upper_bound": upper,
                "root_bound_rule": bound_rule,
                "station_sweep_min_abs_residual_m": abs(minimum["residual_m"]),
                "station_sweep_argmin_m": minimum["station_forward_m"],
                "station_sweep_zero_crossing": zero_crossing,
            })

            if mechanism == "D_PROBE_DOMINANT":
                sample = G.sample_at(event, corridor["visible"], s_probe, left)
                active = [obstacle for obstacle in corridor["visible"]
                          if obstacle["start"] - EPS <= s_probe <= obstacle["end"] + EPS]
                active_right = min((o["d_right"] for o in active), default=math.nan)
                active_left = max((o["d_left"] for o in active), default=math.nan)
                geometry = {
                    "development_event_id": event_id,
                    "bag": event_summary["bag"],
                    "reference_snapshot_id": manifest_by_event[event_id]["reference_snapshot_id"],
                    "side": prod["side"],
                    "production_stage": prod["generator_stage"],
                    "production_template": prod["template"],
                    "ego_d_m": event["ego_d"],
                    "ego_speed_mps": event["ego_speed"],
                    "d_target_m": target,
                    "s_probe_forward_m": s_probe,
                    "d_probe_m": d_probe,
                    "corridor_branch_lower_m": sample["lower"],
                    "corridor_branch_upper_m": sample["upper"],
                    "corridor_branch_width_m": sample["width"],
                    "corridor_branch_center_m": sample["center"],
                    "d_probe_distance_from_lower_m": d_probe - sample["lower"],
                    "d_probe_distance_from_upper_m": sample["upper"] - d_probe,
                    "track_center_interval_lower_m": sample["track_lower"],
                    "track_center_interval_upper_m": sample["track_upper"],
                    "active_obstacle_ids": json.dumps(sample["active_obstacles"]),
                    "active_inflated_obstacle_right_m": active_right,
                    "active_inflated_obstacle_left_m": active_left,
                    "cluster_start_forward_m": corridor["start"],
                    "cluster_end_forward_m": corridor["end"],
                    "cluster_start_relative_to_s_probe_m": corridor["start"] - s_probe,
                    "cluster_end_relative_to_s_probe_m": corridor["end"] - s_probe,
                    "local_reference_curvature_radpm": sample["curvature"],
                    "curvature_sign": "POSITIVE" if sample["curvature"] > 0 else "NEGATIVE" if sample["curvature"] < 0 else "ZERO",
                    "entry_scale": prod["entry_scale"],
                    "exit_scale": prod["exit_scale"],
                    "footprint_aware_interval_status": "NO_EXACT_CANDIDATE_INDEPENDENT_INTERVAL",
                    "footprint_aware_interval_note": "exact validator interval depends on generated path yaw; track center interval already subtracts half-width and wall margin",
                }
                geometry_rows.append(geometry)
                correction_rows.append({
                    **base,
                    "side": prod["side"],
                    "production_stage": prod["generator_stage"],
                    "production_s_probe": s_probe,
                    "production_d_probe": d_probe,
                    "d_oracle_at_production_s_probe": required,
                    "delta_d_probe_required_m": correction,
                    "absolute_delta_d_probe_required_m": abs(correction),
                    "correction_direction": "TOWARD_ZERO" if abs(required) < abs(d_probe) else "AWAY_FROM_ZERO",
                    "corridor_width_m": sample["width"],
                    "corridor_center_m": sample["center"],
                    "d_probe_minus_corridor_center_m": d_probe - sample["center"],
                    "station_sweep_min_abs_residual_m": abs(minimum["residual_m"]),
                    "station_sweep_zero_crossing": zero_crossing,
                })
            continue

        if zero_interface_pairs:
            oracle, prod = min(zero_interface_pairs,
                               key=lambda pair: (abs(number(pair[0], "d_mid") - number(pair[1], "d_mid")),
                                                 -best_safety(pair[0])))
            taxonomy.append(base | {
                "mechanism_class": "TEMPLATE_OR_TRANSITION_SELECTION",
                "classification_evidence": "same target and scales need d_mid freedom absent from zero-interface template; no analytic probe exists",
                "production_stage": prod["generator_stage"],
                "production_template": prod["template"],
                "side": prod["side"],
                "production_d_target": prod["d_target"],
                "production_d_mid": prod["d_mid"],
                "nearest_oracle_valid_d_mid": oracle["d_mid"],
            })
        else:
            taxonomy.append(base | {
                "mechanism_class": "OTHER",
                "classification_evidence": "oracle validity requires a different target and/or other P3 degrees; no direct probe or transition isolation",
                "representative_side": event_summary.get("best_side", ""),
                "representative_d_target": event_summary.get("best_d_target", ""),
                "representative_d_mid": event_summary.get("best_d_mid", ""),
            })

    assert len(taxonomy) == sum(row["classification"] == "ORACLE_VALID_P3_EXISTS" for row in summary)
    write_csv(HERE / "mapping_failure_taxonomy.csv", taxonomy)
    write_csv(HERE / "dprobe_geometry_features.csv", geometry_rows)
    write_csv(HERE / "dprobe_required_correction.csv", correction_rows)
    write_csv(HERE / "dprobe_dominant_cases.csv", [row for row in taxonomy
                                                    if row["mechanism_class"] == "D_PROBE_DOMINANT"])
    write_csv(HERE / "station_counterfactual_scan.csv", station_rows)

    public_summary = []
    taxonomy_by_event = {row["development_event_id"]: row for row in taxonomy}
    for row in summary:
        public_summary.append({
            "development_event_id": row["event_id"],
            "dataset_role": "DEVELOPMENT",
            "episode_id": manifest_by_event[row["event_id"]]["episode_id"],
            "bag": row["bag"],
            "elapsed_s": row["elapsed_s"],
            "callback_sequence": row["callback_sequence"],
            "evaluation_sequence": row["evaluation_sequence"],
            "exact_lineage": row["exact_input_lineage"],
            "constructed_digest_parity": row["constructed_digest_multiset_parity"],
            "returned_digest_parity": row["returned_digest_multiset_parity"],
            "production_constructed_count": row["production_constructed_count"],
            "production_returned_count": row["production_returned_count"],
            "production_hard_valid_count": row["production_hard_valid_count"],
            "oracle_total_requests": row["total_request_count"],
            "oracle_constructed_paths": row["constructed_path_count"],
            "oracle_validator_executions": row["validator_execution_count"],
            "oracle_hard_valid_count": row["oracle_hard_valid_count"],
            "oracle_runtime_s": row["oracle_runtime_s"],
            "oracle_classification": row["classification"],
            "mechanism_class": taxonomy_by_event.get(row["event_id"], {}).get("mechanism_class", "NOT_APPLICABLE_NO_ORACLE_VALID"),
        })
    write_csv(HERE / "development_oracle_summary.csv", public_summary)

    computation = []
    for row in public_summary:
        computation.append({
            "scope": row["development_event_id"],
            "total_requests": row["oracle_total_requests"],
            "constructed_paths": row["oracle_constructed_paths"],
            "validator_executions": row["oracle_validator_executions"],
            "hard_valid_rows": row["oracle_hard_valid_count"],
            "runtime_s": row["oracle_runtime_s"],
            "cache_policy": "reference and evaluator logs indexed once; duplicate oracle requests removed exactly",
        })
    computation.append({
        "scope": "TOTAL_40_DEVELOPMENT_EPISODES",
        "total_requests": sum(int(row["oracle_total_requests"]) for row in public_summary),
        "constructed_paths": sum(int(row["oracle_constructed_paths"]) for row in public_summary),
        "validator_executions": sum(int(row["oracle_validator_executions"]) for row in public_summary),
        "hard_valid_rows": sum(int(row["oracle_hard_valid_count"]) for row in public_summary),
        "runtime_s": sum(float(row["oracle_runtime_s"]) for row in public_summary),
        "cache_policy": "behavior-preserving exact input indexing and request de-duplication",
    })
    write_csv(HERE / "computation_summary.csv", computation)

    counts = Counter(row["mechanism_class"] for row in taxonomy)
    outcomes = Counter(row["oracle_classification"] for row in public_summary)
    corrections = [row["delta_d_probe_required_m"] for row in correction_rows]
    analysis = {
        "development_episode_count_searched": len(public_summary),
        "conclusive_oracle_count": sum(truth(row["exact_lineage"]) and truth(row["constructed_digest_parity"])
                                       and truth(row["returned_digest_parity"]) for row in public_summary),
        "oracle_valid_count": outcomes["ORACLE_VALID_P3_EXISTS"],
        "oracle_invalid_count": outcomes["NO_VALID_P3_FOUND_IN_ORACLE_DOMAIN"],
        "mechanism_counts_among_oracle_valid": dict(counts),
        "dprobe_dominant_count": counts["D_PROBE_DOMINANT"],
        "dprobe_bag_count": len({row["bag"] for row in correction_rows}),
        "dprobe_reference_domain_count": len({manifest_by_event[row["development_event_id"]]["reference_snapshot_id"]
                                               for row in correction_rows}),
        "dprobe_delta_signed_min_m": min(corrections, default=math.nan),
        "dprobe_delta_signed_median_m": float(np.median(corrections)) if corrections else math.nan,
        "dprobe_delta_signed_max_m": max(corrections, default=math.nan),
        "dprobe_delta_abs_median_m": float(np.median(np.abs(corrections))) if corrections else math.nan,
        "dprobe_corrections_toward_zero_count": sum(row["correction_direction"] == "TOWARD_ZERO" for row in correction_rows),
    }
    (HERE / "analysis_summary.json").write_text(json.dumps(analysis, indent=2) + "\n", encoding="utf-8")

    # Descriptive plots only; no model or mapping is fitted.
    labels = ["D_PROBE_DOMINANT", "S_PROBE_DOMINANT", "JOINT_S_AND_D_PROBE",
              "ROOT_FILTERING_DOMINANT", "TEMPLATE_OR_TRANSITION_SELECTION", "OTHER", "NOT_IDENTIFIABLE"]
    figure, axis = plt.subplots(figsize=(10, 5))
    axis.bar(range(len(labels)), [counts[label] for label in labels], color="#3b6ea8")
    axis.set_xticks(range(len(labels)), labels, rotation=28, ha="right")
    axis.set_ylabel("independent DEVELOPMENT episodes")
    axis.set_title("Mechanism taxonomy among oracle-valid episodes")
    axis.grid(axis="y", alpha=.2)
    figure.tight_layout(); figure.savefig(PLOTS / "mechanism_frequency.png", dpi=170); plt.close(figure)

    figure, axis = plt.subplots(figsize=(8, 4.8))
    names = [row["development_event_id"] for row in correction_rows]
    colors = ["#b33c2e" if value < 0 else "#2e73b8" for value in corrections]
    axis.bar(names, corrections, color=colors)
    axis.axhline(0.0, color="black", linewidth=.8)
    axis.set_ylabel("required delta d_probe [m]")
    axis.set_title("D_PROBE_DOMINANT fixed-s corrections")
    axis.grid(axis="y", alpha=.2)
    figure.tight_layout(); figure.savefig(PLOTS / "dprobe_required_correction.png", dpi=170); plt.close(figure)

    figure, axis = plt.subplots(figsize=(7, 5))
    for row in correction_rows:
        axis.scatter(row["corridor_width_m"], row["delta_d_probe_required_m"], s=65)
        axis.annotate(row["development_event_id"], (row["corridor_width_m"], row["delta_d_probe_required_m"]),
                      xytext=(5, 4), textcoords="offset points", fontsize=8)
    axis.axhline(0.0, color="black", linewidth=.8)
    axis.set_xlabel("selected-side corridor width at production s_probe [m]")
    axis.set_ylabel("required delta d_probe [m]")
    axis.set_title("Descriptive geometry vs correction (no fitted policy)")
    axis.grid(alpha=.2)
    figure.tight_layout(); figure.savefig(PLOTS / "dprobe_correction_vs_corridor_width.png", dpi=170); plt.close(figure)

    figure, axis = plt.subplots(figsize=(9, 5))
    grouped: dict[str, list[dict]] = defaultdict(list)
    for row in station_rows:
        grouped[row["development_event_id"]].append(row)
    for event_id, rows in grouped.items():
        axis.plot([float(row["station_forward_m"]) for row in rows],
                  [float(row["residual_m"]) for row in rows], label=event_id)
    axis.axhline(0.0, color="black", linewidth=.8)
    axis.set_xlabel("counterfactual probe station [m]")
    axis.set_ylabel("d_oracle(s) - current-policy d_probe(s) [m]")
    axis.set_title("Current-anchor station counterfactual for analytic cases")
    axis.legend(fontsize=8); axis.grid(alpha=.2)
    figure.tight_layout(); figure.savefig(PLOTS / "station_counterfactual_residual.png", dpi=170); plt.close(figure)

    bags = sorted({row["bag"] for row in public_summary})
    valid_counts = [sum(row["bag"] == bag and row["oracle_classification"] == "ORACLE_VALID_P3_EXISTS"
                        for row in public_summary) for bag in bags]
    invalid_counts = [sum(row["bag"] == bag and row["oracle_classification"] != "ORACLE_VALID_P3_EXISTS"
                          for row in public_summary) for bag in bags]
    figure, axis = plt.subplots(figsize=(9, 5))
    short = [bag.replace("rosbag2_2026_", "") for bag in bags]
    axis.bar(short, valid_counts, label="oracle valid", color="#3a8c5a")
    axis.bar(short, invalid_counts, bottom=valid_counts, label="no valid in domain", color="#999999")
    axis.set_ylabel("independent DEVELOPMENT episodes")
    axis.set_title("Oracle outcome by bag")
    axis.legend(); axis.grid(axis="y", alpha=.2)
    figure.tight_layout(); figure.savefig(PLOTS / "oracle_outcome_by_bag.png", dpi=170); plt.close(figure)
    print(json.dumps(analysis, indent=2))


if __name__ == "__main__":
    main()
