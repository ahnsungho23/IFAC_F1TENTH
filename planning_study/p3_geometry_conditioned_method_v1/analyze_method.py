#!/usr/bin/env python3
"""DEVELOPMENT-only bounded factorized P3 method prototype.

The selector sees only pre-validation geometry and factor metadata.  Oracle outcomes are
loaded only after selection to define the 23-episode evaluation universe/mechanism labels.
No production source, configuration, validator, or dataset split is modified.
"""

from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import math
import os
import subprocess
import time
from collections import Counter, defaultdict
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/p3_geometry_conditioned_method_mpl")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
CORPUS = REPO / "planning_study/p3_mapping_research_corpus_v1"
RAW = CORPUS / "raw_oracle"
FACTOR = REPO / "planning_study/p3_factorized_candidate_design_v1"
GEOMETRY_SOURCE = REPO / "planning_study/p3_mapping_miss_diagnosis_v1/analyze_mapping_miss.py"
HARNESS = Path("/tmp/p3_oracle_v2_build/p3_family_oracle_harness")
EXPECTED_HARNESS_SHA256 = "8b23f2b6df54c2e93794ee087f3ebd7a14f9921544af4f97eef755535e045e7e"
WORK = Path("/tmp/p3_geometry_conditioned_method_v1")
PLOTS = HERE / "plots"
LOG_ROOT = Path("/tmp/p3_oracle_v2_logs")
REFERENCE_STREAM = Path("/tmp/p3_oracle_snapshots_v1.tsv")
PREPARATION = REPO / "planning_study/p3_oracle_pilot_v2_preparation"
EPS = 1.0e-9
BUDGETS = [4, 8, 12, 16, 24]
SELECTORS = [
    "H3_FIXED_BOUNDED",
    "H4A_GEOMETRY_TRANSITION",
    "H4B_GEOMETRY_LATERAL_TRANSITION",
]


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


G = load_module("method_geometry", GEOMETRY_SOURCE)


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


def number(row: dict, key: str, default: float = math.nan) -> float:
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def truth(value) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def percentile(values: list[float], q: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=float), q)) if values else math.nan


def clamp(value: float, lower: float = -1.5, upper: float = 1.5) -> float:
    return min(max(value, lower), upper)


def records(path: Path, kind: str) -> list[dict]:
    rows = [line.split("\t") for line in path.read_text(encoding="utf-8").splitlines()
            if line.startswith(kind + "\t")]
    if not rows:
        return []
    header = rows[0][1:]
    return [dict(zip(header, row[1:])) for row in rows[1:]]


def jsonl(path: Path):
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def write_requests(path: Path, selector: str, configurations: list[dict]) -> None:
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write("P3_ORACLE_REQUESTS_V1\n")
        for config in configurations:
            stream.write(
                "Q\t{}\t0\t{}\t{:.17g}\t{:.17g}\t{:.17g}\t{:.17g}\n".format(
                    selector, int(config["side"] == "LEFT"), config["d_target"],
                    config["d_mid"], config["entry_scale"], config["exit_scale"])
            )


def run_harness(event_id: str, event_path: Path, selector: str, budget: int,
                configurations: list[dict]) -> tuple[list[dict], float]:
    request = WORK / f"{event_id}.{selector}.K{budget}.requests"
    output = WORK / f"{event_id}.{selector}.K{budget}.out"
    write_requests(request, selector, configurations)
    started = time.perf_counter()
    with output.open("w", encoding="utf-8", newline="\n") as stream:
        subprocess.run([str(HARNESS), str(event_path), str(request)], check=True, stdout=stream)
    return records(output, "CANDIDATE"), time.perf_counter() - started


def config_key(row: dict) -> tuple:
    return (row["side"], float(row["d_target"]), float(row["d_mid"]),
            float(row["entry_scale"]), float(row["exit_scale"]))


def unique_values(values) -> list[float]:
    output = []
    for value in sorted(float(item) for item in values):
        if not output or abs(value - output[-1]) > 1.0e-12:
            output.append(value)
    return output


def normalized(value: float, values: list[float]) -> float:
    if len(values) <= 1 or values[-1] - values[0] <= EPS:
        return 0.0
    return (value - values[0]) / (values[-1] - values[0])


def transition_priority(entry_norm: float, exit_norm: float) -> float:
    archetypes = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0), (0.5, 0.5)]
    return min(index + math.hypot(entry_norm - entry, exit_norm - exit)
               for index, (entry, exit) in enumerate(archetypes))


def side_geometry(event: dict, side: str) -> dict:
    left = side == "LEFT"
    corridor = G.make_corridor(event, left)
    cluster = corridor["cluster"]
    start = min(row["start"] for row in cluster)
    end = max(row["end"] for row in cluster)
    midpoint = 0.5 * (start + end)
    midpoint_sample = G.sample_at(event, corridor["visible"], midpoint, left)
    span = G.span_samples(corridor)
    bottleneck = min(span, key=lambda row: (row["width"], row["station"]))
    domain_low, domain_high = ((0.0, 1.5) if left else (-1.5, 0.0))
    components = G.connected_ranges(corridor, domain_low, domain_high)

    cluster_ids = {row["id"] for row in cluster}
    later = [row for row in corridor["visible"]
             if row["id"] not in cluster_ids and row["start"] > end + EPS]
    if later:
        merge_distance = min(row["start"] for row in later) - end
        merge_status = "TO_NEXT_VISIBLE_OBSTACLE"
    else:
        merge_distance = max(0.0, 15.0 - end)
        merge_status = "CENSORED_AT_15M_LOOKAHEAD"

    horizon = min(15.0, end + merge_distance)
    local = G.local_reference_points(event, max(horizon, end))
    kappas = [row["kappa"] for station, row in local if station + EPS >= max(0.0, start)]
    if not kappas:
        kappas = [G.nearest_ref(event, event["ego_s"])["kappa"]]
    widths = [row["d_left"] + row["d_right"] for station, row in local
              if station + EPS >= max(0.0, start)]
    sign_change = min(kappas) < -0.05 and max(kappas) > 0.05
    pattern = "SIGN_CHANGE" if sign_change else (
        "POSITIVE" if max(kappas) > 0.05 else
        "NEGATIVE" if min(kappas) < -0.05 else "NEAR_ZERO")
    entry_ref = G.nearest_ref(event, G.wrap(event, event["ego_s"] + max(0.0, start)))
    obstacle_ref = G.nearest_ref(event, G.wrap(event, event["ego_s"] + midpoint))
    exit_ref = G.nearest_ref(event, G.wrap(event, event["ego_s"] + end + merge_distance))
    return {
        "side": side, "left": left, "corridor": corridor, "components": components,
        "cluster_start": start, "cluster_end": end, "cluster_span": end - start,
        "raw_d_right": min(row["raw_d_right"] for row in cluster),
        "raw_d_left": max(row["raw_d_left"] for row in cluster),
        "inflated_d_right": min(row["d_right"] for row in cluster),
        "inflated_d_left": max(row["d_left"] for row in cluster),
        "midpoint_lower": midpoint_sample["lower"],
        "midpoint_upper": midpoint_sample["upper"],
        "midpoint_center": midpoint_sample["center"],
        "midpoint_width": midpoint_sample["width"],
        "bottleneck_lower": bottleneck["lower"],
        "bottleneck_upper": bottleneck["upper"],
        "bottleneck_center": bottleneck["center"],
        "bottleneck_width": bottleneck["width"],
        "bottleneck_station": bottleneck["station"],
        "available_entry_distance": max(0.0, start),
        "merge_distance": merge_distance, "merge_status": merge_status,
        "kappa_entry": entry_ref["kappa"], "kappa_obstacle": obstacle_ref["kappa"],
        "kappa_exit": exit_ref["kappa"],
        "max_abs_kappa": max(abs(value) for value in kappas),
        "curvature_sign_change": sign_change, "curvature_pattern": pattern,
        "track_width_variation": max(widths) - min(widths) if widths else 0.0,
    }


def feature_row(event_id: str, event: dict, geometry: dict) -> dict:
    components = geometry["components"]
    return {
        "development_event_id": event_id, "feature_scope": "EVENT_SIDE",
        "online_feature_contract": "PRE_PLANNING_ONLY_NO_VALIDATOR_OR_ORACLE_OUTCOME",
        "side": geometry["side"], "ego_d_m": event["ego_d"],
        "ego_speed_mps": event["ego_speed"],
        "obstacle_start_distance_m": geometry["cluster_start"],
        "available_entry_distance_m": geometry["available_entry_distance"],
        "obstacle_longitudinal_span_m": geometry["cluster_span"],
        "inflated_obstacle_d_right_m": geometry["inflated_d_right"],
        "inflated_obstacle_d_left_m": geometry["inflated_d_left"],
        "chosen_side_free_lower_m": geometry["midpoint_lower"],
        "chosen_side_free_upper_m": geometry["midpoint_upper"],
        "chosen_side_free_width_m": geometry["midpoint_width"],
        "corridor_bottleneck_lower_m": geometry["bottleneck_lower"],
        "corridor_bottleneck_upper_m": geometry["bottleneck_upper"],
        "corridor_bottleneck_width_m": geometry["bottleneck_width"],
        "corridor_bottleneck_center_m": geometry["bottleneck_center"],
        "corridor_bottleneck_station_m": geometry["bottleneck_station"],
        "available_merge_exit_distance_m": geometry["merge_distance"],
        "available_merge_exit_status": geometry["merge_status"],
        "reference_curvature_entry_radpm": geometry["kappa_entry"],
        "reference_curvature_obstacle_radpm": geometry["kappa_obstacle"],
        "reference_curvature_exit_radpm": geometry["kappa_exit"],
        "maximum_abs_reference_curvature_radpm": geometry["max_abs_kappa"],
        "curvature_sign_change": geometry["curvature_sign_change"],
        "curvature_pattern": geometry["curvature_pattern"],
        "track_width_variation_m": geometry["track_width_variation"],
        "corridor_component_count": len(components),
        "corridor_components_json": json.dumps(components),
    }


def lateral_factors(event_id: str, side: str, geometry: dict,
                    production: list[dict]) -> list[dict]:
    factors: dict[tuple[float, float], dict] = {}

    def add(target: float, middle: float, target_source: str, mid_source: str,
            priority: float) -> None:
        key = (clamp(target), clamp(middle))
        if key not in factors or priority < factors[key]["lateral_priority"]:
            factors[key] = {
                "side": side, "d_target": key[0], "d_mid": key[1],
                "target_source": target_source, "mid_source": mid_source,
                "lateral_priority": priority,
                "lateral_factor_id": "L_" + hashlib.sha256(
                    f"{event_id}|{side}|{key[0]:.17g}|{key[1]:.17g}".encode()).hexdigest()[:12],
            }

    side_prod = [row for row in production if row["side"] == side]
    for row in side_prod:
        add(number(row, "d_target"), number(row, "d_mid"),
            "PRODUCTION_TARGET", "PRODUCTION_MID", 0.0)

    targets: list[tuple[float, str, float]] = []
    for target in unique_values(number(row, "d_target") for row in side_prod):
        targets.append((target, "PRODUCTION_TARGET", 0.0))
    for component_index, (lower, upper) in enumerate(geometry["components"]):
        near, far = sorted((lower, upper), key=lambda value: (abs(value), value))
        anchors = [
            (near + (far - near) / 64.0, "COMPONENT_NEAR_INSET", 1.0),
            (near + 0.25 * (far - near), "COMPONENT_QUARTER", 2.0),
            (0.5 * (near + far), "COMPONENT_CENTER", 3.0),
            (near + 0.75 * (far - near), "COMPONENT_THREE_QUARTER", 4.0),
        ]
        targets.extend((value, f"{name}_C{component_index}", priority)
                       for value, name, priority in anchors)

    for target, target_source, target_priority in targets:
        center = geometry["bottleneck_center"]
        mids = [
            (target, "MID_EQUALS_TARGET", 0.0),
            (center, "BOTTLENECK_CENTER", 1.0),
            (0.5 * (target + center), "TARGET_CENTER_HALF", 2.0),
            (0.75 * target, "REFERENCE_INWARD_25PCT", 3.0),
            (0.5 * target, "REFERENCE_INWARD_50PCT", 4.0),
        ]
        for middle, mid_source, mid_priority in mids:
            add(target, middle, target_source, mid_source,
                1.0 + target_priority + mid_priority)
    return list(factors.values())


def build_pool(event_id: str, event: dict, production: list[dict]) -> tuple[list[dict], list[dict]]:
    sides = sorted({row["side"] for row in production}, key=lambda side: side != "RIGHT")
    geometries = {side: side_geometry(event, side) for side in sides}
    transitions = sorted({(number(row, "entry_scale"), number(row, "exit_scale"))
                          for row in production})
    entries = unique_values(entry for entry, _ in transitions)
    exits = unique_values(exit for _, exit in transitions)
    production_keys = {config_key(row) for row in production}
    pool: dict[tuple, dict] = {}
    for side in sides:
        geometry = geometries[side]
        laterals = lateral_factors(event_id, side, geometry, production)
        for lateral in laterals:
            for entry, exit in transitions:
                key = (side, lateral["d_target"], lateral["d_mid"], entry, exit)
                if key in production_keys:
                    continue
                entry_norm, exit_norm = normalized(entry, entries), normalized(exit, exits)
                candidate = {
                    **lateral, "entry_scale": entry, "exit_scale": exit,
                    "entry_normalized": entry_norm, "exit_normalized": exit_norm,
                    "transition_priority": transition_priority(entry_norm, exit_norm),
                    "configuration_key": "|".join(str(value) for value in key),
                }
                desired_entry = 1.0 if (
                    geometry["available_entry_distance"] < 0.25 or
                    geometry["max_abs_kappa"] > 0.5 or
                    geometry["cluster_start"] > 2.0) else 0.0
                desired_exit = 0.0 if geometry["merge_distance"] < 2.0 else (
                    1.0 if geometry["curvature_sign_change"] or
                    geometry["track_width_variation"] > 0.6 else 0.0)
                transition_error = abs(entry_norm - desired_entry) + abs(exit_norm - desired_exit)
                candidate["desired_entry_normalized"] = desired_entry
                candidate["desired_exit_normalized"] = desired_exit
                candidate["geometry_transition_error"] = transition_error

                narrow = geometry["bottleneck_width"] < 0.25
                near, far = sorted(geometry["components"][0], key=lambda value: (abs(value), value))
                near_inset = near + (far - near) / 64.0
                three_quarter = near + 0.75 * (far - near)
                if geometry["cluster_start"] > 2.0:
                    # Ample entry distance makes a deeper side displacement physically reachable.
                    desired_target = three_quarter
                    desired_mid = three_quarter
                else:
                    desired_target = geometry["bottleneck_center"] if narrow else near_inset
                    desired_mid = geometry["bottleneck_center"] if (
                        geometry["cluster_span"] >= 0.12 or narrow) else lateral["d_target"]
                scale = max(0.1, geometry["bottleneck_width"])
                lateral_error = (
                    abs(lateral["d_target"] - desired_target) / scale +
                    abs(lateral["d_mid"] - desired_mid) / scale)
                candidate["desired_d_target_m"] = desired_target
                candidate["desired_d_mid_m"] = desired_mid
                candidate["geometry_lateral_error"] = lateral_error
                candidate["geometry"] = geometry
                pool[key] = candidate
    features = [feature_row(event_id, event, geometries[side]) for side in sides]
    return list(pool.values()), features


def selector_score(candidate: dict, selector: str) -> tuple:
    side_order = 0 if candidate["side"] == "RIGHT" else 1
    lexical = candidate["configuration_key"]
    if selector == "H3_FIXED_BOUNDED":
        return (
            candidate["lateral_priority"] + candidate["transition_priority"],
            max(candidate["lateral_priority"], candidate["transition_priority"]),
            candidate["lateral_priority"], candidate["transition_priority"],
            side_order, lexical)
    if selector == "H4A_GEOMETRY_TRANSITION":
        return (
            candidate["geometry_transition_error"] + 0.20 * candidate["lateral_priority"],
            candidate["geometry_transition_error"], candidate["lateral_priority"],
            side_order, lexical)
    if selector == "H4B_GEOMETRY_LATERAL_TRANSITION":
        return (
            candidate["geometry_lateral_error"] +
            1.5 * candidate["geometry_transition_error"] +
            0.05 * candidate["lateral_priority"],
            candidate["geometry_lateral_error"], candidate["geometry_transition_error"],
            side_order, lexical)
    raise RuntimeError(selector)


def oracle_rank(row: dict) -> tuple:
    return (
        int(row.get("exit_reaches_next_obstacle", "0")),
        number(row, "braking_deficit_m") > EPS, number(row, "braking_deficit_m"),
        number(row, "velocity_loss"), -number(row, "minimum_normalized_safety_slack"),
        number(row, "global_path_deviation_m"), int(row.get("request_index", "0")),
    )


def reference_snapshots() -> dict[str, list[str]]:
    events: dict[str, list[str]] = {}
    current = None
    for line in REFERENCE_STREAM.read_text(encoding="utf-8").splitlines():
        parts = line.split("\t")
        if parts[0] == "EVENT":
            current = parts[1]
            events[current] = []
        elif parts[0] == "W" and current is not None:
            events[current].append(line)
    return {
        "ref_3f5d487564f4ee3d": events["E03"],
        "ref_4dc0768eb37d9590": events["E01"],
    }


def write_event(path: Path, event_id: str, bag: str, evaluation: dict,
                ref_lines: list[str]) -> None:
    ego = evaluation["ego"]
    s_values = [float(line.split("\t")[2]) for line in ref_lines]
    spacings = sorted(b - a for a, b in zip(s_values, s_values[1:]))
    track_length = s_values[-1] + spacings[len(spacings) // 2]
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write("P3_ORACLE_EVENT_V1\n")
        stream.write(
            f"EVENT\t{event_id}\t{bag}\t{evaluation['callback_sequence']}\t"
            f"{evaluation['evaluation_sequence']}\tPLAN_PRIMARY\t{ego['s']:.17g}\t"
            f"{ego['d']:.17g}\t{ego['speed_mps']:.17g}\t"
            f"{evaluation['source_stamp_ns']}\t{evaluation['source_epoch']}\n")
        stream.write(f"REFERENCE\t{len(ref_lines)}\n")
        stream.write("\n".join(ref_lines) + "\n")
        obstacles = evaluation["obstacles"]
        stream.write(f"OBSTACLES\t{len(obstacles)}\n")
        for obstacle in obstacles:
            forward = (float(obstacle["s_end"]) - float(obstacle["s_start"])) % track_length
            reverse = (float(obstacle["s_start"]) - float(obstacle["s_end"])) % track_length
            span = min(forward, reverse)
            width = abs(float(obstacle["d_left"]) - float(obstacle["d_right"]))
            size = math.hypot(span, width)
            stream.write(
                "O\t{}\t{:.17g}\t{:.17g}\t{:.17g}\t{:.17g}\t{:.17g}\t"
                "{:.17g}\t0\t0\t{}\t{}\n".format(
                    obstacle["id"], obstacle["s_center"], obstacle["s_start"],
                    obstacle["s_end"], obstacle["d_right"], obstacle["d_left"], size,
                    int(bool(obstacle["is_static"])), int(bool(obstacle["is_visible"]))))
        stream.write("END_EVENT\n")


def select_success_controls(per_bag: int = 6) -> list[dict]:
    """Hash-select seen strict successes outside every frozen failure episode."""
    replay_manifest = read_csv(PREPARATION / "replay_manifest.csv")
    failure_episodes = read_csv(PREPARATION / "failure_episodes.csv")
    excluded: dict[str, list[tuple[int, int]]] = defaultdict(list)
    for row in failure_episodes:
        excluded[row["bag"]].append(
            (int(row["first_callback_sequence"]), int(row["last_callback_sequence"])))

    selected = []
    for replay in replay_manifest:
        bag, run_id = replay["bag"], replay["run_id"]
        run = LOG_ROOT / run_id
        eligible = []
        for evaluation in jsonl(run / "evaluation_events.jsonl"):
            callback = int(evaluation["callback_sequence"])
            if any(first <= callback <= last for first, last in excluded[bag]):
                continue
            if (evaluation.get("clearance_pass") != "STRICT" or
                evaluation.get("evaluation_role") != "TEST_ACTIVE_PRIMARY" or
                int(evaluation.get("hard_valid_total_actual", 0)) <= 0 or
                int(evaluation.get("returned_candidate_count", 0)) <= 0 or
                not evaluation.get("obstacles")):
                continue
            selection_hash = hashlib.sha256(
                ("p3_geometry_method_success_control_v1|" + bag + "|" +
                 str(evaluation["callback_sequence"]) + "|" +
                 str(evaluation["evaluation_sequence"])).encode()).hexdigest()
            eligible.append((selection_hash, evaluation))
        eligible.sort(key=lambda item: item[0])
        chosen = eligible[:per_bag]
        keys = {(int(item[1]["callback_sequence"]), int(item[1]["evaluation_sequence"]))
                for item in chosen}
        candidates: dict[tuple[int, int], list[dict]] = defaultdict(list)
        for candidate in jsonl(run / "candidate_events.jsonl"):
            key = (int(candidate["callback_sequence"]), int(candidate["evaluation_sequence"]))
            if key in keys and candidate.get("clearance_pass") == "STRICT":
                candidates[key].append(candidate)
        for selection_hash, evaluation in chosen:
            key = (int(evaluation["callback_sequence"]), int(evaluation["evaluation_sequence"]))
            event_candidates = candidates[key]
            if not event_candidates:
                raise RuntimeError(f"missing success-control candidates: {bag} {key}")
            expected_ids = tuple(evaluation[name] for name in (
                "input_snapshot_id", "ego_snapshot_id", "obstacle_snapshot_id",
                "reference_snapshot_id"))
            if any(tuple(candidate[name] for name in (
                "input_snapshot_id", "ego_snapshot_id", "obstacle_snapshot_id",
                "reference_snapshot_id")) != expected_ids for candidate in event_candidates):
                raise RuntimeError(f"success-control lineage mismatch: {bag} {key}")
            selected.append({
                "bag": bag, "run_id": run_id, "selection_hash": selection_hash,
                "evaluation": evaluation, "candidates": event_candidates,
                "frozen_failure_episode_exclusion": True,
            })
    selected.sort(key=lambda item: (item["bag"], item["selection_hash"]))
    for index, item in enumerate(selected, 1):
        item["control_event_id"] = f"SCE{index:03d}"
    return selected


def evaluate_success_controls(geometry_rows: list[dict]) -> tuple[list[dict], list[dict]]:
    refs = reference_snapshots()
    input_dir = HERE / "success_control_inputs"
    input_dir.mkdir(parents=True, exist_ok=True)
    controls = select_success_controls()
    manifest_rows = []
    regression_rows = []
    for item in controls:
        event_id, evaluation = item["control_event_id"], item["evaluation"]
        event_path = input_dir / f"{event_id}.event"
        write_event(event_path, event_id, item["bag"], evaluation,
                    refs[evaluation["reference_snapshot_id"]])
        event = G.parse_event(event_path)
        production = item["candidates"]

        # Empty direct-request file reconstructs the frozen production candidates for parity.
        _, baseline_runtime = run_harness(event_id, event_path, "SUCCESS_BASELINE_PARITY", 0, [])
        baseline_output = WORK / f"{event_id}.SUCCESS_BASELINE_PARITY.K0.out"
        reconstructed_all = records(baseline_output, "BASELINE_CONSTRUCTED_CANDIDATE")
        reconstructed_returned = records(baseline_output, "BASELINE_CANDIDATE")
        constructed_parity = Counter(row["path_digest"] for row in reconstructed_all) == Counter(
            row["path_digest"] for row in production)
        returned_parity = Counter(row["path_digest"] for row in reconstructed_returned) == Counter(
            row["path_digest"] for row in production if row["returned_by_policy"])
        if not constructed_parity or not returned_parity:
            raise RuntimeError(f"{event_id}: success-control production digest parity failed")
        production_valid = [row for row in production if bool(row["hard_valid"])]
        production_selected = next((row for row in production if bool(row["selected"])), None)
        if production_selected is None:
            production_selected = min(
                production_valid, key=lambda row: int(row["final_rank"])
                if int(row["final_rank"]) >= 0 else 10**9)
        production_side_availability = "|".join(sorted({row["side"] for row in production_valid}))
        production_slack = float(
            production_selected["rank_tuple"]["minimum_normalized_safety_slack"])
        production_footprint = float(production_selected["margins"]["footprint_track_m"])
        production_obstacle = float(production_selected["margins"]["obstacle_m"])

        pool, features = build_pool(event_id, event, production)
        for feature in features:
            feature["feature_scope"] = "PILOT_SEEN_DEVELOPMENT_SUCCESS_CONTROL_SIDE"
            feature["success_control_selection_hash"] = item["selection_hash"]
        geometry_rows.extend(features)
        manifest_rows.append({
            "control_event_id": event_id,
            "dataset_role": "PILOT_SEEN_DEVELOPMENT_DATA_SUCCESS_CONTROL_AUXILIARY",
            "official_failure_split_membership": False,
            "bag": item["bag"], "callback_sequence": evaluation["callback_sequence"],
            "evaluation_sequence": evaluation["evaluation_sequence"],
            "input_snapshot_id": evaluation["input_snapshot_id"],
            "ego_snapshot_id": evaluation["ego_snapshot_id"],
            "obstacle_snapshot_id": evaluation["obstacle_snapshot_id"],
            "reference_snapshot_id": evaluation["reference_snapshot_id"],
            "selection_hash": item["selection_hash"],
            "selection_rule": "lowest six SHA256 per bag among strict production successes",
            "outside_every_frozen_failure_episode": True,
            "exact_candidate_lineage": True,
            "constructed_digest_multiset_parity": constructed_parity,
            "returned_digest_multiset_parity": returned_parity,
            "production_constructed_count": len(production),
            "production_hard_valid_count": len(production_valid),
            "production_selected_side": production_selected["side"],
            "baseline_parity_runtime_s": baseline_runtime,
            "event_input_sha256": sha256(event_path),
        })
        for selector in SELECTORS:
            ordered = sorted(pool, key=lambda candidate: selector_score(candidate, selector))
            for budget in BUDGETS:
                selected = ordered[:budget]
                rows, runtime = run_harness(event_id, event_path, selector, budget, selected)
                digest_first = {}
                effective = []
                for row in rows:
                    digest = row.get("path_digest", "")
                    if digest and digest not in digest_first:
                        digest_first[digest] = len(effective)
                        effective.append(row)
                hard_valid = [row for row in effective if row["hard_valid"] == "1"]
                best = min(hard_valid, key=oracle_rank) if hard_valid else None
                proposed_sides = "|".join(sorted({row["side"] for row in hard_valid}))
                regression_rows.append({
                    "control_event_id": event_id, "selector": selector, "budget_k": budget,
                    "dataset_role": "PILOT_SEEN_DEVELOPMENT_DATA_SUCCESS_CONTROL_AUXILIARY",
                    "production_digest_parity": constructed_parity and returned_parity,
                    "production_hard_valid_count": len(production_valid),
                    "production_hard_valid_side_availability": production_side_availability,
                    "production_selected_side": production_selected["side"],
                    "production_selected_minimum_normalized_safety_slack": production_slack,
                    "production_selected_footprint_track_margin_m": production_footprint,
                    "production_selected_obstacle_margin_m": production_obstacle,
                    "proposed_selected_exact_tuple_count": len(selected),
                    "proposed_construction_guard_reject_count":
                        len(rows) - sum(bool(row.get("path_digest", "")) for row in rows),
                    "proposed_duplicate_path_count":
                        sum(bool(row.get("path_digest", "")) for row in rows) - len(effective),
                    "proposed_deduplicated_validator_calls": len(effective),
                    "proposed_hard_valid_count": len(hard_valid),
                    "proposed_generates_hard_valid": bool(hard_valid),
                    "proposed_hard_valid_side_availability": proposed_sides,
                    "production_selected_side_available_in_proposed":
                        production_selected["side"] in {row["side"] for row in hard_valid},
                    "proposed_best_minimum_normalized_safety_slack":
                        best["minimum_normalized_safety_slack"] if best else "",
                    "proposed_minus_production_normalized_safety_slack":
                        number(best, "minimum_normalized_safety_slack") - production_slack
                        if best else "",
                    "proposed_best_footprint_track_margin_m":
                        best["footprint_track_margin_m"] if best else "",
                    "proposed_minus_production_footprint_margin_m":
                        number(best, "footprint_track_margin_m") - production_footprint if best else "",
                    "proposed_best_obstacle_margin_m": best["obstacle_margin_m"] if best else "",
                    "proposed_minus_production_obstacle_margin_m":
                        number(best, "obstacle_margin_m") - production_obstacle if best else "",
                    "offline_audit_harness_wall_runtime_s": runtime,
                    "deployment_fallback_behavior":
                        "PRODUCTION_FIRST_SKIP_PROPOSED_STAGE_WHEN_PRODUCTION_HARD_VALID",
                    "deployment_additional_validator_calls_on_success": 0,
                    "deployment_selected_path_changed": False,
                })
        print("SUCCESS_CONTROL", event_id, flush=True)
    return manifest_rows, regression_rows


def main() -> None:
    HERE.mkdir(parents=True, exist_ok=True)
    PLOTS.mkdir(parents=True, exist_ok=True)
    WORK.mkdir(parents=True, exist_ok=True)
    if not HARNESS.is_file() or sha256(HARNESS) != EXPECTED_HARNESS_SHA256:
        raise RuntimeError("frozen exact-validator harness missing or hash mismatch")

    production = read_csv(RAW / "production_candidates.csv")
    summary = read_csv(CORPUS / "development_oracle_summary.csv")
    taxonomy = read_csv(CORPUS / "mapping_failure_taxonomy.csv")
    production_by_event: dict[str, list[dict]] = defaultdict(list)
    for row in production:
        production_by_event[row["event_id"]].append(row)
    event_ids = sorted(production_by_event)
    if event_ids != [f"DVE{index:03d}" for index in range(1, 41)]:
        raise RuntimeError("expected exact DVE001..DVE040 DEVELOPMENT corpus")
    oracle_valid_events = {
        row["development_event_id"] for row in summary
        if row["oracle_classification"] == "ORACLE_VALID_P3_EXISTS"
    }
    mechanism = {row["development_event_id"]: row["mechanism_class"] for row in taxonomy}

    geometry_rows: list[dict] = []
    result_rows: list[dict] = []
    selected_candidate_rows: list[dict] = []
    for event_id in event_ids:
        event_path = RAW / "inputs" / f"{event_id}.event"
        event = G.parse_event(event_path)
        pool, features = build_pool(event_id, event, production_by_event[event_id])
        geometry_rows.extend(features)
        for selector in SELECTORS:
            ordered = sorted(pool, key=lambda candidate: selector_score(candidate, selector))
            for budget in BUDGETS:
                selected = ordered[:budget]
                rows, runtime = run_harness(event_id, event_path, selector, budget, selected)
                if len(rows) != len(selected):
                    raise RuntimeError(f"{event_id}/{selector}/K{budget}: result count mismatch")
                digest_first: dict[str, int] = {}
                effective = []
                for index, row in enumerate(rows):
                    digest = row.get("path_digest", "")
                    if digest and digest not in digest_first:
                        digest_first[digest] = index
                        effective.append(row)
                constructed_count = sum(bool(row.get("path_digest", "")) for row in rows)
                construction_reject_count = len(rows) - constructed_count
                duplicate_path_count = constructed_count - len(effective)
                hard_valid = [row for row in effective if row["hard_valid"] == "1"]
                best = min(hard_valid, key=oracle_rank) if hard_valid else None
                result_rows.append({
                    "development_event_id": event_id, "selector": selector, "budget_k": budget,
                    "oracle_valid_in_full_development_oracle": event_id in oracle_valid_events,
                    "failure_mechanism": mechanism.get(
                        event_id, "NOT_APPLICABLE_NO_ORACLE_VALID"),
                    "raw_prevalidation_pool_count": len(pool),
                    "selected_exact_tuple_count": len(selected),
                    "constructed_path_count": constructed_count,
                    "construction_guard_reject_count": construction_reject_count,
                    "raw_harness_validator_execution_count": sum(
                        row["validator_executed"] == "1" for row in rows),
                    "duplicate_path_count": duplicate_path_count,
                    "deduplicated_validator_call_count": len(effective),
                    "hard_valid_candidate_count": len(hard_valid),
                    "hard_valid_side_availability": "|".join(sorted({row["side"] for row in hard_valid})),
                    "recovered": bool(hard_valid),
                    "best_side": best["side"] if best else "",
                    "best_d_target": best["d_target"] if best else "",
                    "best_d_mid": best["d_mid"] if best else "",
                    "best_entry_scale": best["entry_scale"] if best else "",
                    "best_exit_scale": best["exit_scale"] if best else "",
                    "best_minimum_normalized_safety_slack":
                        best["minimum_normalized_safety_slack"] if best else "",
                    "best_footprint_track_margin_m": best["footprint_track_margin_m"] if best else "",
                    "best_obstacle_margin_m": best["obstacle_margin_m"] if best else "",
                    "raw_harness_wall_runtime_s": runtime,
                    "runtime_contract":
                        "raw subprocess wall; harness validates before offline digest dedup",
                })
                for index, (candidate, row) in enumerate(zip(selected, rows)):
                    selected_candidate_rows.append({
                        "development_event_id": event_id, "selector": selector,
                        "budget_k": budget, "selection_rank": index + 1,
                        "side": candidate["side"], "d_target": candidate["d_target"],
                        "d_mid": candidate["d_mid"], "target_source": candidate["target_source"],
                        "mid_source": candidate["mid_source"],
                        "entry_scale": candidate["entry_scale"],
                        "exit_scale": candidate["exit_scale"],
                        "desired_d_target_m": candidate["desired_d_target_m"],
                        "desired_d_mid_m": candidate["desired_d_mid_m"],
                        "desired_entry_normalized": candidate["desired_entry_normalized"],
                        "desired_exit_normalized": candidate["desired_exit_normalized"],
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

    success_manifest_rows, success_regression_rows = evaluate_success_controls(geometry_rows)
    write_csv(HERE / "success_control_manifest.csv", success_manifest_rows)
    write_csv(HERE / "success_control_regression.csv", success_regression_rows)
    write_csv(HERE / "geometry_features.csv", geometry_rows)
    write_csv(HERE / "development_results.csv", result_rows)
    write_csv(HERE / "selected_candidates_audit.csv", selected_candidate_rows)

    valid_results = [row for row in result_rows
                     if truth(row["oracle_valid_in_full_development_oracle"])]
    budget_rows = []
    mechanism_rows = []
    for selector in SELECTORS:
        for budget in BUDGETS:
            rows = [row for row in valid_results
                    if row["selector"] == selector and int(row["budget_k"]) == budget]
            recovered = [row for row in rows if truth(row["recovered"])]
            runtimes = [float(row["raw_harness_wall_runtime_s"]) for row in rows]
            budget_rows.append({
                "selector": selector, "budget_k": budget,
                "oracle_valid_episode_count": len(rows),
                "recovered_episode_count": len(recovered),
                "recovery_rate": len(recovered) / len(rows),
                "selected_candidate_count_total": sum(
                    int(row["selected_exact_tuple_count"]) for row in rows),
                "selected_candidates_per_episode_p50": percentile(
                    [int(row["selected_exact_tuple_count"]) for row in rows], 50),
                "selected_candidates_per_episode_p95": percentile(
                    [int(row["selected_exact_tuple_count"]) for row in rows], 95),
                "selected_candidates_per_episode_max": max(
                    int(row["selected_exact_tuple_count"]) for row in rows),
                "raw_harness_validator_executions_total": sum(
                    int(row["raw_harness_validator_execution_count"]) for row in rows),
                "construction_guard_reject_count_total": sum(
                    int(row["construction_guard_reject_count"]) for row in rows),
                "deduplicated_validator_calls_total": sum(
                    int(row["deduplicated_validator_call_count"]) for row in rows),
                "duplicate_path_count_total": sum(int(row["duplicate_path_count"]) for row in rows),
                "hard_valid_candidate_count_total": sum(
                    int(row["hard_valid_candidate_count"]) for row in rows),
                "runtime_wall_p50_s": percentile(runtimes, 50),
                "runtime_wall_p95_s": percentile(runtimes, 95),
                "runtime_wall_p99_s": percentile(runtimes, 99),
                "runtime_wall_max_s": max(runtimes),
                "runtime_interpretation":
                    "raw exact-harness subprocess wall, not production callback latency",
            })
            for mechanism_name in [
                "D_PROBE_DOMINANT", "S_PROBE_DOMINANT",
                "TEMPLATE_OR_TRANSITION_SELECTION", "OTHER",
            ]:
                subset = [row for row in rows if row["failure_mechanism"] == mechanism_name]
                subset_recovered = [row for row in subset if truth(row["recovered"])]
                mechanism_rows.append({
                    "selector": selector, "budget_k": budget,
                    "failure_mechanism": mechanism_name,
                    "episode_count": len(subset),
                    "recovered_episode_count": len(subset_recovered),
                    "recovery_rate": len(subset_recovered) / len(subset) if subset else math.nan,
                    "recovered_event_ids": "|".join(
                        sorted(row["development_event_id"] for row in subset_recovered)),
                    "unrecovered_event_ids": "|".join(sorted(
                        row["development_event_id"] for row in subset
                        if not truth(row["recovered"]))),
                })
    write_csv(HERE / "recovery_by_budget.csv", budget_rows)
    write_csv(HERE / "recovery_by_failure_mechanism.csv", mechanism_rows)
    write_csv(HERE / "candidate_budget_runtime.csv", budget_rows)

    # Cross-study comparison.  Oracle-informed rows remain explicitly non-online.
    comparison_rows = []
    prior_ablation = read_csv(FACTOR / "factorization_ablation.csv")
    prior_variants = [
        ("PRODUCTION", "BASELINE_PRODUCTION", "ONLINE_PRODUCTION_BASELINE"),
        ("SIMPLE_UNCOUPLE_D_MID", "A_UNCOUPLE_D_MID_DEV_COUNTERFACTUAL",
         "DEVELOPMENT_ORACLE_INFORMED_UPPER_BOUND"),
        ("SIMPLE_TRANSITION_CROSS_PAIR", "B_CROSS_PAIR_TRANSITIONS",
         "ONLINE_FACTOR_VALUES_BUT_EXHAUSTIVE_NOT_BOUNDED"),
        ("SIMPLE_A_PLUS_B", "AB_DEV_COUNTERFACTUAL",
         "DEVELOPMENT_ORACLE_INFORMED_UPPER_BOUND"),
        ("FULL_LxT", "FULL_LxT_DEV_UPPER_BOUND",
         "DEVELOPMENT_ORACLE_INFORMED_UPPER_BOUND"),
    ]
    for label, variant, interpretation in prior_variants:
        rows = [row for row in prior_ablation
                if row["variant"] == variant and row["development_event_id"] in oracle_valid_events]
        recovered = [row for row in rows if truth(row["recovered"])]
        candidates = [int(row["unique_combination_count"]) for row in rows]
        comparison_rows.append({
            "method": label, "source_variant": variant, "budget_k": "UNBOUNDED_OR_PRODUCTION",
            "online_selectable": interpretation.startswith("ONLINE_"),
            "interpretation": interpretation,
            "oracle_valid_episode_count": len(rows),
            "recovered_episode_count": len(recovered),
            "recovery_rate": len(recovered) / len(rows),
            "candidate_count_total": sum(candidates),
            "candidates_per_episode_p50": percentile(candidates, 50),
            "candidates_per_episode_p95": percentile(candidates, 95),
            "candidates_per_episode_max": max(candidates),
            "validator_calls_total": sum(int(row["validator_execution_count"]) for row in rows),
            "duplicate_path_count_total": sum(int(row["duplicate_path_count"]) for row in rows),
            "hard_valid_candidate_count_total": sum(
                int(row["hard_valid_candidate_count"]) for row in rows),
        })
    for selector in SELECTORS:
        for budget in BUDGETS:
            row = next(row for row in budget_rows
                       if row["selector"] == selector and int(row["budget_k"]) == budget)
            comparison_rows.append({
                "method": selector, "source_variant": selector, "budget_k": budget,
                "online_selectable": True,
                "interpretation": "DETERMINISTIC_PRE_PLANNING_GEOMETRY_OR_FIXED_RULE",
                "oracle_valid_episode_count": row["oracle_valid_episode_count"],
                "recovered_episode_count": row["recovered_episode_count"],
                "recovery_rate": row["recovery_rate"],
                "candidate_count_total": row["selected_candidate_count_total"],
                "candidates_per_episode_p50": row["selected_candidates_per_episode_p50"],
                "candidates_per_episode_p95": row["selected_candidates_per_episode_p95"],
                "candidates_per_episode_max": row["selected_candidates_per_episode_max"],
                "validator_calls_total": row["deduplicated_validator_calls_total"],
                "duplicate_path_count_total": row["duplicate_path_count_total"],
                "hard_valid_candidate_count_total": row["hard_valid_candidate_count_total"],
            })
    write_csv(HERE / "comparison_summary.csv", comparison_rows)

    success_summary_rows = []
    for selector in SELECTORS:
        for budget in BUDGETS:
            rows = [row for row in success_regression_rows
                    if row["selector"] == selector and int(row["budget_k"]) == budget]
            hard_valid_rows = [row for row in rows if truth(row["proposed_generates_hard_valid"])]
            side_rows = [row for row in rows
                         if truth(row["production_selected_side_available_in_proposed"])]
            slack_deltas = [float(row["proposed_minus_production_normalized_safety_slack"])
                            for row in hard_valid_rows]
            footprint_deltas = [float(row["proposed_minus_production_footprint_margin_m"])
                                for row in hard_valid_rows]
            obstacle_deltas = [float(row["proposed_minus_production_obstacle_margin_m"])
                               for row in hard_valid_rows]
            success_summary_rows.append({
                "selector": selector, "budget_k": budget,
                "success_control_count": len(rows),
                "generates_hard_valid_count": len(hard_valid_rows),
                "production_selected_side_available_count": len(side_rows),
                "offline_deduplicated_validator_calls_total": sum(
                    int(row["proposed_deduplicated_validator_calls"]) for row in rows),
                "offline_construction_guard_reject_count_total": sum(
                    int(row["proposed_construction_guard_reject_count"]) for row in rows),
                "offline_duplicate_path_count_total": sum(
                    int(row["proposed_duplicate_path_count"]) for row in rows),
                "normalized_safety_slack_delta_p50": percentile(slack_deltas, 50),
                "normalized_safety_slack_delta_min": min(slack_deltas) if slack_deltas else math.nan,
                "footprint_margin_delta_p50_m": percentile(footprint_deltas, 50),
                "footprint_margin_delta_min_m": min(footprint_deltas) if footprint_deltas else math.nan,
                "obstacle_margin_delta_p50_m": percentile(obstacle_deltas, 50),
                "obstacle_margin_delta_min_m": min(obstacle_deltas) if obstacle_deltas else math.nan,
                "deployment_additional_validator_calls_total": 0,
                "deployment_selected_path_change_count": 0,
                "deployment_contract": "PRODUCTION_FIRST_SKIP_STAGE_ON_PRODUCTION_SUCCESS",
            })
    write_csv(HERE / "success_control_summary.csv", success_summary_rows)

    # Plots are descriptive DEVELOPMENT results only.
    figure, axis = plt.subplots(figsize=(8.5, 5.2))
    for selector in SELECTORS:
        rows = [row for row in budget_rows if row["selector"] == selector]
        axis.plot([int(row["budget_k"]) for row in rows],
                  [int(row["recovered_episode_count"]) for row in rows],
                  marker="o", label=selector)
    axis.axhline(23, linestyle="--", color="#555", linewidth=1, label="FULL upper bound 23")
    axis.set(xticks=BUDGETS, xlabel="proposed-stage candidate budget K",
             ylabel="recovered oracle-valid DEVELOPMENT episodes",
             title="Bounded factorized selector recovery")
    axis.grid(alpha=.2); axis.legend(fontsize=8); figure.tight_layout()
    figure.savefig(PLOTS / "recovery_by_budget.png", dpi=170); plt.close(figure)

    figure, axis = plt.subplots(figsize=(8.5, 5.2))
    k24 = [row for row in mechanism_rows if int(row["budget_k"]) == 24]
    mechanisms = ["D_PROBE_DOMINANT", "S_PROBE_DOMINANT",
                  "TEMPLATE_OR_TRANSITION_SELECTION", "OTHER"]
    x = np.arange(len(mechanisms)); width = 0.24
    for index, selector in enumerate(SELECTORS):
        values = [next(int(row["recovered_episode_count"]) for row in k24
                       if row["selector"] == selector and row["failure_mechanism"] == mechanism_name)
                  for mechanism_name in mechanisms]
        axis.bar(x + (index - 1) * width, values, width, label=selector)
    axis.set_xticks(x, ["D_PROBE", "S_PROBE", "TEMPLATE/T", "OTHER"])
    axis.set(ylabel="recovered episodes", title="K=24 recovery by failure mechanism")
    axis.grid(axis="y", alpha=.2); axis.legend(fontsize=7); figure.tight_layout()
    figure.savefig(PLOTS / "recovery_by_mechanism_k24.png", dpi=170); plt.close(figure)

    figure, axis = plt.subplots(figsize=(8.5, 5.2))
    for selector in SELECTORS:
        rows = [row for row in success_summary_rows if row["selector"] == selector]
        axis.plot([int(row["budget_k"]) for row in rows],
                  [int(row["generates_hard_valid_count"]) for row in rows],
                  marker="o", label=selector)
    axis.axhline(len(success_manifest_rows), linestyle="--", color="#555", linewidth=1,
                 label=f"controls {len(success_manifest_rows)}")
    axis.set(xticks=BUDGETS, xlabel="candidate budget K",
             ylabel="success controls with >=1 hard-valid proposed candidate",
             title="Auxiliary success-control standalone availability")
    axis.grid(alpha=.2); axis.legend(fontsize=8); figure.tight_layout()
    figure.savefig(PLOTS / "success_control_availability.png", dpi=170); plt.close(figure)

    manifest = {
        "study": "P3_GEOMETRY_CONDITIONED_METHOD_V1",
        "dataset_scope": "DVE001..DVE040 DEVELOPMENT only",
        "validation_or_holdout_rows_read": 0,
        "production_source_modified": False,
        "split_modified": False,
        "harness_sha256": sha256(HARNESS),
        "budgets": BUDGETS, "selectors": SELECTORS,
        "online_pool_contract": (
            "production lateral/target values plus deterministic corridor anchors and "
            "midpoint recipes crossed only with same-event production transition factors"),
        "oracle_outcome_online_feature_count": 0,
        "success_control_contract": (
            "hash-selected seen strict production successes outside every frozen failure "
            "episode; official split unchanged"),
        "success_control_count": len(success_manifest_rows),
        "success_control_production_digest_parity_pass_count": sum(
            truth(row["constructed_digest_multiset_parity"]) and
            truth(row["returned_digest_multiset_parity"])
            for row in success_manifest_rows),
        "validation_or_holdout_manifest_rows_loaded": 0,
        "frozen_failure_episode_callbacks_retained_for_success_control": 0,
    }
    (HERE / "execution_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
