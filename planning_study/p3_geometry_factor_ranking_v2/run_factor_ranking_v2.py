#!/usr/bin/env python3
"""Seen-only geometry-conditioned factor ranking study for bounded direct-P3 fallback.

This is research instrumentation, not production planner code.  It reads only the
three already-materialized seen datasets, generates factors from online-available
geometry, ranks before construction, and delegates reconstruction/validation to the
frozen exact C++ harness.  Oracle coordinates and outcomes are joined only after
selection to define evaluation denominators and descriptive miss labels.
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
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/p3_geometry_factor_ranking_v2_mpl")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
REFERENCE = REPO / "planning_study/p3_reference_oracle_v2"
METHOD_V1 = REPO / "planning_study/p3_geometry_conditioned_method_v1"
METHOD_IMPL = METHOD_V1 / "analyze_method.py"
REFERENCE_IMPL = REFERENCE / "run_reference_oracle_v2.py"
HARNESS = Path("/tmp/p3_oracle_v2_build/p3_family_oracle_harness")
WORK = Path("/tmp/p3_geometry_factor_ranking_v2")
PLOTS = HERE / "plots"

EXPECTED = {
    "oracle_v2_spec": "62b63b8ab05d5a73565398141bd05a9db4a102541855fb2e3634d85b18a6bffe",
    "evaluation_contract": "226b1b44a9bea6adf26715658f36ae8e7b2c322e030f363270728f9e044b1dad",
    "split_manifest": "c57cfe8e57dfca4bb318d30047e8f7215f6994e88a39babfbdbb10ce7637b2f4",
    "harness": "8b23f2b6df54c2e93794ee087f3ebd7a14f9921544af4f97eef755535e045e7e",
}

DATASET_ROLES = ["PILOT_SEEN_DEVELOPMENT_DATA", "DEVELOPMENT", "VALIDATION_SEEN_AFTER_V1"]
BUDGETS = [8, 12, 16, 24]
HYPOTHESES = [
    "R1_FEASIBILITY_LEXICOGRAPHIC",
    "R2_BALANCED_PHYSICS_SCORE",
    "R3_LEXICOGRAPHIC_COVERAGE_RESERVE",
]
EPS = 1.0e-9

# Frozen production values used by stationsFor().
PRE_APEX_FAR_M = 11.442220427651225
POST_APEX_FAR_M = 6.178529850015357
LOOKAHEAD_M = 15.0
OUTSIDE_EXIT_MULTIPLIER = 0.4060036444074003
MAXIMUM_LATERAL_SLOPE = 0.8
MAXIMUM_EXIT_LENGTH_M = 0.0
TARGET_BOUND_M = 1.5

COMPONENT_FRACTIONS = [
    0.0, 1.0 / 64.0, 1.0 / 32.0, 1.0 / 16.0, 1.0 / 8.0,
    3.0 / 16.0, 1.0 / 4.0, 3.0 / 8.0, 1.0 / 2.0,
    5.0 / 8.0, 2.0 / 3.0, 3.0 / 4.0, 7.0 / 8.0,
    15.0 / 16.0, 63.0 / 64.0, 1.0,
]
TARGET_LOCAL_OFFSETS_M = [0.01, 0.02, 0.04, 0.06, 0.08, 0.10]
MID_ABSOLUTE_OFFSETS_M = [0.01, 0.02, 0.04, 0.05, 0.08, 0.10, 0.15, 0.20]
CENTER_INTERPOLATION_FRACTIONS = [0.125, 0.25, 0.375, 0.4375, 0.5, 0.75, 1.0]
REFERENCE_INWARD_FACTORS = [0.25, 0.5, 0.75]


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


M = load_module("factor_v2_method_v1", METHOD_IMPL)
R = load_module("factor_v2_reference", REFERENCE_IMPL)
G = M.G


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


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
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def number(row: dict | None, key: str, default: float = math.nan) -> float:
    if row is None:
        return default
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def truth(value) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def usable(row: dict) -> bool:
    return (row.get("hard_valid") == "1"
            and row.get("exit_reaches_next_obstacle") == "0"
            and number(row, "braking_deficit_m", math.inf) <= EPS)


def percentile(values: list[float], q: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    position = (len(ordered) - 1) * q / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    ratio = position - lower
    return ordered[lower] + ratio * (ordered[upper] - ordered[lower])


def canonical(value: float) -> str:
    return float(value).hex()


def config_key(row: dict) -> tuple:
    return (row["side"], canonical(row["d_target"]), canonical(row["d_mid"]),
            canonical(row["entry_scale"]), canonical(row["exit_scale"]))


def unique_exact(values) -> list[float]:
    return sorted({float(value) for value in values if math.isfinite(float(value))})


def clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))


def freeze_gate() -> dict:
    observed = {
        "oracle_v2_spec": sha256(REFERENCE / "oracle_v2_spec.json"),
        "evaluation_contract": sha256(REFERENCE / "evaluation_contract.json"),
        # Hash only.  This script never parses the split manifest or addresses holdout rows.
        "split_manifest": sha256(REPO / "planning_study/p3_mapping_research_corpus_v1/dataset_split_manifest.csv"),
        "harness": sha256(HARNESS),
    }
    mismatch = {key: {"expected": EXPECTED[key], "observed": value}
                for key, value in observed.items() if value != EXPECTED[key]}
    if mismatch:
        raise RuntimeError("FROZEN_AUTHORITY_MISMATCH " + json.dumps(mismatch, sort_keys=True))
    return observed


def production_transition_pairs(production: list[dict]) -> list[tuple[float, float]]:
    return sorted({(float(row["entry_scale"]), float(row["exit_scale"]))
                   for row in production})


def strict_contexts(event_path: Path) -> list[dict]:
    return [row for row in R.query_contexts(event_path) if row["gate"] == "STRICT"]


def outside_is_left(event: dict, cluster_start: float, cluster_end: float) -> bool:
    obstacle_s = G.wrap(event, event["ego_s"] + 0.5 * (cluster_start + cluster_end))
    reference = event["reference"]
    nearest = G.nearest_ref(event, obstacle_s)
    obstacle_index = next(index for index, waypoint in enumerate(reference)
                          if waypoint is nearest)
    curvature_sum = 0.0
    for offset in range(len(reference)):
        index = (obstacle_index + offset) % len(reference)
        forward = G.forward(event, reference[obstacle_index]["s"], reference[index]["s"])
        if forward > 2.0:
            break
        curvature_sum += reference[index]["kappa"]
    return curvature_sum < 0.0


def stations_for(event: dict, context: dict, side: str, entry: float,
                 exit_scale: float, target: float) -> tuple[float, ...]:
    cluster_start = float(context["cluster_start"])
    cluster_end = float(context["cluster_end"])
    outside = outside_is_left(event, cluster_start, cluster_end)
    multiplier = OUTSIDE_EXIT_MULTIPLIER if ((side == "LEFT") == outside) else 1.0
    combined = exit_scale * multiplier
    if MAXIMUM_EXIT_LENGTH_M > 0.0:
        combined = min(combined, MAXIMUM_EXIT_LENGTH_M / POST_APEX_FAR_M)
    exit_length = POST_APEX_FAR_M * combined
    required = abs(target - event["ego_d"]) / MAXIMUM_LATERAL_SLOPE
    spacing = statistics.median(
        event["reference"][i]["s"] - event["reference"][i - 1]["s"]
        for i in range(1, len(event["reference"])))
    apex = cluster_start
    start = apex - apex * PRE_APEX_FAR_M * entry / LOOKAHEAD_M
    if not apex >= required:
        apex = max(required, spacing)
        start = 0.0
    return (start, apex, 0.5 * (apex + cluster_end), cluster_end,
            cluster_end + exit_length)


def segment_value_derivatives(segment: tuple, station: float) -> tuple[float, float, float]:
    start, end, coefficients = segment
    h = end - start
    t = clamp((station - start) / h, 0.0, 1.0)
    c = coefficients
    value = sum(c[index] * t ** index for index in range(6))
    d1 = sum(index * c[index] * t ** (index - 1) for index in range(1, 6)) / h
    d2 = sum(index * (index - 1) * c[index] * t ** (index - 2)
             for index in range(2, 6)) / (h * h)
    return value, d1, d2


def profile_value_derivatives(profile: list[tuple], ego_d: float,
                              station: float) -> tuple[float, float, float]:
    if station <= profile[0][0]:
        return ego_d, 0.0, 0.0
    for segment in profile:
        if station <= segment[1]:
            return segment_value_derivatives(segment, station)
    return 0.0, 0.0, 0.0


def geometry_for_side(event: dict, context: dict) -> dict:
    side = context["side"]
    left = side == "LEFT"
    visible = G.expanded_visible(event)
    cluster = G.nearest_cluster(visible)
    start = float(context["cluster_start"])
    end = float(context["cluster_end"])
    domain_lower, domain_upper = sorted(
        (float(context["minimum_target"]), float(context["maximum_target"])))
    corridor = G.make_corridor(event, left, outside_is_left(event, start, end))
    components = G.connected_ranges(corridor, domain_lower, domain_upper)
    span = G.span_samples(corridor)
    bottleneck = min(span, key=lambda row: (row["width"], row["station"]))
    midpoint = G.sample_at(event, visible, 0.5 * (start + end), left)

    cluster_ids = {row["id"] for row in cluster}
    later = [row for row in visible if row["id"] not in cluster_ids
             and row["start"] > end + EPS]
    next_obstacle_start = min((row["start"] for row in later), default=math.inf)

    # Common preconstruction samples.  Pair-specific knot and segment samples are added later.
    horizon = 32.0
    stations = {0.0, max(0.0, start), max(0.0, end), max(0.0, 0.5 * (start + end))}
    reference_stations = []
    for station, _ in G.local_reference_points(event, horizon):
        stations.add(station)
        reference_stations.append(station)
    for obstacle in visible:
        for value in (obstacle["start"], obstacle["center"], obstacle["end"]):
            if -EPS <= value <= horizon + EPS:
                stations.add(max(0.0, value))
    samples = [G.sample_at(event, visible, station, left) for station in sorted(stations)]
    center_samples = [row for row in samples if start - EPS <= row["station"] <= end + EPS]
    center_values = unique_exact(
        [bottleneck["center"], midpoint["center"]] +
        [row["center"] for row in center_samples if math.isfinite(row["center"])])
    return {
        "side": side,
        "left": left,
        "visible": visible,
        "cluster": cluster,
        "cluster_start": start,
        "cluster_end": end,
        "domain_lower": domain_lower,
        "domain_upper": domain_upper,
        "components": components,
        "bottleneck_center": bottleneck["center"],
        "bottleneck_width": bottleneck["width"],
        "bottleneck_station": bottleneck["station"],
        "midpoint_center": midpoint["center"],
        "next_obstacle_start": next_obstacle_start,
        "samples": samples,
        "reference_stations": reference_stations,
        "center_values": center_values,
    }


def lateral_factors(event_id: str, event: dict, geometry: dict,
                    production: list[dict]) -> list[dict]:
    side = geometry["side"]
    lower, upper = geometry["domain_lower"], geometry["domain_upper"]
    side_production = [row for row in production if row["side"] == side]
    production_targets = unique_exact(float(row["d_target"]) for row in side_production)
    production_pairs = {(float(row["d_target"]), float(row["d_mid"]))
                        for row in side_production}
    target_candidates: dict[str, tuple[float, str, int]] = {}

    def add_target(value: float, source: str, priority: int) -> None:
        value = clamp(value, lower, upper)
        key = canonical(value)
        previous = target_candidates.get(key)
        if previous is None or (priority, source) < (previous[2], previous[1]):
            target_candidates[key] = (value, source, priority)

    for value in production_targets:
        add_target(value, "PRODUCTION_TARGET", 0)
        for offset in TARGET_LOCAL_OFFSETS_M:
            add_target(value - offset, f"PRODUCTION_TARGET_MINUS_{offset:.2f}M", 2)
            add_target(value + offset, f"PRODUCTION_TARGET_PLUS_{offset:.2f}M", 2)
    for component_index, (component_lower, component_upper) in enumerate(geometry["components"]):
        near, far = sorted((component_lower, component_upper), key=lambda value: (abs(value), value))
        for fraction in COMPONENT_FRACTIONS:
            add_target(near + fraction * (far - near),
                       f"COMPONENT_C{component_index}_NEAR_TO_FAR_F{fraction:.8f}", 1)
    for index, value in enumerate(geometry["center_values"]):
        add_target(value, f"CORRIDOR_CENTER_SAMPLE_{index}", 1)

    factors: dict[tuple[str, str], dict] = {}

    def add_factor(target: float, middle: float, target_source: str, mid_source: str,
                   source_priority: int) -> None:
        middle = clamp(middle, -TARGET_BOUND_M, TARGET_BOUND_M)
        key = (canonical(target), canonical(middle))
        row = {
            "side": side,
            "d_target": target,
            "d_mid": middle,
            "target_source": target_source,
            "mid_source": mid_source,
            "source_priority": source_priority,
        }
        previous = factors.get(key)
        if previous is None or (source_priority, target_source, mid_source) < (
                previous["source_priority"], previous["target_source"], previous["mid_source"]):
            factors[key] = row

    for target, target_source, target_priority in sorted(target_candidates.values()):
        if (target, target) in production_pairs:
            add_factor(target, target, target_source, "PRODUCTION_OR_EQUAL_TARGET", 0)
        add_factor(target, target, target_source, "MID_EQUALS_TARGET", target_priority)
        for center_index, center in enumerate(geometry["center_values"]):
            for fraction in CENTER_INTERPOLATION_FRACTIONS:
                add_factor(target, target + fraction * (center - target), target_source,
                           f"TO_CORRIDOR_CENTER_{center_index}_F{fraction:.3f}",
                           target_priority + 1)
        for factor in REFERENCE_INWARD_FACTORS:
            add_factor(target, factor * target, target_source,
                       f"REFERENCE_INWARD_F{factor:.2f}", target_priority + 2)
        for offset in MID_ABSOLUTE_OFFSETS_M:
            add_factor(target, target - offset, target_source,
                       f"TARGET_MINUS_{offset:.2f}M", target_priority + 1)
            add_factor(target, target + offset, target_source,
                       f"TARGET_PLUS_{offset:.2f}M", target_priority + 1)
    for target, middle in production_pairs:
        add_factor(target, middle, "PRODUCTION_TARGET", "PRODUCTION_MID", 0)

    output = []
    for index, factor in enumerate(sorted(
            factors.values(), key=lambda row: (
                row["source_priority"], abs(row["d_mid"] - row["d_target"]),
                abs(row["d_target"]), row["target_source"], row["mid_source"],
                canonical(row["d_target"]), canonical(row["d_mid"])))):
        factor["lateral_factor_index"] = index
        factor["lateral_factor_id"] = "L2_" + hashlib.sha256(
            (event_id + "|" + "|".join(config_key({
                **factor, "entry_scale": 0.0, "exit_scale": 0.0,
            })[:3])).encode()).hexdigest()[:12]
        output.append(factor)
    return output


def normalized(value: float, values: list[float]) -> float:
    if len(values) < 2 or values[-1] - values[0] <= EPS:
        return 0.0
    return (value - values[0]) / (values[-1] - values[0])


def pair_metrics(event: dict, geometry: dict, lateral: dict,
                 entry: float, exit_scale: float, entries: list[float],
                 exits: list[float]) -> dict:
    stations = stations_for(event, geometry, geometry["side"], entry, exit_scale,
                            lateral["d_target"])
    positive = all(stations[index + 1] - stations[index] > EPS for index in range(4))
    if not positive:
        return {
            "construction_guard_proxy": 1, "stations": stations,
            "exit_conflict_proxy": 1,
            "max_corridor_violation_m": math.inf, "sum_corridor_violation_m": math.inf,
            "slope_excess": math.inf, "curvature_proxy": math.inf,
            "center_error": math.inf, "minimum_clearance_m": -math.inf,
            "shape_energy": math.inf,
        }
    profile = G.make_profile(stations, [event["ego_d"], lateral["d_target"],
                                        lateral["d_mid"], lateral["d_target"], 0.0])
    sample_stations = {station for station in geometry["reference_stations"]
                       if station <= stations[4] + max(5.0, abs(event["ego_speed"])) + EPS}
    violations = []
    later_obstacle_violations = []
    clearances = []
    center_errors = []
    for station in sorted(value for value in sample_stations if value >= 0.0):
        corridor = G.sample_at(event, geometry["visible"], station, geometry["left"])
        value, _, _ = profile_value_derivatives(profile, event["ego_d"], station)
        lower, upper = corridor["lower"], corridor["upper"]
        if not (math.isfinite(lower) and math.isfinite(upper)):
            violation = TARGET_BOUND_M
            clearance = -TARGET_BOUND_M
        else:
            violation = max(0.0, lower - value, value - upper)
            clearance = min(value - lower, upper - value)
            if geometry["cluster_start"] - EPS <= station <= geometry["cluster_end"] + EPS:
                center_errors.append(abs(value - corridor["center"]) / max(0.05, corridor["width"]))
        violations.append(violation)
        clearances.append(clearance)
        cluster_ids = {row["id"] for row in geometry["cluster"]}
        if station > geometry["cluster_end"] + EPS and any(
                obstacle_id not in cluster_ids for obstacle_id in corridor["active_obstacles"]):
            later_obstacle_violations.append(violation)
    # The exact validator sees the reconstructed reference-waypoint sequence, not hidden
    # extrema between two reference samples.  Use the same discrete sampling topology for
    # the cheap lateral-shape proxies.
    discrete = []
    for station in geometry["reference_stations"]:
        if station > stations[4] + max(5.0, abs(event["ego_speed"])) + EPS:
            break
        value, _, _ = profile_value_derivatives(profile, event["ego_d"], station)
        discrete.append((station, value))
    slopes = []
    for (s0, d0), (s1, d1) in zip(discrete, discrete[1:]):
        if s1 - s0 > EPS:
            slopes.append(abs(d1 - d0) / (s1 - s0))
    second = []
    for (s0, d0), (s1, d1), (s2, d2) in zip(discrete, discrete[1:], discrete[2:]):
        if s1 - s0 > EPS and s2 - s1 > EPS:
            first = (d1 - d0) / (s1 - s0)
            latter = (d2 - d1) / (s2 - s1)
            second.append(abs(latter - first) / max(EPS, 0.5 * (s2 - s0)))
    shape_energy = sum(value * value for value in slopes) / max(1, len(slopes))
    peak_slope = max(slopes, default=math.inf)
    return {
        "construction_guard_proxy": 0,
        "stations": stations,
        "exit_conflict_proxy": int(max(later_obstacle_violations, default=0.0) > EPS),
        "max_corridor_violation_m": max(violations, default=math.inf),
        "sum_corridor_violation_m": sum(violations),
        "slope_excess": max(0.0, peak_slope - MAXIMUM_LATERAL_SLOPE),
        "peak_lateral_slope_proxy": peak_slope,
        "curvature_proxy": max(second, default=math.inf),
        "center_error": sum(center_errors) / len(center_errors) if center_errors else math.inf,
        "minimum_clearance_m": min(clearances, default=-math.inf),
        "shape_energy": shape_energy,
        "entry_normalized": normalized(entry, entries),
        "exit_normalized": normalized(exit_scale, exits),
    }


def build_pair_pool(item: dict, contexts: list[dict]) -> tuple[list[dict], dict]:
    event = G.parse_event(item["event_path"])
    transitions = production_transition_pairs(item["production"])
    entries = unique_exact(entry for entry, _ in transitions)
    exits = unique_exact(exit_scale for _, exit_scale in transitions)
    production_keys = {config_key({
        "side": row["side"], "d_target": float(row["d_target"]),
        "d_mid": float(row["d_mid"]), "entry_scale": float(row["entry_scale"]),
        "exit_scale": float(row["exit_scale"]),
    }) for row in item["production"]}
    pair_pool = []
    lateral_count = 0
    geometry_rows = []
    for context in sorted(contexts, key=lambda row: row["side"] != "RIGHT"):
        geometry = geometry_for_side(event, context)
        laterals = lateral_factors(item["event_id"], event, geometry, item["production"])
        lateral_count += len(laterals)
        geometry_rows.append({
            "side": geometry["side"], "domain_lower": geometry["domain_lower"],
            "domain_upper": geometry["domain_upper"],
            "component_count": len(geometry["components"]),
            "components": geometry["components"],
            "bottleneck_center": geometry["bottleneck_center"],
            "bottleneck_width": geometry["bottleneck_width"],
            "bottleneck_station": geometry["bottleneck_station"],
            "next_obstacle_start": geometry["next_obstacle_start"],
            "lateral_factor_count": len(laterals),
        })
        for lateral in laterals:
            for transition_index, (entry, exit_scale) in enumerate(transitions):
                candidate = {
                    **lateral,
                    "entry_scale": entry,
                    "exit_scale": exit_scale,
                    "transition_index": transition_index,
                }
                if config_key(candidate) in production_keys:
                    continue
                candidate.update(pair_metrics(
                    event, geometry, lateral, entry, exit_scale, entries, exits))
                candidate["configuration_key"] = "|".join(config_key(candidate))
                candidate["preconstruction_shape_key"] = "|".join([
                    candidate["side"], canonical(candidate["d_target"]),
                    canonical(candidate["d_mid"]),
                    *(canonical(value) for value in candidate["stations"]),
                ])
                pair_pool.append(candidate)
    return pair_pool, {
        "event": event,
        "lateral_factor_evaluations": lateral_count,
        "pair_priority_evaluations": len(pair_pool),
        "geometry": geometry_rows,
        "transition_factor_count": len(transitions),
    }


def base_numeric_score(candidate: dict) -> float:
    violation = candidate["max_corridor_violation_m"]
    if not math.isfinite(violation):
        return 1.0e9
    clearance_penalty = max(0.0, 0.02 - candidate["minimum_clearance_m"])
    return (
        40.0 * violation +
        2.0 * candidate["sum_corridor_violation_m"] +
        8.0 * candidate["slope_excess"] +
        0.12 * candidate["curvature_proxy"] +
        0.25 * candidate["center_error"] +
        2.0 * clearance_penalty +
        0.02 * candidate["shape_energy"] +
        0.002 * candidate["source_priority"]
    )


def stable_tie(candidate: dict) -> tuple:
    return (
        candidate["source_priority"], candidate["lateral_factor_index"],
        candidate["transition_index"], candidate["side"] != "RIGHT",
        candidate["configuration_key"],
    )


def deduplicate_shape_order(rows: list[dict]) -> list[dict]:
    output = []
    seen = set()
    for row in rows:
        key = row["preconstruction_shape_key"]
        if key in seen:
            continue
        seen.add(key)
        output.append(row)
    return output


def ordered_candidates(pool: list[dict], hypothesis: str) -> list[dict]:
    feasible = [row for row in pool if not row["construction_guard_proxy"]]
    guarded = [row for row in pool if row["construction_guard_proxy"]]
    if hypothesis == "R1_FEASIBILITY_LEXICOGRAPHIC":
        key = lambda row: (
            row["exit_conflict_proxy"], row["max_corridor_violation_m"] > EPS,
            row["max_corridor_violation_m"], row["slope_excess"] > EPS,
            row["slope_excess"], row["sum_corridor_violation_m"],
            row["curvature_proxy"], row["center_error"],
            -row["minimum_clearance_m"], row["shape_energy"], stable_tie(row))
        return deduplicate_shape_order(
            sorted(feasible, key=key) + sorted(guarded, key=stable_tie))
    if hypothesis == "R2_BALANCED_PHYSICS_SCORE":
        key = lambda row: (
            row["exit_conflict_proxy"], base_numeric_score(row),
            row["max_corridor_violation_m"], row["slope_excess"],
            row["curvature_proxy"], stable_tie(row))
        return deduplicate_shape_order(
            sorted(feasible, key=key) + sorted(guarded, key=stable_tie))
    if hypothesis != "R3_LEXICOGRAPHIC_COVERAGE_RESERVE":
        raise RuntimeError(hypothesis)

    # Greedy best-first: retain the physics score, but reward separation from already
    # selected factors.  This prevents a 24-slot budget from collapsing onto near-identical
    # target/mid values.  No validator/oracle outcome enters the ordering.
    remaining = sorted(feasible, key=lambda row: (
        row["exit_conflict_proxy"], base_numeric_score(row), stable_tie(row)))
    selected = []
    while remaining:
        if not selected:
            index = 0
        else:
            def priority(row: dict) -> tuple:
                distance = min(
                    (0.5 if row["side"] != chosen["side"] else 0.0) +
                    abs(row["d_target"] - chosen["d_target"]) / 1.5 +
                    abs(row["d_mid"] - chosen["d_mid"]) / 3.0 +
                    0.20 * abs(row["entry_normalized"] - chosen["entry_normalized"]) +
                    0.20 * abs(row["exit_normalized"] - chosen["exit_normalized"])
                    for chosen in selected)
                return (
                    row["exit_conflict_proxy"],
                    base_numeric_score(row) - 2.0 * min(1.0, distance),
                    base_numeric_score(row), stable_tie(row))
            index = min(range(len(remaining)), key=lambda item: priority(remaining[item]))
        selected.append(remaining.pop(index))
        # Only the first 48 can be reconstructed under the predeclared attempt cap.
        if len(selected) >= 48:
            break
    if remaining:
        selected.extend(remaining)
    return deduplicate_shape_order(selected + sorted(guarded, key=stable_tie))


def write_requests(path: Path, hypothesis: str, configurations: list[dict]) -> None:
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write("P3_ORACLE_REQUESTS_V1\n")
        for row in configurations:
            stream.write(
                "Q\t{}\t0\t{}\t{:.17g}\t{:.17g}\t{:.17g}\t{:.17g}\n".format(
                    hypothesis, int(row["side"] == "LEFT"), row["d_target"],
                    row["d_mid"], row["entry_scale"], row["exit_scale"]))


def parse_candidate_rows(text: str) -> list[dict]:
    header = None
    output = []
    for line in text.splitlines():
        parts = line.split("\t")
        if parts[0] != "CANDIDATE":
            continue
        if len(parts) > 1 and parts[1] == "event_id":
            header = parts[1:]
        elif header is not None:
            output.append(dict(zip(header, parts[1:])))
    return output


def evaluate_raw_order(item: dict, label: str, ordered: list[dict],
                       attempt_cap: int) -> tuple[list[dict], list[dict], float]:
    WORK.mkdir(parents=True, exist_ok=True)
    configurations = ordered[:attempt_cap]
    request = WORK / f"{item['event_id']}.{label}.requests"
    write_requests(request, label, configurations)
    started = time.perf_counter()
    completed = subprocess.run(
        [str(HARNESS), str(item["event_path"]), str(request)],
        check=True, text=True, stdout=subprocess.PIPE)
    runtime = time.perf_counter() - started
    rows = parse_candidate_rows(completed.stdout)
    if len(rows) != len(configurations):
        raise RuntimeError(f"harness row mismatch {item['event_id']} {label}")

    constructed_configs = []
    constructed_rows = []
    digests = set()
    guards = 0
    duplicates = 0
    raw_validators = 0
    for attempt, (candidate, row) in enumerate(zip(configurations, rows), 1):
        raw_validators += row.get("validator_executed") == "1"
        digest = row.get("path_digest", "")
        if not digest:
            guards += 1
            continue
        if digest in digests:
            duplicates += 1
        else:
            digests.add(digest)
        annotated = dict(row)
        annotated["_reconstruction_attempts"] = attempt
        annotated["_construction_guard_rejects"] = guards
        annotated["_duplicate_constructed_paths"] = duplicates
        annotated["_raw_harness_validator_executions"] = raw_validators
        constructed_configs.append(candidate)
        constructed_rows.append(annotated)
    return constructed_configs, constructed_rows, runtime


def run_ranked_prefix(item: dict, hypothesis: str, ordered: list[dict],
                      pool: list[dict]) -> tuple[list[dict], list[dict], float]:
    if hypothesis != "R3_LEXICOGRAPHIC_COVERAGE_RESERVE":
        return evaluate_raw_order(item, hypothesis, ordered, 48)

    r1_order = ordered_candidates(pool, "R1_FEASIBILITY_LEXICOGRAPHIC")
    coverage_order = ordered_candidates(pool, "R3_LEXICOGRAPHIC_COVERAGE_RESERVE")
    r1_configs, r1_rows, r1_runtime = evaluate_raw_order(
        item, hypothesis + ".R1", r1_order, 24)
    coverage_configs, coverage_rows, coverage_runtime = evaluate_raw_order(
        item, hypothesis + ".COVERAGE", coverage_order, 96)

    # Fully-reconstructed-candidate quota schedule: K12 receives ten lexicographic and two
    # coverage candidates; K24 receives eighteen and six.  A post-reconstruction digest
    # duplicate consumes reconstruction budget but not logical exact-validator budget.
    schedule = (
        [("R1", index) for index in range(10)] +
        [("COVERAGE", index) for index in range(2)] +
        [("R1", index) for index in range(10, 18)] +
        [("COVERAGE", index) for index in range(2, 6)] +
        [("R1", index) for index in range(18, len(r1_rows))] +
        [("COVERAGE", index) for index in range(6, len(coverage_rows))]
    )
    sources = {
        "R1": (r1_configs, r1_rows),
        "COVERAGE": (coverage_configs, coverage_rows),
    }
    output_configs = []
    output_rows = []
    used = {"R1": -1, "COVERAGE": -1}
    for source, index in schedule:
        configs, rows = sources[source]
        if index >= len(rows):
            continue
        used[source] = max(used[source], index)
        row = rows[index]
        attempts = guards = duplicates = validators = 0
        for source_name, used_index in used.items():
            if used_index < 0:
                continue
            source_row = sources[source_name][1][used_index]
            attempts += int(source_row["_reconstruction_attempts"])
            guards += int(source_row["_construction_guard_rejects"])
            duplicates += int(source_row["_duplicate_constructed_paths"])
            validators += int(source_row["_raw_harness_validator_executions"])
        annotated = dict(row)
        annotated["_reconstruction_attempts"] = attempts
        annotated["_construction_guard_rejects"] = guards
        annotated["_duplicate_constructed_paths"] = duplicates
        annotated["_raw_harness_validator_executions"] = validators
        output_configs.append(configs[index])
        output_rows.append(annotated)
        if len(output_rows) >= max(BUDGETS):
            break
    return output_configs, output_rows, r1_runtime + coverage_runtime


def prefixes(configurations: list[dict], rows: list[dict]) -> dict[int, dict]:
    output = {}
    for budget in BUDGETS:
        count = min(budget, len(rows))
        effective_rows = rows[:count]
        effective_configs = configurations[:count]
        last = effective_rows[-1] if effective_rows else {}
        attempts = int(last.get("_reconstruction_attempts", 0))
        unique_rows = []
        unique_configs = []
        digests = set()
        for candidate, row in zip(effective_configs, effective_rows):
            digest = row["path_digest"]
            if digest in digests:
                continue
            digests.add(digest)
            unique_configs.append(candidate)
            unique_rows.append(row)
        hard = [row for row in unique_rows if row["hard_valid"] == "1"]
        usable_rows = [row for row in unique_rows if usable(row)]
        output[budget] = {
            "configs": effective_configs,
            "rows": effective_rows,
            "reconstruction_attempts": attempts,
            "fully_reconstructed_candidate_count": count,
            "constructed_unique_count": len(unique_rows),
            "construction_guard_rejects": int(last.get("_construction_guard_rejects", 0)),
            "duplicate_constructed_paths": count - len(unique_rows),
            "raw_harness_validator_executions": int(
                last.get("_raw_harness_validator_executions", 0)),
            "logical_exact_validator_calls": len(unique_rows),
            "hard_valid_count": len(hard),
            "usable_valid_count": len(usable_rows),
            "hard_recovered": bool(hard),
            "usable_recovered": bool(usable_rows),
            "best_hard": min(hard, key=lambda row: R.method_rank(row, int(row["request_index"])))
                         if hard else None,
            "best_usable": min(usable_rows,
                               key=lambda row: R.method_rank(row, int(row["request_index"])))
                           if usable_rows else None,
        }
    return output


def oracle_rows() -> dict[tuple[str, str], dict]:
    files = {
        "PILOT_SEEN_DEVELOPMENT_DATA": "pilot_seen_recomputed.csv",
        "DEVELOPMENT": "development_recomputed.csv",
        "VALIDATION_SEEN_AFTER_V1": "validation_seen_recomputed.csv",
    }
    output = {}
    for role, filename in files.items():
        for row in read_csv(REFERENCE / filename):
            output[(role, row["event_id"])] = row
    return output


def mechanism_rows() -> dict[tuple[str, str], str]:
    output = {}
    for row in read_csv(REFERENCE / "remaining_failure_taxonomy.csv"):
        output[(row["dataset_role"], row["event_id"])] = row["primary_mechanism"]
    return output


def baseline_rows() -> list[dict]:
    return read_csv(REFERENCE / "frozen_method_event_results.csv")


def aggregate(result_rows: list[dict], metric: str, role: str,
              hypothesis: str, budget: int) -> dict:
    roles = {"DEVELOPMENT", "VALIDATION_SEEN_AFTER_V1"} if role == "COMBINED_SEEN" else {role}
    denominator_field = f"oracle_v2_{metric}_feasible"
    recovered_field = f"{metric}_recovered"
    rows = [row for row in result_rows if row["dataset_role"] in roles
            and row["hypothesis"] == hypothesis and int(row["budget_k"]) == budget
            and truth(row[denominator_field])]
    recovered = [row for row in rows if truth(row[recovered_field])]
    return {
        "dataset_role": role,
        "validity_metric": metric.upper(),
        "hypothesis": hypothesis,
        "budget_k": budget,
        "oracle_v2_feasible_production_failure_count": len(rows),
        "recovered_episode_count": len(recovered),
        "recovery_rate": len(recovered) / len(rows) if rows else math.nan,
        "recovered_event_ids": "|".join(sorted(row["event_id"] for row in recovered)),
        "unrecovered_event_ids": "|".join(sorted(
            row["event_id"] for row in rows if not truth(row[recovered_field]))),
    }


def create_plots(recovery_rows: list[dict], budget_rows: list[dict]) -> None:
    PLOTS.mkdir(parents=True, exist_ok=True)
    for role in ["DEVELOPMENT", "VALIDATION_SEEN_AFTER_V1", "COMBINED_SEEN"]:
        figure, axis = plt.subplots(figsize=(8.6, 5.2))
        for hypothesis in HYPOTHESES:
            rows = [row for row in recovery_rows if row["dataset_role"] == role
                    and row["validity_metric"] == "USABLE"
                    and row["hypothesis"] == hypothesis]
            axis.plot([int(row["budget_k"]) for row in rows],
                      [int(row["recovered_episode_count"]) for row in rows],
                      marker="o", label=hypothesis)
        axis.set(xticks=BUDGETS, xlabel="unique reconstructed candidate budget K",
                 ylabel="usable-recovered oracle-v2-feasible failures",
                 title=f"P3 factor ranking v2 — {role}")
        axis.grid(alpha=.2)
        axis.legend(fontsize=7)
        figure.tight_layout()
        figure.savefig(PLOTS / f"usable_recovery_{role.lower()}.png", dpi=170)
        plt.close(figure)

    figure, axis = plt.subplots(figsize=(8.6, 5.2))
    rows = [row for row in budget_rows if row["dataset_role"] == "COMBINED_SEEN"]
    labels = [f"{row['hypothesis'].split('_')[0]} K{row['budget_k']}" for row in rows]
    axis.bar(range(len(rows)), [float(row["validator_calls_per_episode_p95"]) for row in rows])
    axis.set_xticks(range(len(rows)), labels, rotation=45, ha="right")
    axis.set(ylabel="logical exact validator calls p95",
             title="Bounded expensive-compute budget")
    axis.grid(axis="y", alpha=.2)
    figure.tight_layout()
    figure.savefig(PLOTS / "expensive_compute_budget.png", dpi=170)
    plt.close(figure)


def diagnose_target_cases() -> None:
    """Exact full expanded-pool audit for the four predeclared budget cases."""
    WORK.mkdir(parents=True, exist_ok=True)
    targets = {"DVE029", "DVE039", "VUE013", "VUE025"}
    output = []
    for item in R.input_catalog():
        if item["event_id"] not in targets:
            continue
        pool, _ = build_pair_pool(item, strict_contexts(item["event_path"]))
        request = WORK / f"{item['event_id']}.full_factor_space.requests"
        write_requests(request, "FULL_FACTOR_SPACE_DIAGNOSTIC", pool)
        started = time.perf_counter()
        completed = subprocess.run(
            [str(HARNESS), str(item["event_path"]), str(request)],
            check=True, text=True, stdout=subprocess.PIPE)
        rows = parse_candidate_rows(completed.stdout)
        usable_pairs = [(candidate, row) for candidate, row in zip(pool, rows) if usable(row)]
        hard_pairs = [(candidate, row) for candidate, row in zip(pool, rows)
                      if row["hard_valid"] == "1"]
        for hypothesis in HYPOTHESES:
            ordered = ordered_candidates(pool, hypothesis)
            ranks = {candidate["configuration_key"]: rank
                     for rank, candidate in enumerate(ordered, 1)}
            ranked_usable = sorted(
                (ranks[candidate["configuration_key"]], candidate, row)
                for candidate, row in usable_pairs)
            first = ranked_usable[0] if ranked_usable else None
            output.append({
                "event_id": item["event_id"], "dataset_role": item["dataset_role"],
                "hypothesis": hypothesis, "expanded_pair_count": len(pool),
                "hard_valid_pair_count": len(hard_pairs),
                "usable_valid_pair_count": len(usable_pairs),
                "factor_space_contains_usable": bool(usable_pairs),
                "first_usable_pair_rank_before_reconstruction_dedup": first[0] if first else "",
                "first_usable_side": first[1]["side"] if first else "",
                "first_usable_d_target": first[1]["d_target"] if first else "",
                "first_usable_d_mid": first[1]["d_mid"] if first else "",
                "first_usable_entry_scale": first[1]["entry_scale"] if first else "",
                "first_usable_exit_scale": first[1]["exit_scale"] if first else "",
                "first_usable_target_source": first[1]["target_source"] if first else "",
                "first_usable_mid_source": first[1]["mid_source"] if first else "",
                "full_pool_audit_runtime_s": time.perf_counter() - started,
                "online_use": False,
                "interpretation": "POST_SELECTION_FACTOR_SPACE_COMPLETENESS_DIAGNOSTIC",
            })
        print(f"TARGET_DIAG {item['event_id']} pairs={len(pool)} usable={len(usable_pairs)}",
              flush=True)
    write_csv(HERE / "target_case_factor_space_audit.csv", output)


def main() -> None:
    HERE.mkdir(parents=True, exist_ok=True)
    WORK.mkdir(parents=True, exist_ok=True)
    PLOTS.mkdir(parents=True, exist_ok=True)
    frozen = freeze_gate()
    items = R.input_catalog()
    oracle = oracle_rows()
    mechanisms = mechanism_rows()

    result_rows = []
    selected_rows = []
    factor_stats = []
    for item_index, item in enumerate(items, 1):
        contexts = strict_contexts(item["event_path"])
        pool, stats = build_pair_pool(item, contexts)
        factor_stats.append({
            "dataset_role": item["dataset_role"], "event_id": item["event_id"],
            "strict_side_context_count": len(contexts),
            "lateral_factor_evaluations": stats["lateral_factor_evaluations"],
            "pair_priority_evaluations_per_hypothesis": stats["pair_priority_evaluations"],
            "transition_factor_count": stats["transition_factor_count"],
            "geometry_json": json.dumps(stats["geometry"], sort_keys=True),
        })
        oracle_row = oracle[(item["dataset_role"], item["event_id"])]
        for hypothesis in HYPOTHESES:
            ordered = ordered_candidates(pool, hypothesis)
            configs, raw_rows, runtime = run_ranked_prefix(item, hypothesis, ordered, pool)
            by_budget = prefixes(configs, raw_rows)
            for budget, result in by_budget.items():
                best_hard = result["best_hard"]
                best_usable = result["best_usable"]
                result_rows.append({
                    "dataset_role": item["dataset_role"],
                    "event_id": item["event_id"],
                    "bag": oracle_row["bag"],
                    "hypothesis": hypothesis,
                    "budget_k": budget,
                    "production_first_invoked": True,
                    "oracle_v2_hard_feasible": oracle_row["oracle_v2_hard_label"] ==
                        "ORACLE_V2_HARD_VALID_P3_EXISTS",
                    "oracle_v2_usable_feasible": oracle_row["oracle_v2_usable_label"] ==
                        "ORACLE_V2_USABLE_VALID_P3_EXISTS",
                    "failure_mechanism": mechanisms.get(
                        (item["dataset_role"], item["event_id"]), "NOT_IN_REMAINING_MISS_TABLE"),
                    "lateral_factor_evaluations": stats["lateral_factor_evaluations"],
                    "pair_priority_evaluations": stats["pair_priority_evaluations"],
                    "reconstruction_attempts": result["reconstruction_attempts"],
                    "fully_reconstructed_candidate_count":
                        result["fully_reconstructed_candidate_count"],
                    "constructed_unique_count": result["constructed_unique_count"],
                    "construction_guard_rejects": result["construction_guard_rejects"],
                    "duplicate_constructed_paths": result["duplicate_constructed_paths"],
                    "raw_harness_validator_executions": result["raw_harness_validator_executions"],
                    "logical_exact_validator_calls": result["logical_exact_validator_calls"],
                    "hard_valid_count": result["hard_valid_count"],
                    "usable_valid_count": result["usable_valid_count"],
                    "hard_recovered": result["hard_recovered"],
                    "usable_recovered": result["usable_recovered"],
                    "best_hard_side": best_hard["side"] if best_hard else "",
                    "best_hard_d_target": best_hard["d_target"] if best_hard else "",
                    "best_hard_d_mid": best_hard["d_mid"] if best_hard else "",
                    "best_hard_entry_scale": best_hard["entry_scale"] if best_hard else "",
                    "best_hard_exit_scale": best_hard["exit_scale"] if best_hard else "",
                    "best_usable_path_digest": best_usable["path_digest"] if best_usable else "",
                    "audit_harness_wall_runtime_s_shared_across_budgets": runtime,
                })
                seen_selected_digests = set()
                for rank, (candidate, row) in enumerate(zip(result["configs"], result["rows"]), 1):
                    duplicate_digest = row["path_digest"] in seen_selected_digests
                    seen_selected_digests.add(row["path_digest"])
                    selected_rows.append({
                        "dataset_role": item["dataset_role"], "event_id": item["event_id"],
                        "hypothesis": hypothesis, "budget_k": budget, "selection_rank": rank,
                        "side": candidate["side"], "d_target": candidate["d_target"],
                        "d_mid": candidate["d_mid"], "entry_scale": candidate["entry_scale"],
                        "exit_scale": candidate["exit_scale"],
                        "target_source": candidate["target_source"],
                        "mid_source": candidate["mid_source"],
                        "max_corridor_violation_m": candidate["max_corridor_violation_m"],
                        "slope_excess": candidate["slope_excess"],
                        "curvature_proxy": candidate["curvature_proxy"],
                        "exit_conflict_proxy": candidate["exit_conflict_proxy"],
                        "path_digest": row["path_digest"], "hard_valid": row["hard_valid"],
                        "usable_valid": usable(row),
                        "duplicate_path_digest_within_budget": duplicate_digest,
                        "first_failure_reason": row["first_failure_reason"],
                    })
        print(f"V2 {item_index}/{len(items)} {item['dataset_role']} {item['event_id']}", flush=True)

    write_csv(HERE / "factor_ranking_results.csv", result_rows)
    write_csv(HERE / "selected_candidates_audit.csv", selected_rows)
    write_csv(HERE / "factor_generation_audit.csv", factor_stats)

    recovery_rows = []
    for role in ["DEVELOPMENT", "VALIDATION_SEEN_AFTER_V1", "COMBINED_SEEN"]:
        for metric in ["hard", "usable"]:
            for hypothesis in HYPOTHESES:
                for budget in BUDGETS:
                    recovery_rows.append(aggregate(result_rows, metric, role, hypothesis, budget))
    write_csv(HERE / "recovery_by_budget.csv", recovery_rows)

    mechanism_output = []
    mechanism_names = sorted({row["failure_mechanism"] for row in result_rows})
    for role in ["DEVELOPMENT", "VALIDATION_SEEN_AFTER_V1", "COMBINED_SEEN"]:
        roles = {"DEVELOPMENT", "VALIDATION_SEEN_AFTER_V1"} if role == "COMBINED_SEEN" else {role}
        for hypothesis in HYPOTHESES:
            for budget in BUDGETS:
                for mechanism in mechanism_names:
                    rows = [row for row in result_rows if row["dataset_role"] in roles
                            and row["hypothesis"] == hypothesis and int(row["budget_k"]) == budget
                            and truth(row["oracle_v2_usable_feasible"])
                            and row["failure_mechanism"] == mechanism]
                    recovered = [row for row in rows if truth(row["usable_recovered"])]
                    mechanism_output.append({
                        "dataset_role": role, "hypothesis": hypothesis, "budget_k": budget,
                        "failure_mechanism": mechanism, "usable_feasible_count": len(rows),
                        "usable_recovered_count": len(recovered),
                        "recovered_event_ids": "|".join(sorted(row["event_id"] for row in recovered)),
                        "unrecovered_event_ids": "|".join(sorted(
                            row["event_id"] for row in rows if not truth(row["usable_recovered"]))),
                    })
    write_csv(HERE / "recovery_by_failure_mechanism.csv", mechanism_output)

    budget_output = []
    for role in ["DEVELOPMENT", "VALIDATION_SEEN_AFTER_V1", "COMBINED_SEEN"]:
        roles = {"DEVELOPMENT", "VALIDATION_SEEN_AFTER_V1"} if role == "COMBINED_SEEN" else {role}
        for hypothesis in HYPOTHESES:
            for budget in BUDGETS:
                rows = [row for row in result_rows if row["dataset_role"] in roles
                        and row["hypothesis"] == hypothesis and int(row["budget_k"]) == budget]
                def values(field):
                    return [float(row[field]) for row in rows]
                budget_output.append({
                    "dataset_role": role, "hypothesis": hypothesis, "budget_k": budget,
                    "episode_count": len(rows),
                    "cheap_lateral_factor_evaluations_total": sum(values("lateral_factor_evaluations")),
                    "cheap_pair_priority_evaluations_total": sum(values("pair_priority_evaluations")),
                    "p3_reconstruction_attempts_total": sum(values("reconstruction_attempts")),
                    "p3_reconstruction_attempts_per_episode_p95": percentile(values("reconstruction_attempts"), 95),
                    "p3_reconstruction_attempts_per_episode_max": max(values("reconstruction_attempts")),
                    "fully_reconstructed_candidates_total": sum(
                        values("fully_reconstructed_candidate_count")),
                    "unique_constructed_candidates_total": sum(values("constructed_unique_count")),
                    "validator_calls_total": sum(values("logical_exact_validator_calls")),
                    "validator_calls_per_episode_p95": percentile(values("logical_exact_validator_calls"), 95),
                    "validator_calls_per_episode_max": max(values("logical_exact_validator_calls")),
                    "raw_audit_harness_validator_executions_total": sum(values("raw_harness_validator_executions")),
                    "construction_guard_rejects_total": sum(values("construction_guard_rejects")),
                    "duplicate_constructed_paths_total": sum(values("duplicate_constructed_paths")),
                })
    write_csv(HERE / "expensive_compute_budget.csv", budget_output)

    success_manifest = read_csv(METHOD_V1 / "success_control_manifest.csv")
    success_rows = [{
        "control_event_id": row["control_event_id"], "dataset_role": row["dataset_role"],
        "bag": row["bag"], "production_hard_valid_count": row["production_hard_valid_count"],
        "production_digest_parity": truth(row["constructed_digest_multiset_parity"]) and
            truth(row["returned_digest_multiset_parity"]),
        "production_first_v2_invoked": False,
        "additional_factor_evaluations": 0, "additional_pair_priority_evaluations": 0,
        "additional_p3_reconstructions": 0, "additional_exact_validator_calls": 0,
        "selected_path_changed": False,
        "contract": "PRODUCTION_HARD_VALID_THEN_SKIP_V2_STAGE",
    } for row in success_manifest]
    write_csv(HERE / "success_control.csv", success_rows)

    # Frozen baseline comparison; per-method exact rows were recomputed under Oracle v2.
    comparison = []
    frozen_rows = baseline_rows()
    for role in ["DEVELOPMENT", "VALIDATION_SEEN_AFTER_V1", "COMBINED_SEEN"]:
        roles = {"DEVELOPMENT", "VALIDATION_SEEN_AFTER_V1"} if role == "COMBINED_SEEN" else {role}
        for method in ["PRODUCTION", "H3_FIXED_BOUNDED", "H4A_GEOMETRY_TRANSITION",
                       "H4B_GEOMETRY_LATERAL_TRANSITION", "REFERENCE_ORACLE_V2"]:
            if method == "REFERENCE_ORACLE_V2":
                source = [row for (row_role, _), row in oracle.items() if row_role in roles]
                usable_feasible = sum(row["oracle_v2_usable_label"] ==
                                      "ORACLE_V2_USABLE_VALID_P3_EXISTS" for row in source)
                hard_feasible = sum(row["oracle_v2_hard_label"] ==
                                    "ORACLE_V2_HARD_VALID_P3_EXISTS" for row in source)
                comparison.append({
                    "dataset_role": role, "method": method, "budget_k": "FINITE_EXHAUSTIVE",
                    "hard_feasible_denominator": hard_feasible, "hard_recovered": hard_feasible,
                    "usable_feasible_denominator": usable_feasible,
                    "usable_recovered": usable_feasible,
                    "interpretation": "OFFLINE_FINITE_DOMAIN_REFERENCE_NOT_ONLINE",
                })
                continue
            source = [row for row in frozen_rows if row["dataset_role"] in roles
                      and row["method"] == method]
            comparison.append({
                "dataset_role": role, "method": method,
                "budget_k": source[0]["candidate_budget_k"] if source else "",
                "hard_feasible_denominator": sum(truth(row["oracle_v2_hard_feasible"]) for row in source),
                "hard_recovered": sum(truth(row["hard_valid_recovered"]) for row in source),
                "usable_feasible_denominator": sum(truth(row["oracle_v2_usable_feasible"]) for row in source),
                "usable_recovered": sum(truth(row["usable_valid_recovered"]) for row in source),
                "interpretation": "FROZEN_EXACT_RECOMPUTATION",
            })
        for hypothesis in HYPOTHESES:
            for budget in BUDGETS:
                hard = aggregate(result_rows, "hard", role, hypothesis, budget)
                use = aggregate(result_rows, "usable", role, hypothesis, budget)
                comparison.append({
                    "dataset_role": role, "method": hypothesis, "budget_k": budget,
                    "hard_feasible_denominator": hard["oracle_v2_feasible_production_failure_count"],
                    "hard_recovered": hard["recovered_episode_count"],
                    "usable_feasible_denominator": use["oracle_v2_feasible_production_failure_count"],
                    "usable_recovered": use["recovered_episode_count"],
                    "interpretation": "SEEN_ONLY_V2_DESIGN_RESULT",
                })
    write_csv(HERE / "frozen_baseline_comparison.csv", comparison)
    create_plots(recovery_rows, budget_output)

    manifest = {
        "study": "P3_GEOMETRY_FACTOR_RANKING_V2_SEEN_ONLY",
        "datasets_loaded": Counter(item["dataset_role"] for item in items),
        "allowed_roles": DATASET_ROLES,
        "final_holdout_rows_loaded": 0,
        "split_manifest_parsed": False,
        "frozen_authorities": frozen,
        "hypotheses": HYPOTHESES,
        "budgets": BUDGETS,
        "production_source_modified": False,
        "planner_algorithm_modified": False,
        "oracle_coordinates_used_for_online_factor_generation_or_ranking": False,
        "usable_valid_predicate": (
            "hard_valid && !exit_reaches_next_obstacle && braking_deficit_m <= 1e-9"),
        "single_stream_request_cap": 48,
        "coverage_reserve_r1_stream_cap": 24,
        "coverage_reserve_coverage_stream_cap": 96,
        "fully_reconstructed_candidate_budgets": BUDGETS,
        "harness_limitation": (
            "audit harness couples reconstruction and validation; logical online validator "
            "count removes post-reconstruction path-digest duplicates"),
    }
    (HERE / "execution_manifest.json").write_text(
        json.dumps(manifest, indent=2, default=dict) + "\n", encoding="utf-8")


if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] == "--diagnose-targets":
        diagnose_target_cases()
    else:
        main()
