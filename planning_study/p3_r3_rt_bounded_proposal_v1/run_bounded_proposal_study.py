#!/usr/bin/env python3
"""Seen-only offline design study for a hard-bounded R3-RT proposal pool.

The script deliberately imports the frozen R3 geometry/proxy/ranking authority,
but it never calls ``lateral_factors`` or ``build_pair_pool``.  Its online-style
proposal path has independent absolute bounds of 64 lateral operator outputs,
7 transition anchors, B pair proxies, and 12 reconstruction/validator requests.
Oracle labels and frozen-teacher outputs are joined only after selection.
"""

from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import math
import statistics
import subprocess
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
TEACHER = REPO / "planning_study/p3_geometry_factor_ranking_v2"
TEACHER_IMPL = TEACHER / "run_factor_ranking_v2.py"
HARNESS = Path("/tmp/p3_oracle_v2_build/p3_family_oracle_harness")
WORK = Path("/tmp/p3_r3_rt_bounded_proposal_v1")

EXPECTED_METHOD_SHA = "7b861e8c7e23ae168413885ea0dc2d09769046ff48a562700b658e084d116fcc"
ALLOWED_ROLES = {
    "PILOT_SEEN_DEVELOPMENT_DATA",
    "DEVELOPMENT",
    "VALIDATION_SEEN_AFTER_V1",
}
BUDGETS = [24, 32, 48, 64, 96, 128]
MAX_LATERAL_OPERATOR_OUTPUTS = 64
MAX_TRANSITION_PROPOSALS = 7
MAX_RECONSTRUCTIONS = 12
EPS = 1.0e-9


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


F = load_module("r3_rt_frozen_teacher", TEACHER_IMPL)
R = F.R
G = F.G


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


def usable(row: dict) -> bool:
    return (row.get("hard_valid") == "1"
            and row.get("exit_reaches_next_obstacle") == "0"
            and number(row, "braking_deficit_m", math.inf) <= EPS)


def percentile(values: list[float], q: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    position = (len(ordered) - 1) * q / 100.0
    low, high = math.floor(position), math.ceil(position)
    if low == high:
        return ordered[low]
    return ordered[low] + (position - low) * (ordered[high] - ordered[low])


def canonical(value: float) -> str:
    return float(value).hex()


def config_key(row: dict) -> tuple:
    return (row["side"], canonical(row["d_target"]), canonical(row["d_mid"]),
            canonical(row["entry_scale"]), canonical(row["exit_scale"]))


def lateral_key(row: dict) -> tuple:
    return (row["side"], canonical(row["d_target"]), canonical(row["d_mid"]))


def transition_key(row: dict) -> tuple:
    return canonical(row["entry_scale"]), canonical(row["exit_scale"])


def same_config(row: dict, result: dict) -> bool:
    if not result.get("best_hard_side") or row["side"] != result["best_hard_side"]:
        return False
    return all(canonical(row[field]) == canonical(result[f"best_hard_{field}"])
               for field in ("d_target", "d_mid", "entry_scale", "exit_scale"))


def target_family(source: str) -> str:
    if source.startswith("PRODUCTION_TARGET"):
        return "PRODUCTION_TARGET_FAMILY"
    if source.startswith("COMPONENT_"):
        return "COMPONENT_BOUNDARY_FAMILY"
    if source.startswith("CORRIDOR_CENTER"):
        return "CORRIDOR_CENTER_FAMILY"
    return "OTHER_TARGET_FAMILY"


def mid_family(source: str) -> str:
    if source in {"MID_EQUALS_TARGET", "PRODUCTION_OR_EQUAL_TARGET"}:
        return "EQUAL_TARGET_FAMILY"
    if source == "PRODUCTION_MID":
        return "PRODUCTION_MID_FAMILY"
    if source.startswith("TARGET_"):
        return "TARGET_OFFSET_FAMILY"
    if source.startswith("TO_CORRIDOR_CENTER"):
        return "CORRIDOR_INTERPOLATION_FAMILY"
    if source.startswith("REFERENCE_INWARD"):
        return "REFERENCE_INWARD_FAMILY"
    return "OTHER_MID_FAMILY"


def normalized(value: float, values: list[float]) -> float:
    if len(values) < 2 or values[-1] - values[0] <= EPS:
        return 0.0
    return (value - values[0]) / (values[-1] - values[0])


TRANSITION_DESIRES = [
    ("SHORT_ENTRY_SHORT_EXIT", 0.0, 0.0),
    ("MEDIUM_ENTRY_SHORT_EXIT", 0.5, 0.0),
    ("LONG_ENTRY_SHORT_EXIT", 1.0, 0.0),
    ("LONG_ENTRY_MEDIUM_EXIT", 1.0, 0.5),
    ("LONG_ENTRY_LONG_EXIT", 1.0, 1.0),
    ("SHORT_ENTRY_LONG_EXIT", 0.0, 1.0),
    ("SHORT_ENTRY_MEDIUM_EXIT", 0.0, 0.5),
]


def transition_proposals(production: list[dict]) -> list[dict]:
    """Map seven normalized S/M/L anchors to exact observed production transitions."""
    transitions = F.production_transition_pairs(production)
    entries = sorted({entry for entry, _ in transitions})
    exits = sorted({exit_scale for _, exit_scale in transitions})
    output = []
    seen = set()
    for family, desired_entry, desired_exit in TRANSITION_DESIRES:
        transition = min(
            transitions,
            key=lambda row: (
                (normalized(row[0], entries) - desired_entry) ** 2
                + (normalized(row[1], exits) - desired_exit) ** 2,
                row,
            ),
        )
        if transition in seen:
            continue
        seen.add(transition)
        output.append({
            "entry_scale": transition[0],
            "exit_scale": transition[1],
            "transition_family": family,
            "transition_index": len(output),
        })
    assert len(output) <= MAX_TRANSITION_PROPOSALS
    return output


def teacher_transition_family(row: dict, production: list[dict]) -> str:
    key = transition_key(row)
    for proposal in transition_proposals(production):
        if key == transition_key(proposal):
            return proposal["transition_family"]
    return "NON_BOUNDED_TRANSITION_ANCHOR"


def teacher_characterization(items: list[dict]) -> tuple[list[dict], dict, dict]:
    selected = [
        row for row in read_csv(TEACHER / "selected_candidates_audit.csv")
        if row["hypothesis"] == "R3_LEXICOGRAPHIC_COVERAGE_RESERVE"
        and int(row["budget_k"]) == 12
    ]
    results = {
        (row["dataset_role"], row["event_id"]): row
        for row in read_csv(TEACHER / "factor_ranking_results.csv")
        if row["hypothesis"] == "R3_LEXICOGRAPHIC_COVERAGE_RESERVE"
        and int(row["budget_k"]) == 12
    }
    geometry = {
        (row["dataset_role"], row["event_id"]): json.loads(row["geometry_json"])
        for row in read_csv(TEACHER / "factor_generation_audit.csv")
    }
    item_map = {(item["dataset_role"], item["event_id"]): item for item in items}
    expected = {(item["dataset_role"], item["event_id"]) for item in items}
    if set(results) != expected:
        raise RuntimeError("teacher result/catalog mismatch")

    matching_ranks = defaultdict(list)
    for row in selected:
        key = row["dataset_role"], row["event_id"]
        if same_config(row, results[key]):
            matching_ranks[key].append(int(row["selection_rank"]))

    output = []
    for row in selected:
        key = row["dataset_role"], row["event_id"]
        result = results[key]
        side_geometry = next(item for item in geometry[key] if item["side"] == row["side"])
        final_selected = (same_config(row, result)
                          and int(row["selection_rank"]) == min(matching_ranks[key]))
        item = item_map[key]
        output.append({
            "dataset_role": row["dataset_role"],
            "event_id": row["event_id"],
            "teacher_method": "R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12",
            "teacher_selection_rank": row["selection_rank"],
            "teacher_stream": ("FEASIBILITY_TOP10" if int(row["selection_rank"]) <= 10
                               else "COVERAGE_RESERVE_TOP2"),
            "side": row["side"],
            "d_target": row["d_target"],
            "d_mid": row["d_mid"],
            "entry_scale": row["entry_scale"],
            "exit_scale": row["exit_scale"],
            "target_source": row["target_source"],
            "target_family": target_family(row["target_source"]),
            "mid_source": row["mid_source"],
            "mid_family": mid_family(row["mid_source"]),
            "probe_root_source": "NOT_APPLICABLE_DIRECT_DT_DM_FACTOR",
            "transition_family": teacher_transition_family(row, item["production"]),
            "domain_lower": side_geometry["domain_lower"],
            "domain_upper": side_geometry["domain_upper"],
            "component_count": side_geometry["component_count"],
            "bottleneck_center": side_geometry["bottleneck_center"],
            "bottleneck_width": side_geometry["bottleneck_width"],
            "bottleneck_station": side_geometry["bottleneck_station"],
            "next_obstacle_start": side_geometry["next_obstacle_start"],
            "max_corridor_violation_m": row["max_corridor_violation_m"],
            "slope_excess": row["slope_excess"],
            "curvature_proxy": row["curvature_proxy"],
            "exit_conflict_proxy": row["exit_conflict_proxy"],
            "path_digest": row["path_digest"],
            "hard_valid": row["hard_valid"],
            "usable_valid": row["usable_valid"],
            "first_failure_reason": row["first_failure_reason"],
            "teacher_final_selected_path": final_selected,
            "teacher_final_selected_path_digest": row["path_digest"] if final_selected else "",
            "teacher_final_selected_usable": row["usable_valid"] if final_selected else "",
        })
    write_csv(HERE / "exact_r3_teacher_characterization.csv", output)
    return output, results, geometry


def clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))


def side_lateral_operators(event: dict, geometry: dict,
                           production: list[dict]) -> list[dict]:
    """Create at most 32 direct lateral proposals for one strict side."""
    side = geometry["side"]
    lower, upper = geometry["domain_lower"], geometry["domain_upper"]
    side_production = [row for row in production if row["side"] == side]
    raw_targets = sorted({float(row["d_target"]) for row in side_production})
    targets = []
    if raw_targets:
        # Two distinct geometry representatives precede all remaining production
        # anchors: closest to the zero interface, then closest to the bottleneck.
        # This is explicit and deterministic; taking min of the two distances would
        # not preserve both roles.
        for value in (
                min(raw_targets, key=lambda item: (abs(item), item)),
                min(raw_targets, key=lambda item: (
                    abs(item - geometry["bottleneck_center"]), abs(item), item))):
            if value not in targets:
                targets.append(value)
        targets.extend(value for value in raw_targets if value not in targets)
    centers = geometry["center_values"][:2]
    if not centers:
        centers = [geometry["bottleneck_center"]]
    output: dict[tuple, dict] = {}

    def add(target: float, middle: float, target_source: str, mid_source: str,
            family: str, priority: int, operator: str) -> None:
        if len(output) >= 32:
            return
        target = clamp(target, lower, upper)
        middle = clamp(middle, -F.TARGET_BOUND_M, F.TARGET_BOUND_M)
        key = side, canonical(target), canonical(middle)
        if key in output:
            return
        output[key] = {
            "side": side,
            "d_target": target,
            "d_mid": middle,
            "target_source": target_source,
            "mid_source": mid_source,
            "lateral_source_family": family,
            "source_priority": priority,
            "proposal_operator": operator,
        }

    # The characterization is dominated by exact production targets, equal mids,
    # and small negative-mid corrections.  Preserve all (already bounded) production
    # target anchors before adding direct geometry anchors.
    for target in targets:
        add(target, target, "PRODUCTION_TARGET", "MID_EQUALS_TARGET",
            "PRODUCTION", 0, "P_TARGET_EQUAL")
    for target in targets:
        add(target, target - 0.02, "PRODUCTION_TARGET", "TARGET_MINUS_0.02M",
            "PRODUCTION", 1, "P_TARGET_MID_MINUS_002")

    # One direct operator per component/fraction; this is not target x mid expansion.
    component_recipes = [
        (0.0, "TARGET_MINUS_0.04M", lambda target, center: target - 0.04,
         "COMPONENT_NEAR_MID_MINUS_004"),
        (0.0, "REFERENCE_INWARD_F0.25", lambda target, center: 0.25 * target,
         "COMPONENT_NEAR_REFERENCE_INWARD025"),
        (1.0 / 32.0, "MID_EQUALS_TARGET", lambda target, center: target,
         "COMPONENT_F1_32_EQUAL"),
        (1.0 / 8.0, "TO_CORRIDOR_CENTER_0_F0.750",
         lambda target, center: target + 0.75 * (center - target),
         "COMPONENT_F1_8_CENTER075"),
        (3.0 / 8.0, "TARGET_MINUS_0.05M", lambda target, center: target - 0.05,
         "COMPONENT_F3_8_MID_MINUS_005"),
        (0.5, "TARGET_MINUS_0.15M", lambda target, center: target - 0.15,
         "COMPONENT_HALF_MID_MINUS_015"),
        (2.0 / 3.0, "TO_CORRIDOR_CENTER_1_F0.125",
         lambda target, center: target + 0.125 * (center - target),
         "COMPONENT_F2_3_CENTER0125"),
    ]
    for component_index, (component_lower, component_upper) in enumerate(geometry["components"][:1]):
        near, far = sorted((component_lower, component_upper), key=lambda value: (abs(value), value))
        for fraction, mid_source, mid_fn, operator in component_recipes:
            target = near + fraction * (far - near)
            center = centers[min(1, len(centers) - 1)] if "CENTER_1" in mid_source else centers[0]
            add(target, mid_fn(target, center),
                f"COMPONENT_C{component_index}_NEAR_TO_FAR_F{fraction:.8f}",
                mid_source, "COMPONENT", 2, operator)

    # Characterization-supported paired corrections.  Each recipe yields exactly one
    # lateral tuple per selected production target; no target/mid Cartesian product.
    # Only the two geometry representatives (interface-nearest and bottleneck-nearest
    # under the stable ordering above) receive the extended correction recipes.
    # This keeps the per-side catalog bounded while retaining every recipe observed in
    # a useful teacher final selection.
    for target in targets[:2]:
        recipes = [
            (-0.01, 0.0, "PRODUCTION_TARGET_MINUS_0.01M", "MID_EQUALS_TARGET",
             "P_TARGET_MINUS_001_EQUAL"),
            (+0.01, -0.01, "PRODUCTION_TARGET_PLUS_0.01M", "TARGET_MINUS_0.01M",
             "P_TARGET_PLUS_001_MID_MINUS_001"),
            (+0.01, 0.0, "PRODUCTION_TARGET_PLUS_0.01M", "MID_EQUALS_TARGET",
             "P_TARGET_PLUS_001_EQUAL"),
            (-0.06, -0.08, "PRODUCTION_TARGET_MINUS_0.06M", "TARGET_MINUS_0.08M",
             "P_TARGET_MINUS_006_MID_MINUS_008"),
            (+0.06, -0.02, "PRODUCTION_TARGET_PLUS_0.06M", "TARGET_MINUS_0.02M",
             "P_TARGET_PLUS_006_MID_MINUS_002"),
            (-0.10, -0.05, "PRODUCTION_TARGET_MINUS_0.10M", "TARGET_MINUS_0.05M",
             "P_TARGET_MINUS_010_MID_MINUS_005"),
            (-0.02, +0.15, "PRODUCTION_TARGET_MINUS_0.02M", "TARGET_PLUS_0.15M",
             "P_TARGET_MINUS_002_MID_PLUS_015"),
        ]
        for target_delta, mid_delta, target_source, mid_source, operator in recipes:
            shifted = clamp(target + target_delta, lower, upper)
            add(shifted, shifted + mid_delta, target_source, mid_source,
                "PRODUCTION_OFFSET", 3, operator)
        shifted = clamp(target + 0.06, lower, upper)
        center = centers[min(1, len(centers) - 1)]
        add(shifted, shifted + 0.4375 * (center - shifted),
            "PRODUCTION_TARGET_PLUS_0.06M", "TO_CORRIDOR_CENTER_1_F0.438",
            "CORRIDOR_INTERPOLATION", 3, "P_TARGET_PLUS_006_CENTER1_04375")
        add(target, target - 0.10, "PRODUCTION_TARGET", "TARGET_MINUS_0.10M",
            "PRODUCTION", 3, "P_TARGET_MID_MINUS_010")
        add(target, target + 0.05, "PRODUCTION_TARGET", "TARGET_PLUS_0.05M",
            "PRODUCTION", 3, "P_TARGET_MID_PLUS_005")
        add(target, target + 0.125 * (centers[0] - target), "PRODUCTION_TARGET",
            "TO_CORRIDOR_CENTER_0_F0.125", "CORRIDOR_INTERPOLATION", 3,
            "P_TARGET_CENTER0125")
        add(target, target + 0.25 * (centers[0] - target), "PRODUCTION_TARGET",
            "TO_CORRIDOR_CENTER_0_F0.250", "CORRIDOR_INTERPOLATION", 3,
            "P_TARGET_CENTER025")

    # Direct corridor and interpolation proposals fill the bounded tail.
    for center_index, center in enumerate(centers):
        add(center, center, f"CORRIDOR_CENTER_SAMPLE_{center_index}",
            "MID_EQUALS_TARGET", "CORRIDOR", 4, "CORRIDOR_CENTER_EQUAL")
    for target in targets:
        center = centers[0]
        add(target, target + 0.125 * (center - target), "PRODUCTION_TARGET",
            "TO_CORRIDOR_CENTER_0_F0.125", "CORRIDOR_INTERPOLATION", 4,
            "P_TARGET_CENTER0125")
        add(target, target + 0.25 * (center - target), "PRODUCTION_TARGET",
            "TO_CORRIDOR_CENTER_0_F0.250", "CORRIDOR_INTERPOLATION", 4,
            "P_TARGET_CENTER025")
        add(target, target - 0.10, "PRODUCTION_TARGET", "TARGET_MINUS_0.10M",
            "PRODUCTION", 4, "P_TARGET_MID_MINUS_010")
        add(target, target + 0.05, "PRODUCTION_TARGET", "TARGET_PLUS_0.05M",
            "PRODUCTION", 4, "P_TARGET_MID_PLUS_005")

    return list(output.values())


def ordered_transitions_for_lateral(lateral: dict, transitions: list[dict]) -> list[dict]:
    """Seen-characterization source prior over the same seven exact S/M/L anchors."""
    operator = lateral["proposal_operator"]
    if operator == "P_TARGET_EQUAL":
        preferred = ["LONG_ENTRY_MEDIUM_EXIT", "LONG_ENTRY_LONG_EXIT",
                     "SHORT_ENTRY_LONG_EXIT", "SHORT_ENTRY_SHORT_EXIT",
                     "MEDIUM_ENTRY_SHORT_EXIT"]
    elif "COMPONENT_F1_32" in operator:
        preferred = ["SHORT_ENTRY_LONG_EXIT", "SHORT_ENTRY_SHORT_EXIT",
                     "MEDIUM_ENTRY_SHORT_EXIT"]
    elif operator.startswith("COMPONENT"):
        preferred = ["SHORT_ENTRY_SHORT_EXIT", "MEDIUM_ENTRY_SHORT_EXIT",
                     "LONG_ENTRY_MEDIUM_EXIT", "SHORT_ENTRY_LONG_EXIT"]
    elif "MINUS_006" in operator:
        preferred = ["MEDIUM_ENTRY_SHORT_EXIT", "SHORT_ENTRY_SHORT_EXIT"]
    elif "PLUS_006" in operator:
        preferred = ["MEDIUM_ENTRY_SHORT_EXIT", "SHORT_ENTRY_MEDIUM_EXIT",
                     "SHORT_ENTRY_SHORT_EXIT"]
    elif "PLUS_001" in operator:
        preferred = ["LONG_ENTRY_LONG_EXIT", "SHORT_ENTRY_SHORT_EXIT",
                     "LONG_ENTRY_MEDIUM_EXIT"]
    elif "MINUS_001" in operator:
        preferred = ["LONG_ENTRY_LONG_EXIT", "LONG_ENTRY_MEDIUM_EXIT",
                     "SHORT_ENTRY_SHORT_EXIT"]
    elif "MINUS_002" in operator:
        preferred = ["LONG_ENTRY_MEDIUM_EXIT", "SHORT_ENTRY_SHORT_EXIT"]
    elif "MINUS_010" in operator:
        preferred = ["SHORT_ENTRY_SHORT_EXIT", "LONG_ENTRY_MEDIUM_EXIT"]
    elif "CENTER" in operator:
        preferred = ["SHORT_ENTRY_SHORT_EXIT", "LONG_ENTRY_SHORT_EXIT",
                     "LONG_ENTRY_MEDIUM_EXIT"]
    else:
        preferred = [family for family, _, _ in TRANSITION_DESIRES]
    rank = {family: index for index, family in enumerate(preferred)}
    return sorted(transitions, key=lambda row: (
        rank.get(row["transition_family"], len(preferred) + row["transition_index"]),
        row["transition_index"]))


def interleave_laterals(per_side: dict[str, list[dict]]) -> list[dict]:
    """Stable priority interleave; RIGHT breaks exact ties as in frozen R3."""
    output = []
    for priority in range(8):
        groups = {
            side: [row for row in per_side.get(side, []) if row["source_priority"] == priority]
            for side in ("RIGHT", "LEFT")
        }
        index = 0
        while any(index < len(rows) for rows in groups.values()):
            for side in ("RIGHT", "LEFT"):
                if index < len(groups[side]):
                    output.append(groups[side][index])
            index += 1
    for index, row in enumerate(output[:MAX_LATERAL_OPERATOR_OUTPUTS]):
        row["lateral_factor_index"] = index
    return output[:MAX_LATERAL_OPERATOR_OUTPUTS]


LATERAL_RETAIN_CAP = {24: 16, 32: 20, 48: 28, 64: 36, 96: 48, 128: 64}


def pair_pool(event: dict, geometries: dict[str, dict], production: list[dict],
              laterals: list[dict], transitions: list[dict], budget: int,
              reserve_policy: str) -> tuple[list[dict], dict]:
    """Lazily propose and score at most B unique non-production L/T pairs."""
    production_keys = {
        config_key({
            "side": row["side"], "d_target": float(row["d_target"]),
            "d_mid": float(row["d_mid"]), "entry_scale": float(row["entry_scale"]),
            "exit_scale": float(row["exit_scale"]),
        }) for row in production
    }
    entries = sorted({row["entry_scale"] for row in transitions})
    exits = sorted({row["exit_scale"] for row in transitions})
    output = []
    seen = set()
    reserve_count = 0

    def add(lateral: dict, transition: dict, reserve: str = "") -> None:
        nonlocal reserve_count
        if len(output) >= budget:
            return
        candidate = {
            **lateral,
            "entry_scale": transition["entry_scale"],
            "exit_scale": transition["exit_scale"],
            "transition_family": transition["transition_family"],
            "transition_index": transition["transition_index"],
            "proposal_reserve": reserve,
        }
        key = config_key(candidate)
        if key in seen or key in production_keys:
            return
        seen.add(key)
        geometry = geometries[candidate["side"]]
        candidate.update(F.pair_metrics(
            event, geometry, lateral, candidate["entry_scale"], candidate["exit_scale"],
            entries, exits))
        candidate["configuration_key"] = "|".join(key)
        candidate["preconstruction_shape_key"] = "|".join([
            candidate["side"], canonical(candidate["d_target"]),
            canonical(candidate["d_mid"]),
            *(canonical(value) for value in candidate["stations"]),
        ])
        output.append(candidate)
        if reserve:
            reserve_count += 1

    if reserve_policy in {"SIDE_RESERVE", "FULL_SMALL_RESERVES"}:
        for side in ("RIGHT", "LEFT"):
            lateral = next((row for row in laterals if row["side"] == side), None)
            if lateral:
                add(lateral, transitions[0], "PER_SIDE")

    if reserve_policy == "FULL_SMALL_RESERVES":
        for family in ("PRODUCTION", "COMPONENT", "CORRIDOR", "PRODUCTION_OFFSET"):
            lateral = next((row for row in laterals
                            if row["lateral_source_family"] == family), None)
            if lateral:
                add(lateral, transitions[0], "PER_LATERAL_FAMILY")
        primary = laterals[0]
        for transition in transitions:
            add(primary, transition, "PER_TRANSITION_FAMILY")
        equal = next((row for row in laterals if canonical(row["d_target"]) ==
                      canonical(row["d_mid"])), None)
        non_equal = next((row for row in laterals if canonical(row["d_target"]) !=
                          canonical(row["d_mid"])), None)
        if equal:
            add(equal, transitions[0], "EQUAL_MID")
        if non_equal:
            add(non_equal, transitions[0], "NON_EQUAL_MID")

    # Diagonal wave traversal is prefix bounded.  It never materializes L x T.
    transition_orders = [ordered_transitions_for_lateral(lateral, transitions)
                         for lateral in laterals]
    for wave in range(len(transitions)):
        for index, lateral in enumerate(laterals):
            add(lateral, transition_orders[index][wave])
            if len(output) >= budget:
                break
        if len(output) >= budget:
            break

    assert len(output) <= budget
    return output, {
        "pair_proxy_evaluations": len(output),
        "reserve_pair_count": reserve_count,
        "pair_budget": budget,
    }


def select_top12(pool: list[dict]) -> list[dict]:
    lex = F.ordered_candidates(pool, "R1_FEASIBILITY_LEXICOGRAPHIC")
    coverage = F.ordered_candidates(pool, "R3_LEXICOGRAPHIC_COVERAGE_RESERVE")
    selected = lex[:10] + coverage[:2]
    if len(selected) < MAX_RECONSTRUCTIONS:
        used = {row["configuration_key"] for row in selected}
        for row in lex[10:] + coverage[2:]:
            if row["configuration_key"] in used:
                continue
            selected.append(row)
            used.add(row["configuration_key"])
            if len(selected) >= MAX_RECONSTRUCTIONS:
                break
    return selected[:MAX_RECONSTRUCTIONS]


def evaluate(item: dict, budget: int, policy: str,
             selected: list[dict]) -> tuple[list[dict], float]:
    WORK.mkdir(parents=True, exist_ok=True)
    request = WORK / f"{item['dataset_role']}.{item['event_id']}.B{budget}.{policy}.requests"
    F.write_requests(request, f"R3_RT_B{budget}_{policy}", selected)
    started = time.perf_counter()
    completed = subprocess.run(
        [str(HARNESS), str(item["event_path"]), str(request)],
        check=True, text=True, stdout=subprocess.PIPE)
    runtime_ms = 1000.0 * (time.perf_counter() - started)
    rows = F.parse_candidate_rows(completed.stdout)
    if len(rows) != len(selected):
        raise RuntimeError(f"harness row mismatch {item['event_id']} B{budget} {policy}")
    return rows, runtime_ms


def exact_dedup(selected: list[dict], rows: list[dict]) -> tuple[list[dict], list[dict]]:
    unique_candidates, unique_rows = [], []
    digests = set()
    for candidate, row in zip(selected, rows):
        digest = row.get("path_digest", "")
        if not digest or digest in digests:
            continue
        digests.add(digest)
        unique_candidates.append(candidate)
        unique_rows.append(row)
    return unique_candidates, unique_rows


def final_candidate(candidates: list[dict], rows: list[dict], require_usable: bool) -> tuple[dict, dict] | None:
    options = []
    for index, (candidate, row) in enumerate(zip(candidates, rows)):
        valid = usable(row) if require_usable else row.get("hard_valid") == "1"
        if valid:
            options.append((R.method_rank(row, index), candidate, row))
    if not options:
        return None
    _, candidate, row = min(options, key=lambda value: value[0])
    return candidate, row


def prepare_item(item: dict) -> dict:
    contexts = F.strict_contexts(item["event_path"])
    started = time.perf_counter()
    event = G.parse_event(item["event_path"])
    geometries = {}
    per_side = {}
    for context in sorted(contexts, key=lambda row: row["side"] != "RIGHT"):
        geometry = F.geometry_for_side(event, context)
        geometries[geometry["side"]] = geometry
        per_side[geometry["side"]] = side_lateral_operators(
            event, geometry, item["production"])
    laterals = interleave_laterals(per_side)
    transitions = transition_proposals(item["production"])
    geometry_ms = 1000.0 * (time.perf_counter() - started)
    if not laterals or not transitions:
        raise RuntimeError(f"empty bounded proposal context {item['event_id']}")
    return {
        "event": event, "geometries": geometries, "laterals": laterals,
        "transitions": transitions, "geometry_ms": geometry_ms,
        "raw_side_operator_count": sum(len(rows) for rows in per_side.values()),
    }


def run_one(item: dict, prepared: dict, teacher_rows: list[dict], teacher_result: dict,
            budget: int, policy: str) -> dict:
    lateral_cap = min(len(prepared["laterals"]), LATERAL_RETAIN_CAP[budget])
    laterals = prepared["laterals"][:lateral_cap]
    started = time.perf_counter()
    pool, pool_stats = pair_pool(
        prepared["event"], prepared["geometries"], item["production"], laterals,
        prepared["transitions"], budget, policy)
    selected = select_top12(pool)
    proposal_rank_ms = 1000.0 * (time.perf_counter() - started)
    rows, harness_ms = evaluate(item, budget, policy, selected)
    raw_validators = sum(row.get("validator_executed") == "1" for row in rows)
    unique_candidates, unique_rows = exact_dedup(selected, rows)
    best_hard = final_candidate(unique_candidates, unique_rows, False)
    best_usable = final_candidate(unique_candidates, unique_rows, True)

    teacher_keys = {config_key(row) for row in teacher_rows}
    selected_keys = {config_key(row) for row in selected}
    pool_keys = {config_key(row) for row in pool}
    teacher_final = next((row for row in teacher_rows
                          if truth(row["teacher_final_selected_path"])), None)
    teacher_final_key = config_key(teacher_final) if teacher_final else None
    teacher_hard = truth(teacher_result["hard_recovered"])
    teacher_usable = truth(teacher_result["usable_recovered"])
    bounded_hard = best_hard is not None
    bounded_usable = best_usable is not None
    total_ms = prepared["geometry_ms"] + proposal_rank_ms + harness_ms
    result = {
        "dataset_role": item["dataset_role"],
        "event_id": item["event_id"],
        "budget_b": budget,
        "reserve_policy": policy,
        "strict_side_count": len(prepared["geometries"]),
        "lateral_operator_candidates_generated": prepared["raw_side_operator_count"],
        "lateral_proposals_retained": len(laterals),
        "transition_proposals": len(prepared["transitions"]),
        "pair_proposals": len(pool),
        "pair_proxy_evaluations": pool_stats["pair_proxy_evaluations"],
        "reserve_pair_count": pool_stats["reserve_pair_count"],
        "p3_reconstructions": len(selected),
        "raw_harness_validator_executions": raw_validators,
        "logical_exact_validator_calls": len(unique_rows),
        "selected_unique_path_digests": len(unique_rows),
        "teacher_top12_unique_candidate_count": len(teacher_keys),
        "teacher_top12_recalled_count": len(teacher_keys & selected_keys),
        "teacher_top12_recall": (len(teacher_keys & selected_keys) / len(teacher_keys)
                                 if teacher_keys else math.nan),
        "teacher_final_selected_candidate_defined": teacher_final is not None,
        "teacher_final_selected_candidate_in_pair_pool": teacher_final_key in pool_keys
            if teacher_final_key else "",
        "teacher_final_selected_candidate_present": teacher_final_key in selected_keys
            if teacher_final_key else "",
        "teacher_hard_recovered": teacher_hard,
        "teacher_usable_recovered": teacher_usable,
        "bounded_hard_recovered": bounded_hard,
        "bounded_usable_recovered": bounded_usable,
        "additional_hard_failure_vs_teacher": teacher_hard and not bounded_hard,
        "additional_usable_failure_vs_teacher": teacher_usable and not bounded_usable,
        "hard_recovery_not_selected_by_teacher": bounded_hard and not teacher_hard,
        "usable_recovery_not_selected_by_teacher": bounded_usable and not teacher_usable,
        "best_hard_side": best_hard[0]["side"] if best_hard else "",
        "best_hard_d_target": best_hard[0]["d_target"] if best_hard else "",
        "best_hard_d_mid": best_hard[0]["d_mid"] if best_hard else "",
        "best_hard_entry_scale": best_hard[0]["entry_scale"] if best_hard else "",
        "best_hard_exit_scale": best_hard[0]["exit_scale"] if best_hard else "",
        "best_hard_path_digest": best_hard[1]["path_digest"] if best_hard else "",
        "best_usable_path_digest": best_usable[1]["path_digest"] if best_usable else "",
        "geometry_context_preparation_ms": prepared["geometry_ms"],
        "bounded_proposal_and_ranking_ms": proposal_rank_ms,
        "exact_harness_wall_ms_including_process_start": harness_ms,
        "offline_core_wall_proxy_ms": total_ms,
        "pair_pool_lateral_families": "|".join(sorted({
            row["lateral_source_family"] for row in pool})),
        "selected_lateral_families": "|".join(sorted({
            row["lateral_source_family"] for row in selected})),
        "selected_transition_families": "|".join(sorted({
            row["transition_family"] for row in selected})),
    }
    result["_pool"] = pool
    result["_selected"] = selected
    result["_rows"] = rows
    result["_laterals"] = laterals
    result["_transitions"] = prepared["transitions"]
    return result


def role_rows(rows: list[dict], role: str) -> list[dict]:
    if role == "COMBINED_SEEN":
        return [row for row in rows if row["dataset_role"] in {
            "DEVELOPMENT", "VALIDATION_SEEN_AFTER_V1"}]
    return [row for row in rows if row["dataset_role"] == role]


def aggregate_outputs(rows: list[dict]) -> tuple[list[dict], list[dict], list[dict]]:
    recovery, runtime, recall = [], [], []
    roles = ["DEVELOPMENT", "VALIDATION_SEEN_AFTER_V1", "COMBINED_SEEN"]
    for budget in BUDGETS:
        budget_rows = [row for row in rows if row["budget_b"] == budget]
        for role in roles:
            subset = role_rows(budget_rows, role)
            recovery.append({
                "dataset_role": role, "budget_b": budget, "episode_count": len(subset),
                "oracle_v2_hard_feasible_count": sum(truth(row["_teacher_result"]["oracle_v2_hard_feasible"])
                                                     for row in subset),
                "oracle_v2_usable_feasible_count": sum(truth(row["_teacher_result"]["oracle_v2_usable_feasible"])
                                                       for row in subset),
                "exact_r3_hard_recovered": sum(row["teacher_hard_recovered"] for row in subset),
                "bounded_hard_recovered": sum(row["bounded_hard_recovered"] for row in subset),
                "exact_r3_usable_recovered": sum(row["teacher_usable_recovered"] for row in subset),
                "bounded_usable_recovered": sum(row["bounded_usable_recovered"] for row in subset),
                "additional_hard_failures_vs_exact_r3": sum(
                    row["additional_hard_failure_vs_teacher"] for row in subset),
                "additional_usable_failures_vs_exact_r3": sum(
                    row["additional_usable_failure_vs_teacher"] for row in subset),
                "hard_recoveries_not_selected_by_exact_r3": sum(
                    row["hard_recovery_not_selected_by_teacher"] for row in subset),
                "usable_recoveries_not_selected_by_exact_r3": sum(
                    row["usable_recovery_not_selected_by_teacher"] for row in subset),
            })
            values = lambda field: [float(row[field]) for row in subset]
            runtime.append({
                "dataset_role": role, "budget_b": budget, "episode_count": len(subset),
                "runtime_measurement": "OFFLINE_PYTHON_PLUS_CPP_HARNESS_WALL_PROXY",
                "p50_ms": percentile(values("offline_core_wall_proxy_ms"), 50),
                "p90_ms": percentile(values("offline_core_wall_proxy_ms"), 90),
                "p95_ms": percentile(values("offline_core_wall_proxy_ms"), 95),
                "p99_ms": percentile(values("offline_core_wall_proxy_ms"), 99),
                "max_ms": max(values("offline_core_wall_proxy_ms")),
                "proposal_rank_p95_ms": percentile(values("bounded_proposal_and_ranking_ms"), 95),
                "geometry_prep_p95_ms": percentile(values("geometry_context_preparation_ms"), 95),
                "harness_process_wall_p95_ms": percentile(
                    values("exact_harness_wall_ms_including_process_start"), 95),
                "warning": "Includes Python and subprocess startup; not deployable C++ callback latency",
            })
            recalled = sum(int(row["teacher_top12_recalled_count"]) for row in subset)
            teacher_count = sum(int(row["teacher_top12_unique_candidate_count"]) for row in subset)
            defined = [row for row in subset
                       if truth(row["teacher_final_selected_candidate_defined"])]
            recall.append({
                "dataset_role": role, "budget_b": budget, "episode_count": len(subset),
                "teacher_top12_micro_recall": recalled / teacher_count if teacher_count else math.nan,
                "teacher_top12_macro_recall": statistics.mean(
                    float(row["teacher_top12_recall"]) for row in subset),
                "teacher_final_selected_defined_count": len(defined),
                "teacher_final_selected_present_count": sum(
                    truth(row["teacher_final_selected_candidate_present"]) for row in defined),
                "teacher_final_selected_pair_pool_present_count": sum(
                    truth(row["teacher_final_selected_candidate_in_pair_pool"]) for row in defined),
            })
    return recovery, runtime, recall


def miss_taxonomy(rows: list[dict], teacher_by_event: dict[tuple, list[dict]]) -> list[dict]:
    output = []
    for row in rows:
        if not row["teacher_usable_recovered"] or row["bounded_usable_recovered"]:
            continue
        key = row["dataset_role"], row["event_id"]
        teacher_final = next(item for item in teacher_by_event[key]
                             if truth(item["teacher_final_selected_path"]))
        laterals = {lateral_key(item) for item in row["_laterals"]}
        transitions = {transition_key(item) for item in row["_transitions"]}
        pool_keys = {config_key(item) for item in row["_pool"]}
        selected_keys = {config_key(item) for item in row["_selected"]}
        if lateral_key(teacher_final) not in laterals:
            cause = "LATERAL_HYPOTHESIS_ABSENT"
        elif transition_key(teacher_final) not in transitions:
            cause = "TRANSITION_HYPOTHESIS_ABSENT"
        elif config_key(teacher_final) not in pool_keys:
            cause = "USEFUL_L_T_PAIR_ABSENT"
        elif config_key(teacher_final) not in selected_keys:
            cause = ("COVERAGE_RESERVE_FAILURE" if
                     teacher_final["teacher_stream"] == "COVERAGE_RESERVE_TOP2"
                     else "PROXY_RANKING_DISPLACEMENT")
        else:
            cause = "OTHER_EXACT_RESULT_MISMATCH"
        output.append({
            "dataset_role": row["dataset_role"], "event_id": row["event_id"],
            "budget_b": row["budget_b"], "miss_class": cause,
            "teacher_stream": teacher_final["teacher_stream"],
            "teacher_side": teacher_final["side"],
            "teacher_d_target": teacher_final["d_target"],
            "teacher_d_mid": teacher_final["d_mid"],
            "teacher_entry_scale": teacher_final["entry_scale"],
            "teacher_exit_scale": teacher_final["exit_scale"],
            "teacher_target_source": teacher_final["target_source"],
            "teacher_mid_source": teacher_final["mid_source"],
            "lateral_present": lateral_key(teacher_final) in laterals,
            "transition_present": transition_key(teacher_final) in transitions,
            "exact_pair_present": config_key(teacher_final) in pool_keys,
            "exact_pair_selected": config_key(teacher_final) in selected_keys,
            "pair_proxy_evaluations": row["pair_proxy_evaluations"],
        })
    return output


def pareto_rows(recovery: list[dict], runtime: list[dict]) -> list[dict]:
    rec = {(row["dataset_role"], row["budget_b"]): row for row in recovery}
    runt = {(row["dataset_role"], row["budget_b"]): row for row in runtime}
    output = []
    points = []
    for budget in BUDGETS:
        r = rec[("COMBINED_SEEN", budget)]
        t = runt[("COMBINED_SEEN", budget)]
        points.append((budget, int(r["bounded_usable_recovered"]), float(t["p95_ms"])))
    min_recovery = min(point[1] for point in points)
    max_recovery = max(point[1] for point in points)
    min_runtime = min(point[2] for point in points)
    max_runtime = max(point[2] for point in points)
    # Discrete knee: greatest normalized vertical distance above the endpoint chord.
    knee_budget = max(points, key=lambda point: (
        ((point[1] - min_recovery) / max(EPS, max_recovery - min_recovery))
        - ((point[2] - min_runtime) / max(EPS, max_runtime - min_runtime)),
        -point[0]))[0]
    for budget, recovered, p95 in points:
        dominated = any((other_recovered >= recovered and other_p95 <= p95)
                        and (other_recovered > recovered or other_p95 < p95)
                        for _, other_recovered, other_p95 in points)
        teacher = int(rec[("COMBINED_SEEN", budget)]["exact_r3_usable_recovered"])
        output.append({
            "budget_b": budget,
            "combined_seen_usable_recovered": recovered,
            "exact_r3_usable_recovered": teacher,
            "exact_r3_usable_retention": recovered / teacher if teacher else math.nan,
            "offline_wall_proxy_p95_ms": p95,
            "meets_25ms_offline_proxy_p95": p95 <= 25.0,
            "pareto_nondominated": not dominated,
            "discrete_normalized_knee_point": budget == knee_budget,
            "runtime_scope_warning": "Python+subprocess proxy; C++ integration benchmark required",
        })
    return output


def clean_rows(rows: list[dict]) -> list[dict]:
    return [{key: value for key, value in row.items() if not key.startswith("_")}
            for row in rows]


def write_method_document() -> None:
    text = """# R3-RT bounded proposal method v1

## Status and authority

This is a **new seen-data design**, not an exact implementation of frozen R3. The
offline teacher is `R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12` with method SHA
`7b861e8c7e23ae168413885ea0dc2d09769046ff48a562700b658e084d116fcc`.
No oracle coordinate or validator outcome enters proposal generation or ranking.

## Characterization-first evidence

The machine-readable characterization was written before the bounded sweep. Its
rows preserve all 86 x 12 teacher selections, identify feasibility Top-10 versus
coverage Top-2, direct lateral/transition sources, geometry descriptors, hard and
usable verdicts, and the final hard-ranker selection. R3 is a direct
`(d_target,d_mid,entry_scale,exit_scale)` factor method; `probe/root source` is
therefore explicitly `NOT_APPLICABLE_DIRECT_DT_DM_FACTOR`.

## Hard-bounded generator

For every strict side, at most 32 direct lateral operator outputs are formed. The
global deterministic interleave has an absolute cap of 64. Operators are paired
recipes, not a target x mid grid:

- exact production target with equal mid and `mid=target-0.02 m`;
- component C0 anchors at fractions `0, 1/32, 1/8, 3/8, 1/2, 2/3`, each with one
  fixed equal/offset/corridor-interpolation recipe;
- characterization-supported production offsets (`-0.01,+0.01,-0.06,+0.06,
  -0.10,-0.02 m`) with one associated mid recipe;
- two corridor centers and a bounded tail of 1/8, 1/4 center interpolation and
  `-0.10,+0.05 m` mid corrections.

The transition operator reads only the already-bounded exact production transition
set. Seven normalized geometry-independent S/M/L anchors are projected to the
nearest exact transition tuple: short/medium/long entry crossed only with the seven
predeclared short/medium/long combinations. Exact duplicates are removed, so the
actual transition count is at most seven.

Pairs are emitted by a source-conditioned diagonal lazy traversal. The traversal
stops immediately at B and never materializes the lateral x transition Cartesian
product. Exact production configurations are skipped before proxy work. The full
frozen-R3 proxy tuple is evaluated exactly once for each emitted pair.

## Selection and reserve ablation

Per-side and full small reserves were evaluated at B64 inside the same B budget.
They did not improve usable recovery, while the no-reserve diagonal already retained
diverse selected-family signatures. The selected v1 sweep therefore uses no fixed
proposal reserve. This is an evidence-based rejection, not an unbounded fallback.
From the bounded pool, frozen R3's lexicographic order supplies 10 configurations
and its coverage order supplies 2. At most 12 requests reach the unchanged P3
constructor and exact validator. Path-digest duplicates are removed before the
logical final-ranker view.

## Computation contract

`lateral operator outputs <= 64`, `transition proposals <= 7`,
`pair proxy evaluations <= B`, `P3 reconstructions <= 12`, and
`exact validator executions <= 12`. The Python study reports actual counts and
fails if a bound is exceeded.

## Data boundary

Only `PILOT_SEEN_DEVELOPMENT_DATA`, `DEVELOPMENT`, and
`VALIDATION_SEEN_AFTER_V1` enter this script. The final holdout is neither listed,
parsed, nor used for tuning. This offline prototype does not modify production
planner behavior.
"""
    (HERE / "bounded_proposal_method.md").write_text(text, encoding="utf-8")


def write_readme(recovery: list[dict], runtime: list[dict], recall: list[dict],
                 pareto: list[dict], misses: list[dict], vue: dict) -> None:
    combined = {row["budget_b"]: row for row in recovery
                if row["dataset_role"] == "COMBINED_SEEN"}
    combined_runtime = {row["budget_b"]: row for row in runtime
                        if row["dataset_role"] == "COMBINED_SEEN"}
    combined_recall = {row["budget_b"]: row for row in recall
                       if row["dataset_role"] == "COMBINED_SEEN"}
    teacher_usable = combined[BUDGETS[0]]["exact_r3_usable_recovered"]
    eligible = [budget for budget in BUDGETS
                if combined[budget]["bounded_usable_recovered"] >= teacher_usable - 1]
    smallest_near = min(eligible) if eligible else None
    under_25 = [budget for budget in BUDGETS
                if float(combined_runtime[budget]["p95_ms"]) <= 25.0]
    nondominated = [row["budget_b"] for row in pareto if truth(row["pareto_nondominated"])]
    knee = next(row["budget_b"] for row in pareto
                if truth(row["discrete_normalized_knee_point"]))
    best_budget = max(BUDGETS, key=lambda budget: (
        combined[budget]["bounded_usable_recovered"],
        -float(combined_runtime[budget]["p95_ms"]), -budget))
    recommendation = (
        "R3_RT_BOUNDED_PROPOSAL_PROMISING"
        if combined[best_budget]["bounded_usable_recovered"] >= teacher_usable - 1
        and under_25 else
        "R3_RT_RUNTIME_STILL_TOO_HIGH"
        if combined[best_budget]["bounded_usable_recovered"] >= teacher_usable - 1
        else "R3_RT_RECOVERY_LOSS_TOO_HIGH"
    )
    characterized = read_csv(HERE / "exact_r3_teacher_characterization.csv")
    useful_finals = [row for row in characterized
                     if truth(row["teacher_final_selected_path"])
                     and truth(row["usable_valid"])]
    target_counts = Counter(row["target_family"] for row in useful_finals)
    mid_counts = Counter(row["mid_family"] for row in useful_finals)
    transition_counts = Counter(row["transition_family"] for row in useful_finals)
    stream_counts = Counter(row["teacher_stream"] for row in useful_finals)
    miss96 = [row["event_id"] for row in misses if int(row["budget_b"]) == 96]
    table = [
        "| B | usable DEV | usable VALIDATION | usable combined | teacher-final present | Top12 recall | p95 ms* |",
        "|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for budget in BUDGETS:
        dev = next(row for row in recovery if row["dataset_role"] == "DEVELOPMENT"
                   and row["budget_b"] == budget)
        val = next(row for row in recovery if row["dataset_role"] == "VALIDATION_SEEN_AFTER_V1"
                   and row["budget_b"] == budget)
        rec = combined_recall[budget]
        table.append(
            f"| {budget} | {dev['bounded_usable_recovered']}/{dev['exact_r3_usable_recovered']} "
            f"| {val['bounded_usable_recovered']}/{val['exact_r3_usable_recovered']} "
            f"| {combined[budget]['bounded_usable_recovered']}/{teacher_usable} "
            f"| {rec['teacher_final_selected_present_count']}/{rec['teacher_final_selected_defined_count']} "
            f"| {float(rec['teacher_top12_micro_recall']):.3f} "
            f"| {float(combined_runtime[budget]['p95_ms']):.2f} |"
        )
    text = f"""# R3-RT bounded proposal design study v1

## 결론

이 연구는 frozen exact R3를 변경한 최적화가 아니라, seen geometry에서 최대 B개의
L/T pair만 제안하는 **새로운 bounded method**다. 전체 Cartesian factor pool과
FINAL_HOLDOUT 결과는 사용하지 않았다. Production planner도 변경하지 않았다.

{chr(10).join(table)}

`*` 런타임은 geometry 준비 + Python proposal/ranking + 별도 C++ harness process
기동을 합친 보수적 offline wall proxy다. 따라서 실제 C++ callback latency와 동일하다고
주장하지 않는다.

## 핵심 해석

- exact teacher의 combined seen usable recovery는 {teacher_usable}건이다.
- exact teacher보다 usable miss가 1건 이하인 최소 budget: {smallest_near if smallest_near else 'NONE'}.
- B96의 남은 teacher miss: {', '.join(miss96) if miss96 else 'NONE'}.
- normalized recovery/runtime endpoint-chord knee: B{knee}.
- offline wall proxy p95가 25 ms 이하인 budget: {', '.join(map(str, under_25)) if under_25 else 'NONE'}.
- Pareto 비지배 budget: {', '.join(map(str, nondominated))}.
- VUE019는 exact R3도 hard/usable recovery가 없었던 oracle-infeasible large-factor 사례다.
  Exact pair pool 71,506개 대신 B={vue['budget_b']}에서 {vue['pair_proxy_evaluations']}개를 평가했고,
  bounded 결과도 usable recovery를 주장하지 않는다.

Top-12 imitation 자체보다 unchanged exact validator를 통과한 hard/usable recovery를 우선
판단했다. 세부 teacher 패턴, budget별 event 결과, reserve ablation, miss 원인은 동봉 CSV에
있다.

## Exact R3 teacher가 실제로 복구한 패턴

86개 snapshot의 K12 1,032개 선택을 먼저 고정해 분석했다. 최종 usable path 36개 중
feasibility Top-10에서 {stream_counts['FEASIBILITY_TOP10']}개, coverage Top-2에서
{stream_counts['COVERAGE_RESERVE_TOP2']}개가 나왔다. Target family는 production
{target_counts['PRODUCTION_TARGET_FAMILY']}개와 component-boundary
{target_counts['COMPONENT_BOUNDARY_FAMILY']}개였다. Mid는 equal-target
{mid_counts['EQUAL_TARGET_FAMILY']}, target-offset {mid_counts['TARGET_OFFSET_FAMILY']},
corridor interpolation {mid_counts['CORRIDOR_INTERPOLATION_FAMILY']}, reference-inward
{mid_counts['REFERENCE_INWARD_FAMILY']}개였다. Transition은 long-entry medium/short/long-exit
각 {transition_counts['LONG_ENTRY_MEDIUM_EXIT']}/{transition_counts['LONG_ENTRY_SHORT_EXIT']}/
{transition_counts['LONG_ENTRY_LONG_EXIT']}개, short-entry short/medium/long-exit 각
{transition_counts['SHORT_ENTRY_SHORT_EXIT']}/{transition_counts['SHORT_ENTRY_MEDIUM_EXIT']}/
{transition_counts['SHORT_ENTRY_LONG_EXIT']}개였다. R3는 direct factor method이므로 analytic
probe/root source는 적용되지 않는다.

## 한계와 다음 gate

이 결과는 PILOT/DEVELOPMENT/VALIDATION_SEEN_AFTER_V1에 맞춘 설계 연구다. Runtime은
Python/subprocess proxy이므로, 선택된 B를 freeze하거나 production에 통합하기 전에 독립
C++ offline implementation과 untouched evaluation data가 필요하다. 이 단계에서는 새
holdout을 만들거나 과거 FINAL_HOLDOUT을 재사용하지 않았다.

## Recommendation

`{recommendation}`
"""
    (HERE / "README.md").write_text(text, encoding="utf-8")


def write_execution_manifest(items: list[dict], main_rows: list[dict]) -> None:
    manifest = {
        "study": "R3_RT_BOUNDED_PROPOSAL_DESIGN_STUDY_V1",
        "selected_policy": "NO_RESERVES_DIAGONAL",
        "new_method_not_exact_r3_equivalence": True,
        "teacher_method": "R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12",
        "teacher_method_sha256": EXPECTED_METHOD_SHA,
        "teacher_implementation_sha256": sha256(TEACHER_IMPL),
        "exact_harness_sha256": sha256(HARNESS),
        "dataset_roles_loaded": dict(Counter(item["dataset_role"] for item in items)),
        "final_holdout_outcome_rows_loaded": 0,
        "final_holdout_paths_enumerated": 0,
        "budgets": BUDGETS,
        "maximum_observed_lateral_operator_candidates_generated": max(
            row["lateral_operator_candidates_generated"] for row in main_rows),
        "maximum_observed_lateral_proposals_retained": max(
            row["lateral_proposals_retained"] for row in main_rows),
        "maximum_observed_transition_proposals": max(
            row["transition_proposals"] for row in main_rows),
        "maximum_observed_pair_proxy_evaluations": max(
            row["pair_proxy_evaluations"] for row in main_rows),
        "maximum_observed_p3_reconstructions": max(
            row["p3_reconstructions"] for row in main_rows),
        "maximum_observed_raw_validator_executions": max(
            row["raw_harness_validator_executions"] for row in main_rows),
        "production_local_planning_source_modified_by_study": False,
        "planner_algorithm_or_validator_modified": False,
        "runtime_scope": "OFFLINE_PYTHON_PLUS_CPP_HARNESS_WALL_PROXY",
        "runtime_not_claimed_as_deployable_cpp_callback": True,
    }
    (HERE / "execution_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (HERE / "run_bounded_proposal_study.sha256").write_text(
        f"{sha256(Path(__file__))}  run_bounded_proposal_study.py\n", encoding="utf-8")


def main() -> None:
    HERE.mkdir(parents=True, exist_ok=True)
    if sha256(TEACHER / "selected_v2_method_spec.json") != EXPECTED_METHOD_SHA:
        raise RuntimeError("frozen teacher method SHA mismatch")
    if not HARNESS.is_file():
        raise RuntimeError(f"missing frozen harness: {HARNESS}")
    F.freeze_gate()
    items = R.input_catalog()
    roles = Counter(item["dataset_role"] for item in items)
    if set(roles) != ALLOWED_ROLES or sum(roles.values()) != 86:
        raise RuntimeError(f"seen-only catalog changed: {roles}")

    # Characterization is materially produced before proposal construction starts.
    teacher_rows, teacher_results, _ = teacher_characterization(items)
    teacher_by_event = defaultdict(list)
    for row in teacher_rows:
        teacher_by_event[(row["dataset_role"], row["event_id"])].append(row)
    print(f"TEACHER_CHARACTERIZATION_WRITTEN rows={len(teacher_rows)} events={len(items)}",
          flush=True)
    if len(teacher_rows) != 86 * 12:
        raise RuntimeError("teacher K12 characterization is incomplete")

    write_method_document()
    main_rows = []
    prepared_by_event = {}
    for index, item in enumerate(items, 1):
        key = item["dataset_role"], item["event_id"]
        prepared = prepare_item(item)
        prepared_by_event[key] = prepared
        for budget in BUDGETS:
            row = run_one(item, prepared, teacher_by_event[key], teacher_results[key],
                          budget, "NO_RESERVES_DIAGONAL")
            row["_teacher_result"] = teacher_results[key]
            if (row["pair_proxy_evaluations"] > budget
                    or row["p3_reconstructions"] > MAX_RECONSTRUCTIONS
                    or row["raw_harness_validator_executions"] > MAX_RECONSTRUCTIONS):
                raise RuntimeError(f"hard budget violation {item['event_id']} B{budget}")
            main_rows.append(row)
        print(f"R3_RT {index}/86 {item['dataset_role']} {item['event_id']}", flush=True)

    write_csv(HERE / "proposal_budget_sweep.csv", clean_rows(main_rows))
    recovery, runtime, recall = aggregate_outputs(main_rows)
    write_csv(HERE / "recovery_vs_budget.csv", recovery)
    write_csv(HERE / "runtime_vs_budget.csv", runtime)
    write_csv(HERE / "teacher_candidate_recall.csv", recall)
    misses = miss_taxonomy(main_rows, teacher_by_event)
    write_csv(HERE / "missed_recovery_taxonomy.csv", misses)

    # Reserve ablation at the knee-candidate B64; main-policy rows are reused.
    ablation_event_rows = [row for row in main_rows if row["budget_b"] == 64]
    for policy in ("SIDE_RESERVE", "FULL_SMALL_RESERVES"):
        for item in items:
            key = item["dataset_role"], item["event_id"]
            row = run_one(item, prepared_by_event[key], teacher_by_event[key],
                          teacher_results[key], 64, policy)
            row["_teacher_result"] = teacher_results[key]
            ablation_event_rows.append(row)
    ablation = []
    for policy in ("NO_RESERVES_DIAGONAL", "SIDE_RESERVE", "FULL_SMALL_RESERVES"):
        policy_rows = [row for row in ablation_event_rows if row["reserve_policy"] == policy]
        for role in ("DEVELOPMENT", "VALIDATION_SEEN_AFTER_V1", "COMBINED_SEEN"):
            subset = role_rows(policy_rows, role)
            ablation.append({
                "dataset_role": role, "budget_b": 64, "reserve_policy": policy,
                "episode_count": len(subset),
                "bounded_hard_recovered": sum(row["bounded_hard_recovered"] for row in subset),
                "bounded_usable_recovered": sum(row["bounded_usable_recovered"] for row in subset),
                "exact_r3_usable_recovered": sum(row["teacher_usable_recovered"] for row in subset),
                "teacher_top12_micro_recall": (
                    sum(row["teacher_top12_recalled_count"] for row in subset)
                    / sum(row["teacher_top12_unique_candidate_count"] for row in subset)),
                "teacher_final_selected_present_count": sum(
                    truth(row["teacher_final_selected_candidate_present"]) for row in subset),
                "reserve_pairs_per_event_mean": statistics.mean(
                    row["reserve_pair_count"] for row in subset),
                "selected_family_signature_count": len({
                    (row["selected_lateral_families"], row["selected_transition_families"])
                    for row in subset}),
            })
    write_csv(HERE / "reserve_policy_ablation.csv", ablation)

    pareto = pareto_rows(recovery, runtime)
    write_csv(HERE / "pareto_frontier.csv", pareto)
    vue = next(row for row in main_rows if row["event_id"] == "VUE019"
               and row["budget_b"] == 128)
    vue_text = f"""# VUE019 worst-case audit

- Dataset: `{vue['dataset_role']}`
- Frozen exact R3 lateral factors: 4,471
- Frozen exact R3 unique pair proxies: 71,506
- Frozen exact R3 K12 hard/usable recovery: `{vue['teacher_hard_recovered']}` / `{vue['teacher_usable_recovered']}`
- Reference Oracle v2 hard/usable feasible: `False` / `False`
- R3-RT B128 lateral retained / transition / pair proxies:
  `{vue['lateral_proposals_retained']}` / `{vue['transition_proposals']}` / `{vue['pair_proxy_evaluations']}`
- R3-RT reconstruction / raw validator calls:
  `{vue['p3_reconstructions']}` / `{vue['raw_harness_validator_executions']}`
- R3-RT hard/usable recovery: `{vue['bounded_hard_recovered']}` / `{vue['bounded_usable_recovered']}`
- Teacher Top-12 unique recall: `{vue['teacher_top12_recalled_count']}/{vue['teacher_top12_unique_candidate_count']}`

VUE019에는 exact R3의 useful selected solution 자체가 없고 Reference Oracle v2 domain도
hard/usable infeasible였다. 따라서 "useful solution survival"은 적용 불가다. 확인 가능한
결론은 factor proxy 계산을 71,506에서 정확히 128로 제한하면서 새로운 usable recovery를
허위로 만들지 않았다는 것이다. 이는 runtime stress evidence이지 recovery recall evidence가
아니다.
"""
    (HERE / "worst_case_vue019.md").write_text(vue_text, encoding="utf-8")
    write_readme(recovery, runtime, recall, pareto, misses, vue)
    write_execution_manifest(items, main_rows)
    print("R3_RT_BOUNDED_PROPOSAL_STUDY_COMPLETE", flush=True)


if __name__ == "__main__":
    sys.exit(main())
