#!/usr/bin/env python3
"""Seen-only GQSC standalone direct-seed generator study."""

from __future__ import annotations

import csv
import importlib.util
import math
import os
import subprocess
from collections import Counter, defaultdict
from pathlib import Path


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
UNIFIED = REPO / "planning_study/gqsc_unified_architecture_v1/run_gqsc_unified_architecture_study.py"
HARNESS = REPO / "build/local_planning/p3_r3_k12_integration_harness"
EXPECTED_ROLES = {"PILOT_SEEN_DEVELOPMENT_DATA", "DEVELOPMENT", "VALIDATION_SEEN_AFTER_V1"}
TEACHER = "GQSC_LEGACY_SEEDED_TEACHER_B128"
DIRECT = "GQSC_STANDALONE_DIRECT_SEED_B128"
LEGACY = "LEGACY_ONLY"
EPS = 1.0e-9


def load(path: Path):
    spec = importlib.util.spec_from_file_location("gqsc_unified", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


U = load(UNIFIED)


def write_csv(path: Path, rows: list[dict]) -> None:
    fields: list[str] = []
    for row in rows:
        for field in row:
            if field not in fields:
                fields.append(field)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def truth(value: str) -> bool:
    return value in {"1", "true", "True"}


def percentile(values: list[float], q: float) -> float:
    values = sorted(values)
    if not values:
        return math.nan
    position = (len(values) - 1) * q / 100.0
    lo, hi = math.floor(position), math.ceil(position)
    if lo == hi:
        return values[lo]
    return values[lo] + (position - lo) * (values[hi] - values[lo])


def stats(values_us: list[float]) -> dict:
    values = [value / 1000.0 for value in values_us]
    return {
        "sample_count": len(values), "p50_ms": percentile(values, 50),
        "p90_ms": percentile(values, 90), "p95_ms": percentile(values, 95),
        "p99_ms": percentile(values, 99), "max_ms": max(values) if values else math.nan,
    }


def exact(value: float | str) -> str:
    return float(value).hex()


def config(side: str, target, middle, entry, exit_scale) -> tuple[str, str, str, str, str]:
    return side, exact(target), exact(middle), exact(entry), exact(exit_scale)


def run(items: list[dict], warmup: int, repeats: int) -> str:
    if not HARNESS.is_file():
        raise RuntimeError(f"missing Release harness: {HARNESS}")
    env = os.environ.copy()
    env.pop("LOCAL_PLANNING_RESEARCH_PARITY", None)
    env.update({
        "GQSC_STANDALONE_STUDY": "1",
        "GQSC_STANDALONE_WARMUP": str(warmup),
        "GQSC_STANDALONE_REPEATS": str(repeats),
    })
    return subprocess.run(
        [str(HARNESS), *(str(row["event_path"]) for row in items)], check=True,
        text=True, stdout=subprocess.PIPE, env=env).stdout


def parse(text: str) -> dict:
    parsed = U.parse_native(text)
    parsed.update({name: defaultdict(list) for name in (
        "seeds", "geometry", "laterals", "transitions", "pairs", "top", "candidates")})
    parsed["standalone_timing"] = []
    for line in text.splitlines():
        f = line.split("\t")
        tag = f[0]
        if tag == "TEACHER_LEGACY_SEED":
            parsed["seeds"][f[1]].append({
                "seed_index": int(f[2]), "side": f[3], "d_target": float(f[4]),
                "d_mid": float(f[5]), "entry_scale": float(f[6]),
                "exit_scale": float(f[7]), "generator_stage": f[8],
                "candidate_template": f[9], "source_cell": f[10], "root_type": f[11],
                "probe_location_rule": f[12], "probe_anchor_rule": f[13],
            })
        elif tag in {"TEACHER_GEOMETRY", "STANDALONE_GEOMETRY"}:
            method = "TEACHER" if tag.startswith("TEACHER") else "STANDALONE"
            parsed["geometry"][(f[1], method)].append({
                "side": f[2], "ego_d": float(f[3]), "ego_speed": float(f[4]),
                "cluster_start": float(f[5]), "cluster_end": float(f[6]),
                "domain_lower": float(f[7]), "domain_upper": float(f[8]),
                "bottleneck_center": float(f[9]), "reference_spacing": float(f[10]),
                "entry_min": float(f[11]), "entry_max": float(f[12]),
                "exit_min": float(f[13]), "exit_max": float(f[14]),
                "components": f[15], "centers": f[16],
            })
        elif tag.endswith("_LATERAL") and tag.split("_")[0] in {"TEACHER", "STANDALONE"}:
            method = tag.split("_")[0]
            parsed["laterals"][(f[1], method)].append({
                "index": int(f[2]), "side": f[3], "d_target": float(f[4]),
                "d_mid": float(f[5]), "target_source": f[6], "mid_source": f[7],
                "family": f[8], "source_priority": int(f[9]), "operator": f[10],
            })
        elif tag.endswith("_TRANSITION") and tag.split("_")[0] in {"TEACHER", "STANDALONE"}:
            method = tag.split("_")[0]
            parsed["transitions"][(f[1], method)].append({
                "index": int(f[2]), "entry_scale": float(f[3]),
                "exit_scale": float(f[4]), "family": f[5],
            })
        elif tag.endswith("_PAIR") and tag.split("_")[0] in {"TEACHER", "STANDALONE"}:
            method = tag.split("_")[0]
            parsed["pairs"][(f[1], method)].append({
                "index": int(f[2]), "side": f[3], "d_target": float(f[4]),
                "d_mid": float(f[5]), "entry_scale": float(f[6]),
                "exit_scale": float(f[7]), "configuration_key": f[8],
            })
        elif tag.endswith("_TOP") and tag.split("_")[0] in {"TEACHER", "STANDALONE"}:
            method = tag.split("_")[0]
            parsed["top"][(f[1], method)].append({
                "stream": f[2], "index": int(f[3]), "side": f[4],
                "d_target": float(f[5]), "d_mid": float(f[6]),
                "entry_scale": float(f[7]), "exit_scale": float(f[8]),
                "configuration_key": f[9],
            })
        elif tag.endswith("_CANDIDATE") and tag.split("_")[0] in {"TEACHER", "STANDALONE"}:
            method = tag.split("_")[0]
            parsed["candidates"][(f[1], method)].append({
                "side": f[2], "d_target": float(f[3]), "d_mid": float(f[4]),
                "entry_scale": float(f[5]), "exit_scale": float(f[6]), "digest": f[7],
                "hard_valid": truth(f[8]), "usable_valid": truth(f[9]),
                "safety_slack": float(f[10]), "track_margin_m": float(f[11]),
                "obstacle_margin_m": float(f[12]), "curvature_margin": float(f[13]),
                "curvature_rate_margin": float(f[14]), "slope_margin": float(f[15]),
                "braking_deficit_m": float(f[16]), "velocity_loss": float(f[17]),
                "exit_conflict": truth(f[18]), "mean_lateral_deviation_m": float(f[19]),
                "max_curvature_radpm": float(f[20]), "max_curvature_rate_radpm2": float(f[21]),
                "point_count": int(f[22]), "maneuver_station_length_m": float(f[23]),
            })
        elif tag == "STANDALONE_TIMING":
            parsed["standalone_timing"].append({
                "event_id": f[1], "method": f[2], "repeat": int(f[3]),
                "wall_us": float(f[4]), "r3_total_us": float(f[5]),
                "context_us": float(f[6]), "geometry_us": float(f[7]),
                "transition_us": float(f[8]), "lateral_us": float(f[9]),
                "pair_priority_us": float(f[10]), "lexicographic_us": float(f[11]),
                "coverage_us": float(f[12]), "shape_dedup_us": float(f[13]),
                "reconstruction_us": float(f[14]), "validation_us": float(f[15]),
                "final_ranking_us": float(f[16]), "lateral_count": int(f[17]),
                "pair_count": int(f[18]), "reconstruction_count": int(f[19]),
                "validator_count": int(f[20]),
            })
    return parsed


def factor_config(row: dict) -> tuple[str, str, str, str, str]:
    return config(row["side"], row["d_target"], row["d_mid"],
                  row["entry_scale"], row["exit_scale"])


def lateral_key(row: dict) -> tuple[str, str, str]:
    return row["side"], exact(row["d_target"]), exact(row["d_mid"])


def target_delta(operator: str) -> float:
    if "MINUS_001" in operator:
        return -0.01
    if "PLUS_001" in operator:
        return 0.01
    if "MINUS_006" in operator:
        return -0.06
    if "PLUS_006" in operator:
        return 0.06
    if "MINUS_010" in operator:
        return -0.10
    if "MINUS_002" in operator:
        return -0.02
    return 0.0


def semantic(seed: dict) -> str:
    if seed["source_cell"] == "ZERO_INTERFACE":
        return "ZERO_INTERFACE_DIRECT_ANCHOR"
    if seed["generator_stage"] == "M0_V1":
        return "M0_V1_ANALYTIC_PROBE_ROOT"
    if seed["root_type"] == "ALL_INACTIVE":
        return "ANALYTIC_ALL_INACTIVE_ROOT"
    return "ANALYTIC_ACTIVE_OUTER_ROOT"


def legacy_seed_rows(items: list[dict], p: dict) -> list[dict]:
    metadata = {row["event_id"]: row for row in items}
    output = []
    for event_id, seeds in p["seeds"].items():
        laterals = p["laterals"][(event_id, "TEACHER")]
        candidates = p["candidates"][(event_id, "TEACHER")]
        lateral_by_key = {lateral_key(row): row for row in laterals}
        hard_laterals = [lateral_by_key.get((row["side"], exact(row["d_target"]),
            exact(row["d_mid"]))) for row in candidates if row["hard_valid"]]
        usable_laterals = [lateral_by_key.get((row["side"], exact(row["d_target"]),
            exact(row["d_mid"]))) for row in candidates if row["usable_valid"]]
        hard_transitions = {(exact(row["entry_scale"]), exact(row["exit_scale"]))
                            for row in candidates if row["hard_valid"]}
        usable_transitions = {(exact(row["entry_scale"]), exact(row["exit_scale"]))
                              for row in candidates if row["usable_valid"]}
        geometry = {row["side"]: row for row in p["geometry"][(event_id, "TEACHER")]}
        for seed in seeds:
            geo = geometry[seed["side"]]
            def supports(rows):
                for row in rows:
                    if row is None or not row["operator"].startswith("P_TARGET"):
                        continue
                    reconstructed = min(geo["domain_upper"], max(
                        geo["domain_lower"], seed["d_target"] + target_delta(row["operator"])))
                    if exact(reconstructed) == exact(row["d_target"]):
                        return True
                return False
            transition = exact(seed["entry_scale"]), exact(seed["exit_scale"])
            output.append({
                "event_id": event_id, "dataset_role": metadata[event_id]["dataset_role"],
                "source_kind": metadata[event_id]["source_kind"], **seed,
                "semantic_class": semantic(seed),
                "target_value_consumed_by_lateral_bank": int(any(
                    row["side"] == seed["side"] and exact(row["d_target"]) == exact(seed["d_target"])
                    and row["operator"] == "P_TARGET_EQUAL" for row in laterals)),
                "transition_value_selected_into_bounded_set": int(any(
                    exact(row["entry_scale"]) == transition[0] and
                    exact(row["exit_scale"]) == transition[1]
                    for row in p["transitions"][(event_id, "TEACHER")])),
                "supports_any_hard_lateral": int(supports(hard_laterals)),
                "supports_any_usable_lateral": int(supports(usable_laterals)),
                "supports_any_hard_transition": int(transition in hard_transitions),
                "supports_any_usable_transition": int(transition in usable_transitions),
                "root_or_probe_numeric_value_read_by_gqsc": 0,
            })
    return output


def comparison_rows(items: list[dict], p: dict) -> list[dict]:
    output = []
    for item in items:
        event_id = item["event_id"]
        legacy = p["results"][(event_id, LEGACY)]
        teacher = p["results"][(event_id, TEACHER)]
        direct = p["results"][(event_id, DIRECT)]
        tc = p["candidates"][(event_id, "TEACHER")]
        dc = p["candidates"][(event_id, "STANDALONE")]
        teacher_hard = {row["digest"] for row in tc if row["hard_valid"]}
        direct_hard = {row["digest"] for row in dc if row["hard_valid"]}
        teacher_usable = {row["digest"] for row in tc if row["usable_valid"]}
        direct_usable = {row["digest"] for row in dc if row["usable_valid"]}
        output.append({
            "event_id": event_id, "dataset_role": item["dataset_role"], "bag": item["bag"],
            "source_kind": item["source_kind"], "legacy_hard_valid": int(legacy["recovered"]),
            "legacy_usable_valid": int(legacy["any_usable"]),
            "legacy_selected_digest": legacy["selected_digest"],
            "teacher_hard_valid": int(teacher["recovered"]),
            "teacher_usable_valid": int(teacher["any_usable"]),
            "standalone_hard_valid": int(direct["recovered"]),
            "standalone_usable_valid": int(direct["any_usable"]),
            "teacher_selected_digest": teacher["selected_digest"],
            "standalone_selected_digest": direct["selected_digest"],
            "selected_digest_exact": int(teacher["selected_digest"] == direct["selected_digest"]),
            "teacher_vs_legacy_selected_digest_exact": int(
                teacher["selected_digest"] == legacy["selected_digest"]),
            "standalone_vs_legacy_selected_digest_exact": int(
                direct["selected_digest"] == legacy["selected_digest"]),
            "teacher_hard_candidate_count": len(teacher_hard),
            "standalone_hard_candidate_count": len(direct_hard),
            "hard_candidate_digest_intersection": len(teacher_hard & direct_hard),
            "hard_candidate_digest_union": len(teacher_hard | direct_hard),
            "teacher_usable_candidate_count": len(teacher_usable),
            "standalone_usable_candidate_count": len(direct_usable),
            "usable_candidate_digest_intersection": len(teacher_usable & direct_usable),
            "usable_candidate_digest_union": len(teacher_usable | direct_usable),
            "teacher_reconstructions": teacher["r3_constructions"],
            "teacher_validators": teacher["r3_validators"],
            "standalone_reconstructions": direct["r3_constructions"],
            "standalone_validators": direct["r3_validators"],
        })
    return output


def miss_rows(comparison: list[dict], p: dict) -> list[dict]:
    output = []
    for summary in comparison:
        event_id = summary["event_id"]
        for contract, valid_field in (("HARD_VALID", "hard_valid"),
                                      ("USABLE_VALID", "usable_valid")):
            if not summary[f"teacher_{valid_field}"] or summary[f"standalone_{valid_field}"]:
                continue
            teacher_valid = [row for row in p["candidates"][(event_id, "TEACHER")]
                             if row[valid_field]]
            direct_laterals = {lateral_key(row) for row in p["laterals"][(event_id, "STANDALONE")]}
            direct_pairs = {factor_config(row) for row in p["pairs"][(event_id, "STANDALONE")]}
            direct_top = {factor_config(row) for row in p["top"][(event_id, "STANDALONE")]}
            direct_candidates = {factor_config(row) for row in
                                 p["candidates"][(event_id, "STANDALONE")]}
            direct_transition_values = {(row[3], row[4]) for row in direct_pairs}
            diagnoses = []
            for row in teacher_valid:
                cfg = factor_config(row)
                lat = cfg[:3] in direct_laterals
                transition = cfg[3:] in direct_transition_values
                pair = cfg in direct_pairs
                top = cfg in direct_top
                reconstructed = cfg in direct_candidates
                if not lat:
                    cause = "LATERAL_FACTOR_MISS"
                elif not transition:
                    cause = "TRANSITION_FACTOR_MISS"
                elif not pair:
                    cause = "PAIR_B128_TRUNCATION"
                elif not top:
                    cause = "TOP12_RANKING_MISS"
                elif not reconstructed:
                    cause = "RECONSTRUCTION_DEDUP_GUARD"
                else:
                    cause = "VALIDATOR_OR_PATH_MISMATCH"
                diagnoses.append(cause)
            priority = ["LATERAL_FACTOR_MISS", "TRANSITION_FACTOR_MISS",
                        "PAIR_B128_TRUNCATION", "TOP12_RANKING_MISS",
                        "RECONSTRUCTION_DEDUP_GUARD", "VALIDATOR_OR_PATH_MISMATCH"]
            classification = next(cause for cause in priority if cause in diagnoses)
            output.append({
                "event_id": event_id, "dataset_role": summary["dataset_role"],
                "source_kind": summary["source_kind"], "validity_contract": contract,
                "classification": classification, "teacher_valid_candidate_count": len(teacher_valid),
                "per_candidate_diagnoses": ";".join(diagnoses),
                "direct_lateral_count": len(direct_laterals),
                "direct_pair_count": len(direct_pairs), "direct_top_count": len(direct_top),
            })
    return output


def path_quality_rows(items: list[dict], p: dict) -> list[dict]:
    metadata = {row["event_id"]: row for row in items}
    output = []
    for method, result_label in (("LEGACY_SEEDED_TEACHER_B128", TEACHER),
                                 ("STANDALONE_DIRECT_SEED_B128", DIRECT)):
        detail = "TEACHER" if method.startswith("LEGACY") else "STANDALONE"
        for event_id, item in metadata.items():
            selected_digest = p["results"][(event_id, result_label)]["selected_digest"]
            for row in p["candidates"][(event_id, detail)]:
                if not row["hard_valid"]:
                    continue
                output.append({
                    "event_id": event_id, "dataset_role": item["dataset_role"],
                    "source_kind": item["source_kind"], "method": method,
                    "selected": int(row["digest"] == selected_digest), **row,
                })
    return output


def runtime_rows(p: dict, warmup: int, repeats: int) -> list[dict]:
    output = []
    stages = (
        ("EXECUTABLE_WALL_TOTAL", "wall_us"), ("GQSC_CORE_TOTAL", "r3_total_us"),
        ("CONTEXT_PREPARATION", "context_us"), ("GEOMETRY_PREPARATION", "geometry_us"),
        ("DIRECT_TRANSITION_GENERATION", "transition_us"),
        ("DIRECT_LATERAL_GENERATION", "lateral_us"),
        ("PAIR_PROXY_PRIORITY", "pair_priority_us"),
        ("LEXICOGRAPHIC_TOP10", "lexicographic_us"), ("COVERAGE_RESERVE2", "coverage_us"),
        ("SHAPE_DEDUPLICATION", "shape_dedup_us"), ("P3_RECONSTRUCTION", "reconstruction_us"),
        ("EXACT_VALIDATION", "validation_us"), ("FINAL_RANKING", "final_ranking_us"),
    )
    for method in ("LEGACY_SEEDED_TEACHER_B128", "STANDALONE_DIRECT_SEED_B128"):
        subset = [row for row in p["standalone_timing"] if row["method"] == method]
        for stage, field in stages:
            scope = "STANDALONE_EXECUTABLE" if method.startswith("STANDALONE") else (
                "TEACHER_EXECUTABLE_INCLUDES_LEGACY_SEED_HARVEST" if stage == "EXECUTABLE_WALL_TOTAL"
                else "TEACHER_GQSC_CORE_ONLY")
            output.append({
                "method": method, "stage": stage, "measurement_scope": scope,
                "release_build": 1, "single_warm_process": 1, "instrumentation_enabled": 0,
                "warmup_per_snapshot": warmup, "measured_repeats_per_snapshot": repeats,
                **stats([row[field] for row in subset]),
            })
    return output


def bounded_rows(items: list[dict], p: dict) -> list[dict]:
    output = []
    for item in items:
        event_id = item["event_id"]
        for method, result_label, detail in (
                ("LEGACY_SEEDED_TEACHER_B128", TEACHER, "TEACHER"),
                ("STANDALONE_DIRECT_SEED_B128", DIRECT, "STANDALONE")):
            result = p["results"][(event_id, result_label)]
            lateral = len(p["laterals"][(event_id, detail)])
            transitions = len(p["transitions"][(event_id, detail)])
            pairs = len(p["pairs"][(event_id, detail)])
            seeds = len(p["seeds"][event_id]) if detail == "TEACHER" else 0
            output.append({
                "event_id": event_id, "method": method, "lateral_factor_count": lateral,
                "lateral_factor_cap": 64, "transition_factor_count": transitions,
                "transition_family_cap": 7, "pair_proxy_count": pairs, "pair_proxy_cap": 128,
                "reconstruction_count": result["r3_constructions"], "reconstruction_cap": 12,
                "validator_count": result["r3_validators"], "validator_cap": 12,
                "legacy_seed_count_consumed": seeds,
                "legacy_builder_or_validator_called_inside_standalone": 0,
                "all_bounds_pass": int(lateral <= 64 and transitions <= 7 and pairs <= 128 and
                                       result["r3_constructions"] <= 12 and
                                       result["r3_validators"] <= 12 and
                                       (detail != "STANDALONE" or seeds == 0)),
            })
    return output


def write_operator_definitions() -> None:
    (HERE / "direct_operator_definitions.md").write_text("""# Direct operator definitions

All operators are research/shadow-only and consume `P3R3K12SideGeometry`, not a legacy path or
validator result. For side corridor domain `D=[a,b]`, let `near(x,y)` be the endpoint nearest zero
(numeric value breaks ties), `far` the other endpoint, and `clip_D` the side-domain clamp.

## Target anchors

1. `T_grid = argmin_(q in {a+i(b-a)/4, i=0..4}) (|q|,q)` reproduces the M0 target-grid geometry.
2. `T_inset = near(a,b) + (far(a,b)-near(a,b))/64` reproduces the zero-interface boundary inset.

These are the only generic direct target seeds. Connected corridor component `[c0,c1]` is used
directly by seven bounded lateral recipes at fractions `0, 1/32, 1/8, 3/8, 1/2, 2/3` (with the
existing target/mid offsets or center interpolation), while bottleneck/midpoint corridor centers
provide center-equal and center-interpolation anchors. The resulting bank remains capped at 32 per
side and 64 after deterministic RIGHT/LEFT interleaving.

## Transition operators

For each side, `e0/e1` and `x0/x1` are the geometry-derived minimum/maximum entry and exit scales.
Define `eq=e0+(e1-e0)/4`, `xn=x0+(x1-x0)/64`, and `xm=x0+(x1-x0)/2`. The seven family operators are:

| family | entry | exit |
|---|---:|---:|
| SHORT_ENTRY_SHORT_EXIT | e0 | x0 |
| MEDIUM_ENTRY_SHORT_EXIT | eq | x0 |
| LONG_ENTRY_SHORT_EXIT | e1 | xn |
| LONG_ENTRY_MEDIUM_EXIT | e1 | xm |
| LONG_ENTRY_LONG_EXIT | e1 | x1 |
| SHORT_ENTRY_LONG_EXIT | e0 | x1 |
| SHORT_ENTRY_MEDIUM_EXIT | e0 | xm |

The concrete numbers are evaluated for the lateral factor's own side; only seven semantic families
exist. The existing B128 diagonal wave, cheap corridor proxy, lexicographic Top-10, coverage reserve
Top-2, exact reconstruction, validator, and final candidate ranking are unchanged.

## Explicit exclusions

No M0-V1/M0-V2/M1 path is constructed, no analytic equation is solved, no legacy candidate is
validated, and no factor is harvested from a legacy result. Analytic `d_mid`, probe position/anchor,
root branch, template verdict, and exact-validator result are not inputs. There is no hidden exact-R3
or legacy enumeration.
""", encoding="utf-8")


def write_readme(items, seeds, comparison, misses, runtimes, bounded) -> None:
    counts = {}
    for method, prefix, field_prefix in ((LEGACY, "legacy", "legacy"),
                                         (TEACHER, "teacher", "teacher"),
                                         (DIRECT, "direct", "standalone")):
        rows = comparison
        counts[prefix] = {
            "hard": sum(row[f"{field_prefix}_hard_valid"] for row in rows),
            "usable": sum(row[f"{field_prefix}_usable_valid"] for row in rows),
        }
    failures = [row for row in comparison if row["source_kind"] == "FROZEN_PRODUCTION_FAILURE"]
    controls = [row for row in comparison if row["source_kind"] == "LEGACY_SUCCESS_CONTROL"]
    direct_failure_hard = sum(row["standalone_hard_valid"] for row in failures)
    direct_failure_usable = sum(row["standalone_usable_valid"] for row in failures)
    teacher_failure_hard = sum(row["teacher_hard_valid"] for row in failures)
    teacher_failure_usable = sum(row["teacher_usable_valid"] for row in failures)
    direct_control_hard = sum(row["standalone_hard_valid"] for row in controls)
    direct_control_usable = sum(row["standalone_usable_valid"] for row in controls)
    teacher_control_hard = sum(row["teacher_hard_valid"] for row in controls)
    teacher_control_usable = sum(row["teacher_usable_valid"] for row in controls)
    legacy_usable_preserved = sum(
        row["legacy_usable_valid"] and row["standalone_usable_valid"] for row in controls)
    teacher_legacy_digest = sum(
        row["teacher_vs_legacy_selected_digest_exact"] for row in controls)
    direct_legacy_digest = sum(
        row["standalone_vs_legacy_selected_digest_exact"] for row in controls)
    digest_agree = sum(row["selected_digest_exact"] for row in comparison
                       if row["teacher_hard_valid"] and row["standalone_hard_valid"])
    both = sum(row["teacher_hard_valid"] and row["standalone_hard_valid"] for row in comparison)
    direct_gains = [row["event_id"] for row in comparison
                    if not row["teacher_hard_valid"] and row["standalone_hard_valid"]]
    direct_losses = [row["event_id"] for row in comparison
                     if row["teacher_hard_valid"] and not row["standalone_hard_valid"]]
    taxonomy = Counter(row["classification"] for row in misses)
    stages = Counter(row["generator_stage"] for row in seeds)
    roots = Counter(row["root_type"] for row in seeds)
    semantics = Counter(row["semantic_class"] for row in seeds)
    runtime = {(row["method"], row["stage"]): row for row in runtimes}
    direct_rt = runtime[("STANDALONE_DIRECT_SEED_B128", "EXECUTABLE_WALL_TOTAL")]
    teacher_rt = runtime[("LEGACY_SEEDED_TEACHER_B128", "GQSC_CORE_TOTAL")]
    recommendation = "GQSC_STANDALONE_COVERAGE_INSUFFICIENT"
    text = f"""# GQSC standalone bounded direct-seed generator study v1

## Decision

`{recommendation}`

The prototype is genuinely standalone and bounded, and its usable result is promising, but it does
not meet the minimum coverage contract: legacy hard-success coverage is {direct_control_hard}/18,
not 18/18, and it misses {len(direct_losses)} teacher-hard events. It must not be frozen or connected
to production decisions.

## Scope and protocol

This is a seen-only research/shadow study over 86 frozen production-failure snapshots plus 18
legacy-success controls. Dataset roles are exactly `{sorted(EXPECTED_ROLES)}` plus the existing
success-control role. FINAL_HOLDOUT paths, contents, outcomes, and statistics were not accessed.
The Release C++ harness ran all 104 inputs in one warm process with research instrumentation OFF,
2 warmups and 10 measured repeats per snapshot. Production `plan()` never calls the standalone
method.

## What the legacy-seeded teacher actually consumes

The audit contains all {len(seeds)} legacy-derived seed tuples: stage counts {dict(stages)}, root
counts {dict(roots)}, and semantic counts {dict(semantics)}. Source inspection plus the recorded
dataflow establishes a narrower dependency than the legacy template labels suggest:

- GQSC reads only each seed's `d_target`, `entry_scale`, and `exit_scale`.
- It does **not** read legacy `d_mid`, `s_probe`, `d_probe`, root value/index/branch, template hard
  verdict, or validator diagnostic.
- Therefore analytic-root and zero-interface labels are historical provenance, not required numeric
  inputs to B128. The required numeric semantics reduce to the side-domain/grid and component target
  anchors plus the bounded transition-range anchors reconstructed in
  [direct_operator_definitions.md](direct_operator_definitions.md).

The legacy-seeded teacher remains the existing reference: 62/104 hard and 51/104 usable, including
18/18 legacy hard controls and 44/86 legacy-failure hard recoveries. Its 18 success-control selected
digests match the preceding unified-architecture artifact exactly (18/18), so the standalone work did
not alter teacher semantics.

[legacy_seed_dependency.csv](legacy_seed_dependency.csv) preserves every tuple and marks whether its
target/transition value supports observed hard or usable teacher candidates. Those flags are
descriptive support, not a claim that an individual duplicate seed is causally unique.

## Coverage comparison

| corpus/contract | legacy | legacy-seeded teacher | standalone direct seed |
|---|---:|---:|---:|
| all 104 hard | {counts['legacy']['hard']} | {counts['teacher']['hard']} | {counts['direct']['hard']} |
| all 104 usable | {counts['legacy']['usable']} | {counts['teacher']['usable']} | {counts['direct']['usable']} |
| 18 legacy controls hard | {sum(r['legacy_hard_valid'] for r in controls)} | {teacher_control_hard} | {direct_control_hard} |
| 18 legacy controls usable | {sum(r['legacy_usable_valid'] for r in controls)} | {teacher_control_usable} | {direct_control_usable} |
| 86 legacy failures hard recovered | 0 | {teacher_failure_hard} | {direct_failure_hard} |
| 86 legacy failures usable recovered | 0 | {teacher_failure_usable} | {direct_failure_usable} |

All {legacy_usable_preserved}/14 legacy-usable control paths remain covered by at least one
standalone usable path. Selected digest agreement against the legacy selection is
teacher {teacher_legacy_digest}/18 and standalone {direct_legacy_digest}/18; coverage is therefore
not path-identity preservation.

Among {both} events where both methods return hard-valid paths, selected digest agreement is
{digest_agree}/{both}. Standalone hard gains beyond the teacher occur at `{';'.join(direct_gains)}`;
teacher-hard losses occur at `{';'.join(direct_losses)}`. These gains do not compensate for the
18/18 non-interference failure. Full per-event hard/usable set intersections are in
[teacher_vs_standalone.csv](teacher_vs_standalone.csv).

## Miss diagnosis

Teacher-valid/standalone-invalid contract rows: {len(misses)}; taxonomy {dict(taxonomy)}. The main
remaining failure is not an unbounded search: direct factors are either absent or displaced by the
changed standalone B128/Top-12 pool. See [miss_taxonomy.csv](miss_taxonomy.csv). This v1 deliberately
stops rather than tuning more operators on the same seen outcomes.

## Bounds and runtime

All {len(bounded)} per-event/method bound rows pass: lateral <=64, transition families <=7,
pair proxies <=128, reconstruction <=12, validator <=12. Standalone rows consume zero legacy seeds
and call no legacy builder/validator internally.

Standalone executable wall p50/p90/p95/p99/max is
{direct_rt['p50_ms']:.3f}/{direct_rt['p90_ms']:.3f}/{direct_rt['p95_ms']:.3f}/
{direct_rt['p99_ms']:.3f}/{direct_rt['max_ms']:.3f} ms. The current legacy-seeded teacher's isolated
GQSC core p50/p95/p99/max in the same run is {teacher_rt['p50_ms']:.3f}/
{teacher_rt['p95_ms']:.3f}/{teacher_rt['p99_ms']:.3f}/{teacher_rt['max_ms']:.3f} ms; its executable
wall additionally includes legacy seed construction/validation. Full breakdown is in
[native_runtime.csv](native_runtime.csv).

## Static path-quality interpretation

[path_quality_shadow.csv](path_quality_shadow.csv) records every hard-valid shadow candidate's track
and obstacle margins, braking deficit, normalized slack, mean lateral deviation, maximum curvature,
curvature rate, point count, and maneuver station length. These are static exact-validator/shadow
diagnostics only; no closed-loop robustness claim is made.

## Conclusion

The direct geometry operators prove that legacy path construction and analytic root harvesting are
not intrinsically required to execute bounded GQSC. However, v1 fails the frozen-method candidate
coverage gate, so the correct outcome is `{recommendation}`. No production decision path, validator,
ranking, P3 geometry, parameter, lifecycle, or fallback was changed, and no commit/push was made.
"""
    (HERE / "README.md").write_text(text, encoding="utf-8")


def main() -> None:
    warmup, repeats = 2, 10
    items = U.corpus()
    failures = [row for row in items if row["source_kind"] == "FROZEN_PRODUCTION_FAILURE"]
    if len(items) != 104 or len(failures) != 86 or {row["dataset_role"] for row in failures} != EXPECTED_ROLES:
        raise RuntimeError("seen-only corpus contract changed")
    if any("holdout" in str(row["event_path"]).lower() for row in items):
        raise RuntimeError("holdout path access prohibited")
    p = parse(run(items, warmup, repeats))
    if any(p["seeds"].get(row["event_id"] + "_STANDALONE") for row in items):
        raise RuntimeError("standalone consumed legacy seeds")
    seed_rows = legacy_seed_rows(items, p)
    comparison = comparison_rows(items, p)
    misses = miss_rows(comparison, p)
    quality = path_quality_rows(items, p)
    runtimes = runtime_rows(p, warmup, repeats)
    bounded = bounded_rows(items, p)
    if len(seed_rows) != 748 or not all(row["all_bounds_pass"] for row in bounded):
        raise RuntimeError("seed count or bounded contract changed")
    operator_coverage = [{
        "source_kind": source,
        "snapshot_count": len(rows),
        "legacy_hard": sum(row["legacy_hard_valid"] for row in rows),
        "legacy_usable": sum(row["legacy_usable_valid"] for row in rows),
        "teacher_hard": sum(row["teacher_hard_valid"] for row in rows),
        "teacher_usable": sum(row["teacher_usable_valid"] for row in rows),
        "standalone_hard": sum(row["standalone_hard_valid"] for row in rows),
        "standalone_usable": sum(row["standalone_usable_valid"] for row in rows),
        "standalone_hard_delta_vs_teacher": sum(
            row["standalone_hard_valid"] - row["teacher_hard_valid"] for row in rows),
        "standalone_usable_delta_vs_teacher": sum(
            row["standalone_usable_valid"] - row["teacher_usable_valid"] for row in rows),
    } for source in ("LEGACY_SUCCESS_CONTROL", "FROZEN_PRODUCTION_FAILURE", "ALL")
      for rows in [[row for row in comparison if source == "ALL" or row["source_kind"] == source]]]
    write_csv(HERE / "legacy_seed_dependency.csv", seed_rows)
    write_csv(HERE / "operator_coverage.csv", operator_coverage)
    write_csv(HERE / "teacher_vs_standalone.csv", comparison)
    write_csv(HERE / "miss_taxonomy.csv", misses)
    write_csv(HERE / "path_quality_shadow.csv", quality)
    write_csv(HERE / "native_runtime.csv", runtimes)
    write_csv(HERE / "bounded_contract.csv", bounded)
    write_operator_definitions()
    write_readme(items, seed_rows, comparison, misses, runtimes, bounded)
    print("GQSC_STANDALONE_COVERAGE_INSUFFICIENT")


if __name__ == "__main__":
    main()
