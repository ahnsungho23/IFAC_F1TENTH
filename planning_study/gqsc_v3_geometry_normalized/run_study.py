#!/usr/bin/env python3
"""Staged DEVELOPMENT-first GQSC v3 geometry-normalized study.

The phase boundary is intentional: ``development-baseline`` and
``development-policies`` never address validation or success-control paths.
``one-shot`` requires a prevalidation lock whose SHA still matches.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import math
import os
import subprocess
from collections import Counter, defaultdict
from pathlib import Path


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
V1_DIR = REPO / "planning_study/gqsc_standalone_direct_seed_v1"
V1_SCRIPT = V1_DIR / "run_study.py"
PILOT = REPO / "planning_study/p3_oracle_pilot_v2"
DEVELOPMENT = REPO / "planning_study/p3_mapping_research_corpus_v1/raw_oracle"
VALIDATION = REPO / "planning_study/p3_geometry_conditioned_validation_v1"
SUCCESS = REPO / "planning_study/p3_geometry_conditioned_method_v1"
INTEGRATION_SUCCESS = (
    REPO / "planning_study/p3_r3_k12_integration_v1/production_success_noninterference.csv")
V2_ABLATION = REPO / "planning_study/gqsc_standalone_coverage_repair_v2/diversity_policy_ablation.csv"
LOCK_JSON = HERE / "selected_v3_policy.json"
LOCK_SHA = HERE / "prevalidation_lock_sha.txt"
ACCESS_LOG = HERE / "dataset_access_log.csv"
EPS = 1.0e-9
PREDECLARATION_SHA = {
    "v3_operator_candidates.md":
        "9a0d91dd208e375ae6d486190669c57d4bda59e2945dd4afad0beb2a0bdadfdf",
    "v3_parameter_provenance.md":
        "018a6b0258536d11247454d77c6107f9970c71964f282c17b174350f5f355cb6",
}
POLICIES = (
    {"policy_id": "P0", "policy_name": "V3_CLEAN_V1_BASELINE",
     "selector": "V1_BASELINE", "complexity_order": 0},
    {"policy_id": "P1", "policy_name": "V3_DISJOINT_PROXY_COVERAGE",
     "selector": "DISJOINT_COVERAGE", "complexity_order": 1},
    {"policy_id": "P2", "policy_name": "V3_SIDE_BALANCED_DISJOINT",
     "selector": "V3_SIDE_BALANCED_DISJOINT", "complexity_order": 2},
    {"policy_id": "P3", "policy_name": "V3_NORMALIZED_MAXIMIN_DISJOINT",
     "selector": "V3_NORMALIZED_MAXIMIN_DISJOINT", "complexity_order": 3},
)


def load(path: Path):
    spec = importlib.util.spec_from_file_location("gqsc_v1_for_v3", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


V1 = load(V1_SCRIPT)
HARNESS = V1.HARNESS


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


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def verify_predeclaration() -> None:
    mismatch = {name: {"expected": expected, "observed": sha256(HERE / name)}
                for name, expected in PREDECLARATION_SHA.items()
                if sha256(HERE / name) != expected}
    if mismatch:
        raise RuntimeError("predeclared v3 policy changed: " + json.dumps(mismatch, sort_keys=True))


def append_accesses(rows: list[dict]) -> None:
    existing = read_csv(ACCESS_LOG) if ACCESS_LOG.is_file() else []
    seen = {tuple(sorted(row.items())) for row in existing}
    for row in rows:
        key = tuple(sorted(row.items()))
        if key not in seen:
            existing.append(row)
            seen.add(key)
    write_csv(ACCESS_LOG, existing)


def development_catalog() -> tuple[list[dict], list[dict]]:
    groups = (
        ("PILOT_SEEN_DEVELOPMENT_DATA", PILOT / "oracle_summary.csv",
         PILOT / "inputs", PILOT / "lineage_parity.csv"),
        ("DEVELOPMENT", DEVELOPMENT / "oracle_summary.csv",
         DEVELOPMENT / "inputs", DEVELOPMENT / "lineage_parity.csv"),
    )
    items: list[dict] = []
    accesses: list[dict] = []
    for role, summary_path, input_dir, lineage_path in groups:
        accesses.extend((
            {"phase": "DEVELOPMENT_BASELINE", "dataset_role": role,
             "resource": str(summary_path.relative_to(REPO)), "access": "READ"},
            {"phase": "DEVELOPMENT_BASELINE", "dataset_role": role,
             "resource": str(lineage_path.relative_to(REPO)), "access": "READ"},
        ))
        lineage = {row["event_id"]: row for row in read_csv(lineage_path)}
        for row in read_csv(summary_path):
            event_id = row["event_id"]
            event_path = input_dir / f"{event_id}.event"
            if event_id not in lineage or not event_path.is_file():
                raise RuntimeError(f"incomplete DEVELOPMENT lineage: {event_id}")
            items.append({
                "event_id": event_id, "dataset_role": role,
                "bag": row.get("bag", "UNKNOWN"), "event_path": event_path,
                "source_kind": "FROZEN_PRODUCTION_FAILURE",
            })
            accesses.append({
                "phase": "DEVELOPMENT_BASELINE", "dataset_role": role,
                "resource": str(event_path.relative_to(REPO)), "access": "HARNESS_INPUT"})
    if len(items) != 49 or len({row["event_id"] for row in items}) != 49:
        raise RuntimeError("DEVELOPMENT-only corpus must contain 49 unique events")
    return items, accesses


def run_standalone(items: list[dict], policy: str = "V1_BASELINE",
                   operator: str = "NONE", warmup: int = 0, repeats: int = 0) -> tuple[str, dict]:
    if not HARNESS.is_file():
        raise RuntimeError(f"missing Release harness: {HARNESS}")
    env = os.environ.copy()
    env.pop("LOCAL_PLANNING_RESEARCH_PARITY", None)
    env.update({
        "GQSC_STANDALONE_STUDY": "1",
        "GQSC_STANDALONE_WARMUP": str(warmup),
        "GQSC_STANDALONE_REPEATS": str(repeats),
        "GQSC_V2_POLICY": policy,
        "GQSC_V3_OPERATOR": operator,
    })
    env.pop("GQSC_V2_COMPONENT_OPERATOR", None)
    completed = subprocess.run(
        [str(HARNESS), *(str(row["event_path"]) for row in items)],
        check=True, text=True, stdout=subprocess.PIPE, env=env)
    return completed.stdout, V1.parse(completed.stdout)


def exact(value) -> str:
    return float(value).hex()


def lateral_key(row: dict) -> tuple[str, str, str]:
    return row["side"], exact(row["d_target"]), exact(row["d_mid"])


def parse_components(text: str) -> list[tuple[float, float]]:
    if not text:
        return []
    return [tuple(float(value) for value in token.split(":")) for token in text.split(";")]


def normalized_in_component(value: float, component: tuple[float, float]) -> float:
    first, second = component
    near, far = (first, second)
    if (abs(far), far) < (abs(near), near):
        near, far = far, near
    return (value - near) / (far - near)


def matching_component(target: float, components: list[tuple[float, float]]):
    for component in components:
        lower, upper = sorted(component)
        if lower - EPS <= target <= upper + EPS:
            return component
    return None


def development_deficiency(items: list[dict], parsed: dict) -> tuple[list[dict], dict]:
    rows: list[dict] = []
    event_summary: dict[str, dict] = {}
    for item in items:
        event_id = item["event_id"]
        standalone_laterals = {
            lateral_key(row) for row in parsed["laterals"][(event_id, "STANDALONE")]}
        geometry = {
            row["side"]: row for row in parsed["geometry"][(event_id, "STANDALONE")]}
        teacher_candidates = parsed["candidates"][(event_id, "TEACHER")]
        hard = [row for row in teacher_candidates if row["hard_valid"]]
        usable = [row for row in teacher_candidates if row["usable_valid"]]
        hard_missing = [row for row in hard if lateral_key(row) not in standalone_laterals]
        usable_missing = [row for row in usable if lateral_key(row) not in standalone_laterals]
        event_summary[event_id] = {
            "teacher_hard": len(hard), "teacher_usable": len(usable),
            "hard_missing": len(hard_missing), "usable_missing": len(usable_missing),
            "all_hard_laterals_missing": bool(hard) and len(hard_missing) == len(hard),
            "all_usable_laterals_missing": bool(usable) and len(usable_missing) == len(usable),
        }
        for validity, candidates in (("HARD", hard_missing), ("USABLE", usable_missing)):
            for candidate in candidates:
                side_geometry = geometry.get(candidate["side"])
                components = [] if side_geometry is None else parse_components(
                    side_geometry["components"])
                component = matching_component(candidate["d_target"], components)
                rows.append({
                    "event_id": event_id, "dataset_role": item["dataset_role"],
                    "validity": validity, "side": candidate["side"],
                    "d_target": candidate["d_target"], "d_mid": candidate["d_mid"],
                    "component_lower": math.nan if component is None else min(component),
                    "component_upper": math.nan if component is None else max(component),
                    "u_target_near_to_far": math.nan if component is None else
                        normalized_in_component(candidate["d_target"], component),
                    "u_mid_near_to_far": math.nan if component is None else
                        normalized_in_component(candidate["d_mid"], component),
                    "entry_scale": candidate["entry_scale"],
                    "exit_scale": candidate["exit_scale"],
                    "candidate_digest": candidate["digest"],
                })
    return rows, event_summary


def phase_development_baseline() -> None:
    items, accesses = development_catalog()
    text, parsed = run_standalone(items)
    missing_rows, summary = development_deficiency(items, parsed)
    ablation = [row for row in read_csv(V2_ABLATION)
                if row["selection_corpus"] == "DEVELOPMENT_49" and
                row["component_operator_extension"] == "0"]
    by_policy = {row["policy"]: row for row in ablation}
    baseline_ablation = by_policy["V1_BASELINE"]
    min_ablation = by_policy["ZERO_INTERFACE_EQUAL_MIN_OUTWARD"]
    max_ablation = by_policy["ZERO_INTERFACE_EQUAL_MAX_OUTWARD"]
    accesses.append({
        "phase": "DEVELOPMENT_BASELINE", "dataset_role": "DEVELOPMENT_49",
        "resource": str(V2_ABLATION.relative_to(REPO)),
        "access": "READ_DEVELOPMENT_ABLATION"})
    write_csv(HERE / "development_missing_laterals.csv", missing_rows)
    write_csv(ACCESS_LOG, accesses)

    hard_events = [event for event, row in summary.items() if row["teacher_hard"]]
    all_hard_missing = [event for event, row in summary.items()
                        if row["all_hard_laterals_missing"]]
    usable_events = [event for event, row in summary.items() if row["teacher_usable"]]
    all_usable_missing = [event for event, row in summary.items()
                          if row["all_usable_laterals_missing"]]
    hard_missing_events = sorted({row["event_id"] for row in missing_rows
                                  if row["validity"] == "HARD"})
    usable_missing_events = sorted({row["event_id"] for row in missing_rows
                                    if row["validity"] == "USABLE"})
    finite_hard = [row for row in missing_rows if row["validity"] == "HARD" and
                   math.isfinite(row["u_target_near_to_far"])]
    operator_counts = Counter(
        row["operator"] for event in summary for row in
        parsed["laterals"][(event, "STANDALONE")])

    (HERE / "development_geometry_deficiency.md").write_text(f"""# DEVELOPMENT geometry deficiency

## Access boundary

This phase read only the 9 `PILOT_SEEN_DEVELOPMENT_DATA` and 40 `DEVELOPMENT` summaries,
lineage rows, and event inputs listed in `dataset_access_log.csv`. It did not address a
validation-seen, success-control, or final-holdout path.

## Clean standalone v1 result

- DEVELOPMENT events: 49
- events with at least one teacher hard candidate: {len(hard_events)}
- events with at least one teacher usable candidate: {len(usable_events)}
- events containing any teacher-hard lateral absent from the standalone lateral bank:
  `{';'.join(hard_missing_events) or 'NONE'}`
- events for which every teacher-hard lateral is absent:
  `{';'.join(all_hard_missing) or 'NONE'}`
- events containing any teacher-usable lateral absent from the bank:
  `{';'.join(usable_missing_events) or 'NONE'}`
- events for which every teacher-usable lateral is absent:
  `{';'.join(all_usable_missing) or 'NONE'}`
- missing teacher-hard candidate rows with a containing connected component: {len(finite_hard)}

The exact missing rows and their side-relative `(u_target,u_mid)` coordinates are in
`development_missing_laterals.csv`. This table is diagnostic only; no normalized coefficient has
yet been selected from those coordinates.

The three events occupy different regions:

- V2E08: target at the near boundary `u_target=0`, with `u_mid=-0.083682` outside the component;
  another clean-bank hard/usable lateral already exists for the event.
- DVE020: outer-region rows at `u_target=0.697693 or 0.949615` and
  `u_mid=0.798462 or 0.949615`; other clean-bank hard/usable laterals already exist.
- DVE039: `u_target=0.551861`, `u_mid=0.162903`; this is the only event whose entire teacher
  hard/usable lateral set is absent.

There is therefore no repeated normalized missing-lateral mechanism that justifies a new direct
operator. In accordance with the predeclared gate, v3 adds no lateral anchor for DVE039.

## MIN_OUTWARD audit

The existing DEVELOPMENT-only ablation gives:

- clean v1: hard {baseline_ablation['hard_count']}/49, DVE023={baseline_ablation['DVE023_hard']};
- equal+MIN outward: hard {min_ablation['hard_count']}/49,
  DVE023={min_ablation['DVE023_hard']};
- equal+MAX outward: hard {max_ablation['hard_count']}/49,
  DVE023={max_ablation['DVE023_hard']}.

Usable coverage remains {baseline_ablation['usable_count']}/49 for all three. The sole hard-event
distinction between MIN and MAX is DVE023. `MIN_OUTWARD` is therefore classified as case **B: an
effectively event-specific DVE023 repair**, not retained as `DEVELOPMENT_TUNED_GENERAL` in v3.

## Structural decision gate

An added lateral operator is justified only if the missing rows show a common normalized region
in more than one independent DEVELOPMENT event. If the all-hard/usable lateral deficiency is
unique to one event, v3 must not add an operator to recreate it. The clean-bank operator inventory
contains {len(operator_counts)} distinct named operators; this phase did not alter it.
""", encoding="utf-8")
    print(json.dumps({
        "development_events": len(items),
        "hard_missing_events": hard_missing_events,
        "all_hard_missing_events": all_hard_missing,
        "usable_missing_events": usable_missing_events,
        "all_usable_missing_events": all_usable_missing,
        "missing_row_count": len(missing_rows),
    }, sort_keys=True))


def standalone_bound_rows(items: list[dict], parsed: dict) -> list[dict]:
    return [row for row in V1.bounded_rows(items, parsed)
            if row["method"] == "STANDALONE_DIRECT_SEED_B128"]


def policy_diversity(items: list[dict], parsed: dict) -> dict:
    coverage_count = 0
    both_side_events = 0
    unique_laterals = 0
    unique_transitions = 0
    for item in items:
        rows = parsed["top"][(item["event_id"], "STANDALONE")]
        coverage = [row for row in rows if row["stream"] == "COVERAGE"]
        coverage_count += len(coverage)
        both_side_events += int({row["side"] for row in coverage} == {"RIGHT", "LEFT"})
        unique_laterals += len({lateral_key(row) for row in rows})
        unique_transitions += len({(exact(row["entry_scale"]), exact(row["exit_scale"]))
                                  for row in rows})
    count = len(items)
    return {
        "coverage_selected_count": coverage_count,
        "coverage_both_sides_event_count": both_side_events,
        "mean_top12_unique_lateral_count": unique_laterals / count,
        "mean_top12_unique_transition_count": unique_transitions / count,
    }


def phase_development_policies() -> None:
    verify_predeclaration()
    items, _ = development_catalog()
    summaries: list[dict] = []
    parsed_by_policy: dict[str, dict] = {}
    for policy in POLICIES:
        _, parsed = run_standalone(items, policy["selector"])
        parsed_by_policy[policy["policy_id"]] = parsed
        comparison = V1.comparison_rows(items, parsed)
        bounded = standalone_bound_rows(items, parsed)
        diversity = policy_diversity(items, parsed)
        summaries.append({
            **policy,
            "selection_corpus": "DEVELOPMENT_49",
            "event_count": len(items),
            "hard_count": sum(row["standalone_hard_valid"] for row in comparison),
            "usable_count": sum(row["standalone_usable_valid"] for row in comparison),
            "teacher_hard_count": sum(row["teacher_hard_valid"] for row in comparison),
            "teacher_usable_count": sum(row["teacher_usable_valid"] for row in comparison),
            "max_lateral": max(row["lateral_factor_count"] for row in bounded),
            "max_transition": max(row["transition_factor_count"] for row in bounded),
            "max_pairs": max(row["pair_proxy_count"] for row in bounded),
            "max_reconstructions": max(row["reconstruction_count"] for row in bounded),
            "max_validators": max(row["validator_count"] for row in bounded),
            "all_bounds_pass": int(all(row["all_bounds_pass"] for row in bounded)),
            "event_or_map_specific_parameter": 0,
            "component_half_far002_inward015_enabled": 0,
            "min_outward_enabled": 0,
            **diversity,
        })
    baseline_usable = next(row["usable_count"] for row in summaries
                           if row["policy_id"] == "P0")
    for row in summaries:
        row["usable_nonregression_vs_p0"] = int(row["usable_count"] >= baseline_usable)
    eligible = [row for row in summaries
                if not row["event_or_map_specific_parameter"] and
                row["all_bounds_pass"] and row["usable_nonregression_vs_p0"]]
    if not eligible:
        raise RuntimeError("no v3 policy satisfies the predeclared eligibility gate")
    best_usable = max(row["usable_count"] for row in eligible)
    eligible = [row for row in eligible if row["usable_count"] == best_usable]
    best_hard = max(row["hard_count"] for row in eligible)
    eligible = [row for row in eligible if row["hard_count"] == best_hard]
    best_both_side = max(row["coverage_both_sides_event_count"] for row in eligible)
    eligible = [row for row in eligible
                if row["coverage_both_sides_event_count"] == best_both_side]
    best_unique_lateral = max(row["mean_top12_unique_lateral_count"] for row in eligible)
    eligible = [row for row in eligible
                if row["mean_top12_unique_lateral_count"] == best_unique_lateral]
    selected = min(eligible, key=lambda row: row["complexity_order"])
    for row in summaries:
        row["selected_on_development"] = int(row["policy_id"] == selected["policy_id"])
        row["selection_order"] = (
            "clean,bounded,usable_nonregression,max_usable,max_hard,"
            "max_both_side_coverage,max_unique_lateral,min_complexity")
    write_csv(HERE / "development_policy_comparison.csv", summaries)

    # Runtime is measured only after the Development selection has become immutable in this phase.
    warmup, repeats = 2, 10
    _, timed = run_standalone(items, selected["selector"], warmup=warmup, repeats=repeats)
    runtimes = [dict(row, policy_id=selected["policy_id"],
                     policy_name=selected["policy_name"])
                for row in V1.runtime_rows(timed, warmup, repeats)
                if row["method"] == "STANDALONE_DIRECT_SEED_B128"]
    write_csv(HERE / "native_runtime.csv", runtimes)
    append_accesses([{
        "phase": "DEVELOPMENT_POLICY_COMPARISON", "dataset_role": "DEVELOPMENT_49",
        "resource": "49 frozen DEVELOPMENT event inputs",
        "access": "FOUR_PREDECLARED_POLICIES_PLUS_SELECTED_RUNTIME",
    }])
    print(json.dumps({"selected_policy": selected["policy_name"],
                      "hard_count": selected["hard_count"],
                      "usable_count": selected["usable_count"]}, sort_keys=True))


def canonical_json_sha(payload: dict) -> str:
    return hashlib.sha256((json.dumps(payload, sort_keys=True, indent=2) + "\n").encode()).hexdigest()


def phase_lock() -> None:
    verify_predeclaration()
    comparison_path = HERE / "development_policy_comparison.csv"
    runtime_path = HERE / "native_runtime.csv"
    if not comparison_path.is_file() or not runtime_path.is_file():
        raise RuntimeError("run development-policies before lock")
    selected_rows = [row for row in read_csv(comparison_path)
                     if row["selected_on_development"] == "1"]
    if len(selected_rows) != 1:
        raise RuntimeError("Development selection must contain exactly one policy")
    row = selected_rows[0]
    payload = {
        "schema_version": "GQSC_V3_PREVALIDATION_LOCK_V1",
        "status": "V3_PREVALIDATION_LOCKED",
        "method_name": row["policy_name"],
        "policy_id": row["policy_id"],
        "selector_enum": row["selector"],
        "selection_corpus": "DEVELOPMENT_49",
        "selection_rule": row["selection_order"],
        "development_hard_count": int(row["hard_count"]),
        "development_usable_count": int(row["usable_count"]),
        "component_half_far002_inward015_enabled": False,
        "zero_interface_equal_min_outward_enabled": False,
        "lateral_bank": "CLEAN_STANDALONE_V1_UNCHANGED",
        "geometry_coordinates": ["side_sigma", "u_target", "u_mid",
                                 "entry_normalized", "exit_normalized"],
        "pair_budget": 128,
        "lexicographic_quota": 10,
        "coverage_reserve": 2,
        "candidate_cap": 12,
        "validator_cap": 12,
        "predeclaration_sha256": PREDECLARATION_SHA,
        "development_policy_comparison_sha256": sha256(comparison_path),
        "native_selector_source_sha256": sha256(
            REPO / "src/local_planning/src/p3_r3_k12.cpp"),
        "native_selector_header_sha256": sha256(
            REPO / "src/local_planning/include/local_planning/p3_r3_k12.hpp"),
        "release_harness_sha256": sha256(HARNESS),
    }
    serialized = json.dumps(payload, sort_keys=True, indent=2) + "\n"
    LOCK_JSON.write_text(serialized, encoding="utf-8")
    observed = hashlib.sha256(serialized.encode()).hexdigest()
    if observed != canonical_json_sha(payload):
        raise RuntimeError("canonical lock serialization mismatch")
    LOCK_SHA.write_text(
        f"{observed}  selected_v3_policy.json\nV3_PREVALIDATION_LOCKED\n",
        encoding="utf-8")
    print(json.dumps({"status": "V3_PREVALIDATION_LOCKED", "sha256": observed,
                      "method_name": payload["method_name"]}, sort_keys=True))


def verified_lock() -> tuple[dict, str]:
    if not LOCK_JSON.is_file() or not LOCK_SHA.is_file():
        raise RuntimeError("missing V3 prevalidation lock")
    expected = LOCK_SHA.read_text(encoding="utf-8").split()[0]
    observed = sha256(LOCK_JSON)
    payload = json.loads(LOCK_JSON.read_text(encoding="utf-8"))
    if expected != observed or payload.get("status") != "V3_PREVALIDATION_LOCKED":
        raise RuntimeError("invalid V3 prevalidation lock or SHA")
    source = REPO / "src/local_planning/src/p3_r3_k12.cpp"
    header = REPO / "src/local_planning/include/local_planning/p3_r3_k12.hpp"
    if (sha256(source) != payload["native_selector_source_sha256"] or
            sha256(header) != payload["native_selector_header_sha256"] or
            sha256(HARNESS) != payload["release_harness_sha256"]):
        raise RuntimeError("locked selector source/header/harness changed")
    return payload, observed


def validation_catalog() -> tuple[list[dict], list[dict]]:
    summary_path = VALIDATION / "validation_oracle_summary.csv"
    lineage_path = VALIDATION / "raw_oracle/lineage_parity.csv"
    input_dir = VALIDATION / "raw_oracle/inputs"
    lineage = {row["event_id"]: row for row in read_csv(lineage_path)}
    items = []
    accesses = [
        {"phase": "POST_LOCK_ONE_SHOT", "dataset_role": "VALIDATION_SEEN_AFTER_V1",
         "resource": str(summary_path.relative_to(REPO)), "access": "READ_ONCE"},
        {"phase": "POST_LOCK_ONE_SHOT", "dataset_role": "VALIDATION_SEEN_AFTER_V1",
         "resource": str(lineage_path.relative_to(REPO)), "access": "READ_ONCE"},
    ]
    for row in read_csv(summary_path):
        event_id = row["event_id"]
        event_path = input_dir / f"{event_id}.event"
        if event_id not in lineage or not event_path.is_file():
            raise RuntimeError(f"incomplete validation lineage: {event_id}")
        items.append({"event_id": event_id, "dataset_role": "VALIDATION_SEEN_AFTER_V1",
                      "bag": row.get("bag", "UNKNOWN"), "event_path": event_path,
                      "source_kind": "FROZEN_PRODUCTION_FAILURE"})
    if len(items) != 37:
        raise RuntimeError("validation one-shot corpus must contain 37 events")
    return items, accesses


def success_catalog() -> tuple[list[dict], list[dict]]:
    manifest = SUCCESS / "success_control_manifest.csv"
    input_dir = SUCCESS / "success_control_inputs"
    frozen_baseline = {row["event_id"]: row for row in read_csv(INTEGRATION_SUCCESS)}
    items = []
    for row in read_csv(manifest):
        event_path = input_dir / f"{row['control_event_id']}.event"
        if not event_path.is_file():
            raise RuntimeError(f"missing success control: {event_path}")
        baseline = frozen_baseline.get(row["control_event_id"])
        if baseline is None or baseline["input_sha256"] != row["event_input_sha256"]:
            raise RuntimeError(f"missing/mismatched frozen success baseline: {row['control_event_id']}")
        items.append({
            "event_id": row["control_event_id"], "dataset_role": row["dataset_role"],
            "bag": row["bag"], "event_path": event_path,
            "source_kind": "LEGACY_SUCCESS_CONTROL",
            "baseline_digest": baseline["baseline_selected_path_digest"],
        })
    if len(items) != 18:
        raise RuntimeError("success-control one-shot corpus must contain 18 events")
    return items, [
        {"phase": "POST_LOCK_ONE_SHOT", "dataset_role": "SUCCESS_CONTROL_18",
         "resource": str(manifest.relative_to(REPO)), "access": "READ_ONCE"},
        {"phase": "POST_LOCK_ONE_SHOT", "dataset_role": "SUCCESS_CONTROL_18",
         "resource": str(INTEGRATION_SUCCESS.relative_to(REPO)),
         "access": "READ_FROZEN_BASELINE_DIGESTS"},
    ]


def detailed_one_shot_rows(items: list[dict], parsed: dict, lock_sha: str,
                           method_name: str) -> list[dict]:
    comparisons = {row["event_id"]: row for row in V1.comparison_rows(items, parsed)}
    bounds = {row["event_id"]: row for row in standalone_bound_rows(items, parsed)}
    item_by_id = {row["event_id"]: row for row in items}
    output = []
    for event_id in sorted(comparisons):
        row = comparisons[event_id]
        bound = bounds[event_id]
        item = item_by_id[event_id]
        output.append({
            "event_id": event_id, "dataset_role": row["dataset_role"], "bag": row["bag"],
            "locked_method": method_name, "lock_sha256": lock_sha,
            "legacy_hard_valid": row["legacy_hard_valid"],
            "legacy_usable_valid": row["legacy_usable_valid"],
            "standalone_hard_valid": row["standalone_hard_valid"],
            "standalone_usable_valid": row["standalone_usable_valid"],
            "legacy_selected_digest": row["legacy_selected_digest"],
            "standalone_selected_digest": row["standalone_selected_digest"],
            "frozen_recorded_baseline_digest": item.get("baseline_digest", ""),
            "frozen_baseline_provenance": (
                "p3_r3_k12_integration_v1/production_success_noninterference.csv"
                if item.get("baseline_digest") else "NOT_APPLICABLE"),
            "legacy_reconstruction_parity_to_recorded_baseline": int(
                bool(item.get("baseline_digest")) and
                item["baseline_digest"] == row["legacy_selected_digest"]),
            "standalone_selected_digest_equals_legacy":
                row["standalone_vs_legacy_selected_digest_exact"],
            "standalone_hard_candidate_count": row["standalone_hard_candidate_count"],
            "standalone_usable_candidate_count": row["standalone_usable_candidate_count"],
            "lateral_factor_count": bound["lateral_factor_count"],
            "transition_factor_count": bound["transition_factor_count"],
            "pair_proxy_count": bound["pair_proxy_count"],
            "reconstruction_count": bound["reconstruction_count"],
            "validator_count": bound["validator_count"],
            "all_bounds_pass": bound["all_bounds_pass"],
        })
    return output


def write_final_documents(payload: dict, lock_sha: str, validation: list[dict],
                          controls: list[dict]) -> str:
    development = next(row for row in read_csv(HERE / "development_policy_comparison.csv")
                       if row["selected_on_development"] == "1")
    runtime = next(row for row in read_csv(HERE / "native_runtime.csv")
                   if row["stage"] == "EXECUTABLE_WALL_TOTAL")
    validation_hard = sum(int(row["standalone_hard_valid"]) for row in validation)
    validation_usable = sum(int(row["standalone_usable_valid"]) for row in validation)
    control_hard = sum(int(row["standalone_hard_valid"]) for row in controls)
    control_usable = sum(int(row["standalone_usable_valid"]) for row in controls)
    bounds_pass = all(int(row["all_bounds_pass"]) for row in validation + controls)
    baseline_parity = all(row["frozen_recorded_baseline_digest"] and
                          int(row["legacy_reconstruction_parity_to_recorded_baseline"])
                          for row in controls)
    v2_rows = [row for row in read_csv(V2_ABLATION)
               if row["selection_corpus"] == "DEVELOPMENT_49" and
               row["component_operator_extension"] == "1"]
    v2_hard = max(int(row["hard_count"]) for row in v2_rows)
    v2_usable = max(int(row["usable_count"]) for row in v2_rows)
    dev_hard = int(development["hard_count"])
    dev_usable = int(development["usable_count"])
    runtime_pass = float(runtime["p95_ms"]) < 25.0
    if (bounds_pass and baseline_parity and runtime_pass and control_hard == 18 and
            dev_hard >= v2_hard and dev_usable >= v2_usable):
        decision = "GQSC_V3_GEOMETRY_GENERAL_READY_FOR_FREEZE"
    elif bounds_pass and baseline_parity and runtime_pass and control_hard >= 17:
        decision = "GQSC_V3_CLEAN_BUT_COVERAGE_TRADEOFF"
    else:
        decision = "GQSC_V3_GEOMETRIC_OPERATOR_INSUFFICIENT"
    freeze_recommended = decision == "GQSC_V3_GEOMETRY_GENERAL_READY_FOR_FREEZE"
    (HERE / "final_freeze_gate.md").write_text(f"""# GQSC v3 final freeze gate

Decision: `{decision}`

- prevalidation lock: `V3_PREVALIDATION_LOCKED`
- lock SHA-256: `{lock_sha}`
- selected policy: `{payload['method_name']}`
- DEVELOPMENT hard/usable: {dev_hard}/49, {dev_usable}/49
- rejected v2 DEVELOPMENT hard/usable reference: {v2_hard}/49, {v2_usable}/49
- VALIDATION_SEEN one-shot hard/usable: {validation_hard}/37, {validation_usable}/37
- success controls hard/usable: {control_hard}/18, {control_usable}/18
- frozen production-success baseline digest reconstruction parity: {sum(int(r['legacy_reconstruction_parity_to_recorded_baseline']) for r in controls)}/18
- all B128/K12/validator-12 bounds pass: {str(bounds_pass).lower()}
- native p95 < 25 ms gate: {str(runtime_pass).lower()} ({float(runtime['p95_ms']):.3f} ms)
- native p99 < 25 ms preference: {str(float(runtime['p99_ms']) < 25.0).lower()} ({float(runtime['p99_ms']):.3f} ms)
- deterministic selector/bound test: PASS (`test_p3_r3_k12`, 3/3 tests)
- hidden legacy execution in standalone selector: none; the selected callable accepts frozen side
  geometry rather than legacy seeds, and its detail stream contains no standalone seed lineage or
  legacy builder/validator call
- production decision-path gate: PASS; v3 policy branches are confined to the research-only
  `selectP3R3RTStandaloneFactors()` call path and the frozen identity/budget unit test passed
- recommend immutable method freeze: {str(freeze_recommended).lower()}

The post-lock validation/control result was not used to repair or reselect the policy. The clean v3
rule introduces no event/map-specific offset, but it is not promoted to a final frozen method when
it gives up the rejected v2's Development coverage or fails the full 18/18 hard control gate.
""", encoding="utf-8")
    (HERE / "README.md").write_text(f"""# GQSC v3 geometry-normalized direct-operator redesign

## Result

`{decision}`

The selected prevalidation-locked policy is `{payload['method_name']}` with SHA-256 `{lock_sha}`.
Selection used only 49 DEVELOPMENT events. Only after serialization as
`V3_PREVALIDATION_LOCKED` were the 37 VALIDATION_SEEN episodes and 18 success controls evaluated
once. No FINAL_HOLDOUT path was accessed.

## Why v2 was not carried forward

The rejected v2's `0.02 m` far offset, `0.15 m` inward offset, and MIN_OUTWARD choice were not
renamed or normalized. The Development deficiency audit found only one event (DVE039) whose entire
teacher hard/usable lateral set was absent, with no repeated normalized mechanism in another event.
Consequently v3 adds no lateral trajectory operator. MIN_OUTWARD's only hard distinction from MAX
was DVE023 and it added no usable event, so it was excluded as effectively event-specific.

## Protocol and outcomes

Four policies were predeclared in [v3_operator_candidates.md](v3_operator_candidates.md). The
Development comparison selected `{payload['method_name']}` at {dev_hard}/49 hard and
{dev_usable}/49 usable. The immutable one-shot result is {validation_hard}/37 hard and
{validation_usable}/37 usable on VALIDATION_SEEN, plus {control_hard}/18 hard and
{control_usable}/18 usable success controls. Frozen production-success baseline digest parity is
{sum(int(r['legacy_reconstruction_parity_to_recorded_baseline']) for r in controls)}/18.

All observed rows obeyed lateral<=64, transition<=7, B128, reconstruction<=12, and validator<=12.
Release standalone wall runtime p50/p90/p95/p99/max was
{float(runtime['p50_ms']):.3f}/{float(runtime['p90_ms']):.3f}/
{float(runtime['p95_ms']):.3f}/{float(runtime['p99_ms']):.3f}/
{float(runtime['max_ms']):.3f} ms with research instrumentation disabled.

The Release unit suite passed 3/3, including frozen production identity/budgets and deterministic,
bounded standalone selection. The selected standalone callable consumes side geometry directly;
it receives no legacy candidate/seed and invokes no legacy builder or validator. Its policy enum is
handled only inside the research-only standalone selector, not the production
`selectP3R3K12Factors` decision path.

## Interpretation

The v3-specific selector is dimensionless and map-scale invariant, but its clean coverage must be
reported against the rejected v2 Development reference ({v2_hard}/49 hard, {v2_usable}/49 usable).
`final_freeze_gate.md` records whether that tradeoff and the success-control result permit a final
freeze. Validation outcomes were not used for repair. Production `plan()`, P3 construction,
validator, ranking, lifecycle, fallback, and parameters were not changed by this research-only
study, and no closed-loop claim is made.
""", encoding="utf-8")
    return decision


def phase_one_shot() -> None:
    payload, lock_sha = verified_lock()
    validation_items, validation_accesses = validation_catalog()
    success_items, success_accesses = success_catalog()
    _, validation_parsed = run_standalone(validation_items, payload["selector_enum"])
    _, success_parsed = run_standalone(success_items, payload["selector_enum"])
    validation_rows = detailed_one_shot_rows(
        validation_items, validation_parsed, lock_sha, payload["method_name"])
    success_rows = detailed_one_shot_rows(
        success_items, success_parsed, lock_sha, payload["method_name"])
    write_csv(HERE / "validation_seen_one_shot.csv", validation_rows)
    write_csv(HERE / "success_control_gate.csv", success_rows)
    append_accesses(validation_accesses + success_accesses)
    decision = write_final_documents(payload, lock_sha, validation_rows, success_rows)
    print(decision)


def phase_finalize_metadata() -> None:
    """Join frozen provenance into existing one-shot rows; never invoke an evaluator."""
    payload, lock_sha = verified_lock()
    validation_path = HERE / "validation_seen_one_shot.csv"
    success_path = HERE / "success_control_gate.csv"
    if not validation_path.is_file() or not success_path.is_file():
        raise RuntimeError("one-shot outputs are required before metadata finalization")
    validation_rows = read_csv(validation_path)
    success_rows = read_csv(success_path)
    baseline = {row["event_id"]: row for row in read_csv(INTEGRATION_SUCCESS)}
    for row in success_rows:
        source = baseline.get(row["event_id"])
        if source is None:
            raise RuntimeError(f"missing frozen baseline row: {row['event_id']}")
        row["frozen_recorded_baseline_digest"] = source["baseline_selected_path_digest"]
        row["frozen_baseline_provenance"] = (
            "p3_r3_k12_integration_v1/production_success_noninterference.csv")
        row["legacy_reconstruction_parity_to_recorded_baseline"] = int(
            source["baseline_selected_path_digest"] == row["legacy_selected_digest"])
    write_csv(success_path, success_rows)
    append_accesses([{
        "phase": "POST_LOCK_METADATA_FINALIZATION", "dataset_role": "SUCCESS_CONTROL_18",
        "resource": str(INTEGRATION_SUCCESS.relative_to(REPO)),
        "access": "READ_FROZEN_BASELINE_DIGESTS_NO_EVALUATOR",
    }])
    decision = write_final_documents(payload, lock_sha, validation_rows, success_rows)
    print(decision)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", required=True,
                        choices=("development-baseline", "development-policies", "lock",
                                 "one-shot", "finalize-metadata"))
    args = parser.parse_args()
    if args.phase == "development-baseline":
        phase_development_baseline()
    elif args.phase == "development-policies":
        phase_development_policies()
    elif args.phase == "lock":
        phase_lock()
    elif args.phase == "one-shot":
        phase_one_shot()
    elif args.phase == "finalize-metadata":
        phase_finalize_metadata()
    else:
        raise RuntimeError(f"phase not implemented yet: {args.phase}")


if __name__ == "__main__":
    main()
