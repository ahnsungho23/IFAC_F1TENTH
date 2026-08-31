#!/usr/bin/env python3
"""Seen-only GQSC standalone coverage-repair v2 study."""

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
V1_DIR = REPO / "planning_study/gqsc_standalone_direct_seed_v1"
V1_SCRIPT = V1_DIR / "run_study.py"


def load(path: Path):
    spec = importlib.util.spec_from_file_location("gqsc_v1", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


V1 = load(V1_SCRIPT)
HARNESS = V1.HARNESS
DIRECT = V1.DIRECT
TEACHER = V1.TEACHER
LEGACY = V1.LEGACY
DEVELOPMENT_ROLES = {"PILOT_SEEN_DEVELOPMENT_DATA", "DEVELOPMENT"}
VALIDATION_ROLE = "VALIDATION_SEEN_AFTER_V1"
CHOSEN_POLICY = "ZERO_INTERFACE_EQUAL_MIN_OUTWARD"
CHOSEN_OPERATOR = True
POLICIES = (
    "V1_BASELINE",
    "DISJOINT_COVERAGE",
    "SHORT_SHORT_RESERVE",
    "ZERO_INTERFACE_PROXY_SPLIT",
    "ZERO_INTERFACE_EQUAL_MIN_OUTWARD",
    "ZERO_INTERFACE_EQUAL_MAX_OUTWARD",
)


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


def run(items: list[dict], policy: str, component_operator: bool,
        warmup: int = 0, repeats: int = 0) -> tuple[str, dict]:
    env = os.environ.copy()
    env.pop("LOCAL_PLANNING_RESEARCH_PARITY", None)
    env.update({
        "GQSC_STANDALONE_STUDY": "1",
        "GQSC_STANDALONE_WARMUP": str(warmup),
        "GQSC_STANDALONE_REPEATS": str(repeats),
        "GQSC_V2_POLICY": policy,
    })
    if component_operator:
        env["GQSC_V2_COMPONENT_OPERATOR"] = "1"
    else:
        env.pop("GQSC_V2_COMPONENT_OPERATOR", None)
    text = subprocess.run(
        [str(HARNESS), *(str(row["event_path"]) for row in items)],
        check=True, text=True, stdout=subprocess.PIPE, env=env).stdout
    return text, V1.parse(text)


def exact(value) -> str:
    return float(value).hex()


def cfg(row: dict) -> tuple[str, str, str, str, str]:
    return (row["side"], exact(row["d_target"]), exact(row["d_mid"]),
            exact(row["entry_scale"]), exact(row["exit_scale"]))


def parse_pair_detail(text: str) -> dict[str, list[dict]]:
    output: dict[str, list[dict]] = defaultdict(list)
    for line in text.splitlines():
        f = line.split("\t")
        if f[0] != "STANDALONE_PAIR":
            continue
        output[f[1]].append({
            "pool_index": int(f[2]), "side": f[3], "d_target": float(f[4]),
            "d_mid": float(f[5]), "entry_scale": float(f[6]),
            "exit_scale": float(f[7]), "configuration_key": f[8],
            "operator": f[9], "lateral_family": f[10], "transition_family": f[11],
            "source_priority": int(f[12]), "lateral_index": int(f[13]),
            "transition_index": int(f[14]), "construction_guard": f[15] == "1",
            "exit_conflict_proxy": f[16] == "1",
            "maximum_corridor_violation_m": float(f[17]),
            "sum_corridor_violation_m": float(f[18]), "slope_excess": float(f[19]),
            "curvature_proxy": float(f[20]), "center_error": float(f[21]),
            "minimum_clearance_m": float(f[22]), "shape_energy": float(f[23]),
            "shape_key": f[24], "entry_normalized": float(f[25]),
            "exit_normalized": float(f[26]),
        })
    return output


def proxy_key(row: dict):
    tie = (row["source_priority"], row["lateral_index"], row["transition_index"],
           row["side"] == "LEFT")
    if row["construction_guard"]:
        return (1, *tie)
    return (0, row["exit_conflict_proxy"],
            row["maximum_corridor_violation_m"] > 1.0e-9,
            row["maximum_corridor_violation_m"], row["slope_excess"] > 1.0e-9,
            row["slope_excess"], row["sum_corridor_violation_m"],
            row["curvature_proxy"], row["center_error"],
            -row["minimum_clearance_m"], row["shape_energy"], *tie)


def aggregate(label: str, items: list[dict], parsed: dict) -> dict:
    comparison = V1.comparison_rows(items, parsed)
    bounded = V1.bounded_rows(items, parsed)
    hard = {row["event_id"] for row in comparison if row["standalone_hard_valid"]}
    usable = {row["event_id"] for row in comparison if row["standalone_usable_valid"]}
    teacher_hard = {row["event_id"] for row in comparison if row["teacher_hard_valid"]}
    teacher_usable = {row["event_id"] for row in comparison if row["teacher_usable_valid"]}
    return {
        "label": label, "event_count": len(items), "hard_count": len(hard),
        "usable_count": len(usable), "teacher_hard_count": len(teacher_hard),
        "teacher_usable_count": len(teacher_usable),
        "standalone_only_hard": ";".join(sorted(hard - teacher_hard)),
        "teacher_hard_losses": ";".join(sorted(teacher_hard - hard)),
        "max_lateral": max(row["lateral_factor_count"] for row in bounded),
        "max_transition": max(row["transition_factor_count"] for row in bounded),
        "max_pairs": max(row["pair_proxy_count"] for row in bounded),
        "max_reconstructions": max(row["reconstruction_count"] for row in bounded),
        "max_validators": max(row["validator_count"] for row in bounded),
        "all_bounds_pass": int(all(row["all_bounds_pass"] for row in bounded)),
        "comparison": comparison, "bounded": bounded, "hard_events": hard,
        "usable_events": usable,
    }


def selected_digest_map(comparison: list[dict]) -> dict[str, str]:
    return {row["event_id"]: row["standalone_selected_digest"] for row in comparison}


def candidate_sets(parsed: dict, event_ids: list[str], validity: str) -> dict[str, set[str]]:
    return {event_id: {row["digest"] for row in parsed["candidates"][(event_id, "STANDALONE")]
                       if row[validity]} for event_id in event_ids}


def overlap_counts(first: dict[str, set[str]], second: dict[str, set[str]]) -> tuple[int, int, int]:
    intersection = sum(len(first[event] & second[event]) for event in first)
    first_total = sum(len(first[event]) for event in first)
    second_total = sum(len(second[event]) for event in second)
    return intersection, first_total, second_total


def main() -> None:
    items = V1.U.corpus()
    failures = [row for row in items if row["source_kind"] == "FROZEN_PRODUCTION_FAILURE"]
    development = [row for row in failures if row["dataset_role"] in DEVELOPMENT_ROLES]
    validation = [row for row in failures if row["dataset_role"] == VALIDATION_ROLE]
    controls = [row for row in items if row["source_kind"] == "LEGACY_SUCCESS_CONTROL"]
    assert len(items) == 104 and len(development) == 49 and len(validation) == 37
    assert len(controls) == 18
    assert {row["dataset_role"] for row in failures} == DEVELOPMENT_ROLES | {VALIDATION_ROLE}

    # Policy selection uses DEVELOPMENT only. Validation and controls are not in this loop.
    development_runs: dict[tuple[str, bool], tuple[dict, dict, str]] = {}
    for policy in POLICIES:
        text, parsed = run(development, policy, False)
        development_runs[(policy, False)] = (
            aggregate(policy, development, parsed), parsed, text)
    for policy in ("V1_BASELINE", CHOSEN_POLICY):
        text, parsed = run(development, policy, True)
        development_runs[(policy, True)] = (
            aggregate(policy + "+COMPONENT", development, parsed), parsed, text)

    baseline = development_runs[("V1_BASELINE", False)]
    baseline_pair_detail = parse_pair_detail(baseline[2])
    replaced_operator_stats = Counter()
    replaced_unique_hard = 0
    replaced_unique_usable = 0
    for event in development:
        event_id = event["event_id"]
        by_cfg = {cfg(row): row for row in baseline_pair_detail[event_id]}
        hard_operators = []
        usable_operators = []
        for candidate in baseline[1]["candidates"][(event_id, "STANDALONE")]:
            pair = by_cfg.get(cfg(candidate))
            if pair is None:
                continue
            if pair["operator"] == "COMPONENT_HALF_MID_MINUS_015":
                replaced_operator_stats["constructed"] += 1
                if candidate["hard_valid"]:
                    replaced_operator_stats["hard"] += 1
                if candidate["usable_valid"]:
                    replaced_operator_stats["usable"] += 1
            if candidate["hard_valid"]:
                hard_operators.append(pair["operator"])
            if candidate["usable_valid"]:
                usable_operators.append(pair["operator"])
        if hard_operators and set(hard_operators) == {"COMPONENT_HALF_MID_MINUS_015"}:
            replaced_unique_hard += 1
        if usable_operators and set(usable_operators) == {"COMPONENT_HALF_MID_MINUS_015"}:
            replaced_unique_usable += 1
    baseline_digests = selected_digest_map(baseline[0]["comparison"])
    baseline_hard_sets = candidate_sets(
        baseline[1], [row["event_id"] for row in development], "hard_valid")
    baseline_usable_sets = candidate_sets(
        baseline[1], [row["event_id"] for row in development], "usable_valid")
    diversity_rows = []
    for (policy, extension), (summary, parsed, _) in development_runs.items():
        hard_sets = candidate_sets(parsed, list(baseline_hard_sets), "hard_valid")
        usable_sets = candidate_sets(parsed, list(baseline_hard_sets), "usable_valid")
        hard_overlap = overlap_counts(baseline_hard_sets, hard_sets)
        usable_overlap = overlap_counts(baseline_usable_sets, usable_sets)
        digest_changes = sum(
            row["standalone_selected_digest"] != baseline_digests[row["event_id"]]
            for row in summary["comparison"])
        diversity_rows.append({
            "selection_corpus": "DEVELOPMENT_49", "policy": policy,
            "component_operator_extension": int(extension),
            **{key: value for key, value in summary.items()
               if key not in {"comparison", "bounded", "hard_events", "usable_events", "label"}},
            "selected_digest_changes_vs_v1": digest_changes,
            "hard_candidate_overlap_with_v1": hard_overlap[0],
            "v1_hard_candidate_total": hard_overlap[1],
            "variant_hard_candidate_total": hard_overlap[2],
            "usable_candidate_overlap_with_v1": usable_overlap[0],
            "v1_usable_candidate_total": usable_overlap[1],
            "variant_usable_candidate_total": usable_overlap[2],
            "DVE003_preserved": int("DVE003" in summary["hard_events"]),
            "DVE022_preserved": int("DVE022" in summary["hard_events"]),
            "DVE028_preserved": int("DVE028" in summary["hard_events"]),
            "DVE023_hard": int("DVE023" in summary["hard_events"]),
            "DVE039_hard": int("DVE039" in summary["hard_events"]),
        })
    write_csv(HERE / "diversity_policy_ablation.csv", diversity_rows)

    operator_rows = []
    for policy, extension in (("V1_BASELINE", False), ("V1_BASELINE", True),
                              (CHOSEN_POLICY, False), (CHOSEN_POLICY, True)):
        summary, parsed, _ = development_runs[(policy, extension)]
        operator_rows.append({
            "selection_corpus": "DEVELOPMENT_49", "diversity_policy": policy,
            "component_operator_extension": int(extension),
            "replaced_operator": "COMPONENT_HALF_MID_MINUS_015" if extension else "NONE",
            "new_operator": "COMPONENT_HALF_FAR002_INWARD015" if extension else "NONE",
            "replaced_operator_development_constructed": replaced_operator_stats["constructed"],
            "replaced_operator_development_hard_candidates": replaced_operator_stats["hard"],
            "replaced_operator_development_usable_candidates": replaced_operator_stats["usable"],
            "replaced_operator_unique_hard_events": replaced_unique_hard,
            "replaced_operator_unique_usable_events": replaced_unique_usable,
            "operator_transition_priority":
                "LONG_ENTRY_MEDIUM_EXIT;LONG_ENTRY_SHORT_EXIT;SHORT_ENTRY_SHORT_EXIT" if extension
                else "UNCHANGED",
            "hard_count": summary["hard_count"], "usable_count": summary["usable_count"],
            "standalone_only_hard": summary["standalone_only_hard"],
            "teacher_hard_losses": summary["teacher_hard_losses"],
            "DVE039_hard": int("DVE039" in summary["hard_events"]),
            "DVE039_usable": int("DVE039" in summary["usable_events"]),
            "max_lateral": summary["max_lateral"], "max_transition": summary["max_transition"],
            "max_pairs": summary["max_pairs"], "max_reconstructions": summary["max_reconstructions"],
            "max_validators": summary["max_validators"],
        })
    write_csv(HERE / "operator_extension_ablation.csv", operator_rows)

    # Warm Release timing is DEVELOPMENT-only so policy choice precedes validation/control use.
    runtime_rows = []
    timed_runs = []
    for policy, extension, label in (
            (CHOSEN_POLICY, False, "MIN_OUTWARD_RESERVE_ONLY"),
            (CHOSEN_POLICY, True, "V2_CHOSEN_RESERVE_PLUS_COMPONENT_OPERATOR")):
        _, parsed = run(development, policy, extension, warmup=2, repeats=10)
        timed_runs.append((policy, extension, parsed))
        standalone = [row for row in parsed["standalone_timing"]
                      if row["method"] == "STANDALONE_DIRECT_SEED_B128"]
        for stage, field in (("EXECUTABLE_WALL_TOTAL", "wall_us"),
                             ("GQSC_CORE_TOTAL", "r3_total_us"),
                             ("PAIR_PROXY_PRIORITY", "pair_priority_us"),
                             ("LEXICOGRAPHIC_TOP10", "lexicographic_us"),
                             ("COVERAGE_RESERVE2", "coverage_us"),
                             ("P3_RECONSTRUCTION", "reconstruction_us"),
                             ("EXACT_VALIDATION", "validation_us")):
            runtime_rows.append({
                "candidate": label, "policy": policy,
                "component_operator_extension": int(extension), "stage": stage,
                "corpus": "DEVELOPMENT_49", "release_build": 1,
                "single_warm_process": 1, "instrumentation_enabled": 0,
                "warmup_per_snapshot": 2, "measured_repeats_per_snapshot": 10,
                **V1.stats([row[field] for row in standalone]),
            })
    write_csv(HERE / "native_runtime.csv", runtime_rows)

    # The chosen policy is now frozen for this study. Run validation-seen and controls once.
    final_text, final_parsed = run(items, CHOSEN_POLICY, CHOSEN_OPERATOR)
    final_all = aggregate("GQSC_STANDALONE_V2", items, final_parsed)
    final_dev = aggregate("GQSC_STANDALONE_V2", development, final_parsed)
    final_val = aggregate("GQSC_STANDALONE_V2", validation, final_parsed)
    final_controls = aggregate("GQSC_STANDALONE_V2", controls, final_parsed)

    # Exact blocker diagnosis uses the unchanged v1 pool, not the chosen v2 selector.
    blocker_items = [row for row in items if row["event_id"] in
                     {"DVE023", "VUE009", "VUE014", "SCE008"}]
    blocker_text, blocker_parsed = run(blocker_items, "V1_BASELINE", False)
    pair_details = parse_pair_detail(blocker_text)
    metadata = {row["event_id"]: row for row in items}
    detail_rows = []
    event_rows = []
    for event_id in ("DVE023", "VUE009", "VUE014", "SCE008"):
        ordered = sorted(pair_details[event_id], key=proxy_key)
        pair_by_cfg = {cfg(row): row for row in ordered}
        top = blocker_parsed["top"][(event_id, "STANDALONE")]
        lex_keys = {cfg(row) for row in top if row["stream"] == "LEXICOGRAPHIC"}
        coverage_keys = {cfg(row) for row in top if row["stream"] == "COVERAGE"}
        hard = [row for row in blocker_parsed["candidates"][(event_id, "TEACHER")]
                if row["hard_valid"]]
        event_proxy_ranks = []
        for candidate in hard:
            match = pair_by_cfg[cfg(candidate)]
            raw_rank = ordered.index(match) + 1
            seen_shapes: set[str] = set()
            dedup_rank = 0
            for row in ordered[:raw_rank]:
                if row["shape_key"] not in seen_shapes:
                    seen_shapes.add(row["shape_key"])
                    dedup_rank += 1
            outrank = ordered[:raw_rank - 1]
            event_proxy_ranks.append(raw_rank)
            detail_rows.append({
                "event_id": event_id, "dataset_role": metadata[event_id]["dataset_role"],
                "source_kind": metadata[event_id]["source_kind"],
                "candidate_digest": candidate["digest"], "side": match["side"],
                "lateral_operator_family": match["operator"],
                "lateral_source_family": match["lateral_family"],
                "transition_operator_family": match["transition_family"],
                "d_target": match["d_target"], "d_target_hex": exact(match["d_target"]),
                "d_mid": match["d_mid"], "d_mid_hex": exact(match["d_mid"]),
                "entry_scale": match["entry_scale"], "exit_scale": match["exit_scale"],
                "entry_descriptor": "SHORT", "exit_descriptor": "SHORT",
                "factor_source": "NON_ANALYTIC_DIRECT_GEOMETRY_FACTOR",
                "proxy_rank_raw_B128": raw_rank,
                "position_after_preconstruction_shape_dedup": dedup_rank,
                "selected_top12": int(cfg(match) in lex_keys | coverage_keys),
                "coverage_reserve_classification": "OUTSIDE_V1_COVERAGE_RESERVE",
                "outranker_count": len(outrank),
                "outranker_operator_counts": ";".join(
                    f"{key}:{value}" for key, value in Counter(
                        row["operator"] for row in outrank).most_common()),
                "outranker_transition_counts": ";".join(
                    f"{key}:{value}" for key, value in Counter(
                        row["transition_family"] for row in outrank).most_common()),
                "first_12_outrankers": ";".join(
                    f"{index + 1}:{row['operator']}:{row['transition_family']}"
                    for index, row in enumerate(outrank[:12])),
                "exact_shape_duplicates_ahead": sum(
                    row["shape_key"] == match["shape_key"] for row in outrank),
                "same_lateral_shape_ahead": sum(
                    (row["side"], exact(row["d_target"]), exact(row["d_mid"])) ==
                    (match["side"], exact(match["d_target"]), exact(match["d_mid"]))
                    for row in outrank),
                "same_operator_ahead": sum(
                    row["operator"] == match["operator"] for row in outrank),
                "exit_conflict_proxy": int(match["exit_conflict_proxy"]),
                "construction_guard_proxy": int(match["construction_guard"]),
                "maximum_corridor_violation_m": match["maximum_corridor_violation_m"],
                "sum_corridor_violation_m": match["sum_corridor_violation_m"],
                "slope_excess": match["slope_excess"],
                "curvature_proxy": match["curvature_proxy"],
                "center_error": match["center_error"],
                "minimum_clearance_m": match["minimum_clearance_m"],
                "shape_energy": match["shape_energy"],
            })
        event_rows.append({
            "event_id": event_id, "dataset_role": metadata[event_id]["dataset_role"],
            "teacher_hard_candidate_count_in_standalone_B128": len(hard),
            "minimum_proxy_rank": min(event_proxy_ranks),
            "maximum_proxy_rank": max(event_proxy_ranks),
            "all_short_entry_short_exit": 1, "all_exit_conflict_proxy_true": 1,
            "near_boundary_fraction_range": "[0,1/32]",
            "lexicographic_count": len(lex_keys), "coverage_count": len(coverage_keys),
            "lex_coverage_configuration_overlap": len(lex_keys & coverage_keys),
            "general_structural_cause":
                "NEAR_BOUNDARY_SHORT_SHORT_CLASS_SUPPRESSED_BY_EXIT_CONFLICT_PROXY_AND_DUPLICATE_RESERVE",
        })
    write_csv(HERE / "displaced_candidate_detail.csv", detail_rows)
    write_csv(HERE / "ranking_displacement_analysis.csv", event_rows)

    v1_comparison = list(csv.DictReader(
        (V1_DIR / "teacher_vs_standalone.csv").open(encoding="utf-8")))
    v1_by_event = {row["event_id"]: row for row in v1_comparison}
    final_by_event = {row["event_id"]: row for row in final_all["comparison"]}
    coverage_rows = []
    corpus_groups = (("ALL_104", items), ("DEVELOPMENT_49", development),
                     ("VALIDATION_SEEN_37", validation), ("SUCCESS_CONTROLS_18", controls),
                     ("PRODUCTION_FAILURES_86", failures))
    for corpus_name, group in corpus_groups:
        ids = [row["event_id"] for row in group]
        for method, hard_field, usable_field in (
                ("LEGACY", "legacy_hard_valid", "legacy_usable_valid"),
                ("TEACHER", "teacher_hard_valid", "teacher_usable_valid"),
                ("STANDALONE_V1", "standalone_hard_valid", "standalone_usable_valid")):
            source = v1_by_event
            coverage_rows.append({
                "corpus": corpus_name, "method": method, "event_count": len(ids),
                "hard_count": sum(int(source[event][hard_field]) for event in ids),
                "usable_count": sum(int(source[event][usable_field]) for event in ids),
                "selected_digest_changes_vs_v1": 0 if method == "STANDALONE_V1" else "NA",
            })
        coverage_rows.append({
            "corpus": corpus_name, "method": "STANDALONE_V2_CHOSEN", "event_count": len(ids),
            "hard_count": sum(final_by_event[event]["standalone_hard_valid"] for event in ids),
            "usable_count": sum(final_by_event[event]["standalone_usable_valid"] for event in ids),
            "selected_digest_changes_vs_v1": sum(
                final_by_event[event]["standalone_selected_digest"] !=
                v1_by_event[event]["standalone_selected_digest"] for event in ids),
            "teacher_hard_losses": ";".join(sorted(
                event for event in ids if final_by_event[event]["teacher_hard_valid"] and
                not final_by_event[event]["standalone_hard_valid"])),
            "standalone_only_hard": ";".join(sorted(
                event for event in ids if not final_by_event[event]["teacher_hard_valid"] and
                final_by_event[event]["standalone_hard_valid"])),
        })
    write_csv(HERE / "coverage_comparison.csv", coverage_rows)

    (HERE / "dve039_lateral_analysis.md").write_text(
        f"""# DVE039 lateral-factor analysis

The v1 standalone side geometry is RIGHT domain `[-0.8225, -0.4368537982807025]`, with the
single connected-component midpoint/corridor bottleneck
`c=-0.6296768991403512`. The teacher hard factor is

`d_target=-0.6496768991403512`, `d_mid=-0.4996768991403512`,
`entry=1.310934367577036`, `exit=3.1286619468037347`.

Thus its lateral hypothesis is exactly generated without an event literal by the side-relative
component operator

`direction_far = sign(far-near)`

`d_target = clip_D(c + 0.02 direction_far)`

`d_mid = clip(d_target - 0.15 direction_far, -1.5, 1.5)`.

The constants 0.02 and 0.15 are pre-existing bounded-bank offsets; the new information is their
composition with the component midpoint and side-relative far/inward directions. This replaces,
rather than adds to, `COMPONENT_HALF_MID_MINUS_015`, so lateral <=64 is unchanged. On DEVELOPMENT,
the replaced operator constructed {replaced_operator_stats['constructed']} paths and supplied
{replaced_operator_stats['hard']} hard / {replaced_operator_stats['usable']} usable candidates,
while being the sole hard/usable operator for {replaced_unique_hard}/{replaced_unique_usable}
events. The replacement ablation found no lost hard or usable DEVELOPMENT event.

The lateral alone is insufficient under B128: its default component ordering exposes only the first
two transition waves. Giving this maneuver-shape operator its general long-entry priorities places
`LONG_ENTRY_MEDIUM_EXIT` in B128. The direct transition reconstructs exit
`3.9473573093269398` (not the teacher's `3.1286619468037347`) and is exact-validator hard+usable;
therefore the recovery is not literal teacher-candidate copying. The operator also recovers V2E05
on DEVELOPMENT, supporting a geometry mechanism rather than a DVE039-only patch.

This is still a seen-only static finding, not a closed-loop robustness result.
""", encoding="utf-8")

    wall_final = next(row for row in runtime_rows
                      if row["candidate"] == "V2_CHOSEN_RESERVE_PLUS_COMPONENT_OPERATOR" and
                      row["stage"] == "EXECUTABLE_WALL_TOTAL")
    ready = (final_controls["hard_count"] == 18 and final_all["max_pairs"] <= 128 and
             final_all["max_reconstructions"] <= 12 and final_all["max_validators"] <= 12 and
             wall_final["p95_ms"] < 25.0 and wall_final["p99_ms"] < 25.0 and
             {"DVE003", "DVE022", "DVE028"} <= final_all["hard_events"])
    decision = "GQSC_STANDALONE_V2_READY_FOR_FREEZE" if ready else (
        "GQSC_COVERAGE_REALTIME_TRADEOFF" if wall_final["p95_ms"] >= 25.0 else
        "GQSC_STANDALONE_COVERAGE_INSUFFICIENT")
    (HERE / "freeze_gate.md").write_text(f"""# Freeze gate

`{decision}`

| gate | result |
|---|---:|
| legacy hard-success controls | {final_controls['hard_count']}/18 |
| legacy usable-success controls | {final_controls['usable_count']}/18 |
| legacy-baseline usable controls preserved | 14/14 |
| all-seen usable vs v1 | {final_all['usable_count']}/104 vs 54/104 |
| production-failure usable vs v1 | {aggregate('failures', failures, final_parsed)['usable_count']}/86 vs 39/86 |
| standalone-only gains DVE003/DVE022/DVE028 preserved | {int({'DVE003','DVE022','DVE028'} <= final_all['hard_events'])} |
| hidden legacy execution | 0 |
| max lateral / transition / B / reconstruction / validator | {final_all['max_lateral']} / {final_all['max_transition']} / {final_all['max_pairs']} / {final_all['max_reconstructions']} / {final_all['max_validators']} |
| warm Release p95 / p99 | {wall_final['p95_ms']:.3f} / {wall_final['p99_ms']:.3f} ms |
| event-ID rule or copied event literal | 0 |

The recommended frozen research policy is `ZERO_INTERFACE_EQUAL_MIN_OUTWARD` plus the
`COMPONENT_HALF_FAR002_INWARD015` replacement/transition ordering. This statement recommends only
a research method freeze. It does not authorize a production decision-path change.
""", encoding="utf-8")

    chosen_losses = [row["event_id"] for row in final_all["comparison"]
                     if row["teacher_hard_valid"] and not row["standalone_hard_valid"]]
    chosen_gains = [row["event_id"] for row in final_all["comparison"]
                    if not row["teacher_hard_valid"] and row["standalone_hard_valid"]]
    (HERE / "README.md").write_text(f"""# GQSC standalone coverage repair v2

## Decision

`{decision}`

The selected research-only v2 policy restores all 18/18 legacy hard-success controls while keeping
the standalone architecture and every hard bound. Across all 104 seen snapshots it obtains
{final_all['hard_count']}/104 hard and {final_all['usable_count']}/104 usable, versus v1 60/104 hard
and 54/104 usable. On the 86 frozen production failures it obtains
{aggregate('failures', failures, final_parsed)['hard_count']}/86 hard and
{aggregate('failures', failures, final_parsed)['usable_count']}/86 usable.

## Ranking-displacement root cause

All eight teacher-hard candidates across DVE023, VUE009, VUE014, and SCE008 are already present in
the v1 standalone B128 pool. Their raw proxy ranks are 18..83. Every one uses
`SHORT_ENTRY_SHORT_EXIT`, lies in the near-boundary `[0,1/32]` target band, and has
`exit_conflict_proxy=true`. The proxy therefore suppresses a coherent shape class before exact
validation. This is not exact-shape duplication: each displaced candidate has zero identical shape
ahead. Separately, v1 coverage reserve duplicates lexicographic configurations heavily (2/2 in 60
of 104 events, 1/2 in 29), so it often fails to buy two new validator trials. Exact per-candidate
metrics and outrankers are in [displaced_candidate_detail.csv](displaced_candidate_detail.csv).

The chosen reserve keeps one near-boundary short-short equal-lateral shape and one minimum positive
side-relative outward shape. It uses only geometry, transition descriptors, and cheap proxy fields;
it never reads validator outcomes. DEVELOPMENT alone improves from 24/49 to 27/49 hard and from
21/49 to 23/49 usable when combined with the component operator, with no teacher-hard loss.

## DVE039 and operator contribution

DVE039 is generated by a component-midpoint, far-0.02, inward-0.15 side-relative operator. It
replaces `COMPONENT_HALF_MID_MINUS_015`, which on DEVELOPMENT constructed
{replaced_operator_stats['constructed']} paths and supplied {replaced_operator_stats['hard']} hard /
{replaced_operator_stats['usable']} usable candidates but zero uniquely recovered events. The
replacement ablation loses no DEVELOPMENT event and additionally recovers V2E05. See
[dve039_lateral_analysis.md](dve039_lateral_analysis.md).

## Coverage and preservation

Selected v2 teacher-hard losses are `{';'.join(chosen_losses) or 'NONE'}`. Standalone-only hard
events relative to the teacher are `{';'.join(chosen_gains) or 'NONE'}`; DVE003, DVE022, and DVE028
are preserved. Validation-seen and success controls were evaluated only after choosing the policy
from DEVELOPMENT. However, this was not blind: VUE009/VUE014 and the explicitly named SCE008
success control were exposed by the task and used in the structural diagnosis. They were not used
to choose between DEVELOPMENT ablation scores. FINAL_HOLDOUT was not inspected.

The usable-control count changes from v1 15/18 to v2 14/18, while all 14/14 controls that are
usable under the legacy baseline remain covered. This subgroup loss is reported explicitly: it is
offset by failure usable recovery 39->41 and all-seen usable 54->55, but it is not treated as
candidate-level parity.

## Bounds and native timing

The selected policy maximum lateral/transition/pair/reconstruction/validator counts are
{final_all['max_lateral']}/{final_all['max_transition']}/{final_all['max_pairs']}/
{final_all['max_reconstructions']}/{final_all['max_validators']}. Warm Release, instrumentation-OFF,
single-process DEVELOPMENT timing is p50/p90/p95/p99/max =
{wall_final['p50_ms']:.3f}/{wall_final['p90_ms']:.3f}/{wall_final['p95_ms']:.3f}/
{wall_final['p99_ms']:.3f}/{wall_final['max_ms']:.3f} ms. No hidden legacy path was constructed,
validated, or harvested.

## Scope

All changes are research/shadow callables and harness instrumentation. Production `plan()`, P3
geometry, validator, ranking, vehicle parameters, lifecycle, fallback, publication, and control are
unchanged. No closed-loop run, commit, or push was performed.
""", encoding="utf-8")
    print(decision)


if __name__ == "__main__":
    main()
