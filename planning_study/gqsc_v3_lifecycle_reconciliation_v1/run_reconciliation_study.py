#!/usr/bin/env python3
"""Reproduce the seen-only GQSC-v3 lifecycle reconciliation audit.

The corpus loader is deliberately reused from the frozen main-integration study.  That loader
enumerates exactly four DEVELOPMENT/VALIDATION_SEEN/success-control directories and has no final
holdout path or discovery fallback.
"""

from __future__ import annotations

import csv
import importlib.util
from pathlib import Path


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
BASE_SCRIPT = REPO / "planning_study/gqsc_v3_main_integration_v1/run_integration_study.py"
FIVE = {"DVE004", "DVE006", "DVE007", "VUE011", "VUE013"}


def load_base():
    spec = importlib.util.spec_from_file_location("gqsc_main_integration_study", BASE_SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {BASE_SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_csv(name: str, rows: list[dict], fields: list[str] | None = None) -> None:
    if not rows:
        raise RuntimeError(f"refusing to write empty {name}")
    selected_fields = fields if fields is not None else list(rows[0])
    with (HERE / name).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=selected_fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def parse_parity(lines: list[str], roles: dict[str, str]) -> dict[str, dict]:
    names = (
        "record", "event_id", "lateral_order_exact", "transition_order_exact",
        "pair_order_exact", "top12_exact", "path_digests_verdicts_exact",
        "selected_path_exact", "plan_selected_path_exact", "no_hidden_legacy_seed",
        "snapshot_lineage_exact", "method_sha_exact", "bounded_contract_pass",
        "pair_proxy_count", "reconstruction_count", "validator_count", "legacy_seed_count",
        "hard_valid_count", "selected_path_digest", "failure_classification")
    parsed: dict[str, dict] = {}
    for line in lines:
        values = line.split("\t")
        if len(values) != len(names) or values[0] != "GQSC_MAIN_PARITY":
            raise RuntimeError(f"unexpected parity row: {line}")
        row = dict(zip(names, values))
        row["dataset_role"] = roles[row["event_id"]]
        parsed[row["event_id"]] = row
    return parsed


def parse_sequence(lines: list[str]) -> dict[str, list[dict]]:
    names = (
        "record", "event_id", "scenario", "step", "state", "has_output",
        "fresh_selected", "invalidated", "complete", "suffix_revalidated",
        "suffix_hard_valid", "guard_raw_revalidated", "raw_validation_attempted",
        "original_candidate_identity", "original_path_digest", "output_path_digest",
        "suffix_point_count", "fallback_kind", "evaluator_collision_horizon_m",
        "lifecycle_collision_horizon_m", "reason")
    parsed: dict[str, list[dict]] = {}
    for line in lines:
        values = line.split("\t")
        if values[0] == "GQSC_SEQUENCE_SKIP" and len(values) == 4:
            parsed[values[1]] = [{
                "record": values[0], "event_id": values[1], "scenario": "NOT_ELIGIBLE",
                "step": "SKIP", "reason": values[2] + ":" + values[3],
            }]
            continue
        if len(values) != len(names) or values[0] != "GQSC_SEQUENCE":
            raise RuntimeError(f"unexpected sequence row: {line}")
        row = dict(zip(names, values))
        parsed.setdefault(row["event_id"], []).append(row)
    return parsed


def parse_invocations(lines: list[str], roles: dict[str, str]) -> dict[str, dict]:
    names = (
        "record", "event_id", "primary_evaluations", "fallback_new_evaluations",
        "safe_stop_escape_evaluations", "same_input_cache_reuses", "new_evaluations_total",
        "same_input_duplicate_evaluations", "outcome")
    parsed: dict[str, dict] = {}
    for line in lines:
        values = line.split("\t")
        if len(values) != len(names) or values[0] != "GQSC_INVOCATION":
            raise RuntimeError(f"unexpected invocation row: {line}")
        row = dict(zip(names, values))
        row["dataset_role"] = roles[row["event_id"]]
        row["counterfactual_pre_fix_new_evaluations"] = str(
            int(row["new_evaluations_total"]) + int(row["same_input_cache_reuses"]))
        row["fallback_inputs_identical_to_primary"] = row["same_input_cache_reuses"]
        row["fallback_requires_new_primary_evaluation"] = "0"
        parsed[row["event_id"]] = row
    return parsed


def post_fix_lifecycle(
    parity: dict[str, dict], sequence: dict[str, list[dict]], roles: dict[str, str]
) -> list[dict]:
    rows: list[dict] = []
    exact_keys = (
        "lateral_order_exact", "transition_order_exact", "pair_order_exact", "top12_exact",
        "path_digests_verdicts_exact", "selected_path_exact", "plan_selected_path_exact",
        "no_hidden_legacy_seed", "snapshot_lineage_exact", "method_sha_exact",
        "bounded_contract_pass")
    for event_id in sorted(parity):
        event_rows = sequence[event_id]
        eligible = event_rows[0]["step"] != "SKIP"
        by_step = {row["step"]: row for row in event_rows} if eligible else {}
        fresh_rows = [row for row in event_rows if row.get("step") == "FRESH"]
        horizons_exact = eligible and len(fresh_rows) == 2 and all(
            abs(float(row["evaluator_collision_horizon_m"]) -
                float(row["lifecycle_collision_horizon_m"])) <= 1.0e-9
            for row in fresh_rows)
        rows.append({
            "event_id": event_id,
            "dataset_role": roles[event_id],
            "frozen_stateless_parity_exact": int(all(parity[event_id][key] == "1" for key in exact_keys)),
            "gqsc_hard_valid_exists": int(eligible),
            "fresh_ownership_success": int(eligible and all(row["has_output"] == "1" for row in fresh_rows)),
            "evaluator_lifecycle_horizon_exact": int(horizons_exact),
            "same_input_continuation_success": int(eligible and by_step["HELD_SAME_INPUT"]["has_output"] == "1"),
            "guard_raw_path_preserved": int(eligible and by_step["GUARD_FAIL_RAW_PASS"]["has_output"] == "1"),
            "dropout_continuation_success": int(eligible and by_step["OBSTACLE_DISAPPEARANCE"]["has_output"] == "1"),
            "prefix_trim_success": int(eligible and by_step["FORWARD_TRIM"]["has_output"] == "1"),
            "completion_success": int(eligible and by_step["COMPLETE"]["complete"] == "1"),
            "new_blocker_invalidation_success": int(
                eligible and by_step["INVALIDATE_AND_FALLBACK"]["invalidated"] == "1"),
            "pair_proxy_count": parity[event_id]["pair_proxy_count"],
            "reconstruction_count": parity[event_id]["reconstruction_count"],
            "validator_count": parity[event_id]["validator_count"],
            "selected_path_digest": parity[event_id]["selected_path_digest"],
            "reason": event_rows[0].get("reason", "NONE") if not eligible else "POST_FIX_ALL_GATES_PASS",
        })
    return rows


def five_event_rows(lines: list[str], post: dict[str, dict]) -> list[dict]:
    names = (
        "record", "event_id", "ego_s", "ego_d", "ego_speed", "selected_path_digest",
        "z0", "z1", "z2", "z3", "z4", "cluster_start_forward_m",
        "cluster_end_forward_m", "frozen_evaluator_horizon_m",
        "pre_fix_lifecycle_raw_horizon_m", "path_point_count", "evaluator_sample_count",
        "pre_fix_lifecycle_sample_count", "path_last_forward_m", "next_obstacle_id",
        "next_obstacle_start_forward_m", "next_obstacle_end_forward_m", "evaluator_valid",
        "pre_fix_lifecycle_valid", "pre_fix_failure_obstacle_id",
        "pre_fix_failure_waypoint_index", "pre_fix_failure_waypoint_s",
        "pre_fix_failure_waypoint_d", "pre_fix_failure_obstacle_start",
        "pre_fix_failure_obstacle_end", "visible_obstacle_intervals",
        "evaluator_obstacle_intervals", "pre_fix_lifecycle_obstacle_intervals")
    rows: list[dict] = []
    for line in lines:
        values = line.split("\t")
        if len(values) != len(names) or values[0] != "GQSC_HORIZON":
            raise RuntimeError(f"unexpected horizon row: {line}")
        row = dict(zip(names, values))
        row["pre_fix_classification"] = "LIFECYCLE_REVALIDATION_OVERREACH"
        row["post_fix_lifecycle_horizon_m"] = row["frozen_evaluator_horizon_m"]
        row["post_fix_fresh_ownership_success"] = post[row["event_id"]]["fresh_ownership_success"]
        rows.append(row)
    return rows


def runtime_rows(
    lines: list[str], invocations: dict[str, dict], roles: dict[str, str]
) -> list[dict]:
    names = (
        "record", "event_id", "repeat", "evaluation_wall_us", "gqsc_generation_us",
        "exact_validation_us", "final_ranking_us", "lifecycle_us", "fallback_us",
        "callback_total_us", "pair_proxy_count", "reconstruction_count", "validator_count",
        "outcome")
    rows: list[dict] = []
    for line in lines:
        values = line.split("\t")
        if len(values) != len(names) or values[0] != "GQSC_MAIN_TIMING":
            raise RuntimeError(f"unexpected timing row: {line}")
        source = dict(zip(names, values))
        event_id = source["event_id"]
        row = {
            "event_id": event_id,
            "dataset_role": roles[event_id],
            "repeat": source["repeat"],
            "evaluation_wall_ms": float(source["evaluation_wall_us"]) / 1000.0,
            "gqsc_generation_ms": float(source["gqsc_generation_us"]) / 1000.0,
            "exact_validation_ms": float(source["exact_validation_us"]) / 1000.0,
            "final_ranking_ms": float(source["final_ranking_us"]) / 1000.0,
            "lifecycle_ms": float(source["lifecycle_us"]) / 1000.0,
            "fallback_ms": float(source["fallback_us"]) / 1000.0,
            "callback_total_ms": float(source["callback_total_us"]) / 1000.0,
            "new_gqsc_evaluations": invocations[event_id]["new_evaluations_total"],
            "same_input_cache_reuses": invocations[event_id]["same_input_cache_reuses"],
            "safe_stop_escape_evaluations": invocations[event_id]["safe_stop_escape_evaluations"],
            "pair_proxy_count": source["pair_proxy_count"],
            "reconstruction_count": source["reconstruction_count"],
            "validator_count": source["validator_count"],
            "outcome": source["outcome"],
        }
        rows.append(row)
    return rows


def main() -> None:
    base = load_base()
    paths, roles = base.events()
    selected = [path for path in paths if path.stem in FIVE]
    if {path.stem for path in selected} != FIVE:
        raise RuntimeError("five-event diagnosis corpus incomplete")

    parity = parse_parity(base.run("GQSC_V3_MAIN_PARITY", paths), roles)
    sequence = parse_sequence(base.run("GQSC_V3_SEQUENTIAL_REPLAY", paths))
    invocations = parse_invocations(base.run("GQSC_V3_INVOCATION_AUDIT", paths), roles)
    post_rows = post_fix_lifecycle(parity, sequence, roles)
    post_by_id = {row["event_id"]: row for row in post_rows}
    diagnosis = five_event_rows(
        base.run("GQSC_V3_HORIZON_DIAGNOSIS", selected), post_by_id)
    timings = runtime_rows(base.run("GQSC_V3_MAIN_TIMING", paths, {
        "GQSC_V3_TIMING_WARMUP": "2", "GQSC_V3_TIMING_REPEATS": "7"}),
        invocations, roles)

    invocation_rows = [invocations[event_id] for event_id in sorted(invocations)]
    write_csv("five_event_diagnosis.csv", diagnosis)
    write_csv("duplicate_invocation_analysis.csv", invocation_rows, [
        "event_id", "dataset_role", "primary_evaluations", "fallback_new_evaluations",
        "safe_stop_escape_evaluations", "same_input_cache_reuses", "new_evaluations_total",
        "same_input_duplicate_evaluations", "counterfactual_pre_fix_new_evaluations",
        "fallback_inputs_identical_to_primary", "fallback_requires_new_primary_evaluation",
        "outcome"])
    write_csv("post_fix_lifecycle_results.csv", post_rows)
    write_csv("callback_runtime.csv", timings)

    callback_values = [float(row["callback_total_ms"]) for row in timings]
    evaluation_values = [float(row["evaluation_wall_ms"]) for row in timings]
    print("PARITY", len(parity), sum(row["frozen_stateless_parity_exact"] for row in post_rows))
    eligible = [row for row in post_rows if row["gqsc_hard_valid_exists"]]
    print("LIFECYCLE", len(eligible), sum(row["fresh_ownership_success"] for row in eligible))
    print("HORIZON_EXACT", sum(row["evaluator_lifecycle_horizon_exact"] for row in eligible))
    print("CACHE_REUSE", sum(int(row["same_input_cache_reuses"]) for row in invocation_rows))
    print("NEW_EVALUATION_MAX", max(int(row["new_evaluations_total"]) for row in invocation_rows))
    print("BOUNDS", max(int(row["pair_proxy_count"]) for row in post_rows),
          max(int(row["reconstruction_count"]) for row in post_rows),
          max(int(row["validator_count"]) for row in post_rows))
    print("EVALUATION_MS", *(f"{base.percentile(evaluation_values, q):.6f}" for q in
          (0.5, 0.9, 0.95, 0.99)), f"{max(evaluation_values):.6f}")
    print("CALLBACK_MS", *(f"{base.percentile(callback_values, q):.6f}" for q in
          (0.5, 0.9, 0.95, 0.99)), f"{max(callback_values):.6f}")


if __name__ == "__main__":
    main()
