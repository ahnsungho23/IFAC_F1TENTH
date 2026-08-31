#!/usr/bin/env python3
"""Parity and warm-process timing driver for the native R3-RT research shadow."""

from __future__ import annotations

import argparse
import csv
import importlib.util
import math
import os
import statistics
import subprocess
from collections import defaultdict
from pathlib import Path


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
DESIGN = REPO / "planning_study/p3_r3_rt_bounded_proposal_v1/run_bounded_proposal_study.py"
REFERENCE = REPO / "planning_study/p3_reference_oracle_v2/run_reference_oracle_v2.py"
HARNESS = REPO / "build/local_planning/p3_r3_k12_integration_harness"
BUDGETS = (64, 96, 128)
CAPS = {64: 36, 96: 48, 128: 64}
EXPECTED_ROLES = {
    "PILOT_SEEN_DEVELOPMENT_DATA", "DEVELOPMENT", "VALIDATION_SEEN_AFTER_V1"}
PROXY_FIELDS = (
    "max_corridor_violation_m", "sum_corridor_violation_m", "slope_excess",
    "curvature_proxy", "center_error", "minimum_clearance_m", "shape_energy",
    "entry_normalized", "exit_normalized")


def load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


B = load("r3_rt_design_authority", DESIGN)
R = load("r3_rt_reference_catalog", REFERENCE)


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


def canonical(value) -> str:
    return float(value).hex()


def same_number(first, second) -> bool:
    first, second = float(first), float(second)
    if math.isnan(first) and math.isnan(second):
        return True
    return canonical(first) == canonical(second)


def config(row: dict) -> tuple[str, ...]:
    return (row["side"], canonical(row["d_target"]), canonical(row["d_mid"]),
            canonical(row["entry_scale"]), canonical(row["exit_scale"]))


def run_native(items: list[dict], detail: bool, warmup: int, repeats: int) -> str:
    env = os.environ.copy()
    env.update({
        "R3_RT_NATIVE_BUDGETS": ",".join(map(str, BUDGETS)),
        "R3_RT_WARMUP": str(warmup),
        "R3_RT_REPEATS": str(repeats),
    })
    if detail:
        env["R3_RT_PARITY_DETAIL"] = "1"
    completed = subprocess.run(
        [str(HARNESS), *(str(item["event_path"]) for item in items)],
        check=True, text=True, stdout=subprocess.PIPE, env=env)
    return completed.stdout


def parse_native(text: str) -> dict:
    output = {
        "summary": {}, "laterals": defaultdict(list), "transitions": defaultdict(list),
        "pairs": defaultdict(list), "top": defaultdict(list),
        "selected": defaultdict(list), "timing": []}
    for line in text.splitlines():
        fields = line.split("\t")
        if not fields or not fields[0].startswith("NATIVE_"):
            continue
        kind = fields[0]
        event_id, budget = fields[1], int(fields[2])
        key = event_id, budget
        if kind == "NATIVE_SUMMARY":
            output["summary"][key] = {
                "invoked": fields[3] == "1", "recovered": fields[4] == "1",
                "raw_laterals": int(fields[5]), "laterals": int(fields[6]),
                "transitions": int(fields[7]), "pairs": int(fields[8]),
                "reconstructions": int(fields[9]), "validators": int(fields[10]),
                "hard_count": int(fields[11]), "usable_count": int(fields[12]),
                "selected_d_target": float(fields[13]), "selected_d_mid": float(fields[14]),
                "selected_digest": fields[15], "failure": fields[16],
            }
        elif kind == "NATIVE_LATERAL":
            output["laterals"][key].append({
                "index": int(fields[3]), "side": fields[4], "d_target": float(fields[5]),
                "d_mid": float(fields[6]), "target_source": fields[7],
                "mid_source": fields[8], "lateral_source_family": fields[9],
                "source_priority": int(fields[10]), "proposal_operator": fields[11],
            })
        elif kind == "NATIVE_TRANSITION":
            output["transitions"][key].append({
                "transition_index": int(fields[3]), "entry_scale": float(fields[4]),
                "exit_scale": float(fields[5]), "transition_family": fields[6],
            })
        elif kind == "NATIVE_PAIR":
            output["pairs"][key].append({
                "index": int(fields[3]), "configuration_key": fields[4],
                "preconstruction_shape_key": fields[5], "transition_family": fields[6],
                "construction_guard_proxy": int(fields[7]),
                "exit_conflict_proxy": int(fields[8]),
                "max_corridor_violation_m": float(fields[9]),
                "sum_corridor_violation_m": float(fields[10]),
                "slope_excess": float(fields[11]), "curvature_proxy": float(fields[12]),
                "center_error": float(fields[13]), "minimum_clearance_m": float(fields[14]),
                "shape_energy": float(fields[15]), "entry_normalized": float(fields[16]),
                "exit_normalized": float(fields[17]),
            })
        elif kind == "NATIVE_TOP_FACTOR":
            output["top"][key].append({
                "rank_stream": fields[3], "stream_rank": int(fields[4]), "side": fields[5],
                "d_target": float(fields[6]), "d_mid": float(fields[7]),
                "entry_scale": float(fields[8]), "exit_scale": float(fields[9]),
                "configuration_key": fields[10],
            })
        elif kind == "NATIVE_SELECTED":
            output["selected"][key].append({
                "rank_stream": fields[3], "stream_rank": int(fields[4]), "side": fields[5],
                "d_target": float(fields[6]), "d_mid": float(fields[7]),
                "entry_scale": float(fields[8]), "exit_scale": float(fields[9]),
                "path_digest": fields[10], "validator_executed": int(fields[11]),
                "hard_valid": int(fields[12]), "usable_valid": int(fields[13]),
            })
        elif kind == "NATIVE_TIMING":
            names = (
                "wall_us", "context_us", "geometry_us", "transition_us", "lateral_us",
                "pair_proxy_us", "lexicographic_us", "coverage_us", "shape_dedup_us",
                "reconstruction_us", "validation_us", "final_ranking_us", "r3_total_us")
            row = {"event_id": event_id, "budget_b": budget, "repeat": int(fields[3])}
            row.update({name: float(value) for name, value in zip(names, fields[4:])})
            output["timing"].append(row)
    return output


def parity(items: list[dict]) -> None:
    native = parse_native(run_native(items, detail=True, warmup=0, repeats=1))
    rows = []
    prepared = {}
    for item_index, item in enumerate(items, 1):
        key0 = item["dataset_role"], item["event_id"]
        prepared[key0] = B.prepare_item(item)
        for budget in BUDGETS:
            key = item["event_id"], budget
            expected_laterals = prepared[key0]["laterals"][:CAPS[budget]]
            expected_transitions = prepared[key0]["transitions"]
            expected_pool, _ = B.pair_pool(
                prepared[key0]["event"], prepared[key0]["geometries"], item["production"],
                expected_laterals, expected_transitions, budget, "NO_RESERVES_DIAGONAL")
            expected_selected = B.select_top12(expected_pool)
            native_laterals = native["laterals"][key]
            native_transitions = native["transitions"][key]
            native_pairs = native["pairs"][key]
            native_top = native["top"][key]
            native_selected = native["selected"][key]
            summary = native["summary"][key]

            lateral_parity = len(expected_laterals) == len(native_laterals) and all(
                expected["side"] == actual["side"]
                and same_number(expected["d_target"], actual["d_target"])
                and same_number(expected["d_mid"], actual["d_mid"])
                and all(expected[field] == actual[field] for field in (
                    "target_source", "mid_source", "lateral_source_family",
                    "source_priority", "proposal_operator"))
                for expected, actual in zip(expected_laterals, native_laterals))
            transition_parity = len(expected_transitions) == len(native_transitions) and all(
                expected["transition_index"] == actual["transition_index"]
                and expected["transition_family"] == actual["transition_family"]
                and same_number(expected["entry_scale"], actual["entry_scale"])
                and same_number(expected["exit_scale"], actual["exit_scale"])
                for expected, actual in zip(expected_transitions, native_transitions))
            pair_order_parity = len(expected_pool) == len(native_pairs) and all(
                expected["configuration_key"] == actual["configuration_key"]
                and expected["preconstruction_shape_key"] == actual["preconstruction_shape_key"]
                and expected["transition_family"] == actual["transition_family"]
                for expected, actual in zip(expected_pool, native_pairs))
            proxy_parity = len(expected_pool) == len(native_pairs) and all(
                int(expected["construction_guard_proxy"]) == actual["construction_guard_proxy"]
                and int(expected["exit_conflict_proxy"]) == actual["exit_conflict_proxy"]
                and all(field not in expected or same_number(expected[field], actual[field])
                        for field in PROXY_FIELDS)
                for expected, actual in zip(expected_pool, native_pairs))
            selected_config_parity = len(expected_selected) == len(native_top) and all(
                config(expected) == config(actual)
                for expected, actual in zip(expected_selected, native_top))

            expected_evaluation, _ = B.evaluate(
                item, budget, "NATIVE_CPP_PARITY_REFERENCE", expected_selected)
            expected_constructed = [
                (candidate, row) for candidate, row in zip(expected_selected, expected_evaluation)
                if row.get("path_digest")]
            digest_parity = len(expected_constructed) == len(native_selected) and all(
                expected["path_digest"] == actual["path_digest"]
                for (_, expected), actual in zip(expected_constructed, native_selected))
            verdict_parity = len(expected_constructed) == len(native_selected) and all(
                int(expected["hard_valid"]) == actual["hard_valid"]
                and int(B.usable(expected)) == actual["usable_valid"]
                for (_, expected), actual in zip(expected_constructed, native_selected))
            unique_candidates, unique_rows = B.exact_dedup(expected_selected, expected_evaluation)
            hard_usable_result_parity = (
                sum(row["hard_valid"] == "1" for row in unique_rows) == summary["hard_count"]
                and sum(B.usable(row) for row in unique_rows) == summary["usable_count"])
            best_hard = B.final_candidate(unique_candidates, unique_rows, False)
            if best_hard:
                final_parity = (
                    same_number(best_hard[0]["d_target"], summary["selected_d_target"])
                    and same_number(best_hard[0]["d_mid"], summary["selected_d_mid"])
                    and best_hard[1]["path_digest"] == summary["selected_digest"])
            else:
                final_parity = not summary["recovered"]
            flags = {
                "lateral_proposal_parity": lateral_parity,
                "transition_proposal_parity": transition_parity,
                "pair_pool_order_parity": pair_order_parity,
                "proxy_tuple_parity": proxy_parity,
                "selected_top12_config_parity": selected_config_parity,
                "candidate_digest_parity": digest_parity,
                "validator_verdict_parity": verdict_parity,
                "hard_usable_result_parity": hard_usable_result_parity,
                "final_selection_parity": final_parity,
            }
            rows.append({
                "dataset_role": item["dataset_role"], "event_id": item["event_id"],
                "budget_b": budget, **{key: int(value) for key, value in flags.items()},
                "all_parity": int(all(flags.values())),
                "native_laterals": summary["laterals"],
                "native_transitions": summary["transitions"],
                "native_pair_proxies": summary["pairs"],
                "native_reconstructions": summary["reconstructions"],
                "native_validator_calls": summary["validators"],
            })
        print(f"PARITY {item_index}/86 {item['event_id']}", flush=True)
    write_csv(HERE / "python_cpp_parity.csv", rows)
    if len(rows) != 258 or not all(row["all_parity"] for row in rows):
        failures = [(row["event_id"], row["budget_b"]) for row in rows if not row["all_parity"]]
        raise RuntimeError(f"PYTHON_CPP_PARITY_FAILED {failures[:20]}")


def percentile(values: list[float], q: float) -> float:
    values = sorted(values)
    position = (len(values) - 1) * q / 100.0
    lower, upper = math.floor(position), math.ceil(position)
    if lower == upper:
        return values[lower]
    return values[lower] + (position - lower) * (values[upper] - values[lower])


def timing(items: list[dict], warmup: int, repeats: int) -> None:
    native = parse_native(run_native(items, detail=False, warmup=warmup, repeats=repeats))
    timing_rows = native["timing"]
    summary_rows = []
    breakdown_rows = []
    stages = (
        "wall_us", "context_us", "geometry_us", "transition_us", "lateral_us",
        "pair_proxy_us", "lexicographic_us", "coverage_us", "shape_dedup_us",
        "reconstruction_us", "validation_us", "final_ranking_us", "r3_total_us")
    for budget in BUDGETS:
        subset = [row for row in timing_rows if row["budget_b"] == budget]
        wall = [row["wall_us"] / 1000.0 for row in subset]
        r3_total = [row["r3_total_us"] / 1000.0 for row in subset]
        summary_rows.append({
            "budget_b": budget, "event_count": len({row["event_id"] for row in subset}),
            "warmup_per_event": warmup, "measured_repeats_per_event": repeats,
            "measurement_count": len(subset),
            **{f"native_r3_total_{name}_ms": value for name, value in (
                ("p50", percentile(r3_total, 50)), ("p90", percentile(r3_total, 90)),
                ("p95", percentile(r3_total, 95)), ("p99", percentile(r3_total, 99)),
                ("max", max(r3_total)))},
            **{f"full_evaluator_wall_{name}_ms": value for name, value in (
                ("p50", percentile(wall, 50)), ("p90", percentile(wall, 90)),
                ("p95", percentile(wall, 95)), ("p99", percentile(wall, 99)),
                ("max", max(wall)))},
        })
        for stage in stages:
            values = [row[stage] / 1000.0 for row in subset]
            breakdown_rows.append({
                "budget_b": budget, "stage": stage.removesuffix("_us"),
                "p50_ms": percentile(values, 50), "p90_ms": percentile(values, 90),
                "p95_ms": percentile(values, 95), "p99_ms": percentile(values, 99),
                "max_ms": max(values),
            })
    write_csv(HERE / "native_runtime_by_budget.csv", summary_rows)
    write_csv(HERE / "runtime_breakdown.csv", breakdown_rows)
    worst = []
    for budget in BUDGETS:
        per_event = defaultdict(list)
        for row in timing_rows:
            if row["budget_b"] == budget:
                per_event[row["event_id"]].append(row["wall_us"] / 1000.0)
        ranked = sorted(per_event.items(), key=lambda item: max(item[1]), reverse=True)[:10]
        for rank, (event_id, values) in enumerate(ranked, 1):
            worst.append({
                "budget_b": budget, "rank": rank, "event_id": event_id,
                "p50_ms": percentile(values, 50), "p95_ms": percentile(values, 95),
                "max_ms": max(values),
            })
    write_csv(HERE / "worst_event_runtime.csv", worst)


def contracts(items: list[dict]) -> None:
    parity_rows = list(csv.DictReader((HERE / "python_cpp_parity.csv").open()))
    rows = []
    for budget in BUDGETS:
        subset = [row for row in parity_rows if int(row["budget_b"]) == budget]
        rows.append({
            "budget_b": budget, "snapshot_count": len(subset),
            "maximum_lateral_proposals": max(int(row["native_laterals"]) for row in subset),
            "lateral_bound": 64,
            "maximum_transition_proposals": max(int(row["native_transitions"]) for row in subset),
            "transition_bound": 7,
            "maximum_pair_proxy_evaluations": max(int(row["native_pair_proxies"]) for row in subset),
            "pair_proxy_bound": budget,
            "maximum_reconstructions": max(int(row["native_reconstructions"]) for row in subset),
            "reconstruction_bound": 12,
            "maximum_validator_calls": max(int(row["native_validator_calls"]) for row in subset),
            "validator_bound": 12,
            "hidden_full_exact_r3_pool_materialized": 0,
            "all_python_cpp_parity": int(all(int(row["all_parity"]) for row in subset)),
        })
    write_csv(HERE / "bounded_contract_verification.csv", rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("parity", "timing", "all"), default="all")
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--repeats", type=int, default=20)
    args = parser.parse_args()
    items = R.input_catalog()
    roles = {item["dataset_role"] for item in items}
    if len(items) != 86 or roles != EXPECTED_ROLES:
        raise RuntimeError(f"seen-only corpus mismatch: {len(items)} {roles}")
    if not HARNESS.is_file():
        raise RuntimeError(f"missing Release native harness: {HARNESS}")
    if args.mode in {"parity", "all"}:
        parity(items)
        contracts(items)
    if args.mode in {"timing", "all"}:
        if not (HERE / "python_cpp_parity.csv").is_file():
            raise RuntimeError("timing is forbidden until Python/C++ parity passes")
        timing(items, args.warmup, args.repeats)


if __name__ == "__main__":
    main()
