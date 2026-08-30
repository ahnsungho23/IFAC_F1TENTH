#!/usr/bin/env python3
"""Coverage-complete finite-domain P3 reference oracle for already-seen data only.

The runner never opens the frozen split manifest as structured data.  Its SHA is
recorded, while event inputs come only from the three already materialized seen-data
oracle directories.  FINAL_HOLDOUT_UNSEEN is therefore not addressable by this script.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import itertools
import json
import math
import subprocess
import time
from collections import Counter, defaultdict
from pathlib import Path


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
PILOT = REPO / "planning_study/p3_oracle_pilot_v2"
CORPUS = REPO / "planning_study/p3_mapping_research_corpus_v1"
METHOD_DIR = REPO / "planning_study/p3_geometry_conditioned_method_v1"
VALIDATION = REPO / "planning_study/p3_geometry_conditioned_validation_v1"
METHOD_IMPL = METHOD_DIR / "analyze_method.py"
SPEC = HERE / "oracle_v2_spec.json"
CONTRACT = HERE / "evaluation_contract.json"
SPLIT_MANIFEST = CORPUS / "dataset_split_manifest.csv"
HARNESS = Path("/tmp/p3_oracle_v2_build/p3_family_oracle_harness")
WORK = Path("/tmp/p3_reference_oracle_v2")
CACHE = WORK / "event_results"

EXPECTED = {
    "oracle_v2_spec_sha256": "62b63b8ab05d5a73565398141bd05a9db4a102541855fb2e3634d85b18a6bffe",
    "evaluation_contract_sha256": "226b1b44a9bea6adf26715658f36ae8e7b2c322e030f363270728f9e044b1dad",
    "exact_validator_harness_sha256": "8b23f2b6df54c2e93794ee087f3ebd7a14f9921544af4f97eef755535e045e7e",
    "dataset_split_manifest_sha256": "c57cfe8e57dfca4bb318d30047e8f7215f6994e88a39babfbdbb10ce7637b2f4",
    "frozen_method_impl_sha256": "900a262e49436fa75a1567d49b7823ef4e4cb7f08d313389cd2de26c647c3722",
}

CHUNK_SIZE = 25000
EPS = 1.0e-9
FROZEN_METHODS = {
    "H3_FIXED_BOUNDED": 24,
    "H4A_GEOMETRY_TRANSITION": 24,
    "H4B_GEOMETRY_LATERAL_TRANSITION": 12,
}
VERIFICATION_IDS = {
    "VUE036": "VUE036_KNOWN_WITNESS",
    "DVE001": "KNOWN_OLD_ORACLE_POSITIVE",
    "DVE004": "KNOWN_OLD_ORACLE_POSITIVE",
    "DVE012": "KNOWN_OLD_ORACLE_POSITIVE",
    "DVE003": "KNOWN_OLD_ORACLE_NEGATIVE",
    "DVE008": "KNOWN_OLD_ORACLE_NEGATIVE",
    "DVE009": "KNOWN_OLD_ORACLE_NEGATIVE",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def freeze_gate() -> dict:
    observed = {
        "oracle_v2_spec_sha256": sha256(SPEC),
        "evaluation_contract_sha256": sha256(CONTRACT),
        "exact_validator_harness_sha256": sha256(HARNESS),
        "dataset_split_manifest_sha256": sha256(SPLIT_MANIFEST),
        "frozen_method_impl_sha256": sha256(METHOD_IMPL),
    }
    mismatch = {key: {"expected": value, "observed": observed[key]}
                for key, value in EXPECTED.items() if value != observed[key]}
    if mismatch:
        raise RuntimeError("FROZEN_INPUT_MISMATCH: " + json.dumps(mismatch, sort_keys=True))
    return observed


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


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


def truth(value) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def number(row: dict | None, key: str, default: float = math.nan) -> float:
    if row is None:
        return default
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def usable(row: dict) -> bool:
    return (row.get("hard_valid") == "1"
            and row.get("exit_reaches_next_obstacle") == "0"
            and number(row, "braking_deficit_m", math.inf) <= EPS)


def canonical_float(value) -> str:
    return float(value).hex()


def config_key(gate: str, side: str, target, middle, entry, exit_scale) -> tuple:
    return (gate, side, canonical_float(target), canonical_float(middle),
            canonical_float(entry), canonical_float(exit_scale))


def row_config_key(row: dict) -> tuple:
    return config_key(row["gate"], row["side"], row["d_target"], row["d_mid"],
                      row["entry_scale"], row["exit_scale"])


def method_rank(row: dict, tiebreak: int) -> tuple:
    return (
        int(row.get("exit_reaches_next_obstacle", "0")),
        number(row, "braking_deficit_m") > EPS,
        number(row, "braking_deficit_m"),
        number(row, "velocity_loss"),
        -number(row, "minimum_normalized_safety_slack"),
        number(row, "global_path_deviation_m"),
        tiebreak,
    )


def unique_values(values) -> list[float]:
    # Exact-bit anchors can reconstruct different digests even when their decimal
    # values differ by only a few ulps.  The v2 specification therefore deduplicates
    # exact request tuples, never epsilon-near values.
    return sorted({float(item) for item in values if math.isfinite(float(item))})


def base_grid(lower: float, upper: float, step: float) -> list[float]:
    lower, upper = sorted((lower, upper))
    values = {lower, upper}
    integer = math.ceil((lower - 1.0e-12) / step)
    while integer * step <= upper + 1.0e-12:
        values.add(min(upper, max(lower, integer * step)))
        integer += 1
    return unique_values(values)


def add_anchors(values: list[float], anchors, lower: float, upper: float,
                with_local_lattice: bool) -> list[float]:
    output = list(values)
    for anchor_value in anchors:
        anchor = float(anchor_value)
        if lower - 1.0e-12 <= anchor <= upper + 1.0e-12:
            output.append(min(upper, max(lower, anchor)))
            if with_local_lattice:
                for offset in range(-4, 5):
                    value = anchor + 0.01 * offset
                    if lower - 1.0e-12 <= value <= upper + 1.0e-12:
                        output.append(min(upper, max(lower, value)))
    return unique_values(output)


def input_catalog() -> list[dict]:
    groups = [
        ("PILOT_SEEN_DEVELOPMENT_DATA", PILOT / "oracle_summary.csv",
         PILOT / "inputs", PILOT / "lineage_parity.csv"),
        ("DEVELOPMENT", CORPUS / "raw_oracle/oracle_summary.csv",
         CORPUS / "raw_oracle/inputs", CORPUS / "raw_oracle/lineage_parity.csv"),
        ("VALIDATION_SEEN_AFTER_V1", VALIDATION / "validation_oracle_summary.csv",
         VALIDATION / "raw_oracle/inputs", VALIDATION / "raw_oracle/lineage_parity.csv"),
    ]
    output = []
    for role, summary_path, input_dir, lineage_path in groups:
        lineage = {row["event_id"]: row for row in read_csv(lineage_path)}
        for row in read_csv(summary_path):
            event_id = row["event_id"]
            event_path = input_dir / f"{event_id}.event"
            if not event_path.is_file():
                raise RuntimeError(f"missing seen-data event input: {event_path}")
            output.append({
                "dataset_role": role, "event_id": event_id, "row": row,
                "lineage": lineage[event_id], "event_path": event_path,
                "production": json.loads(row["production_candidate_parameters"]),
            })
    counts = Counter(item["dataset_role"] for item in output)
    expected = {"PILOT_SEEN_DEVELOPMENT_DATA": 9, "DEVELOPMENT": 40,
                "VALIDATION_SEEN_AFTER_V1": 37}
    if counts != expected:
        raise RuntimeError(f"seen-data input counts changed: {counts}")
    return output


def expected_selector_rows() -> dict[tuple[str, str], list[dict]]:
    output: dict[tuple[str, str], list[dict]] = defaultdict(list)
    development = read_csv(METHOD_DIR / "selected_candidates_audit.csv")
    validation = read_csv(VALIDATION / "validation_selected_candidates.csv")
    for row in development:
        selector = row["selector"]
        if selector in FROZEN_METHODS and int(row["budget_k"]) == FROZEN_METHODS[selector]:
            output[(row["development_event_id"], selector)].append(row)
    for row in validation:
        selector = row["selector"]
        if selector in FROZEN_METHODS and int(row["budget_k"]) == FROZEN_METHODS[selector]:
            output[(row["validation_event_id"], selector)].append(row)
    return output


def write_requests(path: Path, configs: list[tuple]) -> None:
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write("P3_ORACLE_REQUESTS_V1\n")
        for gate, side, target, middle, entry, exit_scale in configs:
            stream.write(
                "Q\tORACLE_V2\t{}\t{}\t{:.17g}\t{:.17g}\t{:.17g}\t{:.17g}\n".format(
                    int(gate == "RELAXED"), int(side == "LEFT"), target, middle,
                    entry, exit_scale))


def parse_harness_output(text: str) -> tuple[list[dict], list[dict]]:
    headers = {}
    contexts, candidates = [], []
    for line in text.splitlines():
        parts = line.split("\t")
        if not parts:
            continue
        kind = parts[0]
        if kind not in {"CONTEXT", "CANDIDATE"}:
            continue
        if len(parts) > 1 and parts[1] == "event_id":
            headers[kind] = parts[1:]
            continue
        if kind not in headers:
            continue
        row = dict(zip(headers[kind], parts[1:]))
        (contexts if kind == "CONTEXT" else candidates).append(row)
    return contexts, candidates


def run_harness(event_path: Path, request_path: Path) -> tuple[list[dict], list[dict], float]:
    started = time.perf_counter()
    completed = subprocess.run([str(HARNESS), str(event_path), str(request_path)],
                               check=True, text=True, stdout=subprocess.PIPE)
    contexts, candidates = parse_harness_output(completed.stdout)
    return contexts, candidates, time.perf_counter() - started


def query_contexts(event_path: Path) -> list[dict]:
    WORK.mkdir(parents=True, exist_ok=True)
    empty = WORK / "empty.requests"
    write_requests(empty, [])
    contexts, candidates, _ = run_harness(event_path, empty)
    if candidates:
        raise RuntimeError("empty context query unexpectedly returned candidates")
    return [row for row in contexts if row.get("valid") == "1"
            and row.get("side_valid") == "1"]


def special_maps(item: dict, method, selector_expectations: dict) -> tuple:
    event_id = item["event_id"]
    event = method.G.parse_event(item["event_path"])
    pool, _ = method.build_pool(event_id, event, item["production"])
    pool_by_side = defaultdict(list)
    for candidate in pool:
        pool_by_side[candidate["side"]].append(candidate)

    selector_configs = {}
    selector_metadata = {}
    all_factor_keys = set()
    h4a_rank = {}
    for selector, budget in FROZEN_METHODS.items():
        ordered = sorted(pool, key=lambda row: method.selector_score(row, selector))
        selected = ordered[:budget]
        selector_configs[selector] = selected
        selector_metadata[selector] = {"pool_size": len(ordered), "budget": budget}
        for rank, candidate in enumerate(ordered, 1):
            key = config_key("STRICT", candidate["side"], candidate["d_target"],
                             candidate["d_mid"], candidate["entry_scale"],
                             candidate["exit_scale"])
            all_factor_keys.add(key)
            if selector == "H4A_GEOMETRY_TRANSITION":
                h4a_rank[key] = rank

    expected = {}
    for selector in FROZEN_METHODS:
        for row in selector_expectations.get((event_id, selector), []):
            key = config_key("STRICT", row["side"], row["d_target"], row["d_mid"],
                             row["entry_scale"], row["exit_scale"])
            expected[(selector, key)] = row
    return pool, pool_by_side, selector_configs, selector_metadata, all_factor_keys, h4a_rank, expected


def request_domain(item: dict, contexts: list[dict], pool: list[dict],
                   pool_by_side: dict) -> tuple[list[tuple], dict]:
    production = item["production"]
    pairs = sorted({(float(row["entry_scale"]), float(row["exit_scale"]))
                    for row in production})
    configs = []
    domain_rows = []
    gate_order = {"STRICT": 0, "RELAXED": 1}
    side_order = {"RIGHT": 0, "LEFT": 1}
    contexts = sorted(contexts, key=lambda row: (gate_order[row["gate"]],
                                                  side_order[row["side"]]))
    for context in contexts:
        gate, side = context["gate"], context["side"]
        lower, upper = sorted((float(context["minimum_target"]),
                               float(context["maximum_target"])))
        side_production = [row for row in production if row["side"] == side]
        target_production = [float(row["d_target"]) for row in side_production]
        mid_production = [float(row["d_mid"]) for row in side_production]
        target_factors = [float(row["d_target"]) for row in pool_by_side[side]]
        mid_factors = [float(row["d_mid"]) for row in pool_by_side[side]]

        targets = base_grid(lower, upper, 0.01)
        targets = add_anchors(targets, target_production, lower, upper, True)
        targets = add_anchors(targets, target_factors, lower, upper, False)
        middles = base_grid(-1.5, 1.5, 0.01)
        middles = add_anchors(middles, mid_production, -1.5, 1.5, True)
        middles = add_anchors(middles, mid_factors, -1.5, 1.5, False)
        for target, middle, pair in itertools.product(targets, middles, pairs):
            configs.append((gate, side, target, middle, pair[0], pair[1]))
        domain_rows.append({
            "gate": gate, "side": side, "minimum_target": lower,
            "maximum_target": upper, "target_value_count": len(targets),
            "d_mid_value_count": len(middles), "transition_pair_count": len(pairs),
            "request_count": len(targets) * len(middles) * len(pairs),
        })
    # Frozen selectors can intentionally rank factor requests whose target later
    # fails the C++ side-domain construction guard.  Submit the complete pool's
    # exact STRICT tuples as an outcome-independent union so constructed-count
    # and digest/verdict parity include those guarded requests too.
    factor_union = [
        ("STRICT", row["side"], float(row["d_target"]), float(row["d_mid"]),
         float(row["entry_scale"]), float(row["exit_scale"]))
        for row in pool
    ]
    production_union = [
        ("STRICT", row["side"], float(row["d_target"]), float(row["d_mid"]),
         float(row["entry_scale"]), float(row["exit_scale"]))
        for row in production
    ]
    configs.extend(production_union)
    configs.extend(factor_union)
    # Exact float tuple dedup; ordering is stable and follows the declared nested ordering.
    unique = []
    seen = set()
    for config in configs:
        key = config_key(*config)
        if key not in seen:
            seen.add(key)
            unique.append(config)
    gate_order = {"STRICT": 0, "RELAXED": 1}
    side_order = {"RIGHT": 0, "LEFT": 1}
    unique.sort(key=lambda row: (gate_order[row[0]], side_order[row[1]],
                                 row[2], row[3], row[4], row[5]))
    return unique, {"contexts": domain_rows, "pre_dedup_count": len(configs),
                    "exact_factor_pool_union_count": len(factor_union),
                    "exact_production_request_union_count": len(production_union),
                    "unique_request_count": len(unique)}


def compact_candidate(row: dict, global_index: int) -> dict:
    keys = [
        "phase", "gate", "side", "d_target", "d_mid", "entry_scale", "exit_scale",
        "z0", "z1", "z2", "z3", "z4", "point_count", "validator_executed",
        "hard_valid", "first_failure_reason", "center_track_margin_m",
        "footprint_track_margin_m", "obstacle_margin_m",
        "waypoint0_center_track_margin_m", "waypoint0_footprint_track_margin_m",
        "peak_lateral_slope", "lateral_slope_margin", "peak_positive_curvature_radpm",
        "peak_negative_curvature_radpm", "signed_curvature_margin_radpm",
        "peak_curvature_rate_radpm2", "curvature_rate_margin_radpm2",
        "braking_deficit_m", "velocity_loss", "minimum_normalized_safety_slack",
        "global_path_deviation_m", "exit_reaches_next_obstacle", "path_digest",
    ]
    output = {key: row.get(key, "") for key in keys}
    output["global_request_index"] = global_index
    return output


def evaluate_event(item: dict, method, selector_expectations: dict, force: bool = False) -> dict:
    CACHE.mkdir(parents=True, exist_ok=True)
    cache_path = CACHE / f"{item['event_id']}.json"
    if cache_path.is_file() and not force:
        return json.loads(cache_path.read_text(encoding="utf-8"))

    contexts = query_contexts(item["event_path"])
    (pool, pool_by_side, selector_configs, selector_meta, all_factor_keys,
     h4a_rank, expected) = special_maps(item, method, selector_expectations)
    configs, domain = request_domain(item, contexts, pool, pool_by_side)

    selector_selected_keys = {}
    for selector, selected in selector_configs.items():
        selector_selected_keys[selector] = {
            config_key("STRICT", row["side"], row["d_target"], row["d_mid"],
                       row["entry_scale"], row["exit_scale"]): rank
            for rank, row in enumerate(selected, 1)
        }
    special_keys = set(all_factor_keys)
    production_digests = {row.get("path_digest", "") for row in item["production"]
                          if row.get("path_digest")}
    required_digests = set(production_digests)
    required_digests.update(row.get("path_digest", "") for row in expected.values()
                            if row.get("path_digest"))

    counts = Counter()
    first_failures = Counter()
    digest_hashes = set()
    required_digest_observed = defaultdict(list)
    special_results = {}
    best_hard = None
    best_usable = None
    runtime = 0.0
    global_offset = 0
    chunk_file = WORK / f"{item['event_id']}.chunk.requests"

    for chunk_index in range(0, len(configs), CHUNK_SIZE):
        chunk = configs[chunk_index:chunk_index + CHUNK_SIZE]
        write_requests(chunk_file, chunk)
        _, rows, elapsed = run_harness(item["event_path"], chunk_file)
        runtime += elapsed
        if len(rows) != len(chunk):
            raise RuntimeError(
                f"{item['event_id']} coverage failure: {len(rows)} rows for {len(chunk)} requests")
        for local_index, row in enumerate(rows):
            global_index = global_offset + local_index
            counts["emitted_candidate_rows"] += 1
            constructed = int(row.get("point_count", "0")) > 0
            validated = row.get("validator_executed") == "1"
            hard = row.get("hard_valid") == "1"
            if constructed:
                counts["constructed_paths"] += 1
            if validated:
                counts["validator_executions"] += 1
            if hard:
                counts["hard_valid_rows"] += 1
            if usable(row):
                counts["usable_valid_rows"] += 1
            reason = row.get("first_failure_reason", "")
            if not hard:
                first_failures[reason or "CONSTRUCTION_GUARD_OR_UNSPECIFIED"] += 1
            digest = row.get("path_digest", "")
            if digest:
                digest_hashes.add(int(digest, 16))
                if digest in required_digests:
                    required_digest_observed[digest].append({
                        "hard_valid": row.get("hard_valid"),
                        "first_failure_reason": reason,
                        "config_key": list(row_config_key(row)),
                    })
            key = row_config_key(row)
            if key in special_keys:
                special_results[key] = compact_candidate(row, global_index)
            compact = compact_candidate(row, global_index)
            if hard and (best_hard is None or method_rank(compact, global_index) <
                         method_rank(best_hard, int(best_hard["global_request_index"]))):
                best_hard = compact
            if usable(row) and (best_usable is None or method_rank(compact, global_index) <
                                method_rank(best_usable,
                                            int(best_usable["global_request_index"]))):
                best_usable = compact
        global_offset += len(chunk)
        if chunk_index == 0 or global_offset == len(configs) or global_offset % 250000 == 0:
            print("PROGRESS", item["event_id"], global_offset, "/", len(configs),
                  "hard", counts["hard_valid_rows"], flush=True)

    if counts["emitted_candidate_rows"] != len(configs):
        raise RuntimeError(f"{item['event_id']} incomplete finite-domain coverage")

    production_parity = []
    for candidate in item["production"]:
        digest = candidate.get("path_digest", "")
        observations = required_digest_observed.get(digest, [])
        verdict_match = any(
            truth(row["hard_valid"]) == truth(candidate.get("hard_valid"))
            and row["first_failure_reason"] == candidate.get("first_failure_reason", "")
            for row in observations)
        production_parity.append({
            "path_digest": digest, "observed": bool(observations),
            "validator_result_match": verdict_match,
        })

    selector_results = {}
    for selector, budget in FROZEN_METHODS.items():
        rows = []
        for key, rank in selector_selected_keys[selector].items():
            if key not in special_results:
                raise RuntimeError(f"{item['event_id']} missing selected factor request {selector}")
            row = dict(special_results[key])
            row["selection_rank"] = rank
            rows.append(row)
        digests = set()
        effective = []
        for row in sorted(rows, key=lambda value: int(value["selection_rank"])):
            digest = row.get("path_digest", "")
            if digest and digest not in digests:
                digests.add(digest)
                effective.append(row)
        hard = [row for row in effective if row["hard_valid"] == "1"]
        usable_rows = [row for row in effective if usable(row)]
        best_method_hard = min(hard, key=lambda row: method_rank(
            row, int(row["selection_rank"]))) if hard else None
        best_method_usable = min(usable_rows, key=lambda row: method_rank(
            row, int(row["selection_rank"]))) if usable_rows else None
        expected_rows = selector_expectations.get((item["event_id"], selector), [])
        expected_parity = []
        for expected_row in expected_rows:
            key = config_key("STRICT", expected_row["side"], expected_row["d_target"],
                             expected_row["d_mid"], expected_row["entry_scale"],
                             expected_row["exit_scale"])
            actual = special_results.get(key)
            expected_parity.append(bool(actual)
                                   and actual.get("path_digest", "") ==
                                   expected_row.get("path_digest", "")
                                   and truth(actual.get("hard_valid")) ==
                                   truth(expected_row.get("hard_valid")))
        selector_results[selector] = {
            **selector_meta[selector], "selected_count": len(rows),
            "constructed_count": sum(bool(row.get("path_digest")) for row in rows),
            "validator_execution_count": sum(row.get("validator_executed") == "1" for row in rows),
            "unique_path_count": len(effective), "hard_valid_count": len(hard),
            "usable_valid_count": len(usable_rows), "hard_recovered": bool(best_method_hard),
            "usable_recovered": bool(best_method_usable),
            "best_hard": best_method_hard, "best_usable": best_method_usable,
            "frozen_artifact_candidate_parity": all(expected_parity) if expected_parity else None,
            "frozen_artifact_candidate_parity_count": len(expected_parity),
        }

    full_h4a_hard_ranks = []
    full_h4a_usable_ranks = []
    for key, rank in h4a_rank.items():
        row = special_results.get(key)
        if row and row["hard_valid"] == "1":
            full_h4a_hard_ranks.append(rank)
        if row and usable(row):
            full_h4a_usable_ranks.append(rank)

    result = {
        "schema_version": "P3_REFERENCE_ORACLE_V2_EVENT_RESULT_1",
        "dataset_role": item["dataset_role"], "event_id": item["event_id"],
        "event_input_sha256": sha256(item["event_path"]),
        "lineage": item["lineage"], "old_summary": item["row"],
        "domain": domain, "declared_request_count": len(configs),
        "emitted_candidate_rows": counts["emitted_candidate_rows"],
        "constructed_path_count": counts["constructed_paths"],
        "validator_execution_count": counts["validator_executions"],
        "unique_constructed_path_digest_count": len(digest_hashes),
        "hard_valid_count": counts["hard_valid_rows"],
        "usable_valid_count": counts["usable_valid_rows"],
        "hard_classification": ("ORACLE_V2_HARD_VALID_P3_EXISTS" if best_hard else
                                "NO_HARD_VALID_P3_IN_DECLARED_FINITE_DOMAIN"),
        "usable_classification": ("ORACLE_V2_USABLE_VALID_P3_EXISTS" if best_usable else
                                  "NO_USABLE_VALID_P3_IN_DECLARED_FINITE_DOMAIN"),
        "best_hard": best_hard, "best_usable": best_usable,
        "first_failure_distribution": dict(first_failures),
        "production_candidate_count": len(item["production"]),
        "production_digest_and_validator_parity": all(
            row["observed"] and row["validator_result_match"] for row in production_parity),
        "production_parity_rows": production_parity,
        "selector_results": selector_results,
        "complete_factor_pool_size": len(pool),
        "complete_h4a_pool_hard_valid_count": len(full_h4a_hard_ranks),
        "complete_h4a_pool_usable_valid_count": len(full_h4a_usable_ranks),
        "first_h4a_order_hard_valid_rank": min(full_h4a_hard_ranks, default=None),
        "first_h4a_order_usable_valid_rank": min(full_h4a_usable_ranks, default=None),
        "runtime_wall_s": runtime,
        "coverage_complete": counts["emitted_candidate_rows"] == len(configs),
    }
    cache_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n",
                          encoding="utf-8")
    return result


def old_request_paths(item: dict) -> tuple[Path | None, Path | None]:
    event_id = item["event_id"]
    if event_id.startswith("V2E"):
        root = Path("/tmp/p3_oracle_pilot_v2_work")
    elif event_id.startswith("DVE"):
        root = Path("/tmp/p3_mapping_research_corpus_v1_work")
    elif event_id.startswith("VUE"):
        root = Path("/tmp/p3_geometry_conditioned_validation_v1/oracle")
    else:
        return None, None
    return root / f"{event_id}.coarse.requests", root / f"{event_id}.refine.requests"


def request_file_contains(path: Path | None, best: dict | None) -> bool:
    if path is None or best is None or not path.is_file():
        return False
    wanted = config_key(best["gate"], best["side"], best["d_target"], best["d_mid"],
                        best["entry_scale"], best["exit_scale"])
    for line in path.read_text(encoding="utf-8").splitlines()[1:]:
        parts = line.split("\t")
        if len(parts) != 8:
            continue
        key = config_key("RELAXED" if int(parts[2]) else "STRICT",
                         "LEFT" if int(parts[3]) else "RIGHT",
                         parts[4], parts[5], parts[6], parts[7])
        if key == wanted:
            return True
    return False


def flatten_result(result: dict) -> dict:
    old = result["old_summary"]
    best = result.get("best_hard") or {}
    usable_best = result.get("best_usable") or {}
    return {
        "dataset_role": result["dataset_role"], "event_id": result["event_id"],
        "bag": old.get("bag", ""), "elapsed_s": old.get("elapsed_s", ""),
        "callback_sequence": old.get("callback_sequence", ""),
        "evaluation_sequence": old.get("evaluation_sequence", ""),
        "old_oracle_label": old.get("classification", "NOT_RUN"),
        "oracle_v2_hard_label": result["hard_classification"],
        "oracle_v2_usable_label": result["usable_classification"],
        "declared_request_count": result["declared_request_count"],
        "constructed_path_count": result["constructed_path_count"],
        "validator_execution_count": result["validator_execution_count"],
        "unique_constructed_path_digest_count": result["unique_constructed_path_digest_count"],
        "hard_valid_count": result["hard_valid_count"],
        "usable_valid_count": result["usable_valid_count"],
        "best_hard_gate": best.get("gate", ""), "best_hard_side": best.get("side", ""),
        "best_hard_d_target": best.get("d_target", ""),
        "best_hard_d_mid": best.get("d_mid", ""),
        "best_hard_entry_scale": best.get("entry_scale", ""),
        "best_hard_exit_scale": best.get("exit_scale", ""),
        "best_hard_path_digest": best.get("path_digest", ""),
        "best_hard_exit_reaches_next_obstacle":
            best.get("exit_reaches_next_obstacle", ""),
        "best_hard_braking_deficit_m": best.get("braking_deficit_m", ""),
        "best_hard_velocity_loss": best.get("velocity_loss", ""),
        "best_hard_minimum_normalized_safety_slack":
            best.get("minimum_normalized_safety_slack", ""),
        "best_hard_obstacle_margin_m": best.get("obstacle_margin_m", ""),
        "best_usable_path_digest": usable_best.get("path_digest", ""),
        "exact_input_lineage": result["lineage"].get("instrumentation_v2_join", ""),
        "production_digest_and_validator_parity":
            result["production_digest_and_validator_parity"],
        "coverage_complete": result["coverage_complete"],
        "runtime_wall_s": result["runtime_wall_s"],
    }


def changed_cause(item: dict, result: dict) -> tuple[bool, str]:
    old = item["row"].get("classification", "")
    new_positive = result["best_hard"] is not None
    changed = old == "NO_VALID_P3_FOUND_IN_ORACLE_DOMAIN" and new_positive
    if not changed:
        return False, "UNCHANGED"
    if item["event_id"] == "VUE036":
        return True, "ORACLE_REFINEMENT_MISS"
    coarse, refine = old_request_paths(item)
    best = result["best_hard"]
    if request_file_contains(coarse, best) or request_file_contains(refine, best):
        return True, "OLD_REQUEST_PRESENT_REQUIRES_SEPARATE_IMPLEMENTATION_AUDIT"
    # Factor anchors are exact non-outcome-derived requests in v2.
    return True, "OLD_SEARCH_COVERAGE_GAP_BEST_V2_TUPLE_NOT_REQUESTED"


def method_event_rows(results: list[dict]) -> list[dict]:
    rows = []
    for result in results:
        if result["dataset_role"] not in {"DEVELOPMENT", "VALIDATION_SEEN_AFTER_V1"}:
            continue
        feasible = result["best_hard"] is not None
        for method_name in ["PRODUCTION", *FROZEN_METHODS]:
            if method_name == "PRODUCTION":
                production_candidates = json.loads(
                    result["old_summary"]["production_candidate_parameters"])
                production_hard = [row for row in production_candidates
                                   if truth(row.get("hard_valid"))]
                production_usable = [
                    row for row in production_hard
                    if not truth(row.get("rank_tuple", {}).get(
                        "exit_reaches_next_obstacle", False))
                    and float(row.get("rank_tuple", {}).get(
                        "braking_deficit_m", math.inf)) <= EPS
                ]
                hard_recovered = bool(production_hard)
                usable_recovered = bool(production_usable)
                selected_count = int(result["old_summary"].get(
                    "production_constructed_count", "0"))
                validator_count = sum(truth(row.get("validator_executed"))
                                      for row in production_candidates)
                method_result = {
                    "unique_path_count": len({row.get("path_digest")
                                              for row in production_candidates
                                              if row.get("path_digest")}),
                    "hard_valid_count": len(production_hard),
                    "usable_valid_count": len(production_usable),
                    "best_hard": production_hard[0] if production_hard else None,
                    "best_usable": production_usable[0] if production_usable else None,
                }
            else:
                method_result = result["selector_results"][method_name]
                hard_recovered = method_result["hard_recovered"]
                usable_recovered = method_result["usable_recovered"]
                selected_count = method_result["selected_count"]
                validator_count = method_result["validator_execution_count"]
            rows.append({
                "dataset_role": result["dataset_role"], "event_id": result["event_id"],
                "method": method_name,
                "candidate_budget_k": FROZEN_METHODS.get(method_name, "PRODUCTION"),
                "oracle_v2_hard_feasible": feasible,
                "oracle_v2_usable_feasible": result["best_usable"] is not None,
                "hard_valid_recovered": hard_recovered,
                "usable_valid_recovered": usable_recovered,
                "selected_or_constructed_candidate_count": selected_count,
                "validator_execution_count": validator_count,
                "unique_path_count": method_result.get("unique_path_count", ""),
                "hard_valid_candidate_count": method_result.get("hard_valid_count", ""),
                "usable_valid_candidate_count": method_result.get("usable_valid_count", ""),
                "frozen_artifact_candidate_parity":
                    method_result.get("frozen_artifact_candidate_parity", ""),
                "frozen_artifact_candidate_parity_count":
                    method_result.get("frozen_artifact_candidate_parity_count", ""),
                "best_hard_path_digest": (method_result.get("best_hard") or {}).get(
                    "path_digest", ""),
                "best_usable_path_digest": (method_result.get("best_usable") or {}).get(
                    "path_digest", ""),
            })
    return rows


def aggregate_method_comparison(event_rows: list[dict]) -> list[dict]:
    output = []
    for role in ("DEVELOPMENT", "VALIDATION_SEEN_AFTER_V1"):
        scoped = [row for row in event_rows if row["dataset_role"] == role]
        feasible_ids = {row["event_id"] for row in scoped if truth(row["oracle_v2_hard_feasible"])}
        usable_feasible_ids = {row["event_id"] for row in scoped
                               if truth(row["oracle_v2_usable_feasible"])}
        success_sets = {
            method: {row["event_id"] for row in scoped
                     if row["method"] == method and truth(row["oracle_v2_hard_feasible"])
                     and truth(row["hard_valid_recovered"])}
            for method in ["PRODUCTION", *FROZEN_METHODS]
        }
        h3, h4a = success_sets["H3_FIXED_BOUNDED"], success_sets["H4A_GEOMETRY_TRANSITION"]
        usable_success_sets = {
            method: {row["event_id"] for row in scoped
                     if row["method"] == method
                     and row["event_id"] in usable_feasible_ids
                     and truth(row["usable_valid_recovered"])}
            for method in ["PRODUCTION", *FROZEN_METHODS]
        }
        h3_usable = usable_success_sets["H3_FIXED_BOUNDED"]
        h4a_usable = usable_success_sets["H4A_GEOMETRY_TRANSITION"]
        for method in ["PRODUCTION", *FROZEN_METHODS]:
            rows = [row for row in scoped if row["method"] == method
                    and row["event_id"] in feasible_ids]
            hard = success_sets[method]
            usable_ids = {row["event_id"] for row in rows if truth(row["usable_valid_recovered"])}
            output.append({
                "dataset_role": role, "method": method,
                "candidate_budget_k": rows[0]["candidate_budget_k"] if rows else "",
                "oracle_v2_hard_feasible_production_failure_count": len(feasible_ids),
                "oracle_v2_usable_feasible_production_failure_count":
                    len(usable_feasible_ids),
                "hard_valid_recovery_count": len(hard),
                "hard_valid_recovery_rate": len(hard) / len(feasible_ids) if feasible_ids else math.nan,
                "usable_valid_recovery_count": len(usable_ids),
                "usable_valid_recovery_rate":
                    len(usable_ids) / len(usable_feasible_ids)
                    if usable_feasible_ids else math.nan,
                "candidate_count_total": sum(int(row["selected_or_constructed_candidate_count"])
                                             for row in rows),
                "validator_execution_count_total": sum(int(row["validator_execution_count"])
                                                       for row in rows),
                "h3_h4a_hard_success_intersection_count": len(h3 & h4a),
                "h3_only_hard_success_count": len(h3 - h4a),
                "h4a_only_hard_success_count": len(h4a - h3),
                "h3_only_event_ids": "|".join(sorted(h3 - h4a)),
                "h4a_only_event_ids": "|".join(sorted(h4a - h3)),
                "h3_h4a_usable_success_intersection_count":
                    len(h3_usable & h4a_usable),
                "h3_only_usable_success_count": len(h3_usable - h4a_usable),
                "h4a_only_usable_success_count": len(h4a_usable - h3_usable),
                "h3_only_usable_event_ids": "|".join(sorted(h3_usable - h4a_usable)),
                "h4a_only_usable_event_ids": "|".join(sorted(h4a_usable - h3_usable)),
            })
    return output


def remaining_taxonomy(results: list[dict]) -> list[dict]:
    rows = []
    for result in results:
        if result["dataset_role"] not in {"DEVELOPMENT", "VALIDATION_SEEN_AFTER_V1"}:
            continue
        h4a = result["selector_results"]["H4A_GEOMETRY_TRANSITION"]
        if result["best_hard"] is None or h4a["hard_recovered"]:
            continue
        best = result["best_hard"]
        production = json.loads(result["old_summary"]["production_candidate_parameters"])
        same_target = any(row["side"] == best["side"]
                          and abs(float(row["d_target"]) - float(best["d_target"])) <= EPS
                          for row in production)
        same_lateral = any(row["side"] == best["side"]
                           and abs(float(row["d_target"]) - float(best["d_target"])) <= EPS
                           and abs(float(row["d_mid"]) - float(best["d_mid"])) <= EPS
                           for row in production)
        same_transition = any(abs(float(row["entry_scale"]) -
                                  float(best["entry_scale"])) <= EPS
                              and abs(float(row["exit_scale"]) -
                                      float(best["exit_scale"])) <= EPS
                              for row in production)
        first_rank = result["first_h4a_order_hard_valid_rank"]
        if first_rank is not None and first_rank > 24:
            mechanism = "CANDIDATE_BUDGET_TRUNCATION"
        elif result["complete_h4a_pool_hard_valid_count"] > 0:
            mechanism = "TRANSITION_OR_TOPK_SELECTION"
        elif same_lateral and not same_transition:
            mechanism = "TRANSITION_SELECTION"
        elif same_target and same_transition and not same_lateral:
            mechanism = "PROBE_ROOT_OR_D_MID_SELECTION"
        elif same_target and not same_lateral:
            mechanism = "LATERAL_FACTOR_SELECTION"
        elif not same_target:
            mechanism = "MULTI_PARAMETER_OR_FACTOR_SPACE_COVERAGE"
        else:
            mechanism = "OTHER"
        rows.append({
            "dataset_role": result["dataset_role"], "event_id": result["event_id"],
            "oracle_v2_best_side": best["side"],
            "oracle_v2_best_d_target": best["d_target"],
            "oracle_v2_best_d_mid": best["d_mid"],
            "oracle_v2_best_entry_scale": best["entry_scale"],
            "oracle_v2_best_exit_scale": best["exit_scale"],
            "complete_factor_pool_size": result["complete_factor_pool_size"],
            "complete_h4a_pool_hard_valid_count":
                result["complete_h4a_pool_hard_valid_count"],
            "complete_h4a_pool_usable_valid_count":
                result["complete_h4a_pool_usable_valid_count"],
            "first_h4a_order_hard_valid_rank": first_rank or "",
            "first_h4a_order_usable_valid_rank":
                result["first_h4a_order_usable_valid_rank"] or "",
            "oracle_v2_usable_feasible": result["best_usable"] is not None,
            "usable_recoverable_miss":
                result["best_usable"] is not None and not h4a["usable_recovered"],
            "same_production_target": same_target,
            "same_production_lateral_pair": same_lateral,
            "same_production_transition_pair": same_transition,
            "primary_mechanism": mechanism,
            "classification_contract": "DESCRIPTIVE_SEEN_DATA_NO_METHOD_REDESIGN",
        })
    return rows


def materialize_outputs(catalog: list[dict], results: list[dict], observed_hashes: dict) -> None:
    item_by_id = {item["event_id"]: item for item in catalog}
    flat = [flatten_result(result) for result in results]
    old_vs = []
    for result in results:
        item = item_by_id[result["event_id"]]
        changed, cause = changed_cause(item, result)
        old_vs.append({
            **flatten_result(result), "old_negative_became_oracle_v2_positive": changed,
            "old_miss_cause_if_identifiable": cause,
            "historical_artifact_overwritten": False,
        })
    write_csv(HERE / "old_vs_v2_oracle_labels.csv", old_vs)
    write_csv(HERE / "pilot_seen_recomputed.csv",
              [row for row in flat
               if row["dataset_role"] == "PILOT_SEEN_DEVELOPMENT_DATA"])
    write_csv(HERE / "development_recomputed.csv",
              [row for row in flat if row["dataset_role"] == "DEVELOPMENT"])
    write_csv(HERE / "validation_seen_recomputed.csv",
              [row for row in flat if row["dataset_role"] == "VALIDATION_SEEN_AFTER_V1"])

    method_rows = method_event_rows(results)
    write_csv(HERE / "frozen_method_event_results.csv", method_rows)
    write_csv(HERE / "frozen_method_comparison.csv",
              aggregate_method_comparison(method_rows))
    write_csv(HERE / "remaining_failure_taxonomy.csv", remaining_taxonomy(results))

    verification = []
    for event_id, purpose in VERIFICATION_IDS.items():
        result = next(row for row in results if row["event_id"] == event_id)
        old_positive = result["old_summary"]["classification"] == "ORACLE_VALID_P3_EXISTS"
        expected_positive = event_id == "VUE036" or old_positive
        selector_parity = [value["frozen_artifact_candidate_parity"]
                           for value in result["selector_results"].values()
                           if value["frozen_artifact_candidate_parity"] is not None]
        verification.append({
            "event_id": event_id, "verification_purpose": purpose,
            "exact_input_lineage": result["lineage"].get("instrumentation_v2_join", ""),
            "production_digest_and_validator_parity":
                result["production_digest_and_validator_parity"],
            "frozen_selector_candidate_digest_and_verdict_parity":
                all(selector_parity) if selector_parity else "NOT_APPLICABLE",
            "oracle_v2_hard_label": result["hard_classification"],
            "expected_hard_positive": expected_positive,
            "expected_positive_met": bool(result["best_hard"]) if expected_positive else True,
            "finite_domain_coverage_complete": result["coverage_complete"],
            "verification_pass": result["production_digest_and_validator_parity"]
                and result["coverage_complete"]
                and (bool(result["best_hard"]) if expected_positive else True)
                and (all(selector_parity) if selector_parity else True),
        })
    write_csv(HERE / "oracle_v2_verification.csv", verification)
    if not all(truth(row["verification_pass"]) for row in verification):
        raise RuntimeError("oracle-v2 verification gate failed")

    vue = next(result for result in results if result["event_id"] == "VUE036")
    vue_rows = []
    for kind, row in (("ORACLE_V2_BEST_HARD", vue["best_hard"]),
                      ("ORACLE_V2_BEST_USABLE", vue["best_usable"])):
        row = row or {}
        vue_rows.append({
            "event_id": "VUE036", "record_kind": kind,
            "d_target": row.get("d_target", ""), "d_mid": row.get("d_mid", ""),
            "entry_scale": row.get("entry_scale", ""),
            "exit_scale": row.get("exit_scale", ""),
            "path_digest": row.get("path_digest", ""),
            "hard_valid": row.get("hard_valid", ""),
            "exit_reaches_next_obstacle": row.get("exit_reaches_next_obstacle", ""),
            "braking_deficit_m": row.get("braking_deficit_m", ""),
            "minimum_normalized_safety_slack":
                row.get("minimum_normalized_safety_slack", ""),
            "obstacle_margin_m": row.get("obstacle_margin_m", ""),
            "declared_request_count": vue["declared_request_count"],
            "hard_valid_count": vue["hard_valid_count"],
            "usable_valid_count": vue["usable_valid_count"],
            "old_miss_class": "ORACLE_REFINEMENT_MISS",
        })
    for selector, method_result in vue["selector_results"].items():
        row = method_result.get("best_hard") or {}
        vue_rows.append({
            "event_id": "VUE036", "record_kind": f"KNOWN_FROZEN_WITNESS_{selector}",
            "d_target": row.get("d_target", ""), "d_mid": row.get("d_mid", ""),
            "entry_scale": row.get("entry_scale", ""),
            "exit_scale": row.get("exit_scale", ""),
            "path_digest": row.get("path_digest", ""),
            "hard_valid": row.get("hard_valid", ""),
            "exit_reaches_next_obstacle": row.get("exit_reaches_next_obstacle", ""),
            "braking_deficit_m": row.get("braking_deficit_m", ""),
            "minimum_normalized_safety_slack":
                row.get("minimum_normalized_safety_slack", ""),
            "obstacle_margin_m": row.get("obstacle_margin_m", ""),
            "selection_rank": row.get("selection_rank", ""),
            "declared_request_count": vue["declared_request_count"],
            "hard_valid_count": vue["hard_valid_count"],
            "usable_valid_count": vue["usable_valid_count"],
            "old_miss_class": "ORACLE_REFINEMENT_MISS",
        })
    write_csv(HERE / "vue036_recovery.csv", vue_rows)

    computation = []
    for result in results:
        computation.append({
            "scope": "EVENT", "dataset_role": result["dataset_role"],
            "event_id": result["event_id"],
            "declared_requests": result["declared_request_count"],
            "emitted_candidate_rows": result["emitted_candidate_rows"],
            "constructed_paths": result["constructed_path_count"],
            "validator_executions": result["validator_execution_count"],
            "unique_constructed_paths": result["unique_constructed_path_digest_count"],
            "hard_valid_rows": result["hard_valid_count"],
            "usable_valid_rows": result["usable_valid_count"],
            "runtime_wall_s": result["runtime_wall_s"],
            "coverage_complete": result["coverage_complete"],
        })
    for role in ["PILOT_SEEN_DEVELOPMENT_DATA", "DEVELOPMENT",
                 "VALIDATION_SEEN_AFTER_V1", "ALL_SEEN"]:
        scoped = results if role == "ALL_SEEN" else [
            row for row in results if row["dataset_role"] == role]
        computation.append({
            "scope": "AGGREGATE", "dataset_role": role, "event_id": "",
            "event_count": len(scoped),
            "declared_requests": sum(row["declared_request_count"] for row in scoped),
            "emitted_candidate_rows": sum(row["emitted_candidate_rows"] for row in scoped),
            "constructed_paths": sum(row["constructed_path_count"] for row in scoped),
            "validator_executions": sum(row["validator_execution_count"] for row in scoped),
            "hard_valid_rows": sum(row["hard_valid_count"] for row in scoped),
            "usable_valid_rows": sum(row["usable_valid_count"] for row in scoped),
            "hard_feasible_event_count": sum(row["best_hard"] is not None for row in scoped),
            "usable_feasible_event_count": sum(row["best_usable"] is not None for row in scoped),
            "runtime_wall_s": sum(row["runtime_wall_s"] for row in scoped),
            "coverage_complete": all(row["coverage_complete"] for row in scoped),
            **observed_hashes,
        })
    write_csv(HERE / "computation_summary.csv", computation)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--verification", action="store_true")
    parser.add_argument("--full", action="store_true")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--events", default="")
    args = parser.parse_args()
    if not args.verification and not args.full and not args.events:
        parser.error("select --verification, --full, or --events")

    hashes = freeze_gate()
    method = load_module("frozen_factor_method_oracle_v2", METHOD_IMPL)
    catalog = input_catalog()
    expectations = expected_selector_rows()
    wanted = set(filter(None, args.events.split(",")))
    if args.verification:
        wanted.update(VERIFICATION_IDS)
    if args.full:
        wanted.update(item["event_id"] for item in catalog)
    selected = [item for item in catalog if item["event_id"] in wanted]
    missing = wanted - {item["event_id"] for item in selected}
    if missing:
        raise RuntimeError(f"unknown or forbidden event ids: {sorted(missing)}")

    results = []
    for index, item in enumerate(selected, 1):
        started = time.perf_counter()
        result = evaluate_event(item, method, expectations, force=args.force)
        results.append(result)
        print("EVENT_DONE", item["event_id"], index, "/", len(selected),
              "requests", result["declared_request_count"],
              "hard", result["hard_valid_count"],
              "wall", time.perf_counter() - started, flush=True)

    if args.verification and not args.full:
        if not any(row["event_id"] == "VUE036" and row["best_hard"] for row in results):
            raise RuntimeError("STOP: VUE036 not recovered by Reference Oracle v2")
        # Minimal verification artifact can be written before full recomputation.
        verification = []
        for result in results:
            selector_parity = [value["frozen_artifact_candidate_parity"]
                               for value in result["selector_results"].values()
                               if value["frozen_artifact_candidate_parity"] is not None]
            old_positive = result["old_summary"]["classification"] == "ORACLE_VALID_P3_EXISTS"
            expected_positive = result["event_id"] == "VUE036" or old_positive
            passed = (result["coverage_complete"]
                      and result["production_digest_and_validator_parity"]
                      and (all(selector_parity) if selector_parity else True)
                      and (bool(result["best_hard"]) if expected_positive else True))
            verification.append({
                "event_id": result["event_id"],
                "verification_purpose": VERIFICATION_IDS[result["event_id"]],
                "oracle_v2_hard_label": result["hard_classification"],
                "production_digest_and_validator_parity":
                    result["production_digest_and_validator_parity"],
                "frozen_selector_candidate_digest_and_verdict_parity":
                    all(selector_parity) if selector_parity else "NOT_APPLICABLE",
                "coverage_complete": result["coverage_complete"],
                "verification_pass": passed,
            })
        write_csv(HERE / "oracle_v2_verification.csv", verification)
        if not all(truth(row["verification_pass"]) for row in verification):
            raise RuntimeError("STOP: verification gate failed")
        return

    if args.full:
        all_results = [evaluate_event(item, method, expectations) for item in catalog]
        materialize_outputs(catalog, all_results, hashes)


if __name__ == "__main__":
    main()
