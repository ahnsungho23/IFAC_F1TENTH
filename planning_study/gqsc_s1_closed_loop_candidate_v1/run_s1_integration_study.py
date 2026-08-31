#!/usr/bin/env python3
"""Reproduce the seen-only S1 parity, lifecycle, invocation, bounds, and runtime gates."""

from __future__ import annotations

from collections import Counter
import csv
import importlib.util
from pathlib import Path


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
BASE_SCRIPT = REPO / 'planning_study/gqsc_v3_main_integration_v1/run_integration_study.py'
SPEC = importlib.util.spec_from_file_location('gqsc_seen_harness', BASE_SCRIPT)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f'cannot load seen-only harness helper: {BASE_SCRIPT}')
HELPER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HELPER)
METHOD_SHA = '670f39a23479bcdcc1db0829a895257443fec8ee2f78f186bc1beb224090b776'


def write_csv(name: str, rows: list[dict]) -> None:
    if not rows:
        raise RuntimeError(f'no rows for {name}')
    with (HERE / name).open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    paths, roles = HELPER.events()

    parity_rows: list[dict] = []
    parity_names = (
        'record', 'event_id', 'lateral_order_exact', 'transition_order_exact',
        'pair_order_exact', 'top12_exact', 'path_digests_verdicts_exact',
        'selected_path_exact', 'plan_selected_path_exact', 'no_hidden_legacy_seed',
        'snapshot_lineage_exact', 'method_sha_exact', 'bounded_contract_pass',
        'pair_proxy_count', 'reconstruction_count', 'validator_count', 'legacy_seed_count',
        'hard_valid_count', 'usable_valid_count', 'selected_path_digest',
        'failure_classification')
    for line in HELPER.run('GQSC_S1_MAIN_PARITY', paths):
        fields = line.split('\t')
        if fields[0] != 'GQSC_MAIN_PARITY' or len(fields) != len(parity_names):
            raise RuntimeError(f'unexpected parity row: {line}')
        raw = dict(zip(parity_names, fields))
        parity_rows.append({
            'event_id': raw['event_id'], 'dataset_role': roles[raw['event_id']],
            **{key: value for key, value in raw.items() if key not in {'record', 'event_id'}}})
    write_csv('frozen_parity.csv', parity_rows)

    invocation_rows: list[dict] = []
    invocation_names = (
        'record', 'event_id', 'primary_evaluations', 'fallback_evaluations',
        'safe_stop_escape_evaluations', 'cached_same_input_reuses', 'r3_evaluations_total',
        'gqsc_s1_evaluations_total', 'non_escape_fallback_evaluations', 'outcome')
    for line in HELPER.run('GQSC_S1_INVOCATION_AUDIT', paths):
        fields = line.split('\t')
        if fields[0] != 'GQSC_INVOCATION' or len(fields) != len(invocation_names):
            raise RuntimeError(f'unexpected invocation row: {line}')
        raw = dict(zip(invocation_names, fields))
        invocation_rows.append({
            'event_id': raw['event_id'], 'dataset_role': roles[raw['event_id']],
            **{key: value for key, value in raw.items() if key not in {'record', 'event_id'}}})
    write_csv('evaluation_count_per_callback.csv', invocation_rows)

    sequence_rows: list[dict] = []
    sequence_names = (
        'record', 'event_id', 'scenario', 'step', 'state', 'has_output',
        'fresh_selected', 'invalidated', 'complete', 'suffix_revalidated',
        'suffix_hard_valid', 'guard_raw_revalidated', 'raw_validation_attempted',
        'original_candidate_identity', 'original_path_digest', 'output_path_digest',
        'suffix_point_count', 'fallback_kind', 'evaluator_collision_horizon_m',
        'lifecycle_collision_horizon_m', 'reason')
    for line in HELPER.run('GQSC_S1_SEQUENTIAL_REPLAY', paths):
        fields = line.split('\t')
        if fields[0] == 'GQSC_SEQUENCE_SKIP' and len(fields) == 4:
            sequence_rows.append({
                'event_id': fields[1], 'dataset_role': roles[fields[1]],
                'scenario': 'NOT_ELIGIBLE', 'step': 'SKIP', 'state': 'IDLE',
                'has_output': '0', 'fresh_selected': '0', 'invalidated': '0',
                'complete': '0', 'suffix_revalidated': '0', 'suffix_hard_valid': '0',
                'guard_raw_revalidated': '0', 'raw_validation_attempted': '0',
                'original_candidate_identity': 'NONE', 'original_path_digest': 'NONE',
                'output_path_digest': 'NONE', 'suffix_point_count': '0',
                'fallback_kind': 'NONE', 'evaluator_collision_horizon_m': 'nan',
                'lifecycle_collision_horizon_m': 'nan',
                'reason': fields[2] + ':' + fields[3]})
            continue
        if fields[0] != 'GQSC_SEQUENCE' or len(fields) != len(sequence_names):
            raise RuntimeError(f'unexpected sequence row: {line}')
        raw = dict(zip(sequence_names, fields))
        sequence_rows.append({
            'event_id': raw['event_id'], 'dataset_role': roles[raw['event_id']],
            **{key: value for key, value in raw.items() if key not in {'record', 'event_id'}}})
    write_csv('sequential_replay.csv', sequence_rows)

    timing_rows: list[dict] = []
    timing_names = (
        'record', 'event_id', 'repeat', 'evaluation_wall_us', 'gqsc_generation_us',
        'exact_validation_us', 'final_ranking_us', 'lifecycle_us', 'fallback_us',
        'callback_total_us', 'pair_proxy_count', 'reconstruction_count',
        'validator_count', 'outcome')
    timing_lines = HELPER.run('GQSC_S1_MAIN_TIMING', paths, {
        'GQSC_S1_TIMING_WARMUP': '2', 'GQSC_S1_TIMING_REPEATS': '7'})
    for line in timing_lines:
        fields = line.split('\t')
        if fields[0] != 'GQSC_MAIN_TIMING' or len(fields) != len(timing_names):
            raise RuntimeError(f'unexpected timing row: {line}')
        raw = dict(zip(timing_names, fields))
        timing_rows.append({
            'event_id': raw['event_id'], 'dataset_role': roles[raw['event_id']],
            **{key: value for key, value in raw.items() if key not in {'record', 'event_id'}}})
    write_csv('callback_runtime.csv', timing_rows)

    contract_rows: list[dict] = []
    for role in [entry[0] for entry in HELPER.CATALOG] + ['ALL_SEEN']:
        subset = parity_rows if role == 'ALL_SEEN' else [
            row for row in parity_rows if row['dataset_role'] == role]
        contract_rows.append({
            'dataset_role': role, 'event_count': len(subset), 'method_sha': METHOD_SHA,
            'pair_proxy_cap': 128,
            'observed_pair_proxy_max': max(int(row['pair_proxy_count']) for row in subset),
            'reconstruction_cap': 12,
            'observed_reconstruction_max': max(
                int(row['reconstruction_count']) for row in subset),
            'validator_cap': 12,
            'observed_validator_max': max(int(row['validator_count']) for row in subset),
            'all_bounds_pass': int(all(
                row['bounded_contract_pass'] == '1' for row in subset)),
            'legacy_seed_count_max': max(int(row['legacy_seed_count']) for row in subset)})
    write_csv('bounded_contract.csv', contract_rows)

    callback_ms = [float(row['callback_total_us']) / 1000.0 for row in timing_rows]
    evaluation_ms = [float(row['evaluation_wall_us']) / 1000.0 for row in timing_rows]
    parity_keys = (
        'lateral_order_exact', 'transition_order_exact', 'pair_order_exact', 'top12_exact',
        'path_digests_verdicts_exact', 'selected_path_exact', 'plan_selected_path_exact',
        'no_hidden_legacy_seed', 'snapshot_lineage_exact', 'method_sha_exact',
        'bounded_contract_pass')
    print('PARITY', sum(all(row[key] == '1' for key in parity_keys) for row in parity_rows),
          '/', len(parity_rows))
    print('HARD_VALID_EVENTS', sum(int(row['hard_valid_count']) > 0 for row in parity_rows))
    print('USABLE_VALID_EVENTS', sum(int(row['usable_valid_count']) > 0 for row in parity_rows))
    print('SEQUENCE', Counter(row['step'] for row in sequence_rows))
    print('GQSC_EVALUATIONS_PER_CALLBACK', Counter(
        int(row['gqsc_s1_evaluations_total']) for row in invocation_rows))
    print('CALLBACK_MS', {key: value for key, value in (
        ('p50', HELPER.percentile(callback_ms, 0.50)),
        ('p90', HELPER.percentile(callback_ms, 0.90)),
        ('p95', HELPER.percentile(callback_ms, 0.95)),
        ('p99', HELPER.percentile(callback_ms, 0.99)), ('max', max(callback_ms)))})
    print('EVALUATION_MS', {key: value for key, value in (
        ('p50', HELPER.percentile(evaluation_ms, 0.50)),
        ('p90', HELPER.percentile(evaluation_ms, 0.90)),
        ('p95', HELPER.percentile(evaluation_ms, 0.95)),
        ('p99', HELPER.percentile(evaluation_ms, 0.99)), ('max', max(evaluation_ms)))})


if __name__ == '__main__':
    main()
