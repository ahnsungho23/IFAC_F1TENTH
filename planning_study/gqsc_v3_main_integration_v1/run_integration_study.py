#!/usr/bin/env python3
"""
Reproduce the seen-only frozen GQSC-v3 main-integration gates.

This script deliberately enumerates only DEVELOPMENT, VALIDATION_SEEN_AFTER_V1, and the frozen
success controls. It has no final-holdout path or discovery fallback.
"""

from __future__ import annotations

from collections import Counter
import csv
import os
from pathlib import Path
import subprocess


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
HARNESS = REPO / 'build/local_planning/p3_r3_k12_integration_harness'
CATALOG = (
    ('PILOT_SEEN_DEVELOPMENT_DATA', REPO / 'planning_study/p3_oracle_pilot_v2/inputs', 9),
    ('DEVELOPMENT', REPO / 'planning_study/p3_mapping_research_corpus_v1/raw_oracle/inputs', 40),
    ('VALIDATION_SEEN_AFTER_V1',
     REPO / 'planning_study/p3_geometry_conditioned_validation_v1/raw_oracle/inputs', 37),
    ('SUCCESS_CONTROL_SEEN',
     REPO / 'planning_study/p3_geometry_conditioned_method_v1/success_control_inputs', 18),
)


def events() -> tuple[list[Path], dict[str, str]]:
    paths: list[Path] = []
    roles: dict[str, str] = {}
    for role, directory, expected in CATALOG:
        found = sorted(directory.glob('*.event'))
        if len(found) != expected:
            raise RuntimeError(f'{role}: expected {expected} events, found {len(found)}')
        for path in found:
            if path.stem in roles:
                raise RuntimeError(f'duplicate event id: {path.stem}')
            paths.append(path)
            roles[path.stem] = role
    if len(paths) != 104:
        raise RuntimeError(f'seen-only corpus must contain 104 events, found {len(paths)}')
    return paths, roles


def run(mode: str, paths: list[Path], extra: dict[str, str] | None = None) -> list[str]:
    if not HARNESS.is_file():
        raise RuntimeError(f'missing Release harness: {HARNESS}')
    env = os.environ.copy()
    env.pop('LOCAL_PLANNING_RESEARCH_PARITY', None)
    env[mode] = '1'
    if extra:
        env.update(extra)
    completed = subprocess.run(
        [str(HARNESS), *(str(path) for path in paths)],
        check=True, text=True, stdout=subprocess.PIPE, env=env)
    return completed.stdout.splitlines()


def write_csv(path: Path, fieldnames: list[str], rows: list[dict]) -> None:
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


def percentile(values: list[float], quantile: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return float('nan')
    position = quantile * (len(ordered) - 1)
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def main() -> None:
    paths, roles = events()

    parity_rows: list[dict] = []
    for line in run('GQSC_V3_MAIN_PARITY', paths):
        fields = line.split('\t')
        if fields[0] != 'GQSC_MAIN_PARITY' or len(fields) != 20:
            raise RuntimeError(f'unexpected parity row: {line}')
        row = dict(zip((
            'record', 'event_id', 'lateral_order_exact', 'transition_order_exact',
            'pair_order_exact', 'top12_exact', 'path_digests_verdicts_exact',
            'selected_path_exact', 'plan_selected_path_exact', 'no_hidden_legacy_seed',
            'snapshot_lineage_exact', 'method_sha_exact',
            'bounded_contract_pass', 'pair_proxy_count', 'reconstruction_count',
            'validator_count', 'legacy_seed_count', 'hard_valid_count', 'selected_path_digest',
            'failure_classification'), fields))
        row['dataset_role'] = roles[row['event_id']]
        parity_rows.append(row)
    parity_fields = ['event_id', 'dataset_role'] + [
        key for key in parity_rows[0] if key not in {'record', 'event_id', 'dataset_role'}]
    write_csv(HERE / 'frozen_parity.csv', parity_fields, parity_rows)

    sequence_rows: list[dict] = []
    for line in run('GQSC_V3_SEQUENTIAL_REPLAY', paths):
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
                'reason': fields[2] + ':' + fields[3],
            })
            continue
        if fields[0] != 'GQSC_SEQUENCE' or len(fields) != 21:
            raise RuntimeError(f'unexpected sequential row: {line}')
        names = (
            'record', 'event_id', 'scenario', 'step', 'state', 'has_output',
            'fresh_selected', 'invalidated', 'complete', 'suffix_revalidated',
            'suffix_hard_valid', 'guard_raw_revalidated', 'raw_validation_attempted',
            'original_candidate_identity', 'original_path_digest', 'output_path_digest',
            'suffix_point_count', 'fallback_kind', 'evaluator_collision_horizon_m',
            'lifecycle_collision_horizon_m', 'reason')
        row = dict(zip(names, fields))
        row['dataset_role'] = roles[row['event_id']]
        sequence_rows.append(row)
    sequence_fields = [
        'event_id', 'dataset_role', 'scenario', 'step', 'state', 'has_output',
        'fresh_selected', 'invalidated', 'complete', 'suffix_revalidated',
        'suffix_hard_valid', 'guard_raw_revalidated', 'raw_validation_attempted',
        'original_candidate_identity', 'original_path_digest', 'output_path_digest',
        'suffix_point_count', 'fallback_kind', 'evaluator_collision_horizon_m',
        'lifecycle_collision_horizon_m', 'reason']
    write_csv(HERE / 'sequential_replay.csv', sequence_fields, sequence_rows)

    timing_rows: list[dict] = []
    timing_lines = run('GQSC_V3_MAIN_TIMING', paths, {
        'GQSC_V3_TIMING_WARMUP': '2', 'GQSC_V3_TIMING_REPEATS': '7'})
    for line in timing_lines:
        fields = line.split('\t')
        if fields[0] != 'GQSC_MAIN_TIMING' or len(fields) != 14:
            raise RuntimeError(f'unexpected timing row: {line}')
        names = (
            'record', 'event_id', 'repeat', 'evaluation_wall_us', 'gqsc_generation_us',
            'exact_validation_us', 'final_ranking_us', 'lifecycle_us', 'fallback_us',
            'callback_total_us', 'pair_proxy_count', 'reconstruction_count',
            'validator_count', 'outcome')
        row = dict(zip(names, fields))
        row['dataset_role'] = roles[row['event_id']]
        timing_rows.append(row)
    timing_fields = [
        'event_id', 'dataset_role', 'repeat', 'evaluation_wall_us', 'gqsc_generation_us',
        'exact_validation_us', 'final_ranking_us', 'lifecycle_us', 'fallback_us',
        'callback_total_us', 'pair_proxy_count', 'reconstruction_count', 'validator_count',
        'outcome']
    write_csv(HERE / 'callback_runtime.csv', timing_fields, timing_rows)

    contract_rows: list[dict] = []
    for role in [entry[0] for entry in CATALOG] + ['ALL_SEEN']:
        subset = parity_rows if role == 'ALL_SEEN' else [
            row for row in parity_rows if row['dataset_role'] == role]
        contract_rows.append({
            'dataset_role': role,
            'event_count': len(subset),
            'method_sha': '965f6ce65b7ce5b1c6426a0975c1dfafe779e89eb20308bb09a11a1b4a22e780',
            'pair_proxy_cap': 128,
            'observed_pair_proxy_max': max(int(row['pair_proxy_count']) for row in subset),
            'reconstruction_cap': 12,
            'observed_reconstruction_max': max(int(row['reconstruction_count']) for row in subset),
            'validator_cap': 12,
            'observed_validator_max': max(int(row['validator_count']) for row in subset),
            'all_bounds_pass': int(all(row['bounded_contract_pass'] == '1' for row in subset)),
            'legacy_seed_count': max(int(row['legacy_seed_count']) for row in subset),
        })
    contract_fields = list(contract_rows[0])
    write_csv(HERE / 'bounded_contract.csv', contract_fields, contract_rows)

    callback_values = [float(row['callback_total_us']) / 1000.0 for row in timing_rows]
    evaluation_values = [float(row['evaluation_wall_us']) / 1000.0 for row in timing_rows]
    print('PARITY', len(parity_rows), sum(
        all(row[key] == '1' for key in (
            'lateral_order_exact', 'transition_order_exact', 'pair_order_exact', 'top12_exact',
            'path_digests_verdicts_exact', 'selected_path_exact', 'plan_selected_path_exact',
            'no_hidden_legacy_seed', 'snapshot_lineage_exact', 'method_sha_exact',
            'bounded_contract_pass'))
        for row in parity_rows))
    print('SEQUENCE_ROWS', len(sequence_rows), Counter(row['step'] for row in sequence_rows))
    print('CALLBACK_MS', {
        'p50': percentile(callback_values, 0.50), 'p90': percentile(callback_values, 0.90),
        'p95': percentile(callback_values, 0.95), 'p99': percentile(callback_values, 0.99),
        'max': max(callback_values)})
    print('EVALUATION_MS', {
        'p50': percentile(evaluation_values, 0.50), 'p90': percentile(evaluation_values, 0.90),
        'p95': percentile(evaluation_values, 0.95), 'p99': percentile(evaluation_values, 0.99),
        'max': max(evaluation_values)})
    print('OUTCOMES', Counter(row['outcome'] for row in timing_rows))
    print('FALLBACKS', Counter(
        row['fallback_kind'] for row in sequence_rows if row['step'] == 'INVALIDATE_AND_FALLBACK'))


if __name__ == '__main__':
    main()
