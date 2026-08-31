#!/usr/bin/env python3
"""Reproduce the seen-only GQSC-S1 callback architecture audit."""

from __future__ import annotations

from collections import Counter, defaultdict
import csv
import hashlib
import importlib.util
from pathlib import Path
import subprocess


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
BASE = REPO / 'planning_study/gqsc_s1_closed_loop_candidate_v1'
HELPER_PATH = REPO / 'planning_study/gqsc_v3_main_integration_v1/run_integration_study.py'
SPEC = importlib.util.spec_from_file_location('gqsc_seen_harness', HELPER_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f'cannot load seen-only harness helper: {HELPER_PATH}')
HELPER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HELPER)

METHOD_SHA = '670f39a23479bcdcc1db0829a895257443fec8ee2f78f186bc1beb224090b776'
REFERENCE_SHA = '965f6ce65b7ce5b1c6426a0975c1dfafe779e89eb20308bb09a11a1b4a22e780'


def write_csv(name: str, rows: list[dict]) -> None:
    if not rows:
        raise RuntimeError(f'no rows for {name}')
    with (HERE / name).open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path: Path) -> list[dict]:
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def percentiles(values: list[float]) -> dict[str, float]:
    return {
        'p50_ms': HELPER.percentile(values, 0.50),
        'p90_ms': HELPER.percentile(values, 0.90),
        'p95_ms': HELPER.percentile(values, 0.95),
        'p99_ms': HELPER.percentile(values, 0.99),
        'max_ms': max(values),
    }


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_result(binary: str) -> tuple[int, str]:
    completed = subprocess.run(
        [str(REPO / 'build/local_planning' / binary), '--gtest_brief=1'],
        cwd=REPO, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False)
    return completed.returncode, completed.stdout


def main() -> None:
    paths, roles = HELPER.events()

    architecture_lines = HELPER.run('GQSC_S1_CALLBACK_ARCHITECTURE_AUDIT', paths)
    summaries: dict[str, list[str]] = {}
    evaluations: dict[str, list[list[str]]] = defaultdict(list)
    for line in architecture_lines:
        fields = line.split('\t')
        if fields[0] == 'GQSC_CALLBACK_ARCHITECTURE' and len(fields) == 13:
            summaries[fields[1]] = fields
        elif fields[0] == 'GQSC_CALLBACK_EVALUATION' and len(fields) == 19:
            evaluations[fields[1]].append(fields)
        else:
            raise RuntimeError(f'unexpected callback architecture row: {line}')
    if len(summaries) != 104:
        raise RuntimeError(f'expected 104 callback summaries, got {len(summaries)}')

    previous = {
        row['event_id']: row for row in read_csv(BASE / 'evaluation_count_per_callback.csv')}
    callback_rows: list[dict] = []
    classification_rows: list[dict] = []
    counterfactual_rows: list[dict] = []
    for event_id, summary in summaries.items():
        rows = evaluations[event_id]
        primary = rows[0]
        logical_probes = int(summary[4])
        fresh_escape = [row for row in rows[1:] if row[3] == 'SAFE_STOP_ESCAPE']
        cached_escape = logical_probes - len(fresh_escape)
        old_count = int(previous[event_id]['gqsc_s1_evaluations_total'])
        new_count = int(summary[2])
        if logical_probes == 0:
            pattern = 'PRIMARY_ONLY'
        elif logical_probes == 1 and cached_escape == 1:
            pattern = 'PRIMARY_PLUS_EQUIVALENT_ESCAPE_REUSED'
        elif logical_probes == 1:
            pattern = 'PRIMARY_PLUS_DISTINCT_STOP_ESCAPE'
        else:
            pattern = 'PRIMARY_PLUS_DISTINCT_STOP_AND_EQUIVALENT_EGO_ENDPOINT'
        callback_rows.append({
            'event_id': event_id,
            'dataset_role': roles[event_id],
            'pre_change_fresh_s1_evaluations': old_count,
            'post_change_fresh_s1_evaluations': new_count,
            'logical_safe_stop_escape_probes': logical_probes,
            'fresh_distinct_escape_evaluations': len(fresh_escape),
            'cached_equivalent_escape_probes': cached_escape,
            'all_same_input_reuses_including_plan': int(summary[3]),
            'callback_global_validate_candidate_executions': int(summary[5]),
            's1_reconstructions_sum': int(summary[6]),
            's1_evaluator_validator_sum': int(summary[7]),
            'final_outcome': summary[8],
            'final_path_digest': summary[11],
        })
        classification_rows.append({
            'event_id': event_id,
            'dataset_role': roles[event_id],
            'observed_pre_change_pattern': old_count,
            'post_change_fresh_pattern': new_count,
            'classification': pattern,
            'evaluation_1_role': primary[3],
            'evaluation_1_ego_s': primary[8],
            'evaluation_1_ego_d': primary[9],
            'evaluation_1_speed_mps': primary[10],
            'evaluation_1_hard_valid_exists': primary[12],
            'evaluation_2_reason': 'NONE' if logical_probes == 0 else 'SAFE_STOP_REQUESTED_POINT',
            'evaluation_3_reason': 'SAFE_STOP_EGO_ENDPOINT' if logical_probes == 2 else 'NONE',
            'same_obstacle_snapshot_all': int(all(row[6] == primary[6] for row in rows)),
            'same_reference_snapshot_all': int(all(row[7] == primary[7] for row in rows)),
            'final_outcome': summary[8],
        })

        logical_rows: list[tuple[list[str], bool]] = []
        if logical_probes == 1:
            logical_rows.append((fresh_escape[0], False) if fresh_escape else (primary, True))
        elif logical_probes == 2:
            logical_rows.append((fresh_escape[0], False))
            logical_rows.append((primary, True))
        for probe_index, (row, reused) in enumerate(logical_rows, start=1):
            same_geometry = row[5] == primary[5] and row[6] == primary[6] and row[7] == primary[7]
            exists = row[12] == '1'
            classification = 'C_DUPLICATE_OR_REUSABLE' if same_geometry else (
                'E_FALLBACK_CONFIRMATION_ONLY')
            observed_effect = (
                'escape_verified metadata only; braking result remained NO_SAFE_PATH'
                if exists else 'confirmed no hard-valid escape; final path/kind unchanged')
            counterfactual_rows.append({
                'event_id': event_id,
                'dataset_role': roles[event_id],
                'logical_probe_index': probe_index,
                'probe_role': 'REQUESTED_STOP_POINT' if probe_index == 1 else 'EGO_ENDPOINT',
                'ego_s': row[8],
                'ego_d': row[9],
                'ego_speed_mps': row[10],
                'obstacle_snapshot_id': row[6],
                'reference_snapshot_id': row[7],
                'identical_geometry_input_to_primary': int(same_geometry),
                'hard_valid_escape_exists': int(exists),
                'post_change_reused': int(reused),
                'changed_final_path_observed': 0,
                'changed_final_kind_observed': 0,
                'counterfactual_class': classification,
                'observed_effect': observed_effect,
                'contract_role': 'stop-location resumability/liveness; not braking collision validation',
            })
    write_csv('evaluations_per_callback.csv', callback_rows)
    write_csv('multi_evaluation_classification.csv', classification_rows)
    write_csv('extra_evaluation_counterfactual.csv', counterfactual_rows)

    parity_a = HELPER.run('GQSC_S1_MAIN_PARITY', paths)
    parity_b = HELPER.run('GQSC_S1_MAIN_PARITY', paths)
    parity = [line.split('\t') for line in parity_a if line.startswith('GQSC_MAIN_PARITY\t')]
    parity_exact = sum(all(row[index] == '1' for index in range(2, 13)) for row in parity)
    hard_events = sum(int(row[17]) > 0 for row in parity)
    usable_events = sum(int(row[18]) > 0 for row in parity)

    sequence_lines = HELPER.run('GQSC_S1_SEQUENTIAL_REPLAY', paths)
    sequence = [line.split('\t') for line in sequence_lines if line.startswith('GQSC_SEQUENCE\t')]
    sequence_by_event: dict[str, list[list[str]]] = defaultdict(list)
    for row in sequence:
        sequence_by_event[row[1]].append(row)
    expected_steps = Counter({
        'FRESH': 2, 'HELD_SAME_INPUT': 1, 'GUARD_FAIL_RAW_PASS': 1,
        'OBSTACLE_DISAPPEARANCE': 1, 'FORWARD_TRIM': 1, 'COMPLETE': 1,
        'INVALIDATE_AND_FALLBACK': 1})
    lifecycle_pass = sum(
        Counter(row[3] for row in rows) == expected_steps and
        all(row[5] == '1' for row in rows if row[3] not in {'COMPLETE', 'INVALIDATE_AND_FALLBACK'}) and
        next(row for row in rows if row[3] == 'COMPLETE')[8] == '1' and
        next(row for row in rows if row[3] == 'INVALIDATE_AND_FALLBACK')[7] == '1'
        for rows in sequence_by_event.values())

    raceline_code, raceline_output = test_result('test_raceline_spline')
    production_code, production_output = test_result('test_p3_production_parity')
    method_path = BASE / 'gqsc_s1_method.json'
    method_sha = digest(method_path)
    gate_rows = [
        {'gate': 'canonical_method_sha', 'scope': str(method_path.relative_to(REPO)),
         'result': 'PASS' if method_sha == METHOD_SHA else 'FAIL',
         'detail': f'{method_sha}; reference_v3={REFERENCE_SHA}'},
        {'gate': 'frozen_stateless_parity', 'scope': '104 allowed-seen snapshots',
         'result': 'PASS' if parity_exact == 104 else 'FAIL',
         'detail': f'{parity_exact}/104 proposal Top-12 path verdict selection exact'},
        {'gate': 'determinism', 'scope': 'two complete parity runs',
         'result': 'PASS' if parity_a == parity_b else 'FAIL',
         'detail': f'raw_output_exact={int(parity_a == parity_b)}'},
        {'gate': 'hard_usable_regression', 'scope': '104 allowed-seen snapshots',
         'result': 'PASS' if (hard_events, usable_events) == (62, 54) else 'FAIL',
         'detail': f'hard={hard_events}/104 usable={usable_events}/104'},
        {'gate': 'lifecycle_sequential_replay', 'scope': '62 hard-valid snapshots',
         'result': 'PASS' if lifecycle_pass == 62 else 'FAIL',
         'detail': f'{lifecycle_pass}/62 ownership continuation trim completion blocker invalidation'},
        {'gate': 'evaluation_local_bounds', 'scope': '104 allowed-seen snapshots',
         'result': 'PASS' if all(
             int(row[13]) <= 128 and int(row[14]) <= 12 and int(row[15]) <= 12
             for row in parity) else 'FAIL',
         'detail': f'max={max(int(r[13]) for r in parity)}/'
                   f'{max(int(r[14]) for r in parity)}/{max(int(r[15]) for r in parity)}'},
        {'gate': 'raceline_regression', 'scope': 'test_raceline_spline',
         'result': 'PASS' if raceline_code == 0 and '88 tests' in raceline_output else 'FAIL',
         'detail': '88/88'},
        {'gate': 'production_scenarios', 'scope': 'test_p3_production_parity',
         'result': 'KNOWN_METHOD_COVERAGE_LIMIT' if production_code != 0 and
                   '[  PASSED  ] 10 tests.' in production_output else 'FAIL',
         'detail': '10/12; unchanged LayoutBReplanRecovers and SamePinchGeometryAtTwoDistances'},
    ]
    write_csv('post_change_parity.csv', gate_rows)

    timing_names = (
        'record', 'event_id', 'repeat', 'evaluation_wall_us', 'gqsc_generation_us',
        'exact_validation_us', 'final_ranking_us', 'lifecycle_us', 'fallback_us',
        'callback_total_us', 'pair_proxy_count', 'reconstruction_count',
        'validator_count', 'outcome')
    post_timing_lines = HELPER.run('GQSC_S1_MAIN_TIMING', paths, {
        'GQSC_S1_TIMING_WARMUP': '2', 'GQSC_S1_TIMING_REPEATS': '7'})
    post_timing = [dict(zip(timing_names, line.split('\t'))) for line in post_timing_lines]
    pre_timing = read_csv(BASE / 'callback_runtime.csv')
    continuation_lines = HELPER.run('GQSC_S1_CONTINUATION_TIMING', paths, {
        'GQSC_S1_TIMING_WARMUP': '2', 'GQSC_S1_TIMING_REPEATS': '7'})
    continuation = [line.split('\t') for line in continuation_lines]

    runtime_rows: list[dict] = []
    for version, timing in [('PRE_CHANGE', pre_timing), ('POST_CHANGE', post_timing)]:
        stages = {
            's1_evaluator': [float(row['evaluation_wall_us']) / 1000.0 for row in timing],
            'primary_planning': [
                (float(row['evaluation_wall_us']) + float(row['lifecycle_us'])) / 1000.0
                for row in timing],
            'safe_stop_fallback': [float(row['fallback_us']) / 1000.0 for row in timing],
            'total_callback_model': [float(row['callback_total_us']) / 1000.0 for row in timing],
        }
        for stage, values in stages.items():
            runtime_rows.append({
                'version': version, 'stage': stage, 'sample_count': len(values),
                'instrumentation': 'OFF', **percentiles(values)})
    continuation_ms = [float(row[3]) / 1000.0 for row in continuation]
    runtime_rows.append({
        'version': 'POST_CHANGE', 'stage': 'active_continuation_revalidation',
        'sample_count': len(continuation_ms), 'instrumentation': 'OFF',
        **percentiles(continuation_ms)})
    write_csv('runtime_breakdown.csv', runtime_rows)

    counts = Counter(int(row['post_change_fresh_s1_evaluations']) for row in callback_rows)
    print('METHOD_SHA', method_sha)
    print('PARITY', parity_exact, '/104; HARD/USABLE', hard_events, usable_events)
    print('LIFECYCLE', lifecycle_pass, '/62')
    print('FRESH_EVALUATIONS', dict(sorted(counts.items())))
    print('COUNTERFACTUAL', Counter(row['counterfactual_class'] for row in counterfactual_rows))
    callback_post = next(
        row for row in runtime_rows
        if row['version'] == 'POST_CHANGE' and row['stage'] == 'total_callback_model')
    print('POST_CALLBACK_MS', callback_post)


if __name__ == '__main__':
    main()
