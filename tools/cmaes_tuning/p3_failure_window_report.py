#!/usr/bin/env python3
"""Dump full detail for the failure window callbacks."""
import json
import sys

BASE = "/home/sungho/Documents/GitHub/2026_IFAC/runs/cmaes_tuning/p3_hybrid_p0_cma_v1/.evaluation/normal_finals_best_evaluable_retry"

WINDOWS = {"attempt_1": (394, 422), "attempt_2": (374, 412)}


def load(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    return rows


for attempt, (lo, hi) in WINDOWS.items():
    rows = load(f"{BASE}/{attempt}/p3_diagnostics.jsonl")
    print(f"\n{'#'*110}\n# {attempt}: callbacks {lo}..{hi}\n{'#'*110}")
    for r in rows:
        cb = r["callback_sequence"]
        if not (lo <= cb <= hi):
            continue
        obs = "; ".join(
            f"id{o['id']}:s[{o['s_start']:.3f},{o['s_end']:.3f}] d[{o['d_right']:.3f},{o['d_left']:.3f}]"
            for o in r.get("obstacles", [])
        )
        print(
            f"cb{cb:>4} s={r['ego_s']:.3f} d={r['ego_d']:+.3f} v={r['ego_speed_mps']:.2f} "
            f"oseq={r.get('obstacle_sequence')} state={r['lifecycle_state']:<15} owner={r['path_owner']:<15} "
            f"safestop={int(bool(r.get('safe_stop_active')))} m0/m1={r.get('m0_candidate_count')}/{r.get('m1_candidate_count')} "
            f"cluster_ids={r.get('cluster_obstacle_ids')} cluster_fwd=[{r.get('cluster_start_forward_m')},{r.get('cluster_end_forward_m')}]"
        )
        print(f"      sel={r.get('selected_candidate_identity')} src={r.get('selected_source')} branch={r.get('selected_branch')}")
        print(f"      fresh_rej={r.get('fresh_rejection','')!r} suffix_rej={r.get('suffix_rejection','')!r} "
              f"replan={r.get('same_callback_replan_attempted')}/{r.get('same_callback_replan_succeeded')} "
              f"guard_ready={r.get('selection_guard_ready')} guard_same_id={r.get('guard_contained_same_id_count')} "
              f"lifecycle_reason={r.get('lifecycle_reason','')[:80]!r}")
        print(f"      obstacles: {obs}")
