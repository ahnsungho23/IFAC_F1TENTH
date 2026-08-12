#!/usr/bin/env python3
"""Render the deterministic path-family audit's raw TSV files."""

from __future__ import annotations

import argparse
import csv
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


FAMILIES = ("A", "B1", "B2", "B3", "C")
SCENARIOS = (
    "validation_004",
    "validation_014",
    "validation_019",
    "validation_034",
    "validation_039",
)


def read_tsv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream, delimiter="\t"))


def number(value: str) -> int | float | None:
    if value == "":
        return None
    parsed = float(value)
    return int(parsed) if parsed.is_integer() else parsed


def boolean(value: str) -> bool:
    return value.lower() == "true"


def histogram(value: str) -> dict[str, int]:
    result: dict[str, int] = {}
    if not value:
        return result
    for item in value.split(";"):
        reason, count = item.rsplit("=", 1)
        result[reason] = int(count)
    return result


def csv_write(path: Path, fieldnames: list[str], rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def short_scenario(name: str) -> str:
    return name.removeprefix("validation_")


def display(value: str, digits: int = 4) -> str:
    if value == "":
        return "-"
    return f"{float(value):.{digits}f}"


def family_cell(row: dict[str, str]) -> str:
    if row["gate_reason"] == "not_needed":
        return "not needed"
    return f'{row["feasible"]}/{row["generated"]}'


def render_readme(
    output_dir: Path,
    scenario_rows: list[dict[str, str]],
    all_family_rows: list[dict[str, str]],
    performance: dict[str, Any],
) -> None:
    scenarios = {row["scenario"]: row for row in scenario_rows}
    families = {(row["scenario"], row["family"]): row for row in all_family_rows}

    lines = [
        "# Path-family feasibility audit v1",
        "",
        "## 결론",
        "",
        "현재 hard safety/vehicle constraint를 그대로 적용했을 때 `validation_014`만 "
        "B1의 실제 available-entry 100% 확장으로 feasible해졌다. 나머지 네 scenario는 "
        "audit-only C2 variable-d family까지 feasible 후보가 0개였으므로 P로 분류했다. "
        "CMA-ES와 production tuning은 실행하지 않았고, 어떤 family도 production planner에 반영하지 않았다.",
        "",
        "| scenario | A | B1 | B2 | B3 | C | class | dominant constraint |",
        "|---|---:|---:|---:|---:|---:|:---:|---|",
    ]
    for scenario in SCENARIOS:
        summary = scenarios[scenario]
        cells = [family_cell(families[(scenario, family)]) for family in FAMILIES]
        lines.append(
            f'| {short_scenario(scenario)} | {" | ".join(cells)} | '
            f'{summary["classification"]} | {summary["dominant_constraint"]} |'
        )

    lines.extend(
        [
            "",
            "표의 값은 `hard-feasible/generated`이다.",
            "",
            "## Family별 수치",
            "",
            "| scenario | family | generated | feasible | slack | peak slope | peak curvature | "
            "peak curvature rate | footprint clearance m | obstacle clearance m | runtime ms |",
            "|---|:---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for scenario in SCENARIOS:
        for family in FAMILIES:
            row = families[(scenario, family)]
            lines.append(
                f'| {short_scenario(scenario)} | {family} | {row["generated"]} | '
                f'{row["feasible"]} | {display(row["best_slack"])} | '
                f'{display(row["best_peak_slope"])} | {display(row["best_peak_curvature"])} | '
                f'{display(row["best_peak_curvature_rate"])} | '
                f'{display(row["best_footprint_clearance"])} | '
                f'{display(row["best_obstacle_clearance"])} | {display(row["runtime_ms"], 3)} |'
            )

    lines.extend(
        [
            "",
            "정확한 entry/exit 길이와 side별 rejection histogram은 각 CSV에 보존했다.",
            "",
            "## Family C side별 closest evidence",
            "",
            "| scenario | side | footprint violation m | obstacle violation m | curvature excess | "
            "slope excess | curvature-rate excess |",
            "|---|:---:|---:|---:|---:|---:|---:|",
            "| 004 | left | 0.000000 | 0.000291 | 0.205179 | 0.000000 | 0.000000 |",
            "| 004 | right | local obstacle/track interval empty | - | - | - | - |",
            "| 019 | left | 0.191744 | 0.002703 | 1.376443 | 0.000000 | 0.000000 |",
            "| 019 | right | 0.184685 | 0.000000 | 0.061697 | 0.000000 | 0.000000 |",
            "| 034 | left | local obstacle/track interval empty | - | - | - | - |",
            "| 034 | right | 0.162493 | 0.000000 | 0.000000 | 0.000000 | 0.000000 |",
            "| 039 | left | 0.171473 | 0.008720 | 0.303708 | 0.000000 | 0.000000 |",
            "| 039 | right | 0.055195 | 0.014215 | 0.018405 | 0.000000 | 0.000000 |",
            "",
            "## Scenario 판정",
            "",
            "- **004 — P:** C2에서도 0/540. right는 obstacle/track local interval 자체가 비었고, "
            "closest left C 후보도 obstacle 0.000291 m 침범과 curvature limit 0.205179 rad/m 초과가 남았다.",
            "- **014 — E:** B1에서 2/60. 최선 후보는 right `target_d=-0.377943 m`, "
            "entry `1.792384 m`이며 slack `0.037679`, footprint clearance `0.009591 m`, "
            "obstacle clearance `0.056519 m`이다. Exit 확장만으로는 0/75였다.",
            "- **019 — P:** C2에서도 0/1080. closest C 후보에 footprint 0.184685 m 침범과 "
            "curvature limit 0.061697 rad/m 초과가 함께 남았다.",
            "- **034 — P:** C2에서도 0/540. left local interval이 비었고, closest right C 후보의 "
            "footprint가 wall bound를 0.162493 m 침범했다.",
            "- **039 — P:** C2에서도 0/1080. closest C 후보에 footprint 0.055195 m, "
            "obstacle 0.014215 m 침범과 curvature limit 0.018405 rad/m 초과가 함께 남았다.",
            "",
            "## Family A harness 검증",
            "",
            "004/014/034/039는 기존 bag의 대표 no-feasible 입력에서 알려진 side/rejection signature와 "
            "일치했다. 019의 기존 기록은 현재 production code와 불일치했다: 기존 기록은 right "
            "target-range gate였지만, 현재 code의 짧은 deterministic replay에서 동일 event "
            "(`source_stamp=11.74 s`, obstacle sequence 175)는 right 후보 45개를 실제 생성했다. "
            "그 분포는 maximum_lateral_slope 30, footprint_track_bound 9, maximum_curvature 6이다. "
            "offline Family A가 이 90개 전체의 값과 histogram을 재현했으므로, 019는 stale historical "
            "signature가 아니라 current replay snapshot을 source of truth로 사용했다.",
            "",
            "## 방법과 격리",
            "",
            "Family A/B는 diagnostic target에서 production `raceline_spline_planner.cpp`를 직접 컴파일해 "
            "obstacle expansion, reference interpolation, Cartesian reconstruction, curvature recomputation, "
            "footprint track-bound, obstacle clearance, slope/curvature/rate, velocity feasibility와 slack "
            "계산을 그대로 호출했다. C만 audit-only piecewise quintic-Hermite C2 lateral representation이며 "
            "그 결과도 동일 production validators를 통과해야 feasible로 집계했다. Private 접근은 이 "
            "`BUILD_TESTING` diagnostic translation unit에만 한정했다.",
            "",
            "production node source, YAML parameter, controller, perception, tracking LUT, footprint validator, "
            "state machine과 safe-stop lifecycle은 변경하지 않았다. 변경은 test-only CMake target, 세 audit "
            "utility와 이 결과 디렉터리에 한정된다.",
            "",
            "## Determinism과 자원 사용",
            "",
            f'- 대표 scenario: `{performance["determinism"]["scenario"]}`',
            f'- digest 1/2: `{performance["determinism"]["first_digest"]}` / '
            f'`{performance["determinism"]["second_digest"]}`',
            f'- 동일 여부: `{str(performance["determinism"]["passed"]).lower()}` '
            "(candidate/feasible count, best candidate, histogram, geometry hash 포함)",
            f'- offline helper wall/CPU: `{performance["offline_geometry"]["wall_s"]:.6f} s` / '
            f'`{performance["offline_geometry"]["cpu_s"]:.6f} s`',
            f'- primary/repeat candidate evaluation: '
            f'`{performance["offline_geometry"]["primary_candidate_evaluations"]}` / '
            f'`{performance["offline_geometry"]["determinism_repeat_candidate_evaluations"]}`',
            f'- replay: 019 snapshot 복구용 1회, 500 lockstep step / simulated 5.0 s; '
            f'wall `{performance["snapshot_recovery_replay"]["wall_s_estimated"]:.3f} s` '
            "(ROS log timestamp 추정, replay CPU는 계측되지 않음)",
            f'- 최종 audit target rebuild / package test wall: '
            f'`{performance["build_and_validation"]["latest_target_rebuild_wall_s"]:.1f} s` / '
            f'`{performance["build_and_validation"]["package_test_wall_s"]:.1f} s`',
            f'- 관측 wall 합계: `{performance["total_observed_wall_s"]:.3f} s` '
            "(기존 snapshot 추출과 앞선 exploratory build 시간 제외)",
            "",
            "## 다음 단계",
            "",
            "production 적용은 하지 않는다. 다음으로 검토할 수 있는 가장 작은 가설은 014에 한정해 "
            "B1의 actual available-entry 100% parameterization을 production 후보로 별도 검증하는 것이다. "
            "004/019/034/039는 C2 표현력 확장으로도 해결되지 않았으므로 path-family 추가보다 "
            "track/footprint/obstacle/curvature의 국소 물리 호환성을 먼저 조사해야 한다.",
            "",
            "## 파일",
            "",
            "- `summary.json`: machine-readable 판정과 evidence",
            "- `scenario_comparison.csv`: scenario/family 집계와 실제 길이",
            "- `rejection_histograms.csv`: scenario/family/side rejection counts",
            "- `best_candidates.csv`: feasible 또는 closest candidate와 constraint violation",
            "- `performance.json`: replay, determinism, CPU/wall usage",
            "- `snapshots/*.snapshot.tsv`: audit 입력을 자체 포함한 planner snapshots",
            "",
        ]
    )
    (output_dir / "README.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--git-commit", required=True)
    args = parser.parse_args()
    output_dir = args.input_dir.resolve()

    scenario_rows = read_tsv(output_dir / "raw_scenarios.tsv")
    family_rows = read_tsv(output_dir / "raw_families.tsv")
    best_rows = read_tsv(output_dir / "raw_best_candidates.tsv")
    raw_performance = read_tsv(output_dir / "raw_performance.tsv")[0]
    all_family_rows = [row for row in family_rows if row["side"] == "all"]

    scenario_by_name = {row["scenario"]: row for row in scenario_rows}
    comparison_rows: list[dict[str, Any]] = []
    for row in all_family_rows:
        scenario = scenario_by_name[row["scenario"]]
        comparison_rows.append(
            {
                "scenario": row["scenario"],
                "family": row["family"],
                "evaluated": row["gate_reason"] != "not_needed",
                "generated": row["generated"],
                "hard_feasible": row["feasible"],
                "best_status": row["best_status"],
                "best_minimum_normalized_safety_slack": row["best_slack"],
                "best_peak_lateral_slope": row["best_peak_slope"],
                "best_peak_curvature_radpm": row["best_peak_curvature"],
                "best_peak_curvature_rate_radpm2": row["best_peak_curvature_rate"],
                "best_footprint_wall_clearance_m": row["best_footprint_clearance"],
                "best_obstacle_clearance_m": row["best_obstacle_clearance"],
                "best_effective_entry_length_m": row["best_entry_length"],
                "best_exit_length_m": row["best_exit_length"],
                "runtime_ms": row["runtime_ms"],
                "entry_lengths_used_m": row["entry_lengths_used"],
                "exit_lengths_used_m": row["exit_lengths_used"],
                "classification": scenario["classification"],
                "dominant_constraint": scenario["dominant_constraint"],
                "secondary_constraints": scenario["secondary_constraints"],
                "gate_reason": row["gate_reason"],
                "input_reconstruction": scenario["input_reconstruction"],
                "logical_stamp_ns": scenario["logical_stamp_ns"],
                "source_bag": scenario["source_bag"],
                "config_sha256": scenario["config_sha256"],
                "scenario_digest": scenario["digest"],
            }
        )
    csv_write(
        output_dir / "scenario_comparison.csv",
        list(comparison_rows[0]),
        comparison_rows,
    )

    rejection_rows: list[dict[str, Any]] = []
    for row in family_rows:
        for reason, count in histogram(row["rejection_histogram"]).items():
            rejection_rows.append(
                {
                    "scenario": row["scenario"],
                    "family": row["family"],
                    "side": row["side"],
                    "generated": row["generated"],
                    "hard_feasible": row["feasible"],
                    "rejection_reason": reason,
                    "count": count,
                    "gate_reason": row["gate_reason"],
                }
            )
    csv_write(
        output_dir / "rejection_histograms.csv",
        list(rejection_rows[0]),
        rejection_rows,
    )
    csv_write(output_dir / "best_candidates.csv", list(best_rows[0]), best_rows)

    families_by_scenario: dict[str, dict[str, dict[str, Any]]] = {}
    for row in all_family_rows:
        families_by_scenario.setdefault(row["scenario"], {})[row["family"]] = {
            "evaluated": row["gate_reason"] != "not_needed",
            "generated": int(row["generated"]),
            "hard_feasible": int(row["feasible"]),
            "runtime_ms": number(row["runtime_ms"]),
            "gate_reason": row["gate_reason"] or None,
            "rejection_histogram": histogram(row["rejection_histogram"]),
            "best": {
                "status": row["best_status"] or None,
                "minimum_normalized_safety_slack": number(row["best_slack"]),
                "peak_lateral_slope": number(row["best_peak_slope"]),
                "peak_curvature_radpm": number(row["best_peak_curvature"]),
                "peak_curvature_rate_radpm2": number(row["best_peak_curvature_rate"]),
                "footprint_wall_clearance_m": number(row["best_footprint_clearance"]),
                "obstacle_clearance_m": number(row["best_obstacle_clearance"]),
                "effective_entry_length_m": number(row["best_entry_length"]),
                "exit_length_m": number(row["best_exit_length"]),
                "geometry_hash": row["best_geometry_hash"] or None,
            },
            "entry_lengths_used_m": [float(v) for v in row["entry_lengths_used"].split(";") if v],
            "exit_lengths_used_m": [float(v) for v in row["exit_lengths_used"].split(";") if v],
        }

    c_evidence: dict[str, dict[str, dict[str, Any]]] = {}
    for row in best_rows:
        if row["family"] != "C":
            continue
        c_evidence.setdefault(row["scenario"], {})[row["side"]] = {
            "status": row["status"],
            "d_start_m": number(row["d_start"]),
            "d_mid_m": number(row["d_mid"]),
            "d_end_m": number(row["d_end"]),
            "minimum_footprint_wall_violation_m": number(row["footprint_violation"]),
            "minimum_obstacle_clearance_violation_m": number(row["obstacle_violation"]),
            "curvature_over_limit_radpm": number(row["curvature_excess"]),
            "lateral_slope_over_limit": number(row["slope_excess"]),
            "curvature_rate_over_limit_radpm2": number(row["curvature_rate_excess"]),
            "rejection_reason": row["rejection_reason"],
            "geometry_hash": row["geometry_hash"],
        }
    for row in family_rows:
        if row["family"] != "C" or row["side"] == "all" or not row["gate_reason"]:
            continue
        c_evidence.setdefault(row["scenario"], {}).setdefault(
            row["side"],
            {
                "status": "gate_rejected",
                "gate_reason": row["gate_reason"],
            },
        )

    summary = {
        "schema": "path_family_feasibility_audit/1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "git_commit": args.git_commit,
        "scope": {
            "scenarios": list(SCENARIOS),
            "families": list(FAMILIES),
            "cma_es_executed": False,
            "production_code_or_parameters_changed": False,
            "winner_productionized": False,
        },
        "source_of_truth": {
            "production_geometry": "src/local_planning/src/raceline_spline_planner.cpp",
            "production_header": "src/local_planning/include/local_planning/raceline_spline_planner.hpp",
            "production_config": "src/local_planning/config/local_planning.yaml",
            "config_sha256": scenario_rows[0]["config_sha256"],
            "audit_helper": "tools/cmaes_tuning/path_family_feasibility_audit.cpp",
        },
        "harness_verification": {
            "family_a_current_planner_verified_scenarios": list(SCENARIOS),
            "existing_bag_signature_consistent_scenarios": [
                "validation_004",
                "validation_014",
                "validation_034",
                "validation_039",
            ],
            "snapshot_recovery_replay_scenarios": ["validation_019"],
            "validation_019_historical_signature_stale": True,
            "validation_019_current_family_a": {
                "source_stamp_ns": 11740000000,
                "obstacle_sequence": 175,
                "left_generated": 45,
                "right_generated": 45,
                "left_rejections": {"maximum_curvature": 45},
                "right_rejections": {
                    "maximum_lateral_slope": 30,
                    "footprint_track_bound": 9,
                    "maximum_curvature": 6,
                },
                "offline_digest": scenario_by_name["validation_019"]["digest"],
            },
        },
        "classification_counts": {
            label: sum(row["classification"] == label for row in scenario_rows)
            for label in ("E", "X", "EX", "T", "P", "M")
        },
        "scenarios": {},
        "recommendation": {
            "production_change_in_this_audit": None,
            "smallest_next_hypothesis": (
                "Validate actual-available-entry fraction 1.00 only for validation_014; "
                "do not productionize from this offline audit alone."
            ),
            "piecewise_variable_d_justified_by_feasibility": False,
        },
    }
    for row in scenario_rows:
        summary["scenarios"][row["scenario"]] = {
            "classification": row["classification"],
            "dominant_constraint": row["dominant_constraint"],
            "secondary_constraints": [v for v in row["secondary_constraints"].split(";") if v],
            "snapshot": {
                "frame_index": int(row["frame_index"]),
                "logical_stamp_ns": int(row["logical_stamp_ns"]),
                "ego_s_m": float(row["ego_s"]),
                "ego_d_m": float(row["ego_d"]),
                "ego_speed_mps": float(row["ego_speed"]),
                "input_reconstruction": row["input_reconstruction"],
                "source_bag": row["source_bag"],
                "digest": row["digest"],
            },
            "families": families_by_scenario[row["scenario"]],
            "family_c_side_evidence": c_evidence.get(row["scenario"]),
        }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    primary_evaluations = sum(int(row["generated"]) for row in all_family_rows)
    repeat_evaluations = sum(
        int(row["generated"])
        for row in all_family_rows
        if row["scenario"] == raw_performance["determinism_scenario"]
    )
    replay_wall_s = 226.293
    latest_target_rebuild_wall_s = 10.9
    package_test_wall_s = 34.6
    performance = {
        "schema": "path_family_feasibility_audit_performance/1",
        "cma_es_executed": False,
        "production_code_or_parameters_changed": False,
        "offline_geometry": {
            "wall_s": float(raw_performance["total_wall_s"]),
            "cpu_s": float(raw_performance["total_cpu_s"]),
            "primary_candidate_evaluations": primary_evaluations,
            "determinism_repeat_candidate_evaluations": repeat_evaluations,
            "total_candidate_evaluations": primary_evaluations + repeat_evaluations,
            "summed_primary_family_runtime_ms": sum(
                float(row["runtime_ms"]) for row in all_family_rows
            ),
        },
        "determinism": {
            "scenario": raw_performance["determinism_scenario"],
            "first_digest": raw_performance["first_digest"],
            "second_digest": raw_performance["second_digest"],
            "passed": boolean(raw_performance["deterministic"]),
            "comparison_scope": [
                "candidate_count",
                "hard_feasible_count",
                "best_candidate",
                "rejection_histogram",
                "geometry_hash",
            ],
        },
        "snapshot_recovery_replay": {
            "required": boolean(raw_performance["replay_required"]),
            "count": 1,
            "scenarios": ["validation_019"],
            "execution_mode": "deterministic_lockstep",
            "domain_id": 221,
            "simulator_seed": 410020,
            "scan_noise_std_m": 0.01,
            "step_count": 500,
            "simulated_duration_s": 5.0,
            "wall_s_estimated": replay_wall_s,
            "wall_measurement": "difference between first and final ROS log timestamps",
            "cpu_s": None,
            "candidate_yaml": (
                "runs/cmaes_tuning/medium_lockstep_stage1_v2/candidates/validation_baseline.yaml"
            ),
            "scenario_manifest": (
                "runs/cmaes_tuning/medium_lockstep_stage1_v2/artifacts/scenarios/validation/"
                "validation_019/manifest.json"
            ),
            "tuning_config": "tools/cmaes_tuning/config/tuning_config.yaml",
        },
        "build_and_validation": {
            "latest_target_rebuild_wall_s": latest_target_rebuild_wall_s,
            "package_test_wall_s": package_test_wall_s,
            "package_test_summary": {
                "tests": 134,
                "errors": 0,
                "failures": 0,
                "skipped": 12,
            },
            "cpu_s": None,
        },
        "total_observed_wall_s": (
            replay_wall_s
            + float(raw_performance["total_wall_s"])
            + latest_target_rebuild_wall_s
            + package_test_wall_s
        ),
        "total_measured_cpu_s": float(raw_performance["total_cpu_s"]),
        "total_measured_cpu_scope": "offline geometry helper only",
        "measurement_limits": [
            "Replay and build/test CPU usage were not captured.",
            "Total observed wall time excludes reused-snapshot extraction and exploratory builds.",
        ],
    }
    (output_dir / "performance.json").write_text(
        json.dumps(performance, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    render_readme(output_dir, scenario_rows, all_family_rows, performance)


if __name__ == "__main__":
    main()
