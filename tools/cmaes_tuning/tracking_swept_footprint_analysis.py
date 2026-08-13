#!/usr/bin/env python3
"""Analyze tracking/swept-footprint loss and benchmark episode-level parallelism."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

TOOL_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOL_ROOT))

from cmaes_tuning.configuration import load_config  # noqa: E402
from cmaes_tuning.parallel_diagnostic import ParallelEpisodeBenchmark  # noqa: E402
from cmaes_tuning.schemas import sha256_file  # noqa: E402
from cmaes_tuning.tracking_swept_analysis import aggregate_analysis  # noqa: E402


SCENARIOS = ("009", "024", "029")


def _source_paths(workspace: Path) -> list[tuple[Path, Path]]:
    medium = workspace / "runs/cmaes_tuning/medium_lockstep_stage1_v2"
    diagnostic = workspace / "runs/cmaes_tuning/narrow_candidate_selection_diagnostic_v2"
    paths = []
    for identifier in SCENARIOS:
        seed = 410001 + int(identifier)
        episode = (
            diagnostic / "new_planner" / f"validation_{identifier}"
            / f"seed_{seed}" / "attempt_00")
        manifest = (
            medium / "artifacts/scenarios/validation" / f"validation_{identifier}"
            / "manifest.json")
        if not (episode / "bag/metadata.yaml").is_file():
            raise FileNotFoundError(f"source bag missing: {episode}")
        if not manifest.is_file():
            raise FileNotFoundError(f"source manifest missing: {manifest}")
        paths.append((episode.resolve(), manifest.resolve()))
    return paths


def _number(value: Any, digits: int = 3) -> str:
    if value is None:
        return "N/A"
    return f"{float(value):.{digits}f}"


def _millimetres(value: Any) -> str:
    if value is None:
        return "N/A"
    return f"{1000.0 * float(value):.2f}"


def _cause_description(code: str) -> str:
    return {
        "A": "planner centerline 자체가 벽에 가까움",
        "B": "centerline 대비 rectangular footprint 적용 손실",
        "C": "lateral tracking position error",
        "D": "heading error에 따른 corner 돌출",
        "E": "waypoint/temporal interpolation sampling",
        "F": "raster 보수성 또는 TTC swept/noise envelope",
        "G": "controller steering transient/actuator response",
        "H": "기타",
    }.get(code, "기타")


def _performance_table(performance: dict[str, Any] | None) -> list[str]:
    if not performance:
        return ["Performance benchmark가 아직 실행되지 않았습니다."]
    lines = [
        "| workers | wall-clock s | episodes/min | mean CPU % | peak process RAM GiB | "
        "peak system used GiB | mismatch | DDS/process contamination | swap increase | "
        "infra failure/retry |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for stage in performance.get("benchmarks", []):
        lines.append(
            f"| {stage['workers']} | {stage['wall_clock_sec']:.2f} | "
            f"{stage['episodes_per_min']:.2f} | "
            f"{stage['average_cpu_utilization_percent']:.1f} | "
            f"{stage['peak_process_rss_bytes'] / (1024 ** 3):.2f} | "
            f"{stage['peak_system_used_bytes'] / (1024 ** 3):.2f} | "
            f"{stage['deterministic_mismatch_count']} | "
            f"{stage['dds_collision_count']}/"
            f"{stage['process_contamination_count']} | "
            f"{'yes' if stage['swap_increase'] else 'no'} | "
            f"{stage['infrastructure_failure_count']}/"
            f"{stage['infrastructure_retry_count']} |")
    return lines


def write_readme(
    output: Path, summary: dict[str, Any], performance: dict[str, Any] | None,
    source_paths: list[tuple[Path, Path]],
) -> None:
    rows = []
    effect_rows = []
    for scenario in summary["scenarios"]:
        critical = scenario["critical_decomposition_m"]
        collision_raster = scenario["collision_raster"]
        rows.append(
            f"| {scenario['scenario_id']} | "
            f"{_millimetres(critical['planner_predicted_wall_headroom'])} | "
            f"{_millimetres(scenario['minimums_m']['planned_polygon_raster'])} | "
            f"{_millimetres(scenario['minimums_m']['actual_polygon_temporally_dense'])} | "
            f"{_millimetres(scenario['minimums_m']['actual_evaluator'])} | "
            f"{_millimetres(scenario['tracking_error_m']['max'])}/"
            f"{_millimetres(scenario['tracking_error_m']['p95'])}/"
            f"{_millimetres(scenario['tracking_error_m']['p99'])} | "
            f"{_number(scenario['heading_error_rad']['max'])}/"
            f"{_number(scenario['heading_error_rad']['p95'])} | "
            f"{scenario['dominant_failure_cause']} "
            f"({_cause_description(scenario['dominant_failure_cause'])}) |")
        effect_rows.append(
            f"| {scenario['scenario_id']} | "
            f"{_millimetres(critical['wallward_lateral_error'])} | "
            f"{_millimetres(critical['heading_corner_protrusion'])}/"
            f"{_millimetres(scenario['heading_corner_protrusion_m']['max'])} | "
            f"{_millimetres(scenario['maximum_waypoint_sampling_loss_m'])}/"
            f"{_millimetres(scenario['actual_temporal_sampling_loss_m'])} | "
            f"{_millimetres(collision_raster.get('ttc_sweep_m'))}/"
            f"{_millimetres(collision_raster.get('scan_noise_guard_m'))} | "
            f"{collision_raster.get('first_collision_base_raster_overlap')} |")

    lateral = summary["aggregate_tracking_error_m"]
    swept = summary["aggregate_swept_clearance_loss_m"]
    # Three collision-only rollouts do not establish a safe production quantile. Even when the
    # scalar reserve exceeds observed e_lat, its placement in the validator is not equivalent to
    # the measured swept loss, so the production classification must remain unresolved.
    reserve_assessment = (
        "판단 불가. 0.14 m는 관측된 centerline lateral error 최대값보다 수치상 크지만, "
        "collision-only 3개와 하나의 속도 조건뿐이며 reserve가 적용되는 planner Frenet "
        "track-width convention과 raster swept-footprint loss가 동치가 아니다.")

    performance_lines = _performance_table(performance)
    selected_workers = performance.get("selected_workers") if performance else "N/A"
    speedup = performance.get("actual_speedup") if performance else None
    gate = performance.get("determinism_gate", {}) if performance else {}
    workers_two = gate.get("workers_2", {})
    exact_comparisons = sum(
        len(stage.get("deterministic_comparisons", []))
        for stage in performance.get("benchmarks", [])
    ) if performance else 0
    total_mismatches = sum(
        stage.get("deterministic_mismatch_count", 0)
        for stage in performance.get("benchmarks", [])
    ) if performance else "N/A"
    valid_infrastructure_failures = sum(
        stage.get("infrastructure_failure_count", 0)
        for stage in performance.get("benchmarks", [])
    ) if performance else "N/A"
    valid_infrastructure_retries = sum(
        stage.get("infrastructure_retry_count", 0)
        for stage in performance.get("benchmarks", [])
    ) if performance else "N/A"
    excluded_failures = (
        performance.get("excluded_infrastructure_failure_attempt_count", "N/A")
        if performance else "N/A")
    excluded_retries = (
        performance.get("excluded_infrastructure_retry_count", "N/A")
        if performance else "N/A")
    excluded_dds_markers = (
        performance.get("excluded_dds_error_marker_count", "N/A")
        if performance else "N/A")
    correlations = summary["aggregate_correlations"]

    def correlation(name: str) -> str:
        return _number(correlations[name].get("coefficient"), 3)
    domain_preflight = performance.get("domain_preflight", {}) if performance else {}
    source_lines = []
    for episode, manifest in source_paths:
        source_lines.append(
            f"- `{episode}` (`episode_result.json` sha256 "
            f"`{sha256_file(episode / 'episode_result.json')}`), manifest "
            f"`{sha256_file(manifest)}`")

    lines = [
        "# Tracking swept-footprint analysis v1",
        "",
        "## Scope and immutable inputs",
        "",
        "기존 lockstep bag만 읽어 0.56 m × 0.287 m, base_link-centred footprint로 분석했다. "
        "CMA, planner/controller/perception/physics 알고리즘 및 production parameter는 변경하지 "
        "않았다. no-feasible 004/014/019/034/039는 수정하거나 재실행하지 않았다.",
        "",
        *source_lines,
        "",
        "동일 최신 diagnostic에는 비충돌 control rollout이 없으므로 별도 control을 만들지 "
        "않았다. 아래 percentile은 production margin이 아니라 세 collision rollout의 diagnostic "
        "estimate다.",
        "",
        "## Clearance decomposition",
        "",
        "| scenario | planner headroom mm | planned polygon min mm | "
        "actual dense polygon min mm | evaluator min mm | |e_lat| max/p95/p99 mm | "
        "|heading| max/p95 rad | dominant |",
        "|---|---:|---:|---:|---:|---:|---:|---|",
        *rows,
        "",
        "세 scenario 모두 critical local-path centerline clearance는 159.10~172.72 mm였지만 "
        "동일 pose에 rectangular footprint를 놓으면 0 mm였다. planner-side Frenet headroom "
        "14.40~50.96 mm와 evaluator clearance의 차이는 각각 50.96, 47.01, 7.08 mm다. "
        "따라서 dominant cause는 모두 B다.",
        "",
        "| scenario | critical wallward e_lat mm | corner protrusion critical/max mm | "
        "waypoint/10ms sampling loss mm | TTC sweep/noise guard mm | "
        "base footprint overlap at first TTC collision |",
        "|---|---:|---:|---:|---:|---|",
        *effect_rows,
        "",
        "음의 wallward error는 선택한 avoidance wall에서 멀어지는 방향이다. 세 critical "
        "sample 모두 음수여서 tracking position error가 collision 순간의 지배 원인은 아니었다. "
        "heading corner 돌출은 critical 3.22/7.59/9.38 mm, scenario 최대 "
        "12.43/14.73/22.82 mm였다. waypoint 및 10 ms temporal interpolation loss는 모두 "
        "0 mm였다. validation_009/024는 종료 시 base footprint도 벽과 겹쳤고, "
        "validation_029는 base footprint clearance가 남은 상태에서 TTC swept/noise envelope가 "
        "collision을 앞당겼다.",
        "",
        f"Aggregate `|e_lat|`: max={_millimetres(lateral['max'])} mm, "
        f"p95={_millimetres(lateral['p95'])} mm, p99={_millimetres(lateral['p99'])} mm.",
        "",
        f"Aggregate `e_swept = planned polygon clearance - actual polygon clearance`: "
        f"max={_millimetres(swept['max'])} mm, p95={_millimetres(swept['p95'])} mm, "
        f"p99={_millimetres(swept['p99'])} mm.",
        "",
        "Aggregate Pearson correlation (n=517): "
        f"`|e_lat|-speed`={correlation('abs_lateral_error_vs_speed')}, "
        f"`|e_lat|-|curvature|`="
        f"{correlation('abs_lateral_error_vs_abs_curvature')}, "
        f"`|e_lat|-|curvature-rate|`="
        f"{correlation('abs_lateral_error_vs_abs_curvature_rate')}, "
        f"`|heading|-|curvature|`="
        f"{correlation('abs_heading_error_vs_abs_curvature')}, "
        f"`e_swept-|curvature|`="
        f"{correlation('swept_loss_vs_abs_curvature')}, "
        f"`e_swept-|steering|`="
        f"{correlation('swept_loss_vs_abs_steering')}, "
        f"`e_swept-|steering-rate|`="
        f"{correlation('swept_loss_vs_abs_steering_rate')}. "
        "속도가 사실상 4.0 m/s로 고정되고 collision rollout 3개뿐이므로 speed×curvature "
        "LUT를 식별할 근거는 없다.",
        "",
        "각 scenario의 critical timestep, footprint rotation, position/heading loss, waypoint와 "
        "10 ms temporal interpolation loss, TTC sweep와 scan-noise guard는 "
        "`clearance_decomposition.csv`와 `summary.json`에 분리 저장했다. CSV에서 "
        "`is_minimum_clearance_sample=true` 및 `is_pre_collision_0p5s=true`로 critical 구간을 "
        "표시했다.",
        "",
        "## Fixed 0.14 m tracking reserve",
        "",
        reserve_assessment,
        "",
        "## Performance",
        "",
        "- CPU: Intel Core i7-14650HX, 16C/24T",
        f"- workers=2 determinism gate: "
        f"{'PASS' if performance and performance['determinism_gate_accepted'] else 'FAIL/미실행'}",
        f"- gate deterministic mismatch: "
        f"{workers_two.get('deterministic_mismatch_count', 'N/A')}",
        f"- benchmark exact hash/metric comparisons: {exact_comparisons} accepted, "
        f"{total_mismatches} mismatch",
        f"- 최종 선택 worker 수: {selected_workers}",
        f"- 실제 speedup: {_number(speedup, 2)}x",
        f"- 선택 이유: {performance.get('selection_reason', 'N/A') if performance else 'N/A'}",
        f"- 실행 직전 domain check: parent ROS_DOMAIN_ID="
        f"{domain_preflight.get('parent_ros_domain_id', 'unset') or 'unset'}, "
        f"explicit 210~217 conflicts="
        f"{len(domain_preflight.get('explicit_process_conflicts', []))}",
        f"- 유효 benchmark infrastructure failure/retry: "
        f"{valid_infrastructure_failures}/{valid_infrastructure_retries}",
        f"- 제외된 preflight infrastructure failure/retry: "
        f"{excluded_failures}/{excluded_retries} "
        f"(sandbox DDS socket error markers="
        f"{excluded_dds_markers})",
        "",
        *performance_lines,
        "",
        "각 worker는 ROS_DOMAIN_ID 210~217, output/tmp/cache/ROS_HOME, simulator process, "
        "process group과 log를 분리했다. 한 episode 내부 lockstep 순서는 변경하지 않았다. "
        "infrastructure 실패 attempt는 과학 결과 집계에서 제외한다.",
        "",
        "## Recommendation",
        "",
        "accepted-but-collision의 지배 원인은 위 scenario별 분해를 따른다. 현 데이터만으로 "
        "tracking LUT나 wall margin을 확정하지 않는다. 다음 단계는 "
        "**tracking-envelope/validator 수정 먼저**이며, no-feasible narrow 사례에는 별도로 "
        "**path-family 확장도 필요**하다. 즉 전체 validation 관점에서는 둘 다 필요하지만 "
        "이번 009/024/029의 우선순위는 swept-footprint-aware validator 정합이다.",
        "",
    ]
    (output / "README.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", default=str(TOOL_ROOT.parents[1]))
    parser.add_argument(
        "--config", default=str(TOOL_ROOT / "config" / "tuning_config.yaml"))
    parser.add_argument(
        "--output", default="runs/cmaes_tuning/tracking_swept_footprint_analysis_v1")
    parser.add_argument("--skip-performance", action="store_true")
    parser.add_argument("--performance-only", action="store_true")
    parser.add_argument("--benchmark-jobs", type=int, default=8)
    parser.add_argument("--domain-start", type=int, default=210)
    args = parser.parse_args()

    workspace = Path(args.workspace).resolve()
    output = (workspace / args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    config_path = Path(args.config).resolve()
    config = load_config(config_path)
    sources = _source_paths(workspace)
    medium = workspace / "runs/cmaes_tuning/medium_lockstep_stage1_v2"
    candidate = medium / "candidates/validation_rank_01_gen_003_cand_004.yaml"
    performance = None
    if not args.skip_performance:
        performance = ParallelEpisodeBenchmark(
            workspace, config_path, output, domain_start=args.domain_start,
            scan_noise_std=0.01, maximum_duration=0.50,
            post_obstacle_distance=3.0, retries=1).run(
                candidate, sources[0][1], 410010,
                benchmark_job_count=args.benchmark_jobs)
    elif (output / "performance.json").is_file():
        performance = json.loads((output / "performance.json").read_text(encoding="utf-8"))

    if args.performance_only:
        print(json.dumps(performance, indent=2, sort_keys=True))
        return 0
    summary = aggregate_analysis(
        sources, output,
        float(config["evaluation"]["footprint_sample_step_m"]),
        dict(config["evaluation"]["simulator_collision"]))
    write_readme(output, summary, performance, sources)
    print(json.dumps({
        "output": str(output),
        "scenario_count": summary["scenario_count"],
        "sample_count": summary["sample_count"],
        "selected_workers": performance.get("selected_workers") if performance else None,
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
