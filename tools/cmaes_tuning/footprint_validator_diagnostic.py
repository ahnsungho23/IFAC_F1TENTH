#!/usr/bin/env python3
"""Run and summarize the three footprint-validator diagnostic lockstep episodes."""

from __future__ import annotations

import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import re
import sys
from typing import Any

TOOL_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOL_ROOT))

from cmaes_tuning.bag_reader import read_bag  # noqa: E402
from cmaes_tuning.parallel_diagnostic import (  # noqa: E402
    DiagnosticJob,
    ParallelEpisodeBenchmark,
)
from cmaes_tuning.schemas import atomic_write_json, sha256_file  # noqa: E402


SCENARIOS = ("009", "024", "029")


def paths(workspace: Path) -> dict[str, Path]:
    medium = workspace / "runs/cmaes_tuning/medium_lockstep_stage1_v2"
    return {
        "medium": medium,
        "narrow": workspace / "runs/cmaes_tuning/narrow_candidate_selection_diagnostic_v2",
        "tracking": workspace / "runs/cmaes_tuning/tracking_swept_footprint_analysis_v1",
        "candidate": medium / "candidates/validation_rank_01_gen_003_cand_004.yaml",
        "config": workspace / "tools/cmaes_tuning/config/tuning_config.yaml",
        "validation": medium / "artifacts/scenarios/validation",
    }


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def baseline_directory(source: dict[str, Path], identifier: str) -> Path:
    seed = 410001 + int(identifier)
    return (
        source["narrow"] / "new_planner" / f"validation_{identifier}"
        / f"seed_{seed}" / "attempt_00"
    )


def parse_baseline_audit(directory: Path) -> dict[str, Any]:
    expression = re.compile(
        r"generated=(\d+) feasible=(\d+) rank=(\d+) side=(\w+) target=([-0-9.]+) "
        r"entry_requested/effective=([-0-9.]+)/([-0-9.]+) exit=([-0-9.]+) "
        r"wall/obstacle=([-0-9.]+)/([-0-9.]+) peak_curvature/rate="
        r"([-0-9.]+)/([-0-9.]+) speed_loss=([-0-9.]+) min_slack=([-0-9.]+)"
    )
    log_path = directory / "logs/local_planning.log"
    match = expression.search(log_path.read_text(encoding="utf-8", errors="replace"))
    if match is None:
        return {}
    values = match.groups()
    return {
        "generated_count": int(values[0]),
        "feasible_count": int(values[1]),
        "rank": int(values[2]),
        "side": values[3],
        "target_d_m": float(values[4]),
        "requested_entry_length_m": float(values[5]),
        "effective_entry_length_m": float(values[6]),
        "exit_length_m": float(values[7]),
        "centerline_wall_clearance_m": float(values[8]),
        "obstacle_headroom_m": float(values[9]),
        "peak_curvature_radpm": float(values[10]),
        "peak_curvature_rate_radpm2": float(values[11]),
    }


def load_candidate_group(attempt: Path) -> list[dict[str, Any]]:
    event_path = attempt / "planner_candidate_events.jsonl"
    groups: dict[tuple[Any, ...], dict[int, dict[str, Any]]] = {}
    order: list[tuple[Any, ...]] = []
    if not event_path.is_file():
        return []
    for line in event_path.read_text(encoding="utf-8").splitlines():
        if not line:
            continue
        event = json.loads(line)
        key = (
            event.get("source_stamp_ns"),
            event.get("obstacle_sequence"),
            event.get("decision"),
        )
        if key not in groups:
            groups[key] = {}
            order.append(key)
        groups[key][int(event["generation_index"])] = event
    if not order:
        return []
    # Prefer the first complete decision group. Later groups are commitment revalidation and do
    # not belong in the initial apples-to-apples candidate count.
    maximum_size = max(len(groups[key]) for key in order)
    selected_key = next(key for key in order if len(groups[key]) == maximum_size)
    return [groups[selected_key][index] for index in sorted(groups[selected_key])]


def safe_stop_used(attempt: Path) -> bool:
    bag = read_bag(attempt / "bag", topics=["/avoid_waypoints"])
    return any(
        "safe_stop" in str(record.message.ot_line)
        for record in bag.topic("/avoid_waypoints")
    )


def outcome(episode: dict[str, Any], lockstep: dict[str, Any], attempt: Path) -> dict[str, Any]:
    failure = episode.get("failure", {})
    rollout = read_json(attempt / "rollout_summary.json")
    return {
        "success": bool(lockstep.get("scenario_success")),
        "collision": bool(failure.get("collision")),
        "off_track": bool(failure.get("off_track")),
        "planner_failure": bool(failure.get("planner_failure")),
        "timeout": rollout.get("terminated") == "timeout",
        "safe_stop": safe_stop_used(attempt),
    }


def selected_audit(events: list[dict[str, Any]]) -> dict[str, Any] | None:
    return next((event for event in events if event.get("selected")), None)


def rejection_counts(events: list[dict[str, Any]]) -> Counter[str]:
    return Counter(
        str(event.get("rejection_reason") or "")
        for event in events if not event.get("feasible")
    )


def matching_old_candidate(
    events: list[dict[str, Any]], old_audit: dict[str, Any], old_side: str | None,
) -> dict[str, Any] | None:
    """Recover the generated candidate matching the immutable old selection."""

    if not events or not old_audit or old_side is None:
        return None
    side = "left" if old_side.lower() == "left" else "right"
    same_side = [event for event in events if event.get("side") == side]
    if not same_side:
        return None
    target = float(old_audit["target_d_m"])
    entry = float(old_audit["effective_entry_length_m"])
    exit_length = float(old_audit["exit_length_m"])
    best = min(
        same_side,
        key=lambda event: (
            abs(float(event["target_d_m"]) - target),
            abs(float(event["effective_entry_length_m"]) - entry),
            abs(float(event["exit_length_m"]) - exit_length),
            int(event["generation_index"]),
        ),
    )
    differences = (
        abs(float(best["target_d_m"]) - target),
        abs(float(best["effective_entry_length_m"]) - entry),
        abs(float(best["exit_length_m"]) - exit_length),
    )
    return best if max(differences) <= 1.0e-5 else None


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = list(rows[0]) if rows else []
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def classify(before: dict[str, Any], after: dict[str, Any], feasible: int) -> str:
    if not before["collision"]:
        return "not_applicable"
    if after["success"]:
        return "A_collision_to_success"
    if feasible == 0 or after["safe_stop"]:
        return "B_collision_to_no_feasible_or_safe_stop"
    return "C_collision_to_collision"


def aggregate(
    workspace: Path,
    output: Path,
    stage: dict[str, Any] | None,
) -> dict[str, Any]:
    source = paths(workspace)
    tracking = read_json(source["tracking"] / "summary.json")
    tracking_by_id = {item["scenario_id"]: item for item in tracking["scenarios"]}
    attempts: dict[str, Path] = {}
    for status_path in (output / "performance/diagnostic_replays").glob(
        "worker_*/job_*/attempt_*/runner_status.json"
    ):
        status = read_json(status_path)
        attempt = status_path.parent
        result_path = attempt / "lockstep_result.json"
        if result_path.is_file() and read_json(result_path).get("valid"):
            attempts[status["scenario_id"]] = attempt

    rows: list[dict[str, Any]] = []
    rejection_records: list[dict[str, Any]] = []
    scenario_summaries = []
    for identifier in SCENARIOS:
        scenario_id = f"validation_{identifier}"
        old_attempt = baseline_directory(source, identifier)
        new_attempt = attempts.get(scenario_id)
        if new_attempt is None:
            raise RuntimeError(f"valid new replay missing: {scenario_id}")
        old_episode = read_json(old_attempt / "episode_result.json")
        old_lockstep = read_json(old_attempt / "lockstep_result.json")
        new_episode = read_json(new_attempt / "episode_result.json")
        new_lockstep = read_json(new_attempt / "lockstep_result.json")
        old_audit = parse_baseline_audit(old_attempt)
        events = load_candidate_group(new_attempt)
        selected = selected_audit(events)
        old_candidate = matching_old_candidate(
            events, old_audit, old_lockstep.get("selected_side"))
        counts = rejection_counts(events)
        footprint_rejects = counts.get("footprint_track_bound", 0)
        feasible_count = sum(bool(event.get("feasible")) for event in events)
        old_state = outcome(old_episode, old_lockstep, old_attempt)
        new_state = outcome(new_episode, new_lockstep, new_attempt)
        old_tracking = tracking_by_id[scenario_id]
        old_metrics = old_episode["metrics"]
        new_metrics = new_episode["metrics"]
        classification = classify(old_state, new_state, feasible_count)
        false_safe_removed = bool(
            old_candidate is not None
            and not old_candidate.get("feasible")
            and old_candidate.get("rejection_reason") == "footprint_track_bound")
        scenario = {
            "scenario_id": scenario_id,
            "seed": 410001 + int(identifier),
            "classification": classification,
            "old": {
                **old_state,
                "generated_candidate_count": old_audit.get("generated_count"),
                "feasible_candidate_count": old_audit.get("feasible_count"),
                "footprint_track_bound_reject_count": 0,
                "selected_side": old_lockstep.get("selected_side"),
                "selected_target_d_m": old_lockstep.get("committed_target_d"),
                "selected_entry_length_m": old_audit.get("effective_entry_length_m"),
                "selected_exit_length_m": old_audit.get("exit_length_m"),
                "minimum_centerline_wall_clearance_m":
                    old_audit.get("centerline_wall_clearance_m"),
                "minimum_planned_footprint_wall_clearance_m":
                    old_tracking["minimums_m"]["planned_polygon_raster"],
                "actual_swept_footprint_clearance_m":
                    old_metrics["minimum_wall_clearance_m"],
                "minimum_obstacle_clearance_m":
                    old_metrics["minimum_obstacle_clearance_m"],
                "peak_curvature_radpm": old_metrics["planned_path_max_curvature_radpm"],
                "curvature_rate_rms_radpm2":
                    old_metrics["planned_curvature_rate_rms_radpm2"],
            },
            "new": {
                **new_state,
                "generated_candidate_count": len(events),
                "feasible_candidate_count": feasible_count,
                "footprint_track_bound_reject_count": footprint_rejects,
                "rejection_reason_counts": dict(sorted(counts.items())),
                "selected_side": new_lockstep.get("selected_side"),
                "selected_target_d_m": new_lockstep.get("committed_target_d"),
                "selected_entry_length_m": (
                    selected.get("effective_entry_length_m") if selected else None),
                "selected_exit_length_m": selected.get("exit_length_m") if selected else None,
                "minimum_centerline_wall_clearance_m": (
                    selected.get("minimum_centerline_wall_clearance_m") if selected else None),
                "minimum_planned_footprint_wall_clearance_m": (
                    selected.get("minimum_rectangular_footprint_wall_clearance_m")
                    if selected else None),
                "actual_swept_footprint_clearance_m":
                    new_metrics["minimum_wall_clearance_m"],
                "minimum_obstacle_clearance_m":
                    new_metrics["minimum_obstacle_clearance_m"],
                "peak_curvature_radpm": new_metrics["planned_path_max_curvature_radpm"],
                "curvature_rate_rms_radpm2":
                    new_metrics["planned_curvature_rate_rms_radpm2"],
            },
            "false_safe_candidate_removed": false_safe_removed,
            "old_selected_candidate_replay_audit": old_candidate,
            "new_attempt": str(new_attempt),
        }
        scenario_summaries.append(scenario)
        flat = {
            "scenario_id": scenario_id,
            "classification": classification,
            "false_safe_candidate_removed": false_safe_removed,
        }
        for prefix, values in (("old", scenario["old"]), ("new", scenario["new"])):
            for key, value in values.items():
                if key != "rejection_reason_counts":
                    flat[f"{prefix}_{key}"] = value
        flat["old_selected_candidate_rejection_reason"] = (
            old_candidate.get("rejection_reason") if old_candidate else None)
        rows.append(flat)
        for event in events:
            if event.get("feasible"):
                continue
            rejection_records.append({
                "scenario_id": scenario_id,
                "simulator_seed": scenario["seed"],
                **event,
            })

    output.mkdir(parents=True, exist_ok=True)
    write_csv(output / "scenario_comparison.csv", rows)
    (output / "candidate_rejections.jsonl").write_text(
        "".join(
            json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n"
            for record in rejection_records
        ),
        encoding="utf-8",
    )
    previous_performance = read_json(source["tracking"] / "performance.json")
    performance = {
        "schema": "footprint_validator_performance/1",
        "benchmark_reused": True,
        "benchmark_source": str(source["tracking"] / "performance.json"),
        "benchmark_source_sha256": sha256_file(source["tracking"] / "performance.json"),
        "validated_max_workers": previous_performance["selected_workers"],
        "validated_speedup": previous_performance["actual_speedup"],
        "actual_worker_count": len(SCENARIOS),
        "actual_stage": stage,
        "episode_internal_lockstep_parallelized": False,
        "cma_executed": False,
    }
    atomic_write_json(output / "performance.json", performance)
    summary = {
        "schema": "footprint_validator_diagnostic/1",
        "source_policy": "immutable_reference_artifacts_plus_three_lockstep_replays",
        "physical_footprint": {
            "length_m": 0.56,
            "width_m": 0.287,
            "reference_point": "base_link_centered",
            "wall_safety_margin_changed": False,
            "simulator_ttc_or_noise_guard_included": False,
        },
        "scenario_count": len(scenario_summaries),
        "scenarios": scenario_summaries,
        "false_safe_removed_count": sum(
            bool(item["false_safe_candidate_removed"]) for item in scenario_summaries),
        "classification_counts": dict(sorted(Counter(
            item["classification"] for item in scenario_summaries).items())),
        "candidate_rejection_record_count": len(rejection_records),
        "cma_executed": False,
        "path_family_changed": False,
        "candidate_ranking_changed": False,
    }
    atomic_write_json(output / "summary.json", summary)
    write_readme(output, summary, performance)
    return summary


def metric(value: Any, digits: int = 4) -> str:
    return "N/A" if value is None else f"{float(value):.{digits}f}"


def state_label(values: dict[str, Any]) -> str:
    labels = [name for name in ("collision", "off_track", "planner_failure", "timeout")
              if values.get(name)]
    if values.get("success"):
        return "success"
    if values.get("safe_stop"):
        labels.append("safe_stop")
    return "+".join(labels) if labels else "incomplete"


def write_readme(output: Path, summary: dict[str, Any], performance: dict[str, Any]) -> None:
    rows = []
    causes = []
    for item in summary["scenarios"]:
        old = item["old"]
        new = item["new"]
        rows.append(
            f"| {item['scenario_id']} | {state_label(old)} | {state_label(new)} | "
            f"{new['feasible_candidate_count']}/{new['generated_candidate_count']} | "
            f"{new['footprint_track_bound_reject_count']} | "
            f"{new['selected_side'] or 'N/A'} / "
            f"{metric(new['selected_target_d_m'])} | "
            f"{metric(old['minimum_planned_footprint_wall_clearance_m'])} -> "
            f"{metric(new['minimum_planned_footprint_wall_clearance_m'])} | "
            f"{metric(old['actual_swept_footprint_clearance_m'])} -> "
            f"{metric(new['actual_swept_footprint_clearance_m'])} | "
            f"{item['classification']} |"
        )
        if item["classification"].startswith("B_"):
            counts = new["rejection_reason_counts"]
            causes.append(
                f"- `{item['scenario_id']}`: false-safe 후보는 제거됐지만 대체 moving path가 "
                f"없어 safe-stop/no-feasible이 됐다. rejection counts={counts}.")
        elif item["classification"].startswith("A_"):
            causes.append(
                f"- `{item['scenario_id']}`: selected footprint headroom "
                f"{metric(new['minimum_planned_footprint_wall_clearance_m'])} m인 대체 후보가 "
                "실제 collision 없이 scenario를 완료했다.")
        else:
            causes.append(
                f"- `{item['scenario_id']}`: footprint hard gate 뒤에도 collision이 남아 "
                "validator 단독 해결에 실패했다.")

    stage = performance.get("actual_stage") or {}
    lines = [
        "# Footprint-aware track-bound validator diagnostic v1",
        "",
        "## 변경 범위",
        "",
        "CMA, path family, target sampling, candidate ranking, controller, perception, tracking "
        "LUT와 기존 parameter 값은 변경하지 않았다. `d_left/d_right`를 reference에서 물리 "
        "track boundary까지의 거리로 사용하고, candidate `x/y/yaw`의 0.56 x 0.287 m "
        "base_link-centred rectangle 네 corner를 동일 branch의 local reference segment에 "
        "투영해 보간 boundary와 비교한다. "
        "`wall_safety_margin_m`은 기존 값 그대로 정확히 한 번 적용하며, 위반 후보는 "
        "`footprint_track_bound`로 hard reject한다.",
        "",
        "기존 centerline headroom은 candidate ranker 호환 metric으로 유지했다. 따라서 "
        "enumeration/ranking 의미는 바뀌지 않았고 footprint 검사는 추가 hard gate다.",
        "",
        "## Physical footprint와 simulator envelope 분리",
        "",
        "Production validation은 물리 footprint와 기존 wall margin만 사용한다. 약 20 mm TTC "
        "sweep 및 7.5 mm scan-noise guard는 evaluator diagnostic에만 남기며 vehicle length/width나 "
        "production wall margin에 합치지 않았다.",
        "",
        "## 009/024/029 apples-to-apples replay",
        "",
        "| scenario | 기존 | 신규 | feasible/generated | footprint reject | selected side/d | "
        "planned footprint m | actual swept m | 판정 |",
        "|---|---|---|---:|---:|---|---:|---:|---|",
        *rows,
        "",
        *causes,
        "",
        f"False-safe accepted candidate removed: "
        f"{summary['false_safe_removed_count']}/{summary['scenario_count']}.",
        "",
        "상세 entry/exit, centerline/footprint clearance, obstacle clearance, peak curvature/rate, "
        "safe-stop 여부는 `scenario_comparison.csv`에 있고 모든 초기 hard rejection은 "
        "`candidate_rejections.jsonl`에 있다.",
        "",
        "## Waypoint sampling",
        "",
        "기존 swept analysis에서 세 경로의 waypoint 및 10 ms interpolation loss가 모두 0 m였기 "
        "때문에 path를 변경하는 continuous solver나 새 sample-spacing parameter를 추가하지 "
        "않았다. 기존 ordered global waypoint pose 전부를 deterministic하게 검사한다.",
        "",
        "## Performance",
        "",
        "기존 8-worker exact determinism benchmark를 재사용했고 benchmark 자체는 반복하지 않았다.",
        f"이번에는 scenario가 3개이므로 worker를 3개로 자동 제한했다. valid="
        f"{stage.get('valid_episode_count', 'N/A')}/3, wall-clock="
        f"{metric(stage.get('wall_clock_sec'), 2)} s, mismatch="
        f"{stage.get('deterministic_mismatch_count', 'N/A')}, infrastructure failure="
        f"{stage.get('infrastructure_failure_count', 'N/A')}, DDS collision="
        f"{stage.get('dds_collision_count', 'N/A')}, process contamination="
        f"{stage.get('process_contamination_count', 'N/A')}.",
        "",
        "## 결론",
        "",
        "이번 1차 성공 기준은 3/3 success가 아니라 기존과 동일한 side/target/entry/exit "
        "false-safe candidate가 `footprint_track_bound`로 제거되는지와 predicted physical "
        "validity/actual outcome 정합이다. B가 남으면 validator는 올바르게 보수화됐지만 현재 "
        "path representation에 대체 경로가 없는 것이므로 다음 단계는 **path-family 확장 필요**다. "
        "C가 남으면 **validator 추가 수정 필요**, 두 유형이 함께 있으면 **둘 다 필요**다.",
        "",
    ]
    (output / "README.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", default=str(TOOL_ROOT.parents[1]))
    parser.add_argument(
        "--output", default="runs/cmaes_tuning/footprint_validator_diagnostic_v1")
    parser.add_argument("--aggregate-only", action="store_true")
    parser.add_argument("--domain-start", type=int, default=210)
    args = parser.parse_args()
    workspace = Path(args.workspace).resolve()
    output = (workspace / args.output).resolve()
    source = paths(workspace)
    stage = None
    if not args.aggregate_only:
        jobs = [
            DiagnosticJob(
                index=index,
                scenario_id=f"validation_{identifier}",
                scenario_manifest=(
                    source["validation"] / f"validation_{identifier}/manifest.json"),
                candidate=source["candidate"],
                simulator_seed=410001 + int(identifier),
            )
            for index, identifier in enumerate(SCENARIOS)
        ]
        benchmark = ParallelEpisodeBenchmark(
            workspace=workspace,
            config=source["config"],
            output=output,
            domain_start=args.domain_start,
            scan_noise_std=0.01,
            maximum_duration=65.0,
            post_obstacle_distance=3.0,
            retries=1,
        )
        stage, results = benchmark.run_stage(
            "diagnostic_replays", min(8, len(jobs)), jobs)
        if len(results) != len(jobs):
            raise RuntimeError(
                f"only {len(results)}/{len(jobs)} diagnostic replays were valid")
    elif (output / "performance/diagnostic_replays/stage_summary.json").is_file():
        stage = read_json(output / "performance/diagnostic_replays/stage_summary.json")
    summary = aggregate(workspace, output, stage)
    print(json.dumps({
        "output": str(output),
        "classification_counts": summary["classification_counts"],
        "false_safe_removed_count": summary["false_safe_removed_count"],
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
