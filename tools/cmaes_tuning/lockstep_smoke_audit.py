#!/usr/bin/env python3
"""Execute the 2-scenario x 10-repeat deterministic lockstep acceptance smoke test."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

TOOL_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOL_ROOT))

from cmaes_tuning.lockstep import (  # noqa: E402
    common_random_number_schedule,
    summarize_lockstep_runs,
)
from cmaes_tuning.configuration import load_config  # noqa: E402
from cmaes_tuning.schemas import atomic_write_json, sha256_file  # noqa: E402


def enrich_safety_result(result: dict, result_path: Path) -> dict:
    """Backfill safety fields for resumable episodes written by schema v1."""
    episode_path = result_path.parent / "episode_result.json"
    episode = (
        json.loads(episode_path.read_text(encoding="utf-8"))
        if episode_path.is_file() else {})
    failure = episode.get("failure", {})
    result["off_track"] = bool(failure.get("off_track", result.get("off_track", False)))
    result["planner_failure"] = bool(
        failure.get("planner_failure", result.get("planner_failure", False)))
    result["scenario_success"] = bool(
        result.get("completed") and not result.get("collision") and
        not result["off_track"] and not result["planner_failure"])
    atomic_write_json(result_path, result)
    return result


def run() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", default=str(TOOL_ROOT.parents[1]))
    parser.add_argument(
        "--config", default=str(TOOL_ROOT / "config" / "tuning_config.yaml"))
    parser.add_argument(
        "--candidate",
        default="runs/cmaes_tuning/e2e_timing_10hz_v1/candidates/baseline.yaml")
    parser.add_argument(
        "--scenario-root",
        default="runs/cmaes_tuning/e2e_timing_10hz_v1/scenarios/training")
    parser.add_argument(
        "--output", default="runs/cmaes_tuning/lockstep_smoke_v1")
    parser.add_argument("--repetitions", type=int, default=10)
    parser.add_argument("--simulator-seed", type=int, default=12345)
    parser.add_argument("--scan-noise-std", type=float, default=0.01)
    parser.add_argument("--post-obstacle-distance", type=float, default=3.0)
    parser.add_argument("--domain-start", type=int, default=171)
    parser.add_argument("--dds-transport", default="UDPv4")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()

    workspace = Path(args.workspace).resolve()
    config = load_config(Path(args.config).resolve())
    output = (workspace / args.output).resolve()
    candidate = (workspace / args.candidate).resolve()
    scenario_root = (workspace / args.scenario_root).resolve()
    output.mkdir(parents=True, exist_ok=True)
    previous_report_path = output / "smoke_audit.json"
    previous_report = (
        json.loads(previous_report_path.read_text(encoding="utf-8"))
        if args.resume and previous_report_path.is_file() else None)
    (output / "artifacts").mkdir(exist_ok=True)
    shutil.copy2(candidate, output / "artifacts" / "baseline.yaml")
    shutil.copy2(Path(args.config), output / "artifacts" / "tuning_config.yaml")

    scenarios = {
        scenario: scenario_root / scenario / "manifest.json"
        for scenario in ("training_001", "training_009")
    }
    for scenario, path in scenarios.items():
        target = output / "artifacts" / f"{scenario}_manifest.json"
        shutil.copy2(path, target)

    revision = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=workspace, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False).stdout.strip()
    source_tree_dirty = bool(subprocess.run(
        ["git", "status", "--porcelain"], cwd=workspace, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False).stdout.strip())
    source_paths = {
        "lockstep_episode": TOOL_ROOT / "lockstep_episode.py",
        "lockstep_helpers": TOOL_ROOT / "cmaes_tuning" / "lockstep.py",
        "detector_wrapper": workspace / "src/obstacle_detector/src/obstacle_detector_node.cpp",
        "planner_wrapper": workspace / "src/local_planning/src/local_planner_node.cpp",
        "state_wrapper": workspace / "src/state_machine/src/state_machine_node.cpp",
        "controller_wrapper": workspace / "src/f1tenth_control/control_code/control_map_node.cpp",
        "simulator_config": Path(config["paths"]["simulator_config"]),
        "simulator_backend": Path(config["paths"]["simulator_collision_source"]),
    }
    experiment_manifest = {
        "schema": "cma_lockstep_experiment/1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "execution_mode": "deterministic_lockstep",
        "localization_mode": "ground_truth",
        "obstacle_input": "simulator_lidar_only",
        "obstacle_ground_truth_available_to_planner": False,
        "scenario_ids": list(scenarios),
        "simulator_seed": args.simulator_seed,
        "scan_noise_std_m": args.scan_noise_std,
        "physics_timestep_sec": float(config["lockstep"]["timestep_sec"]),
        "repetitions": args.repetitions,
        "episode_scope": {
            "kind": "obstacle_encounter_window",
            "start": "scenario_spawn",
            "end": "obstacle_s_plus_recovery_distance",
            "post_obstacle_distance_m": args.post_obstacle_distance,
        },
        "fixed_controller_configuration": config["controller"],
        "candidate": str(candidate),
        "candidate_sha256": sha256_file(candidate),
        "scenario_manifest_sha256": {
            name: sha256_file(path) for name, path in scenarios.items()
        },
        "source_revision": revision,
        "source_tree_dirty": source_tree_dirty,
        "source_sha256": {
            name: sha256_file(path) for name, path in source_paths.items() if path.is_file()
        },
        "common_random_numbers_contract": {
            "key": ["scenario_id", "simulator_seed"],
            "candidate_sharing": "all candidates receive the same ordered key set",
            "example_schedule": common_random_number_schedule(
                ["candidate_A", "candidate_B"], list(scenarios), [101, 202]),
        },
        "production_defaults_changed": False,
        "medium_cma_executed": False,
    }
    atomic_write_json(output / "experiment_manifest.json", experiment_manifest)

    all_results = {}
    started = time.monotonic()
    job_index = 0
    for scenario, manifest in scenarios.items():
        results = []
        for repetition in range(args.repetitions):
            episode_dir = output / "episodes" / scenario / f"repeat_{repetition:02d}"
            result_path = episode_dir / "lockstep_result.json"
            if args.resume and result_path.is_file():
                result = json.loads(result_path.read_text(encoding="utf-8"))
                if result.get("valid"):
                    result = enrich_safety_result(result, result_path)
                    results.append(result)
                    job_index += 1
                    continue
            environment = dict(os.environ)
            environment["ROS_DOMAIN_ID"] = str(args.domain_start + job_index)
            command = [
                sys.executable, str(TOOL_ROOT / "lockstep_episode.py"),
                "--workspace", str(workspace), "--config", str(Path(args.config).resolve()),
                "--candidate", str(candidate), "--scenario", str(manifest),
                "--output", str(episode_dir),
                "--simulator-seed", str(args.simulator_seed),
                "--scan-noise-std", str(args.scan_noise_std),
                "--post-obstacle-distance", str(args.post_obstacle_distance),
                "--dds-transport", args.dds_transport,
            ]
            print(
                f"[{job_index + 1}/{len(scenarios) * args.repetitions}] "
                f"{scenario} repeat_{repetition:02d}", flush=True)
            completed = subprocess.run(
                command, cwd=workspace, env=environment, text=True,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False)
            (episode_dir / "audit_stdout.log").write_text(
                completed.stdout, encoding="utf-8")
            if completed.returncode != 0 or not result_path.is_file():
                raise RuntimeError(
                    f"lockstep episode failed ({scenario}/{repetition}, rc={completed.returncode}); "
                    f"see {episode_dir / 'audit_stdout.log'}")
            result = json.loads(result_path.read_text(encoding="utf-8"))
            result = enrich_safety_result(result, result_path)
            results.append(result)
            job_index += 1
        all_results[scenario] = {
            "summary": summarize_lockstep_runs(results),
            "runs": results,
        }

    async_comparison_path = workspace / (
        "runs/cmaes_tuning/deterministic_replay_v1/live_closed_loop_comparison.json")
    async_comparison = (
        json.loads(async_comparison_path.read_text(encoding="utf-8"))
        if async_comparison_path.is_file() else None)
    accepted = all(item["summary"]["accepted"] for item in all_results.values())
    refresh_wall_sec = time.monotonic() - started
    episode_execution_wall_sec = (
        float(previous_report.get("episode_execution_wall_sec",
                                  previous_report.get("elapsed_wall_sec")))
        if previous_report is not None else refresh_wall_sec)
    report = {
        "schema": "cma_lockstep_smoke_audit/1",
        "accepted": accepted,
        "medium_cma_recommendation": "start" if accepted else "hold",
        "elapsed_wall_sec": episode_execution_wall_sec,
        "episode_execution_wall_sec": episode_execution_wall_sec,
        "report_refresh_wall_sec": refresh_wall_sec,
        "episode_scope": experiment_manifest["episode_scope"],
        "scenarios": all_results,
        "production_async_reference": async_comparison,
        "production_vs_lockstep_interpretation": {
            "production_async": (
                "controller/wall-timer phase diverged first, followed by pose, scan, perception, "
                "and commitment divergence"),
            "lockstep": (
                "logical stamp barriers remove callback arrival order and wall-timer phase from fitness"),
            "roles": {
                "lockstep": "CMA optimization environment",
                "production_async": "final end-to-end robustness validation environment",
            },
        },
    }
    atomic_write_json(output / "smoke_audit.json", report)
    print(json.dumps({
        "accepted": accepted,
        "summaries": {name: item["summary"] for name, item in all_results.items()},
    }, indent=2, sort_keys=True))
    return 0 if accepted else 3


if __name__ == "__main__":
    raise SystemExit(run())
