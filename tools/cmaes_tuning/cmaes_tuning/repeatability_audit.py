"""Repeated closed-loop audit without invoking CMA ask/tell."""

from __future__ import annotations

from collections import defaultdict
import copy
import csv
import json
import math
from pathlib import Path
import shutil
from typing import Any

import numpy as np
import yaml

from .configuration import resolve_workspace_path
from .experiment_logger import create_experiment_manifest
from .noise_diagnostics import analyze_episode_noise
from .objective import candidate_fitness
from .parameter_space import ParameterSpace
from .scenario_audit import audit_scenario_dataset
from .scenario_generator import generate_stratified_scenario_dataset
from .schemas import atomic_write_json, sha256_file
from .simulation_runner import EpisodeRunner, ExperimentInfrastructureError


def _statistics(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "mean": None, "std": None, "min": None, "max": None}
    array = np.asarray(values, dtype=np.float64)
    return {
        "count": int(array.size),
        "mean": float(np.mean(array)),
        "std": float(np.std(array, ddof=1)) if array.size > 1 else 0.0,
        "min": float(np.min(array)),
        "max": float(np.max(array)),
    }


def _rate(rows: list[dict[str, Any]], predicate: Any) -> float:
    return sum(bool(predicate(row)) for row in rows) / len(rows) if rows else 0.0


def _nested(document: dict[str, Any], *keys: str) -> float | None:
    current: Any = document
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return None
        current = current[key]
    return float(current) if current is not None else None


class RepeatabilityAudit:
    def __init__(
        self,
        config: dict[str, Any],
        workspace_root: str | Path,
        config_path: str | Path,
        experiment_id: str,
    ):
        self.workspace_root = Path(workspace_root).resolve()
        self.config_path = Path(config_path).resolve()
        self.config = copy.deepcopy(config)
        output_root = resolve_workspace_path(
            self.config["paths"]["output_root"], self.workspace_root
        )
        self.directory = output_root / experiment_id
        self.directory.mkdir(parents=True, exist_ok=True)
        self.config["paths"]["output_root"] = str(self.directory)
        self.frozen_config_path = self.directory / "configuration" / "tuning_config.yaml"
        self.frozen_config_path.parent.mkdir(parents=True, exist_ok=True)
        self.frozen_config_path.write_text(
            yaml.safe_dump(self.config, sort_keys=False), encoding="utf-8"
        )

        self.datasets = self._prepare_datasets()
        self.dataset_audit = audit_scenario_dataset(
            self.datasets,
            self.config,
            self.directory / "scenario_dataset_audit.json",
        )
        if not self.dataset_audit["all_valid"]:
            raise RuntimeError("scenario dataset failed offline validity audit")
        self.representative_scenarios = self._select_representative_scenarios()
        self.candidates = self._prepare_candidates()
        self.runner = EpisodeRunner(self.config, self.workspace_root)
        self.episode_counter = 0
        all_scenarios = self.datasets["training"] + self.datasets["validation"]
        manifest = create_experiment_manifest(
            self.directory / "experiment_manifest.json",
            self.workspace_root,
            self.frozen_config_path,
            self.config,
            all_scenarios,
        )
        manifest.update(
            {
                "experiment_type": "repeatability_and_dataset_audit",
                "localization_mode": self.config["experiment"].get(
                    "localization_mode", "mcl"
                ),
                "medium_or_full_cma_executed": False,
                "dataset_audit_path": str(
                    (self.directory / "scenario_dataset_audit.json").resolve()
                ),
                "dataset_audit_sha256": sha256_file(
                    self.directory / "scenario_dataset_audit.json"
                ),
                "repeatability_scenario_ids": [
                    json.loads(path.read_text(encoding="utf-8"))["scenario_id"]
                    for path in self.representative_scenarios
                ],
                "candidate_hashes": {
                    name: sha256_file(path) for name, path in self.candidates.items()
                },
            }
        )
        atomic_write_json(self.directory / "experiment_manifest.json", manifest)

    def _prepare_datasets(self) -> dict[str, list[Path]]:
        source_value = str(
            self.config["repeatability"].get(
                "scenario_dataset_source_experiment", ""
            )
        ).strip()
        if not source_value:
            return generate_stratified_scenario_dataset(
                self.config, self.workspace_root
            )
        source = resolve_workspace_path(source_value, self.workspace_root)
        source_datasets = {
            split: sorted((source / "scenarios" / split).glob("*/manifest.json"))
            for split in ("training", "validation")
        }
        expected = {
            "training": int(self.config["dataset"]["training_count"]),
            "validation": int(self.config["dataset"]["validation_count"]),
        }
        for split, paths in source_datasets.items():
            if len(paths) != expected[split]:
                raise RuntimeError(
                    f"source experiment {source} has {len(paths)} {split} scenarios; "
                    f"expected {expected[split]}"
                )
        # Keep the exact scenario geometry and baked maps while refreshing only
        # simulator provenance that legitimately changes between diagnostics
        # (seed/noise/publication controls). Source manifests remain untouched.
        datasets: dict[str, list[Path]] = {"training": [], "validation": []}
        runtime = self.config["simulator_runtime"]
        simulator_config = Path(self.config["paths"]["simulator_config"]).resolve()
        collision_source = Path(
            self.config["paths"]["simulator_collision_source"]
        ).resolve()
        for split, paths in source_datasets.items():
            for source_manifest in paths:
                document = json.loads(source_manifest.read_text(encoding="utf-8"))
                document["simulator_config"] = str(simulator_config)
                document["simulator_config_sha256"] = sha256_file(simulator_config)
                document["simulator_collision_source"] = str(collision_source)
                document["simulator_collision_source_sha256"] = sha256_file(
                    collision_source
                )
                document["simulator_collision_model"]["scan_noise_std_m"] = float(
                    runtime["scan_noise_std_m"]
                )
                document["source_manifest"] = str(source_manifest.resolve())
                document["source_manifest_sha256"] = sha256_file(source_manifest)
                destination = (
                    self.directory
                    / "scenarios"
                    / split
                    / str(document["scenario_id"])
                    / "manifest.json"
                )
                atomic_write_json(destination, document)
                datasets[split].append(destination)
        return datasets

    def _select_representative_scenarios(self) -> list[Path]:
        exact_ids = list(
            self.config["repeatability"].get("representative_scenario_ids", [])
        )
        if exact_ids:
            by_id = {
                str(json.loads(path.read_text(encoding="utf-8"))["scenario_id"]): path
                for path in self.datasets["training"]
            }
            missing = [scenario_id for scenario_id in exact_ids if scenario_id not in by_id]
            if missing:
                raise RuntimeError(f"representative training scenarios missing: {missing}")
            if len(set(exact_ids)) != len(exact_ids):
                raise ValueError("representative_scenario_ids contains duplicates")
            return [by_id[scenario_id] for scenario_id in exact_ids]
        target_bands = self.config["repeatability"]["representative_lateral_bands"]
        by_category_band: dict[tuple[str, str], list[Path]] = defaultdict(list)
        for path in self.datasets["training"]:
            manifest = json.loads(path.read_text(encoding="utf-8"))
            by_category_band[
                (str(manifest["category"]), str(manifest["lateral_band"]))
            ].append(path)
        selected = []
        for category in self.config["dataset"]["categories"]:
            band = str(target_bands[category])
            candidates = sorted(by_category_band[(str(category), band)])
            if not candidates:
                raise RuntimeError(f"no representative scenario for {category}/{band}")
            selected.append(candidates[0])
        expected = int(self.config["repeatability"]["scenario_count"])
        if len(selected) != expected:
            raise ValueError("repeatability scenario count differs from category selection")
        return selected

    def _prepare_candidates(self) -> dict[str, Path]:
        parameter_path = resolve_workspace_path(
            self.config["paths"]["parameter_space_yaml"], self.workspace_root
        )
        baseline_source = resolve_workspace_path(
            self.config["paths"]["baseline_planner_yaml"], self.workspace_root
        )
        parameter_space = ParameterSpace.load(parameter_path)
        parameter_space.validate_baseline(baseline_source)
        candidate_directory = self.directory / "candidates"
        candidate_directory.mkdir(parents=True, exist_ok=True)
        baseline = candidate_directory / "baseline.yaml"
        parameter_space.copy_baseline(baseline_source, baseline)
        requested = list(
            self.config["repeatability"].get(
                "candidate_names", ["baseline", "smoke_best"]
            )
        )
        unknown = set(requested) - {"baseline", "smoke_best"}
        if unknown or not requested:
            raise ValueError(f"invalid repeatability candidate_names: {requested}")
        candidates = {"baseline": baseline}
        if "smoke_best" not in requested:
            return {name: candidates[name] for name in requested}
        smoke_best = candidate_directory / "smoke_best.yaml"
        smoke_source = resolve_workspace_path(
            self.config["repeatability"]["smoke_best_candidate_yaml"],
            self.workspace_root,
        )
        if not smoke_source.is_file():
            raise FileNotFoundError(f"smoke best candidate is missing: {smoke_source}")
        shutil.copy2(smoke_source, smoke_best)
        candidates["smoke_best"] = smoke_best
        return {name: candidates[name] for name in requested}

    def _next_domain(self) -> int:
        # Repeatability retries are issued one at a time below, so every call
        # needs one isolated domain rather than reserving a retry-sized block.
        domain = int(self.config["ros"]["domain_id_start"]) + self.episode_counter
        self.episode_counter += 1
        if domain > 230:
            raise ExperimentInfrastructureError("configured ROS domain pool is exhausted")
        return domain

    def _existing_result(
        self, repetition_root: Path, scenario_path: Path
    ) -> tuple[dict[str, Any], Path] | None:
        for result_path in sorted(repetition_root.glob("attempt_*/episode_result.json")):
            attempt = result_path.parent
            result = json.loads(result_path.read_text(encoding="utf-8"))
            if result.get("valid", False):
                return result, attempt
        return None

    def _run_episode(
        self,
        candidate_name: str,
        candidate_path: Path,
        scenario_path: Path,
        repetition: int,
    ) -> dict[str, Any]:
        scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
        repetition_root = (
            self.directory
            / "repeatability_episodes"
            / candidate_name
            / str(scenario["scenario_id"])
            / f"repeat_{repetition:02d}"
        )
        existing = self._existing_result(repetition_root, scenario_path)
        if existing is None:
            prior_attempts = sorted(repetition_root.glob("attempt_*"))
            next_attempt_index = max(
                (
                    int(path.name.removeprefix("attempt_"))
                    for path in prior_attempts
                    if path.name.removeprefix("attempt_").isdigit()
                ),
                default=-1,
            ) + 1
            maximum_attempts = int(
                self.config["repeatability"][
                    "max_total_infrastructure_attempts_per_episode"
                ]
            )
            result = None
            attempt = None
            for attempt_index in range(next_attempt_index, maximum_attempts):
                attempt = repetition_root / f"attempt_{attempt_index}"
                result = self.runner.run_once(
                    candidate_path,
                    scenario_path,
                    attempt,
                    self._next_domain(),
                )
                if result.get("valid", False):
                    break
            if result is None or attempt is None or not result.get("valid", False):
                raise ExperimentInfrastructureError(
                    f"episode remained invalid after {maximum_attempts} total attempts: "
                    f"{repetition_root}"
                )
        else:
            result, attempt = existing
        attempt_index = int(attempt.name.removeprefix("attempt_"))
        infrastructure_attempt_count = len(list(repetition_root.glob("attempt_*")))
        diagnostics_path = attempt / "noise_diagnostics.json"
        if diagnostics_path.is_file():
            diagnostics = json.loads(diagnostics_path.read_text(encoding="utf-8"))
        else:
            diagnostics = {}
        if diagnostics.get("schema") != "cmaes_episode_noise_diagnostics/8":
            diagnostics = analyze_episode_noise(
                attempt / "bag",
                scenario_path,
                attempt / "runner_status.json",
            )
            atomic_write_json(diagnostics_path, diagnostics)
        fitness = candidate_fitness([result], self.config)["fitness"]
        return {
            "candidate": candidate_name,
            "scenario_id": scenario["scenario_id"],
            "category": scenario["category"],
            "lateral_band": scenario["lateral_band"],
            "repetition": repetition,
            "attempt_index": attempt_index,
            "infrastructure_attempt_count": infrastructure_attempt_count,
            "episode_directory": str(attempt.resolve()),
            "classification": result["classification"],
            "completed": bool(result["metrics"]["completed"]),
            "collision": bool(result["failure"]["collision"]),
            "collision_topic": bool(result["failure"]["collision_topic"]),
            "external_collision": bool(result["failure"]["external_collision"]),
            "collision_agreement": bool(result["failure"]["collision_agreement"]),
            "off_track": bool(result["failure"]["off_track"]),
            "planner_failure": bool(result["failure"]["planner_failure"]),
            "completion_time_s": float(result["metrics"]["completion_time_s"]),
            "fitness": float(fitness),
            "minimum_true_obstacle_clearance_m": float(
                result["metrics"]["minimum_obstacle_clearance_m"]
            ),
            "steering_tv_per_s": float(
                result["metrics"]["steering_total_variation_per_s"]
            ),
            "planned_curvature_rate_rms": float(
                result["metrics"]["planned_curvature_rate_rms_radpm2"]
            ),
            "noise": diagnostics,
        }

    @staticmethod
    def _summarize_group(rows: list[dict[str, Any]]) -> dict[str, Any]:
        safety_signatures = {
            (
                bool(row["collision"]),
                bool(row["off_track"]),
                bool(row["planner_failure"]),
            )
            for row in rows
        }
        completed_rows = [row for row in rows if row["completed"]]
        initial_targets = [
            value
            for row in rows
            if (
                value := _nested(
                    row["noise"], "planner_log", "initial_commit_target_d_m"
                )
            ) is not None
        ]
        backend_before_commitment = [
            value
            for row in rows
            if (
                value := _nested(
                    row["noise"],
                    "scan_identity",
                    "unique_backend_scans_before_commitment",
                )
            ) is not None
        ]
        ros_before_commitment = [
            value
            for row in rows
            if (
                value := _nested(
                    row["noise"],
                    "scan_identity",
                    "ros_scan_messages_before_commitment",
                )
            ) is not None
        ]
        return {
            "run_count": len(rows),
            "success_rate": _rate(rows, lambda row: row["classification"] == "success"),
            "collision_rate": _rate(rows, lambda row: row["collision"]),
            "off_track_rate": _rate(rows, lambda row: row["off_track"]),
            "planner_failure_rate": _rate(rows, lambda row: row["planner_failure"]),
            "completion_time_s": _statistics(
                [float(row["completion_time_s"]) for row in completed_rows]
            ),
            "fitness": _statistics([float(row["fitness"]) for row in rows]),
            "minimum_true_obstacle_clearance_m": _statistics(
                [float(row["minimum_true_obstacle_clearance_m"]) for row in rows]
            ),
            "steering_tv_per_s": _statistics(
                [float(row["steering_tv_per_s"]) for row in rows]
            ),
            "planned_curvature_rate_rms": _statistics(
                [float(row["planned_curvature_rate_rms"]) for row in rows]
            ),
            "initial_commit_target_d_m": _statistics(initial_targets),
            "unique_backend_scans_before_commitment": _statistics(
                backend_before_commitment
            ),
            "ros_scan_messages_before_commitment": _statistics(
                ros_before_commitment
            ),
            "obstacle_id_split_rate": _rate(
                rows,
                lambda row: bool(
                    row["noise"].get("detector_semantics", {}).get(
                        "obstacle_id_split", False
                    )
                ),
            ),
            "obstacle_id_split_before_commitment_rate": _rate(
                rows,
                lambda row: bool(
                    row["noise"].get("detector_semantics", {}).get(
                        "obstacle_id_split_before_commitment", False
                    )
                ),
            ),
            "duplicate_scan_contributed_to_confirmation_count": sum(
                row["noise"].get("detector_semantics", {}).get(
                    "existence_confirmation_duplicate_contribution"
                ) is True
                for row in rows
            ),
            "duplicate_scan_contributed_to_envelope_stability_count": sum(
                row["noise"].get("detector_semantics", {}).get(
                    "envelope_stability_duplicate_contribution"
                ) is True
                for row in rows
            ),
            "infrastructure_attempt_count": _statistics(
                [float(row["infrastructure_attempt_count"]) for row in rows]
            ),
            "episodes_requiring_infrastructure_retry_rate": _rate(
                rows, lambda row: int(row["infrastructure_attempt_count"]) > 1
            ),
            "total_invalid_infrastructure_attempts": sum(
                max(0, int(row["infrastructure_attempt_count"]) - 1)
                for row in rows
            ),
            "safety_outcome_changed_between_repetitions": len(safety_signatures) > 1,
            "safety_outcomes": [
                {
                    "collision": signature[0],
                    "off_track": signature[1],
                    "planner_failure": signature[2],
                }
                for signature in sorted(safety_signatures)
            ],
        }

    @staticmethod
    def _noise_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
        paths = {
            "scan_residual_std_m": ("scan_noise", "residual_m", "std"),
            "scan_residual_mean_m": ("scan_noise", "residual_m", "mean"),
            "localization_position_error_mean_m": (
                "localization",
                "position_error_m",
                "mean",
            ),
            "localization_yaw_error_mean_rad": (
                "localization",
                "yaw_error_rad",
                "mean",
            ),
            "localization_exact_timestamp_match_fraction": (
                "localization",
                "exact_source_timestamp_match_fraction",
            ),
            "localization_bridge_step_error_max_m": (
                "localization",
                "bridge_introduced_position_step_error_m",
                "max",
            ),
            "localization_bridge_yaw_step_error_max_rad": (
                "localization",
                "bridge_introduced_yaw_step_error_rad",
                "max",
            ),
            "tf_position_error_mean_m": (
                "tf_consistency",
                "position_error_m",
                "mean",
            ),
            "tf_yaw_error_mean_rad": (
                "tf_consistency",
                "yaw_error_rad",
                "mean",
            ),
            "tf_duplicate_edge_sample_count": (
                "tf_consistency",
                "duplicate_edge_sample_count",
            ),
            "drive_interval_std_us": ("topic_interval_us", "/drive", "std"),
            "scan_interval_std_us": ("topic_interval_us", "/scan", "std"),
            "planner_detection_to_path_ms": (
                "planner_timing",
                "detection_to_path_ms",
            ),
            "detector_scan_to_static_obs_ms": (
                "planner_timing",
                "first_detection_scan_to_static_obs_ms",
            ),
            "first_scan_to_first_static_ms": (
                "planner_timing",
                "first_scan_to_first_static_ms",
            ),
            "first_scan_to_first_confirmed_static_ms": (
                "planner_timing",
                "first_scan_to_first_confirmed_static_ms",
            ),
            "first_static_to_confirmed_static_ms": (
                "planner_timing",
                "first_confirmed_static_after_first_static_ms",
            ),
            "first_drive_to_first_static_ms": (
                "planner_timing",
                "first_drive_to_first_static_ms",
            ),
            "first_drive_to_first_confirmed_static_ms": (
                "planner_timing",
                "first_drive_to_first_confirmed_static_ms",
            ),
            "detector_processing_latency_mean_ms": (
                "planner_timing",
                "scan_to_static_obs_processing_ms",
                "mean",
            ),
            "static_messages_before_first_path": (
                "planner_timing",
                "nonempty_static_messages_before_first_path",
            ),
            "first_detection_manifest_center_error_m": (
                "planner_timing",
                "first_detection",
                "center_error_to_manifest_m",
            ),
            "first_detection_ego_distance_m": (
                "planner_timing",
                "first_detection",
                "ego_center_distance_m",
            ),
            "first_detection_d_center_m": (
                "planner_timing",
                "first_detection",
                "d_center_m",
            ),
            "first_path_maximum_abs_d_m": (
                "planner_timing",
                "first_path",
                "maximum_abs_d_m",
            ),
            "planner_initial_commit_target_d_m": (
                "planner_log",
                "initial_commit_target_d_m",
            ),
            "planner_replacement_count": (
                "planner_log",
                "replacement_count",
            ),
            "planner_hard_commitment_collision_count": (
                "planner_log",
                "hard_commitment_collision_count",
            ),
            "planner_safe_stop_latch_count": (
                "planner_log",
                "safe_stop_latch_count",
            ),
            "scan_publications_per_backend_mean": (
                "scan_identity",
                "publications_per_backend_scan",
                "mean",
            ),
            "scan_publications_per_backend_max": (
                "scan_identity",
                "publications_per_backend_scan",
                "max",
            ),
            "post_initial_scan_publications_per_backend_mean": (
                "scan_identity",
                "post_initial_publications_per_backend_scan",
                "mean",
            ),
            "post_initial_scan_publications_per_backend_max": (
                "scan_identity",
                "post_initial_publications_per_backend_scan",
                "max",
            ),
            "duplicate_scan_confirmation_flag": (
                "detector_semantics",
                "existence_confirmation_duplicate_contribution",
            ),
            "duplicate_scan_envelope_flag": (
                "detector_semantics",
                "envelope_stability_duplicate_contribution",
            ),
            "obstacle_id_split_flag": (
                "detector_semantics",
                "obstacle_id_split",
            ),
            "unique_backend_scans_before_commitment": (
                "scan_identity",
                "unique_backend_scans_before_commitment",
            ),
            "ros_scan_messages_before_commitment": (
                "scan_identity",
                "ros_scan_messages_before_commitment",
            ),
            "launch_duration_s": ("process_phase_durations_s", "launch_duration_s"),
            "wait_ready_duration_s": (
                "process_phase_durations_s",
                "wait_ready_duration_s",
            ),
            "runtime_load_average_1m_mean": (
                "runtime",
                "load_average_1m_mean",
            ),
            "runtime_poll_interval_s_max": (
                "runtime",
                "poll_interval_s_max",
            ),
            "t3_minus_t2_ms": (
                "end_to_end_timing", "latencies_ms", "t3_minus_t2",
            ),
            "t4_minus_t3_ms": (
                "end_to_end_timing", "latencies_ms", "t4_minus_t3",
            ),
            "t5_minus_t4_ms": (
                "end_to_end_timing", "latencies_ms", "t5_minus_t4",
            ),
            "t8_minus_t6_ms": (
                "end_to_end_timing", "latencies_ms", "t8_minus_t6",
            ),
            "t9_minus_t8_ms": (
                "end_to_end_timing", "latencies_ms", "t9_minus_t8",
            ),
            "t9_minus_t1_ms": (
                "end_to_end_timing", "latencies_ms", "t9_minus_t1",
            ),
            "avoidance_start_ego_s": (
                "end_to_end_timing", "avoidance_start_ego_s",
            ),
            "timing_complete_chain_flag": (
                "end_to_end_timing", "complete_chain",
            ),
        }
        summary = {}
        for name, path in paths.items():
            values = []
            for row in rows:
                value = _nested(row["noise"], *path)
                if value is not None and math.isfinite(value):
                    values.append(value)
            summary[name] = _statistics(values)
        return summary

    def run(self) -> dict[str, Any]:
        repetitions = int(self.config["repeatability"]["repetitions"])
        rows = []
        # Interleave candidates within each scenario/repetition so time-varying
        # host load cannot systematically favor the candidate that runs first.
        for repetition in range(repetitions):
            for scenario_path in self.representative_scenarios:
                for candidate_name, candidate_path in self.candidates.items():
                    rows.append(
                        self._run_episode(
                            candidate_name,
                            candidate_path,
                            scenario_path,
                            repetition,
                        )
                    )
                    atomic_write_json(
                        self.directory / "repeatability_runs.json",
                        {"schema": "cmaes_repeatability_runs/1", "runs": rows},
                    )

        grouped: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
        by_candidate: dict[str, list[dict[str, Any]]] = defaultdict(list)
        for row in rows:
            grouped[(str(row["candidate"]), str(row["scenario_id"]))].append(row)
            by_candidate[str(row["candidate"])].append(row)
        combinations = []
        for (candidate, scenario_id), group in sorted(grouped.items()):
            combinations.append(
                {
                    "candidate": candidate,
                    "scenario_id": scenario_id,
                    "category": group[0]["category"],
                    "lateral_band": group[0]["lateral_band"],
                    **self._summarize_group(group),
                }
            )
        candidate_summaries = {
            candidate: {
                **self._summarize_group(group),
                "noise": self._noise_summary(group),
            }
            for candidate, group in sorted(by_candidate.items())
        }
        unstable = [
            item
            for item in combinations
            if item["safety_outcome_changed_between_repetitions"]
        ]
        completion_cvs = []
        fitness_scaled_stds = []
        for item in combinations:
            completion = item["completion_time_s"]
            if completion["mean"] not in (None, 0.0) and completion["std"] is not None:
                completion_cvs.append(float(completion["std"]) / float(completion["mean"]))
            fitness = item["fitness"]
            if fitness["std"] is not None and fitness["mean"] is not None:
                fitness_scaled_stds.append(
                    float(fitness["std"]) / max(1.0, abs(float(fitness["mean"])))
                )
        stability = {
            "safety_outcome_flip_count": len(unstable),
            "maximum_completed_time_coefficient_of_variation": max(
                completion_cvs, default=0.0
            ),
            "maximum_fitness_scaled_std": max(fitness_scaled_stds, default=0.0),
            "acceptance_thresholds": {
                "safety_outcome_flip_count": 0,
                "completed_time_cv_max": 0.05,
                "fitness_scaled_std_max": 0.10,
            },
        }
        stability["suitable_for_single_repeat_cma"] = (
            stability["safety_outcome_flip_count"] == 0
            and stability["maximum_completed_time_coefficient_of_variation"] <= 0.05
            and stability["maximum_fitness_scaled_std"] <= 0.10
        )
        report = {
            "schema": "cmaes_repeatability_audit/1",
            "execution_order": "repetition_then_scenario_then_interleaved_candidate",
            "medium_or_full_cma_executed": False,
            "localization_mode": self.config["experiment"].get(
                "localization_mode", "mcl"
            ),
            "fixed_controller_configuration": self.config["controller"],
            "run_count": len(rows),
            "repetitions_per_combination": repetitions,
            "candidate_summaries": candidate_summaries,
            "scenario_candidate_combinations": combinations,
            "unstable_safety_combinations": unstable,
            "stability": stability,
            "collision_agreement": {
                "agreement_count": sum(bool(row["collision_agreement"]) for row in rows),
                "run_count": len(rows),
                "all_agree": all(bool(row["collision_agreement"]) for row in rows),
                "topic_and_external_collision_count": sum(
                    bool(row["collision_topic"]) and bool(row["external_collision"])
                    for row in rows
                ),
                "topic_and_external_clear_count": sum(
                    not bool(row["collision_topic"])
                    and not bool(row["external_collision"])
                    for row in rows
                ),
            },
            "dataset_audit_path": str(
                (self.directory / "scenario_dataset_audit.json").resolve()
            ),
        }
        previous_value = str(
            self.config["repeatability"].get("previous_mcl_experiment", "")
        ).strip()
        if previous_value:
            previous_root = resolve_workspace_path(previous_value, self.workspace_root)
            previous_path = previous_root / "repeatability_audit.json"
            previous = json.loads(previous_path.read_text(encoding="utf-8"))
            previous_baseline = previous["candidate_summaries"]["baseline"]
            current_baseline = candidate_summaries["baseline"]
            previous_flips = sum(
                item.get("candidate") == "baseline"
                and bool(item.get("safety_outcome_changed_between_repetitions"))
                for item in previous["scenario_candidate_combinations"]
            )
            current_flips = int(stability["safety_outcome_flip_count"])
            previous_fitness_std = float(previous_baseline["fitness"]["std"])
            current_fitness_std = float(current_baseline["fitness"]["std"])
            report["previous_mcl_comparison"] = {
                "source": str(previous_path.resolve()),
                "source_localization_mode": previous.get("localization_mode", "mcl"),
                "previous_baseline": previous_baseline,
                "safety_flip_count": {
                    "previous": previous_flips,
                    "current": current_flips,
                    "change": current_flips - previous_flips,
                },
                "fitness_std": {
                    "previous": previous_fitness_std,
                    "current": current_fitness_std,
                    "ratio_current_to_previous": (
                        current_fitness_std / previous_fitness_std
                        if previous_fitness_std > 0.0 else None
                    ),
                },
            }
        atomic_write_json(self.directory / "repeatability_audit.json", report)
        csv_fields = [
            "candidate",
            "scenario_id",
            "category",
            "lateral_band",
            "run_count",
            "success_rate",
            "collision_rate",
            "off_track_rate",
            "planner_failure_rate",
            "completion_time_mean_s",
            "completion_time_std_s",
            "fitness_mean",
            "fitness_std",
            "minimum_true_obstacle_clearance_mean_m",
            "minimum_true_obstacle_clearance_std_m",
            "steering_tv_per_s_mean",
            "steering_tv_per_s_std",
            "planned_curvature_rate_rms_mean",
            "planned_curvature_rate_rms_std",
            "safety_outcome_changed_between_repetitions",
            "episodes_requiring_infrastructure_retry_rate",
        ]
        with (self.directory / "repeatability_summary.csv").open(
            "w", encoding="utf-8", newline=""
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=csv_fields)
            writer.writeheader()
            for item in combinations:
                writer.writerow(
                    {
                        "candidate": item["candidate"],
                        "scenario_id": item["scenario_id"],
                        "category": item["category"],
                        "lateral_band": item["lateral_band"],
                        "run_count": item["run_count"],
                        "success_rate": item["success_rate"],
                        "collision_rate": item["collision_rate"],
                        "off_track_rate": item["off_track_rate"],
                        "planner_failure_rate": item["planner_failure_rate"],
                        "completion_time_mean_s": item["completion_time_s"]["mean"],
                        "completion_time_std_s": item["completion_time_s"]["std"],
                        "fitness_mean": item["fitness"]["mean"],
                        "fitness_std": item["fitness"]["std"],
                        "minimum_true_obstacle_clearance_mean_m": item[
                            "minimum_true_obstacle_clearance_m"
                        ]["mean"],
                        "minimum_true_obstacle_clearance_std_m": item[
                            "minimum_true_obstacle_clearance_m"
                        ]["std"],
                        "steering_tv_per_s_mean": item["steering_tv_per_s"]["mean"],
                        "steering_tv_per_s_std": item["steering_tv_per_s"]["std"],
                        "planned_curvature_rate_rms_mean": item[
                            "planned_curvature_rate_rms"
                        ]["mean"],
                        "planned_curvature_rate_rms_std": item[
                            "planned_curvature_rate_rms"
                        ]["std"],
                        "safety_outcome_changed_between_repetitions": item[
                            "safety_outcome_changed_between_repetitions"
                        ],
                        "episodes_requiring_infrastructure_retry_rate": item[
                            "episodes_requiring_infrastructure_retry_rate"
                        ],
                    }
                )
        return report
