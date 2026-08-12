"""Test A-D orchestration with cache and hard gating before CMA smoke."""

from __future__ import annotations

import json
from pathlib import Path
import shutil
from typing import Any

from .controller_audit import audit_to_file
from .configuration import resolve_workspace_path
from .evaluator import evaluate_episode
from .experiment_logger import (
    append_result_csv,
    atomic_pickle,
    create_experiment_manifest,
)
from .objective import candidate_fitness
from .parameter_space import ParameterSpace
from .scenario_generator import generate_fixed_scenarios
from .schemas import atomic_write_json
from .simulation_runner import EpisodeRunner, ExperimentInfrastructureError


class SmokeExperiment:
    def __init__(
        self,
        config: dict[str, Any],
        workspace_root: str | Path,
        config_path: str | Path,
        experiment_id: str,
    ):
        self.config = config
        self.workspace_root = Path(workspace_root).resolve()
        self.config_path = Path(config_path).resolve()
        base_output = resolve_workspace_path(
            config["paths"]["output_root"], self.workspace_root
        )
        self.directory = base_output / experiment_id
        self.directory.mkdir(parents=True, exist_ok=True)
        self.config["paths"]["output_root"] = str(self.directory)
        parameter_path = self.workspace_root / self.config["paths"]["parameter_space_yaml"]
        baseline_path = self.workspace_root / self.config["paths"]["baseline_planner_yaml"]
        self.parameter_space = ParameterSpace.load(parameter_path)
        self.parameter_space.validate_baseline(baseline_path)
        self.baseline_yaml = self.directory / "candidates" / "baseline.yaml"
        if not self.baseline_yaml.exists():
            self.parameter_space.copy_baseline(baseline_path, self.baseline_yaml)
        self.scenarios = generate_fixed_scenarios(self.config, self.workspace_root)
        self.runner = EpisodeRunner(self.config, self.workspace_root)
        self.episode_counter = 0
        create_experiment_manifest(
            self.directory / "experiment_manifest.json",
            self.workspace_root,
            self.config_path,
            self.config,
            self.scenarios,
        )

    def _next_domain(self) -> int:
        stride = int(self.config["runner"]["max_infrastructure_retries"]) + 1
        domain = int(self.config["ros"]["domain_id_start"]) + self.episode_counter * stride
        self.episode_counter += 1
        if domain > 230:
            raise ExperimentInfrastructureError("configured ROS domain pool is exhausted")
        return domain

    def candidate_yaml(self, candidate_id: str, z: list[float]) -> Path:
        path = self.directory / "candidates" / f"{candidate_id}.yaml"
        metadata_path = path.with_suffix(".json")
        if not path.exists():
            metadata = self.parameter_space.write_candidate_yaml(
                self.workspace_root / self.config["paths"]["baseline_planner_yaml"], path, z
            )
            atomic_write_json(metadata_path, metadata)
        return path

    def run_candidate(
        self,
        candidate_id: str,
        candidate_yaml: Path,
        scenarios: list[Path],
        generation: int | str,
    ) -> dict[str, Any]:
        episode_results = []
        for scenario_path in scenarios:
            scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
            episode_root = (
                self.directory / "episodes" / candidate_id / str(scenario["scenario_id"])
            )
            cached = None
            for result_path in sorted(episode_root.glob("attempt_*/episode_result.json")):
                attempt_directory = result_path.parent
                # Re-run the current evaluator against immutable bag/manifest
                # artifacts so metric fixes do not require another simulation.
                cached = evaluate_episode(
                    attempt_directory / "bag",
                    scenario_path,
                    attempt_directory / "rollout_summary.json",
                    attempt_directory / "runner_status.json",
                    self.config,
                    result_path,
                )
                if cached.get("valid", False):
                    break
                cached = None
            if cached is None:
                cached = self.runner.run_with_retries(
                    candidate_yaml,
                    scenario_path,
                    episode_root,
                    self._next_domain(),
                )
            episode_results.append(cached)
        aggregate = candidate_fitness(episode_results, self.config)
        result = {
            "schema": "cmaes_candidate_result/1",
            "candidate_id": candidate_id,
            "generation": generation,
            "candidate_yaml": str(candidate_yaml),
            "episode_results": episode_results,
            "aggregate": aggregate,
        }
        result_path = self.directory / "candidate_results" / f"{candidate_id}.json"
        atomic_write_json(result_path, result)
        for episode in episode_results:
            metrics = episode["metrics"]
            failure = episode["failure"]
            append_result_csv(
                self.directory / "results.csv",
                {
                    "candidate_id": candidate_id,
                    "generation": generation,
                    "scenario_id": episode["scenario_id"],
                    "fitness": aggregate["fitness"],
                    "classification": episode["classification"],
                    "collision": failure["collision"],
                    "off_track": failure["off_track"],
                    "planner_failure": failure["planner_failure"],
                    "completed": metrics["completed"],
                    "completion_time_s": metrics["completion_time_s"],
                    "progress_fraction": metrics["progress_fraction"],
                    "minimum_obstacle_clearance_m": metrics["minimum_obstacle_clearance_m"],
                    "minimum_wall_clearance_m": metrics["minimum_wall_clearance_m"],
                    "steering_total_variation_per_s": metrics[
                        "steering_total_variation_per_s"
                    ],
                    "planned_curvature_rate_rms_radpm2": metrics[
                        "planned_curvature_rate_rms_radpm2"
                    ],
                },
            )
        return result

    def test_a(self) -> dict[str, Any]:
        result = self.run_candidate("baseline", self.baseline_yaml, self.scenarios[:1], "A")
        attempt = int(result["episode_results"][0].get("attempt_count", 1)) - 1
        bag_path = (
            self.directory
            / "episodes"
            / "baseline"
            / "train_000"
            / f"attempt_{attempt}"
            / "bag"
        )
        audit = audit_to_file(
            bag_path,
            self.config["controller"],
            self.directory / "controller_audit.json",
        )
        report = {"test": "A", "candidate": result, "controller_audit": audit}
        atomic_write_json(self.directory / "test_a.json", report)
        return report

    def test_b(self) -> dict[str, Any]:
        result = self.run_candidate("baseline", self.baseline_yaml, self.scenarios, "B")
        report = {"test": "B", "candidate": result}
        atomic_write_json(self.directory / "test_b.json", report)
        return report

    def test_c(self) -> dict[str, Any]:
        baseline = self.test_b()
        physical = self.parameter_space.baseline_physical()
        physical.update(
            {
                "safety_margin_m": 0.05,
                "obstacle_longitudinal_padding_m": 0.58,
                "pre_apex_far_m": 11.0,
                "post_apex_far_m": 6.5,
                "transition_short": 0.48,
                "transition_middle": 1.40,
                "transition_long": 4.20,
                "outside_line_transition_scale": 0.30,
                "minimum_target_offset_m": 0.25,
                "wall_safety_margin_m": 0.06,
            }
        )
        comparison_yaml = self.candidate_yaml(
            "longer_larger_margin", self.parameter_space.encode(physical)
        )
        comparison = self.run_candidate(
            "longer_larger_margin", comparison_yaml, self.scenarios, "C"
        )
        differences = []
        for first, second in zip(
            baseline["candidate"]["episode_results"], comparison["episode_results"]
        ):
            first_metrics, second_metrics = first["metrics"], second["metrics"]
            differences.append(
                {
                    "scenario_id": first["scenario_id"],
                    "clearance_delta_m": second_metrics["minimum_obstacle_clearance_m"]
                    - first_metrics["minimum_obstacle_clearance_m"],
                    "steering_tv_rate_delta": second_metrics["steering_total_variation_per_s"]
                    - first_metrics["steering_total_variation_per_s"],
                    "completion_time_delta_s": second_metrics["completion_time_s"]
                    - first_metrics["completion_time_s"],
                    "path_length_delta_m": second_metrics["planned_path_mean_length_m"]
                    - first_metrics["planned_path_mean_length_m"],
                    "path_curvature_delta_radpm": second_metrics[
                        "planned_path_max_curvature_radpm"
                    ]
                    - first_metrics["planned_path_max_curvature_radpm"],
                }
            )
        sensitive = any(
            abs(item["clearance_delta_m"]) > 0.002
            or abs(item["steering_tv_rate_delta"]) > 0.002
            or abs(item["completion_time_delta_s"]) > 0.02
            or abs(item["path_length_delta_m"]) > 0.005
            or abs(item["path_curvature_delta_radpm"]) > 0.002
            for item in differences
        )
        report = {
            "test": "C",
            "sensitive": sensitive,
            "baseline": baseline["candidate"],
            "comparison": comparison,
            "differences": differences,
        }
        atomic_write_json(self.directory / "test_c.json", report)
        if not sensitive:
            raise RuntimeError("Test C found no planner/evaluator parameter sensitivity")
        return report

    def test_d(self) -> dict[str, Any]:
        test_c_path = self.directory / "test_c.json"
        if not test_c_path.is_file() or not json.loads(test_c_path.read_text())["sensitive"]:
            raise RuntimeError("Test D is gated on a successful Test C")
        try:
            import cma
        except ImportError as error:
            raise RuntimeError(
                "pycma is missing; install tools/cmaes_tuning/requirements.txt"
            ) from error
        cma_config = self.config["cma"]
        options = {
            "bounds": [0.0, 1.0],
            "popsize": int(cma_config["population_size"]),
            "seed": int(cma_config["seed"]),
            "verbose": -9,
            "verb_disp": 0,
        }
        strategy = cma.CMAEvolutionStrategy(
            self.parameter_space.baseline_z(), float(cma_config["sigma0"]), options
        )
        generation_results = []
        best: tuple[float, Path, str] | None = None
        for generation in range(int(cma_config["generations"])):
            candidates = strategy.ask()
            fitnesses = []
            candidate_results = []
            for index, z in enumerate(candidates):
                clipped = [max(0.0, min(1.0, float(value))) for value in z]
                candidate_id = f"gen_{generation:03d}_cand_{index:03d}"
                yaml_path = self.candidate_yaml(candidate_id, clipped)
                result = self.run_candidate(candidate_id, yaml_path, self.scenarios, generation)
                fitness = float(result["aggregate"]["fitness"])
                fitnesses.append(fitness)
                candidate_results.append(result)
                if best is None or fitness < best[0]:
                    best = fitness, yaml_path, candidate_id
            strategy.tell(candidates, fitnesses)
            generation_payload = {
                "generation": generation,
                "fitnesses": fitnesses,
                "candidate_ids": [result["candidate_id"] for result in candidate_results],
                "mean": [float(value) for value in strategy.mean],
                "sigma": float(strategy.sigma),
            }
            generation_results.append(generation_payload)
            atomic_pickle(self.directory / "checkpoints" / "cma_state.pkl", strategy)
            atomic_write_json(
                self.directory / "checkpoints" / "cma_state.json",
                {
                    "schema": "cmaes_checkpoint/1",
                    "completed_generation": generation,
                    "mean": generation_payload["mean"],
                    "sigma": generation_payload["sigma"],
                    "best_candidate_id": best[2] if best else None,
                    "best_fitness": best[0] if best else None,
                },
            )
        if best is None:
            raise RuntimeError("CMA smoke produced no candidate")
        best_destination = self.directory / "best_so_far.yaml"
        shutil.copy2(best[1], best_destination)
        report = {
            "test": "D",
            "population_size": int(cma_config["population_size"]),
            "generations": int(cma_config["generations"]),
            "training_scenarios": len(self.scenarios),
            "best_fitness": best[0],
            "best_candidate_id": best[2],
            "best_yaml": str(best_destination),
            "generation_results": generation_results,
            "checkpoint_pickle": str(self.directory / "checkpoints" / "cma_state.pkl"),
        }
        atomic_write_json(self.directory / "test_d.json", report)
        return report

    def collision_audit(self) -> dict[str, Any]:
        """Re-evaluate existing bags without running CMA or launching ROS."""
        rows = []
        skipped_attempts = []
        for status_path in sorted(
            (self.directory / "episodes").glob("*/train_*/attempt_*/runner_status.json")
        ):
            attempt_directory = status_path.parent
            status = json.loads(status_path.read_text(encoding="utf-8"))
            if status.get("infrastructure_failures"):
                skipped_attempts.append(str(attempt_directory))
                continue
            scenario_id = str(status["scenario_id"])
            scenario_path = (
                self.directory / "scenarios" / "train" / scenario_id / "manifest.json"
            )
            result = evaluate_episode(
                attempt_directory / "bag",
                scenario_path,
                attempt_directory / "rollout_summary.json",
                status_path,
                self.config,
                attempt_directory / "episode_result.json",
            )
            failure = result.get("failure", {})
            collision_diagnostics = result.get("collision_diagnostics", {})
            external_diagnostics = collision_diagnostics.get("external", {})
            yaw_repair = collision_diagnostics.get("odom_yaw_reset_repair", {})
            topic_timestamp = collision_diagnostics.get(
                "topic_first_true_timestamp_ns"
            )
            external_timestamp = external_diagnostics.get(
                "first_collision_timestamp_ns"
            )
            reconstructed_timestamp = yaw_repair.get("event_timestamp_ns")
            rows.append(
                {
                    "candidate_id": status_path.parts[-4],
                    "scenario_id": scenario_id,
                    "attempt": status_path.parent.name,
                    "valid": bool(result.get("valid", False)),
                    "collision_topic": bool(failure.get("collision_topic", False)),
                    "external_collision": bool(
                        failure.get("external_collision", False)
                    ),
                    "agreement": bool(failure.get("collision_agreement", False)),
                    "external_cause": external_diagnostics.get("cause"),
                    "external_source": collision_diagnostics.get("external_source"),
                    "topic_first_collision_timestamp_ns": topic_timestamp,
                    "external_first_collision_timestamp_ns": external_timestamp,
                    "external_minus_topic_ms": (
                        (external_timestamp - topic_timestamp) * 1.0e-6
                        if external_timestamp is not None and topic_timestamp is not None
                        else None
                    ),
                    "reconstructed_frame_minus_topic_ms": (
                        (reconstructed_timestamp - topic_timestamp) * 1.0e-6
                        if reconstructed_timestamp is not None
                        and topic_timestamp is not None
                        else None
                    ),
                    "external_minus_reconstructed_frame_ms": (
                        (external_timestamp - reconstructed_timestamp) * 1.0e-6
                        if external_timestamp is not None
                        and reconstructed_timestamp is not None
                        else None
                    ),
                    "yaw_reset_repaired": bool(
                        yaw_repair.get("detected", False)
                    ),
                }
            )
        mismatches = [row for row in rows if not row["agreement"]]
        collision_frame_deltas = [
            abs(float(row["external_minus_topic_ms"]))
            for row in rows
            if row["collision_topic"] and row["external_minus_topic_ms"] is not None
        ]
        reconstructed_frame_deltas = [
            abs(float(row["reconstructed_frame_minus_topic_ms"]))
            for row in rows
            if row["reconstructed_frame_minus_topic_ms"] is not None
        ]
        report = {
            "schema": "cmaes_collision_audit/1",
            "episode_count": len(rows),
            "agreement_count": len(rows) - len(mismatches),
            "mismatch_count": len(mismatches),
            "all_agree": not mismatches,
            "topic_collision_count": sum(row["collision_topic"] for row in rows),
            "external_collision_count": sum(row["external_collision"] for row in rows),
            "yaw_reset_repair_count": sum(row["yaw_reset_repaired"] for row in rows),
            "maximum_collision_frame_delta_ms": max(collision_frame_deltas, default=0.0),
            "maximum_external_topic_delta_ms": max(
                collision_frame_deltas, default=0.0
            ),
            "maximum_reconstructed_frame_topic_delta_ms": max(
                reconstructed_frame_deltas, default=0.0
            ),
            "skipped_infrastructure_attempts": skipped_attempts,
            "mismatches": mismatches,
            "episodes": rows,
        }
        atomic_write_json(self.directory / "collision_agreement.json", report)
        return report

    def run_all(self) -> dict[str, Any]:
        test_a = self.test_a()
        test_b = self.test_b()
        test_c = self.test_c()
        test_d = self.test_d()
        report = {"test_a": test_a, "test_b": test_b, "test_c": test_c, "test_d": test_d}
        atomic_write_json(self.directory / "smoke_report.json", report)
        return report
