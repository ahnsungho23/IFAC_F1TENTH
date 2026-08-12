#!/usr/bin/env python3
"""CLI for deterministic static-obstacle CMA-ES smoke testing."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys

TOOL_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOL_ROOT))

from cmaes_tuning.configuration import (
    load_config,
    resolve_workspace_path,
    safe_identifier,
    workspace_root_from_tool,
)
from cmaes_tuning.repeatability_audit import RepeatabilityAudit
from cmaes_tuning.scenario_generator import generate_fixed_scenarios
from cmaes_tuning.smoke_experiment import SmokeExperiment


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument(
        "--config",
        default=str(TOOL_ROOT / "config" / "tuning_config.yaml"),
        help="Tuning config YAML",
    )
    result.add_argument(
        "--experiment-id",
        default=f"smoke_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
        help="Stable experiment directory name; reuse it to resume cached episodes",
    )
    result.add_argument(
        "--localization-mode",
        choices=("mcl", "ground_truth"),
        help="Override experiment.localization_mode and freeze it in the experiment",
    )
    result.add_argument(
        "--simulator-seed",
        type=int,
        help="Override simulator_runtime.simulator_seed",
    )
    result.add_argument(
        "--scan-noise-std",
        type=float,
        help="Override simulator_runtime.scan_noise_std_m",
    )
    result.add_argument(
        "--scan-publication-mode",
        choices=("legacy_republish", "fresh_only"),
        help="Override simulator_runtime.scan_publication_mode",
    )
    result.add_argument(
        "--baseline-only",
        action="store_true",
        help="Run only the hand-tuned baseline in repeatability-audit",
    )
    result.add_argument(
        "--timing-diagnostics",
        action="store_true",
        help="Enable passive T0-T9 monotonic companion events",
    )
    result.add_argument(
        "--state-machine-publish-rate-hz",
        type=float,
        help="Tuning-only effective state timer rate while timing diagnostics are enabled",
    )
    result.add_argument(
        "--scenario-source-experiment",
        help="Reuse exact training/validation manifests and baked maps from this experiment",
    )
    result.add_argument(
        "--previous-mcl-experiment",
        help="MCL repeatability experiment to include as the comparison baseline",
    )
    result.add_argument(
        "--repetitions",
        type=int,
        help="Override repeatability repetitions (used for a one-run localization pilot)",
    )
    result.add_argument(
        "--representative-scenario-ids",
        help="Comma-separated exact training scenario IDs for a focused audit",
    )
    subcommands = result.add_subparsers(dest="command", required=True)
    subcommands.add_parser("generate-scenarios")
    subcommands.add_parser("test-a")
    subcommands.add_parser("test-b")
    subcommands.add_parser("test-c")
    subcommands.add_parser("test-d")
    subcommands.add_parser("collision-audit")
    subcommands.add_parser("generate-dataset")
    subcommands.add_parser("scenario-audit")
    subcommands.add_parser("repeatability-audit")
    subcommands.add_parser("all")
    return result


def main() -> int:
    arguments = parser().parse_args()
    workspace_root = workspace_root_from_tool()
    config_path = Path(arguments.config).resolve()
    config = load_config(config_path)
    if arguments.localization_mode:
        config["experiment"]["localization_mode"] = arguments.localization_mode
    if arguments.simulator_seed is not None:
        if arguments.simulator_seed < 0:
            raise ValueError("--simulator-seed must be non-negative")
        config["simulator_runtime"]["simulator_seed"] = arguments.simulator_seed
    if arguments.scan_noise_std is not None:
        if arguments.scan_noise_std < 0.0:
            raise ValueError("--scan-noise-std must be non-negative")
        config["simulator_runtime"]["scan_noise_std_m"] = arguments.scan_noise_std
    if arguments.scan_publication_mode:
        config["simulator_runtime"]["scan_publication_mode"] = (
            arguments.scan_publication_mode
        )
    if arguments.baseline_only:
        config["repeatability"]["candidate_names"] = ["baseline"]
    if arguments.timing_diagnostics:
        config.setdefault("timing_audit", {})["enabled"] = True
    if arguments.state_machine_publish_rate_hz is not None:
        if arguments.state_machine_publish_rate_hz <= 0.0:
            raise ValueError("--state-machine-publish-rate-hz must be positive")
        config.setdefault("timing_audit", {})[
            "state_machine_publish_rate_hz"
        ] = arguments.state_machine_publish_rate_hz
    if arguments.scenario_source_experiment:
        config["repeatability"]["scenario_dataset_source_experiment"] = (
            arguments.scenario_source_experiment
        )
    if arguments.previous_mcl_experiment:
        config["repeatability"]["previous_mcl_experiment"] = (
            arguments.previous_mcl_experiment
        )
    if arguments.repetitions is not None:
        if arguments.repetitions <= 0:
            raise ValueError("--repetitions must be positive")
        config["repeatability"]["repetitions"] = arguments.repetitions
    if arguments.representative_scenario_ids:
        scenario_ids = [
            item.strip()
            for item in arguments.representative_scenario_ids.split(",")
            if item.strip()
        ]
        config["repeatability"]["representative_scenario_ids"] = scenario_ids
        config["repeatability"]["scenario_count"] = len(scenario_ids)
    experiment_id = safe_identifier(arguments.experiment_id)
    if arguments.command == "generate-scenarios":
        output = (
            resolve_workspace_path(config["paths"]["output_root"], workspace_root)
            / experiment_id
        )
        config["paths"]["output_root"] = str(output)
        paths = generate_fixed_scenarios(config, workspace_root)
        print(json.dumps([str(path) for path in paths], indent=2))
        return 0
    if arguments.command in {
        "generate-dataset",
        "scenario-audit",
        "repeatability-audit",
    }:
        audit = RepeatabilityAudit(
            config, workspace_root, config_path, experiment_id
        )
        if arguments.command == "generate-dataset":
            report = {
                split: [str(path) for path in paths]
                for split, paths in audit.datasets.items()
            }
        elif arguments.command == "scenario-audit":
            report = audit.dataset_audit
        else:
            report = audit.run()
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0
    experiment = SmokeExperiment(config, workspace_root, config_path, experiment_id)
    if arguments.command == "test-a":
        report = experiment.test_a()
    elif arguments.command == "test-b":
        report = experiment.test_b()
    elif arguments.command == "test-c":
        report = experiment.test_c()
    elif arguments.command == "test-d":
        report = experiment.test_d()
    elif arguments.command == "collision-audit":
        report = experiment.collision_audit()
    else:
        report = experiment.run_all()
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
