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

from cmaes_tuning.configuration import load_config, safe_identifier, workspace_root_from_tool
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
    subcommands = result.add_subparsers(dest="command", required=True)
    subcommands.add_parser("generate-scenarios")
    subcommands.add_parser("test-a")
    subcommands.add_parser("test-b")
    subcommands.add_parser("test-c")
    subcommands.add_parser("test-d")
    subcommands.add_parser("all")
    return result


def main() -> int:
    arguments = parser().parse_args()
    workspace_root = workspace_root_from_tool()
    config_path = Path(arguments.config).resolve()
    config = load_config(config_path)
    experiment_id = safe_identifier(arguments.experiment_id)
    if arguments.command == "generate-scenarios":
        output = Path(config["paths"]["output_root"]).resolve() / experiment_id
        config["paths"]["output_root"] = str(output)
        paths = generate_fixed_scenarios(config, workspace_root)
        print(json.dumps([str(path) for path in paths], indent=2))
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
    else:
        report = experiment.run_all()
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
