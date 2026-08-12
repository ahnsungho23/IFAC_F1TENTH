#!/usr/bin/env python3
"""Run the deterministic perception/planner record-replay audit; never runs CMA."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys

TOOL_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOL_ROOT))

from cmaes_tuning.configuration import load_config, safe_identifier, workspace_root_from_tool
from cmaes_tuning.deterministic_replay import DeterministicReplayAudit


def _scenario_repeats(value: str) -> dict[str, int]:
    result: dict[str, int] = {}
    for item in value.split(","):
        scenario_id, separator, repeat = item.strip().partition("=")
        if not separator or not scenario_id:
            raise argparse.ArgumentTypeError(
                "scenario sources must look like training_001=0,training_009=0"
            )
        result[scenario_id] = int(repeat)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default=str(TOOL_ROOT / "config" / "tuning_config.yaml"),
    )
    parser.add_argument("--source-experiment", required=True)
    parser.add_argument(
        "--scenario-sources",
        type=_scenario_repeats,
        default=_scenario_repeats("training_001=0,training_009=0"),
    )
    parser.add_argument("--repetitions", type=int, default=10)
    parser.add_argument("--domain-id-start", type=int, default=181)
    parser.add_argument(
        "--experiment-id",
        default=f"deterministic_replay_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
    )
    arguments = parser.parse_args()
    workspace_root = workspace_root_from_tool()
    config = load_config(arguments.config)
    output = (
        workspace_root / "runs" / "cmaes_tuning" / safe_identifier(arguments.experiment_id)
    )
    audit = DeterministicReplayAudit(
        config=config,
        workspace_root=workspace_root,
        output_directory=output,
        source_experiment=arguments.source_experiment,
        scenario_repeats=arguments.scenario_sources,
        repetitions=arguments.repetitions,
        domain_id_start=arguments.domain_id_start,
    )
    print(json.dumps(audit.run(), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
