#!/usr/bin/env python3
"""OAT probe of four excluded P3 parameters on the frozen Q2 failure case."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any

import yaml


TOOL_ROOT = Path(__file__).resolve().parent
ROOT = TOOL_ROOT.parents[1]
sys.path.insert(0, str(TOOL_ROOT))

from cmaes_tuning.configuration import load_config  # noqa: E402
from cmaes_tuning.schemas import atomic_write_json  # noqa: E402
from p3_production_cma_smoke import preflight, write_scenario  # noqa: E402


TASK_ROOT = ROOT / "runs/cmaes_tuning/p3_cma_parameter_space_blocker_fix_v1"
BASELINE_YAML = ROOT / "src/local_planning/config/local_planning.yaml"
LEGACY_SPACE = TOOL_ROOT / "config/parameter_space.yaml"


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _sha256(path: Path) -> str:
    return _sha256_bytes(path.read_bytes())


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _parameter_contract() -> dict[str, dict[str, Any]]:
    document = yaml.safe_load(LEGACY_SPACE.read_text(encoding="utf-8"))
    selected = {
        "minimum_target_offset_m",
        "obstacle_longitudinal_padding_m",
        "safety_margin_m",
        "wall_safety_margin_m",
    }
    result = {item["name"]: dict(item) for item in document["parameters"] if item["name"] in selected}
    if set(result) != selected:
        raise RuntimeError("legacy parameter contract lacks required OAT parameters")
    return result


def _write_variant(path: Path, parameter: str | None, value: float | None) -> None:
    document = yaml.safe_load(BASELINE_YAML.read_text(encoding="utf-8"))
    parameters = document["local_planner_node"]["ros__parameters"]
    if parameter is not None:
        parameters[parameter] = float(value)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")


def _first_invalidation(log_path: Path) -> dict[str, Any] | None:
    marker = "P3_LIFECYCLE_INVALIDATION "
    for line in log_path.read_text(encoding="utf-8", errors="replace").splitlines():
        if marker in line:
            return json.loads(line.split(marker, 1)[1])
    return None


def _run_variant(
    *, identifier: str, parameter: str | None, value: float | None,
    domain: int, output: Path, scenario: Path, config: Path,
) -> dict[str, Any]:
    root = output / "variants" / identifier
    candidate = root / "candidate.yaml"
    _write_variant(candidate, parameter, value)
    episode = root / "episode"
    environment = dict(os.environ)
    environment.update({
        "ROS_DOMAIN_ID": str(domain),
        "FASTDDS_BUILTIN_TRANSPORTS": "UDPv4",
        "MPLCONFIGDIR": "/tmp/cmaes_matplotlib",
    })
    command = [
        sys.executable, str(TOOL_ROOT / "lockstep_episode.py"),
        "--workspace", str(ROOT), "--config", str(config),
        "--candidate", str(candidate), "--scenario", str(scenario),
        "--output", str(episode), "--simulator-seed", "12345",
        "--scan-noise-std", "0.01", "--post-obstacle-distance", "3.0",
        "--maximum-duration", "8.0", "--dds-transport", "UDPv4",
        "--p3-mode", "TEST_ACTIVE",
    ]
    completed = subprocess.run(
        command, cwd=ROOT, env=environment, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False,
    )
    root.mkdir(parents=True, exist_ok=True)
    (root / "stdout.log").write_text(completed.stdout, encoding="utf-8")
    result_path = episode / "lockstep_result.json"
    episode_path = episode / "episode_result.json"
    if completed.returncode != 0 or not result_path.is_file() or not episode_path.is_file():
        raise RuntimeError(f"{identifier} infrastructure failure returncode={completed.returncode}")
    result = json.loads(result_path.read_text(encoding="utf-8"))
    metrics = json.loads(episode_path.read_text(encoding="utf-8"))
    observation = _first_invalidation(episode / "logs/local_planning.log")
    envelopes = (
        json.dumps(
            observation["same_id_envelopes"], sort_keys=True, separators=(",", ":")
        ).encode("utf-8")
        if observation is not None else None
    )
    record = {
        "identifier": identifier,
        "parameter": parameter,
        "value": value,
        "candidate_yaml_sha256": _sha256(candidate),
        "original_candidate_identity": (
            observation["original_candidate_identity"] if observation else None
        ),
        "original_path_digest": observation["original_path_digest"] if observation else None,
        "lifecycle_reason": (
            observation["lifecycle_reason"] if observation else "NO_INVALIDATION_OBSERVED"
        ),
        "guarded_validator": (
            observation["guarded_validator"] if observation else {
                "attempted": False, "hard_valid": None, "rejection": "NOT_APPLICABLE"
            }
        ),
        "raw_validator": (
            observation["raw_validator"] if observation else {
                "attempted": False, "hard_valid": None, "rejection": "NOT_APPLICABLE"
            }
        ),
        "same_id_envelopes_sha256": _sha256_bytes(envelopes) if envelopes else None,
        "first_invalidation_callback": observation["callback_sequence"] if observation else None,
        "p3_selected_count": result["p3_selected_count"],
        "p3_continuation_count": result["p3_continuation_count"],
        "p3_completion_count": result["p3_completion_count"],
        "invalid_suffix": result["invalid_suffix"],
        "safe_stop": result["safe_stop"],
        "p0_fallback": result["p0_fallback"],
        "scenario_success": result["scenario_success"],
        "minimum_obstacle_clearance_m": metrics["metrics"]["minimum_obstacle_clearance_m"],
        "minimum_wall_clearance_m": metrics["metrics"]["minimum_wall_clearance_m"],
        "artifact_root": str(root),
    }
    atomic_write_json(root / "oat_result.json", record)
    return record


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=TASK_ROOT / ".evaluation/oat")
    parser.add_argument(
        "--config", type=Path, default=TOOL_ROOT / "config/tuning_config.yaml"
    )
    parser.add_argument("--domain-start", type=int, default=221)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    inputs = preflight()
    contracts = _parameter_contract()
    probes = [
        ("baseline", None, None),
        ("minimum_target_offset_lower", "minimum_target_offset_m", float(contracts["minimum_target_offset_m"]["lower"])),
        ("minimum_target_offset_upper", "minimum_target_offset_m", float(contracts["minimum_target_offset_m"]["upper"])),
        ("obstacle_padding_conservative", "obstacle_longitudinal_padding_m", 0.4649924657737441),
        ("safety_margin_conservative", "safety_margin_m", 0.01978925429952077),
        ("wall_margin_conservative", "wall_safety_margin_m", 0.045),
    ]
    preflight_result = {
        "schema": "p3_cma_blocker_oat_preflight/1",
        "status": "PASS",
        "configuration": "Q2_COMPETITION_STYLE",
        "probe_count": len(probes),
        "parallel_workers": 3,
        "safety_authority_lowering_executed": False,
        "trajectory_executed": False,
    }
    if not args.execute:
        print(json.dumps(preflight_result, sort_keys=True))
        return 0

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    atomic_write_json(output / "preflight.json", preflight_result)
    scenario = write_scenario(inputs, output, load_config(args.config.resolve()))
    records: dict[str, dict[str, Any]] = {}
    pending = list(probes)
    attempt = 0
    infrastructure_failures = []
    while pending:
        retry = []
        attempt_root = output / "attempts" / f"attempt_{attempt:03d}"
        for chunk_start in range(0, len(pending), 3):
            chunk = pending[chunk_start:chunk_start + 3]
            with ThreadPoolExecutor(max_workers=3) as pool:
                futures = {
                    pool.submit(
                        _run_variant,
                        identifier=identifier, parameter=parameter, value=value,
                        domain=args.domain_start + index, output=attempt_root,
                        scenario=scenario, config=args.config.resolve(),
                    ): (identifier, parameter, value)
                    for index, (identifier, parameter, value) in enumerate(chunk)
                }
                for future in as_completed(futures):
                    probe = futures[future]
                    try:
                        records[probe[0]] = future.result()
                    except RuntimeError as error:
                        retry.append(probe)
                        infrastructure_failures.append({
                            "attempt": attempt, "identifier": probe[0], "error": str(error)
                        })
        pending = retry
        attempt += 1
        if pending and attempt >= 3:
            raise RuntimeError(f"OAT infrastructure retry budget exceeded: {pending}")

    baseline = records["baseline"]
    classifications = []
    variant_names = {
        "minimum_target_offset_m": ["minimum_target_offset_lower", "minimum_target_offset_upper"],
        "obstacle_longitudinal_padding_m": ["obstacle_padding_conservative"],
        "safety_margin_m": ["safety_margin_conservative"],
        "wall_safety_margin_m": ["wall_margin_conservative"],
    }
    authority = {
        "minimum_target_offset_m": "PERFORMANCE_TUNABLE_CANDIDATE",
        "obstacle_longitudinal_padding_m": "STRUCTURAL_OBSTACLE_SAFETY_AUTHORITY",
        "safety_margin_m": "EXACT_VALIDATOR_OBSTACLE_SAFETY_AUTHORITY",
        "wall_safety_margin_m": "EXACT_VALIDATOR_TRACK_SAFETY_AUTHORITY",
    }
    for name, identifiers in variant_names.items():
        variants = [records[identifier] for identifier in identifiers]
        path_responsive = any(
            item["original_path_digest"] != baseline["original_path_digest"]
            or item["original_candidate_identity"] != baseline["original_candidate_identity"]
            for item in variants
        )
        invalidation_responsive = any(
            item["lifecycle_reason"] != baseline["lifecycle_reason"]
            or item["raw_validator"] != baseline["raw_validator"]
            or item["first_invalidation_callback"] != baseline["first_invalidation_callback"]
            for item in variants
        )
        safety_authority = name != "minimum_target_offset_m"
        classifications.append({
            "name": name,
            "authority": authority[name],
            "path_geometry_responsive": path_responsive,
            "first_raw_invalidation_responsive": invalidation_responsive,
            "include_in_expanded_whitelist": (
                name == "minimum_target_offset_m" and path_responsive
            ),
            "variant_ids": identifiers,
            "safety_authority": safety_authority,
        })
    included = [item["name"] for item in classifications if item["include_in_expanded_whitelist"]]
    result = {
        "schema": "p3_cma_parameter_space_blocker_oat/1",
        "created_at_utc": _utc_now(),
        "status": "PASS",
        "method": "CURRENT_Q2_FAILURE_BASELINE_AND_ONE_AT_A_TIME_PRODUCTION_EPISODES",
        "baseline": baseline,
        "variants": [records[item[0]] for item in probes[1:]],
        "classifications": classifications,
        "included_performance_tunables": included,
        "safety_authority_parameters_excluded": [
            item["name"] for item in classifications if item["safety_authority"]
        ],
        "production_source_modified": False,
        "hard_safety_relaxed": False,
        "infrastructure_failures_excluded_from_results": infrastructure_failures,
    }
    atomic_write_json(output / "oat_summary.json", result)
    print(json.dumps({
        "status": "PASS", "included": included,
        "classifications": classifications,
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
