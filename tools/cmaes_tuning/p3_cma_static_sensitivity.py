#!/usr/bin/env python3
"""Measure whether planner parameters change production P3/M1 output.

The script mutates only temporary copies of the four frozen parity streams and
executes the production-linked parity harness.  It never calls the historical
external evaluator and never changes a ROS parameter or production source file.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import subprocess
import tempfile
from typing import Callable


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BINARY = ROOT / "build/local_planning/p3_production_parity_harness"
STREAM_ROOT = Path(
    "/tmp/ifac_planner_result_source_snapshot_publication_guard_fix/"
    "runs/cmaes_tuning/p3_replacement_check_v1/.evaluation"
)
STREAMS = (
    STREAM_ROOT / "static_streams/replacement_q2_013__cluster_01.stream.tsv",
    STREAM_ROOT / "static_streams/replacement_finals_023__cluster_02.stream.tsv",
    STREAM_ROOT / "sequential_streams/replacement_q2_012__seq_12.stream.tsv",
    STREAM_ROOT / "sequential_streams/replacement_finals_009__seq_05.stream.tsv",
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _replace_scalar(lines: list[str], name: str, value: float) -> None:
    prefix = f"PARAM\t{name}\t"
    matches = [index for index, line in enumerate(lines) if line.startswith(prefix)]
    if len(matches) != 1:
        raise RuntimeError(f"expected one scalar {name}, found {len(matches)}")
    lines[matches[0]] = f"PARAM\t{name}\t{value:.17g}"


def _vector(lines: list[str], name: str) -> tuple[int, list[float]]:
    prefix = f"PARAMV\t{name}\t"
    matches = [index for index, line in enumerate(lines) if line.startswith(prefix)]
    if len(matches) != 1:
        raise RuntimeError(f"expected one vector {name}, found {len(matches)}")
    tokens = lines[matches[0]].split("\t")
    count = int(tokens[2])
    values = [float(token) for token in tokens[3:]]
    if len(values) != count:
        raise RuntimeError(f"malformed vector {name}")
    return matches[0], values


def _replace_vector(lines: list[str], name: str, values: list[float]) -> None:
    index, _ = _vector(lines, name)
    payload = "\t".join(f"{value:.17g}" for value in values)
    lines[index] = f"PARAMV\t{name}\t{len(values)}\t{payload}"


def scalar(name: str) -> Callable[[list[str], float], None]:
    return lambda lines, value: _replace_scalar(lines, name, value)


def vector_index(name: str, index: int) -> Callable[[list[str], float], None]:
    def update(lines: list[str], value: float) -> None:
        _, values = _vector(lines, name)
        values[index] = value
        _replace_vector(lines, name, values)
    return update


def thirds(name: str, descending: bool) -> Callable[[list[str], float], None]:
    def update(lines: list[str], value: float) -> None:
        values = [value / 3.0, 2.0 * value / 3.0, value]
        if descending:
            values.reverse()
        _replace_vector(lines, name, values)
    return update


PARAMETERS = (
    # The original ten-dimensional CMA space.
    ("safety_margin_m", 0.014789254299520768, 0.005, scalar("safety_margin_m"),
     "USED_SAFETY_EXCLUDED"),
    ("obstacle_longitudinal_padding_m", 0.4149924657737441, 0.05,
     scalar("obstacle_longitudinal_padding_m"), "USED_STRUCTURAL_EXCLUDED"),
    ("pre_apex_far_m", 9.669467393145736, 0.50,
     thirds("pre_apex_distances_m", True), "USED_PERFORMANCE_CANDIDATE"),
    ("post_apex_far_m", 5.107334552832739, 0.50,
     thirds("post_apex_distances_m", False), "USED_PERFORMANCE_CANDIDATE"),
    ("transition_short", 0.2740569240066383, 0.05,
     vector_index("transition_distance_scales", 0), "USED_PERFORMANCE_CANDIDATE"),
    ("transition_middle", 0.6991537701867223, 0.05,
     vector_index("transition_distance_scales", 1), "P0_ONLY_NO_PRODUCTION_P3_EFFECT"),
    ("transition_long", 3.5816012944405027, 0.25,
     vector_index("transition_distance_scales", 2), "USED_PERFORMANCE_CANDIDATE"),
    ("outside_line_transition_scale", 0.1, 0.05,
     scalar("outside_line_transition_scale"), "USED_PERFORMANCE_CANDIDATE"),
    ("minimum_target_offset_m", 0.15, 0.02,
     scalar("minimum_target_offset_m"), "USED_PERFORMANCE_CANDIDATE"),
    ("wall_safety_margin_m", 0.04, 0.005, scalar("wall_safety_margin_m"),
     "USED_SAFETY_EXCLUDED"),
    # Additional production-P3 inputs considered, but not automatically whitelisted.
    ("entry_fraction_short", 0.50, 0.05,
     vector_index("entry_transition_fractions", 0), "NEW_PERFORMANCE_CANDIDATE"),
    ("entry_fraction_middle", 0.75, 0.05,
     vector_index("entry_transition_fractions", 1), "NEW_CANDIDATE_IF_SENSITIVE"),
    ("entry_fraction_long", 1.00, 0.05,
     vector_index("entry_transition_fractions", 2), "NEW_CANDIDATE_IF_SENSITIVE"),
    ("maximum_target_offset_m", 1.50, 0.10, scalar("maximum_target_offset_m"),
     "NEW_CANDIDATE_IF_SENSITIVE"),
    ("target_d_candidate_count", 5.0, 1.0, scalar("target_d_candidate_count"),
     "DISCRETE_CANDIDATE_IF_SENSITIVE"),
    ("post_merge_lookahead_m", 5.0, 0.50, scalar("post_merge_lookahead_m"),
     "LIFECYCLE_STRUCTURAL_EXCLUDED"),
    ("post_merge_min_time_sec", 1.0, 0.10, scalar("post_merge_min_time_sec"),
     "LIFECYCLE_STRUCTURAL_EXCLUDED"),
)


def run_harness(binary: Path, streams: list[Path]) -> str:
    completed = subprocess.run(
        [str(binary), *(str(path) for path in streams)],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if completed.returncode != 0:
        raise RuntimeError(f"parity harness failed: {completed.stderr.strip()}")
    return completed.stdout


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=DEFAULT_BINARY)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    binary = args.binary.resolve()
    if not binary.is_file():
        raise FileNotFoundError(binary)
    for stream in STREAMS:
        if not stream.is_file():
            raise FileNotFoundError(stream)

    baseline_output = run_harness(binary, list(STREAMS))
    records = []
    with tempfile.TemporaryDirectory(prefix="p3_cma_sensitivity_") as temporary:
        temporary_root = Path(temporary)
        for name, nominal, delta, mutate, classification in PARAMETERS:
            variants = []
            for direction, value in (("minus", nominal - delta), ("plus", nominal + delta)):
                mutated_streams = []
                for source in STREAMS:
                    lines = source.read_text(encoding="utf-8").splitlines()
                    mutate(lines, value)
                    destination = temporary_root / f"{name}_{direction}_{source.name}"
                    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")
                    mutated_streams.append(destination)
                output = run_harness(binary, mutated_streams)
                variants.append({
                    "direction": direction,
                    "value": value,
                    "output_sha256": hashlib.sha256(output.encode("utf-8")).hexdigest(),
                    "changed_from_nominal": output != baseline_output,
                    "changed_line_count": sum(
                        first != second
                        for first, second in itertools.zip_longest(
                            baseline_output.splitlines(), output.splitlines(), fillvalue=None
                        )
                    ),
                })
            sensitive = any(item["changed_from_nominal"] for item in variants)
            records.append({
                "name": name,
                "nominal": nominal,
                "delta": delta,
                "source_classification": classification,
                "static_output_sensitive": sensitive,
                "variants": variants,
            })

    result = {
        "schema": "production_p3_cma_static_sensitivity/1",
        "method": "FOUR_FROZEN_STREAMS_NOMINAL_PLUS_MINUS_DELTA_PRODUCTION_LINKED_HARNESS",
        "binary": str(binary),
        "binary_sha256": sha256(binary),
        "stream_sha256": {str(path): sha256(path) for path in STREAMS},
        "baseline_output_sha256": hashlib.sha256(baseline_output.encode("utf-8")).hexdigest(),
        "parameters": records,
        "finite_results": all(
            math.isfinite(float(record["nominal"])) and math.isfinite(float(record["delta"]))
            for record in records
        ),
        "trajectory_executed": False,
        "external_evaluator_executed": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({
        "output": str(args.output),
        "sensitive": [record["name"] for record in records if record["static_output_sensitive"]],
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
