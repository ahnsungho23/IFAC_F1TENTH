#!/usr/bin/env python3
"""Frozen revision-2 analysis for the GQSC-S1 runtime qualification.

The script consumes only final-attempt identities declared in run_manifest.csv and reads the
frozen raw formats directly.  It uses Decimal for emitted microsecond values and never invokes an
interpolating quantile implementation.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from pathlib import Path
from typing import Iterable, Mapping, Sequence


WORKLOADS = (
    "W1_SCE018_STANDALONE",
    "W2_SCE018_ROS_NODE_TEST_ACTIVE",
    "W3_SCE018_FULL_STACK_CONTENTION",
)
CONDITIONS = ("A", "B")
REPEATS = (1, 2, 3, 4, 5)
PAIR_ORDER = {1: ("A", "B"), 2: ("B", "A"), 3: ("A", "B"), 4: ("B", "A"), 5: ("A", "B")}
EXPECTED_SCHEDULE = tuple(
    (workload, condition, repeat)
    for workload in WORKLOADS
    for repeat in REPEATS
    for condition in PAIR_ORDER[repeat]
)
DEADLINE_US = Decimal("25000")
TAIL_30_US = Decimal("30000")
TAIL_40_US = Decimal("40000")
STRONG_RELATIVE_CHANGE = Decimal("0.10")
TIMING_SCOPE_EQUIVALENT_W1_W2 = False


MANIFEST_COLUMNS = (
    "workload_id",
    "repeat_id",
    "condition",
    "attempt_id",
    "final_attempt",
    "run_valid",
    "parity_pass",
    "affinity_pass",
    "input_isolation_pass",
    "binary_identity_pass",
    "metric_contract_pass",
    "result_integrity_pass",
    "invalid_rejected_count",
    "raw_dir",
    "retry_reason",
)


def parse_bool(value: str, field: str) -> bool:
    normalized = value.strip().lower()
    if normalized == "true":
        return True
    if normalized == "false":
        return False
    raise ValueError(f"{field} must be true or false, got {value!r}")


def parse_decimal(value: object, field: str) -> Decimal:
    try:
        result = Decimal(str(value))
    except (InvalidOperation, ValueError) as error:
        raise ValueError(f"invalid decimal {field}: {value!r}") from error
    if not result.is_finite() or result < 0:
        raise ValueError(f"{field} must be finite and nonnegative, got {value!r}")
    return result


def nearest_rank(values: Sequence[Decimal], p: Decimal) -> Decimal:
    if not values:
        raise ValueError("nearest_rank requires at least one value")
    if not Decimal(0) < p <= Decimal(1):
        raise ValueError("quantile probability must be in (0, 1]")
    ordered = sorted(values)
    rank = int((p * len(ordered)).to_integral_value(rounding="ROUND_CEILING"))
    return ordered[rank - 1]


def even_median(values: Sequence[Decimal]) -> Decimal:
    if not values:
        raise ValueError("median requires at least one value")
    ordered = sorted(values)
    midpoint = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[midpoint]
    return (ordered[midpoint - 1] + ordered[midpoint]) / Decimal(2)


def population_mean_std(values: Sequence[Decimal]) -> tuple[Decimal, Decimal]:
    if not values:
        raise ValueError("mean/std requires at least one value")
    mean = sum(values, Decimal(0)) / Decimal(len(values))
    variance = sum(((value - mean) ** 2 for value in values), Decimal(0)) / Decimal(len(values))
    return mean, variance.sqrt()


@dataclass(frozen=True)
class Summary:
    n: int
    median_us: Decimal
    mean_us: Decimal
    std_us: Decimal
    p90_us: Decimal
    p95_us: Decimal
    p99_us: Decimal
    maximum_us: Decimal
    gt25_count: int
    gt30_count: int
    gt40_count: int

    @classmethod
    def from_values(cls, values: Sequence[Decimal]) -> "Summary":
        mean, std = population_mean_std(values)
        return cls(
            n=len(values),
            median_us=even_median(values),
            mean_us=mean,
            std_us=std,
            p90_us=nearest_rank(values, Decimal("0.90")),
            p95_us=nearest_rank(values, Decimal("0.95")),
            p99_us=nearest_rank(values, Decimal("0.99")),
            maximum_us=max(values),
            gt25_count=sum(value > DEADLINE_US for value in values),
            gt30_count=sum(value > TAIL_30_US for value in values),
            gt40_count=sum(value > TAIL_40_US for value in values),
        )


@dataclass(frozen=True)
class CellDecision:
    summary: Summary
    repeat_p95_us: tuple[Decimal, ...]
    repeat_pass_count: int
    deadline_pass: bool


def decide_cell(repeat_values: Mapping[int, Sequence[Decimal]]) -> CellDecision:
    if tuple(sorted(repeat_values)) != REPEATS:
        raise ValueError("a workload-condition cell requires repeats 1..5")
    for repeat_id in REPEATS:
        if len(repeat_values[repeat_id]) != 200:
            raise ValueError(f"repeat {repeat_id} requires exactly 200 measured callbacks")
    repeat_p95 = tuple(
        nearest_rank(repeat_values[repeat_id], Decimal("0.95")) for repeat_id in REPEATS
    )
    pooled = [value for repeat_id in REPEATS for value in repeat_values[repeat_id]]
    pooled_summary = Summary.from_values(pooled)
    repeat_pass_count = sum(value <= DEADLINE_US for value in repeat_p95)
    return CellDecision(
        summary=pooled_summary,
        repeat_p95_us=repeat_p95,
        repeat_pass_count=repeat_pass_count,
        deadline_pass=pooled_summary.p95_us <= DEADLINE_US and repeat_pass_count >= 4,
    )


@dataclass(frozen=True)
class Contrast:
    contrast_id: str
    source: tuple[str, str]
    target: tuple[str, str]
    classification_eligible: bool
    improvement_count: int
    worsening_count: int
    direction: str
    relative_change: Decimal
    effect: str
    deadline_crossing: str


def compare_cells(
    contrast_id: str,
    source_key: tuple[str, str],
    target_key: tuple[str, str],
    cells: Mapping[tuple[str, str], CellDecision],
    classification_eligible: bool = True,
) -> Contrast:
    source = cells[source_key]
    target = cells[target_key]
    improvement_count = sum(
        target_value < source_value
        for source_value, target_value in zip(source.repeat_p95_us, target.repeat_p95_us)
    )
    worsening_count = sum(
        target_value > source_value
        for source_value, target_value in zip(source.repeat_p95_us, target.repeat_p95_us)
    )
    if improvement_count >= 4:
        direction = "REPEATABLE_IMPROVEMENT"
        relative_change = (
            (source.summary.p95_us - target.summary.p95_us) / source.summary.p95_us
            if source.summary.p95_us != 0
            else Decimal(0)
        )
    elif worsening_count >= 4:
        direction = "REPEATABLE_WORSENING"
        relative_change = (
            (target.summary.p95_us - source.summary.p95_us) / source.summary.p95_us
            if source.summary.p95_us != 0
            else Decimal("Infinity")
        )
    else:
        direction = "NO_REPEATABLE_DIRECTION"
        relative_change = Decimal(0)
    if source.deadline_pass and not target.deadline_pass and direction == "REPEATABLE_WORSENING":
        crossing = "PASS_TO_FAIL"
    elif not source.deadline_pass and target.deadline_pass and direction == "REPEATABLE_IMPROVEMENT":
        crossing = "FAIL_TO_PASS"
    else:
        crossing = "NONE"
    if crossing != "NONE":
        effect = "REPEATABLE_DEADLINE_CROSSING"
    elif direction != "NO_REPEATABLE_DIRECTION" and relative_change >= STRONG_RELATIVE_CHANGE:
        effect = "STRONG_" + direction.removeprefix("REPEATABLE_")
    else:
        effect = "NOT_STRONG"
    return Contrast(
        contrast_id=contrast_id,
        source=source_key,
        target=target_key,
        classification_eligible=classification_eligible,
        improvement_count=improvement_count,
        worsening_count=worsening_count,
        direction=direction,
        relative_change=relative_change,
        effect=effect,
        deadline_crossing=crossing,
    )


def build_contrasts(cells: Mapping[tuple[str, str], CellDecision]) -> dict[str, Contrast]:
    w1, w2, w3 = WORKLOADS
    return {
        "C1": compare_cells("C1", (w1, "A"), (w2, "A"), cells, TIMING_SCOPE_EQUIVALENT_W1_W2),
        "C2": compare_cells("C2", (w1, "B"), (w2, "B"), cells, TIMING_SCOPE_EQUIVALENT_W1_W2),
        "C3": compare_cells("C3", (w2, "A"), (w3, "A"), cells),
        "C4": compare_cells("C4", (w2, "B"), (w3, "B"), cells),
        "C5": compare_cells("C5", (w1, "A"), (w1, "B"), cells),
        "C6": compare_cells("C6", (w2, "A"), (w2, "B"), cells),
        "C7": compare_cells("C7", (w3, "A"), (w3, "B"), cells),
    }


def classify_environment(
    experiment_valid: bool,
    cells: Mapping[tuple[str, str], CellDecision],
    contrasts: Mapping[str, Contrast],
) -> str:
    if not experiment_valid:
        return "EXPERIMENT_INCONCLUSIVE"
    w1, w2, w3 = WORKLOADS
    confirmed = (
        (
            not cells[(w3, "A")].deadline_pass
            and cells[(w3, "B")].deadline_pass
            and contrasts["C7"].direction == "REPEATABLE_IMPROVEMENT"
        )
        or (
            cells[(w2, "A")].deadline_pass
            and not cells[(w3, "A")].deadline_pass
            and contrasts["C3"].direction == "REPEATABLE_WORSENING"
        )
        or (
            cells[(w2, "B")].deadline_pass
            and not cells[(w3, "B")].deadline_pass
            and contrasts["C4"].direction == "REPEATABLE_WORSENING"
        )
    )
    if TIMING_SCOPE_EQUIVALENT_W1_W2:
        confirmed = confirmed or (
            cells[(w1, "A")].deadline_pass
            and not cells[(w2, "A")].deadline_pass
            and contrasts["C1"].direction == "REPEATABLE_WORSENING"
        ) or (
            cells[(w1, "B")].deadline_pass
            and not cells[(w2, "B")].deadline_pass
            and contrasts["C2"].direction == "REPEATABLE_WORSENING"
        )
    if confirmed:
        return "RUNTIME_ENVIRONMENT_CONFIRMED"
    if any(
        contrast.classification_eligible and contrast.effect.startswith("STRONG_")
        for contrast in contrasts.values()
    ):
        return "RUNTIME_ENVIRONMENT_PARTIAL"
    return "RUNTIME_ENVIRONMENT_NOT_SUPPORTED"


@dataclass(frozen=True)
class ManifestAttempt:
    workload_id: str
    repeat_id: int
    condition: str
    attempt_id: str
    final_attempt: bool
    run_valid: bool
    gates_pass: bool
    invalid_rejected_count: int
    raw_dir: Path
    retry_reason: str


def read_manifest(path: Path) -> list[ManifestAttempt]:
    attempts: list[ManifestAttempt] = []
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != MANIFEST_COLUMNS:
            raise ValueError("run_manifest.csv columns do not match the frozen schema")
        for row in reader:
            workload = row["workload_id"]
            condition = row["condition"]
            repeat_id = int(row["repeat_id"])
            attempt_id = row["attempt_id"]
            if workload not in WORKLOADS or condition not in CONDITIONS or repeat_id not in REPEATS:
                raise ValueError(f"invalid scheduled identity: {workload}/{condition}/R{repeat_id}")
            if attempt_id not in ("ATTEMPT0", "RERUN1"):
                raise ValueError(f"invalid attempt id: {attempt_id}")
            gate_fields = (
                "parity_pass",
                "affinity_pass",
                "input_isolation_pass",
                "binary_identity_pass",
                "metric_contract_pass",
                "result_integrity_pass",
            )
            attempts.append(
                ManifestAttempt(
                    workload_id=workload,
                    repeat_id=repeat_id,
                    condition=condition,
                    attempt_id=attempt_id,
                    final_attempt=parse_bool(row["final_attempt"], "final_attempt"),
                    run_valid=parse_bool(row["run_valid"], "run_valid"),
                    gates_pass=all(parse_bool(row[field], field) for field in gate_fields),
                    invalid_rejected_count=int(row["invalid_rejected_count"]),
                    raw_dir=(path.parent / row["raw_dir"]).resolve(),
                    retry_reason=row["retry_reason"].strip(),
                )
            )
    return attempts


def validate_attempt_schedule(attempts: Sequence[ManifestAttempt]) -> tuple[dict[tuple[str, str, int], ManifestAttempt], list[str]]:
    errors: list[str] = []
    grouped: dict[tuple[str, str, int], list[ManifestAttempt]] = {}
    for attempt in attempts:
        key = (attempt.workload_id, attempt.condition, attempt.repeat_id)
        grouped.setdefault(key, []).append(attempt)
    expected = {(workload, condition, repeat) for workload in WORKLOADS for condition in CONDITIONS for repeat in REPEATS}
    if set(grouped) != expected:
        errors.append("manifest does not contain exactly the 30 scheduled identities")
    final: dict[tuple[str, str, int], ManifestAttempt] = {}
    observed_schedule: list[tuple[str, str, int]] = []
    for attempt in attempts:
        key = (attempt.workload_id, attempt.condition, attempt.repeat_id)
        if not observed_schedule or observed_schedule[-1] != key:
            observed_schedule.append(key)
    if tuple(observed_schedule) != EXPECTED_SCHEDULE:
        errors.append("manifest row order does not match the frozen 30-run schedule")
    for key, rows in grouped.items():
        if len(rows) > 2:
            errors.append(f"{key} has more than one allowed retry")
        ids = [row.attempt_id for row in rows]
        if len(ids) != len(set(ids)) or not ids or ids[0] != "ATTEMPT0":
            errors.append(f"{key} has an invalid attempt sequence")
        if len(ids) == 2 and ids != ["ATTEMPT0", "RERUN1"]:
            errors.append(f"{key} retry is not immediately after ATTEMPT0")
        if "RERUN1" in ids:
            initial = next(row for row in rows if row.attempt_id == "ATTEMPT0")
            if initial.run_valid or not initial.retry_reason:
                errors.append(f"{key} retry lacks a recorded failed initial attempt and reason")
        final_rows = [row for row in rows if row.final_attempt]
        if len(final_rows) != 1:
            errors.append(f"{key} must identify exactly one final attempt")
        else:
            final[key] = final_rows[0]
    for workload in WORKLOADS:
        retry_count = sum(
            any(row.attempt_id == "RERUN1" for row in rows)
            for key, rows in grouped.items()
            if key[0] == workload
        )
        if retry_count > 1:
            errors.append(f"{workload} has more than one repeat requiring a retry")
    return final, errors


def read_w1(path: Path) -> list[Decimal]:
    timing_path = path / "timing.tsv"
    rows: list[Decimal] = []
    seen_indices: list[int] = []
    with timing_path.open(encoding="utf-8") as stream:
        for line in stream:
            fields = line.rstrip("\n").split("\t")
            if not fields or fields[0] != "GQSC_MAIN_TIMING":
                continue
            if len(fields) != 14 or fields[1] != "SCE018":
                raise ValueError(f"malformed W1 timing row in {timing_path}")
            seen_indices.append(int(fields[2]))
            if fields[10:13] != ["128", "12", "12"] or fields[13] != "FRESH_SELECTED":
                raise ValueError(f"W1 parity mismatch in {timing_path}")
            rows.append(parse_decimal(fields[9], "W1 callback_wall_us"))
    if seen_indices != list(range(200)) or len(rows) != 200:
        raise ValueError(f"W1 run {timing_path} does not contain exact indices 0..199")
    return rows


def read_w2_w3(path: Path) -> list[Decimal]:
    joined_path = path / "joined_callbacks.jsonl"
    rows: list[Decimal] = []
    measurement_indices: list[int] = []
    complete = False
    with joined_path.open(encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            if record.get("kind") == "joined" and record.get("phase") == "MEASUREMENT":
                measurement_indices.append(int(record["phase_index"]))
                if record.get("timing_redacted") is not False or "o_total_us" not in record:
                    raise ValueError(f"missing qualification O_total in {joined_path}")
                rows.append(parse_decimal(record["o_total_us"], "stage_us.O_total"))
            elif record.get("kind") == "complete":
                complete = (
                    record.get("joined_callbacks") == 220
                    and record.get("warmup_callbacks") == 20
                    and record.get("measurement_callbacks") == 200
                    and record.get("distinct_source_epochs") == 220
                    and record.get("parity") == "PASS"
                )
    if measurement_indices != list(range(1, 201)) or len(rows) != 200 or not complete:
        raise ValueError(f"ROS run {joined_path} is incomplete or out of order")
    return rows


def decimal_text(value: Decimal) -> str:
    return format(value, "f")


def summary_row(summary: Summary) -> list[str]:
    return [
        str(summary.n),
        decimal_text(summary.median_us),
        decimal_text(summary.mean_us),
        decimal_text(summary.std_us),
        decimal_text(summary.p90_us),
        decimal_text(summary.p95_us),
        decimal_text(summary.p99_us),
        decimal_text(summary.maximum_us),
        str(summary.gt25_count),
        str(summary.gt30_count),
        str(summary.gt40_count),
        decimal_text(Decimal(summary.gt25_count) / Decimal(summary.n)),
        decimal_text(Decimal(summary.gt30_count) / Decimal(summary.n)),
        decimal_text(Decimal(summary.gt40_count) / Decimal(summary.n)),
    ]


def write_outputs(
    output_dir: Path,
    values_by_repeat: Mapping[tuple[str, str, int], Sequence[Decimal]],
    invalid_rejected_count: int,
    validity_errors: Sequence[str],
) -> tuple[str, str]:
    output_dir.mkdir(parents=True, exist_ok=True)
    callback_path = output_dir / "callback_latency.csv"
    with callback_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("workload_id", "condition", "repeat_id", "measurement_index", "latency_us", "latency_ms"))
        for key in sorted(values_by_repeat):
            workload, condition, repeat_id = key
            for index, value in enumerate(values_by_repeat[key], start=1):
                writer.writerow((workload, condition, repeat_id, index, decimal_text(value), decimal_text(value / Decimal(1000))))

    complete_cells = all((workload, condition, repeat) in values_by_repeat for workload in WORKLOADS for condition in CONDITIONS for repeat in REPEATS)
    experiment_valid = not validity_errors and complete_cells
    cells: dict[tuple[str, str], CellDecision] = {}
    if complete_cells:
        for workload in WORKLOADS:
            for condition in CONDITIONS:
                cells[(workload, condition)] = decide_cell(
                    {repeat: values_by_repeat[(workload, condition, repeat)] for repeat in REPEATS}
                )

    repeat_path = output_dir / "repeat_summary.csv"
    with repeat_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("workload_id", "condition", "repeat_id", "n", "median_us", "mean_us", "population_std_us", "p90_us", "p95_us", "p99_us", "maximum_us", "gt25_count", "gt30_count", "gt40_count", "gt25_fraction", "gt30_fraction", "gt40_fraction"))
        for key in sorted(values_by_repeat):
            writer.writerow((*key, *summary_row(Summary.from_values(values_by_repeat[key]))))

    workload_path = output_dir / "workload_summary.csv"
    with workload_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("workload_id", "condition", "n", "median_us", "mean_us", "population_std_us", "p90_us", "p95_us", "p99_us", "maximum_us", "gt25_count", "gt30_count", "gt40_count", "gt25_fraction", "gt30_fraction", "gt40_fraction", "repeat_p95_us", "repeat_pass_count", "deadline_pass"))
        for key in sorted(cells):
            cell = cells[key]
            writer.writerow((*key, *summary_row(cell.summary), ";".join(map(decimal_text, cell.repeat_p95_us)), cell.repeat_pass_count, str(cell.deadline_pass).lower()))

    contrasts: dict[str, Contrast] = build_contrasts(cells) if cells else {}
    contrast_path = output_dir / "contrast_summary.csv"
    with contrast_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("contrast_id", "source", "target", "classification_eligible", "improvement_repeat_count", "worsening_repeat_count", "direction", "pooled_p95_relative_change", "effect", "deadline_crossing"))
        for contrast_id in sorted(contrasts):
            contrast = contrasts[contrast_id]
            writer.writerow((contrast_id, "/".join(contrast.source), "/".join(contrast.target), str(contrast.classification_eligible).lower(), contrast.improvement_count, contrast.worsening_count, contrast.direction, decimal_text(contrast.relative_change), contrast.effect, contrast.deadline_crossing))

    classification = classify_environment(experiment_valid, cells, contrasts) if cells else "EXPERIMENT_INCONCLUSIVE"
    production = (
        "PRODUCTION_RUNTIME_PASS"
        if experiment_valid and cells[(WORKLOADS[2], "B")].deadline_pass
        else "PRODUCTION_RUNTIME_FAIL"
    )
    decision_path = output_dir / "decision.md"
    with decision_path.open("w", encoding="utf-8") as stream:
        stream.write("# Frozen runtime qualification decision\n\n")
        stream.write(f"- Experiment valid: `{str(experiment_valid).lower()}`\n")
        stream.write(f"- Invalid/rejected callback count: `{invalid_rejected_count}`\n")
        stream.write(f"- Production decision: `{production}`\n")
        stream.write(f"- Environment classification: `{classification}`\n")
        stream.write("- W1/W2 timing-scope equivalence: `false`\n")
        if validity_errors:
            stream.write("- Validity errors:\n")
            for error in validity_errors:
                stream.write(f"  - `{error}`\n")
    return production, classification


def analyze(manifest_path: Path, output_dir: Path) -> tuple[str, str]:
    attempts = read_manifest(manifest_path)
    final_attempts, errors = validate_attempt_schedule(attempts)
    values: dict[tuple[str, str, int], list[Decimal]] = {}
    invalid_count = sum(attempt.invalid_rejected_count for attempt in attempts)
    for key, attempt in final_attempts.items():
        if not attempt.run_valid or not attempt.gates_pass:
            errors.append(f"final run invalid: {key}")
            continue
        try:
            values[key] = read_w1(attempt.raw_dir) if key[0] == WORKLOADS[0] else read_w2_w3(attempt.raw_dir)
        except (OSError, ValueError, KeyError, json.JSONDecodeError) as error:
            errors.append(f"raw integrity failure for {key}: {error}")
    return write_outputs(output_dir, values, invalid_count, errors)


def synthetic_values(p95_us: Decimal) -> list[Decimal]:
    return [p95_us] * 200


def synthetic_cells(p95_by_cell: Mapping[tuple[str, str], Sequence[Decimal]]) -> dict[tuple[str, str], CellDecision]:
    return {
        key: decide_cell({repeat: synthetic_values(values[repeat - 1]) for repeat in REPEATS})
        for key, values in p95_by_cell.items()
    }


def self_test() -> None:
    sequence = [Decimal(index) for index in range(1, 201)]
    summary = Summary.from_values(sequence)
    assert summary.median_us == Decimal("100.5")
    assert summary.p90_us == Decimal(180)
    assert summary.p95_us == Decimal(190)
    assert summary.p99_us == Decimal(198)
    repeat_values = {repeat: [Decimal("25000")] * 200 for repeat in REPEATS}
    repeat_values[5] = [Decimal("25000")] * 189 + [Decimal("25001")] * 11
    cell = decide_cell(repeat_values)
    assert cell.deadline_pass and cell.repeat_pass_count == 4 and cell.summary.p95_us == Decimal("25000")

    base = {
        (workload, condition): [Decimal("20000")] * 5
        for workload in WORKLOADS
        for condition in CONDITIONS
    }
    confirmed_data = dict(base)
    confirmed_data[(WORKLOADS[2], "A")] = [Decimal("30000")] * 5
    confirmed_data[(WORKLOADS[2], "B")] = [Decimal("20000")] * 5
    cells = synthetic_cells(confirmed_data)
    assert classify_environment(True, cells, build_contrasts(cells)) == "RUNTIME_ENVIRONMENT_CONFIRMED"

    partial_data = dict(base)
    partial_data[(WORKLOADS[1], "A")] = [Decimal("40000")] * 5
    partial_data[(WORKLOADS[1], "B")] = [Decimal("40000")] * 5
    partial_data[(WORKLOADS[2], "A")] = [Decimal("40000")] * 5
    partial_data[(WORKLOADS[2], "B")] = [Decimal("30000")] * 5
    cells = synthetic_cells(partial_data)
    assert classify_environment(True, cells, build_contrasts(cells)) == "RUNTIME_ENVIRONMENT_PARTIAL"

    cells = synthetic_cells(base)
    contrasts = build_contrasts(cells)
    assert classify_environment(True, cells, contrasts) == "RUNTIME_ENVIRONMENT_NOT_SUPPORTED"
    assert classify_environment(False, cells, contrasts) == "EXPERIMENT_INCONCLUSIVE"
    assert cells[(WORKLOADS[2], "B")].deadline_pass
    print("ANALYSIS_SELF_TEST_PASS")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-manifest", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--self-test", action="store_true")
    arguments = parser.parse_args()
    if arguments.self_test:
        self_test()
        return 0
    if arguments.run_manifest is None or arguments.output_dir is None:
        parser.error("--run-manifest and --output-dir are required unless --self-test is used")
    production, classification = analyze(arguments.run_manifest, arguments.output_dir)
    print(production)
    print(classification)
    return 0


if __name__ == "__main__":
    sys.exit(main())
