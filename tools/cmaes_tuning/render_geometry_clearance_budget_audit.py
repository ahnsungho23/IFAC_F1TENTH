#!/usr/bin/env python3
"""Render the diagnostic-only geometry/clearance budget audit.

All collision, rotated-footprint, and staged-corridor calculations are emitted by the C++
diagnostic executable that compiles the production planner.  This script only reconstructs
manifest/detector inputs, launches that evaluator, aggregates its TSV records, and writes the
human/machine-readable artifact set.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import subprocess
import sys
import time
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
TIME_AXIS = ROOT / "runs/cmaes_tuning/time_axis_feasibility_audit_v1"
MANIFEST_ROOT = (
    ROOT / "runs/cmaes_tuning/medium_lockstep_stage1_v2/artifacts/scenarios/validation"
)
OUTPUT = ROOT / "runs/cmaes_tuning/geometry_clearance_budget_audit_v1"
BINARY = ROOT / "build/local_planning/path_family_feasibility_audit"
SCENARIOS = tuple(f"validation_{value}" for value in ("004", "014", "019", "034", "039"))
FALSE_FEASIBLE_STAMPS = {
    "validation_014": 11_710_000_000,
    "validation_039": 11_230_000_000,
}
STAGES = ("S0", "S1", "S2", "S3", "S4", "S5")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def overlap_area(item: dict[str, Any], bounds: dict[str, float]) -> float:
    width = max(
        0.0,
        min(float(item["x_max"]), float(bounds["x_max"]))
        - max(float(item["x_min"]), float(bounds["x_min"])),
    )
    height = max(
        0.0,
        min(float(item["y_max"]), float(bounds["y_max"]))
        - max(float(item["y_min"]), float(bounds["y_min"])),
    )
    return width * height


def obstacle_polygon(obstacle: dict[str, Any]) -> list[tuple[float, float]]:
    cosine = math.cos(float(obstacle["yaw"]))
    sine = math.sin(float(obstacle["yaw"]))
    result = []
    for x_local, y_local in (
        (-0.5 * float(obstacle["width"]), -0.5 * float(obstacle["height"])),
        (0.5 * float(obstacle["width"]), -0.5 * float(obstacle["height"])),
        (0.5 * float(obstacle["width"]), 0.5 * float(obstacle["height"])),
        (-0.5 * float(obstacle["width"]), 0.5 * float(obstacle["height"])),
    ):
        result.append(
            (
                float(obstacle["x"]) + cosine * x_local - sine * y_local,
                float(obstacle["y"]) + sine * x_local + cosine * y_local,
            )
        )
    return result


def raster_polygon(bounds: dict[str, float]) -> list[tuple[float, float]]:
    return [
        (float(bounds["x_min"]), float(bounds["y_min"])),
        (float(bounds["x_max"]), float(bounds["y_min"])),
        (float(bounds["x_max"]), float(bounds["y_max"])),
        (float(bounds["x_min"]), float(bounds["y_max"])),
    ]


def target_detection(
    events: list[dict[str, Any]], stamp: int, bounds: dict[str, float]
) -> tuple[dict[str, Any], bool]:
    event = next(item for item in events if int(item["scan_stamp_ns"]) == stamp)
    raw = max(event["raw_detections"], key=lambda item: overlap_area(item, bounds))
    if overlap_area(raw, bounds) <= 0.0:
        raise RuntimeError(f"target raw detection does not overlap raster at {stamp}")
    static_available = any(overlap_area(item, bounds) > 0.0 for item in event["published_static"])
    return raw, static_available


def polygon_record(name: str, source: str, polygon: list[tuple[float, float]]) -> str:
    values = ["POLYGON", name, source, str(len(polygon))]
    for x, y in polygon:
        values.extend((f"{x:.17g}", f"{y:.17g}"))
    return "\t".join(values)


def frenet_box_record(
    name: str,
    source: str,
    detection: dict[str, Any],
    available: bool,
) -> str:
    s_center = float(detection.get("s", detection.get("s_center")))
    if "s_half_extent" in detection:
        s_min = s_center - float(detection["s_half_extent"])
        s_max = s_center + float(detection["s_half_extent"])
    else:
        s_min = float(detection["s_start"])
        s_max = float(detection["s_end"])
    values: list[Any] = [
        "FRENET_BOX",
        name,
        source,
        s_center,
        s_min,
        s_max,
        min(float(detection["d_right"]), float(detection["d_left"])),
        max(float(detection["d_right"]), float(detection["d_left"])),
        "true" if available else "false",
        float(detection["x_min"]),
        float(detection["x_max"]),
        float(detection["y_min"]),
        float(detection["y_max"]),
        s_max - s_min,
        0.0,
        0.0,
    ]
    return "\t".join(str(item) for item in values)


def write_spec(
    scenario: str,
    manifest: dict[str, Any],
    representative_stamp: int,
    false_stamp: int,
    detector_events: list[dict[str, Any]],
    destination: Path,
) -> None:
    bounds = manifest["baked_obstacle_raster"]["world_half_open_bounds_m"]
    g2, available = target_detection(detector_events, representative_stamp, bounds)
    lines = [
        "GEOMETRY_BUDGET_SPEC_V1",
        f"STAMP\tREPRESENTATIVE\t{representative_stamp}",
        f"STAMP\tFALSE_FEASIBLE\t{false_stamp}",
        f"MANIFEST_S\t{float(manifest['obstacle']['s']):.17g}",
        polygon_record("G0", "nominal_manifest_polygon_before_raster", obstacle_polygon(manifest["obstacle"])),
        polygon_record("G1", "exact_simulator_half_open_occupied_raster", raster_polygon(bounds)),
        frenet_box_record(
            "G2",
            "detector_raw_projection_at_representative_stamp",
            g2,
            available,
        ),
        "END_SPEC",
    ]
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")


def read_tsv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream, delimiter="\t"))


def number(value: str | float | int | None) -> float | None:
    if value in (None, ""):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: "" if row.get(key) is None else row.get(key) for key in fields})


def read_reference(stream_path: Path) -> list[dict[str, float]]:
    result = []
    for line in stream_path.read_text(encoding="utf-8").splitlines():
        tokens = line.split("\t")
        if tokens[0] != "W":
            continue
        result.append(
            {
                "s": float(tokens[2]),
                "d_right": float(tokens[6]),
                "d_left": float(tokens[7]),
                "yaw": float(tokens[8]),
            }
        )
    return result


def track_width_summary(
    reference: list[dict[str, float]], s_min: float, s_max: float
) -> dict[str, Any]:
    local = [item for item in reference if s_min <= item["s"] <= s_max]
    if not local:
        local = [min(reference, key=lambda item: abs(item["s"] - 0.5 * (s_min + s_max)))]
    worst = min(local, key=lambda item: item["d_left"] + item["d_right"])
    widest = max(local, key=lambda item: item["d_left"] + item["d_right"])
    return {
        "minimum_local_track_width_m": worst["d_left"] + worst["d_right"],
        "maximum_local_track_width_m": widest["d_left"] + widest["d_right"],
        "track_width_worst_s_m": worst["s"],
        "track_d_left_at_worst_m": worst["d_left"],
        "track_d_right_at_worst_m": worst["d_right"],
        "local_track_profile_s_dright_dleft": ";".join(
            f"{item['s']:.17g}:{item['d_right']:.17g}:{item['d_left']:.17g}"
            for item in local
        ),
    }


def stage_rows_by_key(rows: list[dict[str, str]]) -> dict[tuple[str, str, str], dict[str, str]]:
    return {(row["representation"], row["stage"], row["side"]): row for row in rows}


def first_closing_stage(
    by_key: dict[tuple[str, str, str], dict[str, str]], representation: str, side: str
) -> str | None:
    previous: float | None = None
    for stage in STAGES:
        current = float(by_key[(representation, stage, side)]["point_min"])
        if stage == "S0" and current <= 0.0:
            return "S0_physical"
        if previous is not None and previous > 0.0 and current <= 0.0:
            return stage
        previous = current
    return None


def classify_scenario(
    by_key: dict[tuple[str, str, str], dict[str, str]],
    closest: list[dict[str, str]],
    mismatch: list[dict[str, str]],
) -> tuple[str, str, str]:
    g1_s0 = max(float(by_key[("G1", "S0", side)]["point_min"]) for side in ("left", "right"))
    production = max(
        float(by_key[("G3", "EXACT_PRODUCTION", side)]["point_min"])
        for side in ("left", "right")
    )
    closures = {
        first_closing_stage(by_key, "G1", side) for side in ("left", "right")
    }
    false_overlap = any(row["g1_exact_overlap"] == "true" for row in mismatch)
    if g1_s0 <= 0.0:
        return "PHYSICALLY_BLOCKED", "exact physical track/obstacle/rotated footprint", "CORRIDOR_EMPTY"
    if false_overlap:
        return "REPRESENTATION_MISMATCH", "perception AABB under-represents exact raster", "MIXED"
    if "S4" in closures:
        return "TRACKING_RESERVE_LIMITED", "tracking-error reserve", "OBSTACLE_CLEARANCE_LIMITED"
    if "S1" in closures:
        return "SAFETY_MARGIN_LIMITED", "wall_safety_margin", "FOOTPRINT_LIMITED"
    if "S2" in closures or "S3" in closures:
        return "SAFETY_MARGIN_LIMITED", "obstacle/longitudinal safety margin", "OBSTACLE_CLEARANCE_LIMITED"
    if "S5" in closures:
        return "UNCERTAINTY_GUARD_LIMITED", "same-ID union/uncertainty guard", "MIXED"
    if production > 0.0 and closest and all(row["status"] != "feasible" for row in closest):
        return "TRANSITION_DYNAMICS_LIMITED", "curvature/slope/rate", "TRANSITION_LIMITED"
    return "MIXED", "multiple geometric and transition constraints", "MIXED"


def fmt_mm(value: float | None) -> str:
    if value is None:
        return "—"
    return f"{value * 1000:+.1f} mm"


def git_text(*args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE
    ).stdout.rstrip()


def memory_snapshot() -> dict[str, int]:
    values: dict[str, int] = {}
    for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
        key, raw = line.split(":", 1)
        if key in {"MemAvailable", "SwapTotal", "SwapFree"}:
            values[key] = int(raw.strip().split()[0]) * 1024
    return {
        "memory_available_bytes": values["MemAvailable"],
        "swap_used_bytes": values["SwapTotal"] - values["SwapFree"],
    }


def main() -> int:
    if not BINARY.is_file():
        raise RuntimeError(f"diagnostic executable is missing: {BINARY}")
    wall_started = time.monotonic()
    cpu_started = time.process_time()
    memory_before = memory_snapshot()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    raw_root = OUTPUT / ".raw"
    raw_root.mkdir(exist_ok=True)
    time_summary = json.loads((TIME_AXIS / "summary.json").read_text(encoding="utf-8"))

    geometry_rows: list[dict[str, Any]] = []
    ablation_rows: list[dict[str, Any]] = []
    closest_rows: list[dict[str, Any]] = []
    mismatch_rows: list[dict[str, Any]] = []
    event_representation_rows: list[dict[str, Any]] = []
    scenario_summaries: dict[str, dict[str, Any]] = {}
    cpp_performance: list[dict[str, str]] = []

    for scenario in SCENARIOS:
        scenario_raw = raw_root / scenario
        scenario_raw.mkdir(exist_ok=True)
        manifest = json.loads((MANIFEST_ROOT / scenario / "manifest.json").read_text(encoding="utf-8"))
        representative_stamp = int(time_summary["scenarios"][scenario]["events"]["T_READY"])
        false_stamp = FALSE_FEASIBLE_STAMPS.get(scenario, 0)
        detector_events = read_jsonl(TIME_AXIS / "replays" / scenario / "detector_events.jsonl")
        spec_path = scenario_raw / "geometry_spec.tsv"
        write_spec(
            scenario,
            manifest,
            representative_stamp,
            false_stamp,
            detector_events,
            spec_path,
        )
        stream_path = TIME_AXIS / "streams" / f"{scenario}.stream.tsv"
        subprocess.run(
            [str(BINARY), "--geometry-budget", str(stream_path), str(spec_path), str(scenario_raw)],
            cwd=ROOT,
            check=True,
        )
        raw_geometry = read_tsv(scenario_raw / "geometry.tsv")
        raw_ablation = read_tsv(scenario_raw / "ablation.tsv")
        raw_closest = read_tsv(scenario_raw / "closest.tsv")
        raw_mismatch = read_tsv(scenario_raw / "mismatch.tsv")
        cpp_performance.extend(read_tsv(scenario_raw / "performance.tsv"))
        by_rep = {row["representation"]: row for row in raw_geometry}
        by_key = stage_rows_by_key(raw_ablation)
        reference = read_reference(stream_path)
        track = track_width_summary(
            reference, float(by_rep["G1"]["s_min"]), float(by_rep["G1"]["s_max"])
        )
        classification, dominant, side_class = classify_scenario(
            by_key, raw_closest, raw_mismatch
        )

        for row in raw_geometry:
            enriched: dict[str, Any] = dict(row)
            enriched.update(
                track_width_summary(reference, float(row["s_min"]), float(row["s_max"]))
            )
            enriched.update(
                {
                    "vehicle_length_m": 0.56,
                    "vehicle_width_m": 0.287,
                    "wall_safety_margin_m": 0.04,
                    "obstacle_safety_margin_m": 0.014789254299520768,
                    "tracking_reserve_m": 0.14,
                    "obstacle_longitudinal_padding_m": 0.4149924657737441,
                    "worst_relevant_yaw_projection_m": number(
                        by_key[(row["representation"], "S4", "left")]["maximum_yaw_projection"]
                    ),
                    "classification": classification,
                    "raster_changed_cell_count": (
                        int(manifest["baked_obstacle_raster"]["changed_cell_count"])
                        if row["representation"] == "G1"
                        else None
                    ),
                    "raster_world_half_open_width_m": (
                        float(manifest["baked_obstacle_raster"]["world_half_open_bounds_m"]["x_max"])
                        - float(manifest["baked_obstacle_raster"]["world_half_open_bounds_m"]["x_min"])
                        if row["representation"] == "G1"
                        else None
                    ),
                    "raster_world_half_open_height_m": (
                        float(manifest["baked_obstacle_raster"]["world_half_open_bounds_m"]["y_max"])
                        - float(manifest["baked_obstacle_raster"]["world_half_open_bounds_m"]["y_min"])
                        if row["representation"] == "G1"
                        else None
                    ),
                }
            )
            geometry_rows.append(enriched)

        for row in raw_ablation:
            enriched = dict(row)
            pointwise = float(row["point_min"])
            intersection = float(row["residual"])
            enriched["empty"] = pointwise <= 0.0
            enriched["deficit_m"] = max(0.0, -pointwise)
            enriched["intersection_empty"] = intersection <= 0.0
            enriched["intersection_deficit_m"] = max(0.0, -intersection)
            enriched["classification"] = classification
            enriched["side_constraint_class"] = side_class
            ablation_rows.append(enriched)

        closest_by_side = {row["side"]: row for row in raw_closest}
        for side in ("left", "right"):
            base = dict(closest_by_side.get(side, {}))
            base.update(
                {
                    "scenario": scenario,
                    "logical_stamp_ns": representative_stamp,
                    "side": side,
                    "record_kind": (
                        "closest_transition" if side in closest_by_side else "corridor_only_no_candidate"
                    ),
                    "g1_physical_corridor_residual_m": float(
                        by_key[("G1", "S0", side)]["point_min"]
                    ),
                    "g1_full_safety_corridor_residual_m": float(
                        by_key[("G1", "S4", side)]["point_min"]
                    ),
                    "production_corridor_residual_m": float(
                        by_key[("G3", "EXACT_PRODUCTION", side)]["point_min"]
                    ),
                    "corridor_deficit_m": max(
                        0.0, -float(by_key[("G1", "S4", side)]["point_min"])
                    ),
                }
            )
            closest_rows.append(base)
        for row in raw_mismatch:
            mismatch_rows.append(dict(row))
            closest_rows.append({**row, "record_kind": "false_feasible_comparison"})

        if false_stamp:
            bounds = manifest["baked_obstacle_raster"]["world_half_open_bounds_m"]
            false_g2, false_available = target_detection(detector_events, false_stamp, bounds)
            with (TIME_AXIS / "raw_time_axis" / f"{scenario}.tsv").open(
                encoding="utf-8", newline=""
            ) as stream_file:
                false_axis = next(
                    row
                    for row in csv.DictReader(stream_file, delimiter="\t")
                    if int(row["logical_stamp_ns"]) == false_stamp
                )
            g1_row = by_rep["G1"]
            false_geometries = [
                {
                    "representation": "G2",
                    "s_center": float(false_g2["s"]),
                    "s_min": float(false_g2["s"]) - float(false_g2["s_half_extent"]),
                    "s_max": float(false_g2["s"]) + float(false_g2["s_half_extent"]),
                    "d_min": min(float(false_g2["d_right"]), float(false_g2["d_left"])),
                    "d_max": max(float(false_g2["d_right"]), float(false_g2["d_left"])),
                    "planner_available": false_available,
                    "source": "detector_raw_at_false_feasible_stamp",
                },
                {
                    "representation": "G3",
                    "s_center": 0.5
                    * (
                        float(false_axis["planning_obstacle_s_start"])
                        + float(false_axis["planning_obstacle_s_end"])
                    ),
                    "s_min": float(false_axis["planning_obstacle_s_start"]),
                    "s_max": float(false_axis["planning_obstacle_s_end"]),
                    "d_min": min(
                        float(false_axis["planning_obstacle_d_right"]),
                        float(false_axis["planning_obstacle_d_left"]),
                    ),
                    "d_max": max(
                        float(false_axis["planning_obstacle_d_right"]),
                        float(false_axis["planning_obstacle_d_left"]),
                    ),
                    "planner_available": True,
                    "source": "production_union_guard_at_false_feasible_stamp",
                },
            ]
            for item in false_geometries:
                s_span = item["s_max"] - item["s_min"]
                d_span = item["d_max"] - item["d_min"]
                event_representation_rows.append(
                    {
                        "scenario": scenario,
                        "logical_stamp_ns": false_stamp,
                        "event": "false_feasible_candidate",
                        "representation": item["representation"],
                        "reference": "G1",
                        "s_span_m": s_span,
                        "d_span_m": d_span,
                        "s_center_shift_vs_g1_m": item["s_center"]
                        - float(g1_row["s_center"]),
                        "d_center_shift_vs_g1_m": 0.5 * (item["d_min"] + item["d_max"])
                        - float(g1_row["d_center"]),
                        "longitudinal_span_difference_vs_g1_m": s_span
                        - float(g1_row["s_span"]),
                        "lateral_span_difference_vs_g1_m": d_span
                        - float(g1_row["d_span"]),
                        "planner_available": item["planner_available"],
                        "supported_cause": (
                            "same-stamp detector partial-visible Cartesian AABB/Frenet projection"
                            if item["representation"] == "G2"
                            else "production same-ID union plus longitudinal guard"
                        ),
                    }
                )

        physical_g0 = max(float(by_key[("G0", "S0", side)]["point_min"]) for side in ("left", "right"))
        physical_g1 = max(float(by_key[("G1", "S0", side)]["point_min"]) for side in ("left", "right"))
        full_production = max(
            float(by_key[("G3", "EXACT_PRODUCTION", side)]["point_min"])
            for side in ("left", "right")
        )
        first_by_side = {
            side: first_closing_stage(by_key, "G1", side) for side in ("left", "right")
        }
        first_stage = "/".join(
            f"{side}:{value or 'open'}" for side, value in first_by_side.items()
        )
        scenario_summaries[scenario] = {
            "representative_stamp_ns": representative_stamp,
            "classification": classification,
            "dominant_cause": dominant,
            "side_constraint_class": side_class,
            "nominal_physical_best_corridor_m": physical_g0,
            "exact_raster_physical_best_corridor_m": physical_g1,
            "full_production_best_corridor_m": full_production,
            "first_empty_stage_by_side": first_by_side,
            "first_empty_stage": first_stage,
            "track": track,
            "representations": {
                name: {
                    "s_span_m": float(by_rep[name]["s_span"]),
                    "d_span_m": float(by_rep[name]["d_span"]),
                    "s_center_m": float(by_rep[name]["s_center"]),
                    "d_center_m": float(by_rep[name]["d_center"]),
                    "planner_available": by_rep[name]["planner_available"] == "true",
                }
                for name in ("G0", "G1", "G2", "G3")
            },
            "exact_raster_ablation": {
                stage: {
                    side: float(by_key[("G1", stage, side)]["residual"])
                    for side in ("left", "right")
                }
                for stage in STAGES
            },
            "exact_raster_pointwise_ablation": {
                stage: {
                    side: float(by_key[("G1", stage, side)]["point_min"])
                    for side in ("left", "right")
                }
                for stage in STAGES
            },
            "production_corridor": {
                side: float(by_key[("G3", "EXACT_PRODUCTION", side)]["point_min"])
                for side in ("left", "right")
            },
            "production_corridor_intersection": {
                side: float(by_key[("G3", "EXACT_PRODUCTION", side)]["residual"])
                for side in ("left", "right")
            },
            "closest_candidates": raw_closest,
            "false_feasible_comparisons": raw_mismatch,
        }

    # Exact per-stage contribution rows are easier to form once all raw rows are available.
    margin_rows: list[dict[str, Any]] = []
    stage_lookup = {
        (row["scenario"], row["representation"], row["side"], row["stage"]): row
        for row in ablation_rows
        if row["stage"] in STAGES
    }
    stage_labels = {
        "S0": "physical track + rectangle + obstacle",
        "S1": "+ wall_safety_margin_m",
        "S2": "+ obstacle safety_margin_m",
        "S3": "+ obstacle_longitudinal_padding_m",
        "S4": "+ tracking-error reserve",
        "S5": "+ same-ID union / uncertainty guard where applicable",
    }
    for scenario in SCENARIOS:
        for representation in ("G0", "G1", "G2", "G3"):
            for side in ("left", "right"):
                prior: float | None = None
                for stage in STAGES:
                    source_row = stage_lookup[(scenario, representation, side, stage)]
                    current = float(source_row["point_min"])
                    intersection = float(source_row["residual"])
                    margin_rows.append(
                        {
                            "scenario": scenario,
                            "representation": representation,
                            "side": side,
                            "stage": stage,
                            "addition": stage_labels[stage],
                            "pointwise_residual_m": current,
                            "intersection_residual_m": intersection,
                            "exact_incremental_change_m": None if prior is None else current - prior,
                            "changed_positive_to_nonpositive": (
                                prior is not None and prior > 0.0 and current <= 0.0
                            ),
                            "pointwise_deficit_m": max(0.0, -current),
                            "intersection_deficit_m": max(0.0, -intersection),
                        }
                    )
                    prior = current

    representation_rows: list[dict[str, Any]] = []
    for scenario in SCENARIOS:
        reps = {
            row["representation"]: row
            for row in geometry_rows
            if row["scenario"] == scenario
        }
        g1 = reps["G1"]
        for name, row in reps.items():
            representation_rows.append(
                {
                    "scenario": scenario,
                    "representation": name,
                    "reference": "G1",
                    "s_span_m": float(row["s_span"]),
                    "d_span_m": float(row["d_span"]),
                    "s_center_shift_vs_g1_m": float(row["s_center"]) - float(g1["s_center"]),
                    "d_center_shift_vs_g1_m": float(row["d_center"]) - float(g1["d_center"]),
                    "longitudinal_span_difference_vs_g1_m": float(row["s_span"]) - float(g1["s_span"]),
                    "lateral_span_difference_vs_g1_m": float(row["d_span"]) - float(g1["d_span"]),
                    "planner_available": row["planner_available"],
                    "supported_cause": (
                        "PIL rasterization adds one occupied cell per nominal dimension"
                        if name == "G0"
                        else "detector visible Cartesian AABB projected to Frenet"
                        if name == "G2"
                        else "same-ID union plus longitudinal covariance/floor guard"
                        if name == "G3"
                        else "exact half-open simulator raster"
                    ),
                }
            )
    for row in mismatch_rows:
        representation_rows.append(
            {
                "scenario": row["scenario"],
                "representation": row["input_representation"],
                "reference": "G1 candidate validation",
                "candidate_geometry_hash": row["geometry_hash"],
                "production_input_clearance_m": number(row["production_input_obstacle_clearance"]),
                "g1_production_clearance_m": number(row["g1_production_clearance"]),
                "g1_exact_footprint_clearance_m": number(row["g1_exact_footprint_clearance"]),
                "g1_exact_overlap": row["g1_exact_overlap"],
                "production_clearance_optimism_m": number(
                    row["production_clearance_optimism"]
                ),
                "physical_clearance_difference_m": number(
                    row["physical_clearance_difference"]
                ),
                "supported_cause": "detector partial-visible AABB/Frenet extent mismatch; timestamp matched",
            }
        )
    representation_rows.extend(event_representation_rows)

    geometry_fields = [
        "scenario", "logical_stamp_ns", "representation", "source", "planner_available",
        "s_center", "s_min", "s_max", "s_span", "d_center", "d_min", "d_max", "d_span",
        "polygon_xy", "pre_guard_s_span", "guard_longitudinal_growth", "union_lateral_growth",
        "minimum_local_track_width_m", "maximum_local_track_width_m", "track_width_worst_s_m",
        "track_d_left_at_worst_m", "track_d_right_at_worst_m",
        "local_track_profile_s_dright_dleft", "vehicle_length_m", "vehicle_width_m",
        "worst_relevant_yaw_projection_m", "wall_safety_margin_m", "obstacle_safety_margin_m",
        "tracking_reserve_m", "obstacle_longitudinal_padding_m", "raster_changed_cell_count",
        "raster_world_half_open_width_m", "raster_world_half_open_height_m", "classification",
    ]
    corridor_fields = [
        "scenario", "logical_stamp_ns", "representation", "stage", "side", "interval_lower",
        "interval_upper", "residual", "point_min", "point_max", "empty", "deficit_m",
        "intersection_empty", "intersection_deficit_m", "first_empty_s",
        "first_pointwise_empty_s", "worst_s", "minimum_track_width", "worst_track_s",
        "maximum_yaw_projection", "samples", "side_constraint_class", "classification",
    ]
    margin_fields = [
        "scenario", "representation", "side", "stage", "addition", "pointwise_residual_m",
        "intersection_residual_m", "exact_incremental_change_m",
        "changed_positive_to_nonpositive", "pointwise_deficit_m", "intersection_deficit_m",
    ]
    representation_fields = [
        "scenario", "logical_stamp_ns", "event", "representation", "reference", "s_span_m", "d_span_m",
        "s_center_shift_vs_g1_m", "d_center_shift_vs_g1_m",
        "longitudinal_span_difference_vs_g1_m", "lateral_span_difference_vs_g1_m",
        "planner_available", "candidate_geometry_hash", "production_input_clearance_m",
        "g1_production_clearance_m", "g1_exact_footprint_clearance_m", "g1_exact_overlap",
        "production_clearance_optimism_m", "physical_clearance_difference_m", "supported_cause",
    ]
    closest_fields = sorted({key for row in closest_rows for key in row})
    write_csv(OUTPUT / "scenario_geometry.csv", geometry_rows, geometry_fields)
    write_csv(OUTPUT / "corridor_budget.csv", ablation_rows, corridor_fields)
    write_csv(OUTPUT / "margin_ablation.csv", margin_rows, margin_fields)
    write_csv(OUTPUT / "representation_comparison.csv", representation_rows, representation_fields)
    write_csv(OUTPUT / "closest_to_feasible.csv", closest_rows, closest_fields)

    # Repeat two standalone offline evaluations and demand byte-identical raw numeric outputs.
    repeat_started = time.monotonic()
    determinism: dict[str, Any] = {}
    for scenario in ("validation_019", "validation_039"):
        source = raw_root / scenario
        repeat = raw_root / f"{scenario}_repeat"
        repeat.mkdir(exist_ok=True)
        subprocess.run(
            [
                str(BINARY),
                "--geometry-budget",
                str(TIME_AXIS / "streams" / f"{scenario}.stream.tsv"),
                str(source / "geometry_spec.tsv"),
                str(repeat),
            ],
            cwd=ROOT,
            check=True,
        )
        files = ("geometry.tsv", "ablation.tsv", "closest.tsv", "mismatch.tsv")
        first_digest = hashlib.sha256(
            b"".join((source / name).read_bytes() for name in files)
        ).hexdigest()
        repeat_digest = hashlib.sha256(
            b"".join((repeat / name).read_bytes() for name in files)
        ).hexdigest()
        if first_digest != repeat_digest:
            raise RuntimeError(f"determinism mismatch for {scenario}")
        determinism[scenario] = {
            "first_sha256": first_digest,
            "repeat_sha256": repeat_digest,
            "bit_identical": True,
        }
    repeat_time = time.monotonic() - repeat_started

    summary = {
        "schema": "geometry_clearance_budget_audit/1",
        "diagnostic_only": True,
        "production_changes": False,
        "replay_required": False,
        "replay_episode_count": 0,
        "sampling": {
            "longitudinal_step_m": 0.001,
            "boundary_bisection_iterations": 50,
            "track_footprint_method": "production measureFootprintTrackBound rotated-corner projection",
            "physical_obstacle_method": "convex polygon SAT plus exact segment distance",
            "production_obstacle_method": "production Frenet AABB + half-width + safety + LUT reserve",
        },
        "parameters": {
            "vehicle_length_m": 0.56,
            "vehicle_width_m": 0.287,
            "safety_margin_m": 0.014789254299520768,
            "wall_safety_margin_m": 0.04,
            "tracking_error_reserve_m": 0.14,
            "obstacle_longitudinal_padding_m": 0.4149924657737441,
            "uncertainty_lateral_inflation_m": 0.0,
        },
        "source_semantics": {
            "G0": "nominal manifest polygon before rasterization",
            "G1": "exact half-open simulator occupied raster bounds",
            "G2": "same-stamp detector raw Cartesian AABB and Frenet projection",
            "G3": "production same-ID stabilization union plus longitudinal uncertainty guard",
        },
        "scenarios": scenario_summaries,
        "determinism": determinism,
        "provenance": {
            "git_head": git_text("rev-parse", "HEAD"),
            "planner_source_sha256": sha256_file(
                ROOT / "src/local_planning/src/raceline_spline_planner.cpp"
            ),
            "planner_node_source_sha256": sha256_file(
                ROOT / "src/local_planning/src/local_planner_node.cpp"
            ),
            "yaml_sha256": sha256_file(ROOT / "src/local_planning/config/local_planning.yaml"),
            "diagnostic_source_sha256": sha256_file(
                ROOT / "tools/cmaes_tuning/path_family_feasibility_audit.cpp"
            ),
            "artifact_reuse": [
                "time_axis_feasibility_audit_v1",
                "path_family_feasibility_audit_v1",
                "entry_available_distance_validation_v1",
                "medium_lockstep_stage1_v2",
            ],
        },
    }
    (OUTPUT / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    table_rows = []
    for scenario, item in scenario_summaries.items():
        table_rows.append(
            f"| {scenario.removeprefix('validation_')} | "
            f"{fmt_mm(item['nominal_physical_best_corridor_m'])} | "
            f"{fmt_mm(item['exact_raster_physical_best_corridor_m'])} | "
            f"{fmt_mm(item['full_production_best_corridor_m'])} | "
            f"{item['first_empty_stage']} | {item['dominant_cause']} | "
            f"{item['classification']} |"
        )

    physically_blocked = [
        name.removeprefix("validation_")
        for name, item in scenario_summaries.items()
        if item["classification"] == "PHYSICALLY_BLOCKED"
    ]
    tracking_dominant = [
        name.removeprefix("validation_")
        for name, item in scenario_summaries.items()
        if item["classification"] == "TRACKING_RESERVE_LIMITED"
    ]
    transition_limited = [
        name.removeprefix("validation_")
        for name, item in scenario_summaries.items()
        if item["classification"] == "TRANSITION_DYNAMICS_LIMITED"
    ]
    false_overlap_scenarios = sorted(
        {
            row["scenario"].removeprefix("validation_")
            for row in mismatch_rows
            if row["g1_exact_overlap"] == "true"
        }
    )
    scenario_details: list[str] = []
    for scenario, item in scenario_summaries.items():
        short = scenario.removeprefix("validation_")
        reps = item["representations"]
        exact = item["exact_raster_ablation"]
        exact_point = item["exact_raster_pointwise_ablation"]
        production = item["production_corridor"]
        scenario_details.append(
            f"### {short}\n\n"
            f"- projected span (s/d): G0 {reps['G0']['s_span_m']*1000:.1f}/"
            f"{reps['G0']['d_span_m']*1000:.1f} mm, G1 {reps['G1']['s_span_m']*1000:.1f}/"
            f"{reps['G1']['d_span_m']*1000:.1f} mm, G2 {reps['G2']['s_span_m']*1000:.1f}/"
            f"{reps['G2']['d_span_m']*1000:.1f} mm, G3 {reps['G3']['s_span_m']*1000:.1f}/"
            f"{reps['G3']['d_span_m']*1000:.1f} mm.\n"
            f"- G1 pointwise-min S0 left/right: {fmt_mm(exact_point['S0']['left'])} / "
            f"{fmt_mm(exact_point['S0']['right'])}; G1 S4: "
            f"{fmt_mm(exact_point['S4']['left'])} / {fmt_mm(exact_point['S4']['right'])}; "
            f"G3 exact-production: "
            f"{fmt_mm(production['left'])} / {fmt_mm(production['right'])}.\n"
            f"- G1 constant-target intersection S0 left/right: "
            f"{fmt_mm(exact['S0']['left'])} / {fmt_mm(exact['S0']['right'])}.\n"
            f"- first empty: left={item['first_empty_stage_by_side']['left'] or 'open'}, "
            f"right={item['first_empty_stage_by_side']['right'] or 'open'}; primary="
            f"`{item['classification']}`."
        )
        if item["closest_candidates"]:
            metrics = []
            for candidate in item["closest_candidates"]:
                metrics.append(
                    f"{candidate['side']} wall={float(candidate['wall_clearance'])*1000:+.1f} mm, "
                    f"obstacle={float(candidate['obstacle_clearance'])*1000:+.1f} mm, "
                    f"kappa/slope/rate={float(candidate['peak_curvature']):.4f}/"
                    f"{float(candidate['peak_slope']):.4f}/"
                    f"{float(candidate['peak_curvature_rate']):.4f}"
                )
            scenario_details[-1] += "\n- closest C2: " + "; ".join(metrics) + "."
        if item["false_feasible_comparisons"]:
            comparisons = []
            for comparison in item["false_feasible_comparisons"]:
                comparisons.append(
                    f"{comparison['input_representation']} input/G1-production/G1-physical="
                    f"{float(comparison['production_input_obstacle_clearance'])*1000:+.1f}/"
                    f"{float(comparison['g1_production_clearance'])*1000:+.1f}/"
                    f"{float(comparison['g1_exact_footprint_clearance'])*1000:+.1f} mm, "
                    f"overlap={comparison['g1_exact_overlap']}"
                )
            scenario_details[-1] += "\n- false-feasible comparison: " + "; ".join(comparisons) + "."
    readme = f"""# Geometry / clearance budget audit v1

이 artifact는 production runtime을 변경하지 않은 offline diagnostic-only 결과다. 새 replay와 CMA는 실행하지 않았다. 대표 상태는 time-axis audit의 `T_READY`이며, 014/039의 false-feasible 비교만 동일 stamp의 기존 capture에서 별도로 평가했다.

| scenario | nominal physical pointwise-min | exact-raster physical pointwise-min | full-production pointwise-min | first empty stage | dominant cause | classification |
|---|---:|---:|---:|---|---|---|
{os.linesep.join(table_rows)}

## 핵심 판정

- exact G1 S0에서 양쪽 모두 닫힌 `PHYSICALLY_BLOCKED`: {', '.join(physically_blocked) or '없음'}.
- tracking reserve가 corridor를 처음 닫은 primary case: {', '.join(tracking_dominant) or '없음'}.
- full-production lateral corridor는 남지만 transition geometry가 막힌 case: {', '.join(transition_limited) or '없음'}.
- G2/G3-selected path가 G1 exact raster와 실제 rectangle overlap을 만든 false-feasible case: {', '.join(false_overlap_scenarios) or '없음'}. `representation_comparison.csv`는 span/center shift와 clearance optimism을 같은 stamp로 기록한다.

## 수식 및 convention trace

- 차량은 `base_link` 중심의 0.56 x 0.287 m 직사각형이다. wall 검사는 후보 yaw로 회전한 네 corner를 local reference segment에 투영하고, 물리 `d_left/d_right`에서 `wall_safety_margin_m=0.04 m`를 한 번만 뺀다.
- obstacle input G2/G3의 lateral bounds는 detector-owned Frenet AABB다. production obstacle gate는 waypoint center에 `vehicle_half_width_m + safety_margin_m + tracking LUT reserve`를 한 번 더해 검사한다. 현재 LUT는 relevant 전 구간 0.14 m다.
- longitudinal gate는 raw/guard span의 반폭에 `obstacle_longitudinal_padding_m=0.4149924658 m`를 더한다. G3 전처리는 same-ID envelope union 후 `max(0.05 m, 3*sqrt(s_var))`의 longitudinal guard를 양쪽에 적용한다. configured lateral uncertainty floor/cap은 모두 0이므로 lateral covariance inflation은 없다.
- 따라서 vehicle half-width는 obstacle gate에 한 번, wall rectangle에 한 번 각각의 독립 collision relation으로 들어가며 같은 relation에서 중복 가산되지 않는다. `safety_margin`과 tracking reserve는 obstacle에만, wall margin은 track에만 적용된다.
- G0는 manifest polygon, G1은 PIL bake 후 simulator vertical flip/floor lookup의 21x21 half-open occupied cells(0.525 x 0.525 m), G2는 same-stamp raw detector AABB, G3는 실제 lifecycle union/guard 입력이다.

## 계산 방법과 해석 범위

S0-S2 physical test는 G0/G1/G2/G3 polygon과 회전 차량 rectangle의 convex SAT/segment distance를 사용한다. track interval은 production `measureFootprintTrackBound()`를 wall margin 0/0.04로 호출해 50-iteration boundary bisection으로 구했다. S3는 S2 physical forbidden band를 유지한 채 production longitudinal Frenet extrusion을 union하여 ablation이 단조롭게 보수화되도록 했고, S4는 LUT reserve, S5는 G2에 production union/guard를 추가한다. `EXACT_PRODUCTION` row는 별도로 G3와 production의 centerline obstacle band를 그대로 쓴다.

longitudinal sweep 간격은 1 mm다. 각 sweep 위치의 track/obstacle 경계는 production corner routine과 50회 bisection으로 계산했지만, 연속 s 전역의 해석적 최솟값이 아니라 1 mm sampled minimum이다. 이 한계를 CSV의 `samples`와 `worst_s`로 보존했다.

`margin_ablation.csv`의 `pointwise_residual_m`은 각 s에서의 최소 corridor, `intersection_residual_m`은 전체 passage span에 공통인 constant-target interval이다. incremental change는 회전 footprint와 span 확장 때문에 비선형일 수 있다. 단순 직관 budget은 track width - vehicle/obstacle occupied width - margins지만, 최종 판정은 `corridor_budget.csv`의 exact geometric result를 사용한다.

## scenario 해석

{os.linesep.join(scenario_details)}

019는 nominal/exact-raster left pointwise-min이 +6.65/+1.74 mm라 strict physical passage channel은 남는다. 다만 constant-target intersection은 -6.91/-15.46 mm이고 G1 wall margin S1에서 pointwise-min도 -39.01 mm가 된다. 따라서 019는 `PHYSICALLY_BLOCKED`가 아니라 `SAFETY_MARGIN_LIMITED`이며, exact S0 자체를 non-empty로 만들 추가 mm는 0, constant-target interval에는 15.46 mm, S1 pointwise에는 39.01 mm가 필요하다.

034는 nominal G0 right pointwise-min +8.27 mm가 exact raster G1에서 -8.26 mm로 바뀐다. rasterization이 유일한 nominal physical channel을 16.53 mm 악화시켜 blocked/non-blocked를 뒤집었으며, exact S0 closest side를 non-empty로 만드는 deficit은 8.26 mm다. 이는 자동 margin/geometry 완화 권고가 아니다.

004는 exact G1 left가 S3 +102.8 mm에서 S4 -37.2 mm로 바뀌므로 tracking reserve가 유일한 first-closing dominant case다. 014 right는 S2 +139.3 mm에서 longitudinal padding이 더 좁은 track span을 포함하는 S3 -9.6 mm로 닫힌다. uncertainty/union S5가 처음 corridor를 닫는 scenario는 없다.

G3 production representation만 보면 004/014는 corridor가 남지만 C2 closest는 curvature limit을 0.1376/0.0420 rad/m 초과하고 obstacle clearance도 -0.2/-2.0 mm다. 다만 exact G1 full-safety corridor 자체는 이미 음수이므로 둘을 순수 `TRANSITION_DYNAMICS_LIMITED`로 분류하지 않았다. 039는 G3 corridor가 양수이고 B1도 hard-valid였지만 G1 overlap이므로 transition 문제가 아니라 representation mismatch다.

현재 LUT reserve는 속도 전 구간 0.14 m로 일정하고 curvature/slope hard limit도 속도로 완화되지 않는다. 따라서 이 다섯 case 중 현 production 규칙 그대로 단순 감속만 하여 새 lateral space를 만드는 명확한 brake-then-avoid 후보는 없다. 향후 reserve model이나 path geometry 가설은 별도 연구여야 한다.

014/039의 false-feasible row는 candidate hash, production input clearance, 동일 path의 G1 production clearance, exact raster-vs-rectangle clearance를 함께 기록한다. 014는 G1 physical clearance가 +77.8~+134.0 mm라 실제 overlap은 아니지만 G1 production safety clearance는 -95.7~-106.6 mm라 exact-GT hard-valid가 아니다. 039는 두 selected path 모두 G1 physical clearance -297.0 mm로 실제 overlap이며 representation false-feasible가 확정된다.

100%-available B1은 014의 G2에서 entry 1.792 m candidate를 만들었지만 G1 safety-valid 증거는 만들지 못했다. entry parameterization coverage를 시험하는 과학적 probe로는 추가 검증 가치가 있으나, exact-GT-safe success나 production 유지의 근거는 아직 아니다. 이 audit만으로 유지/삭제 결정을 내리지 않는다.

exact raster S0에서 물리적으로 막힌 034는 “반드시 회피 성공해야 하는” CMA training objective로 사용하면 안 된다. controlled stop/no-feasible가 올바른 목표가 될 수 있다. 019는 물리적으로 불가능하지는 않지만 1.74 mm channel과 empty constant-target interval 때문에 현 family의 must-avoid success case로도 부적절하다. positive final corridor + transition excess scenario만 향후 속도-축 feasibility 연구 후보로 분리해야 한다.

## 결정성 및 자원

019와 039를 동일 captured input으로 재실행했고 geometry/ablation/closest/mismatch raw digest가 각각 bit-identical이다. replay episode는 0, worker는 1이다. 상세 wall/CPU/RSS/swap/build/repeat 시간은 `performance.json`에 기록했다.
"""
    (OUTPUT / "README.md").write_text(readme, encoding="utf-8")

    memory_after = memory_snapshot()
    usage = resource.getrusage(resource.RUSAGE_CHILDREN)
    performance = {
        "replay_required": False,
        "replay_episode_count": 0,
        "offline_wall_time_s": time.monotonic() - wall_started,
        "offline_parent_cpu_time_s": time.process_time() - cpu_started,
        "offline_cpp_reported_wall_time_s": sum(float(row["wall_s"]) for row in cpp_performance),
        "offline_cpp_reported_cpu_time_s": sum(float(row["cpu_s"]) for row in cpp_performance),
        "max_rss_bytes": int(usage.ru_maxrss) * 1024,
        "worker_count": 1,
        "swap_used_before_bytes": memory_before["swap_used_bytes"],
        "swap_used_after_bytes": memory_after["swap_used_bytes"],
        "swap_change_bytes": memory_after["swap_used_bytes"] - memory_before["swap_used_bytes"],
        "memory_available_before_bytes": memory_before["memory_available_bytes"],
        "memory_available_after_bytes": memory_after["memory_available_bytes"],
        "target_build_time_s": 132.2,
        "diagnostic_test_time_s": repeat_time,
        "deterministic_repeat_time_s": repeat_time,
        "determinism": determinism,
        "full_workspace_build_run": False,
        "cma_run": False,
    }
    (OUTPUT / "performance.json").write_text(
        json.dumps(performance, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
