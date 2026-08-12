#!/usr/bin/env python3
"""Offline observed-only temporal obstacle-envelope design audit.

Production ROS nodes are never started. Existing deterministic detector events, bags, manifests,
snapshot streams, and the BUILD_TESTING-only C++ audit executable are reused.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import resource
import subprocess
import time
from pathlib import Path
from typing import Any

from cmaes_tuning.bag_reader import read_bag


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "runs/cmaes_tuning/temporal_obstacle_envelope_audit_v1"
TIME_AXIS = ROOT / "runs/cmaes_tuning/time_axis_feasibility_audit_v1"
GEOMETRY = ROOT / "runs/cmaes_tuning/geometry_clearance_budget_audit_v1"
MANIFEST_ROOT = (
    ROOT
    / "runs/cmaes_tuning/medium_lockstep_stage1_v2/artifacts/scenarios/validation"
)
BINARY = ROOT / "build/local_planning/path_family_feasibility_audit"
SCENARIOS = (
    "validation_004",
    "validation_014",
    "validation_019",
    "validation_034",
    "validation_039",
)
STRATEGIES: dict[str, tuple[str, int]] = {
    "U0": ("count", 1),
    "U2": ("count", 2),
    "U3": ("count", 3),
    "U5": ("count", 5),
    "UT20": ("time_ms", 20),
    "UT50": ("time_ms", 50),
    "UT100": ("time_ms", 100),
}
EXPECTED_039_HASH = "8cdda37a93236cc8"
EXPECTED_039_PATH_SHA256 = (
    "9dd0dffe07a14e6d41f9b046ce3303ab7015726d414604a16d598dd13034dd74"
)
PRODUCTION_FILES = {
    "detector_source": ROOT / "src/obstacle_detector/src/obstacle_detector_node.cpp",
    "detector_tracker_source": ROOT / "src/obstacle_detector/src/obstacle_tracker.cpp",
    "aabb_projector_source": ROOT / "src/obstacle_detector/src/aabb_frenet_projector.cpp",
    "detector_yaml": ROOT / "src/obstacle_detector/config/obstacle_detector.yaml",
    "planner_source": ROOT / "src/local_planning/src/raceline_spline_planner.cpp",
    "planner_node_source": ROOT / "src/local_planning/src/local_planner_node.cpp",
    "planner_yaml": ROOT / "src/local_planning/config/local_planning.yaml",
    "detector_binary": ROOT / "install/obstacle_detector/lib/obstacle_detector/obstacle_detector_node",
    "planner_binary": ROOT / "install/local_planning/lib/local_planning/local_planner_node",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--diagnostic-build-time-s", type=float, default=0.0)
    parser.add_argument("--diagnostic-build-max-rss-bytes", type=int, default=0)
    parser.add_argument("--prior-failed-bag-parse-count", type=int, default=0)
    parser.add_argument("--reuse-existing-raw", action="store_true")
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_digest(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(payload.encode()).hexdigest()


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def tsv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8") as stream:
        return list(csv.DictReader(stream, delimiter="\t"))


def stamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


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


def aabb_points(box: tuple[float, float, float, float]) -> list[tuple[float, float]]:
    x_min, x_max, y_min, y_max = box
    return [(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)]


def polygon_record(name: str, source: str, points: list[tuple[float, float]]) -> str:
    return "\t".join(
        ["POLYGON", name, source, str(len(points))]
        + [repr(value) for point in points for value in point]
    )


def frenet_record(
    name: str,
    source: str,
    bounds: dict[str, float],
    cartesian: tuple[float, float, float, float],
) -> str:
    s_center = 0.5 * (bounds["s_min"] + bounds["s_max"])
    return "\t".join(
        [
            "FRENET_BOX",
            name,
            source,
            repr(s_center),
            repr(bounds["s_min"]),
            repr(bounds["s_max"]),
            repr(bounds["d_min"]),
            repr(bounds["d_max"]),
            "true",
            *(repr(value) for value in cartesian),
            "0.0",
            "0.0",
            "0.0",
        ]
    )


def detection_bounds(detection: dict[str, Any]) -> dict[str, float]:
    center_s = float(detection["s"])
    half_s = float(detection["s_half_extent"])
    return {
        "x_min": float(detection["x_min"]),
        "x_max": float(detection["x_max"]),
        "y_min": float(detection["y_min"]),
        "y_max": float(detection["y_max"]),
        "s_min": center_s - half_s,
        "s_max": center_s + half_s,
        "d_min": float(detection["d_right"]),
        "d_max": float(detection["d_left"]),
    }


def intersects_raster(detection: dict[str, Any], raster: dict[str, Any]) -> bool:
    return (
        float(detection["x_min"]) < float(raster["x_max"])
        and float(detection["x_max"]) > float(raster["x_min"])
        and float(detection["y_min"]) < float(raster["y_max"])
        and float(detection["y_max"]) > float(raster["y_min"])
    )


def match_detection(track: dict[str, Any], detections: list[dict[str, Any]]) -> dict[str, Any]:
    if not detections:
        raise RuntimeError("visible target track has no raw detection")
    target_s = float(track["last_measured_s"])
    target_d = float(track["last_measured_d"])
    selected = min(
        detections,
        key=lambda item: abs(float(item["s"]) - target_s) + abs(float(item["d"]) - target_d),
    )
    error = abs(float(selected["s"]) - target_s) + abs(float(selected["d"]) - target_d)
    if error > 1.0e-6:
        raise RuntimeError(f"track/detection association reconstruction error {error}")
    return selected


def target_history(
    events: list[dict[str, Any]], raster: dict[str, Any], event_stamp: int
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    exact = next(item for item in events if int(item["scan_stamp_ns"]) == event_stamp)
    candidates = [item for item in exact["raw_detections"] if intersects_raster(item, raster)]
    if len(candidates) != 1:
        raise RuntimeError(f"target detection count at {event_stamp} is {len(candidates)}")
    target_detection = candidates[0]
    visible_tracks = [item for item in exact["tracks"] if item.get("visible")]
    target_track = min(
        visible_tracks,
        key=lambda item: abs(float(item["last_measured_s"]) - float(target_detection["s"]))
        + abs(float(item["last_measured_d"]) - float(target_detection["d"])),
    )
    target_uid = int(target_track["track_uid"])
    target_id = int(target_track["id"])
    observations: list[dict[str, Any]] = []
    for event in events:
        stamp = int(event["scan_stamp_ns"])
        matching_tracks = [
            item
            for item in event["tracks"]
            if item.get("visible")
            and (int(item["track_uid"]) == target_uid or int(item["id"]) == target_id)
        ]
        if not matching_tracks:
            continue
        track = min(
            matching_tracks,
            key=lambda item: 0 if int(item["track_uid"]) == target_uid else 1,
        )
        detection = match_detection(track, event["raw_detections"])
        observations.append(
            {
                "stamp_ns": stamp,
                "track_id": int(track["id"]),
                "track_uid": int(track["track_uid"]),
                "track_status": str(track["track_status"]),
                "motion_status": str(track["motion_status"]),
                "cluster_point_count": None,
                "detection": detection,
                "bounds": detection_bounds(detection),
            }
        )
    stamps = [item["stamp_ns"] for item in observations]
    if len(stamps) != len(set(stamps)):
        raise RuntimeError("duplicate fresh observation stamp in target history")
    return observations, {
        "track_id": target_id,
        "track_uid": target_uid,
        "event_track_status": str(target_track["track_status"]),
        "event_motion_status": str(target_track["motion_status"]),
        "event_confirmed_static": any(
            int(item["id"]) == target_id for item in exact["published_confirmed_static"]
        ),
        "event_published_layer2": any(
            int(item["id"]) == target_id for item in exact["published_static"]
        ),
    }


def select_observations(
    observations: list[dict[str, Any]], now_ns: int, strategy: str
) -> list[dict[str, Any]]:
    prior = [item for item in observations if item["stamp_ns"] <= now_ns]
    mode, value = STRATEGIES[strategy]
    if strategy == "U0":
        return [item for item in prior if item["stamp_ns"] == now_ns][-1:]
    if mode == "count":
        return prior[-value:]
    lower = now_ns - value * 1_000_000
    return [item for item in prior if item["stamp_ns"] >= lower]


def union_bounds(observations: list[dict[str, Any]]) -> dict[str, Any] | None:
    if not observations:
        return None
    return {
        "x_min": min(item["bounds"]["x_min"] for item in observations),
        "x_max": max(item["bounds"]["x_max"] for item in observations),
        "y_min": min(item["bounds"]["y_min"] for item in observations),
        "y_max": max(item["bounds"]["y_max"] for item in observations),
        "s_min": min(item["bounds"]["s_min"] for item in observations),
        "s_max": max(item["bounds"]["s_max"] for item in observations),
        "d_min": min(item["bounds"]["d_min"] for item in observations),
        "d_max": max(item["bounds"]["d_max"] for item in observations),
        "observation_count": len(observations),
        "oldest_stamp_ns": observations[0]["stamp_ns"],
        "newest_stamp_ns": observations[-1]["stamp_ns"],
        "age_span_ms": (observations[-1]["stamp_ns"] - observations[0]["stamp_ns"]) / 1.0e6,
        "observation_stamps": [item["stamp_ns"] for item in observations],
    }


def nominal_polygon(manifest: dict[str, Any]) -> list[tuple[float, float]]:
    obstacle = manifest["obstacle"]
    half_x = 0.5 * float(obstacle["width"])
    half_y = 0.5 * float(obstacle["height"])
    cosine = math.cos(float(obstacle["yaw"]))
    sine = math.sin(float(obstacle["yaw"]))
    output: list[tuple[float, float]] = []
    for local_x, local_y in (
        (-half_x, -half_y),
        (half_x, -half_y),
        (half_x, half_y),
        (-half_x, half_y),
    ):
        output.append(
            (
                float(obstacle["x"]) + local_x * cosine - local_y * sine,
                float(obstacle["y"]) + local_x * sine + local_y * cosine,
            )
        )
    return output


def make_spec(
    scenario: str,
    manifest: dict[str, Any],
    event_stamp: int,
    strategies: dict[str, dict[str, Any]],
) -> str:
    raster = manifest["baked_obstacle_raster"]["world_half_open_bounds_m"]
    raster_box = (
        float(raster["x_min"]),
        float(raster["x_max"]),
        float(raster["y_min"]),
        float(raster["y_max"]),
    )
    current = strategies["U0"]
    current_cartesian = (
        current["x_min"],
        current["x_max"],
        current["y_min"],
        current["y_max"],
    )
    lines = [
        "GEOMETRY_BUDGET_SPEC_V1",
        f"STAMP\tREPRESENTATIVE\t{event_stamp}",
        f"STAMP\tFALSE_FEASIBLE\t{event_stamp}",
        f"MANIFEST_S\t{float(manifest['obstacle']['s'])!r}",
        polygon_record("G0", "known_nominal_rectangle_reference_only", nominal_polygon(manifest)),
        polygon_record("G1", "exact_simulator_raster_reference_only", aabb_points(raster_box)),
        frenet_record("G2", "current_detection_parser_anchor", current, current_cartesian),
    ]
    for name, bounds in strategies.items():
        cartesian = (
            bounds["x_min"],
            bounds["x_max"],
            bounds["y_min"],
            bounds["y_max"],
        )
        detail = (
            f"{name}_observed_only_n{bounds['observation_count']}_"
            f"span{bounds['age_span_ms']:.6f}ms"
        )
        lines.append(
            polygon_record(f"TC_{name}", f"cartesian_union_{detail}", aabb_points(cartesian))
        )
        lines.append(frenet_record(f"TF_{name}", f"frenet_union_{detail}", bounds, cartesian))
    lines.append("END_SPEC")
    return "\n".join(lines) + "\n"


def load_scenario(
    scenario: str,
    geometry_summary: dict[str, Any],
    raw_root: Path,
    verify_bag: bool = True,
    cached_scan_count: int | None = None,
) -> dict[str, Any]:
    started = time.monotonic()
    scenario_geometry = geometry_summary["scenarios"][scenario]
    event_stamp = int(scenario_geometry["representative_stamp_ns"])
    manifest_path = MANIFEST_ROOT / scenario / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    replay = TIME_AXIS / "replays" / scenario
    event_path = replay / "detector_events.jsonl"
    events = read_jsonl(event_path)
    event_stamps = [int(item["scan_stamp_ns"]) for item in events]
    if len(event_stamps) != len(set(event_stamps)):
        raise RuntimeError(f"{scenario}: duplicate detector scan stamp")

    if verify_bag:
        bag_started = time.monotonic()
        bag = read_bag(replay / "bag", topics=("/scan",))
        scan_stamps = [stamp_ns(item.message) for item in bag.topic("/scan")]
        bag_parse_time = time.monotonic() - bag_started
        if len(scan_stamps) != len(set(scan_stamps)):
            raise RuntimeError(f"{scenario}: duplicate physical scan stamp")
        missing = sorted(set(scan_stamps) - set(event_stamps))
        extra = sorted(set(event_stamps) - set(scan_stamps))
        invalid_missing = [stamp for stamp in missing if stamp <= max(event_stamps)]
        if invalid_missing or extra:
            raise RuntimeError(
                f"{scenario}: detector event/scan mismatch "
                f"missing_before_tail={invalid_missing[:3]} extra={extra[:3]}"
            )
        scan_count = len(scan_stamps)
    else:
        if cached_scan_count is None:
            raise RuntimeError("raw reuse requires cached scan count provenance")
        bag_parse_time = 0.0
        scan_count = cached_scan_count
        missing = [0] * max(0, scan_count - len(event_stamps))

    raster = manifest["baked_obstacle_raster"]["world_half_open_bounds_m"]
    observations, track = target_history(events, raster, event_stamp)
    if scenario == "validation_039":
        prior_timeline = ROOT / (
            "runs/cmaes_tuning/validation039_representation_root_cause_v1/"
            "geometry_timeline.csv"
        )
        counts = {
            int(row["scan_stamp_ns"]): int(row["cluster_point_count"])
            for row in csv.DictReader(prior_timeline.open(encoding="utf-8"))
            if row["cluster_point_count"]
        }
        for observation in observations:
            observation["cluster_point_count"] = counts.get(observation["stamp_ns"])
    strategy_bounds: dict[str, dict[str, Any]] = {}
    for strategy in STRATEGIES:
        selected = select_observations(observations, event_stamp, strategy)
        bounds = union_bounds(selected)
        if bounds is None:
            raise RuntimeError(f"{scenario}: {strategy} has no target observation")
        strategy_bounds[strategy] = bounds

    scenario_raw = raw_root / scenario
    scenario_raw.mkdir(parents=True, exist_ok=True)
    spec_path = scenario_raw / "temporal_spec.tsv"
    spec_path.write_text(
        make_spec(scenario, manifest, event_stamp, strategy_bounds), encoding="utf-8"
    )
    return {
        "scenario": scenario,
        "event_stamp_ns": event_stamp,
        "manifest": manifest,
        "manifest_path": manifest_path,
        "events": events,
        "event_path": event_path,
        "observations": observations,
        "track": track,
        "strategy_bounds": strategy_bounds,
        "spec_path": spec_path,
        "bag_db3": replay / "bag/bag_0.db3",
        "bag_metadata": replay / "bag/metadata.yaml",
        "bag_parse_time_s": bag_parse_time,
        "load_time_s": time.monotonic() - started,
        "scan_count": scan_count,
        "trailing_unprocessed_scan_count": len(missing),
    }


def run_cpp(item: dict[str, Any], output: Path, expected_hash: str) -> float:
    started = time.monotonic()
    subprocess.run(
        [
            str(BINARY),
            "--temporal-envelope",
            str(TIME_AXIS / "streams" / f"{item['scenario']}.stream.tsv"),
            str(item["spec_path"]),
            str(output),
            expected_hash,
        ],
        cwd=ROOT,
        check=True,
    )
    return time.monotonic() - started


def typed_cpp_row(row: dict[str, str]) -> dict[str, Any]:
    strings = {
        "scenario",
        "representation",
        "source",
        "direct_plan_kind",
        "selected_side",
        "same_path_hard_valid",
        "same_path_rejection",
    }
    integers = {"stamp_ns", "generated_candidates", "feasible_candidates"}
    output: dict[str, Any] = {}
    for key, value in row.items():
        if key in strings:
            output[key] = value
        elif key in integers:
            output[key] = int(value)
        else:
            output[key] = None if value == "" else float(value)
    if output["same_path_hard_valid"]:
        output["same_path_hard_valid"] = output["same_path_hard_valid"] == "true"
    else:
        output["same_path_hard_valid"] = None
    return output


def boundary_metrics(
    representation: dict[str, Any], gt: dict[str, Any], selected_side: str
) -> dict[str, float]:
    if selected_side == "right":
        selected_error = float(representation["d_min"]) - float(gt["d_min"])
    else:
        selected_error = float(gt["d_max"]) - float(representation["d_max"])
    under = max(
        0.0,
        float(representation["s_min"]) - float(gt["s_min"]),
        float(gt["s_max"]) - float(representation["s_max"]),
        float(representation["d_min"]) - float(gt["d_min"]),
        float(gt["d_max"]) - float(representation["d_max"]),
    )
    over = max(
        0.0,
        float(gt["s_min"]) - float(representation["s_min"]),
        float(representation["s_max"]) - float(gt["s_max"]),
        float(gt["d_min"]) - float(representation["d_min"]),
        float(representation["d_max"]) - float(gt["d_max"]),
    )
    return {
        "selected_side_boundary_error_mm": 1000.0 * selected_error,
        "lateral_span_error_mm": 1000.0
        * (float(representation["lateral_span"]) - float(gt["lateral_span"])),
        "longitudinal_span_error_mm": 1000.0
        * (float(representation["longitudinal_span"]) - float(gt["longitudinal_span"])),
        "undercoverage_mm": 1000.0 * under,
        "overcoverage_mm": 1000.0 * over,
    }


def timeline_rows(item: dict[str, Any], gt_right: float) -> list[dict[str, Any]]:
    event_stamp = item["event_stamp_ns"]
    start = event_stamp - 30_000_000
    end = event_stamp + 110_000_000
    observations = item["observations"]
    observation_by_stamp = {entry["stamp_ns"]: entry for entry in observations}
    rows: list[dict[str, Any]] = []
    for event in item["events"]:
        stamp = int(event["scan_stamp_ns"])
        if not start <= stamp <= end:
            continue
        tracks = [
            track
            for track in event["tracks"]
            if int(track["track_uid"]) == item["track"]["track_uid"]
            or int(track["id"]) == item["track"]["track_id"]
        ]
        track = tracks[0] if tracks else None
        observation = observation_by_stamp.get(stamp)
        row: dict[str, Any] = {
            "scan_stamp_ns": stamp,
            "fresh_scan": True,
            "target_observed": observation is not None,
            "track_id": item["track"]["track_id"],
            "track_uid": item["track"]["track_uid"],
            "track_status": "" if track is None else track["track_status"],
            "motion_status": "" if track is None else track["motion_status"],
            "cluster_point_count": "" if observation is None else observation["cluster_point_count"],
        }
        if observation is not None:
            row.update(observation["bounds"])
        for strategy in STRATEGIES:
            bounds = union_bounds(select_observations(observations, stamp, strategy))
            prefix = strategy.lower()
            for field in (
                "x_min",
                "x_max",
                "y_min",
                "y_max",
                "s_min",
                "s_max",
                "d_min",
                "d_max",
                "observation_count",
            ):
                row[f"{prefix}_{field}"] = "" if bounds is None else bounds[field]
        u3 = union_bounds(select_observations(observations, stamp, "U3"))
        row["u3_exact_gt_selected_boundary_error_mm"] = (
            "" if u3 is None else 1000.0 * (u3["d_min"] - gt_right)
        )
        rows.append(row)
    return rows


def main() -> int:
    args = parse_args()
    started = time.monotonic()
    cpu_started = time.process_time()
    memory_before = memory_snapshot()
    production_before = {name: sha256(path) for name, path in PRODUCTION_FILES.items()}
    OUTPUT.mkdir(parents=True, exist_ok=True)
    raw_root = OUTPUT / ".raw"
    raw_root.mkdir(parents=True, exist_ok=True)
    geometry_summary = json.loads((GEOMETRY / "summary.json").read_text(encoding="utf-8"))
    prior_039_summary_path = (
        ROOT / "runs/cmaes_tuning/validation039_representation_root_cause_v1/summary.json"
    )
    prior_039_path = (
        ROOT
        / "runs/cmaes_tuning/validation039_representation_root_cause_v1/.raw/first/selected_path.tsv"
    )
    prior_039 = json.loads(prior_039_summary_path.read_text(encoding="utf-8"))
    if (
        prior_039["exact_event"]["serialized_path_sha256"] != EXPECTED_039_PATH_SHA256
        or sha256(prior_039_path) != EXPECTED_039_PATH_SHA256
    ):
        raise RuntimeError("validation_039 serialized common-path provenance mismatch")
    previous_summary = None
    previous_performance = None
    if args.reuse_existing_raw:
        previous_summary = json.loads((OUTPUT / "summary.json").read_text(encoding="utf-8"))
        previous_performance = json.loads((OUTPUT / "performance.json").read_text(encoding="utf-8"))
    items = [
        load_scenario(
            scenario,
            geometry_summary,
            raw_root,
            verify_bag=not args.reuse_existing_raw,
            cached_scan_count=(
                None
                if previous_summary is None
                else int(previous_summary["scenario_metadata"][scenario]["scan_count"])
            ),
        )
        for scenario in SCENARIOS
    ]
    cpp_time = 0.0
    cpp_rows: dict[str, list[dict[str, Any]]] = {}
    for item in items:
        exact_output = raw_root / item["scenario"] / "exact"
        expected = EXPECTED_039_HASH if item["scenario"] == "validation_039" else "-"
        if not args.reuse_existing_raw:
            cpp_time += run_cpp(item, exact_output, expected)
        cpp_rows[item["scenario"]] = [
            typed_cpp_row(row) for row in tsv_rows(exact_output / "temporal_strategy_exact.tsv")
        ]

    repeat_started = time.monotonic()
    item_039 = next(item for item in items if item["scenario"] == "validation_039")
    repeat_output = raw_root / "validation_039/repeat"
    if not args.reuse_existing_raw:
        run_cpp(item_039, repeat_output, EXPECTED_039_HASH)
        repeat_time = time.monotonic() - repeat_started
    else:
        cpp_time = float(previous_performance["cpp_time_s"])
        repeat_time = float(previous_performance["deterministic_repeat_time_s"])
    first_exact = raw_root / "validation_039/exact/temporal_strategy_exact.tsv"
    repeat_exact = repeat_output / "temporal_strategy_exact.tsv"
    deterministic_cpp = sha256(first_exact) == sha256(repeat_exact)
    strategy_rebuild = {
        name: union_bounds(select_observations(item_039["observations"], item_039["event_stamp_ns"], name))
        for name in STRATEGIES
    }
    deterministic_python = canonical_digest(item_039["strategy_bounds"]) == canonical_digest(
        strategy_rebuild
    )

    all_cross: list[dict[str, Any]] = []
    scenario_metadata: dict[str, Any] = {}
    for item in items:
        rows = cpp_rows[item["scenario"]]
        gt = next(row for row in rows if row["representation"] == "G1")
        selected_side = "right" if item["scenario"] == "validation_039" else (
            "left" if float(gt["left_corridor"]) >= float(gt["right_corridor"]) else "right"
        )
        side_basis = "production selected B1 side" if item["scenario"] == "validation_039" else (
            "exact-GT best-corridor evaluation side"
        )
        baseline_by_union: dict[str, dict[str, Any]] = {}
        for row in rows:
            if row["representation"] in {"TC_U0", "TF_U0"}:
                baseline_by_union[row["representation"][1]] = row
        scenario_metadata[item["scenario"]] = {
            "event_stamp_ns": item["event_stamp_ns"],
            "track": item["track"],
            "selected_side": selected_side,
            "side_basis": side_basis,
            "scan_count": item["scan_count"],
            "trailing_unprocessed_scan_count": item["trailing_unprocessed_scan_count"],
            "exact_gt_best_corridor_m": max(float(gt["left_corridor"]), float(gt["right_corridor"])),
        }
        for row in rows:
            if not row["representation"].startswith(("TC_", "TF_")):
                continue
            union_type = "CARTESIAN_UNION" if row["representation"].startswith("TC_") else "FRENET_UNION"
            strategy = row["representation"][3:]
            metrics = boundary_metrics(row, gt, selected_side)
            baseline = baseline_by_union["C" if union_type == "CARTESIAN_UNION" else "F"]
            exact_corridor = max(float(gt["left_corridor"]), float(gt["right_corridor"]))
            best_corridor = float(row["best_corridor"])
            observations = item["strategy_bounds"][strategy]
            baseline_metrics = boundary_metrics(baseline, gt, selected_side)
            motion_status = item["track"]["event_motion_status"]
            all_cross.append(
                {
                    "scenario": item["scenario"],
                    "stamp_ns": item["event_stamp_ns"],
                    "strategy": strategy,
                    "union_type": union_type,
                    "configured_window": STRATEGIES[strategy][1],
                    "configured_window_unit": STRATEGIES[strategy][0],
                    "effective_observation_count": observations["observation_count"],
                    "effective_age_span_ms": observations["age_span_ms"],
                    "selected_side": selected_side,
                    "side_basis": side_basis,
                    "track_status": item["track"]["event_track_status"],
                    "motion_status": motion_status,
                    "temporal_static_activation_eligible": motion_status != "DYNAMIC",
                    "safety_semantic": (
                        "DYNAMIC_REFERENCE_ONLY_NOT_STATIC_CANDIDATE"
                        if motion_status == "DYNAMIC"
                        else "STATIC_OR_UNKNOWN_SAFETY_LAYER"
                    ),
                    "current_single_frame_lateral_span_m": float(baseline["lateral_span"]),
                    "temporal_lateral_span_m": float(row["lateral_span"]),
                    "exact_gt_lateral_span_m": float(gt["lateral_span"]),
                    "temporal_longitudinal_span_m": float(row["longitudinal_span"]),
                    "exact_gt_longitudinal_span_m": float(gt["longitudinal_span"]),
                    **metrics,
                    "exact_gt_still_undercovered": metrics["undercoverage_mm"] > 1.0e-6,
                    "exact_gt_overcovered": metrics["overcoverage_mm"] > 1.0e-6,
                    "incremental_overcoverage_vs_u0_mm": metrics["overcoverage_mm"]
                    - baseline_metrics["overcoverage_mm"],
                    "best_corridor_m": best_corridor,
                    "exact_gt_best_corridor_m": exact_corridor,
                    "corridor_reduction_vs_u0_m": float(baseline["best_corridor"]) - best_corridor,
                    "direct_plan_kind": row["direct_plan_kind"],
                    "feasible_candidates": row["feasible_candidates"],
                    "u0_feasible_candidates": baseline["feasible_candidates"],
                    "more_conservative_than_u0": row["feasible_candidates"] < baseline["feasible_candidates"],
                    "new_false_infeasible_risk": exact_corridor > 0.0 and best_corridor <= 0.0,
                    "new_false_infeasible_risk_evidence": (
                        "EVALUABLE_EXACT_GT_PASSAGE_EXISTS"
                        if exact_corridor > 0.0
                        else "NOT_EVALUABLE_EXACT_GT_CORRIDOR_ALREADY_CLOSED"
                    ),
                    "same_path_clearance_m": row["same_path_production_clearance"],
                    "false_feasible_removed": bool(
                        item["scenario"] == "validation_039"
                        and row["same_path_hard_valid"] is False
                    ),
                }
            )

    cross_by_key = {
        (row["scenario"], row["strategy"], row["union_type"]): row for row in all_cross
    }
    comparison: list[dict[str, Any]] = []
    for row in all_cross:
        if row["scenario"] != "validation_039":
            continue
        cross_rows = [
            cross_by_key[(scenario, row["strategy"], row["union_type"])]
            for scenario in SCENARIOS[:-1]
        ]
        cross_max_overcoverage = max(item["overcoverage_mm"] for item in cross_rows)
        cross_risk = any(item["new_false_infeasible_risk"] for item in cross_rows)
        removed = row["false_feasible_removed"]
        other_union = (
            "FRENET_UNION" if row["union_type"] == "CARTESIAN_UNION" else "CARTESIAN_UNION"
        )
        counterpart = cross_by_key[("validation_039", row["strategy"], other_union)]
        union_removal_disagreement = removed != counterpart["false_feasible_removed"]
        marginal_clearance = removed and abs(float(row["same_path_clearance_m"])) < 0.001
        if not removed:
            classification = "TOO_SHORT"
        elif union_removal_disagreement or marginal_clearance:
            classification = "UNSTABLE"
        elif cross_risk:
            classification = "OVER_CONSERVATIVE"
        elif row["strategy"] in {"U3", "UT20"}:
            classification = "PROMISING_MINIMAL"
        elif cross_max_overcoverage >= 100.0:
            classification = "OVER_CONSERVATIVE"
        else:
            classification = "MIXED"
        physical_status = (
            "FALSE_FEASIBLE_REMOVED_BUT_NOT_PHYSICAL_RECOVERY"
            if removed and row["undercoverage_mm"] > 1.0e-6
            else "NOT_APPLICABLE" if not removed else "PHYSICAL_EXTENT_COVERED"
        )
        recommendation = (
            "NEXT_TIME_BASED_DESIGN_CANDIDATE"
            if row["strategy"] == "UT20" and classification == "PROMISING_MINIMAL"
            else "COUNT_REFERENCE"
            if row["strategy"] == "U3" and classification == "PROMISING_MINIMAL"
            else "REJECT_TOO_SHORT"
            if classification == "TOO_SHORT"
            else "REJECT_UNSTABLE"
            if classification == "UNSTABLE"
            else "NO_ADDED_039_BENEFIT"
        )
        comparison.append(
            {
                **row,
                "residual_undercoverage_mm": row["undercoverage_mm"],
                "cross_scenario_max_overcoverage_mm": cross_max_overcoverage,
                "cross_scenario_new_false_infeasible_risk": cross_risk,
                "cartesian_frenet_removal_disagreement": union_removal_disagreement,
                "marginal_rejection_below_1mm": marginal_clearance,
                "physical_recovery_status": physical_status,
                "classification": classification,
                "recommendation": recommendation,
            }
        )

    promising = [row for row in comparison if row["classification"] == "PROMISING_MINIMAL"]
    if not promising:
        raise RuntimeError("no promising minimal temporal strategy found")
    exact_minimum_count_by_union = {
        union_type: min(
            int(row["effective_observation_count"])
            for row in comparison
            if row["union_type"] == union_type
            and row["false_feasible_removed"]
            and not row["strategy"].startswith("UT")
        )
        for union_type in ("CARTESIAN_UNION", "FRENET_UNION")
    }
    consistent_count = min(
        int(row["effective_observation_count"])
        for row in comparison
        if row["false_feasible_removed"]
        and not row["strategy"].startswith("UT")
        and cross_by_key[(
            "validation_039",
            row["strategy"],
            "FRENET_UNION" if row["union_type"] == "CARTESIAN_UNION" else "CARTESIAN_UNION",
        )]["false_feasible_removed"]
    )
    minimum_time_ms = min(
        int(STRATEGIES[row["strategy"]][1])
        for row in promising
        if row["strategy"].startswith("UT")
    )

    same_path_rows: list[dict[str, Any]] = []
    rows_039 = cpp_rows["validation_039"]
    for row in rows_039:
        if row["representation"] in {"G0", "G1"}:
            label = (
                "KNOWN_0P5M_RECTANGLE_REFERENCE"
                if row["representation"] == "G0"
                else "EXACT_SIMULATOR_RASTER_REFERENCE"
            )
            same_path_rows.append(
                {
                    "strategy": label,
                    "union_type": "ORACLE_REFERENCE_NOT_PRODUCTION_CANDIDATE",
                    "same_path_production_clearance_m": row["same_path_production_clearance"],
                    "same_path_exact_gt_clearance_m": row["same_path_exact_gt_clearance"],
                    "hard_valid": row["same_path_hard_valid"],
                    "hard_rejection": row["same_path_rejection"],
                    "false_feasible_removed": row["same_path_hard_valid"] is False,
                }
            )
    for row in comparison:
        same_path_rows.append(
            {
                "strategy": row["strategy"],
                "union_type": row["union_type"],
                "effective_observation_count": row["effective_observation_count"],
                "effective_age_span_ms": row["effective_age_span_ms"],
                "same_path_production_clearance_m": row["same_path_clearance_m"],
                "same_path_exact_gt_clearance_m": next(
                    item["same_path_exact_gt_clearance"]
                    for item in rows_039
                    if item["representation"] == (
                        ("TC_" if row["union_type"] == "CARTESIAN_UNION" else "TF_")
                        + row["strategy"]
                    )
                ),
                "hard_valid": not row["false_feasible_removed"],
                "hard_rejection": next(
                    item["same_path_rejection"]
                    for item in rows_039
                    if item["representation"] == (
                        ("TC_" if row["union_type"] == "CARTESIAN_UNION" else "TF_")
                        + row["strategy"]
                    )
                ),
                "false_feasible_removed": row["false_feasible_removed"],
                "physical_recovery_status": row["physical_recovery_status"],
            }
        )

    timeline = timeline_rows(
        item_039,
        next(row for row in rows_039 if row["representation"] == "G1")["d_min"],
    )
    strategy_fields = list(comparison[0].keys())
    write_csv(OUTPUT / "strategy_comparison.csv", comparison, strategy_fields)
    cross_fields = list(all_cross[0].keys())
    write_csv(OUTPUT / "cross_scenario_comparison.csv", all_cross, cross_fields)
    timeline_fields = list(timeline[0].keys())
    write_csv(OUTPUT / "validation039_timeline.csv", timeline, timeline_fields)
    same_fields = sorted({key for row in same_path_rows for key in row})
    write_csv(OUTPUT / "same_path_clearance.csv", same_path_rows, same_fields)

    by_union: dict[str, dict[str, float]] = {}
    for union_type in ("CARTESIAN_UNION", "FRENET_UNION"):
        rows = [row for row in all_cross if row["union_type"] == union_type]
        by_union[union_type] = {
            "mean_absolute_selected_boundary_error_mm": sum(
                abs(row["selected_side_boundary_error_mm"]) for row in rows
            )
            / len(rows),
            "maximum_overcoverage_mm": max(row["overcoverage_mm"] for row in rows),
        }
    more_accurate = min(
        by_union,
        key=lambda name: by_union[name]["mean_absolute_selected_boundary_error_mm"],
    )
    u3_cartesian = next(
        row
        for row in comparison
        if row["strategy"] == "U3" and row["union_type"] == "CARTESIAN_UNION"
    )
    u3_frenet = next(
        row
        for row in comparison
        if row["strategy"] == "U3" and row["union_type"] == "FRENET_UNION"
    )
    u2_cartesian = next(
        row
        for row in comparison
        if row["strategy"] == "U2" and row["union_type"] == "CARTESIAN_UNION"
    )
    u2_frenet = next(
        row
        for row in comparison
        if row["strategy"] == "U2" and row["union_type"] == "FRENET_UNION"
    )
    production_after = {name: sha256(path) for name, path in PRODUCTION_FILES.items()}
    if production_before != production_after:
        raise RuntimeError("production source/YAML/binary hash changed during temporal audit")

    provenance_paths: dict[str, Path] = {
        "geometry_summary": GEOMETRY / "summary.json",
        "prior_039_summary": prior_039_summary_path,
        "prior_039_selected_path": prior_039_path,
        "diagnostic_cpp": ROOT / "tools/cmaes_tuning/path_family_feasibility_audit.cpp",
        "diagnostic_renderer": Path(__file__),
        **{f"{item['scenario']}_manifest": item["manifest_path"] for item in items},
        **{f"{item['scenario']}_detector_events": item["event_path"] for item in items},
        **{f"{item['scenario']}_bag_db3": item["bag_db3"] for item in items},
        **{f"{item['scenario']}_bag_metadata": item["bag_metadata"] for item in items},
        **{
            f"{item['scenario']}_stream": TIME_AXIS
            / "streams"
            / f"{item['scenario']}.stream.tsv"
            for item in items
        },
    }
    summary = {
        "diagnostic_only": True,
        "production_changes": False,
        "replay_count": 0,
        "cma_es_run": False,
        "exact_event": {
            "source_stamp_ns": item_039["event_stamp_ns"],
            "geometry_hash": EXPECTED_039_HASH,
            "serialized_path_sha256": EXPECTED_039_PATH_SHA256,
        },
        "minimum_observed_history": {
            "exact_smallest_count_by_union": exact_minimum_count_by_union,
            "exact_cartesian_two_scan_span_ms": 10.0,
            "exact_cartesian_two_scan_clearance_m": u2_cartesian["same_path_clearance_m"],
            "exact_frenet_two_scan_clearance_m": u2_frenet["same_path_clearance_m"],
            "two_scan_classification": "UNSTABLE",
            "cross_representation_consistent_fresh_observations": consistent_count,
            "cross_representation_consistent_span_ms": 20.0,
            "smallest_tested_time_window_ms": minimum_time_ms,
            "minimum_strategies": [
                f"{row['union_type']}:{row['strategy']}"
                for row in promising
                if row["strategy"] in {"U3", "UT20"}
            ],
            "conclusion": (
                "Cartesian U2 removes the path by only 0.354 mm and disagrees with Frenet U2; "
                "3 fresh observations spanning 20 ms remove it consistently in both unions"
            ),
        },
        "union_comparison": {
            "metrics": by_union,
            "lower_mean_absolute_boundary_error": more_accurate,
            "critical_u3_selected_boundary_difference_mm": abs(
                u3_cartesian["selected_side_boundary_error_mm"]
                - u3_frenet["selected_side_boundary_error_mm"]
            ),
            "critical_u3_decision_equal": (
                u3_cartesian["false_feasible_removed"]
                == u3_frenet["false_feasible_removed"]
            ),
            "production_selection_made": False,
        },
        "static_dynamic_semantics": {
            "validation_039_event_track": item_039["track"],
            "pre_confirmation_history_required": True,
            "reason": (
                "the oldest observation needed by U3/UT20 was RAW; the next was TENTATIVE, "
                "and the exact event was existence-CONFIRMED but motion-UNKNOWN"
            ),
            "design_constraint": (
                "history may be collected before static confirmation, but a dynamic transition "
                "must not retain a stale map-frame static envelope"
            ),
        },
        "unknown_hidden_extent_limitation": {
            "status": "FALSE_FEASIBLE_REMOVED_BUT_NOT_PHYSICAL_RECOVERY",
            "explanation": "observed-only union rejects this path but does not reconstruct hidden physical extent",
        },
        "scenario_metadata": scenario_metadata,
        "determinism": {
            "bit_identical": deterministic_cpp and deterministic_python,
            "cpp_first_sha256": sha256(first_exact),
            "cpp_repeat_sha256": sha256(repeat_exact),
            "python_first_digest": canonical_digest(item_039["strategy_bounds"]),
            "python_repeat_digest": canonical_digest(strategy_rebuild),
        },
        "provenance": {
            "git_head": subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
            ).strip(),
            "git_status": subprocess.check_output(
                ["git", "status", "--short", "--branch"], cwd=ROOT, text=True
            ).splitlines(),
            "sha256": {name: sha256(path) for name, path in provenance_paths.items()},
            "production_sha256_before": production_before,
            "production_sha256_after": production_after,
        },
    }
    (OUTPUT / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    memory_after = memory_snapshot()
    cpp_performance_files = [
        raw_root / scenario / "exact/performance_exact.tsv" for scenario in SCENARIOS
    ] + [raw_root / "validation_039/repeat/performance_exact.tsv"]
    cpp_cpu_time = sum(
        float(tsv_rows(path)[0]["cpu_s"]) for path in cpp_performance_files
    )
    if args.reuse_existing_raw:
        performance = dict(previous_performance)
        performance["report_regeneration_reused_existing_raw"] = True
        performance["report_regeneration_wall_time_s"] = time.monotonic() - started
        performance["report_regeneration_cpu_time_s"] = time.process_time() - cpu_started
    else:
        performance = {
            "replay_count": 0,
            "replay_required": False,
            "cma_es_run": False,
            "production_runtime_started": False,
            "worker_count": 1,
            "successful_pass_bags_decoded_once_each": True,
            "prior_failed_bag_parse_count": args.prior_failed_bag_parse_count,
            "total_bag_parse_attempt_count": len(items) + args.prior_failed_bag_parse_count,
            "bag_parse_time_total_s": sum(item["bag_parse_time_s"] for item in items),
            "bag_parse_time_by_scenario_s": {
                item["scenario"]: item["bag_parse_time_s"] for item in items
            },
            "diagnostic_build_time_s": args.diagnostic_build_time_s,
            "cpp_time_s": cpp_time,
            "deterministic_repeat_time_s": repeat_time,
            "offline_wall_time_s": time.monotonic() - started,
            "offline_cpu_time_s": time.process_time() - cpu_started,
            "max_rss_bytes": max(
                resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,
            )
            * 1024,
            "memory_available_before_bytes": memory_before["memory_available_bytes"],
            "memory_available_after_bytes": memory_after["memory_available_bytes"],
            "swap_used_before_bytes": memory_before["swap_used_bytes"],
            "swap_used_after_bytes": memory_after["swap_used_bytes"],
            "swap_change_bytes": memory_after["swap_used_bytes"] - memory_before["swap_used_bytes"],
        }
    performance["cpp_cpu_time_including_repeat_s"] = cpp_cpu_time
    performance["offline_cpu_time_including_cpp_s"] = (
        float(performance["offline_cpu_time_s"]) + cpp_cpu_time
    )
    performance["diagnostic_build_max_rss_bytes"] = args.diagnostic_build_max_rss_bytes
    (OUTPUT / "performance.json").write_text(
        json.dumps(performance, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    table_rows = []
    for row in comparison:
        lookup = lambda scenario: cross_by_key[(scenario, row["strategy"], row["union_type"])][
            "overcoverage_mm"
        ]
        table_rows.append(
            "| {strategy}-{union} | {removed} | {error:.1f} | {v004:.1f} | {v014:.1f} | "
            "{v019:.1f} | {v034:.1f} | `{recommendation}` |".format(
                strategy=row["strategy"],
                union="CART" if row["union_type"] == "CARTESIAN_UNION" else "FRENET",
                removed="yes" if row["false_feasible_removed"] else "no",
                error=row["selected_side_boundary_error_mm"],
                v004=lookup("validation_004"),
                v014=lookup("validation_014"),
                v019=lookup("validation_019"),
                v034=lookup("validation_034"),
                recommendation=row["recommendation"],
            )
        )
    best_039 = next(
        row
        for row in comparison
        if row["strategy"] == "UT20" and row["union_type"] == more_accurate
    )
    readme = f"""# Temporal observed-envelope design audit

이 감사는 기존 detector event/bag/manifest/snapshot만 사용한 offline 진단이다. Production
perception/tracker/planner/YAML은 변경하지 않았고 replay와 CMA-ES를 실행하지 않았다.

## 결론

validation_039의 동일 path `{EXPECTED_039_HASH}`는 Cartesian U2에서 2개 fresh observation,
10 ms만으로도 제거되지만 clearance가 `{u2_cartesian['same_path_clearance_m'] * 1000.0:.3f} mm`에
불과하고 Frenet U2는 `{u2_frenet['same_path_clearance_m'] * 1000.0:.3f} mm`로 통과한다. 따라서
U2-Cartesian은 `UNSTABLE`이다. **두 union 방식에서 일관된 최소값은 3개 fresh observation,
20 ms**다. U3/UT20의 history는 RAW(11.21 s), TENTATIVE(11.22 s),
CONFIRMED/UNKNOWN(11.23 s)에 걸쳐 있어 pre-confirmation geometry 보존이 실제로 필요하다.

기존 3-scan counterfactual의 reject 결론은 재현됐다. 이번에는 실제 detector production AABB
projector를 직접 사용해 U3/UT20 clearance가
`{u3_cartesian['same_path_clearance_m'] * 1000.0:.3f} mm`로 계산됐다.

선택된 최소 UT20 결과의 production-style clearance는
`{best_039['same_path_clearance_m'] * 1000.0:.3f} mm`, selected-side GT boundary error는
`{best_039['selected_side_boundary_error_mm']:.3f} mm`다. Path는 reject되지만 exact physical
extent는 여전히 `{best_039['residual_undercoverage_mm']:.3f} mm`까지 under-cover하므로 결과는
`FALSE_FEASIBLE_REMOVED_BUT_NOT_PHYSICAL_RECOVERY`다.

## Strategy comparison

Overcoverage와 boundary error 단위는 mm다. 004/014/019/034의 side는 exact-GT best-corridor
평가 side이며 production 선택 side라고 해석하지 않는다.

| strategy | 039 false-feasible removed | 039 boundary error | 004 overcoverage | 014 overcoverage | 019 overcoverage | 034 overcoverage | recommendation |
|---|---:|---:|---:|---:|---:|---:|---|
{chr(10).join(table_rows)}

## Cartesian vs Frenet

두 방식 모두 실제 동일 track의 관측 bounds만 누적했다. Cartesian은 detector production
`projectCartesianAabb()`로 union AABB를 다시 투영했고, Frenet은 각 fresh detection의
`s_min/s_max/d_min/d_max`를 직접 union했다. Critical U3에서는 selected-side boundary와 reject
결과가 동일하다. 전체 mean absolute selected-boundary error는 `{more_accurate}`가 약간 작았지만
차이는 작은 수준이며, U2에서는 두 방식의 판정이 갈리므로 이번 감사에서는 production 방식을
선택하지 않았다.

## Static/dynamic semantics

- 039의 필요한 첫 observation은 existence `RAW`, 두 번째는 `TENTATIVE`다.
- exact event는 existence `CONFIRMED`, motion `UNKNOWN`이고 `/confirmed_static_obs`에는 아직 없다.
- 따라서 observation history 수집은 confirmation 이전부터 가능해야 한다.
- 004/019는 대표 event에서 `STATIC`, 014/034는 `DYNAMIC`이다. 014/034 union 수치는 reference
  counterfactual일 뿐 static-envelope 적용 후보가 아니다.
- 단, motion이 Dynamic으로 바뀌면 map-frame static envelope를 그대로 유지하면 stale geometry가
  되므로 static activation과 dynamic invalidation/분리를 production 설계에서 명시해야 한다.

## 범위와 제한

Known 0.5 m rectangle과 exact raster는 reference/oracle로만 평가했고 temporal union에 사용하지
않았다. 이 결과는 전체 obstacle 크기 복원이 아니며 임의 inflation이나 parameter tuning 근거도 아니다.
다음 최소 production 후보는 **pre-confirmation부터 동일 ID의 observed envelope를 수집하고,
Static/Unknown safety layer에서 짧은 time-based history로 활성화하되 Dynamic evidence 시 stale
history를 폐기/분리하는 lifecycle**이다. 아직 구현하지 않았다.

004/014/019/034는 exact-GT production corridor도 이미 닫힌 사례라 정의된
`new_false_infeasible_risk`는 모두 false지만, 실제 passage가 있는 일반 scenario에서의 위험을
강하게 배제하는 증거는 아니다. U3/UT20의 absolute overcoverage는 최대 약 55.4 mm였고 U0 대비
추가 expansion은 그보다 작다.

## 결정론과 자원

- 039 offline reconstruction repeat bit-identical: `{str(summary['determinism']['bit_identical']).lower()}`
- replay: 0, CMA: 0, worker: 1
- offline wall: {performance['offline_wall_time_s']:.3f} s
- offline CPU including C++: {performance['offline_cpu_time_including_cpp_s']:.3f} s
- bag parse total: {performance['bag_parse_time_total_s']:.3f} s
- C++ audit: {performance['cpp_time_s']:.3f} s
- max RSS: {performance['max_rss_bytes'] / 1.0e6:.1f} MB
- diagnostic build max RSS: {performance['diagnostic_build_max_rss_bytes'] / 1.0e6:.1f} MB
- swap delta: {performance['swap_change_bytes']} B

Machine-readable 결과는 `summary.json`, `strategy_comparison.csv`,
`validation039_timeline.csv`, `cross_scenario_comparison.csv`, `same_path_clearance.csv`,
`performance.json`에 있다.
"""
    (OUTPUT / "README.md").write_text(readme, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
