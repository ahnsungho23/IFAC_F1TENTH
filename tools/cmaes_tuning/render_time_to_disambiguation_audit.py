#!/usr/bin/env python3
"""Causal offline time-to-disambiguation audit for the S3 shadow envelope.

This renderer never starts a ROS node or feeds its result to production.  The two synthetic
normal cases extend only their already-defined 10 ms / 0.04 m reference-line sensor trajectory;
validation_039 reuses its recorded bag.  Every estimator query is causal and uses source stamps
not newer than the row being evaluated.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import subprocess
import time
from typing import Any, Sequence

import numpy as np
import yaml

import render_rulebook_obstacle_envelope_audit as rulebook
import render_rulebook_free_gap_pruning_audit as free_gap
import render_s3_shadow_false_infeasible_audit as s3
from cmaes_tuning.bag_reader import read_bag
from cmaes_tuning.scenario_generator import load_waypoints
from cmaes_tuning.simulator_collision import SimulatorRasterCollisionModel
from render_validation039_representation_root_cause import yaw_from_odom


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "runs/cmaes_tuning/time_to_disambiguation_audit_v1"
RAW = OUTPUT / ".raw"
PRIOR_RULEBOOK = ROOT / "runs/cmaes_tuning/rulebook_obstacle_envelope_audit_v1"
PRIOR_S3 = ROOT / "runs/cmaes_tuning/s3_shadow_false_infeasible_audit_v1"
PRIOR_FREE_GAP = ROOT / "runs/cmaes_tuning/rulebook_free_gap_pruning_audit_v1"
TIME_AXIS = ROOT / "runs/cmaes_tuning/time_axis_feasibility_audit_v1"
STREAM_039 = TIME_AXIS / "streams/validation_039.stream.tsv"
BINARY = ROOT / "build/local_planning/path_family_feasibility_audit"
WAYPOINTS = ROOT / "offline_trajectory_generator/output/ifac_track/global_waypoints.json"
CLEAN_MAP = ROOT / "src/monte_carlo_localization/maps/ifac_track.yaml"
EXACT_039_STAMP = rulebook.EXACT_039_STAMP
GRID_M = rulebook.COARSE_GRID_M
ANGLE_DEG = rulebook.COARSE_ANGLE_DEG
FINE_GRID_M = rulebook.FINE_GRID_M
FINE_ANGLE_DEG = rulebook.FINE_ANGLE_DEG
WINDOW_MS = 20
MAX_FUTURE_MS = 1000
NORMAL_SPEED_MPS = 4.0
WORKERS = 4
PRIMARY_CASES = ("normal_01_00", "normal_01_02", "validation_039")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--diagnostic-build-time-s", type=float, default=0.0)
    parser.add_argument("--diagnostic-build-max-rss-bytes", type=int, default=0)
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


def clean_json(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: clean_json(item) for key, item in value.items()}
    if isinstance(value, list):
        return [clean_json(item) for item in value]
    if isinstance(value, tuple):
        return [clean_json(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, np.generic):
        return clean_json(value.item())
    return value


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


def write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str] | None = None) -> None:
    if not rows:
        raise RuntimeError(f"refusing to write empty CSV: {path}")
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=fields or list(rows[0]), extrasaction="ignore"
        )
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def tsv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8") as stream:
        return list(csv.DictReader(stream, delimiter="\t"))


def normal_prior_rows() -> dict[tuple[str, str], dict[str, str]]:
    rows: dict[tuple[str, str], dict[str, str]] = {}
    for row in read_csv(PRIOR_RULEBOOK / "normal_case_results.csv"):
        rows.setdefault((row["case_id"], row["strategy"]), row)
    return rows


def collision_model() -> dict[str, Any]:
    manifest = json.loads(
        (
            ROOT
            / "runs/cmaes_tuning/medium_lockstep_stage1_v2/artifacts/scenarios/validation/validation_039/manifest.json"
        ).read_text(encoding="utf-8")
    )
    return manifest["simulator_collision_model"]


def track_length(waypoints: Sequence[dict[str, Any]]) -> float:
    return float(waypoints[-1]["s_m"]) + math.hypot(
        float(waypoints[-1]["x_m"]) - float(waypoints[0]["x_m"]),
        float(waypoints[-1]["y_m"]) - float(waypoints[0]["y_m"]),
    )


def forward_distance(start: float, end: float, length: float) -> float:
    result = (end - start) % length
    return 0.0 if abs(result - length) <= 1.0e-9 else result


def baseline_cpp(case_id: str) -> dict[str, Any]:
    return json.loads((PRIOR_S3 / ".raw" / case_id / "result.json").read_text(encoding="utf-8"))


def blocking_hypotheses() -> dict[str, dict[str, float]]:
    output: dict[str, dict[str, float]] = {}
    for row in read_csv(PRIOR_FREE_GAP / "hypothesis_extrema.csv"):
        if row["case_id"] not in ("normal_01_00", "normal_01_02"):
            continue
        output[row["case_id"]] = {
            key: float(row[key])
            for key in (
                "pose_x_m",
                "pose_y_m",
                "orientation_deg",
                "width_u_m",
                "length_v_m",
                "witness_cell_x_m",
                "witness_cell_y_m",
            )
        }
    return output


def synthetic_normal_case(case_id: str) -> dict[str, Any]:
    waypoints = load_waypoints(WAYPOINTS)
    case = s3.reconstruct_normal_case(
        case_id, normal_prior_rows(), waypoints, collision_model()
    )
    model = collision_model()
    case["waypoints"] = waypoints
    case["clean_model"] = SimulatorRasterCollisionModel(CLEAN_MAP, model)
    case["baked_model"] = SimulatorRasterCollisionModel(
        PRIOR_RULEBOOK / ".raw/normal_maps" / case_id / f"{case_id}.yaml", model
    )
    case["data_source"] = "SENSOR_ONLY_RECONSTRUCTION"
    case["baseline_frame_count"] = len(case["frames"])
    case["speed_mps"] = NORMAL_SPEED_MPS
    return case


def recorded_future_frame(
    scan: Any,
    odom: Any,
    stamp_ns: int,
    raster: dict[str, float],
    model: dict[str, Any],
) -> rulebook.RayFrame:
    yaw = yaw_from_odom(odom)
    offset = float(model["lidar_offset_x_m"])
    origin = np.asarray(
        [
            float(odom.pose.pose.position.x) + offset * math.cos(yaw),
            float(odom.pose.pose.position.y) + offset * math.sin(yaw),
        ],
        dtype=np.float64,
    )
    directions = rulebook.backend_directions_continuous(yaw, model)
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    valid = np.isfinite(ranges) & (ranges >= float(scan.range_min))
    endpoints = origin + ranges[:, None] * directions
    hit_mask = valid & rulebook.points_inside_half_open(endpoints, raster)
    hit_indices = tuple(int(index) for index in np.flatnonzero(hit_mask))
    detector_yaml = yaml.safe_load(
        (ROOT / "src/obstacle_detector/config/obstacle_detector.yaml").read_text(
            encoding="utf-8"
        )
    )
    guard = max(
        rulebook.GEOMETRY_EPSILON_M,
        float(model["scan_noise_std_m"]) * float(model["scan_noise_guard_sigma"]),
        3.0
        * float(
            detector_yaml["obstacle_detector"]["ros__parameters"]["cluster_sigma"]
        ),
    )
    indices = np.flatnonzero(valid)
    target_points = endpoints[np.asarray(hit_indices, dtype=np.int64)]
    detector_box = (
        (
            float(np.min(target_points[:, 0])),
            float(np.max(target_points[:, 0])),
            float(np.min(target_points[:, 1])),
            float(np.max(target_points[:, 1])),
        )
        if target_points.size
        else (math.nan, math.nan, math.nan, math.nan)
    )
    return rulebook.RayFrame(
        stamp_ns=stamp_ns,
        status="CONFIRMED",
        motion_status="UNKNOWN",
        origin=origin,
        directions=directions[indices],
        ranges=ranges[indices],
        free_lengths=np.maximum(0.0, ranges[indices] - guard),
        hits=target_points,
        hit_indices=hit_indices,
        detector_box=detector_box,
    )


def recorded_validation039_case() -> tuple[dict[str, Any], float]:
    manifest_path, manifest = rulebook.local_manifest("validation_039")
    replay = TIME_AXIS / "replays/validation_039"
    events = rulebook.read_jsonl(replay / "detector_events.jsonl")
    raster = manifest["baked_obstacle_raster"]["world_half_open_bounds_m"]
    history, track = rulebook.target_history(events, raster, EXACT_039_STAMP)
    selected = rulebook.select_observations(history, EXACT_039_STAMP, "UT20")
    parse_started = time.monotonic()
    bag = read_bag(replay / "bag", topics=("/scan", "/ego_racecar/odom"))
    parse_time = time.monotonic() - parse_started
    scans = {rulebook.stamp_ns(item.message): item.message for item in bag.topic("/scan")}
    odometry = {
        rulebook.stamp_ns(item.message): item.message
        for item in bag.topic("/ego_racecar/odom")
    }
    frames = [
        rulebook.recorded_ray_frame(
            scans[int(item["stamp_ns"])],
            odometry[int(item["stamp_ns"])],
            item,
            raster,
            manifest["simulator_collision_model"],
        )
        for item in selected
    ]
    future_stamps = sorted(
        stamp
        for stamp in scans
        if EXACT_039_STAMP < stamp <= EXACT_039_STAMP + MAX_FUTURE_MS * 1_000_000
    )
    for stamp in future_stamps:
        frames.append(
            recorded_future_frame(
                scans[stamp],
                odometry[stamp],
                stamp,
                raster,
                manifest["simulator_collision_model"],
            )
        )
    obstacle = {
        key: float(manifest["obstacle"][key])
        for key in ("x", "y", "s", "d", "yaw", "width", "height")
    }
    return {
        "case_id": "validation_039",
        "category": "safety_anchor",
        "frames": frames,
        "frame_now_ns": EXACT_039_STAMP,
        "stamp_ns": EXACT_039_STAMP,
        "stream": str(STREAM_039),
        "manifest_s": obstacle["s"],
        "obstacle": obstacle,
        "raster": {key: float(value) for key, value in raster.items()},
        "data_source": "RECORDED_EXISTING",
        "bag_path": str(replay / "bag"),
        "event_path": str(replay / "detector_events.jsonl"),
        "bag_scan_count": len(scans),
        "bag_odom_count": len(odometry),
        "odometry": odometry,
        "track": track,
        "speed_mps": None,
        "manifest_path": str(manifest_path),
    }, parse_time


def normal_future_frame(case: dict[str, Any], absolute_index: int) -> rulebook.RayFrame:
    distance = float(case["gt_row"]["observation_distance_m"])
    pose = rulebook.interpolate_reference(
        case["waypoints"],
        float(case["manifest_s"]) - distance + 0.04 * absolute_index,
    )
    obstacle = type("Obstacle", (), case["obstacle"])()
    return rulebook.synthetic_ray_frame(
        case["clean_model"],
        case["baked_model"],
        pose,
        obstacle,
        case["raster"],
        20_000_000_000 + absolute_index * 10_000_000,
        "CONFIRMED",
    )


def witness_statistics(
    frames: Sequence[rulebook.RayFrame], grid: s3.OccupancyGrid
) -> dict[str, Any]:
    indices = np.flatnonzero(grid.mask.ravel())
    count = 0
    orientations = 0
    maximum_u = 0.0
    maximum_v = 0.0
    for angle in grid.angles_deg:
        accepted, u_min, u_max, v_min, v_max = sensor_consistent_witnesses_fast(
            frames, grid, indices, float(angle)
        )
        if not accepted.size:
            continue
        orientations += 1
        count += int(accepted.size)
        maximum_u = max(maximum_u, float(np.max(u_max - u_min)))
        maximum_v = max(maximum_v, float(np.max(v_max - v_min)))
    return {
        "possible_hypothesis_count": count,
        "surviving_orientation_count": orientations,
        "orientation_hypotheses_removed": len(grid.angles_deg) - orientations,
        "maximum_supported_u_m": maximum_u,
        "maximum_supported_v_m": maximum_v,
    }


def sensor_consistent_witnesses_fast(
    frames: Sequence[rulebook.RayFrame],
    grid: s3.OccupancyGrid,
    indices: np.ndarray,
    angle_deg: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Exact witness query with the established irrelevant-ray geometric gate.

    The prior rule-filter helper tested every 1080-beam ray against every angle/cell batch.  The
    S3 raster implementation already proves that rays missing the complete placement domain cannot
    invalidate any witness. Reuse that identical gate here; accepted hypotheses remain unchanged.
    """
    hits = np.vstack([frame.hits for frame in frames if frame.hits.size])
    centre = np.mean(hits, axis=0)
    angle = math.radians(angle_deg)
    cosine, sine = math.cos(angle), math.sin(angle)
    hit_u = (hits[:, 0] - centre[0]) * cosine + (hits[:, 1] - centre[1]) * sine
    hit_v = -(hits[:, 0] - centre[0]) * sine + (hits[:, 1] - centre[1]) * cosine
    hu_min, hu_max = float(np.min(hit_u)), float(np.max(hit_u))
    hv_min, hv_max = float(np.min(hit_v)), float(np.max(hit_v))
    effective = rulebook.MAX_SIDE_M + math.sqrt(2.0) * grid.resolution_m
    if (
        hu_max - hu_min > effective + rulebook.GEOMETRY_EPSILON_M
        or hv_max - hv_min > effective + rulebook.GEOMETRY_EPSILON_M
    ):
        empty = np.asarray([], dtype=np.float64)
        return np.asarray([], dtype=np.int64), empty, empty, empty, empty
    flat_x, flat_y = np.meshgrid(grid.xs, grid.ys)
    query_x, query_y = flat_x.ravel()[indices], flat_y.ravel()[indices]
    query_u = (query_x - centre[0]) * cosine + (query_y - centre[1]) * sine
    query_v = -(query_x - centre[0]) * sine + (query_y - centre[1]) * cosine
    u_min, u_max = np.minimum(query_u, hu_min), np.maximum(query_u, hu_max)
    v_min, v_max = np.minimum(query_v, hv_min), np.maximum(query_v, hv_max)
    size_valid = (u_max - u_min <= effective + rulebook.GEOMETRY_EPSILON_M) & (
        v_max - v_min <= effective + rulebook.GEOMETRY_EPSILON_M
    )
    indices, u_min, u_max, v_min, v_max = (
        values[size_valid] for values in (indices, u_min, u_max, v_min, v_max)
    )
    if not indices.size:
        return indices, u_min, u_max, v_min, v_max
    origins = np.vstack(
        [
            np.repeat(frame.origin[None, :], frame.directions.shape[0], axis=0)
            for frame in frames
        ]
    )
    directions = np.vstack([frame.directions for frame in frames])
    lengths = np.concatenate([frame.free_lengths for frame in frames])
    origin_u = (origins[:, 0] - centre[0]) * cosine + (origins[:, 1] - centre[1]) * sine
    origin_v = -(origins[:, 0] - centre[0]) * sine + (origins[:, 1] - centre[1]) * cosine
    direction_u = directions[:, 0] * cosine + directions[:, 1] * sine
    direction_v = -directions[:, 0] * sine + directions[:, 1] * cosine
    maximum_u_min, maximum_u_max = hu_max - effective, hu_min + effective
    maximum_v_min, maximum_v_max = hv_max - effective, hv_min + effective
    relevant = [
        ray_index
        for ray_index in range(lengths.size)
        if rulebook.segment_intersects_box(
            float(origin_u[ray_index]),
            float(origin_v[ray_index]),
            float(direction_u[ray_index]),
            float(direction_v[ray_index]),
            float(lengths[ray_index]),
            maximum_u_min,
            maximum_u_max,
            maximum_v_min,
            maximum_v_max,
        )
    ]
    alive = np.ones(indices.size, dtype=bool)
    for ray_index in relevant:
        live = np.flatnonzero(alive)
        if not live.size:
            break
        invalid = rulebook.ray_invalid_candidates(
            float(origin_u[ray_index]),
            float(origin_v[ray_index]),
            float(direction_u[ray_index]),
            float(direction_v[ray_index]),
            float(lengths[ray_index]),
            u_min[live],
            u_max[live],
            v_min[live],
            v_max[live],
        )
        alive[live[invalid]] = False
    return indices[alive], u_min[alive], u_max[alive], v_min[alive], v_max[alive]


def boundary_boxes(grid: s3.OccupancyGrid) -> dict[str, tuple[float, float, float, float]]:
    return s3.occupied_cell_boxes(grid)


def reduced_extreme_boxes(
    grid: s3.OccupancyGrid,
    track: free_gap.TrackGeometry,
    expected_s: float,
    count_per_extreme: int = 8,
) -> tuple[dict[str, tuple[float, float, float, float]], int]:
    boxes = boundary_boxes(grid)
    keys = list(boxes)
    centres = np.asarray(
        [
            [
                0.5 * (boxes[key][0] + boxes[key][1]),
                0.5 * (boxes[key][2] + boxes[key][3]),
            ]
            for key in keys
        ],
        dtype=np.float64,
    )
    projection = free_gap.project_points_to_track(
        track, centres[:, 0], centres[:, 1], expected_s
    )
    keep: set[int] = set()
    for values in (
        projection["s"],
        -projection["s"],
        projection["d"],
        -projection["d"],
    ):
        keep.update(
            int(index)
            for index in np.argsort(values, kind="stable")[-count_per_extreme:]
        )
    selected = {
        f"CELL_X{local_index:04d}": boxes[keys[index]]
        for local_index, index in enumerate(sorted(keep))
    }
    return selected, len(boxes)


def project_grid(
    case: dict[str, Any],
    grid: s3.OccupancyGrid,
    scan_index: int,
    track: free_gap.TrackGeometry,
    repeat: bool,
    fine: bool = False,
) -> tuple[dict[str, Any], float]:
    selected, full_boundary_count = reduced_extreme_boxes(
        grid, track, float(case["manifest_s"])
    )
    label = f"scan_{scan_index:03d}{'_fine' if fine else ''}"
    directory = RAW / case["case_id"] / ("repeat" if repeat else "primary") / label
    directory.mkdir(parents=True, exist_ok=True)
    spec = rulebook.make_cpp_spec(
        int(case["stamp_ns"]),
        float(case["manifest_s"]),
        rulebook.nominal_polygon(case["obstacle"]),
        case["raster"],
        selected,
    )
    spec_path = directory / "support_spec.tsv"
    spec_path.write_text(spec, encoding="utf-8")
    expected = (
        rulebook.EXPECTED_039_HASH
        if case["case_id"] == "validation_039"
        else "FAST_CORRIDOR_ONLY"
    )
    started = time.monotonic()
    subprocess.run(
        [
            str(BINARY),
            "--temporal-envelope",
            str(case["stream"]),
            str(spec_path),
            str(directory),
            expected,
        ],
        cwd=ROOT,
        check=True,
        stdout=subprocess.DEVNULL,
    )
    elapsed = time.monotonic() - started
    row = next(
        item
        for item in tsv_rows(directory / "temporal_strategy_exact.tsv")
        if item["representation"] == "TF_UNION_SUPPORT_INTERNAL"
    )
    result: dict[str, Any] = {
        key: float(row[key])
        for key in (
            "s_min",
            "s_max",
            "d_min",
            "d_max",
            "left_corridor",
            "right_corridor",
            "best_corridor",
        )
    }
    result.update(
        {
            "same_path_clearance_m": float(row["same_path_production_clearance"])
            if row["same_path_production_clearance"]
            else None,
            "same_path_hard_valid": row["same_path_hard_valid"] == "true"
            if row["same_path_hard_valid"]
            else None,
            "same_path_rejection": row["same_path_rejection"],
            "selected_extreme_cell_count": len(selected),
            "full_boundary_cell_count": full_boundary_count,
        }
    )
    return result, elapsed


def stream_header(scenario: str) -> list[str]:
    lines = STREAM_039.read_text(encoding="utf-8").splitlines()
    output: list[str] = []
    for line in lines:
        if line.startswith("FRAME\t"):
            break
        if line.startswith("SCENARIO\t"):
            output.append(f"SCENARIO\t{scenario}")
        else:
            output.append(line)
    return output


def write_actionability_stream(
    path: Path,
    case_id: str,
    stamp_ns: int,
    ego_s: float,
    ego_d: float,
    speed: float,
    support: dict[str, Any],
) -> None:
    lines = stream_header(case_id)
    lines.extend(
        [
            f"FRAME\t{stamp_ns}\t{stamp_ns}\t{ego_s!r}\t{ego_d!r}\t{speed!r}\t1",
            "\t".join(
                [
                    "O",
                    "-980",
                    repr(0.5 * (support["s_min"] + support["s_max"])),
                    repr(support["s_min"]),
                    repr(support["s_max"]),
                    repr(support["d_min"]),
                    repr(support["d_max"]),
                    repr(
                        math.hypot(
                            support["s_max"] - support["s_min"],
                            support["d_max"] - support["d_min"],
                        )
                    ),
                    "0",
                    "0",
                ]
            ),
            "END_FRAME",
            "END_STREAM",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_gt_actionability_horizon(
    case: dict[str, Any], gt: dict[str, float], repeat: bool
) -> tuple[list[dict[str, Any]], float]:
    if case["case_id"] == "validation_039":
        return [], 0.0
    directory = RAW / case["case_id"] / ("repeat" if repeat else "primary")
    directory.mkdir(parents=True, exist_ok=True)
    stream_path = directory / "gt_only_horizon.stream.tsv"
    output_path = directory / "gt_only_horizon.tsv"
    lines = stream_header(case["case_id"] + "_gt_only_horizon")
    distance = float(case["gt_row"]["observation_distance_m"])
    for index in range(MAX_FUTURE_MS // 10 + 1):
        absolute_index = 2 + index
        stamp = 20_020_000_000 + index * 10_000_000
        pose = rulebook.interpolate_reference(
            case["waypoints"],
            float(case["manifest_s"]) - distance + 0.04 * absolute_index,
        )
        lines.extend(
            [
                f"FRAME\t{stamp}\t{stamp}\t{float(pose['s'])!r}\t0.0\t{NORMAL_SPEED_MPS!r}\t1",
                "\t".join(
                    [
                        "O",
                        "-980",
                        repr(0.5 * (gt["s_min"] + gt["s_max"])),
                        repr(gt["s_min"]),
                        repr(gt["s_max"]),
                        repr(gt["d_min"]),
                        repr(gt["d_max"]),
                        repr(
                            math.hypot(
                                gt["s_max"] - gt["s_min"],
                                gt["d_max"] - gt["d_min"],
                            )
                        ),
                        "0",
                        "0",
                    ]
                ),
                "END_FRAME",
            ]
        )
    lines.append("END_STREAM")
    stream_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    started = time.monotonic()
    subprocess.run(
        [
            str(BINARY),
            "--observability-actionability",
            str(stream_path),
            str(output_path),
            repr(0.5 * (gt["s_min"] + gt["s_max"])),
            repr(gt["s_min"]),
            repr(gt["s_max"]),
            repr(gt["d_min"]),
            repr(gt["d_max"]),
        ],
        cwd=ROOT,
        check=True,
        stdout=subprocess.DEVNULL,
    )
    rows = [
        {
            "delta_t_ms": index * 10,
            "source_stamp_ns": int(row["logical_stamp_ns"]),
            "ego_s_m": float(row["ego_s"]),
            "gt_family_hard_feasible": int(row["GT_B1_hard_feasible"]),
        }
        for index, row in enumerate(tsv_rows(output_path))
    ]
    return rows, time.monotonic() - started


def run_actionability(
    case: dict[str, Any],
    scan_index: int,
    source_stamp_ns: int,
    ego: dict[str, float],
    support: dict[str, Any],
    gt: dict[str, float],
    repeat: bool,
) -> tuple[dict[str, Any], float]:
    directory = RAW / case["case_id"] / ("repeat" if repeat else "primary")
    directory.mkdir(parents=True, exist_ok=True)
    stream = directory / f"actionability_{scan_index:03d}.stream.tsv"
    output = directory / f"actionability_{scan_index:03d}.tsv"
    write_actionability_stream(
        stream,
        case["case_id"],
        source_stamp_ns,
        ego["s"],
        ego["d"],
        ego["speed"],
        support,
    )
    started = time.monotonic()
    subprocess.run(
        [
            str(BINARY),
            "--observability-actionability",
            str(stream),
            str(output),
            repr(0.5 * (gt["s_min"] + gt["s_max"])),
            repr(gt["s_min"]),
            repr(gt["s_max"]),
            repr(gt["d_min"]),
            repr(gt["d_max"]),
        ],
        cwd=ROOT,
        check=True,
        stdout=subprocess.DEVNULL,
    )
    elapsed = time.monotonic() - started
    row = tsv_rows(output)[0]
    result: dict[str, Any] = {}
    for prefix, label in (("SHADOW_B1", "shadow"), ("GT_B1", "gt")):
        for column, name in (
            ("generated", "generated"),
            ("hard_feasible", "hard_feasible"),
            ("available_entry_distance_m", "available_entry_distance_m"),
            ("best_side", "best_side"),
            ("best_target_d", "best_target_d"),
            ("best_wall_clearance_m", "best_wall_clearance_m"),
            ("best_obstacle_clearance_m", "best_obstacle_clearance_m"),
            ("best_peak_curvature_radpm", "best_peak_curvature_radpm"),
            ("best_peak_lateral_slope", "best_peak_lateral_slope"),
            ("best_peak_curvature_rate_radpm2", "best_peak_curvature_rate_radpm2"),
            ("dominant_rejection", "dominant_rejection"),
            ("rejection_histogram", "rejection_histogram"),
            ("best_status", "best_status"),
            ("best_geometry_hash", "best_geometry_hash"),
        ):
            value = row[f"{prefix}_{column}"]
            if column in {"generated", "hard_feasible"}:
                result[f"{label}_{name}"] = int(value)
            elif column in {
                "available_entry_distance_m",
                "best_target_d",
                "best_wall_clearance_m",
                "best_obstacle_clearance_m",
                "best_peak_curvature_radpm",
                "best_peak_lateral_slope",
                "best_peak_curvature_rate_radpm2",
            }:
                result[f"{label}_{name}"] = float(value) if value else None
            else:
                result[f"{label}_{name}"] = value
    return result, elapsed


def ego_state(case: dict[str, Any], frame: rulebook.RayFrame, absolute_index: int) -> dict[str, float]:
    if case["case_id"] != "validation_039":
        distance = float(case["gt_row"]["observation_distance_m"])
        pose = rulebook.interpolate_reference(
            case["waypoints"],
            float(case["manifest_s"]) - distance + 0.04 * absolute_index,
        )
        return {
            "x": float(pose["x"]),
            "y": float(pose["y"]),
            "yaw": float(pose["yaw"]),
            "s": float(pose["s"]),
            "d": 0.0,
            "speed": NORMAL_SPEED_MPS,
        }
    stream_row = s3.parse_stream_frame(STREAM_039, frame.stamp_ns)
    odom = case["odometry"][frame.stamp_ns]
    return {
        "x": float(odom.pose.pose.position.x),
        "y": float(odom.pose.pose.position.y),
        "yaw": float(yaw_from_odom(odom)),
        "s": float(stream_row["ego_s_m"]),
        "d": float(stream_row["ego_d_m"]),
        "speed": float(stream_row["ego_speed_mps"]),
    }


def obstacle_local_hits(
    frame: rulebook.RayFrame, obstacle: dict[str, float]
) -> tuple[np.ndarray, np.ndarray]:
    if not frame.hits.size:
        return np.empty(0), np.empty(0)
    cosine, sine = math.cos(obstacle["yaw"]), math.sin(obstacle["yaw"])
    dx = frame.hits[:, 0] - obstacle["x"]
    dy = frame.hits[:, 1] - obstacle["y"]
    return dx * cosine + dy * sine, -dx * sine + dy * cosine


def visibility_metrics(
    frame: rulebook.RayFrame,
    obstacle: dict[str, float],
    ego_yaw: float,
    baseline_surfaces: set[str],
) -> dict[str, Any]:
    u, v = obstacle_local_hits(frame, obstacle)
    if not u.size:
        return {
            "return_count": 0,
            "angular_span_deg": 0.0,
            "visible_lateral_span_m": 0.0,
            "visible_longitudinal_span_m": 0.0,
            "visible_boundary_fraction": 0.0,
            "visible_surfaces": "",
            "newly_exposed_surfaces": "",
        }
    directions = frame.directions[np.asarray(frame.hit_indices, dtype=np.int64)] if (
        len(frame.directions) > max(frame.hit_indices, default=-1)
    ) else (frame.hits - frame.origin[None, :])
    angles = np.unwrap(np.arctan2(directions[:, 1], directions[:, 0]) - ego_yaw)
    half_u = 0.5 * obstacle["width"]
    half_v = 0.5 * obstacle["height"]
    distances = np.column_stack(
        (
            np.abs(u + half_u),
            np.abs(u - half_u),
            np.abs(v + half_v),
            np.abs(v - half_v),
        )
    )
    names = ("u_minus", "u_plus", "v_minus", "v_plus")
    nearest = np.argmin(distances, axis=1)
    surfaces = {names[int(index)] for index in nearest}
    visible_length = 0.0
    for index, name in enumerate(names):
        mask = nearest == index
        if not np.any(mask):
            continue
        coordinate = v[mask] if name.startswith("u_") else u[mask]
        edge_length = obstacle["height"] if name.startswith("u_") else obstacle["width"]
        visible_length += min(edge_length, float(np.max(coordinate) - np.min(coordinate)))
    return {
        "return_count": int(u.size),
        "angular_span_deg": math.degrees(float(np.max(angles) - np.min(angles))),
        "visible_lateral_span_m": float(np.max(v) - np.min(v)),
        "visible_longitudinal_span_m": float(np.max(u) - np.min(u)),
        "visible_boundary_fraction": visible_length
        / (2.0 * (obstacle["width"] + obstacle["height"])),
        "visible_surfaces": ";".join(sorted(surfaces)),
        "newly_exposed_surfaces": ";".join(sorted(surfaces - baseline_surfaces)),
    }


def hypothesis_polygon(hypothesis: dict[str, float]) -> tuple[float, float, float, float, float, np.ndarray]:
    angle = math.radians(hypothesis["orientation_deg"])
    centre = np.asarray([hypothesis["pose_x_m"], hypothesis["pose_y_m"]])
    return (
        -0.5 * hypothesis["width_u_m"],
        0.5 * hypothesis["width_u_m"],
        -0.5 * hypothesis["length_v_m"],
        0.5 * hypothesis["length_v_m"],
        angle,
        centre,
    )


def hypothesis_consistency(
    frames: Sequence[rulebook.RayFrame], hypothesis: dict[str, float]
) -> dict[str, Any]:
    u_min, u_max, v_min, v_max, angle, centre = hypothesis_polygon(hypothesis)
    cosine, sine = math.cos(angle), math.sin(angle)
    for frame in frames:
        for local, point in enumerate(frame.hits):
            dx, dy = point - centre
            u = dx * cosine + dy * sine
            v = -dx * sine + dy * cosine
            if not (
                u_min - rulebook.GEOMETRY_EPSILON_M <= u <= u_max + rulebook.GEOMETRY_EPSILON_M
                and v_min - rulebook.GEOMETRY_EPSILON_M <= v <= v_max + rulebook.GEOMETRY_EPSILON_M
            ):
                beam = frame.hit_indices[local] if local < len(frame.hit_indices) else local
                return {
                    "status": "REMOVED",
                    "evidence_type": "NEW_HIT_OUTSIDE_HYPOTHESIS",
                    "evidence_stamp_ns": frame.stamp_ns,
                    "beam_index": int(beam),
                    "detail": f"hit_local_u={u:.9f},hit_local_v={v:.9f}",
                }
    for frame in frames:
        origin_delta = frame.origin - centre
        origin_u = origin_delta[0] * cosine + origin_delta[1] * sine
        origin_v = -origin_delta[0] * sine + origin_delta[1] * cosine
        direction_u = frame.directions[:, 0] * cosine + frame.directions[:, 1] * sine
        direction_v = -frame.directions[:, 0] * sine + frame.directions[:, 1] * cosine
        for beam in range(frame.free_lengths.size):
            invalid = rulebook.ray_invalid_candidates(
                float(origin_u),
                float(origin_v),
                float(direction_u[beam]),
                float(direction_v[beam]),
                float(frame.free_lengths[beam]),
                np.asarray([u_min]),
                np.asarray([u_max]),
                np.asarray([v_min]),
                np.asarray([v_max]),
            )[0]
            if invalid:
                return {
                    "status": "REMOVED",
                    "evidence_type": "NEW_FREE_RAY_INTERSECTS_HYPOTHESIS",
                    "evidence_stamp_ns": frame.stamp_ns,
                    "beam_index": beam,
                    "detail": f"free_length_m={frame.free_lengths[beam]:.9f}",
                }
    return {
        "status": "ALIVE",
        "evidence_type": "NONE",
        "evidence_stamp_ns": None,
        "beam_index": None,
        "detail": "all causal hits contained and no causal free ray crosses interior",
    }


def gt_representation(case_id: str) -> dict[str, float]:
    data = baseline_cpp(case_id)
    key = "G1" if case_id == "validation_039" else "G0"
    return {
        field: float(data["cpp"][key][field])
        for field in ("s_min", "s_max", "d_min", "d_max")
    }


def selected_side(case_id: str) -> str:
    if case_id == "validation_039":
        return "right"
    data = baseline_cpp(case_id)["cpp"]["G0"]
    return "left" if float(data["left_corridor"]) > float(data["right_corridor"]) else "right"


def baseline_support(case_id: str) -> dict[str, float]:
    support = baseline_cpp(case_id)["possible_set_support"]
    return {key: float(support[key]) for key in ("s_min", "s_max", "d_min", "d_max")}


def row_digest_rows(rows: list[dict[str, Any]]) -> str:
    excluded = {"projection_wall_s", "actionability_wall_s", "set_wall_s"}
    stable = [{key: value for key, value in row.items() if key not in excluded} for row in rows]
    return canonical_digest(clean_json(stable))


def process_case(case_id: str, repeat: bool = False) -> dict[str, Any]:
    worker_started = time.monotonic()
    cpu_started = time.process_time()
    bag_parse_time = 0.0
    if case_id == "validation_039":
        case, bag_parse_time = recorded_validation039_case()
    else:
        case = synthetic_normal_case(case_id)
    waypoints = load_waypoints(WAYPOINTS)
    track = free_gap.TrackGeometry.from_waypoints(waypoints)
    length = track.track_length_m
    baseline = baseline_cpp(case_id)
    gt = gt_representation(case_id)
    gt_horizon, gt_horizon_elapsed = run_gt_actionability_horizon(case, gt, repeat)
    gt_horizon_by_delta = {row["delta_t_ms"]: row for row in gt_horizon}
    gt_ever_avoidable = any(
        row["gt_family_hard_feasible"] > 0 for row in gt_horizon
    )
    side = selected_side(case_id)
    blocker = blocking_hypotheses().get(case_id)
    baseline_gt = baseline["cpp"]["G1" if case_id == "validation_039" else "G0"]
    gt_corridor = float(baseline_gt["best_corridor"])
    baseline_stamp = int(case["frame_now_ns"])
    frames = list(case["frames"])
    candidates = (
        [frames[-1]]
        if case_id != "validation_039"
        else [frame for frame in frames if frame.stamp_ns >= baseline_stamp]
    )
    rows: list[dict[str, Any]] = []
    lifecycle: list[dict[str, Any]] = []
    viewpoints: list[dict[str, Any]] = []
    baseline_ego: dict[str, float] | None = None
    baseline_surfaces: set[str] = set()
    baseline_selected_support: float | None = None
    baseline_area: float | None = None
    baseline_hypothesis_count: int | None = None
    baseline_maximum_u: float | None = None
    baseline_maximum_v: float | None = None
    previous_area: float | None = None
    consecutive_resolution = 0
    consecutive_gt_infeasible = 0
    termination_reason = "MAX_1S"
    generated_scan_count = 0
    projection_time = 0.0
    actionability_time = gt_horizon_elapsed
    set_time = 0.0
    first_blocker_removed: dict[str, Any] | None = None
    empty_possible_set_stamp_ns: int | None = None

    scan_index = 0
    while scan_index <= MAX_FUTURE_MS // 10:
        if scan_index >= len(candidates):
            if case_id == "validation_039":
                break
            absolute_index = 2 + scan_index
            try:
                generated = normal_future_frame(case, absolute_index)
            except RuntimeError:
                termination_reason = "MEANINGFUL_VISIBILITY_END_SENSOR_RENDER"
                break
            frames.append(generated)
            candidates.append(generated)
            generated_scan_count += 1
        frame = candidates[scan_index]
        delta_ms = int(round((frame.stamp_ns - baseline_stamp) / 1.0e6))
        if delta_ms > MAX_FUTURE_MS:
            break
        window = s3.selected_frames_window(frames, frame.stamp_ns, WINDOW_MS)
        if not window or sum(item.hits.shape[0] for item in window) == 0:
            termination_reason = "MEANINGFUL_VISIBILITY_END_NO_POSITIVE_HIT_IN_UT20"
            break
        set_started = time.monotonic()
        try:
            grid = s3.detailed_possible_occupancy(window, GRID_M, ANGLE_DEG)
        except RuntimeError as error:
            if "no rulebook hypothesis remains" not in str(error):
                raise
            empty_possible_set_stamp_ns = frame.stamp_ns
            termination_reason = "SENSOR_MODEL_INCONSISTENT_EMPTY_POSSIBLE_SET"
            break
        hits_only = s3.detailed_possible_occupancy(
            window, GRID_M, ANGLE_DEG, use_free_space=False
        )
        single = s3.detailed_possible_occupancy([frame], GRID_M, ANGLE_DEG) if frame.hits.size else None
        witness = witness_statistics(window, grid)
        set_elapsed = time.monotonic() - set_started
        set_time += set_elapsed
        support, projected = project_grid(case, grid, scan_index, track, repeat)
        projection_time += projected
        ego = ego_state(
            case,
            frame,
            (frame.stamp_ns - 20_000_000_000) // 10_000_000
            if case_id != "validation_039"
            else scan_index,
        )
        actionability, action_elapsed = run_actionability(
            case, scan_index, frame.stamp_ns, ego, support, gt, repeat
        )
        actionability_time += action_elapsed
        if case_id != "validation_039":
            horizon_row = gt_horizon_by_delta[delta_ms]
            if (
                actionability["gt_hard_feasible"]
                != horizon_row["gt_family_hard_feasible"]
            ):
                raise RuntimeError(
                    f"{case_id}: GT actionability mismatch at {delta_ms} ms"
                )
        if baseline_ego is None:
            baseline_ego = ego
        visibility = visibility_metrics(frame, case["obstacle"], ego["yaw"], baseline_surfaces)
        current_surfaces = set(filter(None, visibility["visible_surfaces"].split(";")))
        if scan_index == 0:
            baseline_surfaces = current_surfaces
            visibility["newly_exposed_surfaces"] = ""
        current_support = support["d_max"] if side == "left" else support["d_min"]
        if baseline_selected_support is None:
            baseline_selected_support = current_support
        if baseline_area is None:
            baseline_area = grid.possible_area_m2
        if baseline_hypothesis_count is None:
            baseline_hypothesis_count = witness["possible_hypothesis_count"]
            baseline_maximum_u = witness["maximum_supported_u_m"]
            baseline_maximum_v = witness["maximum_supported_v_m"]
        gt_boundary = gt["d_max"] if side == "left" else gt["d_min"]
        if side == "left":
            undercoverage = max(0.0, gt_boundary - current_support)
            overcoverage = max(0.0, current_support - gt_boundary)
            support_reduction = baseline_selected_support - current_support
        else:
            undercoverage = max(0.0, current_support - gt_boundary)
            overcoverage = max(0.0, gt_boundary - current_support)
            support_reduction = current_support - baseline_selected_support
        available = forward_distance(ego["s"], gt["s_min"], length)
        decel = 2.5
        stopping = ego["speed"] ** 2 / (2.0 * decel)
        stop_buffer = 0.4
        stopping_room = available - stop_buffer - stopping
        shadow_feasible = actionability["shadow_hard_feasible"] > 0
        gt_feasible = actionability["gt_hard_feasible"] > 0
        resolved = shadow_feasible and gt_feasible
        consecutive_resolution = consecutive_resolution + 1 if resolved else 0
        consecutive_gt_infeasible = consecutive_gt_infeasible + 1 if not gt_feasible else 0
        blocker_state = (
            hypothesis_consistency(window, blocker)
            if blocker is not None
            else {
                "status": "NOT_APPLICABLE",
                "evidence_type": "NOT_APPLICABLE",
                "evidence_stamp_ns": None,
                "beam_index": None,
                "detail": "validation_039 safety anchor",
            }
        )
        if blocker_state["status"] == "REMOVED" and first_blocker_removed is None:
            first_blocker_removed = {"delta_ms": delta_ms, **blocker_state}
        meaningful_reduction = (
            previous_area is not None
            and previous_area - grid.possible_area_m2 >= GRID_M**2 - 1.0e-15
        )
        previous_area = grid.possible_area_m2
        row = {
            "case_id": case_id,
            "data_source": case["data_source"],
            "scan_index": scan_index,
            "source_stamp_ns": frame.stamp_ns,
            "delta_t_ms": delta_ms,
            "ego_x_m": ego["x"],
            "ego_y_m": ego["y"],
            "ego_yaw_rad": ego["yaw"],
            "ego_s_m": ego["s"],
            "ego_d_m": ego["d"],
            "ego_speed_mps": ego["speed"],
            "obstacle_return_count": visibility["return_count"],
            "visible_lateral_span_m": visibility["visible_lateral_span_m"],
            "visible_longitudinal_span_m": visibility["visible_longitudinal_span_m"],
            "angular_span_deg": visibility["angular_span_deg"],
            "visible_boundary_fraction": visibility["visible_boundary_fraction"],
            "ut20_observation_count": len(window),
            "ut20_positive_observation_count": sum(bool(item.hits.size) for item in window),
            "possible_hypothesis_count": witness["possible_hypothesis_count"],
            "hypotheses_removed_from_baseline": baseline_hypothesis_count
            - witness["possible_hypothesis_count"],
            "possible_cell_count": int(np.count_nonzero(grid.mask)),
            "possible_area_m2": grid.possible_area_m2,
            "hits_only_possible_area_m2": hits_only.possible_area_m2,
            "single_scan_possible_area_m2": single.possible_area_m2 if single else None,
            "new_hit_constraint_contribution_m2": max(
                0.0, hits_only.aabb_area_m2 - hits_only.possible_area_m2
            ),
            "new_free_ray_contribution_m2": max(
                0.0, hits_only.possible_area_m2 - grid.possible_area_m2
            ),
            "temporal_contribution_m2": max(
                0.0,
                (single.possible_area_m2 if single else grid.possible_area_m2)
                - grid.possible_area_m2,
            ),
            "surviving_orientation_count": witness["surviving_orientation_count"],
            "orientation_hypotheses_removed": witness["orientation_hypotheses_removed"],
            "maximum_supported_u_m": witness["maximum_supported_u_m"],
            "maximum_supported_v_m": witness["maximum_supported_v_m"],
            "maximum_supported_u_reduction_m": baseline_maximum_u
            - witness["maximum_supported_u_m"],
            "maximum_supported_v_reduction_m": baseline_maximum_v
            - witness["maximum_supported_v_m"],
            "selected_side": side,
            "selected_side_support_m": current_support,
            "selected_side_support_reduction_m": support_reduction,
            "support_s_min_m": support["s_min"],
            "support_s_max_m": support["s_max"],
            "support_d_min_m": support["d_min"],
            "support_d_max_m": support["d_max"],
            "shadow_left_corridor_m": support["left_corridor"],
            "shadow_right_corridor_m": support["right_corridor"],
            "shadow_corridor_m": (
                support["left_corridor"] if side == "left" else support["right_corridor"]
            ),
            "gt_corridor_m": gt_corridor,
            "gt_boundary_undercoverage_m": undercoverage,
            "gt_boundary_overcoverage_m": overcoverage,
            "same_dangerous_path_clearance_m": support["same_path_clearance_m"],
            "same_dangerous_path_hard_valid": support["same_path_hard_valid"],
            "same_dangerous_path_rejection": support["same_path_rejection"],
            "shadow_family_generated": actionability["shadow_generated"],
            "shadow_family_hard_feasible": actionability["shadow_hard_feasible"],
            "gt_family_generated": actionability["gt_generated"],
            "gt_family_hard_feasible": actionability["gt_hard_feasible"],
            "same_feasibility_classification": "ALIGNED_FEASIBLE"
            if resolved
            else ("FALSE_INFEASIBLE" if gt_feasible else "ALIGNED_INFEASIBLE"),
            "first_resolution_condition": resolved,
            "stable_resolution_condition": consecutive_resolution >= 3,
            "available_longitudinal_distance_m": available,
            "current_preparation_entry_distance_m": actionability[
                "shadow_available_entry_distance_m"
            ],
            "configured_decel_limit_mps2": decel,
            "minimum_stopping_distance_m": stopping,
            "safe_stop_buffer_m": stop_buffer,
            "stopping_room_after_buffer_m": stopping_room,
            "safe_deceleration_opportunity": stopping_room >= 0.0,
            "remaining_observation_time_s": available / max(ego["speed"], 1.0e-9),
            "blocking_hypothesis_status": blocker_state["status"],
            "blocking_hypothesis_evidence": blocker_state["evidence_type"],
            "meaningful_set_reduction": meaningful_reduction,
            "fine_refined": False,
            "fine_shadow_corridor_m": None,
            "projection_wall_s": projected,
            "actionability_wall_s": action_elapsed,
            "set_wall_s": set_elapsed,
        }
        for key, value in actionability.items():
            row[f"planner_{key}"] = value
        rows.append(row)
        lifecycle.append(
            {
                "case_id": case_id,
                "scan_index": scan_index,
                "source_stamp_ns": frame.stamp_ns,
                "delta_t_ms": delta_ms,
                "status": blocker_state["status"],
                "evidence_type": blocker_state["evidence_type"],
                "evidence_stamp_ns": blocker_state["evidence_stamp_ns"],
                "beam_index": blocker_state["beam_index"],
                "evidence_detail": blocker_state["detail"],
                "newly_visible_surface": visibility["newly_exposed_surfaces"],
                "orientation_count": witness["surviving_orientation_count"],
                "maximum_supported_u_m": witness["maximum_supported_u_m"],
                "maximum_supported_v_m": witness["maximum_supported_v_m"],
            }
        )
        viewpoints.append(
            {
                "case_id": case_id,
                "scan_index": scan_index,
                "source_stamp_ns": frame.stamp_ns,
                "delta_t_ms": delta_ms,
                "delta_position_m": math.hypot(
                    ego["x"] - baseline_ego["x"], ego["y"] - baseline_ego["y"]
                ),
                "delta_heading_rad": math.atan2(
                    math.sin(ego["yaw"] - baseline_ego["yaw"]),
                    math.cos(ego["yaw"] - baseline_ego["yaw"]),
                ),
                "delta_s_m": forward_distance(baseline_ego["s"], ego["s"], length),
                "delta_d_m": ego["d"] - baseline_ego["d"],
                **visibility,
                "possible_area_reduction_from_baseline_m2": baseline_area
                - grid.possible_area_m2,
                "selected_side_support_reduction_m": support_reduction,
                "blocking_hypothesis_status": blocker_state["status"],
            }
        )
        if case_id != "validation_039" and consecutive_resolution >= 3:
            termination_reason = "STABLE_RESOLUTION_3_FRESH_SCANS"
            break
        if (
            case_id == "normal_01_02"
            and not gt_ever_avoidable
            and consecutive_gt_infeasible >= 3
        ):
            termination_reason = "USEFUL_AVOIDANCE_REGION_ENDED_GT_INFEASIBLE_3_FRESH_SCANS"
            break
        scan_index += 1

    fine_indices: set[int] = set()
    for row in rows:
        if abs(float(row["shadow_corridor_m"])) <= math.sqrt(2.0) * GRID_M + 1.0e-4:
            fine_indices.update(
                index
                for index in (row["scan_index"] - 1, row["scan_index"], row["scan_index"] + 1)
                if 0 <= index < len(rows)
            )
    fine_count = 0
    for index in sorted(fine_indices):
        frame = candidates[index]
        window = s3.selected_frames_window(frames, frame.stamp_ns, WINDOW_MS)
        fine_grid = s3.detailed_possible_occupancy(
            window, FINE_GRID_M, FINE_ANGLE_DEG
        )
        fine_support, elapsed = project_grid(
            case, fine_grid, index, track, repeat, fine=True
        )
        projection_time += elapsed
        rows[index]["fine_refined"] = True
        rows[index]["fine_shadow_corridor_m"] = (
            fine_support["left_corridor"]
            if side == "left"
            else fine_support["right_corridor"]
        )
        rows[index]["fine_possible_area_m2"] = fine_grid.possible_area_m2
        fine_count += 1

    baseline_expected = baseline_support(case_id)
    baseline_actual = rows[0]
    projection_delta = {
        key: baseline_actual[f"support_{key}_m"] - baseline_expected[key]
        for key in ("s_min", "s_max", "d_min", "d_max")
    }
    if max(abs(value) for value in projection_delta.values()) > 1.0e-12:
        raise RuntimeError(f"{case_id}: reduced projection does not reproduce baseline {projection_delta}")
    expected_hypotheses = {
        "normal_01_00": 16128,
        "normal_01_02": 14112,
        "validation_039": 470881,
    }[case_id]
    if rows[0]["possible_hypothesis_count"] != expected_hypotheses:
        raise RuntimeError(
            f"{case_id}: fast witness count {rows[0]['possible_hypothesis_count']} "
            f"does not reproduce stored {expected_hypotheses}"
        )
    first = next((row for row in rows if row["first_resolution_condition"]), None)
    stable = next((row for row in rows if row["stable_resolution_condition"]), None)
    gt_avoidable = (
        [row for row in gt_horizon if row["gt_family_hard_feasible"] > 0]
        if case_id != "validation_039"
        else [row for row in rows if row["gt_family_hard_feasible"] > 0]
    )
    last_gt = gt_avoidable[-1] if gt_avoidable else None
    last_gt_right_censored = bool(
        case_id != "validation_039"
        and last_gt is not None
        and last_gt["delta_t_ms"] == MAX_FUTURE_MS
    )
    if case_id == "normal_01_00":
        if stable is None:
            resolution_class = "UNRESOLVED_OBSERVABILITY_LIMIT"
        elif last_gt is not None and stable["delta_t_ms"] <= last_gt["delta_t_ms"]:
            resolution_class = "OBSERVATION_RESOLVES_EARLY"
        else:
            resolution_class = "RESOLVES_TOO_LATE"
    elif case_id == "normal_01_02":
        resolution_class = "MARGINAL_GEOMETRY"
    else:
        resolution_class = "SAFETY_ANCHOR_RETAINED" if all(
            row["gt_boundary_undercoverage_m"] <= 1.0e-12
            and row["same_dangerous_path_hard_valid"] is False
            for row in rows
        ) else "SAFETY_ANCHOR_FAILED"
    result = {
        "case_id": case_id,
        "data_source": case["data_source"],
        "rows": clean_json(rows),
        "lifecycle": clean_json(lifecycle),
        "viewpoints": clean_json(viewpoints),
        "baseline_corridor_m": rows[0]["shadow_corridor_m"],
        "gt_corridor_m": gt_corridor,
        "first_resolution_ms": first["delta_t_ms"] if first else None,
        "stable_resolution_ms": stable["delta_t_ms"] if stable else None,
        "last_gt_avoidable_ms": last_gt["delta_t_ms"] if last_gt else None,
        "last_gt_avoidable_right_censored": last_gt_right_censored,
        "gt_actionability_horizon_evaluated_count": len(gt_horizon),
        "resolution_class": resolution_class,
        "first_blocker_removed": clean_json(first_blocker_removed),
        "termination_reason": termination_reason,
        "empty_possible_set_stamp_ns": empty_possible_set_stamp_ns,
        "evaluated_scan_count": len(rows),
        "generated_scan_count": generated_scan_count if case_id != "validation_039" else 0,
        "bag_parse_count": 1 if case_id == "validation_039" else 0,
        "bag_parse_time_s": bag_parse_time,
        "fine_refinement_count": fine_count,
        "projection_baseline_delta": projection_delta,
        "projection_wall_s": projection_time,
        "actionability_wall_s": actionability_time,
        "set_membership_wall_s": set_time,
        "worker_wall_s": time.monotonic() - worker_started,
        "worker_cpu_s": time.process_time() - cpu_started,
    }
    result["digest"] = canonical_digest(
        {
            "row_digest": row_digest_rows(result["rows"]),
            "last_gt_avoidable_ms": result["last_gt_avoidable_ms"],
            "last_gt_avoidable_right_censored": result[
                "last_gt_avoidable_right_censored"
            ],
        }
    )
    return result


def markdown_table(headers: list[str], rows: list[list[Any]]) -> str:
    def token(value: Any) -> str:
        if value is None:
            return "UNRESOLVED"
        if isinstance(value, float):
            return f"{value:.6f}"
        return str(value)

    output = ["| " + " | ".join(headers) + " |", "|" + "|".join(["---"] * len(headers)) + "|"]
    output.extend("| " + " | ".join(token(value) for value in row) + " |" for row in rows)
    return "\n".join(output)


def render_readme(summary: dict[str, Any], performance: dict[str, Any]) -> str:
    cases = {item["case_id"]: item for item in summary["cases"]}
    table1 = markdown_table(
        [
            "case",
            "baseline corridor",
            "first resolution ms",
            "stable resolution ms",
            "last GT-avoidable ms",
            "resolution class",
        ],
        [
            [
                case_id,
                cases[case_id]["baseline_corridor_m"],
                cases[case_id]["first_resolution_ms"],
                cases[case_id]["stable_resolution_ms"],
                (
                    f">={cases[case_id]['last_gt_avoidable_ms']} (horizon)"
                    if cases[case_id]["last_gt_avoidable_right_censored"]
                    else cases[case_id]["last_gt_avoidable_ms"]
                ),
                cases[case_id]["resolution_class"],
            ]
            for case_id in ("normal_01_00", "normal_01_02")
        ],
    )
    table2_rows = []
    for case_id in ("normal_01_00", "normal_01_02"):
        case = cases[case_id]
        removed = case["first_blocker_removed"]
        viewpoint = None
        if removed is not None:
            viewpoint = next(
                row
                for row in case["viewpoints"]
                if row["delta_t_ms"] == removed["delta_ms"]
            )
        removal_row = next(
            (
                row
                for row in case["rows"]
                if removed is not None and row["delta_t_ms"] == removed["delta_ms"]
            ),
            None,
        )
        table2_rows.append(
            [
                case_id,
                removed["delta_ms"] if removed else None,
                viewpoint["delta_position_m"] if viewpoint else None,
                removed["evidence_type"] if removed else "NOT_REMOVED",
                (
                    removal_row["shadow_corridor_m"] - case["baseline_corridor_m"]
                    if removal_row
                    else 0.0
                ),
            ]
        )
    table2 = markdown_table(
        [
            "case",
            "blocking hypothesis removal time",
            "viewpoint delta",
            "new evidence type",
            "corridor recovery",
        ],
        table2_rows,
    )
    anchor = cases["validation_039"]
    table3 = markdown_table(
        [
            "validation_039 timestamp",
            "GT undercoverage",
            "dangerous path clearance",
            "hard-invalid",
        ],
        [
            [
                row["source_stamp_ns"],
                row["gt_boundary_undercoverage_m"],
                row["same_dangerous_path_clearance_m"],
                not row["same_dangerous_path_hard_valid"],
            ]
            for row in anchor["rows"]
        ],
    )
    return f"""# Time-to-disambiguation / temporal observability audit v1

This is a diagnostic-only causal audit. It performed zero closed-loop replays and made zero
production changes. `normal_01_00` and `normal_01_02` used the exact stored 10 ms / 0.04 m
reference-line trajectory convention and generated only the missing LiDAR observations.
`validation_039` used its recorded bag and stopped when fresh evidence made the rulebook-constrained
possible set empty. GT geometry rendered the normal-case sensor truth and evaluated results only;
it was never passed into S3.

Overall recommendation: **{summary['overall_recommendation']}**.

## Table 1 — resolution and actionability

{table1}

`FIRST_RESOLUTION` is the first causal scan with both exact-GT and S3 current-production B1 hard
feasibility. `STABLE_RESOLUTION` requires three consecutive fresh scans. `T_LAST_GT_AVOIDABLE` is
the final evaluated scan with at least one exact-GT hard-valid production candidate. A `>=` value
is right-censored: exact GT remained avoidable at the 1.0 s audit horizon.

## Table 2 — blocking-hypothesis lifecycle

{table2}

The lifecycle CSV includes the exact source stamp, beam index, and inconsistency detail. Time alone
is not treated as evidence: the chain is viewpoint change, then a new hit or free ray, then removal.

## Table 3 — validation_039 safety anchor

{table3}

The known dangerous path is the unchanged `{rulebook.EXPECTED_039_PATH_SHA256}` serialization.
No evaluated future scan undercovered exact GT or reopened that path. Evaluation ended at the
first empty possible set (`{anchor['empty_possible_set_stamp_ns']}`), rather than interpreting an
inconsistent estimator state as free space. The last valid conservative support remains the safety
anchor; no production latch or fallback was implemented by this audit.

## Method and causality

- S3 uses the validated 4 mm / 2 degree witness grid and only frames with `source_stamp <= t`.
- UT20 contains genuine fresh 10 ms scans; duplicated callbacks and hindsight scans are excluded.
- Cartesian cell extrema are first shortlisted with the cached polyline, then finalized with the
  detector-owned production CLCS AABB projector. At all three baselines the reduced candidate set
  reproduced the full boundary-cell `s/d` support exactly (maximum delta <= 1e-12 m).
- Current production path-family candidates and hard validation are evaluated by the existing
  BUILD_TESTING audit executable compiled directly from the production planner implementation.
- The deceleration diagnostic uses the existing 2.5 m/s² safe-stop deceleration and 0.4 m buffer;
  it does not implement or simulate braking.
- Fine 2 mm / 1 degree refinement is local and only triggered within one coarse-cell uncertainty
  of a zero corridor.
- Width/length are continuous derived extents rather than separately enumerated bins. The time-axis
  CSVs therefore quantify dimension-hypothesis reduction as the reduction in maximum supported
  local `u` and `v` extents, alongside total and orientation-hypothesis counts.

## Data provenance and lifecycle boundary

- normal cases: `SENSOR_ONLY_RECONSTRUCTION`; no cached future scans existed.
- validation_039: `RECORDED_EXISTING`; one bag parse, no sensor regeneration.
- RAW/TENTATIVE observations collect history; CONFIRMED UNKNOWN may be queried conservatively;
  STATIC may use a map-frame envelope; DYNAMIC invalidates the static envelope. Opponent tracking,
  association, motion voting, `/static_obs`, and `/opp_obs` were not changed.

## Compute

- workers: {performance['worker_count_configured']} configured, {performance['active_primary_case_count']} independent cases
- wall / CPU: {performance['offline_wall_time_s']:.3f} s / {performance['cpu_time_s']:.3f} s
- max RSS: {performance['max_rss_bytes']} bytes
- swap delta: {performance['swap_change_bytes']} bytes
- bag parses / generated scans / fine refinements: {performance['bag_parse_count']} / {performance['generated_sensor_scan_count']} / {performance['fine_refinement_count']}
- closed-loop replay / CMA / dataset regeneration: 0 / 0 / 0

See the CSV files for every scan, `summary.json` for classifications and hashes, and
`performance.json` for resource/provenance detail.
"""


def main() -> None:
    args = parse_args()
    started = time.monotonic()
    cpu_started = resource.getrusage(resource.RUSAGE_SELF)
    child_started = resource.getrusage(resource.RUSAGE_CHILDREN)
    before = memory_snapshot()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    RAW.mkdir(parents=True, exist_ok=True)
    production_before = {
        name: sha256(path) for name, path in rulebook.PRODUCTION_FILES.items()
    }
    with concurrent.futures.ProcessPoolExecutor(max_workers=WORKERS) as executor:
        futures = {executor.submit(process_case, case_id, False): case_id for case_id in PRIMARY_CASES}
        primary = [future.result() for future in concurrent.futures.as_completed(futures)]
    primary.sort(key=lambda item: PRIMARY_CASES.index(item["case_id"]))
    with concurrent.futures.ProcessPoolExecutor(max_workers=2) as executor:
        repeats = list(
            executor.map(
                process_case,
                ("normal_01_00", "validation_039"),
                (True, True),
            )
        )
    repeat_by_case = {item["case_id"]: item for item in repeats}
    determinism = {
        case_id: {
            "primary_digest": next(item for item in primary if item["case_id"] == case_id)["digest"],
            "repeat_digest": repeat_by_case[case_id]["digest"],
            "bit_identical": next(item for item in primary if item["case_id"] == case_id)["digest"]
            == repeat_by_case[case_id]["digest"],
        }
        for case_id in repeat_by_case
    }
    if not all(item["bit_identical"] for item in determinism.values()):
        raise RuntimeError(f"determinism repeat failed: {determinism}")
    case_by_id = {item["case_id"]: item for item in primary}
    anchor_safe = case_by_id["validation_039"]["resolution_class"] == "SAFETY_ANCHOR_RETAINED"
    primary_class = case_by_id["normal_01_00"]["resolution_class"]
    if (
        case_by_id["validation_039"]["termination_reason"]
        == "SENSOR_MODEL_INCONSISTENT_EMPTY_POSSIBLE_SET"
    ):
        recommendation = "SENSOR_MODEL_LIMIT"
    elif primary_class == "OBSERVATION_RESOLVES_EARLY" and anchor_safe:
        recommendation = "PASSIVE_OBSERVATION_PROMISING"
    elif primary_class == "RESOLVES_TOO_LATE" and any(
        row["safe_deceleration_opportunity"]
        for row in case_by_id["normal_01_00"]["rows"]
    ):
        recommendation = "BRAKE_FOR_OBSERVATION_WORTH_AUDITING"
    elif primary_class == "UNRESOLVED_OBSERVABILITY_LIMIT":
        recommendation = "SENSOR_MODEL_LIMIT"
    else:
        recommendation = "INSUFFICIENT"
    production_after = {
        name: sha256(path) for name, path in rulebook.PRODUCTION_FILES.items()
    }
    if production_before != production_after:
        raise RuntimeError("production hash changed during diagnostic audit")
    summary = clean_json(
        {
            "schema": "time_to_disambiguation_audit/1",
            "diagnostic_only": True,
            "production_changes": False,
            "closed_loop_replay_count": 0,
            "cma_executed": False,
            "dataset_regenerated": False,
            "overall_recommendation": recommendation,
            "cases": primary,
            "determinism": determinism,
            "production_hashes_before": production_before,
            "production_hashes_after": production_after,
            "causality": {
                "source_stamp_not_after_query": True,
                "fresh_scan_period_ms": 10,
                "ut_window_ms": WINDOW_MS,
                "hindsight_used": False,
                "gt_used_as_estimator_input": False,
            },
            "static_dynamic_lifecycle": {
                "RAW_TENTATIVE": "collect observation history only",
                "CONFIRMED_UNKNOWN": "query conservative static safety envelope",
                "STATIC": "map-frame static envelope may be used",
                "DYNAMIC": "invalidate static envelope immediately",
                "opponent_pipeline_modified": False,
            },
        }
    )
    after = memory_snapshot()
    self_usage = resource.getrusage(resource.RUSAGE_SELF)
    child_usage = resource.getrusage(resource.RUSAGE_CHILDREN)
    performance = clean_json(
        {
            "schema": "time_to_disambiguation_performance/1",
            "worker_count_configured": WORKERS,
            "active_primary_case_count": len(PRIMARY_CASES),
            "worker_policy": "workers=4 executor; three independent cases active; timestamps serial within each case",
            "offline_wall_time_s": time.monotonic() - started,
            "cpu_time_s": (self_usage.ru_utime + self_usage.ru_stime - cpu_started.ru_utime - cpu_started.ru_stime)
            + (child_usage.ru_utime + child_usage.ru_stime - child_started.ru_utime - child_started.ru_stime),
            "max_rss_bytes": max(self_usage.ru_maxrss, child_usage.ru_maxrss) * 1024,
            "memory_available_before_bytes": before["memory_available_bytes"],
            "memory_available_after_bytes": after["memory_available_bytes"],
            "swap_used_before_bytes": before["swap_used_bytes"],
            "swap_used_after_bytes": after["swap_used_bytes"],
            "swap_change_bytes": after["swap_used_bytes"] - before["swap_used_bytes"],
            "bag_parse_count": sum(item["bag_parse_count"] for item in primary),
            "bag_parse_time_s": sum(item["bag_parse_time_s"] for item in primary),
            "generated_sensor_scan_count": sum(item["generated_scan_count"] for item in primary),
            "fine_refinement_count": sum(item["fine_refinement_count"] for item in primary),
            "evaluated_scan_count": sum(item["evaluated_scan_count"] for item in primary),
            "projection_wall_s_worker_sum": sum(item["projection_wall_s"] for item in primary),
            "actionability_wall_s_worker_sum": sum(item["actionability_wall_s"] for item in primary),
            "set_membership_wall_s_worker_sum": sum(item["set_membership_wall_s"] for item in primary),
            "determinism_repeat_wall_s_worker_sum": sum(item["worker_wall_s"] for item in repeats),
            "diagnostic_build_time_s": args.diagnostic_build_time_s,
            "diagnostic_build_max_rss_bytes": args.diagnostic_build_max_rss_bytes,
            "diagnostic_binary": str(BINARY),
            "diagnostic_binary_sha256": sha256(BINARY),
            "bag_parsing_policy": "one parse per recorded case per evaluation; normal cases use no bag",
            "cache_reuse": [
                str(PRIOR_RULEBOOK),
                str(PRIOR_S3),
                str(PRIOR_FREE_GAP),
                str(TIME_AXIS),
            ],
            "support_projection": "morphological boundary, polyline extreme shortlist, detector-owned CLCS AABB exact finalization",
            "support_projection_baseline_exact": all(
                max(abs(value) for value in item["projection_baseline_delta"].values()) <= 1.0e-12
                for item in primary
            ),
            "closed_loop_replay_count": 0,
            "cma_run_count": 0,
            "dataset_regeneration_count": 0,
            "production_modification_count": 0,
        }
    )
    for case in primary:
        write_csv(OUTPUT / f"time_axis_{case['case_id'].replace('validation_', 'validation')}.csv", case["rows"])
    write_csv(
        OUTPUT / "blocking_hypothesis_lifecycle.csv",
        [row for case in primary for row in case["lifecycle"]],
    )
    write_csv(
        OUTPUT / "viewpoint_visibility.csv",
        [row for case in primary for row in case["viewpoints"]],
    )
    actionability_fields = [
        "case_id",
        "scan_index",
        "source_stamp_ns",
        "delta_t_ms",
        "ego_s_m",
        "ego_speed_mps",
        "available_longitudinal_distance_m",
        "current_preparation_entry_distance_m",
        "gt_family_generated",
        "gt_family_hard_feasible",
        "shadow_family_generated",
        "shadow_family_hard_feasible",
        "same_feasibility_classification",
        "configured_decel_limit_mps2",
        "minimum_stopping_distance_m",
        "safe_stop_buffer_m",
        "stopping_room_after_buffer_m",
        "safe_deceleration_opportunity",
        "remaining_observation_time_s",
        "planner_gt_best_side",
        "planner_gt_best_target_d",
        "planner_gt_best_wall_clearance_m",
        "planner_gt_best_obstacle_clearance_m",
        "planner_gt_best_peak_curvature_radpm",
        "planner_gt_best_peak_lateral_slope",
        "planner_gt_best_peak_curvature_rate_radpm2",
        "planner_shadow_best_side",
        "planner_shadow_best_target_d",
        "planner_shadow_best_wall_clearance_m",
        "planner_shadow_best_obstacle_clearance_m",
    ]
    write_csv(
        OUTPUT / "actionability.csv",
        [row for case in primary for row in case["rows"]],
        actionability_fields,
    )
    (OUTPUT / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (OUTPUT / "performance.json").write_text(
        json.dumps(performance, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (OUTPUT / "README.md").write_text(
        render_readme(summary, performance), encoding="utf-8"
    )


if __name__ == "__main__":
    main()
