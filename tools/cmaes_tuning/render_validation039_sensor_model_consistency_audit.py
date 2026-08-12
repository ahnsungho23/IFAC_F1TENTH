#!/usr/bin/env python3
"""Offline validation_039 sensor-model consistency root-cause audit.

This is diagnostic code only. It parses the existing lockstep bag once, never publishes, and
never changes detector/planner/controller state. Ground-truth raster geometry labels recorded
returns for analysis; it is never used as an estimator input or to shrink a possible set.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import resource
import subprocess
import time
from typing import Any, Iterable, Sequence

import numpy as np
import yaml

import render_rulebook_free_gap_pruning_audit as free_gap
import render_rulebook_obstacle_envelope_audit as rulebook
import render_s3_shadow_false_infeasible_audit as s3
import render_time_to_disambiguation_audit as time_audit
import render_validation039_representation_root_cause as representation
from cmaes_tuning.bag_reader import read_bag
from cmaes_tuning.scenario_generator import load_waypoints
from cmaes_tuning.simulator_collision import SimulatorRasterCollisionModel


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "runs/cmaes_tuning/validation039_sensor_model_consistency_audit_v1"
RAW = OUTPUT / ".raw"
TIME_AUDIT = ROOT / "runs/cmaes_tuning/time_to_disambiguation_audit_v1"
REPLAY = ROOT / "runs/cmaes_tuning/time_axis_feasibility_audit_v1/replays/validation_039"
SCENARIO = (
    ROOT
    / "runs/cmaes_tuning/medium_lockstep_stage1_v2/artifacts/scenarios/validation/validation_039"
)
BAKED_MAP = (
    ROOT
    / "runs/cmaes_tuning/repeatability_mid4_v2/scenarios/validation/validation_039"
    / "validation_039_map.yaml"
)
WAYPOINTS = ROOT / "offline_trajectory_generator/output/ifac_track/global_waypoints.json"
BASELINE_STAMP = 11_230_000_000
STAMPS = tuple(BASELINE_STAMP + offset * 10_000_000 for offset in range(5))
AUDIT_STAMPS = STAMPS[2:]
GRID_M = 0.004
ANGLE_DEG = 2.0
WORKERS = 4


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
        return {str(key): clean_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean_json(item) for item in value]
    if isinstance(value, np.generic):
        return clean_json(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        raise RuntimeError(f"refusing to write empty CSV: {path}")
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


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


def yaw_from_odom(odom: Any) -> float:
    return time_audit.yaw_from_odom(odom)


@dataclass(frozen=True)
class FrameEvidence:
    stamp_ns: int
    scan: Any
    odom: Any
    event: dict[str, Any]
    yaw_rad: float
    origin: np.ndarray
    directions: np.ndarray
    endpoints: np.ndarray
    valid_indices: np.ndarray
    hit_indices: np.ndarray
    ray_frame: rulebook.RayFrame
    direction_model: str


def direction_array(
    model_name: str,
    scan: Any,
    yaw: float,
    model: dict[str, Any],
    collision: SimulatorRasterCollisionModel,
) -> np.ndarray:
    if model_name == "ROS_CANONICAL_FOV_N":
        angles = (
            float(scan.angle_min)
            + np.arange(len(scan.ranges), dtype=np.float64) * float(scan.angle_increment)
            + yaw
        )
        return np.column_stack((np.cos(angles), np.sin(angles)))
    if model_name == "ANALYTIC_FOV_N_MINUS_1":
        return rulebook.backend_directions_continuous(yaw, model)
    if model_name == "EXACT_BACKEND_LOOKUP":
        return rulebook.backend_directions_model(collision, yaw)
    raise ValueError(model_name)


def build_frame(
    stamp_ns: int,
    model_name: str,
    scans: dict[int, Any],
    odometry: dict[int, Any],
    events: dict[int, dict[str, Any]],
    raster: dict[str, float],
    model: dict[str, Any],
    detector_parameters: dict[str, Any],
    collision: SimulatorRasterCollisionModel,
) -> FrameEvidence:
    scan = scans[stamp_ns]
    odom = odometry[stamp_ns]
    yaw = yaw_from_odom(odom)
    offset = float(model["lidar_offset_x_m"])
    origin = np.asarray(
        [
            float(odom.pose.pose.position.x) + offset * math.cos(yaw),
            float(odom.pose.pose.position.y) + offset * math.sin(yaw),
        ],
        dtype=np.float64,
    )
    directions = direction_array(model_name, scan, yaw, model, collision)
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    # Identical to the established S3 audit. Detector's stricter 14 m gate is reported separately.
    valid = np.isfinite(ranges) & (ranges >= float(scan.range_min))
    endpoints = origin + ranges[:, None] * directions
    hits = valid & rulebook.points_inside_half_open(endpoints, raster)
    hit_indices = np.flatnonzero(hits).astype(np.int64)
    valid_indices = np.flatnonzero(valid).astype(np.int64)
    guard = max(
        rulebook.GEOMETRY_EPSILON_M,
        float(model["scan_noise_std_m"]) * float(model["scan_noise_guard_sigma"]),
        3.0 * float(detector_parameters["cluster_sigma"]),
    )
    ray_frame = rulebook.RayFrame(
        stamp_ns=stamp_ns,
        status="CONFIRMED",
        motion_status="UNKNOWN",
        origin=origin,
        directions=directions[valid_indices],
        ranges=ranges[valid_indices],
        free_lengths=np.maximum(0.0, ranges[valid_indices] - guard),
        hits=endpoints[hit_indices],
        hit_indices=tuple(int(value) for value in hit_indices),
        detector_box=(math.nan, math.nan, math.nan, math.nan),
    )
    return FrameEvidence(
        stamp_ns=stamp_ns,
        scan=scan,
        odom=odom,
        event=events[stamp_ns],
        yaw_rad=yaw,
        origin=origin,
        directions=directions,
        endpoints=endpoints,
        valid_indices=valid_indices,
        hit_indices=hit_indices,
        ray_frame=ray_frame,
        direction_model=model_name,
    )


def clone_ray_frame(
    evidence: FrameEvidence,
    *,
    include_hits: bool = True,
    include_free: bool = True,
    beam_free_length: tuple[int, float] | None = None,
) -> rulebook.RayFrame:
    frame = evidence.ray_frame
    free_lengths = frame.free_lengths.copy() if include_free else np.zeros_like(frame.free_lengths)
    if beam_free_length is not None:
        beam, length = beam_free_length
        rows = np.flatnonzero(evidence.valid_indices == beam)
        if rows.size != 1:
            raise RuntimeError(f"beam {beam} does not map to one valid ray")
        free_lengths[int(rows[0])] = length
    return rulebook.RayFrame(
        stamp_ns=frame.stamp_ns,
        status=frame.status,
        motion_status=frame.motion_status,
        origin=frame.origin,
        directions=frame.directions,
        ranges=frame.ranges,
        free_lengths=free_lengths,
        hits=frame.hits if include_hits else np.empty((0, 2), dtype=np.float64),
        hit_indices=frame.hit_indices if include_hits else (),
        detector_box=frame.detector_box,
    )


def safe_grid(
    frames: Sequence[rulebook.RayFrame],
    resolution_m: float = GRID_M,
    angle_step_deg: float = ANGLE_DEG,
    *,
    use_free_space: bool = True,
) -> tuple[s3.OccupancyGrid | None, str]:
    try:
        return (
            s3.detailed_possible_occupancy(
                frames,
                resolution_m,
                angle_step_deg,
                use_free_space=use_free_space,
            ),
            "NON_EMPTY",
        )
    except RuntimeError as error:
        if "no rulebook hypothesis remains" not in str(error):
            raise
        return None, "EMPTY"


def current_window(
    frames: dict[int, FrameEvidence], now_ns: int
) -> list[rulebook.RayFrame]:
    return [
        frames[stamp].ray_frame
        for stamp in STAMPS
        if now_ns - 20_000_000 <= stamp <= now_ns
    ]


def grid_domain_size(frames: Sequence[rulebook.RayFrame]) -> tuple[int, int]:
    hits = np.vstack([frame.hits for frame in frames])
    padding = math.sqrt(2.0) * rulebook.MAX_SIDE_M + GRID_M
    x_start = math.floor((float(np.min(hits[:, 0])) - padding) / GRID_M) * GRID_M
    x_stop = math.ceil((float(np.max(hits[:, 0])) + padding) / GRID_M) * GRID_M
    y_start = math.floor((float(np.min(hits[:, 1])) - padding) / GRID_M) * GRID_M
    y_stop = math.ceil((float(np.max(hits[:, 1])) + padding) / GRID_M) * GRID_M
    x_count = np.arange(x_start + 0.5 * GRID_M, x_stop, GRID_M).size
    y_count = np.arange(y_start + 0.5 * GRID_M, y_stop, GRID_M).size
    return int(x_count * y_count), int(90.0 / ANGLE_DEG)


def frame_constraint_counts(frames: Sequence[rulebook.RayFrame]) -> dict[str, int]:
    return {
        "hit_constraints": int(sum(frame.hits.shape[0] for frame in frames)),
        "free_ray_constraints": int(sum(frame.free_lengths.size for frame in frames)),
    }


def detector_fragment_audit(
    evidence: FrameEvidence,
    target_beams: set[int],
    parameters: dict[str, Any],
) -> dict[str, Any]:
    scan = evidence.scan
    lambda_rad = math.radians(float(parameters["lambda_deg"]))
    sigma = float(parameters["cluster_sigma"])
    minimum_distance = float(parameters["min_2_points_dist"])
    dphi = float(scan.angle_increment)
    denominator = math.sin(lambda_rad - dphi)
    fragments: list[list[representation.ScanPoint]] = []
    current: list[representation.ScanPoint] = []
    previous: representation.ScanPoint | None = None
    previous_index = -1000

    def flush() -> None:
        nonlocal current
        if current:
            fragments.append(current)
        current = []

    for index, raw_range in enumerate(scan.ranges):
        range_m = float(raw_range)
        if (
            not math.isfinite(range_m)
            or range_m < float(scan.range_min)
            or range_m >= float(parameters["max_range"])
        ):
            continue
        point = representation.ScanPoint(
            index,
            float(evidence.endpoints[index, 0]),
            float(evidence.endpoints[index, 1]),
            range_m,
        )
        same = False
        if previous is not None and index == previous_index + 1:
            maximum_break = 3.0 * sigma
            if denominator > 1.0e-6:
                maximum_break += range_m * math.sin(dphi) / denominator
            same = math.hypot(point.x - previous.x, point.y - previous.y) <= max(
                maximum_break, minimum_distance
            )
        if not same:
            flush()
        current.append(point)
        previous = point
        previous_index = index
    flush()

    target_fragments = [
        fragment
        for fragment in fragments
        if any(point.index in target_beams for point in fragment)
    ]
    kept_for_merge = [
        fragment
        for fragment in target_fragments
        if len(fragment) >= int(parameters["cluster_merge_min_fragment_points"])
    ]
    final_clusters, stats = representation.detector_clusters(
        scan,
        float(evidence.origin[0]),
        float(evidence.origin[1]),
        evidence.yaw_rad,
    )
    final_target = [
        cluster
        for cluster in final_clusters
        if any(point.index in target_beams for point in cluster)
    ]
    fragment_sizes = [len(item) for item in target_fragments]
    if final_target:
        reason = "ACCEPTED"
    elif any(len(item) >= int(parameters["cluster_merge_min_fragment_points"]) for item in target_fragments):
        reason = "MIN_CLUSTER_POINTS_FINAL"
    else:
        reason = "MIN_FRAGMENT_POINTS"
    return {
        "target_fragment_count": len(target_fragments),
        "target_fragment_sizes": ";".join(str(value) for value in fragment_sizes),
        "target_fragments_kept_for_merge": len(kept_for_merge),
        "target_final_cluster_count": len(final_target),
        "target_final_cluster_size": len(final_target[0]) if final_target else 0,
        "target_cluster_rejection_reason": reason,
        "all_clusters_before_merge": stats["clusters_before_merge"],
        "all_clusters_before_final_filter": stats["clusters_before_final_filter"],
        "all_clusters_after_merge": stats["clusters_after_merge"],
        "all_fragments_rejected": stats["fragments_rejected"],
    }


def survivor_cache(
    frames: Sequence[rulebook.RayFrame], grid: s3.OccupancyGrid
) -> tuple[list[dict[str, Any]], np.ndarray]:
    indices = np.flatnonzero(grid.mask.ravel())
    hits = np.vstack([frame.hits for frame in frames])
    centre = np.mean(hits, axis=0)
    cache: list[dict[str, Any]] = []
    for angle_deg in grid.angles_deg:
        accepted, u_min, u_max, v_min, v_max = time_audit.sensor_consistent_witnesses_fast(
            frames, grid, indices, float(angle_deg)
        )
        cache.append(
            {
                "angle_deg": float(angle_deg),
                "indices": accepted,
                "u_min": u_min,
                "u_max": u_max,
                "v_min": v_min,
                "v_max": v_max,
            }
        )
    return cache, centre


def apply_new_constraints(
    cache: Sequence[dict[str, Any]],
    centre: np.ndarray,
    latest: rulebook.RayFrame,
) -> dict[str, int]:
    hit_only = 0
    free_only = 0
    both = 0
    initial = 0
    for item in cache:
        count = int(item["indices"].size)
        if count == 0:
            continue
        initial += count
        angle = math.radians(float(item["angle_deg"]))
        cosine, sine = math.cos(angle), math.sin(angle)
        hit_delta = latest.hits - centre
        hit_u = hit_delta[:, 0] * cosine + hit_delta[:, 1] * sine
        hit_v = -hit_delta[:, 0] * sine + hit_delta[:, 1] * cosine
        hit_ok = np.ones(count, dtype=bool)
        for u, v in zip(hit_u, hit_v):
            hit_ok &= (
                (u >= item["u_min"] - rulebook.GEOMETRY_EPSILON_M)
                & (u <= item["u_max"] + rulebook.GEOMETRY_EPSILON_M)
                & (v >= item["v_min"] - rulebook.GEOMETRY_EPSILON_M)
                & (v <= item["v_max"] + rulebook.GEOMETRY_EPSILON_M)
            )

        origin_delta = latest.origin - centre
        origin_u = origin_delta[0] * cosine + origin_delta[1] * sine
        origin_v = -origin_delta[0] * sine + origin_delta[1] * cosine
        direction_u = latest.directions[:, 0] * cosine + latest.directions[:, 1] * sine
        direction_v = -latest.directions[:, 0] * sine + latest.directions[:, 1] * cosine
        free_ok = np.ones(count, dtype=bool)
        domain = (
            float(np.min(item["u_min"])),
            float(np.max(item["u_max"])),
            float(np.min(item["v_min"])),
            float(np.max(item["v_max"])),
        )
        relevant = [
            ray
            for ray in range(latest.free_lengths.size)
            if rulebook.segment_intersects_box(
                float(origin_u),
                float(origin_v),
                float(direction_u[ray]),
                float(direction_v[ray]),
                float(latest.free_lengths[ray]),
                *domain,
            )
        ]
        for ray in relevant:
            live = np.flatnonzero(free_ok)
            if live.size == 0:
                break
            invalid = rulebook.ray_invalid_candidates(
                float(origin_u),
                float(origin_v),
                float(direction_u[ray]),
                float(direction_v[ray]),
                float(latest.free_lengths[ray]),
                item["u_min"][live],
                item["u_max"][live],
                item["v_min"][live],
                item["v_max"][live],
            )
            free_ok[live[invalid]] = False
        hit_only += int(np.count_nonzero(hit_ok))
        free_only += int(np.count_nonzero(free_ok))
        both += int(np.count_nonzero(hit_ok & free_ok))
    return {
        "cached_survivors_at_plus30": initial,
        "new_plus40_hits_only_survivors": hit_only,
        "new_plus40_free_rays_only_survivors": free_only,
        "new_plus40_hits_and_free_survivors": both,
    }


def tight_box_state(
    hits: list[tuple[int, int, np.ndarray, float]],
    rays: list[tuple[int, int, np.ndarray, np.ndarray, float, float]],
    hit_indices: Sequence[int],
    ray_indices: Sequence[int],
    angle_step_deg: float,
) -> tuple[list[float], dict[float, list[int]]]:
    points = np.asarray([hits[index][2] for index in hit_indices])
    alive: list[float] = []
    failures: dict[float, list[int]] = {}
    effective = rulebook.MAX_SIDE_M + math.sqrt(2.0) * GRID_M
    for angle_deg in np.arange(0.0, 90.0 - 1.0e-12, angle_step_deg):
        angle = math.radians(float(angle_deg))
        cosine, sine = math.cos(angle), math.sin(angle)
        u = points[:, 0] * cosine + points[:, 1] * sine
        v = -points[:, 0] * sine + points[:, 1] * cosine
        u_min, u_max = float(np.min(u)), float(np.max(u))
        v_min, v_max = float(np.min(v)), float(np.max(v))
        if u_max - u_min > effective or v_max - v_min > effective:
            failures[float(angle_deg)] = []
            continue
        rejected: list[int] = []
        for ray_index in ray_indices:
            _, _, origin, direction, free_length, _ = rays[ray_index]
            origin_u = origin[0] * cosine + origin[1] * sine
            origin_v = -origin[0] * sine + origin[1] * cosine
            direction_u = direction[0] * cosine + direction[1] * sine
            direction_v = -direction[0] * sine + direction[1] * cosine
            if rulebook.ray_invalid_candidates(
                float(origin_u),
                float(origin_v),
                float(direction_u),
                float(direction_v),
                float(free_length),
                np.asarray([u_min]),
                np.asarray([u_max]),
                np.asarray([v_min]),
                np.asarray([v_max]),
            )[0]:
                rejected.append(ray_index)
        if rejected:
            failures[float(angle_deg)] = rejected
        else:
            alive.append(float(angle_deg))
    return alive, failures


def find_minimal_core(
    old: FrameEvidence,
    latest: FrameEvidence,
) -> dict[str, Any]:
    hits = [
        (
            latest.stamp_ns,
            int(beam),
            latest.endpoints[int(beam)],
            float(latest.scan.ranges[int(beam)]),
        )
        for beam in latest.hit_indices
    ]
    rows_by_beam = {int(beam): row for row, beam in enumerate(old.valid_indices)}
    rays = [
        (
            old.stamp_ns,
            int(beam),
            old.origin,
            old.directions[int(beam)],
            float(old.ray_frame.free_lengths[rows_by_beam[int(beam)]]),
            float(old.scan.ranges[int(beam)]),
        )
        for beam in old.valid_indices
    ]
    all_hits = list(range(len(hits)))
    all_rays = list(range(len(rays)))
    _, failures = tight_box_state(hits, rays, all_hits, all_rays, ANGLE_DEG)
    all_angles = set(np.arange(0.0, 90.0 - 1.0e-12, ANGLE_DEG))
    coverage = {
        ray: {
            angle
            for angle, rejected in failures.items()
            if ray in rejected
        }
        for ray in all_rays
    }
    single_cover = [ray for ray in all_rays if coverage[ray] >= all_angles]
    if not single_cover:
        raise RuntimeError("expected an interpretable single-free-ray UNSAT cover")
    chosen_rays = [single_cover[0]]
    core_hits = all_hits.copy()
    changed = True
    while changed:
        changed = False
        for hit in core_hits.copy():
            trial = [value for value in core_hits if value != hit]
            if len(trial) < 2:
                continue
            alive, _ = tight_box_state(hits, rays, trial, chosen_rays, ANGLE_DEG)
            if not alive:
                core_hits = trial
                changed = True

    if len(core_hits) != 2 or len(chosen_rays) != 1:
        raise RuntimeError("unexpected minimal core shape")
    first, second = (hits[index] for index in core_hits)
    ray = rays[chosen_rays[0]]
    origin, direction = ray[2], ray[3]
    p, q = first[2], second[2]

    def cross(a: np.ndarray, b: np.ndarray) -> float:
        return float(a[0] * b[1] - a[1] * b[0])

    denominator = cross(direction, q - p)
    intersection_distance = cross(p - origin, q - p) / denominator
    segment_fraction = cross(p - origin, direction) / denominator
    intersection = origin + intersection_distance * direction
    ray_angle = math.atan2(float(direction[1]), float(direction[0]))
    endpoint_angles = [math.atan2(*(point - origin)[::-1]) for point in (p, q)]
    minimum_angle = min(
        abs(math.atan2(math.sin(value - ray_angle), math.cos(value - ray_angle)))
        for value in endpoint_angles
    )
    signed_distances = [cross(direction, point - origin) for point in (p, q)]
    deletion_checks = []
    for hit in core_hits:
        alive, _ = tight_box_state(
            hits,
            rays,
            [value for value in core_hits if value != hit],
            chosen_rays,
            ANGLE_DEG,
        )
        deletion_checks.append(bool(alive))
    alive_without_ray, _ = tight_box_state(hits, rays, core_hits, [], ANGLE_DEG)
    deletion_checks.append(bool(alive_without_ray))
    return {
        "hits": hits,
        "rays": rays,
        "core_hit_indices": core_hits,
        "core_ray_indices": chosen_rays,
        "intersection_distance_m": intersection_distance,
        "intersection_x_m": float(intersection[0]),
        "intersection_y_m": float(intersection[1]),
        "segment_fraction": segment_fraction,
        "free_length_m": ray[4],
        "minimum_ray_shortening_m": ray[4] - intersection_distance,
        "minimum_angular_shift_deg": math.degrees(minimum_angle),
        "minimum_transverse_shift_m": min(abs(value) for value in signed_distances),
        "signed_endpoint_distances_m": signed_distances,
        "deletion_minimal": all(deletion_checks),
        "digest": canonical_digest(
            {
                "hits": [(hits[index][0], hits[index][1]) for index in core_hits],
                "ray": [(rays[index][0], rays[index][1]) for index in chosen_rays],
                "intersection": [intersection_distance, segment_fraction],
            }
        ),
    }


def local_orientation_refinement(
    core: dict[str, Any],
    angle_step_deg: float,
) -> dict[str, Any]:
    alive, failures = tight_box_state(
        core["hits"],
        core["rays"],
        core["core_hit_indices"],
        core["core_ray_indices"],
        angle_step_deg,
    )
    return {
        "model": "CURRENT_ANALYTIC_CORE",
        "position_resolution_m": None,
        "orientation_step_deg": angle_step_deg,
        "orientation_count": int(round(90.0 / angle_step_deg)),
        "survivor_orientation_count": len(alive),
        "status": "NON_EMPTY" if alive else "EMPTY",
        "method": "local continuous tight-box core; no global position-grid sweep",
        "failure_digest": canonical_digest(
            {str(angle): rejected for angle, rejected in failures.items()}
        ),
    }


def full_constraint_orientation_refinement(
    frames: Sequence[rulebook.RayFrame], angle_step_deg: float
) -> dict[str, Any]:
    """Continuous-position tight-box test for the one +40 ms contradiction region.

    The minimum rectangle containing all hits is the least likely to intersect a free ray. If that
    tight box is rejected, moving or expanding it cannot restore consistency. This therefore
    avoids a global fine position-grid sweep while giving a stronger continuous-position result.
    """
    hits: list[tuple[int, int, np.ndarray, float]] = []
    rays: list[tuple[int, int, np.ndarray, np.ndarray, float, float]] = []
    for frame in frames:
        for local, point in enumerate(frame.hits):
            beam = frame.hit_indices[local] if local < len(frame.hit_indices) else local
            hits.append((frame.stamp_ns, int(beam), point, math.nan))
        for ray in range(frame.free_lengths.size):
            rays.append(
                (
                    frame.stamp_ns,
                    ray,
                    frame.origin,
                    frame.directions[ray],
                    float(frame.free_lengths[ray]),
                    float(frame.ranges[ray]),
                )
            )
    alive, failures = tight_box_state(
        hits,
        rays,
        list(range(len(hits))),
        list(range(len(rays))),
        angle_step_deg,
    )
    return {
        "model": "CURRENT_FULL_CONSTRAINT_TIGHT_BOX",
        "position_resolution_m": 0.0,
        "orientation_step_deg": angle_step_deg,
        "orientation_count": int(round(90.0 / angle_step_deg)),
        "survivor_orientation_count": len(alive),
        "status": "NON_EMPTY" if alive else "EMPTY",
        "method": "local +40 ms continuous-position tight-box proof; no global fine position grid",
        "failure_digest": canonical_digest(
            {str(angle): rejected for angle, rejected in failures.items()}
        ),
    }


def model_frames(
    frames: dict[str, dict[int, FrameEvidence]], model_name: str, stamps: Iterable[int]
) -> list[rulebook.RayFrame]:
    return [frames[model_name][stamp].ray_frame for stamp in stamps]


def projected_safety(
    name: str,
    grid: s3.OccupancyGrid,
    case: dict[str, Any],
    track: free_gap.TrackGeometry,
    gt: dict[str, float],
    scan_index: int,
    repeat: bool = False,
) -> dict[str, Any]:
    support, elapsed = time_audit.project_grid(
        case, grid, scan_index, track, repeat=repeat
    )
    undercoverage = max(0.0, float(support["d_min"]) - float(gt["d_min"]))
    overcoverage = max(0.0, float(gt["d_min"]) - float(support["d_min"]))
    return {
        "candidate_correction": name,
        "possible_count": int(np.count_nonzero(grid.mask)),
        "possible_area_m2": grid.possible_area_m2,
        "selected_side": "right",
        "selected_side_support_m": support["d_min"],
        "gt_undercoverage_m": undercoverage,
        "gt_overcoverage_m": overcoverage,
        "dangerous_path_clearance_m": support["same_path_clearance_m"],
        "dangerous_path_hard_valid": support["same_path_hard_valid"],
        "dangerous_path_hard_invalid": support["same_path_hard_valid"] is False,
        "dangerous_path_rejection": support["same_path_rejection"],
        "projection_wall_s": elapsed,
    }


def table(headers: list[str], rows: list[list[Any]]) -> str:
    def token(value: Any) -> str:
        if value is None:
            return "N/A"
        if isinstance(value, float):
            return f"{value:.9f}"
        return str(value)

    output = ["| " + " | ".join(headers) + " |", "|" + "|".join(["---"] * len(headers)) + "|"]
    output.extend("| " + " | ".join(token(value) for value in row) + " |" for row in rows)
    return "\n".join(output)


def render_readme(summary: dict[str, Any], performance: dict[str, Any]) -> str:
    time_rows = summary["time_axis"]
    ablations = summary["model_ablation"]
    core = summary["minimal_unsat_core"]
    corrections = summary["candidate_corrections_safety"]
    detector = {row["source_stamp_ns"]: row for row in summary["raw_vs_detector_evidence"]}
    table1 = table(
        ["timestamp", "raw obstacle returns", "detector cluster", "possible count", "status"],
        [
            [
                row["source_stamp_ns"],
                detector[row["source_stamp_ns"]]["raw_obstacle_labeled_endpoint_count"],
                detector[row["source_stamp_ns"]]["target_final_cluster_count"],
                row["possible_hypothesis_count"],
                row["final_status"],
            ]
            for row in time_rows
        ],
    )
    table2 = table(
        ["ablation", "possible count", "empty?", "039 dangerous path safe?"],
        [
            [
                row["ablation"],
                row.get("possible_cell_count"),
                row["empty"],
                row.get("dangerous_path_safe", "NOT_EVALUATED"),
            ]
            for row in ablations
        ],
    )
    table3 = table(
        ["constraint/core member", "scan", "beam", "type", "conflicting member", "metric inconsistency"],
        [
            [
                row["core_member"],
                row["source_stamp_ns"],
                row["beam_index"],
                row["constraint_type"],
                row["conflicting_member"],
                row["metric_inconsistency"],
            ]
            for row in core["rows"]
        ],
    )
    table4 = table(
        ["candidate correction", "root-cause class", "non-empty restored?", "GT undercoverage", "dangerous path hard-invalid?"],
        [
            [
                row["candidate_correction"],
                row["root_cause_class"],
                row["non_empty_restored"],
                row.get("gt_undercoverage_m"),
                row.get("dangerous_path_hard_invalid"),
            ]
            for row in corrections
        ],
    )
    return f"""# validation_039 sensor-model consistency root-cause audit v1

Diagnostic/offline only. Production files, YAML, planner behavior, detector behavior, controller,
state machine, and tracking were not changed. The recorded bag was parsed once; no replay, CMA,
or dataset regeneration was performed.

Root cause: **{summary['root_cause_class']}**.

The +40 ms empty set is caused by a direction-reconstruction mismatch. Under the previous
continuous `FOV/(N-1)` audit geometry, the +20 ms free ray at beam 829 crosses the segment that
must be occupied by +40 ms hit beams 824 and 826. The exact simulator 2000-bin lookup/truncation
places beam 829 outside that segment and restores a non-empty set. ROS canonical `FOV/N` and
continuous `FOV/(N-1)` are both insufficient reproductions of the range-producing backend.

## Table 1 — causal transition

{table1}

## Table 2 — compact ablation

{table2}

## Table 3 — near-minimal UNSAT core

{table3}

The core is deletion-minimal for the current 2 degree model: removing either hit or the free ray
restores at least one sampled orientation. The compact core alone admits one interleaved finer
orientation, but the full +40 ms constraint set remains empty at 1 degree and 0.5 degree under a
continuous-position tight-box check. Thus the full empty set is not a coarse-grid artifact.

## Table 4 — candidate corrections and safety

{table4}

Only `EXACT_BACKEND_LOOKUP` is a justified correction in this simulator audit. It preserves zero
GT undercoverage and keeps the known path hard-invalid. Dropping hits/history is shown only as an
ablation and is not a proposed fix.

## Raw sensor versus production detector

Raw ranges are internally consistent when interpreted with the direction lookup that generated
them. The production detector separately loses the future obstacle evidence: after +0 ms the
target contributes only 2–4 contiguous/fragmented points, below `min_cluster_points=5`, so no raw
detection or track measurement is produced. The confirmed track continues prediction-only.

This detector information loss is not the cause of the offline S3 empty set, but it is a separate
future static-safety concern. Any follow-up must remain detector/static-safety-side and must not
change dynamic association, Frenet KF, map-velocity KF, `vs/vd`, physical IDs, Mahalanobis gating,
motion voting, or `/opp_obs`.

## Numerical findings

- Minimum core-only angular change: {core['minimum_angular_shift_deg']:.9f} deg.
- Minimum transverse endpoint/pose shift: {1000.0 * core['minimum_transverse_shift_m']:.6f} mm.
- Equivalent free-ray shortening: {core['minimum_ray_shortening_m']:.6f} m; this rules out a small
  range-precision or endpoint-inclusivity explanation.
- Current free-ray guard is {core['free_guard_m']:.6f} m. The free interval does not include its
  own endpoint cell; the contradiction is an old ray crossing later occupied evidence.

## Compute and provenance

- Bag parses: {performance['bag_parse_count']}; workers: {performance['worker_count_configured']}
  maximum, causal timestamps serial.
- Wall / CPU: {performance['wall_time_s']:.3f} / {performance['cpu_time_s']:.3f} s.
- Max RSS: {performance['max_rss_bytes']} bytes; swap delta: {performance['swap_change_bytes']} bytes.
- Full-grid computations: {performance['full_grid_computation_count']}; local fine refinements:
  {performance['local_fine_refinement_count']}.
- Closed-loop replay / production modifications / CMA / dataset regeneration: 0 / 0 / 0 / 0.

See the CSV files for per-beam, per-constraint, pose, extrinsic, detector, and safety evidence.
"""


def main() -> None:
    args = parse_args()
    wall_started = time.monotonic()
    self_started = resource.getrusage(resource.RUSAGE_SELF)
    child_started = resource.getrusage(resource.RUSAGE_CHILDREN)
    memory_before = memory_snapshot()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    RAW.mkdir(parents=True, exist_ok=True)
    time_audit.RAW = RAW
    production_before = {
        name: sha256(path) for name, path in rulebook.PRODUCTION_FILES.items()
    }

    manifest_path, manifest = rulebook.local_manifest("validation_039")
    raster = {
        key: float(value)
        for key, value in manifest["baked_obstacle_raster"]["world_half_open_bounds_m"].items()
    }
    model = manifest["simulator_collision_model"]
    detector_parameters = yaml.safe_load(
        (ROOT / "src/obstacle_detector/config/obstacle_detector.yaml").read_text(
            encoding="utf-8"
        )
    )["obstacle_detector"]["ros__parameters"]
    events_list = rulebook.read_jsonl(REPLAY / "detector_events.jsonl")
    events = {int(item["scan_stamp_ns"]): item for item in events_list}
    bag_parse_started = time.monotonic()
    bag = read_bag(REPLAY / "bag", topics=("/scan", "/ego_racecar/odom"))
    bag_parse_time = time.monotonic() - bag_parse_started
    scans = {rulebook.stamp_ns(item.message): item.message for item in bag.topic("/scan")}
    odometry = {
        rulebook.stamp_ns(item.message): item.message
        for item in bag.topic("/ego_racecar/odom")
    }
    collision = SimulatorRasterCollisionModel(BAKED_MAP, model)
    model_names = (
        "ROS_CANONICAL_FOV_N",
        "ANALYTIC_FOV_N_MINUS_1",
        "EXACT_BACKEND_LOOKUP",
    )
    evidence = {
        name: {
            stamp: build_frame(
                stamp,
                name,
                scans,
                odometry,
                events,
                raster,
                model,
                detector_parameters,
                collision,
            )
            for stamp in STAMPS
        }
        for name in model_names
    }
    current = evidence["ANALYTIC_FOV_N_MINUS_1"]

    time_axis: list[dict[str, Any]] = []
    grids_current: dict[int, s3.OccupancyGrid | None] = {}
    full_grid_count = 0
    for stamp in AUDIT_STAMPS:
        window = current_window(current, stamp)
        grid, status = safe_grid(window)
        full_grid_count += 1
        grids_current[stamp] = grid
        counts = frame_constraint_counts(window)
        domain_cells, orientations = grid_domain_size(window)
        if grid is not None:
            witnesses = time_audit.witness_statistics(window, grid)
            possible_hypotheses = witnesses["possible_hypothesis_count"]
            surviving_orientations = witnesses["surviving_orientation_count"]
            possible_cells = int(np.count_nonzero(grid.mask))
            possible_area = grid.possible_area_m2
            query_count = grid.query_count
        else:
            possible_hypotheses = 0
            surviving_orientations = 0
            possible_cells = 0
            possible_area = 0.0
            query_count = 0
        frame = current[stamp]
        scan_ranges = np.asarray(frame.scan.ranges, dtype=np.float64)
        valid_sensor = np.isfinite(scan_ranges) & (scan_ranges >= float(frame.scan.range_min))
        valid_detector = valid_sensor & (scan_ranges < float(detector_parameters["max_range"]))
        time_axis.append(
            {
                "source_stamp_ns": stamp,
                "delta_t_ms": (stamp - BASELINE_STAMP) // 1_000_000,
                "ego_x_m": float(frame.odom.pose.pose.position.x),
                "ego_y_m": float(frame.odom.pose.pose.position.y),
                "ego_yaw_rad": frame.yaw_rad,
                "lidar_x_m": float(frame.origin[0]),
                "lidar_y_m": float(frame.origin[1]),
                "lidar_yaw_rad": frame.yaw_rad,
                "valid_sensor_range_count": int(np.count_nonzero(valid_sensor)),
                "valid_detector_range_count": int(np.count_nonzero(valid_detector)),
                "raw_obstacle_labeled_endpoint_count": int(frame.hit_indices.size),
                "production_detector_raw_cluster_count": len(frame.event["raw_detections"]),
                "production_track_state": frame.event["tracks"][0]["track_status"]
                if frame.event["tracks"]
                else "NONE",
                "production_track_visible": frame.event["tracks"][0]["visible"]
                if frame.event["tracks"]
                else False,
                "ut20_source_frames": ";".join(str(item.stamp_ns) for item in window),
                **counts,
                "orientation_hypotheses_evaluated": orientations,
                "dimension_hypotheses_evaluated": domain_cells * orientations,
                "dimension_valid_query_count": query_count,
                "possible_cell_count": possible_cells,
                "possible_hypothesis_count": possible_hypotheses,
                "possible_area_m2": possible_area,
                "final_status": status,
            }
        )
    if time_axis[-2]["final_status"] != "NON_EMPTY" or time_axis[-1]["final_status"] != "EMPTY":
        raise RuntimeError("failed to reproduce +30 non-empty -> +40 empty transition")

    grid30 = grids_current[STAMPS[3]]
    assert grid30 is not None
    cache30, centre30 = survivor_cache(current_window(current, STAMPS[3]), grid30)
    cache_metrics = apply_new_constraints(cache30, centre30, current[STAMPS[4]].ray_frame)

    window40 = current_window(current, STAMPS[4])
    domain_cells, orientations = grid_domain_size(window40)
    hits_only40, _ = safe_grid(window40, use_free_space=False)
    full_grid_count += 1
    assert hits_only40 is not None
    size_valid_count = hits_only40.query_count
    rejection_counts = [
        {
            "source_stamp_ns": STAMPS[4],
            "reason": "HIT_CONTAINMENT_FAIL",
            "rejected_hypothesis_count": 0,
            "count_semantics": "generated rectangles contain all hit constraints by construction",
        },
        {
            "source_stamp_ns": STAMPS[4],
            "reason": "DIMENSION_LIMIT_FAIL",
            "rejected_hypothesis_count": domain_cells * orientations - size_valid_count,
            "count_semantics": "angle/cell candidates exceeding <=0.5 m plus coarse outer-cover allowance",
        },
        {
            "source_stamp_ns": STAMPS[4],
            "reason": "FREE_RAY_INTERSECTION_FAIL",
            "rejected_hypothesis_count": size_valid_count,
            "count_semantics": "every dimension-valid current-model candidate is removed by a free ray",
        },
        {
            "source_stamp_ns": STAMPS[4],
            "reason": "TEMPORAL_CROSS_SCAN_CONFLICT",
            "rejected_hypothesis_count": cache_metrics["cached_survivors_at_plus30"]
            - cache_metrics["new_plus40_hits_and_free_survivors"],
            "count_semantics": "cached +30 survivors not satisfying both new +40 constraint groups",
        },
    ]
    for reason in (
        "ORIENTATION_DISCRETIZATION_FAIL",
        "POSE_OR_FRAME_CONSISTENCY_FAIL",
        "RASTER_MODEL_CONFLICT",
        "OTHER",
    ):
        rejection_counts.append(
            {
                "source_stamp_ns": STAMPS[4],
                "reason": reason,
                "rejected_hypothesis_count": 0,
                "count_semantics": "root-cause classification after targeted ablation",
            }
        )
    for label, value in cache_metrics.items():
        rejection_counts.append(
            {
                "source_stamp_ns": STAMPS[4],
                "reason": label.upper(),
                "rejected_hypothesis_count": value,
                "count_semantics": "fixed +30 survivor-set one-group-at-a-time application",
            }
        )

    core = find_minimal_core(current[STAMPS[2]], current[STAMPS[4]])
    core_hit_values = [core["hits"][index] for index in core["core_hit_indices"]]
    core_ray_value = core["rays"][core["core_ray_indices"][0]]
    core_rows: list[dict[str, Any]] = []
    for number, hit in enumerate(core_hit_values, 1):
        core_rows.append(
            {
                "core_member": f"H{number}",
                "source_stamp_ns": hit[0],
                "beam_index": hit[1],
                "range_m": hit[3],
                "endpoint_x_m": float(hit[2][0]),
                "endpoint_y_m": float(hit[2][1]),
                "ray_origin_x_m": float(current[STAMPS[4]].origin[0]),
                "ray_origin_y_m": float(current[STAMPS[4]].origin[1]),
                "constraint_type": "HIT_CONTAINMENT",
                "obstacle_local_interpretation": "later return inside exact simulator raster",
                "conflicting_member": "F1",
                "metric_inconsistency": "opposite sides of old free-ray line; occupied segment is crossed",
            }
        )
    core_rows.append(
        {
            "core_member": "F1",
            "source_stamp_ns": core_ray_value[0],
            "beam_index": core_ray_value[1],
            "range_m": core_ray_value[5],
            "endpoint_x_m": float(
                core_ray_value[2][0] + core_ray_value[5] * core_ray_value[3][0]
            ),
            "endpoint_y_m": float(
                core_ray_value[2][1] + core_ray_value[5] * core_ray_value[3][1]
            ),
            "ray_origin_x_m": float(core_ray_value[2][0]),
            "ray_origin_y_m": float(core_ray_value[2][1]),
            "constraint_type": "FREE_RAY",
            "obstacle_local_interpretation": "older no-return ray crosses later occupied segment",
            "conflicting_member": "H1+H2",
            "metric_inconsistency": f"free interval extends {core['minimum_ray_shortening_m']:.9f} m beyond intersection",
        }
    )

    fine_rows = [
        full_constraint_orientation_refinement(window40, 1.0),
        full_constraint_orientation_refinement(window40, 0.5),
        local_orientation_refinement(core, 1.0),
    ]

    detector_rows: list[dict[str, Any]] = []
    for stamp in STAMPS:
        frame = evidence["ROS_CANONICAL_FOV_N"][stamp]
        truth_frame = evidence["EXACT_BACKEND_LOOKUP"][stamp]
        fragments = detector_fragment_audit(
            frame, set(int(value) for value in truth_frame.hit_indices), detector_parameters
        )
        event = frame.event
        detector_rows.append(
            {
                "source_stamp_ns": stamp,
                "delta_t_ms": (stamp - BASELINE_STAMP) // 1_000_000,
                "raw_obstacle_labeled_endpoint_count": int(truth_frame.hit_indices.size),
                "raw_obstacle_beam_indices": ";".join(
                    str(int(value)) for value in truth_frame.hit_indices
                ),
                **fragments,
                "event_raw_detection_count": len(event["raw_detections"]),
                "track_measurement_produced": int(event["tracker_update"]["matched"]) > 0,
                "tracker_matched_count": event["tracker_update"]["matched"],
                "track_status": event["tracks"][0]["track_status"]
                if event["tracks"]
                else "NONE",
                "motion_status": event["tracks"][0]["motion_status"]
                if event["tracks"]
                else "NONE",
                "track_visible": event["tracks"][0]["visible"] if event["tracks"] else False,
                "exact_gate": "fragment>=2 for merge candidate; final merged cluster>=5; max diagonal<=0.8m",
            }
        )

    convention_rows: list[dict[str, Any]] = []
    convention_grids: dict[str, s3.OccupancyGrid | None] = {}
    for model_name in model_names:
        frames40 = model_frames(evidence, model_name, STAMPS[2:5])
        grid, status = safe_grid(frames40)
        full_grid_count += 1
        convention_grids[model_name] = grid
        witnesses = (
            time_audit.witness_statistics(frames40, grid) if grid is not None else None
        )
        convention_rows.append(
            {
                "record_type": "MODEL_SUMMARY",
                "direction_model": model_name,
                "source_stamp_ns": STAMPS[4],
                "beam_index": None,
                "formula": {
                    "ROS_CANONICAL_FOV_N": "angle_min+i*angle_increment; increment=FOV/N",
                    "ANALYTIC_FOV_N_MINUS_1": "linspace(-FOV/2,+FOV/2,N)",
                    "EXACT_BACKEND_LOOKUP": "2000-bin table; integer truncation; backend indexing",
                }[model_name],
                "possible_cell_count": int(np.count_nonzero(grid.mask)) if grid else 0,
                "possible_hypothesis_count": witnesses["possible_hypothesis_count"]
                if witnesses
                else 0,
                "possible_area_m2": grid.possible_area_m2 if grid else 0.0,
                "status": status,
            }
        )
    for stamp, beam in (
        (core_ray_value[0], core_ray_value[1]),
        *((hit[0], hit[1]) for hit in core_hit_values),
    ):
        canonical = evidence["ROS_CANONICAL_FOV_N"][stamp]
        for model_name in model_names[1:]:
            other = evidence[model_name][stamp]
            canonical_angle = math.atan2(
                canonical.directions[beam, 1], canonical.directions[beam, 0]
            )
            other_angle = math.atan2(other.directions[beam, 1], other.directions[beam, 0])
            delta_angle = math.atan2(
                math.sin(other_angle - canonical_angle),
                math.cos(other_angle - canonical_angle),
            )
            convention_rows.append(
                {
                    "record_type": "CORE_BEAM",
                    "direction_model": model_name,
                    "source_stamp_ns": stamp,
                    "beam_index": beam,
                    "formula": "delta relative to recorded ROS canonical metadata",
                    "angle_delta_deg": math.degrees(delta_angle),
                    "endpoint_displacement_m": float(
                        np.linalg.norm(other.endpoints[beam] - canonical.endpoints[beam])
                    ),
                    "canonical_endpoint_x_m": float(canonical.endpoints[beam, 0]),
                    "canonical_endpoint_y_m": float(canonical.endpoints[beam, 1]),
                    "model_endpoint_x_m": float(other.endpoints[beam, 0]),
                    "model_endpoint_y_m": float(other.endpoints[beam, 1]),
                }
            )

    pose_rows: list[dict[str, Any]] = []
    for stamp in STAMPS:
        frame = current[stamp]
        odom_stamp = rulebook.stamp_ns(frame.odom)
        pose_rows.append(
            {
                "scan_stamp_ns": stamp,
                "pose_stamp_ns": odom_stamp,
                "pose_age_ms": (stamp - odom_stamp) / 1.0e6,
                "interpolation_offset_ms": 0.0,
                "pose_method": "EXACT_SAME_STAMP_LOCKSTEP_ODOMETRY",
                "future_pose_used": False,
                "base_x_m": float(frame.odom.pose.pose.position.x),
                "base_y_m": float(frame.odom.pose.pose.position.y),
                "base_yaw_rad": frame.yaw_rad,
                "lidar_x_m": float(frame.origin[0]),
                "lidar_y_m": float(frame.origin[1]),
                "lidar_yaw_rad": frame.yaw_rad,
                "exact_interpolation_delta_x_m": 0.0,
                "exact_interpolation_delta_y_m": 0.0,
                "exact_interpolation_delta_yaw_rad": 0.0,
            }
        )

    offset = float(model["lidar_offset_x_m"])
    extrinsic_rows = [
        {
            "source": source,
            "source_file": file,
            "dx_m": value,
            "dy_m": 0.0,
            "dyaw_rad": 0.0,
            "difference_from_simulator_dx_m": value - offset,
            "frame_contract": "base_link center -> ego_racecar/laser; lockstep direct odom transform",
        }
        for source, file, value in (
            (
                "production_source_default",
                "src/obstacle_detector/src/obstacle_detector_node.cpp",
                0.275,
            ),
            (
                "production_yaml",
                "src/obstacle_detector/config/obstacle_detector.yaml",
                float(detector_parameters["lockstep_scan_offset_x_m"]),
            ),
            (
                "simulator_manifest",
                str(manifest_path),
                offset,
            ),
            (
                "audit_reconstruction",
                "render_validation039_sensor_model_consistency_audit.py",
                offset,
            ),
        )
    ]

    map_metadata = yaml.safe_load(BAKED_MAP.read_text(encoding="utf-8"))
    map_resolution = float(map_metadata["resolution"])
    map_origin_x, map_origin_y = (float(value) for value in map_metadata["origin"][:2])
    contradiction_beams: list[dict[str, Any]] = []
    backend_latest = evidence["EXACT_BACKEND_LOOKUP"][STAMPS[4]]
    nominal = manifest["obstacle"]
    analytic = {
        "x_min": float(nominal["x"]) - 0.5 * float(nominal["width"]),
        "x_max": float(nominal["x"]) + 0.5 * float(nominal["width"]),
        "y_min": float(nominal["y"]) - 0.5 * float(nominal["height"]),
        "y_max": float(nominal["y"]) + 0.5 * float(nominal["height"]),
    }
    for beam in backend_latest.hit_indices:
        point = backend_latest.endpoints[int(beam)]
        boundary_distance = min(
            point[0] - raster["x_min"],
            raster["x_max"] - point[0],
            point[1] - raster["y_min"],
            raster["y_max"] - point[1],
        )
        analytic_inside = (
            analytic["x_min"] <= point[0] <= analytic["x_max"]
            and analytic["y_min"] <= point[1] <= analytic["y_max"]
        )
        eroded_inside = (
            raster["x_min"] + map_resolution <= point[0] < raster["x_max"] - map_resolution
            and raster["y_min"] + map_resolution <= point[1] < raster["y_max"] - map_resolution
        )
        contradiction_beams.append(
            {
                "source_stamp_ns": STAMPS[4],
                "beam_index": int(beam),
                "range_m": float(backend_latest.scan.ranges[int(beam)]),
                "endpoint_x_m": float(point[0]),
                "endpoint_y_m": float(point[1]),
                "nearest_raster_boundary_distance_m": float(boundary_distance),
                "raster_cell_column": math.floor((point[0] - map_origin_x) / map_resolution),
                "raster_cell_row_after_vertical_flip": math.floor(
                    (point[1] - map_origin_y) / map_resolution
                ),
                "inside_half_open_raster": True,
                "inside_nominal_analytic_obstacle": analytic_inside,
                "inside_after_one_cell_erosion": eroded_inside,
                "label_interpretation": "edge-sensitive exact simulator-raster return; not estimator evidence",
            }
        )

    core_ray_endpoint = core_ray_value[2] + core_ray_value[5] * core_ray_value[3]
    intersection_cell = (
        math.floor(core["intersection_x_m"] / GRID_M),
        math.floor(core["intersection_y_m"] / GRID_M),
    )
    ray_endpoint_cell = (
        math.floor(float(core_ray_endpoint[0]) / GRID_M),
        math.floor(float(core_ray_endpoint[1]) / GRID_M),
    )
    free_guard = core_ray_value[5] - core_ray_value[4]
    ray_semantics_rows = [
        {
            "constraint": "F1_OLD_FREE_RAY",
            "source_stamp_ns": core_ray_value[0],
            "beam_index": core_ray_value[1],
            "sensor_origin_x_m": float(core_ray_value[2][0]),
            "sensor_origin_y_m": float(core_ray_value[2][1]),
            "measured_range_m": core_ray_value[5],
            "free_length_m": core_ray_value[4],
            "free_guard_m": free_guard,
            "intersection_distance_m": core["intersection_distance_m"],
            "intersection_x_m": core["intersection_x_m"],
            "intersection_y_m": core["intersection_y_m"],
            "intersection_grid_cell": f"{intersection_cell[0]};{intersection_cell[1]}",
            "ray_endpoint_grid_cell": f"{ray_endpoint_cell[0]};{ray_endpoint_cell[1]}",
            "endpoint_cell_included_in_free_interval": False,
            "same_endpoint_cell_conflict": intersection_cell == ray_endpoint_cell,
            "ray_semantics": "open obstacle interior intersected only before range-0.09m; tangent allowed",
        },
        {
            "constraint": "H1_H2_LATER_OCCUPIED_SEGMENT",
            "source_stamp_ns": STAMPS[4],
            "beam_index": f"{core_hit_values[0][1]};{core_hit_values[1][1]}",
            "sensor_origin_x_m": float(current[STAMPS[4]].origin[0]),
            "sensor_origin_y_m": float(current[STAMPS[4]].origin[1]),
            "measured_range_m": None,
            "free_length_m": None,
            "free_guard_m": None,
            "intersection_distance_m": core["intersection_distance_m"],
            "intersection_x_m": core["intersection_x_m"],
            "intersection_y_m": core["intersection_y_m"],
            "intersection_grid_cell": f"{intersection_cell[0]};{intersection_cell[1]}",
            "ray_endpoint_grid_cell": None,
            "endpoint_cell_included_in_free_interval": None,
            "same_endpoint_cell_conflict": False,
            "ray_semantics": "rectangle must contain both hits and therefore their convex segment",
        },
    ]

    # Compact model ablations. Every full-grid call is explicit and cached for later safety checks.
    ablation_specs: list[tuple[str, list[rulebook.RayFrame] | None, bool, str]] = [
        ("M0_FULL_CURRENT", window40, True, "current analytic FOV/(N-1)"),
        ("M1_HITS_ONLY", window40, False, "negative rays disabled"),
        ("M2_FREE_RAYS_ONLY", None, True, "unbounded without a positive placement anchor"),
        ("M3_LATEST_SCAN_ONLY", [current[STAMPS[4]].ray_frame], True, "no temporal history"),
        (
            "M4_NO_LATEST_HITS",
            [
                current[STAMPS[2]].ray_frame,
                current[STAMPS[3]].ray_frame,
                clone_ray_frame(current[STAMPS[4]], include_hits=False),
            ],
            True,
            "history plus latest free rays",
        ),
        (
            "M5_NO_LATEST_FREE_RAYS",
            [
                current[STAMPS[2]].ray_frame,
                current[STAMPS[3]].ray_frame,
                clone_ray_frame(current[STAMPS[4]], include_free=False),
            ],
            True,
            "history plus latest hits",
        ),
        (
            "M6_EXACT_BACKEND_CONVENTION",
            model_frames(evidence, "EXACT_BACKEND_LOOKUP", STAMPS[2:5]),
            True,
            "actual range-producing 2000-bin lookup",
        ),
        ("M7_EXACT_POSE_INTERPOLATION", window40, True, "same-stamp pose; delta exactly zero"),
    ]
    ablations: list[dict[str, Any]] = []
    ablation_grids: dict[str, s3.OccupancyGrid] = {}
    for name, frames_for_model, use_free, detail in ablation_specs:
        if frames_for_model is None:
            ablations.append(
                {
                    "ablation": name,
                    "possible_cell_count": None,
                    "possible_area_m2": None,
                    "empty": None,
                    "status": "NOT_IDENTIFIABLE_WITHOUT_HITS",
                    "detail": detail,
                    "dangerous_path_safe": "NOT_MEANINGFUL",
                }
            )
            continue
        grid, status = safe_grid(frames_for_model, use_free_space=use_free)
        full_grid_count += 1
        if grid is not None:
            ablation_grids[name] = grid
        ablations.append(
            {
                "ablation": name,
                "possible_cell_count": int(np.count_nonzero(grid.mask)) if grid else 0,
                "possible_area_m2": grid.possible_area_m2 if grid else 0.0,
                "empty": grid is None,
                "status": status,
                "detail": detail,
                "dangerous_path_safe": "CONSERVATIVE_EMPTY" if grid is None else "PENDING",
            }
        )
    for fine in fine_rows:
        ablations.append(
            {
                "ablation": (
                    f"M8_{fine['model']}_FINE_{fine['orientation_step_deg']}_DEG"
                ),
                "possible_cell_count": fine["survivor_orientation_count"],
                "possible_area_m2": None,
                "empty": fine["status"] == "EMPTY",
                "status": fine["status"],
                "detail": fine["method"],
                "dangerous_path_safe": (
                    "CONSERVATIVE_EMPTY"
                    if fine["status"] == "EMPTY"
                    else "SUBSET_ONLY_NOT_A_FULL_MODEL"
                ),
            }
        )

    # Candidate full models and downstream planner safety evaluation.
    removed_core_ray_frames = [
        clone_ray_frame(current[STAMPS[2]], beam_free_length=(core_ray_value[1], 0.0)),
        current[STAMPS[3]].ray_frame,
        current[STAMPS[4]].ray_frame,
    ]
    shortened_core_ray_frames = [
        clone_ray_frame(
            current[STAMPS[2]],
            beam_free_length=(
                core_ray_value[1],
                float(np.nextafter(core["intersection_distance_m"], 0.0)),
            ),
        ),
        current[STAMPS[3]].ray_frame,
        current[STAMPS[4]].ray_frame,
    ]
    correction_specs: list[tuple[str, list[rulebook.RayFrame], str, bool]] = [
        (
            "EXACT_BACKEND_LOOKUP",
            model_frames(evidence, "EXACT_BACKEND_LOOKUP", STAMPS[2:5]),
            "SCAN_CONVENTION_MISMATCH",
            True,
        ),
        (
            "REMOVE_CORE_FREE_RAY_11250000000_829",
            removed_core_ray_frames,
            "DIAGNOSTIC_OUTLIER_ABLATION",
            False,
        ),
        (
            "SHORTEN_CORE_FREE_RAY_TO_INTERSECTION",
            shortened_core_ray_frames,
            "DIAGNOSTIC_RANGE_ABLATION",
            False,
        ),
        (
            "DROP_LATEST_HITS",
            ablation_specs[4][1],
            "UNJUSTIFIED_INFORMATION_DROP",
            False,
        ),
        (
            "LATEST_SCAN_ONLY",
            [current[STAMPS[4]].ray_frame],
            "UNJUSTIFIED_HISTORY_DROP",
            False,
        ),
    ]
    correction_grids: dict[str, s3.OccupancyGrid | None] = {}
    corrections: list[dict[str, Any]] = [
        {
            "candidate_correction": "DROP_ALL_FREE_RAYS",
            "root_cause_class": "UNJUSTIFIED_NEGATIVE_EVIDENCE_DROP",
            "justified": False,
            "non_empty_restored": True,
            "possible_cell_count": int(np.count_nonzero(hits_only40.mask)),
            "possible_area_m2": hits_only40.possible_area_m2,
            "status": "NON_EMPTY",
        }
    ]
    correction_grids["DROP_ALL_FREE_RAYS"] = hits_only40
    for name, correction_frames, cause, justified in correction_specs:
        grid, status = safe_grid(correction_frames)
        full_grid_count += 1
        correction_grids[name] = grid
        corrections.append(
            {
                "candidate_correction": name,
                "root_cause_class": cause,
                "justified": justified,
                "non_empty_restored": grid is not None,
                "possible_cell_count": int(np.count_nonzero(grid.mask)) if grid else 0,
                "possible_area_m2": grid.possible_area_m2 if grid else 0.0,
                "status": status,
            }
        )

    obstacle = {
        key: float(manifest["obstacle"][key])
        for key in ("x", "y", "s", "d", "yaw", "width", "height")
    }
    case = {
        "case_id": "validation_039",
        "stamp_ns": BASELINE_STAMP,
        "manifest_s": obstacle["s"],
        "obstacle": obstacle,
        "raster": raster,
        "stream": str(time_audit.STREAM_039),
    }
    track = free_gap.TrackGeometry.from_waypoints(load_waypoints(WAYPOINTS))
    gt = time_audit.gt_representation("validation_039")
    safety_jobs = [
        (index, row, correction_grids[row["candidate_correction"]])
        for index, row in enumerate(corrections)
        if correction_grids[row["candidate_correction"]] is not None
    ]
    safety_by_name: dict[str, dict[str, Any]] = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=WORKERS) as executor:
        futures = {
            executor.submit(
                projected_safety,
                row["candidate_correction"],
                grid,
                case,
                track,
                gt,
                100 + index,
            ): row["candidate_correction"]
            for index, row, grid in safety_jobs
            if grid is not None
        }
        for future in concurrent.futures.as_completed(futures):
            safety_by_name[futures[future]] = future.result()
    for row in corrections:
        row.update(safety_by_name.get(row["candidate_correction"], {}))
        if not row["non_empty_restored"]:
            row.update(
                {
                    "gt_undercoverage_m": None,
                    "gt_overcoverage_m": None,
                    "dangerous_path_clearance_m": None,
                    "dangerous_path_hard_valid": None,
                    "dangerous_path_hard_invalid": True,
                    "dangerous_path_rejection": "CONSERVATIVE_EMPTY",
                }
            )
    safety_lookup = {row["candidate_correction"]: row for row in corrections}
    for row in ablations:
        mapping = {
            "M1_HITS_ONLY": "DROP_ALL_FREE_RAYS",
            "M3_LATEST_SCAN_ONLY": "LATEST_SCAN_ONLY",
            "M4_NO_LATEST_HITS": "DROP_LATEST_HITS",
            "M6_EXACT_BACKEND_CONVENTION": "EXACT_BACKEND_LOOKUP",
        }.get(row["ablation"])
        if mapping and mapping in safety_lookup:
            row["dangerous_path_safe"] = safety_lookup[mapping][
                "dangerous_path_hard_valid"
            ]

    corrected = safety_lookup["EXACT_BACKEND_LOOKUP"]
    if not (
        corrected["non_empty_restored"]
        and corrected["gt_undercoverage_m"] <= 1.0e-12
        and corrected["dangerous_path_hard_invalid"]
    ):
        raise RuntimeError("exact backend correction failed validation_039 safety anchor")

    # Repeat without reparsing: current +30, current +40, corrected +40, core and safety projection.
    repeat_grid30, repeat30_status = safe_grid(current_window(current, STAMPS[3]))
    repeat_grid40, repeat40_status = safe_grid(current_window(current, STAMPS[4]))
    repeat_corrected, repeat_corrected_status = safe_grid(
        model_frames(evidence, "EXACT_BACKEND_LOOKUP", STAMPS[2:5])
    )
    full_grid_count += 3
    repeat_core = find_minimal_core(current[STAMPS[2]], current[STAMPS[4]])
    assert repeat_grid30 is not None and repeat_corrected is not None
    repeat_safety = projected_safety(
        "EXACT_BACKEND_LOOKUP_REPEAT",
        repeat_corrected,
        case,
        track,
        gt,
        999,
        repeat=True,
    )
    primary_digest_fields = {
        "stamps": list(AUDIT_STAMPS),
        "plus30_status": "NON_EMPTY",
        "plus30_cells": int(np.count_nonzero(grid30.mask)),
        "plus40_status": "EMPTY",
        "corrected_status": "NON_EMPTY",
        "corrected_cells": int(np.count_nonzero(convention_grids["EXACT_BACKEND_LOOKUP"].mask)),
        "core_digest": core["digest"],
        "support": corrected.get("selected_side_support_m"),
        "dangerous_path_clearance": corrected.get("dangerous_path_clearance_m"),
        "dangerous_path_hard_invalid": corrected.get("dangerous_path_hard_invalid"),
    }
    repeat_digest_fields = {
        "stamps": list(AUDIT_STAMPS),
        "plus30_status": repeat30_status,
        "plus30_cells": int(np.count_nonzero(repeat_grid30.mask)),
        "plus40_status": repeat40_status,
        "corrected_status": repeat_corrected_status,
        "corrected_cells": int(np.count_nonzero(repeat_corrected.mask)),
        "core_digest": repeat_core["digest"],
        "support": repeat_safety.get("selected_side_support_m"),
        "dangerous_path_clearance": repeat_safety.get("dangerous_path_clearance_m"),
        "dangerous_path_hard_invalid": repeat_safety.get("dangerous_path_hard_invalid"),
    }
    determinism = {
        "primary_digest": canonical_digest(clean_json(primary_digest_fields)),
        "repeat_digest": canonical_digest(clean_json(repeat_digest_fields)),
        "bit_identical": clean_json(primary_digest_fields) == clean_json(repeat_digest_fields),
        "primary_fields": clean_json(primary_digest_fields),
        "repeat_fields": clean_json(repeat_digest_fields),
        "bag_reparsed_for_repeat": False,
    }
    if not determinism["bit_identical"]:
        raise RuntimeError(f"determinism mismatch: {determinism}")

    # Core beam numeric deltas and range precision.
    core_range = np.float32(core_ray_value[5])
    core_summary = {
        "rows": core_rows,
        "classification": "NEW_HITS_VS_OLD_FREE_RAY",
        "deletion_minimal": core["deletion_minimal"],
        "intersection_distance_m": core["intersection_distance_m"],
        "segment_fraction": core["segment_fraction"],
        "minimum_ray_shortening_m": core["minimum_ray_shortening_m"],
        "minimum_angular_shift_deg": core["minimum_angular_shift_deg"],
        "minimum_transverse_shift_m": core["minimum_transverse_shift_m"],
        "free_guard_m": free_guard,
        "range_float32_ulp_m": float(np.spacing(core_range)),
        "digest": core["digest"],
    }

    production_after = {
        name: sha256(path) for name, path in rulebook.PRODUCTION_FILES.items()
    }
    if production_before != production_after:
        raise RuntimeError("production hash changed during diagnostic audit")
    memory_after = memory_snapshot()
    self_finished = resource.getrusage(resource.RUSAGE_SELF)
    child_finished = resource.getrusage(resource.RUSAGE_CHILDREN)
    performance = clean_json(
        {
            "schema": "validation039_sensor_model_consistency_performance/1",
            "bag_parse_count": 1,
            "bag_parse_time_s": bag_parse_time,
            "worker_count_configured": WORKERS,
            "worker_policy": "one causal main sequence; up to four independent downstream safety projections",
            "wall_time_s": time.monotonic() - wall_started,
            "cpu_time_s": (self_finished.ru_utime - self_started.ru_utime)
            + (self_finished.ru_stime - self_started.ru_stime)
            + (child_finished.ru_utime - child_started.ru_utime)
            + (child_finished.ru_stime - child_started.ru_stime),
            "max_rss_bytes": max(self_finished.ru_maxrss, child_finished.ru_maxrss) * 1024,
            "memory_available_before_bytes": memory_before["memory_available_bytes"],
            "memory_available_after_bytes": memory_after["memory_available_bytes"],
            "swap_used_before_bytes": memory_before["swap_used_bytes"],
            "swap_used_after_bytes": memory_after["swap_used_bytes"],
            "swap_change_bytes": memory_after["swap_used_bytes"]
            - memory_before["swap_used_bytes"],
            "full_grid_computation_count": full_grid_count,
            "local_fine_refinement_count": len(fine_rows),
            "cached_survivor_set_reuse_count": 1,
            "closed_loop_replay_count": 0,
            "production_modification_count": 0,
            "cma_run_count": 0,
            "dataset_regeneration_count": 0,
            "diagnostic_build_time_s": args.diagnostic_build_time_s,
            "diagnostic_build_max_rss_bytes": args.diagnostic_build_max_rss_bytes,
            "cache_reuse": [
                str(TIME_AUDIT),
                str(ROOT / "runs/cmaes_tuning/s3_shadow_false_infeasible_audit_v1"),
                str(ROOT / "runs/cmaes_tuning/validation039_representation_root_cause_v1"),
                str(ROOT / "runs/cmaes_tuning/rulebook_obstacle_envelope_audit_v1"),
                str(ROOT / "runs/cmaes_tuning/rulebook_free_gap_pruning_audit_v1"),
            ],
        }
    )
    summary = clean_json(
        {
            "schema": "validation039_sensor_model_consistency_audit/1",
            "diagnostic_only": True,
            "root_cause_class": "SCAN_CONVENTION_MISMATCH",
            "empty_set_mechanism": "+20 ms free beam 829 crosses occupied segment between +40 ms hits 824 and 826 under analytic audit directions",
            "raw_sensor_internally_consistent_under_actual_backend": True,
            "current_detector_information_loss": True,
            "current_detector_exact_gate": "future target fragments contain 2-4 points; final minimum is 5",
            "time_axis": time_axis,
            "constraint_rejection_counts": rejection_counts,
            "minimal_unsat_core": core_summary,
            "raw_vs_detector_evidence": detector_rows,
            "scan_convention_comparison": convention_rows,
            "pose_timestamp_audit": pose_rows,
            "extrinsic_audit": extrinsic_rows,
            "ray_endpoint_semantics": ray_semantics_rows,
            "model_ablation": ablations,
            "candidate_corrections_safety": corrections,
            "local_fine_refinement": fine_rows,
            "contradiction_beams": contradiction_beams,
            "determinism": determinism,
            "production_hashes_before": production_before,
            "production_hashes_after": production_after,
            "production_changes": False,
            "closed_loop_replay_count": 0,
            "cma_executed": False,
            "dataset_regenerated": False,
            "gt_use_boundary": "raster used only to label recorded endpoints and evaluate downstream safety; never estimator evidence",
            "opponent_dynamic_pipeline_protection": [
                "Frenet KF unchanged",
                "map-frame velocity KF unchanged",
                "vs/vd unchanged",
                "association and Mahalanobis gating unchanged",
                "physical track identity unchanged",
                "motion voting unchanged",
                "/opp_obs unchanged",
            ],
        }
    )

    write_csv(OUTPUT / "time_axis_consistency.csv", time_axis)
    write_csv(OUTPUT / "constraint_rejection_counts.csv", rejection_counts)
    write_csv(OUTPUT / "minimal_unsat_core.csv", core_rows)
    write_csv(OUTPUT / "raw_vs_detector_evidence.csv", detector_rows)
    write_csv(OUTPUT / "scan_convention_comparison.csv", convention_rows)
    write_csv(OUTPUT / "pose_timestamp_audit.csv", pose_rows)
    write_csv(OUTPUT / "extrinsic_audit.csv", extrinsic_rows)
    write_csv(OUTPUT / "ray_endpoint_semantics.csv", ray_semantics_rows)
    write_csv(OUTPUT / "model_ablation.csv", ablations)
    write_csv(OUTPUT / "candidate_corrections_safety.csv", corrections)
    write_csv(OUTPUT / "contradiction_beams.csv", contradiction_beams)
    write_csv(OUTPUT / "local_fine_refinement.csv", fine_rows)
    (OUTPUT / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    (OUTPUT / "performance.json").write_text(
        json.dumps(performance, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    (OUTPUT / "README.md").write_text(
        render_readme(summary, performance), encoding="utf-8"
    )


if __name__ == "__main__":
    main()
