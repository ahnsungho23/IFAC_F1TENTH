#!/usr/bin/env python3
"""Offline rulebook free-gap pruning audit for the S3 set-membership shadow.

This renderer is diagnostic-only.  It reuses the stored S3 observations and the existing
BUILD_TESTING audit executable, never publishes an envelope, and never changes production input.
The placement predicate is evaluated on the same 4 mm / 2 degree possible-cell witness grid used
by S3_UNION_SIDE_SUPPORT.  Track-span extrema are evaluated exactly for the piecewise-linear
reference model: every overlapping reference segment endpoint and every rectangle-corner
breakpoint is tested.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
from dataclasses import dataclass, replace
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import time
from typing import Any, Sequence

import numpy as np

import render_rulebook_obstacle_envelope_audit as rulebook
import render_s3_shadow_false_infeasible_audit as s3
from cmaes_tuning.scenario_generator import load_waypoints


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "runs/cmaes_tuning/rulebook_free_gap_pruning_audit_v1"
RAW = OUTPUT / ".raw"
PRIOR = ROOT / "runs/cmaes_tuning/s3_shadow_false_infeasible_audit_v1"
RULEBOOK_PRIOR = ROOT / "runs/cmaes_tuning/rulebook_obstacle_envelope_audit_v1"
TEMPORAL = ROOT / "runs/cmaes_tuning/temporal_obstacle_envelope_audit_v1"
REPRESENTATION = ROOT / "runs/cmaes_tuning/validation039_representation_root_cause_v1"
TIME_AXIS = ROOT / "runs/cmaes_tuning/time_axis_feasibility_audit_v1"
SCENARIO_ROOT = ROOT / "runs/cmaes_tuning/medium_lockstep_stage1_v2/artifacts/scenarios/validation"
WAYPOINTS = ROOT / "offline_trajectory_generator/output/ifac_track/global_waypoints.json"
BINARY = ROOT / "build/local_planning/path_family_feasibility_audit"
EXACT_STAMP = rulebook.EXACT_039_STAMP
EXPECTED_HASH = rulebook.EXPECTED_039_HASH
EXPECTED_PATH_SHA256 = rulebook.EXPECTED_039_PATH_SHA256
WORKERS = 4
GRID_M = rulebook.COARSE_GRID_M
ANGLE_DEG = rulebook.COARSE_ANGLE_DEG
RULE_GAP_M = 0.5
NUMERICAL_BAND_M = math.sqrt(2.0) * GRID_M + rulebook.GEOMETRY_EPSILON_M
PRIMARY_CASES = ("normal_01_00", "normal_01_02")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--diagnostic-build-time-s", type=float, default=0.0)
    parser.add_argument("--diagnostic-build-max-rss-bytes", type=int, default=0)
    parser.add_argument("--recovered-attempt-wall-time-s", type=float, default=0.0)
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


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        raise RuntimeError(f"refusing to write empty CSV: {path}")
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


@dataclass(frozen=True)
class TrackSegment:
    s0: float
    s1: float
    x0: float
    y0: float
    tx: float
    ty: float
    nx: float
    ny: float
    length_m: float
    d_left0: float
    d_left1: float
    d_right0: float
    d_right1: float


@dataclass(frozen=True)
class TrackGeometry:
    segments: tuple[TrackSegment, ...]
    points: tuple[tuple[float, float], ...]
    track_length_m: float

    @classmethod
    def from_waypoints(cls, waypoints: Sequence[dict[str, Any]]) -> "TrackGeometry":
        points = tuple((float(item["x_m"]), float(item["y_m"])) for item in waypoints)
        closing = math.hypot(points[0][0] - points[-1][0], points[0][1] - points[-1][1])
        track_length = float(waypoints[-1]["s_m"]) + closing
        segments: list[TrackSegment] = []
        for index, first in enumerate(waypoints):
            second = waypoints[(index + 1) % len(waypoints)]
            x0, y0 = float(first["x_m"]), float(first["y_m"])
            x1, y1 = float(second["x_m"]), float(second["y_m"])
            dx, dy = x1 - x0, y1 - y0
            length = math.hypot(dx, dy)
            if length <= 1.0e-12:
                continue
            s0 = float(first["s_m"])
            s1 = track_length if index + 1 == len(waypoints) else float(second["s_m"])
            segments.append(
                TrackSegment(
                    s0=s0,
                    s1=s1,
                    x0=x0,
                    y0=y0,
                    tx=dx / length,
                    ty=dy / length,
                    nx=-dy / length,
                    ny=dx / length,
                    length_m=length,
                    d_left0=float(first["d_left"]),
                    d_left1=float(second["d_left"]),
                    d_right0=float(first["d_right"]),
                    d_right1=float(second["d_right"]),
                )
            )
        return cls(tuple(segments), points, track_length)


def unwrap_near(value: np.ndarray, expected: float, length: float) -> np.ndarray:
    delta = np.mod(value - expected, length)
    delta = np.where(delta > 0.5 * length, delta - length, delta)
    return expected + delta


def rectangle_corners(
    centre: np.ndarray,
    cosine: float,
    sine: float,
    u_min: np.ndarray,
    u_max: np.ndarray,
    v_min: np.ndarray,
    v_max: np.ndarray,
) -> np.ndarray:
    u = np.stack((u_min, u_max, u_max, u_min), axis=1)
    v = np.stack((v_min, v_min, v_max, v_max), axis=1)
    result = np.empty((u.shape[0], 4, 2), dtype=np.float64)
    result[:, :, 0] = centre[0] + u * cosine - v * sine
    result[:, :, 1] = centre[1] + u * sine + v * cosine
    return result


def _line_rectangle_interval(
    segment: TrackSegment,
    centre: np.ndarray,
    cosine: float,
    sine: float,
    u_min: np.ndarray,
    u_max: np.ndarray,
    v_min: np.ndarray,
    v_max: np.ndarray,
    q: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    px = segment.x0 + q * segment.tx
    py = segment.y0 + q * segment.ty
    du = (px - centre[0]) * cosine + (py - centre[1]) * sine
    dv = -(px - centre[0]) * sine + (py - centre[1]) * cosine
    normal_u = segment.nx * cosine + segment.ny * sine
    normal_v = -segment.nx * sine + segment.ny * cosine
    lower = np.full(q.shape, -math.inf, dtype=np.float64)
    upper = np.full(q.shape, math.inf, dtype=np.float64)
    valid = np.ones(q.shape, dtype=bool)
    for offset, direction, bound_min, bound_max in (
        (du, normal_u, u_min, u_max),
        (dv, normal_v, v_min, v_max),
    ):
        if abs(direction) <= 1.0e-12:
            valid &= (offset >= bound_min - 1.0e-12) & (offset <= bound_max + 1.0e-12)
            continue
        first = (bound_min - offset) / direction
        second = (bound_max - offset) / direction
        lower = np.maximum(lower, np.minimum(first, second))
        upper = np.minimum(upper, np.maximum(first, second))
    valid &= lower <= upper + 1.0e-12
    ratio = np.clip(q / segment.length_m, 0.0, 1.0)
    d_left = segment.d_left0 + ratio * (segment.d_left1 - segment.d_left0)
    d_right = segment.d_right0 + ratio * (segment.d_right1 - segment.d_right0)
    s_value = segment.s0 + ratio * (segment.s1 - segment.s0)
    return lower, upper, d_left, d_right, np.where(valid, s_value, math.nan)


def exact_track_span_gaps(
    track: TrackGeometry,
    expected_s: float,
    centre: np.ndarray,
    angle_deg: float,
    u_min: np.ndarray,
    u_max: np.ndarray,
    v_min: np.ndarray,
    v_max: np.ndarray,
) -> dict[str, np.ndarray]:
    """Piecewise-linear exact cross-section minima for a batch of oriented rectangles."""
    radians = math.radians(angle_deg)
    cosine, sine = math.cos(radians), math.sin(radians)
    corners = rectangle_corners(centre, cosine, sine, u_min, u_max, v_min, v_max)
    count = corners.shape[0]
    left_min = np.full(count, math.inf)
    right_min = np.full(count, math.inf)
    total_min = np.full(count, math.inf)
    left_worst_s = np.full(count, math.nan)
    right_worst_s = np.full(count, math.nan)
    total_worst_s = np.full(count, math.nan)

    # Only nearby segments can intersect a <=0.506 m witness rectangle.  The Cartesian distance
    # gate is deliberately loose and does not participate in the rule decision.
    centre_x = np.mean(corners[:, :, 0], axis=1)
    centre_y = np.mean(corners[:, :, 1], axis=1)
    for segment in track.segments:
        delta_s = float(unwrap_near(np.asarray([segment.s0]), expected_s, track.track_length_m)[0] - expected_s)
        if abs(delta_s) > 1.5:
            continue
        q_corner = (
            (corners[:, :, 0] - segment.x0) * segment.tx
            + (corners[:, :, 1] - segment.y0) * segment.ty
        )
        q_low = np.maximum(0.0, np.min(q_corner, axis=1))
        q_high = np.minimum(segment.length_m, np.max(q_corner, axis=1))
        active = q_low <= q_high + 1.0e-12
        if not np.any(active):
            continue

        q_values = [q_low, q_high]
        q_values.extend(q_corner[:, index] for index in range(4))
        for q_raw in q_values:
            q = np.clip(q_raw, q_low, q_high)
            lower, upper, d_left, d_right, s_value = _line_rectangle_interval(
                segment, centre, cosine, sine, u_min, u_max, v_min, v_max, q
            )
            valid = active & np.isfinite(s_value)
            if not np.any(valid):
                continue
            left_gap = d_left - upper
            right_gap = lower + d_right
            total_gap = d_left + d_right - (upper - lower)
            update = valid & (left_gap < left_min)
            left_min[update] = left_gap[update]
            left_worst_s[update] = s_value[update]
            update = valid & (right_gap < right_min)
            right_min[update] = right_gap[update]
            right_worst_s[update] = s_value[update]
            update = valid & (total_gap < total_min)
            total_min[update] = total_gap[update]
            total_worst_s[update] = s_value[update]

    if np.any(~np.isfinite(left_min)):
        raise RuntimeError("track-span gap evaluation missed a rectangle")
    return {
        "left": left_min,
        "right": right_min,
        "total": total_min,
        "left_worst_s": left_worst_s,
        "right_worst_s": right_worst_s,
        "total_worst_s": total_worst_s,
        "centre_x": centre_x,
        "centre_y": centre_y,
    }


def sensor_consistent_witnesses(
    frames: Sequence[rulebook.RayFrame],
    grid: s3.OccupancyGrid,
    indices: np.ndarray,
    angle_deg: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    hits = np.vstack([frame.hits for frame in frames])
    centre = np.mean(hits, axis=0)
    radians = math.radians(angle_deg)
    cosine, sine = math.cos(radians), math.sin(radians)
    hit_u = (hits[:, 0] - centre[0]) * cosine + (hits[:, 1] - centre[1]) * sine
    hit_v = -(hits[:, 0] - centre[0]) * sine + (hits[:, 1] - centre[1]) * cosine
    hu_min, hu_max = float(np.min(hit_u)), float(np.max(hit_u))
    hv_min, hv_max = float(np.min(hit_v)), float(np.max(hit_v))
    effective = rulebook.MAX_SIDE_M + math.sqrt(2.0) * grid.resolution_m
    if hu_max - hu_min > effective + rulebook.GEOMETRY_EPSILON_M or hv_max - hv_min > effective + rulebook.GEOMETRY_EPSILON_M:
        return (np.asarray([], dtype=np.int64),) * 6

    flat_x, flat_y = np.meshgrid(grid.xs, grid.ys)
    query_x, query_y = flat_x.ravel()[indices], flat_y.ravel()[indices]
    query_u = (query_x - centre[0]) * cosine + (query_y - centre[1]) * sine
    query_v = -(query_x - centre[0]) * sine + (query_y - centre[1]) * cosine
    u_min, u_max = np.minimum(query_u, hu_min), np.maximum(query_u, hu_max)
    v_min, v_max = np.minimum(query_v, hv_min), np.maximum(query_v, hv_max)
    valid = (u_max - u_min <= effective + rulebook.GEOMETRY_EPSILON_M) & (
        v_max - v_min <= effective + rulebook.GEOMETRY_EPSILON_M
    )
    indices, u_min, u_max, v_min, v_max = (
        values[valid] for values in (indices, u_min, u_max, v_min, v_max)
    )
    if not indices.size:
        return indices, u_min, u_max, v_min, v_max, centre

    origins = np.vstack(
        [np.repeat(frame.origin[None, :], frame.directions.shape[0], axis=0) for frame in frames]
    )
    directions = np.vstack([frame.directions for frame in frames])
    lengths = np.concatenate([frame.free_lengths for frame in frames])
    origin_u = (origins[:, 0] - centre[0]) * cosine + (origins[:, 1] - centre[1]) * sine
    origin_v = -(origins[:, 0] - centre[0]) * sine + (origins[:, 1] - centre[1]) * cosine
    direction_u = directions[:, 0] * cosine + directions[:, 1] * sine
    direction_v = -directions[:, 0] * sine + directions[:, 1] * cosine
    alive = np.ones(indices.size, dtype=bool)
    for ray_index in range(lengths.size):
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
    return indices[alive], u_min[alive], u_max[alive], v_min[alive], v_max[alive], centre


def grid_with_mask(base: s3.OccupancyGrid, mask: np.ndarray, label: str) -> s3.OccupancyGrid:
    rows, columns = np.nonzero(mask)
    if not rows.size:
        raise RuntimeError(f"{label}: rule removed every possible cell")
    half = 0.5 * base.resolution_m
    bounds = (
        float(np.min(base.xs[columns]) - half),
        float(np.max(base.xs[columns]) + half),
        float(np.min(base.ys[rows]) - half),
        float(np.max(base.ys[rows]) + half),
    )
    digest = canonical_digest(
        {
            "base": base.digest,
            "label": label,
            "mask_sha256": hashlib.sha256(np.packbits(mask).tobytes()).hexdigest(),
        }
    )
    return replace(
        base,
        mask=mask,
        bounds=bounds,
        possible_area_m2=float(rows.size) * base.resolution_m**2,
        aabb_area_m2=(bounds[1] - bounds[0]) * (bounds[3] - bounds[2]),
        digest=digest,
    )


def enumerate_rule_filtered_set(
    case: dict[str, Any], track: TrackGeometry
) -> tuple[dict[str, Any], s3.OccupancyGrid, s3.OccupancyGrid, s3.OccupancyGrid]:
    started = time.monotonic()
    frames = s3.selected_frames_window(case["frames"], int(case["frame_now_ns"]), 20)
    base = s3.detailed_possible_occupancy(frames, GRID_M, ANGLE_DEG)
    base_flat = base.mask.ravel()
    base_indices = np.flatnonzero(base_flat)
    side_flat = np.zeros(base_flat.shape, dtype=bool)
    total_flat = np.zeros(base_flat.shape, dtype=bool)

    first_angle = np.full(base_flat.shape, math.nan)
    first_u_min = np.full(base_flat.shape, math.nan)
    first_u_max = np.full(base_flat.shape, math.nan)
    first_v_min = np.full(base_flat.shape, math.nan)
    first_v_max = np.full(base_flat.shape, math.nan)
    first_left = np.full(base_flat.shape, math.nan)
    first_right = np.full(base_flat.shape, math.nan)
    first_total = np.full(base_flat.shape, math.nan)
    first_left_s = np.full(base_flat.shape, math.nan)
    first_right_s = np.full(base_flat.shape, math.nan)
    hypothesis_before = 0
    hypothesis_after_side = 0
    hypothesis_after_total = 0
    side_band_hypotheses = 0
    total_band_hypotheses = 0
    gap_cpu_started = time.process_time()

    for angle in base.angles_deg:
        indices, u_min, u_max, v_min, v_max, centre = sensor_consistent_witnesses(
            frames, base, base_indices, float(angle)
        )
        if not indices.size:
            continue
        gaps = exact_track_span_gaps(
            track,
            float(case["manifest_s"]),
            centre,
            float(angle),
            u_min,
            u_max,
            v_min,
            v_max,
        )
        best_side = np.maximum(gaps["left"], gaps["right"])
        side_valid = best_side >= RULE_GAP_M
        total_valid = gaps["total"] >= RULE_GAP_M
        hypothesis_before += int(indices.size)
        hypothesis_after_side += int(np.count_nonzero(side_valid))
        hypothesis_after_total += int(np.count_nonzero(total_valid))
        side_band_hypotheses += int(np.count_nonzero(np.abs(best_side - RULE_GAP_M) <= NUMERICAL_BAND_M))
        total_band_hypotheses += int(np.count_nonzero(np.abs(gaps["total"] - RULE_GAP_M) <= NUMERICAL_BAND_M))
        side_flat[indices[side_valid]] = True
        total_flat[indices[total_valid]] = True

        new = ~np.isfinite(first_angle[indices])
        target = indices[new]
        first_angle[target] = float(angle)
        first_u_min[target], first_u_max[target] = u_min[new], u_max[new]
        first_v_min[target], first_v_max[target] = v_min[new], v_max[new]
        first_left[target], first_right[target], first_total[target] = (
            gaps["left"][new], gaps["right"][new], gaps["total"][new]
        )
        first_left_s[target], first_right_s[target] = (
            gaps["left_worst_s"][new], gaps["right_worst_s"][new]
        )

    if np.any(~np.isfinite(first_angle[base_indices])):
        raise RuntimeError(f"{case['case_id']}: recomputed witness set does not cover cached S3 cells")
    side_mask = side_flat.reshape(base.mask.shape)
    total_mask = total_flat.reshape(base.mask.shape)
    if np.any(side_mask & ~base.mask) or np.any(total_mask & ~base.mask):
        raise RuntimeError("rule filter grew the S3 possible set")
    side_grid = grid_with_mask(base, side_mask, "RULE_SIDE")
    total_grid = grid_with_mask(base, total_mask, "RULE_TOTAL")

    flat_x, flat_y = np.meshgrid(base.xs, base.ys)
    removed = base_flat & ~side_flat
    side = s3.passing_side(case["gt_row"]) if case["case_id"] != "validation_039" else "right"
    removed_indices = np.flatnonzero(removed)
    extremal: dict[str, Any] | None = None
    extremal_indices = removed_indices if removed_indices.size else base_indices
    if extremal_indices.size:
        projection = project_points_to_track(
            track,
            flat_x.ravel()[extremal_indices],
            flat_y.ravel()[extremal_indices],
            float(case["manifest_s"]),
        )
        local = int(np.argmax(projection["d"]) if side == "left" else np.argmin(projection["d"]))
        index = int(extremal_indices[local])
        angle = float(first_angle[index])
        radians = math.radians(angle)
        u_mid = 0.5 * (first_u_min[index] + first_u_max[index])
        v_mid = 0.5 * (first_v_min[index] + first_v_max[index])
        hits_centre = np.mean(np.vstack([frame.hits for frame in frames]), axis=0)
        centre_x = hits_centre[0] + u_mid * math.cos(radians) - v_mid * math.sin(radians)
        centre_y = hits_centre[1] + u_mid * math.sin(radians) + v_mid * math.cos(radians)
        extremal = {
            "support_role": "dominant_removed" if removed_indices.size else "dominant_retained",
            "witness_cell_x_m": float(flat_x.ravel()[index]),
            "witness_cell_y_m": float(flat_y.ravel()[index]),
            "projected_cell_s_m": float(projection["s"][local]),
            "projected_cell_d_m": float(projection["d"][local]),
            "width_u_m": float(first_u_max[index] - first_u_min[index]),
            "length_v_m": float(first_v_max[index] - first_v_min[index]),
            "orientation_deg": angle,
            "pose_x_m": float(centre_x),
            "pose_y_m": float(centre_y),
            "minimum_left_gap_m": float(first_left[index]),
            "minimum_right_gap_m": float(first_right[index]),
            "minimum_total_open_m": float(first_total[index]),
            "left_worst_s_m": float(first_left_s[index]),
            "right_worst_s_m": float(first_right_s[index]),
            "removed_by_F_SIDE_GAP": bool(max(first_left[index], first_right[index]) < RULE_GAP_M),
            "removed_by_F_TOTAL_OPEN": bool(first_total[index] < RULE_GAP_M),
            "retained_by_F_SIDE_GAP": bool(max(first_left[index], first_right[index]) >= RULE_GAP_M),
            "retained_by_F_TOTAL_OPEN": bool(first_total[index] >= RULE_GAP_M),
        }

    result = {
        "case_id": case["case_id"],
        "grid_resolution_m": GRID_M,
        "angle_step_deg": ANGLE_DEG,
        "rule_threshold_m": RULE_GAP_M,
        "numerical_band_m": NUMERICAL_BAND_M,
        "possible_hypothesis_count_before": hypothesis_before,
        "possible_hypothesis_count_after_side": hypothesis_after_side,
        "possible_hypothesis_count_after_total": hypothesis_after_total,
        "possible_cell_count_before": int(np.count_nonzero(base.mask)),
        "possible_cell_count_after_side": int(np.count_nonzero(side_mask)),
        "possible_cell_count_after_total": int(np.count_nonzero(total_mask)),
        "possible_area_before_m2": base.possible_area_m2,
        "possible_area_after_side_m2": side_grid.possible_area_m2,
        "possible_area_after_total_m2": total_grid.possible_area_m2,
        "hypotheses_in_side_numerical_band": side_band_hypotheses,
        "hypotheses_in_total_numerical_band": total_band_hypotheses,
        "dominant_removed_hypothesis": extremal,
        "set_digest_base": base.digest,
        "set_digest_side": side_grid.digest,
        "set_digest_total": total_grid.digest,
        "rule_filter_wall_s": time.monotonic() - started,
        "rule_gap_cpu_s": time.process_time() - gap_cpu_started,
    }
    return result, base, side_grid, total_grid


def project_points_to_track(
    track: TrackGeometry, xs: np.ndarray, ys: np.ndarray, expected_s: float
) -> dict[str, np.ndarray]:
    best_distance = np.full(xs.shape, math.inf)
    best_s = np.full(xs.shape, math.nan)
    best_d = np.full(xs.shape, math.nan)
    for segment in track.segments:
        ratio = np.clip(
            ((xs - segment.x0) * segment.tx + (ys - segment.y0) * segment.ty)
            / segment.length_m,
            0.0,
            1.0,
        )
        qx = segment.x0 + ratio * segment.length_m * segment.tx
        qy = segment.y0 + ratio * segment.length_m * segment.ty
        dx, dy = xs - qx, ys - qy
        distance = dx * dx + dy * dy
        update = distance < best_distance
        raw_s = segment.s0 + ratio * (segment.s1 - segment.s0)
        best_distance[update] = distance[update]
        best_s[update] = raw_s[update]
        best_d[update] = (segment.tx * dy - segment.ty * dx)[update]
    best_s = unwrap_near(best_s, expected_s, track.track_length_m)
    return {"s": best_s, "d": best_d}


def rectangle_gap(
    track: TrackGeometry, expected_s: float, polygon: Sequence[tuple[float, float]]
) -> dict[str, float | bool]:
    points = np.asarray(polygon, dtype=np.float64)
    edge = points[1] - points[0]
    angle = math.degrees(math.atan2(edge[1], edge[0]))
    cosine, sine = math.cos(math.radians(angle)), math.sin(math.radians(angle))
    centre = np.mean(points, axis=0)
    local_u = (points[:, 0] - centre[0]) * cosine + (points[:, 1] - centre[1]) * sine
    local_v = -(points[:, 0] - centre[0]) * sine + (points[:, 1] - centre[1]) * cosine
    gap = exact_track_span_gaps(
        track,
        expected_s,
        centre,
        angle,
        np.asarray([np.min(local_u)]),
        np.asarray([np.max(local_u)]),
        np.asarray([np.min(local_v)]),
        np.asarray([np.max(local_v)]),
    )
    left, right, total = float(gap["left"][0]), float(gap["right"][0]), float(gap["total"][0])
    return {
        "minimum_left_gap_m": left,
        "minimum_right_gap_m": right,
        "minimum_total_open_m": total,
        "left_worst_s_m": float(gap["left_worst_s"][0]),
        "right_worst_s_m": float(gap["right_worst_s"][0]),
        "F_SIDE_GAP_compliant": max(left, right) >= RULE_GAP_M,
        "F_TOTAL_OPEN_compliant": total >= RULE_GAP_M,
        "within_numerical_band": abs(max(left, right) - RULE_GAP_M) <= NUMERICAL_BAND_M,
    }


def load_baseline_support(case_id: str) -> dict[str, Any]:
    directory = PRIOR / ".raw" / case_id
    path = directory / "result.json"
    if not path.is_file():
        path = directory / "repeat_result.json"
    data = json.loads(path.read_text(encoding="utf-8"))
    return data["possible_set_support"]


def project_grid_support(
    case: dict[str, Any], grid: s3.OccupancyGrid, strategy: str, directory: Path
) -> tuple[dict[str, Any], float]:
    boxes = s3.occupied_cell_boxes(grid)
    spec = rulebook.make_cpp_spec(
        int(case["stamp_ns"]),
        float(case["manifest_s"]),
        rulebook.nominal_polygon(case["obstacle"]),
        case["raster"],
        boxes,
    )
    rows, elapsed = s3.run_cpp_spec(Path(case["stream"]), spec, directory, "CORRIDOR_ONLY")
    row = rows["TF_UNION_SUPPORT_INTERNAL"]
    return {
        "s_min": float(row["s_min"]),
        "s_max": float(row["s_max"]),
        "d_min": float(row["d_min"]),
        "d_max": float(row["d_max"]),
        "projected_boundary_cell_count": len(boxes),
        "full_possible_cell_count": int(np.count_nonzero(grid.mask)),
        "strategy": strategy,
    }, elapsed


def combined_support_spec(
    case: dict[str, Any], supports: dict[str, dict[str, Any]], grids: dict[str, s3.OccupancyGrid]
) -> str:
    parser_anchor = next(iter(grids.values())).bounds
    base = rulebook.make_cpp_spec(
        int(case["stamp_ns"]),
        float(case["manifest_s"]),
        rulebook.nominal_polygon(case["obstacle"]),
        case["raster"],
        {"PARSER_ANCHOR": parser_anchor},
    )
    lines = [
        line for line in base.splitlines()
        if not line.startswith("POLYGON\tTC_PARSER_ANCHOR\t")
    ]
    lines.pop()
    for name, support in supports.items():
        bounds = grids[name].bounds
        lines.append(
            "\t".join(
                [
                    "FRENET_BOX",
                    name,
                    "rulebook_free_gap_possible_set_support",
                    repr(0.5 * (support["s_min"] + support["s_max"])),
                    repr(support["s_min"]),
                    repr(support["s_max"]),
                    repr(support["d_min"]),
                    repr(support["d_max"]),
                    "true",
                    repr(bounds[0]),
                    repr(bounds[1]),
                    repr(bounds[2]),
                    repr(bounds[3]),
                    "0",
                    "0",
                    "0",
                ]
            )
        )
    lines.append("END_SPEC")
    return "\n".join(lines) + "\n"


def analyze_case(case: dict[str, Any], track: TrackGeometry, repeat: bool = False) -> dict[str, Any]:
    started = time.monotonic()
    result, base, side, total = enumerate_rule_filtered_set(case, track)
    case_raw = RAW / case["case_id"] / ("repeat" if repeat else "primary")
    side_support, side_projection_s = project_grid_support(
        case, side, "TF_RULE_SIDE", case_raw / "side_projection"
    )
    total_support, total_projection_s = project_grid_support(
        case, total, "TF_RULE_TOTAL", case_raw / "total_projection"
    )
    base_support = load_baseline_support(case["case_id"])
    supports = {
        "TF_BASE": base_support,
        "TF_RULE_SIDE": side_support,
        "TF_RULE_TOTAL": total_support,
    }
    grids = {"TF_BASE": base, "TF_RULE_SIDE": side, "TF_RULE_TOTAL": total}

    contribution: dict[str, Any] | None = None
    if case["case_id"] in PRIMARY_CASES and not repeat:
        frames = s3.selected_frames_window(case["frames"], int(case["frame_now_ns"]), 20)
        size_grid = s3.detailed_possible_occupancy(frames, GRID_M, ANGLE_DEG, use_free_space=False)
        domain_grid = grid_with_mask(size_grid, np.ones(size_grid.mask.shape, dtype=bool), "BOUNDED_DOMAIN")
        size_support, size_projection_s = project_grid_support(
            case, size_grid, "TF_SIZE_PRIOR", case_raw / "size_projection"
        )
        domain_support, domain_projection_s = project_grid_support(
            case, domain_grid, "TF_DOMAIN", case_raw / "domain_projection"
        )
        supports.update({"TF_DOMAIN": domain_support, "TF_SIZE_PRIOR": size_support})
        grids.update({"TF_DOMAIN": domain_grid, "TF_SIZE_PRIOR": size_grid})
        contribution = {
            "bounded_domain_area_m2": domain_grid.possible_area_m2,
            "after_size_prior_area_m2": size_grid.possible_area_m2,
            "after_sensor_free_rays_area_m2": base.possible_area_m2,
            "after_free_gap_area_m2": side.possible_area_m2,
            "size_prior_area_reduction_percent": 100.0 * (1.0 - size_grid.possible_area_m2 / domain_grid.possible_area_m2),
            "sensor_free_ray_area_reduction_percent": 100.0 * (1.0 - base.possible_area_m2 / size_grid.possible_area_m2),
            "free_gap_area_reduction_percent": 100.0 * (1.0 - side.possible_area_m2 / base.possible_area_m2),
            "additional_projection_time_s": size_projection_s + domain_projection_s,
        }

    expected = EXPECTED_HASH if case["case_id"] == "validation_039" else "CORRIDOR_ONLY"
    cpp, combined_s = s3.run_cpp_spec(
        Path(case["stream"]),
        combined_support_spec(case, supports, grids),
        case_raw / "combined",
        expected,
    )
    result.update(
        {
            "supports": supports,
            "cpp": cpp,
            "contribution": contribution,
            "timing": {
                "rule_filter_s": result.pop("rule_filter_wall_s"),
                "side_projection_s": side_projection_s,
                "total_projection_s": total_projection_s,
                "combined_cpp_s": combined_s,
                "worker_wall_s": time.monotonic() - started,
                "rule_gap_cpu_s": result.pop("rule_gap_cpu_s"),
            },
        }
    )
    digest_input = {
        "case_id": case["case_id"],
        "count": result["possible_hypothesis_count_after_side"],
        "cell_count": result["possible_cell_count_after_side"],
        "set_digest": result["set_digest_side"],
        "support": side_support,
        "corridor": cpp["TF_RULE_SIDE"],
    }
    result["digest"] = canonical_digest(digest_input)
    case_raw.mkdir(parents=True, exist_ok=True)
    (case_raw / "result.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return result


def build_cases() -> tuple[list[dict[str, Any]], dict[str, dict[str, str]], float]:
    prior_normal = read_csv(RULEBOOK_PRIOR / "normal_case_results.csv")
    prior_039 = read_csv(RULEBOOK_PRIOR / "validation039_rulebook_timeline.csv")
    lookup: dict[tuple[str, str], dict[str, str]] = {}
    for row in prior_normal:
        lookup[(row["case_id"], row["strategy"])] = row
    for row in prior_039:
        if int(row["scan_stamp_ns"]) == EXACT_STAMP:
            lookup[(row["case_id"], row["strategy"])] = row
    normal_lookup = {
        row["case_id"]: row
        for row in prior_normal
        if row["strategy"] == "S3_RULE_UT20_ORIENT"
    }
    manifest_039 = json.loads((SCENARIO_ROOT / "validation_039/manifest.json").read_text())
    waypoints = load_waypoints(WAYPOINTS)
    cases = [
        s3.reconstruct_normal_case(
            case_id, lookup, waypoints, manifest_039["simulator_collision_model"]
        )
        for case_id in sorted(normal_lookup)
    ]
    case_039, bag_parse_time = s3.validation039_case(lookup)
    return [case_039] + cases, normal_lookup, bag_parse_time


def gt_rule_compliance(case: dict[str, Any], track: TrackGeometry) -> dict[str, Any]:
    obstacle = case["obstacle"]
    polygon = rulebook.nominal_polygon(obstacle)
    gap = rectangle_gap(track, float(case["manifest_s"]), polygon)
    strict_size = float(obstacle["width"]) < 0.5 and float(obstacle["height"]) < 0.5
    return {
        "case_id": case["case_id"],
        "category": case["category"],
        "width_m": float(obstacle["width"]),
        "height_m": float(obstacle["height"]),
        "strict_dimension_compliant": strict_size,
        **gap,
        "primary_F_SIDE_benchmark_included": strict_size and bool(gap["F_SIDE_GAP_compliant"]),
    }


def boundary_metrics(row: dict[str, str], gt: dict[str, str], side: str) -> tuple[float, float]:
    _, under, over = rulebook.boundary_error(row, gt, side)
    return under * 1000.0, over * 1000.0


def percentile(values: Sequence[float], quantile: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=np.float64), quantile))


def strategy_stats(
    strategy: str,
    key: str,
    normal_cases: Sequence[dict[str, Any]],
    results: dict[str, dict[str, Any]],
    compliance: dict[str, dict[str, Any]],
    anchor_safe: bool,
) -> dict[str, Any]:
    values: list[dict[str, Any]] = []
    for case in normal_cases:
        if not compliance[case["case_id"]]["primary_F_SIDE_benchmark_included"]:
            continue
        prior = case["gt_row"]
        row = results[case["case_id"]]["cpp"][key]
        left_gt = float(prior["gt_planner_left_corridor_m"])
        right_gt = float(prior["gt_planner_right_corridor_m"])
        left = float(row["left_corridor"])
        right = float(row["right_corridor"])
        side = s3.passing_side(prior)
        gt = results[case["case_id"]]["cpp"]["G0"]
        under, over = boundary_metrics(row, gt, side)
        values.append(
            {
                "false_feasible": (left_gt <= 0.0 < left) or (right_gt <= 0.0 < right),
                "false_infeasible": (left_gt > 0.0 >= left) or (right_gt > 0.0 >= right),
                "under": under,
                "over": over,
                "corridor_loss": max(0.0, float(prior[f"gt_planner_{side}_corridor_m"]) - float(row[f"{side}_corridor"])) * 1000.0,
                "small": float(prior["obstacle_width_m"]) <= 0.3 and float(prior["obstacle_height_m"]) <= 0.3,
            }
        )
    count = len(values)
    false_feasible = sum(value["false_feasible"] for value in values)
    false_infeasible = sum(value["false_infeasible"] for value in values)
    if not anchor_safe or false_feasible:
        classification = "UNSAFE"
    elif false_infeasible <= 1:
        classification = "PROMISING_SHADOW"
    elif false_infeasible:
        classification = "OVER_CONSERVATIVE"
    else:
        classification = "PROMISING_SHADOW"
    return {
        "strategy": strategy,
        "case_count": count,
        "excluded_ambiguity_stress_count": len(normal_cases) - count,
        "false_feasible_count": false_feasible,
        "false_infeasible_count": false_infeasible,
        "false_feasible_rate": false_feasible / count,
        "false_infeasible_rate": false_infeasible / count,
        "mean_boundary_undercoverage_mm": float(np.mean([value["under"] for value in values])),
        "p95_boundary_undercoverage_mm": percentile([value["under"] for value in values], 95),
        "max_boundary_undercoverage_mm": max(value["under"] for value in values),
        "mean_boundary_overcoverage_mm": float(np.mean([value["over"] for value in values])),
        "p95_boundary_overcoverage_mm": percentile([value["over"] for value in values], 95),
        "max_boundary_overcoverage_mm": max(value["over"] for value in values),
        "mean_corridor_loss_mm": float(np.mean([value["corridor_loss"] for value in values])),
        "small_obstacle_false_infeasible_count": sum(value["false_infeasible"] and value["small"] for value in values),
        "validation039_safety_retained": anchor_safe,
        "classification": classification,
    }


def old_scenario_rows(track: TrackGeometry) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for scenario in ("validation_019", "validation_034"):
        manifest = json.loads((SCENARIO_ROOT / scenario / "manifest.json").read_text())
        obstacle = manifest["obstacle"]
        gap = rectangle_gap(track, float(obstacle["s"]), rulebook.nominal_polygon(obstacle))
        strict_size = float(obstacle["width"]) < 0.5 and float(obstacle["height"]) < 0.5
        overall = strict_size and bool(gap["F_SIDE_GAP_compliant"])
        rows.append(
            {
                "scenario": scenario,
                "width_m": obstacle["width"],
                "height_m": obstacle["height"],
                "strict_dimension_compliant": strict_size,
                **gap,
                "overall_rulebook_placement_compliant": overall,
                "classification": "IN_DISTRIBUTION_RULEBOOK" if overall else "OUT_OF_DISTRIBUTION_RULEBOOK_STRESS",
                "replay_count": 0,
            }
        )
    return rows


def main() -> int:
    args = parse_args()
    if not BINARY.is_file():
        raise RuntimeError(f"missing existing diagnostic binary: {BINARY}")
    OUTPUT.mkdir(parents=True, exist_ok=True)
    RAW.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    cpu_self_before = resource.getrusage(resource.RUSAGE_SELF)
    cpu_children_before = resource.getrusage(resource.RUSAGE_CHILDREN)
    memory_before = memory_snapshot()
    production_before = {name: sha256(path) for name, path in rulebook.PRODUCTION_FILES.items()}

    waypoints = load_waypoints(WAYPOINTS)
    track = TrackGeometry.from_waypoints(waypoints)
    cases, normal_lookup, bag_parse_time = build_cases()
    normal_cases = [case for case in cases if case["case_id"] != "validation_039"]
    compliance_rows = [gt_rule_compliance(case, track) for case in normal_cases]
    compliance = {row["case_id"]: row for row in compliance_rows}

    cached_results: dict[str, dict[str, Any]] = {}
    pending_cases: list[dict[str, Any]] = []
    for case in cases:
        cached = RAW / case["case_id"] / "primary/result.json"
        if cached.is_file():
            cached_results[case["case_id"]] = json.loads(cached.read_text(encoding="utf-8"))
        else:
            pending_cases.append(case)
    parallel_started = time.monotonic()
    with concurrent.futures.ProcessPoolExecutor(max_workers=WORKERS) as executor:
        futures = {
            executor.submit(analyze_case, case, track, False): case["case_id"]
            for case in pending_cases
        }
        computed_results = {
            futures[future]: future.result() for future in concurrent.futures.as_completed(futures)
        }
    results = {**cached_results, **computed_results}
    if set(results) != {case["case_id"] for case in cases}:
        raise RuntimeError("primary case cache/computation coverage mismatch")
    parallel_wall = time.monotonic() - parallel_started
    memory_after_parallel = memory_snapshot()
    if memory_after_parallel["swap_used_bytes"] > memory_before["swap_used_bytes"]:
        raise RuntimeError("swap increased during workers=4; results rejected")

    repeat_started = time.monotonic()
    repeat_cases = [next(case for case in cases if case["case_id"] == item) for item in ("validation_039", "normal_01_00")]
    repeats: list[dict[str, Any]] = []
    pending_repeats: list[dict[str, Any]] = []
    for case in repeat_cases:
        cached = RAW / case["case_id"] / "repeat/result.json"
        if cached.is_file():
            repeats.append(json.loads(cached.read_text(encoding="utf-8")))
        else:
            pending_repeats.append(case)
    with concurrent.futures.ProcessPoolExecutor(max_workers=2) as executor:
        repeats.extend(
            executor.map(lambda_pair_analyze, [(case, track) for case in pending_repeats])
        )
    repeat_time = time.monotonic() - repeat_started
    repeat_by_id = {value["case_id"]: value for value in repeats}

    determinism: dict[str, Any] = {}
    for case_id in ("validation_039", "normal_01_00"):
        first, second = results[case_id], repeat_by_id[case_id]
        first_row, second_row = first["cpp"]["TF_RULE_SIDE"], second["cpp"]["TF_RULE_SIDE"]
        checks = {
            "hypothesis_count_identical": first["possible_hypothesis_count_after_side"] == second["possible_hypothesis_count_after_side"],
            "selected_side_support_identical": first["supports"]["TF_RULE_SIDE"] == second["supports"]["TF_RULE_SIDE"],
            "corridor_identical": (first_row["left_corridor"], first_row["right_corridor"]) == (second_row["left_corridor"], second_row["right_corridor"]),
            "same_path_clearance_identical": first_row.get("same_path_production_clearance") == second_row.get("same_path_production_clearance"),
            "classification_identical": first_row.get("same_path_hard_valid") == second_row.get("same_path_hard_valid"),
            "digest_identical": first["digest"] == second["digest"],
        }
        determinism[case_id] = checks
        if not all(checks.values()):
            raise RuntimeError(f"determinism failure for {case_id}: {checks}")

    anchor_rows: list[dict[str, Any]] = []
    anchor = results["validation_039"]
    gt = anchor["cpp"]["G1"]
    for strategy, key, area_key, count_key in (
        ("S3_BASE", "TF_BASE", "possible_area_before_m2", "possible_hypothesis_count_before"),
        ("S3_RULE_SIDE", "TF_RULE_SIDE", "possible_area_after_side_m2", "possible_hypothesis_count_after_side"),
        ("S3_RULE_TOTAL", "TF_RULE_TOTAL", "possible_area_after_total_m2", "possible_hypothesis_count_after_total"),
    ):
        row = anchor["cpp"][key]
        under, over = boundary_metrics(row, gt, "right")
        hard_invalid = row["same_path_hard_valid"] == "false"
        anchor_rows.append(
            {
                "strategy": strategy,
                "source_stamp_ns": EXACT_STAMP,
                "path_geometry_hash": EXPECTED_HASH,
                "serialized_path_sha256": EXPECTED_PATH_SHA256,
                "possible_hypothesis_count": anchor[count_key],
                "possible_cell_count": anchor["possible_cell_count_before" if strategy == "S3_BASE" else "possible_cell_count_after_side" if strategy == "S3_RULE_SIDE" else "possible_cell_count_after_total"],
                "possible_area_m2": anchor[area_key],
                "selected_side_support_d_min_m": float(row["d_min"]),
                "gt_boundary_undercoverage_mm": under,
                "gt_boundary_overcoverage_mm": over,
                "same_path_clearance_m": float(row["same_path_production_clearance"]),
                "same_path_hard_invalid": hard_invalid,
                "classification": "ANCHOR_RETAINED" if hard_invalid and under == 0.0 else "UNSAFE",
            }
        )
    anchor_safe = {
        row["strategy"]: row["classification"] == "ANCHOR_RETAINED" for row in anchor_rows
    }

    comparison_rows: list[dict[str, Any]] = []
    for case in normal_cases:
        result = results[case["case_id"]]
        prior = normal_lookup[case["case_id"]]
        passing = s3.passing_side(prior)
        for strategy, key in (
            ("S3_BASE", "TF_BASE"),
            ("S3_RULE_SIDE", "TF_RULE_SIDE"),
            ("S3_RULE_TOTAL", "TF_RULE_TOTAL"),
        ):
            row = result["cpp"][key]
            gt_row = result["cpp"]["G0"]
            under, over = boundary_metrics(row, gt_row, passing)
            left_gt, right_gt = float(prior["gt_planner_left_corridor_m"]), float(prior["gt_planner_right_corridor_m"])
            left, right = float(row["left_corridor"]), float(row["right_corridor"])
            comparison_rows.append(
                {
                    "case_id": case["case_id"],
                    "category": case["category"],
                    "primary_F_SIDE_included": compliance[case["case_id"]]["primary_F_SIDE_benchmark_included"],
                    "strategy": strategy,
                    "passing_side": passing,
                    "gt_passing_corridor_m": float(prior[f"gt_planner_{passing}_corridor_m"]),
                    "shadow_passing_corridor_m": float(row[f"{passing}_corridor"]),
                    "false_feasible": (left_gt <= 0.0 < left) or (right_gt <= 0.0 < right),
                    "false_infeasible": (left_gt > 0.0 >= left) or (right_gt > 0.0 >= right),
                    "boundary_undercoverage_mm": under,
                    "boundary_overcoverage_mm": over,
                    "corridor_loss_mm": max(0.0, float(prior[f"gt_planner_{passing}_corridor_m"]) - float(row[f"{passing}_corridor"])) * 1000.0,
                }
            )

    strategy_rows = [
        strategy_stats("S3_BASE", "TF_BASE", normal_cases, results, compliance, anchor_safe["S3_BASE"]),
        strategy_stats("S3_RULE_SIDE", "TF_RULE_SIDE", normal_cases, results, compliance, anchor_safe["S3_RULE_SIDE"]),
        strategy_stats("S3_RULE_TOTAL", "TF_RULE_TOTAL", normal_cases, results, compliance, anchor_safe["S3_RULE_TOTAL"]),
    ]

    false_rows: list[dict[str, Any]] = []
    extrema_rows: list[dict[str, Any]] = []
    interpretation_rows: list[dict[str, Any]] = []
    for case_id in PRIMARY_CASES:
        result = results[case_id]
        prior = normal_lookup[case_id]
        side_name = s3.passing_side(prior)
        base_row, side_row, total_row = (
            result["cpp"]["TF_BASE"], result["cpp"]["TF_RULE_SIDE"], result["cpp"]["TF_RULE_TOTAL"]
        )
        support_before = float(base_row["d_max"] if side_name == "left" else base_row["d_min"])
        support_after = float(side_row["d_max"] if side_name == "left" else side_row["d_min"])
        support_reduction = (support_before - support_after if side_name == "left" else support_after - support_before) * 1000.0
        row = {
            "case_id": case_id,
            "passing_side": side_name,
            "base_corridor_m": float(base_row[f"{side_name}_corridor"]),
            "rule_side_corridor_m": float(side_row[f"{side_name}_corridor"]),
            "rule_total_corridor_m": float(total_row[f"{side_name}_corridor"]),
            "N_before": result["possible_hypothesis_count_before"],
            "N_after_F_SIDE_GAP": result["possible_hypothesis_count_after_side"],
            "N_after_F_TOTAL_OPEN": result["possible_hypothesis_count_after_total"],
            "possible_area_before_m2": result["possible_area_before_m2"],
            "possible_area_after_side_m2": result["possible_area_after_side_m2"],
            "possible_area_after_total_m2": result["possible_area_after_total_m2"],
            "selected_side_support_reduction_mm": support_reduction,
            "corridor_recovery_side_mm": (float(side_row[f"{side_name}_corridor"]) - float(base_row[f"{side_name}_corridor"])) * 1000.0,
            "corridor_recovery_total_mm": (float(total_row[f"{side_name}_corridor"]) - float(base_row[f"{side_name}_corridor"])) * 1000.0,
            "blocking_hypothesis_removed": support_reduction > 0.0,
            "result": "RESOLVED" if float(side_row[f"{side_name}_corridor"]) > 0.0 else "STILL_FALSE_INFEASIBLE",
        }
        false_rows.append(row)
        extrema_rows.append({"case_id": case_id, **(result["dominant_removed_hypothesis"] or {})})

    for case in cases:
        result = results[case["case_id"]]
        side_name = "right" if case["case_id"] == "validation_039" else s3.passing_side(normal_lookup[case["case_id"]])
        base_row = result["cpp"]["TF_BASE"]
        for interpretation, key, count_key, area_key, band_key in (
            ("F_SIDE_GAP", "TF_RULE_SIDE", "possible_hypothesis_count_after_side", "possible_area_after_side_m2", "hypotheses_in_side_numerical_band"),
            ("F_TOTAL_OPEN", "TF_RULE_TOTAL", "possible_hypothesis_count_after_total", "possible_area_after_total_m2", "hypotheses_in_total_numerical_band"),
        ):
            row = result["cpp"][key]
            interpretation_rows.append(
                {
                    "case_id": case["case_id"],
                    "case_scope": "validation039_safety_anchor" if case["case_id"] == "validation_039" else "normal25",
                    "passing_side": side_name,
                    "interpretation": interpretation,
                    "hypotheses_before": result["possible_hypothesis_count_before"],
                    "hypotheses_remaining": result[count_key],
                    "possible_area_m2": result[area_key],
                    "base_corridor_m": float(base_row[f"{side_name}_corridor"]),
                    "corridor_m": float(row[f"{side_name}_corridor"]),
                    "corridor_delta_vs_base_mm": (float(row[f"{side_name}_corridor"]) - float(base_row[f"{side_name}_corridor"])) * 1000.0,
                    "numerical_band_hypotheses": result[band_key],
                }
            )

    contribution_rows: list[dict[str, Any]] = []
    for case_id in PRIMARY_CASES:
        result = results[case_id]
        side_name = s3.passing_side(normal_lookup[case_id])
        contribution = dict(result["contribution"])
        for previous, current, name in (
            ("TF_DOMAIN", "TF_SIZE_PRIOR", "rulebook_size_prior"),
            ("TF_SIZE_PRIOR", "TF_BASE", "sensor_free_rays"),
            ("TF_BASE", "TF_RULE_SIDE", "free_gap_prior"),
        ):
            previous_row, current_row = result["cpp"][previous], result["cpp"][current]
            previous_support = float(previous_row["d_max"] if side_name == "left" else previous_row["d_min"])
            current_support = float(current_row["d_max"] if side_name == "left" else current_row["d_min"])
            support_reduction = previous_support - current_support if side_name == "left" else current_support - previous_support
            contribution_rows.append(
                {
                    "case_id": case_id,
                    "component": name,
                    "selected_side": side_name,
                    "selected_side_support_reduction_mm": support_reduction * 1000.0,
                    "corridor_recovery_mm": (float(current_row[f"{side_name}_corridor"]) - float(previous_row[f"{side_name}_corridor"])) * 1000.0,
                    **contribution,
                }
            )

    old_rows = old_scenario_rows(track)
    production_after = {name: sha256(path) for name, path in rulebook.PRODUCTION_FILES.items()}
    if production_before != production_after:
        raise RuntimeError("production source/binary hashes changed during diagnostic audit")

    cpu_self_after = resource.getrusage(resource.RUSAGE_SELF)
    cpu_children_after = resource.getrusage(resource.RUSAGE_CHILDREN)
    memory_after = memory_snapshot()
    cpu_time = (
        cpu_self_after.ru_utime - cpu_self_before.ru_utime
        + cpu_self_after.ru_stime - cpu_self_before.ru_stime
        + cpu_children_after.ru_utime - cpu_children_before.ru_utime
        + cpu_children_after.ru_stime - cpu_children_before.ru_stime
    )
    max_rss = max(cpu_self_after.ru_maxrss, cpu_children_after.ru_maxrss) * 1024
    elapsed = time.monotonic() - started
    rule_gap_hypotheses = sum(result["possible_hypothesis_count_before"] for result in results.values())
    rule_gap_cpu_s = sum(result["timing"]["rule_gap_cpu_s"] for result in results.values())
    per_hypothesis_s = rule_gap_cpu_s / rule_gap_hypotheses
    memory_delta_per_track = max(
        result["possible_cell_count_before"] * 2 for result in results.values()
    )
    aggregate_worker_reference_s = sum(
        result["timing"]["worker_wall_s"] for result in results.values()
    ) + sum(result["timing"]["worker_wall_s"] for result in repeats)
    performance = {
        "worker_count": WORKERS,
        "worker_policy": "workers=4 for independent cases; case-internal geometry deterministic serial",
        "offline_wall_time_s": elapsed + args.recovered_attempt_wall_time_s,
        "completion_invocation_wall_time_s": elapsed,
        "recovered_interrupted_attempt_wall_time_s": args.recovered_attempt_wall_time_s,
        "parallel_case_wall_time_s": parallel_wall,
        "cpu_time_s": aggregate_worker_reference_s,
        "cpu_time_measurement_basis": "sum of serial per-case worker wall measurements, including child diagnostic queries; recovered from case artifacts",
        "completion_invocation_rusage_cpu_time_s": cpu_time,
        "diagnostic_build_time_s": args.diagnostic_build_time_s,
        "diagnostic_build_max_rss_bytes": args.diagnostic_build_max_rss_bytes,
        "diagnostic_build_reused": True,
        "max_rss_bytes": max_rss,
        "swap_used_before_bytes": memory_before["swap_used_bytes"],
        "swap_used_after_bytes": memory_after["swap_used_bytes"],
        "swap_change_bytes": memory_after["swap_used_bytes"] - memory_before["swap_used_bytes"],
        "memory_available_before_bytes": memory_before["memory_available_bytes"],
        "memory_available_after_parallel_bytes": memory_after_parallel["memory_available_bytes"],
        "memory_available_after_bytes": memory_after["memory_available_bytes"],
        "cache_reuse": [str(RULEBOOK_PRIOR), str(PRIOR), str(TEMPORAL), str(REPRESENTATION)],
        "recovered_primary_case_cache_count": len(cached_results),
        "computed_primary_case_count_this_invocation": len(computed_results),
        "recovered_repeat_case_cache_count": len(repeat_cases) - len(pending_repeats),
        "bag_parse_count": 1,
        "bag_parse_time_s": bag_parse_time,
        "closed_loop_replay_count": 0,
        "dataset_regeneration_count": 0,
        "cma_run_count": 0,
        "determinism_repeat_time_s": max(
            repeat_time,
            max(result["timing"]["worker_wall_s"] for result in repeats),
        ),
        "full_cell_corridor_evaluation_count": 0,
        "support_projection_method": "4-neighbour morphological boundary only",
    }
    production_cost = {
        "measured_offline_reference": True,
        "incremental_rule_gap_cpu_s_per_discrete_hypothesis": per_hypothesis_s,
        "incremental_rule_gap_cpu_us_per_discrete_hypothesis": per_hypothesis_s * 1.0e6,
        "mean_incremental_rule_filter_wall_s_per_track_update": float(np.mean([result["timing"]["rule_filter_s"] for result in results.values()])),
        "max_incremental_rule_filter_wall_s_per_track_update": max(result["timing"]["rule_filter_s"] for result in results.values()),
        "estimated_three_obstacle_serial_rule_filter_wall_s": 3.0 * max(result["timing"]["rule_filter_s"] for result in results.values()),
        "estimated_boolean_mask_memory_delta_bytes_per_track": memory_delta_per_track,
        "estimated_three_obstacle_boolean_mask_memory_delta_bytes": 3 * memory_delta_per_track,
        "production_ready_as_measured": False,
        "note": "offline vectorized diagnostic cost only; no production optimization or behavior was implemented",
    }

    best = next(row for row in strategy_rows if row["strategy"] == "S3_RULE_SIDE")
    side_vs_total_material = any(
        abs(float(a["corridor_m"]) - float(b["corridor_m"])) > 0.001
        for a, b in zip(interpretation_rows[0::2], interpretation_rows[1::2])
    )
    free_gap_material = any(
        row["N_after_F_SIDE_GAP"] < row["N_before"]
        or abs(float(row["corridor_recovery_side_mm"])) > 1.0
        for row in false_rows
    )
    if best["classification"] == "UNSAFE":
        classification = "UNSAFE"
    elif not free_gap_material:
        classification = "INSUFFICIENT"
    elif side_vs_total_material:
        classification = "RULE_INTERPRETATION_DEPENDENT"
    elif best["false_infeasible_count"] <= 1:
        classification = "PROMISING_SHADOW"
    elif best["false_infeasible_count"] > 1:
        classification = "OVER_CONSERVATIVE"
    else:
        classification = "INSUFFICIENT"
    best["classification"] = classification

    write_csv(OUTPUT / "false_infeasible_rule_pruning.csv", false_rows)
    write_csv(OUTPUT / "hypothesis_extrema.csv", extrema_rows)
    write_csv(OUTPUT / "validation039_safety_anchor.csv", anchor_rows)
    write_csv(OUTPUT / "normal25_rule_comparison.csv", comparison_rows)
    write_csv(OUTPUT / "rule_interpretation_comparison.csv", interpretation_rows)
    write_csv(OUTPUT / "old_scenario_rule_compliance.csv", old_rows)
    write_csv(OUTPUT / ".raw/normal25_rule_compliance.csv", compliance_rows)
    write_csv(OUTPUT / ".raw/rule_prior_sensor_contribution.csv", contribution_rows)
    (OUTPUT / "performance.json").write_text(json.dumps(performance, indent=2, sort_keys=True) + "\n")

    summary = {
        "schema": "rulebook_free_gap_pruning_audit/1",
        "diagnostic_only": True,
        "production_changes": False,
        "production_hashes_before": production_before,
        "production_hashes_after": production_after,
        "rule_interpretations": {
            "primary": "F_SIDE_GAP: one complete physical obstacle-to-boundary passage has minimum gap >=0.5 m over the full obstacle span",
            "alternative": "F_TOTAL_OPEN: total unoccupied cross-section has minimum width >=0.5 m without an individual-side requirement",
            "vehicle_footprint_or_planner_margin_in_rule_filter": False,
            "threshold_m": RULE_GAP_M,
            "numerical_sensitivity_band_m": NUMERICAL_BAND_M,
        },
        "normal25_compliance": {
            "fully_F_SIDE_compliant": all(row["primary_F_SIDE_benchmark_included"] for row in compliance_rows),
            "included_count": sum(row["primary_F_SIDE_benchmark_included"] for row in compliance_rows),
            "excluded_case_ids": [row["case_id"] for row in compliance_rows if not row["primary_F_SIDE_benchmark_included"]],
        },
        "primary_false_infeasible_cases": false_rows,
        "strategy_summary": strategy_rows,
        "validation039_safety_anchor": anchor_rows,
        "old_scenario_rule_compliance": old_rows,
        "rule_sensor_contribution": contribution_rows,
        "classification": classification,
        "desired_target_met": best["false_feasible_count"] == 0 and best["false_infeasible_count"] <= 1,
        "F_SIDE_vs_F_TOTAL_materially_different": side_vs_total_material,
        "evidence_strong_enough_for_production_shadow_mode": classification == "PROMISING_SHADOW",
        "static_dynamic_pipeline_separation": {
            "RAW_TENTATIVE": "history collection only",
            "CONFIRMED_UNKNOWN": "rulebook-constrained static safety envelope may be queried in shadow",
            "STATIC": "map-frame envelope allowed in shadow",
            "DYNAMIC": "static/rulebook envelope invalidated immediately",
            "opponent_pipeline_modified": False,
        },
        "future_three_obstacle_generator_note": {
            "dimensions": "strictly <0.5 m on both rectangle axes",
            "free_gap": "validate F_SIDE_GAP >=0.5 m over each full obstacle span",
            "pairwise_distance": ">=1 m physical polygon distance for every pair",
            "start_exclusion": ">=1 m physical polygon distance from the start exclusion region",
            "dataset_generated": False,
        },
        "production_cost_estimate": production_cost,
        "determinism": {"bit_identical": True, "cases": determinism},
        "performance": performance,
        "cma_executed": False,
        "closed_loop_replay_count": 0,
        "dataset_regenerated": False,
        "provenance": {
            "renderer": str(Path(__file__)),
            "renderer_sha256": sha256(Path(__file__)),
            "existing_diagnostic_binary": str(BINARY),
            "existing_diagnostic_binary_sha256": sha256(BINARY),
            "source_stamp_ns": EXACT_STAMP,
            "path_geometry_hash": EXPECTED_HASH,
            "serialized_path_sha256": EXPECTED_PATH_SHA256,
            "prior_artifacts": [str(RULEBOOK_PRIOR), str(PRIOR), str(TEMPORAL), str(REPRESENTATION)],
        },
    }
    (OUTPUT / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")

    dominant = {
        row["case_id"]: next(item for item in extrema_rows if item["case_id"] == row["case_id"])
        for row in false_rows
    }
    table1 = "\n".join(
        f"| {row['case_id']} | {row['base_corridor_m']:.6f} | {row['rule_side_corridor_m']:.6f} | {row['N_before'] - row['N_after_F_SIDE_GAP']} | "
        f"{dominant[row['case_id']].get('support_role', 'none')}: "
        f"{dominant[row['case_id']].get('width_u_m', math.nan):.3f}x{dominant[row['case_id']].get('length_v_m', math.nan):.3f} m @ {dominant[row['case_id']].get('orientation_deg', math.nan):.1f} deg | {row['result']} |"
        for row in false_rows
    )
    table2 = "\n".join(
        f"| {row['strategy']} | {str(row['validation039_safety_retained']).lower()} | {row['false_feasible_count']}/{row['case_count']} | {row['false_infeasible_count']}/{row['case_count']} | {row['classification']} |"
        for row in strategy_rows
    )
    readme = f"""# Rulebook free-gap set-membership pruning audit

이 디렉터리는 완전한 offline diagnostic/shadow 산출물이다. production perception,
planner, validator, margin, tracking, controller, lifecycle, YAML과 generator는 변경하지 않았다.
closed-loop replay, CMA, dataset regeneration은 모두 0회다.

## 결론

Primary rule은 vehicle 폭이나 planner margin을 섞지 않은 `F_SIDE_GAP`이다. 4 mm / 2 deg
S3 possible-cell witness마다 물리 obstacle rectangle과 track boundary 사이의 한쪽 통로가
전체 longitudinal span에서 0.5 m 이상인지 검사했다. 곡선 구간은 centroid 한 점이 아니라
piecewise-linear track segment endpoint와 rectangle-corner breakpoint를 전부 평가했다.

최종 분류는 `{classification}`이다. `S3_RULE_SIDE`의 rule-compliant normal subset
false-feasible은 {best['false_feasible_count']}/{best['case_count']}, false-infeasible은
{best['false_infeasible_count']}/{best['case_count']}이고, validation_039 동일 dangerous path의
hard rejection 유지 여부는 {str(anchor_safe['S3_RULE_SIDE']).lower()}다. Production
shadow-mode 진행 근거는 {str(summary['evidence_strong_enough_for_production_shadow_mode']).lower()}다.
F_SIDE와 F_TOTAL의 전체 benchmark 차이는
{str(summary['F_SIDE_vs_F_TOTAL_materially_different']).lower()}이며, 이 차이는 남은 두
false-infeasible을 복구하는 기여가 아니라 별도 normal case의 overcoverage 변화다.

## Table 1 - remaining false-infeasible root cause

| case | base corridor m | rule-side corridor m | hypotheses removed | dominant removed hypothesis | result |
|---|---:|---:|---:|---|---|
{table1}

## Table 2 - strategy comparison

| strategy | 039 safe | normal false-feasible | normal false-infeasible | classification |
|---|---:|---:|---:|---|
{table2}

## 해석 경계

- `F_SIDE_GAP`은 한쪽 complete passage의 minimum raw geometric gap이다.
- `F_TOTAL_OPEN`은 양쪽 open width 합만 본 ambiguous alternative이며 production 권고로
  사용하지 않았다. 두 해석의 차이는 `rule_interpretation_comparison.csv`에 기록했다.
- hypothesis count는 기존 S3 격자의 `(orientation, possible-cell)` sensor-consistent minimal
  rectangle witness 수다. possible area는 중복을 제거한 possible-cell union이다.
- 정확히 0.5 m 부근의 판정은 rule threshold를 바꾸지 않았다. 현재 4 mm cell의 대각선과
  기존 0.1 mm geometry epsilon을 합친 {NUMERICAL_BAND_M * 1000.0:.3f} mm band의 hypothesis를
  별도 집계했다.
- `normal25_rule_comparison.csv`의 primary 통계는 exact full-span F_SIDE와 strict `<0.5 m`
  dimension을 모두 만족한 case만 포함한다. 제외 case는 `.raw/normal25_rule_compliance.csv`
  ambiguity/stress 표에 원 geometry 그대로 남겼다.
- 019/034는 재실행하지 않았고 exact physical placement의 rule compliance만
  `old_scenario_rule_compliance.csv`에서 판정했다.

## Rule prior와 sensor evidence

`.raw/rule_prior_sensor_contribution.csv`는 normal_01_00/01_02에 대해 동일 bounded evaluation
domain에서 size prior, negative free rays, free-gap prior의 area/support/corridor 기여를 순차
분리한다. Bounded domain은 무한한 no-size hypothesis를 유한하게 비교하기 위한 진단
normalization일 뿐 production prior가 아니다.

## Pipeline 및 향후 generator

RAW/TENTATIVE는 history만 수집하고, CONFIRMED UNKNOWN/STATIC에만 static rulebook shadow를
조회할 수 있다. DYNAMIC 전환은 이를 즉시 무효화해야 한다. Frenet/map-frame KF,
association, track ID, motion voting과 `/opp_obs`는 이 audit에서 전혀 변경하지 않았다.

향후 3-obstacle finals generator는 각 축 `<0.5 m`, 각 obstacle의 full-span
`F_SIDE_GAP>=0.5 m`, physical polygon pairwise distance `>=1 m`, start exclusion `>=1 m`를
동시에 검증해야 한다. 이번 작업에서는 dataset을 만들지 않았다.

## 자원 및 재현성

workers={WORKERS}, offline wall={performance['offline_wall_time_s']:.3f} s,
aggregate worker CPU reference={performance['cpu_time_s']:.3f} s, max RSS={max_rss} bytes,
swap delta={performance['swap_change_bytes']} bytes다. 기존 diagnostic binary와 parsed/stored
geometry를 재사용했고 build={args.diagnostic_build_time_s:.3f} s, replay=0이다.
validation_039와 normal_01_00 반복의 count/support/corridor/clearance/classification/digest는
모두 bit-identical이다.
"""
    (OUTPUT / "README.md").write_text(readme, encoding="utf-8")
    print(json.dumps({"output": str(OUTPUT), "classification": classification, "performance": performance}, indent=2))
    return 0


def lambda_pair_analyze(pair: tuple[dict[str, Any], TrackGeometry]) -> dict[str, Any]:
    case, track = pair
    return analyze_case(case, track, True)


if __name__ == "__main__":
    raise SystemExit(main())
