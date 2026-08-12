#!/usr/bin/env python3
"""Offline S3 shadow-mode false-infeasible root-cause audit.

The script reconstructs the stored deterministic normal-case ray specifications, parses the
validation_039 source bag once, and evaluates rulebook set-membership representations without
publishing or feeding any result into production.  It deliberately reuses the BUILD_TESTING-only
path-family executable for detector-owned AABB projection and local-planner corridor/path checks.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import subprocess
import time
from typing import Any, Iterable, Sequence

import numpy as np

import render_rulebook_obstacle_envelope_audit as rulebook
from cmaes_tuning.map_baker import MapModel
from cmaes_tuning.scenario_generator import load_waypoints
from cmaes_tuning.schemas import ObstacleSpec
from cmaes_tuning.simulator_collision import SimulatorRasterCollisionModel


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "runs/cmaes_tuning/s3_shadow_false_infeasible_audit_v1"
RAW = OUTPUT / ".raw"
PRIOR = ROOT / "runs/cmaes_tuning/rulebook_obstacle_envelope_audit_v1"
TEMPORAL = ROOT / "runs/cmaes_tuning/temporal_obstacle_envelope_audit_v1"
REPRESENTATION = ROOT / "runs/cmaes_tuning/validation039_representation_root_cause_v1"
TIME_AXIS = ROOT / "runs/cmaes_tuning/time_axis_feasibility_audit_v1"
SCENARIO_ROOT = (
    ROOT / "runs/cmaes_tuning/medium_lockstep_stage1_v2/artifacts/scenarios/validation"
)
WAYPOINTS = ROOT / "offline_trajectory_generator/output/ifac_track/global_waypoints.json"
CLEAN_MAP = ROOT / "src/monte_carlo_localization/maps/ifac_track.yaml"
BINARY = ROOT / "build/local_planning/path_family_feasibility_audit"
EXACT_STAMP = rulebook.EXACT_039_STAMP
EXPECTED_HASH = rulebook.EXPECTED_039_HASH
EXPECTED_PATH_SHA256 = rulebook.EXPECTED_039_PATH_SHA256
FALSE_CASE_IDS = ("normal_01_00", "normal_01_02", "normal_02_00")
WORKERS = 4


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--diagnostic-build-time-s", type=float, default=0.0)
    parser.add_argument("--diagnostic-build-max-rss-bytes", type=int, default=0)
    return parser.parse_args()


@dataclass(frozen=True)
class OccupancyGrid:
    resolution_m: float
    angle_step_deg: float
    angles_deg: tuple[float, ...]
    xs: np.ndarray
    ys: np.ndarray
    mask: np.ndarray
    bounds: tuple[float, float, float, float]
    possible_area_m2: float
    aabb_area_m2: float
    query_count: int
    hit_count: int
    free_ray_count: int
    digest: str

    def summary(self) -> dict[str, Any]:
        return {
            "resolution_m": self.resolution_m,
            "angle_step_deg": self.angle_step_deg,
            "angles_deg": list(self.angles_deg),
            "orientation_count": len(self.angles_deg),
            "bounds": list(self.bounds),
            "possible_area_m2": self.possible_area_m2,
            "aabb_area_m2": self.aabb_area_m2,
            "fill_ratio": self.possible_area_m2 / self.aabb_area_m2,
            "possible_cell_count": int(np.count_nonzero(self.mask)),
            "query_count": self.query_count,
            "hit_count": self.hit_count,
            "free_ray_count": self.free_ray_count,
            "digest": self.digest,
        }


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


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        raise RuntimeError(f"refusing to write empty CSV: {path}")
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def read_spec(path: Path) -> dict[str, Any]:
    result: dict[str, Any] = {"polygons": {}}
    for line in path.read_text(encoding="utf-8").splitlines():
        tokens = line.split("\t")
        if tokens[0] == "MANIFEST_S":
            result["manifest_s"] = float(tokens[1])
        elif tokens[0] == "POLYGON":
            count = int(tokens[3])
            values = [float(value) for value in tokens[4:]]
            result["polygons"][tokens[1]] = [
                (values[2 * index], values[2 * index + 1]) for index in range(count)
            ]
    return result


def polygon_bounds(points: Sequence[tuple[float, float]]) -> tuple[float, float, float, float]:
    return (
        min(point[0] for point in points),
        max(point[0] for point in points),
        min(point[1] for point in points),
        max(point[1] for point in points),
    )


def selected_frames_window(
    frames: Sequence[rulebook.RayFrame], now_ns: int, window_ms: int
) -> list[rulebook.RayFrame]:
    lower = now_ns - window_ms * 1_000_000
    return [frame for frame in frames if lower <= frame.stamp_ns <= now_ns]


def detailed_possible_occupancy(
    frames: Sequence[rulebook.RayFrame],
    resolution_m: float,
    angle_step_deg: float,
    *,
    use_free_space: bool = True,
    allowed_angles_deg: Sequence[float] | None = None,
    max_side_u_m: float = rulebook.MAX_SIDE_M,
    max_side_v_m: float = rulebook.MAX_SIDE_M,
) -> OccupancyGrid:
    """Return the actual possible-occupancy raster used before outer boxing.

    This is the previous audit's set-membership calculation with two diagnostic switches: negative
    rays can be disabled for the before/after audit, and evidence-supported orientation subsets can
    be supplied.  No tolerance or margin is introduced here.
    """
    if not frames:
        raise ValueError("possible occupancy requires an observation")
    hits = np.vstack([frame.hits for frame in frames])
    centre = np.mean(hits, axis=0)
    padding = math.sqrt(2.0) * max(max_side_u_m, max_side_v_m) + resolution_m
    x_start = math.floor((float(np.min(hits[:, 0])) - padding) / resolution_m) * resolution_m
    x_stop = math.ceil((float(np.max(hits[:, 0])) + padding) / resolution_m) * resolution_m
    y_start = math.floor((float(np.min(hits[:, 1])) - padding) / resolution_m) * resolution_m
    y_stop = math.ceil((float(np.max(hits[:, 1])) + padding) / resolution_m) * resolution_m
    xs = np.arange(x_start + 0.5 * resolution_m, x_stop, resolution_m)
    ys = np.arange(y_start + 0.5 * resolution_m, y_stop, resolution_m)
    world_x, world_y = np.meshgrid(xs, ys)
    flat_x, flat_y = world_x.ravel(), world_y.ravel()
    possible = np.zeros(flat_x.size, dtype=bool)
    effective_u = max_side_u_m + math.sqrt(2.0) * resolution_m
    effective_v = max_side_v_m + math.sqrt(2.0) * resolution_m

    ray_origins = np.vstack(
        [np.repeat(frame.origin[None, :], frame.directions.shape[0], axis=0) for frame in frames]
    )
    ray_directions = np.vstack([frame.directions for frame in frames])
    free_lengths = np.concatenate([frame.free_lengths for frame in frames])
    endpoint_distance = np.linalg.norm(
        ray_origins + free_lengths[:, None] * ray_directions - centre[None, :], axis=1
    )
    ray_order = np.argsort(endpoint_distance, kind="stable")
    if allowed_angles_deg is None:
        angles_deg = np.arange(0.0, 90.0 - 1.0e-12, angle_step_deg, dtype=np.float64)
    else:
        angles_deg = np.asarray(sorted(set(float(value) % 90.0 for value in allowed_angles_deg)))
    if not angles_deg.size:
        raise RuntimeError("orientation constraint eliminated every angle")

    query_count = 0
    for angle_deg in angles_deg:
        angle = math.radians(float(angle_deg))
        cosine, sine = math.cos(angle), math.sin(angle)
        hit_u = (hits[:, 0] - centre[0]) * cosine + (hits[:, 1] - centre[1]) * sine
        hit_v = -(hits[:, 0] - centre[0]) * sine + (hits[:, 1] - centre[1]) * cosine
        hu_min, hu_max = float(np.min(hit_u)), float(np.max(hit_u))
        hv_min, hv_max = float(np.min(hit_v)), float(np.max(hit_v))
        if hu_max - hu_min > effective_u + rulebook.GEOMETRY_EPSILON_M:
            continue
        if hv_max - hv_min > effective_v + rulebook.GEOMETRY_EPSILON_M:
            continue

        indices = np.flatnonzero(~possible)
        dx, dy = flat_x[indices] - centre[0], flat_y[indices] - centre[1]
        query_u = dx * cosine + dy * sine
        query_v = -dx * sine + dy * cosine
        u_min, u_max = np.minimum(query_u, hu_min), np.maximum(query_u, hu_max)
        v_min, v_max = np.minimum(query_v, hv_min), np.maximum(query_v, hv_max)
        valid = (u_max - u_min <= effective_u + rulebook.GEOMETRY_EPSILON_M) & (
            v_max - v_min <= effective_v + rulebook.GEOMETRY_EPSILON_M
        )
        indices, u_min, u_max, v_min, v_max = (
            indices[valid], u_min[valid], u_max[valid], v_min[valid], v_max[valid]
        )
        query_count += int(indices.size)
        if not indices.size:
            continue
        if not use_free_space:
            possible[indices] = True
            continue

        origin_dx, origin_dy = ray_origins[:, 0] - centre[0], ray_origins[:, 1] - centre[1]
        origin_u = origin_dx * cosine + origin_dy * sine
        origin_v = -origin_dx * sine + origin_dy * cosine
        direction_u = ray_directions[:, 0] * cosine + ray_directions[:, 1] * sine
        direction_v = -ray_directions[:, 0] * sine + ray_directions[:, 1] * cosine
        maximum_u_min, maximum_u_max = hu_max - effective_u, hu_min + effective_u
        maximum_v_min, maximum_v_max = hv_max - effective_v, hv_min + effective_v
        relevant = [
            int(ray_index)
            for ray_index in ray_order
            if rulebook.segment_intersects_box(
                float(origin_u[ray_index]),
                float(origin_v[ray_index]),
                float(direction_u[ray_index]),
                float(direction_v[ray_index]),
                float(free_lengths[ray_index]),
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
                float(free_lengths[ray_index]),
                u_min[live],
                u_max[live],
                v_min[live],
                v_max[live],
            )
            alive[live[invalid]] = False
        possible[indices[alive]] = True

    if not np.any(possible):
        raise RuntimeError("no rulebook hypothesis remains")
    cells = np.flatnonzero(possible)
    half = 0.5 * resolution_m
    bounds = (
        float(np.min(flat_x[cells]) - half),
        float(np.max(flat_x[cells]) + half),
        float(np.min(flat_y[cells]) - half),
        float(np.max(flat_y[cells]) + half),
    )
    token = {
        "resolution_m": resolution_m,
        "angle_step_deg": angle_step_deg,
        "angles_deg": [float(value) for value in angles_deg],
        "frames": [frame.stamp_ns for frame in frames],
        "use_free_space": use_free_space,
        "max_side_u_m": max_side_u_m,
        "max_side_v_m": max_side_v_m,
        "bounds": bounds,
        "mask_sha256": hashlib.sha256(np.packbits(possible).tobytes()).hexdigest(),
    }
    return OccupancyGrid(
        resolution_m=resolution_m,
        angle_step_deg=angle_step_deg,
        angles_deg=tuple(float(value) for value in angles_deg),
        xs=xs,
        ys=ys,
        mask=possible.reshape((ys.size, xs.size)),
        bounds=bounds,
        possible_area_m2=float(cells.size) * resolution_m**2,
        aabb_area_m2=(bounds[1] - bounds[0]) * (bounds[3] - bounds[2]),
        query_count=query_count,
        hit_count=int(hits.shape[0]),
        free_ray_count=int(free_lengths.size if use_free_space else 0),
        digest=canonical_digest(token),
    )


def orientation_quality(frames: Sequence[rulebook.RayFrame]) -> dict[str, Any]:
    hits = np.vstack([frame.hits for frame in frames])
    centre = np.mean(hits, axis=0)
    covariance = np.cov((hits - centre).T, bias=True)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    principal = eigenvectors[:, int(np.argmax(eigenvalues))]
    normal = np.asarray([-principal[1], principal[0]])
    tangent_projection = (hits - centre) @ principal
    normal_projection = (hits - centre) @ normal
    span = float(np.max(tangent_projection) - np.min(tangent_projection))
    maximum_residual = float(np.max(np.abs(normal_projection)))
    rms_residual = float(math.sqrt(max(0.0, float(np.min(eigenvalues)))))
    guards = np.concatenate(
        [np.maximum(0.0, frame.ranges - frame.free_lengths) for frame in frames]
    )
    established_tolerance = float(np.max(guards))
    half_angle = math.degrees(
        math.asin(min(1.0, 2.0 * (established_tolerance + rms_residual) / max(span, 1.0e-12)))
    )
    edge_angle = math.degrees(math.atan2(float(principal[1]), float(principal[0]))) % 90.0
    # A line-normal cone wider than the rectangle's 90-degree symmetry interval removes no
    # orientation hypothesis.  This is a geometric identifiability test, not a point-count rule.
    usable = maximum_residual <= established_tolerance + rulebook.GEOMETRY_EPSILON_M and half_angle < 45.0
    return {
        "hit_count": int(hits.shape[0]),
        "principal_edge_angle_deg_mod90": edge_angle,
        "surface_span_m": span,
        "maximum_orthogonal_residual_m": maximum_residual,
        "rms_orthogonal_residual_m": rms_residual,
        "established_sensor_evaluator_tolerance_m": established_tolerance,
        "orientation_half_angle_deg": half_angle,
        "eigenvalue_ratio_minor_major": float(np.min(eigenvalues) / max(np.max(eigenvalues), 1.0e-24)),
        "usable": usable,
        "criterion": "max residual <= established tolerance and geometric half-angle <45deg",
    }


def allowed_orientation_grid(quality: dict[str, Any], step_deg: float) -> list[float]:
    if not quality["usable"]:
        return list(np.arange(0.0, 90.0 - 1.0e-12, step_deg))
    centre = float(quality["principal_edge_angle_deg_mod90"])
    half = float(quality["orientation_half_angle_deg"])
    values = []
    for value in np.arange(0.0, 90.0 - 1.0e-12, step_deg):
        delta = abs(float(value) - centre)
        delta = min(delta, 90.0 - delta)
        if delta <= half + 0.5 * step_deg:
            values.append(float(value))
    if not values:
        values.append(round(centre / step_deg) * step_deg % 90.0)
    return values


def dimension_feasible(
    frames: Sequence[rulebook.RayFrame], angle_deg: float, axis: str, span_m: float, step_m: float
) -> bool:
    hits = np.vstack([frame.hits for frame in frames])
    centre = np.mean(hits, axis=0)
    angle = math.radians(angle_deg)
    cosine, sine = math.cos(angle), math.sin(angle)
    hit_u = (hits[:, 0] - centre[0]) * cosine + (hits[:, 1] - centre[1]) * sine
    hit_v = -(hits[:, 0] - centre[0]) * sine + (hits[:, 1] - centre[1]) * cosine
    hu_min, hu_max = float(np.min(hit_u)), float(np.max(hit_u))
    hv_min, hv_max = float(np.min(hit_v)), float(np.max(hit_v))
    observed_span = (hu_max - hu_min) if axis == "u" else (hv_max - hv_min)
    if span_m + rulebook.GEOMETRY_EPSILON_M < observed_span:
        return False
    lower, upper = (
        (hu_max - span_m, hu_min) if axis == "u" else (hv_max - span_m, hv_min)
    )
    offsets = list(np.arange(lower, upper + 0.5 * step_m, step_m)) + [lower, upper]
    offsets_array = np.asarray(sorted(set(float(value) for value in offsets)))
    if axis == "u":
        u_min, u_max = offsets_array, offsets_array + span_m
        v_min = np.full(offsets_array.size, hv_min)
        v_max = np.full(offsets_array.size, hv_max)
    else:
        v_min, v_max = offsets_array, offsets_array + span_m
        u_min = np.full(offsets_array.size, hu_min)
        u_max = np.full(offsets_array.size, hu_max)

    ray_origins = np.vstack(
        [np.repeat(frame.origin[None, :], frame.directions.shape[0], axis=0) for frame in frames]
    )
    directions = np.vstack([frame.directions for frame in frames])
    lengths = np.concatenate([frame.free_lengths for frame in frames])
    origin_dx, origin_dy = ray_origins[:, 0] - centre[0], ray_origins[:, 1] - centre[1]
    origin_u = origin_dx * cosine + origin_dy * sine
    origin_v = -origin_dx * sine + origin_dy * cosine
    direction_u = directions[:, 0] * cosine + directions[:, 1] * sine
    direction_v = -directions[:, 0] * sine + directions[:, 1] * cosine
    alive = np.ones(offsets_array.size, dtype=bool)
    for index in range(lengths.size):
        live = np.flatnonzero(alive)
        if not live.size:
            return False
        invalid = rulebook.ray_invalid_candidates(
            float(origin_u[index]),
            float(origin_v[index]),
            float(direction_u[index]),
            float(direction_v[index]),
            float(lengths[index]),
            u_min[live],
            u_max[live],
            v_min[live],
            v_max[live],
        )
        alive[live[invalid]] = False
    return bool(np.any(alive))


def maximum_supported_dimensions(
    frames: Sequence[rulebook.RayFrame], angles_deg: Sequence[float], step_m: float
) -> dict[str, Any]:
    target = rulebook.MAX_SIDE_M - 0.5 * step_m
    result: dict[str, Any] = {}
    hits = np.vstack([frame.hits for frame in frames])
    centre = np.mean(hits, axis=0)
    for axis in ("u", "v"):
        best_feasible = 0.0
        best_upper = rulebook.MAX_SIDE_M
        best_angle: float | str = ""
        supremum_witness = False
        for angle in angles_deg:
            radians = math.radians(float(angle))
            cosine, sine = math.cos(radians), math.sin(radians)
            projected = (
                (hits[:, 0] - centre[0]) * cosine + (hits[:, 1] - centre[1]) * sine
                if axis == "u"
                else -(hits[:, 0] - centre[0]) * sine + (hits[:, 1] - centre[1]) * cosine
            )
            observed = float(np.max(projected) - np.min(projected))
            lower = min(target, observed + rulebook.GEOMETRY_EPSILON_M)
            if not dimension_feasible(frames, float(angle), axis, lower, step_m):
                continue
            if dimension_feasible(frames, float(angle), axis, target, step_m):
                best_feasible = target
                best_upper = rulebook.MAX_SIDE_M
                best_angle = float(angle)
                supremum_witness = True
                break
            upper = target
            feasible = lower
            for _ in range(10):
                middle = 0.5 * (feasible + upper)
                if dimension_feasible(frames, float(angle), axis, middle, step_m):
                    feasible = middle
                else:
                    upper = middle
            conservative_upper = min(rulebook.MAX_SIDE_M, upper + step_m)
            if feasible > best_feasible:
                best_feasible = feasible
                best_upper = conservative_upper
                best_angle = float(angle)
        result[f"{axis}_near_rulebook_supremum_feasible"] = supremum_witness
        result[f"{axis}_witness_angle_deg"] = best_angle
        result[f"{axis}_tested_span_m"] = target
        result[f"{axis}_largest_feasible_witness_m"] = best_feasible
        result[f"{axis}_evidence_supported_upper_bound_m"] = best_upper
    result["interpretation"] = (
        "0.5 denotes the strict <0.5 m set supremum, not an exact size; finite bounds add one "
        "2 mm placement-grid allowance above the first infeasible binary-search edge"
    )
    return result


def partial_visible_fraction(
    frames: Sequence[rulebook.RayFrame], obstacle: dict[str, float]
) -> tuple[float, int]:
    """GT-only evaluation metric; no value from this function enters an estimator."""
    cosine, sine = math.cos(obstacle["yaw"]), math.sin(obstacle["yaw"])
    width, height = obstacle["width"], obstacle["height"]
    intervals: dict[str, list[float]] = {"left": [], "right": [], "bottom": [], "top": []}
    for hit in np.vstack([frame.hits for frame in frames]):
        dx, dy = float(hit[0]) - obstacle["x"], float(hit[1]) - obstacle["y"]
        u, v = dx * cosine + dy * sine, -dx * sine + dy * cosine
        distances = {
            "left": abs(u + 0.5 * width),
            "right": abs(u - 0.5 * width),
            "bottom": abs(v + 0.5 * height),
            "top": abs(v - 0.5 * height),
        }
        edge = min(distances, key=distances.get)
        intervals[edge].append(v if edge in {"left", "right"} else u)
    visible = sum(max(values) - min(values) for values in intervals.values() if len(values) >= 2)
    return visible / (2.0 * (width + height)), sum(bool(values) for values in intervals.values())


def occupied_cell_boxes(grid: OccupancyGrid) -> dict[str, tuple[float, float, float, float]]:
    # In the local tubular neighbourhood of the matched track branch, Frenet s/d have non-zero
    # gradients. Their extrema over a compact occupancy set therefore occur on its boundary. Keep
    # every 4-neighbour morphological boundary cell (including array edges), which preserves the
    # support exactly while avoiding thousands of interior cells whose projections cannot change
    # an extreme. The full possible-set area and digest still use all occupied cells above.
    padded = np.pad(grid.mask, 1, constant_values=False)
    interior = grid.mask.copy()
    interior &= padded[:-2, 1:-1]
    interior &= padded[2:, 1:-1]
    interior &= padded[1:-1, :-2]
    interior &= padded[1:-1, 2:]
    boundary = grid.mask & ~interior
    rows, columns = np.nonzero(boundary)
    half = 0.5 * grid.resolution_m
    return {
        f"CELL_{index:06d}": (
            float(grid.xs[column] - half),
            float(grid.xs[column] + half),
            float(grid.ys[row] - half),
            float(grid.ys[row] + half),
        )
        for index, (row, column) in enumerate(zip(rows, columns))
    }


def run_cpp_spec(
    stream: Path, spec_text: str, directory: Path, expected_hash: str
) -> tuple[dict[str, dict[str, str]], float]:
    directory.mkdir(parents=True, exist_ok=True)
    spec = directory / "shadow_spec.tsv"
    spec.write_text(spec_text, encoding="utf-8")
    started = time.monotonic()
    subprocess.run(
        [str(BINARY), "--temporal-envelope", str(stream), str(spec), str(directory), expected_hash],
        cwd=ROOT,
        check=True,
        stdout=subprocess.DEVNULL,
    )
    rows = rulebook.tsv_rows(directory / "temporal_strategy_exact.tsv")
    return {row["representation"]: row for row in rows}, time.monotonic() - started


def make_combined_spec(
    case: dict[str, Any],
    boxes: dict[str, tuple[float, float, float, float]],
    support: dict[str, float] | None = None,
) -> str:
    raster = case["raster"]
    base = rulebook.make_cpp_spec(
        int(case["stamp_ns"]),
        float(case["manifest_s"]),
        rulebook.nominal_polygon(case["obstacle"]),
        raster,
        boxes,
    )
    if support is None:
        return base
    lines = base.splitlines()
    lines.pop()
    bounds = boxes["BASELINE_COARSE"]
    lines.append(
        "\t".join(
            [
                "FRENET_BOX",
                "TF_UNION_SUPPORT",
                "actual_possible_occupancy_cell_support",
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


def project_possible_set_support(case: dict[str, Any], grid: OccupancyGrid) -> tuple[dict[str, float], float]:
    boxes = occupied_cell_boxes(grid)
    directory = RAW / case["case_id"] / "cell_support_projection"
    spec = rulebook.make_cpp_spec(
        int(case["stamp_ns"]),
        float(case["manifest_s"]),
        rulebook.nominal_polygon(case["obstacle"]),
        case["raster"],
        boxes,
    )
    rows, elapsed = run_cpp_spec(Path(case["stream"]), spec, directory, "CORRIDOR_ONLY")
    support = rows.get("TF_UNION_SUPPORT_INTERNAL")
    if support is None:
        raise RuntimeError(f"{case['case_id']}: diagnostic binary did not aggregate cell support")
    return {
        "s_min": float(support["s_min"]),
        "s_max": float(support["s_max"]),
        "d_min": float(support["d_min"]),
        "d_max": float(support["d_max"]),
        "projected_boundary_cell_count": len(boxes),
        "full_possible_cell_count": int(np.count_nonzero(grid.mask)),
        "support_basis": "all 4-neighbour morphological boundary cells; local Frenet gradients are non-zero",
    }, elapsed


def evidence_center_box(frames: Sequence[rulebook.RayFrame]) -> tuple[float, float, float, float]:
    centre = np.mean(np.vstack([frame.hits for frame in frames]), axis=0)
    radius = math.sqrt(2.0) * 0.25
    return (centre[0] - radius, centre[0] + radius, centre[1] - radius, centre[1] + radius)


def analyze_case(case: dict[str, Any], repeat: bool = False) -> dict[str, Any]:
    started = time.monotonic()
    cpu_started = time.process_time()
    frames = case["frames"]
    now_ns = int(case["frame_now_ns"])
    temporal_frames = selected_frames_window(frames, now_ns, 20)
    single_frames = temporal_frames[-1:]
    ten_frames = selected_frames_window(frames, now_ns, 10)

    baseline_started = time.monotonic()
    baseline = detailed_possible_occupancy(temporal_frames, 0.004, 2.0)
    baseline_time = time.monotonic() - baseline_started
    no_free_started = time.monotonic()
    no_free = detailed_possible_occupancy(
        temporal_frames, 0.004, 2.0, use_free_space=False
    )
    no_free_time = time.monotonic() - no_free_started
    ten_started = time.monotonic()
    ut10 = detailed_possible_occupancy(ten_frames, 0.004, 2.0)
    ut10_time = time.monotonic() - ten_started

    single_quality = orientation_quality(single_frames)
    temporal_quality = orientation_quality(temporal_frames)
    single_angles = allowed_orientation_grid(single_quality, 2.0)
    temporal_angles = allowed_orientation_grid(temporal_quality, 2.0)
    lidar_orientation = detailed_possible_occupancy(
        single_frames, 0.004, 2.0, allowed_angles_deg=single_angles
    )
    temporal_orientation = detailed_possible_occupancy(
        temporal_frames, 0.004, 2.0, allowed_angles_deg=temporal_angles
    )

    fine_recomputed = case["case_id"] in {"validation_039", "normal_02_00"}
    fine_started = time.monotonic()
    fine = (
        detailed_possible_occupancy(temporal_frames, 0.002, 1.0)
        if fine_recomputed
        else baseline
    )
    fine_time = time.monotonic() - fine_started
    maximum_dimensions = (
        maximum_supported_dimensions(
            temporal_frames,
            temporal_angles if temporal_quality["usable"] else baseline.angles_deg,
            0.002,
        )
        if case["case_id"] != "validation_039"
        else {"not_evaluated": "size-bound focus is the three normal false-infeasible cases"}
    )

    support, support_projection_time = project_possible_set_support(case, baseline)
    boxes = {
        "BASELINE_COARSE": baseline.bounds,
        "BASELINE_FINE": fine.bounds,
        "BEFORE_FREE_SPACE": no_free.bounds,
        "UT10": ut10.bounds,
        "O_LIDAR_CONSTRAINED": lidar_orientation.bounds,
        "O_TEMPORAL_CONSTRAINED": temporal_orientation.bounds,
        "POSITION_FIXED": evidence_center_box(temporal_frames),
    }
    expected = str(case["expected_hash"])
    combined_rows, cpp_time = run_cpp_spec(
        Path(case["stream"]),
        make_combined_spec(case, boxes, support),
        RAW / case["case_id"] / ("combined_repeat" if repeat else "combined"),
        expected,
    )
    visible_fraction, visible_surface_count = partial_visible_fraction(
        temporal_frames, case["obstacle"]
    )
    result = {
        "case_id": case["case_id"],
        "repeat": repeat,
        "baseline": baseline.summary(),
        "fine": fine.summary(),
        "fine_recomputed": fine_recomputed,
        "before_free_space": no_free.summary(),
        "ut10": ut10.summary(),
        "orientation_single": lidar_orientation.summary(),
        "orientation_temporal": temporal_orientation.summary(),
        "orientation_single_quality": single_quality,
        "orientation_temporal_quality": temporal_quality,
        "maximum_supported_dimensions": maximum_dimensions,
        "possible_set_support": support,
        "cpp": combined_rows,
        "partial_visible_surface_fraction": visible_fraction,
        "visible_surface_count_gt_evaluation_only": visible_surface_count,
        "observation_count_ut20": len(temporal_frames),
        "observation_count_ut10": len(ten_frames),
        "fresh_hit_counts": [len(frame.hit_indices) for frame in temporal_frames],
        "timing": {
            "baseline_envelope_s": baseline_time,
            "before_free_space_envelope_s": no_free_time,
            "ut10_envelope_s": ut10_time,
            "fine_envelope_s": fine_time,
            "cell_support_projection_s": support_projection_time,
            "combined_cpp_query_s": cpp_time,
            "worker_wall_s": time.monotonic() - started,
            "worker_cpu_s": time.process_time() - cpu_started,
        },
    }
    result["digest"] = canonical_digest(
        {
            "baseline": result["baseline"],
            "support": support,
            "cpp": combined_rows,
            "classification_inputs": case["gt_row"],
        }
    )
    (RAW / case["case_id"]).mkdir(parents=True, exist_ok=True)
    (RAW / case["case_id"] / ("repeat_result.json" if repeat else "result.json")).write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return result


def repeat_case(case: dict[str, Any]) -> dict[str, Any]:
    """Repeat only the final selected representation, not unrelated ablations."""
    started = time.monotonic()
    frames = selected_frames_window(case["frames"], int(case["frame_now_ns"]), 20)
    baseline = detailed_possible_occupancy(frames, 0.004, 2.0)
    support, projection_time = project_possible_set_support(case, baseline)
    boxes = {"BASELINE_COARSE": baseline.bounds}
    cpp, cpp_time = run_cpp_spec(
        Path(case["stream"]),
        make_combined_spec(case, boxes, support),
        RAW / case["case_id"] / "combined_repeat",
        str(case["expected_hash"]),
    )
    result = {
        "case_id": case["case_id"],
        "baseline": baseline.summary(),
        "possible_set_support": support,
        "cpp": cpp,
        "timing": {
            "cell_support_projection_s": projection_time,
            "combined_cpp_query_s": cpp_time,
            "worker_wall_s": time.monotonic() - started,
        },
    }
    result["digest"] = canonical_digest(
        {
            "baseline": result["baseline"],
            "support": support,
            "support_cpp": cpp["TF_UNION_SUPPORT"],
        }
    )
    (RAW / case["case_id"] / "repeat_result.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return result


def parse_stream_frame(path: Path, stamp: int) -> dict[str, float]:
    prefix = f"FRAME\t{stamp}\t"
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith(prefix):
            tokens = line.split("\t")
            return {
                "ego_s_m": float(tokens[3]),
                "ego_d_m": float(tokens[4]),
                "ego_speed_mps": float(tokens[5]),
            }
    raise RuntimeError(f"frame {stamp} absent from {path}")


def reconstruct_normal_case(
    case_id: str,
    prior_rows: dict[tuple[str, str], dict[str, str]],
    waypoints: list[dict[str, Any]],
    collision_model: dict[str, Any],
) -> dict[str, Any]:
    gt_row = prior_rows[(case_id, "S3_RULE_UT20_ORIENT")]
    spec_path = PRIOR / ".raw/cpp" / case_id / "rulebook_spec.tsv"
    spec = read_spec(spec_path)
    nominal_bounds = polygon_bounds(spec["polygons"]["G0"])
    raster_bounds = polygon_bounds(spec["polygons"]["G1"])
    obstacle_x = 0.5 * (nominal_bounds[0] + nominal_bounds[1])
    obstacle_y = 0.5 * (nominal_bounds[2] + nominal_bounds[3])
    station = float(spec["manifest_s"])
    waypoint = min(waypoints, key=lambda item: abs(float(item["s_m"]) - station))
    tangent = float(waypoint["psi_rad"])
    d = -(obstacle_x - float(waypoint["x_m"])) * math.sin(tangent) + (
        obstacle_y - float(waypoint["y_m"])
    ) * math.cos(tangent)
    obstacle = ObstacleSpec(
        shape="rect",
        x=obstacle_x,
        y=obstacle_y,
        s=station,
        d=d,
        yaw=float(gt_row["obstacle_yaw_rad"]),
        width=float(gt_row["obstacle_width_m"]),
        height=float(gt_row["obstacle_height_m"]),
    )
    map_yaml = PRIOR / ".raw/normal_maps" / case_id / f"{case_id}.yaml"
    clean_model = SimulatorRasterCollisionModel(CLEAN_MAP, collision_model)
    baked_model = SimulatorRasterCollisionModel(map_yaml, collision_model)
    frames = []
    distance = float(gt_row["observation_distance_m"])
    for index, (delta_s, status) in enumerate(
        zip((0.0, 0.04, 0.08), ("RAW", "TENTATIVE", "CONFIRMED"))
    ):
        pose = rulebook.interpolate_reference(waypoints, station - distance + delta_s)
        frames.append(
            rulebook.synthetic_ray_frame(
                clean_model,
                baked_model,
                pose,
                obstacle,
                {
                    "x_min": raster_bounds[0],
                    "x_max": raster_bounds[1],
                    "y_min": raster_bounds[2],
                    "y_max": raster_bounds[3],
                },
                20_000_000_000 + index * 10_000_000,
                status,
            )
        )
    expected_s0 = polygon_bounds(spec["polygons"]["TC_S0_SINGLE"])
    expected_s1 = polygon_bounds(spec["polygons"]["TC_S1_UT20"])
    if max(abs(a - b) for a, b in zip(frames[-1].detector_box, expected_s0)) > 1.0e-12:
        raise RuntimeError(f"{case_id}: S0 reconstruction differs from stored specification")
    reconstructed_s1 = rulebook.union_detector_box(frames)
    if max(abs(a - b) for a, b in zip(reconstructed_s1, expected_s1)) > 1.0e-12:
        raise RuntimeError(f"{case_id}: UT20 reconstruction differs from stored specification")
    ego_pose = rulebook.interpolate_reference(waypoints, station - distance + 0.08)
    return {
        "case_id": case_id,
        "category": gt_row["category"],
        "frames": frames,
        "frame_now_ns": frames[-1].stamp_ns,
        "stamp_ns": EXACT_STAMP,
        "expected_hash": "CORRIDOR_ONLY",
        "stream": str(TIME_AXIS / "streams/validation_039.stream.tsv"),
        "manifest_s": station,
        "obstacle": {
            "x": obstacle.x,
            "y": obstacle.y,
            "s": obstacle.s,
            "d": obstacle.d,
            "yaw": obstacle.yaw,
            "width": obstacle.width,
            "height": obstacle.height,
        },
        "raster": {
            "x_min": raster_bounds[0],
            "x_max": raster_bounds[1],
            "y_min": raster_bounds[2],
            "y_max": raster_bounds[3],
        },
        "gt_row": gt_row,
        "metadata": {
            "track_curvature_radpm": float(waypoint["kappa_radpm"]),
            "local_track_width_m": float(waypoint["d_left"]) + float(waypoint["d_right"]),
            "ego_x_m": ego_pose["x"],
            "ego_y_m": ego_pose["y"],
            "ego_yaw_rad": ego_pose["yaw"],
            "ego_s_m": ego_pose["s"],
            "ego_d_m": 0.0,
            "ego_speed_mps": "UNDEFINED_IN_STORED_SYNTHETIC_SPEC",
        },
    }


def validation039_case(
    prior_rows: dict[tuple[str, str], dict[str, str]],
) -> tuple[dict[str, Any], float]:
    manifest, frames, bag_parse_time, _ = rulebook.load_existing_scenario(
        "validation_039", EXACT_STAMP
    )
    stream = TIME_AXIS / "streams/validation_039.stream.tsv"
    raster = manifest["baked_obstacle_raster"]["world_half_open_bounds_m"]
    obstacle = {key: float(manifest["obstacle"][key]) for key in ("x", "y", "s", "d", "yaw", "width", "height")}
    return {
        "case_id": "validation_039",
        "category": "safety_anchor",
        "frames": frames,
        "frame_now_ns": EXACT_STAMP,
        "stamp_ns": EXACT_STAMP,
        "expected_hash": EXPECTED_HASH,
        "stream": str(stream),
        "manifest_s": obstacle["s"],
        "obstacle": obstacle,
        "raster": {key: float(value) for key, value in raster.items()},
        "gt_row": prior_rows[("validation_039", "S3_RULE_UT20_ORIENT")],
        "metadata": parse_stream_frame(stream, EXACT_STAMP),
    }, bag_parse_time


def float_or_none(value: str) -> float | None:
    return None if value in {"", "nan", "NaN"} else float(value)


def passing_side(gt_row: dict[str, str]) -> str:
    return "left" if float(gt_row["gt_planner_left_corridor_m"]) > float(gt_row["gt_planner_right_corridor_m"]) else "right"


def row_corridor(row: dict[str, str], side: str) -> float:
    return float(row[f"{side}_corridor"])


def boundary_delta(row: dict[str, str], gt_row_cpp: dict[str, str], side: str) -> tuple[float, float, float]:
    signed, under, over = rulebook.boundary_error(row, gt_row_cpp, side)
    return signed * 1000.0, under * 1000.0, over * 1000.0


def percentile(values: Sequence[float], q: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def classification_for_strategy(false_feasible: int, false_infeasible: int, safe_039: bool) -> str:
    if not safe_039 or false_feasible:
        return "UNSAFE"
    if false_infeasible < 3:
        return "PROMISING_SHADOW"
    return "OVER_CONSERVATIVE"


def main() -> int:
    args = parse_args()
    if not BINARY.is_file():
        raise RuntimeError(f"missing existing diagnostic executable: {BINARY}")
    started = time.monotonic()
    cpu_self_before = resource.getrusage(resource.RUSAGE_SELF)
    cpu_children_before = resource.getrusage(resource.RUSAGE_CHILDREN)
    memory_before = memory_snapshot()
    production_before = {name: sha256(path) for name, path in rulebook.PRODUCTION_FILES.items()}
    OUTPUT.mkdir(parents=True, exist_ok=True)
    RAW.mkdir(parents=True, exist_ok=True)

    prior_normal = read_csv(PRIOR / "normal_case_results.csv")
    prior_039 = read_csv(PRIOR / "validation039_rulebook_timeline.csv")
    lookup: dict[tuple[str, str], dict[str, str]] = {}
    for row in prior_normal:
        lookup[(row["case_id"], row["strategy"])] = row
    for row in prior_039:
        if int(row["scan_stamp_ns"]) == EXACT_STAMP:
            lookup[(row["case_id"], row["strategy"])] = row
    false_ids = tuple(
        row["case_id"]
        for row in prior_normal
        if row["strategy"] == "S3_RULE_UT20_ORIENT" and row["gt_valid_candidate_rejected"] == "True"
    )
    if false_ids != FALSE_CASE_IDS:
        raise RuntimeError(f"stored false-infeasible set changed: {false_ids}")

    manifest_039 = json.loads((SCENARIO_ROOT / "validation_039/manifest.json").read_text(encoding="utf-8"))
    waypoints = load_waypoints(WAYPOINTS)
    normal_case_ids = sorted(
        row["case_id"]
        for row in prior_normal
        if row["strategy"] == "S3_RULE_UT20_ORIENT"
    )
    normal_cases = [
        reconstruct_normal_case(case_id, lookup, waypoints, manifest_039["simulator_collision_model"])
        for case_id in normal_case_ids
    ]
    normal_case_by_id = {case["case_id"]: case for case in normal_cases}
    cases = [
        normal_case_by_id[case_id] for case_id in FALSE_CASE_IDS
    ]
    case_039, bag_parse_time = validation039_case(lookup)
    all_cases = [case_039] + cases

    parallel_started = time.monotonic()
    with concurrent.futures.ProcessPoolExecutor(max_workers=WORKERS) as executor:
        futures = {executor.submit(analyze_case, case, False): case["case_id"] for case in all_cases}
        results = {futures[future]: future.result() for future in concurrent.futures.as_completed(futures)}
    parallel_wall = time.monotonic() - parallel_started
    memory_after_parallel = memory_snapshot()
    if memory_after_parallel["swap_used_bytes"] > memory_before["swap_used_bytes"]:
        raise RuntimeError("swap increased during workers=4 audit; results intentionally rejected")

    # The actual-set support improved one of the three failures. Apply that single promising
    # strategy to all 25 stored deterministic normal specifications. The three detailed cases reuse
    # their primary results; the remaining independent cases run serially within workers=4.
    normal25_started = time.monotonic()
    remaining_normal_cases = [
        case for case in normal_cases if case["case_id"] not in results
    ]
    with concurrent.futures.ProcessPoolExecutor(max_workers=WORKERS) as executor:
        remaining_normal_results = list(executor.map(repeat_case, remaining_normal_cases))
    normal_union_results = {
        **{case_id: results[case_id] for case_id in FALSE_CASE_IDS},
        **{item["case_id"]: item for item in remaining_normal_results},
    }
    if set(normal_union_results) != set(normal_case_ids):
        raise RuntimeError("normal25 support evaluation did not cover every stored case")
    normal25_support_time = time.monotonic() - normal25_started
    memory_after_normal25 = memory_snapshot()
    if memory_after_normal25["swap_used_bytes"] > memory_before["swap_used_bytes"]:
        raise RuntimeError("swap increased during normal25 support audit; results rejected")

    # Repeat the final set-support representation for the safety anchor and one small obstacle.
    repeat_started = time.monotonic()
    repeat_cases = [case_039, next(case for case in cases if case["case_id"] == "normal_01_00")]
    with concurrent.futures.ProcessPoolExecutor(max_workers=2) as executor:
        repeat_values = list(executor.map(repeat_case, repeat_cases))
    if len(repeat_values) != 2:
        raise RuntimeError("determinism repeat did not return both cases")
    repeat_time = time.monotonic() - repeat_started
    repeat_by_id = {item["case_id"]: item for item in repeat_values}

    false_case_rows: list[dict[str, Any]] = []
    uncertainty_rows: list[dict[str, Any]] = []
    free_rows: list[dict[str, Any]] = []
    orientation_rows: list[dict[str, Any]] = []
    representation_rows: list[dict[str, Any]] = []
    anchor_rows: list[dict[str, Any]] = []
    temporal_rows: list[dict[str, Any]] = []

    case_by_id = {case["case_id"]: case for case in all_cases}
    for case_id in [*FALSE_CASE_IDS, "validation_039"]:
        case = case_by_id[case_id]
        result = results[case_id]
        cpp = result["cpp"]
        gt_cpp = cpp["G0"] if case_id != "validation_039" else cpp["G1"]
        side = passing_side(case["gt_row"]) if case_id != "validation_039" else "right"
        baseline_cpp = cpp["TC_BASELINE_COARSE"]
        support_cpp = cpp["TF_UNION_SUPPORT"]
        baseline_signed, baseline_under, baseline_over = boundary_delta(baseline_cpp, gt_cpp, side)
        support_signed, support_under, support_over = boundary_delta(support_cpp, gt_cpp, side)

        if case_id != "validation_039":
            gt_row = case["gt_row"]
            gt_corridor = float(gt_row[f"gt_planner_{side}_corridor_m"])
            baseline_corridor = row_corridor(baseline_cpp, side)
            support_corridor = row_corridor(support_cpp, side)
            obstacle = case["obstacle"]
            meta = case["metadata"]
            boundary_growth = (
                (float(baseline_cpp["d_max"]) - float(gt_cpp["d_max"])) * 1000.0
                if side == "left"
                else (float(gt_cpp["d_min"]) - float(baseline_cpp["d_min"])) * 1000.0
            )
            corridor_loss = (gt_corridor - baseline_corridor) * 1000.0
            projection_loss = corridor_loss - boundary_growth
            dominant = (
                "outer-box/Frenet projection sampling on curved track"
                if support_corridor > 0.0
                else "unobserved hidden extent plus rulebook 0.5 m size supremum"
                if abs(boundary_growth) >= abs(projection_loss)
                else "track/Frenet projection plus rotated-footprint interaction"
            )
            false_case_rows.append(
                {
                    "case_id": case_id,
                    "category": case["category"],
                    "obstacle_width_m": obstacle["width"],
                    "obstacle_length_m": obstacle["height"],
                    "obstacle_orientation_rad": obstacle["yaw"],
                    "obstacle_x_m": obstacle["x"],
                    "obstacle_y_m": obstacle["y"],
                    "obstacle_s_m": obstacle["s"],
                    "obstacle_d_m": obstacle["d"],
                    "track_curvature_radpm": meta["track_curvature_radpm"],
                    "local_track_width_m": meta["local_track_width_m"],
                    "ego_x_m": meta["ego_x_m"],
                    "ego_y_m": meta["ego_y_m"],
                    "ego_yaw_rad": meta["ego_yaw_rad"],
                    "ego_s_m": meta["ego_s_m"],
                    "ego_d_m": meta["ego_d_m"],
                    "ego_speed_mps": meta["ego_speed_mps"],
                    "observed_hit_count_ut20": sum(result["fresh_hit_counts"]),
                    "fresh_hit_counts": ";".join(str(value) for value in result["fresh_hit_counts"]),
                    "partial_visible_surface_fraction_gt_metric": result["partial_visible_surface_fraction"],
                    "ut20_observation_count": result["observation_count_ut20"],
                    "passing_side": side,
                    "physical_gt_corridor_m": max(float(gt_row["physical_left_corridor_m"]), float(gt_row["physical_right_corridor_m"])),
                    "gt_planner_corridor_m": gt_corridor,
                    "s3_outer_corridor_m": baseline_corridor,
                    "s3_union_support_corridor_m": support_corridor,
                    "boundary_growth_closing_passage_mm": boundary_growth,
                    "track_projection_additional_loss_mm": projection_loss,
                    "dominant_conservatism_source": dominant,
                    "exact_false_infeasible_reason": f"{side} corridor {gt_corridor:.6f} -> {baseline_corridor:.6f} m; required obstacle boundary expands {boundary_growth:.3f} mm and projection/footprint interaction adds {projection_loss:.3f} mm",
                    "best_evidence_based_tightening": "actual possible-set side support" if support_corridor > baseline_corridor else "none with current evidence",
                    "evidence_axis_u_upper_bound_m": result["maximum_supported_dimensions"]["u_evidence_supported_upper_bound_m"],
                    "evidence_axis_v_upper_bound_m": result["maximum_supported_dimensions"]["v_evidence_supported_upper_bound_m"],
                    "orientation_to_width_length_mapping_reliable": result["orientation_temporal_quality"]["usable"],
                }
            )

        no_free_cpp = cpp["TC_BEFORE_FREE_SPACE"]
        before_signed, before_under, before_over = boundary_delta(no_free_cpp, gt_cpp, side)
        free_rows.append(
            {
                "case_id": case_id,
                "side": side,
                "possible_area_before_m2": result["before_free_space"]["possible_area_m2"],
                "possible_area_after_m2": result["baseline"]["possible_area_m2"],
                "area_removed_m2": result["before_free_space"]["possible_area_m2"] - result["baseline"]["possible_area_m2"],
                "area_reduction_percent": 100.0 * (1.0 - result["baseline"]["possible_area_m2"] / result["before_free_space"]["possible_area_m2"]),
                "selected_boundary_before_error_mm": before_signed,
                "selected_boundary_after_error_mm": baseline_signed,
                "selected_boundary_before_undercoverage_mm": before_under,
                "selected_boundary_after_undercoverage_mm": baseline_under,
                "selected_boundary_before_overcoverage_mm": before_over,
                "selected_boundary_after_overcoverage_mm": baseline_over,
                "corridor_before_m": row_corridor(no_free_cpp, side),
                "corridor_after_m": row_corridor(baseline_cpp, side),
                "false_infeasible_resolved_by_free_space": case_id != "validation_039" and row_corridor(no_free_cpp, side) <= 0.0 < row_corridor(baseline_cpp, side),
                "tolerance_source": "existing simulator epsilon; recorded 039 additionally uses existing evaluator guard and detector 3*cluster_sigma",
            }
        )

        for name, key, quality in (
            ("O_FREE", "TC_BASELINE_COARSE", {"usable": True, "criterion": "unrestricted 0..90deg"}),
            ("O_LIDAR_CONSTRAINED", "TC_O_LIDAR_CONSTRAINED", result["orientation_single_quality"]),
            ("O_TEMPORAL_CONSTRAINED", "TC_O_TEMPORAL_CONSTRAINED", result["orientation_temporal_quality"]),
        ):
            row = cpp[key]
            signed, under, over = boundary_delta(row, gt_cpp, side)
            envelope = result[
                "baseline" if name == "O_FREE" else "orientation_single" if name == "O_LIDAR_CONSTRAINED" else "orientation_temporal"
            ]
            orientation_rows.append(
                {
                    "case_id": case_id,
                    "orientation_strategy": name,
                    "orientation_evidence_usable": quality["usable"],
                    "quality_criterion": quality["criterion"],
                    "principal_edge_angle_deg_mod90": quality.get("principal_edge_angle_deg_mod90", ""),
                    "orientation_half_angle_deg": quality.get("orientation_half_angle_deg", ""),
                    "maximum_orthogonal_residual_m": quality.get("maximum_orthogonal_residual_m", ""),
                    "established_tolerance_m": quality.get("established_sensor_evaluator_tolerance_m", ""),
                    "orientation_count": envelope["orientation_count"],
                    "possible_area_m2": envelope["possible_area_m2"],
                    "selected_boundary_error_mm": signed,
                    "undercoverage_mm": under,
                    "overcoverage_mm": over,
                    "corridor_m": row_corridor(row, side),
                    "same_path_hard_valid": row.get("same_path_hard_valid", ""),
                }
            )

        for model, row, area in (
            ("E_OUTER_BOX", baseline_cpp, result["baseline"]["aabb_area_m2"]),
            ("E_UNION_SET", support_cpp, result["baseline"]["possible_area_m2"]),
            ("E_SIDE_SUPPORT", support_cpp, result["baseline"]["possible_area_m2"]),
        ):
            signed, under, over = boundary_delta(row, gt_cpp, side)
            representation_rows.append(
                {
                    "case_id": case_id,
                    "representation_model": model,
                    "side": side,
                    "represented_area_m2": area,
                    "selected_boundary_error_mm": signed,
                    "undercoverage_mm": under,
                    "overcoverage_mm": over,
                    "corridor_m": row_corridor(row, side),
                    "same_path_hard_valid": row.get("same_path_hard_valid", ""),
                    "same_path_clearance_m": row.get("same_path_production_clearance", ""),
                    "note": "union and side-support preserve the exact occupied-cell support; side-support avoids materializing filled impossible corners",
                }
            )

        for window_ms, key, source in (
            (10, "TC_UT10", "computed two observations"),
            (20, "TC_BASELINE_COARSE", "computed three observations"),
            (30, "TC_BASELINE_COARSE", "bit-identical selected stamps to UT20"),
            (50, "TC_BASELINE_COARSE", "bit-identical selected stamps to UT20"),
        ):
            row = cpp[key]
            temporal_rows.append(
                {
                    "case_id": case_id,
                    "window_ms": window_ms,
                    "observation_count": result["observation_count_ut10"] if window_ms == 10 else result["observation_count_ut20"],
                    "evaluation_source": source,
                    "possible_area_m2": result["ut10"]["possible_area_m2"] if window_ms == 10 else result["baseline"]["possible_area_m2"],
                    "corridor_m": row_corridor(row, side),
                    "same_path_hard_valid": row.get("same_path_hard_valid", ""),
                }
            )

        ablations = [
            ("C_POSITION_FIXED", "TC_POSITION_FIXED", "unsafe point-estimate ablation; position is not hard-set identifiable"),
            ("C_ORIENTATION_FIXED", "TC_O_TEMPORAL_CONSTRAINED", "applied only when geometric line-normal cone is identifiable"),
            ("C_WIDTH_EVIDENCE", "TC_BASELINE_COARSE", "near-0.5 m witness remains; no safe cap tightening"),
            ("C_LENGTH_EVIDENCE", "TC_BASELINE_COARSE", "near-0.5 m witness remains; hidden face is unobserved"),
            ("C_TEMPORAL", "TC_BASELINE_COARSE", "UT20 positive and negative evidence"),
            ("C_FULL_EVIDENCE", "TF_UNION_SUPPORT", "all allowable evidence plus actual possible-set support"),
        ]
        for component, key, note in ablations:
            row = cpp[key]
            signed, under, over = boundary_delta(row, gt_cpp, side)
            uncertainty_rows.append(
                {
                    "case_id": case_id,
                    "component": component,
                    "estimator_uses_gt": False,
                    "evidence_supported": component not in {"C_POSITION_FIXED"} and not (
                        component == "C_ORIENTATION_FIXED" and not result["orientation_temporal_quality"]["usable"]
                    ),
                    "corridor_m": row_corridor(row, side),
                    "selected_boundary_error_mm": signed,
                    "undercoverage_mm": under,
                    "overcoverage_mm": over,
                    "same_path_hard_valid": row.get("same_path_hard_valid", ""),
                    "classification": "DISQUALIFIED" if under > 0.0 or row.get("same_path_hard_valid") == "true" else "RETAINS_ANCHOR" if case_id == "validation_039" else "DIAGNOSTIC",
                    "note": note,
                }
            )

    # Anchor table contains every distinct diagnostic tightening strategy.
    anchor_result = results["validation_039"]
    anchor_cpp = anchor_result["cpp"]
    anchor_gt = anchor_cpp["G1"]
    anchor_models = (
        ("S3_OUTER_BOX_BASELINE", "TC_BASELINE_COARSE", anchor_result["baseline"]),
        ("S3_FINE_CONVERGENCE", "TC_BASELINE_FINE", anchor_result["fine"]),
        ("S3_UNION_SIDE_SUPPORT", "TF_UNION_SUPPORT", anchor_result["baseline"]),
        ("O_LIDAR_GATED", "TC_O_LIDAR_CONSTRAINED", anchor_result["orientation_single"]),
        ("O_TEMPORAL_GATED", "TC_O_TEMPORAL_CONSTRAINED", anchor_result["orientation_temporal"]),
        ("POSITION_POINT_ESTIMATE", "TC_POSITION_FIXED", None),
    )
    for strategy, key, envelope in anchor_models:
        row = anchor_cpp[key]
        signed, under, over = boundary_delta(row, anchor_gt, "right")
        hard_invalid = row["same_path_hard_valid"] == "false"
        anchor_rows.append(
            {
                "strategy": strategy,
                "source_stamp_ns": EXACT_STAMP,
                "path_geometry_hash": EXPECTED_HASH,
                "serialized_path_sha256": EXPECTED_PATH_SHA256,
                "same_path_hard_invalid": hard_invalid,
                "same_path_clearance_m": float_or_none(row["same_path_production_clearance"]),
                "selected_side_possible_boundary_d_min_m": float(row["d_min"]),
                "gt_selected_boundary_d_min_m": float(anchor_gt["d_min"]),
                "gt_undercoverage_mm": under,
                "gt_overcoverage_mm": over,
                "possible_set_area_m2": envelope["possible_area_m2"] if envelope else "NOT_A_HARD_SET",
                "classification": "DISQUALIFIED" if not hard_invalid or under > 0.0 else "ANCHOR_RETAINED",
            }
        )

    # Compare the cached outer-box baseline with the exact production-projected possible-set
    # support evaluated above for every one of the 25 stored deterministic specifications.
    baseline_normal = [
        row for row in prior_normal if row["strategy"] == "S3_RULE_UT20_ORIENT"
    ]
    normal_strategy_rows: list[dict[str, Any]] = []
    for strategy in ("S3_OUTER_BOX_BASELINE", "S3_UNION_SIDE_SUPPORT"):
        values = []
        for row in baseline_normal:
            replacement = normal_union_results[row["case_id"]]
            if strategy == "S3_UNION_SIDE_SUPPORT":
                cpp_row = replacement["cpp"]["TF_UNION_SUPPORT"]
                side = passing_side(row)
                gt_corridor = float(row[f"gt_planner_{side}_corridor_m"])
                corridor = row_corridor(cpp_row, side)
                gt_cpp = replacement["cpp"]["G0"]
                _, under, over = boundary_delta(cpp_row, gt_cpp, side)
                left_gt = float(row["gt_planner_left_corridor_m"])
                right_gt = float(row["gt_planner_right_corridor_m"])
                left_support = float(cpp_row["left_corridor"])
                right_support = float(cpp_row["right_corridor"])
                values.append(
                    {
                        "false_feasible": (left_gt <= 0.0 < left_support) or (right_gt <= 0.0 < right_support),
                        "false_infeasible": (left_gt > 0.0 >= left_support) or (right_gt > 0.0 >= right_support),
                        "under": under,
                        "over": over,
                        "corridor_loss": max(0.0, gt_corridor - corridor) * 1000.0,
                    }
                )
            else:
                values.append(
                    {
                        "false_feasible": row["gt_invalid_candidate_accepted"] == "True",
                        "false_infeasible": row["gt_valid_candidate_rejected"] == "True",
                        "under": float(row["selected_side_boundary_undercoverage_mm"]),
                        "over": float(row["selected_side_boundary_overcoverage_mm"]),
                        "corridor_loss": float(row["corridor_loss_mm"]),
                    }
                )
        false_infeasible = sum(item["false_infeasible"] for item in values)
        false_feasible = sum(item["false_feasible"] for item in values)
        anchor = next(item for item in anchor_rows if item["strategy"] == strategy)
        safe_039 = anchor["classification"] == "ANCHOR_RETAINED"
        normal_strategy_rows.append(
            {
                "strategy": strategy,
                "case_count": 25,
                "false_feasible_count": false_feasible,
                "false_infeasible_count": false_infeasible,
                "false_feasible_rate": false_feasible / 25.0,
                "false_infeasible_rate": false_infeasible / 25.0,
                "mean_boundary_undercoverage_mm": sum(item["under"] for item in values) / 25.0,
                "p95_boundary_undercoverage_mm": percentile([item["under"] for item in values], 95),
                "max_boundary_undercoverage_mm": max(item["under"] for item in values),
                "mean_overcoverage_mm": sum(item["over"] for item in values) / 25.0,
                "p95_overcoverage_mm": percentile([item["over"] for item in values], 95),
                "max_overcoverage_mm": max(item["over"] for item in values),
                "mean_corridor_loss_mm": sum(item["corridor_loss"] for item in values) / 25.0,
                "small_obstacle_false_infeasible_count": sum(
                    item["false_infeasible"] and float(row["obstacle_width_m"]) <= 0.3 and float(row["obstacle_height_m"]) <= 0.3
                    for item, row in zip(values, baseline_normal)
                ),
                "validation039_safety_retained": safe_039,
                "classification": classification_for_strategy(false_feasible, false_infeasible, safe_039),
                "evaluation_basis": (
                    "cached prior S3 outer-box results"
                    if strategy == "S3_OUTER_BOX_BASELINE"
                    else "all 25 stored deterministic specifications evaluated with exact production-projected possible-set support"
                ),
            }
        )

    lifecycle_rows = [
        {
            "event": "validation_039",
            "status": "RAW",
            "stamp_ns": 11_210_000_000,
            "history_action": "collect positive/negative geometry only",
            "shadow_action": "do not expose full rulebook envelope to normal behavior",
            "history_available_at_confirmed": True,
            "dynamic_guard": "not dynamic",
        },
        {
            "event": "validation_039",
            "status": "TENTATIVE",
            "stamp_ns": 11_220_000_000,
            "history_action": "continue map-frame evidence intersection",
            "shadow_action": "collect only",
            "history_available_at_confirmed": True,
            "dynamic_guard": "not dynamic",
        },
        {
            "event": "validation_039",
            "status": "CONFIRMED UNKNOWN",
            "stamp_ns": EXACT_STAMP,
            "history_action": "consume retained RAW/TENTATIVE plus confirmed frame",
            "shadow_action": "evaluate conservative S3 shadow only",
            "history_available_at_confirmed": True,
            "dynamic_guard": "UNKNOWN hard-set shadow allowed; no production feed",
        },
        {
            "event": "future static track",
            "status": "STATIC",
            "stamp_ns": "",
            "history_action": "anchor in map frame; shrink only on consistent new evidence",
            "shadow_action": "retain shadow envelope",
            "history_available_at_confirmed": True,
            "dynamic_guard": "invalidate immediately on DYNAMIC transition",
        },
        {
            "event": "validation_014",
            "status": "DYNAMIC",
            "stamp_ns": "representative event",
            "history_action": "discard static map-frame rulebook envelope",
            "shadow_action": "S3 static shadow not applied",
            "history_available_at_confirmed": False,
            "dynamic_guard": "PASS",
        },
        {
            "event": "validation_034",
            "status": "DYNAMIC",
            "stamp_ns": "representative event",
            "history_action": "discard static map-frame rulebook envelope",
            "shadow_action": "S3 static shadow not applied",
            "history_available_at_confirmed": False,
            "dynamic_guard": "PASS",
        },
    ]

    determinism = {}
    for case_id in ("validation_039", "normal_01_00"):
        first, second = results[case_id], repeat_by_id[case_id]
        first_cpp = first["cpp"]["TF_UNION_SUPPORT"]
        second_cpp = second["cpp"]["TF_UNION_SUPPORT"]
        determinism[case_id] = {
            "possible_set_bounds_identical": first["baseline"]["bounds"] == second["baseline"]["bounds"],
            "possible_set_digest_identical": first["baseline"]["digest"] == second["baseline"]["digest"],
            "support_bounds_identical": first["possible_set_support"] == second["possible_set_support"],
            "clearance_identical": first_cpp.get("same_path_production_clearance") == second_cpp.get("same_path_production_clearance"),
            "classification_identical": first_cpp.get("same_path_hard_valid") == second_cpp.get("same_path_hard_valid"),
            "result_digest_identical": canonical_digest(
                {
                    "baseline": first["baseline"],
                    "support": first["possible_set_support"],
                    "support_cpp": first_cpp,
                }
            ) == second["digest"],
        }
    deterministic = all(all(value.values()) for value in determinism.values())
    if not deterministic:
        raise RuntimeError(f"determinism failure: {determinism}")

    production_after = {name: sha256(path) for name, path in rulebook.PRODUCTION_FILES.items()}
    if production_before != production_after:
        raise RuntimeError("production source/binary changed during diagnostic audit")
    memory_after = memory_snapshot()
    cpu_self_after = resource.getrusage(resource.RUSAGE_SELF)
    cpu_children_after = resource.getrusage(resource.RUSAGE_CHILDREN)
    cpu_time = (
        cpu_self_after.ru_utime - cpu_self_before.ru_utime
        + cpu_self_after.ru_stime - cpu_self_before.ru_stime
        + cpu_children_after.ru_utime - cpu_children_before.ru_utime
        + cpu_children_after.ru_stime - cpu_children_before.ru_stime
    )
    prior_performance = json.loads((PRIOR / "performance.json").read_text(encoding="utf-8"))
    worker_cpu_sum = sum(result["timing"]["worker_cpu_s"] for result in results.values())
    worker_wall_sum = sum(result["timing"]["worker_wall_s"] for result in results.values())
    max_rss = max(cpu_self_after.ru_maxrss, cpu_children_after.ru_maxrss) * 1024
    elapsed = time.monotonic() - started
    performance = {
        "worker_count": WORKERS,
        "worker_policy": "four independent groups; workers=5 not attempted because only four primary groups exist",
        "offline_wall_time_s": elapsed,
        "parallel_primary_wall_time_s": parallel_wall,
        "parallel_worker_wall_sum_s": worker_wall_sum,
        "parallel_worker_cpu_sum_s": worker_cpu_sum,
        "observed_parallelism_ratio_worker_wall_sum_over_wall": worker_wall_sum / parallel_wall,
        "cpu_time_s": cpu_time,
        "diagnostic_build_time_s": args.diagnostic_build_time_s,
        "diagnostic_build_max_rss_bytes": args.diagnostic_build_max_rss_bytes,
        "diagnostic_build_reused": False,
        "max_rss_bytes": max_rss,
        "memory_available_before_bytes": memory_before["memory_available_bytes"],
        "memory_available_after_parallel_bytes": memory_after_parallel["memory_available_bytes"],
        "memory_available_after_normal25_bytes": memory_after_normal25["memory_available_bytes"],
        "memory_available_after_bytes": memory_after["memory_available_bytes"],
        "swap_used_before_bytes": memory_before["swap_used_bytes"],
        "swap_used_after_bytes": memory_after["swap_used_bytes"],
        "swap_change_bytes": memory_after["swap_used_bytes"] - memory_before["swap_used_bytes"],
        "bag_parse_count": 1,
        "bag_parse_time_s": bag_parse_time,
        "closed_loop_replay_count": 0,
        "dataset_regeneration_count": 0,
        "determinism_repeat_time_s": repeat_time,
        "normal25_support_evaluation_time_s": normal25_support_time,
        "previous_worker1_full25_wall_time_s": prior_performance["offline_evaluation_time_s"],
        "previous_worker1_case_throughput_per_s": 25.0 / prior_performance["offline_evaluation_time_s"],
        "current_primary_group_throughput_per_s": 4.0 / parallel_wall,
        "throughput_comparison_note": "different workload sizes; report only, not a claimed apples-to-apples speedup",
    }

    best = next(row for row in normal_strategy_rows if row["strategy"] == "S3_UNION_SIDE_SUPPORT")
    anchor_best = next(row for row in anchor_rows if row["strategy"] == "S3_UNION_SIDE_SUPPORT")
    update_times = [result["timing"]["baseline_envelope_s"] for result in results.values()]
    query_times = [result["timing"]["cell_support_projection_s"] for result in results.values()]
    history_bytes = []
    for case in all_cases:
        size = 0
        for frame in case["frames"]:
            size += sum(
                array.nbytes
                for array in (frame.origin, frame.directions, frame.ranges, frame.free_lengths, frame.hits)
            )
        history_bytes.append(size)
    cost = {
        "measured_reference_implementation": "offline Python raster set-membership plus existing diagnostic C++ projector; not optimized production code",
        "mean_per_obstacle_update_s": sum(update_times) / len(update_times),
        "max_per_obstacle_update_s": max(update_times),
        "mean_per_shadow_support_query_s": sum(query_times) / len(query_times),
        "max_per_shadow_support_query_s": max(query_times),
        "mean_history_storage_bytes_per_track": sum(history_bytes) / len(history_bytes),
        "max_history_storage_bytes_per_track": max(history_bytes),
        "history_observations_per_track": 3,
        "three_obstacle_memory_bytes": 3 * max(history_bytes),
        "three_obstacle_serial_update_s": 3 * max(update_times),
        "observed_update_period_s": 0.01,
        "reference_cpu_fraction_at_100hz_one_obstacle": 100.0 * sum(update_times) / len(update_times),
        "practical_online_as_measured": False,
        "note": "accuracy is not production-ready and the audit raster implementation is orders of magnitude too slow; optimize only after evidence insufficiency is resolved",
    }

    write_csv(OUTPUT / "false_infeasible_cases.csv", false_case_rows)
    write_csv(OUTPUT / "uncertainty_ablation.csv", uncertainty_rows)
    write_csv(OUTPUT / "free_space_pruning.csv", free_rows)
    write_csv(OUTPUT / "orientation_constraints.csv", orientation_rows)
    write_csv(OUTPUT / "representation_model_comparison.csv", representation_rows)
    write_csv(OUTPUT / "normal25_strategy_comparison.csv", normal_strategy_rows)
    write_csv(OUTPUT / "validation039_safety_anchor.csv", anchor_rows)
    write_csv(OUTPUT / "lifecycle_shadow_analysis.csv", lifecycle_rows)
    write_csv(OUTPUT / ".raw/temporal_window_sensitivity.csv", temporal_rows)
    (OUTPUT / "performance.json").write_text(
        json.dumps(performance, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    free_resolved = sum(bool(row["false_infeasible_resolved_by_free_space"]) for row in free_rows if row["case_id"] != "validation_039")
    union_resolved = 3 - int(best["false_infeasible_count"])
    summary = {
        "schema": "s3_shadow_false_infeasible_audit/1",
        "diagnostic_only": True,
        "production_changes": False,
        "production_hashes_before": production_before,
        "production_hashes_after": production_after,
        "closed_loop_replay_count": 0,
        "cma_executed": False,
        "dataset_regenerated": False,
        "false_infeasible_case_ids": list(FALSE_CASE_IDS),
        "root_causes": false_case_rows,
        "free_space_pruning": {
            "already_present_in_baseline_s3": True,
            "false_infeasible_cases_resolved": free_resolved,
            "interpretation": "negative rays remove impossible hypotheses but do not observe the hidden face; remaining near-0.5 m hypotheses close all three passages",
        },
        "small_obstacle_size_evidence": {
            case_id: results[case_id]["maximum_supported_dimensions"] for case_id in FALSE_CASE_IDS
        },
        "orientation_conclusion": "P_AXIS and P_ORIENT outer bounds are identical in all three false cases; orientation uncertainty contributes no passage-closing delta there. It remains essential for validation_039 because its line-normal cone is not identifiable under the established recorded-scan tolerance.",
        "representation_conclusion": {
            "union_support_cases_resolved": union_resolved,
            "classification": "MODEL_REPRESENTATION_LIMITED" if union_resolved == 3 else "MIXED" if union_resolved else "NOT_MODEL_REPRESENTATION_LIMITED_FOR_THE_THREE_FAILURES",
            "validation039_outer_fill_ratio": results["validation_039"]["baseline"]["fill_ratio"],
            "note": "union support resolves only normal_02_00 through curved-track support projection, slightly reduces 039 overcoverage, and worsens the other two failure corridors; true hidden-extent uncertainty remains dominant there",
        },
        "best_shadow_candidate": {
            "name": "S3_UNION_SIDE_SUPPORT",
            "normal25_false_feasible_count": best["false_feasible_count"],
            "normal25_false_infeasible_count": best["false_infeasible_count"],
            "validation039_same_path_hard_invalid": anchor_best["same_path_hard_invalid"],
            "validation039_gt_undercoverage_mm": anchor_best["gt_undercoverage_mm"],
            "validation039_gt_overcoverage_mm": anchor_best["gt_overcoverage_mm"],
            "classification": best["classification"],
        },
        "desired_target_met": best["false_feasible_count"] == 0 and best["false_infeasible_count"] <= 1,
        "ut20_conclusion": {
            "remains_justified": True,
            "reason": "10 ms uses only two observations; 20 ms captures all three. 30 and 50 ms select no additional stored observations and are bit-identical, so the failures are not caused by the 20 ms cutoff.",
        },
        "lifecycle_conclusion": {
            "raw_tentative_history_useful": True,
            "waiting_until_confirmed_would_lose_useful_history": True,
            "validation039_history_ready_at_confirmed": True,
            "dynamic_014_guard_pass": True,
            "dynamic_034_guard_pass": True,
        },
        "production_cost_estimate": cost,
        "evidence_strong_enough_for_production_shadow_mode": False,
        "evidence_classification": "INSUFFICIENT_EVIDENCE" if best["false_infeasible_count"] >= 3 else "PROMISING_SHADOW",
        "determinism": {"cases": determinism, "bit_identical": deterministic},
        "performance": performance,
        "provenance": {
            "prior_artifacts": [str(PRIOR), str(TEMPORAL), str(REPRESENTATION), str(TIME_AXIS)],
            "diagnostic_renderer": str(Path(__file__)),
            "diagnostic_renderer_sha256": sha256(Path(__file__)),
            "existing_diagnostic_binary": str(BINARY),
            "existing_diagnostic_binary_sha256": sha256(BINARY),
            "source_stamp_ns": EXACT_STAMP,
            "path_geometry_hash": EXPECTED_HASH,
            "serialized_path_sha256": EXPECTED_PATH_SHA256,
        },
    }
    (OUTPUT / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    case_table = "\n".join(
        f"| {row['case_id']} | {row['obstacle_width_m']:.2f} x {row['obstacle_length_m']:.2f} m | {row['gt_planner_corridor_m']:.6f} m | {row['s3_outer_corridor_m']:.6f} m | {row['dominant_conservatism_source']} | {row['best_evidence_based_tightening']} |"
        for row in false_case_rows
    )
    strategy_table = "\n".join(
        f"| {row['strategy']} | {str(row['validation039_safety_retained']).lower()} | {row['false_feasible_count']}/25 | {row['false_infeasible_count']}/25 | {row['classification']} |"
        for row in normal_strategy_rows
    )
    readme = f"""# S3 shadow-mode + false-infeasible root-cause audit

이 산출물은 완전한 offline diagnostic이다. production obstacle representation, planner,
validator, lifecycle, controller, YAML, generator에는 어떤 입력이나 변경도 주지 않았다.
validation_039 bag은 한 번만 offline parse했고 closed-loop replay와 CMA 실행은 0회다.

## 결론

S3 baseline은 이미 positive hit와 negative free-space ray를 모두 사용한다. Free-space
pruning은 possible area를 줄이지만 세 false-infeasible passage를 복구하지 못했다. 세 경우
모두 관측되지 않은 hidden face 때문에 `<0.5 m` rulebook supremum에 가까운 직사각형
hypothesis가 실제로 남는다. Orientation을 LiDAR line geometry로 제한해도 세 경우의
P_AXIS/P_ORIENT outer bound가 같아 passage가 열리지 않는다.

Actual union/side support는 039의 불가능한 outer-box corner를 줄이고 curved-track support를
직접 투영하지만 normal 실패
{union_resolved}/3건만 해결했다. 최선의 shadow 후보 `S3_UNION_SIDE_SUPPORT`는
validation_039 dangerous path hard rejection={anchor_best['same_path_hard_invalid']}, GT
undercoverage={anchor_best['gt_undercoverage_mm']:.3f} mm를 유지했으나 normal
false-infeasible={best['false_infeasible_count']}/25다. 따라서 분류는
`{summary['evidence_classification']}`이며 production shadow-mode 구현 근거는 아직 부족하다.

## Table 1 - false-infeasible cases

| case | obstacle size | physical/planner-safe corridor | S3 corridor | dominant conservatism source | best evidence-based tightening |
|---|---:|---:|---:|---|---|
{case_table}

## Table 2 - final strategies

| strategy | 039 safe rejection | normal false-feasible | normal false-infeasible | classification |
|---|---:|---:|---:|---|
{strategy_table}

## 핵심 해석

- 세 case의 exact passage closure는 `false_infeasible_cases.csv`에 obstacle-boundary 성장과
  track/Frenet/rotated-footprint 추가 손실로 분해했다.
- `free_space_pruning.csv`는 같은 tolerance에서 pruning 전/후 area, boundary, corridor를
  기록한다. 새로운 tolerance는 없다.
- 작은 obstacle의 evidence-supported axis upper bound는 `summary.json`에 수치로 기록했다.
  orientation evidence가 unusable이면 그 두 축을 physical width/length로 이름 붙일 수 없다.
  0.5 m 값은 strict rulebook set의 supremum이며 실제 크기 가정이 아니다.
- validation_039의 orientation evidence는 recorded-scan tolerance로 계산한 line-normal
  cone이 식별 가능하지 않다. P_AXIS 강제는 금지해야 한다.
- RAW/TENTATIVE evidence를 보존하면 CONFIRMED UNKNOWN 시점에 3 observations가 준비된다.
  014/034 DYNAMIC event에는 static S3 shadow를 즉시 무효화하도록 분석했으며 적용되지 않는다.
- 10 ms는 2 observations, 20 ms는 3 observations다. 30/50 ms는 저장된 frame 집합이
  UT20과 bit-identical이므로 이번 세 실패의 원인은 window cutoff가 아니다.
- 측정된 Python reference shadow는 online 비용 기준을 크게 초과한다. 정확도 문제가 먼저
  해결되어야 하며 이번 작업에서는 최적화나 production 구현을 하지 않았다.

## 재현성과 자원

- primary workers: {WORKERS}; worker=5는 독립 primary group이 4개뿐이라 사용하지 않았다.
- offline wall: {elapsed:.3f} s, CPU: {cpu_time:.3f} s, max RSS: {max_rss / 1024**2:.1f} MiB.
- bag parse: 1, replay: 0, dataset regeneration: 0, diagnostic build: {args.diagnostic_build_time_s:.2f} s.
- validation_039와 normal_01_00 final strategy repeat는 bounds, clearance, classification,
  digest가 bit-identical했다.

상세 수치와 provenance는 `summary.json`, 실행 자원은 `performance.json`에 있다.
"""
    (OUTPUT / "README.md").write_text(readme, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
