"""Pure diagnostic helpers for sub-threshold static-safety evidence.

The module mirrors production adaptive-breakpoint fragmentation and pre-track
fragment merging, but it has no ROS publishers and cannot modify production
tracks, IDs, KFs, votes, or obstacle topics.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class Fragment:
    beam_indices: tuple[int, ...]
    points_xy: np.ndarray
    ranges_m: np.ndarray
    source_fragment_ids: tuple[int, ...]
    stage: str
    production_outcome: str

    @property
    def point_count(self) -> int:
        return len(self.beam_indices)

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        return (
            float(np.min(self.points_xy[:, 0])),
            float(np.max(self.points_xy[:, 0])),
            float(np.min(self.points_xy[:, 1])),
            float(np.max(self.points_xy[:, 1])),
        )

    @property
    def diagonal_m(self) -> float:
        x0, x1, y0, y1 = self.bounds
        return math.hypot(x1 - x0, y1 - y0)

    @property
    def local_spacing_mean_m(self) -> float:
        if self.point_count < 2:
            return 0.0
        ordered = self.points_xy[np.argsort(np.asarray(self.beam_indices))]
        return float(np.mean(np.linalg.norm(np.diff(ordered, axis=0), axis=1)))


@dataclass(frozen=True)
class Extraction:
    raw_fragments: tuple[Fragment, ...]
    accepted_clusters: tuple[Fragment, ...]
    rejected_candidates: tuple[Fragment, ...]


def _fragment(
    beams: Sequence[int], points: Sequence[Sequence[float]], ranges: Sequence[float],
    sources: Sequence[int], stage: str, outcome: str,
) -> Fragment:
    return Fragment(
        tuple(int(value) for value in beams),
        np.asarray(points, dtype=np.float64),
        np.asarray(ranges, dtype=np.float64),
        tuple(int(value) for value in sources),
        stage,
        outcome,
    )


def aabb_and_point_compatible(
    first: Fragment, second: Fragment, distance_m: float, maximum_diagonal_m: float
) -> bool:
    ax0, ax1, ay0, ay1 = first.bounds
    bx0, bx1, by0, by1 = second.bounds
    gap_x = max(0.0, ax0 - bx1, bx0 - ax1)
    gap_y = max(0.0, ay0 - by1, by0 - ay1)
    if gap_x * gap_x + gap_y * gap_y > distance_m * distance_m:
        return False
    delta = first.points_xy[:, None, :] - second.points_xy[None, :, :]
    if float(np.min(np.sum(delta * delta, axis=2))) > distance_m * distance_m:
        return False
    merged = np.vstack((first.points_xy, second.points_xy))
    diagonal = math.hypot(
        float(np.max(merged[:, 0]) - np.min(merged[:, 0])),
        float(np.max(merged[:, 1]) - np.min(merged[:, 1])),
    )
    return diagonal <= maximum_diagonal_m


def merge_fragments(
    fragments: Sequence[Fragment], distance_m: float, maximum_diagonal_m: float
) -> list[Fragment]:
    output = list(fragments)
    changed = True
    while changed:
        changed = False
        for first_index in range(len(output)):
            if changed:
                break
            for second_index in range(first_index + 1, len(output)):
                first, second = output[first_index], output[second_index]
                if not aabb_and_point_compatible(first, second, distance_m, maximum_diagonal_m):
                    continue
                order = np.argsort(np.asarray(first.beam_indices + second.beam_indices))
                beams = np.asarray(first.beam_indices + second.beam_indices)[order]
                points = np.vstack((first.points_xy, second.points_xy))[order]
                ranges = np.concatenate((first.ranges_m, second.ranges_m))[order]
                output[first_index] = _fragment(
                    beams, points, ranges,
                    first.source_fragment_ids + second.source_fragment_ids,
                    "POST_MERGE", "PENDING_FINAL_MIN_POINTS",
                )
                del output[second_index]
                changed = True
                break
    return output


def extract_production_fragments(
    ranges: np.ndarray,
    endpoints_xy: np.ndarray,
    *,
    angle_increment_rad: float,
    range_min_m: float,
    maximum_range_m: float,
    lambda_deg: float,
    cluster_sigma_m: float,
    minimum_two_point_distance_m: float,
    merge_enabled: bool,
    merge_min_fragment_points: int,
    merge_distance_m: float,
    minimum_cluster_points: int,
    maximum_obstacle_diagonal_m: float,
) -> Extraction:
    """Mirror production through the final point-count rejection boundary."""

    lambda_rad = math.radians(lambda_deg)
    denominator = math.sin(lambda_rad - angle_increment_rad)
    raw: list[Fragment] = []
    current_beams: list[int] = []
    current_points: list[np.ndarray] = []
    current_ranges: list[float] = []
    previous_point: np.ndarray | None = None
    previous_index = -1000

    def flush() -> None:
        nonlocal current_beams, current_points, current_ranges
        if current_beams:
            identifier = len(raw)
            raw.append(_fragment(
                current_beams, current_points, current_ranges, (identifier,),
                "RAW_ADAPTIVE_FRAGMENT", "PENDING_TEMPORARY_MIN_POINTS",
            ))
        current_beams, current_points, current_ranges = [], [], []

    for index, value in enumerate(np.asarray(ranges, dtype=np.float64)):
        range_m = float(value)
        if not math.isfinite(range_m) or range_m < range_min_m or range_m >= maximum_range_m:
            continue
        point = np.asarray(endpoints_xy[index], dtype=np.float64)
        same = False
        if previous_point is not None and index == previous_index + 1:
            maximum_break = 3.0 * cluster_sigma_m
            if denominator > 1.0e-6:
                maximum_break += range_m * math.sin(angle_increment_rad) / denominator
            same = float(np.linalg.norm(point - previous_point)) <= max(
                maximum_break, minimum_two_point_distance_m
            )
        if not same:
            flush()
        current_beams.append(index)
        current_points.append(point)
        current_ranges.append(range_m)
        previous_point = point
        previous_index = index
    flush()

    temporary_minimum = merge_min_fragment_points if merge_enabled else minimum_cluster_points
    singleton_or_early = [item for item in raw if item.point_count < temporary_minimum]
    merge_input = [item for item in raw if item.point_count >= temporary_minimum]
    merged = (
        merge_fragments(merge_input, merge_distance_m, maximum_obstacle_diagonal_m)
        if merge_enabled
        else merge_input
    )
    rejected = [
        _fragment(
            item.beam_indices, item.points_xy, item.ranges_m,
            item.source_fragment_ids, item.stage,
            "REJECTED_FINAL_MIN_CLUSTER_POINTS",
        )
        for item in merged if item.point_count < minimum_cluster_points
    ]
    rejected.extend(
        _fragment(
            item.beam_indices, item.points_xy, item.ranges_m,
            item.source_fragment_ids, item.stage,
            "REJECTED_TEMPORARY_FRAGMENT_MIN_POINTS",
        )
        for item in singleton_or_early
    )
    accepted = [
        _fragment(
            item.beam_indices, item.points_xy, item.ranges_m,
            item.source_fragment_ids, item.stage,
            "ACCEPTED_NORMAL_CLUSTER",
        )
        for item in merged if item.point_count >= minimum_cluster_points
    ]
    return Extraction(tuple(raw), tuple(accepted), tuple(rejected))


def compatible_with_identity(
    fragment: Fragment,
    identity_geometry: Sequence[Fragment],
    *,
    distance_m: float,
    maximum_diagonal_m: float,
) -> bool:
    return any(
        aabb_and_point_compatible(fragment, reference, distance_m, maximum_diagonal_m)
        for reference in identity_geometry
    )


def has_timestamp_persistence(
    fragment: Fragment,
    stamp_ns: int,
    history: Sequence[tuple[int, Fragment]],
    *,
    window_ns: int,
    distance_m: float,
    maximum_diagonal_m: float,
) -> bool:
    return any(
        prior_stamp < stamp_ns
        and stamp_ns - prior_stamp <= window_ns
        and aabb_and_point_compatible(fragment, prior, distance_m, maximum_diagonal_m)
        for prior_stamp, prior in history
    )


@dataclass
class ShadowIdentityState:
    physical_id: int
    evidence: list[tuple[int, Fragment]]
    invalidated_at_ns: int | None = None

    def observe(self, stamp_ns: int, fragment: Fragment) -> None:
        if self.invalidated_at_ns is None:
            self.evidence.append((stamp_ns, fragment))

    def update_motion(self, stamp_ns: int, motion_status: str) -> None:
        if motion_status == "DYNAMIC":
            self.evidence.clear()
            self.invalidated_at_ns = stamp_ns

    @property
    def active(self) -> bool:
        return self.invalidated_at_ns is None
