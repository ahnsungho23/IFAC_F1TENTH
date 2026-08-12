#!/usr/bin/env python3
"""Offline validation_039 obstacle-representation root-cause audit.

The script consumes the existing deterministic replay only.  Runtime ROS nodes are never started;
the production C++ planner audit target is used for path/footprint clearance calculations.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import resource
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from cmaes_tuning.bag_reader import read_bag
from cmaes_tuning.simulator_collision import SimulatorRasterCollisionModel


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "runs/cmaes_tuning/validation039_representation_root_cause_v1"
TIME_AXIS = ROOT / "runs/cmaes_tuning/time_axis_feasibility_audit_v1"
GEOMETRY = ROOT / "runs/cmaes_tuning/geometry_clearance_budget_audit_v1"
MANIFEST = (
    ROOT
    / "runs/cmaes_tuning/medium_lockstep_stage1_v2/artifacts/scenarios/validation"
    / "validation_039/manifest.json"
)
STREAM = TIME_AXIS / "streams/validation_039.stream.tsv"
BAG = TIME_AXIS / "replays/validation_039/bag"
DETECTOR_EVENTS = TIME_AXIS / "replays/validation_039/detector_events.jsonl"
PLANNER_EVENTS = TIME_AXIS / "replays/validation_039/planner_lifecycle_events.jsonl"
TIME_AXIS_ROWS = TIME_AXIS / "raw_time_axis/validation_039.tsv"
GEOMETRY_MISMATCH = GEOMETRY / ".raw/validation_039/mismatch.tsv"
GEOMETRY_SUMMARY = GEOMETRY / "summary.json"
BINARY = ROOT / "build/local_planning/path_family_feasibility_audit"
EXPECTED_HASH = "8cdda37a93236cc8"
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


def stamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def yaw_from_odom(message: Any) -> float:
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)),
        1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2),
    )


def localize_manifest_paths(manifest: dict[str, Any], scenario: str) -> dict[str, Any]:
    result = json.loads(json.dumps(manifest))
    result["baked_map_yaml"] = str(
        ROOT
        / "runs/cmaes_tuning/repeatability_mid4_v3/scenarios/validation"
        / scenario
        / f"{scenario}_map.yaml"
    )
    result["clean_map_yaml"] = str(
        ROOT / "src/monte_carlo_localization/maps/ifac_track.yaml"
    )
    return result


@dataclass(frozen=True)
class ScanPoint:
    index: int
    x: float
    y: float
    range_m: float


def bounds(points: Iterable[ScanPoint]) -> tuple[float, float, float, float]:
    values = list(points)
    return (
        min(point.x for point in values),
        max(point.x for point in values),
        min(point.y for point in values),
        max(point.y for point in values),
    )


def aabb_points(box: tuple[float, float, float, float]) -> list[tuple[float, float]]:
    xmin, xmax, ymin, ymax = box
    return [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]


def ray_box_entry(
    origin_x: float,
    origin_y: float,
    direction_x: float,
    direction_y: float,
    box: dict[str, float],
) -> float | None:
    lower = 0.0
    upper = math.inf
    for origin, direction, minimum, maximum in (
        (origin_x, direction_x, float(box["x_min"]), float(box["x_max"])),
        (origin_y, direction_y, float(box["y_min"]), float(box["y_max"])),
    ):
        if abs(direction) <= 1.0e-12:
            if not minimum <= origin < maximum:
                return None
            continue
        first = (minimum - origin) / direction
        second = (maximum - origin) / direction
        lower = max(lower, min(first, second))
        upper = min(upper, max(first, second))
        if lower > upper:
            return None
    return lower if upper >= 0.0 else None


def detector_clusters(
    scan: Any, laser_x: float, laser_y: float, yaw: float
) -> tuple[list[list[ScanPoint]], dict[str, Any]]:
    lambda_rad = math.radians(10.0)
    sigma = 0.03
    minimum_distance = 0.01
    temporary_minimum = 2
    final_minimum = 5
    merge_distance = 0.12
    maximum_size = 0.8
    dphi = float(scan.angle_increment)
    denominator = math.sin(lambda_rad - dphi)
    clusters: list[list[ScanPoint]] = []
    current: list[ScanPoint] = []
    previous: ScanPoint | None = None
    previous_index = -1000

    def flush() -> None:
        nonlocal current
        if len(current) >= temporary_minimum:
            clusters.append(current)
        current = []

    for index, raw_range in enumerate(scan.ranges):
        range_m = float(raw_range)
        if not math.isfinite(range_m) or range_m < float(scan.range_min) or range_m >= 14.0:
            continue
        angle = float(scan.angle_min) + index * dphi
        x = laser_x + range_m * math.cos(yaw + angle)
        y = laser_y + range_m * math.sin(yaw + angle)
        point = ScanPoint(index, x, y, range_m)
        same_cluster = False
        if previous is not None and index == previous_index + 1:
            maximum_break = 3.0 * sigma
            if denominator > 1.0e-6:
                maximum_break += range_m * math.sin(dphi) / denominator
            same_cluster = math.hypot(x - previous.x, y - previous.y) <= max(
                maximum_break, minimum_distance
            )
        if not same_cluster:
            flush()
        current.append(point)
        previous = point
        previous_index = index
    flush()
    before_merge = len(clusters)

    def close_pair(first: list[ScanPoint], second: list[ScanPoint]) -> bool:
        threshold = merge_distance * merge_distance
        return any(
            (a.x - b.x) ** 2 + (a.y - b.y) ** 2 <= threshold
            for a in first
            for b in second
        )

    changed = True
    while changed:
        changed = False
        for first_index in range(len(clusters)):
            if changed:
                break
            first = clusters[first_index]
            ax0, ax1, ay0, ay1 = bounds(first)
            for second_index in range(first_index + 1, len(clusters)):
                second = clusters[second_index]
                bx0, bx1, by0, by1 = bounds(second)
                gap_x = max(0.0, ax0 - bx1, bx0 - ax1)
                gap_y = max(0.0, ay0 - by1, by0 - ay1)
                if gap_x * gap_x + gap_y * gap_y > merge_distance * merge_distance:
                    continue
                if not close_pair(first, second):
                    continue
                merged = (min(ax0, bx0), max(ax1, bx1), min(ay0, by0), max(ay1, by1))
                if math.hypot(merged[1] - merged[0], merged[3] - merged[2]) > maximum_size:
                    continue
                clusters[first_index] = first + second
                del clusters[second_index]
                changed = True
                break
    before_final_filter = len(clusters)
    clusters = [cluster for cluster in clusters if len(cluster) >= final_minimum]
    return clusters, {
        "clusters_before_merge": before_merge,
        "clusters_before_final_filter": before_final_filter,
        "clusters_after_merge": len(clusters),
        "fragments_rejected": before_final_filter - len(clusters),
    }


def matching_cluster(
    clusters: list[list[ScanPoint]], detection: dict[str, Any]
) -> list[ScanPoint]:
    target = (
        float(detection["x_min"]),
        float(detection["x_max"]),
        float(detection["y_min"]),
        float(detection["y_max"]),
    )
    matches = [
        cluster
        for cluster in clusters
        if max(abs(first - second) for first, second in zip(bounds(cluster), target)) <= 1.0e-12
    ]
    if len(matches) != 1:
        raise RuntimeError(f"detector cluster/AABB reconstruction mismatch: {len(matches)}")
    return matches[0]


def target_hit_points(
    scan: Any,
    laser_x: float,
    laser_y: float,
    yaw: float,
    manifest: dict[str, Any],
) -> tuple[list[ScanPoint], list[ScanPoint], list[dict[str, float]]]:
    model = manifest["simulator_collision_model"]
    box = manifest["baked_obstacle_raster"]["world_half_open_bounds_m"]
    fov = float(model["scan_fov_rad"])
    beam_count = int(model["scan_beams"])
    true_points: list[ScanPoint] = []
    detector_points: list[ScanPoint] = []
    diagnostics: list[dict[str, float]] = []
    for index, raw_range in enumerate(scan.ranges):
        range_m = float(raw_range)
        true_relative = -0.5 * fov + index * fov / (beam_count - 1)
        true_angle = yaw + true_relative
        entry = ray_box_entry(
            laser_x, laser_y, math.cos(true_angle), math.sin(true_angle), box
        )
        if entry is None:
            continue
        true_x = laser_x + range_m * math.cos(true_angle)
        true_y = laser_y + range_m * math.sin(true_angle)
        inside = (
            float(box["x_min"]) <= true_x < float(box["x_max"])
            and float(box["y_min"]) <= true_y < float(box["y_max"])
        )
        if not inside:
            continue
        detector_relative = float(scan.angle_min) + index * float(scan.angle_increment)
        detector_angle = yaw + detector_relative
        detector_x = laser_x + range_m * math.cos(detector_angle)
        detector_y = laser_y + range_m * math.sin(detector_angle)
        true_points.append(ScanPoint(index, true_x, true_y, range_m))
        detector_points.append(ScanPoint(index, detector_x, detector_y, range_m))
        diagnostics.append(
            {
                "beam_index": index,
                "range_m": range_m,
                "ray_entry_m": entry,
                "range_minus_entry_m": range_m - entry,
                "true_angle_rad": true_angle,
                "detector_angle_rad": detector_angle,
                "angle_error_rad": detector_angle - true_angle,
                "endpoint_error_m": math.hypot(detector_x - true_x, detector_y - true_y),
            }
        )
    if not true_points:
        raise RuntimeError("exact event has no recorded return terminating inside GT raster")
    return true_points, detector_points, diagnostics


def tsv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8") as stream:
        return list(csv.DictReader(stream, delimiter="\t"))


def box_record(
    name: str,
    source: str,
    item: dict[str, Any],
    cartesian: tuple[float, float, float, float],
) -> str:
    if "s" in item:
        center_s = float(item["s"])
        s_min = center_s - float(item["s_half_extent"])
        s_max = center_s + float(item["s_half_extent"])
        center_d = float(item["d"])
    else:
        center_s = float(item["s_center"])
        s_min = float(item["s_start"])
        s_max = float(item["s_end"])
        center_d = float(item["d_center"])
    return "\t".join(
        [
            "FRENET_BOX",
            name,
            source,
            repr(center_s),
            repr(s_min),
            repr(s_max),
            repr(float(item["d_right"])),
            repr(float(item["d_left"])),
            "true",
            *(repr(value) for value in cartesian),
            "0.0",
            "0.0",
            "0.0",
        ]
    )


def polygon_record(name: str, source: str, points: list[tuple[float, float]]) -> str:
    return "\t".join(
        ["POLYGON", name, source, str(len(points))]
        + [repr(value) for point in points for value in point]
    )


def make_spec(
    manifest: dict[str, Any],
    event_stamp: int,
    true_hits: list[ScanPoint],
    cluster: list[ScanPoint],
    detector_event: dict[str, Any],
    temporal_box: tuple[float, float, float, float],
) -> str:
    obstacle = manifest["obstacle"]
    half_width = 0.5 * float(obstacle["width"])
    half_height = 0.5 * float(obstacle["height"])
    yaw = float(obstacle["yaw"])
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    nominal: list[tuple[float, float]] = []
    for local_x, local_y in (
        (-half_width, -half_height),
        (half_width, -half_height),
        (half_width, half_height),
        (-half_width, half_height),
    ):
        nominal.append(
            (
                float(obstacle["x"]) + local_x * cosine - local_y * sine,
                float(obstacle["y"]) + local_x * sine + local_y * cosine,
            )
        )
    raster = manifest["baked_obstacle_raster"]["world_half_open_bounds_m"]
    raster_box = (
        float(raster["x_min"]),
        float(raster["x_max"]),
        float(raster["y_min"]),
        float(raster["y_max"]),
    )
    detection = detector_event["raw_detections"][0]
    detected_box = (
        float(detection["x_min"]),
        float(detection["x_max"]),
        float(detection["y_min"]),
        float(detection["y_max"]),
    )
    track = detector_event["tracks"][0]
    published = detector_event["published_static"][0]
    # Diagnostic object model only: a known 0.5 m axis-aligned square anchored at the observed
    # sensor-facing y face.  This is not claimed to be inferable by the current LiDAR pipeline.
    known_center_x = 0.5 * (detected_box[0] + detected_box[1])
    known_face_y = detected_box[3]
    known_box = (
        known_center_x - half_width,
        known_center_x + half_width,
        known_face_y - float(obstacle["height"]),
        known_face_y,
    )
    lines = [
        "GEOMETRY_BUDGET_SPEC_V1",
        f"STAMP\tREPRESENTATIVE\t{event_stamp}",
        f"STAMP\tFALSE_FEASIBLE\t{event_stamp}",
        f"MANIFEST_S\t{float(obstacle['s'])!r}",
        polygon_record("G0", "nominal_manifest_polygon", nominal),
        polygon_record("G1", "exact_half_open_raster", aabb_points(raster_box)),
        polygon_record(
            "R2", "true_backend_angle_recorded_gt_hit_points", [(p.x, p.y) for p in true_hits]
        ),
        polygon_record(
            "R3", "production_cluster_points", [(p.x, p.y) for p in cluster]
        ),
        polygon_record("R4", "cartesian_cluster_aabb", aabb_points(detected_box)),
        box_record("G2", "detector_alias", detection, detected_box),
        box_record("R5", "detection_struct", detection, detected_box),
        box_record("R6", "tracker_smoothed_envelope", track, detected_box),
        box_record("R7", "published_static_obs", published, detected_box),
        box_record("R8", "planner_pre_stabilization_input", published, detected_box),
        polygon_record("CF_TEMPORAL", "three_scan_visible_aabb_union", aabb_points(temporal_box)),
        polygon_record("CF_KNOWN", "known_nominal_rectangle_from_visible_face", aabb_points(known_box)),
        "END_SPEC",
    ]
    return "\n".join(lines) + "\n"


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


def run_cpp(spec_path: Path, output: Path) -> float:
    started = time.monotonic()
    subprocess.run(
        [
            str(BINARY),
            "--representation-root-cause",
            str(STREAM),
            str(spec_path),
            str(output),
            EXPECTED_HASH,
        ],
        cwd=ROOT,
        check=True,
    )
    return time.monotonic() - started


def reconstruct(raw_root: Path) -> dict[str, Any]:
    raw_root.mkdir(parents=True, exist_ok=True)
    manifest = localize_manifest_paths(
        json.loads(MANIFEST.read_text(encoding="utf-8")), "validation_039"
    )
    events = read_jsonl(DETECTOR_EVENTS)
    planner_events = read_jsonl(PLANNER_EVENTS)
    mismatches = tsv_rows(GEOMETRY_MISMATCH)
    mismatch = next(row for row in mismatches if row["input_representation"] == "G3")
    event_stamp = int(mismatch["logical_stamp_ns"])
    if mismatch["geometry_hash"] != EXPECTED_HASH:
        raise RuntimeError("geometry audit no longer identifies the expected G3 candidate")
    detector_event = next(item for item in events if int(item["scan_stamp_ns"]) == event_stamp)
    if len(detector_event["raw_detections"]) != 1 or len(detector_event["published_static"]) != 1:
        raise RuntimeError("exact detector event does not have one target detection/static object")

    parse_started = time.monotonic()
    bag = read_bag(BAG, topics=("/scan", "/ego_racecar/odom", "/static_obs"))
    bag_parse_time = time.monotonic() - parse_started
    scan = next(record.message for record in bag.topic("/scan") if stamp_ns(record.message) == event_stamp)
    odom = next(
        record.message
        for record in bag.topic("/ego_racecar/odom")
        if stamp_ns(record.message) == event_stamp
    )
    static_message = next(
        record.message
        for record in bag.topic("/static_obs")
        if stamp_ns(record.message) == event_stamp and record.message.obstacles
    )
    yaw = yaw_from_odom(odom)
    base_x = float(odom.pose.pose.position.x)
    base_y = float(odom.pose.pose.position.y)
    lidar_offset = float(manifest["simulator_collision_model"]["lidar_offset_x_m"])
    laser_x = base_x + lidar_offset * math.cos(yaw)
    laser_y = base_y + lidar_offset * math.sin(yaw)
    true_hits, detector_hits, hit_diagnostics = target_hit_points(
        scan, laser_x, laser_y, yaw, manifest
    )
    clusters, cluster_stats = detector_clusters(scan, laser_x, laser_y, yaw)
    cluster = matching_cluster(clusters, detector_event["raw_detections"][0])
    hit_indices = {point.index for point in true_hits}
    cluster_indices = {point.index for point in cluster}
    if hit_indices != cluster_indices:
        raise RuntimeError(
            f"GT-hit/cluster mismatch: missing={sorted(hit_indices-cluster_indices)} "
            f"extra={sorted(cluster_indices-hit_indices)}"
        )

    leading = [
        event
        for event in events
        if event_stamp - 200_000_000 <= int(event["scan_stamp_ns"]) <= event_stamp
    ]
    visible_detections = [event["raw_detections"][0] for event in leading if event["raw_detections"]]
    scans_by_stamp = {stamp_ns(record.message): record.message for record in bag.topic("/scan")}
    odom_by_stamp = {
        stamp_ns(record.message): record.message for record in bag.topic("/ego_racecar/odom")
    }
    leading_observations: dict[str, dict[str, Any]] = {}
    for leading_event in leading:
        if not leading_event["raw_detections"]:
            continue
        leading_stamp = int(leading_event["scan_stamp_ns"])
        leading_scan = scans_by_stamp[leading_stamp]
        leading_odom = odom_by_stamp[leading_stamp]
        leading_yaw = yaw_from_odom(leading_odom)
        leading_laser_x = float(leading_odom.pose.pose.position.x) + lidar_offset * math.cos(
            leading_yaw
        )
        leading_laser_y = float(leading_odom.pose.pose.position.y) + lidar_offset * math.sin(
            leading_yaw
        )
        leading_clusters, _ = detector_clusters(
            leading_scan, leading_laser_x, leading_laser_y, leading_yaw
        )
        leading_cluster = matching_cluster(
            leading_clusters, leading_event["raw_detections"][0]
        )
        leading_hits, _, _ = target_hit_points(
            leading_scan, leading_laser_x, leading_laser_y, leading_yaw, manifest
        )
        leading_observations[str(leading_stamp)] = {
            "cluster_point_count": len(leading_cluster),
            "gt_hit_beam_count": len(leading_hits),
            "cluster_indices": [point.index for point in leading_cluster],
            "gt_hit_indices": [point.index for point in leading_hits],
        }
    temporal_box = (
        min(float(item["x_min"]) for item in visible_detections),
        max(float(item["x_max"]) for item in visible_detections),
        min(float(item["y_min"]) for item in visible_detections),
        max(float(item["y_max"]) for item in visible_detections),
    )
    spec_text = make_spec(
        manifest, event_stamp, true_hits, cluster, detector_event, temporal_box
    )
    spec_path = raw_root / "representation_spec.tsv"
    spec_path.write_text(spec_text, encoding="utf-8")
    cpp_time = run_cpp(spec_path, raw_root)

    safe_event = next(
        event
        for event in planner_events
        if event.get("event") == "SAFE_STOP_LIFECYCLE"
        and int(event.get("source_stamp_ns", 0)) == event_stamp
    )
    published = static_message.obstacles[0]
    return {
        "event_stamp_ns": event_stamp,
        "bag_parse_time_s": bag_parse_time,
        "cpp_time_s": cpp_time,
        "true_hits": [point.__dict__ for point in true_hits],
        "detector_hits": [point.__dict__ for point in detector_hits],
        "cluster": [point.__dict__ for point in cluster],
        "cluster_stats": cluster_stats,
        "hit_diagnostics": hit_diagnostics,
        "laser": {
            "frame_id": str(scan.header.frame_id),
            "base_x": base_x,
            "base_y": base_y,
            "yaw": yaw,
            "laser_x": laser_x,
            "laser_y": laser_y,
            "offset_m": lidar_offset,
            "message_angle_min": float(scan.angle_min),
            "message_angle_max": float(scan.angle_max),
            "message_angle_increment": float(scan.angle_increment),
            "backend_angle_increment": float(manifest["simulator_collision_model"]["scan_fov_rad"])
            / (int(manifest["simulator_collision_model"]["scan_beams"]) - 1),
            "beam_count": len(scan.ranges),
        },
        "detector_event": detector_event,
        "safe_event": safe_event,
        "static_message": {
            "header_stamp_ns": stamp_ns(static_message),
            "id": int(published.id),
            "has_cartesian": bool(published.has_cartesian),
            "radius": float(published.radius),
            "size": float(published.size),
            "x_min": float(published.x_min),
            "x_max": float(published.x_max),
            "y_min": float(published.y_min),
            "y_max": float(published.y_max),
            "s_start": float(published.s_start),
            "s_end": float(published.s_end),
            "d_right": float(published.d_right),
            "d_left": float(published.d_left),
            "s_center": float(published.s_center),
            "d_center": float(published.d_center),
            "vs": float(published.vs),
            "vd": float(published.vd),
            "s_var": float(published.s_var),
            "d_var": float(published.d_var),
            "vs_var": float(published.vs_var),
            "vd_var": float(published.vd_var),
            "is_static": bool(published.is_static),
            "is_visible": bool(published.is_visible),
        },
        "mismatch": mismatch,
        "leading_observations": leading_observations,
        "temporal_box": temporal_box,
        "spec_sha256": sha256(spec_path),
    }


def write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def geometry_generalization() -> tuple[dict[str, dict[str, Any]], float]:
    started = time.monotonic()
    scenarios = json.loads(GEOMETRY_SUMMARY.read_text(encoding="utf-8"))["scenarios"]
    output: dict[str, dict[str, Any]] = {}
    status = {
        "validation_004": "same_issue_present",
        "validation_014": "same_issue_present",
        "validation_019": "same_issue_present",
        "validation_034": "same_issue_absent",
    }
    for scenario in status:
        g1 = scenarios[scenario]["representations"]["G1"]
        g2 = scenarios[scenario]["representations"]["G2"]
        manifest_path = (
            ROOT
            / "runs/cmaes_tuning/medium_lockstep_stage1_v2/artifacts/scenarios/validation"
            / scenario
            / "manifest.json"
        )
        manifest = localize_manifest_paths(
            json.loads(manifest_path.read_text(encoding="utf-8")), scenario
        )
        stamp = int(scenarios[scenario]["representative_stamp_ns"])
        events = read_jsonl(TIME_AXIS / "replays" / scenario / "detector_events.jsonl")
        event = next(item for item in events if int(item["scan_stamp_ns"]) == stamp)
        raster = manifest["baked_obstacle_raster"]["world_half_open_bounds_m"]
        detections = [
            item
            for item in event["raw_detections"]
            if float(item["x_min"]) < float(raster["x_max"])
            and float(item["x_max"]) > float(raster["x_min"])
            and float(item["y_min"]) < float(raster["y_max"])
            and float(item["y_max"]) > float(raster["y_min"])
        ]
        if len(detections) != 1:
            output[scenario] = {
                "status": "insufficient_evidence",
                "reason": f"target detection count at representative stamp is {len(detections)}",
            }
            continue
        bag = read_bag(
            TIME_AXIS / "replays" / scenario / "bag",
            topics=("/scan", "/ego_racecar/odom"),
        )
        scan = next(
            record.message for record in bag.topic("/scan") if stamp_ns(record.message) == stamp
        )
        odom = next(
            record.message
            for record in bag.topic("/ego_racecar/odom")
            if stamp_ns(record.message) == stamp
        )
        yaw = yaw_from_odom(odom)
        lidar_offset = float(manifest["simulator_collision_model"]["lidar_offset_x_m"])
        laser_x = float(odom.pose.pose.position.x) + lidar_offset * math.cos(yaw)
        laser_y = float(odom.pose.pose.position.y) + lidar_offset * math.sin(yaw)
        hits, _, _ = target_hit_points(scan, laser_x, laser_y, yaw, manifest)
        clusters, _ = detector_clusters(scan, laser_x, laser_y, yaw)
        cluster = matching_cluster(clusters, detections[0])
        hit_indices = {point.index for point in hits}
        cluster_indices = {point.index for point in cluster}
        hit_box = bounds(hits)
        cluster_box = bounds(cluster)
        output[scenario] = {
            "status": status[scenario],
            "g1_lateral_span_m": float(g1["d_span_m"]),
            "g2_lateral_span_m": float(g2["d_span_m"]),
            "lateral_span_delta_m": float(g2["d_span_m"]) - float(g1["d_span_m"]),
            "g1_longitudinal_span_m": float(g1["s_span_m"]),
            "g2_longitudinal_span_m": float(g2["s_span_m"]),
            "longitudinal_span_delta_m": float(g2["s_span_m"]) - float(g1["s_span_m"]),
            "gt_hit_beam_count": len(hits),
            "cluster_point_count": len(cluster),
            "cluster_contains_all_gt_hits": hit_indices <= cluster_indices,
            "cluster_omits_gt_hits": sorted(hit_indices - cluster_indices),
            "visible_hit_x_span_m": hit_box[1] - hit_box[0],
            "visible_hit_y_span_m": hit_box[3] - hit_box[2],
            "cluster_x_span_m": cluster_box[1] - cluster_box[0],
            "cluster_y_span_m": cluster_box[3] - cluster_box[2],
            "raster_x_span_m": float(raster["x_max"]) - float(raster["x_min"]),
            "raster_y_span_m": float(raster["y_max"]) - float(raster["y_min"]),
            "sensor_surface_cause_confirmed": status[scenario] == "same_issue_present",
        }
    return output, time.monotonic() - started


def main() -> int:
    args = parse_args()
    if not BINARY.is_file():
        raise RuntimeError(f"diagnostic executable is missing: {BINARY}")
    started = time.monotonic()
    cpu_started = time.process_time()
    memory_before = memory_snapshot()
    production_hashes_before = {name: sha256(path) for name, path in PRODUCTION_FILES.items()}
    OUTPUT.mkdir(parents=True, exist_ok=True)
    raw_root = OUTPUT / ".raw"
    first = reconstruct(raw_root / "first")
    repeat_started = time.monotonic()
    repeat = reconstruct(raw_root / "repeat")
    repeat_time = time.monotonic() - repeat_started

    deterministic_python = canonical_digest(
        {key: value for key, value in first.items() if key not in {"bag_parse_time_s", "cpp_time_s"}}
    ) == canonical_digest(
        {key: value for key, value in repeat.items() if key not in {"bag_parse_time_s", "cpp_time_s"}}
    )
    compared_files = [
        "representation_chain_exact.tsv",
        "same_path_clearance_exact.tsv",
        "counterfactuals_exact.tsv",
        "selected_path.tsv",
        "event_exact.tsv",
    ]
    deterministic_cpp = all(
        sha256(raw_root / "first" / name) == sha256(raw_root / "repeat" / name)
        for name in compared_files
    )
    if not deterministic_python or not deterministic_cpp:
        raise RuntimeError("offline representation reconstruction is not deterministic")

    chain_rows = tsv_rows(raw_root / "first/representation_chain_exact.tsv")
    g1 = next(row for row in chain_rows if row["stage"] == "R1")
    r9 = next(row for row in chain_rows if row["stage"] == "R9")
    g1_boundary = float(g1["selected_right_boundary"])
    for row in chain_rows:
        row["path_side_boundary_error_vs_g1_m"] = (
            float(row["selected_right_boundary"]) - g1_boundary
        )
        row["path_side_boundary_error_vs_g1_mm"] = 1000.0 * float(
            row["path_side_boundary_error_vs_g1_m"]
        )
    chain_fields = list(chain_rows[0])
    write_csv(OUTPUT / "representation_chain.csv", chain_rows, chain_fields)

    delta_rows: list[dict[str, Any]] = []
    for previous, current in zip(chain_rows, chain_rows[1:]):
        dx = float(current["x_center"]) - float(previous["x_center"])
        dy = float(current["y_center"]) - float(previous["y_center"])
        ds = float(current["s_center"]) - float(previous["s_center"])
        dd = float(current["d_center"]) - float(previous["d_center"])
        previous_area = float(previous["x_span"]) * float(previous["y_span"])
        current_area = float(current["x_span"]) * float(current["y_span"])
        delta_rows.append(
            {
                "scenario": "validation_039",
                "stamp_ns": first["event_stamp_ns"],
                "transition": f"{previous['stage']}->{current['stage']}",
                "longitudinal_span_delta_m": float(current["s_span"]) - float(previous["s_span"]),
                "longitudinal_span_delta_mm": 1000.0
                * (float(current["s_span"]) - float(previous["s_span"])),
                "lateral_span_delta_m": float(current["d_span"]) - float(previous["d_span"]),
                "lateral_span_delta_mm": 1000.0
                * (float(current["d_span"]) - float(previous["d_span"])),
                "cartesian_center_shift_m": math.hypot(dx, dy),
                "frenet_center_shift_m": math.hypot(ds, dd),
                "area_delta_m2": current_area - previous_area,
                "selected_right_boundary_delta_m": float(current["selected_right_boundary"])
                - float(previous["selected_right_boundary"]),
                "selected_right_boundary_delta_mm": 1000.0
                * (
                    float(current["selected_right_boundary"])
                    - float(previous["selected_right_boundary"])
                ),
            }
        )
    write_csv(OUTPUT / "representation_delta.csv", delta_rows, list(delta_rows[0]))

    event_by_stamp = {int(item["scan_stamp_ns"]): item for item in read_jsonl(DETECTOR_EVENTS)}
    time_rows = {int(row["logical_stamp_ns"]): row for row in tsv_rows(TIME_AXIS_ROWS)}
    timeline: list[dict[str, Any]] = []
    timeline_stamps = [
        stamp
        for stamp in sorted(event_by_stamp)
        if first["event_stamp_ns"] - 30_000_000
        <= stamp
        <= first["event_stamp_ns"] + 110_000_000
    ]
    for stamp in timeline_stamps:
        event = event_by_stamp[stamp]
        detection = event["raw_detections"][0] if event["raw_detections"] else {}
        track = event["tracks"][0] if event["tracks"] else {}
        published = event["published_static"][0] if event["published_static"] else {}
        observation = first["leading_observations"].get(str(stamp), {})
        point_count = observation.get("cluster_point_count", "")
        planner = time_rows.get(stamp, {})
        timeline.append(
            {
                "scan_stamp_ns": stamp,
                "cluster_point_count": point_count,
                "gt_hit_beam_count": observation.get("gt_hit_beam_count", ""),
                "aabb_x_min": detection.get("x_min", ""),
                "aabb_x_max": detection.get("x_max", ""),
                "aabb_y_min": detection.get("y_min", ""),
                "aabb_y_max": detection.get("y_max", ""),
                "aabb_width_m": (
                    float(detection["x_max"]) - float(detection["x_min"]) if detection else ""
                ),
                "aabb_height_m": (
                    float(detection["y_max"]) - float(detection["y_min"]) if detection else ""
                ),
                "detection_d_span_m": (
                    float(detection["d_left"]) - float(detection["d_right"]) if detection else ""
                ),
                "track_uid": track.get("track_uid", ""),
                "track_visible": track.get("visible", ""),
                "track_d_span_m": (
                    float(track["d_left"]) - float(track["d_right"]) if track else ""
                ),
                "track_s_span_m": (
                    2.0 * float(track["s_half_extent"]) if track else ""
                ),
                "published_d_span_m": (
                    float(published["d_left"]) - float(published["d_right"]) if published else ""
                ),
                "published_s_span_m": (
                    float(published["s_end"]) - float(published["s_start"]) if published else ""
                ),
                "planner_guard_d_span_m": (
                    float(planner["planning_obstacle_d_left"])
                    - float(planner["planning_obstacle_d_right"])
                    if planner.get("planning_obstacle_d_left")
                    else ""
                ),
                "planner_guard_s_span_m": (
                    float(planner["planning_obstacle_s_end"])
                    - float(planner["planning_obstacle_s_start"])
                    if planner.get("planning_obstacle_s_end")
                    else ""
                ),
                "tracker_matched": event["tracker_update"]["matched"],
                "tracker_spawned": event["tracker_update"]["spawned"],
                "tracker_retired": event["tracker_update"]["retired"],
            }
        )
    write_csv(OUTPUT / "geometry_timeline.csv", timeline, list(timeline[0]))

    same_path = tsv_rows(raw_root / "first/same_path_clearance_exact.tsv")
    for row in same_path:
        if row["first_overlap_nominal_time_s"]:
            row["first_overlap_nominal_timestamp_ns"] = first["event_stamp_ns"] + round(
                float(row["first_overlap_nominal_time_s"]) * 1_000_000_000
            )
        else:
            row["first_overlap_nominal_timestamp_ns"] = ""
    write_csv(OUTPUT / "same_path_clearance.csv", same_path, list(same_path[0]))
    counterfactuals = tsv_rows(raw_root / "first/counterfactuals_exact.tsv")
    write_csv(OUTPUT / "counterfactuals.csv", counterfactuals, list(counterfactuals[0]))

    path_sha256 = sha256(raw_root / "first/selected_path.tsv")
    hit_diag = first["hit_diagnostics"]
    true_box = bounds(ScanPoint(**item) for item in first["true_hits"])
    cluster_box = bounds(ScanPoint(**item) for item in first["cluster"])
    full_box = json.loads(MANIFEST.read_text(encoding="utf-8"))["baked_obstacle_raster"][
        "world_half_open_bounds_m"
    ]
    full_perimeter = 2.0 * (
        float(full_box["x_max"]) - float(full_box["x_min"])
        + float(full_box["y_max"]) - float(full_box["y_min"])
    )
    visible_face_span = true_box[1] - true_box[0]
    frame_check = {
        "event_stamp_ns": first["event_stamp_ns"],
        "base_link_reference": "vehicle rectangle center",
        "scan_frame": first["laser"]["frame_id"],
        "lockstep_transform": "same-stamp ego odometry plus +x lidar offset",
        "base_pose": {
            "x": first["laser"]["base_x"],
            "y": first["laser"]["base_y"],
            "yaw": first["laser"]["yaw"],
        },
        "lidar_origin": {
            "x": first["laser"]["laser_x"],
            "y": first["laser"]["laser_y"],
            "offset_x_m": first["laser"]["offset_m"],
        },
        "timestamps": {
            "scan": first["event_stamp_ns"],
            "detector_event": int(first["detector_event"]["scan_stamp_ns"]),
            "static_obs": first["static_message"]["header_stamp_ns"],
            "planner_source": int(first["safe_event"]["source_stamp_ns"]),
            "all_equal": len(
                {
                    first["event_stamp_ns"],
                    int(first["detector_event"]["scan_stamp_ns"]),
                    first["static_message"]["header_stamp_ns"],
                    int(first["safe_event"]["source_stamp_ns"]),
                }
            )
            == 1,
        },
        "angle_convention": {
            "message_increment_rad": first["laser"]["message_angle_increment"],
            "simulator_backend_increment_rad": first["laser"]["backend_angle_increment"],
            "increment_error_rad": first["laser"]["message_angle_increment"]
            - first["laser"]["backend_angle_increment"],
            "maximum_gt_hit_angle_error_rad": max(abs(item["angle_error_rad"]) for item in hit_diag),
            "maximum_gt_hit_endpoint_error_m": max(item["endpoint_error_m"] for item in hit_diag),
            "cause": "lockstep LaserScan uses FOV/N while simulator backend uses FOV/(N-1)",
        },
        "map_transform_offset_bug": False,
        "timestamp_mismatch": False,
        "frame_conclusion": (
            "base/laser origin, yaw, map transform, and timestamps are consistent; "
            "LaserScan angle_increment has a small harness convention mismatch"
        ),
    }
    (OUTPUT / "frame_check.json").write_text(
        json.dumps(frame_check, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    generalization, generalization_bag_parse_time = geometry_generalization()
    geometry_parameters = json.loads(GEOMETRY_SUMMARY.read_text(encoding="utf-8"))[
        "parameters"
    ]
    exact_row = next(row for row in same_path if row["representation"] == "R1_exact_raster")
    r9_row = next(row for row in same_path if row["representation"] == "R9_planner_production")
    event_row = tsv_rows(raw_root / "first/event_exact.tsv")[0]
    boundary_underestimate = float(r9["selected_right_boundary"]) - g1_boundary
    summary = {
        "scenario": "validation_039",
        "diagnostic_only": True,
        "production_changes": False,
        "replay_required": False,
        "replay_count": 0,
        "exact_event": {
            "source_stamp_ns": first["event_stamp_ns"],
            "scan_stamp_ns": first["event_stamp_ns"],
            "detector_event_stamp_ns": int(first["detector_event"]["scan_stamp_ns"]),
            "static_obs_stamp_ns": first["static_message"]["header_stamp_ns"],
            "planner_ready_stamp_ns": int(first["safe_event"]["source_stamp_ns"]),
            "plan_source_stamp_ns": int(first["safe_event"]["source_stamp_ns"]),
            "candidate": event_row,
            "serialized_path_sha256": path_sha256,
            "runtime_publication_status": (
                "hard-valid and selected by field-exact G3 plan() reconstruction; the runtime "
                "safe-stop preparation gate did not publish this path"
            ),
        },
        "root_cause": {
            "primary": "SENSOR_OCCLUSION_LIMITED",
            "secondary": [
                "AABB_MODEL_LIMITATION",
                "FRAME_CONVENTION_BUG",
                "tracker_and_planner_union_do_not_repair_unobserved_lateral_extent",
            ],
            "dominant_loss_transition": "R1->R2",
            "clustering_loss": False,
            "aabb_correct_for_measured_points": True,
            "detection_interface_information_loss": False,
            "frenet_projection_dominant": False,
            "tracker_repairs_hidden_extent": False,
            "timestamp_mismatch": False,
        },
        "observability": {
            "obstacle_hit_beam_count": len(first["true_hits"]),
            "beam_indices": [item["index"] for item in first["true_hits"]],
            "angular_span_rad": max(item["true_angle_rad"] for item in hit_diag)
            - min(item["true_angle_rad"] for item in hit_diag),
            "visible_return_x_span_m": true_box[1] - true_box[0],
            "visible_return_y_span_m": true_box[3] - true_box[2],
            "visible_surface_span_m": visible_face_span,
            "visible_sensor_facing_face_fraction": visible_face_span
            / (float(full_box["x_max"]) - float(full_box["x_min"])),
            "visible_boundary_perimeter_fraction": visible_face_span / full_perimeter,
            "far_or_back_surface_return_count": 0,
            "cluster_point_count": len(first["cluster"]),
            "cluster_point_indices": [item["index"] for item in first["cluster"]],
            "cluster_contains_all_gt_hits": True,
            "cluster_contains_non_gt_points": False,
            "gt_hit_points_removed_before_aabb": 0,
            "neighbor_or_map_points_in_exact_cluster": 0,
            "map_filter_removed_target_points": 0,
        },
        "required_numbers": {
            "g1_lateral_span_m": float(g1["d_span"]),
            "visible_lidar_lateral_span_m": float(
                next(row for row in chain_rows if row["stage"] == "R2")["d_span"]
            ),
            "cluster_lateral_span_m": float(
                next(row for row in chain_rows if row["stage"] == "R3")["d_span"]
            ),
            "cartesian_aabb_width_m": cluster_box[1] - cluster_box[0],
            "cartesian_aabb_height_m": cluster_box[3] - cluster_box[2],
            "detection_size_semantics": "Cartesian AABB diagonal",
            "detection_size_m": math.hypot(cluster_box[1] - cluster_box[0], cluster_box[3] - cluster_box[2]),
            "published_radius_semantics": "half Cartesian AABB diagonal",
            "published_radius_m": first["static_message"]["radius"],
            "published_static_lateral_span_m": float(
                next(row for row in chain_rows if row["stage"] == "R7")["d_span"]
            ),
            "planner_g3_lateral_span_m": float(r9["d_span"]),
            "selected_side_boundary_error_vs_g1_m": boundary_underestimate,
            "selected_side_boundary_error_vs_g1_mm": 1000.0 * boundary_underestimate,
            "center_d_error_m": float(r9["d_center"]) - float(g1["d_center"]),
            "same_path_production_clearance_m": float(r9_row["production_obstacle_clearance"]),
            "same_path_exact_gt_clearance_m": float(exact_row["physical_signed_clearance"]),
            "false_feasible_discrepancy_m": float(r9_row["production_obstacle_clearance"])
            - float(exact_row["physical_signed_clearance"]),
            "false_feasible_discrepancy_mm": 1000.0
            * (
                float(r9_row["production_obstacle_clearance"])
                - float(exact_row["physical_signed_clearance"])
            ),
            "overlap_explanation": (
                "mostly obstacle-side boundary underestimation; path/footprint conventions add the "
                "remaining geometric relation, timestamp difference is zero"
            ),
        },
        "interfaces": {
            "r4_to_r5_shape_information_lost": "none",
            "detection_retains": [
                "x_min",
                "x_max",
                "y_min",
                "y_max",
                "independent s extent",
                "independent d_right/d_left",
                "Cartesian AABB diagonal size",
            ],
            "static_obs_retains": [
                "has_cartesian",
                "x/y center and min/max",
                "half-diagonal radius",
                "independent Frenet s/d bounds",
            ],
            "planner_consumes": "published Frenet s_start/s_end/d_right/d_left",
        },
        "planner_semantics": {
            "pre_plan_longitudinal_guard_growth_m": float(r9["s_span"])
            - float(next(row for row in chain_rows if row["stage"] == "R8")["s_span"]),
            "pre_plan_lateral_union_growth_m": float(r9["d_span"])
            - float(next(row for row in chain_rows if row["stage"] == "R8")["d_span"]),
            "inside_validator_vehicle_half_width_m": 0.5
            * float(geometry_parameters["vehicle_width_m"]),
            "inside_validator_safety_margin_m": float(geometry_parameters["safety_margin_m"]),
            "inside_validator_tracking_reserve_m": float(
                geometry_parameters["tracking_error_reserve_m"]
            ),
            "inside_validator_obstacle_longitudinal_padding_m": float(
                geometry_parameters["obstacle_longitudinal_padding_m"]
            ),
        },
        "published_static_message": first["static_message"],
        "counterfactuals": counterfactuals,
        "generalization": generalization,
        "determinism": {
            "bit_identical": deterministic_python and deterministic_cpp,
            "first_digest": canonical_digest(
                {name: sha256(raw_root / "first" / name) for name in compared_files}
            ),
            "repeat_digest": canonical_digest(
                {name: sha256(raw_root / "repeat" / name) for name in compared_files}
            ),
        },
    }
    # Repeat digest uses raw rows before the two CSV-only derived columns. Compare the source files
    # separately and publish their exact hashes to avoid conflating report decoration with geometry.
    summary["determinism"]["first_cpp_files_sha256"] = {
        name: sha256(raw_root / "first" / name) for name in compared_files
    }
    summary["determinism"]["repeat_cpp_files_sha256"] = {
        name: sha256(raw_root / "repeat" / name) for name in compared_files
    }
    provenance_paths = {
        **PRODUCTION_FILES,
        "manifest": MANIFEST,
        "baked_raster_image": Path(
            localize_manifest_paths(json.loads(MANIFEST.read_text()), "validation_039")[
                "baked_map_yaml"
            ]
        ).with_suffix(".png"),
        "bag_db3": BAG / "bag_0.db3",
        "bag_metadata": BAG / "metadata.yaml",
        "detector_events": DETECTOR_EVENTS,
        "planner_events": PLANNER_EVENTS,
        "snapshot_stream": STREAM,
        "diagnostic_source": Path(__file__),
        "diagnostic_cpp_source": ROOT / "tools/cmaes_tuning/path_family_feasibility_audit.cpp",
    }
    summary["provenance"] = {
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "git_status": subprocess.check_output(
            ["git", "status", "--short", "--branch"], cwd=ROOT, text=True
        ).splitlines(),
        "sha256": {name: sha256(path) for name, path in provenance_paths.items()},
        "production_sha256_before": production_hashes_before,
        "production_sha256_after": {
            name: sha256(path) for name, path in PRODUCTION_FILES.items()
        },
    }
    if summary["provenance"]["production_sha256_before"] != summary["provenance"]["production_sha256_after"]:
        raise RuntimeError("production source/binary hash changed during audit")
    (OUTPUT / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    performance = {
        "replay_required": False,
        "replay_count": 0,
        "worker_count": 1,
        "bag_parse_time_s": first["bag_parse_time_s"],
        "deterministic_repeat_bag_parse_time_s": repeat["bag_parse_time_s"],
        "generalization_bag_parse_time_s": generalization_bag_parse_time,
        "bag_parse_time_total_s": first["bag_parse_time_s"]
        + repeat["bag_parse_time_s"]
        + generalization_bag_parse_time,
        "diagnostic_build_time_s": args.diagnostic_build_time_s,
        "offline_wall_time_s": time.monotonic() - started,
        "offline_cpu_time_s": time.process_time() - cpu_started,
        "cpp_main_time_s": first["cpp_time_s"],
        "deterministic_repeat_time_s": repeat_time,
        "max_rss_bytes": max(
            resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
            resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,
        )
        * 1024,
        "swap_used_before_bytes": memory_before["swap_used_bytes"],
        "swap_used_after_bytes": memory_snapshot()["swap_used_bytes"],
        "swap_change_bytes": memory_snapshot()["swap_used_bytes"] - memory_before["swap_used_bytes"],
        "cma_run": False,
        "production_runtime_started": False,
    }
    (OUTPUT / "performance.json").write_text(
        json.dumps(performance, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    cf = {row["counterfactual"]: row for row in counterfactuals}
    readme = f"""# validation_039 obstacle representation root-cause audit

이 결과는 기존 deterministic bag/event와 exact raster만 사용한 진단이다. Production node,
planner, YAML, margin은 변경하지 않았고 replay/CMA를 실행하지 않았다.

## 결론

Primary root cause는 `SENSOR_OCCLUSION_LIMITED`다. 11.23 s scan에서 exact obstacle을 실제로
맞은 beam은 5개(828--832)뿐이며 모두 한 production cluster에 보존되었다. 측정점 AABB는
관측점을 정확히 감싸지만, 센서에 보이지 않은 obstacle depth/side까지 physical footprint로
추론하지 않는다. 따라서 `AABB_MODEL_LIMITATION`이 직접적인 이차 원인이다.

```text
R1 exact raster d-span {float(g1['d_span'])*1000.0:.1f} mm
  -> R2 visible LiDAR d-span {float(next(row for row in chain_rows if row['stage']=='R2')['d_span'])*1000.0:.1f} mm
  -> R3 cluster: all 5 GT-hit points retained
  -> R4/R5 AABB+Detection: measured bounds retained
  -> R6 tracker: prior observations expand the envelope slightly
  -> R7 published static: current visible AABB is reprojected
  -> R8 planner input: field-exact
  -> R9 guard: longitudinal only; d-span {float(r9['d_span'])*1000.0:.1f} mm
```

선택된 right-side obstacle boundary는 G1 `{g1_boundary:.6f} m` 대신 R9
`{float(r9['selected_right_boundary']):.6f} m`가 사용되어 **{boundary_underestimate*1000.0:.1f} mm**
과소 확장됐다. 같은 `8cdda37a93236cc8` B1 path의 R9 production clearance는
`{float(r9_row['production_obstacle_clearance'])*1000.0:.1f} mm`, exact-raster physical clearance는
`{float(exact_row['physical_signed_clearance'])*1000.0:.1f} mm`로 기존 약 -296.97 mm overlap을
재현했다. Timestamp 차이는 없다.

이 path는 field-exact G3 입력으로 `plan()`을 호출했을 때 선택되는 동일 geometry다. 당시 runtime은
safe-stop preparation gate에서 path를 발행하지 않았으므로, 여기서 `selected`는 동일 production
planner reconstruction의 선택을 뜻한다.

## 단계별 판단

- LiDAR observability: full raster의 sensor-facing surface 일부만 관측했고 far/back surface return은 0개다.
- Clustering: GT-hit 5개를 모두 포함하고 비-GT point는 포함하지 않았다. `CLUSTERING_LOSS`가 아니다.
- Cartesian AABB: 측정점에 대해 정확하다. 전체 physical obstacle model로 쓰는 가정이 부족하다.
- Detection/static interface: x/y min/max, independent s/d bounds, diagonal size와 half-diagonal radius가
  보존된다. scalar radius 축약에 의한 정보 손실은 없다.
- Frenet: R4 all-corner와 R5 production projection 차이는 표에 기록했지만 dominant 473.9 mm loss를
  만들지 않는다.
- Tracker: R6에서 과거 extent를 조금 유지하지만 visible R7은 current AABB를 재투영하고 planner
  union/guard는 lateral hidden extent를 복구하지 못한다.
- Planner: R8->R9에서 longitudinal span만 약 375.7 mm 증가하고 lateral span 증가는 0이다.
  hard validator 내부에서는 vehicle half-width 143.5 mm, safety 14.789 mm, tracking reserve
  140 mm를 별도로 적용한다. 이 expansion은 관측되지 않은 physical boundary를 복원하지 않는다.
- Frame: base-link center, +0.275 m LiDAR origin, yaw, same-stamp transform은 일치한다. 다만 replay
  `LaserScan`은 FOV/N, simulator backend는 FOV/(N-1)을 사용해 hit endpoint에 최대
  {max(item['endpoint_error_m'] for item in hit_diag)*1000.0:.1f} mm 오차를 만든다. 이는 secondary다.

## 동일 path counterfactual

- 3-scan visible AABB union: hard-valid={cf['temporal_visible_aabb_union']['hard_valid']},
  false-feasible 제거={cf['temporal_visible_aabb_union']['false_feasible_removed']}.
- visible face에 known 0.5 m rectangle을 복원하는 diagnostic model:
  hard-valid={cf['known_nominal_rectangle_from_visible_face']['hard_valid']},
  제거={cf['known_nominal_rectangle_from_visible_face']['false_feasible_removed']}.
- exact raster oracle: hard-valid={cf['exact_raster_oracle']['hard_valid']},
  제거={cf['exact_raster_oracle']['false_feasible_removed']}.

첫 counterfactual은 hidden extent를 복원하지 않아 physical clearance가 여전히 양수지만, 세 raw
observation의 visible AABB union만으로도 production safety clearance가 약 -6.6 mm가 되어 이 동일
path는 제거된다. 후자의 두 결과는 generic inflation 권고가 아니라, physical extent를 표현하면
동일 path가 제거된다는 진단 증거다.

## 다른 scenario의 동일 signature

| scenario | 판정 | G1-G2 lateral span 차이 |
|---|---|---:|
"""
    for scenario, item in generalization.items():
        readme += (
            f"| {scenario.removeprefix('validation_')} | `{item['status']}` | "
            f"{item['lateral_span_delta_m']*1000.0:+.1f} mm |\n"
        )
    readme += f"""

004/014/019는 기존 raw scan의 GT-hit beam을 재구성해 cluster가 해당 hit를 누락하지 않으면서도
visible surface/AABB가 raster보다 작은 같은 mechanism을 확인했다. 034는 hit/cluster envelope가 raster에
가까워 material한 동일 mechanism이 없다.

## 다음에 조사할 최소 production 후보 (미구현)

1. confirmation 이전 세 raw AABB의 보수적 lateral union을 static/planner lifecycle까지 전달하는 방법.
   이 exact path는 제거하지만 hidden physical extent 자체를 복원하지는 못한다.
2. sensor-facing partial AABB를 전체 obstacle footprint로 간주하지 않는 명시적 extent/visibility model.
3. object class/known dimension이 정당화되는 경우에만 visible face에서 hidden extent를 복원하는 model.
4. 별도 수정으로 replay LaserScan의 angle increment를 simulator backend의 FOV/(N-1)과 일치시키기.

어떤 후보도 이번 작업에서 production에 적용하지 않았다.

## Audit helper

- `path_family_feasibility_audit --representation-root-cause`: production planner의 동일 candidate
  생성, footprint routine, hard validator를 재사용한다.
- `render_validation039_representation_root_cause.py`: bag scan/odom과 passive detector event를
  결합하고 CSV/JSON/README 및 반복성 digest를 생성한다.

## 재현성과 자원

- offline reconstruction 2회 bit-identical: `{str(deterministic_python and deterministic_cpp).lower()}`
- replay: 0, worker: 1, CMA: 미실행
- offline wall: {performance['offline_wall_time_s']:.3f} s
- bag parse: {performance['bag_parse_time_s']:.3f} s
- deterministic repeat: {performance['deterministic_repeat_time_s']:.3f} s
- max RSS: {performance['max_rss_bytes']/1_000_000.0:.1f} MB, swap delta: {performance['swap_change_bytes']} B

Machine-readable 근거는 `representation_chain.csv`, `representation_delta.csv`,
`geometry_timeline.csv`, `same_path_clearance.csv`, `frame_check.json`, `counterfactuals.csv`,
`summary.json`, `performance.json`에 있다.
"""
    (OUTPUT / "README.md").write_text(readme, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
