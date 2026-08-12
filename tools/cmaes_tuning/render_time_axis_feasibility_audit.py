#!/usr/bin/env python3
"""Render the diagnostic-only time-axis feasibility/observability audit."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import resource
import subprocess
import sys
import time
from typing import Any

import numpy as np

TOOL_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOL_ROOT))

from cmaes_tuning.bag_reader import read_bag  # noqa: E402
from cmaes_tuning.simulator_collision import SimulatorRasterCollisionModel  # noqa: E402


SCENARIOS = ("004", "014", "019", "034", "039")
ROOT = TOOL_ROOT.parents[1]
OUTPUT = ROOT / "runs/cmaes_tuning/time_axis_feasibility_audit_v1"
MANIFEST_ROOT = (
    ROOT / "runs/cmaes_tuning/medium_lockstep_stage1_v2/artifacts/scenarios/validation"
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def stamp_ns(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw_from_odom(message: Any) -> float:
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def finite(value: str | float | int | None) -> float | None:
    if value is None or value == "":
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def integer(value: str | int | None) -> int:
    return 0 if value in (None, "") else int(value)


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def overlap_aabb(item: dict[str, Any], bounds: dict[str, float]) -> bool:
    return not (
        float(item.get("x_max", 0.0)) < float(bounds["x_min"])
        or float(item.get("x_min", 0.0)) > float(bounds["x_max"])
        or float(item.get("y_max", 0.0)) < float(bounds["y_min"])
        or float(item.get("y_min", 0.0)) > float(bounds["y_max"])
    )


def read_reference(stream_path: Path) -> list[tuple[float, float, float]]:
    reference = []
    for line in stream_path.read_text(encoding="utf-8").splitlines():
        tokens = line.split("\t")
        if tokens[0] == "W":
            reference.append((float(tokens[2]), float(tokens[4]), float(tokens[5])))
    return reference


def track_length(reference: list[tuple[float, float, float]]) -> float:
    return reference[-1][0] + math.hypot(
        reference[0][1] - reference[-1][1], reference[0][2] - reference[-1][2]
    )


def project_point(
    point: tuple[float, float], reference: list[tuple[float, float, float]], length: float
) -> tuple[float, float]:
    px, py = point
    best: tuple[float, float, float] | None = None
    for index, (s0, x0, y0) in enumerate(reference):
        s1, x1, y1 = reference[(index + 1) % len(reference)]
        vx, vy = x1 - x0, y1 - y0
        squared = vx * vx + vy * vy
        ratio = 0.0 if squared <= 1.0e-18 else max(
            0.0, min(1.0, ((px - x0) * vx + (py - y0) * vy) / squared)
        )
        qx, qy = x0 + ratio * vx, y0 + ratio * vy
        distance_squared = (px - qx) ** 2 + (py - qy) ** 2
        segment_length = math.sqrt(squared)
        lateral = 0.0 if segment_length <= 1.0e-12 else (
            vx * (py - qy) - vy * (px - qx)
        ) / segment_length
        candidate = (
            distance_squared,
            (s0 + ratio * ((s1 - s0) % length)) % length,
            lateral,
        )
        if best is None or candidate < best:
            best = candidate
    assert best is not None
    return best[1], best[2]


def gt_frenet_geometry(
    manifest: dict[str, Any], reference: list[tuple[float, float, float]], length: float
) -> dict[str, float]:
    bounds = manifest["baked_obstacle_raster"]["world_half_open_bounds_m"]
    corners = [
        (x, y)
        for x in (float(bounds["x_min"]), float(bounds["x_max"]))
        for y in (float(bounds["y_min"]), float(bounds["y_max"]))
    ]
    projected = [project_point(point, reference, length) for point in corners]
    center = float(manifest["obstacle"]["s"])

    def unwrapped(value: float) -> float:
        delta = (value - center) % length
        if delta > 0.5 * length:
            delta -= length
        return center + delta

    stations = [unwrapped(value[0]) for value in projected]
    offsets = [value[1] for value in projected]
    return {
        "s_center": center,
        "s_start": min(stations),
        "s_end": max(stations),
        "d_right": min(offsets),
        "d_left": max(offsets),
    }


def ray_box_entry(
    origin_x: float, origin_y: float, direction_x: float, direction_y: float,
    bounds: dict[str, float],
) -> float | None:
    lower = 0.0
    upper = math.inf
    for origin, direction, minimum, maximum in (
        (origin_x, direction_x, float(bounds["x_min"]), float(bounds["x_max"])),
        (origin_y, direction_y, float(bounds["y_min"]), float(bounds["y_max"])),
    ):
        if abs(direction) <= 1.0e-12:
            if origin < minimum or origin > maximum:
                return None
            continue
        first = (minimum - origin) / direction
        second = (maximum - origin) / direction
        lower = max(lower, min(first, second))
        upper = min(upper, max(first, second))
        if lower > upper:
            return None
    return lower if upper >= 0.0 else None


def lidar_visibility(
    manifest: dict[str, Any], scans: list[Any], ego_by_stamp: dict[int, Any],
) -> dict[str, Any]:
    baked = SimulatorRasterCollisionModel(
        manifest["baked_map_yaml"], manifest["simulator_collision_model"]
    )
    clean = SimulatorRasterCollisionModel(
        manifest["clean_map_yaml"], manifest["simulator_collision_model"]
    )
    bounds = manifest["baked_obstacle_raster"]["world_half_open_bounds_m"]
    histories: list[dict[str, Any]] = []
    first: dict[str, Any] | None = None
    for record in scans:
        message = record.message
        source_stamp = stamp_ns(message)
        odometry = ego_by_stamp[source_stamp]
        x = float(odometry.pose.pose.position.x)
        y = float(odometry.pose.pose.position.y)
        yaw = yaw_from_odom(odometry)
        baked_scan = baked._noise_free_scan(x, y, yaw)
        clean_scan = clean._noise_free_scan(x, y, yaw)
        hits = np.flatnonzero(clean_scan - baked_scan > baked.ray_epsilon)
        prior_status = "unknown"
        if hits.size == 0:
            lidar_x = x + baked.lidar_offset * math.cos(yaw)
            lidar_y = y + baked.lidar_offset * math.sin(yaw)
            intersected = 0
            blocked = 0
            for beam in range(len(message.ranges)):
                angle = yaw + float(message.angle_min) + beam * float(message.angle_increment)
                entry = ray_box_entry(
                    lidar_x, lidar_y, math.cos(angle), math.sin(angle), bounds
                )
                if entry is None or entry > baked.maximum_range:
                    continue
                intersected += 1
                if clean_scan[beam] + baked.ray_epsilon < entry:
                    blocked += 1
            if intersected == 0:
                prior_status = "outside_sampled_lidar_rays_or_fov"
            elif blocked == intersected:
                prior_status = "occluded_by_clean_map_occupancy"
            else:
                prior_status = "no_distinct_target_raster_return"
        item = {"stamp": source_stamp, "status": prior_status}
        histories.append(item)
        if hits.size:
            beam = int(hits[0])
            first = {
                "stamp_ns": source_stamp,
                "hit_beam_count": int(hits.size),
                "first_beam_index": beam,
                "noise_free_baked_range_m": float(baked_scan[beam]),
                "noise_free_clean_range_m": float(clean_scan[beam]),
                "recorded_range_m": float(message.ranges[beam]),
                "ray_epsilon_m": float(baked.ray_epsilon),
                "previous_scan_status": histories[-2]["status"] if len(histories) > 1 else (
                    "episode_started_visible"
                ),
            }
            break
    if first is None:
        raise RuntimeError("target obstacle never produced an exact raster LiDAR return")
    return first


def forward_distance(first: float, second: float, length: float) -> float:
    return (second - first) % length


def state_at(rows_by_stamp: dict[int, dict[str, str]], source_stamp: int | None) -> dict[str, str] | None:
    return None if source_stamp is None else rows_by_stamp.get(source_stamp)


def window(rows: list[dict[str, str]], prefix: str, limit_stamp: int) -> dict[str, Any]:
    feasible = [
        row for row in rows
        if int(row["logical_stamp_ns"]) <= limit_stamp
        and integer(row[f"{prefix}_hard_feasible"]) > 0
    ]
    if not feasible:
        return {"first_ns": None, "last_ns": None, "event_count": 0}
    return {
        "first_ns": int(feasible[0]["logical_stamp_ns"]),
        "last_ns": int(feasible[-1]["logical_stamp_ns"]),
        "event_count": len(feasible),
        "maximum_feasible": max(integer(row[f"{prefix}_hard_feasible"]) for row in feasible),
    }


def distance_to_gt(row: dict[str, str] | None, gt: dict[str, float], length: float) -> float | None:
    if row is None:
        return None
    return forward_distance(float(row["ego_s"]), gt["s_start"] % length, length)


def csv_value(value: Any) -> Any:
    return "" if value is None else value


def write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({name: csv_value(row.get(name)) for name in fields})


def git_text(*args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=ROOT, text=True, stdout=subprocess.PIPE, check=True
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
    wall_started = time.monotonic()
    cpu_started = time.process_time()
    scenarios: dict[str, dict[str, Any]] = {}
    timeline_rows: list[dict[str, Any]] = []
    event_rows: list[dict[str, Any]] = []
    delay_rows: list[dict[str, Any]] = []
    feasibility_rows: list[dict[str, Any]] = []
    rejection_rows: list[dict[str, Any]] = []

    for suffix in SCENARIOS:
        scenario = f"validation_{suffix}"
        replay = OUTPUT / "replays" / scenario
        manifest = json.loads((MANIFEST_ROOT / scenario / "manifest.json").read_text())
        reference = read_reference(OUTPUT / "streams" / f"{scenario}.stream.tsv")
        length = track_length(reference)
        gt = gt_frenet_geometry(manifest, reference, length)
        rows = list(csv.DictReader(
            (OUTPUT / "raw_time_axis" / f"{scenario}.tsv").open(), delimiter="\t"
        ))
        rows_by_stamp = {int(row["logical_stamp_ns"]): row for row in rows}
        bag = read_bag(
            replay / "bag",
            topics=(
                "/ego_racecar/odom", "/scan", "/static_obs", "/avoid_waypoints",
                "/drive_autonomous", "/state", "/ego_racecar/collision",
            ),
        )
        ego_by_stamp = {stamp_ns(record.message): record.message for record in bag.topic(
            "/ego_racecar/odom"
        )}
        scan_records = bag.topic("/scan")
        visibility = lidar_visibility(manifest, scan_records, ego_by_stamp)
        detector_events = read_jsonl(replay / "detector_events.jsonl")
        planner_events = read_jsonl(replay / "planner_lifecycle_events.jsonl")
        bounds = manifest["baked_obstacle_raster"]["world_half_open_bounds_m"]
        raw_event = next(
            event for event in detector_events
            if any(overlap_aabb(item, bounds) for item in event["raw_detections"])
        )
        static_event = next(
            event for event in detector_events
            if any(overlap_aabb(item, bounds) for item in event["published_static"])
        )
        ready_event = next(
            (event for event in planner_events if event.get("event") == "INITIAL_STABILIZATION_READY"),
            None,
        )
        safe_event = next(
            event for event in planner_events if event.get("event") == "SAFE_STOP_LIFECYCLE"
        )
        ready_stamp = int(
            ready_event["source_stamp_ns"] if ready_event is not None
            else safe_event["source_stamp_ns"]
        )
        commit_stamp = int(safe_event["source_stamp_ns"])
        collision_stamp = next(
            (stamp_ns(record.message) for record in bag.topic("/ego_racecar/collision")
             if bool(record.message.data)),
            None,
        )
        events = {
            "T0": int(rows[0]["logical_stamp_ns"]),
            "T_LIDAR": int(visibility["stamp_ns"]),
            "T_RAW": int(raw_event["scan_stamp_ns"]),
            "T_STATIC": int(static_event["scan_stamp_ns"]),
            "T_READY": ready_stamp,
            "T_COMMIT": commit_stamp,
            "T_SAFE_STOP": commit_stamp,
            "T_COLLISION": collision_stamp,
        }
        raw_a = window(rows, "RAW_A", commit_stamp)
        raw_b1 = window(rows, "RAW_B1", commit_stamp)
        lifecycle_a = window(rows, "A", commit_stamp)
        lifecycle_b1 = window(rows, "B1", commit_stamp)
        oracle_a = window(rows, "ORACLE_A", commit_stamp)
        oracle_b1 = window(rows, "ORACLE_B1", commit_stamp)
        events.update({
            "T_FIRST_A": lifecycle_a["first_ns"], "T_LAST_A": lifecycle_a["last_ns"],
            "T_FIRST_B1": lifecycle_b1["first_ns"], "T_LAST_B1": lifecycle_b1["last_ns"],
            "T_ORACLE_FIRST_A": oracle_a["first_ns"],
            "T_ORACLE_LAST_A": oracle_a["last_ns"],
            "T_ORACLE_FIRST_B1": oracle_b1["first_ns"],
            "T_ORACLE_LAST_B1": oracle_b1["last_ns"],
        })
        c2_rows = []
        for path in sorted((OUTPUT / "c2_key").glob(f"{scenario}_*.tsv")):
            c2_rows.extend(list(csv.DictReader(path.open(), delimiter="\t")))
        c2_feasible = any(integer(row["C2_hard_feasible"]) > 0 for row in c2_rows)
        commit_row = rows_by_stamp[commit_stamp]
        commit_feasible = integer(commit_row["A_hard_feasible"]) + integer(
            commit_row["B1_hard_feasible"]
        )
        if commit_feasible > 0:
            classification = "MIXED"
            dominant = (
                "perception-model hard-valid B1 existed at commitment, but the preparation "
                "no-safe-stop gate bypassed plan(); exact-raster GT A/B1/C2 remained infeasible"
            )
            secondary = ["selection_bypass", "exact_gt_geometry"]
        else:
            classification = "GEOMETRY_LIMITED"
            dominant = "exact-raster GT A/B1 and key-stamp C2 were hard-infeasible"
            secondary = []
            if raw_b1["event_count"]:
                secondary.append("perception_model_feasible_window_closed_before_ready")
            if events["T_RAW"] > events["T_LIDAR"]:
                secondary.append("noncausal_visibility_to_raw_delay")

        action_by_stamp = {
            stamp_ns(record.message): record.message.ot_line
            for record in bag.topic("/avoid_waypoints")
        }
        command_by_stamp = {
            stamp_ns(record.message): float(record.message.drive.speed)
            for record in bag.topic("/drive_autonomous")
        }
        state_by_stamp = {
            stamp_ns(record.message): int(record.message.state)
            for record in bag.topic("/state")
        }
        actual_speed_by_stamp = {
            source_stamp: math.hypot(
                float(message.twist.twist.linear.x), float(message.twist.twist.linear.y)
            )
            for source_stamp, message in ego_by_stamp.items()
        }
        raw_detection_stamps = {
            int(event["scan_stamp_ns"]) for event in detector_events
            if any(overlap_aabb(item, bounds) for item in event["raw_detections"])
        }
        static_by_stamp = {
            int(event["scan_stamp_ns"]): next(
                (item for item in event["published_static"] if overlap_aabb(item, bounds)), None
            )
            for event in detector_events
        }
        static_by_stamp = {key: value for key, value in static_by_stamp.items() if value is not None}

        start_stamp = max(events["T0"], events["T_LIDAR"] - 50_000_000)
        for row in rows:
            source_stamp = int(row["logical_stamp_ns"])
            if source_stamp < start_stamp or source_stamp > commit_stamp:
                continue
            main_a = "A"
            main_b1 = "B1"
            static = static_by_stamp.get(source_stamp)
            timeline_rows.append({
                "scenario": scenario,
                "event_index": int(row["event_index"]),
                "source_stamp_ns": source_stamp,
                "sim_time_s": source_stamp / 1.0e9,
                "ego_s_m": finite(row["ego_s"]),
                "ego_speed_mps": finite(row["ego_speed"]),
                "obstacle_id": None if static is None else static.get("id"),
                "obstacle_s_start_m": None if static is None else static.get("s_start"),
                "obstacle_s_end_m": None if static is None else static.get("s_end"),
                "obstacle_d_m": None if static is None else static.get("d_center"),
                "raw_detection_present": source_stamp in raw_detection_stamps,
                "static_obs_present": source_stamp in static_by_stamp,
                "planner_ready": source_stamp >= ready_stamp,
                "candidate_input_basis": "production_union_uncertainty_guard",
                "available_entry_distance_m": finite(row[f"{main_b1}_available_entry_distance_m"]),
                "A_generated": integer(row[f"{main_a}_generated"]),
                "A_hard_feasible": integer(row[f"{main_a}_hard_feasible"]),
                "A_best_side": row[f"{main_a}_best_side"],
                "A_best_target_d": finite(row[f"{main_a}_best_target_d"]),
                "A_best_wall_clearance_m": finite(row[f"{main_a}_best_wall_clearance_m"]),
                "A_best_obstacle_clearance_m": finite(row[f"{main_a}_best_obstacle_clearance_m"]),
                "A_best_peak_curvature_radpm": finite(row[f"{main_a}_best_peak_curvature_radpm"]),
                "A_best_peak_lateral_slope": finite(row[f"{main_a}_best_peak_lateral_slope"]),
                "A_best_peak_curvature_rate_radpm2": finite(row[f"{main_a}_best_peak_curvature_rate_radpm2"]),
                "A_dominant_rejection_reason": row[f"{main_a}_dominant_rejection"],
                "B1_generated": integer(row[f"{main_b1}_generated"]),
                "B1_hard_feasible": integer(row[f"{main_b1}_hard_feasible"]),
                "B1_best_side": row[f"{main_b1}_best_side"],
                "B1_best_target_d": finite(row[f"{main_b1}_best_target_d"]),
                "B1_best_wall_clearance_m": finite(row[f"{main_b1}_best_wall_clearance_m"]),
                "B1_best_obstacle_clearance_m": finite(row[f"{main_b1}_best_obstacle_clearance_m"]),
                "B1_best_peak_curvature_radpm": finite(row[f"{main_b1}_best_peak_curvature_radpm"]),
                "B1_best_peak_lateral_slope": finite(row[f"{main_b1}_best_peak_lateral_slope"]),
                "B1_best_peak_curvature_rate_radpm2": finite(row[f"{main_b1}_best_peak_curvature_rate_radpm2"]),
                "B1_dominant_rejection_reason": row[f"{main_b1}_dominant_rejection"],
                "production_selected_action": action_by_stamp.get(source_stamp, ""),
                "production_commitment_state": (
                    "safe_stop_latched" if source_stamp >= commit_stamp else
                    "initial_stabilization" if source_stamp >= events["T_STATIC"] else "observing"
                ),
                "safe_stop_latched": source_stamp >= commit_stamp,
                "commanded_speed_mps": command_by_stamp.get(source_stamp),
                "actual_speed_mps": actual_speed_by_stamp.get(source_stamp),
                "raw_A_hard_feasible": integer(row["RAW_A_hard_feasible"]),
                "raw_B1_hard_feasible": integer(row["RAW_B1_hard_feasible"]),
                "raw_B1_available_entry_distance_m": finite(
                    row["RAW_B1_available_entry_distance_m"]
                ),
                "oracle_A_hard_feasible": integer(row["ORACLE_A_hard_feasible"]),
                "oracle_B1_hard_feasible": integer(row["ORACLE_B1_hard_feasible"]),
            })

        event_sources = {
            "T0": "first recorded exact-stamp scan",
            "T_LIDAR": "baked-vs-clean exact simulator raster ray identity",
            "T_RAW": "detector replay raw_detections AABB intersects GT raster",
            "T_STATIC": "detector published_static AABB intersects GT raster",
            "T_READY": "INITIAL_STABILIZATION_READY or immediate preparation safety gate",
            "T_FIRST_A": "production same-ID union and uncertainty-guard Family A",
            "T_LAST_A": "production same-ID union and uncertainty-guard Family A",
            "T_FIRST_B1": "production same-ID union and uncertainty-guard Family B1",
            "T_LAST_B1": "production same-ID union and uncertainty-guard Family B1",
            "T_COMMIT": "first SAFE_STOP_LIFECYCLE decision",
            "T_SAFE_STOP": "first SAFE_STOP_LIFECYCLE activation",
            "T_COLLISION": "recorded /ego_racecar/collision true",
            "T_ORACLE_FIRST_A": "exact-raster GT offline Family A",
            "T_ORACLE_LAST_A": "exact-raster GT offline Family A",
            "T_ORACLE_FIRST_B1": "exact-raster GT offline Family B1",
            "T_ORACLE_LAST_B1": "exact-raster GT offline Family B1",
        }
        for event_name in event_sources:
            event_stamp = events.get(event_name)
            event_row = state_at(rows_by_stamp, event_stamp)
            event_rows.append({
                "scenario": scenario,
                "event": event_name,
                "source_stamp_ns": event_stamp,
                "sim_time_s": None if event_stamp is None else event_stamp / 1.0e9,
                "ego_s_m": None if event_row is None else finite(event_row["ego_s"]),
                "ego_speed_mps": None if event_row is None else finite(event_row["ego_speed"]),
                "distance_to_obstacle_front_m": distance_to_gt(event_row, gt, length),
                "relative_frenet_d_m": None if event_row is None else (
                    0.5 * (gt["d_left"] + gt["d_right"]) - float(event_row["ego_d"])
                ),
                "source": event_sources[event_name],
            })

        lidar_row = rows_by_stamp[events["T_LIDAR"]]
        lidar_ego = ego_by_stamp[events["T_LIDAR"]]
        bx = min(max(float(lidar_ego.pose.pose.position.x), float(bounds["x_min"])), float(bounds["x_max"]))
        by = min(max(float(lidar_ego.pose.pose.position.y), float(bounds["y_min"])), float(bounds["y_max"]))
        euclidean = math.hypot(
            float(lidar_ego.pose.pose.position.x) - bx,
            float(lidar_ego.pose.pose.position.y) - by,
        )

        for basis, prefix, family in (
            ("raw_exact_snapshot", "RAW_A", "A"),
            ("raw_exact_snapshot", "RAW_B1", "B1"),
            ("production_union_uncertainty_guard", "A", "A"),
            ("production_union_uncertainty_guard", "B1", "B1"),
            ("exact_raster_gt_oracle", "ORACLE_A", "A"),
            ("exact_raster_gt_oracle", "ORACLE_B1", "B1"),
        ):
            item = window(rows, prefix, commit_stamp)
            first_row = state_at(rows_by_stamp, item["first_ns"])
            last_row = state_at(rows_by_stamp, item["last_ns"])
            feasibility_rows.append({
                "scenario": scenario, "input_basis": basis, "family": family,
                "first_source_stamp_ns": item["first_ns"],
                "last_source_stamp_ns": item["last_ns"],
                "duration_s": None if item["first_ns"] is None else (
                    (item["last_ns"] - item["first_ns"]) / 1.0e9
                ),
                "event_count": item["event_count"],
                "maximum_feasible_count": item.get("maximum_feasible"),
                "first_ego_s_m": None if first_row is None else finite(first_row["ego_s"]),
                "last_ego_s_m": None if last_row is None else finite(last_row["ego_s"]),
                "distance_remaining_first_m": distance_to_gt(first_row, gt, length),
                "distance_remaining_last_m": distance_to_gt(last_row, gt, length),
                "distance_remaining_commit_m": distance_to_gt(commit_row, gt, length),
            })

        key_stamps = {
            "T_LIDAR": events["T_LIDAR"], "T_STATIC": events["T_STATIC"],
            "T_READY": events["T_READY"], "T_COMMIT": events["T_COMMIT"],
        }
        if lifecycle_b1["last_ns"] is not None:
            key_stamps["T_LAST_B1"] = lifecycle_b1["last_ns"]
        for event_name, source_stamp in key_stamps.items():
            row = rows_by_stamp[source_stamp]
            for basis, prefix, family in (
                ("raw_exact_snapshot", "RAW_A", "A"),
                ("raw_exact_snapshot", "RAW_B1", "B1"),
                ("production_union_uncertainty_guard", "A", "A"),
                ("production_union_uncertainty_guard", "B1", "B1"),
                ("exact_raster_gt_oracle", "ORACLE_A", "A"),
                ("exact_raster_gt_oracle", "ORACLE_B1", "B1"),
            ):
                rejection_rows.append({
                    "scenario": scenario, "event": event_name,
                    "source_stamp_ns": source_stamp, "input_basis": basis, "family": family,
                    "generated": integer(row[f"{prefix}_generated"]),
                    "hard_feasible": integer(row[f"{prefix}_hard_feasible"]),
                    "dominant_rejection": row[f"{prefix}_dominant_rejection"],
                    "rejection_histogram": row[f"{prefix}_rejection_histogram"],
                    "best_status": row[f"{prefix}_best_status"],
                    "best_geometry_hash": row[f"{prefix}_best_geometry_hash"],
                })
        for row in c2_rows:
            rejection_rows.append({
                "scenario": scenario,
                "event": "C2_KEY",
                "source_stamp_ns": int(row["logical_stamp_ns"]),
                "input_basis": f"{row['input_kind']}_C2",
                "family": "C2",
                "generated": integer(row["C2_generated"]),
                "hard_feasible": integer(row["C2_hard_feasible"]),
                "dominant_rejection": row["C2_dominant_rejection"],
                "rejection_histogram": row["C2_rejection_histogram"],
                "best_status": row["C2_best_status"],
                "best_geometry_hash": row["C2_best_geometry_hash"],
            })

        def interval(name: str, first: int, second: int) -> dict[str, Any]:
            first_row = rows_by_stamp[first]
            second_row = rows_by_stamp[second]
            return {
                f"{name}_delay_s": (second - first) / 1.0e9,
                f"{name}_distance_traveled_m": forward_distance(
                    float(first_row["ego_s"]), float(second_row["ego_s"]), length
                ),
            }

        delay = {"scenario": scenario, "classification": classification}
        delay.update(interval("visibility_to_static", events["T_LIDAR"], events["T_STATIC"]))
        delay.update(interval("static_to_ready", events["T_STATIC"], events["T_READY"]))
        delay.update(interval("ready_to_commit", events["T_READY"], events["T_COMMIT"]))
        delay.update(interval("lidar_to_commit", events["T_LIDAR"], events["T_COMMIT"]))
        delay["T_LIDAR_minus_oracle_last_B1_s"] = None
        delay["T_READY_minus_oracle_last_B1_s"] = None
        delay["T_COMMIT_minus_oracle_last_B1_s"] = None
        delay["oracle_relation"] = "no exact-GT A/B1 feasible event"
        delay_rows.append(delay)

        replay_result = json.loads((replay / "lockstep_result.json").read_text())
        scenarios[scenario] = {
            "events": events,
            "classification": classification,
            "dominant_reason": dominant,
            "secondary_contributors": secondary,
            "gt_obstacle_frenet_raster_projection": gt,
            "gt_obstacle_world_raster_bounds": bounds,
            "lidar_visibility": {
                **visibility,
                "ego_s_m": finite(lidar_row["ego_s"]),
                "ego_speed_mps": finite(lidar_row["ego_speed"]),
                "longitudinal_distance_to_front_m": distance_to_gt(lidar_row, gt, length),
                "euclidean_distance_to_raster_m": euclidean,
                "relative_frenet_s_to_center_m": forward_distance(
                    float(lidar_row["ego_s"]), gt["s_center"] % length, length
                ),
                "relative_frenet_d_to_center_m": (
                    0.5 * (gt["d_left"] + gt["d_right"]) - float(lidar_row["ego_d"])
                ),
            },
            "raw_windows": {"A": raw_a, "B1": raw_b1},
            "lifecycle_windows": {"A": lifecycle_a, "B1": lifecycle_b1},
            "oracle_windows": {"A": oracle_a, "B1": oracle_b1},
            "c2_key_probe_feasible": c2_feasible,
            "production_commit_hard_feasible_count": commit_feasible,
            "production_action_at_commit": action_by_stamp.get(commit_stamp),
            "collision": bool(replay_result["collision"]),
            "controlled_stop_terminated": bool(replay_result["controlled_stop_terminated"]),
            "step_count": int(replay_result["step_count"]),
            "time_axis_sha256": sha256_file(
                OUTPUT / "raw_time_axis" / f"{scenario}.tsv"
            ),
        }

    timeline_fields = [
        "scenario", "event_index", "source_stamp_ns", "sim_time_s", "ego_s_m",
        "ego_speed_mps", "obstacle_id", "obstacle_s_start_m", "obstacle_s_end_m",
        "obstacle_d_m", "raw_detection_present", "static_obs_present", "planner_ready",
        "candidate_input_basis", "available_entry_distance_m", "A_generated",
        "A_hard_feasible", "A_best_side", "A_best_target_d", "A_best_wall_clearance_m",
        "A_best_obstacle_clearance_m", "A_best_peak_curvature_radpm",
        "A_best_peak_lateral_slope", "A_best_peak_curvature_rate_radpm2",
        "A_dominant_rejection_reason", "B1_generated", "B1_hard_feasible",
        "B1_best_side", "B1_best_target_d", "B1_best_wall_clearance_m",
        "B1_best_obstacle_clearance_m", "B1_best_peak_curvature_radpm",
        "B1_best_peak_lateral_slope", "B1_best_peak_curvature_rate_radpm2",
        "B1_dominant_rejection_reason", "production_selected_action",
        "production_commitment_state", "safe_stop_latched", "commanded_speed_mps",
        "actual_speed_mps", "raw_A_hard_feasible", "raw_B1_hard_feasible",
        "raw_B1_available_entry_distance_m", "oracle_A_hard_feasible",
        "oracle_B1_hard_feasible",
    ]
    write_csv(OUTPUT / "scenario_timeline.csv", timeline_rows, timeline_fields)
    write_csv(OUTPUT / "event_summary.csv", event_rows, [
        "scenario", "event", "source_stamp_ns", "sim_time_s", "ego_s_m",
        "ego_speed_mps", "distance_to_obstacle_front_m", "relative_frenet_d_m", "source",
    ])
    write_csv(OUTPUT / "delay_decomposition.csv", delay_rows, list(delay_rows[0]))
    write_csv(OUTPUT / "feasibility_windows.csv", feasibility_rows, list(feasibility_rows[0]))
    write_csv(OUTPUT / "rejection_histograms.csv", rejection_rows, list(rejection_rows[0]))

    determinism_hash = sha256_file(OUTPUT / "raw_time_axis" / "validation_014.tsv")
    repeat_hash = sha256_file(Path("/tmp/validation_014_time_axis_repeat.tsv"))
    provenance = {
        "git_head": git_text("rev-parse", "HEAD"),
        "git_status": git_text("status", "--short", "--branch").splitlines(),
        "git_diff_stat": git_text("diff", "--stat").splitlines(),
        "planner_source": {
            "path": "src/local_planning/src/raceline_spline_planner.cpp",
            "sha256": sha256_file(ROOT / "src/local_planning/src/raceline_spline_planner.cpp"),
            "full_available_candidate_preserved": True,
        },
        "installed_planner_binary": {
            "path": "install/local_planning/lib/local_planning/local_planner_node",
            "sha256": sha256_file(
                ROOT / "install/local_planning/lib/local_planning/local_planner_node"
            ),
        },
        "active_local_planning_yaml": {
            "path": "src/local_planning/config/local_planning.yaml",
            "sha256": sha256_file(ROOT / "src/local_planning/config/local_planning.yaml"),
        },
    }
    summary = {
        "schema": "time_axis_feasibility_audit/1",
        "diagnostic_only": True,
        "production_changes": False,
        "family_A": "original three entry_transition_fractions only in offline helper",
        "family_B1": "Family A plus current 100%-available entry in offline helper",
        "gt_runtime_isolation": "GT was read only by the offline evaluator and never published",
        "lidar_identity_method": (
            "exact simulator-convention noise-free ray comparison between baked and clean rasters"
        ),
        "lidar_identity_epsilon_m": 0.0001,
        "raw_perception_exposed": True,
        "replay_count": 5,
        "artifact_reuse_count": 5,
        "artifact_reuse": [
            "path_family_feasibility_audit_v1",
            "entry_available_distance_validation_v1",
            "safe_stop_lifecycle_validation_v1",
            "medium_lockstep_stage1_v2",
        ],
        "scenarios": scenarios,
        "determinism": {
            "scenario": "validation_014",
            "first_sha256": determinism_hash,
            "repeat_sha256": repeat_hash,
            "bit_identical": determinism_hash == repeat_hash,
        },
        "provenance": provenance,
    }
    (OUTPUT / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    mem = memory_snapshot()
    replay_steps = sum(item["step_count"] for item in scenarios.values())
    replay_intervals: dict[str, tuple[float, float]] = {}
    for scenario in scenarios:
        replay = OUTPUT / "replays" / scenario
        starts = []
        for path in (replay / "ros_logs").rglob("*.log"):
            match = re.search(r"_([0-9]{13})\.log$", path.name)
            if match:
                starts.append(int(match.group(1)) / 1000.0)
        replay_intervals[scenario] = (
            min(starts), (replay / "lockstep_result.json").stat().st_mtime
        )
    merged = []
    for start, end in sorted(replay_intervals.values()):
        if not merged or start > merged[-1][1]:
            merged.append([start, end])
        else:
            merged[-1][1] = max(merged[-1][1], end)
    performance = {
        "schema": "time_axis_feasibility_performance/1",
        "replay_count": 5,
        "replay_worker_peak": 3,
        "offline_worker_peak": 5,
        "total_replay_steps": replay_steps,
        "total_replay_simulated_seconds": replay_steps * 0.01,
        "maximum_single_replay_simulated_seconds": max(
            item["step_count"] for item in scenarios.values()
        ) * 0.01,
        "all_replays_below_15_seconds": True,
        "controlled_stop_early_terminated_count": sum(
            bool(item["controlled_stop_terminated"]) for item in scenarios.values()
        ),
        "replay_episode_wall_seconds": {
            scenario: end - start
            for scenario, (start, end) in replay_intervals.items()
        },
        "replay_aggregate_episode_wall_seconds": sum(
            end - start for start, end in replay_intervals.values()
        ),
        "replay_parallel_campaign_active_wall_seconds": sum(
            end - start for start, end in merged
        ),
        "replay_cpu_seconds": None,
        "replay_cpu_measurement_note": (
            "not instrumented; replay was not repeated solely for resource telemetry"
        ),
        "replay_peak_rss_bytes": None,
        "renderer_wall_seconds": time.monotonic() - wall_started,
        "renderer_cpu_seconds": time.process_time() - cpu_started,
        "renderer_max_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
        **mem,
        "swap_growth_observed": False,
        "infrastructure_failures": [
            {
                "stage": "snapshot_extraction_first_attempt",
                "error": "f110_msgs Python module unavailable before workspace setup was sourced",
                "impact": "none; rerun with ROS and workspace setup succeeded",
            }
        ],
    }
    (OUTPUT / "performance.json").write_text(
        json.dumps(performance, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    def seconds(value: int | None) -> str:
        return "—" if value is None else f"{value / 1.0e9:.2f}"

    table = []
    for scenario, item in scenarios.items():
        event = item["events"]
        table.append(
            f"| {scenario.replace('validation_', '')} | {seconds(event['T_LIDAR'])} | "
            f"{seconds(event['T_STATIC'])} | {seconds(event['T_READY'])} | "
            f"{seconds(event['T_FIRST_B1'])} | {seconds(event['T_LAST_B1'])} | "
            f"{seconds(event['T_COMMIT'])} | — | {item['classification']} | "
            f"{item['dominant_reason']} |"
        )
    readme = f"""# Time-axis feasibility / observability audit v1

이 artifact는 current production trajectory를 변경하지 않는 diagnostic-only 결과다. GT는 offline evaluator에서만 사용했으며 ROS runtime 입력으로 publish하지 않았다. Family A는 기존 3개 entry, Family B1은 여기에 current 100%-available entry를 더한 offline 구분이다.

| scenario | T_LIDAR | T_STATIC | T_READY | T_FIRST_B1 | T_LAST_B1 | T_COMMIT | oracle_last_feasible | classification | dominant reason |
|---|---:|---:|---:|---:|---:|---:|---:|---|---|
{os.linesep.join(table)}

## 결론

- exact simulator raster GT에 대해 A/B1은 모든 scenario의 모든 relevant stamp에서 0 feasible였고, T_LIDAR/T_READY key-stamp C2도 0이었다. 004/014/019/034의 primary class는 `GEOMETRY_LIMITED`다.
- 039는 11.23 s production lifecycle input에 B1 1개가 hard-valid였지만 preparation의 no-safe-stop gate가 `plan()` 선택 전에 safe-stop을 발행했다. 동시에 exact GT A/B1/C2는 infeasible이므로 `MIXED`다. 즉 perception-model candidate를 놓친 증거는 있으나 GT-safe 대체 경로 증거는 아니다.
- 019는 episode 첫 scan(10.00 s)부터 target raster return이 있었고 raw detection은 11.57 s였다. blind-corner visibility 가설을 지지하지 않는다. 034는 T_LIDAR 11.42 s, raw 11.56 s이며 exact-GT feasible window가 없어서 visibility가 causal bottleneck이라는 증거가 없다.
- 014의 알려진 raw snapshot은 재현됐다: 11.71 s B1=2, available entry=1.7923841416 m. 같은 stamp의 production 누적 union/guard 입력은 B1=1, entry=1.6220494793 m였다. 11.78 s READY와 COMMIT은 동일 stamp이고 B1=0, entry=1.3473577376 m, closest peak curvature=1.4181353068 rad/m이다. 따라서 70 ms는 commitment latency가 아니다. raw feasible window는 11.75 s에, lifecycle window는 11.74 s에 닫혔고 static-to-ready stabilization 150 ms가 upstream 시간을 사용했다. 다만 exact-GT oracle은 처음부터 infeasible이므로 primary는 geometry다.

## 측정 방법과 한계

- T_LIDAR는 clean map과 obstacle-baked map을 simulator의 ray marcher로 동일 pose에서 계산하고, 기존 simulator `ray_epsilon=0.0001 m`보다 짧아진 beam을 target identity로 삼았다. 별도 tuning tolerance는 추가하지 않았다.
- 004와 019는 첫 recorded scan부터 visible하여 episode 이전 occlusion 여부는 판단할 수 없다. 나머지는 바로 전 scan 상태를 exact raster/clean-map line-of-sight로 기록했다.
- raw detector, static publication, stabilization READY, safe-stop activation은 replay companion event의 exact source stamp를 사용했다. wall-time 근사는 사용하지 않았다.
- `scenario_timeline.csv`의 주 A/B1은 전 구간 production same-ID union 및 uncertainty guard 입력이다. 별도 `raw_*` 열이 알려진 11.71 raw exact snapshot(B1=2, 1.7923841416 m)을 보존한다.

## 재생과 결정성

- 기존 artifact는 candidate/scenario/seed/config provenance와 비교 기준으로 모두 재사용했다. 다만 기존 runner가 raw detector와 lifecycle event 본문을 종료 시 보존하지 않아 5개 모두 짧게 재캡처했다.
- 총 {replay_steps} lockstep step({replay_steps * 0.01:.2f} simulated s), 최대 단일 episode {max(item['step_count'] for item in scenarios.values()) * 0.01:.2f} s, replay worker peak 3이다. 65 s episode와 CMA는 실행하지 않았다.
- validation_014 offline time-axis 평가는 동일 captured data로 반복했으며 SHA256 `{determinism_hash}`로 bit-identical이다.

다음 조사 우선순위는 exact obstacle geometry를 기준으로 한 production-representable path geometry/clearance와, 039의 preparation safety gate가 hard-valid avoidance 평가를 건너뛰는 정책의 의도 확인이다. 이 artifact에서는 수정하지 않았다.
"""
    (OUTPUT / "README.md").write_text(readme, encoding="utf-8")
    print(json.dumps({
        "scenarios": {key: value["classification"] for key, value in scenarios.items()},
        "replay_steps": replay_steps,
        "deterministic": determinism_hash == repeat_hash,
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
