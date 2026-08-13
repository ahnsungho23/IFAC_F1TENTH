"""Offline decomposition of planner path and closed-loop swept-footprint clearance.

This module is diagnostic-only.  It reads immutable lockstep bags and reuses the
external evaluator's 0.56 m x 0.287 m base_link-centred footprint and occupancy
raster conventions; it does not feed any result back into planning or control.
"""

from __future__ import annotations

import bisect
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
from typing import Any, Iterable

import numpy as np

from .bag_reader import MessageRecord, read_bag
from .geometry import OccupancyDistanceField, footprint_obstacle_clearance, oriented_rectangle
from .metrics import odom_samples
from .schemas import atomic_write_json, sha256_file
from .simulator_collision import SimulatorRasterCollisionModel, repair_collision_yaw_reset


ANALYSIS_TOPICS = (
    "/ego_racecar/odom",
    "/car_state/frenet/odom",
    "/avoid_waypoints",
    "/local_waypoints",
    "/drive",
    "/state",
)

TIMESERIES_FIELDS = (
    "simulated_time_s",
    "timestamp_ns",
    "paired_path_timestamp_ns",
    "paired_path_age_s",
    "ego_s_m",
    "ego_d_m",
    "actual_x_m",
    "actual_y_m",
    "actual_yaw_rad",
    "actual_speed_mps",
    "local_target_x_m",
    "local_target_y_m",
    "local_path_heading_rad",
    "local_path_curvature_radpm",
    "local_path_curvature_rate_radpm2",
    "local_waypoint_speed_mps",
    "lateral_tracking_error_m",
    "wallward_lateral_error_m",
    "heading_error_rad",
    "heading_corner_protrusion_m",
    "planned_centerline_wall_clearance_m",
    "planned_footprint_wall_clearance_m",
    "planned_polygon_raster_wall_clearance_m",
    "actual_position_path_heading_clearance_m",
    "actual_swept_footprint_wall_clearance_m",
    "actual_evaluator_wall_clearance_m",
    "obstacle_clearance_m",
    "footprint_clearance_loss_m",
    "tracking_position_clearance_loss_m",
    "heading_rotation_clearance_loss_m",
    "raster_sampling_clearance_loss_m",
    "steering_command_rad",
    "steering_rate_radps",
    "speed_command_mps",
    "avoidance_phase",
    "path_mode",
    "state",
    "is_minimum_clearance_sample",
    "is_pre_collision_0p5s",
)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def interpolate_angle(first: float, second: float, ratio: float) -> float:
    return wrap_angle(first + ratio * wrap_angle(second - first))


def percentile_summary(values: Iterable[float]) -> dict[str, float | int | None]:
    samples = np.asarray([float(value) for value in values if math.isfinite(float(value))])
    if samples.size == 0:
        return {"count": 0, "max": None, "p95": None, "p99": None, "mean": None}
    return {
        "count": int(samples.size),
        "max": float(np.max(samples)),
        "p95": float(np.percentile(samples, 95.0)),
        "p99": float(np.percentile(samples, 99.0)),
        "mean": float(np.mean(samples)),
    }


def pearson(first: Iterable[float], second: Iterable[float]) -> dict[str, float | int | None]:
    pairs = [
        (float(x), float(y)) for x, y in zip(first, second)
        if math.isfinite(float(x)) and math.isfinite(float(y))
    ]
    if len(pairs) < 3:
        return {"count": len(pairs), "coefficient": None, "reason": "fewer_than_3_samples"}
    x = np.asarray([pair[0] for pair in pairs])
    y = np.asarray([pair[1] for pair in pairs])
    if float(np.ptp(x)) <= 1.0e-12 or float(np.ptp(y)) <= 1.0e-12:
        return {"count": len(pairs), "coefficient": None, "reason": "zero_variance"}
    return {
        "count": len(pairs),
        "coefficient": float(np.corrcoef(x, y)[0, 1]),
        "reason": None,
    }


@dataclass(frozen=True)
class PathProjection:
    x: float
    y: float
    heading: float
    curvature: float
    curvature_rate: float
    speed: float
    s: float
    d: float
    segment_index: int
    ratio: float
    lateral_error: float


def project_to_path(x: float, y: float, waypoints: list[Any]) -> PathProjection | None:
    """Project a point onto a piecewise-linear waypoint path deterministically."""

    if not waypoints:
        return None
    if len(waypoints) == 1:
        waypoint = waypoints[0]
        heading = float(waypoint.psi_rad)
        dx, dy = x - float(waypoint.x_m), y - float(waypoint.y_m)
        return PathProjection(
            x=float(waypoint.x_m), y=float(waypoint.y_m), heading=heading,
            curvature=float(waypoint.kappa_radpm), curvature_rate=0.0,
            speed=float(waypoint.vx_mps), s=float(waypoint.s_m), d=float(waypoint.d_m),
            segment_index=0, ratio=0.0,
            lateral_error=-math.sin(heading) * dx + math.cos(heading) * dy,
        )

    best: PathProjection | None = None
    best_squared = math.inf
    for index, (first, second) in enumerate(zip(waypoints, waypoints[1:])):
        x0, y0 = float(first.x_m), float(first.y_m)
        dx, dy = float(second.x_m) - x0, float(second.y_m) - y0
        length_squared = dx * dx + dy * dy
        if length_squared <= 1.0e-18:
            continue
        ratio = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / length_squared))
        projected_x, projected_y = x0 + ratio * dx, y0 + ratio * dy
        squared = (x - projected_x) ** 2 + (y - projected_y) ** 2
        if squared >= best_squared:
            continue
        distance = math.sqrt(length_squared)
        heading = math.atan2(dy, dx)
        curvature_first = float(first.kappa_radpm)
        curvature_second = float(second.kappa_radpm)
        best_squared = squared
        best = PathProjection(
            x=projected_x,
            y=projected_y,
            heading=heading,
            curvature=curvature_first + ratio * (curvature_second - curvature_first),
            curvature_rate=(curvature_second - curvature_first) / distance,
            speed=float(first.vx_mps) + ratio * (float(second.vx_mps) - float(first.vx_mps)),
            s=float(first.s_m) + ratio * (float(second.s_m) - float(first.s_m)),
            d=float(first.d_m) + ratio * (float(second.d_m) - float(first.d_m)),
            segment_index=index,
            ratio=ratio,
            lateral_error=-math.sin(heading) * (x - projected_x)
            + math.cos(heading) * (y - projected_y),
        )
    return best


class RasterFootprintProbe:
    """Exact oriented polygon-to-occupied-cell clearance on the clean map raster."""

    def __init__(self, map_yaml: str | Path, model: dict[str, Any]):
        self.model = SimulatorRasterCollisionModel(map_yaml, model)

    def clearance(
        self, x: float, y: float, yaw: float, length: float, width: float
    ) -> tuple[float, bool]:
        polygon = self.model._map_polygon(oriented_rectangle(x, y, yaw, length, width))
        overlap, _ = self.model._polygon_raster_overlap(polygon)
        if overlap:
            return 0.0, True
        clearance = self.model._clearance_within(polygon, 0.50)
        if not math.isfinite(clearance):
            clearance = self.model._clearance_within(polygon, 2.0)
        return float(clearance), False


def _record_map(records: list[MessageRecord]) -> dict[int, Any]:
    return {record.timestamp_ns: record.message for record in records}


def _latest_message(
    messages: dict[int, Any], ordered_timestamps: list[int], timestamp_ns: int
) -> tuple[int, Any] | tuple[None, None]:
    index = bisect.bisect_right(ordered_timestamps, timestamp_ns) - 1
    if index < 0:
        return None, None
    selected_timestamp = ordered_timestamps[index]
    return selected_timestamp, messages[selected_timestamp]


def _candidate_audit(episode_directory: Path) -> dict[str, float | int | str | None]:
    pattern = re.compile(
        r"generated=(\d+) feasible=(\d+) rank=(\d+) side=(\w+) target=([-0-9.]+) "
        r"entry_requested/effective=([-0-9.]+)/([-0-9.]+) exit=([-0-9.]+) "
        r"wall/obstacle=([-0-9.]+)/([-0-9.]+) peak_curvature/rate="
        r"([-0-9.]+)/([-0-9.]+) speed_loss=([-0-9.]+) min_slack=([-0-9.]+)"
    )
    for path in sorted((episode_directory / "ros_logs").glob("local_planner_node_*.log")):
        match = pattern.search(path.read_text(encoding="utf-8", errors="replace"))
        if match:
            values = match.groups()
            return {
                "generated_count": int(values[0]), "feasible_count": int(values[1]),
                "rank": int(values[2]), "side": values[3], "target_d_m": float(values[4]),
                "requested_entry_length_m": float(values[5]),
                "effective_entry_length_m": float(values[6]), "exit_length_m": float(values[7]),
                "planner_wall_headroom_m": float(values[8]),
                "planner_obstacle_headroom_m": float(values[9]),
                "planner_peak_curvature_radpm": float(values[10]),
                "planner_peak_curvature_rate_radpm2": float(values[11]),
                "planner_velocity_loss": float(values[12]),
                "planner_minimum_normalized_slack": float(values[13]),
            }
    return {}


def _phase(
    mode: str, state: int, projection: PathProjection, target_d: float
) -> str:
    if mode == "raceline_global_handoff":
        return "handoff"
    if mode != "raceline_local_d_offset_spline":
        return "pre_avoid" if state == 0 else "other"
    # d interpolation is linear on the active segment; compare endpoints through the projected
    # segment's local slope. Curvature is unrelated and intentionally not used as a phase proxy.
    if abs(target_d) > 1.0e-9 and abs(projection.d) >= 0.95 * abs(target_d):
        return "obstacle_hold"
    # The path is ordered and its d sign is the committed side. Before the plateau, |d| grows;
    # after it, |d| falls. The nearest segment index is resolved by the caller below.
    return "entry_or_exit"


def _resolve_entry_exit(
    phase: str, projection: PathProjection, waypoints: list[Any], target_d: float
) -> str:
    if phase != "entry_or_exit" or len(waypoints) < 2:
        return phase
    index = min(projection.segment_index, len(waypoints) - 2)
    delta = float(waypoints[index + 1].d_m) - float(waypoints[index].d_m)
    signed_delta = (1.0 if target_d >= 0.0 else -1.0) * delta
    if signed_delta > 1.0e-8:
        return "entry"
    if signed_delta < -1.0e-8:
        return "exit"
    return "obstacle_hold"


def _dense_path_minimum(
    waypoints: list[Any], field: OccupancyDistanceField, length: float, width: float,
    step: float,
) -> dict[str, float]:
    waypoint_minimum = math.inf
    dense_minimum = math.inf
    centerline_minimum = math.inf
    for waypoint in waypoints:
        clearance, _ = field.footprint_clearance(
            float(waypoint.x_m), float(waypoint.y_m), float(waypoint.psi_rad),
            length, width, step)
        waypoint_minimum = min(waypoint_minimum, clearance)
    for first, second in zip(waypoints, waypoints[1:]):
        dx, dy = float(second.x_m) - float(first.x_m), float(second.y_m) - float(first.y_m)
        distance = math.hypot(dx, dy)
        count = max(1, int(math.ceil(distance / step)))
        for sample_index in range(count + 1):
            ratio = sample_index / count
            x = float(first.x_m) + ratio * dx
            y = float(first.y_m) + ratio * dy
            yaw = interpolate_angle(float(first.psi_rad), float(second.psi_rad), ratio)
            footprint_clearance, _ = field.footprint_clearance(
                x, y, yaw, length, width, step)
            point_clearance, _ = field.footprint_clearance(
                x, y, yaw, 0.0, 0.0, step)
            dense_minimum = min(dense_minimum, footprint_clearance)
            centerline_minimum = min(centerline_minimum, point_clearance)
    return {
        "waypoint_footprint_minimum_m": waypoint_minimum,
        "continuous_footprint_minimum_m": dense_minimum,
        "continuous_centerline_minimum_m": centerline_minimum,
        "waypoint_sampling_loss_m": max(0.0, waypoint_minimum - dense_minimum),
    }


def _dense_actual_minimum(
    rows: list[dict[str, Any]], field: OccupancyDistanceField,
    length: float, width: float, step: float,
) -> float:
    minimum = math.inf
    for first, second in zip(rows, rows[1:]):
        dx = float(second["actual_x_m"]) - float(first["actual_x_m"])
        dy = float(second["actual_y_m"]) - float(first["actual_y_m"])
        distance = math.hypot(dx, dy)
        count = max(1, int(math.ceil(distance / step)))
        for sample_index in range(count + 1):
            ratio = sample_index / count
            clearance, _ = field.footprint_clearance(
                float(first["actual_x_m"]) + ratio * dx,
                float(first["actual_y_m"]) + ratio * dy,
                interpolate_angle(
                    float(first["actual_yaw_rad"]), float(second["actual_yaw_rad"]), ratio),
                length, width, step,
            )
            minimum = min(minimum, clearance)
    return minimum


def _write_csv(path: Path, rows: list[dict[str, Any]], fields: Iterable[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(fields), extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def analyze_episode(
    episode_directory: str | Path,
    manifest_path: str | Path,
    output_csv: str | Path,
    footprint_sample_step_m: float = 0.0125,
    collision_configuration: dict[str, Any] | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    episode_directory = Path(episode_directory).resolve()
    manifest_path = Path(manifest_path).resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    episode_result = json.loads(
        (episode_directory / "episode_result.json").read_text(encoding="utf-8"))
    lockstep_result = json.loads(
        (episode_directory / "lockstep_result.json").read_text(encoding="utf-8"))
    bag = read_bag(episode_directory / "bag", ANALYSIS_TOPICS)

    vehicle_length = float(manifest["vehicle_length_m"])
    vehicle_width = float(manifest["vehicle_width_m"])
    collision_model = dict(manifest["simulator_collision_model"])
    collision_model["vehicle_length_m"] = vehicle_length
    collision_model["vehicle_width_m"] = vehicle_width
    field = OccupancyDistanceField(manifest["clean_map_yaml"])
    probe = RasterFootprintProbe(manifest["clean_map_yaml"], collision_model)

    repaired_samples, yaw_repair = repair_collision_yaw_reset(
        odom_samples(bag.topic("/ego_racecar/odom")),
        {
            **(collision_configuration or {}),
            **collision_model,
            "physics_timestep_sec": collision_model["physics_timestep_sec"],
        },
    )
    frenet = _record_map(bag.topic("/car_state/frenet/odom"))
    avoid = _record_map(bag.topic("/avoid_waypoints"))
    local = _record_map(bag.topic("/local_waypoints"))
    drives = _record_map(bag.topic("/drive"))
    states = _record_map(bag.topic("/state"))
    ordered = {
        "frenet": sorted(frenet), "avoid": sorted(avoid), "local": sorted(local),
        "drives": sorted(drives), "states": sorted(states),
    }

    first_avoid_ns = min(
        (
            timestamp for timestamp, message in avoid.items()
            if message.wpnts and message.ot_line == "raceline_local_d_offset_spline"
        ),
        default=None,
    )
    if first_avoid_ns is None:
        raise ValueError(f"no committed avoidance path in {episode_directory}")
    first_timestamp_ns = int(repaired_samples[0]["timestamp_ns"])
    external = episode_result["collision_diagnostics"]["external"]
    collision_timestamp_ns = external.get("first_collision_timestamp_ns")
    target_d = float(lockstep_result["committed_target_d"])
    side_sign = 1.0 if lockstep_result["selected_side"] == "left" else -1.0

    rows: list[dict[str, Any]] = []
    previous_steering: float | None = None
    previous_timestamp_ns: int | None = None
    for sample in repaired_samples:
        timestamp_ns = int(sample["timestamp_ns"])
        if timestamp_ns < first_avoid_ns:
            continue
        path_timestamp_ns, local_message = _latest_message(
            local, ordered["local"], timestamp_ns)
        _, avoid_message = _latest_message(avoid, ordered["avoid"], timestamp_ns)
        _, frenet_message = _latest_message(frenet, ordered["frenet"], timestamp_ns)
        _, drive_message = _latest_message(drives, ordered["drives"], timestamp_ns)
        _, state_message = _latest_message(states, ordered["states"], timestamp_ns)
        if not all((local_message, avoid_message, frenet_message, drive_message, state_message)):
            continue
        path_waypoints = list(local_message.wpnts)
        projection = project_to_path(float(sample["x"]), float(sample["y"]), path_waypoints)
        if projection is None:
            continue

        planned_centerline, _ = field.footprint_clearance(
            projection.x, projection.y, projection.heading, 0.0, 0.0,
            footprint_sample_step_m)
        planned_footprint, _ = field.footprint_clearance(
            projection.x, projection.y, projection.heading, vehicle_length, vehicle_width,
            footprint_sample_step_m)
        # The evaluator samples the complete rectangular footprint at 12.5 mm. This retains the
        # validated external convention for the full time series. Exact polygon-to-cell distance
        # is evaluated only at the critical sample below to quantify raster sampling bias.
        planned_exact = planned_footprint
        position_path_heading, _ = field.footprint_clearance(
            float(sample["x"]), float(sample["y"]), projection.heading,
            vehicle_length, vehicle_width, footprint_sample_step_m)
        actual_evaluator, _ = field.footprint_clearance(
            float(sample["x"]), float(sample["y"]), float(sample["yaw"]),
            vehicle_length, vehicle_width, footprint_sample_step_m)
        actual_exact = actual_evaluator
        obstacle_clearance, _ = footprint_obstacle_clearance(
            float(sample["x"]), float(sample["y"]), float(sample["yaw"]),
            vehicle_length, vehicle_width, manifest["obstacle"])
        heading_error = wrap_angle(float(sample["yaw"]) - projection.heading)
        corner_protrusion = (
            0.5 * vehicle_length * abs(math.sin(heading_error))
            + 0.5 * vehicle_width * abs(math.cos(heading_error))
            - 0.5 * vehicle_width
        )
        steering = float(drive_message.drive.steering_angle)
        dt = (
            (timestamp_ns - previous_timestamp_ns) * 1.0e-9
            if previous_timestamp_ns is not None else 0.0)
        steering_rate = (
            (steering - previous_steering) / dt
            if previous_steering is not None and dt > 1.0e-9 else 0.0)
        state = int(state_message.state)
        phase = _resolve_entry_exit(
            _phase(avoid_message.ot_line, state, projection, target_d),
            projection, path_waypoints, target_d)
        pose = frenet_message.pose.pose
        row = {
            "simulated_time_s": (timestamp_ns - first_timestamp_ns) * 1.0e-9,
            "timestamp_ns": timestamp_ns,
            "paired_path_timestamp_ns": path_timestamp_ns,
            "paired_path_age_s": (timestamp_ns - int(path_timestamp_ns)) * 1.0e-9,
            "ego_s_m": float(pose.position.x),
            "ego_d_m": float(pose.position.y),
            "actual_x_m": float(sample["x"]),
            "actual_y_m": float(sample["y"]),
            "actual_yaw_rad": float(sample["yaw"]),
            "actual_speed_mps": float(sample["speed"]),
            "local_target_x_m": projection.x,
            "local_target_y_m": projection.y,
            "local_path_heading_rad": projection.heading,
            "local_path_curvature_radpm": projection.curvature,
            "local_path_curvature_rate_radpm2": projection.curvature_rate,
            "local_waypoint_speed_mps": projection.speed,
            "lateral_tracking_error_m": projection.lateral_error,
            "wallward_lateral_error_m": side_sign * projection.lateral_error,
            "heading_error_rad": heading_error,
            "heading_corner_protrusion_m": corner_protrusion,
            "planned_centerline_wall_clearance_m": planned_centerline,
            "planned_footprint_wall_clearance_m": planned_footprint,
            "planned_polygon_raster_wall_clearance_m": planned_exact,
            "actual_position_path_heading_clearance_m": position_path_heading,
            "actual_swept_footprint_wall_clearance_m": actual_exact,
            "actual_evaluator_wall_clearance_m": actual_evaluator,
            "obstacle_clearance_m": obstacle_clearance,
            "footprint_clearance_loss_m": max(0.0, planned_centerline - planned_exact),
            "tracking_position_clearance_loss_m": planned_exact - position_path_heading,
            "heading_rotation_clearance_loss_m": position_path_heading - actual_exact,
            "raster_sampling_clearance_loss_m": actual_exact - actual_evaluator,
            "steering_command_rad": steering,
            "steering_rate_radps": steering_rate,
            "speed_command_mps": float(drive_message.drive.speed),
            "avoidance_phase": phase,
            "path_mode": avoid_message.ot_line,
            "state": state,
            "is_minimum_clearance_sample": False,
            "is_pre_collision_0p5s": bool(
                collision_timestamp_ns is not None
                and int(collision_timestamp_ns) - 500_000_000 <= timestamp_ns
                <= int(collision_timestamp_ns)),
        }
        rows.append(row)
        previous_steering = steering
        previous_timestamp_ns = timestamp_ns

    if not rows:
        raise ValueError(f"no timestamp-paired avoidance samples in {episode_directory}")
    critical = min(rows, key=lambda row: float(row["actual_evaluator_wall_clearance_m"]))
    critical["is_minimum_clearance_sample"] = True
    critical_planned_exact, _ = probe.clearance(
        float(critical["local_target_x_m"]), float(critical["local_target_y_m"]),
        float(critical["local_path_heading_rad"]), vehicle_length, vehicle_width)
    critical_position_exact, _ = probe.clearance(
        float(critical["actual_x_m"]), float(critical["actual_y_m"]),
        float(critical["local_path_heading_rad"]), vehicle_length, vehicle_width)
    critical_actual_exact, _ = probe.clearance(
        float(critical["actual_x_m"]), float(critical["actual_y_m"]),
        float(critical["actual_yaw_rad"]), vehicle_length, vehicle_width)

    committed_paths: dict[tuple[tuple[float, ...], ...], Any] = {}
    for message in avoid.values():
        if message.ot_line != "raceline_local_d_offset_spline" or len(message.wpnts) < 2:
            continue
        key = tuple(
            (
                float(waypoint.x_m), float(waypoint.y_m), float(waypoint.psi_rad),
                float(waypoint.kappa_radpm), float(waypoint.d_m),
            )
            for waypoint in message.wpnts
        )
        committed_paths.setdefault(key, message)
    path_sampling = [
        _dense_path_minimum(
            list(message.wpnts), field, vehicle_length, vehicle_width,
            footprint_sample_step_m)
        for message in committed_paths.values()
    ]
    sampling_loss = max(
        (item["waypoint_sampling_loss_m"] for item in path_sampling), default=0.0)
    actual_sample_min = min(
        float(row["actual_swept_footprint_wall_clearance_m"]) for row in rows)
    actual_dense_min = _dense_actual_minimum(
        rows, field, vehicle_length, vehicle_width, footprint_sample_step_m)
    actual_temporal_sampling_loss = max(0.0, actual_sample_min - actual_dense_min)

    absolute_lateral = [abs(float(row["lateral_tracking_error_m"])) for row in rows]
    absolute_heading = [abs(float(row["heading_error_rad"])) for row in rows]
    swept_loss = [
        float(row["planned_polygon_raster_wall_clearance_m"])
        - float(row["actual_swept_footprint_wall_clearance_m"])
        for row in rows
    ]
    absolute_curvature = [abs(float(row["local_path_curvature_radpm"])) for row in rows]
    absolute_curvature_rate = [
        abs(float(row["local_path_curvature_rate_radpm2"])) for row in rows]
    absolute_steering = [abs(float(row["steering_command_rad"])) for row in rows]
    absolute_steering_rate = [abs(float(row["steering_rate_radps"])) for row in rows]
    audit = _candidate_audit(episode_directory)
    raster_geometry = episode_result["collision_diagnostics"]["raster_geometry"]
    pose_diagnostics = raster_geometry.get("pose_diagnostics") or {}

    contributions = {
        "A_centerline_already_close_m": max(
            0.0, 0.04 - float(critical["planned_centerline_wall_clearance_m"])),
        "B_footprint_rotation_from_centerline_m": max(
            0.0, float(critical["planned_centerline_wall_clearance_m"])
            - critical_planned_exact),
        "C_tracking_position_m": max(
            0.0, critical_planned_exact - critical_position_exact),
        "D_heading_rotation_m": max(
            0.0, critical_position_exact - critical_actual_exact),
        "E_waypoint_or_temporal_sampling_m": max(sampling_loss, actual_temporal_sampling_loss),
        "F_raster_or_ttc_envelope_m": max(
            0.0, critical_actual_exact
            - float(critical["actual_evaluator_wall_clearance_m"]))
        + float(pose_diagnostics.get("ttc_sweep_m") or 0.0)
        + float(pose_diagnostics.get("scan_noise_guard_m") or 0.0),
    }
    dominant = max(contributions, key=contributions.get)
    dominant_mapping = {
        "A_centerline_already_close_m": "A",
        "B_footprint_rotation_from_centerline_m": "B",
        "C_tracking_position_m": "C",
        "D_heading_rotation_m": "D",
        "E_waypoint_or_temporal_sampling_m": "E",
        "F_raster_or_ttc_envelope_m": "F",
    }
    phase_overshoot: dict[str, float] = {}
    for row in rows:
        phase = str(row["avoidance_phase"])
        phase_overshoot[phase] = max(
            phase_overshoot.get(phase, -math.inf),
            float(row["wallward_lateral_error_m"]),
        )

    summary = {
        "scenario_id": manifest["scenario_id"],
        "source_episode": str(episode_directory),
        "source_manifest": str(manifest_path),
        "source_hashes": {
            "episode_result": sha256_file(episode_directory / "episode_result.json"),
            "lockstep_result": sha256_file(episode_directory / "lockstep_result.json"),
            "scenario_manifest": sha256_file(manifest_path),
        },
        "valid": bool(episode_result["valid"]),
        "failure": episode_result["failure"],
        "selected_side": lockstep_result["selected_side"],
        "target_d_m": target_d,
        "candidate_audit": audit,
        "sample_count": len(rows),
        "critical_timestamp_ns": int(critical["timestamp_ns"]),
        "critical_simulated_time_s": float(critical["simulated_time_s"]),
        "collision_timestamp_ns": collision_timestamp_ns,
        "minimums_m": {
            "planned_centerline": min(
                float(row["planned_centerline_wall_clearance_m"]) for row in rows),
            "planned_footprint_evaluator": min(
                float(row["planned_footprint_wall_clearance_m"]) for row in rows),
            "planned_polygon_raster": min(
                float(row["planned_polygon_raster_wall_clearance_m"]) for row in rows),
            "actual_position_path_heading": min(
                float(row["actual_position_path_heading_clearance_m"]) for row in rows),
            "actual_polygon_sampled": actual_sample_min,
            "actual_polygon_temporally_dense": actual_dense_min,
            "actual_evaluator": min(
                float(row["actual_evaluator_wall_clearance_m"]) for row in rows),
            "obstacle": min(float(row["obstacle_clearance_m"]) for row in rows),
        },
        "critical_decomposition_m": {
            "planner_predicted_wall_headroom": audit.get("planner_wall_headroom_m"),
            "planned_centerline": float(critical["planned_centerline_wall_clearance_m"]),
            "planned_footprint_evaluator": float(critical["planned_footprint_wall_clearance_m"]),
            "planned_polygon_raster": critical_planned_exact,
            "actual_center_with_path_heading": critical_position_exact,
            "actual_polygon_raster": critical_actual_exact,
            "actual_evaluator": float(critical["actual_evaluator_wall_clearance_m"]),
            "lateral_tracking_error": float(critical["lateral_tracking_error_m"]),
            "wallward_lateral_error": float(critical["wallward_lateral_error_m"]),
            "heading_error_rad": float(critical["heading_error_rad"]),
            "heading_corner_protrusion": float(critical["heading_corner_protrusion_m"]),
        },
        "tracking_error_m": percentile_summary(absolute_lateral),
        "heading_error_rad": percentile_summary(absolute_heading),
        "heading_corner_protrusion_m": percentile_summary(
            float(row["heading_corner_protrusion_m"]) for row in rows),
        "swept_clearance_loss_m": percentile_summary(swept_loss),
        "path_sampling": path_sampling,
        "maximum_waypoint_sampling_loss_m": sampling_loss,
        "actual_temporal_sampling_loss_m": actual_temporal_sampling_loss,
        "phase_wallward_overshoot_m": phase_overshoot,
        "collision_raster": {
            "external_source": episode_result["collision_diagnostics"]["external_source"],
            "recorded_scan_cause": external.get("cause"),
            "fallback_cause": raster_geometry.get("cause"),
            "first_collision_base_raster_overlap": pose_diagnostics.get("base_raster_overlap"),
            "raster_clearance_at_first_fallback_collision_m": pose_diagnostics.get(
                "raster_clearance_m"),
            "ttc_sweep_m": pose_diagnostics.get("ttc_sweep_m"),
            "scan_noise_guard_m": pose_diagnostics.get("scan_noise_guard_m"),
            "yaw_repair": yaw_repair,
        },
        "correlations": {
            "abs_lateral_error_vs_speed": pearson(
                absolute_lateral, (float(row["actual_speed_mps"]) for row in rows)),
            "abs_lateral_error_vs_abs_curvature": pearson(
                absolute_lateral, absolute_curvature),
            "abs_lateral_error_vs_abs_curvature_rate": pearson(
                absolute_lateral, absolute_curvature_rate),
            "abs_heading_error_vs_abs_curvature": pearson(
                absolute_heading, absolute_curvature),
            "swept_loss_vs_abs_curvature": pearson(swept_loss, absolute_curvature),
            "swept_loss_vs_abs_steering": pearson(swept_loss, absolute_steering),
            "swept_loss_vs_abs_steering_rate": pearson(swept_loss, absolute_steering_rate),
        },
        "cause_contributions_m": contributions,
        "dominant_failure_cause": dominant_mapping[dominant],
        "dominant_failure_cause_key": dominant,
    }
    decomposition = {
        "scenario_id": manifest["scenario_id"],
        **summary["critical_decomposition_m"],
        "waypoint_sampling_loss_m": sampling_loss,
        "actual_temporal_sampling_loss_m": actual_temporal_sampling_loss,
        "raster_ttc_sweep_m": pose_diagnostics.get("ttc_sweep_m"),
        "raster_noise_guard_m": pose_diagnostics.get("scan_noise_guard_m"),
        "dominant_failure_cause": summary["dominant_failure_cause"],
    }
    _write_csv(Path(output_csv), rows, TIMESERIES_FIELDS)
    return summary, decomposition


def aggregate_analysis(
    scenarios: list[tuple[str | Path, str | Path]], output_directory: str | Path,
    footprint_sample_step_m: float = 0.0125,
    collision_configuration: dict[str, Any] | None = None,
) -> dict[str, Any]:
    output_directory = Path(output_directory).resolve()
    timeseries_directory = output_directory / "tracking_timeseries"
    summaries = []
    decompositions = []
    all_rows: list[dict[str, str]] = []
    for episode, manifest in scenarios:
        scenario_id = json.loads(Path(manifest).read_text(encoding="utf-8"))["scenario_id"]
        csv_path = timeseries_directory / f"{scenario_id}.csv"
        summary, decomposition = analyze_episode(
            episode, manifest, csv_path, footprint_sample_step_m,
            collision_configuration)
        summaries.append(summary)
        decompositions.append(decomposition)
        with csv_path.open("r", encoding="utf-8", newline="") as stream:
            all_rows.extend(csv.DictReader(stream))

    absolute_lateral = [abs(float(row["lateral_tracking_error_m"])) for row in all_rows]
    swept_loss = [
        float(row["planned_polygon_raster_wall_clearance_m"])
        - float(row["actual_swept_footprint_wall_clearance_m"])
        for row in all_rows
    ]
    speed = [float(row["actual_speed_mps"]) for row in all_rows]
    curvature = [abs(float(row["local_path_curvature_radpm"])) for row in all_rows]
    curvature_rate = [abs(float(row["local_path_curvature_rate_radpm2"])) for row in all_rows]
    heading = [abs(float(row["heading_error_rad"])) for row in all_rows]
    steering = [abs(float(row["steering_command_rad"])) for row in all_rows]
    steering_rate = [abs(float(row["steering_rate_radps"])) for row in all_rows]
    aggregate = {
        "schema": "tracking_swept_footprint_analysis/1",
        "source_policy": "read_only_existing_artifacts",
        "vehicle_footprint": {
            "length_m": 0.56, "width_m": 0.287,
            "reference_point": "base_link_centered",
        },
        "scenario_count": len(summaries),
        "sample_count": len(all_rows),
        "scenarios": summaries,
        "aggregate_tracking_error_m": percentile_summary(absolute_lateral),
        "aggregate_swept_clearance_loss_m": percentile_summary(swept_loss),
        "aggregate_correlations": {
            "abs_lateral_error_vs_speed": pearson(absolute_lateral, speed),
            "abs_lateral_error_vs_abs_curvature": pearson(absolute_lateral, curvature),
            "abs_lateral_error_vs_abs_curvature_rate": pearson(
                absolute_lateral, curvature_rate),
            "abs_heading_error_vs_abs_curvature": pearson(heading, curvature),
            "swept_loss_vs_abs_curvature": pearson(swept_loss, curvature),
            "swept_loss_vs_abs_steering": pearson(swept_loss, steering),
            "swept_loss_vs_abs_steering_rate": pearson(swept_loss, steering_rate),
        },
        "fixed_tracking_reserve_m": 0.14,
        "production_parameter_changed": False,
        "cma_executed": False,
    }
    output_directory.mkdir(parents=True, exist_ok=True)
    atomic_write_json(output_directory / "summary.json", aggregate)
    scenario_fields = [
        "scenario_id", "selected_side", "target_d_m", "dominant_failure_cause",
        "sample_count", "planner_predicted_headroom_m", "planned_centerline_min_m",
        "planned_footprint_min_m", "actual_polygon_min_m", "actual_evaluator_min_m",
        "obstacle_min_m", "lateral_error_max_m", "lateral_error_p95_m",
        "lateral_error_p99_m", "heading_error_max_rad", "heading_error_p95_rad",
        "swept_loss_max_m", "swept_loss_p95_m", "swept_loss_p99_m",
        "waypoint_sampling_loss_m", "actual_temporal_sampling_loss_m",
    ]
    scenario_rows = []
    for summary in summaries:
        scenario_rows.append({
            "scenario_id": summary["scenario_id"],
            "selected_side": summary["selected_side"],
            "target_d_m": summary["target_d_m"],
            "dominant_failure_cause": summary["dominant_failure_cause"],
            "sample_count": summary["sample_count"],
            "planner_predicted_headroom_m": summary["candidate_audit"].get(
                "planner_wall_headroom_m"),
            "planned_centerline_min_m": summary["minimums_m"]["planned_centerline"],
            "planned_footprint_min_m": summary["minimums_m"]["planned_polygon_raster"],
            "actual_polygon_min_m": summary["minimums_m"]["actual_polygon_temporally_dense"],
            "actual_evaluator_min_m": summary["minimums_m"]["actual_evaluator"],
            "obstacle_min_m": summary["minimums_m"]["obstacle"],
            "lateral_error_max_m": summary["tracking_error_m"]["max"],
            "lateral_error_p95_m": summary["tracking_error_m"]["p95"],
            "lateral_error_p99_m": summary["tracking_error_m"]["p99"],
            "heading_error_max_rad": summary["heading_error_rad"]["max"],
            "heading_error_p95_rad": summary["heading_error_rad"]["p95"],
            "swept_loss_max_m": summary["swept_clearance_loss_m"]["max"],
            "swept_loss_p95_m": summary["swept_clearance_loss_m"]["p95"],
            "swept_loss_p99_m": summary["swept_clearance_loss_m"]["p99"],
            "waypoint_sampling_loss_m": summary["maximum_waypoint_sampling_loss_m"],
            "actual_temporal_sampling_loss_m": summary["actual_temporal_sampling_loss_m"],
        })
    _write_csv(output_directory / "scenario_summary.csv", scenario_rows, scenario_fields)
    decomposition_fields = list(decompositions[0]) if decompositions else []
    _write_csv(
        output_directory / "clearance_decomposition.csv",
        decompositions,
        decomposition_fields,
    )
    return aggregate
