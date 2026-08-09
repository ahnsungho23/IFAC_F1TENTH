"""Independent external evaluator for a completed episode."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

from .bag_reader import read_bag
from .geometry import OccupancyDistanceField, footprint_obstacle_clearance
from .metrics import (
    avoidance_window,
    odom_samples,
    planned_path_metrics,
    progress_metrics,
    speed_loss_during_avoidance,
    steering_metrics,
)
from .objective import episode_cost
from .schemas import atomic_write_json, sha256_file


RECORDED_TOPICS = (
    "/ego_racecar/odom",
    "/drive",
    "/drive_autonomous",
    "/ego_racecar/collision",
    "/avoid_waypoints",
    "/local_waypoints",
    "/global_waypoints",
    "/state",
    "/static_obs",
)


def _load_json(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _verify_manifest(manifest: dict[str, Any]) -> list[str]:
    failures = []
    checks = (
        ("clean_map_yaml", "clean_map_yaml_sha256"),
        ("clean_map_image", "clean_map_image_sha256"),
        ("baked_map_yaml", "baked_map_yaml_sha256"),
        ("baked_map_image", "baked_map_image_sha256"),
        ("waypoint_file", "waypoint_sha256"),
        ("simulator_config", "simulator_config_sha256"),
    )
    for path_key, hash_key in checks:
        path = Path(manifest[path_key])
        if not path.is_file():
            failures.append(f"missing_artifact:{path_key}")
        elif sha256_file(path) != manifest[hash_key]:
            failures.append(f"hash_mismatch:{path_key}")
    return failures


def _planner_failure(
    bag: Any,
    completed: bool,
    evaluation: dict[str, Any],
) -> tuple[bool, dict[str, Any]]:
    static_records = bag.topic("/static_obs")
    detected_records = [record for record in static_records if record.message.obstacles]
    avoid_records = bag.topic("/avoid_waypoints")
    valid_paths = [
        record
        for record in avoid_records
        if record.message.wpnts
        and record.message.ot_line in (
            "raceline_local_d_offset_spline",
            "raceline_global_handoff",
        )
    ]
    preparation_paths = [
        record
        for record in avoid_records
        if record.message.wpnts
        and record.message.ot_line == "raceline_static_prepare"
    ]
    usable_paths = valid_paths + preparation_paths
    safe_stops = [
        record
        for record in avoid_records
        if record.message.ot_line == "raceline_static_safe_stop"
    ]
    terminal_safe_stop = False
    safe_stop_duration = 0.0
    if safe_stops:
        # A preparation prefix is a valid stabilization output. Only a sustained
        # safe-stop after the last usable output is a planner failure.
        last_valid_ns = max((record.timestamp_ns for record in usable_paths), default=-1)
        terminal_stops = [record for record in safe_stops if record.timestamp_ns > last_valid_ns]
        if terminal_stops:
            episode_end_ns = max(
                (record.timestamp_ns for record in avoid_records),
                default=terminal_stops[-1].timestamp_ns,
            )
            safe_stop_duration = (episode_end_ns - terminal_stops[0].timestamp_ns) * 1.0e-9
            terminal_safe_stop = safe_stop_duration >= float(
                evaluation["safe_stop_failure_duration_sec"]
            )
    no_path_after_detection = False
    if detected_records:
        first_detection = detected_records[0].timestamp_ns
        no_path_after_detection = not any(
            record.timestamp_ns >= first_detection for record in usable_paths
        )
    planner_failure = bool(detected_records) and (terminal_safe_stop or (no_path_after_detection and not completed))
    return planner_failure, {
        "obstacle_detected": bool(detected_records),
        "first_detection_ns": detected_records[0].timestamp_ns if detected_records else None,
        "valid_avoidance_path_messages": len(valid_paths),
        "preparation_path_messages": len(preparation_paths),
        "safe_stop_messages": len(safe_stops),
        "terminal_safe_stop": terminal_safe_stop,
        "safe_stop_duration_s": safe_stop_duration,
        "no_path_after_detection": no_path_after_detection,
    }


def evaluate_episode(
    bag_directory: str | Path,
    scenario_manifest_path: str | Path,
    lap_summary_path: str | Path,
    runner_status_path: str | Path,
    config: dict[str, Any],
    output_path: str | Path | None = None,
) -> dict[str, Any]:
    manifest = _load_json(scenario_manifest_path)
    runner_status = _load_json(runner_status_path)
    infrastructure_failures = list(runner_status.get("infrastructure_failures", []))
    infrastructure_failures.extend(_verify_manifest(manifest))
    try:
        lap_summary = _load_json(lap_summary_path)
    except (FileNotFoundError, json.JSONDecodeError):
        lap_summary = {}
        infrastructure_failures.append("missing_or_invalid_lap_summary")

    try:
        bag = read_bag(bag_directory, RECORDED_TOPICS)
    except Exception as error:
        result = {
            "schema": "cmaes_episode_result/1",
            "valid": False,
            "classification": "invalid_episode",
            "infrastructure_failures": infrastructure_failures + [f"bag_error:{error}"],
            "scenario_id": manifest.get("scenario_id", "unknown"),
        }
        if output_path:
            atomic_write_json(output_path, result)
        return result

    required_nonempty = (
        "/ego_racecar/odom",
        "/drive",
        "/drive_autonomous",
        "/ego_racecar/collision",
        "/avoid_waypoints",
        "/global_waypoints",
        "/state",
        "/static_obs",
    )
    for topic in required_nonempty:
        if not bag.topic(topic):
            infrastructure_failures.append(f"empty_required_topic:{topic}")
    if infrastructure_failures:
        result = {
            "schema": "cmaes_episode_result/1",
            "valid": False,
            "classification": "invalid_episode",
            "infrastructure_failures": sorted(set(infrastructure_failures)),
            "scenario_id": manifest["scenario_id"],
        }
        if output_path:
            atomic_write_json(output_path, result)
        return result

    evaluation = config["evaluation"]
    samples = odom_samples(bag.topic("/ego_racecar/odom"))
    progress = progress_metrics(
        samples,
        manifest["waypoint_file"],
        float(evaluation["start_speed_threshold_mps"]),
        float(evaluation["lap_fraction"]),
    )
    wall_field = OccupancyDistanceField(manifest["clean_map_yaml"])
    minimum_obstacle_clearance = math.inf
    minimum_wall_clearance = math.inf
    geometry_collision = False
    off_track = False
    for sample in progress.get("running_samples", samples):
        clearance, overlap = footprint_obstacle_clearance(
            sample["x"],
            sample["y"],
            sample["yaw"],
            float(manifest["vehicle_length_m"]),
            float(manifest["vehicle_width_m"]),
            manifest["obstacle"],
        )
        minimum_obstacle_clearance = min(minimum_obstacle_clearance, clearance)
        geometry_collision = geometry_collision or overlap
        wall_clearance, wall_overlap = wall_field.footprint_clearance(
            sample["x"],
            sample["y"],
            sample["yaw"],
            float(manifest["vehicle_length_m"]),
            float(manifest["vehicle_width_m"]),
            float(evaluation["footprint_sample_step_m"]),
        )
        minimum_wall_clearance = min(minimum_wall_clearance, wall_clearance)
        off_track = off_track or wall_overlap
    if not math.isfinite(minimum_obstacle_clearance):
        minimum_obstacle_clearance = 0.0
    if not math.isfinite(minimum_wall_clearance):
        minimum_wall_clearance = 0.0

    collision_topic = any(
        bool(record.message.data) for record in bag.topic("/ego_racecar/collision")
    )
    collision = collision_topic or geometry_collision
    planner_failure, planner_diagnostics = _planner_failure(
        bag, bool(progress["completed"]), evaluation
    )
    hard_failure = collision or off_track or planner_failure
    completion_failure = not bool(progress["completed"]) and not hard_failure
    if hard_failure:
        classification = "safety_failure"
    elif completion_failure:
        classification = "completion_failure"
    else:
        classification = "success"

    start_ns = int(progress["start_timestamp_ns"])
    end_ns = int(progress["end_timestamp_ns"])
    steering = steering_metrics(
        bag.topic("/drive"),
        start_ns,
        end_ns,
        float(evaluation["steering_rate_deadband_radps"]),
    )
    planned = planned_path_metrics(bag.topic("/avoid_waypoints"))
    avoidance = avoidance_window(
        bag.topic("/avoid_waypoints"), bag.topic("/state"), start_ns
    )
    metrics = {
        "completed": bool(progress["completed"]),
        "termination_reason": str(lap_summary.get("terminated", "unknown")),
        "episode_timeout_s": float(config["runner"]["episode_timeout_sec"]),
        "completion_time_s": float(progress["completion_time_s"]),
        "progress_m": float(progress["progress_m"]),
        "progress_fraction": float(progress["progress_fraction"]),
        "remaining_progress_fraction": 0.0
        if progress["completed"]
        else 1.0 - float(progress["progress_fraction"]),
        "minimum_obstacle_clearance_m": minimum_obstacle_clearance,
        "minimum_wall_clearance_m": minimum_wall_clearance,
        "steering_total_variation_rad": steering["total_variation_rad"],
        "steering_total_variation_per_s": steering["total_variation_per_s"],
        "steering_oscillations_per_s": steering["oscillations_per_s"],
        "planned_path_max_curvature_radpm": planned["maximum_curvature_radpm"],
        "planned_curvature_rate_rms_radpm2": planned["curvature_rate_rms_radpm2"],
        "planned_path_mean_length_m": planned["mean_path_length_m"],
        "unique_planned_path_count": planned["unique_path_count"],
        "avoidance_start_s": avoidance["start_s"],
        "avoidance_completion_s": avoidance["completion_s"],
        "speed_loss_during_avoidance_fraction": speed_loss_during_avoidance(
            progress, avoidance["start_ns"], avoidance["completion_ns"]
        ),
    }
    result = {
        "schema": "cmaes_episode_result/1",
        "valid": True,
        "classification": classification,
        "infrastructure_failures": [],
        "scenario_id": manifest["scenario_id"],
        "failure": {
            "collision": collision,
            "collision_topic": collision_topic,
            "ground_truth_obstacle_overlap": geometry_collision,
            "off_track": off_track,
            "planner_failure": planner_failure,
            "completion_failure": completion_failure,
        },
        "planner_diagnostics": planner_diagnostics,
        "metrics": metrics,
        "lap_referee_summary": lap_summary,
        "bag_topics": {name: len(bag.topic(name)) for name in RECORDED_TOPICS},
    }
    result["performance_cost"] = episode_cost(metrics, config)
    if output_path:
        atomic_write_json(output_path, result)
    return result
