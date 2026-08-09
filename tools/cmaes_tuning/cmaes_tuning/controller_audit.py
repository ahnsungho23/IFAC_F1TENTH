"""Offline audit of planner velocity propagation and controller forwarding."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .bag_reader import BagData, read_bag
from .schemas import atomic_write_json


AUDIT_TOPICS = (
    "/avoid_waypoints",
    "/local_waypoints",
    "/drive_autonomous",
    "/drive",
)


def _waypoint_speeds(records: list[Any], *, overtake: bool) -> list[float]:
    speeds = []
    for record in records:
        message = record.message
        waypoints = message.wpnts
        if overtake and getattr(message, "ot_line", "") not in (
            "raceline_local_d_offset_spline",
            "raceline_global_handoff",
            "raceline_static_safe_stop",
        ):
            continue
        speeds.extend(float(waypoint.vx_mps) for waypoint in waypoints)
    return speeds


def _drive_speeds(records: list[Any]) -> list[float]:
    return [float(record.message.drive.speed) for record in records]


def _selector_alignment(autonomous_records: list[Any], drive_records: list[Any]) -> tuple[int, float]:
    if not autonomous_records or not drive_records:
        return 0, 0.0
    matched = 0
    maximum_error = 0.0
    drive_index = 0
    for autonomous in autonomous_records:
        while (
            drive_index + 1 < len(drive_records)
            and abs(drive_records[drive_index + 1].timestamp_ns - autonomous.timestamp_ns)
            <= abs(drive_records[drive_index].timestamp_ns - autonomous.timestamp_ns)
        ):
            drive_index += 1
        drive_record = drive_records[drive_index]
        if abs(drive_record.timestamp_ns - autonomous.timestamp_ns) > 50_000_000:
            continue
        matched += 1
        maximum_error = max(
            maximum_error,
            abs(
                float(autonomous.message.drive.speed)
                - float(drive_record.message.drive.speed)
            ),
        )
    return matched, maximum_error


def audit_bag(bag_directory: str | Path, controller_config: dict[str, Any]) -> dict[str, Any]:
    bag: BagData = read_bag(bag_directory, AUDIT_TOPICS)
    avoid = _waypoint_speeds(bag.topic("/avoid_waypoints"), overtake=True)
    local = _waypoint_speeds(bag.topic("/local_waypoints"), overtake=False)
    autonomous = _drive_speeds(bag.topic("/drive_autonomous"))
    delivered = _drive_speeds(bag.topic("/drive"))
    matched_count, selector_max_error = _selector_alignment(
        bag.topic("/drive_autonomous"), bag.topic("/drive")
    )
    positive_avoid = [value for value in avoid if value > 1.0e-6]
    positive_local = [value for value in local if value > 1.0e-6]
    result = {
        "schema": "cmaes_controller_audit/1",
        "planner_avoid_waypoint_count": len(avoid),
        "state_machine_local_waypoint_count": len(local),
        "drive_autonomous_count": len(autonomous),
        "drive_count": len(delivered),
        "selector_timestamp_matched_count": matched_count,
        "planner_min_positive_velocity_mps": min(positive_avoid) if positive_avoid else -1.0,
        "local_min_positive_velocity_mps": min(positive_local) if positive_local else -1.0,
        "drive_autonomous_min_mps": min(autonomous) if autonomous else -1.0,
        "drive_min_mps": min(delivered) if delivered else -1.0,
        "selector_max_index_aligned_speed_error_mps": selector_max_error,
        "configured_min_speed_mps": float(controller_config["min_speed_mps"]),
        "configured_max_speed_mps": float(controller_config["max_speed_mps"]),
        "planner_velocity_below_configured_min_speed": bool(
            positive_local
            and min(positive_local) < float(controller_config["min_speed_mps"]) - 1.0e-6
        ),
        "source_contract": {
            "planner_velocity": "Wpnt.vx_mps in /avoid_waypoints",
            "state_machine_velocity": "Wpnt.vx_mps in /local_waypoints",
            "controller_output": "/drive_autonomous",
            "simulator_input": "/drive",
            "min_speed_effect": (
                "control_map_node applies max(min_speed, curvature_speed_limit) before taking "
                "min with the selected waypoint velocity; it can weaken future-curvature "
                "pre-braking but does not raise an explicit zero-speed waypoint"
            ),
        },
    }
    return result


def audit_to_file(
    bag_directory: str | Path,
    controller_config: dict[str, Any],
    output_path: str | Path,
) -> dict[str, Any]:
    result = audit_bag(bag_directory, controller_config)
    atomic_write_json(output_path, result)
    return result
