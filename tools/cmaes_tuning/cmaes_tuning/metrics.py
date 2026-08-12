"""Metric primitives for one closed-loop static-obstacle episode."""

from __future__ import annotations

import hashlib
import math
from typing import Any

import numpy as np

from .bag_reader import MessageRecord
from .scenario_generator import load_waypoints, track_length


def quaternion_yaw(quaternion: Any) -> float:
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


def odom_samples(records: list[MessageRecord]) -> list[dict[str, float]]:
    samples = []
    for record in records:
        pose = record.message.pose.pose
        twist = record.message.twist.twist
        samples.append(
            {
                "timestamp_ns": record.timestamp_ns,
                "header_timestamp_ns": int(record.message.header.stamp.sec) * 1_000_000_000
                + int(record.message.header.stamp.nanosec),
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "yaw": quaternion_yaw(pose.orientation),
                "speed": math.hypot(float(twist.linear.x), float(twist.linear.y)),
                "yaw_rate": float(twist.angular.z),
            }
        )
    return samples


def progress_metrics(
    samples: list[dict[str, float]],
    waypoint_path: str,
    start_speed_threshold: float,
    lap_fraction: float,
) -> dict[str, Any]:
    reference = load_waypoints(waypoint_path)
    length = track_length(reference)
    xy = np.asarray([(float(wp["x_m"]), float(wp["y_m"])) for wp in reference])
    reference_s = np.asarray([float(wp["s_m"]) for wp in reference])
    reference_v = np.asarray([float(wp["vx_mps"]) for wp in reference])
    start_index = next(
        (index for index, sample in enumerate(samples) if sample["speed"] > start_speed_threshold),
        None,
    )
    if start_index is None:
        return {
            "started": False,
            "start_timestamp_ns": samples[0]["timestamp_ns"] if samples else 0,
            "end_timestamp_ns": samples[-1]["timestamp_ns"] if samples else 0,
            "progress_m": 0.0,
            "progress_fraction": 0.0,
            "completed": False,
            "completion_time_s": 0.0,
            "track_length_m": length,
            "reference": reference,
            "reference_xy": xy,
            "reference_s": reference_s,
            "reference_v": reference_v,
            "nearest_indices": [],
        }
    running = samples[start_index:]
    nearest = [
        int(np.argmin(np.square(xy[:, 0] - sample["x"]) + np.square(xy[:, 1] - sample["y"])))
        for sample in running
    ]
    progress = 0.0
    completion_timestamp = None
    last_s = float(reference_s[nearest[0]])
    for sample, index in zip(running[1:], nearest[1:]):
        current_s = float(reference_s[index])
        forward = (current_s - last_s) % length
        if forward <= 0.5 * length:
            progress += forward
        last_s = current_s
        if completion_timestamp is None and progress >= length * lap_fraction:
            completion_timestamp = sample["timestamp_ns"]
    end_timestamp = running[-1]["timestamp_ns"]
    completed = completion_timestamp is not None
    duration_end = completion_timestamp if completed else end_timestamp
    return {
        "started": True,
        "start_timestamp_ns": running[0]["timestamp_ns"],
        "end_timestamp_ns": end_timestamp,
        "progress_m": progress,
        "progress_fraction": min(1.0, progress / (length * lap_fraction)),
        "completed": completed,
        "completion_time_s": (duration_end - running[0]["timestamp_ns"]) * 1.0e-9,
        "track_length_m": length,
        "reference": reference,
        "reference_xy": xy,
        "reference_s": reference_s,
        "reference_v": reference_v,
        "nearest_indices": nearest,
        "running_samples": running,
    }


def steering_metrics(
    records: list[MessageRecord], start_ns: int, end_ns: int, rate_deadband: float
) -> dict[str, float]:
    selected = [
        (record.timestamp_ns, float(record.message.drive.steering_angle))
        for record in records
        if start_ns <= record.timestamp_ns <= end_ns
    ]
    if len(selected) < 2:
        return {"total_variation_rad": 0.0, "total_variation_per_s": 0.0, "oscillations_per_s": 0.0}
    duration = max(1.0e-9, (selected[-1][0] - selected[0][0]) * 1.0e-9)
    variation = sum(abs(second[1] - first[1]) for first, second in zip(selected, selected[1:]))
    rates = []
    for first, second in zip(selected, selected[1:]):
        dt = (second[0] - first[0]) * 1.0e-9
        if dt > 1.0e-6:
            rates.append((second[1] - first[1]) / dt)
    signs = [1 if rate > rate_deadband else -1 if rate < -rate_deadband else 0 for rate in rates]
    active_signs = [sign for sign in signs if sign]
    reversals = sum(first != second for first, second in zip(active_signs, active_signs[1:]))
    return {
        "total_variation_rad": variation,
        "total_variation_per_s": variation / duration,
        "oscillations_per_s": reversals / duration,
    }


def _path_hash(message: Any) -> str:
    digest = hashlib.sha256()
    for waypoint in message.wpnts:
        digest.update(
            f"{waypoint.x_m:.6f},{waypoint.y_m:.6f},{waypoint.kappa_radpm:.6f};".encode()
        )
    return digest.hexdigest()


def planned_path_metrics(records: list[MessageRecord]) -> dict[str, float | int]:
    unique: dict[str, Any] = {}
    for record in records:
        message = record.message
        if message.ot_line != "raceline_local_d_offset_spline" or len(message.wpnts) < 2:
            continue
        unique.setdefault(_path_hash(message), message)
    maximum_curvature = 0.0
    rates: list[float] = []
    lengths: list[float] = []
    for message in unique.values():
        length = 0.0
        for first, second in zip(message.wpnts, message.wpnts[1:]):
            distance = math.hypot(second.x_m - first.x_m, second.y_m - first.y_m)
            if distance <= 1.0e-6:
                continue
            length += distance
            rates.append((float(second.kappa_radpm) - float(first.kappa_radpm)) / distance)
        lengths.append(length)
        maximum_curvature = max(
            maximum_curvature,
            max(abs(float(waypoint.kappa_radpm)) for waypoint in message.wpnts),
        )
    rms = math.sqrt(sum(rate * rate for rate in rates) / len(rates)) if rates else 0.0
    return {
        "unique_path_count": len(unique),
        "maximum_curvature_radpm": maximum_curvature,
        "curvature_rate_rms_radpm2": rms,
        "mean_path_length_m": sum(lengths) / len(lengths) if lengths else 0.0,
    }


def avoidance_window(
    avoid_records: list[MessageRecord], state_records: list[MessageRecord], run_start_ns: int
) -> dict[str, int | float | None]:
    start = next(
        (
            record.timestamp_ns
            for record in avoid_records
            if record.timestamp_ns >= run_start_ns
            and record.message.ot_line == "raceline_local_d_offset_spline"
            and record.message.wpnts
        ),
        None,
    )
    if start is None:
        return {"start_ns": None, "completion_ns": None, "start_s": -1.0, "completion_s": -1.0}
    handoff = next(
        (
            record.timestamp_ns
            for record in avoid_records
            if record.timestamp_ns >= start and record.message.ot_line == "raceline_global_handoff"
        ),
        None,
    )
    state_avoid_seen = False
    state_completion = None
    for record in state_records:
        if record.timestamp_ns < start:
            continue
        if int(record.message.state) == 1:
            state_avoid_seen = True
        elif state_avoid_seen and int(record.message.state) == 0:
            state_completion = record.timestamp_ns
            break
    completion = state_completion or handoff
    return {
        "start_ns": start,
        "completion_ns": completion,
        "start_s": (start - run_start_ns) * 1.0e-9,
        "completion_s": (completion - run_start_ns) * 1.0e-9 if completion else -1.0,
    }


def speed_loss_during_avoidance(
    progress: dict[str, Any], start_ns: int | None, completion_ns: int | None
) -> float:
    if start_ns is None:
        return 0.0
    end_ns = completion_ns or progress["end_timestamp_ns"]
    reference_xy = progress["reference_xy"]
    reference_v = progress["reference_v"]
    losses = []
    for sample in progress.get("running_samples", []):
        if not start_ns <= sample["timestamp_ns"] <= end_ns:
            continue
        index = int(
            np.argmin(
                np.square(reference_xy[:, 0] - sample["x"])
                + np.square(reference_xy[:, 1] - sample["y"])
            )
        )
        target = float(reference_v[index])
        if target > 1.0e-6:
            losses.append(max(0.0, target - sample["speed"]) / target)
    return sum(losses) / len(losses) if losses else 0.0
