"""Offline diagnostics for repeatability noise sources."""

from __future__ import annotations

from bisect import bisect_left
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any

import numpy as np

from .bag_reader import MessageRecord, read_bag
from .metrics import odom_samples, quaternion_yaw
from .simulator_collision import SimulatorRasterCollisionModel


TOPICS = (
    "/ego_racecar/odom",
    "/scan",
    "/ego_racecar/scan_identity",
    "/pf/pose/odom",
    "/car_state/frenet/odom",
    "/drive",
    "/drive_autonomous",
    "/static_obs",
    "/confirmed_static_obs",
    "/avoid_waypoints",
    "/cma_timing/events",
    "/tf",
)


def _statistics(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "mean": None, "std": None, "p95": None, "max": None}
    array = np.asarray(values, dtype=np.float64)
    return {
        "count": int(array.size),
        "mean": float(np.mean(array)),
        "std": float(np.std(array, ddof=1)) if array.size > 1 else 0.0,
        "p95": float(np.percentile(array, 95.0)),
        "max": float(np.max(array)),
    }


def _interval_statistics(records: list[MessageRecord]) -> dict[str, float | int | None]:
    intervals = [
        (second.timestamp_ns - first.timestamp_ns) * 1.0e-3
        for first, second in zip(records, records[1:])
        if second.timestamp_ns > first.timestamp_ns
    ]
    return _statistics(intervals)


def _nearest_index(timestamps: list[int], target: int) -> int | None:
    if not timestamps:
        return None
    index = bisect_left(timestamps, target)
    choices = [item for item in (index - 1, index) if 0 <= item < len(timestamps)]
    return min(choices, key=lambda item: abs(timestamps[item] - target))


def _header_timestamp_ns(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _ranges_sha256(message: Any) -> str:
    payload = np.asarray(message.ranges, dtype="<f4").tobytes()
    return hashlib.sha256(payload).hexdigest()


def chain_timing_events(events: list[dict[str, Any]]) -> dict[str, Any]:
    """Select the first causal T0-T9 chain from monotonic companion events."""
    ordered = sorted(events, key=lambda item: int(item["steady_time_ns"]))

    def first(name: str, predicate: Any = None) -> dict[str, Any] | None:
        for item in ordered:
            if item.get("event") == name and (predicate is None or predicate(item)):
                return dict(item)
        return None

    selected: dict[str, dict[str, Any] | None] = {
        "T0": first("T0_STATIC_OBS"),
        "T1": first("T1_AVOID_WAYPOINTS"),
        "T2": first("T2_STATE_CONFIRMATION"),
        "T3": first("T3_STATE_TRANSITION"),
        "T4": first("T4_LOCAL_WAYPOINTS"),
        "T5": first("T5_CONTROL_CONSUME"),
        "T6": first("T6_DRIVE_AUTONOMOUS"),
    }
    t6 = selected["T6"]
    selected["T7"] = first(
        "T7_DRIVE",
        lambda item: t6 is not None and int(item.get("input_drive_stamp_ns", -1)) ==
        int(t6.get("drive_stamp_ns", -2)),
    )
    t7 = selected["T7"]
    selected["T8"] = first(
        "T8_SIM_RECEIVE",
        lambda item: t7 is not None and int(item.get("drive_stamp_ns", -1)) ==
        int(t7.get("drive_stamp_ns", -2)),
    )
    t8 = selected["T8"]
    selected["T9"] = first(
        "T9_PHYSICS_APPLY",
        lambda item: t8 is not None and int(item.get("drive_receive_sequence", -1)) ==
        int(t8.get("drive_receive_sequence", -2)),
    )

    latency_pairs = {
        "t3_minus_t2": ("T2", "T3"),
        "t4_minus_t3": ("T3", "T4"),
        "t5_minus_t4": ("T4", "T5"),
        "t8_minus_t6": ("T6", "T8"),
        "t9_minus_t8": ("T8", "T9"),
        "t9_minus_t1": ("T1", "T9"),
    }
    latencies = {}
    for name, (start_name, end_name) in latency_pairs.items():
        start = selected[start_name]
        end = selected[end_name]
        latencies[name] = (
            (int(end["steady_time_ns"]) - int(start["steady_time_ns"])) * 1.0e-6
            if start is not None and end is not None else None
        )
    monotonic_order_valid = all(
        selected[first_name] is None or selected[second_name] is None or
        int(selected[first_name]["steady_time_ns"]) <=
        int(selected[second_name]["steady_time_ns"])
        for first_name, second_name in zip(
            ("T0", "T1", "T2", "T3", "T4", "T5", "T6", "T7", "T8"),
            ("T1", "T2", "T3", "T4", "T5", "T6", "T7", "T8", "T9"),
        )
    )
    return {
        "events": selected,
        "latencies_ms": latencies,
        "complete_chain": all(item is not None for item in selected.values()),
        "monotonic_order_valid": monotonic_order_valid,
    }


def _end_to_end_timing_diagnostics(
    timing_records: list[MessageRecord],
    identity_records: list[MessageRecord],
    frenet_records: list[MessageRecord],
    planner_log: dict[str, Any],
) -> dict[str, Any]:
    events = []
    invalid_count = 0
    for record in timing_records:
        try:
            item = json.loads(record.message.data)
        except (AttributeError, TypeError, json.JSONDecodeError):
            invalid_count += 1
            continue
        if item.get("schema") != "cma_timing_event/1" or "steady_time_ns" not in item:
            invalid_count += 1
            continue
        events.append(item)

    identities = []
    for record in identity_records:
        try:
            item = json.loads(record.message.data)
        except (AttributeError, TypeError, json.JSONDecodeError):
            continue
        if item.get("schema") == "f1tenth_scan_identity/1" and \
                "publish_steady_time_ns" in item:
            identities.append(item)
    identities.sort(key=lambda item: int(item["publish_steady_time_ns"]))
    identity_times = [int(item["publish_steady_time_ns"]) for item in identities]

    frenet_by_time = sorted(
        ((_header_timestamp_ns(record.message), record.message) for record in frenet_records),
        key=lambda item: item[0],
    )
    frenet_times = [item[0] for item in frenet_by_time]
    initial_target = planner_log.get("initial_commit_target_d_m")
    initial_obstacle_id = None
    chained = chain_timing_events(events)
    t1 = chained["events"].get("T1")
    t0 = chained["events"].get("T0")
    if t1 is not None and int(t1.get("obstacle_id", -1)) >= 0:
        initial_obstacle_id = int(t1["obstacle_id"])
    elif t0 is not None and t0.get("obstacle_ids"):
        initial_obstacle_id = int(t0["obstacle_ids"][0])

    for event in chained["events"].values():
        if event is None:
            continue
        event_time = int(event["steady_time_ns"])
        identity_index = bisect_left(identity_times, event_time) - 1
        if identity_index >= 0:
            identity = identities[identity_index]
            event.setdefault("backend_scan_index", int(identity["backend_scan_index"]))
            event.setdefault("backend_reset_index", int(identity["reset_index"]))
            for key in ("ego_x", "ego_y", "ego_yaw", "speed_mps"):
                if key in identity:
                    event.setdefault(key, float(identity[key]))
        ros_time = int(event.get("application_ros_time_ns", event.get("ros_time_ns", 0)))
        frenet_index = _nearest_index(frenet_times, ros_time)
        if frenet_index is not None:
            frenet_time, frenet = frenet_by_time[frenet_index]
            event.setdefault("ego_s", float(frenet.pose.pose.position.x))
            event.setdefault("ego_d", float(frenet.pose.pose.position.y))
            event["frenet_alignment_error_ms"] = abs(frenet_time - ros_time) * 1.0e-6
        if initial_obstacle_id is not None:
            event.setdefault("obstacle_id", initial_obstacle_id)
        if initial_target is not None:
            event["initial_committed_target_d"] = float(initial_target)

    chained.update(
        {
            "schema": "cma_end_to_end_timing/1",
            "timing_event_message_count": len(timing_records),
            "valid_timing_event_count": len(events),
            "invalid_timing_event_count": invalid_count,
            "state_machine_timer_delay_ms": chained["latencies_ms"]["t3_minus_t2"],
            "avoidance_physics_latency_ms": chained["latencies_ms"]["t9_minus_t1"],
            "avoidance_start_ego_s": (
                chained["events"]["T9"].get("ego_s")
                if chained["events"]["T9"] is not None else None
            ),
            "initial_committed_target_d_m": initial_target,
        }
    )
    return chained


def _scan_identity_diagnostics(
    scans: list[MessageRecord],
    identity_records: list[MessageRecord],
    commitment_timestamp_ns: int | None,
) -> tuple[dict[str, Any], dict[int, dict[str, Any]]]:
    identities = []
    invalid_identity_messages = 0
    for record in identity_records:
        try:
            item = json.loads(record.message.data)
        except (AttributeError, TypeError, json.JSONDecodeError):
            invalid_identity_messages += 1
            continue
        if item.get("schema") != "f1tenth_scan_identity/1":
            invalid_identity_messages += 1
            continue
        identities.append(item)
    identity_by_stamp = {
        int(item["publish_timestamp_ns"]): item for item in identities
    }
    matched: list[dict[str, Any]] = []
    scan_by_header: dict[int, dict[str, Any]] = {}
    hash_mismatches = 0
    for record in scans:
        header = _header_timestamp_ns(record.message)
        ranges_hash = _ranges_sha256(record.message)
        identity = identity_by_stamp.get(header)
        entry = {
            "header_timestamp_ns": header,
            "storage_timestamp_ns": int(record.timestamp_ns),
            "ranges_sha256": ranges_hash,
        }
        if identity is not None:
            entry.update(identity)
            hash_mismatches += int(identity.get("ranges_sha256") != ranges_hash)
        matched.append(entry)
        scan_by_header[header] = entry

    exact_identity = bool(identities)
    if exact_identity:
        matched_identity = [item for item in matched if "backend_scan_index" in item]
        keys = [
            (int(item["reset_index"]), int(item["backend_scan_index"]))
            for item in matched_identity
        ]
        counts_by_key = Counter(keys)
        publication_counts = list(counts_by_key.values())
        initial_publication_counts = [
            count for (_, backend_index), count in counts_by_key.items()
            if backend_index == 0
        ]
        post_initial_publication_counts = [
            count for (_, backend_index), count in counts_by_key.items()
            if backend_index > 0
        ]
        consecutive_duplicate_count = sum(
            first == second
            and int(current["header_timestamp_ns"])
            != int(previous["header_timestamp_ns"])
            for first, second, previous, current in zip(
                keys, keys[1:], matched_identity, matched_identity[1:]
            )
        )
        source = "backend_identity"
    else:
        hashes = [item["ranges_sha256"] for item in matched]
        publication_counts = []
        previous_hash = None
        for value in hashes:
            if not publication_counts or value != previous_hash:
                publication_counts.append(1)
            else:
                publication_counts[-1] += 1
            previous_hash = value
        initial_publication_counts = []
        post_initial_publication_counts = []
        consecutive_duplicate_count = sum(
            first == second
            and int(current["header_timestamp_ns"])
            != int(previous["header_timestamp_ns"])
            for first, second, previous, current in zip(
                hashes, hashes[1:], matched, matched[1:]
            )
        )
        source = "consecutive_ranges_hash_fallback"

    before_commitment = [
        item
        for item in matched
        if commitment_timestamp_ns is not None
        and int(item["header_timestamp_ns"]) <= commitment_timestamp_ns
    ]
    before_keys = {
        (int(item["reset_index"]), int(item["backend_scan_index"]))
        for item in before_commitment
        if "backend_scan_index" in item
    }
    observed_modes = sorted(
        {str(item["publication_mode"]) for item in identities if "publication_mode" in item}
    )
    observed_seeds = sorted(
        {int(item["simulator_seed"]) for item in identities if "simulator_seed" in item}
    )
    observed_sigmas = sorted(
        {float(item["scan_noise_std"]) for item in identities if "scan_noise_std" in item}
    )
    output = {
        "identity_source": source,
        "ros_scan_message_count": len(scans),
        "identity_message_count": len(identities),
        "invalid_identity_message_count": invalid_identity_messages,
        "identity_match_count": sum("backend_scan_index" in item for item in matched),
        "identity_match_fraction": (
            sum("backend_scan_index" in item for item in matched) / len(scans)
            if scans else None
        ),
        "ranges_hash_mismatch_count": hash_mismatches,
        "unique_backend_scan_count": len(publication_counts),
        "publications_per_backend_scan": _statistics(
            [float(item) for item in publication_counts]
        ),
        "initial_scan_publications_per_reset": _statistics(
            [float(item) for item in initial_publication_counts]
        ),
        "post_initial_publications_per_backend_scan": _statistics(
            [float(item) for item in post_initial_publication_counts]
        ),
        "publication_count_histogram": {
            str(count): frequency
            for count, frequency in sorted(Counter(publication_counts).items())
        },
        "maximum_duplicate_publication_count": max(publication_counts, default=0),
        "duplicate_ros_publication_count": sum(
            max(0, item - 1) for item in publication_counts
        ),
        "consecutive_duplicate_with_new_timestamp_count": consecutive_duplicate_count,
        "observed_publication_modes": observed_modes,
        "observed_simulator_seeds": observed_seeds,
        "observed_scan_noise_std_m": observed_sigmas,
        "commitment_timestamp_ns": commitment_timestamp_ns,
        "ros_scan_messages_before_commitment": (
            len(before_commitment) if commitment_timestamp_ns is not None else None
        ),
        "unique_backend_scans_before_commitment": (
            len(before_keys) if exact_identity and commitment_timestamp_ns is not None else None
        ),
    }
    return output, scan_by_header


def _wrapped_error(first: float, second: float) -> float:
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


def _localization_diagnostics(
    ground_truth: list[dict[str, float]], pf_records: list[MessageRecord]
) -> dict[str, Any]:
    ground_truth_timestamps = [
        int(item.get("header_timestamp_ns", item["timestamp_ns"]))
        for item in ground_truth
    ]
    position_errors = []
    yaw_errors = []
    alignment_errors_ms = []
    exact_match_count = 0
    frame_match_count = 0
    child_frame_match_count = 0
    matched_steps: list[tuple[dict[str, float], MessageRecord]] = []
    by_exact_stamp = {
        int(item.get("header_timestamp_ns", item["timestamp_ns"])): item
        for item in ground_truth
    }
    for record in pf_records:
        pf_timestamp = _header_timestamp_ns(record.message)
        index = _nearest_index(ground_truth_timestamps, pf_timestamp)
        if index is None:
            continue
        reference = ground_truth[index]
        pose = record.message.pose.pose
        position_errors.append(
            math.hypot(
                float(pose.position.x) - float(reference["x"]),
                float(pose.position.y) - float(reference["y"]),
            )
        )
        estimated_yaw = quaternion_yaw(pose.orientation)
        yaw_errors.append(_wrapped_error(estimated_yaw, float(reference["yaw"])))
        alignment_errors_ms.append(
            abs(pf_timestamp - ground_truth_timestamps[index]) * 1.0e-6
        )
        exact = by_exact_stamp.get(pf_timestamp)
        if exact is not None:
            exact_match_count += 1
            matched_steps.append((exact, record))
        frame_match_count += int(record.message.header.frame_id == "map")
        child_frame_match_count += int(
            record.message.child_frame_id == "ego_racecar/base_link"
        )

    output_position_steps = []
    output_yaw_steps = []
    introduced_position_step_errors = []
    introduced_yaw_step_errors = []
    for (source_a, output_a), (source_b, output_b) in zip(
        matched_steps, matched_steps[1:]
    ):
        output_pose_a = output_a.message.pose.pose
        output_pose_b = output_b.message.pose.pose
        output_dx = float(output_pose_b.position.x) - float(output_pose_a.position.x)
        output_dy = float(output_pose_b.position.y) - float(output_pose_a.position.y)
        source_dx = float(source_b["x"]) - float(source_a["x"])
        source_dy = float(source_b["y"]) - float(source_a["y"])
        output_yaw_a = quaternion_yaw(output_pose_a.orientation)
        output_yaw_b = quaternion_yaw(output_pose_b.orientation)
        output_dyaw = math.atan2(
            math.sin(output_yaw_b - output_yaw_a),
            math.cos(output_yaw_b - output_yaw_a),
        )
        source_dyaw = math.atan2(
            math.sin(float(source_b["yaw"]) - float(source_a["yaw"])),
            math.cos(float(source_b["yaw"]) - float(source_a["yaw"])),
        )
        output_position_steps.append(math.hypot(output_dx, output_dy))
        output_yaw_steps.append(abs(output_dyaw))
        introduced_position_step_errors.append(
            math.hypot(output_dx - source_dx, output_dy - source_dy)
        )
        introduced_yaw_step_errors.append(_wrapped_error(output_dyaw, source_dyaw))
    exact_fraction = exact_match_count / len(pf_records) if pf_records else None
    introduced_position_max = max(introduced_position_step_errors, default=None)
    introduced_yaw_max = max(introduced_yaw_step_errors, default=None)
    return {
        "position_error_m": _statistics(position_errors),
        "yaw_error_rad": _statistics(yaw_errors),
        "timestamp_alignment_error_ms": _statistics(alignment_errors_ms),
        "source_message_count": len(ground_truth),
        "output_message_count": len(pf_records),
        "exact_source_timestamp_match_count": exact_match_count,
        "exact_source_timestamp_match_fraction": exact_fraction,
        "map_frame_match_fraction": (
            frame_match_count / len(pf_records) if pf_records else None
        ),
        "base_child_frame_match_fraction": (
            child_frame_match_count / len(pf_records) if pf_records else None
        ),
        "output_position_step_m": _statistics(output_position_steps),
        "output_yaw_step_rad": _statistics(output_yaw_steps),
        "bridge_introduced_position_step_error_m": _statistics(
            introduced_position_step_errors
        ),
        "bridge_introduced_yaw_step_error_rad": _statistics(
            introduced_yaw_step_errors
        ),
        "bridge_introduced_discontinuity": bool(
            (introduced_position_max is not None and introduced_position_max > 1.0e-12)
            or (introduced_yaw_max is not None and introduced_yaw_max > 1.0e-12)
        ),
    }


def _tf_consistency_diagnostics(
    tf_records: list[MessageRecord], pf_records: list[MessageRecord]
) -> dict[str, Any]:
    edge_by_stamp: dict[int, list[Any]] = {}
    for record in tf_records:
        for transform in record.message.transforms:
            if (
                transform.header.frame_id == "map"
                and transform.child_frame_id == "ego_racecar/base_link"
            ):
                edge_by_stamp.setdefault(_header_timestamp_ns(transform), []).append(transform)
    position_errors = []
    yaw_errors = []
    exact_matches = 0
    for record in pf_records:
        stamp = _header_timestamp_ns(record.message)
        transforms = edge_by_stamp.get(stamp, [])
        if not transforms:
            continue
        exact_matches += 1
        transform = transforms[0].transform
        pose = record.message.pose.pose
        position_errors.append(
            math.hypot(
                float(pose.position.x) - float(transform.translation.x),
                float(pose.position.y) - float(transform.translation.y),
            )
        )
        yaw_errors.append(
            _wrapped_error(
                quaternion_yaw(pose.orientation),
                quaternion_yaw(transform.rotation),
            )
        )
    duplicate_samples = sum(max(0, len(items) - 1) for items in edge_by_stamp.values())
    return {
        "edge": "map->ego_racecar/base_link",
        "edge_sample_count": sum(len(items) for items in edge_by_stamp.values()),
        "unique_edge_timestamp_count": len(edge_by_stamp),
        "duplicate_edge_sample_count": duplicate_samples,
        "pose_exact_timestamp_match_count": exact_matches,
        "pose_exact_timestamp_match_fraction": (
            exact_matches / len(pf_records) if pf_records else None
        ),
        "position_error_m": _statistics(position_errors),
        "yaw_error_rad": _statistics(yaw_errors),
    }


def _scan_noise_diagnostics(
    model: SimulatorRasterCollisionModel,
    scans: list[MessageRecord],
    ground_truth: list[dict[str, float]],
    maximum_scan_samples: int = 12,
) -> dict[str, Any]:
    if not scans or not ground_truth:
        return {"sampled_scans": 0, "residual_m": _statistics([])}
    by_header = {
        int(item["header_timestamp_ns"]): item
        for item in ground_truth
        if "header_timestamp_ns" in item
    }
    selected_indices = np.linspace(
        0, len(scans) - 1, min(maximum_scan_samples, len(scans)), dtype=int
    )
    residuals: list[float] = []
    geometric_outlier_count = 0
    raw_residual_count = 0
    sampled_scans = 0
    for index in sorted(set(int(item) for item in selected_indices)):
        record = scans[index]
        stamp = record.message.header.stamp
        header_timestamp = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        pose = by_header.get(header_timestamp)
        if pose is None:
            continue
        predicted = model._noise_free_scan(
            float(pose["x"]), float(pose["y"]), float(pose["yaw"])
        )
        observed = np.asarray(record.message.ranges, dtype=np.float64)
        valid = (
            np.isfinite(observed)
            & np.isfinite(predicted)
            & (predicted < model.maximum_range - 0.05)
        )
        raw = observed[valid] - predicted[valid]
        # Beam-edge quantization and a storage-time pose mismatch can move a
        # ray to a different wall. Those are geometry/timing outliers, not the
        # simulator's additive 0.01 m scan noise.
        inlier = np.abs(raw) <= 0.05
        residuals.extend(raw[inlier].tolist())
        geometric_outlier_count += int(np.count_nonzero(~inlier))
        raw_residual_count += int(raw.size)
        sampled_scans += 1
    return {
        "sampled_scans": sampled_scans,
        "residual_m": _statistics(residuals),
        "configured_sigma_m": model.scan_noise_std,
        "geometric_or_pose_timing_outlier_fraction": (
            geometric_outlier_count / raw_residual_count
            if raw_residual_count
            else None
        ),
    }


def _first_nonempty(records: list[MessageRecord], field: str) -> MessageRecord | None:
    return next((record for record in records if getattr(record.message, field)), None)


def _first_obstacle_snapshot(record: MessageRecord | None, manifest: dict[str, Any]) -> dict[str, Any] | None:
    if record is None:
        return None
    truth = manifest["obstacle"]
    obstacle = min(
        record.message.obstacles,
        key=lambda item: math.hypot(
            float(item.x_center) - float(truth["x"]),
            float(item.y_center) - float(truth["y"]),
        ),
    )
    return {
        "id": int(obstacle.id),
        "object_count": len(record.message.obstacles),
        "has_cartesian": bool(obstacle.has_cartesian),
        "is_static": bool(obstacle.is_static),
        "is_visible": bool(obstacle.is_visible),
        "x_center_m": float(obstacle.x_center),
        "y_center_m": float(obstacle.y_center),
        "center_error_to_manifest_m": math.hypot(
            float(obstacle.x_center) - float(truth["x"]),
            float(obstacle.y_center) - float(truth["y"]),
        ),
        "radius_m": float(obstacle.radius),
        "s_center_m": float(obstacle.s_center),
        "d_center_m": float(obstacle.d_center),
        "s_extent_m": float(obstacle.s_end) - float(obstacle.s_start),
        "d_extent_m": float(obstacle.d_left) - float(obstacle.d_right),
    }


def _first_path_snapshot(record: MessageRecord | None) -> dict[str, Any] | None:
    if record is None:
        return None
    lateral = [float(waypoint.d_m) for waypoint in record.message.wpnts]
    return {
        "waypoint_count": len(lateral),
        "first_d_m": lateral[0],
        "last_d_m": lateral[-1],
        "minimum_d_m": min(lateral),
        "maximum_d_m": max(lateral),
        "maximum_abs_d_m": max(abs(value) for value in lateral),
        "mean_d_m": float(np.mean(lateral)),
    }


def _planner_timing(
    bag: Any, manifest: dict[str, Any], ground_truth: list[dict[str, float]]
) -> dict[str, Any]:
    static_records = bag.topic("/static_obs")
    confirmed_records = bag.topic("/confirmed_static_obs")
    scan_by_header = {
        _header_timestamp_ns(record.message): record for record in bag.topic("/scan")
    }
    static_processing_delays = []
    for record in static_records:
        scan = scan_by_header.get(_header_timestamp_ns(record.message))
        if scan is not None:
            static_processing_delays.append(
                (record.timestamp_ns - scan.timestamp_ns) * 1.0e-6
            )

    detection_record = _first_nonempty(static_records, "obstacles")
    confirmed_record = _first_nonempty(confirmed_records, "obstacles")
    first_drive_record = next(
        iter(bag.topic("/drive_autonomous") or bag.topic("/drive")), None
    )
    detection = detection_record.timestamp_ns if detection_record is not None else None
    first_path_record = next(
        (
            record
            for record in bag.topic("/avoid_waypoints")
            if record.message.wpnts
            and (detection is None or record.timestamp_ns >= detection)
        ),
        None,
    )
    first_path = first_path_record.timestamp_ns if first_path_record is not None else None
    detection_header = (
        _header_timestamp_ns(detection_record.message)
        if detection_record is not None
        else None
    )
    path_header = (
        _header_timestamp_ns(first_path_record.message)
        if first_path_record is not None
        else None
    )
    detection_scan = (
        scan_by_header.get(detection_header) if detection_header is not None else None
    )
    truth_by_header = {
        int(item["header_timestamp_ns"]): item
        for item in ground_truth
        if "header_timestamp_ns" in item
    }
    ego = truth_by_header.get(detection_header) if detection_header is not None else None
    obstacle_snapshot = _first_obstacle_snapshot(detection_record, manifest)
    if ego is not None and obstacle_snapshot is not None:
        obstacle_snapshot["ego_center_distance_m"] = math.hypot(
            obstacle_snapshot["x_center_m"] - float(ego["x"]),
            obstacle_snapshot["y_center_m"] - float(ego["y"]),
        )
    nonempty_before_path = sum(
        bool(record.message.obstacles)
        and detection is not None
        and record.timestamp_ns >= detection
        and (first_path is None or record.timestamp_ns <= first_path)
        for record in static_records
    )
    return {
        "first_detection_timestamp_ns": detection,
        "first_path_after_detection_timestamp_ns": first_path,
        "detection_to_path_ms": (first_path - detection) * 1.0e-6
        if detection is not None and first_path is not None
        else None,
        "detection_header_to_path_header_ms": (path_header - detection_header) * 1.0e-6
        if detection_header is not None and path_header is not None
        else None,
        "first_detection_scan_to_static_obs_ms": (
            (detection - detection_scan.timestamp_ns) * 1.0e-6
            if detection is not None and detection_scan is not None
            else None
        ),
        "scan_to_static_obs_processing_ms": _statistics(static_processing_delays),
        "nonempty_static_messages_before_first_path": nonempty_before_path,
        "first_confirmed_static_after_first_static_ms": (
            (confirmed_record.timestamp_ns - detection) * 1.0e-6
            if confirmed_record is not None and detection is not None
            else None
        ),
        "first_scan_to_first_static_ms": (
            (detection_record.timestamp_ns - bag.topic("/scan")[0].timestamp_ns)
            * 1.0e-6
            if detection_record is not None and bag.topic("/scan")
            else None
        ),
        "first_scan_to_first_confirmed_static_ms": (
            (confirmed_record.timestamp_ns - bag.topic("/scan")[0].timestamp_ns)
            * 1.0e-6
            if confirmed_record is not None and bag.topic("/scan")
            else None
        ),
        "first_drive_to_first_static_ms": (
            (detection_record.timestamp_ns - first_drive_record.timestamp_ns)
            * 1.0e-6
            if detection_record is not None and first_drive_record is not None
            else None
        ),
        "first_drive_to_first_confirmed_static_ms": (
            (confirmed_record.timestamp_ns - first_drive_record.timestamp_ns)
            * 1.0e-6
            if confirmed_record is not None and first_drive_record is not None
            else None
        ),
        "confirmed_static_topic_recorded": "/confirmed_static_obs" in bag.topic_types,
        "first_detection": obstacle_snapshot,
        "first_path": _first_path_snapshot(first_path_record),
    }


def _detector_semantics_diagnostics(
    static_records: list[MessageRecord],
    scan_by_header: dict[int, dict[str, Any]],
    commitment_timestamp_ns: int | None,
) -> dict[str, Any]:
    callback_entries = []
    first_nonempty_index = None
    all_ids: set[int] = set()
    before_commitment_ids: set[int] = set()
    for index, record in enumerate(static_records):
        header = _header_timestamp_ns(record.message)
        identity = scan_by_header.get(header)
        callback_entries.append(identity)
        obstacle_ids = {int(item.id) for item in record.message.obstacles}
        all_ids.update(obstacle_ids)
        if (
            commitment_timestamp_ns is not None
            and header <= commitment_timestamp_ns
        ):
            before_commitment_ids.update(obstacle_ids)
        if obstacle_ids and first_nonempty_index is None:
            first_nonempty_index = index

    def backend_key(item: dict[str, Any] | None) -> tuple[int, int] | None:
        if item is None or "backend_scan_index" not in item:
            return None
        return int(item["reset_index"]), int(item["backend_scan_index"])

    existence_keys: list[tuple[int, int]] = []
    envelope_keys: list[tuple[int, int]] = []
    if first_nonempty_index is not None:
        existence_keys = [
            key
            for key in (
                backend_key(item)
                for item in callback_entries[max(0, first_nonempty_index - 2): first_nonempty_index + 1]
            )
            if key is not None
        ]
        envelope_keys = [
            key
            for key in (
                backend_key(item)
                for item in callback_entries[max(0, first_nonempty_index - 1): first_nonempty_index + 1]
            )
            if key is not None
        ]
    exact_window_available = len(existence_keys) == 3
    envelope_window_available = len(envelope_keys) == 2
    return {
        "static_obs_callback_count": len(static_records),
        "first_nonempty_static_callback_index": first_nonempty_index,
        "existence_confirmation_window_callbacks": len(existence_keys),
        "unique_backend_scans_in_existence_window": len(set(existence_keys)),
        "existence_confirmation_duplicate_contribution": (
            len(set(existence_keys)) < 3 if exact_window_available else None
        ),
        "envelope_stability_window_callbacks": len(envelope_keys),
        "unique_backend_scans_in_envelope_window": len(set(envelope_keys)),
        "envelope_stability_duplicate_contribution": (
            len(set(envelope_keys)) < 2 if envelope_window_available else None
        ),
        "observed_obstacle_ids": sorted(all_ids),
        "obstacle_id_split": len(all_ids) > 1,
        "obstacle_ids_before_commitment": sorted(before_commitment_ids),
        "obstacle_id_split_before_commitment": len(before_commitment_ids) > 1,
    }


def planner_log_diagnostics(log_path: str | Path) -> dict[str, Any]:
    path = Path(log_path)
    if not path.is_file():
        return {
            "log_available": False,
            "initial_commit_target_d_m": None,
            "initial_commit_timestamp_ns": None,
            "commit_target_d_m": _statistics([]),
            "replacement_count": 0,
            "hard_commitment_collision_count": 0,
            "hard_collision_obstacle_ids": [],
            "safe_stop_latch_count": 0,
        }
    content = path.read_text(encoding="utf-8", errors="replace")
    initial = re.findall(
        r"Committed\s+(left|right)\s+d-offset spline.*?target d=([-+0-9.eE]+)",
        content,
    )
    timestamped_initial = re.search(
        r"\[(\d+)\.(\d{1,9})\].*?Committed\s+"
        r"(left|right)\s+d-offset spline.*?target d=([-+0-9.eE]+)",
        content,
    )
    replacements = re.findall(
        r"Replaced commitment with\s+(left|right)\s+d-offset spline.*?target d=([-+0-9.eE]+)",
        content,
    )
    targets = [float(value) for _, value in initial + replacements]
    hard_collision_ids = [
        int(value)
        for value in re.findall(
            r"Hard commitment collision;.*?obstacle_id=(\d+)", content
        )
    ]
    return {
        "log_available": True,
        "initial_commit_side": initial[0][0] if initial else None,
        "initial_commit_target_d_m": float(initial[0][1]) if initial else None,
        "initial_commit_timestamp_ns": (
            int(timestamped_initial.group(1)) * 1_000_000_000
            + int(timestamped_initial.group(2).ljust(9, "0"))
            if timestamped_initial else None
        ),
        "commit_target_d_m": _statistics(targets),
        "replacement_count": len(replacements),
        "hard_commitment_collision_count": len(hard_collision_ids),
        "hard_collision_obstacle_ids": sorted(set(hard_collision_ids)),
        "safe_stop_latch_count": content.count("Static safe-stop latched"),
        "safe_stop_release_count": content.count("Safe-stop released"),
    }


def analyze_episode_noise(
    bag_directory: str | Path,
    scenario_manifest_path: str | Path,
    runner_status_path: str | Path,
) -> dict[str, Any]:
    bag = read_bag(
        bag_directory,
        TOPICS,
        optional_topics=(
            "/car_state/frenet/odom",
            "/cma_timing/events",
            "/confirmed_static_obs",
            "/ego_racecar/scan_identity",
            "/tf",
        ),
    )
    manifest = json.loads(Path(scenario_manifest_path).read_text(encoding="utf-8"))
    status = json.loads(Path(runner_status_path).read_text(encoding="utf-8"))
    ground_truth = odom_samples(bag.topic("/ego_racecar/odom"))
    runtime_configuration = status.get("simulator_runtime_configuration", {})
    collision_model = dict(manifest["simulator_collision_model"])
    if "scan_noise_std_m" in runtime_configuration:
        collision_model["scan_noise_std_m"] = float(
            runtime_configuration["scan_noise_std_m"]
        )
    model = SimulatorRasterCollisionModel(
        manifest["baked_map_yaml"], collision_model
    )
    planner_log = planner_log_diagnostics(
        Path(runner_status_path).parent / "logs" / "local_planning.log"
    )
    scan_identity, scan_by_header = _scan_identity_diagnostics(
        bag.topic("/scan"),
        bag.topic("/ego_racecar/scan_identity"),
        planner_log["initial_commit_timestamp_ns"],
    )
    interval_topics = (
        "/ego_racecar/odom",
        "/scan",
        "/pf/pose/odom",
        "/drive",
        "/drive_autonomous",
        "/static_obs",
        "/avoid_waypoints",
    )
    transitions = status.get("transitions", [])
    phase_durations = {}
    for first, second in zip(transitions, transitions[1:]):
        phase_durations[f"{first['state'].lower()}_duration_s"] = float(
            second["monotonic_s"]
        ) - float(first["monotonic_s"])
    return {
        "schema": "cmaes_episode_noise_diagnostics/8",
        "topic_interval_us": {
            topic: _interval_statistics(bag.topic(topic)) for topic in interval_topics
        },
        "scan_noise": _scan_noise_diagnostics(
            model, bag.topic("/scan"), ground_truth
        ),
        "scan_identity": scan_identity,
        "localization": _localization_diagnostics(
            ground_truth, bag.topic("/pf/pose/odom")
        ),
        "tf_consistency": _tf_consistency_diagnostics(
            bag.topic("/tf"), bag.topic("/pf/pose/odom")
        ),
        "planner_timing": _planner_timing(bag, manifest, ground_truth),
        "detector_semantics": _detector_semantics_diagnostics(
            bag.topic("/static_obs"),
            scan_by_header,
            planner_log["initial_commit_timestamp_ns"],
        ),
        "end_to_end_timing": _end_to_end_timing_diagnostics(
            bag.topic("/cma_timing/events"),
            bag.topic("/ego_racecar/scan_identity"),
            bag.topic("/car_state/frenet/odom"),
            planner_log,
        ),
        "planner_log": planner_log,
        "process_phase_durations_s": phase_durations,
        "runtime": status.get("runtime_diagnostics", {}),
        "simulator_runtime_configuration": runtime_configuration,
    }
