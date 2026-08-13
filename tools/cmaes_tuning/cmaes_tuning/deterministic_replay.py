"""Deterministic ROS 2 record/replay audit for obstacle perception and local planning."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import time
from typing import Any, Iterable

from .bag_reader import BagData, read_bag
from .schemas import atomic_write_json, sha256_file
from .simulation_runner import ProcessSupervisor


INPUT_TOPICS = (
    "/scan",
    "/ego_racecar/scan_identity",
    "/ego_racecar/odom",
    "/pf/pose/odom",
    "/car_state/frenet/odom",
    "/tf",
    "/global_waypoints",
    "/state",
)

OUTPUT_TOPICS = (
    "/static_obs",
    "/confirmed_static_obs",
    "/avoid_waypoints",
    "/cma_replay/detector_events",
    "/cma_replay/planner_events",
)

FORBIDDEN_REPLAY_TOPICS = {
    "/static_obs",
    "/confirmed_static_obs",
    "/avoid_waypoints",
    "/drive",
    "/drive_autonomous",
    "/ego_racecar/collision",
}


def _header_stamp_ns(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _canonical_hash(value: Any) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _float_token(value: float) -> str:
    number = float(value)
    if not math.isfinite(number):
        return str(number)
    return number.hex()


def obstacle_geometry(message: Any) -> list[dict[str, Any]]:
    """Return header-free obstacle geometry in stable ID order."""
    fields = (
        "s_center", "s_start", "s_end", "d_center", "d_right", "d_left",
        "x_center", "y_center", "x_min", "x_max", "y_min", "y_max",
        "radius", "size", "x_var", "y_var", "s_var", "d_var",
    )
    result = []
    for obstacle in sorted(message.obstacles, key=lambda item: int(item.id)):
        item: dict[str, Any] = {
            "id": int(obstacle.id),
            "is_static": bool(obstacle.is_static),
            "is_visible": bool(obstacle.is_visible),
            "has_cartesian": bool(obstacle.has_cartesian),
        }
        for field in fields:
            item[field] = _float_token(getattr(obstacle, field))
        result.append(item)
    return result


def waypoint_geometry_hash(message: Any) -> str:
    """Hash only path geometry/limits, excluding wall-clock headers and switch stamps."""
    fields = (
        "s_m", "d_m", "x_m", "y_m", "d_right", "d_left", "psi_rad",
        "kappa_radpm", "vx_mps", "ax_mps2",
    )
    payload = {
        "ot_side": str(message.ot_side),
        "ot_line": str(message.ot_line),
        "waypoints": [
            {
                "id": int(waypoint.id),
                **{field: _float_token(getattr(waypoint, field)) for field in fields},
            }
            for waypoint in message.wpnts
        ],
    }
    return _canonical_hash(payload)


def _parse_json_records(records: Iterable[Any], schema: str) -> list[dict[str, Any]]:
    events = []
    for record in records:
        try:
            event = json.loads(record.message.data)
        except (AttributeError, TypeError, json.JSONDecodeError):
            continue
        if event.get("schema") == schema:
            event["_record_timestamp_ns"] = int(record.timestamp_ns)
            events.append(event)
    return events


def _identity_by_stamp(source: BagData) -> tuple[dict[int, dict[str, Any]], dict[int, dict[str, Any]]]:
    by_stamp: dict[int, dict[str, Any]] = {}
    by_index: dict[int, dict[str, Any]] = {}
    events = _parse_json_records(
        source.topic("/ego_racecar/scan_identity"), "f1tenth_scan_identity/1"
    )
    for event in events:
        stamp = int(event["publish_timestamp_ns"])
        index = int(event["backend_scan_index"])
        by_stamp[stamp] = event
        by_index[index] = event
    return by_stamp, by_index


def _event_scan_index(event: dict[str, Any], identities: dict[int, dict[str, Any]]) -> int | None:
    stamp = event.get("scan_stamp_ns", event.get("source_stamp_ns"))
    identity = identities.get(int(stamp)) if stamp is not None else None
    return int(identity["backend_scan_index"]) if identity is not None else None


def _first_event(events: list[dict[str, Any]], predicate: Any) -> dict[str, Any] | None:
    return next((event for event in events if predicate(event)), None)


def _mean_std(values: list[float | int | None]) -> dict[str, float | int | None]:
    finite = [float(value) for value in values if value is not None and math.isfinite(value)]
    if not finite:
        return {"count": 0, "mean": None, "std": None, "min": None, "max": None}
    return {
        "count": len(finite),
        "mean": statistics.fmean(finite),
        "std": statistics.stdev(finite) if len(finite) > 1 else 0.0,
        "min": min(finite),
        "max": max(finite),
    }


def analyze_replay_bag(source_bag: str | Path, replay_bag: str | Path) -> dict[str, Any]:
    """Extract scan-index decisions from one replay output bag."""
    source = read_bag(
        source_bag,
        topics=("/ego_racecar/scan_identity",),
    )
    output = read_bag(replay_bag, topics=OUTPUT_TOPICS, optional_topics=OUTPUT_TOPICS)
    identities, identities_by_index = _identity_by_stamp(source)
    detector_events = _parse_json_records(
        output.topic("/cma_replay/detector_events"),
        "cma_replay_detector_event/1",
    )
    planner_events = _parse_json_records(
        output.topic("/cma_replay/planner_events"),
        "cma_replay_planner_event/1",
    )

    for event in detector_events:
        event["backend_scan_index"] = _event_scan_index(event, identities)
    for event in planner_events:
        event["backend_scan_index"] = _event_scan_index(event, identities)

    first_raw = _first_event(detector_events, lambda event: bool(event["raw_detections"]))
    first_confirmed = _first_event(
        detector_events,
        lambda event: any(
            track["track_status"] == "CONFIRMED" for track in event["tracks"]
        ),
    )
    first_stable = _first_event(
        detector_events,
        lambda event: any(
            track["track_status"] == "CONFIRMED" and
            int(track["envelope_stable_streak"]) >= 2
            for track in event["tracks"]
        ),
    )
    first_static = _first_event(
        detector_events, lambda event: bool(event["published_static"])
    )
    stabilization_start = _first_event(
        planner_events,
        lambda event: event.get("event") == "INITIAL_STABILIZATION_START",
    )
    stabilization_ready = _first_event(
        planner_events,
        lambda event: event.get("event") == "INITIAL_STABILIZATION_READY",
    )
    commitment = _first_event(
        planner_events,
        lambda event: event.get("event") == "COMMITMENT" and
        not bool(event.get("replacing", False)),
    )

    committed_path = next(
        (
            record.message for record in output.topic("/avoid_waypoints")
            if record.message.ot_line == "raceline_local_d_offset_spline" and
            bool(record.message.wpnts)
        ),
        None,
    )

    raw_by_scan: dict[int, str] = {}
    tracks_by_scan: dict[int, str] = {}
    static_by_scan: dict[int, str] = {}
    track_history: list[dict[str, Any]] = []
    for event in detector_events:
        index = event.get("backend_scan_index")
        if index is None:
            continue
        raw_by_scan[int(index)] = _canonical_hash(event["raw_detections"])
        tracks_by_scan[int(index)] = _canonical_hash(event["tracks"])
        static_by_scan[int(index)] = _canonical_hash(event["published_static"])
        track_history.append(
            {
                "backend_scan_index": int(index),
                "tracks": [
                    {
                        "id": int(track["id"]),
                        "track_uid": int(track["track_uid"]),
                        "hits": int(track["hits"]),
                        "confirmation_history": track["confirmation_history"],
                        "track_status": track["track_status"],
                        "envelope_stable_streak": int(track["envelope_stable_streak"]),
                    }
                    for track in event["tracks"]
                ],
            }
        )

    first_static_geometry = first_static["published_static"] if first_static else []
    observed_detector_indices = {
        int(event["backend_scan_index"])
        for event in detector_events if event.get("backend_scan_index") is not None
    }
    missing_detector_indices = sorted(
        set(identities_by_index) - observed_detector_indices
    )
    return {
        "schema": "cma_deterministic_replay_result/1",
        "source_bag": str(Path(source_bag).resolve()),
        "replay_bag": str(Path(replay_bag).resolve()),
        "source_identity_count": len(identities),
        "detector_event_count": len(detector_events),
        "unmatched_detector_event_count": sum(
            event.get("backend_scan_index") is None for event in detector_events
        ),
        "missing_detector_scan_indices": missing_detector_indices,
        "first_raw_detection_scan_index": (
            first_raw.get("backend_scan_index") if first_raw else None
        ),
        "confirmation_scan_index": (
            first_confirmed.get("backend_scan_index") if first_confirmed else None
        ),
        "envelope_stability_scan_index": (
            first_stable.get("backend_scan_index") if first_stable else None
        ),
        "first_static_obs_scan_index": (
            first_static.get("backend_scan_index") if first_static else None
        ),
        "first_static_obstacle_ids": (
            [int(item["id"]) for item in first_static_geometry]
        ),
        "first_static_geometry": first_static_geometry,
        "first_static_geometry_hash": _canonical_hash(first_static_geometry),
        "raw_detection_sequence_hash": _canonical_hash(raw_by_scan),
        "tracker_sequence_hash": _canonical_hash(tracks_by_scan),
        "static_geometry_sequence_hash": _canonical_hash(static_by_scan),
        "track_hit_history": track_history,
        "planner_stabilization_start_scan_index": (
            stabilization_start.get("backend_scan_index") if stabilization_start else None
        ),
        "planner_stabilization_ready_scan_index": (
            stabilization_ready.get("backend_scan_index") if stabilization_ready else None
        ),
        "planner_commitment_scan_index": (
            commitment.get("backend_scan_index") if commitment else None
        ),
        "committed_obstacle_id": (
            int(commitment["obstacle_id"]) if commitment else None
        ),
        "committed_obstacle_ids": (
            [int(item) for item in commitment.get("obstacle_ids", [])]
            if commitment else []
        ),
        "go_left": bool(commitment["go_left"]) if commitment else None,
        "initial_target_d": float(commitment["target_d"]) if commitment else None,
        "entry_transition_scale": (
            float(commitment["entry_transition_scale"]) if commitment else None
        ),
        "exit_transition_scale": (
            float(commitment["exit_transition_scale"]) if commitment else None
        ),
        "effective_entry_transition_scale": (
            float(commitment["effective_entry_transition_scale"])
            if commitment else None
        ),
        "effective_exit_transition_scale": (
            float(commitment["effective_exit_transition_scale"])
            if commitment else None
        ),
        "avoidance_path_geometry_hash": (
            waypoint_geometry_hash(committed_path) if committed_path is not None else None
        ),
        "avoidance_path_waypoint_count": (
            len(committed_path.wpnts) if committed_path is not None else 0
        ),
        "_fingerprints": {
            "raw_by_scan": raw_by_scan,
            "tracks_by_scan": tracks_by_scan,
            "static_by_scan": static_by_scan,
        },
    }


def first_replay_divergence(
    reference: dict[str, Any], other: dict[str, Any]
) -> dict[str, Any] | None:
    """Return the first scan-index/subsystem divergence between two replay summaries."""
    reference_fingerprints = reference["_fingerprints"]
    other_fingerprints = other["_fingerprints"]
    components = (
        ("raw_by_scan", "detector_raw_detection"),
        ("tracks_by_scan", "obstacle_tracker"),
        ("static_by_scan", "static_obs_publication"),
    )
    indices = sorted(
        set().union(
            *(
                {int(index) for index in fingerprints.keys()}
                for fingerprints in (
                    reference_fingerprints["raw_by_scan"],
                    other_fingerprints["raw_by_scan"],
                )
            )
        )
    )
    for index in indices:
        key = str(index)
        for field, subsystem in components:
            first = reference_fingerprints[field].get(index)
            if first is None:
                first = reference_fingerprints[field].get(key)
            second = other_fingerprints[field].get(index)
            if second is None:
                second = other_fingerprints[field].get(key)
            if first != second:
                return {
                    "backend_scan_index": index,
                    "subsystem": subsystem,
                    "cause": "missing_scan_callback" if first is None or second is None
                    else "different_output_for_same_recorded_input",
                }

    milestones = (
        ("planner_stabilization_start_scan_index", "planner_stabilization_start"),
        ("planner_stabilization_ready_scan_index", "planner_timer_readiness"),
        ("planner_commitment_scan_index", "planner_commitment"),
        ("committed_obstacle_id", "planner_obstacle_selection"),
        ("go_left", "planner_side_selection"),
        ("initial_target_d", "planner_target_selection"),
        ("avoidance_path_geometry_hash", "planner_path_geometry"),
    )
    for field, subsystem in milestones:
        if reference.get(field) != other.get(field):
            return {
                "backend_scan_index": other.get("planner_commitment_scan_index"),
                "subsystem": subsystem,
                "cause": f"{field}_mismatch",
            }
    return None


def summarize_replays(results: list[dict[str, Any]]) -> dict[str, Any]:
    if not results:
        raise ValueError("at least one replay result is required")
    reference = results[0]
    divergences = [first_replay_divergence(reference, result) for result in results[1:]]
    fields = (
        "confirmation_scan_index",
        "envelope_stability_scan_index",
        "first_static_obs_scan_index",
        "planner_stabilization_start_scan_index",
        "planner_stabilization_ready_scan_index",
        "planner_commitment_scan_index",
    )
    target_values = [result["initial_target_d"] for result in results]
    deterministic = (
        all(divergence is None for divergence in divergences) and
        all(result["unmatched_detector_event_count"] == 0 for result in results) and
        all(not result.get("missing_detector_scan_indices", []) for result in results)
    )
    return {
        "replay_count": len(results),
        "perception_planner_deterministic": deterministic,
        "scan_index_statistics": {
            field: _mean_std([result[field] for result in results]) for field in fields
        },
        "target_d_statistics": _mean_std(target_values),
        "unique_first_static_geometry_hashes": sorted(
            {result["first_static_geometry_hash"] for result in results}
        ),
        "unique_static_geometry_sequence_hashes": sorted(
            {result["static_geometry_sequence_hash"] for result in results}
        ),
        "unique_tracker_sequence_hashes": sorted(
            {result["tracker_sequence_hash"] for result in results}
        ),
        "unique_raw_detection_sequence_hashes": sorted(
            {result["raw_detection_sequence_hash"] for result in results}
        ),
        "unique_obstacle_id_sequences": sorted(
            {tuple(result["first_static_obstacle_ids"]) for result in results}
        ),
        "unique_sides": sorted({result["go_left"] for result in results}, key=str),
        "unique_transition_scale_tuples": sorted(
            {
                (
                    result["entry_transition_scale"],
                    result["exit_transition_scale"],
                    result["effective_entry_transition_scale"],
                    result["effective_exit_transition_scale"],
                )
                for result in results
            },
            key=str,
        ),
        "unique_path_geometry_hashes": sorted(
            {result["avoidance_path_geometry_hash"] for result in results}, key=str
        ),
        "divergences_from_replay_00": divergences,
        "first_divergence": next(
            (divergence for divergence in divergences if divergence is not None), None
        ),
    }


def replay_static_geometry_variance(
    source_bag: str | Path, replay_bags: Iterable[str | Path]
) -> dict[str, Any]:
    """Measure numeric `/static_obs` spread at aligned backend scan indices."""
    source = read_bag(source_bag, topics=("/ego_racecar/scan_identity",))
    identities, identities_by_index = _identity_by_stamp(source)
    sequences: list[dict[int, list[dict[str, Any]]]] = []
    missing: list[list[int]] = []
    for replay_bag in replay_bags:
        output = read_bag(
            replay_bag,
            topics=("/cma_replay/detector_events",),
            optional_topics=("/cma_replay/detector_events",),
        )
        sequence: dict[int, list[dict[str, Any]]] = {}
        for event in _parse_json_records(
            output.topic("/cma_replay/detector_events"),
            "cma_replay_detector_event/1",
        ):
            index = _event_scan_index(event, identities)
            if index is not None:
                sequence[int(index)] = event["published_static"]
        sequences.append(sequence)
        missing.append(sorted(set(identities_by_index) - set(sequence)))

    shared_indices = set.intersection(*(set(sequence) for sequence in sequences))
    numeric_fields = (
        "s_center", "s_start", "s_end", "d_center", "d_right", "d_left",
        "x_min", "x_max", "y_min", "y_max",
    )
    maximum_spans = {field: 0.0 for field in numeric_fields}
    comparable_sample_count = 0
    for index in shared_indices:
        geometries = [
            sorted(sequence[index], key=lambda obstacle: int(obstacle["id"]))
            for sequence in sequences
        ]
        structures = [[int(obstacle["id"]) for obstacle in geometry] for geometry in geometries]
        if not structures[0] or not all(item == structures[0] for item in structures[1:]):
            continue
        comparable_sample_count += 1
        for obstacle_index in range(len(geometries[0])):
            for field in numeric_fields:
                values = [float(geometry[obstacle_index][field]) for geometry in geometries]
                maximum_spans[field] = max(
                    maximum_spans[field], max(values) - min(values)
                )

    first_geometries = []
    for sequence in sequences:
        first = next((sequence[index] for index in sorted(sequence) if sequence[index]), [])
        first_geometries.append(first)
    first_field_statistics: dict[str, Any] = {}
    if first_geometries and all(
        len(geometry) == len(first_geometries[0]) and geometry
        for geometry in first_geometries
    ):
        for field in numeric_fields:
            first_field_statistics[field] = _mean_std(
                [float(geometry[0][field]) for geometry in first_geometries]
            )
    return {
        "run_count": len(sequences),
        "source_scan_count": len(identities_by_index),
        "missing_detector_scan_indices_by_run": missing,
        "shared_aligned_scan_count": len(shared_indices),
        "comparable_nonempty_scan_count": comparable_sample_count,
        "maximum_aligned_geometry_span": maximum_spans,
        "first_static_geometry_statistics": first_field_statistics,
    }


def _first_mapping_difference(
    first: dict[int, Any], second: dict[int, Any]
) -> tuple[int | None, str | None]:
    for index in sorted(set(first) | set(second)):
        if index not in first or index not in second:
            return index, "event_presence"
        if first[index] != second[index]:
            return index, "value"
    return None, None


def _first_common_value_difference(
    first: dict[int, Any], second: dict[int, Any]
) -> tuple[int | None, str | None]:
    """Compare aligned backend indices while ignoring recorder-only edge coverage."""
    for index in sorted(set(first) & set(second)):
        if first[index] != second[index]:
            return index, "value"
    return None, None


def _live_stream_data(bag_path: str | Path) -> dict[str, Any]:
    topics = (
        "/ego_racecar/scan_identity",
        "/cma_timing/events",
        "/static_obs",
        "/avoid_waypoints",
    )
    bag = read_bag(bag_path, topics=topics, optional_topics=topics)
    identity_by_stamp, identity_by_index = _identity_by_stamp(bag)
    pose_by_index: dict[int, tuple[str, str, str]] = {}
    scan_hash_by_index: dict[int, str] = {}
    identity_record_timestamp_by_index: dict[int, int] = {}
    for record in bag.topic("/ego_racecar/scan_identity"):
        try:
            event = json.loads(record.message.data)
        except (AttributeError, TypeError, json.JSONDecodeError):
            continue
        if event.get("schema") != "f1tenth_scan_identity/1":
            continue
        index = int(event["backend_scan_index"])
        pose_by_index[index] = (
            _float_token(event["ego_x"]),
            _float_token(event["ego_y"]),
            _float_token(event["ego_yaw"]),
        )
        scan_hash_by_index[index] = str(event["ranges_sha256"])
        identity_record_timestamp_by_index[index] = int(record.timestamp_ns)

    physics_by_step: dict[int, tuple[str, str]] = {}
    for event in _parse_json_records(
        bag.topic("/cma_timing/events"), "cma_timing_event/1"
    ):
        if event.get("event") != "T9_PHYSICS_APPLY":
            continue
        physics_by_step[int(event["backend_step_index"])] = (
            _float_token(event["command_speed_mps"]),
            _float_token(event["steering_angle_rad"]),
        )

    static_by_index: dict[int, str] = {}
    for record in bag.topic("/static_obs"):
        stamp = _header_stamp_ns(record.message)
        identity = identity_by_stamp.get(stamp)
        if identity is None:
            continue
        static_by_index[int(identity["backend_scan_index"])] = _canonical_hash(
            obstacle_geometry(record.message)
        )

    committed_paths = []
    identity_record_items = sorted(identity_record_timestamp_by_index.items())
    for record in bag.topic("/avoid_waypoints"):
        message = record.message
        if message.ot_line != "raceline_local_d_offset_spline" or not message.wpnts:
            continue
        prior = [
            index for index, timestamp in identity_record_items
            if timestamp <= int(record.timestamp_ns)
        ]
        committed_paths.append(
            {
                "backend_scan_index": prior[-1] if prior else None,
                "geometry_hash": waypoint_geometry_hash(message),
            }
        )
    return {
        "identity_by_index": identity_by_index,
        "pose_by_index": pose_by_index,
        "scan_hash_by_index": scan_hash_by_index,
        "physics_by_step": physics_by_step,
        "static_by_index": static_by_index,
        "committed_paths": committed_paths,
    }


def compare_live_closed_loop(
    first_bag: str | Path,
    second_bag: str | Path,
    first_noise_diagnostics: str | Path | None = None,
    second_noise_diagnostics: str | Path | None = None,
) -> dict[str, Any]:
    """Locate the causal ordering of two live runs by backend step/scan index."""
    first = _live_stream_data(first_bag)
    second = _live_stream_data(second_bag)
    command_index, command_kind = _first_mapping_difference(
        first["physics_by_step"], second["physics_by_step"]
    )
    pose_index, pose_kind = _first_common_value_difference(
        first["pose_by_index"], second["pose_by_index"]
    )
    scan_index, scan_kind = _first_common_value_difference(
        first["scan_hash_by_index"], second["scan_hash_by_index"]
    )
    static_index, static_kind = _first_common_value_difference(
        first["static_by_index"], second["static_by_index"]
    )

    def noise_summary(path: str | Path | None) -> dict[str, Any] | None:
        if path is None:
            return None
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        planner = payload.get("planner", payload.get("planner_log", {}))
        timing = payload.get("end_to_end_timing", {})
        return {
            "initial_target_d": timing.get(
                "initial_committed_target_d_m",
                planner.get("initial_commit_target_d_m"),
            ),
            "initial_commit_side": planner.get("initial_commit_side"),
            "initial_commit_timestamp_ns": planner.get("initial_commit_timestamp_ns"),
            "first_nonempty_path_backend_scan_index": (
                timing.get("events", {}).get("T1") or {}
            ).get("backend_scan_index"),
        }

    first_noise = noise_summary(first_noise_diagnostics)
    second_noise = noise_summary(second_noise_diagnostics)

    def commit_scan_index(
        noise: dict[str, Any] | None, stream: dict[str, Any]
    ) -> int | None:
        timestamp = (noise or {}).get("initial_commit_timestamp_ns")
        if timestamp is None:
            return None
        prior = [
            index for index, identity in stream["identity_by_index"].items()
            if int(identity["publish_timestamp_ns"]) <= int(timestamp)
        ]
        return max(prior) if prior else None

    first_commit_index = commit_scan_index(first_noise, first)
    second_commit_index = commit_scan_index(second_noise, second)
    if first_noise is not None:
        first_noise["commitment_backend_scan_index"] = first_commit_index
    if second_noise is not None:
        second_noise["commitment_backend_scan_index"] = second_commit_index
    stages = [
        {
            "event": "command_or_physics_phase_divergence",
            "backend_index": command_index,
            "difference_kind": command_kind,
        },
        {
            "event": "ego_pose_divergence",
            "backend_index": pose_index,
            "difference_kind": pose_kind,
        },
        {
            "event": "scan_geometry_divergence",
            "backend_index": scan_index,
            "difference_kind": scan_kind,
        },
        {
            "event": "static_obs_divergence",
            "backend_index": static_index,
            "difference_kind": static_kind,
        },
        {
            "event": "initial_commitment",
            "first_backend_index": (
                first_noise or {}).get("commitment_backend_scan_index"),
            "second_backend_index": (
                second_noise or {}
            ).get("commitment_backend_scan_index"),
            "first_target_d": (first_noise or {}).get("initial_target_d"),
            "second_target_d": (second_noise or {}).get("initial_target_d"),
        },
    ]
    return {
        "schema": "cma_live_closed_loop_divergence/1",
        "first_bag": str(Path(first_bag).resolve()),
        "second_bag": str(Path(second_bag).resolve()),
        "causal_stages": stages,
        "first_command_or_physics_phase_divergence_backend_step": command_index,
        "command_or_physics_phase_divergence_values": {
            "first": first["physics_by_step"].get(command_index),
            "second": second["physics_by_step"].get(command_index),
        } if command_index is not None else None,
        "first_ego_pose_divergence_backend_scan": pose_index,
        "ego_pose_divergence_values": {
            "first": first["pose_by_index"].get(pose_index),
            "second": second["pose_by_index"].get(pose_index),
        } if pose_index is not None else None,
        "first_scan_geometry_divergence_backend_scan": scan_index,
        "scan_geometry_divergence_hashes": {
            "first": first["scan_hash_by_index"].get(scan_index),
            "second": second["scan_hash_by_index"].get(scan_index),
        } if scan_index is not None else None,
        "first_static_obs_divergence_backend_scan": static_index,
        "first_run_commitment": first_noise,
        "second_run_commitment": second_noise,
    }


@dataclass(frozen=True)
class RecordedStream:
    scenario_id: str
    source_repeat: int
    source_attempt: int
    bag: Path
    scenario_manifest: Path
    episode_result: Path
    noise_diagnostics: Path


class DeterministicReplayAudit:
    """Run clean-process detector/planner replays and persist a reproducible audit."""

    def __init__(
        self,
        config: dict[str, Any],
        workspace_root: str | Path,
        output_directory: str | Path,
        source_experiment: str | Path,
        scenario_repeats: dict[str, int],
        repetitions: int = 10,
        domain_id_start: int = 181,
    ):
        if repetitions < 10:
            raise ValueError("deterministic replay audit requires at least 10 repetitions")
        if set(INPUT_TOPICS) & FORBIDDEN_REPLAY_TOPICS:
            raise AssertionError("replay inputs include a forbidden downstream/GT topic")
        self.config = config
        self.workspace_root = Path(workspace_root).resolve()
        self.output_directory = Path(output_directory).resolve()
        self.source_experiment = Path(source_experiment).resolve()
        self.scenario_repeats = dict(scenario_repeats)
        self.repetitions = repetitions
        self.domain_id_start = domain_id_start

    def _stream(self, scenario_id: str, repeat: int) -> RecordedStream:
        base = (
            self.source_experiment / "repeatability_episodes" / "baseline" /
            scenario_id / f"repeat_{repeat:02d}"
        )
        attempts = sorted(
            path for path in base.glob("attempt_*")
            if (path / "bag" / "metadata.yaml").is_file() and
            (path / "episode_result.json").is_file()
        )
        if not attempts:
            raise FileNotFoundError(f"no valid source attempt under {base}")
        attempt = attempts[-1]
        attempt_index = int(attempt.name.split("_")[-1])
        manifest = (
            self.source_experiment / "scenarios" / "training" /
            scenario_id / "manifest.json"
        )
        return RecordedStream(
            scenario_id=scenario_id,
            source_repeat=repeat,
            source_attempt=attempt_index,
            bag=attempt / "bag",
            scenario_manifest=manifest,
            episode_result=attempt / "episode_result.json",
            noise_diagnostics=attempt / "noise_diagnostics.json",
        )

    def _shell_prefix(self) -> str:
        ros = self.config["ros"]
        setup_paths = (
            Path(ros["ros_setup"]),
            (self.workspace_root / ros["workspace_setup"]).resolve(),
            Path(ros["simulator_setup"]),
        )
        missing = [str(path) for path in setup_paths if not path.is_file()]
        if missing:
            raise FileNotFoundError(f"missing ROS setup files: {missing}")
        return " && ".join(f"source {path}" for path in setup_paths)

    def _environment(self, run_directory: Path, domain_id: int) -> dict[str, str]:
        environment = dict(os.environ)
        environment.update(
            {
                "ROS_DOMAIN_ID": str(domain_id),
                "ROS_LOG_DIR": str((run_directory / "ros_logs").resolve()),
                "CMA_WORKSPACE_ROOT": str(self.workspace_root),
                "F1_MAP": str(self.config["experiment"]["map_name"]),
                "FASTDDS_BUILTIN_TRANSPORTS": str(
                    self.config["ros"].get("fastdds_builtin_transports", "UDPv4")
                ),
            }
        )
        (run_directory / "ros_logs").mkdir(parents=True, exist_ok=True)
        return environment

    def _recorded_stream_manifest(self, stream: RecordedStream) -> dict[str, Any]:
        database = next(stream.bag.glob("*.db3"))
        source_episode = json.loads(stream.episode_result.read_text(encoding="utf-8"))
        scenario = json.loads(stream.scenario_manifest.read_text(encoding="utf-8"))
        source_experiment_manifest = json.loads(
            (self.source_experiment / "experiment_manifest.json").read_text(encoding="utf-8")
        )
        return {
            "schema": "cma_recorded_input_stream/1",
            "scenario_id": stream.scenario_id,
            "source_repeat": stream.source_repeat,
            "source_attempt": stream.source_attempt,
            "source_bag": str(stream.bag),
            "source_bag_metadata_sha256": sha256_file(stream.bag / "metadata.yaml"),
            "source_bag_database_sha256": sha256_file(database),
            "source_episode_result_sha256": sha256_file(stream.episode_result),
            "source_scenario_manifest": str(stream.scenario_manifest),
            "source_scenario_manifest_sha256": sha256_file(stream.scenario_manifest),
            "input_topics": list(INPUT_TOPICS),
            "forbidden_topics_not_replayed": sorted(FORBIDDEN_REPLAY_TOPICS),
            "obstacle_ground_truth_replayed": False,
            "localization_mode": source_experiment_manifest.get("localization_mode"),
            "simulator_runtime_configuration": source_experiment_manifest.get(
                "simulator_runtime_configuration"
            ),
            "controller_configuration": source_experiment_manifest.get(
                "controller_configuration"
            ),
            "source_classification": source_episode.get("classification"),
            "source_failure": source_episode.get("failure"),
            "clean_map_yaml": scenario["clean_map_yaml"],
            "clean_map_yaml_sha256": scenario["clean_map_yaml_sha256"],
        }

    def _run_once(
        self, stream: RecordedStream, replay_index: int, domain_id: int
    ) -> dict[str, Any]:
        run_directory = (
            self.output_directory / "replays" / stream.scenario_id /
            f"replay_{replay_index:02d}"
        )
        run_directory.mkdir(parents=True, exist_ok=True)
        output_bag = run_directory / "bag"
        cached = run_directory / "replay_result.json"
        if cached.is_file() and (output_bag / "metadata.yaml").is_file():
            return json.loads(cached.read_text(encoding="utf-8"))

        scenario = json.loads(stream.scenario_manifest.read_text(encoding="utf-8"))
        baseline = self.source_experiment / "candidates" / "baseline.yaml"
        environment = self._environment(run_directory, domain_id)
        supervisor = ProcessSupervisor(
            run_directory, environment, self._shell_prefix()
        )
        commands = {
            "pipeline": [
                "ros2", "launch", "local_planning", "local_planning.launch.py",
                f"params_file:={baseline}",
                f"reference_map:={scenario['clean_map_yaml']}",
                "simulator:=true", "use_sim_time:=false",
                "replay_diagnostics_enable:=true",
            ],
            "recorder": [
                "ros2", "bag", "record", "--storage", "sqlite3",
                "--disable-keyboard-controls", "--output", str(output_bag),
                "--topics", *OUTPUT_TOPICS,
            ],
            "player": [
                "ros2", "bag", "play", str(stream.bag),
                "--disable-keyboard-controls", "--wait-for-all-acked", "1000",
                "--topics", *INPUT_TOPICS,
            ],
        }
        status: dict[str, Any] = {
            "schema": "cma_deterministic_replay_status/1",
            "scenario_id": stream.scenario_id,
            "replay_index": replay_index,
            "domain_id": domain_id,
            "commands": commands,
            "valid": False,
            "failures": [],
        }
        atomic_write_json(run_directory / "status.json", status)
        try:
            supervisor.start("pipeline", commands["pipeline"])
            time.sleep(4.0)
            if supervisor.early_exits():
                status["failures"].extend(supervisor.early_exits())
                return status
            supervisor.start("recorder", commands["recorder"])
            time.sleep(2.0)
            player = supervisor.start("player", commands["player"])
            deadline = time.monotonic() + 45.0
            while player.process.poll() is None and time.monotonic() < deadline:
                early = supervisor.early_exits(ignore={"player"})
                if early:
                    status["failures"].extend(early)
                    break
                time.sleep(0.1)
            if player.process.poll() is None:
                status["failures"].append("player_timeout")
            elif player.process.returncode != 0:
                status["failures"].append(
                    f"player_exit:{player.process.returncode}"
                )
            time.sleep(1.0)
            supervisor.stop_one("recorder", 8.0, 2.0)
        finally:
            cleanup = supervisor.stop_all(3.0, 2.0)
            status["failures"].extend(cleanup)
            atomic_write_json(run_directory / "status.json", status)

        if status["failures"]:
            return status
        result = analyze_replay_bag(stream.bag, output_bag)
        result.update(
            {
                "scenario_id": stream.scenario_id,
                "replay_index": replay_index,
                "domain_id": domain_id,
            }
        )
        status["valid"] = True
        atomic_write_json(run_directory / "status.json", status)
        atomic_write_json(cached, result)
        return result

    def run(self) -> dict[str, Any]:
        self.output_directory.mkdir(parents=True, exist_ok=True)
        streams = {
            scenario_id: self._stream(scenario_id, repeat)
            for scenario_id, repeat in self.scenario_repeats.items()
        }
        manifests_directory = self.output_directory / "recorded_streams"
        manifests_directory.mkdir(parents=True, exist_ok=True)
        for stream in streams.values():
            atomic_write_json(
                manifests_directory / f"{stream.scenario_id}.json",
                self._recorded_stream_manifest(stream),
            )

        experiment_manifest = {
            "schema": "cma_deterministic_replay_experiment/1",
            "source_experiment": str(self.source_experiment),
            "source_experiment_manifest_sha256": sha256_file(
                self.source_experiment / "experiment_manifest.json"
            ),
            "repetitions_per_stream": self.repetitions,
            "scenario_source_repeats": self.scenario_repeats,
            "input_topics": list(INPUT_TOPICS),
            "output_topics": list(OUTPUT_TOPICS),
            "simulator_physics_started": False,
            "controller_started": False,
            "obstacle_ground_truth_replayed": False,
            "production_algorithm_parameters_changed": False,
        }
        atomic_write_json(
            self.output_directory / "experiment_manifest.json", experiment_manifest
        )

        scenario_results: dict[str, list[dict[str, Any]]] = {}
        offset = 0
        for scenario_id, stream in streams.items():
            results = []
            for replay_index in range(self.repetitions):
                result = self._run_once(
                    stream, replay_index, self.domain_id_start + offset
                )
                offset += 1
                if not result.get("valid", True):
                    raise RuntimeError(
                        f"invalid replay {scenario_id}/{replay_index}: {result}"
                    )
                results.append(result)
            scenario_results[scenario_id] = results

        summary = {
            "schema": "cma_deterministic_replay_audit/1",
            "scenarios": {
                scenario_id: summarize_replays(results)
                for scenario_id, results in scenario_results.items()
            },
        }
        summary["perception_planner_deterministic"] = all(
            scenario["perception_planner_deterministic"]
            for scenario in summary["scenarios"].values()
        )
        atomic_write_json(self.output_directory / "replay_audit.json", summary)
        return summary
