#!/usr/bin/env python3
"""Extract threshold-free localization audit features from one rosbag, read-only."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from pathlib import Path
from typing import Any, Callable, Optional

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


POSE_TOPICS = ("/pf/pose/odom",)
FRENET_TOPICS = ("/car_state/frenet/odom",)
ODOM_TOPICS = ("/odom", "/car_state/odom")
IMU_TOPICS = ("/sensors/imu/raw", "/sensors/imu", "/imu/data")
SCAN_TOPICS = ("/scan", "/scan_slow", "/slow_scan")
PARTICLE_TOPICS = ("/pf/viz/particles",)
GT_TOPICS = ("/ego_racecar/odom",)
GLOBAL_TOPICS = ("/global_waypoints",)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--pose-topic", default="", help="Exact pose topic override")
    parser.add_argument("--force", action="store_true", help="Allow replacing an existing output file")
    return parser.parse_args()


def storage_id(bag: Path) -> str:
    document = yaml.safe_load((bag / "metadata.yaml").read_text())
    return str(document["rosbag2_bagfile_information"]["storage_identifier"])


def first_available(
    candidates: tuple[str, ...], types: dict[str, str], allowed_types: Optional[set[str]] = None
) -> Optional[str]:
    return next(
        (
            name
            for name in candidates
            if name in types and (allowed_types is None or types[name] in allowed_types)
        ),
        None,
    )


def header_ns(message: Any) -> Optional[int]:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw_of(quaternion: Any) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def nearest(
    records: list[tuple], times: list[int], timestamp_ns: int
) -> tuple[Optional[tuple], Optional[float]]:
    if not records:
        return None, None
    position = bisect.bisect_left(times, timestamp_ns)
    choices = []
    if position < len(records):
        choices.append(records[position])
    if position > 0:
        choices.append(records[position - 1])
    result = min(choices, key=lambda record: abs(record[0] - timestamp_ns))
    return result, (result[0] - timestamp_ns) / 1e9


def particle_summary(message: Any) -> tuple:
    poses = list(getattr(message, "poses", []))
    if not poses:
        return 0, None, None, None, None, None
    xs = [float(pose.position.x) for pose in poses]
    ys = [float(pose.position.y) for pose in poses]
    yaws = [yaw_of(pose.orientation) for pose in poses]
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    x_std = math.sqrt(sum((value - mean_x) ** 2 for value in xs) / len(xs))
    y_std = math.sqrt(sum((value - mean_y) ** 2 for value in ys) / len(ys))
    resultant = math.hypot(
        sum(math.cos(value) for value in yaws) / len(yaws),
        sum(math.sin(value) for value in yaws) / len(yaws),
    )
    yaw_std = math.sqrt(max(0.0, -2.0 * math.log(max(resultant, 1e-15))))
    split_values = xs if x_std >= y_std else ys
    median = sorted(split_values)[len(split_values) // 2]
    first = [pose for pose, value in zip(poses, split_values) if value <= median]
    second = [pose for pose, value in zip(poses, split_values) if value > median]
    multimodality_proxy = None
    if first and second:
        first_center = (
            sum(pose.position.x for pose in first) / len(first),
            sum(pose.position.y for pose in first) / len(first),
        )
        second_center = (
            sum(pose.position.x for pose in second) / len(second),
            sum(pose.position.y for pose in second) / len(second),
        )
        multimodality_proxy = math.hypot(
            first_center[0] - second_center[0], first_center[1] - second_center[1]
        )
    weights = getattr(message, "weights", None)
    ess = None
    if weights is not None:
        finite = [float(value) for value in weights if math.isfinite(float(value)) and float(value) >= 0.0]
        total = sum(finite)
        if total > 0.0:
            normalized = [value / total for value in finite]
            ess = 1.0 / sum(value * value for value in normalized)
    return len(poses), x_std, y_std, yaw_std, multimodality_proxy, ess


def integrated_absolute_speed(
    records: list[tuple[int, float]], times: list[int], start_ns: int, end_ns: int
) -> Optional[float]:
    if not records or end_ns <= start_ns:
        return None
    start_record, _ = nearest(records, times, start_ns)
    end_record, _ = nearest(records, times, end_ns)
    if start_record is None or end_record is None:
        return None
    begin = bisect.bisect_right(times, start_ns)
    finish = bisect.bisect_left(times, end_ns)
    points = [(start_ns, abs(start_record[1]))]
    points.extend((record[0], abs(record[1])) for record in records[begin:finish])
    points.append((end_ns, abs(end_record[1])))
    return sum(
        0.5 * (first[1] + second[1]) * (second[0] - first[0]) / 1e9
        for first, second in zip(points, points[1:])
    )


def nearest_reference(reference: list[tuple[float, float, float]], s_value: float) -> Optional[tuple[float, float, float]]:
    if not reference:
        return None
    track_length = max(item[0] for item in reference)
    if track_length <= 0.0:
        return min(reference, key=lambda item: abs(item[0] - s_value))
    return min(
        reference,
        key=lambda item: min(abs(item[0] - s_value), track_length - min(abs(item[0] - s_value), track_length)),
    )


def serialize(value: Any) -> Any:
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return ""
    if isinstance(value, bool):
        return int(value)
    return value


def main() -> int:
    args = parse_args()
    bag = args.bag.resolve()
    output = args.output.resolve()
    if not (bag / "metadata.yaml").is_file():
        raise SystemExit(f"metadata.yaml not found: {bag}")
    if output == bag or bag in output.parents:
        raise SystemExit("output must not be inside the read-only bag directory")
    if output.exists() and not args.force:
        raise SystemExit(f"output exists; pass --force to replace: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage_id(bag)),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    pose_topic = args.pose_topic or first_available(POSE_TOPICS, topic_types)
    if not pose_topic or pose_topic not in topic_types:
        raise SystemExit("no exact localization pose topic found; use --pose-topic only with a verified topic")

    chosen = {
        "pose": pose_topic,
        "frenet": first_available(FRENET_TOPICS, topic_types),
        "odom": first_available(ODOM_TOPICS, topic_types),
        "imu": first_available(IMU_TOPICS, topic_types, {"sensor_msgs/msg/Imu"}),
        "scan": first_available(SCAN_TOPICS, topic_types),
        "particle": first_available(PARTICLE_TOPICS, topic_types),
        "gt": first_available(GT_TOPICS, topic_types),
        "global": first_available(GLOBAL_TOPICS, topic_types),
        "tf": "/tf" if "/tf" in topic_types else None,
        "tf_static": "/tf_static" if "/tf_static" in topic_types else None,
    }
    selected_topics = {value for value in chosen.values() if value is not None}
    message_types = {name: get_message(topic_types[name]) for name in selected_topics}

    poses: list[dict[str, Any]] = []
    frenet: list[tuple[int, float, float]] = []
    odom: list[tuple[int, float]] = []
    imu: list[tuple[int, float]] = []
    scan: list[tuple[int]] = []
    particles: list[tuple] = []
    gt: list[tuple[int, float, float, float]] = []
    tf: list[tuple[int]] = []
    tf_static_present = False
    reference: list[tuple[float, float, float]] = []

    while reader.has_next():
        topic, payload, recorded_ns = reader.read_next()
        if topic not in selected_topics:
            continue
        message = deserialize_message(payload, message_types[topic])
        if topic == chosen["pose"]:
            pose = message.pose.pose
            covariance = list(message.pose.covariance)
            poses.append(
                {
                    "t": int(recorded_ns),
                    "header_t": header_ns(message),
                    "x": float(pose.position.x),
                    "y": float(pose.position.y),
                    "yaw": yaw_of(pose.orientation),
                    "cov_xx": float(covariance[0]) if len(covariance) > 35 else None,
                    "cov_yy": float(covariance[7]) if len(covariance) > 35 else None,
                    "cov_yawyaw": float(covariance[35]) if len(covariance) > 35 else None,
                }
            )
        elif topic == chosen["frenet"]:
            frenet.append((int(recorded_ns), float(message.pose.pose.position.x), float(message.pose.pose.position.y)))
        elif topic == chosen["odom"]:
            odom.append((int(recorded_ns), float(message.twist.twist.linear.x)))
        elif topic == chosen["imu"]:
            imu.append((int(recorded_ns), float(message.angular_velocity.z)))
        elif topic == chosen["scan"]:
            scan.append((int(recorded_ns),))
        elif topic == chosen["particle"]:
            particles.append((int(recorded_ns), *particle_summary(message)))
        elif topic == chosen["gt"]:
            pose = message.pose.pose
            gt.append((int(recorded_ns), float(pose.position.x), float(pose.position.y), yaw_of(pose.orientation)))
        elif topic == chosen["tf"]:
            tf.append((int(recorded_ns),))
        elif topic == chosen["tf_static"]:
            tf_static_present = True
        elif topic == chosen["global"] and not reference:
            reference = [
                (float(point.s_m), float(point.d_left), float(point.d_right))
                for point in getattr(message, "wpnts", [])
            ]

    for records in (frenet, odom, imu, scan, particles, gt, tf):
        records.sort(key=lambda item: item[0])
    record_times = {
        "frenet": [item[0] for item in frenet],
        "odom": [item[0] for item in odom],
        "imu": [item[0] for item in imu],
        "scan": [item[0] for item in scan],
        "particles": [item[0] for item in particles],
        "gt": [item[0] for item in gt],
        "tf": [item[0] for item in tf],
    }
    poses.sort(key=lambda item: item["t"])

    fields = [
        "schema_version", "bag_path", "pose_topic", "timestamp_ns", "pose_header_timestamp_ns",
        "localization_label", "localization_age_s", "pose_sample_gap_s", "timestamp_regression",
        "x_m", "y_m", "yaw_rad", "unwrapped_yaw_rad", "dx_m", "dy_m",
        "distance_jump_m", "unwrapped_dyaw_rad",
        "implied_speed_mps", "implied_yaw_rate_radps", "scan_pose_offset_s", "odom_pose_offset_s",
        "frenet_pose_offset_s", "imu_pose_offset_s", "tf_nearest_offset_s", "tf_static_present",
        "frenet_s_m", "frenet_d_m", "ds_m", "dd_m", "backward_s_jump_m", "abs_d_jump_m",
        "projection_branch_jump_proxy_m", "track_bound_inconsistency", "reference_available",
        "odom_speed_mps", "localization_minus_odom_speed_mps", "imu_yaw_rate_radps",
        "localization_minus_imu_yaw_rate_radps", "short_window_s",
        "localization_short_window_displacement_m", "odom_short_window_distance_m",
        "short_window_displacement_residual_m", "covariance_xx", "covariance_yy",
        "covariance_yawyaw", "particle_count", "particle_x_std_m", "particle_y_std_m",
        "particle_yaw_circular_std_rad", "particle_multimodality_proxy_m", "particle_ess",
        "gt_x_m", "gt_y_m", "gt_yaw_rad",
        "gt_position_error_m", "gt_yaw_error_rad", "gt_frenet_s_error_m", "gt_frenet_d_error_m",
        "availability_note",
    ]
    rows = []
    previous_pose = None
    previous_pose_header_ns = None
    previous_frenet = None
    unwrapped_yaw = None
    window_start_index = 0
    for pose_index, pose in enumerate(poses):
        dt = None if previous_pose is None else (pose["t"] - previous_pose["t"]) / 1e9
        dx = None if previous_pose is None else pose["x"] - previous_pose["x"]
        dy = None if previous_pose is None else pose["y"] - previous_pose["y"]
        distance = None if dx is None else math.hypot(dx, dy)
        dyaw = None if previous_pose is None else wrap_angle(pose["yaw"] - previous_pose["yaw"])
        if unwrapped_yaw is None:
            unwrapped_yaw = pose["yaw"]
        elif dyaw is not None:
            unwrapped_yaw += dyaw
        speed = distance / dt if distance is not None and dt is not None and dt > 0.0 else None
        yaw_rate = dyaw / dt if dyaw is not None and dt is not None and dt > 0.0 else None

        nearest_scan, scan_offset = nearest(scan, record_times["scan"], pose["t"])
        nearest_odom, odom_offset = nearest(odom, record_times["odom"], pose["t"])
        nearest_frenet, frenet_offset = nearest(frenet, record_times["frenet"], pose["t"])
        nearest_imu, imu_offset = nearest(imu, record_times["imu"], pose["t"])
        nearest_tf, tf_offset = nearest(tf, record_times["tf"], pose["t"])
        nearest_particles, _ = nearest(particles, record_times["particles"], pose["t"])
        nearest_gt, _ = nearest(gt, record_times["gt"], pose["t"])

        ds = dd = backward = abs_d_jump = branch_proxy = track_inconsistent = None
        ref_available = bool(reference)
        if nearest_frenet is not None:
            if previous_frenet is not None:
                ds = nearest_frenet[1] - previous_frenet[1]
                dd = nearest_frenet[2] - previous_frenet[2]
                backward = max(0.0, -ds)
                abs_d_jump = abs(dd)
                branch_proxy = abs(abs(ds) - distance) if distance is not None else None
            nearest_ref = nearest_reference(reference, nearest_frenet[1])
            if nearest_ref is not None:
                track_inconsistent = nearest_frenet[2] > nearest_ref[1] or nearest_frenet[2] < -nearest_ref[2]
            previous_frenet = nearest_frenet

        gt_position_error = gt_yaw_error = None
        if nearest_gt is not None:
            gt_position_error = math.hypot(pose["x"] - nearest_gt[1], pose["y"] - nearest_gt[2])
            gt_yaw_error = wrap_angle(pose["yaw"] - nearest_gt[3])
        while (
            window_start_index + 1 < pose_index
            and pose["t"] - poses[window_start_index + 1]["t"] >= 500_000_000
        ):
            window_start_index += 1
        window_start = poses[window_start_index] if pose_index > 0 else None
        window_s = None
        localization_window_displacement = None
        odom_window_distance = None
        window_residual = None
        if window_start is not None:
            window_s = (pose["t"] - window_start["t"]) / 1e9
            localization_window_displacement = math.hypot(
                pose["x"] - window_start["x"], pose["y"] - window_start["y"]
            )
            odom_window_distance = integrated_absolute_speed(
                odom, record_times["odom"], window_start["t"], pose["t"]
            )
            if odom_window_distance is not None:
                window_residual = localization_window_displacement - odom_window_distance
        notes = []
        if not reference:
            notes.append("no global reference: track-bound/projection context unavailable")
        if nearest_particles is None:
            notes.append("no particle cloud")
        elif nearest_particles[6] is None:
            notes.append("particle message has no weights: ESS unavailable")
        if nearest_gt is None:
            notes.append("no simulator GT")
        else:
            notes.append("GT Frenet errors unavailable without explicit GT-to-reference projection")
        row = {
            "schema_version": "localization_features/1",
            "bag_path": str(bag),
            "pose_topic": pose_topic,
            "timestamp_ns": pose["t"],
            "pose_header_timestamp_ns": pose["header_t"],
            "localization_label": "LOC_UNKNOWN",
            "localization_age_s": None if pose["header_t"] is None else (pose["t"] - pose["header_t"]) / 1e9,
            "pose_sample_gap_s": dt,
            "timestamp_regression": pose["header_t"] is not None and previous_pose_header_ns is not None and pose["header_t"] < previous_pose_header_ns,
            "x_m": pose["x"], "y_m": pose["y"], "yaw_rad": pose["yaw"],
            "unwrapped_yaw_rad": unwrapped_yaw,
            "dx_m": dx, "dy_m": dy, "distance_jump_m": distance,
            "unwrapped_dyaw_rad": dyaw, "implied_speed_mps": speed,
            "implied_yaw_rate_radps": yaw_rate, "scan_pose_offset_s": scan_offset,
            "odom_pose_offset_s": odom_offset, "frenet_pose_offset_s": frenet_offset,
            "imu_pose_offset_s": imu_offset, "tf_nearest_offset_s": tf_offset,
            "tf_static_present": tf_static_present,
            "frenet_s_m": nearest_frenet[1] if nearest_frenet else None,
            "frenet_d_m": nearest_frenet[2] if nearest_frenet else None,
            "ds_m": ds, "dd_m": dd, "backward_s_jump_m": backward,
            "abs_d_jump_m": abs_d_jump, "projection_branch_jump_proxy_m": branch_proxy,
            "track_bound_inconsistency": track_inconsistent, "reference_available": ref_available,
            "odom_speed_mps": nearest_odom[1] if nearest_odom else None,
            "localization_minus_odom_speed_mps": speed - nearest_odom[1] if speed is not None and nearest_odom else None,
            "imu_yaw_rate_radps": nearest_imu[1] if nearest_imu else None,
            "localization_minus_imu_yaw_rate_radps": yaw_rate - nearest_imu[1] if yaw_rate is not None and nearest_imu else None,
            "short_window_s": window_s,
            "localization_short_window_displacement_m": localization_window_displacement,
            "odom_short_window_distance_m": odom_window_distance,
            "short_window_displacement_residual_m": window_residual,
            "covariance_xx": pose["cov_xx"], "covariance_yy": pose["cov_yy"],
            "covariance_yawyaw": pose["cov_yawyaw"],
            "particle_count": nearest_particles[1] if nearest_particles else None,
            "particle_x_std_m": nearest_particles[2] if nearest_particles else None,
            "particle_y_std_m": nearest_particles[3] if nearest_particles else None,
            "particle_yaw_circular_std_rad": nearest_particles[4] if nearest_particles else None,
            "particle_multimodality_proxy_m": nearest_particles[5] if nearest_particles else None,
            "particle_ess": nearest_particles[6] if nearest_particles else None,
            "gt_x_m": nearest_gt[1] if nearest_gt else None,
            "gt_y_m": nearest_gt[2] if nearest_gt else None,
            "gt_yaw_rad": nearest_gt[3] if nearest_gt else None,
            "gt_position_error_m": gt_position_error, "gt_yaw_error_rad": gt_yaw_error,
            "gt_frenet_s_error_m": None, "gt_frenet_d_error_m": None,
            "availability_note": "; ".join(notes),
        }
        rows.append({key: serialize(row.get(key)) for key in fields})
        previous_pose = pose
        if pose["header_t"] is not None:
            previous_pose_header_ns = pose["header_t"]

    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {len(rows)} LOC_UNKNOWN feature rows to {output}")
    print("selected topics:", ", ".join(f"{key}={value}" for key, value in chosen.items() if value))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
