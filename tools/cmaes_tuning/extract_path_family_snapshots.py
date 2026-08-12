#!/usr/bin/env python3
"""Extract static planner inputs from existing lockstep bags without replaying them.

The output is a deterministic, line-oriented stream consumed by the C++ path-family
audit.  It contains the production YAML values, one global reference, and every
non-empty exact-stamp `/static_obs` + Frenet-odometry pair.  Geometry validation is
deliberately not performed here; that remains in the production C++ planner.
"""

from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path
import sys
from typing import Any

import yaml

TOOL_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOL_ROOT))

from cmaes_tuning.bag_reader import read_bag  # noqa: E402


SCALAR_PARAMETERS = (
    "detection_lookahead_m",
    "obstacle_cluster_gap_m",
    "obstacle_longitudinal_padding_m",
    "vehicle_length_m",
    "vehicle_half_width_m",
    "safety_margin_m",
    "tracking_error_reserve_m",
    "wall_safety_margin_m",
    "fallback_track_half_width_m",
    "outside_line_transition_scale",
    "post_merge_lookahead_m",
    "post_merge_min_time_sec",
    "minimum_target_offset_m",
    "maximum_target_offset_m",
    "target_d_candidate_count",
    "maximum_lateral_slope",
    "maximum_curvature_radpm",
    "maximum_curvature_rate_radpm2",
    "safe_stop_buffer_m",
    "safe_stop_deceleration_mps2",
    "minimum_path_points",
    "uncertainty_sigma_scale",
    "uncertainty_min_longitudinal_inflation_m",
    "uncertainty_min_lateral_inflation_m",
    "uncertainty_max_lateral_inflation_m",
)

VECTOR_PARAMETERS = (
    "tracking_error_lut_speed_bins_mps",
    "tracking_error_lut_curvature_bins_radpm",
    "tracking_error_lut_values_m",
    "avoidance_velocity_limit_speed_bins_mps",
    "avoidance_velocity_limit_lateral_accel_mps2",
    "pre_apex_distances_m",
    "post_apex_distances_m",
    "entry_transition_fractions",
    "transition_distance_scales",
)


def stamp_ns(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def float_token(value: Any) -> str:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"non-finite snapshot value: {value!r}")
    return format(number, ".17g")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def extract(args: argparse.Namespace) -> None:
    bag = read_bag(
        args.bag,
        topics=("/global_waypoints", "/static_obs", "/car_state/frenet/odom"),
    )
    references = bag.topic("/global_waypoints")
    if not references:
        raise ValueError("bag has no /global_waypoints message")
    reference = references[-1].message

    odometry_by_stamp = {
        stamp_ns(record.message): record.message
        for record in bag.topic("/car_state/frenet/odom")
    }
    frames: list[tuple[int, int, Any, Any]] = []
    for record in bag.topic("/static_obs"):
        message = record.message
        logical_stamp = stamp_ns(message)
        odometry = odometry_by_stamp.get(logical_stamp)
        if odometry is None or (not message.obstacles and not args.include_empty):
            continue
        frames.append((logical_stamp, record.timestamp_ns, odometry, message))
    if not frames:
        raise ValueError("bag has no non-empty exact-stamp planner input pair")

    config_path = args.config.resolve()
    config_document = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    parameters = config_document["local_planner_node"]["ros__parameters"]
    missing = [
        name for name in (*SCALAR_PARAMETERS, *VECTOR_PARAMETERS)
        if name not in parameters
    ]
    if missing:
        raise ValueError(f"production config is missing audit parameters: {missing}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="\n") as output:
        output.write("PATH_FAMILY_STREAM_V1\n")
        output.write(f"SCENARIO\t{args.scenario}\n")
        output.write(f"SOURCE_BAG\t{bag.uri}\n")
        output.write(f"CONFIG_PATH\t{config_path}\n")
        output.write(f"CONFIG_SHA256\t{sha256_file(config_path)}\n")
        for name in SCALAR_PARAMETERS:
            output.write(f"PARAM\t{name}\t{float_token(parameters[name])}\n")
        for name in VECTOR_PARAMETERS:
            values = parameters[name]
            output.write(
                "PARAMV\t" + name + "\t" + str(len(values)) + "\t" +
                "\t".join(float_token(value) for value in values) + "\n"
            )

        output.write(f"REFERENCE\t{len(reference.wpnts)}\n")
        for waypoint in reference.wpnts:
            values = (
                int(waypoint.id),
                waypoint.s_m,
                waypoint.d_m,
                waypoint.x_m,
                waypoint.y_m,
                waypoint.d_right,
                waypoint.d_left,
                waypoint.psi_rad,
                waypoint.kappa_radpm,
                waypoint.vx_mps,
                waypoint.ax_mps2,
            )
            output.write(
                "W\t" + str(values[0]) + "\t" +
                "\t".join(float_token(value) for value in values[1:]) + "\n"
            )

        for logical_stamp, record_stamp, odometry, obstacle_array in frames:
            output.write(
                "FRAME\t" + str(logical_stamp) + "\t" + str(record_stamp) + "\t" +
                float_token(odometry.pose.pose.position.x) + "\t" +
                float_token(odometry.pose.pose.position.y) + "\t" +
                float_token(abs(odometry.twist.twist.linear.x)) + "\t" +
                str(len(obstacle_array.obstacles)) + "\n"
            )
            for obstacle in obstacle_array.obstacles:
                values = (
                    int(obstacle.id),
                    obstacle.s_center,
                    obstacle.s_start,
                    obstacle.s_end,
                    obstacle.d_right,
                    obstacle.d_left,
                    obstacle.size,
                    obstacle.s_var,
                    obstacle.d_var,
                )
                output.write(
                    "O\t" + str(values[0]) + "\t" +
                    "\t".join(float_token(value) for value in values[1:]) + "\n"
                )
            output.write("END_FRAME\n")
        output.write("END_STREAM\n")

    print(
        f"scenario={args.scenario} frames={len(frames)} "
        f"reference={len(reference.wpnts)} output={args.output}"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--include-empty", action="store_true",
        help="retain exact-stamp empty obstacle frames for lifecycle-time reconstruction")
    return parser.parse_args()


if __name__ == "__main__":
    extract(parse_args())
