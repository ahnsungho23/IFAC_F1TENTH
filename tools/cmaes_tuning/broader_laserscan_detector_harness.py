#!/usr/bin/env python3
"""Feed one deterministic scan sequence through the actual obstacle detector node.

The detector is launched separately in an isolated ROS_DOMAIN_ID.  This harness
only publishes diagnostic inputs and records passive production outputs/events.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
import yaml

from f110_msgs.msg import ObstacleArray, Wpnt, WpntArray


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def set_stamp(message, value: int) -> None:
    message.header.stamp.sec = value // 1_000_000_000
    message.header.stamp.nanosec = value % 1_000_000_000


def latched_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def sensor_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )


def quaternion_z_w(yaw: float) -> tuple[float, float]:
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def obstacle_payload(message: ObstacleArray) -> list[dict[str, object]]:
    return [
        {
            "id": int(obstacle.id),
            "s_center": float(obstacle.s_center),
            "s_start": float(obstacle.s_start),
            "s_end": float(obstacle.s_end),
            "d_center": float(obstacle.d_center),
            "d_right": float(obstacle.d_right),
            "d_left": float(obstacle.d_left),
            "vs": float(obstacle.vs),
            "vd": float(obstacle.vd),
            "is_static": bool(obstacle.is_static),
            "is_visible": bool(obstacle.is_visible),
            "has_cartesian": bool(obstacle.has_cartesian),
            "x_min": float(obstacle.x_min),
            "x_max": float(obstacle.x_max),
            "y_min": float(obstacle.y_min),
            "y_max": float(obstacle.y_max),
        }
        for obstacle in message.obstacles
    ]


class Harness(Node):
    def __init__(self, request: dict[str, object], arrays: dict[str, np.ndarray]):
        super().__init__("uniform_laserscan_detector_harness")
        self.request = request
        self.arrays = arrays
        self.events: dict[int, dict[str, object]] = {}
        self.outputs: dict[str, dict[int, list[dict[str, object]]]] = {
            "static": {},
            "confirmed_static": {},
            "opp": {},
        }
        self.wp_pub = self.create_publisher(WpntArray, "/global_waypoints", latched_qos())
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", latched_qos())
        self.odom_pub = self.create_publisher(Odometry, "/ego_racecar/odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", sensor_qos())
        self.create_subscription(String, "/cma_replay/detector_events", self.on_event, 1000)
        self.create_subscription(
            ObstacleArray, "/static_obs", lambda message: self.on_output("static", message), 10
        )
        self.create_subscription(
            ObstacleArray,
            "/confirmed_static_obs",
            lambda message: self.on_output("confirmed_static", message),
            10,
        )
        self.create_subscription(
            ObstacleArray, "/opp_obs", lambda message: self.on_output("opp", message), 10
        )

    def on_event(self, message: String) -> None:
        event = json.loads(message.data)
        self.events[int(event["scan_stamp_ns"])] = event

    def on_output(self, name: str, message: ObstacleArray) -> None:
        self.outputs[name][stamp_ns(message)] = obstacle_payload(message)

    def make_waypoints(self) -> WpntArray:
        result = WpntArray()
        result.header.frame_id = "map"
        for index, source in enumerate(self.request["waypoints"]):
            waypoint = Wpnt()
            waypoint.id = int(source.get("id", index))
            waypoint.s_m = float(source["s_m"])
            waypoint.x_m = float(source["x_m"])
            waypoint.y_m = float(source["y_m"])
            waypoint.psi_rad = float(source.get("psi_rad", 0.0))
            waypoint.d_left = float(source.get("d_left", 1.5))
            waypoint.d_right = float(source.get("d_right", 1.5))
            waypoint.vx_mps = float(source.get("vx_mps", 4.0))
            result.wpnts.append(waypoint)
        return result

    def make_map(self) -> OccupancyGrid:
        if "free_map" in self.request:
            source = self.request["free_map"]
            result = OccupancyGrid()
            result.header.frame_id = "map"
            result.info.resolution = float(source.get("resolution_m", 0.05))
            result.info.width = int(source.get("width", 800))
            result.info.height = int(source.get("height", 800))
            result.info.origin.position.x = float(source.get("origin_x_m", -20.0))
            result.info.origin.position.y = float(source.get("origin_y_m", -20.0))
            result.info.origin.orientation.w = 1.0
            result.data = [0] * (result.info.width * result.info.height)
            return result
        path = Path(self.request["map_yaml"])
        metadata = yaml.safe_load(path.read_text(encoding="utf-8"))
        image_path = Path(metadata["image"])
        if not image_path.is_absolute():
            image_path = path.parent / image_path
        pixels = np.asarray(Image.open(image_path).convert("L"), dtype=np.uint8)
        occupied = np.flipud(pixels) <= int(self.request.get("occupied_gray_threshold", 128))
        result = OccupancyGrid()
        result.header.frame_id = "map"
        result.info.resolution = float(metadata["resolution"])
        result.info.width = int(occupied.shape[1])
        result.info.height = int(occupied.shape[0])
        result.info.origin.position.x = float(metadata["origin"][0])
        result.info.origin.position.y = float(metadata["origin"][1])
        z, w = quaternion_z_w(float(metadata["origin"][2]))
        result.info.origin.orientation.z = z
        result.info.origin.orientation.w = w
        result.data = np.where(occupied, 100, 0).astype(np.int8).ravel().tolist()
        return result

    def spin_until(self, predicate, timeout_s: float, description: str) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return
        raise TimeoutError(description)

    def run(self) -> dict[str, object]:
        waypoint_message = self.make_waypoints()
        map_message = self.make_map()
        # Latched messages are repeated while discovery settles.  No scan is
        # published until both production subscriptions are visible.
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            self.wp_pub.publish(waypoint_message)
            self.map_pub.publish(map_message)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.wp_pub.get_subscription_count() and self.map_pub.get_subscription_count():
                break
        else:
            raise TimeoutError("detector did not discover latched inputs")
        # Allow CLCS creation and map callback to complete before the first scan.
        settle_until = time.monotonic() + 0.8
        while time.monotonic() < settle_until:
            rclpy.spin_once(self, timeout_sec=0.02)

        for index, frame in enumerate(self.request["frames"]):
            stamp = int(frame["stamp_ns"])
            odom = Odometry()
            set_stamp(odom, stamp)
            odom.header.frame_id = "map"
            odom.child_frame_id = "ego_racecar/base_link"
            odom.pose.pose.position.x = float(frame["x_m"])
            odom.pose.pose.position.y = float(frame["y_m"])
            z, w = quaternion_z_w(float(frame["yaw_rad"]))
            odom.pose.pose.orientation.z = z
            odom.pose.pose.orientation.w = w
            odom.twist.twist.linear.x = float(frame.get("speed_mps", 4.0))
            odom.twist.twist.angular.z = float(frame.get("yaw_rate_radps", 0.0))

            scan = LaserScan()
            set_stamp(scan, stamp)
            scan.header.frame_id = "ego_racecar/laser"
            scan.angle_min = float(self.request["angle_min_rad"])
            scan.angle_max = float(self.request["angle_max_rad"])
            scan.angle_increment = float(self.request["angle_increment_rad"])
            scan.range_min = 0.0
            scan.range_max = 30.0
            scan.ranges = np.asarray(self.arrays[f"scan_{index:04d}"], dtype="<f4").tolist()
            self.odom_pub.publish(odom)
            self.scan_pub.publish(scan)
            self.spin_until(
                lambda stamp=stamp: stamp in self.events,
                3.0,
                f"detector event missing for {stamp}",
            )
            self.spin_until(
                lambda stamp=stamp: all(
                    stamp in self.outputs[name] for name in ("static", "confirmed_static", "opp")
                ),
                2.0,
                f"detector layer output missing for {stamp}",
            )

        ordered = []
        for frame in self.request["frames"]:
            stamp = int(frame["stamp_ns"])
            ordered.append(
                {
                    "stamp_ns": stamp,
                    "event": self.events[stamp],
                    "published_static": self.outputs["static"][stamp],
                    "published_confirmed_static": self.outputs["confirmed_static"][stamp],
                    "published_opp": self.outputs["opp"][stamp],
                }
            )
        return {
            "schema": "uniform_laserscan_detector_sequence/1",
            "case_id": self.request["case_id"],
            "mode": self.request["mode"],
            "frames": ordered,
        }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--request", type=Path, required=True)
    parser.add_argument("--scans", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    request = json.loads(args.request.read_text(encoding="utf-8"))
    with np.load(args.scans) as loaded:
        arrays = {key: np.asarray(loaded[key], dtype=np.float64) for key in loaded.files}
    rclpy.init()
    node = Harness(request, arrays)
    try:
        result = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
