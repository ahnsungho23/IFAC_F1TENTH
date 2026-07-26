#!/usr/bin/env python3
# Copyright 2026 2026_IFAC contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Exercise the Cartesian-static-obstacle to Cartesian-avoidance-path contract."""

import argparse
import csv
import math
import sys
import time

from f110_msgs.msg import Obstacle, ObstacleArray, OTWpntArray, Wpnt, WpntArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def load_waypoints(path):
    """Load the standard global_waypoints CSV."""
    result = []
    with open(path, newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            waypoint = Wpnt()
            waypoint.id = int(row['id'])
            waypoint.s_m = float(row['s'])
            waypoint.x_m = float(row['x_m'])
            waypoint.y_m = float(row['y_m'])
            waypoint.psi_rad = float(row['psi_rad'])
            waypoint.kappa_radpm = float(row['kappa_radpm'])
            waypoint.vx_mps = float(row['vx_mps'])
            waypoint.ax_mps2 = float(row['ax_mps2'])
            waypoint.d_left = float(row['d_left'])
            waypoint.d_right = float(row['d_right'])
            result.append(waypoint)
    return result


class CartesianPipelineProbe(Node):
    """Publish one Cartesian obstacle and wait for a valid Cartesian path."""

    def __init__(self, waypoints):
        super().__init__('cartesian_static_pipeline_probe')
        self.waypoints = waypoints
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.global_pub = self.create_publisher(WpntArray, '/global_waypoints', latched)
        self.obstacle_pub = self.create_publisher(
            ObstacleArray, '/perception/static_obstacles/cartesian', 10)
        self.odom_pub = self.create_publisher(Odometry, '/car_state/frenet/odom', 10)
        self.path_sub = self.create_subscription(
            OTWpntArray, '/avoid_waypoints', self.on_path, 10)
        self.passed = False
        self.failure = ''

        self.ego_index = max(1, len(waypoints) // 8)
        self.obstacle_index = (self.ego_index + 12) % len(waypoints)
        self.timer = self.create_timer(0.05, self.publish_inputs)

    def publish_inputs(self):
        """Publish synchronized test inputs."""
        now = self.get_clock().now().to_msg()
        global_message = WpntArray()
        global_message.wpnts = self.waypoints
        self.global_pub.publish(global_message)

        reference = self.waypoints[self.obstacle_index]
        obstacle = Obstacle()
        obstacle.id = 1
        obstacle.has_cartesian = True
        obstacle.x_center = reference.x_m
        obstacle.y_center = reference.y_m
        obstacle.radius = 0.20 * math.sqrt(2.0)
        obstacle.x_min = reference.x_m - 0.20
        obstacle.x_max = reference.x_m + 0.20
        obstacle.y_min = reference.y_m - 0.20
        obstacle.y_max = reference.y_m + 0.20
        obstacle.size = math.hypot(0.40, 0.40)
        obstacle.is_static = True
        obstacle.is_visible = True
        obstacle_message = ObstacleArray()
        obstacle_message.header.stamp = now
        obstacle_message.header.frame_id = 'map'
        obstacle_message.obstacles = [obstacle]
        self.obstacle_pub.publish(obstacle_message)

        ego = self.waypoints[self.ego_index]
        odometry = Odometry()
        odometry.header.stamp = now
        odometry.pose.pose.position.x = ego.s_m
        odometry.pose.pose.position.y = 0.0
        odometry.twist.twist.linear.x = max(0.5, ego.vx_mps)
        self.odom_pub.publish(odometry)

    def on_path(self, message):
        """Accept only a non-empty, finite Cartesian avoidance path."""
        if not message.wpnts:
            return
        if not all(
                math.isfinite(point.x_m) and math.isfinite(point.y_m) and
                math.isfinite(point.s_m) for point in message.wpnts):
            self.failure = 'path contains non-finite Cartesian coordinates'
            return
        if max(abs(point.d_m) for point in message.wpnts) < 0.05:
            self.failure = 'path did not move laterally around the obstacle'
            return
        self.passed = True


def main():
    """Run the probe against an already running local_planner_node."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--waypoints-csv', required=True)
    parser.add_argument('--timeout', type=float, default=8.0)
    arguments = parser.parse_args()

    waypoints = load_waypoints(arguments.waypoints_csv)
    if len(waypoints) < 30:
        print('FAIL: at least 30 waypoints are required')
        return 2

    rclpy.init()
    node = CartesianPipelineProbe(waypoints)
    deadline = time.monotonic() + arguments.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline and not node.passed:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.passed:
            print('PASS: Cartesian static obstacle produced a finite Cartesian avoidance path')
            return 0
        print(f'FAIL: {node.failure or "no non-empty avoidance path received"}')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
