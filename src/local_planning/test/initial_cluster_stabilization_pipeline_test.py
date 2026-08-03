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

"""늦게 들어온 군집 구성원이 횡방향 commitment 전에 포함되는지 검사한다."""

import math
import sys
import time

from f110_msgs.msg import Obstacle, ObstacleArray, OTWpntArray, Wpnt, WpntArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def latched_qos():
    """글로벌 waypoint에 사용하는 transient-local QoS를 반환한다."""
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


class InitialClusterProbe(Node):
    """첫 검출 직후 왼쪽을 막는 두 번째 군집 구성원을 추가한다."""

    def __init__(self):
        super().__init__('initial_cluster_stabilization_probe')
        self.global_pub = self.create_publisher(
            WpntArray, '/global_waypoints', latched_qos())
        self.obstacle_pub = self.create_publisher(ObstacleArray, '/static_obs', 10)
        self.odom_pub = self.create_publisher(
            Odometry, '/car_state/frenet/odom', 10)
        self.path_sub = self.create_subscription(
            OTWpntArray, '/avoid_waypoints', self.on_path, 10)
        self.started = time.monotonic()
        self.second_started = None
        self.second_publish_count = 0
        self.saw_preparation = False
        self.preparation_started = None
        self.passed = False
        self.failure = ''
        self.timer = self.create_timer(0.025, self.publish_inputs)

    @staticmethod
    def reference():
        """순서가 증가하는 s를 가진 직선 기준 경로를 만든다."""
        message = WpntArray()
        message.header.frame_id = 'map'
        for index in range(300):
            waypoint = Wpnt()
            waypoint.id = index
            waypoint.s_m = 0.1 * index
            waypoint.x_m = waypoint.s_m
            waypoint.y_m = 0.0
            waypoint.psi_rad = 0.0
            waypoint.kappa_radpm = 0.0
            waypoint.vx_mps = 2.0
            waypoint.d_left = 1.5
            waypoint.d_right = 1.5
            message.wpnts.append(waypoint)
        return message

    @staticmethod
    def obstacle(obstacle_id, x_min, x_max, y_min, y_max):
        """직선 기준 경로용 detector 형식 Frenet 장애물을 만든다."""
        obstacle = Obstacle()
        obstacle.id = obstacle_id
        obstacle.has_cartesian = True
        obstacle.is_static = True
        obstacle.is_visible = True
        obstacle.x_min = x_min
        obstacle.x_max = x_max
        obstacle.y_min = y_min
        obstacle.y_max = y_max
        obstacle.x_center = 0.5 * (x_min + x_max)
        obstacle.y_center = 0.5 * (y_min + y_max)
        obstacle.radius = 0.5 * math.hypot(x_max - x_min, y_max - y_min)
        obstacle.s_start = x_min
        obstacle.s_end = x_max
        obstacle.s_center = obstacle.x_center
        obstacle.d_right = y_min
        obstacle.d_left = y_max
        obstacle.d_center = obstacle.y_center
        obstacle.size = 2.0 * obstacle.radius
        return obstacle

    def publish_inputs(self):
        """첫 장애물 0.1초 뒤에 두 번째 군집 구성원을 발행한다."""
        stamp = self.get_clock().now().to_msg()
        reference = self.reference()
        reference.header.stamp = stamp
        self.global_pub.publish(reference)

        obstacles = ObstacleArray()
        obstacles.header.stamp = stamp
        obstacles.header.frame_id = 'map'
        obstacles.obstacles = [
            self.obstacle(17, 6.8, 7.2, -0.2, 0.2),
        ]
        if time.monotonic() - self.started >= 0.10:
            if self.second_started is None:
                self.second_started = time.monotonic()
            self.second_publish_count += 1
            obstacles.obstacles.append(
                self.obstacle(18, 7.3, 7.7, 0.2, 1.0))
        self.obstacle_pub.publish(obstacles)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = 'map'
        odometry.pose.pose.position.x = 0.0
        odometry.pose.pose.position.y = 0.0
        odometry.twist.twist.linear.x = 1.0
        self.odom_pub.publish(odometry)

    def on_path(self, message):
        """먼저 preparation을 확인하고 지연된 오른쪽 commitment를 요구한다."""
        if not message.wpnts:
            return
        if message.ot_line == 'raceline_static_prepare':
            self.saw_preparation = True
            if self.preparation_started is None:
                self.preparation_started = time.monotonic()
            if max(abs(point.d_m) for point in message.wpnts) > 1.0e-9:
                self.failure = 'preparation path has a lateral offset'
            return
        if message.ot_line != 'raceline_local_d_offset_spline':
            return
        if not self.saw_preparation or self.second_started is None:
            self.failure = 'avoidance committed before observing the complete cluster'
            return
        stabilization_duration = time.monotonic() - self.preparation_started
        if stabilization_duration < 0.13:
            self.failure = (
                'avoidance committed before the configured minimum stabilization duration: '
                f'{stabilization_duration:.3f}s')
            return
        if self.second_publish_count < 3:
            self.failure = (
                'avoidance committed after only '
                f'{self.second_publish_count} topic observations of the new cluster ID')
            return
        peak = max(message.wpnts, key=lambda point: abs(point.d_m)).d_m
        if peak >= -0.05:
            self.failure = (
                f'planner did not choose the right side of the expanded cluster: d={peak:.3f}')
            return
        self.passed = True


def main():
    """이미 실행 중인 local_planner_node를 대상으로 probe를 실행한다."""
    rclpy.init()
    node = InitialClusterProbe()
    deadline = time.monotonic() + 8.0
    try:
        while (
                rclpy.ok() and time.monotonic() < deadline and
                not node.passed and not node.failure):
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.passed:
            print(
                'PASS: minimum stabilization time and three real observations included the '
                'late cluster ID before selecting the safe right side')
            return 0
        print(f'FAIL: {node.failure or "no completed avoidance commitment"}')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
