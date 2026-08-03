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

"""합류 뒤 tail 장애물이 첫 maneuver를 멈추지 않고 연결되는지 검사한다."""

import math
import sys
import time

from f110_msgs.msg import Obstacle, ObstacleArray, OTWpntArray, StateMachine
from f110_msgs.msg import Wpnt, WpntArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def latched_qos():
    """글로벌 waypoint와 state에 사용하는 transient-local QoS를 반환한다."""
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


class PostMergeTailProbe(Node):
    """첫 경로의 controller 전용 tail 안에 두 번째 장애물을 추가한다."""

    def __init__(self):
        super().__init__('post_merge_tail_chaining_probe')
        self.global_pub = self.create_publisher(
            WpntArray, '/global_waypoints', latched_qos())
        self.obstacle_pub = self.create_publisher(
            ObstacleArray, '/static_obs', 10)
        self.odom_pub = self.create_publisher(
            Odometry, '/car_state/frenet/odom', 10)
        self.state_pub = self.create_publisher(
            StateMachine, '/state', latched_qos())
        self.avoid_sub = self.create_subscription(
            OTWpntArray, '/avoid_waypoints', self.on_avoid, 10)

        self.stage = 'first'
        self.ego_s = 0.0
        self.ego_d = 0.0
        self.second_s = None
        self.first_path_end = None
        self.outputs_with_second = 0
        self.passed = False
        self.failure = ''
        self.timer = self.create_timer(0.025, self.publish_inputs)

    @staticmethod
    def reference():
        """순서가 증가하는 s를 가진 직선 레이스 라인을 만든다."""
        message = WpntArray()
        message.header.frame_id = 'map'
        for index in range(400):
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
    def obstacle(obstacle_id, center_s):
        """왼쪽 maneuver가 필요한 오른쪽 장애물 상자를 만든다."""
        obstacle = Obstacle()
        obstacle.id = obstacle_id
        obstacle.has_cartesian = True
        obstacle.is_static = True
        obstacle.is_visible = True
        obstacle.x_center = center_s
        obstacle.y_center = -0.55
        obstacle.x_min = center_s - 0.2
        obstacle.x_max = center_s + 0.2
        obstacle.y_min = -1.2
        obstacle.y_max = 0.1
        obstacle.s_start = center_s - 0.2
        obstacle.s_end = center_s + 0.2
        obstacle.s_center = center_s
        obstacle.d_right = -1.2
        obstacle.d_left = 0.1
        obstacle.d_center = -0.55
        obstacle.radius = 0.5 * math.hypot(0.4, 1.3)
        obstacle.size = 2.0 * obstacle.radius
        return obstacle

    @staticmethod
    def path_d_at(message, target_s):
        """지정한 s에 가장 가까운 경로 표본의 d를 반환한다."""
        return min(message.wpnts, key=lambda waypoint: abs(
            waypoint.s_m - target_s)).d_m

    def publish_inputs(self):
        """최신 localization과 장애물 관측을 계속 발행한다."""
        stamp = self.get_clock().now().to_msg()
        reference = self.reference()
        reference.header.stamp = stamp
        self.global_pub.publish(reference)

        obstacles = ObstacleArray()
        obstacles.header.stamp = stamp
        obstacles.header.frame_id = 'map'
        obstacles.obstacles.append(self.obstacle(51, 7.0))
        if self.stage == 'chaining':
            obstacles.obstacles.append(self.obstacle(52, self.second_s))
        self.obstacle_pub.publish(obstacles)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = 'map'
        odometry.pose.pose.position.x = self.ego_s
        odometry.pose.pose.position.y = self.ego_d
        odometry.twist.twist.linear.x = 2.0
        self.odom_pub.publish(odometry)

        state = StateMachine()
        state.header.stamp = stamp
        state.header.frame_id = 'map'
        state.state = StateMachine.STATE_AVOID
        self.state_pub.publish(state)

    def on_avoid(self, message):
        """첫 회피가 두 번째 회피로 직접 교체되는지 확인한다."""
        if not message.wpnts:
            if self.stage == 'chaining':
                self.failure = 'avoid waypoints became empty while chaining'
            return

        if self.stage == 'first':
            if message.ot_line != 'raceline_local_d_offset_spline':
                return
            if max(waypoint.d_m for waypoint in message.wpnts) < 0.35:
                self.failure = 'first obstacle did not produce a left avoidance'
                return
            first_merge = message.wpnts[-1].s_m - 5.0
            self.first_path_end = message.wpnts[-1].s_m
            self.second_s = first_merge + 1.0
            self.ego_s = 8.5
            self.ego_d = self.path_d_at(message, self.ego_s)
            self.stage = 'chaining'
            self.get_logger().info(
                f'inserted obstacle 52 at s={self.second_s:.2f} inside '
                f'the first controller tail; ego d={self.ego_d:.2f}')
            return

        self.outputs_with_second += 1
        if message.ot_line in (
                'raceline_static_safe_stop',
                'raceline_static_prepare',
                'raceline_global_handoff'):
            self.failure = (
                f'unexpected {message.ot_line} instead of direct chaining')
            return
        if message.ot_line != 'raceline_local_d_offset_spline':
            return
        if message.wpnts[-1].s_m <= self.first_path_end + 1.0:
            return
        if abs(message.wpnts[0].d_m - self.ego_d) > 0.10:
            self.failure = 'next maneuver was discontinuous from the current ego d'
            return
        if max(waypoint.d_m for waypoint in message.wpnts) < 0.35:
            self.failure = 'next maneuver did not avoid obstacle 52 on the left'
            return
        self.passed = True


def main():
    """새로 실행한 local_planner_node를 대상으로 probe를 실행한다."""
    rclpy.init()
    node = PostMergeTailProbe()
    deadline = time.monotonic() + 10.0
    try:
        while (
                rclpy.ok() and time.monotonic() < deadline and
                not node.passed and not node.failure):
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.passed:
            print(
                'PASS: post-merge-tail obstacle replaced the first commitment '
                f'directly after {node.outputs_with_second} outputs')
            return 0
        print(f'FAIL: {node.failure or "post-merge chaining did not complete"}')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
