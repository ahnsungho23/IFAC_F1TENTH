#!/usr/bin/env python3
"""Trail-to-overtake escalation test for the opponent_detector overtake planner.

Scenario: the ego is ALREADY in a tight-trailing steady state — crawling at the opponent's speed
(0.8 m/s) only 2 m behind it on a straight track. Before the runway extension this state was a
trap: the predicted catch point was so close that no clearing apex was reachable under the
slope/curvature limits, every commit rejected with "no room", and the planner trailed forever.

Expected behaviour now:
  1. COMMIT      : an overtake is committed FROM the tight-trail gap (runway pushed out so the
                   ego swings out first and passes after reaching the apex)
  2. APEX        : the committed path reaches a clearing lateral offset (|d| >= 0.6)
  3. GAP HOLD    : samples that are NOT yet laterally clear of the opponent and inside the hold
                   gap stay near the opponent speed (the ego must not close on a still-centered
                   opponent mid-ramp)
  4. ACCELERATION: once laterally clear the profile accelerates well above the opponent speed

Run (with the workspace sourced), while `opponent_detector_node` is running in VEHICLE mode
(the harness publishes ego odom on /pf/pose/odom — do NOT launch with simulator:=true):
    python3 src/opponent_detector/test/trail_to_overtake_test.py
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from f110_msgs.msg import WpntArray, Wpnt, OTWpntArray


NUM_BEAMS = 1080
FOV = 4.7
ANGLE_MIN = -FOV / 2.0
ANGLE_INC = FOV / NUM_BEAMS
RANGE_MAX = 30.0

# Scenario: tight-trail steady state — ego matches the slow opponent's speed 2 m behind it.
OPP_START_X = 2.0
OPP_SPEED = 0.8
EGO_START_X = 0.0
EGO_SPEED = 0.8
DURATION_S = 6.0
RATE_HZ = 20.0


def latched_qos():
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class Harness(Node):
    def __init__(self):
        super().__init__("trail_to_overtake_test")
        self.wpnt_pub = self.create_publisher(WpntArray, "/global_waypoints", latched_qos())
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", latched_qos())
        self.odom_pub = self.create_publisher(Odometry, "/pf/pose/odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.static_tf = StaticTransformBroadcaster(self)
        self.dyn_tf = TransformBroadcaster(self)

        self.sub_ot = self.create_subscription(
            OTWpntArray, "/overtake_waypoints", self.on_ot, 10)

        self.overtake_count = 0
        self.trail_count = 0
        self.first_commit_t = None
        self.apex_reached = 0.0        # max |d| seen on any overtake path
        self.gap_hold_ok = True        # pre-clear samples must stay near the opponent speed
        self.accelerates = False       # some laterally-clear sample runs well above opp speed

        self.done = False
        self.ok = False
        self.publish_static_inputs()
        self.t0 = time.time()
        self.timer = self.create_timer(1.0 / RATE_HZ, self.tick)

    def publish_static_inputs(self):
        # straight raceline along +x, 0.1 m spacing, corridor +-2 m, raceline speed 5 m/s
        arr = WpntArray()
        arr.header.frame_id = "map"
        for i in range(301):
            w = Wpnt()
            w.id = i
            w.s_m = i * 0.1
            w.x_m = i * 0.1
            w.y_m = 0.0
            w.psi_rad = 0.0
            w.d_left = 2.0
            w.d_right = 2.0
            w.vx_mps = 5.0
            arr.wpnts.append(w)
        self.wpnt_pub.publish(arr)

        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.resolution = 0.05
        grid.info.width = 700
        grid.info.height = 200
        grid.info.origin.position.x = -5.0
        grid.info.origin.position.y = -5.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (grid.info.width * grid.info.height)
        self.map_pub.publish(grid)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "laser"
        tf.transform.rotation.w = 1.0
        self.static_tf.sendTransform(tf)

    def tick(self):
        t = time.time() - self.t0
        if t > DURATION_S:
            self.finish()
            return
        stamp = self.get_clock().now().to_msg()

        ego_x = EGO_START_X + EGO_SPEED * t
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = ego_x
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = EGO_SPEED
        self.odom_pub.publish(odom)

        # constant-gap opponent facet across CONTIGUOUS beams
        opp_x = OPP_START_X + OPP_SPEED * t
        ranges = [float("inf")] * NUM_BEAMS
        rel_x = opp_x - ego_x
        if rel_x > 0.15:
            half_w = 0.12
            ang_span = math.atan2(half_w, rel_x)
            i0 = int(math.ceil((-ang_span - ANGLE_MIN) / ANGLE_INC))
            i1 = int(math.floor((ang_span - ANGLE_MIN) / ANGLE_INC))
            for idx in range(max(0, i0), min(NUM_BEAMS - 1, i1) + 1):
                ang = ANGLE_MIN + idx * ANGLE_INC
                r = rel_x / math.cos(ang)
                if 0.1 < r < RANGE_MAX:
                    ranges[idx] = r
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "laser"
        scan.angle_min = ANGLE_MIN
        scan.angle_max = ANGLE_MIN + FOV
        scan.angle_increment = ANGLE_INC
        scan.range_min = 0.1
        scan.range_max = RANGE_MAX
        scan.ranges = ranges
        self.scan_pub.publish(scan)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "map"
        tf.child_frame_id = "laser"
        tf.transform.translation.x = ego_x
        tf.transform.rotation.w = 1.0
        self.dyn_tf.sendTransform(tf)

        self.last_ego_x = ego_x
        self.last_opp_x = opp_x

    def on_ot(self, msg: OTWpntArray):
        t = time.time() - self.t0
        if not msg.wpnts:
            return
        if msg.ot_line == "trail":
            self.trail_count += 1
            return
        self.overtake_count += 1
        if self.first_commit_t is None:
            self.first_commit_t = t
        max_d = max(abs(w.d_m) for w in msg.wpnts)
        self.apex_reached = max(self.apex_reached, max_d)
        # gap hold: laterally-unclear samples still ahead of the pass must not run much faster
        # than the opponent (only meaningful before the apex — check samples with tiny |d| in
        # the first half of the path)
        half_span = (msg.wpnts[0].s_m + msg.wpnts[-1].s_m) / 2.0
        opp_x = getattr(self, "last_opp_x", OPP_START_X)
        for w in msg.wpnts:
            if w.s_m < half_span and abs(w.d_m) < 0.2 and abs(w.s_m - opp_x) < 1.0:
                if w.vx_mps > OPP_SPEED + 0.6:
                    self.gap_hold_ok = False
            if abs(w.d_m) >= 0.55 and w.vx_mps >= 2.0:
                self.accelerates = True

    def finish(self):
        print("\n=========== TRAIL -> OVERTAKE ESCALATION TEST RESULT ===========", flush=True)
        commit_s = f"{self.first_commit_t:.2f}s" if self.first_commit_t is not None else "-"
        print(f"  COMMIT   overtake OTs from tight trail:  {self.overtake_count} (first at {commit_s})", flush=True)
        print(f"  (trail msgs before/alongside commits:    {self.trail_count})", flush=True)
        print(f"  APEX     max |d| on the committed path:  {self.apex_reached:.2f} (>= 0.60)", flush=True)
        print(f"  GAPHOLD  unclear samples near opp speed:  {self.gap_hold_ok}", flush=True)
        print(f"  ACCEL    clear samples speed up (>=2.0):  {self.accelerates}", flush=True)
        self.ok = (
            self.overtake_count >= 3
            and self.apex_reached >= 0.60
            and self.gap_hold_ok
            and self.accelerates
        )
        print(f"  ==> {'PASS' if self.ok else 'FAIL'}", flush=True)
        print("================================================================\n", flush=True)
        self.done = True


def main():
    rclpy.init()
    node = Harness()
    print("[harness] running trail->overtake escalation test...", flush=True)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if node.ok else 1)


if __name__ == "__main__":
    main()
