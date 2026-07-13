#!/usr/bin/env python3
"""Overtake state-machine integration test for the opponent_detector overtake planner.

Scenario: a straight track, a SLOW MOVING opponent ahead on the raceline (dynamic, blocks the
corridor), the ego closing in faster. Expected state-machine behaviour:

  1. COMMIT   : non-empty overtake OTs (ot_line == "overtake") published while committed
  2. SUSTAIN  : the commit stays alive through replans all the way past the opponent — the
                runway extension keeps the plan feasible as the gap shrinks, so the planner
                must NOT abort into trailing mid-approach (that was the tight-trail trap bug)
  3. COMPLETE : the maneuver ends with ONE empty OT (opponent passed / out of window)
  4. SILENCE  : no further OT messages after the final empty OT

Trail OTs (ot_line == "trail") are OPTIONAL in this scenario (they may appear briefly during
opponent velocity-estimate warmup); when present they must stay on the raceline and slow to the
opponent speed. Dedicated trailing/escalation coverage lives in trail_to_overtake_test.py.

The harness ego FOLLOWS the latest published local path laterally (like the real controller
does) — the planner's ego-deviation guard replans from the current pose whenever the car is not
on the committed line, so a laterally frozen ego would keep resetting the swing-out.

Run (with the workspace sourced), while `opponent_detector_node` is running in VEHICLE mode
(the harness publishes ego odom on /pf/pose/odom — do NOT launch with simulator:=true):
    python3 src/opponent_detector/test/commit_lock_test.py
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

# Scenario: opponent starts 4 m ahead ON the raceline, crawling at 0.8 m/s (dynamic, blocking).
# Ego advances at 2.0 m/s -> closes at 1.2 m/s, catches around t=3.3 s, completes around t~5.5 s.
OPP_START_X = 4.0
OPP_SPEED = 0.8
EGO_START_X = 0.0
EGO_SPEED = 2.0
DURATION_S = 10.0
RATE_HZ = 20.0


def latched_qos():
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def facet_points(cx, cy, half_width=0.12, n=10):
    """A short vertical facet of surface points centred at (cx, cy)."""
    return [(cx, cy + (2.0 * i / (n - 1) - 1.0) * half_width) for i in range(n)]


class Harness(Node):
    def __init__(self):
        super().__init__("overtake_state_machine_test")
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
        self.empty_times = []
        self.first_commit_t = None
        self.first_trail_t = None
        self.last_trail_t = None
        self.last_msg_t = None
        self.path_is_local = True     # every published path must NOT cover the whole track
        self.path_avoids_opp = True   # overtake path must keep lateral clearance near the opponent
        self.trail_on_raceline = True # trail path must stay on the raceline (|d| small)
        self.trail_slows_to_opp = True  # trail path tail speed must match the opponent speed

        self.done = False
        self.ok = False
        self.follow_path = None  # latest non-empty local path [(x, y)]; ego tracks its lateral
        self.ego_y = 0.0
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

        # all-free occupancy grid
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

        # static TF map -> laser (identity; scan carries ego-relative ranges)
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
        # lateral: perfect tracking of the latest local path (ease back to the raceline without it)
        if self.follow_path:
            self.ego_y = min(self.follow_path, key=lambda p: abs(p[0] - ego_x))[1]
        else:
            self.ego_y += max(-0.05, min(0.05, -self.ego_y))
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = ego_x
        odom.pose.pose.position.y = self.ego_y
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = EGO_SPEED
        self.odom_pub.publish(odom)

        # moving opponent on the raceline: rasterize its facet per BEAM over the angular span
        # (contiguous beam indices — per-point sampling leaves index holes at close range and
        # breaks the detector's adaptive-breakpoint clustering); the ego may be laterally offset
        opp_x = OPP_START_X + OPP_SPEED * t
        ranges = [float("inf")] * NUM_BEAMS
        dx = opp_x - ego_x
        if dx > 0.15:
            half_w = 0.12
            a0 = math.atan2(-half_w - self.ego_y, dx)
            a1 = math.atan2(half_w - self.ego_y, dx)
            i0 = int(math.ceil((a0 - ANGLE_MIN) / ANGLE_INC))
            i1 = int(math.floor((a1 - ANGLE_MIN) / ANGLE_INC))
            for idx in range(max(0, i0), min(NUM_BEAMS - 1, i1) + 1):
                ang = ANGLE_MIN + idx * ANGLE_INC
                r = dx / math.cos(ang)
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

        # dynamic TF: laser rides with the ego
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "map"
        tf.child_frame_id = "laser"
        tf.transform.translation.x = ego_x
        tf.transform.translation.y = self.ego_y
        tf.transform.rotation.w = 1.0
        self.dyn_tf.sendTransform(tf)

        self.last_ego_x = ego_x
        self.last_opp_x = opp_x

    def on_ot(self, msg: OTWpntArray):
        t = time.time() - self.t0
        if not msg.wpnts:
            self.empty_times.append(t)
            self.follow_path = None
            return
        self.follow_path = [(w.x_m, w.y_m) for w in msg.wpnts]
        self.last_msg_t = t
        # local-path check: the published segment must be much shorter than the track (30 m)
        s_vals = [w.s_m for w in msg.wpnts]
        if max(s_vals) - min(s_vals) > 20.0:
            self.path_is_local = False
        if msg.ot_line == "trail":
            self.trail_count += 1
            if self.first_trail_t is None:
                self.first_trail_t = t
            self.last_trail_t = t
            # the follow path stays on the raceline...
            if max(abs(w.d_m) for w in msg.wpnts) > 0.35:
                self.trail_on_raceline = False
            # ...and its tail decelerates to the opponent speed (0.8 m/s here)
            if msg.wpnts[-1].vx_mps > OPP_SPEED + 0.4:
                self.trail_slows_to_opp = False
        else:
            self.overtake_count += 1
            if self.first_commit_t is None:
                self.first_commit_t = t
            # clearance check: near the opponent's s, the path must be laterally offset
            opp_x = getattr(self, "last_opp_x", None)
            if opp_x is not None:
                near = [w for w in msg.wpnts if abs(w.s_m - opp_x) < 0.5]
                if near and max(abs(w.d_m) for w in near) < 0.3:
                    self.path_avoids_opp = False

    def finish(self):
        print("\n============= OVERTAKE STATE-MACHINE TEST RESULT =============", flush=True)
        commit_s = f"{self.first_commit_t:.2f}s" if self.first_commit_t is not None else "-"
        trail_s = f"{self.first_trail_t:.2f}s" if self.first_trail_t is not None else "-"
        empties = ", ".join(f"{e:.2f}s" for e in self.empty_times) or "-"
        # SILENCE: no path message after the FINAL empty OT
        silent = (not self.empty_times) or (self.last_msg_t is None) or (
            self.last_msg_t < self.empty_times[-1])
        # SUSTAIN: the commit must survive the whole approach without falling back to trailing
        # AFTER it started (the runway extension guarantees a feasible replan as the gap shrinks;
        # a trail message after first_commit_t means the plan was lost mid-approach = trap bug)
        sustained = (self.first_commit_t is not None) and (
            self.last_trail_t is None or self.last_trail_t < self.first_commit_t)
        print(f"  COMMIT  overtake OTs published:          {self.overtake_count} (first at {commit_s})", flush=True)
        print(f"  TRAIL   follow OTs (optional, warmup):   {self.trail_count} (first at {trail_s})", flush=True)
        print(f"  SUSTAIN no trail after commit started:   {sustained}", flush=True)
        print(f"  CLEAR   empty OTs (completion):          {len(self.empty_times)} (at {empties})", flush=True)
        print(f"  SILENCE no messages after final empty:   {silent}", flush=True)
        print(f"  every path covers only a local segment:  {self.path_is_local}", flush=True)
        print(f"  overtake path laterally clears opponent: {self.path_avoids_opp}", flush=True)
        print(f"  trail path stays on the raceline:        {self.trail_on_raceline}", flush=True)
        print(f"  trail tail speed matches the opponent:   {self.trail_slows_to_opp}", flush=True)
        self.ok = (
            self.overtake_count >= 3
            and sustained
            and len(self.empty_times) == 1
            and silent
            and self.path_is_local
            and self.path_avoids_opp
            and self.trail_on_raceline
            and self.trail_slows_to_opp
        )
        print(f"  ==> {'PASS' if self.ok else 'FAIL'}", flush=True)
        print("==============================================================\n", flush=True)
        self.done = True


def main():
    rclpy.init()
    node = Harness()
    print("[harness] running overtake state-machine test...", flush=True)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if node.ok else 1)


if __name__ == "__main__":
    main()
