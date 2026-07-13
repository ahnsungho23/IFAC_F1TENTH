#!/usr/bin/env python3
"""Off-path recovery test for the opponent_detector overtake planner.

Covers the "weird local path" edge cases: a published path must always relate to where the car
ACTUALLY is, and a stale ego Frenet projection must never keep producing paths.

Scenario: straight track, slow opponent ahead, ego closing in (same as commit_lock_test), then
two injected faults:

  Phase A (ego knocked off the committed path, t=1.5s): while COMMITTED the ego odom teleports
      0.9 m laterally to the side OPPOSITE the apex (simulates a bump/slide/wall contact).
      Expected: the ego-deviation guard reacts within a few cycles — every non-empty OT after
      the teleport must be anchored near the NEW ego pose (abort->trail rebuilds from the
      current pose), never the stale committed line (which sits >0.9 m away).

  Phase B (ego off the projection domain, t=2.8s): the ego odom teleports 30 m off the track
      (CLCS projection fails -> the last valid (s,d) would go stale). Expected: after
      ego_pose_grace_s the active path is cleared with ONE empty OT and the planner goes
      SILENT — no path may ever be planned from the frozen pre-crash pose.

Run (with the workspace sourced), while `opponent_detector_node` is running in VEHICLE mode
(the harness publishes ego odom on /pf/pose/odom — do NOT launch with simulator:=true):
    python3 src/opponent_detector/test/offpath_recovery_test.py
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

OPP_START_X = 4.0
OPP_SPEED = 0.8
EGO_SPEED = 2.0
RATE_HZ = 20.0
DURATION_S = 7.0

T_OFFPATH = 1.5      # Phase A: lateral teleport while committed
OFFPATH_Y = -0.9     # opposite the (left, +d) apex -> the stale line is >= 0.9 m away
T_OFFDOMAIN = 2.8    # Phase B: teleport far off the track (CLCS projection fails)
OFFDOMAIN_Y = 30.0
NEAR_EGO_M = 0.5     # a recovered path must pass within this distance of the ego
REACT_DELAY = 0.20   # cycles the guard may need before the first reaction message
PHASE_B_CLEAR_S = 1.0  # empty OT must arrive within this after the off-domain teleport


def latched_qos():
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class Harness(Node):
    def __init__(self):
        super().__init__("offpath_recovery_test")
        self.wpnt_pub = self.create_publisher(WpntArray, "/global_waypoints", latched_qos())
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", latched_qos())
        self.odom_pub = self.create_publisher(Odometry, "/pf/pose/odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.static_tf = StaticTransformBroadcaster(self)
        self.dyn_tf = TransformBroadcaster(self)

        self.sub_ot = self.create_subscription(
            OTWpntArray, "/overtake_waypoints", self.on_ot, 10)

        self.commit_count = 0            # overtake OTs before Phase A
        self.reaction_count = 0          # messages seen in the Phase A reaction window
        self.stale_paths_a = 0           # non-empty OTs far from the ego after the teleport
        self.empties_b = 0               # empty OTs in the Phase B clear window
        self.late_paths_b = 0            # non-empty OTs after the Phase B clear window
        self.ego_x = 0.0
        self.ego_y = 0.0

        self.done = False
        self.ok = False
        self.publish_static_inputs()
        self.t0 = time.time()
        self.timer = self.create_timer(1.0 / RATE_HZ, self.tick)

    def publish_static_inputs(self):
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

        self.ego_x = EGO_SPEED * t
        if t >= T_OFFDOMAIN:
            self.ego_y = OFFDOMAIN_Y      # Phase B: off the CLCS projection domain
        elif t >= T_OFFPATH:
            self.ego_y = OFFPATH_Y        # Phase A: knocked off the committed path
        else:
            self.ego_y = 0.0

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.ego_x
        odom.pose.pose.position.y = self.ego_y
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = EGO_SPEED
        self.odom_pub.publish(odom)

        # opponent facet at (opp_x, 0): rasterize per BEAM over the angular span from the CURRENT
        # ego pose (contiguous indices — per-point sampling breaks the breakpoint clustering at
        # close range; Phase B sees no opponent at all)
        ranges = [float("inf")] * NUM_BEAMS
        opp_x = OPP_START_X + OPP_SPEED * t
        dx = opp_x - self.ego_x
        if t < T_OFFDOMAIN and dx > 0.15:
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

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "map"
        tf.child_frame_id = "laser"
        tf.transform.translation.x = self.ego_x
        tf.transform.translation.y = self.ego_y
        tf.transform.rotation.w = 1.0
        self.dyn_tf.sendTransform(tf)

    def on_ot(self, msg: OTWpntArray):
        t = time.time() - self.t0

        if not msg.wpnts:
            if T_OFFDOMAIN <= t <= T_OFFDOMAIN + PHASE_B_CLEAR_S:
                self.empties_b += 1
            return

        if t < T_OFFPATH:
            if msg.ot_line != "trail":
                self.commit_count += 1
            return

        if t >= T_OFFDOMAIN + PHASE_B_CLEAR_S:
            self.late_paths_b += 1        # any path after the clear window = stale-ego planning
            return
        if t >= T_OFFDOMAIN:
            return                        # inside the Phase B grace/clear window

        # Phase A reaction window: every path must be anchored near the NEW ego pose
        if t >= T_OFFPATH + REACT_DELAY:
            self.reaction_count += 1
            near = min(math.hypot(w.x_m - self.ego_x, w.y_m - self.ego_y) for w in msg.wpnts)
            if near > NEAR_EGO_M:
                self.stale_paths_a += 1

    def finish(self):
        print("\n================ OFF-PATH RECOVERY TEST RESULT ================", flush=True)
        committed = self.commit_count >= 3
        recovered_a = self.reaction_count >= 1 and self.stale_paths_a == 0
        cleared_b = self.empties_b >= 1
        silent_b = self.late_paths_b == 0
        print(f"  SETUP    overtake committed before the fault: {committed} "
              f"({self.commit_count} OTs)", flush=True)
        print(f"  PHASE A  paths re-anchored to the moved ego:  {recovered_a} "
              f"({self.reaction_count} near-ego, {self.stale_paths_a} stale)", flush=True)
        print(f"  PHASE B  cleared after ego went off-domain:   {cleared_b} "
              f"({self.empties_b} empty OTs)", flush=True)
        print(f"  PHASE B  silent afterwards (no stale-ego OT): {silent_b} "
              f"({self.late_paths_b} late paths)", flush=True)
        self.ok = committed and recovered_a and cleared_b and silent_b
        print(f"  ==> {'PASS' if self.ok else 'FAIL'}", flush=True)
        print("===============================================================\n", flush=True)
        self.done = True


def main():
    rclpy.init()
    node = Harness()
    print("[harness] running off-path recovery test...", flush=True)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if node.ok else 1)


if __name__ == "__main__":
    main()
