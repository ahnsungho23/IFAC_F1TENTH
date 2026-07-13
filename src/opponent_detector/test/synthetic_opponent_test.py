#!/usr/bin/env python3
"""Synthetic integration test for the opponent_detector node.

Injects a fake straight raceline, a free occupancy map, an ego pose at the origin, a static
TF map->laser, and a /scan containing (a) a MOVING opponent car (along +x) and (b) a STATIC
object beside the line. Then it checks that the detector reports the moving object as a dynamic
obstacle with non-zero vs, keeps the static object static, and populates /proj_opponent_trajectory.

Run (with the workspace sourced), while `opponent_detector_node` is running:
    python3 src/opponent_detector/test/synthetic_opponent_test.py
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster

from f110_msgs.msg import WpntArray, Wpnt, ObstacleArray, ProjOppTraj


NUM_BEAMS = 1080
FOV = 4.7
ANGLE_MIN = -FOV / 2.0
ANGLE_INC = FOV / NUM_BEAMS
RANGE_MAX = 30.0

OPP_START_X = 4.0
OPP_SPEED = 1.0        # m/s along +x (== +s on this straight line)
STATIC_OBJ = (6.0, 1.2)   # a car-sized (0.5x0.5 m) STATIONARY obstacle inside the corridor
DURATION_S = 6.0
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


def box_points(cx, cy, half=0.25, n=60):
    """The two sensor-facing edges of a (2*half)x(2*half) box -> an L-shaped cluster.

    For a 0.5x0.5 m box (half=0.25) the AABB diagonal is ~0.707 m, so it only survives the size
    gate once max_obs_size exceeds that (the point of the static-obstacle change). n is high so the
    surface points fill every LiDAR beam across the box (a contiguous cluster, not fragments).
    """
    x0, y0 = cx - half, cy - half
    pts = []
    for i in range(n):
        t = i / (n - 1)
        pts.append((x0, y0 + 2.0 * half * t))   # near (min-x) face
        pts.append((x0 + 2.0 * half * t, y0))   # near (min-y) face
    return pts


class Harness(Node):
    def __init__(self):
        super().__init__("synthetic_opponent_test")
        self.wpnt_pub = self.create_publisher(WpntArray, "/global_waypoints", latched_qos())
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", latched_qos())
        self.odom_pub = self.create_publisher(Odometry, "/pf/pose/odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.static_tf = StaticTransformBroadcaster(self)

        self.sub_obs = self.create_subscription(
            ObstacleArray, "/perception/obstacles", self.on_obstacles, 10)
        self.sub_proj = self.create_subscription(
            ProjOppTraj, "/proj_opponent_trajectory", self.on_proj, 10)

        self.max_dyn_vs = 0.0
        self.saw_dynamic = False
        self.saw_static = False
        self.static_size = 0.0
        self.static_wrongly_dynamic = False
        self.proj_points = 0
        self.proj_on_traj = False

        self.done = False
        self.ok = False
        self.publish_static_inputs()
        self.t0 = time.time()
        self.timer = self.create_timer(1.0 / RATE_HZ, self.tick)

    # ---- static / latched inputs ----
    def publish_static_inputs(self):
        # straight raceline along +x, 0.1 m spacing, corridor +-2 m
        arr = WpntArray()
        arr.header.frame_id = "map"
        for i in range(201):
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

        # all-free occupancy grid covering the scene
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.resolution = 0.05
        grid.info.width = 600     # 30 m
        grid.info.height = 200    # 10 m
        grid.info.origin.position.x = -5.0
        grid.info.origin.position.y = -5.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (grid.info.width * grid.info.height)
        self.map_pub.publish(grid)

        # static TF map -> laser (identity; ego at origin)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "laser"
        tf.transform.rotation.w = 1.0
        self.static_tf.sendTransform(tf)

    # ---- periodic scan + odom ----
    def tick(self):
        t = time.time() - self.t0
        if t > DURATION_S:
            self.finish()
            return
        stamp = self.get_clock().now().to_msg()

        # ego odom at origin
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        # build scan surface points (laser frame == map frame here)
        opp_x = OPP_START_X + OPP_SPEED * t
        pts = facet_points(opp_x, 0.0) + box_points(STATIC_OBJ[0], STATIC_OBJ[1])

        ranges = [float("inf")] * NUM_BEAMS
        for (px, py) in pts:
            if px <= 0.0:
                continue
            ang = math.atan2(py, px)
            if ang < ANGLE_MIN or ang > ANGLE_MIN + FOV:
                continue
            idx = int(round((ang - ANGLE_MIN) / ANGLE_INC))
            if 0 <= idx < NUM_BEAMS:
                r = math.hypot(px, py)
                if r < ranges[idx]:
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

    # ---- output checks ----
    def on_obstacles(self, msg: ObstacleArray):
        for ob in msg.obstacles:
            near_static = abs(ob.d_center - STATIC_OBJ[1]) < 0.5   # the stationary-obstacle lane
            if near_static:
                # the car-sized stationary obstacle: must be detected and stay static
                self.static_size = max(self.static_size, ob.size)
                if ob.is_static:
                    self.saw_static = True
                else:
                    self.static_wrongly_dynamic = True
            elif not ob.is_static and abs(ob.vs) > 0.3:
                # the moving opponent (near d=0)
                self.saw_dynamic = True
                self.max_dyn_vs = max(self.max_dyn_vs, abs(ob.vs))

    def on_proj(self, msg: ProjOppTraj):
        self.proj_points = max(self.proj_points, int(msg.nrofpoints))
        if msg.opp_is_on_trajectory:
            self.proj_on_traj = True

    def finish(self):
        print("\n================ SYNTHETIC OPPONENT TEST RESULT ================", flush=True)
        print(f"  dynamic opponent detected (is_static=False, |vs|>0.3): {self.saw_dynamic}", flush=True)
        print(f"  max estimated |vs| of opponent [m/s]:                  {self.max_dyn_vs:.2f}  (truth {OPP_SPEED:.2f})", flush=True)
        print(f"  car-sized static obstacle detected & kept static:      {self.saw_static}", flush=True)
        print(f"  static obstacle box size (AABB diag) [m]:              {self.static_size:.2f}  (0.5x0.5 -> ~0.71)", flush=True)
        print(f"  static obstacle NEVER misclassified dynamic:           {not self.static_wrongly_dynamic}", flush=True)
        print(f"  /proj_opponent_trajectory populated (points):          {self.proj_points}", flush=True)
        print(f"  opp_is_on_trajectory flag seen:                        {self.proj_on_traj}", flush=True)
        self.ok = (self.saw_dynamic and self.saw_static and not self.static_wrongly_dynamic
                   and self.static_size > 0.5 and self.proj_points > 0 and self.proj_on_traj
                   and abs(self.max_dyn_vs - OPP_SPEED) < 0.6)
        print(f"  ==> {'PASS' if self.ok else 'FAIL'}", flush=True)
        print("================================================================\n", flush=True)
        self.done = True


def main():
    rclpy.init()
    node = Harness()
    print("[harness] running...", flush=True)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if node.ok else 1)


if __name__ == "__main__":
    main()
