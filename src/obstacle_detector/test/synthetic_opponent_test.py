#!/usr/bin/env python3
"""Synthetic integration test for the obstacle_detector node (Layer 1/2/3).

Injects a fake straight raceline, a free occupancy map, an ego pose at the origin, a static
TF map->laser, and a /scan containing (a) a MOVING opponent car (along +x), (b) a STATIC
object beside the line, and (c) ONE static object seen as TWO scan fragments (occlusion-style
split). It also injects a fourth static object as 3-beam and 2-beam fragments separated by one
missing ray; neither fragment reaches min_cluster_points alone. Then it checks the layered outputs:
  * /opp_obs (Layer 3)    reports the moving object as a single dynamic obstacle with non-zero vs;
  * /static_obs (Layer 2) reports the car-sized stationary object and keeps it static;
  * pre-tracking cluster merge restores the 3+2 beam fragments as one detectable static object;
  * a one-frame 0.45 m static-object jump passes the Euclidean hard gate but is rejected by the
    Mahalanobis gate, so the confirmed track does not jump with it;
  * the farther five-point object keeps greater position uncertainty than the nearer dense object;
  * every visible output has a finite Cartesian AABB, AABB centre, and enclosing-circle radius;
  * the layer-merged fragmented object publishes the union of both Cartesian AABBs;
  * a predicted-only track keeps its Frenet state but does not publish its stale Cartesian AABB;
  * the stationary object NEVER leaks into /opp_obs (and the opponent never leaks into /static_obs);
  * the per-layer merge folds the fragmented object into ONE /static_obs entry whose d-envelope
    covers BOTH fragments (never two simultaneous entries).

Run (with the workspace sourced), while `obstacle_detector_node` is running:
    ros2 run obstacle_detector obstacle_detector_node --ros-args \
        --params-file src/obstacle_detector/config/obstacle_detector.yaml
    python3 src/obstacle_detector/test/synthetic_opponent_test.py
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
from tf2_ros import StaticTransformBroadcaster

from f110_msgs.msg import WpntArray, Wpnt, ObstacleArray


NUM_BEAMS = 1080
FOV = 4.7
ANGLE_MIN = -FOV / 2.0
ANGLE_INC = FOV / NUM_BEAMS
RANGE_MAX = 30.0

OPP_START_X = 4.0
OPP_SPEED = 1.0        # m/s along +x (== +s on this straight line)
STATIC_OBJ = (6.0, 1.2)   # a car-sized (0.5x0.5 m) STATIONARY obstacle inside the corridor
# ONE stationary object rendered as TWO scan fragments (facet centres 0.4 m apart in d, edge gap
# 0.16 m < layer_merge_gap_d): the per-layer merge must publish it as a SINGLE /static_obs entry.
FRAG_OBJ = (6.0, -1.25)
FRAG_HALF_SEP = 0.2       # fragment centres at d = -1.05 / -1.45
PREMERGE_OBJ = (8.0, 0.65)  # emitted as 3 beams + gap + 2 beams; each is too small alone
STATIC_OUTLIER_SHIFT = 0.45  # inside assoc_gate=0.5, but statistically implausible once converged
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


def inject_premerge_fragments(ranges):
    """Inject two sub-threshold fragments whose closest points are less than 0.12 m apart."""
    cx, cy = PREMERGE_OBJ
    center_angle = math.atan2(cy, cx)
    center_idx = int(round((center_angle - ANGLE_MIN) / ANGLE_INC))
    object_range = math.hypot(cx, cy)
    # Three contiguous beams, one deliberately missing beam, then two contiguous beams.
    # At about 8 m, the nearest cross-gap point pair is about 0.07 m apart.
    for offset in (-3, -2, -1, 1, 2):
        idx = center_idx + offset
        if 0 <= idx < len(ranges):
            ranges[idx] = min(ranges[idx], object_range)


def valid_cartesian(ob):
    values = (
        ob.x_min, ob.x_max, ob.y_min, ob.y_max,
        ob.x_center, ob.y_center, ob.radius,
    )
    if not ob.has_cartesian or not all(math.isfinite(value) for value in values):
        return False
    if ob.x_min > ob.x_max or ob.y_min > ob.y_max or ob.radius <= 0.0:
        return False
    expected_x = 0.5 * (ob.x_min + ob.x_max)
    expected_y = 0.5 * (ob.y_min + ob.y_max)
    expected_radius = 0.5 * math.hypot(ob.x_max - ob.x_min, ob.y_max - ob.y_min)
    return (
        abs(ob.x_center - expected_x) < 1e-6
        and abs(ob.y_center - expected_y) < 1e-6
        and abs(ob.radius - expected_radius) < 1e-6
    )


class Harness(Node):
    def __init__(self):
        super().__init__("synthetic_opponent_test")
        self.wpnt_pub = self.create_publisher(WpntArray, "/global_waypoints", latched_qos())
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", latched_qos())
        self.odom_pub = self.create_publisher(Odometry, "/pf/pose/odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.static_tf = StaticTransformBroadcaster(self)

        self.sub_static = self.create_subscription(
            ObstacleArray, "/static_obs", self.on_static, 10)
        self.sub_opp = self.create_subscription(
            ObstacleArray, "/opp_obs", self.on_opp, 10)

        self.max_dyn_vs = 0.0
        self.saw_dynamic = False       # opponent present in /opp_obs (near d=0, |vs|>0.3)
        self.saw_static = False        # stationary obstacle present in /static_obs
        self.static_size = 0.0
        self.static_wrongly_dynamic = False   # stationary obstacle leaked into /opp_obs
        self.opp_wrongly_static = False        # opponent leaked into /static_obs
        self.opp_populated = False             # /opp_obs carried at least one obstacle
        self.saw_frag_merged = False   # fragmented object seen as ONE entry covering both facets
        self.max_frag_entries = 0      # max SIMULTANEOUS /static_obs entries in the fragment region
        self.saw_pretracking_merge = False  # 3+2 beam fragments survived only by cluster_merge
        self.max_premerge_entries = 0
        self.static_position_var = None
        self.premerge_position_var = None
        self.static_baseline_s = None
        self.max_static_s_deviation = 0.0
        self.outlier_injected = False
        self.visible_cartesian_missing = False
        self.invalid_cartesian = False
        self.stale_cartesian_leak = False
        self.saw_predicted_without_cartesian = False
        self.saw_merged_cartesian_union = False

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
        static_x = STATIC_OBJ[0]
        if not self.outlier_injected and t >= 3.0:
            # Exactly one frame: the physical hard gate accepts 0.45 m, but the converged
            # covariance should make the Mahalanobis gate reject this measurement.
            static_x += STATIC_OUTLIER_SHIFT
            self.outlier_injected = True
        pts = (facet_points(opp_x, 0.0) + box_points(static_x, STATIC_OBJ[1])
               + facet_points(FRAG_OBJ[0], FRAG_OBJ[1] + FRAG_HALF_SEP)
               + facet_points(FRAG_OBJ[0], FRAG_OBJ[1] - FRAG_HALF_SEP))

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
        inject_premerge_fragments(ranges)

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
    def on_static(self, msg: ObstacleArray):
        # Layer 2: everything here must be a static obstacle.
        frag_entries = 0
        premerge_entries = 0
        for ob in msg.obstacles:
            if ob.is_visible:
                if not ob.has_cartesian:
                    self.visible_cartesian_missing = True
                elif not valid_cartesian(ob):
                    self.invalid_cartesian = True
            elif ob.has_cartesian:
                self.stale_cartesian_leak = True
            else:
                self.saw_predicted_without_cartesian = True

            near_static = abs(ob.d_center - STATIC_OBJ[1]) < 0.5
            near_frag = abs(ob.d_center - FRAG_OBJ[1]) < 0.6
            near_premerge = (
                abs(ob.s_center - PREMERGE_OBJ[0]) < 0.5
                and abs(ob.d_center - PREMERGE_OBJ[1]) < 0.25
            )
            if near_static:
                self.saw_static = True
                self.static_size = max(self.static_size, ob.size)
                self.static_position_var = ob.s_var + ob.d_var
                # The visible L-shaped faces have a centroid offset from the geometric box centre.
                # Measure the injected one-frame jump relative to the confirmed pre-outlier track,
                # not relative to the unavailable physical centre.
                if self.static_baseline_s is None:
                    self.static_baseline_s = ob.s_center
                else:
                    self.max_static_s_deviation = max(
                        self.max_static_s_deviation,
                        abs(ob.s_center - self.static_baseline_s),
                    )
            elif near_frag:
                frag_entries += 1
                if ob.d_left - ob.d_right >= 0.45:
                    self.saw_frag_merged = True  # the d-envelope spans BOTH facets
                if ob.has_cartesian and ob.y_max - ob.y_min >= 0.50:
                    self.saw_merged_cartesian_union = True
            elif near_premerge:
                premerge_entries += 1
                self.saw_pretracking_merge = True
                self.premerge_position_var = ob.s_var + ob.d_var
            elif abs(ob.d_center) < 0.5:
                # the opponent's lane (near d=0) must NOT appear in the static layer
                self.opp_wrongly_static = True
        self.max_frag_entries = max(self.max_frag_entries, frag_entries)
        self.max_premerge_entries = max(self.max_premerge_entries, premerge_entries)

    def on_opp(self, msg: ObstacleArray):
        # Layer 3: at most one obstacle, the dynamic opponent.
        if msg.obstacles:
            self.opp_populated = True
        for ob in msg.obstacles:
            if ob.is_visible:
                if not ob.has_cartesian:
                    self.visible_cartesian_missing = True
                elif not valid_cartesian(ob):
                    self.invalid_cartesian = True
            elif ob.has_cartesian:
                self.stale_cartesian_leak = True
            else:
                self.saw_predicted_without_cartesian = True

            near_static = abs(ob.d_center - STATIC_OBJ[1]) < 0.5
            near_frag = abs(ob.d_center - FRAG_OBJ[1]) < 0.6
            near_premerge = (
                abs(ob.s_center - PREMERGE_OBJ[0]) < 0.5
                and abs(ob.d_center - PREMERGE_OBJ[1]) < 0.25
            )
            if near_static or near_frag or near_premerge:
                # a stationary object must NOT leak into the opponent layer
                self.static_wrongly_dynamic = True
            elif not ob.is_static and abs(ob.vs) > 0.3:
                self.saw_dynamic = True
                self.max_dyn_vs = max(self.max_dyn_vs, abs(ob.vs))

    def finish(self):
        print("\n================ SYNTHETIC OBSTACLE TEST RESULT ================", flush=True)
        print(f"  dynamic opponent on /opp_obs (is_static=False, |vs|>0.3): {self.saw_dynamic}", flush=True)
        print(f"  max estimated |vs| of opponent [m/s]:                     {self.max_dyn_vs:.2f}  (truth {OPP_SPEED:.2f})", flush=True)
        print(f"  car-sized static obstacle on /static_obs:                 {self.saw_static}", flush=True)
        print(f"  static obstacle box size (AABB diag) [m]:                 {self.static_size:.2f}  (0.5x0.5 -> ~0.71)", flush=True)
        print(f"  static obstacle NEVER leaked into /opp_obs:               {not self.static_wrongly_dynamic}", flush=True)
        print(f"  opponent NEVER leaked into /static_obs:                   {not self.opp_wrongly_static}", flush=True)
        print(f"  /opp_obs populated at least once:                         {self.opp_populated}", flush=True)
        print(f"  one-frame 0.45 m outlier injected:                        {self.outlier_injected}", flush=True)
        print(f"  max confirmed static-track s deviation [m]:               "
              f"{self.max_static_s_deviation:.3f} "
              "(Mahalanobis target < 0.10)", flush=True)
        print(f"  dense-near position variance (s_var+d_var):               "
              f"{self.static_position_var}", flush=True)
        print(f"  sparse-far position variance (s_var+d_var):               "
              f"{self.premerge_position_var}", flush=True)
        print(f"  every visible obstacle has valid Cartesian AABB/radius:    "
              f"{not self.visible_cartesian_missing and not self.invalid_cartesian}", flush=True)
        print(f"  layer-merged object publishes Cartesian AABB union:        "
              f"{self.saw_merged_cartesian_union}", flush=True)
        print(f"  predicted-only obstacle observed with has_cartesian=false: "
              f"{self.saw_predicted_without_cartesian}", flush=True)
        print(f"  stale Cartesian AABB NEVER leaked from invisible track:   "
              f"{not self.stale_cartesian_leak}", flush=True)
        print(f"  pre-tracking 3+2 beam fragments restored as ONE track:    {self.saw_pretracking_merge} "
              f"(max simultaneous entries: {self.max_premerge_entries})", flush=True)
        print(f"  fragmented object merged into ONE /static_obs entry:      {self.saw_frag_merged} "
              f"(max simultaneous entries in region: {self.max_frag_entries})", flush=True)
        self.ok = (self.saw_dynamic and self.saw_static and not self.static_wrongly_dynamic
                   and not self.opp_wrongly_static and self.static_size > 0.5
                   and self.opp_populated and abs(self.max_dyn_vs - OPP_SPEED) < 0.6
                   and self.outlier_injected and self.static_baseline_s is not None
                   and self.max_static_s_deviation < 0.10
                   and self.static_position_var is not None
                   and self.premerge_position_var is not None
                   and self.premerge_position_var > self.static_position_var
                   and not self.visible_cartesian_missing and not self.invalid_cartesian
                   and self.saw_merged_cartesian_union
                   and self.saw_predicted_without_cartesian
                   and not self.stale_cartesian_leak
                   and self.saw_pretracking_merge and self.max_premerge_entries == 1
                   and self.saw_frag_merged and self.max_frag_entries == 1)
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
