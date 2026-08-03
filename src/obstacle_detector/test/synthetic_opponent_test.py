#!/usr/bin/env python3
"""계층형 obstacle_detector 노드의 합성 통합 테스트이다.

직선 raceline, 전부 비어 있는 점유 지도, 원점의 자차 자세, map->laser 정적 TF를 주입한다.
/scan에는 (a) +x 방향으로 움직이는 상대 차량, (b) 경로 옆의 정지 물체, (c) 가림처럼 두
파편으로 보이는 하나의 정지 물체를 넣는다. 네 번째 정지 물체는 beam 하나를 비운 3-beam과
2-beam 파편으로 넣으며, 어느 파편도 단독으로 min_cluster_points에 도달하지 못한다.
이후 다음 계층 출력 계약을 검사한다.
  * /opp_obs(계층 3)가 이동 물체 하나를 0이 아닌 vs의 동적 장애물로 보고한다.
  * /static_obs(계층 2)가 차량 크기 정지 물체를 정적으로 유지한다.
  * 추적 전 군집 병합이 3+2 beam 파편을 검출 가능한 정지 물체 하나로 복원한다.
  * 한 프레임의 0.45 m 정적 물체 점프는 Euclidean hard gate를 통과하지만 Mahalanobis
    게이트에서 거부되어 확정 트랙이 그 위치로 튀지 않는다.
  * 먼 거리의 5점 물체가 가까운 고밀도 물체보다 큰 위치 불확실성을 유지한다.
  * 현재 보이는 모든 출력이 유한한 Cartesian AABB, 중심, 외접원 반지름을 가진다.
  * 계층 병합 물체가 두 Cartesian AABB의 합집합을 발행한다.
  * 예측만 남은 트랙은 Frenet 상태를 유지하되 오래된 Cartesian AABB를 발행하지 않는다.
  * 이동 물체가 먼저 /static_obs에 임시 정적으로 나타난 뒤 같은 ID로 /opp_obs로 이동하며
    다시 /static_obs로 돌아오지 않는다.
  * 정지 물체가 /opp_obs에 절대 섞이지 않는다.
  * 계층 병합이 두 파편을 모두 덮는 d 외곽을 가진 /static_obs 항목 하나로 합친다.

workspace를 source하고 `obstacle_detector_node`를 실행한 상태에서 다음과 같이 실행한다.
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
from visualization_msgs.msg import Marker, MarkerArray

from f110_msgs.msg import WpntArray, Wpnt, ObstacleArray


NUM_BEAMS = 1080
FOV = 4.7
ANGLE_MIN = -FOV / 2.0
ANGLE_INC = FOV / NUM_BEAMS
RANGE_MAX = 30.0

OPP_START_X = 4.0
OPP_SPEED = 1.0        # +x 방향 속도[m/s], 이 직선 경로에서는 +s와 같다.
STATIC_OBJ = (6.0, 1.2)   # 주행 영역 안의 차량 크기(0.5 x 0.5 m) 정지 장애물
# 하나의 정지 물체를 두 스캔 파편으로 표현한다. 면 중심은 d 방향 0.4 m, 모서리 간격은
# layer_merge_gap_d보다 작은 0.16 m이므로 계층 병합 후 /static_obs 항목 하나여야 한다.
FRAG_OBJ = (6.0, -1.25)
FRAG_HALF_SEP = 0.2       # 파편 중심 d = -1.05 / -1.45
PREMERGE_OBJ = (8.0, 0.65)  # 3 beam + 빈 beam + 2 beam으로, 각각은 단독 검출에 너무 작다.
STATIC_OUTLIER_SHIFT = 0.45  # assoc_gate=0.5 안이지만 수렴 후에는 통계적으로 비현실적인 이동
DURATION_S = 6.0
RATE_HZ = 20.0


def latched_qos():
    """정적 입력을 늦은 구독자도 받도록 transient-local QoS를 만든다."""
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def facet_points(cx, cy, half_width=0.12, n=10):
    """입력 (cx, cy)를 중심으로 짧은 수직 표면 점들을 만든다."""
    return [(cx, cy + (2.0 * i / (n - 1) - 1.0) * half_width) for i in range(n)]


def box_points(cx, cy, half=0.25, n=60):
    """센서를 향한 정사각형의 두 면을 L자 군집으로 만든다.

    half=0.25인 0.5 x 0.5 m 상자의 AABB 대각선은 약 0.707 m이므로 max_obs_size가 이보다
    클 때만 크기 게이트를 통과한다. n을 크게 두어 상자 범위의 LiDAR beam을 모두 채우고
    파편이 아닌 연속 군집이 되게 한다.
    """
    x0, y0 = cx - half, cy - half
    pts = []
    for i in range(n):
        t = i / (n - 1)
        pts.append((x0, y0 + 2.0 * half * t))   # 센서에 가까운 min-x 면
        pts.append((x0 + 2.0 * half * t, y0))   # 센서에 가까운 min-y 면
    return pts


def inject_premerge_fragments(ranges):
    """최근접점 간격이 0.12 m 미만인 임계점 수 이하 파편 두 개를 주입한다."""
    cx, cy = PREMERGE_OBJ
    center_angle = math.atan2(cy, cx)
    center_idx = int(round((center_angle - ANGLE_MIN) / ANGLE_INC))
    object_range = math.hypot(cx, cy)
    # 연속 3개 beam 뒤 하나를 의도적으로 비우고 연속 2개 beam을 둔다.
    # 약 8 m 거리에서 빈 구간 양쪽 최근접점은 약 0.07 m 떨어져 있다.
    for offset in (-3, -2, -1, 1, 2):
        idx = center_idx + offset
        if 0 <= idx < len(ranges):
            ranges[idx] = min(ranges[idx], object_range)


def valid_cartesian(ob):
    """현재 Cartesian AABB와 파생 중심·반지름의 출력 계약을 검사한다."""
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


def valid_frenet(ob):
    """최종 기준인 Frenet 형상 출력 계약을 검사한다."""
    values = (
        ob.s_start, ob.s_end, ob.s_center,
        ob.d_right, ob.d_left, ob.d_center, ob.size,
    )
    return (
        all(math.isfinite(value) for value in values)
        and ob.d_right <= ob.d_left
        and ob.size >= 0.0
    )


class Harness(Node):
    """합성 입력을 발행하고 검출기 출력을 누적 검증하는 ROS 2 노드이다."""

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
        self.sub_static_markers = self.create_subscription(
            MarkerArray, "/static_obs/markers", self.on_static_markers, 10)
        self.sub_opp_markers = self.create_subscription(
            MarkerArray, "/opp_obs/markers", self.on_opp_markers, 10)

        self.max_dyn_vs = 0.0
        self.saw_dynamic = False       # /opp_obs에 d≈0, |vs|>0.3인 상대 차량이 나타났는지
        self.saw_static = False        # /static_obs에 정지 장애물이 나타났는지
        self.static_size = 0.0
        self.static_wrongly_dynamic = False   # 정지 장애물이 /opp_obs에 잘못 들어갔는지
        self.saw_opp_provisional_static = False
        self.opp_static_after_dynamic = False
        self.provisional_opp_ids = set()
        self.dynamic_id_continuity = False
        self.opp_populated = False             # /opp_obs가 한 번이라도 물체를 실었는지
        self.saw_frag_merged = False   # 두 면을 덮는 파편 물체가 항목 하나로 보였는지
        self.max_frag_entries = 0      # 파편 영역에 동시에 나타난 /static_obs 항목의 최댓값
        self.saw_pretracking_merge = False  # cluster_merge로만 3+2 beam 파편이 살아났는지
        self.max_premerge_entries = 0
        self.static_position_var = None
        self.premerge_position_var = None
        self.static_baseline_s = None
        self.max_static_s_deviation = 0.0
        self.outlier_injected = False
        self.visible_cartesian_missing = False
        self.invalid_cartesian = False
        self.invalid_frenet = False
        self.stale_cartesian_leak = False
        self.saw_predicted_without_cartesian = False
        self.saw_merged_cartesian_union = False
        self.saw_static_frenet_marker = False
        self.saw_opp_frenet_marker = False

        self.done = False
        self.ok = False
        self.publish_static_inputs()
        self.t0 = time.time()
        self.timer = self.create_timer(1.0 / RATE_HZ, self.tick)

    # 정적·latched 입력
    def publish_static_inputs(self):
        """직선 raceline, 빈 지도, 항등 TF를 한 번 발행한다."""

        # +x 방향, 0.1 m 간격, 좌우 2 m 주행 영역의 직선 raceline
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

        # 전체 합성 장면을 덮는 완전히 빈 점유 지도
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

        # 원점 자차를 나타내는 map -> laser 항등 정적 TF
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "laser"
        tf.transform.rotation.w = 1.0
        self.static_tf.sendTransform(tf)

    # 주기적 scan·odometry 입력
    def tick(self):
        """상대 차량을 이동시키며 합성 scan과 자차 odometry를 발행한다."""
        t = time.time() - self.t0
        if t > DURATION_S:
            self.finish()
            return
        stamp = self.get_clock().now().to_msg()

        # 자차는 map 원점에서 정지해 있다고 가정한다.
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        # 이 합성 장면에서는 laser 좌표와 map 좌표가 같으므로 표면점을 바로 만든다.
        opp_x = OPP_START_X + OPP_SPEED * t
        static_x = STATIC_OBJ[0]
        if not self.outlier_injected and t >= 3.0:
            # 정확히 한 프레임만 0.45 m 이동시킨다. 물리 hard gate는 통과하지만 수렴된
            # 공분산을 사용한 Mahalanobis 게이트는 이 측정을 거부해야 한다.
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

    # 출력 검사
    def on_static(self, msg: ObstacleArray):
        """계층 2의 형상·분류·병합·불확실성 계약을 검사한다."""

        # 계층 2의 모든 항목은 정적 장애물이어야 한다.
        frag_entries = 0
        premerge_entries = 0
        for ob in msg.obstacles:
            if not valid_frenet(ob):
                self.invalid_frenet = True
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
                # 보이는 L자 면의 중심은 실제 상자 중심에서 치우쳐 있다. 알 수 없는 물리 중심이
                # 아니라 outlier 주입 전에 확정된 트랙 위치를 기준으로 한 프레임 점프를 측정한다.
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
                    self.saw_frag_merged = True  # d 외곽이 두 파편 면을 모두 덮는다.
                if ob.has_cartesian and ob.y_max - ob.y_min >= 0.50:
                    self.saw_merged_cartesian_union = True
            elif near_premerge:
                premerge_entries += 1
                self.saw_pretracking_merge = True
                self.premerge_position_var = ob.s_var + ob.d_var
            elif abs(ob.d_center) < 0.5:
                # 모든 검출 물체는 세 번째 hit에서 의도적으로 임시 정적으로 처음 발행된다.
                self.saw_opp_provisional_static = True
                self.provisional_opp_ids.add(ob.id)
                if self.saw_dynamic:
                    self.opp_static_after_dynamic = True
        self.max_frag_entries = max(self.max_frag_entries, frag_entries)
        self.max_premerge_entries = max(self.max_premerge_entries, premerge_entries)

    def on_opp(self, msg: ObstacleArray):
        """계층 3에 동적 상대만 있고 ID가 이어지는지 검사한다."""

        # 계층 3에는 동적 상대 차량이 최대 하나만 있어야 한다.
        if msg.obstacles:
            self.opp_populated = True
        for ob in msg.obstacles:
            if not valid_frenet(ob):
                self.invalid_frenet = True
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
                # 정지 물체는 상대 차량 계층에 절대 들어가면 안 된다.
                self.static_wrongly_dynamic = True
            elif not ob.is_static and abs(ob.vs) > 0.3:
                self.saw_dynamic = True
                self.max_dyn_vs = max(self.max_dyn_vs, abs(ob.vs))
                if ob.id in self.provisional_opp_ids:
                    self.dynamic_id_continuity = True

    @staticmethod
    def valid_frenet_marker(marker):
        """Frenet 외곽에서 만든 map 좌표 LINE_STRIP 하나를 검사한다."""
        return (
            marker.action == Marker.ADD
            and marker.type == Marker.LINE_STRIP
            and marker.header.frame_id == "map"
            and len(marker.points) >= 5
            and all(
                math.isfinite(point.x)
                and math.isfinite(point.y)
                and math.isfinite(point.z)
                for point in marker.points
            )
        )

    def on_static_markers(self, msg: MarkerArray):
        """RViz용 /static_obs 복제본에 유효한 Frenet 경계가 있는지 확인한다."""
        self.saw_static_frenet_marker = (
            self.saw_static_frenet_marker
            or any(self.valid_frenet_marker(marker) for marker in msg.markers)
        )

    def on_opp_markers(self, msg: MarkerArray):
        """RViz용 /opp_obs 복제본에 유효한 Frenet 경계가 있는지 확인한다."""
        self.saw_opp_frenet_marker = (
            self.saw_opp_frenet_marker
            or any(self.valid_frenet_marker(marker) for marker in msg.markers)
        )

    def finish(self):
        """누적된 모든 검증 조건을 평가하고 통합 테스트 결과를 출력한다."""
        print("\n================ SYNTHETIC OBSTACLE TEST RESULT ================", flush=True)
        print(f"  dynamic opponent on /opp_obs (is_static=False, |vs|>0.3): {self.saw_dynamic}", flush=True)
        print(f"  max estimated |vs| of opponent [m/s]:                     {self.max_dyn_vs:.2f}  (truth {OPP_SPEED:.2f})", flush=True)
        print(f"  car-sized static obstacle on /static_obs:                 {self.saw_static}", flush=True)
        print(f"  static obstacle box size (AABB diag) [m]:                 {self.static_size:.2f}  (0.5x0.5 -> ~0.71)", flush=True)
        print(f"  static obstacle NEVER leaked into /opp_obs:               {not self.static_wrongly_dynamic}", flush=True)
        print(f"  opponent first appeared as provisional /static_obs:       "
              f"{self.saw_opp_provisional_static}", flush=True)
        print(f"  provisional -> dynamic kept the same ID:                  "
              f"{self.dynamic_id_continuity}", flush=True)
        print(f"  dynamic opponent NEVER returned to /static_obs:           "
              f"{not self.opp_static_after_dynamic}", flush=True)
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
        print(f"  every published obstacle has valid Frenet AABB bounds:     "
              f"{not self.invalid_frenet}", flush=True)
        print(f"  /static_obs/markers mirrors a Frenet boundary:             "
              f"{self.saw_static_frenet_marker}", flush=True)
        print(f"  /opp_obs/markers mirrors a Frenet boundary:                "
              f"{self.saw_opp_frenet_marker}", flush=True)
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
                   and self.saw_opp_provisional_static and self.dynamic_id_continuity
                   and not self.opp_static_after_dynamic and self.static_size > 0.5
                   and self.opp_populated and abs(self.max_dyn_vs - OPP_SPEED) < 0.6
                   and self.outlier_injected and self.static_baseline_s is not None
                   and self.max_static_s_deviation < 0.10
                   and self.static_position_var is not None
                   and self.premerge_position_var is not None
                   and self.premerge_position_var > self.static_position_var
                   and not self.visible_cartesian_missing and not self.invalid_cartesian
                   and not self.invalid_frenet
                   and self.saw_static_frenet_marker
                   and self.saw_opp_frenet_marker
                   and self.saw_merged_cartesian_union
                   and self.saw_predicted_without_cartesian
                   and not self.stale_cartesian_leak
                   and self.saw_pretracking_merge and self.max_premerge_entries == 1
                   and self.saw_frag_merged and self.max_frag_entries == 1)
        print(f"  ==> {'PASS' if self.ok else 'FAIL'}", flush=True)
        print("================================================================\n", flush=True)
        self.done = True


def main():
    """검증 노드를 돌려 완료 후 성공 여부를 프로세스 종료 코드로 반환한다."""
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
