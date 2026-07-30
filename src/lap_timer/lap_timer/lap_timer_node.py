#!/usr/bin/env python3
import math

from builtin_interfaces.msg import Duration
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rviz_2d_overlay_msgs.msg import OverlayText
from std_msgs.msg import ColorRGBA, Float32, Float64
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker

# 선택적 import (파라미터에 따라 사용)
try:
    from ackermann_msgs.msg import AckermannDriveStamped
except Exception:
    AckermannDriveStamped = None
try:
    from geometry_msgs.msg import AccelStamped
except Exception:
    AccelStamped = None


class FrenetLapTimer(Node):
    def __init__(self):
        super().__init__('lap_timer')

        # ---- Parameters ----
        self.declare_parameter('odom_topic', '/car_state/frenet/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('drive_msg_type', 'ackermann')  # 'ackermann' or 'accel'
        self.declare_parameter('start_window', 0.5)      # [m]
        self.declare_parameter('wrap_threshold', 10.0)    # [m]
        self.declare_parameter('min_lap_time', 3.0)       # [s]
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('use_position_x_as_s', True)
        self.declare_parameter('rviz_text', True)
        self.declare_parameter('exact_zero_mode', False)
        self.declare_parameter('exact_zero_eps', 0.05)

        # 실시간 HUD용
        self.declare_parameter('hud_rate', 10.0)          # [Hz]
        self.declare_parameter('speed_unit', 'mps')       # 'mps' or 'kmh'
        self.declare_parameter('steer_in_deg', False)     # True면 deg로 표시

        gp = self.get_parameter
        self.odom_topic = gp('odom_topic').value
        self.drive_topic = gp('drive_topic').value
        self.drive_msg_type = gp('drive_msg_type').value.lower()
        self.start_window = float(gp('start_window').value)
        self.wrap_threshold = float(gp('wrap_threshold').value)
        self.min_lap_time = float(gp('min_lap_time').value)
        self.frame_id = gp('frame_id').value
        self.use_position_x_as_s = bool(gp('use_position_x_as_s').value)
        self.show_rviz_text = bool(gp('rviz_text').value)
        self.exact_zero_mode = bool(gp('exact_zero_mode').value)
        self.exact_zero_eps = float(gp('exact_zero_eps').value)

        self.hud_rate = float(gp('hud_rate').value)
        self.speed_unit = (gp('speed_unit').value or 'mps').lower()
        self.steer_in_deg = bool(gp('steer_in_deg').value)

        # ---- Pub/Sub/Service ----
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.cb_odom, 50)

        # /drive 구독: 타입 선택
        self.sub_drive = None
        if self.drive_msg_type == 'ackermann':
            if AckermannDriveStamped is None:
                self.get_logger().warn(
                    'ackermann_msgs not found. Install or set drive_msg_type:=accel'
                )
            else:
                self.sub_drive = self.create_subscription(
                    AckermannDriveStamped, self.drive_topic, self.cb_drive_ack, 20
                )
        elif self.drive_msg_type in ('accel', 'accelstamped'):
            if AccelStamped is None:
                self.get_logger().warn('geometry_msgs AccelStamped not importable.')
            else:
                self.sub_drive = self.create_subscription(
                    AccelStamped, self.drive_topic, self.cb_drive_accel, 20
                )
        else:
            self.get_logger().warn(
                f'Unknown drive_msg_type: {self.drive_msg_type}. '
                'Supported: ackermann | accel'
            )

        self.pub_lap = self.create_publisher(Float64, 'lap_time', 10)
        self.pub_best = self.create_publisher(Float64, 'best_lap_time', 10)
        self.pub_marker = (
            self.create_publisher(Marker, 'lap_time_text', 10)
            if self.show_rviz_text
            else None
        )
        self.pub_overlay = self.create_publisher(OverlayText, 'lap_hud', 10)

        self.pub_speed = self.create_publisher(Float32, 'speed', 10)
        self.pub_steer = self.create_publisher(Float32, 'steer', 10)

        self.srv_reset_best = self.create_service(Empty, 'reset_best_lap', self.on_reset_best)

        # ---- State ----
        self.prev_s = None
        self.lap_start_time = self.get_clock().now()
        self.lap_count = 0
        self.best_lap = math.inf
        self.last_lap_time = float('nan')

        # 최신 구동 정보
        self.last_speed = float('nan')  # m/s (ackermann) / accel 모드에선 NaN
        self.last_steer = float('nan')  # rad (ackermann) / accel 모드엔 NaN

        # HUD 주기 퍼블리시 타이머
        period = 1.0 / max(self.hud_rate, 0.1)
        self.hud_timer = self.create_timer(period, self.tick_hud)

        self.get_logger().info(
            f'LapTimer running: odom={self.odom_topic}, drive={self.drive_topic} '
            f'({self.drive_msg_type}), '
            f'mode={"exact-zero" if self.exact_zero_mode else "wrap"}, '
            f'hud_rate={self.hud_rate} Hz, speed_unit={self.speed_unit}, '
            f'steer_in_deg={self.steer_in_deg}'
        )

    # ---------- helpers ----------
    def extract_s(self, odom: Odometry) -> float:
        if self.use_position_x_as_s:
            return float(odom.pose.pose.position.x)
        return float(odom.twist.twist.linear.x)

    def fmt_speed(self, v_mps: float) -> str:
        if not math.isfinite(v_mps):
            return '-'
        if self.speed_unit == 'kmh':
            return f'{v_mps:.2f} km/h'
        return f'{v_mps:.2f} m/s'

    def fmt_steer(self, steer_rad: float) -> str:
        if not math.isfinite(steer_rad):
            return '-'
        if self.steer_in_deg:
            return f'{math.degrees(steer_rad):.2f} deg'
        return f'{steer_rad:.3f} rad'

    # ---------- services ----------
    def on_reset_best(self, req, res):
        self.best_lap = math.inf
        self.get_logger().info('Best lap reset.')
        msg = Float64()
        msg.data = float('nan')
        self.pub_best.publish(msg)
        return res

    # ---------- callbacks ----------
    def cb_drive_ack(self, msg):
        # ackermann_msgs/AckermannDriveStamped
        self.last_speed = float(msg.drive.speed)
        self.last_steer = float(msg.drive.steering_angle)
        # 편의 토픽
        self.pub_speed.publish(Float32(data=self.last_speed))
        self.pub_steer.publish(Float32(data=self.last_steer))

    def cb_drive_accel(self, msg):
        # geometry_msgs/AccelStamped (가속도만 존재) → 여기선 HUD에 직접 쓰지 않음
        self.last_speed = float('nan')
        self.last_steer = float('nan')

    def cb_odom(self, odom: Odometry):
        now = self.get_clock().now()
        s = self.extract_s(odom)

        if self.prev_s is None:
            self.prev_s = s
            return

        elapsed = (now - self.lap_start_time).nanoseconds * 1e-9
        triggered = False

        if self.exact_zero_mode:
            crossed_exact_zero = (
                abs(s) <= self.exact_zero_eps
                and abs(self.prev_s) > self.exact_zero_eps
                and elapsed >= self.min_lap_time
            )
            if crossed_exact_zero:
                triggered = True
        else:
            crossed_start = (self.prev_s >= self.wrap_threshold and s <= self.start_window)
            big_drop = (self.prev_s - s) > (self.wrap_threshold * 0.5)
            if (crossed_start or big_drop) and elapsed >= self.min_lap_time:
                triggered = True

        if triggered:
            self.lap_count += 1
            self.last_lap_time = elapsed
            self.publish_lap(self.last_lap_time)
            self.lap_start_time = now

        self.prev_s = s

    # ---------- periodic HUD ----------
    def tick_hud(self):
        """랩 진행 중에도 현재 상태와 랩 기록을 HUD로 표시합니다."""
        hud = OverlayText()
        hud.action = getattr(OverlayText, 'ACTION_ADD', 0)

        best_txt = f'{self.best_lap:.3f} s' if math.isfinite(self.best_lap) else '-'
        last_txt = (
            f'{self.last_lap_time:.3f} s'
            if math.isfinite(self.last_lap_time)
            else '-'
        )

        sp_txt = self.fmt_speed(self.last_speed)
        st_txt = self.fmt_steer(self.last_steer)

        hud.text = (
            f'LAP #{self.lap_count}  last: {last_txt}\n'
            f'BEST: {best_txt}\n'
            f'SPEED: {sp_txt}   STEER: {st_txt}'
        )
        hud.text_size = 20.0
        hud.line_width = 2
        hud.font = 'DejaVu Sans'
        hud.fg_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        hud.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0)  # 배경 투명 (RViz에서 바꿀 수 있음)

        self.pub_overlay.publish(hud)

    # ---------- publish on lap event ----------
    def publish_lap(self, lap_time: float):
        # lap_time
        self.pub_lap.publish(Float64(data=lap_time))

        # best_lap 업데이트/발행
        is_new_best = lap_time < self.best_lap
        if is_new_best:
            self.best_lap = lap_time
        self.pub_best.publish(Float64(data=self.best_lap))

        # 로그
        if is_new_best:
            self.get_logger().info(f'Lap {self.lap_count}: {lap_time:.3f} s  (NEW BEST)')
        else:
            bl = self.best_lap if math.isfinite(self.best_lap) else float('nan')
            self.get_logger().info(f'Lap {self.lap_count}: {lap_time:.3f} s  | Best: {bl:.3f} s')

        # 3D 텍스트는 이벤트 때만 (선택)
        if self.show_rviz_text and self.pub_marker is not None:
            m1 = Marker()
            m1.header.frame_id = self.frame_id
            m1.header.stamp = self.get_clock().now().to_msg()
            m1.ns = 'lap_timer'
            m1.id = 0
            m1.type = Marker.TEXT_VIEW_FACING
            m1.action = Marker.ADD
            m1.pose.position.x = 0.0
            m1.pose.position.y = 0.0
            m1.pose.position.z = 1.6
            m1.scale.z = 0.40
            m1.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            m1.text = f'LAP {self.lap_count}: {lap_time:.3f} s'
            m1.lifetime = Duration(sec=3, nanosec=0)
            self.pub_marker.publish(m1)

            m2 = Marker()
            m2.header.frame_id = self.frame_id
            m2.header.stamp = m1.header.stamp
            m2.ns = 'lap_timer'
            m2.id = 1
            m2.type = Marker.TEXT_VIEW_FACING
            m2.action = Marker.ADD
            m2.pose.position.x = 0.0
            m2.pose.position.y = 0.0
            m2.pose.position.z = 1.2
            m2.scale.z = 0.34
            m2.color = ColorRGBA(r=0.6, g=1.0, b=0.6, a=1.0)
            txt = f'BEST: {self.best_lap:.3f} s' if math.isfinite(self.best_lap) else 'BEST: -'
            if is_new_best:
                txt += '  NEW BEST'
            m2.text = txt
            m2.lifetime = Duration(sec=3, nanosec=0)
            self.pub_marker.publish(m2)


def main():
    rclpy.init()
    node = FrenetLapTimer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
