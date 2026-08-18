#!/usr/bin/env python3
"""리플레이 시작 시점의 MCL 포즈를 원본 백에서 꺼내 /initialpose 로 한 번 쏜다.

왜 필요한가: 리플레이는 백 중간(자율 구간 직전)부터 재생하므로 원본의 /initialpose
(대개 주행 맨 앞에 몰려 있다)가 재생되지 않는다. auto_init_from_waypoints 에 맡기면
wpnts[0] 에 파티클을 뿌려 차가 실제로 있는 곳과 무관한 자리에서 시작한다.

원본 포즈로 씨앗을 주는 것은 "이미 수렴한 상태에서 출발"을 뜻하므로, 이 하네스는
초기 수렴 성능이 아니라 **추종 중 오차**를 재는 도구다. 그 구분을 지킬 것.
"""
import argparse, math, sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, required=True)
    ap.add_argument("--y", type=float, required=True)
    ap.add_argument("--yaw", type=float, required=True)
    ap.add_argument("--repeat", type=int, default=3)
    ap.add_argument("--period", type=float, default=1.0)
    args = ap.parse_args()

    rclpy.init()
    node = Node("mcl_seed_pose")
    pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", QoSProfile(depth=1))
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = args.x
    msg.pose.pose.position.y = args.y
    msg.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(args.yaw / 2.0)
    # RViz 기본 공분산과 같은 크기 (0.25 m^2, 0.068 rad^2)
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.06853891945200942

    sent = 0
    def tick():
        nonlocal sent
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        sent += 1
        node.get_logger().info(f"seeded /initialpose ({sent}/{args.repeat}) "
                               f"x={args.x:.3f} y={args.y:.3f} yaw={args.yaw:.3f}")
        if sent >= args.repeat:
            raise SystemExit(0)

    node.create_timer(args.period, tick)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
