#!/usr/bin/env python3
"""차가 실제로 따르는 /global_waypoints(latched)를 CSV로 저장하는 유틸.

lap_referee·LUT 생성기에 '실행 중인 라인과 반드시 일치하는' 웨이포인트를 공급하기 위한
도구다. 파일 사본(랩탑 vs 젯슨)이 어긋나거나 순서가 반대면 referee의 진행거리가 누적되지
않는 사고(2026-08-13)가 나는데, 라이브 토픽을 덤프하면 그 실패 클래스가 원천 차단된다.

사용:  python3 tools/dump_global_waypoints.py <출력.csv> [대기초=10]
출력 컬럼: s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps (referee는 x_m/y_m/psi_rad,
LUT 생성기는 행 순서 기준 kappa_radpm을 사용)
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from f110_msgs.msg import WpntArray


def main() -> int:
    out = sys.argv[1] if len(sys.argv) > 1 else "live_global_waypoints.csv"
    wait_sec = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0

    rclpy.init()
    node = Node("global_waypoints_dumper")
    got = {}

    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,  # latched — 늦게 붙어도 수신
    )
    node.create_subscription(WpntArray, "/global_waypoints", lambda m: got.setdefault("msg", m), qos)

    deadline = time.monotonic() + wait_sec
    while time.monotonic() < deadline and "msg" not in got:
        rclpy.spin_once(node, timeout_sec=0.25)
    rclpy.try_shutdown()

    if "msg" not in got:
        print(f"DUMP_FAIL: {wait_sec:.0f}초 내 /global_waypoints 미수신", file=sys.stderr)
        return 1

    wpnts = got["msg"].wpnts
    with open(out, "w") as f:
        f.write("s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps\n")
        for w in wpnts:
            f.write(f"{w.s_m},{w.x_m},{w.y_m},{w.psi_rad},{w.kappa_radpm},{w.vx_mps}\n")
    length = wpnts[-1].s_m if wpnts else 0.0
    print(f"DUMP_OK: {len(wpnts)}점, s_max={length:.2f}m -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
