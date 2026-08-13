#!/usr/bin/env python3
"""GT(/ego_racecar/odom) 대비 MCL(/pf/pose/odom) 위치추정 오차 실측 프로브.

시뮬 주행 중 별도 터미널에서 띄워 두면 두 토픽을 동시 구독해 GT 헤딩 기준
종/횡방향 오차와 yaw 오차를 수집하고, 주기적으로 + 종료(Ctrl+C) 시점에
CSV(샘플 전체)와 JSON(백분위 요약)을 기록한다.

용도: local_planning 장애물 클리어런스에 넣을 localization_reserve_m 산정
(횡방향 |오차|의 P95가 기준값). 분석 전용 도구라 Python으로 유지한다.

사용:
  python3 tools/mcl_gt_error_probe.py            # runs/mcl_error_probe/<시각>/ 에 기록
  python3 tools/mcl_gt_error_probe.py --warmup-sec 10 --output-dir <dir>
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

DEFAULT_GT_TOPIC = "/ego_racecar/odom"
DEFAULT_MCL_TOPIC = "/pf/pose/odom"
GT_BUFFER_SEC = 3.0
SUMMARY_PERIOD_SEC = 10.0


def yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def percentile(sorted_vals: list[float], p: float) -> float:
    if not sorted_vals:
        return float("nan")
    idx = min(len(sorted_vals) - 1, max(0, int(round(p / 100.0 * (len(sorted_vals) - 1)))))
    return sorted_vals[idx]


class ErrorProbe(Node):
    def __init__(self, out_dir: Path, warmup_sec: float, gt_topic: str, mcl_topic: str):
        super().__init__("mcl_gt_error_probe")
        self.out_dir = out_dir
        self.warmup_sec = warmup_sec
        # (stamp, x, y, yaw, speed) — stamp 정렬 버퍼, MCL 콜백에서 선형보간에 사용
        self.gt_buffer: list[tuple[float, float, float, float, float]] = []
        self.samples: list[tuple[float, float, float, float, float]] = []
        self.first_stamp: float | None = None
        self.csv_path = out_dir / "samples.csv"
        self.summary_path = out_dir / "summary.json"
        self.csv_file = self.csv_path.open("w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["t_rel_sec", "lon_err_m", "lat_err_m", "yaw_err_rad", "gt_speed_mps"])
        self.create_subscription(Odometry, gt_topic, self.on_gt, 50)
        self.create_subscription(Odometry, mcl_topic, self.on_mcl, 50)
        self.create_timer(SUMMARY_PERIOD_SEC, self.write_summary)
        self.get_logger().info(
            f"recording {gt_topic} vs {mcl_topic} -> {out_dir} (warmup {warmup_sec:.0f}s 제외)"
        )

    @staticmethod
    def stamp_sec(msg: Odometry) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_gt(self, msg: Odometry) -> None:
        t = self.stamp_sec(msg)
        p = msg.pose.pose
        entry = (t, p.position.x, p.position.y, yaw_from_quat(p.orientation),
                 abs(msg.twist.twist.linear.x))
        self.gt_buffer.append(entry)
        cutoff = t - GT_BUFFER_SEC
        while self.gt_buffer and self.gt_buffer[0][0] < cutoff:
            self.gt_buffer.pop(0)

    def on_mcl(self, msg: Odometry) -> None:
        t = self.stamp_sec(msg)
        gt = self.interpolate_gt(t)
        if gt is None:
            return
        gt_x, gt_y, gt_yaw, gt_speed = gt
        if self.first_stamp is None:
            self.first_stamp = t
        t_rel = t - self.first_stamp
        if t_rel < self.warmup_sec:
            return
        p = msg.pose.pose
        dx = p.position.x - gt_x
        dy = p.position.y - gt_y
        # GT 헤딩 기준 분해: lon = 진행방향, lat = 좌측(+)
        cos_y, sin_y = math.cos(gt_yaw), math.sin(gt_yaw)
        lon_err = dx * cos_y + dy * sin_y
        lat_err = -dx * sin_y + dy * cos_y
        yaw_err = wrap_angle(yaw_from_quat(p.orientation) - gt_yaw)
        self.samples.append((t_rel, lon_err, lat_err, yaw_err, gt_speed))
        self.csv_writer.writerow(
            [f"{t_rel:.3f}", f"{lon_err:.4f}", f"{lat_err:.4f}", f"{yaw_err:.4f}", f"{gt_speed:.2f}"]
        )

    def interpolate_gt(self, t: float):
        buf = self.gt_buffer
        if len(buf) < 2 or not (buf[0][0] <= t <= buf[-1][0]):
            return None
        idx = bisect.bisect_left([e[0] for e in buf], t)
        lo, hi = buf[max(0, idx - 1)], buf[min(len(buf) - 1, idx)]
        span = hi[0] - lo[0]
        a = 0.0 if span <= 0.0 else (t - lo[0]) / span
        yaw = lo[3] + wrap_angle(hi[3] - lo[3]) * a
        return (
            lo[1] + (hi[1] - lo[1]) * a,
            lo[2] + (hi[2] - lo[2]) * a,
            wrap_angle(yaw),
            lo[4] + (hi[4] - lo[4]) * a,
        )

    def write_summary(self) -> None:
        if not self.samples:
            return
        self.csv_file.flush()
        abs_lat = sorted(abs(s[2]) for s in self.samples)
        abs_lon = sorted(abs(s[1]) for s in self.samples)
        abs_yaw = sorted(abs(s[3]) for s in self.samples)
        moving_lat = sorted(abs(s[2]) for s in self.samples if s[4] > 0.5)
        summary = {
            "samples": len(self.samples),
            "duration_sec": round(self.samples[-1][0] - self.samples[0][0], 1),
            "warmup_excluded_sec": self.warmup_sec,
            "lat_abs_m": {
                "p50": round(percentile(abs_lat, 50), 4),
                "p95": round(percentile(abs_lat, 95), 4),
                "p99": round(percentile(abs_lat, 99), 4),
                "max": round(abs_lat[-1], 4),
            },
            "lat_abs_moving_m": {
                "samples": len(moving_lat),
                "p95": round(percentile(moving_lat, 95), 4),
                "p99": round(percentile(moving_lat, 99), 4),
                "max": round(moving_lat[-1], 4) if moving_lat else float("nan"),
            },
            "lon_abs_m": {
                "p95": round(percentile(abs_lon, 95), 4),
                "max": round(abs_lon[-1], 4),
            },
            "yaw_abs_rad": {
                "p95": round(percentile(abs_yaw, 95), 4),
                "max": round(abs_yaw[-1], 4),
            },
        }
        self.summary_path.write_text(json.dumps(summary, indent=2))

    def shutdown(self) -> None:
        self.write_summary()
        self.csv_file.close()
        if self.samples:
            abs_lat = sorted(abs(s[2]) for s in self.samples)
            print(
                f"\n[mcl_gt_error_probe] {len(self.samples)} samples -> {self.summary_path}"
                f"\n  횡방향 |오차| P95 = {percentile(abs_lat, 95):.4f} m"
                f" / P99 = {percentile(abs_lat, 99):.4f} m / max = {abs_lat[-1]:.4f} m"
            )
        else:
            print("\n[mcl_gt_error_probe] 수집된 샘플 없음 — 두 토픽이 모두 발행 중인지 확인")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gt-topic", default=DEFAULT_GT_TOPIC)
    parser.add_argument("--mcl-topic", default=DEFAULT_MCL_TOPIC)
    parser.add_argument("--warmup-sec", type=float, default=10.0,
                        help="MCL 수렴 전 초기 구간 제외 (기본 10초)")
    parser.add_argument("--output-dir", default=None,
                        help="기본: runs/mcl_error_probe/<시각>")
    args = parser.parse_args()

    if args.output_dir:
        out_dir = Path(args.output_dir)
    else:
        stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        out_dir = Path(__file__).resolve().parent.parent / "runs" / "mcl_error_probe" / stamp
    out_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = ErrorProbe(out_dir, args.warmup_sec, args.gt_topic, args.mcl_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
