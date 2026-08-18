#!/usr/bin/env python3
"""백의 특정 시각(백 시작 기준 초)에서 /pf/pose/odom 의 x y yaw 를 출력한다."""
import sys, math, os
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def main(path, at):
    storage = "mcap" if any(f.endswith(".mcap") for f in os.listdir(path)) else "sqlite3"
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage), rosbag2_py.ConverterOptions("", ""))
    ty = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=["/pf/pose/odom"]))
    T = get_message(ty["/pf/pose/odom"])
    t0 = None; best = None
    while r.has_next():
        _, d, ts = r.read_next()
        ts /= 1e9
        if t0 is None: t0 = ts
        t = ts - t0
        m = deserialize_message(d, T)
        if best is None or abs(t - at) < abs(best[0] - at):
            q = m.pose.pose.orientation
            best = (t, m.pose.pose.position.x, m.pose.pose.position.y,
                    math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        if t > at + 1.0: break
    print(f"{best[1]:.6f} {best[2]:.6f} {best[3]:.6f}")

main(sys.argv[1], float(sys.argv[2]))
