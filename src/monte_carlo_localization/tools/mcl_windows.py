#!/usr/bin/env python3
"""백에서 연속 자율주행(autonomous & !estop & 실제 이동) 구간을 찾아 출력한다.

리플레이는 실시간으로 도는데 백 전체는 대부분 수동/정지라 낭비가 크다.
이 스크립트가 뽑아 준 [시작, 길이]를 mcl_replay.sh 에 그대로 넘긴다.
"""
import sys, bisect
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def read(p, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=p, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    ty = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in ty]))
    c = {}
    while r.has_next():
        tp, d, ts = r.read_next(); c.setdefault(tp, get_message(ty[tp]))
        yield tp, deserialize_message(d, c[tp]), ts / 1e9

def look(ts, vs, t):
    i = bisect.bisect_right(ts, t) - 1
    return vs[i] if i >= 0 else None

def main(path, min_len=8.0):
    mode, est, odo = [], [], []
    t0 = None
    for tp, m, ts in read(path, ["/drive_mode", "/estop_lock", "/odom"]):
        if t0 is None: t0 = ts
        t = ts - t0
        if tp == "/drive_mode": mode.append((t, m.data))
        elif tp == "/estop_lock": est.append((t, m.data))
        elif tp == "/odom": odo.append((t, abs(m.twist.twist.linear.x)))
    mt = [x[0] for x in mode]; mv = [x[1] for x in mode]
    et = [x[0] for x in est];  ev = [x[1] for x in est]
    wins = []; start = None
    for t, v in odo:
        ok = look(mt, mv, t) == "autonomous" and not look(et, ev, t) and v > 0.3
        if ok and start is None: start = t
        elif not ok and start is not None:
            if t - start >= 1.0: wins.append((start, t - start))
            start = None
    if start is not None: wins.append((start, odo[-1][0] - start))
    # 짧은 구간 사이 공백 2초 이하는 하나로 잇는다 (리플레이 오버헤드 절감)
    merged = []
    for s, d in wins:
        if merged and s - (merged[-1][0] + merged[-1][1]) <= 2.0:
            merged[-1] = (merged[-1][0], s + d - merged[-1][0])
        else:
            merged.append((s, d))
    keep = [w for w in merged if w[1] >= min_len]
    print(f"{path}")
    print(f"  자율+주행 구간 {len(merged)}개, 그중 {min_len:.0f}s 이상 {len(keep)}개, "
          f"합계 {sum(d for _, d in merged):.0f}s")
    for s, d in sorted(keep, key=lambda x: -x[1]):
        print(f"    --start {s:7.1f} --duration {d:6.1f}")
    if keep:
        b, bd = max(keep, key=lambda x: x[1])
        print(f"  최장: --start {b:.1f} --duration {bd:.1f}")

for p in sys.argv[1:]: main(p)
