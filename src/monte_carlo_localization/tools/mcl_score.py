#!/usr/bin/env python3
"""MCL 포즈 품질 채점기 — GT 없이 스캔-맵 정합으로 횡방향 위치오차를 역산한다.

실차에는 GT가 없다. 대신 백에 /scan 과 /pf/pose/odom 이 함께 있으므로, MCL 이 준
포즈로 스캔을 맵에 얹어 보고 "얼마나 옆으로 밀어야 벽에 제일 잘 붙는가"를 잰다.
그 밀어야 하는 양이 횡방향 위치오차의 추정치다.

    python3 mcl_score.py <bag_dir> [--map src/monte_carlo_localization/maps/map.yaml]

⚠️ 두 가지 한계를 알고 쓸 것.
  1. 종방향(s)은 못 잰다. 복도형 트랙에서 스캔은 s 를 구속하지 않아, 2차원으로 풀면
     해가 복도를 따라 자유롭게 미끄러진다(실측: 해의 50%가 탐색 경계에 붙었다).
     그래서 헤딩 법선 방향 1차원만 푼다. s 정확도는 speed_to_erpm_gain 이 쥔다.
  2. 이것은 GT 가 아니다. 잔차에는 위치오차 말고 요 오차·맵 오차·라이다 노이즈가
     같이 들어 있고, 평행이동만 훑으므로 요 오차가 가짜 횡오차로 새어 든다.
     최적 보정 후에도 잔차 중앙값이 0.10 m 근처에서 안 내려가는 것이 그 바닥값이다.

거리장은 '자유/점유 경계면까지의 거리'(|SDF|)를 쓴다. '점유셀까지의 거리'로 잡으면
맵 바깥의 거대한 점유 영역 안으로 스캔을 통째로 밀어 넣을 때 잔차가 0이 되어
최적화가 탐색 경계로 달아난다(2026-08-18 에 실제로 겪음).
"""
from __future__ import annotations
import argparse, bisect, json, math, os, sys
import numpy as np
from scipy.ndimage import distance_transform_edt

DEFAULT_MAP = "src/monte_carlo_localization/maps/map.yaml"
LASER_X_DEFAULT = 0.27


def load_map(map_yaml):
    import yaml
    from PIL import Image
    with open(map_yaml) as fh:
        meta = yaml.safe_load(fh)
    img_path = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), meta["image"])
    img = np.asarray(Image.open(img_path).convert("L"))
    # ROS map_server 규약: png 는 위->아래, 격자는 아래->위
    img = img[::-1, :]
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    occ_th = float(meta.get("occupied_thresh", 0.65))
    free_th = float(meta.get("free_thresh", 0.196))
    if int(meta.get("negate", 0)):
        img = 255 - img
    p_occ = (255.0 - img) / 255.0          # 어두울수록 점유
    occupied = p_occ > occ_th
    free = p_occ < free_th
    return occupied, free, res, ox, oy


def surface_distance(occupied, free, res):
    """자유/점유 경계면까지의 거리."""
    d_free = distance_transform_edt(~occupied) * res
    d_occ = distance_transform_edt(~free) * res
    return np.where(occupied, d_occ, d_free)


def read_bag(path, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    storage = "mcap"
    if not any(f.endswith(".mcap") for f in os.listdir(path)):
        storage = "sqlite3"
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
           rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    present = [t for t in topics if t in types]
    if not present:
        raise SystemExit(f"{path}: {topics} 중 아무것도 없습니다.")
    r.set_filter(rosbag2_py.StorageFilter(topics=present))
    cache = {}
    while r.has_next():
        topic, data, ts = r.read_next()
        cache.setdefault(topic, get_message(types[topic]))
        yield topic, deserialize_message(data, cache[topic]), ts / 1e9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def step_lookup(times, values, t):
    i = bisect.bisect_right(times, t) - 1
    return values[i] if i >= 0 else None


def pctl(v, q):
    if len(v) == 0:
        return float("nan")
    s = np.sort(np.asarray(v))
    return float(s[min(int(q * len(s)), len(s) - 1)])


def score(bag, map_yaml, span, max_range, max_scans, laser_x, autonomous_only):
    occupied, free, res, ox, oy = load_map(map_yaml)
    dist = surface_distance(occupied, free, res)
    H, W = dist.shape

    scans, poses, mode, estop = [], [], [], []
    t0 = None
    for topic, m, ts in read_bag(
            bag, ["/scan", "/pf/pose/odom", "/drive_mode", "/estop_lock"]):
        if t0 is None:
            t0 = ts
        t = ts - t0
        if topic == "/scan":
            scans.append((t, m))
        elif topic == "/pf/pose/odom":
            poses.append((t, m.pose.pose.position.x, m.pose.pose.position.y,
                          yaw_of(m.pose.pose.orientation)))
        elif topic == "/drive_mode":
            mode.append((t, m.data))
        elif topic == "/estop_lock":
            estop.append((t, m.data))
    if not scans or not poses:
        raise SystemExit(f"{bag}: /scan 또는 /pf/pose/odom 이 없습니다.")

    mt = [x[0] for x in mode]; mv = [x[1] for x in mode]
    et = [x[0] for x in estop]; ev = [x[1] for x in estop]
    pt = [x[0] for x in poses]; pv = [(x[1], x[2], x[3]) for x in poses]

    def selected(t):
        if not autonomous_only or not mode:
            return True
        return step_lookup(mt, mv, t) == "autonomous" and not step_lookup(et, ev, t)

    use = [(t, m) for t, m in scans if selected(t)]
    if not use:
        raise SystemExit(f"{bag}: 채점 대상 스캔이 없습니다 "
                         f"(--all 로 자율주행 필터를 끌 수 있습니다).")
    use = use[::max(1, len(use) // max_scans)]

    def lookup(px, py):
        ix = np.clip(((px - ox) / res).astype(np.int32), 0, W - 1)
        iy = np.clip(((py - oy) / res).astype(np.int32), 0, H - 1)
        return dist[iy, ix]

    lat_grid = np.arange(-span, span + 1e-9, 0.01)
    raw, corr, best, edge = [], [], [], 0
    for t, m in use:
        P = step_lookup(pt, pv, t)
        if P is None:
            continue
        x, y, th = P
        rng = np.asarray(m.ranges, dtype=np.float64)
        ang = m.angle_min + np.arange(len(rng)) * m.angle_increment
        ok = np.isfinite(rng) & (rng > m.range_min) & (rng < max_range)
        if ok.sum() < 60:
            continue
        rng, ang = rng[ok], ang[ok]
        if len(rng) > 220:
            idx = np.linspace(0, len(rng) - 1, 220).astype(int)
            rng, ang = rng[idx], ang[idx]
        bx = laser_x + rng * np.cos(ang)
        by = rng * np.sin(ang)
        c, s = math.cos(th), math.sin(th)
        px = x + c * bx - s * by
        py = y + s * bx + c * by
        raw.append(float(np.median(lookup(px, py))))
        nx, ny = -s, c
        curve = np.median(
            lookup(px[None, :] + lat_grid[:, None] * nx,
                   py[None, :] + lat_grid[:, None] * ny), axis=1)
        j = int(np.argmin(curve))
        if abs(lat_grid[j]) > span - 1e-6:
            edge += 1
        best.append(float(curve[j]))
        corr.append(float(lat_grid[j]))

    n = len(corr)
    a = np.abs(np.asarray(corr))
    over = float(np.mean(np.asarray(raw) > 0.20)) if raw else float("nan")
    return {
        "bag": bag, "scans": n,
        "edge_ratio": edge / n if n else float("nan"),
        "err_mean": float(a.mean()) if n else float("nan"),
        "err_p50": pctl(a, .50), "err_p90": pctl(a, .90), "err_p95": pctl(a, .95),
        "err_max": float(a.max()) if n else float("nan"),
        "bias_mean": float(np.mean(corr)) if n else float("nan"),
        "resid_mean": float(np.mean(raw)) if raw else float("nan"),
        "resid_p50": pctl(raw, .50), "resid_p90": pctl(raw, .90),
        "resid_over_020": over,
        "resid_p50_corrected": pctl(best, .50),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--map", default=DEFAULT_MAP)
    ap.add_argument("--span", type=float, default=1.2, help="횡 탐색 범위 [m]")
    ap.add_argument("--max-range", type=float, default=8.0)
    ap.add_argument("--max-scans", type=int, default=600)
    ap.add_argument("--laser-x", type=float, default=LASER_X_DEFAULT)
    ap.add_argument("--all", action="store_true",
                    help="자율주행 구간 필터를 끄고 전체 스캔을 채점")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    out = [score(b, args.map, args.span, args.max_range, args.max_scans,
                 args.laser_x, not args.all) for b in args.bags]
    if args.json:
        print(json.dumps(out, indent=2))
        return 0
    for r in out:
        print(f"\n{r['bag']}   스캔 {r['scans']}개  "
              f"(탐색경계 걸림 {100*r['edge_ratio']:.1f}%)")
        print(f"  횡오차   평균 {r['err_mean']:.3f}  p50 {r['err_p50']:.3f}  "
              f"p90 {r['err_p90']:.3f}  p95 {r['err_p95']:.3f}  max {r['err_max']:.3f} m")
        print(f"  편향(부호) {r['bias_mean']:+.3f} m")
        print(f"  스캔-벽 잔차 평균 {r['resid_mean']:.3f}  p50 {r['resid_p50']:.3f}  "
              f"p90 {r['resid_p90']:.3f}   0.20 m 초과 {100*r['resid_over_020']:.1f}%")
        print(f"  보정 후 잔차 p50 {r['resid_p50_corrected']:.3f} (방법 바닥값)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
