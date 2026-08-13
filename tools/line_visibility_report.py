#!/usr/bin/env python3
"""레이스라인의 '코너 뒤 노출 거리'(line-of-sight reveal distance) 분석.

각 웨이포인트 j(잠재적 장애물 위치)에 대해, 경로 뒤쪽 i에서 j까지의 직선 시선이
점유 격자(벽·미지 영역 = 차폐)를 통과하지 않는 최대 호 길이를 구한다. 이 값이
P3 커밋 요구(7~9 m)보다 작으면 그 지점의 은닉 장애물은 라인 선택과 무관하게
늦게 발견된다.

배경(2026-08-13): out-out-out 라인이 코너 뒤 시야를 넓혀줄 것이라는 가설을 이
도구로 검증했으나, ifac_track(시뮬)과 실차 slam 맵 모두에서 기각됐다 —
복도 폭(~2 m) 대비 시선 코드(7~12 m)가 길어서 안쪽 벽 차폐를 라인 이동(±0.5 m)
으로는 극복할 수 없다 (min 노출 2.9→3.0 m, <7 m 구간 23.7 m/35.2 m 불변).
결론: 은닉 장애물 대응은 라인이 아니라 (a) 단계적 safe-stop 사다리와
(b) 랩 간 장애물 기억(adaptive global trajectory)이 담당해야 한다.

사용:
  python3 tools/line_visibility_report.py <map.yaml> <global_waypoints.csv> [lookahead=12.0]
"""
import csv
import math
import os
import sys

import numpy as np
import yaml
from PIL import Image


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    map_yaml = os.path.expanduser(sys.argv[1])
    line_csv = os.path.expanduser(sys.argv[2])
    lookahead = float(sys.argv[3]) if len(sys.argv) > 3 else 12.0

    info = yaml.safe_load(open(map_yaml))
    img = np.array(
        Image.open(os.path.join(os.path.dirname(map_yaml), info["image"])).convert("L"))
    res, origin = info["resolution"], info["origin"]
    occ = img < 250  # 벽(0)·미지(205) 모두 차폐로 간주 (보수적)
    height = img.shape[0]

    def occupied(x: float, y: float) -> bool:
        c = int((x - origin[0]) / res)
        r = height - 1 - int((y - origin[1]) / res)
        if c < 0 or r < 0 or r >= occ.shape[0] or c >= occ.shape[1]:
            return True
        return bool(occ[r, c])

    def los_clear(p, q) -> bool:
        dist = math.hypot(q[0] - p[0], q[1] - p[1])
        steps = max(2, int(dist / 0.02))
        for t in range(1, steps):
            f = t / steps
            if occupied(p[0] + f * (q[0] - p[0]), p[1] + f * (q[1] - p[1])):
                return False
        return True

    rows = list(csv.DictReader(open(line_csv)))
    s_key = "s" if "s" in rows[0] else "s_m"
    pts = [(float(r["x_m"]), float(r["y_m"])) for r in rows]
    s = [float(r[s_key]) for r in rows]
    count = len(pts)
    lap = s[-1] + (s[1] - s[0])

    reveal = np.zeros(count)
    for j in range(count):
        best = 0.0
        i = j
        while True:
            i = (i - 1) % count
            arc = (s[j] - s[i]) % lap
            if arc > lookahead or i == j:
                break
            if arc > best and los_clear(pts[i], pts[j]):
                best = arc
        reveal[j] = best

    frac = lap / count
    print(f"track {lap:.1f} m, {count} pts, lookahead {lookahead:.1f} m")
    print(f"reveal: min {reveal.min():.2f}  p10 {np.percentile(reveal, 10):.2f}  "
          f"median {np.median(reveal):.2f}  max {reveal.max():.2f}")
    for threshold in (7.0, 9.0):
        blocked = (reveal < threshold).sum() * frac
        print(f"  reveal < {threshold:.0f} m (P3 커밋 위험): {blocked:.1f} m 구간 "
              f"({100.0 * blocked / lap:.0f}%)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
