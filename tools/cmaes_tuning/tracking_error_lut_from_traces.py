#!/usr/bin/env python3
"""Build the local planner's tracking-error LUT from recorded lap_referee traces.

The LUT reserves, per (planned speed, path curvature) cell, how far the car may drift from the
path it was told to follow. That is a measurable quantity, not a guess: on an OBSTACLE-FREE lap
the published local path IS the global race line, so the referee's |lat_err| against the race line
is exactly the tracking error, and it already carries the speed and the nearest waypoint index.

Only obstacle-free samples may be used. During an avoidance maneuver the car leaves the race line
on purpose by 0.3-0.5 m, and counting that as tracking error inflates every cell it lands in --
which is why --obstacle-s exists and why aggregating raw traces blindly produces a table that
measures avoidance amplitude instead.

  usage:
    tracking_error_lut_from_traces.py TRACE.csv [TRACE.csv ...] \
        --waypoints offline_trajectory_generator/output/ifac_track/global_waypoints.csv \
        [--obstacle-s 9.0 12.4 15.4] [--obstacle-exclusion-m 3.0] \
        [--max-s 4.8] [--safety-factor 1.25] [--floor 0.05]

Cells the race line never visits (fast through a hairpin, or anything below 3 m/s during normal
racing) collect no samples and are filled by monotone extension from the cells that did, never
by anything smaller than an already-measured neighbour. Every emitted cell is reported with its
sample count so a thinly-supported number is visible rather than implied.
"""
import argparse
import csv
import math
import os
import sys

# 2026-08-19 실차 세션(42.81 m 라인, 17랩)의 방문 분포로 다시 잡은 축.
#
# 곡률: 라인 실측 max |kappa| = 0.460 이다. 종전 축의 마지막 두 칸([0.5,0.9), [0.9,1.316))은
#   점유 0% 인 죽은 칸이었다 — 상단을 차량 조향한계(1.3163)로 잡았던 탓이다. 그 값은
#   "낼 수 있는 곡률"이지 "라인이 방문하는 곡률"이 아니다. 새 축의 칸 점유는 31/12/33/23%.
# 속도: 라인의 계획속도가 아니라 **캡을 걸고 돈 랩이 실제로 만든 표본 분포**로 잡는다.
#   출발 떨림을 뺀 14,796 표본에서 칸별 표본이 [3157, 3128, 2078, 4211, 1583, 639]이다.
#   1.0 에 노드를 두는 것은 avoidance_minimum_speed_mps 가 1.0 이라 갭 통과 가부를
#   그 행이 결정하기 때문이다. 상단 7.0 은 실측 최대 6.92 를 덮는다.
SPEED_BINS = [0.0, 1.0, 1.6, 2.2, 2.9, 4.5, 7.0]
CURVATURE_BINS = [0.0, 0.1, 0.2, 0.35, 0.46]


def bin_index(value, bins):
    for i in range(len(bins) - 1):
        if bins[i] <= value < bins[i + 1]:
            return i
    return len(bins) - 2


def load_curvature(path):
    with open(path) as handle:
        return {i: abs(float(r["kappa_radpm"])) for i, r in enumerate(csv.DictReader(handle))}


def start_transient_end(rows, min_speed, scan_sec, settle_sec, ratio):
    """랩 출발 과도구간의 끝 시각을 찾는다. 두 가지가 섞여 있다.

    (1) 떨림 — 정지에서 출발 명령을 넣으면 구동계가 몇 초 버벅인다. v 가 0 과 1 m/s 를
        오간다. --min-speed 만으로는 못 거른다: 떨림 중에도 v 가 순간 1.0 까지 튀어
        문턱을 통과하는데 차는 아직 라인에 못 올라탔다.
        (2026-08-19 crawl10_lap1: 떨림 구간 |lat_err| 최대 0.835, 그 뒤 0.174)

    (2) 초기 가속 — 떨림 없이도 정지에서 출발하면 v 가 cmd_v 를 따라잡는 데 ~1 초 걸린다.
        이 구간의 표본은 **라벨이 틀린다**: 표는 측정 v 로 비닝하는데, 그때 오차를 만든
        것은 명령된 주행 영역이다. full_lap3 t=0.4s 는 v=1.14 인데 cmd_v=3.36 이라
        v[1.0,1.6) 행에 3.4 m/s 짜리 오차가 들어간다.
        (2026-08-19 실측: v/cmd_v < 0.8 인 표본 269개가 **전부** t<1s 에 있고 그 |lat_err|
         p95 가 0.405 다. 전체 p95 는 0.163. 정상 주행의 v/cmd_v 는 p10=0.96 ~ p50=0.99 라
         0.7 문턱은 정상 구간을 건드리지 않는다.)

    판정에 lat_err 를 쓰지 않는다. 오차로 자르면 "오차가 커서 오차 큰 표본을 버리는"
    순환이 되어 표가 체계적으로 낙관적이 된다. 둘 다 속도 현상이므로 속도로만 자른다.

    fill() 이 셀마다 **최댓값**을 쓰므로 이런 표본 하나가 행 전체를 정해 버린다.

    반환: 잘라낼 시각 t_cut (그 이하 표본은 버린다). 과도구간이 없으면 None.
    """
    if "t" not in rows[0] or "cmd_v" not in rows[0]:
        return None
    t0 = float(rows[0]["t"])
    last = None
    for row in rows:
        t = float(row["t"]) - t0
        if t > scan_sec:
            break
        speed = float(row["v"])
        commanded = float(row["cmd_v"])
        stalled = speed < min_speed and commanded > min_speed
        ramping = commanded > min_speed and speed < ratio * commanded
        if stalled or ramping:
            last = t
    return None if last is None else last + settle_sec


def collect(trace_paths, curvature_by_index, obstacle_s, exclusion_m, max_s, min_speed,
            start_scan_sec=0.0, start_settle_sec=1.0, start_ratio=0.7):
    cells = {}
    used, skipped = 0, 0
    for trace_path in trace_paths:
        try:
            with open(trace_path) as handle:
                rows = list(csv.DictReader(handle))
        except OSError as error:
            print(f"  skip {trace_path}: {error}", file=sys.stderr)
            continue
        if not rows or "lat_err" not in rows[0]:
            print(f"  skip {trace_path}: no lat_err column", file=sys.stderr)
            continue
        t_cut = (start_transient_end(rows, min_speed, start_scan_sec, start_settle_sec,
                                     start_ratio)
                 if start_scan_sec > 0.0 else None)
        if t_cut is not None:
            print(f"  {os.path.basename(trace_path):24s} 출발 과도구간 {t_cut:5.2f}s 까지 제외",
                  file=sys.stderr)
        t_origin = float(rows[0]["t"]) if "t" in rows[0] else 0.0
        for row in rows:
            if t_cut is not None and float(row["t"]) - t_origin <= t_cut:
                skipped += 1
                continue
            speed = float(row["v"])
            station = float(row["s"])
            error = abs(float(row["lat_err"]))
            index = int(row["nearest_idx"])
            if speed < min_speed:
                continue
            if max_s is not None and station > max_s:
                skipped += 1
                continue
            if any(abs(station - o) < exclusion_m for o in obstacle_s):
                skipped += 1
                continue
            curvature = curvature_by_index.get(index, 0.0)
            key = (bin_index(speed, SPEED_BINS), bin_index(curvature, CURVATURE_BINS))
            cells.setdefault(key, []).append(error)
            used += 1
    return cells, used, skipped


def aggregate(samples, quantile):
    """셀 대표값. quantile >= 1.0 이면 최댓값.

    최댓값은 "그 칸에서 관측된 최악"이라 안전 쪽으로 정직하지만, 표본이 1000개를 넘으면
    사실상 단 하나의 순간이 칸 전체를 정한다. 2026-08-19 실측에서 같은 칸의 p95 와 max 가
    0.150 대 0.446 으로 3배 갈렸다. 분위수를 쓰면 그 꼬리를 잘라내는 대신 "그만큼은 넘을
    수 있다"를 받아들이는 것이므로, safety_factor 로 다시 덮을지 함께 판단해야 한다.
    """
    if quantile >= 1.0:
        return max(samples)
    ordered = sorted(samples)
    index = min(int(quantile * len(ordered)), len(ordered) - 1)
    return ordered[index]


def fill(cells, safety_factor, floor, quantile=1.0):
    speeds = len(SPEED_BINS)
    curvatures = len(CURVATURE_BINS)
    measured = {}
    for (si, ci), samples in cells.items():
        measured[(si, ci)] = aggregate(samples, quantile)

    table = [[None] * curvatures for _ in range(speeds)]
    for si in range(speeds):
        for ci in range(curvatures):
            # A LUT node sits on a bin edge, so it must cover the cells on both sides of it.
            neighbours = [
                measured[(s, c)]
                for s in (si - 1, si) for c in (ci - 1, ci)
                if (s, c) in measured
            ]
            if neighbours:
                table[si][ci] = max(neighbours) * safety_factor

    # Tracking error grows with both speed and curvature, so an unmeasured node is bounded from
    # below by every node it dominates (slower and straighter) and from above by every node that
    # dominates it (faster and more curved). Taking the larger of the dominated values first keeps
    # the table monotone -- the gap-driven speed cap inverts it by bisection and needs that.
    known = [row[:] for row in table]
    for si in range(speeds):
        for ci in range(curvatures):
            dominated = [
                known[s][c]
                for s in range(si + 1) for c in range(ci + 1)
                if known[s][c] is not None
            ]
            if dominated:
                value = max(dominated)
                if table[si][ci] is None or table[si][ci] < value:
                    table[si][ci] = value
    # A node below everything measured (slow and straight) has no lower bound to inherit; the
    # smallest value that still dominates it is the tightest honest ceiling available.
    for si in range(speeds):
        for ci in range(curvatures):
            if table[si][ci] is not None:
                continue
            dominating = [
                known[s][c]
                for s in range(si, speeds) for c in range(ci, curvatures)
                if known[s][c] is not None
            ]
            table[si][ci] = min(dominating) if dominating else floor
    return [
        [math.ceil(max(v, floor) * 200.0) / 200.0 for v in row]
        for row in table
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("traces", nargs="+")
    parser.add_argument("--waypoints", required=True)
    parser.add_argument("--obstacle-s", nargs="*", type=float, default=[])
    parser.add_argument("--obstacle-exclusion-m", type=float, default=3.0)
    parser.add_argument("--max-s", type=float, default=None)
    parser.add_argument("--min-speed", type=float, default=0.3)
    parser.add_argument("--safety-factor", type=float, default=1.25)
    parser.add_argument("--floor", type=float, default=0.05)
    parser.add_argument("--start-scan-sec", type=float, default=15.0,
                        help="랩 앞부분 이 시간 안에서 출발 떨림을 찾는다. 0 이면 비활성")
    parser.add_argument("--start-settle-sec", type=float, default=1.0,
                        help="마지막 과도표본 이후 추가로 버릴 시간")
    parser.add_argument("--start-speed-ratio", type=float, default=0.7,
                        help="v < 이 비율 x cmd_v 이면 아직 명령속도를 못 따라잡은 것으로 본다")
    parser.add_argument("--quantile", type=float, default=1.0,
                        help="셀 대표값의 분위수. 1.0 이면 최댓값(기본). 0.95 면 p95")
    args = parser.parse_args()

    curvature_by_index = load_curvature(args.waypoints)
    cells, used, skipped = collect(
        args.traces, curvature_by_index, args.obstacle_s,
        args.obstacle_exclusion_m, args.max_s, args.min_speed,
        args.start_scan_sec, args.start_settle_sec, args.start_speed_ratio)
    if not used:
        print("no obstacle-free samples survived the filters", file=sys.stderr)
        return 1
    print(f"# obstacle-free samples: {used} (excluded {skipped})")
    print("# measured max |lat_err| per cell, with sample counts:")
    for si in range(len(SPEED_BINS) - 1):
        for ci in range(len(CURVATURE_BINS) - 1):
            samples = cells.get((si, ci))
            if samples:
                ordered = sorted(samples)
                print(
                    f"#   v[{SPEED_BINS[si]:.1f},{SPEED_BINS[si+1]:.1f}) "
                    f"k[{CURVATURE_BINS[ci]:.2f},{CURVATURE_BINS[ci+1]:.2f}): "
                    f"n={len(samples):5d} p95={ordered[int(0.95*len(ordered))]:.3f} "
                    f"max={max(samples):.3f}")

    table = fill(cells, args.safety_factor, args.floor, args.quantile)
    agg = "max" if args.quantile >= 1.0 else f"p{int(args.quantile*100)}"
    print(f"\n# cell aggregate {agg}, safety factor {args.safety_factor}, floor {args.floor} m,"
          f" unmeasured nodes filled by monotone extension")
    print("    tracking_error_lut_speed_bins_mps: " +
          "[" + ", ".join(f"{b}" for b in SPEED_BINS) + "]")
    print("    tracking_error_lut_curvature_bins_radpm: " +
          "[" + ", ".join(f"{b}" for b in CURVATURE_BINS) + "]")
    print("    tracking_error_lut_values_m: [")
    for si, row in enumerate(table):
        measured_flags = "".join(
            "m" if (si, ci) in cells or (si - 1, ci) in cells or
                   (si, ci - 1) in cells or (si - 1, ci - 1) in cells else "-"
            for ci in range(len(CURVATURE_BINS)))
        print(f"      {', '.join(f'{v:.3f}' for v in row)},"
              f"   # v={SPEED_BINS[si]:<4} [{measured_flags}]")
    print("    ]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
