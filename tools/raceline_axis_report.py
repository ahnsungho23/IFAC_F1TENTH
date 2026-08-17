#!/usr/bin/env python3
"""새 raceline에 맞춰 추종오차 LUT의 곡률·속도 축을 다시 잡는다.

왜 필요한가: LUT 축은 `tracking_error_lut_from_traces.py` 상단에 하드코딩돼 있고,
지금 값은 예전 트랙에서 온 것이다. 라인이 바뀌면 방문하지 않는 칸(죽은 칸)이 생기고,
정작 회피 경로가 사는 구간의 해상도가 모자란다. 2026-08-17 output/map 실측:

    곡률 축 [0, 0.2, 0.5, 0.9, 1.316] 점유율  46.6% / 50.0% / 3.4% / 0%
    실측 max |kappa| = 0.560  ->  마지막 두 칸이 존재하지 않는 영역

축의 마지막 칸을 차량 조향한계(tan(0.41)/0.3302 = 1.3163)로 잡으면 안 된다. 그것은
"낼 수 있는 곡률"이지 "라인이 방문하는 곡률"이 아니다. 축은 방문 분포로 잡는다.

  사용:
    python3 tools/raceline_axis_report.py \
        offline_trajectory_generator/output/map/global_waypoints.json

표준 라이브러리만 쓴다 (현장 랩탑에서 의존성 없이 돌아야 한다).
"""
import argparse
import json
import sys

CURRENT_CURVATURE_BINS = [0.0, 0.2, 0.5, 0.9, 1.316266519079011]
CURRENT_SPEED_BINS = [0.0, 1.5, 3.0, 4.5, 6.5]


def load_waypoints(path):
    with open(path) as handle:
        data = json.load(handle)
    # 글로벌 플래너가 발행하는 것과 같은 IQP 궤적을 쓴다 — 차가 실제로 따르는 라인이다.
    for key in ("global_traj_wpnts_iqp", "global_traj_wpnts_sp"):
        if key in data:
            return data[key]["wpnts"], key
    raise SystemExit(f"{path}: global_traj_wpnts_iqp/sp 를 찾지 못했습니다.")


def occupancy(values, bins):
    counts = [0] * (len(bins) - 1)
    for value in values:
        for index in range(len(bins) - 1):
            if bins[index] <= value < bins[index + 1]:
                counts[index] += 1
                break
        else:
            counts[-1] += 1
    return counts


def report_axis(name, values, bins):
    counts = occupancy(values, bins)
    total = len(values)
    print(f"\n  현재 {name} 축 {[round(b, 3) for b in bins]}")
    for index, count in enumerate(counts):
        share = 100.0 * count / total
        bar = "#" * int(share / 2)
        dead = "   <- 죽은 칸" if count == 0 else ""
        print(f"    [{bins[index]:6.3f}, {bins[index + 1]:6.3f})  "
              f"{count:4d} ({share:5.1f}%) {bar}{dead}")


def quantile(sorted_values, q):
    if not sorted_values:
        return 0.0
    index = min(int(q * len(sorted_values)), len(sorted_values) - 1)
    return sorted_values[index]


def propose_curvature_bins(curvatures):
    """아래쪽에 고정 해상도를 두고, 위쪽 끝만 실측 최대치에 맞춘다.

    분위수로 잡으면 안 된다 — 라인의 상당 부분이 **정확히** kappa=0(직선)이라 하위 분위가
    전부 0으로 붕괴해 폭 0인 죽은 칸이 나온다. 실제로 output/map은 p25=0.000이었다.

    그리고 이 축은 라인이 아니라 **회피 스플라인의 곡률로 조회된다**. 실측(2026-08-17
    16:17 백) 회피 경로의 |kappa|는 고속 구간에서 0.002~0.17이었고, 갭 통과 속도를 가르는
    것이 바로 그 구간이다. 그래서 앞쪽 두 칸을 그 안에 고정으로 둔다.

    마지막 칸은 라인 실측 최대치다. 차량 조향한계(1.3163)로 잡으면 라인이 절대 가지 않는
    영역에 칸을 낭비하게 된다 — 지금 축의 [0.9, 1.316) 칸이 정확히 그 상태(점유 0%)다.
    """
    top = round(max(curvatures), 3)
    # 저곡률 고정 경계 + 실측 상단. 상단이 낮은 트랙이면 고정 경계 일부가 상단을 넘으므로
    # 버린다(그대로 두면 폭 0이거나 뒤집힌 칸이 생긴다).
    edges = [e for e in (0.0, 0.10, 0.20, 0.35) if e < top] + [top]
    return merge_empty_bins(edges, curvatures)


def propose_speed_bins(speeds, floor_speed, lap_caps):
    """계획한 랩 캡을 그대로 칸 경계로 삼는다.

    라인의 계획속도 분포로 잡으면 안 된다 — 라인은 2.8 m/s 아래를 계획하지 않지만, LUT의
    저속 행은 **캡을 걸고 도는 랩**이 채운다. 표는 trace의 실측 속도로 비닝되므로 축은
    그 캡들에 맞아야 한다.

    gap 기반 속도 역산은 [floor, 요청속도] 구간에서 이분 탐색한다. 그 구간 안에 칸 경계가
    하나도 없으면 표가 평평해지고 역산이 "바닥 아니면 전속"의 계단이 된다
    (2026-08-17 확정: 가용 예약 11 mm 차이가 1.36 vs 6.30 m/s 를 갈랐다).
    """
    edges = {0.0, round(floor_speed, 1)}
    edges.update(round(c, 1) for c in lap_caps)
    edges.add(round(max(speeds), 1))
    return sorted(e for e in edges if e <= round(max(speeds), 1))


def merge_empty_bins(edges, values):
    """점유 0인 칸이 남지 않을 때까지 안쪽 경계를 없앤다. 양 끝은 유지한다."""
    while len(edges) > 2:
        counts = occupancy(values, edges)
        empty = [i for i, c in enumerate(counts) if c == 0]
        if not empty:
            break
        # 빈 칸의 오른쪽 경계를 지운다. 마지막 칸이면 왼쪽 경계를 지운다.
        index = empty[0]
        drop = index + 1 if index + 1 < len(edges) - 1 else index
        if drop <= 0 or drop >= len(edges):
            break
        edges = edges[:drop] + edges[drop + 1:]
    return edges


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("waypoints_json")
    parser.add_argument("--floor-speed", type=float, default=1.0,
                        help="avoidance_minimum_speed_mps (기본 1.0)")
    parser.add_argument("--lap-caps", nargs="*", type=float,
                        default=[1.0, 2.0, 3.0, 4.5],
                        help="LUT 랩에서 걸 max_speed 캡들 — 속도 축 경계가 된다")
    args = parser.parse_args()

    wpnts, source = load_waypoints(args.waypoints_json)
    curvatures = [abs(w["kappa_radpm"]) for w in wpnts]
    speeds = [w["vx_mps"] for w in wpnts]
    length = max(w["s_m"] for w in wpnts)

    print(f"라인: {args.waypoints_json}  ({source})")
    print(f"  {len(wpnts)}점, 길이 {length:.2f} m, "
          f"vx {min(speeds):.2f}~{max(speeds):.2f}, max |kappa| {max(curvatures):.4f}")

    ordered = sorted(curvatures)
    print("  |kappa| 분위: " + "  ".join(
        f"p{int(q * 100)}={quantile(ordered, q):.3f}"
        for q in (0.10, 0.25, 0.50, 0.75, 0.90, 0.95, 1.00)))

    report_axis("곡률", curvatures, CURRENT_CURVATURE_BINS)
    report_axis("속도", speeds, CURRENT_SPEED_BINS)
    print("    (속도 점유율은 캡 없는 라인 기준이라 저속 행이 0으로 보이는 것이 정상이다 —")
    print("     저속 행은 캡을 걸고 도는 랩이 채운다. 속도 축은 그 캡으로 잡는다.)")

    curvature_bins = propose_curvature_bins(curvatures)
    speed_bins = propose_speed_bins(speeds, args.floor_speed, args.lap_caps)

    print("\n  제안 축 — tracking_error_lut_from_traces.py 상단에 그대로 넣는다:")
    print(f"    CURVATURE_BINS = {curvature_bins}")
    print(f"    SPEED_BINS     = {speed_bins}")
    print(f"\n  LUT 값 배열 길이는 {len(speed_bins) - 1} x {len(curvature_bins) - 1} "
          f"= {(len(speed_bins) - 1) * (len(curvature_bins) - 1)} 개가 된다.")
    print("  local_planning.yaml 의 두 축 배열과 tracking_error_lut_values_m 을 같이 맞출 것.")
    print("\n  ⚠️ maximum_curvature_radpm(1.3163)은 건드리지 않는다 — 트랙이 아니라")
    print("     차량 조향한계 tan(0.41)/0.3302 다.")

    report_axis("제안 곡률", curvatures, curvature_bins)
    return 0


if __name__ == "__main__":
    sys.exit(main())
