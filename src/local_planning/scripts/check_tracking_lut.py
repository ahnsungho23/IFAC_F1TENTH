#!/usr/bin/env python3
# Copyright 2026 2026_IFAC contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
대회장 추종오차 LUT 점검기 (2026-08-20).

리허설 rosbag 하나로 '현재 LUT 가 이 바닥에서도 유효한가'를 1분 안에 판정한다.
LUT(v,κ) 는 차+제어기 성질이라 트랙이 바뀌어도 이설되지만, 바닥 마찰이 다르면
오차가 전반적으로 커질 수 있다. 그 경우 LUT 를 다시 재지 말고
local_planning.yaml 의 localization_reserve_m (상수 바닥값) 하나만 올린다.

사용:
  python3 check_tracking_lut.py <bag 경로> [--yaml <local_planning.yaml>]

측정: 자율주행(estop 아님) & v>1.0 구간에서 |ego_d − /local_waypoints d(ego_s)| 를
LUT 의 (속도, |κ|) 칸으로 비닝해 p95 를 구하고, 칸별 LUT+localization_reserve 와
비교한다. κ 는 매칭된 waypoint 의 kappa_radpm (플래너가 조회하는 바로 그 값).

표본 규칙 — 오염 배제가 핵심이다 (없으면 충돌 리허설 백에서 권고값이 폭주한다):
  (1) 자율 전환·estop 해제 후 2.0 s 는 버린다 (복구 과도구간).
  (2) 경로 전환 순간(연속 두 메시지의 자차 위치 d 가 0.15 m 이상 점프) 후 1.5 s 는
      버린다 — 그 오차는 컨트롤러가 아니라 리플랜 불연속이 만든 것이다.
  (3) v>1.0, 경로가 자차를 덮는 표본(최근접 wpnt s 거리 ≤ 1.0 m)만 쓴다.
  실측 예 (run_161657): 배제 전 p95 0.925 칸이 배제 후 정상 범위로 내려온다 —
  그 칸의 표본은 전부 충돌 직후 정지·복구 구간이었다.
"""
import argparse
import bisect
import os
import sys

import numpy as np


def read_bag(path, topics):
    """지정 토픽 메시지를 bag 에서 (topic, msg, stamp[s]) 로 순회한다."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(
        rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    cache = {}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        cache.setdefault(topic, get_message(types[topic]))
        yield topic, deserialize_message(data, cache[topic]), stamp / 1e9


def main():
    """측정 결과로 LUT 대비 판정과 localization_reserve_m 권고를 출력한다."""
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bag')
    parser.add_argument(
        '--yaml', default=None,
        help='local_planning.yaml 경로 (기본: 이 스크립트 기준 ../config)')
    parser.add_argument('--min-speed', type=float, default=1.0)
    args = parser.parse_args()

    import yaml as yaml_mod
    yaml_path = args.yaml or os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', 'config', 'local_planning.yaml')
    with open(yaml_path, encoding='utf-8') as handle:
        params = yaml_mod.safe_load(handle)['local_planner_node']['ros__parameters']
    speed_bins = list(params['tracking_error_lut_speed_bins_mps'])
    curv_bins = list(params['tracking_error_lut_curvature_bins_radpm'])
    loc_reserve = float(params.get('localization_reserve_m', 0.0))
    lut = np.array(params['tracking_error_lut_values_m']).reshape(
        len(speed_bins), len(curv_bins))

    paths = []
    frenet = []
    mode = []
    estop = []
    path_count = 0
    for topic, msg, stamp in read_bag(
            args.bag, ['/local_waypoints', '/car_state/frenet/odom',
                       '/drive_mode', '/estop_lock']):
        if topic == '/local_waypoints':
            path_count += 1
            if msg.wpnts and path_count % 2 == 0:
                paths.append((stamp, np.array(
                    [[w.s_m, w.d_m, abs(w.kappa_radpm)] for w in msg.wpnts])))
        elif topic == '/car_state/frenet/odom':
            frenet.append((stamp, msg.pose.pose.position.x,
                           msg.pose.pose.position.y, msg.twist.twist.linear.x))
        elif topic == '/drive_mode':
            mode.append((stamp, msg.data))
        elif topic == '/estop_lock':
            estop.append((stamp, bool(msg.data)))
    if not paths or not frenet:
        print('표본 없음: /local_waypoints 또는 /car_state/frenet/odom 이 백에 없다.')
        return 2

    mode_t = [x[0] for x in mode]
    mode_v = [x[1] for x in mode]
    estop_t = [x[0] for x in estop]
    estop_v = [x[1] for x in estop]
    path_t = [x[0] for x in paths]
    frenet_t = [x[0] for x in frenet]

    transitions = []
    for k in range(1, len(mode)):
        if mode[k][1] != mode[k - 1][1]:
            transitions.append(mode[k][0])
    for k in range(1, len(estop)):
        if estop[k][1] != estop[k - 1][1]:
            transitions.append(estop[k][0])
    transitions.sort()

    def in_transient(t, window=2.0):
        idx = bisect.bisect_right(transitions, t) - 1
        return idx >= 0 and t - transitions[idx] < window

    track_len = max(float(p[:, 0].max()) for _, p in paths) + 0.5

    def d_at(path, ego_s):
        ds = np.abs(path[:, 0] - ego_s)
        ds = np.minimum(ds, track_len - ds)
        nearest = int(np.argmin(ds))
        return path[nearest, 1] if ds[nearest] <= 1.0 else None

    switches = []
    for k in range(1, len(paths)):
        stamp = paths[k][0]
        fi = bisect.bisect_right(frenet_t, stamp) - 1
        if fi < 0:
            continue
        ego_s = frenet[fi][1]
        prev_d = d_at(paths[k - 1][1], ego_s)
        cur_d = d_at(paths[k][1], ego_s)
        if prev_d is not None and cur_d is not None and abs(cur_d - prev_d) > 0.15:
            switches.append(stamp)

    def after_switch(t, window=1.5):
        idx = bisect.bisect_right(switches, t) - 1
        return idx >= 0 and t - switches[idx] < window

    err_values = []
    speeds = []
    kappas = []
    for stamp, ego_s, ego_dv, ego_v in frenet:
        mi = bisect.bisect_right(mode_t, stamp) - 1
        ei = bisect.bisect_right(estop_t, stamp) - 1
        if not (mi >= 0 and mode_v[mi] == 'autonomous'):
            continue
        if ei >= 0 and estop_v[ei]:
            continue
        if ego_v < args.min_speed:
            continue
        if in_transient(stamp) or after_switch(stamp):
            continue
        pi = bisect.bisect_right(path_t, stamp) - 1
        if pi < 0 or stamp - path_t[pi] > 0.5:
            continue
        path = paths[pi][1]
        ds = np.abs(path[:, 0] - ego_s)
        ds = np.minimum(ds, track_len - ds)
        nearest = int(np.argmin(ds))
        if ds[nearest] > 1.0:
            continue
        err_values.append(abs(ego_dv - path[nearest, 1]))
        speeds.append(ego_v)
        kappas.append(path[nearest, 2])

    err = np.array(err_values)
    spd = np.array(speeds)
    kap = np.array(kappas)
    keep = np.isfinite(err)
    err, spd, kap = err[keep], spd[keep], kap[keep]

    print(f'표본 {len(err)}개 (자율, v>{args.min_speed}, 과도·전환 배제)   '
          f'LUT {len(speed_bins)}x{len(curv_bins)}, '
          f'localization_reserve_m={loc_reserve}   경로전환 {len(switches)}회')
    if len(err) < 200:
        print('⚠️ 표본이 200개 미만 — 판정 신뢰 불가. 랩을 더 돌 것.')
        return 2

    worst = 0.0
    print(f"\n{'v칸':>10s}{'κ칸':>12s}{'n':>6s}{'실측p95':>9s}{'LUT+loc':>9s}  판정")
    for si in range(len(speed_bins)):
        v_lo = speed_bins[si]
        v_hi = speed_bins[si + 1] if si + 1 < len(speed_bins) else 99.0
        for ci in range(len(curv_bins)):
            c_lo = curv_bins[ci]
            c_hi = curv_bins[ci + 1] if ci + 1 < len(curv_bins) else 99.0
            mask = (spd >= v_lo) & (spd < v_hi) & (kap >= c_lo) & (kap < c_hi)
            count = int(mask.sum())
            if count < 50:
                continue
            p95 = float(np.percentile(err[mask], 95))
            budget = lut[si, ci] + loc_reserve
            over = p95 - budget
            suspicious = p95 > 2.0 * budget
            if not suspicious:
                worst = max(worst, over)
            if over <= 0:
                tag = 'OK'
            elif suspicious:
                tag = f'+{over:.3f} ⚠️오염?'
            else:
                tag = f'+{over:.3f}'
            print(f'{v_lo:4.1f}-{v_hi:4.1f}{c_lo:6.2f}-{c_hi:4.2f}'
                  f'{count:6d}{p95:9.3f}{budget:9.3f}  {tag}')
    print()
    print('(⚠️오염? = p95 가 예산의 2배 초과 — 충돌·수동개입이 섞였을 가능성.')
    print(' 권고 계산에서 제외했다. 정상 주행이었다면 해당 구간 백을 따로 볼 것.)')
    if worst <= 0.0:
        print('✅ 모든 칸 OK — localization_reserve_m 그대로 두면 된다.')
        return 0
    bump = float(np.ceil((loc_reserve + worst) / 0.05) * 0.05)
    print(f'⚠️ 최대 초과 {worst:.3f} m → local_planning.yaml 의 '
          f'localization_reserve_m 를 {bump:.2f} 로 올릴 것 (현재 {loc_reserve:.2f}).')
    print('   빌드 불필요 — yaml 수정 후 local_planning 재시작만 하면 된다.')
    return 1


if __name__ == '__main__':
    sys.exit(main())
