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
"""주행 백에서 P3 회귀 시나리오를 뽑아 PATH_FAMILY_STREAM_V1 스트림으로 저장한다.

왜 필요한가: P3의 후보 생성을 고치려면 "지금 무엇이 왜 실패하는지"가 먼저 테스트로
고정돼야 한다. 그런데 기준선(global_waypoints)은 offline_trajectory_generator/output/에
있고 그 디렉터리는 gitignore다. 그래서 시나리오 파일이 기준선을 **직접 포함**한다 —
그러면 다른 환경에서도 그대로 재현된다.

사용:
  python3 tools/extract_p3_scenarios.py <bag> <yaml> <out.stream> \
      --at 78.3 --at 14.5 [--name failing_cluster_11]

  --at 은 백 시작 기준 초. 그 시각에 가장 가까운 p3_shadow 콜백의 자차/장애물 상태를
  한 프레임으로 담는다. 여러 번 주면 한 파일에 여러 프레임이 들어간다.
"""
import argparse
import json
import pathlib
import sys

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# 스트림이 실어야 하는 파라미터. 여기 없는 값은 RacelineSplineParameters의 구조체 기본값이
# 쓰이므로, 시나리오가 운영과 다른 마진으로 조용히 통과할 수 있다. 새 파라미터를 YAML에
# 추가하면 이 목록도 같이 늘릴 것 — test_p3_production_parity.cpp가 전부 요구한다.
SCALARS = [
    'detection_lookahead_m', 'obstacle_cluster_gap_m', 'obstacle_longitudinal_padding_m',
    'vehicle_length_m', 'vehicle_half_width_m', 'safety_margin_m', 'tracking_error_reserve_m',
    'wall_safety_margin_m', 'fallback_track_half_width_m', 'outside_line_transition_scale',
    'post_merge_lookahead_m', 'post_merge_min_time_sec', 'minimum_target_offset_m',
    'maximum_target_offset_m', 'target_d_candidate_count', 'maximum_lateral_slope',
    'maximum_curvature_radpm', 'maximum_curvature_rate_radpm2', 'safe_stop_buffer_m',
    'safe_stop_deceleration_mps2', 'minimum_path_points', 'localization_reserve_m',
    'avoidance_minimum_speed_mps', 'margin_pass_speed_cap_mps',
    'commitment_retention_reserve_fraction',
]
VECTORS = [
    'tracking_error_lut_speed_bins_mps', 'tracking_error_lut_curvature_bins_radpm',
    'tracking_error_lut_values_m', 'avoidance_velocity_limit_speed_bins_mps',
    'avoidance_velocity_limit_lateral_accel_mps2', 'pre_apex_distances_m',
    'post_apex_distances_m', 'entry_transition_fractions', 'transition_distance_scales',
]


def load_params(path):
    doc = yaml.safe_load(open(path))
    params = doc['local_planner_node']['ros__parameters']
    missing = [k for k in SCALARS + VECTORS if k not in params]
    if missing:
        sys.exit(f'YAML에 없는 파라미터: {missing}')
    return params


def read_bag(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    need = {'/global_waypoints', '/local_planning/p3_shadow', '/confirmed_static_obs'}
    cls = {n: get_message(t) for n, t in types.items() if n in need}
    reference, shadows, obstacles = None, [], []
    t0 = None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in cls:
            continue
        if t0 is None:
            t0 = stamp
        t = (stamp - t0) * 1e-9
        message = deserialize_message(data, cls[topic])
        if topic == '/global_waypoints' and reference is None:
            reference = message.wpnts
        elif topic == '/local_planning/p3_shadow':
            try:
                shadows.append((t, json.loads(message.data)))
            except ValueError:
                pass
        elif topic == '/confirmed_static_obs':
            obstacles.append((t, message.obstacles))
    if reference is None or not shadows:
        sys.exit('백에 /global_waypoints 또는 /local_planning/p3_shadow가 없다')
    return reference, shadows, obstacles


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    parser.add_argument('yaml')
    parser.add_argument('out')
    parser.add_argument('--at', type=float, action='append', required=True)
    parser.add_argument('--name', default=None)
    # 같은 시각에 콜백이 여러 개 있고 대부분은 "커밋 유지"라 신규 평가를 하지 않는다. 시각만으로
    # 고르면 재현하려던 장면이 아니라 그 옆의 유지 콜백이 잡힌다(실측으로 확인). 재현하려는
    # 판정 종류를 명시해서 그 조건을 만족하는 가장 가까운 콜백을 고른다.
    parser.add_argument('--reject', default=None,
                        help='이 fresh_rejection 값을 가진 콜백만 후보로 삼는다')
    args = parser.parse_args()

    params = load_params(args.yaml)
    reference, shadows, obstacle_stream = read_bag(args.bag)

    lines = ['PATH_FAMILY_STREAM_V1',
             f'SCENARIO\t{args.name or pathlib.Path(args.out).stem}',
             f'SOURCE_BAG\t{pathlib.Path(args.bag).name}',
             f'CONFIG_PATH\t{args.yaml}']
    for key in SCALARS:
        lines.append(f'PARAM\t{key}\t{float(params[key])!r}')
    for key in VECTORS:
        values = params[key]
        lines.append('PARAMV\t{}\t{}\t{}'.format(
            key, len(values), '\t'.join(repr(float(v)) for v in values)))
    lines.append(f'REFERENCE\t{len(reference)}')
    for w in reference:
        lines.append('W\t{}\t{!r}\t{!r}\t{!r}\t{!r}\t{!r}\t{!r}\t{!r}\t{!r}\t{!r}\t{!r}'.format(
            w.id, w.s_m, w.d_m, w.x_m, w.y_m, w.d_right, w.d_left,
            w.psi_rad, w.kappa_radpm, w.vx_mps, w.ax_mps2))

    for want in args.at:
        # 그 시각에 가장 가까운 p3_shadow 콜백 = 그 순간 플래너가 실제로 본 자차 상태.
        pool = [s for s in shadows
                if args.reject is None or s[1].get('fresh_rejection') == args.reject]
        if not pool:
            sys.exit(f'--reject {args.reject} 인 콜백이 백에 없다')
        t, shadow = min(pool, key=lambda s: abs(s[0] - want))
        if abs(t - want) > 0.5:
            sys.exit(f'--at {want}: 가장 가까운 콜백이 {t:.2f}s로 0.5s 넘게 떨어져 있다')
        # 장애물은 **p3_shadow가 로그한 것**을 쓴다. 노드는 buildGuardedObstacles()로 면별
        # 불확실성 팽창을 먼저 적용한 뒤 그 결과를 evaluateP3Shadow에 넣고, p3_shadow는 바로
        # 그 값을 로그한다(local_planner_node.cpp:2255, snapshot.maneuver.obstacles).
        # 플래너 자신은 가드를 적용하지 않으므로 이 값을 그대로 넣는 것이 충실한 재현이다.
        # /confirmed_static_obs 원본을 넣으면 팽창이 빠져 오프라인이 운영보다 관대해진다 —
        # 실제로 그렇게 만든 첫 시나리오는 백에서 실패한 장면이 오프라인에서 성공했다.
        obstacles = shadow['obstacles']
        lines.append('FRAME\t{}\t0\t{!r}\t{!r}\t{!r}\t{}'.format(
            int(t * 1e9), shadow['ego_s'], shadow['ego_d'], shadow['ego_speed_mps'],
            len(obstacles)))
        for o in obstacles:
            # s_var/d_var/size는 플래너가 쓰지 않는다(가드는 이미 반영돼 있다).
            center = 0.5 * (o['s_start'] + o['s_end'])
            lines.append('O\t{}\t{!r}\t{!r}\t{!r}\t{!r}\t{!r}\t{!r}\t{!r}\t{!r}'.format(
                o['id'], center, o['s_start'], o['s_end'], o['d_right'], o['d_left'],
                o['d_left'] - o['d_right'], 0.0, 0.0))
        lines.append('END_FRAME')
        print(f'  프레임 t={t:.2f}  ego s={shadow["ego_s"]:.2f} d={shadow["ego_d"]:+.3f} '
              f'v={shadow["ego_speed_mps"]:.2f}  장애물 {len(obstacles)}개  '
              f'기각={shadow["fresh_rejection"]}')
    lines.append('END_STREAM')

    pathlib.Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    pathlib.Path(args.out).write_text('\n'.join(lines) + '\n')
    print(f'{args.out} 저장 ({len(lines)}줄)')


if __name__ == '__main__':
    main()
