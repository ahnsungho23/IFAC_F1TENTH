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
local_planning YAML의 종방향 한계표가 velocity_limits.csv와 같은지 검사한다.

플래너와 오프라인 라인 생성기가 **같은 차량 모델**을 쓰게 하는 것이 이 표의 목적이므로,
둘이 갈라지면 목적 자체가 사라진다. VESC를 다시 재면 csv만 고치고 이 테스트를 돌려
YAML을 맞추면 된다.

⚠️ 횡가속 열(max_lateral_accel)은 **일부러 검사하지 않는다**. csv는 저속에서 9~10 m/s²를
허용하지만 2026-08-19 실차 실측 달성치는 p90 6.71 m/s²였고, 플래너 캡은 그 실측을 근거로
7.0/6.5를 유지한다. csv의 그 열은 라인 생성기의 가정이지 플래너가 따라야 할 값이 아니다.
"""
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
# test/ -> local_planning/ -> src/ -> <repo>
REPO = HERE.parents[2]
CSV = REPO / 'offline_trajectory_generator' / 'config' / 'velocity_limits.csv'
YAMLS = [
    HERE.parent / 'config' / 'local_planning.yaml',
    HERE.parent / 'config' / 'local_planning_sim.yaml',
]
TOLERANCE = 1e-9
# YAML 키 -> csv 열 인덱스 (0=speed, 1=max_accel, 2=max_decel, 3=max_lateral_accel)
COLUMNS = {
    'avoidance_velocity_limit_speed_bins_mps': 0,
    'avoidance_velocity_limit_accel_mps2': 1,
    'avoidance_velocity_limit_decel_mps2': 2,
}


def read_csv(path):
    rows = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        rows.append([float(x) for x in line.split(',')])
    return rows


def read_array(text, key):
    match = re.search(r'^\s*' + re.escape(key) + r':\s*\[([^\]]*)\]\s*$', text, re.M)
    if match is None:
        return None
    return [float(x) for x in match.group(1).split(',') if x.strip()]


def test_longitudinal_limits_match_velocity_limits_csv():
    assert main() == 0


def main():
    if not CSV.exists():
        print(f'FAIL: velocity_limits.csv를 찾지 못했습니다: {CSV}')
        return 1
    rows = read_csv(CSV)
    problems = []
    for yaml_path in YAMLS:
        text = yaml_path.read_text()
        for key, column in COLUMNS.items():
            values = read_array(text, key)
            expected = [row[column] for row in rows]
            if values is None:
                problems.append(f'{yaml_path.name}: {key} 가 없습니다')
                continue
            if len(values) != len(expected):
                problems.append(
                    f'{yaml_path.name}: {key} 길이 {len(values)} != csv 행 수 {len(expected)}')
                continue
            for index, (got, want) in enumerate(zip(values, expected)):
                if abs(got - want) > TOLERANCE:
                    problems.append(
                        f'{yaml_path.name}: {key}[{index}] = {got} != csv {want}')
    if problems:
        print('FAIL: YAML 종방향 한계표가 velocity_limits.csv와 다릅니다')
        for problem in problems:
            print(f'  - {problem}')
        return 1
    print(f'OK: 종방향 한계표가 csv({len(rows)}행)와 일치합니다 '
          f'({", ".join(p.name for p in YAMLS)})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
