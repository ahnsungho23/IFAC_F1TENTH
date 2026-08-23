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
local_planning이 자신의 차량 한계 CSV와 동기화되었는지 검사한다.

speed/accel/decel은 ``config/local_planning_velocity_limits.csv``를 정확히 따른다.
lateral은 CSV를 기본 표로 쓰되 배포된 control의 ``max_lateral_accel``보다
큰 권한을 계획하지 않도록 ``min(CSV, control)``로 합성하며, 그 계약은
``test_control_contract_match.py``가 따로 고정한다.
"""
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
CSV = HERE.parent / 'config' / 'local_planning_velocity_limits.csv'
YAMLS = [
    HERE.parent / 'config' / 'local_planning.yaml',
    HERE.parent / 'config' / 'local_planning_sim.yaml',
]
NODE = HERE.parent / 'src' / 'local_planner_node.cpp'
HARNESS = HERE / 'stuck_case_harness.cpp'
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


def validate_rows(rows):
    problems = []
    if not rows:
        return ['csv에 데이터 행이 없습니다']
    for index, row in enumerate(rows):
        if len(row) != 4:
            problems.append(f'csv {index + 1}번째 데이터 행의 열 수 {len(row)} != 4')
            continue
        if index > 0 and row[0] <= rows[index - 1][0]:
            problems.append(f'csv speed[{index}]={row[0]} 이 엄격히 증가하지 않습니다')
        if any(value <= 0.0 for value in row[1:]):
            problems.append(f'csv {index + 1}번째 데이터 행에 0 이하 한계가 있습니다')
    return problems


def read_array(text, key):
    match = re.search(r'^\s*' + re.escape(key) + r':\s*\[([^\]]*)\]\s*$', text, re.M)
    if match is None:
        return None
    return [float(x) for x in match.group(1).split(',') if x.strip()]


def read_declared_array(text, key):
    match = re.search(
        r'"' + re.escape(key) + r'"\s*,\s*std::vector<double>\s*\{([^}]*)\}', text, re.S)
    if match is None:
        return None
    return [float(x) for x in match.group(1).split(',') if x.strip()]


def read_harness_array(text, key):
    match = re.search(r'p\.' + re.escape(key) + r'\s*=\s*\{([^}]*)\}', text, re.S)
    if match is None:
        return None
    return [float(x) for x in match.group(1).split(',') if x.strip()]


def compare_columns(label, text, reader, rows, problems):
    for key, column in COLUMNS.items():
        values = reader(text, key)
        expected = [row[column] for row in rows]
        if values is None:
            problems.append(f'{label}: {key} 가 없습니다')
            continue
        if len(values) != len(expected):
            problems.append(f'{label}: {key} 길이 {len(values)} != csv 행 수 {len(expected)}')
            continue
        for index, (got, want) in enumerate(zip(values, expected)):
            if abs(got - want) > TOLERANCE:
                problems.append(f'{label}: {key}[{index}] = {got} != csv {want}')


def test_longitudinal_limits_match_velocity_limits_csv():
    assert main() == 0


def main():
    if not CSV.exists():
        print(f'FAIL: velocity_limits.csv를 찾지 못했습니다: {CSV}')
        return 1
    rows = read_csv(CSV)
    problems = validate_rows(rows)
    if problems:
        print('FAIL: local_planning 전용 CSV 형식이 잘못됐습니다')
        for problem in problems:
            print(f'  - {problem}')
        return 1
    for yaml_path in YAMLS:
        compare_columns(yaml_path.name, yaml_path.read_text(), read_array, rows, problems)
    compare_columns('C++ declare', NODE.read_text(), read_declared_array, rows, problems)
    compare_columns('stuck_case_harness', HARNESS.read_text(), read_harness_array, rows, problems)
    if problems:
        print('FAIL: local_planning 종방향 한계표가 전용 CSV와 다릅니다')
        for problem in problems:
            print(f'  - {problem}')
        return 1
    print(f'OK: 종방향 한계표가 local csv({len(rows)}행)와 일치합니다 '
          f'({", ".join(p.name for p in YAMLS)}, C++ declare, stuck_case_harness)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
