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
f1tenth_control 변경이 local_planning에 미반영되는 것을 빌드 단계에서 막는다.

이 검사는 조향/횡가속 제어 계약을 다룬다. planner lateral은
``min(local_planning_velocity_limits.csv, control max_lateral_accel)``이다. 종방향
accel/decel은 local planning 전용 CSV 계약이므로 control 스칼라와 강제로 맞추지 않는다.
"""

import math
import pathlib
import re


HERE = pathlib.Path(__file__).resolve().parent
LOCAL = HERE.parent
SRC = LOCAL.parent
CONTROL = SRC / 'f1tenth_control' / 'launch'
REAL = CONTROL / 'control_real.launch.py'
SIM = CONTROL / 'control_sim.launch.py'
COMMON = CONTROL / '_control_common.py'
NODE = LOCAL / 'src' / 'local_planner_node.cpp'
REAL_YAML = LOCAL / 'config' / 'local_planning.yaml'
SIM_YAML = LOCAL / 'config' / 'local_planning_sim.yaml'
LIMITS_CSV = LOCAL / 'config' / 'local_planning_velocity_limits.csv'
TOL = 1.0e-9


def launch_default(text, name):
    match = re.search(
        rf"['\"]{re.escape(name)}['\"]\s*,\s*default_value\s*=\s*['\"]([^'\"]+)",
        text,
    )
    assert match, f'launch default missing: {name}'
    return float(match.group(1))


def common_literal(text, name):
    match = re.search(rf"['\"]{re.escape(name)}['\"]\s*:\s*([-+0-9.eE]+)", text)
    assert match, f'control common literal missing: {name}'
    return float(match.group(1))


def named_literal(text, name):
    match = re.search(rf'\b{re.escape(name)}\s*=\s*([-+0-9.eE]+)', text)
    assert match, f'named literal missing: {name}'
    return float(match.group(1))


def yaml_scalar(text, name):
    match = re.search(rf'^\s+{re.escape(name)}:\s*([-+0-9.eE]+)\s*$', text, re.M)
    assert match, f'local YAML scalar missing: {name}'
    return float(match.group(1))


def yaml_vector(text, name):
    match = re.search(rf'^\s+{re.escape(name)}:\s*\[([^\]]+)\]\s*$', text, re.M)
    assert match, f'local YAML vector missing: {name}'
    return [float(value.strip()) for value in match.group(1).split(',')]


def node_default(text, name):
    match = re.search(
        rf'declare_parameter<double>\(\s*"{re.escape(name)}"\s*,\s*([-+0-9.eE]+)\s*\)',
        text,
        re.S,
    )
    assert match, f'local C++ declare default missing: {name}'
    return float(match.group(1))


def node_vector(text, name):
    match = re.search(
        rf'"{re.escape(name)}"\s*,\s*std::vector<double>\s*\{{([^}}]+)\}}', text, re.S)
    assert match, f'local C++ declare vector missing: {name}'
    return [float(value.strip()) for value in match.group(1).split(',')]


def csv_lateral_limits():
    values = []
    for raw_line in LIMITS_CSV.read_text().splitlines():
        line = raw_line.strip()
        if not line or line.startswith('#'):
            continue
        columns = [float(value) for value in line.split(',')]
        assert len(columns) == 4, f'local limits csv must have 4 columns: {raw_line}'
        values.append(columns[3])
    return values


def assert_close(actual, expected, label):
    assert math.isclose(actual, expected, rel_tol=0.0, abs_tol=TOL), (
        f'{label}: {actual!r} != {expected!r}. control 변경 시 local_planning의 '
        'C++ declare/YAML/문서를 같은 통합에서 반드시 동기화하세요.'
    )


def test_control_contract_matches_local_planning():
    real_text = REAL.read_text()
    sim_text = SIM.read_text()
    common_text = COMMON.read_text()
    node_text = NODE.read_text()
    real_yaml = REAL_YAML.read_text()
    sim_yaml = SIM_YAML.read_text()

    real_contract = {
        'control_wheelbase_m': common_literal(common_text, 'wheelbase'),
        'control_max_steering_left_rad': launch_default(real_text, 'max_steering_left'),
        'control_max_steering_right_rad': launch_default(real_text, 'max_steering_right'),
        'control_understeer_gradient_left_rad_per_mps2':
            launch_default(common_text, 'understeer_gradient_left'),
        'control_understeer_gradient_right_rad_per_mps2':
            launch_default(common_text, 'understeer_gradient_right'),
        'control_max_steering_rate_radps': launch_default(common_text, 'max_steering_rate'),
    }
    for name, control_value in real_contract.items():
        assert_close(yaml_scalar(real_yaml, name), control_value, f'operational YAML {name}')
        assert_close(node_default(node_text, name), control_value, f'C++ default {name}')

    # 시뮬은 운영 차량과 wheelbase/K_us/rate는 공유하지만 차량 모델이 좌우 대칭이라
    # 각도는 control_sim.launch.py의 값을 따로 대조한다.
    sim_contract = dict(real_contract)
    sim_contract['control_max_steering_left_rad'] = named_literal(sim_text, 'max_steering_left')
    sim_contract['control_max_steering_right_rad'] = named_literal(sim_text, 'max_steering_right')
    for name, control_value in sim_contract.items():
        assert_close(yaml_scalar(sim_yaml, name), control_value, f'simulation YAML {name}')

    max_lateral_accel = launch_default(real_text, 'max_lateral_accel')
    csv_lateral = csv_lateral_limits()
    expected_lateral = [min(value, max_lateral_accel) for value in csv_lateral]
    lateral_targets = {
        '운영 YAML': yaml_vector(real_yaml, 'avoidance_velocity_limit_lateral_accel_mps2'),
        '시뮬 YAML': yaml_vector(sim_yaml, 'avoidance_velocity_limit_lateral_accel_mps2'),
        'C++ declare': node_vector(node_text, 'avoidance_velocity_limit_lateral_accel_mps2'),
    }
    for label, lateral in lateral_targets.items():
        assert len(lateral) == len(expected_lateral)
        for index, (value, expected) in enumerate(zip(lateral, expected_lateral)):
            assert_close(value, expected, f'{label} planner lateral row {index}')

    # Legacy 절대 곡률 상한이 좌조향 물리 한계보다 남아야 부호별 min 합성이
    # 좌코너에서도 제어기가 낼 수 없는 곡률을 허용하지 않는다.
    legacy_curvature = yaml_scalar(real_yaml, 'maximum_curvature_radpm')
    left_physical = math.tan(real_contract['control_max_steering_left_rad']) / real_contract[
        'control_wheelbase_m'
    ]
    assert legacy_curvature <= left_physical + TOL
