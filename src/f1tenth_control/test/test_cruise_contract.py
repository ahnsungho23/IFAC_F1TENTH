"""CRUISE 파이프라인의 패키지 간 계약 검사.

CRUISE는 세 패키지에 걸쳐 있다: 상태머신이 간섭을 판정해 STATE_CRUISE 를 내고,
cruise_controller_node 가 그 상태에서만 /cruise_speed_limit 을 내며, control_map_node 가
그 상한을 종방향 목표속도에 씌운다. 세 패키지의 YAML/런치 기본값이 서로 어긋나도 각
패키지의 단위 시험은 전부 통과하므로, 그 어긋남을 잡는 것은 이 파일뿐이다.

여기서 검사하는 것은 **정적 구성의 필요조건**이다. 실제 유지 가능성(런타임 p*)은
공분산·상대 횡위치·상대속도에 의존하므로 실주행으로 확인한다
(state_machine/docs/cruise_pipeline_port_proposal.md §9).
"""

import re
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
CRUISE_YAML = REPO_ROOT / 'src/f1tenth_control/config/cruise_controller.yaml'
STATE_MACHINE_YAML = REPO_ROOT / 'src/state_machine/config/state_machine.yaml'
CONTROL_COMMON = REPO_ROOT / 'src/f1tenth_control/launch/_control_common.py'


def _ros_params(path):
    """`<node>: ros__parameters:` 아래 값을 평평한 dict 로 돌려준다."""
    with open(path, encoding='utf-8') as handle:
        tree = yaml.safe_load(handle)
    params = {}
    for node in tree.values():
        if isinstance(node, dict) and 'ros__parameters' in node:
            params.update(node['ros__parameters'])
    return params


def _launch_defaults(path):
    """DeclareLaunchArgument('<이름>', default_value='<값>') 를 긁어 dict 로 돌려준다.

    런치 인자가 YAML 을 덮어쓰는 구조(parameters=[config_file, {오버라이드}])라, 실제
    기동값은 런치 기본값이다. 그래서 코드를 import 하지 않고 원문을 파싱한다
    (import 하면 launch/launch_ros 의존이 시험에 끌려온다).
    """
    text = path.read_text(encoding='utf-8')
    pattern = re.compile(
        r"""['"](?P<name>\w+)['"]\s*,\s*default_value\s*=\s*['"](?P<value>[^'"]*)['"]""")
    return {m.group('name'): m.group('value') for m in pattern.finditer(text)}


def _as_float(text):
    return float(text)


def _as_bool(text):
    return text.strip().lower() == 'true'


@pytest.fixture(scope='module')
def cruise():
    return _ros_params(CRUISE_YAML)


@pytest.fixture(scope='module')
def state_machine():
    return _ros_params(STATE_MACHINE_YAML)


@pytest.fixture(scope='module')
def launch():
    return _launch_defaults(CONTROL_COMMON)


def _desired_gap(cruise, launch):
    """고정 거리 모드의 목표 간격. 시간 간격 모드면 상한(max_desired_gap)을 돌려준다."""
    distance_mode = _as_bool(launch.get('trailing_mode_distance',
                                        str(cruise['trailing_mode_distance'])))
    trailing_gap = _as_float(launch.get('trailing_gap', cruise['trailing_gap']))
    if distance_mode:
        return trailing_gap
    max_desired = _as_float(launch.get('max_desired_gap', cruise['max_desired_gap']))
    assert max_desired > 0.0, (
        '시간 간격 모드에서는 목표 간격이 속도에 비례해 무한히 커지므로 '
        'max_desired_gap 으로 상한을 두어야 한다 (0 = 무제한)')
    return max_desired


def test_launch_and_yaml_defaults_agree(cruise, launch):
    """런치 기본값이 YAML 을 덮어쓰므로, 둘이 다르면 YAML 은 죽은 설정이 된다."""
    for name in ('trailing_gap', 'minimum_gap', 'max_desired_gap',
                 'opponent_timeout', 'state_timeout', 'blind_trailing_speed',
                 'emergency_stop_distance'):
        if name not in launch or name not in cruise:
            continue
        assert _as_float(launch[name]) == pytest.approx(float(cruise[name])), (
            f'{name}: 런치 기본값 {launch[name]} != YAML {cruise[name]} — '
            '런치가 YAML 을 덮어쓰므로 YAML 쪽이 무효가 된다')


def test_desired_gap_within_interference_distance(cruise, state_machine, launch):
    """불변식: cruise 의 목표 간격 <= 상태머신의 간섭 판정 거리.

    목표 간격이 더 크면, cruise 가 유지하려는 바로 그 간격에서 상태머신이 "간섭 없음"으로
    판정해 CRUISE 를 빠져나간다. 상한이 풀려 가속 -> 재진입 -> 급감속의 리밋사이클이 된다.
    """
    desired_gap = _desired_gap(cruise, launch)
    interference_distance = float(state_machine['interference_distance_m'])
    assert desired_gap <= interference_distance, (
        f'목표 간격 {desired_gap} m > 간섭 판정 거리 {interference_distance} m — '
        'GLOBAL/CRUISE 리밋사이클 조건')


def test_equilibrium_latch_margin(cruise, state_machine, launch):
    """평형에서 래치가 유지될 수 있는 구성인지(필요조건).

    상태머신의 종방향 확률은 P_long = P(0 <= gap <= interference_distance_m) 이다
    (interference_predicate.cpp). 목표 간격이 그 경계와 같으면 평형에서 확률질량이 정확히
    반으로 갈려 P_long -> 0.5 이고, p* = P_lat * P_long <= 0.5 가 된다. 따라서 등호 구성에서는
    래치 하단 p_off 가 0.5 보다 **작아야** 래치 유지가 가능하다.

    (p_off < 0.5 는 필요조건일 뿐이다. 실제로는 p* = P_lat * 0.5 > p_off 이어야 하므로
     P_lat > 2*p_off 가 요구되는데, P_lat 은 런타임 횡방향 불확실성에 달려 있어 구성만으로는
     판정할 수 없다 — 제안서 §9 의 실주행 판정 항목.)
    """
    desired_gap = _desired_gap(cruise, launch)
    interference_distance = float(state_machine['interference_distance_m'])
    p_off = float(state_machine['interference_p_off'])

    if desired_gap == pytest.approx(interference_distance):
        assert p_off < 0.5, (
            f'목표 간격이 간섭 거리와 같은데(=={desired_gap} m) p_off={p_off} >= 0.5 다. '
            '평형에서 P_long -> 0.5 이므로 래치가 수학적으로 유지될 수 없다')


def test_timeout_relations(cruise, state_machine, launch):
    """상위 노드의 stale 판정이 하위 노드보다 먼저 풀리면 보호 공백이 생긴다."""
    opponent_timeout = _as_float(launch.get('opponent_timeout', cruise['opponent_timeout']))
    opponent_stale = float(state_machine['opponent_stale_timeout_sec'])
    assert opponent_timeout <= opponent_stale, (
        f'cruise opponent_timeout {opponent_timeout} s > 상태머신 '
        f'opponent_stale_timeout_sec {opponent_stale} s — 상태머신이 CRUISE 를 유지하는 동안 '
        'cruise 가 상대차를 신선하다고 오판하는 구간이 생긴다')

    # /state 하트비트: 상태머신 발행 주기의 2배 이상을 기다려야 한 프레임 유실에 흔들리지 않는다.
    state_timeout = _as_float(launch.get('state_timeout', cruise['state_timeout']))
    fsm_period = 1.0 / float(state_machine['publish_rate_hz'])
    assert state_timeout >= 2.0 * fsm_period, (
        f'cruise state_timeout {state_timeout} s < 상태머신 발행주기 {fsm_period:.3f} s 의 2배')

    # /cruise_speed_limit 감시: cruise 발행 주기의 2배 이상.
    limit_timeout = _as_float(launch['cruise_speed_limit_timeout'])
    cruise_period = 1.0 / float(cruise['publish_rate_hz'])
    assert limit_timeout >= 2.0 * cruise_period, (
        f'control_map_node cruise_speed_limit_timeout {limit_timeout} s < '
        f'cruise 발행주기 {cruise_period:.3f} s 의 2배')


def test_emergency_distance_below_desired_gap(cruise, launch):
    """비상정지 거리가 목표 간격 이상이면 평형에서 상시 정지 명령이 나간다."""
    desired_gap = _desired_gap(cruise, launch)
    emergency = _as_float(launch.get('emergency_stop_distance',
                                     cruise['emergency_stop_distance']))
    assert emergency < desired_gap, (
        f'emergency_stop_distance {emergency} m >= 목표 간격 {desired_gap} m')
