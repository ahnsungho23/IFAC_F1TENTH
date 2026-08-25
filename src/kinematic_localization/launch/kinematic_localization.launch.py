import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ---------------------------------------------------------------------------
# launch 인자를 "안 넘겼다"는 뜻의 표식.
#
# ros2 의 parameters=[yaml, {...}] 는 **뒤에 오는 dict 가 항상 이긴다**. 예전 이 파일은
# 인자를 넘겼든 안 넘겼든 12개 키를 전부 dict 에 넣었기 때문에, DeclareLaunchArgument 의
# default_value 가 YAML 을 무조건 덮어썼다 — YAML 을 고쳐도 노드에 도달하지 않았다.
#
#   실측(2026-08-24): YAML 에 tilt_compensation_enable: true / odom_window_at_scan_end: false /
#   map_name: "map" 을 적어 뒀는데, 노드 기동 로그는 tilt=false 였고 §9 는 켜져 있었다.
#   운영자는 3일간 "고친 줄 알고" 주행했다. 16:31/16:48 백이 그 상태로 기록됐다.
#
# 그래서 인자 기본값을 이 표식으로 두고, **표식과 다른 값이 실제로 들어온 인자만**
# override dict 에 올린다. 안 넘긴 인자는 dict 에 아예 없으므로 YAML 이 그대로 산다.
# 빈 문자열은 여전히 유효한 "명시적" 값이다 (map_name:= 는 순수 오돔).
#
# 인자를 새로 추가할 때는 OVERRIDABLE 에 변환기와 함께 등록할 것. parameters=[...] 의
# dict 에 키를 직접 넣지 말 것.
# ---------------------------------------------------------------------------
KEEP_YAML = '__yaml__'


def _as_bool(name, raw):
    # bool('false') 는 파이썬에서 True 다 — 절대 bool() 로 캐스팅하지 말 것.
    low = raw.strip().lower()
    if low in ('1', 'true', 'yes', 'on'):
        return True
    if low in ('0', 'false', 'no', 'off'):
        return False
    raise RuntimeError(
        f"launch 인자 '{name}' 는 bool 이어야 합니다 (받은 값: '{raw}').\n"
        "  쓸 수 있는 값: true/false, 1/0, yes/no, on/off")


def _as_float(name, raw):
    try:
        return float(raw)
    except ValueError:
        raise RuntimeError(
            f"launch 인자 '{name}' 는 실수여야 합니다 (받은 값: '{raw}').")


def _as_str(_name, raw):
    return raw


# 인자 이름 -> 값 변환기. 여기 등록된 것만 YAML 을 덮어쓸 수 있다.
OVERRIDABLE = {
    'map_name': _as_str,
    'slam_mode': _as_bool,
    'map_output_file': _as_str,
    'use_sim_time': _as_bool,
    'fast_corner_free_mode': _as_bool,
    'corner_yaw_rate_thresh': _as_float,
    'corner_speed_thresh': _as_float,
    'corner_lat_accel_thresh': _as_float,
    'odom_window_at_scan_end': _as_bool,
    'scan_stamp_convention': _as_str,
    'tilt_compensation_enable': _as_bool,
    'roll_gradient_rad_per_mps2': _as_float,
    # 시뮬(gym) 프레임/토픽 오버라이드 (2026-08-26). gym 은 base_link/odom 프레임도
    # /odom 토픽도 없고 map -> ego_racecar/base_link TF 를 직접 발행한다. 이 다섯 개를
    # 런치에서 넘길 수 없어서, 시뮬에서는 ProcessScan() 의 base->laser extrinsic lookup 이
    # 매 스캔 실패하고 곧바로 return 했다 — 포즈 발행은 그 함수 안에 한 군데뿐이라
    # /pf/pose/odom 이 0 Hz 였고 하류(frenet/local planner/FSM)가 통째로 멈췄다.
    'lidar_topic': _as_str,
    'odom_topic': _as_str,
    'odom_frame': _as_str,
    'base_frame': _as_str,
    'publish_map_odom_tf': _as_bool,
}


def _localization_node(context, *_args, **_kwargs):
    # launch_ros는 존재하지 않는 parameter file을 조용히 건너뛸 수 있다. 그러면 노드는
    # gate/source downsample/smoothing이 코드 기본값인 채 정상 기동하므로, launch 단계에서
    # 깨진 symlink까지 포함해 즉시 차단한다. 파일 내용의 세대 일치는 노드의
    # config_schema_version 가드가 한 번 더 확인한다.
    config = os.path.join(
        get_package_share_directory('kinematic_localization'),
        'config',
        'kinematic_localization.yaml',
    )
    if not os.path.isfile(config):
        raise RuntimeError(
            f'필수 파라미터 파일이 없거나 symlink가 깨졌습니다: {config}\n'
            'src/kinematic_localization/config와 install overlay를 복구·재빌드한 뒤 '
            '다시 실행하세요.')

    # 실제로 넘어온 인자만 모은다. 나머지는 YAML 이 그대로 이긴다.
    overrides = {}
    for name, cast in OVERRIDABLE.items():
        raw = LaunchConfiguration(name).perform(context)
        if raw == KEEP_YAML:
            continue
        overrides[name] = cast(name, raw)

    # 무엇이 YAML 을 덮었는지 기동 로그에 남긴다. "조용한 override" 를 없애는 게 목적이라
    # 덮은 게 없을 때도 한 줄 찍는다.
    if overrides:
        summary = ', '.join(f'{k}={v}' for k, v in sorted(overrides.items()))
        print(f'[kinematic_localization.launch] YAML override (launch 인자): {summary}')
    else:
        print('[kinematic_localization.launch] launch 인자 override 없음 — 전부 YAML 값')

    return [Node(
        package='kinematic_localization',
        executable='localization_node',
        name='kinematic_localization',
        output='screen',
        parameters=[config, overrides],
    )]


def generate_launch_description():
    keep = f'기본 {KEEP_YAML} = YAML 값을 그대로 쓴다.'
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value=KEEP_YAML,
            description='Frozen map name: maps/<map_name>.kissmap in the package share '
                        '(an absolute path is used as-is, empty = pure odometry). '
                        'Ignored when slam_mode is true. ' + keep),
        DeclareLaunchArgument(
            'slam_mode',
            default_value=KEEP_YAML,
            description='Online SLAM mode: no frozen map, auto-init at identity on the '
                        'first scan, save the accumulated map via the ~/save_map service '
                        'or on shutdown. ' + keep),
        DeclareLaunchArgument(
            'map_output_file',
            default_value=KEEP_YAML,
            description='Output .kissmap path for slam_mode (relative paths resolve '
                        'against the process cwd). ' + keep),
        DeclareLaunchArgument('use_sim_time', default_value=KEEP_YAML, description=keep),
        # --- 시뮬(gym) 프레임/토픽 ---------------------------------------------------
        # gym 에서 띄울 때의 값:
        #   base_frame:=ego_racecar/base_link  odom_topic:=/ego_racecar/odom
        #   publish_map_odom_tf:=false   (gym 이 map -> ego_racecar/base_link 를 이미 발행)
        # 실차에서는 넘기지 말 것 — YAML(base_link, /odom, true)이 그대로 산다.
        DeclareLaunchArgument('lidar_topic', default_value=KEEP_YAML, description=keep),
        DeclareLaunchArgument('odom_topic', default_value=KEEP_YAML, description=keep),
        DeclareLaunchArgument('odom_frame', default_value=KEEP_YAML, description=keep),
        DeclareLaunchArgument(
            'base_frame', default_value=KEEP_YAML,
            description='스캔 프레임과 TF 로 이어져 있어야 한다. gym 은 '
                        'ego_racecar/base_link. ' + keep),
        DeclareLaunchArgument(
            'publish_map_odom_tf', default_value=KEEP_YAML,
            description='map -> odom TF 발행. gym 처럼 다른 발행자가 이미 base 를 '
                        '매달고 있으면 false. ' + keep),
        # §8 fast-corner free mode. YAML을 고치지 않고 실차에서 A/B 하려고 인자로 뺐다.
        # 파라미터는 생성자에서 한 번만 읽으므로 `ros2 param set`으로는 못 바꾼다 —
        # 반드시 기동 시점에 넘겨야 한다.
        DeclareLaunchArgument(
            'fast_corner_free_mode', default_value=KEEP_YAML,
            description='고속 코너 프레임에서 Omega/횡하한/반복상한·수렴/게이트/스무딩을 '
                        '모두 우회한다. 실차 미검증. ' + keep),
        DeclareLaunchArgument('corner_yaw_rate_thresh', default_value=KEEP_YAML,
                              description=keep),
        DeclareLaunchArgument('corner_speed_thresh', default_value=KEEP_YAML,
                              description=keep),
        DeclareLaunchArgument('corner_lat_accel_thresh', default_value=KEEP_YAML,
                              description=keep),
        # §9 스캔 끝-시각 동기화. 2026-08-22 실차 A/B/A 로 검증돼 YAML 기본이 true 였다
        # (자세 보정 0.83 -> 0.43 -> 0.87 deg, 양쪽에서 돌아옴 = -49%).
        # 되돌려서 비교하려면 odom_window_at_scan_end:=false 로 띄운다.
        DeclareLaunchArgument(
            'odom_window_at_scan_end', default_value=KEEP_YAML,
            description='등록 prior 와 발행 스탬프를 스캔 스윕 끝에 맞춘다 (OdomAtStrict + '
                        '스캔 FIFO). ' + keep),
        DeclareLaunchArgument(
            'scan_stamp_convention', default_value=KEEP_YAML,
            description="/scan 헤더가 스윕의 'begin'(이 차, 재생 A/B 로 판정) 인지 'end' 인지. "
                        'begin 이면 scan_end = header + (N-1)*time_increment. ' + keep),
        # 섀시 롤 보정 (2026-08-22 transition_global 도입). 실차 A/B 를 위해 인자로 뺐다.
        #
        # 🔴 roll_gradient 0.0270 은 이 차의 값이 아닐 가능성이 높다. 어젯밤 백 4개(자율
        #    250 s)로 두 가지 독립 검증을 했고 둘 다 반박한다:
        #      ① ICP 잔차가 |a_lat| 에 평평하다 (0~1 에서 0.2381, 7~20 에서 0.2353,
        #         회귀 기울기 -0.0025). 보정 안 된 9.3도 롤이 있다면 나올 열화가 없다.
        #      ② 물리적으로 배제된다 — 라이다 높이 0.11 m 에서 롤 φ 면 옆빔이
        #         0.11/sin(φ) 에서 바닥을 친다. 0.0270 → φ=9.3도 → 0.68 m. 실측 옆빔
        #         중앙거리는 고 a_lat 에서 좌 1.350 / 우 1.126 m 로 한쪽도 안 눌린다.
        #         여기서 φ < 5.6도, 즉 gradient < 0.0163 이 나온다.
        #    0.0270 은 edge_test 섀시의 9.4도와 일치하는 값이다. 실차 A/B 로 판정할 것.
        DeclareLaunchArgument(
            'tilt_compensation_enable', default_value=KEEP_YAML,
            description='섀시 롤 보정. 위치추정이 나빠지면 false 로 먼저 끄고 볼 것. ' + keep),
        DeclareLaunchArgument(
            'roll_gradient_rad_per_mps2', default_value=KEEP_YAML,
            description='roll = 이 값 * a_lat [rad/(m/s^2)]. 이 차 실측 상한은 0.0163. ' + keep),
        OpaqueFunction(function=_localization_node),
    ])
