#!/usr/bin/env zsh
# Two-car cruise scenario launcher for ROS 2 Jazzy + f1sim_C.

SCRIPT_PATH="${0:A}"
MODE="${1:-}"

export ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-49}"
export CRUISE_OPP_SPEED_SCALE="${CRUISE_OPP_SPEED_SCALE:-0.8}"
export CRUISE_USE_OBSTACLES="${CRUISE_USE_OBSTACLES:-0}"
export CRUISE_OPEN_GUI="${CRUISE_OPEN_GUI:-0}"
export CRUISE_SKIP_BUILD="${CRUISE_SKIP_BUILD:-0}"
export CRUISE_RVIZ="${CRUISE_RVIZ:-true}"

IFAC_WS="${IFAC_WS:-$HOME/2026_IFAC}"
F1SIM_WS="${F1SIM_WS:-$HOME/f1sim_C}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.zsh"
IFAC_MAP_YAML="$IFAC_WS/src/monte_carlo_localization/maps/map.yaml"
WAYPOINTS_JSON="$IFAC_WS/offline_trajectory_generator/output/map/global_waypoints.json"
F1SIM_MAP_DIR="$F1SIM_WS/f1tenth_gym_ros/maps"
CRUISE_MAP_STEM="$F1SIM_MAP_DIR/cruise_map"
CRUISE_OBS_STEM="${CRUISE_MAP_STEM}_obs"
BUILD_WORKERS="${BUILD_WORKERS:-1}"

usage() {
  echo "사용법: $SCRIPT_PATH [--gui | --obstacles] [--speed 배율] [--no-build] [--no-rviz]"
  echo "  기본          현재 IFAC 기준 맵에서 2대 크루즈 시뮬레이션 실행"
  echo "  --gui         장애물 GUI에서 cruise_map을 편집·저장한 뒤 _obs 맵으로 실행"
  echo "  --obstacles   기존 cruise_map_obs 맵으로 실행(기준 맵 정합성 검사)"
  echo "  --speed N     상대차 센터라인 속도 배율(기본 0.8)"
  echo "  --no-build    실행 전 colcon build 생략"
  echo "  --no-rviz     RViz를 띄우지 않음"
}

fail() {
  echo -e "\033[1;31m[오류] $*\033[0m"
  return 1
}

source_ros_base() {
  [[ -r "$ROS_SETUP" ]] || fail "ROS 환경 파일이 없습니다: $ROS_SETUP" || return 1
  source "$ROS_SETUP"
}

source_stack() {
  source_ros_base || return 1
  [[ -r "$F1SIM_WS/install/setup.zsh" ]] || fail "f1sim_C가 빌드되지 않았습니다." || return 1
  [[ -r "$IFAC_WS/install/setup.zsh" ]] || fail "2026_IFAC가 빌드되지 않았습니다." || return 1
  source "$F1SIM_WS/install/setup.zsh"
  source "$IFAC_WS/install/setup.zsh"
}

prepare_cruise_map() {
  [[ -r "$IFAC_MAP_YAML" ]] || fail "기준 맵이 없습니다: $IFAC_MAP_YAML" || return 1
  mkdir -p "$F1SIM_MAP_DIR"
  python3 - "$IFAC_MAP_YAML" "${CRUISE_MAP_STEM}.yaml" <<'PY'
import os
import shutil
import sys
import yaml

source_yaml, target_yaml = map(os.path.abspath, sys.argv[1:])
with open(source_yaml, encoding="utf-8") as stream:
    metadata = yaml.safe_load(stream)
source_image = metadata["image"]
if not os.path.isabs(source_image):
    source_image = os.path.join(os.path.dirname(source_yaml), source_image)
image_ext = os.path.splitext(source_image)[1]
target_image = os.path.splitext(target_yaml)[0] + image_ext

if not os.path.exists(target_image) or open(source_image, "rb").read() != open(target_image, "rb").read():
    shutil.copy2(source_image, target_image)

metadata["image"] = os.path.basename(target_image)
rendered = yaml.safe_dump(metadata, sort_keys=False)
old = open(target_yaml, encoding="utf-8").read() if os.path.exists(target_yaml) else ""
if old != rendered:
    with open(target_yaml, "w", encoding="utf-8") as stream:
        stream.write(rendered)
PY
}

validate_obstacle_map() {
  [[ -r "${CRUISE_OBS_STEM}.yaml" ]] || fail "장애물 맵이 없습니다: ${CRUISE_OBS_STEM}.yaml" || return 1
  [[ -r "${CRUISE_OBS_STEM}.obstacles.yaml" ]] || fail "GUI sidecar가 없습니다: ${CRUISE_OBS_STEM}.obstacles.yaml" || return 1
  python3 - "${CRUISE_MAP_STEM}.yaml" "${CRUISE_OBS_STEM}.yaml" <<'PY'
import os
import sys
import yaml
from PIL import Image

def load(path):
    with open(path, encoding="utf-8") as stream:
        meta = yaml.safe_load(stream)
    image = meta["image"]
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(path), image)
    return meta, image

base, base_image = load(sys.argv[1])
obs, obs_image = load(sys.argv[2])
keys = ("resolution", "origin", "negate", "occupied_thresh", "free_thresh")
if any(base.get(key) != obs.get(key) for key in keys):
    raise SystemExit("장애물 맵의 해상도/원점이 현재 IFAC 기준 맵과 다릅니다. --gui로 다시 저장하세요.")
if Image.open(base_image).size != Image.open(obs_image).size:
    raise SystemExit("장애물 맵 이미지 크기가 현재 IFAC 기준 맵과 다릅니다. --gui로 다시 저장하세요.")
if os.path.getmtime(sys.argv[2]) < os.path.getmtime(sys.argv[1]):
    raise SystemExit("기준 맵이 장애물 맵보다 새롭습니다. --gui로 장애물 맵을 다시 저장하세요.")
PY
}

validate_runtime() {
  python3 - <<'PY' || return 1
import importlib

missing = []
for module in ("gym", "f110_gym", "transforms3d", "pyglet"):
    try:
        importlib.import_module(module)
    except Exception as exc:
        missing.append(f"{module}: {exc}")
if missing:
    raise SystemExit("f1sim Python 의존성이 없습니다:\n  " + "\n  ".join(missing))
PY

  python3 - "$IFAC_MAP_YAML" \
    "$IFAC_WS/offline_trajectory_generator/output/map/metadata.json" \
    "$IFAC_WS" <<'PY' || return 1
import json
import os
import sys
import yaml
from PIL import Image

with open(sys.argv[1], encoding="utf-8") as stream:
    current = yaml.safe_load(stream)
with open(sys.argv[2], encoding="utf-8") as stream:
    generated = json.load(stream)
current_image = current["image"]
if not os.path.isabs(current_image):
    current_image = os.path.join(os.path.dirname(sys.argv[1]), current_image)
generated_image = generated["map_image"]
if not os.path.isabs(generated_image):
    generated_image = os.path.join(sys.argv[3], generated_image)
same_geometry = (
    float(current["resolution"]) == float(generated["resolution"])
    and list(current["origin"]) == list(generated["origin"])
    and Image.open(current_image).size == Image.open(generated_image).size
)
waypoints_are_older = os.path.getmtime(sys.argv[2]) < os.path.getmtime(current_image)
if not same_geometry or waypoints_are_older:
    raise SystemExit(
        "global_waypoints가 현재 map에서 생성되지 않았습니다. "
        "global_path_gen GUI에서 현재 map.yaml을 열고 Save한 뒤 다시 실행하세요."
    )
PY

  python3 - "$IFAC_MAP_YAML" \
    "$IFAC_WS/install/particle_filter_cpp/share/particle_filter_cpp/maps/map.yaml" <<'PY'
import os
import sys
import yaml

def load(path):
    with open(path, encoding="utf-8") as stream:
        meta = yaml.safe_load(stream)
    image = meta["image"]
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(path), image)
    return meta, image

source, source_image = load(sys.argv[1])
installed, installed_image = load(sys.argv[2])
keys = ("resolution", "origin", "negate", "occupied_thresh", "free_thresh")
if any(source.get(key) != installed.get(key) for key in keys):
    raise SystemExit("MCL install 맵이 소스 맵과 다릅니다. 전체 colcon build가 필요합니다.")
if open(source_image, "rb").read() != open(installed_image, "rb").read():
    raise SystemExit("MCL install 이미지가 소스 맵과 다릅니다. 전체 colcon build가 필요합니다.")
PY
}

load_spawn_poses() {
  [[ -s "$WAYPOINTS_JSON" ]] || fail "waypoint JSON이 없습니다: $WAYPOINTS_JSON" || return 1
  local values
  values="$(python3 - "$WAYPOINTS_JSON" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    points = json.load(stream)["centerline_waypoints"]["wpnts"]
if len(points) < 2:
    raise SystemExit("centerline waypoint가 부족합니다.")
ego = points[0]
half_s = points[-1]["s_m"] * 0.5
opp = min(points, key=lambda point: abs(point["s_m"] - half_s))
print(ego["x_m"], ego["y_m"], ego["psi_rad"],
      opp["x_m"], opp["y_m"], opp["psi_rad"])
PY
)" || return 1
  read -r EGO_X EGO_Y EGO_YAW OPP_X OPP_Y OPP_YAW <<< "$values"
  export EGO_X EGO_Y EGO_YAW OPP_X OPP_Y OPP_YAW
}

open_next_tab() {
  local next_mode="$1"
  local title="$2"
  echo -e "\033[1;36m[안내] Enter를 누르면 ${title} 탭을 엽니다.\033[0m"
  read -r
  gnome-terminal --tab --title="$title" -- zsh "$SCRIPT_PATH" "$next_mode"
}

launch_then_next() {
  local next_mode="$1"
  local title="$2"
  shift 2
  "$@" &
  local launch_pid=$!
  open_next_tab "$next_mode" "$title"
  wait "$launch_pid"
}

if [[ "$MODE" != tab* ]]; then
  while (( $# > 0 )); do
    case "$1" in
      --gui)
        export CRUISE_OPEN_GUI=1 CRUISE_USE_OBSTACLES=1
        ;;
      --obstacles)
        export CRUISE_USE_OBSTACLES=1
        ;;
      --speed)
        shift
        (( $# > 0 )) || { usage; exit 2; }
        export CRUISE_OPP_SPEED_SCALE="$1"
        ;;
      --no-build)
        export CRUISE_SKIP_BUILD=1
        ;;
      --no-rviz)
        export CRUISE_RVIZ=false
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        fail "알 수 없는 옵션: $1"
        usage
        exit 2
        ;;
    esac
    shift
  done
  gnome-terminal --window --title="0: Cruise Setup" -- zsh "$SCRIPT_PATH" tab0
  exit 0
fi

load_spawn_poses || exec zsh

if [[ "$MODE" == "tab0" ]]; then
  source_ros_base || exec zsh
  prepare_cruise_map || exec zsh

  echo -e "\033[1;33m[Terminal 0] 기존 시뮬레이션 노드를 정리합니다.\033[0m"
  pkill -f "gym_bridge|particle_filter|global_trajectory_publisher_node|frenet_odom_node|local_planner_node|obstacle_detector_node|state_machine_node|cruise_controller_node|control_map_node|drive_source_selector|opponent_drive_controller|rviz2" 2>/dev/null
  ros2 daemon stop 2>/dev/null

  if [[ "$CRUISE_SKIP_BUILD" != "1" ]]; then
    echo -e "\033[1;36m[Terminal 0] f1sim_C와 2026_IFAC를 Jazzy로 빌드합니다.\033[0m"
    cd "$F1SIM_WS" || exec zsh
    CMAKE_BUILD_PARALLEL_LEVEL="$BUILD_WORKERS" colcon build --symlink-install --parallel-workers "$BUILD_WORKERS" || exec zsh
    source "$F1SIM_WS/install/setup.zsh"
    cd "$IFAC_WS" || exec zsh
    CMAKE_BUILD_PARALLEL_LEVEL="$BUILD_WORKERS" colcon build --symlink-install --parallel-workers "$BUILD_WORKERS" || exec zsh
  fi
  source_stack || exec zsh
  ros2 pkg prefix opponent_simulator >/dev/null 2>&1 || {
    fail "opponent_simulator가 빌드되지 않았습니다. --no-build 없이 다시 실행하세요."
    exec zsh
  }
  validate_runtime || exec zsh

  if [[ "$CRUISE_RVIZ" == "true" ]] && ! command -v xacro >/dev/null 2>&1; then
    echo -e "\033[1;33m[경고] ros-jazzy-xacro가 없어 RViz 차량 모델을 띄울 수 없습니다.\033[0m"
    echo -e "\033[1;33m[경고] 이번 실행은 headless로 계속합니다. 설치: sudo apt install ros-jazzy-xacro\033[0m"
    export CRUISE_RVIZ=false
  fi

  if [[ "$CRUISE_OPEN_GUI" == "1" ]]; then
    echo -e "\033[1;33m[GUI] cruise_map.yaml을 선택하고 장애물을 배치한 뒤 '맵만 저장'을 누르고 창을 닫으세요.\033[0m"
    echo -e "\033[1;33m[GUI] '적용 & 실행'은 별도 시뮬레이터를 띄우므로 사용하지 마세요.\033[0m"
    python3 "$F1SIM_WS/tools/obstacle_map_maker.py"
  fi
  if [[ "$CRUISE_USE_OBSTACLES" == "1" ]]; then
    validate_obstacle_map || exec zsh
  fi

  echo "에고 시작: (${EGO_X}, ${EGO_Y}, yaw=${EGO_YAW})"
  echo "상대 시작: (${OPP_X}, ${OPP_Y}, yaw=${OPP_YAW}) — 센터라인 약 반 바퀴 지점"
  open_next_tab tab1 "1: F1Sim Two Agents"
  exec zsh

elif [[ "$MODE" == "tab1" ]]; then
  source_stack || exec zsh
  map_stem="$CRUISE_MAP_STEM"
  [[ "$CRUISE_USE_OBSTACLES" == "1" ]] && map_stem="$CRUISE_OBS_STEM"
  map_ext="$(python3 - "$map_stem.yaml" <<'PY'
import os, sys, yaml
with open(sys.argv[1], encoding="utf-8") as stream:
    image = yaml.safe_load(stream)["image"]
print(os.path.splitext(image)[1])
PY
)"
  cd "$F1SIM_WS" || exec zsh
  launch_then_next tab2 "2: MCL Localization" \
    ros2 launch f1tenth_gym_ros obstacle_sim_launch.py \
      map_path:="$map_stem" map_img_ext:="$map_ext" num_agent:=2 \
      sx:="$EGO_X" sy:="$EGO_Y" stheta:="$EGO_YAW" \
      sx1:="$OPP_X" sy1:="$OPP_Y" stheta1:="$OPP_YAW" \
      rviz:="$CRUISE_RVIZ" start_map_server:=false
  exec zsh

elif [[ "$MODE" == "tab2" ]]; then
  source_stack || exec zsh
  cd "$IFAC_WS" || exec zsh
  ros2 launch particle_filter_cpp mcl_launch.py \
    mod:=sim map_name:=map map_topic:=/localization/base_map use_rviz:="$CRUISE_RVIZ" &
  launch_pid=$!
  sleep 3
  quaternion="$(python3 - "$EGO_YAW" <<'PY'
import math, sys
yaw = float(sys.argv[1])
print(math.sin(yaw / 2.0), math.cos(yaw / 2.0))
PY
)"
  read -r qz qw <<< "$quaternion"
  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: $EGO_X, y: $EGO_Y}, orientation: {z: $qz, w: $qw}}}}"
  open_next_tab tab3 "3: Global + Centerline"
  wait "$launch_pid"
  exec zsh

elif [[ "$MODE" == "tab3" ]]; then
  source_stack || exec zsh
  cd "$IFAC_WS" || exec zsh
  export F1_MAP=map
  launch_then_next tab4 "4: Opponent Centerline Control" \
    ros2 launch global_planning global_planning.launch.py map_name:=map
  exec zsh

elif [[ "$MODE" == "tab4" ]]; then
  source_stack || exec zsh
  cd "$IFAC_WS" || exec zsh
  launch_then_next tab5 "5: Local Planning + Detection" \
    ros2 launch opponent_simulator opponent_simulator.launch.py \
      waypoints_topic:=/centerline_waypoints \
      speed_scale:="$CRUISE_OPP_SPEED_SCALE"
  exec zsh

elif [[ "$MODE" == "tab5" ]]; then
  source_stack || exec zsh
  cd "$IFAC_WS" || exec zsh
  export F1_MAP=map
  launch_then_next tab6 "6: State Machine" \
    ros2 launch local_planning local_planning.launch.py \
      simulator:=true use_sim_time:=true start_obstacle_detector:=true
  exec zsh

elif [[ "$MODE" == "tab6" ]]; then
  source_stack || exec zsh
  cd "$IFAC_WS" || exec zsh
  launch_then_next tab7 "7: Ego Cruise Control" \
    ros2 launch state_machine state_machine.launch.py
  exec zsh

elif [[ "$MODE" == "tab7" ]]; then
  source_stack || exec zsh
  cd "$IFAC_WS" || exec zsh
  launch_then_next tab8 "8: Cruise Monitor" \
    ros2 launch f1tenth_control control_sim.launch.py force_autonomous:=true
  exec zsh

elif [[ "$MODE" == "tab8" ]]; then
  source_stack || exec zsh
  echo ""
  echo "크루즈 확인 토픽:"
  echo "  /centerline_waypoints  상대차 추종 경로"
  echo "  /opp_racecar/odom     상대차 위치"
  echo "  /opp_obs              상대차 검출 및 is_interfering"
  echo "  /state                GLOBAL/AVOID/CRUISE"
  echo "  /cruise_speed_limit   에고 속도 상한"
  echo "  /drive, /opp_drive    에고/상대차 구동 명령"
  echo ""
  ros2 topic list | rg "centerline_waypoints|opp_obs|opp_racecar/odom|cruise_speed_limit|/state$|/drive$|/opp_drive$" || true
  exec zsh
fi
