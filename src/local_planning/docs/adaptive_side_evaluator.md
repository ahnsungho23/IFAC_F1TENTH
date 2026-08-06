# Adaptive Side Evaluator

## 1. 목적

`adaptive_side_evaluator`는 글로벌 경로의 각 waypoint에 장애물을 배치하고 로컬 플래너가
왼쪽 통과, 오른쪽 통과, 안전 정지 중 무엇을 선택하는지 일괄 계산하는 오프라인 C++ 실행 파일이다.
ROS 2 노드가 아니므로 topic을 구독하거나 발행하지 않는다.

Python에서 좌우 판정 공식을 다시 구현하지 않고 `RacelineSplinePlanner`의 다음 로직을 그대로 쓴다.

1. 장애물 경계와 clearance로 좌우 목표 `d` 계산
2. waypoint별 트랙 경계 사전 검사
3. quintic spline 생성
4. 장애물·경계·slope·곡률·곡률 변화율 검사
5. 좌우 score와 headroom tie-break
6. 필요할 때 `minimum_avoidance_clearance_m` 재시도

런타임 `plan()`과 다른 점은 데이터셋 생성을 위해 장애물이 글로벌 라인 `d=0`을 직접 막지 않아도
주어진 장애물을 평가한다는 것뿐이다. 나머지 후보 생성과 검증 코드는 공통이다.

## 2. 입력 시나리오

기본 입력은 `ruleset_adaptive_globalpath/map/global_waypoints.csv`이다. 현재 142개 waypoint 각각에
`d=-0.5, -0.4, ..., +0.5`의 11개 장애물을 배치하므로 총 1,562개 시나리오가 된다.

- 장애물 크기: 기본 `0.20 m` 정사각형 Frenet footprint
- ego 위치: 장애물보다 기본 `7.0 m` 뒤의 글로벌 라인 `d=0`
- ego 속도: ego `s`에 가장 가까운 글로벌 waypoint의 `vx_mps`
- 폐곡선 끝: track length를 사용해 wrap 처리

## 3. 빌드와 실행

```bash
source /opt/ros/jazzy/setup.zsh
colcon build --packages-select local_planning
source install/setup.zsh

ros2 run local_planning adaptive_side_evaluator \
  --reference ruleset_adaptive_globalpath/map/global_waypoints.csv \
  --output /tmp/side_evaluations.csv
```

일반 사용에서는 모든 planner YAML 값을 전달해야 하므로 아래 Python 오케스트레이터를 실행하는
것이 안전하다.

```bash
python3 offline_trajectory_generator/generate_adaptive_overlays.py \
  --evaluate-only \
  --output-root ruleset_adaptive_globalpath/adaptive_overlays
```

## 4. 출력

평가기 CSV에는 시나리오별로 다음 내용을 기록한다.

- `index`, 장애물 `s/d`, ego `s/d/speed`
- 최종 `decision`: `left`, `right`, `safe_stop`
- 최종 plan 종류와 target `d`
- 좌우 후보의 평가 여부, 성공 여부, target `d`, 최소 경계 여유
- 좌우 실패 이유와 최종 결과 이유
- 축소 clearance 평가·선택 여부와 생성 path point 수

`safe_stop`은 좌우 후보가 모두 실패하거나 안전한 이동 path를 만들 수 없는 경우다. 실패 원문은
최적화 보상과 제약 위반 집계에 사용할 수 있도록 삭제하거나 축약하지 않는다.

## 5. 반복 제약 최적화 연결

오케스트레이터의 `--planner-override KEY=VALUE`를 반복해 다음 네 파라미터를 후보별로 평가할 수 있다.

```bash
python3 offline_trajectory_generator/generate_adaptive_overlays.py \
  --evaluate-only \
  --planner-override transition_distance_scales=1.0,1.25,1.5 \
  --planner-override outside_line_transition_scale=1.35 \
  --planner-override commitment_clearance_reserve_m=0.05 \
  --planner-override minimum_avoidance_clearance_m=0.18
```

`minimum_avoidance_clearance_m`은 반드시
`vehicle_half_width_m + hard_collision_margin_m` 이상이고 `obstacle_clearance_m` 이하여야 한다.
평가기는 이 hard constraint를 위반한 후보를 실행 전에 거부한다.
