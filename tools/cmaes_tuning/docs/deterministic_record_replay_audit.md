# Deterministic record/replay audit

## 1. 목적

이 도구는 남은 repeatability variation을 다음 두 경로로 분리합니다.

1. 같은 sensor/ego 입력에서도 ROS callback과 planner timer 실행 순서가 달라지는 경로
2. live controller command와 physics 적용 위상이 먼저 달라져 다음 ego pose와 LiDAR가 달라지는 경로

Medium/full CMA는 실행하지 않습니다. Production perception/planner 파라미터와 알고리즘도
변경하지 않습니다.

## 2. 기록 입력 계약

Replay 입력은 `/scan`, backend scan identity, simulator/GT localization odometry, Frenet
odometry, `/tf`, `/global_waypoints`, `/state`로 제한됩니다. Source bag의 exact LaserScan
ranges와 timestamp를 그대로 사용하고 backend scan index로 결과를 정렬합니다.

다음 항목은 replay하지 않습니다.

- `/static_obs`, `/confirmed_static_obs`
- `/avoid_waypoints`
- `/drive`, `/drive_autonomous`
- `/ego_racecar/collision`
- obstacle manifest의 위치·크기·footprint

Scenario manifest는 orchestration이 obstacle-free clean map의 경로와 hash를 확인하는 데만
사용합니다. Detector와 planner는 obstacle GT를 받지 않습니다.

## 3. 실행 구조

```text
recorded LaserScan/ego/TF/reference/state
  -> obstacle_detector
  -> /static_obs
  -> local_planning
  -> /avoid_waypoints
```

각 repetition은 새 ROS domain과 새 node process를 사용합니다. Simulator physics, controller,
state machine 및 CMA runner는 시작하지 않습니다. Detector/planner의 default-off companion
event를 함께 기록해 raw detection, track hit history, 3-of-5, envelope stability,
stabilization 및 commitment를 source scan index에 대응시킵니다.

## 4. 실행

```bash
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh

PYTHONPATH=tools/cmaes_tuning \
python3 tools/cmaes_tuning/deterministic_replay_audit.py \
  --experiment-id deterministic_replay_v1 \
  --source-experiment runs/cmaes_tuning/e2e_timing_10hz_v1 \
  --scenario-sources training_001=0,training_009=0 \
  --repetitions 10
```

10회 미만은 audit으로 인정하지 않고 CLI가 거부합니다. Source bag, scenario manifest, clean
map, controller/simulator 조건과 모든 파일 hash는 experiment manifest에 남습니다.

## 5. 판정

다음이 모두 같아야 `perception_planner_deterministic=true`입니다.

- raw detection 및 tracker sequence
- confirmation/envelope-stability/최초 `/static_obs` scan index
- obstacle ID와 발행 기하
- planner stabilization/commit scan index
- side, `target_d`, transition scale
- timestamp를 제외한 `/avoid_waypoints` geometry hash
- 모든 backend scan의 detector callback 존재

동일 입력에서 tracker 또는 planner 결과가 다르면 ROS execution-order variation으로
분류합니다. Replay가 동일하지만 live run만 다르면 controller/physics로 시작된 closed-loop
trajectory variation으로 분류합니다. 두 현상은 동시에 존재할 수 있습니다.

## 6. 결과 파일

- `experiment_manifest.json`: replay 범위와 금지 입력, source hash
- `recorded_streams/<scenario>.json`: source episode와 GT/fresh-scan/noise 조건
- `replays/<scenario>/replay_XX/bag`: 실제 replay 출력
- `replays/<scenario>/replay_XX/replay_result.json`: scan-index milestone과 hash
- `replay_audit.json`: 10회 통계와 최초 divergence
- `static_geometry_variance.json`: 정렬된 `/static_obs` 수치 span
- `live_closed_loop_comparison.json`: drive/physics부터 target까지 live 인과 순서

Replay 중 backend scan callback 누락이 있으면 해당 run을 숨기지 않고 index를 기록합니다.
초기 commitment 이후의 단발 누락과 decision 이전 누락을 구분해 해석해야 합니다.
