# 정적 장애물 CMA-ES smoke-test 도구

이 디렉터리는 기존 local planner의 경로 생성 및 perception 코드를 바꾸지 않고,
정적 장애물 회피 파라미터를 외부에서 평가하는 오프라인 실험 도구입니다. 현재 기본
설정은 품질 좋은 최적값을 찾기 위한 대규모 탐색이 아니라 `population=4`,
`generation=2`, 고정 시나리오 3개의 연결 검증용 smoke test입니다.

## 구성

- `config/parameter_space.yaml`: 10차원 정규화 변수, baseline, 물리 범위, ROS 파라미터 매핑
- `config/tuning_config.yaml`: 시나리오, ROS 실행, controller, evaluator, 목적함수, CMA 설정
- `cmaes_runner.py`: 시나리오 생성과 Test A~D 실행 CLI
- `cmaes_tuning/scenario_generator.py`: seed가 고정된 시나리오 및 baked map/manifest 생성
- `cmaes_tuning/simulation_runner.py`: episode별 전체 ROS stack 실행·종료·재시도
- `cmaes_tuning/evaluator.py`: rosbag과 ground truth 기반 외부 평가
- `cmaes_tuning/controller_audit.py`: planner 속도부터 최종 `/drive`까지 전달 검증
- `cmaes_tuning/objective.py`: safety-dominant scalar fitness 계산
- `cmaes_tuning/smoke_experiment.py`: Test A~D와 pycma ask/tell/checkpoint 연결

생성 결과는 기본적으로 `/tmp/cmaes_tuning/<experiment-id>`에 저장됩니다. 후보 YAML,
시나리오 manifest와 map, episode별 bag/log/status/result, 후보 집계, CSV, CMA checkpoint,
`best_so_far.yaml`을 포함합니다. 시나리오 ground truth는 evaluator만 읽으며 planner와
perception에는 전달하지 않습니다.

## 파라미터 처리

CMA 변수 `z`는 모두 `[0, 1]`입니다. 후보마다 기존
`src/local_planning/config/local_planning.yaml`을 복사하고 whitelist 항목만 수정합니다.
`pre_apex_far_m`와 `post_apex_far_m`는 각각 ordered 3-element 배열로 변환하며,
`transition_short < transition_middle < transition_long`을 검증합니다.

## 설치 및 단위 테스트

ROS 2 Jazzy와 이 workspace를 먼저 build한 환경에서 다음 순서로 실행합니다.

```bash
python3 -m pip install -r tools/cmaes_tuning/requirements.txt
PYTHONPATH=tools/cmaes_tuning \
  python3 -m unittest discover -s tools/cmaes_tuning/tests -v
```

## 실행 순서

`<experiment-id>`는 같은 실험을 재개할 때 동일하게 사용합니다. 유효한 기존 episode는
bag과 manifest를 현재 evaluator로 다시 평가한 뒤 재사용합니다.

```bash
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh

python3 tools/cmaes_tuning/cmaes_runner.py \
  --experiment-id smoke_static_v1 generate-scenarios

python3 tools/cmaes_tuning/cmaes_runner.py \
  --experiment-id smoke_static_v1 test-a
python3 tools/cmaes_tuning/cmaes_runner.py \
  --experiment-id smoke_static_v1 test-b
python3 tools/cmaes_tuning/cmaes_runner.py \
  --experiment-id smoke_static_v1 test-c
python3 tools/cmaes_tuning/cmaes_runner.py \
  --experiment-id smoke_static_v1 test-d
```

`test-d`는 같은 experiment의 `test_c.json`에 민감도 통과 결과가 있을 때만 실행됩니다.
모든 단계를 한 번에 실행하려면 마지막 인자를 `all`로 지정할 수 있지만, 먼저 A~C의
결과를 확인하는 방식을 권장합니다.

## episode와 실패 처리

상태는 `PREPARE → LAUNCH → WAIT_READY → RUN → TERMINATE → COLLECT → EVALUATE →
CLEANUP` 순서로 `runner_status.json`에 기록됩니다. simulator/topic/recorder/hash/bag/
cleanup 문제는 `invalid_episode`로 분류하여 재시도하며 CMA fitness에 넣지 않습니다.
collision, off-track, planner failure는 `safety_failure`, 그 외 미완주는
`completion_failure`입니다.

bag에는 odometry, `/drive`, `/drive_autonomous`, collision, `/avoid_waypoints`,
`/local_waypoints`, `/global_waypoints`, `/state`, `/static_obs`를 기록합니다. `/scan`은
준비 상태 확인에만 사용하고 기본 bag에는 기록하지 않습니다.

## 해석 시 주의사항

- collision topic은 simulator의 실제 판정을 보존하므로 evaluator의 연속 기하 footprint
  overlap보다 보수적일 수 있습니다. 둘을 결과에 따로 기록합니다.
- wall clearance는 장애물이 없는 clean map과 차량 footprint로 계산합니다.
- 현재 3개 시나리오는 framework 연결 검증용이며 일반화 성능을 주장하기에 부족합니다.
- 대규모 탐색 전에는 별도 validation scenario, 반복 실행 분산, 실제 vehicle footprint와
  simulator collision geometry 일치 여부를 추가 검증해야 합니다.
