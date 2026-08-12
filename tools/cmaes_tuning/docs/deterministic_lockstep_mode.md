# CMA deterministic lockstep simulation

## 1. 목적과 범위

이 실행 경로는 CMA tuning에서 동일한 planner parameter, scenario, simulator seed, initial
condition이 동일한 trajectory와 fitness를 만들도록 합니다. Production ROS launch의 비동기
timer와 callback 구조는 유지되며 모든 `lockstep_mode` 기본값은 `false`입니다.

GT ego pose는 localization 대체와 동일-step snapshot 결합에만 사용합니다. 장애물 GT,
manifest footprint, baked-map obstacle 위치는 perception/planner 입력에 전달하지 않습니다.
장애물 경로는 실제 F110 backend LiDAR에서 시작해
`/scan -> obstacle_detector -> /static_obs -> local_planning` 순서를 그대로 따릅니다.

## 2. Step 계약

하나의 10 ms logical step k는 다음 barrier를 통과합니다.

1. backend physics state k를 고정합니다.
2. state k에서 GT odom과 sigma 0.01 m noisy LaserScan k를 취득합니다.
3. 동일 timestamp의 odom과 scan을 한 immutable snapshot으로 발행합니다.
4. detector가 정확히 한 번 갱신되어 `/static_obs(k)`를 확정합니다.
5. 동일 timestamp Frenet state와 obstacle로 planner를 정확히 한 번 실행합니다.
6. FSM을 한 번 평가하고 `/state(k)`, `/local_waypoints(k)`를 확정합니다.
7. controller를 한 번 실행해 `/drive_autonomous(k)`를 확정합니다.
8. coordinator가 그 값 자체를 다음 `env.step()`에 적용해 state k+1을 계산합니다.

각 C++ wrapper는 입력 header timestamp의 정확한 일치를 barrier로 사용합니다. Wall timer,
DDS callback 도착 순서, wall-clock timestamp는 decision에 사용하지 않습니다. DDS는 transport일
뿐이며 coordinator는 최종 command k가 확인될 때까지 step k+1을 발행하지 않습니다.

## 3. 재사용하는 알고리즘

- Detector의 clustering, map filter, Kalman tracker와 3-of-5 confirmation을 그대로 호출합니다.
- Planner의 `RacelineSplinePlanner`, stabilization, commitment, side/scale 선택을 그대로 호출합니다.
- State machine의 M-of-N history와 기존 FSM을 그대로 호출합니다.
- Controller의 L1 guidance, steering/speed 제한 계산을 그대로 호출합니다.
- Physics와 LaserScan은 `f110_gym` C++ backend를 직접 사용합니다.

별도 구현은 ROS I/O를 logical event로 묶는 얇은 adapter와 runner뿐입니다. Scenario manifest는
runner/evaluator만 읽으며 perception/planner node command에는 전달되지 않습니다.

## 4. Seed와 Common Random Numbers

Runner 입력은 `--scenario`, `--candidate`, `--simulator-seed`, `--scan-noise-std`입니다. Backend
constructor/reset에 명시적 seed를 전달하므로 동일 scan index의 noise realization이 같습니다.
`common_random_number_schedule()`은 모든 candidate에 동일하게 정렬된
`(scenario_id, simulator_seed)` 집합을 배정합니다.

## 5. Smoke audit 실행

```bash
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
source /home/sungho/f1sim_C/install/local_setup.zsh
python3 tools/cmaes_tuning/lockstep_smoke_audit.py \
  --output runs/cmaes_tuning/lockstep_smoke_v1 \
  --repetitions 10 \
  --simulator-seed 12345 \
  --scan-noise-std 0.01
```

Smoke scope는 spawn부터 obstacle을 통과하고 추가 3 m를 진행할 때까지의 완전한 obstacle-encounter
window입니다. 매 step에 physics/perception/planning/FSM/control이 모두 들어갑니다. 이는 긴 lap의
wall 비용을 들이지 않고 결정성 계약을 검증하기 위한 범위이며, medium CMA 전에는 동일 runner의
평가 종료 범위와 objective 정의를 experiment manifest에 고정해야 합니다.

Bit-identical 판정 대상은 physics state, scan, `/static_obs`, `/avoid_waypoints`, controller command,
trajectory의 전체 sequence SHA-256입니다. `target_d`와 fitness는 절대 오차 1e-12 이하도 허용하지만
현재 smoke에서는 exact equality도 함께 기록합니다. 결과는 experiment manifest, 각 episode bag,
runner/evaluator 결과, `lockstep_result.json`, 최종 `smoke_audit.json`으로 보존됩니다.
