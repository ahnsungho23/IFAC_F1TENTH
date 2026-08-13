# 정적 장애물 CMA-ES 실험 및 audit 도구

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
- `src/cma_gt_localization`: tuning 전용 simulator GT ego pose bridge(C++, TF 미발행)
- `cmaes_tuning/evaluator.py`: rosbag과 ground truth 기반 외부 평가
- `cmaes_tuning/simulator_collision.py`: simulator raster/LiDAR TTC collision 재판정
- `cmaes_tuning/tracking_swept_analysis.py`: planner path, rectangular footprint, 실제 pose의
  wall-clearance 손실을 timestamp별로 분해하는 read-only 분석
- `cmaes_tuning/parallel_diagnostic.py`: episode 내부 lockstep을 유지한 채 독립 episode만
  격리 병렬 실행하고 exact-hash determinism gate와 1/2/4/8-worker benchmark를 수행
- `tracking_swept_footprint_analysis.py`: 위 분석과 병렬 benchmark의 CLI 진입점
- `cmaes_tuning/controller_audit.py`: planner 속도부터 최종 `/drive`까지 전달 검증
- `cmaes_tuning/deterministic_replay.py`: 기록 입력의 scan-index 정렬 replay 및 live 폐루프 비교
- `deterministic_replay_audit.py`: simulator/controller/CMA를 시작하지 않는 replay audit CLI
- `cmaes_tuning/objective.py`: safety-dominant scalar fitness 계산
- `cmaes_tuning/smoke_experiment.py`: Test A~D와 pycma ask/tell/checkpoint 연결

생성 결과는 기본적으로 `runs/cmaes_tuning/<experiment-id>`에 영구 저장됩니다. 후보 YAML,
시나리오 manifest와 map, episode별 bag/log/status/result, 후보 집계, CSV, CMA checkpoint,
`best_so_far.yaml`을 포함합니다. 시나리오 ground truth는 evaluator만 읽으며 planner와
perception에는 전달하지 않습니다.

본 실험용 dataset은 smoke 3개와 분리되어 있으며 기본값은 training 25개,
validation 40개입니다. Validation manifest는 CMA ask/tell fitness 경로에 전달하지
않습니다.

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

# 기존 bag만 재평가하며 ROS stack이나 CMA optimization은 실행하지 않음
python3 tools/cmaes_tuning/cmaes_runner.py \
  --experiment-id smoke_static_v1 collision-audit

# stratified training/validation dataset 생성 및 offline validity audit
python3 tools/cmaes_tuning/cmaes_runner.py \
  --experiment-id repeatability_mid4_v3 generate-dataset
python3 tools/cmaes_tuning/cmaes_runner.py \
  --experiment-id repeatability_mid4_v3 scenario-audit

# CMA ask/tell 없이 baseline/smoke-best 반복 실행만 수행
python3 tools/cmaes_tuning/cmaes_runner.py \
  --experiment-id repeatability_mid4_v3 repeatability-audit

# 이전 MCL audit와 정확히 같은 scenario/map으로 baseline만 GT localization 재실행
python3 tools/cmaes_tuning/cmaes_runner.py \
  --experiment-id repeatability_gt_mid4_v1 \
  --localization-mode ground_truth \
  --baseline-only \
  --scenario-source-experiment runs/cmaes_tuning/repeatability_mid4_v3 \
  --previous-mcl-experiment runs/cmaes_tuning/repeatability_mid4_v3 \
  repeatability-audit
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

bag에는 odometry, `/scan`, `/ego_racecar/scan_identity`, `/drive`, `/drive_autonomous`, collision, `/avoid_waypoints`,
`/local_waypoints`, `/global_waypoints`, `/state`, `/static_obs`, `/confirmed_static_obs`를
기록합니다. `/confirmed_static_obs`는 offline timing audit 전용이며 local planner 입력은
계속 `/static_obs`입니다. `/scan`은
collision topic을 복사하지 않고 simulator와 같은 noisy LiDAR TTC 식을 외부에서 다시
계산하기 위해 필요합니다. 과거 `/scan` 없는 bag은 아래의 raster TTC envelope로
재평가되며 결과에 `legacy_baked_raster_ttc_envelope`로 표시됩니다.
`/tf`도 기록해 localization pose와 simulator의 `map -> ego_racecar/base_link` edge를
동일 timestamp에서 비교합니다.

## Simulator scan publication audit mode

`simulator_runtime`의 `simulator_seed`, `scan_noise_std_m`,
`scan_publication_mode`은 simulator launch argument로 명시적으로 전달되고 frozen config,
experiment manifest, episode runner status에 저장됩니다. `legacy_republish`는 기존 0.004초
timer가 최신 backend scan을 반복 발행하는 호환 모드이고, `fresh_only`는 odom/TF rate는
유지하면서 `/scan`만 실제 `env.step()` measurement마다 한 번 발행합니다.

`/ego_racecar/scan_identity`는 tuning-only 진단 topic입니다. backend reset/index,
measurement 생성 시각, ROS 발행 시각, float32 ranges SHA-256, seed, noise sigma를 담습니다.
planner/perception 입력에는 연결되지 않으며 obstacle manifest나 GT obstacle geometry도
포함하지 않습니다. `noise_diagnostics.json`은 다음을 직접 계산합니다.

- backend scan당 ROS publication 평균/최대와 새 timestamp를 가진 연속 duplicate 수
- 첫 planner commitment 전 unique backend scan 수와 ROS scan callback 수
- 첫 non-empty `/static_obs` 직전 3개 callback의 unique backend scan 수
- `min_hits_confirm=3` 및 `envelope_stability_frames=2`에 duplicate가 기여했는지 여부
- 첫 `/static_obs`, 첫 `/confirmed_static_obs`, 첫 avoid path timing과 obstacle ID split

향후 Common Random Numbers는 `(generation, scenario_id, noise_seed)` schedule을 먼저
고정한 뒤 같은 generation의 모든 candidate가 동일 scenario/seed 순서를 공유하는
방식으로 적용합니다. 예를 들어 candidate A/B 모두 scenario X를 seed 100, 200으로
각각 평가합니다. candidate별 seed를 만들거나 ask 순서에서 RNG를 소비하면 안 됩니다.

## Avoidance command end-to-end timing audit

`--timing-diagnostics`는 default-off `/cma_timing/events` companion instrumentation을 켭니다.
모든 node는 Linux monotonic/steady clock의 nanosecond 값을 event 발생 지점에서 기록하며,
진단 토픽을 planner/perception/control 입력으로 사용하지 않습니다.

| Event | 실제 계측 지점 |
|---|---|
| T0 | local planner가 첫 relevant non-empty `/static_obs`를 수신 |
| T1 | local planner가 첫 non-empty `/avoid_waypoints`를 실제 발행 |
| T2 | state machine callback에서 3-of-5 조건이 처음 충족 |
| T3 | timer callback에서 committed `GLOBAL -> AVOID` 전환 |
| T4 | 전환 뒤 첫 avoidance `/local_waypoints` 실제 발행 |
| T5 | controller가 그 avoidance local path를 처음 사용하는 cycle |
| T6 | 같은 cycle의 `/drive_autonomous` 실제 발행 |
| T7 | selector의 대응 `/drive` 실제 발행 |
| T8 | gym bridge의 대응 drive callback 진입 |
| T9 | 그 command를 처음 사용한 backend `env.step()` 호출 직전 |

분석기는 T6→T7은 input drive stamp, T7→T8은 output drive stamp, T8→T9은 simulator receive
sequence로 연결합니다. 단순히 토픽상 가까운 event를 고르지 않습니다. 각 event에는 가능한
backend scan index, ego `s/x/y`, speed, obstacle ID, 최초 committed `target_d`도 붙습니다.

production default `state_machine.publish_rate_hz=10`은 변경하지 않습니다.
`--state-machine-publish-rate-hz` override는 `--timing-diagnostics`가 함께 있을 때만 유효합니다.
다음 두 명령은 CMA ask/tell 없이 동일 baseline 25회를 각각 실행합니다.

```bash
python3 tools/cmaes_tuning/cmaes_runner.py \
  --experiment-id e2e_timing_10hz_v1 \
  --localization-mode ground_truth \
  --simulator-seed 12345 --scan-noise-std 0.01 \
  --scan-publication-mode fresh_only --baseline-only \
  --scenario-source-experiment runs/cmaes_tuning/scan_semantics_realistic_fresh_v1 \
  --representative-scenario-ids training_000,training_001,training_002,training_003,training_009 \
  --repetitions 5 --timing-diagnostics \
  --state-machine-publish-rate-hz 10 repeatability-audit

python3 tools/cmaes_tuning/cmaes_runner.py \
  --experiment-id e2e_timing_100hz_v1 \
  --localization-mode ground_truth \
  --simulator-seed 12345 --scan-noise-std 0.01 \
  --scan-publication-mode fresh_only --baseline-only \
  --scenario-source-experiment runs/cmaes_tuning/scan_semantics_realistic_fresh_v1 \
  --representative-scenario-ids training_000,training_001,training_002,training_003,training_009 \
  --repetitions 5 --timing-diagnostics \
  --state-machine-publish-rate-hz 100 repeatability-audit
```

episode별 `noise_diagnostics.json`에는 T0–T9 event와 latency가, experiment-level
`repeatability_audit.json`에는 전체 분포가 저장됩니다. 두 실험은
`cmaes_tuning.timing_audit_compare`로 success/failure별 latency와 scenario 내 분산을 비교합니다.

```bash
PYTHONPATH=tools/cmaes_tuning python3 -m cmaes_tuning.timing_audit_compare \
  --ten-hz-experiment runs/cmaes_tuning/e2e_timing_10hz_v1 \
  --hundred-hz-experiment runs/cmaes_tuning/e2e_timing_100hz_v1 \
  --output runs/cmaes_tuning/e2e_timing_comparison_v1/timing_rate_comparison.json
```

## Perception/planner deterministic record/replay audit

이 audit은 기존 live bag에서 다음 입력만 재생합니다.

```text
/scan, /ego_racecar/scan_identity, /ego_racecar/odom, /pf/pose/odom,
/car_state/frenet/odom, /tf, /global_waypoints, /state
```

`/static_obs`, `/confirmed_static_obs`, `/avoid_waypoints`, drive/collision 토픽은 재생하지
않습니다. Detector에는 obstacle이 baked되지 않은 clean map만 제공하며 scenario obstacle GT와
manifest footprint는 perception/planner 입력에 전달하지 않습니다. Replay process는
`obstacle_detector -> /static_obs -> local_planning -> /avoid_waypoints`만 실행하고 simulator
physics, state machine, controller 및 CMA ask/tell은 실행하지 않습니다.

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

각 replay는 별도 ROS domain과 clean process에서 실행됩니다. 결과는 source bag/manifest/map
hash와 함께 `recorded_streams/*.json`, `replays/*/replay_result.json`,
`replay_audit.json`에 저장됩니다. 비교 기준은 wall clock이 아니라
`/ego_racecar/scan_identity`의 backend scan index입니다. Path hash는 timestamp를 제외하고
waypoint geometry와 limits 전체를 포함합니다. 상세 절차와 해석은
[`docs/deterministic_record_replay_audit.md`](docs/deterministic_record_replay_audit.md)를
참고합니다.

## Deterministic lockstep CMA mode

`lockstep_episode.py`는 production timer를 사용하지 않는 tuning-only closed-loop runner입니다.
동일 logical timestamp의 GT odom과 실제 noisy backend scan을 묶고, detector, planner, FSM,
controller의 기존 core를 순서대로 한 번씩 실행한 뒤 command k를 다음 physics step에 적용합니다.
장애물 GT는 evaluator만 접근하며 perception/planner 입력은 계속 LiDAR 기반입니다.

```bash
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
source /home/sungho/f1sim_C/install/local_setup.zsh
python3 tools/cmaes_tuning/lockstep_smoke_audit.py \
  --output runs/cmaes_tuning/lockstep_smoke_v1 \
  --repetitions 10 --simulator-seed 12345 --scan-noise-std 0.01
```

모든 production YAML과 launch argument의 `lockstep_mode` 기본값은 `false`입니다. 실행 계약,
hash 기준, CRN schedule은
[`docs/deterministic_lockstep_mode.md`](docs/deterministic_lockstep_mode.md)에 설명합니다.

## Medium lockstep CMA-ES

Lockstep smoke acceptance 후 `medium_lockstep_cma.py`로 resumable Stage 1 medium
experiment를 실행합니다. 모든 candidate에 고정 common-random-number schedule을
적용하고, pycma를 ask/tell 전후로 checkpoint하며, validation 40개는 CMA fitness에서
분리합니다. 실행법과 artifact 구조는
[`docs/medium_lockstep_cma.md`](docs/medium_lockstep_cma.md)에 설명합니다.

## Localization mode

`experiment.localization_mode`는 `mcl` 또는 `ground_truth`이며 frozen config,
`experiment_manifest.json`, episode `runner_status.json`에 모두 기록됩니다.

- `mcl`: 기존 `particle_filter_cpp/mcl_launch.py`를 실행하고 `/initialpose`를 적용합니다.
- `ground_truth`: MCL process를 시작하지 않고 `cma_gt_localization`이
  `/ego_racecar/odom`의 timestamp/pose를 `/pf/pose/odom` 인터페이스로 복사합니다.

GT bridge는 TF를 발행하지 않습니다. simulator가 이미
`map -> ego_racecar/base_link -> ego_racecar/laser`를 발행하고, 기존 sim MCL도 launch에서
두 TF publication flag가 모두 false이기 때문입니다. Runner는 control 시작 전에
`/pf/pose/odom` publisher가 정확히 하나인지, 선택하지 않은 localization provider가
존재하지 않는지 확인합니다. Bridge에는 obstacle topic, scenario manifest, baked-map
geometry 입력이 없으며 perception 경로는 계속 `/scan -> obstacle_detector -> /static_obs`입니다.

Fast DDS는 짧은 clean-process episode를 반복할 때 SHM zombie와 participant가 누적되지
않도록 모든 실험 process에 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`를 고정합니다. 종료 시에는
각 ROS launch parent뿐 아니라 동일 process group의 모든 child가 사라졌는지 확인하고,
SIGINT, SIGTERM, SIGKILL 순서의 공동 grace window로 정리합니다.

`repeatability-audit`는 5개 category 대표 scenario에 대해 baseline과 smoke-best를
각 5회 실행합니다. Candidate는 repetition/scenario 안에서 교차 배치하여 시간에 따른
CPU load 편향을 줄입니다. 기본 controller max speed는 CMA 변수 밖의 고정 4.0 m/s입니다.
결과는 `repeatability_audit.json`, `repeatability_summary.csv`, episode별
`noise_diagnostics.json`에 저장됩니다.

## simulator collision 규칙과 evaluator 대응

1. 단일 차량의 map collision은 `cpp_backend.cpp::check_collision()` polygon 교차가
   아닙니다. 이 함수는 차량 간 충돌만 검사합니다. Map collision은 매 0.01초마다
   noisy LiDAR scan을 만든 뒤 `check_ttc()`가 검사합니다.
2. 차량 기준점은 `base_link` 중심이며 footprint는 길이 0.56 m, 폭 0.287 m입니다.
   LiDAR는 중심에서 전방 0.275 m에 있지만, beam별 차량 경계 거리를 LiDAR 좌표계에서
   계산하므로 최종 물리 footprint는 위 사각형과 같습니다.
3. simulator는 map PNG를 상하 반전한 후 gray value가 128 이하인 pixel을 occupied로
   사용합니다. World point는 map origin의 yaw를 역회전하고, 해상도로 나눈 값을
   `floor`한 half-open pixel cell에 대응시킵니다. Simulator ray lookup은 map 바깥을
   마지막 pixel로 alias하며, 현재 track은 그 외곽이 occupied입니다. Evaluator의
   footprint가 map 영역 밖으로 나가면 별도의 보수적 boundary collision으로 처리합니다.
4. obstacle manifest의 연속 사각형은 `MapModel.bake()`에서 PNG polygon으로
   rasterize됩니다. 따라서 실제 simulator footprint는 manifest 경계 자체가 아니라
   그 경계를 포함해 칠해진 occupied pixel cell들의 합집합입니다. 0.025 m map에서는
   최대 pixel 단위의 양자화 차이가 생길 수 있습니다. 각 scenario manifest의
   `baked_obstacle_raster`에는 상하 반전 이후 cell index, half-open world bounds,
   변경 cell 수를 함께 저장해 연속 obstacle geometry와 직접 대조할 수 있습니다.
5. TTC는 각 beam에 대해
   `(noisy_scan_range - vehicle_side_distance) / (speed * cos(beam_angle))`를 계산해
   `0 <= TTC < 0.005 s`이면 충돌입니다. Scan noise는 beam마다 독립인 표준편차
   0.01 m Gaussian입니다.
6. 현재 simulator는 TTC 충돌 시 state index 3~6을 0으로 만들며, 여기에 yaw index 4가
   포함됩니다. 따라서 collision frame odometry의 yaw=0을 그대로 polygon 평가하면
   실제 충돌 직전 orientation을 잃어버립니다. Evaluator는 collision topic을 보지 않고
   이동 중이던 차량의 불가능한 1-step 정지와 yaw reset을 검출하고, 직전 고유 physics
   pose의 yaw 증가량으로 해당 frame yaw를 복원합니다.
7. 신규 bag에서는 기록된 `/scan`에 위 TTC 식을 그대로 다시 적용한 결과가 external
   collision입니다. Legacy bag에서는 복원한 oriented footprint를 `speed*0.005 s`만큼
   진행 방향으로 sweep하고 scan-noise guard만큼 확장한 뒤 baked raster와 비교합니다.
   Collision topic과 external 결과가 다르면 episode는
   `collision_convention_mismatch` invalid episode로 차단되어 CMA fitness에 들어가지
   않습니다.

최소 재현 데이터는 `tests/fixtures/train_000_collision_mismatch.json`에 있습니다.
기록 yaw=0으로 계산하면 manifest obstacle clearance가 약 +0.0203 m이지만, 소실된 yaw를
복원하고 baked raster/TTC convention을 적용하면 collision입니다. `collision-audit`의
결과는 experiment 디렉터리의 `collision_agreement.json`에 저장됩니다.

## Hybrid CMA v2 (2026-08-12)

v1 frozen rule gate의 1번 장애물(s=9.114)은 차량 풀락 곡률(1.316 rad/m) 코너 출구 2.5 m
뒤에 있어 어느 쪽으로도 회피가 물리적으로 불가능했고, v1 CMA의 완주 0/26은 파라미터가
아니라 이 배치가 원인이었습니다. v2는 규정(≤0.5×0.5 m, 쌍별 ≥1 m, 자유 폭 ≥0.5 m, 출발선
1 m 밖)과 **차량 실현성**(κ-shadow, 통과변 통로 지속성, 병합 감쇠 모델)을 함께 만족하는
장애물을 새로 생성해 사용합니다.

- `clean_lap_lut_traces.sh <out> [domain]`: 클린 맵에서 속도 상한 1.6/2.0/2.5/2.9 + race
  프로파일 랩을 자동 주행·기록. 결과 trace를 `tracking_error_lut_from_traces.py`에 넣으면
  저속 행까지 실측된 LUT가 나옵니다.
- `generate_cma_scenarios_v2.py`: 시드 고정 무작위 배치 + 규정/실현성 필터 + map bake +
  `rule_gate_v2.json`/manifest 생성. FINALS는 baseline에서 완주가 증명된 Q2 쌍 + s=41.15
  우측 통과 강제 장애물(좌측은 게이트가 막힘)로 구성됩니다. 배치 논거는 스크립트 상단과
  각 상수의 주석에 있습니다.
- `p3_hybrid_cma_v2.py`: v2 whitelist(`outside_line_transition_scale` 하한 0.05→0.35 —
  v1 하한은 outside exit 전체를 곡률-불가능 dead zone으로 만들었음)와 v2 시나리오로
  baseline → smoke(4×2) → main(기본 8×8)을 실행합니다. `--baseline-only`로 production
  구성의 폐루프 완주만 따로 확인할 수 있습니다.

```bash
python3 tools/cmaes_tuning/generate_cma_scenarios_v2.py
python3 tools/cmaes_tuning/p3_hybrid_cma_v2.py --execute --baseline-only   # 완주 게이트
python3 tools/cmaes_tuning/p3_hybrid_cma_v2.py --execute --resume          # 본 실행
```

## 해석 시 주의사항

- 연속 manifest clearance는 분석 metric이며 collision 정답이 아닙니다. Safety failure는
  recorded scan TTC 또는 legacy baked-raster TTC 결과로 판정하고 collision topic은
  독립 agreement 확인에만 사용합니다.
- wall clearance는 장애물이 없는 clean map과 차량 footprint로 계산합니다.
- 현재 3개 시나리오는 framework 연결 검증용이며 일반화 성능을 주장하기에 부족합니다.
- 대규모 탐색 전에는 recorded `/scan`을 포함한 새 episode에서
  `collision_agreement.json`의 `all_agree=true`를 다시 확인해야 합니다.
