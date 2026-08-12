# Medium lockstep CMA-ES 실행

## 목적

`medium_lockstep_cma.py`는 production ROS 경로와 분리된 deterministic lockstep
환경에서 Stage 1 planner parameter 최적화를 수행한다. GT는 ego localization에만
사용하며, obstacle detector와 local planner의 장애물 입력은 simulator LiDAR scan뿐이다.

## 고정 실험 조건

1. `training_000`부터 `training_009`까지 category별 2개, 총 10개 scenario를
   manifest 순서대로 사용한다.
2. 각 scenario는 두 개의 명시적 noise seed로 실행한다.
3. 한 generation의 모든 candidate는 동일한 20개 `(scenario_id, seed)` 순서를
   공유한다.
4. `population_size=10`, `generations=5`, `scan_noise_std=0.01 m`,
   `speed=4.0 m/s`를 사용한다.
5. validation 40개는 ask/tell에 사용하지 않으며 scenario별 하나의 unseen seed로
   baseline과 training 상위 3개만 평가한다.
6. 정상 obstacle window가 끝나지 않는 실패 rollout은 CMA 전용 8초 simulated-time
   horizon에서 종료하고 동일 evaluator로 safety/progress cost를 계산한다.

## 실행과 재개

ROS 2, workspace, F110 simulator 환경을 source한 뒤 다음과 같이 실행한다.

```bash
python3 tools/cmaes_tuning/medium_lockstep_cma.py \
  --output runs/cmaes_tuning/medium_lockstep_stage1_v2
```

같은 명령을 다시 실행하면 valid rollout은 candidate hash, scenario ID, simulator
seed, scan noise를 확인한 뒤 재사용한다. 각 generation의 `cma_after_ask.pkl`과
`cma_after_tell.pkl`은 ask 결과와 CMA 내부 상태를 보존하므로 중간 실패 뒤 정확한
상태에서 이어진다.

## 저장 구조

- `experiment_manifest.json`: 조건, source/config hash, CRN ordering, dataset 분포
- `artifacts/`: 고정 config, parameter space, baseline, scenario manifests, pycma source
- `episodes/`: rollout별 bag, evaluator metric, lockstep hash, 실행 로그
- `candidate_results/`: candidate aggregate와 rollout 요약
- `generations/`: ask plan, mean/sigma/covariance, ranking, generation checkpoint
- `checkpoints/`: 최신 exact-resume CMA state
- `validation_comparison.json`: validation-only 비교
- `final_best.yaml`, `final_report.json`: 최종 candidate와 분석 결과

## Generation 0 gate

Generation 0 이후 parameter/YAML 반영, fitness 구분 가능성, hard safety dominance,
finite parameter, 동일 rollout 재실행의 lockstep hash와 scalar fitness 일치를 검사한다.
하나라도 실패하면 ask/tell을 진행하지 않고 `experiment_halted.json`을 남긴다.
