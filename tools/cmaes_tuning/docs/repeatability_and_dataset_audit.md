# Repeatability 및 scenario dataset audit

## 목적

이 단계는 CMA-ES ask/tell을 실행하지 않고, 동일 candidate와 scenario를 반복했을 때의
closed-loop 분산을 측정하고 training/validation dataset을 고정하는 단계입니다.

## 실행 절차

1. `generate-dataset`으로 seed가 고정된 training 25개와 validation 40개를 생성합니다.
2. `scenario-audit`으로 drivable region, wall overlap, spawn 거리, passage, manifest/raster
   대응, 파일 hash, split 중복을 검사합니다.
3. `repeatability-audit`으로 5개 category 대표 scenario를 고릅니다.
4. Baseline과 smoke-best를 각 scenario에서 5회 실행합니다.
5. Candidate를 repetition과 scenario 안에서 교차 실행해 host load 순서 편향을 줄입니다.
6. 유효 episode만 fitness 통계에 포함하고 startup, localization, recorder 문제는
   infrastructure retry로 분리합니다.

## 고정 조건

- Controller max speed: 4.0 m/s
- Fast DDS transport: UDPv4
- Collision agreement: recorded `/scan` TTC external 판정과
  `/ego_racecar/collision`이 일치해야 유효
- Validation scenario: CMA ask/tell fitness에 사용 금지

4.0 m/s는 3.0 m/s smoke 조건보다 한 단계 높지만 실제 고속 목표 조건으로 바로
넘어가지 않는 중간 속도입니다. 속도는 planner parameter와 분리하며 CMA 변수에 넣지
않습니다.

## 출력 구조

```text
runs/cmaes_tuning/<experiment_id>/
├── candidates/
├── configuration/
├── scenarios/training/
├── scenarios/validation/
├── repeatability_episodes/
├── experiment_manifest.json
├── scenario_dataset_audit.json
├── repeatability_runs.json
├── repeatability_audit.json
└── repeatability_summary.csv
```

각 attempt에는 candidate/scenario hash, runner status, process command, controller 설정,
bag, lap metrics, evaluator result, noise diagnostics가 함께 저장됩니다.

## 해석 원칙

- Completion-time std는 completed episode만 대상으로 계산하고 count를 함께 기록합니다.
- Fitness, clearance, steering TV/s, curvature-rate RMS는 모든 유효 episode를 대상으로
  계산합니다.
- Safety outcome flip이 하나라도 있으면 single-repeat CMA 적합으로 판단하지 않습니다.
- Scan noise, localization error, topic interval, planner detection-to-path latency,
  process startup duration, runtime load를 분리해 원인을 판단합니다.
