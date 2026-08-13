# CMA Common Random Numbers 설계

## 목적

같은 generation의 모든 candidate가 동일 scenario와 동일 LiDAR noise seed set을 사용하게 해
candidate 간 fitness 차이에서 Monte Carlo noise를 줄인다. 이 설계는 fresh-scan-only와
explicit simulator seed를 전제로 한다.

## Schedule

Generation 시작 전에 다음 순서가 고정돼야 한다.

```text
generation g
  scenario X, seed 100
  scenario X, seed 200
  scenario Y, seed 100
  scenario Y, seed 200
```

Candidate A와 B는 모두 위 task key를 동일 순서로 평가한다. Candidate ID, ask 순서,
worker 시작 시각으로 seed를 만들면 안 된다.

## Manifest contract

Generation manifest에 다음을 저장한다.

- `crn_schedule_version`
- `generation`
- ordered `scenario_ids`
- ordered `noise_seeds`
- `(scenario_id, noise_seed)` task list와 SHA-256
- simulator source/config/backend binary hash
- `scan_publication_mode: fresh_only`
- `scan_noise_std_m`
- candidate별 task completion/cache key

Episode cache key는 최소
`candidate_yaml_hash + scenario_manifest_hash + simulator_hash + noise_seed + controller_hash`
조합이어야 한다.

## Ask/tell aggregation

각 candidate는 모든 동일 CRN task가 valid일 때만 tell에 들어간다. Infrastructure failure는
같은 task key로 재시도하고 다른 seed로 대체하지 않는다. Candidate fitness는 먼저
scenario/seed별 episode fitness를 계산한 뒤 모든 candidate에 동일한 aggregation과 CVaR
규칙을 적용한다.

## 현재 적용 판단

구조 구현은 가능하지만 현재 fresh-only repeatability에서도 safety flip이 3/5다. CRN은
LiDAR RNG sequence를 candidate 간 맞출 뿐 ROS scheduling, controller timing, detector가
confirmation하는 scan index를 고정하지 못한다. 따라서 remaining timing audit가 끝나기 전
CRN medium CMA를 실행하지 않는다.

