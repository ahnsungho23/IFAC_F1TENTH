# R3-RT native C++ shadow implementation and timing study v1

## Final decision

`R3_RT_NATIVE_TIMING_NEEDS_OPTIMIZATION`

Python design과 native C++ semantics는 허용된 seen snapshot 86개 × B64/B96/B128 = 258개
조합에서 exact parity를 통과했다. Bounded R3 core는 모든 측정에서 25 ms 안이었지만, 실제
호출 순서의 선행 production ladder를 포함한 evaluator p95는 40.40--41.90 ms였다. 현재 운영
주기는 25 ms이므로 어떤 B도 freeze-ready로 선언하지 않는다.

## What was implemented

- `selectP3R3RTFactors()`는 Python design의 geometry-conditioned lateral operator, 7-anchor
  transition mapping, NO_RESERVES_DIAGONAL wave, proxy tuple, feasibility Top-10 및 coverage Top-2를
  C++로 옮긴 research-only selector다.
- `evaluateP3R3RTShadow()`는 기존 P3 reconstruction, exact validator, digest dedup과 final ranker를
  재사용하는 명시적 research callable이다.
- Production `plan()`과 `evaluateP3Shadow()`의 기본 exact-R3 경로는 이 API를 호출하지 않는다.
- Runtime budget은 64/96/128만 허용한다. 전체 exact-R3 factor pool을 만드는 fallback은 없다.

Reference exact-R3 method spec SHA-256은
`7b861e8c7e23ae168413885ea0dc2d09769046ff48a562700b658e084d116fcc`로 재검증했다.

## Parity result

[python_cpp_parity.csv](python_cpp_parity.csv)는 각 event-budget마다 다음 9 gate를 기록한다.

1. lateral proposal
2. transition proposal
3. ordered pair pool
4. proxy tuple
5. selected Top-12 configuration
6. reconstructed path digest
7. hard/usable validator verdict
8. event-level hard/usable aggregate result
9. final selected path

모든 gate가 **258/258 PASS**다. Python reference reconstruction/validation은 parity에만 사용했고,
그 subprocess wall time은 native timing 결과에 포함하지 않았다.

Production non-interference도 별도로 확인했다.

- pre-integration success control: **18/18** selected digest 및 production sequence exact
- R3 invocation/추가 validator: **0/0** on those controls
- frozen exact-R3 seen parity: **86/86**
- 기존 targeted CTest: candidate rank, exact R3, maneuver lifecycle, production parity PASS
- config/launch diff: **0**

`params_match_yaml` 한 건은 이 변경과 무관한 기존 `stuck_case_harness.cpp`의
`safety_margin_m 0.05` 대 운영 YAML `0.08` 불일치로 실패했다. 요청 범위에 따라 변경하지 않았다.

## Timing method

- Release build (`-DCMAKE_BUILD_TYPE=Release`)
- instrumentation OFF
- 단일 native process가 86개 event와 세 budget을 순회
- planner/reference setup 후 각 event-budget warmup 5회
- 각 event-budget 20회 측정: budget당 1,720 samples
- CSV/상세 parity serialization은 timed region 밖이며 timing run에서는 비활성

핵심 결과는 [native_runtime_by_budget.csv](native_runtime_by_budget.csv), 단계별 분해는
[runtime_breakdown.csv](runtime_breakdown.csv), event tail은
[worst_event_runtime.csv](worst_event_runtime.csv)에 있다. 가장 느린 full evaluator event는 모든
budget에서 DVE029였고 max는 69.42/70.76/70.91 ms였다.

## Budget/recovery result

| B | DEV+VAL usable retention | native R3 p50/p95/p99/max ms | full evaluator p50/p95/p99/max ms |
|---:|---:|---:|---:|
| 64 | 30/32 | 5.323 / 12.907 / 16.836 / 17.125 | 13.190 / 40.396 / 68.938 / 69.422 |
| 96 | 31/32 | 5.957 / 13.773 / 16.827 / 16.939 | 13.825 / 40.976 / 70.124 / 70.759 |
| 128 | 32/32 | 6.339 / 14.288 / 16.835 / 16.985 | 14.455 / 41.904 / 70.506 / 70.909 |

세부 recovery와 exact-R3 comparison은
[candidate_budget_comparison.md](candidate_budget_comparison.md)에 있다.

## Deadline interpretation

운영/시뮬 YAML은 모두 25 ms, 즉 40 Hz를 지정한다. Timer에는 deadline miss를 강제하는 hard
real-time mechanism이 없으므로 이는 soft real-time cycle budget이다. Active path continuation은
fresh evaluation을 생략할 수 있지만, fresh recovery 자체는 현재 한 callback 안에서 완결되어야
한다. 근거는 [deadline_contract_audit.md](deadline_contract_audit.md)에 정리했다.

## Reproduction

```bash
source /opt/ros/jazzy/setup.zsh
colcon build --packages-select local_planning --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.zsh
python3 planning_study/p3_r3_rt_native_cpp_v1/run_native_cpp_study.py --mode parity
python3 planning_study/p3_r3_rt_native_cpp_v1/run_native_cpp_study.py \
  --mode timing --warmup 5 --repeats 20
```

이 연구는 FINAL_HOLDOUT event content/result를 읽거나 재실행하지 않았고, study driver도 세
허용 role 86개만 받아들인다. 다만 작업 중 repository-wide `find`가 FINAL_HOLDOUT 아래의 일부
파일명까지 출력한 적은 있다. 파일 내용·manifest·결과는 열지 않았고 어떤 계산/선택에도
들어가지 않았다. Large closed-loop benchmark, production decision enable, commit, push도 수행하지
않았다.
