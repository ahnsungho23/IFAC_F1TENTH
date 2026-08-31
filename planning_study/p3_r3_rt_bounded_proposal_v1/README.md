# R3-RT bounded proposal design study v1

## 결론

이 연구는 frozen exact R3를 변경한 최적화가 아니라, seen geometry에서 최대 B개의
L/T pair만 제안하는 **새로운 bounded method**다. 전체 Cartesian factor pool과
FINAL_HOLDOUT 결과는 사용하지 않았다. Production planner도 변경하지 않았다.

| B | usable DEV | usable VALIDATION | usable combined | teacher-final present | Top12 recall | p95 ms* |
|---:|---:|---:|---:|---:|---:|---:|
| 24 | 11/14 | 11/18 | 22/32 | 15/36 | 0.127 | 119.77 |
| 32 | 13/14 | 12/18 | 25/32 | 17/36 | 0.156 | 130.20 |
| 48 | 13/14 | 15/18 | 28/32 | 25/36 | 0.202 | 160.37 |
| 64 | 13/14 | 17/18 | 30/32 | 26/36 | 0.237 | 202.15 |
| 96 | 13/14 | 18/18 | 31/32 | 30/36 | 0.264 | 295.81 |
| 128 | 14/14 | 18/18 | 32/32 | 31/36 | 0.300 | 382.54 |

`*` 런타임은 geometry 준비 + Python proposal/ranking + 별도 C++ harness process
기동을 합친 보수적 offline wall proxy다. 따라서 실제 C++ callback latency와 동일하다고
주장하지 않는다.

## 핵심 해석

- exact teacher의 combined seen usable recovery는 32건이다.
- exact teacher보다 usable miss가 1건 이하인 최소 budget: 96.
- B96의 남은 teacher miss: DVE039.
- normalized recovery/runtime endpoint-chord knee: B64.
- offline wall proxy p95가 25 ms 이하인 budget: NONE.
- Pareto 비지배 budget: 24, 32, 48, 64, 96, 128.
- VUE019는 exact R3도 hard/usable recovery가 없었던 oracle-infeasible large-factor 사례다.
  Exact pair pool 71,506개 대신 B=128에서 128개를 평가했고,
  bounded 결과도 usable recovery를 주장하지 않는다.

Top-12 imitation 자체보다 unchanged exact validator를 통과한 hard/usable recovery를 우선
판단했다. 세부 teacher 패턴, budget별 event 결과, reserve ablation, miss 원인은 동봉 CSV에
있다.

## Exact R3 teacher가 실제로 복구한 패턴

86개 snapshot의 K12 1,032개 선택을 먼저 고정해 분석했다. 최종 usable path 36개 중
feasibility Top-10에서 32개, coverage Top-2에서
4개가 나왔다. Target family는 production
28개와 component-boundary
8개였다. Mid는 equal-target
12, target-offset 16,
corridor interpolation 7, reference-inward
1개였다. Transition은 long-entry medium/short/long-exit
각 12/8/
4개, short-entry short/medium/long-exit 각
8/1/
3개였다. R3는 direct factor method이므로 analytic
probe/root source는 적용되지 않는다.

## 한계와 다음 gate

이 결과는 PILOT/DEVELOPMENT/VALIDATION_SEEN_AFTER_V1에 맞춘 설계 연구다. Runtime은
Python/subprocess proxy이므로, 선택된 B를 freeze하거나 production에 통합하기 전에 독립
C++ offline implementation과 untouched evaluation data가 필요하다. 이 단계에서는 새
holdout을 만들거나 과거 FINAL_HOLDOUT을 재사용하지 않았다.

## Recommendation

`R3_RT_RUNTIME_STILL_TOO_HIGH`
