# R3-K12 real-time profiling and behavior-preserving optimization v1

## 결론

Frozen method `R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12`의 과학적 의미와 결과를 바꾸지 않고 R3
core p50을 213.80 ms에서 17.19 ms로 줄였다. p50은 25 ms 아래지만 p95/p99/max는
132.73/196.07/263.35 ms다. strict/relaxed production ladder까지 포함한 evaluator wall은
p50 29.80 ms, p95 166.05 ms다. 따라서 **기능적으로 exact하지만 충분한 tail margin을 가진
25 ms closed-loop real-time qualification은 달성하지 못했다.** large closed-loop benchmark는
요청대로 실행하지 않았다.

## 동결 계약

- 시작 checkpoint: `p3_r3_k12_functional_integration_v1`
  (`17d853f3399fb68ed468bc30602633d39f59d7a0`)
- method: `R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12`
- method SHA-256: `7b861e8c7e23ae168413885ea0dc2d09769046ff48a562700b658e084d116fcc`
- feasibility quota 10, coverage reserve 2, K=12 유지
- factor/order/tie/dedup/path geometry/validator/rank/selection/lifecycle/config 변경 없음

현재 method spec의 SHA를 다시 계산해 위 값과 일치함을 확인했다. `src/local_planning/config`는
시작 tag 대비 diff가 없다.

## 측정 계약

Combined-seen production-failure snapshot 86개를 event당 독립 process, 순차 실행했다. Build는
Release, GCC 13.3.0, CMake 3.28.3이며 측정 host는 Intel i7-14650HX(24 logical CPU)다. percentile은
86개 event의 선형 보간 percentile이다. 이는 deterministic offline harness 측정이며 ROS executor,
DDS, callback scheduling을 포함한 closed-loop benchmark가 아니다.

A/B/C를 구분했다.

- A: instrumentation OFF production-equivalent R3.
- B: active research cycle ON, lineage/capture와 pure JSON serialization 포함. 파일 I/O는 제외했다.
- C: process startup, event parse, reference setup, output 등 standalone harness 비용.

[profiling_breakdown.csv](profiling_breakdown.csv)는 profile-first baseline과 최종 phase 분포,
[before_after_runtime.csv](before_after_runtime.csv)는 전체 개선,
[instrumentation_overhead.csv](instrumentation_overhead.csv)는 A/B/C 분리를 기록한다.

## 병목 해석

초기 p95의 지배 항목은 pair priority 1,016.21 ms와 coverage ordering 1,047.36 ms였고 exact
validation은 5.99 ms였다. 최종 p95에서도 pair priority 76.23 ms가 가장 크다. factor pool은
p50 3,677.5개지만 p95 35,783.25개, max 71,506개다. frozen semantics를 유지하면 이 전체 pool의
proxy metric을 평가해야 한다. 이 입력 의존적 factor explosion이 25 ms tail을 넘는 핵심
병목이며 validator나 research serialization을 원인으로 돌릴 수 없다.

최종 instrumentation ON의 research-only p50은 lineage 0.100 ms, capture 0.096 ms, pure JSON
serialization 2.539 ms다. standalone harness 자체 p50은 OFF 4.744 ms, ON 4.787 ms다.

## Exact parity

[parity_after_optimization.csv](parity_after_optimization.csv)에 각 material optimization gate를
기록했다. 최종 결과는 다음과 같다.

- frozen combined-seen ordered Top-12/digest/verdict/selection: OFF 86/86, ON 86/86
- production-success non-interference: 18/18
- frozen recovery and final selection reproduction: 40/40
- R3 constructed/validator maximum: 12/12
- package tests: known parameter/control mismatch 두 건을 제외한 25/25

두 known mismatch는 task 지시대로 수정하지 않았다. 세부 변경은
[optimization_log.md](optimization_log.md), install hygiene는
[build_hygiene_report.md](build_hygiene_report.md)에 있다.

## Reproduction

Release harness를 빌드한 뒤 다음을 실행한다.

```bash
python3 src/local_planning/test/profile_p3_r3_k12_runtime.py \
  --harness <build>/p3_r3_k12_integration_harness \
  --output /tmp/r3_k12_profile.csv

python3 src/local_planning/test/check_p3_r3_k12_parity.py \
  --harness <build>/p3_r3_k12_integration_harness --jobs 4

LOCAL_PLANNING_RESEARCH_PARITY=1 \
python3 src/local_planning/test/check_p3_r3_k12_parity.py \
  --harness <build>/p3_r3_k12_integration_harness --jobs 4
```

이 단계에서는 method/K/quota를 runtime에 맞춰 조정하지 않았고 production parameter를 바꾸지
않았으며 commit/push도 수행하지 않았다.
