# Candidate-budget comparison

## Recovery

Exact-R3 teacher가 usable recovery를 가진 DEVELOPMENT+VALIDATION 32건과, pilot을 포함한 전체
허용 seen corpus의 36건을 함께 표시한다.

| B | DEV usable | VALIDATION usable | DEV+VAL retention | all-seen retention | all-seen hard recovery |
|---:|---:|---:|---:|---:|---:|
| 64 | 13/14 | 17/18 | 30/32 (93.75%) | 34/36 (94.44%) | 42 (exact-R3: 40) |
| 96 | 13/14 | 18/18 | 31/32 (96.88%) | 35/36 (97.22%) | 43 (exact-R3: 40) |
| 128 | 14/14 | 18/18 | 32/32 (100%) | 36/36 (100%) | 44 (exact-R3: 40) |

Hard recovery가 exact-R3보다 큰 것은 bounded method가 exact teacher Top-12와 다른 후보를
선택한 사건이 있기 때문이다. Oracle upper bound나 새로운 holdout 결과라는 뜻은 아니다.

## Native warm-process runtime

단위는 ms다. `R3 total`은 bounded recovery 자체, `full evaluator`는 같은 snapshot에 대한 선행
strict/relaxed production ladder와 bounded recovery를 모두 포함한다. Node publication과 ROS executor
queueing은 포함하지 않는다.

| B | R3 p50 | R3 p95 | R3 p99 | R3 max | full p50 | full p95 | full p99 | full max |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 64 | 5.323 | 12.907 | 16.836 | 17.125 | 13.190 | 40.396 | 68.938 | 69.422 |
| 96 | 5.957 | 13.773 | 16.827 | 16.939 | 13.825 | 40.976 | 70.124 | 70.759 |
| 128 | 6.339 | 14.288 | 16.835 | 16.985 | 14.455 | 41.904 | 70.506 | 70.909 |

이전 exact-R3 최적화 연구의 같은 86-event warm-process 결과는 R3 invocation p50/p95/p99/max
17.188/132.728/196.074/263.354 ms, full evaluator 29.798/166.053/223.225/303.858 ms였다
(`planning_study/p3_r3_k12_runtime_optimization_v1/before_after_runtime.csv`). 측정 campaign이 다르므로
절대적인 paired speedup 주장은 하지 않지만, bounded proposal이 factor-pool tail을 제거했다는 방향은
분명하다.

## Hard compute bounds

| B | lateral max | transition max | pair proxy max | reconstruction max | validator max |
|---:|---:|---:|---:|---:|---:|
| 64 | 36 | 7 | 64 | 12 | 12 |
| 96 | 48 | 7 | 96 | 12 | 12 |
| 128 | 64 | 7 | 128 | 12 | 12 |

전체 exact-R3 Cartesian factor space는 native bounded path에서 생성하지 않는다. Pair pool은
diagonal wave를 따라 budget 도달 시 중단한다.

## Interpretation

B128이 recovery를 완전히 보존하고 native R3 core도 25 ms 안에 들어오지만, 실제 호출 순서의
선행 ladder 포함 p95가 41.9 ms다. 따라서 preference rule만으로 B128을 freeze할 수 없으며, 세
budget 모두 현재는 timing optimization gate로 돌아가야 한다.
