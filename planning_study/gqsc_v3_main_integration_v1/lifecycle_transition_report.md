# Lifecycle transition report

## 판정

Frozen GQSC v3의 stateless adapter는 exact하지만, 현재 fresh lifecycle과 collision-horizon 계약이
일치하지 않습니다. 분류는 `LIFECYCLE/METHOD_INTERFACE`이며 최종 상태는
`GQSC_V3_LIFECYCLE_BLOCKER`입니다.

## 정상적으로 보존된 순차 동작

fresh ownership이 성립한 56개 독립 snapshot 각각에 두 sequence를 적용했습니다.

1. `FRESH → HELD_SAME_INPUT → GUARD_FAIL_RAW_PASS → OBSTACLE_DISAPPEARANCE → FORWARD_TRIM → COMPLETE`
2. `FRESH → synthetic full-width blocker → INVALIDATE_AND_FALLBACK`

모든 56건에서 동일 입력과 detector dropout은 original candidate identity/path digest를 유지했고,
진행 시 원본 geometry를 바꾸지 않고 prefix만 잘랐으며, cluster 통과 뒤 complete했습니다. 새
blocker는 56/56 immutable suffix를 무효화했습니다. continuation 단계에서는 fresh generator를
다시 호출하지 않았습니다.

## Fresh ownership blocker 5건

| Event | Dataset | Evaluator horizon (m) | Lifecycle raw horizon (m) | 차이 (m) |
|---|---|---:|---:|---:|
| DVE004 | DEVELOPMENT | 1.898893 | 5.108793 | 3.209900 |
| DVE006 | DEVELOPMENT | 3.028851 | 5.597345 | 2.568495 |
| DVE007 | DEVELOPMENT | 2.497678 | 5.194034 | 2.696356 |
| VUE011 | VALIDATION_SEEN_AFTER_V1 | 3.146972 | 5.658452 | 2.511480 |
| VUE013 | VALIDATION_SEEN_AFTER_V1 | 2.961218 | 5.658129 | 2.696912 |

모든 실패 reason은 다음과 같습니다.

```text
FRESH_RAW_EXACT_HARD_INVALID:d-offset intersects an inflated static-obstacle box
```

## Source-level 원인

GQSC reconstruction의 exact validation은 다음 public scope rule을 사용합니다.

```text
H_eval = maneuverScopeEnd(ego, obstacles, selected_cluster_ids, z3)
```

`maneuverScopeEnd`는 `z3 + post_merge_lookahead`보다 앞에 다음 cluster가 있으면 그 expanded
front에서 horizon을 자릅니다. 반면 `P3ManeuverLifecycle::selectFresh`의 guarded certificate는
evaluator 결과를 재사용하지만 raw geometry는 다음 식으로 다시 검사합니다.

```text
H_fresh = selected_cluster_end_forward_m + post_merge_lookahead_m
```

따라서 동일 ego, 동일 raw obstacle snapshot, 동일 selected path임에도 raw check가 evaluator가
책임지지 않은 다음 cluster까지 포함합니다. 5건은 `H_eval`에서는 hard-valid였지만 더 긴
`H_fresh`에서 obstacle collision으로 거부됐습니다. 이는 validator tolerance나 verdict의 차이가
아니라 validator에 넘긴 obstacle horizon의 차이입니다.

## 왜 이 작업에서 고치지 않았는가

사용자 계약은 lifecycle, validator, frozen method를 보존하고 interface failure를 분류하라고
명시했습니다. 두 horizon 중 하나를 바꾸면 lifecycle 책임 범위 또는 frozen evaluation contract가
달라집니다. 따라서 GQSC operator나 후보 순서를 조정하지 않았고 lifecycle도 수정하지 않았습니다.

closed-loop smoke 전에 별도 승인된 integration repair에서 다음 계약 중 하나를 명시적으로
결정하고 동일 scope를 fresh/continuation/chain/safe-stop escape 전체에 적용해야 합니다.

- evaluator의 `maneuverScopeEnd`를 lifecycle도 그대로 사용
- frozen evaluator contract를 더 긴 lifecycle horizon으로 다시 정의

첫 선택이 현재 source의 “다음 cluster는 chain maneuver가 책임진다”는 기존 주석과 맞지만,
이 문서는 수정 승인을 대신하지 않습니다.
