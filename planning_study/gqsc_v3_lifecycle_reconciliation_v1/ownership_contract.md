# P3 maneuver ownership contract

## 결론

선택지 중 가장 가까운 것은 **C: 다음 obstacle 전에 재계획할 때까지**다. 더 정확한 production
계약은 다음처럼 명시적인 interval이다.

```text
owned obstacle-collision interval at creation = [0, H_eval]
H_eval = maneuverScopeEnd(creation ego, selected obstacle snapshot,
                          current cluster ids, current cluster end)
```

이 경계는 선택 시의 절대 track station으로 고정한다. ego가 monotonic하게 `p` m 진행한 active
suffix에서는 남은 상대 horizon이 `H_active = H_eval - p`다. 매 callback 새 obstacle 앞에서
`maneuverScopeEnd()`를 다시 계산하지 않는다. 그렇게 재절단하면 이미 소유한 구간 안에 새로 나타난
blocker를 horizon 밖으로 숨길 수 있기 때문이다.

## 책임 분리

| 구간 | owner | 검증 의미 |
|---|---|---|
| current-obstacle avoidance | 현재 immutable P3 record | current expanded cluster 전체를 obstacle collision 검사 |
| merge-to-raceline | 현재 immutable P3 record | nominally cluster rear 뒤 `post_merge_lookahead_m`; 다음 cluster 전면이 더 가까우면 거기까지 |
| next-obstacle interaction | next chained maneuver | 기존 next cluster가 frozen boundary를 정하며, 현재 certificate는 그 expanded front 안으로 진입하지 않음 |
| active suffix | 같은 immutable P3 record | 이미 지난 prefix만 제거하고 frozen spatial boundary까지 guarded/raw exact revalidation |
| full published path geometry | 현재 path publisher/controller | obstacle horizon과 무관하게 전체 path의 track/footprint/curvature/ordering 검사 |

## 기존 next obstacle과 새 blocker

- **선택 시 이미 존재한 next cluster:** frozen boundary 밖이며 chaining 대상이다. raw path tail이 그
  obstacle 쪽으로 연장됐다는 이유만으로 current maneuver를 즉시 폐기하지 않는다.
- **선택 후 boundary 안에 나타난 obstacle/envelope growth:** 현재 maneuver가 이미 책임진 공간이므로
  guarded/raw validator가 검사하고 충돌이면 즉시 invalidation한다.
- **boundary 밖 새 obstacle:** 다음 maneuver stabilization/chaining이 담당한다. 현재 path의 전체
  track/footprint geometry 검사는 여전히 유지된다.

새 단위 테스트 `FrozenOwnershipBoundaryExcludesExistingNextClusterButCatchesNewBlocker`는 기존 next
cluster가 경계 밖에 있을 때 continuation이 유지되면서도, 그보다 앞의 소유 구간에 새 full-width
blocker를 추가하면 `CURRENT_RAW_OBSTACLE_COLLISION`으로 무효화됨을 고정한다.

## callback/replanning 근거

운영 `planning_period_ms=25`이므로 fresh input을 40 Hz로 본다. 다음 obstacle은
`tryEarlyChainedManeuver()`가 current cluster를 통과한 뒤 old merge 전에도 선제 계획할 수 있고,
merge/completion/handoff 단계의 `beginChainedManeuverIfNeeded()`가 plan-then-swap을 수행한다. 따라서
current raw tail 전체를 현재 obstacle certificate가 영구 보증해야 한다는 B 계약은 production의
chaining 구조와 맞지 않는다.

이 계약은 안전 검사를 줄이는 새로운 정책이 아니다. 2026-08-16에 이미 후보 평가/legacy plan
재검증에 도입된 next-cluster ownership을 lifecycle metadata와 continuation에 복원한 것이다.
