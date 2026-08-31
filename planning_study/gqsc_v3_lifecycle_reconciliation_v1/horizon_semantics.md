# 장애물 충돌 horizon 의미 복원

## 좌표와 공통 validator 계약

ego station을 `s_e`, 현재 expanded obstacle cluster의 전면/후면 상대거리를 각각
`r_C,start`, `r_C,end`, 설정값을 `L_merge = post_merge_lookahead_m = 5.0 m`라고 둔다.
`forwardDistance()`가 폐곡선 wrap을 처리하므로 아래 모든 거리는 현재 ego 기준 전방거리다.

`validateCandidate()`의 horizon은 **obstacle collision loop만** 제한한다. ordered reference에서 만든
path 전체에 대한 최소 점수, 진행 방향, entry continuity, 회전 vehicle footprint 대 track bound,
곡률/곡률률 검사는 horizon 밖에서도 그대로 수행된다. 따라서 짧은 collision horizon은 path를
자르거나 footprint/track 검사를 줄이는 옵션이 아니다.

## A. frozen GQSC evaluator horizon

현재 선택/생성 evaluator가 쓰는 식은 다음과 같다.

```text
r_nominal = r_C,end + L_merge
r_next = min(expanded obstacle.start)
         over obstacle.id not in current_cluster
         and expanded obstacle.start > r_C,end + epsilon

H_eval = r_nominal                                      if r_next does not exist
       = max(r_C,end, min(r_nominal, r_next))           otherwise
```

구현은 `RacelineSplinePlanner::maneuverScopeEnd()` 한 곳이다. GQSC의 현재 cluster ID와 P3 knot
`z3`의 cluster-end 상대거리를 넘기며, 반환값을 `buildCandidate()`의 exact validation certificate에
기록한다. 시작점은 현재 ego 뒤가 아니라 candidate path의 첫 ordered forward sample이고, obstacle
collision 검사는 그 sample부터 `H_eval` 이하의 sample까지만 한다.

`r_next` obstacle은 boundary를 정의하기 위해 expanded interval 목록에는 나타날 수 있지만, 그
expanded front보다 전진한 path sample을 검사하지 않으므로 다음 obstacle 몸체를 현재 maneuver의
충돌 책임에 포함하지 않는다. current cluster는 반드시 포함되며 `max(r_C,end, ...)` 때문에 자기
cluster 후면보다 앞에서 절단될 수 없다.

## B. 수정 전 lifecycle raw-revalidation horizon

수정 전 `P3ManeuverLifecycle::selectFresh()`와 continuation은 다음 값을 독립 재구성했다.

```text
H_raw_pre = r_C,end + L_merge
```

fresh에서는 selected cluster-end를 그대로 사용했고, continuation에서는 frozen cluster-end의 현재
ego 상대 전방거리와 5 m를 합했다. `r_next` 절단이 없으므로 5건에서는 GQSC가 경계 밖으로 둔 다음
cluster 또는 그 뒤 obstacle까지 raw collision loop에 들어왔다. path 시작은 같은 candidate의 현재
suffix였지만 obstacle horizon만 달랐다.

## 다섯 반례의 수치

| event | `r_C,end` (m) | `H_eval` (m) | `H_raw_pre` (m) | evaluator/pre-fix sample | boundary를 정한 다음 obstacle | pre-fix 충돌 obstacle |
|---|---:|---:|---:|---:|---:|---:|
| DVE004 | 0.108793 | 1.898893 | 5.108793 | 8 / 21 | 17 | 19 |
| DVE006 | 0.597345 | 3.028851 | 5.597345 | 12 / 22 | 79 | 79 |
| DVE007 | 0.194034 | 2.497678 | 5.194034 | 10 / 21 | 15 | 15 |
| VUE011 | 0.658452 | 3.146972 | 5.658452 | 13 / 23 | 76 | 76 |
| VUE013 | 0.658129 | 2.961218 | 5.658129 | 12 / 22 | 76 | 76 |

모두 같은 path digest가 `H_eval`에서는 hard-valid이고 `H_raw_pre`에서는 obstacle collision이다.
전체 interval과 실패 waypoint는 [five_event_diagnosis.csv](five_event_diagnosis.csv)에 보존했다.
DVE004처럼 boundary obstacle 17 뒤의 obstacle 19에서 실패하는 경우와, boundary를 정한 obstacle
자체의 interior sample까지 넓혀 실패하는 경우가 모두 있다.

## merge/exit와 긴 raw path의 관계

P3 path의 `z4`와 controller용 global tail은 collision certificate보다 멀리 갈 수 있다. 이 길이는
현재 obstacle을 피한 뒤 race line으로 합류하고 controller lookahead를 공급하기 위한 geometry다.
다음 obstacle까지 한 path가 영구 책임진다는 뜻이 아니다. production에는 현재 obstacle 통과 후
`tryEarlyChainedManeuver()`와 `beginChainedManeuverIfNeeded()`가 다음 cluster를 별도 snapshot으로
재계획하는 경로가 있다. 따라서 이 5건의 긴 tail만으로 `PATH_MERGE_INSUFFICIENT`라고 판정할 근거는
없다.

## historical rationale

- `19f5c842b5c0b63fdb2821499ea7c2a4f3b9a71e` (2026-08-16 09:55 +09): 후보 선택과
  commitment 검증의 maneuver horizon을 통일하려는 첫 변경.
- `301e8e814faaa8d5eac0829765dc3548cbd7cf76` (2026-08-16 18:14 +09): 반대쪽 통과가 필요한
  두 obstacle이 4.2 m 간격일 때 앞 maneuver가 뒤 obstacle까지 함께 만족할 수 없던 1179/1401
  callback failure를 근거로 `maneuverScopeEnd()`의 next-cluster 절단을 도입. commit message는 다음
  obstacle을 chaining이 맡는다고 명시한다.
- `b942c867415fd20b1bab240e179cc373dee60ff6` (2026-08-16 23:37 +09): `plan()` 후보 재검증의
  세 번째 call site도 같은 helper로 통일.
- 현재 checkout은 `4437eea0912e4d15c32214385d303794b4be6292`의 import history를 거쳤고,
  evaluator/helper는 절단 의미를 유지했지만 P3 lifecycle fresh/continuation 두 곳은 nominal 5 m
  식으로 남아 있었다.

## 판정

다섯 건의 pre-fix 분류는 모두 `LIFECYCLE_REVALIDATION_OVERREACH`다. 이는 “짧은 GQSC path가
다음 obstacle을 못 피했다”는 판단이 아니라, 같은 frozen path를 두 validator call site가 서로 다른
spatial contract로 심판한 결함이다.
