# 최소 integration fix와 안전성 논증

## diagnosis-first 결과

수정 전 별도 horizon harness로 다섯 path를 같은 digest/geometry로 두 번 exact validation했다.
`H_eval`에서는 5/5 hard-valid, nominal `cluster_end + 5 m`에서는 0/5 hard-valid였다. 실패 sample은
모두 next-cluster boundary 이후였다. 따라서 GQSC factor, P3 geometry, validator tolerance를 바꾸기
전에 `LIFECYCLE_REVALIDATION_OVERREACH`로 분류했다.

duplicate audit도 먼저 callback 단계를 분리했다. outer TEST_ACTIVE evaluator와 fallback
`plan()` primary는 ego/obstacle/reference가 같을 때 순수 결정 함수의 같은 입력이었다. 반면
safe-stop escape와 chaining은 ego 또는 obstacle set이 달라 별도 계산이 필요했다.

## 구현한 최소 변경 1: explicit ownership certificate

- 각 exact-validated candidate trace에 실제 사용한
  `obstacle_collision_horizon_forward_m`를 기록한다.
- selected result가 그 값을 그대로 전달한다.
- fresh lifecycle의 certificate reuse, guarded fallback, raw validation이 모두 같은 selected 값을 쓴다.
- immutable record가 creation-relative 값을 보관하고 continuation은 monotonic progress만 빼서 같은
  absolute track boundary를 유지한다.

안전성은 다음 이유로 보존된다.

1. exact validator의 항목/마진/tolerance는 바뀌지 않았다.
2. track/rotated-footprint/curvature/ordering은 여전히 path 전체를 검사한다.
3. current expanded cluster rear보다 앞에서 자를 수 없다.
4. 선택 뒤 frozen owned interval 안에 새 obstacle이 나타나면 현재 snapshot의 guarded/raw 검사로
   즉시 invalidation한다.
5. boundary 밖 next cluster는 기존 40 Hz chaining/plan-then-swap과 safe-stop ladder가 담당한다.

현재 snapshot으로 helper를 매 callback 재계산하는 대안은 채택하지 않았다. 새 blocker가 owned
interval 안에 나타났을 때 바로 그 blocker front로 horizon을 줄이면 검사 대상에서 스스로 빠져
안전하지 않다. full raw path 전체를 현재 maneuver가 검사하는 대안도 역사적으로 반대측 연속
obstacle에서 가능한 모든 current path를 전멸시킨다.

## 구현한 최소 변경 2: exact same-input result reuse

outer evaluator result에 ego scalars, ordered full obstacle messages, planner parameter/reference revision을
input witness로 보관한다. fallback `plan()`은 witness가 전부 exact-match할 때만 candidate generation
결과를 재사용한다. 이후 side lock filtering, candidate reconstruction metrics 재측정, exact validator,
final rank/speed shaping은 기존 경로대로 실행한다.

이는 memoized method output일 뿐 output shortcut이 아니다. 다른 stop-point ego, next-cluster input,
parameter/reference revision, message order/field가 하나라도 달라지면 새 GQSC 평가를 수행한다. 연구
계측에는 cache hit counter만 추가했으며 기본-OFF decision behavior를 바꾸지 않는다.

## re-evaluation gates

- frozen stateless proposal/path/verdict/selected digest parity: 104/104 exact
- GQSC hard-valid 61건 fresh ownership 및 evaluator/lifecycle horizon: 61/61 exact
- same-input continuation, guarded/raw fallback, dropout, prefix trim, completion: 각 61/61
- owned interval 안 새 blocker invalidation: 61/61
- pair/reconstruction/validator 최대: 128/12/12
- 동일-input 중복 새 평가: 0; cache reuse 43
- lifecycle unit suite: 20/20

다만 이 fix는 frozen method의 기존 coverage miss를 수리하지 않는다. production scenario 3개,
generator-independent tight-gap/커버리지 property는 그대로 실패하며, 그 실패를 감추도록 test ceiling이나
method를 변경하지 않았다.
