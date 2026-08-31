# Runtime impact of coverage miss

`runtime_by_group.csv`는 Release build에서 deterministic property 538개를 각각 한 번 실행한
진단값이다. warm repeated real-time benchmark가 아니므로 배포 latency 주장에는 쓰지 않는다.

- nonblocking semantic 101건은 후보를 만들지 않아 계산량은 작지만, 결과 reason 계약 때문에
  failure로 집계됐다. 실제 sequential planner에서 retry/safe-stop을 일으키는지는 이 task에서
  replay하지 않았으므로 `[INFERENCE]`로도 단정하지 않는다.
- Top-12/B128/lateral miss는 대체로 K=12 exact validator까지 소비하고도 선택되지 않는다.
  따라서 miss의 비용은 “0 candidate”가 아니라 최대 bounded work 후 실패하는 비용이다.
- 더 큰 B나 K는 이 audit에서 실행하지 않았다. B128 miss 2건과 Top-12 miss 9건은 비용 증가로
  회복 가능성이 있는 위치를 식별할 뿐, runtime tradeoff의 실측값은 아니다.
- raceline mismatch는 GQSC가 reconstruction 전에 중단돼 빠르지만 기능적으로 무효다. 낮은
  runtime을 장점으로 해석하면 안 된다.
