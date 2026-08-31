# VUE019 worst-case audit

- Dataset: `VALIDATION_SEEN_AFTER_V1`
- Frozen exact R3 lateral factors: 4,471
- Frozen exact R3 unique pair proxies: 71,506
- Frozen exact R3 K12 hard/usable recovery: `False` / `False`
- Reference Oracle v2 hard/usable feasible: `False` / `False`
- R3-RT B128 lateral retained / transition / pair proxies:
  `64` / `7` / `128`
- R3-RT reconstruction / raw validator calls:
  `12` / `12`
- R3-RT hard/usable recovery: `False` / `False`
- Teacher Top-12 unique recall: `0/12`

VUE019에는 exact R3의 useful selected solution 자체가 없고 Reference Oracle v2 domain도
hard/usable infeasible였다. 따라서 "useful solution survival"은 적용 불가다. 확인 가능한
결론은 factor proxy 계산을 71,506에서 정확히 128로 제한하면서 새로운 usable recovery를
허위로 만들지 않았다는 것이다. 이는 runtime stress evidence이지 recovery recall evidence가
아니다.
