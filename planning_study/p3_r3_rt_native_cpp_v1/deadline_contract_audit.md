# Local-planning deadline contract audit

## 결론

현재 운영 YAML이 정하는 nominal planning period는 **25 ms (40 Hz)**다. 다만 이 저장소는
deadline miss를 감지해 작업을 중단하거나 별도 real-time scheduler로 강제하지 않는다. 따라서
25 ms는 물리적으로 중요한 **운영 soft deadline / 주기 예산**이며, 코드가 보장하는 hard
deadline은 아니다.

Native R3-RT 단독 구간은 세 budget 모두 측정 max 17.13 ms 이하였지만, 선행 production
strict/relaxed ladder를 포함한 evaluator wall은 p95 40.40--41.90 ms, max 69.42--70.91 ms였다.
그러므로 현재 통합 형태를 25 ms 계약에 맞는다고 판정할 수 없다.

## Source/config evidence

- `LocalPlannerNode`의 선언 기본값은 50 ms다
  (`src/local_planning/src/local_planner_node.cpp:487`). 이것은 parameter가 없을 때의 fallback이다.
- 실차 운영 YAML과 simulation YAML은 모두 `planning_period_ms: 25`를 설정한다
  (`src/local_planning/config/local_planning.yaml:686`,
  `src/local_planning/config/local_planning_sim.yaml:452`).
- 표준 launch는 `local_planning.yaml`을 기본 parameter file로 선택하고
  (`src/local_planning/launch/local_planning.launch.py:36-38`), planner-only launch도 같은 파일을
  node에 전달한다 (`src/local_planning/launch/local_planner_only.launch.py:38-45,86-94`).
- non-lockstep 실행은 이 값을 그대로 wall timer period로 사용한다
  (`src/local_planning/src/local_planner_node.cpp:930-933`). Planning callback group은
  `MutuallyExclusive`다 (`src/local_planning/src/local_planner_node.cpp:863-877`).

즉 정상 launch의 effective contract는 25 ms이며, source fallback 50 ms를 runtime-effective
값으로 해석하면 안 된다.

## Upstream update contract

- obstacle detector는 `/scan` subscription callback에서 구동되고
  (`src/obstacle_detector/src/obstacle_detector_node.cpp:81-83,1562-1565`), confirmed/static output을
  매 scan마다 발행한다 (`.../obstacle_detector_node.cpp:1742-1749`). LiDAR scan rate 자체는 이
  repository의 planner parameter로 고정되지 않는다.
- Frenet odometry는 upstream odometry callback마다 변환된다
  (`src/global_planning/src/frenet_odom_node.cpp:291-316`). 따라서 이것도 local planner가 별도
  고정 rate를 강제하지 않는다.
- localization의 scan-gap watchdog은 40 Hz다
  (`src/kinematic_localization/config/kinematic_localization.yaml:99-106`). 이것은 정상 scan rate의
  선언이 아니라 scan gap 중 odom-only 출력의 fallback rate다.

따라서 planner의 40 Hz timer는 upstream scan/odom과 독립된 주기 소비자다. 입력은 최신
snapshot으로 drain되고, 센서 rate가 40 Hz라고 source만으로 단정할 수 없다.

## Can recovery span multiple nominal cycles?

현재 fresh selection은 한 callback 안에서 완결된다. Active maneuver가 유효하면
continuation-first branch가 evaluator 호출 전에 반환해 계산을 생략하지만
(`src/local_planning/src/local_planner_node.cpp:3320-3341`), continuation이 실패하면 같은 callback에서
fresh evaluator를 호출한다 (`.../local_planner_node.cpp:3343-3352`). Native bounded selector도 호출
내 stack-local state만 사용하며 다음 callback으로 proposal/ranking 작업을 넘기는 상태는 없다.

[INFERENCE] 현재 계약에서 recovery 계산을 임의로 여러 cycle에 나누면, 그동안 어떤 경로가
authority를 가지는지와 snapshot lineage가 바뀐다. 이는 단순 최적화가 아니라 lifecycle/fallback
설계 변경이므로 이 연구에서는 허용되지 않는다.

## Deadline classification

- Nominal period: **25 ms**.
- Enforced hard deadline: **없음**.
- Engineering interpretation: **soft real-time cycle budget**.
- Current native bounded R3 core: 25 ms 이내.
- Current preceding-ladder-inclusive evaluator: p95/p99/max가 25 ms 초과.
- Final timing gate: **FAIL for freeze readiness**.
