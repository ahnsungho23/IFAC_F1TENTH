# 용어집

## Path

[GENERAL THEORY] 공간에 놓인 기하학적 길이다. “어디로 갈 것인가”를 말하지만 각 점을 언제 지날지는 반드시 포함하지 않는다. 현재 `/avoid_waypoints`는 속도 필드도 가지므로 순수 선보다 정보가 많지만, full time-indexed trajectory와는 구분한다.

## Trajectory

[GENERAL THEORY] 위치·속도·자세 등이 시간 `t`의 함수로 주어진 운동 계획이다. 같은 path도 2초에 통과하는 trajectory와 5초에 통과하는 trajectory는 횡가속·조향 demand가 다르다. 현재 P3를 complete dynamic trajectory optimizer라고 부르면 안 된다.

## Waypoint

[GENERAL THEORY] path를 이산적으로 표현하는 한 표본점이다. 위치뿐 아니라 `s,d,heading,curvature,speed` 같은 필드를 함께 가질 수 있다. [CURRENT IMPLEMENTATION] P3는 global waypoint 순서를 복사해 shifted waypoint 배열을 만든다.

## Raceline

[GENERAL THEORY] racing track에서 lap time이나 차량 한계를 고려해 선택한 기준 주행선이다. [CURRENT IMPLEMENTATION] local planner는 이 선의 ordered sample을 reference로 삼고 장애물 구간에서 `d(s)`만 바꾼 뒤 다시 `d=0`으로 합류한다.

## Frenet coordinates

[GENERAL THEORY] 기준 곡선을 따라가는 좌표 `s`와 기준 곡선에서 옆으로 떨어진 좌표 `d`로 위치를 표현하는 방식이다. 트랙 앞/뒤와 좌/우를 planning 문제에 직접 대응시킨다. self-near hairpin에서는 projection branch continuity가 중요하다.

## Arc length

[GENERAL THEORY] 곡선을 따라 실제로 잰 누적 거리다. 임의 매개변수 `q`에 대해 `ds=||dr/dq||dq`다. arc length로 미분하면 curvature가 “1 m 진행당 heading 변화”라는 물리 의미를 가진다.

## Tangent

[GENERAL THEORY] 곡선이 그 점에서 향하는 순간 방향이다. arc-length reference에서는 `t=dr/ds`가 unit vector다. heading `ψ`와 `t=[cosψ,sinψ]`로 연결된다.

## Normal

[GENERAL THEORY] tangent에 수직인 방향이다. 이 저장소의 Frenet 규약에서 left normal은 `n=[-sinψ,cosψ]`이고 양의 `d` 방향이다. `p=r+dn`이 Frenet-to-Cartesian 이동의 핵심이다.

## Heading

[GENERAL THEORY] path tangent가 global x축과 이루는 방향각이다. `atan2(y',x')`로 구한다. heading 자체와 heading이 거리당 변하는 curvature는 다른 양이다.

## Curvature

[GENERAL THEORY] path를 1 m 진행할 때 heading이 얼마나 변하는지를 나타내며 `κ=dψ/ds`다. 원에서는 크기가 `1/R`이고 좌우를 나타내는 부호를 가질 수 있다. [CURRENT IMPLEMENTATION] planner는 candidate Cartesian sample에서 이를 다시 계산한다.

## Curvature rate

[GENERAL THEORY] curvature가 얼마나 빨리 변하는지를 뜻한다. `dκ/ds`는 공간 변화율이고, `dκ/dt=v dκ/ds`는 시간 변화율이다. [CURRENT IMPLEMENTATION] hard validator는 인접 waypoint의 spatial rate를 검사한다.

## Spline

[GENERAL THEORY] 여러 polynomial segment를 knot에서 이어 만든 piecewise curve다. 하나의 고차 polynomial로 전체를 표현하는 것보다 local shape를 제어하고 수치적으로 다루기 쉽다. 연결점에서 어느 차수 derivative까지 같은지가 smoothness를 정한다.

## Quintic

[GENERAL THEORY] 최고 차수가 5인 polynomial이다. 계수가 여섯 개여서 양 끝의 위치·1차·2차 미분 여섯 조건을 맞출 수 있다. “quintic을 쓴다”만으로 안전성이나 novelty가 생기지는 않는다.

## Hermite

[GENERAL THEORY] 함수값뿐 아니라 derivative 값을 경계조건으로 사용해 polynomial을 구성하는 방식이다. quintic Hermite segment는 양 끝의 `d,d',d''`를 받는다. [CURRENT IMPLEMENTATION] P3는 knot마다 공유된 derivative와 acceleration을 양옆 segment에 준다.

## Knot

[GENERAL THEORY] spline segment가 만나는 기준 위치다. knot의 위치와 함수값, derivative 상태가 전체 spline 형상을 결정한다. [CURRENT IMPLEMENTATION] P3는 다섯 station knot와 네 segment를 사용한다.

## Candidate

[GENERAL THEORY] generator가 제안한 하나의 계획 가설이다. 생성되었다고 publish 가능한 것은 아니며 hard validation을 통과해야 한다. [CURRENT IMPLEMENTATION] P3 trace는 candidate identity, path digest, rejection reason을 남긴다.

## Hard constraint

[GENERAL THEORY] 위반하면 다른 장점으로 보상할 수 없이 후보를 탈락시키는 조건이다. wall, collision, curvature limit 같은 최소 허용 조건이 예다. ranking weight와 혼동하면 안 된다.

## Feasibility

[GENERAL THEORY] 주어진 제약과 모델 아래에서 실행 가능한지를 뜻한다. geometric feasibility, speed-profile feasibility, dynamic closed-loop feasibility는 서로 다른 수준이다. 현재 hard-valid는 현재 구현된 모델 범위의 feasibility이지 모든 실제 동역학의 증명이 아니다.

## Ranking

[GENERAL THEORY] feasible candidate가 여러 개일 때 선호 순서를 정하는 규칙이다. [CURRENT IMPLEMENTATION] 현재는 weighted sum이 아니라 exit, braking, speed loss, slack, deviation, generation order의 lexicographic comparison이다.

## Commitment

[GENERAL THEORY] 선택한 maneuver를 다음 sensor frame에서도 일정 조건 아래 유지하겠다는 계획 권한이다. [CURRENT IMPLEMENTATION] original P3 path와 obstacle identity/guard를 기록하고 current suffix가 hard-valid인 동안 geometry를 고정한다.

## Fallback

[GENERAL THEORY] 주 경로가 유효한 출력을 만들지 못할 때 사용하는 보수적 대안이다. [CURRENT IMPLEMENTATION] margin slow pass, braking safe-stop, stale-odometry emergency hold, global handoff 같은 서로 다른 fallback이 있다. fallback은 모두 같은 위험 수준이나 같은 geometry를 뜻하지 않는다.

## Safe-stop

[GENERAL THEORY] 위험 전에 차량을 정지시키기 위한 계획과 그 유지 정책이다. [CURRENT IMPLEMENTATION] 일반적으로 장애물 전까지 이어지는 non-empty braking path이며 마지막 waypoint 속도가 0이다. latch된 뒤에는 별도 release 조건을 만족해야 한다.

## Receding horizon

[GENERAL THEORY] 현재 상태에서 유한한 앞 구간을 반복 계획하고, 일부를 실행한 뒤 새 상태로 다시 계획하는 방식이다. 현재 local planner도 앞쪽 maneuver scope를 보지만, classic MPC처럼 연속 control sequence를 매 tick 최적화하는 것은 아니다. lifecycle 때문에 active suffix가 유효하면 horizon을 매번 새로 최적화하지 않는다.

## Planning authority

[GENERAL THEORY] 어떤 입력이나 상태가 path geometry, speed, stop, handoff 중 무엇을 바꿀 수 있는지를 뜻한다. [CURRENT IMPLEMENTATION] raw obstacle은 speed hint, confirmed static obstacle은 lateral path, safe-stop latch는 stop 유지 authority를 가진다.

## Guard

[CURRENT IMPLEMENTATION] fresh selection 때 conservative obstacle envelope를 frozen record로 저장해 progressive reveal의 작은 변화가 path를 흔들지 않게 하는 경계다. live same-ID envelope가 guard 안에 있는 동안 frozen geometry로 검증할 수 있다. guard가 깨져도 raw/retention exact validation이 후속 판단을 한다.

## Retention band

[CURRENT IMPLEMENTATION] 이미 committed된 path를 재검증할 때만 tracking reserve 일부를 완화하는 band다. 차량 physical half-width와 base clearance는 줄이지 않는다. fresh candidate 생성 margin을 줄이는 기능은 아니다.

## Handoff

[GENERAL THEORY] 한 planner/state의 경로 권한을 다음 상태로 안전하게 넘기는 절차다. [CURRENT IMPLEMENTATION] AVOID에서 GLOBAL 복귀가 확인되기 전까지 ego를 덮는 non-empty `d=0` loop를 발행할 수 있다.

## 자가 점검 질문

1. path와 trajectory는 무엇이 다른가?
2. heading, curvature, curvature rate의 단위와 관계를 설명하라.
3. hard constraint, feasibility, ranking은 어떤 순서로 연결되는가?
4. commitment, Guard, retention band의 책임 차이를 설명하라.
5. receding horizon이라고 해서 현재 planner가 MPC인 것은 아닌 이유를 설명하라.

