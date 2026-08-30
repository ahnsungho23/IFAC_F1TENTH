# 이 교재를 공부하는 법

## 이 문서 묶음의 기준

[CURRENT IMPLEMENTATION] 이 교재는 `planning_cleanup` 브랜치의 `b4e54ebc3b915d28609353c7e1c93f3ec1cec164`에서 현재 소스를 다시 읽어 작성했다. 기존 `planning_study/01_architecture`부터 `06_research`까지는 좋은 참고 자료지만, 구현 사실의 최종 근거는 항상 `src/`의 현재 코드와 설정이다. 소스가 바뀌면 링크의 줄 번호와 설명도 다시 확인해야 한다.

태그는 다음처럼 읽는다.

- `[CURRENT IMPLEMENTATION]`: 현재 저장소에서 직접 확인한 동작이다.
- `[GENERAL THEORY]`: 특정 코드와 무관한 수학·차량 이론이다.
- `[INFERENCE]`: 코드와 이론을 연결한 해석이며, 런타임 실측 그 자체는 아니다.
- `[OPEN QUESTION]`: 소스만으로 답할 수 없거나 실험이 필요한 문제다.

## 권장 학습 순서

```text
큰 그림
  ↓
perception 계약 ──→ Frenet 좌표
                       ↓
                 P3와 5차 경로
                       ↓
              곡률 ──→ 차량 실현성
                       ↓
                검증 ──→ 순위
                       ↓
                  lifecycle
                       ↓
             실패 분석과 연구 질문
```

1. [`01_planning_big_picture.md`](./01_planning_big_picture.md)에서 입력부터 출력까지 한 문장으로 말할 수 있게 한다.
2. [`02_perception_to_planning.md`](./02_perception_to_planning.md)에서 어떤 장애물 배열이 경로 권한을 갖는지 구분한다.
3. [`03_frenet_coordinates.md`](./03_frenet_coordinates.md)를 종이에 그려 `s`, `d`, tangent, normal을 익힌다.
4. [`04_p3_and_quintic_path.md`](./04_p3_and_quintic_path.md)의 다섯 knot와 네 개의 quintic segment를 손으로 재구성한다.
5. [`05_curvature.md`](./05_curvature.md)와 [`06_vehicle_feasibility.md`](./06_vehicle_feasibility.md)를 연달아 공부한다. 경로의 기하와 차량의 조향·속도를 분리해서 생각해야 한다.
6. [`07_candidate_validation.md`](./07_candidate_validation.md)과 [`08_candidate_ranking.md`](./08_candidate_ranking.md)에서 “탈락”과 “선호”를 구분한다.
7. [`09_planning_lifecycle.md`](./09_planning_lifecycle.md)에서 한 프레임의 최적 경로보다 여러 프레임의 일관성이 왜 중요한지 본다.
8. 마지막으로 [`10_failures_and_limitations.md`](./10_failures_and_limitations.md), [`11_research_questions.md`](./11_research_questions.md), [`self_test.md`](./self_test.md)를 사용한다.

## 외울 것과 유도할 것

외우지 말고 매번 유도해야 하는 항목은 다음과 같다.

- `[GENERAL THEORY]` `p(s)=r(s)+d(s)n(s)`에서 normal의 부호와 Cartesian 좌표를 직접 전개한다.
- `[GENERAL THEORY]` 임의 매개변수 `q`에 대한 곡률식을 유도하고, `q=s`일 때 왜 단순해지는지 확인한다.
- `[GENERAL THEORY]` `a_y=v²κ`와 `κ=tan(δ)/L`을 단위까지 검사한다.
- `[GENERAL THEORY]` quintic의 여섯 계수가 양 끝의 `d,d',d''` 여섯 조건을 만족하는지 대입한다.
- `[CURRENT IMPLEMENTATION]` `CandidateRankKey` 두 개를 위에서 아래로 비교해 실제 승자를 손으로 고른다.
- `[CURRENT IMPLEMENTATION]` safe-stop 해제의 네 경로를 top-level OR로 다시 그린다.

기억해도 되는 것은 용어와 책임 경계다. 예를 들어 local planner는 confirmed static obstacle에 대해 `d(s)` 경로를 만들고, raw obstacle은 별도의 감속 힌트만 준다. 반면 특정 숫자는 YAML·launch·런타임 override에 따라 달라질 수 있으므로 암기 대상이 아니다.

## 숫자 예제를 푸는 방법

[GENERAL THEORY] 다음처럼 단위표를 먼저 만든다.

| 기호 | 뜻 | 단위 |
|---|---|---|
| `s` | 기준선 누적 길이 | m |
| `d` | 기준선 왼쪽 방향 횡오프셋 | m |
| `κ` | 부호 있는 곡률 | 1/m |
| `v` | 속도 | m/s |
| `a_y` | 횡가속도 | m/s² |
| `δ` | 앞바퀴 등가 조향각 | rad |
| `L` | wheelbase | m |

예를 들어 `v=4 m/s`, `κ=0.5 1/m`이면 `a_y=4²×0.5=8 m/s²`이다. 답만 쓰지 말고 “속도가 두 배가 되면 횡가속도는 네 배”라는 물리 의미를 붙인다.

## 준비도 기준

다음 질문에 그림과 식으로 답할 수 있으면 다음 단계로 넘어간다.

- perception 준비 완료: `/static_obs`, `/confirmed_static_obs`, `/opp_obs`의 역할과 local planner 권한을 구분한다.
- Frenet 준비 완료: 직선·원호·hairpin에서 동일한 `(s,d)` 직관과 branch ambiguity를 설명한다.
- P3 준비 완료: 다섯 station과 `[ego_d,target,middle,target,0]`를 보고 네 segment를 그린다.
- 곡률 준비 완료: 세 점 Menger 곡률과 미분 곡률의 장단점을 설명한다.
- feasibility 준비 완료: 같은 곡률도 속도가 다르면 왜 가능/불가능이 달라지는지 계산한다.
- planner 준비 완료: hard validation을 통과한 후보만 ranking된다는 것을 예로 보인다.
- lifecycle 준비 완료: 새 후보가 더 좋아 보여도 committed suffix를 유지하는 이유를 설명한다.

## 소스 읽기 최소 규칙

[CURRENT IMPLEMENTATION] C++ 문법을 전부 알 필요는 없다. 함수의 입력, 반환값, `if`의 탈락 조건, 호출 순서만 표시해도 충분하다. 첫 회독에서는 다음 연결만 따라간다.

- timer와 전체 안전 사이클: [`local_planner_node.cpp:3507`](../../src/local_planning/src/local_planner_node.cpp#L3507), [`local_planner_node.cpp:3775`](../../src/local_planning/src/local_planner_node.cpp#L3775)
- P3 후보 생성: [`p3_shadow.cpp:75`](../../src/local_planning/src/p3_shadow.cpp#L75)
- 공통 hard validator: [`raceline_spline_planner.cpp:2805`](../../src/local_planning/src/raceline_spline_planner.cpp#L2805)
- 공통 ranking: [`candidate_rank.hpp:75`](../../src/local_planning/include/local_planning/candidate_rank.hpp#L75)
- committed suffix: [`p3_maneuver_lifecycle.cpp:272`](../../src/local_planning/src/p3_maneuver_lifecycle.cpp#L272)

[OPEN QUESTION] 소스 정적 분석은 실제 차량에서 어떤 파라미터 override가 적용됐는지, 추종오차가 얼마였는지, 후보 cap이 얼마나 자주 포화됐는지를 증명하지 않는다. 그 질문은 동일 commit·동일 설정을 고정한 replay와 차량 로그가 필요하다.

## 반드시 설명할 수 있어야 하는 질문

1. 네 태그는 어떤 증거 수준을 구분하는가?
2. 왜 파라미터 숫자보다 책임 경계를 먼저 공부해야 하는가?
3. P3 전에 Frenet을, ranking 전에 validation을 공부해야 하는 이유는 무엇인가?
4. `a_y=v²κ` 예제에서 속도 두 배가 횡가속도 네 배가 되는 이유는 무엇인가?
5. 정적 소스 분석만으로 확인할 수 없는 것은 무엇인가?

