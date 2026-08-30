# Local planning self-test

[GENERAL THEORY] 먼저 답을 보지 말고 그림·식·단위·현재 구현 근거를 함께 써 본다. “코드에 있다”만 쓰지 말고 왜 필요한지 설명해야 통과다.

## 00. 학습 방법

1. `S00-1` 네 태그는 어떤 증거 수준을 구분하는가?
2. `S00-2` 왜 파라미터 숫자보다 책임 경계를 먼저 공부해야 하는가?
3. `S00-3` P3 전에 Frenet을, ranking 전에 validation을 공부해야 하는 이유는 무엇인가?
4. `S00-4` `a_y=v²κ`에서 속도 두 배가 횡가속도 네 배가 되는 이유는 무엇인가?
5. `S00-5` 정적 소스 분석만으로 확인할 수 없는 것은 무엇인가?

## 01. Planning big picture

1. `S01-1` local planner가 global waypoint 순서를 유지하는 이유는 무엇인가?
2. `S01-2` `runSafetyPlanningCycle()`은 어떤 핵심 질문들을 조정하는가?
3. `S01-3` 현재 P3와 `P0_BACKUP_ONLY`는 어떤 관계인가?
4. `S01-4` hard-valid 후보와 selected 후보의 차이는 무엇인가?
5. `S01-5` continuation-first가 경로 jitter와 계산량을 동시에 줄이는 이유는 무엇인가?

## 02. Perception to planning

1. `S02-1` `/static_obs`가 완전히 raw한 scan point 배열이 아닌 이유는 무엇인가?
2. `S02-2` `/confirmed_static_obs`만 P3 lateral geometry authority를 갖는 이유는 무엇인가?
3. `S02-3` Frenet obstacle envelope의 네 경계는 각각 무엇을 뜻하는가?
4. `S02-4` 정적 collision과 동적 collision의 수학적 차이는 무엇인가?
5. `S02-5` raw slowdown이 false positive를 lateral jitter로 증폭하지 않는 이유는 무엇인가?

## 03. Frenet coordinates

1. `S03-1` `p=r+dn`에서 `n=[-sinψ,cosψ]`가 되는 이유는 무엇인가?
2. `S03-2` arc length로 매개화하면 tangent와 curvature 계산이 왜 쉬워지는가?
3. `S03-3` `p'=(1-κ_rd)t+d'n`의 두 항은 각각 무엇을 뜻하는가?
4. `S03-4` hairpin에서 stateless nearest projection이 위험한 이유는 무엇인가?
5. `S03-5` 현재 planner의 raceline-lock는 어떤 ordering 검사를 통해 보존되는가?

## 04. P3 and quintic

1. `S04-1` 다섯 station과 `[d_e,d_t,d_m,d_t,0]`는 각각 무엇을 뜻하는가?
2. `S04-2` quintic이 여섯 boundary condition에 자연스러운 이유는 무엇인가?
3. `S04-3` 현재 P3가 textbook single smoothstep과 다른 점은 무엇인가?
4. `S04-4` knot state 공유가 `C²`를 만드는 이유는 무엇인가?
5. `S04-5` M0/M1과 polynomial 차수를 혼동하면 안 되는 이유는 무엇인가?
6. `S04-6` analytic root 뒤에도 exact validator가 필요한 이유는 무엇인가?

## 05. Curvature

1. `S05-1` 임의 parameter `q`의 Cartesian curvature 식을 어떻게 유도하는가?
2. `S05-2` `atan2`가 `atan`보다 필요한 이유는 무엇인가?
3. `S05-3` 현재 local cubic 방식이 P3 polynomial을 직접 미분하는 방식과 어떻게 다른가?
4. `S05-4` constant `d`인데도 원호의 curvature가 바뀌는 이유는 무엇인가?
5. `S05-5` `dκ/ds`와 steering angle rate가 같은 것이 아닌 이유는 무엇인가?

## 06. Vehicle feasibility

1. `S06-1` `κ=tanδ/L`은 어떤 가정에서 나오는가?
2. `S06-2` `a_y=v²κ` 때문에 고속에서 같은 path가 더 어려워지는 이유는 무엇인가?
3. `S06-3` current controller의 understeer 식과 kinematic exact 식은 어떻게 다른가?
4. `S06-4` 현재 planner가 speed feasibility를 반영하는 다섯 계층은 무엇인가?
5. `S06-5` spatial curvature rate가 steering actuator guarantee가 아닌 이유는 무엇인가?
6. `S06-6` 현재 source에서 확인되지 않는 dynamic feasibility 항목은 무엇인가?

## 07. Candidate validation

1. `S07-1` generator와 validator를 분리해야 하는 이유는 무엇인가?
2. `S07-2` entry discontinuity가 두 조건의 AND인 이유는 무엇인가?
3. `S07-3` center track bound와 footprint track bound는 어떻게 다른가?
4. `S07-4` obstacle collision에만 maneuver horizon을 적용하는 이유는 무엇인가?
5. `S07-5` lateral acceleration이 현재 hard reject가 아닌데도 어떻게 반영되는가?
6. `S07-6` hard-valid가 dynamic closed-loop safety guarantee가 아닌 이유는 무엇인가?

## 08. Candidate ranking

1. `S08-1` hard validation과 ranking의 책임은 어떻게 다른가?
2. `S08-2` 실제 일곱 단계 lexicographic 우선순위는 무엇인가?
3. `S08-3` weighted sum과 lexicographic ranking의 trade-off 방식은 어떻게 다른가?
4. `S08-4` curvature가 현재 ranking에 들어가는 정확한 경로는 무엇인가?
5. `S08-5` braking deficit 0인 후보가 speed loss보다 먼저 우선되는 이유는 무엇인가?
6. `S08-6` generation index가 deterministic behavior에 필요한 이유는 무엇인가?

## 09. Planning lifecycle

1. `S09-1` continuation-first가 단순 previous-path preference와 다른 이유는 무엇인가?
2. `S09-2` fresh selection이 guarded와 raw validation을 모두 요구하는 이유는 무엇인가?
3. `S09-3` retention band에서 줄어드는 것과 절대 줄지 않는 것은 무엇인가?
4. `S09-4` completion check가 suffix validation보다 먼저인 이유는 무엇인가?
5. `S09-5` AVOID에서 non-empty handoff loop가 필요한 이유는 무엇인가?
6. `S09-6` safe-stop release A/B/C/D는 어떤 top-level 논리로 결합되는가?

## 10. Failures and limitations

1. `S10-1` ego-entry discontinuity 수정이 왜 단순 slope gate에서 AND gate로 발전했는가?
2. `S10-2` waypoint별 speed cap만으로 제동 feasibility를 보장할 수 없는 이유는 무엇인가?
3. `S10-3` maneuver collision horizon을 generation과 continuation에서 같게 써야 하는 이유는 무엇인가?
4. `S10-4` 안전하게 정지하는 것과 정지 후 탈출 가능한 것은 왜 다른가?
5. `S10-5` 현재 구현된 항목 중 research novelty로 다시 주장하면 안 되는 것은 무엇인가?
6. `S10-6` cap-24 candidate 전멸이 continuous infeasibility proof가 아닌 이유는 무엇인가?

## 11. Research questions

1. `S11-1` 이미 구현된 기능과 research gap을 어떻게 구분하는가?
2. `S11-2` 후보 A에서 cap-24 실패와 continuous infeasibility는 왜 다른가?
3. `S11-3` 후보 B가 단순 bicycle rollout 추가로 끝나면 novelty가 약한 이유는 무엇인가?
4. `S11-4` 후보 C에서 per-frame error quantile을 maneuver risk로 바로 부를 수 없는 이유는 무엇인가?
5. `S11-5` 후보 D의 safety와 liveness property는 각각 무엇인가?
6. `S11-6` 네 연구 후보를 시작하기 전에 어떤 baseline을 고정해야 하는가?

## 용어 종합

1. `SG-1` path와 trajectory는 무엇이 다른가?
2. `SG-2` heading, curvature, curvature rate의 단위와 관계를 설명하라.
3. `SG-3` hard constraint, feasibility, ranking은 어떤 순서로 연결되는가?
4. `SG-4` commitment, Guard, retention band의 책임 차이를 설명하라.
5. `SG-5` receding horizon이라고 해서 현재 planner가 MPC인 것은 아닌 이유를 설명하라.

# Answers

## 00. 학습 방법 정답

1. `S00-1` `[CURRENT IMPLEMENTATION]`은 source-confirmed 사실, `[GENERAL THEORY]`는 일반 이론, `[INFERENCE]`는 둘을 연결한 해석, `[OPEN QUESTION]`은 추가 실험이나 분석이 필요한 주장이다.
2. `S00-2` 책임 경계는 어떤 입력이 path·speed·stop을 바꾸는지 결정하지만 숫자는 YAML·launch·override로 변할 수 있다. 경계를 알아야 숫자가 어느 식과 판단에 들어가는지도 해석할 수 있다.
3. `S00-3` P3의 독립변수와 corridor가 `s,d`이므로 Frenet이 선행되어야 한다. ranking은 hard-valid 집합만 다루므로 무엇이 먼저 탈락하는지 모르면 선택 정책을 오해한다.
4. `S00-4` 원운동의 `a_y=v²/R`에서 `κ=1/R`이므로 `a_y=v²κ`다. `κ`가 같을 때 `v`가 2배면 제곱항이 4배가 된다.
5. `S00-5` 실제 runtime parameter, 실행 binary, sensor/actuator latency, physical tracking error와 collision safety는 source만으로 확정할 수 없다.

## 01. Planning big picture 정답

1. `S01-1` obstacle·track width·FSM·controller가 같은 progress domain과 배열 순서를 공유하게 하고, hairpin의 임의 Cartesian connector가 다른 branch로 넘어가는 위험을 줄이기 위해서다.
2. `S01-2` 입력 준비/staleness, existing commitment, fresh planning 필요성, candidate/fallback 선택, safe-stop latch, raw slowdown과 global handoff/non-empty publication을 조정한다.
3. `S01-3` P3가 현재 유일한 회피 candidate generator다. `P0_BACKUP_ONLY`는 별도 옛 grid가 아니라 P3 출력이 없을 때 사용하는 기존 safety/fallback orchestration과 호환 진단 이름이다.
4. `S01-4` hard-valid는 모든 구현된 필수 제약을 통과했다는 뜻이다. selected는 그 집합에서 lexicographic ranking으로 최종 선택된 하나다.
5. `S01-5` 현재 committed suffix가 valid하면 geometry를 그대로 재사용해 jitter를 막고, evaluator 호출 전에 return해 corridor/root/candidate 계산도 생략한다.

## 02. Perception to planning 정답

1. `S02-1` `/static_obs`는 cluster, tracking, existence confirmation, envelope stability를 거친 object array다. planner가 confirmed-static subset과 대비해 raw slowdown이라고 부를 뿐 LaserScan 원본은 아니다.
2. `S02-2` 횡경로는 여러 callback 동안 차량이 추종할 강한 약속이므로 motion Static까지 확인한 안정된 evidence를 요구한다. 승격 전 object는 빠른 감속 증거로만 써 false positive의 lateral 증폭을 막는다.
3. `S02-3` `s_start/s_end`는 진행 방향 점유 구간, `d_right/d_left`는 횡방향 오른쪽/왼쪽 경계다.
4. `S02-4` 정적 collision은 spatial path와 고정 envelope의 교집합 문제다. 동적 collision은 ego와 obstacle의 time-parameterized states가 같은 시간에 겹치는지를 봐야 한다.
5. `S02-5` raw overlay는 기존 waypoint speed에 min cap만 씌우고 `d(s)`와 side를 바꾸지 않으므로 false positive가 좌우 경로 전환으로 나타나지 않는다.

## 03. Frenet coordinates 정답

1. `S03-1` tangent `t=[cosψ,sinψ]`를 반시계로 90도 회전하면 left normal `[-sinψ,cosψ]`가 된다.
2. `S03-2` `||dr/ds||=1`이라 tangent가 unit vector가 되고, parameter speed를 따로 나누지 않아도 `κ=dψ/ds`로 해석할 수 있다.
3. `S03-3` `(1-κ_rd)t`는 reference를 따라가는 성분이 reference curvature와 offset 때문에 변한 것이고, `d'n`은 횡오프셋 변화가 더한 normal 성분이다.
4. `S03-4` 가까운 반대 leg가 Euclidean nearest가 될 수 있어 `s`가 진행 중 branch에서 다른 branch로 점프할 수 있다. 이전 `s`와 monotonic window가 필요하다.
5. `S03-5` global next index부터 sample을 복사하고, validator가 ego-forward `s` 증가가 양수인지 검사해 ordered race-line 진행을 강제한다.

## 04. P3 and quintic 정답

1. `S04-1` station은 entry start, target apex, cluster 내부 middle, cluster end, merge다. offset은 ego에서 시작해 target, analytic middle, target을 거쳐 race line `0`으로 돌아가는 상태다.
2. `S04-2` quintic에는 계수 여섯 개가 있어 양 끝의 위치 `d`, 1차 미분 `d'`, 2차 미분 `d''` 여섯 조건을 독립적으로 맞출 수 있다.
3. `S04-3` current P3는 네 segment와 다섯 knot를 쓰며 내부 derivative/acceleration을 harmonic/secant rule로 공유하고 `d_m`을 analytic root로 구한다. single smoothstep은 한 segment의 양 끝 derivative/acceleration을 0으로 둔 특수형이다.
4. `S04-4` 한 knot의 동일한 `d,d',d''` 값을 왼쪽 segment 끝과 오른쪽 segment 시작에 동시에 경계조건으로 주므로 0·1·2차 미분이 이어진다.
5. `S04-5` M0/M1은 둘 다 같은 quintic reconstruction을 쓴다. 차이는 active-set/branch/template 후보를 제안하는 policy와 순서다.
6. `S04-6` root는 sampled corridor 조건을 만족하는 parameter proposal일 뿐 Cartesian footprint, collision, ordering, curvature, speed-related geometry를 모두 증명하지 않기 때문이다.

## 05. Curvature 정답

1. `S05-1` `ψ=atan2(y_q,x_q)`를 미분해 `(x_qy_qq-y_qx_qq)/(x_q²+y_q²)`를 얻고, 이를 `ds/dq=sqrt(x_q²+y_q²)`로 나누면 분모가 `3/2`승인 Cartesian 식이 된다.
2. `S05-2` `atan2`는 두 derivative의 부호로 사분면을 유지하고 `x'=0`인 수직 tangent도 처리한다. 단순 비율의 `atan`은 이 정보를 잃는다.
3. `S05-3` current 방식은 normal-shift된 최종 Cartesian waypoint 주변에 local cubic을 fit해 reference와 sampling까지 포함한 derivative를 추정한다. P3의 analytic `d(s)`만 symbolic 미분하는 것이 아니다.
4. `S05-4` 원호 왼쪽 constant offset은 반지름을 `R-d`로 바꾼다. 따라서 `d'=d''=0`이어도 curvature는 `1/(R-d)`로 바뀐다.
5. `S05-5` `dκ/ds`는 경로의 공간 속성이다. actuator steering rate는 속도와 `δ(κ,v)` 모델을 통해 `dδ/dt`로 변환해야 한다.

## 06. Vehicle feasibility 정답

1. `S06-1` rigid wheel, no lateral slip, low-speed kinematic bicycle, instantaneous steering이라는 가정에서 회전 중심 geometry로 `R=L/tanδ`가 나온다.
2. `S06-2` 같은 `κ`에서도 횡가속이 `v²`로 증가하고 understeer steering demand도 속도제곱 항을 가지므로 tire와 actuator 한계에 더 빨리 도달한다.
3. `S06-3` kinematic exact는 `δ=atan(Lκ)`인 no-slip geometry다. current controller model은 정상상태 understeer를 `δ=κ(L+K_usv²)`로 보상하는 선형 inverse model이다.
4. `S06-4` 방향별 curvature hard gate, lateral-acceleration speed cap, 접근 제동 ramp, longitudinal forward/backward pass, ego braking deficit ranking이다.
5. `S06-5` spatial rate에는 차량 속도, speed change, understeer, steering mapping과 actuator lag/rate가 없다. 이들을 거쳐야 temporal angle rate가 된다.
6. `S06-6` full tire/friction-circle rollout, closed-loop tracking reachability, dynamic obstacle time collision, sideslip dynamic bicycle certificate, joint probabilistic error bound가 production hard validator에서 확인되지 않는다.

## 07. Candidate validation 정답

1. `S07-1` generator는 제한된 모델로 candidate를 빠르게 제안하고 validator는 reconstructed final path를 독립적으로 검사한다. 이 분리는 analytic construction의 누락이나 sampling 차이가 바로 publish authority가 되는 것을 막는다.
2. `S07-2` gap만 보면 정상 tracking error도 기각하고 slope만 보면 아주 작은 entry distance로 나눠 발산한다. budget 초과와 baseline slope 초과가 함께 있을 때만 진짜 jump로 본다.
3. `S07-3` center bound는 waypoint 중심 `d`가 track range 안인지 본다. footprint bound는 heading을 가진 vehicle rectangle의 네 corner를 local reference에 projection해 wall 침범을 본다.
4. `S07-4` 현재 maneuver가 책임지는 obstacle까지만 spatial collision authority를 두고 다음 obstacle은 chained planning으로 넘기기 위해서다. wall과 path geometry는 여전히 전체 path에서 검사한다.
5. `S07-5` curvature/gap에 맞춰 waypoint speed를 낮추고 accel/decel propagation을 한다. 남은 ego braking deficit은 hard reject 대신 ranking에서 더 실현 가능한 후보를 우선한다.
6. `S07-6` hard-valid는 static Frenet envelope, sampled geometry와 현재 scalar/table model 범위의 판정이다. tire, delay, dynamic obstacle, closed-loop tracking의 완전한 시간 진화를 증명하지 않는다.

## 08. Candidate ranking 정답

1. `S08-1` hard validation은 최소조건 위반 candidate를 제거한다. ranking은 모두 허용된 candidate 사이의 정책적 선호를 결정한다.
2. `S08-2` next-obstacle exit 미침범, braking feasible, 작은 deficit, 작은 velocity loss, 큰 minimum normalized slack, 작은 평균 `|d|`, 작은 generation index 순이다.
3. `S08-3` weighted sum은 항 사이 보상을 허용하지만 lexicographic은 상위 항 차이가 있으면 하위 항의 크기와 무관하게 거기서 승자를 정한다.
4. `S08-4` curvature/rate는 hard gate를 통과해야 하고 normalized margin이 minimum safety slack의 일부가 된다. 독립 `∫κ²ds` cost는 없다.
5. `S08-5` current ego state에서 profile을 제때 따라잡을 수 없는 후보보다 seam braking이 가능한 후보를 먼저 택하기 위한 우선순위다.
6. `S08-6` 완전 동률이나 대칭 geometry에서도 같은 생성 후보를 항상 택해 output/digest가 작은 비결정성으로 바뀌지 않게 한다.

## 09. Planning lifecycle 정답

1. `S09-1` 단순 preference는 새 후보와 비교해 bonus를 줄 수 있지만, continuation-first는 current suffix를 exact revalidate하고 valid하면 evaluator 자체를 호출하지 않는다.
2. `S09-2` accumulated guard는 progressive reveal에 대한 보수 geometry를, raw는 detector가 실제로 낸 same-callback geometry를 검사한다. 둘 다 통과해야 stale/guard artifact가 ownership을 얻지 않는다.
3. `S09-3` committed 재검증의 tracking-error reserve portion만 fraction으로 줄어든다. vehicle half-width와 safety margin으로 이루어진 physical base clearance는 줄지 않는다.
4. `S09-4` obstacle region을 이미 통과한 정상 maneuver가 tail point 부족 때문에 invalid로 오판되고 safe-stop/replan으로 넘어가는 것을 막기 위해서다.
5. `S09-5` FSM의 GLOBAL 복귀 판정이 non-empty avoid path의 tail/offset을 필요로 하기 때문이다. ego를 덮는 `d=0` loop로 복귀를 확인한 뒤 empty로 간다.
6. `S09-6` A, B, C, D 중 하나가 성립하면 release되는 OR다. 각 branch 내부의 stopped, fresh sequence, consecutive count, FSM selectability 같은 조건은 AND다.

## 10. Failures and limitations 정답

1. `S10-1` 처음에는 ego-entry 검사가 없어 실제 jump를 놓쳤고, slope-only 수정은 작은 denominator 때문에 정상 path를 과잉 기각했다. budget 초과와 baseline slope 초과의 AND로 두 실패를 분리했다.
2. `S10-2` 각 점 cap은 인접 점 사이의 속도 변화율을 제한하지 않아 curve dip에서 속도가 튈 수 있다. energy relation의 forward/backward propagation이 필요하다.
3. `S10-3` 선택 때 통과한 scope를 다음 callback에서 더 넓게 검사하면 같은 path가 즉시 폐기되어 path churn과 safe-stop을 만든다. 동일 maneuver responsibility를 공유해야 한다.
4. `S10-4` collision 전 제동거리가 충분해도 멈춘 위치에서 target offset을 만들 진입거리와 curvature room이 부족할 수 있다.
5. `S10-5` quintic/C², curvature와 curvature-rate 검사, lateral acceleration speed cap, accel/decel pass, lexicographic rank, continuation/retention, safe-stop lifecycle, raw/confirmed authority 분리는 이미 구현되어 있다.
6. `S10-6` 24개는 정해진 knot/template/root family의 finite sample이다. 그 밖의 continuous `d(s)` 또는 다른 knot structure가 feasible할 가능성을 배제하지 않는다.

## 11. Research questions 정답

1. `S11-1` current source에 이미 있는 mechanism은 baseline이다. 반복 관측되는 미해결 failure 또는 formal/empirical guarantee 부재를 측정 가능한 질문으로 만들고 기존 문헌 대비 차이를 입증해야 gap이다.
2. `S11-2` cap-24 실패는 finite proposal set 전멸이고 continuous infeasibility는 허용 함수 공간 전체에 해가 없다는 훨씬 강한 주장이다. oracle이나 certificate가 없으면 둘을 같게 볼 수 없다.
3. `S11-3` dynamics-aware rollout은 기존 연구가 매우 많다. model을 하나 붙이는 것보다 real-time error bound, current controller coupling, 실측 causal gain 같은 새 요소가 필요하다.
4. `S11-4` frame error는 시간 상관과 repeated exposure가 있고 lifecycle branch가 authority를 유지한다. per-frame coverage를 단순히 더하거나 그대로 maneuver collision probability라 부를 수 없다.
5. `S11-5` safety는 collision/invalid path를 publish하지 않는 성질이고, liveness는 bounded conditions에서 progress, valid handoff 또는 명시적 terminal stop으로 결국 이동해 silent deadlock에 빠지지 않는 성질이다.
6. `S11-6` commit, runtime parameter, raw inputs, candidate trace/order, selected identity/digest, controller/vehicle values, failure metric, compute platform과 runtime distribution을 고정해야 한다.

## 용어 종합 정답

1. `SG-1` path는 공간 기하이고 trajectory는 각 상태를 언제 지나는지까지 시간으로 지정한다. speed field가 있는 waypoint path도 full dynamic state trajectory와는 구분할 수 있다.
2. `SG-2` heading은 rad, curvature `dψ/ds`는 `1/m`, spatial curvature rate는 `1/m²`다. 시간 변화는 속도를 곱해 변환하며 steering model은 추가로 필요하다.
3. `SG-3` generator가 candidate를 만들고 hard constraint로 feasibility를 가른 뒤, hard-valid 집합만 ranking하여 하나를 선택한다.
4. `SG-4` commitment는 특정 maneuver geometry/identity의 지속 권한, Guard는 selection 때 frozen obstacle envelope, retention band는 committed path의 reserve jitter를 흡수하는 재검증 완화다.
5. `SG-5` receding horizon은 유한 앞 구간을 반복 다루는 넓은 개념이다. 현재 planner는 control sequence를 매 tick cost optimization하는 MPC가 아니며 valid committed suffix가 있으면 새 최적화도 생략한다.

