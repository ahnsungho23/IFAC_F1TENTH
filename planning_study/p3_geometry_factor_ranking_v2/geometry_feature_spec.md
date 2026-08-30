# Geometry feature specification

## Online-available 입력만 사용

Ranking 전에 사용하는 원자료는 다음뿐이다.

- ego `s`, `d`, speed
- 현재 obstacle snapshot과 production과 같은 inflation semantics
- 현재 ordered global/reference path의 `s`, track width, curvature
- strict `buildP3ShadowPlanningContext`와 동등한 side bounds
- 같은 input에서 production이 이미 제시한 lateral/transition factor provenance

Oracle label, oracle-valid tuple, exact-validator failure, hard margin은 online feature가 아니다.

## Corridor

각 side의 station sample에서 vehicle half width `0.15 m`, wall margin `0.04 m`, obstacle clearance-expanded interval을 적용한다. 선택 side의 feasible interval을

\[
C_i=[l_i,u_i],\qquad c_i=(l_i+u_i)/2,
\]

로 둔다. Blocking cluster 구간의 interval intersection으로 connected target component를 얻는다. Bottleneck은 `(width,station)` 오름차순이다.

## Production station semantics의 cheap 복원

Outside side는 cluster midpoint에 가장 가까운 reference index부터 2 m 동안 curvature를 합해 `sum(kappa)<0`이면 LEFT다. Candidate `(t,e,x)`에 대해

\[
L_{req}=|t-d_{ego}|/0.8,
\]

\[
z_1=s_{cluster,start},\quad
z_0=z_1-z_1\frac{11.442220427651225}{15}e.
\]

`z1 < L_req`이면 production과 같이 `z0=0`, `z1=max(L_req, reference_spacing)`로 바꾼다. 나머지는

\[
z_2=(z_1+s_{cluster,end})/2,
\quad z_3=s_{cluster,end},
\]

\[
z_4=z_3+6.178529850015357\,x\,m_{outside},
\]

이고 `m_outside=0.4060036444074003` 또는 `1`이다. 현재 `maximum_exit_length_m=0`이므로 exit cap은 비활성이다.

## Candidate proxy

Exact source harmonic derivative와 quintic-Hermite coefficient 식을 cheap arithmetic으로 복원하지만 Cartesian path, speed shaping, footprint는 만들지 않는다. 실제 reference-waypoint station에서 `d(s)`를 sample한다.

- `max_corridor_violation_m`: `max(max(l_i-d_i, d_i-u_i, 0))`
- `sum_corridor_violation_m`: 위 violation 합
- `minimum_clearance_m`: `min(d_i-l_i,u_i-d_i)`
- `center_error`: blocking span에서 `|d_i-c_i|/max(0.05,u_i-l_i)` 평균
- `peak_lateral_slope_proxy`: 인접 reference sample의 `|Δd|/Δs` 최대
- `slope_excess`: `max(0,peak_proxy-0.8)`
- `curvature_proxy`: discrete lateral slope의 station 차분 최대
- `shape_energy`: sampled lateral-slope 제곱 평균
- `exit_conflict_proxy`: cluster 뒤 active later obstacle가 있는 reference sample에서 선택 corridor 위반이 양수인지 여부

`exit_conflict_proxy`는 frozen `exit_reaches_next_obstacle` 판정의 대체물이 아니다. Candidate를 만들기 전의 coarse priority일 뿐이며 최종 USABLE 판정은 frozen diagnostic을 그대로 쓴다.

## Tie/provenance feature

- target/mid source priority
- lateral factor generation index
- transition pair index
- normalized entry/exit 위치
- exact configuration lexical key

모든 feature와 단위는 [factor_space_spec.json](factor_space_spec.json), 계산 authority는 [run_factor_ranking_v2.py](run_factor_ranking_v2.py)다.
