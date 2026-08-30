# E02/E04 waypoint-0 semantics

## 판정

`waypoint 0`은 물리적 ego pose가 아니다. 현재 production P3는 `ego.s`보다 앞선 첫 ordered
global-reference sample을 `nextReferenceIndex(ego.s)`로 고르고, 그 sample의 `s`에서 P3 profile을
평가해 Cartesian 점을 만든다 (`p3_shadow.cpp:1369-1405`).

다음과 같이 쓸 수 있다.

\[
i_0=\operatorname{nextReferenceIndex}(s_e),\quad
\Delta s_0=\operatorname{forwardDistance}(s_e,s^{ref}_{i_0}),
\]

\[
d_0=P_3(\Delta s_0;d_e,d_{target},d_{mid},z_0,\ldots,z_4),
\]

\[
(x_0,y_0)=(x^{ref}_{i_0},y^{ref}_{i_0})+
d_0(-\sin\psi^{ref}_{i_0},\cos\psi^{ref}_{i_0}).
\]

- `s_0`는 ego `s`와 ordered reference가 정하지만 ego 위치와 같지 않다.
- `d_0/x_0/y_0`는 일반적으로 ego state만으로 고정되지 않는다. `Delta s_0 <= z0`이면
  `evaluateProfile()`이 `ego.d`를 반환하지만, E02/E04는 `z0` 뒤 첫 P3 segment 안에 있다.
- ego state 구조에는 yaw가 없다. `finalizeP3ShadowPath()` 뒤 `updateGeometry()`가 첫 다섯
  candidate `x/y` sample의 local cubic derivative로 `psi_0=atan2(y',x')`를 다시 계산한다
  (`raceline_spline_planner.cpp:2724-2802`). cubic fit이 퇴화하면 다음 candidate chord를 쓰며
  이 경우도 ego yaw가 아니다 (`2808-2815`).
- 따라서 waypoint-0 position과 orientation이 모두 실제 fixed ego pose에 대응하지 않는다.
  waypoint-0 footprint 실패만으로 물리적 ego footprint가 경계 밖이라고 판정할 수 없다.

## 왜 d_target과 d_mid가 waypoint 0에 영향을 줄 수 있는가

P3 offset knot는 `[d_ego,d_target,d_mid,d_target,0]`이다. 첫 segment의 끝점은 `d_target`이고,
그 끝의 derivative와 acceleration은 앞·뒤 secant를 함께 사용한다
(`p3_shadow.cpp:928-950`). 뒤 secant에는 `d_mid`가 들어간다. 그러므로 첫 segment 내부의
`d_0`와 그 주변 `x/y` tangent는 `d_target`과 `d_mid` 둘 다에 의존할 수 있다. 이 의존성은
position뿐 아니라 rotated rectangle의 네 corner 위치에도 전달된다.

footprint validator는 waypoint의 candidate-derived `(x,y,psi)`와 현재 parameter의
`vehicle_length_m=0.56`, `vehicle_half_width_m=0.15`, track-boundary reserve를 사용해 네 corner를
local reference segment에 투영한다. 최소 corner clearance가 음수이면
`footprint_track_bound`이다 (`raceline_spline_planner.cpp:2249-2395`).

## E02/E04 실제 수치

아래 값은 v1 exact-parity oracle pilot의 nearest-invalid path를 waypoint-0 의미만 다시 해석한
것이다. 새 oracle search 결과가 아니다.

| event | ego `(s,d)` m | `z0` m | first sample `(s,d)` m | `Delta s0` m | first `(x,y,psi)` | 해석 |
|---|---|---:|---|---:|---|---|
| E02 | `(9.2907000568, +1.0357765927)` | `0` | `(9.5293866329, +1.0346226667)` | `0.2386865761` | `(-4.0646224698, 1.7370460441, -0.1073408686)` | ego pose와 다른 future reference sample이며 첫 P3 segment 내부 |
| E04 | `(21.4454662605, -1.1578872475)` | `-2.22e-16` | `(21.5681761092, -1.1561586186)` | `0.1227098486` | `(8.3428215182, -1.0674188869, +0.5075562774)` | ego pose와 다른 future reference sample이며 첫 P3 segment 내부 |

E02의 nearest-invalid `(d_target,d_mid)=(+0.36,+0.11) m`, E04의 값은
`(-0.4388022563,-0.45) m`였다. 두 path 모두 waypoint 0에서 footprint failure가 났지만, 그
검사는 실제 ego pose가 아니라 위 candidate-specific future sample에 대한 검사다.

## 준비 단계 해석 수정

v1 문서의 “frozen footprint가 경로 시작부터 invalid”는 “candidate의 첫 sampled waypoint가
invalid”로 좁혀 읽어야 한다. 실제 ego footprint invalid라는 주장은 position과 orientation이
실제 fixed ego pose인 별도 검사 없이는 성립하지 않는다.
