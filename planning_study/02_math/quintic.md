# Quintic polynomial과 경계조건

## 1. 일반 5차 다항식

구간 `s in [s0,s1]`, `h=s1-s0`, 정규화 변수 `u=(s-s0)/h`를 사용하면

```text
d(u) = c0 + c1 u + c2 u^2 + c3 u^3 + c4 u^4 + c5 u^5
```

이다. 6개 계수는 구간 양 끝의 위치·1차 미분·2차 미분 6조건으로 유일하게 결정된다.

```text
d(s0)=d0, d'(s0)=v0, d''(s0)=a0
d(s1)=d1, d'(s1)=v1, d''(s1)=a1
```

## 2. 현재 코드의 계수

코드는 다음 중간값을 사용한다.

```text
c0 = d0
c1 = h v0
c2 = 0.5 h^2 a0
r0 = d1 - c0 - c1 - c2
r1 = h v1 - c1 - 2c2
r2 = h^2 a1 - 2c2
c3 = 10r0 - 4r1 + 0.5r2
c4 = -15r0 + 7r1 - r2
c5 = 6r0 - 3r1 + 0.5r2
```

구현은 [`p3_shadow.cpp:613`](../../src/local_planning/src/p3_shadow.cpp#L613)에 있다. `h<=1e-9`이면 예외로 후보를 중단한다.

## 3. 미분 단위

계수는 정규화 변수 `u` 기준이지만 입력 `v0,v1`은 `dd/ds`, `a0,a1`은 `d^2d/ds^2`다. 그래서 `c1`에 `h`, `c2`에 `h^2`가 들어간다.

```text
dd/ds = (1/h) dd/du
d2d/ds2 = (1/h^2) d2d/du2
```

이 scaling을 빼면 transition 길이를 바꿀 때 실제 slope/curvature가 잘못 변한다.

## 4. smoothstep은 특수한 경우

`d0=0`, `d1=1`, 양 끝의 1·2차 미분이 모두 0이면

```text
d(u)=10u^3-15u^4+6u^5
```

가 된다. 즉 5차 smoothstep은 quintic Hermite의 특수한 경계조건이다. 현재 P3는 내부 knot derivative/acceleration을 0으로 고정하지 않으므로 일반 Hermite가 필요하다.

## 5. 왜 cubic이 아니라 quintic인가

Cubic Hermite는 양 끝의 위치와 1차 미분 네 조건을 만족할 수 있다. 양 끝의 2차 미분까지 지정해 knot에서 C2를 맞추려면 6자유도의 quintic이 자연스럽다.

현재 설계에서 C2가 중요한 이유:

- `d'` jump는 heading jump로 연결될 수 있다.
- `d''` jump는 curvature jump로 연결될 수 있다.
- downstream steering FF가 `kappa`를 직접 사용한다.

단, C2 `d(s)`가 actuator steering rate를 자동 보장하는 것은 아니다. 최종 Cartesian curvature rate는 별도로 hard check하고, 시간 steering rate는 publish diagnostics와 controller limiter가 담당한다.

## 6. 내부 knot의 harmonic derivative

단순 중앙차분 대신 두 secant의 부호와 크기를 고려하는 `sourceHarmonicDerivative()`를 쓴다: [`p3_shadow.cpp:638`](../../src/local_planning/src/p3_shadow.cpp#L638), [`p3_analytic_solver.hpp`](../../src/local_planning/include/local_planning/p3_analytic_solver.hpp).

직관적으로 secant 부호가 바뀌는 극값 부근에서 derivative를 과도하게 두지 않고, 같은 부호일 때 weighted harmonic 형태로 monotonic tendency를 유지하려는 규칙이다. P3 M1 analytic solver는 이 branch regime을 구분하여 root를 찾는다: [`p3_shadow.cpp:679`](../../src/local_planning/src/p3_shadow.cpp#L679).

## 7. 수치적으로 주의할 점

- 아주 작은 `h`: `d'`, `d''`, curvature가 폭증할 수 있어 코드가 non-positive segment를 기각한다.
- 큰 offset/짧은 transition: `maximum_lateral_slope`, curvature, curvature-rate에서 탈락한다.
- knot는 C2라도 global waypoint sampling이 성기면 discrete curvature estimation이 흔들릴 수 있다.
- analytic local cubic geometry fallback 여부에 따라 같은 `d(s)`의 최종 `kappa`가 조금 달라질 수 있다.

## 8. 손으로 검산할 질문

1. 주어진 `h,d0,v0,a0,d1,v1,a1`에서 계수 6개를 계산할 수 있는가?
2. `u=0,1`에 대입해 6개 경계조건을 확인할 수 있는가?
3. transition 길이 `h`를 2배로 하면 같은 offset 변화의 slope와 curvature 경향이 어떻게 바뀌는가?
4. 다섯 knot를 네 segment로 연결할 때 왜 공통 `d,d',d''`가 C2를 만드는가?
