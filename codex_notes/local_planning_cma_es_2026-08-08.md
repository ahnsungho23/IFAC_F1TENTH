# local_planning CMA-ES 재시작 기록 (2026-08-08)

이 파일은 다음 CMA-ES 실행 때 출발 조건과 이미 확인한 실패를 반복하지 않기 위한 Codex 작업
기록이다. 운영 파라미터의 공식 설명은 `src/local_planning/docs/local_planner.md`를 우선한다.

## 재현 기준

- 장애물 파일: `/home/sungho/f1sim_C/f1tenth_gym_ros/maps/ifac_track_obs.obstacles.yaml`
- 장애물 파일 SHA-256: `061e454573889ed3390114b4f4522dd05624af11c8197f6716ad9a743a027386`
- 장애물 중심/크기:
  - `(-6.8523635274, -0.2089476270)`, `0.5 x 0.5 m`
  - `(-17.4495586607, 5.3914981796)`, `0.5 x 0.5 m`
  - `(-5.0041235349, 2.5866314572)`, `0.5 x 0.5 m`
- 물리 map: `ifac_track_obs`, waypoint/reference map: obstacle-free `ifac_track`
- waypoint 생성기 `max_width_distance_m=3.0`
- global waypoint `d_left/d_right`는 차량 반폭과 벽 안전마진이 반영된 차량 중심 한계이다.
  local_planning의 추가 벽 마진은 `0.0 m`로 고정한다.
- 차량 폭 `0.287 m`, 반폭 `0.1435 m`, 길이 `0.56 m`
- `tracking_error_reserve_m=0.14`
- 조향 한계 `+-0.41 rad`, wheelbase `0.3302 m`, planner 곡률 한계
  `tan(0.41)/0.3302 = 1.316266519079011 rad/m`
- `outside_line_transition_scale=0.1`
- 속도 조건: controller `min_speed=max_speed=1.5 m/s`
- ROS graph는 매 실행 별도 `ROS_DOMAIN_ID`를 쓰고 `/ego_racecar/odom` bridge와
  `/avoid_waypoints` planner publisher가 각각 정확히 하나인지 확인한다.

## CMA-ES 구성

- seed: `260808`
- 7개 변수, anchor 3개 + 2세대 x population 8 = 서로 다른 후보 19개
- 최종 최고 후보 반복 검증 2회 추가
- 범위:
  - `safety_margin_m`: `[0.0, 0.12]`
  - `obstacle_longitudinal_padding_m`: `[0.0, 0.60]`
  - `pre_apex_far_m`: `[3.0, 9.0]`, 배열은 `[far, 2far/3, far/3]`
  - `post_apex_far_m`: `[2.0, 7.0]`, 배열은 `[far/3, 2far/3, far]`
  - transition short: `[0.05, 0.80]`
  - transition middle: `short+0.10 .. 1.80`
  - transition long: `middle+0.20 .. 3.50`
- 완주 우선, 그다음 lap time으로 평가한다. collision, safe-stop timeout, no-start를 성공으로
  취급하지 않는다.

## 폐기한 평가 조건

초기 탐색에서 controller `min_speed=0`을 사용했더니 곡률 조향 권한 상한이 0인 지점에서 정상
회피 경로의 waypoint 속도가 양수인데도 차량이 약 `36.59 m` 진행 후 정지했다. 목적함수가 이
timeout을 충돌보다 좋게 평가해 잘못된 최고 후보가 됐다. 이 결과는 폐기했다. 이후 모든 유효
결과는 `min_speed=max_speed=1.5 m/s` 조건이다.

## 기준 후보와 실패 원인

사용자가 제안한 기준:

```yaml
safety_margin_m: 0.03
obstacle_longitudinal_padding_m: 0.30
pre_apex_distances_m: [6.0, 4.0, 2.0]
post_apex_distances_m: [1.0, 2.0, 3.0]
transition_distance_scales: [0.1, 0.6, 2.0]
```

- 첫 obstacle 0 왼쪽 경로는 commit했다.
- 다음 blocking obstacle에서 왼쪽은 `maximum_curvature_radpm` 초과, 오른쪽은 waypoint track
  bound 초과로 양쪽이 모두 거부되어 safe-stop이 latch됐다.
- 최종 결과는 진행 `14.933166 m`, 충돌 위치 `(-0.400055, 4.014728)`였다.

19개 후보 집계는 완주 2, 충돌 13, safe-stop timeout 4였다. 반복 로그 event 합계는 track-bound
거부 62, 곡률 거부 42, safe-stop latch 40, avoidance commit 34였다. event 수는 후보 수가 아니다.

## 최종 최고 후보와 반복 결과

```yaml
obstacle_longitudinal_padding_m: 0.3661363460730684
vehicle_half_width_m: 0.1435
safety_margin_m: 0.058229308745201366
tracking_error_reserve_m: 0.140
pre_apex_distances_m: [9.0, 6.0, 3.0]
post_apex_distances_m: [1.6471992864388174, 3.294398572877635, 4.9415978593164525]
transition_distance_scales: [0.2126428647700918, 1.074713915992601, 3.5]
outside_line_transition_scale: 0.1
maximum_curvature_radpm: 1.316266519079011
```

- CMA 관측: `30.640038 s`, 충돌 없음, 최소 referee clearance `0.194844 m`
- repeat 0: `30.540069 s`, 충돌 없음, 최소 referee clearance `0.185400 m`
- repeat 1: `30.619978 s`, 충돌 없음, 최소 referee clearance `0.195067 m`
- 세 실행 모두 1랩 완주, 최고 속도 `1.5 m/s`, 평균 속도 약 `1.49 m/s`
- 이 값은 `src/local_planning/config/local_planning.yaml`에 적용됐다.

두 번째 완주 후보는 safety `0.06`, padding `0.45`, pre `[9,6,3]`, post `[2,4,6]`, scales
`[0.4,1.2,3.2]`이며 `30.84 s`였다.

## 다음 탐색 시작점

1. 장애물 YAML 해시와 좌표가 위 기준과 같은지 먼저 확인한다. 다르면 이전 objective 비교값을
   그대로 이어 쓰지 말고 현 운영값을 새 anchor로 3회 재검증한다.
2. 현 운영값을 첫 anchor로 사용한다. 기존 `[6,4,2]/[1,2,3]/[0.1,0.6,2.0]`으로 되돌아가지
   않는다.
3. 최고 후보가 `pre_apex_far=9.0`, transition long `3.5`로 두 탐색 상한에 닿았다. 추가 개선 시
   이 두 상한을 먼저 `pre_far=11~12 m`, long `4.0~4.5` 정도로 확장하되 detection lookahead
   `12 m`와의 관계를 확인한다.
4. 안전마진과 tracking reserve를 동시에 최적화하지 않는다. 우선 경로 형상 범위를 확장하고,
   `tracking_error_reserve_m=0.14`는 고정한다.
5. 성공 조건은 simulator collision false, safe-stop/timeout 없음, 1랩 완주이며 최고 후보를 최소
   2회 더 반복한다.
6. `/tmp/combined_margin_geometry_cma_const15/search_result.json`은 당시 전체 결과이며 SHA-256은
   `87f82202e55395ca725678bec5b3b60126bbba2d9f6bfa0472297e8489b9c80f`이다. `/tmp`가 정리되면
   이 문서의 값과 집계를 기준으로 새 evaluator를 만든다.
