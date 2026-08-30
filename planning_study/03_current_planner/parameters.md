# 현재 local planner 파라미터 지도

## 1. 읽는 순서

실제 값의 우선순위는 다음과 같다.

```text
C++ declare default
  < YAML params_file
  < launch의 뒤쪽 parameter dictionary
  < CLI/launch argument override
  < runtime parameter change(허용된 경우)
```

현재 기본 launch는 YAML을 먼저 넣고 `p3_mode`, diagnostics, lockstep, hold 관련 값을 launch argument로 다시 넣는다: [`local_planner_only.launch.py:86`](../../src/local_planning/launch/local_planner_only.launch.py#L86).

따라서 아래 표는 **현재 기본 YAML + 기본 launch argument** 기준이다. 특정 rosbag의 runtime-effective 값은 아니다.

## 2. perception/geometry

| 파라미터 | 현재값 | 단위 | 역할 |
|---|---:|---|---|
| `detection_lookahead_m` | 15.0 | m | visible obstacle horizon |
| `obstacle_cluster_gap_m` | 0.8 | m | 한 maneuver로 묶을 longitudinal gap |
| `obstacle_longitudinal_padding_m` | 0.0 | m | detector AABB 앞/뒤 기하 팽창 |
| `vehicle_length_m` | 0.56 | m | footprint 길이 |
| `vehicle_half_width_m` | 0.15 | m | footprint 반폭 |
| `safety_margin_m` | 0.08 | m | obstacle lateral 추가 margin |
| `wall_safety_margin_m` | 0.04 | m | wall boundary reserve |
| `fallback_track_half_width_m` | 1.50 | m | waypoint width가 invalid일 때 fallback |

근거: [`local_planning.yaml:3`](../../src/local_planning/config/local_planning.yaml#L3), [`local_planning.yaml:477`](../../src/local_planning/config/local_planning.yaml#L477).

### 주의할 주석 불일치

YAML 15~19행 주석에는 `safety_margin=0.00`인 과거 계산이 남아 있지만 scalar는 0.08이다. safe-stop 주석 일부도 과거 longitudinal padding 0.4149925를 말하지만 scalar는 0.0이다. 현재 동작 판단에서는 주석 속 숫자가 아니라 실제 key/value와 source를 우선해야 한다.

## 3. tracking/uncertainty reserve

| 파라미터 | 현재값 | 효과 |
|---|---:|---|
| `obstacle_reserve_mode` | `none` | obstacle tracking LUT gate off |
| `tracking_error_reserve_m` | 0.0 | gate off 상태에서 사용 안 함 |
| `localization_reserve_m` | 0.0 | 별도 reserve 없음 |
| `uncertainty_sigma_scale` | 0.0 | `s_var,d_var` sigma 팽창 off |
| `uncertainty_min_longitudinal_inflation_m` | 0.0 | longitudinal floor off |
| `uncertainty_min_lateral_inflation_m` | 0.0 | lateral floor off |
| `uncertainty_max_lateral_inflation_m` | 0.0 | face history inflation도 0으로 clamp |

근거: [`local_planning.yaml:42`](../../src/local_planning/config/local_planning.yaml#L42), [`local_planning.yaml:699`](../../src/local_planning/config/local_planning.yaml#L699).

메시지에는 covariance가 있고 guard code도 존재하지만 현재 operational risk inflation은 꺼져 있다는 뜻이다.

## 4. candidate shape

| 파라미터 | 현재값 |
|---|---|
| `pre_apex_distances_m` | `[11.4422, 7.6281, 3.8141]` |
| `post_apex_distances_m` | `[2.0595, 4.1190, 6.1785]` |
| `entry_transition_fractions` | `[0.51458, 0.75, 1.0]` |
| `transition_distance_scales` | `[0.49717, 0.69915, 3.69877]` |
| `outside_line_transition_scale` | `0.40600` |
| `maximum_exit_length_m` | `0.0` (비활성) |
| `post_merge_lookahead_m` | `5.0` |
| `post_merge_min_time_sec` | `1.0` |
| `minimum_target_offset_m` | `0.15` |
| `maximum_target_offset_m` | `1.50` |
| `target_d_candidate_count` | `5` |

근거: [`local_planning.yaml:501`](../../src/local_planning/config/local_planning.yaml#L501).

`target_d_candidate_count=5`가 전체 후보 수 5를 뜻하지 않는다. P3는 M0/M1 조합을 포함하여 전체 cap 24를 가진다.

## 5. geometry feasibility

| 파라미터 | 현재값 | 단위 |
|---|---:|---|
| `maximum_lateral_slope` | 0.8 | m/m |
| `entry_discontinuity_min_budget_m` | 0.20 | m |
| `entry_continuity_baseline_m` | 0.50 | m |
| `maximum_curvature_radpm` | 1.3162665 | 1/m |
| `maximum_curvature_rate_radpm2` | 20.0 | 1/m² |
| `control_wheelbase_m` | 0.33 | m |
| `control_max_steering_left_rad` | 0.410 | rad |
| `control_max_steering_right_rad` | 0.361 | rad |
| `control_max_steering_rate_radps` | 20.0 | rad/s |
| `analytic_path_geometry_enable` | true | - |

근거: [`local_planning.yaml:544`](../../src/local_planning/config/local_planning.yaml#L544), [`local_planning.yaml:591`](../../src/local_planning/config/local_planning.yaml#L591).

`control_max_steering_rate_radps`는 candidate hard reject가 아니라 final feasibility diagnostic에 사용된다.

## 6. velocity limits

속도 bin은 `[0,1,...,9]` m/s다.

| 속도 구간 | lateral accel limit | accel limit | decel limit |
|---|---:|---:|---:|
| 0~3 m/s | 7.6 m/s² | 3.7 m/s² | 2.0 m/s² |
| 4 m/s | 7.0 | 3.7 | 2.0 |
| 5 m/s | 7.0 | 3.47 | 2.0 |
| 6 m/s | 7.0 | 3.33 | 2.0 |
| 7~9 m/s | 6.5 | 3.0 | 2.0 |

실제 값은 linear interpolation된다: [`local_planning.yaml:204`](../../src/local_planning/config/local_planning.yaml#L204).

추가 값:

- launch speed floor: 1.0 m/s
- avoidance minimum speed: 1.0 m/s
- confirmed speed envelope: on
- confirmed post hold: 1.0 m
- confirmed response delay: 0.15 s
- approach base/max decel: source/YAML의 별도 값 사용

## 7. raw slowdown

`/static_obs` provisional geometry를 path 후보에는 넣지 않고 speed hint로만 사용한다.

| 파라미터 | 현재값 |
|---|---:|
| enable | true |
| topic | `/static_obs` |
| trigger | 12.0 m |
| speed cap | 2.8 m/s |
| hold | 1.0 s |
| response delay | 0.15 s |
| distance margin | 0.50 m |

근거: [`local_planning.yaml:306`](../../src/local_planning/config/local_planning.yaml#L306).

## 8. lifecycle/safe-stop

| 파라미터 | 현재값 | 의미 |
|---|---:|---|
| `safe_stop_buffer_m` | 2.60 m | nominal stop target 여유 |
| `safe_stop_deceleration_mps2` | 1.8 | stop profile decel |
| `minimum_path_points` | 8 | path sample 하한 |
| `obstacle_stale_timeout_sec` | 0.75 s | perception degraded threshold |
| `odometry_stale_timeout_sec` | 5.0 s | Frenet stale threshold |
| `safe_stop_release_cycles` | 8 | release evidence debounce |
| `safe_stop_blind_release_sec` | 4.0 s | stopped blind timeout |
| `safe_stop_blind_creep_speed_mps` | 0.7 | blind release cap |
| `initial_observation_count` | 3 | first cluster stabilization |
| `initial_observation_min_duration_sec` | 0.15 s | 최소 안정화 시간 |
| `initial_observation_max_wait_sec` | 0.35 s | 최대 대기 |

근거: [`local_planning.yaml:605`](../../src/local_planning/config/local_planning.yaml#L605), [`local_planning.yaml:660`](../../src/local_planning/config/local_planning.yaml#L660), [`local_planning.yaml:694`](../../src/local_planning/config/local_planning.yaml#L694).

## 9. topics/runtime mode

| key | 값 |
|---|---|
| global | `/global_waypoints` |
| confirmed obstacle | `/confirmed_static_obs` |
| raw speed hint | `/static_obs` |
| ego Frenet | `/car_state/frenet/odom` |
| FSM | `/state` |
| output | `/avoid_waypoints` |
| planning period | 25 ms |
| P3 mode | `TEST_ACTIVE` |
| P3 diagnostics | `EVENTS` |

근거: [`local_planning.yaml:814`](../../src/local_planning/config/local_planning.yaml#L814), [`local_planning.yaml:842`](../../src/local_planning/config/local_planning.yaml#L842).

## 10. 파라미터 연구 시 원칙

1. hard constraint, model parameter, empirical margin, lifecycle debounce를 같은 종류의 tuning knob로 취급하지 않는다.
2. planner/controller 공유 값은 동시에 바꾼다.
3. YAML scalar와 launch override를 모두 기록한다.
4. margin을 줄여 candidate가 늘었다는 사실과 실제 collision risk가 줄었다는 사실을 구분한다.
5. latency/margin 값은 실측 분포와 coverage 목표를 함께 기록한다.
6. 후보 수·선택률뿐 아니라 rejection reason, minimum slack, tracking error, actuator saturation을 함께 본다.
