# obstacle_detector 검출 테스트 명령

이 문서는 detector-only 패키지의 빌드, 기동, synthetic 검출 테스트 순서를 정리한다.

## 1. 빌드

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-up-to obstacle_detector
source install/setup.zsh
```

## 2. 다른 ROS 시스템과 격리

동일 네트워크의 다른 차량, rosbag, map server와 토픽이 섞이지 않도록 두 터미널에 같은 값을 적용한다.

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=87
```

## 3. Detector 실행

터미널 A:

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=87

ros2 launch obstacle_detector obstacle_detector_node.launch.py
```

synthetic harness가 `/global_waypoints`, `/map`, ego odom, TF, `/scan`을 직접 발행하므로 별도 시뮬레이터는
필요하지 않다.

## 4. Synthetic harness 실행

터미널 B:

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=87

python3 src/obstacle_detector/test/synthetic_opponent_test.py
```

검증 항목:

1. 움직이는 상대차가 `/opp_obs`에 `is_static=false`로 나타난다.
2. 정지 장애물이 `/static_obs`에 나타난다.
3. 정지 장애물이 `/opp_obs`로 새지 않는다.
4. 상대차가 먼저 `/static_obs`에 provisional로 나타난 뒤 같은 ID로 `/opp_obs`에 이동하고,
   dynamic 확정 후에는 `/static_obs`로 돌아오지 않는다.
5. 정지 장애물은 scan face가 변해도 하나의 ID를 유지하고 `/confirmed_static_obs`에서도
   `/static_obs`와 같은 ID를 사용한다.
6. Production과 같이 동일한 `/global_waypoints`를 2초마다 재발행해도 CLCS와 물리 객체 ID가
   초기화되지 않는다.
7. 각각 5포인트 미만인 3+2 beam 파편이 tracking 전에 하나의 detection으로 복원된다.
8. 두 track으로 관측된 정적 객체가 layer merge를 거쳐 `/static_obs`의 한 객체로 병합된다.
9. Hard gate 안의 1프레임 0.45 m outlier가 Mahalanobis gate에서 거부된 뒤 공간적으로 같은
   cluster에 재연결되며, 확정 track의 공개 envelope는 뛰지 않는다.
10. 원거리 5포인트 detection의 위치 공분산이 근거리 조밀 detection보다 크게 유지된다.
11. 모든 visible 객체가 유효한 Cartesian AABB, 중심, 반지름을 발행한다.
12. Layer merge 객체의 Cartesian AABB가 구성 track AABB의 합집합을 포함한다.
13. Invisible track이 과거 Cartesian AABB를 `has_cartesian=true`로 잘못 발행하지 않는다.
14. 모든 객체의 `s_start/s_end/d_right/d_left`가 유한하고 `d_right <= d_left`이며, visible
    객체의 경계는 같은 Cartesian AABB 전체를 투영한 결과다. 폐루프 wrap의
    `s_start > s_end`는 허용한다.
15. `/static_obs/markers`와 `/opp_obs/markers`에 최종 Frenet 경계 기반 `LINE_STRIP`이 나타난다.

테스트가 끝난 뒤 터미널 A의 detector를 `Ctrl+C`로 종료한다.

## 5. 실제 시뮬레이터에서 실행

시뮬레이터가 `/scan`, `/ego_racecar/odom`, TF, `/map`, `/global_waypoints`를 발행하는지 먼저 확인한다.

```bash
ros2 topic info /scan -v
ros2 topic info /ego_racecar/odom -v
ros2 topic info /map -v
ros2 topic info /global_waypoints -v
```

Detector:

```bash
ros2 launch obstacle_detector obstacle_detector.launch.py simulator:=true rviz:=true
```

시뮬레이터가 `/clock`을 발행하는 경우에만 다음을 추가한다.

```bash
use_sim_time:=true
```

## 6. 출력 확인

```bash
ros2 topic echo /static_obs --no-arr
ros2 topic echo /opp_obs --no-arr
ros2 topic hz /static_obs
ros2 topic hz /opp_obs
ros2 topic echo /static_obs/markers --no-arr
ros2 topic echo /opp_obs/markers --no-arr
```

두 layer 토픽의 주기는 `/scan` 주기와 비슷해야 하며 장애물이 없을 때도 빈 배열이 계속 발행되어야 한다.

## 7. 문제 진단

| 증상 | 확인 항목 |
|---|---|
| 아무 출력도 없음 | `/global_waypoints`, scan→map TF, `/scan` publisher 확인 |
| 벽이 장애물로 나옴 | `/map`, `use_map_filter`, `wall_assoc_distance_m`(넓히면 벽 근처 더 제거) 확인 |
| 장애물이 전부 사라짐 | live map에 장애물이 baked-in 되었는지 확인하고 `detector_map_yaml`에 clean map 지정 |
| 상대차가 계속 static/unknown으로 남음 | `motion_classification.dynamic_chi2_threshold`, `dynamic_vote_window/required`, debug 로그의 `Tv`, `votes(S/D)`, `invalid_Pv` 확인 |
| 원거리 중심 변화에 track이 끌림 | adaptive covariance 파라미터와 `assoc_use_mahalanobis` 확인 |
| 정상 detection이 자주 새 track이 됨 | timestamp, `assoc_mahalanobis_gate`, `meas_var_s/d`, process noise 확인 |
| 작은 파편이 통째로 사라짐 | `cluster_merge_enable`, `cluster_merge_distance`, `cluster_merge_min_fragment_points` 확인 |
| 하나의 detection이 여러 track으로 갈라짐 | 먼저 `cluster_merge_*`, 이후 `assoc_gate` 확인 |
| 여러 track이 여러 출력으로 나옴 | `layer_merge_enable`, `layer_merge_gap_s/d` 확인 |
| 후단에서 Cartesian 장애물을 거부함 | `has_cartesian`, AABB min/max, 중심, 양수 `radius` 확인 |
| 후단에서 Frenet 장애물을 거부함 | Frenet 경계 유한성, `d_right <= d_left`, global waypoint 준비 여부 확인 |
| miss 이후 Cartesian 장애물이 과거 위치에 남음 | predicted-only 출력이 `has_cartesian=false`인지 확인 |
| RViz 장애물과 local planner 판단 영역이 다름 | `/static_obs/markers`를 표시 중인지 확인하고 기존 `/perception/obstacles/markers` display 제거 |
| RViz에서 scan과 map이 어긋남 | scan header frame, map→scan TF, ROS domain의 중복 TF publisher 확인 |

## Perception 진단 로그 확인

기본 설정은 1초마다 `DIAG perception` 로그를 출력한다.

```bash
ros2 launch obstacle_detector obstacle_detector_node.launch.py 2>&1 | \
  grep --line-buffered "DIAG perception"
```

`scans=processed/received`와 `drop(clcs/tf)`로 scan 전체 처리 실패를 먼저 확인하고, 이어서 beam,
cluster, Layer 1 reject, association, classification 순서로 원인을 좁힌다. Association의
`euclid_reject`와 `maha_reject`는 객체 수가 아니라 track-detection 후보 쌍 수다.

Track별 map 속도, `Tv`, vote, 위치 RMS, confidence와 마지막 measurement 경과시간은 다음처럼
motion debug를 켜서 확인한다.

```bash
ros2 run obstacle_detector obstacle_detector_node --ros-args \
  --params-file "$(ros2 pkg prefix obstacle_detector)/share/obstacle_detector/config/obstacle_detector.yaml" \
  -p motion_classification.debug_enable:=true
```

Detector 내부에는 scan noise filter와 deskew가 없으므로 해당 통계는 출력하지 않는다. 향후 전처리
노드를 연결하면 noise/deskew 통계는 그 노드의 로그 또는 diagnostics에서 별도로 확인한다.
