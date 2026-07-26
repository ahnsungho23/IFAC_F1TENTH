# 2026-07-25 Perception DB·맵 분석

## 1. 분석 입력

- 주행 DB: `rosbag2_2026_07_25-22_08_50`
- 정지 DB: `rosbag2_2026_07_25-22_08_36`
- 경로: `2026_07_25_map/map/global_waypoints.csv`

두 DB에는 `/scan`, `/sensors/imu/raw`, `/odom`, `/pf/pose/odom`, `/tf`가 있다. Perception
출력과 `/map`, `/tf_static`은 기록되지 않았다. 따라서 센서 품질과 전처리 효과는 분석할 수 있지만,
정적 장애물의 정답 기반 precision/recall은 이 DB만으로 계산할 수 없다. 전달된 맵 폴더도 SLAM
occupancy map이 아니라 글로벌 경로 파일이다.

## 2. DB 기본 통계

| 항목 | 주행 DB | 정지 DB |
|---|---:|---:|
| 길이 | 12.092 s | 0.652 s |
| 전체 메시지 | 4,262 | 143 |
| `/scan` | 444, 39.87 Hz | 23, 40.07 Hz |
| `/sensors/imu/raw` | 565, 50.01 Hz | 18, 49.91 Hz |
| `/odom` | 566, 50.03 Hz | 11, 50.42 Hz |
| `/pf/pose/odom` | 363, 30.39 Hz | 21, 31.40 Hz |
| 평균/최대 속도 | 1.73 / 4.05 m/s | 0 / 0 m/s |
| MCL 이동 거리 | 19.83 m | 0.015 m |

LiDAR는 프레임당 1,081개 빔이며 `scan_time=0.025 s`이다. 유효 빔 비율은 주행 DB 평균
98.15%, 정지 DB 평균 95.21%다. scan과 가장 가까운 IMU/odom header stamp 차이는 주행 DB에서
중앙값 약 5 ms, 최댓값 약 10 ms이므로 현재 `deskew_sensor_timeout=0.15 s` 안에 충분히 들어온다.

## 3. Deskew 판단

주행 DB의 IMU z축 원시 회전률 범위는 `-212.53~126.77`이고, `π/180`을 적용하면 최대 절댓값은
3.71 rad/s다. 한 스캔 동안 생길 수 있는 회전 왜곡은 최대 약 5.31도다. 따라서 회전 deskew는
유지하는 것이 타당하다.

IMU와 odom 회전률의 단순 최소제곱 scale은 0.02693, 상관계수는 0.73이었다. 현재 `π/180`
(0.01745)은 센서가 deg/s를 내보낸다는 관찰에는 맞지만 odom과 완전히 일치하지 않는다. odom 자체의
지연, 필터, 기준 프레임 차이가 섞여 있으므로 이 DB 하나로 scale을 0.02693으로 바꾸지 않는다.
다음 기록에는 원시 IMU 단위 확인용 정지·일정 회전 시험을 포함해야 한다.

## 4. Raw-noise filter 분석

현재 파라미터(`noise_eps=0.05`, `noise_eps_scale=1.5`, 이웃 2개, 반창 4)를 두 DB에 오프라인
적용했다.

| 항목 | 주행 DB | 정지 DB |
|---|---:|---:|
| 평균 beam 제거율 | 0.0497% | 0% |
| 95% beam 제거율 | 0.195% | 0% |
| 최대 beam 제거율 | 0.486% | 0% |
| 장애물 크기 클러스터가 줄어든 프레임 | 0 | 0 |

이 기록에서는 필터가 공격적이지 않았지만 실제 작은 정적 장애물 정답이 없으므로
`noise_filter_enable=false`를 유지한다. 장애물 표적을 배치한 DB와 f1sim_C sweep에서 recall을
확인한 뒤 기본 활성화를 결정한다.

## 5. 맵 곡률 분석

글로벌 경로는 132점, 길이 약 33.0 m, 평균 간격 0.2516 m다. 절대 곡률은 평균 0.278,
95 percentile 0.725, 최댓값 1.032 rad/m다.

주요 급곡률 구간은 다음과 같다.

| 조건 | s 구간 [m] |
|---|---|
| `|kappa| >= 0.7` | 0.50–1.01, 12.33–13.33 |
| `|kappa| >= 0.5` | 0.00–1.76, 12.33–13.84, 17.11–18.36, 19.62, 29.94–30.19 |

특히 `s=12.33–13.33 m` 구간이 최대 곡률 hairpin이다. 향후 분산 분석은 직선
`|kappa|<0.3`, 중간 `0.3<=|kappa|<0.7`, hairpin `|kappa|>=0.7`로 나눠 Cartesian
`x/y` 검출 중심의 반복 관측 분산과 CLCS 변환 후 `s/d` 분산을 함께 비교한다.

## 6. 결론과 코드 반영

1. CLCS는 동적 상대차 추적과 로컬 경로의 트랙 위상 보존을 위해 계속 사용한다.
2. 정적 장애물 외부 인터페이스는 Cartesian `(x,y)`, Frenet `(s,d)`, 전체 AABB를 감싸는
   최대 반지름 `r`로 분리한다.
3. 로컬 플래너는 Cartesian 중심을 CLCS로 한 번 투영하고 원의 반지름을 s/d 양축에 동일하게
   적용한다. Hairpin에서 네 모서리를 각각 투영해 다른 트랙 가지로 넘어가는 방식을 사용하지 않는다.
4. 회전 deskew, fragment merge, 가변 측정 공분산은 유지한다.
5. raw-noise filter는 이번 DB에서는 안전해 보이지만 정답 데이터가 없어 기본 비활성화를 유지한다.

## 7. 참고 브랜치 검토

- `origin/parkm_combine`: ordered-scan DBSCAN core-point filter를 기본 활성화하고, 정적·동적 출력을
  `/static_obs`, `/opp_obs`로 분리한다. 또한 Frenet local planner를 함께 둔다.
- `origin/layered-obstacle-detector`: map/static/dynamic 3-layer 분류와 같은 계층별 출력 구조를
  유지하지만 raw-noise filter는 제거돼 있다.

현재 구현은 두 브랜치의 유효한 방향만 가져왔다. 정적 출력은 별도 토픽으로 분리하고 좌표 계약은
Cartesian `(x,y)`, Frenet `(s,d)`, 최대 반지름 `r`로 정했다. ordered-scan noise filter 코드는 유지하지만
이번 DB의 정답 부족 때문에 기본 활성화하지 않았다. 동적 상대차의 속도 분류와 CLCS 추적은 기존
Frenet 방식을 유지한다.
