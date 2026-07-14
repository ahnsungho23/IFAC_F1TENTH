# ifac_track 맵

손그림 트랙 스케치를 F1TENTH MCL(`particle_filter_cpp`) 로컬라이제이션용 점유격자로 변환한 맵입니다.

## 1. 규격

| 항목 | 값 |
|---|---|
| 이미지 | `ifac_track.png` (8-bit grayscale, 알파 없음) |
| 픽셀 크기 | 880 × 320 px |
| 해상도 | 0.025 m/px |
| 맵 전체 크기 | **22.0 × 8.0 m** (캔버스 = OccupancyGrid extent) |
| origin | `[-19.6624, -1.5854, 0.0]` |
| mode / negate | `trinary` / `0` |
| occupied_thresh / free_thresh | `0.65` / `0.25` |

## 2. 픽셀값 관례

nav2 map_server(`negate=0`)는 `occ = (255 − pixel)/255` 로 분류합니다.

| 픽셀 | occ | OccupancyGrid | 의미 |
|---|---|---|---|
| 0 (검정) | 1.000 | 100 | 벽(occupied) — 외벽·인필드 섬 경계 |
| 128 (회색) | 0.498 | −1 | 미지(unknown) — 트랙 바깥 + 섬 내부 |
| 254 (흰색) | 0.004 | 0 | 자유(free) — 주행 코리도 |

> 128은 밴드 `[0.25, 0.65]` 중앙이라 표준 임계값에서 견고하게 unknown으로 로드됩니다. (기준 맵 `fuck_f1`은 205 + free_thresh 0.25 조합이라 바깥이 free로 로드되는 문제가 있었고, 본 맵은 이를 교정.)

## 3. 좌표계 / 시작점

- **시작점 = 트랙 오른쪽 아래 90° 코너 코리도 중앙 = 월드 (0, 0)** 으로 가정하여 origin을 역산했습니다.
- 픽셀↔월드 변환(상하 반전 반영, H=320):
  - `world_x = origin_x + (u + 0.5)·res`
  - `world_y = origin_y + (H − v − 0.5)·res`
- 맵 범위: x ∈ [−19.66, 2.34] m, y ∈ [−1.59, 6.41] m.
- `origin[2] = 0`(맵 회전 없음). 차량 **초기 pose(주행 시작 방향)** 는 맵 회전이 아니라 RViz `2D Pose Estimate` 또는 시뮬 스폰에서 별도 지정합니다.

## 4. 생성 방법 (재현)

1. 원본 스크린샷 `Screenshot from 2026-07-14 21-33-58.png`(1016×271)에서 트랙 영역 crop (`x[335..863] y[55..249]`).
2. grayscale 임계값(<160)으로 벽 마스크 추출 → 종횡비 보존해 830×304로 리사이즈 → 1px 팽창(벽 ≥2px, 4-연결 폐합).
3. 880×320 캔버스(배경 128)에 중앙 배치, 벽 = 0.
4. 4-연결 컴포넌트 라벨링: 테두리와 닿는 라벨 = 바깥(unknown), 내부 최대 컴포넌트 = 코리도(free=254), 나머지 섬 = unknown.
5. 우하단 코너 코리도 중심을 시작점으로 origin 계산 → PNG/YAML 저장.

> 생성·검증 스크립트: 세션 scratchpad의 `build_map.py`, `finalize.py`. 손그림을 바꾸면 재실행해 갱신합니다.

## 5. 사용법

```bash
cd ~/2026_IFAC
colcon build --packages-select particle_filter_cpp      # 맵을 install에 반영
source /opt/ros/humble/setup.zsh && source install/setup.zsh
ros2 launch particle_filter_cpp mcl_launch.py mod:=sim map_name:=ifac_track use_rviz:=true
```

## 6. 검증 결과

- 실제 `nav2_map_server` 로드 확인: `width=880, height=320, resolution=0.025, origin=[-19.6624,-1.5854,0]`.
- `/map` 셀 분포: **occupied(100) 24562 / free(0) 131837 / unknown(−1) 125201** — 예측과 정확히 일치, 바깥/섬이 unknown으로 로드됨.
- PF 정합: 초기화는 `data==0`(free)에만 시드([particle_filter.cpp:272](../src/particle_filter.cpp)), 광선 장애물은 `data>50`([:814](../src/particle_filter.cpp)) → 바깥 unknown은 시드 제외·광선 투과로 적합.

## 7. 참고 / 한계

- 손그림 종횡비(2.731)를 보존해 트랙 실제 점유 범위는 약 **20.75 × 7.6 m**(22×8 캔버스 중앙 배치). 22 m 폭에 꽉 채우려면 리사이즈 목표를 864×304로 바꿔 재생성.
- 벽 형상은 손그림 기반 근사치입니다. 실측 트랙과 다르면 스케치를 정밀화 후 재생성하세요.
- 플래너용 센터라인 CSV(`new_map_con/maps`)와 글로벌 웨이포인트는 본 맵 좌표계에 맞춰 별도 생성해야 합니다.
