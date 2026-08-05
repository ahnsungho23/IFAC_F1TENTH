# Monte Carlo Localization (MCL)

High-performance particle filter localization for F1TENTH with unified real/simulation configuration.

## Quick Start

```bash
# Build
colcon build --packages-select particle_filter_cpp
source install/setup.bash

# Real car
ros2 launch particle_filter_cpp mcl_launch.py mod:=real

# Simulation
ros2 launch particle_filter_cpp mcl_launch.py mod:=sim

# Bag playback
ros2 launch particle_filter_cpp mcl_launch.py mod:=bag

# To change map, launch with map_name:='your_map'
```

## Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mod` | `real` | Launch mode: `real`, `sim`, or `bag` |
| `map_name` | `sibal1` | Map file to load |
| `use_rviz` | `true` | Launch RViz visualization |

## Topics

### Real Mode (`mod:=real`)
- **Input**: `/scan`, `/odom` - LiDAR and odometry data
- **Output**: `/pf/pose/odom` - Localized pose for planner/controller
- **Transforms**: `map → base_link`
- **Timing**: Real time

### Simulation Mode (`mod:=sim`)
- **Input**: `/scan`, `/ego_racecar/odom` - Simulation sensor data
- **Output**: `/pf/pose/odom` - Localized pose
- **Transforms**: `map → base_link`
- **Timing**: Simulation time

### Bag Playback Mode (`mod:=bag`)
- **Input**: `/scan`, `/odom` - Recorded LiDAR and odometry data
- **Output**: `/pf/pose/odom` - Localized pose
- **Transforms**: `map → base_link`
- **Timing**: Simulation time

### Visualization
- `/pf/viz/particles` - Particle cloud
- `/pf/viz/inferred_pose` - Estimated pose marker
- `/map` - Map display

## Key Configuration

Edit `config/mcl_config.yaml`:

```yaml
# Core MCL
max_particles: 4000           # Number of particles
update_rate: 100              # Real (Hz) / 200 (sim)
max_pose_range: 10000.0       # Map coordinate limits (m)

# Vehicle parameters (auto-set by sim_mode)
# Real hardware: wheelbase=0.325, lidar_offset=0.288
# Simulation: wheelbase=0.324, lidar_offset=0.25

# Pose EMA smoothing (2026-07-29 저속 지연 개선) — use_pose_ekf=false일 때만 사용
# 유효 alpha = min(alpha_max, smoothing_alpha + min(1, |v|/velocity_full)·alpha_gain)
# 출력 지연 시정수 τ ≈ (1/timer_frequency)·(1-α)/α — alpha가 작을수록 매끈하지만 느리다.
smoothing_alpha: 0.3               # 실차 base. 구값 0.05는 저속 τ≈0.63 s 지연 유발
smoothing_velocity_full_mps: 2.0   # 속도 적응 보정이 최대가 되는 속도
smoothing_alpha_gain: 0.4          # 최대 속도에서 base에 더해지는 폭
smoothing_alpha_max: 0.8           # 유효 alpha 상한
# 시뮬은 mcl_launch.py가 smoothing_alpha=0.5로 오버라이드 (모션 모델 튜닝과 한 세트)

# Pose fusion EKF (2026-07-29, 기본 활성) — 자세한 파라미터는 mcl_config.yaml 주석 참고
# 휠 odom 포즈 델타(laser 프레임 변환)로 매 주기 예측 + MCL 기대 포즈로 보정.
# 측정 노이즈 R = 파티클 가중 공분산에 "차체 종방향만" 25배 불신을 더해, 평행벽 복도의
# 진행방향 표류(구 0.8~1.7 m/랩)를 차단하고 코너에서 회전된 오차를 고게인으로 보정한다.
# sim 검증(2026-07-29): 5.0~7.0 m/s 프로파일에서 GT 대비 p50 1.1 cm / 최대 10 cm.
# sim 검증(2026-08-03, D1~D6 구조 수정 후): ifac_track 신규 생성 궤적 1.5~3.0 m/s,
# 180 s 주행 GT 대비 p50 4.4 cm / p95 8.8 cm. 단, x≈-18.5,y≈4.0 코너(궤적 곡률
# k=2.25 > 조향 한계 1.2 — 트랙이 물리적으로 타이트)에서 랩당 ~0.6 s 일시 이탈 후
# 자기회복(max 87 cm). 4.0~6.5 m/s 프로파일은 해당 코너에서 벽 충돌로 완전 발산.
use_pose_ekf: true
```

## Initialization

### RViz Method
1. Launch MCL with RViz (`use_rviz:=true`)
2. Use "2D Pose Estimate" tool to set initial pose
3. Odometry tracking starts immediately

### Global Method
- Automatic initialization when MCL converges
- No manual intervention required
- Activates when pose estimate stabilizes

## Available Maps

Place map files in `maps/` directory:
- `sibal1` (default racing circuit)
- `Spielberg_map` (F1 Austria GP)
- `levine` (multi-floor building)
- `map_1753950572` (real sensor data)

## Algorithm

Monte Carlo Localization with dual-rate architecture:
- **High-frequency odometry tracking** (100-200 Hz): Smooth interpolation
- **Low-frequency MCL corrections** (~6 Hz): Drift correction from sensors

## Prerequisites

- Map file loaded in `maps/` directory
- LiDAR and odometry data available
- Sufficient computational resources for particle filtering