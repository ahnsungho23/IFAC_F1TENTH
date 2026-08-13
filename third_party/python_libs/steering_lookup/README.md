# Steering lookup

ROS 2 Jazzy용 Python steering lookup 라이브러리입니다. 시스템 식별로 만든 CSV에서 속도와
횡가속도에 대응하는 조향각을 보간합니다.

## 빌드

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-select steering_lookup
source install/setup.zsh
```

## 사용

```python
from steering_lookup.lookup_steer_angle import LookupSteerAngle

steer_lookup = LookupSteerAngle('NUC6_glc_pacejka')
steer_angle = steer_lookup.lookup_steer_angle(accel=5.0, vel=3.5)
```

CSV는 `cfg/<model_name>_lookup_table.csv` 형식으로 패키지 share에 설치됩니다. 현재
`NUC2_hangar_pacejka`, `NUC6_glc_pacejka`, `SIM_linear` 모델을 제공합니다.
