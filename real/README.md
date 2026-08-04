# real/ — 실차 원클릭 실행 (Terminator + SSH)

실차 풀스택(`src/f1tenth_control/LAUNCH_FULLSTACK.md` §3)을 매번 손으로 띄우는 대신,
Terminator 한 창에 분할 화면으로 띄웁니다. 모든 창이 젯슨(`$F1_HOST`, 기본 `miru@10.1.1.3`)에
ssh로 붙어 실행합니다. `sim/`의 런처 구조를 실차용으로 옮긴 것입니다.

## 빠른 시작

```bash
~/2026_IFAC/real/global.sh    # 글로벌 주행만 (6창, local_planning 없음)
~/2026_IFAC/real/local.sh     # 풀스택 (7창, local_planning + obstacle_detector 포함)
```

전제: 젯슨에 ssh 키 인증이 등록돼 있을 것 (`ssh-copy-id miru@10.1.1.3` 한 번이면 이후 묻지 않음).

## 창 구성 (LAUNCH_FULLSTACK.md §3 순서)

| 창 | global.sh | local.sh | 내용 |
|---|---|---|---|
| 1 | O | O | bringup (`f1tenth_stack bringup_launch.py`) |
| 2 | O | O | MCL (`mod:=real map_name:=$F1_MAP_NAME use_rviz:=false`) |
| 3 | O | O | global_planning (`F1_MAP=$F1_MAP_NAME`) |
| 4 | — | O | local_planning `simulator:=false` (obstacle_detector 기본 포함) |
| 5 | O | O | state_machine (`/state` + `/local_waypoints`) |
| 6 | O | O | f1tenth_control (`control_real.launch.py`) — 마지막에 기동 |
| 7 | O | O | RViz(ssh -X) + `ros2 bag record -a --start-paused` |

- global 모드에서는 local_planning이 없으므로 state_machine이 GLOBAL에 머물며 글로벌 라인을
  `/local_waypoints`로 그대로 릴리합니다.
- 녹화는 **일시정지 상태로 시작**합니다. 재개/정지:
  `ros2 service call /rosbag2_recorder/resume rosbag2_interfaces/srv/Resume` (`.../pause .../Pause`)
- 초기 위치는 수동: 차 정차 → 창 7 RViz에서 **2D Pose Estimate** → 스캔·벽 겹침 확인 후 창 6 기동.

## 개별 실행 / 기타 역할

```bash
~/2026_IFAC/real/run_real.sh <role> [name:=value ...]
# roles: bringup | mcl | global | local | state | control | rviz | rvizlocal | stop [--all] | scratch
~/2026_IFAC/real/run_real.sh control max_speed:=2.5 min_speed:=0.5   # 셰이크다운
~/2026_IFAC/real/run_real.sh stop                                    # 스택만 원격 종료
~/2026_IFAC/real/run_real.sh stop --all                              # bringup까지 종료
```

## 환경변수

| 변수 | 기본값 | 설명 |
|---|---|---|
| `F1_HOST` | `miru@10.1.1.3` | 젯슨 ssh 대상 (`~/.ssh/config`에 Host 별칭을 만들어두면 그 이름도 가능) |
| `F1_MAP_NAME` | `map` | 지도 이름 (MCL `map_name:=` / global·local의 `F1_MAP`) |

## 주의사항

- ssh는 비대화형이라 젯슨 `~/.zshrc`를 안 읽습니다 — `ROS_DOMAIN_ID=67`·`RMW_IMPLEMENTATION`·
  `F1_MAP`은 `run_real.sh`가 창마다 명시 export합니다 (젯슨 zshrc의 잘못된 `F1_MAP`과 무관하게 동작).
- terminator 템플릿의 `title`에 **쉼표(,)를 넣으면 크래시**합니다(리스트로 파싱됨). 쉼표 금지.
- 창 7의 RViz는 ssh X-forward라 소프트웨어 렌더링(`LIBGL_ALWAYS_SOFTWARE=1`)이라 느릴 수 있습니다.
  답답하면 `run_real.sh rvizlocal`로 본체에서 띄우세요 (토픽은 도메인 67로 그대로 보임).
- RViz를 닫으면 그 창의 rosbag 녹화도 같이 종료됩니다.
