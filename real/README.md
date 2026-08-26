# real/ — 실차 원클릭 실행 (Terminator + SSH)

실차 풀스택(`src/f1tenth_control/LAUNCH_FULLSTACK.md` §3)을 매번 손으로 띄우는 대신,
Terminator 한 창에 분할 화면으로 띄웁니다. 모든 창이 젯슨(`$F1_HOST`, 기본 `miru@10.1.1.1`)에
ssh로 붙어 실행합니다. `sim/`의 런처 구조를 실차용으로 옮긴 것입니다.

## 빠른 시작

```bash
~/2026_IFAC/real/global.sh    # 글로벌 주행만 (6창, local_planning 없음)
~/2026_IFAC/real/local.sh     # 풀스택 (7창, local_planning + obstacle_detector 포함)
```

전제: 젯슨에 ssh 키 인증이 등록돼 있을 것 (`ssh-copy-id miru@10.1.1.1` 한 번이면 이후 묻지 않음).

## 창 구성 (실차 실행 순서)

| 창 | role | global.sh | local.sh | 실제로 도는 명령 |
|---|---|---|---|---|
| 1 | `bringup` | O | O | **시각 동기화** → `sudo jetson_clocks && f110` |
| 2 | `ping` | — | O | `real/lidar_link_watch.sh 192.168.0.10` — 기동 1회 점검 후 감시 |
| 3 | `mcl` | O | O | `cd ~/2026_IFAC && sc && ros2 launch kinematic_localization kinematic_localization.launch.py map_name:=$F1_MAP_NAME` |
| 4 | `global` | O | O | `... ros2 launch global_planning global_planning.launch.py map_name:=$F1_MAP_NAME` |
| 5 | `local` | — | O | `... ros2 launch local_planning local_planning.launch.py` |
| 6 | `state` | O | O | `... ros2 launch state_machine state_machine.launch.py` |
| 7 | `control` | O | O | `... ros2 launch f1tenth_control control_real.launch.py` — 마지막에 기동 |
| — | `rviz` | O | — | RViz(ssh -X) + `ros2 bag record -a --start-paused` |

🔴 **`local.sh` 레이아웃에는 RViz·rosbag 창이 없다.** 녹화가 필요하면 별도 창에서
기본 시각화는 **Foxglove**입니다 — 레이아웃에 페인이 들어 있고, 본체 PC 브라우저에서
app.foxglove.dev → *Open connection* → `ws://10.1.1.1:8765`로 붙습니다.
RViz가 필요하면 `~/2026_IFAC/real/run_real.sh rviz`(젯슨 X-forward + bag) 또는 `... rvizlocal`(본체 RViz만).
⚠️ `rvizlocal`이 싣는 `kicp_real.rviz`는 **safety 브랜치에 없습니다**(main에만 있음) — 기본 `rviz2`로 뜹니다.
rosbag 녹화는 젯슨에서 `f1rec` alias로 따로 돌립니다.

⚠️ **2번 창이 진단의 반쪽이다.** 응답이 끊기면 라이다 **이더넷 링크**가 죽은 것이고,
ping은 되는데 `/scan`만 멈추면 **라이다 본체/드라이버**다. 2026-08-25 백에서 라이다가 충돌
1.3초 뒤 `Not Connected`로 죽고 89초간 안 돌아왔는데, 그 둘을 현장에서 가를 계기가 없었다.

🔴 **기동할 때 딱 한 번** `192.168.0.10`을 점검합니다(`real/lidar_link_watch.sh`).
`Destination Host Unreachable`이면 `usbc_power.sh cycle --force`로 USB-C 허브를 재부착하고
다시 확인합니다. **그 뒤로는 ping만 흘리고 자동 복구를 하지 않습니다.**

같은 허브에 **조이스틱이 물려 있어** 사이클 중에는 `/joy`가 끊깁니다 = 조이스틱 E-stop을
못 씁니다. 그래서 차가 서 있고 자율 미체결인 **기동 시점에만** 복구합니다. 주행 중에
링크가 죽으면 이 창이 unreachable을 찍어주고, 복구는 사람이 판단해서 수동으로 돌립니다.

| 변수 | 기본값 | 뜻 |
|---|---|---|
| `LIDAR_PROBE_COUNT` | `5` | 기동 점검에 던질 ping 개수 |
| `LIDAR_PROBE_DEADLINE` | `6` | 점검 제한시간 [s] — ⚠️ **3.5초 밑으로 줄이지 말 것** |
| `LIDAR_MAX_CYCLES` | `2` | 기동 점검에서 재부착해 볼 최대 횟수 |
| `LIDAR_SETTLE` | `12` | 재부착 뒤 링크를 기다리는 시간 [s] |
| `LIDAR_AUTOCYCLE` | `1` | `0`이면 점검만 하고 복구 안 함 |

⚠️ **점검을 짧게 하면 안 됩니다.** ARP 해석이 실패해 커널이 `Destination Host Unreachable`을
내기까지 **약 3.1초**가 걸립니다(젯슨 실측). ARP 캐시가 비어 있는 기동 시점이 정확히 그
경우라, `-i 0.3` 같은 빠른 점검은 2.3초에 끝나 진짜 unreachable을 "무응답"으로 오판하고
복구를 건너뜁니다. 무응답(타임아웃)은 허브 문제가 아니므로 **일부러 재부착하지 않습니다.**

⚠️ 재부착에 성공해도 그 사이 `urg_node`가 죽었을 수 있습니다 — **1번 창(bringup)을 다시
띄우세요.** 스크립트가 그렇게 안내합니다.

- global 모드에서는 local_planning이 없으므로 state_machine이 GLOBAL에 머물며 글로벌 라인을
  `/local_waypoints`로 그대로 릴리합니다.
- 녹화는 **일시정지 상태로 시작**합니다. 재개/정지:
  `ros2 service call /rosbag2_recorder/resume rosbag2_interfaces/srv/Resume` (`.../pause .../Pause`)
- 초기 위치는 수동: 차 정차 → Foxglove(또는 RViz)에서 **2D Pose Estimate** →
  스캔·벽 겹침 확인 후 7번 control 기동.

## 개별 실행 / 기타 역할

```bash
~/2026_IFAC/real/run_real.sh <role> [name:=value ...]
# roles: bringup | ping | mcl | global | local | state | control | time | rviz | rvizlocal | foxglove | stop [--all] | scratch
~/2026_IFAC/real/run_real.sh time                                     # 젯슨 시계를 이 기기에 강제로 맞춤
~/2026_IFAC/real/run_real.sh control max_speed:=2.5 min_speed:=0.5   # 셰이크다운
~/2026_IFAC/real/run_real.sh stop                                    # 스택만 원격 종료
~/2026_IFAC/real/run_real.sh stop --all                              # bringup까지 종료
```

## 환경변수

| 변수 | 기본값 | 설명 |
|---|---|---|
| `F1_HOST` | `miru@10.1.1.1` | 젯슨 ssh 대상 (`~/.ssh/config`에 Host 별칭을 만들어두면 그 이름도 가능) |
| `F1_MAP_NAME` | `map` | 지도 이름 — `mcl`·`global` 둘 다 `map_name:=` 인자로 넘긴다 |
| `F1_TIME_SKEW_MAX` | `1.0` | 이 초를 넘게 어긋났을 때만 젯슨 시계를 건드린다 |
| `F1_TIME_SAMPLES` | `7` | 시각 측정 표본 수 (최소 RTT 표본만 쓴다) |

## 주의사항

- 🔑 **원격 명령은 대화형 zsh(`zsh -ic`) 안에서 돕니다.** `f110`·`sc`가 젯슨 `~/.zshrc`의
  alias라 비대화형 ssh로는 `command not found`가 납니다.
  ⚠️ **alias는 parse 시점에 확장**되므로 `source ~/.zshrc; sc && ros2 ...`를 한 줄로 보내면
  안 먹습니다 — `zsh -i`가 rc를 먼저 읽고 `-c` 문자열을 그 뒤에 파싱하는 순서라야 합니다.
- `ROS_DOMAIN_ID=70`·`RMW_IMPLEMENTATION`은 rc를 읽은 **뒤에** export하므로 젯슨 zshrc에
  다른 값이 있어도 이깁니다.
- 1번 창의 `sudo`는 비밀번호를 물을 수 있습니다 — `ssh -t`로 tty를 주므로 프롬프트가 정상 동작합니다.
  (2026-08-25 현재 젯슨은 `sudo NOPASSWD`라 실제로는 안 묻습니다.)
- 🔑 **1번 창이 기동 전에 젯슨 시계를 이 기기에 맞춥니다.** 젯슨 RTC에는 백업 배터리가 없어
  전원을 끊으면 시계가 1970으로 가고, 차량망에는 상위 NTP가 없어 `systemd-timesyncd`가
  "동기화됨"이라 보고해도 실제로는 아무 데서도 시각을 못 받습니다. bag·`jetson_load.sh`·분석
  스크립트가 전부 "같은 시계"를 전제하므로, 어긋나면 두 기록의 정렬이 **조용히** 틀립니다.
  - ⚠️ **측정은 여러 번 재서 RTT가 가장 작은 표본만 씁니다**(NTP의 minimum filter). 단발
    왕복으로 재면 시끄러운 WiFi에서 거짓말합니다 — 2026-08-25 실측에서 같은 순간의 추정치가
    RTT 0.18 s→+0.13 s / 0.96 s→+0.53 s / 2.69 s→+1.37 s로 **RTT에 그대로 비례**했습니다.
    최소 RTT(0.044 s) 표본으로 재니 실제 오차는 **+0.044 s ±0.022 s**였습니다.
  - `F1_TIME_SKEW_MAX`(기본 1.0 s) 이내면 건드리지 않습니다. 강제로 맞추려면 `run_real.sh time`.
  - 설정 후에도 어긋나 있으면 `systemd-timesyncd`가 되돌린 것이니 젯슨에서 한 번만
    `sudo timedatectl set-ntp false`.
- 🔴 **`fastdds_car.xml`은 현재 적용되어 있지 않습니다 (2026-08-26 확인).**
  이 문서는 한동안 "젯슨이 이 프로파일을 쓴다"고 적어 뒀지만, 젯슨 `~/.zshrc`에
  `FASTRTPS_DEFAULT_PROFILES_FILE` export가 **없습니다** — `zsh -ic 'echo $FASTRTPS_DEFAULT_PROFILES_FILE'`
  가 빈 값입니다. `~/fastdds_car.xml` 파일만 남아 있습니다. `.zshrc` mtime이 `Jan 1`이라
  RTC가 1970으로 리셋된 구간에 `.zshrc`가 덮어써지면서 그 한 줄이 날아간 것으로 보입니다.
  - ⚠️ **08-25 이후 백을 분석할 때 이걸 전제로 두지 마세요.** 스톨이 남아 있었다면
    "조치가 안 먹혔다"가 아니라 "조치가 애초에 안 걸려 있었다"일 수 있습니다.
  - 시각화를 Foxglove로 옮기면서(브릿지가 젯슨 안에서 구독) 원격 RELIABLE 리더 자체가
    없어졌으므로 **당장 되살리지 않습니다.** 다시 켜려면 젯슨 `~/.zshrc`에
    `export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds_car.xml` 한 줄.
  - 정본은 `real/fastdds_car.xml`, 젯슨 사본은 `~/fastdds_car.xml` — **고치면 양쪽을 같이.**
- terminator 템플릿의 `title`에 **쉼표(,)를 넣으면 크래시**합니다(리스트로 파싱됨). 쉼표 금지.
- 🔑 **RViz는 `src/kinematic_localization/rviz/kicp_real.rviz`를 기본으로 싣습니다** — 모든 구독이
  **BEST_EFFORT · Depth 1**입니다. 경기장 WiFi가 시끄러울 때 RELIABLE 구독자는 발행자에게
  역압을 걸고, KICP는 단일스레드 실행기라 그 역압에 노드가 통째로 섭니다
  (`docs/kinematic_localization.md` §10-7-7 — ⚠️ 이 절은 **main 브랜치에만** 있습니다).
  RELIABLE 발행자 ↔ BEST_EFFORT 구독자는 정상
  매칭이라 **데이터는 그대로 옵니다**. 다른 설정을 쓰려면 `F1_RVIZ_CFG=<경로>`.
  ⚠️ `/map` 구독이 Volatile이라 **맵이 뜨는 데 최대 10초** 걸립니다(고장 아님).
  ⚠️ TF는 못 낮춥니다 — `tf2_ros::TransformListener`가 QoS를 코드에 박아둡니다.
- 🟢 **시각화 대안 — `run_real.sh foxglove` (권장, `real/foxglove_bridge.yaml`)**
  `foxglove_bridge`를 **젯슨에서** 띄우고 본체 PC는 브라우저로 붙습니다
  (https://app.foxglove.dev → *Open connection* → `ws://10.1.1.1:8765`).
  🔑 브릿지가 젯슨 안에서 구독하므로 **DDS가 WiFi를 아예 안 넘습니다** — RViz를 본체에서
  띄웠을 때 KICP를 세우던 원격 RELIABLE 리더가 없어집니다. WiFi에는 WebSocket(TCP) 하나만
  흐르고 그 지연은 브릿지 송신 버퍼(`send_buffer_limit`, 넘으면 대기 없이 드롭)가 흡수합니다.
  🔑 **`/tf`도 같이 해결됩니다** — `rvizlocal`의 BEST_EFFORT 대책이 못 덮던 부분입니다
  (`tf2_ros::TransformListener`가 QoS(100) RELIABLE을 코드에 박아둠). 리스너가 젯슨 안으로
  들어오면 그 경로가 무해해집니다.
  🔑 구독 QoS를 **일부러 안 낮췄습니다**(로컬이라 역압 위험 없음) → `/map`·`/global_waypoints`의
  래치 샘플이 즉시 옵니다. `rvizlocal`의 "맵이 10초 뒤에 뜬다" 문제가 여기서는 없습니다.
  🔑 브릿지는 **뷰어가 실제로 켠 패널의 토픽만** 그때 구독합니다 — `topic_whitelist: [".*"]`
  여도 WiFi 트래픽은 보고 있는 만큼만 나갑니다.
  🔑 **광고 토픽을 RViz 기본 설정이 그리던 것만으로 좁혔습니다** — `/tf`·`/tf_static`·`/map`·
  `/slow_scan`·`/pf/pose/odom`·`/global_waypoints/markers`·`/local_waypoints/path`·
  `/perception/obstacles/markers`. 기본 `.*`는 젯슨 전 토픽(46채널)을 광고하면서
  `vesc_msgs` 스키마를 못 찾아 에러를 반복합니다(그 패키지는 `~/f1tenth_ws`에 있는데 이
  role은 `~/2026_IFAC`만 source합니다). ⚠️ 정규식 **완전 일치**라 접두사만 적으면 안 됩니다.
  🔑 **`/scan`(40 Hz)은 일부러 뺐습니다.** 대신 role이 `topic_tools throttle`로
  `/slow_scan`(기본 5 Hz, `F1_SCAN_HZ`로 조절)을 만들어 그것만 광고합니다 — 40 Hz LaserScan을
  WiFi로 넘기면 송신 버퍼가 밀려 화면이 뒤처집니다. 브릿지를 끄면 throttle도 같이 정리됩니다.
  🔴 **뷰어가 발행할 수 있는 토픽을 `/initialpose` 하나로 좁혔습니다.** 기본
  `capabilities`에는 `clientPublish`·`services`가 다 들어 있어 브라우저에서 누구나 `/drive`를
  발행하거나 서비스를 호출할 수 있습니다. `services`는 빼고, `clientPublish`는 남긴 채
  `client_topic_whitelist: ["/initialpose"]`로 제한했습니다 — rviz2를 안 쓰므로 **2D Pose
  Estimate 수단은 남겨야** 하기 때문입니다(자동 초기화가 residual 게이트에 걸리면 사람이 줘야 함).
  📐 **패널 레이아웃: `real/foxglove_layout.json`** — Foxglove 좌측 *Layouts* → **Import from file**
  로 불러옵니다(뷰어 PC에서 하는 작업이라 젯슨과 무관). 3D 패널 하나(맵·`/slow_scan`·pose 화살표·
  글로벌/로컬 웨이포인트·장애물 마커 + 그리드)와 `/pf/pose/odom` Raw Messages 패널로 구성됩니다.
  - 🔑 **`followTf`가 `map`으로 고정**돼 있고 **Pose estimate 툴의 발행 토픽이 `/initialpose`**로
    맞춰져 있습니다. 이 둘이 초기 포즈를 제대로 찍기 위한 핵심입니다.
  - 레이아웃이 쓰는 토픽은 전부 위 화이트리스트 안에 있습니다(교차 검증 완료).
  - ⚠️ Foxglove에서 손으로 고친 뒤에는 *Export* 해서 이 파일을 갱신하세요.
  ⚠️ 젯슨에 패키지 설치 필요: `sudo apt install ros-jazzy-foxglove-bridge`.
  ⚠️ `fastdds_car.xml`은 **그대로 둡니다** — 발행자 쪽 상한이라 층이 다르고 대체재가 아닙니다.
- `rviz` role의 RViz는 ssh X-forward라 소프트웨어 렌더링(`LIBGL_ALWAYS_SOFTWARE=1`)이라 느릴 수 있습니다.
  답답하면 `run_real.sh rvizlocal`로 본체에서 띄우세요 (토픽은 도메인 70로 그대로 보임).
- RViz를 닫으면 그 창의 rosbag 녹화도 같이 종료됩니다.
