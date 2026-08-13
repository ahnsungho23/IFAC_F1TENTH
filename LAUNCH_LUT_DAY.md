# 실차 LUT 측정 데이 체크리스트

목표: 장애물 없는 클린 랩 **6회**로 추적오차 LUT 실차값을 얻는다.
(2026-08-13 1·2차 시도에서 trace 전량 유실 — 원인·대책이 아래 절차에 반영돼 있음.
 배경 1: referee는 랩 완주/no_start/timeout 때만 스스로 저장하며 Ctrl+C는 저장 없이 죽는다.
 배경 2 — **referee "odom 수신 0건"의 직접 원인(2026-08-13 확정, 수정 완료)**:
 lap_referee.launch.py가 `odom_topic` 인자를 **선언하지 않아** runner가 넘긴
 `odom_topic:=/pf/pose/odom`이 조용히 버려졌고(launch는 미선언 인자를 에러 없이 무시),
 referee가 시뮬 전용 `/ego_racecar/odom`을 구독해 실차에서 영원히 굶었다. runner의
 사전 체크(`topic echo /pf/pose/odom`)는 올바른 토픽을 봐서 🟢가 떴던 것 — 그래서 더
 헷갈렸다. launch에 인자 선언+전달을 추가해 수정했다 (**젯슨 배포본도 재배포 필요**).
 배경 3 — 통신 계층 보강: HY_MIRU는 멀티캐스트가 안 흐르는 망이고, `ROS_STATIC_PEERS`의
 유니캐스트는 FastDDS 기본 `maxInitialPeersRange=4` 때문에 젯슨 참가자 인덱스 0~3까지만
 닿는다(젯슨 풀스택은 프로세스 8개+) — `ros2 topic list` 공백·디스커버리 복불복의 원인.
 `tools/fastdds_car_client.xml` 프로필이 0~31까지 커버한다(car_env.sh·runner 자동 적용).
 랩탑 ufw는 무죄(7/29부터 ENABLED=no, 차단 로그 0건 — 11:02에 넣은 allow 규칙은 무의미).
 대책 = Plan A(referee를 젯슨에서 실행, §2-1) + Plan B(랩탑 + 프로필).)

## 0. 속도 계획 (총 6랩, 장애물 없음)

| 순서 | control_real 인자 | 랩 수 | runner prefix | 채우는 LUT 영역 |
|---|---|---|---|---|
| 1 | `max_speed:=2.0 min_speed:=2.0` | 2랩 | `slow20_lap1`, `slow20_lap2` | **저속 행 (최중요)** — 회피 하한속도 2.0이 실제로 쓰는 값 |
| 2 | `max_speed:=2.5 min_speed:=2.0` | 2랩 | `slow25_lap1`, `slow25_lap2` | 저·중속 행 |
| 3 | 평소 레이스 설정 (인자 없음) | 2랩 | `fast_lap1`, `fast_lap2` | 고속 셀 (v ≥ 3.0) |

- 정지·복구가 있었던 랩은 폐기하고 그 prefix로 재주행.
- 시간·배터리가 남으면 `max_speed:=1.6`·`2.9` 각 1랩 추가(셀 커버리지 향상, 필수 아님).
- ⚠️ 속도를 바꿀 때마다 **control_real 재시작** (파라미터는 시작 시 1회만 읽음).

## 1. 랩탑 준비

HY_MIRU(젯슨 핫스팟) 접속 — 인터넷 안 되는 게 정상.

```bash
# ① 1회만 (아무 터미널):
ros2 daemon stop
# (wifi 절전은 NetworkManager HY_MIRU 프로필에 powersave=disable로 영구 설정됨 — 2026-08-13.
#  확인: nmcli -f 802-11-wireless.powersave connection show HY_MIRU)

# ② RViz·f1rec 등 ROS를 쓰는 터미널마다 이 한 줄:
source ~/2026_IFAC/tools/car_env.sh    # = DOMAIN 70 + STATIC_PEERS + FastDDS 프로필(참가자 0~31)
```

⚠️ `ros2 topic list`가 비어 보이면 ① 그 터미널이 ②를 안 했거나 ② 데몬이 옛 환경이다 —
②를 소스한 터미널에서 `ros2 daemon stop` 후 재시도. (프로필 적용 전에는 새 프로세스가
디스커버리 복불복으로 진짜 0개가 나올 수도 있었다 — 이제는 환경 문제일 확률이 높다.)

runner(`tools/lut_lap_runner.sh`) 터미널은 **아무 설정도 불필요** — 스크립트가 자체 설정한다.
젯슨 터미널들도 `.zshrc`에 도메인 70이 있으므로 추가 조치 없음.

통신 자가진단 (스택이 뜬 뒤, **먼저 위 ②를 소스한 터미널에서** — 안 하면 도메인 0의
포트를 듣게 되어 [4]가 무조건 0개로 나온다):
```bash
source ~/2026_IFAC/tools/car_env.sh
bash ~/2026_IFAC/src/f1tenth_control/tools/f1net_client.sh
```
판독: [4]에 젯슨(10.1.1.1)이 뜨면 멀티캐스트까지 정상. **자기 IP·127.0.0.1만 떠도 이제
치명적이지 않다** — HY_MIRU는 멀티캐스트를 안 흘리는 게 실측 확인됐고(2026-08-13),
프로필의 유니캐스트 디스커버리가 그걸 우회한다. 최종 기준은 runner의 odom 프리체크다.

## 2. 풀스택 실행 순서 (LAUNCH_FULLSTACK.md 요약 + 도메인 70 기준)

젯슨 `.zshrc`의 `F1_MAP=map`·`ROS_DOMAIN_ID=70` 확인 후:

| # | 어디 | 명령 | 넘어가기 전 확인 |
|---|---|---|---|
| T1 | 젯슨 | `f110` | `/scan` 40Hz, `/odom` 50Hz, `/joy` 20Hz |
| T2 | 젯슨 | `cd ~/2026_IFAC && source install/setup.zsh` → `ros2 launch particle_filter_cpp mcl_launch.py mod:=real map_name:=map use_rviz:=false` | `/pf/pose/odom` 발행 |
| — | 랩탑 | RViz2 (위 1번 env 적용된 터미널에서) → **2D Pose Estimate** | 스캔이 벽과 정합 |
| T3 | 젯슨 | `cd ~/2026_IFAC` → `ros2 launch global_planning global_planning.launch.py` | 경로가 지도와 정합 |
| T4 | 젯슨 | `ros2 launch local_planning local_planning.launch.py simulator:=false` | — |
| T5 | 젯슨 | `ros2 launch state_machine state_machine.launch.py` | `/local_waypoints` 발행 |
| T6 | 젯슨 | `source ~/f1tenth_ws/install/setup.zsh && source ~/2026_IFAC/install/setup.zsh && cd ~/2026_IFAC` → `ros2 launch f1tenth_control control_real.launch.py max_speed:=2.0 min_speed:=2.0` | 마지막에. 표 0의 속도 인자 사용 |

## 2-1. Plan A (권장) — referee를 젯슨에서 실행

referee가 젯슨 로컬이면 odom이 같은 호스트라 **wifi가 데이터 경로에서 완전히 빠진다** —
디스커버리·수신 문제가 원천 제거되고 trace 타이밍 품질도 좋다. 배포(랩탑, 젯슨 옆에서):

```bash
bash ~/2026_IFAC/tools/deploy_referee_to_jetson.sh        # 기본 miru@10.1.1.1
```
⚠️ 랩탑에서 lap_referee 소스·launch·runner를 고칠 때마다 **재배포**해야 젯슨 사본에
반영된다 (예: 2026-08-13 odom_topic launch 수정 이후 반드시 1회 재배포).

이후 랩마다 **젯슨 터미널**에서:
```bash
bash ~/lut_lap_runner.sh slow20_lap1
```
(runner가 ~/lut_ws 오버레이를 자동 source. HB 로그·no_start 스모크 전부 동일하게 동작)

세션 끝나면 랩탑에서 회수:
```bash
mkdir -p ~/lut_traces && scp "miru@10.1.1.1:~/lut_traces/*" ~/lut_traces/
```

랩탑에서 runner를 돌리는 Plan B도 프로필 덕에 이제 동작해야 하지만, **한 번이라도 또
"odom 0건"이 나오면 고민 말고 Plan A로 갈 것.**

## 3. 주행 전 배선 스모크 (1분, 생략 금지)

차를 **세워둔 채** — Plan A면 젯슨에서 `bash ~/lut_lap_runner.sh smoke_test`, Plan B면 랩탑에서:
```bash
bash ~/2026_IFAC/tools/lut_lap_runner.sh smoke_test
```
- runner가 odom 수신을 먼저 검증한다 — 미수신이면 시작을 거부하고 진단 안내를 띄움
- odom이 잡히면 referee가 뜨고, 차가 서 있으므로 **8초 뒤 no_start로 자기 종료 +
  `~/lut_traces/smoke_test_*.json/csv` 생성** ← 이 파일이 생겨야 전 구간 배선 증명
- 파일이 안 생기면 주행을 시작하지 말 것

## 3-1. referee 심박(HB) 로그 읽는 법 (5초마다 1줄)

```
HB: phase=RUN progress=12.3/32.9m idx=48 lat=0.05 v=2.01 odom(n=1520, age=0.0s)
```
- **progress가 주행 중 계속 오르면 정상** — 한 랩이면 lap_complete로 자기 종료
- `odom age`가 커지거나 `n`이 안 늘면 → **odom 수신 두절** (통신 문제; `<prefix>_odom_hz.log`와 대조)
- progress는 0인데 `idx`가 **감소**하면 → 웨이포인트 순서가 주행 방향과 반대 (라이브 덤프가
  실패해 파일 사본으로 폴백한 경우에만 가능)
- `HB: odom 수신 0건` 경고가 반복되면 → 위치추정 토픽 자체가 안 들어옴
- 랩이 안 끝나면 **HB 줄 몇 개를 그대로 복사해 Claude에게** 주면 즉시 원인 특정 가능

## 4. 랩 절차 (랩마다 반복)

```
① (선택) f1rec <prefix>          # bag 보험 + MCL 공분산 분석용 — A 누르기 전 시작
② Plan A(젯슨 터미널): bash ~/lut_lap_runner.sh slow20_lap1
   Plan B(랩탑):       bash ~/2026_IFAC/tools/lut_lap_runner.sh slow20_lap1
   → "odom 수신 확인" 출력 확인
③ 조이스틱 A (자율 시작)
④ 1랩 완주 → referee가 'lap_complete' 찍고 스스로 종료 (⚠️ Ctrl+C 금지)
   → runner가 저장 파일과 lap_time을 출력
⑤ 파일 확인됐으면 다음 prefix로 ②부터 반복. 속도 구간이 바뀌면 T6 재시작
```

## 5. 끝나면

`~/lut_traces/`에 `<prefix>_summary.json` + `<prefix>_trace.csv` **6쌍**이 있는지 확인.
이후 처리(품질검사 → LUT 생성 `--safety-factor 1.25` → n=0 셀·규정 간격(2.0 m/s에서
≤0.109 m) 검증 → local_planning.yaml 반영)는 Claude 세션에 경로만 알려주면 진행됨.
상세 배경: src/local_planning/docs/local_planner.md "실차 LUT 갱신 절차".
