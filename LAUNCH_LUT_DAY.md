# 실차 LUT 측정 데이 체크리스트

목표: 장애물 없는 클린 랩 **6회**로 추적오차 LUT 실차값을 얻는다.
(2026-08-13 1차 시도에서 trace 전량 유실 — 원인·대책이 아래 절차에 반영돼 있음.
 배경: referee는 랩 완주/no_start/timeout 때만 스스로 저장하며 Ctrl+C는 저장 없이 죽는다.
 젯슨 핫스팟은 AP→클라이언트 멀티캐스트가 불안정해 STATIC_PEERS 유니캐스트로 우회한다.)

## 0. 속도 계획 (총 6랩, 장애물 없음)

| 순서 | control_real 인자 | 랩 수 | runner prefix | 채우는 LUT 영역 |
|---|---|---|---|---|
| 1 | `max_speed:=2.0 min_speed:=2.0` | 2랩 | `slow20_lap1`, `slow20_lap2` | **저속 행 (최중요)** — 회피 하한속도 2.0이 실제로 쓰는 값 |
| 2 | `max_speed:=2.5 min_speed:=2.0` | 2랩 | `slow25_lap1`, `slow25_lap2` | 저·중속 행 |
| 3 | 평소 레이스 설정 (인자 없음) | 2랩 | `fast_lap1`, `fast_lap2` | 고속 셀 (v ≥ 3.0) |

- 정지·복구가 있었던 랩은 폐기하고 그 prefix로 재주행.
- 시간·배터리가 남으면 `max_speed:=1.6`·`2.9` 각 1랩 추가(셀 커버리지 향상, 필수 아님).
- ⚠️ 속도를 바꿀 때마다 **control_real 재시작** (파라미터는 시작 시 1회만 읽음).

## 1. 랩탑 준비 (모든 터미널 공통)

```bash
# HY_MIRU(젯슨 핫스팟) 접속 — 인터넷 안 되는 게 정상
export ROS_DOMAIN_ID=70
export ROS_STATIC_PEERS=10.1.1.1      # 멀티캐스트 우회 (핵심)
sudo iw dev wlo1 set power_save off    # wifi 절전이 다운링크 드랍의 단골 원인
ros2 daemon stop
```

통신 자가진단 (스택이 뜬 뒤):
```bash
bash ~/2026_IFAC/src/f1tenth_control/tools/f1net_client.sh
```
🔴 판독 주의: [4]의 수신 IP 목록에 **젯슨(10.1.1.1)이 있어야** 정상.
자기 IP(10.1.1.x)와 127.0.0.1만 있으면 실패다 — 스크립트의 🟢 문구를 믿지 말 것.

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

## 3. 주행 전 배선 스모크 (1분, 생략 금지)

차를 **세워둔 채** 랩탑에서:
```bash
bash ~/2026_IFAC/tools/lut_lap_runner.sh smoke_test
```
- runner가 odom 수신을 먼저 검증한다 — 미수신이면 시작을 거부하고 진단 안내를 띄움
- odom이 잡히면 referee가 뜨고, 차가 서 있으므로 **8초 뒤 no_start로 자기 종료 +
  `~/lut_traces/smoke_test_*.json/csv` 생성** ← 이 파일이 생겨야 전 구간 배선 증명
- 파일이 안 생기면 주행을 시작하지 말 것

## 4. 랩 절차 (랩마다 반복)

```
① (선택) f1rec <prefix>          # bag 보험 + MCL 공분산 분석용 — A 누르기 전 시작
② bash ~/2026_IFAC/tools/lut_lap_runner.sh slow20_lap1
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
