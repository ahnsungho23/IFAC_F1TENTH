# SCE018 ROS replay/collector

## 목적

이 패키지는 canonical `SCE018.event`를 production ROS 메시지로 변환해 실제
`local_planner_node`의 공개 입력에 발행한다. Planner 내부 클래스나 private API를 호출하지
않는다. `/local_planning/live_runtime_profile`과 `/local_planning/p3_shadow`를
`callback_sequence`로 결합하고, 동일 geometry/digest/count 계약을 자동 검증한다.

## Fresh 전이

`TEST_ACTIVE`가 committed suffix를 유지하면 evaluator를 호출하지 않는다. 드라이버는
동일 obstacle을 먼저 증가한 stamp로 발행한 뒤 1초 낮은 stamp로 발행한다. Production
`acceptObstacles()`는 0.5초 이상의 regression을 source restart로 처리하여 source epoch와
lifecycle을 초기화한다. Regression 메시지는 폐기되지 않으며 geometry는 SCE018과 같다.

이 전이는 bag loop/source restart를 위한 기존 production 복구 경로다. P3 mode, parameter,
guard, reference, ego scalar, obstacle scalar를 바꾸지 않는다.

## 빌드

상위 `build_tools.zsh`를 실행한다. 빌드·install·log는 이 연구 디렉터리의 ignored 경로에만
생성된다.

Planner는 별도 `release_overlay/_install`을 tools overlay 다음에 source한다. Smoke와
qualification runner는 `ros2 pkg prefix local_planning`과 frozen installed-node SHA-256을
검사하므로 normal workspace의 non-Release node로 조용히 되돌아갈 수 없다.

## 출력 모드

- `smoke_mode:=true`: callback identity, digest, count, outcome과 timing 필드 존재 여부만
  기록한다. 모든 시간 값과 raw JSON은 폐기한다.
- `smoke_mode:=false`: 미래 qualification 전용이다. Joined row에 `O_total`을 기록하고 raw
  profile/diagnostic JSON도 보존한다. 이번 unblock 작업에서는 실행하지 않는다.

Qualification runner는 `target_callbacks:=220`, `warmup_callbacks:=20`을 고정한다. 출력은
처음 20개를 `phase=WARMUP`, 이어지는 200개를 `phase=MEASUREMENT`로 표시하며 기존 경로를
덮어쓰지 않는다. `run_w1_qualification.zsh`, `run_w2_qualification.zsh`,
`run_w3_qualification.zsh`는 protocol revision 2에서도 실행하지 않은 미래 데이터 수집
명령이다. 세 runner 모두 timing 입력을 시작하기 전에 실제 planner/harness PID와 affinity를
`affinity.txt`에 기록한다. W3는 experiment-owned background process group도
`0-7,10-23`인지 검증한다. 모든 runner는 AC 전원과 `powersave` governor를
`system_context.txt`에 남긴다.

결과 분석은 timing 전에 동결된 `../analyze_qualification.py`만 사용한다. W1의
`callback_wall_us`와 W2/W3의 `o_total_us`를 raw에서 직접 읽고 nearest-rank p90/p95/p99,
4/5 repeat gate, 10% strong effect, production decision, 최종 precedence를 적용한다. W1과
W2/W3 timing 범위는 동일하지 않으므로 W1→W2 차이를 순수 ROS overhead로 해석하지 않는다.

`evaluation_sequence`는 현재 public diagnostic에 노출되지 않는다. 한 qualifying callback에
`fresh_evaluation_count==1`을 강제한 뒤, joined callback의 증가 순서로
`derived_evaluation_sequence`를 생성한다. Node 내부 연구 sequence라고 가장하지 않는다.
