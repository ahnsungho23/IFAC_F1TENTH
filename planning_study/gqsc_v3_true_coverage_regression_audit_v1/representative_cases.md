# Representative cases

이 문서는 frozen GQSC를 변경하지 않은 exact reconstruction audit의 대표 사례다. `stage C`는
두 operator가 모두 존재해 frozen pairing order에 들어갈 수 있음을 뜻하고, 구현이 실제로
materialize한 bounded pool은 `stage D`다.

## Selection-only — `passing_mixed_f1`

- exact-valid current-operator P3: digest `eec06006cc9c4f59`
- pair는 B128 안에 있었지만 ordered Top-12 밖이었다.
- 따라서 lateral/transition operator나 P3 family를 바꾸지 않아도 selector가 놓친 사례다.

## Selection-only property — `RP2_3`

- ego `(s,d,v)=(0.28127823672934049, 0.57675532978881394, 3)`
- witness `(d_target,d_mid,entry,exit)=(0.53690764785339706, 0.53690764785339706, 0.51458109301505117, 0.49716841625749453)`
- B128에는 exact-valid pair가 32개 있었지만 Top-12는 전부 hard-invalid였다.

## B128 truncation — `RP2_22`

- operator lateral과 transition은 모두 존재하며 full cross audit에서 hard-valid P3를 3개 찾았다.
- 그 exact pair들은 B128 어느 행에도 없었다. 이는 Top-12보다 앞선 bounded pair coverage miss다.

## Lateral operator — `RP3_90`

- legacy exact-valid witness digest `9c6f83969ca3a510`.
- witness transition은 direct transition bank에 있었지만 `(d_target,d_mid)=(0.64960380242450455, 0.64960380242450455)` lateral은 없었다.

## Transition operator — `layoutB_failing_f1`

- current lateral과 legacy bisected entry `0.86298565063591948`를 결합하면 exact validator를 통과한다.
- current direct transition bank만 쓴 full cross에는 valid가 0개였다. frame 0/2와 pinch 두 frame도 같은 격리 결과다.

## Parameter/interface mismatch — `raceline_tight_gap`

- test contract는 `vehicle_half_width=0.12`, `safety_margin=0.03`이다.
- `frozenR3Visible()`/`frozenR3CorridorSample()`은 method 내부에 `0.15`, `0.08`, wall `0.04`, lookahead `15.0`을 고정해 geometry를 만들며, 이 입력에서는 side geometry가 비어 direct transition set도 비었다.
- legacy는 같은 test parameter와 validator로 8개 hard-valid P3를 만들었다. 이는 operator tuning이 아니라 parameter/input contract mismatch다.

## Corridor-only comparator — `RP2_17`

- conservative 1D reachability oracle는 corridor를 찾았지만 legacy exact P3와 current full operator cross 모두 hard-valid witness를 찾지 못했다.
- 이 audit은 이를 P3 representation limit으로 승격하지 않고 `OTHER/CORRIDOR_ORACLE_ONLY_NO_EXACT_P3_WITNESS`로 남긴다.

## Nonblocking semantics — `RP2_2`

- legacy result는 `no static obstacle blocks the global race line`이며 회피 후보가 필요 없다.
- current GQSC wrapper는 early return의 reason을 `GQSC_V3_FROZEN_ENTRY`로 남겨 property predicate가 miss로 센다. 101/200이 이 유형이다.
