# Frozen GQSC verification

## SHA

2026-08-31 현재 다음을 다시 계산했다.

| artifact | expected | actual | result |
|---|---|---|---|
| `gqsc_v3_geometry_general_method.json` | `965f6ce65b7ce5b1c6426a0975c1dfafe779e89eb20308bb09a11a1b4a22e780` | same | PASS |
| `selected_v3_policy.json` prevalidation lock | `ea30a441695732e87d43d6863e1a31992dbfdd45749d324047495b6c96824185` | same | PASS |

sidecar는 artifact directory에서 `sha256sum -c`로 확인한다. CMake configure assertion,
`test_gqsc_v3_frozen_contract.py`, startup constant도 같은 두 SHA를 사용한다.

## frozen commit 비교

frozen candidate commit `8a8dcd93b43d4b34ec797bef23df4d8435422e59` 대비:

- canonical method JSON diff: 0
- `p3_r3_k12.cpp/.hpp` frozen selector/operator source diff: 0
- `src/local_planning/config`, controller, perception, localization diff: 0
- observed pair/reconstruction/validator max: 128/12/12
- hidden legacy seed count: 0

변경은 lifecycle-owned obstacle horizon metadata/continuation, exact same-input cache witness/counter,
integration harness와 문서에 한정된다. GQSC operators, B128, K12, 10+2 reserve, ranking, tie-break, dedup,
P3 construction, exact validator 및 vehicle/config는 수정하지 않았다.

## dataset boundary

재현 스크립트는 기존 main-integration loader의 네 seen-only directory만 명시적으로 연다:
PILOT_SEEN_DEVELOPMENT_DATA, DEVELOPMENT, VALIDATION_SEEN_AFTER_V1, SUCCESS_CONTROL_SEEN. directory
discovery fallback이나 final-holdout path가 없다. 이 작업에서 `FINAL_HOLDOUT` event/manifest/result
contents를 열거나 실행하지 않았고 large closed-loop benchmark도 실행하지 않았다.
