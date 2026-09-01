# GQSC 연구 재개 요약 (2026-09-02)

이 디렉터리는 기존 권위 아티팩트를 대체하거나 큰 표를 복사하지 않는다. OS 이전의 과학적 상태, Validation v1 교정, 현재 `research` HEAD와의 차이, 다음 실험만 연결한다.

## 결론

- Validation v1의 37개 episode는 모두 `VALIDATION_SEEN_AFTER_V1`이다. 새 모델 선택에는 사용할 수 없다.
- 구 Oracle의 hard-feasible 분모 `24`는 Oracle v2에서 `25`로 교정됐다. false negative는 전체 seen 범위에서 `V2E09`, validation에서 `VUE036` 두 건이며, 확인된 false positive는 없다.
- 최신 계약에서 validation의 Oracle-v2 hard/usable 분모는 각각 `25/19`이다. H3은 `14/25 hard`, `10/19 usable`, H4-A는 `14/25 hard`, `11/19 usable`이다.
- H4-A의 교정 후 hard 미회복 11건은 probe/root 또는 `d_mid` 선택 5건, multi-parameter/factor-space coverage 4건, candidate-budget truncation 2건이다.
- 진단 이후 R3와 frozen GQSC-S1이 여러 coverage miss를 이미 해결했다. 현재 핵심 S1/P3 파일은 마지막 pre-migration 과학 체크포인트와 Git blob이 동일하다. 남은 최우선 질문은 알고리즘 변경이 아니라 frozen S1의 live fresh-generation 지연이 CPU 배치/부하에 의해 설명되는지이다.

## 문서

- [research_state.md](research_state.md): 증거·계보·정의·교정 결론
- [validation_v1_corrected_accounting.csv](validation_v1_corrected_accounting.csv): 원 보고치와 최신 분모의 최소 회계표
- [unresolved_cases.csv](unresolved_cases.csv): H4-A hard 미회복 11건의 최신 상태
- [next_experiment_plan.md](next_experiment_plan.md): 최대 3개, 우선순위가 있는 비침습 실험 계획

## 안전 경계

외부 연구 데이터의 금지 분할, 그 데이터, 매니페스트, 스크립트 및 결과 내용은 열기·해시·파싱·실행하지 않았다. 다만 작업 중 한 차례의 광범위한 저장소 파일명 조회가 금지 분할과 연관된 결과 디렉터리의 경로명들을 반환했다. 내용 접근은 없었지만, 이 때문에 엄격한 “관련 경로명조차 열거하지 않음” 준수 증명은 하지 않는다. 이후 모든 조회는 허용된 파일을 정확히 지정했다.

소스 코드, 알고리즘, Git ref, 데이터셋, label은 변경하지 않았고 commit/push하지 않았다.
