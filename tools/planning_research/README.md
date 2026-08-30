# Local planning research tools

이 디렉터리는 production 결정을 바꾸지 않는 계측 준비와 read-only rosbag 감사를
담습니다. 어떤 도구도 원본 bag을 repair/convert/delete/move하지 않습니다.

## 1. Planner instrumentation

`prepare_instrumented_run.py`는 현재 Git HEAD/dirty status, local-planner source/config SHA-256
manifest, 연구 파라미터 overlay를 새 출력 디렉터리에 만듭니다. 일반 운영 YAML을 먼저
적용하고 overlay를 두 번째로 적용해야 합니다. 런타임 계측은 기본 OFF이며, ON에서는
callback이 bounded queue에 `try_lock` 제출만 합니다. 큐가 잠겼거나 가득 차면 이벤트를
버리고 `dropped_log_count`를 다음 `PLANNING_EVENT`에 남깁니다. 종료 시 writer thread가
queue를 비우고 JSONL을 flush합니다. 기존 run 디렉터리는 덮어쓰지 않습니다.

출력:

- `metadata.json`: schema/provenance/effective parameter snapshot
- `planning_events.jsonl`: callback/lifecycle/global actual counters/runtime
- `candidate_events.jsonl`: 버려진 M0 side를 포함한 모든 실제 구성 후보와 M1 probe/root
- `run_summary.json`: clean shutdown과 최종 accepted/written/dropped cycle 수

`research_all_violation_audit_enable=true`는 production first-failure validator 뒤에 별도
non-authoritative pass를 실행합니다. 기본은 false이며 그 pass의 플래그는 선택/순위/발행에
사용되지 않습니다.

## 2. 137-bag inventory audit

```bash
python3 tools/planning_research/rosbag_inventory_audit.py \
  --bag-root /home/sungho/.local/share/Trash/files \
  --inventory /home/sungho/rosbag_inventory.csv \
  --output-dir planning_study/research_data_audit
```

기본 실행은 metadata와 metadata가 명시한 storage file size까지만 읽습니다. 정확한 중복
확인이 필요할 때에만 `--content-hash-candidates`를 추가하며, 이 경우에도 구조/크기/topic
signature가 같은 후보 그룹의 storage file만 순차적으로 읽습니다. 자동 삭제 판단은 하지
않습니다.

## 3. Localization feature extraction

ROS 2 Jazzy와 이 workspace overlay를 source한 뒤 한 bag을 대상으로 실행합니다.

```bash
python3 tools/planning_research/rosbag_localization_audit/extract_localization_features.py \
  /read/only/bag --output /separate/output/features.csv
```

모든 행의 label은 `LOC_UNKNOWN`입니다. nearest-message offset과 연속성 값은 feature일 뿐
quality threshold가 아닙니다. 출력 경로가 bag 내부이면 거부하고, 기존 출력은 `--force`
없이는 덮어쓰지 않습니다.
