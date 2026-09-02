# Current GQSC research checkpoint

## Infrastructure

- Ubuntu 22.04 / ROS 2 Humble recovery: complete.
- Simulator and runtime restoration: complete.
- Map, global planning, local planning, state/controller wiring restoration: complete.
- Git-history cleanup: complete.
- Main repository remote branches: `main`, `research`; active branch: `research`.
- Research-backup remote branch: `main`.
- Research data and checkpoint provenance: restored.

These infrastructure tasks are **CLOSED FOR NOW** unless new evidence requires reopening them.

## Scientific state

- Validation v1 diagnosis: complete.
- Oracle v2 correction: complete over the declared finite seen-data domain.
- `VUE036`: old Oracle refinement miss / false negative, not P3-family infeasibility.
- `V2E09`: old Oracle search-coverage false negative; its Oracle label is corrected, while frozen
  S1 selector coverage for that case remains unresolved.
- All 37 Validation v1 cases are `VALIDATION_SEEN_AFTER_V1`; they must not select or tune a new
  method.
- No final-holdout data content was opened or evaluated in this parking task. Existing provenance
  records a prior filename-only scope incident without content access. `FINAL_HOLDOUT_UNSEEN`
  remains unevaluated and unchanged.

Primary lineage references are
`planning_study/research_resume_20260902/research_state.md`,
`planning_study/p3_validation_v1_diagnosis/`, and
`planning_study/p3_reference_oracle_v2/`.

## Current frozen method

- Lineage: frozen GQSC-S1 / R3-K12.
- Method: `LEX8_GLOBAL_DISJOINT_COVERAGE4`.
- Pair / lexicographic / coverage / reconstruction / validator limits: `128 / 8 / 4 / 12 / 12`.
- Method SHA-256: `670f39a23479bcdcc1db0829a895257443fec8ee2f78f186bc1beb224090b776`.
- Planner configuration SHA-256:
  `4fe480351a80135ff2a6e4592f661ff8a5670c032e12554d85065339d16ea960`.
- SCE018 event SHA-256:
  `572adb59ea24f3f06bed7502eb870e57a33a5416e8630106b67bc2b1d0b405bf`.
- Planning deadline / production mode: `25 ms` / `TEST_ACTIVE`.

Frozen authorities:

- `planning_study/gqsc_s1_live_runtime_qualification_v1/README.md`
- `planning_study/gqsc_s1_live_runtime_qualification_v1/workload_freeze.md`
- `planning_study/gqsc_s1_live_runtime_qualification_v1/decision_contract.md`
- `planning_study/gqsc_s1_live_runtime_qualification_v1/parity_contract.md`
- `planning_study/p3_geometry_factor_ranking_v2/selected_v2_method_spec.md`

## Runtime qualification

- R3: permanently `EXPERIMENT_INCONCLUSIVE`.
- R3 created 2,000 W1 measured callbacks, but the W2 PID-evidence operational defect invalidated
  the execution before W2 timing. No R3 scientific latency conclusion was issued, and no R3 timing
  enters the R4 dataset.
- R4: valid, 30/30 final runs, 6,000 newly collected measured callbacks, zero rejected/invalid
  measured callbacks.
- Parity and affinity/system-state contracts: pass.
- Production decision: `PRODUCTION_RUNTIME_PASS`.
- Environment classification: `RUNTIME_ENVIRONMENT_PARTIAL`.

| Cell | Pooled p95 | Repeat deadline pass | >25 ms callbacks | Decision |
|---|---:|---:|---:|---|
| W1-A | 5.236 ms | 5/5 | 0/1000 | `DEADLINE_PASS` |
| W1-B | 5.311 ms | 5/5 | 0/1000 | `DEADLINE_PASS` |
| W2-A | 13.834 ms | 5/5 | 0/1000 | `DEADLINE_PASS` |
| W2-B | 13.283 ms | 5/5 | 0/1000 | `DEADLINE_PASS` |
| W3-A | 14.916 ms | 5/5 | 0/1000 | `DEADLINE_PASS` |
| W3-B | 12.597 ms | 5/5 | 0/1000 | `DEADLINE_PASS` |

W1 and W2 timing scopes are not equivalent. Their difference is descriptive cross-layer evidence,
not a causal measurement of ROS overhead.

The strongest classification-eligible runtime evidence is W3 A->B: pooled p95 decreased by
15.55%, with improvement in 5/5 matched repeats. Both W3-A and W3-B already passed the 25 ms
deadline, so the frozen rule yields `RUNTIME_ENVIRONMENT_PARTIAL`, not
`RUNTIME_ENVIRONMENT_CONFIRMED`.

## Evidence preservation

- R3 raw aggregate:
  `3c7a9ac0fb52b0ac71b64d53edb2f3d01adafddffbbb0edc31428fcbcaebba8b`.
- R4 raw aggregate:
  `1d632f90ed0ce03b617a3bea550d3554f5a8d58f9d61d653d904205eef45b718`.
- R4 results aggregate:
  `980a7cdb9529b6e07711915705cdc35973e0abc668e291f62aa800b23abffdaf`.
- Durable backup: `git@github.com:ahnsungho23/IFAC_RESEARCH_BACKUP.git`, branch `main`, commit
  `0a45c343464ed49fa519d061bfce827cc5511d24`.
- Backup payload: 307 files, 49,536,140 bytes, verified against a per-file SHA-256 inventory.
- Frozen R4 protocol tag: `gqsc_s1_live_runtime_qualification_v1-r4`, tag object
  `94afde231b29cc3fdf18014be1be870747739774`, target
  `7950527d531fec50c76baa5be3142d781b980d1f`.

## CLOSED FOR NOW

- OS/environment recovery.
- ROS 2 Humble runtime restoration.
- Map/global/local/controller wiring restoration.
- Git-history cleanup.
- Research-data recovery.
- Validation-v1 integrity reconstruction.
- Oracle-v2 false-negative correction.
- SCE018 production runtime qualification.
- Frozen S1 Release build provenance.
- Same-input runtime qualification infrastructure.

## Not yet proven

R4 does not prove that:

- all GQSC geometries meet 25 ms;
- the worst-case geometry meets 25 ms;
- every speed meets 25 ms;
- real-car runtime meets 25 ms;
- the exact magnitude of ROS overhead is known;
- the result generalizes to another CPU or host; or
- the final holdout will perform successfully.

These limitations must accompany any future use of the R4 result.
