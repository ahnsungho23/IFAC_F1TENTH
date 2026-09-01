# Frozen SHA verification

## Verified values

- Canonical S1 serialization:
  `planning_study/gqsc_s1_closed_loop_candidate_v1/gqsc_s1_method.json`
- SHA-256:
  `670f39a23479bcdcc1db0829a895257443fec8ee2f78f186bc1beb224090b776`
- Expected frozen SHA-256:
  `670f39a23479bcdcc1db0829a895257443fec8ee2f78f186bc1beb224090b776`
- Reference v3 SHA-256:
  `965f6ce65b7ce5b1c6426a0975c1dfafe779e89eb20308bb09a11a1b4a22e780`
- Pre-smoke tag target:
  `534e5e63fdc19146c24887f697ed6b9f869d2347`

## Source-diff contract

The frozen method serialization, S1/v3 frozen-contract headers, GQSC proposal/selection
implementation, P3 analytic construction, validator, ranker, vehicle parameters, lifecycle, and
controller source were not edited. The working source diff is limited to default-OFF runtime
measurement plumbing, its harness reporting mode, parameter declarations set to false, and
research documentation.

`git diff --check` passes. No commit, tag, or push was created in this task.

FINAL_HOLDOUT was not inspected or executed.
