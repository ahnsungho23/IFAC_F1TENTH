# GQSC research parking checkpoint — 2026-09-02

The GQSC research session is parked at the validated R4 SCE018 runtime checkpoint before A1
competition work. This checkpoint adds documentation only: no experiment, validation rerun,
algorithm change, parameter change, dataset-label change, or holdout evaluation was performed.

- [Current checkpoint](current_checkpoint.md): authoritative infrastructure, scientific, method,
  runtime, evidence, and limitation state.
- [Resume after A1](resume_after_a1.md): the mandatory restart point and ordered roadmap.

The frozen pre-result protocol remains tagged
`gqsc_s1_live_runtime_qualification_v1-r4` at
`7950527d531fec50c76baa5be3142d781b980d1f`. The distinct post-result parking tag
`gqsc_s1_live_runtime_qualification_v1-r4-result` identifies the documentation commit containing
this checkpoint.

Scientific raw/results are not stored in the main repository. They are archived on the `main`
branch of `git@github.com:ahnsungho23/IFAC_RESEARCH_BACKUP.git` in commit
`0a45c343464ed49fa519d061bfce827cc5511d24` with a deterministic SHA-256 inventory. The local
ignored R3/R4 artifacts remain as an additional copy.

Do not restart any item marked **CLOSED FOR NOW** in `current_checkpoint.md` after A1 unless new
evidence directly invalidates it.
