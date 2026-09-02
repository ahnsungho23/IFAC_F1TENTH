# Revision 4 PID-evidence operational repair

Revision 4 changes only process-identity evidence and execution provenance. The scientific
workload, input, binaries, planner invocation, callback counts, schedule, affinity policy, parity,
metrics, retry rules, statistics, thresholds, decisions, and classification precedence remain
identical to revision 2 and revision 3.

## Root cause and R3 disposition

In the R3 W2 runner, `find_child_executable` declared `local candidate` inside its polling loop.
With zsh `TYPESET_SILENT` disabled, redeclaring the already-local name emitted `candidate=''` on
stdout on every polling iteration after the first. The caller used command substitution, so these
declaration diagnostics and the later numeric PID were captured together as one non-PID value.
`record_pid_evidence` therefore could not access one valid `/proc/<PID>` path. W3's
`find_group_executable` used the same vulnerable pattern, although R3 stopped before W3 timing.

R3 remains `EXPERIMENT_INCONCLUSIVE`: W1 completed 10 runs/2,000 measured callbacks; W2 R1-A
`ATTEMPT0` and `RERUN1` failed before replay; W2/W3 measured counts are zero; and no partial latency
analysis was performed. `executions/r3/INCONCLUSIVE.md` records the byte-preservation hashes.

## Strict resolver contract

`tools/pid_evidence.zsh` resolves only within the owned child relation or owned process group. A
success emits exactly one decimal PID and nothing else on stdout. Diagnostics use stderr. Before a
PID can match, the resolver verifies a numeric value, a live `/proc` entry, exact frozen executable,
matching executable name in argv[0], and successful affinity query. Zero matches fail; more than one
valid match fails as ambiguous. The caller re-verifies the resolved PID before recording evidence.

W2 uses the strict owned-child resolver for both planner and replay. W3 uses the strict owned-group
resolver for planner and replay and retains complete background-group affinity checks. W1 does not
have the vulnerable local-declaration command substitution; its existing scientific runner is
unchanged.

## Non-scientific preflight

`tools/test_pid_resolver.zsh` tests numeric-only success stdout, empty success stderr, stderr-only
zero/ambiguity failures, exact executable, `/proc`, and affinity access. The external
`tools/preflight_pid_affinity.zsh` then checks W1 A/B with discarded untimed harness output, W2 A/B
without replay, and the full W3 A/B process set without replay. No 20+200 scientific capture is
started, and temporary validation output is removed.

Pre-commit live validation passed for W1 A/B, W2 A/B, and W3 A/B. W3 verified planner affinity
`0-23`/`8` and background affinity `0-7,10-23`.

## R4 operational tool identities

- W1 runner SHA-256 (unchanged):
  `e25e1d19cda2c32766d91808eec331e88eeae3bc69695738553a36403963f07f`
- W2 runner SHA-256:
  `45ed3dffcd25215f67665bc651cd1066819fa68d362a4a96a1978614185d2613`
- W3 runner SHA-256:
  `94c41eb3ed9a73ecf02880fa585bda192295362424d770c4b49ec3b048992b92`
- PID resolver SHA-256:
  `845d648e6c40cbe78ab09ecd63d852e76ffaa3096d806e194130ea4c0c406b92`
- PID resolver self-test SHA-256:
  `1c12ce8863c4cd062bea267a15dc40ef50446300fe94caab8c655a4eafb45f3d`
- PID/affinity preflight SHA-256:
  `c9706b168cdd664c138a2f6cd214e1f7f5607120d14a3cb9b261ca277dbf0b1d`
- Executable-mode preflight SHA-256:
  `1ee16147d30a3a0831084d1c0b24c4372ae4f0d88f397f0d687878f626278ad8`
- Frozen analyzer SHA-256 (unchanged):
  `92321d50542004236de9154a9098e5e0ebc4897d3c61531d7fd4c73160e455db`
