# Revision 3 operational repair

Revision 3 is a pre-result operational repair only. It does not change the workload, schedule,
input, binary, affinity, parity, timing, validity, retry, quantile, threshold, contrast, or final
classification contracts frozen by revision 2.

## Provenance

- Revision 0: W1 ready; W2/W3 blocked.
- Revision 1: same-input W1/W2/W3 protocol established.
- Binary gate: Release-equivalent W1 and W2/W3 binaries frozen.
- Revision 2: decision rules, quantiles, validity rules, and actual-affinity evidence frozen.
- Revision-2 execution attempt: `PRE_RUN_GATE_FAILED_BEFORE_TIMING`; W1 runner Git mode was
  `100644`, direct invocation returned `permission denied`, scheduled runs started were zero, and
  scientific timing samples were zero.
- Revision 3: W1 Git executable mode repaired to `100755`, frozen-tree/filesystem executable
  preflight added, and the distinct `executions/r3/` result namespace frozen before timing.

The immutable revision-2 annotated tag object is
`4542c0ab71c4bd94be3ed69e0d5f269e535c4c7d`; its target is
`1ee613c74b28ad08cfb92f8e46b25aa947837aee`.

The R2 untimed parity artifact is preserved under
`validation/pre_run_failures/r2_w1_executable_mode/` with SHA-256
`7255c3998a9287bb1adbdfbc676f5f960654e5fd1b15bd999eb2f75f4d5dd97a`.

## Mandatory pre-timing operation

After the revision-3 commit has been pushed and annotated tag
`gqsc_s1_live_runtime_qualification_v1-r3` has been created and pushed, run:

```zsh
planning_study/gqsc_s1_live_runtime_qualification_v1/tools/preflight_executable_modes.zsh \
  gqsc_s1_live_runtime_qualification_v1-r3
```

The preflight checks current filesystem executability and the frozen Git-tree mode for every shell
script the execution protocol invokes directly. The Python analyzer remains interpreter-invoked
and scientifically unchanged.
