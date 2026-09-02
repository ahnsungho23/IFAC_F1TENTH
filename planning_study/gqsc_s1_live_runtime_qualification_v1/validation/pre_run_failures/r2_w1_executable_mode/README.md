# R2 W1 executable-mode pre-run failure

The revision-2 execution attempt stopped before scientific timing. The frozen Git tree stored
`tools/run_w1_qualification.zsh` with mode `100644`, so the protocol's required direct invocation
failed with `permission denied`.

No scheduled run started. No warm-up or measurement callback was collected, and no latency
statistic was observed. `R1_PARITY_BEFORE.txt` is untimed preflight/parity evidence only and must
not be treated as, or mixed with, the revision-3 scientific raw dataset.

Artifact SHA-256:

`7255c3998a9287bb1adbdfbc676f5f960654e5fd1b15bd999eb2f75f4d5dd97a`
