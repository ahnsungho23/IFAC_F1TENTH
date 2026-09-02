# Revision-3 execution namespace

Revision 3 writes all scientific capture artifacts below `raw/` and all post-completion analysis
artifacts below `results/`. Both directories must be absent or empty at the final pre-timing gate.
They are intentionally not populated in the protocol commit and are not to be committed or pushed
automatically after execution.

The revision-2 untimed failure evidence is retained separately under
`validation/pre_run_failures/r2_w1_executable_mode/`.
