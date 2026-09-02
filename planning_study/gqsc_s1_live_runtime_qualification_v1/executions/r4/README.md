# Revision-4 execution namespace

Revision 4 restarts the complete qualification from the beginning. All 30 new scientific capture
artifacts belong below `raw/`; complete-dataset analyzer outputs belong below `results/`. Both
directories must be absent or empty at the final pre-timing gate and remain separate from R3.

Scientific raw/results are intentionally local and ignored by Git. They must not be committed or
pushed automatically.
