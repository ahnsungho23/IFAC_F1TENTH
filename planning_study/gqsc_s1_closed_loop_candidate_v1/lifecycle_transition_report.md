# S1 lifecycle transition report

Frozen S1 produced at least one hard-valid path in 62 of the 104 explicitly allowed seen
snapshots. Both fresh-ownership scenarios accepted the selected path for all 62 events, so the
124 `FRESH` rows are 124/124 successful.

For every one of the 62 independently selected events:

- same-input active suffix continuation remained hard-valid;
- obstacle disappearance retained the frozen ownership contract;
- forward progress trimmed only the passed prefix;
- the frozen completion boundary completed without outputting a stale suffix;
- a new full-width obstacle inside the owned interval invalidated the path and entered the
  unchanged `NO_SAFE_PATH` fallback in the synthetic test.

The guard-growth case stayed hard-valid in 62/62 events. Of those, 39 used guarded/raw
revalidation and 23 remained hard-valid without the raw fallback. The lifecycle horizon
reconciliation that was already present before S1 is unchanged; S1 supplies a different selected
candidate set but does not change lifecycle ownership, fallback, or safe-stop semantics.

The detailed rows are in `sequential_replay.csv`. This is deterministic snapshot sequencing, not a
large closed-loop benchmark.
