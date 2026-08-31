# DVE039 lateral-factor analysis

The v1 standalone side geometry is RIGHT domain `[-0.8225, -0.4368537982807025]`, with the
single connected-component midpoint/corridor bottleneck
`c=-0.6296768991403512`. The teacher hard factor is

`d_target=-0.6496768991403512`, `d_mid=-0.4996768991403512`,
`entry=1.310934367577036`, `exit=3.1286619468037347`.

Thus its lateral hypothesis is exactly generated without an event literal by the side-relative
component operator

`direction_far = sign(far-near)`

`d_target = clip_D(c + 0.02 direction_far)`

`d_mid = clip(d_target - 0.15 direction_far, -1.5, 1.5)`.

The constants 0.02 and 0.15 are pre-existing bounded-bank offsets; the new information is their
composition with the component midpoint and side-relative far/inward directions. This replaces,
rather than adds to, `COMPONENT_HALF_MID_MINUS_015`, so lateral <=64 is unchanged. On DEVELOPMENT,
the replaced operator constructed 7 paths and supplied
3 hard / 3 usable candidates,
while being the sole hard/usable operator for 0/0
events. The replacement ablation found no lost hard or usable DEVELOPMENT event.

The lateral alone is insufficient under B128: its default component ordering exposes only the first
two transition waves. Giving this maneuver-shape operator its general long-entry priorities places
`LONG_ENTRY_MEDIUM_EXIT` in B128. The direct transition reconstructs exit
`3.9473573093269398` (not the teacher's `3.1286619468037347`) and is exact-validator hard+usable;
therefore the recovery is not literal teacher-candidate copying. The operator also recovers V2E05
on DEVELOPMENT, supporting a geometry mechanism rather than a DVE039-only patch.

This is still a seen-only static finding, not a closed-loop robustness result.
