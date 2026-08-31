# DEVELOPMENT geometry deficiency

## Access boundary

This phase read only the 9 `PILOT_SEEN_DEVELOPMENT_DATA` and 40 `DEVELOPMENT` summaries,
lineage rows, and event inputs listed in `dataset_access_log.csv`. It did not address a
validation-seen, success-control, or final-holdout path.

## Clean standalone v1 result

- DEVELOPMENT events: 49
- events with at least one teacher hard candidate: 23
- events with at least one teacher usable candidate: 18
- events containing any teacher-hard lateral absent from the standalone lateral bank:
  `DVE020;DVE039;V2E08`
- events for which every teacher-hard lateral is absent:
  `DVE039`
- events containing any teacher-usable lateral absent from the bank:
  `DVE020;DVE039;V2E08`
- events for which every teacher-usable lateral is absent:
  `DVE039`
- missing teacher-hard candidate rows with a containing connected component: 6

The exact missing rows and their side-relative `(u_target,u_mid)` coordinates are in
`development_missing_laterals.csv`. This table is diagnostic only; no normalized coefficient has
yet been selected from those coordinates.

The three events occupy different regions:

- V2E08: target at the near boundary `u_target=0`, with `u_mid=-0.083682` outside the component;
  another clean-bank hard/usable lateral already exists for the event.
- DVE020: outer-region rows at `u_target=0.697693 or 0.949615` and
  `u_mid=0.798462 or 0.949615`; other clean-bank hard/usable laterals already exist.
- DVE039: `u_target=0.551861`, `u_mid=0.162903`; this is the only event whose entire teacher
  hard/usable lateral set is absent.

The side-geometry cross-check makes the lack of a shared component rule more explicit. For DVE039,
the RIGHT component is `[-0.822500,-0.436854]`, and its recorded bottleneck center
`-0.629677` is exactly the side-relative component midpoint (`u=0.5`). The missing target
`-0.649677` is the old v2 target 0.02 m farther from zero than that midpoint, while its mid
`-0.499677` is 0.15 m inward from that target. This is numerical reconstruction evidence, not a
geometric derivation. DVE020 also has a midpoint bottleneck, but its missing targets occupy
`u=0.697693` and `u=0.949615`, not the DVE039 region; moreover DVE020 is already recovered by other
clean-bank laterals. V2E08 is a one-sided component-boundary case (`u_target=0`) whose mid lies
slightly outside the component, and it too already has another clean recovery.

The frozen standalone geometry trace exposes component endpoints, center/bottleneck values,
entry/exit ranges, reference spacing, and obstacle-cluster stations. It does not expose a scalar
reference-curvature descriptor for this comparison, so this audit does not label any missing row as
`CURVED_CORRIDOR` or infer curvature causality. No repeated bottleneck, center, boundary, or
inward/outward pattern across independent all-missing DEVELOPMENT events is established.

There is therefore no repeated normalized missing-lateral mechanism that justifies a new direct
operator. In accordance with the predeclared gate, v3 adds no lateral anchor for DVE039.

## MIN_OUTWARD audit

The existing DEVELOPMENT-only ablation gives:

- clean v1: hard 24/49, DVE023=0;
- equal+MIN outward: hard 25/49,
  DVE023=1;
- equal+MAX outward: hard 24/49,
  DVE023=0.

Usable coverage remains 21/49 for all three. The sole hard-event
distinction between MIN and MAX is DVE023. `MIN_OUTWARD` is therefore classified as case **B: an
effectively event-specific DVE023 repair**, not retained as `DEVELOPMENT_TUNED_GENERAL` in v3.

## Structural decision gate

An added lateral operator is justified only if the missing rows show a common normalized region
in more than one independent DEVELOPMENT event. If the all-hard/usable lateral deficiency is
unique to one event, v3 must not add an operator to recreate it. The clean-bank operator inventory
contains 22 distinct named operators; this phase did not alter it.
