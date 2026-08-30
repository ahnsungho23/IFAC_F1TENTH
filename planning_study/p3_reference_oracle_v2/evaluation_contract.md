# HARD_VALID and USABLE_VALID evaluation contract

Status: **FROZEN BEFORE SEEN-DATA RECOMPUTATION**  
Machine-readable authority: `evaluation_contract.json`

## HARD_VALID_P3

`HARD_VALID_P3 := (hard_valid == 1)` with an empty first-failure reason.

This is exactly the frozen production hard validator. It does not add an offline threshold and does not weaken any production gate.

## USABLE_VALID_P3

`USABLE_VALID_P3 := HARD_VALID_P3`

`AND (exit_reaches_next_obstacle == 0)`

`AND (braking_deficit_m <= 1e-9)`.

The two added conditions come from existing production ranking semantics:

- A transition that reaches the next obstacle is inconsistent with completing the current maneuver before the next obstacle.
- A positive braking deficit means the remaining distance is insufficient for the configured response delay and speed-dependent deceleration to the candidate speed profile. `1e-9` is the already-frozen candidate-ranking epsilon.

## Deliberately not thresholded

No additional threshold is applied to obstacle margin, normalized slack, velocity loss, center-track margin, footprint margin, slope margin, curvature margin, curvature-rate margin, or global deviation.

Footprint, slope, curvature, and curvature-rate are already exact hard gates. Center-track does not replace the rotated rectangular footprint. Obstacle margin and normalized slack have measurement scope/reserve semantics different from the maneuver-bounded hard collision test. Velocity loss and global deviation are quality objectives without a frozen physical accept/reject threshold.

Therefore:

`USABLE_VALID_P3 => HARD_VALID_P3`, but a negative descriptive obstacle margin or normalized slack alone does not imply unusability.

The contract changes neither production validator nor ranker. It is an offline research evaluation label only.

