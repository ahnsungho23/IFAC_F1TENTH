# Diagnostic semantics audit

This audit distinguishes exact validator gates from ranking-only or descriptive measurements. The implementation authority is the current frozen baseline source, not the names of CSV columns.

## Evaluation call path

`validateP3ShadowPath()` expands the supplied visible obstacles, runs `measureCandidate()`, then calls the exact `validateCandidate()` with the supplied collision horizon and obstacle-reserve scale. It reports the hard verdict and the separately measured diagnostics ([source](../../src/local_planning/src/raceline_spline_planner.cpp#L1183)).

## Diagnostic meanings

Let waypoint `i` have forward station `q_i`, lateral offset `d_i`, speed `v_i`, curvature `kappa_i`, and reference widths `L_i=d_left`, `R_i=d_right`. Let `r_i=trackBoundaryReserve(v_i,kappa_i)` and `D=maximum_target_offset_m`.

| Field | Source-grounded meaning | Hard gate? | Contract treatment |
|---|---|---:|---|
| `hard_valid` | Exact conjunction of entry continuity, minimum points, ordered reference progression, center track bounds, rotated rectangular footprint bounds, horizon-limited inflated-obstacle collision, lateral slope, curvature rate, and directional curvature checks | Yes | `HARD_VALID_P3` authority |
| `footprint_track_margin_m` | Minimum clearance obtained by projecting all four corners of the yawed rectangle (`vehicle_length_m`, `vehicle_half_width_m`) to nearby reference segments, with track-boundary reserve applied once ([source](../../src/local_planning/src/raceline_spline_planner.cpp#L2246)) | Yes, negative is rejected | No second threshold |
| `center_track_margin_m` | Minimum center-point room to `d_left/d_right` after `trackBoundaryReserve()` ([source](../../src/local_planning/src/raceline_spline_planner.cpp#L2112)) | Center bound is hard; this scalar is not the rotated footprint | No extra threshold |
| `obstacle_margin_m` | Minimum signed lateral distance to every expanded visible obstacle while the waypoint's forward station overlaps its full `[start,end]`, using the default measurement clearance ([source](../../src/local_planning/src/raceline_spline_planner.cpp#L2139)) | No | Audit/ranking only |
| hard obstacle collision | Collision with an inflated visible obstacle only when the waypoint is also within the evaluator's optional maximum collision-forward horizon ([source](../../src/local_planning/src/raceline_spline_planner.cpp#L3031)) | Yes | Already inside `hard_valid` |
| `lateral_slope_margin` | `maximum_lateral_slope - observed peak |Delta d / Delta s|` | Yes | No second threshold |
| signed curvature margin | Direction-specific steering-curvature limit minus absolute waypoint curvature | Yes | No second threshold |
| curvature-rate margin | `maximum_curvature_rate_radpm2 - max |Delta kappa / Delta s|` | Yes | No second threshold |
| `braking_deficit_m` | Maximum positive shortage between available forward distance and `v_ego * response_delay + (v_ego^2-v_wp^2)/(2a_decel)` ([source](../../src/local_planning/src/raceline_spline_planner.cpp#L2170)) | No; production ranks zero before positive deficits | Must be `<= 1e-9` for USABLE |
| `exit_reaches_next_obstacle` | Candidate exit transition enters a later clearance-expanded obstacle interval before merge; it is the first production ranking key ([rank source](../../src/local_planning/include/local_planning/candidate_rank.hpp#L49)) | No; fallback is preserved | Must be false for USABLE |
| `velocity_loss` | Mean nonnegative fractional speed loss against the reference speed | No | Report only; no frozen safety cutoff |
| `global_path_deviation_m` | Mean absolute Frenet offset | No | Report only; no frozen safety cutoff |
| `minimum_normalized_safety_slack` | Minimum of body-wall, obstacle, directional-curvature, and curvature-rate normalized slacks ([source](../../src/local_planning/src/raceline_spline_planner.cpp#L2203)) | No | Report only |

The scalar definitions reconstructed from source are:

- Center-track margin: `min_i min(L_i-r_i-d_i, d_i+R_i-r_i)`.
- Body-wall ranking room: replace `r_i` above by `r_i + vehicle_half_width_m + avoidanceTrackingErrorReserve(v_i,kappa_i)`.
- Obstacle margin for each longitudinally overlapping expanded obstacle `o`: with `a_o=d_right,o-clearance_i` and `b_o=d_left,o+clearance_i`, the signed gap is `a_o-d_i` if `d_i<=a_o`, `d_i-b_o` if `d_i>=b_o`, and `-min(d_i-a_o,b_o-d_i)` otherwise; take the minimum over samples/obstacles.
- Normalized slack: `min(body_wall_room/D, obstacle_margin/D, (kappa_limit-|kappa|)/kappa_limit, (kappa_rate_limit-peak_rate)/kappa_rate_limit)`.
- Lateral slope: `max_i |d_i-d_(i-1)|/(q_i-q_(i-1))`.
- Curvature rate: `max_i |kappa_i-kappa_(i-1)|/(q_i-q_(i-1))`.
- Velocity loss: mean of `max(0,v_ref,i-v_i)/v_ref,i` over samples with positive reference speed.
- Global deviation: mean of `|d_i|`.
- Braking deficit: `max_i [v_ego*t_response + (v_ego^2-v_i^2)/(2*a_decel(v_ego,v_i)) - q_i]_+`, evaluated only where `v_ego>v_i`.
- Exit conflict is true iff a later obstacle begins after the current cluster end and before the merge station, and at least one path sample in the overlap of that obstacle and the exit interval satisfies `d_right,o-clearance-eps < d_i < d_left,o+clearance+eps` ([source](../../src/local_planning/src/p3_shadow.cpp#L1514)).
- Footprint margin is the minimum, over the four yaw-rotated rectangle corners and nearby reference segments, of the signed physical-track-bound clearance after applying the track-boundary reserve once. It is not reducible to center margin minus a heading-independent half-width ([source](../../src/local_planning/src/raceline_spline_planner.cpp#L2269)).

## Why negative obstacle/slack is not an automatic usability failure

The measured obstacle margin considers the full longitudinal extent of every expanded visible obstacle. The hard collision test can be explicitly bounded by the maneuver collision horizon and can use an evaluator-specific reserve scale. Consequently, a path can be exact hard-valid while its descriptive obstacle margin or combined slack is negative. Treating those diagnostics as an additional hard or usable gate would silently create a new validator with different semantics.

Likewise, body-wall slack includes vehicle body plus tracking-error reserve for ranking comparability, while the exact rotated footprint is separately hard-validated. The source explicitly states that this ranking quantity never rejects a candidate ([source](../../src/local_planning/src/raceline_spline_planner.cpp#L2126)).

## Offline suitability

All fields used by the frozen contract are deterministic functions of the exact evaluator snapshot, reconstructed path, frozen parameters, and frozen C++ code. `exit_reaches_next_obstacle` and braking deficit are physically interpretable existing production ranking diagnostics. No new learned threshold or outcome-fitted cutoff is introduced.

Waypoint-0 center/footprint margins are samples of the corresponding path-wide quantities, not new gates. Minimum/maximum commanded speed, confirmed critical speed, and speed-hold stations describe the already-frozen speed shaping. Point count, path digest, source branch, root/probe lineage, and generation index are construction/provenance fields rather than safety metrics. `all_observed_violation_flags` re-evaluates existing validator predicates for audit visibility; it does not add a rejection rule.
