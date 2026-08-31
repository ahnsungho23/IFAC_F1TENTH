# Frozen R3-K12 rank and factor structure

## 1. Discrete factor space

For side `q in {RIGHT, LEFT}`, let `P_q` be the production factors on that side. The frozen
transition catalog is the exact unique set

`A = unique_sorted({(entry_scale, exit_scale) : p in P_RIGHT union P_LEFT})`.

The target catalog contains exact-bit unique, domain-clamped values from production targets and
their `+/-{0.01,0.02,0.04,0.06,0.08,0.10} m` offsets, 16 near-to-far fractions of each corridor
component, and every sampled corridor center. For every target `t`, the lateral catalog adds

- `m=t`,
- `m=t+alpha(c-t)` for each center `c` and
  `alpha in {0.125,0.25,0.375,0.4375,0.5,0.75,1}`,
- `m=beta*t` for `beta in {0.25,0.5,0.75}`,
- `m=t+/-delta` for
  `delta in {0.01,0.02,0.04,0.05,0.08,0.10,0.15,0.20} m`, and
- the production `(t,m)` pairs.

Middle values are clamped to `[-1.5,1.5] m`, exact duplicate `(t,m)` pairs retain the minimum
`(source_priority,target_source,mid_source)`, and the lateral rows receive the frozen source-based
order. If `L_q` is the resulting lateral count and `M=|A|`, then

`N_raw = M * sum_q L_q`,

`F = union_q ((L_q x A) minus exact production configurations)`.

The largest seen event, VUE019, has `sum L_q=4,471`, `M=16`, `N_raw=71,536`, 30 excluded
production configurations, and `|F|=71,506`. The center-conditioned target/middle expansion
followed by the transition Cartesian product is the source of this tail.

## 2. Pair-dependent geometry

For factor `f=(q,t,m,e,x)`, the five stations are

`required=|t-d_ego|/0.8`,

`apex=cluster_start` and
`start=apex-apex*(11.442220427651225/15)*e` when `apex>=required`; otherwise
`apex=max(required,reference_spacing)` and `start=0`,

`s=[start,apex,(apex+cluster_end)/2,cluster_end,cluster_end+6.178529850015357*x*mu_q]`,

where `mu_q=0.4060036444074003` on the outside side and `1` otherwise. The proxy profile is the
frozen four-segment C2 quintic-Hermite profile through offsets
`[d_ego,t,m,t,0]` at these stations.

The exact configuration key `(q,t,m,e,x)` is factor-local. Stations depend on target and transition;
the sampled profile and all corridor/slope/curvature quantities depend jointly on lateral and
transition factors. Reference sample bounds, centers, later-obstacle flags, and cluster geometry are
invocation/side invariants.

## 3. Feasibility stream

Construction-guard rows are placed after non-guard rows. Every non-guard row is ordered by the
exact tuple

`R_lex(f)=(`
`exit_conflict, max_violation>1e-9, max_violation,`
`slope_excess>1e-9, slope_excess, sum_violation, curvature_proxy,`
`center_error, -minimum_clearance, shape_energy,`
`source_priority, lateral_index, transition_index, go_left, configuration_key)`.

The first 10 exact preconstruction shapes are retained. Guard rows use only the final stable tie.

## 4. Coverage stream

The base score is

`B(f)=40 Vmax + 2 Vsum + 8 S + 0.12 K + 0.25 C`
`     +2 max(0,0.02-clearance) +0.02 E +0.002 source_priority`.

Non-finite corridor rows receive `1e9`. The first coverage row minimizes
`(exit_conflict,B,stable_tie)`. For the current selected set `Q`, later rows minimize

`(exit_conflict, B-2 min(1,D(f,Q)), B, stable_tie)`,

where

`D(f,Q)=min_g in Q [0.5 I(side differs)+|t_f-t_g|/1.5+|m_f-m_g|/3`
`                     +0.2|entry_norm_f-entry_norm_g|`
`                     +0.2|exit_norm_f-exit_norm_g|]`.

Selection stops at two unique shapes, with the frozen 48-row fallback and guarded ordering intact.

## 5. Monotonicity and bounds

The generated lateral order is source-priority based, not proxy-score based. Corridor violation and
center error are piecewise absolute/max functions of the quintic profile; moving `d_mid` toward a
corridor center can decrease and then increase them. Station switching at
`cluster_start=|t-d_ego|/0.8` also makes transition effects piecewise. Therefore neither lateral nor
transition index defines a monotone score stream.

All weighted coverage terms except the diversity subtraction are non-negative, giving only the weak
lower bound `B>=0.002*source_priority`. The second coverage score can be as low as `B-2`; these
bounds do not exclude unseen factors in the frozen events. Lexicographic prefixes likewise require
the sampled profile before `exit_conflict`, violation, and slope flags are known.

Consequently a k-way merge or best-first Cartesian walk cannot be exact from component order alone.
Before v2 every configuration was fully scored because the Top-10 stream needed all lexicographic
fields and the coverage reserve needed every base score and selected-dependent distance.
