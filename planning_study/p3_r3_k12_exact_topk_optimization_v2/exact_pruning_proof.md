# Exact pruning and non-pruning proof

## Accepted exact reductions

### Station-basis equivalence

For one side geometry, the reference segment index and powers used by proxy evaluation are a pure
function of the exact five-station bit pattern. Replacing the old cache key
`(d_target,transition_index)` with `bits(stations[0..4])` therefore changes only how often the same
basis is constructed. The arithmetic performed for each factor is unchanged.

### Profile-metric equivalence

Define

`H(f)=bits(side,d_target,d_mid,stations[0],...,stations[4])`.

For fixed side geometry, equal `H` implies identical quintic coefficients, identical sampled profile,
construction guard, exit conflict, violation sums, slope/curvature proxies, center error, clearance,
and shape energy. The v2 implementation evaluates those fields once per `H` and bit-copies them to
other configurations. It retains each row's own entry/exit normalization because coverage distance
uses transition identity even when geometry is identical. Thus coverage greedy state and dedup
semantics are preserved.

### Lexicographic shape representative

For any exact shape class, all proxy fields are equal. Its first occurrence in the globally sorted
feasibility stream is therefore the member with the minimum stable tie. Sorting all rows and then
deduplicating is exactly equivalent to selecting the stable-tie minimum representative of every
shape, partial-sorting those representatives, and taking 10. The current generation order visits
transition indices in ascending frozen order; the stored profile representative is that minimum.

### Compact pool and invariant samples

Pool-only source/key strings do not precede any distinguishing numeric stable-tie field and were
empty before final materialization. Numeric rows uniquely identify
`(source_priority,lateral_index,transition_index,side)`. They can therefore be stored compactly and
the Python-compatible keys created only for the selected 10+2 rows. Corridor bounds, center flags,
later-obstacle flags, and station differences are copied into the exact station basis once; all
candidate floating operations keep the original order.

## Why global lazy pair pruning was rejected

1. `R_lex` is not separable as `r_lateral+r_transition` and is not monotone in either component
   order. Its leading Boolean and violation fields require evaluating the joint quintic profile.
2. Coverage row 1 requires the global minimum of `B`; row 2 uses a diversity term that changes after
   row 1 and depends on both normalized transitions and lateral values.
3. The universal coverage lower bounds (`0.002*source_priority` and, after diversity, that value
   minus 2) are too weak to prove exclusion.
4. A bounded heap or partial sort reduces ordering cost only after scores exist; it cannot justify
   skipping proxy evaluation.
5. Deduplicating equal geometry rows before coverage would be wrong because their different
   entry/exit normalizations can change the greedy diversity trajectory. v2 reuses only the proxy
   fields and keeps every configuration in that stream.

VUE019 is the decisive observed countercase for further exact pruning: all 71,506 non-production
configurations have unique `H`, so the only proven full-proxy skip count is zero. No unproved region
was removed.

## Rejected performance experiments

- Eight metric workers preserved 86/86 parity but increased the measured worst-event runtime and
  CPU pressure relative to the fixed four-worker implementation; it was reverted.
- Grouping metric work by basis preserved 86/86 parity but worsened tail balance because basis
  sample lengths differ; it was reverted.
- Approximate score cutoffs, tolerance changes, reduced center catalogs, and changed factor order
  were not attempted because they change the frozen method.

The retained implementation is exact lazy reuse for equivalence classes plus exact partial
selection after scoring. It is not an approximate or globally lazy factor-space search.
