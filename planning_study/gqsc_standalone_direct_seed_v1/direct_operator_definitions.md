# Direct operator definitions

All operators are research/shadow-only and consume `P3R3K12SideGeometry`, not a legacy path or
validator result. For side corridor domain `D=[a,b]`, let `near(x,y)` be the endpoint nearest zero
(numeric value breaks ties), `far` the other endpoint, and `clip_D` the side-domain clamp.

## Target anchors

1. `T_grid = argmin_(q in {a+i(b-a)/4, i=0..4}) (|q|,q)` reproduces the M0 target-grid geometry.
2. `T_inset = near(a,b) + (far(a,b)-near(a,b))/64` reproduces the zero-interface boundary inset.

These are the only generic direct target seeds. Connected corridor component `[c0,c1]` is used
directly by seven bounded lateral recipes at fractions `0, 1/32, 1/8, 3/8, 1/2, 2/3` (with the
existing target/mid offsets or center interpolation), while bottleneck/midpoint corridor centers
provide center-equal and center-interpolation anchors. The resulting bank remains capped at 32 per
side and 64 after deterministic RIGHT/LEFT interleaving.

## Transition operators

For each side, `e0/e1` and `x0/x1` are the geometry-derived minimum/maximum entry and exit scales.
Define `eq=e0+(e1-e0)/4`, `xn=x0+(x1-x0)/64`, and `xm=x0+(x1-x0)/2`. The seven family operators are:

| family | entry | exit |
|---|---:|---:|
| SHORT_ENTRY_SHORT_EXIT | e0 | x0 |
| MEDIUM_ENTRY_SHORT_EXIT | eq | x0 |
| LONG_ENTRY_SHORT_EXIT | e1 | xn |
| LONG_ENTRY_MEDIUM_EXIT | e1 | xm |
| LONG_ENTRY_LONG_EXIT | e1 | x1 |
| SHORT_ENTRY_LONG_EXIT | e0 | x1 |
| SHORT_ENTRY_MEDIUM_EXIT | e0 | xm |

The concrete numbers are evaluated for the lateral factor's own side; only seven semantic families
exist. The existing B128 diagonal wave, cheap corridor proxy, lexicographic Top-10, coverage reserve
Top-2, exact reconstruction, validator, and final candidate ranking are unchanged.

## Explicit exclusions

No M0-V1/M0-V2/M1 path is constructed, no analytic equation is solved, no legacy candidate is
validated, and no factor is harvested from a legacy result. Analytic `d_mid`, probe position/anchor,
root branch, template verdict, and exact-validator result are not inputs. There is no hidden exact-R3
or legacy enumeration.
