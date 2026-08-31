# Parameter provenance

No event/map-specific numeric transition or lateral value was introduced.

| item | class | origin and scaling | dataset use | map-scale implication | disposition |
|---|---|---|---|---|---|
| `K=12` | COMPUTATIONAL | frozen reconstruction/validator bound | pre-existing frozen v3 | none geometrically | unchanged |
| `B=128` | COMPUTATIONAL | frozen pair-proxy bound | pre-existing frozen v3 | none geometrically | unchanged |
| frozen `10+2` quota | COMPUTATIONAL | v3 lexicographic/coverage allocation | frozen reference | none | reference only |
| S1 `8+4` quota | COMPUTATIONAL | fixed reallocation of K12; global existing disjoint coverage | predeclared after structural diagnosis, evaluated on allowed seen data | none | clean research candidate, not frozen |
| S2 `8+4` quota | COMPUTATIONAL | same K12 split plus categorical unseen-transition preference | predeclared, allowed seen ablation | none | rejected for regression |
| S3 `6+6` quota | COMPUTATIONAL | symmetric K12 allocation control | predeclared, allowed seen ablation | none | no gain over S1 |
| B1 long-short stripe | DIMENSIONLESS_GEOMETRIC | existing transition-family category; no scale literal | repeated B128 mechanism in two exact cases, then allowed seen ablation | recomputed from each geometry | rejected for regression |
| B2 short-short/long-short stripes | DIMENSIONLESS_GEOMETRIC | existing transition-family categories; no scale literal | same two-case repeated mechanism, then allowed seen ablation | recomputed from each geometry | rejected for regression |
| exact shape dedup | COMPUTATIONAL | frozen bit-exact preconstruction shape key | pre-existing frozen v3 | none | unchanged |
| stable tie order | COMPUTATIONAL | frozen source/lateral/transition/side order | pre-existing frozen v3 | none | unchanged |
| runtime warmup `2`, repeat `10` | COMPUTATIONAL | measurement protocol only | 104 allowed seen snapshots | none; not online | reporting only |
| legacy exit ratio `1/64` | DIMENSIONLESS_GEOMETRIC | existing M0-V1 source sample over `[x_min,x_max]` | source provenance and five witnesses | scales with exit interval | analyzed, not copied into GQSC |
| legacy entry midpoint `1/2` per iteration | COMPUTATIONAL | standard interval bisection | source provenance | interval-normalized | analyzed, not adopted |
| observed entry/exit witness values | EVENT_OR_MAP_SPECIFIC outputs | validator-steered results for named frames | five production/raceline frames | would require recomputation | forbidden and not adopted |

S1 is map-independent as a bounded selector because it changes only allocation inside K12 and
uses existing normalized/geometry proxy fields.  B1/B2 are also geometry-scaled in definition,
but their empirical non-regression failure prevents selection.  No new PHYSICAL or
DEVELOPMENT_TUNED_GENERAL method constant was added.
