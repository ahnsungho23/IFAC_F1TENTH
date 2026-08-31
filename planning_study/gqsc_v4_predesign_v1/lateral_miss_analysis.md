# Lateral-operator miss analysis

The two exact misses do not repeat one geometry relation.

| case | valid left domain | exact witness `(d_target,d_mid)` | normalized target | missing structure |
|---|---|---|---:|---|
| RP3_90 | `[0.342707604849, 0.956500000000]` | `(0.649603802425, 0.649603802425)` | `0.5` | component/domain half with `d_mid=d_target` |
| RP3_296 | `[0.425141089977, 0.981500000000]` | `(0.385836200860, 0.385836200860)` | `-0.0706466` | equal target/mid extrapolated below the reported constant-side domain |

RP3_90 is a clean normalized midpoint equality.  RP3_296 is qualitatively different: its legacy
witness lies outside the frozen direct side-domain interval and cannot be represented as the same
in-domain midpoint operator.  One shared rule is therefore not independently supported by both
cases.

No lateral operator is proposed.  Adding only the RP3_90 midpoint rule or copying RP3_296's
negative normalized offset would be snapshot-specific coverage repair.  This is a diagnosis-only
result and does not prove a lateral operator cannot be generalized with more independent evidence.
