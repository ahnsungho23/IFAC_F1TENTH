# Freeze gate

`GQSC_STANDALONE_V2_READY_FOR_FREEZE`

| gate | result |
|---|---:|
| legacy hard-success controls | 18/18 |
| legacy usable-success controls | 14/18 |
| legacy-baseline usable controls preserved | 14/14 |
| all-seen usable vs v1 | 55/104 vs 54/104 |
| production-failure usable vs v1 | 41/86 vs 39/86 |
| standalone-only gains DVE003/DVE022/DVE028 preserved | 1 |
| hidden legacy execution | 0 |
| max lateral / transition / B / reconstruction / validator | 64 / 7 / 128 / 12 / 12 |
| warm Release p95 / p99 | 15.155 / 18.689 ms |
| event-ID rule or copied event literal | 0 |

The recommended frozen research policy is `ZERO_INTERFACE_EQUAL_MIN_OUTWARD` plus the
`COMPONENT_HALF_FAR002_INWARD015` replacement/transition ordering. This statement recommends only
a research method freeze. It does not authorize a production decision-path change.
