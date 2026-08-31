# GQSC v3 parameter provenance before outcome evaluation

## 선언

v3는 DEVELOPMENT deficiency가 여러 independent event에서 반복되지 않았기 때문에 새 lateral
trajectory anchor를 추가하지 않는다. 따라서 `0.02 m`, `0.15 m` 또는 그 값을 흉내 내는
normalized coefficient가 없다. 아래 표는 outcome comparison 전에 확정한 v3-specific 상수와
상속값을 분리한다.

| symbol/rule | equation/value | units | source quantity | class | tunable | selection data |
|---|---|---|---|---|---:|---|
| coverage disjoint quota | `2` | candidate | frozen K12 coverage quota | `COMPUTATIONAL` | no | frozen R3-K12 contract |
| lexicographic quota | `10` | candidate | frozen K12 quota | `COMPUTATIONAL` | no | frozen R3-K12 contract |
| pair budget | `128` | pair proxy | standalone v1 hard bound | `COMPUTATIONAL` | no | frozen standalone contract |
| reconstruction/validator cap | `12/12` | calls | K12 | `COMPUTATIONAL` | no | frozen standalone contract |
| side coordinate | `sigma=0 RIGHT, 1 LEFT` | 1 | categorical side | `DIMENSIONLESS_GEOMETRIC` | no | convention, no outcome |
| normalized target | `u_t=(d_target-n)/(f-n)` | 1 | side-domain endpoints | `DIMENSIONLESS_GEOMETRIC` | no | exact input geometry |
| normalized mid | `u_m=(d_mid-n)/(f-n)` | 1 | side-domain endpoints | `DIMENSIONLESS_GEOMETRIC` | no | exact input geometry |
| normalized transition | `e,x in [0,1]` from geometry min/max | 1 | entry/exit geometry | `DIMENSIONLESS_GEOMETRIC` | no | inherited direct transition definition |
| maximin coordinate weights | all five coefficients `1` | 1 | unweighted Euclidean geometry | `DIMENSIONLESS_GEOMETRIC` | no | symmetry/equal treatment, no outcome fit |
| side-balanced slots | `1 RIGHT + 1 LEFT` when both eligible | candidate | coverage quota 2 | `COMPUTATIONAL` | no | categorical symmetry, no outcome |
| floating comparison epsilon | `1e-9` | numeric tolerance | existing binary64 comparison convention | `COMPUTATIONAL` | no | inherited source convention |
| final tie | existing proxy then stable tie | n/a | frozen selector | `COMPUTATIONAL` | no | inherited deterministic rule |

## 상속된 clean v1 lateral bank

Clean v1에는 과거 R3 general bank에서 상속한 absolute meter offsets가 있다. 이들은 v3가 새로
도입하거나 outcome에 맞춰 고른 상수가 아니며, 이번 연구는 그 전체 bank의 재동결을 주장하지
않는다. v3-specific provenance gate는 다음 두 항목을 명시적으로 금지한다.

- `COMPONENT_HALF_FAR002_INWARD015`: disabled in every v3 policy
- `ZERO_INTERFACE_EQUAL_MIN_OUTWARD`: not a v3 candidate

향후 clean v1 absolute basis 자체를 완전히 normalized basis로 교체하려면 별도 연구와 새
prevalidation lock이 필요하다. 이번 task에서는 DVE039을 위한 대체 coefficient를 만들지 않는다.

## Map-scale contract

새 v3 rule의 `u_t`, `u_m`, `e`, `x`와 maximin distance는 positive uniform geometry scale에
불변이다. B128/K12와 slot count도 map scale에 무관하다. v3가 새 절대 meter trajectory
offset을 도입하지 않으므로 이번 redesign 때문에 추가되는 map-scale retuning 항목은 없다.
