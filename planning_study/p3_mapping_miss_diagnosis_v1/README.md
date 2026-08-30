# P3 Direct Mapping-Miss Diagnosis v1

## 결론

저장된 `p3_oracle_pilot_v2` 결과에서 요청된 논리식을 다시 적용하면 direct
same-target/same-transition `d_mid` miss는 **V2E01, V2E02, V2E08**이다. ID는 사전 가정하지
않고 `classification == ORACLE_VALID_P3_EXISTS`와
`any_valid_same_target_scales_but_missed_d_mid == true`의 교집합으로 도출했다.

진단 결과는 다음과 같다.

| event | bag | direct 비교 tuple `(d_target, entry, exit)` | production → oracle `d_mid` | production probe residual | 지배 분류 |
|---|---|---|---|---:|---|
| V2E01 | 08-24 19:24 | `(-0.456880, 0.514581, 0.604987)` | `-0.536880 → -0.510000` | `+0.026880 m` | `D_PROBE_DOMINANT` |
| V2E02 | 08-24 19:00 | `(+0.150000, 1.310934, 6.630707)` | `+0.414194 → +0.170000` | `-0.244194 m` | `D_PROBE_DOMINANT` |
| V2E08 | 08-25 09:23 | `(-0.150000, 0.514581, 7.397546)` | `-0.150000 → -0.100000` | `+0.027909 m` | `ROOT_FILTERING_DOMINANT` |

V2E01과 V2E02에서는 oracle `d_mid`가 current point constraint의 root가 아니어서 **analytic
solve 이전의 probe constraint에서 제외**됐다. 같은 `s_probe`에서 `d_probe`만 oracle 경로의
값으로 바꾸면 두 값 모두 production의 active branch와 lateral root bound를 통과한다. 반대로
현 anchor 의미를 유지한 deterministic station sweep에서는 residual zero가 없다. 따라서 두
독립 non-stress bag에 대한 가장 직접적인 진단은 `d_probe` anchor miss다.

V2E08은 수학적 direct predicate에는 포함되지만 `LOCALIZATION_STRESS_CANDIDATE`다. 더구나
oracle `d_mid=-0.10`은 production의 `SAME_SIGN_NEGATIVE__SIGN_CHANGE__SAME_SIGN_POSITIVE`
active branch 대신 `SIGN_CHANGE__SIGN_CHANGE__SIGN_CHANGE`이고, M0 lateral bound
`[-0.7725,-0.15]` 밖이다. station을 옮겨 point residual을 0으로 만들 수 있어도 root filter가
제거한다. 그러므로 강한 non-stress 공통 결론은 V2E01/V2E02에 한정하고, V2E08은 별도 stress
및 branch-completeness 증거로만 보존한다.

모든 P3 Oracle Pilot v2 event V2E01–V2E09에는 영구적으로
**`PILOT_SEEN_DEVELOPMENT_DATA`**를 부여했다. 이 9건은 향후 final unseen validation 또는
holdout evidence로 사용할 수 없다. 전체 표는 [case_summary.csv](case_summary.csv)에 있다.

## 범위와 provenance

- Frozen baseline: `research_instrumentation_v2`, commit
  `80ae205fd470f537bd1e5449fb906c5548f6653a`.
- v2 source/config aggregate SHA-256: `55a078d8...2100521` / `6009b321...e334a`.
- 현재 `p3_shadow.cpp` SHA-256: `cbbcc6e181d0d416b3d3be47b0be5b2cbfd4cf39ddd34cc1ac223989e9257ca5`.
- 현재 `p3_analytic_solver.hpp` SHA-256:
  `e410106266984237f6b1e3286b56aa0093d8ae394a66cc838dbcb2d16c3b8081`.
- 입력 artifact SHA-256: `oracle_summary.csv` `0d30bba9...c2c95d6`,
  `production_candidates.csv` `c1b4fa0c...2c12d1`, `oracle_valid_candidates.csv`
  `4c50a44c...96333`.
- v2의 9/9 evaluator lineage 및 64/64 production candidate digest/verdict/margin parity를
  acceptance gate로 유지했다. 이 진단의 oracle 대표 경로도 동일한 audit-only C++ harness로
  재구성하여 모두 hard-valid와 path digest가 일치했다.
- production source, parameter, validator, ranker, lifecycle, speed shaping, collision model은
  수정하지 않았다. `/tmp` detached audit copy의 출력만 확장했고 새 mapping은 만들지 않았다.

원래 oracle domain은 finite deterministic grid다. 아래의 `observed valid d_mid range`는 해당
grid에서 관측된 support이지 연속 feasible set의 수학적 증명이 아니다.

## Current mapping과 진단식

### Corridor와 probe

`[CURRENT IMPLEMENTATION]` corridor sample station 집합은 ego `0`, cluster start/end/midpoint,
각 visible obstacle의 start/center/end, horizon 안의 모든 global-reference station이다. track
interval에는 `vehicle_half_width + wall_safety_margin = 0.15 + 0.04 m`가 투영되고, active
obstacle의 이미 확장된 lateral envelope를 빼서 side branch를 만든다
([p3_shadow.cpp](../../src/local_planning/src/p3_shadow.cpp):1100–1195). Obstacle envelope 자체에는
`vehicle_half_width + safety_margin = 0.15 + 0.08 m`가 이미 들어간다. 사용 parameter는
[local_planning.yaml](../../src/local_planning/config/local_planning.yaml):28,40,498이다.

Open cluster span의 branch samples를 `B`라 하면 production probe는 다음과 같다
([p3_shadow.cpp](../../src/local_planning/src/p3_shadow.cpp):1208–1265).

```text
M0_V1:
  s_probe = arg max_{b in B} (|kappa_ref(b.s)|, b.s)
  d_probe = clamp(d_target, b.lower + 0.08, b.upper - 0.08)
            (inset interval이 비면 b.center)

M0_V2 / M1 analytic templates:
  s_probe = arg min_{b in B} (b.width, b.s)
  d_probe = (b.lower + b.upper) / 2
```

따라서 curvature tie는 **뒤 station**, bottleneck tie는 **앞 station**을 고른다. M0_V1은
`CURVATURE_CONTINUITY / CONTINUITY_BIASED_SAFE_ANCHOR`, M0_V2와 M1은
`BOTTLENECK_CENTER / CORRIDOR_CENTER`다. 세 direct snapshot에서 실제 constructed stage는
V2E01 `M0_V1 6 + M1 3`, V2E02 `M1 3`, V2E08 `M0_V1 6 + M1 3`이며 M0_V2 constructed
candidate는 0이었다.

### Analytic equation과 filter order

Unknown을 `x=d_mid`라 하면 source harmonic derivative가 active인 segment에서 sample 값은
다음 rational-affine form이다.

```text
d_P3(s_probe; x) = alpha x + beta + w (a x + b)/(c x + e)
d_P3(s_probe; x) = d_probe

=> A x^2 + B x + C = 0
A = alpha c
B = alpha e + (beta-d_probe)c + w a
C = (beta-d_probe)e + w b
```

Production coefficient construction은
[p3_analytic_solver.hpp](../../src/local_planning/include/local_planning/p3_analytic_solver.hpp):291–300,
stable quadratic solve는 같은 파일 303–348이다. Raw roots에는 순서대로
`finite → exact 3-knot branch regime → lateral bound → reconstructed forward residual <= 1e-12`
filter가 적용되고, 그 뒤에만 경로를 구성하여 hard validator를 실행한다
([p3_shadow.cpp](../../src/local_planning/src/p3_shadow.cpp):1285–1345). M1은 active-outer
quadratic 외에 all-inactive linear root도 검사한다
([p3_shadow.cpp](../../src/local_planning/src/p3_shadow.cpp):1708–1758, 2192–2250).

각 raw root, branch/bound/residual filter, constructed candidate failure는
[production_mapping_trace.csv](production_mapping_trace.csv)에 빠짐없이 분리했다. ZERO_INTERFACE
template은 analytic point equation이 없고 `d_mid=d_target`을 직접 구성하므로 별도
`NOT_APPLICABLE_ZERO_INTERFACE` row다.

## V2E01 — curvature-continuity anchor miss

### Environment → production mapping

- Ego `(s,d,v)=(5.827398,-0.358708,0.183166)`.
- Single obstacle id 145, ego-forward span `[0.141337,0.164800]`, raw lateral
  `[-0.226880,-0.208339]`, expanded `[-0.456880,+0.021661]`.
- RIGHT `d_target=-0.4568796095`.
- M0_V1은 obstacle center이자 `z2=0.1530681159`에서 `|kappa|=0.304297`인 sample을
  curvature-critical로 선택했다. RIGHT branch는 `[-0.8975,-0.4568796]`이고, upper에 다시
  0.08 m inset을 적용하여 `d_probe=-0.5368796095`가 됐다.
- Primary entry/exit `0.514581/0.604987`의 polynomial은
  `0.0554788664 x² + 0.0548890980 x + 0.0134776249 = 0`이다.
- Raw root `-0.536879610`은 accepted되어 경로를 만들었으나 첫 failure는 left control
  steering curvature다. 다른 raw root `-0.452489886`은
  `SIGN_CHANGE__SIGN_CHANGE__SIGN_CHANGE`라 branch filter에서 path construction 전에
  제거됐다.
- 다른 exit/entry 조합도 accepted root를 사실상 같은 `-0.536880`으로 고정했고 curvature
  또는 `footprint_track_bound`로 실패했다. M1 bottleneck-center는 같은 station에서
  `d_probe=-0.677190`, root `-0.677190`을 만들고 right control steering curvature로 실패했다.
  세부 조합은 trace CSV에 있다.

### Oracle counterfactual

같은 target/entry/exit에서 nearest valid는 `d_mid*=-0.510000`; grid-observed valid range는
`[-0.510000,-0.460000]`. nearest representative hard margins는 footprint track `+0.345146`,
obstacle `+1.500000`, slope `+0.052732`, signed curvature `+0.161093`, curvature rate
`+15.788850`, normalized slack `+0.122386`이다. 같은 tuple의 largest-slack point는
`d_mid=-0.460000`, normalized slack `+0.298851`이다.

```text
d_oracle(s_probe_prod) = -0.510000
d_probe_prod           = -0.5368796095
Delta d_probe          = +0.0268796095 m
```

Oracle mid는 production assumed branch와 bound `[-0.8975,-0.4568796]`를 모두 통과한다.
따라서 fixed station에서 anchor만 `-0.510000`으로 바뀌었다면 exact root로 수용된다. 반면
현 continuity-anchor 의미로 full open-cluster station을 sweep한 최소 `|R(s)|`는 production
center에서 그대로 `0.0268796 m`이고 zero/near-zero station이 없다. 분류는
**A_PROBE_CONSTRAINT_MISS / D_PROBE_DOMINANT**다.

## V2E02 — bottleneck-center anchor miss

### Environment → production mapping

- Ego `(s,d,v)=(14.502267,+0.049712,0)`.
- Single obstacle id 228, span `[0.183192,0.399942]`, raw lateral
  `[-0.330816,-0.211613]`, expanded `[-0.560816,+0.018387]`.
- LEFT target `+0.15`, M1 `NEAR_LONG`, entry/exit `1.310934/6.630707`.
- Bottleneck station은 obstacle center이자 `z2=0.2915671120`; LEFT branch
  `[+0.018387,+0.810000]`의 center인 `d_probe=+0.4141936096`를 사용했다.
- Polynomial은 `0.1831924839 x² - 0.0969262534 x + 0.00871840156 = 0`이다.
- Raw `+0.114901533`은 all-sign-change라 active-branch filter에서 제거됐다. Raw
  `+0.414193610`만 accepted되어 path를 만들었고 `maximum_lateral_slope`로 실패했다.
  all-inactive linear counterpart도 같은 active-regime root를 내어 inactive predicate에서
  제거됐다. ZERO_BOUNDARY_SHORT/SPAN은 `d_target=d_mid=+0.1603125`를 short exit과 결합해
  right control steering curvature로 실패했다.

### Oracle counterfactual

같은 target/entry/exit의 nearest valid는 `d_mid*=+0.170000`; grid-observed range는
`[+0.110000,+0.170000]`. hard margins는 footprint track `+0.125626`, obstacle `+0.151589`,
slope `+0.348793`, signed curvature `+0.497082`, rate `+17.343898`, normalized slack
`+0.092431`이다. 같은 tuple largest-slack point는 `d_mid=+0.1603125`, normalized slack
`+0.094609`; event-global largest는 다른 target `+0.26`이므로 direct 진단과 분리했다.

```text
d_oracle(s_probe_prod) = +0.170000
d_probe_prod           = +0.4141936096
Delta d_probe          = -0.2441936096 m
```

Oracle mid는 assumed branch 및 component bound `[+0.15,+0.81]`를 모두 통과하므로 fixed
station에서 anchor만 바꾸면 accepted analytic root다. 현 corridor-center 의미를 유지한
station sweep의 최소 residual도 `-0.244194 m`이며 zero가 없다. 분류는
**A_PROBE_CONSTRAINT_MISS / D_PROBE_DOMINANT**다.

## V2E08 — 별도 localization-stress/root-filter 사례

V2E08은 direct predicate의 세 번째 event지만 `LOCALIZATION_STRESS_CANDIDATE`이며 강한
non-stress 결론에서 제외한다.

- Ego `(s,d,v)=(14.696796,-0.008081,0)`, obstacle id 110의 span은 ego 뒤
  `-0.015183 m`에서 앞 `+0.389372 m`까지 걸친다. raw lateral은
  `[+0.227764,+0.808856]`, expanded는 `[-0.002236,+1.038856]`이다.
- M0_V1은 near-zero curvature tie의 뒤 reference sample `s_probe=0.351273820`을 택하고,
  continuity anchor는 `d_target=d_probe=-0.15`를 그대로 유지했다.
- Primary long-exit polynomial은
  `-10.35808054 x² - 3.106012591 x - 0.2328450764 = 0`. Accepted root
  `-0.1500000016`은 left control steering curvature로 실패하고, raw `-0.1498637211`은 branch
  mismatch로 제거됐다.
- M1 bottleneck-center도 같은 station에서 `d_probe=-0.374868`; active root
  `-0.552723`은 slope로 실패하고, 다른 raw `-0.149998`과 all-inactive counterpart는 branch
  predicate에서 제거됐다.

같은 M0 target/entry/exit nearest oracle `d_mid*=-0.10`은 production probe에서
`d_oracle=-0.122091`, 따라서 `Delta d_probe=+0.027909 m`다. 그러나 anchor를 그렇게 바꿔도
oracle regime은 all-sign-change이며 root upper bound `-0.15`보다 안쪽이다. 즉 branch와 bound
둘 다 실패하므로 fixed-`s_probe` anchor 변경만으로는 production root가 되지 않는다.

현 continuity anchor의 station sweep은 `s≈0.250756` 및 cluster-end 직전
`s≈0.389372`에서 residual이 거의 0이다. 첫 값은 P3의 `z1=0.250643` 부근이지만 original
corridor sample set에는 없는 transition knot이고, 두 번째 `z3`는 open-cluster probe 선택에서
제외되는 끝점이다. 더 중요한 점은 어느 station도 branch/lateral filter를 통과시키지 못한다는
것이다. 분류는 **B_ROOT_SOLVER_OR_BRANCH_FILTER_MISS / ROOT_FILTERING_DOMINANT**다.

## V2E03 — transition/template 선택 증거

V2E03은 direct probe miss가 아니다. Production M1 `ZERO_BOUNDARY_SHORT`는
`d_target=d_mid=+0.1995871002`, entry/exit `0.514581/0.497168`을 만들었고 right control
steering curvature로 실패했다. Oracle에는 exact same target/mid가 entry `0.514581`, exit
`6.620728`과 결합될 때 hard-valid인 row가 있으며 normalized slack은 `+0.009442`다.

두 scale 값은 모두 production이 다른 template에서 이미 사용한 값이지만 ZERO_INTERFACE는
long exit과 조합하지 않았다. 따라서 이는 `s_probe/d_probe` 직접 증거가 아니라
**TEMPLATE_OR_TRANSITION_SELECTION** evidence다. [production_vs_oracle.csv](production_vs_oracle.csv)에
별도 row로 보존했다.

## Cross-event pattern

두 independent non-stress bag의 공통점은 single near-ego obstacle, cluster 안의 한 station에서
point constraint를 부과하고, 그 station이 각각 P3 `z2`와 일치하면서 `d_mid`가 anchor에
사실상 고정됐다는 점이다. 그러나 실패한 anchor 의미는 서로 다르다.

- V2E01: expanded obstacle edge와 같은 `d_target`에서 M0가 추가 0.08 m inset을 적용해 더
  바깥 root를 강제했다.
- V2E02: 넓은 LEFT branch의 center가 near-zero target보다 0.264 m 더 바깥이라, feasible
  path보다 훨씬 큰 middle excursion을 강제했다.
- 두 경우 모두 fixed station의 필요한 anchor로 바꾸면 branch/bound를 통과하고, station만
  바꾸는 현 anchor sweep은 실패한다.
- V2E08은 같은 mechanism으로 묶을 수 없다. station residual은 맞출 수 있지만 desired
  oracle middle이 active-outer sign regime와 side-monotonic bound 밖이다.

따라서 반복된 non-stress mechanism은 “curvature station이 항상 이르거나 늦다”가 아니라,
**한 점의 current anchor가 feasible middle excursion을 과도하게 바깥으로 고정한다**는 것이다.
V2E08은 별도의 branch-completeness/root-bound mechanism을 시사한다.

## 요청된 9개 질문에 대한 답

1. **Direct event는 무엇인가?** V2E01, V2E02, V2E08. 단, E08은 stress flag 때문에 강한
   non-stress 결론에서 별도 취급한다.
2. **왜 oracle `d_mid`가 production에 없었나?** E01은 continuity-safe anchor가
   `-0.53688`, E02는 bottleneck center가 `+0.41419`를 요구해 oracle mid가 point equation의
   root가 아니었다. E08은 point equation도 불일치하고 active branch 및 lateral bound도
   위반했다.
3. **어느 단계의 실패인가?** E01/E02는 root solve 이전 A
   `PROBE_CONSTRAINT_MISS`; 생성된 production roots는 그 뒤 validator에서 각각 curvature와
   slope로 실패했다. E08 oracle mid의 결정적 장벽은 B `ROOT_SOLVER_OR_BRANCH_FILTER_MISS`다.
4. **`s_probe` 고정, `d_probe` 변경으로 회복하는가?** E01은 `+0.026880 m`, E02는
   `-0.244194 m` anchor 변경 시 yes. E08은 point equation만 맞출 뿐 branch/bound 때문에 no.
5. **현재 `d_probe` 의미를 유지하고 다른 station으로 회복하는가?** E01/E02 no. E08은
   point residual zero station은 있으나 production accepted root로는 no.
6. **지배 문제는 무엇인가?** E01/E02 `D_PROBE_DOMINANT`; E08
   `ROOT_FILTERING_DOMINANT`. 세 사례를 하나의 s/d 원인으로 강제하지 않았다.
7. **독립 bag 반복 패턴이 있는가?** Yes, E01/E02 두 08-24 bag에서 current anchor가 필요한
   feasible middle보다 바깥으로 치우친다. E08까지 포함한 단일 공통 mechanism은 없다.
8. **새 geometry-to-probe mapping 설계를 정당화하는가?** **개발 연구를 시작할 근거는
   충분하지만 production 채택 근거는 아니다.** 두 independent non-stress 사례가 fixed-station
   anchor miss를 직접 보이고, 모든 사례는 이미 seen development data다. 새 unseen corpus와
   frozen comparison이 필요하다.
9. **향후 가장 방어 가능한 가설은?** 아래 4개다. 아직 구현하거나 튜닝하지 않았다.

   - H1: expanded obstacle envelope 위에 M0의 추가 `safety_margin` inset을 다시 쓰는 anchor가
     일부 near-ego case에서 필요 이상으로 바깥 root를 강제한다.
   - H2: bottleneck branch center 한 점보다 `d_target` continuity를 보존하는 feasible anchor
     후보가 wide corridor에서 더 적합할 수 있다.
   - H3: curvature-critical/bottleneck 한 station만 쓰는 대신 기존 geometry의 복수 constraint
     station 후보를 비교하면 one-point aliasing을 줄일 수 있다.
   - H4: `d_mid`가 `d_target`보다 center 쪽인 all-sign-change P3를 배제하는 branch/bound 가정의
     coverage를 별도 연구해야 한다. 현재 근거는 stress case E08 하나라 가장 약하다.

## Artifacts

- [case_summary.csv](case_summary.csv): 9개 전 event의 permanent development-data label,
  lineage/parity, 역할과 geometry.
- [production_mapping_trace.csv](production_mapping_trace.csv): template/branch별 probe, equation
  coefficient, 모든 active/all-inactive raw root, pre-construction filter, constructed validator 결과.
- [oracle_valid_representatives.csv](oracle_valid_representatives.csv): nearest, same-tuple largest
  margin, observed interval endpoints, event-global largest-margin 대표와 full hard margins/path.
- [production_vs_oracle.csv](production_vs_oracle.csv): direct primary comparison 및 V2E03 별도 비교.
- [probe_residual_at_production.csv](probe_residual_at_production.csv): Test A의 exact residual.
- [station_counterfactual_scan.csv](station_counterfactual_scan.csv): 각 open-cluster interval의
  501-point deterministic Test B scan, zero/near-zero 및 local minima flag.
- [root_filter_diagnosis.csv](root_filter_diagnosis.csv): oracle mid의 equation, branch, lateral-bound
  counterfactual과 A/B/C/D 분류.
- [paths/](paths/): 각 oracle representative의 frozen C++ reconstructed full Frenet/Cartesian path.
- [plots/](plots/): 각 direct event의 A XY/corridor, B Frenet, C residual, D `d_mid` view.
- [analyze_mapping_miss.py](analyze_mapping_miss.py): source식과 artifact join을 재현하는
  read-only analysis script.

Plot 바로가기:

- V2E01: [A](plots/V2E01_A_xy_corridor.png), [B](plots/V2E01_B_frenet_d.png),
  [C](plots/V2E01_C_probe_residual.png), [D](plots/V2E01_D_d_mid.png)
- V2E02: [A](plots/V2E02_A_xy_corridor.png), [B](plots/V2E02_B_frenet_d.png),
  [C](plots/V2E02_C_probe_residual.png), [D](plots/V2E02_D_d_mid.png)
- V2E08 stress: [A](plots/V2E08_A_xy_corridor.png), [B](plots/V2E08_B_frenet_d.png),
  [C](plots/V2E08_C_probe_residual.png), [D](plots/V2E08_D_d_mid.png)

## Evidence limits

- `[INFERENCE]` H1–H4는 진단에서 유도한 future design hypotheses이지 현재 구현의 설계 의도나
  성능 보장이 아니다.
- Dense station scan은 “그 station을 corridor sample로 강제했을 때 current anchor formula가
  내는 값”의 counterfactual이다. Production `chooseProbe()`가 그 station을 실제 후보 집합에
  추가한다는 뜻은 아니다.
- Oracle valid range는 finite grid observation이며 연속 feasible interval proof가 아니다.
- V2E08에서 localization이 원인이라고 주장하지도, localization과 무관하다고 주장하지도
  않는다.
- 이 진단은 새 mapping의 superiority, 전체 P3 failure의 해결, final
  `MAPPING_MISSED_EXISTING_P3` dataset label을 주장하지 않는다.
