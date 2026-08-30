# Selector hypotheses and DEVELOPMENT evidence

## H3 — fixed bounded factorized basis

가설은 small fixed ordering만으로 artificial template coupling의 상당 부분을 제거할 수 있다는 것이다. geometry에 따라 candidate order를 바꾸지 않으므로 가장 단순한 factorization baseline이다.

- K=4/8/12/16/24 회복: `8/10/10/11/13` of 23.
- K=24에서 oracle-informed simple A+B와 같은 13/23을 bounded online-selectable factor recipes로 재현했다.
- limitation: K=12에서 10/23으로 H4-B보다 6개 적었다. geometry를 무시한 순서가 후보 budget을 낭비한다.

## H4-A — geometry-conditioned transition

가설은 entry 공간, merge 공간, reference curvature와 track-width 변화가 필요한 transition length를 설명하며, lateral factor priority는 고정해도 된다는 것이다.

- K=4/8/12/16/24 회복: `9/12/14/14/16` of 23.
- K=24에서 template/transition 12/13, D-probe 2/3, OTHER 2/6을 회복했다.
- success control K=24: hard-valid 생성 18/18, production-selected side availability 17/18.
- production-first fallback에서는 success event에 proposed stage를 실행하지 않으므로 실제 선택 변화와 추가 validator 호출은 0이다.
- limitation: standalone replacement라면 한 control에서 production-selected side가 빠지고, best proposed margin이 production selected margin보다 낮아지는 사례가 있다.

## H4-B — geometry-conditioned lateral and transition

가설은 corridor width/center, obstacle span과 entry distance로 lateral target/middle도 함께 고르면 더 작은 K로 같은 회복을 얻을 수 있다는 것이다.

- K=4/8/12/16/24 회복: `13/15/16/16/16` of 23.
- K=12에서 H4-A K=24와 같은 16/23을 절반의 tuple budget으로 달성했다.
- K=12 mechanism: D-probe 2/3, S-probe 1/1, template/transition 11/13, OTHER 2/6.
- success control K=12: standalone hard-valid 생성 11/18, production-selected side availability 10/18.
- limitation: failure recovery 효율은 가장 높지만 success geometry breadth가 부족하다. production-first failure-only stage가 아니면 replacement로 사용할 수 없다.

## Development-only 결정

Primary는 `H4A K=24`다. H4-B보다 후보 budget은 크지만 failure 회복은 동일하고, auxiliary success control 18/18에서 hard-valid availability를 유지했다. production-first fallback은 필수다.

Fallback/efficiency ablation은 `H4B K=12`다. 같은 16/23을 절반의 K로 얻으므로 geometry-conditioned lateral selection의 계산 효율 기여를 분리할 수 있다. success-control 단독 availability가 11/18이므로 primary replacement로 승격하지 않는다.

이 선택은 DEVELOPMENT freeze이며 unseen 일반화나 논문 novelty를 의미하지 않는다.

