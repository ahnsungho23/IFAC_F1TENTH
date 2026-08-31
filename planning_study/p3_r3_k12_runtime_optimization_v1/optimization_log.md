# R3-K12 behavior-preserving optimization log

모든 단계는 frozen `R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12`의 factor 의미, 순서, 10+2 quota,
K=12, tie, dedup, P3 geometry, validator, 최종 선택을 유지했다. 각 material 단계 직후 combined-seen
86개에서 ordered factor, path digest, validator verdict, final selected digest를 검사했다.

1. Profile-only 계측을 추가했다. context/geometry/factor/ordering/dedup/reconstruction/validation/rank와
   research lineage/capture/serialization을 분리했다. 이 시점 parity는 86/86이었다.
2. vector capacity를 예약하고 4개 quintic segment를 stack array로 바꾸며 exact hex를 factor마다
   캐시했다. lateral p50은 61.5 ms에서 16.7 ms로 감소했다.
3. 무거운 factor 객체 대신 pool index를 정렬했다. 비교식과 반환 순서는 동일하다.
4. 같은 exact `(d_target, transition)`의 reference station `t**i`를 재사용했다. 각 candidate의
   coefficient 곱과 Python-compatible 보상합 순서는 유지했다.
5. lexicographic 10개와 coverage 2개가 확보된 뒤 사용되지 않는 후속 순위를 계산하지 않았다.
   앞선 shape duplicate와 48개 coverage fallback 순서는 보존했다.
6. corridor/slopes/second-difference 임시 vector를 제거하고 같은 sample 순서에서 max/min 및
   compensated sum을 한 번에 누적했다.
7. Python `float.hex()` 형식을 고정 버퍼와 `to_chars`로 생성하고 key 임시 stream을 제거했다.
8. shape equality key를 8개 raw binary64 bit로 보관하고 선택된 12개에만 문자열을 만들었다.
9. full sort 대신 동일 total-order comparator로 필요한 최소 factor만 반복 선택했다.
10. lateral target/pair dedup을 exact-bit hash로 바꾸고 string 보유 객체 대신 index를 정렬했다.
11. 후보별 독립 pair metric을 최대 4개의 고정 contiguous chunk로 병렬 계산했다. pool index,
    candidate 내부 합산 순서, 결과 materialization은 결정적이다.
12. stable tie 앞의 `(source_priority, lateral_index, transition_index, side)`가 row를 유일하게
    식별하므로 configuration 문자열은 선택된 factor에만 만들고 production 제외는 exact bit key로
    수행했다.
13. pool에는 source 문자열을 복사하지 않고 lateral source catalog index만 보관했다.
14. lateral comparator를 short-circuit 비교로 바꿔 매 비교의 string/hex tuple 복사를 제거했다.

할당 수를 allocator hook으로 직접 계수하지는 않았다. ROS process 전역 allocation과 profiler
개입 비용을 planner allocation으로 오인할 위험이 더 컸기 때문이다. 대신 코드상 후보마다 있던
6개 임시 metric vector, 2개 긴 key string, 4개 source/hex string 복사와 full-object sort/erase를
제거했다. 이 판단은 phase timer와 exact parity로 검증했다.

최종 factor 통계는 lateral factor p50/max 1,220.5/4,471, pair pool p50/max
3,677.5/71,506이다. profile basis build p50/max는 124/2,064, cache hit p50/max는
3,539.5/69,442이다. 즉 중복 station/power 계산은 크게 줄었지만, frozen method는 여전히 최대
71,506개 factor의 proxy를 평가한다.
