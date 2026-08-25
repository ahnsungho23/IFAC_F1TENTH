# 제안서 — 간섭 판단(is_interfering)의 state_machine 이관 + 확률적 글로벌 라인 접근 판정
ㅇ
> **이력 문서:** `map_creator`와 `static_obstacle_map`은 2026-08-24에 `main`에서
> 제거됐다. 아래 두 패키지 관련 내용은 당시 의존성 분석 기록이다.

- 작성일: 2026-08-19 (브랜치 `myungsub_study`)
- 상태: **제안 (팀 합의 전)** — `f110_msgs` 메시지 변경을 포함하므로 전원 합의가 필요하다
- 변경 범위: `f110_msgs` / `obstacle_detector` / `state_machine` / `f1tenth_control`(cruise) (4개 패키지)

> ## 🗂️ HISTORICAL — 착지 구현의 설계 근거 보존본 (2026-08-24 부착)
>
> 이 문서는 **2026-08-19 설계 제안의 보존본**이며, 아래 서술이 곧 현재 구현은 아니다.
> 함께 가져오는 이유는 **착지 코드가 이 문서를 직접 참조**하기 때문이다:
> `interference_predicate.hpp` 의 "see §3.1/§3.3", `f1tenth_control/cruise_controller.cpp`
> 비상정지 주석의 "design rule 4 in §3.4".
>
> **착지 구현과의 차이**
> - `interference_source` 파라미터는 **도입되지 않았다** — 상태머신이 항상 내부 확률 술어를
>   쓴다(디텍터 `is_interfering` 위임 경로 없음).
> - 본문의 Phase 구분·이관 절차는 **이력**이다. 이관은 이미 완료됐다.
>
> **현행 계약의 정본은 `docs/state_machine_node.md`** 다. 파라미터 값은
> `config/state_machine.yaml` 을 따른다.

---

## 1. 요약 (TL;DR)

CRUISE 상태 전이를 결정하는 간섭 판단(`is_interfering`)이 현재 **인지 노드(obstacle_detector)** 안에
있다. 이를 팀 원칙("판단은 planning/FSM이 소유한다")에 맞게 **state_machine으로 이관**하고,
이관하는 김에 현행 이진 판정(현재 d의 corridor 겹침 + 등속 1초 종방향 예측)을
**공분산 기반 확률 판정**("상대가 T초 안에 내 글로벌 라인 밴드에 들어올 확률")으로 업그레이드한다.

이를 위해 필요한 것:

1. `f110_msgs/Obstacle.msg`에 교차공분산 필드 2개 추가 (`d_vd_cov`, `s_vs_cov`)
2. `obstacle_detector`의 `mergeLayer()`에서 트래커 `P(2,3)`/`P(0,1)`을 채움
   (size-가중 평균 + Cauchy–Schwarz 클램프로 PSD 보장)
3. `state_machine`에 확률 술어 구현 + `interference_source` 파라미터
   (기본 `detector` = **거동 변화 0으로 착지**, 시뮬 A/B 검증 후 `internal`로 전환)
4. `f1tenth_control`(cruise)의 속도 상한에 **공분산 시간 전파(constraint tightening)** 반영
   (기본 `τ_max = 0` = **구 거동과 비트 동일**, Phase 3에서 활성 — §3.4)

모든 실패 모드는 기존과 동일하게 "느려지는 방향"(GLOBAL 복귀)으로 강등된다. 되돌림은
파라미터 하나다.

📎 **실제 구현 코드는 §3.5**, **수식 ↔ 문헌 대응표는 부록 B**에 있다. 둘 다 파일 재대조·
빌드 실행을 포함한 적대적 검증을 거쳤다(§3.5 서두).

### 1.1 새로 들어오는 로직·개념 (용어집)

이 제안이 스택에 **처음 들여오는** 개념만 모았다. 각 항목의 `[Fn]`은 부록 B.4의 서지이고,
✅/🟡/🔴는 그 검증 등급이다(✅ = 수식 원문 대조 완료, 🟡 = 서지만 확인, 🔴 = 수식 근거 사용 금지).
**이 표만 읽어도 §3 전체의 어휘가 해석된다.**

| # | 개념 | 한 줄 정의 | 이 제안서에서 쓰이는 곳 | 문헌 |
|---|---|---|---|---|
| ① | **교차공분산** `cov(d,vd)`·`cov(s,vs)` | 위치와 속도 추정오차가 **함께** 틀리는 정도. 트래커 `P`의 비대각 성분 | **§3.1** 메시지 신규 필드 2개(`d_vd_cov`/`s_vs_cov`)의 정체. 지금은 대각만 발행하고 버린다 | — (트래커 산출물) |
| ② | **PSD(양반정치) 2×2 블록** | `[[var_a, cov],[cov, var_b]]`가 유효한 공분산이려면 대각 ≥ 0 **이고** `det ≥ 0` | **§3.2** 클램프의 판정 기준. 대각을 축별 max로, 교차항을 평균으로 뽑으면 이 조건이 깨질 수 있다 | 🟡 [F2] Horn–Johnson §7.2 |
| ③ | **Cauchy–Schwarz 클램프** | `\|cov\| ≤ √(var_a·var_b)`로 잘라 ②를 강제하는 폐형 사영 | **§3.2** 발행 직전 1줄. `0.99` 계수로 수치 여유를 둔다 | 🟡 [F1] Higham 2002 (문제 정식화 선례일 뿐 직접 출처 아님 — B.5-② 참고) |
| ④ | **총공분산 법칙**(혼합 모멘트) | 여러 추정을 합칠 때 `Σwᵢ[Pᵢ + (xᵢ−x̄)(xᵢ−x̄)ᵀ]`로 산포까지 더하는 엄밀 병합 | **§3.2 기각안.** 파편 트랙은 "같은 점의 경쟁 가설"이 아니라 "한 차의 다른 부분"이라 위치 산포를 더하면 차체 기하를 **이중계상**한다 | 🟡 [F4] Blom–Bar-Shalom 1988 · [F5] Salmond 1990 · [F6] Runnalls 2007 |
| ⑤ | **공분산 시간 전파** | 등속 모델에서 `σ²(t) = var_x + 2t·cov + t²·var_v` | **§3.3 `σ_d²(t)`**(횡)와 **§3.4 `σ_g²(τ)`**(종). ①이 필요한 **직접적 이유** — 이 항이 빠지면 전파가 편향된다 | 🟡 [F7] Bar-Shalom Ch.5 |
| ⑥ | **DWNA 프로세스 노이즈** | `Q = [[dt⁴/4, dt³/2],[dt³/2, dt²]]·σ_a²` — 가속도를 백색잡음으로 본 등속 모델의 Q | **전제.** 트래커(`obstacle_tracker.cpp:58-68`)가 이미 쓰고 있고, 이것 때문에 `cov`가 매 스텝 **0이 아니게** 된다 | 🟡 [F8] Bar-Shalom §6.3.2, p.274 |
| ⑦ | **밴드 점유 확률** | 가우시안이 구간 `[−b, b]`에 있을 확률 = `Φ((b−μ)/σ) − Φ((−b−μ)/σ)` | **§3.3 `P_lat`/`P_long`.** "상대가 내 라인 밴드를 점유하는가"를 0/1이 아니라 확률로 답한다 | 🟡 [F3] Papoulis–Pillai Ch.4 |
| ⑧ | **블록 대각 독립 → 곱 분해** | 결합가우시안에서 `Σ_ab = 0`이면 두 블록이 독립이라 `P = P_lat·P_long` | **§3.3.** 우리 트래커는 F·Q·H가 전부 (s,vs)/(d,vd) 블록 대각이라 이 분해가 **근사가 아니라 엄밀**하다 | ✅ [F9] Bishop §2.3.2 식 (2.98) |
| ⑨ | **곡선좌표 확률적 위협 판정** | Frenet `d` 밴드 점유 + 종방향 조건을 확률 임계로 판정하는 전체 구조 | **§3.3의 골격.** `d = 0`이 곧 글로벌 라인이므로 "라인 접근"이 `\|d\| ≤ b` 판정으로 환원된다 | 🔴 [F11] Kim et al. 2015 — **수식 근거 금지**, 구조적 선례로만 |
| ⑩ | **확률 슈미트 트리거** | 진입 `p* ≥ p_on`, 유지 `p* > p_off` (`p_off < p_on`) + id 래치 | **§3.3.** 현행 거리 슈미트(1.0/1.2 m)의 확률 공간 버전. 현행에 없는 **횡축 채터링**도 함께 해소 | — (현행 구조 계승) |
| ⑪ | **확률 제약 → 결정론적 등가**(constraint tightening) | `P(g ≥ d_e) ≥ 1−ε ⟺ ĝ − z_ε·σ_g ≥ d_e`, `z_ε = Φ⁻¹(1−ε)` | **§3.4의 핵심.** 현행 `gap − 2.0·√s_var`가 이미 이것의 `τ=0` 특수형이라, 시간 전파로 일반화하는 것이 이 제안이다 | ✅ [F13] Schwarm–Nikolaou 1999 |
| ⑫ | **Certainty equivalence 유지** | 안전 제약만 조이고 추적 목표는 **평균 그대로** 둔다 | **§3.4 설계 규칙 1.** 피드포워드에 z를 적용하면 정상상태 언더슛이 생긴다. [F13]은 반대로 "평균만 대입하고 **안 조이는**" 것이 틀렸다고 명시 | ✅ [F13] · 🟡 [F22] Mayne tube MPC |
| ⑬ | **제동거리 안전 속도 상한** | `v_safe = √(v_lead² + 2a·d)` — 상대가 지금 감속해도 충돌하지 않는 속도 | **§3.4 `v_brake`.** 현행에 이미 있고, 여기에 ⑪의 조인 간격과 상대속도 **하한**을 먹인다 | ✅ [F17] RSS Lemma 2 |
| ⑭ | **안전=하드 / 추적=소프트 분리** | 안전 제약은 완화 불가, 성능 목표만 완화 변수를 허용 | **§3.4 구조.** 우리는 QP가 아니라 `min(feedback, v_brake)` 캡으로 같은 분리를 구현한다(B.5-⑥) | ✅ [F19] Ames CDC 2014 |

🔑 **한 문장 요약**: ①이 있어야 ⑤가 편향 없이 돌고, ⑤가 있어야 ⑦⑧로 **§3.3의 판정 확률**과
⑪⑬으로 **§3.4의 속도 상한**이 나온다. ②③은 그 ①이 유효한 공분산이 되도록 지키는 장치다.

### 1.2 읽을거리 — 필독 / 추가

부록 B.4의 23개 서지 중 리뷰·구현에 실제로 필요한 것만 추렸다. **등급이 낮은 문헌을 수식
근거로 쓰지 말 것** — 부록 B.4의 경고가 그대로 적용된다.

#### 🔴 필독 5편 (이 제안의 수식이 직접 기대는 것)

| 순위 | 문헌 | 왜 필독인가 | 읽을 범위 | 접근 |
|---|---|---|---|---|
| 1 | ✅ **[F13]** Schwarm & Nikolaou, "Chance-constrained model predictive control," *AIChE J.* 45(8):1743–1752, 1999. DOI `10.1002/aic.690450811` | **§3.4 전체의 1차 근거.** 식 (5)–(9)가 우리 `ĝ − z_ε·σ_g ≥ d_e`와 동일 구조이고, 결정적으로 **(10)식 = "평균만 대입하고 조이지 않는 것"을 "is not correct"라고 명시**한다 → 우리가 왜 naive 버전을 안 쓰는지의 근거 | 식 (5)–(11) | 저자 배포 PDF (`chee.uh.edu/…/nikolaou/chanceconstrained.pdf`) |
| 2 | 🟡 **[F7][F8]** Bar-Shalom, Li, Kirubarajan, *Estimation with Applications to Tracking and Navigation*, Wiley 2001 | **트래커 전제와 ⑤⑥의 출처.** Ch.5는 KF 전파(`P⁺ = FPFᵀ + Q`), §6.3.2 p.274는 우리 코드에 그대로 있는 DWNA `Q`. 이걸 모르면 `cov`가 왜 0이 아닌지 설명이 안 된다 | Ch.5 + **§6.3.2, p.274** | 도서 (DOI `10.1002/0471221279.ch5`/`.ch6`) |
| 3 | ✅ **[F17]** Shalev-Shwartz, Shammah, Shashua, "On a Formal Model of Safe and Scalable Self-driving Cars" (RSS), arXiv:1708.06374 | **§3.4 `v_brake`의 결정론적 조상.** Lemma 2의 안전거리가 우리 식과 대응하며, 우리는 그 반응시간 여유 ρ를 확률 조임 `z_ε·σ_g(τ_s)`로 **치환**했다(B.5-⑤) | §3.1 Definition 1, **Lemma 2** | arXiv 무료 |
| 4 | ✅ **[F19]** Ames, Grizzle, Tabuada, "Control barrier function based QPs with application to ACC," *IEEE CDC 2014*, pp. 6271–6278. DOI `10.1109/CDC.2014.7040372` | **§3.4의 구조적 근거.** (HC1) 하드 제약 / (SC1) 소프트 목표 분리와 "no relaxation is used"가 우리 캡 구조의 원형 | (HC1)(SC1)(HC1-CBF), §V-C | 저자 배포 PDF (`ames.caltech.edu/CLF_QP_ACC_final.pdf`) |
| 5 | ✅ **[F9]** Bishop, *Pattern Recognition and Machine Learning*, Springer 2006 | **§3.3 곱 분해(⑧)의 정당화.** 식 (2.98)이 있어야 `P = P_lat·P_long`이 우리 트래커에서 근사가 아님을 주장할 수 있다 | §2.3.2 "Partitioned Gaussians", 식 **(2.98)** | 저자 무료 배포 PDF |

> 📌 필독 5편이면 **§3.3·§3.4의 모든 수식이 해석된다.** 1·3·4·5는 전부 무료 PDF로 접근 가능하고,
> 2만 도서다(해당 페이지만 보면 된다).

#### 🟡 추가 — 목적별

**(a) 구현·튜닝을 직접 맡는다면**
- 🟡 **[F15]** Moser, Schmied, Waschl, del Re, "Flexible Spacing ACC Using Stochastic MPC," *IEEE TCST* 26(1):114–127, 2018. DOI `10.1109/TCST.2017.2658193` — 우리와 **같은 응용**(ACC 간격 chance constraint). ⚠️ 본문 수식 미대조 → 식 번호 인용 금지
- ✅ **[F14]** Moser, 석사논문, JKU Linz, 2015 — §3.5.4.1 "Deterministic equivalent constraints"가 ⑪을 **가장 친절하게** 전개한다. 무료 PDF(`epub.jku.at/download/pdf/379855`). 보조 인용만
- ✅ **[F21]** Hewing & Zeilinger, "SMPC Using Probabilistic Reachable Sets," *IEEE CDC 2018*. arXiv:1805.07145 — 조임을 **집합**으로 보는 관점. 우리 `τ_max`를 나중에 늘릴 때의 이론적 안전망
- ✅ **[F20]** Ames, Xu, Grizzle, Tabuada, *IEEE TAC* 62(8):3861–3876, 2017 — [F19]의 확장본. 식 (44) time-headway 제약이 우리 `trailing_gap`의 시간 모드와 대응

**(b) §3.2 병합·PSD 판단을 재검토한다면**
- 🟡 **[F2]** Horn & Johnson, *Matrix Analysis* 2nd ed., **§7.2** — ②의 근거. ⚠️ Sylvester's criterion은 **PD** 판정이고 우리는 **PSD**라 *모든* 주소행렬식이 필요하다(정리 번호 인용 금지)
- 🟡 **[F1]** Higham 2002, *IMA J. Numer. Anal.* 22(3):329–343 — ⚠️ 제약이 `diag(X)=e`라 **우리 클램프의 직접 출처가 아니다.** 문제 정식화 선례로만
- 🟡 **[F4][F5][F6]** Blom–Bar-Shalom 1988 / Salmond 1990 / Runnalls 2007 — ④(기각안)의 출처. **왜 기각했는지**를 검토하려면 이 셋
- 🟡 **[F3]** Papoulis & Pillai 4th ed. — ⑦(구간확률)·Schwarz 부등식

**(c) 배경·계보로 읽는다면**
- ✅ **[F16]** Charnes & Cooper 1959/1963 — chance-constrained programming의 **기원**. 기원 각주로만(식은 [F13]에서)
- 🟡 **[F12]** Jansson & Gustafsson, *Automatica* 44(9):2347–2351, 2008 — 통계적 위협 판정. 🔴 **DOI 주의: `…/j.automatica.2008.01.016`이 맞다**(흔한 오인용 `…030`은 전혀 다른 논문)
- 🔴 **[F11]** Kim, Jo, Lim, Lee, Sunwoo, *IEEE T-ITS* 16(3):1559–1575, 2015 — ⑨와 **가장 가까운 선행 연구**지만 전문 미접근. **수식 근거 사용 금지**, 구조적 선례로만
- 🔴 **[F23]** Farina et al. 2016 / Mesbah 2016 (서베이) — 개괄용. **단독 인용은 약하다**, 반드시 [F13]/[F21]과 함께
- 🟡 **[F18]** Gipps 1981, *Transp. Res. B* 15(2):105–111 — 안전거리 추종모델의 고전. ⚠️ verbatim은 2차 문헌 **L. Lücken(단독 저자)** arXiv:1902.04927에서 읽은 것이라 원문 대조 필요

**(d) 이 스택의 계보 — 로드맵 판단용** (⚠️ 이번 조사 범위 밖, 수식 미검증)
- **ForzaETH Race Stack**, arXiv:2403.11784 / *J. Field Robotics* 2025 — 우리 cruise trailing·`GapData`의 **직계 조상**. perception–FSM 분리 구조가 §4-B 기각 사유의 근거
- **Predictive Spliner**, arXiv:2410.04868 (IEEE RA-L) — CV 외삽의 한계(T > 1 s)를 상대 라인 GP 학습으로 넘는 **다음 단계**. B.5-③에서 `T ≤ 1.0 s`로 자른 이유와 직결

---

## 2. 배경과 문제

### 2.1 아키텍처 — 판단이 인지 계층에 있다

이 스택의 원칙은 "컨트롤러는 판단하지 않는다 — 판단은 planning이 전담한다"이다
(f1tenth_control에서 요레이트 카운터스티어·데드존 바닥 등을 같은 근거로 제거했다).
같은 원칙을 인지에 적용하면: **검출기는 측정을 발행하고, 상태 전이 술어는 FSM이 소유**해야 한다.

그런데 현재 CRUISE 전이의 실질 결정자는 검출기다:

```
obstacle_detector::isOpponentInterfering()          ← 판단이 여기 있다
  (obstacle_detector_node.cpp:935-994)
        ↓ is_interfering (bool)
state_machine::has_interfering_opponent()           ← FSM은 bool을 중계만 한다
  (state_machine_node.cpp:314-318 — allow && seen && interfering && fresh)
        ↓
STATE_GLOBAL ↔ STATE_CRUISE 전이
```

이관이 깔끔한 근거: **`is_interfering`을 읽는 소비자는 state_machine 하나뿐이다.**
(cruise_controller는 `s_center/vs/s_var` 등 측정 필드만, static_obstacle_map은
Cartesian AABB만 읽는다 — 전 저장소 grep으로 확인.)

### 2.2 기능 — 현행 판정의 한계

현행 `isOpponentInterfering()`은:

| 축 | 현행 | 한계 |
|---|---|---|
| 횡방향 | **현재** d 구간이 ego corridor(`ego_d ± 0.26`)와 겹치는가 | **예측이 없다.** vd로 라인에 접근 중인 상대를 간격이 좁아질 때까지 못 본다. 하드 임계라 corridor 경계에서 프레임 단위 채터링 (거리축만 슈미트, 횡축은 히스테리시스 없음) |
| 종방향 | `rear_gap − closing·1s ≤ 임계` (등속 점예측) | 불확실성 미반영 — `s_var`가 큰 고스트도 확정 상대와 동일 취급 |
| 불확실성 | 없음 | 트래커가 이미 계산하는 `P`를 버린다 |

### 2.3 부수 발견 — 파라미터 불일치 (이 제안이 함께 해소)

`obstacle_detector.yaml:96`의 `interference_distance_m: 1.0`은 코드 기본값(5.0)·문서(5.0 진입/5.5 해제)·
cruise의 `trailing_gap: 5.0`과 어긋나 있다. 이대로면 CRUISE 진입 순간 `gap_error = 1.0 − 5.0 = −4.0`으로
속도 상한이 0 근처로 떨어지고, 1.2 m만 벌어지면 GLOBAL 복귀 → 재가속 → 재접근의
**급정지/재가속 리밋사이클**이 구조적으로 발생한다. 판단이 FSM으로 오면 이 거리 파라미터도
FSM 소유가 되므로, 이관 시 `interference_distance_m: 5.0`으로 정렬한다
(f1tenth_control/AGENTS.md의 "trailing_gap과 정렬" 요구와 일치).

---

## 3. 제안 상세

### 3.0 예상 파일 구조

`(신규)` = 새로 만드는 파일, `(수정)` = 기존 파일 변경. 괄호 안 숫자는 §5의 Phase.

```
2026_IFAC/
├── f110_msgs/
│   └── msg/
│       └── Obstacle.msg ......................................... (수정) P1
│              └ s_vs_cov / d_vd_cov 2필드 추가 (§3.1)
│
├── src/obstacle_detector/            [측정만 발행하는 순수 인지 노드로 축소]
│   ├── src/
│   │   └── obstacle_detector_node.cpp ........................... (수정) P1, P4
│   │          P1: mergeLayer()에 교차공분산 누적 + PSD 클램프 (§3.2)
│   │          P4: isOpponentInterfering() / 간섭 래치 멤버 제거
│   ├── include/obstacle_detector/
│   │   ├── obstacle_tracker.hpp ................................. (수정) P1
│   │   │      clampCrossCovariance() 헬퍼 — Track::P 의미론이 정의된 헤더에 둔다
│   │   │      (헤더 전용 inline이라 f110_msgs를 링크 안 하는 기존 gtest에서도 컴파일된다)
│   │   └── obstacle_detector_node.hpp ........................... (수정) P4
│   │          isOpponentInterfering 선언(:140) + interference_* 멤버(:199-200) 제거
│   ├── config/
│   │   └── obstacle_detector.yaml ............................... (수정) P4
│   │          interference_* 8개 제거 (state_machine.yaml로 이사)
│   ├── test/
│   │   ├── test_obstacle_tracker.cpp ............................ (수정) P1
│   │   │      클램프 헬퍼 유닛 테스트 — gtest 배선이 이미 있어 CMake 수정 0줄
│   │   └── synthetic_opponent_test.py ........................... (수정) P4
│   │          is_interfering assert를 state_machine 쪽 검증으로 이관
│   ├── docs/obstacle_detector_node.md ........................... (수정) P1+P4
│   │          P1: 신규 필드 계약 / P4: is_interfering 제거
│   ├── README.md / README_en.md ................................. (수정) P4
│   └── AGENTS.md ................................................ (수정) P1+P4
│          P1: PSD 불변식 조항 / P4: "must set is_interfering" 삭제
│
├── src/state_machine/                [간섭 판단의 새 소유자]
│   ├── include/state_machine/
│   │   ├── interference_predicate.hpp ........................... (신규) P2
│   │   │      ROS 비의존 순수 함수 — 입출력 구조체 + 판정 API
│   │   └── state_machine_node.hpp ............................... (수정) P2
│   │          술어 상태(래치 id / 파라미터 캐시) 멤버 추가
│   ├── src/
│   │   ├── interference_predicate.cpp ........................... (신규) P2
│   │   │      §3.3 확률 술어 본체 (Φ, 시간 전파, 슈미트)
│   │   └── state_machine_node.cpp ............................... (수정) P2
│   │          has_interfering_opponent()가 interference_source로 분기
│   ├── test/
│   │   └── test_interference_predicate.cpp ...................... (신규) P2
│   ├── config/
│   │   └── state_machine.yaml ................................... (수정) P2
│   │          interference_source 외 파라미터 7개 (§3.3 표)
│   ├── CMakeLists.txt ........................................... (수정) P2
│   │          interference_predicate 라이브러리 + ament_add_gtest 블록 신설
│   │          (현재 이 패키지에는 test 타겟이 없다 — 배선부터 만들어야 함)
│   ├── package.xml .............................................. (수정) P2
│   │          <test_depend>ament_cmake_gtest</test_depend> 추가
│   ├── docs/
│   │   ├── interference_judgment_migration_proposal.md .......... (신규) P0  ← 이 문서
│   │   └── state_machine_node.md ................................ (수정) P2
│   └── AGENTS.md ................................................ (수정) P2
│          "CRUISE 전이 술어는 이 패키지가 소유한다" 명문화
│
└── src/f1tenth_control/              [공분산을 속도 상한으로 소비]
    ├── include/f1tenth_control/
    │   └── cruise_controller.hpp ................................ (수정) P2
    │          CruiseControllerInput에 opponent_vs_variance /
    │          opponent_s_vs_cov, Config에 τ_max / z_v 추가
    ├── control_code/
    │   ├── cruise_controller.cpp ................................ (수정) P2
    │   │      §3.4 σ_g(τ) 전파 + v_opp_lb 조임
    │   └── cruise_controller_node.cpp ........................... (수정) P2
    │          opponent_.vs_var / .s_vs_cov 전달, 신규 파라미터 선언
    ├── config/
    │   └── cruise_controller.yaml ............................... (수정) P2
    │          gap_uncertainty_horizon_max / opp_speed_confidence_z
    ├── test/
    │   └── test_cruise_controller.cpp ........................... (수정) P2
    │          τ_max=0 비트 동일성 + 조임 단조성 케이스 추가
    ├── docs/
    │   └── cruise_controller_node.md ............................ (신규) P2
    │          🔴 AGENTS.md:39가 요구하는데 현재 파일이 없다 (부채 해소)
    └── AGENTS.md ................................................ (수정) P2
           cruise가 소비하는 공분산 필드와 조임 규칙 명시
```

**요약**: 신규 5개 / 수정 22개.

🔴 **P1 파일 수가 2개 → 6개로 늘었다** (§3.5.2의 검증 결과 반영). 클램프 헬퍼를
`obstacle_tracker.hpp`에 두어 **기존 gtest 배선(CMake 수정 0줄)으로 검증 가능**하게 만든
결정과, P1 시점에 필드 계약을 문서화해야 한다는 요구 때문이다. **`f110_msgs/Obstacle.msg`,
`obstacle_detector_node.cpp`, `obstacle_tracker.hpp`, `test_obstacle_tracker.cpp`,
`obstacle_detector_node.md`, `AGENTS.md`** — 그래도 §5 Phase 1의 "거동 변화 **0**"은 유지된다
(아무 소비자도 아직 새 필드를 읽지 않는다).

> ⚠️ `src/f1tenth_control/`은 개발 원본(`~/F1tenth_control`)의 동기화 사본이다. 그쪽 CLAUDE.md
> 규칙대로 **양쪽 diff를 확인한 뒤** 반영하고, `f1up` 시 팀원 변경을 덮어쓰지 않도록 주의한다.

### 3.1 f110_msgs — `Obstacle.msg` 확장

```
# Cross-covariances of the Frenet CV tracker blocks. Together with (s_var, vs_var) and
# (d_var, vd_var) they form valid PSD 2x2 blocks for time-propagation of uncertainty.
float64 s_vs_cov
float64 d_vd_cov
```

- 위치: 기존 `s_var ... vd_var` 블록 바로 아래
- `d_vd_cov`만 필요해도 `s_vs_cov`를 **같이** 넣는다 — 종방향 전파(`σ_s²(t)`)도 같은 편향
  문제를 갖고, 메시지 변경은 비싸므로 두 번 하지 않는다
- 미기록 발행자/구 bag에서는 기본값 0.0 = "교차상관 없음"으로 자연 강등된다

### 3.2 obstacle_detector — `mergeLayer()`에서 채움 + PSD 클램프

값의 원천은 이미 존재한다: 트래커 상태 `[s, vs, d, vd]`의 `Track::P`(public,
`obstacle_tracker.hpp:133`)에 `P(2,3) = cov(d,vd)`가 살아 있고, `predict()`의 DWNA Q가
`0.5·dt³·σ²` 교차항을 매 스텝 주입한다(`obstacle_tracker.cpp:58-71`). 현재는
`mergeLayer()`가 대각만 추출하며 버린다(`obstacle_detector_node.cpp:811-814`).

채움 규칙 (멤버 루프에서 `vs/vd`와 동일한 size-가중 평균 후 클램프):

```cpp
// 멤버 루프 내 (obstacle_detector_node.cpp:800-834)
cov_s_vs += w * t->P(0, 1);
cov_d_vd += w * t->P(2, 3);

// 발행 직전 (:847-850 부근) — 대각은 기존 max 방식 유지, 교차항만 클램프
m.ob.s_vs_cov = std::clamp(cov_s_vs / w_sum,
    -0.99 * std::sqrt(m.ob.s_var * m.ob.vs_var),
    +0.99 * std::sqrt(m.ob.s_var * m.ob.vs_var));
m.ob.d_vd_cov = std::clamp(cov_d_vd / w_sum,
    -0.99 * std::sqrt(m.ob.d_var * m.ob.vd_var),
    +0.99 * std::sqrt(m.ob.d_var * m.ob.vd_var));
```

클램프가 필요한 이유: 대각을 축별 **max**(worst 멤버)로 뽑는 기존 규칙을 유지한 채 교차항을
다른 멤버에서 가져오면 `|cov| > √(d_var·vd_var)`가 되어 **비양정치(invalid) 공분산**이 될 수
있다. Cauchy–Schwarz 상계로 자르면 어떤 조합에서도 2×2 블록이 유효하다. 실전에서 상대차는
보통 트랙 1개로 병합되므로 대부분 그 멤버의 `P(2,3)` 그대로가 된다.

#### 왜 "엄밀한 혼합 모멘트"가 아니라 클램프 절충인가 (검토 기록)

**이 규칙은 통계적으로 엄밀한 병합이 아니다.** 리뷰에서 나온 질문이므로 왜 그 길을 안 갔는지
근거를 남긴다. 엄밀한 종착지는 총공분산 법칙(law of total covariance)이다:

```
cov_mix(d, vd) = Σᵢ wᵢ·[ Pᵢ(2,3) + (dᵢ − d̄)(vdᵢ − v̄d) ] / w_sum
                        └ 멤버 내부 공분산 ┘   └ 멤버 중심들의 산포 ┘
```

평균·분산·교차항을 **전부** 이 형태로 맞추면 결과 2×2 블록은 **구성적으로 PSD**라 클램프가
수학적으로 불필요해진다. 그럼에도 채택하지 않은 이유가 셋이다.

**① 발행 필드 `s_center`/`d_center`는 `d̄` 자리에 쓸 수 없다.** 이 둘은 통계적 가중평균이
아니라 **기하학적 포락선 중점**이다(`:838-843` — `lo/hi`와 `d_left/d_right`는 멤버들의
min/max 극값이다). 임의의 중심 `c`에 대해

```
E[(X − c)(Y − c′)] = cov(X,Y) + (μ̄ − c)(ν̄ − c′)
```

이므로 `c = d_center ≠ d̄`이면 **부호를 예측할 수 없는 편향항**이 교차공분산에 섞인다
(분산이라면 항상 부풀리는 쪽이라 보수적이지만, 교차항은 어느 쪽으로든 틀어진다).
게다가 `has_cartesian`이면 재투영이 이 값들을 덮어쓰고(`:874-885`), 멤버 루프가 끝난 뒤에
계산되므로 누적에 쓰려면 두 번째 패스가 필요하다.
— 다만 이것만이라면 **해결 가능한 문제다**: 올바른 `d̄`는 루프 안에 이미 있다
(`rel`(wrap 처리된 멤버 s, `:802`), `t->d()`, `t->vd()`, 가중치 `w`).

**② 진짜 이유 — 멤버는 "같은 점에 대한 서로 다른 추정"이 아니다.** 혼합 모멘트 공식은
멤버들이 동일한 양의 경쟁 가설일 때 성립하는데, 여기서 멤버는 **한 차의 서로 다른
부분(파편 트랙)**이다. 그래서 산포항이 항별로 다른 성격을 갖는다:

| 산포항 | 정체 | 분산/공분산에 더하면 |
|---|---|---|
| `(dᵢ − d̄)²`, `(sᵢ − s̄)²` (위치) | 차체 **기하** — 이미 `s_start/s_end`, `d_left/d_right`, `size`에 들어 있다 | 🔴 **이중계상.** §3.3 술어가 `b_eff = b + w/2`로 차폭을 이미 반영하는데 σ_d에도 넣으면 같은 정보를 두 번 쓴다 |
| `(vsᵢ − v̄s)²`, `(vdᵢ − v̄d)²` (속도) | 강체의 병진속도는 파편이 공유하므로 이 산포는 **추정 불일치**(+회전 성분) | 🟢 방어 가능 — 파편들이 속도에 동의 못 하면 실제로 더 불확실하다 |
| `(dᵢ − d̄)(vdᵢ − v̄d)` (교차) | **기하 × 추정의 혼합** — CV 트래커가 모델링하지 않는 회전 구조가 섞인다 | ⚠️ 물리적 의미가 불분명 |

즉 "엄밀한" 공식을 그대로 적용하면 **위치 산포에서 기하를 이중계상하며 오히려 틀려진다.**
엄밀성이 이 도메인에서 정확성을 뜻하지 않는 경우다.

**③ 그래서 채택한 절충**: 교차항은 `Pᵢ(2,3)`의 가중평균만 — 즉 **순수 추정 불확실성 성분만**
가져오고 기하 성분은 배제한다. 그 대가로 대각(축별 max)과 교차항(가중평균)의 **출처가 달라져**
PSD가 깨질 수 있고, 클램프는 정확히 그 틈만 막는 **최소 안전망**이다.

**잔여 보수성(허용 가능)**: 클램프가 실제로 걸리는 경우는 드물다 — 대각이 이미 max로
부풀려져 있어 Cauchy–Schwarz 상계가 그만큼 헐겁기 때문이다. 걸리더라도 상관계수를
|ρ| ≤ 0.99로 자르는 것뿐이라 σ_g²(τ)·σ_d²(t)는 여전히 유효하며, 방향은 "불확실성을 조금 덜
믿는" 쪽이다. 그리고 상대차가 트랙 1개로 병합되는 통상적인 경우엔 그 멤버의 `P` 블록이
그대로 나가므로 **위 논의 전체가 무관해진다.**

**선택적 개선 (미착수)**: 위 표의 🟢 행 — 속도 산포항만 `vs_var`/`vd_var`에 더하는 것은
물리적으로 방어 가능하다. 다만 이득은 다중 파편 병합이 얼마나 자주 일어나느냐에 달렸으므로,
**Phase 3 시뮬에서 병합 멤버 수 분포를 먼저 로그로 측정**하고 유의미할 때만 별건으로 다룬다.
(이 제안의 범위 밖 — KISS)

검출기의 `isOpponentInterfering()`은 **과도기 동안 그대로 유지**한다(하위 호환) —
Phase 4에서 제거.

### 3.3 state_machine — 확률적 간섭 술어

FSM은 필요한 입력을 **이미 전부 구독 중**이다: `/opp_obs`(on_opponent, `:407-417`),
`/car_state/frenet/odom`(`ego_s`/`ego_d`를 `:443-444`에서 이미 읽음, `twist.linear.x`=v_s),
`/global_waypoints`(랩 길이 계산 가능). **새 구독 배선이 없다.**

#### 수식

Frenet 기준선이 글로벌 라인(`/global_waypoints`)이므로 **`d = 0`이 곧 내 글로벌 라인**이다.
판정량은 "시평 T 안에 상대가 라인 밴드를 점유하며 내 전방 유효 구간에 있을 확률":

```
[횡]  μ_d(t)  = d_center + vd·t
      σ_d²(t) = d_var + 2t·d_vd_cov + t²·vd_var
      b_eff   = (ego_half_width + lateral_margin) + (d_left − d_right)/2
      P_lat(t) = Φ((b_eff − μ_d)/σ_d) − Φ((−b_eff − μ_d)/σ_d)

[종]  μ_g(t)  = wrapΔ(s_center, ego_s) − span/2 − ego_front_offset + (vs − ego_vs)·t
      σ_g²(t) = s_var + 2t·s_vs_cov + t²·vs_var
      P_long(t) = Φ((D − μ_g)/σ_g) − Φ((0 − μ_g)/σ_g)      (D = interference_distance_m)

[결합] P(t) = P_lat(t) · P_long(t)
      p* = max{ P(t) : t ∈ {0, 0.25, 0.5, 0.75, 1.0} }
```

- **곱 분해는 이 코드베이스에서 근사가 아니라 엄밀하다**: 트래커의 F·Q·H가 전부
  (s,vs)/(d,vd) 블록 대각이고 P 초기값이 대각(`obstacle_tracker.cpp:560`)이라 s-블록과
  d-블록의 교차공분산이 항상 0이다.
- 전방 반 랩 게이트(현행 `:951` 동일): `wrapΔ ∉ (0, L/2)`이면 즉시 false.
- Φ는 `0.5 * std::erfc(-x / std::sqrt(2.0))` — 의존성 추가 없음. grid 5점 × Φ 4회 = FSM 주기에
  무시 가능한 비용.
- CV 횡방향 외삽의 유효 지평이 짧으므로(상대 기동은 일시적, 미래 횡위치는 본질적으로
  바이모달) **T는 1.0 s를 넘기지 않는다** — 현행 `interference_time_horizon_sec`와 동일.

#### 게이트·히스테리시스 (현행 구조 계승)

- **확률 슈미트**: `p* ≥ p_on`에서 진입, 래치 중 `p* > p_off`면 유지 (`p_off < p_on`).
  현행 거리 슈미트(1.0/1.2 m)의 확률 공간 버전이며, 횡축 채터링(§2.2)도 함께 해소된다.
- **id 래치**: 상대가 바뀌면 넓은 임계를 물려받지 않고 `p_on`부터 재판정 (현행 `:986-989` 동일).
- **신선도**: 기존 `opponent_stale_timeout_sec`(0.3 s) 그대로 — stale이면 간섭 없음.
- **고스트 게이트**: `is_visible == false`(예측 전용)인 동안은 **진입 불가, 유지만 허용**.
  진입은 실측 프레임에서만.

#### 파라미터 (`config/state_machine.yaml`에 추가)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `interference_source` | `"detector"` | `detector` = 메시지의 `is_interfering` 사용(구 거동, **착지 기본값**) / `internal` = 위 확률 술어 |
| `interference_distance_m` | 5.0 | 종방향 유효 거리 D. **cruise `trailing_gap`(5.0)과 정렬** (§2.3 불일치 해소) |
| `interference_horizon_sec` | 1.0 | 시평 T (CV 유효 지평 상한) |
| `interference_p_on` / `p_off` | 0.7 / 0.4 | 확률 슈미트 진입/유지 임계 |
| `interference_ego_half_width_m` | 0.16 | corridor 반폭 (검출기에서 이동) |
| `interference_lateral_margin_m` | 0.10 | 횡 여유 (검출기에서 이동) |
| `interference_ego_front_offset_m` | 0.25 | 전면 오프셋 (검출기에서 이동) |

구현은 cruise_controller_lib 패턴을 따른다: **술어를 순수 함수 라이브러리로 분리**해
gtest를 붙인다 (ROS 의존 없는 `interference_predicate.cpp` + `state_machine_node`가 호출).

### 3.4 f1tenth_control(cruise) — 공분산의 시간 전파를 속도 상한에 반영

cruise는 종방향만 다루므로 **s-블록(`s_var`, `vs_var`, `s_vs_cov`)**을 소비한다
(d-블록은 §3.3의 FSM 술어 전용). 설계 원칙은 확률 MPC 문헌의 표준 구조를 따른다:
**"추적은 평균으로, 안전은 조여진 제약으로"** — 확률 제약(chance constraint)을 결정론적
등가로 바꾸는 constraint tightening이며, CBF-QP의 "성능=소프트/안전=하드" 분리와 같은
구조다 (부록 문헌 참고).

현행(`cruise_controller.cpp:26-27`)은 `effective_gap = ĝ − 2.0·√s_var` — **위치 불확실성만,
τ = 0에서만** 반영한다. 속도 추정이 나쁜 상대의 `vs`는 피드포워드와 제동거리 캡에
액면가로 들어간다.

#### 수식

간격 g의 확률 제약 `P(g(τ) ≥ d_e) ≥ 1−ε`의 가우시안 등가는
`ĝ(τ) − z_ε·σ_g(τ) ≥ d_e` (`z_ε = Φ⁻¹(1−ε)`). ego s를 결정론으로 두면
`var(g) = s_var`, `cov(g, vs) = s_vs_cov`가 부호 그대로 보존되므로:

```
σ_g²(τ) = s_var + 2τ·s_vs_cov + τ²·vs_var            (CV 시간 전파)

v_opp_lb = max(0, v̂s − z_v·√vs_var)                  (제동 캡용 상대속도 하한)
τ_s      = clamp((v_ego − v_opp_lb)/a_rel, 0, τ_max)  (상대 제동 소요 시간)

effective_gap = max(0, ĝ − z_ε·σ_g(τ_s))              ← 현행 식의 시간 전파 일반화
v_brake       = √(v_opp_lb² + 2·a_rel·max(0, effective_gap − d_e))
feedback      = v̂s + Kp·e + Ki·∫e + Kd·(v̂s − v_ego)   ← 평균 그대로 (조이지 않는다)
speed_limit   = clamp(min(feedback, v_brake), 0, v_max)
비상정지       = ĝ − z_ε·√s_var ≤ d_e → speed_limit 0   ← τ = 0 유지 (즉시 점유 판정)
```

#### 설계 규칙 4개

1. **조임은 캡(`effective_gap`, `v_opp_lb`)에만, 피드포워드는 평균 그대로.** z를
   피드포워드에도 적용하면 정상상태 언더슛(항상 목표보다 더 뒤에서 따라감)이 생긴다.
   불확실성에 비례해 **평형 간격이 물러나는 것**(effective_gap 경유)까지가 의도된 거동이고,
   현행 `uncertainty_sigma`의 의미와 연속적이다.
2. **τ_s는 제동이 실제로 필요한 시평이다.** 접근이 빠를수록 τ_s가 길어지고 σ_g(τ_s)가
   커져 **캡이 더 일찍 조여진다** — 불확실성 비례 time-headway처럼 거동한다.
3. **§3.2의 PSD 클램프가 이 수식의 전제다.** `|s_vs_cov| ≤ √(s_var·vs_var)`이면
   σ_g²(τ)의 판별식이 ≤ 0이라 **모든 τ에서 σ_g² ≥ 0**이 보장된다. 검출기의 클램프와
   컨트롤러의 전파식은 한 쌍이다.
4. **비상정지는 τ = 0.** 지금 이 순간의 점유 판정이므로 속도 불확실성이 무관하다.

#### 파라미터 (`config/cruise_controller.yaml`)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `uncertainty_sigma` | 2.0 (유지) | z_ε로 재해석 (2.0 ↔ ε ≈ 2.3%). 이름·값 변경 없음 |
| `gap_uncertainty_horizon_max` (τ_max) | **0.0** | 0 = 전파 없음 → **현행과 비트 동일** (착지 기본값). Phase 3에서 1.0 (CV 유효 지평) |
| `opp_speed_confidence_z` (z_v) | **0.0** | 0 = 평균 사용(구 거동). Phase 3에서 1.0 안팎으로 A/B |

구현: `CruiseControllerInput`에 `opponent_vs_variance`/`opponent_s_vs_cov` 추가, 노드가
`opponent_.vs_var`/`opponent_.s_vs_cov`를 전달. 순수 함수 구조가 이미 있으므로
(`cruise_controller_lib`) 기존 `test_cruise_controller`를 확장한다.

---

### 3.5 구현 명세 (Phase 1·2에서 실제로 넣는 코드)

§3.1~§3.4가 **무엇을 왜** 하는지라면, 이 절은 **어떻게**다. 아래 코드는 전부 저장소 실제 파일과
대조해 작성했고, 별도 적대적 검증 패스에서 파일을 다시 읽어 존재하지 않는 API·컴파일 오류·
수학적 허점·기존 테스트 파괴를 찾아 수정한 결과다. 검증에서 나온 blocker 3건은 모두 반영돼 있다.

> ⚠️ 이 절의 코드는 **합의(Phase 0) 전 착지 금지**다. §3.1의 메시지 변경이 전원 동시 리빌드를
> 요구하기 때문이다(§6).

#### 3.5.1 `f110_msgs/msg/Obstacle.msg` 확장

**파일**: `/home/tenmeneat/2026_IFAC/f110_msgs/msg/Obstacle.msg`
(⚠️ 이 저장소의 `f110_msgs`는 **레포 루트**에 있다. `src/` 아래가 아니며, 머신에 있는 다른 5개 사본이나 `install/` 생성본을 고치면 아무 효과가 없다. 특히 `install/f110_msgs/share/f110_msgs/msg/Obstacle.msg`는 이 소스로의 **심볼릭 링크**다 — 워크스페이스가 이미 `--symlink-install`로 빌드되어 있다.)

기존 variance 블록 끝(`float64 vd_var`, 32행)과 `bool is_static`(33행) 사이에 삽입한다.

```diff
 float64 vs
 float64 vd
 float64 s_var
 float64 d_var
 float64 vs_var
 float64 vd_var
+# Cross-covariances of the Frenet constant-velocity tracker blocks. Together with
+# (s_var, vs_var) and (d_var, vd_var) they form valid PSD 2x2 blocks, which lets a consumer
+# propagate the obstacle's positional uncertainty over a prediction horizon:
+#   var_s(t) = s_var + 2*t*s_vs_cov + t^2*vs_var
+# A producer that does not fill them leaves 0.0, which reads as "no cross-correlation" and
+# degrades to the previous diagonal-only propagation.
+float64 s_vs_cov
+float64 d_vd_cov
 bool is_static
 bool is_visible
```

> `.msg` 주석은 이 파일 관례대로 영어. `d_vd_cov`만 필요해도 `s_vs_cov`를 같이 넣는다 — 메시지 ABI 변경은 비싸므로 두 번 하지 않는다.

###### 삽입 위치의 호환성 트레이드오프 (의식적 선택)

평면 CDR에서 **말미 추가**는 구 구독자가 자기가 아는 필드까지 정상 역직렬화하고 잉여 바이트만
무시하므로, 부분 리빌드 사고가 나도 피해가 없다. 반면 여기서 택한 **중간 삽입**은 뒤따르는
`is_static` / `is_visible` / `is_interfering`까지 전부 밀어 **조용한 오독**을 만든다 — 엄밀히 더
위험한 선택이다.

그럼에도 중간 삽입을 유지하는 이유는 variance 블록의 응집(읽는 사람이 6개 필드를 한 덩어리로
본다)이며, **그 전제조건은 §5 Phase 1의 "전원 동시 리빌드" 규칙이다.** 이 규칙을 지킬 수 없는
상황이면 말미 추가로 되돌려야 한다.

---

#### 3.5.2 `obstacle_detector` — `mergeLayer()` 구현

###### (0) 클램프 헬퍼: **별도 함수로 분리**한다 (인라인 아님)

**결정 근거**: 이 패키지에는 gtest 배선이 **이미 있다**(`CMakeLists.txt:69-121`, `ament_cmake_gtest`
+ 4개 타깃). 다만 `mergeLayer()`는 `ObstacleDetectorNode`의 private 멤버이고, 어떤 gtest 타깃도
`obstacle_detector_node.cpp`를 컴파일하지 않으므로 **직접 테스트가 불가능**하다. 헬퍼를
`obstacle_tracker.hpp`(= `Track::P`의 의미론이 정의된 헤더, `:133`)에 두면 **CMakeLists 수정 0줄**로
기존 `test_obstacle_tracker` 타깃에서 검증할 수 있다. 이 타깃은 `global_planning rclcpp` +
`Eigen3::Eigen`만 링크하고 **`f110_msgs`를 링크하지 않지만**, 헤더 전용 `inline` 헬퍼이므로
그대로 컴파일된다. `obstacle_detector_node.cpp`의 익명 namespace(:26-37)에 넣는 대안은 테스트
불가이므로 채택하지 않는다.

**파일**: `/home/tenmeneat/2026_IFAC/src/obstacle_detector/include/obstacle_detector/obstacle_tracker.hpp`

먼저 include 2개 추가 (25-27행 블록). **기존 블록의 알파벳 순서를 유지**해야 하므로
`<cstddef>` **앞**에 넣는다:

```diff
+#include <algorithm>
+#include <cmath>
 #include <cstddef>
 #include <deque>
 #include <vector>
```

그다음 `Track` 구조체 닫는 `};`(171행) 바로 뒤에 헬퍼를 추가한다:

```diff
     double s() const { return x(0); }
     double vs() const { return x(1); }
     double d() const { return x(2); }
     double vd() const { return x(3); }
 };
 
+// Cauchy-Schwarz clip for one 2x2 block of the tracker covariance, applied where a merged
+// block is assembled from mixed sources. mergeLayer() takes the merged diagonal as the
+// per-axis max over members while the cross term is a size-weighted mean, so |cov| may exceed
+// sqrt(var_a * var_b) and the block stops being positive semi-definite. Even a single member is
+// not safe: kalmanUpdate() re-symmetrizes P and clamps only the diagonal non-negative, leaving
+// the off-diagonal without any magnitude guarantee. Clipping the correlation to |rho| <= 0.99
+// keeps the published block usable for uncertainty propagation.
+// Degenerate or non-finite input returns 0.0, which consumers read as "no cross-correlation":
+// a zero variance mathematically forces cov = 0, and neither a NaN nor an infinite value may
+// ever reach the topic. Note std::max(0.0, NaN) is 0.0, so a NaN variance also lands here.
+inline double clampCrossCovariance(double cov, double var_a, double var_b)
+{
+    const double bound = 0.99 * std::sqrt(std::max(0.0, var_a * var_b));
+    if (!std::isfinite(bound) || !std::isfinite(cov))
+    {
+        return 0.0;
+    }
+    return std::clamp(cov, -bound, bound);
+}
+
 // Per-update tracker diagnostics. Association rejection counters count track-detection candidate
```

`bound >= 0`이 구성적으로 보장되므로 `std::clamp(v, lo, hi)`의 `lo <= hi` 전제가 항상 성립한다
(`0.0 < -0.0`은 false이므로 `bound == 0`에서도 UB 없음).

> `cov < 0`이고 `bound == 0.0`인 경우 `std::clamp`는 `lo`인 **`-0.0`을 반환**한다. 수치적으로
> `-0.0 == 0.0`이므로 무해하고 아래 유닛 테스트도 통과하지만, `ros2 topic echo`에 `-0.0`이
> 보이는 것은 **정상 출력**이다.

###### (a) 누적 변수 선언 — `double vd_var = 0.0;`(792행) 뒤

**파일**: `/home/tenmeneat/2026_IFAC/src/obstacle_detector/src/obstacle_detector_node.cpp`

```diff
         double w_sum = 0.0;
         double vs = 0.0;
         double vd = 0.0;
         double s_var = 0.0;
         double vs_var = 0.0;
         double d_var = 0.0;
         double vd_var = 0.0;
+        double cov_s_vs = 0.0;
+        double cov_d_vd = 0.0;
         int id = std::numeric_limits<int>::max();
         bool visible = false;
```

들여쓰기 8칸, 기존 지역 변수와 동일하게 접미 언더스코어 없는 bare snake_case.

###### (b) 멤버 루프 안 누적 — `vd_var = std::max(...)`(814행) 뒤

(a)에서 2줄 추가했으므로 적용 후 기준 **816행** 뒤.

```diff
             const double w = std::max(t->size, 1e-3);  // size-weighted merged velocity
             w_sum += w;
             vs += w * t->vs();
             vd += w * t->vd();
             s_var = std::max(s_var, t->P(0, 0));  // conservative: worst member uncertainty
             vs_var = std::max(vs_var, t->P(1, 1));
             d_var = std::max(d_var, t->P(2, 2));
             vd_var = std::max(vd_var, t->P(3, 3));
+            cov_s_vs += w * t->P(0, 1);  // cross terms: same size weighting as the velocity
+            cov_d_vd += w * t->P(2, 3);
             id = std::min(id, t->id);  // oldest member id -> stable across frames
             visible = visible || t->is_visible;
```

`P(0,1)=cov(s,vs)`, `P(2,3)=cov(d,vd)`는 상태 순서 `[s, vs, d, vd]`(`obstacle_tracker.hpp:132`)와
`predict()`의 `F(0,1)=F(2,3)=dt`로 확정된 배치다. DWNA 프로세스 노이즈 `Q`가 매 predict마다
`0.5·dt³·σ²`를 이 위치에 주입하므로(`obstacle_tracker.cpp:58-71`) 구조적 0이 아니다.

###### (c) `m.ob` 대입부 + Cauchy–Schwarz 클램프 — `m.ob.vd_var = vd_var;`(850행) 뒤

(a)+(b)로 4줄 추가했으므로 적용 후 기준 **854행** 뒤.

```diff
         m.ob.vs = vs / w_sum;
         m.ob.vd = vd / w_sum;
         m.ob.s_var = s_var;
         m.ob.vs_var = vs_var;
         m.ob.d_var = d_var;
         m.ob.vd_var = vd_var;
+        // Size-weighted mean of the member cross terms, clipped against the merged (per-axis
+        // max) diagonal. The two come from different sources, so the clip is what keeps the
+        // published 2x2 blocks positive semi-definite.
+        m.ob.s_vs_cov = clampCrossCovariance(cov_s_vs / w_sum, s_var, vs_var);
+        m.ob.d_vd_cov = clampCrossCovariance(cov_d_vd / w_sum, d_var, vd_var);
         m.ob.is_static = is_static_layer;
         m.ob.is_visible = visible;
```

**⚠️ 순서가 load-bearing이다.** 850행보다 **앞**에 넣으면 안 된다 — 상계를 `m.ob.s_var`에서 읽는
구현(§3.2 초기 스케치가 그랬다)이라면 그 시점엔 아직 기본값 0.0이라 `sqrt(0*0)=0`으로 두 필드가
조용히 0이 된다(컴파일 에러도 런타임 에러도 로그도 없음). 위 코드는 상계를 지역 변수
`s_var/vs_var/d_var/vd_var`에서 읽으므로 834행(멤버 루프 종료) 이후 어디서나 안전하지만, 기존
`m.ob.x_var = std::max(s_var, d_var);`(868행) 선례와 맞추어 대각 대입 바로 뒤에 둔다.

새 `#include`는 필요 없다 — `<algorithm>`(7행)·`<cmath>`(9행) 모두 이미 포함되어 있고
`std::clamp`는 이 파일에서 이미 여러 곳에서 쓰인다. `obstacle_detector_node.hpp`가
`obstacle_tracker.hpp`를 include하므로 헬퍼는 같은 `obstacle_detector` 네임스페이스에서
수식자 없이 보인다.

###### §3.0 / §5 개정 (이 항목이 파일 목록을 바꾼다)

§3.0의 파일 트리는 P1을 "`f110_msgs` + `obstacle_detector` **2개 파일**"로 적었으나, 위 결정으로
P1 대상이 **6개**가 된다. §3.0/§5를 아래로 갱신한다.

| 파일 | Phase | 내용 |
|---|---|---|
| `f110_msgs/msg/Obstacle.msg` | **P1** | 필드 2개 추가 (§3.1) |
| `src/obstacle_detector/src/obstacle_detector_node.cpp` | **P1** | mergeLayer 누적 + 클램프 (§3.2 a·b·c) |
| `src/obstacle_detector/include/obstacle_detector/obstacle_tracker.hpp` | **P1 (신규 추가)** | `clampCrossCovariance()` 헬퍼 |
| `src/obstacle_detector/test/test_obstacle_tracker.cpp` | **P1 (신규 추가)** | 헬퍼 유닛 테스트 2개 |
| `src/obstacle_detector/docs/obstacle_detector_node.md` | **P1** + P4 | P1: 신규 필드 계약 / P4: is_interfering 제거 |
| `src/obstacle_detector/AGENTS.md` | **P1** + P4 | P1: PSD 불변식 조항 / P4: "must set is_interfering" 삭제 |

§5 Phase 1의 "거동 변화 **0**"은 유지된다 — 아무 소비자도 아직 새 필드를 읽지 않는다.

---

##### 엣지 케이스 처리 (항목 4)

`mergeLayer()`가 헬퍼에 넘기는 `var_a`/`var_b`는 **`0.0`으로 초기화된 `std::max` 누산기**
(`:789-792` 선언, `:811-814` 갱신)라는 점이 아래 표의 전제다. 이 사실이 도달 가능성을 크게
줄인다.

| 상황 | 도달 가능성 | 동작 |
|---|---|---|
| `var == 0.0` | **있음.** 컴포넌트의 모든 멤버가 `P(i,i) == 0.0`이면 누산기가 0.0에 머문다. `kalmanUpdate()`의 `P(i,i) = std::max(0.0, P(i,i))`(`obstacle_tracker.cpp:170-173`)가 정확히 0.0에 안착시킬 수 있다 | `bound = 0.0` → 클램프 범위 `[-0.0, +0.0]` → cov가 `±0.0`으로 강제. **Cauchy–Schwarz가 요구하는 수학적 정답**이지 버그가 아니다 |
| `var == +inf` | **있음.** `std::max(0.0, inf) = inf`이므로 누산기를 그대로 통과한다 | `bound = inf` → `!std::isfinite(bound)` 가드 → 0.0 반환 |
| `var < 0.0` | **이 호출 지점에서는 불가능.** 누산기가 0.0에서 시작하는 `std::max`이므로 결과는 항상 `>= 0`이다. (참고: 트랙 자체의 `P(i,i)`는 음수가 될 수 있다 — `predict()`는 대각을 클램프하지 않으므로, 대각만 클램프된 비-PSD 블록이 `P00 + 2·dt·P01 + dt²·P11`로 전파되면 음수가 나온다. LDLT 실패 경로(`:156, :163`)가 필요조건이 아니다) | 헬퍼의 `std::max(0.0, var_a*var_b)`는 **다른 호출자를 위한 방어**이며 유닛 테스트가 그 계약을 고정한다 |
| `var`가 NaN | **헬퍼에 NaN으로 도달하지 않는다.** `std::max(a,b)`는 `(a<b)?b:a`이므로 `std::max(0.0, NaN) == 0.0` — 누산기가 NaN을 조용히 버린다 | 결과적으로 `bound = 0.0` 경로와 동일 (cov = ±0.0) |
| `cov`가 NaN | **있음.** `cov_s_vs += w * t->P(0,1)`은 `+=` 누적이라 NaN이 그대로 전파된다 | `!std::isfinite(cov)` 가드로 0.0 반환. 가드가 없으면 비교가 모두 false가 되어 **NaN이 그대로 `/opp_obs`로 발행**된다 |
| `w_sum == 0.0` | **불가능.** `w = std::max(t->size, 1e-3) >= 1e-3`이고 component는 `if (comp.empty()) { continue; }`(775-778행)로 비어 있지 않음 | 새 가드 불필요. 기존 코드도 845-846행에서 이미 무방비로 나눈다 |
| `t->size`가 NaN | 이론상 가능. `std::max(NaN, 1e-3)`은 NaN을 반환 → `w`, `w_sum`, **기존 `m.ob.vs`/`vd`까지** 전부 오염 | **기존 노출이며 이 변경으로 생긴 것이 아니다.** 새 필드만 `isfinite`로 막고 `vs/vd`는 방치하는 부분 가드를 추가하지 않는다. 신규 필드는 헬퍼 덕분에 0.0으로 강등되고, 기존 `vs/vd`는 종전대로 NaN이 나간다 |
| 신규 spawn 트랙 | 항상. spawn은 `P`를 순수 대각(0.5/4.0/0.5/4.0)으로 초기화(`obstacle_tracker.cpp:560-564`) | 첫 프레임들은 `s_vs_cov = d_vd_cov = 0.0`이 **정상**이다. predict()가 한 번 돌아야 Q의 교차항이 들어간다. 짧은 테스트에서 "항상 0"인 것이 곧 버그는 아니다 |
| 멤버 1개 (통상적인 상대차) | 대부분 | `cov = P(0,1)` 원본 그대로. 0.99 계수는 거의 완전 상관인 블록에서만 물린다 |
| 재투영 블록(876-885행) | `has_cartesian`일 때 | `s_start/s_end/s_center/d_right/d_left/d_center/size`만 덮어쓴다. `*_var`와 신규 cov는 **영향 없음** |

> **⚠️ 확인 필요 (모델링 비일관성, 크래시 아님)**: 재투영 후 발행되는 `s_center/d_center`는
> Cartesian AABB 포락선 중점인 반면 `s_var/vs_var/s_vs_cov`는 여전히 Kalman 중심의 불확실성을
> 기술한다. 평균과 공분산이 같은 점을 가리키지 않는다. 이번 변경이 만든 문제는 아니지만
> 소비자 문서에 남겨야 한다.

###### 다운스트림 (범위 밖 — 확인 결과 **PSD는 깨지지 않는다**)

`local_planning`의 `mergeObstacleVariances()`(`/home/tenmeneat/2026_IFAC/src/local_planning/src/local_planner_node.cpp:136-146`)는
`x_var/y_var/s_var/d_var/vs_var/vd_var`만 필드별로 합치고 신규 교차항은 건드리지 않는다. 초기
검토에서 "여기서 비양정치 블록이 다시 생긴다"고 우려했으나, 코드를 대조한 결과 **그렇지 않다**:

- `auto merged = first;`(`:222`)로 `first`의 교차항이 그대로 승계된다.
- `conservativeVariance()`(`:103-108`)는 입력을 `isfinite && >= 0`으로 정화한 뒤 `max`를 취하므로
  대각은 **증가만** 한다 → `|cov| <= √(a_old·b_old) <= √(a_new·b_new)`가 유지된다.
- 대각이 줄어드는 유일한 경우(비유한/음수 입력이 0.0으로 정화될 때)는 검출기가 이미
  `cov = 0.0`으로 발행한 경우다.
- 게다가 `local_planning`은 `ObstacleArray`를 **발행하지 않는다**(전 저장소 grep 결과 발행자는
  `obstacle_detector` 3개 토픽 + `static_obstacle_map` 1개뿐). 이 병합 결과는 노드 내부에서만 쓰인다.

**실제로 남는 성질**은 PSD 위반이 아니라 **보수적 근사**다: 병합 후에도 교차항이 `first` 것만
남아 두 장애물의 합집합을 더 이상 기술하지 않는다(대각은 커졌는데 상관은 한쪽 것). 블록은
유효하되 상관을 과소평가하는 방향이므로 안전한 열화이며, 이번 범위에서 고치지 않는다.

---

##### 검증 (항목 5)

###### 5-1. gtest — 배선은 **있다**

조사 전제와 달리 `src/obstacle_detector/CMakeLists.txt:69-121`에 `ament_cmake_gtest`와 4개
타깃(`test_obstacle_tracker`, `test_aabb_frenet_projector`, `test_frenet_marker_builder`,
`test_wall_distance_filter`)이 배선되어 있다. 다만:

- **`mergeLayer()` 자체는 gtest로 직접 검증 불가** — `ObstacleDetectorNode`의 private 멤버이고,
  이를 컴파일하려면 `rclcpp::Node` 생성까지 끌어와야 한다. 2-멤버 합성 component 테스트는 새
  gtest 타깃 + friend 선언 또는 노드 인스턴스화 하네스가 필요해 KISS를 위반한다.
- **클램프 헬퍼는 검증 가능** — `obstacle_tracker.hpp`에 두었으므로 기존 `test_obstacle_tracker`
  타깃에서 **CMakeLists 수정 없이** 테스트된다.

**파일**: `/home/tenmeneat/2026_IFAC/src/obstacle_detector/test/test_obstacle_tracker.cpp`
include에 `#include <limits>` 추가 후(현재 `<gtest/gtest.h>`, `<vector>`만 있음), 익명 namespace
안 (`}  // namespace` 앞, 342행)에 삽입한다. 테스트 본문은
`namespace obstacle_detector { namespace { ... } }` 안이므로 헬퍼를 수식자 없이 호출한다.

```cpp
TEST(CrossCovarianceClamp, ClipsToCauchySchwarzBound)
{
    // Multi-member merge: diag comes from member A (per-axis max), cross term from member B,
    // so the raw weighted mean can exceed the bound. sqrt(1.0 * 4.0) = 2.0.
    EXPECT_NEAR(clampCrossCovariance(5.0, 1.0, 4.0), 0.99 * 2.0, 1e-12);
    EXPECT_NEAR(clampCrossCovariance(-5.0, 1.0, 4.0), -0.99 * 2.0, 1e-12);
    // Inside the bound the value passes through untouched (the usual single-member case).
    EXPECT_NEAR(clampCrossCovariance(0.5, 1.0, 4.0), 0.5, 1e-12);
}

TEST(CrossCovarianceClamp, DegenerateInputCollapsesToZero)
{
    // A zero variance mathematically forces cov = 0 (reachable: the merged diagonal is a
    // max-accumulator seeded at 0.0 and every member may hold P(i,i) == 0.0).
    EXPECT_DOUBLE_EQ(clampCrossCovariance(0.3, 0.0, 4.0), 0.0);
    // NaN must never reach the topic. Only the cross term can carry NaN into the helper
    // (the diagonal max-accumulator drops NaN), but the guard covers both operands.
    EXPECT_DOUBLE_EQ(
        clampCrossCovariance(std::numeric_limits<double>::quiet_NaN(), 1.0, 4.0), 0.0);
    EXPECT_DOUBLE_EQ(
        clampCrossCovariance(0.3, std::numeric_limits<double>::quiet_NaN(), 4.0), 0.0);
    // An infinite variance survives the max-accumulator, so it is a real input here.
    EXPECT_DOUBLE_EQ(
        clampCrossCovariance(0.3, std::numeric_limits<double>::infinity(), 4.0), 0.0);
    // Defensive only: mergeLayer() cannot produce a negative variance (max-accumulator seeded
    // at 0.0). This locks the contract for any future caller passing a raw P(i,i).
    EXPECT_DOUBLE_EQ(clampCrossCovariance(0.3, -1.0, 4.0), 0.0);
}
```

실행:

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
colcon test --packages-select obstacle_detector --event-handlers console_direct+
colcon test-result --verbose
```

기존 10개 테스트는 영향받지 않는다 — 헤더에 `inline` 자유 함수와 표준 include 2개를 더할
뿐이고 `Track`/`TrackerParams`/`ObstacleTracker`의 어떤 시그니처도 바꾸지 않는다.

###### 5-2. 빌드 — 메시지 ABI 파괴이므로 부분 빌드 금지

`vd_var`와 `is_static` **사이**에 삽입하므로 직렬화 레이아웃이 바뀐다. DDS 타입 이름은
`f110_msgs/msg/Obstacle` 그대로라 **구 정의로 빌드된 노드는 토픽 매칭에 성공한 뒤 필드가 밀린
쓰레기를 역직렬화**한다(에러 없이). `colcon build --packages-up-to obstacle_detector`로는
부족하다.

```bash
cd ~/2026_IFAC
source /opt/ros/jazzy/setup.zsh
rm -rf build/f110_msgs install/f110_msgs        # 생성 사본이 stale로 남는 것을 방지
colcon build --packages-above f110_msgs --symlink-install
```

`--packages-above`는 지정 패키지 **자신을 포함**하므로 재빌드 대상은 **f110_msgs + 의존 8개 =
9개**다. 의존 8개 목록은 §6 참조.

**생성본 반영 확인 — 반드시 rosidl이 만든 산출물을 본다.**

```bash
# ✗ 하지 말 것: install/.../share/.../Obstacle.msg 는 소스로의 심볼릭 링크라
#   빌드가 실패해도, 아예 돌지 않아도 통과한다 (검증이 되지 않는다).

# ✓ 생성 C++ 헤더
grep -n "s_vs_cov\|d_vd_cov" \
  install/f110_msgs/include/f110_msgs/f110_msgs/msg/detail/obstacle__struct.hpp

# ✓ 생성 Python 바인딩
source install/setup.zsh
python3 -c "from f110_msgs.msg import Obstacle; \
f = Obstacle.get_fields_and_field_types(); \
assert 's_vs_cov' in f and 'd_vd_cov' in f, f; print('ok')"
```

빌드 후 **실행 중인 모든 노드를 재시작**해야 한다. 기존 rosbag도 새 타입으로 잘못
역직렬화된다.

`-Werror`가 없으므로(`CMakeLists.txt:9-11`은 `-Wall -Wextra -Wpedantic`만) 누적 변수만 선언하고
대입을 빠뜨린 반쪽 구현도 **경고만 내고 통과해 0.0을 발행**한다. 종료 코드가 아니라 로그를
확인한다. 기본 이벤트 핸들러에서 컴파일러 경고가 눌리고, 변경이 없으면 colcon이 재빌드를
건너뛰므로 두 가지를 강제한다:

```bash
rm -rf build/obstacle_detector
colcon build --packages-select obstacle_detector --event-handlers console_direct+ 2>&1 \
  | grep -i "unused-variable\|unused-but-set"
```

###### 5-3. 런타임 확인 — 필드 존재 + 불변식

```bash
# (1) 메시지 정의에 필드가 실렸는지
ros2 interface show f110_msgs/msg/Obstacle | grep -n "cov"

# (2) 실제 발행값 육안 확인 (스택 기동 후)
#     /opp_obs 는 상대차가 없거나 ego 오도메트리가 stale이면(:1309-1318) 발행 자체가 억제되어
#     --once 가 영원히 블록되므로 timeout 으로 감싼다.
timeout 10 ros2 topic echo /opp_obs --once
timeout 10 ros2 topic echo /static_obs --once   # mergeLayer는 3회 호출되므로 정적 레이어에도 채워진다
```

**불변식 상시 감시** (스크래치 실행, 커밋 대상 아님). 상계는 소비자 계약(`√(var·var)`)이 아니라
**발행자가 실제로 보장하는 더 강한 계약(`0.99·√(var·var)`)** 으로 조인다 — 그래야 "클램프를 아예
빼먹은 구현"(원시 |ρ| = 0.995 통과)까지 잡힌다:

```bash
cd ~/2026_IFAC && source /opt/ros/jazzy/setup.zsh && source install/setup.zsh
python3 - <<'PY'
import math, rclpy
from f110_msgs.msg import ObstacleArray

BOUND_FACTOR = 0.99   # producer contract, stricter than the Cauchy-Schwarz bound
rclpy.init()
node = rclpy.create_node('cov_invariant_check')
stat = {'seen': 0, 'bad': 0, 'nonzero': 0, 'msgs': 0}

def cb(msg):
    stat['msgs'] += 1
    for o in msg.obstacles:
        stat['seen'] += 1
        for cov, a, b, name in ((o.s_vs_cov, o.s_var, o.vs_var, 's_vs'),
                                (o.d_vd_cov, o.d_var, o.vd_var, 'd_vd')):
            bound = BOUND_FACTOR * math.sqrt(max(0.0, a * b))
            if cov != 0.0:
                stat['nonzero'] += 1
            if not math.isfinite(cov) or abs(cov) > bound + 1e-9:
                stat['bad'] += 1
                print(f'VIOLATION id={o.id} {name}: |{cov:.6g}| > {bound:.6g} '
                      f'(var_a={a:.6g}, var_b={b:.6g})')
    if stat['msgs'] % 100 == 0:
        print(f"[progress] msgs={stat['msgs']} obstacles={stat['seen']} "
              f"violations={stat['bad']} nonzero_cov={stat['nonzero']}")

for topic in ('/opp_obs', '/static_obs', '/confirmed_static_obs'):
    node.create_subscription(ObstacleArray, topic, cb, 10)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    print(f"\nchecked={stat['seen']} violations={stat['bad']} nonzero_cov={stat['nonzero']}")
PY
```

판정 기준:
- `violations == 0` — PSD 불변식 + 0.99 상계 유지 (필수).
- `nonzero_cov > 0` — 배선이 실제로 값을 전달하고 있다는 증거. **0이면 (c)를 850행보다 앞에
  넣었거나 (b) 누적을 빠뜨린 것**을 의심한다(단, spawn 직후 몇 프레임은 정상적으로 0이다 —
  최소 수 초는 돌려야 한다).

멤버 2개 이상 병합 케이스를 실제로 밟게 하려면 `layer_merge_gap_s`/`layer_merge_gap_d`를 일시적으로
키워 파편 트랙이 한 component로 묶이도록 유도한 뒤 위 스크립트를 다시 돌린다. 멤버 1개일 때는
클램프가 사실상 무동작이므로 그 상태의 통과는 클램프에 대한 증거가 되지 못한다.

---

##### 문서 갱신 (root CLAUDE.md 필수 항목)

**`/home/tenmeneat/2026_IFAC/src/obstacle_detector/docs/obstacle_detector_node.md`** (236-237행 문단 뒤):

```diff
 유효한 합집합이 있으면 `has_cartesian=true`와 함께 중심, 경계, 반지름 및 재투영된 Frenet 경계를
 채운다. `x_var/y_var`는 기준 경로 접선으로 공분산을 완전히 회전하기 전까지 보수적으로
 `max(s_var, d_var)`를 양축에 사용한다.
+
+`s_vs_cov`/`d_vd_cov`는 멤버 트랙 `P(0,1)`/`P(2,3)`의 size-가중 평균이며, 병합된 대각(축별
+max)에 대해 Cauchy–Schwarz 상계 `0.99·√(var_a·var_b)`로 잘라 발행한다. 대각은 max, 교차항은
+가중평균으로 **출처가 다르기 때문에** 클램프 없이는 2×2 블록이 비양정치(non-PSD)가 될 수 있다.
+분산이 0이거나 비유한값이면 0.0을 발행하며, 소비자는 이를 "교차상관 정보 없음"으로 읽어 대각만
+쓰는 기존 전파로 강등한다(`-0.0` 출력도 같은 의미의 정상값이다). 갓 생성된 track은 `P`가 순수
+대각이므로 첫 프레임들이 0.0인 것은 정상이다.
+
+주의: 재투영이 일어나면(`has_cartesian=true`) `s_center`/`d_center`는 Cartesian AABB 포락선의
+중점으로 덮어써지는 반면 `s_var`/`vs_var`/`s_vs_cov`는 여전히 Kalman 중심 기준이다. 평균과
+공분산의 기준점이 다르므로, 예측 전파를 쓰는 소비자는 이 불일치를 알고 있어야 한다.
```

**`/home/tenmeneat/2026_IFAC/src/obstacle_detector/AGENTS.md`** — "Runtime and messages" 섹션(19-33행)에 추가:

```diff
 - Use `f110_msgs/msg/ObstacleArray` for `/static_obs`, `/confirmed_static_obs`, and `/opp_obs`;
   do not create a new obstacle message. `/confirmed_static_obs` is the confirmed-only Layer-2
   interface for persistent map consumers; it must not change the existing `/static_obs` contract.
+- Every published obstacle must carry a positive **semi**-definite Frenet covariance: the
+  `(s_var, vs_var, s_vs_cov)` and `(d_var, vd_var, d_vd_cov)` blocks must satisfy
+  `|cov| <= sqrt(var_a * var_b)`, and this node tightens that to `|cov| <= 0.99 * sqrt(var_a *
+  var_b)`. Any merge that mixes sources for the diagonal and the cross term must route the
+  cross term through `clampCrossCovariance()` (`obstacle_tracker.hpp`). A zero or non-finite
+  input must publish 0.0, which consumers read as "no cross-correlation".
```


#### 3.5.3 `state_machine` — 확률 술어 구현

모든 코드를 `/tmp/.../scratchpad/ip/` 에서 `g++ -std=c++17 -Wall -Wextra -Wpedantic` 로 실제 컴파일하고 gtest 11케이스 전부 통과시킨 결과다 (경고 0건).

> 적대적 검증에서 3건의 major 결함을 잡아 반영했다: ① `P_long` 하한 0으로 인한 겹침 구간 이탈,
> ② `d_center`가 edge 중점이라는 잘못된 가정, ③ `p_off = 0.0`에서 래치가 영구 고착되는 슈미트 비교.
> 초안이 ⚠️①·⚠️⑥으로 '확인 필요'에 미뤄 둔 항목 중 실제 결함이었던 것은 여기서 해소했다.

##### 1. `src/state_machine/include/state_machine/interference_predicate.hpp` (신규)

ROS 비의존. `cruise_controller_lib` 패턴대로 **래치를 클래스 내부에 둔다** — 호출자가 래치를 들고 있게 하면 `state_machine_node.cpp`에 상태 멤버 2개와 슈미트 분기가 다시 새어 나와 순수 함수로 뽑은 의미가 사라진다.

```cpp
#pragma once

#include <vector>

namespace state_machine
{

// Tuning of the probabilistic interference predicate. All lengths are metres, all times seconds.
struct InterferenceConfig
{
  double distance_m{5.0};
  double horizon_sec{1.0};
  double p_on{0.7};
  double p_off{0.4};
  double ego_half_width_m{0.16};
  double lateral_margin_m{0.10};
  double ego_front_offset_m{0.25};
};

// Ego pose and motion in global-raceline Frenet coordinates. The lateral band is centred on
// d = 0 (the raceline itself), so ego d is deliberately not an input.
struct InterferenceEgoState
{
  double s{0.0};
  double vs{0.0};
  double track_length{0.0};
};

// ROS-free mirror of the fields this predicate needs from f110_msgs/msg/Obstacle.
// d_center is the projected footprint centre and is NOT assumed to be the midpoint of
// (d_right, d_left) — the producer does not guarantee that. See occupancy_probability().
struct InterferenceObstacle
{
  int id{-1};
  double s_center{0.0};
  double s_start{0.0};
  double s_end{0.0};
  double d_center{0.0};
  double d_left{0.0};
  double d_right{0.0};
  double vs{0.0};
  double vd{0.0};
  double s_var{0.0};
  double d_var{0.0};
  double vs_var{0.0};
  double vd_var{0.0};
  double s_vs_cov{0.0};
  double d_vd_cov{0.0};
  bool is_static{true};
  bool is_visible{true};
};

struct InterferenceResult
{
  bool interfering{false};
  // Peak occupancy probability of the deciding obstacle, or the largest probability observed
  // when no obstacle crossed its threshold. Diagnostics only.
  double probability{0.0};
  int id{-1};
};

// Schmitt-latched interference predicate. The latch lives inside the object, so update() is not
// idempotent: call it exactly once per FSM tick and cache the result.
class InterferencePredicate
{
public:
  explicit InterferencePredicate(const InterferenceConfig & config = InterferenceConfig{});

  void configure(const InterferenceConfig & config);

  InterferenceResult update(
    const InterferenceEgoState & ego,
    const std::vector<InterferenceObstacle> & obstacles);

  // Peak p*(t) over the horizon grid for a single obstacle. Pure: the latch is untouched.
  double occupancy_probability(
    const InterferenceEgoState & ego,
    const InterferenceObstacle & obstacle) const;

  void reset();

private:
  InterferenceConfig config_;
  bool opponent_interference_latched_{false};
  int opponent_interference_id_{-1};
};

}  // namespace state_machine
```

설계 결정 3가지:

- **기본 인자 있는 ctor + `configure()`**: 노드 멤버는 파라미터를 읽기 **전에** 초기화되므로 (`rclcpp::Node` 생성자 본문보다 멤버 초기화가 먼저), 값으로 들고 있으려면 기본 생성이 가능해야 한다. `std::optional<InterferencePredicate>` + `.value()` 산개보다 이쪽이 짧다.
- **`InterferenceObstacle`은 POD만** — `f110_msgs` 타입을 헤더에 노출하면 `${f110_msgs_TARGETS}` 링크가 필요해져 "ROS 비의존" 전제가 깨지고 gtest 타겟도 메시지 패키지를 끌고 와야 한다. msg→struct 변환은 노드가 한다.
- **`InterferenceEgoState`에 `d`가 없다** — 제안서 §3.3의 `P_lat`는 밴드를 **d=0(글로벌 라인)** 기준으로 잡는다. 검출기 원본이 `ego_d ± corridor_half_width`를 쓰던 것과 다른 의도적 변경이다(⚠️② 참조).

##### 2. `src/state_machine/src/interference_predicate.cpp` (신규)

```cpp
#include "state_machine/interference_predicate.hpp"

#include <algorithm>
#include <cmath>

namespace state_machine
{
namespace
{

// Fractions of the horizon at which P(t) is sampled. Five points keep the cost negligible at the
// FSM rate while still catching a mid-horizon crossing that neither endpoint sees.
constexpr double kHorizonGrid[] = {0.0, 0.25, 0.5, 0.75, 1.0};

// Below this the Gaussian is treated as a point mass instead of dividing by sigma.
constexpr double kSigmaEpsilon = 1e-9;

double standard_normal_cdf(double x)
{
  return 0.5 * std::erfc(-x / std::sqrt(2.0));
}

// P(lower <= X <= upper) for X ~ N(mean, sigma^2). sigma == 0 degenerates to the step-function
// limit, which is what every zero-variance (deterministic) input must reproduce. upper == lower
// is a legal degenerate band (all half-widths zero), so it is admitted, not rejected.
double interval_probability(double lower, double upper, double mean, double sigma)
{
  if (!std::isfinite(mean) || !(upper >= lower)) {
    return 0.0;
  }
  if (sigma <= kSigmaEpsilon) {
    return (mean >= lower && mean <= upper) ? 1.0 : 0.0;
  }
  const double probability =
    standard_normal_cdf((upper - mean) / sigma) - standard_normal_cdf((lower - mean) / sigma);
  return std::clamp(probability, 0.0, 1.0);
}

// P(X <= upper) for X ~ N(mean, sigma^2). The longitudinal gap is one-sided on purpose: a
// negative gap means the opponent already overlaps the ego, which is the most interfering case
// there is. The legacy detector expressed the same thing as max(0, rear_gap) <= D.
double at_most_probability(double upper, double mean, double sigma)
{
  if (!std::isfinite(mean)) {
    return 0.0;
  }
  if (sigma <= kSigmaEpsilon) {
    return (mean <= upper) ? 1.0 : 0.0;
  }
  return std::clamp(standard_normal_cdf((upper - mean) / sigma), 0.0, 1.0);
}

// Wrap-aware forward distance on a closed track, always in [0, track_length).
// state_machine_node.cpp has an equivalent helper, but it lives in an anonymous namespace and
// cannot be linked from here; three duplicated lines beat exporting it.
double forward_s_distance(double from, double to, double track_length)
{
  const double delta = std::fmod(to - from, track_length);
  return delta < 0.0 ? delta + track_length : delta;
}

// Variance of a constant-velocity state propagated by t, floored at zero so a non-finite or
// non-PSD input degrades to "deterministic" instead of producing NaN.
double propagated_variance(double var, double cross_cov, double rate_var, double t)
{
  return std::max(0.0, var + 2.0 * t * cross_cov + t * t * rate_var);
}

}  // namespace

InterferencePredicate::InterferencePredicate(const InterferenceConfig & config)
{
  configure(config);
}

void InterferencePredicate::configure(const InterferenceConfig & config)
{
  config_ = config;
  config_.distance_m = std::max(0.0, config_.distance_m);
  config_.horizon_sec = std::max(0.0, config_.horizon_sec);
  config_.p_on = std::clamp(config_.p_on, 0.0, 1.0);
  config_.p_off = std::clamp(config_.p_off, 0.0, config_.p_on);
  config_.ego_half_width_m = std::max(0.0, config_.ego_half_width_m);
  config_.lateral_margin_m = std::max(0.0, config_.lateral_margin_m);
  config_.ego_front_offset_m = std::max(0.0, config_.ego_front_offset_m);
  reset();
}

void InterferencePredicate::reset()
{
  opponent_interference_latched_ = false;
  opponent_interference_id_ = -1;
}

double InterferencePredicate::occupancy_probability(
  const InterferenceEgoState & ego,
  const InterferenceObstacle & obstacle) const
{
  const double track_length = ego.track_length;
  if (!std::isfinite(track_length) || !(track_length > 0.0) || !std::isfinite(ego.s) ||
    !std::isfinite(ego.vs))
  {
    return 0.0;
  }
  if (!std::isfinite(obstacle.s_center) || !std::isfinite(obstacle.d_center) ||
    !std::isfinite(obstacle.d_left) || !std::isfinite(obstacle.d_right) ||
    !std::isfinite(obstacle.vs) || !std::isfinite(obstacle.vd))
  {
    return 0.0;
  }

  // Only the forward half-lap counts as "ahead": on a closed track the wrap would otherwise rank
  // an opponent just behind the ego as L - eps ahead.
  const double center_ahead = forward_s_distance(ego.s, obstacle.s_center, track_length);
  if (!(center_ahead > 0.0) || center_ahead >= 0.5 * track_length) {
    return 0.0;
  }

  double longitudinal_span = forward_s_distance(obstacle.s_start, obstacle.s_end, track_length);
  if (!std::isfinite(longitudinal_span) || longitudinal_span > 0.5 * track_length) {
    longitudinal_span = 0.0;
  }

  // Lateral band expressed on the obstacle's *centre*, which is the random variable. The
  // footprint occupies [d_center + (d_right - d_center), d_center + (d_left - d_center)] and the
  // ego corridor is [-base, base]; the two overlap exactly when the centre lies inside
  // [-base - (left - d_center), base - (right - d_center)]. This reproduces the legacy interval
  // overlap test verbatim (obstacle_detector_node.cpp:971-978). A symmetric Minkowski band
  // |d_center| <= base + halfwidth would only be equivalent if d_center were the midpoint of the
  // edges, and it is not: aabb_frenet_projector.cpp:279-289 replaces the raceline-facing edge
  // with the exact curve-to-AABB distance. d_left/d_right are not ordered by the producer either,
  // hence the min/max normalisation (same as the detector).
  const double base_half_width = config_.ego_half_width_m + config_.lateral_margin_m;
  const double obstacle_left = std::max(obstacle.d_left, obstacle.d_right);
  const double obstacle_right = std::min(obstacle.d_left, obstacle.d_right);
  const double band_lower = -base_half_width - (obstacle_left - obstacle.d_center);
  const double band_upper = base_half_width - (obstacle_right - obstacle.d_center);

  const double gap_now =
    center_ahead - 0.5 * longitudinal_span - config_.ego_front_offset_m;
  const double relative_vs = obstacle.vs - ego.vs;

  double best_probability = 0.0;
  for (const double ratio : kHorizonGrid) {
    const double t = ratio * config_.horizon_sec;

    const double mu_d = obstacle.d_center + obstacle.vd * t;
    const double sigma_d = std::sqrt(
      propagated_variance(obstacle.d_var, obstacle.d_vd_cov, obstacle.vd_var, t));
    const double p_lateral = interval_probability(band_lower, band_upper, mu_d, sigma_d);
    if (!(p_lateral > 0.0)) {
      continue;
    }

    const double mu_g = gap_now + relative_vs * t;
    const double sigma_g = std::sqrt(
      propagated_variance(obstacle.s_var, obstacle.s_vs_cov, obstacle.vs_var, t));
    const double p_longitudinal = at_most_probability(config_.distance_m, mu_g, sigma_g);

    best_probability = std::max(best_probability, p_lateral * p_longitudinal);
  }
  return best_probability;
}

InterferenceResult InterferencePredicate::update(
  const InterferenceEgoState & ego,
  const std::vector<InterferenceObstacle> & obstacles)
{
  InterferenceResult result;
  for (const auto & obstacle : obstacles) {
    if (obstacle.is_static) {
      continue;
    }
    const bool same_latched_opponent =
      opponent_interference_latched_ && opponent_interference_id_ == obstacle.id;
    // Ghost gate: a prediction-only frame may hold an existing latch but never open a new one.
    if (!obstacle.is_visible && !same_latched_opponent) {
      continue;
    }

    const double probability = occupancy_probability(ego, obstacle);
    // Entry is >= p_on and hold is > p_off, exactly as the proposal states. The leading positive
    // test keeps a degenerate p_on = p_off = 0 config from welding the latch shut.
    const bool interfering = probability > 0.0 &&
      (same_latched_opponent ? probability > config_.p_off : probability >= config_.p_on);

    if (interfering && (!result.interfering || probability > result.probability)) {
      result.interfering = true;
      result.probability = probability;
      result.id = obstacle.id;
    } else if (!result.interfering && probability > result.probability) {
      result.probability = probability;
    }
  }

  // An empty or fully rejected frame releases the latch, so the next opponent is re-judged from
  // p_on instead of inheriting the wider hold threshold.
  opponent_interference_latched_ = result.interfering;
  opponent_interference_id_ = result.interfering ? result.id : -1;
  return result;
}

}  // namespace state_machine
```

###### 수식 → 코드 대응

| 제안서 수식 | 구현 | 비고 |
|---|---|---|
| `Φ(x)` | `standard_normal_cdf` = `0.5 * erfc(-x/√2)` | — |
| `σ_d²(t) = d_var + 2t·d_vd_cov + t²·vd_var` | `propagated_variance(d_var, d_vd_cov, vd_var, t)`, `max(0,·)`로 바닥 | — |
| `σ_g²(t) = s_var + 2t·s_vs_cov + t²·vs_var` | 같은 헬퍼 재사용 | — |
| `b_eff = (w_ego + margin) + (d_left − d_right)/2` | **비대칭 밴드로 정정**: `[−base − (left − d_center), base − (right − d_center)]` | 🔺 제안서 수식 개정 필요. `d_center`는 edge 중점이 아니다 (`aabb_frenet_projector.cpp:266-267, 279-289`) |
| `μ_g = wrapΔ(s_center, ego_s) − span/2 − offset + (vs − ego_vs)t` | `gap_now + relative_vs * t` | — |
| `P_long = Φ((D−μ_g)/σ) − Φ((0−μ_g)/σ)` | **단측으로 정정**: `at_most_probability(D, μ_g, σ_g)` = `Φ((D−μ_g)/σ_g)` | 🔺 제안서 수식 개정 필요. 겹침(μ_g<0)은 간섭 최대 상태이지 0이 아니다 |
| `p* = max_t P_lat·P_long` | `best_probability`, `P_lat == 0`이면 `P_long` 계산 생략 | — |

**밴드 유도(1줄 증명)**: 장애물 실체는 중심 `d`에 대해 `[d + (right − d_center), d + (left − d_center)]`를 점유한다. ego corridor `[−base, base]`와 겹칠 조건은 `d + (left − d_center) ≥ −base` 그리고 `d + (right − d_center) ≤ base` 이므로, `d ∈ [−base − (left − d_center), base − (right − d_center)]`. `d_center`가 edge 중점이면 이 구간은 정확히 `±b_eff`로 축약된다.

**분산 0 degenerate**: `sigma <= 1e-9`이면 `interval_probability`/`at_most_probability`가 지시함수로 떨어진다. `0/0` NaN을 만들지 않고, 결정론적 입력에서 구 기하 판정과 **비트 단위로 같은 결과**를 낸다. 비유한 분산이 들어오면 `std::max(0.0, NaN)`이 `0.0`을 돌려주므로(표준 `max`는 `(a<b)?b:a`) 자동으로 degenerate 경로로 강등된다.

##### 3. `src/state_machine/` 노드 diff

###### 3-1. `include/state_machine/state_machine_node.hpp`

```diff
 #include "nav_msgs/msg/path.hpp"
 #include "rclcpp/rclcpp.hpp"
+#include "state_machine/interference_predicate.hpp"
```

```diff
   bool evaluate_avoid_path_exhausted();
   bool evaluate_stopped_path_clear();
+
+  // Advances the interference latch exactly once per FSM tick and caches the decision, so the
+  // two has_interfering_opponent() readers in publish_state_cycle() cannot double-step it.
+  void update_interference_state();
```

```diff
   std::string invalid_local_path_policy_;
   std::string static_obstacles_topic_;
+  std::string interference_source_;
```

```diff
   double stopped_path_obstacle_lookahead_m_{3.0};
   double stopped_path_ego_half_width_m_{0.16};
+  double interference_distance_m_{5.0};
+  double interference_horizon_sec_{1.0};
+  double interference_p_on_{0.7};
+  double interference_p_off_{0.4};
+  double interference_ego_half_width_m_{0.16};
+  double interference_lateral_margin_m_{0.10};
+  double interference_ego_front_offset_m_{0.25};
```

```diff
   bool stopped_path_clear_latched_{false};
+  bool interference_active_{false};
+  double interference_probability_{0.0};
   rclcpp::Time last_frenet_time_{0, 0, RCL_ROS_TIME};
```

```diff
   f110_msgs::msg::ObstacleArray::SharedPtr static_obstacles_msg_;
+  f110_msgs::msg::ObstacleArray::SharedPtr opponent_obstacles_msg_;
   std::deque<bool> avoid_path_history_;
 
   uint8_t committed_state_{f110_msgs::msg::StateMachine::STATE_GLOBAL};
+  InterferencePredicate interference_predicate_;
```

###### 3-2. `src/state_machine_node.cpp` — 파라미터 (선언/읽기/검증 3단 전부)

```diff
 #include <string>
 #include <utility>
+#include <vector>
```

```diff
   declare_parameter<std::string>("invalid_local_path_policy", "global_fallback");
+  declare_parameter<std::string>("interference_source", "detector");
```

```diff
   declare_parameter<double>("stopped_path_ego_half_width_m", 0.16);
+  declare_parameter<double>("interference_distance_m", 5.0);
+  declare_parameter<double>("interference_horizon_sec", 1.0);
+  declare_parameter<double>("interference_p_on", 0.7);
+  declare_parameter<double>("interference_p_off", 0.4);
+  declare_parameter<double>("interference_ego_half_width_m", 0.16);
+  declare_parameter<double>("interference_lateral_margin_m", 0.10);
+  declare_parameter<double>("interference_ego_front_offset_m", 0.25);
```

```diff
   static_obstacles_topic_ = get_parameter("static_obstacles_topic").as_string();
+  interference_source_ = get_parameter("interference_source").as_string();
```

```diff
   stopped_path_ego_half_width_m_ =
     get_parameter("stopped_path_ego_half_width_m").as_double();
+  interference_distance_m_ = get_parameter("interference_distance_m").as_double();
+  interference_horizon_sec_ = get_parameter("interference_horizon_sec").as_double();
+  interference_p_on_ = get_parameter("interference_p_on").as_double();
+  interference_p_off_ = get_parameter("interference_p_off").as_double();
+  interference_ego_half_width_m_ =
+    get_parameter("interference_ego_half_width_m").as_double();
+  interference_lateral_margin_m_ =
+    get_parameter("interference_lateral_margin_m").as_double();
+  interference_ego_front_offset_m_ =
+    get_parameter("interference_ego_front_offset_m").as_double();
```

검증 블록 — `invalid_local_path_policy` 검증 **뒤**(`state_machine_node.cpp:173-176`)에 이어 붙이고, 그 자리에서 술어를 configure 한다.

```diff
   if (invalid_local_path_policy_ != "global_fallback") {
     throw std::invalid_argument(
             "invalid_local_path_policy currently supports only 'global_fallback'");
   }
+  if (!std::isfinite(interference_distance_m_) || interference_distance_m_ < 0.0 ||
+    !std::isfinite(interference_horizon_sec_) || interference_horizon_sec_ < 0.0 ||
+    !std::isfinite(interference_p_on_) || interference_p_on_ < 0.0 ||
+    interference_p_on_ > 1.0 || !std::isfinite(interference_p_off_) ||
+    interference_p_off_ < 0.0 || interference_p_off_ > interference_p_on_ ||
+    !std::isfinite(interference_ego_half_width_m_) ||
+    interference_ego_half_width_m_ < 0.0 ||
+    !std::isfinite(interference_lateral_margin_m_) ||
+    interference_lateral_margin_m_ < 0.0 ||
+    !std::isfinite(interference_ego_front_offset_m_) ||
+    interference_ego_front_offset_m_ < 0.0)
+  {
+    throw std::invalid_argument("invalid interference predicate parameter value");
+  }
+  if (interference_source_ != "detector" && interference_source_ != "internal") {
+    throw std::invalid_argument(
+            "interference_source supports only 'detector' or 'internal'");
+  }
+
+  InterferenceConfig interference_config;
+  interference_config.distance_m = interference_distance_m_;
+  interference_config.horizon_sec = interference_horizon_sec_;
+  interference_config.p_on = interference_p_on_;
+  interference_config.p_off = interference_p_off_;
+  interference_config.ego_half_width_m = interference_ego_half_width_m_;
+  interference_config.lateral_margin_m = interference_lateral_margin_m_;
+  interference_config.ego_front_offset_m = interference_ego_front_offset_m_;
+  interference_predicate_.configure(interference_config);
```

> `p_on = 0.0`은 검증을 통과하지만, 술어의 `probability > 0.0` 가드가 "확률 0은 절대 간섭이 아니다"를
> 보장하므로 래치가 고착되지 않는다. 노드 검증과 라이브러리 가드가 이중으로 걸린다.

###### 3-3. `has_interfering_opponent()` 분기

기존 함수(`:314-318`)는 **캐시 리더**로 축소하고 (const 유지 — 호출부 2곳 `:614`, `:701`이 모두 `publish_state_cycle()` 안에 있어 비결정적 이중 전진 문제가 구조적으로 사라진다), 실제 판정은 틱당 1회 실행되는 새 함수가 한다.

```diff
 bool StateMachineNode::has_interfering_opponent() const
 {
-  return allow_cruise_transition_ && opponent_seen_ && opponent_interfering_ &&
-         is_fresh(last_opponent_time_, opponent_stale_timeout_sec_);
+  return interference_active_;
+}
+
+void StateMachineNode::update_interference_state()
+{
+  interference_probability_ = 0.0;
+  // opponent_seen_ never returns to false once set; freshness is the real gate. Both gates stay
+  // outermost so allow_cruise_transition_ disables the detector and internal source identically.
+  if (!allow_cruise_transition_ || !opponent_seen_ ||
+    !is_fresh(last_opponent_time_, opponent_stale_timeout_sec_))
+  {
+    interference_predicate_.reset();
+    interference_active_ = false;
+    return;
+  }
+
+  if (interference_source_ == "detector") {
+    interference_active_ = opponent_interfering_;
+    interference_probability_ = opponent_interfering_ ? 1.0 : 0.0;
+    return;
+  }
+
+  if (opponent_obstacles_msg_ == nullptr || !has_fresh_frenet() || !has_valid_global()) {
+    interference_predicate_.reset();
+    interference_active_ = false;
+    return;
+  }
+  const double track_length = track_length_from(*global_wpnts_msg_);
+  if (!(track_length > 0.0)) {
+    interference_predicate_.reset();
+    interference_active_ = false;
+    return;
+  }
+
+  InterferenceEgoState ego;
+  ego.s = frenet_odom_msg_->pose.pose.position.x;
+  ego.vs = frenet_odom_msg_->twist.twist.linear.x;
+  ego.track_length = track_length;
+
+  std::vector<InterferenceObstacle> obstacles;
+  obstacles.reserve(opponent_obstacles_msg_->obstacles.size());
+  for (const auto & obstacle : opponent_obstacles_msg_->obstacles) {
+    if (obstacle.is_actually_a_gap) {
+      continue;
+    }
+    InterferenceObstacle entry;
+    entry.id = obstacle.id;
+    entry.s_center = obstacle.s_center;
+    entry.s_start = obstacle.s_start;
+    entry.s_end = obstacle.s_end;
+    entry.d_center = obstacle.d_center;
+    entry.d_left = obstacle.d_left;
+    entry.d_right = obstacle.d_right;
+    entry.vs = obstacle.vs;
+    entry.vd = obstacle.vd;
+    entry.s_var = obstacle.s_var;
+    entry.d_var = obstacle.d_var;
+    entry.vs_var = obstacle.vs_var;
+    entry.vd_var = obstacle.vd_var;
+    // f110_msgs/Obstacle carries no cross-covariance yet (proposal Phase 1). 0.0 means "no
+    // correlation", which is exactly what an unaware publisher would emit after the field lands.
+    entry.s_vs_cov = 0.0;
+    entry.d_vd_cov = 0.0;
+    entry.is_static = obstacle.is_static;
+    entry.is_visible = obstacle.is_visible;
+    obstacles.push_back(entry);
+  }
+
+  const auto result = interference_predicate_.update(ego, obstacles);
+  interference_active_ = result.interfering;
+  interference_probability_ = result.probability;
 }
```

`track_length_from()`은 같은 TU의 익명 namespace(`:33-41`)에 있으므로 그대로 호출 가능하고, `has_valid_global()` 선행 확인으로 `global_wpnts_msg_` 널 역참조와 `fmod(x, 0.0)` NaN을 둘 다 막는다. `frenet_odom_msg_->twist.twist.linear.x`가 `v_s`인 것은 발행자에서 확인됨 (`global_planning/src/frenet_odom_node.cpp:351`).

###### 3-4. `on_opponent()` — 메시지 보관 1줄

detector 경로의 bool 추출은 **그대로 둔다**. `interference_source`를 되돌렸을 때 구 거동이 그대로 살아 있어야 하고, 빈 배열이 와도 `last_opponent_time_`은 갱신된다는 "검출기 살아있음 + 상대 없음" 구분도 유지된다.

```diff
   if (msg == nullptr) {
     return;
   }
   opponent_seen_ = true;
+  opponent_obstacles_msg_ = msg;
   opponent_interfering_ = std::any_of(
```

###### 3-5. `publish_state_cycle()` — 틱당 1회 평가

```diff
 void StateMachineNode::publish_state_cycle()
 {
+  update_interference_state();
   const bool global_ready = has_valid_global();
```

```diff
-      "FSM inputs: global=%s frenet=%s avoid_wpnts=%s opp_obs=%s.",
+      "FSM inputs: global=%s frenet=%s avoid_wpnts=%s opp_obs=%s (p*=%.2f).",
       global_ready ? "true" : "false",
       frenet_ready ? "true" : "false",
       avoid_ready ? "true" : "false",
-      opponent_seen_ ? (has_interfering_opponent() ? "interfering" : "clear") : "unseen");
+      opponent_seen_ ? (has_interfering_opponent() ? "interfering" : "clear") : "unseen",
+      interference_probability_);
```

##### 4. `src/state_machine/CMakeLists.txt` diff

`cruise_controller_lib`(`f1tenth_control/CMakeLists.txt:32-39`) 선례를 그대로 따른다: **STATIC 명시, 설치하지 않음**. 헤더는 기존 `install(DIRECTORY include/ ...)`가 이미 설치하고, EXPORT 세트가 없어 다운스트림 `find_package` 대상도 아니다.

```diff
 find_package(f110_msgs REQUIRED)
 
+add_library(interference_predicate_lib STATIC src/interference_predicate.cpp)
+target_include_directories(interference_predicate_lib
+  PUBLIC
+    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
+    $<INSTALL_INTERFACE:include>
+)
+
 add_executable(state_machine_node src/state_machine_node.cpp)
```

```diff
 target_link_libraries(state_machine_node
+  interference_predicate_lib
   rclcpp::rclcpp
```

```diff
   DESTINATION share/${PROJECT_NAME}
 )
 
+if(BUILD_TESTING)
+  find_package(ament_cmake_gtest REQUIRED)
+  ament_add_gtest(test_interference_predicate test/test_interference_predicate.cpp)
+  if(TARGET test_interference_predicate)
+    target_link_libraries(test_interference_predicate interference_predicate_lib)
+  endif()
+endif()
+
 ament_package()
```

##### 5. `src/state_machine/package.xml` diff

```diff
   <depend>f110_msgs</depend>
 
+  <test_depend>ament_cmake_gtest</test_depend>
+
   <export>
```

`ament_cmake_gtest`가 없으면 `colcon test`가 아니라 **`colcon build` 단계의 `find_package`에서** 죽는다 (`BUILD_TESTING`이 기본 ON).

##### 6. `src/state_machine/config/state_machine.yaml` 추가분

기존 `:20` 주석은 detector 동작을 단정하고 있으므로 함께 고친다.

```diff
-    # AVOID는 모든 상태에서 최우선입니다. CRUISE는 /opp_obs의 is_interfering=true로 진입합니다.
+    # AVOID는 모든 상태에서 최우선입니다. CRUISE 진입 근거는 interference_source가 정합니다.
     allow_avoid_transition: true
     allow_cruise_transition: true
```

`opponent_stale_timeout_sec`(`:31`) 뒤에 블록을 추가한다 (들여쓰기 4칸).

```yaml
    # 마지막 /opp_obs가 이 시간보다 오래되면 간섭 없음으로 보고 CRUISE에서 GLOBAL로 복귀합니다.
    opponent_stale_timeout_sec: 0.3

    # CRUISE 간섭 판단의 출처입니다.
    #   "detector" = obstacle_detector가 채운 /opp_obs의 is_interfering을 그대로 중계 (구 거동)
    #   "internal" = 아래 파라미터로 이 노드가 직접 확률 술어를 평가
    # 파라미터는 기동 시 1회만 읽으므로 값을 바꾼 뒤 노드를 재시작해야 합니다.
    interference_source: "detector"

    # 종방향 유효 거리 D. cruise_controller의 trailing_gap(5.0)과 정렬한 값이며,
    # 검출기 쪽 구값 1.0보다 훨씬 커서 CRUISE가 더 일찍 걸립니다.
    # 판정은 "갭 ≤ D일 확률"이므로 이미 겹친(갭 음수) 상대차도 간섭으로 셉니다.
    interference_distance_m: 5.0
    # 등속 외삽의 유효 지평입니다. 횡방향 CV 예측이 금방 무의미해지므로 1.0초를 넘기지 않습니다.
    interference_horizon_sec: 1.0
    # 확률 슈미트: p*가 p_on 이상이면 진입, 같은 상대차를 물고 있는 동안은 p_off 초과까지 유지합니다.
    interference_p_on: 0.7
    interference_p_off: 0.4
    # 글로벌 라인 corridor 반폭 = ego_half_width + lateral_margin.
    # 상대차 폭은 발행된 d_left/d_right/d_center에서 그대로 유도하므로 별도 값이 없습니다.
    interference_ego_half_width_m: 0.16
    interference_lateral_margin_m: 0.10
    # 차량 기준점에서 앞범퍼까지의 오프셋이며 종방향 갭에서 뺍니다.
    interference_ego_front_offset_m: 0.25
```

##### 7. `src/state_machine/test/test_interference_predicate.cpp` (신규, 11케이스)

```cpp
#include <gtest/gtest.h>

#include <vector>

#include "state_machine/interference_predicate.hpp"

namespace
{

state_machine::InterferenceConfig defaultConfig()
{
  state_machine::InterferenceConfig config;
  config.distance_m = 5.0;
  config.horizon_sec = 1.0;
  config.p_on = 0.7;
  config.p_off = 0.4;
  config.ego_half_width_m = 0.16;
  config.lateral_margin_m = 0.10;
  config.ego_front_offset_m = 0.25;
  return config;
}

// Moves the obstacle laterally while keeping its edges consistent with its centre. The detector
// derives d_left/d_right and d_center from the same Cartesian AABB, so a fixture that shifts
// d_center alone would describe an obstacle that cannot be produced.
void placeLaterally(
  state_machine::InterferenceObstacle & obstacle, double d_center, double half_width)
{
  obstacle.d_center = d_center;
  obstacle.d_left = d_center + half_width;
  obstacle.d_right = d_center - half_width;
}

// Deterministic opponent centred on the raceline: every variance is zero, so the predicate is
// expected to collapse to the legacy geometric test.
state_machine::InterferenceObstacle opponentAt(double s_center, double half_span)
{
  state_machine::InterferenceObstacle obstacle;
  obstacle.id = 7;
  obstacle.s_center = s_center;
  obstacle.s_start = s_center - half_span;
  obstacle.s_end = s_center + half_span;
  placeLaterally(obstacle, 0.0, 0.15);
  obstacle.is_static = false;
  obstacle.is_visible = true;
  return obstacle;
}

state_machine::InterferenceEgoState egoAt(double s, double vs, double track_length)
{
  state_machine::InterferenceEgoState ego;
  ego.s = s;
  ego.vs = vs;
  ego.track_length = track_length;
  return ego;
}

}  // namespace

// 1. Legacy equivalence: with zero variance the decision must match the detector's
//    "rear gap <= interference_distance_m and lateral overlap" rule.
TEST(InterferencePredicate, MatchesLegacyDecisionWithZeroVariance)
{
  state_machine::InterferencePredicate predicate(defaultConfig());
  const auto ego = egoAt(0.0, 0.0, 100.0);

  // rear gap = 3.0 - 0.2 - 0.25 = 2.55 <= 5.0
  const auto near = predicate.update(ego, {opponentAt(3.0, 0.2)});
  EXPECT_TRUE(near.interfering);
  EXPECT_EQ(near.id, 7);
  EXPECT_DOUBLE_EQ(near.probability, 1.0);

  predicate.reset();

  // rear gap = 20.0 - 0.2 - 0.25 = 19.55 > 5.0
  const auto far = predicate.update(ego, {opponentAt(20.0, 0.2)});
  EXPECT_FALSE(far.interfering);
  EXPECT_DOUBLE_EQ(far.probability, 0.0);
}

// 2. Head-on closing: the opponent is outside D now but the ego closes the gap inside the
//    horizon. The legacy single-shot closure term is replaced by the P(t) grid maximum.
TEST(InterferencePredicate, ClosingSpeedTriggersInsideHorizon)
{
  state_machine::InterferencePredicate predicate(defaultConfig());
  auto opponent = opponentAt(9.0, 0.2);
  opponent.vs = 2.0;

  // rear gap now = 8.55, at t = 1.0 s it becomes 8.55 + (2.0 - 6.0) = 4.55 <= 5.0
  const auto closing = predicate.update(egoAt(0.0, 6.0, 100.0), {opponent});
  EXPECT_TRUE(closing.interfering);

  predicate.reset();

  // Same geometry without a speed difference stays outside D for the whole horizon.
  const auto matched = predicate.update(egoAt(0.0, 2.0, 100.0), {opponent});
  EXPECT_FALSE(matched.interfering);
}

// 3. Lateral approach: the opponent is longitudinally relevant but off the raceline band, and
//    only its lateral velocity brings it inside within the horizon.
TEST(InterferencePredicate, LateralApproachTriggersOnlyWhenBandIsReached)
{
  state_machine::InterferencePredicate predicate(defaultConfig());
  const auto ego = egoAt(0.0, 0.0, 100.0);

  auto approaching = opponentAt(3.0, 0.2);
  placeLaterally(approaching, 1.0, 0.15);
  approaching.vd = -1.0;
  const auto crossing = predicate.update(ego, {approaching});
  EXPECT_TRUE(crossing.interfering);

  predicate.reset();

  auto parallel = approaching;
  parallel.vd = 0.0;
  const auto holding_line = predicate.update(ego, {parallel});
  EXPECT_FALSE(holding_line.interfering);
}

// 4. Ghost gate: a prediction-only frame (is_visible == false) may hold an existing latch but
//    must never open one.
TEST(InterferencePredicate, GhostFrameHoldsButNeverEnters)
{
  state_machine::InterferencePredicate predicate(defaultConfig());
  const auto ego = egoAt(0.0, 0.0, 100.0);

  auto ghost = opponentAt(3.0, 0.2);
  ghost.is_visible = false;
  EXPECT_FALSE(predicate.update(ego, {ghost}).interfering);

  auto measured = opponentAt(3.0, 0.2);
  EXPECT_TRUE(predicate.update(ego, {measured}).interfering);

  // Same id, prediction-only: the latch survives.
  EXPECT_TRUE(predicate.update(ego, {ghost}).interfering);

  // Different id, prediction-only: no inherited latch, so entry is refused.
  auto other_ghost = ghost;
  other_ghost.id = 9;
  EXPECT_FALSE(predicate.update(ego, {other_ghost}).interfering);
}

// 5. Wrap boundary: only the forward half-lap counts as ahead, and a forward opponent across
//    s = 0 must still be seen.
TEST(InterferencePredicate, RespectsForwardHalfLapAcrossTheWrap)
{
  state_machine::InterferencePredicate predicate(defaultConfig());

  auto behind = opponentAt(19.5, 0.0);
  placeLaterally(behind, 0.0, 0.0);
  EXPECT_FALSE(predicate.update(egoAt(1.0, 0.0, 20.0), {behind}).interfering);

  auto ahead = opponentAt(1.0, 0.0);
  placeLaterally(ahead, 0.0, 0.0);
  EXPECT_TRUE(predicate.update(egoAt(19.5, 0.0, 20.0), {ahead}).interfering);
}

// 6. Degenerate variance: sigma == 0 must behave as a step function, not as a smoothed CDF.
TEST(InterferencePredicate, ZeroVarianceCollapsesToStepFunction)
{
  auto config = defaultConfig();
  config.ego_front_offset_m = 0.0;
  state_machine::InterferencePredicate predicate(config);
  const auto ego = egoAt(0.0, 0.0, 100.0);

  auto on_boundary = opponentAt(5.0, 0.0);
  placeLaterally(on_boundary, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(predicate.occupancy_probability(ego, on_boundary), 1.0);

  auto past_boundary = on_boundary;
  past_boundary.s_center = 5.0 + 1e-6;
  past_boundary.s_start = past_boundary.s_center;
  past_boundary.s_end = past_boundary.s_center;
  EXPECT_DOUBLE_EQ(predicate.occupancy_probability(ego, past_boundary), 0.0);

  // Lateral edge behaves the same way: the band is +-(0.16 + 0.10) for a point footprint.
  auto lateral_edge = on_boundary;
  placeLaterally(lateral_edge, 0.26, 0.0);
  EXPECT_DOUBLE_EQ(predicate.occupancy_probability(ego, lateral_edge), 1.0);
  placeLaterally(lateral_edge, 0.26 + 1e-6, 0.0);
  EXPECT_DOUBLE_EQ(predicate.occupancy_probability(ego, lateral_edge), 0.0);
}

// 7. Probability Schmitt trigger: p* = 0.5 holds a latch (p_off = 0.4) but cannot open one
//    (p_on = 0.7).
TEST(InterferencePredicate, ProbabilitySchmittHoldsBetweenThresholds)
{
  state_machine::InterferencePredicate predicate(defaultConfig());
  const auto ego = egoAt(0.0, 0.0, 100.0);

  auto certain = opponentAt(3.0, 0.0);
  placeLaterally(certain, 0.0, 0.0);
  EXPECT_TRUE(predicate.update(ego, {certain}).interfering);

  // Mean sits exactly on the band edge with a small sigma, so P_lat ~= 0.5 and P_long == 1.
  auto marginal = certain;
  placeLaterally(marginal, 0.26, 0.0);
  marginal.d_var = 0.0025;
  const auto held = predicate.update(ego, {marginal});
  EXPECT_TRUE(held.interfering);
  EXPECT_NEAR(held.probability, 0.5, 1e-6);

  predicate.reset();
  const auto refused = predicate.update(ego, {marginal});
  EXPECT_FALSE(refused.interfering);
  EXPECT_NEAR(refused.probability, 0.5, 1e-6);
}

// 8. An empty frame is "detector alive, nobody there": it must release the latch so the next
//    opponent is judged from p_on again.
TEST(InterferencePredicate, EmptyFrameReleasesLatch)
{
  state_machine::InterferencePredicate predicate(defaultConfig());
  const auto ego = egoAt(0.0, 0.0, 100.0);

  EXPECT_TRUE(predicate.update(ego, {opponentAt(3.0, 0.2)}).interfering);
  EXPECT_FALSE(predicate.update(ego, {}).interfering);

  auto marginal = opponentAt(3.0, 0.0);
  placeLaterally(marginal, 0.26, 0.0);
  marginal.d_var = 0.0025;
  EXPECT_FALSE(predicate.update(ego, {marginal}).interfering);
}

// 9. Regression: an opponent that already overlaps the ego longitudinally (mu_g < 0) is the most
//    interfering case there is. A two-sided P(0 <= gap <= D) would score it 0 and drop CRUISE at
//    the closest possible moment.
TEST(InterferencePredicate, OverlappingOpponentStaysInterfering)
{
  state_machine::InterferencePredicate predicate(defaultConfig());
  const auto ego = egoAt(0.0, 0.0, 100.0);

  // center_ahead = 0.3, span = 0.4, front offset = 0.25 -> mu_g = -0.15.
  auto overlapping = opponentAt(0.3, 0.2);
  const auto result = predicate.update(ego, {overlapping});
  EXPECT_TRUE(result.interfering);
  EXPECT_DOUBLE_EQ(result.probability, 1.0);
}

// 10. Regression: d_center is not the midpoint of (d_right, d_left) — the projector replaces the
//     raceline-facing edge with the exact curve-to-AABB distance. The decision must follow the
//     real footprint, not a symmetric band around |d_center|.
TEST(InterferencePredicate, AsymmetricEdgesFollowTheActualFootprint)
{
  state_machine::InterferencePredicate predicate(defaultConfig());
  const auto ego = egoAt(0.0, 0.0, 100.0);

  // Footprint touches the raceline (d_right = 0.0) while its centre projects at 0.55. A
  // Minkowski band (b_eff = 0.26 + 0.10 = 0.36 < 0.55) would have missed it.
  auto touching = opponentAt(3.0, 0.2);
  touching.d_center = 0.55;
  touching.d_right = 0.0;
  touching.d_left = 0.2;
  EXPECT_DOUBLE_EQ(predicate.occupancy_probability(ego, touching), 1.0);

  // Same centre, footprint entirely outside the corridor: no overlap either way.
  auto clear = touching;
  clear.d_right = 0.30;
  clear.d_left = 0.50;
  EXPECT_DOUBLE_EQ(predicate.occupancy_probability(ego, clear), 0.0);
}

// 11. Degenerate thresholds must not weld the latch shut: p* = 0 is never interference, and the
//     hold test is strict so p_off = 0.0 still releases.
TEST(InterferencePredicate, ZeroProbabilityNeverInterferes)
{
  auto config = defaultConfig();
  config.p_on = 0.0;
  config.p_off = 0.0;
  state_machine::InterferencePredicate predicate(config);
  const auto ego = egoAt(0.0, 0.0, 100.0);

  EXPECT_FALSE(predicate.update(ego, {opponentAt(20.0, 0.2)}).interfering);
  EXPECT_TRUE(predicate.update(ego, {opponentAt(3.0, 0.2)}).interfering);
  EXPECT_FALSE(predicate.update(ego, {opponentAt(20.0, 0.2)}).interfering);
}
```

실행 결과 (`g++ -std=c++17 -Wall -Wextra -Wpedantic`, 경고 0):

```
[==========] 11 tests from 1 test suite ran. (0 ms total)
[  PASSED  ] 11 tests.
```

##### 8. 문서·규칙 갱신 (CLAUDE.md 요구사항)

**8-1. `src/state_machine/AGENTS.md:31-33` 교체:**

```markdown
  - CRUISE entry evidence is owned by this package and selected by `interference_source`:
    `detector` relays `is_interfering` from a fresh `/opp_obs`, `internal` evaluates
    `interference_predicate.{hpp,cpp}` on the same message. AVOID keeps priority either way.
  - CRUISE selects GLOBAL geometry and returns to GLOBAL on a false predicate, an empty frame,
    or stale `/opp_obs`.
  - `update_interference_state()` is the only place the predicate latch advances. It runs once
    per FSM tick; `has_interfering_opponent()` is a cache reader and must stay side-effect free.
  - The lateral test uses the published `d_center`/`d_left`/`d_right` triple directly. Do not
    assume `d_center` is the midpoint of the edges: the projector replaces the raceline-facing
    edge with the exact curve-to-AABB distance.
```

**8-2. `src/state_machine/AGENTS.md` Package Layout에 1행 추가** (`:21` `Launch entrypoints...` 뒤):

```markdown
- ROS-free unit tests live in `test/` and link the `interference_predicate_lib` static library.
```

**8-3. `docs/state_machine_node.md` §2.1 첫 항목 교체:**

```markdown
- `GLOBAL`: 확인된 `/avoid_waypoints`가 있으면 `AVOID`, 그렇지 않고 간섭 판정이 참이면
  `CRUISE`로 진입합니다. 간섭 판정의 출처는 `interference_source`가 정합니다
  (`detector` = `/opp_obs`의 `is_interfering` 중계, `internal` = 이 노드의 확률 술어).
```

**8-4. §3 토픽 표의 `/opp_obs` 행 교체:**

```markdown
| 구독 | `/opp_obs` | `f110_msgs/msg/ObstacleArray` | Reliable + Volatile, 상대차 Frenet 측정값(간섭 판정 입력) |
```

**8-5. §4 파라미터 표에 8행 추가** (`stopped_path_ego_half_width_m` 행 뒤):

```markdown
| `interference_source` | `detector` | CRUISE 간섭 판정 출처 (`detector` / `internal`) |
| `interference_distance_m` | `5.0` | 종방향 유효 거리 D. "갭 ≤ D일 확률"로 평가 |
| `interference_horizon_sec` | `1.0` | 등속 외삽 지평 T (5점 그리드로 샘플) |
| `interference_p_on` | `0.7` | 확률 슈미트 진입 임계 (`p* ≥ p_on`) |
| `interference_p_off` | `0.4` | 같은 상대차 유지 임계 (`p* > p_off`) |
| `interference_ego_half_width_m` | `0.16` | 글로벌 라인 corridor 반폭의 차체 성분 |
| `interference_lateral_margin_m` | `0.10` | corridor 반폭의 안전 여유 성분 |
| `interference_ego_front_offset_m` | `0.25` | 기준점→앞범퍼 오프셋 (종방향 갭에서 차감) |
```

표 아래 문장에 다음을 덧붙인다:

```markdown
`interference_source`도 기동 시 1회만 읽으므로 시뮬 A/B는 노드 재시작으로만 가능합니다.
`internal` 모드의 횡방향 판정은 `/opp_obs`의 `d_center`/`d_left`/`d_right`가 같은 AABB에서
유도된다는 전제에 서 있으며, 밴드는 글로벌 라인(d=0) 기준입니다.
```

**8-6. 제안서 `docs/interference_judgment_migration_proposal.md` §3.3 수식 개정 (필수):**

- `P_long(t) = Φ((D − μ_g)/σ_g)` 로 정정 (하한 제거). 겹침 구간(`μ_g < 0`)은 간섭 확률 1이며,
  이는 구 검출기의 `max(0, rear_gap) ≤ D`와 같은 의미다.
- `b_eff` 대칭 밴드를 비대칭 밴드로 정정:
  `μ_d ∈ [−base − (d_left − d_center), base − (d_right − d_center)]`, `base = w_ego + margin`.
  `d_center`가 edge 중점이면 기존 `±b_eff`로 축약된다.

##### 9. ⚠️ 확인 필요

**⚠️ 확인 필요 ①: 횡 밴드 중심이 `ego_d`가 아니라 `d = 0`이다.**
제안서 §3.3의 문언("`d=0`이 곧 내 글로벌 라인")을 그대로 따랐고, 이는 검출기 원본(`corridor = ego_d ± half_width`, `obstacle_detector_node.cpp:971-974`)과 **다른 판정**이다. ego가 회피 중이라 `|ego_d|`가 클 때, 신규 술어는 "상대가 **글로벌 라인**을 점유하는가"를 묻고 구 술어는 "상대가 **내 현재 진행선**을 막는가"를 묻는다. CRUISE가 글로벌 기하를 선택한다는 점에서 전자가 일관되지만, AVOID→CRUISE 전이 판정(`resolve_requested_state()` `:641-644`)에는 의미 차이가 있다. 의도 확인 필요.

**⚠️ 확인 필요 ②: `s_vs_cov` / `d_vd_cov`는 지금 항상 0.0이다.**
`f110_msgs/msg/Obstacle.msg`에 두 필드가 아직 없다(Phase 1 미착수 — 파일 직접 확인함). 구조체 필드로만 열어 두고 노드가 0.0을 채운다 = "교차상관 없음". 이 상태에서 `σ_d²(t) = d_var + t²·vd_var`가 되어 제안서가 지적한 편향(공분산 무시 → 분산 과소/과대)이 그대로 남는다. Phase 1 착지 후 노드의 `entry.s_vs_cov = obstacle.s_vs_cov;` 2줄만 바꾸면 되며, 술어와 테스트는 무수정이다.

**⚠️ 확인 필요 ③: `interference_distance_m` 기본값이 1.0 → 5.0으로 5배 커진다.**
제안서 §2.3의 정렬 의도를 따랐지만(`obstacle_detector.yaml:96`의 1.0 vs `cruise_controller.yaml:14`의 `trailing_gap: 5.0`), 이건 `interference_source: "internal"`로 바꾸는 순간 **CRUISE가 훨씬 일찍 걸린다**는 뜻이다. 게다가 이번 수정으로 `P_long`이 단측이 되어 겹침 구간까지 포함하므로 CRUISE 점유 구간은 초안보다 더 넓어진다. 시뮬 A/B에서 CRUISE 점유율을 먼저 측정할 것. 반대로 `detector` 모드에서는 이 값이 전혀 쓰이지 않으므로 착지 시점 거동은 무변경이다.

**⚠️ 확인 필요 ④: 노드 전체 빌드는 이 환경에서 수행하지 못했다.**
이 개발 머신에는 `/opt/ros/humble`만 있고(`ls /opt/ros` 확인) 타깃은 Jazzy다. 순수 라이브러리 + gtest 11케이스는 g++로 실제 컴파일·실행 검증했으나, `state_machine_node.cpp` 쪽 diff는 정적 대조만 했다(모든 diff 컨텍스트 라인, 멤버명, `is_fresh` 시그니처, `track_length_from` 위치, `f110_msgs/Obstacle` 필드 존재는 파일 대조로 확인 완료). 젯슨에서 아래를 반드시 실행할 것.

```bash
cd ~/2026_IFAC && source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-select state_machine
colcon test --packages-select state_machine && colcon test-result --verbose
```

**⚠️ 확인 필요 ⑤ (사소):** `/opp_obs`에는 원래 gap 엔트리가 오지 않지만(`obstacle_detector_node.cpp:853`이 `is_actually_a_gap = false` 고정), 노드 변환 루프에서 방어적으로 걸러낸다 — 술어 라이브러리에 msg 전용 필드를 넣지 않기 위한 배치다. 또한 현재 `/opp_obs`는 전방 최근접 **1대**만 싣는다(`:1296-1302`). `update()`의 다중 상대차 래치 로직은 향후 확장 대비이며 현 시점에서는 항상 0 또는 1개 원소만 순회한다.


#### 3.5.4 `f1tenth_control`(cruise) — 공분산 조임 구현

##### 착지 요약

| 항목 | 결정 |
|---|---|
| `CruiseControllerInput` 신규 필드 위치 | **`dt` 뒤(구조체 끝)** — 기존 테스트 6곳의 집합 초기화를 **고치지 않는다** |
| `CruiseControllerConfig` 신규 필드 위치 | `uncertainty_sigma` 뒤 (Config는 집합 초기화 사용처가 없어 안전) |
| 기본값 | `gap_uncertainty_horizon_max = 0.0`, `opp_speed_confidence_z = 0.0` → **현행과 비트 동일**(단 아래 §2-3의 isfinite 가드가 있어야 성립) |
| 주석 언어 | `.hpp`/`.cpp`/`test` = **영어**(cruise 파일군 관례), `.yaml`·`docs/` = **한국어** |
| CMakeLists | **수정 불필요** — 단 `docs/` 신설이 **빌드 전제**다(§6 참고) |
| 함께 착지해야 하는 파일 | `docs/cruise_controller_node.md`(신규·한국어), `AGENTS.md`(수정) — 둘 다 루트 `CLAUDE.md` 정책상 필수 |

###### §5 대비 이탈 (먼저 선언한다)

§5는 Phase 1(`f110_msgs` msg 확장 + detector 채움) → Phase 2(FSM + cruise) 순서이고, §3.4 본문도
"노드가 `opponent_.vs_var`/`opponent_.s_vs_cov`를 전달"이라고 쓰여 있다. **이번 착지는 그 순서를
의도적으로 어긴다** — §3.1(msg 확장)을 기다리지 않고 cruise 쪽만 먼저 넣고, `s_vs_cov`는
노드에서 `0.0`을 주입한다. 근거와 대가는 다음과 같다.

- **근거**: msg 확장은 f110_msgs 의존 8개 패키지 전원 재빌드 + ROS 2 타입 해시 변경(구/신 노드가
  `/opp_obs`·`/static_obs`에서 연결 실패, 기존 rosbag 재생 불가)을 수반하는 별건 결정이다.
  컨트롤러 쪽 배선은 그 결정과 독립적으로, 거동 변화 0으로 먼저 착지시킬 수 있다.
- **대가**: §3.1이 들어오기 전까지 전파식의 교차항은 항상 0이므로
  **σ_g²(τ) = s_var + τ²·vs_var (대각 근사)** 로만 동작한다. 조임 방향은 보수적이지만, cov가
  음수인 상대(멀어지는 방향으로 상관)에서는 과도하게 조인다.
- **하드 전제조건**: 따라서 **Phase 3의 `τ_max = 1.0` / `z_v > 0` 활성화는 §3.1 + §3.2가 모두
  착지한 뒤에만 허용한다.** §3.2의 PSD 클램프가 없는 상태에서 τ_max를 켜면 σ_g²이 음수로 내려가
  `std::max(0.0, ·)`에 잘리고, 조임이 **조용히 약해지는** 구간이 생긴다.

---

##### 1. `src/f1tenth_control/include/f1tenth_control/cruise_controller.hpp`

**🔴 필드 추가 위치 결정 — "테스트를 고치지 않는" 쪽을 택했다.**
`test_cruise_controller.cpp`는 6곳(:24, :31, :39, :47, :56, :58)에서
`controller.update({1.5, 1.5, 4.0, 4.0, 0.0, 0.02})` 형태의 **집합 초기화**를 쓴다.
새 필드를 `dt` **앞**에 끼우면 6번째 값 `0.02`가 `opponent_vs_variance`로 들어가고 `dt`는
NSDMI 기본값으로 채워진다. 이 경로를 실제로 컴파일해 확인했다 — g++ 11.4.0 `-Wall -Wextra`에서
**경고가 한 줄도 안 뜨고**(NSDMI가 `-Wmissing-field-initializers`를 억제한다), τ_max=0/z_v=0에서는
테스트도 그대로 통과하므로 **나중에 z_v를 켜는 순간 근거 없이 값이 바뀐다**. 따라서 신규 2필드를
**구조체 맨 끝**에 붙이고 기존 테스트는 **한 줄도 건드리지 않는다**(신규 케이스만 필드별 대입 사용).
구조체를 8필드로 늘린 뒤 기존 6원소 집합 초기화를 `-Wall -Wextra`로 컴파일해 **경고 0**도 실측했다.

반대로 `CruiseControllerConfig`는 `defaultConfig()`(test:8-19)가 필드별 대입이라 어디에
넣어도 안 깨지고 새 필드는 NSDMI 0.0을 상속한다 — 그래서 의미상 인접한
`uncertainty_sigma` 뒤에 둔다.

```diff
--- a/src/f1tenth_control/include/f1tenth_control/cruise_controller.hpp
+++ b/src/f1tenth_control/include/f1tenth_control/cruise_controller.hpp
@@ -17,5 +17,10 @@
   double integral_limit{2.0};
   double uncertainty_sigma{2.0};
+  // Chance-constraint tightening of the longitudinal gap. Both defaults reproduce the legacy
+  // behaviour exactly: a zero horizon collapses the covariance propagation onto tau = 0, and a
+  // zero confidence factor keeps the mean opponent speed in the braking cap.
+  double gap_uncertainty_horizon_max{0.0};
+  double opp_speed_confidence_z{0.0};
   bool allow_acceleration{true};
 };
 
@@ -28,6 +33,11 @@
   double opponent_speed{0.0};
   double opponent_s_variance{0.0};
   double dt{0.02};
+  // Appended after dt on purpose. The existing tests build this struct with aggregate
+  // initialization of the first six members, so a field inserted ahead of dt would silently
+  // shift those values instead of failing to compile (verified: no -Wextra warning either way).
+  double opponent_vs_variance{0.0};
+  double opponent_s_vs_cov{0.0};
 };
 
 struct CruiseControllerOutput
@@ -37,6 +47,8 @@
   double gap_error{0.0};
   double relative_speed{0.0};
   double gap_integral{0.0};
+  double tighten_horizon{0.0};
+  double gap_sigma{0.0};
 };
 
 class CruiseLongitudinalController
```

`tighten_horizon`(τ_s)·`gap_sigma`(σ_g(τ_s))는 진단용이다. `f110_msgs/GapData`는
`header` + `gap_diff`/`vs_diff`/`gap_int`뿐이고, 여기에 필드를 붙이면 **§3.1과 똑같은
전원 재빌드 + 타입 해시 변경 비용을 한 번 더 치르게 된다**. 그래서 이번 범위에서는 GapData를
건드리지 않고 **Output에만 담아 throttle 로그로 노출**한다(§Message Policy).
`CruiseControllerOutput`은 저장소 어디서도 집합 초기화되지 않으므로(전 저장소 grep 확인)
필드 추가가 안전하다.

---

##### 2. `src/f1tenth_control/control_code/cruise_controller.cpp`

###### 2-1. 생성자 (신규 config 새니타이즈)

```diff
@@ -13,3 +13,5 @@
   config_.integral_limit = std::max(0.0, config_.integral_limit);
   config_.uncertainty_sigma = std::max(0.0, config_.uncertainty_sigma);
+  config_.gap_uncertainty_horizon_max = std::max(0.0, config_.gap_uncertainty_horizon_max);
+  config_.opp_speed_confidence_z = std::max(0.0, config_.opp_speed_confidence_z);
 }
```

> `std::clamp(v, lo, hi)`는 `lo > hi`이면 UB다. 위 두 줄이 `gap_uncertainty_horizon_max`를
> 음수·NaN 모두에서 0.0 이상으로 강제하므로, 아래 `std::clamp(·, 0.0, τ_max)`의 사전조건이
> 항상 성립한다. τ_max = 0.0이면 구간이 `[0, 0]`이 되는데 이는 UB가 아니라 정의된 거동이며,
> 의도한 "전파 없음"과 정확히 일치한다.

###### 2-2. `update()` 수정본 전문

```cpp
CruiseControllerOutput CruiseLongitudinalController::update(
  const CruiseControllerInput & input)
{
  CruiseControllerOutput output;
  const double dt = std::clamp(input.dt, 1e-3, 0.2);
  const double ego_speed = std::max(0.0, input.ego_speed);
  const double opponent_speed = std::max(0.0, input.opponent_speed);
  const double s_variance = std::max(0.0, input.opponent_s_variance);

  // A non-finite second-moment input would poison the propagation below: 0.0 * inf is NaN and
  // std::max(0.0, NaN) yields 0.0, which would silently disable the tightening instead of
  // making it stricter. Fall back to "no extra information" explicitly.
  const double vs_variance = std::isfinite(input.opponent_vs_variance) ?
    std::max(0.0, input.opponent_vs_variance) : 0.0;
  const double s_vs_cov = std::isfinite(input.opponent_s_vs_cov) ? input.opponent_s_vs_cov : 0.0;

  // Lower confidence bound of the opponent speed. Used by the caps only; the feedforward below
  // keeps the mean estimate. z_v = 0 makes this max(0.0, opponent_speed - 0.0), and
  // opponent_speed is already non-negative, so the legacy value is reproduced bit for bit.
  const double opponent_speed_lb = std::max(
    0.0, opponent_speed - config_.opp_speed_confidence_z * std::sqrt(vs_variance));

  // Time the opponent would need to brake down to the ego speed. relative_deceleration is
  // clamped to >= 0.01 in the constructor, so this can never divide by zero.
  //
  // The isfinite guard is load bearing, not defensive noise: with both speeds at +inf the
  // difference is NaN, and std::clamp(NaN, 0.0, 0.0) returns NaN rather than 0.0 (neither
  // NaN < lo nor hi < NaN holds, so clamp returns the value untouched). That NaN would flow
  // into sigma_g, collapse it to 0.0 through std::max(0.0, NaN), and silently drop the
  // uncertainty margin that the legacy code always applied. Verified by a differential sweep.
  double tighten_horizon = 0.0;
  const double closing_speed = ego_speed - opponent_speed_lb;
  if (std::isfinite(closing_speed)) {
    tighten_horizon = std::clamp(
      closing_speed / config_.relative_deceleration,
      0.0, config_.gap_uncertainty_horizon_max);
  }
  output.tighten_horizon = tighten_horizon;

  // tau = 0 slice of the propagation, kept separately because the emergency-stop rule below is
  // an occupancy decision about *now* and must not move with the closing speed.
  const double sigma_s = std::sqrt(s_variance);
  const double immediate_gap = std::max(
    0.0, input.gap - config_.uncertainty_sigma * sigma_s);

  // Constant-velocity propagation of the gap variance:
  //   sigma_g^2(tau) = s_var + 2*tau*s_vs_cov + tau^2*vs_var
  // Once §3.2 lands, the detector clamps |s_vs_cov| to 0.99*sqrt(s_var*vs_var), which makes the
  // discriminant strictly negative and the quadratic strictly positive for every tau. Until
  // then the caller hard-wires the cross term to 0.0, and the std::max guard is the only thing
  // standing between a malformed producer and std::sqrt. With gap_uncertainty_horizon_max = 0
  // the horizon is 0, both extra terms are exactly +/-0.0, and sigma_g == sigma_s bit for bit.
  const double sigma_g = std::sqrt(
    std::max(
      0.0,
      s_variance + 2.0 * tighten_horizon * s_vs_cov +
      tighten_horizon * tighten_horizon * vs_variance));
  output.gap_sigma = sigma_g;

  output.effective_gap = std::max(
    0.0, input.gap - config_.uncertainty_sigma * sigma_g);
  output.gap_error = output.effective_gap - std::max(0.0, input.desired_gap);
  output.relative_speed = opponent_speed - ego_speed;

  gap_integral_ = std::clamp(
    gap_integral_ + output.gap_error * dt,
    -config_.integral_limit, config_.integral_limit);
  output.gap_integral = gap_integral_;

  // Emergency stop stays at tau = 0. At a zero horizon immediate_gap and effective_gap are the
  // same expression, so this branch is unchanged from the legacy code.
  if (immediate_gap <= config_.emergency_stop_distance) {
    output.speed_limit = 0.0;
    return output;
  }

  const double feedback_speed =
    opponent_speed +
    config_.proportional_gain * output.gap_error +
    config_.integral_gain * gap_integral_ +
    config_.derivative_gain * output.relative_speed;

  // If the opponent were to brake now, this cap leaves enough relative stopping distance before
  // the emergency boundary. It complements the gap feedback during high closing-speed
  // approaches. Both terms are the tightened ones: the gap propagated over tighten_horizon and
  // the lower bound of the opponent speed, so a poorly observed opponent lowers the cap.
  const double usable_gap =
    std::max(0.0, output.effective_gap - config_.emergency_stop_distance);
  const double braking_speed = std::sqrt(
    opponent_speed_lb * opponent_speed_lb +
    2.0 * config_.relative_deceleration * usable_gap);

  output.speed_limit = std::clamp(
    std::min(feedback_speed, braking_speed), 0.0, config_.maximum_speed);
  if (!config_.allow_acceleration) {
    output.speed_limit = std::min(output.speed_limit, ego_speed);
  }
  return output;
}
```

###### 2-3. τ_max=0 · z_v=0에서 비트 동일성이 성립하는 이유 (코드 상의 4지점)

| 지점 | 신규 코드 | τ_max=0·z_v=0일 때 | 현행 코드 |
|---|---|---|---|
| 상대속도 | `max(0, opponent_speed - z_v*sqrt(vs_var))` | `z_v*x = 0.0` → `max(0, opponent_speed)`, 그런데 `opponent_speed`는 이미 `max(0,·)` 결과라 **항등** | `opponent_speed` |
| τ_s | `isfinite(closing) ? clamp(closing/a_rel, 0, 0) : 0` | 유한이면 `clamp(·,0,0) = 0`, 비유한이면 **명시적으로 0** | (없음 = 0) |
| σ_g | `sqrt(max(0, s_var + 2τ·cov + τ²·vs_var))` | `τ = 0` → `2*0.0*cov = ±0.0`, `0.0*0.0*vs_var = 0.0`, `s_var + (±0.0) + 0.0 == s_var` | `sqrt(max(0, s_var))` |
| 비상정지 | `immediate_gap = max(0, gap - z_ε·sigma_s)` 와 비교 | `sigma_g == sigma_s`이므로 `immediate_gap`과 `effective_gap`이 **같은 식** | `effective_gap`과 비교 |

- **τ_s의 isfinite 가드가 비트 동일성의 필수 조건이다.** 이 가드가 없으면
  `ego_speed = opponent_speed = +inf`에서 `inf - inf = NaN`이 되고,
  `std::clamp(NaN, 0.0, 0.0)`는 0.0이 **아니라** NaN을 반환한다(`NaN < lo` 거짓,
  `hi < NaN` 거짓 → 값을 그대로 반환). 그 NaN이 σ_g²에 들어가면
  `sqrt(max(0.0, NaN)) = 0.0`이 되어 현행의 `sqrt(s_var)`가 통째로 사라진다.
  실측 반례: `gap=10.6312, ego=inf, opp=inf, s_var=1e9, uncertainty_sigma=2`
  → 현행 `effective_gap=0, gap_error=-1.5, gap_integral=-0.03`,
  가드 없는 신규 `effective_gap=10.6312, gap_error=9.13122, gap_integral=0.182624`.
  노드가 `isfinite(obstacle.vs)`·`isfinite(vs)`로 걸러 실전 도달성은 없지만,
  `cruise_controller_lib`는 ROS 비의존 순수 라이브러리이므로 라이브러리 계약으로 막는다.
- 비상정지에서 `max(0.0, ...)`를 **그대로 유지**한 것이 핵심이다. 이걸 빼면 `gap`이 NaN일 때
  현행(`max(0,NaN)=0.0 → 정지`)과 갈린다. 유지하면 τ_max=0에서 두 변수가 문자 그대로 같은
  식이라 분기가 자명하게 불변이다.
- **FMA 융합에 대해**: σ_g 식은 형태가 바뀌었다(`sqrt(max(0, s_var))` →
  `sqrt(max(0, s_var + 2τc + τ²v))`). 그럼에도 안전한 이유는 τ = +0.0에서 추가 항이 정확히
  ±0.0으로 평가되고, `-O3 -march=native -flto`(CMakeLists:12-19, `-ffast-math` 없음)의 FMA
  축약 `fma(2τ, c, s_var)`도 τ=0에서 정확히 `s_var`를 돌려주기 때문이다 — **단 c·v가 유한할
  때만** 성립하며, 그 유한성을 보장하는 것이 위의 두 isfinite 가드다.

###### 2-4. 조이지 **않은** 곳 (설계 규칙 1)

`feedback_speed`의 첫 항 `opponent_speed`와 `output.relative_speed = opponent_speed - ego_speed`는
**평균 v̂s 그대로**다. 여기까지 `opponent_speed_lb`를 적용하면 정상상태 언더슛(항상 목표보다
뒤에서 추종)이 생긴다. 교체 대상은 `braking_speed`의 제곱항 하나뿐이다.

###### 2-5. 조임이 출력에 도달하는 경로 (Phase 3 A/B의 전제)

`usable_gap = max(0, effective_gap - d_e)`가 0으로 바닥을 치면 `braking_speed`는
`opponent_speed_lb`가 된다. 즉 **제동 캡은 z_v=0에서 상대 속도 아래로 절대 내려가지 않는다.**
`EmergencyStopIgnoresPropagationHorizon`에서 상한이 3.0(제동 캡)이 아니라 1.080인 이유가
이것이다 — 조임을 출력으로 실어 나르는 것은 `gap_error`를 통한 **피드백 항**이다.
따라서 `trailing_p_gain > 0`은 τ_max 조임이 유효하기 위한 전제이며, Phase 3 A/B에서
`trailing_p_gain: 0.0`과 τ_max 활성화를 동시에 시험해선 안 된다.

###### 2-6. 랩 경계(wrap)

컨트롤러 안에는 wrap 연산이 없다. `input.gap`은 노드의 `forwardDelta()`가 이미 `[0, L)`로
풀어 넘긴 값이고, 이번 변경은 `forwardDelta()`·`raw_gap` 계산을 건드리지 않는다. τ_s는
속도만으로 계산되고 σ_g 전파는 s에 직접 손대지 않으므로, 랩 경계에서 부호가 뒤집히는 신규
경로는 없다.

---

##### 3. `src/f1tenth_control/control_code/cruise_controller_node.cpp`

```diff
@@ -98,5 +98,9 @@ CruiseControllerConfig loadControllerConfig()
     config.uncertainty_sigma =
       std::max(0.0, declare_parameter<double>("uncertainty_sigma", 2.0));
+    config.gap_uncertainty_horizon_max = std::max(
+      0.0, declare_parameter<double>("gap_uncertainty_horizon_max", 0.0));
+    config.opp_speed_confidence_z = std::max(
+      0.0, declare_parameter<double>("opp_speed_confidence_z", 0.0));
     config.allow_acceleration = declare_parameter<bool>("allow_accel_trailing", true);
     return config;
   }
```

> 🔴 신규 파라미터는 **`loadControllerConfig()` 한 곳에만** 선언한다. 이 함수는 생성자
> 초기화 리스트(`controller_(loadControllerConfig())`, :27)에서 먼저 호출되고, 생성자
> 본문(:40)은 이미 선언된 `maximum_speed`를 `get_parameter`로 **재조회**한다. 두 곳에 다
> `declare_parameter`를 넣으면 `rclcpp::exceptions::ParameterAlreadyDeclaredException`으로
> 노드가 죽는다.

> 🔴 **`onOpponent()`의 `valid` 술어는 손대지 않는다.** 초기 초안은 여기에
> `std::isfinite(obstacle.vs_var)`를 추가했으나 이는 §5 Phase 2의 "거동 변화 0" 계약을
> 깨는 변경이다 — 조건에 걸리면 장애물이 통째로 기각되고, 다음 타이머에서 opponent가
> stale 처리되어 **`blind_trailing_speed` 1.5 m/s로 강등**된다. 그런데 착지 기본값
> (τ_max=0, z_v=0)에서 `vs_var`는 출력에 전혀 쓰이지 않고, 컨트롤러가 이미 isfinite로
> 가드한다. 이득 0·리스크만 있는 검사이므로 넣지 않고, 아래처럼 **대입 지점에서 강등**한다.

```diff
@@ -258,9 +262,14 @@
     CruiseControllerInput input;
     input.gap = raw_gap;
     input.desired_gap = desired_gap;
     input.ego_speed = ego_vs_;
     input.opponent_speed = opponent_.vs;
     input.opponent_s_variance = opponent_.s_var;
     input.dt = dt;
+    // Degrade a malformed field instead of discarding the whole obstacle: dropping it here
+    // would fail over to blind_trailing_speed, and at the landing defaults this value is not
+    // read at all. The controller guards it a second time.
+    input.opponent_vs_variance = std::isfinite(opponent_.vs_var) ? opponent_.vs_var : 0.0;
+    // f110_msgs/Obstacle carries no s_vs_cov field yet (§3.1 has not landed), so the
+    // cross-covariance term stays at "no correlation" until the message is extended.
+    input.opponent_s_vs_cov = 0.0;
     const auto output = controller_.update(input);
     publishLimit(output.speed_limit);
```

```diff
@@ -276,5 +285,7 @@
     RCLCPP_INFO_THROTTLE(
       get_logger(), *get_clock(), 500,
-      "CRUISE id=%d gap=%.2f/%.2f m ego=%.2f opp=%.2f limit=%.2f m/s visible=%s",
-      opponent_.id, output.effective_gap, desired_gap, ego_vs_, opponent_.vs,
-      output.speed_limit, opponent_.is_visible ? "yes" : "predicted");
+      "CRUISE id=%d gap=%.2f/%.2f m tau=%.2f s sigma=%.2f m ego=%.2f opp=%.2f "
+      "limit=%.2f m/s visible=%s",
+      opponent_.id, output.effective_gap, desired_gap, output.tighten_horizon,
+      output.gap_sigma, ego_vs_, opponent_.vs, output.speed_limit,
+      opponent_.is_visible ? "yes" : "predicted");
```

> 🔴 **`opponent_.s_vs_cov`를 그대로 쓰면 컴파일 에러다.** `f110_msgs/msg/Obstacle.msg`의
> 현재 Frenet 2차 모멘트 필드는 `s_var / d_var / vs_var / vd_var`뿐이며 **교차공분산이 없다**
> (생성 헤더에도 `_vs_var_type`만 존재). 따라서 이번 범위에서는 **Input 필드만 만들어 두고
> 0.0을 넣는 것**이 유일하게 안전한 착지다. §3.1이 들어오면 그 한 줄을
> `input.opponent_s_vs_cov = std::isfinite(opponent_.s_vs_cov) ? opponent_.s_vs_cov : 0.0;`
> 으로 바꾸면 끝난다(마찬가지로 `valid` 술어는 건드리지 않는다).

---

##### 4. `src/f1tenth_control/config/cruise_controller.yaml`

```diff
@@ -22,7 +22,14 @@
     maximum_speed: 12.0
     emergency_stop_distance: 0.45
     relative_deceleration: 2.5
     ego_front_offset: 0.25
     uncertainty_sigma: 2.0
+
+    # 간격 공분산의 시간 전파(§3.4). 두 값 모두 0.0이면 전파·조임이 꺼져
+    # 현행 코드와 비트 단위로 동일하게 동작한다(착지 기본값).
+    # ⚠️ 0.0 이외의 값은 §3.1(msg 확장) + §3.2(PSD 클램프)가 착지한 뒤에만 쓸 것.
+    # gap_uncertainty_horizon_max: 조임에 쓰는 최대 시평 τ_max [s]. Phase 3에서 1.0(CV 유효 지평).
+    # opp_speed_confidence_z: 제동 캡용 상대속도 하한 계수 z_v [-]. Phase 3에서 1.0 안팎으로 A/B.
+    gap_uncertainty_horizon_max: 0.0
+    opp_speed_confidence_z: 0.0
 
     opponent_timeout: 0.15
```

`uncertainty_sigma`(2.0)는 **이름·값 그대로 유지**하고 의미만 z_ε(≈ ε 2.3%)로 재해석한다.
YAML 최상위 키는 `cruise_controller_node:`이고 런치의 `name='cruise_controller_node'`
(`launch/_control_common.py:417-431`)와 일치하므로 네임스페이스 배선은 손댈 것이 없다.

---

##### 5. `src/f1tenth_control/test/test_cruise_controller.cpp`

기존 5개 TEST와 `defaultConfig()`는 **수정하지 않는다**. 파일 상단에 `#include <limits>`를
추가하고, 아래 6개를 마지막 TEST(`PositionalUncertaintyReducesUsableGap`) 뒤,
`}  // namespace` 앞에 삽입한다. 신규 케이스는 새 필드를 쓰므로 집합 초기화 대신
**필드별 대입**을 쓴다.

```cpp
TEST(CruiseController, ZeroHorizonIgnoresSpeedCovariance)
{
  auto config = defaultConfig();
  config.uncertainty_sigma = 2.0;
  f1tenth_control::CruiseLongitudinalController controller(config);

  f1tenth_control::CruiseControllerInput legacy;
  legacy.gap = 3.0;
  legacy.desired_gap = 1.5;
  legacy.ego_speed = 6.0;
  legacy.opponent_speed = 4.0;
  legacy.opponent_s_variance = 0.04;
  legacy.dt = 0.02;
  controller.reset();
  const auto reference = controller.update(legacy);

  for (const double cov : {-0.3, 0.0, 0.3}) {
    f1tenth_control::CruiseControllerInput probed = legacy;
    probed.opponent_vs_variance = 1.7;
    probed.opponent_s_vs_cov = cov;
    controller.reset();
    const auto output = controller.update(probed);
    EXPECT_DOUBLE_EQ(output.tighten_horizon, 0.0);
    EXPECT_DOUBLE_EQ(output.gap_sigma, reference.gap_sigma);
    EXPECT_DOUBLE_EQ(output.effective_gap, reference.effective_gap);
    EXPECT_DOUBLE_EQ(output.speed_limit, reference.speed_limit);
  }
}

TEST(CruiseController, NonFiniteSpeedsKeepTheZeroHorizonMargin)
{
  auto config = defaultConfig();
  config.uncertainty_sigma = 2.0;
  config.gap_uncertainty_horizon_max = 1.0;
  f1tenth_control::CruiseLongitudinalController controller(config);

  f1tenth_control::CruiseControllerInput input;
  input.gap = 10.0;
  input.desired_gap = 1.5;
  input.ego_speed = std::numeric_limits<double>::infinity();
  input.opponent_speed = std::numeric_limits<double>::infinity();
  input.opponent_s_variance = 0.25;
  input.dt = 0.02;

  // inf - inf is NaN, and std::clamp(NaN, 0.0, tau_max) returns NaN. Without the isfinite
  // guard sigma_g would collapse to 0.0 and the whole uncertainty margin would vanish.
  const auto output = controller.update(input);
  EXPECT_DOUBLE_EQ(output.tighten_horizon, 0.0);
  EXPECT_DOUBLE_EQ(output.gap_sigma, 0.5);
  EXPECT_DOUBLE_EQ(output.effective_gap, 9.0);
}

TEST(CruiseController, PropagationHorizonTightensSpeedLimit)
{
  auto config = defaultConfig();
  config.uncertainty_sigma = 2.0;

  f1tenth_control::CruiseControllerInput input;
  input.gap = 4.0;
  input.desired_gap = 1.5;
  input.ego_speed = 6.0;
  input.opponent_speed = 3.0;
  input.opponent_s_variance = 0.01;
  input.dt = 0.02;
  input.opponent_vs_variance = 0.25;

  f1tenth_control::CruiseLongitudinalController without_horizon(config);
  const auto slack = without_horizon.update(input);

  config.gap_uncertainty_horizon_max = 1.0;
  f1tenth_control::CruiseLongitudinalController with_horizon(config);
  const auto tight = with_horizon.update(input);

  EXPECT_GT(tight.tighten_horizon, 0.0);
  EXPECT_GT(tight.gap_sigma, slack.gap_sigma);
  EXPECT_LT(tight.effective_gap, slack.effective_gap);
  EXPECT_LT(tight.speed_limit, slack.speed_limit);
}

TEST(CruiseController, PositiveCrossCovarianceTightensMoreThanNegative)
{
  auto config = defaultConfig();
  config.uncertainty_sigma = 2.0;
  config.gap_uncertainty_horizon_max = 1.0;
  f1tenth_control::CruiseLongitudinalController controller(config);

  // |cov| <= sqrt(s_var * vs_var) = 0.15, so every probed block below is a valid PSD 2x2.
  f1tenth_control::CruiseControllerInput input;
  input.gap = 4.0;
  input.desired_gap = 1.5;
  input.ego_speed = 6.0;
  input.opponent_speed = 3.0;
  input.opponent_s_variance = 0.09;
  input.dt = 0.02;
  input.opponent_vs_variance = 0.25;

  input.opponent_s_vs_cov = 0.0;
  const auto neutral = controller.update(input);
  controller.reset();
  input.opponent_s_vs_cov = 0.10;
  const auto positive = controller.update(input);
  controller.reset();
  input.opponent_s_vs_cov = -0.10;
  const auto negative = controller.update(input);

  EXPECT_GT(positive.gap_sigma, neutral.gap_sigma);
  EXPECT_LT(negative.gap_sigma, neutral.gap_sigma);
  EXPECT_LT(positive.speed_limit, neutral.speed_limit);
  EXPECT_GT(negative.speed_limit, neutral.speed_limit);
}

TEST(CruiseController, NoisierOpponentSpeedLowersBrakingCapOnly)
{
  auto config = defaultConfig();
  config.opp_speed_confidence_z = 1.0;
  f1tenth_control::CruiseLongitudinalController controller(config);

  f1tenth_control::CruiseControllerInput input;
  input.gap = 10.0;
  input.desired_gap = 1.5;
  input.ego_speed = 5.0;
  input.opponent_speed = 4.0;
  input.dt = 0.02;

  const auto confident = controller.update(input);
  controller.reset();
  input.opponent_vs_variance = 1.0;
  const auto noisy = controller.update(input);

  EXPECT_LT(noisy.speed_limit, confident.speed_limit);
  EXPECT_DOUBLE_EQ(noisy.effective_gap, confident.effective_gap);
  EXPECT_DOUBLE_EQ(noisy.gap_error, confident.gap_error);
  EXPECT_DOUBLE_EQ(noisy.relative_speed, confident.relative_speed);
}

TEST(CruiseController, EmergencyStopIgnoresPropagationHorizon)
{
  auto config = defaultConfig();
  config.uncertainty_sigma = 2.0;
  config.gap_uncertainty_horizon_max = 1.0;
  f1tenth_control::CruiseLongitudinalController controller(config);

  f1tenth_control::CruiseControllerInput input;
  input.gap = 1.2;
  input.desired_gap = 1.5;
  input.ego_speed = 6.0;
  input.opponent_speed = 3.0;
  input.opponent_s_variance = 0.01;
  input.dt = 0.02;
  input.opponent_vs_variance = 0.25;

  const auto output = controller.update(input);
  EXPECT_LT(output.effective_gap, config.emergency_stop_distance);
  EXPECT_GT(output.speed_limit, 0.0);
}
```

각 케이스가 고정하는 성질 (손계산 값은 전부 실행 대조로 확인):

| TEST | 고정하는 성질 | 실측값 |
|---|---|---|
| `ZeroHorizonIgnoresSpeedCovariance` | 착지 기본값에서 `vs_var`·`s_vs_cov`(양/0/음)가 **출력에 0의 영향** | τ=0 → σ_g=√0.04=0.2, 세 경우 전부 비트 동일 |
| `NonFiniteSpeedsKeepTheZeroHorizonMargin` | 비유한 속도에서 τ가 NaN이 되어 **불확실성 마진이 사라지는 회귀**를 차단 | τ=0, σ_g=0.5, effective_gap=9.0 |
| `PropagationHorizonTightensSpeedLimit` | τ_max를 켜면 σ_g↑ → effective_gap↓ → 속도 상한↓ (조임 단조성) | τ=clamp(1.2,0,1)=1.0, σ_g=0.5099 → 상한 4.7000 → **3.8802** |
| `PositiveCrossCovarianceTightensMoreThanNegative` | cov 부호가 σ_g와 상한에 **부호 그대로** 반영 (PSD 유효 블록만 사용) | σ_g² = 0.3400 / 0.5400 / 0.1400 → 상한 3.7338 / 3.4303 / 4.1517 |
| `NoisierOpponentSpeedLowersBrakingCapOnly` | z_v는 **제동 캡만** 조이고 `effective_gap`/`gap_error`/`relative_speed`는 불변 (설계 규칙 1) | v_lb 4→3, 제동 캡 7.9844 → **7.5333** |
| `EmergencyStopIgnoresPropagationHorizon` | 비상정지는 τ=0 고정 — effective_gap이 d_e 아래로 조여져도 **정지하지 않는다** (설계 규칙 4) | immediate_gap=1.0 > 0.45 → 통과, effective_gap=0.1802 < 0.45, 상한 **1.0802** (제동 캡 3.0이 아니라 피드백이 지배 — §2-5) |

---

##### 6. 함께 착지해야 하는 문서 (선택이 아니다)

- **`src/f1tenth_control/docs/cruise_controller_node.md` (신규, 한국어).**
  현재 이 파일도, `docs/` 디렉터리 자체도 **존재하지 않는다**. 세 가지가 동시에 걸린다:
  1. `AGENTS.md:39`가 "Keep `docs/cruise_controller_node.md` synchronized"를 요구한다.
  2. 루트 `CLAUDE.md`의 Node Documentation 정책이 이번 규모의 변경에 한국어 문서를 의무화한다.
  3. `CMakeLists.txt:142-144`의 `install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})`가
     없는 디렉터리를 참조해 **`colcon build`의 install 단계가 실패한다**
     (`file INSTALL cannot find .../docs` — 최소 재현 프로젝트로 확인).
     즉 문서를 만들기 전에는 이 변경을 빌드해 검증할 수조차 없다.

  패키지 `.gitignore`에 `docs/`가 있으므로 커밋 시
  `git add -f src/f1tenth_control/docs/cruise_controller_node.md`가 필요하다
  (다른 패키지들도 같은 방식으로 docs를 추적 중 — `git ls-files | grep docs/` 15건).
- **`src/f1tenth_control/AGENTS.md` (수정).** "Cruise parameters live in
  `config/cruise_controller.yaml`" 항목 아래에 신규 파라미터 2개와 그 안전 기본값
  (`0.0` = 현행 거동), 그리고 "§3.1·§3.2 착지 전에는 0.0을 유지한다"는 제약을 추가한다.
  루트 `CLAUDE.md`의 Node-Level AGENTS.md Policy가 요구한다.

---

##### 검증 결과 (이 머신에서 실제로 실행함)

- 이 머신에는 ROS 2 **Humble만** 있어 `colcon` 전체 빌드가 불가하지만,
  `cruise_controller_lib`는 ROS 의존이 0인 순수 함수 라이브러리라 단독 검증이 가능하다.
- `g++ 11.4.0 -std=c++17 -O3 -march=native -flto -Wall -Wextra -I include` 컴파일 — **경고 0**.
  (필드를 끝에 붙였을 때 기존 6원소 집합 초기화가 `-Wmissing-field-initializers`를 유발하지
  않음을 별도 컴파일로 확인. 반대로 `dt` **앞**에 끼우면 경고 없이 6번째 값이
  `opponent_vs_variance`로 들어가고 `dt`는 NSDMI로 떨어지는 것도 실행 출력으로 확인.)
- gtest **11/11 PASS** (기존 5개 전부 무수정 통과 + 신규 6개).
- **비트 동일성 차등 스윕**: 현행 구현과 신규 구현을 서로 다른 네임스페이스로 함께 링크해
  `-O3 -march=native -flto -ffp-contract=off`로 빌드하고, config 9개 × input 8개 ×
  30,000 스텝 = **2,160,000 스텝**을 `speed_limit`/`effective_gap`/`gap_error`/
  `relative_speed`/`gap_integral` `memcmp`로 비교했다. `s_var`/`vs_var`/`s_vs_cov`뿐 아니라
  **`gap`/`ego_speed`/`opponent_speed`에도** `{0, -0, 1e-12, 0.04, 0.45, 3.7, 1e9, ±inf, NaN}`을
  섞었다.
  - **isfinite 가드 있는 최종안: 불일치 0건.**
  - **가드 없는 초기 초안: 불일치 48건** — 전부 `ego_speed`/`opponent_speed`가 동시에
    비유한이라 τ가 NaN이 되는 경로. 초기 초안이 보고한 "불일치 0건"은 스윕이 비유한 값을
    분산 필드에만 주입해 이 경로를 밟지 못한 결과였다.
  - `-ffp-contract` 기본값(fast)에서는 두 구현을 서로 다른 TU에 두면 적분 누산에 1 ULP 차이가
    생기는데, 이는 하네스의 인라이닝 차이에서 오는 FMA 축약 artifact이며 실제 빌드에서는
    같은 표현식이 같은 TU에 있으므로 발생하지 않는다(`-ffp-contract=off`로 0건 확인).
- 검증 산출물:
  `/tmp/claude-1000/-home-haejun-2026-IFAC/eb49d485-2d33-424d-89dc-fcc322c73415/scratchpad/v/`
  (`include/`, `control_code/`, `test/merged.cpp`, `bitcmp.cpp`, `bitcmp_big.cpp`, `nums.cpp`)

---

##### ⚠️ 남은 확인 필요

1. **`f110_msgs/Obstacle.msg`에 `s_vs_cov`가 없다.** 위 코드는 노드에서 `0.0`을 넣어
   컴파일 가능한 상태로 착지시킨다. §3.1이 들어오면 그 한 줄을
   `input.opponent_s_vs_cov = std::isfinite(opponent_.s_vs_cov) ? opponent_.s_vs_cov : 0.0;`
   로 바꾼다(`valid` 술어는 계속 건드리지 않는다).
2. **§3.2의 PSD 클램프가 σ_g² ≥ 0의 전제인데 아직 미착수다.** 현재
   `obstacle_detector_node.cpp`의 `mergeLayer()`는 `s_var`/`vs_var`를 멤버 트랙별 **원소별
   max**(:811-812)로 뽑고 교차항은 발행조차 하지 않는다(:848 부근). §3.2의
   Cauchy–Schwarz 클램프(0.99 계수)가 들어와야 비로소 판별식이 엄격히 음수가 되어 모든 τ에서
   σ_g² > 0이 보장된다. 컨트롤러 쪽 `std::sqrt(std::max(0.0, ...))`가 크래시는 막지만,
   클램프 없이 τ_max를 켜면 σ_g가 0으로 잘려 **조임이 조용히 약해지는** 구간이 생긴다.
   → 착지 요약의 하드 전제조건(§5 대비 이탈) 참조.
3. **실기 검증은 이 머신에서 불가**(ROS 2 Humble만 설치, `install/f1tenth_control`은
   이미 삭제된 노드를 담은 stale 산출물). 젯슨/시뮬 머신에서 **먼저 §6의 `docs/`를 만든 뒤**
   `cb --packages-select f1tenth_control`, 이어서
   `ros2 launch f1tenth_control control_sim.launch.py`로 `/cruise_speed_limit` 발행 지속성
   (끊기면 control_map_node가 0.15 s 뒤 1.5 m/s로 강등)을 반드시 확인할 것.

---

## 4. 왜 이 설계인가 — 기각한 대안

| 대안 | 기각 사유 |
|---|---|
| **A. 확률 판정을 검출기 안에 구현** (메시지 변경 불필요) | 판단이 인지 계층에 남는다 — 이 제안의 출발점과 정면 충돌. 소비자가 FSM 하나뿐이라 이관 비용도 낮다 |
| **B. 검출기를 state_machine에 소스 레벨 흡수** | ① `/static_obs`·`/opp_obs` 소비자(local_planning·cruise·static_obstacle_map) 때문에 토픽이 안 없어져 이득 없음 ② 검출기 크래시가 `/state`·`/local_waypoints`를 통째로 죽임 (현행은 0.3 s 후 GLOBAL 강등 = 우아한 실패) ③ 단일 스레드 FSM에 스캔레이트 콜백 유입 ④ 의존성(tf2/global_planning CLCS/nav2)·런치(터미널 4/8)·테스트 구조 파괴 |
| **C. 교차공분산 없이 대각만으로 확률 판정** | `σ_d²(t)`의 `2t·cov` 항을 상·하계로만 묶을 수 있어 판정이 흐려짐. worst-case 방향이 μ 위치에 따라 뒤집혀(μ가 밴드 안이면 σ↑→P↓) 이중 평가가 필요해짐 — 메시지에 2필드 넣는 것보다 복잡하다 |
| **D. worst 멤버 블록 통째 채택** (PSD 대안) | 항상 유효하지만 기존 `d_var`/`vd_var`의 의미(축별 max)가 바뀜. 클램프 방식이 기존 코드와 충돌 최소 |

---

## 5. 마이그레이션 계획

| Phase | 내용 | 거동 변화 |
|---|---|---|
| **0. 합의** | 이 문서 팀 리뷰. `f110_msgs` 변경 승인 (메시지 정책 게이트) | — |
| **1. msg + detector** | 필드 2개 추가, `mergeLayer()` 채움+클램프, 헬퍼(`obstacle_tracker.hpp`)+유닛테스트, 필드 계약 문서화 — **파일 6개**(§3.0), **전원 동시 리빌드** | **0** (아무도 새 필드를 안 읽음) |
| **2. FSM + cruise** | FSM 술어 라이브러리 + cruise 공분산 캡(§3.4) + gtest + 파라미터/YAML | **0** (기본값 `detector`, `τ_max=0`, `z_v=0`) |
| **3. 검증·전환** | 2-car 시뮬(`sim/launch_cruise.zsh`) A/B → 통과 시 `internal` + `τ_max: 1.0`·`z_v` 활성 | 이때부터 |
| **4. 정리** | 검출기 `isOpponentInterfering()`·`interference_*` 파라미터 제거, AGENTS.md/docs 갱신 (`obstacle_detector/AGENTS.md`의 "must set is_interfering" 항목, `synthetic_opponent_test.py`의 간섭 assert 이전) | 없음 (죽은 코드 제거) |

후속 메시지 계약 정리에서 legacy 간섭 bool과 FSM의 detector 롤백 분기를 제거했다. 이제
`/opp_obs`는 측정값만 전달하고 FSM 확률 술어가 유일한 간섭 판정 경로다.

---

## 6. 호환성·비용 (합의 전 반드시 인지할 것)

- **`f110_msgs` 의존 8개 패키지 전부 리빌드**: global_planning, local_planning, state_machine,
  f1tenth_control, obstacle_detector, monte_carlo_localization, opponent_simulator,
  static_obstacle_map.
- **ROS 2 Jazzy 타입 해시**: 구 빌드와 신 빌드의 노드는 `Obstacle`을 포함한 토픽
  (`/opp_obs`, `/static_obs`, `/confirmed_static_obs` 등)으로 **아예 연결되지 않는다.**
  부분 배포 금지 — 젯슨·랩탑 전부 같은 커밋으로 동시 리빌드해야 한다. 대회 일정상
  주행 없는 날에 수행할 것.
- **기존 rosbag**: 구 타입으로 녹화된 ObstacleArray 토픽은 신 빌드 구독자로 재생되지 않는다.
  분석 도구(bag 리플레이 기반)는 구 워크스페이스 사본으로 돌리거나 bag을 재녹화한다.
- 다른 체크아웃 사본(`~/a/2026_IFAC`, `race_stack` 내 f110_msgs)과의 정합은 각자 pull로 해결.

---

## 7. 검증 계획

1. **단위**: 술어 순수 함수 gtest — 정지 상대/등속 접근/횡 접근(vd≠0)/고스트/랩 경계 wrap/
   cov 극단값(클램프 경계)에서 P의 단조성·경계값. cruise(§3.4): `τ_max=0`이 현행과
   비트 동일 / 접근속도↑ → 캡 단조 강화 / cov 부호별 σ_g 거동 (`test_cruise_controller` 확장).
2. **시뮬 A/B** (`sim/launch_cruise.zsh`, opponent_simulator ground truth):
   - **캘리브레이션**: 예측 `P(t)` vs 실제 t초 후 라인 밴드 진입 빈도 (bin별 신뢰도 곡선)
   - **ROC**: `internal` vs `detector` — 진입 리드타임(라인 접근을 몇 초 먼저 잡나)과 오탐률
   - **채터링**: CRUISE↔GLOBAL 전이 횟수/랩 비교 (§2.2 횡 채터링 해소 확인)
3. **강등 경로**: 검출기 kill → 0.3 s 내 GLOBAL 복귀 + cruise blind 캡 1.5 m/s 유지 확인
   (기존 fail-safe 불변식 회귀 테스트).

---

## 8. 리스크와 롤백

| 리스크 | 완화 |
|---|---|
| 전원 리빌드 창 확보 실패 | Phase 1·2는 거동 변화 0이라 착지 자체는 안전. 전환(Phase 3)만 연기 가능 |
| 확률 술어의 파라미터 미숙 (p_on/p_off) | 기본값 `detector`로 착지, 시뮬 A/B 통과 전 전환 금지. 롤백 = `interference_source: detector` 파라미터 하나 |
| CV 외삽 과신 | T ≤ 1.0 s 고정. 그 이상이 필요해지면 상대 라인 학습(Predictive Spliner 방식)이 다음 단계 — 이 제안의 범위 밖 |
| 병합 공분산 비양정치 | Cauchy–Schwarz 클램프로 구조적으로 차단 (§3.2) |

---

## 부록 A — 관련 문헌 (선별)

- Lefèvre, Vasquez, Laugier, *A survey on motion prediction and risk assessment for intelligent
  vehicles*, ROBOMECH J., 2014 — physics-based 예측 + 확률 리스크의 표준 분류
- Jansson & Gustafsson, *A framework and automotive application of collision avoidance decision
  making*, Automatica, 2008 — 불확실 상태의 통계적 위협 판정
- Curvilinear-Coordinate-Based Object and Situation Assessment for Highly Automated Vehicles,
  IEEE T-ITS — 곡선 좌표계 확률적 차로 연관 (본 제안과 동형)
- Baumann et al., *ForzaETH Race Stack*, J. Field Robotics 2025 (arXiv:2403.11784) — 이 스택
  trailing/GapData의 계보; perception–FSM 분리 구조
- *Predictive Spliner* (arXiv:2410.04868, IEEE RA-L) — CV 한계를 상대 라인 GP 학습으로 넘는
  다음 단계 (T > 1 s가 필요해질 때의 로드맵)
- Koschi & Althoff, *SPOT*, IEEE IV 2017 — 가우시안 불신 시의 set-based 대안

§3.4 (cruise 속도 결정) 관련:

- Ames, Grizzle, Tabuada, *Control barrier function based quadratic programs with application
  to adaptive cruise control*, IEEE CDC 2014 (확장판 arXiv:1609.06408, IEEE TAC 2017) —
  제동거리 안전을 **하드 제약**으로, 속도 추적을 **소프트 목표**로 분리하는 구조의 원류.
  §3.4의 "캡 vs 피드포워드" 분리가 이 구조다
- Moser, Schmied, Waschl, del Re, *Flexible Spacing Adaptive Cruise Control Using Stochastic
  Model Predictive Control*, IEEE TCST 2018 — 선행차 미래 속도의 가우시안 예측 + 간격
  chance constraint. §3.4의 `ĝ − z_ε·σ_g(τ)` 조임과 동형
- Mesbah, *Stochastic model predictive control: An overview and perspectives for future
  research*, IEEE Control Systems Magazine, 2016 — chance constraint → 결정론적 등가
  (constraint tightening)의 일반론
- Shalev-Shwartz, Shammah, Shashua, *On a Formal Model of Safe and Scalable Self-Driving
  Cars* (RSS), arXiv:1708.06374 — worst-case 제동 안전거리의 결정론적 조상. z_ε·σ 조임은
  이것의 확률 완화판이다

---

## 부록 B — 수식 ↔ 문헌 대응표

이 표는 §3.2–§3.4의 각 수식이 어느 문헌의 어느 식에 대응하는지, 그리고 **우리가 어디서 의도적으로
벗어났는지**를 리뷰어가 한 번에 확인하기 위한 것이다. 대상 문서는
`/home/tenmeneat/2026_IFAC/src/state_machine/docs/interference_judgment_migration_proposal.md`.

**검증 등급 표기** (인용 위생)

| 표기 | 의미 | 논문에 쓸 때 |
|---|---|---|
| ✅ | 전문·저자 배포 PDF·arXiv 프리프린트를 직접 대조해 **수식 verbatim 확인** | 식 번호까지 인용 가능 (게재본 번호 차이만 주의) |
| 🟡 | 서지(저자/저널/권호/페이지/DOI)는 확인, **수식 verbatim 미확인** | 식 번호 인용 금지, "동형" 수준 서술만 |
| 🔴 **미확인** | 전문 미접근 또는 수식 신뢰도 low | **수식 근거로 사용 금지.** 구조적 선례·개괄 인용으로만 |

**코드 라인 인용 규칙**: 아래 모든 `:줄번호`는 브랜치 `myungsub_study` 기준 실측값이다. 파일이
바뀌면 이 부록도 함께 갱신한다. `(현행)`은 이미 존재하는 코드, `(예정)`은 §3.x가 삽입할 위치다.

---

### B.1 §3.2 — 병합 공분산 (obstacle_detector)

대상 파일: `/home/tenmeneat/2026_IFAC/src/obstacle_detector/src/obstacle_detector_node.cpp`

- `(현행)` 멤버 루프 `:800-834` — 대각만 축별 max로 누적, 교차공분산 **누적 코드 없음**
- `(현행)` 포락선 중점·대각 발행 `:838-843`(`s_center`/`d_center`), `:847-850`(`s_var`…`vd_var`)
- `(예정)` §3.2의 교차항 누적은 `:800-834` 루프 안, PSD 클램프는 `:847-850` 직후

| # | 우리 수식 | 근거 문헌 | 그 문헌의 원 수식 | 대응 | 차이·주의 |
|---|---|---|---|---|---|
| **E1** | `cov ← clamp(cov, ±0.99·√(var_a·var_b))` (PSD 클램프) | 🟡 Higham 2002 [F1] (문제 정식화 선례) · 🟡 Horn–Johnson 2013 §7.2 [F2] · 🟡 Papoulis–Pillai 2002 Ch.6 [F3] | `min_X ‖A−X‖_F s.t. X=Xᵀ, X⪰0, diag(X)=e` [F1] / 2×2 PSD ⟺ `a≥0, b≥0, ab−c²≥0` [F2] / `\|C_xy\| ≤ σ_x σ_y` [F3] | `a↔var_a`, `b↔var_b`, `c↔cov`. 대각이 고정된 2×2에서 PSD 집합은 1차원 구간 `\|c\|≤√(ab)`이고 목적함수가 `2(c₀−c)²`로 환원되므로, 그 위로의 프로베니우스 사영이 정확히 우리 clamp | ① ⚠️ **[F1]과의 차이 2가지**: (a) Higham의 제약은 `diag(X)=e`, 즉 **단위 대각(상관행렬)**이고 우리는 임의 고정 대각(공분산)이다. (b) [F1]의 기여는 n×n 교대사영 **반복 알고리즘**이지 2×2 폐형이 아니다 — 위 2×2 사영은 초등적이라 원래 인용이 필요 없다. 따라서 [F1]은 "고정 대각 + 프로베니우스 최근접 PSD"라는 **문제 정식화의 선례**로만 인용하고, 식 번호를 우리 clamp의 출처로 쓰지 말 것 ② `0.99` 계수 때문에 결과는 엄밀한 최근접 PSD가 아니다. **두 분산이 모두 양수일 때만 PD**(det = ab(1−0.99²) > 0)이고, 어느 한쪽이 0이면 상계가 0이라 `cov=0`이 강제되어 **PSD(특이)**가 된다 — "항상 PD"라고 쓰지 말 것 ③ **2×2 블록에만 유효** — n≥3에서 off-diagonal 개별 클램프는 PSD를 보장하지 않는다(그래서 [F1]의 반복 알고리즘이 존재). 우리 코드가 s-블록/d-블록 각각 2×2에만 적용한다는 전제에 정당성이 전적으로 의존 ④ [F1][F2][F3]는 **독립적 세 근거가 아니라 동일 사실의 최적화·행렬·확률론적 세 표현** — 나열해 근거를 부풀리지 말 것 |
| **E2** | (기각안) `cov_mix = Σᵢwᵢ[Pᵢ(2,3)+(dᵢ−d̄)(vdᵢ−v̄d)]/w_sum` (총공분산 법칙) | 🟡 Blom–Bar-Shalom 1988 [F4] · 🟡 Salmond 1990 [F5] · 🟡 Runnalls 2007 [F6] | IMM mixing: `P^{0j} = Σᵢ μ^{i\|j}{Pⁱ + [x̂ⁱ−x̂^{0j}][x̂ⁱ−x̂^{0j}]'}` [F4] / 혼합 모멘트 `P = Σᵢβᵢ[Pᵢ+(xᵢ−x̄)(xᵢ−x̄)ᵀ]` [F5] | `μ^{i\|j}(또는 βᵢ) ↔ wᵢ/w_sum`, `x̂ⁱ ↔ (dᵢ, vdᵢ)`, `x̂^{0j} ↔ (d̄, v̄d)`. **행렬 대 행렬로 맞출 것**: `Pⁱ`(문헌의 멤버 공분산 **행렬**) ↔ 우리 트랙 `P`의 (d,vd) 블록 `[[Pᵢ(2,2), Pᵢ(2,3)],[Pᵢ(2,3), Pᵢ(3,3)]]`이고, 그 (0,1) 성분이 `Pᵢ(2,3)`이다 (스칼라 `Pᵢ(2,3)`을 `Pⁱ`에 직접 등치시키면 타입 오류) | **우리는 이 식을 채택하지 않았다**(§3.2 검토 기록 ①②③). 문헌의 전제는 "멤버 = 동일 대상의 경쟁 가설"인데 우리 멤버는 **한 차의 파편 트랙**이라 위치 산포항이 차체 기하이고, 이는 `size`/`d_left`/`d_right`와 §3.3의 `b_eff`에 이미 들어가 **이중계상**된다. [F4]는 가중치가 마르코프 모드확률이라 **가중치 산출 근거로는 인용 불가**. [F6]의 기여는 병합식이 아니라 병합쌍 선택 KL 상계이므로 부수 인용 |

> 중심 이동 항등식 `E[(X−c)(Y−c′)] = cov(X,Y) + (μ̄−c)(ν̄−c′)`(§3.2 ①의 편향 논증)은 기댓값 정의의
> 직접 전개이므로 **인용 불필요**. 논문에는 한 줄 유도로 적고 새 기여로 주장하지 말 것.

---

### B.2 §3.3 — 확률적 간섭 술어 (state_machine) + 트래커 노이즈 모델

관련 코드: `/home/tenmeneat/2026_IFAC/src/obstacle_detector/src/obstacle_tracker.cpp` (전부 현행)

| 위치 | 줄 | 내용 |
|---|---|---|
| `predict()` | `:46-72` | `dt` clamp `:48`, `F` `:50-52`, DWNA `Q` `:58-68`, `t.P = F P Fᵀ + Q` `:71` |
| `measurementCovariance()` | `:74-82` | `R` 대각 `:79-80` |
| `kalmanUpdate()` | `:128-175` | `H` `:131-133`, `R` `:135`, `K` `:165`, `(I−KH)P` `:168`, **재대칭화 `:171` + 대각 클램프 `:174`** |
| 트랙 스폰 시 `P` 초기화 | `:560-564` | `P = I`, 이어서 `P(0,0)=0.5, P(1,1)=4.0, P(2,2)=0.5, P(3,3)=4.0` |

| # | 우리 수식 | 근거 문헌 | 그 문헌의 원 수식 | 대응 | 차이·주의 |
|---|---|---|---|---|---|
| **E3** | `σ_d²(t) = d_var + 2t·d_vd_cov + t²·vd_var` (§3.4의 `σ_g²(τ)`도 동형) | 🟡 Bar-Shalom·Li·Kirubarajan 2001 Ch.5 [F7] | KF 예측: `P(k+1\|k) = F(k)P(k\|k)F(k)' + Q(k)` | `F(t)=[[1,t],[0,1]]`(코드 `:50-52`와 동형), `P(k\|k)=[[d_var, d_vd_cov],[d_vd_cov, vd_var]]`를 대입한 `F P Fᵀ`의 (0,0) 성분. 즉 `Cov(Ax)=A Cov(x) Aᵀ`의 CV 특수화 | **우리는 `Q=0`(순수 외삽)이다.** 문헌의 예측식에는 `+Q(k)`가 있고, [F8]의 DWNA를 함께 쓰면 `+ (t⁴/4)σ_a²`가 추가된다 → 긴 지평에서 t⁴ 항이 지배하므로 위치 불확실성을 **과소평가**한다. 우리는 대신 **T ≤ 1.0 s**로 지평을 잘라 봉쇄한다(§B.5-③). 별도 이름 붙은 정리가 아니므로 **새 기여로 주장 금지**. ⚠️ 확인 필요: [F7]의 식 번호(통상 (5.2.3-2)로 인용)는 원문 미대조 — 번호 없이 `§5.2` 수준으로 인용 |
| **E4** | `Q = [[dt⁴/4, dt³/2],[dt³/2, dt²]]·σ_a²` (축별, `obstacle_tracker.cpp:58-68`) | 🟡 Bar-Shalom et al. 2001 §6.3.2, p.274 (DWNA) [F8] | `x(k+1)=Fx(k)+Γv(k)`, `F=[[1,T],[0,1]]`, `Γ=[T²/2, T]ᵀ`, `E[v(k)v(j)]=σ_v²δ_kj` ⇒ `Q=ΓΓᵀσ_v² = [[T⁴/4,T³/2],[T³/2,T²]]σ_v²` | `T ↔ dt`, `σ_v² ↔ process_var_vs / process_var_vd`. 기호만 다르고 **추가·누락 항 없이 완전 일치** | ① `Q=ΓΓᵀσ²`는 축별 2×2에서 **rank 1, det=0인 특이행렬**(4×4로는 rank 2) — Q의 역/Cholesky를 요구하는 코드에서는 CWNA(같은 책 §6.2.2)를 써야 한다. 현행 코드는 `t.P = F P Fᵀ + Q`(`:71`)만 쓰므로 문제없음 ② 가변 dt마다 재계산해야 하는데 `predict()`가 `dt`를 `[0, dt_max]`로 clamp(`:48`) 후 매 스텝 재계산하므로 충족 ③ ⚠️ 확인 필요: 파라미터 이름 `process_var_vs`/`process_var_vd`(2.0 / 8.0, `config/obstacle_detector.yaml:114-115`)는 실제로는 **가속도 분산 σ_a²**다(σ_a ≈ 1.41 / 2.83 m/s²). [F8]의 튜닝 지침은 `σ_a ≈ 0.5·a_max ~ a_max` — 상대차 `a_max` 실측치와 대조해 범위 내인지 확인하고, 논문에는 기호를 `σ_a²`로 표기할 것 (코드 이름은 오해를 부른다) ④ 식 번호 미확인 → `[BLK01, §6.3.2, p.274]` 형태로 절+페이지 인용 |
| **E5** | `P_lat(t) = Φ((b_eff−μ_d)/σ_d) − Φ((−b_eff−μ_d)/σ_d)`, `P_long(t) = Φ((D−μ_g)/σ_g) − Φ((0−μ_g)/σ_g)` | 🟡 Papoulis–Pillai 2002 Ch.4 [F3] (교과서 사실) | `x~N(η,σ²)` ⇒ `P{a<x≤b} = G((b−η)/σ) − G((a−η)/σ)`, `G(x)=(1/√2π)∫_{−∞}^x e^{−y²/2}dy` | `η↔μ_d`(또는 `μ_g`), `σ↔σ_d`(`σ_g`), `[a,b]↔[−b_eff, b_eff]`(`[0,D]`), `G↔Φ`. **수학적으로는 근사 없이 완전 일치** | ① 정의상 자명한 항등식 → **새 수식으로 제시하지 말 것.** 기여는 μ·σ의 산출(E3/E4)과 `b_eff`의 정의(트랙 밴드 − 차폭 마진)에 있다 ② 표기 주의: Papoulis는 표준정규 CDF를 `Φ`가 아니라 `G`로 쓴다 — 우리 `Φ`의 정의를 본문에 명시 ③ `μ_d = d_center + vd·t`의 `d_center`는 **기하 포락선 중점**(통계 평균 아님, §3.2 ①) → 정규성 가정이 이미 근사 ④ d가 절단(트랙 밖 불가) 또는 다봉이면 `\|μ\|≫b_eff` 꼬리에서 상대오차가 크다 ⑤ 🔴 **구현 요구사항 — σ=0 나눗셈**: `kalmanUpdate()`가 `t.P(i,i) = std::max(0.0, t.P(i,i))`(`:174`)로 대각 0을 허용하고 병합이 그 max를 그대로 발행하므로 `d_var = vd_var = 0`이면 σ_d(t) ≡ 0이 되어 위 식이 **0으로 나눈다**(`σ_g`/`P_long`도 동일). §3.3 구현은 σ에 하한(예: 1e-6 m)을 두거나 σ=0일 때 결정론적 지시함수로 분기해야 한다. §3.4는 σ_g를 곱하기만 하므로 이 경로가 없다 ⑥ ⚠️ 확인 필요: [F3]의 절·식 번호 미대조 → 번호 없이 `Ch.4` 수준 인용 |
| **E6** | `P(t) = P_lat(t)·P_long(t)` (곱 분해) | ✅ Bishop 2006 §2.3.2, "Partitioned Gaussians" 박스, 식 (2.98) [F9] · 🟡 Anderson 2003 §2.4 [F10] | `p(x_a)=N(x_a\|μ_a,Σ_aa)` (2.98) — **주변화 결과** [F9] / 결합정규에서 `X⁽¹⁾⟂X⁽²⁾ ⟺ Σ₁₂=0` [F10] | `x_a ↔ (d, vd)`, `x_b ↔ (s, vs)`, `Σ_ab ↔ s–d 교차공분산 블록` | ① **단일 트랙에서는 엄밀하다**: 우리는 Cartesian 공분산을 Frenet으로 투영하지 않고 **Frenet 상태 `[s,vs,d,vd]`를 네이티브로 추적**하며, `F`(`:50-52`)·`Q`(`:58-68`)·`P` 초기값(`:560-564`)이 모두 블록대각이고 `H`(`:131-133`)가 (s,d)만 뽑고 `R`(`:79-80`)이 대각이라 `(I−KH)P`(`:168`)와 후속 재대칭화(`:171`)가 블록대각을 보존한다 ② 🔴 **다중 파편 병합에서는 엄밀하지 않다**: 발행되는 것은 병합 결과이고 `s_var/d_var`는 멤버별 **max**, `vs/vd`는 size-가중 평균, `s_center/d_center`는 **기하 포락선 중점**(`:838-843`)이다. 한 차가 여러 트랙으로 쪼개지면 파편들의 s와 d는 회전한 차체 때문에 실제로 상관을 갖는데, `Obstacle.msg`에 s–d 교차 필드가 없어 독립이 **스키마에 의해 강제**될 뿐이다. 논문에는 "단일 트랙 유지 시 엄밀 / 병합 시 모델 가정"으로 쓰고, Phase 3에서 **병합 멤버 수 분포**를 로그로 보고할 것 ③ ⚠️ `R` 대각은 **모델 가정**이다(LiDAR 극좌표 잡음은 s–d 상관을 가질 수 있음) — "설계상 강제한 블록대각"으로 서술하고 시뮬에서 `\|ρ_sd\|`를 실측 보고 ④ ⚠️ **인블록 PSD는 별개 문제**: `:174`의 대각 클램프는 대각만 0으로 끌어올리므로 `P(2,2)=0`인데 `P(2,3)≠0`인 비PSD 블록이 만들어질 수 있다. s–d 블록대각성은 깨지지 않지만 E1 클램프의 원천이 오염되며, 이때 클램프 상계가 0이 되어 `cov=0`이 강제되므로 E1이 최종 안전망 역할을 한다 ⑤ [F9]의 (2.98)은 **주변화** 결과이므로 독립 주장에는 `Σ_ab=0`을 별도 명시할 것(주변화 자체는 독립을 함의하지 않는다). 표의 `Σ_ab=0 ⇒ Λ_ab=0`·`\|Σ\|=\|Σ_aa\|\|Σ_bb\|` 인수분해 사슬은 **Bishop의 번호 붙은 식이 아니다** — ✅는 (2.98)에만 적용되고, 인수분해는 우리 쪽 한 줄 유도 또는 [F10]으로 근거를 댈 것 ⑥ ⚠️ [F10] 정리 번호 미확인 → `§2.4`로만 인용 |
| **E7** | 곡선좌표계 확률적 위협 판정 전체 구조 (Frenet d 밴드 점유 + 종방향 조건 → 확률 임계 판정) | 🔴 **미확인(수식)** Kim et al. TITS 2015 [F11] · 🟡 Jansson–Gustafsson 2008 [F12] | [F11] 전문 미접근 — 초록 수준: cubic Hermite spline 차선 곡선좌표계로 융합 트랙 투영 후 차로 경계 사이 적분 = "Probabilistic Lane Association" / [F12] 사후분포 `p(x\|Y)` 위 '개입 필요 집합' 적분 + 베이즈 위험 비교, 적분은 Monte Carlo | [F11] 차선 경계 ↔ 우리 밴드 `±b_eff`, 차로 점유확률 ↔ `P_lat` / [F12] `p(x\|Y)` ↔ 우리 예측 가우시안, 개입 필요 집합 ↔ 위협 영역, 베이즈 위험 임계 ↔ `p_on`/`p_off` | 🔴 **[F11]은 수식 근거로 쓰지 말 것** — IEEE Xplore 403으로 전문 미접근, 논문 자신의 기호·식을 확인하지 못했다(서지는 Crossref로 확인 완료, B.4 참조). "곡선좌표계에서 횡방향 가우시안을 경계 사이로 적분하는 접근의 **선례**"로만 한정 인용. 또한 이들은 다차선 도로의 **차로 배정**이고 우리는 레이싱 트랙 **폭 밴드 점유**라 경계 정의·개수가 다르다. [F12]는 **번호 붙은 원문 수식을 그대로 인용하지 말 것**(ScienceDirect 403). 우리 방식은 [F12] 프레임워크의 **(a) 사후분포 가우시안 · (b) 위험집합 축정렬 박스** 특수화로 적분을 닫힌형 Φ 차분으로 대체한 것임을 명시해야 한다 — 그들의 기여인 일반성(임의 분포·임의 위험집합)을 우리는 쓰지 않는다. 그들이 명시적으로 다루는 오개입/미개입 **비용비**를 우리는 단일 임계 `p_on`으로 대체했으므로, 그 임계가 어떤 비용비에 대응하는지 밝혀야 프레임워크 인용이 정합적이다 |

---

### B.3 §3.4 — cruise 속도 상한 (f1tenth_control)

대상 파일: `/home/tenmeneat/2026_IFAC/src/f1tenth_control/control_code/cruise_controller.cpp`

- `(현행)` 입력 클램프 `:22-23`(`ego_speed`/`opponent_speed`를 `max(0, ·)`), `sigma_s` `:24`
- `(현행)` `effective_gap` `:26-27` / 비상정지 `:36-38` / `feedback_speed` `:41-45`
- `(현행)` `usable_gap` `:49-50` / `braking_speed` `:51-53` / `speed_limit` `:55-56`
- `(현행)` gap 정의 `cruise_controller_node.cpp:247`(`center_ahead`), `:248-251`(`longitudinal_span` + 무효화), `:252-253`(`raw_gap`)
- `(예정)` `σ_g(τ)` 전파 · `v_opp_lb` · `τ_s`는 §3.4 신설 (현행 코드에 없음)

| # | 우리 수식 | 근거 문헌 | 그 문헌의 원 수식 | 대응 | 차이·주의 |
|---|---|---|---|---|---|
| **E8** | `P(g(τ) ≥ d_e) ≥ 1−ε ⟺ ĝ(τ) − z_ε·σ_g(τ) ≥ d_e`, `z_ε=Φ⁻¹(1−ε)` | ✅ Schwarm–Nikolaou 1999 [F13] (1차) · ✅ Moser 2015 학위논문 §3.5.4.1 [F14] (verbatim 동일) · 🟡 Moser et al. TCST 2018 [F15] (응용) · 🟡 Charnes–Cooper 1963 [F16] (기원 각주) | [F13] `Pr{aᵀ(Mx+c) ≤ b} ≥ α` (5) ⇒ `āᵀ(Mx+c) ≤ b − K_α√((Mx+c)ᵀP_a(Mx+c))` (9), `K_α = F⁻¹(α)`; (10)은 "평균만 대입"이 **틀렸음**을 보이는 반례식 / [F14] `ℙ[Δx ≥ d_min + h·v] ≥ 1−α_LB` (3.38) ⇒ `Φ(·) ≤ α_LB` (3.41) ⇒ `Δx̄ ≥ d_min + h·v − Φ⁻¹(α_LB)Σ_Δx^{1/2}` (3.42) ⇒ `Δx̄ ≥ d_min + h·v + Φ⁻¹(1−α_LB)Σ_Δx^{1/2}` (3.43), Fig.3.8 캡션 `m = Φ⁻¹(1−α_LB)Σ^{1/2}` | `Δx̄ ↔ ĝ(τ)`, `d_min + h·v ↔ d_e`, `Σ_Δx^{1/2} ↔ σ_g(τ)`, `α_LB ↔ ε`, `Φ⁻¹(1−α_LB) ↔ z_ε`, tightening 양 `m ↔ z_ε·σ_g`. **(3.43)은 우리 식의 이항만 바꾼 완전히 동일한 식** | ① [F13]은 상한 제약, 우리는 하한 → `Φ⁻¹(α)=−Φ⁻¹(1−α)` 부호 반전 명시([F14]의 (3.42)→(3.43)이 이 대칭성을 명시적으로 사용) ② [F13]의 무작위성은 모델계수 a에 있어 `σ_y`가 결정변수 x에 의존 → SOCP가 되지만, 우리 `σ_g`는 **추정기가 주는 외생값**이라 상수 오프셋 → 그 논문의 볼록성 논증을 물려받지 않는다 ③ 등가(⟺)는 g가 가우시안일 때만 exact ④ [F14] §3.5.4.1이 스스로 "individual chance constraints"라 명시 — **개별 chance constraint**이므로 예측구간 전체의 joint 보증 없음 ⑤ [F14]는 학위논문(peer-review 약함) → **인용은 [F13] 또는 [F15]로, [F14]는 유도 확인용**. [F14]의 유도는 "the variable v is assumed to be certain"(자차 속도 결정론) 가정 하 — 우리도 ego s를 결정론으로 두므로(§3.4) 동일 가정 ⑥ 🟡 [F15][F16] **수식 verbatim 미확인** — 식 번호 인용 금지, 원문 대조 후에만 "동형" 서술. [F15]는 목적이 **연비 최적화**이고 σ를 실측 학습으로 얻으므로 **ε 선택 근거를 그대로 가져오면 안 된다** ⑦ `uncertainty_sigma=2.0 ↔ ε≈2.28%`(`Φ(2)=0.97725`, `cruise_controller.yaml:26`) — 값·이름 불변 |
| **E9** | `v_brake = √(v_opp_lb² + 2·a_rel·max(0, effective_gap − d_e))` | ✅ Shalev-Shwartz et al. 2017 (RSS) Lemma 2 [F17] · 🟡 Gipps 1981 [F18] | [F17] `d_min = [v_rρ + ½a_max,accel ρ² + (v_r+ρa_max,accel)²/(2a_min,brake) − v_f²/(2a_max,brake)]_+` / [F18] `v_safe = −B(τ/2+θ) + √([B(τ/2+θ)]² + B[2g(t) − v_f(t)τ + v_ℓ(t)²/B̂])` | [F17]에서 `ρ=0`, `a_min,brake=a_max,brake=a` 특수화 후 `d_min ≤ d`를 `v_r`에 대해 풀면 정확히 `v_r ≤ √(v_f²+2ad)`. `v_f ↔ v_opp_lb`, `d ↔ usable_gap`, `a ↔ 우리 코드의 relative_deceleration`. [F18]에서는 `τ=θ=0, B=B̂=a` 극한 | ① **우리 식은 독자 유도가 아니라 두 문헌의 공통 특수화** — 정직히 명시할 것 ② 🔴 **기호 주의 — `a_rel`은 '상대' 감속도가 아니다**: RSS의 `a_min,brake`/`a_max,brake`는 **각 차량 자신의** 감속도이고, 두 값을 같게 놓는 특수화이므로 *상대* 감속도는 오히려 0이 된다. 코드 파라미터 이름 `relative_deceleration`(2.5, `cruise_controller.yaml:24`)은 E4 ③의 `process_var_vs`와 같은 종류의 오해를 부른다 → 논문 기호는 `a`(대칭 가정 하 공통 감속도)로 쓰고 코드명은 각주로 밝힐 것 ③ 🔴 **ρ=0(응답시간 0) 가정이 낙관적**: 검출→계획→제어 지연이 있으므로 RSS 등가가 되려면 ρ만큼 여유거리를 별도로 빼야 한다. 우리는 대신 `z_ε·σ_g(τ_s)` 조임으로 대체한다(§B.5-⑤ — 등가 보증이 아니라 **치환**임을 명시) ④ `a_min,brake=a_max,brake` 대칭 가정은 **앞차가 우리보다 세게 제동하는 경우를 커버하지 못한다**(RSS는 의도적으로 비대칭). 우리는 `v_opp_lb = max(0, v̂s − z_v√vs_var)`로 상대속도를 하한 보수화해 일부만 상쇄한다 ⑤ [F18]의 Lücken(2019) 지적: `B > B̂`이면 충돌 보증이 깨진다 → `a_rel`을 자차 성능 낙관치로 잡지 말 것 ⑥ RSS `d_min`은 **앞끝~뒷끝 거리** — 우리 `gap`은 `center_ahead − span/2 − ego_front_offset`(`cruise_controller_node.cpp:252-253`)이라 차체 보정이 되어 있다. ⚠️ 단 `:249-250`에서 `longitudinal_span`이 비유한이거나 `> 0.5·L`이면 **0으로 무효화**되어 상대차 길이 보정이 사라지는 경로가 있고, 상대차 후면은 실제 범퍼가 아니라 측정 s-스팬의 절반 근사다 — 논문에 각주로 밝힐 것 ⑦ 🟡 [F18] verbatim은 2차 문헌(L. Lücken, arXiv:1902.04927, 식 (5)(7))에서 읽은 것 — Gipps 1981 원문(감속을 음수 `b_n`으로 표기)과 기호가 다르므로 인용 전 원문 대조 필요 ⑧ 🔴 **혼동 금지**: CBF 문헌 [F19]의 force-based 식은 (38) `D(t+T) = D(t) − ½[v₀−v(t)]²/(c_d g)`, 안전집합 `C_F = {(D,v) \| D − ½(v₀−v)²/(c_d g) ≥ 1.8v}`, (39) `h_F = −1.8x₂ − ½(v₀−x₂)²/(c_d g) + z`에서 나오는 `v ≤ v₀ + √(2 c_d g D)`로 **선행차 정속 `v₀` 가정**이며 우리 식과 다른 상한이다(`√(v_l²+2ad) ≤ v_l+√(2ad)`, 우리가 더 보수적). [F19]를 우리 `v_brake`의 출처로 인용하면 지적당한다 |
| **E10** | 안전=캡(`min(feedback, v_brake)`, 비상정지) / 추적=피드포워드(`v̂s + PID`) 분리 | ✅ Ames–Grizzle–Tabuada CDC 2014 [F19] · ✅ Ames–Xu–Grizzle–Tabuada TAC 2017 [F20] | [F20] Hard: `D − τ_d v_f ≥ 0` (44), `h(x)=D−τ_d v_f` / Soft: `V(x) := (v_f−v_d)²` / QP 제약행렬 `A_clf=[L_gV, −1], b_clf=−L_fV−cV` (47), **`A_cbf=[L_gB, 0]`, `b_cbf=−L_fB+γ/B`** (48) / [F19] `D ≥ v/2` (HC1) ⇒ `z ≥ 1.8x₂`, `v−v_d → 0` (SC1), `(SC1-CLF)`의 완화변수 `δ_sc`, `(HC1-CBF)` 뒤 "Since this is a hard constraint, no relaxation is used" | `h = D − τ_d v_f ↔ g − d_e` (헤드웨이 여유), `A_cbf`의 **둘째 성분 0 = 슬랙 없음 ↔ 우리 안전 캡은 완화하지 않음**, `δ_sc`(CLF 완화) ↔ 우리 추적 피드포워드의 소프트 성격, `V=(v_f−v_d)²` ↔ 속도/간격 추적 비용, `τ_d ↔ d_e`의 헤드웨이 성분 | ① 🔴 **우리는 QP도 CBF도 풀지 않는다** — `min()` 캡이므로 forward invariance 보증이 없다. **"하드/소프트 분리라는 설계 철학만 차용"**이라고 명시하고 메커니즘 동일성을 주장하지 말 것 ② [F19][F20] 모두 **확률 개념이 전혀 없다(결정론적)** — E8(chance constraint)과 섞어 인용 금지. [F19]는 선행차 정속 `v₀`(`ż = v₀ − x₂`), [F20]도 `a_L ∈ [−a_l g, a'_l g]`의 **집합 유계** 처리다 ③ [F19] §V-C는 스스로 "Since it may be the case that these constraints will conflict with the torque values needed to satisfy the hard constraint (HC1-CBF)"라며 힘 제약과 헤드웨이 CBF의 **충돌 가능성**을 인정하고 별도 force-based CBF를 추가한다([F20]도 `h^c_F`/`h^o_F`로 동일 처리) — 우리가 "안전=하드"라고 쓰려면 액추에이터 한계(`maximum_speed` clamp, 실제 감속 능력)와의 양립성을 같은 수준으로 논증해야 한다 ④ 저자 목록 주의: **CDC판 3인 / TAC판 4인(Xu 추가)** — 같은 저자로 쓰면 서지 오류 ⑤ 인용 식 번호는 저자 배포 PDF / arXiv:1609.06408v2 기준 — 게재본 번호와 다를 수 있음 |
| **E11** | 조임은 캡에만, `feedback = max(0, v̂s) + Kp·e + Ki∫e + Kd·(max(0,v̂s) − max(0,v_ego))`는 **평균 그대로** (certainty equivalence) | ✅ Hewing–Zeilinger CDC 2018 [F21] · 🟡 Mayne–Seron–Raković 2005 [F22] (verbatim은 2006 후속 [F22b]) | [F21] `x(k)=z(k)+e(k)`, `u(k)=v(k)+Ke(k)` (7), `z_{i+1}=Az_i+Bv_i` (8a) "the nominal predicted system state z_i is deterministic", `z_i ∈ Z := X ⊖ R_x` (12a) / [F22b] `x̄⁺=Ax̄+Bū` (6) "obtained from (1) by neglecting the disturbances", `X̄ ≜ X ⊖ S` (18) | `z_i`(공칭=평균, `E[w]=0`) ↔ 우리 피드포워드가 쓰는 `v̂s`/`ĝ`, `v_i` ↔ 피드포워드 입력, `X ⊖ R_x` ↔ `d_e → d_e + z_ε·σ_g` 조임, `R_x`(확률 도달집합) ↔ `z_ε·σ_g(τ_s)`, `K e(k)` ↔ PID 피드백. [F21] Remark 5의 "half-space 제약을 marginal 분포로 조임"이 우리처럼 **스칼라 간격 제약 하나**를 `Φ⁻¹(1−ε)σ`만큼 미는 것을 정당화(다차원 χ² 타원체가 아님) | ① ⚠️ **코드와의 정확한 일치**: 현행 `cruise_controller.cpp:22-23`이 `ego_speed`/`opponent_speed`를 `std::max(0.0, ·)`로 클램프하므로 피드포워드 항은 `v̂s`가 아니라 `max(0, v̂s)`다. §5 Phase 2가 약속한 "`τ_max=0, z_v=0`에서 비트 동일"을 문자 그대로 지키려면 신규 구현이 이 클램프를 유지해야 한다(`z_v=0`일 때만 `v_opp_lb`와 일치) ② `z = E[x]`는 **`E[w]=0` 전제**에 의존 — 상대차 거동 예측이 bias를 가지면 certainty equivalence가 깨진다. bias 없음을 보이거나 `d_e`에 흡수시켜야 한다 ③ [F21]은 매 스텝 `z(k)=x(k)` 리셋 시 **폐루프 chance constraint 만족을 별도로 논증**한다(unimodality 가정). 우리는 매 사이클 측정으로 리셋하므로 그 보증이 **그냥 따라오지 않는다** — recursive feasibility도 별도 논거 필요 ④ [F21]은 LTI + i.i.d. 외란 전제 ⑤ [F21]의 기본형은 정상상태 `R`(Remark 6)이고 우리는 **τ에 따라 변하는 `σ_g(τ_s)`**를 쓰므로 n-step PRS 쪽에 가깝다 — 인용 시 이 점 명시 ⑥ 🔴 **[F22]를 확률 근거로 쓰지 말 것**: Mayne의 `S`는 **유계 외란을 전부 덮는 집합(위반확률 0)**이고 가우시안은 support가 무한해 그런 `S`가 존재하지 않는다. "공칭+조임"이라는 **구조**의 원형으로만 인용 ⑦ 🟡 [F22] 2005 원문 수식 verbatim 미확인 → verbatim이 필요하면 [F22b](2006, output feedback판, 관측기 오차집합 `S̃` 추가됨) 사용 |

---

### B.4 각주 — 정식 서지 (검증 등급 포함)

> 서지 검증 방법: 아래 모든 DOI를 Crossref API로 직접 조회해 제목·저자·권호·페이지·연도를 대조했고,
> arXiv 항목은 arXiv API로 저자·제목을 대조했다. ✅ 표시된 수식은 저자 배포 PDF / arXiv / 공개
> 학위논문 PDF를 내려받아 원문 대조한 것이다.

- **[F1]** 🟡 N. J. Higham, "Computing the nearest correlation matrix—a problem from finance," *IMA J. Numer. Anal.*, vol. 22, no. 3, pp. 329–343, Jul. 2002. DOI 10.1093/imanum/22.3.329. (서지 ✅ Crossref 대조. ⚠️ **문제 정식화 선례로만 인용** — Higham의 제약은 `diag(X)=e`(단위 대각/상관행렬)이고 기여는 n×n 반복 알고리즘이라, 우리의 임의 고정대각 2×2 폐형 클램프의 직접 출처가 아니다.)
- **[F2]** 🟡 R. A. Horn and C. R. Johnson, *Matrix Analysis*, 2nd ed. Cambridge Univ. Press, 2013, §7.2 (Sylvester's criterion 및 그 PSD 변형). ISBN 978-0-521-83940-2. ⚠️ 확인 필요: "Theorem 7.2.5"라는 **번호는 2차 출처 의존** — 번호 없이 `§7.2`로 인용할 것. ⚠️ Sylvester's criterion 자체는 **양정치(PD)** 판정(선행 주소행렬식 > 0)이고 우리가 쓰는 것은 **PSD** 판정이라 *모든* 주소행렬식 ≥ 0이 필요하다: 선행 주소행렬식만으로는 불충분(`[[0,0],[0,−1]]`은 선행 주소행렬식이 0, 0인데 PSD 아님). 우리 2×2 검사는 대각 2개 + det = **모든 주소행렬식**이라 필요충분.
- **[F3]** 🟡 A. Papoulis and S. U. Pillai, *Probability, Random Variables and Stochastic Processes*, 4th ed. McGraw-Hill, 2002 (Ch. 4 정규분포 구간확률 / Ch. 6 Schwarz 부등식·상관계수, 색인상 p. 210). ISBN 0-07-366011-6. ⚠️ 절·식 번호 미대조.
- **[F4]** 🟡 H. A. P. Blom and Y. Bar-Shalom, "The interacting multiple model algorithm for systems with Markovian switching coefficients," *IEEE Trans. Autom. Control*, vol. 33, no. 8, pp. 780–783, Aug. 1988. DOI 10.1109/9.1299. (서지 ✅ Crossref 대조 / 식 번호 미확인)
- **[F5]** 🟡 D. J. Salmond, "Mixture reduction algorithms for target tracking in clutter," *Proc. SPIE*, vol. 1305 (*Signal and Data Processing of Small Targets 1990*), pp. 434–445, 1990. DOI 10.1117/12.21610. (서지 ✅ Crossref 대조 — Crossref는 시작 페이지 434만 기재. 초록의 "preserve the mean and covariance of the mixture"만 확인, 가중치 기호 `β` vs `w`·식 번호 미전사)
- **[F6]** 🟡 A. R. Runnalls, "Kullback-Leibler approach to Gaussian mixture reduction," *IEEE Trans. Aerosp. Electron. Syst.*, vol. 43, no. 3, pp. 989–999, Jul. 2007. DOI 10.1109/TAES.2007.4383588. (서지 ✅ Crossref 대조)
- **[F7]/[F8]** 🟡 Y. Bar-Shalom, X. R. Li, and T. Kirubarajan, *Estimation with Applications to Tracking and Navigation*. Wiley, 2001 — [F7] Ch. 5 "State Estimation in Discrete-Time Linear Dynamic Systems," pp. 199–266 (DOI 10.1002/0471221279.ch5), [F8] Ch. 6 "Estimation for Kinematic Models," pp. 267–299, 그중 §6.3.2 "Discrete White Noise Acceleration Model," **p. 274** (DOI 10.1002/0471221279.ch6). ISBN 978-0-471-41655-5. (장 제목·페이지 범위 ✅ Crossref 대조.) 교차검증: FilterPy `Q_discrete_white_noise` 문서가 근거로 "Bar-Shalom … Page 274"를 명시하고, `dt=0.1, var=1`에서 `[[2.5e-5,5e-4],[5e-4,1e-2]]`를 산출 = `dt⁴/4, dt³/2, dt²`와 정확히 일치. ⚠️ 책 원문 식 번호와 §6.3.2라는 절 번호는 미확인 → `§6.3.2, p.274`로 인용하고 절 번호가 불확실하면 `Ch. 6, p. 274`로 낮출 것.
- **[F9]** ✅ C. M. Bishop, *Pattern Recognition and Machine Learning*. Springer, 2006, §2.3.2 "Marginal Gaussian distributions"의 "Partitioned Gaussians" 요약 박스, 식 **(2.98)** `p(x_a)=N(x_a|μ_a,Σ_aa)`. ISBN 978-0-387-31073-2. ⚠️ ✅는 **(2.98)에만** 적용된다 — E6 표의 `Σ_ab=0 ⇒ Λ_ab=0` 인수분해 사슬은 Bishop의 번호 붙은 식이 아니다.
- **[F10]** 🟡 T. W. Anderson, *An Introduction to Multivariate Statistical Analysis*, 3rd ed. Wiley, 2003, §2.4. ISBN 0-471-36091-0. ⚠️ 정리 번호 미확인. **전제 주의**: '무상관 ⇒ 독립'은 **결합정규**일 때만 — E2의 혼합 병합이나 E1의 클램프를 거친 뒤에도 결합가우시안 가정이 유지되는지는 "모멘트 매칭된 가우시안 대리분포 위에서만" 정당하다.
- **[F11]** 🔴 **미확인(수식)** J. Kim, K. Jo, W. Lim, M. Lee, and M. Sunwoo, "Curvilinear-Coordinate-Based Object and Situation Assessment for Highly Automated Vehicles," *IEEE Trans. Intell. Transp. Syst.*, vol. 16, no. 3, pp. 1559–1575, Jun. 2015. DOI 10.1109/TITS.2014.2369737. **서지 ✅ Crossref 대조 완료**(저자 전체 이름: Junsoo Kim, Kichun Jo, Wontaek Lim, Minchul Lee, Myoungho Sunwoo — 현행 부록 A에 저자·권호가 빠져 있으므로 이 서지로 교체할 것). **수식은 전문 미접근 → 근거로 사용 금지.**
- **[F12]** 🟡 J. Jansson and F. Gustafsson, "A framework and automotive application of collision avoidance decision making," *Automatica*, vol. 44, no. 9, pp. 2347–2351, Sep. 2008. **DOI 10.1016/j.automatica.2008.01.016** (✅ Crossref 대조). ⚠️ 이 논문은 흔히 `…/j.automatica.2008.01.030`으로 **오인용**되는데, 그 DOI는 실제로 G. Chesi and Y. S. Hung, "Stability analysis of uncertain genetic sum regulatory networks," *Automatica*, vol. 44, no. 9, pp. 2298–2305 (완전히 다른 논문)를 가리킨다 — 부록 A에 DOI를 넣을 때 **016**을 쓸 것. 보조: (a) J. Jansson, **J. Johansson**, F. Gustafsson, "Decision Making for Collision Avoidance Systems," SAE Tech. Paper 2002-01-0403, DOI 10.4271/2002-01-0403 — ⚠️ 2차 문헌이 자주 "Jansson, **Ekmark**, Gustafsson"으로 쓰지만 Crossref 저자 목록은 Jansson/Johansson/Gustafsson으로 **Ekmark는 저자가 아니다**; (b) R. Karlsson, J. Jansson, F. Gustafsson, "Model-based statistical tracking and decision making for collision avoidance application," *Proc. ACC 2004*, pp. 3435–3440, DOI 10.23919/ACC.2004.1384441; (c) J. Jansson, Ph.D. dissertation No. 950, Linköping Univ., 2005 (🟡 서지 수준 인용만).
- **[F13]** ✅ A. T. Schwarm and M. Nikolaou, "Chance-constrained model predictive control," *AIChE J.*, vol. 45, no. 8, pp. 1743–1752, Aug. 1999. DOI 10.1002/aic.690450811 (서지 ✅ Crossref). 저자 배포 원고 PDF(`chee.uh.edu/…/nikolaou/chanceconstrained.pdf`)로 식 (5)–(11) verbatim 대조 완료: (5) `Pr{aᵀ(Mx+c) ≤ b} ≥ α`, (6) `Var(aᵀ(Mx+c)) = (Mx+c)ᵀP_a(Mx+c)`, (8) `(b−āᵀ(Mx+c))/√((Mx+c)ᵀP_a(Mx+c)) ≥ K_α`, (9) 그 이항형. ⚠️ 식 번호는 원고 기준 — 게재본과 다를 수 있음. 이 논문은 (10)식 = "평균만 대입하고 조이지 않는 것"이 **"is not correct"**라고 명시한다 → 우리가 왜 naive 버전을 쓰지 않는지의 직접 근거.
- **[F14]** ✅ D. Moser, "Stochastic Model Predictive Control Applied to Cooperative Adaptive Cruise Control," Master's thesis (Diplom-Ingenieur, Mechatronics), Johannes Kepler Univ. Linz, Feb. 2015. PDF: https://epub.jku.at/download/pdf/379855 — §3.5.4.1 "Deterministic equivalent constraints for lower bound", 식 (3.38)–(3.43), Fig. 3.8 캡션 `m = Φ⁻¹(1−α_LB)Σ_Δx^{1/2}` verbatim 대조 완료. 학위논문 → **보조 인용만**.
- **[F15]** 🟡 D. Moser, R. Schmied, H. Waschl, and L. del Re, "Flexible Spacing Adaptive Cruise Control Using Stochastic Model Predictive Control," *IEEE Trans. Control Syst. Technol.*, vol. 26, no. 1, pp. 114–127, Jan. 2018. DOI 10.1109/TCST.2017.2658193 (서지 ✅ Crossref). 동일 계열 peer-reviewed 학회판: D. Moser, H. Waschl, H. Kirchsteiger, R. Schmied, L. del Re, "Cooperative adaptive cruise control applying stochastic linear model predictive control strategies," *Proc. ECC 2015*, pp. 3383–3388, DOI 10.1109/ECC.2015.7331057 (🟡, 서지 ✅ Crossref). ⚠️ 둘 다 **본문 수식 verbatim 미확인** — 식 번호 인용 금지.
- **[F16]** 🟡 A. Charnes and W. W. Cooper, "Deterministic Equivalents for Optimizing and Satisficing under Chance Constraints," *Oper. Res.*, vol. 11, no. 1, pp. 18–39, Feb. 1963. DOI 10.1287/opre.11.1.18 (선행: "Chance-Constrained Programming," *Manage. Sci.*, vol. 6, no. 1, pp. 73–79, Oct. 1959. DOI 10.1287/mnsc.6.1.73). 둘 다 서지 ✅ Crossref. ⚠️ 유료 원문 미열람 → **기원 각주로만**, 식은 [F13]/[F14]에서 인용.
- **[F17]** ✅ S. Shalev-Shwartz, S. Shammah, A. Shashua, "On a Formal Model of Safe and Scalable Self-driving Cars," arXiv:1708.06374 (v1 2017-08-21; 최신 v6). 저자·제목 ✅ arXiv API 대조 — §3.1 Definition 1 / **Lemma 2** verbatim 대조. ⚠️ 프리프린트(정식 peer review 아님, 다만 IEEE 2846-2022의 기반). ⚠️ `10.48550/arXiv.1708.06374`는 **DataCite** 등록이라 Crossref 조회로는 404가 난다 — DOI를 쓸 거면 arXiv ID를 함께 표기할 것.
- **[F18]** 🟡 P. G. Gipps, "A behavioural car-following model for computer simulation," *Transp. Res. B*, vol. 15, no. 2, pp. 105–111, Apr. 1981. DOI 10.1016/0191-2615(81)90037-0 (서지 ✅ Crossref). ⚠️ verbatim은 2차 문헌 **L. Lücken(단독 저자)**, "Resolving Collisions for the Gipps Car-Following Model," arXiv:1902.04927, 식 (5)(7)에서 읽음(저자 ✅ arXiv API 대조 — "et al."로 쓰지 말 것) → 인용 전 1981 원문 대조 필수.
- **[F19]** ✅ A. D. Ames, J. W. Grizzle, P. Tabuada, "Control barrier function based quadratic programs with application to adaptive cruise control," *Proc. 53rd IEEE CDC*, pp. 6271–6278, Dec. 2014. DOI 10.1109/CDC.2014.7040372 (서지 ✅ Crossref). 저자 배포 PDF(`ames.caltech.edu/CLF_QP_ACC_final.pdf`)로 (HC1) `D ≥ v/2`·`z ≥ 1.8x₂`, (SC1) `v−v_d → 0`, (SC1-CLF)의 `δ_sc`, (HC1-CBF) + "Since this is a hard constraint, no relaxation is used", §V-C의 (FC1)(FC2)·식 (38)(39) verbatim 대조 완료.
- **[F20]** ✅ A. D. Ames, **X. Xu**, J. W. Grizzle, P. Tabuada, "Control Barrier Function Based Quadratic Programs for Safety Critical Systems," *IEEE Trans. Autom. Control*, vol. 62, no. 8, pp. 3861–3876, Aug. 2017. DOI 10.1109/TAC.2016.2638961 (서지·저자 4인 ✅ Crossref). arXiv:1609.06408v2로 (43) 모델·`a_L ∈ [−a_l g, a'_l g]`, (44) `D − τ_d v_f ≥ 0`, `V(x) := (v_f−v_d)²`, (47) `A_clf=[L_gV,−1]`, (48) `A_cbf=[L_gB,0]`, `b_cbf=−L_fB+γ/B` verbatim 대조 완료.
- **[F21]** ✅ L. Hewing and M. N. Zeilinger, "Stochastic Model Predictive Control for Linear Systems Using Probabilistic Reachable Sets," *Proc. IEEE CDC 2018*, pp. 5182–5188. DOI 10.1109/CDC.2018.8619554 (서지 ✅ Crossref; arXiv:1805.07145v2 ✅ arXiv API) — (7)(8a)(8b)(9)(12a)(12b), Remark 3/5/6 대조.
- **[F22]** 🟡 D. Q. Mayne, M. M. Seron, S. V. Raković, "Robust model predictive control of constrained linear systems with bounded disturbances," *Automatica*, vol. 41, no. 2, pp. 219–224, Feb. 2005. DOI 10.1016/j.automatica.2004.08.019 (서지 ✅ Crossref / 수식 verbatim 미확인). **[F22b]** ✅ D. Q. Mayne, S. V. Raković, R. Findeisen, F. Allgöwer, "Robust output feedback model predictive control of constrained linear systems," *Automatica*, vol. 42, no. 7, pp. 1217–1222, Jul. 2006. DOI 10.1016/j.automatica.2006.03.005 (서지 ✅ Crossref) — 식 (6)(18) verbatim 확인.
- **[F23]** 🔴 **미확인(수식)** 개괄 인용용: M. Farina, L. Giulioni, R. Scattolini, "Stochastic linear Model Predictive Control with chance constraints – A review," *J. Process Control*, vol. 44, pp. 53–67, Aug. 2016. DOI 10.1016/j.jprocont.2016.03.005 / A. Mesbah, "Stochastic Model Predictive Control: An Overview and Perspectives for Future Research," *IEEE Control Syst. Mag.*, vol. 36, no. 6, pp. 30–44, Dec. 2016. DOI 10.1109/MCS.2016.2602087. 둘 다 서지 ✅ Crossref. **서베이 단독 인용은 약하다** — 반드시 [F13]/[F21]과 함께 쓸 것. (현행 부록 A는 Mesbah만 단독 인용 → [F13] 추가 권장.)

> **표에 없는 부록 A 항목**(Lefèvre et al. ROBOMECH 2014, ForzaETH arXiv:2403.11784, Predictive Spliner
> arXiv:2410.04868, Koschi & Althoff SPOT IEEE IV 2017)은 **이번 조사 범위 밖**이라 수식은 검증되지
> 않았다. arXiv 두 건은 ID·제목·저자만 arXiv API로 확인했다(2403.11784 = *ForzaETH Race Stack — Scaled
> Autonomous Head-to-Head Racing on Fully Commercial off-the-Shelf Hardware*; 2410.04868 = *Predictive
> Spliner: Data-Driven Overtaking in Autonomous Racing Using Opponent Trajectory Prediction*).
> 로드맵·계보 언급용으로는 무방하나, **수식 근거로 승격시키기 전에 별도 검증**할 것.

---

### B.5 이 제안이 문헌과 다른 점 (의도적 선택)

1. **혼합 공분산에서 위치 산포항을 배제한다** (E2 vs [F4][F5]). 총공분산 법칙의 엄밀 병합은
   `(dᵢ−d̄)²` 항을 더하지만, 우리 병합 멤버는 "동일 대상의 경쟁 가설"이 아니라 **한 차의 파편**이라
   그 산포는 **차체 기하**이고 이미 `size`/`d_left`/`d_right`와 §3.3의 `b_eff = (ego_half_width +
   lateral_margin) + (d_left − d_right)/2`에 반영되어 있다. 엄밀식을 그대로 쓰면 **같은 정보를 두 번**
   계상해 오히려 틀려진다. 대가로 대각(축별 max)과 교차항(가중평균)의 출처가 달라져 PSD가 깨질 수
   있고, E1의 클램프가 정확히 그 틈만 막는 최소 안전망이다.

2. **클램프는 "최근접 PSD"가 아니라 수치 여유를 둔 사영이다** (E1 vs [F1]). `0.99` 계수만큼
   프로베니우스 최적성을 의도적으로 희생해 `|ρ| ≤ 0.99`의 여유를 남긴다. 두 분산이 모두 양수면
   결과 블록은 PD(det = ab(1−0.99²) > 0), 어느 한쪽이 0이면 cov도 0으로 강제되어 PSD(특이)다.
   방향은 항상 "불확실성을 조금 덜 믿는" 쪽이고, `σ_g²(τ) ≥ 0`(§3.4 설계규칙 3의 판별식 조건)은
   여전히 보장된다 — 판별식 `4c² − 4ab ≤ 4(0.99²−1)ab < 0`(ab > 0), `ab = 0`이면 `c = 0`이라
   `σ² = a + t²b ≥ 0`. 이 클램프는 `kalmanUpdate()`의 대각 클램프(`obstacle_tracker.cpp:174`)가
   `P(2,2)=0`인데 `P(2,3)≠0`인 비PSD 블록을 만드는 경로까지 함께 막는다(상계가 0이 되어 cov 0).

3. **예측 전파에서 `Q`를 빼는 대신 지평을 `T ≤ 1.0 s`로 자른다** (E3 vs [F7][F8]). 표준 KF 예측은
   `+Q(k)`를 더하며 DWNA에서는 `(t⁴/4)σ_a²`가 장기에 지배한다. 우리는 이를 생략하는 대신 CV
   외삽의 유효 지평 자체를 상한으로 묶어 과소평가 구간에 들어가지 않게 한다.
   ⚠️ **기준 명시**: `T = 1.0 s`는 현행 **YAML 값**(`obstacle_detector.yaml:98`의
   `interference_time_horizon_sec: 1.0`)과 같지만, 검출기 **코드 기본값은 2.0**
   (`obstacle_detector_node.cpp:162`)이다. §3.3이 신설하는 파라미터 이름은
   `interference_horizon_sec`이므로, 선언 시 기본값을 **1.0으로 명시**해 §2.3이 지적한
   `interference_distance_m`(YAML 1.0 vs 코드 5.0)류의 불일치를 반복하지 않는다.
   근거: 상대 기동은 일시적이고 미래 횡위치는 본질적으로 바이모달이라 `t > 1 s`에서는 Q를 더해도
   단일 가우시안 가정 자체가 먼저 무너진다. `T > 1 s`가 필요해지면 상대 라인 학습(Predictive
   Spliner 계열)이 다음 단계이며 이 제안의 범위 밖이다.

4. **곱 분해 `P = P_lat·P_long`은 단일 트랙에서 엄밀, 병합에서는 강제된 가정이다**
   (E6 vs [F9][F10] 및 [F11]의 통상 caveat). 문헌이 경고하는 s–d 커플링은 **Cartesian 공분산을
   Frenet으로 투영할 때** 생기는데, 우리 트래커는 Frenet 상태 `[s,vs,d,vd]`를 네이티브로 추정하고
   `F`(`:50-52`)·`Q`(`:58-68`)·`P₀`(`:560-564`)가 블록대각, `R`(`:79-80`)이 대각이라
   `(I−KH)P`(`:168`)와 재대칭화(`:171`)가 블록대각을 보존한다 — **상대차가 트랙 1개로 유지되는
   통상적인 경우에는 근사가 아니라 엄밀**하다.
   ⚠️ 그러나 발행되는 것은 **병합 결과**다(`s_var/d_var`는 멤버별 max, `vs/vd`는 size-가중 평균,
   `s_center/d_center`는 기하 포락선 중점 `:838-843`). 한 차가 여러 파편으로 쪼개지면 파편들의
   s와 d는 회전한 차체 때문에 실제로 상관을 가지며, 그 상관은 `Obstacle.msg`에 s–d 교차 필드가
   없어 **스키마가 독립을 강제**할 뿐이다. 논문에는 "단일 트랙 시 엄밀 / 다중 파편 병합 시 모델
   가정"으로 정확히 쓰고, Phase 3 시뮬에서 **병합 멤버 수 분포**와 `|ρ_sd|`를 실측 보고한다.
   (`R` 대각도 동일하게 "설계상 강제한 블록대각"으로 서술한다.)

5. **RSS/Gipps의 반응시간 여유(ρ, τ, θ)를 확률적 조임 `z_ε·σ_g(τ_s)`로 치환한다** (E9 vs [F17][F18]).
   우리 `v_brake`는 `ρ=0, a_min,brake=a_max,brake` 특수화라 결정론적으로는 낙관적이며, 그 자리를
   `effective_gap = ĝ − z_ε·σ_g(τ_s)`와 `v_opp_lb = max(0, v̂s − z_v√vs_var)`가 대신한다.
   이는 **등가 보증이 아니라 치환**이다 — 지연이 σ에 반영되어 있다는 가정에 의존하므로,
   Phase 3 A/B에서 실제 검출→제어 지연(≈ 스캔 주기 + FSM 주기)이 `z_ε·σ_g(τ_s)`에 상당하는 여유를
   만드는지 실측으로 확인해야 한다. ⚠️ 기호 주의: 대칭 특수화에서 `a`는 **각 차량 자신의**
   감속도이므로 코드명 `relative_deceleration`은 '상대 감속도'라는 뜻이 아니다 — 논문에는 `a`로
   쓰고 코드명을 각주로 밝힌다.

6. **"안전=하드"는 CBF-QP가 아니라 `min()` 캡으로 구현된다** (E10 vs [F19][F20]). [F20] 식 (48)의
   `A_cbf = [L_gB, 0]`(슬랙 계수 0)이 주는 forward invariance 보증은 우리에게 없다. 차용한 것은
   **"완화 가능한 성능항과 완화 불가한 안전항을 분리한다"는 구조**뿐이며, 그 결과 액추에이터
   한계와의 양립성([F19] §V-C가 "these constraints will conflict with the torque values needed to
   satisfy the hard constraint (HC1-CBF)"라며 스스로 지적하고 별도 force-based CBF로 해결한 문제)은
   우리 쪽에서 별도로 논증되지 않은 상태다.

7. **확률 판정에는 σ 하한이 필요하다** (E5 구현 요구사항). `kalmanUpdate()`의 대각 클램프
   (`obstacle_tracker.cpp:174`)와 병합의 축별 max 때문에 `d_var = vd_var = 0`(또는
   `s_var = vs_var = 0`)이 원리적으로 도달 가능하고, 이때 §3.3의 `Φ((b_eff−μ)/σ)`가 0으로 나눈다.
   §3.3 구현은 σ에 하한(예: 1e-6 m)을 두거나 σ=0일 때 결정론적 지시함수로 분기해야 하며, 이는
   §7 단위 테스트의 "cov 극단값(클램프 경계)" 케이스에 **σ=0 케이스를 추가**해 회귀로 고정한다.
   (§3.4의 cruise는 σ_g를 곱하기만 하므로 이 경로가 없다.)
