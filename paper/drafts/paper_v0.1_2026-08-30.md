# Geometry-to-Root Mapping: Bounded Analytic Candidate Generation for Frenet Local Planning

**Working Draft v0.1 — August 2026**
**Authors: TBD**

## Abstract

Real-time local path planning for autonomous vehicles requires a balance between path-space coverage, physical feasibility, and computational cost. Sampling-based Frenet planners commonly generate a discrete set of polynomial trajectories and select a feasible candidate, while corridor-based planners often formulate a continuous optimization problem over multiple path variables. This work investigates a third approach: using environment geometry to construct a small number of position constraints on a low-dimensional \(C^2\) Frenet spline family and analytically inverting those constraints into a bounded set of path parameters.

We consider a five-knot piecewise-quintic Frenet path family parameterized primarily by a target lateral offset \(d_{\mathrm{target}}\) and an intermediate shape parameter \(d_{\mathrm{mid}}\). For a fixed spline branch and a geometry-derived position constraint

$$
d_{\mathrm{P3}}(s_{\mathrm{probe}};d_{\mathrm{target}},d_{\mathrm{mid}})
=
d_{\mathrm{probe}},
$$

the dependence of internal harmonic derivatives on \(d_{\mathrm{mid}}\) yields a linear-fractional form, and the resulting spline sample equation reduces to a polynomial of degree at most two. Consequently, each active position constraint produces at most two algebraic \(d_{\mathrm{mid}}\) candidates.

The central research question is therefore not how to densely search \(d_{\mathrm{mid}}\), but how to map the feasible obstacle corridor into a small set of informative geometric constraints \((s_{\mathrm{probe}},d_{\mathrm{probe}})\). We evaluate geometry-driven probe and anchor policies against the existing production heuristic, a broader P3 oracle, and sampling-based alternatives while retaining the same downstream hard validator, velocity shaping, candidate ranking, and temporal lifecycle. Evaluation is designed to measure oracle-relative feasible-path recovery, actual validator calls, candidate count, runtime, safety margins, temporal stability, real-recorded replay performance, and closed-loop simulation performance.

**Results: TBD after frozen-policy evaluation.**

---

## I. Introduction

Autonomous racing and high-speed mobile robotics require local planners to react to obstacles within a limited computation budget while generating paths that remain compatible with track boundaries, vehicle geometry, steering limits, and the available speed profile. A practical local planner must therefore solve two different problems. First, it must represent a sufficiently rich set of avoidance paths. Second, it must identify useful members of that path family without evaluating an unnecessarily large number of candidates.

A common solution in Frenet-frame planning is to discretize terminal states or trajectory parameters, construct multiple polynomial trajectories, validate them, and select the best feasible trajectory. This strategy has been highly influential in autonomous-driving trajectory planning [1], [2] and remains the basis of sampling-based implementations such as the CommonRoad Reactive Planner. Its principal advantage is simplicity and broad empirical coverage. Its principal computational trade-off is that increasing the discretization density increases the number of generated and validated trajectories.

An alternative is to first construct a collision-free or drivable corridor and optimize a path inside that corridor. Recent work such as the Frenet Corridor Planner formulates path generation as an optimization problem over a sequence of spatial path variables [3]. Such approaches can directly reason about the entire corridor, but online computation is then associated with a multi-variable constrained optimization problem.

This work investigates a different design point. Instead of densely sampling a continuous spline parameter or continuously optimizing a large set of path knots, we ask whether environmental geometry can identify a small number of informative equality constraints from which useful path parameters can be solved directly.

The motivation arose from the development of a local obstacle-avoidance planner based on a low-dimensional Frenet spline family. Early path families provided insufficient intermediate shape freedom in curved obstacle corridors. Introducing an intermediate lateral degree of freedom, \(d_{\mathrm{mid}}\), increased the representational capacity of the path family, but also introduced a new search dimension. Exhaustive evaluation can determine whether a feasible path exists within this family, but is unsuitable as the primary online candidate-generation mechanism.

The key observation is that the environment already contains information about where an avoidance path should pass. Track boundaries, vehicle footprint, safety margins, and obstacle geometry define a feasible lateral corridor

$$
C(s)=[l(s),u(s)].
$$

Rather than asking

$$
\text{“Which }d_{\mathrm{mid}}\text{ values should be sampled?”},
$$

we ask

$$
\text{“Which geometric conditions should the path satisfy?”}
$$

and then solve the spline parameter that satisfies those conditions.

For a position constraint

$$
d(s_{\mathrm{probe}})=d_{\mathrm{probe}},
$$

the considered P3 spline family permits analytic inversion of \(d_{\mathrm{mid}}\) within a fixed derivative branch. The sample equation is at most quadratic, yielding at most two algebraic roots per active condition. The continuous search over \(d_{\mathrm{mid}}\) can therefore be replaced by a bounded candidate-generation operation.

However, algebraic solvability alone does not solve the planning problem. The central difficulty becomes the mapping

$$
C(s),\kappa_{\mathrm{ref}}(s),O
\longrightarrow
(s_{\mathrm{probe}},d_{\mathrm{probe}}).
$$

The existing production implementation already exposes this unresolved design choice: different candidate templates use different geometry heuristics, including a reference-curvature-critical station and a minimum-width corridor bottleneck. This motivates a systematic study of which corridor features provide informative constraints for analytic spline inversion.

This paper therefore investigates whether a principled geometry-to-root mapping can reduce candidate generation and validation while preserving the feasible-path coverage of a substantially broader P3 search.

The intended contributions of this work are: a formulation that maps feasible-corridor geometry into bounded algebraic candidates of a low-dimensional \(C^2\) Frenet spline family; a controlled analysis of probe-location and lateral-anchor policies while fixing the remaining planner stack; an oracle-relative evaluation separating mapping failure from path-family infeasibility; and an evaluation protocol combining deterministic geometry tests, recorded real-system inputs, temporal replay, and closed-loop simulation.

These contributions remain hypotheses until the frozen-policy experiments reported later in this paper are completed.

---

## II. Related Work

### A. Sampling-Based Frenet Trajectory Generation

Werling et al. introduced a widely used Frenet-frame trajectory-generation framework in which polynomial trajectories are constructed from terminal-state conditions and evaluated according to feasibility and cost [1]. The subsequent formulation using discretized terminal manifolds further developed this approach for time-critical street scenarios [2].

Modern implementations such as the CommonRoad Reactive Planner retain the same general philosophy: a discrete set of quintic Frenet trajectories is sampled, candidates are checked for kinematic feasibility and collision, and a feasible trajectory is selected.

The approach considered in this paper differs in the candidate-generation stage. Rather than discretizing the primary intermediate spline-shape parameter, it first specifies a geometric position condition and solves the corresponding parameter analytically.

### B. Corridor-Based Path Optimization

Corridor-based planners reduce obstacle avoidance to path generation inside a feasible or drivable region. The Frenet Corridor Planner constructs a drivable Frenet corridor and subsequently solves a spatial optimization problem for a smooth collision-avoiding path [3].

Our formulation also begins with a Frenet feasible corridor. However, the corridor is not directly discretized into a high-dimensional path optimization problem. Instead, selected corridor features produce a small set of position equalities that are inverted into low-dimensional spline parameters. The corridor therefore acts as a source of candidate-generating constraints rather than as the complete optimization domain.

### C. Algebraic and Finite Candidate Generation

Analytic root computation has previously been used for efficient trajectory generation. Rauscher and Sawodny formulate trajectory-planning conditions for integrator-chain dynamics using polynomial elimination and compute candidate solutions through polynomial roots [4]. The use of polynomial roots is therefore not itself novel.

There is also a broader tradition of reducing continuous motion-planning spaces to finite candidate families. For example, geometric analysis of bounded-curvature Dubins problems can restrict the optimum to a finite set of analytically computable path candidates [5].

The distinction explored in this work is the use of obstacle- and corridor-derived spatial position conditions to analytically recover an intermediate parameter of a \(C^2\) Frenet avoidance spline. The research question is not whether algebraic roots can be used in trajectory planning, but whether local feasible geometry can provide sufficiently informative constraints that a small root set retains the useful coverage of a much larger parameter search.

---

## III. Problem Formulation

Let the global reference path be

$$
\mathbf r(s)=
\begin{bmatrix}
x_r(s)\\
y_r(s)
\end{bmatrix},
$$

where \(s\) denotes reference-path arc length. Let

$$
\mathbf t(s)
$$

and

$$
\mathbf n(s)
$$

denote the unit tangent and left normal of the reference path.

A local avoidance path is represented in Frenet coordinates by a lateral function \(d(s)\):

$$
\mathbf p(s)
=
\mathbf r(s)+d(s)\mathbf n(s).
$$

Given track boundaries and static-obstacle geometry, vehicle-width and safety-margin requirements define a vehicle-center feasible lateral set. A selected connected side corridor is represented as

$$
C(s)=[l(s),u(s)].
$$

The planner must produce a candidate path that satisfies the downstream hard-feasibility contract, including track and vehicle-footprint constraints, static-obstacle clearance, longitudinal waypoint ordering, lateral-slope bounds, spatial curvature-rate bounds, and directional curvature limits.

Speed is subsequently shaped using curvature-dependent lateral-acceleration constraints and longitudinal acceleration/deceleration feasibility.

Let

$$
\mathcal P_{\mathrm{P3}}
$$

denote the considered P3 spline family, and let

$$
\mathcal V(P)\in\{0,1\}
$$

denote the fixed production hard validator.

For a scene \(X\), define P3-family feasibility as

$$
\exists P\in\mathcal P_{\mathrm{P3}}
\quad\text{s.t.}\quad
\mathcal V(P)=1.
$$

A candidate-generation mapping \(M\) is successful on \(X\) if

$$
\exists P\in M(X)
\quad\text{s.t.}\quad
\mathcal V(P)=1.
$$

This allows the primary candidate-generation recovery metric to be defined relative to a broader P3 oracle:

$$
R_{\mathrm{oracle}}
=
\frac{
\#\{X:\mathcal P_{\mathrm{P3}}\text{ feasible and }M(X)\text{ recovers a valid path}\}
}{
\#\{X:\mathcal P_{\mathrm{P3}}\text{ feasible}\}
}.
$$

This metric deliberately separates a failure of the candidate mapping from a failure of the underlying path family.

---

## IV. Low-Dimensional P3 Path Family

The P3 lateral path is represented using five longitudinal knots,

$$
z_0<z_1<z_2<z_3<z_4,
$$

with lateral values

$$
[d(z_0),d(z_1),d(z_2),d(z_3),d(z_4)]
=
[d_{\mathrm{ego}},
d_{\mathrm{target}},
d_{\mathrm{mid}},
d_{\mathrm{target}},
0].
$$

The four adjacent intervals are connected using quintic Hermite segments. Each segment is determined by position, first derivative, and second derivative at both endpoints. Shared knot derivatives and accelerations are used so that the full lateral profile is \(C^2\).

The target lateral offset \(d_{\mathrm{target}}\) defines the primary avoidance side and lateral target, while

$$
d_{\mathrm{mid}}
$$

provides an additional intermediate shape degree of freedom.

This intermediate parameter is important in curved corridors because holding a single target lateral offset across the obstacle region may not adequately track the shape of the available free corridor.

The current study treats \(d_{\mathrm{target}}\) as fixed by the baseline deterministic target-selection policy during the first geometry-to-root experiments. This isolates the effect of the probe and anchor mapping from changes in the target-offset policy.

---

## V. Analytic Inversion of the Intermediate Offset

Consider a selected probe station

$$
s_{\mathrm{probe}}
$$

and desired lateral position

$$
d_{\mathrm{probe}}.
$$

The path is constrained by

$$
d_{\mathrm{P3}}
(
s_{\mathrm{probe}};
d_{\mathrm{target}},
d_{\mathrm{mid}}
)
=
d_{\mathrm{probe}}.
$$

Let

$$
x=d_{\mathrm{mid}}.
$$

The secant slopes adjacent to internal P3 knots depend affinely on \(x\). When neighboring secant slopes have the same sign, the internal derivative is generated using a weighted harmonic rule. Under a fixed derivative branch, the resulting derivative has a linear-fractional dependence on \(x\),

$$
m(x)
=
\frac{ax+b}{cx+e}.
$$

A quintic Hermite spline sample at a fixed longitudinal station is linear in its endpoint positions, derivatives, and second derivatives. Therefore, within a fixed branch, the probe sample can be written in the form

$$
d_{\mathrm{P3}}(s_{\mathrm{probe}};x)
=
\alpha x+\beta
+
w\frac{ax+b}{cx+e}.
$$

Applying the position constraint gives

$$
\alpha x+\beta
+
w\frac{ax+b}{cx+e}
=
d_{\mathrm{probe}}.
$$

After multiplying by the denominator,

$$
Ax^2+Bx+C=0,
$$

where

$$
A=\alpha c,
$$

$$
B=\alpha e+(\beta-d_{\mathrm{probe}})c+wa,
$$

and

$$
C=(\beta-d_{\mathrm{probe}})e+wb.
$$

Thus an active fixed-branch position condition produces at most two algebraic roots,

$$
d_{\mathrm{mid}}
\in
\{x_1,x_2\}.
$$

Each root remains only a candidate. It must satisfy the assumed derivative branch, parameter bounds, forward equation residual, exact spline reconstruction, and the unchanged downstream hard validator.

The analytic inversion therefore reduces candidate generation but does not replace safety validation.

---

## VI. Geometry-to-Root Mapping

The remaining planning problem is to construct useful constraints

$$
(s_{\mathrm{probe}},d_{\mathrm{probe}})
$$

from the feasible corridor.

We write the mapping as

$$
s_{\mathrm{probe}}
=
F(C,\kappa_{\mathrm{ref}},O,\ldots),
$$

and

$$
d_{\mathrm{probe}}
=
G(C(s_{\mathrm{probe}}),d_{\mathrm{target}},\ldots).
$$

The current production implementation already contains multiple heuristic choices. One policy selects an internal corridor station with maximum reference-curvature magnitude and projects \(d_{\mathrm{target}}\) into an additionally inset feasible interval. Another analytic policy selects the minimum-width corridor station and uses the corridor center as the lateral anchor.

The proposed research therefore does not initially assume that either rule is optimal. Instead, a controlled policy study compares a small set of globally defined geometry mappings.

Initial probe-location hypotheses include the reference-curvature-critical location,

$$
F_{\kappa}
=
\arg\max_s|\kappa_{\mathrm{ref}}(s)|,
$$

the minimum-width bottleneck,

$$
F_w
=
\arg\min_s [u(s)-l(s)],
$$

and the maximum corridor-center displacement relative to the target offset,

$$
F_c
=
\arg\max_s
\left|
\frac{l(s)+u(s)}{2}
-d_{\mathrm{target}}
\right|.
$$

Initial lateral-anchor hypotheses include the corridor center,

$$
G_c
=
\frac{l(s_{\mathrm{probe}})+u(s_{\mathrm{probe}})}{2},
$$

and a feasible projection of \(d_{\mathrm{target}}\),

$$
G_p
=
\operatorname{clip}
(
d_{\mathrm{target}},
l(s_{\mathrm{probe}})+m,
u(s_{\mathrm{probe}})-m
).
$$

The final mapping used for evaluation will be selected using training data only and frozen before evaluation on new geometry.

If a single position constraint is empirically insufficient across relevant corridor classes, the formulation can be extended to a small bounded set of probe hypotheses. The objective remains a bounded candidate count rather than a dense discretization of \(d_{\mathrm{mid}}\).

---

## VII. Fixed Downstream Planning Stack

To isolate the effect of candidate generation, the first study keeps the downstream planner unchanged.

All generated candidates use the same P3 spline reconstruction, velocity shaping, hard validator, candidate-ranking comparator, and temporal lifecycle as the production baseline.

The hard validator checks the continuity between the current ego state and the candidate entry, sufficient forward path, center track bounds, rotated vehicle footprint against track boundaries, static-obstacle collision, increasing longitudinal order, lateral slope, spatial curvature rate, and directional curvature limits.

Lateral acceleration and longitudinal acceleration/deceleration are handled primarily through velocity-profile shaping rather than by changing the path family.

Only hard-valid candidates enter ranking. The production ranker uses a lexicographic comparator rather than a weighted sum. The priority is: avoiding exit overlap with the next obstacle, braking feasibility, braking-distance deficit, velocity loss, minimum normalized safety slack, global-path deviation, and deterministic generation order.

The temporal lifecycle is also frozen. An active P3 maneuver is continued while its remaining suffix remains valid under current geometry, rather than reselecting a fresh path at every frame. This allows candidate-generation changes to be evaluated without simultaneously altering path-commitment behavior.

---

## VIII. Experimental Methodology

### A. Dataset Structure

Evaluation will use three conceptually separated datasets.

**GEOMETRY_CORE** contains deterministic or simulation-based scenes with trusted localization or ground-truth geometry. It is used for policy development, oracle comparisons, and controlled ablation.

**REAL_REPLAY_CLEAN** contains recorded real-system planning windows for which localization quality is independently judged sufficiently reliable. These data are used to test candidate generation on real sensor and planner inputs.

**LOCALIZATION_STRESS** contains recorded windows with localization discontinuities or other localization-quality concerns. These data are not mixed into the primary candidate-generation metric and are used only for robustness analysis.

Recorded replay is not treated as counterfactual closed-loop evaluation because a newly generated path would alter the vehicle’s subsequent state and sensor observations. Closed-loop performance is therefore evaluated separately in simulation.

### B. Baselines

The primary baseline is the frozen production P3 candidate generator and its existing M0-V1, M0-V2, and M1 template ladder.

A broader P3 oracle evaluates a substantially denser or otherwise more complete search over the same P3 family under the same validator. The oracle is not an online competitor; it is used to determine whether the underlying P3 family contains a feasible solution.

Additional controlled baselines include the individual production geometry policies and budget-matched parameter-sampling strategies.

The final proposed geometry-to-root policy is frozen before validation and holdout evaluation.

### C. Primary Metrics

The primary metric is P3-oracle-relative feasible-path recovery.

Computational metrics include actual constructed candidate count, actual executed validator calls, root-solver count, and end-to-end candidate-generation runtime. Internal discarded-side computations, strict/relaxed attempts, and repeated validation are counted rather than relying only on the externally retained candidate cap.

Path-quality metrics include track margin, obstacle margin, curvature margin, curvature-rate margin, lateral-slope margin, velocity loss, braking-distance deficit, and deviation from the reference line.

Temporal metrics include selected-path identity changes, avoidance-side switches, continuation duration, fresh-replanning frequency, invalidation frequency, and safe-stop transitions.

Closed-loop simulation additionally measures collision rate, off-track rate, completion rate, planner failures, speed loss, and path-tracking behavior.

---

## IX. Research Questions

**RQ1 — Coverage.** Can geometry-derived position constraints recover a high fraction of hard-valid paths known to exist within the P3 family?

**RQ2 — Computation.** Can this recovery be achieved with materially fewer constructed candidates and hard-validator calls than the production heuristic ladder or sampling-based alternatives?

**RQ3 — Geometric informativeness.** Which corridor features provide the most informative longitudinal probe locations and lateral anchors for analytic inversion?

**RQ4 — Generalization.** Does a mapping selected on training geometries retain its recovery and computation properties on unseen obstacle layouts, curved corridors, and recorded real-system inputs?

**RQ5 — Temporal behavior.** Does reducing the candidate set introduce instability at fresh replanning events, or can bounded analytic candidate generation retain the temporal stability of the existing continuation lifecycle?

---

## X. Planned Results

### A. Main Offline Geometry Result

| Method                    | Oracle-Relative Recovery | Constructed Candidates | Actual Validator Calls | Runtime p50 | Runtime p95 |
| ------------------------- | -----------------------: | ---------------------: | ---------------------: | ----------: | ----------: |
| Production P3             |                      TBD |                    TBD |                    TBD |         TBD |         TBD |
| Budget-Matched Sampling   |                      TBD |                    TBD |                    TBD |         TBD |         TBD |
| Curvature Probe           |                      TBD |                    TBD |                    TBD |         TBD |         TBD |
| Bottleneck Probe          |                      TBD |                    TBD |                    TBD |         TBD |         TBD |
| Center-Displacement Probe |                      TBD |                    TBD |                    TBD |         TBD |         TBD |
| Proposed Frozen Mapping   |                      TBD |                    TBD |                    TBD |         TBD |         TBD |

### B. Mapping Failure Analysis

| Failure Category             | Production | Proposed |
| ---------------------------- | ---------: | -------: |
| No algebraic candidate       |        TBD |      TBD |
| Branch/bound rejection       |        TBD |      TBD |
| Track-bound rejection        |        TBD |      TBD |
| Obstacle collision           |        TBD |      TBD |
| Curvature rejection          |        TBD |      TBD |
| Curvature-rate rejection     |        TBD |      TBD |
| Lateral-slope rejection      |        TBD |      TBD |
| Oracle-feasible mapping miss |        TBD |      TBD |

### C. Path Quality

| Method     | Min. Track Margin | Min. Obstacle Margin | Curvature Slack | Rate Slack | Velocity Loss |
| ---------- | ----------------: | -------------------: | --------------: | ---------: | ------------: |
| Production |               TBD |                  TBD |             TBD |        TBD |           TBD |
| Proposed   |               TBD |                  TBD |             TBD |        TBD |           TBD |

### D. Closed-Loop Evaluation

| Method     | Success | Collision | Off-Track | Planner Failure | Completion Time |
| ---------- | ------: | --------: | --------: | --------------: | --------------: |
| Production |     TBD |       TBD |       TBD |             TBD |             TBD |
| Proposed   |     TBD |       TBD |       TBD |             TBD |             TBD |

---

## XI. Discussion

The proposed formulation deliberately separates three questions that are often conflated in candidate-based local planning.

The first is **path-family expressiveness**: does the P3 family contain a feasible path?

The second is **candidate mapping**: if such a path exists, can the online generator identify a useful parameter value using only a small number of geometric conditions?

The third is **candidate selection**: given multiple valid candidates, which path should the ranker select?

This distinction is essential for interpreting failure cases. A planner failure should not automatically be attributed to insufficient spline expressiveness. If a broader P3 oracle finds a valid path while the online mapping does not, the failure is a mapping-coverage problem. Conversely, if the oracle also fails, changing only the probe-selection policy cannot resolve the case.

This framework also provides a direct method for evaluating computation. The relevant quantity is not merely the number of candidates retained in the final planner result, but the total computation scheduled in producing, reconstructing, validating, discarding, and revalidating candidates.

A successful geometry-to-root method should therefore demonstrate not only low nominal candidate count but a reduction in actual validation work while maintaining or improving oracle-relative recovery.

---

## XII. Limitations

The geometry-to-root formulation does not provide completeness over arbitrary collision-free trajectories. It is limited by the expressiveness of the chosen five-knot P3 family and by the information retained in the selected geometric constraints.

A single probe condition cannot by itself certify full-corridor feasibility. All generated paths must therefore continue to pass the exact downstream validator.

The current vehicle-feasibility model is dynamics-aware but is not a complete kinodynamic rollout. Steering-angle geometry, spatial curvature rate, lateral-acceleration-based speed shaping, and longitudinal acceleration/deceleration are modeled, but full tire-force coupling, steering-actuator dynamics, and closed-loop uncertainty propagation are outside the first study.

Recorded rosbag replay provides real-system inputs but cannot reproduce the counterfactual future sensor stream that would have resulted from following a newly generated path. Closed-loop claims must therefore be based on simulation or future real-vehicle experiments rather than replay alone.

Finally, the use of analytic roots and finite candidate sets is not itself claimed as novel. The novelty claim, if supported by the final literature review and experiments, must concern the specific geometry-to-algebraic-candidate formulation, its bounded computational structure, and its demonstrated coverage/generalization properties.

---

## XIII. Conclusion

This paper studies whether feasible-corridor geometry can replace dense parameter exploration with a small, analytically generated candidate set for Frenet local planning. The considered P3 family provides an intermediate lateral shape parameter whose value can be recovered from a geometry-derived position constraint through an at-most-quadratic equation within a fixed spline branch.

The remaining challenge is to construct geometric constraints that are informative enough to retain feasible-path coverage across diverse corridor geometries. By separating path-family feasibility, candidate mapping, validation, ranking, and temporal lifecycle, the proposed evaluation framework is designed to determine whether candidate reduction is achieved through principled geometry rather than through loss of coverage.

**Final quantitative conclusion: TBD after frozen-policy, holdout, real-replay, and closed-loop evaluation.**

---

## References

[1] M. Werling, J. Ziegler, S. Kammel, and S. Thrun, “Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame,” *IEEE International Conference on Robotics and Automation (ICRA)*, pp. 987–993, 2010. doi: 10.1109/ROBOT.2010.5509799.

[2] M. Werling, S. Kammel, J. Ziegler, and L. Gröll, “Optimal Trajectories for Time-Critical Street Scenarios Using Discretized Terminal Manifolds,” *The International Journal of Robotics Research*, vol. 31, no. 3, pp. 346–359, 2012. doi: 10.1177/0278364911423042.

[3] F. M. Tariq, Z.-H. Yeh, A. Singh, D. Isele, and S. Bae, “Frenet Corridor Planner: An Optimal Local Path Planning Framework for Autonomous Driving,” *IEEE Intelligent Vehicles Symposium (IV)*, 2025. doi: 10.1109/IV64158.2025.11097649.

[4] F. Rauscher and O. Sawodny, “Efficient Online Trajectory Planning for Integrator Chain Dynamics Using Polynomial Elimination,” *IEEE Robotics and Automation Letters*, 2021. doi: 10.1109/LRA.2021.3072857.

[5] B. Jha, Z. Chen, and T. Shima, “On Shortest Dubins Path via a Circular Boundary,” *Automatica*, vol. 121, Art. no. 109192, 2020. doi: 10.1016/j.automatica.2020.109192.
