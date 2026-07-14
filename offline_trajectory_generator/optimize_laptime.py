"""Differentiable lap-time raceline optimizer (GPU, torch or Apple MLX).

Instead of minimizing curvature as a proxy (``optimize_min_curvature`` +
hand-tuned curvature/smooth/length weights), this module minimizes the lap
time itself by gradient descent through a differentiable vehicle model:

  1. The raceline is parameterized as a lateral offset ``alpha`` along the
     centerline normals, mapped through a sigmoid so it can never leave the
     drivable corridor: ``alpha = lower + (upper - lower) * sigmoid(theta)``.
  2. Speed at every waypoint is the curvature-limited speed
     ``v = sqrt(a_lat / |kappa|)`` capped at ``max_speed``, then propagated
     through the longitudinal accel/decel limits with an exact min-plus
     transitive closure (doubling trick, O(N log N), fully vectorized and
     differentiable — the same physics as ``velocity_profile`` but batched).
  3. Loss = lap time  +  smoothness regularizer. Gradients flow from the lap
     time back into ``theta`` because every step above is differentiable.

Multi-start: B random initializations (plus the corridor center and an
optional min-curvature warm start) are optimized *simultaneously* as one
(B, N) batch — on CUDA (torch) or Apple Metal (MLX) this is where the GPU
parallelism pays off. The restart with the lowest lap time wins.

Backends: ``torch`` (CUDA/CPU, Ubuntu) or ``mlx`` (Apple Silicon). Selected
automatically; both share the same loss code via a tiny ops adapter, so the
two backends compute identical math.

This module is imported lazily by ``generate_global_trajectory.py`` when
``--optimizer laptime`` is selected; the base generator keeps running with
plain numpy/scipy when neither framework is installed.
"""

from __future__ import annotations

import importlib.util
import math
import time
from dataclasses import dataclass, replace

import numpy as np

_BACKENDS = ("torch", "mlx")


def resolve_backend(name: str | None = "auto") -> str:
    """Resolve 'auto' to an installed framework (torch preferred, mlx fallback)."""
    name = (name or "auto").strip().lower()
    if name == "auto":
        for candidate in _BACKENDS:
            if importlib.util.find_spec(candidate) is not None:
                return candidate
        raise ImportError(
            "The lap-time optimizer needs 'torch' (any platform, CUDA for GPU) "
            "or 'mlx' (Apple Silicon macOS). Install one, or use --optimizer mincurv."
        )
    if name not in _BACKENDS:
        raise ValueError(f"Unknown laptime backend '{name}'; expected auto/torch/mlx.")
    if importlib.util.find_spec(name) is None:
        raise ImportError(f"Requested --laptime-backend {name} but '{name}' is not installed.")
    return name


@dataclass(frozen=True)
class LapTimeParams:
    """Optimizer settings, filled from the generator CLI args."""

    iters: int = 2000
    restarts: int = 16
    lr: float = 0.08
    smooth_weight: float = 0.2
    init_spread: float = 2.0
    seed: int = 0
    backend: str = "auto"
    device: str = "auto"  # torch only: auto/cpu/cuda
    # vehicle limits (same semantics as velocity_profile)
    v_max: float = 4.0
    v_min: float = 1.0
    a_lat: float = 4.0
    a_acc: float = 3.0
    a_dec: float = 5.0
    # steering limit: bends sharper than this are undrivable (0 disables)
    kappa_max: float = 1.2
    kappa_weight: float = 20.0


def params_from_args(args) -> LapTimeParams:
    return LapTimeParams(
        iters=int(getattr(args, "laptime_iters", 2000)),
        restarts=int(getattr(args, "laptime_restarts", 16)),
        lr=float(getattr(args, "laptime_lr", 0.08)),
        smooth_weight=float(getattr(args, "laptime_smooth_weight", 0.2)),
        init_spread=float(getattr(args, "laptime_init_spread", 2.0)),
        seed=int(getattr(args, "laptime_seed", 0)),
        backend=str(getattr(args, "laptime_backend", "auto")),
        device=str(getattr(args, "laptime_device", "auto")),
        v_max=float(args.max_speed),
        v_min=float(args.min_speed),
        a_lat=float(args.max_lateral_accel),
        a_acc=float(args.max_accel),
        a_dec=float(args.max_decel),
        kappa_max=float(getattr(args, "max_curvature", 1.2)),
    )


# --------------------------------------------------------------------------- #
# Shared differentiable model (backend-agnostic via the `ops` adapter)
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class _Ops:
    """The few tensor ops the loss needs, bound per backend."""

    roll: callable  # roll(x, shift, axis)
    sqrt: callable
    abs: callable
    minimum: callable
    maximum: callable
    sigmoid: callable
    atan2: callable
    sin: callable
    cos: callable
    sum: callable  # sum(x, axis)
    mean0: callable  # mean over everything -> scalar


def _lap_time_terms(ops: _Ops, theta, center, normals, lower, span, p: LapTimeParams,
                    smooth_w=None):
    """Return (per-restart loss (B,), per-restart lap time (B,)).

    theta: (B, N); center/normals: (N, 2); lower/span: (N,).
    smooth_w: optional per-restart smoothness weights (B,) — the AI portfolio
    optimizes several regularization strengths in one batch; None keeps the
    single ``p.smooth_weight`` for all restarts.
    Every operation is differentiable in theta.
    """
    alpha = lower + span * ops.sigmoid(theta)  # (B, N), always inside the corridor
    pts = center[None, :, :] + alpha[:, :, None] * normals[None, :, :]  # (B, N, 2)

    d = ops.roll(pts, -1, 1) - pts  # segment vectors i -> i+1
    seg2 = ops.sum(d * d, 2)
    seg = ops.sqrt(seg2 + 1e-12)  # (B, N)

    psi = ops.atan2(d[:, :, 1], d[:, :, 0])
    dpsi_raw = ops.roll(psi, -1, 1) - psi
    dpsi = ops.atan2(ops.sin(dpsi_raw), ops.cos(dpsi_raw))  # wrapped heading change
    kappa = dpsi / (0.5 * (seg + ops.roll(seg, -1, 1)) + 1e-9)

    # Curvature-limited squared speed clipped to [v_min, v_max] (identical to
    # velocity_profile: the accel/decel closure below may still go under v_min).
    u = p.a_lat / (ops.abs(kappa) + 1e-4)
    u = ops.minimum(u, u * 0.0 + p.v_max * p.v_max)
    u = ops.maximum(u, u * 0.0 + p.v_min * p.v_min)

    # Longitudinal limits as an exact min-plus transitive closure (doubling):
    # forward (accel): u[i] <= min_j (u[i-j] + 2*a_acc*dist(i-j, i))
    # backward (decel): u[i] <= min_j (u[i+j] + 2*a_dec*dist(i, i+j))
    n = int(theta.shape[1])
    w_fwd = ops.roll(seg, 1, 1)  # distance from i-1 to i
    w_bwd = seg  # distance from i to i+1
    shift = 1
    while shift < n:
        u = ops.minimum(u, ops.roll(u, shift, 1) + 2.0 * p.a_acc * w_fwd)
        u = ops.minimum(u, ops.roll(u, -shift, 1) + 2.0 * p.a_dec * w_bwd)
        w_fwd = w_fwd + ops.roll(w_fwd, shift, 1)
        w_bwd = w_bwd + ops.roll(w_bwd, -shift, 1)
        shift *= 2

    v = ops.sqrt(ops.maximum(u, u * 0.0 + 1e-2))  # floor 0.1 m/s
    lap = ops.sum(seg / v, 1)  # (B,)

    dalpha = alpha - ops.roll(alpha, 1, 1)
    smooth = ops.sum(dalpha * dalpha, 1)
    loss = lap + (p.smooth_weight * smooth if smooth_w is None else smooth_w * smooth)
    if p.kappa_max > 0.0:
        # Steering limit: the speed model only slows down for curvature, so
        # without this term a v_min-floored kink is "free" — the optimizer
        # would happily cut corners the car cannot physically steer through.
        excess = ops.maximum(ops.abs(kappa) - p.kappa_max, kappa * 0.0)
        loss = loss + p.kappa_weight * ops.sum(excess * excess * seg, 1)
    return loss, lap


def _lr_at(it: int, p: LapTimeParams) -> float:
    """Step decay: full lr for 60% of the run, then x0.3, then x0.1."""
    frac = it / max(p.iters, 1)
    if frac < 0.6:
        return p.lr
    if frac < 0.85:
        return p.lr * 0.3
    return p.lr * 0.1


def _build_inits(
    lower: np.ndarray,
    span: np.ndarray,
    p: LapTimeParams,
    warm_alpha: np.ndarray | None,
) -> np.ndarray:
    """Initial theta batch: corridor center, optional warm start, random spreads."""
    n = len(lower)
    rng = np.random.default_rng(p.seed)
    inits = [np.zeros(n)]  # sigmoid(0)=0.5 -> middle of the corridor
    if warm_alpha is not None:
        frac = np.clip((warm_alpha - lower) / np.maximum(span, 1e-9), 1e-4, 1.0 - 1e-4)
        inits.append(np.log(frac / (1.0 - frac)))  # logit: reproduce the warm line
    while len(inits) < max(int(p.restarts), len(inits)):
        inits.append(rng.normal(0.0, p.init_spread, size=n))
    return np.stack(inits).astype(np.float32)


# --------------------------------------------------------------------------- #
# Backend training loops
# --------------------------------------------------------------------------- #

def _run_torch(theta0, center, normals, lower, span, p: LapTimeParams, log,
               smooth_vec: np.ndarray | None = None):
    import torch

    device = p.device
    if device == "auto":
        device = "cuda" if torch.cuda.is_available() else "cpu"
    if device.startswith("cuda") and not torch.cuda.is_available():
        device = "cpu"
    log(f"[laptime] backend=torch device={device}")

    def t(x):
        return torch.tensor(np.asarray(x, dtype=np.float32), device=device)

    ops = _Ops(
        roll=lambda x, s, a: torch.roll(x, s, a),
        sqrt=torch.sqrt, abs=torch.abs,
        minimum=torch.minimum, maximum=torch.maximum,
        sigmoid=torch.sigmoid, atan2=torch.atan2, sin=torch.sin, cos=torch.cos,
        sum=lambda x, a: torch.sum(x, dim=a),
        mean0=torch.mean,
    )
    c, nrm, lo, sp = t(center), t(normals), t(lower), t(span)
    sw = t(smooth_vec) if smooth_vec is not None else None
    theta = torch.tensor(theta0, device=device, requires_grad=True)
    opt = torch.optim.Adam([theta], lr=p.lr)

    for it in range(p.iters):
        for group in opt.param_groups:
            group["lr"] = _lr_at(it, p)
        opt.zero_grad()
        losses, laps = _lap_time_terms(ops, theta, c, nrm, lo, sp, p, sw)
        loss = losses.sum()
        loss.backward()
        opt.step()
        if it % max(p.iters // 5, 1) == 0 or it == p.iters - 1:
            log(f"[laptime] iter {it:4d} best_lap={float(laps.min()):.3f}s")

    with torch.no_grad():
        losses, laps = _lap_time_terms(ops, theta, c, nrm, lo, sp, p, sw)
        alpha = lo + sp * torch.sigmoid(theta)
    return alpha.cpu().numpy(), laps.cpu().numpy()


def _run_mlx(theta0, center, normals, lower, span, p: LapTimeParams, log,
             smooth_vec: np.ndarray | None = None):
    import mlx.core as mx

    log("[laptime] backend=mlx (Apple Metal)")
    ops = _Ops(
        roll=lambda x, s, a: mx.roll(x, s, a),
        sqrt=mx.sqrt, abs=mx.abs,
        minimum=mx.minimum, maximum=mx.maximum,
        sigmoid=mx.sigmoid, atan2=mx.arctan2, sin=mx.sin, cos=mx.cos,
        sum=lambda x, a: mx.sum(x, axis=a),
        mean0=mx.mean,
    )
    c = mx.array(np.asarray(center, dtype=np.float32))
    nrm = mx.array(np.asarray(normals, dtype=np.float32))
    lo = mx.array(np.asarray(lower, dtype=np.float32))
    sp = mx.array(np.asarray(span, dtype=np.float32))
    sw = mx.array(np.asarray(smooth_vec, dtype=np.float32)) if smooth_vec is not None else None
    theta = mx.array(theta0)

    def objective(th):
        losses, _ = _lap_time_terms(ops, th, c, nrm, lo, sp, p, sw)
        return mx.sum(losses)

    grad_fn = mx.value_and_grad(objective)

    # Plain Adam on a single tensor (mlx.optimizers targets nn parameter trees).
    m = mx.zeros_like(theta)
    v = mx.zeros_like(theta)
    b1, b2, eps = 0.9, 0.999, 1e-8
    for it in range(p.iters):
        _, g = grad_fn(theta)
        m = b1 * m + (1.0 - b1) * g
        v = b2 * v + (1.0 - b2) * g * g
        mh = m / (1.0 - b1 ** (it + 1))
        vh = v / (1.0 - b2 ** (it + 1))
        theta = theta - _lr_at(it, p) * mh / (mx.sqrt(vh) + eps)
        mx.eval(theta, m, v)
        if it % max(p.iters // 5, 1) == 0 or it == p.iters - 1:
            _, laps = _lap_time_terms(ops, theta, c, nrm, lo, sp, p)
            mx.eval(laps)
            log(f"[laptime] iter {it:4d} best_lap={float(laps.min()):.3f}s")

    losses, laps = _lap_time_terms(ops, theta, c, nrm, lo, sp, p)
    alpha = lo + sp * mx.sigmoid(theta)
    mx.eval(alpha, laps)
    return np.array(alpha), np.array(laps)


# --------------------------------------------------------------------------- #
# Public entry points
# --------------------------------------------------------------------------- #

def _corridor(
    center_xy: np.ndarray,
    d_right: np.ndarray,
    d_left: np.ndarray,
    safety_width: float,
    boundary_margin: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Normals + [lower, upper] lateral-offset bounds (same as optimize_min_curvature)."""
    heading = np.roll(center_xy, -1, axis=0) - center_xy
    psi = np.arctan2(heading[:, 1], heading[:, 0])
    normals = np.stack([-np.sin(psi), np.cos(psi)], axis=1)
    clearance = safety_width * 0.5 + boundary_margin
    lower = -np.maximum(d_right - clearance, 0.0)
    upper = np.maximum(d_left - clearance, 0.0)
    return normals, lower, upper, upper - lower


def optimize_lap_time(
    center_xy: np.ndarray,
    d_right: np.ndarray,
    d_left: np.ndarray,
    safety_width: float,
    boundary_margin: float,
    args,
    warm_alpha: np.ndarray | None = None,
    log=print,
) -> tuple[np.ndarray, dict]:
    """Gradient-descend the lap time over the lateral-offset raceline.

    Same call contract as ``optimize_min_curvature`` (plus an optional
    min-curvature warm start): returns the optimized (N, 2) raceline and an
    info dict with per-restart lap times.
    """
    p = params_from_args(args)
    backend = resolve_backend(p.backend)

    normals, lower, upper, span = _corridor(
        center_xy, d_right, d_left, safety_width, boundary_margin
    )
    if np.all(span <= 1e-3):
        log("[laptime] corridor is empty (track too narrow for safety width); keeping centerline")
        return center_xy, {"laps": [], "backend": backend}

    theta0 = _build_inits(lower, span, p, warm_alpha)
    t0 = time.time()
    runner = _run_torch if backend == "torch" else _run_mlx
    alphas, laps = runner(theta0, center_xy, normals, lower, span, p, log)
    elapsed = time.time() - t0

    best = int(np.argmin(laps))
    log(
        f"[laptime] done in {elapsed:.1f}s: restarts={len(laps)} "
        f"best_lap={laps[best]:.3f}s worst={laps.max():.3f}s (pick #{best})"
    )
    raceline = center_xy + normals * alphas[best][:, None]
    info = {
        "backend": backend,
        "laps": [float(x) for x in laps],
        "best_index": best,
        "elapsed_s": elapsed,
    }
    return raceline, info


# --------------------------------------------------------------------------- #
# AI portfolio search (--optimizer ai): GD portfolio + exact rescoring + ES
# --------------------------------------------------------------------------- #

_SMOOTH_PORTFOLIO = (0.25, 1.0, 4.0)  # multipliers on laptime_smooth_weight
_SPREAD_PORTFOLIO = (0.8, 2.0, 4.0)   # random-init stddevs (logit space)


def _ai_inits(
    lower: np.ndarray,
    span: np.ndarray,
    restarts: int,
    seed: int,
    seed_alphas: list[np.ndarray],
) -> np.ndarray:
    """Init batch anchored on the known-good lines (mincurv warm start, incumbent).

    Most restarts are the anchor logits plus wrap-smoothed noise of increasing
    amplitude — the search refines AROUND the min-curvature solution instead of
    starting from wall-to-wall random lines (those converge to wavy, barely
    faster local optima that look nothing like a raceline). Every 4th restart
    stays fully random for global exploration.
    """
    from scipy.ndimage import gaussian_filter1d

    n = len(lower)
    rng = np.random.default_rng(seed)
    anchors = [np.zeros(n)]  # corridor center
    for alpha in seed_alphas:
        frac = np.clip((alpha - lower) / np.maximum(span, 1e-9), 1e-4, 1.0 - 1e-4)
        anchors.append(np.log(frac / (1.0 - frac)))
    inits = list(anchors)
    perturb_scales = (0.3, 0.8, 1.5)
    while len(inits) < max(int(restarts), len(inits)):
        k = len(inits)
        if k % 4 == 3:  # global exploration
            inits.append(rng.normal(0.0, _SPREAD_PORTFOLIO[k % len(_SPREAD_PORTFOLIO)], size=n))
            continue
        base = anchors[1 + (k % max(len(anchors) - 1, 1))] if len(anchors) > 1 else anchors[0]
        noise = gaussian_filter1d(rng.normal(0.0, 1.0, n), 3.0, mode="wrap")
        noise *= perturb_scales[k % len(perturb_scales)] / (np.std(noise) + 1e-9)
        inits.append(base + noise)
    return np.stack(inits).astype(np.float32)


def _es_polish(
    alpha: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    exact,
    rng: np.random.Generator,
    rounds: int = 30,
    lam: int = 16,
    sigma: float = 0.10,
    smooth_px: float = 3.0,
    log=print,
) -> tuple[np.ndarray, float]:
    """(1+lambda) evolution strategy directly on the EXACT lap time.

    Gradient descent optimizes the differentiable surrogate; this derivative-free
    polish optimizes the true reported objective (velocity_profile lap time), so
    it can exploit model mismatch and escape flat/sticky gradient regions.
    Perturbations are wrap-around-smoothed noise fields clipped to the corridor.
    """
    from scipy.ndimage import gaussian_filter1d

    best = alpha.copy()
    best_lap = exact(best)
    start_lap = best_lap
    n = len(alpha)
    for _ in range(rounds):
        improved = False
        for _ in range(lam):
            noise = gaussian_filter1d(rng.normal(0.0, 1.0, n), smooth_px, mode="wrap")
            scale = float(np.std(noise))
            if scale < 1e-9:
                continue
            cand = np.clip(best + noise * (sigma / scale), lower, upper)
            lap = exact(cand)
            if lap < best_lap - 1e-4:
                best, best_lap = cand, lap
                improved = True
        if not improved:
            sigma *= 0.6
            if sigma < 0.01:
                break
    # De-wiggle sweep AFTER convergence: deterministic smoothing proposals,
    # accepted only when they improve the score, so leftover wiggles that were
    # not actually buying lap time flatten out and the line stays
    # raceline-shaped. (Interleaving these DURING the search greedily drags it
    # into a smoother-but-slower basin — measured 14.3 s -> 15.3 s — so they
    # must stay a post-pass.)
    for _ in range(10):
        smoothed_any = False
        for smooth_sigma in (1.0, 2.0, 4.0):
            cand = np.clip(gaussian_filter1d(best, smooth_sigma, mode="wrap"), lower, upper)
            lap = exact(cand)
            if lap < best_lap - 1e-4:
                best, best_lap = cand, lap
                smoothed_any = True
        if not smoothed_any:
            break
    if best_lap < start_lap - 1e-4:
        log(f"[ai]   ES polish: score {start_lap:.3f} -> {best_lap:.3f}")
    return best, best_lap


def optimize_lap_time_ai(
    center_xy: np.ndarray,
    d_right: np.ndarray,
    d_left: np.ndarray,
    safety_width: float,
    boundary_margin: float,
    args,
    evaluate,
    warm_alpha: np.ndarray | None = None,
    log=print,
) -> tuple[np.ndarray, dict]:
    """Multi-technique lap-time search (``--optimizer ai``, GUI "AI Optimize").

    Runs ``--ai-epochs`` alternation rounds; each epoch applies three techniques:
      1. GPU gradient descent on the differentiable model, with a PORTFOLIO of
         smoothness weights spread across the restart batch (one batched run
         explores several regularization strengths at once).
      2. Exact rescoring: every restart is re-evaluated with the true
         ``velocity_profile`` lap time via ``evaluate(points_xy) -> seconds``
         (the GD winner under the surrogate is not always the true winner).
      3. Evolution-strategy polish of the incumbent directly on the exact lap
         time (derivative-free, exploits surrogate/model mismatch).
    The next epoch warm-restarts from the polished incumbent (basin hopping).
    """
    p = params_from_args(args)
    backend = resolve_backend(p.backend)
    epochs = max(int(getattr(args, "ai_epochs", 3)), 1)

    normals, lower, upper, span = _corridor(
        center_xy, d_right, d_left, safety_width, boundary_margin
    )
    if np.all(span <= 1e-3):
        log("[ai] corridor is empty (track too narrow for safety width); keeping centerline")
        return center_xy, {"laps": [], "backend": backend, "epochs": 0}

    def exact(alpha: np.ndarray) -> float:
        return float(evaluate(center_xy + normals * alpha[:, None]))

    # "score" below = exact velocity_profile lap time [s] + steering-limit
    # penalty; once a line is drivable (no kappa excess) the score IS its lap
    # time, but infeasible lines can score in the hundreds — that is intended.
    best_alpha = 0.5 * (lower + upper)
    best_lap = exact(best_alpha)
    log(f"[ai] baseline score (corridor center): {best_lap:.3f}")
    if warm_alpha is not None:
        warm = np.clip(warm_alpha, lower, upper)
        warm_lap = exact(warm)
        log(f"[ai] baseline score (min-curvature warm start): {warm_lap:.3f}")
        if warm_lap < best_lap:
            best_alpha, best_lap = warm, warm_lap

    runner = _run_torch if backend == "torch" else _run_mlx
    rng = np.random.default_rng(p.seed + 1000)
    # ES perturbation wavelength in samples, targeting ~0.6 m regardless of the
    # optimizer_step the centerline was resampled at (shorter-wavelength noise
    # just adds curvature spikes after the final resample).
    mean_seg = float(np.mean(np.linalg.norm(np.roll(center_xy, -1, axis=0) - center_xy, axis=1)))
    es_smooth_px = max(2.0, 0.6 / max(mean_seg, 1e-6))
    history: list[float] = []
    t0 = time.time()
    for epoch in range(epochs):
        log(f"[ai] epoch {epoch + 1}/{epochs} (incumbent {best_lap:.3f}s)")
        seeds = [best_alpha] + ([np.clip(warm_alpha, lower, upper)] if warm_alpha is not None else [])
        theta0 = _ai_inits(lower, span, p.restarts, p.seed + 31 * epoch, seeds)
        smooth_vec = np.array(
            [p.smooth_weight * _SMOOTH_PORTFOLIO[i % len(_SMOOTH_PORTFOLIO)]
             for i in range(theta0.shape[0])],
            dtype=np.float32,
        )
        p_epoch = replace(p, seed=p.seed + 31 * epoch)
        alphas, _diff_laps = runner(
            theta0, center_xy, normals, lower, span, p_epoch, log, smooth_vec=smooth_vec
        )

        # Exact rescoring of every restart with the true reported lap time.
        exact_laps = np.array([exact(a) for a in alphas])
        idx = int(np.argmin(exact_laps))
        log(
            f"[ai]   GD portfolio: best score {exact_laps[idx]:.3f} "
            f"worst {exact_laps.max():.3f} over {len(exact_laps)} restarts"
        )
        if exact_laps[idx] < best_lap:
            best_alpha, best_lap = alphas[idx].astype(np.float64), float(exact_laps[idx])

        best_alpha, best_lap = _es_polish(
            best_alpha, lower, upper, exact, rng, smooth_px=es_smooth_px, log=log
        )
        history.append(best_lap)

    elapsed = time.time() - t0
    log(f"[ai] done in {elapsed:.1f}s: best score {best_lap:.3f} "
        f"({' -> '.join(f'{x:.3f}' for x in history)})")
    raceline = center_xy + normals * best_alpha[:, None]
    info = {
        "backend": backend,
        "epochs": epochs,
        "best_lap": best_lap,
        "history": history,
        "elapsed_s": elapsed,
    }
    return raceline, info
