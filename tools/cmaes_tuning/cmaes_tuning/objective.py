"""Bounded episode costs and safety-dominant candidate fitness."""

from __future__ import annotations

import math
from typing import Iterable


PERFORMANCE_KEYS = (
    "time",
    "progress",
    "obstacle_clearance",
    "wall_clearance",
    "steering_tv_rate",
    "curvature_rate_rms",
)


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def episode_cost(metrics: dict, config: dict) -> dict:
    weights = config["objective"]["weights"]
    evaluation = config["evaluation"]
    completed = bool(metrics["completed"])
    timeout = float(metrics["episode_timeout_s"])
    time_cost = clamp01(float(metrics["completion_time_s"]) / timeout) if completed else 1.0
    progress_cost = 0.0 if completed else clamp01(1.0 - float(metrics["progress_fraction"]))
    obstacle_target = float(evaluation["obstacle_clearance_target_m"])
    wall_target = float(evaluation["wall_clearance_target_m"])
    obstacle_cost = clamp01(
        (obstacle_target - float(metrics["minimum_obstacle_clearance_m"])) / obstacle_target
    )
    wall_cost = clamp01(
        (wall_target - float(metrics["minimum_wall_clearance_m"])) / wall_target
    )
    steering_cost = clamp01(
        float(metrics["steering_total_variation_per_s"])
        / float(evaluation["steering_tv_rate_max_radps"])
    )
    curvature_cost = clamp01(
        float(metrics["planned_curvature_rate_rms_radpm2"])
        / float(evaluation["curvature_rate_rms_max_radpm2"])
    )
    normalized = {
        "time": time_cost,
        "progress": progress_cost,
        "obstacle_clearance": obstacle_cost,
        "wall_clearance": wall_cost,
        "steering_tv_rate": steering_cost,
        "curvature_rate_rms": curvature_cost,
    }
    weighted = {name: float(weights[name]) * normalized[name] for name in PERFORMANCE_KEYS}
    return {
        "normalized_terms": normalized,
        "weighted_terms": weighted,
        "j_performance": sum(weighted.values()),
    }


def upper_tail_cvar(values: Iterable[float], alpha: float) -> float:
    samples = sorted(float(value) for value in values)
    if not samples:
        raise ValueError("CVaR requires at least one sample")
    if not 0.0 <= alpha < 1.0:
        raise ValueError("CVaR alpha must be in [0, 1)")
    count = max(1, int(math.ceil((1.0 - alpha) * len(samples))))
    return sum(samples[-count:]) / count


def candidate_fitness(episodes: list[dict], config: dict) -> dict:
    if not episodes:
        raise ValueError("candidate fitness requires episodes")
    if any(not episode.get("valid", False) for episode in episodes):
        raise ValueError("invalid episodes must never enter CMA fitness")
    costs = [episode_cost(episode["metrics"], config) for episode in episodes]
    values = [cost["j_performance"] for cost in costs]
    failure_count = sum(
        bool(episode["failure"]["collision"])
        or bool(episode["failure"]["off_track"])
        or bool(episode["failure"]["planner_failure"])
        for episode in episodes
    )
    weights = config["objective"]["weights"]
    j_max = sum(float(weights[name]) for name in PERFORMANCE_KEYS)
    beta = float(config["objective"]["beta_cvar"])
    epsilon = float(config["objective"]["epsilon"])
    multiplier = (1.0 + beta) * j_max + epsilon
    mean = sum(values) / len(values)
    cvar = upper_tail_cvar(values, float(config["objective"]["cvar_alpha"]))
    quality = mean + beta * cvar
    fitness = multiplier * failure_count + quality
    return {
        "fitness": fitness,
        "failure_count": failure_count,
        "mean_performance_cost": mean,
        "cvar_performance_cost": cvar,
        "quality_cost": quality,
        "failure_multiplier": multiplier,
        "j_max": j_max,
        "episode_costs": costs,
    }
