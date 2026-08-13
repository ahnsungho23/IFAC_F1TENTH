from __future__ import annotations

from pathlib import Path
import sys


TOOL_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_ROOT))

from cmaes_tuning.configuration import load_config
from p3_hybrid_cma import hybrid_fitness, preflight


def episode() -> dict:
    return {
        "valid": True,
        "metrics": {
            "completed": True,
            "episode_timeout_s": 65.0,
            "completion_time_s": 12.0,
            "progress_fraction": 1.0,
            "minimum_obstacle_clearance_m": 0.12,
            "minimum_wall_clearance_m": 0.05,
            "steering_total_variation_per_s": 1.0,
            "planned_curvature_rate_rms_radpm2": 1.0,
            "speed_loss_during_avoidance_fraction": 0.10,
        },
    }


def lockstep(obstacle_count: int) -> dict:
    return {
        "step_count": 1200,
        "obstacle_count": obstacle_count,
        "collision": False,
        "off_track": False,
        "nonfinite": False,
        "unsafe_path_published": False,
        "planner_failure": False,
        "completed": True,
        "invalid_suffix": True,
        "p0_fallback_interval_count": 1,
        "p0_fallback_duration_s": 0.5,
        "safe_stop_interval_count": 0,
        "safe_stop_duration_s": 0.0,
        "p3_ownership_fraction": 0.7,
        "p3_completion_count": obstacle_count,
    }


def test_competition_q2_and_finals_preflight() -> None:
    result = preflight()
    assert result["selected"]["Q2_COMPETITION_STYLE"]["configuration"][
        "actual_obstacle_count"] == 2
    assert result["selected"]["FINALS_COMPETITION_STYLE"]["configuration"][
        "actual_obstacle_count"] == 3
    assert result["space"].dimension == 6


def test_safely_rejected_suffix_and_p0_fallback_are_not_hard_failures() -> None:
    config = load_config(TOOL_ROOT / "config/tuning_config.yaml")
    result = hybrid_fitness(
        [(episode(), lockstep(2)), (episode(), lockstep(3))], config)
    assert result["feasible"] is True
    assert result["hard_failure_count"] == 0
    assert result["mean_p0_fallback_fraction"] > 0.0


def test_each_actual_safety_failure_is_hard() -> None:
    config = load_config(TOOL_ROOT / "config/tuning_config.yaml")
    for key in (
        "collision", "off_track", "nonfinite", "unsafe_path_published",
        "planner_failure",
    ):
        result_row = lockstep(2)
        result_row[key] = True
        result = hybrid_fitness([(episode(), result_row)], config)
        assert result["feasible"] is False
        assert result["hard_failure_rows"][0][key] is True


def test_incomplete_scenario_is_not_feasible() -> None:
    config = load_config(TOOL_ROOT / "config/tuning_config.yaml")
    result_row = lockstep(2)
    result_row["completed"] = False
    result = hybrid_fitness([(episode(), result_row)], config)
    assert result["feasible"] is False
    assert result["hard_failure_rows"][0]["incomplete_scenario"] is True


def test_safe_stop_penalty_exceeds_p0_fallback_penalty() -> None:
    config = load_config(TOOL_ROOT / "config/tuning_config.yaml")
    p0_row = lockstep(2)
    safe_row = lockstep(2)
    p0_row["p0_fallback_duration_s"] = 2.0
    safe_row["p0_fallback_duration_s"] = 0.0
    safe_row["p0_fallback_interval_count"] = 0
    safe_row["safe_stop_duration_s"] = 2.0
    safe_row["safe_stop_interval_count"] = 1
    p0 = hybrid_fitness([(episode(), p0_row)], config)
    safe = hybrid_fitness([(episode(), safe_row)], config)
    assert safe["fitness"] > p0["fitness"]
