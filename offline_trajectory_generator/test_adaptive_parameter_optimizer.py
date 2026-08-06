#!/usr/bin/env python3
"""Unit tests for feasibility-first adaptive CMA-ES helpers."""

from pathlib import Path

import numpy as np
import yaml

from generate_adaptive_overlays import planner_values
from optimize_adaptive_parameters import (
    CandidateResult,
    SearchSpace,
    rank_population,
    summarize_evaluations,
)


ROOT = Path(__file__).resolve().parent.parent


def search_space() -> SearchSpace:
    config = yaml.safe_load(
        (ROOT / "offline_trajectory_generator/config/adaptive_cmaes.yaml").read_text(
            encoding="utf-8"
        )
    )
    base = planner_values(ROOT / "src/local_planning/config/local_planning.yaml", [])
    return SearchSpace(config, base)


def candidate(safe_stop: int, clearance: float) -> CandidateResult:
    space = search_space()
    parameters = space.decode(space.initial_vector)
    parameters["minimum_avoidance_clearance_m"] = clearance
    return CandidateResult(
        generation=0,
        candidate=0,
        parameters=parameters,
        decision_counts={"left": 1562 - safe_stop, "safe_stop": safe_stop},
        selected_reduced_clearance=0,
        min_selected_headroom=0.1,
        row_count=1562,
    )


def test_search_space_round_trip_preserves_initial_parameters() -> None:
    space = search_space()
    decoded = space.decode(space.initial_vector)
    assert np.allclose(space.encode(decoded), space.initial_vector)
    assert np.allclose(decoded["transition_distance_scales"], [0.1, 0.6, 2.0])
    assert decoded["minimum_avoidance_clearance_m"] == 0.151


def test_decoded_transition_scales_are_strictly_ordered() -> None:
    space = search_space()
    for vector in (np.zeros(space.dimension), np.ones(space.dimension)):
        first, second, third = space.decode(vector)["transition_distance_scales"]
        assert 0.0 < first < second < third


def test_clearance_floor_uses_vehicle_width_plus_hard_margin() -> None:
    space = search_space()
    assert space.clearance_range.minimum == 0.121 + 0.03
    assert space.clearance_range.maximum == 0.25


def test_safe_candidate_outranks_higher_clearance_safe_stop() -> None:
    safe = candidate(0, 0.151)
    unsafe = candidate(1, 0.25)
    ranks = rank_population([unsafe, safe])
    assert ranks == [1.0, 0.0]


def test_higher_clearance_wins_between_safe_candidates() -> None:
    low = candidate(0, 0.151)
    high = candidate(0, 0.20)
    ranks = rank_population([low, high])
    assert ranks == [1.0, 0.0]


def test_evaluation_summary_uses_selected_side_headroom() -> None:
    parameters = search_space().decode(search_space().initial_vector)
    result = summarize_evaluations(
        [
            {
                "decision": "left",
                "left_headroom": "0.12",
                "right_headroom": "0.01",
                "selected_reduced_clearance": "1",
            },
            {
                "decision": "right",
                "left_headroom": "0.02",
                "right_headroom": "0.08",
                "selected_reduced_clearance": "0",
            },
        ],
        generation=0,
        candidate=0,
        parameters=parameters,
    )
    assert result.safe_stop_count == 0
    assert result.selected_reduced_clearance == 1
    assert result.min_selected_headroom == 0.08
