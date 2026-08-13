from copy import deepcopy
from types import SimpleNamespace

from cmaes_tuning.deterministic_replay import (
    first_replay_divergence,
    summarize_replays,
    waypoint_geometry_hash,
)


def _waypoint(value: float) -> SimpleNamespace:
    return SimpleNamespace(
        id=0,
        s_m=value,
        d_m=0.2,
        x_m=1.0,
        y_m=2.0,
        d_right=1.0,
        d_left=1.0,
        psi_rad=0.1,
        kappa_radpm=0.2,
        vx_mps=4.0,
        ax_mps2=0.0,
    )


def _path(stamp: int, value: float = 1.0) -> SimpleNamespace:
    return SimpleNamespace(
        ot_side="left",
        ot_line="raceline_local_d_offset_spline",
        wpnts=[_waypoint(value)],
        header=SimpleNamespace(stamp=stamp),
        last_switch_time=stamp,
    )


def _result() -> dict:
    return {
        "unmatched_detector_event_count": 0,
        "confirmation_scan_index": 12,
        "envelope_stability_scan_index": 12,
        "first_static_obs_scan_index": 12,
        "planner_stabilization_start_scan_index": 13,
        "planner_stabilization_ready_scan_index": 28,
        "planner_commitment_scan_index": 28,
        "committed_obstacle_id": 0,
        "first_static_obstacle_ids": [0],
        "go_left": True,
        "initial_target_d": 0.48,
        "entry_transition_scale": 3.5,
        "exit_transition_scale": 0.25,
        "effective_entry_transition_scale": 0.35,
        "effective_exit_transition_scale": 0.025,
        "first_static_geometry_hash": "geometry",
        "raw_detection_sequence_hash": "raw_sequence",
        "static_geometry_sequence_hash": "static_sequence",
        "tracker_sequence_hash": "tracker_sequence",
        "avoidance_path_geometry_hash": "path",
        "_fingerprints": {
            "raw_by_scan": {10: "raw", 11: "raw2"},
            "tracks_by_scan": {10: "track", 11: "track2"},
            "static_by_scan": {10: "empty", 11: "static"},
        },
    }


def test_path_hash_ignores_timestamp_but_detects_geometry_change():
    assert waypoint_geometry_hash(_path(1)) == waypoint_geometry_hash(_path(99))
    assert waypoint_geometry_hash(_path(1)) != waypoint_geometry_hash(_path(1, 1.01))


def test_summary_marks_identical_replays_deterministic():
    result = _result()
    summary = summarize_replays([result, deepcopy(result)])
    assert summary["perception_planner_deterministic"] is True
    assert summary["target_d_statistics"]["std"] == 0.0
    assert summary["first_divergence"] is None


def test_first_divergence_reports_tracker_before_planner():
    reference = _result()
    other = deepcopy(reference)
    other["_fingerprints"]["tracks_by_scan"][11] = "different"
    other["initial_target_d"] = 0.52
    divergence = first_replay_divergence(reference, other)
    assert divergence == {
        "backend_scan_index": 11,
        "subsystem": "obstacle_tracker",
        "cause": "different_output_for_same_recorded_input",
    }


def test_first_divergence_reports_timer_milestone_when_detector_matches():
    reference = _result()
    other = deepcopy(reference)
    other["planner_stabilization_ready_scan_index"] = 29
    divergence = first_replay_divergence(reference, other)
    assert divergence["subsystem"] == "planner_timer_readiness"
