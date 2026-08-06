#!/usr/bin/env python3
"""Regression tests for adaptive overlay orchestration helpers."""

from pathlib import Path

import numpy as np

from generate_adaptive_overlays import evaluation_item, offset_label, output_name
from generate_global_trajectory import MapInfo, count_off_map_waypoints


def test_output_name_uses_requested_index_and_signed_tenths() -> None:
    item = {"index": 0, "d": 0.2}
    assert offset_label(-0.5) == "m05"
    assert offset_label(0.0) == "p00"
    assert output_name(item, "outline") == "idx_000_d_p02_outline.png"


def test_default_reference_has_exactly_1562_scenarios() -> None:
    reference = (
        Path(__file__).resolve().parent.parent
        / "ruleset_adaptive_globalpath/map/global_waypoints.csv"
    )
    waypoint_count = sum(1 for _ in reference.open(encoding="utf-8")) - 1
    lateral_count = len(np.arange(-0.5, 0.5 + 0.05, 0.1))
    assert waypoint_count == 142
    assert waypoint_count * lateral_count == 1562


def test_evaluation_item_converts_numeric_csv_fields() -> None:
    converted = evaluation_item(
        {"index": "2", "s": "1.25", "d": "-0.1", "decision": "right"}
    )
    assert converted["index"] == 2
    assert converted["s"] == 1.25
    assert converted["d"] == -0.1
    assert converted["decision"] == "right"


def test_off_map_counter_handles_dense_closing_endpoint() -> None:
    info = MapInfo(
        yaml_path=Path("map.yaml"),
        image_path=Path("map.png"),
        resolution=0.1,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        negate=0,
        occupied_thresh=0.65,
        free_thresh=0.196,
        height=20,
        width=20,
    )
    points = np.asarray([[0.2, 0.2], [1.5, 0.2], [1.5, 1.5], [0.2, 1.5]])
    free = np.ones((20, 20), dtype=np.uint8)
    free[2, 8:11] = 0
    assert count_off_map_waypoints(points, free, info, flip_y=False) >= 1
