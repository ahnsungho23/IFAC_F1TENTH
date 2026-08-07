"""Regression tests for persisted GUI optimizer compatibility."""

from pathlib import Path

import pytest

from trajectory_gui import load_gui_params


@pytest.mark.parametrize("legacy_optimizer", ("laptime", "ai"))
def test_load_gui_params_normalizes_removed_optimizer(
    tmp_path: Path, legacy_optimizer: str
) -> None:
    params_path = tmp_path / "gui_params.yaml"
    params_path.write_text(f"optimizer: {legacy_optimizer}\n", encoding="utf-8")

    assert load_gui_params(params_path)["optimizer"] == "mincurv"

    for supported_optimizer in ("centerline", "mincurv"):
        params_path.write_text(
            f"optimizer: {supported_optimizer}\n", encoding="utf-8"
        )
        assert load_gui_params(params_path)["optimizer"] == supported_optimizer
