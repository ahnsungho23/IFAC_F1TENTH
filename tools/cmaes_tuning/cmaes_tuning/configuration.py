"""Configuration loading with workspace-relative path resolution."""

from __future__ import annotations

from pathlib import Path
import re
from typing import Any

import yaml


def workspace_root_from_tool() -> Path:
    return Path(__file__).resolve().parents[3]


def load_config(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if config.get("schema") != "cmaes_tuning_config/1":
        raise ValueError("unsupported tuning-config schema")
    return config


def resolve_workspace_path(value: str | Path, workspace_root: str | Path) -> Path:
    path = Path(value)
    return path.resolve() if path.is_absolute() else (Path(workspace_root) / path).resolve()


def safe_identifier(value: str) -> str:
    result = re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("._")
    if not result:
        raise ValueError("identifier has no safe characters")
    return result
