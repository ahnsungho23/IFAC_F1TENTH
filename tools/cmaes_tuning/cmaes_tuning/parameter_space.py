"""Normalized CMA variables and safe baseline-YAML patching."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path
import shutil
from typing import Iterable

import yaml


@dataclass(frozen=True)
class ParameterDefinition:
    name: str
    baseline: float
    lower: float
    upper: float
    unit: str
    transform: str
    target_ros_parameter: str
    decode: str | None
    description: str


class ParameterSpace:
    """Single source of truth for z in [0, 1] and planner YAML decoding."""

    _SCALAR_NAMES = {
        "safety_margin_m",
        "obstacle_longitudinal_padding_m",
        "outside_line_transition_scale",
        "minimum_target_offset_m",
        "wall_safety_margin_m",
    }

    def __init__(self, definitions: list[ParameterDefinition]):
        if not definitions:
            raise ValueError("parameter space is empty")
        self.definitions = definitions
        names = [definition.name for definition in definitions]
        if len(set(names)) != len(names):
            raise ValueError("parameter names must be unique")
        for definition in definitions:
            if definition.transform != "linear":
                raise ValueError(f"unsupported transform for {definition.name}")
            if not definition.lower < definition.upper:
                raise ValueError(f"invalid bounds for {definition.name}")
            if not definition.lower <= definition.baseline <= definition.upper:
                raise ValueError(f"baseline outside bounds for {definition.name}")

    @classmethod
    def load(cls, path: str | Path) -> "ParameterSpace":
        with Path(path).open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        if document.get("schema") != "cmaes_parameter_space/1":
            raise ValueError("unsupported parameter-space schema")
        definitions = []
        for item in document.get("parameters", []):
            definitions.append(
                ParameterDefinition(
                    name=str(item["name"]),
                    baseline=float(item["baseline"]),
                    lower=float(item["lower"]),
                    upper=float(item["upper"]),
                    unit=str(item["unit"]),
                    transform=str(item["transform"]),
                    target_ros_parameter=str(item["target_ros_parameter"]),
                    decode=item.get("decode"),
                    description=str(item["description"]),
                )
            )
        return cls(definitions)

    @property
    def names(self) -> list[str]:
        return [definition.name for definition in self.definitions]

    @property
    def dimension(self) -> int:
        return len(self.definitions)

    def decode(self, z: Iterable[float]) -> dict[str, float]:
        values = list(z)
        if len(values) != self.dimension:
            raise ValueError(f"expected {self.dimension} normalized values")
        physical: dict[str, float] = {}
        for definition, normalized in zip(self.definitions, values):
            value = float(normalized)
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{definition.name} normalized value is outside [0, 1]")
            physical[definition.name] = (
                definition.lower + value * (definition.upper - definition.lower)
            )
        self._validate_physical(physical)
        return physical

    def encode(self, physical: dict[str, float]) -> list[float]:
        encoded = []
        for definition in self.definitions:
            value = float(physical[definition.name])
            encoded.append((value - definition.lower) / (definition.upper - definition.lower))
        self.decode(encoded)
        return encoded

    def baseline_physical(self) -> dict[str, float]:
        return {definition.name: definition.baseline for definition in self.definitions}

    def baseline_z(self) -> list[float]:
        return self.encode(self.baseline_physical())

    @staticmethod
    def _validate_physical(physical: dict[str, float]) -> None:
        transitions = [
            physical["transition_short"],
            physical["transition_middle"],
            physical["transition_long"],
        ]
        if not transitions[0] < transitions[1] < transitions[2]:
            raise ValueError("transition_short < transition_middle < transition_long is required")

    def planner_patch(self, physical: dict[str, float]) -> dict[str, object]:
        self._validate_physical(physical)
        pre_far = physical["pre_apex_far_m"]
        post_far = physical["post_apex_far_m"]
        patch: dict[str, object] = {
            name: physical[name] for name in sorted(self._SCALAR_NAMES)
        }
        patch["pre_apex_distances_m"] = [pre_far, 2.0 * pre_far / 3.0, pre_far / 3.0]
        patch["post_apex_distances_m"] = [post_far / 3.0, 2.0 * post_far / 3.0, post_far]
        patch["transition_distance_scales"] = [
            physical["transition_short"],
            physical["transition_middle"],
            physical["transition_long"],
        ]
        return patch

    @staticmethod
    def _ros_parameters(document: dict) -> dict:
        try:
            parameters = document["local_planner_node"]["ros__parameters"]
        except (KeyError, TypeError) as error:
            raise ValueError("baseline YAML lacks local_planner_node.ros__parameters") from error
        if not isinstance(parameters, dict):
            raise ValueError("planner ros__parameters must be a mapping")
        return parameters

    def validate_baseline(self, baseline_yaml: str | Path, tolerance: float = 1.0e-9) -> None:
        with Path(baseline_yaml).open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        parameters = self._ros_parameters(document)
        expected = self.planner_patch(self.baseline_physical())
        for name, expected_value in expected.items():
            actual = parameters[name]
            if isinstance(expected_value, list):
                if len(actual) != len(expected_value) or any(
                    abs(float(a) - float(b)) > tolerance
                    for a, b in zip(actual, expected_value)
                ):
                    raise ValueError(f"parameter-space baseline differs from {name}")
            elif abs(float(actual) - float(expected_value)) > tolerance:
                raise ValueError(f"parameter-space baseline differs from {name}")

    def write_candidate_yaml(
        self,
        baseline_yaml: str | Path,
        output_yaml: str | Path,
        z: Iterable[float],
    ) -> dict[str, object]:
        source = Path(baseline_yaml)
        destination = Path(output_yaml)
        with source.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        parameters = self._ros_parameters(document)
        before = dict(parameters)
        physical = self.decode(z)
        patch = self.planner_patch(physical)
        for name, value in patch.items():
            if name not in parameters:
                raise ValueError(f"baseline YAML lacks whitelisted parameter {name}")
            parameters[name] = value
        changed = {name for name in parameters if parameters[name] != before[name]}
        if not changed.issubset(patch):
            raise AssertionError(f"non-whitelisted parameters changed: {sorted(changed - set(patch))}")
        destination.parent.mkdir(parents=True, exist_ok=True)
        with destination.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(document, stream, sort_keys=False)
        return {
            "normalized": list(z),
            "physical": physical,
            "planner_patch": patch,
            "candidate_sha256": hashlib.sha256(destination.read_bytes()).hexdigest(),
        }

    @staticmethod
    def copy_baseline(baseline_yaml: str | Path, output_yaml: str | Path) -> None:
        destination = Path(output_yaml)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(baseline_yaml, destination)
