#!/usr/bin/env python3
"""Run a compiled f110_gym backend for the uniform-contract prototype audit.

The caller selects the simulator source tree with ``PYTHONPATH``.  Outputs are
temporary NPZ/JSON data consumed by the offline audit; this is not production
runtime code.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import yaml

from f110_gym.envs.cpp_simulator import CppSimulator


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("request", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    request = json.loads(args.request.read_text(encoding="utf-8"))
    simulator_yaml = yaml.safe_load(
        Path(request["simulator_yaml"]).read_text(encoding="utf-8")
    )["bridge"]["ros__parameters"]
    beam_count = int(request.get("beam_count", simulator_yaml["scan_beams"]))
    field_of_view = float(request.get("field_of_view_rad", simulator_yaml["scan_fov"]))
    simulator = CppSimulator(
        simulator_yaml["vehicle"],
        1,
        int(request.get("seed", simulator_yaml["simulator_seed"])),
        time_step=float(simulator_yaml["timestep"]),
        lidar_dist=float(simulator_yaml["scan_distance_to_base_link"]),
        num_beams=beam_count,
        fov=field_of_view,
        scan_noise_std=float(request.get("scan_noise_std_m", 0.0)),
    )
    arrays: dict[str, np.ndarray] = {}
    metadata: dict[str, object] = {
        "beam_count": beam_count,
        "field_of_view_rad": field_of_view,
        "labels": [],
    }
    if hasattr(simulator, "physical_scan_angles"):
        arrays["physical_relative_angles_rad"] = np.asarray(
            simulator.physical_scan_angles, dtype=np.float64
        )
        contract = simulator.scan_contract
        metadata["contract"] = {
            "angle_min_rad": float(contract.angle_min),
            "angle_max_rad": float(contract.angle_max),
            "angle_increment_rad": float(contract.angle_increment),
        }
    map_yaml = request.get("map_yaml")
    if map_yaml:
        map_yaml = Path(map_yaml)
        map_metadata = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
        map_extension = Path(map_metadata["image"]).suffix
        simulator.set_map(str(map_yaml), map_extension)
        for index, pose in enumerate(request.get("poses", [])):
            simulator.reset(
                np.asarray([[pose["x_m"], pose["y_m"], pose["yaw_rad"]]], dtype=np.float64)
            )
            observation = simulator.step(np.zeros((1, 2), dtype=np.float64))
            arrays[f"scan_{index:04d}"] = np.asarray(
                observation["scans"][0], dtype=np.float64
            )
            metadata["labels"].append(pose["label"])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(args.output, **arrays)
    args.output.with_suffix(".json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()
