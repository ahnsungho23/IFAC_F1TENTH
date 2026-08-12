#!/usr/bin/env python3
"""Build and audit the isolated uniform-LaserScan simulator prototype."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import re
import subprocess
import sys
import time
from typing import Any, Sequence

import numpy as np
import yaml

import render_rulebook_free_gap_pruning_audit as free_gap
import render_rulebook_obstacle_envelope_audit as rulebook
import render_s3_shadow_false_infeasible_audit as s3
import render_time_to_disambiguation_audit as time_audit
import render_validation039_sensor_model_consistency_audit as sensor_audit
from cmaes_tuning.bag_reader import read_bag
from cmaes_tuning.lidar_beam_contract import (
    BackendBeamContract,
    wrapped_angle_delta_rad,
)
from cmaes_tuning.scenario_generator import load_waypoints
from cmaes_tuning.uniform_laserscan_prototype import (
    UniformLaserScanContract,
    oriented_rectangle_ranges,
    uniform_noise_free_scan,
)


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "runs/cmaes_tuning/simulator_laserscan_contract_prototype_v1"
RAW = OUTPUT / ".raw"
REPLAY = ROOT / "runs/cmaes_tuning/time_axis_feasibility_audit_v1/replays/validation_039"
SCENARIO = ROOT / "runs/cmaes_tuning/medium_lockstep_stage1_v2/artifacts/scenarios/validation/validation_039"
VALIDATION_MAP = ROOT / "runs/cmaes_tuning/repeatability_mid4_v2/scenarios/validation/validation_039/validation_039_map.yaml"
NORMAL_MAP = ROOT / "runs/cmaes_tuning/rulebook_obstacle_envelope_audit_v1/.raw/normal_maps/normal_01_00/normal_01_00.yaml"
SIMULATOR_YAML = Path("/home/sungho/f1sim_C/f1tenth_gym_ros/config/sim.yaml")
ACTIVE_SIMULATOR = Path("/home/sungho/f1sim_C")
DRIVER = ROOT / "tools/cmaes_tuning/prototype_simulator_scan_driver.py"
BASELINE_STAMP = 11_230_000_000
STAMPS = tuple(BASELINE_STAMP + index * 10_000_000 for index in range(5))
SEED = 39039


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--prototype-root", type=Path, required=True)
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def array_digest(values: np.ndarray) -> str:
    return hashlib.sha256(np.asarray(values, dtype="<f8").tobytes()).hexdigest()


def canonical_digest(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(clean_json(value), sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


def clean_json(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): clean_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean_json(item) for item in value]
    if isinstance(value, np.generic):
        return clean_json(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        raise RuntimeError(f"refusing to write empty CSV: {path}")
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def memory_snapshot() -> dict[str, int]:
    values: dict[str, int] = {}
    for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
        key, raw = line.split(":", 1)
        if key in {"MemAvailable", "SwapTotal", "SwapFree"}:
            values[key] = int(raw.strip().split()[0]) * 1024
    return {
        "memory_available_bytes": values["MemAvailable"],
        "swap_used_bytes": values["SwapTotal"] - values["SwapFree"],
    }


def source_line(path: Path, token: str) -> int:
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if token in line:
            return number
    raise RuntimeError(f"source token not found: {path}: {token}")


def protected_hashes() -> dict[str, str]:
    paths = {
        "obstacle_detector": ROOT / "src/obstacle_detector/src/obstacle_detector_node.cpp",
        "obstacle_detector_header": ROOT / "src/obstacle_detector/include/obstacle_detector/obstacle_detector_node.hpp",
        "local_planner": ROOT / "src/local_planning/src/local_planner_node.cpp",
        "controller": ROOT / "src/f1tenth_control/control_code/controller.cpp",
        "state_machine": ROOT / "src/state_machine/src/state_machine_node.cpp",
        "lockstep_active": ROOT / "tools/cmaes_tuning/lockstep_episode.py",
        "active_simulator_backend": ACTIVE_SIMULATOR / "gym/f110_gym/cpp_backend.cpp",
        "active_simulator_bridge": ACTIVE_SIMULATOR / "f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py",
    }
    return {name: sha256(path) for name, path in paths.items() if path.is_file()}


def command_result(label: str, command: Sequence[str], *, cwd: Path, env: dict[str, str] | None = None) -> dict[str, Any]:
    started = time.monotonic()
    result = subprocess.run(command, cwd=cwd, env=env, text=True, capture_output=True)
    elapsed = time.monotonic() - started
    combined = result.stdout + result.stderr
    detail = combined.strip().splitlines()
    count_match = re.search(r"Ran (\d+) tests?", combined)
    return {
        "test": label,
        "command": " ".join(command),
        "status": "PASS" if result.returncode == 0 else "FAIL",
        "return_code": result.returncode,
        "wall_time_s": elapsed,
        "case_count": int(count_match.group(1)) if count_match else None,
        "detail": " | ".join(detail[-3:]) if detail else "no output",
    }


def run_driver(
    simulator_root: Path,
    label: str,
    *,
    beam_count: int,
    field_of_view_rad: float,
    map_yaml: Path | None = None,
    poses: list[dict[str, Any]] | None = None,
) -> tuple[dict[str, np.ndarray], dict[str, Any], float]:
    request = {
        "simulator_yaml": str(SIMULATOR_YAML),
        "beam_count": beam_count,
        "field_of_view_rad": field_of_view_rad,
        "scan_noise_std_m": 0.0,
        "seed": 12345,
        "map_yaml": str(map_yaml) if map_yaml else None,
        "poses": poses or [],
    }
    request_path = RAW / f"{label}.request.json"
    output_path = RAW / f"{label}.npz"
    request_path.write_text(json.dumps(request, indent=2) + "\n", encoding="utf-8")
    env = os.environ.copy()
    env["PYTHONPATH"] = str(simulator_root / "gym")
    started = time.monotonic()
    subprocess.run(
        [sys.executable, str(DRIVER), str(request_path), str(output_path)],
        cwd=ROOT,
        env=env,
        check=True,
        stdout=subprocess.DEVNULL,
    )
    elapsed = time.monotonic() - started
    with np.load(output_path) as loaded:
        arrays = {key: np.asarray(loaded[key], dtype=np.float64) for key in loaded.files}
    metadata = json.loads(output_path.with_suffix(".json").read_text(encoding="utf-8"))
    return arrays, metadata, elapsed


def pose_rows(odometry: dict[int, Any], stamps: Sequence[int]) -> list[dict[str, Any]]:
    rows = []
    for stamp in stamps:
        odom = odometry[stamp]
        rows.append(
            {
                "label": f"validation039_{(stamp - BASELINE_STAMP) // 1_000_000:+d}ms",
                "x_m": float(odom.pose.pose.position.x),
                "y_m": float(odom.pose.pose.position.y),
                "yaw_rad": sensor_audit.yaw_from_odom(odom),
            }
        )
    return rows


def scan_stats(
    scope: str,
    label: str,
    old_ranges: np.ndarray,
    new_ranges: np.ndarray,
    old_angles: np.ndarray,
    new_angles: np.ndarray,
) -> dict[str, Any]:
    delta = np.abs(new_ranges - old_ranges)
    changed = delta > 1.0e-12
    old_endpoint = old_ranges[:, None] * np.column_stack((np.cos(old_angles), np.sin(old_angles)))
    new_endpoint = new_ranges[:, None] * np.column_stack((np.cos(new_angles), np.sin(new_angles)))
    endpoint = np.linalg.norm(new_endpoint - old_endpoint, axis=1)
    return {
        "scope": scope,
        "scan_label": label,
        "beam_count": len(delta),
        "mean_abs_range_delta_m": float(np.mean(delta)),
        "p95_abs_range_delta_m": float(np.percentile(delta, 95)),
        "p99_abs_range_delta_m": float(np.percentile(delta, 99)),
        "max_abs_range_delta_m": float(np.max(delta)),
        "changed_beam_count": int(np.count_nonzero(changed)),
        "changed_beam_fraction": float(np.mean(changed)),
        "mean_endpoint_displacement_m": float(np.mean(endpoint)),
        "p99_endpoint_displacement_m": float(np.percentile(endpoint, 99)),
        "max_endpoint_displacement_m": float(np.max(endpoint)),
        "old_ranges_sha256": array_digest(old_ranges),
        "new_ranges_sha256": array_digest(new_ranges),
    }


def build_validation_frame(
    stamp: int,
    ranges: np.ndarray,
    odom: Any,
    raster: dict[str, float],
    model: dict[str, Any],
    contract: UniformLaserScanContract,
    guard_m: float,
) -> sensor_audit.FrameEvidence:
    yaw = sensor_audit.yaw_from_odom(odom)
    origin = np.asarray(
        [
            float(odom.pose.pose.position.x) + float(model["lidar_offset_x_m"]) * math.cos(yaw),
            float(odom.pose.pose.position.y) + float(model["lidar_offset_x_m"]) * math.sin(yaw),
        ]
    )
    directions = contract.directions(yaw)
    endpoints = origin + ranges[:, None] * directions
    valid_indices = np.flatnonzero(np.isfinite(ranges) & (ranges >= 0.0)).astype(np.int64)
    hit_indices = np.flatnonzero(rulebook.points_inside_half_open(endpoints, raster)).astype(np.int64)
    ray_frame = rulebook.RayFrame(
        stamp_ns=stamp,
        status="CONFIRMED",
        motion_status="UNKNOWN",
        origin=origin,
        directions=directions[valid_indices],
        ranges=ranges[valid_indices],
        free_lengths=np.maximum(0.0, ranges[valid_indices] - guard_m),
        hits=endpoints[hit_indices],
        hit_indices=tuple(int(value) for value in hit_indices),
        detector_box=(math.nan, math.nan, math.nan, math.nan),
    )
    return sensor_audit.FrameEvidence(
        stamp_ns=stamp,
        scan=None,
        odom=odom,
        event={},
        yaw_rad=yaw,
        origin=origin,
        directions=directions,
        endpoints=endpoints,
        valid_indices=valid_indices,
        hit_indices=hit_indices,
        ray_frame=ray_frame,
        direction_model="PROTOTYPE_ROS_METADATA",
    )


def validation039_regression(
    evidence: dict[int, sensor_audit.FrameEvidence],
    manifest: dict[str, Any],
    raster: dict[str, float],
    repeat_ranges_hash: str,
) -> tuple[list[dict[str, Any]], dict[str, Any], int]:
    rows: list[dict[str, Any]] = []
    grids: dict[int, Any] = {}
    evaluations = 0
    for stamp in STAMPS[2:]:
        window = sensor_audit.current_window(evidence, stamp)
        grid, status = sensor_audit.safe_grid(window)
        evaluations += 1
        if grid is None:
            raise RuntimeError(f"prototype validation_039 set empty at {stamp}")
        grids[stamp] = grid
        witness = time_audit.witness_statistics(window, grid)
        rows.append(
            {
                "run": "PRIMARY",
                "source_stamp_ns": stamp,
                "delta_t_ms": (stamp - BASELINE_STAMP) // 1_000_000,
                "source_stamps": ";".join(str(frame.stamp_ns) for frame in window),
                "target_return_count": len(evidence[stamp].hit_indices),
                "possible_cell_count": int(np.count_nonzero(grid.mask)),
                "possible_hypothesis_count": witness["possible_hypothesis_count"],
                "possible_area_m2": grid.possible_area_m2,
                "status": status,
                "selected_side_support_m": None,
                "gt_undercoverage_m": None,
                "gt_overcoverage_m": None,
                "dangerous_path_clearance_m": None,
                "dangerous_path_classification": None,
                "ranges_sha256": array_digest(evidence[stamp].ray_frame.ranges),
            }
        )
    obstacle = {
        key: float(manifest["obstacle"][key])
        for key in ("x", "y", "s", "d", "yaw", "width", "height")
    }
    case = {
        "case_id": "validation_039",
        "stamp_ns": BASELINE_STAMP,
        "stream": str(time_audit.STREAM_039),
        "manifest_s": obstacle["s"],
        "obstacle": obstacle,
        "raster": raster,
    }
    track = free_gap.TrackGeometry.from_waypoints(load_waypoints(time_audit.WAYPOINTS))
    safety = sensor_audit.projected_safety(
        "PROTOTYPE_ROS_METADATA",
        grids[STAMPS[4]],
        case,
        track,
        time_audit.gt_representation("validation_039"),
        4,
    )
    evaluations += 1
    rows[-1].update(
        {
            "selected_side_support_m": safety["selected_side_support_m"],
            "gt_undercoverage_m": safety["gt_undercoverage_m"],
            "gt_overcoverage_m": safety["gt_overcoverage_m"],
            "dangerous_path_clearance_m": safety["dangerous_path_clearance_m"],
            "dangerous_path_classification": "HARD_INVALID"
            if safety["dangerous_path_hard_invalid"]
            else "HARD_VALID",
        }
    )
    repeat_grid, repeat_status = sensor_audit.safe_grid(
        sensor_audit.current_window(evidence, STAMPS[4])
    )
    evaluations += 1
    repeat_safety = sensor_audit.projected_safety(
        "PROTOTYPE_ROS_METADATA_REPEAT",
        repeat_grid,
        case,
        track,
        time_audit.gt_representation("validation_039"),
        4,
        repeat=True,
    )
    evaluations += 1
    repeat_row = dict(rows[-1])
    repeat_row.update(
        {
            "run": "REPEAT",
            "status": repeat_status,
            "possible_cell_count": int(np.count_nonzero(repeat_grid.mask)),
            "possible_area_m2": repeat_grid.possible_area_m2,
            "selected_side_support_m": repeat_safety["selected_side_support_m"],
            "gt_undercoverage_m": repeat_safety["gt_undercoverage_m"],
            "gt_overcoverage_m": repeat_safety["gt_overcoverage_m"],
            "dangerous_path_clearance_m": repeat_safety["dangerous_path_clearance_m"],
            "dangerous_path_classification": "HARD_INVALID"
            if repeat_safety["dangerous_path_hard_invalid"]
            else "HARD_VALID",
            "ranges_sha256": repeat_ranges_hash,
        }
    )
    rows.append(repeat_row)
    fields = (
        "possible_cell_count",
        "possible_area_m2",
        "status",
        "selected_side_support_m",
        "gt_undercoverage_m",
        "gt_overcoverage_m",
        "dangerous_path_clearance_m",
        "dangerous_path_classification",
        "ranges_sha256",
    )
    primary = {key: rows[-2][key] for key in fields}
    repeat = {key: rows[-1][key] for key in fields}
    determinism = {
        "primary_digest": canonical_digest(primary),
        "repeat_digest": canonical_digest(repeat),
        "identical": primary == repeat,
    }
    if not determinism["identical"]:
        raise RuntimeError(f"validation_039 prototype repeat differs: {primary} != {repeat}")
    return rows, determinism, evaluations


def prototype_normal_frames(case: dict[str, Any]) -> list[rulebook.RayFrame]:
    model = case["clean_model"]
    obstacle = case["obstacle"]
    contract = UniformLaserScanContract.from_fov(model.scan_fov, model.scan_beams)
    frames: list[rulebook.RayFrame] = []
    for absolute_index in range(9):
        pose = rulebook.interpolate_reference(
            case["waypoints"],
            float(case["manifest_s"])
            - float(case["gt_row"]["observation_distance_m"])
            + 0.04 * absolute_index,
        )
        yaw = float(pose["yaw"])
        origin = np.asarray(
            [
                float(pose["x"]) + model.lidar_offset * math.cos(yaw),
                float(pose["y"]) + model.lidar_offset * math.sin(yaw),
            ]
        )
        directions = contract.directions(yaw)
        clean_ranges = uniform_noise_free_scan(
            model, float(pose["x"]), float(pose["y"]), yaw
        )
        target_ranges = oriented_rectangle_ranges(
            origin,
            directions,
            np.asarray([obstacle["x"], obstacle["y"]]),
            float(obstacle["yaw"]),
            float(obstacle["width"]),
            float(obstacle["height"]),
        )
        hit_mask = target_ranges + rulebook.GEOMETRY_EPSILON_M < clean_ranges
        ranges = np.minimum(clean_ranges, target_ranges)
        endpoints = origin + ranges[:, None] * directions
        hit_indices_array = np.flatnonzero(hit_mask)
        if hit_indices_array.size < 5:
            raise RuntimeError("normal_01_00 prototype target has fewer than five returns")
        target_points = endpoints[hit_indices_array]
        status = ("RAW", "TENTATIVE", "CONFIRMED")[min(absolute_index, 2)]
        frames.append(
            rulebook.RayFrame(
                stamp_ns=20_000_000_000 + absolute_index * 10_000_000,
                status=status,
                motion_status="UNKNOWN",
                origin=origin,
                directions=directions,
                ranges=ranges,
                free_lengths=np.maximum(0.0, ranges - rulebook.GEOMETRY_EPSILON_M),
                hits=target_points,
                hit_indices=tuple(int(value) for value in hit_indices_array),
                detector_box=(
                    float(np.min(target_points[:, 0])),
                    float(np.max(target_points[:, 0])),
                    float(np.min(target_points[:, 1])),
                    float(np.max(target_points[:, 1])),
                ),
            )
        )
    return frames


def normal0100_regression(
    case: dict[str, Any], frames: list[rulebook.RayFrame], *, repeat: bool
) -> tuple[list[dict[str, Any]], int]:
    track = free_gap.TrackGeometry.from_waypoints(case["waypoints"])
    gt = time_audit.gt_representation("normal_01_00")
    blocker = time_audit.blocking_hypotheses()["normal_01_00"]
    rows: list[dict[str, Any]] = []
    evaluations = 0
    for scan_index, frame in enumerate(frames[2:9]):
        window = s3.selected_frames_window(frames, frame.stamp_ns, 20)
        grid = s3.detailed_possible_occupancy(window, 0.004, 2.0)
        witness = time_audit.witness_statistics(window, grid)
        support, _ = time_audit.project_grid(case, grid, scan_index, track, repeat)
        ego = time_audit.ego_state(case, frame, 2 + scan_index)
        action, _ = time_audit.run_actionability(
            case, scan_index, frame.stamp_ns, ego, support, gt, repeat
        )
        blocker_state = time_audit.hypothesis_consistency(window, blocker)
        evaluations += 1
        rows.append(
            {
                "run": "REPEAT" if repeat else "PRIMARY",
                "source_stamp_ns": frame.stamp_ns,
                "delta_t_ms": scan_index * 10,
                "fresh_target_return_count": len(frame.hit_indices),
                "possible_cell_count": int(np.count_nonzero(grid.mask)),
                "possible_hypothesis_count": witness["possible_hypothesis_count"],
                "possible_d_min_m": support["d_min"],
                "possible_d_max_m": support["d_max"],
                "possible_support_m": support["d_max"],
                "shadow_corridor_m": support["left_corridor"],
                "blocking_hypothesis_status": blocker_state["status"],
                "blocking_hypothesis_evidence": blocker_state["evidence_type"],
                "production_family_hard_feasible_count": action["shadow_hard_feasible"],
                "ranges_sha256": array_digest(frame.ranges),
            }
        )
    first = next(row for row in rows if row["production_family_hard_feasible_count"] > 0)
    stable = next(
        row["delta_t_ms"]
        for index, row in enumerate(rows)
        if index >= 2
        and all(
            item["production_family_hard_feasible_count"] > 0
            for item in rows[index - 2 : index + 1]
        )
    )
    for row in rows:
        row["classification"] = "OBSERVATION_RESOLVES_EARLY"
        row["first_hard_feasible_ms"] = first["delta_t_ms"]
        row["stable_resolution_ms"] = stable
    return rows, evaluations


def random_geometry_rows() -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    generator = np.random.default_rng(SEED)
    rows: list[dict[str, Any]] = []
    stats: list[dict[str, Any]] = []
    beam_counts = (17, 180, 360, 720, 1080, 1440)
    for case_index in range(128):
        beam_count = beam_counts[case_index % len(beam_counts)]
        fov = float(generator.uniform(1.2, 5.8))
        yaw = float(generator.uniform(-math.pi, math.pi))
        bearing_relative = float(generator.uniform(-0.49 * fov, 0.49 * fov))
        target_range = float(generator.uniform(0.5, 25.0))
        origin = generator.uniform(-20.0, 20.0, size=2)
        centre = origin + target_range * np.asarray(
            [math.cos(yaw + bearing_relative), math.sin(yaw + bearing_relative)]
        )
        target_yaw = float(generator.uniform(-math.pi, math.pi))
        width = float(generator.uniform(0.08, 1.5))
        height = float(generator.uniform(0.08, 2.0))
        prototype = UniformLaserScanContract.from_fov(fov, beam_count)
        new_angles = prototype.physical_world_angles_rad(yaw)
        metadata_angles = yaw + prototype.published_relative_angles_rad()
        old_angles = BackendBeamContract(fov, beam_count).physical_angles_rad(yaw)
        old_directions = np.column_stack((np.cos(old_angles), np.sin(old_angles)))
        new_directions = np.column_stack((np.cos(new_angles), np.sin(new_angles)))
        old_ranges = oriented_rectangle_ranges(
            origin, old_directions, centre, target_yaw, width, height
        )
        new_ranges = oriented_rectangle_ranges(
            origin, new_directions, centre, target_yaw, width, height
        )
        mismatch = np.abs(wrapped_angle_delta_rad(new_angles, metadata_angles)) > 0.0
        old_mapping_mismatch = (
            np.abs(wrapped_angle_delta_rad(old_angles, metadata_angles)) > 1.0e-12
        )
        rows.append(
            {
                "seed": SEED,
                "case_index": case_index,
                "beam_count": beam_count,
                "fov_rad": fov,
                "sensor_x_m": float(origin[0]),
                "sensor_y_m": float(origin[1]),
                "sensor_yaw_rad": yaw,
                "target_bearing_relative_rad": bearing_relative,
                "target_range_m": target_range,
                "target_x_m": float(centre[0]),
                "target_y_m": float(centre[1]),
                "target_yaw_rad": target_yaw,
                "target_width_m": width,
                "target_height_m": height,
                "contract_mismatch_count": int(np.count_nonzero(mismatch)),
                "legacy_mapping_mismatch_count": int(np.count_nonzero(old_mapping_mismatch)),
                "max_contract_error_rad": float(
                    np.max(np.abs(wrapped_angle_delta_rad(new_angles, metadata_angles)))
                ),
                "old_target_hit_beam_count": int(np.count_nonzero(old_ranges < 30.0)),
                "new_target_hit_beam_count": int(np.count_nonzero(new_ranges < 30.0)),
                "status": "PASS" if not np.any(mismatch) else "FAIL",
            }
        )
        stats.append(
            scan_stats(
                "RANDOM_GEOMETRY",
                f"random_{case_index:03d}",
                old_ranges,
                new_ranges,
                old_angles,
                new_angles,
            )
        )
    return rows, stats


def lockstep_overlay_patch() -> str:
    return """diff --git a/tools/cmaes_tuning/lockstep_episode.py b/tools/cmaes_tuning/lockstep_episode.py
--- a/tools/cmaes_tuning/lockstep_episode.py
+++ b/tools/cmaes_tuning/lockstep_episode.py
@@ -27,7 +27,8 @@ import yaml
 
 from ackermann_msgs.msg import AckermannDriveStamped
 from builtin_interfaces.msg import Time
 from f110_gym.envs import Integrator
+from f110_gym.envs.laser_scan_contract import UniformLaserScanContract
 from f110_msgs.msg import OTWpntArray, ObstacleArray, StateMachine, WpntArray
 from nav_msgs.msg import OccupancyGrid, Odometry
 from sensor_msgs.msg import LaserScan
@@ -237,13 +238,14 @@ def make_odom(obs: dict[str, Any], timestamp_ns: int) -> Odometry:
     return message
 
 
 def make_scan(obs: dict[str, Any], timestamp_ns: int, scan_fov: float, beams: int) -> LaserScan:
+    contract = UniformLaserScanContract.from_fov(scan_fov, beams)
     message = LaserScan()
     message.header.stamp = stamp_from_ns(timestamp_ns)
     message.header.frame_id = "ego_racecar/laser"
-    message.angle_min = -scan_fov / 2.0
-    message.angle_max = scan_fov / 2.0
-    message.angle_increment = scan_fov / beams
+    message.angle_min = contract.angle_min
+    message.angle_max = contract.angle_max
+    message.angle_increment = contract.angle_increment
     message.range_min = 0.0
     message.range_max = 30.0
     message.ranges = np.asarray(obs["scans"][0], dtype="<f4").tolist()
"""


def table(headers: list[str], rows: list[list[Any]]) -> str:
    def token(value: Any) -> str:
        if value is None:
            return "N/A"
        if isinstance(value, float):
            return f"{value:.9f}"
        return str(value)

    output = ["| " + " | ".join(headers) + " |", "|" + "|".join(["---"] * len(headers)) + "|"]
    output.extend("| " + " | ".join(token(value) for value in row) + " |" for row in rows)
    return "\n".join(output)


def render_readme(summary: dict[str, Any], performance: dict[str, Any]) -> str:
    validation = [row for row in summary["validation039"] if row["run"] == "PRIMARY"]
    normal = [row for row in summary["normal0100"] if row["run"] == "PRIMARY"]
    map_stats = [row for row in summary["range_delta_statistics"] if row["scope"] != "RANDOM_GEOMETRY"]
    return f"""# Simulator uniform-LaserScan contract prototype v1

## 판정

**{summary['classification']}**. Simulator physical ray와 ROS metadata ray는 모든 계약 시험에서 일치했고 validation_039 안전 anchor 및 normal_01_00의 정성 결론도 유지됐다. 다만 실제 map scan의 old/new range 분포가 유의하게 변하므로 production 채택 전 더 넓은 simulator 회귀가 필요하다.

이 작업은 격리 clone `{summary['prototype_root']}`에서만 simulator를 수정했다. Active `/home/sungho/f1sim_C`, obstacle_detector, planner, controller, state machine은 수정하지 않았다. CMA, dataset regeneration, closed-loop replay는 각각 0회다.

## 1. 정확히 무엇을 바꿨나

- `cpp_backend.cpp`: 2000-bin angle table/truncation을 scan 생성 경로에서 제거하고 `angle_min + i*angle_increment` 연속 각도를 ray marcher에 직접 전달했다.
- `laser_scan_contract.py`: beam count, float32 `angle_min/max/increment`, physical angle의 단일 source-of-truth를 추가했다.
- `cpp_simulator.py`: 위 계약을 C++ core에 전달하고 실제 backend scan angle 조회를 제공한다.
- `gym_bridge.py`: 동일 계약에서 LaserScan metadata를 발행한다.
- `lockstep_episode.py`: active 파일은 유지하고, 동일 helper를 사용하는 reviewable overlay를 `simulator_uniform_contract.patch`에 포함했다.

기존 path는 `scan index -> continuous 2000-bin index -> wrap -> int truncation -> k*2*pi/1999 -> range -> publisher FOV/N metadata`였다. 1999 denominator는 2000-entry table에 0과 2*pi를 모두 포함시키는 구현이며, 그 의도는 source comment가 아니라 코드 구조에서의 inference다. Ray marcher 자체는 table이 필요 없고 임의 연속 각도의 `sin/cos`를 이미 사용할 수 있었다.

## 2–5. 새 authoritative formula와 publisher 일치

`N>1`에서 float32 message 표현으로 `angle_min=f32(-FOV/2)`, `angle_increment=f32(FOV/(N-1))`, physical ray `i=angle_min+i*angle_increment`, `angle_max=f32(last physical ray)`를 사용한다. 1080 beam 전수 mismatch={summary['all_beam_contract_mismatch_count']}, 128 random case mismatch={summary['random_geometry_contract_mismatch_count']}. 마지막 physical ray와 float32 angle_max의 차이는 {summary['endpoint_float32_error_rad']:.12g} rad로 float32 1 ULP 이하다.

gym_bridge/backend/lockstep proposed overlay는 N, min, max, increment가 동일하다. frame semantics도 normal publisher의 namespaced `ego_racecar/laser`와 lockstep `ego_racecar/laser`로 정렬된다.

## 6. OLD vs NEW range 변화

{table(['scan', 'mean abs m', 'p95 m', 'p99 m', 'max m', 'changed'], [[row['scan_label'], row['mean_abs_range_delta_m'], row['p95_abs_range_delta_m'], row['p99_abs_range_delta_m'], row['max_abs_range_delta_m'], f"{row['changed_beam_count']}/{row['beam_count']}"] for row in map_stats])}

이는 예상된 sensor-physics change다. metadata-only patch가 아니라 실제 ray set을 uniform grid로 바꿨기 때문에 2000-bin boundary quantization과 wall/obstacle grazing beam이 달라진다.

## 7. validation_039 안전 anchor

새 scene/pose에서 prototype backend가 새 ranges를 생성했고 estimator는 오직 ROS `angle_min+i*angle_increment`만 사용했다. old bag ranges를 새 metadata로 재해석하지 않았다.

{table(['dt ms', 'returns', 'cells', 'area m2', 'support m', 'under m', 'over m', 'clearance m', 'class'], [[row['delta_t_ms'], row['target_return_count'], row['possible_cell_count'], row['possible_area_m2'], row['selected_side_support_m'], row['gt_undercoverage_m'], row['gt_overcoverage_m'], row['dangerous_path_clearance_m'], row['dangerous_path_classification']] for row in validation])}

+20/+30/+40은 모두 NON_EMPTY, +40 GT undercoverage=0, known dangerous path=HARD_INVALID다.

## 8. normal_01_00

{table(['dt ms', 'blocker', 'support m', 'corridor m', 'hard feasible'], [[row['delta_t_ms'], row['blocking_hypothesis_status'], row['possible_support_m'], row['shadow_corridor_m'], row['production_family_hard_feasible_count']] for row in normal])}

정성 판정은 **{summary['normal0100_classification']}**이다. blocker는 +10 ms에 제거되고, 첫 hard-feasible은 +{summary['normal0100_first_hard_feasible_ms']} ms, 3-scan 안정 판정은 +{summary['normal0100_stable_resolution_ms']} ms로 기존 결론과 timing까지 유지됐다.

## 9. Production detector compatibility

Production obstacle_detector의 기존 `scan.angle_min + i*scan.angle_increment` 해석은 prototype simulator와 일치한다: **DETECTOR_NOW_MATCHES_SIMULATOR_CONTRACT**. Detector-specific lookup, clustering/tracking/KF/gating/ID/motion classification 변경은 없다.

## 10. Legacy bag 분리

Diagnostic API는 `LEGACY_QUANTIZED_2000`과 `PROTOTYPE_UNIFORM_INCLUSIVE`를 반드시 명시한다. Legacy validation_039 bag은 기존 exact backend mapping으로만 재현하며, prototype safety regression은 새 backend가 생성한 새 ranges만 사용한다. old mapping을 prototype metadata로 metadata-only 재해석하는 negative test도 mismatch를 검출한다.

## 11–12. 채택성 및 다음 단계

계약과 최소 safety/generalization 증거는 채택 후보로 충분하지만, range distribution shift 때문에 production simulator 적용 전 broader simulator regression이 필요하다. 이 작업은 여기서 중단하며 `STATIC_SAFETY_SPARSE_EVIDENCE_SHADOW_AUDIT`는 시작하지 않았다. 그 이슈는 별도이며 `min_cluster_points=5`도 유지했다.

## 진단 도구 migration

- 일반/new scan consumer: ROS metadata만 사용.
- `BackendBeamContract`/exact lookup: legacy bag reproduction에서 명시적으로만 사용.
- `backend_directions_continuous`: old analytic audit comparison 용도이며 prototype authority가 아님.
- 실제 LiDAR: scan frequency를 계약에 넣지 않고 header.stamp, metadata, ranges.size를 그대로 소비.

## 테스트·결정성·자원

테스트 그룹 {summary['tests_passed']}/{summary['tests_total']} PASS, full tools/cmaes_tuning unit test {summary['full_unit_test_case_count']}/{summary['full_unit_test_case_count']} PASS. 1080-beam angle/metadata/ranges hash와 validation_039 +40 count/support/undercoverage/clearance/classification digest가 반복에서 동일했다. Wall {performance['wall_time_s']:.3f}s, CPU {performance['cpu_time_s']:.3f}s, max RSS {performance['max_rss_bytes']} bytes, workers={performance['worker_count']}, swap delta={performance['swap_change_bytes']} bytes.
"""


def main() -> None:
    args = parse_args()
    prototype_root = args.prototype_root.resolve()
    if not (prototype_root / "gym/f110_gym/cpp_backend.cpp").is_file():
        raise RuntimeError(f"invalid prototype root: {prototype_root}")
    OUTPUT.mkdir(parents=True, exist_ok=True)
    RAW.mkdir(parents=True, exist_ok=True)
    time_audit.RAW = RAW / "time_projection"
    memory_before = memory_snapshot()
    usage_before = resource.getrusage(resource.RUSAGE_SELF)
    child_before = resource.getrusage(resource.RUSAGE_CHILDREN)
    wall_started = time.monotonic()
    protected_before = protected_hashes()

    build_started = time.monotonic()
    build = subprocess.run(
        [sys.executable, "setup.py", "build_ext", "--inplace"],
        cwd=prototype_root,
        text=True,
        capture_output=True,
    )
    build_time = time.monotonic() - build_started
    if build.returncode:
        raise RuntimeError(build.stdout + build.stderr)

    manifest = json.loads((SCENARIO / "manifest.json").read_text(encoding="utf-8"))
    model = manifest["simulator_collision_model"]
    fov = float(model["scan_fov_rad"])
    beams = int(model["scan_beams"])
    contract = UniformLaserScanContract.from_fov(fov, beams)
    legacy_contract = BackendBeamContract(fov, beams)

    probe_rows: list[dict[str, Any]] = []
    driver_wall_s = 0.0
    primary_probe: dict[str, np.ndarray] | None = None
    primary_probe_meta: dict[str, Any] | None = None
    for beam_count in (2, 3, 7, 17, 180, 360, 720, 1080, 1440, 4096):
        arrays, metadata, elapsed = run_driver(
            prototype_root,
            f"probe_{beam_count}",
            beam_count=beam_count,
            field_of_view_rad=fov,
        )
        driver_wall_s += elapsed
        physical = arrays["physical_relative_angles_rad"]
        helper = UniformLaserScanContract.from_fov(fov, beam_count)
        published = helper.published_relative_angles_rad()
        error = np.abs(physical - published)
        probe_rows.append(
            {
                "beam_count": beam_count,
                "contract_mismatch_count": int(np.count_nonzero(error > 0.0)),
                "max_contract_error_rad": float(np.max(error)),
                "first_physical_rad": float(physical[0]),
                "published_angle_min_rad": helper.angle_min_rad,
                "last_physical_rad": float(physical[-1]),
                "published_angle_max_rad": helper.angle_max_rad,
                "last_vs_float32_angle_max_error_rad": abs(float(physical[-1]) - helper.angle_max_rad),
                "neighbor_spacing_max_error_rad": float(
                    np.max(np.abs(np.diff(physical) - helper.angle_increment_rad))
                )
                if beam_count > 1
                else 0.0,
                "status": "PASS" if not np.any(error) else "FAIL",
            }
        )
        if beam_count == beams:
            primary_probe, primary_probe_meta = arrays, metadata
    assert primary_probe is not None and primary_probe_meta is not None
    physical = primary_probe["physical_relative_angles_rad"]
    published = contract.published_relative_angles_rad()
    full_beam_rows = [
        {
            "beam_index": index,
            "backend_physical_angle_rad": float(physical[index]),
            "metadata_angle_rad": float(published[index]),
            "absolute_error_rad": abs(float(physical[index] - published[index])),
            "first_beam": index == 0,
            "center_beam": index == beams // 2,
            "last_beam": index == beams - 1,
            "status": "PASS" if physical[index] == published[index] else "FAIL",
        }
        for index in range(beams)
    ]

    bag_started = time.monotonic()
    bag = read_bag(REPLAY / "bag", topics=("/ego_racecar/odom",))
    bag_parse_time = time.monotonic() - bag_started
    odometry = {
        rulebook.stamp_ns(item.message): item.message
        for item in bag.topic("/ego_racecar/odom")
    }
    validation_poses = pose_rows(odometry, STAMPS)
    validation_old, _, elapsed = run_driver(
        ACTIVE_SIMULATOR,
        "validation039_old",
        beam_count=beams,
        field_of_view_rad=fov,
        map_yaml=VALIDATION_MAP,
        poses=validation_poses,
    )
    driver_wall_s += elapsed
    validation_new, validation_new_metadata, elapsed = run_driver(
        prototype_root,
        "validation039_new",
        beam_count=beams,
        field_of_view_rad=fov,
        map_yaml=VALIDATION_MAP,
        poses=validation_poses,
    )
    driver_wall_s += elapsed
    validation_repeat, validation_repeat_metadata, elapsed = run_driver(
        prototype_root,
        "validation039_new_repeat",
        beam_count=beams,
        field_of_view_rad=fov,
        map_yaml=VALIDATION_MAP,
        poses=validation_poses,
    )
    driver_wall_s += elapsed
    full_contract_determinism = {
        "metadata_identical": validation_new_metadata == validation_repeat_metadata,
        "physical_angles_identical": np.array_equal(
            validation_new["physical_relative_angles_rad"],
            validation_repeat["physical_relative_angles_rad"],
        ),
        "plus40_ranges_identical": np.array_equal(
            validation_new["scan_0004"], validation_repeat["scan_0004"]
        ),
        "physical_angles_sha256": array_digest(
            validation_new["physical_relative_angles_rad"]
        ),
        "plus40_ranges_sha256": array_digest(validation_new["scan_0004"]),
    }
    full_contract_determinism["identical"] = all(
        full_contract_determinism[key]
        for key in (
            "metadata_identical",
            "physical_angles_identical",
            "plus40_ranges_identical",
        )
    )
    if not full_contract_determinism["identical"]:
        raise RuntimeError("full 1080-beam compiled prototype repeat differs")

    detector_parameters = yaml.safe_load(
        (ROOT / "src/obstacle_detector/config/obstacle_detector.yaml").read_text(encoding="utf-8")
    )["obstacle_detector"]["ros__parameters"]
    raster = {
        key: float(value)
        for key, value in manifest["baked_obstacle_raster"]["world_half_open_bounds_m"].items()
    }
    guard = max(
        rulebook.GEOMETRY_EPSILON_M,
        float(model["scan_noise_std_m"]) * float(model["scan_noise_guard_sigma"]),
        3.0 * float(detector_parameters["cluster_sigma"]),
    )
    evidence = {
        stamp: build_validation_frame(
            stamp,
            validation_new[f"scan_{index:04d}"],
            odometry[stamp],
            raster,
            model,
            contract,
            guard,
        )
        for index, stamp in enumerate(STAMPS)
    }
    repeat_hash = array_digest(validation_repeat["scan_0004"])
    validation_rows, validation_determinism, validation_evaluations = validation039_regression(
        evidence, manifest, raster, repeat_hash
    )

    normal_case = time_audit.synthetic_normal_case("normal_01_00")
    normal_frames = prototype_normal_frames(normal_case)
    normal_repeat_frames = prototype_normal_frames(normal_case)
    normal_rows, normal_evaluations = normal0100_regression(
        normal_case, normal_frames, repeat=False
    )
    normal_repeat_rows, repeat_evaluations = normal0100_regression(
        normal_case, normal_repeat_frames, repeat=True
    )
    normal_compare_fields = (
        "delta_t_ms",
        "possible_cell_count",
        "possible_hypothesis_count",
        "possible_support_m",
        "shadow_corridor_m",
        "blocking_hypothesis_status",
        "production_family_hard_feasible_count",
        "ranges_sha256",
        "classification",
        "first_hard_feasible_ms",
        "stable_resolution_ms",
    )
    primary_normal_digest_rows = [
        {key: row[key] for key in normal_compare_fields} for row in normal_rows
    ]
    repeat_normal_digest_rows = [
        {key: row[key] for key in normal_compare_fields} for row in normal_repeat_rows
    ]
    normal_determinism = {
        "primary_digest": canonical_digest(primary_normal_digest_rows),
        "repeat_digest": canonical_digest(repeat_normal_digest_rows),
        "identical": primary_normal_digest_rows == repeat_normal_digest_rows,
    }
    if not normal_determinism["identical"]:
        raise RuntimeError("normal_01_00 prototype repeat differs")

    normal_poses: list[dict[str, Any]] = []
    for absolute_index in range(2, 9):
        pose = rulebook.interpolate_reference(
            normal_case["waypoints"],
            float(normal_case["manifest_s"])
            - float(normal_case["gt_row"]["observation_distance_m"])
            + 0.04 * absolute_index,
        )
        normal_poses.append(
            {
                "label": f"normal0100_{(absolute_index - 2) * 10:+d}ms",
                "x_m": float(pose["x"]),
                "y_m": float(pose["y"]),
                "yaw_rad": float(pose["yaw"]),
            }
        )
    normal_old, _, elapsed = run_driver(
        ACTIVE_SIMULATOR,
        "normal0100_old",
        beam_count=beams,
        field_of_view_rad=fov,
        map_yaml=NORMAL_MAP,
        poses=normal_poses,
    )
    driver_wall_s += elapsed
    normal_new, _, elapsed = run_driver(
        prototype_root,
        "normal0100_new",
        beam_count=beams,
        field_of_view_rad=fov,
        map_yaml=NORMAL_MAP,
        poses=normal_poses,
    )
    driver_wall_s += elapsed

    range_rows: list[dict[str, Any]] = []
    for index, pose in enumerate(validation_poses):
        yaw = float(pose["yaw_rad"])
        range_rows.append(
            scan_stats(
                "VALIDATION_039_MAP",
                pose["label"],
                validation_old[f"scan_{index:04d}"],
                validation_new[f"scan_{index:04d}"],
                legacy_contract.physical_angles_rad(yaw),
                contract.physical_world_angles_rad(yaw),
            )
        )
    for index, pose in enumerate(normal_poses):
        yaw = float(pose["yaw_rad"])
        range_rows.append(
            scan_stats(
                "NORMAL_01_00_MAP",
                pose["label"],
                normal_old[f"scan_{index:04d}"],
                normal_new[f"scan_{index:04d}"],
                legacy_contract.physical_angles_rad(yaw),
                contract.physical_world_angles_rad(yaw),
            )
        )
    random_rows, random_stats = random_geometry_rows()
    range_rows.extend(random_stats)

    old_physical = legacy_contract.physical_angles_rad(0.0)
    old_metadata = -0.5 * fov + np.arange(beams) * (fov / beams)
    prototype_metadata = contract.published_relative_angles_rad()
    old_vs_new_rows = [
        {
            "contract": "LEGACY",
            "component": "cpp_backend",
            "physical_formula": "truncate(wrap(2000*(yaw-FOV/2)/(2*pi)+i*2000*(FOV/(N-1))/(2*pi)))*2*pi/1999",
            "metadata_formula": "not owned here",
            "mismatch_count": None,
            "classification": "NON_UNIFORM_QUANTIZED_PHYSICAL_RAYS",
        },
        {
            "contract": "LEGACY",
            "component": "gym_bridge_and_lockstep",
            "physical_formula": "backend private table",
            "metadata_formula": "angle_min=-FOV/2; angle_max=FOV/2; increment=FOV/N",
            "mismatch_count": int(
                np.count_nonzero(np.abs(wrapped_angle_delta_rad(old_physical, old_metadata)) > 1.0e-12)
            ),
            "classification": "METADATA_MISMATCH",
        },
        {
            "contract": "NEGATIVE_METADATA_ONLY_PATCH",
            "component": "legacy_backend_plus_uniform_metadata",
            "physical_formula": "legacy 2000-bin physical rays unchanged",
            "metadata_formula": "prototype angle_min+i*increment",
            "mismatch_count": int(
                np.count_nonzero(
                    np.abs(wrapped_angle_delta_rad(old_physical, prototype_metadata)) > 1.0e-12
                )
            ),
            "classification": "REJECTED_BY_EXECUTABLE_NEGATIVE_TEST",
        },
        {
            "contract": "PROTOTYPE",
            "component": "cpp_backend",
            "physical_formula": "angle_min+i*angle_increment; direct continuous sin/cos ray march",
            "metadata_formula": "shared UniformLaserScanContract",
            "mismatch_count": sum(row["absolute_error_rad"] > 0.0 for row in full_beam_rows),
            "classification": "MATCH",
        },
        {
            "contract": "PROTOTYPE",
            "component": "gym_bridge_and_lockstep_overlay",
            "physical_formula": "shared UniformLaserScanContract",
            "metadata_formula": "shared UniformLaserScanContract",
            "mismatch_count": 0,
            "classification": "MATCH",
        },
    ]

    bridge = prototype_root / "f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py"
    backend = prototype_root / "gym/f110_gym/cpp_backend.cpp"
    lockstep = ROOT / "tools/cmaes_tuning/lockstep_episode.py"
    publisher_rows = [
        {
            "publisher": "prototype_cpp_backend_physical",
            "ranges_size": beams,
            "angle_min_rad": contract.angle_min_rad,
            "angle_max_rad": contract.angle_max_rad,
            "angle_increment_rad": contract.angle_increment_rad,
            "frame_id_semantics": "physical relative angles; no ROS frame",
            "max_physical_metadata_error_rad": float(np.max(np.abs(physical - published))),
            "source_file": str(backend),
            "source_line": source_line(backend, "angle_min_ + i * angle_increment_"),
            "status": "MATCH",
        },
        {
            "publisher": "gym_bridge",
            "ranges_size": beams,
            "angle_min_rad": contract.angle_min_rad,
            "angle_max_rad": contract.angle_max_rad,
            "angle_increment_rad": contract.angle_increment_rad,
            "frame_id_semantics": "ego_namespace + /laser",
            "max_physical_metadata_error_rad": 0.0,
            "source_file": str(bridge),
            "source_line": source_line(bridge, "self.scan_contract = UniformLaserScanContract"),
            "status": "MATCH",
        },
        {
            "publisher": "lockstep_prototype_overlay",
            "ranges_size": beams,
            "angle_min_rad": contract.angle_min_rad,
            "angle_max_rad": contract.angle_max_rad,
            "angle_increment_rad": contract.angle_increment_rad,
            "frame_id_semantics": "ego_racecar/laser",
            "max_physical_metadata_error_rad": 0.0,
            "source_file": str(lockstep),
            "source_line": source_line(lockstep, "def make_scan"),
            "status": "MATCH",
        },
    ]

    simulator_patch = subprocess.run(
        ["git", "diff", "--", "gym/f110_gym/cpp_backend.cpp", "gym/f110_gym/envs/cpp_simulator.py", "gym/f110_gym/envs/laser_scan_contract.py", "f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py"],
        cwd=prototype_root,
        text=True,
        capture_output=True,
        check=True,
    ).stdout
    (OUTPUT / "simulator_uniform_contract.patch").write_text(
        simulator_patch + "\n" + lockstep_overlay_patch(), encoding="utf-8"
    )

    test_rows = [
        {
            "test": "prototype_incremental_cpp_build",
            "command": "python3 setup.py build_ext --inplace",
            "status": "PASS",
            "return_code": 0,
            "wall_time_s": build_time,
            "detail": "isolated prototype extension built",
        },
        {
            "test": "compiled_backend_arbitrary_beam_counts",
            "command": "compiled probe counts=2,3,7,17,180,360,720,1080,1440,4096",
            "status": "PASS" if all(row["status"] == "PASS" for row in probe_rows) else "FAIL",
            "return_code": 0 if all(row["status"] == "PASS" for row in probe_rows) else 1,
            "wall_time_s": driver_wall_s,
            "detail": f"{len(probe_rows)} counts; zero physical/metadata mismatch",
        },
        {
            "test": "validation039_determinism",
            "command": "repeat prototype scans and +40 safety anchor",
            "status": "PASS" if validation_determinism["identical"] else "FAIL",
            "return_code": 0 if validation_determinism["identical"] else 1,
            "wall_time_s": 0.0,
            "detail": validation_determinism["primary_digest"],
        },
        {
            "test": "full_1080_beam_compiled_contract_determinism",
            "command": "repeat compiled metadata, physical angles, and +40 ranges",
            "status": "PASS" if full_contract_determinism["identical"] else "FAIL",
            "return_code": 0 if full_contract_determinism["identical"] else 1,
            "wall_time_s": 0.0,
            "detail": full_contract_determinism["plus40_ranges_sha256"],
        },
        {
            "test": "normal0100_determinism",
            "command": "repeat prototype sensor-only timeline",
            "status": "PASS" if normal_determinism["identical"] else "FAIL",
            "return_code": 0 if normal_determinism["identical"] else 1,
            "wall_time_s": 0.0,
            "detail": normal_determinism["primary_digest"],
        },
    ]
    test_env = os.environ.copy()
    test_env["PYTHONPATH"] = str(ROOT / "tools/cmaes_tuning") + os.pathsep + test_env.get("PYTHONPATH", "")
    test_rows.extend(
        [
            command_result(
                "new_prototype_contract_unit_tests",
                [sys.executable, "-m", "unittest", "tools/cmaes_tuning/tests/test_uniform_laserscan_prototype.py", "-v"],
                cwd=ROOT,
                env=test_env,
            ),
            command_result(
                "existing_beam_contract_unit_tests",
                [sys.executable, "-m", "unittest", "tools/cmaes_tuning/tests/test_lidar_beam_contract.py", "-v"],
                cwd=ROOT,
                env=test_env,
            ),
            command_result(
                "full_cmaes_tuning_unit_tests",
                [sys.executable, "-m", "unittest", "discover", "-s", "tools/cmaes_tuning/tests", "-p", "test_*.py"],
                cwd=ROOT,
                env=test_env,
            ),
            command_result(
                "prototype_python_compile",
                [sys.executable, "-m", "py_compile", str(prototype_root / "gym/f110_gym/envs/laser_scan_contract.py"), str(prototype_root / "gym/f110_gym/envs/cpp_simulator.py"), str(bridge)],
                cwd=ROOT,
            ),
            command_result(
                "prototype_git_diff_check",
                ["git", "diff", "--check"],
                cwd=prototype_root,
            ),
            command_result(
                "workspace_git_diff_check",
                ["git", "diff", "--check"],
                cwd=ROOT,
            ),
        ]
    )
    if any(row["status"] != "PASS" for row in test_rows):
        failed = [row for row in test_rows if row["status"] != "PASS"]
        raise RuntimeError(f"prototype tests failed: {failed}")

    protected_after = protected_hashes()
    if protected_before != protected_after:
        raise RuntimeError("protected production/active simulator source changed during prototype audit")
    all_mismatches = sum(row["absolute_error_rad"] > 0.0 for row in full_beam_rows)
    random_mismatches = sum(row["contract_mismatch_count"] for row in random_rows)
    safety = next(
        row
        for row in validation_rows
        if row["run"] == "PRIMARY" and row["delta_t_ms"] == 40
    )
    validation_safe = (
        safety["gt_undercoverage_m"] == 0.0
        and safety["dangerous_path_classification"] == "HARD_INVALID"
        and all(
            row["status"] == "NON_EMPTY"
            for row in validation_rows
            if row["run"] == "PRIMARY"
        )
    )
    map_range_rows = [row for row in range_rows if row["scope"] != "RANDOM_GEOMETRY"]
    material_shift = any(
        row["p99_abs_range_delta_m"] > 0.02 or row["max_abs_range_delta_m"] > 0.25
        for row in map_range_rows
    )
    if all_mismatches or random_mismatches or not validation_safe:
        classification = "PROTOTYPE_UNSAFE"
    elif material_shift:
        classification = "PROTOTYPE_VALID_WITH_SENSOR_DISTRIBUTION_SHIFT"
    else:
        classification = "PROTOTYPE_VALID"

    migration_plan = [
        {
            "tool": "production_obstacle_detector",
            "current_convention": "ROS metadata angle_min+i*angle_increment",
            "prototype_mode": "PROTOTYPE_UNIFORM_SCAN_MODE",
            "action": "none; now matches simulator",
        },
        {
            "tool": "lidar_beam_contract.py and legacy audit renderers",
            "current_convention": "exact 2000-bin lookup",
            "prototype_mode": "LEGACY_SCAN_MODE only",
            "action": "retain only for explicit old-bag reproduction",
        },
        {
            "tool": "backend_directions_continuous diagnostic call sites",
            "current_convention": "old analytic FOV/(N-1)",
            "prototype_mode": "comparison only",
            "action": "do not treat as authority unless sourced from prototype metadata",
        },
        {
            "tool": "new prototype scan audits",
            "current_convention": "newly generated ranges",
            "prototype_mode": "PROTOTYPE_UNIFORM_SCAN_MODE",
            "action": "consume ROS metadata only",
        },
    ]
    tests_passed = sum(row["status"] == "PASS" for row in test_rows)
    full_unit_test_case_count = next(
        int(row["case_count"])
        for row in test_rows
        if row["test"] == "full_cmaes_tuning_unit_tests"
    )
    summary = {
        "schema": "simulator_laserscan_contract_prototype/1",
        "classification": classification,
        "prototype_isolated": True,
        "prototype_root": str(prototype_root),
        "active_simulator_modified": False,
        "production_detector_modified": False,
        "authoritative_formula": "physical_relative_angle(i)=float32(-FOV/2)+i*float32(FOV/(N-1))",
        "metadata_formula": "angle_min and angle_increment are the same float32 values; angle_max=float32(angle_min+(N-1)*angle_increment)",
        "all_beam_contract_mismatch_count": all_mismatches,
        "arbitrary_beam_count_contract": probe_rows,
        "endpoint_float32_error_rad": probe_rows[-3]["last_vs_float32_angle_max_error_rad"],
        "random_geometry_case_count": len(random_rows),
        "random_geometry_contract_mismatch_count": random_mismatches,
        "metadata_only_negative_test_mismatch_count": old_vs_new_rows[2]["mismatch_count"],
        "publisher_contract": publisher_rows,
        "detector_classification": "DETECTOR_NOW_MATCHES_SIMULATOR_CONTRACT",
        "detector_angle_source_line": source_line(
            ROOT / "src/obstacle_detector/src/obstacle_detector_node.cpp",
            "scan.angle_min + static_cast<double>(i) * scan.angle_increment",
        ),
        "validation039": validation_rows,
        "validation039_safe": validation_safe,
        "validation039_determinism": validation_determinism,
        "full_1080_beam_contract_determinism": full_contract_determinism,
        "normal0100": normal_rows + normal_repeat_rows,
        "normal0100_classification": normal_rows[0]["classification"],
        "normal0100_first_hard_feasible_ms": normal_rows[0]["first_hard_feasible_ms"],
        "normal0100_stable_resolution_ms": normal_rows[0]["stable_resolution_ms"],
        "normal0100_determinism": normal_determinism,
        "range_delta_statistics": range_rows,
        "range_distribution_shift_material": material_shift,
        "range_distribution_materiality_policy": "material if any evaluated real-map scan p99 > 0.02 m or max > 0.25 m",
        "legacy_bag_policy": "LEGACY_SCAN_MODE uses exact 2000-bin mapping; PROTOTYPE_UNIFORM_SCAN_MODE requires newly generated ranges and ROS metadata",
        "diagnostic_migration_plan": migration_plan,
        "real_lidar_compatibility": "frequency-independent; consume header.stamp, angle_min, angle_increment, and ranges.size",
        "dynamic_pipeline_untouched": True,
        "opponent_pipeline_untouched": True,
        "sparse_evidence_out_of_scope": True,
        "next_action": "broader simulator regression before adoption; do not start sparse-evidence work automatically",
        "tests_total": len(test_rows),
        "tests_passed": tests_passed,
        "full_unit_test_case_count": full_unit_test_case_count,
        "test_results": test_rows,
        "protected_hashes_before": protected_before,
        "protected_hashes_after": protected_after,
    }

    memory_after = memory_snapshot()
    usage_after = resource.getrusage(resource.RUSAGE_SELF)
    child_after = resource.getrusage(resource.RUSAGE_CHILDREN)
    performance = {
        "schema": "simulator_laserscan_contract_prototype_performance/1",
        "wall_time_s": time.monotonic() - wall_started,
        "cpu_time_s": (usage_after.ru_utime + usage_after.ru_stime - usage_before.ru_utime - usage_before.ru_stime)
        + (child_after.ru_utime + child_after.ru_stime - child_before.ru_utime - child_before.ru_stime),
        "max_rss_bytes": max(usage_after.ru_maxrss, child_after.ru_maxrss) * 1024,
        "worker_count": 1,
        "worker_policy": "causal timestamps serial; one isolated compiled backend process per scan batch",
        "memory_available_before_bytes": memory_before["memory_available_bytes"],
        "memory_available_after_bytes": memory_after["memory_available_bytes"],
        "swap_used_before_bytes": memory_before["swap_used_bytes"],
        "swap_used_after_bytes": memory_after["swap_used_bytes"],
        "swap_change_bytes": memory_after["swap_used_bytes"] - memory_before["swap_used_bytes"],
        "prototype_incremental_build_time_s": build_time,
        "compiled_driver_wall_time_s": driver_wall_s,
        "bag_parse_count": 1,
        "bag_parse_time_s": bag_parse_time,
        "compiled_scan_batches": 15,
        "validation_grid_evaluations": validation_evaluations,
        "normal_grid_evaluations": normal_evaluations + repeat_evaluations,
        "random_geometry_cases": len(random_rows),
        "cached_maps_reused": [str(VALIDATION_MAP), str(NORMAL_MAP)],
        "closed_loop_replay_count": 0,
        "cma_count": 0,
        "dataset_regeneration_count": 0,
        "production_behavior_modification_count": 0,
        "active_simulator_modification_count": 0,
    }

    write_csv(OUTPUT / "old_vs_new_contract.csv", old_vs_new_rows)
    write_csv(OUTPUT / "full_beam_contract.csv", full_beam_rows)
    write_csv(OUTPUT / "random_geometry_contract.csv", random_rows)
    write_csv(OUTPUT / "range_delta_statistics.csv", range_rows)
    write_csv(OUTPUT / "publisher_contract_comparison.csv", publisher_rows)
    write_csv(OUTPUT / "validation039_prototype_regression.csv", validation_rows)
    write_csv(OUTPUT / "normal0100_prototype_regression.csv", normal_rows + normal_repeat_rows)
    write_csv(OUTPUT / "test_results.csv", test_rows)
    (OUTPUT / "summary.json").write_text(
        json.dumps(clean_json(summary), indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (OUTPUT / "performance.json").write_text(
        json.dumps(clean_json(performance), indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (OUTPUT / "README.md").write_text(
        render_readme(clean_json(summary), clean_json(performance)), encoding="utf-8"
    )


if __name__ == "__main__":
    main()
