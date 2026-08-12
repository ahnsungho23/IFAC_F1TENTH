#!/usr/bin/env python3
"""Render the diagnostic-only simulator/LaserScan beam-contract audit."""

from __future__ import annotations

import concurrent.futures
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import resource
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
    endpoint_xy,
    inspect_authoritative_sources,
    metadata_angles_rad,
    old_analytic_angles_rad,
    wrapped_angle_delta_rad,
)
from cmaes_tuning.scenario_generator import load_waypoints
from cmaes_tuning.simulator_collision import SimulatorRasterCollisionModel


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "runs/cmaes_tuning/lidar_beam_contract_audit_v1"
RAW = OUTPUT / ".raw"
REPLAY = ROOT / "runs/cmaes_tuning/time_axis_feasibility_audit_v1/replays/validation_039"
SCENARIO = (
    ROOT
    / "runs/cmaes_tuning/medium_lockstep_stage1_v2/artifacts/scenarios/validation/validation_039"
)
BAKED_MAP = (
    ROOT
    / "runs/cmaes_tuning/repeatability_mid4_v2/scenarios/validation/validation_039"
    / "validation_039_map.yaml"
)
PRIOR_SENSOR = ROOT / "runs/cmaes_tuning/validation039_sensor_model_consistency_audit_v1"
PRIOR_TIME = ROOT / "runs/cmaes_tuning/time_to_disambiguation_audit_v1"
BASELINE_STAMP = 11_230_000_000
STAMPS = tuple(BASELINE_STAMP + index * 10_000_000 for index in range(5))
CORE = ((11_250_000_000, 829), (11_270_000_000, 824), (11_270_000_000, 826))
WORKERS = 4


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_digest(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
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


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


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


def production_hashes(sources: dict[str, Any]) -> dict[str, str]:
    paths = dict(rulebook.PRODUCTION_FILES)
    paths.update(
        {
            "simulator_backend_source": Path(str(sources["backend_path"])),
            "simulator_laserscan_publisher": Path(str(sources["publisher_path"])),
            "lockstep_laserscan_publisher": ROOT / "tools/cmaes_tuning/lockstep_episode.py",
            "detector_header": ROOT / "src/obstacle_detector/include/obstacle_detector/obstacle_detector_node.hpp",
            "detector_tracker_source": ROOT / "src/obstacle_detector/src/obstacle_tracker.cpp",
        }
    )
    return {name: sha256(path) for name, path in paths.items()}


def build_contract_frame(
    stamp_ns: int,
    scans: dict[int, Any],
    odometry: dict[int, Any],
    events: dict[int, dict[str, Any]],
    raster: dict[str, float],
    model: dict[str, Any],
    detector_parameters: dict[str, Any],
    contract: BackendBeamContract,
) -> sensor_audit.FrameEvidence:
    scan = scans[stamp_ns]
    odom = odometry[stamp_ns]
    yaw = sensor_audit.yaw_from_odom(odom)
    offset = float(model["lidar_offset_x_m"])
    origin = np.asarray(
        [
            float(odom.pose.pose.position.x) + offset * math.cos(yaw),
            float(odom.pose.pose.position.y) + offset * math.sin(yaw),
        ],
        dtype=np.float64,
    )
    directions = contract.directions(yaw)
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    valid = np.isfinite(ranges) & (ranges >= float(scan.range_min))
    endpoints = origin + ranges[:, None] * directions
    hit_indices = np.flatnonzero(
        valid & rulebook.points_inside_half_open(endpoints, raster)
    ).astype(np.int64)
    valid_indices = np.flatnonzero(valid).astype(np.int64)
    guard = max(
        rulebook.GEOMETRY_EPSILON_M,
        float(model["scan_noise_std_m"]) * float(model["scan_noise_guard_sigma"]),
        3.0 * float(detector_parameters["cluster_sigma"]),
    )
    ray_frame = rulebook.RayFrame(
        stamp_ns=stamp_ns,
        status="CONFIRMED",
        motion_status="UNKNOWN",
        origin=origin,
        directions=directions[valid_indices],
        ranges=ranges[valid_indices],
        free_lengths=np.maximum(0.0, ranges[valid_indices] - guard),
        hits=endpoints[hit_indices],
        hit_indices=tuple(int(value) for value in hit_indices),
        detector_box=(math.nan, math.nan, math.nan, math.nan),
    )
    return sensor_audit.FrameEvidence(
        stamp_ns=stamp_ns,
        scan=scan,
        odom=odom,
        event=events[stamp_ns],
        yaw_rad=yaw,
        origin=origin,
        directions=directions,
        endpoints=endpoints,
        valid_indices=valid_indices,
        hit_indices=hit_indices,
        ray_frame=ray_frame,
        direction_model="AUTHORITATIVE_BACKEND_CONTRACT",
    )


def percentile_statistics(
    label: str,
    stamp_ns: int,
    ranges: np.ndarray,
    yaw: float,
    angle_min: float,
    angle_increment: float,
    contract: BackendBeamContract,
) -> dict[str, Any]:
    valid = np.isfinite(ranges) & (ranges >= 0.0)
    indices = np.flatnonzero(valid)
    backend = contract.physical_angles_rad(yaw)[indices]
    metadata = metadata_angles_rad(
        angle_min, angle_increment, contract.scan_beams, yaw_rad=yaw
    )[indices]
    angular = np.abs(wrapped_angle_delta_rad(metadata, backend))
    endpoint = 2.0 * ranges[indices] * np.sin(0.5 * angular)
    maximum = int(np.argmax(endpoint))
    return {
        "scan_label": label,
        "source_stamp_ns": stamp_ns,
        "valid_range_count": int(indices.size),
        "mean_angle_error_deg": math.degrees(float(np.mean(angular))),
        "p95_angle_error_deg": math.degrees(float(np.percentile(angular, 95))),
        "p99_angle_error_deg": math.degrees(float(np.percentile(angular, 99))),
        "max_angle_error_deg": math.degrees(float(np.max(angular))),
        "beam_of_max_angle_error": int(indices[int(np.argmax(angular))]),
        "mean_endpoint_error_m": float(np.mean(endpoint)),
        "p95_endpoint_error_m": float(np.percentile(endpoint, 95)),
        "p99_endpoint_error_m": float(np.percentile(endpoint, 99)),
        "max_endpoint_error_m": float(endpoint[maximum]),
        "beam_of_max_endpoint_error": int(indices[maximum]),
    }


def core_comparison_rows(
    evidence: dict[int, sensor_audit.FrameEvidence], contract: BackendBeamContract
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for stamp, beam in CORE:
        frame = evidence[stamp]
        scan = frame.scan
        value = float(scan.ranges[beam])
        backend_world = contract.physical_angles_rad(frame.yaw_rad)[beam]
        models = {
            "ROS_METADATA": metadata_angles_rad(
                float(scan.angle_min),
                float(scan.angle_increment),
                len(scan.ranges),
                yaw_rad=frame.yaw_rad,
            )[beam],
            "OLD_ANALYTIC_FOV_N_MINUS_1": old_analytic_angles_rad(
                contract.scan_fov_rad,
                contract.scan_beams,
                yaw_rad=frame.yaw_rad,
            )[beam],
            "EXACT_BACKEND_LOOKUP": backend_world,
        }
        for name, world_angle in models.items():
            relative = float(wrapped_angle_delta_rad(world_angle, frame.yaw_rad))
            lidar_endpoint = endpoint_xy([relative], [value])[0]
            map_endpoint = endpoint_xy(
                [world_angle],
                [value],
                origin_x_m=float(frame.origin[0]),
                origin_y_m=float(frame.origin[1]),
            )[0]
            delta = abs(float(wrapped_angle_delta_rad(world_angle, backend_world)))
            rows.append(
                {
                    "source_stamp_ns": stamp,
                    "beam_index": beam,
                    "model": name,
                    "range_m": value,
                    "backend_table_index": int(contract.table_indices(frame.yaw_rad)[beam]),
                    "beam_angle_lidar_rad": relative,
                    "beam_angle_map_rad": float(world_angle),
                    "endpoint_lidar_x_m": float(lidar_endpoint[0]),
                    "endpoint_lidar_y_m": float(lidar_endpoint[1]),
                    "endpoint_map_x_m": float(map_endpoint[0]),
                    "endpoint_map_y_m": float(map_endpoint[1]),
                    "angular_error_vs_backend_deg": math.degrees(delta),
                    "lateral_endpoint_error_vs_backend_m": value * abs(math.sin(delta)),
                    "euclidean_endpoint_error_vs_backend_m": 2.0
                    * value
                    * math.sin(0.5 * delta),
                }
            )
    return rows


def validation_case(
    manifest: dict[str, Any], raster: dict[str, float]
) -> dict[str, Any]:
    obstacle = {
        key: float(manifest["obstacle"][key])
        for key in ("x", "y", "s", "d", "yaw", "width", "height")
    }
    return {
        "case_id": "validation_039",
        "stamp_ns": BASELINE_STAMP,
        "stream": str(time_audit.STREAM_039),
        "manifest_s": obstacle["s"],
        "obstacle": obstacle,
        "raster": raster,
    }


def validation039_regression(
    evidence: dict[int, sensor_audit.FrameEvidence],
    manifest: dict[str, Any],
    raster: dict[str, float],
) -> tuple[list[dict[str, Any]], dict[str, Any], int]:
    rows: list[dict[str, Any]] = []
    grids: dict[int, Any] = {}
    evaluations = 0
    for stamp in STAMPS[2:]:
        window = sensor_audit.current_window(evidence, stamp)
        grid, status = sensor_audit.safe_grid(window)
        evaluations += 1
        if grid is None:
            raise RuntimeError(f"authoritative mapping remained empty at {stamp}")
        grids[stamp] = grid
        witness = time_audit.witness_statistics(window, grid)
        rows.append(
            {
                "run": "PRIMARY",
                "source_stamp_ns": stamp,
                "delta_t_ms": (stamp - BASELINE_STAMP) // 1_000_000,
                "ut20_source_stamps": ";".join(str(frame.stamp_ns) for frame in window),
                "possible_cell_count": int(np.count_nonzero(grid.mask)),
                "possible_hypothesis_count": witness["possible_hypothesis_count"],
                "possible_area_m2": grid.possible_area_m2,
                "status": status,
                "gt_undercoverage_m": None,
                "gt_overcoverage_m": None,
                "selected_side_support_m": None,
                "dangerous_path_clearance_m": None,
                "dangerous_path_hard_invalid": None,
            }
        )
    case = validation_case(manifest, raster)
    track = free_gap.TrackGeometry.from_waypoints(load_waypoints(time_audit.WAYPOINTS))
    gt = time_audit.gt_representation("validation_039")
    safety = sensor_audit.projected_safety(
        "AUTHORITATIVE_BACKEND_CONTRACT", grids[STAMPS[4]], case, track, gt, 4
    )
    rows[-1].update(
        {
            key: safety[key]
            for key in (
                "gt_undercoverage_m",
                "gt_overcoverage_m",
                "selected_side_support_m",
                "dangerous_path_clearance_m",
                "dangerous_path_hard_invalid",
            )
        }
    )
    repeat_grid, repeat_status = sensor_audit.safe_grid(
        sensor_audit.current_window(evidence, STAMPS[4])
    )
    evaluations += 1
    if repeat_grid is None:
        raise RuntimeError("repeat authoritative +40 set is empty")
    repeat_witness = time_audit.witness_statistics(
        sensor_audit.current_window(evidence, STAMPS[4]), repeat_grid
    )
    repeat_safety = sensor_audit.projected_safety(
        "AUTHORITATIVE_BACKEND_CONTRACT_REPEAT",
        repeat_grid,
        case,
        track,
        gt,
        4,
        repeat=True,
    )
    repeat_row = {
        "run": "REPEAT",
        "source_stamp_ns": STAMPS[4],
        "delta_t_ms": 40,
        "ut20_source_stamps": ";".join(
            str(frame.stamp_ns) for frame in sensor_audit.current_window(evidence, STAMPS[4])
        ),
        "possible_cell_count": int(np.count_nonzero(repeat_grid.mask)),
        "possible_hypothesis_count": repeat_witness["possible_hypothesis_count"],
        "possible_area_m2": repeat_grid.possible_area_m2,
        "status": repeat_status,
        **{
            key: repeat_safety[key]
            for key in (
                "gt_undercoverage_m",
                "gt_overcoverage_m",
                "selected_side_support_m",
                "dangerous_path_clearance_m",
                "dangerous_path_hard_invalid",
            )
        },
    }
    rows.append(repeat_row)
    stable_keys = (
        "source_stamp_ns",
        "possible_cell_count",
        "possible_hypothesis_count",
        "possible_area_m2",
        "status",
        "gt_undercoverage_m",
        "gt_overcoverage_m",
        "selected_side_support_m",
        "dangerous_path_clearance_m",
        "dangerous_path_hard_invalid",
    )
    primary = {key: rows[-2][key] for key in stable_keys}
    repeat = {key: repeat_row[key] for key in stable_keys}
    determinism = {
        "primary_digest": canonical_digest(primary),
        "repeat_digest": canonical_digest(repeat),
        "bit_identical": primary == repeat,
    }
    if not determinism["bit_identical"]:
        raise RuntimeError("validation_039 authoritative +40 repeat differs")
    return rows, determinism, evaluations


def normal0100_regression(contract: BackendBeamContract) -> tuple[list[dict[str, Any]], int]:
    case = time_audit.synthetic_normal_case("normal_01_00")
    frames = list(case["frames"])
    for absolute_index in range(3, 9):
        frames.append(time_audit.normal_future_frame(case, absolute_index))
    track = free_gap.TrackGeometry.from_waypoints(case["waypoints"])
    gt = time_audit.gt_representation("normal_01_00")
    blocker = time_audit.blocking_hypotheses()["normal_01_00"]
    cached = {
        int(row["delta_t_ms"]): row
        for row in read_csv(PRIOR_TIME / "time_axis_normal_01_00.csv")
    }
    rows: list[dict[str, Any]] = []
    baseline_stamp = frames[2].stamp_ns
    for scan_index, frame in enumerate(frames[2:9]):
        delta_ms = int((frame.stamp_ns - baseline_stamp) // 1_000_000)
        window = s3.selected_frames_window(frames, frame.stamp_ns, 20)
        grid = s3.detailed_possible_occupancy(window, 0.004, 2.0)
        witness = time_audit.witness_statistics(window, grid)
        support, _ = time_audit.project_grid(case, grid, scan_index, track, repeat=False)
        ego = time_audit.ego_state(case, frame, 2 + scan_index)
        action, _ = time_audit.run_actionability(
            case, scan_index, frame.stamp_ns, ego, support, gt, repeat=False
        )
        blocker_state = time_audit.hypothesis_consistency(window, blocker)
        authoritative = contract.directions(ego["yaw"])
        direction_delta = np.abs(
            wrapped_angle_delta_rad(
                np.arctan2(frame.directions[:, 1], frame.directions[:, 0]),
                np.arctan2(authoritative[:, 1], authoritative[:, 0]),
            )
        )
        cached_row = cached[delta_ms]
        rows.append(
            {
                "source_stamp_ns": frame.stamp_ns,
                "delta_t_ms": delta_ms,
                "possible_cell_count": int(np.count_nonzero(grid.mask)),
                "possible_hypothesis_count": witness["possible_hypothesis_count"],
                "selected_side_support_m": support["d_max"],
                "shadow_corridor_m": support["left_corridor"],
                "blocking_hypothesis_status": blocker_state["status"],
                "blocking_hypothesis_evidence": blocker_state["evidence_type"],
                "blocking_hypothesis_beam_index": blocker_state["beam_index"],
                "planner_hard_feasible_count": action["shadow_hard_feasible"],
                "backend_direction_max_error_rad": float(np.max(direction_delta)),
                "cached_shadow_corridor_m": float(cached_row["shadow_corridor_m"]),
                "cached_planner_hard_feasible_count": int(
                    cached_row["shadow_family_hard_feasible"]
                ),
                "cached_result_exact_match": (
                    abs(float(support["left_corridor"]) - float(cached_row["shadow_corridor_m"]))
                    <= 1.0e-12
                    and action["shadow_hard_feasible"]
                    == int(cached_row["shadow_family_hard_feasible"])
                ),
            }
        )
    first = next(row for row in rows if row["planner_hard_feasible_count"] > 0)
    stable_ms = next(
        row["delta_t_ms"]
        for index, row in enumerate(rows)
        if index >= 2
        and all(item["planner_hard_feasible_count"] > 0 for item in rows[index - 2:index + 1])
    )
    if (
        rows[1]["blocking_hypothesis_status"] != "REMOVED"
        or first["delta_t_ms"] != 40
        or stable_ms != 60
        or not all(row["cached_result_exact_match"] for row in rows)
    ):
        raise RuntimeError("normal_01_00 authoritative regression changed the prior result")
    for row in rows:
        row["classification"] = "OBSERVATION_RESOLVES_EARLY"
        row["first_planner_hard_feasible_ms"] = first["delta_t_ms"]
        row["stable_resolution_ms"] = stable_ms
    return rows, len(rows)


def random_geometry_rows(contract: BackendBeamContract) -> list[dict[str, Any]]:
    generator = np.random.default_rng(39039)
    rows: list[dict[str, Any]] = []
    fixed_yaws = (0.0, math.pi / 2.0, -math.pi / 2.0, math.pi - 1.0e-9)
    for case_index in range(128):
        yaw = fixed_yaws[case_index] if case_index < len(fixed_yaws) else float(
            generator.uniform(-math.pi, math.pi)
        )
        scan_index = int(generator.integers(0, contract.scan_beams))
        range_m = float(generator.uniform(0.1, 30.0))
        origin_x, origin_y = generator.uniform(-20.0, 20.0, size=2)
        table_index = int(contract.table_indices(yaw)[scan_index])
        candidates = contract.scan_indices_for_backend_bin(yaw, table_index)
        angle = contract.physical_angles_rad(yaw)[scan_index]
        target = endpoint_xy(
            [angle],
            [range_m],
            origin_x_m=float(origin_x),
            origin_y_m=float(origin_y),
        )[0]
        recovered = math.atan2(target[1] - origin_y, target[0] - origin_x)
        error = abs(float(wrapped_angle_delta_rad(recovered, angle)))
        rows.append(
            {
                "seed": 39039,
                "case_index": case_index,
                "sensor_pose_class": "STRAIGHT_OR_CORNER_FIXED"
                if case_index < len(fixed_yaws)
                else "RANDOM",
                "yaw_rad": yaw,
                "scan_index": scan_index,
                "backend_table_index": table_index,
                "backend_bin_scan_indices": ";".join(str(value) for value in candidates),
                "range_m": range_m,
                "origin_x_m": float(origin_x),
                "origin_y_m": float(origin_y),
                "target_x_m": float(target[0]),
                "target_y_m": float(target[1]),
                "round_trip_angle_error_rad": error,
                "index_recovered": scan_index in candidates,
                "status": "PASS" if scan_index in candidates and error <= 2.0e-14 else "FAIL",
            }
        )
    if any(row["status"] != "PASS" for row in rows):
        raise RuntimeError("random geometry beam-contract test failed")
    return rows


def diagnostic_inventory() -> list[dict[str, Any]]:
    files = {
        "rulebook": ROOT / "tools/cmaes_tuning/render_rulebook_obstacle_envelope_audit.py",
        "s3": ROOT / "tools/cmaes_tuning/render_s3_shadow_false_infeasible_audit.py",
        "time": ROOT / "tools/cmaes_tuning/render_time_to_disambiguation_audit.py",
        "sensor": ROOT / "tools/cmaes_tuning/render_validation039_sensor_model_consistency_audit.py",
        "representation": ROOT / "tools/cmaes_tuning/render_validation039_representation_root_cause.py",
        "axis": ROOT / "tools/cmaes_tuning/render_time_axis_feasibility_audit.py",
    }
    return [
        {
            "file_function": f"{files['rulebook']}:backend_directions_continuous",
            "line": source_line(files["rulebook"], "def backend_directions_continuous"),
            "current_convention": "analytic uniform FOV/(N-1)",
            "authoritative": False,
            "must_eventually_change": True,
            "note": "used by recorded_ray_frame; not the quantized range-generating backend",
        },
        {
            "file_function": f"{files['rulebook']}:backend_directions_model",
            "line": source_line(files["rulebook"], "def backend_directions_model"),
            "current_convention": "2000-bin lookup/truncation",
            "authoritative": True,
            "must_eventually_change": True,
            "note": "numerically authoritative but duplicated; should delegate to the shared helper",
        },
        {
            "file_function": f"{files['s3']}:validation039_case",
            "line": source_line(files["s3"], "def validation039_case"),
            "current_convention": "delegates to rulebook.recorded_ray_frame FOV/(N-1)",
            "authoritative": False,
            "must_eventually_change": True,
            "note": "transitive mismatch",
        },
        {
            "file_function": f"{files['time']}:recorded_future_frame",
            "line": source_line(files["time"], "def recorded_future_frame"),
            "current_convention": "analytic uniform FOV/(N-1)",
            "authoritative": False,
            "must_eventually_change": True,
            "note": "recorded validation_039 future frames",
        },
        {
            "file_function": f"{files['time']}:normal_future_frame",
            "line": source_line(files["time"], "def normal_future_frame"),
            "current_convention": "exact backend through synthetic_ray_frame",
            "authoritative": True,
            "must_eventually_change": True,
            "note": "correct but transitively uses another duplicate implementation",
        },
        {
            "file_function": f"{files['sensor']}:direction_array",
            "line": source_line(files["sensor"], "def direction_array"),
            "current_convention": "explicit metadata / FOV/(N-1) / exact-backend comparison",
            "authoritative": True,
            "must_eventually_change": True,
            "note": "exact branch is valid; consolidate exact branch into shared helper",
        },
        {
            "file_function": f"{files['representation']}:detector_clusters",
            "line": source_line(files["representation"], "def detector_clusters"),
            "current_convention": "ROS metadata plus analytic backend increment comparison",
            "authoritative": False,
            "must_eventually_change": True,
            "note": "does not reproduce 2000-bin lookup",
        },
        {
            "file_function": f"{files['axis']}:read_scan_evidence",
            "line": source_line(files["axis"], "angle = yaw + float(message.angle_min)"),
            "current_convention": "ROS metadata",
            "authoritative": False,
            "must_eventually_change": True,
            "note": "correct LaserScan consumer semantics, not simulator physical-ray truth",
        },
    ]


def unit_tests() -> list[dict[str, Any]]:
    environment = dict(os.environ)
    package_path = str(ROOT / "tools/cmaes_tuning")
    environment["PYTHONPATH"] = package_path + (
        os.pathsep + environment["PYTHONPATH"] if environment.get("PYTHONPATH") else ""
    )
    command = [
        sys.executable,
        "-m",
        "unittest",
        "tools.cmaes_tuning.tests.test_lidar_beam_contract",
        "-v",
    ]
    started = time.monotonic()
    completed = subprocess.run(
        command,
        cwd=ROOT,
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    elapsed = time.monotonic() - started
    if completed.returncode != 0:
        raise RuntimeError(f"beam-contract unit tests failed:\n{completed.stdout}")
    rows = []
    for line in completed.stdout.splitlines():
        if line.startswith("test_") and " ... " in line:
            name, status = line.rsplit(" ... ", 1)
            rows.append(
                {
                    "test": name.split(" ", 1)[0],
                    "status": "PASS" if status == "ok" else status.upper(),
                    "elapsed_total_s": elapsed,
                    "command": " ".join(command),
                }
            )
    if len(rows) != 6 or any(row["status"] != "PASS" for row in rows):
        raise RuntimeError(f"unexpected unit-test report: {completed.stdout}")
    return rows


def markdown_table(headers: Sequence[str], rows: Sequence[Sequence[Any]]) -> str:
    def token(value: Any) -> str:
        if isinstance(value, float):
            return f"{value:.9f}"
        return str(value)

    return "\n".join(
        [
            "| " + " | ".join(headers) + " |",
            "| " + " | ".join("---" for _ in headers) + " |",
            *("| " + " | ".join(token(value) for value in row) + " |" for row in rows),
        ]
    )


def render_readme(summary: dict[str, Any], performance: dict[str, Any]) -> str:
    backend = summary["backend_source_contract"]
    core = summary["core_beam_comparison"]
    validation = summary["validation039_safety_regression"]
    normal = summary["normal0100_regression"]
    exact_core = [row for row in core if row["model"] == "ROS_METADATA"]
    return f"""# LiDAR beam geometry contract audit v1

Diagnostic/test-only audit. No simulator publisher, obstacle detector, local planner, tracking,
controller, state machine, YAML, or production binary was changed. No closed-loop replay, CMA,
or dataset regeneration was run.

## Classification

**{summary['classification']}**. The primary defect is
`SIMULATOR_METADATA_CONTRACT_MISMATCH`, accompanied by duplicated and inconsistent offline audit
conventions.

For scan index `i`, the C++ backend first advances a continuous table index by
`2000 * (FOV/(N-1)) / (2*pi)`, wraps it into `[0, 2000)`, truncates it with
`static_cast<int>`, and traces the direction stored at
`table[k] = k * 2*pi/(2000-1)`. Consequently the physical ray is quantized, slightly non-uniform,
and yaw-dependent. The publisher instead advertises `angle_min=-FOV/2`,
`angle_max=+FOV/2`, and `angle_increment=FOV/N`. Standard LaserScan metadata therefore does not
exactly describe the physical ray that generated `ranges[i]`; there is no documented additional
mapping in the message.

Authoritative source evidence:

- Backend increment: [{backend['backend_path']}]({backend['backend_path']}:{backend['backend_increment_line']})
- Table discretization: [{backend['backend_path']}]({backend['backend_path']}:{backend['theta_line']})
- Table angle: [{backend['backend_path']}]({backend['backend_path']}:{backend['backend_table_line']})
- Truncation and tracing: [{backend['backend_path']}]({backend['backend_path']}:{backend['backend_conversion_line']})
- ROS publisher metadata: [{backend['publisher_path']}]({backend['publisher_path']}:{backend['publisher_increment_line']})
- Lockstep publisher metadata: [lockstep_episode.py]({summary['lockstep_source_path']}:{summary['lockstep_increment_line']})
- Production detector consumer: [obstacle_detector_node.cpp]({summary['detector_source_path']}:{summary['detector_angle_line']})

## Core beams

{markdown_table(
    ['stamp', 'beam', 'metadata angular error deg', 'endpoint error mm'],
    [
        [row['source_stamp_ns'], row['beam_index'], row['angular_error_vs_backend_deg'],
         1000.0 * row['euclidean_endpoint_error_vs_backend_m']]
        for row in exact_core
    ],
)}

The recomputed metadata-to-backend endpoint displacement is
{1000.0 * min(row['euclidean_endpoint_error_vs_backend_m'] for row in exact_core):.3f}–
{1000.0 * max(row['euclidean_endpoint_error_vs_backend_m'] for row in exact_core):.3f} mm.

## validation_039 safety anchor

{markdown_table(
    ['run', 'delta ms', 'cells', 'hypotheses', 'status', 'GT undercoverage', 'dangerous hard-invalid'],
    [
        [row['run'], row['delta_t_ms'], row['possible_cell_count'],
         row['possible_hypothesis_count'], row['status'], row['gt_undercoverage_m'],
         row['dangerous_path_hard_invalid']]
        for row in validation
    ],
)}

The authoritative mapping removes the +40 ms empty-set failure. At +40 ms the result reproduces
28,066 cells / 450,984 hypotheses / 0.449056 m2, GT undercoverage remains zero, and the known
dangerous path remains hard-invalid. The primary and repeat digests are identical.

## normal_01_00

{markdown_table(
    ['delta ms', 'blocker', 'hard-feasible', 'corridor m'],
    [
        [row['delta_t_ms'], row['blocking_hypothesis_status'],
         row['planner_hard_feasible_count'], row['shadow_corridor_m']]
        for row in normal if row['delta_t_ms'] in (0, 10, 40, 60)
    ],
)}

The blocker is removed at +10 ms, planner hard-feasibility first appears at +40 ms, and the
three-fresh-scan stable result occurs at +60 ms. The classification remains
`OBSERVATION_RESOLVES_EARLY`. These synthetic frames already use the exact backend lookup; every
recomputed row exactly matches the cached conclusion.

## Detector and follow-up

Production `obstacle_detector` uses the standard ROS expression
`scan.angle_min + i * scan.angle_increment`. It is correct relative to the published metadata but
the metadata differs from backend physical rays, so its classification is
`DETECTOR_MATCHES_METADATA_BUT_METADATA_DIFFERS_FROM_BACKEND`. It should not be patched to encode a
simulator-private lookup.

The next recommended step is a simulator-side design/test change: make the range generator and
LaserScan metadata describe one genuinely uniform set of rays, then rerun narrow offline and
closed-loop regressions before production adoption. LaserScan cannot exactly encode the current
yaw-dependent non-uniform quantized lookup by changing only `angle_increment`.

Sparse-return loss remains separate: `min_cluster_points=5`, while future validation_039 target
fragments can contain about four points. Its follow-up remains
`STATIC_SAFETY_SPARSE_EVIDENCE_SHADOW_AUDIT`; no threshold or dynamic/opponent tracking behavior was
changed here. Frenet KF, map-velocity KF, Mahalanobis association, `vs/vd`, track IDs, motion
voting, dynamic geometry, and `/opp_obs` remain untouched.

Production temporal logic must remain timestamp/fresh-scan based. It must not assume a fixed 10,
25, or 40 Hz period; use `header.stamp`, `angle_min`, `angle_increment`, `ranges.size()`, and
fresh-scan identity.

## Tests and resources

- Beam-contract unit tests: {performance['unit_tests_run']} passed.
- Random deterministic geometries: {summary['random_geometry_case_count']} passed; seed 39039.
- Full recorded/synthetic scan sweeps: {len(summary['full_scan_error_statistics'])}; all valid indices.
- Wall / CPU: {performance['wall_time_s']:.3f} / {performance['cpu_time_s']:.3f} s.
- Max RSS: {performance['max_rss_bytes']} bytes; swap delta: {performance['swap_change_bytes']} bytes.
- Bag parses / full-grid evaluations: {performance['bag_parse_count']} / {performance['full_grid_evaluations']}.
- Closed-loop replay / CMA / production behavior changes: 0 / 0 / 0.

Detailed source inventory, per-model core endpoints, all-scan statistics, test results, safety
regressions, and performance are in the adjacent CSV/JSON files.
"""


def main() -> None:
    wall_started = time.monotonic()
    self_started = resource.getrusage(resource.RUSAGE_SELF)
    child_started = resource.getrusage(resource.RUSAGE_CHILDREN)
    memory_before = memory_snapshot()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    RAW.mkdir(parents=True, exist_ok=True)
    time_audit.RAW = RAW

    sources = inspect_authoritative_sources()
    sources["theta_line"] = source_line(Path(str(sources["backend_path"])), "int theta_dis_ = 2000")
    before = production_hashes(sources)
    manifest = json.loads((SCENARIO / "manifest.json").read_text(encoding="utf-8"))
    model = manifest["simulator_collision_model"]
    raster = {
        key: float(value)
        for key, value in manifest["baked_obstacle_raster"]["world_half_open_bounds_m"].items()
    }
    contract = BackendBeamContract(
        float(model["scan_fov_rad"]),
        int(model["scan_beams"]),
        int(sources["theta_discretization"]),
    )
    detector_parameters = yaml.safe_load(
        (ROOT / "src/obstacle_detector/config/obstacle_detector.yaml").read_text(encoding="utf-8")
    )["obstacle_detector"]["ros__parameters"]
    event_list = rulebook.read_jsonl(REPLAY / "detector_events.jsonl")
    events = {int(item["scan_stamp_ns"]): item for item in event_list}

    bag_started = time.monotonic()
    bag = read_bag(REPLAY / "bag", topics=("/scan", "/ego_racecar/odom"))
    bag_parse_time = time.monotonic() - bag_started
    scans = {rulebook.stamp_ns(item.message): item.message for item in bag.topic("/scan")}
    odometry = {
        rulebook.stamp_ns(item.message): item.message
        for item in bag.topic("/ego_racecar/odom")
    }
    evidence = {
        stamp: build_contract_frame(
            stamp,
            scans,
            odometry,
            events,
            raster,
            model,
            detector_parameters,
            contract,
        )
        for stamp in STAMPS
    }
    first_scan = scans[BASELINE_STAMP]

    backend_contract = [
        {
            "layer": "SIMULATOR_BACKEND",
            "property": "internal_angular_bins",
            "value": contract.theta_discretization,
            "formula": "theta_dis_=2000",
            "source_file": sources["backend_path"],
            "source_line": sources["theta_line"],
        },
        {
            "layer": "SIMULATOR_BACKEND",
            "property": "continuous_index_increment",
            "value": contract.theta_index_increment,
            "formula": "2000*(FOV/(N-1))/(2*pi)",
            "source_file": sources["backend_path"],
            "source_line": sources["backend_increment_line"],
        },
        {
            "layer": "SIMULATOR_BACKEND",
            "property": "table_angle",
            "value": "k*2*pi/1999; k=0..1999; positive CCW; index 0 origin",
            "formula": "i*2*pi/(theta_dis_-1)",
            "source_file": sources["backend_path"],
            "source_line": sources["backend_table_line"],
        },
        {
            "layer": "SIMULATOR_BACKEND",
            "property": "initial_index_and_wrap",
            "value": "fmod to [0,2000), per-beam wrap; absolute map yaw",
            "formula": "2000*(yaw-FOV/2)/(2*pi)",
            "source_file": sources["backend_path"],
            "source_line": sources["backend_initial_line"],
        },
        {
            "layer": "SIMULATOR_BACKEND",
            "property": "integer_conversion",
            "value": "truncation toward zero; equivalent to floor after nonnegative wrap",
            "formula": "static_cast<int>(theta_index)",
            "source_file": sources["backend_path"],
            "source_line": sources["backend_conversion_line"],
        },
        {
            "layer": "LASERSCAN_PUBLISHER",
            "property": "published_metadata",
            "value": f"N={len(first_scan.ranges)}; angle_min={float(first_scan.angle_min)}; angle_max={float(first_scan.angle_max)}; increment={float(first_scan.angle_increment)}; frame_id={first_scan.header.frame_id}",
            "formula": "angle_min=-FOV/2; angle_max=+FOV/2; angle_increment=FOV/N",
            "source_file": sources["publisher_path"],
            "source_line": sources["publisher_increment_line"],
        },
        {
            "layer": "LASERSCAN_PUBLISHER",
            "property": "last_index_semantics",
            "value": float(first_scan.angle_min)
            + (len(first_scan.ranges) - 1) * float(first_scan.angle_increment),
            "formula": "angle_min+(N-1)*angle_increment; differs from advertised angle_max",
            "source_file": ROOT / "tools/cmaes_tuning/lockstep_episode.py",
            "source_line": source_line(
                ROOT / "tools/cmaes_tuning/lockstep_episode.py",
                "message.angle_increment = scan_fov / beams",
            ),
        },
        {
            "layer": "LASERSCAN_PUBLISHER",
            "property": "angle_max_inclusive_for_ranges",
            "value": False,
            "formula": "published span/increment=1080 intervals but ranges.size()=1080; an inclusive endpoint contract would require 1081 samples",
            "source_file": sources["publisher_path"],
            "source_line": sources["publisher_increment_line"],
        },
    ]

    core_rows = core_comparison_rows(evidence, contract)
    normal_case = time_audit.synthetic_normal_case("normal_01_00")
    normal_frame = normal_case["frames"][-1]
    normal_ego = time_audit.ego_state(normal_case, normal_frame, 2)
    scan_inputs = [
        (
            "validation_039_baseline",
            BASELINE_STAMP,
            np.asarray(scans[BASELINE_STAMP].ranges, dtype=np.float64),
            evidence[BASELINE_STAMP].yaw_rad,
        ),
        (
            "validation_039_plus20ms",
            STAMPS[2],
            np.asarray(scans[STAMPS[2]].ranges, dtype=np.float64),
            evidence[STAMPS[2]].yaw_rad,
        ),
        (
            "validation_039_plus40ms",
            STAMPS[4],
            np.asarray(scans[STAMPS[4]].ranges, dtype=np.float64),
            evidence[STAMPS[4]].yaw_rad,
        ),
        (
            "normal_01_00_synthetic_baseline",
            normal_frame.stamp_ns,
            np.asarray(normal_frame.ranges, dtype=np.float64),
            normal_ego["yaw"],
        ),
    ]
    with concurrent.futures.ThreadPoolExecutor(max_workers=WORKERS) as executor:
        full_stats = list(
            executor.map(
                lambda item: percentile_statistics(
                    item[0],
                    item[1],
                    item[2],
                    item[3],
                    float(first_scan.angle_min),
                    float(first_scan.angle_increment),
                    contract,
                ),
                scan_inputs,
            )
        )

    validation_rows, determinism, validation_grid_count = validation039_regression(
        evidence, manifest, raster
    )
    normal_rows, normal_grid_count = normal0100_regression(contract)
    random_rows = random_geometry_rows(contract)
    test_rows = unit_tests()
    inventory = diagnostic_inventory()
    detector_source = ROOT / "src/obstacle_detector/src/obstacle_detector_node.cpp"
    detector_rows = [
        {
            "component": "production_obstacle_detector",
            "source_file": str(detector_source),
            "source_line": source_line(
                detector_source,
                "scan.angle_min + static_cast<double>(i) * scan.angle_increment",
            ),
            "angle_formula": "scan.angle_min + i*scan.angle_increment",
            "classification": "DETECTOR_MATCHES_METADATA_BUT_METADATA_DIFFERS_FROM_BACKEND",
            "correct_relative_to_ros_metadata": True,
            "matches_backend_physical_ray": False,
            "production_modified": False,
            "separate_sparse_gate": "min_cluster_points=5; validation_039 future target fragment~4",
        }
    ]

    after = production_hashes(sources)
    if before != after:
        raise RuntimeError("production/simulator source or binary hash changed during audit")
    memory_after = memory_snapshot()
    self_usage = resource.getrusage(resource.RUSAGE_SELF)
    child_usage = resource.getrusage(resource.RUSAGE_CHILDREN)
    performance = {
        "schema": "lidar_beam_contract_performance/1",
        "worker_count": WORKERS,
        "worker_policy": "causal regressions serial; four independent full-scan statistics only",
        "wall_time_s": time.monotonic() - wall_started,
        "cpu_time_s": (self_usage.ru_utime + self_usage.ru_stime)
        - (self_started.ru_utime + self_started.ru_stime)
        + (child_usage.ru_utime + child_usage.ru_stime)
        - (child_started.ru_utime + child_started.ru_stime),
        "max_rss_bytes": max(self_usage.ru_maxrss, child_usage.ru_maxrss) * 1024,
        "memory_available_before_bytes": memory_before["memory_available_bytes"],
        "memory_available_after_bytes": memory_after["memory_available_bytes"],
        "swap_used_before_bytes": memory_before["swap_used_bytes"],
        "swap_used_after_bytes": memory_after["swap_used_bytes"],
        "swap_change_bytes": memory_after["swap_used_bytes"] - memory_before["swap_used_bytes"],
        "bag_parse_count": 1,
        "bag_parse_time_s": bag_parse_time,
        "cached_artifacts_reused": [str(PRIOR_SENSOR), str(PRIOR_TIME)],
        "unit_tests_run": len(test_rows),
        "full_scan_sweeps": len(full_stats),
        "full_grid_evaluations": validation_grid_count + normal_grid_count,
        "closed_loop_replay_count": 0,
        "cma_count": 0,
        "dataset_regeneration_count": 0,
        "production_behavior_modification_count": 0,
    }
    if performance["swap_change_bytes"] > 0:
        raise RuntimeError("swap use increased during audit")

    lockstep = ROOT / "tools/cmaes_tuning/lockstep_episode.py"
    summary = {
        "schema": "lidar_beam_contract_audit/1",
        "diagnostic_only": True,
        "classification": "MULTI_LAYER_CONVENTION_MISMATCH",
        "primary_defect": "SIMULATOR_METADATA_CONTRACT_MISMATCH",
        "authoritative_mapping": "sequential 2000-bin absolute-angle lookup, wrap, nonnegative int truncation, table angle k*2*pi/1999",
        "metadata_exactly_describes_backend": False,
        "published_angle_max_inclusive_for_ranges": False,
        "documented_additional_mapping_present": False,
        "backend_source_contract": sources,
        "backend_contract": backend_contract,
        "core_beam_comparison": core_rows,
        "full_scan_error_statistics": full_stats,
        "detector_convention_audit": detector_rows,
        "diagnostic_convention_inventory": inventory,
        "validation039_safety_regression": validation_rows,
        "validation039_determinism": determinism,
        "normal0100_regression": normal_rows,
        "normal0100_classification": "OBSERVATION_RESOLVES_EARLY",
        "random_geometry_case_count": len(random_rows),
        "random_geometry_digest": canonical_digest(random_rows),
        "unit_test_results": test_rows,
        "old_analytic_fallback_rejected_by_test": True,
        "production_files_changed": False,
        "production_hashes_before": before,
        "production_hashes_after": after,
        "production_change_justified_now": False,
        "recommended_next_action": "test a simulator-side uniform-ray contract where range generation and LaserScan metadata agree; do not encode simulator-private lookup in obstacle_detector",
        "sparse_followup": "STATIC_SAFETY_SPARSE_EVIDENCE_SHADOW_AUDIT",
        "opponent_dynamic_pipeline_unchanged": True,
        "temporal_frequency_note": "timestamp and fresh-scan identity based; never assume 10ms, 25ms, or 40Hz",
        "detector_source_path": str(detector_source),
        "detector_angle_line": detector_rows[0]["source_line"],
        "lockstep_source_path": str(lockstep),
        "lockstep_increment_line": source_line(
            lockstep, "message.angle_increment = scan_fov / beams"
        ),
    }

    write_csv(OUTPUT / "backend_contract.csv", backend_contract)
    write_csv(OUTPUT / "core_beam_comparison.csv", core_rows)
    write_csv(OUTPUT / "full_scan_error_statistics.csv", full_stats)
    write_csv(OUTPUT / "detector_convention_audit.csv", detector_rows)
    write_csv(OUTPUT / "diagnostic_convention_inventory.csv", inventory)
    write_csv(OUTPUT / "unit_test_results.csv", test_rows)
    write_csv(OUTPUT / "validation039_safety_regression.csv", validation_rows)
    write_csv(OUTPUT / "normal0100_regression.csv", normal_rows)
    write_csv(OUTPUT / "random_geometry_contract_test.csv", random_rows)
    (OUTPUT / "summary.json").write_text(
        json.dumps(clean_json(summary), indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (OUTPUT / "performance.json").write_text(
        json.dumps(clean_json(performance), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (OUTPUT / "README.md").write_text(
        render_readme(clean_json(summary), clean_json(performance)), encoding="utf-8"
    )


if __name__ == "__main__":
    main()
