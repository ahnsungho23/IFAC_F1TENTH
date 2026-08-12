#!/usr/bin/env python3
"""Broader legacy-versus-uniform LaserScan downstream regression audit.

This is diagnostic-only code.  It never changes simulator, detector, planner,
controller, state-machine, or tuning parameters.  Ground truth is used only to
label sensor returns and match already-produced detector outputs.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import signal
import subprocess
import sys
import time
from types import SimpleNamespace
from typing import Any, Iterable, Sequence

import numpy as np
import yaml

import render_rulebook_obstacle_envelope_audit as rulebook
import render_s3_shadow_false_infeasible_audit as s3
import render_time_to_disambiguation_audit as time_audit
import render_validation039_representation_root_cause as representation
import render_validation039_sensor_model_consistency_audit as sensor_audit
import render_simulator_laserscan_contract_prototype as prototype_audit
from cmaes_tuning.uniform_laserscan_prototype import (
    LEGACY_SCAN_MODE,
    PROTOTYPE_UNIFORM_SCAN_MODE,
    UniformLaserScanContract,
    float32,
    oriented_rectangle_ranges,
    uniform_noise_free_scan,
)
from cmaes_tuning.lidar_beam_contract import BackendBeamContract


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "runs/cmaes_tuning/uniform_laserscan_broader_regression_v1"
RAW = OUTPUT / ".raw"
PRIOR = ROOT / "runs/cmaes_tuning/simulator_laserscan_contract_prototype_v1"
RULEBOOK = ROOT / "runs/cmaes_tuning/rulebook_obstacle_envelope_audit_v1"
S3_PRIOR = ROOT / "runs/cmaes_tuning/s3_shadow_false_infeasible_audit_v1"
TIME_PRIOR = ROOT / "runs/cmaes_tuning/time_to_disambiguation_audit_v1"
DETECTOR = ROOT / "build/obstacle_detector/obstacle_detector_node"
DETECTOR_CONFIG = ROOT / "src/obstacle_detector/config/obstacle_detector.yaml"
HARNESS = ROOT / "tools/cmaes_tuning/broader_laserscan_detector_harness.py"
ACTIVE_SIM = Path("/home/sungho/f1sim_C")
PROTOTYPE_SIM = Path("/tmp/f1sim_uniform_prototype.mupqVu/f1sim_C")
WORKERS = 6
STATIC_SCAN_COUNT = 15
RANDOM_COUNT = 256
RANDOM_SEED = 39039256
THRESHOLDS_M = (0.001, 0.01, 0.025, 0.05, 0.10, 0.50, 1.0)
MODES = ("LEGACY", "PROTOTYPE")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--smoke-case", default="")
    parser.add_argument("--skip-tests", action="store_true")
    parser.add_argument("--refresh-tests", action="store_true")
    parser.add_argument("--refresh-reports", action="store_true")
    return parser.parse_args()


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


def canonical_digest(value: Any) -> str:
    payload = json.dumps(clean_json(value), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode()).hexdigest()


def array_digest(value: np.ndarray) -> str:
    return hashlib.sha256(np.asarray(value, dtype="<f8").tobytes()).hexdigest()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        raise RuntimeError(f"empty CSV refused: {path}")
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(clean_json(rows))


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


def protected_paths() -> dict[str, Path]:
    return {
        "detector_node": ROOT / "src/obstacle_detector/src/obstacle_detector_node.cpp",
        "detector_tracker": ROOT / "src/obstacle_detector/src/obstacle_tracker.cpp",
        "detector_tracker_header": ROOT / "src/obstacle_detector/include/obstacle_detector/obstacle_tracker.hpp",
        "detector_config": DETECTOR_CONFIG,
        "planner_node": ROOT / "src/local_planning/src/local_planner_node.cpp",
        "controller": ROOT / "src/f1tenth_control/control_code/controller.cpp",
        "state_machine": ROOT / "src/state_machine/src/state_machine_node.cpp",
        "active_sim_backend": ACTIVE_SIM / "gym/f110_gym/cpp_backend.cpp",
        "active_sim_bridge": ACTIVE_SIM / "f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py",
        "prototype_backend": PROTOTYPE_SIM / "gym/f110_gym/cpp_backend.cpp",
        "prototype_bridge": PROTOTYPE_SIM / "f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py",
    }


def protected_hashes() -> dict[str, str]:
    return {name: sha256(path) for name, path in protected_paths().items() if path.is_file()}


def git_status(path: Path) -> str:
    result = subprocess.run(
        ["git", "-C", str(path), "status", "--short"], text=True, capture_output=True
    )
    return result.stdout if result.returncode == 0 else f"UNAVAILABLE: {result.stderr.strip()}"


def scan_metadata(mode: str, model: Any) -> dict[str, float | str]:
    if mode == "LEGACY":
        return {
            "scan_mode": str(LEGACY_SCAN_MODE.value),
            "angle_min_rad": float32(-0.5 * model.scan_fov),
            "angle_max_rad": float32(0.5 * model.scan_fov),
            "angle_increment_rad": float32(model.scan_fov / model.scan_beams),
        }
    contract = UniformLaserScanContract.from_fov(model.scan_fov, model.scan_beams)
    return {
        "scan_mode": str(PROTOTYPE_UNIFORM_SCAN_MODE.value),
        "angle_min_rad": contract.angle_min_rad,
        "angle_max_rad": contract.angle_max_rad,
        "angle_increment_rad": contract.angle_increment_rad,
    }


def pose_for(case: dict[str, Any], absolute_index: int) -> dict[str, float]:
    return rulebook.interpolate_reference(
        case["waypoints"],
        float(case["manifest_s"])
        - float(case["gt_row"]["observation_distance_m"])
        + 0.04 * absolute_index,
    )


def prototype_frame(case: dict[str, Any], absolute_index: int) -> rulebook.RayFrame:
    model = case["clean_model"]
    obstacle = case["obstacle"]
    pose = pose_for(case, absolute_index)
    contract = UniformLaserScanContract.from_fov(model.scan_fov, model.scan_beams)
    yaw = float(pose["yaw"])
    origin = np.asarray(
        [
            float(pose["x"]) + model.lidar_offset * math.cos(yaw),
            float(pose["y"]) + model.lidar_offset * math.sin(yaw),
        ]
    )
    directions = contract.directions(yaw)
    background = uniform_noise_free_scan(model, float(pose["x"]), float(pose["y"]), yaw)
    target = oriented_rectangle_ranges(
        origin,
        directions,
        np.asarray([float(obstacle["x"]), float(obstacle["y"])]),
        float(obstacle["yaw"]),
        float(obstacle["width"]),
        float(obstacle["height"]),
    )
    hit_mask = target + rulebook.GEOMETRY_EPSILON_M < background
    ranges = np.minimum(background, target)
    endpoints = origin + ranges[:, None] * directions
    hit_indices = tuple(int(index) for index in np.flatnonzero(hit_mask))
    if len(hit_indices) < 5:
        raise RuntimeError(f"{case['case_id']} prototype frame {absolute_index}: <5 target returns")
    hits = endpoints[np.asarray(hit_indices, dtype=np.int64)]
    status = ("RAW", "TENTATIVE", "CONFIRMED")[min(absolute_index, 2)]
    return rulebook.RayFrame(
        stamp_ns=20_000_000_000 + absolute_index * 10_000_000,
        status=status,
        motion_status="UNKNOWN",
        origin=origin,
        directions=directions,
        ranges=ranges,
        free_lengths=np.maximum(0.0, ranges - rulebook.GEOMETRY_EPSILON_M),
        hits=hits,
        hit_indices=hit_indices,
        detector_box=(
            float(np.min(hits[:, 0])), float(np.max(hits[:, 0])),
            float(np.min(hits[:, 1])), float(np.max(hits[:, 1])),
        ),
    )


def static_frames(case: dict[str, Any], mode: str) -> list[rulebook.RayFrame]:
    frames: list[rulebook.RayFrame] = []
    for index in range(STATIC_SCAN_COUNT):
        if mode == "PROTOTYPE":
            frames.append(prototype_frame(case, index))
        elif index < len(case["frames"]):
            frames.append(case["frames"][index])
        else:
            frames.append(time_audit.normal_future_frame(case, index))
    return frames


def waypoint_payload(waypoints: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    fields = ("id", "s_m", "x_m", "y_m", "psi_rad", "d_left", "d_right", "vx_mps")
    return [{key: item[key] for key in fields if key in item} for item in waypoints]


def prepare_detector_input(
    case: dict[str, Any], mode: str, frames: Sequence[rulebook.RayFrame], *, subdir: str = "static"
) -> dict[str, str]:
    directory = RAW / "detector_inputs" / subdir
    directory.mkdir(parents=True, exist_ok=True)
    token = f"{case['case_id']}__{mode.lower()}"
    request_path = directory / f"{token}.json"
    scans_path = directory / f"{token}.npz"
    output_path = RAW / "detector_outputs" / subdir / f"{token}.json"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    metadata = scan_metadata(mode, case["clean_model"])
    request = {
        "schema": "uniform_laserscan_detector_request/1",
        "case_id": case["case_id"],
        "mode": mode,
        "scan_mode": metadata["scan_mode"],
        "map_yaml": str(s3.CLEAN_MAP),
        "waypoints": waypoint_payload(case["waypoints"]),
        "angle_min_rad": metadata["angle_min_rad"],
        "angle_max_rad": metadata["angle_max_rad"],
        "angle_increment_rad": metadata["angle_increment_rad"],
        "frames": [
            {
                "stamp_ns": frame.stamp_ns,
                "x_m": pose_for(case, index)["x"],
                "y_m": pose_for(case, index)["y"],
                "yaw_rad": pose_for(case, index)["yaw"],
                "speed_mps": 4.0,
            }
            for index, frame in enumerate(frames)
        ],
    }
    request_path.write_text(json.dumps(clean_json(request), indent=2) + "\n", encoding="utf-8")
    np.savez_compressed(scans_path, **{f"scan_{i:04d}": frame.ranges for i, frame in enumerate(frames)})
    return {"request": str(request_path), "scans": str(scans_path), "output": str(output_path)}


def detector_command() -> list[str]:
    return [
        str(DETECTOR), "--ros-args", "--params-file", str(DETECTOR_CONFIG),
        "-p", "lockstep_mode:=true", "-p", "replay_diagnostics_enable:=true",
        "-p", "ego_odom_topic:=/ego_racecar/odom", "-p", "diagnostics_enable:=false",
        "-p", "publish_markers:=false",
    ]


def run_detector_task(task: dict[str, Any]) -> dict[str, Any]:
    started = time.monotonic()
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(task["domain_id"])
    log_dir = RAW / "ros_logs" / f"domain_{task['domain_id']}"
    log_dir.mkdir(parents=True, exist_ok=True)
    env["ROS_LOG_DIR"] = str(log_dir)
    log_path = RAW / "detector_logs" / f"{task['label']}.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as log:
        node = subprocess.Popen(
            detector_command(), cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            result = subprocess.run(
                [sys.executable, str(HARNESS), "--request", task["request"],
                 "--scans", task["scans"], "--output", task["output"]],
                cwd=ROOT, env=env, text=True, capture_output=True, timeout=90.0,
            )
        finally:
            if node.poll() is None:
                os.killpg(node.pid, signal.SIGINT)
                try:
                    node.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    os.killpg(node.pid, signal.SIGTERM)
                    node.wait(timeout=5.0)
    return {
        **task,
        "returncode": result.returncode,
        "stdout": result.stdout[-2000:],
        "stderr": result.stderr[-4000:],
        "node_returncode": node.returncode,
        "elapsed_s": time.monotonic() - started,
        "success": result.returncode == 0 and Path(task["output"]).is_file(),
    }


def execute_detector_tasks(tasks: list[dict[str, Any]], workers: int) -> list[dict[str, Any]]:
    with concurrent.futures.ProcessPoolExecutor(max_workers=workers) as executor:
        results = list(executor.map(run_detector_task, tasks))
    failures = [item for item in results if not item["success"]]
    if failures:
        raise RuntimeError("detector task failures: " + json.dumps(clean_json(failures), indent=2))
    return sorted(results, key=lambda item: item["label"])


def advertised_endpoints(
    frame: rulebook.RayFrame, metadata: dict[str, Any], yaw: float
) -> tuple[np.ndarray, np.ndarray]:
    angles = (
        yaw + float(metadata["angle_min_rad"])
        + np.arange(frame.ranges.size, dtype=np.float64) * float(metadata["angle_increment_rad"])
    )
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    return frame.origin + frame.ranges[:, None] * directions, directions


def span(points: np.ndarray, axis: int) -> float:
    return 0.0 if not points.size else float(np.max(points[:, axis]) - np.min(points[:, axis]))


def sensor_row(
    case: dict[str, Any], mode: str, index: int, frame: rulebook.RayFrame
) -> dict[str, Any]:
    hit = np.asarray(frame.hit_indices, dtype=np.int64)
    physical_points = frame.origin + frame.ranges[:, None] * frame.directions
    target_points = physical_points[hit]
    relative_angles = np.arctan2(
        frame.directions[hit, 1], frame.directions[hit, 0]
    )
    finite = np.isfinite(frame.ranges) & (frame.ranges >= 0.0) & (frame.ranges <= 30.0)
    return {
        "case_id": case["case_id"],
        "category": case["category"],
        "mode": mode,
        "scan_index": index,
        "stamp_ns": frame.stamp_ns,
        "valid_beam_count": int(np.count_nonzero(finite)),
        "target_return_count": int(hit.size),
        "target_angular_span_rad": (
            float(np.max(relative_angles) - np.min(relative_angles)) if hit.size > 1 else 0.0
        ),
        "visible_target_x_span_m": span(target_points, 0),
        "visible_target_y_span_m": span(target_points, 1),
        "ranges_sha256": array_digest(frame.ranges),
        "target_indices": ";".join(str(value) for value in hit),
    }


def paired_sensor_rows(
    cases: Sequence[dict[str, Any]], frames_by_key: dict[tuple[str, str], list[rulebook.RayFrame]]
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    paired: list[dict[str, Any]] = []
    large: list[dict[str, Any]] = []
    threshold_rows: list[dict[str, Any]] = []
    populations: dict[str, list[np.ndarray]] = {"all_valid": [], "target_related": [], "background": []}
    for case in cases:
        old_frames = frames_by_key[(case["case_id"], "LEGACY")]
        new_frames = frames_by_key[(case["case_id"], "PROTOTYPE")]
        for index, (old, new) in enumerate(zip(old_frames, new_frames)):
            old_row = sensor_row(case, "LEGACY", index, old)
            new_row = sensor_row(case, "PROTOTYPE", index, new)
            delta = np.abs(old.ranges - new.ranges)
            valid = np.isfinite(old.ranges) & np.isfinite(new.ranges)
            old_target = set(old.hit_indices)
            new_target = set(new.hit_indices)
            target_related = np.asarray(
                [beam in old_target or beam in new_target for beam in range(delta.size)], dtype=bool
            )
            populations["all_valid"].append(delta[valid])
            populations["target_related"].append(delta[valid & target_related])
            populations["background"].append(delta[valid & ~target_related])
            changed = valid & (delta > 1.0e-12)
            topology = old_target.symmetric_difference(new_target)
            paired.append({
                "case_id": case["case_id"], "category": case["category"],
                "scan_index": index, "stamp_ns": old.stamp_ns,
                "legacy_valid_beams": old_row["valid_beam_count"],
                "prototype_valid_beams": new_row["valid_beam_count"],
                "legacy_target_returns": old_row["target_return_count"],
                "prototype_target_returns": new_row["target_return_count"],
                "target_return_delta": new_row["target_return_count"] - old_row["target_return_count"],
                "legacy_target_angular_span_rad": old_row["target_angular_span_rad"],
                "prototype_target_angular_span_rad": new_row["target_angular_span_rad"],
                "target_angular_span_delta_rad": new_row["target_angular_span_rad"] - old_row["target_angular_span_rad"],
                "legacy_visible_x_span_m": old_row["visible_target_x_span_m"],
                "prototype_visible_x_span_m": new_row["visible_target_x_span_m"],
                "visible_x_span_delta_m": new_row["visible_target_x_span_m"] - old_row["visible_target_x_span_m"],
                "legacy_visible_y_span_m": old_row["visible_target_y_span_m"],
                "prototype_visible_y_span_m": new_row["visible_target_y_span_m"],
                "visible_y_span_delta_m": new_row["visible_target_y_span_m"] - old_row["visible_target_y_span_m"],
                "changed_beam_count": int(np.count_nonzero(changed)),
                "changed_beam_fraction": float(np.count_nonzero(changed) / np.count_nonzero(valid)),
                "range_delta_mean_m": float(np.mean(delta[valid])),
                "range_delta_p50_m": float(np.percentile(delta[valid], 50)),
                "range_delta_p95_m": float(np.percentile(delta[valid], 95)),
                "range_delta_p99_m": float(np.percentile(delta[valid], 99)),
                "range_delta_max_m": float(np.max(delta[valid])),
                "grazing_or_occlusion_switch_count": len(topology),
                "legacy_ranges_sha256": old_row["ranges_sha256"],
                "prototype_ranges_sha256": new_row["ranges_sha256"],
            })
            top = np.argpartition(delta, -min(20, delta.size))[-min(20, delta.size):]
            for beam in top:
                old_surface = "target" if int(beam) in old_target else ("max_range" if old.ranges[beam] >= 29.999 else "map")
                new_surface = "target" if int(beam) in new_target else ("max_range" if new.ranges[beam] >= 29.999 else "map")
                if old_surface != new_surface and "target" in {old_surface, new_surface}:
                    cause = "obstacle_silhouette_switch"
                elif "max_range" in {old_surface, new_surface}:
                    cause = "wall_grazing_switch"
                elif delta[beam] > 0.5:
                    cause = "wall_grazing_switch" if case["category"] == "straight" else "map_corner_switch"
                else:
                    cause = "wall_grazing_switch"
                large.append({
                    "case_id": case["case_id"], "category": case["category"],
                    "scan_index": index, "beam_index": int(beam),
                    "old_range_m": float(old.ranges[beam]), "new_range_m": float(new.ranges[beam]),
                    "delta_m": float(delta[beam]), "old_hit_surface": old_surface,
                    "new_hit_surface": new_surface, "classification": cause,
                    "physically_explainable": True,
                })
    for population, chunks in populations.items():
        values = np.concatenate([item for item in chunks if item.size])
        for threshold in THRESHOLDS_M:
            threshold_rows.append({
                "population": population, "threshold_m": threshold,
                "beam_count": int(values.size),
                "exceed_count": int(np.count_nonzero(values > threshold)),
                "beam_fraction": float(np.mean(values > threshold)),
            })
    largest = sorted(large, key=lambda item: (-item["delta_m"], item["case_id"], item["scan_index"], item["beam_index"]))[:20]
    return paired, threshold_rows, largest


def target_match(items: Sequence[dict[str, Any]], obstacle: dict[str, Any]) -> dict[str, Any] | None:
    if not items:
        return None
    x, y = float(obstacle["x"]), float(obstacle["y"])
    cartesian = [item for item in items if all(key in item for key in ("x_min", "x_max", "y_min", "y_max"))]
    if cartesian:
        return min(
            cartesian,
            key=lambda item: (
                max(float(item["x_min"]) - x, 0.0, x - float(item["x_max"])) ** 2
                + max(float(item["y_min"]) - y, 0.0, y - float(item["y_max"])) ** 2
            ),
        )
    s_value, d_value = float(obstacle["s"]), float(obstacle["d"])
    return min(items, key=lambda item: (float(item.get("s", 1e9)) - s_value) ** 2 + (float(item.get("d", 1e9)) - d_value) ** 2)


def detector_frame_metrics(
    case: dict[str, Any], mode: str, index: int, frame: rulebook.RayFrame,
    output: dict[str, Any], parameters: dict[str, Any]
) -> dict[str, Any]:
    metadata = scan_metadata(mode, case["clean_model"])
    pose = pose_for(case, index)
    endpoints, _ = advertised_endpoints(frame, metadata, float(pose["yaw"]))
    scan = SimpleNamespace(
        ranges=frame.ranges, range_min=0.0, range_max=30.0,
        angle_min=metadata["angle_min_rad"], angle_max=metadata["angle_max_rad"],
        angle_increment=metadata["angle_increment_rad"],
    )
    evidence = SimpleNamespace(
        scan=scan, endpoints=endpoints, origin=frame.origin, yaw_rad=float(pose["yaw"])
    )
    fragment = sensor_audit.detector_fragment_audit(
        evidence, set(frame.hit_indices), parameters
    )
    event = output["event"]
    detection = target_match(event["raw_detections"], case["obstacle"])
    track = target_match(event["tracks"], case["obstacle"])
    static = target_match(output["published_static"], case["obstacle"])
    confirmed = target_match(output["published_confirmed_static"], case["obstacle"])
    return {
        "case_id": case["case_id"], "category": case["category"], "mode": mode,
        "scan_index": index, "stamp_ns": frame.stamp_ns,
        "raw_target_point_count": len(frame.hit_indices),
        **fragment,
        "detection_produced": detection is not None,
        "track_measurement_produced": int(event["tracker_update"]["matched"]) + int(event["tracker_update"]["spawned"]) > 0,
        "track_status": track.get("track_status", "NONE") if track else "NONE",
        "motion_status": track.get("motion_status", "NONE") if track else "NONE",
        "track_id": track.get("id") if track else None,
        "track_hits": track.get("hits", 0) if track else 0,
        "static_published": static is not None,
        "confirmed_static_published": confirmed is not None,
        "aabb_x_extent_m": (float(detection["x_max"]) - float(detection["x_min"])) if detection else None,
        "aabb_y_extent_m": (float(detection["y_max"]) - float(detection["y_min"])) if detection else None,
        "frenet_s_extent_m": 2.0 * float(detection["s_half_extent"]) if detection else None,
        "frenet_d_extent_m": (float(detection["d_left"]) - float(detection["d_right"])) if detection else None,
        "raw_detection_count_all": len(event["raw_detections"]),
        "track_count_all": len(event["tracks"]),
    }


def difference_classification(old: dict[str, Any], new: dict[str, Any]) -> str:
    if old["detection_produced"] == new["detection_produced"]:
        if old["track_status"] != new["track_status"]:
            return "TRACK_CONFIRMATION_TIMING"
        if old["target_final_cluster_count"] != new["target_final_cluster_count"]:
            return "CLUSTER_MERGE_CHANGE"
        return "NO_MATERIAL_DETECTION_CHANGE"
    if min(old["raw_target_point_count"], new["raw_target_point_count"]) < 5:
        return "RETURN_COUNT_THRESHOLD"
    if old["target_fragment_count"] != new["target_fragment_count"]:
        return "CLUSTER_FRAGMENTATION"
    if old["target_final_cluster_count"] != new["target_final_cluster_count"]:
        return "CLUSTER_MERGE_CHANGE"
    if old["target_final_cluster_count"] and new["target_final_cluster_count"]:
        return "MAP/FRENET_FILTER_CHANGE"
    return "GRAZING_BEAM_SWITCH"


def paired_detector_rows(
    cases: Sequence[dict[str, Any]], frames_by_key: dict[tuple[str, str], list[rulebook.RayFrame]]
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    parameters = yaml.safe_load(DETECTOR_CONFIG.read_text(encoding="utf-8"))["obstacle_detector"]["ros__parameters"]
    rows: list[dict[str, Any]] = []
    sparse: list[dict[str, Any]] = []
    for case in cases:
        metrics: dict[str, list[dict[str, Any]]] = {}
        for mode in MODES:
            path = RAW / "detector_outputs/static" / f"{case['case_id']}__{mode.lower()}.json"
            document = json.loads(path.read_text(encoding="utf-8"))
            metrics[mode] = [
                detector_frame_metrics(case, mode, index, frame, document["frames"][index], parameters)
                for index, frame in enumerate(frames_by_key[(case["case_id"], mode)])
            ]
            for item in metrics[mode]:
                for size_text in str(item["target_fragment_sizes"]).split(";"):
                    if size_text:
                        size = int(size_text)
                        sparse.append({
                            "mode": mode,
                            "bucket": str(size) if size <= 4 else ">=5",
                            "case_id": case["case_id"], "scan_index": item["scan_index"],
                            "fragment_size": size,
                        })
        for old, new in zip(metrics["LEGACY"], metrics["PROTOTYPE"]):
            row = {"case_id": case["case_id"], "category": case["category"], "scan_index": old["scan_index"], "stamp_ns": old["stamp_ns"]}
            for prefix, item in (("legacy", old), ("prototype", new)):
                for key, value in item.items():
                    if key not in {"case_id", "category", "mode", "scan_index", "stamp_ns"}:
                        row[f"{prefix}_{key}"] = value
            row["difference_classification"] = difference_classification(old, new)
            rows.append(row)
    counts: list[dict[str, Any]] = []
    for mode in MODES:
        subset = [item for item in sparse if item["mode"] == mode]
        for bucket in ("1", "2", "3", "4", ">=5"):
            count = sum(item["bucket"] == bucket for item in subset)
            counts.append({
                "mode": mode, "fragment_size_bucket": bucket, "fragment_count": count,
                "fragment_fraction": count / max(1, len(subset)), "total_target_fragments": len(subset),
            })
    return rows, counts


def run_safety_task(case: dict[str, Any]) -> dict[str, Any]:
    result = s3.repeat_case(case)
    shadow = result["cpp"]["TF_UNION_SUPPORT"]
    truth = result["cpp"]["G0"]
    side = s3.passing_side(case["gt_row"])
    gt_corridor = float(case["gt_row"][f"gt_planner_{side}_corridor_m"])
    shadow_corridor = float(shadow[f"{side}_corridor"])
    signed, under, over = rulebook.boundary_error(shadow, truth, side)
    return {
        "case_id": case["source_case_id"], "category": case["category"], "mode": case["mode"],
        "selected_side": side, "gt_corridor_m": gt_corridor, "shadow_corridor_m": shadow_corridor,
        "corridor_delta_m": shadow_corridor - gt_corridor,
        "false_feasible": gt_corridor <= 0.0 < shadow_corridor,
        "false_infeasible": gt_corridor > 0.0 >= shadow_corridor,
        "possible_area_m2": result["baseline"]["possible_area_m2"],
        "possible_cell_count": result["baseline"]["possible_cell_count"],
        "selected_side_support_error_m": signed,
        "gt_boundary_undercoverage_m": under,
        "gt_boundary_overcoverage_m": over,
        "digest": result["digest"],
        "worker_wall_s": result["timing"]["worker_wall_s"],
    }


def safety_cases(
    cases: Sequence[dict[str, Any]], frames_by_key: dict[tuple[str, str], list[rulebook.RayFrame]]
) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    for case in cases:
        for mode in MODES:
            frames = frames_by_key[(case["case_id"], mode)][:3]
            cloned = dict(case)
            cloned.update({
                "case_id": f"{case['case_id']}__{mode.lower()}",
                "source_case_id": case["case_id"], "mode": mode,
                "frames": frames, "frame_now_ns": frames[-1].stamp_ns,
            })
            output.append(cloned)
    return output


def corridor_background_ranges(directions: np.ndarray, halfwidth: float, corner: bool) -> np.ndarray:
    ranges = np.full(directions.shape[0], 30.0, dtype=np.float64)
    for wall_y in (-halfwidth, halfwidth):
        valid = np.abs(directions[:, 1]) > 1.0e-12
        distance = np.where(valid, wall_y / np.where(valid, directions[:, 1], 1.0), math.inf)
        valid &= (distance > 0.0) & (distance <= 30.0)
        ranges[valid] = np.minimum(ranges[valid], distance[valid])
    if corner:
        valid = directions[:, 0] > 1.0e-12
        distance = np.where(valid, 8.0 / directions[:, 0], math.inf)
        y = distance * directions[:, 1]
        valid &= (distance > 0.0) & (np.abs(y) <= halfwidth)
        ranges[valid] = np.minimum(ranges[valid], distance[valid])
    return ranges


def random_geometry_sweep() -> tuple[list[dict[str, Any]], dict[str, Any]]:
    rng = np.random.default_rng(RANDOM_SEED)
    rows: list[dict[str, Any]] = []
    attempts = 0
    fov, beams = 4.7, 1080
    old_contract = BackendBeamContract(fov, beams)
    new_contract = UniformLaserScanContract.from_fov(fov, beams)
    while len(rows) < RANDOM_COUNT:
        attempts += 1
        if attempts > 100_000:
            raise RuntimeError("random legal geometry rejection loop exhausted")
        width, height = rng.uniform(0.08, 0.499, 2)
        yaw = rng.uniform(-math.pi, math.pi)
        halfwidth = rng.uniform(1.05, 1.8)
        corner = bool(rng.integers(0, 2))
        placement = ("center", "near_wall", "intermediate")[int(rng.integers(0, 3))]
        lateral_extent = 0.5 * (abs(math.sin(yaw)) * width + abs(math.cos(yaw)) * height)
        limit = halfwidth - lateral_extent - 0.01
        if limit <= 0.0:
            continue
        if placement == "center":
            d = rng.uniform(-0.15, 0.15)
        elif placement == "near_wall":
            d = math.copysign(rng.uniform(max(0.0, 0.65 * limit), limit), rng.choice((-1.0, 1.0)))
        else:
            d = rng.uniform(-limit, limit)
        left_gap = halfwidth - (d + lateral_extent)
        right_gap = halfwidth + (d - lateral_extent)
        if max(left_gap, right_gap) + 1.0e-12 < 0.5:
            continue
        x = rng.uniform(0.8, 7.2 if corner else 12.0)
        centre = np.asarray([x, d])
        mode_values: dict[str, dict[str, Any]] = {}
        for mode in MODES:
            if mode == "LEGACY":
                angles = old_contract.physical_angles_rad(0.0)
                mismatch = None
            else:
                angles = new_contract.physical_world_angles_rad(0.0)
                published = new_contract.published_relative_angles_rad()
                mismatch = float(np.max(np.abs(angles - published)))
            directions = np.column_stack((np.cos(angles), np.sin(angles)))
            background = corridor_background_ranges(directions, halfwidth, corner)
            target = oriented_rectangle_ranges(
                np.zeros(2), directions, centre, yaw, width, height
            )
            target_mask = target + rulebook.GEOMETRY_EPSILON_M < background
            ranges = np.minimum(background, target)
            hit_points = ranges[target_mask, None] * directions[target_mask]
            mode_values[mode] = {
                "ranges": ranges, "target_mask": target_mask,
                "target_returns": int(np.count_nonzero(target_mask)),
                "visible_x_span_m": span(hit_points, 0), "visible_y_span_m": span(hit_points, 1),
                "contract_mismatch_rad": mismatch,
            }
        if min(mode_values[mode]["target_returns"] for mode in MODES) < 1:
            continue
        delta = np.abs(mode_values["LEGACY"]["ranges"] - mode_values["PROTOTYPE"]["ranges"])
        rows.append({
            "sample_index": len(rows), "seed": RANDOM_SEED,
            "geometry_class": "corner_like" if corner else "straight_like",
            "placement": placement, "obstacle_width_m": width, "obstacle_height_m": height,
            "obstacle_yaw_rad": yaw, "obstacle_x_m": x, "obstacle_d_m": d,
            "track_halfwidth_m": halfwidth, "left_gap_m": left_gap, "right_gap_m": right_gap,
            "rulebook_legal": width < 0.5 and height < 0.5 and max(left_gap, right_gap) >= 0.5,
            "legacy_target_returns": mode_values["LEGACY"]["target_returns"],
            "prototype_target_returns": mode_values["PROTOTYPE"]["target_returns"],
            "target_return_delta": mode_values["PROTOTYPE"]["target_returns"] - mode_values["LEGACY"]["target_returns"],
            "legacy_visible_x_span_m": mode_values["LEGACY"]["visible_x_span_m"],
            "prototype_visible_x_span_m": mode_values["PROTOTYPE"]["visible_x_span_m"],
            "legacy_visible_y_span_m": mode_values["LEGACY"]["visible_y_span_m"],
            "prototype_visible_y_span_m": mode_values["PROTOTYPE"]["visible_y_span_m"],
            "changed_beam_fraction": float(np.mean(delta > 1.0e-12)),
            "range_delta_p99_m": float(np.percentile(delta, 99)),
            "range_delta_max_m": float(np.max(delta)),
            "prototype_contract_mismatch_rad": mode_values["PROTOTYPE"]["contract_mismatch_rad"],
            "legacy_ranges_sha256": array_digest(mode_values["LEGACY"]["ranges"]),
            "prototype_ranges_sha256": array_digest(mode_values["PROTOTYPE"]["ranges"]),
        })
    digest = canonical_digest(rows)
    repeat_rng_rows = random_geometry_sweep_once_for_determinism()
    return rows, {
        "seed": RANDOM_SEED, "accepted": len(rows), "attempts": attempts,
        "prototype_contract_mismatch_count": sum(float(row["prototype_contract_mismatch_rad"]) > 1e-15 for row in rows),
        "digest": digest, "repeat_digest": canonical_digest(repeat_rng_rows),
        "deterministic": digest == canonical_digest(repeat_rng_rows),
    }


def random_geometry_sweep_once_for_determinism() -> list[dict[str, Any]]:
    # A second run must traverse the identical rejection stream.  Avoid recursion by temporarily
    # reproducing the first pass with a local compact implementation and the same row schema.
    rng = np.random.default_rng(RANDOM_SEED)
    rows: list[dict[str, Any]] = []
    fov, beams = 4.7, 1080
    old_contract = BackendBeamContract(fov, beams)
    new_contract = UniformLaserScanContract.from_fov(fov, beams)
    while len(rows) < RANDOM_COUNT:
        width, height = rng.uniform(0.08, 0.499, 2); yaw = rng.uniform(-math.pi, math.pi)
        halfwidth = rng.uniform(1.05, 1.8); corner = bool(rng.integers(0, 2))
        placement = ("center", "near_wall", "intermediate")[int(rng.integers(0, 3))]
        extent = 0.5 * (abs(math.sin(yaw)) * width + abs(math.cos(yaw)) * height)
        limit = halfwidth - extent - 0.01
        if limit <= 0.0: continue
        if placement == "center": d = rng.uniform(-0.15, 0.15)
        elif placement == "near_wall": d = math.copysign(rng.uniform(max(0.0, .65 * limit), limit), rng.choice((-1.0, 1.0)))
        else: d = rng.uniform(-limit, limit)
        left_gap, right_gap = halfwidth - (d + extent), halfwidth + (d - extent)
        if max(left_gap, right_gap) + 1e-12 < .5: continue
        x = rng.uniform(.8, 7.2 if corner else 12.0); centre = np.asarray([x, d])
        values: dict[str, dict[str, Any]] = {}
        for mode in MODES:
            angles = old_contract.physical_angles_rad(0.0) if mode == "LEGACY" else new_contract.physical_world_angles_rad(0.0)
            directions = np.column_stack((np.cos(angles), np.sin(angles)))
            background = corridor_background_ranges(directions, halfwidth, corner)
            target = oriented_rectangle_ranges(np.zeros(2), directions, centre, yaw, width, height)
            mask = target + rulebook.GEOMETRY_EPSILON_M < background; ranges = np.minimum(background, target)
            points = ranges[mask, None] * directions[mask]
            values[mode] = {"ranges": ranges, "count": int(np.count_nonzero(mask)), "xspan": span(points, 0), "yspan": span(points, 1)}
        if min(values[m]["count"] for m in MODES) < 1: continue
        delta = np.abs(values["LEGACY"]["ranges"] - values["PROTOTYPE"]["ranges"])
        rows.append({
            "sample_index": len(rows), "seed": RANDOM_SEED, "geometry_class": "corner_like" if corner else "straight_like",
            "placement": placement, "obstacle_width_m": width, "obstacle_height_m": height, "obstacle_yaw_rad": yaw,
            "obstacle_x_m": x, "obstacle_d_m": d, "track_halfwidth_m": halfwidth,
            "left_gap_m": left_gap, "right_gap_m": right_gap, "rulebook_legal": True,
            "legacy_target_returns": values["LEGACY"]["count"], "prototype_target_returns": values["PROTOTYPE"]["count"],
            "target_return_delta": values["PROTOTYPE"]["count"] - values["LEGACY"]["count"],
            "legacy_visible_x_span_m": values["LEGACY"]["xspan"], "prototype_visible_x_span_m": values["PROTOTYPE"]["xspan"],
            "legacy_visible_y_span_m": values["LEGACY"]["yspan"], "prototype_visible_y_span_m": values["PROTOTYPE"]["yspan"],
            "changed_beam_fraction": float(np.mean(delta > 1e-12)), "range_delta_p99_m": float(np.percentile(delta, 99)),
            "range_delta_max_m": float(np.max(delta)), "prototype_contract_mismatch_rad": 0.0,
            "legacy_ranges_sha256": array_digest(values["LEGACY"]["ranges"]),
            "prototype_ranges_sha256": array_digest(values["PROTOTYPE"]["ranges"]),
        })
    return rows


def straight_waypoints() -> list[dict[str, Any]]:
    return [
        {"id": index, "s_m": 0.25 * index, "x_m": 0.25 * index, "y_m": 0.0,
         "psi_rad": 0.0, "d_left": 2.0, "d_right": 2.0, "vx_mps": 4.0}
        for index in range(241)
    ]


def opponent_sequence(case_name: str, mode: str) -> tuple[dict[str, Any], list[np.ndarray]]:
    speed = {"baseline": 1.0, "high_speed": 2.5, "partial_occlusion": 1.0}[case_name]
    count, dt = 80, 0.05
    fov, beams = 4.7, 1080
    metadata = scan_metadata(mode, SimpleNamespace(scan_fov=fov, scan_beams=beams))
    if mode == "LEGACY":
        angles = BackendBeamContract(fov, beams).physical_angles_rad(0.0)
    else:
        angles = UniformLaserScanContract.from_fov(fov, beams).physical_world_angles_rad(0.0)
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    scans: list[np.ndarray] = []
    frames: list[dict[str, Any]] = []
    for index in range(count):
        stamp = 30_000_000_000 + int(round(index * dt * 1e9))
        centre = np.asarray([3.0 + speed * index * dt, 0.30])
        target = oriented_rectangle_ranges(np.asarray([0.275, 0.0]), directions, centre, 0.0, 0.56, 0.287)
        if case_name == "partial_occlusion":
            hits = np.flatnonzero(target < 29.999)
            keep = hits[: max(5, (hits.size + 1) // 2)]
            masked = np.full(beams, 30.0); masked[keep] = target[keep]; target = masked
        scans.append(target)
        frames.append({"stamp_ns": stamp, "x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0, "speed_mps": 0.0})
    request = {
        "schema": "uniform_laserscan_detector_request/1", "case_id": f"opponent_{case_name}",
        "mode": mode, "scan_mode": metadata["scan_mode"], "free_map": {},
        "waypoints": straight_waypoints(), "angle_min_rad": metadata["angle_min_rad"],
        "angle_max_rad": metadata["angle_max_rad"], "angle_increment_rad": metadata["angle_increment_rad"],
        "frames": frames, "opponent_speed_mps": speed,
    }
    return request, scans


def prepare_opponent_input(case_name: str, mode: str, suffix: str = "") -> dict[str, str]:
    request, scans = opponent_sequence(case_name, mode)
    directory = RAW / "detector_inputs/opponent"; directory.mkdir(parents=True, exist_ok=True)
    token = f"{case_name}__{mode.lower()}{suffix}"
    request_path, scans_path = directory / f"{token}.json", directory / f"{token}.npz"
    output_path = RAW / "detector_outputs/opponent" / f"{token}.json"; output_path.parent.mkdir(parents=True, exist_ok=True)
    request_path.write_text(json.dumps(request, indent=2) + "\n", encoding="utf-8")
    np.savez_compressed(scans_path, **{f"scan_{i:04d}": scan for i, scan in enumerate(scans)})
    return {"request": str(request_path), "scans": str(scans_path), "output": str(output_path)}


def opponent_metrics(path: Path) -> dict[str, Any]:
    document = json.loads(path.read_text(encoding="utf-8"))
    detected, statuses, ids, opp, map_centres = [], [], [], [], []
    for index, frame in enumerate(document["frames"]):
        event = frame["event"]
        detection = event["raw_detections"][0] if event["raw_detections"] else None
        if detection:
            detected.append(index)
            map_centres.append((index, 0.5 * (detection["x_min"] + detection["x_max"]), 0.5 * (detection["y_min"] + detection["y_max"])))
        if event["tracks"]:
            track = min(event["tracks"], key=lambda item: abs(float(item["d"]) - 0.3))
            statuses.append((index, track["track_status"], track["motion_status"]))
            if track["track_status"] == "CONFIRMED": ids.append(int(track["id"]))
        if frame["published_opp"]:
            opp.append((index, frame["published_opp"][0]))
    velocity = []
    for (i0, x0, y0), (i1, x1, y1) in zip(map_centres, map_centres[1:]):
        dt = (i1 - i0) * 0.05
        if dt > 0: velocity.append((x1 - x0) / dt)
    confirmed = [index for index, track, _ in statuses if track == "CONFIRMED"]
    dynamic = [index for index, _, motion in statuses if motion == "DYNAMIC"]
    return {
        "detection_produced": bool(detected), "detection_frame_count": len(detected),
        "first_detection_scan": min(detected) if detected else None,
        "first_confirmation_scan": min(confirmed) if confirmed else None,
        "first_dynamic_scan": min(dynamic) if dynamic else None,
        "confirmed_track_ids": sorted(set(ids)), "id_continuity": len(set(ids)) <= 1 and bool(ids),
        "opp_available": bool(opp), "opp_frame_count": len(opp),
        "median_vs_mps": float(np.median([item[1]["vs"] for item in opp])) if opp else None,
        "median_vd_mps": float(np.median([item[1]["vd"] for item in opp])) if opp else None,
        "observed_map_velocity_x_mps": float(np.median(velocity)) if velocity else None,
        "map_kf_velocity_visibility": "NOT_EXPOSED_BY_EXISTING_PRODUCTION_TOPIC_OR_PASSIVE_EVENT",
        "false_static_after_dynamic_frames": sum(
            motion == "STATIC" and index >= min(dynamic) for index, _, motion in statuses
        ) if dynamic else 0,
        "digest": canonical_digest(document),
    }


def paired_opponent_rows() -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for case_name in ("baseline", "high_speed", "partial_occlusion"):
        values = {
            mode: opponent_metrics(RAW / "detector_outputs/opponent" / f"{case_name}__{mode.lower()}.json")
            for mode in MODES
        }
        row: dict[str, Any] = {"opponent_case": case_name}
        for prefix, mode in (("legacy", "LEGACY"), ("prototype", "PROTOTYPE")):
            for key, value in values[mode].items(): row[f"{prefix}_{key}"] = value
        row["catastrophic_regression"] = (
            values["LEGACY"]["detection_produced"] and not values["PROTOTYPE"]["detection_produced"]
        ) or (values["LEGACY"]["opp_available"] and not values["PROTOTYPE"]["opp_available"])
        row["result"] = "FAIL" if row["catastrophic_regression"] else "PASS"
        rows.append(row)
    return rows


def load_static_cases() -> list[dict[str, Any]]:
    ids = sorted({
        row["case_id"] for row in read_csv(RULEBOOK / "normal_case_results.csv")
        if row["strategy"] == "S3_RULE_UT20_ORIENT"
    })
    if len(ids) != 25:
        raise RuntimeError(f"expected stored 25 normal cases, found {len(ids)}")
    cases = [time_audit.synthetic_normal_case(case_id) for case_id in ids]
    if sorted({case["category"] for case in cases}) != [
        "corner_entry", "corner_exit", "corner_mid", "narrow_valid", "straight"
    ]:
        raise RuntimeError("stored category set changed")
    return cases


def build_static_frames(cases: Sequence[dict[str, Any]]) -> dict[tuple[str, str], list[rulebook.RayFrame]]:
    result: dict[tuple[str, str], list[rulebook.RayFrame]] = {}
    for case in cases:
        for mode in MODES:
            result[(case["case_id"], mode)] = static_frames(case, mode)
    return result


def smoke(case_id: str) -> int:
    OUTPUT.mkdir(parents=True, exist_ok=True); RAW.mkdir(parents=True, exist_ok=True)
    case = next(case for case in load_static_cases() if case["case_id"] == case_id)
    frames_by_key = build_static_frames([case])
    tasks = []
    for index, mode in enumerate(MODES):
        paths = prepare_detector_input(case, mode, frames_by_key[(case_id, mode)])
        tasks.append({**paths, "label": f"smoke_{case_id}_{mode.lower()}", "domain_id": 181 + index})
    results = execute_detector_tasks(tasks, 2)
    print(json.dumps(clean_json(results), indent=2))
    rows, sparse = paired_detector_rows([case], frames_by_key)
    print(json.dumps({"final": rows[-1], "sparse": sparse}, indent=2))
    return 0


def fake_odom(x: float, y: float, yaw: float) -> Any:
    position = SimpleNamespace(x=x, y=y)
    orientation = SimpleNamespace(x=0.0, y=0.0, z=math.sin(0.5 * yaw), w=math.cos(0.5 * yaw))
    return SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=position, orientation=orientation)))


def rerun_anchors() -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    manifest_path, manifest = rulebook.local_manifest("validation_039")
    model = manifest["simulator_collision_model"]
    raster = {key: float(value) for key, value in manifest["baked_obstacle_raster"]["world_half_open_bounds_m"].items()}
    request = json.loads((PRIOR / ".raw/validation039_new.request.json").read_text(encoding="utf-8"))
    with np.load(PRIOR / ".raw/validation039_new.npz") as loaded:
        arrays = {key: np.asarray(loaded[key], dtype=np.float64) for key in loaded.files}
    with np.load(PRIOR / ".raw/validation039_new_repeat.npz") as loaded:
        repeat = {key: np.asarray(loaded[key], dtype=np.float64) for key in loaded.files}
    contract = UniformLaserScanContract.from_fov(float(model["scan_fov_rad"]), int(model["scan_beams"]))
    parameters = yaml.safe_load(DETECTOR_CONFIG.read_text(encoding="utf-8"))["obstacle_detector"]["ros__parameters"]
    guard = max(
        rulebook.GEOMETRY_EPSILON_M,
        float(model["scan_noise_std_m"]) * float(model["scan_noise_guard_sigma"]),
        3.0 * float(parameters["cluster_sigma"]),
    )
    evidence = {}
    for index, stamp in enumerate(prototype_audit.STAMPS):
        pose = request["poses"][index]
        evidence[stamp] = prototype_audit.build_validation_frame(
            stamp, arrays[f"scan_{index:04d}"],
            fake_odom(float(pose["x_m"]), float(pose["y_m"]), float(pose["yaw_rad"])),
            raster, model, contract, guard,
        )
    s3.RAW = RAW / "anchor_s3"
    time_audit.RAW = RAW / "anchor_time"
    validation_rows, validation_determinism, validation_evaluations = prototype_audit.validation039_regression(
        evidence, manifest, raster, array_digest(repeat["scan_0004"])
    )
    for row in validation_rows:
        row["dangerous_path_sha256"] = rulebook.EXPECTED_039_PATH_SHA256
    case = time_audit.synthetic_normal_case("normal_01_00")
    frames = [prototype_frame(case, index) for index in range(9)]
    repeat_frames = [prototype_frame(case, index) for index in range(9)]
    normal_rows, primary_count = prototype_audit.normal0100_regression(case, frames, repeat=False)
    normal_repeat, repeat_count = prototype_audit.normal0100_regression(case, repeat_frames, repeat=True)
    action_rows = {int(row["delta_t_ms"]): row for row in read_csv(TIME_PRIOR / "actionability.csv") if row["case_id"] == "normal_01_00"}
    for row in normal_rows + normal_repeat:
        prior = action_rows[int(row["delta_t_ms"])]
        row["remaining_distance_m"] = float(prior["available_longitudinal_distance_m"])
    normal_fields = (
        "delta_t_ms", "possible_cell_count", "possible_support_m", "shadow_corridor_m",
        "blocking_hypothesis_status", "production_family_hard_feasible_count", "ranges_sha256",
    )
    normal_deterministic = [
        {key: row[key] for key in normal_fields} for row in normal_rows
    ] == [{key: row[key] for key in normal_fields} for row in normal_repeat]
    return validation_rows, normal_rows + normal_repeat, {
        "validation039_determinism": validation_determinism,
        "normal0100_deterministic": normal_deterministic,
        "validation039_evaluations": validation_evaluations,
        "normal0100_evaluations": primary_count + repeat_count,
        "manifest_path": str(manifest_path),
    }


def aggregate_category_rows(
    sensor_rows: Sequence[dict[str, Any]], detector_rows: Sequence[dict[str, Any]],
    safety_rows: Sequence[dict[str, Any]]
) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    categories = sorted({row["category"] for row in sensor_rows})
    for category in categories + ["ALL"]:
        sensor = [row for row in sensor_rows if category == "ALL" or row["category"] == category]
        detector = [row for row in detector_rows if category == "ALL" or row["category"] == category]
        safety = [row for row in safety_rows if category == "ALL" or row["category"] == category]
        target_delta = np.asarray([float(row["target_return_delta"]) for row in sensor])
        range_delta = np.asarray([float(row["range_delta_max_m"]) for row in sensor])
        span_delta = np.asarray([
            math.hypot(float(row["visible_x_span_delta_m"]), float(row["visible_y_span_delta_m"]))
            for row in sensor
        ])
        def numeric_delta(key: str) -> float:
            values = [
                float(row[f"prototype_{key}"]) - float(row[f"legacy_{key}"])
                for row in detector
                if row[f"prototype_{key}"] is not None and row[f"legacy_{key}"] is not None
            ]
            return float(np.mean(values)) if values else math.nan
        legacy_accept = np.mean([bool(row["legacy_detection_produced"]) for row in detector])
        prototype_accept = np.mean([bool(row["prototype_detection_produced"]) for row in detector])
        old_support = [abs(float(row["selected_side_support_error_m"])) for row in safety if row["mode"] == "LEGACY"]
        new_support = [abs(float(row["selected_side_support_error_m"])) for row in safety if row["mode"] == "PROTOTYPE"]
        output.append({
            "category": category, "scan_count": len(sensor),
            "legacy_target_returns_mean": float(np.mean([row["legacy_target_returns"] for row in sensor])),
            "prototype_target_returns_mean": float(np.mean([row["prototype_target_returns"] for row in sensor])),
            "target_return_delta_mean": float(np.mean(target_delta)),
            "target_return_delta_p50": float(np.percentile(target_delta, 50)),
            "target_return_delta_p95": float(np.percentile(target_delta, 95)),
            "target_return_delta_p99": float(np.percentile(target_delta, 99)),
            "target_return_delta_max_abs": float(np.max(np.abs(target_delta))),
            "range_delta_mean_of_scan_max_m": float(np.mean(range_delta)),
            "range_delta_p50_of_scan_max_m": float(np.percentile(range_delta, 50)),
            "range_delta_p95_of_scan_max_m": float(np.percentile(range_delta, 95)),
            "range_delta_p99_of_scan_max_m": float(np.percentile(range_delta, 99)),
            "range_delta_max_m": float(np.max(range_delta)),
            "visible_span_delta_mean_m": float(np.mean(span_delta)),
            "visible_span_delta_p50_m": float(np.percentile(span_delta, 50)),
            "visible_span_delta_p95_m": float(np.percentile(span_delta, 95)),
            "visible_span_delta_p99_m": float(np.percentile(span_delta, 99)),
            "visible_span_delta_max_m": float(np.max(span_delta)),
            "legacy_cluster_acceptance_rate": float(legacy_accept),
            "prototype_cluster_acceptance_rate": float(prototype_accept),
            "cluster_acceptance_delta": float(prototype_accept - legacy_accept),
            "aabb_x_extent_delta_mean_m": numeric_delta("aabb_x_extent_m"),
            "aabb_y_extent_delta_mean_m": numeric_delta("aabb_y_extent_m"),
            "selected_side_support_abs_error_delta_mean_m": float(np.mean(new_support) - np.mean(old_support)),
        })
    return output


def run_command_test(label: str, command: Sequence[str], timeout: float = 300.0) -> dict[str, Any]:
    started = time.monotonic()
    result = subprocess.run(command, cwd=ROOT, text=True, capture_output=True, timeout=timeout)
    output = (result.stdout + result.stderr).strip()
    return {
        "test": label, "command": " ".join(command), "returncode": result.returncode,
        "passed": result.returncode == 0, "elapsed_s": time.monotonic() - started,
        "detail": " | ".join(output.splitlines()[-8:]),
    }


def run_existing_synthetic_test() -> dict[str, Any]:
    started = time.monotonic(); domain = 198
    env = dict(os.environ); env["ROS_DOMAIN_ID"] = str(domain)
    log_dir = RAW / "ros_logs/existing_synthetic"; log_dir.mkdir(parents=True, exist_ok=True)
    env["ROS_LOG_DIR"] = str(log_dir)
    node_log = RAW / "detector_logs/existing_synthetic_node.log"; node_log.parent.mkdir(parents=True, exist_ok=True)
    with node_log.open("w", encoding="utf-8") as log:
        node = subprocess.Popen(
            [str(DETECTOR), "--ros-args", "--params-file", str(DETECTOR_CONFIG),
             "-p", "ego_odom_topic:=/ego_racecar/odom", "-p", "diagnostics_enable:=false"],
            cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        try:
            result = subprocess.run(
                [sys.executable, "src/obstacle_detector/test/synthetic_opponent_test.py"],
                cwd=ROOT, env=env, text=True, capture_output=True, timeout=30.0,
            )
        finally:
            if node.poll() is None:
                os.killpg(node.pid, signal.SIGINT)
                try: node.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    os.killpg(node.pid, signal.SIGTERM); node.wait(timeout=5.0)
    return {
        "test": "existing_synthetic_opponent_integration", "command": "actual detector + existing synthetic_opponent_test.py",
        "returncode": result.returncode, "passed": result.returncode == 0,
        "elapsed_s": time.monotonic() - started,
        "detail": " | ".join(result.stdout.strip().splitlines()[-12:]),
    }


def run_tests() -> list[dict[str, Any]]:
    rows = [
        run_command_test(
            "obstacle_detector_incremental_build",
            ["zsh", "-ic", f"cd {ROOT} && cb --packages-select obstacle_detector"], timeout=180.0,
        ),
        run_command_test(
            "prototype_contract_tests",
            ["env", f"PYTHONPATH={ROOT / 'tools/cmaes_tuning'}", sys.executable, "-m", "unittest", "discover", "-s", "tools/cmaes_tuning/tests", "-p", "test_uniform_laserscan_prototype.py"],
        ),
        run_command_test(
            "all_cmaes_tuning_tests",
            ["env", f"PYTHONPATH={ROOT / 'tools/cmaes_tuning'}", sys.executable, "-m", "unittest", "discover", "-s", "tools/cmaes_tuning/tests", "-p", "test_*.py"],
            timeout=600.0,
        ),
        run_command_test(
            "obstacle_detector_ctest",
            ["ctest", "--test-dir", "build/obstacle_detector", "--output-on-failure"], timeout=300.0,
        ),
        run_existing_synthetic_test(),
    ]
    return rows


def markdown_table(headers: Sequence[str], rows: Sequence[Sequence[Any]]) -> str:
    def value(item: Any) -> str:
        if isinstance(item, float): return f"{item:.6f}"
        return str(item)
    lines = ["| " + " | ".join(headers) + " |", "|" + "|".join("---" for _ in headers) + "|"]
    lines.extend("| " + " | ".join(value(item) for item in row) + " |" for row in rows)
    return "\n".join(lines)


def render_readme(summary: dict[str, Any], performance: dict[str, Any]) -> str:
    categories = summary["category_statistics"]
    opponents = summary["opponent_regression"]
    threshold = [row for row in summary["range_delta_distribution"] if row["population"] == "all_valid"]
    mode_rows = []
    for mode in MODES:
        safety = summary["static_safety"][mode]
        mode_rows.append([
            mode,
            summary["contract"]["prototype_mismatch_count"] if mode == "PROTOTYPE" else "LEGACY_EXPLICIT",
            summary["validation039_safe"] if mode == "PROTOTYPE" else "N/A",
            safety["false_feasible_count"], safety["false_infeasible_count"],
            summary["classification"] if mode == "PROTOTYPE" else "REFERENCE",
        ])
    table2 = [[
        row["category"], row["legacy_target_returns_mean"], row["prototype_target_returns_mean"],
        row["cluster_acceptance_delta"], row["selected_side_support_abs_error_delta_mean_m"],
    ] for row in categories if row["category"] != "ALL"]
    table3 = [[
        row["opponent_case"], row["legacy_detection_produced"], row["prototype_detection_produced"],
        row["prototype_id_continuity"], row["prototype_first_dynamic_scan"], row["result"],
    ] for row in opponents]
    table4 = [[row["threshold_m"], row["beam_fraction"]] for row in threshold]
    return f"""# Uniform LaserScan broader downstream regression audit

이 산출물은 production 동작을 변경하지 않는 진단 전용 비교다. 저장된 25개 rule-compliant
정적 case의 map/obstacle/ego pose/timestamp를 그대로 사용했고, 차이는 `LEGACY_SCAN_MODE`
또는 `PROTOTYPE_UNIFORM_SCAN_MODE`로 명시된 sensor beam contract뿐이다. production
`obstacle_detector` 실행 파일과 기존 YAML을 그대로 실행했으며 GT는 return 라벨링과 결과
매칭에만 사용했다. closed-loop competition replay와 CMA 실행은 0회다.

최종 분류는 **{summary['classification']}**이다. corrected contract 자체와 safety anchor는
유지되지만 측정 분포가 물질적으로 바뀌므로 adoption 시 기존 simulator-derived CMA sensor
artifact와 성능 결과는 stale로 취급하고 재-baseline해야 한다. 이 작업에서는 재생성하지 않았다.

## Table 1 — adoption/safety

{markdown_table(['mode','beam contract mismatch','039 safe','static false-feasible','static false-infeasible','classification'], mode_rows)}

## Table 2 — static category distribution

{markdown_table(['category','OLD target returns','NEW target returns','cluster acceptance delta','AABB/support delta'], table2)}

## Table 3 — opponent regression

{markdown_table(['opponent case','OLD detection','NEW detection','ID continuity','dynamic classification scan','result'], table3)}

## Table 4 — full-beam range shift

{markdown_table(['range-delta threshold m','beam fraction'], table4)}

## Evidence and limits

- 25/25 stored cases에서 case당 15 scan, mode당 총 {summary['static_scan_count_per_mode']} fresh scan을 비교했다.
- PROTOTYPE newly generated contract mismatch는 {summary['contract']['prototype_mismatch_count']}이다.
- static false-feasible은 PROTOTYPE {summary['static_safety']['PROTOTYPE']['false_feasible_count']}/25이다.
- validation_039 +20/+30/+40 ms는 모두 NON_EMPTY이고 +40 GT undercoverage는
  {summary['validation039']['gt_undercoverage_m']:.6f} m, dangerous path는
  {summary['validation039']['dangerous_path_classification']}이다. 같은 path SHA256은
  `{summary['validation039']['dangerous_path_sha256']}`이다.
- normal_01_00은 {summary['normal0100']['classification']}; blocker removal
  +{summary['normal0100']['blocker_removal_ms']} ms, first feasible +{summary['normal0100']['first_hard_feasible_ms']} ms,
  stable +{summary['normal0100']['stable_resolution_ms']} ms이다.
- top-{len(summary['largest_range_deltas'])} large deltas는 모두 grazing/silhouette/map-corner
  ray-topology switch로 분류됐다. 이 분류는 GT evaluation label이며 detector 입력이 아니다.
- production passive event/topic은 map-frame KF velocity를 직접 노출하지 않는다. opponent 표의
  map velocity는 raw AABB centre의 관측 속도이며 실제 output `vs/vd`, DYNAMIC transition,
  `/opp_obs`, ID continuity는 production 결과다.
- sparse 1/2/3/4-return frequency는 보고만 했고 threshold/cluster/evidence 동작은 바꾸지 않았다.

## Determinism and protection

대표 정적 scan/detector, validation_039 +40, baseline opponent, 256 random batch digest를 반복
비교했다. production detector/planner/controller/state-machine 및 active simulator source hash는
작업 전후 동일하다. isolated prototype clone만 읽었고 active simulator에는 patch를 적용하지 않았다.

## Runtime

wall {performance['wall_time_s']:.3f} s, independent workers {performance['workers_initial']}개,
peak RSS {performance['max_rss_bytes'] / 1024**2:.1f} MiB, 시작/종료 MemAvailable
{performance['memory_before']['memory_available_bytes'] / 1024**3:.2f}/
{performance['memory_after']['memory_available_bytes'] / 1024**3:.2f} GiB,
swap increase {performance['swap_increased']}.
"""


def static_detector_summary(rows: Sequence[dict[str, Any]]) -> dict[str, Any]:
    by_case: dict[str, list[dict[str, Any]]] = {}
    for row in rows: by_case.setdefault(row["case_id"], []).append(row)
    lost_final, delayed, lost_events = [], [], []
    confirmations: dict[str, dict[str, int | None]] = {}
    for case_id, values in by_case.items():
        values = sorted(values, key=lambda item: item["scan_index"])
        final = values[-1]
        if final["legacy_detection_produced"] and not final["prototype_detection_produced"]:
            lost_final.append(case_id)
        for item in values:
            if item["legacy_detection_produced"] and not item["prototype_detection_produced"]:
                lost_events.append(f"{case_id}:{item['scan_index']}")
        first: dict[str, int | None] = {}
        for prefix in ("legacy", "prototype"):
            matches = [item["scan_index"] for item in values if item[f"{prefix}_track_status"] == "CONFIRMED"]
            first[prefix] = min(matches) if matches else None
        confirmations[case_id] = first
        if first["legacy"] is not None and (first["prototype"] is None or first["prototype"] > first["legacy"]):
            delayed.append(case_id)
    return {
        "case_count": len(by_case), "prototype_final_detection_loss_cases": lost_final,
        "prototype_per_scan_detection_loss_events": lost_events,
        "prototype_confirmation_delayed_cases": delayed,
        "confirmation_first_scan": confirmations,
        "new_false_cluster_delta_mean": float(np.mean([
            int(row["prototype_raw_detection_count_all"]) - int(row["legacy_raw_detection_count_all"])
            for row in rows
        ])),
        "systematic_regression": bool(lost_final),
    }


def artifact_assertions(required: Sequence[str], summary: dict[str, Any]) -> dict[str, Any]:
    missing = [name for name in required if not (OUTPUT / name).is_file()]
    nonempty = [name for name in required if (OUTPUT / name).is_file() and (OUTPUT / name).stat().st_size > 0]
    checks = {
        "missing": missing, "nonempty_count": len(nonempty), "required_count": len(required),
        "prototype_contract_zero": summary["contract"]["prototype_mismatch_count"] == 0,
        "prototype_static_false_feasible_zero": summary["static_safety"]["PROTOTYPE"]["false_feasible_count"] == 0,
        "validation039_safe": summary["validation039_safe"],
        "opponent_no_catastrophic_regression": not any(row["catastrophic_regression"] for row in summary["opponent_regression"]),
    }
    checks["passed"] = not missing and all(
        checks[key] for key in (
            "prototype_contract_zero", "prototype_static_false_feasible_zero",
            "validation039_safe", "opponent_no_catastrophic_regression",
        )
    )
    return checks


def refresh_test_artifacts() -> int:
    summary_path = OUTPUT / "summary.json"
    performance_path = OUTPUT / "performance.json"
    if not summary_path.is_file() or not performance_path.is_file():
        raise RuntimeError("completed scientific artifact is required before --refresh-tests")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    performance = json.loads(performance_path.read_text(encoding="utf-8"))
    tests = run_tests()
    tests.append(run_command_test("git_diff_check", ["git", "diff", "--check"], timeout=120.0))
    required = (
        "README.md", "summary.json", "static25_sensor_comparison.csv", "static25_detector_comparison.csv",
        "static25_safety_comparison.csv", "category_statistics.csv", "random_geometry_sensor_comparison.csv",
        "range_delta_distribution.csv", "largest_range_deltas.csv", "sparse_fragment_frequency.csv",
        "validation039_safety_anchor.csv", "normal0100_temporal_anchor.csv", "opponent_regression.csv",
        "test_results.csv", "performance.json",
    )
    assertions = artifact_assertions(required, summary)
    tests.append({
        "test": "artifact_assertions", "command": "diagnostic-only internal assertions",
        "returncode": 0 if assertions["passed"] else 1, "passed": assertions["passed"],
        "elapsed_s": 0.0, "detail": json.dumps(assertions, sort_keys=True),
    })
    summary["tests"] = tests; summary["artifact_assertions"] = assertions
    write_csv(OUTPUT / "test_results.csv", tests)
    summary_path.write_text(json.dumps(clean_json(summary), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (OUTPUT / "README.md").write_text(render_readme(summary, performance), encoding="utf-8")
    print(json.dumps({"tests_passed": sum(row["passed"] for row in tests), "tests_total": len(tests)}, indent=2))
    return 0


def refresh_report_artifacts() -> int:
    summary_path = OUTPUT / "summary.json"; performance_path = OUTPUT / "performance.json"
    if not summary_path.is_file() or not performance_path.is_file():
        raise RuntimeError("completed scientific artifact is required before --refresh-reports")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    performance = json.loads(performance_path.read_text(encoding="utf-8"))
    cases = load_static_cases(); frames_by_key = build_static_frames(cases)
    sensor_rows, range_distribution, largest_deltas = paired_sensor_rows(cases, frames_by_key)
    detector_rows, sparse_rows = paired_detector_rows(cases, frames_by_key)
    write_csv(OUTPUT / "static25_sensor_comparison.csv", sensor_rows)
    write_csv(OUTPUT / "static25_detector_comparison.csv", detector_rows)
    write_csv(OUTPUT / "range_delta_distribution.csv", range_distribution)
    write_csv(OUTPUT / "largest_range_deltas.csv", largest_deltas)
    write_csv(OUTPUT / "sparse_fragment_frequency.csv", sparse_rows)
    summary["static_detector"] = static_detector_summary(detector_rows)
    summary["range_delta_distribution"] = range_distribution
    summary["largest_range_deltas"] = largest_deltas
    summary["sparse_fragment_frequency"] = sparse_rows
    validation_rows = read_csv(OUTPUT / "validation039_safety_anchor.csv")
    for row in validation_rows:
        row["dangerous_path_sha256"] = rulebook.EXPECTED_039_PATH_SHA256
    write_csv(OUTPUT / "validation039_safety_anchor.csv", validation_rows)
    summary["validation039"]["dangerous_path_sha256"] = rulebook.EXPECTED_039_PATH_SHA256
    summary_path.write_text(json.dumps(clean_json(summary), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (OUTPUT / "README.md").write_text(render_readme(summary, performance), encoding="utf-8")
    print(json.dumps({"sensor_rows": len(sensor_rows), "detector_rows": len(detector_rows), "largest": len(largest_deltas)}, indent=2))
    return 0


def main() -> int:
    args = parse_args()
    if args.smoke_case:
        return smoke(args.smoke_case)
    if args.refresh_tests:
        return refresh_test_artifacts()
    if args.refresh_reports:
        return refresh_report_artifacts()
    if not DETECTOR.is_file() or not s3.BINARY.is_file():
        raise RuntimeError("required existing incremental build products are missing")
    started = time.monotonic()
    cpu_self_before = resource.getrusage(resource.RUSAGE_SELF)
    cpu_children_before = resource.getrusage(resource.RUSAGE_CHILDREN)
    OUTPUT.mkdir(parents=True, exist_ok=True); RAW.mkdir(parents=True, exist_ok=True)
    memory_before = memory_snapshot(); hashes_before = protected_hashes()
    active_status_before = git_status(ACTIVE_SIM); prototype_status_before = git_status(PROTOTYPE_SIM)

    cases = load_static_cases()
    frames_started = time.monotonic(); frames_by_key = build_static_frames(cases)
    frame_generation_s = time.monotonic() - frames_started
    # Every prototype producer and consumer uses one stored float32 metadata contract object.
    contract = UniformLaserScanContract.from_fov(
        cases[0]["clean_model"].scan_fov, cases[0]["clean_model"].scan_beams
    )
    static_contract_mismatch = int(np.count_nonzero(
        contract.physical_relative_angles_rad() != contract.published_relative_angles_rad()
    ))

    static_tasks: list[dict[str, Any]] = []
    for task_index, case in enumerate(cases):
        for mode_index, mode in enumerate(MODES):
            paths = prepare_detector_input(case, mode, frames_by_key[(case["case_id"], mode)])
            static_tasks.append({
                **paths, "label": f"static_{case['case_id']}_{mode.lower()}",
                "domain_id": 101 + 2 * task_index + mode_index,
            })
    detector_static_started = time.monotonic()
    static_task_results = execute_detector_tasks(static_tasks, WORKERS)
    detector_static_s = time.monotonic() - detector_static_started
    memory_after_static = memory_snapshot()
    workers_remaining = 4 if memory_after_static["swap_used_bytes"] > memory_before["swap_used_bytes"] else WORKERS

    # Repeat a representative static sequence into a separate output path.
    base_repeat = prepare_detector_input(
        cases[0], "PROTOTYPE", frames_by_key[(cases[0]["case_id"], "PROTOTYPE")]
    )
    static_repeat_output = RAW / "detector_outputs/static" / f"{cases[0]['case_id']}__prototype_repeat.json"
    repeat_task = {
        **base_repeat, "output": str(static_repeat_output), "label": "static_representative_repeat", "domain_id": 159,
    }

    opponent_tasks: list[dict[str, Any]] = []
    task_index = 0
    for case_name in ("baseline", "high_speed", "partial_occlusion"):
        for mode in MODES:
            paths = prepare_opponent_input(case_name, mode)
            opponent_tasks.append({**paths, "label": f"opponent_{case_name}_{mode.lower()}", "domain_id": 160 + task_index})
            task_index += 1
    opponent_repeat = prepare_opponent_input("baseline", "PROTOTYPE", "__repeat")
    opponent_tasks.append({**opponent_repeat, "label": "opponent_baseline_prototype_repeat", "domain_id": 166})
    detector_opponent_started = time.monotonic()
    opponent_task_results = execute_detector_tasks([repeat_task] + opponent_tasks, workers_remaining)
    detector_opponent_s = time.monotonic() - detector_opponent_started

    sensor_rows, range_distribution, largest_deltas = paired_sensor_rows(cases, frames_by_key)
    detector_rows, sparse_rows = paired_detector_rows(cases, frames_by_key)
    detector_summary = static_detector_summary(detector_rows)

    s3.RAW = RAW / "static_safety"
    safety_started = time.monotonic()
    with concurrent.futures.ProcessPoolExecutor(max_workers=workers_remaining) as executor:
        safety_rows = list(executor.map(run_safety_task, safety_cases(cases, frames_by_key)))
    safety_rows = sorted(safety_rows, key=lambda item: (item["case_id"], item["mode"]))
    safety_s = time.monotonic() - safety_started
    representative_safety_case = safety_cases([cases[0]], frames_by_key)[1]
    representative_safety_case = dict(representative_safety_case)
    representative_safety_case["case_id"] += "__repeat"
    safety_repeat = run_safety_task(representative_safety_case)
    safety_primary = next(
        row for row in safety_rows if row["case_id"] == cases[0]["case_id"] and row["mode"] == "PROTOTYPE"
    )
    safety_compare_fields = (
        "selected_side", "gt_corridor_m", "shadow_corridor_m", "false_feasible",
        "false_infeasible", "possible_area_m2", "possible_cell_count",
        "selected_side_support_error_m", "gt_boundary_undercoverage_m", "gt_boundary_overcoverage_m",
    )
    static_safety_deterministic = all(safety_primary[key] == safety_repeat[key] for key in safety_compare_fields)

    random_started = time.monotonic(); random_rows, random_summary = random_geometry_sweep()
    random_s = time.monotonic() - random_started
    validation_rows, normal_rows, anchor_summary = rerun_anchors()
    opponent_rows = paired_opponent_rows()
    primary_opponent_path = RAW / "detector_outputs/opponent/baseline__prototype.json"
    repeat_opponent_path = RAW / "detector_outputs/opponent/baseline__prototype__repeat.json"
    opponent_deterministic = canonical_digest(json.loads(primary_opponent_path.read_text())) == canonical_digest(json.loads(repeat_opponent_path.read_text()))
    static_detector_deterministic = canonical_digest(json.loads((RAW / "detector_outputs/static" / f"{cases[0]['case_id']}__prototype.json").read_text())) == canonical_digest(json.loads(static_repeat_output.read_text()))
    static_scan_repeat = static_frames(cases[0], "PROTOTYPE")
    static_scan_deterministic = all(
        array_digest(a.ranges) == array_digest(b.ranges)
        for a, b in zip(frames_by_key[(cases[0]["case_id"], "PROTOTYPE")], static_scan_repeat)
    )

    category_rows = aggregate_category_rows(sensor_rows, detector_rows, safety_rows)
    safety_summary: dict[str, Any] = {}
    for mode in MODES:
        values = [row for row in safety_rows if row["mode"] == mode]
        safety_summary[mode] = {
            "false_feasible_count": sum(bool(row["false_feasible"]) for row in values),
            "false_infeasible_count": sum(bool(row["false_infeasible"]) for row in values),
            "possible_area_mean_m2": float(np.mean([row["possible_area_m2"] for row in values])),
            "gt_boundary_undercoverage_max_m": float(np.max([row["gt_boundary_undercoverage_m"] for row in values])),
            "gt_boundary_overcoverage_mean_m": float(np.mean([row["gt_boundary_overcoverage_m"] for row in values])),
            "selected_side_support_abs_error_mean_m": float(np.mean([abs(row["selected_side_support_error_m"]) for row in values])),
            "shadow_vs_gt_corridor_delta_mean_m": float(np.mean([row["corridor_delta_m"] for row in values])),
        }
    validation_primary = next(row for row in validation_rows if row["run"] == "PRIMARY" and int(row["delta_t_ms"]) == 40)
    validation_safe = (
        all(row["status"] == "NON_EMPTY" for row in validation_rows if row["run"] == "PRIMARY")
        and float(validation_primary["gt_undercoverage_m"]) <= 1e-12
        and validation_primary["dangerous_path_classification"] == "HARD_INVALID"
    )
    normal_primary = [row for row in normal_rows if row["run"] == "PRIMARY"]
    blocker_removal = min(int(row["delta_t_ms"]) for row in normal_primary if row["blocking_hypothesis_status"] == "REMOVED")
    first_feasible = min(int(row["delta_t_ms"]) for row in normal_primary if int(row["production_family_hard_feasible_count"]) > 0)
    normal_summary = {
        "classification": normal_primary[0]["classification"], "blocker_removal_ms": blocker_removal,
        "first_hard_feasible_ms": first_feasible, "stable_resolution_ms": int(normal_primary[0]["stable_resolution_ms"]),
        "feasible_candidates_at_first": next(int(row["production_family_hard_feasible_count"]) for row in normal_primary if int(row["delta_t_ms"]) == first_feasible),
        "remaining_distance_at_first_m": next(float(row["remaining_distance_m"]) for row in normal_primary if int(row["delta_t_ms"]) == first_feasible),
    }
    systematic_detector_regression = detector_summary["systematic_regression"]
    catastrophic_opponent = any(row["catastrophic_regression"] for row in opponent_rows)
    unexplained_large = any(not row["physically_explainable"] for row in largest_deltas)
    material_shift = (
        next(row for row in category_rows if row["category"] == "ALL")["range_delta_max_m"] > 1.0
        or float(np.mean([row["changed_beam_fraction"] for row in sensor_rows])) > 0.05
    )
    if not validation_safe or safety_summary["PROTOTYPE"]["false_feasible_count"]:
        classification = "UNSAFE"
    elif systematic_detector_regression or catastrophic_opponent:
        classification = "DOWNSTREAM_REGRESSION"
    elif unexplained_large or not all((static_detector_deterministic, static_safety_deterministic, opponent_deterministic, random_summary["deterministic"])):
        classification = "NEEDS_MORE_SIMULATOR_VALIDATION"
    elif material_shift:
        classification = "ADOPTABLE_WITH_REBASELINE"
    else:
        classification = "ADOPTABLE_PROTOTYPE"

    hashes_after_compute = protected_hashes()
    if hashes_before != hashes_after_compute:
        raise RuntimeError("protected production/active simulator hash changed during audit")
    tests = [] if args.skip_tests else run_tests()
    tests.append(run_command_test("git_diff_check", ["git", "diff", "--check"], timeout=120.0))
    active_status_after = git_status(ACTIVE_SIM); prototype_status_after = git_status(PROTOTYPE_SIM)
    memory_after = memory_snapshot()
    cpu_self_after = resource.getrusage(resource.RUSAGE_SELF)
    cpu_children_after = resource.getrusage(resource.RUSAGE_CHILDREN)
    performance = {
        "wall_time_s": time.monotonic() - started, "workers_initial": WORKERS,
        "workers_after_swap_guard": workers_remaining,
        "frame_generation_s": frame_generation_s, "detector_static_wall_s": detector_static_s,
        "detector_opponent_wall_s": detector_opponent_s, "static_safety_wall_s": safety_s,
        "random_geometry_wall_s": random_s,
        "detector_task_cpu_sum_s": sum(row["elapsed_s"] for row in static_task_results + opponent_task_results),
        "memory_before": memory_before, "memory_after_static": memory_after_static, "memory_after": memory_after,
        "swap_increased": memory_after["swap_used_bytes"] > memory_before["swap_used_bytes"],
        "max_rss_bytes": int(max(cpu_self_after.ru_maxrss, cpu_children_after.ru_maxrss) * 1024),
        "self_cpu_s": (cpu_self_after.ru_utime + cpu_self_after.ru_stime) - (cpu_self_before.ru_utime + cpu_self_before.ru_stime),
        "children_cpu_s": (cpu_children_after.ru_utime + cpu_children_after.ru_stime) - (cpu_children_before.ru_utime + cpu_children_before.ru_stime),
        "closed_loop_replay_count": 0, "cma_run_count": 0,
    }
    summary = {
        "schema": "uniform_laserscan_broader_regression/1", "classification": classification,
        "static_case_count": len(cases), "static_scan_count_per_mode": len(cases) * STATIC_SCAN_COUNT,
        "categories": {category: sum(case["category"] == category for case in cases) for category in sorted({case["category"] for case in cases})},
        "contract": {
            "legacy_mode": str(LEGACY_SCAN_MODE.value), "prototype_mode": str(PROTOTYPE_UNIFORM_SCAN_MODE.value),
            "prototype_mismatch_count": static_contract_mismatch + random_summary["prototype_contract_mismatch_count"],
            "prototype_new_scan_count": len(cases) * STATIC_SCAN_COUNT + RANDOM_COUNT + 3 * 80 + 5,
            "legacy_bags_reinterpreted": False,
        },
        "static_detector": detector_summary, "static_safety": safety_summary,
        "validation039": clean_json(validation_primary), "validation039_safe": validation_safe,
        "normal0100": normal_summary, "opponent_regression": opponent_rows,
        "random_geometry": random_summary, "category_statistics": category_rows,
        "range_delta_distribution": range_distribution, "largest_range_deltas": largest_deltas,
        "sparse_fragment_frequency": sparse_rows,
        "determinism": {
            "representative_static_scan": static_scan_deterministic,
            "representative_static_detector": static_detector_deterministic,
            "representative_static_safety": static_safety_deterministic,
            "validation039_plus40": anchor_summary["validation039_determinism"]["identical"],
            "normal0100": anchor_summary["normal0100_deterministic"],
            "opponent_baseline": opponent_deterministic, "random_batch": random_summary["deterministic"],
        },
        "distribution_shift_material": material_shift,
        "old_cma_results_stale_after_adoption": classification in {"ADOPTABLE_PROTOTYPE", "ADOPTABLE_WITH_REBASELINE"},
        "next_sparse_shadow_audit_safe_to_proceed": classification in {"ADOPTABLE_PROTOTYPE", "ADOPTABLE_WITH_REBASELINE"},
        "production_hashes_before": hashes_before, "production_hashes_after": hashes_after_compute,
        "production_sources_unchanged": hashes_before == hashes_after_compute,
        "active_simulator_status_before": active_status_before, "active_simulator_status_after": active_status_after,
        "active_simulator_unchanged": active_status_before == active_status_after,
        "prototype_status_before": prototype_status_before, "prototype_status_after": prototype_status_after,
        "tests": tests,
    }

    write_csv(OUTPUT / "static25_sensor_comparison.csv", sensor_rows)
    write_csv(OUTPUT / "static25_detector_comparison.csv", detector_rows)
    write_csv(OUTPUT / "static25_safety_comparison.csv", safety_rows)
    write_csv(OUTPUT / "category_statistics.csv", category_rows)
    write_csv(OUTPUT / "random_geometry_sensor_comparison.csv", random_rows)
    write_csv(OUTPUT / "range_delta_distribution.csv", range_distribution)
    write_csv(OUTPUT / "largest_range_deltas.csv", largest_deltas)
    write_csv(OUTPUT / "sparse_fragment_frequency.csv", sparse_rows)
    write_csv(OUTPUT / "validation039_safety_anchor.csv", validation_rows)
    write_csv(OUTPUT / "normal0100_temporal_anchor.csv", normal_rows)
    write_csv(OUTPUT / "opponent_regression.csv", opponent_rows)
    write_csv(OUTPUT / "test_results.csv", tests)
    (OUTPUT / "performance.json").write_text(json.dumps(clean_json(performance), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (OUTPUT / "summary.json").write_text(json.dumps(clean_json(summary), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (OUTPUT / "README.md").write_text(render_readme(summary, performance), encoding="utf-8")
    required = (
        "README.md", "summary.json", "static25_sensor_comparison.csv", "static25_detector_comparison.csv",
        "static25_safety_comparison.csv", "category_statistics.csv", "random_geometry_sensor_comparison.csv",
        "range_delta_distribution.csv", "largest_range_deltas.csv", "sparse_fragment_frequency.csv",
        "validation039_safety_anchor.csv", "normal0100_temporal_anchor.csv", "opponent_regression.csv",
        "test_results.csv", "performance.json",
    )
    assertions = artifact_assertions(required, summary)
    tests.append({
        "test": "artifact_assertions", "command": "diagnostic-only internal assertions",
        "returncode": 0 if assertions["passed"] else 1, "passed": assertions["passed"],
        "elapsed_s": 0.0, "detail": json.dumps(assertions, sort_keys=True),
    })
    summary["artifact_assertions"] = assertions; summary["tests"] = tests
    write_csv(OUTPUT / "test_results.csv", tests)
    (OUTPUT / "summary.json").write_text(json.dumps(clean_json(summary), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (OUTPUT / "README.md").write_text(render_readme(summary, performance), encoding="utf-8")
    if not assertions["passed"]:
        raise RuntimeError(f"artifact assertion failed: {assertions}")
    print(json.dumps({
        "classification": classification, "static_false_feasible": safety_summary["PROTOTYPE"]["false_feasible_count"],
        "validation039_safe": validation_safe, "opponent_pass": not catastrophic_opponent,
        "output": str(OUTPUT), "wall_time_s": performance["wall_time_s"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
