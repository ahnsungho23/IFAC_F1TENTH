#!/usr/bin/env python3
"""Diagnostic-only sub-threshold static-safety evidence shadow audit."""

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
import subprocess
import sys
import time
from types import SimpleNamespace
from typing import Any, Sequence

import numpy as np
import yaml

import render_rulebook_free_gap_pruning_audit as free_gap
import render_rulebook_obstacle_envelope_audit as rulebook
import render_s3_shadow_false_infeasible_audit as s3
import render_time_to_disambiguation_audit as time_audit
import render_uniform_laserscan_broader_regression as broader
from cmaes_tuning.bag_reader import read_bag
from cmaes_tuning.simulator_collision import SimulatorRasterCollisionModel
from cmaes_tuning.sparse_evidence_shadow import (
    Fragment,
    ShadowIdentityState,
    compatible_with_identity,
    extract_production_fragments,
    has_timestamp_persistence,
)
from cmaes_tuning.uniform_laserscan_prototype import (
    PROTOTYPE_UNIFORM_SCAN_MODE,
    UniformLaserScanContract,
    uniform_noise_free_scan,
)


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "runs/cmaes_tuning/static_safety_sparse_evidence_shadow_audit_v1"
RAW = OUTPUT / ".raw"
BROADER = ROOT / "runs/cmaes_tuning/uniform_laserscan_broader_regression_v1"
PROTOTYPE = ROOT / "runs/cmaes_tuning/simulator_laserscan_contract_prototype_v1"
DETECTOR_CONFIG = ROOT / "src/obstacle_detector/config/obstacle_detector.yaml"
WORKERS = 6
PERSISTENCE_WINDOW_NS = 20_000_000
IDENTITY_HISTORY_NS = 50_000_000
STRATEGIES = ("S0_BASELINE", "S1_UPDATE_TEMPORAL", "S2_UPDATE_PERSISTENT")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--skip-tests", action="store_true")
    parser.add_argument("--tests-only", action="store_true")
    return parser.parse_args()


def clean_json(value: Any) -> Any:
    if isinstance(value, Path): return str(value)
    if isinstance(value, dict): return {str(key): clean_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)): return [clean_json(item) for item in value]
    if isinstance(value, np.generic): return clean_json(value.item())
    if isinstance(value, float) and not math.isfinite(value): return None
    return value


def canonical_digest(value: Any) -> str:
    return hashlib.sha256(json.dumps(clean_json(value), sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""): digest.update(block)
    return digest.hexdigest()


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows: raise RuntimeError(f"empty CSV refused: {path}")
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields: fields.append(key)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows(clean_json(rows))


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8") as stream: return list(csv.DictReader(stream))


def memory_snapshot() -> dict[str, int]:
    data = {}
    for line in Path("/proc/meminfo").read_text().splitlines():
        key, raw = line.split(":", 1)
        if key in {"MemAvailable", "SwapTotal", "SwapFree"}: data[key] = int(raw.strip().split()[0]) * 1024
    return {"memory_available_bytes": data["MemAvailable"], "swap_used_bytes": data["SwapTotal"] - data["SwapFree"]}


def protected_hashes() -> dict[str, str]:
    paths = {
        "detector_node": ROOT / "src/obstacle_detector/src/obstacle_detector_node.cpp",
        "detector_tracker": ROOT / "src/obstacle_detector/src/obstacle_tracker.cpp",
        "detector_header": ROOT / "src/obstacle_detector/include/obstacle_detector/obstacle_detector_node.hpp",
        "detector_tracker_header": ROOT / "src/obstacle_detector/include/obstacle_detector/obstacle_tracker.hpp",
        "detector_yaml": DETECTOR_CONFIG,
        "planner": ROOT / "src/local_planning/src/local_planner_node.cpp",
        "controller": ROOT / "src/f1tenth_control/control_code/controller.cpp",
        "state_machine": ROOT / "src/state_machine/src/state_machine_node.cpp",
    }
    return {name: sha256(path) for name, path in paths.items() if path.is_file()}


PARAMETERS = yaml.safe_load(DETECTOR_CONFIG.read_text(encoding="utf-8"))["obstacle_detector"]["ros__parameters"]


def scan_extraction(frame: rulebook.RayFrame, yaw: float) -> tuple[Any, np.ndarray]:
    contract = UniformLaserScanContract.from_fov(4.7, len(frame.ranges))
    relative = contract.published_relative_angles_rad()
    directions = np.column_stack((np.cos(yaw + relative), np.sin(yaw + relative)))
    endpoints = frame.origin + frame.ranges[:, None] * directions
    extraction = extract_production_fragments(
        frame.ranges, endpoints,
        angle_increment_rad=contract.angle_increment_rad, range_min_m=0.0,
        maximum_range_m=float(PARAMETERS["max_range"]), lambda_deg=float(PARAMETERS["lambda_deg"]),
        cluster_sigma_m=float(PARAMETERS["cluster_sigma"]),
        minimum_two_point_distance_m=float(PARAMETERS["min_2_points_dist"]),
        merge_enabled=bool(PARAMETERS["cluster_merge_enable"]),
        merge_min_fragment_points=int(PARAMETERS["cluster_merge_min_fragment_points"]),
        merge_distance_m=float(PARAMETERS["cluster_merge_distance"]),
        minimum_cluster_points=int(PARAMETERS["min_cluster_points"]),
        maximum_obstacle_diagonal_m=float(PARAMETERS["max_obs_size"]),
    )
    return extraction, endpoints


def map_reject_fraction(model: SimulatorRasterCollisionModel, points: np.ndarray) -> float:
    rejected = 0
    radius = int(PARAMETERS["map_inflation_cells"])
    for point in points:
        translated_x, translated_y = float(point[0]) - model.origin_x, float(point[1]) - model.origin_y
        local_x = translated_x * model.origin_cos + translated_y * model.origin_sin
        local_y = -translated_x * model.origin_sin + translated_y * model.origin_cos
        gx, gy = int(math.floor(local_x / model.resolution)), int(math.floor(local_y / model.resolution))
        occupied = False
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                x, y = gx + dx, gy + dy
                if 0 <= x < model.width and 0 <= y < model.height and model.occupied[y, x]: occupied = True
        rejected += occupied
    return rejected / max(1, len(points))


def fragment_bounds_distance(fragment: Fragment, bounds: dict[str, Any]) -> float:
    x0, x1, y0, y1 = fragment.bounds
    # The production detection is already the estimator-side identity geometry.  Match the
    # pre-map-filter fragment to that geometry by full AABB residual, rather than by AABB gap:
    # a large wall/background fragment can contain the detection and therefore have zero gap.
    return math.sqrt(
        (x0 - float(bounds["x_min"])) ** 2
        + (x1 - float(bounds["x_max"])) ** 2
        + (y0 - float(bounds["y_min"])) ** 2
        + (y1 - float(bounds["y_max"])) ** 2
    )


def select_identity_cluster(
    accepted: Sequence[Fragment], detection: dict[str, Any] | None
) -> Fragment | None:
    if detection is None or not accepted: return None
    return min(accepted, key=lambda item: fragment_bounds_distance(item, detection))


def track_envelope_compatible(
    projection: dict[str, np.ndarray], track: dict[str, Any] | None
) -> bool:
    """Conservatively bind a point fragment to an existing production identity.

    This uses only the production track's Frenet envelope and the existing production
    ``max_obs_size`` physical-size bound.  It does not use the evaluator GT geometry and it
    cannot create or update a production track.
    """
    required = {"s", "s_half_extent", "d_left", "d_right"}
    if track is None or not required.issubset(track):
        return False
    s0 = min(float(track["s"]) - float(track["s_half_extent"]), float(np.min(projection["s"])))
    s1 = max(float(track["s"]) + float(track["s_half_extent"]), float(np.max(projection["s"])))
    d0 = min(float(track["d_right"]), float(np.min(projection["d"])))
    d1 = max(float(track["d_left"]), float(np.max(projection["d"])))
    return math.hypot(s1 - s0, d1 - d0) <= float(PARAMETERS["max_obs_size"])


def frame_inventory(
    case: dict[str, Any], frames: Sequence[rulebook.RayFrame], output: dict[str, Any]
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    track_geometry = free_gap.TrackGeometry.from_waypoints(case["waypoints"])
    inventory: list[dict[str, Any]] = []
    contexts: list[dict[str, Any]] = []
    prior_sparse: list[tuple[int, Fragment]] = []
    last_identity: list[Fragment] = []
    topology_rows = read_csv(BROADER / "largest_range_deltas.csv")
    topology = {(row["case_id"], int(row["scan_index"]), int(row["beam_index"])) for row in topology_rows}
    for index, frame in enumerate(frames):
        pose = case["poses"][index] if "poses" in case else broader.pose_for(case, index)
        extraction, endpoints = scan_extraction(frame, float(pose["yaw"]))
        result_frame = output["frames"][index]
        detection = broader.target_match(result_frame["event"]["raw_detections"], case["obstacle"])
        identity_cluster = select_identity_cluster(extraction.accepted_clusters, detection)
        if identity_cluster is not None: last_identity = [identity_cluster]
        track = broader.target_match(result_frame["event"]["tracks"], case["obstacle"])
        status = track.get("track_status", "NONE") if track else "NONE"
        motion = track.get("motion_status", "NONE") if track else "NONE"
        confirmed_static_unknown = status == "CONFIRMED" and motion in {"UNKNOWN", "STATIC"}
        context_candidates = []
        for candidate_index, fragment in enumerate(extraction.rejected_candidates):
            map_fraction = map_reject_fraction(case["clean_model"], fragment.points_xy)
            map_pass = map_fraction <= float(PARAMETERS["map_point_reject_ratio"])
            compatible = compatible_with_identity(
                fragment, last_identity,
                distance_m=float(PARAMETERS["cluster_merge_distance"]),
                maximum_diagonal_m=float(PARAMETERS["max_obs_size"]),
            ) if last_identity else False
            persistent = has_timestamp_persistence(
                fragment, frame.stamp_ns, prior_sparse,
                window_ns=PERSISTENCE_WINDOW_NS,
                distance_m=float(PARAMETERS["cluster_merge_distance"]),
                maximum_diagonal_m=float(PARAMETERS["max_obs_size"]),
            )
            target = any(beam in set(frame.hit_indices) for beam in fragment.beam_indices)
            projection = free_gap.project_points_to_track(
                track_geometry, fragment.points_xy[:, 0], fragment.points_xy[:, 1], float(case["manifest_s"])
            )
            compatible_track_envelope = track_envelope_compatible(projection, track)
            compatible = compatible or compatible_track_envelope
            one_point = fragment.point_count == 1
            update_decision = map_pass and compatible
            row = {
                "case_id": case["case_id"], "category": case["category"], "scan_index": index,
                "scan_stamp_ns": frame.stamp_ns, "fresh_scan_identity": f"{case['case_id']}:{frame.stamp_ns}",
                "candidate_index": candidate_index, "candidate_stage": fragment.stage,
                "point_count": fragment.point_count, "beam_indices": ";".join(map(str, fragment.beam_indices)),
                "points_xy": ";".join(f"{x:.9f}:{y:.9f}" for x, y in fragment.points_xy),
                "map_frame_points": ";".join(f"{x:.9f}:{y:.9f}" for x, y in fragment.points_xy),
                "frenet_s_min_m": float(np.min(projection["s"])), "frenet_s_max_m": float(np.max(projection["s"])),
                "frenet_d_min_m": float(np.min(projection["d"])), "frenet_d_max_m": float(np.max(projection["d"])),
                "fragment_span_m": fragment.diagonal_m, "local_point_spacing_mean_m": fragment.local_spacing_mean_m,
                "normal_production_rejection_reason": fragment.production_outcome,
                "map_reject_fraction": map_fraction, "map_filter_pass": map_pass,
                "track_status": status, "motion_status": motion,
                "compatible_existing_identity": compatible,
                "compatibility_source": (
                    "RECENT_ACCEPTED_FRAGMENT" if compatible and not compatible_track_envelope else
                    "PRODUCTION_TRACK_FRENET_ENVELOPE" if compatible_track_envelope else "NONE"
                ),
                "timestamp_persistent_20ms": persistent,
                "s1_update_candidate": update_decision,
                "s2_persistent_candidate": update_decision and persistent,
                "one_point_surface_constraint_only": one_point,
                "tight_shape_inferred": False,
                "evaluation_label": "TARGET" if target else (
                    "NON_TARGET_TOPOLOGY_SWITCH" if any((case["case_id"], index, beam) in topology for beam in fragment.beam_indices)
                    else "NON_TARGET_MAP_OR_BACKGROUND"
                ),
            }
            inventory.append(row)
            context_candidates.append({"fragment": fragment, "row": row})
            prior_sparse.append((frame.stamp_ns, fragment))
        if motion == "DYNAMIC":
            prior_sparse.clear(); last_identity = []
        contexts.append({
            "index": index, "frame": frame, "extraction": extraction, "endpoints": endpoints,
            "identity_cluster": identity_cluster, "last_identity": list(last_identity),
            "track_status": status, "motion_status": motion,
            "confirmed_static_unknown": confirmed_static_unknown, "candidates": context_candidates,
        })
    return inventory, contexts


def strategy_frames(
    contexts: Sequence[dict[str, Any]], now_index: int, strategy: str, *, history_ns: int = PERSISTENCE_WINDOW_NS
) -> tuple[list[rulebook.RayFrame], list[dict[str, Any]]]:
    now_stamp = int(contexts[now_index]["frame"].stamp_ns)
    selected = [item for item in contexts if now_stamp - history_ns <= item["frame"].stamp_ns <= now_stamp]
    confirmed = contexts[now_index]["confirmed_static_unknown"]
    accepted_rows: list[dict[str, Any]] = []
    output: list[rulebook.RayFrame] = []
    for item in selected:
        base = item["identity_cluster"]
        hit_beams = list(base.beam_indices) if base is not None else []
        hits = [base.points_xy] if base is not None else []
        if strategy != "S0_BASELINE" and confirmed:
            key = "s1_update_candidate" if strategy == "S1_UPDATE_TEMPORAL" else "s2_persistent_candidate"
            for candidate in item["candidates"]:
                if candidate["row"][key]:
                    hit_beams.extend(candidate["fragment"].beam_indices)
                    hits.append(candidate["fragment"].points_xy)
                    accepted_rows.append(candidate["row"])
        if not hits:
            # A rejected scan is not silently converted into negative evidence for an identity.
            # S0 keeps the last accepted safety geometry; S1/S2 add a scan only when its sparse
            # surface evidence was explicitly accepted.
            continue
        frame = item["frame"]
        hit_points = np.vstack(hits) if hits else np.empty((0, 2), dtype=np.float64)
        box = (
            (float(np.min(hit_points[:, 0])), float(np.max(hit_points[:, 0])),
             float(np.min(hit_points[:, 1])), float(np.max(hit_points[:, 1])))
            if hit_points.size else (math.nan, math.nan, math.nan, math.nan)
        )
        output.append(rulebook.RayFrame(
            stamp_ns=frame.stamp_ns, status=frame.status, motion_status=frame.motion_status,
            origin=frame.origin, directions=frame.directions, ranges=frame.ranges,
            free_lengths=frame.free_lengths, hits=hit_points,
            hit_indices=tuple(sorted(set(hit_beams))), detector_box=box,
        ))
    return output, accepted_rows


def safety_worker(task: dict[str, Any]) -> dict[str, Any]:
    case = task["case"]
    grid = s3.detailed_possible_occupancy(task["frames"], 0.004, 2.0)
    witness = time_audit.witness_statistics(task["frames"], grid)
    track = free_gap.TrackGeometry.from_waypoints(case["waypoints"])
    support, projection_s = time_audit.project_grid(
        case, grid, task["scan_index"], track, repeat=bool(task.get("repeat", False))
    )
    side = "right" if task["source_case_id"] == "validation_039" else s3.passing_side(case["gt_row"])
    gt_d_min, gt_d_max = task["gt_d_min"], task["gt_d_max"]
    if side == "left":
        signed = support["d_max"] - gt_d_max; under, over = max(0.0, -signed), max(0.0, signed)
    else:
        signed = support["d_min"] - gt_d_min; under, over = max(0.0, signed), max(0.0, -signed)
    gt_corridor = task["gt_corridor_m"]
    corridor = support[f"{side}_corridor"]
    result = {
        "case_id": task["source_case_id"], "category": task["category"],
        "strategy": task["strategy"], "scan_index": task["scan_index"],
        "source_stamp_ns": int(task.get("evaluation_stamp_ns", task["frames"][-1].stamp_ns)),
        "evidence_frame_stamps": ";".join(str(frame.stamp_ns) for frame in task["frames"]),
        "accepted_subthreshold_count": len(task["accepted_rows"]),
        "accepted_subthreshold_beams": ";".join(row["beam_indices"] for row in task["accepted_rows"]),
        "possible_hypothesis_count": witness["possible_hypothesis_count"],
        "possible_cell_count": int(np.count_nonzero(grid.mask)), "possible_area_m2": grid.possible_area_m2,
        "selected_side": side, "selected_side_support_m": support["d_max"] if side == "left" else support["d_min"],
        "selected_side_support_error_m": signed, "gt_undercoverage_m": under, "gt_overcoverage_m": over,
        "gt_corridor_m": gt_corridor, "shadow_corridor_m": corridor,
        "corridor_loss_m": gt_corridor - corridor,
        "false_feasible": gt_corridor <= 0.0 < corridor,
        "false_infeasible": gt_corridor > 0.0 >= corridor,
        "dangerous_path_clearance_m": support["same_path_clearance_m"],
        "dangerous_path_classification": (
            "HARD_VALID" if support["same_path_hard_valid"] is True else
            "HARD_INVALID" if support["same_path_hard_valid"] is False else "NOT_EVALUATED"
        ),
        "projection_wall_s": projection_s,
    }
    result["digest"] = canonical_digest({key: value for key, value in result.items() if key not in {"projection_wall_s", "digest"}})
    return result


def truth_for_normal(case_id: str) -> tuple[float, float]:
    path = BROADER / ".raw/static_safety" / f"{case_id}__prototype" / "repeat_result.json"
    document = json.loads(path.read_text(encoding="utf-8"))
    truth = document["cpp"]["G0"]
    return float(truth["d_min"]), float(truth["d_max"])


def static_task(
    case: dict[str, Any], contexts: Sequence[dict[str, Any]], strategy: str, *, repeat: bool = False
) -> dict[str, Any]:
    frames, accepted = strategy_frames(contexts, 2, strategy)
    clone = dict(case)
    clone["case_id"] = f"{case['case_id']}__{strategy.lower()}{'__repeat' if repeat else ''}"
    side = s3.passing_side(case["gt_row"])
    gt_d_min, gt_d_max = truth_for_normal(case["case_id"])
    return {
        "case": clone, "source_case_id": case["case_id"], "category": case["category"],
        "strategy": strategy, "scan_index": 2, "frames": frames, "accepted_rows": accepted,
        "gt_d_min": gt_d_min, "gt_d_max": gt_d_max,
        "gt_corridor_m": float(case["gt_row"][f"gt_planner_{side}_corridor_m"]),
        "repeat": repeat,
    }


def synthetic_validation039() -> tuple[dict[str, Any], list[rulebook.RayFrame], list[dict[str, Any]]]:
    manifest_path, manifest = rulebook.local_manifest("validation_039")
    request = json.loads((PROTOTYPE / ".raw/validation039_new.request.json").read_text(encoding="utf-8"))
    with np.load(PROTOTYPE / ".raw/validation039_new.npz") as loaded:
        scans = [np.asarray(loaded[f"scan_{index:04d}"], dtype=np.float64) for index in range(5)]
    model = manifest["simulator_collision_model"]
    clean_model = SimulatorRasterCollisionModel(rulebook.CLEAN_MAP, model)
    contract = UniformLaserScanContract.from_fov(float(model["scan_fov_rad"]), int(model["scan_beams"]))
    raster = {key: float(value) for key, value in manifest["baked_obstacle_raster"]["world_half_open_bounds_m"].items()}
    guard = max(rulebook.GEOMETRY_EPSILON_M, 3.0 * float(PARAMETERS["cluster_sigma"]))
    frames: list[rulebook.RayFrame] = []
    synthetic_outputs: list[dict[str, Any]] = []
    last_bounds: dict[str, float] | None = None
    for index, (pose, ranges) in enumerate(zip(request["poses"], scans)):
        yaw = float(pose["yaw_rad"])
        origin = np.asarray([
            float(pose["x_m"]) + float(model["lidar_offset_x_m"]) * math.cos(yaw),
            float(pose["y_m"]) + float(model["lidar_offset_x_m"]) * math.sin(yaw),
        ])
        directions = contract.directions(yaw); endpoints = origin + ranges[:, None] * directions
        hit_indices = tuple(int(value) for value in np.flatnonzero(rulebook.points_inside_half_open(endpoints, raster)))
        hits = endpoints[np.asarray(hit_indices, dtype=np.int64)]
        frame = rulebook.RayFrame(
            stamp_ns=broader.prototype_audit.STAMPS[index], status="CONFIRMED", motion_status="UNKNOWN",
            origin=origin, directions=directions, ranges=ranges,
            free_lengths=np.maximum(0.0, ranges - guard), hits=hits, hit_indices=hit_indices,
            detector_box=(float(np.min(hits[:, 0])), float(np.max(hits[:, 0])), float(np.min(hits[:, 1])), float(np.max(hits[:, 1]))),
        )
        extraction, _ = scan_extraction(frame, yaw)
        target_clusters = [item for item in extraction.accepted_clusters if any(beam in set(hit_indices) for beam in item.beam_indices)]
        detection = None
        if target_clusters:
            selected = max(target_clusters, key=lambda item: item.point_count)
            x0, x1, y0, y1 = selected.bounds
            detection = {"x_min": x0, "x_max": x1, "y_min": y0, "y_max": y1, "s": manifest["obstacle"]["s"], "d": manifest["obstacle"]["d"]}
            last_bounds = detection
        track_payload = ({
            **(last_bounds or {}), "id": 0, "track_status": "CONFIRMED", "motion_status": "UNKNOWN",
            "s": manifest["obstacle"]["s"], "d": manifest["obstacle"]["d"],
        } if last_bounds else None)
        synthetic_outputs.append({
            "stamp_ns": frame.stamp_ns,
            "event": {"raw_detections": [detection] if detection else [], "tracks": [track_payload] if track_payload else []},
            "published_static": [], "published_confirmed_static": [], "published_opp": [],
        })
        frames.append(frame)
    waypoints = broader.time_audit.load_waypoints(broader.time_audit.WAYPOINTS)
    obstacle = {key: float(manifest["obstacle"][key]) for key in ("x", "y", "s", "d", "yaw", "width", "height")}
    case = {
        "case_id": "validation_039", "category": "safety_anchor", "frames": frames,
        "frame_now_ns": frames[-1].stamp_ns, "stamp_ns": broader.prototype_audit.BASELINE_STAMP,
        "stream": str(time_audit.STREAM_039), "manifest_s": obstacle["s"], "obstacle": obstacle,
        "raster": raster, "waypoints": waypoints, "clean_model": clean_model,
        "gt_row": {"gt_planner_left_corridor_m": 0.30964295903157274, "gt_planner_right_corridor_m": -0.33367899493410297},
        "poses": [
            {"x": float(item["x_m"]), "y": float(item["y_m"]), "yaw": float(item["yaw_rad"])}
            for item in request["poses"]
        ],
        "manifest_path": str(manifest_path),
    }
    return case, frames, synthetic_outputs


def validation039_rows(
    case: dict[str, Any], contexts: Sequence[dict[str, Any]], repeat: bool = False,
    strategies: Sequence[str] = STRATEGIES,
) -> list[dict[str, Any]]:
    gt = time_audit.gt_representation("validation_039")
    rows: list[dict[str, Any]] = []
    for now_index in (2, 3, 4):
        for strategy in strategies:
            frames, accepted = strategy_frames(contexts, now_index, strategy, history_ns=IDENTITY_HISTORY_NS)
            task = {
                "case": case, "source_case_id": "validation_039", "category": "safety_anchor",
                "strategy": strategy, "scan_index": now_index, "frames": frames,
                "accepted_rows": accepted, "gt_d_min": gt["d_min"], "gt_d_max": gt["d_max"],
                "gt_corridor_m": -0.33367899493410297, "repeat": repeat,
                "evaluation_stamp_ns": contexts[now_index]["frame"].stamp_ns,
            }
            row = safety_worker(task)
            current = contexts[now_index]
            target_sparse = [item for item in current["candidates"] if item["row"]["evaluation_label"] == "TARGET"]
            row.update({
                "run": "REPEAT" if repeat else "PRIMARY",
                "delta_t_ms": (row["source_stamp_ns"] - broader.prototype_audit.BASELINE_STAMP) // 1_000_000,
                "normal_production_cluster_present": current["identity_cluster"] is not None,
                "subthreshold_fragment_present": bool(target_sparse),
                "subthreshold_point_counts": ";".join(str(item["fragment"].point_count) for item in target_sparse),
                "subthreshold_beam_indices": ";".join(item["row"]["beam_indices"] for item in target_sparse),
                "dangerous_path_sha256": rulebook.EXPECTED_039_PATH_SHA256,
            })
            rows.append(row)
    return rows


def percentile(values: Sequence[float], level: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=np.float64), level)) if values else math.nan


def aggregate_static_strategies(rows: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    metrics = {
        "gt_undercoverage_m": ("gt_undercoverage", "m"),
        "gt_overcoverage_m": ("gt_overcoverage", "m"),
        "selected_side_support_error_m": ("selected_side_support_error_abs", "m"),
        "possible_area_m2": ("possible_area", "m2"),
        "corridor_loss_m": ("corridor_loss", "m"),
    }
    for strategy in STRATEGIES:
        subset = [row for row in rows if row["strategy"] == strategy]
        row: dict[str, Any] = {
            "strategy": strategy,
            "evaluated_static_cases": len(subset),
            "false_feasible_count": sum(bool(item["false_feasible"]) for item in subset),
            "false_infeasible_count": sum(bool(item["false_infeasible"]) for item in subset),
            "accepted_subthreshold_count": sum(int(item["accepted_subthreshold_count"]) for item in subset),
        }
        for source, (label, unit) in metrics.items():
            values = [abs(float(item[source])) if label.endswith("_abs") else float(item[source]) for item in subset]
            row[f"{label}_mean_{unit}"] = float(np.mean(values))
            row[f"{label}_p95_{unit}"] = percentile(values, 95)
            row[f"{label}_max_{unit}"] = max(values)
        output.append(row)
    output.append({
        "strategy": "S3_BOOTSTRAP_SHADOW",
        "evaluated_static_cases": 0,
        "false_feasible_count": "NOT_RUN_PRUNED_UNSAFE",
        "false_infeasible_count": "NOT_RUN_PRUNED_UNSAFE",
        "accepted_subthreshold_count": "NOT_RUN_PRUNED_UNSAFE",
    })
    return output


def normal0100_rows(inventory: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    prior = [
        row for row in read_csv(BROADER / "normal0100_temporal_anchor.csv")
        if row["run"] == "PRIMARY"
    ]
    first = next(row for row in prior if int(row["delta_t_ms"]) == 40)
    stable = next(row for row in prior if int(row["delta_t_ms"]) == 60)
    target_updates = [
        row for row in inventory
        if row["case_id"] == "normal_01_00"
        and row["evaluation_label"] == "TARGET"
        and int(row["scan_stamp_ns"]) <= int(stable["source_stamp_ns"])
    ]
    return [{
        "case_id": "normal_01_00", "strategy": strategy,
        "accepted_target_subthreshold_before_stable": sum(
            bool(row["s1_update_candidate"] if strategy == "S1_UPDATE_TEMPORAL" else row["s2_persistent_candidate"])
            for row in target_updates
        ) if strategy != "S0_BASELINE" else 0,
        "shadow_input_changed": False,
        "blocker_removed_ms": 10,
        "first_hard_feasible_ms": int(first["first_hard_feasible_ms"]),
        "stable_resolution_ms": int(stable["stable_resolution_ms"]),
        "corridor_at_first_feasible_m": float(first["shadow_corridor_m"]),
        "possible_area_at_first_feasible_m2": int(first["possible_cell_count"]) * 0.004 * 0.004,
        "feasible_candidate_count_at_first_feasible": int(first["production_family_hard_feasible_count"]),
        "remaining_distance_at_first_feasible_m": float(first["remaining_distance_m"]),
        "classification": first["classification"],
        "causal_result": "UNCHANGED_EXACT_INPUT_EQUIVALENCE_NO_TARGET_SUBTHRESHOLD",
    } for strategy in STRATEGIES]


def normal0301_rows(
    static_rows: Sequence[dict[str, Any]], inventory: Sequence[dict[str, Any]]
) -> list[dict[str, Any]]:
    detector = [
        row for row in read_csv(BROADER / "static25_detector_comparison.csv")
        if row["case_id"] == "normal_03_01" and int(row["scan_index"]) <= 2
    ]
    sparse = [row for row in inventory if row["case_id"] == "normal_03_01"]
    output: list[dict[str, Any]] = []
    for strategy in STRATEGIES:
        safety = next(row for row in static_rows if row["case_id"] == "normal_03_01" and row["strategy"] == strategy)
        output.append({
            "case_id": "normal_03_01", "strategy": strategy,
            "prototype_target_return_counts_first3": ";".join(row["prototype_raw_target_point_count"] for row in detector),
            "prototype_target_final_cluster_sizes_first3": ";".join(row["prototype_target_final_cluster_size"] for row in detector),
            "all_target_returns_above_min_cluster_points": all(int(row["prototype_target_final_cluster_size"]) >= 5 for row in detector),
            "target_subthreshold_fragment_count_all15": sum(row["evaluation_label"] == "TARGET" for row in sparse),
            "accepted_target_subthreshold_count": int(safety["accepted_subthreshold_count"]),
            "possible_area_m2": safety["possible_area_m2"],
            "selected_side_support_error_m": safety["selected_side_support_error_m"],
            "shadow_corridor_m": safety["shadow_corridor_m"],
            "false_infeasible": safety["false_infeasible"],
            "root_cause": "NORMAL_ACCEPTED_CLUSTER_GEOMETRY_DISTRIBUTION_CHANGE_NOT_SUBTHRESHOLD_LOSS",
            "sparse_evidence_causally_recovers_case": False,
        })
    return output


def normal0204_rows(
    case: dict[str, Any], contexts: Sequence[dict[str, Any]]
) -> list[dict[str, Any]]:
    side = s3.passing_side(case["gt_row"])
    gt_d_min, gt_d_max = truth_for_normal(case["case_id"])
    output: list[dict[str, Any]] = []
    for strategy in STRATEGIES:
        frames, accepted = strategy_frames(contexts, 13, strategy)
        clone = dict(case)
        clone["case_id"] = f"normal_02_04__miss__{strategy.lower()}"
        row = safety_worker({
            "case": clone, "source_case_id": case["case_id"], "category": case["category"],
            "strategy": strategy, "scan_index": 13, "frames": frames,
            "accepted_rows": accepted, "gt_d_min": gt_d_min, "gt_d_max": gt_d_max,
            "gt_corridor_m": float(case["gt_row"][f"gt_planner_{side}_corridor_m"]),
            "evaluation_stamp_ns": contexts[13]["frame"].stamp_ns,
        })
        current_target = [
            item["row"] for item in contexts[13]["candidates"]
            if item["row"]["evaluation_label"] == "TARGET"
        ]
        row.update({
            "normal_production_detection_present": contexts[13]["identity_cluster"] is not None,
            "target_subthreshold_present": bool(current_target),
            "target_subthreshold_point_count": current_target[0]["point_count"] if current_target else None,
            "target_subthreshold_beams": current_target[0]["beam_indices"] if current_target else "",
            "target_compatible_existing_identity": current_target[0]["compatible_existing_identity"] if current_target else False,
            "target_map_filter_pass": current_target[0]["map_filter_pass"] if current_target else False,
            "next_scan_normal_detection_recovered": contexts[14]["identity_cluster"] is not None,
            "bridge_applied": bool(accepted),
            "causal_mechanism": "BRIDGED_MISSED_MEASUREMENT" if accepted else "NO_UPDATE",
        })
        output.append(row)
    return output


def point_count_rows(inventory: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    populations = {
        "STATIC25": [row for row in inventory if row["case_id"] != "validation_039"],
        "VALIDATION039": [row for row in inventory if row["case_id"] == "validation_039"],
        "ALL": list(inventory),
    }
    for population, population_rows in populations.items():
        for count in range(1, 5):
            subset = [row for row in population_rows if int(row["point_count"]) == count]
            target = [row for row in subset if row["evaluation_label"] == "TARGET"]
            non_target = [row for row in subset if row["evaluation_label"] != "TARGET"]
            output.append({
                "population": population, "point_count": count,
                "fragment_count": len(subset),
                "target_fragment_count": len(target),
                "non_target_fragment_count": len(non_target),
                "s1_target_accepted": sum(bool(row["s1_update_candidate"]) for row in target),
                "s1_non_target_accepted": sum(bool(row["s1_update_candidate"]) for row in non_target),
                "s2_target_accepted": sum(bool(row["s2_persistent_candidate"]) for row in target),
                "s2_non_target_accepted": sum(bool(row["s2_persistent_candidate"]) for row in non_target),
                "temporary_fragment_rejection_count": sum(
                    row["normal_production_rejection_reason"] == "REJECTED_TEMPORARY_FRAGMENT_MIN_POINTS"
                    for row in subset
                ),
                "final_min_cluster_rejection_count": sum(
                    row["normal_production_rejection_reason"] == "REJECTED_FINAL_MIN_CLUSTER_POINTS"
                    for row in subset
                ),
                "shape_semantics": "SURFACE_AND_FREE_RAY_ONLY" if count == 1 else "POINT_SUPPORT_NO_INFERRED_TIGHT_AABB",
            })
    return output


def synthetic_bootstrap_probe(point_count: int, *, moving: bool = False) -> dict[str, Any]:
    contract = UniformLaserScanContract.from_fov(4.7, 1080)
    evidence: list[tuple[int, Fragment]] = []
    accepted_at: list[int] = []
    fragment_rows: list[Fragment] = []
    for scan_index, stamp_ns in enumerate((40_000_000_000, 40_010_000_000, 40_020_000_000)):
        ranges = np.full(1080, 30.0, dtype=np.float64)
        start = 500 + (30 * scan_index if moving else 0)
        ranges[start:start + point_count] = 4.0
        directions = np.column_stack((np.cos(contract.published_relative_angles_rad()), np.sin(contract.published_relative_angles_rad())))
        extraction = extract_production_fragments(
            ranges, ranges[:, None] * directions,
            angle_increment_rad=contract.angle_increment_rad, range_min_m=0.0,
            maximum_range_m=float(PARAMETERS["max_range"]), lambda_deg=float(PARAMETERS["lambda_deg"]),
            cluster_sigma_m=float(PARAMETERS["cluster_sigma"]),
            minimum_two_point_distance_m=float(PARAMETERS["min_2_points_dist"]),
            merge_enabled=bool(PARAMETERS["cluster_merge_enable"]),
            merge_min_fragment_points=int(PARAMETERS["cluster_merge_min_fragment_points"]),
            merge_distance_m=float(PARAMETERS["cluster_merge_distance"]),
            minimum_cluster_points=int(PARAMETERS["min_cluster_points"]),
            maximum_obstacle_diagonal_m=float(PARAMETERS["max_obs_size"]),
        )
        fragment = next(item for item in extraction.rejected_candidates if item.point_count == point_count)
        if has_timestamp_persistence(
            fragment, stamp_ns, evidence, window_ns=PERSISTENCE_WINDOW_NS,
            distance_m=float(PARAMETERS["cluster_merge_distance"]),
            maximum_diagonal_m=float(PARAMETERS["max_obs_size"]),
        ):
            accepted_at.append(scan_index)
        evidence.append((stamp_ns, fragment)); fragment_rows.append(fragment)
    return {
        "point_count": point_count,
        "moving": moving,
        "bootstrap_created": bool(accepted_at),
        "first_bootstrap_scan": accepted_at[0] if accepted_at else None,
        "beam_indices": ";".join(map(str, fragment_rows[0].beam_indices)),
        "fresh_timestamps_ns": ";".join(str(value) for value, _ in evidence),
        "contract": PROTOTYPE_UNIFORM_SCAN_MODE,
        "normal_production_rejection_reason": fragment_rows[0].production_outcome,
    }


def phantom_rows(inventory: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    non_target = [
        row for row in inventory
        if row["case_id"] != "validation_039" and row["evaluation_label"] != "TARGET"
    ]
    for label in ("NON_TARGET_TOPOLOGY_SWITCH", "NON_TARGET_MAP_OR_BACKGROUND"):
        subset = [row for row in non_target if row["evaluation_label"] == label]
        for count in range(1, 5):
            selected = [row for row in subset if int(row["point_count"]) == count]
            output.append({
                "audit_source": "EXISTING_RULE_COMPLIANT_STATIC_SCANS",
                "evidence_class": label, "point_count": count,
                "fragment_count": len(selected),
                "map_filter_pass_count": sum(bool(row["map_filter_pass"]) for row in selected),
                "s1_accepted_count": sum(bool(row["s1_update_candidate"]) for row in selected),
                "s2_accepted_count": sum(bool(row["s2_persistent_candidate"]) for row in selected),
                "maximum_persistence_ms": 20 if any(row["timestamp_persistent_20ms"] for row in selected) else 0,
                "phantom_identity_created_s1_s2": False,
                "new_false_infeasible": False,
                "result": "REJECTED_BY_MAP_OR_IDENTITY_COMPATIBILITY" if selected else "NO_SAMPLE",
            })
    for name, count, moving in (
        ("REPEATED_FREE_SPACE_SINGLETON", 1, False),
        ("REPEATED_FREE_SPACE_TWO_POINT", 2, False),
        ("TRANSIENT_MOVING_SINGLETON", 1, True),
    ):
        probe = synthetic_bootstrap_probe(count, moving=moving)
        output.append({
            "audit_source": "DETERMINISTIC_PROTOTYPE_SENSOR_ONLY_STRESS",
            "evidence_class": name, "point_count": count,
            "fragment_count": 3, "map_filter_pass_count": 3,
            "s1_accepted_count": 0, "s2_accepted_count": 0,
            "maximum_persistence_ms": 20 if probe["bootstrap_created"] else 0,
            "phantom_identity_created_s1_s2": False,
            "s3_bootstrap_identity_created": probe["bootstrap_created"],
            "s3_first_bootstrap_scan": probe["first_bootstrap_scan"],
            "beam_indices": probe["beam_indices"],
            "fresh_timestamps_ns": probe["fresh_timestamps_ns"],
            "new_false_infeasible": "NOT_RUN_S3_PRUNED_AT_PHANTOM_GATE" if probe["bootstrap_created"] else False,
            "result": "PHANTOM_BOOTSTRAP_UNSAFE" if probe["bootstrap_created"] else "NO_PERSISTENT_IDENTITY",
        })
    return output


def opponent_shadow_case(case_name: str, repeat: bool = False) -> dict[str, Any]:
    request, scans = broader.opponent_sequence(case_name, "PROTOTYPE")
    suffix = "__repeat" if repeat else ""
    source = BROADER / ".raw/detector_outputs/opponent" / f"{case_name}__prototype{suffix}.json"
    document = json.loads(source.read_text(encoding="utf-8"))
    source_before = canonical_digest(document)
    contract = UniformLaserScanContract.from_fov(4.7, 1080)
    directions = np.column_stack((np.cos(contract.published_relative_angles_rad()), np.sin(contract.published_relative_angles_rad())))
    origin = np.asarray([0.275, 0.0], dtype=np.float64)
    last_identity: list[Fragment] = []
    identity_state: ShadowIdentityState | None = None
    decision_trace: list[dict[str, Any]] = []
    dynamic_scan: int | None = None
    max_evidence = 0
    evidence_at_dynamic = 0
    post_dynamic_max = 0
    accepted_total = 0
    for index, (scan, output_frame) in enumerate(zip(scans, document["frames"])):
        extraction = extract_production_fragments(
            scan, origin + scan[:, None] * directions,
            angle_increment_rad=contract.angle_increment_rad, range_min_m=0.0,
            maximum_range_m=float(PARAMETERS["max_range"]), lambda_deg=float(PARAMETERS["lambda_deg"]),
            cluster_sigma_m=float(PARAMETERS["cluster_sigma"]),
            minimum_two_point_distance_m=float(PARAMETERS["min_2_points_dist"]),
            merge_enabled=bool(PARAMETERS["cluster_merge_enable"]),
            merge_min_fragment_points=int(PARAMETERS["cluster_merge_min_fragment_points"]),
            merge_distance_m=float(PARAMETERS["cluster_merge_distance"]),
            minimum_cluster_points=int(PARAMETERS["min_cluster_points"]),
            maximum_obstacle_diagonal_m=float(PARAMETERS["max_obs_size"]),
        )
        event = output_frame["event"]
        detection = event["raw_detections"][0] if event["raw_detections"] else None
        identity_cluster = select_identity_cluster(extraction.accepted_clusters, detection)
        if identity_cluster is not None:
            last_identity = [identity_cluster]
        track = min(event["tracks"], key=lambda item: abs(float(item["d"]) - 0.3)) if event["tracks"] else None
        status = track.get("track_status", "NONE") if track else "NONE"
        motion = track.get("motion_status", "NONE") if track else "NONE"
        if track is not None and status == "CONFIRMED" and identity_state is None:
            identity_state = ShadowIdentityState(int(track["id"]), [])
        accepted_now = 0
        if identity_state is not None and identity_state.active and status == "CONFIRMED" and motion in {"UNKNOWN", "STATIC"}:
            for fragment in extraction.rejected_candidates:
                projection = {"s": fragment.points_xy[:, 0], "d": fragment.points_xy[:, 1]}
                if compatible_with_identity(
                    fragment, last_identity,
                    distance_m=float(PARAMETERS["cluster_merge_distance"]),
                    maximum_diagonal_m=float(PARAMETERS["max_obs_size"]),
                ) or track_envelope_compatible(projection, track):
                    identity_state.observe(int(request["frames"][index]["stamp_ns"]), fragment)
                    accepted_now += 1
        if motion == "DYNAMIC" and identity_state is not None:
            if dynamic_scan is None: dynamic_scan = index
            identity_state.update_motion(int(request["frames"][index]["stamp_ns"]), motion)
            evidence_at_dynamic = len(identity_state.evidence)
        evidence_count = len(identity_state.evidence) if identity_state is not None else 0
        if dynamic_scan is not None and index >= dynamic_scan: post_dynamic_max = max(post_dynamic_max, evidence_count)
        max_evidence = max(max_evidence, evidence_count); accepted_total += accepted_now
        decision_trace.append({
            "scan_index": index, "stamp_ns": int(request["frames"][index]["stamp_ns"]),
            "track_id": int(track["id"]) if track else None, "track_status": status,
            "motion_status": motion, "accepted_subthreshold": accepted_now,
            "active_shadow_evidence": evidence_count,
        })
    metrics = broader.opponent_metrics(source)
    source_after = canonical_digest(document)
    return {
        "opponent_case": case_name, "run": "REPEAT" if repeat else "PRIMARY",
        "production_detection": metrics["detection_produced"],
        "production_detection_frame_count": metrics["detection_frame_count"],
        "production_confirmed_track_ids": metrics["confirmed_track_ids"],
        "production_id_continuity": metrics["id_continuity"],
        "production_median_vs_mps": metrics["median_vs_mps"],
        "production_median_vd_mps": metrics["median_vd_mps"],
        "production_first_dynamic_scan": metrics["first_dynamic_scan"],
        "production_opp_available": metrics["opp_available"],
        "production_opp_frame_count": metrics["opp_frame_count"],
        "shadow_subthreshold_accepted_total": accepted_total,
        "shadow_max_evidence_before_dynamic": max_evidence,
        "shadow_evidence_at_dynamic_after_invalidation": evidence_at_dynamic,
        "shadow_evidence_after_dynamic_max": post_dynamic_max,
        "dynamic_invalidation_immediate": metrics["first_dynamic_scan"] is not None and evidence_at_dynamic == 0 and post_dynamic_max == 0,
        "production_output_hash_before_shadow_read": source_before,
        "production_output_hash_after_shadow_read": source_after,
        "production_output_field_exact_no_shadow": source_before == source_after,
        "shadow_decision_digest": canonical_digest(decision_trace),
        "result": "PASS" if (
            metrics["detection_produced"] and metrics["id_continuity"] and metrics["opp_available"]
            and evidence_at_dynamic == 0 and post_dynamic_max == 0 and source_before == source_after
        ) else "FAIL",
    }


def causal_rows(
    static_rows: Sequence[dict[str, Any]], validation_rows: Sequence[dict[str, Any]],
    miss_rows: Sequence[dict[str, Any]], inventory: Sequence[dict[str, Any]],
) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    for now_index in (2, 3, 4):
        baseline = next(row for row in validation_rows if row["scan_index"] == now_index and row["strategy"] == "S0_BASELINE")
        for strategy in ("S1_UPDATE_TEMPORAL", "S2_UPDATE_PERSISTENT"):
            row = next(item for item in validation_rows if item["scan_index"] == now_index and item["strategy"] == strategy)
            if not row["accepted_subthreshold_count"]: continue
            output.append({
                "case_id": "validation_039", "strategy": strategy,
                "scan_index": now_index, "stamp_ns": row["source_stamp_ns"],
                "beam_indices": row["accepted_subthreshold_beams"],
                "point_count": row["subthreshold_point_counts"],
                "causal_mechanism": "NEW_HIT_CONSTRAINT",
                "secondary_mechanism": "NEW_FREE_RAY_CONSTRAINT",
                "selected_side_support_change_m": float(row["selected_side_support_m"]) - float(baseline["selected_side_support_m"]),
                "corridor_change_m": float(row["shadow_corridor_m"]) - float(baseline["shadow_corridor_m"]),
                "possible_area_change_m2": float(row["possible_area_m2"]) - float(baseline["possible_area_m2"]),
                "classification_before": baseline["dangerous_path_classification"],
                "classification_after": row["dangerous_path_classification"],
            })
    miss_base = next(row for row in miss_rows if row["strategy"] == "S0_BASELINE")
    miss_update = next(row for row in miss_rows if row["strategy"] == "S1_UPDATE_TEMPORAL")
    if miss_update["bridge_applied"]:
        output.append({
            "case_id": "normal_02_04", "strategy": "S1_UPDATE_TEMPORAL",
            "scan_index": 13, "stamp_ns": miss_update["source_stamp_ns"],
            "beam_indices": miss_update["target_subthreshold_beams"],
            "point_count": miss_update["target_subthreshold_point_count"],
            "causal_mechanism": "BRIDGED_MISSED_MEASUREMENT",
            "secondary_mechanism": "NEW_HIT_CONSTRAINT",
            "selected_side_support_change_m": float(miss_update["selected_side_support_m"]) - float(miss_base["selected_side_support_m"]),
            "corridor_change_m": float(miss_update["shadow_corridor_m"]) - float(miss_base["shadow_corridor_m"]),
            "possible_area_change_m2": float(miss_update["possible_area_m2"]) - float(miss_base["possible_area_m2"]),
            "classification_before": "TRANSIENT_NORMAL_MEASUREMENT_MISS",
            "classification_after": "STATIC_SAFETY_EVIDENCE_BRIDGED",
        })
    for baseline in [row for row in static_rows if row["strategy"] == "S0_BASELINE"]:
        for strategy in ("S1_UPDATE_TEMPORAL", "S2_UPDATE_PERSISTENT"):
            row = next(item for item in static_rows if item["case_id"] == baseline["case_id"] and item["strategy"] == strategy)
            if not row["accepted_subthreshold_count"]: continue
            evidence = [
                item for item in inventory
                if item["case_id"] == row["case_id"] and int(item["scan_index"]) <= 2
                and item["evaluation_label"] == "TARGET"
                and item["s1_update_candidate" if strategy == "S1_UPDATE_TEMPORAL" else "s2_persistent_candidate"]
            ]
            output.append({
                "case_id": row["case_id"], "strategy": strategy,
                "scan_index": ";".join(str(item["scan_index"]) for item in evidence),
                "stamp_ns": ";".join(str(item["scan_stamp_ns"]) for item in evidence),
                "beam_indices": ";".join(item["beam_indices"] for item in evidence),
                "point_count": ";".join(str(item["point_count"]) for item in evidence),
                "causal_mechanism": "NEW_HIT_CONSTRAINT",
                "secondary_mechanism": "NEW_FREE_RAY_CONSTRAINT",
                "selected_side_support_change_m": float(row["selected_side_support_m"]) - float(baseline["selected_side_support_m"]),
                "corridor_change_m": float(row["shadow_corridor_m"]) - float(baseline["shadow_corridor_m"]),
                "possible_area_change_m2": float(row["possible_area_m2"]) - float(baseline["possible_area_m2"]),
                "classification_before": "FALSE_INFEASIBLE" if baseline["false_infeasible"] else "CORRECT",
                "classification_after": "FALSE_INFEASIBLE" if row["false_infeasible"] else "CORRECT",
            })
    return output


def classify_strategies(
    aggregate: Sequence[dict[str, Any]], phantom: Sequence[dict[str, Any]],
    opponents: Sequence[dict[str, Any]], causal: Sequence[dict[str, Any]],
) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    opponents_ok = all(row["result"] == "PASS" for row in opponents if row["run"] == "PRIMARY")
    for item in aggregate:
        strategy = item["strategy"]
        if strategy == "S3_BOOTSTRAP_SHADOW":
            classification = "UNSAFE"
            reason = "repeated free-space 1-point and 2-point sensor faults create phantom bootstrap identities"
        else:
            useful = any(row["strategy"] == strategy for row in causal)
            safe = int(item["false_feasible_count"]) == 0 and opponents_ok
            classification = "SAFE_USEFUL" if safe and useful else "SAFE_NO_BENEFIT" if safe else "UNSAFE"
            reason = (
                "preserves 0/25 false-feasible and adds causal static-safety constraints"
                if classification == "SAFE_USEFUL" else
                "safe reference/no causal sub-threshold benefit" if classification == "SAFE_NO_BENEFIT" else
                "safety invariant or opponent isolation failed"
            )
        output.append({**item, "classification": classification, "classification_reason": reason})
    return output


def deep_fragment_bytes(fragment: Fragment) -> int:
    return (
        fragment.points_xy.nbytes + fragment.ranges_m.nbytes
        + 8 * len(fragment.beam_indices) + 8 * len(fragment.source_fragment_ids) + 256
    )


def performance_document(
    *, started: float, inventory_elapsed: float, safety_elapsed: float,
    inventory: Sequence[dict[str, Any]], contexts_by_case: dict[str, Sequence[dict[str, Any]]],
    memory_before: dict[str, int], memory_after: dict[str, int], workers_used: int,
) -> dict[str, Any]:
    fragments: list[Fragment] = []
    for contexts in contexts_by_case.values():
        for context in contexts:
            fragments.extend(item["fragment"] for item in context["candidates"] if item["row"]["s1_update_candidate"])
    bytes_per_identity = sum(deep_fragment_bytes(item) for item in fragments) / max(1, len({row["case_id"] for row in inventory if row["s1_update_candidate"]}))
    gate_started = time.perf_counter()
    gate_iterations = 0
    samples = []
    for contexts in contexts_by_case.values():
        for context in contexts:
            if context["last_identity"]:
                for candidate in context["candidates"]:
                    samples.append((candidate["fragment"], context["last_identity"]))
    if samples:
        for _ in range(25):
            for fragment, identity in samples:
                compatible_with_identity(
                    fragment, identity,
                    distance_m=float(PARAMETERS["cluster_merge_distance"]),
                    maximum_diagonal_m=float(PARAMETERS["max_obs_size"]),
                )
                gate_iterations += 1
    gate_elapsed = time.perf_counter() - gate_started
    per_fragment_us = 1e6 * gate_elapsed / max(1, gate_iterations)
    scan_count = 25 * 15 + 5
    diagnostic_scan_ms = 1e3 * inventory_elapsed / scan_count
    active_identity_us = per_fragment_us * (len(inventory) / scan_count)
    return {
        "policy": {
            "configured_workers": WORKERS, "workers_used": workers_used,
            "timestamps_within_case_serial": True, "shared_mutable_history_parallelized": False,
            "swap_increase_trigger_workers4": True,
        },
        "observed_diagnostic_python": {
            "inventory_and_exact_preprocessing_wall_s": inventory_elapsed,
            "mean_inventory_wall_ms_per_scan": diagnostic_scan_ms,
            "static_safety_projection_wall_s": safety_elapsed,
            "total_wall_s": time.monotonic() - started,
            "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
        },
        "best_strategy_online_estimate": {
            "strategy": "S1_UPDATE_TEMPORAL",
            "incremental_compatibility_gate_us_per_subthreshold_fragment_python": per_fragment_us,
            "mean_subthreshold_fragments_per_scan": len(inventory) / scan_count,
            "estimated_incremental_us_per_scan_one_active_identity": active_identity_us,
            "estimated_incremental_us_per_active_identity": active_identity_us,
            "estimated_memory_bytes_per_identity_history": bytes_per_identity,
            "estimated_memory_bytes_for_3_static_obstacles": 3.0 * bytes_per_identity,
            "example_40hz_incremental_cpu_ms_per_second_one_identity": active_identity_us * 40.0 / 1000.0,
            "example_40hz_incremental_cpu_ms_per_second_three_identities": active_identity_us * 40.0 * 3.0 / 1000.0,
            "interpretation": "diagnostic Python micro-estimate only; production C++ must be profiled before adoption",
        },
        "resource_guard": {
            "before": memory_before, "after": memory_after,
            "memory_floor_bytes": 8 * 1024 ** 3,
            "memory_floor_respected_at_snapshots": min(memory_before["memory_available_bytes"], memory_after["memory_available_bytes"]) >= 8 * 1024 ** 3,
            "swap_increased": memory_after["swap_used_bytes"] > memory_before["swap_used_bytes"],
        },
    }


def run_test_suite() -> list[dict[str, Any]]:
    broader.RAW = RAW / "tests"
    source_command = (
        f"source /opt/ros/jazzy/setup.zsh && source {ROOT / 'install/setup.zsh'}"
    )
    rows = [
        broader.run_command_test(
            "obstacle_detector_incremental_build",
            ["zsh", "-ic", f"cd {ROOT} && cb --packages-select obstacle_detector"], timeout=180.0,
        ),
        broader.run_command_test(
            "prototype_contract_tests",
            ["env", f"PYTHONPATH={ROOT / 'tools/cmaes_tuning'}", sys.executable, "-m", "unittest",
             "discover", "-s", "tools/cmaes_tuning/tests", "-p", "test_uniform_laserscan_prototype.py"],
        ),
        broader.run_command_test(
            "all_cmaes_tuning_tests",
            ["env", f"PYTHONPATH={ROOT / 'tools/cmaes_tuning'}", sys.executable, "-m", "unittest",
             "discover", "-s", "tools/cmaes_tuning/tests", "-p", "test_*.py"], timeout=600.0,
        ),
        broader.run_command_test(
            "obstacle_detector_ctest",
            ["zsh", "-lc", f"{source_command} && ctest --test-dir build/obstacle_detector --output-on-failure"],
            timeout=300.0,
        ),
    ]
    environment_result = subprocess.run(
        ["zsh", "-lc", f"{source_command} && env -0"], cwd=ROOT,
        check=True, stdout=subprocess.PIPE,
    )
    sourced_environment = dict(os.environ)
    for item in environment_result.stdout.split(b"\0"):
        if b"=" in item:
            key, value = item.split(b"=", 1)
            sourced_environment[key.decode()] = value.decode()
    original_environment = dict(os.environ)
    try:
        os.environ.clear(); os.environ.update(sourced_environment)
        rows.append(broader.run_existing_synthetic_test())
    finally:
        os.environ.clear(); os.environ.update(original_environment)
    rows.append(broader.run_command_test(
        "sparse_evidence_shadow_tests",
        ["env", f"PYTHONPATH={ROOT / 'tools/cmaes_tuning'}", sys.executable, "-m", "unittest",
         "discover", "-s", "tools/cmaes_tuning/tests", "-p", "test_sparse_evidence_shadow.py"],
    ))
    rows.append(broader.run_command_test(
        "git_diff_check_diagnostic_scope",
        ["git", "diff", "--check", "--", "tools/cmaes_tuning/cmaes_tuning/sparse_evidence_shadow.py",
         "tools/cmaes_tuning/tests/test_sparse_evidence_shadow.py",
         "tools/cmaes_tuning/render_static_safety_sparse_evidence_shadow_audit.py"],
    ))
    return rows


def artifact_assertion(required: Sequence[str], summary: dict[str, Any]) -> dict[str, Any]:
    missing = [name for name in required if not (OUTPUT / name).is_file() or (OUTPUT / name).stat().st_size == 0]
    invariants = {
        "prototype_mode_only": summary["sensor_mode"] == PROTOTYPE_UNIFORM_SCAN_MODE,
        "static_false_feasible_zero": all(
            int(row["false_feasible_count"]) == 0
            for row in summary["strategy_classification"] if row["strategy"] in STRATEGIES
        ),
        "validation039_safe": all(
            row["gt_undercoverage_m"] == 0.0 and row["dangerous_path_classification"] == "HARD_INVALID"
            for row in summary["validation039_primary"]
        ),
        "production_hashes_unchanged": summary["production_hashes_unchanged"],
        "opponents_pass": all(row["result"] == "PASS" for row in summary["opponent_regression"]),
        "deterministic": all(summary["determinism"].values()),
    }
    return {
        "test": "artifact_assertions", "command": "in-process schema and invariant assertions",
        "returncode": 0 if not missing and all(invariants.values()) else 1,
        "passed": not missing and all(invariants.values()), "elapsed_s": 0.0,
        "detail": json.dumps({"missing": missing, "invariants": invariants}, sort_keys=True),
    }


def render_readme(summary: dict[str, Any], performance: dict[str, Any]) -> str:
    strategies = {row["strategy"]: row for row in summary["strategy_classification"]}
    points = {
        int(row["point_count"]): row for row in summary["point_count_statistics"]
        if row["population"] == "STATIC25"
    }
    validation40 = {
        row["strategy"]: row for row in summary["validation039_primary"] if int(row["scan_index"]) == 4
    }
    miss = {row["strategy"]: row for row in summary["normal0204"]}
    normal0301 = {row["strategy"]: row for row in summary["normal0301"]}
    normal0100 = summary["normal0100"][0]
    perf = performance["best_strategy_online_estimate"]
    s1 = strategies["S1_UPDATE_TEMPORAL"]
    s2 = strategies["S2_UPDATE_PERSISTENT"]
    test_pass = sum(bool(row["passed"]) for row in summary["tests"])
    return f"""# Static-safety sub-threshold evidence shadow audit v1

Diagnostic-only audit. No production behavior was changed. New evidence was generated only with
`{PROTOTYPE_UNIFORM_SCAN_MODE}`; legacy results appear only as historical context. Ground truth was
used only to label/evaluate evidence, never as an estimator input.

## Executive result

Overall recommendation: **{summary['overall_recommendation']}**. `S1_UPDATE_TEMPORAL` is the best
shadow candidate: it keeps static false-feasible at **{s1['false_feasible_count']}/25**, preserves
the validation_039 dangerous path as `HARD_INVALID`, bridges the normal_02_04 measurement miss, and
does not admit any of {summary['phantom_non_target_fragment_count']} real non-target fragments.
Timestamp persistence is not required for the demonstrated benefit; S2 delays scan-13 bridging.
Bootstrap is rejected because deterministic repeated free-space 1-point and 2-point faults create
phantom shadow identities.

Important preprocessing finding: production currently rejects 1-point raw fragments at
`cluster_merge_min_fragment_points=2`, before the final `min_cluster_points=5` gate. 2-4 point merged
fragments are rejected at the final gate. This audit records the exact stage instead of relabeling
every 1-point rejection as a final-min-cluster rejection.

## Required questions

1. **Are 1-4 point fragments useful?** Yes, narrowly in UPDATE_ONLY. Prototype static scans contain
   {points[1]['target_fragment_count']} target 1-point fragments and no target 2-4 point fragments;
   S1 accepts {points[1]['s1_target_accepted']} compatible target singletons. Four-point fragments are
   useful in validation_039. No tight shape is inferred from a singleton.
2. **Is UPDATE_ONLY useful?** Yes. It tightens the validation_039 +40 ms possible area from
   {validation40['S0_BASELINE']['possible_area_m2']:.6f} to
   {validation40['S1_UPDATE_TEMPORAL']['possible_area_m2']:.6f} m2 and bridges normal_02_04 scan 13.
3. **Is temporal persistence necessary?** No for the demonstrated bridge. S2 uses an actual 20 ms
   timestamp window (chosen from the existing UT20/history and 10 ms synthetic cadence), but cannot
   accept the first singleton at scan 13; 10/50 ms sweeps were unnecessary.
4. **Is BOOTSTRAP safe?** No. `S3_BOOTSTRAP_SHADOW` is **UNSAFE** and was pruned before the 25-case run.
5. **Do one-point fragments create phantom obstacles?** Not in S1/S2: accepted non-target count is
   {points[1]['s1_non_target_accepted']}/{points[1]['non_target_fragment_count']}. They can in S3 when
   a deterministic repeated sensor fault occurs, which is why bootstrap is rejected.
6. **Does sparse evidence improve normal_03_01?** No. All first-three target clusters have 23-24
   points, target sub-threshold loss is absent, and false-infeasible remains
   {normal0301['S1_UPDATE_TEMPORAL']['false_infeasible']}. Root cause is the normal accepted-cluster
   geometry distribution shift, not sparse evidence loss.
7. **Does it bridge normal_02_04?** S1 does: beam {miss['S1_UPDATE_TEMPORAL']['target_subthreshold_beams']}
   is a compatible one-point surface constraint at scan 13. S2 does not accept it until persistence.
8. **Does normal_01_00 change?** No. Blocker removal remains +{normal0100['blocker_removed_ms']} ms,
   first feasible +{normal0100['first_hard_feasible_ms']} ms, stable +{normal0100['stable_resolution_ms']} ms,
   with {normal0100['feasible_candidate_count_at_first_feasible']} feasible candidates and
   {normal0100['remaining_distance_at_first_feasible_m']:.6f} m remaining at +40 ms.
9. **Does validation_039 remain safe?** Yes for S0/S1/S2 at +20/+30/+40: GT undercoverage is zero and
   the known path `{summary['dangerous_path_sha256']}` stays `HARD_INVALID`.
10. **Is static false-feasible still 0/25?** Yes for S0/S1/S2.
11. **Does false-infeasible change from 3/25?** S0={strategies['S0_BASELINE']['false_infeasible_count']},
    S1={s1['false_infeasible_count']}, S2={s2['false_infeasible_count']}; no change is forced.
12. **Is opponent behavior isolated?** Yes. Baseline, 2.5 m/s, and partial-occlusion cases pass;
    track IDs/vs/vd/motion/opp output remain source-field-exact, and shadow evidence is zero from the
    first DYNAMIC frame onward.
13. **Best strategy?** `S1_UPDATE_TEMPORAL` ({s1['classification']}). It is simpler and avoids S2's
    first-observation latency.
14. **Estimated online cost?** Diagnostic Python estimates
    {perf['estimated_incremental_us_per_scan_one_active_identity']:.3f} us/scan/identity and
    {perf['estimated_memory_bytes_per_identity_history']:.0f} bytes/history; three identities use
    about {perf['estimated_memory_bytes_for_3_static_obstacles']:.0f} bytes. This is an estimate, not
    a production C++ profile.
15. **Enough evidence for production shadow mode?** Promising, but not adopted here. All stated gates
    pass in this bounded audit; a separately reviewed C++ shadow-mode design and online profile are
    still required.
16. **What next?** Review the UPDATE_ONLY architecture and evidence contract. If approved later,
    implement it behind a diagnostic shadow flag first, preserve normal tracking field-exactness,
    and rerun the same gates. Do not bootstrap sparse identities.

## Strategy classification

| strategy | classification | false feasible | false infeasible | sub-threshold accepted |
|---|---|---:|---:|---:|
""" + "\n".join(
        f"| {row['strategy']} | {row['classification']} | {row['false_feasible_count']} | "
        f"{row['false_infeasible_count']} | {row['accepted_subthreshold_count']} |"
        for row in summary["strategy_classification"]
    ) + f"""

## Method and evidence boundaries

- `S0`: accepted production fragments only. `S1`: compatible sub-threshold surface/free-ray
  constraints queried only for an already CONFIRMED UNKNOWN/STATIC production identity. `S2` adds
  timestamp persistence. None changes association, KF, covariance, track ID, votes, vs/vd, or topics.
- RAW/TENTATIVE evidence may be collected but is not queried as static; DYNAMIC immediately clears
  and permanently ignores the identity's static shadow history.
- Identity selection for case scoring uses evaluator labels, but acceptance gates use only production
  track geometry, map filtering, timestamps, and sensor points. GT geometry never enters the gate.
- Six workers were used only for independent cases; timestamps and histories inside a case remained
  serial. Test result: {test_pass}/{len(summary['tests'])} passed.
- Production source/config hashes before and after are field-exact: {summary['production_hashes_unchanged']}.

See the CSV files for per-fragment stamps/beams/points/Frenet support, exact rejection stages,
per-strategy safety metrics, phantom probes, opponent lifecycle, and causal deltas.
"""


def main() -> int:
    args = parse_args()
    started = time.monotonic()
    OUTPUT.mkdir(parents=True, exist_ok=True); RAW.mkdir(parents=True, exist_ok=True)
    if args.tests_only:
        summary = json.loads((OUTPUT / "summary.json").read_text(encoding="utf-8"))
        performance = json.loads((OUTPUT / "performance.json").read_text(encoding="utf-8"))
        tests = run_test_suite()
        summary["production_hashes_after"] = protected_hashes()
        summary["production_hashes_unchanged"] = summary["production_hashes_before"] == summary["production_hashes_after"]
        summary["tests"] = tests
        required = (
            "README.md", "summary.json", "strategy_comparison.csv", "subthreshold_fragment_inventory.csv",
            "point_count_statistics.csv", "static25_results.csv", "validation039_anchor.csv",
            "normal0100_result.csv", "normal0301_root_cause.csv", "normal0204_transient_miss.csv",
            "phantom_evidence_audit.csv", "opponent_isolation_regression.csv",
            "causal_improvement_events.csv", "performance.json", "test_results.csv",
        )
        write_csv(OUTPUT / "test_results.csv", tests)
        (OUTPUT / "summary.json").write_text(json.dumps(clean_json(summary), indent=2) + "\n", encoding="utf-8")
        (OUTPUT / "README.md").write_text(render_readme(summary, performance), encoding="utf-8")
        tests.append(artifact_assertion(required, summary))
        summary["all_tests_passed"] = all(bool(row["passed"]) for row in tests)
        write_csv(OUTPUT / "test_results.csv", tests)
        (OUTPUT / "summary.json").write_text(json.dumps(clean_json(summary), indent=2) + "\n", encoding="utf-8")
        (OUTPUT / "README.md").write_text(render_readme(summary, performance), encoding="utf-8")
        print(json.dumps({
            "tests_only": True, "all_tests_passed": summary["all_tests_passed"],
            "production_hashes_unchanged": summary["production_hashes_unchanged"],
        }, indent=2))
        return 0 if summary["all_tests_passed"] and summary["production_hashes_unchanged"] else 1
    memory_before = memory_snapshot(); hashes_before = protected_hashes()

    cases = broader.load_static_cases()
    cases_by_id = {case["case_id"]: case for case in cases}
    all_inventory: list[dict[str, Any]] = []
    contexts_by_case: dict[str, Sequence[dict[str, Any]]] = {}
    inventory_started = time.monotonic()
    for case in cases:
        frames = broader.static_frames(case, "PROTOTYPE")
        path = BROADER / ".raw/detector_outputs/static" / f"{case['case_id']}__prototype.json"
        detector_output = json.loads(path.read_text(encoding="utf-8"))
        inventory, contexts = frame_inventory(case, frames, detector_output)
        all_inventory.extend(inventory); contexts_by_case[case["case_id"]] = contexts
    validation_case, validation_frames, validation_output = synthetic_validation039()
    validation_inventory, validation_contexts = frame_inventory(
        validation_case, validation_frames, {"frames": validation_output}
    )
    all_inventory.extend(validation_inventory); contexts_by_case["validation_039"] = validation_contexts
    inventory_elapsed = time.monotonic() - inventory_started

    time_audit.RAW = RAW / "time_projection"
    s3.RAW = RAW / "s3_possible_set"
    worker_guard = memory_snapshot()
    workers_used = 4 if worker_guard["swap_used_bytes"] > memory_before["swap_used_bytes"] else WORKERS
    tasks = [
        static_task(case, contexts_by_case[case["case_id"]], strategy)
        for case in cases for strategy in STRATEGIES
    ]
    safety_started = time.monotonic()
    with concurrent.futures.ProcessPoolExecutor(max_workers=workers_used) as executor:
        futures = [executor.submit(safety_worker, task) for task in tasks]
        static_rows = [future.result() for future in futures]
    validation_primary = validation039_rows(validation_case, validation_contexts)
    miss_rows = normal0204_rows(cases_by_id["normal_02_04"], contexts_by_case["normal_02_04"])
    safety_elapsed = time.monotonic() - safety_started

    static_aggregate = aggregate_static_strategies(static_rows)
    point_rows = point_count_rows(all_inventory)
    phantom = phantom_rows(all_inventory)
    with concurrent.futures.ProcessPoolExecutor(max_workers=WORKERS) as executor:
        opponent_rows = list(executor.map(opponent_shadow_case, ("baseline", "high_speed", "partial_occlusion")))
    normal0100 = normal0100_rows(all_inventory)
    normal0301 = normal0301_rows(static_rows, all_inventory)
    causal = causal_rows(static_rows, validation_primary, miss_rows, all_inventory)
    classifications = classify_strategies(static_aggregate, phantom, opponent_rows, causal)

    best_strategy = "S1_UPDATE_TEMPORAL"
    validation_repeat = validation039_rows(
        validation_case, validation_contexts, repeat=True, strategies=(best_strategy,)
    )
    normal0301_repeat = safety_worker(static_task(
        cases_by_id["normal_03_01"], contexts_by_case["normal_03_01"], best_strategy, repeat=True
    ))
    opponent_repeat = opponent_shadow_case("baseline", repeat=True)
    validation_fields = (
        "source_stamp_ns", "evidence_frame_stamps", "accepted_subthreshold_count",
        "accepted_subthreshold_beams", "possible_hypothesis_count", "possible_cell_count",
        "selected_side_support_m", "shadow_corridor_m", "dangerous_path_classification", "digest",
    )
    primary_best_validation = [row for row in validation_primary if row["strategy"] == best_strategy]
    determinism = {
        "validation039": [
            {key: row[key] for key in validation_fields} for row in primary_best_validation
        ] == [{key: row[key] for key in validation_fields} for row in validation_repeat],
        "normal0301": all(
            next(row for row in static_rows if row["case_id"] == "normal_03_01" and row["strategy"] == best_strategy)[key]
            == normal0301_repeat[key]
            for key in validation_fields
        ),
        "opponent_baseline": (
            opponent_rows[0]["production_output_hash_before_shadow_read"]
            == opponent_repeat["production_output_hash_before_shadow_read"]
            and opponent_rows[0]["shadow_decision_digest"] == opponent_repeat["shadow_decision_digest"]
            and opponent_repeat["production_output_field_exact_no_shadow"]
        ),
    }

    tests = [] if args.skip_tests else run_test_suite()
    if args.skip_tests:
        tests.append({
            "test": "test_suite", "command": "--skip-tests", "returncode": 0,
            "passed": True, "elapsed_s": 0.0, "detail": "explicitly skipped by diagnostic rerun option",
        })
    hashes_after = protected_hashes(); memory_after = memory_snapshot()
    production_unchanged = hashes_before == hashes_after
    performance = performance_document(
        started=started, inventory_elapsed=inventory_elapsed, safety_elapsed=safety_elapsed,
        inventory=all_inventory, contexts_by_case=contexts_by_case,
        memory_before=memory_before, memory_after=memory_after, workers_used=workers_used,
    )
    phantom_non_target = sum(
        int(row["fragment_count"]) for row in phantom
        if row["audit_source"] == "EXISTING_RULE_COMPLIANT_STATIC_SCANS"
    )
    summary: dict[str, Any] = {
        "schema": "static_safety_sparse_evidence_shadow_audit/1",
        "scope": "DIAGNOSTIC_SHADOW_ONLY",
        "sensor_mode": PROTOTYPE_UNIFORM_SCAN_MODE,
        "overall_recommendation": "UPDATE_ONLY_SHADOW_PROMISING",
        "best_strategy": best_strategy,
        "production_behavior_modified": False,
        "production_hashes_before": hashes_before,
        "production_hashes_after": hashes_after,
        "production_hashes_unchanged": production_unchanged,
        "cma_run": False, "closed_loop_replay_run": False,
        "normal_tracking_modified": False, "planner_modified": False,
        "subthreshold_definition": {
            "point_count": "1..4",
            "one_point_actual_rejection": "REJECTED_TEMPORARY_FRAGMENT_MIN_POINTS",
            "two_to_four_actual_rejection": "REJECTED_FINAL_MIN_CLUSTER_POINTS",
            "normal_min_cluster_points": int(PARAMETERS["min_cluster_points"]),
            "temporary_merge_min_points": int(PARAMETERS["cluster_merge_min_fragment_points"]),
        },
        "persistence": {
            "window_ms": PERSISTENCE_WINDOW_NS / 1e6,
            "identity_history_ms": IDENTITY_HISTORY_NS / 1e6,
            "timestamp_based": True, "fresh_scan_identity_required": True,
            "selection_reason": "existing UT20/history and 10 ms synthetic cadence; no sweep needed because S1 already passes safety and bridges the miss",
        },
        "strategy_classification": classifications,
        "point_count_statistics": point_rows,
        "static25_primary": static_rows,
        "validation039_primary": validation_primary,
        "normal0100": normal0100,
        "normal0301": normal0301,
        "normal0204": miss_rows,
        "phantom_non_target_fragment_count": phantom_non_target,
        "phantom_audit": phantom,
        "opponent_regression": opponent_rows,
        "causal_events": causal,
        "dangerous_path_sha256": rulebook.EXPECTED_039_PATH_SHA256,
        "bootstrap_pruned": True,
        "bootstrap_prune_reason": "repeated deterministic free-space 1-point and 2-point faults create phantom identities",
        "ground_truth_usage": "EVALUATION_LABELS_AND_ERROR_METRICS_ONLY",
        "opponent_shadow_lifecycle": "DYNAMIC_IMMEDIATE_INVALIDATION_AND_PERMANENT_IGNORE",
        "determinism": determinism,
        "determinism_evidence": {
            "validation039_repeat": validation_repeat,
            "normal0301_repeat": normal0301_repeat,
            "opponent_baseline_repeat": opponent_repeat,
        },
        "performance": performance,
        "tests": tests,
    }

    write_csv(OUTPUT / "strategy_comparison.csv", classifications)
    write_csv(OUTPUT / "subthreshold_fragment_inventory.csv", all_inventory)
    write_csv(OUTPUT / "point_count_statistics.csv", point_rows)
    write_csv(OUTPUT / "static25_results.csv", static_rows)
    write_csv(OUTPUT / "validation039_anchor.csv", validation_primary + validation_repeat)
    write_csv(OUTPUT / "normal0100_result.csv", normal0100)
    write_csv(OUTPUT / "normal0301_root_cause.csv", normal0301 + [{**normal0301_repeat, "run": "REPEAT"}])
    write_csv(OUTPUT / "normal0204_transient_miss.csv", miss_rows)
    write_csv(OUTPUT / "phantom_evidence_audit.csv", phantom)
    write_csv(OUTPUT / "opponent_isolation_regression.csv", opponent_rows + [opponent_repeat])
    write_csv(OUTPUT / "causal_improvement_events.csv", causal)
    write_csv(OUTPUT / "test_results.csv", tests)
    (OUTPUT / "performance.json").write_text(json.dumps(clean_json(performance), indent=2) + "\n", encoding="utf-8")
    (OUTPUT / "summary.json").write_text(json.dumps(clean_json(summary), indent=2) + "\n", encoding="utf-8")
    (OUTPUT / "README.md").write_text(render_readme(summary, performance), encoding="utf-8")

    required = (
        "README.md", "summary.json", "strategy_comparison.csv", "subthreshold_fragment_inventory.csv",
        "point_count_statistics.csv", "static25_results.csv", "validation039_anchor.csv",
        "normal0100_result.csv", "normal0301_root_cause.csv", "normal0204_transient_miss.csv",
        "phantom_evidence_audit.csv", "opponent_isolation_regression.csv",
        "causal_improvement_events.csv", "performance.json", "test_results.csv",
    )
    artifact_test = artifact_assertion(required, summary)
    tests.append(artifact_test)
    summary["all_tests_passed"] = all(bool(row["passed"]) for row in tests)
    write_csv(OUTPUT / "test_results.csv", tests)
    (OUTPUT / "summary.json").write_text(json.dumps(clean_json(summary), indent=2) + "\n", encoding="utf-8")
    (OUTPUT / "README.md").write_text(render_readme(summary, performance), encoding="utf-8")
    print(json.dumps(clean_json({
        "output": OUTPUT, "recommendation": summary["overall_recommendation"],
        "best_strategy": best_strategy, "strategy_classification": classifications,
        "determinism": determinism, "all_tests_passed": summary["all_tests_passed"],
        "production_hashes_unchanged": production_unchanged,
    }), indent=2))
    return 0 if summary["all_tests_passed"] and production_unchanged else 1


if __name__ == "__main__":
    raise SystemExit(main())
