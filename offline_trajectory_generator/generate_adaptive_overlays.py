#!/usr/bin/env python3
"""Evaluate local-planner sides and render time-labelled adaptive overlays."""

from __future__ import annotations

import argparse
import atexit
import contextlib
import csv
import io
import json
import math
import os
import shutil
import subprocess
import tempfile
import time
import traceback
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime
from pathlib import Path
from typing import Any
from zoneinfo import ZoneInfo

import cv2
import numpy as np
import yaml

from generate_global_trajectory import generate_trajectory
from trajectory_gui import load_gui_params, make_namespace, render_preview_rgb


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
DEFAULT_REFERENCE = REPO_ROOT / "ruleset_adaptive_globalpath/map/global_waypoints.csv"
DEFAULT_LOCAL_PARAMS = REPO_ROOT / "src/local_planning/config/local_planning.yaml"
DEFAULT_GUI_PARAMS = SCRIPT_DIR / "gui_params.yaml"
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "ruleset_adaptive_globalpath/adaptive_overlays"

PLANNER_PARAMETER_KEYS = (
    "detection_lookahead_m",
    "obstacle_cluster_gap_m",
    "obstacle_longitudinal_padding_m",
    "obstacle_clearance_m",
    "blocking_margin_m",
    "vehicle_half_width_m",
    "boundary_margin_m",
    "fallback_track_half_width_m",
    "pre_apex_distances_m",
    "post_apex_distances_m",
    "transition_distance_scales",
    "outside_line_transition_scale",
    "post_merge_lookahead_m",
    "post_merge_min_time_sec",
    "minimum_target_offset_m",
    "maximum_target_offset_m",
    "commitment_clearance_reserve_m",
    "minimum_avoidance_clearance_m",
    "side_tie_epsilon_m",
    "maximum_lateral_slope",
    "maximum_curvature_radpm",
    "maximum_curvature_rate_radpm2",
    "safe_stop_buffer_m",
    "safe_stop_deceleration_mps2",
    "minimum_path_points",
    "hard_collision_margin_m",
)

STATE: dict[str, Any] = {}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, default=DEFAULT_REFERENCE)
    parser.add_argument("--local-params", type=Path, default=DEFAULT_LOCAL_PARAMS)
    parser.add_argument("--gui-params", type=Path, default=DEFAULT_GUI_PARAMS)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--evaluator", type=Path)
    parser.add_argument("--timezone", default="Asia/Seoul")
    parser.add_argument("--run-id")
    parser.add_argument("--workers", type=int, default=min(8, os.cpu_count() or 1))
    parser.add_argument("--ego-lookback", type=float, default=7.0)
    parser.add_argument("--obstacle-size", type=float, default=0.20)
    parser.add_argument("--d-min", type=float, default=-0.5)
    parser.add_argument("--d-max", type=float, default=0.5)
    parser.add_argument("--d-step", type=float, default=0.1)
    parser.add_argument("--side-tolerance", type=float, default=0.05)
    parser.add_argument("--outline-boundary-margin", type=float, default=0.40)
    parser.add_argument("--outline-smooth-sigma", type=float, default=2.5)
    parser.add_argument("--planner-override", action="append", default=[], metavar="KEY=VALUE")
    parser.add_argument("--evaluate-only", action="store_true")
    parser.add_argument(
        "--limit", type=int, help="Smoke-test only: render/evaluate the first N rows"
    )
    parser.add_argument("--strict-count", type=int, default=1562)
    return parser.parse_args()


def make_run_id(prefix: str, timezone_name: str) -> tuple[str, str]:
    now = datetime.now(ZoneInfo(timezone_name))
    milliseconds = now.microsecond // 1000
    run_id = f"{prefix}_{now:%Y%m%d_%H%M%S}_{milliseconds:03d}_KST"
    return run_id, now.isoformat(timespec="milliseconds")


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        payload = yaml.safe_load(stream) or {}
    if not isinstance(payload, dict):
        raise RuntimeError(f"YAML root must be a mapping: {path}")
    return payload


def parse_override(assignment: str) -> tuple[str, Any]:
    if "=" not in assignment:
        raise ValueError(f"planner override must be KEY=VALUE: {assignment}")
    key, text = assignment.split("=", maxsplit=1)
    if key not in PLANNER_PARAMETER_KEYS:
        raise ValueError(f"unsupported planner override: {key}")
    if key in {"pre_apex_distances_m", "post_apex_distances_m", "transition_distance_scales"}:
        value: Any = [float(item) for item in text.split(",") if item]
    elif key == "minimum_path_points":
        value = int(text)
    else:
        value = float(text)
    return key, value


def planner_values(path: Path, overrides: list[str]) -> dict[str, Any]:
    payload = load_yaml(path)
    try:
        values = dict(payload["local_planner_node"]["ros__parameters"])
    except (KeyError, TypeError) as exc:
        raise RuntimeError(f"missing local_planner_node.ros__parameters in {path}") from exc
    for assignment in overrides:
        key, value = parse_override(assignment)
        values[key] = value
    missing = [key for key in PLANNER_PARAMETER_KEYS if key not in values]
    if missing:
        raise RuntimeError(f"planner YAML is missing evaluator parameters: {', '.join(missing)}")
    return {key: values[key] for key in PLANNER_PARAMETER_KEYS}


def evaluator_path(explicit: Path | None) -> Path:
    candidates = [] if explicit is None else [explicit]
    candidates.extend(
        [
            REPO_ROOT / "install/local_planning/lib/local_planning/adaptive_side_evaluator",
            REPO_ROOT / "build/local_planning/adaptive_side_evaluator",
        ]
    )
    found = shutil.which("adaptive_side_evaluator")
    if found:
        candidates.append(Path(found))
    for candidate in candidates:
        path = candidate.expanduser().resolve()
        if path.is_file() and os.access(path, os.X_OK):
            return path
    raise RuntimeError(
        "adaptive_side_evaluator was not found; build it with "
        "`colcon build --packages-select local_planning`"
    )


def command_value(value: Any) -> str:
    if isinstance(value, list):
        return ",".join(str(item) for item in value)
    return str(value)


def run_evaluator(
    executable: Path,
    reference: Path,
    output: Path,
    values: dict[str, Any],
    args: argparse.Namespace,
) -> None:
    command = [
        str(executable),
        "--reference",
        str(reference),
        "--output",
        str(output),
        "--ego-lookback",
        str(args.ego_lookback),
        "--obstacle-size",
        str(args.obstacle_size),
        "--d-min",
        str(args.d_min),
        "--d-max",
        str(args.d_max),
        "--d-step",
        str(args.d_step),
    ]
    for key in PLANNER_PARAMETER_KEYS:
        command.extend(["--param", f"{key}={command_value(values[key])}"])
    completed = subprocess.run(command, text=True, capture_output=True, check=False)
    if completed.returncode != 0:
        raise RuntimeError(
            f"side evaluator failed ({completed.returncode}):\n{completed.stderr.strip()}"
        )


def load_evaluations(path: Path) -> list[dict[str, Any]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def load_reference(path: Path) -> dict[int, dict[str, float]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    return {
        int(row["id"]): {
            "index": int(row["id"]),
            "s": float(row.get("s_m", row.get("s", 0.0))),
            "x": float(row["x_m"]),
            "y": float(row["y_m"]),
            "psi": float(row["psi_rad"]),
        }
        for row in rows
    }


def load_xy(path: Path) -> list[list[float]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    return [[float(row["x_m"]), float(row["y_m"])] for row in rows]


def resolve_map(gui_values: dict[str, Any]) -> tuple[Path, Path, dict[str, Any]]:
    map_yaml = Path(str(gui_values["map_yaml"])).expanduser().resolve()
    config = load_yaml(map_yaml)
    image = Path(str(config["image"])).expanduser()
    if not image.is_absolute():
        image = map_yaml.parent / image
    if not image.is_file():
        raise RuntimeError(f"map image does not exist: {image}")
    return map_yaml, image.resolve(), config


def offset_label(value: float) -> str:
    tenths = int(round(value * 10.0))
    return f"{'m' if tenths < 0 else 'p'}{abs(tenths):02d}"


def output_name(item: dict[str, Any], label: str) -> str:
    return f"idx_{int(item['index']):03d}_d_{offset_label(float(item['d']))}_{label}.png"


def world_to_pixel(point: np.ndarray, state: dict[str, Any]) -> np.ndarray:
    origin_x, origin_y = state["origin"]
    resolution = state["resolution"]
    row = (point[1] - origin_y) / resolution
    if state["flip_y"]:
        row = (state["base_image"].shape[0] - 1) - row
    return np.asarray([(point[0] - origin_x) / resolution, row], dtype=np.float64)


def rounded_pixel(point: np.ndarray, state: dict[str, Any]) -> tuple[int, int]:
    pixel = np.rint(world_to_pixel(point, state)).astype(int)
    return int(pixel[0]), int(pixel[1])


def wall_endpoint(
    start: np.ndarray, direction: np.ndarray, state: dict[str, Any]
) -> tuple[tuple[int, int], float, bool]:
    image = state["base_image"]
    height, width = image.shape
    step = state["resolution"] * 0.25
    max_distance = math.hypot(width, height) * state["resolution"]
    last = rounded_pixel(start, state)
    for distance in np.arange(0.0, max_distance + step, step):
        col, row = rounded_pixel(start + float(distance) * direction, state)
        if col < 0 or col >= width or row < 0 or row >= height:
            return last, max(0.0, float(distance) - step), False
        last = (col, row)
        if int(image[row, col]) < 250:
            return last, float(distance), True
    return last, max_distance, False


def fill_to_wall(
    image: np.ndarray,
    center: np.ndarray,
    tangent: np.ndarray,
    normal: np.ndarray,
    side_sign: float,
    state: dict[str, Any],
) -> tuple[list[float], bool]:
    half = 0.5 * state["obstacle_size"]
    direction = side_sign * normal
    edge_center = center + side_sign * half * normal
    sample_step = state["resolution"] * 0.25
    sample_count = max(9, int(math.ceil(state["obstacle_size"] / sample_step)) + 1)
    offsets = np.linspace(-half, half, sample_count)
    near_pixels: list[tuple[int, int]] = []
    wall_pixels: list[tuple[int, int]] = []
    distances: list[float] = []
    found_all = True
    for offset in offsets:
        start = edge_center + float(offset) * tangent
        near_pixels.append(rounded_pixel(start, state))
        endpoint, distance, found = wall_endpoint(start, direction, state)
        wall_pixels.append(endpoint)
        distances.append(distance)
        found_all = found_all and found
    polygon = np.asarray(near_pixels + list(reversed(wall_pixels)), dtype=np.int32)
    cv2.fillPoly(image, [polygon], 0, lineType=cv2.LINE_8)
    return distances, found_all


def draw_obstacle(
    image: np.ndarray,
    center: np.ndarray,
    tangent: np.ndarray,
    normal: np.ndarray,
    state: dict[str, Any],
) -> None:
    half = 0.5 * state["obstacle_size"]
    corners = np.asarray(
        [
            center - half * tangent - half * normal,
            center + half * tangent - half * normal,
            center + half * tangent + half * normal,
            center - half * tangent + half * normal,
        ]
    )
    pixels = np.rint([world_to_pixel(point, state) for point in corners]).astype(np.int32)
    cv2.fillConvexPoly(image, pixels, 0, lineType=cv2.LINE_8)


def modified_map(
    item: dict[str, Any], state: dict[str, Any]
) -> tuple[np.ndarray, list[float], bool]:
    waypoint = state["reference"][int(item["index"])]
    psi = waypoint["psi"]
    tangent = np.asarray([math.cos(psi), math.sin(psi)])
    normal = np.asarray([-math.sin(psi), math.cos(psi)])
    center = np.asarray([waypoint["x"], waypoint["y"]]) + float(item["d"]) * normal
    image = state["base_image"].copy()
    distances: list[float] = []
    walls_found = True
    signs = [-1.0] if item["decision"] == "left" else [1.0]
    if item["decision"] == "safe_stop":
        signs = [-1.0, 1.0]
    for sign in signs:
        side_distances, side_found = fill_to_wall(
            image, center, tangent, normal, sign, state
        )
        distances.extend(side_distances)
        walls_found = walls_found and side_found
    draw_obstacle(image, center, tangent, normal, state)
    return image, distances, walls_found


def draw_polyline(
    image_rgb: np.ndarray,
    points: np.ndarray,
    color: tuple[int, int, int],
    thickness: int,
    state: dict[str, Any],
) -> None:
    pixels = np.rint([world_to_pixel(point, state) for point in points]).astype(np.int32)
    height, width = image_rgb.shape[:2]
    pixels[:, 0] = np.clip(pixels[:, 0], 0, width - 1)
    pixels[:, 1] = np.clip(pixels[:, 1], 0, height - 1)
    for first, second in zip(pixels, np.roll(pixels, -1, axis=0)):
        cv2.line(image_rgb, tuple(first), tuple(second), color, thickness, cv2.LINE_AA)


def reference_overlay(image: np.ndarray, state: dict[str, Any]) -> np.ndarray:
    rgb = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    thickness = max(1, int(round(max(rgb.shape[:2]) / 700)))
    draw_polyline(rgb, state["center_xy"], (0, 120, 255), thickness, state)
    draw_polyline(rgb, state["global_xy"], (255, 70, 45), thickness + 1, state)
    return rgb


def crossing_d(points: np.ndarray, waypoint: dict[str, float]) -> float:
    tangent = np.asarray([math.cos(waypoint["psi"]), math.sin(waypoint["psi"])])
    normal = np.asarray([-math.sin(waypoint["psi"]), math.cos(waypoint["psi"])])
    rel = points - np.asarray([waypoint["x"], waypoint["y"]])
    longitudinal = rel @ tangent
    lateral = rel @ normal
    crossings: list[float] = []
    for index in range(len(points)):
        nxt = (index + 1) % len(points)
        first = longitudinal[index]
        second = longitudinal[nxt]
        if first == 0.0:
            crossings.append(float(lateral[index]))
        elif first * second <= 0.0 and abs(second - first) > 1.0e-9:
            ratio = -first / (second - first)
            value = lateral[index] + ratio * (lateral[nxt] - lateral[index])
            if abs(value) <= 2.5:
                crossings.append(float(value))
    return min(crossings, key=abs) if crossings else math.nan


def initialize_worker(config: dict[str, Any]) -> None:
    global STATE
    cv2.setNumThreads(1)
    image = cv2.imread(config["map_image"], cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"could not read map image: {config['map_image']}")
    worker_dir = Path(tempfile.mkdtemp(prefix=f"adaptive_overlay_{os.getpid()}_"))
    map_config = dict(config["map_config"])
    map_config["image"] = "map.png"
    (worker_dir / "map.yaml").write_text(
        yaml.safe_dump(map_config, sort_keys=False), encoding="utf-8"
    )
    STATE = {
        **config,
        "base_image": image,
        "worker_dir": worker_dir,
        "reference": {int(key): value for key, value in config["reference"].items()},
        "center_xy": np.asarray(config["center_xy"], dtype=np.float64),
        "global_xy": np.asarray(config["global_xy"], dtype=np.float64),
    }
    atexit.register(shutil.rmtree, worker_dir, ignore_errors=True)


def generate_path(
    image: np.ndarray,
    values: dict[str, Any],
    item: dict[str, Any],
    state: dict[str, Any],
) -> dict[str, Any]:
    worker_dir: Path = state["worker_dir"]
    if not cv2.imwrite(str(worker_dir / "map.png"), image):
        return {"status": "geometry_failure", "detail": "map_write_failed"}
    run_values = dict(values)
    run_values["map_yaml"] = str(worker_dir / "map.yaml")
    run_values["output_dir"] = ""
    run_values["debug_image"] = False
    try:
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            result = generate_trajectory(make_namespace(run_values))
        rgb = render_preview_rgb(result, None, True, True)
        waypoint = state["reference"][int(item["index"])]
        path_d = crossing_d(result.global_traj.points_xy, waypoint)
        obstacle_d = float(item["d"])
        half = 0.5 * state["obstacle_size"]
        tolerance = state["side_tolerance"]
        if item["decision"] == "left":
            side_ok = path_d > obstacle_d + half - tolerance
        else:
            side_ok = path_d < obstacle_d - half + tolerance
        if int(result.off_map_wpnts) > 0:
            status, detail = "geometry_failure", "off_map"
        elif int(result.kappa_violations) > 0:
            status, detail = "geometry_failure", "curvature"
        elif not side_ok:
            status, detail = "side_failure", "wrong_side"
        else:
            status, detail = "success", ""
        return {
            "status": status,
            "detail": detail,
            "rgb": rgb,
            "path_d": path_d,
            "side_ok": side_ok,
            "off_map_waypoints": int(result.off_map_wpnts),
            "kappa_violations": int(result.kappa_violations),
            "max_abs_kappa": float(result.max_abs_kappa),
        }
    except Exception:  # noqa: BLE001 - preserve every failed scenario in the manifest.
        return {
            "status": "geometry_failure",
            "detail": "generator_exception",
            "error": traceback.format_exc(),
        }


def write_png(path: Path, rgb: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp.png")
    if not cv2.imwrite(str(temporary), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
        raise RuntimeError(f"failed to write PNG: {temporary}")
    os.replace(temporary, path)


def generate_one(item: dict[str, Any]) -> dict[str, Any]:
    state = STATE
    started = time.perf_counter()
    image, wall_distances, walls_found = modified_map(item, state)
    record = dict(item)
    record.update(
        {
            "wall_distance_min": min(wall_distances) if wall_distances else math.nan,
            "wall_distance_max": max(wall_distances) if wall_distances else math.nan,
            "walls_found": walls_found,
            "base_generation_status": "not_run",
            "outline_status": "not_run",
            "failure_detail": "",
            "path_crossing_d": math.nan,
            "off_map_waypoints": 0,
            "kappa_violations": 0,
            "error": "",
        }
    )

    if item["decision"] == "safe_stop":
        label = "safe_stop"
        relative = Path("images/safe_stop") / output_name(item, label)
        rgb = reference_overlay(image, state)
    else:
        base = generate_path(image, state["gui_values"], item, state)
        record["base_generation_status"] = base["status"]
        if base["status"] == "success" and walls_found:
            label = item["decision"]
            relative = Path(f"images/{label}") / output_name(item, label)
            rgb = base["rgb"]
            chosen = base
        else:
            outline = generate_path(image, state["outline_values"], item, state)
            if not walls_found:
                outline["status"] = "geometry_failure"
                outline["detail"] = "wall_not_found"
            if outline["status"] == "success":
                subdirectory = f"success_{item['decision']}"
                record["outline_status"] = f"outline_success_{item['decision']}"
            elif outline["status"] == "side_failure":
                subdirectory = "failure_side"
                record["outline_status"] = "outline_side_failure"
            else:
                subdirectory = "failure_geometry"
                record["outline_status"] = "outline_geometry_failure"
            label = "outline"
            relative = Path("images/outline") / subdirectory / output_name(item, label)
            rgb = outline.get("rgb", reference_overlay(image, state))
            chosen = outline
        record["failure_detail"] = chosen.get("detail", "")
        record["path_crossing_d"] = chosen.get("path_d", math.nan)
        record["off_map_waypoints"] = chosen.get("off_map_waypoints", 0)
        record["kappa_violations"] = chosen.get("kappa_violations", 0)
        record["error"] = chosen.get("error", "")

    output_path = Path(state["working_dir"]) / relative
    write_png(output_path, rgb)
    record["final_label"] = label
    record["relative_png_path"] = str(relative)
    record["generation_seconds"] = time.perf_counter() - started
    return record


def evaluation_item(row: dict[str, str]) -> dict[str, Any]:
    return {
        **row,
        "index": int(row["index"]),
        "s": float(row["s"]),
        "d": float(row["d"]),
        "decision": row["decision"],
    }


def write_reports(report_dir: Path, rows: list[dict[str, Any]], summary: dict[str, Any]) -> None:
    report_dir.mkdir(parents=True, exist_ok=True)
    (report_dir / "manifest.json").write_text(
        json.dumps(rows, indent=2, ensure_ascii=False, allow_nan=True), encoding="utf-8"
    )
    if rows:
        fieldnames: list[str] = []
        for row in rows:
            for key in row:
                if key not in fieldnames:
                    fieldnames.append(key)
        with (report_dir / "manifest.csv").open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
    (report_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )


def main() -> int:
    args = parse_args()
    if args.workers <= 0 or args.strict_count <= 0:
        raise RuntimeError("--workers and --strict-count must be positive")
    prefix = "learn" if args.evaluate_only else "run"
    generated_id, started_at = make_run_id(prefix, args.timezone)
    run_id = args.run_id or generated_id
    if not run_id or Path(run_id).name != run_id:
        raise RuntimeError("--run-id must be one directory name")
    output_root = args.output_root.expanduser().resolve()
    if args.evaluate_only:
        working_dir = output_root / "_learning" / run_id
        final_dir = working_dir
    else:
        working_dir = output_root / "_incomplete" / run_id
        final_dir = output_root / run_id
    if working_dir.exists() or (final_dir.exists() and final_dir != working_dir):
        raise RuntimeError(f"run directory already exists: {run_id}")
    working_dir.mkdir(parents=True)
    report_dir = working_dir / "reports"
    parameter_dir = working_dir / "parameters"
    report_dir.mkdir()
    parameter_dir.mkdir()

    local_values = planner_values(args.local_params.expanduser().resolve(), args.planner_override)
    gui_values = load_gui_params(args.gui_params.expanduser().resolve())
    gui_values["optimizer"] = "mincurv"
    outline_values = dict(gui_values)
    outline_values["boundary_margin"] = args.outline_boundary_margin
    outline_values["smooth_sigma"] = args.outline_smooth_sigma
    evaluator = evaluator_path(args.evaluator)

    (parameter_dir / "local_planning_snapshot.yaml").write_text(
        yaml.safe_dump({"local_planner_node": {"ros__parameters": local_values}}, sort_keys=False),
        encoding="utf-8",
    )
    (parameter_dir / "gui_params_snapshot.yaml").write_text(
        yaml.safe_dump(gui_values, sort_keys=False), encoding="utf-8"
    )
    (parameter_dir / "outline_overrides.yaml").write_text(
        yaml.safe_dump(
            {
                "boundary_margin": args.outline_boundary_margin,
                "smooth_sigma": args.outline_smooth_sigma,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    run_config = {
        "run_id": run_id,
        "started_at": started_at,
        "timezone": args.timezone,
        "reference": str(args.reference.expanduser().resolve()),
        "evaluator": str(evaluator),
        "ego_lookback_m": args.ego_lookback,
        "obstacle_size_m": args.obstacle_size,
        "d_min": args.d_min,
        "d_max": args.d_max,
        "d_step": args.d_step,
        "side_tolerance_m": args.side_tolerance,
        "evaluate_only": args.evaluate_only,
    }
    (parameter_dir / "run_config.yaml").write_text(
        yaml.safe_dump(run_config, sort_keys=False), encoding="utf-8"
    )

    evaluation_csv = report_dir / "side_evaluations.csv"
    reference_path = args.reference.expanduser().resolve()
    run_evaluator(
        evaluator,
        reference_path,
        evaluation_csv,
        local_values,
        args,
    )
    raw_evaluations = load_evaluations(evaluation_csv)
    if args.limit is not None:
        if args.limit <= 0:
            raise RuntimeError("--limit must be positive")
        raw_evaluations = raw_evaluations[: args.limit]
    expected = len(raw_evaluations) if args.limit is not None else args.strict_count
    if len(raw_evaluations) != expected:
        raise RuntimeError(
            f"expected {expected} evaluation rows, got {len(raw_evaluations)}"
        )
    decision_counts = Counter(row["decision"] for row in raw_evaluations)

    if args.evaluate_only:
        rows = [evaluation_item(row) for row in raw_evaluations]
        summary = {
            **run_config,
            "count": len(rows),
            "decision_counts": dict(decision_counts),
        }
        write_reports(report_dir, rows, summary)
        (working_dir / "_SUCCESS").touch()
        print(json.dumps(summary, indent=2, ensure_ascii=False))
        return 0

    map_yaml, map_image, map_config = resolve_map(gui_values)
    reference = load_reference(reference_path)
    reference_dir = reference_path.parent
    config = {
        "map_image": str(map_image),
        "map_config": map_config,
        "origin": [float(map_config["origin"][0]), float(map_config["origin"][1])],
        "resolution": float(map_config["resolution"]),
        "flip_y": not bool(gui_values.get("no_flip_y", False)),
        "reference": reference,
        "center_xy": load_xy(reference_dir / "centerline.csv"),
        "global_xy": load_xy(reference_path),
        "gui_values": gui_values,
        "outline_values": outline_values,
        "working_dir": str(working_dir),
        "obstacle_size": args.obstacle_size,
        "side_tolerance": args.side_tolerance,
        "map_yaml": str(map_yaml),
    }
    items = [evaluation_item(row) for row in raw_evaluations]
    workers = max(1, min(args.workers, len(items)))
    rows: list[dict[str, Any]] = []
    render_started = time.perf_counter()
    with ProcessPoolExecutor(
        max_workers=workers,
        initializer=initialize_worker,
        initargs=(config,),
    ) as executor:
        futures = [executor.submit(generate_one, item) for item in items]
        for completed_count, future in enumerate(as_completed(futures), start=1):
            rows.append(future.result())
            if completed_count == 1 or completed_count % 50 == 0 or completed_count == len(items):
                elapsed = time.perf_counter() - render_started
                rate = completed_count / max(elapsed, 1.0e-9)
                remaining = (len(items) - completed_count) / max(rate, 1.0e-9)
                print(
                    f"[{completed_count}/{len(items)}] {rate:.2f} image/s "
                    f"ETA {remaining / 60.0:.1f} min",
                    flush=True,
                )
    rows.sort(key=lambda row: (int(row["index"]), float(row["d"])))
    png_count = sum(1 for _ in (working_dir / "images").rglob("*.png"))
    if png_count != expected:
        raise RuntimeError(f"expected {expected} PNG files, got {png_count}")
    final_counts = Counter(row["final_label"] for row in rows)
    outline_counts = Counter(
        row["outline_status"] for row in rows if row["final_label"] == "outline"
    )
    summary = {
        **run_config,
        "count": len(rows),
        "png_count": png_count,
        "decision_counts": dict(decision_counts),
        "final_label_counts": dict(final_counts),
        "outline_counts": dict(outline_counts),
        "render_seconds": time.perf_counter() - render_started,
    }
    write_reports(report_dir, rows, summary)
    (working_dir / ("_PARTIAL" if args.limit is not None else "_SUCCESS")).touch()
    final_dir.parent.mkdir(parents=True, exist_ok=True)
    os.replace(working_dir, final_dir)
    if args.limit is None:
        (output_root / "latest_run.txt").write_text(
            str(final_dir.relative_to(output_root)) + "\n", encoding="utf-8"
        )
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
