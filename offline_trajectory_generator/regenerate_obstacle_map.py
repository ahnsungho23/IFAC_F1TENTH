#!/usr/bin/env python3
"""Headless regeneration driver for the map_creator pipeline.

Loads the exact same gui_params.yaml the trajectory GUI uses (via
trajectory_gui.load_gui_params), overrides only the input map (the painted
obstacle map) and the output directory, runs generate_trajectory(), applies the
physical swap gates from learning_adaptive_globalpath/MAP_CREATOR_PROPOSAL.md
(3.4), and writes global_waypoints.json + metadata.json + gate_report.json.

Exit codes: 0 = generated and all physical gates passed, 1 = gate failure,
2 = generation error / bad invocation.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

try:
    from trajectory_gui import load_gui_params, make_namespace, render_preview_rgb
except ImportError as exc:  # pragma: no cover - environment guard
    print(
        "[regenerate_obstacle_map] cannot import trajectory_gui "
        f"({exc}). python3-tk must be installed (trajectory_gui imports tkinter).",
        file=sys.stderr,
    )
    sys.exit(2)

from generate_global_trajectory import generate_trajectory, write_outputs


def densify_closed(points_xy: np.ndarray, step: float) -> np.ndarray:
    closed = np.vstack([points_xy, points_xy[:1]])
    seg = np.diff(closed, axis=0)
    seg_len = np.hypot(seg[:, 0], seg[:, 1])
    out = []
    for i, length in enumerate(seg_len):
        n = max(1, int(math.ceil(length / step)))
        t = np.arange(n) / n
        out.append(closed[i] + t[:, None] * seg[i])
    return np.vstack(out)


def min_distance_to_aabb(points_xy: np.ndarray, box: dict) -> float:
    dx = np.maximum(
        np.maximum(box["x_min"] - points_xy[:, 0], 0.0), points_xy[:, 0] - box["x_max"])
    dy = np.maximum(
        np.maximum(box["y_min"] - points_xy[:, 1], 0.0), points_xy[:, 1] - box["y_max"])
    return float(np.min(np.hypot(dx, dy)))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-yaml", required=True, type=Path,
                        help="painted obstacle map yaml (input)")
    parser.add_argument("--gui-params", type=Path,
                        default=SCRIPT_DIR / "gui_params.yaml")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--obstacles-json", type=Path, default=None,
                        help="obstacle snapshot written by map_creator; enables the "
                             "path-to-obstacle clearance gate")
    parser.add_argument("--min-clearance", type=float, default=0.42,
                        help="required min distance [m] from the path to every baked "
                             "obstacle AABB (local-planner silence clearance)")
    parser.add_argument("--max-kappa", type=float, default=3.2,
                        help="physical curvature gate [rad/m] (local planner limit)")
    parser.add_argument("--safety-width", type=float, default=None,
                        help="override gui_params safety_width (retry pass when the "
                             "clearance gate failed)")
    parser.add_argument("--smooth-sigma", type=float, default=None,
                        help="override gui_params smooth_sigma for a retry pass")
    parser.add_argument("--preview-png", action="store_true",
                        help="also write obstacle_debug_overlay.png (painted map + "
                             "centerline + regenerated raceline) for quick inspection")
    args = parser.parse_args()

    if not args.map_yaml.is_file():
        print(f"[regenerate_obstacle_map] map yaml not found: {args.map_yaml}",
              file=sys.stderr)
        return 2

    values = load_gui_params(args.gui_params)
    values["map_yaml"] = str(args.map_yaml)
    values["output_dir"] = str(args.output_dir)
    if args.safety_width is not None:
        values["safety_width"] = float(args.safety_width)
    if args.smooth_sigma is not None:
        values["smooth_sigma"] = float(args.smooth_sigma)
    gen_args = make_namespace(values)

    started = time.time()
    try:
        result = generate_trajectory(gen_args)
    except Exception as exc:  # noqa: BLE001 - report and gate
        report = {
            "status": "generation_error",
            "error": f"{type(exc).__name__}: {exc}",
            "elapsed_sec": time.time() - started,
        }
        args.output_dir.mkdir(parents=True, exist_ok=True)
        (args.output_dir / "gate_report.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=False))
        print(f"[regenerate_obstacle_map] generation failed: {report['error']}",
              file=sys.stderr)
        return 1

    traj = result.global_traj
    s_m = np.asarray(traj.s_m, dtype=float)
    kappa = np.asarray(traj.kappa_radpm, dtype=float)
    points_xy = np.asarray(traj.points_xy, dtype=float)

    gates: dict[str, dict] = {}

    def gate(name: str, passed: bool, **info) -> None:
        gates[name] = {"passed": bool(passed), **info}

    gate("off_map", result.off_map_wpnts == 0, off_map_wpnts=int(result.off_map_wpnts))
    gate("finite", bool(np.all(np.isfinite(points_xy)) and np.all(np.isfinite(s_m))
                        and np.all(np.isfinite(kappa))))
    gate("s_strictly_increasing", bool(np.all(np.diff(s_m) > 0.0)))
    max_abs_kappa = float(np.max(np.abs(kappa))) if kappa.size else float("nan")
    gate("kappa_physical", max_abs_kappa <= args.max_kappa,
         max_abs_kappa=max_abs_kappa, limit=args.max_kappa)

    clearance_info: dict = {"checked": False}
    if args.obstacles_json and args.obstacles_json.is_file():
        obstacles = json.loads(args.obstacles_json.read_text()).get("obstacles", [])
        dense = densify_closed(points_xy, 0.02)
        min_clear = math.inf
        per_obstacle = []
        for obs in obstacles:
            dist = min_distance_to_aabb(dense, obs)
            per_obstacle.append({"id": obs.get("id"), "min_distance_m": dist})
            min_clear = min(min_clear, dist)
        clearance_info = {
            "checked": True,
            "min_clearance_m": None if math.isinf(min_clear) else min_clear,
            "required_m": args.min_clearance,
            "per_obstacle": per_obstacle,
        }
        gate("obstacle_clearance",
             math.isinf(min_clear) or min_clear >= args.min_clearance,
             **{k: v for k, v in clearance_info.items() if k != "per_obstacle"})

    # Quality-only indicators (never block the swap; MAP_CREATOR_PROPOSAL 3.4).
    quality = {
        "kappa_violations_generator_limit": int(result.kappa_violations),
        "generator_max_curvature": float(getattr(gen_args, "max_curvature", float("nan"))),
        "waypoint_count": int(len(s_m)),
        "lap_time_estimate_sec": float(result.lap_time),
    }

    all_passed = all(g["passed"] for g in gates.values())
    report = {
        "status": "ok" if all_passed else "gate_failure",
        "gates": gates,
        "quality": quality,
        "clearance": clearance_info,
        "elapsed_sec": time.time() - started,
        "map_yaml": str(args.map_yaml),
        "gui_params": str(args.gui_params),
    }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    if all_passed:
        write_outputs(args.output_dir, result.map_info, result.center_traj,
                      result.global_traj, result.lap_time, gen_args)
        if args.preview_png:
            import cv2
            # 베이스라인 생성 경로의 debug_overlay.png 와 짝이 되는 이름.
            # 이쪽은 칠해진 obstacle_map 위에 재생성 라인을 그린 것이다.
            cv2.imwrite(str(args.output_dir / "obstacle_debug_overlay.png"),
                        render_preview_rgb(result)[:, :, ::-1])
    (args.output_dir / "gate_report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False))

    print(f"[regenerate_obstacle_map] {report['status']} "
          f"(elapsed {report['elapsed_sec']:.1f}s, gates: "
          + ", ".join(f"{k}={'PASS' if v['passed'] else 'FAIL'}"
                      for k, v in gates.items()) + ")")
    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
