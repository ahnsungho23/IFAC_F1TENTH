#!/usr/bin/env python3
"""Generate a global raceline from a ROS map YAML without running ROS 2."""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np
import yaml
from scipy.ndimage import gaussian_filter1d
from scipy.optimize import minimize


@dataclass(frozen=True)
class MapInfo:
    yaml_path: Path
    image_path: Path
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    negate: int
    occupied_thresh: float
    free_thresh: float
    height: int
    width: int


@dataclass(frozen=True)
class Trajectory:
    points_xy: np.ndarray
    d_right: np.ndarray
    d_left: np.ndarray
    s_m: np.ndarray
    psi_rad: np.ndarray
    kappa_radpm: np.ndarray
    vx_mps: np.ndarray
    ax_mps2: np.ndarray


@dataclass(frozen=True)
class GenerationResult:
    map_info: MapInfo
    image: np.ndarray
    free_mask: np.ndarray
    center_traj: Trajectory
    global_traj: Trajectory
    lap_time: float
    flip_y: bool
    # Waypoints that ended up outside the drivable free space. Non-zero means
    # the extraction most likely latched onto map noise instead of the track.
    off_map_wpnts: int = 0
    # Waypoints whose curvature exceeds the vehicle steering limit
    # (--max-curvature). Non-zero means the raceline is not drivable as-is.
    kappa_violations: int = 0
    max_abs_kappa: float = 0.0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Standalone global trajectory generator for ROS map YAML files."
    )
    add_generator_arguments(parser)
    return parser


def parse_args() -> argparse.Namespace:
    return build_arg_parser().parse_args()


def add_generator_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--map-yaml", required=True, type=Path, help="SLAM map YAML path.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output directory. Defaults to offline_trajectory_generator/output/<map_name>.",
    )
    parser.add_argument("--waypoint-step", type=float, default=0.1, help="Final waypoint spacing [m].")
    parser.add_argument("--optimizer-step", type=float, default=0.2, help="Internal optimizer spacing [m].")
    parser.add_argument("--safety-width", type=float, default=0.35, help="Vehicle+safety width used for bounds [m].")
    parser.add_argument("--boundary-margin", type=float, default=0.03, help="Extra margin from detected walls [m].")
    parser.add_argument("--max-width-distance", type=float, default=5.0, help="Raycast limit for track width [m].")
    parser.add_argument(
        "--width-mode",
        choices=("distance", "raycast", "hybrid"),
        default="hybrid",
        help="Track-width estimation method. "
             "distance: fast but d_left==d_right (no asymmetry); "
             "raycast: directional but noise-sensitive; "
             "hybrid (recommended): combines both for robustness + asymmetry.",
    )
    parser.add_argument("--max-speed", type=float, default=4.0, help="Maximum waypoint speed [m/s].")
    parser.add_argument("--min-speed", type=float, default=1.0, help="Minimum waypoint speed [m/s].")
    parser.add_argument("--max-lateral-accel", type=float, default=4.0, help="Lateral acceleration limit [m/s^2].")
    parser.add_argument("--max-accel", type=float, default=3.0, help="Longitudinal acceleration limit [m/s^2].")
    parser.add_argument("--max-decel", type=float, default=5.0, help="Longitudinal deceleration limit [m/s^2].")
    parser.add_argument(
        "--outside-keepout-m", type=float, default=0.0,
        help="Out-out-out mode [m]: in corners, shrink the INNER-side room by up to this "
             "distance so mincurv settles on the outside band. Widens the lidar sightline "
             "past inner walls (earlier hidden-obstacle reveal) and lowers peak curvature. "
             "0 disables (default). Typical: 0.4-0.8.",
    )
    parser.add_argument(
        "--outside-corner-kappa", type=float, default=0.25,
        help="Centerline |curvature| [rad/m] at which the outside keepout reaches full "
             "strength (quadratic ramp from 0; straights are untouched).",
    )
    parser.add_argument(
        "--outside-smooth-sigma", type=float, default=6.0,
        help="Gaussian smoothing (in optimizer points) applied to the keepout profile so "
             "the usable band edge has no steps.",
    )
    parser.add_argument(
        "--max-curvature", type=float, default=1.2,
        help="Vehicle steering limit as max path curvature [rad/m] "
             "(= tan(max_steer)/wheelbase; ~1.2 for F1TENTH). The raceline "
             "optimizers penalize sharper bends as undrivable and the final "
             "trajectory is validated against it. 0 disables.",
    )
    parser.add_argument("--smooth-sigma", type=float, default=2.0, help="Closed-curve smoothing sigma in samples.")
    parser.add_argument(
        "--raceline-smooth-sigma", type=float, default=1.0,
        help="Gaussian smoothing (in samples) applied to the final raceline after "
             "the waypoint-step resample. The optimizers emit piecewise-linear "
             "lines with vertices at optimizer-step spacing; resampling those "
             "finer concentrates each vertex's heading change into one waypoint "
             "and produces phantom curvature spikes above the steering limit. "
             "~1 sample removes the kinks without changing the geometry. 0 disables.",
    )
    parser.add_argument("--median-kernel", type=int, default=3, help="Odd pixel kernel for salt-and-pepper map denoising.")
    parser.add_argument("--morph-kernel", type=int, default=5, help="Map cleanup kernel size in pixels.")
    parser.add_argument("--morph-open-iterations", type=int, default=1)
    parser.add_argument("--morph-close-iterations", type=int, default=1)
    parser.add_argument("--skeleton-prune-iterations", type=int, default=80)
    parser.add_argument("--min-skeleton-component-area", type=int, default=40)
    parser.add_argument(
        "--min-track-width",
        type=float,
        default=0.3,
        help="Drop skeleton pixels where the free space is narrower than this [m]. "
             "Suppresses centerline candidates inside scan-noise regions the car "
             "could never drive through. 0 disables.",
    )
    parser.add_argument("--min-centerline-angle", type=float, default=75.0)
    parser.add_argument("--spike-filter-iterations", type=int, default=8)
    parser.add_argument(
        "--optimizer",
        choices=("mincurv", "centerline", "laptime", "ai"),
        default="centerline",
        help="centerline: keep the skeleton line; mincurv: scipy minimum-curvature; "
             "laptime: differentiable lap-time gradient descent on GPU "
             "(torch CUDA or Apple MLX; see optimize_laptime.py); "
             "ai: multi-technique search — per-epoch GD portfolio + exact rescoring "
             "+ evolution-strategy polish (see --ai-epochs).",
    )
    parser.add_argument("--max-optimizer-iter", type=int, default=200)
    parser.add_argument("--curvature-weight", type=float, default=1.0)
    parser.add_argument("--smooth-weight", type=float, default=0.04)
    parser.add_argument("--length-weight", type=float, default=0.002)
    # --- laptime optimizer (only used with --optimizer laptime) ---
    parser.add_argument("--laptime-iters", type=int, default=2000,
                        help="laptime: gradient-descent iterations.")
    parser.add_argument("--laptime-restarts", type=int, default=16,
                        help="laptime: candidate lines optimized in parallel on the GPU.")
    parser.add_argument("--laptime-lr", type=float, default=0.08,
                        help="laptime: Adam learning rate (step-decayed 1x/0.3x/0.1x).")
    parser.add_argument("--laptime-smooth-weight", type=float, default=0.2,
                        help="laptime: weight of the lateral-offset smoothness regularizer.")
    parser.add_argument("--laptime-init-spread", type=float, default=2.0,
                        help="laptime: stddev of the random restart initializations (logit space).")
    parser.add_argument("--laptime-seed", type=int, default=0)
    parser.add_argument("--laptime-backend", choices=("auto", "torch", "mlx"), default="auto",
                        help="laptime: ML framework (auto = torch if installed, else mlx).")
    parser.add_argument("--laptime-device", type=str, default="auto",
                        help="laptime, torch only: auto (CUDA if available), cpu, cuda, cuda:N.")
    parser.add_argument("--laptime-no-warm-start", action="store_true",
                        help="laptime: skip seeding one restart from the min-curvature solution.")
    parser.add_argument("--ai-epochs", type=int, default=3,
                        help="ai: alternation rounds of (GPU GD portfolio -> exact rescoring "
                             "-> ES polish); each epoch warm-restarts from the best line so far.")
    parser.add_argument(
        "--no-straighten-straights",
        dest="straighten_straights",
        action="store_false",
        help="Disable clearance-checked straight segment replacement.",
    )
    parser.set_defaults(straighten_straights=True)
    parser.add_argument(
        "--straight-kappa-threshold",
        type=float,
        default=0.2,
        help="Maximum absolute curvature [rad/m] considered straight.",
    )
    parser.add_argument(
        "--straight-min-length",
        type=float,
        default=1.5,
        help="Minimum path length [m] for straight replacement candidates.",
    )
    parser.add_argument(
        "--straight-clearance-margin",
        type=float,
        default=0.03,
        help="Extra clearance [m] required when validating straight replacements.",
    )
    parser.add_argument(
        "--straight-blend-length",
        type=float,
        default=0.5,
        help="Length [m] used to blend each end of a straight replacement.",
    )
    parser.add_argument("--reverse", action="store_true", help="Reverse waypoint order.")
    parser.add_argument(
        "--no-flip-y",
        action="store_true",
        help="Disable standard ROS map image y-axis flip.",
    )
    parser.add_argument(
        "--unknown-as-free",
        action="store_true",
        help="Treat unknown gray map pixels as free. Useful only for maps with closed walls.",
    )
    parser.add_argument("--debug-image", action="store_true", help="Write debug_overlay.png.")


def default_output_dir(map_yaml: Path) -> Path:
    return Path(__file__).resolve().parent / "output" / map_yaml.stem


def validate_args(args: argparse.Namespace) -> None:
    if args.waypoint_step <= 0.0 or args.optimizer_step <= 0.0:
        raise RuntimeError("waypoint-step and optimizer-step must be positive.")
    if args.safety_width <= 0.0:
        raise RuntimeError("safety-width must be positive.")
    if args.max_speed <= 0.0 or args.min_speed <= 0.0:
        raise RuntimeError("speed parameters must be positive.")
    if args.min_speed > args.max_speed:
        raise RuntimeError("min-speed must be <= max-speed.")
    if args.straight_kappa_threshold < 0.0:
        raise RuntimeError("straight-kappa-threshold must be non-negative.")
    if args.straight_min_length < 0.0 or args.straight_clearance_margin < 0.0:
        raise RuntimeError("straight length and clearance parameters must be non-negative.")
    if args.straight_blend_length < 0.0:
        raise RuntimeError("straight-blend-length must be non-negative.")


def load_map(map_yaml: Path, unknown_as_free: bool) -> tuple[MapInfo, np.ndarray, np.ndarray]:
    with map_yaml.open("r", encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream)

    image_path = Path(cfg["image"])
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path

    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"Could not read map image: {image_path}")

    resolution = float(cfg["resolution"])
    origin = cfg.get("origin", [0.0, 0.0, 0.0])
    mode = str(cfg.get("mode", "trinary")).lower()
    negate = int(cfg.get("negate", 0))
    occupied_thresh = float(cfg.get("occupied_thresh", 0.65))
    free_thresh = float(cfg.get("free_thresh", 0.196))

    normalized = image.astype(np.float64) / 255.0
    occupancy = normalized if negate else 1.0 - normalized
    if unknown_as_free:
        free = occupancy < occupied_thresh
    elif mode == "trinary":
        # ROS map_saver commonly writes unknown cells as gray around value 205.
        # Treat only near-free endpoint pixels as drivable so the generator does
        # not route through unknown background outside a cropped SLAM map.
        free = image <= 5 if negate else image >= 250
        if int(np.count_nonzero(free)) == 0:
            free = occupancy <= free_thresh
    else:
        free = occupancy <= free_thresh

    info = MapInfo(
        yaml_path=map_yaml,
        image_path=image_path,
        resolution=resolution,
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        origin_yaw=float(origin[2]) if len(origin) > 2 else 0.0,
        negate=negate,
        occupied_thresh=occupied_thresh,
        free_thresh=free_thresh,
        height=int(image.shape[0]),
        width=int(image.shape[1]),
    )
    return info, image, free.astype(np.uint8)


def occupied_mask_from(image: np.ndarray, info: MapInfo) -> np.ndarray:
    """Binary mask of measured (occupied) wall pixels from the raw map image."""
    normalized = image.astype(np.float64) / 255.0
    occupancy = normalized if info.negate else 1.0 - normalized
    return (occupancy >= info.occupied_thresh).astype(np.uint8)


def cleanup_free_mask(
    free_mask: np.ndarray,
    median_kernel: int,
    morph_kernel: int,
    open_iterations: int,
    close_iterations: int,
    occupied_mask: np.ndarray | None = None,
) -> np.ndarray:
    mask = (free_mask > 0).astype(np.uint8) * 255
    median_size = max(1, median_kernel)
    if median_size % 2 == 0:
        median_size += 1
    if median_size > 1:
        mask = cv2.medianBlur(mask, median_size)

    kernel_size = max(1, morph_kernel)
    if kernel_size % 2 == 0:
        kernel_size += 1
    kernel = np.ones((kernel_size, kernel_size), np.uint8)

    if close_iterations > 0:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=close_iterations)
    if open_iterations > 0:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=open_iterations)

    if occupied_mask is not None:
        # Cleanup may only reclaim unknown/noise pixels — never measured walls.
        # Median blur / morphological closing with a large kernel can otherwise
        # swallow a thin interior wall entirely, and every later stage (widths,
        # corridor bounds, off-map validation) would then believe the wall's
        # area is drivable, letting the raceline cut straight through it.
        # Sub-speckle occupied blobs stay removable so LiDAR salt noise inside
        # the track does not needlessly pinch the corridor.
        speckle_area = max(9, median_size * median_size)
        walls = remove_small_components((occupied_mask > 0).astype(np.uint8), speckle_area)
        mask = np.where(walls > 0, 0, mask).astype(np.uint8)

    mask[0, :] = 0
    mask[-1, :] = 0
    mask[:, 0] = 0
    mask[:, -1] = 0

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats((mask > 0).astype(np.uint8), 8)
    if num_labels <= 1:
        raise RuntimeError("No free-space component was found in the map.")

    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return np.where(labels == largest, 255, 0).astype(np.uint8)


def remove_small_components(binary_mask: np.ndarray, min_area: int) -> np.ndarray:
    mask = (binary_mask > 0).astype(np.uint8)
    if min_area <= 0:
        return mask
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if num_labels <= 1:
        return mask
    cleaned = np.zeros_like(mask)
    for label in range(1, num_labels):
        if stats[label, cv2.CC_STAT_AREA] >= min_area:
            cleaned[labels == label] = 1
    return cleaned


def _zhang_suen_thinning(binary01: np.ndarray) -> np.ndarray:
    """Vectorized Zhang-Suen thinning: topology-preserving 1-px skeleton.

    Unlike the morphological-erosion "skeleton", this keeps the track loop
    connected, so the extraction does not need opencv-contrib's
    cv2.ximgproc.thinning to produce a usable centerline.
    """
    img = (binary01 > 0).astype(np.uint8)
    changed = True
    while changed:
        changed = False
        for step in (0, 1):
            p = np.pad(img, 1)
            p2 = p[0:-2, 1:-1]
            p3 = p[0:-2, 2:]
            p4 = p[1:-1, 2:]
            p5 = p[2:, 2:]
            p6 = p[2:, 1:-1]
            p7 = p[2:, 0:-2]
            p8 = p[1:-1, 0:-2]
            p9 = p[0:-2, 0:-2]
            ring = (p2, p3, p4, p5, p6, p7, p8, p9, p2)
            b = (
                p2.astype(np.int16) + p3 + p4 + p5 + p6 + p7 + p8 + p9
            )
            a = np.zeros_like(b)
            for k in range(8):
                a += ((ring[k] == 0) & (ring[k + 1] == 1)).astype(np.int16)
            if step == 0:
                cond = (p2 * p4 * p6 == 0) & (p4 * p6 * p8 == 0)
            else:
                cond = (p2 * p4 * p8 == 0) & (p2 * p6 * p8 == 0)
            remove = (img == 1) & (b >= 2) & (b <= 6) & (a == 1) & cond
            if bool(np.any(remove)):
                img[remove] = 0
                changed = True
    return (img * 255).astype(np.uint8)


def skeletonize(binary_mask: np.ndarray) -> np.ndarray:
    image = np.where(binary_mask > 0, 255, 0).astype(np.uint8)
    if hasattr(cv2, "ximgproc") and hasattr(cv2.ximgproc, "thinning"):
        return cv2.ximgproc.thinning(image)
    return _zhang_suen_thinning(image)


def prune_skeleton(
    skeleton: np.ndarray,
    prune_iterations: int,
    min_component_area: int,
) -> np.ndarray:
    mask = remove_small_components(skeleton, min_component_area)
    kernel = np.ones((3, 3), np.uint8)
    iterations = max(0, int(prune_iterations))
    for _ in range(iterations):
        neighbors = cv2.filter2D(mask, cv2.CV_16S, kernel, borderType=cv2.BORDER_CONSTANT) - mask
        endpoints = (mask > 0) & (neighbors <= 1)
        if not bool(np.any(endpoints)):
            break
        mask[endpoints] = 0
    mask = remove_small_components(mask, min_component_area)
    # Deliberately no keep_largest_component here: pixel count is a poor
    # discriminator against noise blobs. The contour stage picks the loop with
    # the largest ENCLOSED area instead, which is the actual track loop.
    return (mask * 255).astype(np.uint8)


def extract_centerline_pixels(
    skeleton: np.ndarray,
    prune_iterations: int,
    min_component_area: int,
    min_points: int = 20,
) -> np.ndarray:
    pruned = prune_skeleton(skeleton, prune_iterations, min_component_area)
    if cv2.countNonZero(pruned) >= min_points:
        skeleton = pruned

    contours, _ = cv2.findContours(skeleton, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    candidates = []
    for contour in contours:
        points = contour.reshape(-1, 2)
        if len(points) < min_points:
            continue
        length = cv2.arcLength(contour, closed=True)
        area = abs(cv2.contourArea(contour))
        candidates.append((length, area, points))

    if not candidates:
        raise RuntimeError("Could not extract a closed centerline from the skeletonized map.")

    # Enclosed area first: the real track loop encircles the map interior,
    # while long-but-thin noise contours (scan artifacts, hatching) enclose
    # almost nothing even though they beat the loop on arc length.
    candidates.sort(key=lambda item: (item[1], item[0]), reverse=True)
    return remove_consecutive_duplicates(candidates[0][2].astype(np.float64))


def remove_consecutive_duplicates(points: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    if len(points) == 0:
        return points
    keep = [0]
    for i in range(1, len(points)):
        if np.linalg.norm(points[i] - points[keep[-1]]) > eps:
            keep.append(i)
    out = points[keep]
    if len(out) > 1 and np.linalg.norm(out[0] - out[-1]) <= eps:
        out = out[:-1]
    return out


def pixel_to_world(points_col_row: np.ndarray, info: MapInfo, flip_y: bool) -> np.ndarray:
    x = info.origin_x + points_col_row[:, 0] * info.resolution
    if flip_y:
        y = info.origin_y + (info.height - 1 - points_col_row[:, 1]) * info.resolution
    else:
        y = info.origin_y + points_col_row[:, 1] * info.resolution
    return np.column_stack([x, y])


def world_to_pixel(points_xy: np.ndarray, info: MapInfo, flip_y: bool) -> np.ndarray:
    col = (points_xy[:, 0] - info.origin_x) / info.resolution
    if flip_y:
        row = (info.height - 1) - (points_xy[:, 1] - info.origin_y) / info.resolution
    else:
        row = (points_xy[:, 1] - info.origin_y) / info.resolution
    return np.column_stack([col, row])


def smooth_closed(points_xy: np.ndarray, sigma: float) -> np.ndarray:
    if sigma <= 0.0 or len(points_xy) < 5:
        return points_xy
    x = gaussian_filter1d(points_xy[:, 0], sigma=sigma, mode="wrap")
    y = gaussian_filter1d(points_xy[:, 1], sigma=sigma, mode="wrap")
    return np.column_stack([x, y])


def cumulative_s(points_xy: np.ndarray) -> tuple[np.ndarray, float]:
    closed = np.vstack([points_xy, points_xy[0]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    s = np.insert(np.cumsum(seg), 0, 0.0)
    return s, float(s[-1])


def resample_closed(points_xy: np.ndarray, step: float) -> np.ndarray:
    points_xy = remove_consecutive_duplicates(points_xy)
    if len(points_xy) < 4:
        raise RuntimeError("Need at least four points to resample a closed trajectory.")

    s, total = cumulative_s(points_xy)
    if total <= step * 4:
        raise RuntimeError("Extracted centerline is too short for the requested waypoint step.")

    closed = np.vstack([points_xy, points_xy[0]])
    sample_count = max(4, int(math.floor(total / step)))
    new_s = np.linspace(0.0, total, sample_count, endpoint=False)
    x = np.interp(new_s, s, closed[:, 0])
    y = np.interp(new_s, s, closed[:, 1])
    return np.column_stack([x, y])


def remove_sharp_spikes(
    points_xy: np.ndarray,
    min_angle_deg: float,
    iterations: int,
) -> np.ndarray:
    if min_angle_deg <= 0.0 or iterations <= 0:
        return points_xy

    filtered = remove_consecutive_duplicates(points_xy)
    for _ in range(iterations):
        if len(filtered) < 8:
            break
        prev_vec = np.roll(filtered, 1, axis=0) - filtered
        next_vec = np.roll(filtered, -1, axis=0) - filtered
        prev_norm = np.linalg.norm(prev_vec, axis=1)
        next_norm = np.linalg.norm(next_vec, axis=1)
        denom = prev_norm * next_norm
        dot = np.sum(prev_vec * next_vec, axis=1)
        cos_angle = np.divide(dot, denom, out=np.ones_like(dot), where=denom > 1e-9)
        angles = np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0)))
        spike_indices = np.flatnonzero(angles < min_angle_deg)
        if len(spike_indices) == 0:
            break

        max_remove = max(1, len(filtered) // 12)
        if len(spike_indices) > max_remove:
            order = np.argsort(angles[spike_indices])
            spike_indices = spike_indices[order[:max_remove]]
        keep = np.ones(len(filtered), dtype=bool)
        keep[spike_indices] = False
        filtered = filtered[keep]

    return filtered


def filter_and_resample_closed(
    points_xy: np.ndarray,
    step: float,
    args: argparse.Namespace,
) -> np.ndarray:
    filtered = remove_sharp_spikes(
        points_xy,
        args.min_centerline_angle,
        args.spike_filter_iterations,
    )
    # A spike filter that eats a large share of the points is misfiring on the
    # path shape (e.g. a tightly-cut raceline), not removing pin artifacts.
    # Resampling the surviving points would bridge the gaps with wall-crossing
    # chords, so prefer the unfiltered path in that case.
    if len(filtered) < max(4, int(0.7 * len(points_xy))):
        filtered = points_xy
    sampled = resample_closed(filtered, step)
    post_filtered = remove_sharp_spikes(
        sampled,
        args.min_centerline_angle,
        args.spike_filter_iterations,
    )
    if max(4, int(0.7 * len(sampled))) <= len(post_filtered) < len(sampled):
        sampled = resample_closed(post_filtered, step)
    return sampled


def headings_and_curvature(points_xy: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    prev_pts = np.roll(points_xy, 1, axis=0)
    next_pts = np.roll(points_xy, -1, axis=0)
    chord = next_pts - prev_pts
    psi = np.arctan2(chord[:, 1], chord[:, 0])

    a = np.linalg.norm(points_xy - prev_pts, axis=1)
    b = np.linalg.norm(next_pts - points_xy, axis=1)
    c = np.linalg.norm(next_pts - prev_pts, axis=1)
    cross = (
        (points_xy[:, 0] - prev_pts[:, 0]) * (next_pts[:, 1] - prev_pts[:, 1])
        - (points_xy[:, 1] - prev_pts[:, 1]) * (next_pts[:, 0] - prev_pts[:, 0])
    )
    denom = a * b * c
    kappa = np.divide(2.0 * cross, denom, out=np.zeros_like(cross), where=denom > 1e-9)

    seg = np.linalg.norm(np.roll(points_xy, -1, axis=0) - points_xy, axis=1)
    s = np.insert(np.cumsum(seg[:-1]), 0, 0.0)
    return s, psi, kappa


def normals_from_heading(psi: np.ndarray) -> np.ndarray:
    return np.column_stack([-np.sin(psi), np.cos(psi)])


def raycast_distance(
    free_mask: np.ndarray,
    point_xy: np.ndarray,
    direction_xy: np.ndarray,
    info: MapInfo,
    flip_y: bool,
    max_distance_m: float = 10.0,
) -> float:
    step_m = max(info.resolution * 0.5, 0.01)
    steps = int(max_distance_m / step_m)
    point = point_xy.copy()
    for i in range(1, steps + 1):
        point = point_xy + direction_xy * (i * step_m)
        pixel = world_to_pixel(point.reshape(1, 2), info, flip_y)[0]
        col = int(round(pixel[0]))
        row = int(round(pixel[1]))
        if row < 0 or row >= free_mask.shape[0] or col < 0 or col >= free_mask.shape[1]:
            return max(0.0, (i - 1) * step_m)
        if free_mask[row, col] == 0:
            return max(0.0, (i - 1) * step_m)
    return max_distance_m


def raycast_distance_robust(
    free_mask: np.ndarray,
    point_xy: np.ndarray,
    direction_xy: np.ndarray,
    info: MapInfo,
    flip_y: bool,
    max_distance_m: float = 10.0,
    min_wall_pixels: int = 2,
) -> float:
    """
    Robust raycast that requires multiple consecutive wall pixels to avoid noise.
    Returns distance to the first SOLID wall (not a single-pixel noise).
    """
    step_m = max(info.resolution * 0.5, 0.01)
    steps = int(max_distance_m / step_m)

    consecutive_walls = 0
    first_wall_idx = -1

    for i in range(1, steps + 1):
        point = point_xy + direction_xy * (i * step_m)
        pixel = world_to_pixel(point.reshape(1, 2), info, flip_y)[0]
        col = int(round(pixel[0]))
        row = int(round(pixel[1]))

        # Out of bounds = solid wall
        if row < 0 or row >= free_mask.shape[0] or col < 0 or col >= free_mask.shape[1]:
            if first_wall_idx < 0:
                first_wall_idx = i - 1
            consecutive_walls += 1
            if consecutive_walls >= min_wall_pixels:
                return max(0.0, first_wall_idx * step_m)
            continue

        # Wall pixel
        if free_mask[row, col] == 0:
            if first_wall_idx < 0:
                first_wall_idx = i
            consecutive_walls += 1
            if consecutive_walls >= min_wall_pixels:
                return max(0.0, first_wall_idx * step_m)
        else:
            # Free space - reset counter (it was just noise)
            consecutive_walls = 0
            first_wall_idx = -1

    return max_distance_m


def track_widths(
    points_xy: np.ndarray,
    free_mask: np.ndarray,
    info: MapInfo,
    flip_y: bool,
    max_distance_m: float,
    width_mode: str,
) -> tuple[np.ndarray, np.ndarray]:
    if width_mode == "hybrid":
        return track_widths_hybrid(points_xy, free_mask, info, flip_y, max_distance_m)

    if width_mode == "distance":
        dist = cv2.distanceTransform((free_mask > 0).astype(np.uint8), cv2.DIST_L2, 5)
        pixels = np.round(world_to_pixel(points_xy, info, flip_y)).astype(np.int32)
        pixels[:, 0] = np.clip(pixels[:, 0], 0, info.width - 1)
        pixels[:, 1] = np.clip(pixels[:, 1], 0, info.height - 1)
        widths = dist[pixels[:, 1], pixels[:, 0]] * info.resolution
        widths = np.clip(widths, 1e-3, max_distance_m)
        return widths.copy(), widths.copy()

    # raycast mode
    _, psi, _ = headings_and_curvature(points_xy)
    left_normals = normals_from_heading(psi)
    right_normals = -left_normals

    # Use ROBUST raycast (requires 2+ consecutive wall pixels)
    d_left = np.array(
        [
            raycast_distance_robust(free_mask, p, n, info, flip_y, max_distance_m, min_wall_pixels=2)
            for p, n in zip(points_xy, left_normals)
        ]
    )
    d_right = np.array(
        [
            raycast_distance_robust(free_mask, p, n, info, flip_y, max_distance_m, min_wall_pixels=2)
            for p, n in zip(points_xy, right_normals)
        ]
    )
    return d_right, d_left


def track_widths_hybrid(
    points_xy: np.ndarray,
    free_mask: np.ndarray,
    info: MapInfo,
    flip_y: bool,
    max_distance_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """
    HYBRID MODE (recommended): Robust directional raycast that preserves left/right
    asymmetry, with distance-transform used ONLY as a floor sanity-check.

    Design:
    - Robust raycast (requires 2+ consecutive wall pixels) gives the true perpendicular
      distance to each side's wall -> preserves asymmetry when the raceline hugs a curve.
    - Distance transform is NOT used to clamp both sides down (that would destroy the
      asymmetry). It is only used to detect a raycast FAILURE: if a directional raycast
      returns the max distance (hit nothing) but the isotropic min-clearance shows a wall
      is nearby, the raycast likely slipped through a gap -> fall back to min-clearance
      for that single side only.

    Guarantees:
    1. Left/right asymmetry preserved (curve-hugging raceline reports narrow-inside,
       wide-outside).
    2. Single-pixel noise/holes never create false walls (min_wall_pixels gate).
    3. Ray-through-gap failures caught by the min-clearance floor.
    """
    # Isotropic true nearest-wall clearance (used only as a failure floor, not a clamp)
    dist = cv2.distanceTransform((free_mask > 0).astype(np.uint8), cv2.DIST_L2, 5)
    pixels = np.round(world_to_pixel(points_xy, info, flip_y)).astype(np.int32)
    pixels[:, 0] = np.clip(pixels[:, 0], 0, info.width - 1)
    pixels[:, 1] = np.clip(pixels[:, 1], 0, info.height - 1)
    min_clearance = dist[pixels[:, 1], pixels[:, 0]] * info.resolution
    min_clearance = np.clip(min_clearance, 1e-3, max_distance_m)

    # Directional robust raycast for each side
    _, psi, _ = headings_and_curvature(points_xy)
    left_normals = normals_from_heading(psi)
    right_normals = -left_normals

    d_left = np.array(
        [
            raycast_distance_robust(free_mask, p, n, info, flip_y, max_distance_m, min_wall_pixels=2)
            for p, n in zip(points_xy, left_normals)
        ]
    )
    d_right = np.array(
        [
            raycast_distance_robust(free_mask, p, n, info, flip_y, max_distance_m, min_wall_pixels=2)
            for p, n in zip(points_xy, right_normals)
        ]
    )

    # Failure floor (per side, only when raycast clearly overshot through a gap):
    # if the ray hit nothing (>= max_distance) but a wall is actually close, trust the floor.
    ray_failed_left = d_left >= max_distance_m - 1e-6
    ray_failed_right = d_right >= max_distance_m - 1e-6
    d_left = np.where(ray_failed_left, min_clearance, d_left)
    d_right = np.where(ray_failed_right, min_clearance, d_right)

    # Tiny positive floor for downstream safety
    d_left = np.clip(d_left, 1e-3, max_distance_m)
    d_right = np.clip(d_right, 1e-3, max_distance_m)

    return d_right, d_left


def optimize_min_curvature(
    center_xy: np.ndarray,
    d_right: np.ndarray,
    d_left: np.ndarray,
    safety_width: float,
    boundary_margin: float,
    args: argparse.Namespace,
) -> np.ndarray:
    if args.optimizer == "centerline":
        return center_xy

    _, psi, kappa_center = headings_and_curvature(center_xy)
    normals = normals_from_heading(psi)
    clearance = safety_width * 0.5 + boundary_margin

    # Out-out-out 모드: 코너에서 "안쪽" 경계 여유를 keepout만큼 줄여 최적화가 바깥
    # 밴드에 머물게 한다. 안쪽 벽이 라이다를 가리는 코너 너머 은닉 장애물을 더 멀리서
    # 드러내는 것이 목적(P3 커밋에 7~9 m 노출이 필요). kappa>0(좌회전)의 안쪽은 좌측.
    d_left_eff = d_left.astype(np.float64).copy()
    d_right_eff = d_right.astype(np.float64).copy()
    keepout_max = float(getattr(args, "outside_keepout_m", 0.0) or 0.0)
    if keepout_max > 0.0:
        corner_kappa = max(float(getattr(args, "outside_corner_kappa", 0.25)), 1e-6)
        kappa_s = gaussian_filter1d(
            kappa_center, sigma=max(float(getattr(args, "outside_smooth_sigma", 6.0)), 0.1),
            mode="wrap")
        gate = np.clip(np.abs(kappa_s) / corner_kappa, 0.0, 1.0) ** 2
        keepout = keepout_max * gate
        # 안쪽 여유를 줄이되, 최소한 clearance만큼의 밴드는 항상 남긴다(경계 역전 방지).
        floor = clearance + 1e-3
        left_inner = kappa_s > 0.0
        d_left_eff = np.where(
            left_inner, np.maximum(d_left_eff - keepout, floor), d_left_eff)
        d_right_eff = np.where(
            ~left_inner, np.maximum(d_right_eff - keepout, floor), d_right_eff)

    lower = -np.maximum(d_right_eff - clearance, 0.0)
    upper = np.maximum(d_left_eff - clearance, 0.0)
    if np.all(upper <= 1e-3) and np.all(lower >= -1e-3):
        return center_xy

    bounds = list(zip(lower, upper))
    x0 = np.zeros(len(center_xy), dtype=np.float64)

    max_curv = float(getattr(args, "max_curvature", 0.0))

    def objective(alpha: np.ndarray) -> float:
        shifted = center_xy + normals * alpha[:, None]
        seg = np.linalg.norm(np.roll(shifted, -1, axis=0) - shifted, axis=1)
        _, _, kappa = headings_and_curvature(shifted)
        dalpha = np.diff(np.r_[alpha, alpha[0]])
        cost = float(
            args.curvature_weight * np.mean(kappa * kappa)
            + args.smooth_weight * np.mean(dalpha * dalpha)
            + args.length_weight * np.mean(seg)
        )
        if max_curv > 0.0:
            # Bends sharper than the steering limit are undrivable, not just slow.
            # Mean (not sum) keeps the term smooth enough for L-BFGS-B's numerical
            # gradients — a stiff sum-based penalty destabilizes the solve.
            excess = np.maximum(np.abs(kappa) - max_curv, 0.0)
            cost += 25.0 * float(np.mean(excess * excess))
        return cost

    result = minimize(
        objective,
        x0,
        method="L-BFGS-B",
        bounds=bounds,
        options={
            "maxiter": args.max_optimizer_iter,
            "maxfun": max(20000, args.max_optimizer_iter * (len(center_xy) + 1) * 3),
            # 1e-5 lets the min-curvature objective hit a genuine convergence
            # (status 0) within a few hundred iterations; the resulting raceline is
            # within ~3% of the fully-iterated optimum, so a tighter tolerance only
            # buys iteration-limit warnings without a meaningful quality gain.
            "ftol": 1e-5,
            "gtol": 1e-5,
            "maxls": 25,
        },
    )
    # L-BFGS-B status 1 means the iteration/eval limit was reached. The returned x is
    # still the best raceline found so far and is perfectly usable, so we treat that
    # as a benign stop and do not surface it. A finer optimizer-step multiplies the
    # number of optimization variables, which is the usual reason the limit is hit;
    # keeping optimizer-step moderate is the right knob, not a louder warning.
    if not result.success and getattr(result, "status", None) != 1:
        print(f"[WARN] optimizer did not fully converge: {result.message}")

    return center_xy + normals * result.x[:, None]


def limit_curvature_spikes(
    points_xy: np.ndarray,
    max_curv: float,
    iterations: int = 40,
    max_run: int = 3,
) -> np.ndarray:
    """Flatten ISOLATED curvature spikes above the steering limit.

    Piecewise-linear resample vertices and filter/straightening joints show up
    as 1-3 waypoint spikes of huge curvature; blending just those waypoints
    toward their neighbours' midpoint removes them with millimetre-level moves.
    Runs longer than ``max_run`` are real corners — touching them here only
    shifts the kink to the run boundary (measured: it amplified a 6 rad/m
    corner to 17 rad/m), so they are left intact for the validation warning.
    """
    pts = points_xy.copy()
    n = len(pts)
    for _ in range(iterations):
        _, _, kappa = headings_and_curvature(pts)
        bad = np.abs(kappa) > max_curv
        if not bad.any():
            break
        target = np.zeros(n, dtype=bool)
        for start, end in circular_true_runs(bad):
            idxs = circular_indices(start, end, n)
            if len(idxs) <= max_run:
                target[idxs] = True
        if not target.any():
            break
        target = target | np.roll(target, 1) | np.roll(target, -1)
        chord_mid = 0.5 * (np.roll(pts, 1, axis=0) + np.roll(pts, -1, axis=0))
        pts[target] = 0.7 * pts[target] + 0.3 * chord_mid[target]
    return pts


def count_off_map_waypoints(
    points_xy: np.ndarray,
    free_mask: np.ndarray,
    info: MapInfo,
    flip_y: bool,
) -> int:
    """Number of waypoints whose segment to the next waypoint leaves free space.

    The check is done on a densified copy (~2 px spacing) so a segment slicing
    through a thin wall between two waypoints is caught as well.
    """
    dense = resample_closed(points_xy, max(info.resolution * 2.0, 1e-3))
    pixels = np.round(world_to_pixel(dense, info, flip_y)).astype(int)
    cols = np.clip(pixels[:, 0], 0, info.width - 1)
    rows = np.clip(pixels[:, 1], 0, info.height - 1)
    off = free_mask[rows, cols] == 0
    if not bool(np.any(off)):
        return 0
    # Map dense hits back to waypoint count: one per waypoint whose span is hit.
    s_dense, total = cumulative_s(dense)
    s_wpts, _ = cumulative_s(points_xy)
    hit_s = s_dense[off]
    spans = np.searchsorted(s_wpts, hit_s, side="right") - 1
    return int(len(np.unique(np.clip(spans, 0, len(points_xy) - 1))))


def optimize_raceline(
    center_xy: np.ndarray,
    d_right: np.ndarray,
    d_left: np.ndarray,
    args: argparse.Namespace,
) -> np.ndarray:
    """Dispatch to the selected raceline optimizer (centerline/mincurv/laptime/ai)."""
    if args.optimizer not in ("laptime", "ai"):
        return optimize_min_curvature(
            center_xy, d_right, d_left, args.safety_width, args.boundary_margin, args
        )

    # Lazy import: torch/mlx are optional deps, only needed for laptime/ai.
    import sys

    module_dir = str(Path(__file__).resolve().parent)
    if module_dir not in sys.path:
        sys.path.insert(0, module_dir)
    from optimize_laptime import optimize_lap_time, optimize_lap_time_ai

    # The GUI injects progress_log to mirror optimizer progress in its status
    # bar (long ai/laptime runs otherwise look frozen there); CLI keeps print.
    log = getattr(args, "progress_log", print)

    warm_alpha = None
    if not getattr(args, "laptime_no_warm_start", False):
        mincurv_xy = optimize_min_curvature(
            center_xy, d_right, d_left, args.safety_width, args.boundary_margin, args
        )
        _, psi, _ = headings_and_curvature(center_xy)
        normals = normals_from_heading(psi)
        warm_alpha = np.sum((mincurv_xy - center_xy) * normals, axis=1)

    if args.optimizer == "ai":
        def evaluate(points_xy: np.ndarray) -> float:
            # The exact objective the tool reports: velocity_profile lap time
            # after the SAME post-processing the pipeline applies to the winner
            # (spike filter + optimizer_step and waypoint_step resamples; only
            # the straightening pass is skipped as it needs the map). Scoring
            # the raw coarse line instead would reward/punish sampling
            # artifacts the pipeline later removes, ranking candidates by the
            # wrong number.
            pts = filter_and_resample_closed(points_xy, args.optimizer_step, args)
            pts = filter_and_resample_closed(pts, args.waypoint_step, args)
            raceline_sigma = float(getattr(args, "raceline_smooth_sigma", 0.0))
            if raceline_sigma > 0.0:
                pts = smooth_closed(pts, raceline_sigma)
            _, _, kappa = headings_and_curvature(pts)
            _, _, lap = velocity_profile(
                pts, kappa, args.max_speed, args.min_speed,
                args.max_lateral_accel, args.max_accel, args.max_decel,
            )
            max_curv = float(getattr(args, "max_curvature", 0.0))
            if max_curv > 0.0:
                # Undrivable bends must lose the ranking even when the speed
                # model (floored at min_speed) barely penalizes them.
                seg = np.linalg.norm(np.roll(pts, -1, axis=0) - pts, axis=1)
                excess = np.maximum(np.abs(kappa) - max_curv, 0.0)
                lap += 20.0 * float(np.sum(excess * excess * seg))
            return lap

        raceline, _info = optimize_lap_time_ai(
            center_xy,
            d_right,
            d_left,
            args.safety_width,
            args.boundary_margin,
            args,
            evaluate=evaluate,
            warm_alpha=warm_alpha,
            log=log,
        )
        return raceline

    raceline, _info = optimize_lap_time(
        center_xy,
        d_right,
        d_left,
        args.safety_width,
        args.boundary_margin,
        args,
        warm_alpha=warm_alpha,
        log=log,
    )
    return raceline


def velocity_profile(
    points_xy: np.ndarray,
    kappa: np.ndarray,
    max_speed: float,
    min_speed: float,
    max_lateral_accel: float,
    max_accel: float,
    max_decel: float,
) -> tuple[np.ndarray, np.ndarray, float]:
    seg = np.linalg.norm(np.roll(points_xy, -1, axis=0) - points_xy, axis=1)
    curve_speed = np.sqrt(max_lateral_accel / np.maximum(np.abs(kappa), 1e-4))
    v = np.clip(curve_speed, min_speed, max_speed)

    for _ in range(8):
        for i in range(len(v)):
            j = (i + 1) % len(v)
            v[j] = min(v[j], math.sqrt(max(v[i] * v[i] + 2.0 * max_accel * seg[i], 0.0)))
        for i in range(len(v) - 1, -1, -1):
            j = (i - 1) % len(v)
            v[j] = min(v[j], math.sqrt(max(v[i] * v[i] + 2.0 * max_decel * seg[j], 0.0)))

    ax = np.zeros_like(v)
    for i in range(len(v)):
        j = (i + 1) % len(v)
        if seg[i] > 1e-6:
            ax[i] = (v[j] * v[j] - v[i] * v[i]) / (2.0 * seg[i])

    lap_time = float(np.sum(seg / np.maximum(v, 1e-3)))
    return v, ax, lap_time


def circular_true_runs(mask: np.ndarray) -> list[tuple[int, int]]:
    if len(mask) == 0 or not bool(np.any(mask)) or bool(np.all(mask)):
        return []

    n = len(mask)
    false_indices = np.flatnonzero(~mask)
    start_offset = int(false_indices[0] + 1) % n
    rotated = np.roll(mask, -start_offset)

    runs: list[tuple[int, int]] = []
    i = 0
    while i < n:
        if not rotated[i]:
            i += 1
            continue
        start = i
        while i < n and rotated[i]:
            i += 1
        end = i - 1
        runs.append(((start + start_offset) % n, (end + start_offset) % n))
    return runs


def circular_indices(start: int, end: int, count: int) -> np.ndarray:
    if start <= end:
        return np.arange(start, end + 1, dtype=int)
    return np.r_[np.arange(start, count, dtype=int), np.arange(0, end + 1, dtype=int)]


def path_distances_for_indices(points_xy: np.ndarray, indices: np.ndarray) -> tuple[np.ndarray, float]:
    if len(indices) < 2:
        return np.zeros(len(indices), dtype=np.float64), 0.0
    segment_lengths = np.linalg.norm(np.diff(points_xy[indices], axis=0), axis=1)
    distances = np.r_[0.0, np.cumsum(segment_lengths)]
    return distances, float(distances[-1])


def line_has_clearance(
    start_xy: np.ndarray,
    end_xy: np.ndarray,
    free_mask: np.ndarray,
    distance_map_px: np.ndarray,
    info: MapInfo,
    flip_y: bool,
    required_clearance_m: float,
) -> bool:
    delta = end_xy - start_xy
    length = float(np.linalg.norm(delta))
    if length <= info.resolution:
        return False

    step = max(info.resolution * 0.5, 0.01)
    sample_count = max(2, int(math.ceil(length / step)) + 1)
    fractions = np.linspace(0.0, 1.0, sample_count)
    samples = start_xy + fractions[:, None] * delta
    pixels = np.round(world_to_pixel(samples, info, flip_y)).astype(np.int32)
    cols = pixels[:, 0]
    rows = pixels[:, 1]
    in_bounds = (rows >= 0) & (rows < info.height) & (cols >= 0) & (cols < info.width)
    if not bool(np.all(in_bounds)):
        return False
    if not bool(np.all(free_mask[rows, cols] > 0)):
        return False
    clearance = distance_map_px[rows, cols] * info.resolution
    return bool(np.all(clearance >= required_clearance_m))


def straighten_straight_segments(
    points_xy: np.ndarray,
    free_mask: np.ndarray,
    info: MapInfo,
    flip_y: bool,
    args: argparse.Namespace,
) -> np.ndarray:
    if not args.straighten_straights or len(points_xy) < 8:
        return points_xy
    if args.straight_kappa_threshold <= 0.0 or args.straight_min_length <= 0.0:
        return points_xy

    _, _, kappa = headings_and_curvature(points_xy)
    smooth_abs_kappa = gaussian_filter1d(np.abs(kappa), sigma=1.0, mode="wrap")
    straight_mask = smooth_abs_kappa <= args.straight_kappa_threshold
    runs = circular_true_runs(straight_mask)
    if not runs:
        return points_xy

    distance_map_px = cv2.distanceTransform((free_mask > 0).astype(np.uint8), cv2.DIST_L2, 5)
    # Chord validation only needs the car to physically fit (half width plus the
    # straightening margin). boundary_margin is an optimizer-corridor shaping
    # knob; including it here silently disabled straightening on narrow tracks
    # as soon as the margin grew.
    required_clearance = args.safety_width * 0.5 + args.straight_clearance_margin
    straightened = points_xy.copy()
    changed = False

    for start, end in runs:
        indices = circular_indices(start, end, len(points_xy))
        distances, length = path_distances_for_indices(points_xy, indices)
        if len(indices) < 4 or length < args.straight_min_length:
            continue

        start_xy = points_xy[indices[0]]
        end_xy = points_xy[indices[-1]]
        chord = end_xy - start_xy
        chord_length = float(np.linalg.norm(chord))
        if chord_length <= info.resolution or chord_length < args.straight_min_length * 0.5:
            continue
        if not line_has_clearance(
            start_xy,
            end_xy,
            free_mask,
            distance_map_px,
            info,
            flip_y,
            required_clearance,
        ):
            print(
                f"[WARN] straight run of {length:.2f} m not straightened: chord "
                f"clearance is below {required_clearance:.2f} m somewhere along it."
            )
            continue

        fractions = np.divide(distances, length, out=np.zeros_like(distances), where=length > 1e-9)
        line_points = start_xy + fractions[:, None] * chord
        if args.straight_blend_length > 0.0:
            edge_distance = np.minimum(distances, length - distances)
            weights = np.clip(edge_distance / args.straight_blend_length, 0.0, 1.0)
            weights = weights * weights * (3.0 - 2.0 * weights)
        else:
            weights = np.ones_like(distances)
            weights[0] = 0.0
            weights[-1] = 0.0
        straightened[indices] = points_xy[indices] * (1.0 - weights[:, None]) + line_points * weights[:, None]
        changed = True

    return straightened if changed else points_xy


def build_trajectory(
    points_xy: np.ndarray,
    free_mask: np.ndarray,
    info: MapInfo,
    flip_y: bool,
    args: argparse.Namespace,
) -> tuple[Trajectory, float]:
    d_right, d_left = track_widths(
        points_xy, free_mask, info, flip_y, args.max_width_distance, args.width_mode
    )
    s_m, psi, kappa = headings_and_curvature(points_xy)
    vx, ax, lap_time = velocity_profile(
        points_xy,
        kappa,
        args.max_speed,
        args.min_speed,
        args.max_lateral_accel,
        args.max_accel,
        args.max_decel,
    )
    return Trajectory(points_xy, d_right, d_left, s_m, psi, kappa, vx, ax), lap_time


def wpnt_dicts(traj: Trajectory) -> list[dict[str, float | int]]:
    wpnts = []
    for i, point in enumerate(traj.points_xy):
        wpnts.append(
            {
                "id": int(i),
                "s_m": float(traj.s_m[i]),
                "d_m": 0.0,
                "x_m": float(point[0]),
                "y_m": float(point[1]),
                "d_right": float(traj.d_right[i]),
                "d_left": float(traj.d_left[i]),
                "psi_rad": float(traj.psi_rad[i]),
                "kappa_radpm": float(traj.kappa_radpm[i]),
                "vx_mps": float(traj.vx_mps[i]),
                "ax_mps2": float(traj.ax_mps2[i]),
            }
        )
    return wpnts


def wpnt_array(traj: Trajectory) -> dict:
    return {
        "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
        "wpnts": wpnt_dicts(traj),
    }


def csv_number(value: float) -> str:
    rounded = round(float(value), 6)
    if abs(rounded) < 0.0000005:
        rounded = 0.0
    return f"{rounded:.6f}".rstrip("0").rstrip(".")


def trajectory_csv_rows(traj: Trajectory) -> list[dict[str, str | int]]:
    rows = []
    for i, point in enumerate(traj.points_xy):
        rows.append(
            {
                "id": int(i),
                "s": csv_number(traj.s_m[i]),
                "x_m": csv_number(point[0]),
                "y_m": csv_number(point[1]),
                "psi_rad": csv_number(traj.psi_rad[i]),
                "kappa_radpm": csv_number(traj.kappa_radpm[i]),
                "vx_mps": csv_number(traj.vx_mps[i]),
                "ax_mps2": csv_number(traj.ax_mps2[i]),
                "d_left": csv_number(traj.d_left[i]),
                "d_right": csv_number(traj.d_right[i]),
            }
        )
    return rows


def write_csv(path: Path, rows: Iterable[dict]) -> None:
    rows = list(rows)
    if not rows:
        raise RuntimeError(f"No rows to write: {path}")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def write_centerline_csv(path: Path, traj: Trajectory) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["x_m", "y_m", "d_right", "d_left"])
        for point, d_right, d_left in zip(traj.points_xy, traj.d_right, traj.d_left):
            writer.writerow([float(point[0]), float(point[1]), float(d_right), float(d_left)])


def write_outputs(
    output_dir: Path,
    map_info: MapInfo,
    center_traj: Trajectory,
    global_traj: Trajectory,
    lap_time: float,
    args: argparse.Namespace,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    write_centerline_csv(output_dir / "centerline.csv", center_traj)
    write_csv(output_dir / "global_waypoints.csv", trajectory_csv_rows(global_traj))

    payload = {
        "map_info_str": {
            "data": (
                f"offline generator; map={map_info.yaml_path}; "
                f"optimizer={args.optimizer}; estimated_lap_time={lap_time:.3f}s"
            )
        },
        "est_lap_time": {"data": lap_time},
        "centerline_markers": {"markers": []},
        "centerline_waypoints": wpnt_array(center_traj),
        "global_traj_markers_iqp": {"markers": []},
        "global_traj_wpnts_iqp": wpnt_array(global_traj),
        "global_traj_markers_sp": {"markers": []},
        "global_traj_wpnts_sp": wpnt_array(global_traj),
        "trackbounds_markers": {"markers": []},
    }
    (output_dir / "global_waypoints.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")

    metadata = {
        "map_yaml": str(map_info.yaml_path),
        "map_image": str(map_info.image_path),
        "resolution": map_info.resolution,
        "origin": [map_info.origin_x, map_info.origin_y, map_info.origin_yaw],
        "waypoint_count": int(len(global_traj.points_xy)),
        "centerline_count": int(len(center_traj.points_xy)),
        "estimated_lap_time_sec": lap_time,
        "args": {
            key: str(value) if isinstance(value, Path) else value
            for key, value in vars(args).items()
            if not callable(value)  # GUI injects progress_log; keep metadata JSON-safe
        },
    }
    (output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")


def write_debug_image(
    output_dir: Path,
    image: np.ndarray,
    map_info: MapInfo,
    center_xy: np.ndarray,
    global_xy: np.ndarray,
    flip_y: bool,
) -> None:
    debug = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    for points_xy, color in ((center_xy, (255, 0, 0)), (global_xy, (0, 0, 255))):
        pixels = np.round(world_to_pixel(points_xy, map_info, flip_y)).astype(np.int32)
        pixels[:, 0] = np.clip(pixels[:, 0], 0, map_info.width - 1)
        pixels[:, 1] = np.clip(pixels[:, 1], 0, map_info.height - 1)
        for p0, p1 in zip(pixels, np.roll(pixels, -1, axis=0)):
            cv2.line(debug, tuple(p0), tuple(p1), color, 1)
    cv2.imwrite(str(output_dir / "debug_overlay.png"), debug)


def generate_trajectory(args: argparse.Namespace) -> GenerationResult:
    validate_args(args)
    map_info, image, free_raw = load_map(args.map_yaml, args.unknown_as_free)
    free_mask = cleanup_free_mask(
        free_raw,
        args.median_kernel,
        args.morph_kernel,
        args.morph_open_iterations,
        args.morph_close_iterations,
        occupied_mask=occupied_mask_from(image, map_info),
    )
    skeleton = skeletonize(free_mask)
    min_track_width = float(getattr(args, "min_track_width", 0.0))
    if min_track_width > 0.0:
        # The car cannot drive where the corridor is narrower than the track
        # width, so any skeleton pixel there is scan noise, not centerline.
        clearance_px = cv2.distanceTransform((free_mask > 0).astype(np.uint8), cv2.DIST_L2, 5)
        too_narrow = clearance_px * (2.0 * map_info.resolution) < min_track_width
        skeleton = np.where(too_narrow, 0, skeleton).astype(np.uint8)
    center_px = extract_centerline_pixels(
        skeleton,
        args.skeleton_prune_iterations,
        args.min_skeleton_component_area,
    )
    flip_y = not args.no_flip_y
    center_xy = pixel_to_world(center_px, map_info, flip_y)
    center_xy = smooth_closed(center_xy, args.smooth_sigma)
    center_xy = filter_and_resample_closed(center_xy, args.optimizer_step, args)
    if args.reverse:
        center_xy = center_xy[::-1].copy()

    center_right, center_left = track_widths(
        center_xy, free_mask, map_info, flip_y, args.max_width_distance, args.width_mode
    )
    optimized_xy = optimize_raceline(center_xy, center_right, center_left, args)
    optimized_xy = filter_and_resample_closed(optimized_xy, args.optimizer_step, args)
    global_xy = filter_and_resample_closed(optimized_xy, args.waypoint_step, args)
    raceline_sigma = float(getattr(args, "raceline_smooth_sigma", 0.0))
    if raceline_sigma > 0.0:
        # Remove the piecewise-linear vertex kinks left by the coarse optimizer
        # grid (they read as phantom steering-limit violations at fine spacing).
        global_xy = smooth_closed(global_xy, raceline_sigma)
    straightened_xy = straighten_straight_segments(global_xy, free_mask, map_info, flip_y, args)
    if straightened_xy is not global_xy:
        global_xy = resample_closed(straightened_xy, args.waypoint_step)
    max_curv = float(getattr(args, "max_curvature", 0.0))
    if max_curv > 0.0:
        global_xy = limit_curvature_spikes(global_xy, max_curv)

    center_output_xy = filter_and_resample_closed(center_xy, args.waypoint_step, args)
    center_traj, _ = build_trajectory(center_output_xy, free_mask, map_info, flip_y, args)
    global_traj, lap_time = build_trajectory(global_xy, free_mask, map_info, flip_y, args)

    off_map = count_off_map_waypoints(global_traj.points_xy, free_mask, map_info, flip_y)
    if off_map:
        print(
            f"[WARN] {off_map}/{len(global_traj.points_xy)} waypoints lie outside the "
            "drivable free space — the extraction likely picked a noise region. "
            "Check debug_overlay.png; raise --min-track-width or the map-cleanup "
            "parameters (median/morph kernels)."
        )

    max_abs_kappa = float(np.max(np.abs(global_traj.kappa_radpm)))
    kappa_violations = 0
    if max_curv > 0.0:
        kappa_violations = int(np.sum(np.abs(global_traj.kappa_radpm) > max_curv))
        if kappa_violations:
            print(
                f"[WARN] {kappa_violations}/{len(global_traj.points_xy)} waypoints exceed "
                f"the steering limit --max-curvature {max_curv:.2f} rad/m "
                f"(max |kappa| = {max_abs_kappa:.2f}) — the raceline is not drivable "
                "as-is. Widen --safety-width/--boundary-margin, raise --smooth-sigma, "
                "or check whether the track corner itself is tighter than the car's "
                "turning radius."
            )

    return GenerationResult(
        map_info=map_info,
        image=image,
        free_mask=free_mask,
        center_traj=center_traj,
        global_traj=global_traj,
        lap_time=lap_time,
        flip_y=flip_y,
        off_map_wpnts=off_map,
        kappa_violations=kappa_violations,
        max_abs_kappa=max_abs_kappa,
    )


def main() -> int:
    args = parse_args()
    result = generate_trajectory(args)

    output_dir = args.output_dir
    if output_dir is None:
        output_dir = default_output_dir(args.map_yaml)

    write_outputs(output_dir, result.map_info, result.center_traj, result.global_traj, result.lap_time, args)
    if args.debug_image:
        write_debug_image(
            output_dir,
            result.image,
            result.map_info,
            result.center_traj.points_xy,
            result.global_traj.points_xy,
            result.flip_y,
        )

    print(f"Wrote {output_dir / 'global_waypoints.json'}")
    print(f"Wrote {output_dir / 'global_waypoints.csv'}")
    print(f"Waypoints: {len(result.global_traj.points_xy)}, estimated lap time: {result.lap_time:.3f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
