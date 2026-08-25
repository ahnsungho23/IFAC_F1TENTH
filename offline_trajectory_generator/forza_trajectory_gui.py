#!/usr/bin/env python3
"""Offline GUI port of ForzaETH race_stack's Jazzy global planner.

Source revisions used for this port:
  race_stack (ros2-jazzy): 202450f51081b618fcca71924ad6d33f44f90e44
  global_racetrajectory_optimization submodule:
    b1b38ecfefec7200d5bcb243f115af1d795b3766

Only the centerline and iterative minimum-curvature (mincurv_iqp) branches are
ported. ROS publishers, mapping services, shortest-path generation, and live
vehicle-pose handling are intentionally outside this standalone tool.

Shares no file with trajectory_gui.py (the C++ generator's GUI): the pieces it
used to import from there now live in forza_preview.py and forza_common.py.

race_stack is MIT (Copyright (c) 2024 ForzaETH). The optimization modules it
depends on are LGPL-3.0 and are vendored under vendor/ with their LICENSE.
"""

from __future__ import annotations

import argparse
import configparser
import json
import math
import sys
import threading
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import tkinter as tk
import yaml
from scipy.signal import savgol_filter
from scipy.spatial import cKDTree
from skimage.morphology import skeletonize
from skimage.segmentation import watershed
from tkinter import filedialog, messagebox, ttk

from forza_common import (
    GenerationResult,
    MapInfo,
    Trajectory,
    count_off_map_waypoints,
    default_output_dir,
    load_map,
    pixel_to_world,
    world_to_pixel,
    write_debug_image,
    write_outputs,
)
from forza_preview import render_preview_rgb, rgb_to_photoimage


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_PARAMS_YAML = SCRIPT_DIR / "forza_gui_params.yaml"
DEFAULT_OPTIMIZER_CONFIG_DIR = SCRIPT_DIR / "config" / "forza"
VENDOR_SRC = SCRIPT_DIR / "vendor"
HELPER_FUNCS_SRC = VENDOR_SRC / "helper_funcs_glob"
RACE_STACK_COMMIT = "202450f51081b618fcca71924ad6d33f44f90e44"
OPTIMIZER_COMMIT = "b1b38ecfefec7200d5bcb243f115af1d795b3766"

# vendor/ holds the exact optimization submodule revision used by the referenced
# ForzaETH Jazzy branch (transitive closure only; see vendor/README.md). Keeping
# it inside this package means the GUI needs no ROS runtime, no colcon workspace
# and no separately installed trajectory_planning_helpers.
if str(VENDOR_SRC) not in sys.path:
    sys.path.insert(0, str(VENDOR_SRC))
if str(HELPER_FUNCS_SRC) not in sys.path:
    sys.path.insert(0, str(HELPER_FUNCS_SRC))

import trajectory_planning_helpers as tph  # noqa: E402
import interp_track as forza_interp_track  # noqa: E402
import prep_track as forza_prep_track  # noqa: E402


@dataclass(frozen=True)
class TrackBounds:
    right: np.ndarray | None
    left: np.ndarray | None
    distance_transform: np.ndarray | None
    used_watershed: bool


@dataclass(frozen=True)
class GenerationWarning:
    title: str
    message: str
    suggestions: tuple[str, ...]
    current_parameters: str
    technical_detail: str


def generation_warning_from_exception(
    exc: Exception,
    args: argparse.Namespace,
) -> GenerationWarning:
    """Convert worker failures into a non-modal, actionable GUI warning."""
    detail = f"{type(exc).__name__}: {exc}"
    normalized = str(exc).lower()
    current_parameters = (
        f"현재 설정: Expected center length "
        f"{float(args.expected_centerline_length):.1f} m · "
        f"Safety width {float(args.safety_width):.2f} m · "
        f"Max curvature {float(args.max_curvature):.2f} rad/m · "
        f"Max speed {float(args.max_speed):.2f} m/s"
    )

    if "constraints are inconsistent" in normalized:
        return GenerationWarning(
            title="IQP 경로 생성 불가",
            message=(
                "현재 centerline, 트랙 폭, Safety width 및 곡률 제한을 동시에 "
                "만족하는 경로가 없습니다."
            ),
            suggestions=(
                "Expected center length를 실제 한 바퀴 길이에 맞추세요.",
                "Safety width가 검출된 최소 트랙 폭보다 작은지 확인하세요.",
                "차량 조향 한계가 허용하면 Max curvature를 조금 높이세요.",
                "맵의 작은 폐곡선, 좁은 구간 및 경계 노이즈를 정리하세요.",
                "Safety width는 실제 차량 폭보다 작게 설정하지 마세요.",
            ),
            current_parameters=current_parameters,
            technical_detail=detail,
        )

    if "problem not solvable" in normalized and "track might be too small" in normalized:
        return GenerationWarning(
            title="트랙 폭 부족",
            message="검출된 트랙 폭이 현재 Safety width를 수용하지 못합니다.",
            suggestions=(
                "Safety width가 검출된 최소 트랙 폭보다 작은지 확인하세요.",
                "맵 경계가 안쪽으로 돌출되거나 끊어진 구간을 정리하세요.",
                "Safety width는 실제 차량 폭보다 작게 설정하지 마세요.",
            ),
            current_parameters=current_parameters,
            technical_detail=detail,
        )

    return GenerationWarning(
        title="경로 생성 실패",
        message="현재 파라미터로 trajectory를 생성하지 못했습니다.",
        suggestions=(
            "왼쪽 입력값과 선택한 map/config 경로를 확인하세요.",
            "파라미터를 조절한 뒤 Rebuild를 다시 누르세요.",
        ),
        current_parameters=current_parameters,
        technical_detail=detail,
    )


def render_forza_preview_rgb(
    result: GenerationResult,
    target_size: tuple[int, int] | None,
    show_centerline: bool,
    show_raceline: bool,
) -> np.ndarray:
    """Render the Forza raceline at exactly one output-image pixel."""
    image_rgb = render_preview_rgb(result, target_size, show_centerline, False)
    if not show_raceline:
        return image_rgb

    pixels = world_to_pixel(result.global_traj.points_xy, result.map_info, result.flip_y)
    scale_x = image_rgb.shape[1] / result.map_info.width
    scale_y = image_rgb.shape[0] / result.map_info.height
    pixels[:, 0] = (pixels[:, 0] + 0.5) * scale_x - 0.5
    pixels[:, 1] = (pixels[:, 1] + 0.5) * scale_y - 0.5
    pixels = np.round(pixels).astype(np.int32)
    pixels[:, 0] = np.clip(pixels[:, 0], 0, image_rgb.shape[1] - 1)
    pixels[:, 1] = np.clip(pixels[:, 1], 0, image_rgb.shape[0] - 1)
    for p0, p1 in zip(pixels, np.roll(pixels, -1, axis=0)):
        cv2.line(image_rgb, tuple(p0), tuple(p1), (255, 70, 45), 1, cv2.LINE_8)

    # The shared renderer draws the start marker before this output-space
    # overlay, so redraw it to keep the green marker above the red raceline.
    start = tuple(pixels[0])
    marker_radius = max(3, int(round(3 * min(scale_x, scale_y))))
    cv2.circle(image_rgb, start, marker_radius, (0, 220, 90), -1, cv2.LINE_AA)
    return image_rgb


def default_values() -> dict[str, Any]:
    return {
        "map_yaml": "",
        "output_dir": "",
        "optimizer_config_dir": str(DEFAULT_OPTIMIZER_CONFIG_DIR),
        "occupancy_grid_threshold": 10.0,
        "filter_kernel_size": 9,
        "expected_centerline_length": 0.0,
        "safety_width": 0.7,
        "max_curvature": 1.0,
        "max_speed": 15.0,
        "longitudinal_accel_scale": 1.0,
        "lateral_accel_scale": 1.0,
        "machine_accel_scale": 1.0,
        "dynamic_model_exponent": 1.0,
        "velocity_filter_window": 0,
        "reverse": False,
        "show_centerline": True,
        "show_raceline": True,
        "debug_image": True,
    }


def load_gui_params(path: Path) -> dict[str, Any]:
    values = default_values()
    if path.is_file():
        with path.open("r", encoding="utf-8") as stream:
            loaded = yaml.safe_load(stream) or {}
        if isinstance(loaded, dict):
            for key in values:
                if key in loaded:
                    values[key] = loaded[key]
    return values


def save_gui_params(path: Path, values: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(values, stream, sort_keys=False, allow_unicode=True)


def make_generation_args(values: dict[str, Any]) -> argparse.Namespace:
    map_text = str(values.get("map_yaml", "")).strip()
    if not map_text:
        raise ValueError("map YAML을 선택하세요.")
    map_yaml = Path(map_text).expanduser().resolve()
    if not map_yaml.is_file():
        raise ValueError(f"map YAML이 없습니다: {map_yaml}")

    config_dir = Path(str(values["optimizer_config_dir"])).expanduser()
    if not config_dir.is_absolute():
        config_dir = SCRIPT_DIR / config_dir
    config_dir = config_dir.resolve()
    required = (
        config_dir / "racecar_f110.ini",
        config_dir / "veh_dyn_info/ggv.csv",
        config_dir / "veh_dyn_info/ax_max_machines.csv",
    )
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise ValueError("Forza optimizer 설정 파일이 없습니다: " + ", ".join(missing))

    filter_kernel = int(values["filter_kernel_size"])
    if filter_kernel < 1:
        raise ValueError("Filter kernel은 1 이상이어야 합니다.")
    safety_width = float(values["safety_width"])
    if safety_width <= 0.0:
        raise ValueError("Safety width는 0보다 커야 합니다.")
    max_curvature = float(values["max_curvature"])
    if max_curvature <= 0.0:
        raise ValueError("Max curvature는 0보다 커야 합니다.")
    threshold = float(values["occupancy_grid_threshold"])
    if not 0.0 < threshold <= 100.0:
        raise ValueError("Occupancy threshold는 0보다 크고 100 이하여야 합니다.")
    expected_length = float(values["expected_centerline_length"])
    if expected_length < 0.0:
        raise ValueError("Expected centerline length는 0 이상이어야 합니다.")

    max_speed = float(values["max_speed"])
    if max_speed <= 0.0:
        raise ValueError("Max speed는 0보다 커야 합니다.")
    longitudinal_accel_scale = float(values["longitudinal_accel_scale"])
    lateral_accel_scale = float(values["lateral_accel_scale"])
    machine_accel_scale = float(values["machine_accel_scale"])
    if min(
        longitudinal_accel_scale,
        lateral_accel_scale,
        machine_accel_scale,
    ) <= 0.0:
        raise ValueError("가속도 배율은 모두 0보다 커야 합니다.")
    dynamic_model_exponent = float(values["dynamic_model_exponent"])
    if not 1.0 <= dynamic_model_exponent <= 2.0:
        raise ValueError("Dynamic model exponent는 1.0~2.0이어야 합니다.")
    velocity_filter_window = int(values["velocity_filter_window"])
    if velocity_filter_window != 0 and (
        velocity_filter_window < 3 or velocity_filter_window % 2 == 0
    ):
        raise ValueError("Velocity filter window은 0(끄기) 또는 3 이상의 홀수여야 합니다.")

    output_text = str(values.get("output_dir", "")).strip()
    return argparse.Namespace(
        map_yaml=map_yaml,
        output_dir=Path(output_text).expanduser().resolve() if output_text else None,
        optimizer_config_dir=config_dir,
        occupancy_grid_threshold=threshold,
        filter_kernel_size=filter_kernel,
        expected_centerline_length=expected_length,
        safety_width=safety_width,
        max_curvature=max_curvature,
        max_speed=max_speed,
        longitudinal_accel_scale=longitudinal_accel_scale,
        lateral_accel_scale=lateral_accel_scale,
        machine_accel_scale=machine_accel_scale,
        dynamic_model_exponent=dynamic_model_exponent,
        velocity_filter_window=velocity_filter_window,
        reverse=bool(values.get("reverse", False)),
        debug_image=bool(values.get("debug_image", True)),
        optimizer="forza_mincurv_iqp",
    )


def forza_filter_map(
    image: np.ndarray,
    info: MapInfo,
    occupancy_grid_threshold: float,
    filter_kernel_size: int,
) -> np.ndarray:
    """Port of GlobalPlannerLogic.filter_map for a map-server image."""
    normalized = image.astype(np.float64) / 255.0
    occupancy_percent = (normalized if info.negate else 1.0 - normalized) * 100.0
    bw = np.where(occupancy_percent < occupancy_grid_threshold, 255, 0).astype(np.uint8)
    kernel = np.ones((filter_kernel_size, filter_kernel_size), np.uint8)
    return cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel, iterations=2)


def extract_centerline(
    skeleton: np.ndarray,
    map_resolution: float,
    expected_length: float = 0.0,
) -> np.ndarray:
    """Exact shortest-closed-contour selection used by ForzaETH."""
    contours, hierarchy = cv2.findContours(skeleton, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if hierarchy is None:
        raise RuntimeError("skeleton에서 폐곡선을 찾지 못했습니다.")

    closed_contours: list[np.ndarray] = []
    for i, contour in enumerate(contours):
        opened = hierarchy[0][i][2] < 0 and hierarchy[0][i][3] < 0
        if not opened:
            closed_contours.append(contour)
    if not closed_contours:
        raise RuntimeError("skeleton에서 폐곡선을 찾지 못했습니다.")

    line_lengths: list[float] = []
    for contour in closed_contours:
        points = contour[:, 0, :].astype(np.float64)
        length = (
            float(np.linalg.norm(points - np.roll(points, 1, axis=0), axis=1).sum())
            * map_resolution
        )
        if expected_length > 0.0 and abs(expected_length / length - 1.0) >= 0.15:
            length = math.inf
        line_lengths.append(length)
    if min(line_lengths) == math.inf:
        raise RuntimeError(
            "예상 centerline 길이의 ±15% 안에 드는 폐곡선이 없습니다."
        )
    return closed_contours[int(np.argmin(line_lengths))][:, 0, :].astype(np.float64)


def smooth_centerline(centerline: np.ndarray) -> np.ndarray:
    """Port of ForzaETH's two-pass Savitzky-Golay seam smoothing."""
    centerline_length = len(centerline)
    if centerline_length > 2000:
        filter_length = int(centerline_length / 200) * 10 + 1
    elif centerline_length > 1000:
        filter_length = 81
    elif centerline_length > 500:
        filter_length = 41
    else:
        filter_length = 21
    if centerline_length < filter_length:
        raise RuntimeError(
            f"centerline 점이 {centerline_length}개뿐이라 Forza 평활 창 {filter_length}을 적용할 수 없습니다."
        )

    centerline_smooth = savgol_filter(centerline, filter_length, 3, axis=0)
    half = int(centerline_length / 2)
    rotated = np.append(centerline[half:], centerline[:half], axis=0)
    rotated_smooth = savgol_filter(rotated, filter_length, 3, axis=0)
    centerline_smooth[:filter_length] = rotated_smooth[half:half + filter_length]
    centerline_smooth[-filter_length:] = rotated_smooth[half - filter_length:half]
    return centerline_smooth


def interp_closed_track(points_xy: np.ndarray, step: float = 0.1) -> np.ndarray:
    reftrack = np.column_stack((points_xy, np.zeros((len(points_xy), 2))))
    return forza_interp_track.interp_track(reftrack=reftrack, stepsize_approx=step)[:, :2]


def closed_contours_from_mask(mask: np.ndarray) -> list[np.ndarray]:
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if hierarchy is None:
        return []
    closed: list[np.ndarray] = []
    for i, contour in enumerate(contours):
        opened = hierarchy[0][i][2] < 0 and hierarchy[0][i][3] < 0
        if not opened:
            closed.append(contour)
    return closed


def classify_bounds_by_path_direction(
    first: np.ndarray,
    second: np.ndarray,
    centerline_meter: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Offline replacement for ForzaETH's live initial vehicle pose check."""
    center = centerline_meter[0]
    tangent = centerline_meter[1] - centerline_meter[-1]
    norm = float(np.linalg.norm(tangent))
    if norm <= 1e-9:
        tangent = centerline_meter[1] - centerline_meter[0]
        norm = float(np.linalg.norm(tangent))
    tangent /= max(norm, 1e-9)
    right_normal = np.array([tangent[1], -tangent[0]])

    def signed_nearest(boundary: np.ndarray) -> float:
        nearest = boundary[int(np.argmin(np.linalg.norm(boundary - center, axis=1)))]
        return float(np.dot(nearest - center, right_normal))

    if signed_nearest(first) >= signed_nearest(second):
        return first, second
    return second, first


def extract_track_bounds(
    centerline_pixels: np.ndarray,
    centerline_meter: np.ndarray,
    filtered_map: np.ndarray,
    info: MapInfo,
) -> TrackBounds:
    """Port ForzaETH's watershed boundary extraction with its DT fallback."""
    center_image = np.zeros(filtered_map.shape, dtype=np.uint8)
    cv2.drawContours(center_image, [centerline_pixels.astype(np.int32)], 0, 255, 2, cv2.LINE_8)
    _, markers = cv2.connectedComponents(center_image)
    distance_transform = cv2.distanceTransform(filtered_map, cv2.DIST_L2, 5)
    labels = watershed(-distance_transform, markers, mask=filtered_map.astype(bool))

    closed: list[np.ndarray] = []
    for label in np.unique(labels):
        if label == 0:
            continue
        label_mask = np.zeros(filtered_map.shape, dtype=np.uint8)
        label_mask[labels == label] = 255
        closed.extend(closed_contours_from_mask(label_mask))

    if len(closed) != 2:
        return TrackBounds(None, None, distance_transform, False)

    outer_pixels = max(closed, key=len)[:, 0, :].astype(np.float64)
    inner_pixels = min(closed, key=len)[:, 0, :].astype(np.float64)
    outer_meter = pixel_to_world(outer_pixels, info, flip_y=True)
    inner_meter = pixel_to_world(inner_pixels, info, flip_y=True)
    right, left = classify_bounds_by_path_direction(outer_meter, inner_meter, centerline_meter)
    return TrackBounds(right, left, None, True)


def distances_to_bounds(
    points_xy: np.ndarray, right: np.ndarray, left: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    right_interp = interp_closed_track(right)
    left_interp = interp_closed_track(left)
    d_right = cKDTree(right_interp).query(points_xy, k=1)[0]
    d_left = cKDTree(left_interp).query(points_xy, k=1)[0]
    return np.asarray(d_right), np.asarray(d_left)


def centerline_with_widths(
    centerline_pixels: np.ndarray,
    centerline_meter: np.ndarray,
    bounds: TrackBounds,
    info: MapInfo,
) -> np.ndarray:
    if bounds.used_watershed and bounds.right is not None and bounds.left is not None:
        d_right, d_left = distances_to_bounds(centerline_meter, bounds.right, bounds.left)
    elif bounds.distance_transform is not None:
        cols = np.clip(centerline_pixels[:, 0].astype(int), 0, info.width - 1)
        rows = np.clip(centerline_pixels[:, 1].astype(int), 0, info.height - 1)
        widths_raw = bounds.distance_transform[rows, cols] * info.resolution
        if len(widths_raw) != len(centerline_meter):
            widths = np.interp(
                np.arange(len(centerline_meter)),
                np.arange(len(widths_raw)),
                widths_raw,
            )
        else:
            widths = widths_raw
        d_right = widths
        d_left = widths
    else:
        raise RuntimeError("트랙 경계 거리를 계산할 수 없습니다.")
    return np.column_stack((centerline_meter, d_right, d_left))


def read_forza_parameters(config_dir: Path) -> dict[str, Any]:
    parser = configparser.ConfigParser()
    if not parser.read(config_dir / "racecar_f110.ini"):
        raise RuntimeError("racecar_f110.ini를 읽지 못했습니다.")
    return {
        "ggv_file": json.loads(parser.get("GENERAL_OPTIONS", "ggv_file")),
        "ax_max_machines_file": json.loads(
            parser.get("GENERAL_OPTIONS", "ax_max_machines_file")
        ),
        "stepsize_opts": json.loads(parser.get("GENERAL_OPTIONS", "stepsize_opts")),
        "reg_smooth_opts": json.loads(parser.get("GENERAL_OPTIONS", "reg_smooth_opts")),
        "veh_params": json.loads(parser.get("GENERAL_OPTIONS", "veh_params")),
        "vel_calc_opts": json.loads(parser.get("GENERAL_OPTIONS", "vel_calc_opts")),
        "optim_opts": json.loads(
            parser.get("OPTIMIZATION_OPTIONS", "optim_opts_mincurv")
        ),
    }


def optimize_min_curvature_iqp(
    reftrack_imported: np.ndarray,
    config_dir: Path,
    safety_width: float,
    max_curvature: float,
    max_speed: float,
    longitudinal_accel_scale: float,
    lateral_accel_scale: float,
    machine_accel_scale: float,
    dynamic_model_exponent: float,
    velocity_filter_window: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, float]:
    """The mincurv_iqp branch from ForzaETH trajectory_optimizer.py."""
    pars = read_forza_parameters(config_dir)
    ggv, ax_max_machines = tph.import_veh_dyn_info.import_veh_dyn_info(
        ggv_import_path=str(config_dir / "veh_dyn_info" / pars["ggv_file"]),
        ax_max_machines_import_path=str(
            config_dir / "veh_dyn_info" / pars["ax_max_machines_file"]
        ),
    )
    max_profile_speed = min(float(ggv[-1, 0]), float(ax_max_machines[-1, 0]))
    if max_speed > max_profile_speed:
        raise ValueError(
            f"Max speed {max_speed:.2f} m/s를 GGV/모터 데이터가 덮지 못합니다. "
            f"{max_profile_speed:.2f} m/s 이하로 설정하세요."
        )
    ggv = np.array(ggv, dtype=np.float64, copy=True)
    ax_max_machines = np.array(ax_max_machines, dtype=np.float64, copy=True)
    ggv[:, 1] *= longitudinal_accel_scale
    ggv[:, 2] *= lateral_accel_scale
    ax_max_machines[:, 1] *= machine_accel_scale

    reftrack, normvectors, spline_matrix, _, _ = forza_prep_track.prep_track(
        reftrack_imp=reftrack_imported,
        reg_smooth_opts=pars["reg_smooth_opts"],
        stepsize_opts=pars["stepsize_opts"],
        debug=True,
        min_width=None,
    )
    alpha, reftrack, normvectors, _, _, _, _ = tph.iqp_handler.iqp_handler(
        reftrack=reftrack,
        normvectors=normvectors,
        A=spline_matrix,
        spline_len=np.zeros(reftrack.shape),
        psi=np.zeros(reftrack.shape),
        kappa=np.zeros(reftrack.shape),
        dkappa=np.zeros(reftrack.shape),
        kappa_bound=max_curvature,
        w_veh=safety_width,
        print_debug=True,
        plot_debug=False,
        stepsize_interp=pars["stepsize_opts"]["stepsize_reg"],
        iters_min=pars["optim_opts"]["iqp_iters_min"],
        curv_error_allowed=pars["optim_opts"]["iqp_curverror_allowed"],
    )

    (
        raceline,
        _,
        coeffs_x,
        coeffs_y,
        spline_indices,
        spline_parameters,
        s_points,
        spline_lengths,
        element_lengths,
    ) = tph.create_raceline.create_raceline(
        refline=reftrack[:, :2],
        normvectors=normvectors,
        alpha=alpha,
        stepsize_interp=pars["stepsize_opts"]["stepsize_interp_after_opt"],
    )
    psi, kappa = tph.calc_head_curv_an.calc_head_curv_an(
        coeffs_x=coeffs_x,
        coeffs_y=coeffs_y,
        ind_spls=spline_indices,
        t_spls=spline_parameters,
    )
    velocity = tph.calc_vel_profile.calc_vel_profile(
        ggv=ggv,
        ax_max_machines=ax_max_machines,
        v_max=max_speed,
        kappa=kappa,
        el_lengths=element_lengths,
        closed=True,
        filt_window=velocity_filter_window or None,
        dyn_model_exp=dynamic_model_exponent,
        drag_coeff=pars["veh_params"]["dragcoeff"],
        m_veh=pars["veh_params"]["mass"],
    )
    acceleration = tph.calc_ax_profile.calc_ax_profile(
        vx_profile=np.append(velocity, velocity[0]),
        el_lengths=element_lengths,
        eq_length_output=False,
    )
    time_profile = tph.calc_t_profile.calc_t_profile(
        vx_profile=velocity,
        ax_profile=acceleration,
        el_lengths=element_lengths,
    )

    trajectory = np.column_stack((s_points, raceline, psi, kappa, velocity, acceleration))
    closed_trajectory = np.vstack((trajectory, trajectory[0]))
    closed_trajectory[-1, 0] = float(np.sum(spline_lengths))

    bound_right = reftrack[:, :2] + reftrack[:, 2, None] * normvectors
    bound_left = reftrack[:, :2] - reftrack[:, 3, None] * normvectors
    return closed_trajectory, bound_right, bound_left, float(time_profile[-1]), max_curvature


def normalize_psi(psi: np.ndarray) -> np.ndarray:
    # trajectory_planning_helpers uses north as zero; f110_msgs/map uses +x.
    return (psi + math.pi / 2.0 + math.pi) % (2.0 * math.pi) - math.pi


def build_center_trajectory(centerline: np.ndarray) -> Trajectory:
    points = centerline[:, :2]
    psi, kappa = tph.calc_head_curv_num.calc_head_curv_num(
        path=points,
        el_lengths=0.1 * np.ones(len(points) - 1),
        is_closed=False,
    )
    return Trajectory(
        points_xy=points,
        d_right=centerline[:, 2],
        d_left=centerline[:, 3],
        s_m=np.arange(len(points), dtype=np.float64) * 0.1,
        psi_rad=normalize_psi(np.asarray(psi)),
        kappa_radpm=np.asarray(kappa),
        vx_mps=np.zeros(len(points)),
        ax_mps2=np.zeros(len(points)),
    )


def build_global_trajectory(
    forza_trajectory: np.ndarray,
    right: np.ndarray,
    left: np.ndarray,
) -> Trajectory:
    points = forza_trajectory[:, 1:3]
    d_right, d_left = distances_to_bounds(points, right, left)
    return Trajectory(
        points_xy=points,
        d_right=d_right,
        d_left=d_left,
        s_m=forza_trajectory[:, 0],
        psi_rad=normalize_psi(forza_trajectory[:, 3]),
        kappa_radpm=forza_trajectory[:, 4],
        vx_mps=forza_trajectory[:, 5],
        ax_mps2=forza_trajectory[:, 6],
    )


def generate_forza_trajectory(args: argparse.Namespace) -> GenerationResult:
    info, image, _ = load_map(args.map_yaml, unknown_as_free=False)
    filtered = forza_filter_map(
        image,
        info,
        args.occupancy_grid_threshold,
        args.filter_kernel_size,
    )
    skeleton = skeletonize(filtered, method="lee")
    skeleton_u8 = np.asarray(skeleton, dtype=np.uint8)
    if int(skeleton_u8.max(initial=0)) <= 1:
        skeleton_u8 *= 255

    center_pixels = smooth_centerline(
        extract_centerline(
            skeleton_u8,
            info.resolution,
            args.expected_centerline_length,
        )
    )
    center_meter = interp_closed_track(pixel_to_world(center_pixels, info, flip_y=True), step=0.1)
    if args.reverse:
        center_pixels = np.flip(center_pixels, axis=0).copy()
        center_meter = np.flip(center_meter, axis=0).copy()

    bounds = extract_track_bounds(center_pixels, center_meter, filtered, info)
    centerline = centerline_with_widths(center_pixels, center_meter, bounds, info)
    forza_traj, opt_right, opt_left, lap_time, max_curvature = optimize_min_curvature_iqp(
        centerline,
        args.optimizer_config_dir,
        args.safety_width,
        args.max_curvature,
        args.max_speed,
        args.longitudinal_accel_scale,
        args.lateral_accel_scale,
        args.machine_accel_scale,
        args.dynamic_model_exponent,
        args.velocity_filter_window,
    )

    if bounds.used_watershed and bounds.right is not None and bounds.left is not None:
        final_right, final_left = bounds.right, bounds.left
    else:
        final_right, final_left = opt_right, opt_left

    center_traj = build_center_trajectory(centerline)
    global_traj = build_global_trajectory(forza_traj, final_right, final_left)
    off_map = count_off_map_waypoints(global_traj.points_xy, filtered, info, flip_y=True)
    violations = int(np.count_nonzero(np.abs(global_traj.kappa_radpm) > max_curvature + 1e-6))
    return GenerationResult(
        map_info=info,
        image=image,
        free_mask=filtered,
        center_traj=center_traj,
        global_traj=global_traj,
        lap_time=lap_time,
        flip_y=True,
        off_map_wpnts=off_map,
        kappa_violations=violations,
        max_abs_kappa=float(np.max(np.abs(global_traj.kappa_radpm))),
        gen_args=args,
    )


# 기하를 바꾸는 인자들. output_dir/debug_image 는 어디에 무엇을 쓰느냐일 뿐이라 뺀다.
GEOMETRY_ARG_KEYS = (
    "map_yaml",
    "optimizer_config_dir",
    "occupancy_grid_threshold",
    "filter_kernel_size",
    "expected_centerline_length",
    "safety_width",
    "max_curvature",
    "max_speed",
    "longitudinal_accel_scale",
    "lateral_accel_scale",
    "machine_accel_scale",
    "dynamic_model_exponent",
    "velocity_filter_window",
    "reverse",
)


def stale_generation_keys(
    generated: argparse.Namespace | None,
    current: argparse.Namespace,
) -> list[str]:
    """Rebuild 이후 화면에서 바뀐 '기하에 영향 주는' 인자 이름들.

    비어 있지 않으면 self.result 의 기하와 현재 화면 값이 다르다는 뜻이다. 이걸
    무시하고 저장하면 metadata 는 새 값을, CSV 는 옛 기하를 담아 조용히 어긋난다.
    """
    if generated is None:
        return []
    stale = []
    for key in GEOMETRY_ARG_KEYS:
        if getattr(generated, key, None) != getattr(current, key, None):
            stale.append(key)
    return stale


def write_forza_outputs(
    output_dir: Path,
    result: GenerationResult,
    args: argparse.Namespace,
) -> None:
    """Write common f110-compatible files without inventing a shortest path."""
    write_outputs(
        output_dir,
        result.map_info,
        result.center_traj,
        result.global_traj,
        result.lap_time,
        args,
    )

    waypoints_path = output_dir / "global_waypoints.json"
    payload = json.loads(waypoints_path.read_text(encoding="utf-8"))
    payload["global_traj_wpnts_sp"] = {
        "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
        "wpnts": [],
    }
    payload["map_info_str"]["data"] += "; shortest_path=not_generated"
    waypoints_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    metadata_path = output_dir / "metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata["forza_source"] = {
        "race_stack_branch": "ros2-jazzy",
        "race_stack_commit": RACE_STACK_COMMIT,
        "optimizer_commit": OPTIMIZER_COMMIT,
        "generated_branches": ["centerline", "mincurv_iqp"],
    }
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")


class ForzaTrajectoryGui:
    def __init__(
        self,
        root: tk.Tk,
        params_path: Path,
        values: dict[str, Any],
    ) -> None:
        self.root = root
        self.params_path = params_path
        self.values = values
        self.result: GenerationResult | None = None
        self.generation_warning: GenerationWarning | None = None
        self.preview_rgb: np.ndarray | None = None
        self.photo: tk.PhotoImage | None = None
        self.busy = False

        root.title("ForzaETH Centerline + Minimum Curvature")
        root.geometry("1320x820")
        root.minsize(900, 600)
        root.columnconfigure(1, weight=1)
        root.rowconfigure(0, weight=1)

        panel_host = ttk.Frame(root, width=410)
        panel_host.grid(row=0, column=0, sticky="nsew")
        panel_host.grid_propagate(False)
        panel_host.rowconfigure(0, weight=1)
        panel_host.columnconfigure(0, weight=1)
        panel_canvas = tk.Canvas(panel_host, highlightthickness=0, width=390)
        panel_canvas.grid(row=0, column=0, sticky="nsew")
        panel_scrollbar = ttk.Scrollbar(
            panel_host,
            orient="vertical",
            command=panel_canvas.yview,
        )
        panel_scrollbar.grid(row=0, column=1, sticky="ns")
        panel_canvas.configure(yscrollcommand=panel_scrollbar.set)
        panel = ttk.Frame(panel_canvas, padding=12)
        panel_window = panel_canvas.create_window((0, 0), window=panel, anchor="nw")
        panel.bind(
            "<Configure>",
            lambda _event: panel_canvas.configure(scrollregion=panel_canvas.bbox("all")),
        )
        panel_canvas.bind(
            "<Configure>",
            lambda event: panel_canvas.itemconfigure(panel_window, width=event.width),
        )
        viewer = ttk.Frame(root)
        viewer.grid(row=0, column=1, sticky="nsew")
        viewer.rowconfigure(0, weight=1)
        viewer.columnconfigure(0, weight=1)

        self.canvas = tk.Canvas(viewer, bg="#171a20", highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.canvas.bind("<Configure>", lambda _event: self.render())

        ttk.Label(panel, text="ForzaETH Global Planner", font=("Sans", 16, "bold")).pack(
            anchor="w", pady=(0, 4)
        )
        ttk.Label(panel, text="Centerline + mincurv_iqp only").pack(anchor="w", pady=(0, 14))

        self.variables: dict[str, tk.Variable] = {
            "map_yaml": tk.StringVar(value=str(values["map_yaml"])),
            "output_dir": tk.StringVar(value=str(values["output_dir"])),
            "optimizer_config_dir": tk.StringVar(value=str(values["optimizer_config_dir"])),
            "occupancy_grid_threshold": tk.DoubleVar(value=float(values["occupancy_grid_threshold"])),
            "filter_kernel_size": tk.IntVar(value=int(values["filter_kernel_size"])),
            "expected_centerline_length": tk.DoubleVar(
                value=float(values["expected_centerline_length"])
            ),
            "safety_width": tk.DoubleVar(value=float(values["safety_width"])),
            "max_curvature": tk.DoubleVar(value=float(values["max_curvature"])),
            "max_speed": tk.DoubleVar(value=float(values["max_speed"])),
            "longitudinal_accel_scale": tk.DoubleVar(
                value=float(values["longitudinal_accel_scale"])
            ),
            "lateral_accel_scale": tk.DoubleVar(value=float(values["lateral_accel_scale"])),
            "machine_accel_scale": tk.DoubleVar(value=float(values["machine_accel_scale"])),
            "dynamic_model_exponent": tk.DoubleVar(
                value=float(values["dynamic_model_exponent"])
            ),
            "velocity_filter_window": tk.IntVar(value=int(values["velocity_filter_window"])),
            "reverse": tk.BooleanVar(value=bool(values["reverse"])),
            "show_centerline": tk.BooleanVar(value=bool(values["show_centerline"])),
            "show_raceline": tk.BooleanVar(value=bool(values["show_raceline"])),
            "debug_image": tk.BooleanVar(value=bool(values["debug_image"])),
        }
        self.numeric_entry_variables: dict[str, tk.StringVar] = {}
        self.numeric_scale_variables: dict[str, tk.DoubleVar] = {}
        self.numeric_entry_committers: dict[str, Any] = {}

        self._path_row(panel, "Map YAML", "map_yaml", self.choose_map)
        self._path_row(panel, "Output", "output_dir", self.choose_output)
        self._path_row(panel, "Optimizer config", "optimizer_config_dir", self.choose_config)
        self._scale_row(panel, "Occupancy threshold", "occupancy_grid_threshold", 1, 40)
        self._scale_row(panel, "Filter kernel", "filter_kernel_size", 1, 19)
        self._scale_row(panel, "Expected center length [m]", "expected_centerline_length", 0, 150)
        self._scale_row(panel, "Safety width [m]", "safety_width", 0.3, 1.2)
        self._scale_row(panel, "Max curvature [rad/m]", "max_curvature", 0.2, 3.0)
        ttk.Separator(panel).pack(fill="x", pady=(14, 4))
        ttk.Label(panel, text="Speed profile", font=("Sans", 11, "bold")).pack(anchor="w")
        self._scale_row(panel, "Max speed [m/s]", "max_speed", 0.5, 15.0)
        self._scale_row(panel, "Longitudinal accel scale", "longitudinal_accel_scale", 0.1, 2.0)
        self._scale_row(panel, "Lateral accel scale", "lateral_accel_scale", 0.1, 2.0)
        self._scale_row(panel, "Machine accel scale", "machine_accel_scale", 0.1, 2.0)
        self._scale_row(panel, "Dynamic model exponent", "dynamic_model_exponent", 1.0, 2.0)
        self._scale_row(panel, "Velocity filter window", "velocity_filter_window", 0, 21)

        for key, text in (
            ("reverse", "Reverse direction"),
            ("show_centerline", "Show centerline (blue)"),
            ("show_raceline", "Show min-curvature (red)"),
            ("debug_image", "Save debug overlay"),
        ):
            command = self.render if key.startswith("show_") else None
            ttk.Checkbutton(panel, text=text, variable=self.variables[key], command=command).pack(
                anchor="w", pady=3
            )

        buttons = ttk.Frame(panel)
        buttons.pack(fill="x", pady=(16, 8))
        self.rebuild_button = ttk.Button(buttons, text="Rebuild", command=self.rebuild)
        self.rebuild_button.pack(side="left", fill="x", expand=True, padx=(0, 4))
        ttk.Button(buttons, text="Save outputs", command=self.save_outputs).pack(
            side="left", fill="x", expand=True, padx=(4, 0)
        )
        self.status = tk.StringVar(value="Map을 선택한 뒤 Rebuild를 누르세요.")
        ttk.Label(panel, textvariable=self.status, wraplength=350, justify="left").pack(
            anchor="w", fill="x", pady=(12, 0)
        )

        if str(values["map_yaml"]).strip() and Path(str(values["map_yaml"])).is_file():
            root.after(100, self.rebuild)
        else:
            root.after(100, self.choose_map)

    def _path_row(self, parent: ttk.Frame, label: str, key: str, command: Any) -> None:
        ttk.Label(parent, text=label).pack(anchor="w", pady=(8, 2))
        row = ttk.Frame(parent)
        row.pack(fill="x")
        ttk.Entry(row, textvariable=self.variables[key]).pack(side="left", fill="x", expand=True)
        ttk.Button(row, text="…", width=3, command=command).pack(side="left", padx=(5, 0))

    def _scale_row(
        self, parent: ttk.Frame, label: str, key: str, low: float, high: float
    ) -> None:
        variable = self.variables[key]
        integer = isinstance(variable, tk.IntVar)
        entry_variable = tk.StringVar()
        scale_variable = tk.DoubleVar(
            value=min(max(float(variable.get()), low), high)
        )
        self.numeric_entry_variables[key] = entry_variable
        self.numeric_scale_variables[key] = scale_variable
        syncing_scale = False

        def format_value(value: Any) -> str:
            if integer:
                return str(int(round(float(value))))
            return f"{float(value):.10g}"

        def update_widgets(*_: Any, force_entry: bool = False) -> None:
            nonlocal syncing_scale
            value = float(variable.get())
            if force_entry or self.root.focus_get() is not entry:
                entry_variable.set(format_value(value))
            scale_value = min(max(value, low), high)
            if abs(float(scale_variable.get()) - scale_value) > 1e-12:
                syncing_scale = True
                try:
                    scale_variable.set(scale_value)
                finally:
                    syncing_scale = False

        def commit_entry() -> None:
            text = entry_variable.get().strip()
            try:
                value = float(text)
            except ValueError as exc:
                update_widgets(force_entry=True)
                raise ValueError(f"{label} 값이 숫자가 아닙니다: {text}") from exc
            if not math.isfinite(value):
                update_widgets(force_entry=True)
                raise ValueError(f"{label} 값은 유한한 숫자여야 합니다: {text}")
            normalized: float | int = int(round(value)) if integer else value
            variable.set(normalized)
            entry_variable.set(format_value(normalized))

        def apply_entry(_event: tk.Event | None = None) -> str:
            try:
                commit_entry()
            except ValueError as exc:
                self.status.set(str(exc))
            return "break"

        def apply_scale(value_text: str) -> None:
            if syncing_scale:
                return
            value = float(value_text)
            normalized: float | int = int(round(value)) if integer else value
            variable.set(normalized)
            update_widgets(force_entry=True)

        self.numeric_entry_committers[key] = commit_entry
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=(10, 0))
        ttk.Label(row, text=label).pack(side="left")
        entry = ttk.Entry(row, textvariable=entry_variable, width=10, justify="right")
        entry.pack(side="right")
        ttk.Scale(
            parent,
            variable=scale_variable,
            from_=low,
            to=high,
            orient="horizontal",
            command=apply_scale,
        ).pack(fill="x")
        variable.trace_add("write", update_widgets)
        entry.bind("<Return>", apply_entry)
        entry.bind("<KP_Enter>", apply_entry)
        entry.bind("<FocusOut>", apply_entry)
        update_widgets(force_entry=True)

    def choose_map(self) -> None:
        selected = filedialog.askopenfilename(
            title="ROS map YAML 선택", filetypes=[("YAML", "*.yaml *.yml"), ("All", "*")]
        )
        if selected:
            self.variables["map_yaml"].set(selected)
            if not str(self.variables["output_dir"].get()).strip():
                self.variables["output_dir"].set(str(default_output_dir(Path(selected)) / "forza"))
            self.rebuild()

    def choose_output(self) -> None:
        selected = filedialog.askdirectory(title="출력 디렉터리 선택")
        if selected:
            self.variables["output_dir"].set(selected)

    def choose_config(self) -> None:
        selected = filedialog.askdirectory(title="Forza optimizer config 디렉터리 선택")
        if selected:
            self.variables["optimizer_config_dir"].set(selected)

    def current_values(self) -> dict[str, Any]:
        for commit_entry in self.numeric_entry_committers.values():
            commit_entry()
        values = {key: variable.get() for key, variable in self.variables.items()}
        values["filter_kernel_size"] = int(round(float(values["filter_kernel_size"])))
        filter_window = int(round(float(values["velocity_filter_window"])))
        if filter_window > 0:
            filter_window = max(3, filter_window)
            if filter_window % 2 == 0:
                filter_window += 1
        values["velocity_filter_window"] = filter_window
        self.variables["velocity_filter_window"].set(filter_window)
        self.numeric_entry_variables["velocity_filter_window"].set(str(filter_window))
        return values

    def rebuild(self) -> None:
        if self.busy:
            return
        try:
            values = self.current_values()
            args = make_generation_args(values)
            save_gui_params(self.params_path, values)
        except Exception as exc:  # noqa: BLE001 - report invalid GUI input.
            messagebox.showerror("입력 오류", str(exc))
            return
        self.busy = True
        self.result = None
        self.generation_warning = None
        self.photo = None
        self.rebuild_button.configure(state="disabled")
        self.status.set("Forza centerline 및 mincurv_iqp 계산 중…")
        self.render()

        def worker() -> None:
            try:
                result = generate_forza_trajectory(args)
            except Exception as exc:  # noqa: BLE001 - show an actionable warning in the GUI.
                traceback.print_exc()
                warning = generation_warning_from_exception(exc, args)
                self.root.after(0, lambda warning=warning: self.finish_error(warning))
                return
            self.root.after(0, lambda: self.finish_result(result))

        threading.Thread(target=worker, daemon=True).start()

    def finish_error(self, warning: GenerationWarning) -> None:
        self.busy = False
        self.result = None
        self.generation_warning = warning
        self.photo = None
        self.rebuild_button.configure(state="normal")
        self.status.set(f"{warning.title} — 파라미터를 조절한 뒤 Rebuild를 다시 누르세요.")
        self.render()

    def finish_result(self, result: GenerationResult) -> None:
        self.busy = False
        self.rebuild_button.configure(state="normal")
        self.result = result
        self.generation_warning = None
        self.status.set(
            f"완료: center {len(result.center_traj.points_xy)} / mincurv "
            f"{len(result.global_traj.points_xy)} points, lap {result.lap_time:.3f}s, "
            f"max speed {float(np.max(result.global_traj.vx_mps)):.2f}m/s, "
            f"max |κ| {result.max_abs_kappa:.3f}"
        )
        self.render()

    def render(self) -> None:
        width = max(self.canvas.winfo_width() - 8, 1)
        height = max(self.canvas.winfo_height() - 8, 1)
        if self.generation_warning is not None:
            self.render_warning(self.generation_warning, width, height)
            return
        if self.busy:
            self.render_canvas_message("경로를 계산하고 있습니다…", width, height)
            return
        if self.result is None:
            return
        image = render_forza_preview_rgb(
            self.result,
            (width, height),
            bool(self.variables["show_centerline"].get()),
            bool(self.variables["show_raceline"].get()),
        )
        self.photo = rgb_to_photoimage(image)
        self.canvas.delete("all")
        self.canvas.create_image(width // 2, height // 2, image=self.photo, anchor="center")

    def render_canvas_message(self, message: str, width: int, height: int) -> None:
        self.canvas.delete("all")
        self.canvas.create_text(
            width // 2,
            height // 2,
            text=message,
            fill="#aeb6c2",
            font=("Sans", 15, "bold"),
            width=max(240, width - 80),
            justify="center",
        )

    def render_warning(
        self,
        warning: GenerationWarning,
        width: int,
        height: int,
    ) -> None:
        self.canvas.delete("all")
        margin = max(24, min(64, width // 12))
        card_left = margin
        card_right = max(card_left + 280, width - margin)
        card_top = max(24, min(64, height // 10))
        card_bottom = max(card_top + 420, height - card_top)
        text_width = max(240, card_right - card_left - 80)
        center_x = (card_left + card_right) // 2

        self.canvas.create_rectangle(
            card_left,
            card_top,
            card_right,
            card_bottom,
            fill="#241d20",
            outline="#d75b65",
            width=2,
        )
        self.canvas.create_oval(
            center_x - 22,
            card_top + 28,
            center_x + 22,
            card_top + 72,
            fill="#d75b65",
            outline="",
        )
        self.canvas.create_text(
            center_x,
            card_top + 50,
            text="!",
            fill="#ffffff",
            font=("Sans", 22, "bold"),
        )
        self.canvas.create_text(
            center_x,
            card_top + 96,
            text=warning.title,
            fill="#ff8b94",
            font=("Sans", 19, "bold"),
            width=text_width,
            justify="center",
            anchor="n",
        )
        self.canvas.create_text(
            center_x,
            card_top + 140,
            text=warning.message,
            fill="#f0e7e8",
            font=("Sans", 12),
            width=text_width,
            justify="center",
            anchor="n",
        )
        self.canvas.create_text(
            center_x,
            card_top + 196,
            text=warning.current_parameters,
            fill="#f2bd73",
            font=("Sans", 11, "bold"),
            width=text_width,
            justify="center",
            anchor="n",
        )
        suggestions = "\n".join(f"• {item}" for item in warning.suggestions)
        self.canvas.create_text(
            card_left + 40,
            card_top + 244,
            text=suggestions,
            fill="#d5d9df",
            font=("Sans", 11),
            width=text_width,
            justify="left",
            anchor="nw",
        )
        self.canvas.create_text(
            center_x,
            card_bottom - 64,
            text="왼쪽 파라미터를 조절한 뒤 Rebuild를 다시 누르세요.",
            fill="#8fd3a7",
            font=("Sans", 12, "bold"),
            width=text_width,
            justify="center",
            anchor="s",
        )
        self.canvas.create_text(
            center_x,
            card_bottom - 28,
            text=f"기술 정보: {warning.technical_detail}",
            fill="#8f969f",
            font=("Sans", 9),
            width=text_width,
            justify="center",
            anchor="s",
        )

    def save_outputs(self) -> None:
        if self.result is None:
            messagebox.showinfo("저장", "먼저 Rebuild를 실행하세요.")
            return
        try:
            values = self.current_values()
            args = make_generation_args(values)
            stale = stale_generation_keys(self.result.gen_args, args)
            if stale:
                messagebox.showwarning(
                    "Rebuild 필요",
                    "화면 값이 바뀌었는데 아직 Rebuild하지 않았습니다. 지금 저장하면 "
                    "metadata 는 새 값을, 경로 파일은 옛 기하를 담아 어긋납니다.\n\n"
                    "바뀐 항목: " + ", ".join(stale) + "\n\n"
                    "Rebuild를 먼저 누르세요.",
                )
                return
            output = args.output_dir or default_output_dir(args.map_yaml) / "forza"
            write_forza_outputs(output, self.result, args)
            if args.debug_image:
                write_debug_image(
                    output,
                    self.result.image,
                    self.result.map_info,
                    self.result.center_traj.points_xy,
                    self.result.global_traj.points_xy,
                    self.result.flip_y,
                )
            save_gui_params(self.params_path, values)
        except Exception as exc:  # noqa: BLE001 - report filesystem errors.
            messagebox.showerror("저장 실패", str(exc))
            return
        self.status.set(f"저장 완료: {output}")


def parse_cli() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="ForzaETH Jazzy centerline + mincurv_iqp offline GUI."
    )
    parser.add_argument("--map-yaml", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--params-yaml", type=Path, default=DEFAULT_PARAMS_YAML)
    parser.add_argument("--optimizer-config-dir", type=Path, default=None)
    parser.add_argument("--occupancy-grid-threshold", type=float, default=None)
    parser.add_argument("--filter-kernel-size", type=int, default=None)
    parser.add_argument("--expected-centerline-length", type=float, default=None)
    parser.add_argument("--safety-width", type=float, default=None)
    parser.add_argument("--max-curvature", type=float, default=None)
    parser.add_argument("--max-speed", type=float, default=None, help="최고속도 [m/s].")
    parser.add_argument(
        "--longitudinal-accel-scale",
        type=float,
        default=None,
        help="GGV 종방향 가속도 배율.",
    )
    parser.add_argument(
        "--lateral-accel-scale",
        type=float,
        default=None,
        help="GGV 횡방향 가속도 배율.",
    )
    parser.add_argument(
        "--machine-accel-scale",
        type=float,
        default=None,
        help="모터 가속도 배율.",
    )
    parser.add_argument(
        "--dynamic-model-exponent",
        type=float,
        default=None,
        help="TUM 동역학 모델 지수 [1.0, 2.0].",
    )
    parser.add_argument(
        "--velocity-filter-window",
        type=int,
        default=None,
        help="이동평균 필터 크기(0=끄기, 켤 때는 3 이상 홀수).",
    )
    parser.add_argument(
        "--reverse",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="centerline과 minimum-curvature 경로의 진행 방향을 뒤집습니다.",
    )
    parser.add_argument(
        "--render-test",
        type=Path,
        default=None,
        help="GUI 없이 결과 overlay PNG와 waypoint 파일을 만들고 종료합니다.",
    )
    return parser.parse_args()


def run_render_test(values: dict[str, Any], output_png: Path) -> int:
    args = make_generation_args(values)
    result = generate_forza_trajectory(args)
    output_png.parent.mkdir(parents=True, exist_ok=True)
    image = render_forza_preview_rgb(result, None, True, True)
    cv2.imwrite(str(output_png), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    output_dir = args.output_dir or output_png.parent / "waypoints"
    write_forza_outputs(output_dir, result, args)
    print(f"Rendered: {output_png}")
    print(f"Outputs: {output_dir}")
    print(
        f"Centerline: {len(result.center_traj.points_xy)}, mincurv_iqp: "
        f"{len(result.global_traj.points_xy)}, lap time: {result.lap_time:.3f}s"
    )
    return 0


def main() -> int:
    cli = parse_cli()
    values = load_gui_params(cli.params_yaml)
    if cli.map_yaml is not None:
        values["map_yaml"] = str(cli.map_yaml)
    if cli.output_dir is not None:
        values["output_dir"] = str(cli.output_dir)
    if cli.optimizer_config_dir is not None:
        values["optimizer_config_dir"] = str(cli.optimizer_config_dir)
    if cli.occupancy_grid_threshold is not None:
        values["occupancy_grid_threshold"] = cli.occupancy_grid_threshold
    if cli.filter_kernel_size is not None:
        values["filter_kernel_size"] = cli.filter_kernel_size
    if cli.expected_centerline_length is not None:
        values["expected_centerline_length"] = cli.expected_centerline_length
    if cli.safety_width is not None:
        values["safety_width"] = cli.safety_width
    if cli.max_curvature is not None:
        values["max_curvature"] = cli.max_curvature
    if cli.max_speed is not None:
        values["max_speed"] = cli.max_speed
    if cli.longitudinal_accel_scale is not None:
        values["longitudinal_accel_scale"] = cli.longitudinal_accel_scale
    if cli.lateral_accel_scale is not None:
        values["lateral_accel_scale"] = cli.lateral_accel_scale
    if cli.machine_accel_scale is not None:
        values["machine_accel_scale"] = cli.machine_accel_scale
    if cli.dynamic_model_exponent is not None:
        values["dynamic_model_exponent"] = cli.dynamic_model_exponent
    if cli.velocity_filter_window is not None:
        values["velocity_filter_window"] = cli.velocity_filter_window
    if cli.reverse is not None:
        values["reverse"] = cli.reverse
    if cli.render_test is not None:
        if not str(values["map_yaml"]).strip():
            raise SystemExit("--render-test에는 --map-yaml도 필요합니다.")
        return run_render_test(values, cli.render_test)

    root = tk.Tk()
    ForzaTrajectoryGui(root, cli.params_yaml, values)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
