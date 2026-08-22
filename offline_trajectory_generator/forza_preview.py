#!/usr/bin/env python3
"""Map preview rendering for the ForzaETH trajectory GUI.

Forza-only module. These three functions are byte-identical copies of the ones
in trajectory_gui.py (verified against both edge_test@95843f7a and
adaptive_global@6ba865b3); they live here so forza_trajectory_gui.py shares no
file with the C++ generator's GUI. The two previews are free to diverge.

draw_polyline is not optional: render_preview_rgb calls it.
"""

from __future__ import annotations

import cv2
import numpy as np
import tkinter as tk

from forza_common import GenerationResult, world_to_pixel


def draw_polyline(
    image_rgb: np.ndarray,
    points_xy: np.ndarray,
    result: GenerationResult,
    color: tuple[int, int, int],
    thickness: int,
) -> None:
    pixels = np.round(world_to_pixel(points_xy, result.map_info, result.flip_y)).astype(np.int32)
    pixels[:, 0] = np.clip(pixels[:, 0], 0, result.map_info.width - 1)
    pixels[:, 1] = np.clip(pixels[:, 1], 0, result.map_info.height - 1)
    for p0, p1 in zip(pixels, np.roll(pixels, -1, axis=0)):
        cv2.line(image_rgb, tuple(p0), tuple(p1), color, thickness, cv2.LINE_AA)


def render_preview_rgb(
    result: GenerationResult,
    target_size: tuple[int, int] | None = None,
    show_centerline: bool = True,
    show_rt_lane: bool = True,
) -> np.ndarray:
    image_rgb = cv2.cvtColor(result.image, cv2.COLOR_GRAY2RGB)
    thickness = max(1, int(round(max(result.map_info.width, result.map_info.height) / 700)))

    if show_centerline:
        draw_polyline(image_rgb, result.center_traj.points_xy, result, (0, 120, 255), thickness)
    if show_rt_lane:
        draw_polyline(image_rgb, result.global_traj.points_xy, result, (255, 70, 45), thickness + 1)

    start_px = np.round(
        world_to_pixel(result.global_traj.points_xy[:1], result.map_info, result.flip_y)[0]
    ).astype(np.int32)
    start_px[0] = np.clip(start_px[0], 0, result.map_info.width - 1)
    start_px[1] = np.clip(start_px[1], 0, result.map_info.height - 1)
    cv2.circle(image_rgb, tuple(start_px), max(3, thickness + 2), (0, 220, 90), -1, cv2.LINE_AA)

    if target_size is None:
        return image_rgb

    target_w, target_h = target_size
    if target_w <= 1 or target_h <= 1:
        return image_rgb
    scale = min(target_w / image_rgb.shape[1], target_h / image_rgb.shape[0])
    scale = max(scale, 0.01)
    new_size = (
        max(1, int(round(image_rgb.shape[1] * scale))),
        max(1, int(round(image_rgb.shape[0] * scale))),
    )
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_NEAREST
    return cv2.resize(image_rgb, new_size, interpolation=interpolation)


def rgb_to_photoimage(image_rgb: np.ndarray) -> tk.PhotoImage:
    # Binary PPM (P6) is understood by every Tk version (8.5 has no PNG) and
    # skips the PNG+base64 encode, which keeps resize re-renders cheap.
    height, width = image_rgb.shape[:2]
    header = f"P6 {width} {height} 255 ".encode("ascii")
    data = header + np.ascontiguousarray(image_rgb).tobytes()
    return tk.PhotoImage(data=data, format="PPM")
