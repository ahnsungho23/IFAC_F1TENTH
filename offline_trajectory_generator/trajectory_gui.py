#!/usr/bin/env python3
"""Tkinter GUI for offline trajectory generation and RT lane preview."""

from __future__ import annotations

import argparse
import base64
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import tkinter as tk
import yaml
from tkinter import filedialog, messagebox, ttk

from generate_global_trajectory import (
    GenerationResult,
    default_output_dir,
    generate_trajectory,
    write_debug_image,
    write_outputs,
    world_to_pixel,
)


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
DEFAULT_PARAMS_YAML = SCRIPT_DIR / "gui_params.yaml"
NO_GENERATE_KEYS = {"output_dir", "debug_image", "show_centerline", "show_rt_lane"}


@dataclass(frozen=True)
class NumericSpec:
    key: str
    label: str
    default: float
    min_value: float
    max_value: float
    resolution: float
    integer: bool = False


NUMERIC_SPECS = [
    NumericSpec("waypoint_step", "Waypoint step", 0.1, 0.03, 0.5, 0.01),
    NumericSpec("optimizer_step", "Optimizer step", 0.2, 0.05, 0.8, 0.01),
    NumericSpec("safety_width", "Safety width", 0.35, 0.08, 1.0, 0.01),
    NumericSpec("boundary_margin", "Boundary margin", 0.03, 0.0, 0.25, 0.005),
    NumericSpec("max_width_distance", "Max width dist", 5.0, 0.3, 8.0, 0.1),
    NumericSpec("max_speed", "Max speed", 4.0, 0.5, 9.0, 0.1),
    NumericSpec("min_speed", "Min speed", 1.0, 0.1, 4.0, 0.1),
    NumericSpec("max_lateral_accel", "Max lat accel", 4.0, 0.5, 10.0, 0.1),
    NumericSpec("max_accel", "Max accel", 3.0, 0.1, 6.0, 0.1),
    NumericSpec("max_decel", "Max decel", 5.0, 0.1, 10.0, 0.1),
    NumericSpec("smooth_sigma", "Smooth sigma", 2.0, 0.0, 6.0, 0.1),
    NumericSpec("median_kernel", "Median kernel", 3, 1, 11, 2, integer=True),
    NumericSpec("morph_kernel", "Morph kernel", 5, 1, 15, 2, integer=True),
    NumericSpec("morph_open_iterations", "Morph open", 1, 0, 4, 1, integer=True),
    NumericSpec("morph_close_iterations", "Morph close", 1, 0, 4, 1, integer=True),
    NumericSpec("skeleton_prune_iterations", "Skel prune", 80, 0, 250, 5, integer=True),
    NumericSpec("min_skeleton_component_area", "Skel min area", 40, 0, 1000, 10, integer=True),
    NumericSpec("min_centerline_angle", "Min path angle", 75.0, 0.0, 120.0, 1.0),
    NumericSpec("spike_filter_iterations", "Spike filter", 8, 0, 12, 1, integer=True),
    NumericSpec("max_optimizer_iter", "Optimizer iter", 120, 20, 300, 10, integer=True),
    NumericSpec("curvature_weight", "Curvature wt", 1.0, 0.0, 5.0, 0.05),
    NumericSpec("smooth_weight", "Smooth wt", 0.04, 0.0, 0.3, 0.005),
    NumericSpec("length_weight", "Length wt", 0.002, 0.0, 0.02, 0.0005),
]
SPEC_BY_KEY = {spec.key: spec for spec in NUMERIC_SPECS}


def normalize_numeric_value(spec: NumericSpec, value: Any) -> float | int:
    number = float(value)
    if spec.resolution > 0.0:
        steps = round((number - spec.min_value) / spec.resolution)
        number = spec.min_value + steps * spec.resolution
    number = min(max(number, spec.min_value), spec.max_value)
    if spec.integer:
        return int(round(number))
    return float(number)


def numeric_decimals(spec: NumericSpec) -> int:
    if spec.integer:
        return 0
    text = f"{spec.resolution:.10f}".rstrip("0").rstrip(".")
    if "." not in text:
        return 0
    return len(text.split(".", maxsplit=1)[1])


def format_numeric_value(spec: NumericSpec, value: Any) -> str:
    normalized = normalize_numeric_value(spec, value)
    if spec.integer:
        return f"{int(normalized)}"
    return f"{float(normalized):.{numeric_decimals(spec)}f}"


def normalize_gui_values(values: dict[str, Any]) -> dict[str, Any]:
    normalized = dict(values)
    for spec in NUMERIC_SPECS:
        try:
            normalized[spec.key] = normalize_numeric_value(spec, normalized.get(spec.key, spec.default))
        except (TypeError, ValueError):
            normalized[spec.key] = spec.default
    return normalized


def first_existing_map() -> Path:
    candidates = [
        REPO_ROOT / "monte_carlo_localization/maps/slam_map.yaml",
        REPO_ROOT / "monte_carlo_localization/maps/first_map.yaml",
        REPO_ROOT / "new_map_con/maps/oct_28.yaml",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return REPO_ROOT / "monte_carlo_localization/maps/slam_map.yaml"


def default_gui_values() -> dict[str, Any]:
    values: dict[str, Any] = {spec.key: spec.default for spec in NUMERIC_SPECS}
    values.update(
        {
            "map_yaml": "",
            "output_dir": "",
            "optimizer": "centerline",
            "width_mode": "distance",
            "show_centerline": True,
            "show_rt_lane": True,
            "debug_image": True,
            "reverse": False,
            "no_flip_y": False,
            "unknown_as_free": False,
        }
    )
    return values


def load_gui_params(path: Path) -> dict[str, Any]:
    values = default_gui_values()
    if not path.exists():
        return values
    with path.open("r", encoding="utf-8") as stream:
        loaded = yaml.safe_load(stream) or {}
    if isinstance(loaded, dict):
        for key in values:
            if key in loaded:
                values[key] = loaded[key]
    return normalize_gui_values(values)


def save_gui_params(path: Path, values: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    serializable: dict[str, Any] = {}
    for key, value in values.items():
        if isinstance(value, Path):
            serializable[key] = str(value)
        elif isinstance(value, np.generic):
            serializable[key] = value.item()
        else:
            serializable[key] = value
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(serializable, stream, sort_keys=False, allow_unicode=True)


def make_namespace(values: dict[str, Any]) -> argparse.Namespace:
    values = normalize_gui_values(values)
    map_yaml_text = str(values["map_yaml"]).strip()
    if not map_yaml_text:
        raise ValueError("map YAML is empty")
    map_yaml = Path(map_yaml_text)
    if not map_yaml.is_file():
        raise ValueError(f"map YAML does not exist: {map_yaml}")

    return argparse.Namespace(
        map_yaml=map_yaml,
        output_dir=Path(values["output_dir"]) if values.get("output_dir") else None,
        waypoint_step=float(values["waypoint_step"]),
        optimizer_step=float(values["optimizer_step"]),
        safety_width=float(values["safety_width"]),
        boundary_margin=float(values["boundary_margin"]),
        max_width_distance=float(values["max_width_distance"]),
        width_mode=str(values["width_mode"]),
        max_speed=float(values["max_speed"]),
        min_speed=float(values["min_speed"]),
        max_lateral_accel=float(values["max_lateral_accel"]),
        max_accel=float(values["max_accel"]),
        max_decel=float(values["max_decel"]),
        smooth_sigma=float(values["smooth_sigma"]),
        median_kernel=int(values["median_kernel"]),
        morph_kernel=int(values["morph_kernel"]),
        morph_open_iterations=int(values["morph_open_iterations"]),
        morph_close_iterations=int(values["morph_close_iterations"]),
        skeleton_prune_iterations=int(values["skeleton_prune_iterations"]),
        min_skeleton_component_area=int(values["min_skeleton_component_area"]),
        min_centerline_angle=float(values["min_centerline_angle"]),
        spike_filter_iterations=int(values["spike_filter_iterations"]),
        optimizer=str(values["optimizer"]),
        max_optimizer_iter=int(values["max_optimizer_iter"]),
        curvature_weight=float(values["curvature_weight"]),
        smooth_weight=float(values["smooth_weight"]),
        length_weight=float(values["length_weight"]),
        reverse=bool(values["reverse"]),
        no_flip_y=bool(values["no_flip_y"]),
        unknown_as_free=bool(values["unknown_as_free"]),
        debug_image=bool(values.get("debug_image", False)),
    )


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
    ok, encoded = cv2.imencode(".png", cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR))
    if not ok:
        raise RuntimeError("Failed to encode preview image for Tk")
    data = base64.b64encode(encoded.tobytes()).decode("ascii")
    return tk.PhotoImage(data=data, format="PNG")


class TrajectoryGui:
    def __init__(
        self,
        root: tk.Tk,
        initial_map: Path | None,
        initial_output: Path | None,
        params_path: Path,
        initial_values: dict[str, Any],
    ):
        self.root = root
        self.root.title("Offline Trajectory Generator")
        self.root.geometry("1280x820")
        self.root.minsize(940, 620)

        self.variables: dict[str, tk.Variable] = {}
        self.pending_after: str | None = None
        self.generation_id = 0
        self.running = False
        self.current_result: GenerationResult | None = None
        self.current_args: argparse.Namespace | None = None
        self.photo: tk.PhotoImage | None = None
        self.last_render_size = (0, 0)
        self.params_path = params_path

        self._build_layout(initial_map, initial_output, initial_values)
        self.save_current_params()
        if initial_map is None:
            self.status_var.set("Choose a map YAML to start.")
            self.root.after(100, self.prompt_initial_map)
        else:
            self.schedule_generate(delay_ms=100)

    def _build_layout(
        self,
        initial_map: Path | None,
        initial_output: Path | None,
        initial_values: dict[str, Any],
    ) -> None:
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=0)

        left = ttk.Frame(self.root, width=390)
        left.grid(row=0, column=0, sticky="nsw")
        left.grid_propagate(False)
        left.rowconfigure(0, weight=1)

        panel_canvas = tk.Canvas(left, highlightthickness=0, width=390)
        panel_scroll = ttk.Scrollbar(left, orient="vertical", command=panel_canvas.yview)
        self.panel = ttk.Frame(panel_canvas)
        self.panel.bind(
            "<Configure>",
            lambda event: panel_canvas.configure(scrollregion=panel_canvas.bbox("all")),
        )
        panel_canvas.create_window((0, 0), window=self.panel, anchor="nw")
        panel_canvas.configure(yscrollcommand=panel_scroll.set)
        panel_canvas.grid(row=0, column=0, sticky="nsew")
        panel_scroll.grid(row=0, column=1, sticky="ns")

        viewer_frame = ttk.Frame(self.root)
        viewer_frame.grid(row=0, column=1, sticky="nsew")
        viewer_frame.rowconfigure(0, weight=1)
        viewer_frame.columnconfigure(0, weight=1)

        self.canvas = tk.Canvas(viewer_frame, background="#1b1f23", highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.canvas.bind("<Configure>", self._on_canvas_resize)

        status = ttk.Frame(self.root)
        status.grid(row=1, column=0, columnspan=2, sticky="ew")
        status.columnconfigure(0, weight=1)
        self.status_var = tk.StringVar(value="Ready")
        ttk.Label(status, textvariable=self.status_var, anchor="w").grid(
            row=0, column=0, sticky="ew", padx=8, pady=4
        )

        self._build_controls(initial_map, initial_output, initial_values)

    def _build_controls(
        self,
        initial_map: Path | None,
        initial_output: Path | None,
        initial_values: dict[str, Any],
    ) -> None:
        pad = {"padx": 8, "pady": 4}
        row = 0

        ttk.Label(self.panel, text="Map", font=("", 11, "bold")).grid(row=row, column=0, sticky="w", **pad)
        row += 1
        map_value = str(initial_map) if initial_map is not None else str(initial_values.get("map_yaml", ""))
        self.variables["map_yaml"] = tk.StringVar(value=map_value)
        self._entry_with_button(row, "YAML", self.variables["map_yaml"], self.browse_map)
        row += 1

        saved_output = str(initial_values.get("output_dir", ""))
        if initial_output is not None:
            default_output = str(initial_output)
        elif initial_map is not None:
            default_output = saved_output or str(default_output_dir(initial_map))
        else:
            default_output = saved_output
        self.variables["output_dir"] = tk.StringVar(value=default_output)
        self._entry_with_button(row, "Output", self.variables["output_dir"], self.browse_output)
        row += 1

        buttons = ttk.Frame(self.panel)
        buttons.grid(row=row, column=0, columnspan=3, sticky="ew", **pad)
        buttons.columnconfigure(0, weight=1)
        buttons.columnconfigure(1, weight=1)
        ttk.Button(buttons, text="Rebuild", command=lambda: self.schedule_generate(0)).grid(
            row=0, column=0, sticky="ew", padx=(0, 4)
        )
        ttk.Button(buttons, text="Save", command=self.save_outputs).grid(
            row=0, column=1, sticky="ew", padx=(4, 0)
        )
        row += 1

        ttk.Separator(self.panel).grid(row=row, column=0, columnspan=3, sticky="ew", padx=8, pady=8)
        row += 1

        ttk.Label(self.panel, text="Trajectory", font=("", 11, "bold")).grid(row=row, column=0, sticky="w", **pad)
        row += 1

        self.variables["optimizer"] = tk.StringVar(value=str(initial_values.get("optimizer", "centerline")))
        self._option(row, "Optimizer", self.variables["optimizer"], ("centerline", "mincurv"))
        row += 1

        self.variables["width_mode"] = tk.StringVar(value=str(initial_values.get("width_mode", "distance")))
        self._option(row, "Width mode", self.variables["width_mode"], ("distance", "raycast"))
        row += 1

        self.variables["show_centerline"] = tk.BooleanVar(value=bool(initial_values.get("show_centerline", True)))
        self.variables["show_rt_lane"] = tk.BooleanVar(value=bool(initial_values.get("show_rt_lane", True)))
        self.variables["debug_image"] = tk.BooleanVar(value=bool(initial_values.get("debug_image", True)))
        row = self._check(row, "Show centerline", self.variables["show_centerline"], render_only=True)
        row = self._check(row, "Show RT lane", self.variables["show_rt_lane"], render_only=True)
        row = self._check(row, "Save debug image", self.variables["debug_image"], render_only=False)

        self.variables["reverse"] = tk.BooleanVar(value=bool(initial_values.get("reverse", False)))
        self.variables["no_flip_y"] = tk.BooleanVar(value=bool(initial_values.get("no_flip_y", False)))
        self.variables["unknown_as_free"] = tk.BooleanVar(value=bool(initial_values.get("unknown_as_free", False)))
        row = self._check(row, "Reverse", self.variables["reverse"])
        row = self._check(row, "No flip Y", self.variables["no_flip_y"])
        row = self._check(row, "Unknown as free", self.variables["unknown_as_free"])

        ttk.Separator(self.panel).grid(row=row, column=0, columnspan=3, sticky="ew", padx=8, pady=8)
        row += 1

        ttk.Label(self.panel, text="Parameters", font=("", 11, "bold")).grid(row=row, column=0, sticky="w", **pad)
        row += 1

        for spec in NUMERIC_SPECS:
            value = normalize_numeric_value(spec, initial_values.get(spec.key, spec.default))
            self.variables[spec.key] = tk.DoubleVar(value=float(value))
            self._scale(row, spec)
            row += 1

        self.panel.columnconfigure(1, weight=1)
        self.panel.columnconfigure(2, weight=0)
        for key, variable in self.variables.items():
            variable.trace_add("write", lambda *_args, changed_key=key: self.on_variable_changed(changed_key))

    def _entry_with_button(self, row: int, label: str, variable: tk.Variable, command: Any) -> None:
        ttk.Label(self.panel, text=label).grid(row=row, column=0, sticky="w", padx=8, pady=4)
        ttk.Entry(self.panel, textvariable=variable, width=26).grid(row=row, column=1, sticky="ew", padx=4, pady=4)
        ttk.Button(self.panel, text="...", width=3, command=command).grid(row=row, column=2, sticky="e", padx=8, pady=4)

    def _option(self, row: int, label: str, variable: tk.Variable, values: tuple[str, ...]) -> None:
        ttk.Label(self.panel, text=label).grid(row=row, column=0, sticky="w", padx=8, pady=4)
        ttk.OptionMenu(self.panel, variable, variable.get(), *values).grid(
            row=row, column=1, columnspan=2, sticky="ew", padx=4, pady=4
        )

    def _check(
        self,
        row: int,
        label: str,
        variable: tk.BooleanVar,
        render_only: bool = False,
    ) -> int:
        del render_only
        ttk.Checkbutton(self.panel, text=label, variable=variable).grid(
            row=row, column=0, columnspan=3, sticky="w", padx=8, pady=2
        )
        return row + 1

    def _scale(self, row: int, spec: NumericSpec) -> None:
        variable = self.variables[spec.key]
        label_var = tk.StringVar()
        entry_var = tk.StringVar()

        def update_label(*_: Any) -> None:
            formatted = format_numeric_value(spec, variable.get())
            label_var.set(f"{spec.label}: {formatted}")
            if self.root.focus_get() is not entry:
                entry_var.set(formatted)

        def apply_entry(_event: tk.Event | None = None) -> str:
            text = entry_var.get().strip()
            try:
                value = normalize_numeric_value(spec, text)
            except (TypeError, ValueError):
                update_label()
                self.status_var.set(f"Invalid {spec.label}: {text}")
                return "break"
            variable.set(float(value))
            entry_var.set(format_numeric_value(spec, value))
            return "break"

        variable.trace_add("write", update_label)
        ttk.Label(self.panel, textvariable=label_var).grid(row=row, column=0, sticky="w", padx=8, pady=3)
        scale = ttk.Scale(
            self.panel,
            from_=spec.min_value,
            to=spec.max_value,
            variable=variable,
        )
        scale.grid(row=row, column=1, sticky="ew", padx=8, pady=3)
        entry = ttk.Entry(self.panel, textvariable=entry_var, width=8, justify="right")
        entry.grid(row=row, column=2, sticky="e", padx=(0, 8), pady=3)
        entry.bind("<Return>", apply_entry)
        entry.bind("<FocusOut>", apply_entry)
        update_label()

    def browse_map(self) -> None:
        filename = filedialog.askopenfilename(
            title="Select ROS map YAML",
            initialdir=str(first_existing_map().parent),
            filetypes=(("Map YAML", "*.yaml *.yml"), ("All files", "*")),
        )
        if filename:
            self.variables["map_yaml"].set(filename)
            if not self.variables["output_dir"].get():
                self.variables["output_dir"].set(str(default_output_dir(Path(filename))))

    def prompt_initial_map(self) -> None:
        self.browse_map()
        if not self.variables["map_yaml"].get().strip():
            self.canvas.delete("all")
            self.canvas.create_text(
                self.canvas.winfo_width() // 2,
                self.canvas.winfo_height() // 2,
                text="Select a map YAML with the left Browse button.",
                fill="#f7f7f7",
                width=max(300, self.canvas.winfo_width() - 80),
            )

    def browse_output(self) -> None:
        dirname = filedialog.askdirectory(initialdir=str(REPO_ROOT))
        if dirname:
            self.variables["output_dir"].set(dirname)

    def collect_values(self) -> dict[str, Any]:
        values: dict[str, Any] = {}
        for key, variable in self.variables.items():
            if isinstance(variable, tk.BooleanVar):
                values[key] = bool(variable.get())
            else:
                values[key] = variable.get()
        for spec in NUMERIC_SPECS:
            values[spec.key] = normalize_numeric_value(spec, values[spec.key])
        return values

    def save_current_params(self) -> None:
        try:
            save_gui_params(self.params_path, self.collect_values())
        except OSError as exc:
            self.status_var.set(f"Parameter save failed: {exc}")

    def on_variable_changed(self, key: str) -> None:
        self.save_current_params()
        if key in {"show_centerline", "show_rt_lane"}:
            self.render_current()
            return
        if key not in NO_GENERATE_KEYS:
            self.schedule_generate()

    def schedule_generate(self, delay_ms: int = 450) -> None:
        if self.pending_after is not None:
            self.root.after_cancel(self.pending_after)
        self.pending_after = self.root.after(delay_ms, self.start_generate)

    def start_generate(self) -> None:
        self.pending_after = None
        values = self.collect_values()
        try:
            args = make_namespace(values)
        except (TypeError, ValueError) as exc:
            self.status_var.set(f"Invalid parameter: {exc}")
            return

        self.generation_id += 1
        generation_id = self.generation_id
        self.running = True
        self.status_var.set("Generating...")

        def worker() -> None:
            started = time.perf_counter()
            try:
                result = generate_trajectory(args)
            except Exception as exc:  # noqa: BLE001 - GUI reports calculation failures to the user.
                self.root.after(0, lambda: self.finish_error(generation_id, exc))
                return
            elapsed = time.perf_counter() - started
            self.root.after(0, lambda: self.finish_generate(generation_id, args, result, elapsed))

        threading.Thread(target=worker, daemon=True).start()

    def finish_error(self, generation_id: int, exc: Exception) -> None:
        if generation_id != self.generation_id:
            return
        self.running = False
        self.status_var.set(f"Error: {exc}")
        self.canvas.delete("all")
        self.canvas.create_text(
            self.canvas.winfo_width() // 2,
            self.canvas.winfo_height() // 2,
            text=str(exc),
            fill="#f7f7f7",
            width=max(300, self.canvas.winfo_width() - 80),
        )

    def finish_generate(
        self,
        generation_id: int,
        args: argparse.Namespace,
        result: GenerationResult,
        elapsed: float,
    ) -> None:
        if generation_id != self.generation_id:
            return
        self.running = False
        self.current_args = args
        self.current_result = result
        self.status_var.set(
            f"Waypoints {len(result.global_traj.points_xy)} | "
            f"lap {result.lap_time:.3f}s | generated {elapsed:.2f}s"
        )
        self.render_current()

    def _on_canvas_resize(self, _event: tk.Event) -> None:
        if self.current_result is None:
            return
        size = (self.canvas.winfo_width(), self.canvas.winfo_height())
        if abs(size[0] - self.last_render_size[0]) > 16 or abs(size[1] - self.last_render_size[1]) > 16:
            self.render_current()

    def render_current(self) -> None:
        if self.current_result is None:
            return
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        image_rgb = render_preview_rgb(
            self.current_result,
            (width - 16, height - 16),
            show_centerline=bool(self.variables["show_centerline"].get()),
            show_rt_lane=bool(self.variables["show_rt_lane"].get()),
        )
        self.photo = rgb_to_photoimage(image_rgb)
        self.canvas.delete("all")
        x = max(0, (width - image_rgb.shape[1]) // 2)
        y = max(0, (height - image_rgb.shape[0]) // 2)
        self.canvas.create_image(x, y, image=self.photo, anchor="nw")
        self.last_render_size = (width, height)

    def save_outputs(self) -> None:
        if self.current_result is None or self.current_args is None:
            messagebox.showwarning("Save", "No trajectory has been generated yet.")
            return
        output_dir_text = self.variables["output_dir"].get().strip()
        output_dir = Path(output_dir_text) if output_dir_text else default_output_dir(self.current_args.map_yaml)
        args = self.current_args
        args.output_dir = output_dir
        args.debug_image = bool(self.variables["debug_image"].get())
        try:
            write_outputs(
                output_dir,
                self.current_result.map_info,
                self.current_result.center_traj,
                self.current_result.global_traj,
                self.current_result.lap_time,
                args,
            )
            if args.debug_image:
                write_debug_image(
                    output_dir,
                    self.current_result.image,
                    self.current_result.map_info,
                    self.current_result.center_traj.points_xy,
                    self.current_result.global_traj.points_xy,
                    self.current_result.flip_y,
                )
        except Exception as exc:  # noqa: BLE001 - GUI reports filesystem failures.
            messagebox.showerror("Save failed", str(exc))
            return
        self.status_var.set(f"Saved to {output_dir}")


def parse_cli() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="GUI for offline trajectory generation.")
    parser.add_argument(
        "--map-yaml",
        type=Path,
        default=None,
        help="Initial map YAML. If omitted, the GUI opens a file picker on startup.",
    )
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument(
        "--params-yaml",
        type=Path,
        default=DEFAULT_PARAMS_YAML,
        help="GUI parameter YAML that is loaded and updated automatically.",
    )
    parser.add_argument(
        "--render-test",
        type=Path,
        default=None,
        help="Headless test mode: render preview PNG and exit.",
    )
    return parser.parse_args()


def resolve_initial_map(cli_map: Path | None, saved_values: dict[str, Any]) -> Path | None:
    if cli_map is not None:
        return cli_map
    saved_map = str(saved_values.get("map_yaml", "")).strip()
    if saved_map:
        saved_path = Path(saved_map)
        if saved_path.is_file():
            return saved_path
    return None


def run_render_test(map_yaml: Path, output_png: Path) -> int:
    values = {spec.key: spec.default for spec in NUMERIC_SPECS}
    values.update(
        {
            "map_yaml": str(map_yaml),
            "output_dir": "",
            "optimizer": "centerline",
            "width_mode": "distance",
            "reverse": False,
            "no_flip_y": False,
            "unknown_as_free": False,
            "debug_image": False,
        }
    )
    result = generate_trajectory(make_namespace(values))
    image_rgb = render_preview_rgb(result, None, True, True)
    output_png.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_png), cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR))
    print(f"Rendered {output_png}")
    print(f"Waypoints: {len(result.global_traj.points_xy)}, estimated lap time: {result.lap_time:.3f}s")
    return 0


def main() -> int:
    args = parse_cli()
    if args.render_test is not None:
        return run_render_test(args.map_yaml or first_existing_map(), args.render_test)

    saved_values = load_gui_params(args.params_yaml)
    initial_map = resolve_initial_map(args.map_yaml, saved_values)
    root = tk.Tk()
    TrajectoryGui(root, initial_map, args.output_dir, args.params_yaml, saved_values)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
