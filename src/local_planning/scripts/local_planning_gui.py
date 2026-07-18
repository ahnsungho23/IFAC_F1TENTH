#!/usr/bin/env python3
"""Tkinter GUI for real-time fine-tuning of local planning parameters and instant RViz reflection."""

from __future__ import annotations

import argparse
import math
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List

import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import yaml

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from f110_msgs.msg import Wpnt, WpntArray, Obstacle, ObstacleArray


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent
DEFAULT_YAML_PATH = PACKAGE_DIR / "config" / "local_planning.yaml"


@dataclass(frozen=True)
class NumericSpec:
    key: str
    label: str
    default: float
    min_value: float
    max_value: float
    resolution: float
    integer: bool = False
    group: str = "General"
    node_target: str = "both"  # 'static_obstacle_avoidance_node', 'local_planner_node', or 'both'


NUMERIC_SPECS = [
    # Track & Safety Margins
    NumericSpec("safety_margin", "Safety margin (m)", 0.28, 0.05, 1.20, 0.01, group="Track & Safety", node_target="both"),
    NumericSpec("corridor_width", "Corridor width (m)", 0.65, 0.20, 2.00, 0.02, group="Track & Safety", node_target="static_obstacle_avoidance_node"),
    NumericSpec("wall_margin", "Wall margin (m)", 0.35, 0.05, 1.20, 0.01, group="Track & Safety", node_target="local_planner_node"),
    NumericSpec("passing_buffer_s", "Passing buffer (m)", 1.20, 0.20, 4.00, 0.10, group="Track & Safety", node_target="static_obstacle_avoidance_node"),
    NumericSpec("lookahead_distance", "Lookahead dist (m)", 15.0, 3.0, 35.0, 0.5, group="Track & Safety", node_target="static_obstacle_avoidance_node"),

    # Lattice & Avoidance Geometry
    NumericSpec("avoid_offset", "Avoid offset (m)", 1.00, 0.20, 3.00, 0.05, group="Lattice & Geometry", node_target="local_planner_node"),
    NumericSpec("lattice_num_candidates", "Lattice candidates", 9, 3, 21, 2, integer=True, group="Lattice & Geometry", node_target="local_planner_node"),
    NumericSpec("lookahead_wpnt_num", "Lookahead wpnts", 40, 10, 120, 5, integer=True, group="Lattice & Geometry", node_target="local_planner_node"),
    NumericSpec("waypoint_step", "Waypoint step (m)", 0.10, 0.02, 0.50, 0.01, group="Lattice & Geometry", node_target="static_obstacle_avoidance_node"),
    NumericSpec("poly_degree", "Polynomial degree", 3, 1, 5, 1, integer=True, group="Lattice & Geometry", node_target="local_planner_node"),

    # Cost Weights
    NumericSpec("weight_obs", "Weight Obstacle", 10.0, 0.0, 50.0, 0.5, group="Cost Weights", node_target="both"),
    NumericSpec("weight_lat", "Weight Lateral", 1.0, 0.0, 15.0, 0.1, group="Cost Weights", node_target="both"),
    NumericSpec("weight_smooth", "Weight Smoothness", 2.5, 0.0, 20.0, 0.2, group="Cost Weights", node_target="both"),
    NumericSpec("weight_speed", "Weight Speed Loss", 1.5, 0.0, 15.0, 0.1, group="Cost Weights", node_target="both"),

    # Speed Profile & Dynamics
    NumericSpec("max_lat_accel", "Max lat accel (m/s²)", 6.0, 1.0, 14.0, 0.2, group="Speed Profile", node_target="both"),
    NumericSpec("min_speed", "Min speed (m/s)", 1.5, 0.2, 6.0, 0.1, group="Speed Profile", node_target="both"),
    NumericSpec("speed_reduction_ratio", "Speed reduction", 0.80, 0.20, 1.00, 0.05, group="Speed Profile", node_target="local_planner_node"),
    NumericSpec("static_vel_threshold", "Static vel thresh", 0.35, 0.05, 1.50, 0.05, group="Speed Profile", node_target="static_obstacle_avoidance_node"),

    # Virtual Obstacle Simulator (for RViz testing when stationary)
    NumericSpec("virt_obs_s", "Virt Obs Dist s (m)", 10.0, 2.0, 30.0, 0.5, group="Virtual Obstacle (RViz Test)", node_target="virtual_sim"),
    NumericSpec("virt_obs_d", "Virt Obs Offset d (m)", 0.0, -2.0, 2.0, 0.05, group="Virtual Obstacle (RViz Test)", node_target="virtual_sim"),
    NumericSpec("virt_obs_r", "Virt Obs Radius (m)", 0.35, 0.1, 1.5, 0.05, group="Virtual Obstacle (RViz Test)", node_target="virtual_sim"),
]

SPEC_BY_KEY = {spec.key: spec for spec in NUMERIC_SPECS}


def normalize_numeric_value(spec: NumericSpec, value: Any, *, clamp_to_slider: bool = False) -> float | int:
    number = float(value)
    if spec.resolution > 0.0:
        steps = round((number - spec.min_value) / spec.resolution)
        number = spec.min_value + steps * spec.resolution
    number = max(number, spec.min_value)
    if clamp_to_slider:
        number = min(number, spec.max_value)
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


class LocalPlanningGuiNode(Node):
    """Background ROS 2 node to handle async parameter synchronization and virtual obstacle simulation."""

    def __init__(self, gui_app: 'LocalPlanningGui') -> None:
        super().__init__("local_planning_gui_node")
        self.gui = gui_app

        # Service clients for dynamic parameter updates
        self.param_clients = {
            "static_obstacle_avoidance_node": self.create_client(SetParameters, "/static_obstacle_avoidance_node/set_parameters"),
            "local_planner_node": self.create_client(SetParameters, "/local_planner_node/set_parameters")
        }

        # Subscribers for virtual obstacle simulation
        self.global_wpnts: List[Wpnt] = []
        self.car_s = 0.0
        self.track_length = 0.0

        self.sub_global = self.create_subscription(
            WpntArray, "/global_waypoints", self.on_global_waypoints, 1)
        self.sub_odom = self.create_subscription(
            Odometry, "/car_state/frenet/odom", self.on_frenet_odom, 10)

        # Publisher for virtual obstacle
        self.pub_obs = self.create_publisher(ObstacleArray, "/obstacles", 10)

        # Virtual obstacle simulation timer (10 Hz)
        self.sim_timer = self.create_timer(0.1, self.on_sim_timer)
        self.get_logger().info("LocalPlanningGuiNode initialized.")

    def on_global_waypoints(self, msg: WpntArray) -> None:
        if not msg.wpnts:
            return
        self.global_wpnts = list(msg.wpnts)
        self.track_length = self.global_wpnts[-1].s_m

    def on_frenet_odom(self, msg: Odometry) -> None:
        self.car_s = msg.pose.pose.position.x

    def sync_parameter(self, target_node: str, param_name: str, value: Any, spec: NumericSpec) -> None:
        if target_node == "virtual_sim":
            return
        targets = []
        if target_node == "both":
            targets = ["static_obstacle_avoidance_node", "local_planner_node"]
        else:
            targets = [target_node]

        for t in targets:
            client = self.param_clients.get(t)
            if client and client.service_is_ready():
                param_type = Parameter.Type.INTEGER if spec.integer else Parameter.Type.DOUBLE
                p = Parameter(param_name, param_type, value)
                req = SetParameters.Request()
                req.parameters = [p.to_parameter_msg()]
                client.call_async(req)

    def on_sim_timer(self) -> None:
        if not self.gui.variables.get("enable_virtual_obs", tk.BooleanVar(value=False)).get():
            return
        if not self.global_wpnts:
            return

        # Get virtual obstacle parameters from GUI variables
        s_dist = float(self.gui.variables["virt_obs_s"].get())
        d_offset = float(self.gui.variables["virt_obs_d"].get())
        radius = float(self.gui.variables["virt_obs_r"].get())

        target_s = self.car_s + s_dist
        if self.track_length > 0:
            target_s = math.fmod(target_s, self.track_length)
            if target_s < 0:
                target_s += self.track_length

        # Interpolate cartesian coordinates and heading at target_s
        idx1 = 0
        min_diff = float('inf')
        for i, w in enumerate(self.global_wpnts):
            diff = abs(w.s_m - target_s)
            if diff < min_diff:
                min_diff = diff
                idx1 = i

        w1 = self.global_wpnts[idx1]
        idx2 = (idx1 + 1) % len(self.global_wpnts)
        w2 = self.global_wpnts[idx2]

        ds = w2.s_m - w1.s_m
        if ds <= 0:
            ds = 0.1
        ratio = max(0.0, min(1.0, (target_s - w1.s_m) / ds))

        base_x = w1.x_m + ratio * (w2.x_m - w1.x_m)
        base_y = w1.y_m + ratio * (w2.y_m - w1.y_m)
        psi = math.atan2(w2.y_m - w1.y_m, w2.x_m - w1.x_m)

        # Apply lateral offset d_offset perpendicular to path heading
        obs_x = base_x - d_offset * math.sin(psi)
        obs_y = base_y + d_offset * math.cos(psi)

        obs = Obstacle()
        obs.id = 999
        obs.s_center = target_s
        obs.d_center = d_offset
        obs.s_start = target_s - radius
        obs.s_end = target_s + radius
        obs.d_right = d_offset - radius
        obs.d_left = d_offset + radius
        obs.size = radius * 2.0
        obs.vs = 0.0
        obs.vd = 0.0
        obs.is_static = True

        # Provide a rough bounding box in cartesian space
        for dx, dy in [(-radius, -radius), (radius, -radius), (radius, radius), (-radius, radius)]:
            from geometry_msgs.msg import Point
            pt = Point()
            pt.x = obs_x + dx * math.cos(psi) - dy * math.sin(psi)
            pt.y = obs_y + dx * math.sin(psi) + dy * math.cos(psi)
            pt.z = 0.0
            obs.polygon_points.append(pt)

        arr = ObstacleArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = "map"
        arr.obstacles.append(obs)
        self.pub_obs.publish(arr)


class LocalPlanningGui:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Local Planning Fine-Tuner & Live RViz Preview")
        self.root.geometry("640x880")
        self.root.minsize(580, 600)

        self.variables: Dict[str, Any] = {}
        self.scale_variables: Dict[str, tk.DoubleVar] = {}
        self.label_variables: Dict[str, tk.StringVar] = {}
        self.entry_variables: Dict[str, tk.StringVar] = {}

        self.pending_sync: Dict[str, Any] = {}
        self.sync_after_id: str | None = None

        # Setup TTK styles matching trajectory_gui.py
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Section.TLabel", font=("TkDefaultFont", 11, "bold"), foreground="#1a5fb4")
        style.configure("Group.TLabel", font=("TkDefaultFont", 10, "bold"), foreground="#3d3846")
        style.configure("Action.TButton", font=("TkDefaultFont", 10, "bold"))

        # Setup layout
        self.main_frame = ttk.Frame(self.root, padding=10)
        self.main_frame.pack(fill="both", expand=True)

        # Scrollable panel setup
        self.canvas = tk.Canvas(self.main_frame, highlightthickness=0)
        self.scrollbar = ttk.Scrollbar(self.main_frame, orient="vertical", command=self.canvas.yview)
        self.panel = ttk.Frame(self.canvas, padding=(4, 4, 16, 4))
        self.panel.bind(
            "<Configure>",
            lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")),
        )
        self.canvas_frame_id = self.canvas.create_window((0, 0), window=self.panel, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        self.canvas.bind("<Configure>", lambda e: self.canvas.itemconfig(self.canvas_frame_id, width=e.width))
        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")

        self.status_var = tk.StringVar(value="Ready. Loading parameters from config...")
        self.status_bar = ttk.Label(self.root, textvariable=self.status_var, relief="sunken", anchor="w", padding=(8, 4))
        self.status_bar.pack(side="bottom", fill="x")

        # Load initial values from YAML
        initial_values = self.load_yaml_values(DEFAULT_YAML_PATH)
        self.build_widgets(initial_values)

        # Initialize background ROS 2 node
        rclpy.init(args=None)
        self.ros_node = LocalPlanningGuiNode(self)
        self.ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self.ros_thread.start()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.status_var.set(f"Loaded config from {DEFAULT_YAML_PATH.name}. Connected to ROS 2.")

    def _spin_ros(self) -> None:
        try:
            rclpy.spin(self.ros_node)
        except Exception as e:
            pass

    def load_yaml_values(self, path: Path) -> Dict[str, Any]:
        result = {}
        if not path.exists():
            return result
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
            if not isinstance(data, dict):
                return result

            # Read from local_planner_node and static_obstacle_avoidance_node
            for node_key in ["local_planner_node", "static_obstacle_avoidance_node"]:
                node_data = data.get(node_key, {}).get("ros__parameters", {})
                if isinstance(node_data, dict):
                    for k, v in node_data.items():
                        if k not in result:
                            result[k] = v
        except Exception as e:
            print(f"Warning: Failed to load YAML: {e}")
        return result

    def build_widgets(self, initial_values: Dict[str, Any]) -> None:
        row = 0

        # Header Control Actions
        actions_frame = ttk.Frame(self.panel)
        actions_frame.grid(row=row, column=0, columnspan=3, sticky="ew", pady=(0, 12))
        actions_frame.columnconfigure(0, weight=1)
        actions_frame.columnconfigure(1, weight=1)
        actions_frame.columnconfigure(2, weight=1)

        ttk.Button(actions_frame, text="Save to YAML", style="Action.TButton", command=self.save_to_yaml).grid(row=0, column=0, sticky="ew", padx=2)
        ttk.Button(actions_frame, text="Reload YAML", command=self.reload_from_yaml).grid(row=0, column=1, sticky="ew", padx=2)
        ttk.Button(actions_frame, text="Reset Defaults", command=self.reset_defaults).grid(row=0, column=2, sticky="ew", padx=2)
        row += 1

        ttk.Separator(self.panel).grid(row=row, column=0, columnspan=3, sticky="ew", pady=6)
        row += 1

        # Virtual Obstacle Simulator (for RViz testing)
        ttk.Label(self.panel, text="Virtual Obstacle Simulation (RViz Live Testing)", style="Section.TLabel").grid(row=row, column=0, columnspan=3, sticky="w", pady=(6, 2))
        row += 1

        self.variables["enable_virtual_obs"] = tk.BooleanVar(value=False)
        ttk.Checkbutton(self.panel, text="Enable Virtual Obstacle on /obstacles topic (For stationary RViz testing)", variable=self.variables["enable_virtual_obs"]).grid(row=row, column=0, columnspan=3, sticky="w", padx=8, pady=4)
        row += 1

        # Group and build numeric scales
        current_group: str | None = None
        for spec in NUMERIC_SPECS:
            if spec.group != current_group:
                current_group = spec.group
                ttk.Separator(self.panel).grid(row=row, column=0, columnspan=3, sticky="ew", pady=(8, 4))
                row += 1
                ttk.Label(self.panel, text=current_group, style="Group.TLabel").grid(row=row, column=0, columnspan=3, sticky="w", padx=4, pady=(6, 2))
                row += 1

            value = normalize_numeric_value(spec, initial_values.get(spec.key, spec.default))
            self.variables[spec.key] = tk.DoubleVar(value=float(value))
            row = self._scale(row, spec)

        self.panel.columnconfigure(1, weight=1)
        self.panel.columnconfigure(2, weight=0)

    def _scale(self, row: int, spec: NumericSpec) -> int:
        variable = self.variables[spec.key]
        slider_var = tk.DoubleVar(
            value=float(normalize_numeric_value(spec, variable.get(), clamp_to_slider=True))
        )
        self.scale_variables[spec.key] = slider_var
        label_var = tk.StringVar()
        entry_var = tk.StringVar()
        self.label_variables[spec.key] = label_var
        self.entry_variables[spec.key] = entry_var
        syncing_slider = False

        def update_widgets(*_: Any, force_entry: bool = False) -> None:
            nonlocal syncing_slider
            formatted = format_numeric_value(spec, variable.get())
            label_var.set(spec.label)
            if force_entry or self.root.focus_get() is not entry:
                entry_var.set(formatted)
            slider_value = normalize_numeric_value(spec, variable.get(), clamp_to_slider=True)
            if abs(float(slider_var.get()) - float(slider_value)) > 1e-12:
                syncing_slider = True
                try:
                    slider_var.set(float(slider_value))
                finally:
                    syncing_slider = False

            # Schedule debounced ROS 2 parameter sync
            self.schedule_sync(spec.key, variable.get(), spec)

        def apply_entry(_event: tk.Event | None = None) -> str:
            text = entry_var.get().strip()
            try:
                value = normalize_numeric_value(spec, text)
            except (TypeError, ValueError):
                update_widgets(force_entry=True)
                self.status_var.set(f"Invalid {spec.label}: {text}")
                return "break"
            variable.set(float(value))
            entry_var.set(format_numeric_value(spec, value))
            return "break"

        def apply_scale(value_text: str) -> None:
            if syncing_slider:
                return
            value = normalize_numeric_value(spec, value_text, clamp_to_slider=True)
            if abs(float(variable.get()) - float(value)) > 1e-12:
                variable.set(float(value))

        variable.trace_add("write", update_widgets)
        control = ttk.Frame(self.panel)
        control.grid(row=row, column=0, columnspan=3, sticky="ew", padx=8, pady=(3, 3))
        control.columnconfigure(2, weight=1)

        ttk.Label(control, textvariable=label_var, width=24, anchor="w").grid(row=0, column=0, sticky="w")
        entry = ttk.Entry(control, textvariable=entry_var, width=9, justify="right")
        entry.grid(row=0, column=1, sticky="ew", padx=(6, 6))
        scale = ttk.Scale(
            control,
            from_=spec.min_value,
            to=spec.max_value,
            variable=slider_var,
            command=apply_scale,
        )
        scale.grid(row=0, column=2, sticky="ew")
        entry.bind("<Return>", apply_entry)
        entry.bind("<KP_Enter>", apply_entry)
        entry.bind("<FocusOut>", apply_entry)
        update_widgets(force_entry=True)
        return row + 1

    def schedule_sync(self, key: str, value: Any, spec: NumericSpec) -> None:
        self.pending_sync[key] = (value, spec)
        if self.sync_after_id is not None:
            self.root.after_cancel(self.sync_after_id)
        self.sync_after_id = self.root.after(80, self.flush_sync)

    def flush_sync(self) -> None:
        self.sync_after_id = None
        if not hasattr(self, "ros_node"):
            return
        for key, (value, spec) in list(self.pending_sync.items()):
            norm_val = normalize_numeric_value(spec, value)
            self.ros_node.sync_parameter(spec.node_target, key, norm_val, spec)
        self.pending_sync.clear()
        self.status_var.set("Synchronized parameters to ROS 2 nodes & RViz.")

    def save_to_yaml(self) -> None:
        try:
            # Read existing structure if possible
            existing_data = {}
            if DEFAULT_YAML_PATH.exists():
                with open(DEFAULT_YAML_PATH, "r", encoding="utf-8") as f:
                    existing_data = yaml.safe_load(f) or {}

            if "local_planner_node" not in existing_data:
                existing_data["local_planner_node"] = {"ros__parameters": {}}
            if "static_obstacle_avoidance_node" not in existing_data:
                existing_data["static_obstacle_avoidance_node"] = {"ros__parameters": {}}

            lp_params = existing_data["local_planner_node"].setdefault("ros__parameters", {})
            soa_params = existing_data["static_obstacle_avoidance_node"].setdefault("ros__parameters", {})

            for spec in NUMERIC_SPECS:
                if spec.node_target == "virtual_sim":
                    continue
                val = normalize_numeric_value(spec, self.variables[spec.key].get())
                if spec.node_target in ["local_planner_node", "both"]:
                    lp_params[spec.key] = val
                if spec.node_target in ["static_obstacle_avoidance_node", "both"]:
                    soa_params[spec.key] = val

            DEFAULT_YAML_PATH.parent.mkdir(parents=True, exist_ok=True)
            with open(DEFAULT_YAML_PATH, "w", encoding="utf-8") as f:
                yaml.dump(existing_data, f, allow_unicode=True, default_flow_style=False, sort_keys=False)

            self.status_var.set(f"Successfully saved all parameters to {DEFAULT_YAML_PATH}")
            messagebox.showinfo("Saved", f"Parameters successfully saved to:\n{DEFAULT_YAML_PATH}")
        except Exception as e:
            messagebox.showerror("Save Error", f"Failed to save YAML:\n{e}")

    def reload_from_yaml(self) -> None:
        if not DEFAULT_YAML_PATH.exists():
            messagebox.showwarning("File not found", f"YAML file not found at {DEFAULT_YAML_PATH}")
            return
        vals = self.load_yaml_values(DEFAULT_YAML_PATH)
        for spec in NUMERIC_SPECS:
            if spec.key in vals:
                norm = normalize_numeric_value(spec, vals[spec.key])
                self.variables[spec.key].set(float(norm))
        self.status_var.set(f"Reloaded values from {DEFAULT_YAML_PATH.name}")

    def reset_defaults(self) -> None:
        for spec in NUMERIC_SPECS:
            self.variables[spec.key].set(float(spec.default))
        self.status_var.set("Reset all parameters to factory defaults.")

    def on_close(self) -> None:
        if hasattr(self, "ros_node"):
            try:
                self.ros_node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    app = LocalPlanningGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
