#!/usr/bin/env python3
"""
ROS2 optimizer bridge that generates global_waypoints.json using the real
global_racetrajectory_optimization trajectory_optimizer pipeline.
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path

import cv2
import numpy as np
from scipy.signal import savgol_filter
from skimage.morphology import skeletonize

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from global_racetrajectory_optimization import helper_funcs_glob
from global_racetrajectory_optimization.trajectory_optimizer import trajectory_optimizer


def _extract_centerline(skeleton: np.ndarray, resolution: float) -> np.ndarray:
    contours, hierarchy = cv2.findContours(skeleton.astype(np.uint8), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if hierarchy is None or len(contours) == 0:
        raise RuntimeError("No contours found in skeleton")

    closed = []
    for i, cont in enumerate(contours):
        opened = hierarchy[0][i][2] < 0 and hierarchy[0][i][3] < 0
        if not opened:
            closed.append(cont)
    if not closed:
        raise RuntimeError("No closed contours found in skeleton")

    lengths = []
    for cont in closed:
        c = np.array(cont).reshape(-1, 2)
        seg = np.diff(np.vstack([c, c[0]]), axis=0)
        lengths.append(np.sum(np.linalg.norm(seg, axis=1)) * resolution)

    idx = int(np.argmin(lengths))
    return np.array(closed[idx]).reshape(-1, 2)


def _smooth_centerline(centerline: np.ndarray) -> np.ndarray:
    n = len(centerline)
    if n > 2000:
        fl = int(n / 200) * 10 + 1
    elif n > 1000:
        fl = 81
    elif n > 500:
        fl = 41
    else:
        fl = 21
    fl = min(fl, n - 1 if (n - 1) % 2 == 1 else n - 2)
    if fl < 5:
        return centerline
    c1 = savgol_filter(centerline, fl, 3, axis=0)
    half = n // 2
    c2 = np.append(centerline[half:], centerline[:half], axis=0)
    c2 = savgol_filter(c2, fl, 3, axis=0)
    c1[:fl] = c2[half:half + fl]
    c1[-fl:] = c2[half - fl:half]
    return c1


def _write_centerline_csv(path: Path, centerline_xy: np.ndarray, d_right: np.ndarray, d_left: np.ndarray) -> None:
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        for (x, y), dr, dl in zip(centerline_xy, d_right, d_left):
            w.writerow([float(x), float(y), float(dr), float(dl)])


def _wpnt_array_from_traj(traj: np.ndarray, d_right: np.ndarray, d_left: np.ndarray) -> dict:
    wpnts = []
    for i, p in enumerate(traj):
        # traj: [s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2]
        wpnts.append({
            "id": int(i),
            "s_m": float(p[0]),
            "d_m": 0.0,
            "x_m": float(p[1]),
            "y_m": float(p[2]),
            "d_right": float(d_right[i]),
            "d_left": float(d_left[i]),
            "psi_rad": float(p[3]),
            "kappa_radpm": float(p[4]),
            "vx_mps": float(p[5]),
            "ax_mps2": float(p[6]),
        })
    return {"header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"}, "wpnts": wpnts}


def _head_curv_num(path_xy: np.ndarray, ds: float) -> tuple[np.ndarray, np.ndarray]:
    """Numerical heading/curvature for polyline path (closed=False)."""
    n = len(path_xy)
    if n < 3:
        return np.zeros(max(0, n - 1)), np.zeros(max(0, n - 1))
    dxy = np.diff(path_xy, axis=0)
    psi = np.arctan2(dxy[:, 1], dxy[:, 0])
    psi_unwrap = np.unwrap(psi)
    kappa = np.gradient(psi_unwrap, ds)
    return psi_unwrap, kappa


def _marker_array_from_wpnts(wpnts: dict, color: tuple[float, float, float], scale: float = 0.05) -> dict:
    out = []
    for i, w in enumerate(wpnts["wpnts"]):
        out.append({
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
            "ns": "",
            "id": int(i),
            "type": 2,
            "action": 0,
            "pose": {
                "position": {"x": w["x_m"], "y": w["y_m"], "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            "scale": {"x": scale, "y": scale, "z": scale},
            "color": {"r": color[0], "g": color[1], "b": color[2], "a": 1.0},
            "lifetime": {"sec": 0, "nanosec": 0},
            "frame_locked": False,
            "points": [],
            "colors": [],
            "text": "",
            "mesh_resource": "",
            "mesh_use_embedded_materials": False,
        })
    return {"markers": out}


class ProbeNode(Node):
    def __init__(self):
        super().__init__("gb_optimizer_ros2_probe")
        self.map_msg = None
        self.odom_msg = None
        self.create_subscription(OccupancyGrid, "/map", self.map_cb, 10)
        self.create_subscription(Odometry, "/pf/pose/odom", self.odom_cb, 10)

    def map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg

    def odom_cb(self, msg: Odometry):
        self.odom_msg = msg


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-dir", required=True)
    parser.add_argument("--wait-timeout-sec", type=float, default=10.0)
    parser.add_argument("--safety-width", type=float, default=0.5)
    parser.add_argument("--safety-width-sp", type=float, default=0.2)
    args = parser.parse_args()

    map_dir = Path(args.map_dir)
    map_dir.mkdir(parents=True, exist_ok=True)
    out_json = map_dir / "global_waypoints.json"

    rclpy.init(args=None)
    node = ProbeNode()
    deadline = node.get_clock().now().nanoseconds + int(args.wait_timeout_sec * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.map_msg is not None and node.odom_msg is not None:
            break
    if node.map_msg is None:
        node.destroy_node()
        rclpy.shutdown()
        print("[global_planner_node_ros2] Missing /map", flush=True)
        return 2

    map_msg = node.map_msg
    node.destroy_node()
    rclpy.shutdown()

    h = map_msg.info.height
    w = map_msg.info.width
    res = float(map_msg.info.resolution)
    ox = float(map_msg.info.origin.position.x)
    oy = float(map_msg.info.origin.position.y)

    grid = np.array(map_msg.data, dtype=np.int16).reshape((h, w))
    grid = np.where(grid == -1, 100, grid)
    bw = np.where(grid < 50, 255, 0).astype(np.uint8)
    opening = cv2.morphologyEx(bw, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8), iterations=2)
    skel = skeletonize(opening, method="lee")

    centerline_px = _extract_centerline(skel, res)
    centerline_px = _smooth_centerline(centerline_px)
    centerline_xy = np.zeros_like(centerline_px, dtype=float)
    centerline_xy[:, 0] = centerline_px[:, 0] * res + ox
    centerline_xy[:, 1] = centerline_px[:, 1] * res + oy

    centerline_for_interp = np.column_stack([centerline_xy, np.zeros((len(centerline_xy), 2))])
    centerline_xy = helper_funcs_glob.src.interp_track.interp_track(
        reftrack=centerline_for_interp, stepsize_approx=0.1
    )[:, :2]

    # Distance transform based track widths
    dist = cv2.distanceTransform(opening, cv2.DIST_L2, 5)
    px = np.clip(np.round((centerline_xy[:, 0] - ox) / res).astype(int), 0, w - 1)
    py = np.clip(np.round((centerline_xy[:, 1] - oy) / res).astype(int), 0, h - 1)
    widths = np.clip(dist[py, px] * res, 0.2, 5.0)
    d_right = widths
    d_left = widths

    module_dir = Path(__file__).resolve().parent / "global_racetrajectory_optimization"
    inputs_dir = module_dir / "inputs"
    cfg_dir = Path(__file__).resolve().parent.parent / "config" / "global_planner"
    # trajectory_optimizer expects racecar_f110.ini in input_path root
    ini_src = cfg_dir / "racecar_f110.ini"
    ini_dst = inputs_dir / "racecar_f110.ini"
    if ini_src.exists():
        ini_dst.write_text(ini_src.read_text())
    veh_cfg_src = cfg_dir / "veh_dyn_info"
    veh_cfg_dst = inputs_dir / "veh_dyn_info"
    veh_cfg_dst.mkdir(parents=True, exist_ok=True)
    for name in ("ggv.csv", "ax_max_machines.csv"):
        src = veh_cfg_src / name
        if src.exists():
            (veh_cfg_dst / name).write_text(src.read_text())

    _write_centerline_csv(inputs_dir / "map_centerline.csv", centerline_xy, d_right, d_left)
    os.chdir(str(inputs_dir))

    traj_iqp, bound_r_iqp, bound_l_iqp, est_iqp = trajectory_optimizer(
        input_path=str(inputs_dir),
        track_name="map_centerline",
        curv_opt_type="mincurv_iqp",
        safety_width=float(args.safety_width),
        plot=False,
    )

    _write_centerline_csv(inputs_dir / "map_centerline_2.csv", traj_iqp[:, 1:3], d_right[: len(traj_iqp)], d_left[: len(traj_iqp)])
    traj_sp, bound_r_sp, bound_l_sp, est_sp = trajectory_optimizer(
        input_path=str(inputs_dir),
        track_name="map_centerline_2",
        curv_opt_type="shortest_path",
        safety_width=float(args.safety_width_sp),
        plot=False,
    )

    # Distances to bounds for waypoint payload
    # Upstream helper returns one min-distance array (not separate right/left arrays).
    min_d_iqp = helper_funcs_glob.src.calc_min_bound_dists.calc_min_bound_dists(
        trajectory=traj_iqp, bound1=bound_r_iqp, bound2=bound_l_iqp, length_veh=0.0, width_veh=0.0
    )
    min_d_sp = helper_funcs_glob.src.calc_min_bound_dists.calc_min_bound_dists(
        trajectory=traj_sp, bound1=bound_r_sp, bound2=bound_l_sp, length_veh=0.0, width_veh=0.0
    )
    dr_iqp = min_d_iqp
    dl_iqp = min_d_iqp
    dr_sp = min_d_sp
    dl_sp = min_d_sp

    # centerline waypoint fields
    center_wp = []
    psi_center, kappa_center = _head_curv_num(centerline_xy, 0.1)
    for i, (xy, psi, kap, dr, dl) in enumerate(zip(centerline_xy[:-1], psi_center, kappa_center, d_right[:-1], d_left[:-1])):
        center_wp.append({
            "id": int(i),
            "s_m": float(i * 0.1),
            "d_m": 0.0,
            "x_m": float(xy[0]),
            "y_m": float(xy[1]),
            "d_right": float(dr),
            "d_left": float(dl),
            "psi_rad": float(psi + np.pi / 2.0),
            "kappa_radpm": float(kap),
            "vx_mps": 0.0,
            "ax_mps2": 0.0,
        })
    center_arr = {"header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"}, "wpnts": center_wp}

    iqp_arr = _wpnt_array_from_traj(traj_iqp, dr_iqp, dl_iqp)
    sp_arr = _wpnt_array_from_traj(traj_sp, dr_sp, dl_sp)

    out = {
        "map_info_str": {"data": f"IQP estimated lap time: {est_iqp:.4f}s; SP estimated lap time: {est_sp:.4f}s;"},
        "est_lap_time": {"data": float(est_sp)},
        "centerline_markers": _marker_array_from_wpnts(center_arr, (0.0, 0.0, 1.0)),
        "centerline_waypoints": center_arr,
        "global_traj_markers_iqp": _marker_array_from_wpnts(iqp_arr, (1.0, 0.2, 0.0)),
        "global_traj_wpnts_iqp": iqp_arr,
        "global_traj_markers_sp": _marker_array_from_wpnts(sp_arr, (0.5, 1.0, 0.0)),
        "global_traj_wpnts_sp": sp_arr,
        "trackbounds_markers": {"markers": []},
    }

    out_json.write_text(json.dumps(out))
    print(f"[global_planner_node_ros2] Wrote {out_json}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
