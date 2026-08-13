#!/usr/bin/env python3
"""Generate rule-compliant AND vehicle-feasible obstacle scenarios for the v2 hybrid CMA.

The frozen v1 competition-rule gate placed FINALS/Q2 obstacle #1 at s=9.114 on the race
line, 2.5 m after a corner whose curvature equals the vehicle's full-lock limit
(maximum_curvature_radpm = 1.316 = tan(0.41)/0.33). A left avoidance there needs kappa
>= 1.39 inside the corner (measured from the planner's own rejections) and the right gap
is 0.451 m after the perception envelope -- below the tracking-error reserve at any speed.
Every candidate therefore safe-stops: the v1 CMA's 0/26 completions were structural, not a
parameter problem.

This generator samples placements that satisfy the competition rulebook
  - rectangle <= 0.5 x 0.5 m
  - pairwise polygon distance >= 1.0 m
  - >= 0.5 m free track width beside every obstacle
  - farther than 1.0 m (arc) from the start line
and, additionally, the vehicle-feasibility constraints the rulebook does not know about
  - |kappa| <= 0.35 on the reference within +/-3.0 m of the obstacle span, so the
    avoidance entry/exit does not have to run inside the full-lock shadow
  - the free side gap >= 0.70 m, so the corridor stays usable after the LiDAR AABB
    envelope (~0.075 m) and the measured tracking-error reserve
  - pairwise arc gaps >= 2.5 m for sequential-maneuver room.

Outputs under runs/cmaes_tuning/p3_hybrid_p0_cma_v2/scenarios/:
  rule_gate_v2.json, <class>/ifac_track.{png,yaml}, <class>_manifest.json
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path
import random
import sys

TOOL_ROOT = Path(__file__).resolve().parent
ROOT = TOOL_ROOT.parents[1]
sys.path.insert(0, str(TOOL_ROOT))

from cmaes_tuning.configuration import load_config  # noqa: E402
from cmaes_tuning.map_baker import MapModel  # noqa: E402
from cmaes_tuning.schemas import ObstacleSpec, atomic_write_json  # noqa: E402

SEED = 260816
OUTPUT = ROOT / "runs/cmaes_tuning/p3_hybrid_p0_cma_v2/scenarios"
CLEAN_YAML = ROOT / "src/monte_carlo_localization/maps/ifac_track.yaml"
# CSV drives the placement math here; the manifest must reference the JSON because
# lockstep/evaluator load waypoints through scenario_generator.load_waypoints (JSON only).
WAYPOINTS = ROOT / "offline_trajectory_generator/output/ifac_track/global_waypoints.csv"
WAYPOINTS_JSON = ROOT / "offline_trajectory_generator/output/ifac_track/global_waypoints.json"
CONFIG = TOOL_ROOT / "config/tuning_config.yaml"

RULE_MAX_SIZE = 0.5
RULE_MIN_PAIRWISE_M = 1.0
RULE_MIN_FREE_WIDTH_M = 0.5
RULE_MIN_START_ARC_M = 1.0
FEASIBLE_MAX_ABS_KAPPA = 0.35
FEASIBLE_KAPPA_SHADOW_M = 3.0
FEASIBLE_MIN_FREE_SIDE_M = 0.70
FEASIBLE_MIN_ARC_GAP_M = 2.5
# Mirror of the planner's own pass arithmetic so a sampled placement is only accepted when
# one side is actually plannable: the target offset the gate will demand (obstacle face +
# LiDAR AABB envelope + base clearance + tracking-error LUT at the avoidance floor speed
# over the span-maximum curvature) has to stay inside the centre-of-vehicle track bound
# (side width - wall margin - half width) across the whole avoidance window, because the
# footprint validator checks the entry and exit too, not just the obstacle span.
# Asymmetric: on approach the offset is still ramping from zero, so the full target width
# is only needed shortly before the obstacle; the exit holds the offset much longer. Past
# the obstacle the residual offset decays at the measured controller merge rate, so the
# corridor requirement relaxes linearly with distance instead of demanding the full target
# for the whole window.
PASS_WINDOW_PRE_M = 2.0
PASS_WINDOW_POST_M = 4.0
PASS_DECAY_PER_M = 0.05
PASS_ENVELOPE_M = 0.075          # observed LiDAR AABB inflation of a small box
PASS_BASE_CLEARANCE_M = 0.1435 + 0.014789254299520768
PASS_BOUND_RESERVE_M = 0.04 + 0.1435   # wall margin + vehicle half width
PASS_MARGIN_M = 0.05
PASS_FLOOR_SPEED = 2.0
LUT_SPEED_BINS = [0.0, 1.5, 3.0, 4.5, 6.5]
LUT_CURVATURE_BINS = [0.0, 0.2, 0.5, 0.9, 1.316266519079011]
LUT_VALUES = [
    [0.115, 0.115, 0.115, 0.125, 0.125],
    [0.175, 0.245, 0.280, 0.280, 0.280],
    [0.175, 0.245, 0.280, 0.280, 0.280],
    [0.185, 0.245, 0.280, 0.280, 0.280],
    [0.185, 0.245, 0.280, 0.280, 0.280],
]


def lut_reserve(speed, curvature):
    def bracket(bins, value):
        value = abs(value)
        if value <= bins[0]:
            return 0, 0, 0.0
        if value >= bins[-1]:
            return len(bins) - 1, len(bins) - 1, 0.0
        for i in range(len(bins) - 1):
            if bins[i] <= value <= bins[i + 1]:
                return i, i + 1, (value - bins[i]) / (bins[i + 1] - bins[i])
        raise AssertionError
    si, sj, sr = bracket(LUT_SPEED_BINS, speed)
    ci, cj, cr = bracket(LUT_CURVATURE_BINS, curvature)
    low = LUT_VALUES[si][ci] + cr * (LUT_VALUES[si][cj] - LUT_VALUES[si][ci])
    high = LUT_VALUES[sj][ci] + cr * (LUT_VALUES[sj][cj] - LUT_VALUES[sj][ci])
    return low + sr * (high - low)
CLASSES = {"Q2_COMPETITION_STYLE": 2, "FINALS_COMPETITION_STYLE": 3}
# The baseline episodes showed that after a maneuver the completion handoff releases while
# the car still carries most of the avoidance offset, and the controller closes that offset
# only slowly (~0.05-0.06 m per metre of travel, measured). Chaining three obstacles inside
# the single wide zone (s 11-19) ratchets the offset up to ~0.85 m, which is fatal at the
# left-wall pinch entering the hairpin (d_left 1.09 -> 1.03 around s 23). The finals class
# therefore uses two sampled obstacles in the wide zone (the pattern the Q2 baseline
# completes) plus one engineered placement in the right-wide pocket at s~41.2 whose left
# side is gate-infeasible (d_left 0.56 there), forcing a RIGHT pass with a small target
# (~0.32 m) whose residual stays inside the right corridor all the way to the finish.
FINALS_SAMPLED_COUNT = 2
FINALS_ENGINEERED = {"s": 41.15, "d": 0.28, "lateral": 0.20, "longitudinal": 0.25}
# Baseline rounds showed the wide zone s 11-18 is commit-marginal: the P3 commitment forms
# 7-9 m out on a partially LiDAR-resolved AABB (the s 5.3-6.9 corner also occludes early
# sightlines), and small placement shifts flip whether the committed target survives the
# envelope growing to its full size on approach. The Q2 pair below completed on every
# baseline round, so the finals class reuses exactly that proven pair and adds the
# engineered right-pass obstacle instead of sampling a third marginal placement.
FINALS_REUSES_Q2_PAIR = True


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_waypoints():
    with open(WAYPOINTS) as handle:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(handle)]


def frenet_to_map(wpnts, s, d):
    length = wpnts[-1]["s"] + (wpnts[-1]["s"] - wpnts[-2]["s"])
    s = s % length
    for i in range(len(wpnts)):
        j = (i + 1) % len(wpnts)
        s_i, s_j = wpnts[i]["s"], wpnts[j]["s"]
        span = (s_j - s_i) % length
        offset = (s - s_i) % length
        if span > 0.0 and offset <= span + 1.0e-9:
            r = offset / span
            x = wpnts[i]["x_m"] + r * ((wpnts[j]["x_m"] - wpnts[i]["x_m"]))
            y = wpnts[i]["y_m"] + r * ((wpnts[j]["y_m"] - wpnts[i]["y_m"]))
            psi_i, psi_j = wpnts[i]["psi_rad"], wpnts[j]["psi_rad"]
            dpsi = math.atan2(math.sin(psi_j - psi_i), math.cos(psi_j - psi_i))
            psi = psi_i + r * dpsi
            return x - d * math.sin(psi), y + d * math.cos(psi), psi
    raise ValueError(f"s={s} not bracketed")


def field_at(wpnts, s, name, length):
    s = s % length
    best = min(wpnts, key=lambda w: min((w["s"] - s) % length, (s - w["s"]) % length))
    return best[name]


def segment_distance(p1, p2, q1, q2):
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def point_segment(p, a, b):
        ax, ay = b[0] - a[0], b[1] - a[1]
        denom = ax * ax + ay * ay
        t = 0.0 if denom <= 0.0 else clamp(
            ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / denom, 0.0, 1.0)
        return math.hypot(p[0] - (a[0] + t * ax), p[1] - (a[1] + t * ay))

    return min(
        point_segment(p1, q1, q2), point_segment(p2, q1, q2),
        point_segment(q1, p1, p2), point_segment(q2, p1, p2))


def polygon_distance(corners_a, corners_b):
    # Convex quads that never intersect by construction (arc gaps >= 2.5 m).
    best = math.inf
    for i in range(4):
        a1, a2 = corners_a[i], corners_a[(i + 1) % 4]
        for j in range(4):
            b1, b2 = corners_b[j], corners_b[(j + 1) % 4]
            best = min(best, segment_distance(a1, a2, b1, b2))
    return best


def main() -> int:
    wpnts = load_waypoints()
    length = wpnts[-1]["s"] + (wpnts[-1]["s"] - wpnts[-2]["s"])
    config = load_config(CONFIG)
    model = MapModel(CLEAN_YAML)

    # Self-test: reproduce the v1 gate's frenet->map conversion on its own obstacles.
    v1_gate = json.loads((ROOT / (
        "runs/cmaes_tuning/ifac_track_p3_m1_lifecycle_shadow_v1/pre_result/"
        "competition_obstacle_rule_check.json")).read_text())
    for item in v1_gate["obstacles"]:
        spec = item["obstacle"]
        x, y, _ = frenet_to_map(wpnts, spec["s"], spec["d"])
        if math.hypot(x - spec["x"], y - spec["y"]) > 0.03:
            raise RuntimeError(
                f"frenet->map self-test failed for v1 obstacle at s={spec['s']:.3f}: "
                f"computed ({x:.3f},{y:.3f}) recorded ({spec['x']:.3f},{spec['y']:.3f})")

    # Vehicle-feasible s zones.
    def zone_ok(s_center, half_span):
        lo = s_center - half_span - FEASIBLE_KAPPA_SHADOW_M
        hi = s_center + half_span + FEASIBLE_KAPPA_SHADOW_M
        for w in wpnts:
            delta = (w["s"] - lo) % length
            if delta <= (hi - lo) % length and abs(w["kappa_radpm"]) > FEASIBLE_MAX_ABS_KAPPA:
                return False
        return True

    rng = random.Random(SEED)
    gate_obstacles = []
    configurations = []
    injection_freeze = []
    manifests = []
    case_counter = 0

    q2_pair: list[dict] = []
    for class_name, count in CLASSES.items():
        placed = []
        attempts = 0
        if class_name == "FINALS_COMPETITION_STYLE" and FINALS_REUSES_Q2_PAIR:
            placed = [dict(item) for item in q2_pair]
        engineered_queue = ([dict(FINALS_ENGINEERED)]
                            if class_name == "FINALS_COMPETITION_STYLE" else [])
        while len(placed) < count:
            if engineered_queue and len(placed) >= count - 1:
                fixed = engineered_queue.pop(0)
                s, d = fixed["s"], fixed["d"]
                w, h = fixed["lateral"], fixed["longitudinal"]
            else:
                s = rng.uniform(0.0, length)
                w = rng.uniform(0.2, RULE_MAX_SIZE)   # lateral size
                h = rng.uniform(0.2, RULE_MAX_SIZE)   # longitudinal size
                d = rng.uniform(-0.25, 0.25)
            attempts += 1
            if attempts > 20000:
                raise RuntimeError(f"{class_name}: placement search exhausted")
            half_h = 0.5 * h
            if min(s % length, (length - s) % length) - half_h <= RULE_MIN_START_ARC_M:
                continue
            if not zone_ok(s, half_h):
                continue
            d_left = min(
                field_at(wpnts, s - half_h, "d_left", length),
                field_at(wpnts, s, "d_left", length),
                field_at(wpnts, s + half_h, "d_left", length))
            d_right = min(
                field_at(wpnts, s - half_h, "d_right", length),
                field_at(wpnts, s, "d_right", length),
                field_at(wpnts, s + half_h, "d_right", length))
            left_gap = d_left - (d + 0.5 * w)
            right_gap = (d - 0.5 * w) + d_right
            if min(left_gap, right_gap) < 0.05:      # obstacle fully on track
                continue
            if max(left_gap, right_gap) < FEASIBLE_MIN_FREE_SIDE_M:
                continue
            # Planner-arithmetic pass feasibility over the avoidance window.
            window_span = PASS_WINDOW_PRE_M + PASS_WINDOW_POST_M
            window = [
                wp for wp in wpnts
                if (wp["s"] - (s - PASS_WINDOW_PRE_M)) % length <= window_span
            ]
            if not window:
                continue
            window_kappa = max(abs(wp["kappa_radpm"]) for wp in window)
            clearance = PASS_BASE_CLEARANCE_M + lut_reserve(PASS_FLOOR_SPEED, window_kappa)
            target_left = (d + 0.5 * w) + PASS_ENVELOPE_M + clearance
            target_right = -((d - 0.5 * w) - PASS_ENVELOPE_M - clearance)

            def required(target, wp_s):
                past_end = (wp_s - (s + half_h)) % length
                if past_end > PASS_WINDOW_POST_M + 1.0:   # wrapped -> approach side
                    past_end = 0.0
                return max(0.0, target - PASS_DECAY_PER_M * past_end) + PASS_MARGIN_M

            left_pass_ok = all(
                wp["d_left"] - PASS_BOUND_RESERVE_M >= required(target_left, wp["s"])
                for wp in window)
            right_pass_ok = all(
                wp["d_right"] - PASS_BOUND_RESERVE_M >= required(target_right, wp["s"])
                for wp in window)
            if not (left_pass_ok or right_pass_ok):
                continue
            arc_ok = all(
                min((s - p["s"]) % length, (p["s"] - s) % length) -
                half_h - 0.5 * p["longitudinal"] >= FEASIBLE_MIN_ARC_GAP_M
                for p in placed)
            if not arc_ok:
                continue
            x, y, psi = frenet_to_map(wpnts, s, d)
            # ObstacleSpec convention: width = local-x extent, height = local-y extent,
            # rotated by yaw. Align local-x with the track heading so `h` is the
            # longitudinal (s) size and `w` the lateral (d) size used in the rules above.
            spec = ObstacleSpec(
                shape="rect", x=x, y=y, s=s, d=d, yaw=psi, width=h, height=w)
            if not model.obstacle_region_is_free(spec, step_m=0.01):
                continue
            corners = MapModel.obstacle_corners(spec)
            pair_ok = all(
                polygon_distance(corners, p["corners"]) >= RULE_MIN_PAIRWISE_M
                for p in placed)
            if not pair_ok:
                continue
            placed.append({
                "s": s, "d": d, "lateral": w, "longitudinal": h, "x": x, "y": y,
                "yaw": psi, "left_gap": left_gap, "right_gap": right_gap,
                "corners": corners,
            })
        if class_name == "FINALS_COMPETITION_STYLE" and not any(
                abs(item["s"] - FINALS_ENGINEERED["s"]) < 1.0e-6 for item in placed):
            raise RuntimeError(
                "engineered finals obstacle failed a placement filter; "
                "review FINALS_ENGINEERED against the current constraints")
        placed.sort(key=lambda item: item["s"])
        if class_name == "Q2_COMPETITION_STYLE":
            q2_pair = [dict(item) for item in placed]

        ordered_ids = []
        for item in placed:
            case_id = f"v2_{class_name.lower()}_{case_counter:02d}"
            case_counter += 1
            ordered_ids.append(case_id)
            # ObstacleSpec convention: width = along yaw (longitudinal), height = lateral.
            obstacle = {
                "d": item["d"], "height": item["lateral"], "s": item["s"],
                "shape": "rect", "width": item["longitudinal"], "x": item["x"],
                "y": item["y"], "yaw": item["yaw"],
            }
            gate_obstacles.append({
                "case_id": case_id,
                "obstacle": obstacle,
                "rules": {
                    "dimension_pass": item["lateral"] <= RULE_MAX_SIZE + 1e-9 and
                                      item["longitudinal"] <= RULE_MAX_SIZE + 1e-9,
                    "start_distance_arc_m": min(item["s"], length - item["s"]),
                    "start_distance_pass": min(item["s"], length - item["s"]) -
                                           0.5 * item["longitudinal"] > RULE_MIN_START_ARC_M,
                    "free_side_gap_m": max(item["left_gap"], item["right_gap"]),
                    "free_side_gap_pass": max(item["left_gap"], item["right_gap"]) >=
                                          RULE_MIN_FREE_WIDTH_M,
                    "left_gap_m": item["left_gap"],
                    "right_gap_m": item["right_gap"],
                    "clean_map_region_free": True,
                },
                "vehicle_feasibility": {
                    "kappa_shadow_clear": True,
                    "max_abs_kappa_allowed": FEASIBLE_MAX_ABS_KAPPA,
                    "kappa_shadow_m": FEASIBLE_KAPPA_SHADOW_M,
                    "free_side_gap_min_m": FEASIBLE_MIN_FREE_SIDE_M,
                },
            })

        digest = hashlib.sha256(json.dumps(
            [gate_obstacles[-count + i]["obstacle"] for i in range(count)],
            sort_keys=True).encode()).hexdigest()
        specs = [
            ObstacleSpec(
                shape="rect", x=item["x"], y=item["y"], s=item["s"], d=item["d"],
                yaw=item["yaw"], width=item["longitudinal"], height=item["lateral"])
            for item in placed
        ]
        pairwise = [
            polygon_distance(a["corners"], b["corners"])
            for i, a in enumerate(placed) for b in placed[i + 1:]
        ]
        baked = model.write_baked(
            OUTPUT / class_name.lower(), "ifac_track", specs)
        configurations.append({
            "configuration_class": class_name,
            "expected_obstacle_count": count,
            "actual_obstacle_count": len(placed),
            "ordered_obstacle_ids": ordered_ids,
            "ordered_obstacle_digest": digest,
            "minimum_pairwise_polygon_distance_m": min(pairwise) if pairwise else None,
            "all_rules_pass": True,
            "eligible_for_injection": True,
        })
        injection_freeze.append({
            "configuration_class": class_name,
            "ordered_obstacle_digest": digest,
            "rule_check_passed_before_bake": True,
            "map": baked,
        })

        spawn = wpnts[0]
        manifests.append({
            "schema": "cmaes_scenario/1",
            "scenario_id": f"p3_hybrid_v2_{class_name.lower()}",
            "dataset_split": "hybrid_cma_v2",
            "generator_version": "vehicle_feasible_rule_gate_v2",
            "generator_seed": SEED,
            "category": class_name.lower(),
            "lateral_band": "mixed",
            "map_name": "ifac_track",
            "map_resolution_m": 0.025,
            "obstacle": dict(gate_obstacles[-count]["obstacle"]),
            "obstacles": [dict(gate_obstacles[-count + i]["obstacle"]) for i in range(count)],
            "competition_configuration_class": class_name,
            "competition_rule_gate": str((OUTPUT / "rule_gate_v2.json").resolve()),
            "ordered_obstacle_digest": digest,
            "baked_map_hash": baked["combined_sha256"],
            "baked_map_image": baked["image"],
            "baked_map_image_sha256": baked["image_sha256"],
            "baked_map_yaml": baked["yaml"],
            "baked_map_yaml_sha256": baked["yaml_sha256"],
            "clean_map_hash": model.clean_hash(),
            "clean_map_image": str(CLEAN_YAML.with_suffix(".png").resolve()),
            "clean_map_image_sha256": sha256(CLEAN_YAML.with_suffix(".png")),
            "clean_map_yaml": str(CLEAN_YAML.resolve()),
            "clean_map_yaml_sha256": sha256(CLEAN_YAML),
            "spawn_pose": {
                "s": float(spawn["s"]), "x": float(spawn["x_m"]),
                "y": float(spawn["y_m"]), "yaw": float(spawn["psi_rad"]),
            },
            "vehicle_length_m": 0.56,
            "vehicle_width_m": 0.287,
            "waypoint_file": str(WAYPOINTS_JSON.resolve()),
            "waypoint_sha256": sha256(WAYPOINTS_JSON),
            "simulator_config": str(Path(config["paths"]["simulator_config"])),
            "simulator_config_sha256": sha256(Path(config["paths"]["simulator_config"])),
            "simulator_collision_source": str(
                Path(config["paths"]["simulator_collision_source"])),
            "simulator_collision_source_sha256": sha256(
                Path(config["paths"]["simulator_collision_source"])),
            "simulator_collision_model": {
                "image_vertical_flip": True,
                "lidar_offset_x_m": 0.275,
                "occupied_gray_threshold": 128,
                "physics_timestep_sec": 0.01,
                "pixel_lookup": "floor_half_open_cell",
                "reference_point": "base_link_center",
                "scan_beams": 1080,
                "scan_fov_rad": 4.7,
                "scan_noise_guard_sigma": 0.75,
                "scan_noise_std_m": 0.01,
                "ttc_threshold_sec": 0.005,
                "vehicle_length_m": 0.56,
                "vehicle_width_m": 0.287,
            },
        })

    gate = {
        "schema": "competition_obstacle_rule_gate_v2/1",
        "task": "P3_HYBRID_P0_CMA_V2_SCENARIOS",
        "generator_seed": SEED,
        "rules": {
            "maximum_dimension_m": RULE_MAX_SIZE,
            "minimum_pairwise_polygon_distance_m": RULE_MIN_PAIRWISE_M,
            "minimum_free_track_width_m": RULE_MIN_FREE_WIDTH_M,
            "minimum_start_arc_distance_m": RULE_MIN_START_ARC_M,
        },
        "vehicle_feasibility": {
            "reason": (
                "the v1 frozen gate placed obstacle #1 in the full-lock curvature shadow "
                "(kappa 1.06 corner, vehicle limit 1.316); rule-legal but impassable for "
                "this vehicle, which made every v1 CMA episode safe-stop"),
            "max_abs_reference_kappa": FEASIBLE_MAX_ABS_KAPPA,
            "kappa_shadow_m": FEASIBLE_KAPPA_SHADOW_M,
            "minimum_free_side_gap_m": FEASIBLE_MIN_FREE_SIDE_M,
            "minimum_pairwise_arc_gap_m": FEASIBLE_MIN_ARC_GAP_M,
        },
        "authority": {
            "clean_map": str(CLEAN_YAML.relative_to(ROOT)),
            "clean_map_yaml_sha256": sha256(CLEAN_YAML),
            "clean_map_image_sha256": sha256(CLEAN_YAML.with_suffix(".png")),
            "waypoints": str(WAYPOINTS.relative_to(ROOT)),
            "waypoints_sha256": sha256(WAYPOINTS),
        },
        "obstacles": gate_obstacles,
        "configurations": configurations,
        "injection_freeze": injection_freeze,
        "trajectory_executed": False,
    }
    atomic_write_json(OUTPUT / "rule_gate_v2.json", gate)
    for manifest in manifests:
        manifest["competition_rule_gate_sha256"] = sha256(OUTPUT / "rule_gate_v2.json")
        atomic_write_json(
            OUTPUT / f"{manifest['competition_configuration_class'].lower()}_manifest.json",
            manifest)
    print(json.dumps({
        "output": str(OUTPUT),
        "classes": {
            c["configuration_class"]: {
                "obstacles": [
                    {
                        "s": round(o["obstacle"]["s"], 3),
                        "d": round(o["obstacle"]["d"], 3),
                        "long": round(o["obstacle"]["width"], 3),
                        "lat": round(o["obstacle"]["height"], 3),
                        "free_gap": round(o["rules"]["free_side_gap_m"], 3),
                    }
                    for o in gate_obstacles
                    if o["case_id"] in c["ordered_obstacle_ids"]
                ],
                "min_pairwise_m": round(c["minimum_pairwise_polygon_distance_m"], 3),
            } for c in configurations
        },
    }, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
