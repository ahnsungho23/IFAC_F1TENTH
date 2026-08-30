#!/usr/bin/env python3
"""Read-only reconstruction for P3 direct mapping-miss diagnosis v1.

This script only reads frozen v2 inputs/results.  It ports the exact P3 point-equation,
branch tests, corridor construction, and quintic-Hermite profile formulas from the frozen
production source.  It never writes planner source/configuration or changes runtime policy.
"""

from __future__ import annotations

import csv
import json
import math
import os
import pathlib
import subprocess
import tempfile
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[1]
ORACLE = REPO / "planning_study/p3_oracle_pilot_v2"
OUT = HERE
PLOTS = OUT / "plots"
PATHS = OUT / "paths"
HARNESS = pathlib.Path("/tmp/p3_oracle_v2_build/p3_family_oracle_harness")

EPS = 1.0e-9
ROOT_RESIDUAL_TOL = 1.0e-12
VEHICLE_HALF_WIDTH = 0.15
SAFETY_MARGIN = 0.08
WALL_SAFETY_MARGIN = 0.04
MAX_TARGET = 1.5


def rows(path: pathlib.Path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def f(row, key):
    value = row.get(key, "")
    return float(value) if value not in ("", None) else math.nan


def is_true(value):
    return str(value).strip().lower() in {"1", "true", "yes"}


def close(a, b, tol=1.0e-9):
    return math.isfinite(a) and math.isfinite(b) and abs(a - b) <= tol


def write_csv(path, data, fieldnames=None):
    data = list(data)
    if fieldnames is None:
        fieldnames = []
        for row in data:
            for key in row:
                if key not in fieldnames:
                    fieldnames.append(key)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(data)


def parse_event(path):
    event = {"reference": [], "obstacles": []}
    lines = path.read_text().splitlines()
    assert lines[0] == "P3_ORACLE_EVENT_V1"
    for line in lines[1:]:
        t = line.split("\t")
        if t[0] == "EVENT":
            event.update(
                event_id=t[1], bag=t[2], callback_sequence=int(t[3]),
                evaluation_sequence=int(t[4]), evaluation_role=t[5],
                ego_s=float(t[6]), ego_d=float(t[7]), ego_speed=float(t[8]),
                source_stamp_ns=int(t[9]), source_epoch=int(t[10]))
        elif t[0] == "W":
            event["reference"].append({
                "id": int(t[1]), "s": float(t[2]), "d": float(t[3]),
                "x": float(t[4]), "y": float(t[5]), "d_right": float(t[6]),
                "d_left": float(t[7]), "psi": float(t[8]), "kappa": float(t[9]),
                "vx": float(t[10]), "ax": float(t[11]),
            })
        elif t[0] == "O":
            event["obstacles"].append({
                "id": int(t[1]), "s_center": float(t[2]), "s_start": float(t[3]),
                "s_end": float(t[4]), "d_right": float(t[5]), "d_left": float(t[6]),
                "size": float(t[7]), "s_var": float(t[8]), "d_var": float(t[9]),
                "is_static": bool(int(t[10])), "is_visible": bool(int(t[11])),
            })
        elif t[0] == "END_EVENT":
            break
    ds = sorted(
        event["reference"][i]["s"] - event["reference"][i - 1]["s"]
        for i in range(1, len(event["reference"])))
    event["track_length"] = event["reference"][-1]["s"] + ds[len(ds) // 2]
    return event


def wrap(event, s):
    return s % event["track_length"]


def forward(event, from_s, to_s):
    return wrap(event, to_s - from_s)


def next_ref_index(event, s):
    s = wrap(event, s)
    ref = event["reference"]
    lo, hi = 0, len(ref)
    while lo < hi:
        mid = (lo + hi) // 2
        if ref[mid]["s"] < s:
            lo = mid + 1
        else:
            hi = mid
    return 0 if lo == len(ref) else lo


def nearest_ref(event, s):
    nxt = next_ref_index(event, s)
    prev = len(event["reference"]) - 1 if nxt == 0 else nxt - 1
    def distance(index):
        ahead = forward(event, s, event["reference"][index]["s"])
        return min(ahead, event["track_length"] - ahead)
    return event["reference"][nxt if distance(nxt) < distance(prev) else prev]


def local_reference_points(event, end_forward):
    ref = event["reference"]
    first = next_ref_index(event, event["ego_s"])
    result = []
    for count in range(len(ref)):
        waypoint = ref[(first + count) % len(ref)]
        station = forward(event, event["ego_s"], waypoint["s"])
        if station > end_forward + EPS:
            break
        result.append((station, waypoint))
    return result


def expanded_visible(event):
    result = []
    for obstacle in event["obstacles"]:
        center = forward(event, event["ego_s"], obstacle["s_center"])
        span = min(
            forward(event, obstacle["s_start"], obstacle["s_end"]),
            forward(event, obstacle["s_end"], obstacle["s_start"]))
        if not span > EPS:
            span = max(0.05, abs(obstacle["size"]))
        half = 0.5 * span
        raw_right = min(obstacle["d_right"], obstacle["d_left"])
        raw_left = max(obstacle["d_right"], obstacle["d_left"])
        item = {
            "id": obstacle["id"], "start": center - half, "end": center + half,
            "center": center, "raw_d_right": raw_right, "raw_d_left": raw_left,
            "d_right": raw_right - (VEHICLE_HALF_WIDTH + SAFETY_MARGIN),
            "d_left": raw_left + (VEHICLE_HALF_WIDTH + SAFETY_MARGIN),
        }
        if item["end"] >= 0.0 and item["start"] <= 15.0:
            result.append(item)
    return sorted(result, key=lambda x: x["start"])


def nearest_cluster(visible):
    first = next((i for i, obstacle in enumerate(visible)
                  if obstacle["d_right"] <= 0.0 <= obstacle["d_left"]), None)
    if first is None:
        return []
    cluster = [visible[first]]
    end = visible[first]["end"]
    for obstacle in visible[first + 1:]:
        if obstacle["start"] > end + 0.8:
            break
        cluster.append(obstacle)
        end = max(end, obstacle["end"])
    return cluster


def subtract(intervals, forbidden):
    result = []
    for lower, upper in intervals:
        if forbidden[1] <= lower + EPS or forbidden[0] >= upper - EPS:
            result.append((lower, upper))
            continue
        if forbidden[0] > lower + EPS:
            result.append((lower, min(upper, forbidden[0])))
        if forbidden[1] < upper - EPS:
            result.append((max(lower, forbidden[1]), upper))
    return result


def unique_sorted(values):
    result = []
    for value in sorted(values):
        if not result or abs(value - result[-1]) > 1.0e-12:
            result.append(value)
    return result


def sample_at(event, visible, station, left):
    ref = nearest_ref(event, wrap(event, event["ego_s"] + station))
    left_width = ref["d_left"] if ref["d_left"] > 0.05 else 1.5
    right_width = ref["d_right"] if ref["d_right"] > 0.05 else 1.5
    track = (-right_width + VEHICLE_HALF_WIDTH + WALL_SAFETY_MARGIN,
             left_width - VEHICLE_HALF_WIDTH - WALL_SAFETY_MARGIN)
    feasible = [track] if track[1] > track[0] + EPS else []
    active = []
    for obstacle in visible:
        if station + EPS < obstacle["start"] or station - EPS > obstacle["end"]:
            continue
        active.append(obstacle["id"])
        feasible = subtract(feasible, (obstacle["d_right"], obstacle["d_left"]))
    chosen = feasible[-1] if left and feasible else (feasible[0] if feasible else (math.nan, math.nan))
    return {
        "station": station, "track_lower": track[0], "track_upper": track[1],
        "feasible": feasible, "lower": chosen[0], "upper": chosen[1],
        "center": 0.5 * (chosen[0] + chosen[1]), "width": chosen[1] - chosen[0],
        "curvature": ref["kappa"], "active_obstacles": active,
    }


def make_corridor(event, left, outside_is_left=False):
    visible = expanded_visible(event)
    cluster = nearest_cluster(visible)
    start = min(x["start"] for x in cluster)
    end = max(x["end"] for x in cluster)
    outside_multiplier = 0.4060036444074003 if left == outside_is_left else 1.0
    horizon = end + 6.178529850015357 * 3.698773101198193 * outside_multiplier
    stations = [0.0, start, end, 0.5 * (start + end)]
    for obstacle in visible:
        if obstacle["end"] < -EPS or obstacle["start"] > horizon + EPS:
            continue
        stations.extend([
            max(0.0, obstacle["start"]), min(max(obstacle["center"], 0.0), horizon),
            min(horizon, obstacle["end"]),
        ])
    for station, _ in local_reference_points(event, horizon):
        stations.append(station)
    samples = [sample_at(event, visible, s, left) for s in unique_sorted(stations)]
    return {"visible": visible, "cluster": cluster, "start": start, "end": end,
            "samples": samples, "left": left}


def span_samples(corridor):
    span = [x for x in corridor["samples"]
            if x["station"] > corridor["start"] + EPS
            and x["station"] < corridor["end"] - EPS]
    if span:
        return span
    midpoint = 0.5 * (corridor["start"] + corridor["end"])
    return [min(corridor["samples"], key=lambda x: abs(x["station"] - midpoint))]


def choose_probe(corridor, target, policy):
    span = span_samples(corridor)
    if policy == "CURVATURE_CONTINUITY":
        selected = max(span, key=lambda x: (abs(x["curvature"]), x["station"]))
        lower, upper = selected["lower"] + SAFETY_MARGIN, selected["upper"] - SAFETY_MARGIN
        desired = min(max(target, lower), upper) if lower <= upper else selected["center"]
        return selected, desired
    if policy == "BOTTLENECK_CENTER":
        selected = min(span, key=lambda x: (x["width"], x["station"]))
        return selected, selected["center"]
    raise RuntimeError(policy)


def connected_ranges(corridor, domain_low, domain_high):
    active = None
    for sample in corridor["samples"]:
        if sample["station"] < corridor["start"] - EPS or sample["station"] > corridor["end"] + EPS:
            continue
        if active is None:
            active = list(sample["feasible"])
            continue
        nxt = []
        for previous in active:
            for current in sample["feasible"]:
                intersection = (max(previous[0], current[0]), min(previous[1], current[1]))
                if intersection[1] >= intersection[0] - EPS:
                    nxt.append(intersection)
        active = sorted(set(nxt))
        if not active:
            break
    ranges = []
    for interval in active or []:
        lower, upper = max(interval[0], domain_low), min(interval[1], domain_high)
        if upper >= lower - EPS:
            ranges.append((lower, upper))
    if not ranges:
        ranges = [(domain_low, domain_high)]
    ranges.sort(key=lambda x: (-(x[1] - x[0]), x[0], x[1]))
    return ranges[:2]


def source_branch(left, right):
    if left == 0.0 and right == 0.0:
        return "ZERO_BOTH"
    if left == 0.0:
        return "ZERO_LEFT"
    if right == 0.0:
        return "ZERO_RIGHT"
    if left > 0.0 and right > 0.0:
        return "SAME_SIGN_POSITIVE"
    if left < 0.0 and right < 0.0:
        return "SAME_SIGN_NEGATIVE"
    return "SIGN_CHANGE"


def harmonic(h_left, h_right, left, right):
    return (h_left + h_right) / (h_left / left + h_right / right) if left * right > 0.0 else 0.0


def knot_states(stations, offsets):
    secants = [(offsets[i + 1] - offsets[i]) / (stations[i + 1] - stations[i]) for i in range(4)]
    derivatives = [0.0] * 5
    accelerations = [0.0] * 5
    branches = []
    for i in range(1, 4):
        hl, hr = stations[i] - stations[i - 1], stations[i + 1] - stations[i]
        branches.append(source_branch(secants[i - 1], secants[i]))
        derivatives[i] = harmonic(hl, hr, secants[i - 1], secants[i])
        accelerations[i] = 2.0 * (secants[i] - secants[i - 1]) / (hl + hr)
    return secants, derivatives, accelerations, branches


def quintic(start, end, d0, v0, a0, d1, v1, a1):
    h = end - start
    if not h > EPS:
        raise RuntimeError("non-positive quintic-Hermite segment")
    c = [d0, h * v0, 0.5 * h * h * a0, 0.0, 0.0, 0.0]
    r0 = d1 - c[0] - c[1] - c[2]
    r1 = h * v1 - c[1] - 2.0 * c[2]
    r2 = h * h * a1 - 2.0 * c[2]
    c[3] = 10.0 * r0 - 4.0 * r1 + 0.5 * r2
    c[4] = -15.0 * r0 + 7.0 * r1 - r2
    c[5] = 6.0 * r0 - 3.0 * r1 + 0.5 * r2
    return start, end, c


def make_profile(stations, offsets, zero_derivatives=False):
    _, derivatives, accelerations, _ = knot_states(stations, offsets)
    if zero_derivatives:
        derivatives = [0.0] * 5
    return [quintic(stations[i], stations[i + 1], offsets[i], derivatives[i], accelerations[i],
                    offsets[i + 1], derivatives[i + 1], accelerations[i + 1]) for i in range(4)]


def eval_segment(segment, station):
    start, end, c = segment
    t = min(1.0, max(0.0, (station - start) / (end - start)))
    value = c[5]
    for i in range(4, -1, -1):
        value = value * t + c[i]
    return value


def eval_profile(stations, ego_d, target, middle, station):
    segments = make_profile(stations, [ego_d, target, middle, target, 0.0])
    if station <= segments[0][0]:
        return ego_d
    for segment in segments:
        if station <= segment[1]:
            return eval_segment(segment, station)
    return 0.0


def linear_without_side(stations, ego_d, target, middle, segment, t):
    profile = make_profile(stations, [ego_d, target, middle, target, 0.0], True)
    station = stations[segment] + t * (stations[segment + 1] - stations[segment])
    return eval_segment(profile[segment], station)


def derivative_weight(stations, segment, t):
    h = stations[segment + 1] - stations[segment]
    t2, t3, t4, t5 = t * t, t ** 3, t ** 4, t ** 5
    return h * ((-4.0 * t3 + 7.0 * t4 - 3.0 * t5)
                if segment in (0, 2) else (t - 6.0 * t3 + 8.0 * t4 - 3.0 * t5))


def equation_details(stations, ego_d, target, station, desired, lower, upper):
    segment = 0
    while segment + 1 < 4 and station > stations[segment + 1]:
        segment += 1
    h = stations[segment + 1] - stations[segment]
    t = min(1.0, max(0.0, (station - stations[segment]) / h))
    assumed = (["SAME_SIGN_POSITIVE", "SIGN_CHANGE", "SAME_SIGN_NEGATIVE"]
               if target > 0.0 else
               ["SAME_SIGN_NEGATIVE", "SIGN_CHANGE", "SAME_SIGN_POSITIVE"])
    entry = segment <= 1
    if entry:
        hl, hr = stations[1] - stations[0], stations[2] - stations[1]
        fixed_slope = (target - ego_d) / hl
        k = (hl + hr) * fixed_slope
        a, b, c, e = k, -k * target, hl, hr * hr * fixed_slope - hl * target
        active_map = "ENTRY"
    else:
        hl, hr = stations[3] - stations[2], stations[4] - stations[3]
        fixed_slope = (0.0 - target) / hr
        k = (hl + hr) * fixed_slope
        a, b, c, e = -k, k * target, -hr, hl * hl * fixed_slope + hr * target
        active_map = "EXIT"
    intercept = linear_without_side(stations, ego_d, target, 0.0, segment, t)
    slope = linear_without_side(stations, ego_d, target, 1.0, segment, t) - intercept
    weight = derivative_weight(stations, segment, t)
    shifted = intercept - desired
    polynomial = (slope * c,
                  slope * e + shifted * c + weight * a,
                  shifted * e + weight * b)
    aa, bb, cc = polynomial
    scale = max(abs(aa), abs(bb), abs(cc), float.fromhex("0x0.0000000000001p-1022"))
    na, nb, nc = aa / scale, bb / scale, cc / scale
    raw = []
    if aa == 0.0:
        equation_class = "CONSTANT_ALL_ROOTS" if bb == 0.0 and cc == 0.0 else (
            "CONSTANT_NO_ROOT" if bb == 0.0 else "LINEAR_ONE_ROOT")
        if bb != 0.0:
            raw = [-nc / nb]
    else:
        disc = nb * nb - 4.0 * na * nc
        tol = 256.0 * np.finfo(float).eps * max(abs(nb * nb) + abs(4.0 * na * nc), np.finfo(float).tiny)
        if disc < 0.0 and disc >= -tol:
            disc = 0.0
        if disc < 0.0:
            equation_class = "QUADRATIC_NO_REAL_ROOT"
        elif math.sqrt(disc) == 0.0:
            raw = [-0.5 * nb / na]
            equation_class = "QUADRATIC_DOUBLE_ROOT"
        else:
            q = -0.5 * (nb + math.copysign(math.sqrt(disc), nb))
            raw = sorted([q / na, nc / q])
            if abs(raw[0] - raw[1]) <= 128.0 * np.finfo(float).eps * max(1.0, abs(raw[0]), abs(raw[1])):
                raw = raw[:1]
            equation_class = "QUADRATIC_TWO_ROOTS" if len(raw) == 2 else "QUADRATIC_NUMERICALLY_DUPLICATE_ROOT"
    roots = []
    for root in raw:
        finite = math.isfinite(root)
        branch = knot_states(stations, [ego_d, target, root, target, 0.0])[3] if finite else []
        branch_ok = finite and branch == assumed
        bounded = branch_ok and root >= lower - EPS and root <= upper + EPS
        residual = eval_profile(stations, ego_d, target, root, station) - desired if finite else math.nan
        accepted = bounded and abs(residual) <= ROOT_RESIDUAL_TOL * max(1.0, abs(desired))
        status = ("NONFINITE_ROOT" if not finite else
                  "BRANCH_MISMATCH" if not branch_ok else
                  "LATERAL_BOUND" if not bounded else
                  "FORWARD_RESIDUAL" if not accepted else "ACCEPTED")
        roots.append({"root": root, "finite": finite, "branch_regime": "__".join(branch),
                      "branch_ok": branch_ok, "bounded": bounded, "forward_residual": residual,
                      "accepted": accepted, "root_filter_status": status})
    return {
        "segment_index": segment, "normalized_t": t, "active_map": active_map,
        "assumed_branch_regime": "__".join(assumed), "fractional_a": a,
        "fractional_b": b, "fractional_c": c, "fractional_e": e,
        "affine_intercept": intercept, "affine_slope": slope,
        "derivative_weight": weight, "poly_a": aa, "poly_b": bb, "poly_c": cc,
        "equation_class": equation_class, "roots": roots,
    }


def inactive_equation_details(stations, ego_d, target, station, desired, lower, upper):
    """Exact solveAllInactive() linear counterpart used by M1."""
    segment = 0
    while segment + 1 < 4 and station > stations[segment + 1]:
        segment += 1
    length = stations[segment + 1] - stations[segment]
    normalized = min(1.0, max(0.0, (station - stations[segment]) / length))
    intercept = linear_without_side(stations, ego_d, target, 0.0, segment, normalized)
    slope = linear_without_side(stations, ego_d, target, 1.0, segment, normalized) - intercept
    coefficient_scale = max(1.0, abs(intercept), abs(slope), abs(desired))
    zero_tolerance = 128.0 * np.finfo(float).eps * coefficient_scale
    if abs(slope) <= zero_tolerance:
        return {"equation_class": "LINEAR_DEGENERATE", "roots": [],
                "intercept": intercept, "slope": slope}
    root = (desired - intercept) / slope
    finite = math.isfinite(root)
    branches = knot_states(stations, [ego_d, target, root, target, 0.0])[3] if finite else []
    branch_ok = finite and not any(branch.startswith("SAME_SIGN") for branch in branches)
    bounded = branch_ok and lower - EPS <= root <= upper + EPS
    residual = eval_profile(stations, ego_d, target, root, station) - desired if finite else math.nan
    accepted = bounded and abs(residual) <= ROOT_RESIDUAL_TOL * max(1.0, abs(desired))
    status = ("NONFINITE_ROOT" if not finite else "ALL_INACTIVE_BRANCH_MISMATCH" if not branch_ok
              else "LATERAL_BOUND" if not bounded else "FORWARD_RESIDUAL" if not accepted else "ACCEPTED")
    return {"equation_class": "LINEAR_ONE_ROOT", "intercept": intercept, "slope": slope,
            "roots": [{"root": root, "finite": finite, "branch_regime": "__".join(branches),
                       "branch_ok": branch_ok, "bounded": bounded,
                       "forward_residual": residual, "accepted": accepted,
                       "root_filter_status": status}]}


def forced_anchor(event, left, station, target, policy):
    sample = sample_at(event, expanded_visible(event), station, left)
    if policy == "CURVATURE_CONTINUITY":
        lower, upper = sample["lower"] + SAFETY_MARGIN, sample["upper"] - SAFETY_MARGIN
        desired = min(max(target, lower), upper) if lower <= upper else sample["center"]
    else:
        desired = sample["center"]
    return sample, desired


def domain_for(search_domain, event_id, side):
    values = search_domain[event_id]["target_domains"]["STRICT_" + side]
    return min(values), max(values)


def root_bounds(event, prod, corridor, search_domain):
    target = f(prod, "d_target")
    if prod["generator_stage"] == "M0_V1":
        midpoint = 0.5 * (corridor["start"] + corridor["end"])
        sample = min(corridor["samples"], key=lambda x: abs(x["station"] - midpoint))
        lower, upper = max(-MAX_TARGET, sample["lower"]), min(MAX_TARGET, sample["upper"])
        if prod["side"] == "LEFT":
            lower = max(lower, target)
        else:
            upper = min(upper, target)
        return lower, upper, "M0_V1_MID_INTERVAL_PLUS_SIDE_MONOTONICITY"
    low, high = domain_for(search_domain, prod["event_id"], prod["side"])
    components = connected_ranges(corridor, low, high)
    containing = [x for x in components if x[0] - EPS <= f(prod, "d_mid") <= x[1] + EPS]
    chosen = containing[0] if containing else min(components, key=lambda x: min(abs(target-x[0]), abs(target-x[1])))
    return chosen[0], chosen[1], "M1_CONNECTED_CONSTANT_COMPONENT"


def path_points(event, target, middle, stations):
    path_end = stations[-1] + max(5.0, abs(event["ego_speed"]) * 1.0)
    result = []
    for station, ref in local_reference_points(event, path_end):
        d = eval_profile(stations, event["ego_d"], target, middle, station)
        result.append({"station": station, "s_m": ref["s"], "d_m": d,
                       "x_m": ref["x"] - d * math.sin(ref["psi"]),
                       "y_m": ref["y"] + d * math.cos(ref["psi"])})
    return result


def exact_harness_path(event_id, representative, output):
    if not HARNESS.exists():
        return False
    with tempfile.TemporaryDirectory(prefix="p3_mapping_path_", dir="/tmp") as directory:
        request = pathlib.Path(directory) / "one.requests"
        raw_path = pathlib.Path(directory) / "path.tsv"
        request.write_text(
            "P3_ORACLE_REQUESTS_V1\nQ\tDIAGNOSIS\t{}\t{}\t{:.17g}\t{:.17g}\t{:.17g}\t{:.17g}\n".format(
                1 if representative["gate"] == "RELAXED" else 0,
                1 if representative["side"] == "LEFT" else 0,
                representative["d_target"], representative["d_mid"],
                representative["entry_scale"], representative["exit_scale"]))
        process = subprocess.run(
            [str(HARNESS), str(ORACLE / "inputs" / f"{event_id}.event"),
             str(request), str(raw_path)], text=True, capture_output=True, check=True)
        candidate_line = next(line for line in process.stdout.splitlines() if line.startswith("CANDIDATE\t" + event_id + "\t0\t"))
        if "\t1\t1\t" not in candidate_line:
            raise RuntimeError(f"representative did not reconstruct hard-valid: {event_id}")
        with raw_path.open(newline="") as stream:
            reader = csv.DictReader(stream, delimiter="\t")
            data = list(reader)
        write_csv(output, data)
    return True


def reference_at_forward(event, station):
    target = wrap(event, event["ego_s"] + station)
    nxt = next_ref_index(event, target)
    prev = len(event["reference"]) - 1 if nxt == 0 else nxt - 1
    a, b = event["reference"][prev], event["reference"][nxt]
    span = forward(event, a["s"], b["s"])
    ratio = 0.0 if span <= EPS else forward(event, a["s"], target) / span
    ratio = min(1.0, max(0.0, ratio))
    delta = math.atan2(math.sin(b["psi"] - a["psi"]), math.cos(b["psi"] - a["psi"]))
    return {key: a[key] + ratio * (b[key] - a[key]) for key in ("x", "y", "d_left", "d_right")} | {
        "psi": a["psi"] + ratio * delta}


def xy_at(event, station, d):
    ref = reference_at_forward(event, station)
    return ref["x"] - d * math.sin(ref["psi"]), ref["y"] + d * math.cos(ref["psi"])


def main():
    PLOTS.mkdir(exist_ok=True)
    PATHS.mkdir(exist_ok=True)
    production = rows(ORACLE / "production_candidates.csv")
    valid = rows(ORACLE / "oracle_valid_candidates.csv")
    summary = rows(ORACLE / "oracle_summary.csv")
    search_domain = json.loads((ORACLE / "search_domain.json").read_text())
    events = {event_id: parse_event(ORACLE / "inputs" / f"{event_id}.event")
              for event_id in [f"V2E{i:02d}" for i in range(1, 10)]}

    direct_ids = []
    for row in summary:
        if (row["classification"] == "ORACLE_VALID_P3_EXISTS"
                and is_true(row["any_valid_same_target_scales_but_missed_d_mid"])):
            direct_ids.append(row["event_id"])
    assert direct_ids == ["V2E01", "V2E02", "V2E08"], direct_ids

    case_rows = []
    for row in summary:
        event_id = row["event_id"]
        event = events[event_id]
        visible = expanded_visible(event)
        cluster = nearest_cluster(visible)
        role = ("DIRECT_MAPPING_MISS_PRIMARY" if event_id in direct_ids else
                "TEMPLATE_OR_TRANSITION_SELECTION" if event_id == "V2E03" else
                "NO_VALID_P3_IN_ORACLE_DOMAIN")
        if event_id == "V2E08":
            role = "DIRECT_MAPPING_MISS_LOCALIZATION_STRESS_SEPARATE"
        case_rows.append({
            "event_id": event_id, "bag": row["bag"], "elapsed_s": row["elapsed_s"],
            "callback_sequence": row["callback_sequence"],
            "evaluation_sequence": row["evaluation_sequence"],
            "development_data_label": "PILOT_SEEN_DEVELOPMENT_DATA",
            "localization_label": row["localization_label"],
            "provenance_flag": row["provenance_flag"], "analysis_role": role,
            "oracle_classification": row["classification"],
            "direct_same_target_scale_dmid_miss": event_id in direct_ids,
            "exact_lineage_and_parity": is_true(row["exact_input_lineage"]) and is_true(row["constructed_digest_multiset_parity"]) and is_true(row["returned_digest_multiset_parity"]),
            "production_constructed_count": row["production_constructed_count"],
            "production_hard_valid_count": row["production_hard_valid_count"],
            "oracle_hard_valid_count": row["oracle_hard_valid_count"],
            "ego_s_m": row["ego_s_m"], "ego_d_m": row["ego_d_m"],
            "ego_speed_mps": row["ego_speed_mps"],
            "cluster_start_forward_m": min((x["start"] for x in cluster), default=math.nan),
            "cluster_end_forward_m": max((x["end"] for x in cluster), default=math.nan),
            "inflated_obstacle_geometry_json": json.dumps(visible, separators=(",", ":")),
        })
    write_csv(OUT / "case_summary.csv", case_rows)

    # Representative selection is data-derived.  Direct representatives are constrained to an
    # exact production side/target/entry/exit tuple.  Global safest is retained separately.
    representatives = []
    primary = {}
    for event_id in direct_ids:
        prod_analytic = [p for p in production if p["event_id"] == event_id and p["s_probe"]]
        candidates = []
        for oracle_row in valid:
            if oracle_row["event_id"] != event_id:
                continue
            matches = [p for p in prod_analytic
                       if p["side"] == oracle_row["side"]
                       and close(f(p, "d_target"), f(oracle_row, "d_target"))
                       and close(f(p, "entry_scale"), f(oracle_row, "entry_scale"))
                       and close(f(p, "exit_scale"), f(oracle_row, "exit_scale"))]
            if matches:
                nearest_prod = min(matches, key=lambda p: abs(f(p, "d_mid") - f(oracle_row, "d_mid")))
                candidates.append((oracle_row, nearest_prod,
                                   abs(f(nearest_prod, "d_mid") - f(oracle_row, "d_mid"))))
        nearest = min(candidates, key=lambda x: (x[2], -f(x[0], "minimum_normalized_safety_slack")))
        safest_direct = max(candidates, key=lambda x: f(x[0], "minimum_normalized_safety_slack"))
        safest_global = max((r for r in valid if r["event_id"] == event_id),
                            key=lambda r: f(r, "minimum_normalized_safety_slack"))
        primary[event_id] = nearest
        chosen = [
            ("DIRECT_NEAREST_D_MID", nearest[0], nearest[1]),
            ("DIRECT_LARGEST_HARD_SAFETY_MARGIN", safest_direct[0], safest_direct[1]),
            ("EVENT_GLOBAL_LARGEST_HARD_SAFETY_MARGIN", safest_global, None),
        ]
        # Observed grid support for the primary exact tuple; this is not claimed as a continuous proof.
        base = nearest[0]
        same_group = [r for r in valid if r["event_id"] == event_id and r["side"] == base["side"]
                      and close(f(r, "d_target"), f(base, "d_target"))
                      and close(f(r, "entry_scale"), f(base, "entry_scale"))
                      and close(f(r, "exit_scale"), f(base, "exit_scale"))]
        interval_min = min(same_group, key=lambda r: f(r, "d_mid"))
        interval_max = max(same_group, key=lambda r: f(r, "d_mid"))
        chosen.extend([
            ("DIRECT_OBSERVED_VALID_D_MID_MIN", interval_min, nearest[1]),
            ("DIRECT_OBSERVED_VALID_D_MID_MAX", interval_max, nearest[1]),
        ])
        seen = set()
        for role, row, matched in chosen:
            key = (role, row["gate"], row["side"], f(row, "d_target"), f(row, "d_mid"),
                   f(row, "entry_scale"), f(row, "exit_scale"))
            if key in seen:
                continue
            seen.add(key)
            path_file = PATHS / f"{event_id}_{role.lower()}.csv"
            item = {
                "event_id": event_id, "representative_role": role,
                "development_data_label": "PILOT_SEEN_DEVELOPMENT_DATA",
                "localization_stress_candidate": event_id == "V2E08",
                "diagnosis_primary": role == "DIRECT_NEAREST_D_MID",
                "gate": row["gate"], "phase": row["phase"], "side": row["side"],
                "d_target": f(row, "d_target"), "d_mid": f(row, "d_mid"),
                "entry_scale": f(row, "entry_scale"), "exit_scale": f(row, "exit_scale"),
                "z0": f(row, "z0"), "z1": f(row, "z1"), "z2": f(row, "z2"),
                "z3": f(row, "z3"), "z4": f(row, "z4"),
                "minimum_center_track_margin_m": f(row, "center_track_margin_m"),
                "minimum_footprint_track_margin_m": f(row, "footprint_track_margin_m"),
                "minimum_obstacle_margin_m": f(row, "obstacle_margin_m"),
                "lateral_slope_margin": f(row, "lateral_slope_margin"),
                "signed_curvature_margin_radpm": f(row, "signed_curvature_margin_radpm"),
                "curvature_rate_margin_radpm2": f(row, "curvature_rate_margin_radpm2"),
                "braking_deficit_m": f(row, "braking_deficit_m"),
                "minimum_normalized_safety_slack": f(row, "minimum_normalized_safety_slack"),
                "velocity_loss": f(row, "velocity_loss"),
                "global_path_deviation_m": f(row, "global_path_deviation_m"),
                "waypoint0_footprint_track_margin_m": f(row, "waypoint0_footprint_track_margin_m"),
                "path_digest": row["path_digest"], "path_file": str(path_file.relative_to(OUT)),
                "matched_production_stage": matched["generator_stage"] if matched else "",
                "matched_production_template": matched["template"] if matched else "",
                "matched_production_d_mid": f(matched, "d_mid") if matched else math.nan,
                "distance_to_matched_production_d_mid": abs(f(matched, "d_mid") - f(row, "d_mid")) if matched else math.nan,
                "observed_valid_d_mid_min_same_tuple": f(interval_min, "d_mid"),
                "observed_valid_d_mid_max_same_tuple": f(interval_max, "d_mid"),
                "observed_valid_count_same_tuple": len(same_group),
            }
            reconstructed = exact_harness_path(event_id, item, path_file)
            item["exact_cpp_path_reconstruction"] = reconstructed
            if matched:
                event = events[event_id]
                left = matched["side"] == "LEFT"
                corridor = make_corridor(event, left)
                lower, upper, _ = root_bounds(event, matched, corridor, search_domain)
                matched_stations = [f(matched, f"z{i}") for i in range(5)]
                s_probe = f(matched, "s_probe")
                d_probe = f(matched, "d_probe")
                required = eval_profile(
                    [item[f"z{i}"] for i in range(5)], event["ego_d"], item["d_target"],
                    item["d_mid"], s_probe)
                details = equation_details(
                    matched_stations, event["ego_d"], f(matched, "d_target"),
                    s_probe, d_probe, lower, upper)
                actual_branch = "__".join(knot_states(
                    matched_stations,
                    [event["ego_d"], f(matched, "d_target"), item["d_mid"],
                     f(matched, "d_target"), 0.0])[3])
                branch_ok = actual_branch == details["assumed_branch_regime"]
                bounds_ok = lower - EPS <= item["d_mid"] <= upper + EPS
                item.update({
                    "matched_production_s_probe": s_probe,
                    "matched_production_d_probe": d_probe,
                    "d_oracle_at_matched_production_s_probe": required,
                    "probe_residual_at_matched_production_pair_m": required - d_probe,
                    "oracle_d_mid_branch_pass_if_anchor_changed": branch_ok,
                    "oracle_d_mid_lateral_bound_pass_if_anchor_changed": bounds_ok,
                    "oracle_d_mid_would_be_accepted_root_if_only_anchor_changed": branch_ok and bounds_ok,
                })
            representatives.append(item)
    write_csv(OUT / "oracle_valid_representatives.csv", representatives)

    mapping_trace = []
    corridors = {}
    for event_id in direct_ids:
        event = events[event_id]
        for prod in [p for p in production if p["event_id"] == event_id]:
            base = {
                "event_id": event_id, "development_data_label": "PILOT_SEEN_DEVELOPMENT_DATA",
                "localization_stress_candidate": event_id == "V2E08",
                "mapping_source": prod["mapping_source"], "generator_stage": prod["generator_stage"],
                "template": prod["template"], "source_cell": prod["source_cell"],
                "side": prod["side"], "returned_by_policy": prod["returned_by_policy"],
                "discarded_side": prod["discarded_side"], "d_target": prod["d_target"],
                "entry_scale": prod["entry_scale"], "exit_scale": prod["exit_scale"],
                "constructed_d_mid": prod["d_mid"], "constructed_hard_valid": prod["hard_valid"],
                "constructed_first_failure": prod["first_failure_reason"],
                "path_digest": prod["path_digest"],
            }
            if not prod["s_probe"]:
                mapping_trace.append(base | {
                    "probe_policy": "NONE_ZERO_INTERFACE", "s_probe": "", "d_probe": "",
                    "s_probe_selection_reason": "ZERO_INTERFACE_TEMPLATE_HAS_NO_ANALYTIC_POINT_CONSTRAINT",
                    "d_probe_selection_reason": "ZERO_INTERFACE_TEMPLATE_SETS_D_MID_EQUAL_D_TARGET",
                    "root_ordinal": "", "raw_d_mid_root": prod["d_mid"],
                    "root_filter_status": "NOT_APPLICABLE_ZERO_INTERFACE",
                    "accepted_before_construction": True,
                })
                continue
            left = prod["side"] == "LEFT"
            corridor = corridors.setdefault((event_id, left), make_corridor(event, left))
            policy = "CURVATURE_CONTINUITY" if prod["generator_stage"] == "M0_V1" else "BOTTLENECK_CENTER"
            selected, desired = choose_probe(corridor, f(prod, "d_target"), policy)
            assert close(selected["station"], f(prod, "s_probe"), 2e-9), (event_id, prod["template"], selected["station"], prod["s_probe"])
            assert close(desired, f(prod, "d_probe"), 2e-9), (event_id, prod["template"], desired, prod["d_probe"])
            lower, upper, bound_rule = root_bounds(event, prod, corridor, search_domain)
            stations = [f(prod, f"z{i}") for i in range(5)]
            details = equation_details(stations, event["ego_d"], f(prod, "d_target"),
                                       f(prod, "s_probe"), f(prod, "d_probe"), lower, upper)
            accepted = [r for r in details["roots"] if r["accepted"]]
            assert any(close(r["root"], f(prod, "d_mid"), 5e-9) for r in accepted), (event_id, prod["d_mid"], accepted)
            for ordinal, root in enumerate(details["roots"]):
                constructed = close(root["root"], f(prod, "d_mid"), 5e-9) and root["accepted"]
                mapping_trace.append(base | {
                    "probe_policy": policy, "s_probe": prod["s_probe"], "d_probe": prod["d_probe"],
                    "s_probe_selection_reason": (
                        "MAX_ABS_REFERENCE_CURVATURE_IN_OPEN_CLUSTER_TIE_LATER_STATION"
                        if policy == "CURVATURE_CONTINUITY" else
                        "MIN_BRANCH_WIDTH_IN_OPEN_CLUSTER_TIE_EARLIER_STATION"),
                    "d_probe_selection_reason": (
                        "CLAMP_D_TARGET_TO_BRANCH_INSET_BY_SAFETY_MARGIN_0.08M_ELSE_CENTER"
                        if policy == "CURVATURE_CONTINUITY" else "SELECTED_BRANCH_INTERVAL_CENTER"),
                    "probe_branch_lower": selected["lower"], "probe_branch_upper": selected["upper"],
                    "probe_branch_center": selected["center"], "probe_branch_width": selected["width"],
                    "probe_reference_curvature_radpm": selected["curvature"],
                    "probe_active_obstacle_ids": json.dumps(selected["active_obstacles"]),
                    "root_lower_bound": lower, "root_upper_bound": upper, "root_bound_rule": bound_rule,
                    "equation_segment_index": details["segment_index"],
                    "equation_normalized_t": details["normalized_t"],
                    "equation_active_fractional_map": details["active_map"],
                    "equation_class": details["equation_class"],
                    "polynomial_quadratic": details["poly_a"],
                    "polynomial_linear": details["poly_b"],
                    "polynomial_constant": details["poly_c"],
                    "assumed_branch_regime": details["assumed_branch_regime"],
                    "root_ordinal": ordinal, "raw_d_mid_root": root["root"],
                    "actual_branch_regime": root["branch_regime"],
                    "root_finite": root["finite"], "root_branch_match": root["branch_ok"],
                    "root_within_lateral_bound": root["bounded"],
                    "root_forward_residual": root["forward_residual"],
                    "root_filter_status": root["root_filter_status"],
                    "accepted_before_construction": root["accepted"],
                    "constructed_from_this_root": constructed,
                    "validator_result_for_this_root": prod["first_failure_reason"] if constructed else "NOT_CONSTRUCTED",
                })
            if prod["generator_stage"] == "M1":
                inactive = inactive_equation_details(
                    stations, event["ego_d"], f(prod, "d_target"),
                    f(prod, "s_probe"), f(prod, "d_probe"), lower, upper)
                for ordinal, root in enumerate(inactive["roots"]):
                    matching_constructed = [p for p in production
                        if p["event_id"] == event_id and p["generator_stage"] == "M1"
                        and p["template"] == prod["template"] and p["source_cell"] == "ALL_INACTIVE"
                        and close(f(p, "d_target"), f(prod, "d_target"))
                        and close(f(p, "entry_scale"), f(prod, "entry_scale"))
                        and close(f(p, "exit_scale"), f(prod, "exit_scale"))
                        and close(f(p, "d_mid"), root["root"], 5e-9)]
                    mapping_trace.append(base | {
                        "source_cell": "ALL_INACTIVE",
                        "probe_policy": policy, "s_probe": prod["s_probe"], "d_probe": prod["d_probe"],
                        "s_probe_selection_reason": "MIN_BRANCH_WIDTH_IN_OPEN_CLUSTER_TIE_EARLIER_STATION",
                        "d_probe_selection_reason": "SELECTED_BRANCH_INTERVAL_CENTER",
                        "probe_branch_lower": selected["lower"], "probe_branch_upper": selected["upper"],
                        "probe_branch_center": selected["center"], "probe_branch_width": selected["width"],
                        "probe_reference_curvature_radpm": selected["curvature"],
                        "probe_active_obstacle_ids": json.dumps(selected["active_obstacles"]),
                        "root_lower_bound": lower, "root_upper_bound": upper, "root_bound_rule": bound_rule,
                        "equation_segment_index": details["segment_index"],
                        "equation_normalized_t": details["normalized_t"],
                        "equation_active_fractional_map": "ALL_INACTIVE_LINEAR",
                        "equation_class": inactive["equation_class"],
                        "polynomial_quadratic": 0.0,
                        "polynomial_linear": inactive["slope"],
                        "polynomial_constant": inactive["intercept"] - f(prod, "d_probe"),
                        "assumed_branch_regime": "NO_SAME_SIGN_BRANCH_AT_ANY_INTERIOR_KNOT",
                        "root_ordinal": ordinal, "raw_d_mid_root": root["root"],
                        "actual_branch_regime": root["branch_regime"],
                        "root_finite": root["finite"], "root_branch_match": root["branch_ok"],
                        "root_within_lateral_bound": root["bounded"],
                        "root_forward_residual": root["forward_residual"],
                        "root_filter_status": root["root_filter_status"],
                        "accepted_before_construction": root["accepted"],
                        "constructed_from_this_root": bool(matching_constructed),
                        "validator_result_for_this_root": (
                            matching_constructed[0]["first_failure_reason"] if matching_constructed
                            else "NOT_CONSTRUCTED"),
                    })
    write_csv(OUT / "production_mapping_trace.csv", mapping_trace)

    compare_rows, residual_rows, filter_rows, scan_rows = [], [], [], []
    event_classification = {}
    for event_id in direct_ids:
        event = events[event_id]
        oracle_row, nearest_prod, _ = primary[event_id]
        oracle_tuple = {
            "event_id": event_id, "gate": oracle_row["gate"], "side": oracle_row["side"],
            "d_target": f(oracle_row, "d_target"), "d_mid": f(oracle_row, "d_mid"),
            "entry_scale": f(oracle_row, "entry_scale"), "exit_scale": f(oracle_row, "exit_scale"),
            "stations": [f(oracle_row, f"z{i}") for i in range(5)],
        }
        relevant = [p for p in production if p["event_id"] == event_id and p["s_probe"]
                    and p["side"] == oracle_row["side"]
                    and close(f(p, "d_target"), oracle_tuple["d_target"])
                    and close(f(p, "entry_scale"), oracle_tuple["entry_scale"])
                    and close(f(p, "exit_scale"), oracle_tuple["exit_scale"])]
        # Deduplicate identical policy/probe/root equations.
        unique = {}
        for prod in relevant:
            key = (prod["generator_stage"], prod["template"], prod["s_probe"], prod["d_probe"], prod["d_mid"])
            unique.setdefault(key, prod)
        relevant = list(unique.values())
        has_filter_block = False
        all_scan_min = []
        for prod in relevant:
            left = prod["side"] == "LEFT"
            corridor = corridors.setdefault((event_id, left), make_corridor(event, left))
            lower, upper, bound_rule = root_bounds(event, prod, corridor, search_domain)
            policy = "CURVATURE_CONTINUITY" if prod["generator_stage"] == "M0_V1" else "BOTTLENECK_CENTER"
            s_probe, d_probe = f(prod, "s_probe"), f(prod, "d_probe")
            d_required = eval_profile(oracle_tuple["stations"], event["ego_d"],
                                      oracle_tuple["d_target"], oracle_tuple["d_mid"], s_probe)
            point_residual = d_required - d_probe
            details = equation_details(
                [f(prod, f"z{i}") for i in range(5)], event["ego_d"], f(prod, "d_target"),
                s_probe, d_probe, lower, upper)
            oracle_branches = knot_states(
                [f(prod, f"z{i}") for i in range(5)],
                [event["ego_d"], f(prod, "d_target"), oracle_tuple["d_mid"], f(prod, "d_target"), 0.0])[3]
            branch_ok = "__".join(oracle_branches) == details["assumed_branch_regime"]
            bounds_ok = oracle_tuple["d_mid"] >= lower - EPS and oracle_tuple["d_mid"] <= upper + EPS
            polynomial_residual = ((details["poly_a"] * oracle_tuple["d_mid"]
                                    + details["poly_b"]) * oracle_tuple["d_mid"]
                                   + details["poly_c"])
            is_current_root = abs(point_residual) <= ROOT_RESIDUAL_TOL * max(1.0, abs(d_probe))
            if is_current_root and branch_ok and bounds_ok:
                classification, secondary = "D_OTHER", "ROOT_EXISTS_BUT_NOT_PRESENT_IN_SAVED_CONSTRUCTED_SET"
            elif not branch_ok or not bounds_ok:
                classification = "B_ROOT_SOLVER_OR_BRANCH_FILTER_MISS"
                secondary = "CURRENT_POINT_CONSTRAINT_ALSO_EXCLUDES_D_MID" if not is_current_root else "NONE"
                has_filter_block = True
            else:
                classification, secondary = "A_PROBE_CONSTRAINT_MISS", "NONE"
            common = {
                "event_id": event_id, "development_data_label": "PILOT_SEEN_DEVELOPMENT_DATA",
                "localization_stress_candidate": event_id == "V2E08",
                "production_stage": prod["generator_stage"], "production_template": prod["template"],
                "probe_policy": policy, "side": prod["side"],
                "d_target": prod["d_target"], "entry_scale": prod["entry_scale"],
                "exit_scale": prod["exit_scale"], "production_d_mid": prod["d_mid"],
                "oracle_d_mid": oracle_tuple["d_mid"], "s_probe_prod": s_probe,
                "d_probe_prod": d_probe, "d_oracle_at_s_probe_prod": d_required,
                "probe_residual_m": point_residual, "d_probe_required": d_required,
                "delta_d_probe_required_minus_production_m": point_residual,
            }
            residual_rows.append(common)
            filter_rows.append(common | {
                "analytic_polynomial_residual_at_oracle_d_mid": polynomial_residual,
                "current_point_equation_root": is_current_root,
                "oracle_branch_regime": "__".join(oracle_branches),
                "assumed_branch_regime": details["assumed_branch_regime"],
                "branch_filter_pass_if_anchor_changed": branch_ok,
                "root_lower_bound": lower, "root_upper_bound": upper,
                "lateral_bound_pass_if_anchor_changed": bounds_ok,
                "would_be_accepted_root_if_only_d_probe_changed": branch_ok and bounds_ok,
                "diagnostic_class": classification, "secondary_factor": secondary,
            })
            compare_rows.append(common | {
                "production_first_failure": prod["first_failure_reason"],
                "oracle_minimum_normalized_safety_slack": f(oracle_row, "minimum_normalized_safety_slack"),
                "oracle_footprint_track_margin_m": f(oracle_row, "footprint_track_margin_m"),
                "oracle_obstacle_margin_m": f(oracle_row, "obstacle_margin_m"),
                "oracle_lateral_slope_margin": f(oracle_row, "lateral_slope_margin"),
                "oracle_signed_curvature_margin_radpm": f(oracle_row, "signed_curvature_margin_radpm"),
                "oracle_curvature_rate_margin_radpm2": f(oracle_row, "curvature_rate_margin_radpm2"),
                "diagnostic_class": classification,
            })

            start = max(0.0, corridor["start"]) + 1.0e-8
            end = corridor["end"] - 1.0e-8
            grid = np.linspace(start, end, 501)
            local_scan = []
            existing_stations = [x["station"] for x in span_samples(corridor)]
            for station in grid:
                sample, anchor = forced_anchor(event, left, float(station), oracle_tuple["d_target"], policy)
                oracle_d = eval_profile(oracle_tuple["stations"], event["ego_d"],
                                        oracle_tuple["d_target"], oracle_tuple["d_mid"], float(station))
                local_scan.append({
                    "event_id": event_id, "production_stage": prod["generator_stage"],
                    "production_template": prod["template"], "probe_policy": policy,
                    "station_forward_m": float(station), "current_policy_d_probe": anchor,
                    "oracle_d_at_station": oracle_d, "residual_m": oracle_d - anchor,
                    "branch_lower": sample["lower"], "branch_upper": sample["upper"],
                    "branch_center": sample["center"],
                    "is_production_s_probe": abs(float(station) - s_probe) <= (end-start)/500.0/2.0,
                    "is_existing_corridor_sample": any(abs(float(station)-s) <= (end-start)/500.0/2.0 for s in existing_stations),
                })
            for i, row in enumerate(local_scan):
                row["zero_crossing_after_this_row"] = (i + 1 < len(local_scan)
                    and row["residual_m"] * local_scan[i + 1]["residual_m"] < 0.0)
                previous_abs = abs(local_scan[i - 1]["residual_m"]) if i > 0 else math.inf
                next_abs = abs(local_scan[i + 1]["residual_m"]) if i + 1 < len(local_scan) else math.inf
                row["is_local_abs_residual_minimum"] = (
                    abs(row["residual_m"]) <= previous_abs and abs(row["residual_m"]) <= next_abs)
                row["is_near_zero_1e_4m"] = abs(row["residual_m"]) <= 1.0e-4
            minimum = min(local_scan, key=lambda x: abs(x["residual_m"]))
            for row in local_scan:
                row["is_min_abs_residual"] = row is minimum
                row["min_abs_residual_m_for_mapping"] = abs(minimum["residual_m"])
                row["station_at_min_abs_residual_m"] = minimum["station_forward_m"]
            all_scan_min.append(abs(minimum["residual_m"]))
            scan_rows.extend(local_scan)
        event_classification[event_id] = (
            "ROOT_FILTERING_DOMINANT" if has_filter_block else
            "D_PROBE_DOMINANT" if all(value > 1.0e-4 for value in all_scan_min) else
            "S_PROBE_DOMINANT")

    # V2E03: exact target/mid exists in oracle only with another already-existing scale pairing.
    e03_prod = [p for p in production if p["event_id"] == "V2E03" and p["template"] == "ZERO_BOUNDARY_SHORT"][0]
    e03_valid = [r for r in valid if r["event_id"] == "V2E03"
                 and close(f(r, "d_target"), f(e03_prod, "d_target"), 1e-8)
                 and close(f(r, "d_mid"), f(e03_prod, "d_mid"), 1e-8)]
    e03_best = max(e03_valid, key=lambda r: f(r, "minimum_normalized_safety_slack"))
    compare_rows.append({
        "event_id": "V2E03", "development_data_label": "PILOT_SEEN_DEVELOPMENT_DATA",
        "localization_stress_candidate": False, "production_stage": e03_prod["generator_stage"],
        "production_template": e03_prod["template"], "probe_policy": "NONE_ZERO_INTERFACE",
        "side": e03_prod["side"], "d_target": e03_prod["d_target"],
        "entry_scale": e03_prod["entry_scale"], "exit_scale": e03_prod["exit_scale"],
        "production_d_mid": e03_prod["d_mid"], "oracle_d_mid": e03_best["d_mid"],
        "oracle_entry_scale": e03_best["entry_scale"], "oracle_exit_scale": e03_best["exit_scale"],
        "production_first_failure": e03_prod["first_failure_reason"],
        "oracle_minimum_normalized_safety_slack": e03_best["minimum_normalized_safety_slack"],
        "diagnostic_class": "TEMPLATE_OR_TRANSITION_SELECTION",
        "explanation": "same production d_target/d_mid becomes valid with existing entry=0.5145810930150512 and long exit=6.620727869394503",
    })
    write_csv(OUT / "production_vs_oracle.csv", compare_rows)
    write_csv(OUT / "probe_residual_at_production.csv", residual_rows)
    write_csv(OUT / "root_filter_diagnosis.csv", filter_rows)
    write_csv(OUT / "station_counterfactual_scan.csv", scan_rows)

    # Four requested views per direct case.
    for event_id in direct_ids:
        event = events[event_id]
        oracle_row, _, _ = primary[event_id]
        stations = [f(oracle_row, f"z{i}") for i in range(5)]
        target, middle = f(oracle_row, "d_target"), f(oracle_row, "d_mid")
        left = oracle_row["side"] == "LEFT"
        corridor = corridors.setdefault((event_id, left), make_corridor(event, left))
        prod_rows = [p for p in production if p["event_id"] == event_id]
        oracle_path = path_points(event, target, middle, stations)
        stress = " [LOCALIZATION_STRESS_CANDIDATE]" if event_id == "V2E08" else ""

        # A: XY / corridor view.
        fig, ax = plt.subplots(figsize=(8, 6))
        end_view = min(8.0, max(2.0, corridor["end"] + 3.0))
        refs = local_reference_points(event, end_view)
        center_xy = [(w["x"], w["y"]) for _, w in refs]
        left_xy = [(w["x"] - w["d_left"] * math.sin(w["psi"]),
                    w["y"] + w["d_left"] * math.cos(w["psi"])) for _, w in refs]
        right_xy = [(w["x"] + w["d_right"] * math.sin(w["psi"]),
                     w["y"] - w["d_right"] * math.cos(w["psi"])) for _, w in refs]
        ax.plot(*zip(*center_xy), color="0.65", lw=1, label="global reference")
        ax.plot(*zip(*left_xy), "k-", lw=1.3, label="physical track bounds")
        ax.plot(*zip(*right_xy), "k-", lw=1.3)
        for obstacle in corridor["visible"]:
            for raw, color, alpha, label in [(True, "tab:orange", .25, "raw obstacle"),
                                              (False, "tab:red", .15, "inflated obstacle")]:
                dr = obstacle["raw_d_right"] if raw else obstacle["d_right"]
                dl = obstacle["raw_d_left"] if raw else obstacle["d_left"]
                corners = [xy_at(event, obstacle["start"], dr), xy_at(event, obstacle["start"], dl),
                           xy_at(event, obstacle["end"], dl), xy_at(event, obstacle["end"], dr)]
                ax.fill(*zip(*(corners + [corners[0]])), color=color, alpha=alpha, label=label)
        for index, prod in enumerate(prod_rows):
            p_stations = [f(prod, f"z{i}") for i in range(5)]
            ppath = path_points(event, f(prod, "d_target"), f(prod, "d_mid"), p_stations)
            visible_path = [p for p in ppath if p["station"] <= end_view]
            if visible_path:
                ax.plot([p["x_m"] for p in visible_path], [p["y_m"] for p in visible_path],
                        color="tab:red", alpha=.25, lw=1, label="production P3" if index == 0 else None)
        op = [p for p in oracle_path if p["station"] <= end_view]
        ax.plot([p["x_m"] for p in op], [p["y_m"] for p in op], color="tab:blue", lw=2.5,
                label="oracle-valid representative")
        ego_xy = xy_at(event, 0.0, event["ego_d"])
        ax.scatter(*ego_xy, marker="*", s=100, color="black", zorder=5, label="ego")
        for prod in {p["generator_stage"]: p for p in prod_rows if p["s_probe"]}.values():
            px, py = xy_at(event, f(prod, "s_probe"), f(prod, "d_probe"))
            ax.scatter(px, py, marker="x", s=70, label=f"{prod['generator_stage']} probe")
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_title(f"{event_id} XY / corridor{stress}")
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        handles, labels = ax.get_legend_handles_labels()
        unique = dict(zip(labels, handles)); ax.legend(unique.values(), unique.keys(), fontsize=8)
        fig.tight_layout(); fig.savefig(PLOTS / f"{event_id}_A_xy_corridor.png", dpi=170); plt.close(fig)

        # B: Frenet d(s).
        fig, ax = plt.subplots(figsize=(9, 5.5))
        sgrid = np.linspace(0.0, min(end_view, max(stations[-1], corridor["end"] + 1.0)), 700)
        bounds = [sample_at(event, corridor["visible"], float(s), left) for s in sgrid]
        ax.fill_between(sgrid, [b["lower"] for b in bounds], [b["upper"] for b in bounds],
                        color="tab:green", alpha=.12, label="selected-side corridor branch")
        for obstacle in corridor["visible"]:
            ax.fill_between([max(0, obstacle["start"]), obstacle["end"]],
                            [obstacle["d_right"]] * 2, [obstacle["d_left"]] * 2,
                            color="tab:red", alpha=.22, label="inflated forbidden interval")
        for index, prod in enumerate(prod_rows):
            pz = [f(prod, f"z{i}") for i in range(5)]
            dcurve = [eval_profile(pz, event["ego_d"], f(prod, "d_target"), f(prod, "d_mid"), float(s)) for s in sgrid]
            ax.plot(sgrid, dcurve, color="tab:red", alpha=.25, lw=1,
                    label="production P3" if index == 0 else None)
        od = [eval_profile(stations, event["ego_d"], target, middle, float(s)) for s in sgrid]
        ax.plot(sgrid, od, color="tab:blue", lw=2.2, label="oracle-valid representative")
        for prod in {p["generator_stage"]: p for p in prod_rows if p["s_probe"]}.values():
            ax.scatter(f(prod, "s_probe"), f(prod, "d_probe"), marker="x", s=80,
                       label=f"{prod['generator_stage']} production probe")
            oracle_at = eval_profile(stations, event["ego_d"], target, middle, f(prod, "s_probe"))
            ax.scatter(f(prod, "s_probe"), oracle_at, facecolors="none", edgecolors="tab:blue", s=70,
                       label=f"oracle d at {prod['generator_stage']} probe")
        ax.axhline(0, color="0.6", lw=.7); ax.set_xlim(0, sgrid[-1])
        ax.set_xlabel("forward station from ego [m]"); ax.set_ylabel("d [m]")
        ax.set_title(f"{event_id} Frenet d(s){stress}")
        handles, labels = ax.get_legend_handles_labels(); unique = dict(zip(labels, handles))
        ax.legend(unique.values(), unique.keys(), fontsize=8)
        fig.tight_layout(); fig.savefig(PLOTS / f"{event_id}_B_frenet_d.png", dpi=170); plt.close(fig)

        # C: residual scan.
        fig, ax = plt.subplots(figsize=(8.5, 5))
        event_scan = [r for r in scan_rows if r["event_id"] == event_id]
        groups = defaultdict(list)
        for row in event_scan:
            groups[(row["production_stage"], row["probe_policy"])].append(row)
        for key, group in groups.items():
            ax.plot([r["station_forward_m"] for r in group], [r["residual_m"] for r in group],
                    lw=2, label=" / ".join(key))
            minimum = next(r for r in group if r["is_min_abs_residual"])
            ax.scatter(minimum["station_forward_m"], minimum["residual_m"], s=45)
        for prod in {p["generator_stage"]: p for p in prod_rows if p["s_probe"]}.values():
            ax.axvline(f(prod, "s_probe"), ls="--", alpha=.6,
                       label=f"{prod['generator_stage']} production s_probe")
        ax.axhline(0, color="black", lw=.9)
        ax.set_xlabel("counterfactual probe station [m]")
        ax.set_ylabel("R(s)=d_oracle(s)-d_probe,current(s) [m]")
        ax.set_title(f"{event_id} current-anchor residual scan{stress}")
        ax.legend(fontsize=8); fig.tight_layout()
        fig.savefig(PLOTS / f"{event_id}_C_probe_residual.png", dpi=170); plt.close(fig)

        # D: production roots vs observed hard-valid oracle d_mid.
        fig, ax = plt.subplots(figsize=(7, 5))
        matching_valid = [r for r in valid if r["event_id"] == event_id
                          and r["side"] == oracle_row["side"]
                          and close(f(r, "d_target"), target)
                          and close(f(r, "entry_scale"), f(oracle_row, "entry_scale"))
                          and close(f(r, "exit_scale"), f(oracle_row, "exit_scale"))]
        ax.scatter(np.zeros(len(prod_rows)), [f(p, "d_mid") for p in prod_rows],
                   color="tab:red", alpha=.65, label="production constructed d_mid")
        ax.scatter(np.ones(len(matching_valid)), [f(r, "d_mid") for r in matching_valid],
                   c=[f(r, "minimum_normalized_safety_slack") for r in matching_valid],
                   cmap="viridis", s=25, label="oracle hard-valid d_mid (same tuple)")
        ax.scatter([1], [middle], marker="*", s=140, color="tab:blue", label="diagnosis representative")
        ax.axhline(target, color="0.4", ls=":", label="d_target")
        ax.set_xticks([0, 1], ["production", "oracle same tuple"])
        ax.set_ylabel("d_mid [m]"); ax.set_title(f"{event_id} d_mid miss{stress}")
        ax.legend(fontsize=8); fig.tight_layout(); fig.savefig(PLOTS / f"{event_id}_D_d_mid.png", dpi=170); plt.close(fig)

    analysis_summary = {
        "direct_event_ids_derived": direct_ids,
        "event_dominant_classification": event_classification,
        "production_source_head": subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=REPO, text=True, capture_output=True, check=True).stdout.strip(),
        "all_events_label": "PILOT_SEEN_DEVELOPMENT_DATA",
        "v2e03": {
            "production_target": f(e03_prod, "d_target"), "production_mid": f(e03_prod, "d_mid"),
            "production_entry": f(e03_prod, "entry_scale"), "production_exit": f(e03_prod, "exit_scale"),
            "oracle_valid_entry": f(e03_best, "entry_scale"), "oracle_valid_exit": f(e03_best, "exit_scale"),
            "oracle_minimum_normalized_safety_slack": f(e03_best, "minimum_normalized_safety_slack"),
        },
    }
    (OUT / "analysis_summary.json").write_text(json.dumps(analysis_summary, indent=2) + "\n")
    print(json.dumps(analysis_summary, indent=2))


if __name__ == "__main__":
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/p3_mapping_mpl")
    main()
