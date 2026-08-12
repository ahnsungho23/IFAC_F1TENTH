#!/usr/bin/env python3
"""Build the local planner's tracking-error LUT from recorded lap_referee traces.

The LUT reserves, per (planned speed, path curvature) cell, how far the car may drift from the
path it was told to follow. That is a measurable quantity, not a guess: on an OBSTACLE-FREE lap
the published local path IS the global race line, so the referee's |lat_err| against the race line
is exactly the tracking error, and it already carries the speed and the nearest waypoint index.

Only obstacle-free samples may be used. During an avoidance maneuver the car leaves the race line
on purpose by 0.3-0.5 m, and counting that as tracking error inflates every cell it lands in --
which is why --obstacle-s exists and why aggregating raw traces blindly produces a table that
measures avoidance amplitude instead.

  usage:
    tracking_error_lut_from_traces.py TRACE.csv [TRACE.csv ...] \
        --waypoints offline_trajectory_generator/output/ifac_track/global_waypoints.csv \
        [--obstacle-s 9.0 12.4 15.4] [--obstacle-exclusion-m 3.0] \
        [--max-s 4.8] [--safety-factor 1.25] [--floor 0.05]

Cells the race line never visits (fast through a hairpin, or anything below 3 m/s during normal
racing) collect no samples and are filled by monotone extension from the cells that did, never
by anything smaller than an already-measured neighbour. Every emitted cell is reported with its
sample count so a thinly-supported number is visible rather than implied.
"""
import argparse
import csv
import math
import sys

SPEED_BINS = [0.0, 1.5, 3.0, 4.5, 6.5]
CURVATURE_BINS = [0.0, 0.2, 0.5, 0.9, 1.316266519079011]


def bin_index(value, bins):
    for i in range(len(bins) - 1):
        if bins[i] <= value < bins[i + 1]:
            return i
    return len(bins) - 2


def load_curvature(path):
    with open(path) as handle:
        return {i: abs(float(r["kappa_radpm"])) for i, r in enumerate(csv.DictReader(handle))}


def collect(trace_paths, curvature_by_index, obstacle_s, exclusion_m, max_s, min_speed):
    cells = {}
    used, skipped = 0, 0
    for trace_path in trace_paths:
        try:
            with open(trace_path) as handle:
                rows = list(csv.DictReader(handle))
        except OSError as error:
            print(f"  skip {trace_path}: {error}", file=sys.stderr)
            continue
        if not rows or "lat_err" not in rows[0]:
            print(f"  skip {trace_path}: no lat_err column", file=sys.stderr)
            continue
        for row in rows:
            speed = float(row["v"])
            station = float(row["s"])
            error = abs(float(row["lat_err"]))
            index = int(row["nearest_idx"])
            if speed < min_speed:
                continue
            if max_s is not None and station > max_s:
                skipped += 1
                continue
            if any(abs(station - o) < exclusion_m for o in obstacle_s):
                skipped += 1
                continue
            curvature = curvature_by_index.get(index, 0.0)
            key = (bin_index(speed, SPEED_BINS), bin_index(curvature, CURVATURE_BINS))
            cells.setdefault(key, []).append(error)
            used += 1
    return cells, used, skipped


def fill(cells, safety_factor, floor):
    speeds = len(SPEED_BINS)
    curvatures = len(CURVATURE_BINS)
    measured = {}
    for (si, ci), samples in cells.items():
        measured[(si, ci)] = max(samples)

    table = [[None] * curvatures for _ in range(speeds)]
    for si in range(speeds):
        for ci in range(curvatures):
            # A LUT node sits on a bin edge, so it must cover the cells on both sides of it.
            neighbours = [
                measured[(s, c)]
                for s in (si - 1, si) for c in (ci - 1, ci)
                if (s, c) in measured
            ]
            if neighbours:
                table[si][ci] = max(neighbours) * safety_factor

    # Tracking error grows with both speed and curvature, so an unmeasured node is bounded from
    # below by every node it dominates (slower and straighter) and from above by every node that
    # dominates it (faster and more curved). Taking the larger of the dominated values first keeps
    # the table monotone -- the gap-driven speed cap inverts it by bisection and needs that.
    known = [row[:] for row in table]
    for si in range(speeds):
        for ci in range(curvatures):
            dominated = [
                known[s][c]
                for s in range(si + 1) for c in range(ci + 1)
                if known[s][c] is not None
            ]
            if dominated:
                value = max(dominated)
                if table[si][ci] is None or table[si][ci] < value:
                    table[si][ci] = value
    # A node below everything measured (slow and straight) has no lower bound to inherit; the
    # smallest value that still dominates it is the tightest honest ceiling available.
    for si in range(speeds):
        for ci in range(curvatures):
            if table[si][ci] is not None:
                continue
            dominating = [
                known[s][c]
                for s in range(si, speeds) for c in range(ci, curvatures)
                if known[s][c] is not None
            ]
            table[si][ci] = min(dominating) if dominating else floor
    return [
        [math.ceil(max(v, floor) * 200.0) / 200.0 for v in row]
        for row in table
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("traces", nargs="+")
    parser.add_argument("--waypoints", required=True)
    parser.add_argument("--obstacle-s", nargs="*", type=float, default=[])
    parser.add_argument("--obstacle-exclusion-m", type=float, default=3.0)
    parser.add_argument("--max-s", type=float, default=None)
    parser.add_argument("--min-speed", type=float, default=0.3)
    parser.add_argument("--safety-factor", type=float, default=1.25)
    parser.add_argument("--floor", type=float, default=0.05)
    args = parser.parse_args()

    curvature_by_index = load_curvature(args.waypoints)
    cells, used, skipped = collect(
        args.traces, curvature_by_index, args.obstacle_s,
        args.obstacle_exclusion_m, args.max_s, args.min_speed)
    if not used:
        print("no obstacle-free samples survived the filters", file=sys.stderr)
        return 1
    print(f"# obstacle-free samples: {used} (excluded {skipped})")
    print("# measured max |lat_err| per cell, with sample counts:")
    for si in range(len(SPEED_BINS) - 1):
        for ci in range(len(CURVATURE_BINS) - 1):
            samples = cells.get((si, ci))
            if samples:
                ordered = sorted(samples)
                print(
                    f"#   v[{SPEED_BINS[si]:.1f},{SPEED_BINS[si+1]:.1f}) "
                    f"k[{CURVATURE_BINS[ci]:.2f},{CURVATURE_BINS[ci+1]:.2f}): "
                    f"n={len(samples):5d} p95={ordered[int(0.95*len(ordered))]:.3f} "
                    f"max={max(samples):.3f}")

    table = fill(cells, args.safety_factor, args.floor)
    print(f"\n# safety factor {args.safety_factor}, floor {args.floor} m,"
          f" unmeasured nodes filled by monotone extension")
    print("    tracking_error_lut_speed_bins_mps: " +
          "[" + ", ".join(f"{b}" for b in SPEED_BINS) + "]")
    print("    tracking_error_lut_curvature_bins_radpm: " +
          "[" + ", ".join(f"{b}" for b in CURVATURE_BINS) + "]")
    print("    tracking_error_lut_values_m: [")
    for si, row in enumerate(table):
        measured_flags = "".join(
            "m" if (si, ci) in cells or (si - 1, ci) in cells or
                   (si, ci - 1) in cells or (si - 1, ci - 1) in cells else "-"
            for ci in range(len(CURVATURE_BINS)))
        print(f"      {', '.join(f'{v:.3f}' for v in row)},"
              f"   # v={SPEED_BINS[si]:<4} [{measured_flags}]")
    print("    ]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
