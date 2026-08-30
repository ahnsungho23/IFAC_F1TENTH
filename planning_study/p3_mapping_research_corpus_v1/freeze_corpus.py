#!/usr/bin/env python3
"""Freeze episode-level P3 research roles without reading non-pilot oracle outcomes."""

from __future__ import annotations

import csv
import hashlib
import json
import math
from collections import Counter, defaultdict
from pathlib import Path


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
PREP = REPO / "planning_study/p3_oracle_pilot_v2_preparation/oracle_eligible_snapshots.csv"
PILOT = REPO / "planning_study/p3_oracle_pilot_v2/oracle_summary.csv"
REFERENCE = Path("/tmp/p3_oracle_snapshots_v1.tsv")
SEED = "p3_mapping_research_corpus_v1_seed_20260830"
SPLIT_VERSION = "episode_iterative_stratification_v1"
ROLES = ("DEVELOPMENT", "VALIDATION_UNSEEN", "FINAL_HOLDOUT_UNSEEN")


def read_csv(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, rows: list[dict]) -> None:
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def digest(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def sha256(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def reference_domains() -> dict[str, list[dict]]:
    streams: dict[str, list[dict]] = {}
    current = ""
    for line in REFERENCE.read_text(encoding="utf-8").splitlines():
        fields = line.split("\t")
        if fields[0] == "EVENT":
            current = fields[1]
            streams[current] = []
        elif fields[0] == "W" and current:
            streams[current].append({"s": float(fields[2]), "kappa": float(fields[9])})
    return {
        "ref_3f5d487564f4ee3d": streams["E03"],
        "ref_4dc0768eb37d9590": streams["E01"],
    }


def nearest_curvature(points: list[dict], ego_s: float) -> float:
    spacing = sorted(b["s"] - a["s"] for a, b in zip(points, points[1:]))[len(points) // 2]
    length = points[-1]["s"] + spacing
    def distance(point: dict) -> float:
        delta = (point["s"] - ego_s) % length
        return min(delta, length - delta)
    return min(points, key=distance)["kappa"]


def bin_features(row: dict, refs: dict[str, list[dict]]) -> dict[str, str | float]:
    obstacles = json.loads(row["obstacle_summary"])
    centers = [0.5 * (float(o["d_left"]) + float(o["d_right"])) for o in obstacles]
    if centers and all(value > 0.05 for value in centers):
        obstacle_side = "LEFT"
    elif centers and all(value < -0.05 for value in centers):
        obstacle_side = "RIGHT"
    else:
        obstacle_side = "MIXED_OR_CENTER"
    widths = [abs(float(o["d_left"]) - float(o["d_right"])) for o in obstacles]
    maximum_width = max(widths, default=0.0)
    width_bin = "NARROW" if maximum_width < 0.20 else "MEDIUM" if maximum_width < 0.50 else "WIDE"
    obstacle_count = len(obstacles)
    count_bin = "ONE" if obstacle_count == 1 else "TWO" if obstacle_count == 2 else "THREE_PLUS"
    speed = float(row["ego_speed_mps"])
    speed_bin = "STOPPED" if speed < 0.05 else "LOW" if speed < 1.0 else "MOVING"
    curvature = nearest_curvature(refs[row["reference_snapshot_id"]], float(row["ego_s_m"]))
    curvature_abs = abs(curvature)
    curvature_bin = "LOW" if curvature_abs < 0.25 else "MEDIUM" if curvature_abs < 0.75 else "HIGH"
    curvature_sign = "POSITIVE" if curvature > 0.05 else "NEGATIVE" if curvature < -0.05 else "NEAR_ZERO"
    return {
        "obstacle_side_bin": obstacle_side,
        "obstacle_count_bin": count_bin,
        "obstacle_max_width_m": maximum_width,
        "obstacle_width_bin": width_bin,
        "ego_speed_bin": speed_bin,
        "reference_curvature_radpm": curvature,
        "reference_curvature_abs_bin": curvature_bin,
        "reference_curvature_sign_bin": curvature_sign,
        "lifecycle_context_bin": row["lifecycle_state"] + "__" + row["lifecycle_owner"],
    }


def tokens(row: dict) -> tuple[str, ...]:
    axes = (
        "bag", "reference_snapshot_id", "obstacle_side_bin", "obstacle_count_bin",
        "obstacle_width_bin", "ego_speed_bin", "reference_curvature_abs_bin",
        "reference_curvature_sign_bin", "lifecycle_context_bin",
    )
    return tuple(f"{axis}={row[axis]}" for axis in axes)


def iterative_assign(rows: list[dict], capacities: dict[str, int], seed_suffix: str) -> dict[str, str]:
    """Greedy multilabel stratification with fixed capacities and deterministic hash tie-breaks."""
    total = len(rows)
    assert sum(capacities.values()) == total
    feature_total = Counter(token for row in rows for token in tokens(row))
    ordered = sorted(
        rows,
        key=lambda row: (
            min(feature_total[token] for token in tokens(row)),
            sum(feature_total[token] for token in tokens(row)),
            digest(SEED + seed_suffix + "|ORDER|" + row["episode_id"]),
        ),
    )
    assigned_count = Counter()
    assigned_feature: dict[str, Counter] = defaultdict(Counter)
    result: dict[str, str] = {}
    for row in ordered:
        candidates = [role for role in capacities if assigned_count[role] < capacities[role]]
        scores = []
        for role in candidates:
            ratio = capacities[role] / total
            global_target = capacities[role]
            current_global_error = (assigned_count[role] - global_target) ** 2
            next_global_error = (assigned_count[role] + 1 - global_target) ** 2
            score = 0.20 * (next_global_error - current_global_error) / max(1.0, global_target)
            for token in tokens(row):
                target = feature_total[token] * ratio
                current_error = (assigned_feature[role][token] - target) ** 2
                next_error = (assigned_feature[role][token] + 1 - target) ** 2
                score += (next_error - current_error) / max(1.0, target)
            scores.append((score, digest(SEED + seed_suffix + "|TIE|" + row["episode_id"] + "|" + role), role))
        role = min(scores)[2]
        result[row["episode_id"]] = role
        assigned_count[role] += 1
        for token in tokens(row):
            assigned_feature[role][token] += 1
    assert assigned_count == Counter(capacities)
    return result


def main() -> None:
    HERE.mkdir(parents=True, exist_ok=True)
    refs = reference_domains()
    eligible = read_csv(PREP)
    pilot_summary = read_csv(PILOT)
    pilot_by_key = {
        (row["bag"], row["callback_sequence"], row["evaluation_sequence"]): row["event_id"]
        for row in pilot_summary
    }
    enriched = []
    for source in eligible:
        row = dict(source)
        row.update(bin_features(row, refs))
        key = (row["bag"], row["callback_sequence"], row["evaluation_sequence"])
        row["pilot_event_id"] = pilot_by_key.get(key, "")
        enriched.append(row)
    pilot = [row for row in enriched if row["pilot_event_id"]]
    remaining = [row for row in enriched if not row["pilot_event_id"]]
    assert len(pilot) == 9 and len(remaining) == 184

    split = iterative_assign(
        remaining,
        {"DEVELOPMENT": 110, "VALIDATION_UNSEEN": 37, "FINAL_HOLDOUT_UNSEEN": 37},
        "|MAIN_SPLIT",
    )
    development = [row for row in remaining if split[row["episode_id"]] == "DEVELOPMENT"]
    subset = iterative_assign(development, {"ORACLE_SELECTED": 40, "NOT_SELECTED": 70}, "|DEV_SUBSET")

    manifest = []
    for row in enriched:
        if row["pilot_event_id"]:
            role = "PILOT_SEEN_DEVELOPMENT_DATA"
            oracle_selected = False
        else:
            role = split[row["episode_id"]]
            oracle_selected = role == "DEVELOPMENT" and subset[row["episode_id"]] == "ORACLE_SELECTED"
        manifest.append({
            "dataset_role": role,
            "development_oracle_selected": oracle_selected,
            "split_seed": SEED,
            "split_algorithm_version": SPLIT_VERSION,
            "split_uses_oracle_outcomes": False,
            "source_eligible_manifest_sha256": sha256(PREP),
            **row,
        })
    manifest.sort(key=lambda row: (ROLES.index(row["dataset_role"]) if row["dataset_role"] in ROLES else -1,
                                   row["bag"], float(row["elapsed_s"]), row["episode_id"]))
    write_csv(HERE / "dataset_split_manifest.csv", manifest)

    selected = [row for row in manifest if row["dataset_role"] == "DEVELOPMENT"
                and str(row["development_oracle_selected"]).lower() == "true"]
    selected.sort(key=lambda row: digest(SEED + "|DEV_ORACLE_ORDER|" + row["episode_id"]))
    for index, row in enumerate(selected, 1):
        row["development_event_id"] = f"DVE{index:03d}"
        row["selection_rank"] = index
        row["selection_uses_oracle_outcomes"] = False
    write_csv(HERE / "development_oracle_manifest.csv", selected)

    manifest_sha = sha256(HERE / "dataset_split_manifest.csv")
    selected_sha = sha256(HERE / "development_oracle_manifest.csv")
    (HERE / "split_freeze.json").write_text(json.dumps({
        "split_seed": SEED,
        "split_algorithm_version": SPLIT_VERSION,
        "source_eligible_manifest_sha256": sha256(PREP),
        "dataset_split_manifest_sha256": manifest_sha,
        "development_oracle_manifest_sha256": selected_sha,
        "counts": dict(Counter(row["dataset_role"] for row in manifest)),
        "development_oracle_selected_count": len(selected),
        "outcome_fields_consulted_for_non_pilot_split": [],
        "feature_axes": [token.split("=", 1)[0] for token in tokens(selected[0])],
    }, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "counts": dict(Counter(row["dataset_role"] for row in manifest)),
        "selected": len(selected),
        "manifest_sha256": manifest_sha,
        "selected_sha256": selected_sha,
    }, indent=2))


if __name__ == "__main__":
    main()
