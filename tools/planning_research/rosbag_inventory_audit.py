#!/usr/bin/env python3
"""Read-only rosbag metadata inventory, capability, duplicate, and triage audit."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable

import yaml


SCHEMA_VERSION = "rosbag_research_inventory/1"

# Exact names observed in the supplied inventory and corroborated by repository source/config.
# A topic is accepted for a role only when its metadata type also matches the declared type set.
ROLE_RULES: dict[str, dict[str, set[str]]] = {
    "confirmed_static_obstacles": {
        "/confirmed_static_obs": {"f110_msgs/msg/ObstacleArray"},
    },
    "historical_static_obstacles": {
        "/static_obs": {"f110_msgs/msg/ObstacleArray"},
        "/perception/obstacles": {"f110_msgs/msg/ObstacleArray", "f110_msgs/msg/ObstacleArray"},
    },
    "local_avoidance_path": {
        "/avoid_waypoints": {"f110_msgs/msg/OTWpntArray", "f110_msgs/msg/WpntArray"},
    },
    "selected_local_path": {
        "/local_waypoints": {"f110_msgs/msg/WpntArray", "f110_msgs/msg/OTWpntArray"},
    },
    "global_reference_waypoints": {
        "/global_waypoints": {"f110_msgs/msg/WpntArray"},
    },
    "localization_pose": {
        "/pf/pose/odom": {"nav_msgs/msg/Odometry"},
    },
    "frenet_odometry": {
        "/car_state/frenet/odom": {"nav_msgs/msg/Odometry"},
    },
    "vehicle_odometry": {
        "/odom": {"nav_msgs/msg/Odometry"},
        "/car_state/odom": {"nav_msgs/msg/Odometry"},
    },
    "simulator_ground_truth_pose": {
        "/ego_racecar/odom": {"nav_msgs/msg/Odometry"},
    },
    "imu": {
        "/sensors/imu/raw": {"sensor_msgs/msg/Imu"},
        "/sensors/imu": {"vesc_msgs/msg/VescImuStamped"},
        "/imu/data": {"sensor_msgs/msg/Imu"},
    },
    "scan": {
        "/scan": {"sensor_msgs/msg/LaserScan"},
        "/scan_slow": {"sensor_msgs/msg/LaserScan"},
        "/slow_scan": {"sensor_msgs/msg/LaserScan"},
    },
    "tf": {"/tf": {"tf2_msgs/msg/TFMessage"}},
    "tf_static": {"/tf_static": {"tf2_msgs/msg/TFMessage"}},
    "particle_cloud": {
        "/pf/viz/particles": {"geometry_msgs/msg/PoseArray"},
    },
    "drive_command": {
        "/drive": {"ackermann_msgs/msg/AckermannDriveStamped"},
        "/drive_autonomous": {"ackermann_msgs/msg/AckermannDriveStamped"},
        "/ackermann_cmd": {"ackermann_msgs/msg/AckermannDriveStamped"},
    },
    "steering": {
        "/commands/servo/position": {"std_msgs/msg/Float64"},
        "/sensors/servo_position_command": {"std_msgs/msg/Float64"},
        "/ackermann_cmd": {"ackermann_msgs/msg/AckermannDriveStamped"},
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-root", type=Path, required=True)
    parser.add_argument("--inventory", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--content-hash-candidates",
        action="store_true",
        help="Hash storage files only inside metadata/size/topic duplicate candidate groups.",
    )
    return parser.parse_args()


def nested_int(value: Any, *keys: str) -> int:
    current = value
    for key in keys:
        if not isinstance(current, dict):
            return 0
        current = current.get(key, 0)
    try:
        return int(current)
    except (TypeError, ValueError):
        return 0


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def normalize_basename(name: str) -> str:
    value = re.sub(r"^Copy of\s+", "", name, flags=re.IGNORECASE)
    value = re.sub(r"\s*\(Copy\)\s*$", "", value, flags=re.IGNORECASE)
    value = re.sub(r"\.2$", "", value)
    value = re.sub(r"\s+copy\s*$", "", value, flags=re.IGNORECASE)
    return value.strip().casefold()


def name_duplicate_pattern(name: str) -> bool:
    return bool(re.search(r"(^Copy of\s|\(Copy\)|\.2$|\scopy$)", name, re.IGNORECASE))


def storage_files(metadata_path: Path, info: dict[str, Any]) -> list[Path]:
    relative: list[str] = []
    for item in info.get("relative_file_paths", []) or []:
        if isinstance(item, str):
            relative.append(item)
    for item in info.get("files", []) or []:
        if isinstance(item, dict) and isinstance(item.get("path"), str):
            relative.append(item["path"])
    if not relative:
        storage = str(info.get("storage_identifier", ""))
        suffix = ".mcap" if storage == "mcap" else ".db3" if storage == "sqlite3" else ""
        relative = [path.name for path in metadata_path.parent.glob(f"*{suffix}")] if suffix else []
    return [metadata_path.parent / item for item in dict.fromkeys(relative)]


def topic_records(info: dict[str, Any]) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for entry in info.get("topics_with_message_count", []) or []:
        metadata = entry.get("topic_metadata", {}) if isinstance(entry, dict) else {}
        records.append(
            {
                "name": str(metadata.get("name", "")),
                "type": str(metadata.get("type", "")),
                "message_count": int(entry.get("message_count", 0) or 0),
            }
        )
    return sorted(records, key=lambda item: (item["name"], item["type"]))


def roles_for(topics: list[dict[str, Any]]) -> dict[str, list[str]]:
    observed = {(item["name"], item["type"]) for item in topics if item["message_count"] > 0}
    roles: dict[str, list[str]] = {}
    for role, rules in ROLE_RULES.items():
        matches = sorted(
            name for name, types in rules.items() if any((name, topic_type) in observed for topic_type in types)
        )
        roles[role] = matches
    return roles


def capabilities(roles: dict[str, list[str]]) -> dict[str, bool]:
    any_obstacles = bool(roles["confirmed_static_obstacles"] or roles["historical_static_obstacles"])
    reference_and_frenet = bool(roles["global_reference_waypoints"] and roles["frenet_odometry"])
    return {
        "CAN_REPLAY_PLANNER_INPUT": any_obstacles and reference_and_frenet,
        "CAN_REPLAY_CURRENT_PLANNER_INPUT": bool(roles["confirmed_static_obstacles"]) and reference_and_frenet,
        "HAS_LOCALIZATION": bool(roles["localization_pose"]),
        "HAS_LOCALIZATION_COVARIANCE": bool(roles["localization_pose"]),
        "HAS_FRENET_STATE": bool(roles["frenet_odometry"]),
        "HAS_ODOMETRY": bool(roles["vehicle_odometry"] or roles["simulator_ground_truth_pose"]),
        "HAS_IMU": bool(roles["imu"]),
        "HAS_SCAN": bool(roles["scan"]),
        "HAS_TF": bool(roles["tf"]),
        "HAS_TF_STATIC": bool(roles["tf_static"]),
        "HAS_PARTICLES": bool(roles["particle_cloud"]),
        "HAS_GT": bool(roles["simulator_ground_truth_pose"]),
        "HAS_PLANNER_OUTPUT": bool(roles["local_avoidance_path"] or roles["selected_local_path"]),
        "HAS_DRIVE_COMMAND": bool(roles["drive_command"]),
        "HAS_STEERING": bool(roles["steering"]),
        "CAN_AUDIT_LOCALIZATION": bool(roles["localization_pose"]),
    }


def load_bag(metadata_path: Path) -> dict[str, Any]:
    raw = metadata_path.read_bytes()
    document = yaml.safe_load(raw) or {}
    info = document.get("rosbag2_bagfile_information", {})
    topics = topic_records(info)
    files = storage_files(metadata_path, info)
    file_rows = []
    for path in files:
        file_rows.append(
            {
                "path": path.name,
                "exists": path.is_file(),
                "size_bytes": path.stat().st_size if path.is_file() else 0,
            }
        )
    duration_ns = nested_int(info, "duration", "nanoseconds")
    start_ns = nested_int(info, "starting_time", "nanoseconds_since_epoch")
    roles = roles_for(topics)
    structural_payload = {
        "storage": str(info.get("storage_identifier", "")),
        "duration_ns": duration_ns,
        "message_count": int(info.get("message_count", 0) or 0),
        "topics": topics,
        "file_sizes": sorted(item["size_bytes"] for item in file_rows),
    }
    structural_sha = hashlib.sha256(
        json.dumps(structural_payload, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    return {
        "path": str(metadata_path.parent),
        "name": metadata_path.parent.name,
        "metadata_path": str(metadata_path),
        "storage_format": str(info.get("storage_identifier", "")),
        "duration_ns": duration_ns,
        "duration_s": duration_ns / 1e9,
        "start_ns": start_ns,
        "end_ns": start_ns + duration_ns if start_ns else 0,
        "message_count": int(info.get("message_count", 0) or 0),
        "size_bytes": sum(item["size_bytes"] for item in file_rows),
        "topics": topics,
        "topic_count": len(topics),
        "files": file_rows,
        "metadata_sha256": hashlib.sha256(raw).hexdigest(),
        "structural_sha256": structural_sha,
        "normalized_basename": normalize_basename(metadata_path.parent.name),
        "name_duplicate_pattern": name_duplicate_pattern(metadata_path.parent.name),
        "roles": roles,
        "capabilities": capabilities(roles),
        "content_sha256": "",
    }


def content_hash(bag: dict[str, Any]) -> str:
    parts = []
    base = Path(bag["path"])
    for item in bag["files"]:
        path = base / item["path"]
        if not path.is_file():
            return "MISSING_STORAGE_FILE"
        parts.append((item["size_bytes"], sha256_file(path)))
    digest = hashlib.sha256()
    for size, value in sorted(parts):
        digest.update(f"{size}:{value}\n".encode())
    return digest.hexdigest()


def write_csv(path: Path, fields: list[str], rows: Iterable[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def group_values(bags: list[dict[str, Any]], field: str) -> dict[str, list[int]]:
    grouped: dict[str, list[int]] = defaultdict(list)
    for index, bag in enumerate(bags):
        value = str(bag[field])
        if value:
            grouped[value].append(index)
    return {key: values for key, values in grouped.items() if len(values) > 1}


def duplicate_rows(bags: list[dict[str, Any]], do_content_hash: bool) -> list[dict[str, Any]]:
    normalized = group_values(bags, "normalized_basename")
    structural = group_values(bags, "structural_sha256")
    metadata = group_values(bags, "metadata_sha256")
    candidate_indexes = {index for group in structural.values() for index in group}
    if do_content_hash:
        for index in sorted(candidate_indexes):
            bags[index]["content_sha256"] = content_hash(bags[index])
    rows: list[dict[str, Any]] = []
    seen: set[tuple[int, int]] = set()
    for indexes in (normalized.values(), structural.values(), metadata.values()):
        for group in indexes:
            for position, left in enumerate(group):
                for right in group[position + 1 :]:
                    pair = (min(left, right), max(left, right))
                    if pair in seen:
                        continue
                    seen.add(pair)
                    first, second = bags[pair[0]], bags[pair[1]]
                    same_content = bool(
                        first["content_sha256"]
                        and first["content_sha256"] == second["content_sha256"]
                    )
                    evidence = []
                    if first["normalized_basename"] == second["normalized_basename"]:
                        evidence.append("NORMALIZED_BASENAME")
                    if first["metadata_sha256"] == second["metadata_sha256"]:
                        evidence.append("METADATA_SHA256")
                    if first["structural_sha256"] == second["structural_sha256"]:
                        evidence.append("STORAGE_SIZE_DURATION_TOPIC_SIGNATURE")
                    if same_content:
                        evidence.append("STORAGE_CONTENT_SHA256")
                    rows.append(
                        {
                            "bag_path_a": first["path"],
                            "bag_path_b": second["path"],
                            "evidence_stages": ";".join(evidence),
                            "exact_duplicate_confirmed": same_content,
                            "content_hash_performed": do_content_hash and pair[0] in candidate_indexes,
                            "automatic_action": "NONE_READ_ONLY",
                        }
                    )
    return sorted(rows, key=lambda row: (row["bag_path_a"], row["bag_path_b"]))


def priority_for(bag: dict[str, Any], possible_duplicate: bool) -> tuple[list[str], list[str], int]:
    name = bag["name"]
    capabilities_ = bag["capabilities"]
    labels: list[str] = []
    reasons: list[str] = []
    score = 0
    if name.startswith("frenet_jump_20260728_"):
        labels.append("PRIORITY_LOCALIZATION_STRESS")
        reasons.append("explicit frenet_jump stress group; inspect actual localization/Frenet topics")
        score += 100
    if any(token in name for token in ("2026_08_24", "2026_08_25", "2026_08_26", "2026-08-24", "2026-08-25", "2026-08-26")):
        labels.append("PRIORITY_RECENT_COMPETITION")
        reasons.append("recording name is in requested 2026-08-24..26 window")
        score += 90
    if capabilities_["CAN_REPLAY_PLANNER_INPUT"] or name in {"avoid_test", "avoid_test2"}:
        labels.append("PRIORITY_PLANNING_REPLAY")
        reasons.append(
            "has obstacle+global+Frenet replay inputs"
            if capabilities_["CAN_REPLAY_PLANNER_INPUT"]
            else "requested avoidance group; topic set is incomplete for current direct replay"
        )
        score += 60
    if any(token in name for token in ("2026_08_16", "2026_08_17", "2026_08_18", "2026-08-16", "2026-08-17", "2026-08-18")):
        labels.append("HISTORICAL_REFERENCE")
        reasons.append("recording name is in requested P3 development window")
        score += 45
    if possible_duplicate:
        labels.append("POSSIBLE_DUPLICATE")
        reasons.append("matched staged name and/or metadata/size/topic signature; no deletion implied")
        score -= 10
    if not labels or not (capabilities_["HAS_LOCALIZATION"] or capabilities_["CAN_REPLAY_PLANNER_INPUT"]):
        labels.append("INSUFFICIENT_TOPICS")
        reasons.append("lacks localization pose and/or complete planner replay inputs")
        score -= 30
    return labels, reasons, score


def markdown_topic_mapping(bags: list[dict[str, Any]]) -> str:
    observed: dict[str, Counter[tuple[str, str]]] = defaultdict(Counter)
    for bag in bags:
        topic_lookup = {(item["name"], item["type"]): item["message_count"] for item in bag["topics"]}
        for role, names in bag["roles"].items():
            for name in names:
                for (topic_name, topic_type), count_ in topic_lookup.items():
                    if topic_name == name and count_ > 0:
                        observed[role][(topic_name, topic_type)] += 1
    lines = [
        "# Topic role mapping",
        "",
        f"Schema: `{SCHEMA_VERSION}`. Mapping uses exact names/types found in the 137 metadata files; "
        "the current planner contract is corroborated by `src/local_planning/src/local_planner_node.cpp` "
        "and its YAML. A count is the number of bags with at least one message.",
        "",
        "| Role | Exact topic | Type | Bags | Authority note |",
        "|---|---|---|---:|---|",
    ]
    for role in ROLE_RULES:
        for (name, topic_type), count_ in sorted(observed[role].items()):
            note = "metadata + current source/config"
            if role == "historical_static_obstacles":
                note = "metadata; historical/remap input, not current confirmed-only default"
            elif role == "simulator_ground_truth_pose":
                note = "metadata + simulator naming; GT interpretation is an explicit inference"
            elif role == "localization_pose":
                note = "metadata + kinematic_localization output contract"
            elif role == "imu" and topic_type == "vesc_msgs/msg/VescImuStamped":
                note = "metadata role; current extractor uses the recorded standard raw IMU companion"
            lines.append(f"| `{role}` | `{name}` | `{topic_type}` | {count_} | {note} |")
    lines.extend(
        [
            "",
            "`/pf/viz/particles` is a `PoseArray`; metadata does not expose particle weights, so ESS "
            "is unavailable unless a future weighted message type is recorded. Odometry covariance is "
            "read from `/pf/pose/odom.pose.covariance`, not from a separate topic.",
            "",
        ]
    )
    return "\n".join(lines)


def localization_schema() -> str:
    return """# Localization feature schema

The extractor emits features only. `localization_label` is always `LOC_UNKNOWN`; no CLEAN/BAD
threshold or automatic deletion/move policy exists.

| Column family | Fields | Meaning |
|---|---|---|
| join/time | bag_path, pose_topic, timestamp_ns, localization_label | Read-only source and pose sample key |
| availability | pose_sample_gap_s, timestamp_regression, scan_pose_offset_s, odom_pose_offset_s, frenet_pose_offset_s, imu_pose_offset_s, tf_nearest_offset_s, tf_static_present | Timestamp proximity; nearest-message offset is signed |
| pose continuity | dx_m, dy_m, distance_jump_m, unwrapped_yaw_rad, unwrapped_dyaw_rad, implied_speed_mps, implied_yaw_rate_radps | Consecutive localization samples; no quality threshold |
| Frenet continuity | frenet_s_m, frenet_d_m, ds_m, dd_m, backward_s_jump_m, abs_d_jump_m | Raw consecutive Frenet deltas; closed-track correction is not guessed |
| cross-sensor | odom_speed_mps, localization_minus_odom_speed_mps, imu_yaw_rate_radps, localization_minus_imu_yaw_rate_radps, short_window_s, localization_short_window_displacement_m, odom_short_window_distance_m, short_window_displacement_residual_m | Populated only when actual topics exist |
| MCL | covariance_xx, covariance_yy, covariance_yawyaw, particle_count, particle_x_std_m, particle_y_std_m, particle_yaw_circular_std_rad, particle_multimodality_proxy_m, particle_ess | Split-centroid separation is a threshold-free multimodality proxy; ESS remains blank for PoseArray because it has no weights |
| ground truth | gt_x_m, gt_y_m, gt_yaw_rad, gt_position_error_m, gt_yaw_error_rad | Populated only for exact `/ego_racecar/odom`; Frenet GT error remains unavailable without an explicit GT projection contract |
| unsupported | track_bound_inconsistency, projection_branch_jump_proxy, gt_frenet_s_error_m, gt_frenet_d_error_m | Blank with an explicit availability note; no reference-map branch policy is invented |

All timestamps come from rosbag storage order/time. The tool never writes to the bag directory.
"""


def group_rows(bags: list[dict[str, Any]], predicate) -> list[str]:
    rows = []
    for bag in bags:
        if predicate(bag["name"]):
            nonempty = [item["name"] for item in bag["topics"] if item["message_count"] > 0]
            rows.append(f"- `{bag['name']}`: {', '.join(nonempty) if nonempty else '(no non-empty topic)' }")
    return rows or ["- (none)"]


def main() -> int:
    args = parse_args()
    root = args.bag_root.resolve()
    output = args.output_dir.resolve()
    if output == root or root in output.parents:
        raise SystemExit("output directory must not be inside the read-only bag root")
    output.mkdir(parents=True, exist_ok=True)

    with args.inventory.open(newline="", encoding="utf-8") as stream:
        supplied_rows = list(csv.DictReader(stream))
    metadata_paths = sorted(root.rglob("metadata.yaml"), key=lambda path: str(path))
    bags = [load_bag(path) for path in metadata_paths]
    duplicates = duplicate_rows(bags, args.content_hash_candidates)
    duplicate_paths = {
        row[key] for row in duplicates for key in ("bag_path_a", "bag_path_b")
    }

    enriched_rows = []
    capability_rows = []
    priority_rows = []
    for bag in bags:
        enriched_rows.append(
            {
                "schema_version": SCHEMA_VERSION,
                "path": bag["path"],
                "name": bag["name"],
                "storage_format": bag["storage_format"],
                "duration_s": f"{bag['duration_s']:.9f}",
                "size_bytes": bag["size_bytes"],
                "message_count": bag["message_count"],
                "topic_count": bag["topic_count"],
                "start_ns": bag["start_ns"],
                "end_ns": bag["end_ns"],
                "topics_json": json.dumps(bag["topics"], separators=(",", ":")),
                "files_json": json.dumps(bag["files"], separators=(",", ":")),
                "metadata_sha256": bag["metadata_sha256"],
                "structural_sha256": bag["structural_sha256"],
                "content_sha256": bag["content_sha256"],
                "name_duplicate_pattern": bag["name_duplicate_pattern"],
                "inventory_path_match": any(row.get("bag_path") == bag["path"] for row in supplied_rows),
            }
        )
        capability_rows.append(
            {
                "path": bag["path"],
                "name": bag["name"],
                **{key: ";".join(value) for key, value in bag["roles"].items()},
                **bag["capabilities"],
            }
        )
        labels, reasons, score = priority_for(bag, bag["path"] in duplicate_paths)
        priority_rows.append(
            {
                "path": bag["path"],
                "name": bag["name"],
                "priority_labels": ";".join(labels),
                "reason": " | ".join(reasons),
                "research_priority_score": score,
                "automatic_action": "NONE_READ_ONLY",
            }
        )

    write_csv(
        output / "rosbag_inventory_enriched.csv",
        list(enriched_rows[0].keys()) if enriched_rows else [],
        enriched_rows,
    )
    write_csv(
        output / "rosbag_topic_capabilities.csv",
        list(capability_rows[0].keys()) if capability_rows else [],
        capability_rows,
    )
    write_csv(
        output / "rosbag_duplicate_candidates.csv",
        ["bag_path_a", "bag_path_b", "evidence_stages", "exact_duplicate_confirmed", "content_hash_performed", "automatic_action"],
        duplicates,
    )
    write_csv(
        output / "rosbag_priority_candidates.csv",
        ["path", "name", "priority_labels", "reason", "research_priority_score", "automatic_action"],
        sorted(priority_rows, key=lambda row: (-int(row["research_priority_score"]), row["name"])),
    )
    (output / "topic_role_mapping.md").write_text(markdown_topic_mapping(bags), encoding="utf-8")
    (output / "localization_feature_schema.md").write_text(localization_schema(), encoding="utf-8")

    capability_count = Counter()
    for bag in bags:
        for name, enabled in bag["capabilities"].items():
            capability_count[name] += int(enabled)
    ranked_recommendations = sorted(
        priority_rows,
        key=lambda row: (
            -int(row["research_priority_score"]),
            name_duplicate_pattern(row["name"]),
            row["name"],
        ),
    )
    recommendations = []
    recommended_normalized_names: set[str] = set()
    for row in ranked_recommendations:
        normalized = normalize_basename(row["name"])
        if normalized in recommended_normalized_names:
            continue
        recommendations.append(row)
        recommended_normalized_names.add(normalized)
        if len(recommendations) == 15:
            break
    report = [
        "# Rosbag triage report",
        "",
        f"- Schema: `{SCHEMA_VERSION}`",
        f"- Supplied inventory rows: **{len(supplied_rows)}**",
        f"- Metadata files discovered read-only: **{len(bags)}**",
        f"- Inventory paths matched: **{sum(row['inventory_path_match'] for row in enriched_rows)}**",
        f"- Planner replay capable (historical static input accepted with remap): **{capability_count['CAN_REPLAY_PLANNER_INPUT']}**",
        f"- Current confirmed-only planner replay capable: **{capability_count['CAN_REPLAY_CURRENT_PLANNER_INPUT']}**",
        f"- Localization feature audit capable: **{capability_count['CAN_AUDIT_LOCALIZATION']}**",
        f"- Simulator GT topic present: **{capability_count['HAS_GT']}**",
        f"- Duplicate candidate pairs: **{len(duplicates)}**",
        f"- Content hashes requested/performed: **{args.content_hash_candidates}**",
        "- Destructive action: **none**",
        "",
        "## Recommended first 15 bags",
        "",
    ]
    report.extend(
        f"{index}. `{row['name']}` — {row['priority_labels']}: {row['reason']}"
        for index, row in enumerate(recommendations, 1)
    )
    report.extend(["", "## Requested recent group (2026-08-24..26)", ""])
    report.extend(group_rows(bags, lambda name: any(token in name for token in ("2026_08_24", "2026_08_25", "2026_08_26", "2026-08-24", "2026-08-25", "2026-08-26"))))
    report.extend(["", "## `frenet_jump_20260728_*` group", ""])
    report.extend(group_rows(bags, lambda name: name.startswith("frenet_jump_20260728_")))
    report.extend(["", "## `avoid_test*` group", ""])
    report.extend(group_rows(bags, lambda name: name in {"avoid_test", "avoid_test2"}))
    report.extend(
        [
            "",
            "## Evidence limits",
            "",
            "- A metadata capability proves recorded topic availability, not message correctness.",
            "- `HAS_GT` treats `/ego_racecar/odom` as simulator ground truth based on its simulator contract; this is an explicit inference and must be confirmed per experiment.",
            "- Duplicate rows are candidates only. Exact storage duplication is confirmed only when optional content hashing is requested and hashes match.",
            "- No localization quality label is assigned.",
            "",
        ]
    )
    (output / "rosbag_triage_report.md").write_text("\n".join(report), encoding="utf-8")
    print(json.dumps({"bags": len(bags), "inventory_rows": len(supplied_rows), "duplicates": len(duplicates), **capability_count}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
