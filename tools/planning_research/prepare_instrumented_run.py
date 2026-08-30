#!/usr/bin/env python3
"""Prepare explicit provenance and a default-off local-planner instrumentation overlay."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path

import yaml


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path.cwd())
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--scenario-id", default="")
    parser.add_argument("--research-output-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--queue-capacity", type=int, default=128)
    parser.add_argument("--all-violation-audit", action="store_true")
    return parser.parse_args()


def git(repo: Path, *args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=repo, check=True, text=True, capture_output=True
    ).stdout.rstrip("\n")


def aggregate_sha(repo: Path, paths: list[Path]) -> tuple[str, list[dict[str, str]]]:
    aggregate = hashlib.sha256()
    manifest = []
    for path in sorted(paths, key=lambda item: str(item.relative_to(repo))):
        relative = str(path.relative_to(repo))
        content = path.read_bytes()
        value = hashlib.sha256(content).hexdigest()
        aggregate.update(relative.encode() + b"\0" + bytes.fromhex(value))
        manifest.append({"path": relative, "sha256": value})
    return aggregate.hexdigest(), manifest


def main() -> int:
    args = arguments()
    repo = args.repo.resolve()
    output = args.output_dir.resolve()
    if output.exists():
        raise SystemExit(f"refusing to overwrite existing preparation directory: {output}")
    if args.queue_capacity < 1:
        raise SystemExit("--queue-capacity must be positive")
    output.mkdir(parents=True)

    package = repo / "src/local_planning"
    source_files = [
        path for root in (package / "include", package / "src")
        for path in root.rglob("*") if path.suffix in {".hpp", ".cpp"}
    ] + [package / "CMakeLists.txt", package / "package.xml"]
    config_files = sorted((package / "config").glob("*"))
    config_files = [path for path in config_files if path.is_file()]
    source_sha, source_manifest = aggregate_sha(repo, source_files)
    config_sha, config_manifest = aggregate_sha(repo, config_files)
    dirty_status = git(repo, "status", "--porcelain=v1", "--untracked-files=all")
    parameter_seed = {
        "base_config_sha256": config_sha,
        "research_all_violation_audit_enable": args.all_violation_audit,
        "research_queue_capacity": args.queue_capacity,
        "scenario_id": args.scenario_id,
    }
    parameter_snapshot = hashlib.sha256(
        json.dumps(parameter_seed, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    provenance = {
        "schema_version": "local_planning_research_preparation/1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "repo": str(repo),
        "git_commit": git(repo, "rev-parse", "HEAD"),
        "git_branch": git(repo, "branch", "--show-current"),
        "git_dirty_status": dirty_status,
        "source_sha256": source_sha,
        "config_sha256": config_sha,
        "requested_parameter_snapshot_id": f"sha256:{parameter_snapshot}",
        "source_manifest": source_manifest,
        "config_manifest": config_manifest,
        "note": "The running node also records its sorted effective declared parameter snapshot in metadata.json.",
    }
    (output / "instrumentation_provenance.json").write_text(
        json.dumps(provenance, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    parameters = {
        "/**": {
            "ros__parameters": {
                "research_instrumentation_enable": True,
                "research_all_violation_audit_enable": args.all_violation_audit,
                "research_output_root": str(args.research_output_root.resolve()),
                "research_run_id": args.run_id,
                "research_scenario_id": args.scenario_id,
                "research_git_commit": provenance["git_commit"],
                "research_git_dirty_status": dirty_status,
                "research_source_sha256": source_sha,
                "research_config_sha256": config_sha,
                "research_parameter_snapshot_id": provenance["requested_parameter_snapshot_id"],
                "research_queue_capacity": args.queue_capacity,
            }
        }
    }
    overlay = output / "research_instrumentation.params.yaml"
    overlay.write_text(yaml.safe_dump(parameters, sort_keys=False), encoding="utf-8")
    (output / "HOW_TO_RUN.txt").write_text(
        "Load the ordinary operational YAML first and this overlay second. For a direct node run:\n"
        f"ros2 run local_planning local_planner_node --ros-args --params-file "
        f"{package / 'config/local_planning.yaml'} --params-file {overlay}\n\n"
        "For launch, pass the overlay only through a launch argument that is verified to append a second "
        "parameter file; do not replace the operational YAML. The run directory must not already exist.\n",
        encoding="utf-8",
    )
    print(output)
    print(f"git={provenance['git_commit']} source={source_sha} config={config_sha}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
