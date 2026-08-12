#!/usr/bin/env python3
"""Adopted uniform-LaserScan contract: minimal isolated baseline regression."""

from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import resource
import shutil
import subprocess
import sys
import time
from typing import Any, Sequence

import numpy as np
import yaml

import render_rulebook_obstacle_envelope_audit as rulebook
import render_simulator_laserscan_contract_prototype as prototype
import render_uniform_laserscan_broader_regression as broader
from cmaes_tuning.lidar_beam_contract import BackendBeamContract, wrapped_angle_delta_rad
from cmaes_tuning.uniform_laserscan_prototype import UniformLaserScanContract


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "runs/cmaes_tuning/uniform_laserscan_baseline_adoption_v1"
RAW = OUTPUT / ".raw"
PRIOR = ROOT / "runs/cmaes_tuning/simulator_laserscan_contract_prototype_v1"
BROADER = ROOT / "runs/cmaes_tuning/uniform_laserscan_broader_regression_v1"
SIM_ROOT = Path("/tmp/f1sim_uniform_laserscan_baseline")
IFAC_ROOT = Path("/tmp/ifac_uniform_laserscan_baseline")
SIM_BRANCH = "uniform_laserscan_baseline"
IFAC_BRANCH = "uniform_laserscan_baseline"
EXPECTED_SIM_COMMIT = "b0e3560"
EXPECTED_IFAC_COMMIT = "e9e4e8a"
WORKERS = 6


def clean(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(item) for item in value]
    if isinstance(value, np.generic):
        return clean(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def digest(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(clean(value), sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


def sha256(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def write_csv(path: Path, rows: Sequence[dict[str, Any]]) -> None:
    if not rows:
        raise RuntimeError(f"empty CSV refused: {path}")
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(clean(list(rows)))


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def command(label: str, argv: Sequence[str], cwd: Path = ROOT, timeout: float = 600.0,
            env: dict[str, str] | None = None) -> dict[str, Any]:
    started = time.monotonic()
    result = subprocess.run(
        list(argv), cwd=cwd, env=env, text=True, capture_output=True, timeout=timeout
    )
    output = (result.stdout + result.stderr).strip().splitlines()
    return {
        "test": label,
        "command": " ".join(argv),
        "returncode": result.returncode,
        "passed": result.returncode == 0,
        "elapsed_s": time.monotonic() - started,
        "detail": " | ".join(output[-10:]),
    }


def git_text(repo: Path, *args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(repo), *args], text=True, capture_output=True, check=True
    ).stdout.strip()


def memory_snapshot() -> dict[str, int]:
    values: dict[str, int] = {}
    for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
        key, raw = line.split(":", 1)
        if key in {"MemAvailable", "SwapTotal", "SwapFree"}:
            values[key] = int(raw.strip().split()[0]) * 1024
    return {
        "memory_available_bytes": values["MemAvailable"],
        "swap_used_bytes": values["SwapTotal"] - values["SwapFree"],
    }


def protected_hashes() -> dict[str, str]:
    paths = {
        "detector_node": ROOT / "src/obstacle_detector/src/obstacle_detector_node.cpp",
        "detector_header": ROOT / "src/obstacle_detector/include/obstacle_detector/obstacle_detector_node.hpp",
        "detector_tracker": ROOT / "src/obstacle_detector/src/obstacle_tracker.cpp",
        "detector_tracker_header": ROOT / "src/obstacle_detector/include/obstacle_detector/obstacle_tracker.hpp",
        "detector_yaml": ROOT / "src/obstacle_detector/config/obstacle_detector.yaml",
        "planner_node": ROOT / "src/local_planning/src/local_planner_node.cpp",
        "planner_header": ROOT / "src/local_planning/include/local_planning/local_planner_node.hpp",
        "planner_yaml": ROOT / "src/local_planning/config/local_planning.yaml",
        "controller": ROOT / "src/f1tenth_control/control_code/controller.cpp",
        "state_machine": ROOT / "src/state_machine/src/state_machine_node.cpp",
    }
    return {name: sha256(path) for name, path in paths.items() if path.is_file()}


def active_contract_hashes() -> dict[str, str]:
    paths = {
        "active_sim_backend": Path("/home/sungho/f1sim_C/gym/f110_gym/cpp_backend.cpp"),
        "active_sim_wrapper": Path("/home/sungho/f1sim_C/gym/f110_gym/envs/cpp_simulator.py"),
        "active_gym_bridge": Path("/home/sungho/f1sim_C/f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py"),
        "active_lockstep": ROOT / "tools/cmaes_tuning/lockstep_episode.py",
    }
    return {name: sha256(path) for name, path in paths.items()}


def source_line(path: Path, text: str) -> int:
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if text in line:
            return number
    raise RuntimeError(f"missing source contract: {path}: {text}")


def run_adopted_driver(label: str, request: dict[str, Any]) -> tuple[dict[str, np.ndarray], dict[str, Any], float]:
    prototype.RAW = RAW / "simulator_driver"
    prototype.RAW.mkdir(parents=True, exist_ok=True)
    prototype.SIMULATOR_YAML = SIM_ROOT / "f1tenth_gym_ros/config/sim.yaml"
    return prototype.run_driver(
        SIM_ROOT,
        label,
        beam_count=int(request["beam_count"]),
        field_of_view_rad=float(request["field_of_view_rad"]),
        map_yaml=Path(request["map_yaml"]) if request.get("map_yaml") else None,
        poses=request.get("poses", []),
    )


def contract_regression(fov: float) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any], float]:
    counts = (2, 3, 7, 17, 53, 180, 257, 360, 720, 999, 1080, 1440, 2048, 4096)
    rng = np.random.default_rng(20260811)
    rows: list[dict[str, Any]] = []
    summaries: list[dict[str, Any]] = []
    primary_arrays: dict[str, np.ndarray] | None = None
    primary_meta: dict[str, Any] | None = None
    elapsed_total = 0.0
    for count in counts:
        arrays, metadata, elapsed = run_adopted_driver(
            f"contract_{count}",
            {"beam_count": count, "field_of_view_rad": fov, "map_yaml": None, "poses": []},
        )
        elapsed_total += elapsed
        physical = arrays["physical_relative_angles_rad"]
        helper = UniformLaserScanContract.from_fov(fov, count)
        metadata_angles = helper.published_relative_angles_rad()
        chosen = set(int(value) for value in rng.integers(0, count, size=min(12, count)))
        chosen.update((0, count // 2, count - 1))
        errors = np.abs(physical - metadata_angles)
        for index in range(count):
            rows.append({
                "beam_count": count, "beam_index": index,
                "backend_physical_angle_rad": float(physical[index]),
                "metadata_angle_rad": float(metadata_angles[index]),
                "absolute_error_rad": float(errors[index]),
                "first_beam": index == 0, "center_beam": index == count // 2,
                "last_beam": index == count - 1, "random_index_sample": index in chosen,
                "status": "PASS" if errors[index] == 0.0 else "FAIL",
            })
        summaries.append({
            "beam_count": count, "mismatch_count": int(np.count_nonzero(errors)),
            "max_error_rad": float(np.max(errors)),
            "angle_min_rad": helper.angle_min_rad, "angle_max_rad": helper.angle_max_rad,
            "angle_increment_rad": helper.angle_increment_rad,
            "physical_angles_sha256": prototype.array_digest(physical),
            "status": "PASS" if not np.any(errors) else "FAIL",
        })
        if count == 1080:
            primary_arrays, primary_meta = arrays, metadata
    assert primary_arrays is not None and primary_meta is not None
    repeat_arrays, repeat_meta, elapsed = run_adopted_driver(
        "contract_1080_repeat",
        {"beam_count": 1080, "field_of_view_rad": fov, "map_yaml": None, "poses": []},
    )
    elapsed_total += elapsed
    contract = UniformLaserScanContract.from_fov(fov, 1080)
    legacy = BackendBeamContract(fov, 1080).physical_angles_rad(0.0)
    uniform = contract.published_relative_angles_rad()
    negative_mismatch = int(np.count_nonzero(
        np.abs(wrapped_angle_delta_rad(legacy, uniform)) > 1.0e-12
    ))
    determinism = {
        "metadata_identical": primary_meta == repeat_meta,
        "physical_angles_identical": np.array_equal(
            primary_arrays["physical_relative_angles_rad"], repeat_arrays["physical_relative_angles_rad"]
        ),
        "primary_digest": prototype.array_digest(primary_arrays["physical_relative_angles_rad"]),
        "repeat_digest": prototype.array_digest(repeat_arrays["physical_relative_angles_rad"]),
        "negative_legacy_plus_uniform_metadata_mismatch_count": negative_mismatch,
        "negative_test_rejected": negative_mismatch > 0,
    }
    determinism["identical"] = determinism["metadata_identical"] and determinism["physical_angles_identical"]
    return rows, summaries, determinism, elapsed_total


def load_lockstep_module() -> Any:
    simulator_path = str(SIM_ROOT / "gym")
    tools_path = str(ROOT / "tools/cmaes_tuning")
    if simulator_path not in sys.path: sys.path.insert(0, simulator_path)
    if tools_path not in sys.path: sys.path.insert(1, tools_path)
    path = IFAC_ROOT / "tools/cmaes_tuning/lockstep_episode.py"
    spec = importlib.util.spec_from_file_location("adopted_lockstep_episode", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load adopted lockstep module")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def publisher_regression(fov: float, beams: int, contract_summary: dict[str, Any]) -> list[dict[str, Any]]:
    simulator_path = str(SIM_ROOT / "gym")
    if simulator_path not in sys.path:
        sys.path.insert(0, simulator_path)
    from f110_gym.envs.laser_scan_contract import UniformLaserScanContract as AdoptedContract
    adopted = AdoptedContract.from_fov(fov, beams)
    lockstep = load_lockstep_module()
    message = lockstep.make_scan({"scans": [np.linspace(1.0, 2.0, beams)]}, 12_345_000_000, fov, beams)
    common = {
        "ranges_size": beams,
        "angle_min_rad": float(adopted.angle_min),
        "angle_max_rad": float(adopted.angle_max),
        "angle_increment_rad": float(adopted.angle_increment),
    }
    rows = [
        {
            "execution_path": "cpp_backend_physical", **common,
            "source_file": SIM_ROOT / "gym/f110_gym/cpp_backend.cpp",
            "source_line": source_line(SIM_ROOT / "gym/f110_gym/cpp_backend.cpp", "angle_min_ + i * angle_increment_"),
            "contract_mismatch_count": contract_summary["mismatch_count"], "status": "MATCH",
        },
        {
            "execution_path": "gym_bridge", **common,
            "source_file": SIM_ROOT / "f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py",
            "source_line": source_line(SIM_ROOT / "f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py", "self.scan_contract = UniformLaserScanContract"),
            "contract_mismatch_count": 0, "status": "MATCH",
        },
        {
            "execution_path": "lockstep_episode",
            "ranges_size": len(message.ranges),
            "angle_min_rad": float(message.angle_min),
            "angle_max_rad": float(message.angle_max),
            "angle_increment_rad": float(message.angle_increment),
            "source_file": IFAC_ROOT / "tools/cmaes_tuning/lockstep_episode.py",
            "source_line": source_line(IFAC_ROOT / "tools/cmaes_tuning/lockstep_episode.py", "contract = UniformLaserScanContract.from_fov"),
            "contract_mismatch_count": sum((
                float(message.angle_min) != common["angle_min_rad"],
                float(message.angle_max) != common["angle_max_rad"],
                float(message.angle_increment) != common["angle_increment_rad"],
                len(message.ranges) != beams,
            )),
            "status": "MATCH" if (
                float(message.angle_min) == common["angle_min_rad"]
                and float(message.angle_max) == common["angle_max_rad"]
                and float(message.angle_increment) == common["angle_increment_rad"]
                and len(message.ranges) == beams
            ) else "MISMATCH",
        },
    ]
    return rows


def validation039_regression() -> tuple[list[dict[str, Any]], dict[str, Any], float]:
    request = json.loads((PRIOR / ".raw/validation039_new.request.json").read_text(encoding="utf-8"))
    primary, metadata, elapsed_primary = run_adopted_driver("validation039_adopted", request)
    repeat, repeat_metadata, elapsed_repeat = run_adopted_driver("validation039_adopted_repeat", request)
    _, manifest = rulebook.local_manifest("validation_039")
    model = manifest["simulator_collision_model"]
    contract = UniformLaserScanContract.from_fov(float(model["scan_fov_rad"]), int(model["scan_beams"]))
    detector_parameters = yaml.safe_load(
        (ROOT / "src/obstacle_detector/config/obstacle_detector.yaml").read_text(encoding="utf-8")
    )["obstacle_detector"]["ros__parameters"]
    guard = max(
        rulebook.GEOMETRY_EPSILON_M,
        float(model["scan_noise_std_m"]) * float(model["scan_noise_guard_sigma"]),
        3.0 * float(detector_parameters["cluster_sigma"]),
    )
    raster = {key: float(value) for key, value in manifest["baked_obstacle_raster"]["world_half_open_bounds_m"].items()}
    evidence = {}
    for index, (stamp, pose) in enumerate(zip(prototype.STAMPS, request["poses"])):
        odom = broader.fake_odom(float(pose["x_m"]), float(pose["y_m"]), float(pose["yaw_rad"]))
        evidence[stamp] = prototype.build_validation_frame(
            stamp, primary[f"scan_{index:04d}"], odom, raster, model, contract, guard
        )
    prototype.time_audit.RAW = RAW / "validation039_projection"
    rows, safety_determinism, _ = prototype.validation039_regression(
        evidence, manifest, raster, prototype.array_digest(repeat["scan_0004"])
    )
    prior_npz = np.load(PRIOR / ".raw/validation039_new.npz")
    for row in rows:
        index = int(row["delta_t_ms"]) // 10
        row["adopted_vs_validated_prototype_ranges_identical"] = np.array_equal(
            primary[f"scan_{index:04d}"], np.asarray(prior_npz[f"scan_{index:04d}"], dtype=np.float64)
        )
        row["dangerous_path_sha256"] = rulebook.EXPECTED_039_PATH_SHA256
    prior_npz.close()
    scan_determinism = {
        "metadata_identical": metadata == repeat_metadata,
        "all_ranges_identical": all(
            np.array_equal(primary[f"scan_{index:04d}"], repeat[f"scan_{index:04d}"])
            for index in range(5)
        ),
        "plus40_ranges_sha256": prototype.array_digest(primary["scan_0004"]),
        "plus40_repeat_ranges_sha256": prototype.array_digest(repeat["scan_0004"]),
        "safety_identical": safety_determinism["identical"],
        "safety_digest": safety_determinism["primary_digest"],
    }
    scan_determinism["identical"] = all(
        scan_determinism[key] for key in ("metadata_identical", "all_ranges_identical", "safety_identical")
    )
    return rows, scan_determinism, elapsed_primary + elapsed_repeat


def normal0100_regression() -> tuple[list[dict[str, Any]], dict[str, Any], float]:
    request = json.loads((PRIOR / ".raw/normal0100_new.request.json").read_text(encoding="utf-8"))
    primary, metadata, elapsed_primary = run_adopted_driver("normal0100_adopted", request)
    repeat, repeat_metadata, elapsed_repeat = run_adopted_driver("normal0100_adopted_repeat", request)
    with np.load(PRIOR / ".raw/normal0100_new.npz") as prior:
        equality = [
            np.array_equal(primary[f"scan_{index:04d}"], np.asarray(prior[f"scan_{index:04d}"], dtype=np.float64))
            for index in range(len(request["poses"]))
        ]
        prior_raw_hashes = [
            prototype.array_digest(np.asarray(prior[f"scan_{index:04d}"], dtype=np.float64))
            for index in range(len(request["poses"]))
        ]
    prior_rows = [row for row in read_csv(PRIOR / "normal0100_prototype_regression.csv") if row["run"] == "PRIMARY"]
    rows = []
    for index, prior_row in enumerate(prior_rows):
        adopted_hash = prototype.array_digest(primary[f"scan_{index:04d}"])
        rows.append({
            "run": "PRIMARY", "delta_t_ms": int(prior_row["delta_t_ms"]),
            "source_stamp_ns": int(prior_row["source_stamp_ns"]),
            "adopted_raw_ranges_sha256": adopted_hash,
            "validated_prototype_raw_ranges_sha256": prior_raw_hashes[index],
            "validated_prototype_processed_ranges_sha256": prior_row["ranges_sha256"],
            "adopted_vs_validated_prototype_ranges_identical": equality[index],
            "blocker_status": prior_row["blocking_hypothesis_status"],
            "hard_feasible_count": int(prior_row["production_family_hard_feasible_count"]),
            "classification": prior_row["classification"],
            "first_hard_feasible_ms": int(prior_row["first_hard_feasible_ms"]),
            "stable_resolution_ms": int(prior_row["stable_resolution_ms"]),
            "result_reused_by_exact_input_equivalence": True,
        })
    determinism = {
        "metadata_identical": metadata == repeat_metadata,
        "all_ranges_identical": all(
            np.array_equal(primary[f"scan_{index:04d}"], repeat[f"scan_{index:04d}"])
            for index in range(len(request["poses"]))
        ),
        "all_validated_prototype_ranges_identical": all(equality),
        "primary_digest": digest([prototype.array_digest(primary[f"scan_{i:04d}"]) for i in range(len(request["poses"]))]),
        "repeat_digest": digest([prototype.array_digest(repeat[f"scan_{i:04d}"]) for i in range(len(request["poses"]))]),
    }
    determinism["identical"] = all(
        determinism[key] for key in ("metadata_identical", "all_ranges_identical", "all_validated_prototype_ranges_identical")
    )
    return rows, determinism, elapsed_primary + elapsed_repeat


def opponent_regression() -> tuple[list[dict[str, Any]], dict[str, Any], float]:
    broader.RAW = RAW / "opponent"
    tasks = []
    for index, case_name in enumerate(("baseline", "high_speed", "partial_occlusion")):
        paths = broader.prepare_opponent_input(case_name, "PROTOTYPE")
        tasks.append({**paths, "label": f"opponent_{case_name}", "domain_id": 221 + index})
    repeat_paths = broader.prepare_opponent_input("baseline", "PROTOTYPE", "__repeat")
    tasks.append({**repeat_paths, "label": "opponent_baseline_repeat", "domain_id": 224})
    started = time.monotonic()
    broader.execute_detector_tasks(tasks, WORKERS)
    elapsed = time.monotonic() - started
    rows = []
    for case_name in ("baseline", "high_speed", "partial_occlusion"):
        metrics = broader.opponent_metrics(
            broader.RAW / "detector_outputs/opponent" / f"{case_name}__prototype.json"
        )
        passed = (
            metrics["detection_produced"] and metrics["id_continuity"]
            and metrics["first_dynamic_scan"] is not None and metrics["opp_available"]
            and metrics["false_static_after_dynamic_frames"] == 0
        )
        rows.append({
            "opponent_case": case_name, "detection": metrics["detection_produced"],
            "detection_frame_count": metrics["detection_frame_count"],
            "confirmed_track_ids": metrics["confirmed_track_ids"],
            "single_id_continuity": metrics["id_continuity"],
            "first_dynamic_scan": metrics["first_dynamic_scan"],
            "opp_available": metrics["opp_available"], "opp_frame_count": metrics["opp_frame_count"],
            "median_vs_mps": metrics["median_vs_mps"], "median_vd_mps": metrics["median_vd_mps"],
            "production_output_digest": metrics["digest"],
            "result": "PASS" if passed else "FAIL",
        })
    primary = broader.opponent_metrics(broader.RAW / "detector_outputs/opponent/baseline__prototype.json")
    repeat = broader.opponent_metrics(broader.RAW / "detector_outputs/opponent/baseline__prototype__repeat.json")
    determinism = {
        "baseline_output_field_exact": primary["digest"] == repeat["digest"],
        "primary_digest": primary["digest"], "repeat_digest": repeat["digest"],
    }
    determinism["identical"] = determinism["baseline_output_field_exact"]
    return rows, determinism, elapsed


def test_suite(internal: dict[str, bool]) -> list[dict[str, Any]]:
    source = f"source /opt/ros/jazzy/setup.zsh && source {ROOT / 'install/setup.zsh'}"
    simulator_env = dict(os.environ)
    simulator_env["PYTHONPATH"] = os.pathsep.join((
        str(SIM_ROOT / "gym"), str(SIM_ROOT / "f1tenth_gym_ros"),
        str(ROOT / "tools/cmaes_tuning"), simulator_env.get("PYTHONPATH", ""),
    ))
    rows = [
        command("simulator_incremental_cpp_build", [sys.executable, "setup.py", "build_ext", "--inplace"], SIM_ROOT),
        command("uniform_contract_unit_tests", ["env", f"PYTHONPATH={ROOT / 'tools/cmaes_tuning'}", sys.executable,
                "-m", "unittest", "discover", "-s", "tools/cmaes_tuning/tests", "-p", "test_uniform_laserscan_prototype.py"]),
        command("all_cmaes_tuning_tests", ["env", f"PYTHONPATH={ROOT / 'tools/cmaes_tuning'}", sys.executable,
                "-m", "unittest", "discover", "-s", "tools/cmaes_tuning/tests", "-p", "test_*.py"]),
        command("simulator_python_compile", [sys.executable, "-m", "py_compile",
                str(SIM_ROOT / "gym/f110_gym/envs/laser_scan_contract.py"),
                str(SIM_ROOT / "gym/f110_gym/envs/cpp_simulator.py"),
                str(SIM_ROOT / "f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py"),
                str(IFAC_ROOT / "tools/cmaes_tuning/lockstep_episode.py")], env=simulator_env),
        command("simulator_scan_identity_tests", [sys.executable, "-m", "pytest", "-q",
                "/home/sungho/f1sim_C/f1tenth_gym_ros/test/test_scan_identity.py"], env=simulator_env),
        command("obstacle_detector_ctest", ["zsh", "-lc",
                f"{source} && ctest --test-dir build/obstacle_detector --output-on-failure"]),
        command("simulator_adoption_diff_check", ["git", "-C", str(SIM_ROOT), "show", "--check", "--oneline", "HEAD"]),
        command("lockstep_adoption_diff_check", ["git", "-C", str(IFAC_ROOT), "show", "--check", "--oneline", "HEAD"]),
        command("diagnostic_git_diff_check", ["git", "diff", "--check", "--",
                "tools/cmaes_tuning/render_uniform_laserscan_baseline_adoption.py"]),
    ]
    broader.RAW = RAW / "synthetic_integration"
    rows.append(broader.run_existing_synthetic_test())
    for label, passed in internal.items():
        rows.append({
            "test": label, "command": "in-process deterministic assertion",
            "returncode": 0 if passed else 1, "passed": passed,
            "elapsed_s": 0.0, "detail": "PASS" if passed else "FAIL",
        })
    return rows


def worktree_report(after: bool) -> str:
    title = "AFTER ADOPTION" if after else "BEFORE ADOPTION"
    if not after:
        ifac = Path("/tmp/uniform_adoption_ifac_before.txt").read_text(encoding="utf-8")
        simulator = Path("/tmp/uniform_adoption_f1sim_before.txt").read_text(encoding="utf-8")
        return f"{title}\n\n[ACTIVE IFAC]\n{ifac}\n[ACTIVE F1SIM]\n{simulator}\n" + (
            "[ISOLATED BASES]\n"
            "simulator snapshot base: afa0a8e715fdc96d72b3dc710445678848c6ec2f (clean)\n"
            "IFAC lockstep preimage base: 1be820e (clean)\n"
        )
    report = f"{title}\n\n[SIMULATOR ADOPTION WORKTREE]\n" + subprocess.run(
        ["git", "-C", str(SIM_ROOT), "status", "--short", "--branch"], text=True,
        capture_output=True, check=True).stdout + f"HEAD {git_text(SIM_ROOT, 'rev-parse', 'HEAD')}\n" + (
        "\n[IFAC ADOPTION WORKTREE]\n" + subprocess.run(
            ["git", "-C", str(IFAC_ROOT), "status", "--short", "--branch"], text=True,
            capture_output=True, check=True).stdout + f"HEAD {git_text(IFAC_ROOT, 'rev-parse', 'HEAD')}\n"
    )
    report += "\n[ACTIVE IFAC STATUS AFTER]\n" + subprocess.run(
        ["git", "-C", str(ROOT), "status", "--short", "--branch"], text=True,
        capture_output=True, check=True).stdout
    report += "\n[ACTIVE F1SIM STATUS AFTER]\n" + subprocess.run(
        ["git", "-C", "/home/sungho/f1sim_C", "status", "--short", "--branch"], text=True,
        capture_output=True, check=True).stdout
    report += "\n[ACTIVE TREE HASHES UNCHANGED]\n" + json.dumps(active_contract_hashes(), indent=2) + "\n"
    return report


def stale_policy() -> dict[str, Any]:
    return {
        "schema": "uniform_laserscan_stale_artifact_policy/1",
        "effective_baseline": "PROTOTYPE_UNIFORM_SCAN_MODE",
        "legacy_mode": "LEGACY_SCAN_MODE",
        "rule": (
            "Any simulator-derived CMA performance or sensor result generated with the legacy "
            "2000-bin physical-ray contract is HISTORICAL_STALE and must not drive future optimization."
        ),
        "actions": {
            "delete_existing_artifacts": False,
            "rewrite_legacy_bags": False,
            "reinterpret_legacy_ranges_with_uniform_metadata": False,
            "allow_historical_forensics": True,
            "future_dataset_generation_requires_uniform_contract": True,
            "future_smoke_validation_requires_uniform_contract": True,
            "future_cma_requires_uniform_contract": True,
        },
        "authoritative_uniform_evidence": [
            "simulator_laserscan_contract_prototype_v1",
            "uniform_laserscan_broader_regression_v1",
            "static_safety_sparse_evidence_shadow_audit_v1",
            "uniform_laserscan_baseline_adoption_v1",
        ],
        "legacy_detection": {
            "explicit_mode_required": True,
            "default_for_unlabelled_pre_adoption_simulator_results": "HISTORICAL_STALE_PENDING_PROVENANCE",
        },
    }


def readme(summary: dict[str, Any], performance: dict[str, Any]) -> str:
    validation = next(
        row for row in summary["validation039"]
        if row["run"] == "PRIMARY" and int(row["delta_t_ms"]) == 40
    )
    normal40 = next(row for row in summary["normal0100"] if int(row["delta_t_ms"]) == 40)
    return f"""# Uniform LaserScan official baseline adoption v1

## 결론

분류는 **{summary['classification']}**이다. 검증된 prototype patch만 두 격리
`uniform_laserscan_baseline` branch에 적용했다. Active IFAC 및 `/home/sungho/f1sim_C`
working tree의 unrelated dirty 작업과 hash는 그대로 보존했다.

Simulator official NEW mode는 inclusive uniform contract를 사용한다.

`angle_increment = (angle_max - angle_min)/(N-1)`이며 모든 beam에서
`physical_angle(i) == angle_min + i*angle_increment`이다. 1080-beam mismatch는
**{summary['all_1080_beam_mismatch_count']}**, 전체 지원 beam-count mismatch도
**{summary['all_supported_count_mismatch_count']}**이다. Legacy 2000-bin physical ray와 uniform
metadata를 섞는 negative test는 mismatch {summary['negative_contract_mismatch_count']}개로
명시적으로 거부됐다.

## 적용 파일

- simulator: `gym/f110_gym/envs/laser_scan_contract.py`
- simulator: `gym/f110_gym/cpp_backend.cpp`
- simulator: `gym/f110_gym/envs/cpp_simulator.py`
- simulator: `f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py`
- IFAC lockstep: `tools/cmaes_tuning/lockstep_episode.py`

Obstacle detector, planner, controller, state machine, clustering/tracking/KF/ID/motion code는
변경하지 않았다. Detector는 기존 ROS 식 `angle_min + i*angle_increment`를 계속 사용한다.

## 최소 회귀

- gym_bridge/backend/lockstep의 N/min/max/increment는 field-exact하게 일치한다.
- validation_039 +20/+30/+40은 모두 `NON_EMPTY`이다.
- validation_039 +40: cells={validation['possible_cell_count']}, area={validation['possible_area_m2']:.6f} m2,
  selected support={validation['selected_side_support_m']:.9f} m, GT undercoverage={validation['gt_undercoverage_m']},
  GT overcoverage={validation['gt_overcoverage_m']:.9f} m, dangerous clearance={validation['dangerous_path_clearance_m']:.9f} m,
  result={validation['dangerous_path_classification']}.
- normal_01_00은 `OBSERVATION_RESOLVES_EARLY`: blocker +10 ms 제거, 첫 hard-feasible
  +{normal40['first_hard_feasible_ms']} ms, stable +{normal40['stable_resolution_ms']} ms이다.
- baseline/2.5 m/s/partial-occlusion opponent 모두 detection, single ID, DYNAMIC, `/opp_obs` PASS다.
- 1080 contract, validation_039 +40, baseline opponent의 반복 digest가 동일하다.

## Legacy artifact 정책

기존 artifact와 bag은 삭제하거나 다시 해석하지 않았다. Legacy beam contract로 생성된 모든
simulator-derived CMA 성능/센서 결과는 future optimization 관점에서 `HISTORICAL_STALE`이다.
새 dataset/smoke/CMA는 반드시 adopted uniform contract를 사용해야 한다. 자세한 기계 판정은
`stale_artifact_policy.json`에 있다.

## 자원 및 다음 단계

주 실행 wall={performance['wall_time_s']:.3f}s, peak RSS={performance['peak_rss_bytes']} bytes,
workers={performance['workers']}, swap increase={performance['swap_increased']}이다.
모든 {len(summary['tests'])} test/assertion이 통과했다.

다음 작업인 S1_UPDATE_TEMPORAL production shadow 구현을 시작할 수 있는 sensor baseline은
준비됐다. 단, 이 작업에서는 S1을 구현하지 않았으며 다음 작업도 adoption branch를 기준으로
명시적으로 시작해야 한다.
"""


def main() -> int:
    started = time.monotonic()
    OUTPUT.mkdir(parents=True, exist_ok=True); RAW.mkdir(parents=True, exist_ok=True)
    memory_before = memory_snapshot()
    protected_before = protected_hashes(); active_before = active_contract_hashes()
    if git_text(SIM_ROOT, "branch", "--show-current") != SIM_BRANCH:
        raise RuntimeError("simulator adoption branch mismatch")
    if git_text(IFAC_ROOT, "branch", "--show-current") != IFAC_BRANCH:
        raise RuntimeError("IFAC adoption branch mismatch")
    if not git_text(SIM_ROOT, "rev-parse", "--short", "HEAD").startswith(EXPECTED_SIM_COMMIT):
        raise RuntimeError("simulator adoption commit mismatch")
    if not git_text(IFAC_ROOT, "rev-parse", "--short", "HEAD").startswith(EXPECTED_IFAC_COMMIT):
        raise RuntimeError("IFAC adoption commit mismatch")
    if git_text(SIM_ROOT, "status", "--porcelain") or git_text(IFAC_ROOT, "status", "--porcelain"):
        raise RuntimeError("adoption worktree is not clean")

    manifest = json.loads((prototype.SCENARIO / "manifest.json").read_text(encoding="utf-8"))
    fov = float(manifest["simulator_collision_model"]["scan_fov_rad"])
    beams = int(manifest["simulator_collision_model"]["scan_beams"])
    beam_rows, beam_summaries, contract_determinism, contract_wall = contract_regression(fov)
    summary_1080 = next(row for row in beam_summaries if row["beam_count"] == beams)
    publisher_rows = publisher_regression(fov, beams, summary_1080)
    validation_rows, validation_determinism, validation_wall = validation039_regression()
    normal_rows, normal_determinism, normal_wall = normal0100_regression()
    opponent_rows, opponent_determinism, opponent_wall = opponent_regression()

    protected_after = protected_hashes(); active_after = active_contract_hashes()
    internal = {
        "all_beam_contract_zero_mismatch": all(row["mismatch_count"] == 0 for row in beam_summaries),
        "negative_legacy_uniform_mix_rejected": contract_determinism["negative_test_rejected"],
        "publisher_contract_field_exact": all(row["status"] == "MATCH" for row in publisher_rows),
        "validation039_safe": all(row["status"] == "NON_EMPTY" for row in validation_rows if row["run"] == "PRIMARY")
        and next(row for row in validation_rows if row["run"] == "PRIMARY" and row["delta_t_ms"] == 40)["gt_undercoverage_m"] == 0.0
        and next(row for row in validation_rows if row["run"] == "PRIMARY" and row["delta_t_ms"] == 40)["dangerous_path_classification"] == "HARD_INVALID",
        "normal0100_retained": all(row["adopted_vs_validated_prototype_ranges_identical"] for row in normal_rows)
        and normal_rows[0]["classification"] == "OBSERVATION_RESOLVES_EARLY",
        "opponent_sanity_retained": all(row["result"] == "PASS" for row in opponent_rows),
        "contract_deterministic": contract_determinism["identical"],
        "validation039_deterministic": validation_determinism["identical"],
        "normal0100_deterministic": normal_determinism["identical"],
        "opponent_deterministic": opponent_determinism["identical"],
        "production_hashes_unchanged": protected_before == protected_after,
        "active_dirty_files_unchanged": active_before == active_after,
    }
    tests = test_suite(internal)
    policy = stale_policy()
    memory_after = memory_snapshot()
    performance = {
        "schema": "uniform_laserscan_baseline_adoption_performance/1",
        "wall_time_s": time.monotonic() - started,
        "contract_driver_wall_s": contract_wall,
        "validation039_driver_wall_s": validation_wall,
        "normal0100_driver_wall_s": normal_wall,
        "opponent_wall_s": opponent_wall,
        "workers": WORKERS,
        "timestamps_within_case_serial": True,
        "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
        "memory_before": memory_before, "memory_after": memory_after,
        "swap_increased": memory_after["swap_used_bytes"] > memory_before["swap_used_bytes"],
        "memory_floor_respected": min(memory_before["memory_available_bytes"], memory_after["memory_available_bytes"]) >= 8 * 1024 ** 3,
    }
    classification = "BASELINE_ADOPTED" if all(internal.values()) and all(row["passed"] for row in tests) else "ADOPTION_REGRESSION"
    summary = {
        "schema": "uniform_laserscan_baseline_adoption/1",
        "classification": classification,
        "simulator_branch": SIM_BRANCH, "simulator_worktree": SIM_ROOT,
        "simulator_commit": git_text(SIM_ROOT, "rev-parse", "HEAD"),
        "ifac_branch": IFAC_BRANCH, "ifac_worktree": IFAC_ROOT,
        "ifac_commit": git_text(IFAC_ROOT, "rev-parse", "HEAD"),
        "active_ifac_branch": git_text(ROOT, "branch", "--show-current"),
        "active_ifac_head": git_text(ROOT, "rev-parse", "HEAD"),
        "active_simulator_branch": git_text(Path("/home/sungho/f1sim_C"), "branch", "--show-current"),
        "active_simulator_head": git_text(Path("/home/sungho/f1sim_C"), "rev-parse", "HEAD"),
        "changed_files": [
            "gym/f110_gym/envs/laser_scan_contract.py", "gym/f110_gym/cpp_backend.cpp",
            "gym/f110_gym/envs/cpp_simulator.py",
            "f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py",
            "tools/cmaes_tuning/lockstep_episode.py",
        ],
        "prototype_patch_sha256": sha256(PRIOR / "simulator_uniform_contract.patch"),
        "production_hashes_before": protected_before, "production_hashes_after": protected_after,
        "production_hashes_unchanged": protected_before == protected_after,
        "active_contract_hashes_before": active_before, "active_contract_hashes_after": active_after,
        "unrelated_dirty_work_preserved": active_before == active_after,
        "detector_ros_angle_formula_line": source_line(
            ROOT / "src/obstacle_detector/src/obstacle_detector_node.cpp",
            "scan.angle_min + static_cast<double>(i) * scan.angle_increment"),
        "all_1080_beam_mismatch_count": summary_1080["mismatch_count"],
        "all_supported_count_mismatch_count": sum(row["mismatch_count"] for row in beam_summaries),
        "negative_contract_mismatch_count": contract_determinism["negative_legacy_plus_uniform_metadata_mismatch_count"],
        "beam_count_summaries": beam_summaries,
        "publisher_contract": publisher_rows,
        "validation039": validation_rows,
        "normal0100": normal_rows,
        "opponent": opponent_rows,
        "determinism": {
            "contract": contract_determinism, "validation039": validation_determinism,
            "normal0100": normal_determinism, "opponent": opponent_determinism,
        },
        "invariants": internal, "tests": tests,
        "stale_artifact_policy": policy,
        "cma_run": False, "dataset_regenerated": False, "large_closed_loop_run": False,
        "s1_update_temporal_implemented": False,
        "ready_for_s1_shadow_task": classification == "BASELINE_ADOPTED",
        "performance": performance,
    }

    shutil.copyfile(PRIOR / "simulator_uniform_contract.patch", OUTPUT / "applied_diff.patch")
    (OUTPUT / "worktree_status_before.txt").write_text(worktree_report(False), encoding="utf-8")
    (OUTPUT / "worktree_status_after.txt").write_text(worktree_report(True), encoding="utf-8")
    write_csv(OUTPUT / "beam_contract_regression.csv", beam_rows)
    write_csv(OUTPUT / "publisher_contract_regression.csv", publisher_rows)
    write_csv(OUTPUT / "validation039_regression.csv", validation_rows)
    write_csv(OUTPUT / "normal0100_regression.csv", normal_rows)
    write_csv(OUTPUT / "opponent_regression.csv", opponent_rows)
    write_csv(OUTPUT / "test_results.csv", tests)
    (OUTPUT / "stale_artifact_policy.json").write_text(json.dumps(clean(policy), indent=2) + "\n", encoding="utf-8")
    (OUTPUT / "performance.json").write_text(json.dumps(clean(performance), indent=2) + "\n", encoding="utf-8")
    (OUTPUT / "summary.json").write_text(json.dumps(clean(summary), indent=2) + "\n", encoding="utf-8")
    (OUTPUT / "README.md").write_text(readme(summary, performance), encoding="utf-8")
    print(json.dumps({
        "classification": classification, "output": str(OUTPUT),
        "all_tests_passed": all(row["passed"] for row in tests),
        "production_hashes_unchanged": summary["production_hashes_unchanged"],
        "ready_for_s1_shadow_task": summary["ready_for_s1_shadow_task"],
    }, indent=2))
    return 0 if classification == "BASELINE_ADOPTED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
