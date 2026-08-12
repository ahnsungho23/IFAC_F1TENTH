"""Episode-level parallel diagnostic runner with a strict determinism gate."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time
from typing import Any

import psutil

from .schemas import atomic_write_json, sha256_file


EXACT_LOCKSTEP_FIELDS = (
    "physics_state_sequence_hash",
    "scan_hash_sequence_hash",
    "static_geometry_sequence_hash",
    "avoid_waypoints_hash_sequence_hash",
    "controller_command_sequence_hash",
    "trajectory_hash",
    "committed_target_d",
    "selected_side",
    "entry_transition_scale",
    "exit_transition_scale",
    "effective_entry_transition_scale",
    "effective_exit_transition_scale",
    "fitness",
    "collision",
    "off_track",
    "planner_failure",
    "scenario_success",
)

EXACT_EPISODE_FIELDS = (
    "classification",
    "failure",
    "metrics",
    "performance_cost",
)

DDS_ERROR_MARKERS = (
    "RTPS_PARTICIPANT Error",
    "Failed to find a free participant index",
    "ROS_DOMAIN_ID",
    "domain id",
    "Error creating socket",
)


def compare_deterministic_results(
    reference_lockstep: dict[str, Any],
    reference_episode: dict[str, Any],
    candidate_lockstep: dict[str, Any],
    candidate_episode: dict[str, Any],
) -> dict[str, Any]:
    mismatches = []
    for field in EXACT_LOCKSTEP_FIELDS:
        if reference_lockstep.get(field) != candidate_lockstep.get(field):
            mismatches.append({
                "document": "lockstep_result", "field": field,
                "reference": reference_lockstep.get(field),
                "candidate": candidate_lockstep.get(field),
            })
    for field in EXACT_EPISODE_FIELDS:
        if reference_episode.get(field) != candidate_episode.get(field):
            mismatches.append({
                "document": "episode_result", "field": field,
                "reference": reference_episode.get(field),
                "candidate": candidate_episode.get(field),
            })
    return {
        "accepted": not mismatches,
        "mismatch_count": len(mismatches),
        "mismatches": mismatches,
    }


def select_worker_count(
    stages: list[dict[str, Any]], minimum_throughput_improvement: float = 0.05
) -> tuple[int, str]:
    """Select the largest stable worker count with material incremental throughput."""

    if not stages:
        return 1, "no benchmark stages"
    selected = int(stages[0]["workers"])
    previous_throughput = float(stages[0]["episodes_per_min"])
    reason = "serial fallback"
    for stage in stages[1:]:
        stable = (
            int(stage["deterministic_mismatch_count"]) == 0
            and int(stage["infrastructure_failure_count"]) == 0
            and int(stage["process_contamination_count"]) == 0
            and int(stage["dds_collision_count"]) == 0
            and not bool(stage["swap_increase"])
        )
        throughput = float(stage["episodes_per_min"])
        improvement = (
            (throughput - previous_throughput) / previous_throughput
            if previous_throughput > 0.0 else math.inf)
        if not stable:
            reason = f"workers={stage['workers']} rejected by stability gate"
            break
        if improvement < minimum_throughput_improvement:
            reason = (
                f"workers={stage['workers']} incremental throughput improvement "
                f"{100.0 * improvement:.1f}% < "
                f"{100.0 * minimum_throughput_improvement:.1f}%")
            break
        selected = int(stage["workers"])
        previous_throughput = throughput
        reason = f"workers={selected} is the largest stable, materially faster stage"
    return selected, reason


@dataclass(frozen=True)
class DiagnosticJob:
    index: int
    scenario_id: str
    scenario_manifest: Path
    candidate: Path
    simulator_seed: int


class ResourceMonitor:
    def __init__(self) -> None:
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.cpu_samples: list[float] = []
        self.peak_process_rss_bytes = 0
        self.peak_system_used_bytes = 0
        self.swap_start_bytes = int(psutil.swap_memory().used)
        self.swap_peak_bytes = self.swap_start_bytes

    def start(self) -> None:
        psutil.cpu_percent(interval=None)
        self._thread = threading.Thread(target=self._sample, daemon=True)
        self._thread.start()

    def _sample(self) -> None:
        root = psutil.Process(os.getpid())
        while not self._stop.wait(0.25):
            self.cpu_samples.append(float(psutil.cpu_percent(interval=None)))
            rss = 0
            try:
                processes = [root, *root.children(recursive=True)]
            except psutil.Error:
                processes = [root]
            for process in processes:
                try:
                    rss += int(process.memory_info().rss)
                except psutil.Error:
                    continue
            self.peak_process_rss_bytes = max(self.peak_process_rss_bytes, rss)
            memory = psutil.virtual_memory()
            self.peak_system_used_bytes = max(
                self.peak_system_used_bytes, int(memory.total - memory.available))
            self.swap_peak_bytes = max(self.swap_peak_bytes, int(psutil.swap_memory().used))

    def stop(self) -> dict[str, Any]:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        return {
            "average_cpu_utilization_percent": (
                sum(self.cpu_samples) / len(self.cpu_samples) if self.cpu_samples else 0.0),
            "peak_process_rss_bytes": self.peak_process_rss_bytes,
            "peak_system_used_bytes": self.peak_system_used_bytes,
            "swap_start_bytes": self.swap_start_bytes,
            "swap_peak_bytes": self.swap_peak_bytes,
            "swap_increase": self.swap_peak_bytes > self.swap_start_bytes + 4 * 1024 * 1024,
        }


class ParallelEpisodeBenchmark:
    def __init__(
        self,
        workspace: str | Path,
        config: str | Path,
        output: str | Path,
        domain_start: int = 210,
        scan_noise_std: float = 0.01,
        maximum_duration: float = 0.50,
        post_obstacle_distance: float = 3.0,
        retries: int = 1,
    ) -> None:
        self.workspace = Path(workspace).resolve()
        self.config = Path(config).resolve()
        self.output = Path(output).resolve()
        self.domain_start = int(domain_start)
        self.scan_noise_std = float(scan_noise_std)
        self.maximum_duration = float(maximum_duration)
        self.post_obstacle_distance = float(post_obstacle_distance)
        self.retries = int(retries)
        self.tool = self.workspace / "tools" / "cmaes_tuning" / "lockstep_episode.py"

    def _historical_infrastructure_summary(self) -> dict[str, int]:
        """Count invalid attempts without including them as scientific results."""

        performance_root = self.output / "performance"
        failure_files = list(performance_root.rglob("infrastructure_failure.json"))
        dds_markers = 0
        for failure_file in failure_files:
            try:
                failure = json.loads(failure_file.read_text(encoding="utf-8"))
                dds_markers += int(failure.get("dds_collision_markers", 0))
            except (OSError, ValueError, json.JSONDecodeError):
                continue
        retry_count = 0
        for job_directory in performance_root.glob("*/worker_*/job_*"):
            attempt_count = sum(1 for _ in job_directory.glob("attempt_*"))
            retry_count += max(0, attempt_count - 1)
        return {
            "excluded_infrastructure_failure_attempt_count": len(failure_files),
            "excluded_infrastructure_retry_count": retry_count,
            "excluded_dds_error_marker_count": dds_markers,
        }

    def _domain_preflight(self) -> dict[str, Any]:
        requested = set(range(self.domain_start, self.domain_start + 8))
        conflicts = []
        unreadable_processes = 0
        for process in psutil.process_iter(["pid", "name"]):
            try:
                domain_value = process.environ().get("ROS_DOMAIN_ID")
            except (psutil.AccessDenied, psutil.NoSuchProcess, OSError):
                unreadable_processes += 1
                continue
            if domain_value is None:
                continue
            try:
                domain_id = int(domain_value)
            except ValueError:
                continue
            if domain_id in requested:
                conflicts.append({
                    "pid": int(process.pid),
                    "process": str(process.info.get("name") or ""),
                    "ros_domain_id": domain_id,
                })
        return {
            "checked_domain_ids": sorted(requested),
            "parent_ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
            "explicit_process_conflicts": conflicts,
            "unreadable_process_count": unreadable_processes,
        }

    def _cache_key(self, job: DiagnosticJob) -> dict[str, Any]:
        return {
            "scenario_id": job.scenario_id,
            "scenario_sha256": sha256_file(job.scenario_manifest),
            "candidate_sha256": sha256_file(job.candidate),
            "config_sha256": sha256_file(self.config),
            "simulator_seed": job.simulator_seed,
            "scan_noise_std_m": self.scan_noise_std,
            "maximum_duration_s": self.maximum_duration,
            "post_obstacle_distance_m": self.post_obstacle_distance,
        }

    def _valid_attempt(
        self, attempt: Path, cache_key: dict[str, Any]
    ) -> tuple[dict[str, Any], dict[str, Any]] | None:
        manifest_path = attempt / "diagnostic_job.json"
        lockstep_path = attempt / "lockstep_result.json"
        episode_path = attempt / "episode_result.json"
        if not all(path.is_file() for path in (manifest_path, lockstep_path, episode_path)):
            return None
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            lockstep = json.loads(lockstep_path.read_text(encoding="utf-8"))
            episode = json.loads(episode_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return None
        if manifest.get("cache_key") != cache_key:
            return None
        if not lockstep.get("valid") or not episode.get("valid"):
            return None
        return lockstep, episode

    @staticmethod
    def _kill_process_group(process: subprocess.Popen[str]) -> None:
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def _run_job(
        self,
        job: DiagnosticJob,
        stage_directory: Path,
        worker_pool: queue.Queue[tuple[int, int]],
    ) -> dict[str, Any]:
        worker_index, domain_id = worker_pool.get()
        try:
            worker_directory = stage_directory / f"worker_{worker_index:02d}"
            job_directory = worker_directory / f"job_{job.index:02d}"
            cache_key = self._cache_key(job)
            for attempt in sorted(job_directory.glob("attempt_*")):
                cached = self._valid_attempt(attempt, cache_key)
                if cached is not None:
                    return {
                        "job_index": job.index, "worker_index": worker_index,
                        "domain_id": domain_id, "attempt": str(attempt),
                        "lockstep": cached[0], "episode": cached[1],
                        "cached": True, "infrastructure_failures": 0,
                        "infrastructure_retries": 0, "dds_collisions": 0,
                    }

            infrastructure_failures = 0
            dds_collisions = 0
            existing_indices = []
            for path in job_directory.glob("attempt_*"):
                try:
                    existing_indices.append(int(path.name.rsplit("_", 1)[1]))
                except (IndexError, ValueError):
                    continue
            first_attempt_index = max(existing_indices, default=-1) + 1
            for retry_index in range(self.retries + 1):
                attempt_index = first_attempt_index + retry_index
                attempt = job_directory / f"attempt_{attempt_index:02d}"
                attempt.mkdir(parents=True, exist_ok=True)
                worker_tmp = worker_directory / "tmp"
                worker_cache = worker_directory / "cache"
                worker_ros = worker_directory / "ros_home"
                for directory in (worker_tmp, worker_cache, worker_ros):
                    directory.mkdir(parents=True, exist_ok=True)
                atomic_write_json(attempt / "diagnostic_job.json", {
                    "schema": "tracking_parallel_job/1",
                    "cache_key": cache_key,
                    "worker_index": worker_index,
                    "ros_domain_id": domain_id,
                    "process_group_isolated": True,
                })
                environment = dict(os.environ)
                environment.update({
                    "ROS_DOMAIN_ID": str(domain_id),
                    "ROS_HOME": str(worker_ros),
                    "TMPDIR": str(worker_tmp),
                    "XDG_CACHE_HOME": str(worker_cache),
                    "MPLCONFIGDIR": str(worker_cache / "matplotlib"),
                    "PYTHONPYCACHEPREFIX": str(worker_cache / "pycache"),
                })
                command = [
                    sys.executable, str(self.tool),
                    "--workspace", str(self.workspace),
                    "--config", str(self.config),
                    "--candidate", str(job.candidate),
                    "--scenario", str(job.scenario_manifest),
                    "--output", str(attempt),
                    "--simulator-seed", str(job.simulator_seed),
                    "--scan-noise-std", str(self.scan_noise_std),
                    "--post-obstacle-distance", str(self.post_obstacle_distance),
                    "--maximum-duration", str(self.maximum_duration),
                    "--dds-transport", "UDPv4",
                ]
                started = time.monotonic()
                process = subprocess.Popen(
                    command, cwd=self.workspace, env=environment,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, start_new_session=True)
                try:
                    stdout, _ = process.communicate(timeout=240.0)
                except subprocess.TimeoutExpired:
                    self._kill_process_group(process)
                    stdout = "episode process timeout\n"
                elapsed = time.monotonic() - started
                (attempt / "diagnostic_stdout.log").write_text(stdout, encoding="utf-8")
                dds_hits = sum(marker.lower() in stdout.lower() for marker in DDS_ERROR_MARKERS)
                dds_collisions += dds_hits
                cached = self._valid_attempt(attempt, cache_key)
                if process.returncode == 0 and cached is not None and dds_hits == 0:
                    return {
                        "job_index": job.index, "worker_index": worker_index,
                        "domain_id": domain_id, "attempt": str(attempt),
                        "lockstep": cached[0], "episode": cached[1],
                        "cached": False, "wall_time_sec": elapsed,
                        "infrastructure_failures": infrastructure_failures,
                        "infrastructure_retries": retry_index,
                        "dds_collisions": dds_collisions,
                    }
                infrastructure_failures += 1
                atomic_write_json(attempt / "infrastructure_failure.json", {
                    "returncode": process.returncode,
                    "dds_collision_markers": dds_hits,
                    "elapsed_wall_sec": elapsed,
                })
            return {
                "job_index": job.index,
                "worker_index": worker_index,
                "domain_id": domain_id,
                "error": (
                    f"diagnostic job {job.index} failed after "
                    f"{self.retries + 1} attempts"),
                "infrastructure_failures": infrastructure_failures,
                "infrastructure_retries": self.retries,
                "dds_collisions": dds_collisions,
            }
        finally:
            worker_pool.put((worker_index, domain_id))

    def run_stage(
        self,
        name: str,
        workers: int,
        jobs: list[DiagnosticJob],
        reference: tuple[dict[str, Any], dict[str, Any]] | None = None,
    ) -> tuple[dict[str, Any], list[dict[str, Any]]]:
        if workers < 1 or workers > 8:
            raise ValueError("workers must be in [1, 8]")
        if self.domain_start + workers - 1 > 230:
            raise ValueError("ROS domain pool must remain within [0, 230]")
        stage_directory = self.output / "performance" / name
        worker_pool: queue.Queue[tuple[int, int]] = queue.Queue()
        for worker_index in range(workers):
            worker_pool.put((worker_index, self.domain_start + worker_index))

        monitor = ResourceMonitor()
        monitor.start()
        started = time.monotonic()
        results: dict[int, dict[str, Any]] = {}
        failure_count = 0
        with ThreadPoolExecutor(max_workers=workers) as executor:
            future_map = {
                executor.submit(self._run_job, job, stage_directory, worker_pool): job.index
                for job in jobs
            }
            for future in as_completed(future_map):
                index = future_map[future]
                try:
                    results[index] = future.result()
                except Exception as error:  # preserve the stage report before failing closed
                    failure_count += 1
                    results[index] = {"job_index": index, "error": str(error)}
        wall_time = time.monotonic() - started
        resources = monitor.stop()
        ordered = [results[index] for index in sorted(results)]
        valid_results = [result for result in ordered if "lockstep" in result]
        mismatch_count = 0
        comparisons = []
        if reference is not None:
            for result in valid_results:
                comparison = compare_deterministic_results(
                    reference[0], reference[1], result["lockstep"], result["episode"])
                comparisons.append({"job_index": result["job_index"], **comparison})
                mismatch_count += int(comparison["mismatch_count"])
        process_contamination = 0
        try:
            children = psutil.Process(os.getpid()).children(recursive=True)
            process_contamination = sum(child.is_running() for child in children)
        except psutil.Error:
            process_contamination = 0
        infrastructure_count = failure_count + sum(
            int(result.get("infrastructure_failures", 0)) for result in ordered)
        infrastructure_retries = sum(
            int(result.get("infrastructure_retries", 0)) for result in ordered)
        dds_count = sum(int(result.get("dds_collisions", 0)) for result in ordered)
        stage = {
            "name": name,
            "workers": workers,
            "job_count": len(jobs),
            "valid_episode_count": len(valid_results),
            "wall_clock_sec": wall_time,
            "episodes_per_min": len(valid_results) * 60.0 / max(wall_time, 1.0e-9),
            "deterministic_mismatch_count": mismatch_count,
            "deterministic_comparisons": comparisons,
            "infrastructure_failure_count": infrastructure_count,
            "infrastructure_retry_count": infrastructure_retries,
            "process_contamination_count": process_contamination,
            "dds_collision_count": dds_count,
            **resources,
        }
        atomic_write_json(stage_directory / "stage_summary.json", stage)
        return stage, valid_results

    def run(
        self,
        candidate: str | Path,
        scenario: str | Path,
        simulator_seed: int,
        benchmark_job_count: int = 8,
    ) -> dict[str, Any]:
        candidate = Path(candidate).resolve()
        scenario = Path(scenario).resolve()
        scenario_id = json.loads(scenario.read_text(encoding="utf-8"))["scenario_id"]
        report_path = self.output / "performance.json"
        if report_path.is_file():
            report = json.loads(report_path.read_text(encoding="utf-8"))
            cached_job = DiagnosticJob(
                0, scenario_id, scenario, candidate, int(simulator_seed))
            if (
                report.get("complete") is True
                and report.get("source_key") == self._cache_key(cached_job)
            ):
                report.update(self._historical_infrastructure_summary())
                report["domain_preflight"] = self._domain_preflight()
                atomic_write_json(report_path, report)
                return report

        serial_job = DiagnosticJob(0, scenario_id, scenario, candidate, int(simulator_seed))
        serial_stage, serial_results = self.run_stage(
            "determinism_gate_serial", 1, [serial_job])
        if len(serial_results) != 1:
            report = {
                "schema": "tracking_parallel_performance/1",
                "complete": False,
                "source_key": self._cache_key(serial_job),
                "cpu": "Intel Core i7-14650HX",
                "physical_cores": psutil.cpu_count(logical=False),
                "logical_cpus": psutil.cpu_count(logical=True),
                "domain_pool": [self.domain_start, self.domain_start + 7],
                "domain_upper_bound": 230,
                "determinism_gate_accepted": False,
                "determinism_gate": {"serial": serial_stage},
                "benchmarks": [],
                "selected_workers": 1,
                "selection_reason": (
                    "serial reference infrastructure failure; benchmark not run"),
                "actual_speedup": 1.0,
                "production_algorithms_changed": False,
                "cma_executed": False,
                **self._historical_infrastructure_summary(),
                "domain_preflight": self._domain_preflight(),
            }
            atomic_write_json(report_path, report)
            return report
        reference = (serial_results[0]["lockstep"], serial_results[0]["episode"])
        gate_jobs = [
            DiagnosticJob(index, scenario_id, scenario, candidate, int(simulator_seed))
            for index in range(2)
        ]
        gate_stage, gate_results = self.run_stage(
            "determinism_gate_workers_2", 2, gate_jobs, reference)
        gate_accepted = (
            len(gate_results) == 2
            and gate_stage["deterministic_mismatch_count"] == 0
            and gate_stage["infrastructure_failure_count"] == 0
            and gate_stage["process_contamination_count"] == 0
            and gate_stage["dds_collision_count"] == 0)

        benchmark_stages = []
        selected_workers = 1
        selection_reason = "workers=2 determinism gate failed; serial fallback"
        jobs = [
            DiagnosticJob(index, scenario_id, scenario, candidate, int(simulator_seed))
            for index in range(benchmark_job_count)
        ]
        if gate_accepted:
            for workers in (1, 2, 4, 8):
                stage, _ = self.run_stage(
                    f"benchmark_workers_{workers}", workers, jobs, reference)
                benchmark_stages.append(stage)
            selected_workers, selection_reason = select_worker_count(benchmark_stages)
        else:
            stage, _ = self.run_stage(
                "benchmark_workers_1", 1, jobs, reference)
            benchmark_stages.append(stage)
        report = {
            "schema": "tracking_parallel_performance/1",
            "complete": True,
            "source_key": self._cache_key(serial_job),
            "cpu": "Intel Core i7-14650HX",
            "physical_cores": psutil.cpu_count(logical=False),
            "logical_cpus": psutil.cpu_count(logical=True),
            "domain_pool": [self.domain_start, self.domain_start + 7],
            "domain_upper_bound": 230,
            "determinism_gate_accepted": gate_accepted,
            "determinism_gate": {
                "serial": serial_stage,
                "workers_2": gate_stage,
            },
            "benchmarks": benchmark_stages,
            "selected_workers": selected_workers,
            "selection_reason": selection_reason,
            "actual_speedup": (
                float(next(
                    stage["episodes_per_min"] for stage in benchmark_stages
                    if int(stage["workers"]) == selected_workers))
                / float(benchmark_stages[0]["episodes_per_min"])
                if benchmark_stages else 1.0),
            "production_algorithms_changed": False,
            "cma_executed": False,
            **self._historical_infrastructure_summary(),
            "domain_preflight": self._domain_preflight(),
        }
        atomic_write_json(report_path, report)
        return report
