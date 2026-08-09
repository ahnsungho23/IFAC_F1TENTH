"""Clean-process ROS episode runner with retryable infrastructure failures."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shlex
import signal
import math
import subprocess
import time
from typing import Any

from .configuration import resolve_workspace_path
from .evaluator import evaluate_episode
from .schemas import atomic_write_json, sha256_file


class ExperimentInfrastructureError(RuntimeError):
    pass


@dataclass
class ManagedProcess:
    name: str
    process: subprocess.Popen
    log_stream: Any
    command: list[str]


class ProcessSupervisor:
    def __init__(self, directory: Path, environment: dict[str, str], shell_prefix: str):
        self.directory = directory
        self.environment = environment
        self.shell_prefix = shell_prefix
        self.processes: list[ManagedProcess] = []

    def start(self, name: str, command: list[str]) -> ManagedProcess:
        log_path = self.directory / "logs" / f"{name}.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        stream = log_path.open("w", encoding="utf-8")
        shell_command = f"{self.shell_prefix} && exec {shlex.join(command)}"
        process = subprocess.Popen(
            ["bash", "-lc", shell_command],
            cwd=self.environment["CMA_WORKSPACE_ROOT"],
            env=self.environment,
            stdout=stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )
        managed = ManagedProcess(name=name, process=process, log_stream=stream, command=command)
        self.processes.append(managed)
        return managed

    def run(self, command: list[str], timeout: float) -> subprocess.CompletedProcess:
        shell_command = f"{self.shell_prefix} && exec {shlex.join(command)}"
        return subprocess.run(
            ["bash", "-lc", shell_command],
            cwd=self.environment["CMA_WORKSPACE_ROOT"],
            env=self.environment,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )

    def early_exits(self, ignore: set[str] | None = None) -> list[str]:
        ignored = ignore or set()
        return [
            f"process_exit:{managed.name}:{managed.process.returncode}"
            for managed in self.processes
            if managed.name not in ignored and managed.process.poll() is not None
        ]

    @staticmethod
    def _signal_group(managed: ManagedProcess, signal_number: int) -> None:
        if managed.process.poll() is None:
            try:
                os.killpg(managed.process.pid, signal_number)
            except ProcessLookupError:
                pass

    def stop_one(self, name: str, sigint_timeout: float, sigterm_timeout: float) -> bool:
        matches = [managed for managed in self.processes if managed.name == name]
        if not matches:
            return True
        managed = matches[-1]
        if managed.process.poll() is None:
            self._signal_group(managed, signal.SIGINT)
            try:
                managed.process.wait(timeout=sigint_timeout)
            except subprocess.TimeoutExpired:
                self._signal_group(managed, signal.SIGTERM)
                try:
                    managed.process.wait(timeout=sigterm_timeout)
                except subprocess.TimeoutExpired:
                    self._signal_group(managed, signal.SIGKILL)
                    try:
                        managed.process.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        return False
        managed.log_stream.close()
        return managed.process.poll() is not None

    def stop_all(self, sigint_timeout: float, sigterm_timeout: float) -> list[str]:
        failures = []
        for managed in reversed(self.processes):
            if not self.stop_one(managed.name, sigint_timeout, sigterm_timeout):
                failures.append(f"process_cleanup_failure:{managed.name}")
        return failures


class EpisodeRunner:
    def __init__(self, config: dict[str, Any], workspace_root: str | Path):
        self.config = config
        self.workspace_root = Path(workspace_root).resolve()

    def _shell_prefix(self) -> str:
        paths = self.config["paths"]
        ros = self.config["ros"]
        setups = [
            Path(ros["ros_setup"]),
            resolve_workspace_path(ros["workspace_setup"], self.workspace_root),
            Path(ros["simulator_setup"]),
        ]
        missing = [str(path) for path in setups if not path.is_file()]
        if missing:
            raise ExperimentInfrastructureError(f"missing setup files: {missing}")
        return " && ".join(f"source {shlex.quote(str(path))}" for path in setups)

    @staticmethod
    def _transition(status: dict[str, Any], state: str) -> None:
        status["state"] = state
        status["transitions"].append(
            {
                "state": state,
                "utc": datetime.now(timezone.utc).isoformat(),
                "monotonic_s": time.monotonic(),
            }
        )

    def _environment(self, episode_directory: Path, domain_id: int) -> dict[str, str]:
        environment = dict(os.environ)
        environment.update(
            {
                "ROS_DOMAIN_ID": str(domain_id),
                "ROS_LOG_DIR": str((episode_directory / "ros_logs").resolve()),
                "CMA_WORKSPACE_ROOT": str(self.workspace_root),
                "F1_MAP": str(self.config["experiment"]["map_name"]),
            }
        )
        (episode_directory / "ros_logs").mkdir(parents=True, exist_ok=True)
        return environment

    def _wait_for_topics(self, supervisor: ProcessSupervisor) -> list[str]:
        required = set(self.config["ros"]["required_topics_before_control"])
        deadline = time.monotonic() + float(self.config["runner"]["ready_timeout_sec"])
        last_topics: set[str] = set()
        while time.monotonic() < deadline:
            early = supervisor.early_exits()
            if early:
                return early
            result = supervisor.run(
                ["ros2", "topic", "list", "--no-daemon", "--spin-time", "0.3"],
                timeout=float(self.config["runner"]["topic_probe_timeout_sec"]),
            )
            if result.returncode == 0:
                last_topics = {line.strip() for line in result.stdout.splitlines() if line.strip()}
                if required.issubset(last_topics):
                    return []
            time.sleep(float(self.config["runner"]["poll_period_sec"]))
        return [f"required_topics_missing:{sorted(required - last_topics)}"]

    def run_once(
        self,
        candidate_yaml: str | Path,
        scenario_manifest_path: str | Path,
        episode_directory: str | Path,
        domain_id: int,
    ) -> dict[str, Any]:
        directory = Path(episode_directory).resolve()
        directory.mkdir(parents=True, exist_ok=True)
        candidate_path = Path(candidate_yaml).resolve()
        manifest_path = Path(scenario_manifest_path).resolve()
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        status: dict[str, Any] = {
            "schema": "cmaes_runner_status/1",
            "state": "",
            "transitions": [],
            "domain_id": domain_id,
            "scenario_id": manifest["scenario_id"],
            "candidate_yaml": str(candidate_path),
            "candidate_sha256": sha256_file(candidate_path),
            "infrastructure_failures": [],
            "process_commands": {},
        }
        status_path = directory / "runner_status.json"
        self._transition(status, "PREPARE")
        expected_hashes = {
            Path(manifest["clean_map_yaml"]): manifest["clean_map_yaml_sha256"],
            Path(manifest["clean_map_image"]): manifest["clean_map_image_sha256"],
            Path(manifest["baked_map_yaml"]): manifest["baked_map_yaml_sha256"],
            Path(manifest["baked_map_image"]): manifest["baked_map_image_sha256"],
            Path(manifest["waypoint_file"]): manifest["waypoint_sha256"],
        }
        for path, expected in expected_hashes.items():
            if not path.is_file() or sha256_file(path) != expected:
                status["infrastructure_failures"].append(f"hash_mismatch:{path}")
        if status["infrastructure_failures"]:
            atomic_write_json(status_path, status)
            return {"valid": False, "runner_status": status}

        environment = self._environment(directory, domain_id)
        supervisor = ProcessSupervisor(directory, environment, self._shell_prefix())
        controller = self.config["controller"]
        paths = self.config["paths"]
        baked_stem = str(Path(manifest["baked_map_yaml"]).with_suffix(""))
        output_prefix = "rollout"
        lap_summary = directory / f"{output_prefix}_summary.json"
        bag_directory = directory / "bag"
        commands = {
            "simulator": [
                "ros2", "launch", "f1tenth_gym_ros", "obstacle_sim_launch.py",
                f"map_path:={baked_stem}", "map_img_ext:=.png",
                f"sx:={manifest['spawn_pose']['x']}",
                f"sy:={manifest['spawn_pose']['y']}",
                f"stheta:={manifest['spawn_pose']['yaw']}",
                "num_agent:=1", "rviz:=false",
            ],
            "global_planning": [
                "ros2", "launch", "global_planning", "global_planning.launch.py",
                f"map_name:={manifest['map_name']}",
            ],
            "mcl": [
                "ros2", "launch", "particle_filter_cpp", "mcl_launch.py",
                "mod:=sim", f"map_name:={manifest['map_name']}", "use_rviz:=false",
                "start_map_server:=true", "use_sim_time:=false",
            ],
            "local_planning": [
                "ros2", "launch", "local_planning", "local_planning.launch.py",
                f"params_file:={candidate_path}",
                f"reference_map:={manifest['clean_map_yaml']}",
                "simulator:=true", "use_sim_time:=false",
            ],
            "state_machine": [
                "ros2", "launch", "state_machine", "state_machine.launch.py",
            ],
            "lap_referee": [
                "ros2", "launch", "lap_referee", "lap_referee.launch.py",
                f"params_file:={resolve_workspace_path(paths['lap_referee_params'], self.workspace_root)}",
                f"waypoints_csv:={resolve_workspace_path(paths['waypoint_csv'], self.workspace_root)}",
                f"output_dir:={directory}", f"output_prefix:={output_prefix}",
            ],
            "recorder": [
                "ros2", "bag", "record", "--storage", "sqlite3",
                "--disable-keyboard-controls", "--output", str(bag_directory), "--topics",
                *self.config["ros"]["recorded_topics"],
            ],
            "controller": [
                "ros2", "launch", "f1tenth_control", "control_sim.launch.py",
                f"max_speed:={controller['max_speed_mps']}",
                f"min_speed:={controller['min_speed_mps']}",
                f"max_lateral_accel:={controller['max_lateral_accel_mps2']}",
                f"base_max_accel:={controller['base_max_accel_mps2']}",
                f"base_max_decel:={controller['base_max_decel_mps2']}",
                f"launch_boost_enable:={'true' if controller['launch_boost_enable'] else 'false'}",
                f"deadzone_floor_speed:={controller['deadzone_floor_speed_mps']}",
            ],
        }
        status["process_commands"] = commands
        try:
            self._transition(status, "LAUNCH")
            for name in (
                "simulator", "global_planning", "mcl", "local_planning", "state_machine",
            ):
                supervisor.start(name, commands[name])
                time.sleep(0.25)

            self._transition(status, "WAIT_READY")
            status["infrastructure_failures"].extend(self._wait_for_topics(supervisor))
            if not status["infrastructure_failures"]:
                spawn_yaw = float(manifest["spawn_pose"]["yaw"])
                initial_pose = {
                    "header": {"frame_id": "map"},
                    "pose": {
                        "pose": {
                            "position": {
                                "x": float(manifest["spawn_pose"]["x"]),
                                "y": float(manifest["spawn_pose"]["y"]),
                                "z": 0.0,
                            },
                            "orientation": {
                                "x": 0.0,
                                "y": 0.0,
                                "z": math.sin(0.5 * spawn_yaw),
                                "w": math.cos(0.5 * spawn_yaw),
                            },
                        },
                        "covariance": [0.0] * 36,
                    },
                }
                initial_result = supervisor.run(
                    [
                        "ros2", "topic", "pub", "--once", "/initialpose",
                        "geometry_msgs/msg/PoseWithCovarianceStamped",
                        json.dumps(initial_pose, separators=(",", ":")),
                    ],
                    timeout=float(self.config["runner"]["topic_probe_timeout_sec"]),
                )
                status["initial_pose_publish"] = {
                    "returncode": initial_result.returncode,
                    "stdout": initial_result.stdout,
                    "stderr": initial_result.stderr,
                }
                if initial_result.returncode != 0:
                    status["infrastructure_failures"].append("initial_pose_publish_failed")
                time.sleep(1.0)
            if not status["infrastructure_failures"]:
                # lap_referee's no-start timer begins at process startup, so
                # start both referee and recorder only after the slow MCL/map
                # readiness phase has completed.
                supervisor.start("lap_referee", commands["lap_referee"])
                supervisor.start("recorder", commands["recorder"])
                time.sleep(1.0)
            if not status["infrastructure_failures"]:
                supervisor.start("controller", commands["controller"])
                drive_deadline = time.monotonic() + 10.0
                drive_seen = False
                while time.monotonic() < drive_deadline:
                    result = supervisor.run(
                        ["ros2", "topic", "list", "--no-daemon", "--spin-time", "0.2"],
                        timeout=float(self.config["runner"]["topic_probe_timeout_sec"]),
                    )
                    topics = set(result.stdout.splitlines()) if result.returncode == 0 else set()
                    if {"/drive", "/drive_autonomous"}.issubset(topics):
                        drive_seen = True
                        break
                    time.sleep(0.2)
                if not drive_seen:
                    status["infrastructure_failures"].append("controller_topics_missing")
                else:
                    # Do not run synchronous ROS CLI parameter probes here: the
                    # controller is already live, and a discovery timeout would
                    # change how far the car travels before RUN. The exact launch
                    # arguments and full candidate YAML are persisted above.
                    status["fixed_controller_configuration"] = dict(controller)

            self._transition(status, "RUN")
            deadline = time.monotonic() + float(self.config["runner"]["episode_timeout_sec"])
            while not status["infrastructure_failures"] and time.monotonic() < deadline:
                if lap_summary.is_file():
                    try:
                        json.loads(lap_summary.read_text(encoding="utf-8"))
                        break
                    except json.JSONDecodeError:
                        pass
                early = supervisor.early_exits(ignore={"lap_referee"})
                if early:
                    status["infrastructure_failures"].extend(early)
                    break
                time.sleep(float(self.config["runner"]["poll_period_sec"]))
            if not lap_summary.is_file() and not status["infrastructure_failures"]:
                status["infrastructure_failures"].append("episode_summary_timeout")

            self._transition(status, "TERMINATE")
            sigint = float(self.config["runner"]["process_sigint_timeout_sec"])
            sigterm = float(self.config["runner"]["process_sigterm_timeout_sec"])
            status["infrastructure_failures"].extend(supervisor.stop_all(sigint, sigterm))

            self._transition(status, "COLLECT")
            if not (bag_directory / "metadata.yaml").is_file():
                status["infrastructure_failures"].append("recorder_failure:metadata_missing")
            atomic_write_json(status_path, status)

            self._transition(status, "EVALUATE")
            atomic_write_json(status_path, status)
            result = evaluate_episode(
                bag_directory,
                manifest_path,
                lap_summary,
                status_path,
                self.config,
                directory / "episode_result.json",
            )
            self._transition(status, "CLEANUP")
            atomic_write_json(status_path, status)
            return result
        except (OSError, subprocess.SubprocessError, ExperimentInfrastructureError) as error:
            status["infrastructure_failures"].append(f"runner_exception:{type(error).__name__}:{error}")
            self._transition(status, "TERMINATE")
            status["infrastructure_failures"].extend(
                supervisor.stop_all(
                    float(self.config["runner"]["process_sigint_timeout_sec"]),
                    float(self.config["runner"]["process_sigterm_timeout_sec"]),
                )
            )
            self._transition(status, "CLEANUP")
            atomic_write_json(status_path, status)
            return {"valid": False, "classification": "invalid_episode", "runner_status": status}

    def run_with_retries(
        self,
        candidate_yaml: str | Path,
        scenario_manifest_path: str | Path,
        output_directory: str | Path,
        domain_id: int,
    ) -> dict[str, Any]:
        maximum = int(self.config["runner"]["max_infrastructure_retries"])
        root = Path(output_directory)
        attempts = []
        for attempt in range(maximum + 1):
            result = self.run_once(
                candidate_yaml,
                scenario_manifest_path,
                root / f"attempt_{attempt}",
                domain_id + attempt,
            )
            attempts.append(result)
            if result.get("valid", False):
                result["attempt_count"] = attempt + 1
                return result
        raise ExperimentInfrastructureError(
            f"episode remained invalid after {maximum + 1} attempts: {attempts[-1]}"
        )
