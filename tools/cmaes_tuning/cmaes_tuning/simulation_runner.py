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
import re
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
        try:
            os.killpg(managed.process.pid, signal_number)
        except ProcessLookupError:
            pass

    @staticmethod
    def _group_is_alive(managed: ManagedProcess) -> bool:
        try:
            os.killpg(managed.process.pid, 0)
            return True
        except ProcessLookupError:
            return False

    def stop_one(self, name: str, sigint_timeout: float, sigterm_timeout: float) -> bool:
        matches = [managed for managed in self.processes if managed.name == name]
        if not matches:
            return True
        managed = matches[-1]
        if self._group_is_alive(managed):
            self._signal_group(managed, signal.SIGINT)
            deadline = time.monotonic() + sigint_timeout
            while self._group_is_alive(managed) and time.monotonic() < deadline:
                time.sleep(0.05)
        if self._group_is_alive(managed):
            self._signal_group(managed, signal.SIGTERM)
            deadline = time.monotonic() + sigterm_timeout
            while self._group_is_alive(managed) and time.monotonic() < deadline:
                time.sleep(0.05)
        if self._group_is_alive(managed):
            self._signal_group(managed, signal.SIGKILL)
            deadline = time.monotonic() + 1.0
            while self._group_is_alive(managed) and time.monotonic() < deadline:
                time.sleep(0.05)
        if managed.process.poll() is None:
            try:
                managed.process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                pass
        if not managed.log_stream.closed:
            managed.log_stream.close()
        return not self._group_is_alive(managed) and managed.process.poll() is not None

    def stop_all(self, sigint_timeout: float, sigterm_timeout: float) -> list[str]:
        managed_processes = list(reversed(self.processes))
        for managed in managed_processes:
            if self._group_is_alive(managed):
                self._signal_group(managed, signal.SIGINT)
        deadline = time.monotonic() + sigint_timeout
        while (
            any(self._group_is_alive(managed) for managed in managed_processes)
            and time.monotonic() < deadline
        ):
            time.sleep(0.05)

        for managed in managed_processes:
            if self._group_is_alive(managed):
                self._signal_group(managed, signal.SIGTERM)
        deadline = time.monotonic() + sigterm_timeout
        while (
            any(self._group_is_alive(managed) for managed in managed_processes)
            and time.monotonic() < deadline
        ):
            time.sleep(0.05)

        for managed in managed_processes:
            if self._group_is_alive(managed):
                self._signal_group(managed, signal.SIGKILL)
        deadline = time.monotonic() + 1.0
        while (
            any(self._group_is_alive(managed) for managed in managed_processes)
            and time.monotonic() < deadline
        ):
            time.sleep(0.05)

        failures = []
        for managed in managed_processes:
            if managed.process.poll() is None:
                try:
                    managed.process.wait(timeout=0.2)
                except subprocess.TimeoutExpired:
                    pass
            if not managed.log_stream.closed:
                managed.log_stream.close()
            if self._group_is_alive(managed) or managed.process.poll() is None:
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
                "FASTDDS_BUILTIN_TRANSPORTS": str(
                    self.config["ros"].get("fastdds_builtin_transports", "DEFAULT")
                ),
            }
        )
        (episode_directory / "ros_logs").mkdir(parents=True, exist_ok=True)
        return environment

    def _wait_for_topics(
        self, supervisor: ProcessSupervisor, status: dict[str, Any]
    ) -> list[str]:
        required = set(self.config["ros"]["required_topics_before_control"])
        timing = self.config.get("timing_audit", {})
        if bool(timing.get("enabled", False)):
            required.add(str(timing.get("topic", "/cma_timing/events")))
        started = time.monotonic()
        deadline = started + float(self.config["runner"]["ready_timeout_sec"])
        initial_delay = float(
            self.config["runner"].get("readiness_initial_delay_sec", 0.0)
        )
        time.sleep(initial_delay)
        last_topics: set[str] = set()
        probe_count = 0
        probe_timeout_count = 0
        while time.monotonic() < deadline:
            early = supervisor.early_exits()
            if early:
                return early
            probe_count += 1
            try:
                result = supervisor.run(
                    ["ros2", "topic", "list", "--no-daemon", "--spin-time", "0.3"],
                    timeout=float(self.config["runner"]["topic_probe_timeout_sec"]),
                )
            except subprocess.TimeoutExpired:
                probe_timeout_count += 1
                time.sleep(float(self.config["runner"]["poll_period_sec"]))
                continue
            if result.returncode == 0:
                last_topics = {line.strip() for line in result.stdout.splitlines() if line.strip()}
                if required.issubset(last_topics):
                    status["readiness_diagnostics"] = {
                        "initial_delay_s": initial_delay,
                        "elapsed_s": time.monotonic() - started,
                        "probe_count": probe_count,
                        "probe_timeout_count": probe_timeout_count,
                        "required_topic_count": len(required),
                    }
                    return []
            time.sleep(float(self.config["runner"]["poll_period_sec"]))
        status["readiness_diagnostics"] = {
            "initial_delay_s": initial_delay,
            "elapsed_s": time.monotonic() - started,
            "probe_count": probe_count,
            "probe_timeout_count": probe_timeout_count,
            "required_topic_count": len(required),
            "last_topic_count": len(last_topics),
        }
        return [f"required_topics_missing:{sorted(required - last_topics)}"]

    def _audit_localization_graph(
        self, supervisor: ProcessSupervisor, status: dict[str, Any]
    ) -> list[str]:
        """Verify that exactly one selected localization provider owns /pf/pose/odom."""
        timeout = float(self.config["runner"]["topic_probe_timeout_sec"])
        info = supervisor.run(
            [
                "ros2", "topic", "info", "--verbose", "--no-daemon",
                "--spin-time", "0.5", "/pf/pose/odom",
            ],
            timeout=timeout,
        )
        nodes = supervisor.run(
            ["ros2", "node", "list", "--no-daemon", "--spin-time", "0.5"],
            timeout=timeout,
        )
        publisher_match = re.search(r"Publisher count:\s*(\d+)", info.stdout)
        publisher_count = int(publisher_match.group(1)) if publisher_match else None
        publisher_section = info.stdout.split("Subscription count:", 1)[0]
        publisher_node_names = sorted(
            set(re.findall(r"^Node name:\s*(\S+)\s*$", publisher_section, re.MULTILINE))
        )
        node_names = sorted(line.strip() for line in nodes.stdout.splitlines() if line.strip())
        mode = str(self.config["experiment"].get("localization_mode", "mcl"))
        status["localization_graph_audit"] = {
            "mode": mode,
            "pose_topic": "/pf/pose/odom",
            "publisher_count": publisher_count,
            "publisher_node_names": publisher_node_names,
            "topic_info_returncode": info.returncode,
            "topic_info": info.stdout,
            "topic_info_stderr": info.stderr,
            "node_list_returncode": nodes.returncode,
            "node_names": node_names,
            "bridge_publishes_tf": False,
            "simulator_tf_edges": [
                "map->ego_racecar/base_link",
                "ego_racecar/base_link->ego_racecar/laser",
            ],
        }
        failures = []
        if info.returncode != 0 or publisher_count != 1:
            failures.append(f"localization_pose_publisher_count:{publisher_count}")
        publisher_basenames = {name.rsplit("/", 1)[-1] for name in publisher_node_names}
        graph_basenames = {name.rsplit("/", 1)[-1] for name in node_names}
        bridge_publishes_pose = "gt_localization_bridge" in publisher_basenames
        particle_filter_publishes_pose = "particle_filter" in publisher_basenames
        if mode == "ground_truth":
            if not bridge_publishes_pose:
                failures.append("gt_localization_bridge_missing")
            if particle_filter_publishes_pose or "particle_filter" in graph_basenames:
                failures.append("particle_filter_present_in_ground_truth_mode")
        else:
            if bridge_publishes_pose or "gt_localization_bridge" in graph_basenames:
                failures.append("gt_localization_bridge_present_in_mcl_mode")
            if not particle_filter_publishes_pose:
                failures.append("particle_filter_missing_in_mcl_mode")
        return failures

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
            "localization_mode": str(
                self.config["experiment"].get("localization_mode", "mcl")
            ),
            "simulator_runtime_configuration": dict(
                self.config["simulator_runtime"]
            ),
            "timing_audit_configuration": dict(
                self.config.get("timing_audit", {})
            ),
            "candidate_yaml": str(candidate_path),
            "candidate_sha256": sha256_file(candidate_path),
            "infrastructure_failures": [],
            "process_commands": {},
            "middleware_environment": {
                "rmw_implementation": os.environ.get(
                    "RMW_IMPLEMENTATION", "rmw_fastrtps_cpp (default)"
                ),
                "fastdds_builtin_transports": str(
                    self.config["ros"].get("fastdds_builtin_transports", "DEFAULT")
                ),
            },
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
        simulator_runtime = self.config["simulator_runtime"]
        timing_audit = self.config.get("timing_audit", {})
        timing_enabled = bool(timing_audit.get("enabled", False))
        timing_topic = str(timing_audit.get("topic", "/cma_timing/events"))
        state_machine_rate = float(
            timing_audit.get("state_machine_publish_rate_hz", 10.0)
        )
        paths = self.config["paths"]
        baked_stem = str(Path(manifest["baked_map_yaml"]).with_suffix(""))
        output_prefix = "rollout"
        lap_summary = directory / f"{output_prefix}_summary.json"
        bag_directory = directory / "bag"
        localization_mode = status["localization_mode"]
        if localization_mode not in {"mcl", "ground_truth"}:
            raise ExperimentInfrastructureError(
                f"unsupported localization_mode: {localization_mode}"
            )
        localization_command = (
            [
                "ros2", "launch", "particle_filter_cpp", "mcl_launch.py",
                "mod:=sim", f"map_name:={manifest['map_name']}", "use_rviz:=false",
                "start_map_server:=true", "use_sim_time:=false",
            ]
            if localization_mode == "mcl"
            else [
                "ros2", "launch", "cma_gt_localization",
                "gt_localization_bridge.launch.py", "use_sim_time:=false",
            ]
        )
        localization_process_name = (
            "mcl" if localization_mode == "mcl" else "gt_localization"
        )
        commands = {
            "simulator": [
                "ros2", "launch", "f1tenth_gym_ros", "obstacle_sim_launch.py",
                f"map_path:={baked_stem}", "map_img_ext:=.png",
                f"sx:={manifest['spawn_pose']['x']}",
                f"sy:={manifest['spawn_pose']['y']}",
                f"stheta:={manifest['spawn_pose']['yaw']}",
                f"simulator_seed:={int(simulator_runtime['simulator_seed'])}",
                f"scan_noise_std:={float(simulator_runtime['scan_noise_std_m'])}",
                f"scan_publication_mode:={simulator_runtime['scan_publication_mode']}",
                "publish_scan_identity:="
                f"{'true' if simulator_runtime.get('publish_scan_identity', True) else 'false'}",
                f"timing_diagnostics_enable:={'true' if timing_enabled else 'false'}",
                f"timing_diagnostics_topic:={timing_topic}",
                "num_agent:=1", "rviz:=false",
            ],
            "global_planning": [
                "ros2", "launch", "global_planning", "global_planning.launch.py",
                f"map_name:={manifest['map_name']}",
            ],
            localization_process_name: localization_command,
            "local_planning": [
                "ros2", "launch", "local_planning", "local_planning.launch.py",
                f"params_file:={candidate_path}",
                f"reference_map:={manifest['clean_map_yaml']}",
                "simulator:=true", "use_sim_time:=false",
                f"timing_diagnostics_enable:={'true' if timing_enabled else 'false'}",
                f"timing_diagnostics_topic:={timing_topic}",
            ],
            "state_machine": [
                "ros2", "launch", "state_machine", "state_machine.launch.py",
                f"timing_diagnostics_enable:={'true' if timing_enabled else 'false'}",
                f"timing_diagnostics_topic:={timing_topic}",
                f"tuning_publish_rate_hz_override:={state_machine_rate if timing_enabled else -1.0}",
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
                f"timing_diagnostics_enable:={'true' if timing_enabled else 'false'}",
                f"timing_diagnostics_topic:={timing_topic}",
            ],
        }
        status["process_commands"] = commands
        try:
            self._transition(status, "LAUNCH")
            for name in (
                "simulator", "global_planning", localization_process_name,
                "local_planning", "state_machine",
            ):
                supervisor.start(name, commands[name])
                time.sleep(0.25)
            # Capture detector startup and the exact scan callbacks that satisfy
            # existence/envelope confirmation. The recorder is observational;
            # control still starts only after the readiness audit below.
            supervisor.start("recorder", commands["recorder"])

            self._transition(status, "WAIT_READY")
            status["infrastructure_failures"].extend(
                self._wait_for_topics(supervisor, status)
            )
            if not status["infrastructure_failures"]:
                status["infrastructure_failures"].extend(
                    self._audit_localization_graph(supervisor, status)
                )
            if (
                not status["infrastructure_failures"]
                and localization_mode == "mcl"
            ):
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
                    timeout=float(
                        self.config["runner"].get(
                            "initial_pose_publish_timeout_sec",
                            self.config["runner"]["topic_probe_timeout_sec"],
                        )
                    ),
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
                    # MCL may auto/global-initialize while its map and scan
                    # pipeline is still starting. Reapply the exact same pose
                    # after initialization so startup ordering cannot overwrite
                    # the scenario spawn before the controller begins.
                    republish_result = supervisor.run(
                        [
                            "ros2",
                            "topic",
                            "pub",
                            "--once",
                            "/initialpose",
                            "geometry_msgs/msg/PoseWithCovarianceStamped",
                            json.dumps(initial_pose, separators=(",", ":")),
                        ],
                        timeout=float(
                            self.config["runner"].get(
                                "initial_pose_publish_timeout_sec",
                                self.config["runner"]["topic_probe_timeout_sec"],
                            )
                        ),
                    )
                    status["initial_pose_republish"] = {
                        "returncode": republish_result.returncode,
                        "stdout": republish_result.stdout,
                        "stderr": republish_result.stderr,
                    }
                    if republish_result.returncode != 0:
                        status["infrastructure_failures"].append(
                            "initial_pose_republish_failed"
                        )
                    time.sleep(float(self.config["runner"]["localization_settle_sec"]))
            elif not status["infrastructure_failures"]:
                status["initial_pose_publish"] = {
                    "skipped": True,
                    "reason": "simulator launch already applied the exact spawn pose; "
                    "ground_truth mode has no MCL initialization",
                }
            if not status["infrastructure_failures"]:
                # lap_referee's no-start timer begins at process startup, so
                # start both referee and recorder only after the slow MCL/map
                # readiness phase has completed.
                supervisor.start("lap_referee", commands["lap_referee"])
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
            runtime_samples = []
            previous_poll = time.monotonic()
            while not status["infrastructure_failures"] and time.monotonic() < deadline:
                poll_now = time.monotonic()
                runtime_samples.append(
                    {
                        "elapsed_s": poll_now - runtime_samples[0]["monotonic_s"]
                        if runtime_samples
                        else 0.0,
                        "monotonic_s": poll_now,
                        "poll_interval_s": poll_now - previous_poll,
                        "load_average_1m": os.getloadavg()[0],
                    }
                )
                previous_poll = poll_now
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
            if runtime_samples:
                poll_intervals = [
                    float(sample["poll_interval_s"]) for sample in runtime_samples[1:]
                ]
                loads = [float(sample["load_average_1m"]) for sample in runtime_samples]
                status["runtime_diagnostics"] = {
                    "sample_count": len(runtime_samples),
                    "load_average_1m_mean": sum(loads) / len(loads),
                    "load_average_1m_max": max(loads),
                    "poll_interval_s_mean": sum(poll_intervals) / len(poll_intervals)
                    if poll_intervals
                    else 0.0,
                    "poll_interval_s_max": max(poll_intervals, default=0.0),
                }
            if not lap_summary.is_file() and not status["infrastructure_failures"]:
                status["infrastructure_failures"].append("episode_summary_timeout")

            self._transition(status, "TERMINATE")
            sigint = float(self.config["runner"]["process_sigint_timeout_sec"])
            sigterm = float(self.config["runner"]["process_sigterm_timeout_sec"])
            status["infrastructure_failures"].extend(supervisor.stop_all(sigint, sigterm))

            self._transition(status, "COLLECT")
            recorder_started = any(
                process.name == "recorder" for process in supervisor.processes
            )
            if recorder_started and not (bag_directory / "metadata.yaml").is_file():
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
        except KeyboardInterrupt:
            status["infrastructure_failures"].append("runner_interrupted")
            self._transition(status, "TERMINATE")
            status["infrastructure_failures"].extend(
                supervisor.stop_all(
                    float(self.config["runner"]["process_sigint_timeout_sec"]),
                    float(self.config["runner"]["process_sigterm_timeout_sec"]),
                )
            )
            self._transition(status, "CLEANUP")
            atomic_write_json(status_path, status)
            raise

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
