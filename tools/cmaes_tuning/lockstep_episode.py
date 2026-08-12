#!/usr/bin/env python3
"""Run one deterministic closed-loop CMA episode against the F110 backend."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any

import gymnasium as gym
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
import rosbag2_py
import yaml

from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Time
from f110_gym.envs import Integrator
from f110_msgs.msg import OTWpntArray, ObstacleArray, StateMachine, WpntArray
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

TOOL_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOL_ROOT))

from cmaes_tuning.configuration import load_config  # noqa: E402
from cmaes_tuning.deterministic_replay import (  # noqa: E402
    obstacle_geometry,
    waypoint_geometry_hash,
)
from cmaes_tuning.evaluator import evaluate_episode  # noqa: E402
from cmaes_tuning.lockstep import canonical_hash, float_token  # noqa: E402
from cmaes_tuning.objective import candidate_fitness  # noqa: E402
from cmaes_tuning.scenario_generator import load_waypoints, track_length  # noqa: E402
from cmaes_tuning.schemas import atomic_write_json, sha256_file  # noqa: E402
from cmaes_tuning.simulation_runner import ProcessSupervisor  # noqa: E402


TOPIC_TYPES = {
    "/ego_racecar/odom": "nav_msgs/msg/Odometry",
    "/scan": "sensor_msgs/msg/LaserScan",
    "/pf/pose/odom": "nav_msgs/msg/Odometry",
    "/car_state/frenet/odom": "nav_msgs/msg/Odometry",
    "/drive": "ackermann_msgs/msg/AckermannDriveStamped",
    "/drive_autonomous": "ackermann_msgs/msg/AckermannDriveStamped",
    "/ego_racecar/collision": "std_msgs/msg/Bool",
    "/avoid_waypoints": "f110_msgs/msg/OTWpntArray",
    "/local_waypoints": "f110_msgs/msg/WpntArray",
    "/global_waypoints": "f110_msgs/msg/WpntArray",
    "/state": "f110_msgs/msg/StateMachine",
    "/static_obs": "f110_msgs/msg/ObstacleArray",
}


def stamp_from_ns(value: int) -> Time:
    return Time(sec=value // 1_000_000_000, nanosec=value % 1_000_000_000)


def stamp_ns(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw_quaternion(yaw: float) -> tuple[float, float]:
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def sequence_hash(items: list[Any]) -> str:
    return canonical_hash(items)


class LockstepCoordinator(Node):
    def __init__(self) -> None:
        super().__init__("cma_lockstep_coordinator")
        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.ego_odom_pub = self.create_publisher(Odometry, "/ego_racecar/odom", reliable)
        self.pf_odom_pub = self.create_publisher(Odometry, "/pf/pose/odom", reliable)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", sensor)
        self.collision_pub = self.create_publisher(Bool, "/ego_racecar/collision", reliable)

        self.condition = threading.Condition()
        self.messages: dict[str, dict[int, Any]] = {}
        self.global_waypoints: WpntArray | None = None
        self.detector_map: OccupancyGrid | None = None
        self.drive_messages: list[AckermannDriveStamped] = []
        self.detector_events: list[dict[str, Any]] = []
        self.planner_events: list[dict[str, Any]] = []
        self.p3_events: dict[int, dict[str, Any]] = {}

        def stamped_callback(topic: str):
            def callback(message: Any) -> None:
                with self.condition:
                    self.messages.setdefault(topic, {})[stamp_ns(message)] = copy.deepcopy(message)
                    self.condition.notify_all()
            return callback

        self.create_subscription(
            WpntArray, "/global_waypoints", self._on_global, transient)
        self.create_subscription(
            OccupancyGrid, "/local_planning/reference_map", self._on_detector_map, transient)
        self.create_subscription(
            Odometry, "/car_state/frenet/odom", stamped_callback("/car_state/frenet/odom"), reliable)
        self.create_subscription(
            ObstacleArray, "/static_obs", stamped_callback("/static_obs"), reliable)
        self.create_subscription(
            OTWpntArray, "/avoid_waypoints", stamped_callback("/avoid_waypoints"), reliable)
        self.create_subscription(
            StateMachine, "/state", stamped_callback("/state"), transient)
        self.create_subscription(
            WpntArray, "/local_waypoints", stamped_callback("/local_waypoints"), reliable)
        self.create_subscription(
            AckermannDriveStamped, "/drive_autonomous",
            stamped_callback("/drive_autonomous"), reliable)
        self.create_subscription(AckermannDriveStamped, "/drive", self._on_drive, reliable)
        self.create_subscription(
            String, "/cma_replay/detector_events", self._on_detector_event,
            QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            String, "/cma_replay/planner_events", self._on_planner_event,
            QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            String, "/local_planning/p3_shadow", self._on_p3_event,
            QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))

    def _on_global(self, message: WpntArray) -> None:
        with self.condition:
            self.global_waypoints = copy.deepcopy(message)
            self.condition.notify_all()

    def _on_detector_map(self, message: OccupancyGrid) -> None:
        with self.condition:
            self.detector_map = copy.deepcopy(message)
            self.condition.notify_all()

    def _on_drive(self, message: AckermannDriveStamped) -> None:
        with self.condition:
            self.drive_messages.append(copy.deepcopy(message))
            self.condition.notify_all()

    def _on_detector_event(self, message: String) -> None:
        try:
            event = json.loads(message.data)
        except json.JSONDecodeError:
            return
        with self.condition:
            self.detector_events.append(event)

    def _on_planner_event(self, message: String) -> None:
        try:
            event = json.loads(message.data)
        except json.JSONDecodeError:
            return
        with self.condition:
            self.planner_events.append(event)

    def _on_p3_event(self, message: String) -> None:
        try:
            event = json.loads(message.data)
            source_stamp = int(event["source_stamp_ns"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            return
        with self.condition:
            self.p3_events[source_stamp] = event
            self.condition.notify_all()

    def wait_global(self, timeout: float) -> WpntArray:
        deadline = time.monotonic() + timeout
        with self.condition:
            while self.global_waypoints is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError("global waypoints were not received")
                self.condition.wait(remaining)
            return copy.deepcopy(self.global_waypoints)

    def wait_detector_map(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        with self.condition:
            while self.detector_map is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError("detector reference map was not received")
                self.condition.wait(remaining)

    def wait_stamped(self, topic: str, expected_stamp_ns: int, timeout: float) -> Any:
        deadline = time.monotonic() + timeout
        with self.condition:
            while expected_stamp_ns not in self.messages.get(topic, {}):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    available = sorted(self.messages.get(topic, {}))[-5:]
                    upstream = {
                        name: sorted(messages)[-3:]
                        for name, messages in self.messages.items() if messages
                    }
                    raise TimeoutError(
                        f"{topic} did not produce stamp {expected_stamp_ns}; available={available}; "
                        f"pending_upstream={upstream}"
                    )
                self.condition.wait(remaining)
            return copy.deepcopy(self.messages[topic].pop(expected_stamp_ns))

    def wait_drive_after(self, previous_count: int, timeout: float) -> AckermannDriveStamped:
        deadline = time.monotonic() + timeout
        with self.condition:
            while len(self.drive_messages) <= previous_count:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError("drive_source_selector did not forward the controller command")
                self.condition.wait(remaining)
            return copy.deepcopy(self.drive_messages[previous_count])

    def wait_p3_event(self, source_stamp_ns: int, timeout: float) -> dict[str, Any]:
        deadline = time.monotonic() + timeout
        with self.condition:
            while source_stamp_ns not in self.p3_events:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError(
                        f"P3 diagnostic did not produce source stamp {source_stamp_ns}")
                self.condition.wait(remaining)
            return dict(self.p3_events.pop(source_stamp_ns))


def make_odom(obs: dict[str, Any], timestamp_ns: int) -> Odometry:
    message = Odometry()
    message.header.stamp = stamp_from_ns(timestamp_ns)
    message.header.frame_id = "map"
    message.child_frame_id = "ego_racecar/base_link"
    message.pose.pose.position.x = float(obs["poses_x"][0])
    message.pose.pose.position.y = float(obs["poses_y"][0])
    yaw = float(obs["poses_theta"][0])
    message.pose.pose.orientation.z, message.pose.pose.orientation.w = yaw_quaternion(yaw)
    message.twist.twist.linear.x = float(obs["linear_vels_x"][0])
    message.twist.twist.linear.y = float(obs["linear_vels_y"][0])
    message.twist.twist.angular.z = float(obs["ang_vels_z"][0])
    return message


def make_scan(obs: dict[str, Any], timestamp_ns: int, scan_fov: float, beams: int) -> LaserScan:
    message = LaserScan()
    message.header.stamp = stamp_from_ns(timestamp_ns)
    message.header.frame_id = "ego_racecar/laser"
    message.angle_min = -scan_fov / 2.0
    message.angle_max = scan_fov / 2.0
    message.angle_increment = scan_fov / beams
    message.range_min = 0.0
    message.range_max = 30.0
    message.ranges = np.asarray(obs["scans"][0], dtype="<f4").tolist()
    return message


def physics_token(obs: dict[str, Any]) -> dict[str, Any]:
    return {
        name: float_token(obs[name][0])
        for name in (
            "poses_x", "poses_y", "poses_theta", "linear_vels_x",
            "linear_vels_y", "ang_vels_z",
        )
    } | {"collision": bool(obs["collisions"][0])}


def scan_payload_hash(message: LaserScan) -> str:
    return hashlib.sha256(np.asarray(message.ranges, dtype="<f4").tobytes()).hexdigest()


def command_token(message: AckermannDriveStamped) -> dict[str, str]:
    return {
        "steering_angle": float_token(message.drive.steering_angle),
        "speed": float_token(message.drive.speed),
        "acceleration": float_token(message.drive.acceleration),
    }


def waypoint_message_is_finite(message: Any) -> bool:
    """Return whether every controller-relevant waypoint field is finite."""
    fields = (
        "s_m", "d_m", "x_m", "y_m", "d_right", "d_left", "psi_rad",
        "kappa_radpm", "vx_mps", "ax_mps2",
    )
    return all(
        math.isfinite(float(getattr(waypoint, field)))
        for waypoint in message.wpnts for field in fields
    )


def command_is_finite(message: AckermannDriveStamped) -> bool:
    return all(math.isfinite(float(value)) for value in (
        message.drive.steering_angle,
        message.drive.speed,
        message.drive.acceleration,
    ))


def contiguous_true_count(flags: list[bool]) -> int:
    """Count false->true intervals without inventing a temporal hysteresis."""
    return sum(flag and (index == 0 or not flags[index - 1])
               for index, flag in enumerate(flags))


def write_bag(path: Path, records: list[tuple[int, int, str, Any]]) -> None:
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    for identifier, (topic, type_name) in enumerate(TOPIC_TYPES.items()):
        writer.create_topic(rosbag2_py.TopicMetadata(identifier, topic, type_name, "cdr", [], ""))
    for timestamp_ns, _, topic, message in sorted(records):
        writer.write(topic, serialize_message(message), timestamp_ns)
    del writer


def simulator_configuration(path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    return payload["bridge"]["ros__parameters"]


def first_milestones(
    detector_events: list[dict[str, Any]], planner_events: list[dict[str, Any]],
    start_ns: int, period_ns: int,
) -> dict[str, Any]:
    def scan_index(stamp: int | None) -> int | None:
        return None if stamp is None else (int(stamp) - start_ns) // period_ns

    confirmation = next(
        (
            event for event in detector_events
            if any(track.get("track_status") == "CONFIRMED" for track in event.get("tracks", []))
        ),
        None,
    )
    commitment = next(
        (
            event for event in planner_events
            if event.get("event") == "COMMITMENT" and not event.get("replacing", False)
        ),
        None,
    )
    return {
        "confirmation_scan_index": scan_index(
            confirmation.get("scan_stamp_ns") if confirmation else None),
        "commitment_scan_index": scan_index(
            commitment.get("source_stamp_ns") if commitment else None),
        "committed_obstacle_id": commitment.get("obstacle_id") if commitment else None,
        "committed_target_d": commitment.get("target_d") if commitment else None,
        "selected_side": (
            "left" if commitment and commitment.get("go_left") else
            "right" if commitment else None
        ),
        "entry_transition_scale": commitment.get("entry_transition_scale") if commitment else None,
        "exit_transition_scale": commitment.get("exit_transition_scale") if commitment else None,
        "effective_entry_transition_scale": (
            commitment.get("effective_entry_transition_scale") if commitment else None),
        "effective_exit_transition_scale": (
            commitment.get("effective_exit_transition_scale") if commitment else None),
    }


def run(args: argparse.Namespace) -> dict[str, Any]:
    workspace = Path(args.workspace).resolve()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    stale_error = output / "lockstep_error.json"
    if stale_error.exists():
        stale_error.unlink()
    manifest_path = Path(args.scenario).resolve()
    candidate_path = Path(args.candidate).resolve()
    config = load_config(args.config)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    sim_config_path = Path(config["paths"]["simulator_config"])
    sim_config = simulator_configuration(sim_config_path)
    period_sec = float(sim_config["timestep"])
    period_ns = int(round(period_sec * 1.0e9))
    start_ns = 10_000_000_000
    seed = int(args.simulator_seed)
    sigma = float(args.scan_noise_std)

    environment = dict(os.environ)
    environment.update({
        "CMA_WORKSPACE_ROOT": str(workspace),
        "F1_MAP": manifest["map_name"],
        "ROS_LOG_DIR": str(output / "ros_logs"),
        "FASTDDS_BUILTIN_TRANSPORTS": args.dds_transport,
    })
    (output / "ros_logs").mkdir(exist_ok=True)
    # rclpy runs in this coordinator process, while ProcessSupervisor receives a copied env.
    os.environ["ROS_LOG_DIR"] = environment["ROS_LOG_DIR"]
    os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = environment["FASTDDS_BUILTIN_TRANSPORTS"]
    shell_prefix = " && ".join(
        f"source {path}" for path in (
            config["ros"]["ros_setup"],
            str((workspace / config["ros"]["workspace_setup"]).resolve()),
            config["ros"]["simulator_setup"],
        )
    )
    supervisor = ProcessSupervisor(output, environment, shell_prefix)
    controller = config["controller"]
    commands = {
        "global_planning": [
            "ros2", "launch", "global_planning", "global_planning.launch.py",
            f"map_name:={manifest['map_name']}",
        ],
        "local_planning": [
            "ros2", "launch", "local_planning", "local_planning.launch.py",
            f"params_file:={candidate_path}",
            f"reference_map:={manifest['clean_map_yaml']}",
            "simulator:=true", "use_sim_time:=false", "lockstep_mode:=true",
            f"lockstep_scan_offset_x_m:={sim_config['scan_distance_to_base_link']}",
            "replay_diagnostics_enable:=true",
            f"p3_mode:={args.p3_mode}",
        ],
        "state_machine": [
            "ros2", "launch", "state_machine", "state_machine.launch.py",
            "lockstep_mode:=true",
        ],
        "controller": [
            "ros2", "launch", "f1tenth_control", "control_sim.launch.py",
            f"max_speed:={controller['max_speed_mps']}",
            f"min_speed:={controller['min_speed_mps']}",
            f"max_lateral_accel:={controller['max_lateral_accel_mps2']}",
            f"base_max_accel:={controller['base_max_accel_mps2']}",
            f"base_max_decel:={controller['base_max_decel_mps2']}",
            f"launch_boost_enable:={str(controller['launch_boost_enable']).lower()}",
            f"deadzone_floor_speed:={controller['deadzone_floor_speed_mps']}",
            "lockstep_mode:=true", f"lockstep_period_sec:={period_sec}",
        ],
    }
    for name, command in commands.items():
        supervisor.start(name, command)

    node: LockstepCoordinator | None = None
    executor: MultiThreadedExecutor | None = None
    spin_thread: threading.Thread | None = None
    records: list[tuple[int, int, str, Any]] = []
    ordinal = 0
    try:
        rclpy.init()
        node = LockstepCoordinator()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        global_waypoints = node.wait_global(float(config["runner"]["ready_timeout_sec"]))
        node.wait_detector_map(float(config["runner"]["ready_timeout_sec"]))
        readiness_deadline = time.monotonic() + float(config["runner"]["ready_timeout_sec"])
        while time.monotonic() < readiness_deadline:
            if (
                node.ego_odom_pub.get_subscription_count() >= 2
                and node.pf_odom_pub.get_subscription_count() >= 1
                and node.scan_pub.get_subscription_count() >= 1
            ):
                break
            time.sleep(0.05)
        else:
            raise TimeoutError(
                "lockstep input subscribers were not ready: "
                f"ego={node.ego_odom_pub.get_subscription_count()} "
                f"pf={node.pf_odom_pub.get_subscription_count()} "
                f"scan={node.scan_pub.get_subscription_count()}")
        # Discovery settling is wall-clock infrastructure only; no physics step has begun yet.
        time.sleep(0.25)

        vehicle = {name: float(value) for name, value in sim_config["vehicle"].items()}
        baked_stem = str(Path(manifest["baked_map_yaml"]).with_suffix(""))
        env = gym.make(
            "f110_gym:f110-v0", map=baked_stem, map_ext=".png", num_agents=1,
            timestep=period_sec, integrator=Integrator.RK4, params=vehicle, seed=seed,
            num_beams=int(sim_config["scan_beams"]), scan_fov=float(sim_config["scan_fov"]),
            lidar_dist=float(sim_config["scan_distance_to_base_link"]), scan_noise_std=sigma,
        )
        spawn = manifest["spawn_pose"]
        obs, _, _, _ = env.reset(np.asarray([[spawn["x"], spawn["y"], spawn["yaw"]]]))

        reference = load_waypoints(manifest["waypoint_file"])
        reference_xy = np.asarray([(float(wp["x_m"]), float(wp["y_m"])) for wp in reference])
        reference_s = np.asarray([float(wp["s_m"]) for wp in reference])
        length = track_length(reference)
        progress = 0.0
        last_s: float | None = None
        started_step: int | None = None
        scenario_complete = False
        lap_complete = False
        collided = False
        controlled_stop_terminated = False
        controlled_stop_start_step: int | None = None
        physics_states: list[dict[str, Any]] = []
        scan_hashes: list[str] = []
        static_hashes: list[str] = []
        path_hashes: list[str] = []
        command_hash_inputs: list[dict[str, str]] = []
        trajectory: list[dict[str, str]] = []
        p3_events: list[dict[str, Any]] = []
        nonfinite_command = False
        nonfinite_path = False
        unsafe_path_published = False

        records.append((start_ns, ordinal, "/global_waypoints", global_waypoints)); ordinal += 1
        configured_timeout_sec = float(config["runner"]["episode_timeout_sec"])
        episode_timeout_sec = (
            configured_timeout_sec if args.maximum_duration is None
            else min(configured_timeout_sec, float(args.maximum_duration))
        )
        maximum_steps = int(math.ceil(episode_timeout_sec / period_sec))
        barrier_timeout = float(args.barrier_timeout)
        scenario_obstacles = manifest.get("obstacles", [manifest["obstacle"]])
        forward_obstacle_distances = [
            (float(obstacle["s"]) - float(spawn["s"])) % length
            for obstacle in scenario_obstacles
        ]
        scenario_distance = max(forward_obstacle_distances) + float(args.post_obstacle_distance)

        for step in range(maximum_steps):
            logical_ns = start_ns + step * period_ns
            odom = make_odom(obs, logical_ns)
            scan = make_scan(
                obs, logical_ns, float(sim_config["scan_fov"]), int(sim_config["scan_beams"]))
            collision = Bool(data=bool(obs["collisions"][0]))
            physics_states.append(physics_token(obs))
            trajectory.append({
                "x": float_token(obs["poses_x"][0]),
                "y": float_token(obs["poses_y"][0]),
                "yaw": float_token(obs["poses_theta"][0]),
            })
            scan_hashes.append(scan_payload_hash(scan))
            for topic, message in (
                ("/ego_racecar/odom", odom), ("/pf/pose/odom", odom),
                ("/scan", scan), ("/ego_racecar/collision", collision),
            ):
                records.append((logical_ns, ordinal, topic, copy.deepcopy(message))); ordinal += 1

            # Publish one immutable snapshot. Exact-stamp dependencies inside each lockstep node
            # form the detector -> planner -> FSM -> controller chain independent of callback order.
            node.ego_odom_pub.publish(odom)
            node.pf_odom_pub.publish(odom)
            node.collision_pub.publish(collision)
            node.scan_pub.publish(scan)
            autonomous = node.wait_stamped("/drive_autonomous", logical_ns, barrier_timeout)
            # By the time the final command exists, every upstream barrier for stamp k completed.
            frenet = node.wait_stamped("/car_state/frenet/odom", logical_ns, barrier_timeout)
            static = node.wait_stamped("/static_obs", logical_ns, barrier_timeout)
            avoid = node.wait_stamped("/avoid_waypoints", logical_ns, barrier_timeout)
            state = node.wait_stamped("/state", logical_ns, barrier_timeout)
            local = node.wait_stamped("/local_waypoints", logical_ns, barrier_timeout)
            p3_event = (
                node.wait_p3_event(logical_ns, barrier_timeout)
                if args.p3_mode == "TEST_ACTIVE" else None
            )
            records.append((logical_ns, ordinal, "/car_state/frenet/odom", frenet)); ordinal += 1
            records.append((logical_ns, ordinal, "/static_obs", static)); ordinal += 1
            records.append((logical_ns, ordinal, "/avoid_waypoints", avoid)); ordinal += 1
            records.append((logical_ns, ordinal, "/state", state)); ordinal += 1
            records.append((logical_ns, ordinal, "/local_waypoints", local)); ordinal += 1
            static_hashes.append(canonical_hash(obstacle_geometry(static)))
            path_hashes.append(waypoint_geometry_hash(avoid))
            if p3_event is not None:
                p3_events.append(p3_event)
            # drive_source_selector is value-preserving but wall-clock restamps. Lockstep applies
            # the controller value directly and records an exact-stamp /drive equivalent.
            drive = copy.deepcopy(autonomous)
            records.append((logical_ns, ordinal, "/drive_autonomous", autonomous)); ordinal += 1
            records.append((logical_ns, ordinal, "/drive", drive)); ordinal += 1
            command_hash_inputs.append(command_token(autonomous))

            nonfinite_command = nonfinite_command or not command_is_finite(autonomous)
            nonfinite_path = nonfinite_path or not (
                waypoint_message_is_finite(avoid) and waypoint_message_is_finite(local)
            )
            if p3_event is not None and str(p3_event.get("path_owner", "")).startswith("P3_"):
                unsafe_path_published = unsafe_path_published or (
                    p3_event.get("lifecycle_state") == "INVALIDATED" or
                    (p3_event.get("suffix_revalidated") is True and
                     p3_event.get("suffix_hard_valid") is not True) or
                    (p3_event.get("path_owner") == "P3_M1" and
                     p3_event.get("fresh_hard_valid") is not True)
                )

            # These are direct output-safety failures, so do not advance physics after observing
            # one.  A safely rejected suffix that routes to P0 is intentionally not included.
            if nonfinite_command or nonfinite_path or unsafe_path_published:
                break

            current_speed = math.hypot(
                float(obs["linear_vels_x"][0]), float(obs["linear_vels_y"][0]))
            if args.controlled_stop_early_termination:
                planner_parameters = yaml.safe_load(candidate_path.read_text(encoding="utf-8"))[
                    "local_planner_node"
                ]["ros__parameters"]
                stopped_speed_threshold = (
                    float(planner_parameters["safe_stop_deceleration_mps2"])
                    * float(planner_parameters["planning_period_ms"]) / 1000.0
                )
                stopped_hold_steps = int(math.ceil(
                    int(planner_parameters["safe_stop_release_cycles"])
                    * float(planner_parameters["planning_period_ms"]) / 1000.0
                    / period_sec
                ))
                safe_stop_commanded = avoid.ot_line == "raceline_static_safe_stop"
                if safe_stop_commanded and current_speed <= stopped_speed_threshold:
                    if controlled_stop_start_step is None:
                        controlled_stop_start_step = step
                    controlled_stop_terminated = (
                        step - controlled_stop_start_step + 1 >= stopped_hold_steps
                    )
                else:
                    controlled_stop_start_step = None
            nearest = int(np.argmin(
                np.square(reference_xy[:, 0] - float(obs["poses_x"][0])) +
                np.square(reference_xy[:, 1] - float(obs["poses_y"][0]))))
            current_s = float(reference_s[nearest])
            if started_step is None and current_speed > float(config["evaluation"]["start_speed_threshold_mps"]):
                started_step = step
                last_s = current_s
            elif started_step is not None and last_s is not None:
                forward = (current_s - last_s) % length
                if forward <= 0.5 * length:
                    progress += forward
                last_s = current_s

            obs, _, _, _ = env.step(np.asarray([
                [float(autonomous.drive.steering_angle), float(autonomous.drive.speed)]
            ]))
            collided = bool(obs["collisions"][0])
            lap_complete = progress >= length * float(config["evaluation"]["lap_fraction"])
            scenario_complete = progress >= scenario_distance
            if collided or scenario_complete or controlled_stop_terminated:
                final_ns = logical_ns + period_ns
                final_odom = make_odom(obs, final_ns)
                final_scan = make_scan(
                    obs, final_ns, float(sim_config["scan_fov"]), int(sim_config["scan_beams"]))
                final_collision = Bool(data=collided)
                physics_states.append(physics_token(obs))
                trajectory.append({
                    "x": float_token(obs["poses_x"][0]),
                    "y": float_token(obs["poses_y"][0]),
                    "yaw": float_token(obs["poses_theta"][0]),
                })
                scan_hashes.append(scan_payload_hash(final_scan))
                for topic, message in (
                    ("/ego_racecar/odom", final_odom), ("/pf/pose/odom", final_odom),
                    ("/scan", final_scan), ("/ego_racecar/collision", final_collision),
                ):
                    records.append((final_ns, ordinal, topic, message)); ordinal += 1
                break

        bag_path = output / "bag"
        write_bag(bag_path, records)
        end_step = len(physics_states) - 1
        lap_summary = {
            "schema": "lap_referee/1",
            "terminated": "nonfinite_or_unsafe_path" if (
                nonfinite_command or nonfinite_path or unsafe_path_published
            ) else "collision" if collided else (
                "scenario_window_complete" if scenario_complete else (
                    "controlled_safe_stop" if controlled_stop_terminated else "timeout")),
            "collided": collided,
            "lap_completed": lap_complete,
            "lap_time_s": (
                (end_step - started_step) * period_sec if started_step is not None else 0.0),
            "progress_m": progress,
            "track_length_m": length,
        }
        atomic_write_json(output / "rollout_summary.json", lap_summary)
        runner_status = {
            "schema": "cmaes_runner_status/1",
            "state": "EVALUATE",
            "domain_id": int(os.environ.get("ROS_DOMAIN_ID", "0")),
            "scenario_id": manifest["scenario_id"],
            "localization_mode": "ground_truth",
            "execution_mode": "deterministic_lockstep",
            "p3_mode": args.p3_mode,
            "simulator_runtime_configuration": {
                "simulator_seed": seed,
                "scan_noise_std_m": sigma,
                "scan_publication_mode": "backend_event_once",
            },
            "candidate_yaml": str(candidate_path),
            "candidate_sha256": sha256_file(candidate_path),
            "infrastructure_failures": [],
            "process_commands": {"deterministic_bag_writer": list(TOPIC_TYPES)},
            "fixed_controller_configuration": controller,
            "deterministic_step_contract": [
                "physics_state", "paired_gt_state", "lidar", "detector", "planner",
                "state_machine", "controller", "command", "env_step",
            ],
        }
        atomic_write_json(output / "runner_status.json", runner_status)
        candidate_events = [
            event for event in node.planner_events
            if event.get("event") == "PLAN_CANDIDATE"
        ]
        (output / "planner_candidate_events.jsonl").write_text(
            "".join(
                json.dumps(event, sort_keys=True, separators=(",", ":")) + "\n"
                for event in candidate_events
            ),
            encoding="utf-8",
        )
        (output / "detector_events.jsonl").write_text(
            "".join(
                json.dumps(event, sort_keys=True, separators=(",", ":")) + "\n"
                for event in node.detector_events
            ),
            encoding="utf-8",
        )
        lifecycle_events = [
            event for event in node.planner_events
            if event.get("event") != "PLAN_CANDIDATE"
        ]
        (output / "planner_lifecycle_events.jsonl").write_text(
            "".join(
                json.dumps(event, sort_keys=True, separators=(",", ":")) + "\n"
                for event in lifecycle_events
            ),
            encoding="utf-8",
        )
        (output / "p3_runtime_events.jsonl").write_text(
            "".join(
                json.dumps(event, sort_keys=True, separators=(",", ":")) + "\n"
                for event in p3_events
            ),
            encoding="utf-8",
        )
        episode = evaluate_episode(
            bag_path, manifest_path, output / "rollout_summary.json",
            output / "runner_status.json", config, output / "episode_result.json")
        invalid_suffix = any(
            event.get("lifecycle_state") == "INVALIDATED" or
            (event.get("suffix_revalidated") is True and
             event.get("suffix_hard_valid") is not True)
            for event in p3_events
        )
        safe_stop_flags = [
            bool(event.get("safe_stop_active")) or
            event.get("production_selected_path_family") in ("SAFE_STOP", "P0_SAFE_STOP")
            for event in p3_events
        ]
        p0_fallback_flags = [
            bool(event.get("p0_backup_only")) and bool(event.get("cluster_obstacle_ids"))
            for event in p3_events
        ]
        p3_ownership_flags = [
            event.get("path_owner") in (
                "P3_M1", "P3_COMMITTED_SUFFIX", "P3_COMPLETION_HANDOFF")
            for event in p3_events
        ]
        safe_stop = any(safe_stop_flags)
        p0_fallback = any(p0_fallback_flags)
        if episode.get("valid"):
            episode["failure"].update({
                "invalid_suffix": invalid_suffix,
                "safe_stop": safe_stop,
                "p0_fallback": p0_fallback,
            })
            if invalid_suffix or safe_stop or p0_fallback:
                episode["classification"] = "safety_failure"
            atomic_write_json(output / "episode_result.json", episode)
        fitness = candidate_fitness([episode], config)["fitness"] if episode.get("valid") else None
        milestones = first_milestones(
            node.detector_events, node.planner_events, start_ns, period_ns)
        failure = episode.get("failure", {})
        off_track = bool(failure.get("off_track"))
        planner_failure = bool(failure.get("planner_failure"))
        result = {
            "schema": "cma_lockstep_episode/1",
            "valid": bool(episode.get("valid")),
            "scenario_id": manifest["scenario_id"],
            "simulator_seed": seed,
            "scan_noise_std_m": sigma,
            "step_count": len(command_hash_inputs),
            "physics_state_sequence_hash": sequence_hash(physics_states),
            "scan_hash_sequence_hash": sequence_hash(scan_hashes),
            "static_geometry_sequence_hash": sequence_hash(static_hashes),
            "avoid_waypoints_hash_sequence_hash": sequence_hash(path_hashes),
            "controller_command_sequence_hash": sequence_hash(command_hash_inputs),
            "trajectory_hash": sequence_hash(trajectory),
            "collision": collided,
            "off_track": off_track,
            "planner_failure": planner_failure,
            "nonfinite": nonfinite_command or nonfinite_path,
            "nonfinite_command": nonfinite_command,
            "nonfinite_path": nonfinite_path,
            "unsafe_path_published": unsafe_path_published,
            "invalid_suffix": invalid_suffix,
            "safe_stop": safe_stop,
            "p0_fallback": p0_fallback,
            "p0_fallback_interval_count": contiguous_true_count(p0_fallback_flags),
            "p0_fallback_callback_count": sum(p0_fallback_flags),
            "p0_fallback_duration_s": sum(p0_fallback_flags) * period_sec,
            "safe_stop_interval_count": contiguous_true_count(safe_stop_flags),
            "safe_stop_callback_count": sum(safe_stop_flags),
            "safe_stop_duration_s": sum(safe_stop_flags) * period_sec,
            "p3_ownership_callback_count": sum(p3_ownership_flags),
            "p3_ownership_duration_s": sum(p3_ownership_flags) * period_sec,
            "p3_ownership_fraction": (
                sum(p3_ownership_flags) / len(p3_events) if p3_events else 0.0),
            "p3_event_count": len(p3_events),
            "p3_selected_count": sum(
                event.get("path_owner") == "P3_M1" for event in p3_events),
            "p3_continuation_count": sum(
                event.get("path_owner") == "P3_COMMITTED_SUFFIX" for event in p3_events),
            "p3_completion_count": sum(
                event.get("lifecycle_state") == "COMPLETE" for event in p3_events),
            "scenario_success": bool(
                scenario_complete and not collided and not off_track and not planner_failure and
                not nonfinite_command and not nonfinite_path and not unsafe_path_published),
            "completed": scenario_complete,
            "controlled_stop_terminated": controlled_stop_terminated,
            "lap_completed": lap_complete,
            "scenario_distance_m": scenario_distance,
            "fitness": fitness,
            "episode_classification": episode.get("classification"),
            **milestones,
            "numeric_tolerances": {
                "committed_target_d_abs_m": 1.0e-12,
                "fitness_abs": 1.0e-12,
            },
            "bit_identical_fields": [
                "physics_state_sequence_hash", "scan_hash_sequence_hash",
                "static_geometry_sequence_hash", "avoid_waypoints_hash_sequence_hash",
                "controller_command_sequence_hash", "trajectory_hash",
            ],
        }
        atomic_write_json(output / "lockstep_result.json", result)
        return result
    finally:
        supervisor.stop_all(3.0, 2.0)
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", default=str(TOOL_ROOT.parents[1]))
    parser.add_argument("--config", default=str(TOOL_ROOT / "config" / "tuning_config.yaml"))
    parser.add_argument("--candidate", required=True)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--simulator-seed", type=int, required=True)
    parser.add_argument("--scan-noise-std", type=float, default=0.01)
    parser.add_argument("--p3-mode", choices=("OFF", "TEST_ACTIVE"), default="OFF")
    parser.add_argument("--barrier-timeout", type=float, default=30.0)
    parser.add_argument(
        "--post-obstacle-distance", type=float, default=3.0,
        help="Closed-loop recovery distance after the obstacle before ending the scenario episode")
    parser.add_argument(
        "--maximum-duration", type=float,
        help=(
            "Optional tuning-only simulated-time horizon. The default preserves the configured "
            "runner timeout; this never changes production ROS runtime behavior."))
    parser.add_argument(
        "--controlled-stop-early-termination", action="store_true",
        help=(
            "Tuning-only early termination after raceline_static_safe_stop remains below the "
            "production stopped-speed threshold for the production release debounce duration."))
    parser.add_argument(
        "--dds-transport", default="UDPv4",
        help="Tuning process transport; lockstep barriers make transport timing non-semantic")
    args = parser.parse_args()
    try:
        result = run(args)
    except Exception as error:
        output = Path(args.output)
        output.mkdir(parents=True, exist_ok=True)
        atomic_write_json(output / "lockstep_error.json", {
            "error_type": type(error).__name__, "error": str(error)})
        raise
    print(json.dumps(result, sort_keys=True))
    return 0 if result["valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
