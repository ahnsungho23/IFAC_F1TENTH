from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("state_machine"),
            "config",
            "state_machine.yaml",
        ]),
        description="Path to state_machine_node parameter YAML",
    )
    timing_diagnostics_enable_arg = DeclareLaunchArgument(
        "timing_diagnostics_enable",
        default_value="false",
        description="Publish tuning-only monotonic T2/T3/T4 events",
    )
    timing_diagnostics_topic_arg = DeclareLaunchArgument(
        "timing_diagnostics_topic",
        default_value="/cma_timing/events",
        description="Companion timing-event topic",
    )
    tuning_publish_rate_hz_override_arg = DeclareLaunchArgument(
        "tuning_publish_rate_hz_override",
        default_value="-1.0",
        description="Tuning-only state timer rate override; -1 keeps YAML publish_rate_hz",
    )
    lockstep_mode_arg = DeclareLaunchArgument(
        "lockstep_mode",
        default_value="false",
        description="CMA-only exact-stamp Frenet/path event mode",
    )

    state_machine_node = Node(
        package="state_machine",
        executable="state_machine_node",
        name="state_machine_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "timing_diagnostics_enable": ParameterValue(
                    LaunchConfiguration("timing_diagnostics_enable"), value_type=bool),
                "timing_diagnostics_topic": LaunchConfiguration("timing_diagnostics_topic"),
                "tuning_publish_rate_hz_override": ParameterValue(
                    LaunchConfiguration("tuning_publish_rate_hz_override"), value_type=float),
                "lockstep_mode": ParameterValue(
                    LaunchConfiguration("lockstep_mode"), value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        params_file_arg,
        timing_diagnostics_enable_arg,
        timing_diagnostics_topic_arg,
        tuning_publish_rate_hz_override_arg,
        lockstep_mode_arg,
        state_machine_node,
    ])
