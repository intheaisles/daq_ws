"""Start an Intel RealSense D455 with the DAQ camera profile."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_setup(context):
    config_file = Path(
        LaunchConfiguration("config_file").perform(context)
    ).expanduser().resolve()
    json_file = Path(
        LaunchConfiguration("json_file_path").perform(context)
    ).expanduser().resolve()
    if not config_file.is_file():
        raise RuntimeError(f"D455 config file does not exist: {config_file}")
    if not json_file.is_file():
        raise RuntimeError(f"D455 advanced-mode JSON does not exist: {json_file}")

    try:
        realsense_share = Path(get_package_share_directory("realsense2_camera"))
    except LookupError as error:
        raise RuntimeError(
            "ROS package 'realsense2_camera' is not installed or sourced. "
            "Install ros-humble-realsense2-camera before starting the D455."
        ) from error

    realsense_launch = realsense_share / "launch" / "rs_launch.py"
    if not realsense_launch.is_file():
        raise RuntimeError(f"RealSense launch file does not exist: {realsense_launch}")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(realsense_launch)),
            launch_arguments={
                "config_file": str(config_file),
                "json_file_path": str(json_file),
            }.items(),
        )
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("daq_bringup"))
    default_config = package_share / "config" / "d455" / "d455.yaml"
    default_json = (
        package_share / "config" / "d455" / "d455_MD_emitter_off.json"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(default_config),
                description="Absolute path to a RealSense ROS 2 parameter YAML",
            ),
            DeclareLaunchArgument(
                "json_file_path",
                default_value=str(default_json),
                description="Absolute path to a RealSense Viewer advanced-mode JSON",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
