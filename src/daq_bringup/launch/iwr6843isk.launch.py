"""Start the IWR6843ISK PointCloud2 publisher for data acquisition."""

from pathlib import Path

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _validate_radar_config(config_file: Path) -> None:
    if not config_file.is_file():
        raise RuntimeError(
            "IWR6843ISK config file does not exist: "
            f"{config_file}\n"
            "Place the TI mmWave .cfg file at that path or pass "
            "radar_config:=/absolute/path/to/profile.cfg."
        )

    commands = set()
    try:
        for line in config_file.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line and not line.startswith(("%", "#")):
                commands.add(line.split()[0])
    except UnicodeDecodeError as error:
        raise RuntimeError(
            f"Radar config must be a UTF-8 text file: {config_file}"
        ) from error

    required_commands = {"profileCfg", "frameCfg", "sensorStart"}
    missing_commands = sorted(required_commands - commands)
    if missing_commands:
        raise RuntimeError(
            f"Radar config is missing required commands {missing_commands}: "
            f"{config_file}"
        )


def _launch_setup(context):
    config_file = Path(
        LaunchConfiguration("radar_config").perform(context)
    ).expanduser().resolve()
    cli_port = Path(
        LaunchConfiguration("cli_port").perform(context)
    ).expanduser().resolve()
    data_port = Path(
        LaunchConfiguration("data_port").perform(context)
    ).expanduser().resolve()

    _validate_radar_config(config_file)

    if cli_port == data_port:
        raise RuntimeError("Radar CLI and data ports must be different devices.")
    if not cli_port.exists():
        raise RuntimeError(f"Radar CLI serial port does not exist: {cli_port}")
    if not data_port.exists():
        raise RuntimeError(f"Radar data serial port does not exist: {data_port}")

    try:
        get_package_prefix("daq_iwr6843isk")
    except LookupError as error:
        raise RuntimeError(
            "ROS package 'daq_iwr6843isk' is not installed or sourced. "
            "Build and source daq_ws before launching."
        ) from error

    actions = [
        Node(
            package="daq_iwr6843isk",
            executable="pcl_pub",
            name="iwr6843_pcl_pub",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "cfg_path": str(config_file),
                    "cli_port": str(cli_port),
                    "data_port": str(data_port),
                }
            ],
        )
    ]
    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        )
    )
    return actions


def generate_launch_description():
    package_share = Path(get_package_share_directory("daq_bringup"))
    default_config = package_share / "config" / "iwr6843isk" / "radar.cfg"
    default_rviz_config = package_share / "rviz" / "iwr6843isk.rviz"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "radar_config",
                default_value=str(default_config),
                description="Absolute path to the TI mmWave CLI .cfg file",
            ),
            DeclareLaunchArgument(
                "cli_port",
                default_value="/dev/ttyUSB0",
                description="IWR6843ISK CLI serial port (115200 baud)",
            ),
            DeclareLaunchArgument(
                "data_port",
                default_value="/dev/ttyUSB1",
                description="IWR6843ISK data serial port (921600 baud)",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz with the radar PointCloud2 display",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(default_rviz_config),
                description="Absolute path to the radar RViz config",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
