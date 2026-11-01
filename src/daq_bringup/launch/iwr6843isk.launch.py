"""Start the IWR6843ISK PointCloud2 publisher for data acquisition."""

from pathlib import Path

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from daq_iwr6843isk.paths import default_output_root
from daq_iwr6843isk.mounting import Mounting


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
    mounting_file = Path(LaunchConfiguration("mounting_file").perform(context)).expanduser().resolve()
    mounting = Mounting.load(mounting_file)

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

    radar_node = Node(
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
                    **{
                        key: ParameterValue(LaunchConfiguration(key), value_type=value_type)
                        for key, value_type in {
                            "record_trial": bool, "output_root": str,
                            "plane_distance_m": float, "incidence_angle_deg": float,
                            "trial_id": str, "duration_s": float, "roi_file": str,
                            "scan_timeout_s": float,
                            "off_axis_deg": str,
                            "mounting_file": str,
                        }.items()
                    },
                }
            ],
        )
    actions = [
        RegisterEventHandler(
            OnProcessExit(target_action=radar_node,
                          on_exit=[EmitEvent(event=Shutdown(reason="Radar trial finished"))]),
            condition=IfCondition(LaunchConfiguration("record_trial")),
        ),
        radar_node,
    ]
    actions.append(
        Node(
            package="daq_iwr6843isk", executable="radar_point_viewer",
            name="radar_point_viewer",
            parameters=[{"mounting_file": str(mounting_file),
                         "show_labels": ParameterValue(LaunchConfiguration("show_point_labels"), value_type=bool)}],
            condition=IfCondition(LaunchConfiguration("rviz")), output="screen",
        )
    )
    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["-d", LaunchConfiguration("rviz_config"), "-f", mounting.parent_frame],
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
            DeclareLaunchArgument("record_trial", default_value="false", description="Save trial artifacts"),
            DeclareLaunchArgument("output_root", default_value=default_output_root(),
                                  description="Parent directory for unique trial folders"),
            DeclareLaunchArgument("plane_distance_m", default_value="0.0",
                                  description="Measured perpendicular distance to glass [m]"),
            DeclareLaunchArgument("incidence_angle_deg", default_value="0.0",
                                  description="Signed incidence angle to fixed glass reference [deg]"),
            DeclareLaunchArgument("trial_id", default_value="trial1", description="Repeat/condition label"),
            DeclareLaunchArgument("duration_s", default_value="0.0",
                                  description="Seconds from first frame; 0 means until Ctrl-C"),
            DeclareLaunchArgument("roi_file", default_value="",
                                  description="Measured rectangular glass ROI JSON in raw radar frame; blank records all points"),
            DeclareLaunchArgument("scan_timeout_s", default_value="3.0",
                                  description="Abort when fresh hardware frames stop for this many seconds"),
            DeclareLaunchArgument("off_axis_deg", default_value="",
                                  description="Optional signed reference-point angle from radar boresight [deg]; blank means unknown"),
            DeclareLaunchArgument("mounting_file", default_value=str(package_share / "config/extrinsics/d455_iwr6843isk.yaml"),
                                  description="Raw radar mounting transform saved with trial data and used by viewer"),
            DeclareLaunchArgument("show_point_labels", default_value="true",
                                  description="Show scan index labels beside radar points (XYZ/SNR remain in table)"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
