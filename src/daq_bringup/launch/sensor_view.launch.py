"""Visualize live D455 RGB-D and IWR6843ISK point clouds together."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _extrinsics_setup(context):
    config_path = Path(
        LaunchConfiguration("extrinsics_file").perform(context)
    ).expanduser().resolve()
    if not config_path.is_file():
        raise RuntimeError(f"Extrinsics file does not exist: {config_path}")

    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    translation = config["translation_m"]
    rotation = config["rotation_rpy_rad"]
    parent_frame = str(config["parent_frame"])
    child_frame = str(config["child_frame"])
    calibrated = bool(config.get("calibrated", False))

    if calibrated:
        status = f"Using calibrated radar-camera extrinsics: {config_path}"
    else:
        status = (
            "WARNING: radar-camera extrinsics are not calibrated. "
            f"Using visualization estimate: {config_path}"
        )

    static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="d455_iwr6843isk_static_tf",
        arguments=[
            "--x", str(translation["x"]),
            "--y", str(translation["y"]),
            "--z", str(translation["z"]),
            "--roll", str(rotation["roll"]),
            "--pitch", str(rotation["pitch"]),
            "--yaw", str(rotation["yaw"]),
            "--frame-id", parent_frame,
            "--child-frame-id", child_frame,
        ],
        output="screen",
    )
    return [LogInfo(msg=status), static_transform]


def generate_launch_description():
    package_share = Path(get_package_share_directory("daq_bringup"))
    default_extrinsics = (
        package_share / "config" / "extrinsics" / "d455_iwr6843isk.yaml"
    )
    default_rviz = package_share / "rviz" / "sensor_view.rviz"

    depth_throttle = Node(
        package="daq_bringup",
        executable="image_rate_throttle",
        name="d455_view_depth_throttle",
        parameters=[{"max_rate_hz": LaunchConfiguration("pointcloud_rate")}],
        remappings=[
            ("input", "/camera/camera/depth/image_rect_raw"),
            ("output", "/daq/view/depth/image_rect_raw"),
        ],
        output="screen",
    )

    register_depth = Node(
        package="depth_image_proc",
        executable="register_node",
        name="d455_view_register_depth",
        remappings=[
            ("depth/image_rect", "/daq/view/depth/image_rect_raw"),
            ("depth/camera_info", "/camera/camera/depth/camera_info"),
            ("rgb/camera_info", "/camera/camera/color/camera_info"),
            (
                "depth_registered/image_rect",
                "/daq/view/aligned_depth_to_color/image_raw",
            ),
            (
                "depth_registered/camera_info",
                "/daq/view/aligned_depth_to_color/camera_info",
            ),
        ],
        output="screen",
    )

    textured_pointcloud = Node(
        package="depth_image_proc",
        executable="point_cloud_xyzrgb_node",
        name="d455_view_rgb_pointcloud",
        remappings=[
            ("rgb/camera_info", "/camera/camera/color/camera_info"),
            ("rgb/image_rect_color", "/camera/camera/color/image_raw"),
            (
                "depth_registered/image_rect",
                "/daq/view/aligned_depth_to_color/image_raw",
            ),
            ("points", "/camera/camera/depth/color/points"),
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="daq_sensor_view",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pointcloud_rate",
                default_value="15.0",
                description="Maximum D455 visualization point-cloud rate in Hz",
            ),
            DeclareLaunchArgument(
                "extrinsics_file",
                default_value=str(default_extrinsics),
                description="D455-to-IWR6843ISK extrinsics YAML",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(default_rviz),
                description="Combined sensor RViz config",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz2",
            ),
            OpaqueFunction(function=_extrinsics_setup),
            depth_throttle,
            register_depth,
            textured_pointcloud,
            rviz,
        ]
    )
