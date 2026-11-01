"""Mount-aware radar-only viewer for an already running acquisition node."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from daq_iwr6843isk.mounting import Mounting


def _setup(context):
    path = LaunchConfiguration('mounting_file').perform(context)
    mounting = Mounting.load(path)
    return [
        Node(package='daq_iwr6843isk', executable='radar_point_viewer',
             name='radar_point_viewer', output='screen',
             parameters=[{'mounting_file': path,
                          'show_labels': ParameterValue(LaunchConfiguration('show_point_labels'), value_type=bool)}]),
        Node(package='rviz2', executable='rviz2', name='radar_rviz',
             arguments=['-d', LaunchConfiguration('rviz_config'), '-f', mounting.parent_frame],
             output='screen'),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('daq_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('mounting_file', default_value=str(share / 'config/extrinsics/d455_iwr6843isk.yaml')),
        DeclareLaunchArgument('rviz_config', default_value=str(share / 'rviz/iwr6843isk.rviz')),
        DeclareLaunchArgument('show_point_labels', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
