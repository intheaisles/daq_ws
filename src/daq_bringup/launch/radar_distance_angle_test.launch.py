"""Record a finite radar-only trial with explicit reference geometry."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('daq_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('plane_distance_m', description='Measured perpendicular distance to glass [m]'),
        DeclareLaunchArgument('incidence_angle_deg', description='Signed incidence to fixed reference point [deg]'),
        DeclareLaunchArgument('duration_s', default_value='30.0',
                              description='Trial duration from first frame; 0 means until Ctrl-C'),
        DeclareLaunchArgument('record_trial', default_value='true',
                              description='Save scan CSV, point CSV, raw UART and metadata'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'iwr6843isk.launch.py')),
            launch_arguments={
                'record_trial': 'true',
                'plane_distance_m': LaunchConfiguration('plane_distance_m'),
                'incidence_angle_deg': LaunchConfiguration('incidence_angle_deg'),
                'duration_s': LaunchConfiguration('duration_s'),
            }.items(),
        ),
    ])
