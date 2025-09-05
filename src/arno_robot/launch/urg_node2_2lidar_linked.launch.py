# src/arno_robot/launch/sensors_2d_linked.launch.py
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('arno_robot')

    static_tfs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'static_transforms.launch.py'])
        )
    )

    urg2d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'urg_node2_2lidar.launch.py'])
        )
    )

    return LaunchDescription([
        static_tfs,
        urg2d,
    ])
