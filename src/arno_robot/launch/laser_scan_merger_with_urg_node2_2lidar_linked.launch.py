#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = FindPackageShare('arno_robot')

    sensors_2d_linked = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'urg_node2_2lidar_linked.launch.py'])
        )
    )

    merger = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'laser_scan_merger_standalone.launch.py'])
        )
    )

    return LaunchDescription([
        sensors_2d_linked,  # brings up TFs + two URG lifecycle nodes
        merger,             # loads the LaserScanMerger component container
    ])
