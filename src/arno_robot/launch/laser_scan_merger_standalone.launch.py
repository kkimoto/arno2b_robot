#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('arno_robot'),
        'config',
        'params_default.yaml',      # contains the 'scan_merger:' block
    ])

    container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',   # single-threaded container (as in the originals)
        name='component_manager_node',
        namespace='',
        composable_node_descriptions=[
            ComposableNode(
                package='laser_scan_merger',
                plugin='util::LaserScanMerger',
                name='scan_merger',         # <-- must match the top-level key in your YAML
                parameters=[params],
                # If the node publishes on 'scan' and you want 'merged_scan', uncomment:
                # remappings=[('scan', 'merged_scan')],
            )
        ],
        output='screen',
    )
    return LaunchDescription([container])
