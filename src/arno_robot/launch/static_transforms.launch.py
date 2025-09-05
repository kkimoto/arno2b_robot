from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_link_to_laser1",
            arguments=[
                "--x", "0.082", "--y", "0.0", "--z", "0.0",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "base_link", "--child-frame-id", "laser_frame_1"
            ]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_link_to_laser2",
            arguments=[
                "--x", "-0.082", "--y", "0.00", "--z", "0.1",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "3.14159",
                "--frame-id", "base_link", "--child-frame-id", "laser_frame_2"
            ]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_footprint_to_base_link",
            arguments=[
                "--x", "-0.082", "--y", "0.00", "--z", "0.1",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "base_footprint", "--child-frame-id", "base_link"
            ]
        ),
    ])
