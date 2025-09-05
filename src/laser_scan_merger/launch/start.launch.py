from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument(
        'robotname',
        default_value='',
        description='Pass robotname to select configuration for merger',
    )
]

def generate_launch_description():

    container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container",
        name="component_manager_node",
        namespace="",
        composable_node_descriptions=[
            ComposableNode(
                package="laser_scan_merger",
                plugin="util::LaserScanMerger",
                name="laser_scan_merger_node",
                parameters=[PathJoinSubstitution([
                    get_package_share_directory('laser_scan_merger'),
                    'config',
                    LaunchConfiguration('robotname'),
                    'param.yaml'
                ])]
            )
        ],
        output="screen"
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(container)

    return ld
