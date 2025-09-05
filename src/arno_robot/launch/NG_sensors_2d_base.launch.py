# sensors_2d_base.launch.py  — composable merger version

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # -------- Arguments you may override --------
    base_frame   = DeclareLaunchArgument('base_frame',   default_value='base_link')
    front_frame  = DeclareLaunchArgument('front_frame',  default_value='laser_frame_1')
    rear_frame   = DeclareLaunchArgument('rear_frame',   default_value='laser_frame_2')

    front_scan   = DeclareLaunchArgument('front_scan',   default_value='/scan_front')
    rear_scan    = DeclareLaunchArgument('rear_scan',    default_value='/scan_rear')
    merged_scan  = DeclareLaunchArgument('merged_scan',  default_value='/scan')

    # Your tested Ethernet YAMLs (inside arno_robot/config)
    front_params = DeclareLaunchArgument('front_params', default_value='params_urg_node2_ether1.yaml')
    rear_params  = DeclareLaunchArgument('rear_params',  default_value='params_urg_node2_ether2.yaml')

    # If your merger’s type name differs, override at launch with: merger_plugin:=<Type>
    merger_plugin = DeclareLaunchArgument(
        'merger_plugin',
        default_value='util::LaserScanMerger'
    )

    # -------- Includes / paths --------
    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('arno_robot'), 'launch', 'static_transforms.launch.py'])
        )
    )

    cfg_dir = PathJoinSubstitution([FindPackageShare('arno_robot'), 'config'])
    front_params_file = PathJoinSubstitution([cfg_dir, LaunchConfiguration('front_params')])
    rear_params_file  = PathJoinSubstitution([cfg_dir, LaunchConfiguration('rear_params')])

    # -------- Two urg_node2 drivers --------
    urg_front = Node(
        package='urg_node2', executable='urg_node2_node', name='front_lidar', output='screen',
        parameters=[front_params_file, {'frame_id': LaunchConfiguration('front_frame')}],
        remappings=[('scan', LaunchConfiguration('front_scan'))],
    )

    urg_rear = Node(
        package='urg_node2', executable='urg_node2_node', name='rear_lidar', output='screen',
        parameters=[rear_params_file, {'frame_id': LaunchConfiguration('rear_frame')}],
        remappings=[('scan', LaunchConfiguration('rear_scan'))],
    )

    # -------- Composable laser_scan_merger --------
    # Covers common forks:
    #  - If the component expects 'scan_topics', we provide a list param.
    #  - If it expects 'scan_in_1'/'scan_in_2', remaps below handle it.
    merger_node = ComposableNode(
        package='laser_scan_merger',
        plugin=LaunchConfiguration('merger_plugin'),
        name='scan_merger',
        parameters=[{
            'destination_frame': LaunchConfiguration('base_frame'),  # some forks use 'target_frame'
            'target_frame':      LaunchConfiguration('base_frame'),
            'scan_topics': [
                LaunchConfiguration('front_scan'),
                LaunchConfiguration('rear_scan'),
            ],
        }],
        remappings=[
            ('scan_in_1', LaunchConfiguration('front_scan')),
            ('scan_in_2', LaunchConfiguration('rear_scan')),
            ('scan',      LaunchConfiguration('merged_scan')),  # merged output
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    merger_container = ComposableNodeContainer(
        name='scan_merger_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # multithreaded container
        composable_node_descriptions=[merger_node],
        output='screen',
    )

    return LaunchDescription([
        # args
        base_frame, front_frame, rear_frame,
        front_scan, rear_scan, merged_scan,
        front_params, rear_params, merger_plugin,
        # static TFs
        static_tf_launch,
        # drivers
        urg_front, urg_rear,
        # merger
        merger_container,
    ])
