#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState, matches_node_name
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    param_file = PathJoinSubstitution([
        FindPackageShare('arno_robot'),
        'config',
        'params_default.yaml'
    ])

    # Two lifecycle nodes (Humble requires 'namespace' kwarg)
    urg1 = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name='lidar1',
        namespace='',
        output='screen',
        parameters=[param_file],
        remappings=[('scan', 'scan_1st')],
    )
    urg2 = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name='lidar2',
        namespace='',
        output='screen',
        parameters=[param_file],
        remappings=[('scan', 'scan_2nd')],
    )

    # Configure both shortly after start (use node-name matcher on Humble)
    configure_both = TimerAction(
        period=0.2,
        actions=[
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_node_name('/lidar1'),
                transition_id=Transition.TRANSITION_CONFIGURE)),
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_node_name('/lidar2'),
                transition_id=Transition.TRANSITION_CONFIGURE)),
        ],
    )

    # When each reaches 'inactive', activate it
    activate_urg1 = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=urg1,
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_node_name('/lidar1'),
                    transition_id=Transition.TRANSITION_ACTIVATE)),
            ],
        )
    )
    activate_urg2 = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=urg2,
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_node_name('/lidar2'),
                    transition_id=Transition.TRANSITION_ACTIVATE)),
            ],
        )
    )

    return LaunchDescription([
        urg1,
        urg2,
        configure_both,
        activate_urg1,
        activate_urg2,
    ])
