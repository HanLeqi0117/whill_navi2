import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, 
                            LogInfo, ExecuteProcess, RegisterEventHandler, TimerAction,
                            EmitEvent, Shutdown)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.event_handlers import OnProcessStart, OnExecutionComplete, OnShutdown, OnProcessExit
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events import lifecycle
from lifecycle_msgs.msg import Transition, TransitionEvent
from launch_ros.event_handlers import OnStateTransition

def generate_launch_description():
    
    # Full Data Path
    with open(os.path.join(
        get_package_share_directory('whill_navi2'),
        'config', 'launch_arg', 'full_data_path_launch_arg.yaml'
    )) as f:
        launcharg_full_data_path = yaml.safe_load(f)['full_data_path_launch']
        
    
    # Node
    slam_offline_rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='slam_offline_rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('whill_navi2'),
            'config', 'rviz2', 'slam_offline_launch.rviz'
        )]
    )
    
    # ExecutePross
    slam_offline_process = Node(
        parameters=[
            os.path.join(
                get_package_share_directory("whill_navi2"),
                'config', 'params', 'slam_offline_node_params.yaml'
            )
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='sync_slam_toolbox_node',
        output='screen'
    )
    ros2bag_play_process = ExecuteProcess(
        cmd=[
            FindExecutable(name='ros2'),
            'bag',
            'play',
            os.path.join(
                launcharg_full_data_path["bag_path_abs"]
            )
        ],
        shell=True
    )
    map_saver_cli_lifecycle_node = LifecycleNode(
        package="nav2_map_server",
        executable="map_saver_cli",
        name="map_saver_cli",
        namespace="slam_offline_launch",
        arguments=["-f", os.path.join(
            launcharg_full_data_path["map_path_abs"],
            launcharg_full_data_path["map_name"]
        )]
    )
    mv_map_to_remap_process = ExecuteProcess(
        cmd=[
            FindExecutable(name='mv'),
            launcharg_full_data_path['map_path_abs'] + '/*',
            launcharg_full_data_path['remap_path_abs']
        ]
    )
    
    # Event
    map_saver_cli_lifecycle_node_configure_event = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_offline_rviz2_node,
            on_start=[
                slam_offline_process,
                ros2bag_play_process,
                EmitEvent(
                    event=lifecycle.ChangeState(
                        lifecycle_node_matcher=matches_action(map_saver_cli_lifecycle_node),
                        transition_id=Transition.TRANSITION_CONFIGURE
                    )
                )
            ]
        )
    )
    map_saver_cli_lifecycle_node_activate_event = RegisterEventHandler(
        OnExecutionComplete(
            target_action=ros2bag_play_process,
            on_completion=[
                LogInfo(msg="Bag Play is over, and going to save Map."),
                EmitEvent(
                    event=lifecycle.ChangeState(
                        lifecycle_node_matcher=matches_action(map_saver_cli_lifecycle_node),
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                ),
                TimerAction(
                    actions=[Shutdown()],
                    period=1.0
                )
            ]
        )
    )
    mv_map_to_remap_event = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                mv_map_to_remap_process
            ]
        )
    )


# test
# if __name__ == '__main__':
#     generate_launch_description()