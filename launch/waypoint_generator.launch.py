import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, EmitEvent, TimerAction
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.events import lifecycle
from lifecycle_msgs.msg import Transition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition


def generate_launch_description():
    
   # Path
    with open(os.path.join(
        get_package_share_directory('whill_navi2'),
        'config', 'launch_arg', 'full_data_path_launch_arg.yaml'
    )) as f:
        launcharg_full_data_path = yaml.safe_load(f)['full_data_path_launch']   
    
    # Node
    waypoint_maker_node = Node(
        package="waypoint_pkg",
        executable="waypoint_maker",
        parameters=[
            os.path.join(
                get_package_share_directory('whill_navi2'),
                'conifg', 'params', 'waypoint_maker_params.yaml'
            ),
            {'way_txt_': os.path.join(
                launcharg_full_data_path['waypoint_path_abs'],
                launcharg_full_data_path['waypoint_name'])}
        ],
        output="screen"
    )
    ros2bag_play_process = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            'bag', 'play', os.path.join(
                launcharg_full_data_path['bag_path_abs'],
                launcharg_full_data_path['bag_name']
            )
        ]
    )
    waypoint_maker_rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='waypoint_maker_rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('whill_navi2'),
            'config', 'rviz2', 'waypoint_maker_launch.rviz'
        )]
    )
    
    # Lifecycle Node
    amcl_lifecycle_node = LifecycleNode(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        namespace='waypoint_generator_launch',
        parameters=[os.path.join(
            get_package_share_directory("whill_navi2"),
            'config', 'params', 'amcl_params.yaml'
        )],
    )
    map_server_lifecycle_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='waypoint_generator_launch',        
        parameters=[os.path.join(
            launcharg_full_data_path['remap_path_abs'],
            launcharg_full_data_path['remap_name']
        )],
        output='screen'
    )    
        
    # Lifecycle Node Configure
    lifecycle_node_configure_event = RegisterEventHandler(
        OnProcessStart(
            target_action=waypoint_maker_rviz2_node,
            on_start=[
                EmitEvent(
                    event=lifecycle.ChangeState(
                        lifecycle_node_matcher=matches_action(amcl_lifecycle_node),
                        transition_id=Transition.TRANSITION_CONFIGURE
                    )
                ),
                EmitEvent(
                    event=lifecycle.ChangeState(
                        lifecycle_node_matcher=matches_action(map_server_lifecycle_node),
                        transition_id=Transition.TRANSITION_CONFIGURE
                    )
                )
                
            ]
        )
    )
    # Lifecycle Node Activate
    lifecycle_node_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server_lifecycle_node,
            start_state='UnConfigured', goal_state='Inactive',
            entities=[
                EmitEvent(
                    event=lifecycle.ChangeState(
                        lifecycle_node_matcher=matches_action(map_server_lifecycle_node),
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                ),
                EmitEvent(
                    event=lifecycle.ChangeState(
                        lifecycle_node_matcher=matches_action(amcl_lifecycle_node),
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                ),
                TimerAction(
                    period=1.0,
                    actions=[ros2bag_play_process]
                ),
                waypoint_maker_node
            ]
        )
    )  
            
    name_dict = locals()
    value_list = []
    for name, value in name_dict.items():
        if ("_arg") in name \
        or ("_node") in name \
        or ("_launch") in name \
        or ("_event") in name:
            # test
            # print(name, type(value))
            value_list.append(value)
            
    return LaunchDescription(
        value_list
    )

