from whill_navi2.ros2_launch_utils import *

def generate_launch_description():

    data_path = DataPath()
    rviz_path = get_rviz_path("whill_navi2", "waypoint_editor.rviz")
    read_path, write_path = data_path.get_rewapypoint_path()
    
##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################


    # Node
    waypoint_editor_rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="waypoint_editor_rviz2",
        arguments=["-d", rviz_path],
        output="screen"
    )
    waypoint_editor_node = Node(
        package="waypoint_pkg",
        executable="waypoint_editor",
        name="waypoint_editor",
        parameters=[{
            "read_file_name": read_path,
            "write_file_name": write_path,
            "save_service_name": "save_service",
            "update_service_name": "update_service",
            "debug": False    
        }],
        output="screen"
    ) 
    # Lifecycle Node
    map_server_lifecycle_node = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace="",
        parameters=[{
            "yaml_filename": os.path.join(
                data_path.remap_dir,
                data_path.remap_name + '.yaml'
            )
        }],
        output="screen"
    )
    
    # When rviz is launched, change map_server to configured
    when_rviz_launched = RegisterEventHandler(
        OnProcessStart(
            target_action=waypoint_editor_rviz2_node,
            on_start=[
                EmitEvent(
                    event=lifecycle.ChangeState(
                        lifecycle_node_matcher=matches_action(map_server_lifecycle_node),
                        transition_id=Transition.TRANSITION_CONFIGURE
                    )
                ),
                TimerAction(
                    actions=[waypoint_editor_node],
                    period=0.1
                )
            ]
        )
    )
    # When map_server is configured, activate ifself
    when_mapserver_configured = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server_lifecycle_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=lifecycle.ChangeState(
                        lifecycle_node_matcher=matches_action(map_server_lifecycle_node),
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                )
            ]
        )
    )
    # When rviz exit, shutdown RosLaunch
    when_rviz_over = RegisterEventHandler(
        OnExecutionComplete(
            target_action=waypoint_editor_rviz2_node,
            on_completion=[
                Shutdown()
            ]
        )
    )
    action_group = GroupAction(
        actions=[
            when_rviz_launched,
            when_mapserver_configured,
            when_rviz_over
        ]
    )
    
    return LaunchDescription([        
        waypoint_editor_rviz2_node,
        map_server_lifecycle_node,
        action_group
    ])
